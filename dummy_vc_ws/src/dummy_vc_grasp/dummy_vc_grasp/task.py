"""抓取任务的状态机 —— 纯逻辑，不 import ROS，可以离线跑通所有分支。

为什么把状态机单独拎出来：真正的坑几乎都在"某一步失败了怎么办"。真机上每次
失败都要重开一次相机、重摆一次方块，一天试不了几次；而迁移逻辑本身是纯函数，
可以用几十行自测把每条边都走一遍（`tools/selftest_grasp_task.py`）。ROS 那边
只负责"执行动作、把结果翻译成事件"。

一次抓取的顺序（和 docs/01 阶段 5 一致）：

    IDLE ─START→ OBSERVE ─看到目标→ MOVE_PRE_GRASP → APPROACH → CLOSE_GRIPPER
         → LIFT → MOVE_PLACE → OPEN_GRIPPER → RETREAT → DONE

任何一步失败：能重试的重试（默认 2 次），不能重试的进 FAULT 并说明原因。
`CLOSE_GRIPPER` / `OPEN_GRIPPER` 失败**不重试** —— 夹爪那条命令失败过就说明
不确定指头在哪，此时"再试一次"可能把方块挤飞或把手指别住。
"""

import enum
from dataclasses import dataclass


class State(enum.Enum):
    IDLE = 'IDLE'
    OBSERVE = 'OBSERVE'
    MOVE_PRE_GRASP = 'MOVE_PRE_GRASP'
    APPROACH = 'APPROACH'
    CLOSE_GRIPPER = 'CLOSE_GRIPPER'
    LIFT = 'LIFT'
    MOVE_PLACE = 'MOVE_PLACE'
    OPEN_GRIPPER = 'OPEN_GRIPPER'
    RETREAT = 'RETREAT'
    DONE = 'DONE'
    FAULT = 'FAULT'


class Event(enum.Enum):
    START = 'START'          # 开始一次抓取（或从 FAULT/DONE 重来）
    TARGET = 'TARGET'        # OBSERVE 期间拿到一个可抓的目标
    SUCCEEDED = 'SUCCEEDED'  # 当前动作完成
    FAILED = 'FAILED'        # 当前动作失败（规划失败、执行失败都算）
    TIMEOUT = 'TIMEOUT'      # 当前动作超时
    ABORT = 'ABORT'          # 外部叫停（Ctrl-C、Estop、参数关掉任务）


#: 可以被"重试"的状态 —— 都是纯运动的步骤。
RETRYABLE = frozenset({State.MOVE_PRE_GRASP, State.APPROACH, State.LIFT,
                       State.MOVE_PLACE, State.RETREAT})

#: 每个状态"该做什么"，动作名交给 ROS 那层解释。
ACTIONS = {
    State.IDLE: 'none',
    State.OBSERVE: 'wait_target',
    State.MOVE_PRE_GRASP: 'move_pre_grasp',
    State.APPROACH: 'descend',
    State.CLOSE_GRIPPER: 'close_gripper',
    State.LIFT: 'lift',
    State.MOVE_PLACE: 'move_place',
    State.OPEN_GRIPPER: 'open_gripper',
    State.RETREAT: 'retreat',
    State.DONE: 'none',
    State.FAULT: 'stop',
}

#: 正常前进时的下一站。
NEXT = {
    State.MOVE_PRE_GRASP: State.APPROACH,
    State.APPROACH: State.CLOSE_GRIPPER,
    State.CLOSE_GRIPPER: State.LIFT,
    State.LIFT: State.MOVE_PLACE,
    State.MOVE_PLACE: State.OPEN_GRIPPER,
    State.OPEN_GRIPPER: State.RETREAT,
    State.RETREAT: State.DONE,
}

#: 这些状态的失败不重试，直接认输并说明为什么。
NO_RETRY = {
    State.CLOSE_GRIPPER: '夹爪合拢失败：指头位置未知，不重试（可能把方块挤飞或别住手指）',
    State.OPEN_GRIPPER: '夹爪张开失败：方块还夹在指间，退回起点比撒手更糟',
}


@dataclass(frozen=True)
class Decision:
    """一次迁移的结果。`action` 是给 ROS 那层的指令。"""
    state: State
    action: str
    note: str
    retries: int = 0

    def describe(self):
        return '{} -> {}（{}），重试 {}{}'.format(
            self.action, self.state.value, self.note, self.retries,
            '' if not self.note else '')


class TaskMachine:
    """带重试计数的状态机。调用方只负责把事件喂进 `decide()`。

    `gripper_ready=False` 表示夹爪的"角度<->开口"还没实测：这时**执行动作照走**
    （起手位、下探都能验），但走到 CLOSE_GRIPPER 会直接进 FAULT 并说明原因。
    这样在夹爪没标定之前也能真机验证前面四步，而不是整条链一起干等。
    """

    def __init__(self, max_retries=2, gripper_ready=False):
        if max_retries < 0:
            raise ValueError('max_retries 不能为负：{}'.format(max_retries))
        self.max_retries = int(max_retries)
        self.gripper_ready = bool(gripper_ready)
        self.state = State.IDLE
        self.retries = 0

    # -- 判定 ---------------------------------------------------------------
    def decide(self, event: Event) -> Decision:
        return self._apply(self.state, event)

    def _fault(self, note):
        self.state = State.FAULT
        self.retries = 0
        return Decision(State.FAULT, ACTIONS[State.FAULT], note)

    def _retry_or_fault(self, note):
        if self.state in NO_RETRY:
            return self._fault('{}（{}）'.format(NO_RETRY[self.state], note))
        if self.state not in RETRYABLE:
            return self._fault(note)
        if self.retries >= self.max_retries:
            return self._fault('重试 {} 次仍失败：{}'.format(self.retries, note))
        self.retries += 1
        return Decision(self.state, ACTIONS[self.state],
                        '{}，重试第 {} 次'.format(note, self.retries), self.retries)

    def _enter(self, state, note):
        if state is State.CLOSE_GRIPPER and not self.gripper_ready:
            return self._fault(
                '夹爪的"角度<->开口"映射还没实测，不发 hand 命令（docs/02 阶段 0.5 量完'
                '再填成 measured）；前四步能验，合爪不能')
        self.state = state
        self.retries = 0 if state in (State.OBSERVE, State.DONE) else self.retries
        return Decision(state, ACTIONS[state], note, self.retries)

    def _apply(self, state, event):
        if event is Event.ABORT:
            return self._fault('被外部叫停')
        if event is Event.START:
            if state in (State.IDLE, State.DONE, State.FAULT):
                self.state = State.IDLE
                self.retries = 0
                return self._enter(State.OBSERVE, '开始一次抓取，等目标')
            return Decision(state, ACTIONS[state],
                            '已经在流程里（{}），忽略 START'.format(state.value),
                            self.retries)

        if state is State.OBSERVE:
            if event is Event.TARGET:
                return self._enter(State.MOVE_PRE_GRASP, '拿到目标，去起手位')
            if event is Event.FAILED:
                return Decision(state, ACTIONS[state], '这一帧没有可抓的目标，继续等',
                                self.retries)
            if event is Event.TIMEOUT:
                return self._fault('等目标超时')
            return self._fault('OBSERVE 期间收到意外事件 {}'.format(event.value))

        if state in (State.IDLE, State.DONE):
            if event in (Event.SUCCEEDED, Event.FAILED, Event.TIMEOUT):
                return Decision(state, ACTIONS[state], '空闲中，没有待办', self.retries)
            return self._fault('{} 期间收到意外事件 {}'.format(state.value, event.value))

        if state is State.FAULT:
            return Decision(state, ACTIONS[state],
                            '故障中，发 START 才能重来', self.retries)

        # 以下都是"正在做某个动作"的状态
        if event is Event.SUCCEEDED:
            nxt = NEXT.get(state)
            if nxt is None:
                return self._fault('{} 没有下一站'.format(state.value))
            return self._enter(nxt, '{} 完成'.format(state.value))

        if event in (Event.FAILED, Event.TIMEOUT):
            why = ('{} 失败'.format(state.value) if event is Event.FAILED
                   else '{} 超时'.format(state.value))
            return self._retry_or_fault(why)

        return self._fault('{} 期间收到意外事件 {}'.format(state.value, event.value))
