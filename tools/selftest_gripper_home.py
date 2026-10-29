#!/usr/bin/env python3
"""自测：找零点那段时序，用一块假板子跑（不碰硬件、不碰 ROS）。

钉住的是这几件事：

1. 两头都推得到 → 认出合拢点，行程和机械行程对上；
2. 两指之间卡着东西 → 行程明显短 → 拒绝（这是只看合拢点抓不到的情形）；
3. 回读冻住（板子标定表坏了）→ 两头读数拉不开 → 拒绝；
4. 读不到位置 → 失败，且理由说"读不到"；
5. 一直动不停 → 超时失败，不许硬编一个数；
6. 上电时手指就贴着合拢止位 → 也认；
7. 边界：行程偏 0.3 圈收、偏 0.5 圈拒；
8. 节点与命令行工具调的是**同一段** `home()`，不许各写一份。
"""
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
sys.path.insert(0, GRASP)

from dummy_vc_grasp import gripper_frames as gf   # noqa: E402
from dummy_vc_grasp import gripper_gate as gg     # noqa: E402
from dummy_vc_grasp import gripper_home as gh     # noqa: E402

FAILED = []


def check(name, condition, detail=''):
    if condition:
        print('  [ok] {}'.format(name))
    else:
        print('  [FAIL] {}{}'.format(name, '  ' + detail if detail else ''))
        FAILED.append(name)


class Clock(object):
    """假时间：`sleep` 只是把表往前拨。"""

    def __init__(self, step=0.05):
        self.t = 0.0
        self.step = step

    def time(self):
        return self.t

    def tick(self, _seconds):
        self.t += self.step


class FakeBoard(object):
    """一块只会"朝目标走、两头都有机械止位"的板子。

    `closed_stop` 是手指合拢时停在的圈数，`open_stop` 是张开到底时停在的圈数；
    两者之差就是机械行程。两指之间夹着东西时，`closed_stop` 会提前（方块占掉的
    那点行程），行程随之变短 —— 这正是要能识别出来的。
    """

    def __init__(self, closed_stop, open_stop, start=None, speed=2.0,
                 readable=True, frozen=False):
        self.closed_stop = float(closed_stop)
        self.open_stop = float(open_stop)
        self.rev = float(start if start is not None else open_stop)
        self.speed = float(speed)
        self.readable = readable
        self.frozen = frozen
        self.target = None
        self.enabled = False
        self.frames = []

    def send(self, frame_id, payload):
        self.frames.append(frame_id)
        if frame_id == gf.ENABLE_FRAME_ID:
            self.enabled = struct.unpack('<I', payload[:4])[0] != 0
        elif frame_id == gf.POSITION_FRAME_ID:
            self.target = struct.unpack('<f', payload[:4])[0]

    def read(self, dt=0.05):
        if not self.readable:
            return None
        if self.frozen:
            return 0.0602                     # 标定表作废时读到的那个常数
        if self.target is not None and self.enabled:
            step = self.speed * dt
            if self.target > self.rev:
                self.rev = min(self.rev + step, self.target, self.closed_stop)
            else:
                self.rev = max(self.rev - step, self.target, self.open_stop)
        return self.rev


def run(board, clock, **kwargs):
    return gh.home(board.send, board.read, sleep=clock.tick,
                   now=clock.time, **kwargs)


def standard_board(**kwargs):
    """出厂标定：合拢 2.951 圈、张开 −1.782 圈，行程 4.733 圈。"""
    return FakeBoard(closed_stop=gf.DEFAULT_REV_CLOSED,
                     open_stop=gf.DEFAULT_REV_CLOSED - gh.DEFAULT_STROKE_REVS,
                     **kwargs)


def case_both_stops():
    clock = Clock()
    board = standard_board(start=0.5)
    result = run(board, clock)
    check('两头都对上 → 成功', result.ok, repr(result.reason))
    check('合拢点就是止位',
          abs((result.rev_closed or -99) - gf.DEFAULT_REV_CLOSED) < 0.01,
          repr(result.rev_closed))
    check('行程量到 ≈4.73 圈',
          abs((result.span_revs or 0) - gh.DEFAULT_STROKE_REVS) < 0.01,
          repr(result.span_revs))
    check('两头都推过', board.frames.count(gf.POSITION_FRAME_ID) >= 2)
    check('最后没在顶着（发了停）', board.enabled is False)


def case_object_in_the_way():
    """两指之间夹着 24 mm 方块：合拢止位提前 2 圈，行程短掉 —— 必须拒绝。"""
    clock = Clock()
    board = FakeBoard(closed_stop=gf.DEFAULT_REV_CLOSED - 2.0,
                      open_stop=gf.DEFAULT_REV_CLOSED - gh.DEFAULT_STROKE_REVS,
                      start=-1.0)
    result = run(board, clock)
    check('卡着东西就拒绝', not result.ok, repr(result.reason))
    check('理由点出"卡着东西"', '卡着东西' in result.reason, result.reason)
    check('没把方块位置当成零点', result.rev_closed is None)


def case_frozen_readback():
    """标定表作废：位置恒为 0.0602 圈，两头都"不动" → 拒绝。"""
    clock = Clock()
    board = standard_board(start=0.0, frozen=True)
    result = run(board, clock, still_s=0.2)
    check('回读冻住就拒绝', not result.ok, repr(result.reason))
    check('理由点出两个方向都没动',
          '两个方向都没动' in result.reason, result.reason)


def case_no_readback():
    clock = Clock()
    board = standard_board(start=0.0, readable=False)
    result = run(board, clock, timeout_s=1.0)
    check('读不到就失败', not result.ok)
    check('理由说"读不到"', '读不到' in result.reason, result.reason)


def case_never_stops():
    """板子一直走、永远不停：把两头都设成很远的地方，2 秒内到不了也停不下来。"""
    clock = Clock()
    board = FakeBoard(closed_stop=90.0, open_stop=-90.0, start=0.0, speed=0.05)
    result = run(board, clock, timeout_s=2.0)
    check('不停就超时失败', not result.ok, repr(result.reason))
    check('理由说"没停"', '没停' in result.reason, result.reason)


def case_already_closed():
    clock = Clock()
    board = standard_board(start=gf.DEFAULT_REV_CLOSED)
    result = run(board, clock)
    check('上电就贴着止位也认', result.ok, repr(result.reason))
    check('零点没跑偏',
          abs((result.rev_closed or -99) - gf.DEFAULT_REV_CLOSED) < 1e-6,
          repr(result.rev_closed))


def case_stroke_tolerance():
    for short, expect in ((0.3, True), (0.5, False)):
        clock = Clock()
        board = FakeBoard(
            closed_stop=gf.DEFAULT_REV_CLOSED,
            open_stop=gf.DEFAULT_REV_CLOSED - (gh.DEFAULT_STROKE_REVS - short),
            start=0.0)
        result = run(board, clock)
        check('行程短 {:.1f} 圈 → {}'.format(short, '收' if expect else '拒'),
              result.ok is expect, repr(result.reason))


def case_stall_detector():
    check('三个点几乎相同 → 停住', gg.stall_detected([1.0, 1.0, 1.0001]))
    check('还在匀速走 → 没停', not gg.stall_detected([1.0, 1.1, 1.2]))
    check('点数不够不下结论', not gg.stall_detected([1.0]))
    check('早期的点不算进来', not gg.stall_detected([0.0, 0.5, 1.0, 1.5]))


def case_shared_implementation():
    node_path = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_can_node.py')
    tool_path = os.path.join(HERE, 'gripper_home.py')
    with open(node_path, encoding='utf-8') as handle:
        node_src = handle.read()
    with open(tool_path, encoding='utf-8') as handle:
        tool_src = handle.read()
    check('节点里调 gh.home', 'gh.home(' in node_src)
    check('工具里也调 gh.home', 'gh.home(' in tool_src)
    check('节点没自己重写一份找零点', node_src.count('encode_position(target') == 0)
    check('没有残留的旧参数 home_plausible_revs',
          'home_plausible_revs' not in node_src)


def main():
    print('seeking both mechanical stops on a fake board')
    case_both_stops()
    case_object_in_the_way()
    case_frozen_readback()
    case_no_readback()
    case_never_stops()
    case_already_closed()
    case_stroke_tolerance()
    case_stall_detector()
    case_shared_implementation()

    if FAILED:
        print('\n{} failure(s): {}'.format(len(FAILED), ', '.join(FAILED)))
        return 1
    print('\nRESULT: the closed stop is located and cross-checked against the '
          'mechanical stroke before it is trusted')
    return 0


if __name__ == '__main__':
    sys.exit(main())
