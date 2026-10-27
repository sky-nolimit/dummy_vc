#!/usr/bin/env python3
"""自测：找零点那段时序，用一块假板子跑（不碰硬件、不碰 ROS）。

钉住的是这几件事：

1. 合拢点比目标早到 → 认出止位，把那一刻的圈数当零点；
2. 已经贴着止位上电 → 一开始就不动，也认；
3. 一直动不停 → 超时失败，不许硬编一个数出来；
4. 读不到位置 → 失败，且理由说的是"读不到"（不是"没停"）；
5. 量出来的合拢点离出厂值太远 → 拒绝（两指之间卡了东西，或者零点挪了）；
6. `tools/gripper_home.py` 与节点用的是**同一段** `home()`，不是两份实现。
"""
import os
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
    """一块只会"朝目标走、走到机械止位就停"的板子。"""

    def __init__(self, start, stop, speed=0.5, readable=True):
        self.rev = float(start)
        self.stop = float(stop)
        self.speed = float(speed)          # 圈/秒
        self.readable = readable
        self.target = None
        self.enabled = False
        self.frames = []

    def send(self, frame_id, payload):
        import struct
        self.frames.append(frame_id)
        if frame_id == gf.ENABLE_FRAME_ID:
            self.enabled = struct.unpack('<I', payload[:4])[0] != 0
        elif frame_id == gf.POSITION_FRAME_ID:
            self.target = struct.unpack('<f', payload[:4])[0]

    def read(self, dt=0.05):
        if not self.readable:
            return None
        if self.target is not None and self.enabled:
            step = self.speed * dt
            if self.target > self.rev:
                self.rev = min(self.rev + step, self.target, self.stop)
            else:
                self.rev = max(self.rev - step, self.target, self.stop)
        return self.rev


def run(board, clock, **kwargs):
    return gh.home(board.send, board.read, sleep=clock.tick,
                   now=clock.time, **kwargs)


def case_stops_before_target():
    """正常情况：从 0.5 圈出发，合拢止位在 2.951 圈。"""
    clock = Clock()
    board = FakeBoard(start=0.5, stop=gf.DEFAULT_REV_CLOSED)
    result = run(board, clock)
    check('认识止位并返回 success', result.ok, repr(result.reason))
    check('零点就是止位那一圈',
          abs((result.rev_closed or -99) - gf.DEFAULT_REV_CLOSED) < 0.01,
          repr(result.rev_closed))
    check('发过使能帧', gf.ENABLE_FRAME_ID in board.frames)
    check('推过位置帧', gf.POSITION_FRAME_ID in board.frames)
    check('最后发了停（不然会一直顶着）',
          board.enabled is False)
    return result


def case_already_at_stop():
    """上电时手指就贴着止位：一开始就不动，也要认。"""
    clock = Clock()
    board = FakeBoard(start=gf.DEFAULT_REV_CLOSED, stop=gf.DEFAULT_REV_CLOSED)
    result = run(board, clock)
    check('贴止位上电也认', result.ok, repr(result.reason))
    check('零点没跑偏',
          abs((result.rev_closed or -99) - gf.DEFAULT_REV_CLOSED) < 1e-6,
          repr(result.rev_closed))


def case_never_stops():
    """板子一直往目标走、又永远不动停 → 超时失败。"""
    clock = Clock()
    board = FakeBoard(start=0.0, stop=90.0, speed=0.05)
    result = run(board, clock, timeout_s=2.0)
    check('不停就超时失败', not result.ok)
    check('说的是"没停"而不是"读不到"', '没停' in result.reason, result.reason)


def case_no_readback():
    """读不到位置：失败，理由必须点出来。"""
    clock = Clock()
    board = FakeBoard(start=0.0, stop=1.0, readable=False)
    result = run(board, clock, timeout_s=1.0)
    check('读不到就失败', not result.ok)
    check('理由说"读不到"', '读不到' in result.reason, result.reason)


def case_implausible_stop():
    """两指之间卡了东西：合拢点比出厂值早 2 圈 → 拒绝，不许拿它当零点。"""
    clock = Clock()
    board = FakeBoard(start=0.4, stop=gf.DEFAULT_REV_CLOSED - 2.0)
    result = run(board, clock)
    check('离出厂值太远就拒绝', not result.ok, repr(result.reason))
    check('理由讲明白了"卡着东西"', '卡着东西' in result.reason, result.reason)
    check('没把那个数当成零点', result.rev_closed is None)


def case_tolerance():
    """边界：偏 0.9 圈要收，偏 1.1 圈要拒。"""
    for delta, expect in ((0.9, True), (1.1, False)):
        clock = Clock()
        board = FakeBoard(start=0.5, stop=gf.DEFAULT_REV_CLOSED + delta)
        result = run(board, clock)
        check('偏 {:+.1f} 圈 → {}'.format(delta, '收' if expect else '拒'),
              result.ok is expect, repr(result.reason))


def case_stall_detector():
    """`stall_detected` 本身：窗口只看最近几个点。"""
    check('三个点几乎相同 → 停住', gg.stall_detected([1.0, 1.0, 1.0001]))
    check('还在匀速走 → 没停',
          not gg.stall_detected([1.0, 1.1, 1.2]))
    check('点数不够不下结论', not gg.stall_detected([1.0]))
    check('早期的点不算进来',
          not gg.stall_detected([0.0, 0.5, 1.0, 1.5]))


def case_shared_implementation():
    """节点和工具必须调同一个 home()，不许各写一份。"""
    node_path = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_can_node.py')
    tool_path = os.path.join(HERE, 'gripper_home.py')
    with open(node_path, encoding='utf-8') as handle:
        node_src = handle.read()
    with open(tool_path, encoding='utf-8') as handle:
        tool_src = handle.read()
    check('节点里调 gh.home', 'gh.home(' in node_src)
    check('工具里也调 gh.home', 'gh.home(' in tool_src)
    check('节点没自己重写一份找零点',
          node_src.count('encode_position(target') == 0)


def main():
    print('seeking the closed stop on a fake board')
    case_stops_before_target()
    case_already_at_stop()
    case_never_stops()
    case_no_readback()
    case_implausible_stop()
    case_tolerance()
    case_stall_detector()
    case_shared_implementation()

    if FAILED:
        print('\n{} failure(s): {}'.format(len(FAILED), ', '.join(FAILED)))
        return 1
    print('\nRESULT: the homing sequence resolves the closed stop without '
          'trusting where the fingers happened to be')
    return 0


if __name__ == '__main__':
    sys.exit(main())
