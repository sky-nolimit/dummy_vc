#!/usr/bin/env python3
"""`gripper_gate.py` 的自测：闭到多宽、"夹住没有"怎么判。

这一层是纯函数，所以判定表可以钉死：位置差、超时、没有回读，各种组合都要
给出确定的答案，不能"看情况"。另外钉一条源码级守卫：`pick_place_node` 不许
再回头去调驱动里的 `hand.*` / `command_for_width`（那条路在真机上会把夹爪
拖去跑标定）。
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
if GRASP not in sys.path:
    sys.path.insert(0, GRASP)

from dummy_vc_grasp import gripper_gate as gg   # noqa: E402

FAILED = []


def ok(label):
    print('  [ok  ] ' + label)


def fail(label, detail=''):
    FAILED.append(label)
    print('  [FAIL] {}{}'.format(label, '：' + detail if detail else ''))


def same(label, got, want):
    if got == want:
        ok('{}（{!r}）'.format(label, got))
    else:
        fail(label, '拿到 {!r}，应该是 {!r}'.format(got, want))


def close(label, got, want, tol=1e-9):
    if abs(got - want) <= tol:
        ok('{}（{:.6f}）'.format(label, got))
    else:
        fail(label, '拿到 {:.6f}，应该是 {:.6f}'.format(got, want))


def expect_error(label, func, *args):
    try:
        func(*args)
    except ValueError:
        ok(label)
        return
    fail(label, '没有拒绝')


def check_close_width():
    print('\n1. 闭到多宽：比方块窄 clearance，但不许窄到夹手指自己')
    close('24 mm 方块 → 22 mm', gg.close_width(0.024), 0.022)
    close('26 mm 方块 → 24 mm', gg.close_width(0.026), 0.024)
    close('clearance 可改', gg.close_width(0.024, clearance_m=0.004), 0.020)
    close('下限之上的窄方块也放行', gg.close_width(0.010), 0.008)
    expect_error('4 mm 方块会夹到手指自己 → 拒绝', gg.close_width, 0.004)
    expect_error('0 宽 → 拒绝', gg.close_width, 0.0)
    expect_error('负数 → 拒绝', gg.close_width, -0.024)
    expect_error('NaN → 拒绝', gg.close_width, float('nan'))
    expect_error('inf → 拒绝', gg.close_width, float('inf'))


def check_verdict():
    print('\n2. 夹住没有：位置差 + 超时，两条都要对上')
    target = 0.022
    # 还在动，什么都别判
    same('没有回读、还没超时 → 继续等', gg.verdict(target, None), None)
    same('回读到目标、还没超时 → 继续等', gg.verdict(target, 0.0221), None)
    # 被方块挡住
    same('实测 24.2 mm（方块挡着）→ 夹住',
         gg.verdict(target, 0.0242), gg.HELD)
    same('差一点点就够 → 仍按夹住算（0.9 mm > 0.8 mm）',
         gg.verdict(target, 0.0229), gg.HELD)
    # 到目标了
    same('实测正好目标、没超时 → 继续等', gg.verdict(target, 0.0220), None)
    same('实测正好目标、超时 → 空夹',
         gg.verdict(target, 0.0220, timed_out=True), gg.EMPTY)
    same('手指合过头（比目标还窄）→ 空夹',
         gg.verdict(target, 0.0190, timed_out=True), gg.EMPTY)
    same('边界：实测 = 目标 + 阈值 → 不算夹住（用二进制精确值，免得被浮点坑）',
         gg.verdict(0.25, 0.375, blocked_m=0.125, timed_out=True), gg.EMPTY)
    same('超过阈值一点就算夹住',
         gg.verdict(0.25, 0.3755, blocked_m=0.125, timed_out=True), gg.HELD)
    same('超时且没有回读 → 判不了', gg.verdict(target, None, timed_out=True), gg.BLIND)
    # 阈值可调
    same('阈值放大到 3 mm：22.9 mm 就不算夹住',
         gg.verdict(target, 0.0229, blocked_m=0.003), None)


def check_describe():
    print('\n3. 给日志的一句话')
    same('没有回读', gg.describe(gg.BLIND, 0.022, None), '没有回读（目标 22.0 mm）')
    text = gg.describe(gg.HELD, 0.022, 0.0242)
    if '22.0' in text and '24.2' in text and '+2.2' in text:
        ok('夹住时把目标、实测、差值都写出来（{}）'.format(text))
    else:
        fail('描述的格式不对', text)


def check_no_hand_commands():
    print('\n4. 源码守卫：抓取节点不许回头去调驱动里的 hand.*')
    path = os.path.join(GRASP, 'dummy_vc_grasp', 'pick_place_node.py')
    with open(path, encoding='utf-8') as handle:
        src = handle.read()
    for banned in ('command_for_width', '.set_angle', 'hand_ready'):
        if banned in src:
            fail('源码里还有 {!r}'.format(banned),
                 '这条会走驱动的 hand.*，真机上会触发 0x382 标定')
        else:
            ok('源码里没有 {!r}'.format(banned))
    guard = ("gripper_enabled" in src and "gripper_gate"
             in src and "close_width" in src)
    if guard:
        ok('走的是 gripper_enabled + gripper_gate.close_width')
    else:
        fail('没有接新的旁路通道（gripper_enabled / gripper_gate）')

    # 只规划（execute=false）时不许去动夹爪：那时手臂没动，在空气里张/合只会
    # 让人以为"夹过了"，合爪还会因为没夹到东西直接判 EMPTY 失败。
    if src.count('if not self.execute:') >= 2:
        ok('张爪/合爪都挡住了"只规划"模式')
    else:
        fail('只规划模式下会去动夹爪',
             'execute=false 时 _open_gripper/_close_gripper 要各自早退')

    # 夹爪零点没量出来时，伸手下去只会空跑：动之前先看 /gripper/status。
    if '_gripper_ready_to_move' in src and 'gripper_status_topic' in src:
        ok('动之前先确认夹爪零点（/gripper/status）')
    else:
        fail('不检查夹爪零点就动手', 'MOVE_PRE_GRASP 之前要调 _gripper_ready_to_move')

    # 放置点：place_x/place_y 曾经声明了却从没被用过，place 退化成抓取点。
    if '_with_place_point' in src and 'pick_place_waypoints' in src:
        ok('放置点真的被接进计划（_with_place_point）')
    else:
        fail('放置点没接进计划', 'place_x/place_y 会继续被无视')


def check_gripper_servo():
    """闭环修正：位置模式有静差，"要 50 停在 52~54"得靠读回再补一发。"""
    print('\n6. 源码守卫：夹爪节点的闭环修正')
    path = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_can_node.py')
    with open(path, encoding='utf-8') as handle:
        src = handle.read()
    for need in ('def _correct_once', 'self._correct_once()',
                 "declare_parameter('width_tol_mm'",
                 "declare_parameter('width_max_iters'",
                 "declare_parameter('width_settle_s'"):
        if need in src:
            ok('源码里有 {!r}'.format(need))
        else:
            fail('源码里没有 {!r}'.format(need),
                 '静差补不回来，张开 50 会停在 52~54')
    # 补发量必须是"2*目标 − 实测"，不能按系数硬推。
    if '2.0 * goal - float(self._last_revs)' in src:
        ok('补发量用 2*目标 − 实测（映射自动对消）')
    else:
        fail('补发量不是 2*目标 − 实测')
    # 到位/放弃/收手三条路径都要把目标清掉，否则会一直补发。
    if src.count('self._target_width_mm = None') >= 3:
        ok('到位、放弃、收手都会清掉目标')
    else:
        fail('有路径没清掉闭环目标', '会一直补发不停')


def main():
    print('gripper_gate 自测')
    check_close_width()
    check_verdict()
    check_describe()
    check_no_hand_commands()
    check_gripper_servo()
    print('\nSUMMARY')
    if FAILED:
        print('  {} 项失败：'.format(len(FAILED)))
        for name in FAILED:
            print('    - ' + name)
        return 1
    print('  RESULT: the grip verdict is pinned down (width, held/empty/blind)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
