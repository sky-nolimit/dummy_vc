#!/usr/bin/env python3
"""离线自检：`tools/read_angles.py` 的零点算术会不会把符号弄反。

这个脚本算出来的数会直接写进 `dummy_controller.yaml`，写错一个符号的后果是
"模型显示对了、手臂反而走反"，而且下次还有人会照它再算一遍。所以要钉住的是：

  * 从配置里读出来的 three 张表长度一致、数值是配置里那一组；
  * 用"理想 home 的读数"算出来的 offset 代回去，模型必须读全零（闭环）；
  * 符号翻转时 offset 也必须整体反号（这条最容易写错成"只反 j1"）；
  * j1 那个具体例子（读数 3.4 度 -> 偏置 +3.185）不许漂。

    python tools/selftest_read_angles.py
"""
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE = os.path.join(HERE, 'read_angles.py')
CONFIG = os.path.join(
    os.path.dirname(HERE), 'dummy_vc_ws', 'src', 'dummy_controller', 'config',
    'dummy_controller.yaml')

FAILURES = []
TOL = 1e-9


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def close(label, got, want, tol=1e-3):
    if abs(got - want) <= tol:
        ok(f'{label}（{got:+.4f}，允许误差 {tol}）')
        return True
    fail(f'{label}: 拿到 {got:+.4f}，应该是 {want:+.4f}±{tol}')
    return False


def load_module():
    spec = importlib.util.spec_from_file_location('read_angles', MODULE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check_config(mod):
    """配置里那几张表要读得出来，而且互相自洽。

    标定之后不再要求 urdf_offset_deg 全零：真正要守的是"原始角 -> 模型角"在
    标定 home 处读零。home_reference_deg 与 urdf_offset_deg 是同一批测量的两
    个面，改了一个忘了另一个，模型就会在 home 处显示一个常数偏置。
    """
    print('\n[1] 读配置与自洽')
    sign, joff, off = mod.read_config_lists(CONFIG)
    if sign is None:
        fail('读不出 joint_sign/joint_offset_deg')
        return
    for name, vals in (('joint_sign', sign), ('joint_offset_deg', joff),
                       ('urdf_offset_deg', off)):
        if len(vals) != 6:
            fail(f'{name} 长度是 {len(vals)}，应该是 6')
    close('joint_sign[0]', sign[0], -1.0)
    close('joint_sign[3]', sign[3], +1.0)
    close('joint_offset_deg[0]', joff[0], 0.215)
    close('joint_offset_deg[1]', joff[1], 74.572)
    close('joint_offset_deg[2]', joff[2], -91.474)

    ref = mod.read_home_reference(CONFIG)
    if ref is None:
        fail('读不出 home_reference_deg')
        return
    model = mod.urdf_angles(ref, sign, joff, off)
    worst = max(abs(v) for v in model)
    if worst <= 1e-6:
        ok(f'标定 home 处模型读零（最大 {worst:.1e}）')
    else:
        fail(f'标定 home 处模型还差 {worst:.3f} 度：home_reference_deg 与 '
             f'urdf_offset_deg 不同步')
    implied = mod.offsets_for_home(ref, sign, joff)
    gap = max(abs(a - b) for a, b in zip(implied, off))
    if gap <= 1e-6:
        ok('urdf_offset_deg 正是这组原始角算出来的')
    else:
        fail(f'urdf_offset_deg 与原始角算出的值差 {gap:.3f} 度')


def check_roundtrip(mod):
    """把 offset 代回去，理想 home 必须读全零 —— 这就是这个工具的用途。"""
    print('\n[2] 理想 home 闭环')
    sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
    joff = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
    # 手工摆出来的"理想 home"，每个关节都偏离了现在记录的值。
    raw = [3.4, 71.0, -93.8, -1.5, -2.7, 1.2]
    off = mod.offsets_for_home(raw, sign, joff)
    model = mod.urdf_angles(raw, sign, joff, off)
    worst = max(abs(v) for v in model)
    if worst <= 1e-9:
        ok(f'代回去六个角都是零（最大 {worst:.2e}）')
    else:
        fail(f'代回去还剩 {worst:.2e}，说明公式或符号不对')
    # 反过来说明：这套 offset 只在"理想 home"成立，别的姿态不该也被压成零。
    other = [3.4 + 10.0, 71.0, -93.8, -1.5, -2.7, 1.2]
    moved = mod.urdf_angles(other, sign, joff, off)
    if abs(moved[0]) > 9.0:
        ok(f'别的姿态不会也被压成零（j1 变成 {moved[0]:+.3f}）')
    else:
        fail(f'别的姿态 j1 也算成了 {moved[0]:+.3f}，offset 被当成了万能修正')


def check_sign_matters(mod):
    """符号整体翻转，offset 必须整体反号，一个关节都不能漏。"""
    print('\n[3] 符号翻转')
    joff = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
    raw = [3.4, 71.0, -93.8, -1.5, -2.7, 1.2]
    pos = [1.0] * 6
    neg = [-1.0] * 6
    a = mod.offsets_for_home(raw, pos, joff)
    b = mod.offsets_for_home(raw, neg, joff)
    for i in range(6):
        if abs(a[i] + b[i]) > TOL:
            fail(f'第 {i + 1} 个关节没有随符号反号：{a[i]:+.3f} vs {b[i]:+.3f}')
            return
    ok('六个关节都整体反号')


def check_j1_example(mod):
    """文档里给用户的那个例子必须算得对。"""
    print('\n[4] j1 的例子')
    # j1: sign = -1, joint_offset_deg = 0.215, 摆到理想 home 时读到 3.4
    off = mod.offsets_for_home([3.4, 0, 0, 0, 0, 0],
                               [-1.0, 0, 0, 0, 0, 0], [0.215, 0, 0, 0, 0, 0])
    close('j1 偏置', off[0], 3.185)
    # 反方向：读到 -2.0 时偏置应为负。
    off2 = mod.offsets_for_home([-2.0, 0, 0, 0, 0, 0],
                                [-1.0, 0, 0, 0, 0, 0], [0.215, 0, 0, 0, 0, 0])
    close('j1 偏置（读到 -2.0）', off2[0], -2.215)


def check_parser(mod):
    """解析器要认注释、空格，缺键要返回 None 而不是炸。"""
    print('\n[5] 解析器')
    text = ('param:\n'
            '    joint_sign: [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]  # 符号\n'
            '    joint_offset_deg: [0.215, 74.572, -91.474, -0.910, '
            '-0.035, -0.040]\n')
    got = mod.parse_flow_list(text, 'joint_offset_deg')
    if got is None or len(got) != 6:
        fail(f'带分组的行没解析出来：{got}')
    else:
        close('解析出的 j2', got[1], 74.572)
    if mod.parse_flow_list(text, 'urdf_offset_deg') is not None:
        fail('缺键时应该返回 None')
    else:
        ok('缺键返回 None')


def check_spread(mod):
    """重复读的跳动量。"""
    print('\n[6] 读数跳动')
    got = mod.spread([[1.0, 2.0], [1.03, 2.5], [0.99, 2.1]])
    close('j1 跳动', got[0], 0.04)
    close('j2 跳动', got[1], 0.5)


def check_home_check(mod):
    """--check-home 的比较：符号必须是"现在 - 参考"，阈值判定要一致。"""
    print('\n[7] 与标定 home 比较')
    ref = [1.435, 74.394, -88.601, 0.389, 1.431, 3.675]
    same = mod.compare_to_reference(ref, ref)
    if any(abs(v) > TOL for v in same):
        fail(f'同一个姿态应当差零，实际 {same}')
    else:
        ok('同一姿态差零')
    # j6 被拧了 +2 度，只有 j6 该报出来。
    moved = list(ref)
    moved[5] += 2.0
    d = mod.compare_to_reference(moved, ref)
    close('j6 差', d[5], 2.0)
    if all(abs(v) < TOL for i, v in enumerate(d) if i != 5):
        ok('其它五个关节不受影响')
    else:
        fail(f'只动 j6，别的关节也变了：{d}')
    if mod.read_home_reference(CONFIG) is None:
        fail('配置里应当有 home_reference_deg')
    else:
        ok('配置里读得到 home_reference_deg')


def main():
    print('=' * 68)
    print('读角工具自检：零点算术的符号与闭环')
    print('=' * 68)
    try:
        mod = load_module()
    except Exception as exc:            # noqa: BLE001 - 报出来比咽下去强
        print(f'  [FAIL] 载入失败：{type(exc).__name__}: {exc}')
        return 1

    check_config(mod)
    check_roundtrip(mod)
    check_sign_matters(mod)
    check_j1_example(mod)
    check_parser(mod)
    check_spread(mod)
    check_home_check(mod)

    print()
    print('=' * 68)
    if FAILURES:
        print(f'{len(FAILURES)} 项失败')
        return 1
    print('RESULT: 零点算术闭环成立，符号翻转、home 比较和缺键都有明确行为')
    return 0


if __name__ == '__main__':
    sys.exit(main())
