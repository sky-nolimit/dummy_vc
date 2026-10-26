#!/usr/bin/env python3
"""离线自查：驱动"启动位姿核对"的判定，以及它确实被接进了 configure。

要防的故障：

    某个轴被碰过一点点，可日志里什么都没说 —— 于是"模型和实物不一致"
    这件事要等到抓歪了三次之后才被发现

判定函数 `describe_home_gap` 住在 `dummy_controller.py` 里，而这个文件
import 了 rclpy/scipy，在 Windows 上根本导不进来。所以这里用 `ast` 把那个
函数从源码里抠出来单独 exec —— 测的就是**驱动里那份**，不是复制的副本。
再顺手断言 `on_configure` 真的调了它：接上产线比写对判定更容易忘。

    python tools/selftest_home_check.py
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
DRIVER = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'dummy_controller', 'dummy_controller.py')

#: 标定 home 的原始角（与 config/dummy_controller.yaml 的 home_reference_deg 同值）
HOME_RAW = [1.435, 74.394, -88.601, 0.389, 1.431, 3.675]

FAILURES = []


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def same(label, got, want):
    if got == want:
        ok(f'{label} = {got}')
    else:
        fail(f'{label}：得到 {got}，应为 {want}')


def load_function(path, name):
    """把源码里那个顶层函数抠出来单独执行，不 import 整个模块。"""
    with open(path, encoding='utf-8') as fh:
        tree = ast.parse(fh.read(), filename=path)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            mod = ast.Module(body=[node], type_ignores=[])
            env = {}
            exec(compile(mod, path, 'exec'), env)   # noqa: S102 - 只跑本仓库源码
            return env[name]
    raise LookupError(f'{path} 里没有顶层函数 {name}')


def check_verdicts(fn):
    """五种情形各来一次，还要守住差值的符号。"""
    print('\n[1] 判定')
    d, v = fn(HOME_RAW, HOME_RAW, 1.0)
    same('原地不动 -> verdict', v, 'at_home')
    same('原地不动 -> 差值', d, [0.0] * 6)

    nudged = list(HOME_RAW)
    nudged[5] += 2.4
    d, v = fn(nudged, HOME_RAW, 1.0)
    same('只动 j6 -> verdict', v, 'one_axis')
    same('只动 j6 -> j6 差值', round(d[5], 3), 2.4)
    if all(abs(x) < 1e-9 for i, x in enumerate(d) if i != 5):
        ok('只动 j6 -> 其它五轴差值为零')
    else:
        fail(f'只动 j6，别的轴也变了：{d}')

    two = list(HOME_RAW)
    two[0] -= 2.0
    two[4] += 1.5
    same('动两个轴 -> verdict', fn(two, HOME_RAW, 1.0)[1], 'one_axis')

    three = list(HOME_RAW)
    for i in (0, 2, 4):
        three[i] += 5.0
    same('动三个轴 -> verdict', fn(three, HOME_RAW, 1.0)[1], 'mixed')

    away = [a + 30.0 for a in HOME_RAW]
    same('整个停在别处 -> verdict', fn(away, HOME_RAW, 1.0)[1], 'elsewhere')

    loose = list(HOME_RAW)
    loose[1] += 0.9
    same('容差内不算偏', fn(loose, HOME_RAW, 1.0)[1], 'at_home')
    same('放宽容差后算偏', fn(loose, HOME_RAW, 0.5)[1], 'one_axis')

    same('没标定 -> verdict', fn(HOME_RAW, None, 1.0)[1], 'no_reference')
    same('没标定 -> 差值', fn(HOME_RAW, None, 1.0)[0], [])
    same('参考长度不对 -> verdict', fn(HOME_RAW, [0.0] * 5, 1.0)[1],
         'no_reference')


def check_sign(fn):
    """差值必须是"现在 - 标定"，符号反了会让人往错的方向改。"""
    print('\n[2] 符号')
    lower = list(HOME_RAW)
    lower[0] -= 3.0
    d, _ = fn(lower, HOME_RAW, 1.0)
    same('j1 比标定小 3 度 -> 差值为负', round(d[0], 3), -3.0)


def check_wired():
    """光有函数不算：configure 里得真的调用它，而且要在 homing 之前。"""
    print('\n[3] 接进 configure')
    with open(DRIVER, encoding='utf-8') as fh:
        src = fh.read()
    if 'self._check_home_pose()' not in src:
        fail('dummy_controller.py 里没有调用 _check_home_pose()')
        return
    ok('调用点存在')
    call = src.index('self._check_home_pose()')
    home = src.index('self.home_dummy()')
    if call < home:
        ok('调用在 homing 之前（homing 之后就分不清是谁歪了）')
    else:
        fail('调用在 homing 之后，那时已经看不出上电时的位姿了')
    if 'def _home_reference(' in src:
        ok('参考姿态由 _home_reference 反推')
    else:
        fail('驱动里没有 _home_reference，检查会永远跳过')
    # 回归：它曾经是独立参数、默认值写成 []，rclpy 把空列表推成 BYTE_ARRAY，
    # YAML 里的 double array 直接让 on_configure 抛
    # InvalidParameterTypeException。反推就不该再有这个参数。
    if "declare_parameter('home_reference_deg'" in src:
        fail('又把 home_reference_deg 声明成参数了：它会和偏置表不同步，'
             '而且空列表默认值会被 rclpy 当成 BYTE_ARRAY')
    else:
        ok('没有把 home_reference_deg 再声明成参数')


def main():
    print('=' * 68)
    print('启动位姿核对自检：判定、符号、接线')
    print('=' * 68)
    try:
        fn = load_function(DRIVER, 'describe_home_gap')
    except Exception as exc:            # noqa: BLE001
        print(f'  [FAIL] 抠不出 describe_home_gap：'
              f'{type(exc).__name__}: {exc}')
        return 1
    check_verdicts(fn)
    check_sign(fn)
    check_wired()

    print()
    print('=' * 68)
    if FAILURES:
        print(f'{len(FAILURES)} 项失败')
        return 1
    print('RESULT: 位姿核对能分清"停错姿态"和"被碰过"，且已接进 configure')
    return 0


if __name__ == '__main__':
    sys.exit(main())
