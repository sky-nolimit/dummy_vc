#!/usr/bin/env python3
"""离线自查：`tools/fit_grasp_correction.py` 会不会把偏差读错。

这个工具是拿来下结论的 —— "偏差是平移还是旋转"决定了是改外参还是查 j1 零位，
所以它自己必须先被验过。要防的三种错，都在真机上见过或差点见过：

  * 转角解出来差个几度，就把"平移"说成"旋转"：合成一组已知旋转的数据，
    解出来必须落回原值（含 2 mm 量尺噪声）。
  * 标反的符号被当成真的：反射不是刚体变换，标反了残差应该是十倍级，
    而且**成对的点位**下工具不许指认具体是哪个轴 —— 那是在猜。
  * 点太少也硬解：3 个点解旋转等于过拟合，工具必须拒绝，而不是给个数。

    python tools/selftest_fit_grasp_correction.py
"""
import contextlib
import importlib.util
import io
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODULE = os.path.join(HERE, 'fit_grasp_correction.py')

FAILURES = []


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def close(label, got, want, tol):
    if abs(got - want) <= tol:
        ok(f'{label}（{got:.4f}，允许误差 {tol}）')
        return True
    fail(f'{label}: 拿到 {got:.4f}，应该是 {want:.4f}±{tol}')
    return False


def load_module():
    spec = importlib.util.spec_from_file_location('fit_grasp_correction', MODULE)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def quiet(func, *args, **kwargs):
    """跑一个会打印的函数，但把它的输出收走，免得把自查报告冲散。"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        out = func(*args, **kwargs)
    return out, buf.getvalue()


def _fit_quiet(mod, src, dst):
    """跑一次 fit_stage，拿回 (判定, 各模型残差)，顺手吞掉它的打印。"""
    (best, results), _ = quiet(mod.fit_stage, '假数据', src, dst)
    return best, results


def grid(n_side=3, lo=-0.10, hi=0.10, y_lo=0.16, y_hi=0.28):
    """一块不对称的网格：x 从 lo 到 hi，y 从 y_lo 到 y_hi，右上角缺一格。"""
    xs = np.linspace(lo, hi, n_side)
    ys = np.linspace(y_lo, y_hi, n_side)
    pts = np.array([(x, y) for y in ys for x in xs])
    return pts[:-1]                       # 去掉一个点，让它在两个轴上都成不了对


def rotate(points, deg):
    th = np.radians(deg)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return points @ R.T


def check_rigid_roundtrip(mod):
    print('\n1. 已知的旋转+平移，能不能解回来')
    rng = np.random.default_rng(0)
    src = grid()
    truth_deg, truth_t = 8.0, np.array([0.010, -0.020])
    dst = rotate(src, truth_deg) + truth_t + rng.normal(0, 0.002, src.shape)
    theta, t, res = mod.fit_rigid(src, dst)
    rms = mod.residual_rms(res)
    close('转角', np.degrees(theta), truth_deg, 1.0)
    close('平移 x', t[0], truth_t[0], 0.005)
    close('平移 y', t[1], truth_t[1], 0.005)
    if rms < 0.003:
        ok(f'残差 {rms * 1000:.1f} mm（噪声 2 mm，解出来应该就在这附近）')
    else:
        fail(f'残差 {rms * 1000:.1f} mm，比噪声大太多')


def check_translation_is_not_called_rotation(mod):
    print('\n2. 纯平移的数据，不许解出旋转来')
    rng = np.random.default_rng(1)
    src = grid()
    dst = src + np.array([-0.030, -0.025]) + rng.normal(0, 0.002, src.shape)
    theta, _, _ = mod.fit_rigid(src, dst)
    close('转角', np.degrees(theta), 0.0, 1.0)
    best, results = _fit_quiet(mod, src, dst)
    if best == '平移':
        ok('判定为「平移」')
    else:
        # 刚体多一个自由度，残差总会小一点点；只要不是明显更贴就行
        ratio = results['刚体']['rms'] / results['平移']['rms']
        if ratio > 0.8:
            ok(f'判定为「{best}」但两者残差只差 {(1 - ratio) * 100:.0f}%，'
               f'不算"明显更贴"')
        else:
            fail(f'判定为「{best}」，残差却比平移小了 {(1 - ratio) * 100:.0f}%')


def check_mirror_named(mod):
    print('\n3. x 标反了，而且点位不对称 —— 应该直接指认 x')
    src = grid()
    dst = src * np.array([-1.0, 1.0])
    base_rms, theta, report = mod.detect_mirror(src, dst)
    if abs(theta) > 90:
        ok(f'视觉刚体转 {theta:+.0f} 度（正常数据不会这样）')
    else:
        fail(f'转角只有 {theta:+.1f} 度，没体现出反射')
    hit = [r for r in report if r[1] < base_rms * 0.5]
    if not hit:
        fail('没有报出任何一个轴标反了')
        return
    label, rms2, is_sym = hit[0]
    if label == 'x':
        ok(f'指认 x（残差 {base_rms * 1000:.0f} -> {rms2 * 1000:.0f} mm）')
    else:
        fail(f'指认成了 {label}')
    if not is_sym:
        ok('这组点不对称，指认是站得住的')
    else:
        fail('这组点其实是成对的，工具不该指认具体轴')


def check_mirror_ambiguous(mod):
    print('\n4. 点位成对时，只许说"有一个轴反了"，不许指认')
    src = np.array([(0.10, 0.16), (-0.10, 0.16), (0.10, 0.26), (-0.10, 0.26)])
    dst = src * np.array([-1.0, 1.0])
    if mod.symmetric_about(src, 0):
        ok('认出这组点在 x 上成对')
    else:
        fail('没认出成对 —— 那就会给出一个猜出来的答案')
    base_rms, _, report = mod.detect_mirror(src, dst)
    hit = [r for r in report if r[1] < base_rms * 0.5]
    if hit and hit[0][2]:
        ok('报出来了，并且标了"成对、分不出是哪个轴"')
    else:
        fail('要么没报，要么没标成对')


def check_too_few_points(mod):
    print('\n5. 点太少，不许硬解旋转和仿射')
    src = np.array([(0.0, 0.16), (0.0, 0.21), (0.0, 0.26)])
    dst = src + np.array([-0.02, -0.01])
    best, results = _fit_quiet(mod, src, dst)
    if best == '平移' and '刚体' not in results and '仿射' not in results:
        ok('3 个点只给平移')
    else:
        fail(f'3 个点给了「{best}」')

    src6 = grid(n_side=3)
    if len(src6) < mod.MIN_POINTS['仿射']:
        _, results6 = _fit_quiet(mod, src6, src6 + 0.01)
        if '仿射' not in results6:
            ok(f'{len(src6)} 个点时不给仿射（要 {mod.MIN_POINTS["仿射"]} 个）')
        else:
            fail('点数不够却给了仿射')


def check_parser(mod):
    print('\n6. 数据文件：表头、空行、注释都不许算成点')
    text = (
        '# 2026-10-25 四个点\n'
        'truth_x,truth_y,target_x,target_y,actual_x,actual_y\n'
        '\n'
        '-0.10,0.16,0.080,0.153,0.040,0.140\n'
        '0.10,0.16,-0.101,0.154,-0.110,0.110\n'
        '   \n'
        '# 下面这行少一列，跳过\n'
        '0.10,0.26,-0.103\n'
        '0.10,0.26,-0.103,0.252,-0.130,0.200\n'
    )
    pts = mod.read_points(text)
    if len(pts) == 3:
        ok('读出 3 个点')
    else:
        fail(f'读出 {len(pts)} 个点，应该是 3 个')
    if pts and abs(pts[0][1][2] - 0.080) < 1e-9:
        ok('第一行数值对得上')
    else:
        fail('第一行数值读错了')


def check_similarity_params(mod):
    """绕 pivot 的"转+缩放+平移"必须能原样重建那个刚体变换。"""
    print('\n7. 换成"绕 pivot 转+缩放"的参数量，能不能还原原来的变换')
    pivot = np.array([-0.302, 0.317])
    src = grid()
    # 和 geometry.similarity_correction 同一套约定：x' = pivot + s R (x - pivot) + shift
    th = np.radians(7.4)
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    scale_want = 0.90
    M = scale_want * R
    t = np.array([0.004, -0.006])
    want = src @ M.T + t
    yaw, sc, shift, shear = mod.similarity_params(M, t, pivot)
    close('转角', yaw, 7.4, 1e-9)
    close('缩放', sc, scale_want, 1e-12)
    if shear < 1e-9:
        ok('各向同性，没有剪切残差')
    else:
        fail(f'各向同性的数据解出剪切 {shear:.4f}，公式推错了')
    rebuilt = pivot + sc * (src - pivot) @ R.T + shift
    err = float(np.abs(rebuilt - want).max())
    if err < 1e-9:
        ok(f'重建误差 {err:.2e} m')
    else:
        fail(f'重建误差 {err:.2e} m，四个参数没把变换还原完整')


def main():
    print('=' * 68)
    print('SELFTEST  fit_grasp_correction')
    print('=' * 68)
    try:
        mod = load_module()
    except Exception as exc:            # noqa: BLE001 - 报出来比咽下去强
        print(f'  [FAIL] 载入失败：{type(exc).__name__}: {exc}')
        return 1

    check_rigid_roundtrip(mod)
    check_translation_is_not_called_rotation(mod)
    check_mirror_named(mod)
    check_mirror_ambiguous(mod)
    check_too_few_points(mod)
    check_parser(mod)
    check_similarity_params(mod)

    print()
    print('=' * 68)
    if FAILURES:
        print(f'{len(FAILURES)} 项失败')
        return 1
    print('RESULT: 拟合工具在已知答案上都解得回来，标反和点太少都会拒绝')
    return 0


if __name__ == '__main__':
    sys.exit(main())
