#!/usr/bin/env python3
"""把"物块真位置 / 日志里的目标 / 实际抓到"三列数据拆成视觉偏差和执行偏差。

为什么要这个工具，而不是拿眼睛看几组数：

  * 偏差是 1~4 cm，量尺噪声是 5~15 mm，两者只差三倍，看数很容易看错；
  * 三个点就能拟合出任意旋转，残差还都落在量尺噪声里 —— 上一轮就是这样：
    "纯平移"和"转 7.7 度"在 x 上都解得动，光看 x 永远分不开；
  * 手算一回一个结论，下次再来一组数据又要重算，还会算错。

所以这里把三种模型分别拟合、分别报残差：谁解得动、谁解得不动，一眼可见。
点数不够时它不会假装能解 —— 刚体要 4 点、仿射要 6 点，少于这个数就只报平移。

另有一项前置检查：**x 或 y 是不是整个标反了**。反射（x -> -x）不是任何刚体姿态
能产生的，所以标反了的那组数据在刚体模型下会留下 20 cm 级的残差 —— 而人在换了
个站位量尺之后，很容易左右标反。这一项要在任何拟合之前报出来，否则后面所有
结论都是对着镜像算的。

用法::

    python tools/fit_grasp_correction.py points.csv
    python tools/fit_grasp_correction.py --inline "0.10,0.16,0.080,0.153,0.040,0.140"
    python tools/fit_grasp_correction.py points.csv --mirror-x   # 已知标反了，按镜像读
    python tools/fit_grasp_correction.py points.csv --pivot -0.302,0.317
        # 换算成 pick_place 的 vision_yaw_deg / vision_scale / vision_shift_*
        # （--inline 的行首是负数时 argparse 会把它当选项，写成 --inline="-0.1,..."）

CSV 每行六个数，单位米::

    truth_x,truth_y,target_x,target_y,actual_x,actual_y

表头、空行、以 # 开头的注释行都会跳过。
"""
import argparse
import sys

import numpy as np

#: 三段的取列方式：(名字, 起点列, 终到列)，列序见模块开头
STAGES = (
    ('视觉（目标 - 真值）', (0, 2), (2, 4)),
    ('执行（实际 - 目标）', (2, 4), (4, 6)),
    ('合计（实际 - 真值）', (0, 2), (4, 6)),
)

#: 低于这个点数就不拟合对应模型（参数个数：平移 2、刚体 3、仿射 6）
MIN_POINTS = {'平移': 2, '刚体': 4, '仿射': 6}


def read_points(text):
    """从文本里读出 (真值, 目标, 实际) 三元组，单位为米。"""
    points = []
    for lineno, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        parts = [p.strip() for p in s.replace(';', ',').split(',')]
        if len(parts) < 6:
            continue                        # 表头、半截行
        try:
            nums = [float(p) for p in parts[:6]]
        except ValueError:
            continue                        # 表头
        points.append((lineno, np.array(nums)))
    return points


def fit_translation(src, dst):
    """只平移。返回 (平移量, 每点残差向量)。"""
    t = (dst - src).mean(axis=0)
    return t, (src + t) - dst


def fit_rigid(src, dst):
    """刚体：旋转 + 平移（SE(2)）。返回 (转角弧度, 平移量, 残差向量)。"""
    ms, md = src.mean(axis=0), dst.mean(axis=0)
    H = (src - ms).T @ (dst - md)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                # 反射：不该出现在正常数据里
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = md - R @ ms
    return np.arctan2(R[1, 0], R[0, 0]), t, (src @ R.T + t) - dst


def fit_affine(src, dst):
    """仿射：x' = a x + b y + c，y' = d x + e y + f。能吃掉缩放与剪切。"""
    n = len(src)
    A = np.zeros((2 * n, 6))
    b = np.zeros(2 * n)
    A[0::2, 0:2] = src
    A[0::2, 2] = 1.0
    A[1::2, 3:5] = src
    A[1::2, 5] = 1.0
    b[0::2] = dst[:, 0]
    b[1::2] = dst[:, 1]
    coef, *_ = np.linalg.lstsq(A, b, rcond=None)
    M = np.array([[coef[0], coef[1]], [coef[3], coef[4]]])
    return M, coef[[2, 5]], (src @ M.T + coef[[2, 5]]) - dst


def residuals(res):
    return float(np.sqrt((res ** 2).sum(axis=1).mean())), \
        float(np.sqrt((res ** 2).sum(axis=1)).max())


def fmt_vec(v):
    return f'({v[0] * 1000:+.1f}, {v[1] * 1000:+.1f}) mm'

def fit_stage(name, src, dst):
    """拟合一段，打印三种模型的结果，返回刚体结果（点数不够时退回平移）。"""
    n = len(src)
    print(f'\n== {name}    {n} 个点')
    results = {}

    t, res = fit_translation(src, dst)
    rms, mx = residuals(res)
    results['平移'] = dict(rms=rms, max=mx, t=t)
    print(f'   纯平移   {fmt_vec(t)}   残差 RMS {rms * 1000:.1f} mm，最大 {mx * 1000:.1f} mm')

    if n < MIN_POINTS['刚体']:
        print(f'   刚体     跳过：要 {MIN_POINTS["刚体"]} 个点才解得动旋转，现在只有 {n} 个'
              f'（3 个点连"纯平移"和"平移+旋转"都分不开）')
        return '平移', results

    theta, t, res = fit_rigid(src, dst)
    rms, mx = residuals(res)
    results['刚体'] = dict(rms=rms, max=mx, t=t, theta=theta)
    print(f'   刚体     转 {np.degrees(theta):+.2f} 度，平移 {fmt_vec(t)}   '
          f'残差 RMS {rms * 1000:.1f} mm，最大 {mx * 1000:.1f} mm')

    if n < MIN_POINTS['仿射']:
        print(f'   仿射     跳过：要 {MIN_POINTS["仿射"]} 个点，现在 {n} 个'
              f'（再多一点就是过拟合，报出来的数不代表任何东西）')
    else:
        M, t2, res = fit_affine(src, dst)
        rms, mx = residuals(res)
        results['仿射'] = dict(rms=rms, max=mx, t=t2, M=M)
        print(f'   仿射     x 缩放 {M[0, 0]:.3f} 剪切 {M[0, 1]:+.3f} / '
              f'y 剪切 {M[1, 0]:+.3f} 缩放 {M[1, 1]:.3f}，平移 {fmt_vec(t2)}   '
              f'残差 RMS {rms * 1000:.1f} mm，最大 {mx * 1000:.1f} mm')

    best = min(('平移', '刚体') + (('仿射',) if '仿射' in results else ()),
               key=lambda k: results[k]['rms'])
    print(f'   -> 最贴的是「{best}」')
    return best, results


def residual_rms(res):
    return float(np.sqrt((res ** 2).sum(axis=1).mean()))


def similarity_params(M, t, pivot):
    """把 `x' = M x + t` 改成"绕 pivot 转 + 缩放 + 平移"的四个参数。

    pick_place 的视觉修正（`_correct_transform`）就是这种形状：

        x' = pivot + scale * R(yaw) * (x - pivot) + shift

    所以给了 pivot 之后四个参数唯一确定。之所以要换成这个形状，是因为
    "旋转"只有绕某个点才有意义：同一个刚体变换，绕原点和绕相机写出来的
    平移量能差好几厘米，直接抄 `t` 会修错地方。

    返回 (转角度数, 缩放, 平移向量, 各向异性残差)。最后一项是 `M` 里
    被"转+缩放"吃不掉的剪切：很小的数（< 0.01）说明缩放是各向同性的，
    模型够用；大了说明问题出在某个长度/高度估错，不是标定能修的。
    """
    U, sv, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:                # 反射：任何相似变换都做不出来
        Vt[-1] *= -1
        R = U @ Vt
    scale = float(np.sqrt(abs(np.linalg.det(M))))
    yaw = float(np.arctan2(R[1, 0], R[0, 0]))
    shift = (M - np.eye(2)) @ pivot + t
    P = R.T @ M
    shear = float(np.abs(P - scale * np.eye(2)).max() / max(scale, 1e-9))
    return np.degrees(yaw), scale, shift, shear


def symmetric_about(src, axis, tol=0.02):
    """这组点在某个坐标上是不是成对的（±x 都有点）。

    成对之后，"x 全反"和"y 全反 + 转 180 度"是同一件事，光看残差分不出是哪个轴
    标反了 —— 这一点要明说，否则会给出一个猜出来的答案。
    """
    for p in src:
        flipped = p.copy()
        flipped[axis] = -flipped[axis]
        if not any(np.linalg.norm(q - flipped) <= tol for q in src):
            return False
    return True


def detect_mirror(src, dst):
    """x、y 各自标反了没有。反射不是刚体变换，标反了刚体就拟合不上。"""
    theta, _, res = fit_rigid(src, dst)
    base_rms = residual_rms(res)
    report = []
    for axis, label in ((0, 'x'), (1, 'y')):
        flipped = src.copy()
        flipped[:, axis] *= -1.0
        _, _, res2 = fit_rigid(flipped, dst)
        report.append((label, residual_rms(res2), symmetric_about(src, axis)))
    return base_rms, np.degrees(theta), report


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('csv', nargs='?', help='数据文件（六列，单位米）')
    ap.add_argument('--inline', help='直接给六列数据，用 ; 分隔多行')
    ap.add_argument('--unit', choices=('m', 'mm'), default='m',
                    help='输入单位，默认米')
    ap.add_argument('--mirror-x', action='store_true',
                    help='x 已知标反了，按取反后的真值来算')
    ap.add_argument('--pivot', metavar='X,Y',
                    help='视觉修正要绕的点（相机正下方），单位米。给了就把拟合结果'
                         '直接换算成 pick_place 的 vision_yaw_deg / vision_scale / '
                         'vision_shift_x_m / vision_shift_y_m')
    args = ap.parse_args()

    if args.inline:
        text = args.inline.replace(';', '\n')
    elif args.csv:
        with open(args.csv, encoding='utf-8') as fh:
            text = fh.read()
    else:
        ap.error('要么给 csv，要么给 --inline')

    points = read_points(text)
    if len(points) < 2:
        print('至少要有 2 个点', file=sys.stderr)
        return 2
    scale = 0.001 if args.unit == 'mm' else 1.0
    truth = np.array([p[1][0:2] * scale for p in points])
    target = np.array([p[1][2:4] * scale for p in points])
    actual = np.array([p[1][4:6] * scale for p in points])

    print(f'读入 {len(points)} 个点（单位 {args.unit}，下面一律按 mm 显示）')

    # 先查标反 —— 必须在任何拟合之前
    if len(points) >= 3:
        base_rms, base_theta, report = detect_mirror(truth, target)
        worse = [r for r in report if r[1] < base_rms * 0.5]
        if worse:
            label, rms2, is_symmetric = worse[0]
            print(f'\n!! 有一个轴整个标反了：取反之后视觉残差从 {base_rms * 1000:.0f} mm '
                  f'掉到 {rms2 * 1000:.0f} mm')
            print('   反射不是任何刚体姿态能产生的，所以这是标号的问题，不是相机的问题。')
            if is_symmetric:
                print(f'   但这组点在 {label} 上成对（±{label} 都有点），'
                      f'"x 全反"和"y 全反 + 转 180 度"在这组数上是同一件事，'
                      f'工具分不出是哪一个。')
                print('   加一个不对称的点（比如 (0, 0.21)）再测，或者拿相机本体的位置当锚点。')
            else:
                print(f'   确认之后加 --mirror-x 重跑，或者把那一列的符号改过来。')
        else:
            print(f'\n方向符号自洽（视觉刚体残差 {base_rms * 1000:.0f} mm）')
        if abs(base_theta) > 90:
            print(f'   注意：最好的刚体拟合要转 {base_theta:+.0f} 度 —— 真实数据不会这样，'
                  f'同样说明这组数里有个反射，或者点太少。')

    if args.mirror_x:
        truth = truth.copy()
        truth[:, 0] *= -1.0
        print('已按 --mirror-x 把真值的 x 取反')

    verdicts = {}
    for name, src_cols, dst_cols in STAGES:
        src = truth if src_cols == (0, 2) else target
        dst = target if dst_cols == (2, 4) else actual
        verdicts[name] = fit_stage(name, src, dst)

    if args.pivot:
        try:
            px, py = (float(s) for s in args.pivot.split(','))
        except ValueError:
            ap.error('--pivot 要写成 "x,y" 两个数，单位米')
        pivot = np.array([px, py])
        _, vis = verdicts['视觉（目标 - 真值）']
        print(f'\n== 填进 config/pick_place.yaml 的视觉修正（绕 ({px:+.3f}, {py:+.3f}) m）')
        for model in ('刚体', '仿射'):
            if model not in vis:
                continue
            if model == '刚体':
                th = vis[model]['theta']
                M = np.array([[np.cos(th), -np.sin(th)],
                              [np.sin(th), np.cos(th)]])
            else:
                M = vis[model]['M']
            # 上面拟合出来的是"真值 -> 检测"；节点要的是反过来的
            # "检测 -> 真值"，所以先求逆，再换算成绕 pivot 的四个参数。
            try:
                Minv = np.linalg.inv(M)
            except np.linalg.LinAlgError:
                print(f'   「{model}」不可逆，跳过')
                continue
            tinv = -Minv @ vis[model]['t']
            yaw, sc, shift, shear = similarity_params(Minv, tinv, pivot)
            print(f'   「{model}」拟合（残差 RMS {vis[model]["rms"] * 1000:.1f} mm）：')
            print(f'     vision_yaw_deg:   {yaw:+.2f}')
            print(f'     vision_scale:     {sc:.3f}')
            print(f'     vision_shift_x_m: {shift[0]:+.4f}')
            print(f'     vision_shift_y_m: {shift[1]:+.4f}')
            if shear > 0.01:
                print(f'     注意：还剩 {shear:.3f} 的剪切吃不掉（不是转+缩放能表达的），'
                      f'说明某个长度或高度估错了，别硬拿这四个数去盖')
        if '平移' in vis and '刚体' not in vis:
            t = vis['平移']['t']
            print(f'   点太少，只能当纯平移：'
                  f'vision_shift_x_m: {t[0]:+.4f}  vision_shift_y_m: {t[1]:+.4f}')

    print('\n' + '=' * 70)
    print('怎么看这些数：')
    print('  * 平移和刚体的残差差不多 -> 偏差是一个固定偏移，改外参/零位一处就够；')
    print('  * 刚体明显更贴（而且转角不止一两度）-> 偏差里有旋转，先查那条轴；')
    print('  * 仿射明显更贴 -> 是缩放或剪切，说明某个长度或高度估错了，不是标定能修的；')
    print('  * 残差本身就有 1 cm 以上 -> 量尺不够准，再多点几个位置，别急着下结论。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
