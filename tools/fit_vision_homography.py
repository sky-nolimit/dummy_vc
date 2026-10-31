#!/usr/bin/env python3
"""用一组"真值 / 相机检测"点，拟合桌面上的单应修正，直接给出配置参数。

为什么是单应，不是平移或仿射：2026-10-31 真机 11 个点（x 跨 20 cm、y 跨 20 cm）
的留一验证结果：

    平移            18.9 mm
    平移+缩放+旋转   14.0 mm
    仿射             9.2 mm
    单应             4.5 mm

物体都在同一张桌面上时，工具面到工具面的映射本身就是单应（透视投影把平面
映成平面）。相机的位姿、俯仰、主点任何一个不准，都恰好落在这个模型里；而
平移、缩放、旋转这些都是它的特例，所以点数够时没理由用更弱的模型。

**留一验证（LOO）才是这里的验收标准**，不是拟合残差：单应 8 个参数，11 个点
的拟合残差必然小，换一个没见过的点才知道能不能用。每条结果都会把 LOO 打出来。

用法::

    python tools/fit_vision_homography.py points.csv
    python tools/fit_vision_homography.py --inline "-0.1,0.15,-0.099,0.116;0,0.15,0.006,0.103;0.1,0.15,0.125,0.088"

CSV 每行四个数，单位米，顺序是::

    truth_x,truth_y,det_x,det_y

表头、空行、以 # 开头的注释行都会跳过。输出里那 8 个数直接抄进
`config/pick_place.yaml` 的 `vision_homography`（检测值 -> 真值的方向）。
"""
import argparse
import os
import sys

import numpy as np

#: 几何层在 ROS 包里，这里跟其它工具一样手工加进 sys.path，不去依赖 colcon 环境
REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PKG_ROOT = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_perception')
if PKG_ROOT not in sys.path:
    sys.path.insert(0, PKG_ROOT)

#: 少于这个点数就只报更弱的模型（参数个数：平移 2、仿射 6、单应 8）
MIN_POINTS = {'平移': 2, '仿射': 5, '单应': 6}

#: 仿射的点数下限比参数个数多 1，否则留一验证没意义
AFFINE_MIN = 4


def read_points(text):
    """读出 [(真值 xy, 检测 xy), ...]，单位为米。"""
    points = []
    for lineno, line in enumerate(text.splitlines(), 1):
        s = line.strip()
        if not s or s.startswith('#'):
            continue
        parts = [p.strip() for p in s.replace(';', ',').split(',')]
        if len(parts) < 4:
            continue                        # 表头、半截行
        try:
            vals = [float(p) for p in parts[:4]]
        except ValueError:
            continue                        # 表头
        truth = np.array(vals[:2], float)
        det = np.array(vals[2:4], float)
        if not (np.all(np.isfinite(truth)) and np.all(np.isfinite(det))):
            print('第 {} 行有非有限数，跳过：{}'.format(lineno, s), file=sys.stderr)
            continue
        points.append((truth, det))
    return points


def fit_translation(det, truth):
    shift = truth.mean(axis=0) - det.mean(axis=0)
    return lambda x: np.asarray(x, float) + shift, shift


def fit_affine(det, truth):
    a = np.hstack([np.asarray(det, float), np.ones((len(det), 1))])
    m, *_ = np.linalg.lstsq(a, np.asarray(truth, float), rcond=None)
    return lambda x: np.hstack([np.asarray(x, float),
                                np.ones((len(x), 1))]) @ m, m


def fit_homography(det, truth):
    from dummy_vc_perception import geometry

    m = geometry.homography_from_points(det, truth)

    def apply(x):
        x = np.asarray(x, float)
        return np.array([geometry.homography_correction(p, m) for p in x])

    return apply, m


def residuals(fn, det, truth):
    err = np.asarray(fn(det), float) - np.asarray(truth, float)
    return np.linalg.norm(err, axis=1)


def loo(fitter, det, truth, min_points):
    """留一验证：每个点都用"去掉它"拟合出来的模型去预测。"""
    if len(det) < min_points + 1:
        return None
    out = []
    for i in range(len(det)):
        keep = [j for j in range(len(det)) if j != i]
        fn, _ = fitter(np.asarray(det)[keep], np.asarray(truth)[keep])
        out.append(np.linalg.norm(np.asarray(fn(det[i:i + 1]))[0] - truth[i]))
    return np.asarray(out, float) * 1e3


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('csv', nargs='?', help='四列 CSV：truth_x,truth_y,det_x,det_y')
    parser.add_argument('--inline', help='直接给点，用分号隔开，每点四个数')
    args = parser.parse_args(argv)

    if args.csv:
        with open(args.csv, encoding='utf-8') as handle:
            text = handle.read()
    elif args.inline:
        text = args.inline
    else:
        parser.error('要么给 CSV 文件，要么用 --inline')

    points = read_points(text)
    if len(points) < 4:
        print('至少要点 4 个才能拟合单应：现在只有 {} 个'.format(len(points)),
              file=sys.stderr)
        return 2
    truth = np.array([p[0] for p in points], float)
    det = np.array([p[1] for p in points], float)

    errors = np.linalg.norm(truth - det, axis=1) * 1e3
    print('{} 个点，原始误差 平均 {:.1f} mm / 最大 {:.1f} mm'.format(
        len(points), errors.mean(), errors.max()))

    models = []
    fn_t, shift = fit_translation(det, truth)
    models.append(('平移', fn_t, len(det), 2))
    if len(points) >= AFFINE_MIN:
        fn_a, m_a = fit_affine(det, truth)
        models.append(('仿射', fn_a, len(points), 6))
    fn_h, m_h = fit_homography(det, truth)
    models.append(('单应', fn_h, len(points), 8))

    print('\n{:<6}{:>10}{:>10}{:>12}'.format('模型', '拟合 rmse', '拟合 max', '留一 rmse'))
    for name, fn, npts, npar in models:
        err = residuals(fn, det, truth) * 1e3
        loo_err = loo({'平移': fit_translation, '仿射': fit_affine,
                       '单应': fit_homography}[name], det, truth,
                      MIN_POINTS[name])
        print('{:<6}{:>9.1f}mm{:>8.1f}mm{:>11}'.format(
            name, np.sqrt((err ** 2).mean()), err.max(),
            '点数不够' if loo_err is None
            else '{:.1f}mm (max {:.1f})'.format(
                np.sqrt((loo_err ** 2).mean()), loo_err.max())))

    err_h = residuals(fn_h, det, truth) * 1e3
    print('\n单应逐点残差（mm）：',
          ' '.join('{:.0f}'.format(v) for v in err_h))

    print('\n抄进 config/pick_place.yaml 的这一行（检测 -> 真值）：')
    print('    vision_homography: [{}]'.format(
        ', '.join('{:.6f}'.format(v) for v in m_h)))
    print('\n这 8 个数是 h11 h12 h13 h21 h22 h23 h31 h32（h33 固定 1）。')
    print('判据：留一 rmse < 10 mm 就比现在准得多；大于 15 mm 说明点不够散或'
          '量的位置不准，别急着填进配置。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
