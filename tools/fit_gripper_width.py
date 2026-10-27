#!/usr/bin/env python3
"""把"电机圈数 → 开口毫米"的实测点拟合成两个常数，直接告诉你改哪两行。

夹爪的开口与电机圈数是线性的（刚性齿条），所以只要两个数：

    开口(mm) = REV_AT_ZERO_WIDTH_MM - MM_PER_REV * 圈数

`gripper_frames.py` 里那两个常量就是这么来的。**标定之后必须重测**：标定决定的是
`encoderHomeOffset`，同一圈数对应的物理位置会整体平移，旧的三点直接作废。

数据从 `tools/gripper_sweep.py` 生成的 CSV 读（`revs,width_mm`），也可以
`--inline "0.10,12.5;0.25,10.5;0.50,8.0"` 手写。

用法：

    python3 tools/fit_gripper_width.py gripper_width.csv
    python3 tools/fit_gripper_width.py --inline "0.10,12.5;0.25,10.5;0.50,8.0"
"""
import argparse
import sys


class NotEnoughPoints(ValueError):
    """少于两个点解不出直线。宁可不给数，也不给一个假装可信的数。"""


def parse_inline(text):
    """`"0.10,12.5; 0.25,10.5"` → [(0.10, 12.5), (0.25, 10.5)]"""
    points = []
    for chunk in text.replace('\n', ';').split(';'):
        chunk = chunk.strip()
        if not chunk:
            continue
        parts = [p.strip() for p in chunk.split(',')]
        if len(parts) != 2:
            raise ValueError('这一块不是 "圈数,开口": {!r}'.format(chunk))
        points.append((float(parts[0]), float(parts[1])))
    return points


def read_csv(path):
    """读 `gripper_sweep.py` 的 CSV，跳过表头、空行、注释和没量（空值）的行。

    两种列序都认：
      * 四列（`target_revs,readback_revs,width_mm,finished`）→ 取第 1、3 列；
      * 两列（`revs,width_mm`）→ 取第 1、2 列。
    取不到数就跳过 —— **绝不许把没量的行当成 0 mm**，那会把拟合整条拉歪。
    """
    points = []
    with open(path, encoding='utf-8') as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) < 2:
                continue
            revs_text, width_text = (parts[0], parts[2]) if len(parts) >= 3 \
                else (parts[0], parts[1])
            try:
                revs = float(revs_text)
                width = float(width_text)
            except ValueError:
                continue          # 表头、没量（空值）、或者是坏行
            points.append((revs, width))
    return points


def fit_width(points):
    """最小二乘拟合 开口 = a + b * 圈数，返回 a、b、残差、RMS。"""
    points = list(points)
    if len(points) < 2:
        raise NotEnoughPoints('至少要两个点，现在只有 {} 个'.format(len(points)))
    if len({round(r, 9) for r, _ in points}) < 2:
        raise NotEnoughPoints('所有点的圈数都一样，画不出直线')
    n = float(len(points))
    mean_x = sum(r for r, _ in points) / n
    mean_y = sum(w for _, w in points) / n
    sxx = sum((r - mean_x) ** 2 for r, _ in points)
    sxy = sum((r - mean_x) * (w - mean_y) for r, w in points)
    slope = sxy / sxx
    intercept = mean_y - slope * mean_x
    residuals = [w - (intercept + slope * r) for r, w in points]
    rms = (sum(e * e for e in residuals) / n) ** 0.5
    return {
        'slope': slope,
        'intercept': intercept,
        'residuals': residuals,
        'rms': rms,
        'points': points,
    }


def report(fit, stream=sys.stdout):
    """把拟合结果印成人能直接抄的样子。"""
    a = fit['intercept']
    b = -fit['slope']
    print('\n拟合：开口 = {:.3f} - {:.3f} * 圈数'.format(a, b), file=stream)
    print('{:>10} {:>10} {:>10} {:>9}'.format('圈数', '实测 mm', '拟合 mm', '残差'),
          file=stream)
    for (revs, width), res in zip(fit['points'], fit['residuals']):
        print('{:>10.4f} {:>10.1f} {:>10.2f} {:>+9.2f}'.format(
            revs, width, a + fit['slope'] * revs, res), file=stream)
    print('\n残差 RMS {:.2f} mm，最大 {:.2f} mm'.format(
        fit['rms'], max(abs(e) for e in fit['residuals'])), file=stream)

    print('\n改这两行（dummy_vc_ws/src/dummy_vc_grasp/dummy_vc_grasp/'
          'gripper_frames.py）：', file=stream)
    print('  REV_AT_ZERO_WIDTH_MM = {:.3f}'.format(a), file=stream)
    print('  MM_PER_REV = {:.3f}'.format(b), file=stream)
    print('\n参考：0 圈 → {:.1f} mm、开口 24 mm → {:.3f} 圈、开口 26 mm → {:.3f} 圈、'
          '满开 55 mm → {:.3f} 圈'.format(
              a,
              (a - 24.0) / b if b else float('nan'),
              (a - 26.0) / b if b else float('nan'),
              (a - 55.0) / b if b else float('nan')), file=stream)
    if fit['rms'] > 0.5:
        print('\n注意：RMS {:.2f} mm 偏大，要么量得粗，要么中间卡了东西、'
              '要么行程两端已经顶到限位（那里不再线性）'.format(fit['rms']),
              file=stream)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='拟合夹爪的圈数↔开口')
    parser.add_argument('csv', nargs='?', help='gripper_sweep.py 生成的 CSV')
    parser.add_argument('--inline', help='"圈数,开口;圈数,开口"')
    args = parser.parse_args(argv)

    if not args.csv and not args.inline:
        parser.error('要么给一个 CSV，要么用 --inline')
    try:
        points = (parse_inline(args.inline) if args.inline
                  else read_csv(args.csv))
        fit = fit_width(points)
    except (ValueError, OSError) as exc:
        print('{}'.format(exc))
        return 2
    return report(fit)


if __name__ == '__main__':
    sys.exit(main())
