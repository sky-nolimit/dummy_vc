#!/usr/bin/env python3
"""量一次夹爪的合拢点，并打印由它推出来的对照表。

    python3 tools/gripper_home.py              # 真干（需要 can0）
    python3 tools/gripper_home.py --dry-run    # 只打印要发什么

## 这个工具解决的问题

`gripper_frames` 里那条 `开口 = 34.294 − 11.620 × 圈数` 的截距跟着**编码器零点**
走，而零点会因为"重新标定一次"就平移（实测平移过 0.22 圈 ≈ 2.5 mm）。所以
"发 0.9 圈去夹 24 mm 方块"这件事，只有在**起点就是那次标定时的位置**时才成立。

这个工具不假设起点：它把手指一路推到合拢（机械止位），量出那一刻的圈数，之后
所有宽度都按"离合拢点多少圈"算。测完会打印新截距和 0/22/24/26/50/55 mm 各对应
多少圈，供你核对。

## 安全

* 两头都是"位置连续几次不动就停"，然后立刻发停；
* **两头的距离**必须和机械行程（55 mm ≈ 4.73 圈）对得上，否则拒绝 —— 这是为了
  排除"两指之间卡着东西"（那种情况下合拢止位会被量成"方块止位"，看起来很正
  常）。行程是机械常数，与编码器零点无关；
* 量完自动张到 `--open-to`（默认 50 mm）待命。
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
if GRASP not in sys.path:
    sys.path.insert(0, GRASP)

from dummy_vc_grasp import gripper_frames as gf       # noqa: E402
from dummy_vc_grasp import gripper_home as gh         # noqa: E402

#: 打印对照表用的宽度（mm）
SHOW_WIDTHS = (0.0, 22.0, 24.0, 26.0, 50.0, 55.0)


def make_send(sock, dry_run):
    """发一帧，不逐帧打印（找零点时一秒要盯 20 次）。"""
    def _send(frame_id, payload):
        if dry_run:
            print('（只打印）{}'.format(gf.describe(frame_id, payload)))
            return
        sock.send(gf.to_socketcan(frame_id, payload))
    return _send


def make_reader(sock, read_one, timeout=0.3):
    """读一次位置：先发查询帧，再等回帧。读不到返回 None。"""
    def _read():
        try:
            sock.send(gf.to_socketcan(gf.POSITION_FEEDBACK_ID,
                                      gf.encode_position_query()))
        except OSError:
            return None
        got = read_one(sock, timeout)
        return None if got is None else got[0]
    return _read


def print_table(rev_closed):
    print('')
    print('以这个合拢点为原点：')
    print('  {:>8}  {:>10}'.format('开口 mm', '位置 圈'))
    for width in SHOW_WIDTHS:
        print('  {:>8.1f}  {:>10.4f}'.format(
            width, gf.width_to_revs(width, rev_closed)))
    print('')
    print('改写这两行（dummy_vc_ws/src/dummy_vc_grasp/dummy_vc_grasp/'
          'gripper_frames.py）：')
    print('  REV_AT_ZERO_WIDTH_MM = {:.3f}'.format(rev_closed * gf.MM_PER_REV))
    print('  MM_PER_REV = {:.3f}   # 没重测就别动，这是齿条节距决定的'.format(
        gf.MM_PER_REV))
    print('')
    print('不过真正让它可靠的是"每次用之前量一次"（节点里 home_on_start，或者'
          '这个工具）；上面两行只是让"没量过的时候"有个像样的默认值。')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--channel', default='can0', help='CAN 接口，默认 can0')
    parser.add_argument('--span', type=float, default=gh.DEFAULT_SPAN_REVS,
                        help='命令目标超过合拢点多少圈，默认 %(default)s')
    parser.add_argument('--timeout', type=float, default=gh.DEFAULT_TIMEOUT_S,
                        help='最多盯多久，默认 %(default)s 秒')
    parser.add_argument('--rate', type=float, default=gh.DEFAULT_RATE_HZ,
                        help='采样频率，默认 %(default)s Hz')
    parser.add_argument('--plausible', type=float, default=None,
                        help='（旧参数，已废弃：改用 --stroke-tol）')
    parser.add_argument('--stroke-tol', type=float,
                        default=gh.DEFAULT_STROKE_TOL_REVS,
                        help='两头行程与机械行程差多少圈就拒绝，默认 %(default)s')
    parser.add_argument('--open-to', type=float, default=50.0,
                        help='量完之后张到多宽，默认 %(default)s mm')
    parser.add_argument('--dry-run', action='store_true',
                        help='只打印要发的帧，不碰硬件（量不出东西）')
    args = parser.parse_args(argv)

    if args.dry_run:
        guess = gf.DEFAULT_REV_CLOSED
        print('（只打印）先往合拢推：目标 {:.4f} 圈（超过止位，位置会停住）'
              .format(max(guess, guess) + args.span))
        print('（只打印）{}'.format(
            gf.describe(gf.ENABLE_FRAME_ID, gf.encode_enable(True))))
        print('（只打印）{}'.format(
            gf.describe(gf.POSITION_FRAME_ID,
                        gf.encode_position(guess + args.span))))
        print('（只打印）再往张开推：目标 = 合拢点 − {:.1f} 圈'.format(args.span))
        print('（只打印）然后校验两头距离 ≈ {:.2f} 圈（机械行程 {:.1f} mm）'.format(
            gh.DEFAULT_STROKE_REVS, gh.DEFAULT_STROKE_REVS * gf.MM_PER_REV))
        print_table(guess)
        return 0

    from gripper_cli import open_socket, read_one

    sock = open_socket(args.channel)
    result = gh.home(
        make_send(sock, False), make_reader(sock, read_one),
        span_revs=args.span, timeout_s=args.timeout, rate_hz=args.rate,
        stroke_tol_revs=args.stroke_tol, log=print)

    if not result.ok:
        print('')
        print('没量成：{}'.format(result.reason))
        sock.close()
        return 4 if 'read' in result.reason or '读不到' in result.reason else 2

    print('')
    print('合拢点 {:.4f} 圈、张开点 {:.4f} 圈'.format(
        result.rev_closed, result.rev_open))
    print('行程 {:.3f} 圈 ≈ {:.1f} mm（机械 {:.1f} mm，差 {:+.1f} mm）'.format(
        result.span_revs, result.span_revs * gf.MM_PER_REV,
        gh.DEFAULT_STROKE_REVS * gf.MM_PER_REV,
        (result.span_revs - gh.DEFAULT_STROKE_REVS) * gf.MM_PER_REV))
    print_table(result.rev_closed)

    width, clamped = gf.clamp_width(args.open_to)
    if clamped:
        print('要张到 {:.1f} mm，超出机械行程，按 {:.1f} mm 发'.format(
            args.open_to, width))
    try:
        sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID,
                                  gf.encode_enable(True)))
        sock.send(gf.to_socketcan(
            gf.POSITION_FRAME_ID,
            gf.encode_position(gf.width_to_revs(width, result.rev_closed))))
        print('已张到 {:.1f} mm（离开合拢点，别让它顶着）'.format(width))
    except OSError as exc:
        print('张开的命令没发出去：{}'.format(exc))
    sock.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
