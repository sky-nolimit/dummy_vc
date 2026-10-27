#!/usr/bin/env python3
"""夹爪的最小命令行：只发合法帧，0x382 在这里根本发不出去。

为什么要有它：真机上调试靠手打 `cansend can0 382#0000000000000000` 太容易出事。
0x382 在这块板上是**触发编码器标定**，载荷是什么都不影响，发一次就可能把夹爪
的标定表弄坏（位置模式从此不输出、回读冻住，只能断电重启）。2026-10-26 和 27 日
各踩过一次，两次都是手打的。

用法（在能访问 can0 的机器上，Ubuntu 虚拟机里就行）：

    python3 tools/gripper_cli.py read                 # 读一次位置/开口
    python3 tools/gripper_cli.py enable               # 发使能
    python3 tools/gripper_cli.py open 30              # 开口开到 30 mm
    python3 tools/gripper_cli.py revs 0.25            # 直接给电机圈数
    python3 tools/gripper_cli.py watch                # 只听 3 秒，解码每帧

    python3 tools/gripper_cli.py --dry-run open 30    # 不碰硬件，只打印要发什么

约定：凡是"发出去会改状态"的动作都在这里显式写出来，不给 `raw` 直通口子 ——
直通口子一开，防 0x382 这件事就白做了。
"""
import argparse
import os
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
if GRASP not in sys.path:
    sys.path.insert(0, GRASP)

from dummy_vc_grasp import gripper_frames as gf   # noqa: E402


def open_socket(channel):
    if not hasattr(socket, 'AF_CAN'):
        sys.exit('这个解释器没有 SocketCAN（不是 Linux？）。先用 --dry-run 看要发什么。')
    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    except OSError as exc:
        sys.exit('建不出 CAN 套接字：{}'.format(exc))
    try:
        sock.bind((channel,))
    except OSError as exc:
        sock.close()
        sys.exit('绑不上 {}（{}）。先 `sudo ip link set {} up type can '
                 'bitrate 1000000`。'.format(channel, exc, channel))
    sock.settimeout(0.3)
    return sock


def send(sock, frame_id, payload, dry_run):
    text = gf.describe(frame_id, payload)
    if dry_run:
        print('（只打印）{}'.format(text))
        return
    sock.send(gf.to_socketcan(frame_id, payload))
    print('发出 {}'.format(text))


def read_one(sock, timeout):
    """等一帧 0x3A3，解出圈数/开口。超时返回 None。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            data = sock.recv(gf.CAN_FRAME.size)
        except socket.timeout:
            continue
        except OSError:
            return None
        if len(data) < gf.CAN_FRAME.size:
            continue
        frame_id, _dlc, payload = gf.CAN_FRAME.unpack(data)
        if frame_id != gf.POSITION_FEEDBACK_ID:
            continue
        revs, finished = gf.decode_position(payload)
        return revs, gf.revs_to_width(revs), finished
    return None


def cmd_read(args, sock):
    send(sock, gf.POSITION_FEEDBACK_ID,
         gf.encode_position_query(), args.dry_run)
    if args.dry_run:
        return 0
    got = read_one(sock, args.timeout)
    if got is None:
        print('{} 秒内没等到 0x{:03X} 回帧。板子没上电？can0 没 up？'
              .format(args.timeout, gf.POSITION_FEEDBACK_ID))
        return 2
    revs, width, finished = got
    print('位置 {:.4f} 圈 = 开口 {:.1f} mm{}'.format(
        revs, width, '（到位）' if finished else ''))
    return 0


def cmd_enable(args, sock):
    send(sock, gf.ENABLE_FRAME_ID, gf.encode_enable(True),
         args.dry_run)
    return 0


def _move(args, sock, width_mm):
    width, clamped = gf.clamp_width(width_mm)
    if clamped:
        print('警告：{:.2f} mm 超出 {}~{} mm，按 {:.2f} mm 发'.format(
            width_mm, gf.MIN_WIDTH_MM, gf.MAX_WIDTH_MM, width))
    revs = gf.width_to_revs(width)
    # 先使能：主控的 power_on 会发一帧 0x381 带 float，板子判 uint32 != 1 →
    # MODE_STOP，所以每次动之前都补一帧使能，不依赖之前那次。
    send(sock, gf.ENABLE_FRAME_ID, gf.encode_enable(True),
         args.dry_run)
    send(sock, gf.POSITION_FRAME_ID, gf.encode_position(revs),
         args.dry_run)
    if args.dry_run:
        return 0
    time.sleep(args.settle)
    got = read_one(sock, args.timeout)
    if got is None:
        print('没等到回帧，看一下 candump 里 0x{:03X} 有没有出来'
              .format(gf.POSITION_FEEDBACK_ID))
        return 2
    revs_now, width_now, finished = got
    print('到位读数 {:.4f} 圈 = 开口 {:.1f} mm{}（目标 {:.1f} mm，差 {:+.1f}）'
          .format(revs_now, width_now, '（到位）' if finished else '',
                  width, width_now - width))
    return 0


def cmd_open(args, sock):
    return _move(args, sock, args.mm)


def cmd_park(args, sock):
    """闭合待命：合拢到 0 mm，然后发一帧停（不长期保持力矩）。"""
    code = _move(args, sock, 0.0)
    send(sock, gf.ENABLE_FRAME_ID, gf.encode_enable(False), args.dry_run)
    if code == 0 and not args.dry_run:
        print('已闭合，并发了停（手指现在不受力）')
    return code


def cmd_revs(args, sock):
    send(sock, gf.ENABLE_FRAME_ID, gf.encode_enable(True),
         args.dry_run)
    send(sock, gf.POSITION_FRAME_ID,
         gf.encode_position(args.revs), args.dry_run)
    return 0


def cmd_watch(args, sock):
    """只听不发：屏幕上滚动解码，用来确认"板子在说话"。"""
    if args.dry_run:
        print('watch 不需要 dry_run')
        return 0
    deadline = time.time() + args.seconds
    seen = 0
    while time.time() < deadline:
        try:
            data = sock.recv(gf.CAN_FRAME.size)
        except socket.timeout:
            continue
        except OSError:
            return 2
        if len(data) < gf.CAN_FRAME.size:
            continue
        frame_id, dlc, payload = gf.CAN_FRAME.unpack(data)
        seen += 1
        label = '夹爪' if frame_id == gf.POSITION_FEEDBACK_ID else '其他节点'
        try:
            text = gf.describe(frame_id, payload)
        except ValueError as exc:
            text = '解不开：{}'.format(exc)
        print('0x{:03X} [{}] {}  {}'.format(frame_id, dlc, label, text))
    print('{} 秒内收到 {} 帧'.format(args.seconds, seen))
    if seen == 0:
        print('一帧都没有：can0 没 up、没接终端电阻，或者这块板没在总线上。')
        return 2
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='夹爪 CAN 命令行（0x382 已封死）')
    parser.add_argument('--channel', default='can0', help='CAN 接口名，默认 can0')
    parser.add_argument('--dry-run', action='store_true', help='只打印要发的帧')
    parser.add_argument('--timeout', type=float, default=1.0, help='等回帧的秒数')
    parser.add_argument('--settle', type=float, default=0.8,
                        help='发完位置后等多久再读（默认 0.8 s）')
    sub = parser.add_subparsers(dest='cmd', required=True)

    sub.add_parser('read', help='读一次位置')
    sub.add_parser('enable', help='发一帧使能')
    sub.add_parser('park', help='闭合到 0 mm 待命，然后发停')
    p_open = sub.add_parser('open', help='开口开到多少毫米')
    p_open.add_argument('mm', type=float)
    p_revs = sub.add_parser('revs', help='直接给电机圈数')
    p_revs.add_argument('revs', type=float)
    p_watch = sub.add_parser('watch', help='只听不发')
    p_watch.add_argument('--seconds', type=float, default=3.0)

    args = parser.parse_args(argv)
    sock = None if args.dry_run else open_socket(args.channel)
    try:
        handler = {'read': cmd_read, 'enable': cmd_enable, 'park': cmd_park,
                   'open': cmd_open,
                   'revs': cmd_revs, 'watch': cmd_watch}[args.cmd]
        return handler(args, sock)
    finally:
        if sock is not None:
            sock.close()


if __name__ == '__main__':
    sys.exit(main())
