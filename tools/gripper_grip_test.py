#!/usr/bin/env python3
"""单独联调：把方块放在两指之间，看夹爪能不能真的夹住。

它不看抓取流程、不碰机械臂，只做四件事：

    1. 读一次当前开口；
    2. 张开到一个位置（默认 45 mm）；
    3. 记一段"空载电流"（这时候手指没碰到任何东西）；
    4. 闭合到比方块窄一点的位置（默认夹 22 mm，方块 24 mm），
       然后连续采样位置和电流 —— 手指被方块挡住时，位置到不了目标、
       电流明显高于空载，这两条同时成立才算"夹住"。

**电流的单位没标定过**（固件回的是 `GetFocCurrent()`），所以这里只比大小、
不当安培读数看。空载与受力两次都是同一台板子、同一个动作量的，比值有意义。

用法：

    python3 tools/gripper_grip_test.py                  # 方块 24 mm，张开 45、夹到 22
    python3 tools/gripper_grip_test.py --grip 30 --object 32
    python3 tools/gripper_grip_test.py --dry-run        # 只打印要发什么

跑完它会把测到的数摆出来，然后提示你用手轻拉方块验证。
"""
import argparse
import os
import socket
import statistics
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
        sys.exit('这个解释器没有 SocketCAN。用 --dry-run 看要发什么。')
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((channel,))
    except OSError as exc:
        sock.close()
        sys.exit('绑不上 {}（{}）。先 `sudo ip link set {} up type can '
                 'bitrate 1000000`。'.format(channel, exc, channel))
    sock.settimeout(0.2)
    return sock


def ask(sock, frame_id, payload, want, timeout):
    """发一帧请求，等 `want` 这个 ID 的回帧，返回 8 字节负载。"""
    sock.send(gf.to_socketcan(frame_id, payload))
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
        got_id, _dlc, got = gf.CAN_FRAME.unpack(data)
        if got_id == want:
            return got
    return None


def read_position(sock, timeout):
    payload = ask(sock, gf.POSITION_FEEDBACK_ID, gf.encode_position_query(),
                  gf.POSITION_FEEDBACK_ID, timeout)
    return None if payload is None else gf.decode_position(payload)


def read_current(sock, timeout):
    payload = ask(sock, gf.CURRENT_FEEDBACK_ID, gf.encode_current_query(),
                  gf.CURRENT_FEEDBACK_ID, timeout)
    return None if payload is None else gf.decode_current(payload)


def move(sock, width_mm, settle):
    """使能 + 走到指定开口，然后等它停稳。"""
    revs = gf.width_to_revs(width_mm)
    sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID, gf.encode_enable(True)))
    sock.send(gf.to_socketcan(gf.POSITION_FRAME_ID, gf.encode_position(revs)))
    time.sleep(settle)
    return revs


def sample(sock, seconds, hz=5.0):
    """连续采 (开口, 电流, 到位)，用来判断"停住了没有/在不在用力"。"""
    rows = []
    deadline = time.time() + seconds
    while time.time() < deadline:
        pos = read_position(sock, 0.5)
        cur = read_current(sock, 0.5)
        if pos is not None and cur is not None:
            rows.append((gf.revs_to_width(pos[0]), cur[0], pos[1]))
        time.sleep(1.0 / hz)
    return rows


def report(rows):
    if not rows:
        return None
    widths = [w for w, _, _ in rows]
    currents = [abs(c) for _, c, _ in rows]
    return {
        'width_median': statistics.median(widths),
        'width_spread': max(widths) - min(widths),
        'current_median': statistics.median(currents),
        'current_max': max(currents),
        'finished': sum(1 for _, _, f in rows if f),
        'samples': len(rows),
    }


def run(args):
    sock = open_socket(args.channel)
    try:
        now = read_position(sock, args.timeout)
        if now is None:
            print('读不到位置：查询已发，板子没答。先确认 can0 UP、夹爪板通电。')
            return 2
        print('当前开口 {:.1f} mm（{:.4f} 圈）'.format(
            gf.revs_to_width(now[0]), now[0]))

        print('\n[1] 张开到 {:.1f} mm'.format(args.open))
        move(sock, args.open, args.settle)
        idle = report(sample(sock, args.baseline_s))
        if idle is None:
            print('    张开后读不到回帧，停在这里')
            return 2
        print('    空载：开口 {:.1f} mm，电流中位数 {:.4f}，到位 {}/{} 次'.format(
            idle['width_median'], idle['current_median'],
            idle['finished'], idle['samples']))

        print('\n[2] 闭合到 {:.1f} mm（方块 {:.1f} mm，应该被挡住）'.format(
            args.grip, args.object))
        move(sock, args.grip, args.settle)
        pressed = report(sample(sock, args.hold_s))
        if pressed is None:
            print('    闭合后读不到回帧')
            return 2
        print('    受力：开口 {:.1f} mm（抖动 {:.1f}），电流中位数 {:.4f}，到位 {}/{} 次'
              .format(pressed['width_median'], pressed['width_spread'],
                      pressed['current_median'], pressed['finished'],
                      pressed['samples']))

        blocked = pressed['width_median'] - args.grip
        print('\n判定')
        print('    目标 {:.1f} mm，实际停在 {:.1f} mm ⇒ 被挡住 {:.1f} mm'.format(
            args.grip, pressed['width_median'], blocked))
        if idle['current_median'] > 1e-9:
            print('    电流比值 {:.2f}×（受力 / 空载）'.format(
                pressed['current_median'] / idle['current_median']))
        else:
            print('    空载电流是 0，比值算不了，只看绝对值和位置')
        if blocked > args.block_mm:
            print('\n⇒ 手指被东西挡住了，位置到不了目标。')
            print('   现在用手轻轻拉一下方块：')
            print('   · 拉不动、电流还稳 ⇒ 夹住了；')
            print('   · 一拉就滑、位置跳变 ⇒ 没夹住（力度不够或指面打滑）。')
        else:
            print('\n⇒ 手指到目标位置了，两指之间没有东西（空夹）。')
            print('   方块是不是太窄/没放进去？或者目标比方块还宽。')
    finally:
        print('\n松开：python3 tools/gripper_cli.py open 45')
        sock.close()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='夹爪单独联调：能不能夹住')
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--open', type=float, default=45.0, help='先张到多开')
    parser.add_argument('--grip', type=float, default=22.0, help='闭合到多宽')
    parser.add_argument('--object', type=float, default=24.0, help='方块多宽')
    parser.add_argument('--settle', type=float, default=1.2, help='每步等多久')
    parser.add_argument('--timeout', type=float, default=1.0)
    parser.add_argument('--baseline-s', type=float, default=1.0, help='空载采多久')
    parser.add_argument('--hold-s', type=float, default=2.5, help='受力采多久')
    parser.add_argument('--block-mm', type=float, default=1.0,
                        help='实际比目标宽多少才算"被挡住"')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args(argv)

    if args.grip >= args.object:
        parser.error('闭合目标 {:.1f} mm 不能比方块 {:.1f} mm 宽，那样夹不到'.format(
            args.grip, args.object))

    if args.dry_run:
        for label, width in (('张开', args.open), ('闭合', args.grip)):
            try:
                revs = gf.width_to_revs(width)
            except gf.OutOfRange as exc:
                print('{} 到 {:.1f} mm 不行：{}'.format(label, width, exc))
                return 2
            print('（只打印）{}：0x{:03X} 使能 + 0x{:03X} 位置 {:.4f} 圈'
                  .format(label, gf.ENABLE_FRAME_ID, gf.POSITION_FRAME_ID, revs))
        return 0

    return run(args)


if __name__ == '__main__':
    sys.exit(main())
