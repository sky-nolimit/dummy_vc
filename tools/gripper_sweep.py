#!/usr/bin/env python3
"""让夹爪自己走到一串位置上，每停一下问你要卡尺读数，最后写成 CSV。

为什么要工具：标定之后 `encoderHomeOffset` 变了，旧的"圈数↔开口"三点作废，
而重测需要在十几个位置上来回量——手打 `cansend` 再手工对表，错一个数就白测。

它只发合法的 `0x381` 使能帧和 `0x385` 位置帧（走 `gripper_frames.to_socketcan`，
所以 `0x382` 那个标定帧在这里根本发不出去）。

用法：

    # 先看要走的点，不碰硬件
    python3 tools/gripper_sweep.py --plan

    # 从当前位置往"开"的方向走（圈数变小 = 开口变大）
    python3 tools/gripper_sweep.py --revs 0,-0.25,-0.5,-0.75,-1.0,-1.5,-2.0,-2.5

    # 往"闭"的方向走
    python3 tools/gripper_sweep.py --revs 0,0.25,0.5,0.75,1.0,1.25 --out close.csv

卡尺量的是**两指内侧的净开口**，每次都量同一个位置（比如指尖上方 5 mm 处）。
量不到就回车跳过，那一行会留空，拟合时自动忽略。
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

#: 从当前位置往"开"的方向，覆盖 13 → 55 mm 附近
DEFAULT_OPEN = '0,-0.25,-0.5,-0.75,-1.0,-1.5,-2.0,-2.5,-3.0'
#: 往"闭"的方向，覆盖 13 → 0 mm 附近
DEFAULT_CLOSE = '0,0.25,0.5,0.75,1.0,1.25'


def parse_revs(text):
    revs = [float(chunk) for chunk in text.replace(' ', '').split(',') if chunk]
    if not revs:
        raise ValueError('--revs 是空的')
    return revs


def open_socket(channel):
    if not hasattr(socket, 'AF_CAN'):
        sys.exit('这个解释器没有 SocketCAN。用 --plan 看要走的点。')
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((channel,))
    except OSError as exc:
        sock.close()
        sys.exit('绑不上 {}（{}）。先 `sudo ip link set {} up type can '
                 'bitrate 1000000`。'.format(channel, exc, channel))
    sock.settimeout(0.3)
    return sock


def have_frame(sock):
    """收一帧，是 0x3A3 就返回 (圈数, 到位)，否则 None。"""
    try:
        data = sock.recv(gf.CAN_FRAME.size)
    except socket.timeout:
        return None
    except OSError:
        return None
    if len(data) < gf.CAN_FRAME.size:
        return None
    frame_id, _dlc, payload = gf.CAN_FRAME.unpack(data)
    if frame_id != gf.POSITION_FEEDBACK_ID:
        return None
    return gf.decode_position(payload)


def read_position(sock, timeout):
    """等一帧位置回读。超时返回 None（板子没在说话）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        got = have_frame(sock)
        if got is not None:
            return got
    return None


def move_to(sock, revs):
    sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID, gf.encode_enable(True)))
    sock.send(gf.to_socketcan(gf.POSITION_FRAME_ID, gf.encode_position(revs)))


def ask_width(target_revs):
    """问卡尺读数。回车 = 跳过，Ctrl-C 退出整轮。"""
    while True:
        try:
            text = input('  实测开口(mm)，回车跳过：').strip()
        except EOFError:
            return None
        if not text:
            return None
        try:
            value = float(text)
        except ValueError:
            print('  这不是一个数：{!r}'.format(text))
            continue
        if not 0.0 <= value <= 200.0:
            print('  0~200 mm 之外，八成打错了')
            continue
        return value


def sweep(args):
    sock = open_socket(args.channel)
    rows = []
    try:
        now = read_position(sock, args.timeout)
        if now is None:
            print('一帧 0x3A3 都没收到：板子没上电 / can0 没 up / 没接终端电阻。')
            return 2
        print('当前位置 {:.4f} 圈 = 开口约 {:.1f} mm'.format(
            now[0], gf.revs_to_width(now[0])))
        sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID, gf.encode_enable(True)))
        print('已发使能。开始走点（随时 Ctrl-C 停，已经量过的不丢）\n')
        print('{:>10} {:>10} {:>8} {:>10}'.format('目标圈数', '回读圈数', '到位', '开口'))
        print('-' * 44)
        for target in args.revs:
            move_to(sock, target)
            time.sleep(args.settle)
            got = read_position(sock, args.timeout)
            if got is None:
                print('{:>10.4f} {}'.format(target, '没等到回读'))
                continue
            revs, finished = got
            width = gf.revs_to_width(revs)
            print('{:>10.4f} {:>10.4f} {:>8} {:>10.1f}'.format(
                target, revs, '是' if finished else '否', width))
            if abs(revs - target) > args.slack:
                print('           ↳ 差了 {:.2f} 圈没走到：多半顶到限位了'
                      .format(revs - target))
            measured = None if args.no_ask else ask_width(target)
            rows.append((target, revs, measured, finished))
    except KeyboardInterrupt:
        print('\n中断了，把已经量到的写下来')
    finally:
        sock.close()

    if args.out:
        with open(args.out, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write('# gripper_sweep.py 生成：target_revs,readback_revs,'
                     'width_mm,finished\n')
            for target, revs, measured, finished in rows:
                fh.write('{:.4f},{:.4f},{},{}\n'.format(
                    target, revs,
                    '' if measured is None else '{:.2f}'.format(measured),
                    1 if finished else 0))
        usable = [r for r in rows if r[2] is not None]
        print('\n写进 {}：{} 行，其中 {} 行有开口读数'.format(
            args.out, len(rows), len(usable)))
        if len(usable) >= 2:
            print('下一步：python3 tools/fit_gripper_width.py {}'.format(args.out))
        else:
            print('至少量两个点才能拟合。')
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='夹爪位置扫描 + 记卡尺读数')
    parser.add_argument('--channel', default='can0')
    parser.add_argument('--revs', default=DEFAULT_OPEN,
                        help='逗号分隔的电机圈数，默认往开的方向：' + DEFAULT_OPEN)
    parser.add_argument('--close', action='store_true',
                        help='用往闭方向的默认序列：' + DEFAULT_CLOSE)
    parser.add_argument('--out', default='gripper_width.csv')
    parser.add_argument('--settle', type=float, default=0.9, help='每步等多久')
    parser.add_argument('--timeout', type=float, default=1.0, help='等回读的秒数')
    parser.add_argument('--slack', type=float, default=0.1,
                        help='回读与目标的差超过这个圈数就提示顶限位')
    parser.add_argument('--no-ask', action='store_true', help='只走点不问读数')
    parser.add_argument('--plan', action='store_true', help='只打印计划')
    args = parser.parse_args(argv)

    if args.close and args.revs == DEFAULT_OPEN:
        args.revs = DEFAULT_CLOSE
    args.revs = parse_revs(args.revs)

    if args.plan:
        print('要走 {} 个点：'.format(len(args.revs)))
        for target in args.revs:
            print('  {:>8.4f} 圈 → 开口约 {:>5.1f} mm（按旧标定，仅作参考）'
                  .format(target, gf.revs_to_width(target)))
        print('\n每个点停下来后量两指内侧的净开口，回来输入。')
        return 0

    return sweep(args)


if __name__ == '__main__':
    sys.exit(main())
