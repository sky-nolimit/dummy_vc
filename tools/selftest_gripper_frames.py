#!/usr/bin/env python3
"""离线自查：夹爪 CAN 帧的编解码，以及"节点有没有绕开这一层自己拼字节"。

它要防的故障是这一类：

    字节摆错了（大小端、补零、帧 ID 拼错），可板子上什么都不说 ——
    夹爪不动，和"没供电""板子不认这套协议"看起来一模一样

真板子要等到 USB-CAN 的线到了才能试，而"字节对不对"是纯算术，在这里就能把
每一种情况钉死。另外两项也不是凑数：

  * SocketCAN 那 16 字节的 `struct can_frame`（id/dlc/对齐）最容易写错，
    错了 `send()` 不报错，只是发出去一帧没人认
  * 节点必须走 `gripper_frames` 这一层。节点里自己写 `0x382` 就意味着有两份
    定义，改了一处忘了另一处，症状同上

    python tools/selftest_gripper_frames.py
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
NODE = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_can_node.py')

#: 纯层允许 import 的东西。多一个 ROS/socket 进来，这个自测就没了。
PURE_IMPORTS = {'math', 'struct'}

FAILURES = []


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def same(label, got, want):
    if got == want:
        ok(label)
        return True
    fail(f'{label}: 拿到 {got!r}，应该是 {want!r}')
    return False


def expect_error(label, exc_types, func, *args):
    try:
        func(*args)
    except exc_types as exc:
        ok(f'{label}（{type(exc).__name__}）')
        return True
    except Exception as exc:            # noqa: BLE001 - 报出来比咽下去强
        fail(f'{label}: 抛了意料外的 {type(exc).__name__}: {exc}')
        return False
    fail(f'{label}: 没有拒绝，返回了一个看起来正常的值')
    return False


def load_frames():
    if GRASP not in sys.path:
        sys.path.insert(0, GRASP)
    try:
        from dummy_vc_grasp import gripper_frames as gf
    except ImportError as exc:
        fail(f'import 失败：{exc}（这一层只该依赖 math/struct）')
        return None
    return gf


def check_pure_layer():
    print('\n0. 纯层只依赖 math / struct')
    path = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_frames.py')
    try:
        with open(path, encoding='utf-8') as fh:
            tree = ast.parse(fh.read())
    except OSError as exc:
        fail(f'读不到 gripper_frames.py：{exc}')
        return
    bad = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods = [a.name.split('.')[0] for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            mods = [(node.module or '').split('.')[0]]
        else:
            continue
        bad += [m for m in mods if m not in PURE_IMPORTS]
    same('gripper_frames.py 的 import', sorted(set(bad)), [])


def check_ids(gf):
    print('\n1. 帧 ID 与参考固件一致（node 7、mode 1/2）')
    same('电流上限帧 0x381', gf.CURRENT_LIMIT_FRAME_ID, 0x381)
    same('角度帧 0x382', gf.ANGLE_FRAME_ID, 0x382)
    same('负载长度 8', gf.DLC, 8)


def check_payload(gf):
    print('\n2. 负载就是 float32 小端 + 4 字节补零')
    same('5.0 度', gf.encode_angle(5.0).hex(' '), '00 00 a0 40 00 00 00 00')
    same('10.0 度', gf.encode_angle(10.0).hex(' '), '00 00 20 41 00 00 00 00')
    same('0.0 度', gf.encode_angle(0.0).hex(' '), '00 00 00 00 00 00 00 00')
    same('0.5 A', gf.encode_current(0.5).hex(' '), '00 00 00 3f 00 00 00 00')
    same('回读 10.0', gf.decode(gf.encode_angle(10.0)), 10.0)
    same('回读 0.25 A', gf.decode(gf.encode_current(0.25)), 0.25)


def check_socketcan(gf):
    print('\n3. SocketCAN 的 16 字节 can_frame：id / dlc / 对齐')
    frame = gf.to_socketcan(gf.ANGLE_FRAME_ID, gf.encode_angle(5.0))
    same('总长 16', len(frame), 16)
    # id 小端 4 字节、dlc 1 字节、3 字节对齐、然后 8 字节负载
    same('id = 82 03 00 00', frame[:4].hex(' '), '82 03 00 00')
    same('dlc = 08', frame[4:5].hex(), '08')
    same('中间 3 字节对齐补零', frame[5:8].hex(), '000000')
    same('负载跟在后面', frame[8:].hex(' '), '00 00 a0 40 00 00 00 00')
    expect_error('负载长度不对要拒绝', ValueError,
                 gf.to_socketcan, gf.ANGLE_FRAME_ID, b'\x00' * 4)
    expect_error('扩展帧 ID 要拒绝', ValueError,
                 gf.to_socketcan, 0x800, gf.encode_angle(0.0))


def check_rejections(gf):
    print('\n4. 越界与非法值：抛异常，不静默夹紧')
    same('0 度可以', len(gf.encode_angle(0.0)), 8)
    same('30 度可以', len(gf.encode_angle(gf.MAX_ANGLE_DEG)), 8)
    same('1.0 A 可以', len(gf.encode_current(1.0)), 8)
    expect_error('30.1 度', gf.OutOfRange, gf.encode_angle, 30.1)
    expect_error('-0.1 度', gf.OutOfRange, gf.encode_angle, -0.1)
    expect_error('NaN 度', gf.OutOfRange, gf.encode_angle, float('nan'))
    expect_error('inf 度', gf.OutOfRange, gf.encode_angle, float('inf'))
    expect_error('1.5 A', gf.OutOfRange, gf.encode_current, 1.5)
    expect_error('bool 不是数字', TypeError, gf.encode_angle, True)
    expect_error('字符串不是数字', TypeError, gf.encode_angle, '5')
    expect_error('4 字节负载解码要拒绝', ValueError, gf.decode, b'\x00' * 4)


def check_clamp(gf):
    print('\n5. 夹紧要留痕：返回 (值, 有没有夹)')
    same('7.5 度不用夹', gf.clamp_angle(7.5), (7.5, False))
    same('-1 度夹到 0', gf.clamp_angle(-1.0), (0.0, True))
    same('40 度夹到 30', gf.clamp_angle(40.0), (30.0, True))
    expect_error('NaN 夹不了，要拒绝', gf.OutOfRange, gf.clamp_angle, float('nan'))


def check_node_uses_frames():
    print('\n6. 节点不许自己拼字节，也不许漏进 setup.py')
    try:
        with open(NODE, encoding='utf-8') as fh:
            source = fh.read()
    except OSError as exc:
        fail(f'读不到 gripper_can_node.py：{exc}')
        return
    tree = ast.parse(source)
    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.module:
                imported.add(node.module)
            # `from dummy_vc_grasp import gripper_frames as gf` 的模块名在 alias 里
            imported.update(a.name for a in node.names)
        elif isinstance(node, ast.Import):
            imported.update(a.name for a in node.names)
    same('节点 import 了 gripper_frames', 'dummy_vc_grasp.gripper_frames' in imported
         or any('gripper_frames' in m for m in imported), True)

    hardcoded = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, int) \
                and node.value in (0x381, 0x382):
            hardcoded.append(node.value)
    same('节点里没有写死的 0x381/0x382', hardcoded, [])

    try:
        with open(os.path.join(GRASP, 'setup.py'), encoding='utf-8') as fh:
            setup_src = fh.read()
    except OSError as exc:
        fail(f'读不到 setup.py：{exc}')
        return
    same('setup.py 注册了 gripper_can_node',
         'gripper_can_node = dummy_vc_grasp.gripper_can_node:main' in setup_src,
         True)


def main():
    gf = load_frames()
    check_pure_layer()
    if gf is not None:
        check_ids(gf)
        check_payload(gf)
        check_socketcan(gf)
        check_rejections(gf)
        check_clamp(gf)
    check_node_uses_frames()

    print()
    if FAILURES:
        print(f'FAILED ({len(FAILURES)} problem(s))')
        for msg in FAILURES:
            print(f'  - {msg}')
        return 1
    print('SUMMARY')
    print('  [ ok ] gripper_frames.py      frame ids and float32 payload match the firmware')
    print('  [ ok ] gripper_can_node.py    goes through the codec, and is registered')
    print('RESULT: every byte of the gripper frame is pinned down offline')
    return 0


if __name__ == '__main__':
    sys.exit(main())
