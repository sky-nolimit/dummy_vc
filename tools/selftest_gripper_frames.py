#!/usr/bin/env python3
"""离线自查：夹爪 CAN 帧的编解码、宽度↔圈数换算，以及节点有没有绕开这一层。

它要防的故障是这一类：

    字节摆错了（大小端、补零、帧 ID 拼错），板子上什么都不说 ——
    夹爪不动，和"没供电""没标定""板子不认这套协议"看起来一模一样

2026-10-26 这份自查跟着 `gripper_frames.py` 一起重写：原来钉的 0x382（角度）在
真板子上是**触发标定**，按下去会把标定表擦掉、位置模式从此不输出。现在钉的是
35 板固件真正的三帧：0x381 使能、0x385 位置、0x3A3 回读。

另外两项也不是凑数：

  * SocketCAN 那 16 字节的 `struct can_frame`（id/dlc/对齐）最容易写错，
    错了 `send()` 不报错，只是发出去一帧没人认
  * 节点必须走 `gripper_frames` 这一层。节点里自己写 `0x385` 就意味着有两份
    定义，改了一处忘了另一处，症状同上

    python tools/selftest_gripper_frames.py
"""
import ast
import os
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')
NODE = os.path.join(GRASP, 'dummy_vc_grasp', 'gripper_can_node.py')

#: 纯层允许 import 的东西。多一个 ROS/socket 进来，这个自测就没了。
PURE_IMPORTS = {'math', 'struct'}

#: 2026-10-27 重标定之后的实测点（卡尺）：圈数 → 开口（mm），两个方向各取一段
MEASURED = ((0.0, 34.0), (0.5, 28.0), (0.9, 24.0), (-1.0, 46.0))

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


def close(label, got, want, tol):
    if abs(got - want) <= tol:
        ok(f'{label}（{got:.3f} vs {want:.3f}）')
        return True
    fail(f'{label}: 拿到 {got:.3f}，应该是 {want:.3f} ± {tol}')
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
    print('\n1. 帧 ID 与 35 板固件一致（node 7，mode 0x01 / 0x05 / 0x23）')
    same('使能帧 0x381', gf.ENABLE_FRAME_ID, 0x381)
    same('位置帧 0x385', gf.POSITION_FRAME_ID, 0x385)
    same('回读帧 0x3A3', gf.POSITION_FEEDBACK_ID, 0x3A3)
    same('负载长度 8', gf.DLC, 8)
    same('回帧请求字节 0x01', gf.ACK_REQUEST, 0x01)
    same('标定帧 0x382（禁区）', gf.CALIBRATION_FRAME_ID, 0x382)

    print('\n1b. 0x382 必须发不出去：那是触发标定，载荷是什么都不影响')
    expect_error('to_socketcan(0x382) 直接拒', gf.ForbiddenFrame,
                 gf.to_socketcan, gf.CALIBRATION_FRAME_ID,
                 gf.encode_position(0.0))
    expect_error('check_frame_id(0x382) 也拒', gf.ForbiddenFrame,
                 gf.check_frame_id, gf.CALIBRATION_FRAME_ID)
    expect_error('非标准帧 ID 拒', ValueError, gf.check_frame_id, 0x800)
    for fid in (gf.ENABLE_FRAME_ID, gf.POSITION_FRAME_ID, gf.POSITION_FEEDBACK_ID):
        gf.check_frame_id(fid)
    ok('0x381 / 0x385 / 0x3A3 都放行')

    print('\n1c. 电流（0x3A1，float32）与零点偏移（0x3A4，int32）的回读')
    same('电流回读 ID', gf.CURRENT_FEEDBACK_ID, 0x3A1)
    same('零点回读 ID', gf.OFFSET_FEEDBACK_ID, 0x3A4)
    amps, finished = gf.decode_current(struct.pack('<f', 0.75) + bytes([1, 0, 0, 0]))
    close('电流按 float32 解', amps, 0.75, 1e-9)
    if finished:
        ok('电流回帧的到位标志读得对')
    else:
        fail('电流回帧的到位标志读错了')
    if gf.decode_offset(struct.pack('<i', -1234) + b'\x00' * 4) == -1234:
        ok('零点偏移按有符号 int32 解，不是 float')
    else:
        fail('零点偏移解错了')


def check_payload(gf):
    print('\n2. 负载字节：使能是 uint32，位置是 float32 小端 + 第 5 字节 1')
    same('使能', gf.encode_enable(True).hex(' '), '01 00 00 00 00 00 00 00')
    same('停', gf.encode_enable(False).hex(' '), '00 00 00 00 00 00 00 00')
    same('0.1 圈（真机实测过的那一帧）',
         gf.encode_position(0.1).hex(' '), 'cd cc cc 3d 01 00 00 00')
    same('0.25 圈', gf.encode_position(0.25).hex(' '), '00 00 80 3e 01 00 00 00')
    same('0.5 圈', gf.encode_position(0.5).hex(' '), '00 00 00 3f 01 00 00 00')
    same('读位置用全零负载', gf.encode_position_query().hex(' '),
         '00 00 00 00 00 00 00 00')

    print('\n3. 回帧解码：float32 圈数 + 第 5 字节"到位"')
    revs, finished = gf.decode_position(bytes.fromhex('e17a763d00000000'))
    close('真机那一帧 0.0602 圈', revs, 0.0602, 1e-4)
    same('第 5 字节 0 = 未到位', finished, False)
    revs, finished = gf.decode_position(bytes.fromhex('e17a763d01000000'))
    same('第 5 字节 1 = 到位', finished, True)


def check_socketcan(gf):
    print('\n4. SocketCAN 的 16 字节 can_frame：id / dlc / 对齐')
    frame = gf.to_socketcan(gf.POSITION_FRAME_ID, gf.encode_position(0.1))
    same('总长 16', len(frame), 16)
    # id 小端 4 字节、dlc 1 字节、3 字节对齐、然后 8 字节负载
    same('id = 85 03 00 00', frame[:4].hex(' '), '85 03 00 00')
    same('dlc = 08', frame[4:5].hex(), '08')
    same('中间 3 字节对齐补零', frame[5:8].hex(), '000000')
    same('负载跟在后面', frame[8:].hex(' '), 'cd cc cc 3d 01 00 00 00')
    expect_error('负载长度不对要拒绝', ValueError,
                 gf.to_socketcan, gf.POSITION_FRAME_ID, b'\x00' * 4)
    expect_error('扩展帧 ID 要拒绝', ValueError,
                 gf.to_socketcan, 0x800, gf.encode_position(0.0))


def check_mapping(gf):
    print('\n5. 宽度↔圈数：实测四点都要落在 0.6 mm 内（拟合 RMS 0.31 mm）')
    for revs, width in MEASURED:
        close('{:.2f} 圈 → 开口'.format(revs), gf.revs_to_width(revs), width, 0.6)
    for width in (0.0, 23.0, 34.0, 46.0, 55.0):
        back = gf.revs_to_width(gf.width_to_revs(width))
        close('{:.1f} mm 来回一趟'.format(width), back, width, 1e-6)
    close('满开 55 mm 的圈数是负的（正常）', gf.width_to_revs(55.0), -1.782, 0.01)
    close('抓 24 mm 块要的圈数', gf.width_to_revs(24.0), 0.886, 0.01)
    close('抓 24 mm 块留 2 mm 余量', gf.width_to_revs(26.0), 0.714, 0.01)

    print('\n5b. 现场量到两头时，映射改用"这两头之间"（2026-10-27 实测那组）')
    closed, span = 4.0341, 4.807
    close('0 mm → 合拢止位本身', gf.width_to_revs(0.0, closed, span), closed, 1e-9)
    close('55 mm → 张开止位本身',
          gf.width_to_revs(55.0, closed, span), closed - span, 1e-9)
    close('24 mm 打在中间',
          gf.width_to_revs(24.0, closed, span), closed - 24.0 * span / 55.0, 1e-9)
    close('两头插值的 24 mm 与固定斜率差不到 1 mm',
          abs(gf.width_to_revs(24.0, closed, span)
              - gf.width_to_revs(24.0, closed)) * gf.MM_PER_REV, 0.0, 1.0)
    for width in (0.0, 23.0, 24.0, 55.0):
        back = gf.revs_to_width(gf.width_to_revs(width, closed, span),
                                closed, span)
        close('{:.1f} mm 用两头插值来回一趟'.format(width), back, width, 1e-9)
    expect_error('行程 <= 0 要拒绝', ValueError,
                 gf.width_to_revs, 10.0, closed, 0.0)


def check_rejections(gf):
    print('\n6. 越界与非法值：抛异常，不静默夹紧')
    ok('0 mm 算得出圈数（{:.3f}）'.format(gf.width_to_revs(0.0)))
    close('55 mm 可以', gf.width_to_revs(55.0), -1.782, 0.002)
    expect_error('55.1 mm', gf.OutOfRange, gf.width_to_revs, 55.1)
    expect_error('-0.1 mm', gf.OutOfRange, gf.width_to_revs, -0.1)
    expect_error('NaN 宽度', gf.OutOfRange, gf.width_to_revs, float('nan'))
    expect_error('inf 宽度', gf.OutOfRange, gf.width_to_revs, float('inf'))
    expect_error('NaN 位置', gf.OutOfRange, gf.encode_position, float('nan'))
    expect_error('bool 不是数字', TypeError, gf.encode_position, True)
    expect_error('字符串不是数字', TypeError, gf.width_to_revs, '15')
    expect_error('4 字节回帧解不开', ValueError, gf.decode_position, b'\x00' * 4)


def check_clamp(gf):
    print('\n7. 夹紧要留痕：返回 (值, 有没有夹)')
    same('15 mm 不用夹', gf.clamp_width(15.0), (15.0, False))
    same('-1 mm 夹到 0', gf.clamp_width(-1.0), (0.0, True))
    same('60 mm 夹到 55', gf.clamp_width(60.0), (55.0, True))
    expect_error('NaN 夹不了，要拒绝', gf.OutOfRange, gf.clamp_width, float('nan'))


def check_node_uses_frames():
    print('\n8. 节点不许自己拼字节，也不许漏进 setup.py')
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
                and node.value in (0x381, 0x385, 0x382, 0x3A3):
            hardcoded.append(hex(node.value))
    same('节点里没有写死的 0x381/0x385/0x3A3', hardcoded, [])

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
        check_mapping(gf)
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
    print('  [ ok ] gripper_frames.py      enable/position/feedback frames match the 35-board firmware')
    print('  [ ok ] gripper_frames.py      width <-> revs mapping reproduces the measured points')
    print('  [ ok ] gripper_can_node.py    goes through the codec, and is registered')
    print('RESULT: every byte of the gripper frame is pinned down offline')
    return 0


if __name__ == '__main__':
    sys.exit(main())
