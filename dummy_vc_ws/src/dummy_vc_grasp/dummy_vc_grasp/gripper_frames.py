"""夹爪那块 35 板固件的 CAN 编解码：纯字节算术，不碰 ROS、不碰 socket。

**2026-10-26 整份重写过。** 最初这两帧是照主控固件里的 `DummyHand`
（`reference/Firmware/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:333-389`）
抄的，于是把 0x381 当成"电流上限"、0x382 当成"角度"。真板子接上 CAN 之后才发现
夹爪上这块板跑的是关节那套 `dummy-35motor-fw`：在那里 **0x382 是触发编码器标定**
——一按就把电机开环推两圈，手指顶到限位、标定失败、`ClearFlash()` 擦掉标定表，
位置模式从此再也不输出（`Ctrl/Motor/motor.cpp:96-103` 那道"没标定就没输出"的闸）。
这个误判让夹爪卡了一整晚，所以下面每条都标了源码出处，别再凭印象改。

帧的约定（标准帧 11 位，`StdId = node_id << 7 | mode`）：

    mode 0x01 → 0x381  使能：前 4 字节 uint32，1 = 使能，0 = 停
    mode 0x05 → 0x385  位置：前 4 字节 float32 圈数，第 5 字节 1 = 要求回一帧
    mode 0x23 → 0x3A3  位置回读：float32 圈数 + 第 5 字节 1 = 已到位

出处：`reference/Firmware/dummy-35motor-fw/UserApp/protocols/interface_can.cpp`
的 `case 0x01/0x05/0x23`。

位置单位是**电机圈数**，不是度也不是毫米（`interface_can.cpp:56-57` 把它乘
`MOTOR_ONE_CIRCLE_SUBDIVIDE_STEPS` 变成内部计数）。开口宽度与圈数是线性的，
2026-10-26 实测三点（量尺，读数量化到 0.5 mm）：0.10 圈 → 12.5 mm、
0.25 圈 → 10.5 mm、0.50 圈 → 8.0 mm，拟合 `开口 = 13.48 - 11.12 * 圈`，
三点残差都在 0.2 mm 内。齿条连杆是刚性的，所以这个一阶关系够用；换夹爪或拆过
齿轮就重测这三个数。
"""

import math
import struct

#: 夹爪板在总线上的节点号（板上 3 位拨码决定，`main.cpp:58` 读的就是它）。
NODE_ID = 7

MODE_ENABLE = 0x01
MODE_POSITION = 0x05
MODE_POSITION_FEEDBACK = 0x23

#: 使能帧、位置帧、位置回读帧
ENABLE_FRAME_ID = (NODE_ID << 7) | MODE_ENABLE                  # 0x381
POSITION_FRAME_ID = (NODE_ID << 7) | MODE_POSITION              # 0x385
POSITION_FEEDBACK_ID = (NODE_ID << 7) | MODE_POSITION_FEEDBACK  # 0x3A3

#: 三种帧都是固定 8 字节
DLC = 8

#: 位置帧第 5 字节：1 = 请板子回一帧 0x3A3（`case 0x05` 里的 `_data[4]`）
ACK_REQUEST = 0x01

#: 开口宽度（mm）与电机圈数的线性关系：开口 = 13.48 - 11.12 * 圈数
REV_AT_ZERO_WIDTH_MM = 13.48
MM_PER_REV = 11.12

#: 两指的机械行程（实测最大开口 55 mm）
MIN_WIDTH_MM = 0.0
MAX_WIDTH_MM = 55.0

#: SocketCAN 的 `struct can_frame`：id(4) + dlc(1) + 3 字节对齐 + data(8)
CAN_FRAME = struct.Struct('<IB3x8s')


class OutOfRange(ValueError):
    """值超出这块板能接受的范围。调用方该夹就夹，别让它悄悄过去。"""


def _as_float(name, value):
    """只要真正的数字：bool 是 int 的子类，/"5" 这种字符串都不许混进来。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError('{} 必须是数字，拿到的是 {!r}'.format(name, value))
    number = float(value)
    if math.isnan(number) or math.isinf(number):
        raise OutOfRange('{} 是 {}，CAN 上没法表达'.format(name, number))
    return number


def encode_enable(enabled):
    """使能帧的 8 字节负载：uint32 1/0，后 4 字节补零。"""
    return struct.pack('<I', 1 if enabled else 0) + b'\x00' * (DLC - 4)


def encode_position(revs):
    """位置帧的 8 字节负载：float32 圈数 + 第 5 字节"请回帧" + 3 字节补零。"""
    value = _as_float('位置', revs)
    return struct.pack('<f', value) + bytes([ACK_REQUEST, 0, 0, 0])


def encode_position_query():
    """读位置：负载内容无所谓（`case 0x23` 总会回），全零就行。"""
    return b'\x00' * DLC


def decode_position(payload):
    """把 8 字节位置回帧解成 `(圈数, 是否到位)`。"""
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    revs = struct.unpack('<f', payload[:4])[0]
    return revs, payload[4] != 0


def width_to_revs(width_mm):
    """开口宽度（mm）→ 电机圈数。越界抛异常，不静默夹紧。"""
    width = _as_float('开口宽度', width_mm)
    if width < MIN_WIDTH_MM or width > MAX_WIDTH_MM:
        raise OutOfRange('开口 {:.2f} mm 超出 [{:g}, {:g}] mm'.format(
            width, MIN_WIDTH_MM, MAX_WIDTH_MM))
    return (REV_AT_ZERO_WIDTH_MM - width) / MM_PER_REV


def revs_to_width(revs):
    """电机圈数 → 开口宽度（mm）。"""
    return REV_AT_ZERO_WIDTH_MM - MM_PER_REV * _as_float('位置', revs)


def clamp_width(width_mm):
    """把越界宽度夹回 [0, 55]，返回 `(夹紧后的值, 有没有夹)`。

    调用方拿第二个返回值决定要不要打一行警告 —— 夹紧这件事必须留下痕迹。
    """
    value = _as_float('开口宽度', width_mm)
    if value < MIN_WIDTH_MM:
        return MIN_WIDTH_MM, True
    if value > MAX_WIDTH_MM:
        return MAX_WIDTH_MM, True
    return value, False


def to_socketcan(frame_id, payload):
    """拼出可以直接 `sock.send()` 的 16 字节 `struct can_frame`。

    单独一个函数是为了能在 Windows 上验字节布局 —— 这是最容易写错、又最不容
    易在真总线上看出来的一步。
    """
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    if not 0 <= frame_id <= 0x7FF:
        raise ValueError('只发标准帧，ID 不能超过 0x7FF，拿到 0x{:X}'.format(frame_id))
    return CAN_FRAME.pack(frame_id, DLC, payload)


def describe(frame_id, payload):
    """一行人话，用来打日志。"""
    if frame_id == POSITION_FRAME_ID:
        revs = struct.unpack('<f', payload[:4])[0]
        return '0x{:03X} 位置 {:.4f} 圈（开口约 {:.1f} mm）'.format(
            frame_id, revs, revs_to_width(revs))
    if frame_id == POSITION_FEEDBACK_ID:
        revs, finished = decode_position(payload)
        return '0x{:03X} 回读 {:.4f} 圈 / 开口 {:.1f} mm{}'.format(
            frame_id, revs, revs_to_width(revs), '（到位）' if finished else '')
    if frame_id == ENABLE_FRAME_ID:
        enabled = struct.unpack('<I', payload[:4])[0] != 0
        return '0x{:03X} {}'.format(frame_id, '使能' if enabled else '停')
    return '0x{:03X} {}'.format(frame_id, payload.hex(' '))
