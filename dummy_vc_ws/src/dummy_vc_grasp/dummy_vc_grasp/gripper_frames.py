"""夹爪那两帧 CAN 报文的编解码：纯字节算术，不碰 ROS、不碰 socket。

为什么要把这一层单独拆出来：帧格式是从参考固件抄的（`DummyHand`，
`reference/Firmware/dummy-ref-core-fw/Robot/instances/dummy_robot.cpp:333-389`），
**没有在真板子上验证过**。所以"字节怎么摆"和"往哪条总线发"分开：前者用纯
Python 就能验（`tools/selftest_gripper_frames.py`，Windows 上也跑得动），后者
只能在 Linux 的 SocketCAN 上跑（`gripper_can_node.py`）。

帧的约定（逐项来自源码，不是猜的）：

    StdId  = node_id << 7 | mode        node_id = 7（夹爪），mode 见下
    IDE    = 标准帧（11 位）
    DLC    = 8
    负载   = float32 小端占前 4 字节，后 4 字节补 0

    mode 0x01 → StdId 0x381，电流上限，夹到 [0, 1] A
    mode 0x02 → StdId 0x382，角度，   夹到 [0, 30] 度

两条不许忘的事实：

1. `hand` 在协议里**只有三个写方法，没有任何回读**（`protocol.json` 里 id
   296~301）。所以"帧发出去了"只说明它离开了网卡，不说明夹爪到了位。
2. 0~30 度是**夹爪自己的角度**，不是开口宽度。宽度↔角度的映射还没实测
   （`docs/02` 阶段 0.5），所以这里不做任何换算：给多少度就发多少度。

越界的选择：**抛异常，不静默夹紧**。静默夹紧会让"我要 40 度它只给了 30 度"
变成一个看不见的错误；要夹就由调用方自己夹、自己把这件事写进日志。
"""

import math
import struct

#: 夹爪板在总线上的节点号。源码里 `DummyHand(_hcan, 7)`。
NODE_ID = 7

MODE_CURRENT_LIMIT = 0x01
MODE_ANGLE = 0x02

#: 电流上限帧（StdId 0x381）与角度帧（StdId 0x382）
CURRENT_LIMIT_FRAME_ID = (NODE_ID << 7) | MODE_CURRENT_LIMIT
ANGLE_FRAME_ID = (NODE_ID << 7) | MODE_ANGLE

#: 源码里两种帧都是固定 8 字节：4 字节 float + 4 字节补零
DLC = 8

#: 源码里的夹紧范围（`SetAngle` 的 0~30、`SetMaxCurrent` 的 0~1）
MIN_ANGLE_DEG = 0.0
MAX_ANGLE_DEG = 30.0
MIN_CURRENT_A = 0.0
MAX_CURRENT_A = 1.0

#: SocketCAN 的 `struct can_frame`：id(4) + dlc(1) + 3 字节对齐 + data(8)
CAN_FRAME = struct.Struct('<IB3x8s')


class OutOfRange(ValueError):
    """值超出夹爪板能接受的范围。调用方该夹就夹，别让它悄悄过去。"""


def _as_float(name, value):
    """只要真正的数字：bool 是 int 的子类，/"5" 这种字符串都不许混进来。"""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError('{} 必须是数字，拿到的是 {!r}'.format(name, value))
    number = float(value)
    if math.isnan(number) or math.isinf(number):
        raise OutOfRange('{} 是 {}，CAN 上没法表达'.format(name, number))
    return number


def _check_range(name, number, low, high, unit):
    if number < low or number > high:
        raise OutOfRange('{} 是 {:.3f} {}，超出夹爪板的 [{:g}, {:g}] {}'.format(
            name, number, unit, low, high, unit))
    return number


def encode_angle(degrees):
    """角度帧的 8 字节负载。`degrees` 必须在 [0, 30]，否则抛异常。"""
    value = _as_float('角度', degrees)
    _check_range('角度', value, MIN_ANGLE_DEG, MAX_ANGLE_DEG, '度')
    return struct.pack('<f', value) + b'\x00' * (DLC - 4)


def encode_current(amperes):
    """电流上限帧的 8 字节负载。`amperes` 必须在 [0, 1]，否则抛异常。"""
    value = _as_float('电流上限', amperes)
    _check_range('电流上限', value, MIN_CURRENT_A, MAX_CURRENT_A, 'A')
    return struct.pack('<f', value) + b'\x00' * (DLC - 4)


def decode(payload):
    """把 8 字节负载解回 float。给日志和自测用，不是给控制回路用。"""
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    return struct.unpack('<f', payload[:4])[0]


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


def clamp_angle(degrees):
    """把越界角度夹回 [0, 30]，返回 `(夹紧后的值, 有没有夹)`。

    调用方拿这个函数夹、拿第二个返回值决定要不要打一行警告 —— 夹紧这件事必须
    留下痕迹，不能安静地发生。
    """
    value = _as_float('角度', degrees)
    if value < MIN_ANGLE_DEG:
        return MIN_ANGLE_DEG, True
    if value > MAX_ANGLE_DEG:
        return MAX_ANGLE_DEG, True
    return value, False


def describe(frame_id, payload):
    """一行人话，用来打日志。"""
    if frame_id == ANGLE_FRAME_ID:
        return '0x{:03X} 角度 {:.2f} 度'.format(frame_id, decode(payload))
    if frame_id == CURRENT_LIMIT_FRAME_ID:
        return '0x{:03X} 电流上限 {:.3f} A'.format(frame_id, decode(payload))
    return '0x{:03X} {}'.format(frame_id, payload.hex(' '))
