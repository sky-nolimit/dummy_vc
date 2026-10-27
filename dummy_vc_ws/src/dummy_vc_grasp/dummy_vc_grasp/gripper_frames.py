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
齿条连杆是刚性的，所以一阶关系够用。

**2026-10-27 重标定之后重测（9 点，卡尺，两个方向各一段）**：0 圈 = 34 mm，
往闭方向 0.25/0.5/0.75/0.9/1.0 圈 → 31/28/26/24/23 mm，往开方向 −0.5/−1.0/−1.5 圈
→ 40/46/52 mm，合起来拟合 `开口 = 34.294 - 11.620 * 圈数`，RMS 0.31 mm、最大
0.48 mm。用 `tools/gripper_sweep.py` 采数、`tools/fit_gripper_width.py` 拟合，
别手算。

一条实测得来的警告：**零点会被重标定静默改掉**。2026-10-27 那天 CAN 掉线、板子
重启之后自己又跑了一次标定，零点从旧位置挪到了 34 mm 处（之前 −0.2195 圈 =
16 mm，之后 0 圈 = 34 mm）。所以：只要板子重启过，就先读一次位置、量一次开口，
确认 `0 圈 ≈ 34 mm` 还成立，再相信下面这两个常数。
"""

import math
import struct

#: 夹爪板在总线上的节点号（板上 3 位拨码决定，`main.cpp:58` 读的就是它）。
NODE_ID = 7

MODE_ENABLE = 0x01
MODE_POSITION = 0x05
MODE_POSITION_FEEDBACK = 0x23
MODE_CURRENT_FEEDBACK = 0x21
MODE_OFFSET_FEEDBACK = 0x24

#: 使能帧、位置帧、位置回读帧
ENABLE_FRAME_ID = (NODE_ID << 7) | MODE_ENABLE                  # 0x381
POSITION_FRAME_ID = (NODE_ID << 7) | MODE_POSITION              # 0x385
POSITION_FEEDBACK_ID = (NODE_ID << 7) | MODE_POSITION_FEEDBACK  # 0x3A3

#: 电流回读（`case 0x21`，float32，单位没标定）与零点偏移回读（`case 0x24`，int32）
CURRENT_FEEDBACK_ID = (NODE_ID << 7) | MODE_CURRENT_FEEDBACK    # 0x3A1
OFFSET_FEEDBACK_ID = (NODE_ID << 7) | MODE_OFFSET_FEEDBACK      # 0x3A4

#: 三种帧都是固定 8 字节
DLC = 8

#: **禁区**：0x382 在这块板上是 `case 0x02` = 触发编码器标定，不是"设角度"。
#: 载荷被完全忽略，发一个全零帧照样会触发：电机被开环推两圈、手指顶到限位、
#: 采样不够 → 标定失败 → 标定表作废 → 位置模式从此不输出，而且**位置回读会
#: 冻在最后一个值上**（读数是查表产物）。恢复只能给板子断电重启。
#: 2026-10-26 和 27 日各踩过一次。`to_socketcan()` 会直接拒绝这个 ID。
CALIBRATION_FRAME_ID = (NODE_ID << 7) | 0x02                     # 0x382

#: 位置帧第 5 字节：1 = 请板子回一帧 0x3A3（`case 0x05` 里的 `_data[4]`）
ACK_REQUEST = 0x01

#: 开口宽度（mm）与电机圈数的线性关系：开口 = 34.294 - 11.620 * 圈数
#:
#: 2026-10-27 重标定后 9 点实测拟合（RMS 0.31 mm）。**重新标定过夹爪就一定要
#: 重测**：标定改的是 `encoderHomeOffset`，同一圈数对应的物理位置整体平移，
#: 旧数据直接作废。重测用 `tools/gripper_sweep.py`（自己走点、问你卡尺读数）+
#: `tools/fit_gripper_width.py`（拟合出这两个常数并打印成可直接粘贴的格式）。
#:
#: **这两个数不是"夹爪的物理常数"，而是"那一次标定下的坐标系"**：斜率
#: `MM_PER_REV` 是齿条节距决定的、不随标定变；截距 `REV_AT_ZERO_WIDTH_MM` 是
#: "手指完全合拢时电机读到多少圈"，它跟着编码器零点走。
#:
#: 所以真正稳的做法是**每次用之前现场量一次零点**（把手指慢慢推到合拢，看它停
#: 在哪个圈数），那就是 `tools/gripper_home.py` 干的事；这两个常数只在"还没量
#: 过"的时候当默认锚点用。见 `DEFAULT_REV_CLOSED`。
REV_AT_ZERO_WIDTH_MM = 34.294
MM_PER_REV = 11.620

#: 默认锚点：手指合拢（开口 0）时电机应该在的圈数。由上面那条拟合关系反推：
#: `34.294 / 11.620 = 2.951`。**它只是默认值** —— 夹爪重新标定过、或者被手动
#: 掰过之后，真实值会变，用 `tools/gripper_home.py` 现场量。
DEFAULT_REV_CLOSED = REV_AT_ZERO_WIDTH_MM / MM_PER_REV

#: 两指的机械行程（实测最大开口 55 mm）。这是**机械极限**：命令到 53 mm 以上
#: 手指就顶到本体、电机堵转，抓取流程里根本用不到那么开，别去试。
MIN_WIDTH_MM = 0.0
MAX_WIDTH_MM = 55.0

#: SocketCAN 的 `struct can_frame`：id(4) + dlc(1) + 3 字节对齐 + data(8)
CAN_FRAME = struct.Struct('<IB3x8s')


class OutOfRange(ValueError):
    """值超出这块板能接受的范围。调用方该夹就夹，别让它悄悄过去。"""


class ForbiddenFrame(ValueError):
    """这个 ID 不许发（目前只有 0x382：触发标定，会把夹爪标定表弄坏）。"""


def check_frame_id(frame_id):
    """发之前把 ID 查一遍。0x382 一票否决，其余只要求是标准帧。"""
    if frame_id == CALIBRATION_FRAME_ID:
        raise ForbiddenFrame(
            '0x{:03X} 是触发编码器标定，不是设角度，载荷是什么都不影响。发了就会'
            '把夹爪的标定表弄坏：位置模式从此不输出、回读冻在最后一个值上，只能断电'
            '重启。要动夹爪请用 0x{:03X}（使能）和 0x{:03X}（位置）。'.format(
                frame_id, ENABLE_FRAME_ID, POSITION_FRAME_ID))
    if not 0 <= frame_id <= 0x7FF:
        raise ValueError('只发标准帧，ID 不能超过 0x7FF，拿到 0x{:X}'.format(frame_id))


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


def encode_current_query():
    """读电流：同位置查询，`case 0x21` 不看载荷。"""
    return b'\x00' * DLC


def encode_offset_query():
    """读零点偏移：`case 0x24` 回 `encoderHomeOffset`（int32 计数）。

    用它判断"板子有没有偷偷重标定"：重标定会改这个值，位置曲线整体平移。
    """
    return b'\x00' * DLC


def decode_position(payload):
    """把 8 字节位置回帧解成 `(圈数, 是否到位)`。"""
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    revs = struct.unpack('<f', payload[:4])[0]
    return revs, payload[4] != 0


def decode_current(payload):
    """把 8 字节电流回帧解成 `(电流, 是否到位)`。

    `interface_can.cpp` 的 `case 0x21` 回的是 `GetFocCurrent()`，float32。
    **单位没标定过**，只当相对量用：同一个动作下比一比大小，别当安培读数。
    """
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    current = struct.unpack('<f', payload[:4])[0]
    return current, payload[4] != 0


def decode_offset(payload):
    """把 8 字节零点偏移回帧解成 int32（`case 0x24`，不是 float）。"""
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
    return struct.unpack('<i', payload[:4])[0]


def width_to_revs(width_mm, rev_closed=DEFAULT_REV_CLOSED, span_revs=None):
    """开口宽度（mm）→ 电机圈数。越界抛异常，不静默夹紧。

    `rev_closed` 是"手指完全合拢时的圈数"，也就是这部分坐标系的原点。
    默认用出厂标定值；现场量过之后请把量到的那个传进来 —— 否则一旦夹爪重新
    标定过（零点平移），同一条命令会把手指送到别的地方去。

    `span_revs` 是"两头止位之间差多少圈"（现场量的行程）。给了它就把
    [0, MAX_WIDTH_MM] 线性铺在这段行程上，两端都精确落在止位上；不给就退回
    固定的 `MM_PER_REV`（齿条节距，4.733 圈 = 55 mm）。两者差在 2% 以内，
    但现场量过的那一份更贴实机。
    """
    width = _as_float('开口宽度', width_mm)
    if width < MIN_WIDTH_MM or width > MAX_WIDTH_MM:
        raise OutOfRange('开口 {:.2f} mm 超出 [{:g}, {:g}] mm'.format(
            width, MIN_WIDTH_MM, MAX_WIDTH_MM))
    closed = _as_float('合拢圈数', rev_closed)
    if span_revs is None:
        return closed - width / MM_PER_REV
    span = _as_float('行程圈数', span_revs)
    if span <= 0.0:
        raise ValueError('行程必须是正的圈数，拿到 {!r}'.format(span_revs))
    return closed - width * span / MAX_WIDTH_MM


def revs_to_width(revs, rev_closed=DEFAULT_REV_CLOSED, span_revs=None):
    """电机圈数 → 开口宽度（mm）。其余参数同 `width_to_revs`。"""
    closed = _as_float('合拢圈数', rev_closed)
    revs = _as_float('位置', revs)
    if span_revs is None:
        return (closed - revs) * MM_PER_REV
    span = _as_float('行程圈数', span_revs)
    if span <= 0.0:
        raise ValueError('行程必须是正的圈数，拿到 {!r}'.format(span_revs))
    return (closed - revs) * MAX_WIDTH_MM / span


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

    所有发送路径都必须经过这里，因为**唯一一道禁止 0x382 的闸也在这里**：
    那一帧在真板子上是"触发编码器标定"，载荷是什么都无所谓，发出去就会把
    夹爪的标定表弄坏（详见 `CALIBRATION_FRAME_ID` 的说明）。
    """
    check_frame_id(frame_id)
    if len(payload) != DLC:
        raise ValueError('负载必须是 {} 字节，拿到 {}'.format(DLC, len(payload)))
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
