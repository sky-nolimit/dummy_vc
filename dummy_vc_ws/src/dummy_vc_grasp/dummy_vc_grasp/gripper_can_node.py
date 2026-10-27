"""把 `/gripper/width`（毫米）变成夹爪板的位置帧，并把回读转成 ROS 话题。

为什么要有这个节点：主控固件里的 `hand.*` 会把整机锁死（`docs/04` A2：
`CanSendMessage` 只在发送完成回调里归还信号量，出错路径都不归还），而且它发的
0x382 在这块板上是**触发标定**（详见 `gripper_frames.py` 的说明）。所以夹爪不走
主控，由 PC 经 USB-CAN 适配器直接对它说话，主控继续管它的六个关节。

先虚拟总线试（不需要任何硬件）：

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    ros2 run dummy_vc_grasp gripper_can_node --ros-args -p channel:=vcan0

真适配器（UTC-T01 是 CANable 兼容，内核 gs_usb 直接给出 can0）：

    sudo modprobe gs_usb
    sudo ip link set can0 type can bitrate 1000000 sample-point 0.667
    sudo ip link set up can0
    ros2 run dummy_vc_grasp gripper_can_node --ros-args -p channel:=can0

发一个开口宽度（毫米），15 mm 是个安全起点：

    ros2 topic pub --once /gripper/width std_msgs/msg/Float64 "{data: 15.0}"

回读在 `/gripper/position`（毫米）：

    ros2 topic echo /gripper/position

两个前提，不然一定白费：

1. 板子必须**已经标定过**（`encoder->IsCalibrated()`），否则固件那道闸
   （`Ctrl/Motor/motor.cpp:96-103`）会让输出恒为零，位置命令一点反应都没有，
   而回读照样在动——这看起来像"命令没进去"，其实是被闸挡了。
2. 位置是**绝对电机圈数**，不是相对量。断电再上电，同一圈数对应同一开口。

还有一条会打架的地方：主控的 `power_on()` 会发 0x381 带一个 float（`maxCurrent`），
而这块板只认 uint32 == 1 才是使能，其余一律 `MODE_STOP` —— **主控一启动就把夹爪
置停**。所以 `enable_before_move` 默认开着，每次下位置前补一帧使能；主控重启过
也不用管它。主控那边绝不能再调 `hand.*`。
"""

import argparse
import math
import socket
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String

from dummy_vc_grasp import gripper_frames as gf
from dummy_vc_grasp import gripper_home as gh


class GripperCanNode(Node):
    def __init__(self):
        super().__init__('gripper_can_node')

        #: SocketCAN 的接口名。虚拟总线是 vcan0，真适配器一般是 can0。
        self.declare_parameter('channel', 'vcan0')
        self.declare_parameter('width_topic', '/gripper/width')
        self.declare_parameter('position_topic', '/gripper/position')
        #: 主动问一次位置（板子自己也会被主控轮询，这里只是不依赖主控）
        self.declare_parameter('poll_hz', 5.0)
        self.declare_parameter('enable_on_start', True)
        #: 每次下位置前再补一帧使能。主控的 power_on 会把夹爪置停（见下方
        #: `_on_width` 的注释），所以这个默认开着。
        self.declare_parameter('enable_before_move', True)
        #: 只打印不发送：用来在没开 can0 的机器上先看一眼要发什么。
        self.declare_parameter('dry_run', False)
        #: 启动时先量一次合拢点。**默认开着**：宽度那条线性关系的截距跟着编码器
        #: 零点走，零点会因为"重新标定一次"就平移，所以不能假设上电时手指在哪。
        self.declare_parameter('home_on_start', True)
        #: 找零点时命令目标超过止位多少圈（整段行程才 4.7 圈）。
        self.declare_parameter('home_span_revs', gh.DEFAULT_SPAN_REVS)
        #: 两头行程与机械行程差多少圈就拒绝（判"两指之间卡了东西"）。
        self.declare_parameter('home_stroke_tol_revs',
                               gh.DEFAULT_STROKE_TOL_REVS)
        #: 量完之后张到多宽（不要让手指一直顶在止位上）。
        self.declare_parameter('idle_width_mm', 50.0)
        #: 退出时合拢到多宽。0 = 完全闭合：手指收回去更不容易磕碰、也不容易
        #: 进灰；想停在半开就改这个数。
        self.declare_parameter('park_width_mm', 0.0)
        #: 退出时要不要做上面那步。
        self.declare_parameter('park_on_shutdown', True)
        #: 退出时合拢要等多久再读回读（手指走 0~50 mm 约 1 s）。
        self.declare_parameter('park_settle_s', 1.5)

        #: 位置模式是 DCE（比例型）控制，负载和齿条摩擦下会停在离目标一截的
        #: 地方：真机上"要 50 mm"经常停在 52~54 mm。所以发完不等它自己收敛，
        #: 而是读回实测、按"目标 = 当前 + 差值"补发一次，最多补 `width_max_iters`
        #: 次。容差 0.8 mm 比手指和物块的接触变形大，不会来回抖。
        self.declare_parameter('width_tol_mm', 0.8)
        self.declare_parameter('width_max_iters', 4)
        self.declare_parameter('width_settle_s', 0.4)
        #: 找零点之前先等板子回一帧位置，最多等这么久（刚开 socket 时板子可能
        #: 正在上电/重启，第一帧要一两百毫秒才到）。
        self.declare_parameter('home_board_timeout_s', 2.0)
        #: 零点没量成时，要不要退回出厂合拢点继续用。**默认不要**：出厂值是别的
        #: 板子的状态，在这块板上整体偏了 12.6 mm（实测），拿它张爪会顶到限位、
        #: 拿它合爪会留 11 mm 缝 —— 比"一步不动"更难查。
        self.declare_parameter('home_allow_default', False)
        #: 零点状态发在这里：ready / default_zero / no_zero。
        self.declare_parameter('status_topic', '/gripper/status')

        self.channel = str(self.get_parameter('channel').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self._enable_before_move = bool(
            self.get_parameter('enable_before_move').value)
        self.sock = None if self.dry_run else self._open_socket(self.channel)

        #: 现场量到的合拢点（圈）。None = 还没量过，按出厂标定值算。
        self.rev_closed = None
        #: 现场量到的两头行程（圈）。None = 还没量过，按齿条节距换算。
        self.span_revs = None
        #: 零点量准了吗（量准了才敢按毫米指挥）。
        self.zero_known = False
        #: 允不允许动夹爪。零点没量成、又没被允许退回出厂值时为 False：
        #: 张爪、合爪、收手全部跳过，并且每次收到命令都说明原因。
        self._may_move = True
        self._last_refusal = 0.0

        self.frames_sent = 0
        self._stop = threading.Event()
        self._reader = None
        self._last_reported_width = None
        self._last_finished = None
        #: 最近一次回读到的电机圈数与开口（闭环修正用）。
        self._last_revs = None
        self._last_width = None
        #: 闭环修正的当前目标：None = 没有在跟的目标。
        self._target_width_mm = None
        self._target_iters = 0
        self._target_ready_at = 0.0

        self.position_pub = self.create_publisher(
            Float64, str(self.get_parameter('position_topic').value), 10)
        self.sub = self.create_subscription(
            Float64, str(self.get_parameter('width_topic').value),
            self._on_width, 10)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), 10)

        # 找零点放在"发布器/接收器都建好、接收线程还没起"的位置：那时没人抢帧，
        # 而状态话题已经能发了。
        if self.sock is not None and bool(
                self.get_parameter('home_on_start').value):
            self._home_on_start()
        # 状态每秒重发一次：抓取节点可能比夹爪节点晚起，晚订阅也能收到。
        self.create_timer(1.0, self._publish_status)

        if not self.dry_run:
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()

        poll_hz = float(self.get_parameter('poll_hz').value)
        self.create_timer(1.0 / max(1.0, poll_hz), self._on_poll)

        enabled = bool(self.get_parameter('enable_on_start').value)
        if enabled:
            self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))

        self.get_logger().info(
            '夹爪通道就绪：{}（{}）。{} → 0x{:03X}，回读 0x{:03X} → {}。'
            '宽度以"手指合拢"那一点为原点（当前 {:.4f} 圈{}）。'
            '前提是板子已经标定过，否则位置命令会被固件那道闸挡掉。'.format(
                self.channel, '只打印不发送' if self.dry_run else 'SocketCAN 已打开',
                self.get_parameter('width_topic').value, gf.POSITION_FRAME_ID,
                gf.POSITION_FEEDBACK_ID,
                self.get_parameter('position_topic').value,
                self.anchor,
                '' if self.rev_closed is not None else (
                    '，出厂值 —— 零点没量出来，本节点不接受移动命令'
                    if not self._may_move else '，出厂值，没量过')))

    # -- 找零点 -----------------------------------------------------------
    @property
    def anchor(self):
        """宽度 ↔ 圈数的原点：量到过就用量的，没量过用出厂标定值。"""
        return (gf.DEFAULT_REV_CLOSED if self.rev_closed is None
                else self.rev_closed)

    @property
    def span(self):
        """两头止位之间差多少圈（没量过就 None，退回固定的齿条节距换算）。"""
        return self.span_revs

    def _read_revs(self, timeout=0.3, attempts=1):
        """同步读一次位置（圈），读不到返回 None。

        只在找零点时用 —— 那会儿接收线程还没起来，没人跟它抢帧。`attempts > 1`
        时会连着重发查询：刚打开 socket 的那一瞬间板子往往还没答上来。
        """
        for _ in range(max(1, int(attempts))):
            revs = self._read_revs_once(timeout)
            if revs is not None:
                return revs
        return None

    def _read_revs_once(self, timeout=0.3):
        try:
            self.sock.send(gf.to_socketcan(
                gf.POSITION_FEEDBACK_ID, gf.encode_position_query()))
        except OSError:
            return None
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                data = self.sock.recv(gf.CAN_FRAME.size)
            except socket.timeout:
                continue
            except OSError:
                return None
            if len(data) < gf.CAN_FRAME.size:
                continue
            frame_id, _dlc, payload = gf.CAN_FRAME.unpack(data)
            if frame_id != gf.POSITION_FEEDBACK_ID:
                continue
            try:
                revs, _finished = gf.decode_position(payload)
            except ValueError:
                continue
            return revs
        return None

    def _wait_for_board(self, timeout=None):
        """等板子第一次回位置。刚开 socket 时它可能正在上电，别立刻判死刑。"""
        if timeout is None:
            timeout = float(self.get_parameter('home_board_timeout_s').value)
        waited = 0.0
        while waited < float(timeout):
            if self._read_revs(timeout=0.4) is not None:
                if waited > 0.0:
                    self.get_logger().info(
                        '{:.1f} 秒后板子开始回位置了'.format(waited))
                return True
            waited += 0.4
        return False

    def _home_on_start(self):
        """启动时先把手指推到合拢，量出"开口 0"在哪一圈。

        这一步是宽度映射可靠的前提：截距跟着编码器零点走，而零点会被一次重新
        标定平移掉。量不到就**不动**（除非显式允许退回出厂值）—— 用错零点张爪
        会顶到限位、合爪会留缝，比不动作更难查。
        """
        log = self.get_logger()

        def _send(frame_id, payload):
            self.sock.send(gf.to_socketcan(frame_id, payload))

        if not self._wait_for_board():
            log.error(
                '夹爪板 {:.1f} 秒里一帧位置都没回。先确认 can0 UP、夹爪板通电、'
                'channel 写对了；这次不会量零点'.format(
                    float(self.get_parameter('home_board_timeout_s').value)))

        result = gh.home(
            _send, lambda: self._read_revs(timeout=0.3, attempts=3),
            span_revs=float(self.get_parameter('home_span_revs').value),
            stroke_tol_revs=float(
                self.get_parameter('home_stroke_tol_revs').value),
            log=lambda message: log.info('找零点：{}'.format(message)))
        if result.ok:
            self.rev_closed = result.rev_closed
            self.span_revs = result.span_revs
            self.zero_known = True
            self._may_move = True
            log.info('合拢点 {:.4f} 圈、张开点 {:.4f} 圈，行程 {:.1f} mm；'
                     '宽度就按这两头之间插值'.format(
                         result.rev_closed, result.rev_open,
                         result.span_revs * gf.MM_PER_REV))
        else:
            allow = bool(self.get_parameter('home_allow_default').value)
            self.zero_known = False
            self._may_move = allow
            if allow:
                log.warn('没量成：{}。按要求退回出厂合拢点 {:.4f} 圈继续 —— '
                         '宽度会整体偏（这块板实测偏 12.6 mm），别拿它夹东西'
                         .format(result.reason, gf.DEFAULT_REV_CLOSED))
            else:
                log.error(
                    '没量成：{}。零点未知，**这次不会动夹爪**（张/合/收手都跳过），'
                    '抓取会在合爪那一步失败。先跑 tools/gripper_home.py 量一次；'
                    '硬要用出厂值就把 home_allow_default 设成 true（不推荐）'
                    .format(result.reason))
        if not self._may_move:
            return
        # 量完（或者没量成）都别让手指顶在止位上。
        width, _ = gf.clamp_width(
            float(self.get_parameter('idle_width_mm').value))
        try:
            self.sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID,
                                           gf.encode_enable(True)))
            self.sock.send(gf.to_socketcan(
                gf.POSITION_FRAME_ID,
                gf.encode_position(gf.width_to_revs(width, self.anchor, self.span))))
            log.info('张到 {:.1f} mm 待命'.format(width))
            # 也进闭环：位置模式有静差，这一发常常停在 52~54 mm，
            # 等接收线程起来以后由 _correct_once 补到位。
            self._target_width_mm = float(width)
            self._target_iters = 0
            self._target_ready_at = time.time() + 1.0
        except OSError as exc:
            log.warn('张开命令没发出去：{}'.format(exc))

    def _publish_status(self):
        """把"零点准不准"发出去，让抓取节点在动之前就能看见。"""
        if self.dry_run:
            return
        if self.zero_known:
            text = 'ready'
        elif self._may_move:
            text = 'default_zero'
        else:
            text = 'no_zero'
        self.status_pub.publish(String(data=text))

    def _refuse(self, note):
        """零点不可用时的统一回绝：说清楚原因，但别每 100 ms 刷一遍。"""
        now = time.time()
        if now - self._last_refusal >= 2.0:
            self._last_refusal = now
            self.get_logger().error(
                '不动夹爪：{}。先跑 tools/gripper_home.py 量零点'
                '（或把 home_allow_default 设成 true 强行用出厂值）'.format(note))

    # -- 传输层 -----------------------------------------------------------
    def _open_socket(self, channel):
        """打开 SocketCAN。打不开就把原因说清楚，别留一个 traceback。"""
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        except AttributeError:                  # 不是 Linux
            self.get_logger().fatal(
                '这个节点只能在 Linux 上跑（SocketCAN）；Windows 上先用'
                ' -p dry_run:=true 看要发什么')
            raise SystemExit(2)
        except OSError as exc:
            self.get_logger().fatal('建不出 CAN 套接字：{}'.format(exc))
            raise SystemExit(2)
        try:
            sock.bind((channel,))
        except OSError as exc:
            sock.close()
            self.get_logger().fatal(
                '绑不上 {}（{}）。虚拟总线先建：\n'
                '  sudo modprobe vcan && sudo ip link add dev {} type vcan'
                ' && sudo ip link set up {}\n'
                '真适配器先起：sudo modprobe gs_usb && '
                'sudo ip link set {} up type can bitrate 1000000'.format(
                    channel, exc, channel, channel, channel))
            raise SystemExit(2)
        # 收帧用超时，否则关节点时 recv 会一直挂着
        sock.settimeout(0.5)
        return sock

    def _send(self, frame_id, payload):
        if self.dry_run:
            self.get_logger().info('（只打印）{}'.format(
                gf.describe(frame_id, payload)))
            return
        frame = gf.to_socketcan(frame_id, payload)
        try:
            self.sock.send(frame)
        except OSError as exc:
            # 总线 down、适配器拔了、发送队列满，都在这里。别把失败吞掉。
            self.get_logger().error('发不出去（0x{:03X}）：{}'.format(frame_id, exc))
            return
        self.frames_sent += 1

    def _read_loop(self):
        """后台收帧：只认 0x3A3（位置回读），其余（关节轮询回帧）丢掉。"""
        while not self._stop.is_set():
            try:
                data = self.sock.recv(gf.CAN_FRAME.size)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < gf.CAN_FRAME.size:
                continue
            frame_id, _dlc, payload = gf.CAN_FRAME.unpack(data)
            if frame_id != gf.POSITION_FEEDBACK_ID:
                continue
            try:
                revs, finished = gf.decode_position(payload)
            except ValueError as exc:
                self.get_logger().warn('回帧解不开：{}'.format(exc))
                continue
            width = gf.revs_to_width(revs, self.anchor, self.span)
            self._last_revs = float(revs)
            self._last_width = float(width)
            self.position_pub.publish(Float64(data=width))
            self._report(revs, width, finished)

    def _report(self, revs, width, finished):
        """回读按变化量打点：5 Hz 全打会把日志淹掉。"""
        moved = (self._last_reported_width is None
                 or abs(width - self._last_reported_width) >= 0.2)
        flipped = finished != self._last_finished
        if not moved and not flipped:
            return
        self._last_reported_width = width
        self._last_finished = finished
        self.get_logger().info('回读 {:.4f} 圈 = 开口 {:.1f} mm{}'.format(
            revs, width, '（到位）' if finished else ''))

    # -- 命令 -------------------------------------------------------------
    def _on_poll(self):
        self._send(gf.POSITION_FEEDBACK_ID, gf.encode_position_query())
        self._correct_once()

    def _on_width(self, msg):
        raw = float(msg.data)
        if math.isnan(raw) or math.isinf(raw):
            self.get_logger().warn('收到 {}，丢掉'.format(raw))
            return
        if not self._may_move:
            # 零点没量出来：宁可一步不动，也不要拿出厂值把手指顶到限位上。
            self._refuse('零点没量出来（/gripper/status={}）'.format(
                'ready' if self.zero_known else 'no_zero'))
            return
        width, clamped = gf.clamp_width(raw)
        if clamped:
            self.get_logger().warn(
                '要 {:.2f} mm，机械行程只有 {:g}~{:g} mm，按 {:.2f} mm 发'.format(
                    raw, gf.MIN_WIDTH_MM, gf.MAX_WIDTH_MM, width))
        revs = gf.width_to_revs(width, self.anchor, self.span)
        # 每发一次位置都补一帧使能。原因：主控固件的 power_on 会调
        # hand.set_enable(true)，而它发的是 0x381 带一个 float（如 0.5A），
        # 35 板那边只认 uint32 == 1 才是使能，其余一律 MODE_STOP —— 主控
        # 一启动就会把夹爪置停。所以不能信任"节点启动时那一次使能"。
        if self._enable_before_move:
            self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))
        self._send(gf.POSITION_FRAME_ID, gf.encode_position(revs))
        self.get_logger().info(
            '开口 {:.2f} mm → 位置 {:.4f} 圈'.format(width, revs))
        # 从这一刻起盯着这个目标，等它停稳再由 _correct_once 补发。
        self._target_width_mm = float(width)
        self._target_iters = 0
        self._target_ready_at = time.time() + float(
            self.get_parameter('width_settle_s').value)

    def _correct_once(self):
        """按回读补发一次位置，把"要 50 停在 52~54"的静差补掉。

        位置模式是比例型控制，负载越大停得越偏；它自己不会收敛到目标。
        补发量用 `2*目标 − 实测`（同一个线性映射下的等量补偿），比按 mm/圈
        硬算更稳：映射里的截距、行程都在里面自动对消了。
        """
        if self._target_width_mm is None or self._last_revs is None:
            return
        if not self._may_move:
            self._target_width_mm = None
            return
        if time.time() < self._target_ready_at:
            return
        tol = float(self.get_parameter('width_tol_mm').value)
        max_iters = int(self.get_parameter('width_max_iters').value)
        error = self._target_width_mm - float(self._last_width)
        if abs(error) <= tol:
            if self._target_iters:
                self.get_logger().info(
                    '开口到位：要 {:.1f} mm，实测 {:.1f} mm（补发 {} 次）'.format(
                        self._target_width_mm, self._last_width,
                        self._target_iters))
            self._target_width_mm = None
            return
        if self._target_iters >= max_iters:
            self.get_logger().warn(
                '开口停在 {:.1f} mm（要 {:.1f} mm），补发 {} 次后还差 '
                '{:+.1f} mm —— 多半是顶到什么了'.format(
                    self._last_width, self._target_width_mm,
                    self._target_iters, error))
            self._target_width_mm = None
            return
        goal = gf.width_to_revs(self._target_width_mm, self.anchor, self.span)
        command = 2.0 * goal - float(self._last_revs)
        self._target_iters += 1
        try:
            if self._enable_before_move:
                self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))
            self._send(gf.POSITION_FRAME_ID, gf.encode_position(command))
        except OSError as exc:
            self.get_logger().error('补发位置失败（{}），不再修'.format(exc))
            self._target_width_mm = None
            return
        self.get_logger().info(
            '差 {:+.1f} mm，补发到 {:.4f} 圈（第 {} 次）'.format(
                error, command, self._target_iters))
        self._target_ready_at = time.time() + float(
            self.get_parameter('width_settle_s').value)

    def _park(self):
        """退出前合拢，让手指停在一个确定的、收起来的位置。

        这一步只在"正常退出"时做：先等接收线程退出去（否则两边抢同一帧），
        再发一帧使能 + 位置，等 settle，最后发停（不长期保持力矩）。
        """
        width = float(self.get_parameter('park_width_mm').value)
        self._target_width_mm = None            # 退出时别让闭环修正来抢
        if not self._may_move:
            self.get_logger().info('零点没量出来，退出时不收手（手指停在原地）')
            return
        revs = gf.width_to_revs(width, self.anchor, self.span)
        try:
            time.sleep(0.6)
            self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))
            self._send(gf.POSITION_FRAME_ID, gf.encode_position(revs))
            settle = float(self.get_parameter('park_settle_s').value)
            time.sleep(max(0.0, settle))
            now = self._read_revs(timeout=0.2)
        except OSError as exc:
            self.get_logger().warning('退出时合拢失败（{}），手指停在原地'
                                      .format(exc))
            return
        finally:
            try:
                self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
            except OSError:
                pass
        if now is None:
            self.get_logger().info('退出前已合拢到 {:.1f} mm（没读到回帧）'
                                   .format(width))
        else:
            self.get_logger().info(
                '退出前已合拢：目标 {:.1f} mm，实测 {:.1f} mm'.format(
                    width, gf.revs_to_width(now, self.anchor, self.span)))

    def shutdown(self):
        self._stop.set()
        if (self.sock is not None and not self.dry_run
                and bool(self.get_parameter('park_on_shutdown').value)):
            self._park()
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


def main(argv=None):
    parser = argparse.ArgumentParser(add_help=False)
    # ros2 run 和 --ros-args 混在一起时，交给 rclpy 剔掉它自己的参数。
    _, ros_argv = parser.parse_known_args(sys.argv[1:] if argv is None else argv)
    rclpy.init(args=ros_argv)
    node = GripperCanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
