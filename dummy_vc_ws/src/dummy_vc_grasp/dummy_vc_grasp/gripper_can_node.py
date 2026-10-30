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
        #: 命令发出后至少等这么久，才认为回读可能是"停稳"的值。位置模式有加减
        #: 速，0~50 mm 要走 1 s 以上；原来的 0.4 s 读到的是半路上的位置，拿它
        #: 算补发量会把目标来回推 —— 2026-10-30 真机日志里"要 50 mm"实测在
        #: 46.9 / 53.0 之间来回跳、补发 4 次还没到位，就是从这里开始的。
        self.declare_parameter('width_settle_s', 1.0)
        #: 时间判据之外的第二条：相邻两帧回读差超过这么多圈，就认为手指还在
        #: 动，继续等。慢慢爬的时候只靠时间判据不够。
        self.declare_parameter('width_settle_revs', 0.02)
        #: 判"板子卡死"要**同时**满足两条：连续这么多次主动查询都没答……
        self.declare_parameter('link_miss_max', 8)
        #: ……并且已经这么久没收到 0x3A3（秒）。2026-10-30 之前只看时间
        #: （3 s，真机上还被配成 2 s）：空闲时 1 Hz 查询，**丢一帧就等于
        #: "2 秒没回"**，于是一次偶发丢帧就把夹爪判死（_may_move=False），
        #: 之后张爪/合爪全被拒、只能断电重启 —— 日志里 13:29:40 那句
        #: "2.0 秒没回过一帧" 紧跟在一帧成功的回读后面，就是个假警报。
        #: 现在丢一帧只让计数 +1，要连丢 8 次（且空够 3 s）才判死。
        self.declare_parameter('link_timeout_s', 3.0)
        #: 空闲时的查询频率（Hz）。发得越少，撞上 35 板那条"一帧没发出去就
        #: 永久锁死发送"的固件缺陷的机会越小；有目标在跟的时候仍按 poll_hz 问。
        self.declare_parameter('idle_poll_hz', 1.0)
        #: 找零点之前先等板子回一帧位置，最多等这么久。**默认 30 秒**：这个节点
        #: 是跟 `pick_place_demo` 一起起的，那一刻往往正是主控在给自己归位的几秒
        #: —— 总线被 5 ms 一轮的运动命令占满，夹爪的帧号最大、优先级最低，回帧
        #: 成片被丢。手动跑 `tools/gripper_home.py` 时臂早停了、总线很空，所以
        #: 同一个操作手动能通、脚本里 2 秒就判死。别再把它调小。
        self.declare_parameter('home_board_timeout_s', 30.0)
        #: 等到了板子之后，**量零点那一次读**最多给它多久。默认 3 秒：能等到
        #: 板子第一帧，说明它活着，接下来只可能是总线又被挤了一段，就别再用
        #: 0.3 秒的窗口去赌（2026-10-30 13:57 那次就是赌输了直接弃用整场）。
        self.declare_parameter('home_first_read_timeout_s', 3.0)
        #: 零点没量成时后台隔多久补量一次（秒），以及最多补几次。0 = 不补。
        #: 一次启动撞上总线高峰不该让整场机器人报废，但也不能无限重试。
        self.declare_parameter('home_retry_s', 10.0)
        self.declare_parameter('home_retry_max', 12)
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
        #: 上一帧回读的圈数（判"停稳"用：相邻两帧差得够小才算停了）。
        self._prev_revs = None
        #: 最后一次实际发出去的圈数命令，以及发它的时间。补发量按
        #: "上次命令 + 实测差" 算，不能按 "2*目标 − 实测" 算 —— 后者默认
        #: 上一条命令就是目标，一旦上一条已经放大过，就会越补越偏。
        self._command_revs = None
        self._command_at = 0.0
        #: 闭环修正的当前目标：None = 没有在跟的目标。
        self._target_width_mm = None
        self._target_iters = 0
        #: 最近一次收到回帧的时间（看门狗用）。0 = 还没收到过。
        self._last_feedback_at = 0.0
        #: 上次收到回帧之后又问了几次（看门狗用）。
        self._polls_since_reply = 0
        #: 连续收帧失败次数（看门狗与日志用）。
        self._recv_errors = 0
        self._poll_tick = 0
        #: 收到过几帧回读、同步读超时过几次（诊断用：看日志就知道板子答得有多
        #: 慢、丢帧有多频繁，不用再猜"是不是 0.3 秒太短"）。
        self._reply_count = 0
        self._read_timeouts = 0
        #: 同步读（量零点）期间让接收线程停在原地，别两边抢同一个 socket。
        self._sync_reading = False
        self._home_retry_thread = None

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
        sent_at = time.time()
        deadline = sent_at + timeout
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
            self._last_feedback_at = time.time()
            self._polls_since_reply = 0
            self._reply_count += 1
            # 头三帧记下往返时间：量零点再失败时，日志能直接回答"是板子慢，
            # 还是这边读的窗口太短"，不用再凭猜。
            if self._reply_count <= 3:
                self.get_logger().info(
                    '回帧 0x{:03X}：往返 {:.0f} ms，{:.4f} 圈'.format(
                        gf.POSITION_FEEDBACK_ID,
                        (self._last_feedback_at - sent_at) * 1e3, revs))
            return revs
        self._read_timeouts += 1
        return None

    def _wait_for_board(self, timeout=None):
        """等板子第一次回位置。刚开 socket 时它可能正在上电，别立刻判死刑。

        等待期间每 2 秒说一声"还在等"，因为这段时间主控往往正在归位、总线很挤，
        夹爪的回帧被丢是常态 —— 沉默比报错更让人以为卡死了。
        """
        if timeout is None:
            timeout = float(self.get_parameter('home_board_timeout_s').value)
        waited = 0.0
        last_note = 0.0
        while waited < float(timeout):
            if self._read_revs(timeout=0.4) is not None:
                if waited > 0.0:
                    self.get_logger().info(
                        '{:.1f} 秒后板子开始回位置了'.format(waited))
                return True
            waited += 0.4
            if waited - last_note >= 2.0:
                last_note = waited
                self.get_logger().info(
                    '还在等夹爪板回帧（{:.0f}/{:.0f} 秒）。主控归位期间总线很挤，'
                    '夹爪的帧优先级最低，等它抢到空档'.format(
                        waited, float(timeout)))
        return False

    def _home_on_start(self):
        """启动时量一次零点；量不成就在后台隔一会儿再试。

        2026-10-30 13:57 真机：日志里刚说"1.2 秒后板子开始回位置了"，紧接着量
        零点却"板子没答"，于是整场抓取被拒（/gripper/status=no_zero）。一次
        启动撞上主控归位、总线挤满不该让机器人报废；但也不能无限重试。
        """
        if self._attempt_home():
            return
        if self._may_move:                       # 已按 home_allow_default 放行
            return
        if float(self.get_parameter('home_retry_s').value) <= 0.0:
            return
        self._home_retry_thread = threading.Thread(
            target=self._home_retry_loop, daemon=True)
        self._home_retry_thread.start()

    def _home_retry_loop(self):
        """后台补量零点：隔 home_retry_s 秒试一次，最多 home_retry_max 次。"""
        log = self.get_logger()
        retry_s = float(self.get_parameter('home_retry_s').value)
        tries = max(1, int(self.get_parameter('home_retry_max').value))
        for attempt in range(1, tries + 1):
            if self._stop.wait(retry_s):
                return
            if self.zero_known or self._may_move:
                return
            log.info('零点还没量出来，补量第 {}/{} 次'.format(attempt, tries))
            if self._attempt_home(quiet=True):
                log.info('补量成功：合拢点 {:.4f} 圈，接下来可以正常夹了'
                         .format(self.rev_closed))
                return
        log.error('补量 {} 次都没成，夹爪这次不参与抓取。先跑 '
                  'tools/gripper_home.py，或者按 _check_link 里那两条 '
                  'candump/cansend 判板子死活'.format(tries))

    def _attempt_home(self, quiet=False):
        """走一遍"等板子 → 量两头 → 张到待命"。返回零点是否量准。

        这一步是宽度映射可靠的前提：截距跟着编码器零点走，而零点会被一次重新
        标定平移掉。量不到就**不动**（除非显式允许退回出厂值）—— 用错零点张爪
        会顶到限位、合爪会留缝，比不动作更难查。
        """
        log = self.get_logger()

        def _send(frame_id, payload):
            self.sock.send(gf.to_socketcan(frame_id, payload))

        board_timeout = float(
            self.get_parameter('home_board_timeout_s').value)
        # 量零点期间让接收线程停在原地：两边都 recv 同一个 socket 时，谁抢到
        # 那一帧是随机的（2026-10-30 之前"时成时不成"就是这么来的）。
        self._sync_reading = True
        try:
            if not self._wait_for_board(board_timeout):
                self._zero_failed('板子 {:.0f} 秒里一帧位置都没回'
                                  '（开始量零点前就问过它了）'.format(board_timeout),
                                  log, quiet)
                return False
            first = self._read_revs(
                timeout=float(self.get_parameter(
                    'home_first_read_timeout_s').value), attempts=2)
            if first is None:
                self._zero_failed(
                    '等到过回帧，但量零点时又读不到（同步读超时 {} 次）'.format(
                        self._read_timeouts), log, quiet)
                return False

            result = gh.home(
                _send, lambda: self._read_revs(timeout=0.3, attempts=2),
                rev0=first,
                span_revs=float(self.get_parameter('home_span_revs').value),
                stroke_tol_revs=float(
                    self.get_parameter('home_stroke_tol_revs').value),
                log=lambda message: log.info('找零点：{}'.format(message)))
            if not result.ok:
                self._zero_failed(result.reason, log)
                return False

            self.rev_closed = result.rev_closed
            self.span_revs = result.span_revs
            self.zero_known = True
            self._may_move = True
            log.info('合拢点 {:.4f} 圈、张开点 {:.4f} 圈，行程 {:.1f} mm；'
                     '宽度就按这两头之间插值'.format(
                         result.rev_closed, result.rev_open,
                         result.span_revs * gf.MM_PER_REV))
            self._push_idle()
            return True
        finally:
            self._sync_reading = False

    def _zero_failed(self, reason, log, quiet=False):
        """零点没量成：按 home_allow_default 决定"用出厂值"还是"一步不动"。"""
        allow = bool(self.get_parameter('home_allow_default').value)
        self.zero_known = False
        self._may_move = allow
        if allow:
            log.warn('没量成：{}。按要求退回出厂合拢点 {:.4f} 圈继续 —— '
                     '宽度会整体偏（这块板实测偏 12.6 mm），别拿它夹东西'
                     .format(reason, gf.DEFAULT_REV_CLOSED))
            return
        if quiet:
            log.info('（补量）没量成：{}'.format(reason))
            return
        log.error(
            '没量成：{}。零点未知，**这次不会动夹爪**（张/合/收手都跳过），'
            '抓取会在合爪那一步失败。每 {:.0f} 秒会自动再试一次（最多 {} 次）；'
            '也可以先跑 tools/gripper_home.py 量一次，或把 home_allow_default '
            '设成 true（不推荐）'.format(
                reason, float(self.get_parameter('home_retry_s').value),
                int(self.get_parameter('home_retry_max').value)))

    def _push_idle(self):
        """量完零点都别让手指顶在止位上，张到待命宽度。"""
        log = self.get_logger()
        width, _ = gf.clamp_width(
            float(self.get_parameter('idle_width_mm').value))
        revs = gf.width_to_revs(width, self.anchor, self.span)
        try:
            self.sock.send(gf.to_socketcan(gf.ENABLE_FRAME_ID,
                                           gf.encode_enable(True)))
            self.sock.send(gf.to_socketcan(
                gf.POSITION_FRAME_ID, gf.encode_position(revs)))
            log.info('张到 {:.1f} mm 待命'.format(width))
            # 也进闭环：位置模式有静差，这一发常常停在 52~54 mm，
            # 等接收线程起来以后由 _correct_once 补到位。
            self._target_width_mm = float(width)
            self._target_iters = 0
            self._command_revs = float(revs)
            self._command_at = time.time()
        except OSError as exc:
            log.warn('张开命令没发出去：{}'.format(exc))

    def _publish_status(self):
        """把"零点准不准"发出去，让抓取节点在动之前就能看见。"""
        if self.dry_run:
            return
        self._check_link()
        if self.zero_known:
            text = 'ready'
        elif self._may_move:
            text = 'default_zero'
        else:
            text = 'no_zero'
        self.status_pub.publish(String(data=text))

    def _check_link(self):
        """板子还活着吗？活着才允许继续动。

        2026-10-28 真机：一次成功的抓取之后，夹爪板突然不再回 0x3A3，
        `candump` 上一条都没有，只有整机断电重启才恢复。35 板固件那条
        "一帧发不出去就永久丢掉 TX 信号量" 的缺陷（见 docs/04 A2-补充2）
        完全能解释这个现象 —— 板子自己的 CAN 发送路径锁死了，它不会再收
        也不会再答。

        我们没法从这边把它救回来（它连 reboot 帧都不处理），能做的是：
        立刻停止发命令、把状态改成 no_zero，并明确告诉人"断电重启"。

        **2026-10-30 修正：判据不能只看时间。** 原来"3 秒（或被配成 2 秒）没
        收到 0x3A3 就判死"，而空闲时查询只有 1 Hz —— 丢一帧就等于"1~2 秒没
        回"，于是实测里一帧刚成功的回读之后紧接着就冒出一句"2.0 秒没回过一帧"，
        夹爪被永久判死。CAN 上一帧偶发被挤掉是常态（主控归位时总线是满的），
        所以现在要求"连续 link_miss_max 次主动查询都没答"**且**"空够
        link_timeout_s 秒"两条同时成立才判死。
        """
        if not self.zero_known or self._last_feedback_at <= 0.0:
            return
        if not self._may_move:
            return
        silent_for = time.time() - self._last_feedback_at
        limit = float(self.get_parameter('link_timeout_s').value)
        miss_max = max(2, int(self.get_parameter('link_miss_max').value))
        misses = int(self._polls_since_reply)
        if misses < miss_max or silent_for < limit:
            return
        self._may_move = False
        self.zero_known = False
        self._target_width_mm = None
        self.get_logger().error(
            '夹爪板连续 {} 次没回位置（0x3A3），已经 {:.1f} 秒。先别断电，'
            '在另一个终端按住这个顺序试一次，能分清是哪种故障：\n'
            '  1) 挂上监听：candump can0,3A3:3FF\n'
            '  2) 另开一个终端问一次：cansend can0 3A3#0000000000000000\n'
            '  有回帧 = 板子活着，是这边的接收出了问题（看上面有没有"收帧失败"）；\n'
            '  没回帧 = 板子卡在自己的 CAN 发送上（35 板固件那条缺陷），只能断'
            '电重启。\n'
            '在那之前本节点不再发任何移动命令。'.format(misses, silent_for))

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
        # 只在内核里收 0x3A3：六轴主控的轮询回帧也在这条总线上，全收进用户态
        # 再丢，会让夹爪那几帧排在队列后面（真机上问一帧要等过半秒）。
        try:
            sock.setsockopt(socket.SOL_CAN_RAW, gf.CAN_RAW_FILTER,
                            gf.can_filter())
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        except OSError as exc:
            # 老内核/虚拟总线不支持也别把节点弄死，只是回到"全收再筛"。
            self.get_logger().warn('设 CAN 过滤/收包缓冲失败（{}），继续但可能'
                                   '收到全总线流量'.format(exc))
        return sock

    def _send(self, frame_id, payload):
        """发一帧。返回 True = 发出去了；False = 没发出去（原因已经记日志）。"""
        if self.dry_run:
            self.get_logger().info('（只打印）{}'.format(
                gf.describe(frame_id, payload)))
            return True
        frame = gf.to_socketcan(frame_id, payload)
        try:
            self.sock.send(frame)
        except OSError as exc:
            # 总线 down、适配器拔了、发送队列满，都在这里。别把失败吞掉。
            self.get_logger().error('发不出去（0x{:03X}）：{}'.format(frame_id, exc))
            return False
        self.frames_sent += 1
        return True

    def _read_loop(self):
        """后台收帧：只认 0x3A3（位置回读），其余（关节轮询回帧）丢掉。"""
        while not self._stop.is_set():
            if self._sync_reading:
                # 量零点期间主线程自己在同一个 socket 上同步读，这里让开：
                # 两边同时 recv 的话那一帧被谁抢到是随机的。
                time.sleep(0.02)
                continue
            try:
                data = self.sock.recv(gf.CAN_FRAME.size)
            except socket.timeout:
                continue
            except OSError as exc:
                # 以前这里直接 break：接收线程静默退出，现象和"板子不回帧"
                # 一模一样（看门狗也是"没收到回帧"），日志里却完全分不出来。
                # 现在留痕、计数，连续失败太多次才放弃。
                self._recv_errors += 1
                self.get_logger().error('收帧失败（{}），连续第 {} 次'.format(
                    exc, self._recv_errors))
                if self._recv_errors >= 10:
                    self.get_logger().error(
                        '连续 {} 次收帧失败，接收线程退出'.format(self._recv_errors))
                    break
                time.sleep(0.05)
                continue
            self._recv_errors = 0
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
            # 判"停稳"要看相邻两帧的差，所以先把上一帧挪过去再覆盖。
            self._prev_revs = self._last_revs
            self._last_revs = float(revs)
            self._last_width = float(width)
            self._last_feedback_at = time.time()
            self._polls_since_reply = 0
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
        self._poll_tick += 1
        # 有目标在跟的时候按 poll_hz 问；空闲时降到 idle_poll_hz —— 少发帧就少
        # 一次撞上固件那条"发送失败就永久锁死"的机会。
        poll_hz = max(0.1, float(self.get_parameter('poll_hz').value))
        idle_hz = max(0.1, float(self.get_parameter('idle_poll_hz').value))
        every = max(1, int(round(poll_hz / idle_hz)))
        if self._target_width_mm is not None or self._poll_tick % every == 0:
            self._send(gf.POSITION_FEEDBACK_ID, gf.encode_position_query())
            # 每问一次就记一笔：一条回帧都没来的话这个数一直涨，看门狗据此
            # 判"板子是不是真的不答了"（见 _check_link）。
            self._polls_since_reply += 1
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
        sent = self._send(gf.POSITION_FRAME_ID, gf.encode_position(revs))
        self.get_logger().info(
            '开口 {:.2f} mm → 位置 {:.4f} 圈{}'.format(
                width, revs, '' if sent else '（没发出去）'))
        # 从这一刻起盯着这个目标，等它停稳再由 _correct_once 补发。
        # 没发出去就不记"上一条命令"：补发时退回开环，别拿一条没发生的命
        # 令当基准（否则补发量会多出一整段）。
        self._target_width_mm = float(width)
        self._target_iters = 0
        self._command_revs = float(revs) if sent else None
        self._command_at = time.time()

    def _correct_once(self):
        """按回读补发一次位置，把"要 50 停在 52~54"的静差补掉。

        位置模式是比例型控制，负载越大停得越偏；它自己不会收敛到目标。

        补发量按"**上一条命令 + 这次实测差**"算，不是"2×目标 − 实测"。
        老写法的前提是上一条命令正好等于目标；一旦上一条已经补过（目标 50 mm
        而命令是 51.5 圈对应的值），它会把那次放大当成新误差继续放大 ——
        再加上 0.4 s 就去读回读（手指还在走），2026-10-30 真机日志里就出现了
        46.9 → 53.0 → 待发 51.5 的来回跳。新写法只补"命令与实际之间差的那一
        段"，静差一次补平，不会越补越大。
        """
        if self._target_width_mm is None or self._last_revs is None:
            return
        if not self._may_move:
            self._target_width_mm = None
            return
        if not self._settled():
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
                '开口停在 {:.1f} mm（要 {:.1f} mm）：补发 {} 次、每次都等停稳了'
                '才补，还差 {:+.1f} mm。先确认手指有没有顶到东西；没顶到就是'
                '"宽度↔圈数"这条线偏了，重新量零点（tools/gripper_home.py）'
                .format(self._last_width, self._target_width_mm,
                        self._target_iters, error))
            self._target_width_mm = None
            return
        goal = gf.width_to_revs(self._target_width_mm, self.anchor, self.span)
        base = (goal if self._command_revs is None
                else float(self._command_revs))
        command = base + (goal - float(self._last_revs))
        # 别把命令送出机械行程：比例控制下补过头会被固件限幅，读数停在限位上，
        # 看起来又像"顶到了"。
        ends = [gf.width_to_revs(w, self.anchor, self.span)
                for w in (gf.MIN_WIDTH_MM, gf.MAX_WIDTH_MM)]
        clamped = min(max(command, min(ends)), max(ends))
        if abs(clamped - command) > 1e-9:
            self.get_logger().warn(
                '补发量 {:.4f} 圈超出机械行程，按 {:.4f} 圈发'.format(
                    command, clamped))
            command = clamped
        self._target_iters += 1
        if self._enable_before_move:
            self._send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))
        if not self._send(gf.POSITION_FRAME_ID, gf.encode_position(command)):
            self.get_logger().error('补发位置没发出去，不再修')
            self._target_width_mm = None
            return
        self._command_revs = float(command)
        self._command_at = time.time()
        self.get_logger().info(
            '差 {:+.1f} mm（实测 {:.1f} mm；上一条命令 {:.4f} 圈），补发到 '
            '{:.4f} 圈（第 {} 次）'.format(
                error, self._last_width, base, command, self._target_iters))

    def _settled(self):
        """回读算"停稳"了吗？没停稳就别拿它算补发量。

        两条都要满足：命令发出已超过 `width_settle_s`，且最近两帧回读的差
        不超过 `width_settle_revs`。
        """
        if self._last_revs is None or self._prev_revs is None:
            return False
        if time.time() - self._command_at < float(
                self.get_parameter('width_settle_s').value):
            return False
        return abs(float(self._last_revs) - float(self._prev_revs)) <= float(
            self.get_parameter('width_settle_revs').value)

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
