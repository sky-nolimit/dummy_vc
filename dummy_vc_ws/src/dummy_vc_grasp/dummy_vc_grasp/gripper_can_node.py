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
"""

import argparse
import math
import socket
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from dummy_vc_grasp import gripper_frames as gf


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
        #: 只打印不发送：用来在没开 can0 的机器上先看一眼要发什么。
        self.declare_parameter('dry_run', False)

        self.channel = str(self.get_parameter('channel').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.sock = None if self.dry_run else self._open_socket(self.channel)

        self.frames_sent = 0
        self._stop = threading.Event()
        self._reader = None
        self._last_reported_width = None
        self._last_finished = None

        self.position_pub = self.create_publisher(
            Float64, str(self.get_parameter('position_topic').value), 10)
        self.sub = self.create_subscription(
            Float64, str(self.get_parameter('width_topic').value),
            self._on_width, 10)

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
            '前提是板子已经标定过，否则位置命令会被固件那道闸挡掉。'.format(
                self.channel, '只打印不发送' if self.dry_run else 'SocketCAN 已打开',
                self.get_parameter('width_topic').value, gf.POSITION_FRAME_ID,
                gf.POSITION_FEEDBACK_ID,
                self.get_parameter('position_topic').value))

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
            width = gf.revs_to_width(revs)
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

    def _on_width(self, msg):
        raw = float(msg.data)
        if math.isnan(raw) or math.isinf(raw):
            self.get_logger().warn('收到 {}，丢掉'.format(raw))
            return
        width, clamped = gf.clamp_width(raw)
        if clamped:
            self.get_logger().warn(
                '要 {:.2f} mm，机械行程只有 {:g}~{:g} mm，按 {:.2f} mm 发'.format(
                    raw, gf.MIN_WIDTH_MM, gf.MAX_WIDTH_MM, width))
        revs = gf.width_to_revs(width)
        self._send(gf.POSITION_FRAME_ID, gf.encode_position(revs))
        self.get_logger().info(
            '开口 {:.2f} mm → 位置 {:.4f} 圈'.format(width, revs))

    def shutdown(self):
        self._stop.set()
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
