"""把 `/gripper/command`（角度，度）变成夹爪板能听到的一帧 CAN。

为什么要有这个节点：主控固件里 `hand.set_angle` 会把整机锁死（`docs/04` A2：
`CanSendMessage` 只在发送完成回调里归还信号量，出错路径都不归还），所以夹爪
不能再走主控那条路。这个节点让 PC 经 USB-CAN 适配器**直接**对着夹爪板说话，
主控完全不知情，也就不受影响——主控继续管它的六个关节。

先虚拟总线试（不需要任何硬件）：

    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0
    ros2 run dummy_vc_grasp gripper_can_node --ros-args -p channel:=vcan0

另一头看帧对不对：

    candump vcan0            # 期望 382  [8]  00 00 A0 40 00 00 00 00

真适配器（UTC-T01 是 CANable 兼容，内核 gs_usb 直接给出 can0）：

    sudo modprobe gs_usb
    sudo ip link set can0 type can bitrate 1000000 sample-point 0.667
    sudo ip link set up can0
    ros2 run dummy_vc_grasp gripper_can_node --ros-args -p channel:=can0

发一帧（5 度，别一上来 30 度）：

    ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 5.0}"

**这个节点不代表夹爪能动。** 帧格式抄自参考固件，板子认不认还没验过；也拿不到
任何反馈（协议里 hand 只有三个写方法）。所以它只负责"把这一帧发出去"，发出去
之后夹爪到底动没动，只能靠眼睛看。
"""

import argparse
import math
import socket
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from dummy_vc_grasp import gripper_frames as gf


class GripperCanNode(Node):
    def __init__(self):
        super().__init__('gripper_can_node')

        #: SocketCAN 的接口名。虚拟总线是 vcan0，真适配器一般是 can0。
        self.declare_parameter('channel', 'vcan0')
        self.declare_parameter('command_topic', '/gripper/command')
        self.declare_parameter('state_topic', '/gripper/state')
        #: 一帧电流上限（0x381）先发出去，别让夹爪带着上次的电流限值动。
        self.declare_parameter('current_limit_a', 0.5)
        self.declare_parameter('send_current_on_start', True)
        #: 只打印不发送：用来在没开 vcan 的机器上先看一眼要发什么。
        self.declare_parameter('dry_run', False)

        self.channel = str(self.get_parameter('channel').value)
        self.dry_run = bool(self.get_parameter('dry_run').value)
        self.sock = None if self.dry_run else self._open_socket(self.channel)

        self.last_angle_deg = None
        self.frames_sent = 0

        self.state_pub = self.create_publisher(
            Float64, str(self.get_parameter('state_topic').value), 10)
        self.sub = self.create_subscription(
            Float64, str(self.get_parameter('command_topic').value),
            self._on_command, 10)

        if not self.dry_run and bool(self.get_parameter('send_current_on_start').value):
            self._send_current_limit(
                float(self.get_parameter('current_limit_a').value))

        self.get_logger().info(
            '夹爪通道就绪：{}（{}）。订阅 {}，发 0x{:03X}/0x{:03X}。'
            '注意这个节点拿不到任何反馈，夹爪动没动只能看。'.format(
                self.channel, '只打印不发送' if self.dry_run else 'SocketCAN 已打开',
                self.get_parameter('command_topic').value,
                gf.CURRENT_LIMIT_FRAME_ID, gf.ANGLE_FRAME_ID))

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
        return sock

    def _send(self, frame_id, payload):
        if self.dry_run:
            self.get_logger().info('（只打印）{}：{}'.format(
                gf.describe(frame_id, payload), payload.hex(' ')))
            return
        frame = gf.to_socketcan(frame_id, payload)
        try:
            self.sock.send(frame)
        except OSError as exc:
            # 总线 down、适配器拔了、发送队列满，都在这里。别把失败吞掉。
            self.get_logger().error('发不出去（0x{:03X}）：{}'.format(frame_id, exc))
            return
        self.frames_sent += 1
        self.get_logger().info('发 {}'.format(gf.describe(frame_id, payload)))

    # -- 命令 -------------------------------------------------------------
    def _send_current_limit(self, amperes):
        raw = float(amperes)
        value = min(max(raw, gf.MIN_CURRENT_A), gf.MAX_CURRENT_A)
        if value != raw:
            self.get_logger().warn(
                '电流上限 {:.3f} A 超出 [{:g}, {:g}]，按 {:.3f} A 发'.format(
                    raw, gf.MIN_CURRENT_A, gf.MAX_CURRENT_A, value))
        self._send(gf.CURRENT_LIMIT_FRAME_ID, gf.encode_current(value))

    def _on_command(self, msg):
        raw = float(msg.data)
        if math.isnan(raw) or math.isinf(raw):
            self.get_logger().warn('收到 {}，丢掉'.format(raw))
            return
        angle, clamped = gf.clamp_angle(raw)
        if clamped:
            self.get_logger().warn(
                '要 {:.2f} 度，夹爪板只认 {:g}~{:g} 度，按 {:.2f} 度发'.format(
                    raw, gf.MIN_ANGLE_DEG, gf.MAX_ANGLE_DEG, angle))
        self._send(gf.ANGLE_FRAME_ID, gf.encode_angle(angle))
        self.last_angle_deg = angle
        self.state_pub.publish(Float64(data=angle))


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
        if node.sock is not None:
            node.sock.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
