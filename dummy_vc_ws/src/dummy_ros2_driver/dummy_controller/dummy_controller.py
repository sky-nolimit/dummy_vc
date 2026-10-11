import time
import rclpy
import asyncio

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState

import numpy as np
import dummy_cli_tool.ref_tool

class dummy_controller(LifecycleNode):
    def __init__(self):
        super().__init__('dummy_controller')
        self.dummy_driver = None

        # ------------------------------------------------------------------
        # 关节名必须与 dummy2_urdf 一致，否则 robot_state_publisher 匹配不上，
        # TF 树是死的（RViz 里机械臂不动，MoveIt 算不出正确末端位姿）。
        #
        # 上游原版用的是 joint_1..joint_6（下划线），URDF 用的是 dummy2_joint1..6，
        # 两者从来没有对齐过。这是阶段 1 修的第一个问题。
        # ------------------------------------------------------------------
        self.joint_names = [f'dummy2_joint{i}' for i in range(1, 7)]

        # 手指关节必须和主关节一起发布，否则这两段 TF 永远缺失
        self.finger_joint_names = ['dummy2_finger_joint1', 'dummy2_finger_joint2']

        # 两指各自的行程上限（米），来自 URDF 的 limit upper
        self.finger_travel_m = 0.023

        # 夹爪全开对应的设备角度（**度**）。标定前用占位值，
        # 只影响 RViz 里手指张开的幅度，不影响 TF 连通性。
        self.hand_max_angle = 90.0

        # ------------------------------------------------------------------
        # 设备角度 -> URDF 关节角 的换算
        #
        # 单位约定（已实测确认）：
        #   robot.joint_N.angle        单位是**度**
        #   robot.move_j(...)          单位是**度**
        #   robot.joint_N.set_position 单位是**度**
        #   URDF / MoveIt / robot_state_publisher  单位是**弧度**
        #
        # 实测依据：手动把机械臂摆到接近 7 字位时回读为
        #   [-0.14, 2.03, 2.30, -2.93, -2.32, 5.18]
        # 这些值作为「度」完全合理，作为「弧度」则远超关节行程。
        # 上游驱动把设备角度直接当弧度发布，于是 RViz 里的模型放大了
        # 180/pi ≈ 57 倍——这正是「模型转得比实物大得多」的原因。
        #
        # 换算式（统一在**度**下计算，最后再转弧度）：
        #   urdf_deg = sign * device_deg + offset_deg
        #
        # offset_deg 是结构性零位偏置。已知 j3 需要约 +90 度：设备的 j3
        # 零位对应 URDF 的 0 度（大臂竖直向下），而 7 字位在 URDF 里是
        # +90 度，设备回读却只有 2.3 度。
        #
        # sign 用来处理电机装配方向。参考项目 dummy_moveit_ws 用
        # [1,1,1,1,-1,-1]，即 j5/j6 反向。
        #
        # 这两个数组会被 ROS 参数覆盖，见 _declare_params。
        # ------------------------------------------------------------------
        self.joint_sign = [1.0] * 6
        self.joint_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

        # 位置参数：单位是**度**（与硬件 move_j 的约定一致）。
        # 注意这与 URDF/MoveIt 的弧度不同，转换只在本节点边界做。
        self.home_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.reset_joints = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])

        # 内部状态统一用**弧度**（ROS 惯例）
        self.current_joints = np.zeros(6)
        self.current_end_pose = np.zeros(6)

        # 轨迹执行
        self._traj_executing = False

        self.get_logger().info("dummy_controller_node Lifecycle node constructor called")
        self.get_logger().info(
            f'joint_sign={self.joint_sign} joint_offset_deg={self.joint_offset_deg}')

    def _declare_params(self):
        """声明并读取参数。on_configure 之后调用。"""
        default_sign = [1.0] * 6
        default_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        self.declare_parameter('joint_sign', default_sign)
        self.declare_parameter('joint_offset_deg', default_offset_deg)
        self.declare_parameter('home_joints_deg', [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.declare_parameter('reset_joints_deg', [0.0, -75.0, 180.0, 0.0, 0.0, 0.0])
        # 手指行程上限（米）与夹爪全开角度，用于把 hand.angle 折算成手指位移，
        # 让 robot_state_publisher 能发布手指关节的 TF
        self.declare_parameter('finger_travel_m', 0.023)
        self.declare_parameter('hand_max_angle', 90.0)

        self.joint_sign = list(self.get_parameter('joint_sign').value)
        self.joint_offset_deg = list(
            self.get_parameter('joint_offset_deg').value)
        self.home_joints = np.array(self.get_parameter('home_joints_deg').value)
        self.reset_joints = np.array(self.get_parameter('reset_joints_deg').value)
        self.finger_travel_m = float(self.get_parameter('finger_travel_m').value)
        self.hand_max_angle = float(self.get_parameter('hand_max_angle').value)

        if len(self.joint_sign) != 6 or len(self.joint_offset_deg) != 6:
            self.get_logger().error(
                'joint_sign / joint_offset_deg must have exactly 6 elements; '
                'falling back to defaults')
            self.joint_sign = [1.0] * 6
            self.joint_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

        self.get_logger().info(f'joint_sign (deg-space) = {self.joint_sign}')
        self.get_logger().info(
            f'joint_offset_deg       = {self.joint_offset_deg}')

    # ----------------------------------------------------------------------
    # 设备角度 <-> URDF 关节角
    #
    # 设备侧一律用**度**，URDF 侧一律用**弧度**。
    # 换算只在下面的函数里发生，别在其它地方再乘一次 pi/180。
    # ----------------------------------------------------------------------
    def device_to_urdf_deg(self, device_angles):
        """设备角度（度）-> URDF 关节角（**度**）。"""
        out = []
        for i in range(6):
            out.append(self.joint_sign[i] * float(device_angles[i])
                       + self.joint_offset_deg[i])
        return np.array(out)

    def device_to_urdf(self, device_angles):
        """设备角度（度）-> URDF 关节角（**弧度**）。

        这是 ROS 侧唯一应该使用的入口（/joint_states 需要弧度）。
        """
        return np.radians(self.device_to_urdf_deg(device_angles))

    def urdf_deg_to_device_deg(self, urdf_deg):
        """URDF 关节角（度）-> 设备角度（度）。"""
        out = []
        for i in range(6):
            s = self.joint_sign[i]
            if abs(s) < 1e-12:
                self.get_logger().error(f'joint_sign[{i}] is zero, using 1.0')
                s = 1.0
            out.append((float(urdf_deg[i]) - self.joint_offset_deg[i]) / s)
        return np.array(out)

    def urdf_to_device(self, urdf_rad):
        """URDF 关节角（弧度）-> 设备角度（**度**）。"""
        return self.urdf_deg_to_device_deg(
            np.degrees(np.asarray(urdf_rad, dtype=float)))

    # ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ Lifecycle node callback ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._declare_params()
        self.init_dummy()
        self.home_dummy()
        # t1 = time.time()
        
        # while True:
        #     if time.time() - t1 > 10:
        #         break
        
        # self.reset_dummy()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(1)
        self.get_logger().info("on_configure() called")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # self.home_dummy()
        self.init_pub_sub()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮  4绿光常亮  5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(4)
        self.get_logger().info("on_activate() called")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        # self.stop_dummy()
        # 设为7字位置
        self.home_dummy()

        self.reset_pub_sub()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(1)
        self.get_logger().info("on_deactivate() called")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        # self.stop_dummy()
        self.reset_dummy()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(2)
        self.get_logger().info("on_cleanup() called")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        # 先设置 RGB，再关闭驱动（off_dummy 会将 driver 置 None）
        self.dummy_driver.robot.set_rgb_mode(0)
        self.off_dummy()
        self.get_logger().info("on_shutdown() called")
        return TransitionCallbackReturn.SUCCESS
    # ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ Lifecycle node callback ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    def on_sigint_shutdown(self):
        self.get_logger().info("Ctrl+C received. Executing custom shutdown.")
        self.reset_dummy()
        t1 = time.time()
        while True:
            if time.time() - t1 > 5:
                break
        self.dummy_driver.robot.set_rgb_mode(0)
        self.reset_pub_sub()
        self.off_dummy()
        
    def init_dummy(self):
        if self.dummy_driver is not None:
            self.get_logger().warn("Dummy driver already initialized")
        else:
            self.dummy_driver = dummy_cli_tool.ref_tool.find_any()
        # 电机设为上电
        self.dummy_driver.robot.set_enable(True)
        
    def init_pub_sub(self):
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/dummy_arm/controller/pose_command',
            self.end_pos_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joints_callback,
            10
        )

        # Publishers
        self.j6_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/j6', 10)
        self.tool0_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/tool0', 10)
        # MoveIt2 / robot_state_publisher 默认订阅 /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF 不再由本节点发布：tool0 / gripper_tcp / camera_link 等全部在
        # dummy2_urdf.xacro 里静态定义，交给 robot_state_publisher。

        # Timer 50ms (20Hz)，6 次硬件往返约 30~60ms，50ms 更稳妥
        self.poll_timer = self.create_timer(0.05, self.call_poll_position)
        
    def call_poll_position(self):
        try:
            # 读取当前关节角度
            self.current_joints = self.current_joints_callback()
            # 读取当前末端位置
            self.current_end_pose = self.current_end_callback()
            self.pub_current_joints_and_end_pose()
        except Exception as e:
            self.get_logger().error(f'Poll position error: {str(e)}')
        
    def pub_current_joints_and_end_pose(self):
        # 发布当前关节角度
        self.pub_current_joints()
        # 发布当前末端位置
        self.pub_current_end_pose()

    def current_joints_callback(self):
        """读取设备原始关节角，换算成 URDF 关节角。

        注意：设备返回的是**原始角度**（弧度），必须经过 sign/offset 换算才
        能作为 URDF 的关节角。上游原版直接把原始值当 URDF 角度发布，所以
        j3 会差 90 度、j5/j6 会反向。
        """
        robot = self.dummy_driver.robot
        raw = [
            robot.joint_1.angle,
            robot.joint_2.angle,
            robot.joint_3.angle,
            robot.joint_4.angle,
            robot.joint_5.angle,
            robot.joint_6.angle,
        ]
        return self.device_to_urdf(raw)

    def current_end_callback(self):
        # 设备不支持末端位姿读取，返回零值
        # 如需末端位姿，需在此处添加正运动学计算
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def current_finger_positions(self):
        """两个手指的 prismatic 位置（米）。

        robot_state_publisher 只对**在 /joint_states 里收到位置**的关节发布 TF。
        手指关节如果一直不发布，dummy_hand_v1_1 -> dummy_leftfinger_v1_1 和
        -> dummy_rightfiger_v1_1 这两段变换就永远缺失，RViz 会报
        "No transform from [dummy_leftfinger_v1_1] to [base_link]"。

        当前还没有夹爪控制接口（阶段 3 做），所以这里只**回读** hand.angle
        并按下面的映射折算成手指位移，先让 TF 树完整。

        finger_travel_m = 0.023 m 是两个手指各自的行程上限（来自 URDF）。
        hand_max_angle 是夹爪全开对应的设备角度，标定前用占位值，
        只影响 RViz 里手指张开的幅度，不影响 TF 树的连通性。
        """
        # TODO(阶段 3)：接上 hand.set_angle 的控制闭环后，这里改为发布
        # 实际指令值而不是回读值。
        try:
            angle = float(getattr(self.dummy_driver.robot.hand, 'angle', 0.0))
        except Exception as exc:
            self.get_logger().warn(
                f'cannot read hand angle: {exc}', throttle_duration_sec=5.0)
            angle = 0.0

        max_angle = self.hand_max_angle
        if abs(max_angle) < 1e-9:
            frac = 0.0
        else:
            frac = max(0.0, min(1.0, angle / max_angle))

        travel = self.finger_travel_m * frac
        return travel, travel

    def pub_current_joints(self):
        try:
            left, right = self.current_finger_positions()

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            # 手指关节必须一起发布，否则这两段 TF 缺失（见方法上方说明）
            joint_state.name = list(self.joint_names) + list(self.finger_joint_names)
            joint_state.position = list(self.current_joints) + [left, right]
            joint_state.velocity = [0.0] * len(joint_state.name)

            self.joint_pub.publish(joint_state)

            self.get_logger().info(
                f'Publish JPOS: j1: {self.current_joints[0]:.2f}, j2: {self.current_joints[1]:.2f}, '
                f'j3: {self.current_joints[2]:.2f}, j4: {self.current_joints[3]:.2f}, '
                f'j5: {self.current_joints[4]:.2f}, j6: {self.current_joints[5]:.2f}, '
                f'fingers: {left:.4f}/{right:.4f}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'Parse joint error: {str(e)}')

    def pub_current_end_pose(self):
        """不发布手写 TF。

        上游原版在这里广播 joint6 -> tool0 和 joint6 -> camera_frame，
        但 URDF 里根本没有叫 joint6 的 frame（只有 dummy2_joint6 这个**关节**
        和 link7_v1_1 这些**链接**），相机节点的 frame_id 又是第三个名字
        fingerprint_camera。结果是 TF 树断裂，RViz 里坐标系对不上。

        阶段 1 起，tool0 / gripper_tcp / camera_link / camera_color_optical_frame
        全部在 dummy2_urdf.xacro 里静态定义，由 robot_state_publisher 统一发布。
        本节点只负责发 /joint_states，不再碰 TF。

        如果将来需要末端位姿，正确做法是从 /joint_states 做正运动学，
        或者直接用 tf2 查询 base_link -> gripper_tcp。
        """
        return

    def reset_pub_sub(self):
        # --- 銷毀原有 pub/sub ---
        if hasattr(self, 'pose_sub') and self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.get_logger().info("Destroyed pose_sub")
        if hasattr(self, 'joint_sub') and self.joint_sub is not None:
            self.destroy_subscription(self.joint_sub)
            self.get_logger().info("Destroyed joint_sub")
        
        if hasattr(self, 'j6_pose_pub') and self.j6_pose_pub is not None:
            self.destroy_publisher(self.j6_pose_pub)
            self.get_logger().info("Destroyed j6_pose_pub")
        if hasattr(self, 'tool0_pose_pub') and self.tool0_pose_pub is not None:
            self.destroy_publisher(self.tool0_pose_pub)
            self.get_logger().info("Destroyed tool0_pose_pub")
        if hasattr(self, 'joint_pub') and self.joint_pub is not None:
            self.destroy_publisher(self.joint_pub)
            self.get_logger().info("Destroyed joint_pub")

        if hasattr(self, 'poll_timer') and self.poll_timer is not None:
            self.destroy_timer(self.poll_timer)
            self.get_logger().info("Destroyed poll_timer")
    
    def _send_named_pose(self, urdf_deg, label):
        """把「URDF 弧度下的目标姿态」下发到硬件。

        输入是按**度**表示的 URDF 关节角（home_joints / reset_joints 的格式）。
        先转成弧度，再经 urdf_to_device 逆变换，最后由 move_j 按度下发。
        """
        urdf_rad = np.radians(np.asarray(urdf_deg, dtype=float))
        device_deg = self.urdf_to_device(urdf_rad)
        try:
            self.move_joints(device_deg)
            self.get_logger().info(
                f'{label}: urdf_deg={np.round(np.asarray(urdf_deg, dtype=float), 2).tolist()} '
                f'-> device_deg={np.round(device_deg, 2).tolist()}')
        except Exception as exc:
            self.get_logger().error(f'{label} failed: {exc}')

    def home_dummy(self, wait=True):
        self._send_named_pose(self.home_joints, 'home')

    def stop_dummy(self):
        # current_joints 已经是 URDF 弧度，先转回度再走同一条下发路径
        self._send_named_pose(np.degrees(self.current_joints), 'emergency stop')

    def reset_dummy(self, wait=True):
        self._send_named_pose(self.reset_joints, 'reset')

    async def check_joints_place(self, target_joints_deg):
        # target_joints_deg 是度，current_joints 是弧度，需要统一单位比较
        target_rad = np.radians(target_joints_deg)
        t1 = time.time()
        while True:
            if np.allclose(self.current_joints, target_rad, atol=0.01):
                break
            if time.time() - t1 > 10:
                self.get_logger().warn("Timeout waiting for joints to reach target")
                break
            await asyncio.sleep(0.05)  # 非阻塞等待

    def end_pos_callback(self, msg: PoseStamped):
        # 提取位置（单位：m）
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # 四元数转欧拉角（Roll, Pitch, Yaw），单位：度
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        # 使用 move_l（笛卡尔空间运动），不是 move_j
        self.dummy_driver.robot.move_l(x, y, z, roll, pitch, yaw)

    def joints_callback(self, msg: JointTrajectory):
        """执行一条关节轨迹。

        上游原版只取 msg.points[0]，等于把整条插值路径丢掉、直接跳到终点。
        MoveIt 规划出来的是几十个 waypoint，这样写轻则动作难看，重则瞬时
        大角度跳变损伤减速器。

        阶段 1 起按 time_from_start 节拍依次下发**全部** waypoint。

        joint_names 的顺序可能与我们的关节顺序不同，必须按名字重排，
        不能假设它们已经是 dummy2_joint1..6 的顺序。
        """
        if self._traj_executing:
            self.get_logger().warn(
                'A trajectory is already executing, ignoring the new one')
            return
        if not msg.points:
            self.get_logger().warn('Received JointTrajectory with no points')
            return

        # 按名字重排到我们的关节顺序
        order = self._trajectory_joint_order(msg.joint_names)
        if order is None:
            return

        self._traj_executing = True
        try:
            self._execute_trajectory(msg, order)
        finally:
            self._traj_executing = False

    def _trajectory_joint_order(self, traj_names):
        """把轨迹的关节顺序映射到本节点的顺序。

        返回 order，其中 order[i] 是「本节点第 i 个关节」在轨迹里的下标。
        名字对不上时返回 None。
        """
        if not traj_names:
            # 没给名字：只能假设顺序一致
            self.get_logger().warn(
                'JointTrajectory has empty joint_names, assuming order matches')
            return list(range(6))

        order = []
        for want in self.joint_names:
            if want in traj_names:
                order.append(traj_names.index(want))
                continue
            # 兼容上游命名，避免因为历史话题名直接拒绝执行
            legacy = 'joint_' + want.replace('dummy2_joint', '')
            if legacy in traj_names:
                self.get_logger().warn(
                    f'Using legacy joint name {legacy} for {want}; '
                    'update the publisher to dummy2_jointN')
                order.append(traj_names.index(legacy))
                continue
            self.get_logger().error(
                f'JointTrajectory does not contain {want} '
                f'(got {list(traj_names)}), refusing to execute')
            return None
        return order

    def _execute_trajectory(self, msg, order):
        total = len(msg.points)
        self.get_logger().info(
            f'Executing trajectory: {total} waypoints, '
            f'joint order map {order}')

        t_start = time.monotonic()
        for idx, point in enumerate(msg.points):
            if len(point.positions) < len(order):
                self.get_logger().error(
                    f'Waypoint {idx} has {len(point.positions)} positions, '
                    f'need at least {len(order)}; aborting')
                return

            # 轨迹顺序 -> 本节点顺序
            urdf_rad = np.array([point.positions[order[i]] for i in range(6)])

            # 按 time_from_start 节拍等待
            target_t = (point.time_from_start.sec
                        + point.time_from_start.nanosec * 1e-9)
            wait = t_start + target_t - time.monotonic()
            if wait > 0:
                time.sleep(wait)

            # URDF 弧度 -> 设备角度（含 sign/offset 逆变换），单位是度
            device_deg = self.urdf_to_device(urdf_rad)
            try:
                self.move_joints(device_deg)
            except Exception as exc:
                self.get_logger().error(
                    f'Waypoint {idx} command failed: {exc}; aborting trajectory')
                return

            if idx % 10 == 0 or idx == total - 1:
                self.get_logger().info(
                    f'  waypoint {idx + 1}/{total} '
                    f't={target_t:.2f}s deg={np.round(device_deg, 2).tolist()}')

        self.get_logger().info('Trajectory execution complete')

    def move_joints(self, joints_deg):
        """下发关节角度（度）。"""
        self.dummy_driver.robot.move_j(joints_deg[0], joints_deg[1], joints_deg[2],
                                       joints_deg[3], joints_deg[4], joints_deg[5])

    def off_dummy(self):
        self.dummy_driver.robot.set_enable(False)
        self.dummy_driver = None
        self.get_logger().info("Dummy driver powered off")

def main(args=None):
    rclpy.init(args=args)
    node = dummy_controller()
    shutdown_done = False
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Executing custom shutdown.")
        node.on_sigint_shutdown()
        shutdown_done = True
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
        if not shutdown_done:
            node.on_sigint_shutdown()
            shutdown_done = True
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

