import time
import rclpy
import asyncio

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

import numpy as np
import dummy_cli_tool.ref_tool

class dummy_controller(LifecycleNode):
    def __init__(self):
        super().__init__('dummy_controller')
        self.dummy_driver = None
        
        # 7字位置
        # self.ready_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.home_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        # Reset位置，收起
        self.reset_joints = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])
        self.current_joints = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.current_end_pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

        self.get_logger().info("dummy_controller_node Lifecycle node constructor called")
    
    # ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ Lifecycle node callback ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    def on_configure(self, state: State) -> TransitionCallbackReturn:
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

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

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
        # 通过 robot.joint_N.angle 读取关节角度（返回弧度）
        robot = self.dummy_driver.robot
        j1 = robot.joint_1.angle
        j2 = robot.joint_2.angle
        j3 = robot.joint_3.angle
        j4 = robot.joint_4.angle
        j5 = robot.joint_5.angle
        j6 = robot.joint_6.angle
        return np.array([j1, j2, j3, j4, j5, j6])

    def current_end_callback(self):
        # 设备不支持末端位姿读取，返回零值
        # 如需末端位姿，需在此处添加正运动学计算
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    def pub_current_joints(self):
        try:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = self.current_joints.tolist()

            self.joint_pub.publish(joint_state)

            self.get_logger().info(
                f'Publish JPOS: j1: {self.current_joints[0]:.2f}, j2: {self.current_joints[1]:.2f}, '
                f'j3: {self.current_joints[2]:.2f}, j4: {self.current_joints[3]:.2f}, '
                f'j5: {self.current_joints[4]:.2f}, j6: {self.current_joints[5]:.2f}'
            )
        except Exception as e:
            self.get_logger().error(f'Parse joint error: {str(e)}')
    
    def pub_current_end_pose(self):
        try:
            # 设备不支持末端位姿读取，current_end_pose 全为零
            # 只发布静态 TF（tool0, camera_frame, isaac_frame），跳过动态 base_link→joint6
            # base_link→joint6 的 TF 应由 robot_state_publisher 根据 URDF + /joint_states 计算
            stamp = self.get_clock().now().to_msg()

            # --- Tool0 TF (relative to joint6) ---
            tool0_tf = TransformStamped()
            tool0_tf.header.stamp = stamp
            tool0_tf.header.frame_id = "joint6"
            tool0_tf.child_frame_id = "tool0"
            tool0_tf.transform.translation.x = 0.0
            tool0_tf.transform.translation.y = 0.0
            tool0_tf.transform.translation.z = 0.07
            tool0_tf.transform.rotation.x = 0.0
            tool0_tf.transform.rotation.y = 0.0
            tool0_tf.transform.rotation.z = 0.0
            tool0_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tool0_tf)

            # --- Camera TF (relative to joint6) ---
            camera_tf = TransformStamped()
            camera_tf.header.stamp = stamp
            camera_tf.header.frame_id = "joint6"
            camera_tf.child_frame_id = "camera_frame"
            camera_tf.transform.translation.x = -0.04
            camera_tf.transform.translation.y = 0.02
            camera_tf.transform.translation.z = 0.025
            camera_tf.transform.rotation.x = 0.0
            camera_tf.transform.rotation.y = 0.0
            camera_tf.transform.rotation.z = 0.0
            camera_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(camera_tf)

            # ISAAC TF
            r = R.from_euler('z', 90, degrees=True)
            isaac_quat = r.as_quat()
            isaac_tf = TransformStamped()
            isaac_tf.header.stamp = stamp
            isaac_tf.header.frame_id = "base_link"
            isaac_tf.child_frame_id = "isaac_frame"
            isaac_tf.transform.translation.x = 0.0
            isaac_tf.transform.translation.y = 0.0
            isaac_tf.transform.translation.z = 0.0
            isaac_tf.transform.rotation.x = isaac_quat[0]
            isaac_tf.transform.rotation.y = isaac_quat[1]
            isaac_tf.transform.rotation.z = isaac_quat[2]
            isaac_tf.transform.rotation.w = isaac_quat[3]
            self.tf_broadcaster.sendTransform(isaac_tf)
        except Exception as e:
            self.get_logger().error(f"Parse pose error: {str(e)}")
    
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

        if hasattr(self, 'tf_broadcaster') and self.tf_broadcaster is not None:
            self.tf_broadcaster = None
            self.get_logger().info("Destroyed tf_broadcaster")
        
        if hasattr(self, 'poll_timer') and self.poll_timer is not None:
            self.destroy_timer(self.poll_timer)
            self.get_logger().info("Destroyed poll_timer")
    
    def home_dummy(self, wait=True):
        self.dummy_driver.robot.move_j(
            self.home_joints[0], self.home_joints[1], self.home_joints[2],
            self.home_joints[3], self.home_joints[4], self.home_joints[5])
        # if wait:
        #     # 等待到位
        #     # self.check_joints_place(self.home_joints)
        #     asyncio.ensure_future(self.check_joints_place(self.home_joints))

    def stop_dummy(self):
        self.dummy_driver.robot.move_j(
            self.current_joints[0], self.current_joints[1], self.current_joints[2],
            self.current_joints[3], self.current_joints[4], self.current_joints[5])
        self.get_logger().info("Dummy Emergency Stop")

    def reset_dummy(self, wait=True):
        self.dummy_driver.robot.move_j(
            self.reset_joints[0], self.reset_joints[1], self.reset_joints[2],
            self.reset_joints[3], self.reset_joints[4], self.reset_joints[5])
        # if wait:
        #     # 等待到位
        #     asyncio.ensure_future(self.check_joints_place(self.reset_joints))
            # self.check_joints_place(self.reset_joints)
        # self.get_logger().info("Dummy reset the position")

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
        if not msg.points:
            self.get_logger().warn("Received JointTrajectory with no points")
            return

        positions = msg.points[0].positions
        if len(positions) < 6:
            self.get_logger().warn(f"Expected 6 joint values, got {len(positions)}")
            return
        # MoveIt2 发送的是弧度，move_j 需要度
        joints_deg = np.degrees(np.array(positions))
        self.move_joints(joints_deg)

    def move_joints(self, joints_deg):
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

