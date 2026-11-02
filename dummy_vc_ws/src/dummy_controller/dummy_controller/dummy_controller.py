import time
import signal
import threading
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
from std_msgs.msg import Float64

# MoveIt 的控制器插件只通过 action 下发轨迹，驱动必须自己当 server；两个
# import 都不能少，缺 GoalStatus 时模块里引用它会 NameError。
from control_msgs.action import FollowJointTrajectory
from action_msgs.msg import GoalStatus

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import dummy_cli_tool.ref_tool

# 关机时最多等多久让手臂回到收缩姿态（秒）。见 _retract_and_power_off()。
# 写死而不用参数：这是安全兜底、不是标定值，新增参数还要同步 YAML 与 launch
# 的 DESCRIPTIONS，否则 launch 直接 RuntimeError。6 秒来自 `ros2 launch` 的
# SIGTERM（默认 5 秒）与 SIGKILL（再 5 秒）：收缩、到位、断使能必须在这 10 秒
# 内做完，否则手臂还在动、使能已经掉了。
SHUTDOWN_RETRACT_TIMEOUT_SEC = 6.0

# 日志里用的关节名，顺序与设备一致。
HOME_JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3',
                    'joint_4', 'joint_5', 'joint_6']


def describe_home_gap(raw, reference, tol_deg):
    """比较"刚上电读到的六个角"与"标定 home 时记下的六个角"。

    返回 `(deltas, verdict)`：`deltas[i] = raw[i] - reference[i]`（度），
    `verdict` 是下面之一。

    * `no_reference` 没有 `home_reference_deg`（还没标定过），检查跳过。
    * `at_home`      六轴都在容差内 —— 手臂就停在标定时的位置。
    * `one_axis`     只有一两个轴偏了，其余都在容差内。这**不是**"手臂停在
                     别处"，而是"某个轴被转过/耦合打滑/标定过期"，模型从这一
                     刻起就是错的，值得警告。
    * `mixed`        三个以上、但不到六个偏了。说不清哪种，只报数不猜。
    * `elsewhere`    六个都偏了 —— 多半是把它停在了别的姿态，属于正常。

    区别就在这里：停错姿态是六轴一起偏，被碰过的是单轴偏。
    """
    if reference is None or len(reference) != 6:
        return [], 'no_reference'
    deltas = [float(a) - float(b) for a, b in zip(raw, reference)]
    off = [i for i, d in enumerate(deltas) if abs(d) > tol_deg]
    if not off:
        return deltas, 'at_home'
    if len(off) <= 2:
        return deltas, 'one_axis'
    if len(off) == 6:
        return deltas, 'elsewhere'
    return deltas, 'mixed'

class dummy_controller(LifecycleNode):
    def __init__(self):
        super().__init__('dummy_controller')
        self.dummy_driver = None

        # ------------------------------------------------------------------
        # 关节名必须与 dummy2_urdf 一致，否则 robot_state_publisher 匹配不上，
        # TF 树是死的（RViz 里机械臂不动，MoveIt 算不出正确末端位姿）。
        # 设备侧用的是 joint_1..joint_6，与 URDF 的 dummy2_joint1..6 对不上。
        # ------------------------------------------------------------------
        self.joint_names = [f'dummy2_joint{i}' for i in range(1, 7)]

        # 手指关节必须和主关节一起发布，否则这两段 TF 永远缺失
        self.finger_joint_names = ['dummy2_finger_joint1', 'dummy2_finger_joint2']

        # 两指各自的行程上限（米），来自 URDF 的 limit upper
        self.finger_travel_m = 0.023

        # 夹爪全开对应的设备角度（**度**）。标定前用占位值。
        self.hand_max_angle = 30.0

        # 夹爪控制量（归一化 0.0 = 全闭，1.0 = 全开）。
        # 目标与回读分开存：目标是我们要它去的，回读是它实际在哪。
        self.gripper_target = 0.0
        self.gripper_frac = 0.0

        # ------------------------------------------------------------------
        # 关节角度换算。基准定义与完整推导见
        # config/dummy_controller.yaml，以及下方 angle_to_urdf 等方法。
        #
        # 单位约定（已实测确认）：
        #   robot.joint_N.angle        度
        #   robot.move_j(...)          度
        #   robot.joint_N.set_position 度
        #   URDF / MoveIt / /joint_states  弧度
        # ------------------------------------------------------------------
        # 这几个默认值必须与 config/dummy_controller.yaml 完全一致：launch 从
        # YAML 读取并显式传入，这里的默认值只是兜底，不同步就会静默跑偏。
        # joint_sign 的 j2/j3 是 +1：读回与指令在 home/reset 之间的位移同向。
        self.joint_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
        self.joint_offset_deg = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
        self.move_j_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

        # 位置参数：单位是**度**，且是 **move_j 空间**（设备侧），**不是** URDF
        # 角度。下发时原样交给 move_j，不做任何变换。
        # home = 7 姿态（手臂自肩部水平前伸），对应 URDF 空间 [0,0,0,0,0,0]。
        self.home_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.reset_joints = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])

        # 内部状态统一用**弧度**（ROS 惯例）。名字里带 urdf_rad 是刻意的：
        # 存的是 URDF 关节角（弧度），**不是**设备的 angle 回读值（度），中间
        # 差一次 angle_to_urdf 换算；弄混就会把弧度直接当度下发。
        self.current_urdf_rad = np.zeros(6)
        self.current_end_pose = np.zeros(6)

        # 最后一次下发成功的 move_j 值，作为"设备当前命令位置"的权威来源。
        # 见 commanded_movej()：不用回读反推，因为那个反推式子没有物理依据。
        self._last_cmd_movej = None

        # 最后一次下发成功的夹爪角度。**夹爪没有回读** —— 设备上 hand 只有
        # set_angle / set_current_limit / set_enable，没有任何可读属性，所以
        # 发布手指位置只能用这个命令值，而不是测量值。
        self._last_gripper_angle = None

        # ------------------------------------------------------------------
        # 轨迹执行
        #
        # `_traj_executing` 是"机械臂正忙"的单一事实来源，**两条入口共用**：
        #   - 旧的话题订阅 /joint_trajectory_controller/joint_trajectory
        #   - MoveIt 用的 action server（见 init_trajectory_action_server）
        #
        # `_traj_lock` 保护它，因为有两个线程会碰：执行器线程（两条入口的回调）
        # 和 `_traj_thread`。没有这把锁时，"检查忙不忙"和"置忙"之间会被另一个
        # 回调插进来，两条轨迹就会同时在动 —— 设备的 move_j 是无状态的，结果
        # 是两个来源交替下发，机械臂走出一条谁都没规划过的路。
        #
        # 执行必须放到线程里：`_execute_trajectory` 按 time_from_start 逐点
        # sleep，一条轨迹动辄几十秒；默认的单线程执行器里它在回调里睡多久，
        # pub_timer（/joint_states）就停多久 —— MoveIt 和 RViz 会看到机器人
        # "消失"，而 execute 的成功判据恰恰依赖 /joint_states。
        # fibre 协议层自己带锁，所以从别的线程调 move_j / 读关节是安全的。
        # ------------------------------------------------------------------
        self._traj_executing = False
        self._traj_lock = threading.Lock()
        self._traj_thread = None
        self._traj_action_server = None
        # 进度日志的上一次打印时刻。给初值，免得在 _execute_trajectory 之外
        # 调用 _progress_log_due() 时 AttributeError。
        self._last_progress_log = 0.0

        # ------------------------------------------------------------------
        # 关机（安全相关）
        #
        # `_shutdown_lock` + `_shutdown_done` 保证"回收缩姿态、等它到位、再断
        # 使能"**只做一次**。退出路径不止一条（lifecycle shutdown、Ctrl+C、
        # spin() 正常返回），可能并发进来；后进来的阻塞在锁上，看到 done 就
        # 返回，这样 main() 的 finally 不会在手臂还在动时拆节点。
        #
        # `_shutting_down` 是给轨迹线程看的：关机时必须先把正在下发的
        # waypoint 停掉，否则收缩姿态的命令会和轨迹的 waypoint 交替发给设备，
        # 机械臂走出一条谁都没规划过的路。
        # ------------------------------------------------------------------
        self._shutdown_lock = threading.Lock()
        self._shutdown_done = False
        self._shutting_down = False

        self.get_logger().info("dummy_controller_node Lifecycle node constructor called")
        self.get_logger().info(
            f'joint_sign={self.joint_sign} joint_offset_deg={self.joint_offset_deg}')

    def _declare_params(self):
        """声明并读取参数。on_configure 之后调用。"""
        default_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
        # (1) 7 姿态的 angle 回读就是该姿态的偏置（此时 URDF 应为全零）
        default_offset_deg = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
        # (2) 下发链零点。与 (1) 之和必须等于 home_joints_deg —— 这是锚点
        #     处唯一能被约束到的关系，两者怎么分配不影响锚点行为，要靠实测
        #     配对验证。
        default_movej_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        # (3) URDF 空间里的附加偏置，默认全零。只在模型与实物**转向相同但
        #     整体差一个固定角度**时改这里；**转向相反**应改 joint_sign。
        default_urdf_offset_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.declare_parameter('joint_sign', default_sign)
        self.declare_parameter('joint_offset_deg', default_offset_deg)
        self.declare_parameter('move_j_offset_deg', default_movej_offset_deg)
        self.declare_parameter('urdf_offset_deg', default_urdf_offset_deg)
        # ---- 启动位姿核对 --------------------------------------------------
        # 标定 home 时的六个原始角不单独存成参数，而是由 joint_offset_deg /
        # joint_sign / urdf_offset_deg 反推（见 _home_reference）：它就是上面
        # 三张表的函数，存一份只会多一个不同步的地方。
        self.declare_parameter('home_check_tol_deg', 1.0)
        # homing 之后等手臂走到位：超时时间与容差。见 _wait_for_pose 的说明。
        self.declare_parameter('home_timeout_s', 20.0)
        self.declare_parameter('home_tol_deg', 1.5)
        self.declare_parameter('home_joints_deg', [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.declare_parameter('reset_joints_deg', [0.0, -75.0, 180.0, 0.0, 0.0, 0.0])
        # ---- 夹爪 ----------------------------------------------------------
        # 手指行程上限（米），来自 URDF 的 limit upper
        self.declare_parameter('finger_travel_m', 0.023)
        # 夹爪全闭 / 全开对应的设备角度（**度**）。范围 0~30 由固件源码确定
        # （dummy_robot.cpp 的 DummyHand::SetAngle 会把 _angle 夹到 [0,30]），
        # 所以全开是 30 而不是 90。方向（0 是全闭还是全开）仍未实测。
        self.declare_parameter('grip_angle_closed', 0.0)
        self.declare_parameter('grip_angle_open', 30.0)
        # 夹持力上限。这是唯一能防"夹碎"的手段。
        #
        # **默认 0.1 是刻意取小的**：取大了第一次装夹就可能压坏东西，应从
        # 极小值开始扫、实测后再填回配置。固件里 DummyHand::maxCurrent 的
        # 默认值是 0.7（范围 0~1），对未标定的夹爪偏大，所以没有沿用 ——
        # DummyHand::SetEnable(true) 做的事就是 SetMaxCurrent(0.7)。
        self.declare_parameter('grip_current_limit', 0.1)
        # 兼容旧参数名，避免已有配置直接报"未声明参数"
        self.declare_parameter('hand_max_angle', 30.0)
        # move_j 下发前的整圈归一化区间（仅在拿不到当前姿态时的回退路径使用）
        self.declare_parameter('wrap_min_deg', 0.0)
        self.declare_parameter('wrap_max_deg', 360.0)
        # 单次 move_j 允许的最大行程（度）。超过就报错而不是静默下发。
        #
        # 设备**会静默忽略够不着的设定值**，所以"下发成功但机械臂不动"能藏
        # 很久。默认 180：一个关节一次转过半圈以上，几乎肯定是整圈折算选错
        # 了，或者目标姿态本身有问题。
        self.declare_parameter('max_move_jump_deg', 180.0)
        # 参考位姿微调（度，逐关节）。见配置文件的说明：只补偿驱动用来做
        # 整圈折算的"当前位置"，不改变 homing 到达的物理姿态。
        self.declare_parameter('movej_current_trim_deg',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # 状态读取与发布的频率，见 init_pub_sub 两个定时器的说明。
        # 读得慢、发得快：读数受阻塞式 USB 往返限制，发布不受。
        self.declare_parameter('state_read_hz', 10.0)
        self.declare_parameter('joint_state_pub_hz', 50.0)
        # move_j 可接受区间，逐关节下界/上界交替，共 12 个数，单位度。
        # **实测得来**。区间外的值被设备静默丢弃，整条命令一起作废。
        self.declare_parameter('movej_accepted_range_deg', [
            -165.0, 165.0, -75.0, 90.0, 45.0, 180.0,
            -180.0, 165.0, -90.0, 90.0, -180.0, 180.0])
        # URDF 全零时应当下发的 move_j 值，逐关节，单位度。
        #
        # **这是整条下发链唯一的常数项**，取代了把 joint_offset_deg 和
        # move_j_offset_deg 相加的写法：那个写法对 j3 给出 -1.474，落在设备
        # 可接受区间 [45, 180] 之外，整条命令被静默丢弃。
        #
        # 取值就是 home_joints_deg：URDF 全零 = 7 姿态。
        self.declare_parameter('movej_urdf_zero_deg',
                               [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        # 设备连接超时（秒）。**不要设为 0 或负数**：fibre 的 find_any 在
        # timeout=None 时会永久阻塞，没有设备时会让启动静默卡死。
        self.declare_parameter('connect_timeout_sec', 30.0)
        # 夹爪初始化超时（秒）。夹爪的 fibre 调用会阻塞，若在 on_configure
        # 里同步等待就会冻死整个节点，连 /joint_states 都不发。
        self.declare_parameter('gripper_init_timeout_sec', 5.0)
        # 是否在启动时初始化夹爪。**这个开关已经无效。**
        #
        # 主控这条通道封掉了，原因见 `_init_gripper_with_timeout`：固件把
        # 0x382 当"设角度"发出去，夹爪板把它当"触发编码器标定"；0x381 被固件
        # 当"电流上限"，夹爪板当 uint32 使能位。调用一次就可能擦掉夹爪的标定
        # 表，旧固件还会因为发送卡死拖垮整机、只能断电。夹爪现在由
        # `dummy_vc_grasp/gripper_can_node` 经 USB-CAN 适配器直接驱动
        # （`/gripper/width`，单位毫米）。
        #
        # 传 true 不报错、也不再发任何帧，只在日志里说"已忽略"。
        self.declare_parameter('gripper_enable', False)
        # 夹爪是否初始化成功。失败、超时或被禁用后为 False。
        self.gripper_ready = False

        # ------------------------------------------------------------------
        # MoveIt 执行接口
        # ------------------------------------------------------------------
        self.declare_parameter('trajectory_action_enable', True)
        # action server 名必须与 dummy_vc_moveit_config/config/moveit_controllers.yaml
        # 里的组合**逐字相等**：
        #     /<controller_name>/<action_ns>
        # controller_name = joint_trajectory_controller
        # action_ns       = follow_joint_trajectory
        # 也就是 /joint_trajectory_controller/follow_joint_trajectory。
        #
        # **只有两段。** action_ns 本身就叫 follow_joint_trajectory，很容易
        # 顺势再补一个 —— 那是错的，而且报错文本和"名字少一段"完全一样：
        #     Action client not connected to action server: ...
        # 规划一路是绿的，所以这个错误很容易被当成规划问题。
        self.declare_parameter(
            'trajectory_action_name',
            '/joint_trajectory_controller/follow_joint_trajectory')
        # 发给客户端的反馈频率（Hz）。**不能太高**：反馈要和 joint_states
        # 抢同一个执行器线程，而且它不发也能让 MoveIt 正常收尾（它等的是
        # result）。0 或负数 = 不发。
        #
        # 目前这个值只决定**执行进度日志**打多密（见 _execute_trajectory 里的
        # _progress_log_due）。goal_handle.publish_feedback() 没有实现：feedback
        # 只影响 RViz 里的进度条，在设备线程上发布它的线程安全性未验证。
        self.declare_parameter('trajectory_feedback_hz', 5.0)

        self.joint_sign = list(self.get_parameter('joint_sign').value)
        self.joint_offset_deg = list(
            self.get_parameter('joint_offset_deg').value)
        self.move_j_offset_deg = list(
            self.get_parameter('move_j_offset_deg').value)
        self.urdf_offset_deg = list(
            self.get_parameter('urdf_offset_deg').value)
        self.home_reference_deg = self._home_reference()
        self.home_check_tol_deg = float(
            self.get_parameter('home_check_tol_deg').value)
        self.home_timeout_s = float(self.get_parameter('home_timeout_s').value)
        self.home_tol_deg = float(self.get_parameter('home_tol_deg').value)
        self.home_joints = np.array(self.get_parameter('home_joints_deg').value)
        self.reset_joints = np.array(self.get_parameter('reset_joints_deg').value)
        self.finger_travel_m = float(self.get_parameter('finger_travel_m').value)
        self.hand_max_angle = float(self.get_parameter('hand_max_angle').value)
        self.grip_angle_closed = float(
            self.get_parameter('grip_angle_closed').value)
        self.grip_angle_open = float(
            self.get_parameter('grip_angle_open').value)
        self.grip_current_limit = float(
            self.get_parameter('grip_current_limit').value)
        # 参数值可能是 bool，也可能被 launch 以字符串 'true'/'false' 传进来。
        raw_enable = self.get_parameter('gripper_enable').value
        if isinstance(raw_enable, str):
            self.gripper_enable = raw_enable.strip().lower() in (
                '1', 'true', 'yes', 'on')
        else:
            self.gripper_enable = bool(raw_enable)
        # 与 gripper_enable 同样的理由：launch 传进来的布尔值可能是字符串。
        raw_act = self.get_parameter('trajectory_action_enable').value
        if isinstance(raw_act, str):
            self.trajectory_action_enable = raw_act.strip().lower() in (
                '1', 'true', 'yes', 'on')
        else:
            self.trajectory_action_enable = bool(raw_act)
        self.trajectory_action_name = str(
            self.get_parameter('trajectory_action_name').value)
        if not self.trajectory_action_name.startswith('/'):
            # rclpy 会把它解析成 "<节点命名空间>/<这个名字>"。加不加都能连上，
            # 但日志里显示的名字会不一样，排查时白费时间，所以统一成绝对名。
            self.get_logger().warn(
                f'trajectory_action_name="{self.trajectory_action_name}" is '
                'not absolute; prefixing "/" so the name in the logs matches '
                'the one MoveIt looks for')
            self.trajectory_action_name = '/' + self.trajectory_action_name
        self.trajectory_feedback_hz = float(
            self.get_parameter('trajectory_feedback_hz').value)
        self.connect_timeout_sec = float(
            self.get_parameter('connect_timeout_sec').value)
        self.movej_current_trim_deg = list(
            self.get_parameter('movej_current_trim_deg').value)
        if len(self.movej_current_trim_deg) != 6:
            self.get_logger().error(
                'movej_current_trim_deg must have exactly 6 elements; '
                'ignoring it and using zeros')
            self.movej_current_trim_deg = [0.0] * 6
        self.movej_urdf_zero_deg = list(
            self.get_parameter('movej_urdf_zero_deg').value)
        if len(self.movej_urdf_zero_deg) != 6:
            self.get_logger().error(
                'movej_urdf_zero_deg must have exactly 6 elements; falling '
                'back to home_joints (URDF zero is the 7 pose)')
            self.movej_urdf_zero_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        if self.connect_timeout_sec <= 0.0:
            self.get_logger().warn(
                f'connect_timeout_sec={self.connect_timeout_sec} would block '
                'forever on a missing device; using 30.0 instead')
            self.connect_timeout_sec = 30.0
        if abs(self.grip_angle_open - self.grip_angle_closed) < 1e-6:
            self.get_logger().error(
                'grip_angle_open and grip_angle_closed are equal; the finger '
                'mapping would divide by zero. Falling back to 0..30 deg, '
                'which is the range the firmware clamps SetAngle to.')
            self.grip_angle_closed = 0.0
            self.grip_angle_open = 30.0

        for name, val in (('joint_sign', self.joint_sign),
                          ('joint_offset_deg', self.joint_offset_deg),
                          ('move_j_offset_deg', self.move_j_offset_deg),
                          ('urdf_offset_deg', self.urdf_offset_deg)):
            if len(val) != 6:
                self.get_logger().error(
                    f'{name} must have exactly 6 elements, got {len(val)}; '
                    'falling back to defaults')
                self.joint_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
                self.joint_offset_deg = [0.215, 74.572, -91.474, -0.910,
                                         -0.035, -0.040]
                self.move_j_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
                self.urdf_offset_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                break

        self.get_logger().info(f'joint_sign            = {self.joint_sign}')
        self.get_logger().info(f'joint_offset_deg     = {self.joint_offset_deg}')
        self.get_logger().info(f'move_j_offset_deg    = {self.move_j_offset_deg}')
        self.get_logger().info(f'urdf_offset_deg      = {self.urdf_offset_deg}')

    # ----------------------------------------------------------------------
    # 坐标换算（改之前务必先读 config/dummy_controller.yaml 的推导记录）
    #
    # 三个空间：
    #   angle    robot.joint_N.angle 的原始回读（度）—— 设备的物理量
    #   move_j   robot.move_j 的指令值（度）—— 设备的指令量
    #   URDF     URDF / MoveIt / /joint_states（弧度）
    #
    # 两条链，各自独立，符号在 URDF 边界上统一施加：
    #   urdf_deg = sign * (angle - joint_offset_deg)
    #   move_j   = urdf_deg / sign + move_j_offset_deg
    #
    # 除以 sign 而不是乘，是因为 sign 的物理含义是"URDF 正方向相对设备
    # 正方向是否翻转"，所以进 URDF 时乘、出 URDF 时除。
    #
    # sign 的确定方法：给某个关节一个正向增量，看实物与 RViz 模型的转向
    # 是否一致；实物逆时针而模型顺时针则该关节为 -1。一次只改一个关节。
    #
    # 一致性锚点（7 姿态，物理已验证）：
    #   angle  = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
    #   urdf   = [0, 0, 0, 0, 0, 0]
    #   move_j = [0, 0, 90, 0, 0, 0]
    # 两个偏置都由此推出，所以锚点处 sign 取任何值都仍映射到 0。
    # ----------------------------------------------------------------------
    def _apply_sign(self, sign, vals):
        return np.array([sign[i] * float(vals[i]) for i in range(6)])

    def _urdf_offsets(self):
        """URDF 空间里的零点偏置（度），逐关节。

        含义：**设备停在 7 姿态（= 模型的零位）时，每个关节在 URDF 空间里的
        真实角度**。底座装歪两度、腕部少校三度，都属于这里要补的。

        它必须同时进两条链，否则只修一半：

            回读  urdf   = sign * (angle - joint_offset_deg) + off
            下发  move_j = (urdf - off) / sign + movej_urdf_zero_deg

        只进回读：模型显示对了，手臂仍按老零点走；只进下发：反过来。
        """
        raw = self.get_parameter('urdf_offset_deg').value
        try:
            vals = [float(v) for v in raw]
        except TypeError:
            vals = []
        if len(vals) != 6:
            if vals:
                self.get_logger().error(
                    'urdf_offset_deg must have exactly 6 elements; ignoring '
                    'it rather than applying a half-specified correction')
            vals = [0.0] * 6
        return vals

    def _home_reference(self):
        """标定 home 时的六个原始角（度）—— 由三张表反推，不单独存。

        URDF 全零时 `angle = (0 - off) / sign + joint_offset_deg`，所以这组数
        完全由 joint_offset_deg / joint_sign / urdf_offset_deg 决定；存一份就
        多一个会不同步的地方，改了 urdf_offset_deg 忘了改它，启动核对就会天天
        报"某个轴被碰过"。
        """
        off = self._urdf_offsets()
        ref = []
        for i in range(6):
            try:
                joff = float(self.joint_offset_deg[i])
            except (TypeError, ValueError):
                return None
            ref.append((0.0 - off[i]) / self._safe_sign(i) + joff)
        return ref

    def _check_home_pose(self):
        """上电第一件事：这六轴现在到底在哪？与标定 home 差多少？

        这一步只读、不发命令、不改状态。必须在 homing 之前做：homing 会把
        手臂拉到按当前标定算出的"home"，那之后看到的"歪"就分不清是标定错
        还是它本来就歪。
        """
        if self.home_reference_deg is None:
            self.get_logger().warn(
                'startup pose check: could not derive the calibrated home from '
                'joint_offset_deg / urdf_offset_deg (a table is malformed); '
                'skipping -- this is NOT the same as "the pose is fine"')
            return
        try:
            raw = self.read_raw_angles()
        except Exception as exc:        # noqa: BLE001
            self.get_logger().warn(
                f'startup pose check: could not read the angles '
                f'({type(exc).__name__}: {exc}); skipping -- this is NOT the '
                f'same as "the pose is fine"')
            return

        deltas, verdict = describe_home_gap(
            raw, self.home_reference_deg, self.home_check_tol_deg)
        table = ', '.join(f'{name} {d:+.2f}'
                          for name, d in zip(HOME_JOINT_NAMES, deltas))
        worst = max(range(6), key=lambda i: abs(deltas[i]))
        self.get_logger().info(
            f'startup pose check: raw = {[round(v, 3) for v in raw]}')
        self.get_logger().info(
            f'startup pose check: raw - home_reference = {table} '
            f'(tolerance {self.home_check_tol_deg:.1f} deg)')

        if verdict == 'at_home':
            self.get_logger().info(
                f'startup pose check: the arm sits on the calibrated home '
                f'(largest gap {deltas[worst]:+.2f} deg on '
                f'{HOME_JOINT_NAMES[worst]})')
        elif verdict == 'one_axis':
            off = [HOME_JOINT_NAMES[i] for i, d in enumerate(deltas)
                   if abs(d) > self.home_check_tol_deg]
            self.get_logger().warn(
                f'startup pose check: only {"/".join(off)} is off the '
                f'calibrated home, the other axes are where they should be. '
                f'That is the signature of a joint that was turned by hand, a '
                f'coupling that slipped, or a calibration that went stale -- '
                f'NOT of an arm parked in another pose. The model will be '
                f'wrong by that amount until it is re-checked '
                f'(the home pose check).')
        elif verdict == 'elsewhere':
            self.get_logger().info(
                'startup pose check: all six axes are off the calibrated home, '
                'which is what parked-in-another-pose looks like. Homing will '
                'bring it back.')
        else:
            self.get_logger().warn(
                f'startup pose check: {sum(1 for d in deltas if abs(d) > self.home_check_tol_deg)} '
                f'of six axes are off the calibrated home ({table}); not '
                f'enough to tell "parked elsewhere" from "something moved", '
                f'so just take note of it.')

    def angle_to_urdf_deg(self, angle_deg):
        """angle 空间（度）-> URDF 关节角（**度**）。"""
        off = self._urdf_offsets()
        shifted = [float(angle_deg[i]) - self.joint_offset_deg[i]
                   for i in range(6)]
        signed = self._apply_sign(self.joint_sign, shifted)
        return np.array([float(signed[i]) + off[i] for i in range(6)])

    def angle_to_urdf(self, angle_deg):
        """angle 空间（度）-> URDF 关节角（**弧度**）。回读发布的唯一入口。"""
        return np.radians(self.angle_to_urdf_deg(angle_deg))

    def urdf_to_angle(self, urdf_rad):
        """URDF 关节角（弧度）-> angle 空间（度）。angle_to_urdf 的逆。"""
        off = self._urdf_offsets()
        urdf_deg = np.degrees(np.asarray(urdf_rad, dtype=float))
        signed = [(float(urdf_deg[i]) - off[i]) / self._safe_sign(i)
                  for i in range(6)]
        return np.array([signed[i] + self.joint_offset_deg[i]
                         for i in range(6)])

    def _safe_sign(self, i):
        s = self.joint_sign[i]
        if abs(s) < 1e-12:
            self.get_logger().error(
                f'joint_sign[{i}] is zero; treating as 1.0')
            return 1.0
        return s

    def commanded_movej(self):
        """本节点最后一次下发成功的 move_j 值。

        `move_j` 是只写不读的接口，设备不回读命令值；用原始回读反推
        （`raw - move_j_offset_deg`）没有物理依据，会把 j3 的值推出设备
        可接受区间，整条命令被静默丢弃。驱动自己知道它发过什么，所以直接用
        命令历史。初值取 None 表示"还没发过"，首次使用时回退到回读反推。
        """
        return self._last_cmd_movej

    def _remember_cmd_movej(self, movej_deg):
        self._last_cmd_movej = [float(v) for v in movej_deg]

    def guess_current_movej(self):
        """当前位置的 move_j 估计值：优先用命令历史，否则回退到回读反推。

        回退路径只用于第一次下发之前；一旦发过命令，命令历史就是准确的。
        """
        if self._last_cmd_movej is not None:
            return [self._last_cmd_movej[i] + self.movej_current_trim_deg[i]
                    for i in range(6)]
        try:
            raw_deg = self.read_raw_angles()
            return [float(raw_deg[i]) - self.move_j_offset_deg[i]
                    + self.movej_current_trim_deg[i]
                    for i in range(6)]
        except Exception:  # noqa: BLE001
            return None

    def normalize_movej(self, movej_deg, current_deg=None):
        """把 move_j 目标折算进设备的可接受区间。

        **设备有一个实测出来的、很窄而且偏离零点的可接受区间**：

            joint_1 [-165, 165]   joint_2 [-75, 90]    joint_3 [45, 180]
            joint_4 [-180, 165]   joint_5 [-90, 90]    joint_6 [-180, 180]

        不能把它当成绕原点的一整圈：j3 的最小可接受值是 +45，任何整圈平移
        都可能仍落在区间外。正确做法是把每个目标平移整圈、落到该关节的
        [lo, hi] 之内；区间可能跨过 0/360 接缝（例如 [-180, 165]），所以用
        带符号的取模而不是夹取。

        找不到合法等价值时**报错**，不静默下发 —— 区间外的值被设备静默丢弃、
        整条命令不执行，静默失败正是这件事能藏很久的原因。
        """
        rng = list(self.get_parameter('movej_accepted_range_deg').value)
        if len(rng) != 12:
            self.get_logger().error(
                'movej_accepted_range_deg must hold 12 numbers (lo,hi per '
                'joint); falling back to a permissive full turn')
            rng = [-180.0, 180.0] * 6

        limit = float(self.get_parameter('max_move_jump_deg').value)
        out = []
        for i in range(6):
            target = float(movej_deg[i])
            lo, hi = float(rng[2 * i]), float(rng[2 * i + 1])
            width = hi - lo
            if width <= 0.0 or width >= 359.999:
                # A full turn (or a bad interval) means every value is
                # reachable, so just take the nearest whole-turn equivalent.
                best = target
                if current_deg is not None:
                    cur = float(current_deg[i])
                    best = target + 360.0 * round((cur - target) / 360.0)
                out.append(best)
                continue

            # Fold into [lo, hi] by moving to the interval's own frame.
            x = (target - lo) % 360.0
            candidates = [x - 360.0, x, x + 360.0]
            inside = [c for c in candidates if -1e-9 <= c <= width + 1e-9]
            if inside:
                # Prefer the equivalent that is closest to where we are.
                if current_deg is not None:
                    cur = float(current_deg[i])
                    best = min(inside,
                               key=lambda c: abs((lo + c) - cur))
                else:
                    best = inside[0]
                best = lo + best
            else:
                # Nothing lands inside this joint's accepted interval: the
                # target pose is outside what the device will do at all.
                self.get_logger().error(
                    f'joint{i + 1}: target {target:.2f} deg has no equivalent '
                    f'inside the accepted interval [{lo:.1f}, {hi:.1f}]. The '
                    'device would drop the WHOLE command silently. Refusing '
                    'it instead -- this means the target pose is outside the '
                    'joint travel, or the calibration is inconsistent.')
                out.append(target)
                continue

            travel = best - (float(current_deg[i])
                             if current_deg is not None else best)
            if current_deg is not None and abs(travel) > limit:
                self.get_logger().error(
                    f'joint{i + 1}: folding {target:.2f} into [{lo:.1f}, '
                    f'{hi:.1f}] gives {best:.2f}, which is {travel:+.2f} deg '
                    f'from the current {float(current_deg[i]):.2f}, beyond '
                    f'max_move_jump_deg={limit:.0f}. Refusing it rather than '
                    'sending something the arm cannot reach.')
            out.append(best)
        return np.array(out)

    def urdf_to_movej(self, urdf_rad, current_deg=None):
        """URDF 关节角（弧度）-> move_j 空间（度）。

        这是把 MoveIt 轨迹点转成设备指令的唯一入口。下滑链是**两步**：

            angle  = urdf_deg / sign + joint_offset_deg
            move_j = angle + move_j_offset_deg
                   = urdf_deg / sign + joint_offset_deg + move_j_offset_deg

        漏掉 joint_offset_deg 会把 j3 的值推出可接受区间 [45, 180]，设备静默
        丢弃整条命令；而锚点处 urdf = 0，两个 offset 只以**和**的形式出现，
        所以这个错靠锚点查不出来，必须用实测的 (move_j, raw) 配对定住。

        current_deg 是设备当前位置（move_j 空间），用于把结果折算进该关节的
        可接受区间。**不传它就不做区间折算**，正常路径都应传入。
        """
        urdf_deg = np.degrees(np.asarray(urdf_rad, dtype=float))
        # 常数项经 get_parameter 读取，而不是用实例属性 —— 这样只 exec 方法、
        # 不跑 on_configure 的测试桩也能驱动这个函数。
        zero = list(self.get_parameter('movej_urdf_zero_deg').value)
        if len(zero) != 6:
            self.get_logger().error(
                'movej_urdf_zero_deg must have exactly 6 elements; using the '
                '7 pose values instead')
            zero = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        off = self._urdf_offsets()
        out = np.array([(float(urdf_deg[i]) - off[i]) / self._safe_sign(i)
                        + zero[i] for i in range(6)])
        return self.normalize_movej(out, current_deg)

    def movej_to_angle(self, movej_deg):
        """move_j 空间（度）-> angle 空间（度）。angle_to_movej 的逆。

        `angle_to_movej` 是 `move_j = angle - joff + zero`，所以逆变换就是

            angle = move_j - zero + joff

        **必须与 angle_to_movej 成对修改**：二者各自看着"合理"却互不逆。
        """
        zero = list(self.get_parameter('movej_urdf_zero_deg').value)
        if len(zero) != 6:
            self.get_logger().error(
                'movej_urdf_zero_deg must have exactly 6 elements; using the '
                '7 pose values instead')
            zero = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        return np.array([float(movej_deg[i]) - zero[i]
                         + self.joint_offset_deg[i] for i in range(6)])

    def angle_to_movej(self, angle_deg):
        """angle 空间（度）-> move_j 空间（度）。movej_to_angle 的逆。

        必须与两步链**代数等价**。两步链是

            angle -> URDF  :  urdf   = sign * (angle - joff)
            URDF  -> move_j:  move_j = urdf/sign + zero

        复合后 sign 相消，于是

            move_j = angle - joff + zero

        其中 `zero = movej_urdf_zero_deg`（URDF 全零时下发的 move_j，即 7 姿态）。

        `urdf_offset_deg` 在这一步**相消**（回读时加、下发时减），所以本函数
        与它无关 —— 这是"两条链共用同一个 URDF 空间"的必然结果，不要凭
        "看起来该相消"写公式，常数项一律由两步链推出。

        **不做整圈归一化**，保持为纯换算。要下发给设备请走 urdf_to_movej
        （它带区间折叠），或自行调用 normalize_movej。
        """
        zero = list(self.get_parameter('movej_urdf_zero_deg').value)
        if len(zero) != 6:
            self.get_logger().error(
                'movej_urdf_zero_deg must have exactly 6 elements; using the '
                '7 pose values instead')
            zero = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        return np.array([float(angle_deg[i]) - self.joint_offset_deg[i]
                         + zero[i] for i in range(6)])

    def movej_to_urdf(self, movej_deg):
        """move_j 空间（度）-> URDF 关节角（**弧度**）。与 urdf_to_movej 互逆。

        下滑链是 `move_j = urdf/sign + zero`，所以逆变换就是

            urdf = sign * (move_j - zero)

        这个函数**必须与 urdf_to_movej 成对修改**：只改一边时往返测试仍能
        通过，只是因为同一个错误在正反两次调用里相互抵消。
        """
        zero = list(self.get_parameter('movej_urdf_zero_deg').value)
        if len(zero) != 6:
            self.get_logger().error(
                'movej_urdf_zero_deg must have exactly 6 elements; using the '
                '7 pose values instead')
            zero = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        urdf_deg = [self._safe_sign(i) * (float(movej_deg[i]) - zero[i])
                    for i in range(6)]
        off = self._urdf_offsets()
        return np.radians(np.array([urdf_deg[i] + off[i] for i in range(6)]))

    # ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ Lifecycle node callback ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """连接设备并把手臂送到 home 姿态。

        每个阶段都要留日志，并整体包一层异常捕获：否则失败时日志戛然而止，
        从外面看不出卡在哪一步。
        """
        try:
            self.get_logger().info('CONFIGURE 1/4: declaring parameters')
            self._declare_params()
            self.get_logger().info('CONFIGURE 1/4: done')

            self.get_logger().info('CONFIGURE 2/4: connecting device')
            self.init_dummy()
            self.get_logger().info('CONFIGURE 2/4: done')

            # 在 homing 之前记一笔"现在到底在哪"。homing 之后就看不出偏差了。
            self._check_home_pose()

            self.get_logger().info(
                'CONFIGURE 3/4: homing to the 7 pose (the arm should move now)')
            self.home_dummy()
            self.get_logger().info('CONFIGURE 3/4: done')

            #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
            self.get_logger().info('CONFIGURE 4/4: setting LED')
            self.dummy_driver.robot.set_rgb_mode(1)
            self.get_logger().info('CONFIGURE 4/4: done')
        except Exception as exc:  # noqa: BLE001
            import traceback
            self.get_logger().error(
                f'on_configure FAILED: {type(exc).__name__}: {exc}')
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE

        self.get_logger().info("on_configure() called")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        # self.home_dummy()
        try:
            self.get_logger().info('ACTIVATE 1/3: creating pubs/subs')
            self.init_pub_sub()
            self.get_logger().info('ACTIVATE 1/3: done')
            self.get_logger().info(
                'ACTIVATE 2/3: creating the FollowJointTrajectory action server')
            self.init_trajectory_action_server()
            self.get_logger().info('ACTIVATE 2/3: done')
            #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮  4绿光常亮  5蓝光常亮
            self.dummy_driver.robot.set_rgb_mode(4)
        except Exception as exc:  # noqa: BLE001
            import traceback
            self.get_logger().error(
                f'on_activate FAILED: {type(exc).__name__}: {exc}')
            self.get_logger().error(traceback.format_exc())
            return TransitionCallbackReturn.FAILURE
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
        """lifecycle shutdown 转换：回收缩姿态、等它到位，再断使能。

        `ros2 launch` 退出时执行的就是这个转换（单独跑节点走
        on_sigint_shutdown），两条路径共用 _retract_and_power_off。
        """
        self._retract_and_power_off('on_shutdown')
        return TransitionCallbackReturn.SUCCESS
    # ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ Lifecycle node callback ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

    def on_sigint_shutdown(self):
        """进程退出前的收尾，只是 _retract_and_power_off 的别名。

        保留这个名字：main() 与文档都按它找关机日志，实现只有一份。
        """
        self._retract_and_power_off('shutdown')

    def shutdown_in_progress(self):
        """关机序列是否已经开始（给 main() 里的 SIGTERM 处理器用）。"""
        return self._shutting_down

    def _retract_and_power_off(self, reason):
        """把手臂收回收缩姿态、**等它真的到位**，然后才切断电机使能。

        为什么必须"等"：reset_dummy() 只把命令发出去（_send_named_pose 发完
        就返回，不等到位，那 2/2 步的回读只用来打日志），紧接着 off_dummy()
        就关使能；手臂还在走的时候断使能，它会停在半路、甚至直接掉下来。所以
        这里读回读、确认真的到位，上限 SHUTDOWN_RETRACT_TIMEOUT_SEC。

        reason 只用于日志（on_shutdown / shutdown）。
        """
        with self._shutdown_lock:
            if self._shutdown_done:
                self.get_logger().info(
                    f'{reason}: the shutdown sequence already ran; nothing to do')
                return
            self._shutdown_done = True

            if self.dummy_driver is None:
                self.get_logger().warn(
                    f'{reason}: the device is already disconnected; cannot '
                    'retract the arm or cut power')
                return

            self._shutting_down = True

            # 让正在执行的轨迹先停下来。它可能正卡在 waypoint 之间的 sleep 里，
            # 所以只等一小会儿；等不到也要接着回收缩（宁可日志里两次命令重叠，
            # 也不能不收缩）。
            if self._traj_thread is not None and self._traj_thread.is_alive():
                self.get_logger().warn(
                    f'{reason}: asking the running trajectory to stop before '
                    'retracting the arm')
                self._traj_thread.join(timeout=1.0)

            self.get_logger().warn(
                f'{reason}: retracting the arm to the folded pose '
                f'(move_j_deg={self.reset_joints.tolist()}) before cutting '
                'motor power')
            # force=True：关机时必须发得出去，不能被"有轨迹在执行"挡回来。
            self._send_named_pose(self.reset_joints, 'reset (shutdown)', force=True)
            self._wait_until_pose(self.reset_joints, SHUTDOWN_RETRACT_TIMEOUT_SEC)

            # 顺序不能反：off_dummy() 会把 driver 置 None，之后就没得发 RGB 了。
            # 整段兜异常：关机例程**绝不能抛**出去 —— 它是在 main() 的 finally
            # 里被调的，抛出去会把后面的 executor.shutdown()/rclpy.shutdown()
            # 一起跳过，日志里只剩一个 traceback。
            try:
                self.dummy_driver.robot.set_rgb_mode(0)
                self.off_dummy()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(
                    f'{reason}: cutting motor power FAILED: {exc}. The arm has '
                    'been sent to the folded pose, but the motors may still be '
                    'enabled -- cut power with the switch if it keeps holding')

            # 拆 pub/sub 与 action server 只是"体面退出"，与安全无关，所以放在
            # 断电**之后**，而且单独兜异常：它里面有最长 30 秒的线程 join，
            # 万一卡住也不能影响前面已经做完的两步。
            try:
                self.reset_pub_sub()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'{reason}: teardown failed: {exc}')

    def _wait_until_pose(self, movej_deg, timeout):
        """等手臂到达目标位姿（move_j 空间，度），最多等 timeout 秒。

        返回是否到位。用**设备回读**判断，不用 self.current_urdf_rad 的缓存：
        Ctrl+C 路径下 spin() 已经返回，读状态的定时器不再触发，缓存会冻在最后
        一个值上 —— 拿它判"到位"要么立刻为真、要么永远不为真，两种都是假结论。
        check_joints_place() 是 async 的，那条路这里走不了，所以自己读。
        """
        try:
            target_rad = np.asarray(self.movej_to_urdf(movej_deg), dtype=float)
        except Exception as exc:  # noqa: BLE001
            # 还没 configure（参数没声明）就退出时，movej_to_urdf 读不到
            # movej_urdf_zero_deg。算不出目标就只能给一段固定时间。
            self.get_logger().warn(
                f'cannot compute the folded pose in URDF space ({exc}); '
                'waiting a fixed 2 s instead of checking the readback')
            time.sleep(2.0)
            return False

        t1 = time.time()
        while time.time() - t1 < timeout:
            try:
                self.read_device()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f'readback failed while retracting: {exc}')
                return False
            if np.allclose(self.current_urdf_rad, target_rad, atol=0.02):
                self.get_logger().info(
                    f'the arm reached the folded pose in {time.time() - t1:.1f}s')
                return True
            time.sleep(0.1)

        self.get_logger().error(
            f'the arm did not reach the folded pose within {timeout}s '
            f'(readback urdf_deg='
            f'{np.round(np.degrees(self.current_urdf_rad), 2).tolist()}, target '
            f'urdf_deg={np.round(np.degrees(target_rad), 2).tolist()}). Cutting '
            'motor power anyway: holding the arm under torque forever is worse, '
            'and launch would SIGKILL us shortly (SHUTDOWN_RETRACT_TIMEOUT_SEC)')
        return False

    def init_dummy(self):
        """连接设备并使能电机。

        每一步都要留日志：静默的 connect 与 set_enable 一旦卡住，日志就停在
        参数打印之后，从外面看不出卡在哪一步。
        """
        self.get_logger().info(
            f'connecting to the device (timeout {self.connect_timeout_sec}s) ...')
        if self.dummy_driver is not None:
            self.get_logger().warn("Dummy driver already initialized")
        else:
            # **必须传 timeout。** fibre 的 find_any 默认 timeout=None，
            # 那是"阻塞到有设备连上为止"——没有设备时会永久挂住，不超时、
            # 不报错、不返回，表现为启动日志戛然而止而机械臂不动。
            self.dummy_driver = dummy_cli_tool.ref_tool.find_any(
                path="usb", timeout=self.connect_timeout_sec)
        if self.dummy_driver is None:
            raise RuntimeError(
                f'no device found within {self.connect_timeout_sec}s. '
                'Check that the arm is powered on, the USB cable is seated, '
                'and that no other process holds the fibre link '
                '(only one process can own it at a time).')
        self.get_logger().info('device connected')

        # 电机设为上电。
        #
        # 这条日志只说"命令已发出"，**不是 "enabled"**：设备是否真的加上了
        # 力矩，软件侧看不到。使能后用手推 joint_1，推不动说明力矩在，推得动
        # 说明根本没生效 —— 不要把这行日志当成"电机已使能"的证据。
        self.get_logger().info('sending set_enable(True) ...')
        self.dummy_driver.robot.set_enable(True)
        self.get_logger().info(
            'set_enable(True) returned. Whether the axes actually hold torque '
            'cannot be read back from here; verify by hand if move_j does '
            'nothing (ros2 param list shows what the driver exposes).')

        # 夹爪初始化。**必须在带超时的线程里做。**
        #
        # 夹爪的 fibre 调用会阻塞，而它是在 on_configure 里同步执行的；挂住
        # 就等于整个节点冻死，/joint_states 一条也发不出去。夹爪挂住绝不能
        # 拖死整条 bringup：超时就放弃它、继续 configure，机械臂本体照样能动。
        self._init_gripper_with_timeout()

    def _init_gripper_with_timeout(self):
        """夹爪不走主控这条路了，这里只留说明。

        两边的固件对同一组 CAN ID 的理解不一样：主控以为在"设角度"/"使能"，
        夹爪板把 0x382 当**触发编码器标定**、把 0x381 的前 4 字节当 uint32
        使能位。调用一次就可能通过 `ClearFlash()` 擦掉夹爪的标定表，使夹爪
        读数冻死、位置模式不出力；旧主控固件的 `CanSendMessage` 用
        `osWaitForever` 拿信号量，一帧没发完就永久丢失，之后所有 CAN 发送
        一起卡死，整机失联、只能断电。

        所以夹爪改由 `dummy_vc_grasp` 的 `gripper_can_node` 经 USB-CAN 适配器
        直接驱动 node 7（话题 `/gripper/width`、`/gripper/position`）。主控这里
        一律不碰设备：即使有人传 `gripper_enable:=true` 也只报错、不发帧。
        """
        if self.gripper_enable:
            self.get_logger().error(
                'gripper_enable:=true is not supported any more and has been '
                'IGNORED: the main-board firmware sends 0x382 (which the '
                'gripper board treats as "start encoder calibration") and '
                '0x381 with a float (which it treats as a uint32 enable). Drive '
                'the gripper with dummy_vc_grasp/gripper_can_node over the '
                'USB-CAN adapter instead (/gripper/width). Nothing was sent to '
                'the gripper.')
        else:
            self.get_logger().info(
                'gripper: driven by gripper_can_node over USB-CAN, not by the '
                'main board; skipping gripper init here.')
        self.gripper_enable = False
        self.gripper_ready = False

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

        # 夹爪指令：std_msgs/Float64，0.0 = 全闭，1.0 = 全开。
        self.gripper_sub = self.create_subscription(
            Float64,
            '/dummy_gripper/command',
            self.on_gripper_command,
            10
        )

        # Publishers
        self.gripper_state_pub = self.create_publisher(
            Float64, '/dummy_gripper/state', 10)
        self.j6_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/j6', 10)
        self.tool0_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/tool0', 10)
        # MoveIt2 / robot_state_publisher 默认订阅 /joint_states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

        # TF 不再由本节点发布：tool0 / gripper_tcp / camera_link 等全部在
        # dummy2_urdf.xacro 里静态定义，交给 robot_state_publisher。

        # ------------------------------------------------------------------
        # 两个定时器，**刻意分开**：一个读硬件，一个发话题。
        #
        # 合在一个定时器里时，读数的阻塞式 USB 往返会把发布频率拖住，而且
        # 全部挤在执行器线程里，RViz 收到的是"跳变"而不是连续变化。拆开后
        # 读数慢一点没关系（关节不会突变），发布稳定按固定频率发缓存值、
        # 不碰 USB。
        # ------------------------------------------------------------------
        read_hz = float(self.get_parameter('state_read_hz').value)
        pub_hz = float(self.get_parameter('joint_state_pub_hz').value)
        if read_hz <= 0.0:
            read_hz = 10.0
        if pub_hz <= 0.0:
            pub_hz = 50.0

        self.poll_timer = self.create_timer(1.0 / read_hz,
                                           self.call_poll_position)
        self.pub_timer = self.create_timer(1.0 / pub_hz,
                                          self.call_publish_state)
        self.get_logger().info(
            f'state read {read_hz:.1f} Hz (device I/O), '
            f'joint_states publish {pub_hz:.1f} Hz (cached)')

    # ======================================================================
    # MoveIt 执行接口：FollowJointTrajectory action server
    # ======================================================================
    def init_trajectory_action_server(self):
        """创建 MoveIt 用来下发轨迹的 action server。

        名字必须是 /<controller_name>/<action_ns>，也就是
        moveit_controllers.yaml 里 controller_names 那条加上 action_ns
        （默认就是 follow_joint_trajectory）。**不要再补一段
        follow_joint_trajectory**：action_ns 本身已经叫这个名字了。对不上时
        MoveIt 只会在 execute 时报
        `Action client not connected to action server: ...`，
        规划一路是绿的，很容易被当成规划问题。

        与话题订阅共用 _execute_trajectory，所以两条入口的下发行为完全一致。

        回调组用 Reentrant：轨迹执行动辄几十秒，默认的互斥回调组会在 execute
        回调运行期间挡住这个节点上的其它回调。Reentrant 只是让 executor 能
        继续转；真正的串行化由 _traj_lock + _traj_executing 负责，不是靠
        回调组。
        """
        if not self.trajectory_action_enable:
            self.get_logger().warn(
                'trajectory_action_enable is false: no FollowJointTrajectory '
                'action server will be created, so MoveIt planning will work '
                'but EXECUTE will fail with "Action client not connected"')
            return

        self._traj_cb_group = ReentrantCallbackGroup()
        self._traj_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.trajectory_action_name,
            execute_callback=self._on_traj_execute,
            goal_callback=self._on_traj_goal,
            cancel_callback=self._on_traj_cancel,
            callback_group=self._traj_cb_group,
        )
        self.get_logger().info(
            f'FollowJointTrajectory action server ready on '
            f'{self.trajectory_action_name} (this is what move_group looks '
            'for when you press Execute)')

    def _on_traj_goal(self, goal_request):
        """收到 execute 请求时决定接不接受。

        返回 ACCEPT 之后 goal_handle 才有效，所以这里只做"格式对不对"的
        静态检查；占用检测放在 _on_traj_execute 里，因为 goal 回调可能被
        调用多次（客户端重复发同一个 goal），在那里改状态会漏。
        """
        traj = goal_request.trajectory
        if not traj.joint_names:
            self.get_logger().error(
                'trajectory goal rejected: joint_names is empty, so the '
                'waypoints cannot be mapped onto our joints')
            return GoalResponse.REJECT
        if not traj.points:
            self.get_logger().error(
                'trajectory goal rejected: 0 waypoints (nothing to do)')
            return GoalResponse.REJECT
        self.get_logger().info(
            f'trajectory goal accepted: {len(traj.points)} waypoint(s), '
            f'joint_names={list(traj.joint_names)}')
        return GoalResponse.ACCEPT

    def _on_traj_cancel(self, goal_handle):
        """cancel 请求：接受请求，但**不**在这里停机械臂。

        MoveIt 只有在用户点 Stop 或超时才 cancel。设备侧的急停走的是
        stop_dummy()，而轨迹线程正卡在 waypoint 之间的 sleep 里，实际效果是
        "最多再走完当前 waypoint 就停"。接受 cancel 而不是拒绝，是因为拒绝
        会让客户端一直等到超时；接受之后尽快 abort 更干净。
        """
        self.get_logger().warn(
            'trajectory cancel requested: the driver will stop at the next '
            'waypoint boundary (the device has no instant stop)')
        return CancelResponse.ACCEPT

    @staticmethod
    def _traj_goal_was_cancelled(goal_handle):
        """goal_handle 上有没有"已被取消"这个状态。

        不直接写 goal_handle.is_cancel_requested：rclpy 各版本的属性名不完全
        一样，而这只是一条守卫，读不到就当没被取消 —— 比 AttributeError
        把整条轨迹打掉要好。
        """
        try:
            return bool(getattr(goal_handle, 'is_cancel_requested', False))
        except Exception:  # noqa: BLE001
            return False

    def _traj_busy(self):
        """当前是否已有轨迹在执行。锁保护，因为问它的线程不止一个。"""
        with self._traj_lock:
            return self._traj_executing

    def _progress_log_due(self):
        """进度日志现在该不该打一条。

        节拍就是 trajectory_feedback_hz：既然没发 feedback，那这个参数唯一
        合理的用途就是决定"多密地告诉人它在动"。<= 0 表示只保留关键点日志。

        每次判定都会推进时间基准 —— 包括"暂不打"的那次。不推进的话，判定
        会一直为真，于是每个 waypoint 都打一条，正好是这段代码要避免的事。
        """
        now = time.monotonic()
        period = (1.0 / self.trajectory_feedback_hz
                  if self.trajectory_feedback_hz > 0 else None)
        due = period is not None and (now - self._last_progress_log) >= period
        self._last_progress_log = now
        return due

    def _on_traj_execute(self, goal_handle):
        """action 的执行回调。

        这里的唯一职责是校验、占位、起执行线程，然后**等线程结束**。
        绝不能在本线程里 sleep 着下发 waypoint：这个回调由执行器线程调用，
        一睡下去 pub_timer（/joint_states）就停了，MoveIt 和 RViz 会以为
        机器人不见了 —— 而 execute 的成功判据恰恰用得到 /joint_states。

        等线程结束用 join 而不是轮询：rclpy 的 execute_callback 也可以返回
        rclpy Future，但那要额外引入 Future 和回填线程；join 的语义是确定的。
        """
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        if self._traj_goal_was_cancelled(goal_handle):
            self.get_logger().warn(
                'trajectory goal was already cancelled before it started')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'goal was cancelled before execution started'
            goal_handle.abort()
            return result

        with self._traj_lock:
            if self._traj_executing:
                busy = True
            else:
                busy = False
                self._traj_executing = True
        if busy:
            # 同一时刻只允许一条轨迹。两条同时下发时设备的 move_j 会交替
            # 收到两套目标，机械臂走出一条谁都没规划过的路。
            self.get_logger().error(
                'rejecting trajectory: another trajectory is already '
                'executing (only one at a time; the device has no notion of '
                'two controllers)')
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = (
                'another trajectory is already executing; refusing to '
                'interleave two command sources')
            goal_handle.abort()
            return result

        # 关节顺序在起线程之前算好：名字对不上时要立刻拒绝，而不是让线程
        # 起来之后再空转一圈。失败即清掉占位，否则这条轨迹会永久锁住这个
        # 节点（后面每条轨迹都会被当成"正忙"）。
        order = self._trajectory_joint_order(traj.joint_names)
        if order is None:
            with self._traj_lock:
                self._traj_executing = False
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = (
                'the trajectory does not name our joints '
                f'({list(self.joint_names)}); nothing was commanded')
            self.get_logger().error(result.error_string)
            goal_handle.abort()
            return result

        # 这个容器由工作线程填、由本线程读。线程启动前建立 happens-before，
        # join 之后读是安全的。
        outcome = {}

        def _work():
            outcome['value'] = self._execute_trajectory(traj, order)

        # try/finally 是必须的：上面已把 _traj_executing 置 True，这条路径若
        # 不负责清掉，标记就**永远是 True**，_traj_busy() 从此恒为真 —— 之后
        # 每条 action 轨迹都被拒绝，home/reset/命名位姿也全被 _send_named_pose
        # 静默拒绝（只打一行警告），退出时的"回收缩姿态"正是被这一条挡掉的。
        try:
            self._traj_thread = threading.Thread(
                target=_work, name='traj_exec', daemon=True)
            self._traj_thread.start()
            self._traj_thread.join()

            ok, code, text = outcome.get('value', (
                False, FollowJointTrajectory.Result.INVALID_GOAL,
                'the execution thread did not report a result'))
            result.error_code = code
            result.error_string = text
            if ok:
                self.get_logger().info(
                    'action: trajectory complete, reporting SUCCESSFUL to '
                    'move_group')
                goal_handle.succeed()
            else:
                self.get_logger().error(
                    f'action: trajectory failed (code={code}): {text}')
                goal_handle.abort()
            return result
        finally:
            with self._traj_lock:
                self._traj_executing = False

    def call_poll_position(self):
        """低频：读一次硬件，刷新缓存的当前姿态。

        单位换算与发布都放在 call_publish_state 里，这里只负责取数。
        """
        try:
            self.current_urdf_rad = self.current_joints_callback()
            # 末端位姿设备不支持回读，恒为零；保留调用是为了接口一致
            self.current_end_pose = self.current_end_callback()
        except Exception as e:
            self.get_logger().error(f'Poll position error: {str(e)}')

    def call_publish_state(self):
        """高频：用缓存值发布，不碰硬件。

        这样 /joint_states 的频率由定时器决定，不受 USB 往返时间影响，
        RViz 的动画才是连续的。
        """
        try:
            self.pub_current_joints_and_end_pose()
        except Exception as e:
            self.get_logger().error(f'Publish state error: {str(e)}')

    def pub_current_joints_and_end_pose(self):
        # 发布当前关节角度
        self.pub_current_joints()
        # 发布当前末端位置
        self.pub_current_end_pose()

    def read_raw_angles(self):
        """读取设备六个关节的原始角度（angle 空间，**度**）。

        这是本节点唯一直接触碰 robot.joint_N.angle 的地方。这个读数既不等于
        URDF 角度，也不等于 move_j 指令值，必须经 angle_to_urdf 换算，不能
        当 URDF 弧度直接发布。
        """
        robot = self.dummy_driver.robot
        return [
            float(robot.joint_1.angle),
            float(robot.joint_2.angle),
            float(robot.joint_3.angle),
            float(robot.joint_4.angle),
            float(robot.joint_5.angle),
            float(robot.joint_6.angle),
        ]

    def read_device(self):
        """刷新缓存的当前姿态，并返回本次读到的原始角度（angle 空间，度）。

        self.current_urdf_rad 会同步更新为由同一批读数算出的 URDF 弧度，
        这样 /joint_states 发布的值与调用方拿到的回读值必然出自同一次读取，
        不会出现"日志和 RViz 各说各话"。

        返回值是**度**（angle 空间），不是弧度：调用方通常要拿去和
        joint_offset_deg 对照、或喂给 angle_to_urdf_deg / angle_to_movej，
        那两个入口收的都是度。
        """
        raw_deg = self.read_raw_angles()
        self.current_urdf_rad = self.angle_to_urdf(raw_deg)
        return raw_deg

    def current_joints_callback(self):
        """读取设备原始关节角，换算成 URDF 关节角（弧度）。

        robot.joint_N.angle 是**原始编码器读数**（度），既不等于 URDF 角度，
        也不等于 move_j 指令值，必须经过 angle_to_urdf 换算。
        """
        return self.angle_to_urdf(self.read_raw_angles())

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

        **夹爪没有回读。** 设备上 `hand` 只有 `set_angle` /
        `set_current_limit` / `set_enable` 三个方法，**没有任何可读属性**，
        所以这里只能返回**最近一次下发的角度**换算出的位置，而不是量出来的
        位置：手指的 TF 是**命令值**，夹住东西时手指被挡住，RViz 里仍显示
        "已经合拢到位"，与实际不符。

        映射：命令角度线性映射到归一化开合度，再乘以单指行程。
            frac   = (angle - grip_angle_closed) / (grip_angle_open - grip_angle_closed)
            travel = finger_travel_m * frac
        grip_angle_closed / grip_angle_open 必须**看着手指实测**，当前是占位
        值；填错只影响 RViz 里手指张开的幅度，不影响夹持力。
        """
        angle = self._last_gripper_angle
        if angle is None:
            # 还没下过任何夹爪命令。返回全闭而不是猜一个中间值：全闭是
            # 机构的安全静止位，也是唯一不需要标定就能断言的姿态。
            angle = self.grip_angle_closed

        frac = self.angle_to_gripper_frac(angle)
        self.gripper_frac = frac

        travel = self.finger_travel_m * frac
        return travel, travel

    def angle_to_gripper_frac(self, angle):
        """设备夹爪角度（度）-> 归一化开合度 0.0(全闭) .. 1.0(全开)。"""
        lo = float(self.get_parameter('grip_angle_closed').value)
        hi = float(self.get_parameter('grip_angle_open').value)
        if abs(hi - lo) < 1e-9:
            return 0.0
        frac = (float(angle) - lo) / (hi - lo)
        return max(0.0, min(1.0, frac))

    def gripper_frac_to_angle(self, frac):
        """归一化开合度 -> 设备夹爪角度（度），gripper_frac_to_angle 的逆。"""
        lo = float(self.get_parameter('grip_angle_closed').value)
        hi = float(self.get_parameter('grip_angle_open').value)
        return lo + (hi - lo) * max(0.0, min(1.0, float(frac)))

    def on_gripper_command(self, msg):
        """这条通道已经关了：夹爪改走 `gripper_can_node` 的 `/gripper/width`。

        留着这个订阅只是为了有人在旧脚本里往 `/dummy_gripper/command` 发东西时
        能立刻看到一句明确的解释，而不是静默无事发生。**一帧都不会发给设备。**
        """
        self.get_logger().error(
            'gripper commands over /dummy_gripper/command are no longer '
            'supported: the main-board firmware turns 0x382 into "start '
            'encoder calibration" and 0x381 into a uint32 enable, so sending '
            'through it damages the gripper and used to hang the whole board. '
            'Use /gripper/width (float64, millimetres) with gripper_can_node.',
            throttle_duration_sec=5.0)

    def pub_current_joints(self):
        try:
            left, right = self.current_finger_positions()

            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            # 手指关节必须一起发布，否则这两段 TF 缺失（见方法上方说明）
            joint_state.name = list(self.joint_names) + list(self.finger_joint_names)
            joint_state.position = list(self.current_urdf_rad) + [left, right]
            joint_state.velocity = [0.0] * len(joint_state.name)

            self.joint_pub.publish(joint_state)

            # 夹爪状态：归一化开合度，供上层任务逻辑判断"夹住了没有"
            grip = Float64()
            grip.data = float(self.gripper_frac)
            self.gripper_state_pub.publish(grip)

            self.get_logger().info(
                f'Publish JPOS: j1: {self.current_urdf_rad[0]:.2f}, j2: {self.current_urdf_rad[1]:.2f}, '
                f'j3: {self.current_urdf_rad[2]:.2f}, j4: {self.current_urdf_rad[3]:.2f}, '
                f'j5: {self.current_urdf_rad[4]:.2f}, j6: {self.current_urdf_rad[5]:.2f}, '
                f'fingers: {left:.4f}/{right:.4f} grip={self.gripper_frac:.2f}',
                throttle_duration_sec=1.0
            )
        except Exception as e:
            self.get_logger().error(f'Parse joint error: {str(e)}')

    def pub_current_end_pose(self):
        """不发布手写 TF。

        手写 TF 会与 URDF 里的 frame 名对不上，TF 树断裂、RViz 里坐标系
        对不上。tool0 / gripper_tcp / camera_link / camera_color_optical_frame
        全部在 dummy2_urdf.xacro 里静态定义，由 robot_state_publisher 统一
        发布；本节点只负责发 /joint_states。

        如果将来需要末端位姿，正确做法是从 /joint_states 做正运动学，或者
        直接用 tf2 查询 base_link -> gripper_tcp。
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
        if hasattr(self, 'gripper_sub') and self.gripper_sub is not None:
            self.destroy_subscription(self.gripper_sub)
            self.get_logger().info("Destroyed gripper_sub")

        if hasattr(self, 'j6_pose_pub') and self.j6_pose_pub is not None:
            self.destroy_publisher(self.j6_pose_pub)
            self.get_logger().info("Destroyed j6_pose_pub")
        if hasattr(self, 'tool0_pose_pub') and self.tool0_pose_pub is not None:
            self.destroy_publisher(self.tool0_pose_pub)
            self.get_logger().info("Destroyed tool0_pose_pub")
        if hasattr(self, 'joint_pub') and self.joint_pub is not None:
            self.destroy_publisher(self.joint_pub)
            self.get_logger().info("Destroyed joint_pub")
        if hasattr(self, 'gripper_state_pub') and \
                self.gripper_state_pub is not None:
            self.destroy_publisher(self.gripper_state_pub)
            self.get_logger().info("Destroyed gripper_state_pub")

        if hasattr(self, 'poll_timer') and self.poll_timer is not None:
            self.destroy_timer(self.poll_timer)
            self.get_logger().info("Destroyed poll_timer")
        if hasattr(self, 'pub_timer') and self.pub_timer is not None:
            self.destroy_timer(self.pub_timer)
            self.get_logger().info("Destroyed pub_timer")

        # action server 必须一起销毁。只销毁 pub/sub 而留着它，deactivate 之后
        # move_group 依然能看到这个名字、依然能连上，于是"节点已停"和"还能执行"
        # 同时成立 —— 下一次 activate 还会再创建一个同名 server，两个 server
        # 抢同一个名字，客户端连到哪个不确定。
        #
        # 先等正在跑的轨迹结束再拆：执行线程还在下发 waypoint，此时销毁 server
        # 会让它的 goal_handle 变成悬空引用。等不住就照拆，但要留下日志。
        if self._traj_thread is not None and self._traj_thread.is_alive():
            self.get_logger().warn(
                'a trajectory is still executing; waiting for it to finish '
                'before destroying the action server')
            self._traj_thread.join(timeout=30.0)
            if self._traj_thread.is_alive():
                self.get_logger().error(
                    'the trajectory thread did not finish within 30 s; '
                    'destroying the action server anyway. It will finish on '
                    'its own, but its result no longer reaches move_group.')
        self._traj_thread = None

        if self._traj_action_server is not None:
            self._traj_action_server.destroy()
            self._traj_action_server = None
            self.get_logger().info("Destroyed trajectory action server")

    def _send_named_pose(self, movej_deg, label, force=False):
        """把命名位姿下发到硬件。

        输入是 **move_j 空间**的角度（度），也就是 home_joints /
        reset_joints 的格式。**直接原样交给 move_j，不做任何变换。**

        为什么不做逆变换：这些值从上游驱动继承，上游直接发给 move_j、中间
        没有任何换算；实体机械臂已验证它们能到达预期姿态，因此必然就是
        move_j 空间的。再套一层换算反而会把姿态改错。

        日志同时打印回读侧换算出的 URDF 角度，方便与 RViz 对照。
        """
        movej_deg = np.asarray(movej_deg, dtype=float)

        # 有一条轨迹正在跑时不能插队。home/reset/急停都会走这里，而轨迹线程
        # 正按节拍下发 waypoint —— 两条命令流交替发给设备，机械臂就会走出一条
        # 谁都没规划过的路。急停是例外：宁可不规矩也要停下，所以 label 里带
        # "emergency" 时照发，但留下日志说明重叠了。
        #
        # force 是关机的例外（见 _retract_and_power_off）：那时必须把手臂收回
        # 收缩姿态，被挡回来的后果是"继续带着使能停在原地"或"直接断使能掉下来"。
        if self._traj_busy() and not force and 'emergency' not in label:
            self.get_logger().warn(
                f'{label}: refused because a trajectory is executing. Wait for '
                'it to finish (or cancel it from RViz) and try again.')
            return

        # 每一步都记日志：move_j 或回读一旦阻塞，日志会停在上一行不动，
        # 无法判断卡在哪一个。
        self.get_logger().info(f'{label}: step 1/2 sending move_j ...')
        try:
            self.move_joints(movej_deg)
        except Exception as exc:
            self.get_logger().error(f'{label} command failed: {exc}')
            return
        self.get_logger().info(f'{label}: step 1/2 sent')

        # 回读仅用于日志对照，与上一步的 try 分开：回读失败若也报
        # "{label} failed"，就成了"日志说失败、实物其实已经动了"的假故障。
        self.get_logger().info(f'{label}: step 2/2 reading back ...')
        try:
            # read_device() 返回的是 angle 空间的**度**，正好是
            # angle_to_urdf_deg 的输入；不要拿 current_urdf_rad 去喂它。
            raw_deg = self.read_device()
            urdf_deg = self.angle_to_urdf_deg(raw_deg)
            self.get_logger().info(
                f'{label}: move_j_deg={np.round(movej_deg, 2).tolist()} | '
                f'readback angle_deg={np.round(raw_deg, 2).tolist()} '
                f'-> urdf_deg={np.round(urdf_deg, 2).tolist()}')
        except Exception as exc:
            self.get_logger().warn(
                f'{label}: command sent, but readback failed: {exc}')

    def _wait_for_pose(self, target_deg, timeout_s, tol_deg, rate_hz=10.0):
        """轮询 raw 角度，直到六轴都进 target 的容差以内（或超时）。

        为什么需要它：`_send_named_pose` 在发完 move_j 之后 25 ms 就回读一次，
        那时电机还没动完 —— 日志看起来像"命令发了、读数毫无变化"，容易被读成
        "命令没生效"。这里真的等：每 100 ms 读一次，判据用**标定 home 的原始
        角**（raw 空间，度），不掺 URDF 换算。

        返回 (ok, last, elapsed, before, moved)：moved 是逐轴位移（度）。
        """
        t0 = time.time()
        before = None
        try:
            before = [float(v) for v in self.read_raw_angles()]
        except Exception as exc:        # noqa: BLE001
            self.get_logger().warn(
                f'home: could not read the angles before the wait '
                f'({type(exc).__name__}: {exc})')
        last = before
        target = np.asarray(target_deg, dtype=float)
        ok = False
        while time.time() - t0 < timeout_s:
            try:
                last = [float(v) for v in self.read_raw_angles()]
            except Exception:            # noqa: BLE001
                time.sleep(1.0 / rate_hz)
                continue
            if np.all(np.abs(np.asarray(last) - target) <= tol_deg):
                ok = True
                break
            time.sleep(1.0 / rate_hz)
        moved = None
        if before is not None and last is not None:
            moved = [float(b - a) for a, b in zip(before, last)]
        return ok, last, time.time() - t0, before, moved

    def home_dummy(self, wait=True):
        """回到 7 姿态，并**确认真的到了**。

        发完命令就往下走是不行的：那样"没到位"只能靠人看 RViz 才发现。这里等
        它走到标定 home，再逐关节打一行"起点 -> 现在 / 目标 / 移动 / 还差多少"，
        并把"目标差很多但几乎没动"的轴单独点名 —— 那是没使能、堵转、或机械卡住
        的指纹，与"走了一半停在半路"不是一回事。
        """
        self._send_named_pose(self.home_joints, 'home')
        if not wait or self.home_reference_deg is None:
            return
        ok, last, elapsed, before, moved = self._wait_for_pose(
            self.home_reference_deg, self.home_timeout_s, self.home_tol_deg)
        if last is None or before is None:
            return
        rows = []
        stuck = []
        for i, name in enumerate(HOME_JOINT_NAMES):
            target = float(self.home_reference_deg[i])
            gap = target - float(last[i])
            step = float(moved[i])
            rows.append(
                f'{name} {float(before[i]):+.1f} -> {float(last[i]):+.1f} '
                f'(target {target:+.1f}, moved {step:+.1f}, gap {gap:+.1f})')
            if abs(gap) > 3.0 * self.home_tol_deg and abs(step) < 1.0:
                stuck.append(name)
        log = self.get_logger().info if ok else self.get_logger().warn
        log(f'home: {"reached" if ok else "did NOT reach"} the calibrated home '
            f'in {elapsed:.1f} s')
        log('home: ' + ' | '.join(rows))
        if stuck:
            self.get_logger().error(
                f'home: {"/".join(stuck)} did not move although the target is '
                f'far away. That is not a slow joint -- the command reached the '
                f'controller and the axis still will not move. Check in this '
                f'order: (1) is the axis enabled (disable the motors, then push '
                f'it by hand: a disabled axis moves freely), (2) is it stalled '
                f'against something, (3) is its current limit too low to lift '
                f'this pose.')

    def stop_dummy(self):
        """急停：把设备**此刻的实际姿态**换算回 move_j 空间再下发一次。

        走 angle -> URDF -> move_j 这条标准两步链，不另拼公式。

        喂给换算函数的必须是 angle 空间的**度**，不是缓存的 URDF 弧度；拿弧度
        去喂，下发的目标会与当前姿态差几十度，急停按下去就是一次大角度抽动
        而不是停住。
        """
        # read_device() 返回 angle 空间（度）的原始读数；move_j 空间就是它
        # 减去 move_j_offset_deg。急停要下发的正是"当前位置"，所以它同时
        # 充当就近等价的参考点。
        raw_deg = self.read_device()            # 刷新 current_urdf_rad（弧度）
        cur_movej = [float(raw_deg[i]) - self.move_j_offset_deg[i]
                     for i in range(6)]
        movej_deg = self.urdf_to_movej(self.current_urdf_rad,
                                       current_deg=cur_movej)
        self._send_named_pose(movej_deg, 'emergency stop')

    def reset_dummy(self, wait=True):
        self._send_named_pose(self.reset_joints, 'reset')

    async def check_joints_place(self, target_urdf_deg):
        """等待关节到位。

        target_urdf_deg 是 **URDF 空间**的角度（度），与 self.current_urdf_rad
        （弧度）同属一个空间，只差单位换算。两边必须同空间：拿设备角度去和
        URDF 弧度比较，"到位"会永远判不成立。
        """
        target_rad = np.radians(target_urdf_deg)
        t1 = time.time()
        while True:
            if np.allclose(self.current_urdf_rad, target_rad, atol=0.01):
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

        按 time_from_start 节拍依次下发**全部** waypoint：只取 msg.points[0]
        等于丢掉整条插值路径，轻则动作难看，重则瞬时大角度跳变损伤减速器。

        joint_names 的顺序可能与我们的关节顺序不同，必须按名字重排，
        不能假设它们已经是 dummy2_joint1..6 的顺序。

        **这里仍然在本线程里阻塞**（与 action 那条入口不同）：话题入口要的是
        "回调结束 = 这条轨迹走完"的简单语义，代价是执行期间 /joint_states
        会停；它不接 MoveIt，这个代价可以接受。action 入口必须让出线程，
        原因见 _on_traj_execute 的 docstring。

        占用标记是加了锁再判再置的：`if self._traj_executing` 之后再置位，
        中间可能被 action 那条入口插进来，两条轨迹就会同时下发。
        """
        # 收到轨迹先记一条日志。缺少这一行会导致「实物没动」时无法判断
        # 是回调没触发、还是被拒绝、还是执行了但没效果。
        self.get_logger().info(
            f'RECEIVED JointTrajectory: {len(msg.points)} point(s), '
            f'joint_names={list(msg.joint_names)}')

        if not msg.points:
            self.get_logger().warn('Received JointTrajectory with no points')
            return

        # 按名字重排到我们的关节顺序。**要在占位之前做**：名字对不上时
        # 直接返回，绝不能留下一个永远的"正忙"标记。
        order = self._trajectory_joint_order(msg.joint_names)
        if order is None:
            return

        with self._traj_lock:
            if self._traj_executing:
                busy = True
            else:
                busy = False
                self._traj_executing = True
        if busy:
            self.get_logger().warn(
                'A trajectory is already executing, ignoring the new one')
            return

        try:
            ok, code, text = self._execute_trajectory(msg, order)
            if not ok:
                self.get_logger().error(
                    f'trajectory from the topic entry point failed '
                    f'(code={code}): {text}')
        finally:
            with self._traj_lock:
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
        """把一条轨迹按节拍下发到设备。返回 (ok, error_code, error_string)。

        返回值这三个东西是给 action 用的：MoveIt 的 execute 结果里
        error_code 非 0 就判定执行失败，并且会把 error_string 打进日志。
        **不能失败也返回成功** —— 那会让 MoveIt 认为手臂到了目标位姿，
        后面所有规划都建立在一个错误的状态上。

        错误码取自 control_msgs/action/FollowJointTrajectory 里的常量，
        不自己编数字：那几个值是客户端按名字解释的。
        """
        total = len(msg.points)
        self.get_logger().info(
            f'Executing trajectory: {total} waypoints, '
            f'joint order map {order}')

        # 进度日志的节拍由 trajectory_feedback_hz 决定（默认 5 Hz 上限），
        # 并在关键点（第一个 / 每 10 个 / 最后一个）强制打印 —— MoveIt 的
        # waypoint 按时间均匀铺开，只按数量打会把几十秒的执行淹在日志里。
        self._last_progress_log = time.monotonic()

        # 当前命令位置（move_j 空间），作为"就近整圈等价"的参考。
        #
        # **用命令历史，不用回读反推。** 后者是 `raw - move_j_offset_deg`，
        # 那个式子没有物理依据：实测 j3 上它与真实命令位置相差 270 度，导致
        # 目标被推到设备拒绝的区间、整条命令静默失效。驱动自己知道发过什么，
        # 那才是权威来源。只有在本节点还没发过任何命令时才回退到回读。
        cur_movej = self.guess_current_movej()
        if cur_movej is None:
            text = ('cannot determine the current command pose (no command sent '
                    'yet and the readback is unavailable); refusing to execute '
                    'a trajectory because the nearest-equivalent conversion '
                    'would be guessing')
            self.get_logger().error(text)
            return (False, FollowJointTrajectory.Result.INVALID_GOAL, text)
        src = ('command history' if self.commanded_movej() is not None
               else 'readback fallback (no command sent yet)')
        self.get_logger().info(
            f'trajectory reference pose from {src}: '
            f'{[round(v, 2) for v in cur_movej]}')

        t_start = time.monotonic()
        for idx, point in enumerate(msg.points):
            if len(point.positions) < len(order):
                text = (f'waypoint {idx} has {len(point.positions)} positions, '
                        f'need at least {len(order)}')
                self.get_logger().error(text + '; aborting')
                return (False, FollowJointTrajectory.Result.INVALID_GOAL, text)

            # 轨迹顺序 -> 本节点顺序
            urdf_rad = np.array([point.positions[order[i]] for i in range(6)])

            # 按 time_from_start 节拍等待
            target_t = (point.time_from_start.sec
                        + point.time_from_start.nanosec * 1e-9)
            wait = t_start + target_t - time.monotonic()
            if wait > 0:
                time.sleep(wait)

            # 关机（Ctrl+C / lifecycle shutdown）时立刻停止下发。关机序列马上
            # 要把手臂收回收缩姿态，这里再发一个 waypoint 就是两条命令流交替
            # 发给设备，机械臂会走出一条谁都没规划过的路。放在 sleep 之后：
            # sleep 期间收到的关机请求，至少能在下发前被发现。
            if self._shutting_down:
                text = ('trajectory aborted: the node is shutting down and the '
                        'arm is being retracted to the folded pose')
                self.get_logger().warn(text)
                return (False, FollowJointTrajectory.Result.INVALID_GOAL, text)

            # URDF 弧度 -> 设备角度（含 sign/offset 逆变换），单位是度。
            # 传入当前位置，结果会被折算成离它最近的整圈等价角 —— 不这样做
            # 就会重演"下发 340 度、设备够不着、静默不动"那个 bug。
            movej_deg = self.urdf_to_movej(urdf_rad, current_deg=cur_movej)
            try:
                self.move_joints(movej_deg)
            except Exception as exc:  # noqa: BLE001
                text = f'waypoint {idx} command failed: {exc}'
                self.get_logger().error(text + '; aborting trajectory')
                return (False, FollowJointTrajectory.Result.INVALID_GOAL, text)
            # 下一个 waypoint 以本 waypoint 为参考，保证轨迹连续
            cur_movej = [float(v) for v in movej_deg]

            if idx % 10 == 0 or idx == total - 1:
                self.get_logger().info(
                    f'  waypoint {idx + 1}/{total} '
                    f't={target_t:.2f}s deg={np.round(movej_deg, 2).tolist()}')
                self._last_progress_log = time.monotonic()
            elif self._progress_log_due():
                self.get_logger().info(
                    f'  waypoint {idx + 1}/{total} '
                    f't={target_t:.2f}s deg={np.round(movej_deg, 2).tolist()}')

        self.get_logger().info('Trajectory execution complete')
        return (True, FollowJointTrajectory.Result.SUCCESSFUL, '')

    def move_joints(self, joints_deg):
        """下发关节角度（度）。

        下发成功后记录本次命令值，作为"设备当前命令位置"的权威来源。设备
        不回读 move_j，而从原始回读反推的式子没有物理依据（实测 j3 可差
        270 度），所以命令历史是唯一可靠的位置来源。
        """
        self.dummy_driver.robot.move_j(joints_deg[0], joints_deg[1], joints_deg[2],
                                       joints_deg[3], joints_deg[4], joints_deg[5])
        self._remember_cmd_movej(joints_deg)

    def off_dummy(self):
        self.dummy_driver.robot.set_enable(False)
        self.dummy_driver = None
        self.get_logger().info("Dummy driver powered off")

def main(args=None):
    rclpy.init(args=args)
    node = dummy_controller()

    # ------------------------------------------------------------------
    # 多线程执行器，**这是 MoveIt 执行能工作的前提之一**。
    #
    # 轨迹执行时执行器里有两件事要同时进行：
    #   - action 的 execute 回调：等 _traj_thread 把 waypoint 走完（几十秒）
    #   - pub_timer：按 50 Hz 发 /joint_states
    # 单线程执行器只能干一件。execute 回调占着线程的那几十秒里，
    # /joint_states 一条都不发，MoveIt 与 RViz 会认为机器人不见了 ——
    # 而判断 execute 是否成功恰恰要用到它。
    #
    # 2 个线程就够（实测无需更多）：一个跑那个长回调，其余回调在另一个上
    # 排队。线程再多只是多几个线程等同一把 fibre 锁。默认的互斥回调组会
    # 让"同一个组里的回调不并发"，所以 pub_timer 不会被别的回调挤掉。
    # ------------------------------------------------------------------
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    # ------------------------------------------------------------------
    # SIGTERM：`ros2 launch` 在 sigterm_timeout（默认 5 秒）之后会给子进程发
    # SIGTERM，而 SIGTERM 的默认动作是**立即终止** —— 正好会打断正在回收缩
    # 姿态的那几秒：手臂停在半路，使能直接掉。所以这里自己接管：
    #
    #   * 关机序列还没开始 → 抛 KeyboardInterrupt，走和 Ctrl+C 完全一样的路；
    #   * 已经在跑 → 只记一条日志，**绝不打断**。回收缩这件事必须一次做完，
    #     半路被杀比不回收缩更危险。
    #
    # 真的卡住不返回，还有 launch 的 SIGKILL 兜底（再 sigkill_timeout，默认
    # 5 秒），所以这里不会把进程变成"杀不死"。
    # ------------------------------------------------------------------
    def _on_sigterm(signum, frame):
        if node.shutdown_in_progress():
            node.get_logger().warn(
                'SIGTERM received while the shutdown sequence is running; '
                'ignoring it so the arm can finish retracting')
            return
        node.get_logger().warn(
            'SIGTERM received (ros2 launch escalates to it); shutting down '
            'like Ctrl+C')
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C received. Executing custom shutdown.")
    except Exception as e:
        node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        # 兜底，而且是**唯一**的执行点：不管 spin() 是因为 Ctrl+C、SIGTERM、
        # rclpy 自己关掉 context 而正常返回，还是因为异常退出，都必须先回收缩
        # 姿态、等它到位，再断使能。on_sigint_shutdown 是幂等且带锁的，重复
        # 调用（比如 on_shutdown 已经做过）只会打一行日志。
        node.on_sigint_shutdown()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

