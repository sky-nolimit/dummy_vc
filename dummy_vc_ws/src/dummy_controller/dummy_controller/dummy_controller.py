import time
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

        # 夹爪全开对应的设备角度（**度**）。标定前用占位值。
        self.hand_max_angle = 90.0

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
        self.joint_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
        self.joint_offset_deg = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
        self.move_j_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]

        # 位置参数：单位是**度**，且这里的角度是 **move_j 空间**（设备侧），
        # **不是** URDF 角度。下发时直接原样交给 move_j，不做任何变换。
        #
        # 这是上游驱动里被实体机械臂验证过的值，直接复用。
        # home = 7 姿态（手臂自肩部水平前伸），对应 URDF 空间 [0,0,0,0,0,0]。
        self.home_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.reset_joints = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])

        # 内部状态统一用**弧度**（ROS 惯例）。
        # 名字里带 urdf_rad 是刻意的：这里存的是 URDF 关节角（弧度），
        # **不是**设备的 angle 回读值（度）。两者之间差一次 angle_to_urdf
        # 换算；属性名含糊时曾经在急停路径上被混用（把弧度直接当度下发）。
        self.current_urdf_rad = np.zeros(6)
        self.current_end_pose = np.zeros(6)

        # 最后一次下发成功的 move_j 值，作为"设备当前命令位置"的权威来源。
        # 见 commanded_movej()：不用回读反推，因为那个反推式子没有物理依据。
        self._last_cmd_movej = None

        # 轨迹执行
        self._traj_executing = False

        self.get_logger().info("dummy_controller_node Lifecycle node constructor called")
        self.get_logger().info(
            f'joint_sign={self.joint_sign} joint_offset_deg={self.joint_offset_deg}')

    def _declare_params(self):
        """声明并读取参数。on_configure 之后调用。"""
        default_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
        # (1) 7 姿态的 angle 回读就是该姿态的偏置（此时 URDF 应为全零）
        default_offset_deg = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
        # (2) move_j 与 angle 在 j3 上差 87.70
        default_movej_offset_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        # (3) URDF 空间里的附加偏置，默认全零。
        #     曾经把 j3 设成 180 想抵消姿态反差，方向加反了，反而把本该为 0
        #     的值推成 180——那正是问题本身。详见 config 里的推导记录。
        #     当模型与实物**转向相同但整体差一个固定角度**时才改这里。
        #     若模型与实物**转向相反**，应改 joint_sign 而不是这里。
        default_urdf_offset_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.declare_parameter('joint_sign', default_sign)
        self.declare_parameter('joint_offset_deg', default_offset_deg)
        self.declare_parameter('move_j_offset_deg', default_movej_offset_deg)
        self.declare_parameter('urdf_offset_deg', default_urdf_offset_deg)
        self.declare_parameter('home_joints_deg', [0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.declare_parameter('reset_joints_deg', [0.0, -75.0, 180.0, 0.0, 0.0, 0.0])
        # ---- 夹爪 ----------------------------------------------------------
        # 手指行程上限（米），来自 URDF 的 limit upper
        self.declare_parameter('finger_travel_m', 0.023)
        # 夹爪全闭 / 全开对应的设备角度（**度**）。阶段 0.4 实测后替换。
        # 这两个值的差决定手指位移的折算比例，填错只影响 RViz 显示。
        self.declare_parameter('grip_angle_closed', 0.0)
        self.declare_parameter('grip_angle_open', 90.0)
        # 夹持力上限。这是唯一能防"夹碎"的手段，从小值开始试。
        self.declare_parameter('grip_current_limit', 1.0)
        # 兼容旧参数名，避免已有配置直接报"未声明参数"
        self.declare_parameter('hand_max_angle', 90.0)
        # move_j 下发前的整圈归一化区间（仅在拿不到当前姿态时的回退路径使用）
        self.declare_parameter('wrap_min_deg', 0.0)
        self.declare_parameter('wrap_max_deg', 360.0)
        # 单次 move_j 允许的最大行程（度）。超过就报错而不是静默下发。
        #
        # 设备**会静默忽略够不着的设定值** —— 这正是"下发成功但机械臂不动"
        # 能藏很久的原因。默认 180：一个关节一次转过半圈以上，几乎肯定是
        # 整圈折算选错了，或者目标姿态本身有问题。
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
        # **实测得来**（tools/probe_range.py）。区间外的值被设备静默丢弃。
        self.declare_parameter('movej_accepted_range_deg', [
            -165.0, 165.0, -75.0, 90.0, 45.0, 180.0,
            -180.0, 165.0, -90.0, 90.0, -180.0, 180.0])
        # 设备连接超时（秒）。**不要设为 0 或负数**：fibre 的 find_any 在
        # timeout=None 时会永久阻塞，没有设备时会让启动静默卡死。
        self.declare_parameter('connect_timeout_sec', 30.0)
        # 夹爪初始化超时（秒）。夹爪的 fibre 调用曾经阻塞过，而它若在
        # on_configure 里同步等待就会冻死整个节点，连 /joint_states 都不发。
        self.declare_parameter('gripper_init_timeout_sec', 5.0)
        # 是否在启动时初始化夹爪。**默认 false。**
        #
        # 实测：夹爪的 fibre 调用会永久阻塞且不返回。那个线程留在那里会一直
        # 持有 fibre 链路，于是后续每个设备调用都排在它后面 —— 不只是夹爪
        # 不可用，**move_j 也会一起卡住，机械臂根本动不了**。
        #
        # 所以默认不碰夹爪：先让机械臂能正常跑起来，夹爪单独作为一项议题去查
        # （很可能是固件或接线问题）。要用夹爪时显式打开 gripper_enable:=true。
        self.declare_parameter('gripper_enable', False)
        # 夹爪是否初始化成功。失败、超时或被禁用后为 False。
        self.gripper_ready = False

        self.joint_sign = list(self.get_parameter('joint_sign').value)
        self.joint_offset_deg = list(
            self.get_parameter('joint_offset_deg').value)
        self.move_j_offset_deg = list(
            self.get_parameter('move_j_offset_deg').value)
        self.urdf_offset_deg = list(
            self.get_parameter('urdf_offset_deg').value)
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
        self.connect_timeout_sec = float(
            self.get_parameter('connect_timeout_sec').value)
        self.movej_current_trim_deg = list(
            self.get_parameter('movej_current_trim_deg').value)
        if len(self.movej_current_trim_deg) != 6:
            self.get_logger().error(
                'movej_current_trim_deg must have exactly 6 elements; '
                'ignoring it and using zeros')
            self.movej_current_trim_deg = [0.0] * 6
        if self.connect_timeout_sec <= 0.0:
            self.get_logger().warn(
                f'connect_timeout_sec={self.connect_timeout_sec} would block '
                'forever on a missing device; using 30.0 instead')
            self.connect_timeout_sec = 30.0
        if abs(self.grip_angle_open - self.grip_angle_closed) < 1e-6:
            self.get_logger().error(
                'grip_angle_open and grip_angle_closed are equal; the finger '
                'mapping would divide by zero. Falling back to 0..90 deg.')
            self.grip_angle_closed = 0.0
            self.grip_angle_open = 90.0

        for name, val in (('joint_sign', self.joint_sign),
                          ('joint_offset_deg', self.joint_offset_deg),
                          ('move_j_offset_deg', self.move_j_offset_deg),
                          ('urdf_offset_deg', self.urdf_offset_deg)):
            if len(val) != 6:
                self.get_logger().error(
                    f'{name} must have exactly 6 elements, got {len(val)}; '
                    'falling back to defaults')
                self.joint_sign = [-1.0, -1.0, -1.0, 1.0, -1.0, -1.0]
                self.joint_offset_deg = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
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
    # 除以 sign 而不是乘，是因为 sign 的物理含义是
    # "URDF 正方向相对设备正方向是否翻转"，所以进 URDF 时乘、出 URDF 时除。
    #
    # sign 的确定方法（可重复，逐关节做）：
    #   用 tools/test_motion.sh 给某个关节一个正向增量，
    #   看实物与 RViz 模型的转向是否一致。
    #   实物逆时针而模型顺时针 -> 该关节 sign 应为 -1。
    #   一次只改一个关节，改完重测。
    #
    # 一致性锚点（7 姿态，物理已验证）：
    #   angle  = [0.215, 74.572, -91.474, -0.910, -0.035, -0.040]
    #   urdf   = [0, 0, 0, 0, 0, 0]
    #   move_j = [0, 0, 90, 0, 0, 0]
    # 两个偏置都由此推出，所以锚点处 sign 取任何值都仍映射到 0。
    # ----------------------------------------------------------------------
    def _apply_sign(self, sign, vals):
        return np.array([sign[i] * float(vals[i]) for i in range(6)])

    def angle_to_urdf_deg(self, angle_deg):
        """angle 空间（度）-> URDF 关节角（**度**）。"""
        shifted = [float(angle_deg[i]) - self.joint_offset_deg[i]
                   for i in range(6)]
        return self._apply_sign(self.joint_sign, shifted)

    def angle_to_urdf(self, angle_deg):
        """angle 空间（度）-> URDF 关节角（**弧度**）。

        这是回读发布的唯一入口（/joint_states 需要弧度）。
        """
        return np.radians(self.angle_to_urdf_deg(angle_deg))

    def urdf_to_angle(self, urdf_rad):
        """URDF 关节角（弧度）-> angle 空间（度）。angle_to_urdf 的逆。"""
        urdf_deg = np.degrees(np.asarray(urdf_rad, dtype=float))
        signed = [float(urdf_deg[i]) / self._safe_sign(i) for i in range(6)]
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

        为什么要有这个东西，而不是从原始回读反推当前位置：

        `move_j` 是只写不读的接口，设备不回读命令值。此前驱动用

            current_move_j = raw - move_j_offset_deg

        反推，但那个式子**没有任何物理依据** —— 它是把两个不同来源的常数
        硬拼在一起。实测暴露出后果：j3 反推得到 -181.474，而依据实测关系
        （readback = move_j - 180，gain +1，残差 < 0.011 度）该关节真实的
        move_j 位置是约 +90。差了 270 度。于是"就近整圈等价"把 j3 的目标从
        -1.474 推到 -360，落进设备拒绝的取值区间，**整条命令被静默丢弃**。

        驱动自己知道它发过什么，所以直接用命令历史既准确又不需要任何假设。
        初值取 None，表示"还没发过"，首次使用时回退到回读反推。
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

        **这个函数修过两次，两次都是同一个教训的不同侧面。**

        第一版把每个角度归一化进 [0, 360)，于是物理意图 -20.22 度被写成
        339.78 度；设备按字面理解要转将近一整圈，命令被静默拒绝。改成"离
        当前位置最近的整圈等价"之后，j1 之类的关节好了，**但 j3 仍然不动**。

        第二版失败的教训更重要：**设备有一个实测出来的、很窄而且偏离零点的
        可接受区间**，而"就近整圈等价"假设区间是绕原点的一整圈。实测区间是

            joint_1 [-165, 165]   joint_2 [-75, 90]    joint_3 [45, 180]
            joint_4 [-180, 165]   joint_5 [-90, 90]    joint_6 [-180, 180]

        j3 的最小可接受值是 +45。驱动早期算出的 -1.474 落在区间外，**而且它
        的任何整圈平移（-361.5、358.5、718.5）也都在区间外** —— 所以整圈折算
        对它无能为力。区间外的值被设备静默丢弃，整条命令不执行。

        正确做法：把每个目标平移整圈，落到该关节的 [lo, hi] 之内。区间可能
        跨过 0/360 接缝（例如 [-180, 165]），所以用带符号的取模而不是夹取。

        找不到合法等价值时**报错**，不静默下发 —— 静默失败正是这件事能藏这么
        久的原因。
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

        这是把 MoveIt 轨迹点转成设备指令的唯一入口。

            move_j = urdf_deg / sign + move_j_offset_deg

        move_j_offset_deg 就是 7 姿态的 move_j 值 [0,0,90,0,0,0]，因为锚点处
        urdf = 0。这一点在 tools/selftest_conversion.py 里有断言。

        current_deg 是设备当前位置（move_j 空间），用于把结果折算成离当前位置
        最近的整圈等价角。**不传它会退回旧的 [0,360) 归一化，而那正是导致
        "下发成功但机械臂不动" 的写法**，所以正常路径都应传入。
        """
        urdf_deg = np.degrees(np.asarray(urdf_rad, dtype=float))
        out = np.array([float(urdf_deg[i]) / self._safe_sign(i)
                        + self.move_j_offset_deg[i]
                        for i in range(6)])
        return self.normalize_movej(out, current_deg)

    def movej_to_angle(self, movej_deg):
        """move_j 空间（度）-> angle 空间（度）。angle_to_movej 的逆。

        必须同时减掉 move_j_offset_deg **并加回** joint_offset_deg。完整的
        换算是 move_j = angle - joint_offset_deg + move_j_offset_deg
        （config/dummy_controller.yaml 里那两条链的复合），所以逆变换少任何
        一项都得不到 angle 空间的值。早期版本漏了 joint_offset_deg，在 7 姿态
        上偏差达 [0.2, 74.6, -91.5, ...] 度。
        """
        return np.array([float(movej_deg[i]) - self.move_j_offset_deg[i]
                         + self.joint_offset_deg[i] for i in range(6)])

    def angle_to_movej(self, angle_deg):
        """angle 空间（度）-> move_j 空间（度）。movej_to_angle 的逆。

        与 config 的两条链保持一致：
            move_j = angle - joint_offset_deg + move_j_offset_deg
        这正是 angle -> URDF -> move_j 的复合（sign 两次相消）。

        **不做整圈归一化**，保持为纯换算。要下发给设备请走 urdf_to_movej
        （它带归一化），或自行调用 normalize_movej。
        """
        return np.array([float(angle_deg[i]) - self.joint_offset_deg[i]
                         + self.move_j_offset_deg[i] for i in range(6)])

    def movej_to_urdf(self, movej_deg):
        """move_j 空间（度）-> URDF 关节角（**弧度**）。与 urdf_to_movej 互逆。"""
        urdf_deg = [self._safe_sign(i)
                    * (float(movej_deg[i]) - self.move_j_offset_deg[i])
                    for i in range(6)]
        return np.radians(np.array(urdf_deg))

    # ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ Lifecycle node callback ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """连接设备并把手臂送到 home 姿态。

        每个阶段都要留日志，并且整体包一层异常捕获。原因：此前这里失败时
        日志会戛然而止（停在参数打印之后），从外面完全看不出卡在哪一步，
        表现为"启动脚本只有几行输出、机械臂不动"。
        """
        try:
            self.get_logger().info('CONFIGURE 1/4: declaring parameters')
            self._declare_params()
            self.get_logger().info('CONFIGURE 1/4: done')

            self.get_logger().info('CONFIGURE 2/4: connecting device')
            self.init_dummy()
            self.get_logger().info('CONFIGURE 2/4: done')

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
            self.get_logger().info('ACTIVATE 1/2: creating pubs/subs')
            self.init_pub_sub()
            self.get_logger().info('ACTIVATE 1/2: done')
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
        """连接设备并使能电机。

        这里每一步都要留日志。此前 connect 与 set_enable 都是静默的,一旦
        卡住或抛异常,日志会停在 _declare_params() 之后、on_configure() 之前,
        从外面看就是"什么都没发生",无法判断卡在哪一步。
        """
        self.get_logger().info(
            f'connecting to the device (timeout {self.connect_timeout_sec}s) ...')
        if self.dummy_driver is not None:
            self.get_logger().warn("Dummy driver already initialized")
        else:
            # **必须传 timeout。** fibre 的 find_any 默认 timeout=None，
            # 那是"阻塞到有设备连上为止"——没有设备时会永久挂住，不超时、
            # 不报错、不返回，表现为启动日志戛然而止而机械臂不动。
            # 参考项目自己的 probe_device.py 用的是 timeout=15.0。
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
        # 注意这条日志的措辞：**"sent" 不是 "enabled"。** 这行以前写的是
        # "motors enabled"，而它只是无条件打印的——设备是否真的加上了力矩，
        # 软件侧看不到。实测中 homing() 能驱动手臂而 move_j 完全无效，所以
        # "set_enable 到底有没有到达轴" 是一个必须靠人工确认的问题：使能后
        # 用手推 joint_1，推不动说明力矩在，推得动说明这行根本没生效。
        #
        # 不要把这行日志当成"电机已使能"的证据。
        self.get_logger().info('sending set_enable(True) ...')
        self.dummy_driver.robot.set_enable(True)
        self.get_logger().info(
            'set_enable(True) returned. Whether the axes actually hold torque '
            'cannot be read back from here; verify by hand if move_j does '
            'nothing (see tools/probe_attrs.py).')

        # 夹爪初始化。**必须在带超时的线程里做。**
        #
        # 实测过一次：日志停在 "motors enabled" 之后就不再前进，/joint_states
        # 一条也不发。主电机明明使能成功了，所以问题只在夹爪通道——夹爪的
        # fibre 调用阻塞住了，而它是在 on_configure 里同步执行的，于是整个
        # 节点冻死，后面的 activate 永远到不了，也就没有发布者。
        #
        # 夹爪挂住绝不能拖死整条 bringup：超时就放弃它、继续 configure，
        # 机械臂本体照样能动，夹爪以后可以单独再弄。
        self._init_gripper_with_timeout()

    def _init_gripper_with_timeout(self):
        """在线程里初始化夹爪，超时就放弃，不阻塞 configure。

        **默认跳过整个夹爪初始化。** 见 gripper_enable 参数的说明：夹爪的
        fibre 调用实测会阻塞且永不返回，而那个线程会一直持有 fibre 链路，
        把后续所有设备调用（包括 move_j）全部排队堵死。所以默认根本不碰它，
        要先在 `ros2 param set` 或启动参数里显式打开。
        """
        if not self.gripper_enable:
            self.get_logger().info(
                'gripper ENABLE is off (gripper_enable=false); skipping '
                'gripper init entirely. This is the default because the '
                'gripper fibre calls have been observed to block forever and '
                'starve every other device call. Start with '
                'gripper_enable:=true only when you are ready to debug the '
                'gripper itself.')
            self.gripper_ready = False
            return

        timeout = float(self.get_parameter('gripper_init_timeout_sec').value)
        result = {}

        def _work():
            try:
                # 分步记日志：夹爪通道一旦阻塞，日志停在哪个 "step" 上就
                # 直接指出是取句柄、设电流还是使能出的问题。
                self.get_logger().info('gripper step 1/3: reading hand handle')
                hand = self.dummy_driver.robot.hand
                self.get_logger().info(
                    f'gripper step 2/3: set_current_limit('
                    f'{self.grip_current_limit})')
                # 要在使能**之前**压低电流上限，否则夹爪可能一上来就用默认大
                # 电流，第一次装夹就压坏东西。
                hand.set_current_limit(self.grip_current_limit)
                self.get_logger().info('gripper step 3/3: set_enable(True)')
                hand.set_enable(True)
                result['ok'] = True
            except Exception as exc:  # noqa: BLE001
                result['err'] = f'{type(exc).__name__}: {exc}'

        self.get_logger().info(
            f'initialising gripper (timeout {timeout}s) ...')
        th = threading.Thread(target=_work, daemon=True,
                              name='gripper-init')
        th.start()
        th.join(timeout)

        if th.is_alive():
            # 线程还活着说明 fibre 调用没返回。
            #
            # **这是个严重状态，不只是"夹爪不可用"。** 那个线程还在持有
            # fibre 链路，后续每一个设备调用都会排在它后面，于是 move_j
            # 也会卡住 —— 实测就是这样：夹爪超时之后，homing 也一起挂死。
            # 所以这条日志要说清楚：不是"夹爪没了、别的还能用"，而是
            # "这条链路被占住了，机械臂也动不了，请重启进程"。
            self.get_logger().error(
                f'gripper init did NOT return within {timeout}s: the fibre call '
                'is blocked and this thread still holds the link. Every later '
                'device call, INCLUDING move_j, will queue behind it, so the '
                'arm will NOT move. Stop this process (Ctrl+C), power-cycle the '
                'arm, and restart with gripper_enable:=false to use the arm '
                'without the gripper.')
            self.gripper_ready = False
            return

        if result.get('ok'):
            self.gripper_ready = True
            self.get_logger().info(
                f'gripper enabled, current limit = {self.grip_current_limit}')
        else:
            self.gripper_ready = False
            self.get_logger().error(
                f'gripper init failed ({result.get("err", "unknown")}); '
                'continuing without gripper')

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
        # 用 Float64 而不是 GripperCommand 动作，是为了不引入 control_msgs
        # 依赖；等接 MoveIt 时再在外面包一层动作即可。
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
        # 原来只有一个 20 Hz 的定时器，里面既读 6 个 joint_N.angle、又读
        # hand.angle、然后才发布 —— 每秒 140 次阻塞式 USB 往返，而且全部在
        # 执行器线程里。后果：
        #   - 发布频率被 USB 往返时间拖住，名义 20 Hz 实际更低
        #   - RViz 收到的是"跳变"而不是连续变化，看起来就是"更新慢、没有过程"
        #   - 轨迹回调、订阅、服务都要和这些读数抢执行器
        #
        # 拆开之后：读数慢一点没关系（关节不会突变），发布则稳定按固定频率
        # 发缓存值，不碰 USB。这正是 RViz 需要的。
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

        这是本节点唯一直接触碰 robot.joint_N.angle 的地方。设备的这个读数
        既不等于 URDF 角度，也不等于 move_j 指令值，必须经 angle_to_urdf
        换算。上游原版直接把原始值当 URDF 弧度发布，所以 j3 差 90 度、模型
        放大约 57 倍。换算的推导见 config/dummy_controller.yaml。
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

        设备的 robot.joint_N.angle 是**原始编码器读数**（度），既不等于
        URDF 角度，也不等于 move_j 指令值，必须经过 angle_to_urdf 换算。
        上游原版直接把原始值当 URDF 弧度发布，所以 j3 差 90 度、模型放大约
        57 倍。换算的推导见 config/dummy_controller.yaml。
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

        映射：设备的 hand.angle 线性映射到归一化开合度，再乘以单指行程。
            frac   = (angle - grip_angle_closed) / (grip_angle_open - grip_angle_closed)
            travel = finger_travel_m * frac
        grip_angle_closed / grip_angle_open 必须实测（阶段 0.4），当前是占位值，
        填错只影响 RViz 里手指张开的幅度，不影响 TF 连通性，也不影响夹持力。
        """
        try:
            angle = float(getattr(self.dummy_driver.robot.hand, 'angle', 0.0))
        except Exception as exc:
            self.get_logger().warn(
                f'cannot read hand angle: {exc}', throttle_duration_sec=5.0)
            angle = 0.0

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
        """接收归一化夹爪指令。

        0.0 = 全闭，1.0 = 全开。超范围会被夹到 [0,1]。
        夹持力由 grip_current_limit 控制，在 configure 时已设好。
        """
        if not self.gripper_ready:
            # 夹爪初始化失败或超时过。这里的调用走同一条会阻塞的 fibre
            # 通道，如果在执行器线程里再阻塞一次，整个节点就彻底不响应了
            # ——包括 /joint_states。所以直接拒绝，而不是去试。
            self.get_logger().error(
                'gripper not ready (init failed or timed out); ignoring the '
                'command. See the "gripper init" line in the startup log.',
                throttle_duration_sec=5.0)
            return

        frac = max(0.0, min(1.0, float(msg.data)))
        angle = self.gripper_frac_to_angle(frac)
        self.gripper_target = frac
        self.get_logger().info(
            f'gripper command: frac={frac:.3f} -> angle={angle:.2f} deg')
        # 也放到线程里：即使这次 fibre 调用卡住，也只丢掉这一条指令，
        # 不会冻结执行器。
        threading.Thread(
            target=self._send_gripper_angle, args=(angle,),
            daemon=True, name='gripper-cmd').start()

    def _send_gripper_angle(self, angle):
        try:
            self.dummy_driver.robot.hand.set_angle(angle)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'gripper command failed: {exc}')

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

    def _send_named_pose(self, movej_deg, label):
        """把命名位姿下发到硬件。

        输入是 **move_j 空间**的角度（度），也就是 home_joints /
        reset_joints 的格式。**直接原样交给 move_j，不做任何变换。**

        为什么不做逆变换：home/reset 这些值是从上游驱动继承的，而上游
        是把它们直接发给 move_j 的，中间没有任何换算（上游根本没有换算
        函数）。实体机械臂已验证这些值能到达预期姿态，所以它们必然就是
        move_j 空间的。再套一层换算反而会把姿态改错。

        日志同时打印回读侧换算出的 URDF 角度，方便与 RViz 对照。
        """
        movej_deg = np.asarray(movej_deg, dtype=float)

        # 每一步都记日志。此前这段是"发指令 -> 回读 -> 打印"一口气走完，
        # 一旦 move_j 或回读阻塞，日志就停在上一行不动，无法判断卡在哪一个。
        # 实测正是如此：卡在 CONFIGURE 3/4 之后完全看不出是 move_j 还是回读。
        self.get_logger().info(f'{label}: step 1/2 sending move_j ...')
        try:
            self.move_joints(movej_deg)
        except Exception as exc:
            self.get_logger().error(f'{label} command failed: {exc}')
            return
        self.get_logger().info(f'{label}: step 1/2 sent')

        # 回读仅用于日志对照，与上一步的 try 分开。回读失败时如果也报
        # "{label} failed"，就又会制造"日志说失败、实物其实已经动了"这种
        # 最难查的假故障——那正是这里当初少一个 read_device() 时的症状。
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

    def home_dummy(self, wait=True):
        self._send_named_pose(self.home_joints, 'home')

    def stop_dummy(self):
        """急停：把设备**此刻的实际姿态**换算回 move_j 空间再下发一次。

        走 angle -> URDF -> move_j 这条被 tools/selftest_conversion.py 覆盖的
        标准两步链，不另拼公式。

        这里曾经有两个叠加的错误：把缓存 self.current_urdf_rad（URDF 弧度）
        喂给按"度"入参的换算函数，而且所用的 angle_to_movej 还漏了
        joint_offset_deg 项。两者合起来使下发的目标与当前姿态差了几十度，
        一旦真的接上急停按钮，按下去就是一次大角度抽动而不是停住。
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

        上游原版只取 msg.points[0]，等于把整条插值路径丢掉、直接跳到终点。
        MoveIt 规划出来的是几十个 waypoint，这样写轻则动作难看，重则瞬时
        大角度跳变损伤减速器。

        阶段 1 起按 time_from_start 节拍依次下发**全部** waypoint。

        joint_names 的顺序可能与我们的关节顺序不同，必须按名字重排，
        不能假设它们已经是 dummy2_joint1..6 的顺序。
        """
        # 收到轨迹先记一条日志。缺少这一行会导致「实物没动」时无法判断
        # 是回调没触发、还是被拒绝、还是执行了但没效果。
        self.get_logger().info(
            f'RECEIVED JointTrajectory: {len(msg.points)} point(s), '
            f'joint_names={list(msg.joint_names)}')

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

        # 当前命令位置（move_j 空间），作为"就近整圈等价"的参考。
        #
        # **用命令历史，不用回读反推。** 后者是 `raw - move_j_offset_deg`，
        # 那个式子没有物理依据：实测 j3 上它与真实命令位置相差 270 度，导致
        # 目标被推到设备拒绝的区间、整条命令静默失效。驱动自己知道发过什么，
        # 那才是权威来源。只有在本节点还没发过任何命令时才回退到回读。
        cur_movej = self.guess_current_movej()
        if cur_movej is None:
            self.get_logger().error(
                'cannot determine the current command pose (no command sent '
                'yet and the readback is unavailable); refusing to execute a '
                'trajectory because the nearest-equivalent conversion would be '
                'guessing.')
            return
        src = ('command history' if self.commanded_movej() is not None
               else 'readback fallback (no command sent yet)')
        self.get_logger().info(
            f'trajectory reference pose from {src}: '
            f'{[round(v, 2) for v in cur_movej]}')

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

            # URDF 弧度 -> 设备角度（含 sign/offset 逆变换），单位是度。
            # 传入当前位置，结果会被折算成离它最近的整圈等价角 —— 不这样做
            # 就会重演"下发 340 度、设备够不着、静默不动"那个 bug。
            movej_deg = self.urdf_to_movej(urdf_rad, current_deg=cur_movej)
            try:
                self.move_joints(movej_deg)
            except Exception as exc:
                self.get_logger().error(
                    f'Waypoint {idx} command failed: {exc}; aborting trajectory')
                return
            # 下一个 waypoint 以本 waypoint 为参考，保证轨迹连续
            cur_movej = [float(v) for v in movej_deg]

            if idx % 10 == 0 or idx == total - 1:
                self.get_logger().info(
                    f'  waypoint {idx + 1}/{total} '
                    f't={target_t:.2f}s deg={np.round(movej_deg, 2).tolist()}')

        self.get_logger().info('Trajectory execution complete')

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

