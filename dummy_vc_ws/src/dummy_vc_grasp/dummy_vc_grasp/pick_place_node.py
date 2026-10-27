"""阶段 5 的 ROS 接线：订阅检测 -> 生成抓取路点 -> 让 MoveIt 走完 -> 夹爪。

这一层只做几件事，其余全在 `planner.py` / `task.py` 里（那两层能离线自测）：

  1. 订阅 `/detected_objects`，把 `Detection3D` 转成 `SceneDetection`
  2. 调 MoveIt 的 `MoveGroup` 动作（plan+execute 一个 goal 搞定，碰撞检查与
     路径插值都还是 MoveIt 在管，不用自己拼轨迹）
  3. 把动作结果翻译成状态机事件，按状态机给的 action 决定下一步
  4. 走到合爪那一步时问 `gripper.FingerOpeningModel` 要角度；没实测就停

两处例外值得单独说：

* **下降（APPROACH）不走关节空间规划**，改用 `GetCartesianPath` 算一条竖直
  直线。关节空间规划只保证"某条无碰撞的关节轨迹存在"，于是最后 8 cm 可以是
  斜着下去、甚至从侧面绕 —— 抓取恰恰要求从正上方压下去。算出来不够直
  （`fraction < cartesian_min_fraction`）就退回关节空间并在日志里说清楚。
* **碰撞体由 `scene_node.py` 负责**，这个节点只管"往哪走"。地面和地台不在
  URDF 里，没有它们，规划器会认为脚下什么都没有。

**默认不执行，只规划**（`execute:=false`）。理由：夹爪 A2 没解决、相机与机械臂
还不能同时在线，这条链第一次上机时最该先看的是"它打算往哪儿走"，而不是让一台
没验过的节点直接驱动机械臂。确认路点对了再加 `-p execute:=true`。
"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint,
                             RobotState)
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty, Float64
from vision_msgs.msg import Detection3DArray

import numpy as np

from dummy_vc_perception import geometry as g
from dummy_vc_perception import gripper as gr

from dummy_vc_grasp import gripper_gate as gripper_gate_mod
from dummy_vc_grasp import planner as grasp_planner
from dummy_vc_grasp import scene_objects as scene
from dummy_vc_grasp import task as task_mod


class Detection(object):
    """`dummy_vc_perception.detection.SceneDetection` 的等价物。

    视觉节点发的是 `Detection3DArray`，里面没有"拟合边长"这种字段，但有
    bbox.size —— 边长就从 bbox 取，其余照抄。
    """

    def __init__(self, position, yaw_rad, quaternion, extent_m, distance_m):
        self.position = np.asarray(position, float)
        self.yaw_rad = float(yaw_rad)
        self.quaternion = tuple(float(v) for v in quaternion)
        self.extent_m = (float(extent_m[0]), float(extent_m[1]))
        self.edge_error_m = 0.0
        self.distance_m = float(distance_m)
        self.metres_per_pixel = (0.0, 0.0)


class PickPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_place_node')

        # -- 接口 -----------------------------------------------------------
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('planning_group', 'dummy_arm')
        self.declare_parameter('ee_link', 'gripper_tcp')
        self.declare_parameter('base_frame', 'base_link')

        # -- 安全 -----------------------------------------------------------
        # 默认只规划不执行：链路第一次上机时先看它打算往哪儿走。
        self.declare_parameter('execute', False)
        self.declare_parameter('max_velocity_scaling', 0.15)
        self.declare_parameter('max_acceleration_scaling', 0.15)
        self.declare_parameter('planning_time_s', 5.0)

        # -- 夹爪 -----------------------------------------------------------
        #
        # 夹爪**不走驱动**：驱动里那条 `hand.*` 会把 0x382 当"设角度"发出去，而
        # 在真板子上 0x382 是**触发标定**（详见 docs/04 的 A2 与 gripper_frames）。
        # 现在走旁路：`gripper_can_node` 把 `/gripper/width`（毫米）编成 0x385，
        # 把 0x3A3 回读发到 `/gripper/position`。这条通道 2026-10-27 真机验过。
        #
        # 默认 `gripper_enabled=False`：没有 USB-CAN 适配器的人跑这个 demo，
        # 照样能验到"竖直下降"那一步，然后在合爪处停下 —— 与以前的默认行为一致。
        self.declare_parameter('gripper_enabled', False)
        self.declare_parameter('gripper_cmd_topic', '/gripper/width')
        self.declare_parameter('gripper_state_topic', '/gripper/position')
        #: 张开到多宽。55 是机械满开，但 53 以上会顶到本体堵转，留一点余量
        self.declare_parameter('gripper_open_m', 0.050)
        #: 闭合时比方块窄多少（有这点预压才有夹持力）
        self.declare_parameter('gripper_clearance_m', 0.002)
        #: 闭合目标的下限（再窄就是夹手指自己）
        self.declare_parameter('gripper_min_close_m', 0.004)
        #: 实测比目标宽多少才算"被方块挡住"
        self.declare_parameter('gripper_blocked_m', 0.0008)
        #: 合爪后等回读的时间；超时还没等到就判空夹/无回读
        self.declare_parameter('gripper_close_timeout_s', 2.5)
        #: 张爪后等多久再往下走（张开不需要证据，只要给它时间动）
        self.declare_parameter('gripper_open_settle_s', 0.8)

        # -- 抓取几何 -------------------------------------------------------
        self.declare_parameter('approach_m', 0.08)
        self.declare_parameter('lift_m', 0.10)
        self.declare_parameter('place_x', 0.0)
        self.declare_parameter('place_y', 0.10)
        # 默认值与 scene_objects / perception 的实测值同源，别在这里写死一个数
        self.declare_parameter('table_z', scene.DEFAULT_TABLE_Z)
        self.declare_parameter('max_reach_m', 0.45)
        self.declare_parameter('min_reach_m', 0.15)

        # -- 节奏 -----------------------------------------------------------
        # 等目标的上限。别设太小：相机（合成节点容器）起来要十几秒，
        # 超时太短会把"相机还没起来"报成"桌上没方块"。
        self.declare_parameter('observe_timeout_s', 60.0)
        self.declare_parameter('action_timeout_s', 40.0)
        self.declare_parameter('max_retries', 2)
        #: 调试用：节点一起来就自动开始一次抓取（默认等外部 START）
        self.declare_parameter('auto_start', False)
        #: 手动开始一次抓取的话题（发任意一条 std_msgs/Empty 即可）
        self.declare_parameter('start_topic', '/pick_place/start')

        # -- 下降那一段（竖直直线） -----------------------------------------
        #: 下降走笛卡尔直线；关掉就退回关节空间规划（最后几厘米可能是斜的）
        self.declare_parameter('approach_cartesian', True)
        self.declare_parameter('cartesian_service', '/compute_cartesian_path')
        self.declare_parameter('execute_trajectory_action', '/execute_trajectory')
        #: 直线插值的步长；越小越"直"，但也越容易因为局部奇异算不出来
        self.declare_parameter('cartesian_max_step', 0.005)
        #: 算出来的直线覆盖率低于这个值就退回关节空间规划
        self.declare_parameter('cartesian_min_fraction', 0.95)

        # -- 起手位够不着时往下压 -------------------------------------------
        #
        # 2026-10-22 真机：方块在 r≈0.30 m，起手位（抓取点正上方 80 mm）发过去
        # 之后 MoveIt 回 error_code=99999，ompl 报
        # "Unable to sample any valid states for goal tree"。原因不是碰撞，是
        # **顶抓姿态下那个高度够不着**：夹爪 +y 朝下时，这条手臂在 r=0.302 m
        # 处 z 只能到 0.00 附近（离线 FK/IK 验算，见 docs/04），而 0.060 m 要
        # 高 60 mm。物体在 r≈0.27 m 以内时 8 cm 起手位是够的，一次改死
        # approach_m 会牺牲近处的余量，所以这里**运行时问一次 IK**，够不着才
        # 压低，压到离抓取点 approach_min_m 还不行就报清楚，别让 MoveIt 白等。
        self.declare_parameter('ik_service', '/compute_ik')
        self.declare_parameter('ik_timeout_s', 0.2)
        #: 起手位最多压到离抓取点多高（再低就没法竖直下降了）
        self.declare_parameter('approach_min_m', 0.005)
        #: 每次往下压多少
        self.declare_parameter('approach_step_m', 0.01)

        self.base_frame = self.get_parameter('base_frame').value
        self.ee_link = self.get_parameter('ee_link').value
        self.group_name = self.get_parameter('planning_group').value
        self.execute = bool(self.get_parameter('execute').value)

        # 夹爪模型：**故意**用未实测的占位模型。真机上要先把阶段 0.5 量完，
        # 再用 opening_endpoints_from_measurements() 换上，否则状态机不会合爪。
        self.gripper_model = gr.unverified_placeholder_model()
        self.limits = grasp_planner.WorkspaceLimits(
            max_reach_m=float(self.get_parameter('max_reach_m').value),
            min_reach_m=float(self.get_parameter('min_reach_m').value))

        self.machine = task_mod.TaskMachine(
            max_retries=int(self.get_parameter('max_retries').value),
            gripper_ready=bool(self.get_parameter('gripper_enabled').value))

        self.plan = None
        self.last_detection = None
        self.action_busy = False
        self.action_started = 0.0
        self.goal_handle = None
        self.current_label = ''
        #: 只规划模式下，"到起手位"这条规划出来的轨迹。下降要从它的**末点**
        #: 起算 —— 只规划时机械臂并没有真的动过去，用"当前状态"算出来的直线
        #: 是从它真正站着的地方出发的，那不是我们要验的东西。
        self.planned_pre_grasp = None
        #: 经 IK 预检之后真正用的位姿（label -> 位置）。“退开”那一步直接复用
        #: 起手位，不再问一遍 IK。
        self.fitted = {}
        self._probe = None
        #: 夹爪最近一次回读的开口（米）。None = 还没收到过 `/gripper/position`
        self.gripper_width_m = None
        #: 合爪/张爪等待的结束时刻与目标开口（秒 / 米）
        self._gripper_deadline = 0.0
        self._gripper_target = 0.0
        #: 'close' / 'open' / ''（没在等）
        self._gripper_mode = ''
        #: 收到过多少条检测（不分状态，用来判断视觉链路是否通）
        self.received_frames = 0
        #: 数 OBSERVE 里收到多少条（判超时用）
        self.observe_frames = 0
        #: 预检的流水号：重试时旧的回调可能晚到，靠它丢掉过期的结果
        self._probe_seq = 0

        self.action_client = ActionClient(
            self, MoveGroup, self.get_parameter('move_action').value)
        self.cartesian_client = self.create_client(
            GetCartesianPath, self.get_parameter('cartesian_service').value)
        self.ik_client = self.create_client(
            GetPositionIK, self.get_parameter('ik_service').value)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory,
            self.get_parameter('execute_trajectory_action').value)
        self.create_subscription(
            Detection3DArray,
            self.get_parameter('detections_topic').value,
            self._on_detections, 10)
        # 手动触发：auto_start 会把"相机还没起来"和"桌上没方块"混在一起，
        # 所以默认留一个由人/上层决定何时开始的入口。
        self.create_subscription(
            Empty, self.get_parameter('start_topic').value, self._on_start, 10)
        # 夹爪：只走旁路话题（`gripper_can_node` 那一路），不碰驱动里的 `hand.*`。
        self.gripper_cmd = self.create_publisher(
            Float64, self.get_parameter('gripper_cmd_topic').value, 10)
        self.create_subscription(
            Float64, self.get_parameter('gripper_state_topic').value,
            self._on_gripper_width, 10)
        # 100 ms 的节拍：管超时、也管"上一步完成了该干什么"。
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'pick_place_node 起来了：group={} ee={} base={}，execute={}（false = 只规划），'
            '夹爪{}'.format(
                self.group_name, self.ee_link, self.base_frame,
                'true' if self.execute else 'false（只规划不执行）',
                '走旁路 {} -> {}（合爪会真的夹，靠回读判）'.format(
                    self.get_parameter('gripper_cmd_topic').value,
                    self.get_parameter('gripper_state_topic').value)
                if bool(self.get_parameter('gripper_enabled').value)
                else '没开（gripper_enabled=false）：能走到竖直下降，然后在合爪处停下'))
        self.get_logger().info(
            '下降{}'.format(
                '走笛卡尔竖直直线（算不出来就退回关节空间）'
                if bool(self.get_parameter('approach_cartesian').value)
                else '退回关节空间规划（approach_cartesian=false）'))
        if not bool(self.get_parameter('auto_start').value):
            # 不打印这一句的话，节点启动后安静地等触发，看起来就像卡死了
            # （2026-10-22 真机上被当成"vision:=false 之后卡住"）。
            self.get_logger().info(
                '还没开始：等 {} 触发（auto_start=false）。检测从 {} 进来 —— '
                '本节点不直接连相机，视觉由 vision_node 发；vision:=false 只表示'
                '这里不再起相机/视觉，不是不再收检测'.format(
                    self.get_parameter('start_topic').value,
                    self.get_parameter('detections_topic').value))
        if bool(self.get_parameter('auto_start').value):
            self._decide(task_mod.Event.START)

    # ------------------------------------------------------------------
    # 状态机
    # ------------------------------------------------------------------
    def _decide(self, event):
        dec = self.machine.decide(event)
        self.get_logger().info('[{}] {} -> {}：{}'.format(
            event.value, dec.action, dec.state.value, dec.note))
        if dec.state is task_mod.State.FAULT:
            self.get_logger().error('任务停在 FAULT：{}'.format(dec.note))
        self._dispatch(dec.action)
        return dec

    def _dispatch(self, action):
        if action == 'wait_target':
            self.plan = None
            self.last_detection = None
            self.planned_pre_grasp = None
            self.fitted = {}
            self._probe = None
            self._gripper_mode = ''
        elif action == 'move_pre_grasp':
            self._send_goal_fitted(
                self.plan.waypoints.pre_grasp, self.plan.waypoints.quaternion,
                'move_pre_grasp')
        elif action == 'descend':
            # 下降前先把手指张开，否则是攥着拳往下压
            self._open_gripper()
            self._descend()
        elif action == 'lift':
            # 抬升同样会被"顶抓姿态抬不高"卡住：grasp + lift_m 在远处一样无解
            self._send_goal_fitted(
                self.plan.waypoints.lift, self.plan.waypoints.quaternion, 'lift')
        elif action == 'move_place':
            self._send_goal(self.plan.waypoints.place,
                            self.plan.waypoints.quaternion, 'move_place')
        elif action == 'retreat':
            # 退回起手位：拿放置点上方当"退开"，避免直接扫过桌面。
            self._send_goal(
                self.fitted.get('move_pre_grasp', self.plan.waypoints.pre_grasp),
                self.plan.waypoints.quaternion, 'retreat')
        elif action == 'close_gripper':
            self._close_gripper(self.plan.object_edge_m)
        elif action == 'open_gripper':
            self._open_gripper()
        elif action == 'stop':
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.action_busy = False
            self._gripper_mode = ''

    def _publish_gripper(self, width_m):
        self.gripper_cmd.publish(Float64(data=float(width_m) * 1e3))

    def _open_gripper(self):
        """张到 `gripper_open_m`。张开不需要证据，等一小会儿让它动完就行。"""
        if not bool(self.get_parameter('gripper_enabled').value):
            return
        if not self.execute:
            # 只规划模式下不去动夹爪：手臂根本没动，张/合都是在空气里演一遍，
            # 徒增困惑（而且合爪会因为没夹到东西直接判 EMPTY 失败）。
            self.get_logger().info('只规划（execute=false）：不动夹爪')
            return
        width = float(self.get_parameter('gripper_open_m').value)
        self._gripper_target = width
        self._gripper_deadline = time.time() + float(
            self.get_parameter('gripper_open_settle_s').value)
        self._gripper_mode = 'open'
        self._publish_gripper(width)
        self.get_logger().info('张爪：下发 {:.1f} mm'.format(width * 1e3))

    def _close_gripper(self, edge_m):
        """合爪：按方块宽度算目标，发出去，再靠回读判"夹住没有"。"""
        if not bool(self.get_parameter('gripper_enabled').value):
            self._decide_failure(
                '合爪这一步没开：gripper_enabled=false。夹爪走的是 USB-CAN 旁路'
                '（gripper_can_node + /gripper/width），不是驱动里的 hand.*')
            return
        if not self.execute:
            # 只规划：不去夹（也没东西可夹），直接放行，让四个路点都能规划一遍。
            self.get_logger().info(
                '只规划（execute=false）：不夹，直接放行到抬升')
            self._gripper_mode = ''
            self._decide_success('只规划模式：跳过了合爪')
            return
        try:
            target = gripper_gate_mod.close_width(
                edge_m,
                float(self.get_parameter('gripper_clearance_m').value),
                float(self.get_parameter('gripper_min_close_m').value))
        except ValueError as exc:
            self._decide_failure('夹爪拒绝合爪：{}'.format(exc))
            return
        self._gripper_target = target
        self._gripper_deadline = time.time() + float(
            self.get_parameter('gripper_close_timeout_s').value)
        self._gripper_mode = 'close'
        self._publish_gripper(target)
        self.get_logger().info(
            '合爪：方块 {:.1f} mm，下发 {:.1f} mm（窄 {:.1f} mm 预压），等回读…'
            .format(edge_m * 1e3, target * 1e3, (edge_m - target) * 1e3))

    def _check_gripper(self):
        """合爪/张爪的等待逻辑。由 `_tick` 每 100 ms 调一次。"""
        mode = getattr(self, '_gripper_mode', '')
        if not mode:
            return
        timed_out = time.time() >= self._gripper_deadline
        if mode == 'open':
            if timed_out:
                self._gripper_mode = ''
                if self.gripper_width_m is None:
                    self.get_logger().warn(
                        '张爪：{} 一直没有回读，继续往下走（张开没有证据也能接受）'
                        .format(self.get_parameter('gripper_state_topic').value))
                else:
                    self.get_logger().info('张爪：实测 {:.1f} mm'.format(
                        self.gripper_width_m * 1e3))
            return
        result = gripper_gate_mod.verdict(
            self._gripper_target, self.gripper_width_m,
            float(self.get_parameter('gripper_blocked_m').value),
            timed_out=timed_out)
        if result is None:
            return
        self._gripper_mode = ''
        detail = gripper_gate_mod.describe(
            result, self._gripper_target, self.gripper_width_m)
        if result == gripper_gate_mod.HELD:
            self._decide_success('夹住了：{}'.format(detail))
        elif result == gripper_gate_mod.EMPTY:
            self._decide_failure(
                '手指合到头了，中间没有东西：{}。方块没在两指之间，或者检测给的位置'
                '偏了'.format(detail))
        else:
            self._decide_failure(
                '夹爪没有回读（{} 没消息）：判不了夹没夹住，不往下走。'
                '先看 gripper_can_node 还在不在'.format(
                    self.get_parameter('gripper_state_topic').value))

    def _decide_success(self, note=''):
        if note:
            self.get_logger().info(note)
        self._decide(task_mod.Event.SUCCEEDED)

    def _decide_failure(self, note):
        self.get_logger().warn(note)
        self._decide(task_mod.Event.FAILED)

    def _on_gripper_width(self, msg):
        """夹爪开口回读（毫米）。只存下来，判断在 `_check_gripper` 里做。"""
        self.gripper_width_m = float(msg.data) * 1e-3

    def _on_start(self, _msg):
        self.get_logger().info('收到开始信号：{}'.format(
            self.get_parameter('start_topic').value))
        self._idle_since = None
        self._idle_hinted = False
        self.observe_frames = 0
        self._observe_started = None
        self._decide(task_mod.Event.START)

    # ------------------------------------------------------------------
    # 视觉
    # ------------------------------------------------------------------
    def _on_detections(self, msg):
        # 先记一条"视觉这条链是通的"，跟状态机无关：没触发之前也要能看出来
        # 相机/vision_node 到底有没有在发。
        self.received_frames += 1
        if self.received_frames == 1:
            self.get_logger().info(
                '收到第 1 条 {}（{} 个目标）—— 视觉这条链是通的'.format(
                    self.get_parameter('detections_topic').value,
                    len(msg.detections)))
        if self.machine.state is not task_mod.State.OBSERVE:
            return
        self.observe_frames = getattr(self, 'observe_frames', 0) + 1
        dets = []
        for det in msg.detections:
            pos = det.bbox.center.position
            quat = det.bbox.center.orientation
            yaw = _yaw_from_quaternion(quat.x, quat.y, quat.z, quat.w)
            size = (det.bbox.size.x, det.bbox.size.y)
            dets.append(Detection([pos.x, pos.y, pos.z], yaw,
                                  (quat.x, quat.y, quat.z, quat.w), size,
                                  float(np.linalg.norm([pos.x, pos.y, pos.z]))))
        if not dets:
            return
        plan, reasons = grasp_planner.first_graspable(
            dets, self.gripper_model, limits=self.limits,
            approach_m=float(self.get_parameter('approach_m').value),
            lift_m=float(self.get_parameter('lift_m').value))
        for why in reasons:
            self.get_logger().info('跳过：{}'.format(why))
        if plan is None:
            self.get_logger().info('这一帧没有可抓的目标（{} 个候选都被拒）'.format(
                len(dets)))
            return
        self.plan = plan
        self.last_detection = dets[0]
        self.get_logger().info('抓到目标：{}'.format(plan.describe()))
        self._decide(task_mod.Event.TARGET)

    # ------------------------------------------------------------------
    # MoveIt
    # ------------------------------------------------------------------
    def _send_goal(self, position, quaternion, label=''):
        if not self.action_client.wait_for_server(timeout_sec=2.0):
            self._decide_failure('MoveIt 的 {} 动作服务端不在（move_group 起了吗？）'.format(
                self.get_parameter('move_action').value))
            return
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = float(
            self.get_parameter('planning_time_s').value)
        goal.request.max_velocity_scaling_factor = float(
            self.get_parameter('max_velocity_scaling').value)
        goal.request.max_acceleration_scaling_factor = float(
            self.get_parameter('max_acceleration_scaling').value)
        goal.request.goal_constraints = [
            _constraints(self.base_frame, self.ee_link, position, quaternion)]
        goal.planning_options.plan_only = not self.execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        self.action_busy = True
        self.action_started = time.time()
        self.current_label = label
        self.get_logger().info('发给 MoveIt：({:.3f}, {:.3f}, {:.3f}) m，{}'.format(
            position[0], position[1], position[2],
            '执行' if self.execute else '只规划'))
        self.action_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    # ------------------------------------------------------------------
    # 可达性预检：顶抓姿态抬不高，够不着就先往下压
    # ------------------------------------------------------------------
    def _send_goal_fitted(self, position, quaternion, label):
        """先问一遍 `/compute_ik`：这个高度够得着吗？够不着就往下压。

        压只改 z（x/y 与姿态不动），所以压完照样能竖直下降，只是下降行程短了；
        下限是抓取点上方 `approach_min_m`。压到底还无解就报清楚 —— 那说明物体
        已经出了"顶抓"这个姿态的作业范围，再让 MoveIt 去试也只是白等 5 秒。
        问不到 IK 服务时按原目标发（和以前完全一样），不因为预检而变保守。
        """
        if not self.ik_client.service_is_ready():
            if not self.ik_client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(
                    '没有 {} 服务，{} 跳过可达性预检 —— 够不着时 MoveIt 只会回 '
                    'error_code=99999（goal tree 采不到解），看不出为什么'.format(
                        self.get_parameter('ik_service').value, label))
                self._send_goal(position, quaternion, label)
                return
        self._probe_seq += 1
        self._probe = {
            'label': label,
            'wanted': np.array(position, float),
            'position': np.array(position, float),
            'quaternion': quaternion,
            'floor_z': float(self.plan.waypoints.grasp[2])
            + float(self.get_parameter('approach_min_m').value),
            'steps': 0,
            'seq': self._probe_seq,
        }
        self.action_busy = True
        self.action_started = time.time()
        self._probe_ik()

    def _probe_ik(self):
        probe = self._probe
        timeout = float(self.get_parameter('ik_timeout_s').value)
        request = GetPositionIK.Request()
        request.ik_request.group_name = self.group_name
        # is_diff：以机械臂当前状态为种子（MoveIt 默认也是这么干的）
        request.ik_request.robot_state.is_diff = True
        request.ik_request.avoid_collisions = True
        request.ik_request.ik_link_name = self.ee_link
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose = Pose()
        pose.position.x = float(probe['position'][0])
        pose.position.y = float(probe['position'][1])
        pose.position.z = float(probe['position'][2])
        pose.orientation.x = float(probe['quaternion'][0])
        pose.orientation.y = float(probe['quaternion'][1])
        pose.orientation.z = float(probe['quaternion'][2])
        pose.orientation.w = float(probe['quaternion'][3])
        request.ik_request.pose_stamped.pose = pose
        request.ik_request.timeout = Duration(
            sec=int(timeout), nanosec=int((timeout % 1.0) * 1e9))
        self.ik_client.call_async(request).add_done_callback(
            lambda f: self._on_ik(f, probe['seq']))

    def _on_ik(self, future, seq):
        probe = self._probe
        if probe is None or probe['seq'] != seq or seq != self._probe_seq:
            return                              # 过期结果（重试/超时之后才回来）
        feasible = False
        detail = ''
        try:
            response = future.result()
        except Exception as exc:                # noqa: BLE001
            detail = '调用失败：{}'.format(exc)
        else:
            feasible = response.error_code.val == MoveItErrorCodes.SUCCESS
            detail = 'error_code={}'.format(response.error_code.val)

        if feasible:
            self._probe = None
            self.action_busy = False
            fitted = tuple(float(v) for v in probe['position'])
            self.fitted[probe['label']] = fitted
            dropped = probe['wanted'][2] - probe['position'][2]
            if dropped > 1e-9:
                self.get_logger().warn(
                    '{} 的高度够不着：({:.3f}, {:.3f}, {:+.3f}) m 在顶抓姿态下'
                    '没有逆解（{}），压低 {:.0f} mm 改成 z={:+.3f} m'
                    '（离抓取点 {:.0f} mm）'.format(
                        probe['label'], probe['wanted'][0], probe['wanted'][1],
                        probe['wanted'][2], detail, dropped * 1e3,
                        probe['position'][2],
                        (probe['position'][2]
                         - float(self.plan.waypoints.grasp[2])) * 1e3))
            self._send_goal(probe['position'], probe['quaternion'],
                            probe['label'])
            return

        step = float(self.get_parameter('approach_step_m').value)
        lowered = probe['position'][2] - step
        if lowered < probe['floor_z'] - 1e-9:
            self._probe = None
            self.action_busy = False
            self._decide_failure(
                '{} 的目标位姿在关节限位内无解（{}）——压低到离抓取点 {:.0f} mm '
                '还是无解，说明物体已经出了顶抓姿态的作业范围。把方块往近处挪'
                '（离底座水平距离 r < 0.27 m 时 8 cm 起手位够得着；r≈0.30 m 时'
                '顶抓最多只能抬到 z≈0.00，也就是抓取点上方约 20 mm）'.format(
                    probe['label'], detail,
                    float(self.get_parameter('approach_min_m').value) * 1e3))
            return
        probe['position'][2] = lowered
        probe['steps'] += 1
        self._probe_ik()

    def _on_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:                # noqa: BLE001 - 一定要回事件
            self._decide_failure('发目标失败：{}'.format(exc))
            return
        if not handle.accepted:
            self._decide_failure('MoveIt 拒了这个目标')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self.action_busy = False
        self.goal_handle = None
        label = self.current_label
        try:
            result = future.result().result
        except Exception as exc:                # noqa: BLE001
            self._decide_failure('取结果失败：{}'.format(exc))
            return
        code = result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self._decide_failure('MoveIt 失败{}：error_code={}'.format(
                '（{}）'.format(label) if label else '', code))
            return
        # 只规划模式下，把"到起手位"那条轨迹的末点记下来：下降要从这里起算。
        if label == 'move_pre_grasp' and not self.execute:
            self._remember_planned_pre_grasp(result)
        self._decide_success('MoveIt 完成{}（error_code={}）'.format(
            '（{}）'.format(label) if label else '', code))

    def _remember_planned_pre_grasp(self, result):
        points = result.planned_trajectory.joint_trajectory.points
        if not points:
            self.planned_pre_grasp = None
            self.get_logger().warn(
                '只规划模式下 MoveIt 没回轨迹（planned_trajectory 是空的），'
                '下降会从"当前状态"起算 —— 那个状态没动过，算出来的不是真机上的直线')
            return
        self.planned_pre_grasp = result.planned_trajectory
        self.get_logger().info(
            '记下起手位轨迹：{} 个点，末点取作为下降的起点'.format(len(points)))

    # ------------------------------------------------------------------
    # 下降：笛卡尔直线
    # ------------------------------------------------------------------
    def _descend(self):
        if not bool(self.get_parameter('approach_cartesian').value):
            self._send_goal(self.plan.waypoints.grasp,
                            self.plan.waypoints.quaternion, 'descend')
            return
        service = self.get_parameter('cartesian_service').value
        if not self.cartesian_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                '没有 {} 服务，这一趟退回关节空间规划（下降可能不是竖直的）'.format(
                    service))
            self._send_goal(self.plan.waypoints.grasp,
                            self.plan.waypoints.quaternion, 'descend')
            return

        request = GetCartesianPath.Request()
        request.header.frame_id = self.base_frame
        request.header.stamp = self.get_clock().now().to_msg()
        request.group_name = self.group_name
        request.link_name = self.ee_link
        request.max_step = float(self.get_parameter('cartesian_max_step').value)
        # 0.0 = 不做跳变检查；我们要的是"能不能一直连下去"，不是"跳得远不远"
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(
            self.get_parameter('max_velocity_scaling').value)
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('max_acceleration_scaling').value)

        pose = Pose()
        position = self.plan.waypoints.grasp
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = float(position[2])
        quaternion = self.plan.waypoints.quaternion
        pose.orientation.x = float(quaternion[0])
        pose.orientation.y = float(quaternion[1])
        pose.orientation.z = float(quaternion[2])
        pose.orientation.w = float(quaternion[3])
        # 只要一个路点：起点是"现在"（或规划出来的起手位），终点是抓取点，
        # 两点之间同姿态 —— 插出来的就是一条竖直直线。
        request.waypoints = [pose]
        request.start_state = self._descend_start_state()

        self.action_busy = True
        self.action_started = time.time()
        self.current_label = 'descend'
        # 实际站着的起手位可能被压低过（_send_goal_fitted），日志按真实的那个报
        start = self.fitted.get('move_pre_grasp', self.plan.waypoints.pre_grasp)
        self.get_logger().info(
            '算竖直下降：从起手位 ({:.3f}, {:.3f}, {:.3f}) m 直下到 ({:.3f}, {:.3f}, {:.3f}) m'.format(
                start[0], start[1], start[2],
                position[0], position[1], position[2]))
        self.cartesian_client.call_async(request).add_done_callback(
            self._on_cartesian)

    def _descend_start_state(self):
        """下降的起点：执行时用真实当前状态，只规划时用规划出来的起手位。"""
        state = RobotState()
        planned = self.planned_pre_grasp
        if not self.execute and planned is not None:
            points = planned.joint_trajectory.points
            if points:
                joints = JointState()
                joints.name = list(planned.joint_trajectory.joint_names)
                joints.position = [float(v) for v in points[-1].positions]
                state.joint_state = joints
                state.is_diff = False
                return state
        # 机械臂真的站在起手位上（execute）或者没有规划可参照：用当前状态
        state.is_diff = True
        return state

    def _on_cartesian(self, future):
        try:
            response = future.result()
        except Exception as exc:                # noqa: BLE001
            self.action_busy = False
            self.get_logger().warn('笛卡尔下降算不出来：{}'.format(exc))
            self._send_goal(self.plan.waypoints.grasp,
                            self.plan.waypoints.quaternion, 'descend')
            return
        fraction = float(response.fraction)
        points = len(response.solution.joint_trajectory.points)
        minimum = float(self.get_parameter('cartesian_min_fraction').value)
        if fraction < minimum or points == 0:
            self.action_busy = False
            self.get_logger().warn(
                '竖直下降只算出 {:.0%}（{} 个路点），退回关节空间规划 —— '
                '这一趟的下降可能不是直的，日志里会说明'.format(fraction, points))
            self._send_goal(self.plan.waypoints.grasp,
                            self.plan.waypoints.quaternion, 'descend')
            return
        self.get_logger().info('竖直下降算出来了：直线覆盖 {:.0%}，{} 个路点'.format(
            fraction, points))
        if not self.execute:
            self.action_busy = False
            self._decide_success('只规划：竖直下降的轨迹已算出（未下发）')
            return
        self._send_trajectory(response.solution)

    def _send_trajectory(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=2.0):
            self.action_busy = False
            self._decide_failure(
                'MoveIt 的 {} 动作服务端不在，算出来的下降轨迹发不出去'.format(
                    self.get_parameter('execute_trajectory_action').value))
            return
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        self.action_busy = True
        self.action_started = time.time()
        self.get_logger().info('下发竖直下降轨迹')
        self.execute_client.send_goal_async(goal).add_done_callback(
            self._on_execute_response)

    def _on_execute_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:                # noqa: BLE001
            self.action_busy = False
            self._decide_failure('发下降轨迹失败：{}'.format(exc))
            return
        if not handle.accepted:
            self.action_busy = False
            self._decide_failure('MoveIt 拒了下降轨迹')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_execute_result)

    def _on_execute_result(self, future):
        self.action_busy = False
        self.goal_handle = None
        try:
            code = future.result().result.error_code.val
        except Exception as exc:                # noqa: BLE001
            self._decide_failure('取下降结果失败：{}'.format(exc))
            return
        if code == MoveItErrorCodes.SUCCESS:
            self._decide_success('MoveIt 完成（descend，error_code={}）'.format(code))
        else:
            self._decide_failure('下降执行失败：error_code={}'.format(code))

    # ------------------------------------------------------------------
    # 节拍
    # ------------------------------------------------------------------
    def _idle_hint(self):
        """IDLE 里一声不吭会被当成卡死：等 5 秒还没人触发就说清在等什么。"""
        started = getattr(self, '_idle_since', None)
        if started is None:
            self._idle_since = time.time()
            return
        if getattr(self, '_idle_hinted', False):
            return
        if time.time() - started < 5.0:
            return
        self._idle_hinted = True
        start_topic = self.get_parameter('start_topic').value
        topic = self.get_parameter('detections_topic').value
        self.get_logger().info(
            '已经在 IDLE 等了 {:.0f} 秒，没收到 {}。视觉是 {} 在发（另一个终端里的 '
            'vision_node），本节点不直接连相机。确认有数据：ros2 topic echo --once {}；'
            '触发抓取：ros2 topic pub --once {} std_msgs/msg/Empty "{{}}"'.format(
                time.time() - started, start_topic, topic, topic, start_topic))

    def _tick(self):
        state = self.machine.state
        if state is task_mod.State.IDLE:
            self._idle_hint()
            return
        self._idle_since = None
        if state is task_mod.State.OBSERVE:
            started = getattr(self, '_observe_started', None)
            if started is None:
                self._observe_started = time.time()
                self.observe_frames = 0
            elif (time.time() - self._observe_started
                  > float(self.get_parameter('observe_timeout_s').value)):
                self._observe_started = None
                if self.observe_frames == 0:
                    # 一条 /detected_objects 都没收到：这跟"桌上没方块"是两回事。
                    self.get_logger().error(
                        '等目标超时：{} 秒里一条 {} 都没收到 —— vision_node 大概没在发。'
                        '先看它的日志（有没有 listening、有没有"连续 N 帧没找到合格的方块"）'
                        .format(self.get_parameter('observe_timeout_s').value,
                                self.get_parameter('detections_topic').value))
                else:
                    self.get_logger().error(
                        '等目标超时：收到 {} 条 {}，但没有一条能抓 —— 看 vision_node 里'
                        '"被丢掉的 blob"给出的原因'.format(
                            self.observe_frames,
                            self.get_parameter('detections_topic').value))
                self._decide(task_mod.Event.TIMEOUT)
            return
        self._observe_started = None
        self._check_gripper()
        if self.action_busy and (time.time() - self.action_started
                                 > float(self.get_parameter('action_timeout_s').value)):
            self.get_logger().warn('动作超时，取消目标')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
            self.action_busy = False
            self._decide(task_mod.Event.TIMEOUT)


def _yaw_from_quaternion(x, y, z, w):
    return float(np.arctan2(2.0 * (w * z + x * y),
                            1.0 - 2.0 * (y * y + z * z)))


def _constraints(frame, link, position, quaternion):
    pos = PositionConstraint()
    pos.header.frame_id = frame
    pos.link_name = link
    pos.weight = 1.0
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.005]
    volume = BoundingVolume()
    volume.primitives = [sphere]
    center = Pose()
    center.position.x = float(position[0])
    center.position.y = float(position[1])
    center.position.z = float(position[2])
    center.orientation.w = 1.0
    volume.primitive_poses = [center]
    pos.constraint_region = volume

    ori = OrientationConstraint()
    ori.header.frame_id = frame
    ori.link_name = link
    ori.weight = 1.0
    ori.orientation.x = float(quaternion[0])
    ori.orientation.y = float(quaternion[1])
    ori.orientation.z = float(quaternion[2])
    ori.orientation.w = float(quaternion[3])
    # 顶抓只要求"朝下"这个姿态，绕竖直轴的容差留一点，否则 IK 容易解不出来。
    ori.absolute_x_axis_tolerance = 0.05
    ori.absolute_y_axis_tolerance = 0.05
    ori.absolute_z_axis_tolerance = 0.4

    cons = Constraints()
    cons.position_constraints = [pos]
    cons.orientation_constraints = [ori]
    return cons


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
