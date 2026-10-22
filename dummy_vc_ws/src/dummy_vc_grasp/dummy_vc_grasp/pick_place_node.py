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
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint,
                             RobotState)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Empty
from vision_msgs.msg import Detection3DArray

import numpy as np

from dummy_vc_perception import geometry as g
from dummy_vc_perception import gripper as gr

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

        # -- 抓取几何 -------------------------------------------------------
        self.declare_parameter('approach_m', 0.08)
        self.declare_parameter('lift_m', 0.10)
        self.declare_parameter('place_x', 0.0)
        self.declare_parameter('place_y', 0.10)
        # 默认值与 scene_objects / perception 的实测值同源，别在这里写死一个数
        self.declare_parameter('table_z', scene.DEFAULT_TABLE_Z)
        # 标称边长：必须和 perception 的 vision.yaml 里那个 object_edge 一致。
        # 视觉用它在 position.z 上加了"方块半高"，抬高的基准少了它就会把指尖
        # 往下压（见 grasp_planner.plan_grasp 的 docstring）。
        self.declare_parameter('object_edge', 0.024)
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
            gripper_ready=self.gripper_model.verified)

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

        self.action_client = ActionClient(
            self, MoveGroup, self.get_parameter('move_action').value)
        self.cartesian_client = self.create_client(
            GetCartesianPath, self.get_parameter('cartesian_service').value)
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
        # 100 ms 的节拍：管超时、也管"上一步完成了该干什么"。
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            'pick_place_node 起来了：group={} ee={} base={}，execute={}（false = 只规划），'
            '夹爪{}'.format(
                self.group_name, self.ee_link, self.base_frame,
                'true' if self.execute else 'false（只规划不执行）',
                '已实测' if self.gripper_model.verified
                else '模型含占位值，走到合爪会停下（{}）'.format(
                    self.gripper_model.describe())))
        self.get_logger().info(
            '下降{}'.format(
                '走笛卡尔竖直直线（算不出来就退回关节空间）'
                if bool(self.get_parameter('approach_cartesian').value)
                else '退回关节空间规划（approach_cartesian=false）'))
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
        elif action == 'move_pre_grasp':
            self._send_goal(self.plan.waypoints.pre_grasp,
                            self.plan.waypoints.quaternion, 'move_pre_grasp')
        elif action == 'descend':
            self._descend()
        elif action == 'lift':
            self._send_goal(self.plan.waypoints.lift,
                            self.plan.waypoints.quaternion, 'lift')
        elif action == 'move_place':
            self._send_goal(self.plan.waypoints.place,
                            self.plan.waypoints.quaternion, 'move_place')
        elif action == 'retreat':
            # 退回起手位：拿放置点上方当"退开"，避免直接扫过桌面。
            self._send_goal(self.plan.waypoints.pre_grasp,
                            self.plan.waypoints.quaternion, 'retreat')
        elif action == 'close_gripper':
            self._command_gripper(self.plan.open_width_m)
        elif action == 'open_gripper':
            self._command_gripper(self.gripper_model.width_open_m.value)
        elif action == 'stop':
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            self.action_busy = False

    def _command_gripper(self, width_m):
        """唯一会碰 `hand.*` 的地方。未实测时 command_for_width 会抛。"""
        try:
            angle = self.gripper_model.command_for_width(width_m)
        except gr.UnverifiedGripperModel as exc:
            self._decide_failure('夹爪拒绝下发：{}'.format(exc))
            return
        # 走到这里只说明"角度↔开口"已实测；驱动侧仍然没有夹爪接口
        # （A2：`hand.set_angle` 会让整机失去响应），所以不许谎报成功。
        self._decide_failure(
            '夹爪映射已实测（{:.1f} mm -> {:.2f} deg），但驱动侧没有夹爪接口'
            '（A2 未解决），这一步没有真正下发'.format(width_m * 1e3, angle))

    def _decide_success(self, note=''):
        if note:
            self.get_logger().info(note)
        self._decide(task_mod.Event.SUCCEEDED)

    def _decide_failure(self, note):
        self.get_logger().warn(note)
        self._decide(task_mod.Event.FAILED)

    def _on_start(self, _msg):
        self.get_logger().info('收到开始信号：{}'.format(
            self.get_parameter('start_topic').value))
        self._decide(task_mod.Event.START)

    # ------------------------------------------------------------------
    # 视觉
    # ------------------------------------------------------------------
    def _on_detections(self, msg):
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
            lift_m=float(self.get_parameter('lift_m').value),
            object_edge_m=float(self.get_parameter('object_edge').value))
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
        self.get_logger().info(
            '算竖直下降：从起手位 ({:.3f}, {:.3f}, {:.3f}) m 直下到 ({:.3f}, {:.3f}, {:.3f}) m'.format(
                self.plan.waypoints.pre_grasp[0],
                self.plan.waypoints.pre_grasp[1],
                self.plan.waypoints.pre_grasp[2],
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
    def _tick(self):
        state = self.machine.state
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
