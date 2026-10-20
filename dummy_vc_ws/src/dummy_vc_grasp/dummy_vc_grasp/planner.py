"""抓取规划：把一次视觉检测变成一个可执行的抓取计划，或者一条"为什么不行"。

这个模块**不 import ROS**，也不碰话题，只做纯几何与规则判断。这样它能用
`tools/selftest_grasp_task.py` 离线验，而真机上出问题时也能分清是"看错了"
（`/detected_objects` 本身不对）还是"计划拒了"（这里给出的理由）。

为什么要有这一层，而不是让状态机直接拿着 Detection3D 去 MoveIt：一个检测通过
了尺寸判据，仍然可能**不可抓** —— 太远（超行程）、太高（两指张不开）、太贴近
底座（肘部撞自己）、在工作面以下（投影穿模）。这些判断放在这里，理由能写进
日志，出错时一眼看得出是哪条拒的。

夹爪那一层单独说：宽度是纯算术（边长 + 2 mm），未实测的模型也算得出来；真正
不许过的是"把宽度翻成 hand.set_angle"那一步（`command_for_width()` 会抛
`UnverifiedGripperModel`）。所以这里照常规划，只把 `gripper_ready` 记进计划，
由状态机决定不许进 CLOSE_GRIPPER。
"""

import math
from dataclasses import dataclass

import numpy as np

from dummy_vc_perception import geometry as g
from dummy_vc_perception import gripper as gr


class NotGraspable(RuntimeError):
    """这次检测不能抓，原因在消息里。"""


@dataclass(frozen=True)
class WorkspaceLimits:
    """抓取工作区（base_link，米/弧度）。默认值按本机臂展与安装位置取。"""
    max_reach_m: float = 0.45        # 底座中心到抓取点的水平距离上限
    min_reach_m: float = 0.15        # 太近则肘部姿态别扭、也容易被底座挡住
    min_height_m: float = -0.041     # 工作面（地面）附近的容差下限
    max_height_m: float = 0.25       # 再高就是举着抓，顶抓不成立
    max_yaw_from_down_rad: float = math.pi  # 顶抓时绕竖直轴的朝向本无限制


@dataclass(frozen=True)
class GraspPlan:
    """一次抓取要的全部信息，够状态机直接照着走。"""
    waypoints: g.GraspWaypoints
    object_edge_m: float
    open_width_m: float
    yaw_rad: float
    reach_m: float
    #: 夹爪的"角度↔开口"是否已实测。未实测时规划照做（宽度只要算术），
    #: 但状态机不会走到 CLOSE_GRIPPER —— 见 task.TaskMachine。
    gripper_ready: bool = False

    def describe(self):
        wp = self.waypoints
        return ('目标 ({:.3f}, {:.3f}, {:.3f}) m，抓取点上方 {:.0f} mm 起手，'
                '开口 {:.1f} mm，水平距离 {:.3f} m'.format(
                    wp.grasp[0], wp.grasp[1], wp.grasp[2],
                    (wp.pre_grasp[2] - wp.grasp[2]) * 1e3,
                    self.open_width_m * 1e3, self.reach_m))


def _object_edge(detection) -> float:
    """方块边长取两条拟合边的均值。

    短边判据已经保证了"短边贴着边长或对角线"，但两条边本身是掩膜外接矩形的
    长短边，长边在斜视下会被拉长。取均值是个折中：它偏向长边一点，而夹爪开口
    本来就要留 2 mm 余量，宁可大一点也不要夹不进去。
    """
    a, b = (float(v) for v in detection.extent_m)
    if not (a > 0.0 and b > 0.0):
        raise NotGraspable('拟合边长不合理：{:.1f} x {:.1f} mm'.format(a * 1e3, b * 1e3))
    return 0.5 * (a + b)


def plan_grasp(detection, model: gr.FingerOpeningModel,
               limits: WorkspaceLimits = WorkspaceLimits(),
               approach_m: float = 0.08, lift_m: float = 0.10,
               place_point=None) -> GraspPlan:
    """由一次检测生成抓取计划；不能抓就抛 NotGraspable，原因写在消息里。"""
    p = np.asarray(detection.position, float)
    if p.shape != (3,):
        raise NotGraspable('抓取点要 3 个数，收到 {} 个'.format(p.size))

    reach = float(math.hypot(p[0], p[1]))
    if reach > limits.max_reach_m:
        raise NotGraspable(
            '目标水平距离 {:.3f} m 超过上限 {:.3f} m'.format(reach, limits.max_reach_m))
    if reach < limits.min_reach_m:
        raise NotGraspable(
            '目标水平距离 {:.3f} m 小于下限 {:.3f} m（太近，肘部会别扭）'.format(
                reach, limits.min_reach_m))
    if p[2] < limits.min_height_m:
        raise NotGraspable(
            '抓取点高度 {:.3f} m 低于工作面附近的容差下限 {:.3f} m'.format(
                p[2], limits.min_height_m))
    if p[2] > limits.max_height_m:
        raise NotGraspable(
            '抓取点高度 {:.3f} m 高于上限 {:.3f} m（顶抓不成立）'.format(
                p[2], limits.max_height_m))

    edge = _object_edge(detection)
    # 宽度是纯算术（边长 + 2 mm 余量），未实测的模型也能算；真正的闸门在
    # command_for_width()，只有状态机走到合爪那一步才会碰它。
    open_width = model.grasp_width_for(edge)

    try:
        wp = g.grasp_waypoints(p, float(detection.yaw_rad),
                               approach_m=approach_m, lift_m=lift_m,
                               place_point=place_point)
    except g.GeometryError as exc:
        raise NotGraspable('生成路点失败：{}'.format(exc))

    return GraspPlan(waypoints=wp, object_edge_m=edge, open_width_m=open_width,
                     yaw_rad=float(detection.yaw_rad), reach_m=reach,
                     gripper_ready=bool(model.verified))


def pick_place_waypoints(plan: GraspPlan, place_xy, table_z, approach_m=0.08,
                         lift_m=0.10):
    """把放置点也算出来（首期放置点是参数给定的，不由视觉算）。

    放置高度取工作面 + 抬升量，避免"停在地面上"导致合爪时蹭着走。
    """
    place = np.array([float(place_xy[0]), float(place_xy[1]),
                      float(table_z) + float(approach_m)], float)
    return g.grasp_waypoints(plan.waypoints.grasp, plan.yaw_rad,
                             approach_m=approach_m, lift_m=lift_m,
                             place_point=place)


def describe_detection(detection) -> str:
    """一行话把一次检测说清楚，日志和报错都用它。"""
    return ('({:.3f}, {:.3f}, {:.3f}) m，yaw={:.1f}°，拟合 {:.1f}x{:.1f} mm，'
            '距离 {:.3f} m'.format(
                detection.position[0], detection.position[1], detection.position[2],
                math.degrees(detection.yaw_rad),
                detection.extent_m[0] * 1e3, detection.extent_m[1] * 1e3,
                detection.distance_m))


def first_graspable(detections, model, limits=WorkspaceLimits(), **kwargs):
    """按顺序试，返回 (plan, 拒绝原因列表)。一个都不行时 plan 是 None。"""
    reasons = []
    for i, det in enumerate(detections):
        try:
            return plan_grasp(det, model, limits=limits, **kwargs), reasons
        except NotGraspable as exc:
            reasons.append('候选 {}：{}'.format(i, exc))
    return None, reasons
