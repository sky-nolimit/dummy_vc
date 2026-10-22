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


#: 两指夹持中心（gripper_tcp）到指尖的距离。2026-10-21 实测：腕部到指尖
#: 130 mm，其中夹取段 35 mm，夹持中心在指尖往上 17.5 mm（见
#: dummy2_urdf.xacro 文件头那段注释）。
DEFAULT_TIP_TO_CENTRE_M = 0.0175
#: 抓取时指尖离工作面留多少。太小会蹭桌面，太大方块夹不稳。
DEFAULT_FINGERTIP_CLEARANCE_M = 0.003


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
    #: 夹持中心（TCP）比方块中心高多少。手指有 17.5 mm 长，不停高一点就会
    #: 插进桌子 —— 见 plan_grasp() 的 docstring。
    tcp_offset_m: float = 0.0

    def describe(self):
        wp = self.waypoints
        return ('目标 ({:.3f}, {:.3f}, {:.3f}) m，夹持中心抬高 {:.1f} mm，'
                '抓取点上方 {:.0f} mm 起手，开口 {:.1f} mm，水平距离 {:.3f} m'.format(
                    wp.grasp[0], wp.grasp[1], wp.grasp[2],
                    self.tcp_offset_m * 1e3,
                    (wp.pre_grasp[2] - wp.grasp[2]) * 1e3,
                    self.open_width_m * 1e3, self.reach_m))


def _object_edge(detection) -> float:
    """方块边长取拟合矩形的**短边**，不取均值。

    为什么长边一个字节都不能信：`detection.detection_from_blob` 是把掩膜的
    像素投到 z = table_z 的平面上再拟合矩形的，而方块有 24 mm 高 —— 顶面沿
    视线方向投到地面会被拉出一条"影子"。2026-10-22 真机实测（24 mm 方块，
    水平 0.302 m，相机离地 0.16 m）：拟合出 **86.2 x 29.5 mm**，长边是方块
    本身的 3.6 倍；按几何算 (R+e/2)·H/(H−e) − (R−e/2) = 79 mm，再加掩膜肥边
    约 6 mm，正好 86 mm。短边 29.5 mm = 24 mm 边长 + 肥边，才是挨着真实尺寸
    的那条。

    之前取两条边的均值，(86.2 + 29.5)/2 = 57.9 mm ⇒ 开口 59.9 mm ⇒ 直接
    "夹爪最大只有 55 mm" 被拒，方块明明在视野正中却抓不了。

    短边本身落在 [e, e·√2] 之间（见 detection.short_edge_error），直接当边长
    用是偏大的上界：最坏情况开口 = 24·√2 + 2 = 36 mm，仍小于 55 mm 的最大
    开口，也仍能夹住 24 mm 的方块（手指从 36 mm 收到 24 mm）。
    """
    a, b = (float(v) for v in detection.extent_m)
    if not (a > 0.0 and b > 0.0):
        raise NotGraspable('拟合边长不合理：{:.1f} x {:.1f} mm'.format(a * 1e3, b * 1e3))
    return min(a, b)


def plan_grasp(detection, model: gr.FingerOpeningModel,
               limits: WorkspaceLimits = WorkspaceLimits(),
               approach_m: float = 0.08, lift_m: float = 0.10,
               place_point=None,
               tip_to_centre_m: float = DEFAULT_TIP_TO_CENTRE_M,
               fingertip_clearance_m: float = DEFAULT_FINGERTIP_CLEARANCE_M,
               object_edge_m: float = None) -> GraspPlan:
    """由一次检测生成抓取计划；不能抓就抛 NotGraspable，原因写在消息里。

    `detection.position` 是**方块中心**（视觉给的就是这个），而 `gripper_tcp`
    是**两指夹持中心** —— 它在指尖上方 `tip_to_centre_m`（实测 17.5 mm）。
    所以 TCP 不能停在方块中心：那样两根指头会插进桌子里
    （17.5 − 12 = 5.5 mm）。正确的做法是让指尖停在离工作面
    `fingertip_clearance_m` 的地方，于是

        TCP 高度 = 工作面 + 余量 + tip_to_centre
                 = 方块中心 + 余量 + tip_to_centre − 边长/2

    对 24 mm 方块就是比中心高 8.5 mm。

    上面那个"边长"必须和**视觉算 position.z 时用的边长**是同一个数（标称值，
    由 `object_edge_m` 传进来），不能用拟合出来的短边。因为
    `position.z = 工作面 + 边长/2`，若这里换成一个更大的边长，减掉的半个边长
    就比视觉加上的多，指尖目标跟着往下沉：掩膜肥边把短边撑到 29.5 mm 时指尖
    只剩 0.25 mm 余量，撑到 41.6 mm（实测到过）就插进桌子 5.8 mm。拟合短边
    只用来决定开口宽度（见 `_object_edge`）。
    """
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

    # 手指要张得开：开口不能小于方块，也不能超过实测的最大开口。
    if open_width > model.width_open_m.value + 1e-9:
        raise NotGraspable(
            '要 {:.1f} mm 开口，夹爪最大只有 {:.1f} mm'.format(
                open_width * 1e3, model.width_open_m.value * 1e3))

    # 抬高的基准用标称边长（和视觉写 position.z 时用的那个一致），没给就退回
    # 拟合短边 —— 那样指尖会下沉，见 docstring。
    height_edge = edge if object_edge_m is None else float(object_edge_m)
    tcp_offset = fingertip_clearance_m + tip_to_centre_m - 0.5 * height_edge
    grasp_point = np.array([p[0], p[1], p[2] + tcp_offset], float)

    try:
        wp = g.grasp_waypoints(grasp_point, float(detection.yaw_rad),
                               approach_m=approach_m, lift_m=lift_m,
                               place_point=place_point)
    except g.GeometryError as exc:
        raise NotGraspable('生成路点失败：{}'.format(exc))

    return GraspPlan(waypoints=wp, object_edge_m=edge, open_width_m=open_width,
                     yaw_rad=float(detection.yaw_rad), reach_m=reach,
                     gripper_ready=bool(model.verified),
                     tcp_offset_m=float(tcp_offset))


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
