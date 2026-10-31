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

#: 抓取朝向量化到多少度。90 = 只区分"横着夹/竖着夹"，正好够正方形方块用；
#: 设成 0 就完全相信视觉给的 yaw（腕部会多转，见 `snap_yaw` 的注释）。
DEFAULT_YAW_SNAP_DEG = 90.0


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
    """方块边长取两条边里的**短边**，不取均值。

    先把实情写在这里：现在这个函数其实恒等于标称边长。
    `vision_node._to_message` 把 bbox.size 填成 `object_edge`（24 mm）的方框，
    不是拟合出来的尺寸，所以 `extent_m` 永远是 (0.024, 0.024)。

    仍然写成"取短边"是因为：一旦把拟合边长接进来，长边一个字节都不能信。
    `detection.detection_from_blob` 是把掩膜像素投到 z = table_z 的平面上再
    拟合矩形的，而方块有 24 mm 高 —— 顶面沿视线方向投到地面会被拉出一条
    "影子"。2026-10-22 真机实测（24 mm 方块，水平 0.302 m，相机离地 0.16 m）：
    拟合出 **86.2 x 29.5 mm**，长边是方块本身的 3.6 倍；按几何算
    (R+e/2)·H/(H−e) − (R−e/2) = 79 mm，再加掩膜肥边约 6 mm，正好 86 mm。
    短边 29.5 = 24 + 肥边，才是挨着真实尺寸的那条。那种尺寸下取均值会算成
    57.9 mm ⇒ 开口 59.9 mm ⇒ 超过 55 mm 的最大开口被直接拒掉。

    短边本身落在 [e, e·√2] 之间（见 detection.short_edge_error），当边长用是
    偏大的上界：最坏情况开口 = 24·√2 + 2 = 36 mm，仍小于 55 mm 的最大开口，
    也仍能夹住 24 mm 的方块（手指从 36 mm 收到 24 mm）。
    """
    a, b = (float(v) for v in detection.extent_m)
    if not (a > 0.0 and b > 0.0):
        raise NotGraspable('拟合边长不合理：{:.1f} x {:.1f} mm'.format(a * 1e3, b * 1e3))
    return min(a, b)


def snap_yaw(yaw_rad, snap_deg=DEFAULT_YAW_SNAP_DEG):
    """把抓取朝向量化到 `snap_deg` 的整数倍。

    为什么可以量化：开口是按**标称边长**算的（现在是 24 mm 的方框，见
    `_object_edge`），手指横着夹、竖着夹都能夹住，所以朝向只需要精确到 90°。

    为什么必须量化：`geometry.top_down_grasp_matrix(yaw)` 让工具系的 x 轴跟着
    方块的 yaw 走，而方块摆得随便（实测 yaw 4.3°、55.3° 都出现过）。任何一个
    非整数倍的 yaw 都要靠腕部（j6 甚至 j4）多转那半圈，规划出来就是"转了半天
    没什么用"的动作（2026-10-27 真机反馈）。量化之后 j6 只会在 0/90/180/270
    里挑，而且同一个方块每次挑的都是同一个。
    """
    if not snap_deg or float(snap_deg) <= 0.0:
        return float(yaw_rad)
    step = math.radians(float(snap_deg))
    return round(float(yaw_rad) / step) * step


def plan_grasp(detection, model: gr.FingerOpeningModel,
               limits: WorkspaceLimits = WorkspaceLimits(),
               approach_m: float = 0.08, lift_m: float = 0.10,
               place_point=None,
               tip_to_centre_m: float = DEFAULT_TIP_TO_CENTRE_M,
               fingertip_clearance_m: float = DEFAULT_FINGERTIP_CLEARANCE_M,
               object_offset=(0.0, 0.0),
               yaw_snap_deg: float = DEFAULT_YAW_SNAP_DEG,
               open_ratio: float = 1.0) -> GraspPlan:
    """由一次检测生成抓取计划；不能抓就抛 NotGraspable，原因写在消息里。

    `detection.position` 是**方块中心**（视觉给的就是这个），而 `gripper_tcp`
    是**两指夹持中心** —— 它在指尖上方 `tip_to_centre_m`（实测 17.5 mm）。
    所以 TCP 不能停在方块中心：那样两根指头会插进桌子里
    （17.5 − 12 = 5.5 mm）。正确的做法是让指尖停在离工作面
    `fingertip_clearance_m` 的地方，于是

        TCP 高度 = 工作面 + 余量 + tip_to_centre
                 = 方块中心 + 余量 + tip_to_centre − 边长/2

    对 24 mm 方块就是比中心高 8.5 mm。

    式子里那个"边长"必须和视觉算 position.z 时用的是同一个数。现在是：视觉
    发来的 `extent_m` 就是标称 24 mm（见 `_object_edge`），所以两边一致。
    哪天视觉改成发拟合尺寸，这里不能跟着换 —— 拟合边长里含 24 mm 方块投在
    桌面上的影子，比真实边长偏大，减掉的半个边长会多于视觉加上的，指尖就会
    往下沉（短边被掩膜肥边撑到 41.6 mm 时能低 5.8 mm，直接插进桌子）。
    """
    p = np.asarray(detection.position, float)
    if p.shape != (3,):
        raise NotGraspable('抓取点要 3 个数，收到 {} 个'.format(p.size))
    # 系统性偏差的补偿（视觉 + 执行合起来那几厘米）。默认是 0，要按实测数据填，
    # 怎么填见 tools/fit_grasp_correction.py 和 config/pick_place.yaml 的注释。
    dx, dy = (float(v) for v in object_offset)
    if dx or dy:
        p = np.array([p[0] + dx, p[1] + dy, p[2]], float)

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
    # 宽度 = 边长 × open_ratio + 2 mm 余量。默认 1.0，就是原来的"边长 + 2 mm"。
    # 2026-10-31 真机：26 mm 的开口对 24 mm 方块只有每侧 1 mm 的侧向容差，
    # 执行误差一到 10 mm 就是"手指顶在棱上、把方块推走"。开口放大 k 倍，
    # 侧向容差变成 (k−1)·边长/2，姿态容差也跟着变大；代价是手指要多开合一段。
    # 真正的闸门仍在 command_for_width()，最大开口超了这里就拒。
    if not float(open_ratio) >= 1.0:
        raise ValueError(
            'open_ratio 不能小于 1.0（开口比方块还窄就夹不到）：{}'.format(
                open_ratio))
    open_width = model.grasp_width_for(edge * float(open_ratio))

    # 手指要张得开：开口不能小于方块，也不能超过实测的最大开口。
    if open_width > model.width_open_m.value + 1e-9:
        raise NotGraspable(
            '要 {:.1f} mm 开口，夹爪最大只有 {:.1f} mm'.format(
                open_width * 1e3, model.width_open_m.value * 1e3))

    tcp_offset = fingertip_clearance_m + tip_to_centre_m - 0.5 * edge
    grasp_point = np.array([p[0], p[1], p[2] + tcp_offset], float)
    yaw = snap_yaw(float(detection.yaw_rad), yaw_snap_deg)

    try:
        wp = g.grasp_waypoints(grasp_point, yaw,
                               approach_m=approach_m, lift_m=lift_m,
                               place_point=place_point)
    except g.GeometryError as exc:
        raise NotGraspable('生成路点失败：{}'.format(exc))

    return GraspPlan(waypoints=wp, object_edge_m=edge, open_width_m=open_width,
                     yaw_rad=yaw, reach_m=reach,
                     gripper_ready=bool(model.verified),
                     tcp_offset_m=float(tcp_offset))


def pick_place_waypoints(plan: GraspPlan, place_xy, table_z, approach_m=0.08,
                         lift_m=0.10, place_z=None, place_z_frame='table',
                         place_surface_m=None, place_clearance_m=0.003,
                         place_lift_edges=0.0):
    """把放置点也算出来（首期放置点是参数给定的，不由视觉算）。

    默认放置高度取工作面 + 抬升量，避免"停在地面上"导致合爪时蹭着走。

    `place_z` 量的是"离工作面多高"还是"base_link 里的绝对高度"，由
    `place_z_frame` 说清楚：

    - `'table'`（默认）：从桌面往上量的高度（人拿尺子量的就是这种），内部加
      `table_z`；
    - `'base'`：已经是 base_link 的 z，直接用。

    **`place_surface_m` 是"放置台顶面"的高度，不是 TCP 的高度 —— 优先用它。**
    2026-10-30 的坑：放置台本身就是 55 mm 高，`place_z: 0.055` 于是把 TCP
    放在台面上，而物块挂在 TCP 下面 `tcp_offset + 边长/2`（24 mm 方块是
    20.5 mm），等于要物块**陷进台面 20.5 mm**，手指也会插进台面。

    给了 `place_surface_m` 就按"松手时物块底面停在台面上方
    `place_clearance_m`"倒推 TCP：

        TCP_z = table_z + place_surface_m + tcp_offset + 边长/2 + clearance

    这样换方块尺寸、换台子高度都不用重算那个数。

    `place_lift_edges` 是"再抬高几个边长"的粗调，默认 0。2026-10-31 真机：
    松手时物块还是蹭台面，用户要求再抬半个边长，于是填 0.5。它加在上面算完的
    z 上（`z += place_lift_edges * 边长`），所以对 place_z 的那几种写法一样有效；
    代价是物块松手后会掉半个边长，落地会偏一点，别随手调大。

    2026-10-27 之前的坑：`place_x`/`place_y` 声明了却从没被读过，放置点悄悄
    等于抓取点（"放置"其实是原地松手）。
    """
    if place_surface_m is not None:
        # 物块底面比 TCP 低这么多：手势是"夹住方块中心"，而 TCP 还比方块中
        # 心高 tcp_offset（手指 17.5 mm 长，不停高就会插进桌子）。
        below = float(plan.tcp_offset_m) + 0.5 * float(plan.object_edge_m)
        z = (float(table_z) + float(place_surface_m) + below
             + float(place_clearance_m))
    elif place_z is None:
        z = float(table_z) + float(approach_m)
    elif place_z_frame == 'table':
        z = float(table_z) + float(place_z)
    elif place_z_frame == 'base':
        z = float(place_z)
    else:
        raise ValueError(
            "place_z_frame 只能是 'table' 或 'base'，收到 {!r}".format(
                place_z_frame))
    z += float(place_lift_edges) * float(plan.object_edge_m)
    place = np.array([float(place_xy[0]), float(place_xy[1]), z], float)
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
