#!/usr/bin/env python3
"""视觉链路的纯几何：像素 -> base_link 下的抓取位姿。

这个文件里**没有 ROS、没有 OpenCV、没有 tf2**，只依赖 numpy。
这样它能在没有 ROS 的机器上被离线自测（tools/selftest_vision_geometry.py），
而自测要盯住的正是最难在现场调试的那部分：坐标轴约定。自测里专门有一条
检查盯着"不许 import ROS/OpenCV"，免得以后顺手 import 进来，把离线自测
变成跑不起来的东西。

轴向约定（REP-103，长度米、角度弧度）：

  camera_color_optical_frame   x 右、y 下、z 前（朝景物）。光学系的轴向与
                               REP-103 不同，这个差异由 rpy=(-pi/2, 0, -pi/2)
                               一次性转掉。本文件只跟光学系打交道，所以
                               像素 (u, v) 与 (x, y) 直接对应。
                               这个 frame 由相机驱动发布（orbbec_camera：
                               camera_link -> camera_color_frame -> 光学系）；
                               URDF 只负责挂载（base_link -> camera_link）和
                               裸 USB 相机的 usb_camera_optical_frame。
  tool0 / gripper_tcp          +y 是从手腕指向两指夹持中心的"接近方向"，
                               ±x 是两指开合方向（dummy2_finger_jointN 的
                               axis 就是 ±x；link7 -> tool0 -> gripper_tcp
                               两段 origin 也都在 +y 上）。
  顶抓（top-down）             gripper_tcp 的 +y 必须朝下（base_link 的 -z），
                               这样它沿 +y 靠近物体；两指开合方向落在桌面内，
                               绕竖直轴转 yaw 去贴合方块的一条边。

  ⚠ docs/01-视觉抓取方案.md 的"阶段 4"里写的是"tool0 的 +y（两指接近方向）
    与桌面平行"——这句话与同一句里的"顶抓"自相矛盾：顶抓时 +y 是朝下的，
    否则夹爪是横着从侧面伸过去的。URDF 是权威（见上），本文件按 +y 朝下实现。
    现场若发现方向反了，要改的是这里一处常数，不是散落各处的矩阵。

单目兜底的限制（相机没到货时用 driver 里现成的 USB 相机，就得靠它）：

    把像素投到 z = 桌面高度 的平面上，等价于假设"物体是贴在桌面上的平面图形"。
    真实物体有高度 h，它的顶面中心会被按 camera_height / (camera_height - h)
    的倍率往外推（见 monocular_planar_inflation）：0.6 m 高、50 mm 的方块就是
    9%，十几毫米。所以单目路线只能把数据通路跑通，验收精度得靠深度相机。
"""
import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


class GeometryError(RuntimeError):
    """几何上做不到：射线与平面平行、交点在相机背后、旋转矩阵退化等。

    宁可让上层报错，也不要发布一个悄悄算错的抓取点。
    """


# --------------------------------------------------------------------------
# 相机模型
# --------------------------------------------------------------------------

class PinholeCamera:
    """针孔 + plumb_bob 畸变，字段与 sensor_msgs/CameraInfo 一一对应。

    只取 `CameraInfo.d` 的**前 5 个**：(k1, k2, p1, p2, k3)，即 plumb_bob。

    有的驱动会发 8 个系数（本仓库的 Orbbec Gemini 335L 就是：它把
    `distortion_model` 写成 plumb_bob，`d` 却是 rational_polynomial 的
    `(k1, k2, p1, p2, k3, k4, k5, k6)` 顺序）。多出来的 k4..k6 在 plumb_bob
    公式里没有位置，这里丢掉它们并把个数记在 `ignored_distortion` 里，让上层
    能 warn 一句，而不是静默丢——Gemini 335L 实测这三个都是 0，丢掉没有代价，
    但换成别的相机就得重新判断。
    """

    def __init__(self, fx, fy, cx, cy, width=None, height=None,
                 distortion=(0.0, 0.0, 0.0, 0.0, 0.0)):
        if not (fx > 0.0 and fy > 0.0):
            raise GeometryError('fx/fy 必须为正，收到 fx={} fy={}'.format(fx, fy))
        self.fx = float(fx)
        self.fy = float(fy)
        self.cx = float(cx)
        self.cy = float(cy)
        self.width = None if width is None else int(width)
        self.height = None if height is None else int(height)
        d = tuple(float(v) for v in distortion)
        if not d:
            d = (0.0, 0.0, 0.0, 0.0, 0.0)
        if len(d) < 5:
            raise GeometryError(
                'distortion 至少要 5 个数 (k1,k2,p1,p2,k3)，收到 {} 个'.format(len(d)))
        self.ignored_distortion = len(d) - 5
        self.distortion = d[:5]

    @classmethod
    def from_camera_info(cls, k, width=None, height=None, d=None):
        """从 CameraInfo 的字段建。k 是长度 9 的行主序内参矩阵。"""
        if len(k) != 9:
            raise GeometryError('CameraInfo.k 要 9 个数，收到 {} 个'.format(len(k)))
        return cls(k[0], k[4], k[2], k[5], width=width, height=height,
                   distortion=tuple(d) if d else (0.0, 0.0, 0.0, 0.0, 0.0))

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0.0, self.cx],
                         [0.0, self.fy, self.cy],
                         [0.0, 0.0, 1.0]])

    def distort_normalized(self, x, y):
        """归一化坐标 -> 含畸变的归一化坐标（plumb_bob 正向，有闭式）。"""
        k1, k2, p1, p2, k3 = self.distortion
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return xd, yd

    def project(self, point_optical):
        """光学系里的点 -> 像素 (u, v)。含畸变；z <= 0 直接拒绝。"""
        x, y, z = (float(v) for v in point_optical)
        if z <= 0.0:
            raise GeometryError('点落在相机背后：z={}'.format(z))
        xd, yd = self.distort_normalized(x / z, y / z)
        return (self.fx * xd + self.cx, self.fy * yd + self.cy)

    def undistort_pixel(self, u, v, iterations=30, tolerance=1e-14):
        """像素 -> 去畸变后的归一化坐标 (x, y)。

        plumb_bob 没有闭式逆，用定点迭代。不收敛时抛异常而不是返回一个
        看起来像模像样的错值。
        """
        mx = (float(u) - self.cx) / self.fx
        my = (float(v) - self.cy) / self.fy
        x, y = mx, my
        for _ in range(iterations):
            xd, yd = self.distort_normalized(x, y)
            ex, ey = xd - mx, yd - my
            if abs(ex) < tolerance and abs(ey) < tolerance:
                return x, y
            x -= ex
            y -= ey
        xd, yd = self.distort_normalized(x, y)
        if abs(xd - mx) > 1e-6 or abs(yd - my) > 1e-6:
            raise GeometryError(
                '去畸变不收敛：像素 ({}, {}) 残差 ({:.3e}, {:.3e})'.format(
                    u, v, xd - mx, yd - my))
        return x, y

    def ray_optical(self, u, v):
        """像素 -> 光学系里的单位方向向量（z 朝前）。"""
        x, y = self.undistort_pixel(u, v)
        d = np.array([x, y, 1.0])
        return d / np.linalg.norm(d)

    def point_optical(self, u, v, depth_m):
        """像素 + 深度（沿光轴的 z，米）-> 光学系里的点。给深度相机用。"""
        if not depth_m > 0.0:
            raise GeometryError('深度必须为正，收到 {}'.format(depth_m))
        x, y = self.undistort_pixel(u, v)
        return np.array([x * float(depth_m), y * float(depth_m), float(depth_m)])

    def metres_per_pixel(self, depth_m):
        """这个深度上 1 像素对应多少米（x、y 分开算，焦距不同）。"""
        return (float(depth_m) / self.fx, float(depth_m) / self.fy)

    def pixel_error_to_metres(self, pixels, depth_m):
        """把"误差几个像素"换算成米 —— 远端精度就是这么估的。"""
        dx, dy = self.metres_per_pixel(depth_m)
        return (float(pixels) * dx, float(pixels) * dy)


# --------------------------------------------------------------------------
# 刚体变换（4x4 齐次矩阵）。节点里用 tf2 拿 base_link -> optical，
# 这里的函数给离线自测和固定关节用。
# --------------------------------------------------------------------------

def transform_from_xyz_rpy(xyz, rpy):
    """固定关节的 (xyz, rpy) -> 4x4 齐次矩阵。

    rpy 是 URDF 的约定：固定轴 XYZ，等价于把 Rz(yaw)·Ry(pitch)·Rx(roll)
    按这个顺序乘起来。xacro 里 camera_link_joint 的 origin 就是这个格式。
    """
    roll, pitch, yaw = (float(v) for v in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(xyz, float)
    return T


def invert_transform(T):
    """刚体变换求逆（用 R 正交的性质，不做通用求逆）。"""
    T = np.asarray(T, float)
    if T.shape != (4, 4):
        raise GeometryError('要 4x4 齐次矩阵，收到 {}'.format(T.shape))
    R = T[:3, :3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ T[:3, 3]
    return out


def intersect_plane_z(origin, direction, plane_z, min_t=1e-9):
    """直线与水平面 z = plane_z 的交点；平行、或交点在起点背后时返回 None。"""
    o = np.asarray(origin, float)
    d = np.asarray(direction, float)
    if abs(d[2]) < 1e-12:
        return None
    t = (float(plane_z) - o[2]) / d[2]
    if t <= min_t:
        return None
    return o + t * d


def backproject_pixel_to_plane(camera, u, v, T_base_optical, plane_z):
    """单目兜底：把像素投到 base_link 下 z = plane_z 的水平面上。

    返回 base_link 下的点。射线到不了那个平面时抛 GeometryError。
    """
    T = np.asarray(T_base_optical, float)
    if T.shape != (4, 4):
        raise GeometryError('T_base_optical 要 4x4，收到 {}'.format(T.shape))
    origin = T[:3, 3]
    direction = T[:3, :3] @ camera.ray_optical(u, v)
    hit = intersect_plane_z(origin, direction, plane_z)
    if hit is None:
        raise GeometryError(
            '像素 ({:.1f}, {:.1f}) 的射线到不了 z={:.4f}：'
            '要么相机在平面下方，要么射线朝上'.format(
                float(u), float(v), float(plane_z)))
    return hit


def monocular_planar_inflation(object_height_m, camera_height_above_plane_m):
    """单目反投影把"有高度的物体"往外推的倍率。

    顶面中心的真实半径是 r，被当成桌面上的点投出来是 r·z/(z-h)：
    h = 0 时为 1.0（无偏），h 越接近相机高度偏得越离谱。
    """
    h = float(object_height_m)
    z = float(camera_height_above_plane_m)
    if h < 0.0:
        raise GeometryError('物体高度不能为负：{}'.format(h))
    if h >= z:
        raise GeometryError('物体顶面 {} m 不低于相机 {} m，反投影无意义'.format(h, z))
    return z / (z - h)


def object_center_on_table(x, y, table_z, height_m):
    """桌面上一个高 height_m 的物体的几何中心 —— 顶抓的抓取点高度取它。"""
    return np.array([float(x), float(y), float(table_z) + float(height_m) / 2.0])


# --------------------------------------------------------------------------
# 顶抓姿态
# --------------------------------------------------------------------------

def top_down_grasp_matrix(yaw_rad):
    """顶抓姿态矩阵。列向量是 gripper_tcp 的三根轴在 base_link 里的方向：

        x = (cos yaw, sin yaw, 0)    两指开合方向，贴合方块的一条边
        y = (0, 0, -1)               接近方向，朝下
        z = x × y = (-sin yaw, cos yaw, 0)

    yaw 是物体在桌面内的朝向（从 +x 量起）。
    """
    c = math.cos(float(yaw_rad))
    s = math.sin(float(yaw_rad))
    x_axis = np.array([c, s, 0.0])
    y_axis = np.array([0.0, 0.0, -1.0])
    z_axis = np.cross(x_axis, y_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def quaternion_from_matrix(R):
    """旋转矩阵 -> 四元数 (x, y, z, w)（ROS 的字段顺序）。Shepperd 分支法。"""
    R = np.asarray(R, float)
    if R.shape != (3, 3):
        raise GeometryError('要 3x3 旋转矩阵，收到 {}'.format(R.shape))
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w, x, y, z = (m21 - m12) / s, 0.25 * s, (m01 + m10) / s, (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w, x, y, z = (m02 - m20) / s, (m01 + m10) / s, 0.25 * s, (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w, x, y, z = (m10 - m01) / s, (m02 + m20) / s, (m12 + m21) / s, 0.25 * s
    q = np.array([x, y, z, w], float)
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        raise GeometryError('退化的旋转矩阵，四元数无从谈起')
    q /= norm
    if q[3] < 0.0:
        q = -q          # 同一个姿态取 w >= 0 的那个表示，日志里好比对
    return (float(q[0]), float(q[1]), float(q[2]), float(q[3]))


def matrix_from_quaternion(q):
    """四元数 (x, y, z, w) -> 3x3 旋转矩阵。"""
    x, y, z, w = (float(v) for v in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise GeometryError('四元数长度为 0')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ])


def top_down_grasp_quaternion(yaw_rad):
    """顶抓姿态的四元数 (x, y, z, w)，直接塞进 Pose.orientation。"""
    return quaternion_from_matrix(top_down_grasp_matrix(yaw_rad))


@dataclass(frozen=True)
class GraspWaypoints:
    """一次顶抓的四个位置（base_link，米）与共用的姿态。

        pre_grasp   抓取点正上方 approach_m —— 先到这里，再直线下去
        grasp       抓取点本身
        lift        抓取点正上方 lift_m —— 合爪之后抬起来
        place       放置点（首期由参数给定，不由视觉算）
        quaternion  四个位置共用（顶抓全程不改末端姿态）

    位移都在 base_link 的 +z 上，所以它跟 yaw 无关，可以单独测。
    """
    pre_grasp: np.ndarray
    grasp: np.ndarray
    lift: np.ndarray
    place: np.ndarray
    quaternion: Tuple[float, float, float, float]

    def as_positions(self) -> Dict[str, np.ndarray]:
        return {'pre_grasp': self.pre_grasp,
                'grasp': self.grasp,
                'lift': self.lift,
                'place': self.place}


def grasp_waypoints(grasp_point, yaw_rad, approach_m=0.08, lift_m=0.10,
                    place_point=None):
    """由抓取点生成四个路点。默认的 8 cm / 10 cm 来自 docs/01 的阶段 5。"""
    if approach_m <= 0.0 or lift_m <= 0.0:
        raise GeometryError(
            'approach/lift 必须为正：approach={} lift={}'.format(approach_m, lift_m))
    p = np.asarray(grasp_point, float)
    if p.shape != (3,):
        raise GeometryError('抓取点要 3 个数，收到 {} 个'.format(p.size))
    up_approach = np.array([0.0, 0.0, float(approach_m)])
    up_lift = np.array([0.0, 0.0, float(lift_m)])
    place = p.copy() if place_point is None else np.asarray(place_point, float)
    return GraspWaypoints(pre_grasp=p + up_approach,
                          grasp=p.copy(),
                          lift=p + up_lift,
                          place=place,
                          quaternion=top_down_grasp_quaternion(yaw_rad))


# --------------------------------------------------------------------------
# 还没实测的数：集中放在一处，别让占位值散落在 xacro / yaml / 代码里
# --------------------------------------------------------------------------

@dataclass(frozen=True)
class Measurement:
    """一个数，加上它的来历。

    source 只有两种取值：

        'measured'     拿尺子/仪器在真机上量过，或者从设备回读来的
        'placeholder'  估计值，用来把代码跑通，**不能拿它做真机验收**

    where 写的是"怎么复现这个数"，不是"这个数是什么意思"。
    """
    value: float
    unit: str
    source: str
    where: str

    SOURCES = ('measured', 'placeholder')

    def __post_init__(self):
        if self.source not in self.SOURCES:
            raise GeometryError('source 只能是 {}，收到 {!r}'.format(
                self.SOURCES, self.source))

    @property
    def is_measured(self) -> bool:
        return self.source == 'measured'

    def describe(self) -> str:
        return '{} {} ({})'.format(self.value, self.unit, self.source)


@dataclass
class SceneMeasurements:
    """场景里那三个"必须实测、实测前只能拿占位值"的数。

    散在 xacro / yaml / 代码里的时候，没人说得清现在到底验到了哪一步。
    集中在这里，配一个 unmeasured()，节点启动时把还没量的报出来：

        [WARN] 还没实测：table_z = 0.0 m (placeholder)

    table_z        base_link 下**工作面**的高度（米）—— 方块底面贴在这个平面上。
                   有桌面时是桌面；本机这套**没有桌子，工件和底座在同一平面**，
                   于是 z = 0（base_link 的原点就在底座底面，j1 在 z = 0.0805，
                   是 78 mm 高的底座顶面）。量法：尺子量 base_link 原点所在平面
                   到工作面的距离，同平面时就是 0。
    tcp_offset_y   link7 端面到两指夹持中心的距离（米），也就是 URDF 里
                   gripper_tcp 的偏移（docs/04 的 A1）。量法：卡尺。
    object_edge    演示用方块的边长（米）。
    """
    table_z: Measurement
    tcp_offset_y: Measurement
    object_edge: Measurement

    def as_dict(self) -> Dict[str, Measurement]:
        return {'table_z': self.table_z,
                'tcp_offset_y': self.tcp_offset_y,
                'object_edge': self.object_edge}

    def unmeasured(self) -> List[str]:
        return sorted(name for name, m in self.as_dict().items() if not m.is_measured)

    def describe_unmeasured(self) -> str:
        items = ['{} = {}'.format(n, self.as_dict()[n].describe())
                 for n in self.unmeasured()]
        return '; '.join(items)


def default_scene_measurements() -> SceneMeasurements:
    """还没上真机时的占位值 —— 每一个都标了怎么量。

    tcp_offset_y 取的是 xacro 里 `tcp_offset_y` 的占位值 0.06；table_z 与
    object_edge 是为了让代码跑通而定的估计。真机验收前必须换成实测值，
    describe_unmeasured() 就是用来在启动日志里把这几个数揪出来的。
    """
    return SceneMeasurements(
        table_z=Measurement(0.0, 'm', 'placeholder',
                            'base_link 原点所在平面到桌面的距离，尺子量'),
        tcp_offset_y=Measurement(0.06, 'm', 'placeholder',
                                 'xacro 里 tcp_offset_y 的占位值；卡尺量 link7 端面到两指夹持中心'),
        object_edge=Measurement(0.04, 'm', 'placeholder',
                                '演示方块的边长，卡尺量'),
    )


def project_scene_measurements() -> SceneMeasurements:
    """本机这套的实际数值（2026-10-16 由用户实测/确认）。

    与 default_scene_measurements() 分开是有意的：默认值保持"全是占位"，
    好让占位那条路径一直被测到；这一份是这台机器现在的真实状态，节点在
    没有配置覆盖时用它，并且在日志里点名每个数的来历。

    - table_z = -0.0405：工作面是**地面**，不是底座底面。这台机器的底座垫在
      一个约 2.5 cm 高的地台上，而 base_link 的原点也不在底座最底面 —— 它是
      底座 stl 包围盒的中心，底面在 z = -0.0155（`base_link.stl` 的 z 范围
      -15.5 ~ +83.5 mm）。所以地面在 base_link 里是 -(0.0155 + 0.025) =
      -0.0405。方块放在地面上，抓取点的 z 就是 -0.0405 + 半个边长。
      （2026-10-19 用户确认：地台约 2.5 cm、相机中心离地 16 cm、方块在地面上。）
    - tcp_offset_y = 0.1125：link7 端面到两指夹持中心。2026-10-21 按实测重算：
      腕部到指尖 130 mm（80 mm 是舵机/驱动板本体 + 50 mm 手指，手指里 15 mm
      是滑动结构、35 mm 是夹取段），夹取中心在夹取段中点 ⇒ 离指尖 17.5 mm ⇒
      离腕部 130 - 17.5 = 112.5 mm。旧的 15 mm 是量错了位置（量到的是腕部
      外壳），它让 URDF 的手指比实物长 100 mm，一开碰撞检查就规划失败。
    - object_edge = 0.024：边长 24 mm 的方形方块。

    注意 tcp_offset_y 已经写进 `dummy2_urdf.xacro`（那里由
    `gripper_wrist_to_tip_m - gripper_tip_to_centre_m` 算出来），两处必须
    一致；改一处不改另一处，规划用的 TCP 和这里算的抓取点就会差几毫米。
    """
    return SceneMeasurements(
        table_z=Measurement(-0.0405, 'm', 'measured',
                            '地面：底座底面 z=-0.0155（stl 包围盒）+ 2.5 cm 地台；方块放在地面上'),
        tcp_offset_y=Measurement(0.1125, 'm', 'measured',
                                 '腕部→指尖 130 mm 减 夹取中心到指尖 17.5 mm（2026-10-21 尺量）'),
        object_edge=Measurement(0.024, 'm', 'measured',
                                '方形方块，边长 24 mm'),
    )
