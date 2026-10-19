"""把分割出来的掩膜变成 base_link 下的抓取目标。纯 numpy，可离线自测。

这一层是"几何"的下游、"ROS 节点"的上游，夹在中间的两个理由：

1. 它真的会算错。掩膜质心投到平面上、朝向拟合、抓取点取多高，每一项错了
   外观都一样：抓取点偏几毫米到几厘米。而这些都能用合成数据离线验证，
   不必等相机、不必上真机。
2. 它不该知道 cv2 和 ROS 的存在。`vision_node.py` 负责拿图、分割、查 TF；
   这里只吃"掩膜 + 相机内参 + 一个 4x4 变换"，所以本机（没 cv2、没 ROS）
   就能跑 `tools/selftest_vision_geometry.py` 把它钉死。

坐标系约定（与 `geometry.py` 一致）：像素是 (u, v)，u 向右、v 向下，
原点在左上角；输出的位置一律在 base_link 下、单位米。
"""

from dataclasses import dataclass
from typing import List, Sequence, Tuple

import numpy as np

from .geometry import (GeometryError, backproject_pixel_to_plane,
                       invert_transform, object_center_on_table,
                       top_down_grasp_quaternion)

# 640x480 上，24 mm 方块在 0.6 m 处大约 20x20 像素 = 400 px。留一个数量级的
# 余量当门限，既能滤掉噪点，也不会把远处的方块误杀掉。
MIN_BLOB_AREA_PX = 40

# 投影到平面上之后，少于这么多个点就没法可靠地拟合朝向。
MIN_PROJECTED_POINTS = 12

# 朝向拟合：先粗扫再细化。正方形的边长对齐角只需在 [0, pi/2) 里找。
COARSE_STEP_DEG = 2.0
FINE_STEP_DEG = 0.02
FINE_WINDOW_DEG = 3.0


@dataclass(frozen=True)
class Blob:
    """一个连通的彩色块，已经在像素空间里量过了。"""
    area_px: int
    u: float                    # 质心列坐标（像素）
    v: float                    # 质心行坐标（像素）
    points: np.ndarray          # (N, 2) 的像素坐标 [[u, v], ...]

    @classmethod
    def from_points(cls, points):
        """从一组像素坐标造一个 blob（测试和自定义分割都用这个入口）。"""
        arr = np.asarray(points, float)
        if arr.ndim != 2 or arr.shape[1] != 2:
            raise GeometryError('像素坐标要是 (N, 2)，收到 {}'.format(arr.shape))
        if arr.shape[0] == 0:
            raise GeometryError('空 blob')
        return cls(area_px=int(arr.shape[0]),
                   u=float(arr[:, 0].mean()),
                   v=float(arr[:, 1].mean()),
                   points=arr)


@dataclass(frozen=True)
class EdgeYaw:
    """在平面内拟合出来的外接矩形：朝向加两条边长（米）。"""
    yaw_rad: float
    extent_a_m: float
    extent_b_m: float


@dataclass(frozen=True)
class SceneDetection:
    """一个抓取目标。position 是方块几何中心（base_link 下，米）。"""
    position: np.ndarray
    yaw_rad: float
    quaternion: Tuple[float, float, float, float]
    extent_m: Tuple[float, float]
    edge_error_m: float         # 拟合出的边长与期望边长差多少，用来看"分割错没错"
    distance_m: float           # 相机到抓取点的距离，用来换算像素精度
    metres_per_pixel: Tuple[float, float]


def blobs_from_labels(labels, min_area_px=MIN_BLOB_AREA_PX,
                      max_blobs=8) -> List[Blob]:
    """`cv2.connectedComponents` 的 labels 图 -> [Blob]（0 号是背景）。

    按面积从大到小排序、只留前 max_blobs 个：演示场景里出现最大的那个就是
    目标方块，剩下的通常是反光或者噪点。
    """
    arr = np.asarray(labels)
    if arr.ndim != 2:
        raise GeometryError('labels 要是二维图，收到 {}'.format(arr.shape))
    blobs = []
    for label_id in np.unique(arr):
        if label_id == 0:
            continue
        vs, us = np.nonzero(arr == label_id)
        area = int(us.size)
        if area < int(min_area_px):
            continue
        blobs.append(Blob.from_points(np.column_stack((us, vs)).astype(float)))
    blobs.sort(key=lambda blob: blob.area_px, reverse=True)
    return blobs[:int(max_blobs)]


def blob_from_mask(mask, min_area_px=MIN_BLOB_AREA_PX) -> Blob:
    """单通道布尔掩膜 -> 一个 Blob（测试用；真实节点走 blobs_from_labels）。"""
    arr = np.asarray(mask)
    if arr.ndim != 2:
        raise GeometryError('mask 要是二维图，收到 {}'.format(arr.shape))
    vs, us = np.nonzero(arr)
    if us.size < int(min_area_px):
        raise GeometryError(
            '掩膜只有 {} 个像素，少于门限 {}'.format(us.size, min_area_px))
    return Blob.from_points(np.column_stack((us, vs)).astype(float))


def project_blob_to_plane(blob, camera, T_base_optical, plane_z,
                          max_points=2000) -> np.ndarray:
    """把一个 blob 的像素投影到 z = plane_z 平面上，返回 (N, 2) 的 base 坐标。

    为什么要在平面上拟合朝向，而不是在图像里拟合：相机不是正对着平面时，
    图像里的角度和平面上差一个视角引起的畸变（透视会把矩形压成梯形）。
    在平面上拟合就与相机姿态无关了。

    投不上去的像素（射线朝上、到不了平面）直接丢掉；丢得太多就抛异常，
    因为那说明相机装反了或者平面高度填错了。
    """
    pts = np.asarray(blob.points, float)
    if pts.shape[0] > max_points:
        stride = int(np.ceil(pts.shape[0] / float(max_points)))
        pts = pts[::stride]
    kept = []
    for u, v in pts:
        try:
            hit = backproject_pixel_to_plane(camera, u, v, T_base_optical, plane_z)
        except GeometryError:
            continue
        kept.append((hit[0], hit[1]))
    if len(kept) < MIN_PROJECTED_POINTS:
        raise GeometryError(
            'blob 的 {} 个像素里只有 {} 个能投到 z={} 平面上：'
            '检查相机是不是在平面下方、或者 table_z 填错了'.format(
                pts.shape[0], len(kept), plane_z))
    return np.asarray(kept, float)


def _rotated_extent(xy, yaw_rad):
    """把点绕原点转 -yaw 之后的外接矩形：(长, 宽)。"""
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    a = xy[:, 0] * c + xy[:, 1] * s
    b = -xy[:, 0] * s + xy[:, 1] * c
    return float(a.max() - a.min()), float(b.max() - b.min())


def fit_edge_yaw(xy, coarse_step_deg=COARSE_STEP_DEG,
                 fine_step_deg=FINE_STEP_DEG,
                 fine_window_deg=FINE_WINDOW_DEG) -> EdgeYaw:
    """拟合平面内那个矩形的朝向和外接尺寸（最小面积外接矩形）。

    正方形在 [0, pi/2) 里就够：转 90 度还是同一个正方形，两指也从另一条边
    夹进去，抓出来的姿态等价。

    不用二阶矩（`atan2(2*mu11, mu20-mu02)`）是因为正方形在二阶矩下是
    各向同性的 —— mu20 == mu02 且 mu11 == 0，朝向在数学上就没有定义。
    最小外接矩形没有这个问题。
    """
    pts = np.asarray(xy, float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        raise GeometryError('点要是 (N, 2)，收到 {}'.format(pts.shape))
    if pts.shape[0] < MIN_PROJECTED_POINTS:
        raise GeometryError(
            '只有 {} 个点，不足以拟合朝向（门限 {}）'.format(
                pts.shape[0], MIN_PROJECTED_POINTS))
    centred = pts - pts.mean(axis=0)

    def best_in(lo, hi, step):
        best_yaw, best_area = lo, None
        yaw = lo
        while yaw <= hi:
            a, b = _rotated_extent(centred, yaw)
            area = a * b
            if best_area is None or area < best_area:
                best_yaw, best_area = yaw, area
            yaw += step
        return best_yaw

    quarter = np.pi / 2.0
    coarse = best_in(0.0, quarter, np.radians(coarse_step_deg))
    window = np.radians(fine_window_deg)
    # 细化时允许越过边界，最后再折回 [0, pi/2)
    fine = best_in(coarse - window, coarse + window, np.radians(fine_step_deg))
    yaw = float(np.mod(fine, quarter))
    a, b = _rotated_extent(centred, yaw)
    return EdgeYaw(yaw_rad=yaw, extent_a_m=a, extent_b_m=b)


def shadow_bias(centre_xy, camera_xyz, plane_z, height_m):
    """斜视看一个立在工作面上的物体时，"影子"质心离相机比真实底面中心更远。

    物体高 h、视线与平面的夹角 θ：把它沿视线投影到平面上，得到的是"底面轮廓
    ⊕ 一段长 h·cotθ 的平移"，质心因此沿视线方向多走了 h·cotθ/2。24 mm 的方块、
    30° 的视线下约 20 mm，正是之前测到的 +18 mm 那个偏置。

    h 是这个物体的**高度**（不是宽度）；平贴在面上的标记物传 0，这时没有影子
    偏置，不该硬减。返回需要从影子质心减掉的二维向量（沿"背离相机"的方向）。
    """
    cam = np.asarray(camera_xyz, float)
    d = np.array([float(centre_xy[0]) - cam[0], float(centre_xy[1]) - cam[1], 0.0])
    horiz = float(np.hypot(d[0], d[1]))
    drop = float(cam[2] - plane_z)
    if horiz < 1e-6 or drop <= 1e-6 or not float(height_m) > 0.0:
        return np.zeros(2)
    shift = 0.5 * float(height_m) * horiz / drop
    return d[:2] / horiz * shift


def detection_from_blob(blob, camera, T_base_optical, plane_z, object_edge_m,
                        max_points=2000, object_height_m=None) -> SceneDetection:
    """blob -> 抓取目标：在平面上定质心、拟合朝向、算抓取点与顶抓姿态。

    `object_height_m` 只有算影子偏置时才用得上，默认等于边长（立方体）。

    朝向（yaw）只在"影子"不够长时才可信：影子被拉成 e × e(1+cotθ) 的矩形，
    当方块正好转过 45° 时，最小外接矩形会贴着视线方向而不是方块的边。夹爪还
    没通，这一条的影响等做抓取时再处理。
    """
    if not float(object_edge_m) > 0.0:
        raise GeometryError('方块边长必须为正：{}'.format(object_edge_m))
    height = float(object_edge_m) if object_height_m is None else float(object_height_m)
    xy = project_blob_to_plane(blob, camera, T_base_optical, plane_z,
                               max_points=max_points)
    if xy.shape[0] < MIN_PROJECTED_POINTS:
        raise GeometryError(
            '平面上只有 {} 个点，不足以定位'.format(xy.shape[0]))

    centre_xy = xy.mean(axis=0) - shadow_bias(
        xy.mean(axis=0), np.asarray(T_base_optical, float)[:3, 3], plane_z,
        height)
    fit = fit_edge_yaw(xy)
    # 抓取点取方块几何中心：底面贴在工作面上，所以比平面高半个边长。
    position = object_center_on_table(centre_xy[0], centre_xy[1], plane_z,
                                      object_edge_m)
    quaternion = top_down_grasp_quaternion(fit.yaw_rad)

    # 相机到这个点的距离：把点变回光学系，取 z。
    T_optical_base = invert_transform(T_base_optical)
    distance = float((T_optical_base @ np.append(position, 1.0))[2])
    if not distance > 0.0:
        raise GeometryError(
            '抓取点在相机背后（光学系 z = {:.4f}）'.format(distance))

    # 只看短边，理由同 depth.detection_from_points：斜向下看立在平面上的方块，
    # 轮廓沿视线方向必然被拉长，长边不可信，短边才是真实边长。
    edge_error = abs(min(fit.extent_a_m, fit.extent_b_m) - object_edge_m)
    return SceneDetection(position=np.asarray(position, float),
                          yaw_rad=fit.yaw_rad,
                          quaternion=quaternion,
                          extent_m=(fit.extent_a_m, fit.extent_b_m),
                          edge_error_m=float(edge_error),
                          distance_m=distance,
                          metres_per_pixel=camera.metres_per_pixel(distance))


def detections_from_blobs(blobs, camera, T_base_optical, plane_z,
                          object_edge_m, max_edge_error_m=0.01,
                          rejects=None, object_height_m=None) -> List[SceneDetection]:
    """批量版本：尺寸对不上的直接丢掉（多半是反光、阴影、或者别的物体）。

    `rejects` 不是 None 时，被丢掉的 blob 会以 `(blob, 原因)` 追加进去。理由：
    现场跑一轮很贵，"没检测到"如果只报一个 blob 数量，就只能靠猜是分割没切出来、
    反投影没落到平面上、还是尺寸对不上 —— 这三件事的修法完全不同。
    """
    out = []
    for blob in blobs:
        try:
            det = detection_from_blob(blob, camera, T_base_optical, plane_z,
                                      object_edge_m,
                                      object_height_m=object_height_m)
        except GeometryError as exc:
            if rejects is not None:
                rejects.append((blob, str(exc)))
            continue
        if det.edge_error_m > float(max_edge_error_m):
            if rejects is not None:
                rejects.append((blob, '拟合出的边长 {:.1f}x{:.1f} mm，与目标 '
                                      '{:.1f} mm 差 {:.1f} mm（上限 {:.1f}）'.format(
                    det.extent_m[0] * 1e3, det.extent_m[1] * 1e3,
                    object_edge_m * 1e3, det.edge_error_m * 1e3,
                    max_edge_error_m * 1e3)))
            continue
        out.append(det)
    return out
