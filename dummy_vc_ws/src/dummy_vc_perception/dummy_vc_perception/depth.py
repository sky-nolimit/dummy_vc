"""用深度图算 3D 点，绕开"把像素投影到 z = 平面"那条路。纯 numpy，可离线自测。

为什么要另起一层，而不是继续用 `detection.project_blob_to_plane()`：

1. 那套做法假设相机水平放置、地面等高处。相机一旦是斜着向下看的（本机是
   斜下 16°），地面落点对俯角极其敏感，而且把"立体方块的轮廓"当成"地面上的
   平面图形"，会沿视线方向拉长几十毫米。深度直接给 3D 点，这两个问题一起消失。
2. 它还能顺带把相机自己标出来：地面在点云里占了一大片，拟合一个平面就能
   反解出相机到底倾了多少度、离地多高 —— 比手量和目测准两个数量级。

坐标系：深度图已经与彩色对齐（driver 的 depth_registration），所以彩色图上的
像素 (u, v) 直接拿去索深度图；两张图分辨率不同时按比例缩放索引。输出的点一律
在 base_link 下、单位米。
"""

import numpy as np

from .detection import (MIN_PROJECTED_POINTS, SceneDetection, fit_edge_yaw)
from .geometry import GeometryError, invert_transform, top_down_grasp_quaternion

# 深度图编码 -> 一个像素值是多少米。16UC1 是毫米（Orbbec / RealSense 的常见
# 格式），32FC1 直接就是米。认不出的编码宁可报错，也不要默默按毫米算 ——
# 那样会得到小 1000 倍的距离，然后表现为"抓取点贴在相机上"。
DEPTH_SCALES = {
    '16UC1': 0.001,
    'mono16': 0.001,
    '32FC1': 1.0,
    '64FC1': 1.0,
}

# 比这个近/远的深度值当噪声丢掉。0 表示"没有回波"，必须丢。
DEFAULT_MIN_RANGE_M = 0.10
DEFAULT_MAX_RANGE_M = 1.50

# 顶面切片：取 z 最高的那一层，厚度是边长的这个比例。
# 为什么要切片而不是用全部点：立方体的正面朝相机、顶面朝上，全部点的质心会
# 被正面往相机方向拉偏（仿真里约 18 mm）。顶面是水平正方形、中心就是立方体
# 的轴上，所以只在顶面上取 xy 才不带这个偏置。
TOP_SLICE_FRACTION = 0.3

# 平面拟合：先最小二乘，再按这个容差剔掉离群点重来。
PLANE_TOLERANCE_M = 0.02
PLANE_ITERATIONS = 4


def depth_scale_for(encoding):
    """图像编码 -> 米/像素值。不认识的编码抛异常（不猜）。"""
    scale = DEPTH_SCALES.get(str(encoding))
    if scale is None:
        raise GeometryError(
            '不认识的深度编码 "{}"，能认的是 {}'.format(
                encoding, ', '.join(sorted(DEPTH_SCALES))))
    return scale


def undistort_pixels(camera, us, vs, iterations=20, tolerance=1e-12):
    """像素 -> 去畸变后的归一化坐标，向量版（单点版在 geometry 里）。

    向量化不是为了好看：一帧掩膜动辄上万个像素，逐点调
    `camera.undistort_pixel()` 会把一个 2 Hz 的节点拖成卡死。
    """
    us = np.asarray(us, float)
    vs = np.asarray(vs, float)
    mx = (us - camera.cx) / camera.fx
    my = (vs - camera.cy) / camera.fy
    x, y = mx.copy(), my.copy()
    k1, k2, p1, p2, k3 = camera.distortion
    for _ in range(iterations):
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        ex, ey = xd - mx, yd - my
        x -= ex
        y -= ey
        if np.max(np.abs(ex)) < tolerance and np.max(np.abs(ey)) < tolerance:
            break
    return x, y


def sample_depth(depth, us, vs, image_shape):
    """按彩色图坐标取深度值。深度图分辨率不同时按比例缩放索引。"""
    arr = np.asarray(depth)
    if arr.ndim != 2:
        raise GeometryError('深度图要是二维的，收到 {}'.format(arr.shape))
    height, width = image_shape[:2]
    depth_h, depth_w = arr.shape
    ud = np.round(np.asarray(us, float) * (depth_w / float(width))).astype(int)
    vd = np.round(np.asarray(vs, float) * (depth_h / float(height))).astype(int)
    np.clip(ud, 0, depth_w - 1, out=ud)
    np.clip(vd, 0, depth_h - 1, out=vd)
    return arr[vd, ud].astype(float)


def points_from_labels(labels, depth, camera, T_base_optical, scale,
                       step=2, min_range=DEFAULT_MIN_RANGE_M,
                       max_range=DEFAULT_MAX_RANGE_M):
    """labels 图 + 深度图 -> (base_link 下的点 (N,3), 每个点所属的 label (N,))。

    只取 label != 0 的像素（0 是背景）；`step` 是采样步长，掩膜很大时抽稀
    就够了 —— 24 mm 方块在 0.3 m 处有上千个像素，全用只是浪费 CPU。
    """
    lab = np.asarray(labels)
    if lab.ndim != 2:
        raise GeometryError('labels 要是二维图，收到 {}'.format(lab.shape))
    vs, us = np.nonzero(lab)
    if us.size == 0:
        return np.zeros((0, 3)), np.zeros(0, dtype=int)
    if step > 1:
        us, vs = us[::step], vs[::step]

    z = sample_depth(depth, us, vs, lab.shape) * float(scale)
    ok = np.isfinite(z) & (z >= float(min_range)) & (z <= float(max_range))
    if not np.any(ok):
        raise GeometryError(
            '掩膜里 {} 个像素没有一个有有效深度（{:.0f}~{:.0f} mm）。'
            '检查深度话题有没有数据、depth_scale 对不对（16UC1 是毫米）'.format(
                us.size, min_range * 1e3, max_range * 1e3))
    us, vs, z = us[ok], vs[ok], z[ok]

    x, y = undistort_pixels(camera, us, vs)
    p_opt = np.stack([x * z, y * z, z], axis=1)

    T = np.asarray(T_base_optical, float)
    if T.shape != (4, 4):
        raise GeometryError('变换要是 4x4，收到 {}'.format(T.shape))
    points = p_opt @ T[:3, :3].T + T[:3, 3]
    return points, lab[vs, us].astype(int)


def detection_from_points(points, object_edge_m, T_base_optical,
                          camera=None) -> SceneDetection:
    """一堆 base_link 下的 3D 点 -> 一个抓取目标（取顶面定 xy，z 减半个边长）。"""
    edge = float(object_edge_m)
    if not edge > 0.0:
        raise GeometryError('方块边长必须为正：{}'.format(object_edge_m))
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise GeometryError('点要是 (N, 3)，收到 {}'.format(pts.shape))
    if pts.shape[0] < MIN_PROJECTED_POINTS:
        raise GeometryError(
            '只有 {} 个 3D 点，不足以定位（门限 {}）'.format(
                pts.shape[0], MIN_PROJECTED_POINTS))

    # 先从整体里剔掉离群的（地面、背景混进掩膜时会有一小撮点跑很远）。
    centre = np.median(pts, axis=0)
    spread = np.linalg.norm(pts - centre, axis=1)
    median = float(np.median(spread))
    mad = float(np.median(np.abs(spread - median))) * 1.4826
    keep = spread <= median + 5.0 * max(mad, 1e-4)
    if np.count_nonzero(keep) >= MIN_PROJECTED_POINTS:
        pts = pts[keep]

    # 顶面：z 最高的那一层。抓取点是立方体几何中心，所以 z 再减半个边长。
    z_top = float(np.percentile(pts[:, 2], 95))
    top = pts[pts[:, 2] >= z_top - TOP_SLICE_FRACTION * edge]
    if top.shape[0] < MIN_PROJECTED_POINTS:
        top = pts
    z_surface = float(np.median(top[:, 2]))

    xy = top[:, :2]
    fit = fit_edge_yaw(xy)
    # 用 2%/98% 分位的中点定中心，而不是中位数：从斜上方看，顶面切片里会混进
    # 侧面的一整条竖边（那一列点的 x 恒为 -边长/2），中位数会被它拖偏几毫米。
    # 方块的顶面是中心对称的，分位中点在没有遮挡时正好是几何中心。
    lo = np.percentile(xy, 2.0, axis=0)
    hi = np.percentile(xy, 98.0, axis=0)
    centre_xy = (lo + hi) / 2.0
    position = np.array([centre_xy[0], centre_xy[1], z_surface - edge / 2.0])

    T_optical_base = invert_transform(np.asarray(T_base_optical, float))
    distance = float((T_optical_base @ np.append(position, 1.0))[2])
    if not distance > 0.0:
        raise GeometryError(
            '抓取点在相机背后（光学系 z = {:.4f}）'.format(distance))

    edge_error = max(abs(fit.extent_a_m - edge), abs(fit.extent_b_m - edge))
    per_pixel = camera.metres_per_pixel(distance) if camera is not None else (0.0, 0.0)
    return SceneDetection(position=position,
                          yaw_rad=fit.yaw_rad,
                          quaternion=top_down_grasp_quaternion(fit.yaw_rad),
                          extent_m=(fit.extent_a_m, fit.extent_b_m),
                          edge_error_m=float(edge_error),
                          distance_m=distance,
                          metres_per_pixel=per_pixel)


def detections_from_labels_and_depth(labels, depth, camera, T_base_optical,
                                     object_edge_m, scale,
                                     step=2, max_edge_error_m=0.01,
                                     rejects=None):
    """批量版：每个 label 一组点，逐组解析。`rejects` 用法同 detection 那边。"""
    labels_arr = np.asarray(labels)
    points, point_labels = points_from_labels(
        labels_arr, depth, camera, T_base_optical, scale, step=step)
    if points.shape[0] == 0:
        return []
    out = []
    for label_id in np.unique(point_labels):
        sel = points[point_labels == label_id]
        label_pixels = int(np.count_nonzero(labels_arr == label_id))
        try:
            det = detection_from_points(sel, object_edge_m, T_base_optical,
                                        camera=camera)
        except GeometryError as exc:
            if rejects is not None:
                rejects.append((label_id, label_pixels, str(exc)))
            continue
        if det.edge_error_m > float(max_edge_error_m):
            if rejects is not None:
                rejects.append((label_id, label_pixels,
                                '顶面拟合出 {:.1f}x{:.1f} mm，与目标 {:.1f} mm '
                                '差 {:.1f} mm（上限 {:.1f}）'.format(
                                    det.extent_m[0] * 1e3, det.extent_m[1] * 1e3,
                                    float(object_edge_m) * 1e3,
                                    det.edge_error_m * 1e3,
                                    max_edge_error_m * 1e3)))
            continue
        out.append(det)
    out.sort(key=lambda d: d.distance_m)
    return out


# ----------------------------------------------------------------------
# 平面拟合：拿深度图把相机自己标出来
# ----------------------------------------------------------------------

def plane_from_points(points, tolerance_m=PLANE_TOLERANCE_M,
                      iterations=PLANE_ITERATIONS):
    """最小二乘拟合平面 -> (单位法向 n, 偏移 d)，满足 n·p = d。

    法向的符号统一成"指向相机"：输入的点在光学系里时，相机在原点，于是
    d 会是负数、|d| 就是相机到平面的距离（= 相机离地高度）。
    """
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise GeometryError('点要是 (N, 3)，收到 {}'.format(pts.shape))
    if pts.shape[0] < 3:
        raise GeometryError('少于 3 个点拟合不出平面')
    normal, offset = None, None
    for _ in range(max(1, int(iterations))):
        centre = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - centre, full_matrices=False)
        normal = vt[-1]
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            raise GeometryError('点退化，拟合不出平面')
        normal = normal / norm
        offset = float(normal @ centre)
        dist = np.abs(pts @ normal - offset)
        keep = dist < float(tolerance_m)
        if keep.all() or np.count_nonzero(keep) < 3:
            break
        pts = pts[keep]
    if offset > 0.0:
        # 让法向朝相机：相机在原点，n·0 - d = -d 应为正 -> d < 0
        normal, offset = -normal, -offset
    return normal, offset


def plane_report(normal_base, offset_optical):
    """地面法向 -> 一串"相机到底装成什么样"的数，给标定用。

    normal_base 是把光学系的法向用 URDF 里**当前写的**变换转到 base_link 之后
    的结果。如果 URDF 写对了，它应该正好是 (0, 0, 1)；写错了就歪，歪多少就是
    要补多少。

    俯角补多少的推导：URDF 的 pitch 参数是以 R_y(pitch) 左乘到相机姿态上，
    所以"写大了"等价于把真实姿态再绕 base 的 y 轴转正那么多。地面法向在这种
    情况下会变成 (sin Δ, 0, cos Δ)，其中 Δ = 写的 - 真的。于是
    真值 = 写的 - asin(nx)，也就是下面 `pitch_correction_deg` 要加的那个数。

    偏航（绕竖直轴转）在这条路上**永远标不出来**：一个水平面绕它自己的法向
    转多少度都还是它自己。要标偏航得用已知位置的物体，另一回事。
    """
    n = np.asarray(normal_base, float)
    norm = float(np.linalg.norm(n))
    if norm < 1e-9:
        raise GeometryError('法向是零向量')
    n = n / norm
    tilt_deg = float(np.degrees(np.arccos(np.clip(n[2], -1.0, 1.0))))
    pitch_correction = float(-np.degrees(np.arcsin(np.clip(n[0], -1.0, 1.0))))
    roll_correction = float(np.degrees(np.arcsin(np.clip(n[1], -1.0, 1.0))))
    return {
        'normal': (float(n[0]), float(n[1]), float(n[2])),
        'tilt_deg': tilt_deg,
        'pitch_correction_deg': pitch_correction,
        'roll_correction_deg': roll_correction,
        'camera_height_m': float(abs(offset_optical)),
    }
