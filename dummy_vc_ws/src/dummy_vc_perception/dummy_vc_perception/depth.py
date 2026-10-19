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

import math

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

# 掩膜内的深度常常分成两堆：方块在近处、桌面或背景在远处（真机实测 0.321 与
# 0.478 两堆，取中位数就选错了）。按这个宽度分箱、从近往远找第一簇。
CLUSTER_BIN_M = 0.02
CLUSTER_MIN_SHARE = 0.02

# 平面拟合：RANSAC 找内点，再用内点做最小二乘；容差就是这个"内点"的宽度。
# 256 次采样对几万个点足够（大半画面不是地面时也能命中），固定种子保证可复现。
PLANE_TOLERANCE_M = 0.02
PLANE_ITERATIONS = 4
RANSAC_TRIALS = 256
RANSAC_SEED = 20261019


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
        # "没有一个有效深度"有两种完全不同的原因：这片像素根本没有深度
        # （双目没匹配上，值就是 0），或者有深度但全部落在量程外（离得太近）。
        # 分开数出来，才知道该去调相机还是去调量程。
        finite = np.isfinite(z) & (z > 0.0)
        zero = int(us.size - np.count_nonzero(finite))
        low = int(np.count_nonzero(finite & (z < float(min_range))))
        high = int(np.count_nonzero(finite & (z > float(max_range))))
        detail = ('没读到深度的 {} 个、近于 {:.0f} mm 的 {} 个、远于 {:.0f} mm 的 {} 个'
                  .format(zero, float(min_range) * 1e3, low,
                          float(max_range) * 1e3, high))
        if np.any(finite):
            detail += '；非零深度 {:.1f}~{:.1f} mm（中位数 {:.1f}）'.format(
                float(np.min(z[finite])) * 1e3, float(np.max(z[finite])) * 1e3,
                float(np.median(z[finite])) * 1e3)
        raise GeometryError(
            '掩膜里 {} 个像素没有一个落在 {:.0f}~{:.0f} mm 里：{}'.format(
                us.size, float(min_range) * 1e3, float(max_range) * 1e3, detail))
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

    # 判"是不是方块"只看**短边**。相机是斜向下看的，立方体的顶面切片里必然还
    # 带进一条侧面，那个方向会被拉长：实测同一个方块报出过 34.9x25.5、
    # 46.6x25.8 mm。短边才是真实边长（24 mm 附近），长边是斜视的代价。
    # 只看短边的代价：又长又窄的东西能混进来 —— 但抓取场景里没有。
    short_edge = min(fit.extent_a_m, fit.extent_b_m)
    edge_error = abs(short_edge - edge)
    per_pixel = camera.metres_per_pixel(distance) if camera is not None else (0.0, 0.0)
    return SceneDetection(position=position,
                          yaw_rad=fit.yaw_rad,
                          quaternion=top_down_grasp_quaternion(fit.yaw_rad),
                          extent_m=(fit.extent_a_m, fit.extent_b_m),
                          edge_error_m=float(edge_error),
                          distance_m=distance,
                          metres_per_pixel=per_pixel)


def near_depth_cluster(depths, bin_m=CLUSTER_BIN_M,
                       min_share=CLUSTER_MIN_SHARE):
    """挑"离相机最近、点又足够多"的那一簇深度。

    掩膜里混进背景时，同一个 label 的点会分成两堆：方块在近处、桌面或背景在
    远处（真机实测 0.321 和 0.478）。取中位数会把背景当成方块 —— 报出来的抓取
    点因此偏了 +0.15 m、z 偏了 -0.09 m。

    方块永远在它自己的颜色掩膜里最靠前，所以从近往远扫，第一簇够多的就是它。
    返回布尔掩膜；只有一簇或者挑不出来时全给 True（调用方还有"全部点"这条退路）。
    """
    d = np.asarray(depths, float)
    keep_all = np.ones(d.shape, bool)
    if d.size < 2 * MIN_PROJECTED_POINTS:
        return keep_all
    lo, hi = float(np.min(d)), float(np.max(d))
    if (hi - lo) < 2.0 * bin_m:
        return keep_all
    bins = int(np.ceil((hi - lo) / bin_m))
    hist, edges = np.histogram(d, bins=bins, range=(lo, hi + 1e-9))
    need = max(MIN_PROJECTED_POINTS, int(round(min_share * float(hist.max()))))
    for i, count in enumerate(hist):
        if count >= need:
            centre = 0.5 * (float(edges[i]) + float(edges[i + 1]))
            return np.abs(d - centre) <= bin_m
    return keep_all


def _depth_stats(depths):
    """一行字说清这堆深度长什么样（分位数比中位数更能看出"两堆"）。"""
    d = np.asarray(depths, float)
    if d.size == 0:
        return '没有点'
    q = np.percentile(d, [5.0, 50.0, 95.0])
    return '深度 {:.3f} m（5%/50%/95% = {:.3f}/{:.3f}/{:.3f}，{:.3f}~{:.3f}）'.format(
        float(np.median(d)), q[0], q[1], q[2], float(np.min(d)), float(np.max(d)))


def detections_from_labels_and_depth(labels, depth, camera, T_base_optical,
                                     object_edge_m, scale,
                                     step=2, max_edge_error_m=0.01,
                                     rejects=None):
    """批量版：每个 label 一组点，逐组解析。`rejects` 用法同 detection 那边。

    每组先试"近处那一簇"，过不了尺寸判据再退回"全部点"：斜视下混进背景时
    近处那簇才是方块，而纯粹噪声凑出来的近处小簇会被尺寸判据挡掉。
    """
    labels_arr = np.asarray(labels)
    points, point_labels = points_from_labels(
        labels_arr, depth, camera, T_base_optical, scale, step=step)
    if points.shape[0] == 0:
        return []
    out = []
    T_inv = None
    for label_id in np.unique(point_labels):
        sel = points[point_labels == label_id]
        label_pixels = int(np.count_nonzero(labels_arr == label_id))
        if T_inv is None:
            T_inv = invert_transform(T_base_optical)
        # 这组点在相机系里的深度，以及它们在 base_link 里占的 z 高度。拟合出来的
        # 边长小得离谱时（虚拟机实测过 8900 像素的方块报 6.8x1.9 mm），靠这两个
        # 数就能分清是"深度噪声大"还是"掩膜切到的不是方块"。
        depth_of = (sel @ T_inv[:3, :3].T + T_inv[:3, 3])[:, 2]
        stats = '这组 {} 点，{}，base z 跨度 {:.1f} mm'.format(
            sel.shape[0], _depth_stats(depth_of),
            float(np.max(sel[:, 2]) - np.min(sel[:, 2])) * 1e3)

        near = near_depth_cluster(depth_of)
        candidates = []
        if not np.all(near) and np.count_nonzero(near) >= MIN_PROJECTED_POINTS:
            near_stats = '近处那一簇 {} 点，{}'.format(
                int(np.count_nonzero(near)), _depth_stats(depth_of[near]))
            candidates.append((near_stats, sel[near]))
        candidates.append((stats, sel))

        det = None
        reasons = []
        for why, subset in candidates:
            try:
                trial = detection_from_points(subset, object_edge_m,
                                              T_base_optical, camera=camera)
            except GeometryError as exc:
                reasons.append('{}；{}'.format(exc, why))
                continue
            if trial.edge_error_m > float(max_edge_error_m):
                reasons.append(
                    '顶面拟合出 {:.1f}x{:.1f} mm，短边与目标 {:.1f} mm '
                    '差 {:.1f} mm（上限 {:.1f}）；{}'.format(
                        trial.extent_m[0] * 1e3, trial.extent_m[1] * 1e3,
                        float(object_edge_m) * 1e3, trial.edge_error_m * 1e3,
                        max_edge_error_m * 1e3, why))
                continue
            det = trial
            break
        if det is None:
            if rejects is not None:
                rejects.append((label_id, label_pixels, ' ／ '.join(reasons)))
            continue
        out.append(det)
    out.sort(key=lambda d: d.distance_m)
    return out


# ----------------------------------------------------------------------
# 平面拟合：拿深度图把相机自己标出来
# ----------------------------------------------------------------------

def plane_from_points(points, tolerance_m=PLANE_TOLERANCE_M,
                      iterations=PLANE_ITERATIONS, trials=RANSAC_TRIALS,
                      T_base_optical=None, max_tilt_deg=None):
    """RANSAC 找最大的那个平面，再用它的内点做最小二乘 -> (法向 n, 偏移 d)。

    法向的符号统一成"指向相机"：输入的点在光学系里时，相机在原点，于是
    d 会是负数、|d| 就是相机到平面的距离（= 相机离地高度）。

    为什么不是直接最小二乘：标定时是整幅采样，机械臂、底座、方块、线缆都在
    画面里，地面往往只占三成。整堆点做 SVD 会被非地面点整体拉偏，给出一个
    "看起来正常"的距离。2026-10-19 就是这么错的：内点 763/2500、RMS 11.2 mm、
    相机到地面 0.206 m（卷尺量的是 0.16），而法向同时偏了 2.5°。
    RANSAC 只用 3 个点定平面、数内点，能扛住大半画面是别的东西。

    `T_base_optical` + `max_tilt_deg` 可选：给了就只接受"法向在 base_link 里
    与竖直方向夹角不超过 max_tilt_deg"的候选。地面按重力必然是水平的，这条先验
    能直接排掉墙壁、显示器、纸箱 —— 2026-10-19 真机上一帧就锁到了 85.7° 的
    竖直面（1.34 m 外的东西），把"相机到地面"报成 1.336 m。
    """
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3:
        raise GeometryError('点要是 (N, 3)，收到 {}'.format(pts.shape))
    if pts.shape[0] < 3:
        raise GeometryError('少于 3 个点拟合不出平面')
    tol = float(tolerance_m)
    cos_limit = None
    rot = None
    if T_base_optical is not None and max_tilt_deg is not None:
        rot = np.asarray(T_base_optical, float)[:3, :3]
        cos_limit = float(math.cos(math.radians(max_tilt_deg)))

    # 固定种子：同一个点云每次跑出同一个结果，日志才可比、自测才能判红。
    rng = np.random.default_rng(RANSAC_SEED)
    picks = rng.integers(0, pts.shape[0], size=(max(1, int(trials)), 3))
    best_normal, best_offset, best_count = None, None, -1
    for triple in picks:
        p0 = pts[triple[0]]
        n = np.cross(pts[triple[1]] - p0, pts[triple[2]] - p0)
        norm = float(np.linalg.norm(n))
        if norm < 1e-12:
            continue                    # 三点共线，定不出平面
        n = n / norm
        if cos_limit is not None:
            # 先验：这条候选平面的法向在 base_link 里必须接近竖直。
            if abs(float((rot @ n)[2])) < cos_limit:
                continue
        d = float(n @ p0)
        count = int(np.count_nonzero(np.abs(pts @ n - d) < tol))
        if count > best_count:
            best_normal, best_offset, best_count = n, d, count
    if best_normal is None:
        raise GeometryError('点退化，拟合不出平面')

    # 精修：只用内点重算。容差一步步收紧（2 cm -> 1 cm -> 5 mm）：RANSAC 选出的
    # 平面常常会为了"多框进几个噪声点"而微微倾斜，收紧之后这些点自然掉出去，
    # 而真正的平面用任意一个子集拟合都得到同一个平面。
    # 注意**不能**拿"全容差下的内点数"当接受条件：地面上杂物的点恰好落在那个
    # 倾斜平面附近时，倾斜版的"内点数"反而更多，会把已经修正的平面又判出局。
    normal, offset = best_normal, best_offset
    for divisor in (1.0, 2.0, 4.0):
        rtol = tol / divisor
        for _ in range(max(1, int(iterations))):
            keep = np.abs(pts @ normal - offset) < rtol
            if np.count_nonzero(keep) < 3:
                break
            sub = pts[keep]
            centre = sub.mean(axis=0)
            _, _, vt = np.linalg.svd(sub - centre, full_matrices=False)
            n = vt[-1]
            norm = float(np.linalg.norm(n))
            if norm < 1e-9:
                break
            n = n / norm
            d = float(n @ centre)
            moved = abs(d - offset) > 1e-9 or float(n @ normal) < 1.0 - 1e-12
            normal, offset = n, d
            if not moved:
                break
    if offset > 0.0:
        # 让法向朝相机：相机在原点，n·0 - d = -d 应为正 -> d < 0
        normal, offset = -normal, -offset
    return normal, offset


def plane_quality(points, normal, offset, tolerance_m=PLANE_TOLERANCE_M):
    """平面拟合的可信度 -> (内点数, 总点数, 残差 RMS 米)。

    只有一个"离地多少米"的数字时，看不出它能不能信：点云是弯的、平面上混进
    了别的物体、或者深度值根本不是沿光轴的 z 距离，都可能给出一个看起来正常
    的数。残差 RMS 在毫米级 = 真的拟合到了一个平面（传感器噪声就那么大）；
    到厘米级 = 这堆点根本不共面，那个距离不能直接写进标定。

    这一条也是"深度值到底是 z 还是斜距"的判据：把斜距当 z 用，地面会被算成
    一个球心在相机的碗，残差会有几厘米，而中心附近的点照样很准。
    """
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 3:
        raise GeometryError('点要是 (N, 3) 且至少 3 个，收到 {}'.format(pts.shape))
    n = np.asarray(normal, float)
    n = n / float(np.linalg.norm(n))
    dist = np.abs(pts @ n - float(offset))
    keep = dist < float(tolerance_m)
    count = int(np.count_nonzero(keep))
    if count < 3:
        # 连三个内点都没有：这堆点根本不共面。这时候仍然给一个数（全体残差），
        # 好让日志能显示"偏了多少"，而不是一个 inf 让人无从判断。
        return count, int(pts.shape[0]), float(np.sqrt(np.mean(dist ** 2)))
    rms = float(np.sqrt(np.mean(dist[keep] ** 2)))
    return count, int(pts.shape[0]), rms


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
