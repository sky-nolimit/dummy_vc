#!/usr/bin/env python3
"""彩色方块 -> base_link 下的抓取目标（vision_msgs/Detection3DArray）。

这一层只做"脏活"：拿图、查 TF、调 cv2 分割，然后交给
`dummy_vc_perception/detection.py` 做几何。分割以外的数学都在那一层，
所以那边能离线自测，这边只能上真机验。

现在默认对着 driver 里那个 USB 相机（`dummy_fingerprint.launch.py`），
深度相机（Orbbec Gemini 335L）接进来时：

    image_topic        /camera/color/image_raw
    camera_info_topic  /camera/color/camera_info
    frame_id           /camera/color/image_raw 的 header 自己带 optical frame

有两条算抓取点的路，`use_depth` 选：

* **深度（默认）**：掩膜里的像素去查深度图，直接得到 3D 点，在顶面上定
  xy、z 减半个边长。不依赖"相机水平、地面等高处"这两个假设，也就没有
  单目那条路对俯角的病态敏感（本机斜下 33°，1° 就能让落点差 8 mm）。
* **单目（`use_depth:=false`）**：把像素投到"工作面"平面上（`table_z`，
  本机是地面 -0.0405），物体有高度时顶面中心会往外偏十几毫米。留着它
  是为了深度不可用（没接相机、深度流坏了）时还能跑，不是首选。

`calibrate_plane:=true` 会每隔一阵用整幅深度图拟合一次地面，把"相机实际
倾了多少度、离地多高"打出来 —— 那是标定 `camera_mount_pitch_deg` 的依据。
"""

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Vector3
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray

from dummy_vc_perception import detection, geometry
from dummy_vc_perception import depth as depth3d


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        scene = geometry.project_scene_measurements()

        self.declare_parameter('image_topic', '/dummy/visual/fingerprint_camera')
        self.declare_parameter('camera_info_topic',
                               '/dummy/visual/fingerprint_camera/camera_info')
        self.declare_parameter('output_topic', '/detected_objects')
        self.declare_parameter('marker_topic', '/detected_objects/markers')
        self.declare_parameter('base_frame', 'base_link')
        # 下面两个数的默认值来自本机的实测记录（geometry.project_scene_measurements）。
        # 用 yaml 覆盖也可以，但两者不一致时日志会指出来。
        self.declare_parameter('table_z', scene.table_z.value)
        self.declare_parameter('object_edge', scene.object_edge.value)
        # 彩色方块的 HSV 门限，一条 6 个数：[H_lo, S_lo, V_lo, H_hi, S_hi, V_hi]，
        # 多条就顺着往后接（节点按 6 个数一组切）。用扁平数组而不是
        # [[...], [...]]，是因为 ROS 2 的参数不支持"数组套数组"。
        # 给成多条而不是一对上下界，是因为红色在 OpenCV 的 H 上跨界
        # （0~10 与 170~180 两头都有），只写一条会抓不全半个方块。
        # 默认是绿色块；红色块写成两条：
        #   hsv_ranges: [0, 120, 70, 10, 255, 255, 170, 120, 70, 180, 255, 255]
        self.declare_parameter('hsv_ranges', [40, 80, 60, 80, 255, 255])
        self.declare_parameter('min_area_px', detection.MIN_BLOB_AREA_PX)
        self.declare_parameter('max_area_fraction', 0.2)
        # 短边是拿"边长"和"对角线"两个假设比过的（见 detection.short_edge_error），
        # 所以门限比早先的 10 mm 收紧到 6 mm：宽出来的那部分是给转向的，
        # 不是给"不像方块的东西"的。
        self.declare_parameter('max_edge_error', 0.006)
        self.declare_parameter('max_distance', 1.5)
        self.declare_parameter('max_rate_hz', 10.0)
        self.declare_parameter('publish_markers', True)
        # 深度路径。关掉就退回"像素投到平面上"的单目路。
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        # 0 表示按图像编码自动定（16UC1 -> 毫米，32FC1 -> 米）。
        self.declare_parameter('depth_scale', 0.0)
        # 掩膜抽稀步长：24 mm 方块在 0.3 m 处有上千个像素，隔一个取一个
        # 已经把定位钉到毫米级，全用只是让 2 Hz 的节点更卡。
        self.declare_parameter('depth_step', 2)
        self.declare_parameter('depth_max_range', depth3d.DEFAULT_MAX_RANGE_M)
        # 打开后每隔一阵用整幅深度图拟合地面，报相机真实的倾角和离地高度。
        self.declare_parameter('calibrate_plane', False)

        self.base_frame = self.get_parameter('base_frame').value
        self.table_z = float(self.get_parameter('table_z').value)
        self.object_edge = float(self.get_parameter('object_edge').value)
        self.hsv_ranges = self._parse_ranges(
            self.get_parameter('hsv_ranges').value)
        self.min_area_px = int(self.get_parameter('min_area_px').value)
        self.max_area_fraction = float(self.get_parameter('max_area_fraction').value)
        self.max_edge_error = float(self.get_parameter('max_edge_error').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        self.min_period = 1.0 / max(float(self.get_parameter('max_rate_hz').value), 1e-3)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.use_depth = bool(self.get_parameter('use_depth').value)
        self.depth_scale_setting = float(self.get_parameter('depth_scale').value)
        self.depth_step = max(1, int(self.get_parameter('depth_step').value))
        self.depth_max_range = float(self.get_parameter('depth_max_range').value)
        self.calibrate_plane = bool(self.get_parameter('calibrate_plane').value)
        self.depth_scale = self.depth_scale_setting or None

        self._check_measured(scene)

        self.bridge = CvBridge()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera = None            # CameraInfo 到了才有
        self.last_stamp = None
        self.misses = 0               # 连续几次没找到目标，用来别刷屏
        self.hits = 0                 # 一共认出来几次，用来决定要不要打坐标
        self.tf_warned = False
        self.geometry_warned = False
        self.distortion_warned = False
        self.depth_warned = False
        self.depth_stale_warned = False
        self.depth_msg = None         # 最近一帧深度（与彩色对齐）
        self.depth_frame_count = 0
        self.depth_size = None         # 深度图的 (宽, 高)
        self.size_reported = False     # 深度/彩色尺寸只报一次
        self.no_info_warned = False    # 没 CameraInfo 只抱怨一次
        self.plane_reports = 0
        self.last_plane_z = None

        self.pub = self.create_publisher(
            Detection3DArray, self.get_parameter('output_topic').value, 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, 1)
        if self.use_depth:
            self.create_subscription(
                Image, self.get_parameter('depth_topic').value, self._on_depth, 1)

        self.get_logger().info(
            'listening on {} (info: {}), publishing {} in {}'.format(
                self.get_parameter('image_topic').value,
                self.get_parameter('camera_info_topic').value,
                self.get_parameter('output_topic').value, self.base_frame))
        if self.use_depth:
            self.get_logger().info(
                '深度路径：{}（步长 {}，编码为 16UC1 时按毫米算{}）'.format(
                    self.get_parameter('depth_topic').value, self.depth_step,
                    '，已按参数覆盖为 {} 米/像素值'.format(self.depth_scale_setting)
                    if self.depth_scale_setting else ''))
        else:
            self.get_logger().info(
                '单目路径：像素投到 z = {:.4f} m 的平面上（俯角每差 1° 落点差约 '
                '8 mm，标定没做完前别信它的绝对精度）'.format(self.table_z))
        self.get_logger().info(
            '工作面 z = {:+.4f} m，目标方块边长 {:.1f} mm，'
            '夹它需要 {:.1f} mm 开口（见 README 的实测记录）'.format(
                self.table_z, self.object_edge * 1000.0,
                (self.object_edge + 0.002) * 1000.0))
        self.get_logger().info('HSV 范围：{}'.format(self._ranges_text()))

    # ------------------------------------------------------------------
    # 参数与实测值的一致性
    # ------------------------------------------------------------------

    def _parse_ranges(self, value):
        """[H,S,V,H,S,V, ...] 扁平数组 -> [(np.uint8 lower, np.uint8 upper), ...]。"""
        flat = [int(v) for v in value]
        if not flat:
            raise ValueError('hsv_ranges 是空的，什么都分不出来')
        if len(flat) % 6:
            raise ValueError(
                'hsv_ranges 要有 6 的倍数个数（每条 [H_lo,S_lo,V_lo,H_hi,S_hi,V_hi]），'
                '现在是 {} 个：{}'.format(len(flat), flat))
        ranges = []
        for index in range(0, len(flat), 6):
            entry = flat[index:index + 6]
            lower = np.array(entry[:3], np.uint8)
            upper = np.array(entry[3:], np.uint8)
            if np.any(lower > upper):
                raise ValueError(
                    'hsv_ranges 第 {} 条的下界比上界大：{} > {}'.format(
                        index // 6, lower.tolist(), upper.tolist()))
            ranges.append((lower, upper))
        return ranges

    def _check_measured(self, scene):
        recorded = scene.as_dict()
        if scene.unmeasured():
            self.get_logger().warn(
                '场景里还有没实测的数：{}'.format(scene.describe_unmeasured()))
        for name, current in (('table_z', self.table_z),
                              ('object_edge', self.object_edge)):
            known = recorded[name]
            if not math.isclose(float(known.value), current, abs_tol=1e-9):
                self.get_logger().warn(
                    '参数 {} = {:.4f} 与代码里记录的实测值 {:.4f} 不一致 —— '
                    '要么这个 yaml 是给别的机器用的，要么实测记录该更新了'.format(
                        name, current, float(known.value)))

    def _ranges_text(self):
        return ', '.join(
            '{}~{}'.format(lower.tolist(), upper.tolist())
            for lower, upper in self.hsv_ranges)

    def _log_rejects(self, rejects):
        """把"为什么这些 blob 没被采用"打出来，最多三条，按面积大的优先。

        两种路径的 `rejects` 元素形状不同：单目路径是 `(Blob, 原因)`，
        深度路径是 `(label, 像素数, 原因)`。这里统一成"多少像素 + 为什么"。
        """
        if not rejects:
            self.get_logger().warn(
                '一个 blob 都没有 —— 分割没切出东西来，先看 HSV 范围对不对')
            return

        def area_of(item):
            return item[0].area_px if hasattr(item[0], 'area_px') else int(item[1])

        def describe(item):
            if hasattr(item[0], 'area_px'):
                blob, reason = item
                return '{} 像素，中心 ({:.0f}, {:.0f})，原因：{}'.format(
                    blob.area_px, blob.u, blob.v, reason)
            label, pixels, reason = item
            return '{} 像素（label {}），原因：{}'.format(pixels, label, reason)

        for item in sorted(rejects, key=area_of, reverse=True)[:3]:
            self.get_logger().warn('  被丢掉的 blob：{}'.format(describe(item)))

    def _log_blobs(self, blobs):
        """画面里到底有几块绿的、最大那几块在哪儿。

        认错对象时（比如抓到一个远处的绿东西、而方块根本没进掩膜），这是唯一
        能定案的证据：方块按几何该落在某个像素上，掩膜里有没有东西在那儿，一眼
        就知道是"分割没切到"还是"切到了但被后面的判据拒了"。
        """
        if not blobs:
            self.get_logger().warn(
                '  画面里一个绿斑都没有（HSV 范围 {}）'.format(self._ranges_text()))
            return
        top = sorted(blobs, key=lambda b: b.area_px, reverse=True)[:3]
        self.get_logger().info(
            '  画面里 {} 个绿斑，最大的几个：{}'.format(
                len(blobs), '、'.join(
                    '({:.0f}, {:.0f}) {} 像素'.format(b.u, b.v, b.area_px)
                    for b in top)))

    def _usable_depth(self, header):
        """最近一帧深度图（ndarray）或 None。None 时用单目路径。"""
        if not self.use_depth:
            return None
        if self.depth_msg is None:
            if not self.depth_warned:
                self.depth_warned = True
                self.get_logger().warn(
                    '还没收到深度图（{}），先退回单目路径'.format(
                        self.get_parameter('depth_topic').value))
            return None
        stamp = self.depth_msg.header.stamp
        age = (header.stamp.sec - stamp.sec) + \
              (header.stamp.nanosec - stamp.nanosec) * 1e-9
        if abs(age) > 5.0 and not self.depth_stale_warned:
            self.depth_stale_warned = True
            self.get_logger().warn(
                '深度图和彩色图的时间戳差了 {:.1f} 秒，可能是两路没同步；'
                '静止场景下凑合能用，动起来就会错位'.format(age))
        try:
            frame = self.bridge.imgmsg_to_cv2(self.depth_msg,
                                              desired_encoding='passthrough')
        except Exception as exc:        # noqa: BLE001 - cv_bridge 抛什么的都有
            self.get_logger().error('深度图转换失败：{}'.format(exc))
            return None
        return np.asarray(frame)

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------

    def _on_camera_info(self, msg):
        try:
            self.camera = geometry.PinholeCamera.from_camera_info(
                msg.k, width=msg.width, height=msg.height, d=msg.d)
        except geometry.GeometryError as exc:
            self.get_logger().error('CameraInfo 用不了：{}'.format(exc))
            return
        if self.camera.fx <= 0.0 or self.camera.fy <= 0.0:
            self.get_logger().error(
                '内参 fx/fy = {}/{}：相机节点没填内参，投不了影'.format(
                    self.camera.fx, self.camera.fy))
            self.camera = None
            return
        # 8 个畸变系数的驱动（Gemini 335L 就是）只用到前 5 个，说一声。
        if self.camera.ignored_distortion and not self.distortion_warned:
            self.distortion_warned = True
            self.get_logger().warn(
                'CameraInfo.d 有 {} 个数，只用了前 5 个 (k1,k2,p1,p2,k3)，'
                '丢掉了 {} 个（k4,k5,k6）。如果那几项不是 0，畸变模型要换。'
                .format(len(msg.d), self.camera.ignored_distortion))
        self._check_depth_size()

    def _on_image(self, msg):
        if self.camera is None:
            # 没有内参就算不了坐标。但**不能安静地 return**：话题名打错一个字母
            # 时，节点会一条日志都不打，看起来像卡死（2026-10-19 真机就这么
            # 卡了一次：camera_info_topic 写成了 /camera/coolor/camera_info）。
            if not self.no_info_warned:
                self.no_info_warned = True
                self.get_logger().error(
                    '收到图像，但没收到过 CameraInfo（{}）—— 没有内参就算不了'
                    '坐标，每帧都直接丢掉。检查 camera_info_topic 有没有写错，'
                    '以及相机驱动是否发布了它'.format(
                        self.get_parameter('camera_info_topic').value))
            return
        if self.last_stamp is not None:
            elapsed = (msg.header.stamp.sec - self.last_stamp.sec) + \
                      (msg.header.stamp.nanosec - self.last_stamp.nanosec) * 1e-9
            if abs(elapsed) < self.min_period:
                return                  # 限流：丢帧比排队强
        self.last_stamp = msg.header.stamp

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:        # noqa: BLE001 - cv_bridge 抛什么的都有
            self.get_logger().error('图像转换失败：{}'.format(exc))
            return

        T = self._lookup(msg.header)
        if T is None:
            return

        labels, blobs = self._segment(frame)
        rejects = []
        depth_frame = self._usable_depth(msg.header)
        if depth_frame is not None:
            try:
                detections = depth3d.detections_from_labels_and_depth(
                    labels, depth_frame, self.camera, T, self.object_edge,
                    self.depth_scale, step=self.depth_step,
                    max_edge_error_m=self.max_edge_error, rejects=rejects)
            except geometry.GeometryError as exc:
                self.get_logger().error('深度路径算抓取点失败：{}'.format(exc))
                return
        else:
            try:
                detections = detection.detections_from_blobs(
                    blobs, self.camera, T, self.table_z, self.object_edge,
                    max_edge_error_m=self.max_edge_error, rejects=rejects)
            except geometry.GeometryError as exc:
                self.get_logger().error('算抓取点失败：{}'.format(exc))
                return
        detections = [d for d in detections if d.distance_m <= self.max_distance]

        if not detections:
            self.misses += 1
            if self.misses in (1, 50) or self.misses % 300 == 0:
                self.get_logger().warn(
                    '连续 {} 帧没找到合格的方块（HSV 范围 {}，blob {} 个）'.format(
                        self.misses, self._ranges_text(), len(blobs)))
                self._log_rejects(rejects)
                self._log_blobs(blobs)
            return
        if self.misses:
            self.get_logger().info('重新找到了目标（之前漏了 {} 帧）'.format(
                self.misses))
            self.misses = 0

        # 终端里就把坐标打出来：单目 0.3 Hz 的环境下 `topic echo` 很容易错过
        # 那一帧，日志是唯一一定能拿到的证据。
        self.hits += 1
        if self.hits == 1 or self.hits % 10 == 0:
            best = detections[0]
            self.get_logger().info(
                '第 {} 次命中：{} 个候选，取最大的那个 '
                'base_link=({:.3f}, {:.3f}, {:.3f}) m，yaw={:.1f}°，'
                '拟合边长 {:.1f}x{:.1f} mm，距离 {:.3f} m'.format(
                    self.hits, len(detections),
                    best.position[0], best.position[1], best.position[2],
                    math.degrees(best.yaw_rad),
                    best.extent_m[0] * 1e3, best.extent_m[1] * 1e3,
                    best.distance_m))
            if depth_frame is not None:
                self._log_depth_probe(labels, depth_frame, T)
            self._log_blobs(blobs)

        array = self._to_message(detections, msg.header)
        self.pub.publish(array)
        if self.publish_markers:
            self.pub_markers.publish(self._to_markers(detections, msg.header))

    # ------------------------------------------------------------------
    # 深度：取帧，以及"用地面把相机自己标出来"
    # ------------------------------------------------------------------

    def _log_depth_probe(self, labels, depth, T):
        """把"深度这一层实际看到什么"压成一行。

        只有最终坐标时，抓取点偏了分不清是掩膜偏、深度不对、还是 TF 的平移
        不对。这一行把原始深度值、以及这些点在 base_link 下的 z 摊开：和
        `table_z`（工作面）一比，就知道差在哪儿。
        """
        scale = self.depth_scale
        if not scale:
            return
        vs, us = np.nonzero(labels)
        if us.size == 0:
            return
        raw = depth3d.sample_depth(depth, us.astype(float), vs.astype(float),
                                   labels.shape)
        ok = np.isfinite(raw) & (raw > 0.0) & (raw * scale < self.depth_max_range)
        if not np.any(ok):
            self.get_logger().warn('深度诊断：掩膜里没有有效深度')
            return
        raw, uu, vv = raw[ok], us[ok].astype(float), vs[ok].astype(float)
        z = raw * scale
        x, y = depth3d.undistort_pixels(self.camera, uu, vv)
        p_opt = np.stack([x * z, y * z, z], axis=1)
        T = np.asarray(T, float)
        z_base = p_opt @ T[:3, :3].T[:, 2] + T[2, 3]
        plane = ('{:.4f}'.format(self.last_plane_z)
                 if self.last_plane_z is not None else '还没标定')
        self.get_logger().info(
            '深度诊断：掩膜 {} 像素，原始值中位数 {:.1f} mm（{:.1f}~{:.1f}），'
            'base_link 下 z 中位数 {:+.4f} m（{:+.4f}~{:+.4f}）；'
            '拟合地面过 z = {} m，参数 table_z = {:+.4f} m'.format(
                vv.size, float(np.median(raw)), float(np.min(raw)),
                float(np.max(raw)), float(np.median(z_base)),
                float(np.min(z_base)), float(np.max(z_base)),
                plane, self.table_z))

    def _check_depth_size(self):
        """深度图必须和彩色图同尺寸，也就是同一个像素坐标系。

        为什么要单独查：没开 depth_registration 时两者尺寸不同（Gemini 335L 的
        深度原生 848x480、彩色 1280x720），按彩色像素去查深度会取到完全无关的
        位置。这个错不会报错，只会让"几千像素的斑"拟合出几毫米的边长。
        """
        cam, size = self.camera, self.depth_size
        if cam is None or size is None or self.size_reported:
            return
        self.size_reported = True
        if (cam.width, cam.height) != size:
            self.get_logger().error(
                '深度图 {}x{} 与彩色图 {}x{} 尺寸不一致 —— 深度没有对齐到彩色，'
                '按彩色像素查深度是错的。请用 depth_registration:=true 重启相机'
                '（把深度对齐到彩色），或者改订阅已经对齐过的深度话题'
                .format(size[0], size[1], cam.width, cam.height))
        else:
            self.get_logger().info(
                '深度图 {}x{}，与彩色一致'.format(size[0], size[1]))

    def _on_depth(self, msg):
        if self.depth_scale is None:
            try:
                self.depth_scale = depth3d.depth_scale_for(msg.encoding)
            except geometry.GeometryError as exc:
                if not self.depth_warned:
                    self.depth_warned = True
                    self.get_logger().error('深度图用不了：{}'.format(exc))
                return
            self.get_logger().info(
                '深度编码 {}，按 {} 米/像素值换算'.format(
                    msg.encoding, self.depth_scale))
        self.depth_size = (int(msg.width), int(msg.height))
        self._check_depth_size()
        self.depth_msg = msg
        self.depth_frame_count += 1
        if self.calibrate_plane and self.depth_frame_count % 10 == 0:
            self._calibrate_from_plane(msg)

    def _calibrate_from_plane(self, msg):
        """整幅深度图拟合地面，反解相机真实的倾角和离地高度。

        为什么值得做：单目那条路在 33° 安装角下，俯角差 1° 抓取点就差 8 mm；
        手量角度到不了 1°，而地面在点云里占一片，拟合出来的法向精度在 0.1°
        量级。这是唯一能替代"手扫角度试出来"的办法。

        注意只标得动俯仰和高度：水平面绕竖直轴转多少度都还是它自己，
        偏航要靠已知位置的物体另做。
        """
        if self.camera is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as exc:        # noqa: BLE001
            self.get_logger().error('深度图转换失败：{}'.format(exc))
            return
        T = self._lookup(msg.header)
        if T is None:
            return
        arr = np.asarray(frame)
        height = int(self.camera.height or arr.shape[0])
        width = int(self.camera.width or arr.shape[1])
        # 按彩色图坐标抽样（深度已对齐到彩色），步长 8 已经够拟合一个平面。
        grid_v, grid_u = np.mgrid[0:height:8, 0:width:8]
        us = grid_u.ravel().astype(float)
        vs = grid_v.ravel().astype(float)
        z = depth3d.sample_depth(arr, us, vs, (height, width)) * self.depth_scale
        ok = np.isfinite(z) & (z > 0.05) & (z < self.depth_max_range)
        if np.count_nonzero(ok) < 50:
            self.get_logger().warn(
                '地面拟合：有效深度只有 {} 个点，跳过'.format(np.count_nonzero(ok)))
            return
        us, vs, z = us[ok], vs[ok], z[ok]
        x, y = depth3d.undistort_pixels(self.camera, us, vs)
        p_opt = np.stack([x * z, y * z, z], axis=1)
        try:
            normal_opt, offset = depth3d.plane_from_points(
                p_opt, T_base_optical=T, max_tilt_deg=25.0)
        except geometry.GeometryError as exc:
            self.get_logger().warn('地面拟合失败：{}'.format(exc))
            return
        report = depth3d.plane_report(T[:3, :3] @ normal_opt, offset)
        inliers, total, rms = depth3d.plane_quality(p_opt, normal_opt, offset)
        self.plane_reports += 1
        nx, ny, nz = report['normal']
        # 平面在 base_link 里过 z = plane_z（在 z 轴上截距）。这个数和 table_z
        # 一比，就知道差的是"相机装高了"还是"工作面不在 z=0 上"——两者的症状
        # 都是抓取点 z 偏，只有这个数能分开。
        normal_base = np.asarray(report['normal'], float)
        point_on_plane = (T[:3, :3] @ (normal_opt * offset)) + T[:3, 3]
        if abs(normal_base[2]) > 1e-6:
            self.last_plane_z = float(normal_base @ point_on_plane
                                      / normal_base[2])
        else:
            self.last_plane_z = None
        text = (
            '地面拟合：法向在 {} 里 = ({:+.3f}, {:+.3f}, {:+.3f})，'
            '相对竖直偏 {:.2f}°；相机到地面的距离 {:.3f} m（垂直距离，'
            '不是沿光轴的斜距；xacro 里相机 z = {:.3f} m）；'
            '面内点 {}/{}，残差 RMS {:.1f} mm；地面在 {} 里过 z = {} m'
            '（参数 table_z = {:.4f} m）。'
            '要把法向压到 (0,0,1)：camera_mount_pitch_deg 加上 {:+.2f}°'.format(
                self.base_frame, nx, ny, nz, report['tilt_deg'],
                report['camera_height_m'], float(T[2, 3]),
                inliers, total, rms * 1e3, self.base_frame,
                ('{:.4f}'.format(self.last_plane_z)
                 if self.last_plane_z is not None else '测不出'),
                self.table_z, report['pitch_correction_deg']))
        # 这个数本身要能判"能不能用"：内点太少或残差到厘米级，就说明这堆点里
        # 混了机械臂/底座/方块，拟合出来的距离和角度都是被拉偏的，不能照着改。
        share = inliers / float(total) if total else 0.0
        if share >= 0.4 and rms < 0.008:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(
                text + '。但内点只占 {:.0f}%、残差 RMS {:.1f} mm —— 这堆点不共面，'
                '上面的距离和角度都别照着改，先把画面里的其他东西挪开'.format(
                    share * 100.0, rms * 1e3))

    # ------------------------------------------------------------------
    # 查 TF：相机光学系在 base_link 里的位姿
    # ------------------------------------------------------------------

    def _lookup(self, header):
        frame_id = header.frame_id
        if not frame_id:
            self.get_logger().error('图像的 frame_id 是空的，无法查 TF')
            return None
        T = self._lookup_once(frame_id, Time.from_msg(header.stamp))
        if T is None:
            # 相机节点的时钟和 TF 的时间戳不一定对得上（尤其刚起来时），
            # 退一步用最新的一帧。eye-to-hand 下这条链不随关节变化，
            # 用最新值在物理上是等价的，只是少了对"标定时序"的检验。
            T = self._lookup_once(frame_id, Time())
        if T is None and not self.tf_warned:
            self.tf_warned = True
            self.get_logger().error(
                '查不到 {} -> {} 的 TF。检查：①相机节点的 camera_frame_id 是不是 '
                'URDF 里有的坐标系（要用 optical frame）；②robot_state_publisher '
                '起来没有；③手眼标定结果写进 xacro 没有'.format(
                    self.base_frame, frame_id))
        return T

    def _lookup_once(self, frame_id, stamp):
        try:
            tf = self.buffer.lookup_transform(
                self.base_frame, frame_id, stamp, Duration(seconds=0.2))
        except Exception:               # noqa: BLE001 - tf2 的异常类型不稳定
            return None
        q = tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = geometry.matrix_from_quaternion((q.x, q.y, q.z, q.w))
        T[:3, 3] = (tf.transform.translation.x, tf.transform.translation.y,
                    tf.transform.translation.z)
        self._warn_camera_geometry(T)
        return T

    def _warn_camera_geometry(self, T):
        """相机装歪/装反了要立刻说，否则只会表现为"抓取点乱飞"。"""
        if self.geometry_warned:
            return
        axis = T[:3, 2]                 # 光轴在 base_link 里的方向
        height = T[2, 3]
        if axis[2] > -0.15:
            self.get_logger().warn(
                '相机光轴 ({:.2f}, {:.2f}, {:.2f}) 几乎是水平的，照不到工作面 —— '
                '顶抓要求相机在工件上方且向下倾斜（本机实装是斜下 33°，'
                '光轴竖直分量约 -0.55）。要么 xacro 里的 camera_link 姿态还是'
                '占位值，要么手眼标定还没做'.format(axis[0], axis[1], axis[2]))
        if height <= self.table_z + 0.05:
            self.get_logger().warn(
                '相机高度 {:.3f} m 不高于工作面 {:.3f} m'.format(
                    height, self.table_z))
        self.geometry_warned = True

    # ------------------------------------------------------------------
    # 分割
    # ------------------------------------------------------------------

    def _segment(self, frame):
        """BGR 图 -> (labels 图, [Blob])。纯几何在 detection.py 里，这里只碰 cv2。

        返回 labels 是为了深度路径：它要按"每个 blob 的像素"去查深度值，
        而 `Blob` 只带像素坐标、不带 label。
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = np.zeros(hsv.shape[:2], np.uint8)
        for lower, upper in self.hsv_ranges:
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, lower, upper))
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        limit = int(self.max_area_fraction * mask.size)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
        keep = np.zeros_like(labels)
        for label_id in range(1, count):
            area = int(stats[label_id, cv2.CC_STAT_AREA])
            if self.min_area_px <= area <= limit:
                keep[labels == label_id] = label_id
        blobs = detection.blobs_from_labels(keep, min_area_px=self.min_area_px)
        return keep, blobs

    # ------------------------------------------------------------------
    # 发消息
    # ------------------------------------------------------------------

    def _to_message(self, detections, header):
        array = Detection3DArray()
        array.header = header
        array.header.frame_id = self.base_frame
        for index, det in enumerate(detections):
            item = Detection3D()
            item.header = array.header
            item.id = 'block_{}'.format(index)
            item.bbox.center = self._pose(det)
            item.bbox.size = Vector3(x=self.object_edge, y=self.object_edge,
                                     z=self.object_edge)
            hyp = ObjectHypothesisWithPose()
            # vision_msgs 4.x（Humble）：ObjectHypothesis 的字段是 class_id/score，
            # 外面包一层 hypothesis；3.x 直接叫 id 的写法在这里不适用。
            hyp.hypothesis.class_id = 'block'
            hyp.hypothesis.score = float(max(0.0, 1.0 - det.edge_error_m / 0.02))
            hyp.pose.pose = item.bbox.center
            item.results = [hyp]
            array.detections.append(item)
        return array

    def _pose(self, det):
        pose = Pose()
        pose.position = Point(x=float(det.position[0]), y=float(det.position[1]),
                              z=float(det.position[2]))
        qx, qy, qz, qw = det.quaternion
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def _to_markers(self, detections, header):
        array = MarkerArray()
        for index, det in enumerate(detections):
            cube = Marker()
            cube.header = header
            cube.header.frame_id = self.base_frame
            cube.ns = 'grasp_target'
            cube.id = index
            cube.type = Marker.CUBE
            cube.action = Marker.ADD
            cube.pose = self._pose(det)
            cube.scale = Vector3(x=self.object_edge, y=self.object_edge,
                                 z=self.object_edge)
            cube.color.r = 0.1
            cube.color.g = 0.9
            cube.color.b = 0.2
            cube.color.a = 0.6
            array.markers.append(cube)

            axis = Marker()             # 两指开合方向：规划时看它有没有贴住边
            axis.header = cube.header
            axis.ns = 'grasp_axis'
            axis.id = index
            axis.type = Marker.ARROW
            axis.action = Marker.ADD
            axis.pose = self._pose(det)
            axis.scale = Vector3(x=0.04, y=0.004, z=0.004)
            axis.color.r = 1.0
            axis.color.g = 0.5
            axis.color.b = 0.0
            axis.color.a = 1.0
            array.markers.append(axis)
        return array


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
