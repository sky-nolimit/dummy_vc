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

单目这条路径把像素投到"工作面"平面上（`table_z`，本机没有桌子所以是 0），
物体有高度时顶面中心会往外偏一点 —— 这就是为什么单目只够把框架跑通。
接了深度之后，把 `_on_image()` 里那次
`detection.detections_from_blobs(...)` 换成"取掩膜内的深度中值"，其余不用动。
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
        self.declare_parameter('max_edge_error', 0.010)
        self.declare_parameter('max_distance', 1.5)
        self.declare_parameter('max_rate_hz', 10.0)
        self.declare_parameter('publish_markers', True)

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

        self.pub = self.create_publisher(
            Detection3DArray, self.get_parameter('output_topic').value, 10)
        self.pub_markers = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)
        self.create_subscription(
            CameraInfo, self.get_parameter('camera_info_topic').value,
            self._on_camera_info, 10)
        self.create_subscription(
            Image, self.get_parameter('image_topic').value, self._on_image, 1)

        self.get_logger().info(
            'listening on {} (info: {}), publishing {} in {}'.format(
                self.get_parameter('image_topic').value,
                self.get_parameter('camera_info_topic').value,
                self.get_parameter('output_topic').value, self.base_frame))
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
        """把"为什么这些 blob 没被采用"打出来，最多三条，按面积大的优先。"""
        if not rejects:
            self.get_logger().warn(
                '一个 blob 都没有 —— 分割没切出东西来，先看 HSV 范围对不对')
            return
        ordered = sorted(rejects, key=lambda item: item[0].area_px, reverse=True)
        for blob, reason in ordered[:3]:
            self.get_logger().warn(
                '  被丢掉的 blob：{} 像素，中心 ({:.0f}, {:.0f})，原因：{}'.format(
                    blob.area_px, blob.u, blob.v, reason))

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

    def _on_image(self, msg):
        if self.camera is None:
            return                      # 还没收到 CameraInfo
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

        blobs = self._segment(frame)
        rejects = []
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

        array = self._to_message(detections, msg.header)
        self.pub.publish(array)
        if self.publish_markers:
            self.pub_markers.publish(self._to_markers(detections, msg.header))

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
        if axis[2] > -0.3:
            self.get_logger().warn(
                '相机光轴 ({:.2f}, {:.2f}, {:.2f}) 几乎是水平的，照不到工作面 —— '
                '顶抓要求相机在工件上方且向下倾斜（本机实装是斜下 30°，'
                '光轴竖直分量约 -0.50）。要么 xacro 里的 camera_link 姿态还是'
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
        """BGR 图 -> [Blob]。纯几何在 detection.py 里，这里只碰 cv2。"""
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
        return blobs

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
