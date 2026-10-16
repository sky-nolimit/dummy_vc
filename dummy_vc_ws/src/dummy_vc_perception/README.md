# dummy_vc_perception

把相机图像变成 `base_link` 下的抓取目标（`vision_msgs/Detection3DArray`）。

对应 `docs/01-视觉抓取方案.md` 的「阶段 4 视觉感知」与「阶段 6 相机接入」。

## 现在有什么

| 文件 | 作用 | 能不能离线自测 |
| --- | --- | --- |
| `dummy_vc_perception/geometry.py` | 相机模型（针孔 + plumb_bob 畸变）、去畸变、桌面反投影、顶抓位姿、测量值来源 | 能，纯 numpy |
| `dummy_vc_perception/gripper.py` | 开口宽度 ↔ `hand.set_angle` 的映射，未实测就拒绝下发 | 能 |
| `dummy_vc_perception/vision_node.py` | 还没写 | 不能（要 ROS） |

分层不是洁癖：这一层将来要在真机上调坐标轴约定，而真机调试一轮很贵。
把它写成纯 numpy，本机（没有 ROS）就能用
`python tools/selftest_vision_geometry.py` 把约定钉死；
`tools/selftest_vision_geometry.py` 里还有一条检查专门盯着这一层
**不许 import ROS / OpenCV**，免得以后有人顺手 import 进来。

## 坐标轴约定（改之前先读这段）

```
camera_color_optical_frame    x 右、y 下、z 前（朝景物）
tool0 / gripper_tcp           +y 是接近方向（手腕 -> 两指夹持中心）
                              ±x 是两指开合方向
顶抓（top-down）              gripper_tcp 的 +y 朝下（base_link 的 -z）
                              两指开合方向躺在桌面内，绕竖直轴转 yaw
```

`docs/01` 的阶段 4 里写的是「tool0 的 +y（两指接近方向）与桌面平行」，
这句话与同一句里的「顶抓」自相矛盾：顶抓时 +y 必须朝下，否则夹爪是横着
从侧面伸过去的。URDF 是权威（`link7 -> tool0 -> gripper_tcp` 两段 origin
都在 +y 上，`dummy2_finger_jointN` 的 axis 在 ±x），所以代码按「+y 朝下」
实现。现场若发现方向反了，改的是 `geometry.py` 里 `top_down_grasp_matrix`
一个地方，不是散落各处的矩阵。

## 还差什么（放真机上之前必须知道）

1. **`vision_node.py` 还没写。** 它要订阅 `/dummy/visual/fingerprint_camera`
   与 `/dummy/visual/fingerprint_camera/camera_info`（driver 里现成的 USB 相机），
   用 `tf2_ros` 查 `base_link -> camera_color_optical_frame`，把
   `geometry.backproject_pixel_to_plane` 的结果发成 `Detection3DArray`。
   落地时 `package.xml` 要补 `vision_msgs` / `cv_bridge` / `tf2_ros` /
   `visualization_msgs`，`setup.py` 要补 `console_scripts` 那一行。

2. **相机的 `camera_frame_id` 现在是 `fingerprint_camera`，不在 URDF 里。**
   必须改成 `camera_link`（`dummy_controller/config/fingerprinter_camera.yaml`
   的 `camera_frame_id`），否则 TF 树是断的 —— 这是 docs/01 里点名的坑。
   另外 `fingerprint_camera.py` 里 `cv2.VideoCapture(6)` 的设备号是硬编码的，
   要参数化（docs/01 的 3.8）。

3. **三个数还没实测**（`geometry.default_scene_measurements()` 里都标着
   `source='placeholder'`，`SceneMeasurements.unmeasured()` 会把它们列出来）：

   | 名字 | 是什么 | 怎么量 |
   | --- | --- | --- |
   | `table_z` | `base_link` 原点所在平面到桌面的距离 | 尺子 |
   | `tcp_offset_y` | `link7` 端面到两指夹持中心的距离（= URDF 的 `gripper_tcp` 偏移，docs/04 的 A1） | 卡尺 |
   | `object_edge` | 演示方块的边长 | 卡尺 |

   `tcp_offset_y` 现在用的是 xacro 里的占位值 0.06 m，`table_z` 直接是 0.0 ——
   它们不影响离线自测，但会让真机上的抓取点整体偏掉。

4. **单目兜底的精度上限。** 把像素投到桌面上，等价于假设物体是贴在桌面上的
   平面图形；物体有高度时顶面中心会被按 `z / (z - h)` 往外推
   （`geometry.monocular_planar_inflation`）：0.6 m 高、50 mm 的方块偏 9%，
   十几毫米。所以 USB 单目只能把数据通路跑通，验收精度得等深度相机。

5. **夹爪的映射是占位值，而且代码拒绝用它下发指令。**
   `gripper.unverified_placeholder_model().command_for_width()` 一定抛
   `UnverifiedGripperModel`。按 docs/02 的阶段 0.5 量完两个端点，
   用 `gripper.opening_endpoints_from_measurements(...)` 建一个实测模型。
   （夹爪本身还有 A2：`hand.*` 会卡死整机，所以这一步现在也做不了。）
