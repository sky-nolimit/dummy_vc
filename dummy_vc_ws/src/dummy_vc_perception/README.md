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

3. **三个数**（`geometry.project_scene_measurements()` 里是这台机器的实测值，
   `default_scene_measurements()` 保持全占位，好让占位那条路径一直被测到）：

   | 名字 | 实测值 | 是什么 |
   | --- | --- | --- |
   | `table_z` | `0.0` m | **没有桌子**，方块就放在和底座同一平面上；`base_link` 的原点正是底座底面（`j1` 在 `z = 0.0805`，即 78 mm 底座的顶面），所以工作面就是 `z = 0` |
   | `tcp_offset_y` | `0.015` m | `link7` 端面到两指夹持中心约 15 mm（尺量，2026-10-16）。同时写进 `dummy2_urdf.xacro` 的 `tcp_offset_y`，**两处必须一致**，自测里有交叉检查 |
   | `object_edge` | `0.024` m | 方形方块，边长 24 mm |

   `tcp_offset_y = 0.015` 比 `dummy2_hand_joint` 的 0.052312 还小，所以
   `gripper_tcp` 落在 hand 坐标系原点**后方**（`y = -0.037312`）：实物两指是
   贴着腕部夹的，而 URDF 里那套手指几何来自原作者模型、与实物不符
   （docs/04 的 A1）。两个后果：**做碰撞感知规划前必须换成简化碰撞体**
   （拿着 52 mm 长的假手掌去规划，靠近工件时会误判碰撞），而且 TCP 内移了
   45 mm，可达范围比按原模型算的窄。

   24 mm 方块配 2 mm 总余量（两侧各 1 mm）= **26 mm 开口**需求；实机最大开口
   还没量过，`hand.*` 又不能用（A2），所以这个需求现在无法签收。


4. **单目兜底的精度上限。** 把像素投到桌面上，等价于假设物体是贴在桌面上的
   平面图形；物体有高度时顶面中心会被按 `z / (z - h)` 往外推
   （`geometry.monocular_planar_inflation`）：0.6 m 高、24 mm 的方块顶面偏
   4 mm 左右，40 mm 的方块偏 7.1 mm。所以 USB 单目只能把数据通路跑通，
   验收精度得等深度相机。

   另一个数是地面真值：本机这套相机内参下，0.6 m 处 **1 像素 = 1.212 mm**；
   不做去畸变时角落像素会偏 **3.22 px ≈ 3.9 mm** —— 都比 docs/01 里估的
   "1 mm/像素"大，验收线（10 mm）里没有多少余量。

5. **深度相机已到货：Orbbec Gemini 335L**（不是 docs/01 里写的 RealSense）。
   `vision_node.py` 的数据源要按它设计：ROS 2 包是 `orbbec_camera`，
   对齐后的深度图在 `/camera/depth/image_raw`（`depth_registration:=true`），
   彩色在 `/camera/color/image_raw`，点云在 `/camera/depth/points`，
   服务 `/camera/set_image_registration_mode` 可在运行时切 `OFF`/`HW_D2C`/`SW_D2C`。
   接进来之前先在 Windows 上用 Orbbec Viewer 确认相机本身是好的。

6. **夹爪的映射是占位值，而且代码拒绝用它下发指令。**
   `gripper.unverified_placeholder_model().command_for_width()` 一定抛
   `UnverifiedGripperModel`。按 docs/02 的阶段 0.5 量完两个端点，
   用 `gripper.opening_endpoints_from_measurements(...)` 建一个实测模型。
   （夹爪本身还有 A2：`hand.*` 会卡死整机，所以这一步现在也做不了。）
