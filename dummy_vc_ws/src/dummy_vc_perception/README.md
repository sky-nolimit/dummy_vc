# dummy_vc_perception

把相机图像变成 `base_link` 下的抓取目标（`vision_msgs/Detection3DArray`）。

对应 `docs/01-视觉抓取方案.md` 的「阶段 4 视觉感知」与「阶段 6 相机接入」。

## 现在有什么

| 文件 | 作用 | 能不能离线自测 |
| --- | --- | --- |
| `dummy_vc_perception/geometry.py` | 相机模型（针孔 + plumb_bob 畸变）、去畸变、桌面反投影、顶抓位姿、测量值来源 | 能，纯 numpy |
| `dummy_vc_perception/gripper.py` | 开口宽度 ↔ `hand.set_angle` 的映射，未实测就拒绝下发 | 能 |
| `dummy_vc_perception/detection.py` | blob → 平面质心 → 边长/朝向 → `SceneDetection`（最小面积外接矩形，不用二阶矩） | 能 |
| `dummy_vc_perception/vision_node.py` | ROS 层：取图、查 TF、HSV 分割、发 `/detected_objects` + RViz 标记 | 不能（要 ROS） |
| `config/vision.yaml` | 节点参数（话题、工作面、边长、HSV 范围、合格判据） | — |

分层不是洁癖：这一层将来要在真机上调坐标轴约定，而真机调试一轮很贵。
把它写成纯 numpy，本机（没有 ROS）就能用
`python tools/selftest_vision_geometry.py` 把约定钉死；
`tools/selftest_vision_geometry.py` 里还有一条检查专门盯着这一层
**不许 import ROS / OpenCV**，免得以后有人顺手 import 进来
（`vision_node.py` 是唯一允许碰 cv2 / rclpy 的文件，它不在那道检查的名单里）。

## 跑起来（在 VM 上）

```bash
# 终端 1：驱动 + USB 相机（driver 会自己 configure/activate）
ros2 launch dummy_controller dummy_fingerprint.launch.py

# 终端 2：视觉节点
ros2 run dummy_vc_perception vision_node --ros-args \
  --params-file $(ros2 pkg prefix dummy_vc_perception)/share/dummy_vc_perception/config/vision.yaml

# 终端 3：看输出（先在相机前面举一块纯色方块）
ros2 topic echo /detected_objects --once
ros2 run rviz2 rviz2      # 加 MarkerArray -> /detected_objects/markers
```

成功判据（由弱到强）：

1. 节点日志有 `listening on ...` 和 `工作面 z = +0.0000 m ...`；
2. 举起方块时 `/detected_objects` 有东西，位置随方块移动而移动；
3. RViz 里绿色方块**贴在**真方块上、橙色箭头与方块一条边平行（45° 摆放时
   箭头也要转 45°，这是朝向有没有算对的判据）；
4. 拿尺子量：`position` 的 xy 与真值差 < 10 mm（单目只够到这个量级，
   见下面第 4 条），`z` 应恒等于 `object_edge / 2 = 12 mm`。

`/detected_objects` 里每个元素：`bbox.center` = `gripper_tcp` 的位姿
（位置 + 顶抓四元数），`bbox.size` = 方块边长，`results[0].hypothesis.class_id`
= `block`，`score` 随边长误差下降。它没有时间戳上的花招：`header` 直接抄、
只有 `frame_id` 改成 `base_link`。

## 坐标轴约定（改之前先读这段）

```
camera_color_optical_frame    x 右、y 下、z 前（朝景物）；**由 orbbec_camera 驱动
                              发布**（camera_link -> camera_color_frame -> 光学系）
usb_camera_optical_frame      裸 USB 相机的光学系，URDF 里 camera_link 的子帧
tool0 / gripper_tcp           +y 是接近方向（手腕 -> 两指夹持中心）
                              ±x 是两指开合方向
顶抓（top-down）              gripper_tcp 的 +y 朝下（base_link 的 -z）
                              两指开合方向躺在桌面内，绕竖直轴转 yaw
```

相机挂载（`base_link -> camera_link`）由 URDF 的 `camera_link_joint` 负责，
目前是用户口头给的粗略位置 `(-0.23, 0.30, 0.15)`、`rpy (0, 90°, 0)`（朝下）。
**`rpy` 必须是 pitch=+90°**：`camera_link` 的 +x 是镜头方向，光学系的 z 就是它，
所以把 +x 转到 -z（朝下）要绕 y 轴 +90°；写成绕 x 轴 -90° 镜头会保持水平朝 +x，
反投影全斜而且不报错。手眼标定后这几个数要换成标定结果。

`docs/01` 的阶段 4 里写的是「tool0 的 +y（两指接近方向）与桌面平行」，
这句话与同一句里的「顶抓」自相矛盾：顶抓时 +y 必须朝下，否则夹爪是横着
从侧面伸过去的。URDF 是权威（`link7 -> tool0 -> gripper_tcp` 两段 origin
都在 +y 上，`dummy2_finger_jointN` 的 axis 在 ±x），所以代码按「+y 朝下」
实现。现场若发现方向反了，改的是 `geometry.py` 里 `top_down_grasp_matrix`
一个地方，不是散落各处的矩阵。

## 还差什么（放真机上之前必须知道）

1. **`vision_node.py` 只在本机做过静态检查，没在 ROS 里跑过。**
   它订阅 `image_topic` 与 `camera_info_topic`（默认是 driver 里现成的 USB 相机），
   用 `tf2_ros` 查 `base_link -> <图像的 frame_id>`，把
   `detection.detections_from_blobs` 的结果发成 `Detection3DArray`。
   查 TF 时先用图像时间戳，查不到再退回最新一帧（eye-to-hand 下这条链不随
   关节变化，物理上等价，只是少了对标定时序的检验）。
   节点自己会检查三件事并打警告：参数里的 `table_z` / `object_edge` 与代码里
   记的实测值不一致、相机光轴不朝下、相机高度不高于工作面。

2. **相机的 `camera_frame_id` 与设备号已经改好**（`dummy_controller` 包里）：
   `fingerprinter_camera.yaml` 的 `camera_frame_id` 改成
   `usb_camera_optical_frame`（URDF 里 `camera_link` 的子帧，rpy -90 0 -90），
   `fingerprint_camera.py` 加了 `device_index` 参数（默认 6），打不开时日志会
   直接告诉你去 `ls /dev/video*` 改参数。

   为什么不写 `camera_color_optical_frame`：那个名字归 Orbbec 驱动所有，URDF
   再定义一份就会让同一个 frame 有两个父节点，TF 树两条边、数字静默变错。
   于是 URDF 只保留一个**不同名**的 `usb_camera_optical_frame` 给裸 USB 相机用。

   为什么不是 `camera_link`：图像是光学系约定（x 右、y 下、z 朝前），把 frame
   写成 `camera_link` 等于声称这些像素在机身坐标系里，针孔模型的 z 会指到侧面，
   而且**照样出数字**，不报错。这类错只能靠 RViz 里看标记歪没歪发现。

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
   **接进来时只需要改 `config/vision.yaml` 的两个话题名**（文件里有注释好的
   那两行），节点其余部分不用动；但那时应该把 `_on_image()` 里那次
   `detections_from_blobs`（平面反投影）
   换成"取掩膜内的深度中值"，单目那套假设就可以扔了。

6. **夹爪的映射是占位值，而且代码拒绝用它下发指令。**
   `gripper.unverified_placeholder_model().command_for_width()` 一定抛
   `UnverifiedGripperModel`。按 docs/02 的阶段 0.5 量完两个端点，
   用 `gripper.opening_endpoints_from_measurements(...)` 建一个实测模型。
   （夹爪本身还有 A2：`hand.*` 会卡死整机，所以这一步现在也做不了。）
