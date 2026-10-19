# dummy_vc_perception

把相机图像变成 `base_link` 下的抓取目标（`vision_msgs/Detection3DArray`）。

对应 `docs/01-视觉抓取方案.md` 的「阶段 4 视觉感知」与「阶段 6 相机接入」。

## 现在有什么

| 文件 | 作用 | 能不能离线自测 |
| --- | --- | --- |
| `dummy_vc_perception/geometry.py` | 相机模型（针孔 + plumb_bob 畸变）、去畸变、桌面反投影、顶抓位姿、测量值来源 | 能，纯 numpy |
| `dummy_vc_perception/gripper.py` | 开口宽度 ↔ `hand.set_angle` 的映射，未实测就拒绝下发 | 能 |
| `dummy_vc_perception/detection.py` | blob → 平面质心 → 边长/朝向 → `SceneDetection`（最小面积外接矩形，不用二阶矩）。**单目兜底用** | 能 |
| `dummy_vc_perception/depth.py` | 深度图 → 3D 点 → 抓取点；地面拟合反解相机倾角。**默认走这条** | 能 |
| `dummy_vc_perception/vision_node.py` | ROS 层：取彩色+深度、查 TF、HSV 分割、发 `/detected_objects` + RViz 标记 | 不能（要 ROS） |
| `config/vision.yaml` | 节点参数（话题、工作面、边长、HSV 范围、合格判据、深度开关） | — |

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

### 深度这条路（默认）与用地面自己标定俯角

节点默认 `use_depth: true`：彩色图照旧用来分割，位置改成"掩膜里的像素配上
深度图，反投影成 3D 点"。这样绕掉了单目那两个假设（物体贴在 `table_z` 平面上、
立体物体的地面投影等于它的边长）——后者在斜视下**数学上就不可能成立**，
拟合出的边长永远是沿视线拉长的（24 mm 方块在斜视下恒被量成 65 × 28.6 mm）。

深度图必须是**已经对齐到彩色**的（`depth_registration:=true`；判断方法是话题
列表里同时有 `/camera/depth/image_unaligned`）。没深度或深度太旧时节点会自己
退回单目那条路并打一次警告，所以拔了深度不会崩，只是精度回到毫米级偏厘米。

**相机到底装成多少度，别再手扫了**——让地面自己说：

```bash
ros2 run dummy_vc_perception vision_node --ros-args \
  --params-file $(ros2 pkg prefix dummy_vc_perception)/share/dummy_vc_perception/config/vision.yaml \
  -p use_depth:=true -p calibrate_plane:=true
```

日志会打出（每 10 帧一次）：

```
法向在 base_link 里 = (+0.012, +0.018, +1.000)，相对竖直偏 1.21°；相机到地面的距离 0.202 m（垂直距离，不是沿光轴的斜距）；面内点 12345/13000，残差 RMS 1.8 mm；地面在 base_link 里过 z = -0.0520 m（参数 table_z = -0.0405 m）；
要把法向压到 (0,0,1)：camera_mount_pitch_deg 加上 -0.68°
```

照它给的数改 `dummy2_urdf.xacro` 的 L474，重启 `robot_state_publisher` 就能收敛到
0.1° 量级——比手扫角度准两个数量级，而且不用再猜。**平面标不出偏航**：一个水平面
绕它自己的法向转多少度都还是它自己，偏航得用已知位置的物体另做。

### 没有机械臂也能验（只用相机）

`base_link -> camera_link` 是固定关节，由 `robot_state_publisher` 从 URDF 直接
发到 `/tf_static`，**跟机械臂在不在、使能没使能无关**。所以相机单独就能把整条
几何链跑通：

```bash
# 终端 1：只要 TF，不碰硬件（不需要驱动，手臂不会动）
xacro $(ros2 pkg prefix dummy2_urdf_description)/share/dummy2_urdf_description/urdf/dummy2_urdf.xacro > /tmp/dummy2.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args \
  -p robot_description:="$(cat /tmp/dummy2.urdf)"

# 终端 2：相机（Orbbec）
ros2 launch orbbec_camera gemini_330_series.launch.py

# 终端 3：核对挂载 —— translation 应该是 [-0.230, 0.300, 0.150]
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame

# 终端 4：视觉节点（Orbbec 的话题名）
ros2 run dummy_vc_perception vision_node --ros-args \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info
```

这条路径验的是「分割 + 投影 + 平面假设 + TF 链」，不验标定：`base_link` 的
位置是 URDF 里声称的，相机实际摆在哪儿它并不知道。所以数值对得上要对得上
的前提是**相机真的摆在那三个数上**；摆歪了数值就歪，那不算失败。

成功判据（由弱到强）：

1. 节点日志有 `listening on ...` 和 `工作面 z = -0.0405 m ...`；
2. 举起方块时 `/detected_objects` 有东西，位置随方块移动而移动；
3. RViz 里绿色方块**贴在**真方块上、橙色箭头与方块一条边平行（45° 摆放时
   箭头也要转 45°，这是朝向有没有算对的判据）；
4. 拿尺子量：`position` 的 xy 与真值差 < 10 mm（走深度这条路应该到几毫米，
   单目兜底只够到 10 mm，见下面第 4 条），`z` 应恒等于
   `object_edge / 2 = 12 mm`。

日志里每次命中（第 1 次和每 10 次）都会打出报出的坐标和拟合边长，被丢掉的
候选也会打原因——排查"为什么没认出来"靠这几行，不用再去 RViz 里猜。

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
位置是卷尺量的 `(-0.23, 0.30, 0.15)`，俯角 `pitch = 33.3°`。
**俯角是地面拟合量出来的，不是估的**：`calibrate_plane:=true` 时节点对整幅深度里
的地面做最小二乘，把法向转到 `base_link` 看它歪多少。xacro 里写 34° 那一次，
法向是 `(+0.012, +0.018, +1.000)`（相对竖直偏 1.21°，要补 -0.68°），所以真实
俯角 ≈ `34 - 0.68 = 33.3°`。这是几万个点的拟合，0.1° 量级。

（更早的一次"用方块单个像素反解出 16°"是**错的**：那个方法把高度当已知、又把
平面假设套在立体方块上，两个误差可以互相抵消成任何数，用户后来扫角度时也发现
"拟合边长两边一起变短、根本不收敛"，就是这个原因。）

**相机是斜下 33°，比原先目测的 30° 略陡**（光轴 `(0.836, 0, -0.549)`）。

不确定度：位置仍是卷尺（x/y 各 ±1 cm 量级），**高度最不确定** —— 卷尺换算过来
约 0.12 m，深度拟合换算过来约 0.16 m，差 4 cm，谁对还没定。手眼标定会给准值。

**`rpy` 是 pitch，不是 roll**：`camera_link` 的 +x 才是镜头方向（光学系的 z 就是
它），绕 y 轴转 pitch 才能把 +x 压向 -z：pitch=90° 是垂直朝下，pitch=33.3° 是斜下
33.3°。写成绕 x 轴转 roll 的话镜头只在 yz 平面里动；写成 `rpy(180°,0,0)` 镜头会
保持水平朝 +x —— 反投影全斜而且不报错。手眼标定后这几个数要换成标定结果。

水平方向的偏差（绕竖直轴）**地面拟合法永远标不出来**（水平面绕自己法向转多少度
都还是它自己），要用已知位置的方块反解。`yaw` 暂时留 0。

斜视的代价（已在自测里记成 note）：平面反投影在倾斜方向上误差被 `1/sin` 放大
（33° 时约 1.8 倍），画面远端更差；画面上方有相当一块是地平线以上，那些像素打在
无穷远、反投影会被丢掉，所以方块要放在画面中间、别贴着上边缘。
在方块那个距离（约 0.27 m）上 1 像素 ≈ 0.44 mm。

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
   | `table_z` | `-0.0405` m | 工作面是**地面**，不是底座底面。`base_link` 的原点在底座 stl 包围盒的中心，底面在 `z = -0.0155`；底座又垫在约 2.5 cm 高的地台上，所以地面在 `base_link` 里是 `-(0.0155 + 0.025)`。方块放在地面上（2026-10-19 用户确认），抓取点 `z = table_z + 半边长` |
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
   节点已经按它写好：彩色 `/camera/color/image_raw` 用来分割，深度
   `/camera/depth/image_raw`（`depth_registration:=true`，已对齐到彩色）用来定位，
   `use_depth: true` 是默认。**接进来只需要改 `config/vision.yaml` 的两个话题名**
   （文件里有注释好的那两行），节点其余部分不用动。
   深度只有 **2 Hz**（USB 2.0 + VirtualBox 的等时传输，见 docs/03），静止场景够用；
   真要提高得换 USB 3.0 线并把相机直通进虚拟机。
   单目那条路留着当兜底（`use_depth:=false`，或深度没数据时节点自己退回）。

6. **夹爪的映射是占位值，而且代码拒绝用它下发指令。**
   `gripper.unverified_placeholder_model().command_for_width()` 一定抛
   `UnverifiedGripperModel`。按 docs/02 的阶段 0.5 量完两个端点，
   用 `gripper.opening_endpoints_from_measurements(...)` 建一个实测模型。
   （夹爪本身还有 A2：`hand.*` 会卡死整机，所以这一步现在也做不了。）
