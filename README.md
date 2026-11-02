# Dummy 机械臂视觉抓取

Dummy 六轴机械臂 + 舵机夹爪 + Orbbec 相机，抓桌面上的方块。

- 系统：Ubuntu 22.04，ROS 2 Humble
- 工作区：`dummy_vc_ws/`（唯一的 ROS 2 工作区）
- 硬件：Dummy 机械臂（USB 串口）、Orbbec Gemini 335L、USB-CAN 适配器（夹爪）

抓取成功的演示：[Dummy_vc.mp4](Dummy_vc.mp4)

## 目录

| 路径 | 内容 |
|---|---|
| `dummy_vc_ws/src/dummy2_urdf` | 机械臂模型（URDF/Xacro + STL） |
| `dummy_vc_ws/src/dummy_controller` | 硬件驱动，经 fibre 控臂 |
| `dummy_vc_ws/src/dummy_vc_moveit_config` | MoveIt 2 配置（运动规划） |
| `dummy_vc_ws/src/dummy_vc_perception` | 相机接入 + 视觉检测 |
| `dummy_vc_ws/src/dummy_vc_grasp` | 夹爪 CAN 节点、抓取规划、抓取状态机 |
| `dummy_vc_ws/src/dummy_vc_calibration` | 关节标定，只在换板子/换零位时用 |

## 构建

```bash
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

每个新终端都要先 `source install/setup.bash`。

## 跑一次抓取（三个终端）

**终端 1 —— 机械臂驱动**

```bash
ros2 launch dummy_controller dummy_controller.launch.py
```

**终端 2 —— MoveIt（接真机）**

```bash
ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py
```

**终端 3 —— 视觉 + 夹爪 + 抓取**

```bash
ros2 launch dummy_vc_grasp pick_place_demo.launch.py execute:=true gripper:=true auto_start:=true
```

流程：等相机 → 检测方块 → 算抓取点 → 规划 → 下降合爪 → 抬起 → 移动到放置点 → 松手。

- `execute:=true` 才真的动；不给就只规划，RViz 里看轨迹。
- 相机已经在别的终端起过时加 `vision:=false`，否则两个进程抢同一个相机。
- 不用 `auto_start:=true` 时，节点等触发：

  ```bash
  ros2 topic pub --once /pick_place/start std_msgs/msg/Empty "{}"
  ```

## 夹爪

夹爪不走机械臂那条总线，走 USB-CAN 适配器。先把它拉起来：

```bash
sudo ip link set can0 up type can bitrate 1000000
```

`gripper:=true` 才会起 `gripper_can_node`。它启动时先把手指推到底量一次零点，日志里能看到 `合拢止位 … 张开止位 …`。没接适配器时可以 `gripper_channel:=vcan0` 干跑。

| 话题 | 内容 |
|---|---|
| `/gripper/width` | 下发的开口宽度，mm |
| `/gripper/position` | 回读的开口宽度，mm |
| `/gripper/status` | `ready` / `default_zero` / `no_zero` |

手动动一下夹爪：

```bash
ros2 topic pub --once /gripper/width std_msgs/msg/Float64 "{data: 30.0}"
ros2 topic echo /gripper/position
```

`no_zero` 表示零点没量出来，抓取节点会停下不动；重跑节点它会自己再量一次。

## 只看、不抓（视觉调试）

```bash
ros2 launch dummy_vc_perception vision_demo.launch.py
ros2 topic echo /detected_objects
```

## 换方块、换放置点

改 `dummy_vc_ws/src/dummy_vc_grasp/config/pick_place.yaml`，也可以在启动时当参数传（如 `place_x:=0.0`）。

| 参数 | 作用 |
|---|---|
| `place_x` / `place_y` | 放置点（base_link 下，米） |
| `place_surface_m` | 放置台顶面离桌面多高（米），默认 0.055 |
| `place_lift_edges` | 松手前再多抬几个方块边长，默认 0.5 |
| `grasp_offset_x_m` / `grasp_offset_y_m` | 补实际抓取与目标之间的固定偏差（米） |
| `grasp_open_ratio` | 开口 = 边长 × 该倍数，默认 1.8；超过 55 mm 自动截到上限并警告 |
| `vision_homography` | 视觉检测到真值的 2D 单应修正，8 个数，全 0 = 不修 |
| `vision_homography_region` | 单应可信范围 `[x0, x1, y0, y1]`，超出会警告（不拦） |
| `execute` | `false` 只规划，`true` 真下发 |
| `max_velocity_scaling` | 关节速度比例，默认 0.15 |

`--symlink-install` 下改 yaml 不用重新构建，重启节点即可。

## 常用查看

```bash
ros2 topic echo /joint_states           # 六个关节角（弧度）
ros2 topic echo /gripper/status         # 夹爪能不能动
ros2 topic echo /detected_objects       # 检测到的方块
ros2 topic hz /camera/color/image_raw   # 相机帧率
```

## 约定

| 项 | 值 |
|---|---|
| 关节名 | `dummy2_joint1` … `dummy2_joint6`，夹爪 `dummy2_finger_joint1/2` |
| 角度单位 | `/joint_states`、MoveIt 用弧度；设备侧 `move_j` 用度 |
| 规划末端 | `gripper_tcp` |
| 夹爪话题单位 | mm（节点内部换算成米） |
| 相机画面 | 640×480 |

## 已知限制

- 深度路径不可用，检测走单目。
- 单应修正只在标定范围内可信：x −0.10…+0.10 m、y 0.15…0.35 m，超范围只警告。
- 在 VirtualBox 里跑时相机偶尔丢帧（`color frame is not decoded`），表现为帧率下降，不影响抓取。

## 来源

- 机械臂与固件：[peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)
- 本仓库：https://github.com/sky-nolimit/dummy_vc
