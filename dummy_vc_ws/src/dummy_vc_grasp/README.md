# dummy_vc_grasp —— 阶段 5：抓取规划与任务状态机

把"`/detected_objects` 里有个方块"变成"手臂真的去把它抓起来"。分两层，界线是
**能不能离线自测**：

| 文件 | 依赖 | 能不能离线自测 |
| --- | --- | --- |
| `planner.py` | numpy + `dummy_vc_perception` 的几何层 | ✅ `tools/selftest_grasp_task.py` |
| `task.py` | 只有标准库 | ✅ 同上（把每条迁移边都走一遍） |
| `pick_place_node.py` | ROS 2 + `moveit_msgs` | ❌ 只能在 VM 上跑 |

```bash
python tools/selftest_grasp_task.py     # 不需要 ROS
```

## 一次抓取的顺序

```
IDLE ─START→ OBSERVE ─看到目标→ MOVE_PRE_GRASP → APPROACH → CLOSE_GRIPPER
     → LIFT → MOVE_PLACE → OPEN_GRIPPER → RETREAT → DONE
```

- **OBSERVE** 单独一步（docs/01 阶段 5 的要求）：目标位姿在这里**冻结**，后面
  每一步都用同一个点。这样失败时能分清"看错了"还是"走偏了"。
- 运动全部走 MoveIt 的 `MoveGroup` 动作（`/move_action`），一个 goal 里
  plan+execute 一起做 —— 碰撞检查和路径插值仍是 MoveIt 的事，节点不自己拼轨迹。
- 可重试的步骤（起手位/下探/抬升/移动到放置点/退回）失败后原地重试，默认 2 次。
- **夹爪那两步不重试**：合爪失败过就说明不确定指头在哪，再试一次可能把方块挤飞
  或把手指别住；开爪失败时方块还在指间，退回起点比撒手更糟。

## 夹爪现在还发不出去（A2）

`gripper.py` 的 `FingerOpeningModel` 在"角度<->开口"没实测之前，`command_for_width()`
抛 `UnverifiedGripperModel`。所以：

- 规划照做（宽度只要算术：边长 + 2 mm）；
- 状态机走到 `CLOSE_GRIPPER` 会直接进 FAULT 并说明原因，**不发** `hand.*` 命令；
- 量完阶段 0.5（docs/02）之后用 `opening_endpoints_from_measurements()` 换成
  实测模型，闸门自动打开。

这样在夹爪没标定之前，前面四步（起手位、下探）在真机上照样能验。

## 在 VM 上跑

需要三样东西同时活着：`move_group`、`vision_node`、本节点。

**两条命令**（一条起 MoveIt，一条起"看 + 抓"）：

```bash
# 终端 1：MoveIt（接真机就是 moveit_hardware.launch.py）
ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py

# 终端 2：视觉（URDF/TF + 相机 + vision_node）+ 本节点
ros2 launch dummy_vc_grasp pick_place_demo.launch.py auto_start:=true
```

`pick_place_demo.launch.py` 只负责"看 + 抓"这一半，**不含机械臂驱动和 MoveIt**
（它们各开一个终端，日志和退出各归各的）。它的开关：

| 参数 | 默认 | 用途 |
|---|---|---|
| `execute` | `false` | true 才真的动；默认只规划（夹爪 A2 还挡着） |
| `auto_start` | `false` | true 则一启动就开始等目标 |
| `vision` | `true` | 视觉那半已经在跑就传 `false` |
| `camera` | `true` | 传给视觉 launch：是否顺带起相机 |
| `place_x` / `place_y` | `0.0` / `0.10` | 放置点（base_link 下，米） |
| `params_file` | 包内 `config/pick_place.yaml` | 换参数就复制一份改 |

想分开排查时的手工等价命令：

```bash
ros2 run dummy_vc_perception vision_node --ros-args \
  --params-file $(ros2 pkg prefix dummy_vc_perception)/share/dummy_vc_perception/config/vision.yaml \
  -p image_topic:=/camera/color/image_raw \
  -p camera_info_topic:=/camera/color/camera_info

ros2 run dummy_vc_grasp pick_place_node --ros-args \
  --params-file $(ros2 pkg prefix dummy_vc_grasp)/share/dummy_vc_grasp/config/pick_place.yaml \
  -p auto_start:=true
```

**判据**（按顺序）：

1. 日志出现 `抓到目标：目标 (x, y, z) m，抓取点上方 80 mm 起手，开口 26.0 mm...`
2. 日志出现 `发给 MoveIt：... m，只规划`，随后 `MoveIt 完成（error_code=1）`
   —— `error_code=1` 是 `SUCCESS`；规划成功说明路点在可达且无碰撞的位置。
3. 想让它真的动：加 `-p execute:=true`。这时先确认 `ee_link`（`gripper_tcp`）
   在 xacro 里是可用的 —— 它的偏移 `tcp_offset_y = 0.015` 是尺量的，实物几何
   与 URDF 手指不同（docs/04 A1），碰撞体还是原作者的模型。
4. 走到合爪时会看到 `夹爪拒绝下发：...未量...`，任务停在 FAULT —— **这是对的**，
   直到阶段 0.5 量完。

## 已知的坑

- `gripper_tcp` 落在 `dummy_hand_joint` 原点**后方**（`0.015 - 0.052312 = -0.0373`）：
  URDF 的手是 52 mm 长、实物 TCP 只有 15 mm，碰撞感知规划前必须换成简化碰撞体
  （docs/04 A1）。
- 放置点现在是参数（`place_x/place_y`），不是视觉算的 —— 首期刻意如此。
- `execute:=true` 之前请确认急停路径：急停仍然只有 Ctrl-C 驱动节点或断电
  （没有 ROS 服务）。
