# dummy_vc_ws — ROS 2 工作区

**这个目录里的每一个包都会被 `colcon build` 编译。** 放不进来的东西请放到仓库根的 `reference/`（本地目录，不上传）。

## 当前包含的包

| 包 | 作用 | 来源 |
|---|---|---|
| `dummy2_urdf` | 机械臂模型（URDF/Xacro + STL）—— **唯一模型来源** | 上游 `SuJerry01/dummy2_urdf` |
| `dummy_controller` | ROS 2 控制器（lifecycle 节点 + fibre 驱动 + `dummy_cli_tool`）—— **唯一硬件通道** | 上游 `yeyiru/dummy_ros2_driver`，本地已改名 |
| `dummy_vc_calibration` | 阶段 0 硬件标定节点（预览-确认-执行） | 我们写的 |
| `dummy_vc_moveit_config` | MoveIt 2 配置（SRDF、限位、规划器、RViz） | 我们写的 |

> **注意包名与目录名**：`dummy_controller` 的目录名是本地改过的，包名沿用上游。
> 仓库根 README 里有说明。

## 规划中、尚未创建的包

按视觉抓取方案（本机 docs/01，不上传）的分阶段计划逐步加入：

| 包 | 作用 | 计划阶段 |
|---|---|---|
| `dummy_vc_bringup` | launch + RViz 配置；分 `bringup_sim` / `bringup_real` 两套 | 阶段 1~2 |
| `dummy_vc_perception` | 视觉：分割、点云聚类、位姿估计 | 阶段 4 |
| `dummy_vc_grasp` | 抓取位姿生成 + 任务状态机 | 阶段 5 |

## 添加新包的注意事项

1. **包名前缀统一用 `dummy_vc_`**，与上游包（`dummy2_`、`dummy_`、`dummy_controller`）区分开，一眼能看出哪些是我们写的。
2. **不要引入第二个机器人模型包。** 模型只允许 `dummy2_urdf` 一份，任何第二份 `robot_description` 都会造成**静默**的 IK 错误 —— 表现为能正常规划但动作全错，极难定位。
3. **不要复制 `dummy_cli_tool`。** 它只存在于 `dummy_controller` 内，原因见根目录 README。

## 构建

```bash
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 对 Python 包生效，改 `.py` 后不必重新 build；但改 **xacro / urdf / launch / yaml 参数文件后仍需重新 build**（或直接用 `--symlink-install` 让它们也走软链）。

## 检查

不需要硬件、也不需要 source ROS 即可运行：

```bash
python ../tools/run_checks.py
```
