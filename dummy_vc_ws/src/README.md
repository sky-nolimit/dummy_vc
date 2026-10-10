# dummy_vc_ws — ROS 2 工作区

**这个目录里的每一个包都会被 `colcon build` 编译。** 放不进来的东西请放到仓库根的 `reference/`。

## 当前包含的包

| 包 | 作用 | 来源 |
|---|---|---|
| `dummy2_urdf` | 机械臂模型（URDF/Xacro + STL） | 上游 `SuJerry01/dummy2_urdf` |
| `dummy_ros2_driver` | ROS 2 控制器 + USB 相机 + fibre 驱动库 | 上游 `yeyiru/dummy_ros2_driver` |

## 规划中、尚未创建的包

按 `docs/01-视觉抓取方案.md` 的分阶段计划逐步加入：

| 包 | 作用 | 计划阶段 |
|---|---|---|
| `dummy_moveit_config` | MoveIt2 配置（用 Setup Assistant 生成） | 阶段 2 |
| `dummy_vc_bringup` | launch + RViz 配置；分 `bringup_sim` / `bringup_real` 两套 | 阶段 1~2 |
| `dummy_vc_perception` | 视觉：分割、点云聚类、位姿估计 | 阶段 4 |
| `dummy_vc_grasp` | 抓取位姿生成 + 任务状态机 | 阶段 5 |

## 添加新包的注意事项

1. **包名前缀统一用 `dummy_vc_`**，与上游包（`dummy2_`、`dummy_`）区分开，一眼能看出哪些是我们写的。
2. **不要引入第二个机器人模型包。** 模型只允许 `dummy2_urdf` 一份，任何第二份 `robot_description` 都会造成静默的 IK 错误。
3. **不要复制 `dummy_cli_tool`。** 它只存在于 `dummy_ros2_driver` 内，原因见根目录 README。

## 构建

```bash
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` 对 Python 包生效，改 `.py` 后不必重新 build；但改 **xacro / urdf / launch / yaml 参数文件后仍需重新 build**（或直接用 `--symlink-install` 让它们也走软链）。
