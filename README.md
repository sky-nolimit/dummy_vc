# Dummy 机械臂视觉抓取工作区

用 Dummy 机械臂（稚晖君 / 木子开源方案）实现桌面视觉抓取。

**当前状态**：方案已定稿，工作区已整合，尚未开始改代码。下一步是阶段 0 的硬件实测。

---

## 目录结构

```
Dummy_vc/
├── README.md              ← 当前文件：总入口
├── .gitignore
│
├── dummy_vc_ws/           ★ 唯一的 ROS 2 工作区，所有构建在这里发生
│   └── src/
│       ├── dummy2_urdf/           机械臂模型（URDF/Xacro + STL）—— 唯一模型来源
│       └── dummy_ros2_driver/     ROS 2 控制器（lifecycle 节点 + fibre 驱动）—— 唯一硬件通道
│
├── docs/                  方案、参数表、决策记录
│   ├── 01-视觉抓取方案.md          完整技术方案（含分阶段计划与风险清单）
│   └── stage0_params.yaml          阶段 0 实测参数表（待生成，是后续代码的输入）
│
├── tools/                 不依赖 ROS 的独立工具脚本
│
└── reference/             只读参考资料，永不参与构建
    └── dummy_moveit_ws/            参考项目（他人的 MoveIt 工作空间）
```

### 两类目录必须分清

| | 位置 | 是否构建 | 能否修改 |
|---|---|---|---|
| **可构建代码** | `dummy_vc_ws/src/` | 是 | 是，我们在这里开发 |
| **参考资料** | `reference/` | **否** | 只读，仅供查阅 |

`reference/` 被刻意放在 ROS 工作区之外。原因见下方「为什么参考项目必须移出工作区」。

---

## 三个原始项目的定位

| 项目 | 定位 | 处置 |
|---|---|---|
| `dummy2_urdf` | 机械臂描述文件 | **采用** → `dummy_vc_ws/src/dummy2_urdf/` |
| `dummy_ros2_driver` | ROS 2 控制器 | **采用** → `dummy_vc_ws/src/dummy_ros2_driver/` |
| `dummy_moveit_ws` | 他人的 MoveIt2 工作空间 | **仅参考** → `reference/dummy_moveit_ws/` |

### 为什么参考项目必须移出工作区

`dummy_moveit_ws` 顶层含 5 个带 `package.xml` 的 ROS 包，且都没有 `COLCON_IGNORE`。若留在工作区内：

1. colcon 会把 5 个包全部编译，拖慢每次构建；
2. **更危险的是** `dummy_moveit_config` 会装出另一份 `dummy-ros2.urdf.xacro`，其关节名是 `joint1..joint6`，而本机机械臂是 `dummy2_joint1..dummy2_joint6`。两份模型同名概念、不同关节名，`find_package` / `ros2 launch` / MoveIt 参数加载都可能**静默**摸到错的那份——表现为 MoveIt 能正常规划但 IK 全错，极难定位。

移到 `reference/` 后，构建范围内不存在第二份机器人模型。

### dummy_cli_tool 有且只有一份

`dummy_moveit_ws/dummy_controller/dummy_controller/dummy_cli_tool/` 下另有一份同名库，但**两份不可合并**：

- `dummy_ros2_driver` 版：把 `import fibre.protocol` 改写为 `import dummy_cli_tool.fibre.protocol`，使其能作为正式 Python 包被 `setup.py` 安装，避免 `fibre` 这个名字污染全局命名空间。**这是唯一能用的版本。**
- 参考项目版：保留原始 `fibre.*` import，另外附了一个 5.2 MB 的 Windows USB 驱动安装程序 `zadig-2.5.exe`。

两者的 Python 源码差异除 import 改写外几乎为零（`usbbulk_transport.py` 表面上差 5 行，实为 2 行 import 改写导致的行号偏移）。**合并会破坏包导入**，因此不合并，只保留 driver 版。

---

## 常用命令

```bash
# 构建（每次都从工作区根目录执行）
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

```bash
# 查看参考项目里的某个文件（只读，不构建）
ls reference/dummy_moveit_ws/
```

---

## 完整方案

技术方案、六个阻塞问题、分阶段实施计划、手眼标定流程、风险清单，全部在：

**→ [docs/01-视觉抓取方案.md](docs/01-视觉抓取方案.md)**

---

## 关键事实（速查）

- 机械臂：6 旋转关节 + 1 固定手爪 + 2 平移手指（行程各 23 mm），关节名 `dummy2_joint1..6`
- 硬件接口：ODrive / fibre，USB 连接，`ref_tool.find_any()` 自动发现
- 夹爪：`robot.hand.set_angle()` / `set_enable()` / `set_current_limit()`，与关节共用同一 fibre 设备
- 相机：深度相机（待到位），**装在末端（eye-in-hand）**
- 运行环境：Ubuntu 22.04 + ROS 2 Humble
- 首期目标：可复现的桌面抓取 Demo（识别 → 规划 → 抓取 → 放置）

## 当前分支状态（整合时的快照）

- `dummy_vc_ws/src/dummy2_urdf`：`main` @ `5e1e401`，工作区干净
- `dummy_vc_ws/src/dummy_ros2_driver`：`high_frequency` @ `03a5b1a`，**工作区有未提交改动**（`dummy_controller.py` 52 增 99 删），`probe_device.py` 为未跟踪文件。这些改动已随整合原样保留。

两个上游仓库的 `.git` 目录均保持原样，以便将来能追溯上游历史。**本项目不对上游做 pull**（硬件已验证的是当前这个 commit）。
