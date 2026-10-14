# Dummy 机械臂视觉抓取工作区

用 Dummy 机械臂（稚晖君 / 木子开源方案）实现桌面视觉抓取。

**当前状态**：工作区已整合；阶段 0（标定）与阶段 1（修驱动）已完成。当前在
1b（逐个关节实测方向）与 1c（夹爪实测）。详见下方「当前进度」表。

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
│       ├── dummy_controller/      ROS 2 控制器（lifecycle 节点 + fibre 驱动）—— 唯一硬件通道
│       └── dummy_vc_calibration/  阶段 0 硬件标定节点（预览-确认-执行）
│
├── docs/                  方案、手册与进展报告
│   ├── 01-视觉抓取方案.md          完整技术方案（分阶段计划与风险清单）
│   ├── 02-阶段0标定操作手册.md      阶段 0 的逐步操作指引
│   └── 03-进展与问题报告.md        **当前进展、阻塞问题与待咨询问题**
│
├── tools/                 不依赖 ROS 的独立工具脚本（索引见 tools/README.md）
│   ├── check_syntax.py            全仓库 Python 语法检查
│   ├── check_encoding.py          编码 / BOM / mojibake 检查
│   └── selftest_conversion.py     坐标换算 + 标定自测（改了换算必跑）
│
└── reference/             只读参考资料，永不参与构建
    ├── dummy_moveit_ws/           参考项目（他人的 MoveIt 工作空间）
    └── upstream-local-changes/    上游未提交改动的存档（patch）
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
| `dummy_ros2_driver` | ROS 2 控制器 | **采用** → `dummy_vc_ws/src/dummy_controller/`（仓库名沿用上游，本地目录已改名） |
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
# 构建（每次都从工作区目录执行）
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

```bash
# 启动驱动 + robot_state_publisher（TF / RViz 都靠它）
ros2 launch dummy_controller dummy_controller.launch.py

# 临时改一个标定值试一次，不编辑任何文件
ros2 launch dummy_controller dummy_controller.launch.py \
  joint_sign:="[-1.0, 1.0, -1.0, 1.0, -1.0, -1.0]"
```

> **标定参数只有一份，在 `dummy_controller/config/dummy_controller.yaml`。**
> launch 不再自带一份默认值副本——它是**在启动时从那个 YAML 读出来的**，
> 命令行参数只用于单次覆盖。以前 launch 里的副本会覆盖 YAML，导致"改了
> 配置文件却毫无效果"；`tools/selftest_conversion.py` 现在会强制这一点。

```bash
# 阶段 0 标定（真机）
ros2 launch dummy_vc_calibration stage0.launch.py

# 先空跑验证，不接硬件
ros2 launch dummy_vc_calibration stage0.launch.py dry_run:=true
```

```bash
# 改完源码后的自检（Windows / Ubuntu 都能跑，不需要 ROS）
python tools/check_syntax.py
python tools/check_encoding.py <文件>
```

```bash
# 查看参考项目里的某个文件（只读，不构建）
ls reference/dummy_moveit_ws/
```

## 当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| — | 工作区整合 | ✅ 完成 |
| — | 硬件接口探测 | ✅ 完成 |
| 0 | 关节角度标定（锚点、两个偏置、六个符号） | ✅ 完成（j1/j4/j5/j6 的符号缺独立证据，见 docs/04 C1） |
| **1** | **修驱动**：关节名、单位、TF、轨迹、坐标换算 | ✅ 完成 |
| 1b | 逐个关节验证方向 | ✅ 完成（几何折叠 + 实测命令双重校验） |
| 1c | 夹爪实测 | ⛔ **阻塞** —— `hand` 接口在固件层不响应，见 docs/04 A2 |
| **2** | **MoveIt 配置** | ✅ 配置包已建（`dummy_vc_moveit_config`），⏳ **待在 VM 上验证** |
| 3 | 驱动补强（限速、急停、GripperCommand 动作） | 部分完成（夹爪那部分被 A2 卡住） |
| 4 | 视觉感知 | 待开始（相机未到货，可先用 USB 相机搭框架） |
| 5 | 抓取规划 + 任务状态机 | 待开始 |
| 6 | 深度相机接入 + 手眼标定 | 待开始（等相机） |

**⚠ 全部已知缺陷与未决问题集中在这里 → [docs/04-已知缺陷与未决问题.md](docs/04-已知缺陷与未决问题.md)**

那里面按严重程度列了每条的现象、证据、影响、以及处理它需要什么。**改夹爪之前务必先看 A1** —— URDF 的夹爪几何与实物不符。

### 离线检查

改完代码跑这个，11 项一次过：

```bash
python tools/run_checks.py
```

### 当前可用的接口

| 接口 | 类型 | 说明 |
|---|---|---|
| `/joint_trajectory_controller/joint_trajectory` | 订阅 `JointTrajectory` | 6 关节轨迹，绝对目标，弧度；按 `time_from_start` 依次执行**全部** waypoint |
| `/joint_states` | 发布 `JointState` | 6 关节 + 2 手指，弧度（手指位置是**命令值**，夹爪无回读） |
| `/dummy_gripper/command` | 订阅 `Float64` | 归一化开合。**当前无效**：设备的 hand 接口永久阻塞 |
| `/dummy_gripper/state` | 发布 `Float64` | 回读的归一化开合度（同上） |
| `/dummy_arm/controller/pose_command` | 订阅 `PoseStamped` | 走 `move_l`，**不推荐**（无末端回读，无法闭环） |

### 已知待办

**完整清单在 [docs/04](docs/04-已知缺陷与未决问题.md)，要点：**

- **夹爪不可驱动**（A2）：三个 `hand.*` 方法永久阻塞，固件不响应 endpoint id 296~301。**抓取的最后一步做不了**，但视觉/定位/规划/移动到位都不需要它
- **URDF 夹爪几何与实物不符**（A1）：形状、抓取中心都不同，差几十毫米。改之前先读那一条，它列出了必须一起改的文件
- **`/joint_states` 滞后于实际运动**（B1）：所以 `tools/final_test.py` 的第 3、4 项 FAIL 不可采信（已在文件顶部标注）
- **急停尚未接线**：`stop_dummy()` 的换算已修，但没有任何调用者，且轨迹执行用 `time.sleep` 阻塞在回调里、没有抢占点
- 驱动在轨迹执行时**不做限位检查**，越界目标照样下发（但 `normalize_movej` 会按设备实测区间折叠，超出区间会报错而不是静默失败）

## 工具

`tools/` 下有 17 个检查与分析脚本，全部索引见 [tools/README.md](tools/README.md)。
最常用的三个：

```bash
python tools/check_syntax.py            # 改完代码必跑
python tools/selftest_conversion.py     # 改坐标换算/标定必跑
python tools/debug_pose_chain.py        # 需要 ROS；看模型此刻是什么姿态
```

`selftest_conversion.py` 会校验锚点、收缩姿态折叠几何、往返恒等，以及
**配置文件 / 驱动代码 / launch 三处默认值是否一致**。最后一项尤其重要——
这三处曾经各自漂移，导致下发角度错了几十度而实物完全不动。

## 编码注意事项

本仓库在 **Windows 上编辑、Ubuntu 上构建**。整合过程中已经发生过一次真实事故：用 PowerShell 写文件时的编码往返，把源文件里的中文全部损坏成乱码（中文被当 GBK 解读后重新按 UTF-8 写出，并附带了 UTF-8 BOM）。

因此约定如下。注意**不是「禁止中文」**——`dummy_vc_ws/src/` 里本来就有大量
继承自上游的中文注释，它们目前完好；真正要守住的是下面这些**字节级性质**：

1. **一律 UTF-8，且不带 BOM。** UTF-16 BOM 会让源码在某些工具下直接不可读，
   而 UTF-8 BOM 会让脚本的 shebang 失效（Linux 上变成
   `bad interpreter`）。
2. **换行符一律 LF**，由 `.gitattributes` 在入库和检出两侧保证。
3. **不要用 PowerShell 的 `Get-Content` / `Set-Content` 改文件**——那一趟
   编码往返正是上次事故的成因。用编辑器，或用
   `[System.IO.File]::ReadAllText` / `WriteAllText` 配 `UTF8Encoding($false)`
   做字节级操作。
4. 改动后用 `tools/check_syntax.py` 和 `tools/check_encoding.py <文件>` 验证；
   后者检查 UTF-8 合法性、BOM 和 mojibake 特征。

## 完整方案

技术方案、六个阻塞问题、分阶段实施计划、手眼标定流程、风险清单：

**→ [docs/01-视觉抓取方案.md](docs/01-视觉抓取方案.md)**

阶段 0 的逐步操作指引（**注意：其中"读 hand.angle 建立角度↔开口映射"的做法
在这个硬件上做不到**，夹爪没有回读，见 docs/04 B2）：

**→ [docs/02-阶段0标定操作手册.md](docs/02-阶段0标定操作手册.md)**

**⚠ 全部已知缺陷与未决问题** —— 每条的现象、证据、影响、以及处理它需要什么。
改夹爪、动 URDF 几何、或遇到"规划成功但机械臂不动"之前先看这份：

**→ [docs/04-已知缺陷与未决问题.md](docs/04-已知缺陷与未决问题.md)**

**要拿夹爪问题去问商家/原作者时，直接用这份** —— 已按"事实 → 复现步骤 →
推断 → 想知道什么"组织，并刻意区分了实测与推断：

**→ [docs/06-夹爪问题咨询材料.md](docs/06-夹爪问题咨询材料.md)**

---

## 关键事实（速查）

- 机械臂：6 旋转关节 + 1 固定手爪 + 2 平移手指（行程各 23 mm），关节名 `dummy2_joint1..6`
- 硬件接口：ODrive / fibre，USB 连接，`ref_tool.find_any()` 自动发现
- 夹爪：`robot.hand.set_angle()` / `set_enable()` / `set_current_limit()`，与关节共用同一 fibre 设备
- 相机：深度相机（待到位），**外置支架固定安装（eye-to-hand）**
- 运行环境：Ubuntu 22.04 + ROS 2 Humble
- 首期目标：可复现的桌面抓取 Demo（识别 → 规划 → 抓取 → 放置）

## 版本控制说明

本仓库（根目录）是**唯一的 git 仓库**。原先三个项目各自带一个嵌套 `.git`，整合时已移除，由本仓库统一接管历史——这样 `git diff` 就能直接看到我们对上游文件做的每一处修改。

移除嵌套 `.git` 之前已逐项核实安全性：

| 仓库 | 移除前状态 | 结论 |
|---|---|---|
| `dummy2_urdf` | `main` @ `5e1e401`，工作区干净 | 全部提交存在于远端，可随时重新 clone |
| `dummy_ros2_driver` | `high_frequency` @ `03a5b1a`，**有未提交改动** | 无未推送提交、无 stash；改动已存档 |
| `dummy_moveit_ws` | `main` @ `cfcee7d`，**有 9 个文件未提交改动** | 无未推送提交、无 stash；改动已存档 |

**上游未提交改动的存档在 [`reference/upstream-local-changes/`](reference/upstream-local-changes/)**：

| 文件 | 内容 |
|---|---|
| `driver-uncommitted-2025.patch` | `dummy_ros2_driver` 的 52 增 99 删改动（基于 `03a5b1a`） |
| `reference-uncommitted-2025.patch` | `dummy_moveit_ws` 的 9 文件改动（基于 `cfcee7d`） |
| `probe_device.py.snapshot` | 该未跟踪文件的副本 |

> 两个 `.patch` 原先被 PowerShell 的 `>` 重定向写成了 **UTF-16**（带 BOM），
> `git apply` 直接拒绝（`error: No valid patches in input`），存档等于不可用。
> 现已转为 UTF-8（无 BOM）+ LF，`git apply --stat` 能正确读出 52/99 与 9 个文件，
> 与下表描述一致。这些改动**上游从未提交也从未公开**，务必别再让它被重新编码。

这些存档值得保留，因为其中包含**上游从未提交、也从未公开**的修复。例如 `aruco_tracker_node.py` 里把原作者自己的机器路径 `/home/hata_ros/apps/dummy_ws/...` 改成了自动查找 + 环境变量覆盖；`aruco_detector_node.py` 把 `transform_pose` 移进了 `try` 块（原代码在 TF 不可用时仍会执行并抛异常）。参考项目里的 `dummy_vision` 用的正是 `camera_color_optical_frame` + `base_link` 这条链，与本项目的外置相机方案同路，有直接借鉴价值。

**不对上游做 pull**：硬件已验证的就是当前这两个 commit，拉取更新会引入未经验证的变更。若将来确需重新获取上游，clone 地址见下方「原始来源」。

## 原始来源

| 项目 | 上游地址 |
|---|---|
| `dummy2_urdf` | https://github.com/SuJerry01/dummy2_urdf.git |
| `dummy_ros2_driver` | https://github.com/yeyiru/dummy_ros2_driver.git |
| `dummy_moveit_ws` | https://github.com/tonggeshuaiqiwudizuijunlang/dummy_moveit_ws.git |
