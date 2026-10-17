# Dummy 机械臂视觉抓取

在稚晖君 Dummy 六轴机械臂上实现基于视觉的抓取。

- **硬件**：Dummy 机械臂（6 旋转关节 + 舵机夹爪）+ 深度相机
- **软件**：Ubuntu 22.04 + ROS 2 Humble
- **开发方式**：Windows 编辑、Ubuntu 构建；GitHub 同步
- **仓库**：https://github.com/sky-nolimit/dummy_vc

---

## 这个仓库里有什么，没有什么

**推送到 GitHub 的只有代码。** 文档和外部参考资料留在本机，因为虚拟机上
测试时用不到它们，而 `reference/` 体积可观且会持续增加。

| 目录 | 上传 | 内容 |
|---|---|---|
| `dummy_vc_ws/` | ✅ | **唯一的 ROS 2 工作区**，所有构建在这里发生 |
| `tools/` | ✅ | 不依赖 ROS 的独立检查与探测脚本（索引见 [tools/README.md](tools/README.md)） |
| `README.md`、`.gitignore` | ✅ | 本文件与构建产物忽略规则 |
| `docs/` | ❌ | 设计文档、标定手册、缺陷记录、咨询材料 |
| `reference/` | ❌ | 外部参考资料：Dummy 固件源码、参考 MoveIt 工作空间、上游 diff |

**❌ 的两项目前只在本机磁盘上**（`docs/` 8 个文件，`reference/` 约 3900 个
文件）。`.gitignore` 已按路径忽略，所以 `git add -A` 不会再带上它们；
要用 `git add -f` 才能强制入库。**同样的目录名在本机仍然存在且可读可改。**

> **git 历史里仍有它们的旧版本**，所以 `.git` 不会因此变小。要彻底清掉需要
> 重写历史，那会改动已有提交的哈希；有需要再说。

---

## 工作区结构

```
Dummy_vc/
├── README.md              ← 当前文件：总入口
├── .gitignore
│
├── dummy_vc_ws/           ★ 唯一的 ROS 2 工作区，所有构建在这里发生
│   └── src/
│       ├── dummy2_urdf/           机械臂模型（URDF/Xacro + STL）—— 唯一模型来源
│       ├── dummy_controller/      ROS 2 控制器（lifecycle 节点 + fibre 驱动）—— 唯一硬件通道
│       ├── dummy_vc_calibration/  阶段 0 硬件标定节点（预览-确认-执行）
│       ├── dummy_vc_perception/   视觉感知（几何层已落地，节点待写）
│       └── dummy_vc_moveit_config/ MoveIt 2 配置（SRDF、限位、规划器、RViz）
│
├── tools/                 不依赖 ROS 的独立工具（索引见 tools/README.md）
│   ├── run_checks.py               ★ 一条命令跑完全部离线检查
│   ├── check_syntax.py             全仓库 Python 语法检查
│   ├── check_downlink.py           下发链对实测配对，抓"静默丢弃"那类故障
│   └── selftest_conversion.py      坐标换算 + 标定自测（改了换算必跑）
│
├── docs/                  ❌ 本地：设计文档、标定手册、缺陷记录
│   ├── 01-视觉抓取方案.md          完整技术方案（分阶段计划与风险清单）
│   ├── 02-阶段0标定操作手册.md      阶段 0 的逐步操作指引
│   ├── 03-进展与问题报告.md        历史快照（问题已解决，部分结论已过时）
│   ├── 04-已知缺陷与未决问题.md     ★ 动 URDF 几何或排查"不动"之前先看
│   ├── 05-固件源码事实.md          固件源码摘录与行号（权威依据）
│   ├── 06-夹爪问题咨询材料.md       可直接发给商家/原作者的说明
│   ├── 07-夹爪协议逆向.md          夹爪 CAN 协议逆向记录
│   └── 08-MoveIt虚拟机验证清单.md   ★ 在虚拟机上验证 MoveIt 的逐步清单
│
└── reference/             ❌ 本地：只读参考资料，永不参与构建
    ├── Firmware/                  Dummy 固件源码（含带注释的核心 7 个文件）
    ├── dummy_moveit_ws/           他人 MoveIt 工作空间（仅参考）
    └── upstream-local-changes/    上游未提交改动的存档
```

### 两类目录必须分清

| | 位置 | 是否构建 | 能否修改 |
|---|---|---|---|
| **可构建代码** | `dummy_vc_ws/src/` | 是 | 是，我们在这里开发 |
| **参考资料** | `reference/` | **否** | 只读，仅供查阅 |

### 为什么参考项目必须移出工作区

`reference/dummy_moveit_ws` 顶层含 5 个带 `package.xml` 的 ROS 包，且都没有
`COLCON_IGNORE`。若留在工作区内：

1. colcon 会把 5 个包全部编译，拖慢每次构建；
2. **更危险的是** `dummy_moveit_config` 会装出另一份 `dummy-ros2.urdf.xacro`，
   其关节名是 `joint1..joint6`，而本机机械臂是 `dummy2_joint1..dummy2_joint6`。
   两份模型同名概念、不同关节名，`find_package` / `ros2 launch` / MoveIt 参数
   加载都可能**静默**摸到错的那份 —— 表现为 MoveIt 能正常规划但 IK 全错，
   极难定位。

移到 `reference/` 后，构建范围内不存在第二份机器人模型。

### dummy_cli_tool 有且只有一份

`reference/dummy_moveit_ws/dummy_controller/dummy_controller/dummy_cli_tool/`
下另有一份同名库，但**两份不可合并**：

- `dummy_ros2_driver` 版：把 `import fibre.protocol` 改写为
  `import dummy_cli_tool.fibre.protocol`，使其能作为正式 Python 包被
  `setup.py` 安装，避免 `fibre` 这个名字污染全局命名空间。**这是唯一能用的版本。**
- 参考项目版：保留原始 `fibre.*` import，另外附了一个 5.2 MB 的 Windows USB
  驱动安装程序 `zadig-2.5.exe`。

两者的 Python 源码差异除 import 改写外几乎为零。**合并会破坏包导入**，
因此不合并，只保留 driver 版。

---

## 常用命令

### 构建

```bash
cd dummy_vc_ws
colcon build --symlink-install
source install/setup.bash
```

### 离线检查（不需要硬件，也不需要 ROS）

改完代码先跑这个，**15 项一次过**：

```bash
python tools/run_checks.py
```

它覆盖：Python 语法、编码/BOM、换行符、URDF 结构与限位、MoveIt 配置与 URDF
及设备实测行程的一致性、坐标换算恒等式、下发链对实测配对、标定自洽
（锚点/收缩折叠/往返/三处默认值一致）、等待逻辑（对模拟的慢速设备）、
MoveIt 的 action 接口、视觉几何层（顶抓轴向/畸变/桌面反投影/夹爪映射）。

### 起驱动

```bash
# 机械臂本体（夹爪默认关闭 —— 见下方"已知阻塞"）
ros2 launch dummy_controller dummy_controller.launch.py

# 另开终端看状态
ros2 topic echo /joint_states
```

### 起 MoveIt

```bash
# 纯规划，不需要硬件：先在 RViz 里验证配置是否正确
ros2 launch dummy_vc_moveit_config demo.launch.py

# 接真机：先起上面的驱动，再起这个
ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py
```

### 动一下某个关节

```bash
# URDF 空间，弧度，绝对目标
ros2 topic pub --once /joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/JointTrajectory \
  "{joint_names: [dummy2_joint1], points: [{positions: [0.3], time_from_start: {sec: 3}}]}"
```

---

## 当前进度

| 阶段 | 内容 | 状态 |
|---|---|---|
| — | 工作区整合、硬件接口探测 | ✅ 完成 |
| 0 | 关节角度标定（锚点、两个偏置、六个符号） | ✅ 完成 |
| **1** | **修驱动**：关节名、单位、TF、轨迹、坐标换算 | ✅ 完成 |
| 1b | 逐个关节验证方向（几何折叠 + 实测命令双重校验） | ✅ 完成 |
| 1c | 夹爪实测 | ⛔ **阻塞** —— 见下方 |
| **2** | **MoveIt 配置** | ✅ 真机 Plan & Execute 已连续可用（含安全退出）；配置见 `docs/08` |
| 3 | 驱动补强（限速、急停、GripperCommand 动作） | 部分完成（FollowJointTrajectory 动作已完成） |
| 4 | 视觉感知 | 🔸 几何层已落地并离线自测；节点待写（相机已到货：Gemini 335L） |
| 5 | 抓取规划 + 任务状态机 | 待开始 |
| 6 | 深度相机接入 + 手眼标定 | 待开始（相机已到货，先在 Windows 上验证） |

### 已知阻塞：夹爪

**6 个关节都能正常控制；但只要调用一次夹爪命令，夹爪不动，而且此后整个
机械臂失去响应，必须断电重启。**

已确认的事实：

- fibre 链路健康（读一个关节角 1 秒返回）
- `hand.set_angle` / `set_enable` / `set_current_limit` **三个都永不返回**
- 夹爪调用**之后**关节命令也不再应答
- 舵机**是通电的**（整机使能后手指变硬、有力矩），线没断、插头没空位

推断：固件卡在自己的 CAN 发送里（`CanSendMessage` 用 `osWaitForever` 取
`sem_can1_tx` 且自己不释放），电机与夹爪共用这一函数和同一条 CAN1。
**根因未定** —— 可能是 CAN 总线进入 error-passive/bus-off，也可能是固件 bug。

**咨询商家/原作者的完整材料在 `docs/06-夹爪问题咨询材料.md`（本地）。**

### 另外两点

- **`docs/04`（本地）里有全部已知缺陷与未决问题**，每条写清现象、证据、
  影响、以及处理它需要什么。动夹爪或 URDF 几何之前先看它。
- **URDF 的夹爪几何与实物不符**（原作者模型 vs 实物的齿轮齿条+直线导轨），
  形状与抓取中心都不同，差几十毫米。`gripper_tcp` 目前是占位值。

---

## 关键事实（速查）

| 项 | 值 |
|---|---|
| 关节名 | `dummy2_joint1` … `dummy2_joint6` + `dummy2_finger_joint1/2` |
| 单位约定 | 设备侧（`angle` 回读、`move_j` 指令）是**度**；URDF / `/joint_states` / MoveIt 是**弧度** |
| 规划链 | `base_link` → `gripper_tcp`（**不是** `link7`） |
| 设备关系 | `raw = move_j + [0, 75, −180, 0, 0, 0]`（两个命名姿态互差 0.01°） |
| 下发链 | `move_j = urdf/sign + movej_urdf_zero_deg` |
| 可接受区间 | j1 ±165°、j2 −75…90°、j3 **45…180°**、j4 −180…165°、j5 ±90°、j6 ±180° |
| 夹爪 CAN | 节点 **7**，角度 **0…30**（mode 0x02）、电流 **0…1**（mode 0x01），约 1 Mbit/s |
| 相机内参 | fx 495.23 / fy 494.59 / cx 303.81 / cy 249.03，640×480（**未标定**） |

**越界的 `move_j` 会被设备静默丢弃** —— 整条命令不执行，日志里什么都没有。
`normalize_movej` 会按区间折叠，找不到合法等价值时**报错**而不是静默下发。

---

## 原始来源

- 机械臂与固件：[peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot)
- 驱动与 URDF：原作者分享的 `dummy_ros2_driver` / `dummy2_urdf`
- MoveIt 参考（仅查阅）：`reference/dummy_moveit_ws`（本地）

---

## 版本控制说明

**AI 负责提交，人负责推送。** 改动完成并通过 `tools/run_checks.py` 后直接
`git commit`，commit message 用中文说明**为什么**改。`git push` 由人决定时机
（通常先在虚拟机上验证）。
