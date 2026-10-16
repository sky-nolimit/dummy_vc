# tools/ — 独立工具索引

把"靠眼睛看、靠记忆猜"的事情变成可复现的检查。除标注"需要 ROS"或"需要硬件"
的以外，都不依赖 ROS 环境，Windows 和 Ubuntu 上都可直接用 Python 执行。

**目前 49 个工具 + 本文件**（数的是 `tools/` 下的文件，去掉本文件和
`ssh_config_for_github` 那个配置模板）。数量不少，所以下面按用途分组。

**曾经有 52 个。** 2026-10-14 清理掉了 9 个：8 个是排查特定假设时写的一次性
脚本（问题定位后不会再被用到），加上 1 个结论本来就是错的。**其中的结论都
已经整理进 `docs/04` 与 `docs/05`，或者有更可靠的替代做法**（见下方各组的
说明）。下面第 4 组的夹爪探测脚本**全部保留** —— 夹爪尚未结案。

> **新增工具前先确认它不与现有工具重叠。** 保留多个职责相同的工具是本项目
> 反复出错的来源之一。

---

## 一、每次改完代码跑这个就够

```bash
python tools/run_checks.py
```

**14 项一次过**，不需要硬件也不需要 source ROS。它串起下面这些（表的顺序就是
`run_checks.py` 里 `CHECKS` 的顺序）：

| 工具 | 作用 |
|---|---|
| `run_checks.py` | ★ **一条命令跑完全部离线检查**，汇总一个结论 |
| `check_syntax.py` | 全仓库 Python 语法检查（Windows 上改代码后必跑） |
| `check_encoding.py` | 编码 / BOM / mojibake 检查（BOM 会让 Linux 上的 shebang 失效） |
| `check_line_endings.py` | **从 git 对象里**检查换行符（Linux 收到的是 blob，不是工作区） |
| `check_urdf.py` | URDF 结构：XML 合法性、父子引用、树连通性、必需坐标系 |
| `check_urdf_limits.py` | URDF 关节限位**不超过**设备实测行程 |
| `check_moveit_config.py` | MoveIt 配置 vs URDF vs 设备实测，三处交叉校验（含规划组类型、控制器类型、robot name 一致性） |
| `check_launch_consistency.py` | launch 不会因"配置参数缺 DESCRIPTION"而在启动时抛错 |
| `check_conversions.py` | **导入真实驱动代码**，验证换算函数两两互逆、快捷式与两步链一致 |
| `check_downlink.py` | **下发链对实测 (move_j, raw) 配对**，抓"静默丢弃"那类故障 |
| `selftest_conversion.py` | **标定自测**：锚点、收缩姿态折叠、往返恒等、三处默认值一致 |
| `selftest_wait.py` | 等待逻辑对**模拟的**慢速设备（含起步延迟）是否可靠 |
| `selftest_verify_moveit.py` | 静态自查 `verify_moveit_vm.py` 自己的调用点（解包元数、`check.xxx` 是否存在、有没有"在 A 函数里 import 却给 B 函数用"）。**它只能在虚拟机上跑，一个笔误要花一个来回**，所以在本地先查 |
| `selftest_check_rviz_plugin.py` | 用**假 ROS 前缀**把 `check_rviz_plugin.py` 的四种情形在本地跑一遍（索引标记齐全 / 缺标记 / 标记指向不存在的文件 / XML 根本没装）。它同样只在虚拟机上跑，**而它出错时报的是"一个 panel 都没装"，跟它要诊断的故障一模一样** |
| `selftest_follow_joint_trajectory.py` | 离线核对 MoveIt 执行接口：action 名与 `moveit_controllers.yaml` **逐字相等**（只有两段 `/<controller_name>/<action_ns>`，重复一段单独判红）、`ActionServer` 真在 `on_activate` 里建起来、execute 回调是普通函数而不是协程、`_execute_trajectory` 每条路径都返回错误码、`_traj_executing` 一定在 `try/finally` 里清掉、执行器线程数够在运动时继续发 `/joint_states` |

**`fix_line_endings.ps1` 不在上表里** —— 它不检查、只**修复**，也不由 `run_checks.py`
调用，Windows 上改过文件后按 `AGENTS.md` 手动跑：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/fix_line_endings.ps1 -Check
```

`-NoProfile -ExecutionPolicy Bypass` 不能省：默认执行策略会以
`... is not digitally signed` 直接拒绝这个脚本（**这不是脚本坏了**）。它把工作区的
CRLF 批量转回 LF，并**排除 `reference/`** —— 那是上游原文。

### 四个最关键的检查，以及为什么

**`check_downlink.py`** —— 拿实测配对去验下发链。**这是唯一能抓"两个错误正好
互相抵消"的检查**：`movej_to_angle` / `angle_to_movej` / `movej_to_urdf` 必须
一起改，而互逆测试抓不到两个匹配的错误。它曾经抓到 j3 目标算成 −1.474、
超出设备区间 `[45,180]`，导致**整条命令被静默丢弃**。

**但它有一个盲点，别把它当成"标定自洽"的证明**：它把驱动的代数抄成了自己的常量
（`SIGN` / `JOFF` / `MJOFF`），所以它证的是"驱动代码与这些常量彼此一致"。当**配置里的
数字与设备实测不符**时它照样报绿 —— `joint_offset_deg` 就是这样：配置里还是旧的
手工锚定值，与 `probe_pairs.py` 实测的 home 回读差 j1 +0.215 / j2 −0.460 /
j3 **−1.458** / j4 +0.908 / j5 −0.035 / j6 −0.033 度（2026-10-16 的精度核对，
决定暂不修），而 `check_downlink.py` 一直 `[ok]`。**"配置 ↔ 实测"这一段只有硬件上的
`probe_pairs.py` / `probe_mapping.py` 能验**，离线检查覆盖不到。

**`selftest_conversion.py`** 独立复算整套标定（不读驱动代码，只读配置）：

1. **锚点** —— 7 姿态下 URDF 必须全零，move_j 必须是 `[0,0,90,...]`
2. **收缩姿态** —— 断电收缩必须**折叠**。这是唯一独立于符号选择的校验：
   收缩时设备读数约全零，所以 `urdf_retract = sign * (0 - offset)`，
   **符号错了就会破坏这一项**
3. **往返恒等** —— URDF → move_j → URDF，以及 URDF → angle → URDF
4. **三处默认值一致** —— 配置文件、驱动代码默认值、launch 默认值

第 4 项尤其重要：这三处**曾经各自漂移，导致下发角度错了几十度而实物完全不动**。

**`selftest_wait.py`** —— 手臂起步有约 1.3 秒延迟，用固定 sleep 会**在设备
还没开始动的时候就判定"没动"**。这个自测用模拟设备覆盖四种情形。

**`selftest_verify_moveit.py`** —— `verify_moveit_vm.py` 是唯一只能在虚拟机上
跑的东西，而它已经因为**自己**的笔误白白花掉三个来回。前两个：把某个辅助函数
的返回值从 2 个改成 3 个、却漏改了另外两处调用点，运行时
`ValueError: too many values to unpack`；以及把"节点名列表里没有"直接判红，
而 `get_node_names()` 本来就可能因为 DDS 发现延迟漏掉活着的节点。

**第三个最值得记：`_wait_future()` 里的 `rclpy` 从来就是 NameError。**
`import rclpy` 写在了 `main()` 里 —— 那是**局部**名字，而 `_wait_future()` 是
模块级函数，看不到它。于是 `rclpy.spin_once()` 每次都抛 `NameError`，又被
`except Exception` 吞掉、退化成 `time.sleep`：**executor 一次都没转过，action
回包永远收不到，只能等 60 秒超时**。而 `move_group` 那边的日志写着
`Motion plan was computed successfully.` —— 于是排查看起来像"规划器坏了"，
实际上脚本根本没在收消息。两次改（先改成"确实 spin"、再加类型门）都没修好，
因为这根本不是逻辑问题而是作用域问题。

所以这个自测**用 AST 静态检查**，不需要 ROS：

1. 六个辅助函数的 `return` 元数必须自洽，且每个调用点解包的个数要对得上
   （返回 `f()` 的转调也算，否则 `param_via_cli` 会被误判成元数不一致）
2. `check.<方法>()` 必须在 `Check` 类上真的存在，参数个数也要在允许区间内
   —— 新增 `check.note` 而忘了定义方法就会在这里被抓住
3. 顶层 `def` 不能重名
4. **不能在某个函数里 `import`、却在另一个函数里用**（第 4 条专治上面那个
   `rclpy`）
5. **顶层被赋成 `None` 的名字不能真被用** —— `except ImportError: rclpy = None`
   这种兜底本身是对的，但它和"顶层 import 没成功"长得一模一样；真出事时
   顶层只剩 `rclpy = None`，`rclpy.spin_once` 报 `AttributeError` 又被吞掉，
   同样是静默失败

第 4、5 条都做过反向验证：把顶层 `import rclpy` 去掉之后，这个自测报
`顶层第 50 行把 rclpy 赋成了 None … 但第 397/669/648/674 行真的在用它`。

它是**纯静态**的（外加第 4 项把三个依赖函数换成桩真跑一遍类型门），所以不需要
任何模拟对象就能跑；代价是它抓不到运行时才出现的问题（比如上面那条 DDS 误报）。

---

## 二、连硬件之前的公共设施

| 工具 | 作用 |
|---|---|
| `device_session.py` | **连接设备并真正使能它**。`connect()` / `power_on()` / `wait_settle()` / `move_and_report()` |
| `rig_common.py` | 驱动侧测试脚本的共用辅助（两阶段等待：先等起步、再等停止） |

`device_session.py` 的存在理由值得记下：**`probe_can_wedge.py` 曾经漏了
`robot.set_enable(True)`**，于是脚本老老实实报告"手臂没动" —— 而那正是它本来
要调查的现象。**一个漏掉的准备步骤会伪造被调查的故障。**

⚠ **同一时刻只有一个进程能持有 fibre 链路。** 跑任何需要硬件的脚本前：

```bash
pkill -f dummy_controller
```

---

## 三、设备行为探测（需要硬件）

| 工具 | 作用 |
|---|---|
| `probe_pairs.py` | **记录 (命令, 回读) 配对** —— 标定的权威数据来源。`--refs-only` 只走两个命名姿态 |
| `probe_mapping.py` | 逐关节测"命令值 ↔ 回读值"的 gain 与 offset。**offset 必须从设备读出来，不能靠推导** |
| `probe_range.py` | 扫出设备**实际接受**的 move_j 区间。区间外会被**静默丢弃**（整条命令不执行） |
| `analyse_range.py` | 归类 `probe_range` 的输出，区分"区间外被拒"与"等于基准位姿的空操作" |
| `probe_moveit_api.py` | 打印**本机** `moveit_configs_utils` 的真实方法签名 |

`probe_moveit_api.py` 的理由：MoveItConfigsBuilder 的参数名在不同发行版里不同，
**猜错一次就要占用一轮 VM 启动**。这个脚本把"猜"换成"问"。

> probe_relation 曾用于拟合 `raw = a·move_j + b`，2026-10-14 删除 ——
> 它与 `probe_mapping.py` 职责重叠，而后者会同时打印下发值和读回值，更便于
> 人工核对。

---

## 四、夹爪专项（需要硬件，且**会卡死设备**）

**先读 `docs/04` 的 A2 和 `docs/06`。** 结论：三个 `hand.*` 方法都永不返回，
而且**调用之后整个机械臂失去响应，必须断电重启**。所以下面这些脚本要谨慎跑，
每个之后都断电重启。

| 工具 | 作用 |
|---|---|
| `probe_hand_timeout.py` | 量一个 hand 调用**到底怎么结束**（返回 / 抛异常 / 永不返回） |
| `probe_can_wedge.py` | **一次 hand 调用是否卡死整条 CAN**：电机 → hand → 电机，看第三步 |
| `probe_scan_after_hand.py` | hand 之后设备还剩什么能用。**一次调用只做一个操作**，带硬期限 |
| `gripper_call.py` | 只调**一个**夹爪方法并报告结果，不做别的 |
| `gripper_sequence.py` | 在一个进程里走完夹爪接口，让**调用顺序**可见 |
| `dump_protocol.py` | 解码设备自己的协议描述，看 `move_j` 与 `hand` 各自需要什么 |
| `probe_gripper.py` | 靠眼睛标定夹爪（因为夹爪读不出来）。**目前做不到，见 docs/04 A2** |
| `test_gripper.py` | 走 ROS 话题驱动夹爪。**当前无效** |

**`probe_scan_after_hand.py` 默认不重复那次永不返回的调用**（用
`--redo-hand-call` 才做）：那个调用会留下一个持有 libusb 互斥量的线程，
**连 Ctrl-C 都无法干净退出**。

**2026-10-14 删掉的夹爪排查脚本，以及它们的结论现在在哪：**

| 曾用工具 | 它得到的东西 | 现在记在 |
|---|---|---|
| gripper_ids | 设备声明 `set_angle` 296/297、`set_enable` 298/299、`set_current_limit` 300/301 | `docs/04` A2 |
| dump_hand_protocol | 固件如何声明 `hand` 这三个 endpoint | `docs/05` |
| compare_endpoints | 与已知可用 endpoint 的**按名**对照（路径式匹配会失效） | `docs/05` |

**若还想查 endpoint id**：用 `probe_scan_after_hand.py --step definition`。
它从**活的对象**读 id 并与设备声明的对照，比解析一份转储更可靠、且不会过期。

---

## 五、驱动侧验收与 MoveIt 验证（需要 ROS）

| 工具 | 作用 |
|---|---|
| `verify_moveit_vm.py` | ★ 向 `move_group` 真发一次规划请求，判定 MoveIt 配置是否可用。**不需要驱动、不需要硬件** —— 只规划不执行 |
| `check_moveit_vm_paths.py` | ★ 在虚拟机上打印 builder 要加载的每个文件**解析到哪个绝对路径、存不存在**。`robot_description` 为空时先用它 |
| `check_moveit_builder.py` | ★ 拿**真实的 `MoveItConfigsBuilder`** 跑一遍 launch 里那条链，原样打印三个返回值并判定。路径都对但仍为空时用它 |
| `check_rviz_plugin.py` | 查 RViz 插件**链路真的断了**的故障：插件 XML 声明了什么、**ament 插件索引**里有没有对应标记、那条标记指向的 XML 在不在、`.so` 能不能 `dlopen`。**先确认不是 `moveit.rviz` 把 Display 写进了 `Panels:` 段**（见第 5 组说明）。不改系统，也不需要 rviz2 正在跑 |
| `selftest_verify_moveit.py` | 静态自查 `verify_moveit_vm.py`：调用点解包的元数、`check.<方法>` 是否存在（见第 1 组） |
| `selftest_check_rviz_plugin.py` | 用假 ROS 前缀离线自测 `check_rviz_plugin.py`（见第 1 组） |
| `selftest_follow_joint_trajectory.py` | 离线核对 MoveIt 执行接口：action 名是否与 `moveit_controllers.yaml` 逐字相等（**只有两段** `/<controller_name>/<action_ns>`，重复一段会单独判）、`ActionServer` 是否真的在 `on_activate` 里建起来、execute 回调是普通函数而不是协程、`_execute_trajectory` 每条路径都返回错误码、以及执行器线程数够不够在机械臂运动时继续发 `/joint_states`。**一句话对应"规划成功、Execute 不动"那个故障** |
| `final_test.py` | 驱动侧最终验收测试。**第 3、4 项 FAIL 不可采信**（`/joint_states` 滞后于实际运动，见 docs/04 B1） |
| `test_motion.sh` | 让某个关节转一个增量并报告结果（带确认提示） |
| `send_joint_delta.py` | `test_motion.sh` 的底层实现 |
| `debug_pose_chain.py` | 打印完整姿态链，用正运动学判断模型此刻显示的是哪种姿态 |
| `print_joints.py` | 列出关节与 URDF 限位 |

上表中需要驱动正在运行的是 `final_test.py`、`test_motion.sh`、
`send_joint_delta.py`、`debug_pose_chain.py`、`print_joints.py`；
**`verify_moveit_vm.py`、`check_moveit_vm_paths.py`、`check_moveit_builder.py`、
`check_rviz_plugin.py`、`selftest_verify_moveit.py`、
`selftest_check_rviz_plugin.py` 不需要**，这几个是给虚拟机/新机器第一次加载
MoveIt 配置用的（配套步骤见 `docs/08-MoveIt虚拟机验证清单.md`）。

**顺序是三个一层套一层，别跳步：**

| 顺序 | 工具 | 回答的问题 |
|---|---|---|
| 1 | `check_moveit_vm_paths.py` | 文件**找得到吗**（解析成绝对路径、存不存在、xacro 能否展开） |
| 2 | `check_moveit_builder.py` | 找到了之后，builder **返回的东西对不对**（三个返回值本身是否可用） |
| 3 | `verify_moveit_vm.py` | 规划**真的能算出来吗**（向 `move_group` 发一次真请求） |

**跳步会让下游工具报出误导性的失败**：文件缺失在 `verify_moveit_vm.py` 里表现为
"规划失败"，看起来像配置写错。

`check_moveit_vm_paths.py` 的理由：`MoveItConfigsBuilder` 要跨两个包找文件 ——
自己（`dummy_vc_moveit_config`）和 URDF 所在的 `dummy2_urdf_description`，而后者
**所在的目录名和包名不一样**（`src/dummy2_urdf/` 里装的是
`dummy2_urdf_description`），这种不一致在不同机器上解析结果可能不同。任何一处
找不到，症状都是同一句 `robot_description` 为空，然后 `robot_state_publisher`
报 `robot_description parameter must not be empty`、`move_group` 报
`Could not find parameter robot_description`、`rviz2` 报 `Error document empty` ——
**三句都没说是哪个文件**。这个脚本把那条链路一次列清楚，并且只报 `[ok]` / `[!!]`。

它的第 4 节会真用 `xacro` 展开一次 URDF，并检查 `kinematics.yaml` 的顶层键是否
正好是 `dummy_arm` / `gripper`：如果看到的是 `['robot_description_kinematics']`
这种"以文件名命名"的单键，说明加载器把整个文件包了一层，那是另一类故障。

`check_moveit_builder.py` 的理由：上一个工具**故意重实现了查找逻辑**，所以它
证明不了真实 builder 返回了什么。本工具走 launch 里那条一模一样的链，打印
`robot_description` / `robot_description_semantic` /
`robot_description_kinematics` 三个值的**类型和原始内容**，并按三条判据给结论：

1. `robot_description` 必须是非空的 URDF 字符串
2. `robot_description_semantic` 必须含 `<group name="dummy_arm"`
3. `robot_description_kinematics` 的**顶层键必须是规划组名**（`dummy_arm`、`gripper`）；
   只有一个以属性名命名的键 = 加载器把文件包了一层，`move_group` 会找不到任何
   规划组的运动学解算器

它还会打印 `moveit_configs_utils` / `moveit_cpp` 的版本号 —— 文件解析方式在
MoveIt 各发行版之间变过，**这是"同一份配置在一台机器上能跑、另一台上不能"的
最常见原因**。

`verify_moveit_vm.py` 的理由：RViz 里拖动能规划**不能证明配置是对的** —— 配置
错了也可能偶尔规划成功，外行分辨不了。它直接向 `move_group` 发一个**已知可达**
的目标，并检查返回的轨迹关节名和每个轨迹点是否都在实测限位内，让 `move_group`
自己回答。

它还会**先验自己**：目标位姿越限时 MoveIt 返回 `-14`（GOAL_CONSTRAINTS_VIOLATED），
看起来像配置坏了，实际是脚本填错了数。填目标时用真机 `move_j` 口径还是 URDF
口径差一个绕圈（`move_j j3=90` 对应 URDF `j3=-90`，而 URDF 上限只有 45），极易
搞混，所以脚本先按实测限位自检一遍再启动 ROS。

`check_rviz_plugin.py` 的理由：它查的是**插件链路真的断了**这一类故障 —— 包没装、
ament 索引里缺标记、或者 XML 指向的 `.so` `dlopen` 不起来。

**先说清楚一个已经踩过的坑，免得下次又走错**：RViz 报
`... with base class type 'rviz_common::Panel' does not exist. Declared types are <EMPTY>`
时，**这个空列表本身通常不是索引问题**。报错里的 `rviz_common::Panel` 是 rviz2
**去查的时候用的名字**（出错的是 Panels 工厂），不是插件声明的基类。MoveIt 的插件
XML 里写的是：

```
<class name="moveit_rviz_plugin/MotionPlanning"
       type="moveit_rviz_plugin::MotionPlanningDisplay"
       base_class_type="rviz_common::Display">
```

是 **Display**。rviz2 按基类各建一个工厂，Panel 工厂里全是 Display 派生类，于是
一个都没有 —— 空列表是**配置把这一块写进了 `Panels:` 段**的正常结果。正确做法是把
它搬到 `Visualization Manager: Displays:` 段（`dummy_vc_ws/src/dummy_vc_moveit_config/
config/moveit.rviz` 已改）。**先查这个，再来跑本脚本。**

真需要跑本脚本时，它按 0~6 段逐段判定并给出结论：

0. 环境（`ROS_DISTRO`、两条 PATH、`rviz2` 在不在）
1. 包在不在 ament 索引里、`share/` 目录在哪
2. XML 声明了哪些类、`base_class_type` 是不是 `rviz_common::Panel`
   （**XML 的根元素就是 `<library>`**，两种形状都处理）
2b. **索引标记 ↔ 磁盘 XML 是否互相印证**，并列出
   `rviz_common` / `rviz_default_plugins` / `moveit_ros_visualization` 三家的
   标记各在不在 —— 用来区分"只有 MoveIt 的索引入了坑"和"索引整个没被读到"
3. `.so` 在不在、能不能 `dlopen`（失败时跟一条 `ldd`，把缺的依赖点出来）
4. `rviz_common` 自己声明了几个 panel —— 它也是 0，就说明问题不在 MoveIt
5. `LD_PRELOAD`、运行中 `rviz2` 的 `/proc/<pid>/environ`（**RViz 是在装包之前
   启动的话，看到的就是旧环境**）

**它自己犯过一次很难发现的错**：插件的 XML **根元素就是 `<library>`**
（`moveit_ros_visualization` 与 `rviz_default_plugins` 都是这个形状），而最初写的是
`root.findall('library')` —— 在根元素底下找 `<library>` 子节点当然找不到，于是
"声明的类"永远是空的。**这个 bug 在虚拟机上会伪装成"MoveIt 的 panel 一个都没装"**，
也就是伪装成它本来要诊断的那个故障，而且看起来像是查到了证据。这就是
`selftest_check_rviz_plugin.py` 存在的原因：用假 ROS 前缀把四种情形（标记齐全 /
缺标记 / 标记指向不存在的文件 / XML 根本没装）在本地跑一遍，假前缀建在系统临时
目录里，**不会往 `tools/` 里留 `.tmp_*` 垃圾**。

`test_motion.sh` 把增量施加在 **URDF 空间**，由驱动自己的逆变换处理设备侧换算，
所以脚本不需要知道 move_j/angle 的偏置 —— **只有一个地方懂换算**。

---

## 六、符号与姿态分析（纯离线）

| 工具 | 作用 |
|---|---|
| `resolve_signs.py` | 三项要求联立求解标定：几何折叠 + 7 姿态读回全零 + 目标链复现实测命令 |
| `fold_signs.py` | 输出**折叠几何表** —— 真正定符号的那个检查 |
| `explore_signs.py` | 穷举 `joint_sign` 组合，按收缩姿态的末端位置排序 |
| `find_pose.py` | 按几何描述（如"大臂竖直小臂水平"）搜出匹配的关节角 |
| `fk_dummy.py` | URDF 正运动学，可画 x-z 侧视 ASCII 图 |
| `trace_control.py` | **用具体例子逐步打印整条控制链的每个数值及其来历** |

**`trace_control.py` 是回答"这个数是怎么来的"最直接的工具。**

判定符号用 `fold_signs.py` / `explore_signs.py`。价值在于把"符号对不对"变成
"认出哪个几何对"，而不是靠推理 —— **注意比较时看绝对方向，不要只看两段之间的
夹角**，因为镜像的折叠在夹角上是一样的。

---

## 七、MoveIt 配置生成

| 工具 | 作用 |
|---|---|
| `write_setup_assistant.py` | 生成 `config/.setup_assistant`（**只写一处**，由 CMake 装到 share 根，见下） |
| `yaml_min.py` | 一个够用的 YAML 读取器（本机没有 PyYAML 且装不上） |

`yaml_min.py` 的理由：没有它，参数文件的错误就只能留到另一台机器上构建时才发现，
**离线检查也就失去意义**。

`write_setup_assistant.py` **只写 `config/` 这一处**，理由是一条踩过的坑：

`MoveItConfigsBuilder` 在 Humble 上**只去 `<share>/.setup_assistant` 找**这个文件，
**不在 `<share>/config/` 下找**。而 `install(DIRECTORY config launch …)` 只会把它放到
`<share>/config/.setup_assistant`。找不到时 builder 不报错，只打两条 warning 然后退回
一个不存在的 `config/dummy2.urdf`，于是 `robot_description` 为空，报错在三个地方各出现
一次、**没有一条提到 URDF**。

所以：源文件留在 `config/`（Setup Assistant 自己的习惯），`CMakeLists.txt` 里用
`install(FILES config/.setup_assistant DESTINATION share/${PROJECT_NAME})` 装到 share
根目录。以前"写两处以防万一"的做法是错的 —— 有一处 builder 根本不看，而"源码里在、
装进去的里面不在"正是这个 bug 最难查的地方。

**改这个文件之后必须重新 `colcon build`**，并且删掉该包的 `build/` 和 `install/`：
`install(DIRECTORY …)` 没有依赖跟踪。

---

## 八、环境相关

| 工具 | 作用 |
|---|---|
| `ssh_config_for_github` | GitHub 22 端口被封时改走 443 的 SSH 配置模板 |
| `setup_github_remote.ps1` | 准备 git 远端、身份、分支名（PowerShell，默认演练） |

---

## 2026-10-14 的清理记录

删掉 9 个。**保留的决定性依据是"还有没有人 import 它"** —— 逐个查过：

| 删除 | 理由 |
|---|---|
| solve_signs | **结论是错的** —— 按位移方向推符号，被几何校验推翻 |
| solve_downlink | 一次性推导，结论已固化进驱动，并由 `check_downlink.py` 持续守着 |
| probe_relation | 与 probe_mapping 职责重叠 |
| find_fold | 与 find_pose、fold_signs 职责重叠 |
| go_zero | 与 send_joint_delta 职责重叠 |
| gripper_ids | 夹爪排查专用，结论已进 `docs/04` |
| dump_hand_protocol | 同上，结论已进 `docs/05` |
| compare_endpoints | 同上 |
| solve_j3 | 曾被本文件引用，但**文件本身早已不存在**（清单里的幽灵条目） |

**查依赖时发现两点，值得记下：**

1. **compare_endpoints 与 dump_hand_protocol 互相 import** —— 删一个必然弄坏
   另一个，只能同进同出。所以它们是一起删的。
2. **solve_signs 被 `fold_signs.py` 和 `resolve_signs.py` 提到**，但查过之后
   发现**只是 docstring 与打印文本里的文件名**，不是 import。
   **靠 grep 名字判断依赖会出错**，必须看是不是真的 `import`。

**保留的共用模块**（删任何东西前先确认它们在用；下表是 2026-10-16 用 AST 解出的
真实 `import` 图，不是 grep 文件名）：

| 模块 | 谁 import 它 |
|---|---|
| `fk_dummy.py` | `check_moveit_config.py`、`check_urdf_limits.py`、`debug_pose_chain.py`、`explore_signs.py`、`find_pose.py`、`fold_signs.py`、`resolve_signs.py`、`selftest_conversion.py`（**8 个，删不得**） |
| `device_session.py` | `probe_can_wedge.py`、`probe_scan_after_hand.py` |
| `rig_common.py` | `final_test.py`、`selftest_wait.py` |
| `yaml_min.py` | `check_moveit_config.py` |

`selftest_wait.py` **不是**被别人 import 的模块：它是被 `run_checks.py` 当脚本执行的
一项检查（它自己 import `rig_common`）。

---

## 2026-10-16 复查：一个都没删，以及为什么

用户要求清掉"过时无用"的脚本，于是把**全部 49 个**又审了一遍。判据和 10-14 那次
一样：**先看"还有没有人 import 它"，再看"它守护的问题是否已结案"**。结论是
**没有满足删除条件的** —— 每个文件都还有一个**活引用**（不是"看起来还有用"）：

| 看起来可以删的 | 活引用在哪 | 结论 |
|---|---|---|
| `probe_moveit_api.py` | `dummy_vc_ws/src/dummy_vc_moveit_config/config/planning_scene_monitor_parameters.yaml` 里写着"在目标机上跑它"来接线 | 保留（接线还没做） |
| `resolve_signs.py` | `fold_signs.py` 的正文点名它做"三项要求联立" | 保留（删了会留下悬空指引） |
| `print_joints.py` | `test_motion.sh --list` 直接调用它 | 保留 |
| `explore_signs.py` | 缺独立证据的 j1/j4/j5/j6 符号（`docs/04` 的 C1 未结案）正是它要穷举的 | 保留 |
| `probe_mapping.py`、`probe_range.py`、`analyse_range.py` | 精度问题 2026-10-16 决定缓一缓，而"配置 ↔ 实测"只有它们能验 | 保留 |
| 第 4 组夹爪那 8 个 | A2 未结案（`hand.*` 会卡死整机、要断电重启） | 保留 |
| `check_*` 12 个 + `selftest_*` 5 个 | 都在 `run_checks.py` 里 | 保留 |

**"老旧程度"在本仓库不构成判据**：git 里所有工具的日期都落在 2026-10-10 ~ 10-16，
整个目录只存在了一周。10-14 那次已经把那批一次性脚本清干净了，**剩下的都在守
未结案的问题**。

**唯一"使命已完成"的是第 8 组那两个 GitHub 环境脚本**（`setup_github_remote.ps1`
+ `ssh_config_for_github`）：远端、身份、`~/.ssh/config` 都已经配好，而它们的产物
不在仓库里，除本文件外没有别的引用。**要删得人来定** —— 删掉就等于放弃"换台机器
重新配一遍"的现成步骤，所以本文件保留它们并在此说明。

这次复查顺带修掉了**索引本身的四处错**（都是"数字对不上现实"这一类）：

1. 工具数 48 → **49**（`tools/` 下 51 个文件，去掉本文件和 `ssh_config_for_github`）
2. `run_checks.py` 的项数 13 → **14**
3. 第一组的表**漏了 `selftest_follow_joint_trajectory.py`**，却列着**并不由
   `run_checks.py` 执行**的 `fix_line_endings.ps1`（已挪到表外单独说明）
4. 共用模块表**漏了被 8 个文件 import 的 `fk_dummy.py`**，并且写错了
   `selftest_wait.py` 的角色

---

## 四个反复踩过的坑

**1. `git ls-files` 的输出不能直接当路径用。** PowerShell 5.1 收到的是带八进制
转义的路径，本仓库 `docs/` 和 `reference/` 下有中文与空格，直接用会报
"路径中具有非法字符"。相关脚本改为从文件系统枚举，或用 `-LiteralPath`。

**2. 不要用 PowerShell 做字符串替换来改源文件。** `Get-Content` + `Set-Content`
的往返会重新编码，已经毁过一个源文件（UTF-8 读、ANSI 写，产生乱码和 BOM）。
需要改文件时用编辑工具，或用 `[System.IO.File]::ReadAllText` /
`WriteAllText` 配 `UTF8Encoding($false)` 做字节级操作，事后跑 `check_encoding.py`。
**即使这样，含引号的 Python 字面量仍极易写坏** —— 本项目至少三次把 launch 文件的
多行字符串引号弄错。**改用编辑工具，不要批量替换。**

**3. `git rm` 遇到未跟踪文件会整体失败。** 一次删多个文件时，其中任何一个没被
跟踪，`git rm` 就报 pathspec 错误并中止，**一个都不删**。删之前先逐个确认跟踪
状态，已跟踪的用 `git rm --cached`（保留磁盘文件），未跟踪的用 `Remove-Item`。

**4. 探测脚本本身也会错，而且错得更隐蔽。** 本项目多次出现"脚本报 NOOP、实际是
脚本读得太早或**漏了准备步骤**"，进而把人引向错误结论。写探测脚本时：

- 等待要用**轮询到位**，不要固定 sleep（手臂起步有约 1.3 秒延迟）
- 基准要用**显式命令的绝对位姿**，不要用"刚读到的值"
- 输出要**同时打印下发的值和读回的值**，让人能自己核对
- **连接和使能要走 `device_session.py`**，别每次重写
