# tools/ — 独立工具索引

把"靠眼睛看、靠记忆猜"的事情变成可复现的检查。除标注"需要 ROS"或"需要硬件"
的以外，都不依赖 ROS 环境，Windows 和 Ubuntu 上都可直接用 Python 执行。

**目前 51 个工具 + 本文件。** 数量不少，所以下面按用途分组，并标出**哪些是
已经用完的一次性探测脚本**（"可选清理"一节）—— 它们记录了当年的排查过程，
但不会再被用到。

> **新增工具前先确认它不与现有工具重叠。** 保留多个职责相同的工具是本项目
> 反复出错的来源之一。

---

## 一、每次改完代码跑这个就够

```bash
python tools/run_checks.py
```

**11 项一次过**，不需要硬件也不需要 source ROS。它串起下面这些：

| 工具 | 作用 |
|---|---|
| `run_checks.py` | ★ **一条命令跑完全部离线检查**，汇总一个结论 |
| `check_syntax.py` | 全仓库 Python 语法检查（Windows 上改代码后必跑） |
| `check_encoding.py` | 编码 / BOM / mojibake 检查（BOM 会让 Linux 上的 shebang 失效） |
| `check_line_endings.py` | **从 git 对象里**检查换行符（Linux 收到的是 blob，不是工作区） |
| `fix_line_endings.ps1` | 把工作区的 CRLF 批量转回 LF（PowerShell）。**排除 `reference/`** —— 那是上游原文 |
| `check_urdf.py` | URDF 结构：XML 合法性、父子引用、树连通性、必需坐标系 |
| `check_urdf_limits.py` | URDF 关节限位**不超过**设备实测行程 |
| `check_moveit_config.py` | MoveIt 配置 vs URDF vs 设备实测，三处交叉校验 |
| `check_launch_consistency.py` | launch 不会因"配置参数缺 DESCRIPTION"而在启动时抛错 |
| `check_conversions.py` | **导入真实驱动代码**，验证换算函数两两互逆、快捷式与两步链一致 |
| `check_downlink.py` | **下发链对实测 (move_j, raw) 配对**，抓"静默丢弃"那类故障 |
| `selftest_conversion.py` | **标定自测**：锚点、收缩姿态折叠、往返恒等、三处默认值一致 |
| `selftest_wait.py` | 等待逻辑对**模拟的**慢速设备（含起步延迟）是否可靠 |

### 三个最关键的检查，以及为什么

**`check_downlink.py`** —— 拿实测配对去验下发链。**这是唯一能抓"两个错误正好
互相抵消"的检查**：`movej_to_angle` / `angle_to_movej` / `movej_to_urdf` 必须
一起改，而互逆测试抓不到两个匹配的错误。它曾经抓到 j3 目标算成 −1.474、
超出设备区间 `[45,180]`，导致**整条命令被静默丢弃**。

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
| `probe_relation.py` | 单关节最小二乘拟合 `raw = a·move_j + b`，读出 gain / offset / sign |
| `probe_mapping.py` | 逐关节测"命令值 ↔ 回读值"的 gain 与 offset。**offset 必须从设备读出来，不能靠推导** |
| `probe_range.py` | 扫出设备**实际接受**的 move_j 区间。区间外会被**静默丢弃**（整条命令不执行） |
| `analyse_range.py` | 归类 `probe_range` 的输出，区分"区间外被拒"与"等于基准位姿的空操作" |
| `probe_moveit_api.py` | 打印**本机** `moveit_configs_utils` 的真实方法签名 |

`probe_moveit_api.py` 的理由：MoveItConfigsBuilder 的参数名在不同发行版里不同，
**猜错一次就要占用一轮 VM 启动**。这个脚本把"猜"换成"问"。

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
| `gripper_ids.py` | 显示夹爪附近的 endpoint ID，确认"固件声明了但不应答" |
| `dump_hand_protocol.py` | 看固件**如何声明** `hand` 这几个 endpoint |
| `dump_protocol.py` | 解码设备自己的协议描述，找 `move_j` 需要什么 |
| `compare_endpoints.py` | 按**名字链**把夹爪 endpoint 与已知能用的对照 |
| `probe_gripper.py` | 靠眼睛标定夹爪（因为夹爪读不出来）。**目前做不到，见 docs/04 A2** |
| `test_gripper.py` | 走 ROS 话题驱动夹爪。**当前无效** |

**`probe_scan_after_hand.py` 默认不重复那次永不返回的调用**（用
`--redo-hand-call` 才做）：那个调用会留下一个持有 libusb 互斥量的线程，
**连 Ctrl-C 都无法干净退出**。

---

## 五、驱动侧验收（需要 ROS，且驱动正在运行）

| 工具 | 作用 |
|---|---|
| `final_test.py` | 驱动侧最终验收测试。**第 3、4 项 FAIL 不可采信**（`/joint_states` 滞后于实际运动，见 docs/04 B1） |
| `test_motion.sh` | 让某个关节转一个增量并报告结果（带确认提示） |
| `send_joint_delta.py` | `test_motion.sh` 的底层实现 |
| `go_zero.py` | 命令到 7 姿态并报告实际停在哪 |
| `debug_pose_chain.py` | 打印完整姿态链，用正运动学判断模型此刻显示的是哪种姿态 |
| `print_joints.py` | 列出关节与 URDF 限位 |

`test_motion.sh` 把增量施加在 **URDF 空间**，由驱动自己的逆变换处理设备侧换算，
所以脚本不需要知道 move_j/angle 的偏置 —— **只有一个地方懂换算**。

---

## 六、符号与姿态分析（纯离线）

| 工具 | 作用 |
|---|---|
| `resolve_signs.py` | 三项要求联立求解标定：几何折叠 + 7 姿态读回全零 + 目标链复现实测命令 |
| `fold_signs.py` | 输出**折叠几何表** —— 真正定符号的那个检查 |
| `explore_signs.py` | 穷举 `joint_sign` 组合，按收缩姿态的末端位置排序 |
| `find_fold.py` | 按"折叠程度"扫描姿态，区分"伸出去"和"折回来" |
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
| `write_setup_assistant.py` | 生成 `.setup_assistant`（写到**包根和 config/ 两处**，见下） |
| `yaml_min.py` | 一个够用的 YAML 读取器（本机没有 PyYAML 且装不上） |

`yaml_min.py` 的理由：没有它，参数文件的错误就只能留到另一台机器上构建时才发现，
**离线检查也就失去意义**。

`write_setup_assistant.py` 写两处的原因：`MoveItConfigsBuilder` 靠这个文件推断
URDF/SRDF 位置，找不到就退回 `config/<robot>.urdf`（不存在），于是
`robot_description` 为空、**后续一切失败，而报错完全指不到原因**。Setup Assistant
存在包根，但某些版本的加载器查 `config/`。

---

## 八、环境相关

| 工具 | 作用 |
|---|---|
| `ssh_config_for_github` | GitHub 22 端口被封时改走 443 的 SSH 配置模板 |
| `setup_github_remote.ps1` | 准备 git 远端、身份、分支名（PowerShell，默认演练） |

---

## 可选清理（尚未执行，等确认）

下面这些是**排查特定假设时写的一次性脚本，问题定位后不会再被用到**。保留的
唯一价值是记录当年的排查过程。**删除前需要确认，不单方面处理。**

| 工具 | 为什么可以删 |
|---|---|
| `solve_signs.py` | **结论是错的** —— 它按位移方向推符号，被几何校验推翻。文件自身已标注 |
| `solve_downlink.py` | 一次性推导脚本，其结论已固化进驱动，并由 `check_downlink.py` 持续守着 |
| `compare_endpoints.py` | 夹爪排查专用，结论已写进 `docs/04` A2 |
| `dump_protocol.py` | 同上，协议结构已整理进 `docs/05` |
| `gripper_ids.py` | 同上 |
| `probe_relation.py` | 与 `probe_mapping.py` 职责重叠（都拟合 gain/offset） |
| `go_zero.py` | 与 `send_joint_delta.py` 职责重叠 |
| `find_fold.py` | 与 `find_pose.py`、`fold_signs.py` 职责重叠 |

**如果夹爪问题还要继续查（比如上 CAN），则第 4 组里除了 `probe_gripper.py`
和 `test_gripper.py` 之外都先留着。**

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
