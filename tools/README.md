# tools/ — 独立工具索引

把"靠眼睛看、靠记忆猜"的事情变成可复现的检查。除标注"需要 ROS"的以外,
都不依赖 ROS 环境,Windows 和 Ubuntu 上都可直接用 Python 执行。

**目前 23 个工具。** 曾经一度有 43 个,其中 19 个是排查某个具体假设时写的一次性
探测脚本 —— 问题定位之后它们不会再被用到,已删除(记录见
`docs/03-进展与问题报告.md` 第五节)。**新增工具前先确认它不与现有工具重叠**,
保留多个职责相同的工具是本项目反复出错的来源之一。

## 每次改完代码跑这几个

| 工具 | 作用 |
|---|---|
| `check_syntax.py` | 全仓库 Python 语法检查(Windows 上改代码后必跑) |
| `check_encoding.py` | 编码 / BOM / mojibake 检查(BOM 会让 Linux 上的 shebang 失效) |
| `check_urdf.py` | URDF 结构:XML 合法性、父子引用、树连通性、必需坐标系 |
| `check_line_endings.py` | 从 git 对象里检查换行符(Linux 收到的是 blob,不是工作区) |
| `fix_line_endings.ps1` | 把工作区的 CRLF 批量转回 LF(PowerShell) |
| `check_launch_consistency.py` | 校验 launch 不会因"配置参数缺 DESCRIPTION"而在启动时抛错 |
| `check_conversions.py` | **导入真实驱动代码**,验证换算函数两两互逆、快捷式与两步链一致 |
| `selftest_conversion.py` | **标定自测**:锚点、收缩姿态折叠、往返恒等、三处默认值一致 |

后两个最有价值。

`selftest_conversion.py` 独立复算整套标定(不读驱动代码,只读配置),校验四项:

1. **锚点** —— 7 姿态下 URDF 必须全零,move_j 必须是 `[0,0,90,...]`
2. **收缩姿态** —— 断电收缩必须**折叠**(小臂折回叠在大臂上)。这是唯一独立于
   符号选择的校验:收缩时设备读数约全零,所以 `urdf_retract = sign * (0 - offset)`,
   **符号错了就会破坏这一项**
3. **往返恒等** —— URDF → move_j → URDF,以及 URDF → angle → URDF
4. **三处默认值一致** —— 配置文件、驱动代码默认值、launch 默认值

第 4 项尤其重要:这三处**曾经各自漂移,导致下发角度错了几十度而实物完全不动**。

`check_conversions.py` 能抓到"公式改错了、但函数仍与自己互逆"这类问题 ——
复刻公式的自测抓不到,因为它测的是同一份错误。

```bash
python tools/selftest_conversion.py
python tools/check_conversions.py
python tools/check_launch_consistency.py
```

## 设备行为探测(需要硬件,且必须先停掉驱动)

同一时刻只有一个进程能持有 fibre 链路,所以跑这些之前要:

```bash
pkill -f dummy_controller
```

| 工具 | 作用 |
|---|---|
| `probe_pairs.py` | **记录 (命令, 回读) 配对** —— 标定的权威数据来源。两个命名姿态 + 逐关节扫描,结果存 JSON。`--refs-only` 只走两个姿态,很快 |
| `probe_relation.py` | 单关节最小二乘拟合 `raw = a·move_j + b`,从数据里读出 gain / offset / sign |
| `resolve_signs.py` | **三项要求联立求解标定**:几何折叠 + 7 姿态读回全零 + 目标链复现实测命令 |
| `fold_signs.py` | 穷举 j2/j3 的符号,用真实 URDF 做正运动学判断收缩姿态是否真的折叠 |
| `analyse_range.py` | 归类 `probe_range` 的输出,区分"区间外被拒"与"等于基准位姿的空操作" |
| `probe_mapping.py` | 逐个关节测量"命令值 ↔ 回读值"的 gain 与 offset。**offset 必须从设备读出来,不能靠推导** —— 实测 j3 的 gain 是 +1、offset 是 −180 |
| `probe_range.py` | 扫出设备**实际接受**的 move_j 取值区间。区间外的值会被**静默丢弃**(整条命令不执行),这是"下发成功但机械臂不动"的直接原因 |
| `trace_control.py` | 用具体例子(如"j3 转 30 度")逐步打印整条控制链路的每个数值及其来历 |
| `solve_signs.py` | **已废弃的符号求解思路,保留作记录** —— 它按位移方向推符号,结论是错的,被几何校验推翻。其 joint_offset 部分仍有效 |

## 需要 ROS 环境(驱动正在运行)

| 工具 | 作用 |
|---|---|
| `debug_pose_chain.py` | 打印完整姿态链,并用正运动学判断模型此刻显示的是哪种姿态 |
| `test_motion.sh` | 让某个关节转一个增量,并报告结果(带确认提示) |
| `send_joint_delta.py` | `test_motion.sh` 的底层实现 |
| `test_gripper.py` | 夹爪开合测试,支持 `--sweep` 扫描渐变 |
| `print_joints.py` | 列出关节与 URDF 限位 |

`test_motion.sh` 把增量施加在 **URDF 空间**,由驱动自己的逆变换处理设备侧换算,
所以脚本不需要知道 move_j/angle 的偏置 —— **只有一个地方懂换算**。

## 符号与姿态分析(纯离线)

| 工具 | 作用 |
|---|---|
| `explore_signs.py` | 穷举 `joint_sign` 组合,按收缩姿态的末端位置排序 |
| `find_fold.py` | 按"折叠程度"扫描姿态,区分"伸出去"和"折回来" |
| `find_pose.py` | 按几何描述(如"大臂竖直小臂水平")搜索匹配的关节角 |
| `fk_dummy.py` | URDF 正运动学,可画 x-z 侧视 ASCII 图 |
| `solve_j3.py` | 由实测映射反解 j3 的标定参数,并说明为什么 sign 不能由读数反推 |

判定符号用 `explore_signs.py`。它的价值在于把"符号对不对"变成"认出哪个几何对",
而不是靠推理 —— **注意比较时看绝对方向,不要只看两段之间的夹角**,因为镜像的
折叠在夹角上是一样的。

## 环境相关

| 工具 | 作用 |
|---|---|
| `ssh_config_for_github` | GitHub 22 端口被封时改走 443 的 SSH 配置模板 |
| `setup_github_remote.ps1` | 准备 git 远端、身份、分支名(PowerShell,默认演练) |

## 四个反复踩过的坑

**1. `git ls-files` 的输出不能直接当路径用。** PowerShell 5.1 收到的是带八进制
转义的路径,而本仓库 `docs/` 下有中文文件名,直接用会报"路径中具有非法字符"。
相关脚本改为从文件系统枚举。

**2. 不要用 PowerShell 做字符串替换来改源文件。** `Get-Content` + `Set-Content`
的往返会重新编码,已经毁过一个源文件(UTF-8 读、ANSI 写,产生乱码和 BOM)。
需要改文件时用编辑器,或在 PowerShell 里用 `[System.IO.File]::ReadAllText` /
`WriteAllText` 配 `UTF8Encoding($false)` 做字节级操作,并且事后跑
`check_encoding.py`。**即使这样,含引号的 Python 字面量仍极易写坏** —— 本项
目至少三次把 launch 文件的多行字符串引号弄错,应改用编辑工具而非批量替换。

**3. `git rm` 遇到未跟踪文件会整体失败。** 一次删多个文件时,其中任何一个没被
跟踪,`git rm` 就会报 pathspec 错误并中止,**一个都不删**。删之前先用
`git ls-files --error-unmatch` 逐个确认跟踪状态,已跟踪的用 `git rm`,未跟踪的
用 `Remove-Item`。

**4. 探测脚本本身也会错,而且错得更隐蔽。** 本项目多次出现"脚本报 NOOP、实际是
脚本读得太早或基准构造有误",进而把人引向错误的结论。写探测脚本时:等待要用
**轮询到位**而不是固定 sleep;基准要用**显式命令的绝对位姿**,不要用"刚读到的
值";输出要**同时打印下发的值和读回的值**,让人能自己核对。
