# tools/ — 独立工具索引

把"靠眼睛看、靠记忆猜"的事情变成可复现的检查。除标注"需要 ROS"或
"需要 numpy"的以外,都不依赖 ROS 环境,Windows 和 Ubuntu 上都可直接用
Python 执行。

**目前 18 个工具。** 曾经有 26 个,其中 8 个是为已修复的问题写的一次性脚本、
或是职责重叠的中间产物,已删除。**新增工具前先确认它不与现有工具重叠**——
保留多个职责相同的工具是本项目反复出错的来源之一。

## 每次改完代码跑这几个

| 工具 | 作用 |
|---|---|
| `check_syntax.py` | 全仓库 Python 语法检查(Windows 上改代码后必跑) |
| `check_encoding.py` | 编码 / BOM / mojibake 检查 |
| `check_urdf.py` | URDF 结构:XML 合法性、父子引用、树连通性、必需坐标系 |
| `check_line_endings.py` | 从 git 对象里检查换行符(Linux 收到的是 blob,不是工作区) |
| `fix_line_endings.ps1` | 把工作区的 CRLF 批量转回 LF(PowerShell) |
| `selftest_conversion.py` | **坐标换算 + 标定自测**,见下 |
| `check_conversions.py` | 换算函数自洽性——**导入真实驱动**逐个验证互为逆。**需要 numpy**(Linux 侧跑) |

`selftest_conversion.py` 是最有价值的一个,它独立复算整套换算(不读驱动代码,
只读配置),校验四项:

1. **锚点** —— 7 姿态下 URDF 必须全零,move_j 必须是 `[0,0,90,...]`
2. **收缩姿态** —— 断电收缩必须**折叠**(小臂折回叠在大臂上)。这是唯一
   独立于符号选择的校验:收缩时设备读数约全零,所以
   `urdf_retract = sign * (0 - offset)`,**符号错了就会破坏这一项**
3. **往返恒等** —— URDF → move_j → URDF,以及 URDF → angle → URDF
4. **默认值不重复** —— 驱动代码默认值与配置一致、launch **确实从配置读**默认值、
   launch 里没有第二份硬编码副本、配置里每个参数都有 launch 说明

第 4 项尤其重要:标定值**曾经同时写在三处**(配置 / 驱动代码 / launch),
它们各自漂移,而且 launch 那份会在运行时**覆盖**配置,导致"改了配置文件却
毫无效果"。改任何换算相关的参数后都要跑它。

```bash
python tools/selftest_conversion.py
```

`check_conversions.py` 与它**互补,不重叠**,两者的失效模式正好相反:

- `selftest_conversion.py` **不读驱动代码**,只从配置独立复算。所以驱动里写错
  公式它永远发现不了——这正是 `angle_to_movej` 漏掉 `joint_offset_deg` 项
  (7 姿态下 j2 差 74.6°、j3 差 91.5°)却在自测里全绿的原因。
- `check_conversions.py` **导入真实驱动类**,校验各换算两两互为逆、且
  "走中间空间"与"走直达函数"结果一致。**互为逆运算会互相掩盖错误**
  (两个都漏同一项时它们仍是彼此的逆),所以关键在它最后那项交叉验证。

它 import numpy,在这台 Windows 上会 `ModuleNotFoundError`,**到 Linux 侧跑**。
```bash
python3 tools/check_conversions.py
```

## 符号与姿态分析(纯离线)

| 工具 | 作用 |
|---|---|
| `explore_signs.py` | 穷举 `joint_sign` 组合,按收缩姿态的末端位置排序,用来判定符号 |
| `find_fold.py` | 按"折叠程度"扫描姿态,区分"伸出去"和"折回来" |
| `find_pose.py` | 按几何描述(如"大臂竖直小臂水平")搜索匹配的关节角 |
| `fk_dummy.py` | URDF 正运动学,可画 x-z 侧视 ASCII 图 |

判定符号用 `explore_signs.py`。它的价值在于把"符号对不对"变成"认出哪个几何
对",而不是靠推理——**注意比较时看绝对方向,不要只看两段之间的夹角**,
因为镜像的折叠在夹角上是一样的。

## 需要 ROS 环境(驱动正在运行)

| 工具 | 作用 |
|---|---|
| `debug_pose_chain.py` | 打印完整姿态链,并用正运动学判断模型此刻显示的是哪种姿态 |
| `test_motion.sh` | 让某个关节转一个增量,并报告结果(带确认提示) |
| `send_joint_delta.py` | `test_motion.sh` 的底层实现 |
| `test_gripper.py` | 夹爪开合测试,支持 `--sweep` 扫描渐变 |
| `print_joints.py` | 列出关节与 URDF 限位 |

`test_motion.sh` 把增量施加在 **URDF 空间**,由驱动自己的逆变换处理设备侧
换算。这样脚本不需要知道 move_j/angle 的偏置,**只有一个地方懂换算**。

## 环境相关

| 工具 | 作用 |
|---|---|
| `ssh_config_for_github` | GitHub 22 端口被封时改走 443 的 SSH 配置模板 |
| `setup_github_remote.ps1` | 准备 git 远端、身份、分支名(PowerShell,默认演练) |

## 三个反复踩过的坑

**1. `git ls-files` 的输出不能直接当路径用。** PowerShell 5.1 收到的是带八进制
转义的路径,而本仓库 `docs/` 下有中文文件名,直接用会报"路径中具有非法字符"。
相关脚本改为从文件系统枚举。

**2. 不要用 PowerShell 做字符串替换来改源文件。** `Get-Content` + `Set-Content`
的往返会重新编码,已经毁过一个源文件(UTF-8 读、ANSI 写,产生乱码和 BOM)。
需要改文件时用编辑器,或在 PowerShell 里用 `[System.IO.File]::ReadAllText` /
`WriteAllText` 配 `UTF8Encoding($false)` 做字节级操作,并且事后跑
`check_encoding.py`。

**3. `git rm` 遇到未跟踪文件会整体失败。** 如果一次删多个文件,其中任何一个
没被跟踪,`git rm` 会报 pathspec 错误并中止,**一个都不删**。删之前先用
`git ls-files --error-unmatch` 逐个确认跟踪状态,已跟踪的用 `git rm`,
未跟踪的用 `Remove-Item`。
