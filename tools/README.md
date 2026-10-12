# tools/ — 独立工具索引

这些脚本**都不依赖 ROS 运行环境也能跑**(除了明确标注需要 ROS 的那几个),
在 Windows 和 Ubuntu 上都可以直接用 Python 执行。它们的作用是:把"靠眼睛看、
靠记忆猜"的事情变成可复现的检查。

## 每次改完代码跑这几个

| 工具 | 作用 |
|---|---|
| `check_syntax.py` | 全仓库 Python 语法检查(Windows 上改代码后必跑) |
| `check_encoding.py` | 编码 / BOM / mojibake 检查 |
| `check_urdf.py` | URDF 结构:XML 合法性、父子引用、树连通性、必需坐标系 |
| `check_line_endings.py` | 从 git 对象里检查换行符(Linux 收到的是 blob,不是工作区) |
| `fix_line_endings.ps1` | 把工作区的 CRLF 批量转回 LF(PowerShell) |
| `selftest_conversion.py` | **坐标换算自测**:锚点、往返恒等、三处默认值一致 |

`selftest_conversion.py` 最有价值。它独立复算整套换算(不读驱动代码,只读配置),
并且比对配置文件、驱动代码默认值、launch 默认值三处是否一致。**这四处曾经各自
漂移,导致下发角度错了几十度而实物不动**,所以每次改换算相关的东西都要跑它。

## 需要 ROS 环境(驱动正在运行)

| 工具 | 作用 |
|---|---|
| `debug_pose_chain.py` | 打印完整姿态链,并用正运动学判断模型此刻显示的是哪种姿态 |
| `test_motion.sh` | 让某个关节转一个增量,并报告结果(带确认提示) |
| `send_joint_delta.py` | `test_motion.sh` 的底层实现 |
| `test_gripper.py` | 夹爪开合测试,支持扫描渐变 |
| `print_joints.py` | 列出关节与 URDF 限位 |

## 纯离线分析(不需要硬件)

| 工具 | 作用 |
|---|---|
| `fk_dummy.py` | URDF 正运动学,可画 x-z 侧视 ASCII 图 |
| `find_pose.py` | 按几何描述(如"大臂竖直小臂水平")搜索匹配的关节角 |
| `compare_poses.py` | 把候选姿态与实测读数对比,报告连杆几何 |
| `solve_calibration.py` | 从"设备读数 + 声称的 URDF 姿态"配对做最小二乘,解 gain/offset |
| `verify_calibration.py` | 独立复算标定,再与配置比对 |
| `dev2urdf.py` | 设备读数与 URDF 角度互转,并对照限位 |

## 环境相关

| 工具 | 作用 |
|---|---|
| `diagnose_pkg.sh` | `ros2 launch` 找不到包时逐项排查 |
| `ssh_config_for_github` | GitHub 22 端口被封时改走 443 的 SSH 配置模板 |
| `setup_github_remote.ps1` | 准备 git 远端、身份、分支名(PowerShell,默认演练) |

## 两个反复踩过的坑

**1. `git ls-files` 的输出不能直接当路径用。** PowerShell 5.1 收到的是带八进制
转义的路径,而本仓库 `docs/` 下有中文文件名,直接用会报"路径中具有非法字符"。
相关脚本改为从文件系统枚举。

**2. 不要用 PowerShell 做字符串替换来改源文件。** `Get-Content` + `Set-Content`
的往返会重新编码,已经毁过一个源文件(UTF-8 读、ANSI 写,产生乱码和 BOM)。
需要改文件时用编辑器,或在 PowerShell 里用 `[System.IO.File]::ReadAllText` /
`WriteAllText` 配 `UTF8Encoding($false)` 做字节级操作,并且事后跑
`check_encoding.py`。
