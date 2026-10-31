# tools/ — 只在本地

**2026-10-31 起，本目录下的工具不再随仓库分发。** 文件仍然留在磁盘上、照常能跑，
只是不进版本库（`.gitignore` 里 `tools/*`，本文件除外）。

理由：这些是探测、标定、离线检查用的脚本，与"把机械臂和夹爪跑起来"无关，
改动频繁、只在开发机上用。仓库里留下它们，等于每次都在维护两份真相。

## 现在仓库里的东西

- `dummy_vc_ws/src/`：运行时要用的节点、launch、配置
- `firmware/`：两块板的固件源码
- `README.md`、`AGENTS.md`

运行入口都在 `dummy_vc_ws/` 的 launch 与 README 里，不需要 tools/ 里的任何东西。

## 本地工具还有哪些

开发机上 `tools/` 里保留着完整的一套，按用途大致分：

| 组 | 代表 | 干什么 |
|---|---|---|
| 离线检查 | `run_checks.py`、`check_*.py`、`selftest_*.py` | 改完代码跑一遍，不需要硬件和 ROS |
| 相机/视觉标定 | `fit_vision_homography.py`、`fit_grasp_correction.py` | 用实测点拟合单应 / 偏差补偿 |
| 夹爪 | `gripper_home.py`、`gripper_*.py`、`probe_*` | 量零点、测开口、抓取实验 |
| 运动/标定核对 | `probe_pairs.py`、`probe_mapping.py`、`fk_dummy.py` | 命令与回读配对、正运动学核对 |

改完代码仍然建议跑：

```bash
C:/Users/xiaoK/.dsh/dsh-runtimes/dsh-primary-runtime/dependencies/python/python.exe tools/run_checks.py
```

（本机 PATH 上的 `python` 是 MSYS2 3.14、没有 numpy，跑不了其中依赖 numpy 的项。）

## 要把它们装回去

它们都在 git 历史里，这一批删除的上一个提交就能完整取回：

```bash
git checkout <删除前的提交> -- tools/
```

取回之后记得处理 `.gitignore` 里的 `tools/*`，否则不会进版本库。
