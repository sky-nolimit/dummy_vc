"""合爪那一步的两件事：闭到多宽、以及"到底夹住没有"。

夹爪只回一个开口（毫米），没有力的回读，所以"夹住"只能靠**位置差**判：

* 手指被方块挡住 → 停在目标之外（实测 > 目标 + 一点点）；
* 没夹到东西 → 手指会走到目标，甚至被两指自己挡住而更窄。

电流其实也能读（`0x3A1`），但它的单位没标定过，只适合人肉看着比大小，
**不拿它当判据** —— 否则阈值就是个瞎猜的数，换个方块就得重调。

纯函数、不 import rclpy，Windows 上能直接跑。
"""
import math

#: 判定结果。`None` 表示"还在等"，别当成失败。
HELD = 'held'          # 手指被东西挡住了 —— 夹住了
EMPTY = 'empty'        # 手指走到目标了 —— 中间没有东西
BLIND = 'blind'        # 一直没有回读，判不了

#: 手指比目标宽多少才算"被挡住"（米）。手指停不稳，留 0.8 mm 余量。
DEFAULT_BLOCKED_M = 0.0008

#: 闭合时最少给手指留多宽（米）。两指本身有厚度，再窄就是在夹自己。
DEFAULT_MIN_CLOSE_M = 0.004

#: 闭合时比方块窄多少（米）。方块 24 mm → 闭到 22 mm。
DEFAULT_CLEARANCE_M = 0.002


def close_width(edge_m, clearance_m=DEFAULT_CLEARANCE_M, min_m=DEFAULT_MIN_CLOSE_M):
    """方块宽 `edge_m` 时，手指该闭到多宽（米）。

    闭到"比方块窄一点"而不是"正好等于方块"，因为手指要先压上去才有夹持力；
    但也不能窄太多，窄过头就是硬顶 —— 位置模式会把力一直加上去。

    夹不了的情况（方块比手指还窄、或者检测给了个荒唐值）抛 `ValueError`，
    不返回一个"看起来正常"的数：这种情况要人看一眼检测数据。
    """
    edge = float(edge_m)
    if not math.isfinite(edge):
        raise ValueError('方块宽度不是有限数：{!r}'.format(edge_m))
    if edge <= 0.0:
        raise ValueError('方块宽度必须为正：{:.6f} m'.format(edge))
    width = edge - float(clearance_m)
    if width < float(min_m):
        raise ValueError(
            '方块只有 {:.1f} mm，闭到 {:.1f} mm 会夹到手指自己（下限 {:.1f} mm）'
            .format(edge * 1e3, width * 1e3, float(min_m) * 1e3))
    return width


def verdict(target_m, measured_m, blocked_m=DEFAULT_BLOCKED_M, timed_out=False):
    """根据目标开口与实测开口判断夹没夹住。

    参数
    ----
    target_m : 下发的手指目标开口（米）
    measured_m : 最近一次回读的开口（米），没有回读就传 `None`
    blocked_m : 实测比目标宽多少才算被挡住
    timed_out : 已经等够了时间；只有这时才允许把"走到目标"判成空夹，
                否则手指可能只是还没合拢

    返回
    ----
    `HELD` / `EMPTY` / `BLIND`，或者 `None`（还在等）
    """
    if measured_m is None:
        return BLIND if timed_out else None
    if float(measured_m) > float(target_m) + float(blocked_m):
        return HELD
    if timed_out:
        return EMPTY
    return None


def describe(result, target_m, measured_m):
    """给日志用的一句话（不夹带判断，只描述）。"""
    if measured_m is None:
        return '没有回读（目标 {:.1f} mm）'.format(target_m * 1e3)
    gap = measured_m - target_m
    return '目标 {:.1f} mm，实测 {:.1f} mm（差 {:+.1f} mm）'.format(
        target_m * 1e3, measured_m * 1e3, gap * 1e3)


def stall_detected(samples, tol=3e-4, need=3):
    """最近 `need` 次位置采样几乎没变 ⇒ 顶到机械止位了。

    用在找零点那一步（`gripper_home`）：命令一个明显超过合拢点的目标，位置
    连续几次不动就说明手指已经顶死，可以停手了。`tol` 是圈，默认 3e-4 圈
    ≈ 3.5 µm 的开口变化 —— 编码器分辨力约 4.5e-5 圈，静止时的抖动在 1e-4
    以内，所以这个阈值不会把"慢速移动"误判成"停住"。

    只看**最近** `need` 个采样：早期那些"还没开始动"的点不能算进来，否则命令
    刚发出去就会被判成顶住了。所以调用方要先把"确实动过"的标志置起来再问。
    """
    window = [float(s) for s in samples[-int(need):]]
    if len(window) < int(need):
        return False
    return max(window) - min(window) <= float(tol)
