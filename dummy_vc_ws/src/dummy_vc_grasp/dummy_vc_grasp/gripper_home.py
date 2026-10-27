"""现场量零点：把手指开到合拢，用它当"开口 0"。

## 为什么需要这一步

`gripper_frames` 里那条 `开口 = 34.294 − 11.620 × 圈数` 的**截距跟着编码器零点走**。
零点在板子的 flash 里，每次重新标定都会平移一次（实测平移过 0.22 圈 ≈ 2.5 mm），
而且被手动掰过、或者换过一块板子，同一个圈数对应的开口就完全不是一回事。

所以"发多少圈"这件事**本身不可靠**。可靠的锚点只有一个：**手指完全合拢的位置**
——那是机械上限，不管之前发生过什么，它都在同一个物理位置上。量出那一刻的圈数，
之后所有命令都按"离合拢点多少圈"来算。

## 怎么量

命令一个明显超过合拢点的目标，然后一路盯着位置：位置连续几次不动了，就是顶到
合拢的机械止位了。这时候立刻发一帧"停"，别继续顶着（电机堵转要发热）。

两条保护：

* **位置连续不动就停**，不靠"转够多少圈"来猜；
* 量出来的合拢点必须落在默认值 ±`plausible_revs` 内，否则**拒绝**（说明两指之间
  卡了东西、或者板子的零点已经挪得太多，需要重新拟合常数）。

这个模块不碰 ROS，socket 由调用方传进来（`send` / `read_revs`），所以在 Windows
上也能用假的板子测整套时序。
"""
import time

from dummy_vc_grasp import gripper_frames as gf
from dummy_vc_grasp import gripper_gate as gg

#: 命令目标超过合拢点多少圈。整段行程才 4.7 圈，6 圈足够从任何位置盖过去。
DEFAULT_SPAN_REVS = 6.0

#: 位置两次采样差小于这个值就算"没动"（圈）。编码器分辨力约 4.5e-5 圈。
DEFAULT_STALL_REVS = 3e-4

#: 连续几次采样都没动，才认定顶住了。
DEFAULT_STALL_SAMPLES = 3

#: 最多盯多久（秒）。整段 4.7 圈、默认速度下应该在 2 s 内顶到。
DEFAULT_TIMEOUT_S = 8.0

#: 采样频率（Hz）。
DEFAULT_RATE_HZ = 20.0

#: 量出来的合拢点离默认值超过这么多圈就拒绝（1 圈 ≈ 11.6 mm 开口）。
DEFAULT_PLAUSIBLE_REVS = 1.0

#: 一开始就完全不动时，等这么久才认定"它本来就在止位上"。
DEFAULT_STILL_S = 1.0

#: 回读拿不到的容忍次数（连续这么多次没有回帧就放弃）。
DEFAULT_MISS_LIMIT = 10


class HomeResult(object):
    """一次找零点的结果。`ok=False` 时 `reason` 说人话。"""

    def __init__(self, ok, rev_closed=None, samples=None, reason='', target=None):
        self.ok = bool(ok)
        self.rev_closed = rev_closed if ok else None
        self.samples = list(samples or [])
        self.reason = reason
        self.target = target

    def __repr__(self):
        return 'HomeResult(ok={}, rev_closed={!r}, n={}, reason={!r})'.format(
            self.ok, self.rev_closed, len(self.samples), self.reason)


def home(send, read_revs, span_revs=DEFAULT_SPAN_REVS,
         stall_revs=DEFAULT_STALL_REVS, stall_samples=DEFAULT_STALL_SAMPLES,
         timeout_s=DEFAULT_TIMEOUT_S, rate_hz=DEFAULT_RATE_HZ,
         rev_closed_guess=gf.DEFAULT_REV_CLOSED,
         plausible_revs=DEFAULT_PLAUSIBLE_REVS, still_s=DEFAULT_STILL_S,
         miss_limit=DEFAULT_MISS_LIMIT, log=None, sleep=time.sleep,
         now=time.time):
    """把手指推到合拢止位，返回量到的圈数。

    参数
    ----
    send(frame_id, payload) : 发一帧（`gripper_frames.to_socketcan` 那套）
    read_revs() -> float|None : 读一次位置（圈），读不到返回 None

    返回 `HomeResult`。成功时 `rev_closed` 就是"开口 0"对应的圈数。
    """
    log = log or (lambda *_: None)

    rev0 = read_revs()
    if rev0 is None:
        return HomeResult(False, reason='读不到位置：板子没答，先确认 can0 UP、'
                                       '夹爪板通电、以及 `--channel` 写对了')
    target = max(float(rev0), float(rev_closed_guess)) + float(span_revs)
    log('起点 {:.4f} 圈（开口约 {:.1f} mm），往 {:.4f} 圈推'.format(
        rev0, gf.revs_to_width(rev0), target))

    send(gf.ENABLE_FRAME_ID, gf.encode_enable(True))
    send(gf.POSITION_FRAME_ID, gf.encode_position(target))

    samples = [float(rev0)]
    started = now()
    misses = 0
    moved = False
    while now() - started < float(timeout_s):
        sleep(1.0 / max(1.0, float(rate_hz)))
        rev = read_revs()
        if rev is None:
            misses += 1
            if misses >= int(miss_limit):
                send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
                return HomeResult(False, samples=samples, target=target,
                                  reason='连续 {} 次读不到位置'.format(misses))
            continue
        misses = 0
        samples.append(float(rev))
        if not moved and abs(samples[-1] - samples[0]) > 0.01:
            moved = True
        if moved and gg.stall_detected(samples, stall_revs, stall_samples):
            send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
            return _finish(samples[-1], samples, target, plausible_revs, log)
        if not moved and now() - started > float(still_s) and len(samples) > 3:
            # 一开始就完全不动：要么它本来就在止位上，要么根本没使能。
            log('推了 {:.1f} 秒位置一点都不动，按"本来就在止位上"处理'
                .format(now() - started))
            send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
            return _finish(samples[-1], samples, target, plausible_revs, log)
    send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
    return HomeResult(False, samples=samples, target=target,
                      reason='{:.1f} 秒里一直没停：是不是两指之间是空的、'
                             '而且命令没生效？'.format(float(timeout_s)))


def _finish(rev_closed, samples, target, plausible_revs, log):
    """停下来之后做一次合理性检查 —— 宁可报错，也不要拿一个错零点去算宽度。"""
    delta = float(rev_closed) - float(gf.DEFAULT_REV_CLOSED)
    if abs(delta) > float(plausible_revs):
        return HomeResult(
            False, samples=samples, target=target,
            reason='量到 {:.3f} 圈，比默认的 {:.3f} 圈偏了 {:+.3f} 圈（≈{:+.1f} mm）。'
                   '这不像"手指合拢"：要么两指之间卡着东西，要么板子的零点挪了。'
                   '把东西拿走重试；多次都偏 → 用 tools/gripper_sweep.py 重新拟合'
                   .format(rev_closed, gf.DEFAULT_REV_CLOSED, delta,
                           -delta * gf.MM_PER_REV))
    log('顶到合拢止位：{:.4f} 圈（默认 {:.4f} 圈，偏 {:+.4f} 圈 ≈ {:+.1f} mm）'.format(
        rev_closed, gf.DEFAULT_REV_CLOSED, delta, -delta * gf.MM_PER_REV))
    return HomeResult(True, rev_closed, samples, '顶到合拢止位', target)
