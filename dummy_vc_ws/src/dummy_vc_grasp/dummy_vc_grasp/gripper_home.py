"""现场量零点：把手指推到两头，用"合拢"那一头当"开口 0"。

## 为什么需要这一步

`gripper_frames` 里那条 `开口 = 34.294 − 11.620 × 圈数` 的**截距跟着编码器零点走**。
零点在板子的 flash 里，每次重新标定都会平移一次（实测平移过 0.22 圈 ≈ 2.5 mm，
2026-10-27 又遇到一次 1.08 圈 ≈ 12.5 mm），而且被手动掰过、或者换过一块板子，
同一个圈数对应的开口就完全不是一回事。

所以"发多少圈"这件事**本身不可靠**。可靠的锚点只有机械止位：**手指完全合拢**那
一点不管之前发生过什么都在同一个物理位置上。

## 怎么量（必须两头都量）

1. 往合拢方向推，位置连续几次不动 ⇒ 合拢止位，记 `rev_closed`；
2. 往张开方向推，同样判停 ⇒ 张开止位，记 `rev_open`；
3. 两个止位之间的距离是**机械行程**（55 mm ≈ 4.73 圈），与编码器零点无关。
   拿它当判据：量出来明显短了，就说明两指之间卡着东西，拒绝。

只量一头不行：一个 24 mm 的方块卡在两指之间时，"合拢止位"会被量成"方块止位"，
而这个错值看起来完全正常。只有把两头的距离和机械行程对上，才能排除这种情况。

## 这个模块不碰 ROS

socket 由调用方传进来（`send` / `read_revs`），所以在 Windows 上也能用假板子测整套
时序。
"""
import time

from dummy_vc_grasp import gripper_frames as gf
from dummy_vc_grasp import gripper_gate as gg

#: 命令目标超出止位多少圈。整段行程才 4.7 圈，6 圈足够从任何位置盖过去。
DEFAULT_SPAN_REVS = 6.0

#: 位置两次采样差小于这个值就算"没动"（圈）。编码器分辨力约 4.5e-5 圈。
DEFAULT_STALL_REVS = 3e-4

#: 连续几次采样都没动，才认定顶住了。
DEFAULT_STALL_SAMPLES = 3

#: 每一头最多盯多久（秒）。整段 4.7 圈、默认速度下应该在 2 s 内顶到。
DEFAULT_TIMEOUT_S = 8.0

#: 采样频率（Hz）。
DEFAULT_RATE_HZ = 20.0

#: 机械行程（圈）= 满开 55 mm ÷ 11.62 mm/圈 ≈ 4.733。这是**机械常数**，与编码器
#: 零点无关 —— 正因为无关，它才能用来验证两头的止位是不是真的量到了。
DEFAULT_STROKE_REVS = gf.MAX_WIDTH_MM / gf.MM_PER_REV

#: 量出来的两头距离与机械行程差多少圈就拒绝。0.4 圈 ≈ 4.6 mm：卡一个 5 mm 以上
#: 的东西就会超差；机械本身的松旷（1~2 mm）不会误判。
DEFAULT_STROKE_TOL_REVS = 0.4

#: 一开始就完全不动时，等这么久才认定"它本来就在止位上"。2.5 s 是 2026-10-29
#: 实测改的：1.0 s 太短，电机从"顶死"状态退回来、齿条背隙吃掉的那一段常常还没
#: 走完，工具就抢着判"本来就在止位上"，然后量出一个假的止位。
DEFAULT_STILL_S = 2.5

#: 回读拿不到的容忍次数（连续这么多次没有回帧就放弃）。
DEFAULT_MISS_LIMIT = 10


class HomeResult(object):
    """一次找零点的结果。`ok=False` 时 `reason` 说人话。"""

    def __init__(self, ok, rev_closed=None, rev_open=None, span_revs=None,
                 samples=None, reason=''):
        self.ok = bool(ok)
        self.rev_closed = rev_closed if ok else None
        self.rev_open = rev_open if ok else None
        self.span_revs = span_revs
        self.samples = list(samples or [])
        self.reason = reason

    def __repr__(self):
        return 'HomeResult(ok={}, rev_closed={!r}, rev_open={!r}, span={!r}, ' \
               'reason={!r})'.format(self.ok, self.rev_closed, self.rev_open,
                                     self.span_revs, self.reason)


def home(send, read_revs, span_revs=DEFAULT_SPAN_REVS,
         stall_revs=DEFAULT_STALL_REVS, stall_samples=DEFAULT_STALL_SAMPLES,
         timeout_s=DEFAULT_TIMEOUT_S, rate_hz=DEFAULT_RATE_HZ,
         rev_closed_guess=gf.DEFAULT_REV_CLOSED,
         stroke_revs=DEFAULT_STROKE_REVS,
         stroke_tol_revs=DEFAULT_STROKE_TOL_REVS, still_s=DEFAULT_STILL_S,
         miss_limit=DEFAULT_MISS_LIMIT, log=None, sleep=time.sleep,
         now=time.time, rev0=None):
    """把手指推到两头，校验行程，返回"开口 0"对应的圈数。

    参数
    ----
    send(frame_id, payload) : 发一帧（`gripper_frames.to_socketcan` 那套）
    read_revs() -> float|None : 读一次位置（圈），读不到返回 None
    rev0 : float|None
        已经读到的起点圈数。传进来就跳过这里那次读 —— 节点启动时会先拿一个
        长超时耐心读一帧（见 `gripper_can_node._attempt_home`），别在读零点
        这一步又重新赌一次 0.3 秒的窗口（2026-10-30 真机就栽在这上面）。

    返回 `HomeResult`。成功时 `rev_closed` 就是"开口 0"对应的圈数。
    """
    log = log or (lambda *_: None)

    if rev0 is None:
        rev0 = read_revs()
        if rev0 is None:
            return HomeResult(False, reason='读不到位置：板子没答，先确认 can0 UP、'
                                           '夹爪板通电、以及 `--channel` 写对了')
    log('起点 {:.4f} 圈，先往合拢方向推'.format(rev0))

    closed = _push(send, read_revs, max(float(rev0), float(rev_closed_guess))
                   + float(span_revs), float(rev0), stall_revs, stall_samples,
                   timeout_s, rate_hz, still_s, miss_limit, log, sleep, now,
                   '合拢')
    if not closed[0]:
        return HomeResult(False, samples=closed[2], reason=closed[3])
    rev_closed = closed[1]

    opened = _push(send, read_revs, rev_closed - float(span_revs), rev_closed,
                   stall_revs, stall_samples, timeout_s, rate_hz, still_s,
                   miss_limit, log, sleep, now, '张开')
    if not opened[0]:
        return HomeResult(False, rev_closed=rev_closed, samples=opened[2],
                          reason=opened[3])
    rev_open = opened[1]

    span = rev_closed - rev_open
    if span <= 0.0:
        return HomeResult(False, rev_closed=rev_closed, rev_open=rev_open,
                          span_revs=span, samples=closed[2],
                          reason='两个方向都没动（合拢 {:.3f}、张开 {:.3f}）。'
                                 '这有两种可能，得用手分一下：①手指本来就顶在'
                                 '合拢止位上，再往合拢推当然不动——这时手能把'
                                 '手指掰开，掰开后再跑一次就行；②命令根本没生效'
                                 '（板子又卡住了）——手推要很用力才动、或者干脆'
                                 '推不动，而 `cansend can0 3A3#0000000000000000` '
                                 '之后回帧里那 4 个字节也不变。'.format(
                                     rev_closed, rev_open))
    if abs(span - float(stroke_revs)) > float(stroke_tol_revs):
        return HomeResult(
            False, rev_closed=rev_closed, rev_open=rev_open, span_revs=span,
            samples=closed[2],
            reason='两头只差 {:.3f} 圈 ≈ {:.1f} mm，机械行程应该是 {:.1f} mm'
                   '（{:.3f} 圈）。短了这么多，只可能是两指之间卡着东西'
                   .format(span, span * gf.MM_PER_REV,
                           float(stroke_revs) * gf.MM_PER_REV,
                           float(stroke_revs)))

    log('合拢 {:.4f} 圈、张开 {:.4f} 圈，行程 {:.3f} 圈 ≈ {:.1f} mm（机械 '
        '{:.1f} mm）'.format(rev_closed, rev_open, span,
                            span * gf.MM_PER_REV,
                            float(stroke_revs) * gf.MM_PER_REV))
    return HomeResult(True, rev_closed, rev_open, span, closed[2],
                      '两头的止位都对上了')


def _push(send, read_revs, target, rev0, stall_revs, stall_samples, timeout_s,
          rate_hz, still_s, miss_limit, log, sleep, now, label):
    """朝 `target` 推，直到位置连续几次不动。

    返回 `(ok, rev_final, samples, reason)`。开头就把使能补一帧 —— 主控的
    `power_on()` 会发 0x381 带 float，板子收到就 MODE_STOP。
    """
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
                return (False, None, samples,
                        '{}这一头连续 {} 次读不到位置'.format(label, misses))
            continue
        misses = 0
        samples.append(float(rev))
        if not moved and abs(samples[-1] - samples[0]) > 0.01:
            moved = True
        if moved and gg.stall_detected(samples, stall_revs, stall_samples):
            send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
            log('{}止位：{:.4f} 圈'.format(label, samples[-1]))
            return True, samples[-1], samples, ''
        if not moved and now() - started > float(still_s) and len(samples) > 3:
            log('{}方向推了 {:.1f} 秒一点都不动，按"本来就在止位上"处理'
                .format(label, now() - started))
            send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
            return True, samples[-1], samples, ''
    send(gf.ENABLE_FRAME_ID, gf.encode_enable(False))
    return (False, None, samples,
            '{}方向 {:.1f} 秒里一直没停：命令没生效？'.format(
                label, float(timeout_s)))
