#!/usr/bin/env python3
"""给主控板固件(dummy-ref-core-fw)打补丁：让一帧发不出去时不再拖死整机。

## 为什么

`Bsp/communication/interface_can.cpp:237-249` 的 `CanSendMessage()` 是这么写的：

    semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    ...
    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);

而归还这个信号量的地方只有一处：`tx_complete_callback()`（发送完成中断）。
于是只要有一帧没能"发完"——邮箱满、仲裁输掉（`AutoRetransmission` 还是 DISABLE）、
没有节点回 ACK——信号量就永久丢失。下一次 `CanSendMessage()` 会永远卡在 `Acquire` 上，
而六个关节的周期更新、USB 协议应答全都走这条路径，整机连同 USB 一起失联，只能断电重启。

2026-10-22 的实测就是这么死的：调一次 `robot.hand.set_angle()` 之后整块主板不答 USB
发现，拔不拔夹爪都一样。

## 打哪三处

1. `interface_can.cpp` 的 `CanSendMessage()`：等信号量改成 10 ms 超时，超时就丢掉这一帧
   （帧都是周期性的，丢一帧无所谓）；`HAL_CAN_AddTxMessage()` 返回非 HAL_OK 时立刻把
   信号量还回去（那一帧不会有完成回调）。
2. `interface_can.cpp` 的 `tx_aborted_callback()`：被中止的帧同样不会有完成回调，
   也要在这里归还。
3. `Core/Src/can.c` 的 CAN1/CAN2 初始化：只把 `AutoBusOff` 改成 ENABLE（进了 bus-off
   能自己退出来），**`AutoRetransmission` 保持 DISABLE**。

   ⚠️ 2026-10-29 的教训：v1 曾经把 NART 也打开，结果真机上机械臂启动到一半就停在
   一个奇怪的姿态（j3 差 54°），`ctrl+c` 也不回收缩。原因是 NART 一开，发往一个
   **不响应节点**的帧会永远重发、把邮箱占死；`HAL_CAN` 只有 3 个发送邮箱，被占满
   之后所有帧都排队失败 —— 我加的超时就把它们全丢了，于是"通电、能读角度、就是不
   动"。NART 关掉时硬件会把发不出去的帧直接丢掉、邮箱立刻释放，这才是安全的做法。
   本工具会自动把 v1 打过的树改回来。
4. `CMakeLists.txt`：`link_libraries("arm_cortexM4lf_math.a")` 少了 `lib` 前缀
   （仓库里的文件叫 `Drivers/CMSIS/Lib/libarm_cortexM4lf_math.a`），链接会失败。
   改成给全路径。顺带说明：`6dof_kinematic.cpp` 用了 `arm_sin_f32` / `arm_cos_f32`，
   这个库是真需要的，不能删。

## 用法

    python3 tools/patch_main_firmware.py --firmware-root ~/dummy-ref-core-fw --check
    python3 tools/patch_main_firmware.py --firmware-root ~/dummy-ref-core-fw

`--check` 只报告还需要改几处，不写文件。幂等：已经打过补丁的树再跑一次会说"没有要改的"。
认不出的源码树（找不到那几段代码）会直接拒绝，不会半途写坏文件。

## 不改什么

- 不改 `DummyHand` 的三帧协议（那是第二步，要动协议就得同步改 ROS 驱动）。
- 不碰 flash 里的参数页。
- 不动看门狗（第三步）。
"""

import argparse
import os
import sys

CAN_CPP = os.path.join('Bsp', 'communication', 'interface_can.cpp')
CAN_INIT = os.path.join('Core', 'Src', 'can.c')
CMAKE = 'CMakeLists.txt'

SEND_OLD = """    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, osWaitForever);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, osWaitForever);
    else
        return;

    if (semaphore_status == osOK)
        HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData, &canCtx->last_heartbeat_mailbox);
"""

SEND_NEW = """    /* 2026-10-28: 一帧发不出去时不许把整机拖死。
     * 原来用 osWaitForever 拿这个信号量，而归还它的只有 tx_complete_callback 一处；
     * 只要有一帧没发完(邮箱满 / 仲裁丢 / 没有 ACK)，信号量就永久丢失，之后所有
     * CanSendMessage 都卡在 Acquire 上 —— 关节更新、协议应答、USB 服务一起停，只能断电。
     * 现在：50 ms 拿不到就丢掉这一帧(帧都是周期性的)；排队失败立刻还回去。
     * 50 ms 而不是更短：50 Hz 的轨迹流一波 7 帧只占 ~1 ms 总线，正常绝不会超时，
     * 真超时了就说明总线上有东西卡住了，这时候丢帧比死等好。 */
    const uint32_t CAN_TX_TIMEOUT_TICKS = 50;
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, CAN_TX_TIMEOUT_TICKS);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, CAN_TX_TIMEOUT_TICKS);
    else
        return;

    if (semaphore_status != osOK)
        return;

    if (HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData,
                             &canCtx->last_heartbeat_mailbox) != HAL_OK)
    {
        /* 没排上队：不会有完成回调，这个信号量必须当场还回去 */
        if (canCtx->handle->Instance == CAN1)
            osSemaphoreRelease(sem_can1_tx);
        else if (canCtx->handle->Instance == CAN2)
            osSemaphoreRelease(sem_can2_tx);
    }
"""

# v1(2026-10-28) 打过的树里 `CanSendMessage` 是这一段（10 ms 超时、注释也短）。
# v2 把整段换成 SEND_NEW，所以先认这一段。
SEND_NEW_V1 = """    /* 2026-10-28: 一帧发不出去时不许把整机拖死。
     * 原来用 osWaitForever 拿这个信号量，而归还它的只有 tx_complete_callback 一处；
     * 只要有一帧没发完(邮箱满 / 仲裁丢 / 没有 ACK)，信号量就永久丢失，之后所有
     * CanSendMessage 都卡在 Acquire 上 —— 关节更新、协议应答、USB 服务一起停，只能断电。
     * 现在：10 ms 拿不到就丢掉这一帧(帧都是周期性的)；排队失败立刻还回去。 */
    const uint32_t CAN_TX_TIMEOUT_TICKS = 10;
    osStatus semaphore_status;
    if (canCtx->handle->Instance == CAN1)
        semaphore_status = osSemaphoreAcquire(sem_can1_tx, CAN_TX_TIMEOUT_TICKS);
    else if (canCtx->handle->Instance == CAN2)
        semaphore_status = osSemaphoreAcquire(sem_can2_tx, CAN_TX_TIMEOUT_TICKS);
    else
        return;

    if (semaphore_status != osOK)
        return;

    if (HAL_CAN_AddTxMessage(canCtx->handle, txHeader, txData,
                             &canCtx->last_heartbeat_mailbox) != HAL_OK)
    {
        /* 没排上队：不会有完成回调，这个信号量必须当场还回去 */
        if (canCtx->handle->Instance == CAN1)
            osSemaphoreRelease(sem_can1_tx);
        else if (canCtx->handle->Instance == CAN2)
            osSemaphoreRelease(sem_can2_tx);
    }
"""


ABORT_OLD = """void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;
}
"""

ABORT_NEW = """void tx_aborted_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (!get_can_ctx(hcan))
        return;
    get_can_ctx(hcan)->TxMailboxAbortCallbackCnt++;

    /* 2026-10-28: 被中止的帧不会有完成回调，信号量要在这里还，否则同样永久丢失。
     * 同一个邮箱的完成回调和中止回调互斥，所以不会重复归还。 */
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
    else if (hcan->Instance == CAN2)
        osSemaphoreRelease(sem_can2_tx);
}
"""

# 只替换子串，缩进原样保留。
# 第 4 个元素是 optional：新树上这些改动本来就不存在，别当成"找不到锚点"报错。
INIT_EDITS = [
    ('can1.autobusoff', 'hcan1.Init.AutoBusOff = DISABLE;',
     'hcan1.Init.AutoBusOff = ENABLE;'),
    ('can2.autobusoff', 'hcan2.Init.AutoBusOff = DISABLE;',
     'hcan2.Init.AutoBusOff = ENABLE;'),
    # v1 把自动重发打开了（那是错的，见文件头），这里改回去
    ('can1.autoretx-revert', 'hcan1.Init.AutoRetransmission = ENABLE;',
     'hcan1.Init.AutoRetransmission = DISABLE;', True),
    ('can2.autoretx-revert', 'hcan2.Init.AutoRetransmission = ENABLE;',
     'hcan2.Init.AutoRetransmission = DISABLE;', True),
]

EDITS = {
    CAN_CPP: [
        ('can.v1-timeout-bump', SEND_NEW_V1, SEND_NEW, True),
        ('can.send-timeout', SEND_OLD, SEND_NEW),
        ('can.abort-release', ABORT_OLD, ABORT_NEW),
    ],
    CAN_INIT: INIT_EDITS,
    CMAKE: [
        ('cmake.dsp-lib-path',
         'link_libraries("arm_cortexM4lf_math.a")',
         '# 2026-10-28: 这一行原来写的是 "arm_cortexM4lf_math.a"，但仓库里的文件叫\n'
         '# libarm_cortexM4lf_math.a（在 Drivers/CMSIS/Lib/ 下），少了 lib 前缀链接器\n'
         '# 就找不到，链接会报 cannot find -larm_cortexM4lf_math.a。改成给全路径。\n'
         '# 6dof_kinematic.cpp 用了 arm_sin_f32 / arm_cos_f32，所以这个库是真需要的。\n'
         'link_libraries(${CMAKE_SOURCE_DIR}/Drivers/CMSIS/Lib/libarm_cortexM4lf_math.a)'),
    ],
}


def read_text(path):
    """按二进制读、统一成 \\n，并告诉调用方这个文件原本是不是 CRLF。"""
    with open(path, 'rb') as f:
        raw = f.read()
    crlf = b'\r\n' in raw
    return raw.decode('utf-8').replace('\r\n', '\n'), crlf


def write_text(path, text, crlf):
    """按原文件的换行风格写回（不把一个 CRLF 文件改成 LF）。"""
    data = text.replace('\n', '\r\n') if crlf else text
    with open(path, 'wb') as f:
        f.write(data.encode('utf-8'))


def apply_edits(text, edits, apply=True):
    """在字符串上做替换，返回 (新文本, 没找到的补丁名列表)。

    补丁项可以是 `(名字, 旧, 新)` 或 `(名字, 旧, 新, optional)`。optional 的项
    在新树上本来就找不到锚点，不算失败（比如"把 v1 打开的重发改回去"）。

    apply 参数只为兼容保留：**总是算出结果**，写不写由调用方决定。
    （早先的写法是 apply=False 就不改文本，于是 --check 永远报告"没有要改的"，
    这个 bug 已经踩过一次。）
    """
    missing = []
    out = text
    for edit in edits:
        name, old, new = edit[0], edit[1], edit[2]
        optional = len(edit) > 3 and edit[3]
        if old in out:
            out = out.replace(old, new, 1)
            continue
        if new in out:
            continue  # 已经打过
        if optional:
            continue
        missing.append(name)
    return out, missing


def count_pending(text, edits):
    """还有多少处真的会改（不含已经打过的、也不含 optional 且本来就不存在的）。"""
    return sum(1 for edit in edits if edit[1] in text)


def patch_file(path, edits, apply=True):
    """返回 (状态, 没找到的补丁名)。状态是 'patched' / 'already' / 'missing' / 'missing-file'。"""
    if not os.path.isfile(path):
        return 'missing-file', [edit[0] for edit in edits]
    text, crlf = read_text(path)
    new_text, missing = apply_edits(text, edits)
    if missing:
        return 'missing', missing
    if new_text == text:
        return 'already', []
    if apply:
        write_text(path, new_text, crlf)
    return 'patched', []


def run_file(root, rel, apply=True, edits=None):
    edits = EDITS[rel] if edits is None else edits
    status, missing = patch_file(os.path.join(root, rel), edits, apply=apply)
    return status, missing


def run(root, apply=True, edits_map=None):
    """返回 (退出码, 报告行列表)。edits_map 只给自测用（扁平布局）。"""
    edits_map = EDITS if edits_map is None else edits_map
    lines = []
    rc = 0
    for rel in edits_map:
        status, missing = run_file(root, rel, apply=apply, edits=edits_map[rel])
        if status == 'patched':
            if apply:
                lines.append('%-42s 已改' % rel)
            else:
                full = os.path.join(root, rel)
                text, _ = read_text(full)
                lines.append('%-42s 还有 %d 处没改'
                             % (rel, count_pending(text, edits_map[rel])))
                rc = 1
        elif status == 'already':
            lines.append('%-42s 没有要改的' % rel)
        elif status == 'missing-file':
            lines.append('%-42s 找不到这个文件' % rel)
            rc = 2
        else:
            for name in missing:
                lines.append('%-42s 找不到要改的代码：%s' % (rel, name))
            rc = 1
    return rc, lines


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='给主控板固件打补丁：一帧发不出去时不再拖死整机')
    parser.add_argument('--firmware-root', required=True,
                        help='主控固件源码根目录（里面有 Bsp/ 和 Core/）')
    parser.add_argument('--check', action='store_true',
                        help='只检查，不写文件（退出码 1 表示还有地方要改）')
    args = parser.parse_args(argv)

    if not os.path.isdir(args.firmware_root):
        print('没有这个目录：%s' % args.firmware_root)
        return 2

    rc, lines = run(args.firmware_root, apply=not args.check)
    for line in lines:
        print(line)
    if args.check and rc == 0:
        print('补丁都打上了。')
    return rc


if __name__ == '__main__':
    sys.exit(main())
