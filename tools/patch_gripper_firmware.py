#!/usr/bin/env python3
"""把 35 板（夹爪那块的 STM32F103）固件改成"丢帧不致命、总线能自愈"。

为什么改这三处
--------------
夹爪板挂在机械臂那条 CAN 上，节点号 7，帧号是 ``7 << 7 | mode``
（``0x381``/``0x385``/``0x3A3``），在这条总线上**优先级最低**：六个关节的帧号
都比它小。两帧同时开始发送时，输的总是它。

原来的固件里：

* ``AutoBusOff = DISABLE`` ——一旦进 bus-off 就**永久**停在那儿，只能断电。
* ``CAN_Send()`` 里 ``HAL_CAN_AddTxMessage() != HAL_OK`` 就 ``Error_Handler()``，
  而它等于 ``__disable_irq(); while(1);`` ——邮箱满一格就把整块板锁死。
* ``AutoRetransmission = DISABLE`` ——仲裁输掉的那一帧直接丢，不重发。

改成：

1. ``AutoBusOff = ENABLE``：总线异常能自动恢复，不用拔电。
2. ``CAN_Send`` 失败只丢掉这一帧，不再锁死整块板。
3. ``AutoRetransmission`` **保持出厂的 DISABLE**（下面这条是踩出来的）。

为什么"自动重发"不能开（2026-10-29 的教训）
-------------------------------------------
第一版补丁把这个开关也打成了 ENABLE，理由是"输掉的帧该重发"。真机上反过来：
夹爪的帧号 ``0x3A3`` 是总线上最大的，**每帧都排在最后**。主控一跑起来就按 5 ms
一轮轮询，夹爪的回复几乎每帧都要重发；重发的帧**一直占着邮箱不释放**，三个
邮箱很快全被"等着重发的旧回复"占满，于是我们真正想发的 ``0x385``/``0x381``
连邮箱都排不进去 —— 现象就是"启动驱动之后夹爪又不动了"。

出厂那个 DISABLE 才是对的：抢不到总线就把这一帧扔掉，**邮箱立刻空出来**给下
一条命令；丢的只是一帧位置回读，5 ms 后主控会再问一次。所以这一版把打过
ENABLE 的树改回 DISABLE，干净的树保持不动。

用法
----
::

    python3 tools/patch_gripper_firmware.py --firmware-root ~/dummy-35motor-fw
    python3 tools/patch_gripper_firmware.py --firmware-root ~/dummy-35motor-fw --check

``--check`` 只检查、不写文件：三处都已经是目标状态时退出码 0，否则 1（方便在
编译前确认"我烧的是改过的"）。重复运行是安全的。

编译与烧录（Ubuntu）
--------------------
::

    sudo apt install -y gcc-arm-none-eabi cmake ninja-build
    cd ~/dummy-35motor-fw
    cmake -B build -DCMAKE_BUILD_TYPE=Release -G Ninja
    cmake --build build
    # 产物：build/Ctrl-Step-STM32-fw.hex

这块板是 STM32F103（cortex-m3），不依赖 CMSIS-DSP，装上面那三个包就能编。

**烧之前先把原来的固件读出来存好**（CubeProgrammer → 连上 → 读 ``0x08000000``
0x10000 字节 → 存成 ``factory.bin``）。SWD 三根线 ``GND``/``SWDIO``/``SWCLK``，
``3.3V`` 不接。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, field
from pathlib import Path

#: 固件里要改的源文件（相对 firmware-root）
CAN_SRC = Path("Core/Src/can.c")

# (原样, 替换成, 说明)
EDITS = (
    (
        "hcan.Init.AutoBusOff = DISABLE;",
        "hcan.Init.AutoBusOff = ENABLE;",
        "bus-off 后自动恢复，不用断电",
    ),
    (
        "hcan.Init.AutoRetransmission = ENABLE;",
        "hcan.Init.AutoRetransmission = DISABLE;",
        "自动重发保持关闭：重发会占死邮箱，命令反而发不出去",
    ),
    (
        "        Error_Handler();\n"
        "    }\n"
        "}\n",
        "        /* 邮箱满就丢掉这一帧，绝不进 Error_Handler：那个函数是\n"
        "           __disable_irq(); while(1);, 一进去整块板再也发不出任何帧，\n"
        "           只能断电重启。丢一帧最多晚 5 ms 再报一次位置。 */\n"
        "        return;\n"
        "    }\n"
        "}\n",
        "发送失败只丢帧，不锁死整块板",
    ),
)


@dataclass
class Result:
    path: Path
    applied: int = 0
    already: int = 0
    missing: list = field(default_factory=list)


def apply_edits(text: str, edits=EDITS, apply: bool = True):
    """在字符串上做替换。返回 (新文本, 改了几处, 已改几处, 没找到的原样行)。"""
    applied = 0
    already = 0
    missing = []
    for old, new, _why in edits:
        if old in text:
            if apply:
                text = text.replace(old, new, 1)
            applied += 1
        elif new in text:
            already += 1
        else:
            missing.append(old.splitlines()[0].strip())
    return text, applied, already, missing


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="surrogateescape")


def patch_file(path: Path, edits=EDITS, apply: bool = True) -> Result:
    """对单个文件打补丁。

    幂等：已经是目标状态就记 already；两边都找不到记 missing（说明这棵树不是我们
    认识的版本，绝不能硬写）。
    """
    text = _read(path)
    new, applied, already, missing = apply_edits(text, edits, apply)
    if apply and applied:
        path.write_text(new, encoding="utf-8", errors="surrogateescape",
                        newline="")
    return Result(path, applied, already, missing)


def run_file(root: Path, rel: Path, apply: bool = True) -> int:
    path = root / rel
    if not path.is_file():
        print(f"错误：找不到 {path}")
        print("     --firmware-root 要指向 35 板固件的根目录"
              "（里面有 Core/Src/can.c）")
        return 2
    result = patch_file(path, EDITS, apply)
    if result.missing:
        print(f"[失败] {rel}：这几行没找到，这棵树不是 35 板那份源码")
        for line in result.missing:
            print(f"        {line}")
        return 1
    verb = "已改" if apply else "待改"
    print(f"[ok] {rel}：{verb} {result.applied} 处，已经是目标状态 "
          f"{result.already} 处")
    for _old, _new, why in EDITS:
        print(f"     - {why}")
    if not apply:
        if result.applied:
            print(f"\n还有 {result.applied} 处没改。去掉 --check 再跑一次。")
            return 1
        print("\n三处都已经是目标状态：这棵树编译出来的固件就是改过的。")
    return 0


def run(root: Path, apply: bool = True) -> int:
    return run_file(root, CAN_SRC, apply)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="把 35 板固件改成丢帧不致命、bus-off 能自愈（见模块 docstring）")
    parser.add_argument("--firmware-root", required=True, type=Path,
                        help="35 板固件根目录（含 Core/Src/can.c）")
    parser.add_argument("--check", action="store_true",
                        help="只检查，不写文件；全部已是目标状态才返回 0")
    args = parser.parse_args(argv)
    root = args.firmware_root.expanduser()
    if not root.is_dir():
        print(f"错误：{root} 不是目录")
        return 2
    return run(root, apply=not args.check)


if __name__ == "__main__":
    sys.exit(main())
