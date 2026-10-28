#!/usr/bin/env python3
"""自测：tools/patch_main_firmware.py 只改它认识的那几处，且能重复跑。

和 selftest_patch_gripper_firmware 一个风格。沙箱只允许往 reference/ 里直接写文件、
建不了子目录，所以文件层面的夹具用固定名字，用完删掉。

如果本机有主控固件源码树（reference/Firmware/dummy-ref-core-fw，被 .gitignore），
就额外对着真文件验一遍——那是"锚点写对了没有"的唯一硬证据。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import patch_main_firmware as pmf  # noqa: E402

HEADER = """\
#include "interface_can.h"

void tx_complete_callback(CAN_HandleTypeDef* hcan, uint8_t mailbox_idx)
{
    if (hcan->Instance == CAN1)
        osSemaphoreRelease(sem_can1_tx);
}

"""

CAN_C = """\
void MX_CAN1_Init(void)
{
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 7;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = ENABLE;
}

void MX_CAN2_Init(void)
{
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 7;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
}
"""

SEND_SIGNATURE = ("void CanSendMessage(CAN_context* canCtx, uint8_t* txData, "
                  "CAN_TxHeaderTypeDef* txHeader)\n{\n")

TMP_ROOT = Path(__file__).resolve().parent.parent / "reference"
REAL_ROOT = TMP_ROOT / "Firmware" / "dummy-ref-core-fw"

FLAT = {
    "selftest_main_interface_can.cpp": pmf.EDITS[pmf.CAN_CPP],
    "selftest_main_can.c": pmf.EDITS[pmf.CAN_INIT],
}


def fixture_can_cpp() -> str:
    return (HEADER + pmf.ABORT_OLD + "\n"
            + SEND_SIGNATURE + pmf.SEND_OLD + "}\n")


def write_fixtures(cpp_text, c_text):
    cpp = TMP_ROOT / "selftest_main_interface_can.cpp"
    init = TMP_ROOT / "selftest_main_can.c"
    cpp.write_text(cpp_text, encoding="utf-8", newline="")
    init.write_text(c_text, encoding="utf-8", newline="")
    return cpp, init


def drop_fixtures():
    for name in FLAT:
        try:
            (TMP_ROOT / name).unlink()
        except FileNotFoundError:
            pass


def check_applies() -> str:
    text, missing = pmf.apply_edits(fixture_can_cpp(), pmf.EDITS[pmf.CAN_CPP], apply=True)
    assert missing == [], missing
    body = text.split("void CanSendMessage")[1]
    assert "osSemaphoreAcquire(sem_can1_tx, osWaitForever)" not in body, body
    assert "CAN_TX_TIMEOUT_TICKS" in body
    assert "!= HAL_OK" in body, body
    assert body.count("osSemaphoreRelease") == 2, body
    abort_body = text.split("void tx_aborted_callback")[1].split("void tx_error")[0]
    assert "osSemaphoreRelease(sem_can1_tx)" in abort_body, abort_body
    return "send 与 abort 两处都改对了"


def check_init_flags() -> str:
    text, missing = pmf.apply_edits(CAN_C, pmf.EDITS[pmf.CAN_INIT], apply=True)
    assert missing == [], missing
    assert "hcan1.Init.AutoBusOff = ENABLE;" in text
    assert "hcan1.Init.AutoRetransmission = ENABLE;" in text
    assert "hcan2.Init.AutoBusOff = ENABLE;" in text
    assert "hcan2.Init.AutoRetransmission = ENABLE;" in text
    assert "hcan1.Init.Prescaler = 7;" in text
    assert "hcan1.Init.TimeTriggeredMode = DISABLE;" in text
    assert "hcan1.Init.TransmitFifoPriority = ENABLE;" in text
    assert "hcan2.Init.TransmitFifoPriority = DISABLE;" in text
    assert text.count("AutoBusOff = ENABLE;") == 2
    return "四个初始化标志都翻了，其它项没动"


def check_idempotent() -> str:
    once, missing = pmf.apply_edits(fixture_can_cpp(), pmf.EDITS[pmf.CAN_CPP], apply=True)
    assert missing == [], missing
    twice, missing2 = pmf.apply_edits(once, pmf.EDITS[pmf.CAN_CPP], apply=True)
    assert missing2 == []
    assert twice == once
    init_once, _ = pmf.apply_edits(CAN_C, pmf.EDITS[pmf.CAN_INIT], apply=True)
    init_twice, _ = pmf.apply_edits(init_once, pmf.EDITS[pmf.CAN_INIT], apply=True)
    assert init_twice == init_once
    return "跑两遍结果一样"


def check_refuses_unknown_tree() -> str:
    try:
        cpp, init = write_fixtures("// 别的东西\n", "int main(void) { return 0; }\n")
        rc, lines = pmf.run(str(TMP_ROOT), apply=True, edits_map=FLAT)
        assert rc == 1, (rc, lines)
        assert any("找不到要改的代码" in line for line in lines), lines
        assert init.read_text(encoding="utf-8") == "int main(void) { return 0; }\n"
        assert cpp.read_text(encoding="utf-8") == "// 别的东西\n"
    finally:
        drop_fixtures()
    return "认不出的树直接拒绝，文件没被写"


def check_file_and_check_mode() -> str:
    try:
        cpp, init = write_fixtures(fixture_can_cpp(), CAN_C)

        rc, _ = pmf.run(str(TMP_ROOT), apply=False, edits_map=FLAT)
        assert rc == 0
        assert cpp.read_text(encoding="utf-8").count("osWaitForever") == 2, "check 模式写了文件"
        assert init.read_text(encoding="utf-8") == CAN_C

        rc, _ = pmf.run(str(TMP_ROOT), apply=True, edits_map=FLAT)
        assert rc == 0
        assert "osSemaphoreAcquire(sem_can1_tx, osWaitForever)" not in cpp.read_text(encoding="utf-8")
        assert "hcan1.Init.AutoBusOff = ENABLE;" in init.read_text(encoding="utf-8")

        rc, lines = pmf.run(str(TMP_ROOT), apply=True, edits_map=FLAT)
        assert rc == 0, (rc, lines)
        assert all("没有要改的" in line for line in lines), lines
        assert cpp.read_bytes().count(b"\r\n") == 0, "换行符被改成了 CRLF"
    finally:
        drop_fixtures()
    return "check 不写、真改对、再跑说没有要改的"


def check_missing_root() -> str:
    rc, lines = pmf.run(str(TMP_ROOT / "no-such-firmware-tree"), apply=True)
    assert rc == 2, rc
    assert all("找不到这个文件" in line for line in lines), lines
    return "目录不存在时返回 2"


def check_real_tree() -> str:
    cpp = REAL_ROOT / pmf.CAN_CPP
    init = REAL_ROOT / pmf.CAN_INIT
    if not cpp.is_file() or not init.is_file():
        return "[skip] 本机没有主控固件源码树"
    _, missing = pmf.apply_edits(cpp.read_text(encoding="utf-8"),
                                 pmf.EDITS[pmf.CAN_CPP], apply=False)
    assert missing == [], "锚点对不上真文件：%s" % missing
    _, missing = pmf.apply_edits(init.read_text(encoding="utf-8"),
                                 pmf.EDITS[pmf.CAN_INIT], apply=False)
    assert missing == [], "初始化项的锚点对不上：%s" % missing
    return "锚点在真源码树里全都找得到"


def main() -> int:
    checks = [
        ("apply", check_applies),
        ("init", check_init_flags),
        ("idem", check_idempotent),
        ("unknown", check_refuses_unknown_tree),
        ("files", check_file_and_check_mode),
        ("no-root", check_missing_root),
        ("real", check_real_tree),
    ]
    failed = 0
    for name, func in checks:
        try:
            note = func()
        except AssertionError as exc:
            print("  [fail] %-8s %s" % (name, exc))
            failed += 1
        else:
            print("  [ ok ] %-8s %s" % (name, note))
    if failed:
        print("\n%d 项失败" % failed)
        return 1
    print("\nALL %d CHECKS PASSED" % len(checks))
    return 0


if __name__ == "__main__":
    sys.exit(main())