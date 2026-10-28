#!/usr/bin/env python3
"""自测：tools/patch_gripper_firmware.py 只改它认识的那三处，且能重复跑。

不碰真机的固件树——字符串层面测替换，文件层面测 I/O，都在仓库里做（系统临时
目录在这台机器上只允许写一层，建不了 Core/Src 这种嵌套结构）。
"""

from __future__ import annotations

import io
import shutil
import sys
import tempfile
from contextlib import contextmanager, redirect_stdout
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import patch_gripper_firmware as pgf  # noqa: E402

CAN_C = """\
static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
}

void CAN_Send(CAN_TxHeaderTypeDef* pHeader, uint8_t* data)
{
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(&hcan, pHeader, data, &TxMailbox) != HAL_OK)
    {
        Error_Handler();
    }
}
"""


@contextmanager
def workspace_tmp():
    root = Path(__file__).resolve().parent.parent / "reference" / "selftest_tmp"
    root.mkdir(parents=True, exist_ok=True)
    path = Path(tempfile.mkdtemp(prefix="patch_fw_", dir=str(root)))
    try:
        yield path
    finally:
        shutil.rmtree(path, ignore_errors=True)


def quiet(func, *args, **kwargs):
    buf = io.StringIO()
    with redirect_stdout(buf):
        code = func(*args, **kwargs)
    return code, buf.getvalue()


def check_applies() -> str:
    text, applied, already, missing = pgf.apply_edits(CAN_C, pgf.EDITS, True)
    assert (applied, already, missing) == (3, 0, []), (applied, already, missing)
    assert "hcan.Init.AutoBusOff = ENABLE;" in text
    assert "hcan.Init.AutoRetransmission = ENABLE;" in text
    body = text.split("void CAN_Send")[1]
    assert "Error_Handler();" not in body, body
    assert "        return;" in body, body
    # 别的初始化项一个字都不许动
    assert "hcan.Init.AutoWakeUp = ENABLE;" in text
    assert "hcan.Init.TransmitFifoPriority = DISABLE;" in text
    assert text.count("hcan.Init.") == CAN_C.count("hcan.Init.")
    return "三处都改了，其它初始化项没动"


def check_idempotent() -> str:
    once, _, _, _ = pgf.apply_edits(CAN_C, pgf.EDITS, True)
    twice, applied, already, missing = pgf.apply_edits(once, pgf.EDITS, True)
    assert (applied, already, missing) == (0, 3, []), (applied, already, missing)
    assert twice == once
    return "重复跑不重复改"


def check_refuses_unknown_tree() -> str:
    stranger = "int main(void) { return 0; }\n"
    text, applied, already, missing = pgf.apply_edits(stranger, pgf.EDITS, True)
    assert applied == 0 and already == 0 and len(missing) == 3
    assert text == stranger
    return "认不出的源码不写"


def check_file_and_check_mode() -> str:
    # 沙箱只允许往 reference/ 下直接写文件，不能新建子目录再往里写，
    # 所以这里用一个固定文件名，用完删掉。
    path = Path(__file__).resolve().parent.parent / "reference" / "selftest_patch_can.c"
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        path.write_text(CAN_C, encoding="utf-8", newline="")
        code, out = quiet(pgf.run_file, path.parent, Path(path.name), False)
        assert code == 1, f"--check 在没改的树上应返回 1，实际 {code}: {out}"
        assert "还有 3 处没改" in out, out
        assert path.read_text(encoding="utf-8") == CAN_C, "--check 居然写了文件"
        code, out = quiet(pgf.run_file, path.parent, Path(path.name), True)
        assert code == 0, out
        assert "hcan.Init.AutoRetransmission = ENABLE;" in path.read_text(encoding="utf-8")
        code, out = quiet(pgf.run_file, path.parent, Path(path.name), False)
        assert code == 0, out
        assert "编译出来的固件就是改过的" in out, out
        assert b"\r\n" not in path.read_bytes()
    finally:
        path.unlink(missing_ok=True)
    return "--check 改前 1、改后 0，且不写文件"


def check_missing_root() -> str:
    with workspace_tmp() as root:
        code, out = quiet(pgf.run_file, root, Path("Core/Src/can.c"), True)
        assert code == 2, code
        assert "找不到" in out, out
    code, out = quiet(pgf.main, ["--firmware-root", "/does/not/exist"])
    assert code == 2, code
    return "目录/文件不存在都报错退出"


def check_local_lparen_matches() -> str:
    """真实固件里 CAN_Send 后面还有别的函数，替换要停在函数末尾。"""
    text = CAN_C + "\nvoid CAN_Init(void) { /* 后面还有函数 */ }\n"
    out, applied, _, missing = pgf.apply_edits(text, pgf.EDITS, True)
    assert applied == 3 and not missing, (applied, missing)
    assert out.endswith("void CAN_Init(void) { /* 后面还有函数 */ }\n")
    return "替换范围停在 CAN_Send 末尾"


def main() -> int:
    checks = [
        ("改动内容", check_applies),
        ("幂等", check_idempotent),
        ("陌生源码树", check_refuses_unknown_tree),
        ("文件与 --check 语义", check_file_and_check_mode),
        ("目录/文件缺失", check_missing_root),
        ("不改到后面的函数", check_local_lparen_matches),
    ]
    failed = 0
    for name, func in checks:
        try:
            detail = func()
        except AssertionError as exc:
            failed += 1
            print(f"[FAIL] {name}: {exc}")
        else:
            print(f"[ok] {name}: {detail}")
    if failed:
        print(f"\n{failed}/{len(checks)} FAILED")
        return 1
    print(f"\nALL {len(checks)} CHECKS PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
