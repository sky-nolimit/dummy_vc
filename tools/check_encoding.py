#!/usr/bin/env python3
"""检查文件是否为合法 UTF-8，以及是否含有预期的文本片段。

控制台编码在 Windows 上不可靠，所以本脚本只输出 ASCII 标签 + repr，
不直接打印中文，避免把「终端显示乱码」误判成「文件损坏」。

用法：
    python tools/check_encoding.py <文件> [<文件> ...]
    python tools/check_encoding.py --probe "回读设备状态" <文件>
"""
import sys


def main(argv):
    probe = None
    args = argv[1:]
    if args and args[0] == '--probe':
        probe = args[1]
        args = args[2:]

    if not args:
        print('用法: check_encoding.py [--probe <文本>] <文件> ...')
        return 1

    bad = 0
    for path in args:
        print(f'--- {path}')
        try:
            raw = open(path, 'rb').read()
        except OSError as exc:
            print(f'    READ-ERROR: {exc}')
            bad += 1
            continue

        print(f'    bytes: {len(raw)}')

        # BOM 检查
        if raw.startswith(b'\xef\xbb\xbf'):
            print('    BOM: UTF-8 BOM present (may cause issues in some tools)')
        elif raw.startswith((b'\xff\xfe', b'\xfe\xff')):
            print('    BOM: UTF-16 BOM present (WRONG for source files)')
            bad += 1

        try:
            text = raw.decode('utf-8')
        except UnicodeDecodeError as exc:
            print(f'    UTF8-DECODE: FAIL -> {exc}')
            bad += 1
            continue
        print('    UTF8-DECODE: OK')

        # 行尾风格
        crlf = raw.count(b'\r\n')
        lf = raw.count(b'\n') - crlf
        print(f'    line-endings: CRLF={crlf} LF-only={lf}')

        # 可疑的 mojibake 特征：UTF-8 被当 GBK 解读后的典型汉字
        mojibake_markers = ['\u9365', '\u70b6', '\u9418', '\u5d88',
                            '\u93c2', '\u951b', '\u6b7f']
        hits = [m for m in mojibake_markers if m in text]
        if hits:
            print('    MOJIBAKE-SUSPECT: found '
                  + ' '.join(f'U+{ord(c):04X}' for c in hits))
            bad += 1
        else:
            print('    MOJIBAKE-SUSPECT: none')

        if probe is not None:
            found = probe in text
            print(f'    PROBE-PRESENT: {found}  (probe U+'
                  + ' U+'.join(f'{ord(c):04X}' for c in probe) + ')')
            if not found:
                bad += 1

    print()
    print(f'files checked: {len(args)}, problems: {bad}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
