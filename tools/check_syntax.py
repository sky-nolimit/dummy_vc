#!/usr/bin/env python3
"""对仓库内所有 Python 文件做 AST 语法检查。

工作区在 Windows 上开发、在 Ubuntu 上构建，语法错误在本地就能拦住，
不必等到 colcon build 才发现。

用法：
    python tools/check_syntax.py            # 检查全部
    python tools/check_syntax.py <路径> ...  # 只检查指定路径
"""
import ast
import os
import sys

# 不检查的目录
SKIP_DIRS = {
    '.git', 'build', 'install', 'log', '__pycache__',
    'node_modules', '.vscode', '.idea',
}

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def iter_py_files(root):
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if fn.endswith('.py'):
                yield os.path.join(dirpath, fn)


def check(path):
    """返回 (ok, 错误描述)。"""
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            src = fh.read()
    except UnicodeDecodeError as exc:
        return False, f'读取失败（编码问题）：{exc}'

    try:
        ast.parse(src, filename=path)
    except SyntaxError as exc:
        loc = f'第 {exc.lineno} 行'
        if exc.offset:
            loc += f' 第 {exc.offset} 列'
        text = (exc.text or '').rstrip()
        detail = f'{exc.msg}  ({loc})'
        if text:
            detail += f'\n      {text}'
        return False, detail
    return True, ''


def main(argv):
    targets = argv[1:] or [REPO_ROOT]

    files = []
    for t in targets:
        if os.path.isfile(t):
            files.append(os.path.abspath(t))
        elif os.path.isdir(t):
            files.extend(iter_py_files(os.path.abspath(t)))
        else:
            print(f'[跳过] 路径不存在：{t}')

    files = sorted(set(files))
    if not files:
        print('没有找到任何 .py 文件')
        return 1

    failed = []
    for path in files:
        ok, detail = check(path)
        rel = os.path.relpath(path, REPO_ROOT)
        if ok:
            print(f'  OK   {rel}')
        else:
            print(f'  失败 {rel}')
            for line in detail.splitlines():
                print(f'       {line}')
            failed.append(rel)

    print()
    print(f'共检查 {len(files)} 个文件，失败 {len(failed)} 个')
    if failed:
        print('失败清单：')
        for rel in failed:
            print(f'  - {rel}')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
