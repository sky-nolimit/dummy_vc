#!/usr/bin/env python3
"""Check line endings of the bytes git actually stores (the blob content).

Why this matters: the repo is edited on Windows where core.autocrlf may be
true. What Linux receives at clone time is the BLOB content, not the Windows
working tree. So we must inspect blobs, not files on disk.

For ROS 2 Python packages the critical files are the installed console
scripts, because a CRLF-shebang ('#!/usr/bin/env python3\\r') makes Linux
fail with "bad interpreter".

Usage:
    python tools/check_line_endings.py                # scan the tree
    python tools/check_line_endings.py --staged       # scan the index
"""
import os
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# These are the files whose line endings can actively break a Linux build/run.
CRITICAL_SUFFIXES = ('.py', '.launch.py', '.sh')
CRITICAL_EXACT = ('.gitattributes',)


def git(*args, binary=False):
    out = subprocess.run(
        ['git', '-C', REPO_ROOT] + list(args),
        capture_output=True,
        check=False,
    )
    if out.returncode != 0:
        return None
    return out.stdout if binary else out.stdout.decode('utf-8', 'replace')


def list_files(rev):
    raw = git('ls-tree', '-r', '--name-only', rev)
    if raw is None:
        return []
    return [x for x in raw.splitlines() if x.strip()]


def blob_bytes(rev, path):
    return git('cat-file', 'blob', f'{rev}:{path}', binary=True)


def classify(data):
    if data is None:
        return 'READ-ERROR', 0, 0
    crlf = data.count(b'\r\n')
    lf_only = data.count(b'\n') - crlf
    if b'\x00' in data:
        return 'binary', crlf, lf_only
    if crlf and lf_only:
        return 'MIXED', crlf, lf_only
    if crlf:
        return 'CRLF', crlf, lf_only
    if lf_only:
        return 'LF', crlf, lf_only
    return 'no-newline', crlf, lf_only


def main(argv):
    rev = 'HEAD'
    if '--staged' in argv:
        rev = None  # handled separately below

    if rev is None:
        raw = git('diff', '--cached', '--name-only')
        files = [x for x in (raw or '').splitlines() if x.strip()]
        getter = lambda p: git('cat-file', 'blob', f':{p}', binary=True)  # noqa: E731
    else:
        files = list_files(rev)
        getter = lambda p: blob_bytes(rev, p)  # noqa: E731

    interesting = [
        f for f in files
        if f.endswith(CRITICAL_SUFFIXES) or os.path.basename(f) in CRITICAL_EXACT
    ]

    counts = {}
    problems = []

    for path in interesting:
        data = getter(path)
        kind, crlf, lf = classify(data)
        counts[kind] = counts.get(kind, 0) + 1
        if kind in ('CRLF', 'MIXED', 'READ-ERROR'):
            problems.append((path, kind, crlf, lf))

    print(f'revision: {rev or "index"}')
    print(f'files scanned (py/launch/sh/.gitattributes): {len(interesting)}')
    print('line-ending summary:')
    for k in sorted(counts):
        print(f'    {k:<12} {counts[k]}')

    if problems:
        print()
        print(f'PROBLEMS: {len(problems)} file(s) stored with CRLF or mixed endings')
        for path, kind, crlf, lf in problems[:40]:
            print(f'    {kind:<6} crlf={crlf:<5} lf={lf:<5} {path}')
        if len(problems) > 40:
            print(f'    ... and {len(problems) - 40} more')
        print()
        print('These will break on Linux. Fix with a .gitattributes containing:')
        print('    * text=auto eol=lf')
        print('then: git add --renormalize . && git commit')
        return 1

    print()
    print('OK: no CRLF or mixed endings in the scanned files.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
