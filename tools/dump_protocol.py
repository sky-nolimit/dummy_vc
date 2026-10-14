#!/usr/bin/env python3
"""Decode the device's own protocol description and find what move_j needs.

probe_attrs.py revealed methods that were never in any guessed list:

    robot.set_command_mode      <- almost certainly the missing step
    robot.set_joint_speed, set_joint_acc
    robot.resting, calibrate_home_offset, reboot
    robot.tuning, joint_all

It also showed the device ships its own protocol description as
dev._json_data (about 55 KB). That is authoritative: it is the firmware's own
declaration of every endpoint, its arguments and their types. Reading it beats
guessing at argument names or valid mode values.

This script decodes that JSON and prints, for every endpoint whose name looks
relevant to motion, its signature as the firmware declares it -- and in
particular whatever enumerates the valid command modes.

It does NOT move the arm.

    pkill -f dummy_controller
    python3 tools/dump_protocol.py
    python3 tools/dump_protocol.py --grep mode
"""
import argparse
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

# Names worth showing in full when no --grep is given.
INTERESTING = ('mode', 'move', 'enable', 'home', 'rest', 'calibrat', 'speed',
               'acc', 'joint', 'position', 'velocity', 'current', 'erase')


def walk(node, path=''):
    """Yield (path, name, dict) for every endpoint-looking dict in the tree."""
    if isinstance(node, dict):
        if 'name' in node and 'type' in node:
            yield path, node.get('name', ''), node
        for k, v in node.items():
            yield from walk(v, f'{path}.{k}' if path else k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from walk(v, f'{path}[{i}]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--grep', default=None,
                    help='only show endpoints whose name matches this substring')
    ap.add_argument('--raw', action='store_true',
                    help='dump the whole JSON (very long)')
    ap.add_argument('--save', default=None,
                    help='write the decoded JSON to this path')
    args = ap.parse_args()

    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except ImportError as exc:
        print(f'FATAL: cannot import dummy_cli_tool ({exc})')
        return 2

    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return 1

    raw = getattr(dev, '_json_data', None)
    if raw is None:
        print('FATAL: dev._json_data not present; cannot read the protocol.')
        return 1
    try:
        doc = json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError) as exc:
        print(f'FATAL: could not parse dev._json_data as JSON: {exc}')
        print(f'       first 200 chars: {str(raw)[:200]}')
        return 1

    if args.save:
        with open(args.save, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=2)
        print(f'saved decoded protocol to {args.save}')
        print()

    if args.raw:
        print(json.dumps(doc, indent=2))
        return 0

    entries = list(walk(doc))
    print(f'protocol describes {len(entries)} endpoints')
    print()

    if args.grep:
        want = args.grep.lower()
        hits = [(p, n, d) for p, n, d in entries if want in n.lower()]
    else:
        hits = [(p, n, d) for p, n, d in entries
                if any(k in n.lower() for k in INTERESTING)]

    if not hits:
        print('no matching endpoints. Try --raw, or --grep with another word.')
        return 0

    for path, name, d in hits:
        print(f'--- {path}')
        print(json.dumps(d, indent=2, ensure_ascii=False))
        print()

    print('=' * 62)
    print('Look for set_command_mode: its declared argument and any enum of')
    print('valid values tells us exactly what to call before move_j.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
