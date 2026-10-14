#!/usr/bin/env python3
"""Compare the hand endpoints against endpoints that are KNOWN to work.

Why a comparison rather than reading the hand entry alone
---------------------------------------------------------
The firmware's protocol dump says this about the gripper:

    "name": "set_angle", "id": 296, "type": "function",
    "inputs":  [ { "name": "angle", "id": 297, "type": "float",
                   "access": "rw", "mode": "r" } ],
    "outputs": []

Three things look off for a setter: the input is marked `mode: "r"`, it carries
an `access` field (which is an attribute notion, not an argument notion), and
`outputs` is empty. That shape reads like a writable ATTRIBUTE rather than a
function. If the firmware declares an attribute but the client calls it as a
function -- or the reverse -- the client ends up waiting for a reply that never
comes, which is exactly the permanent block observed.

But that is an inference from one entry, and this project has been burned
repeatedly by inferences. The decisive test is to compare against endpoints
that are known to work on this same device, in the same dump:

    robot.move_j            works (the arm moves)
    robot.set_enable        works
    joint_1.set_position    in the same family
    joint_1.angle           a real attribute, read successfully

If `hand.set_angle` has the same shape as `joint_1.angle` (an attribute) while
`robot.move_j` has a different one (a function), the mismatch is confirmed and
it is a client-side calling-convention bug, not a hardware fault.

    python3 tools/compare_endpoints.py protocol.json
    python3 tools/compare_endpoints.py protocol.json --show move_j
"""
import argparse
import json
import os
import sys


def walk(node, path=''):
    """Yield (path, dict) for every endpoint-looking dict."""
    if isinstance(node, dict):
        name = node.get('name')
        if name and 'type' in node:
            yield path, node
        for k, v in node.items():
            yield from walk(v, f'{path}.{k}' if path else k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from walk(v, f'{path}[{i}]')


def shape(d):
    """A compact signature: what kind of thing is this, structurally."""
    typ = d.get('type')
    ins = d.get('inputs')
    outs = d.get('outputs')
    members = d.get('members')
    bits = [f'type={typ}']
    if members is not None:
        bits.append(f'members={len(members)}')
    if ins is not None:
        bits.append(f'inputs={len(ins)}')
        for a in ins:
            bits.append(
                f'in[{a.get("name")}]={a.get("type")}'
                + (f' access={a["access"]}' if 'access' in a else '')
                + (f' mode={a["mode"]}' if 'mode' in a else ''))
    if outs is not None:
        bits.append(f'outputs={len(outs)}')
        for o in outs:
            bits.append(f'out[{o.get("name")}]={o.get("type")}')
    for extra in ('access', 'mode'):
        if extra in d:
            bits.append(f'{extra}={d[extra]}')
    return '  '.join(bits)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump', help='protocol.json saved by dump_hand_protocol.py')
    ap.add_argument('--show', default=None,
                    help='print the full JSON for endpoints whose path or name '
                         'contains this substring')
    args = ap.parse_args()

    if not os.path.exists(args.dump):
        print(f'FATAL: {args.dump} not found. Save it first with:')
        print('  python3 tools/dump_hand_protocol.py --save protocol.json')
        return 1
    with open(args.dump, encoding='utf-8-sig') as fh:
        doc = json.load(fh)

    entries = list(walk(doc))
    print(f'the dump holds {len(entries)} endpoints')
    print()

    if args.show:
        want = args.show.lower()
        hits = [(p, d) for p, d in entries
                if want in p.lower() or want in str(d.get('name', '')).lower()]
        for p, d in hits:
            print(f'--- {p}')
            print(json.dumps(d, indent=2, ensure_ascii=False))
            print()
        return 0

    # The comparison set: two things proven to work, two things in the same
    # family, and the three gripper entries under investigation.
    groups = [
        ('PROVEN TO WORK on this device', [
            ('robot.move_j', ('move_j', 'robot.move_j')),
            ('robot.set_enable', ('set_enable', 'robot.set_enable')),
        ]),
        ('JOINTS -- same call family as the arm', [
            ('joint_1.set_position', ('set_position', 'joint_1.set_position')),
            ('joint_1.angle (attribute)', ('angle', 'joint_1.angle')),
        ]),
        ('GRIPPER -- every call blocks', [
            ('hand.set_angle', ('set_angle', 'hand.set_angle')),
            ('hand.set_enable', ('set_enable', 'hand.set_enable')),
            ('hand.set_current_limit', ('set_current_limit',
                                       'hand.set_current_limit')),
        ]),
    ]

    found = {}
    for _label, wanted in sum((g[1] for g in groups), []):
        name_sub, path_sub = wanted
        # Prefer an entry under the requested PARENT path. Matching on the leaf
        # name alone is ambiguous: `set_enable` exists both on robot and on
        # hand, so a name-only match would compare the wrong pair.
        parent, leaf = path_sub.rsplit('.', 1)
        best = None
        for p, d in entries:
            if d.get('name') != leaf and d.get('name') != name_sub:
                continue
            if p == path_sub:
                best = (p, d)
                break
            # accept a members[] path whose parent chain ends with `parent`
            if leaf == d.get('name') and parent.split('.')[-1] in p:
                score = len(p)
                if best is None or score < len(best[0]):
                    best = (p, d)
        if best:
            found[path_sub] = best

    for label, wanted in groups:
        print('=' * 76)
        print(label)
        print('=' * 76)
        for disp, key in wanted:
            got = found.get(key[1])
            if not got:
                print(f'  {disp:<28} NOT FOUND in the dump')
                continue
            p, d = got
            print(f'  {disp:<28} {shape(d)}')
            print(f'  {"":<28} at {p}')
        print()

    print('=' * 76)
    print('HOW TO READ THIS')
    print('=' * 76)
    print('  Compare the gripper rows against the two groups above them.')
    print()
    print('  * If the gripper entries look like "joint_1.angle" (an attribute:')
    print('    access/mode, no real inputs), then the firmware is describing a')
    print('    WRITABLE ATTRIBUTE while the client calls a FUNCTION. The client')
    print('    then waits for a function reply that never comes -- which is')
    print('    exactly the permanent block. That is a client-side calling')
    print('    convention bug, and it is fixable without touching hardware.')
    print()
    print('  * If they look like "robot.move_j" (a function with inputs and')
    print('    outputs), then the declaration is normal for a function and the')
    print('    block is elsewhere -- in the client library or the transport,')
    print('    not in the shape of the declaration.')
    print()
    print('  * If move_j ALSO carries access/mode on its inputs, then that is')
    print('    just how this firmware writes functions, and the gripper rows')
    print('    are unremarkable -- which would rule this theory out. Check')
    print('    that first; it is the cheapest way to kill the idea.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
