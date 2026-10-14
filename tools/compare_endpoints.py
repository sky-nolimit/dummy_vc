#!/usr/bin/env python3
"""Compare the hand endpoints against endpoints KNOWN to work, by NAME chain.

Two bugs this rewrite fixes
---------------------------
1. The previous version matched endpoints against hardcoded dot-paths such as
   "robot.hand.set_angle". The dump is a NESTED ARRAY, so the paths it produces
   look like "[4].members[10].members[0]" and nothing ever matched. Every row
   printed "NOT FOUND", which said nothing about the protocol and everything
   about the matching. Endpoints are now addressed by their chain of NAMES,
   which is stable regardless of how the tree is nested.

2. The dumper printed positional paths for the same reason, so its output could
   not be related to anything. It now prints name chains too.

Why compare at all
------------------
The firmware declares the gripper calls like this:

    "name": "set_angle", "id": 296, "type": "function",
    "inputs":  [ { "name": "angle", "id": 297, "type": "float",
                   "access": "rw", "mode": "r" } ],
    "outputs": []

A `mode` and an `access` on an INPUT is attribute language, not argument
language, and `outputs` is empty. That reads like a writable ATTRIBUTE declared
in function clothing. If the client calls it as a function it will wait for a
reply that never comes -- which is exactly the permanent block observed.

The decisive check is whether the endpoints that DO work on this same device
look the same or different. In particular `robot.move_j`: it works, so whatever
shape it has is a shape this firmware uses successfully.

    python3 tools/compare_endpoints.py protocol.json
    python3 tools/compare_endpoints.py protocol.json --show set_angle
    python3 tools/compare_endpoints.py protocol.json --list hand
"""
import argparse
import json
import os
import sys


def walk(node, chain=()):
    """Yield (name_chain, node) for every endpoint-looking dict.

    The chain is the sequence of `name` fields from the root down, so it does
    not depend on list indices. Endpoints reached through `members` keep the
    parent object's name in the chain, which is what makes
    ("robot", "hand", "set_angle") addressable.
    """
    if isinstance(node, dict):
        name = node.get('name')
        here = chain + (name,) if name else chain
        # Skip argument and return descriptors: they have names too, and
        # including them buries the real endpoints in noise.
        if 'members' in node and node.get('type') != 'object':
            pass
        yield here, node
        for k, v in node.items():
            if k in ('inputs', 'outputs', 'args', 'return'):
                continue
            yield from walk(v, here)
    elif isinstance(node, list):
        for v in node:
            yield from walk(v, chain)


def shape(d):
    """A compact structural signature."""
    typ = d.get('type')
    bits = [f'type={typ}']
    if 'id' in d:
        bits.append(f'id={d["id"]}')
    if 'members' in d:
        bits.append(f'members={len(d["members"])}')
    for key, tag in (('inputs', 'in'), ('outputs', 'out')):
        vals = d.get(key)
        if vals is None:
            continue
        bits.append(f'{key}={len(vals)}')
        for a in vals:
            s = f'{tag}[{a.get("name")}]={a.get("type")}'
            if 'access' in a:
                s += f' access={a["access"]}'
            if 'mode' in a:
                s += f' mode={a["mode"]}'
            bits.append(s)
    for extra in ('access', 'mode'):
        if extra in d:
            bits.append(f'{extra}={d[extra]}')
    return '  '.join(bits)


def find(entries, *names):
    """Return the entry whose chain ENDS with the given names, or None.

    Matching on a suffix lets us ask for ("move_j",) or ("hand", "set_angle")
    without knowing whether the tree keeps "robot" at the root or nests it.
    """
    hits = [(c, d) for c, d in entries
            if len(c) >= len(names) and tuple(c[-len(names):]) == tuple(names)]
    if not hits:
        return None
    # shortest chain wins: the least nested match is the real endpoint
    return min(hits, key=lambda cd: len(cd[0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump', help='protocol.json saved by dump_hand_protocol.py')
    ap.add_argument('--show', default=None,
                    help='print the full JSON for endpoints whose name chain '
                         'contains this substring')
    ap.add_argument('--list', default=None,
                    help='list every endpoint whose name chain contains this '
                         'substring')
    args = ap.parse_args()

    if not os.path.exists(args.dump):
        print(f'FATAL: {args.dump} not found. Save it first with:')
        print('  python3 tools/dump_hand_protocol.py --save protocol.json')
        return 1
    try:
        with open(args.dump, encoding='utf-8-sig') as fh:
            doc = json.load(fh)
    except (OSError, ValueError) as exc:
        print(f'FATAL: cannot read {args.dump}: {exc}')
        return 1

    entries = [(c, d) for c, d in walk(doc) if c]
    print(f'the dump holds {len(entries)} named nodes')
    print()

    if args.list:
        want = args.list.lower()
        hits = [(c, d) for c, d in entries if want in '.'.join(c).lower()]
        for c, d in hits:
            print(f'  {".".join(c):<52} {shape(d)}')
        print(f'  ({len(hits)} match(es))')
        return 0

    if args.show:
        want = args.show.lower()
        hits = [(c, d) for c, d in entries if want in '.'.join(c).lower()]
        for c, d in hits:
            print(f'--- {".".join(c)}')
            print(json.dumps(d, indent=2, ensure_ascii=False))
            print()
        return 0

    groups = [
        ('KNOWN TO WORK on this device', [
            ('robot.move_j', ('move_j',)),
            ('robot.set_enable', ('set_enable',)),
            ('robot.homing', ('homing',)),
        ]),
        ('JOINT FAMILY -- what a working setter looks like', [
            ('joint_1.set_position', ('joint_1', 'set_position')),
            ('joint_1.angle (attribute)', ('joint_1', 'angle')),
            ('joint_1.set_enable', ('joint_1', 'set_enable')),
        ]),
        ('# GRIPPER -- every call blocks', [
            ('hand.set_angle', ('hand', 'set_angle')),
            ('hand.set_enable', ('hand', 'set_enable')),
            ('hand.set_current_limit', ('hand', 'set_current_limit')),
        ]),
    ]

    # Collect the move_j input shape, because it is the single most useful
    # comparison: it works, so its shape is a shape this firmware uses for a
    # call that succeeds.
    movej = find(entries, 'move_j')
    movej_has_attr = False
    if movej is not None:
        for a in (movej[1].get('inputs') or []):
            if 'access' in a or 'mode' in a:
                movej_has_attr = True

    for label, wanted in groups:
        print('=' * 76)
        print(label)
        print('=' * 76)
        for disp, names in wanted:
            got = find(entries, *names)
            if not got:
                print(f'  {disp:<30} NOT FOUND')
                continue
            c, d = got
            flag = ''
            if names[-1] in ('set_angle', 'set_enable',
                             'set_current_limit') and names[0] == 'hand':
                ins = d.get('inputs') or []
                if any('access' in a or 'mode' in a for a in ins):
                    flag = '   <== input carries access/mode'
            print(f'  {disp:<30} {shape(d)}{flag}')
            print(f'  {"":<30} at {".".join(c)}')
        print()

    print('=' * 76)
    print('VERDICT')
    print('=' * 76)
    if movej is None:
        print('  robot.move_j was not found, so the comparison could not be')
        print('  made. Run:  --list move')
        print('  to see what the motion endpoints are actually called.')
    elif not movej_has_attr:
        print('  robot.move_j declares its input WITHOUT access/mode, and it')
        print('  works. If the hand inputs carry access/mode, the gripper is')
        print('  declared in a form the working calls do not use -- i.e. as a')
        print('  writable ATTRIBUTE rather than a function argument. A client')
        print('  calling it as a function would then wait forever for a reply')
        print('  that never comes, which matches the observed block exactly.')
        print()
        print('  That would be a CLIENT-SIDE calling convention problem, and')
        print('  fixable in software. Check the gripper rows above for the')
        print('  "access/mode" flag: present on all three confirms it.')
    else:
        print('  robot.move_j ALSO carries access/mode on its input, so that')
        print('  is simply how this firmware declares functions. The gripper')
        print('  entries are then unremarkable, this theory is dead, and the')
        print('  block is elsewhere -- in the client library or the transport.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
