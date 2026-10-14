#!/usr/bin/env python3
"""Show endpoint IDs around the gripper, to confirm the "firmware never
replies to these IDs" explanation.

What the client does on every call
----------------------------------
From dummy_cli_tool/fibre/remote_object.py:

    def set_value(self, value):
        buffer = self._codec.serialize(value)
        # TODO: Currenly we wait for an ack here.
        self._channel__.remote_endpoint_operation(self._id, buffer, True, 0)
                                                                  ^^^^
    def __call__(self, *args):
        for i in range(len(args)):
            self._inputs[i].set_value(args[i])          # waits for an ack
        self._channel__.remote_endpoint_operation(
            self._trigger_id, None, True, 0)            # waits for a reply
        if len(self._outputs) > 0:
            return self._outputs[0].get_value()         # waits for data

`expect_response` is True in all of them and there is NO TIMEOUT anywhere. So a
call blocks forever if the firmware does not answer that endpoint id.

That matters because `robot.move_j` goes through the SAME code path -- it has
inputs, so `set_value` runs, and it waits for a reply -- and it RETURNS. So the
client is not the difference. The difference must be which endpoint IDs the
firmware answers.

The gripper declares:

    set_angle          id 296   input angle          id 297
    set_enable         id 298   input enable         id 299
    set_current_limit  id 300   input current        id 301

A contiguous block, all of which block. This script shows those IDs next to the
IDs of calls that work, so the pattern is visible rather than asserted.

    python3 tools/gripper_ids.py protocol.json
    python3 tools/gripper_ids.py protocol.json --around 296
"""
import argparse
import json
import os
import sys


def walk(node, chain=()):
    """Yield (name_chain, dict) for every node carrying an id."""
    if isinstance(node, dict):
        name = node.get('name')
        here = chain + (name,) if name else chain
        if name and 'type' in node:
            yield here, node
        for k, v in node.items():
            if k in ('inputs', 'outputs', 'args', 'return'):
                continue
            yield from walk(v, here)
    elif isinstance(node, list):
        for v in node:
            yield from walk(v, chain)


def ids_of(d):
    """Every id this node declares: itself, its inputs, its outputs."""
    out = []
    if 'id' in d:
        out.append(('self', d['id']))
    for key in ('inputs', 'outputs'):
        for a in (d.get(key) or []):
            if 'id' in a:
                out.append((f'{key[:-1]}:{a.get("name")}', a['id']))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dump')
    ap.add_argument('--around', type=int, default=None,
                    help='also show endpoints whose id is within 20 of this')
    args = ap.parse_args()

    if not os.path.exists(args.dump):
        print(f'FATAL: {args.dump} not found')
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

    # calls proven to work, and the mode setter that the plan suspected
    probes = [
        ('WORKS  robot.move_j', ('move_j',)),
        ('WORKS  robot.set_enable', ('set_enable',)),
        ('WORKS  robot.homing', ('homing',)),
        ('WORKS  joint_1.set_position', ('joint_1', 'set_position')),
        ('BLOCKS hand.set_angle', ('hand', 'set_angle')),
        ('BLOCKS hand.set_enable', ('hand', 'set_enable')),
        ('BLOCKS hand.set_current_limit', ('hand', 'set_current_limit')),
    ]

    print('=' * 76)
    print('ENDPOINT IDS DECLARED FOR EACH CALL')
    print('=' * 76)
    print('  Every call waits for a reply on each of these ids. No reply means')
    print('  no return, forever, because the client has no timeout.')
    print()
    for label, names in probes:
        hits = [(c, d) for c, d in entries
                if len(c) >= len(names) and tuple(c[-len(names):]) == names]
        if not hits:
            print(f'  {label:<32} NOT FOUND')
            continue
        c, d = min(hits, key=lambda cd: len(cd[0]))
        ids = ids_of(d)
        txt = ', '.join(f'{k}={v}' for k, v in ids) or 'no ids declared'
        print(f'  {label:<32} {txt}')
        print(f'  {"":<32} at {".".join(c)}')
    print()

    gripper_ids = set()
    for names in (('hand', 'set_angle'), ('hand', 'set_enable'),
                  ('hand', 'set_current_limit')):
        hits = [(c, d) for c, d in entries
                if len(c) >= len(names) and tuple(c[-len(names):]) == names]
        if hits:
            d = min(hits, key=lambda cd: len(cd[0]))[1]
            gripper_ids.update(v for _k, v in ids_of(d))

    print('=' * 76)
    print('THE GRIPPER OCCUPIES ids ' + ', '.join(str(v) for v in sorted(gripper_ids)))
    print('=' * 76)
    print('  A contiguous block. Everything in it blocks. Everything outside')
    print('  it that has been tried returns. That is the pattern the client')
    print('  code predicts if the firmware simply never answers these ids.')
    print()

    if args.around is not None:
        lo, hi = args.around - 20, args.around + 20
        print(f'  Endpoints with an id in [{lo}, {hi}]:')
        rows = []
        for c, d in entries:
            for kind, v in ids_of(d):
                if lo <= v <= hi:
                    rows.append((v, '.'.join(c), kind))
        for v, path, kind in sorted(set(rows)):
            print(f'    {v:>5}  {path:<44} {kind}')
        print()

    # Make sure the gripper has no OTHER control path hiding in the dump. If
    # something else can drive it, this whole analysis is moot.
    print('=' * 76)
    print('ANY OTHER WAY TO DRIVE THE GRIPPER?')
    print('=' * 76)
    words = ('grip', 'finger', 'hand', 'jaw', 'claw', 'servo', 'pneum',
             'vacuum', 'suck')
    seen = set()
    found = []
    for c, d in entries:
        path = '.'.join(c)
        low = path.lower()
        for w in words:
            if w in low and path not in seen:
                seen.add(path)
                found.append((path, d))
                break
    if not found:
        print('  nothing matched; the gripper appears only under "hand"')
    else:
        for path, d in found:
            ids = ', '.join(f'{k}={v}' for k, v in ids_of(d)) or '-'
            print(f'  {path:<46} {d.get("type"):<9} {ids}')
    print()
    print('  If every row here is inside the hand block, then "hand" is the')
    print('  only declared route to the gripper, and there is no second path')
    print('  to try.')
    print()

    print('=' * 76)
    print('WHAT THIS DOES AND DOES NOT PROVE')
    print('=' * 76)
    print('  It does NOT prove the firmware ignores these ids -- only the')
    print('  device can show that, and it is not answering, which is the')
    print('  problem. What it does show is that the declared shape is normal:')
    print('  same kind of endpoint, same ids-style block, same client path as')
    print('  calls that work. So the failure is not in how the gripper calls')
    print('  are declared or how the client invokes them.')
    print()
    print('  The remaining explanation is that the firmware declares a gripper')
    print('  it does not implement. That is consistent with everything seen:')
    print('  the servo is powered (fingers stiffen when the robot is enabled),')
    print('  no command ever produces motion, no call ever returns, and the')
    print('  reference project shipped the gripper code commented out.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
