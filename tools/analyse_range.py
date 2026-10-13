#!/usr/bin/env python3
"""Analyse a probe_range sweep and report the accepted interval per joint.

probe_range marks a value "no" when nothing moved. That is ambiguous: a value
can be ignored because the DEVICE REFUSES it, or simply because the joint is
ALREADY THERE, in which case there is nothing to do and no movement is expected.
The sweep always returns to the same base pose between samples, so the value
that equals the base is always such a no-op -- and it shows up as a spurious
"gap" in the middle of an otherwise contiguous interval.

This script reads the base pose (the readback of every row that reported no
movement with a settle time near the minimum) and reclassifies:

    NOOP-AT-BASE   the command asked for the pose the arm was already in
    REFUSED        the command asked for somewhere else and was ignored

Only the second kind bounds the accepted interval.

Usage:
    python tools/analyse_range.py <sweep.txt>
"""
import re
import sys

# The base readback on this unit: whatever probe_range reports for the base.
BASE_READBACK = {'joint_1': 0.0, 'joint_2': 75.0, 'joint_3': -90.0,
                 'joint_4': 0.0, 'joint_5': 0.0, 'joint_6': 0.0}


def parse(path):
    """Return {joint: [(value, accepted, settle, readback)]}."""
    out = {}
    cur = None
    row = re.compile(
        r'^\s*(-?\d+\.\d+)\s+(YES|no|raise)\s+(\d+\.\d+)\s+\[(.*)\]\s*$')
    for line in open(path, encoding='utf-8', errors='replace'):
        m = re.match(r'^--- (joint_\d):', line)
        if m:
            cur = m.group(1)
            out[cur] = []
            continue
        if cur is None:
            continue
        m = row.match(line)
        if not m:
            continue
        val = float(m.group(1))
        accepted = m.group(2) == 'YES'
        settle = float(m.group(3))
        rb = [float(x) for x in m.group(4).split(',')]
        out[cur].append((val, accepted, settle, rb))
    return out


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    data = parse(sys.argv[1])
    if not data:
        print('no rows parsed; is this a probe_range output file?')
        return 1

    print('=' * 72)
    print('ACCEPTED INTERVAL PER JOINT (no-op-at-base rows reclassified)')
    print('=' * 72)
    print()

    for joint in sorted(data, key=lambda s: int(s.split('_')[1])):
        idx = int(joint.split('_')[1]) - 1
        rows = data[joint]
        accepted = [v for v, ok, _, _ in rows if ok]

        # A refusal is only meaningful if the joint had somewhere to go. The
        # base readback is what the arm returns to between samples, so a
        # command equal to it is a no-op, not a refusal.
        base = BASE_READBACK.get(joint)
        refused = []
        noop = []
        for v, ok, settle, rb in rows:
            if ok:
                continue
            if base is not None and abs(v - base) < 1.0:
                noop.append(v)
            else:
                refused.append(v)

        print(f'{joint}:')
        if accepted:
            print(f'  accepted : {min(accepted):>8.1f} .. {max(accepted):>8.1f}')
        else:
            print('  accepted : none')
        if noop:
            print(f'  no-op    : {noop}  <- equal to the base pose; the joint '
                  'was already there.')
            print('             NOT a refusal, so not an interval boundary.')
        if refused:
            print(f'  refused  : min {min(refused):>8.1f}, '
                  f'max {max(refused):>8.1f}')
            below = [v for v in refused if accepted and v < min(accepted)]
            above = [v for v in refused if accepted and v > max(accepted)]
            if below:
                print(f'             below the interval: {min(below):.1f} '
                      f'.. {max(below):.1f}')
            if above:
                print(f'             above the interval: {min(above):.1f} '
                      f'.. {max(above):.1f}')
        else:
            print('  refused  : none outside the accepted span')
        print()

    print('=' * 72)
    print('These intervals are what the driver must fold its targets into.')
    print('A target outside them is refused SILENTLY, which is the')
    print('"sent ok, arm did not move" symptom.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
