#!/usr/bin/env python3
"""Show how the FIRMWARE declares the `hand` endpoints.

The device ships its own protocol description as `dev._json_data` (~55 KB).
That is the firmware's own statement of every endpoint, its arguments, their
types, and what it should send back -- it is authoritative in a way that
guessing at method names is not.

This matters right now because every `hand.*` call blocks on this unit:

    hand.set_current_limit(0.1)   blocked
    hand.set_enable(True)         blocked
    hand.set_angle(90)            blocked

and `robot.set_enable(True)` alone DOES power the gripper (the fingers stiffen),
so the servo and its driver board are alive. The open question is where the
command stops:

    a) the firmware never answers, so the client waits forever
    b) the firmware answers, but the client fails to recognise it
    c) the client never sends, and waits for a reply to something unsent

Those have completely different fixes, and the declared protocol is what
separates them. In particular: does `set_angle` declare a return value or an
acknowledgement at all? A setter that declares no reply should not be waited on.

    pkill -f dummy_controller
    python3 tools/dump_hand_protocol.py
    python3 tools/dump_hand_protocol.py --save protocol.json
    python3 tools/dump_hand_protocol.py --from-file protocol.json
"""
import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller'))

# Matches a path segment that is "args", "return", or one of those with an
# index, e.g. ".args[0]" or ".return".
UNDER_ARG = re.compile(r'(^|\.)(args|return)(\[\d+\])?(\.|$)')


def walk(node, path=''):
    """Yield (path, dict) for every endpoint-looking dict in the tree.

    Only dicts that look like an ENDPOINT are yielded. Arguments and return
    descriptors also carry a name and a type, and matching them produces noise
    like "hand.set_angle.args[0]", so anything reached THROUGH an args/return
    key is skipped. The segment may carry an index ("args[0]"), hence the regex.
    """
    if isinstance(node, dict):
        name = node.get('name')
        if name and 'type' in node and not UNDER_ARG.search(path):
            yield path, node
        for k, v in node.items():
            yield from walk(v, f'{path}.{k}' if path else k)
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from walk(v, f'{path}[{i}]')


def load(args):
    if args.from_file:
        try:
            # utf-8-sig so a BOM does not turn into a confusing JSON error;
            # the device dump itself has none, but a hand-edited copy might.
            with open(args.from_file, encoding='utf-8-sig') as fh:
                return json.load(fh)
        except (OSError, ValueError) as exc:
            print(f'FATAL: cannot read {args.from_file}: {exc}')
            return None
    try:
        import dummy_cli_tool.ref_tool as ref_tool
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot use dummy_cli_tool ({exc})')
        print('       Use --from-file with a previously saved dump instead.')
        return None
    dev = ref_tool.find_any(path='usb', timeout=15.0)
    if dev is None:
        print('FATAL: no device found. Stop dummy_controller first.')
        return None
    raw = getattr(dev, '_json_data', None)
    if raw is None:
        print('FATAL: dev._json_data is not present; cannot read the protocol.')
        return None
    try:
        return json.loads(raw) if isinstance(raw, str) else raw
    except (TypeError, ValueError) as exc:
        print(f'FATAL: dev._json_data is not JSON ({exc})')
        print(f'       first 200 chars: {str(raw)[:200]}')
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--grep', default='hand',
                    help='substring to match endpoint names (default "hand")')
    ap.add_argument('--save', default=None,
                    help='write the whole decoded protocol here for later use')
    ap.add_argument('--from-file', default=None,
                    help='read a previously saved dump instead of the device')
    ap.add_argument('--raw', action='store_true',
                    help='print the full JSON (very long)')
    args = ap.parse_args()

    doc = load(args)
    if doc is None:
        return 1

    if args.save:
        with open(args.save, 'w', encoding='utf-8') as fh:
            json.dump(doc, fh, indent=2)
        print(f'saved the decoded protocol to {args.save}')
        print()

    if args.raw:
        print(json.dumps(doc, indent=2, ensure_ascii=False))
        return 0

    entries = list(walk(doc))
    print(f'the protocol describes {len(entries)} endpoints')
    print()

    want = args.grep.lower()
    # A hand endpoint is one whose name matches AND which sits under a "hand"
    # path, because joint names could otherwise match by accident.
    hits = [(p, d) for p, d in entries
            if want in str(d.get('name', '')).lower()
            or want in p.lower()]
    if not hits:
        print(f'no endpoint matches "{args.grep}".')
        print('Try --grep with another word, or --raw to see everything.')
        return 0

    print(f'=== {len(hits)} endpoint(s) matching "{args.grep}" ===')
    print()
    for path, d in hits:
        print(f'--- {path}')
        print(json.dumps(d, indent=2, ensure_ascii=False))
        print()

    print('=' * 74)
    print('HOW TO READ THIS')
    print('=' * 74)
    print('  For each hand endpoint, look at:')
    print('    * the declared arguments and their TYPES -- is set_angle taking')
    print('      degrees, or something else? A type mismatch would be a bug')
    print('      in the caller, not in the device')
    print('    * whether a RETURN or acknowledgement is declared at all. A')
    print('      setter that declares nothing to return should not be waited')
    print('      on; if the client waits anyway, that is the block')
    print('    * any enum of valid values, which would bound set_angle')
    print()
    print('  Whatever this shows, it is the firmware talking about itself, so')
    print('  it outranks anything inferred from behaviour.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
