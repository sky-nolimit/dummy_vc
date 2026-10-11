#!/usr/bin/env python3
"""Check that every movable URDF joint is actually published in /joint_states.

Why this exists: robot_state_publisher only emits a transform for a joint once
it has received a position for that joint. If a movable joint is never
published, the TF chain below it is silently broken and RViz reports

    No transform from [<child link>] to [<fixed frame>]

That is exactly what happened here: the driver published only the six arm
joints, never the two prismatic finger joints, so both finger links were
disconnected from base_link.

This script cross-checks the URDF against a live /joint_states message, so it
catches the whole class of problem rather than one instance.

Usage (ROS 2 environment, with the driver running):
    ros2 run <pkg> ...            # no, run it directly:
    python3 tools/check_tf_tree.py
    python3 tools/check_tf_tree.py --xacro path/to/robot.xacro
    python3 tools/check_tf_tree.py --urdf path/to/robot.urdf

Read-only: subscribes to /joint_states, publishes nothing, commands nothing.
Exit code 0 when complete, 1 when something is missing.
"""
import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET


MOVABLE_TYPES = {'revolute', 'prismatic', 'continuous', 'planar', 'floating'}


def strip_ns(tag):
    return tag.split('}', 1)[1] if '}' in tag else tag


def load_tree(path):
    """Return (links, joints) parsed from a .urdf or .xacro file.

    For .xacro we do a preprocessing pass that removes comments and xacro
    tags, because the file is not plain URDF. Property substitution is not
    performed, but joint and link NAMES never contain properties in this
    project, so the names we need survive intact.
    """
    text = open(path, 'r', encoding='utf-8').read()

    if path.endswith('.xacro'):
        # Drop comments (XML comments may not nest, so non-greedy is fine)
        text = re.sub(r'<!--.*?-->', '', text, flags=re.S)
        # Drop xacro-only elements and attributes so ET can parse what's left
        text = re.sub(r'<xacro:[^>]*?/>', '', text, flags=re.S)
        text = re.sub(r'<xacro:[^>]*?>.*?</xacro:[^>]*?>', '', text, flags=re.S)

    root = ET.fromstring(text)
    links = set()
    joints = {}
    for el in root:
        t = strip_ns(el.tag)
        if t == 'link':
            links.add(el.get('name'))
        elif t == 'joint':
            name = el.get('name')
            jtype = el.get('type')
            parent = el.find('parent')
            child = el.find('child')
            joints[name] = {
                'type': jtype,
                'parent': parent.get('link') if parent is not None else None,
                'child': child.get('link') if child is not None else None,
            }
    return links, joints


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--xacro', default=None,
                    help='path to the robot xacro (default: search the workspace)')
    ap.add_argument('--urdf', default=None,
                    help='path to a plain urdf instead of xacro')
    ap.add_argument('--timeout', type=float, default=10.0,
                    help='seconds to wait for the first /joint_states')
    args = ap.parse_args()

    # ---- locate the model -------------------------------------------------
    model = args.urdf or args.xacro
    if model is None:
        here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        candidate = os.path.join(
            here, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf', 'dummy2_urdf.xacro')
        if not os.path.isfile(candidate):
            print(f'cannot find default xacro at {candidate}; pass --xacro')
            return 1
        model = candidate

    print(f'model: {model}')
    try:
        links, joints = load_tree(model)
    except Exception as exc:
        print(f'failed to parse model: {exc}')
        return 1

    movable = {n: j for n, j in joints.items() if j['type'] in MOVABLE_TYPES}
    fixed = {n: j for n, j in joints.items() if j['type'] == 'fixed'}
    print(f'links={len(links)} joints={len(joints)} '
          f'(movable={len(movable)}, fixed={len(fixed)})')
    print()

    # ---- connect to ROS ---------------------------------------------------
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except ImportError as exc:
        print(f'ROS 2 Python not available: {exc}')
        print()
        print('Static analysis only (no /joint_states cross-check).')
        print('Movable joints that MUST appear in /joint_states:')
        for n in sorted(movable):
            print(f'  {n:<28} {movable[n]["type"]:<10} -> {movable[n]["child"]}')
        return 0

    rclpy.init()
    node = rclpy.create_node('tf_tree_checker')
    received = {}

    def cb(msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                received[name] = msg.position[i]

    node.create_subscription(JointState, '/joint_states', cb, 10)

    import time
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline and not received:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not received:
        print(f'no /joint_states received within {args.timeout:.0f}s.')
        print('Is the driver running and activated?')
        print()
        print('Without joint states, robot_state_publisher publishes NO')
        print('movable-joint transforms at all, and RViz will report')
        print('"No transform from [<link>] to [<fixed frame>]" for every link')
        print('below the first movable joint.')
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f'/joint_states has {len(received)} joints:')
    for n in sorted(received):
        print(f'  {n:<30} {received[n]:+.5f}')
    print()

    # ---- cross-check ------------------------------------------------------
    missing = [n for n in movable if n not in received]
    extra = [n for n in received if n not in joints]

    ok = True

    if missing:
        ok = False
        print(f'MISSING {len(missing)} movable joint(s) from /joint_states:')
        for n in missing:
            j = movable[n]
            print(f'  {n:<28} {j["type"]:<10} '
                  f'{j["parent"]} -> {j["child"]}')
        print()
        print('  Each of these breaks the TF chain for its child link.')
        print('  robot_state_publisher will not emit a transform for a joint')
        print('  it has never received a position for. RViz then reports:')
        for n in missing:
            print(f'    No transform from [{movable[n]["child"]}] to [base_link]')
    else:
        print('OK: every movable joint appears in /joint_states.')

    if extra:
        print()
        print(f'Note: {len(extra)} joint(s) published but not in the model:')
        for n in extra:
            print(f'  {n}')
        print('  These are ignored by robot_state_publisher. A name typo here')
        print('  is a common cause of a missing transform.')

    node.destroy_node()
    rclpy.shutdown()

    print()
    if ok:
        # walk the chain to confirm nothing else is disconnected
        child_to_parent = {j['child']: (n, j['parent'])
                           for n, j in joints.items() if j['child']}
        print('static connectivity:')
        for link in sorted(links):
            if link in ('world', 'base_link'):
                continue
            chain = []
            cur = link
            guard = 0
            while cur != 'base_link' and cur in child_to_parent and guard < 64:
                jn, par = child_to_parent[cur]
                chain.append(f'{jn}({par}->{cur})')
                cur = par
                guard += 1
            if cur == 'base_link':
                print(f'  {link:<30} reachable via {len(chain)} joint(s)')
            else:
                print(f'  {link:<30} NOT reachable from base_link')
                ok = False

    print()
    print('RESULT:', 'OK' if ok else 'PROBLEMS FOUND')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
