#!/usr/bin/env python3
"""Validate URDF/Xacro structure offline, without ROS.

Why this exists: the workspace is edited on Windows but built on Ubuntu with
ROS 2. A malformed URDF, or a joint referencing a frame that does not exist,
is otherwise only discovered at launch time on the other machine. This script
catches the structural problems locally in a second.

It performs:
  1. XML well-formedness
  2. every <joint> has parent/child, both referring to a declared <link>
  3. every link is connected (no orphan subtrees) and the graph is a tree
     rooted at a single base link
  4. xacro property references in attributes resolve to a declared property
     (a cheap check: catches typos like ${tcp_offset_z})
  5. required frames for this project exist

Usage:
    python tools/check_urdf.py
    python tools/check_urdf.py <file.xacro>
"""
import os
import re
import sys
import xml.etree.ElementTree as ET

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

DEFAULT_TARGETS = [
    os.path.join(REPO_ROOT, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf', 'dummy2_urdf.xacro'),
]

# Frames this project needs for MoveIt and for the vision pipeline.
# camera_color_optical_frame is deliberately NOT here: it belongs to the camera
# driver (orbbec_camera publishes camera_link -> camera_color_frame ->
# camera_color_optical_frame). Defining it in the URDF as well gave that frame
# two parents, and tf2 then silently mixes the two trees. What the URDF does own
# is the mount (base_link -> camera_link) and the USB fallback optical frame.
REQUIRED_FRAMES = [
    'base_link',
    'link7_v1_1',
    'dummy_hand_v1_1',
    'tool0',
    'gripper_tcp',
    'camera_link',
    'usb_camera_optical_frame',
]

# A root/world frame is needed for MoveIt's virtual joint.
REQUIRED_ROOT_HINTS = ['world']


def parse(path):
    try:
        return ET.parse(path).getroot()
    except ET.ParseError as exc:
        print(f'    XML PARSE FAIL: {exc}')
        return None


def check_file(path):
    print(f'--- {os.path.relpath(path, REPO_ROOT)}')
    root = parse(path)
    if root is None:
        return 1

    problems = 0
    print(f'    root element: <{root.tag} name="{root.get("name")}">')

    links = [e.get('name') for e in root.findall('link')]
    joints = root.findall('joint')
    link_set = set(links)
    print(f'    links={len(links)}  joints={len(joints)}')

    # --- 2. parent/child integrity -----------------------------------------
    edges = []
    for j in joints:
        jn = j.get('name')
        par = j.find('parent')
        ch = j.find('child')
        if par is None or ch is None:
            print(f'    PROBLEM joint "{jn}" missing parent or child')
            problems += 1
            continue
        p = par.get('link')
        c = ch.get('link')
        if p not in link_set:
            print(f'    PROBLEM joint "{jn}" parent "{p}" is not a declared link')
            problems += 1
        if c not in link_set:
            print(f'    PROBLEM joint "{jn}" child "{c}" is not a declared link')
            problems += 1
        edges.append((p, c, jn))

    # --- 3. connectivity and tree structure --------------------------------
    children = {}
    parents = {}
    for p, c, jn in edges:
        children.setdefault(p, []).append((c, jn))
        parents.setdefault(c, []).append((p, jn))

    multiple_parents = {c: v for c, v in parents.items() if len(v) > 1}
    for c, v in multiple_parents.items():
        print(f'    PROBLEM link "{c}" has {len(v)} parents: '
              + ', '.join(jn for _, jn in v))
        problems += 1

    roots = [l for l in links if l not in parents]
    print(f'    root link(s): {roots}')
    if len(roots) != 1:
        print(f'    PROBLEM expected exactly 1 root link, found {len(roots)}')
        problems += 1

    # walk from the root and see if every link is reachable
    if len(roots) == 1:
        seen = set()
        stack = [roots[0]]
        while stack:
            n = stack.pop()
            if n in seen:
                continue
            seen.add(n)
            for c, _ in children.get(n, []):
                stack.append(c)
        orphans = link_set - seen
        if orphans:
            print(f'    PROBLEM {len(orphans)} link(s) unreachable from root: '
                  + ', '.join(sorted(orphans)[:10]))
            problems += 1
        else:
            print('    connectivity: all links reachable from root')

    # --- 4. xacro property references --------------------------------------
    props = {e.get('name') for e in root.iter()
             if e.tag.endswith('property') and e.get('name')}
    # properties declared in included files are unknown here, so only report
    # references that look like this file's own naming style
    raw = open(path, 'r', encoding='utf-8').read()
    refs = set(re.findall(r'\$\{([A-Za-z_][A-Za-z0-9_]*)\s*[-+*/]?\s*[^}]*\}', raw))
    unknown = []
    for r in refs:
        base = re.match(r'([A-Za-z_][A-Za-z0-9_]*)', r).group(1)
        if base in props:
            continue
        # xacro builtins and things defined in included files
        if base in ('pi', 'load_yaml', 'find', 'arg', 'radians', 'degrees'):
            continue
        unknown.append(base)
    # only flag names that look like our own configurable properties
    suspicious = sorted({u for u in unknown if u.startswith(('tcp_', 'camera_', 'hand_', 'mount_'))})
    if suspicious:
        print('    PROBLEM xacro property referenced but not declared here: '
              + ', '.join(suspicious))
        problems += 1
    else:
        declared = sorted(p for p in props if p.startswith(('tcp_', 'camera_', 'hand_', 'mount_')))
        if declared:
            print('    configurable properties: ' + ', '.join(declared))

    # --- 5. required frames ------------------------------------------------
    missing = [f for f in REQUIRED_FRAMES if f not in link_set]
    if missing:
        print('    PROBLEM missing required frames: ' + ', '.join(missing))
        problems += 1
    else:
        print('    required frames: all present')

    hints_missing = [h for h in REQUIRED_ROOT_HINTS if h not in link_set]
    if hints_missing:
        print('    NOTE no world/root frame declared (MoveIt virtual joint needs one): '
              + ', '.join(hints_missing))

    if problems == 0:
        print('    RESULT: OK')
    else:
        print(f'    RESULT: {problems} problem(s)')
    return problems


def main(argv):
    targets = argv[1:] or DEFAULT_TARGETS
    total = 0
    checked = 0
    for t in targets:
        if not os.path.isfile(t):
            print(f'--- {t}')
            print('    SKIP: file not found')
            continue
        checked += 1
        total += check_file(t)

    print()
    print(f'files checked: {checked}, problems: {total}')
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
