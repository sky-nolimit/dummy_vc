#!/usr/bin/env python3
"""Check that the launch file will not refuse to build.

The launch file raises if the config declares a parameter that has no entry in
DESCRIPTIONS. That is a good check, but it only fires at launch time, on the
machine, after a colcon build -- so anything that can catch it earlier is worth
having.

This verifies:
  1. every parameter in the YAML has a DESCRIPTIONS entry (else launch raises)
  2. every launch argument the file declares corresponds to a YAML parameter
     or a known non-calibration flag
  3. the YAML block parses and is non-empty

Usage:
    python tools/check_launch_consistency.py
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
LAUNCH = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'launch', 'dummy_controller.launch.py')
CONFIG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                      'config', 'dummy_controller.yaml')

# Launch arguments that are not calibration parameters.
NON_PARAM_ARGS = {'activate', 'robot_state_publisher'}


def yaml_param_names(path):
    """Names of the keys directly under dummy_controller/ros__parameters.

    Deliberately a small hand rolled scan instead of importing yaml: this
    checker must run on Windows where PyYAML may be absent, and the file has a
    flat, regular structure.
    """
    names = []
    inside = False
    for line in open(path, encoding='utf-8').read().splitlines():
        if re.match(r'^\s*ros__parameters:\s*$', line):
            inside = True
            continue
        if not inside:
            continue
        m = re.match(r'^    ([A-Za-z_]\w*):', line)
        if m:
            names.append(m.group(1))
        elif line.strip() and not line.startswith('    '):
            break          # left the block
    return names


def descriptions_and_args(path):
    tree = ast.parse(open(path, encoding='utf-8').read())
    desc = None
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', None) == 'DESCRIPTIONS' for t in node.targets):
            desc = [k.value for k in node.value.keys]
    # DeclareLaunchArgument('name', ...) calls
    args = re.findall(r"DeclareLaunchArgument\(\s*'([^']+)'", 
                      open(path, encoding='utf-8').read())
    return desc, args


def main():
    print(f'config : {os.path.relpath(CONFIG, REPO)}')
    print(f'launch : {os.path.relpath(LAUNCH, REPO)}')
    print()

    params = yaml_param_names(CONFIG)
    if not params:
        print('FATAL: no parameters found in the YAML block. Either the file')
        print('       changed shape or ros__parameters is missing.')
        return 1
    print(f'  YAML parameters ({len(params)}):')
    for p in params:
        print(f'    {p}')

    desc, args = descriptions_and_args(LAUNCH)
    if not desc:
        print('FATAL: could not find the DESCRIPTIONS dict in the launch file')
        return 1
    # Do NOT try to enumerate DeclareLaunchArgument calls as a completeness
    # check: they are created in a loop over the config, with a variable name,
    # so any regex over call sites sees only the two literal ones and reports
    # false failures. The loop itself is the guarantee; what is worth checking
    # is that the loop is still driven by the config rather than a literal.
    ltext = open(LAUNCH, encoding='utf-8').read()
    print(f'  DESCRIPTIONS entries ({len(desc)})')
    print(f'  literal DeclareLaunchArgument calls: {sorted(args)} '
          '(the rest are created in a loop)')
    print()

    problems = []

    missing = sorted(set(params) - set(desc))
    if missing:
        problems.append(
            f'config declares {missing} but DESCRIPTIONS does not describe '
            'them; the launch file will raise at startup')

    if 'load_config_defaults' not in ltext:
        problems.append(
            'the launch file does not call load_config_defaults(); it has '
            'probably grown a second hardcoded copy of the calibration '
            'values, which would silently override the config')

    unexpected = sorted(set(args) - NON_PARAM_ARGS)
    if unexpected:
        problems.append(
            'launch declares literal calibration arguments, which duplicates '
            f'the config: {unexpected}. They should come from the config via '
            'load_config_defaults() so there is only one source of truth.')

    print('=' * 62)
    if problems:
        print(f'RESULT: {len(problems)} problem(s)')
        for p in problems:
            print(f'  - {p}')
        return 1
    print('RESULT: launch file and config are consistent')
    print(f'  {len(params)} parameters, all described, none duplicated')
    return 0


if __name__ == '__main__':
    sys.exit(main())
