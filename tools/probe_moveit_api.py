#!/usr/bin/env python3
"""Print the real moveit_configs_utils API on THIS machine.

Why: MoveItConfigsBuilder's method signatures differ between MoveIt releases,
and guessing one costs a full launch attempt each time. This asks the installed
package directly, so the next edit is based on fact.

Also prints where the builder looks for .setup_assistant, because "Cannot infer
URDF/SRDF" means it did not find it, and the location it searches is not
documented clearly.

    source /opt/ros/humble/setup.bash
    source dummy_vc_ws/install/setup.bash
    python3 tools/probe_moveit_api.py
"""
import inspect
import os
import sys

TARGET_PKG = 'dummy_vc_moveit_config'


def main():
    print('=' * 74)
    print('moveit_configs_utils: real signatures')
    print('=' * 74)
    try:
        from moveit_configs_utils import MoveItConfigsBuilder
    except Exception as exc:  # noqa: BLE001
        print(f'FATAL: cannot import MoveItConfigsBuilder ({exc})')
        print('       Is MoveIt installed and the workspace sourced?')
        return 1

    print(f'  module file: {inspect.getfile(MoveItConfigsBuilder)}')
    try:
        import moveit_configs_utils
        print(f'  version    : '
              f'{getattr(moveit_configs_utils, "__version__", "unknown")}')
    except Exception:  # noqa: BLE001
        pass
    print()

    # Construction
    print('  MoveItConfigsBuilder.__init__')
    print(f'    {inspect.signature(MoveItConfigsBuilder.__init__)}')
    print()

    # Every public method that returns config, with its signature. These are
    # the ones used to point the builder at our file layout.
    print('  public methods that can be chained:')
    for name in sorted(dir(MoveItConfigsBuilder)):
        if name.startswith('_'):
            continue
        attr = getattr(MoveItConfigsBuilder, name)
        if not callable(attr):
            continue
        try:
            sig = inspect.signature(attr)
        except (TypeError, ValueError):
            sig = '(?)'
        print(f'    {name}{sig}')
    print()

    # Where does it look for the setup assistant file?
    print('=' * 74)
    print('where the builder looks for .setup_assistant')
    print('=' * 74)
    try:
        src = inspect.getsource(MoveItConfigsBuilder)
    except (OSError, TypeError) as exc:
        print(f'  cannot read the source ({exc}); skipping')
        return 0
    lines = src.splitlines()
    for i, line in enumerate(lines):
        if 'setup_assistant' in line.lower() or 'Cannot infer' in line:
            lo = max(0, i - 3)
            hi = min(len(lines), i + 4)
            print(f'  --- around line {i + 1} of the class body')
            for j in range(lo, hi):
                print(f'    {lines[j]}')
            print()

    # And check our own package for the file in each place it might look.
    print('=' * 74)
    print(f'our package: {TARGET_PKG}')
    print('=' * 74)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(TARGET_PKG)
        print(f'  share dir: {share}')
        for rel in ('.setup_assistant', 'config/.setup_assistant',
                    'config/dummy2.srdf', 'config/joint_limits.yaml',
                    'config/moveit_controllers.yaml',
                    'config/planning_scene_monitor_parameters.yaml',
                    'config/ompl_planning.yaml', 'config/kinematics.yaml'):
            p = os.path.join(share, rel)
            print(f'    {"OK  " if os.path.exists(p) else "MISS"} {rel}')
    except Exception as exc:  # noqa: BLE001
        print(f'  cannot locate the package ({exc})')
        print('  is the workspace built and sourced?')
    print()
    print('=' * 74)
    print('NEXT')
    print('=' * 74)
    print('  Send this output back. The two things needed from it:')
    print('    1. the real signature of planning_scene_monitor (and whether a')
    print('       method for it exists at all in this release)')
    print('    2. where the builder expects .setup_assistant, so the URDF and')
    print('       SRDF stop being inferred wrongly')
    return 0


if __name__ == '__main__':
    sys.exit(main())
