#!/usr/bin/env python3
"""Write the MoveIt Setup Assistant metadata file for the config package.

Why this file matters: moveit_configs_utils infers the URDF package from the
config package's NAME by convention, "X_moveit_config" -> "X_description". Our
names do not follow that (the URDF lives in dummy2_urdf_description), so this
file is what tells the loader where to look. Without it, any launch built on
MoveItConfigsBuilder fails at load time.

The timestamps are not decoration either: the Setup Assistant rewrites this file
when it saves, and a stale or missing one is the usual reason "the assistant does
not open my package".

Run once, from the repo root:
    python tools/write_setup_assistant.py
"""
import os
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
TARGET = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_moveit_config',
                      '.setup_assistant')

URDF_PACKAGE = 'dummy2_urdf_description'
URDF_RELATIVE = 'urdf/dummy2_urdf.xacro'
SRDF_RELATIVE = 'config/dummy2.srdf'

TEMPLATE = """# MoveIt Setup Assistant metadata.
#
# moveit_configs_utils infers the URDF package from THIS package's name by
# convention: "X_moveit_config" -> "X_description". Our names do not follow it
# (the URDF lives in {urdf_pkg}), so this file is what tells the loader where to
# actually look. Without it, any launch built on MoveItConfigsBuilder fails at
# load time with a missing robot_description.
#
# The timestamps below are not decoration: the Setup Assistant rewrites this
# file when it saves, and a config package with a stale or missing file is the
# usual reason "the assistant does not open my package".
moveit_setup_assistant_config:
  urdf:
    package: {urdf_pkg}
    relative_path: {urdf_rel}
    use_xacro: true
  srdf:
    relative_path: {srdf_rel}
  configuration:
    author_name: author
    author_email: todo@todo.com
    generated_timestamp: {ts:.6f}
"""


def main():
    now = time.time()
    text = TEMPLATE.format(urdf_pkg=URDF_PACKAGE, urdf_rel=URDF_RELATIVE,
                           srdf_rel=SRDF_RELATIVE, ts=now)
    os.makedirs(os.path.dirname(TARGET), exist_ok=True)
    with open(TARGET, 'w', encoding='utf-8', newline='\n') as fh:
        fh.write(text)
    print(f'wrote {TARGET}')
    print(f'  urdf package : {URDF_PACKAGE}')
    print(f'  urdf path    : {URDF_RELATIVE}')
    print(f'  srdf path    : {SRDF_RELATIVE}')
    print(f'  timestamp    : {now:.6f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
