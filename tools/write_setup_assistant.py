#!/usr/bin/env python3
"""Write the MoveIt Setup Assistant metadata file for the config package.

Why this file matters: moveit_configs_utils infers the URDF package from the
config package's NAME by convention, "X_moveit_config" -> "X_description". Our
names do not follow that (the URDF lives in dummy2_urdf_description), so this
file is what tells the loader where to look. Without it, any launch built on
MoveItConfigsBuilder fails at load time.

WHERE it has to end up is the part that cost us a long debugging session.
MoveItConfigsBuilder on Humble looks for it at

    <share>/dummy_vc_moveit_config/.setup_assistant      <- share ROOT

and NOWHERE ELSE. `install(DIRECTORY config launch ...)` in the package's
CMakeLists.txt puts it at `<share>/config/.setup_assistant`, where the builder
never looks. It then logs `Cannot infer URDF from <share> -- using
config/dummy2.urdf`, that file does not exist either, and robot_description ends
up empty -- surfacing as three errors that mention neither the URDF nor the
lookup. So the source file lives in `config/` (the Setup Assistant's own habit)
and CMakeLists.txt installs it to the share root with an explicit
`install(FILES ...)`. This script therefore writes ONE file.

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
PKG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_moveit_config')

# ONE location, and it is inside config/ -- CMakeLists.txt installs it from here
# to the share root, which is the only place the builder looks. An earlier
# version wrote a second copy to the package root to "cover both cases"; that
# was wrong on two counts: the builder never reads `config/`, and a file that
# exists in one place but not the installed one is exactly the confusion this
# change removes.
TARGETS = [
    os.path.join(PKG, 'config', '.setup_assistant'),
]

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
    for target in TARGETS:
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(text)
        print(f'wrote {target}')
    print(f'  urdf package : {URDF_PACKAGE}')
    print(f'  urdf path    : {URDF_RELATIVE}')
    print(f'  srdf path    : {SRDF_RELATIVE}')
    print(f'  timestamp    : {now:.6f}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
