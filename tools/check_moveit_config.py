#!/usr/bin/env python3
"""Cross-check the MoveIt configuration against the URDF and the device.

MoveIt configuration fails in ways that are hard to attribute from the outside,
because the failure usually appears as "planning failed" or "the arm did not
move", neither of which points at the cause. Most of them are name or number
mismatches that can be caught offline:

  * an SRDF link or joint name that is not in the URDF
  * a joint limit in joint_limits.yaml that disagrees with the URDF
  * a joint limit that is LOOSER than the device will actually accept, which lets
    MoveIt plan a motion the device then ignores silently
  * a controller listing a joint that does not exist
  * a planning group that has no entry in kinematics.yaml or ompl_planning.yaml
  * a group state naming a joint outside its group

Every check below is one of those. It needs no ROS and no hardware, so it can
run on the machine that edits the files.

    python tools/check_moveit_config.py
"""
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

PKG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_moveit_config')
CFG = os.path.join(PKG, 'config')
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf', 'urdf',
                     'dummy2_urdf.xacro')
DRIVER_CFG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                          'config', 'dummy_controller.yaml')

FAILURES = []
NOTES = []


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def note(msg):
    NOTES.append(msg)
    print(f'  [note] {msg}')


def get_list(text, key):
    m = re.search(rf'^\s*{key}\s*:\s*\[([^\]]*)\]', text, re.M)
    return [float(v) for v in m.group(1).split(',')] if m else None


def main():
    from yaml_min import load_file, YamlError

    # ---------------------------------------------------------------- URDF --
    print('=' * 74)
    print('1. URDF: parse and collect names')
    print('=' * 74)
    try:
        import fk_dummy
        links, joints = fk_dummy.parse_model(XACRO)
    except Exception as exc:  # noqa: BLE001
        fail(f'cannot parse the URDF: {exc}')
        return 1
    link_names = set(links)
    joint_by_name = {j['name']: j for j in joints}
    joint_names = set(joint_by_name)
    ok(f'{len(link_names)} links, {len(joint_names)} joints')

    movable = {n: j for n, j in joint_by_name.items()
               if j['type'] in ('revolute', 'continuous', 'prismatic')}
    ok(f'{len(movable)} movable joints: {sorted(movable)}')

    # ---------------------------------------------------------------- SRDF --
    print()
    print('=' * 74)
    print('2. SRDF: every name must exist in the URDF')
    print('=' * 74)
    srdf_path = os.path.join(CFG, 'dummy2.srdf')
    try:
        srdf = ET.parse(srdf_path).getroot()
    except Exception as exc:  # noqa: BLE001
        fail(f'cannot parse {os.path.basename(srdf_path)}: {exc}')
        return 1

    vjoints = srdf.findall('virtual_joint')
    if not vjoints:
        fail('no virtual_joint; MoveIt needs one to attach the arm to world')
    for vj in vjoints:
        p, c = vj.get('parent_frame'), vj.get('child_link')
        if c not in link_names:
            fail(f'virtual_joint child_link "{c}" is not a URDF link')
        if p not in link_names:
            fail(f'virtual_joint parent_frame "{p}" is not a URDF link')
        if p in link_names and c in link_names:
            ok(f'virtual_joint {p} -> {c}')

    groups = {}
    for g in srdf.findall('group'):
        name = g.get('name')
        members = set()
        chain = g.find('chain')
        if chain is not None:
            base, tip = chain.get('base_link'), chain.get('tip_link')
            for lk in (base, tip):
                if lk not in link_names:
                    fail(f'group "{name}": chain link "{lk}" is not a URDF link')
            # walk the chain so its joints are known
            cur, guard = tip, 0
            while cur != base and guard < 100:
                guard += 1
                j = next((x for x in joints if x['child'] == cur), None)
                if j is None:
                    fail(f'group "{name}": no joint leads from {base} to {tip}')
                    break
                members.add(j['name'])
                cur = j['parent']
            if guard >= 100:
                fail(f'group "{name}": chain walk did not terminate')
            ok(f'group "{name}": chain {base} -> {tip} '
               f'({len(members)} joints)')
        for jn in g.findall('joint'):
            jname = jn.get('name')
            members.add(jname)
            if jname not in joint_names:
                fail(f'group "{name}": joint "{jname}" is not a URDF joint')
        if chain is None:
            ok(f'group "{name}": {len(members)} joints by name')
        groups[name] = members

    for gs in srdf.findall('group_state'):
        gname, sname = gs.get('group'), gs.get('name')
        if gname not in groups:
            fail(f'group_state "{sname}" names unknown group "{gname}"')
            continue
        for jn in gs.findall('joint'):
            jname = jn.get('name')
            if jname not in groups[gname]:
                fail(f'group_state "{sname}": joint "{jname}" is not in group '
                     f'"{gname}"')
            try:
                float(jn.get('value'))
            except (TypeError, ValueError):
                fail(f'group_state "{sname}": joint "{jname}" has a '
                     f'non-numeric value')
        ok(f'group_state "{gname}/{sname}"')

    for ee in srdf.findall('end_effector'):
        for attr in ('parent_link',):
            v = ee.get(attr)
            if v and v not in link_names:
                fail(f'end_effector {attr} "{v}" is not a URDF link')
        g = ee.get('group')
        if g not in groups:
            fail(f'end_effector names unknown group "{g}"')
        pg = ee.get('parent_group')
        if pg and pg not in groups:
            fail(f'end_effector names unknown parent_group "{pg}"')
        ok(f'end_effector {ee.get("name")}')

    for dc in srdf.findall('disable_collisions'):
        for attr in ('link1', 'link2'):
            v = dc.get(attr)
            if v not in link_names:
                fail(f'disable_collisions names "{v}", not a URDF link')
        if dc.get('reason') is None:
            note(f'disable_collisions {dc.get("link1")}/{dc.get("link2")} has '
                 'no reason attribute')

    # --------------------------------------------------------- kinematics --
    print()
    print('=' * 74)
    print('3. kinematics.yaml and ompl_planning.yaml cover every group')
    print('=' * 74)
    for fname in ('kinematics.yaml', 'ompl_planning.yaml'):
        try:
            data = load_file(os.path.join(CFG, fname))
        except (YamlError, OSError) as exc:
            fail(f'cannot read {fname}: {exc}')
            continue
        if not isinstance(data, dict):
            fail(f'{fname} did not parse into a mapping')
            continue
        for g in groups:
            if g in data:
                ok(f'{fname} has an entry for group "{g}"')
            else:
                fail(f'{fname} has NO entry for group "{g}"; planning that '
                     'group will fail at load time')

    # ------------------------------------------------------- joint limits --
    print()
    print('=' * 74)
    print('4. joint limits: MoveIt vs URDF vs the measured device travel')
    print('=' * 74)
    try:
        jl = load_file(os.path.join(CFG, 'joint_limits.yaml'))['joint_limits']
    except (YamlError, OSError, KeyError, TypeError) as exc:
        fail(f'cannot read joint_limits.yaml: {exc}')
        jl = {}

    drv = open(DRIVER_CFG, encoding='utf-8').read()
    sign = get_list(drv, 'joint_sign')
    zero = get_list(drv, 'movej_urdf_zero_deg')
    rng = get_list(drv, 'movej_accepted_range_deg')
    have_device = all(v is not None for v in (sign, zero, rng)) \
        and len(rng) == 12
    if not have_device:
        note('driver config lacks the calibration values; skipping the '
             'device comparison')

    print(f'  {"joint":<18} {"MoveIt (deg)":>22} {"URDF (deg)":>22}'
          f' {"device (deg)":>22}')
    for i in range(1, 7):
        name = f'dummy2_joint{i}'
        if name not in movable:
            fail(f'{name} is not a movable joint in the URDF')
            continue
        u = movable[name].get('limits')
        if u is None:
            fail(f'{name} has no URDF limits')
            continue
        ulo, uhi = math.degrees(u[0]), math.degrees(u[1])

        entry = jl.get(name, {})
        mlo = entry.get('min_position')
        mhi = entry.get('max_position')
        if mlo is None or mhi is None:
            fail(f'joint_limits.yaml has no position limits for {name}')
            continue
        mlo, mhi = math.degrees(mlo), math.degrees(mhi)

        if have_device:
            a = sign[i - 1] * (rng[2 * (i - 1)] - zero[i - 1])
            b = sign[i - 1] * (rng[2 * (i - 1) + 1] - zero[i - 1])
            dlo, dhi = min(a, b), max(a, b)
            dtxt = f'{dlo:>10.1f}..{dhi:<10.1f}'
        else:
            dlo = dhi = None
            dtxt = f'{"?":>22}'

        print(f'  {name:<18} {mlo:>10.1f}..{mhi:<10.1f} '
              f'{ulo:>10.1f}..{uhi:<10.1f} {dtxt}')

        if abs(mlo - ulo) > 0.5 or abs(mhi - uhi) > 0.5:
            fail(f'{name}: MoveIt limits [{mlo:.1f}, {mhi:.1f}] disagree with '
                 f'the URDF [{ulo:.1f}, {uhi:.1f}]. MoveIt plans with its own '
                 'copy, so the URDF would not constrain it.')
        if dlo is not None and (mlo < dlo - 1.0 or mhi > dhi + 1.0):
            extra = []
            if mlo < dlo - 1.0:
                extra.append(f'lower {mlo:.1f} < device {dlo:.1f}')
            if mhi > dhi + 1.0:
                extra.append(f'upper {mhi:.1f} > device {dhi:.1f}')
            fail(f'{name}: MoveIt limits exceed the device travel '
                 f'({"; ".join(extra)}). The device drops a command containing '
                 'a value it will not act on, so MoveIt could plan a motion the '
                 'arm then ignores silently.')

    # -------------------------------------------------- trajectory control --
    print()
    print('=' * 74)
    print('5. controller joints exist, and the topic matches the driver')
    print('=' * 74)
    try:
        mc = load_file(os.path.join(CFG, 'moveit_controllers.yaml'))
        scm = mc['moveit_simple_controller_manager']
    except (YamlError, OSError, KeyError, TypeError) as exc:
        fail(f'cannot read moveit_controllers.yaml: {exc}')
        scm = None
    if scm:
        names = scm.get('controller_names') or []
        if not names:
            fail('moveit_simple_controller_manager lists no controllers')
        for cn in names:
            entry = scm.get(cn)
            if not isinstance(entry, dict):
                fail(f'controller "{cn}" is listed but has no definition')
                continue
            for jn in entry.get('joints') or []:
                if jn not in joint_names:
                    fail(f'controller "{cn}" lists joint "{jn}", which is not '
                         'a URDF joint')
            ok(f'controller "{cn}": {len(entry.get("joints") or [])} joints, '
               f'type {entry.get("type")}')

        # The driver's subscription topic is the thing that actually has to
        # match. `JointTrajectoryControllerHandle` publishes to
        # /<controller_name>/joint_trajectory, so the controller NAME has to be
        # the first path segment of the driver's topic. This is worth checking
        # rather than noting: a mismatch publishes the planned trajectory into
        # the void, MoveIt reports success, and the arm simply does not move.
        drv_py = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller',
                              'dummy_controller', 'dummy_controller.py')
        src = open(drv_py, encoding='utf-8').read()
        m = re.search(r"create_subscription\(\s*JointTrajectory,\s*'([^']+)'",
                      src)
        if not m:
            note('could not find the driver\'s JointTrajectory subscription, '
                 'so the controller name was not checked against it')
        else:
            topic = m.group(1)
            want = topic.strip('/').split('/')[0]
            ok(f'driver subscribes to {topic}')
            if want in names:
                ok(f'controller "{want}" publishes to /{want}/joint_trajectory, '
                   'which is the driver topic')
            else:
                fail(f'the driver listens on {topic}, which implies a '
                     f'controller named "{want}", but moveit_controllers.yaml '
                     f'defines {names}. The planned trajectory would be '
                     'published to a topic nobody reads: MoveIt reports '
                     'success and the arm does not move.')

    # ------------------------------------------------- setup assistant file --
    #
    # Two separate things are checked here, because getting the first right and
    # the second wrong is exactly the bug that cost a long debugging session:
    #   1. the file's CONTENT must point at the real URDF package/path;
    #   2. the file must be INSTALLED to the share root, which is the only place
    #      MoveItConfigsBuilder looks on Humble. `install(DIRECTORY config ...)`
    #      puts it under share/config/, where the builder never looks, and it
    #      then silently falls back to a config/dummy2.urdf that does not exist.
    print()
    print('=' * 74)
    print('6. .setup_assistant: content AND installed location')
    print('=' * 74)
    sa = os.path.join(PKG, 'config', '.setup_assistant')
    if not os.path.exists(sa):
        fail('config/.setup_assistant is missing; MoveItConfigsBuilder will '
             'infer the URDF package from the package NAME and fail, because '
             '"dummy_vc_moveit_config" implies "dummy_vc_description"')
    else:
        text = open(sa, encoding='utf-8').read()
        m = re.search(r'package:\s*(\S+)', text)
        r = re.search(r'relative_path:\s*(\S+)', text)
        if not m or not r:
            fail('.setup_assistant lacks urdf package or relative_path')
        else:
            upkg, urel = m.group(1), r.group(1)
            if upkg != 'dummy2_urdf_description':
                fail(f'.setup_assistant points at package "{upkg}", expected '
                     '"dummy2_urdf_description"')
            else:
                ok(f'.setup_assistant urdf: {upkg}/{urel}')
            if r.group(1) and not re.search(r'use_xacro:\s*true', text):
                note('.setup_assistant does not set use_xacro: true, but the '
                     'description file is a .xacro')

    # The install rule. Without it the file never reaches the share root.
    cmake = os.path.join(PKG, 'CMakeLists.txt')
    cmake_text = open(cmake, encoding='utf-8').read() if os.path.exists(cmake) else ''
    if re.search(r'install\(\s*FILES\s+config/\.setup_assistant', cmake_text):
        ok('CMakeLists.txt installs config/.setup_assistant to the share root')
    else:
        fail('CMakeLists.txt does not `install(FILES config/.setup_assistant '
             'DESTINATION share/${PROJECT_NAME})`. Without it the file lands '
             'at share/config/.setup_assistant, where MoveItConfigsBuilder '
             'never looks on Humble, and robot_description ends up empty.')

    # The launch file must not depend on that lookup at all any more.
    launch = os.path.join(PKG, 'launch', 'demo.launch.py')
    launch_text = (open(launch, encoding='utf-8').read()
                   if os.path.exists(launch) else '')
    if re.search(r'\.robot_description\(\s*\n?\s*file_path=', launch_text):
        ok('demo.launch.py passes the URDF path explicitly')
    else:
        fail('demo.launch.py does not pass robot_description(file_path=...) '
             'explicitly; it is relying on the .setup_assistant inference that '
             'silently falls back to a non-existent config/dummy2.urdf')

    print()
    print('=' * 74)
    if FAILURES:
        print(f'RESULT: {len(FAILURES)} PROBLEM(S)')
        for f in FAILURES:
            print(f'  - {f}')
        return 1
    print('RESULT: the MoveIt configuration is consistent with the URDF')
    if NOTES:
        print(f'        ({len(NOTES)} note(s) above need a human decision)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
