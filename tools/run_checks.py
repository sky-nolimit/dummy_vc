#!/usr/bin/env python3
"""Run every offline check and give one verdict.

These are the checks that need neither hardware nor ROS running. They are the
ones to run before committing, and the ones that would have caught several of
this project's worst bugs:

    check_downlink        the down-link against the measured (move_j, raw)
                          pairs. Caught j3's target being computed as -1.474
                          when the device only accepts [45, 180] there.
    check_conversions     the real conversion functions, imported from the
                          driver, checked against each other. Caught a shortcut
                          that disagreed with the two-step chain.
    selftest_conversion   the calibration: anchor, retracted-pose fold, round
                          trips, and that the three copies of the defaults agree.
    selftest_wait         the arm-waiting logic, against simulated devices.
                          Waiting bugs produce confident wrong answers, so this
                          runs without hardware on purpose.
    selftest_verify_moveit the MoveIt verifier's own call sites: tuple arity,
                          and that every `check.<method>` exists. That script
                          only ever runs on the VM, where a typo costs a whole
                          round trip and shows up as a meaningless ValueError.
    selftest_check_rviz_plugin
                          the RViz plugin diagnostic against four fake ROS
                          prefixes, because it too only ever runs on the VM --
                          and when it is wrong it reports "zero panel plugins",
                          which is indistinguishable from the fault it is there
                          to diagnose.
    selftest_follow_joint_trajectory
                          the FollowJointTrajectory contract MoveIt executes
                          through: the action name, that the server is really
                          constructed in on_activate, that the execute callback
                          returns instead of yielding, and that the executor
                          has enough threads to keep publishing /joint_states
                          while the arm moves. All of it offline, from the AST
                          and the config.
    selftest_vision_geometry
                          the vision geometry: the axis conventions of a
                          top-down grasp, the pinhole/distortion round trip,
                          the table back-projection, and how far the monocular
                          fallback is off when the object has height. All three
                          failures look identical from the outside ("the grasp
                          point is a few millimetres off"), and all three are
                          cheap to pin down with synthetic data -- so they are
                          pinned down here rather than on the arm. It also
                          enforces that the geometry layer imports no ROS and
                          no OpenCV, which is why it can run at all.
    selftest_grasp_task
                           the stage-5 grasp planner and task state machine: every
                           state has an action and a way forward, the retryable
                           steps retry to their limit, the two gripper steps
                           never retry, and an uncalibrated gripper stops the
                           run right before CLOSE_GRIPPER instead of sending a
                           hand command. Same reason as above: on the arm each
                           failure costs a re-boot and a re-placed block, while
                           the transitions are pure functions that can all be
                           walked here.

Usage:
    python tools/run_checks.py            # everything
    python tools/run_checks.py --quick    # skip the slow ones
"""
import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

def safe(text):
    """Drop characters the console encoding cannot render.

    On Windows the default console codec is often GBK, and a subprocess's
    captured output can contain bytes replaced by U+FFFD. Printing that raises
    UnicodeEncodeError and hides the report that was the whole point.
    """
    enc = sys.stdout.encoding or 'utf-8'
    return text.encode(enc, errors='replace').decode(enc, errors='replace')


def tracked_files():
    """Every tracked text file, so check_encoding gets an explicit list.

    `check_encoding.py` takes file arguments and prints its usage when given
    none, so it cannot be invoked bare.
    """
    exts = {'.py', '.sh', '.ps1', '.yaml', '.yml', '.xacro', '.urdf', '.xml',
            '.md', '.txt', '.launch', '.cfg', '.json'}
    proc = subprocess.run(['git', 'ls-files'], cwd=REPO, capture_output=True,
                          text=True, encoding='utf-8', errors='replace')
    paths = []
    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line:
            continue
        if os.path.splitext(line)[1].lower() in exts:
            full = os.path.join(REPO, line)
            if os.path.isfile(full):
                paths.append(line)
    return paths


# (script, one-line purpose, quick, extra args)
CHECKS = [
    ('check_syntax.py',
     'every Python file parses', True, []),
    ('check_encoding.py',
     'no BOM, no mojibake', True, 'FILES'),
    ('check_line_endings.py',
     'no CRLF in the git objects Linux will receive', True, []),
    ('check_urdf.py',
     'URDF is valid, the joint tree is connected', True, []),
    ('check_urdf_limits.py',
     'URDF limits do not exceed the measured device travel', True, []),
    ('check_moveit_config.py',
     'the MoveIt config agrees with the URDF and the device', True, []),
    ('check_launch_consistency.py',
     'launch will not raise on an undocumented config key', True, []),
    ('check_conversions.py',
     'the conversion functions agree with each other', True, []),
    ('check_downlink.py',
     'the down-link reproduces the measured poses and stays in range', False,
     []),
    ('selftest_conversion.py',
     'the calibration holds: anchor, fold, round trips, defaults', False, []),
    ('selftest_wait.py',
     'the waiting logic behaves on simulated slow devices', False, []),
    ('selftest_verify_moveit.py',
     'the MoveIt verifier unpacks its own helpers correctly', True, []),
    ('selftest_check_rviz_plugin.py',
     'the RViz plugin diagnostic reads the ament plugin index right', True, []),
    ('selftest_follow_joint_trajectory.py',
     'the MoveIt action contract: name, server, execute, executor', True, []),
    ('selftest_vision_geometry.py',
     'the vision geometry: axis conventions, distortion, table projection', True,
     []),
    ('selftest_grasp_task.py',
     'the grasp planner and task machine: retries, gripper gate, no dead ends',
     True, []),
]


def run(script, extra):
    path = os.path.join(HERE, script)
    if not os.path.exists(path):
        return None, '', f'missing: {script}'
    cmd = [sys.executable, path] + list(extra)
    proc = subprocess.run(cmd, cwd=REPO, capture_output=True, text=True,
                          encoding='utf-8', errors='replace')
    return proc.returncode, proc.stdout, proc.stderr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--quick', action='store_true',
                    help='skip the slow checks')
    ap.add_argument('-v', '--verbose', action='store_true',
                    help='print each check\'s full output')
    args = ap.parse_args()

    print('=' * 74)
    print('OFFLINE CHECKS')
    print('=' * 74)
    print()

    failed = []
    skipped = []
    files = tracked_files()
    for script, purpose, is_quick, extra in CHECKS:
        if args.quick and not is_quick:
            skipped.append(script)
            print(f'  [skip] {script:<30} {purpose}')
            continue
        cmd_extra = files if extra == 'FILES' else extra
        code, out, err = run(script, cmd_extra)
        if code is None:
            failed.append((script, err))
            print(f'  [MISS] {script:<30} {purpose}')
            continue
        if code == 0:
            print(f'  [ ok ] {script:<30} {purpose}')
        else:
            failed.append((script, (out + err).strip()))
            print(f'  [FAIL] {script:<30} {purpose}')

        # echo the script's own verdict line when it has one
        for line in out.splitlines():
            s = line.strip()
            if s.startswith('RESULT:') or s.startswith('SUMMARY'):
                print(f'         {safe(s)}')
        if args.verbose and code != 0:
            for line in (out + err).splitlines():
                print(f'         | {safe(line)}')

    print()
    print('=' * 74)
    if not failed:
        print(f'ALL {len(CHECKS) - len(skipped)} CHECKS PASSED')
        if skipped:
            print(f'({len(skipped)} skipped by --quick)')
        print()
        print('Next: with the driver running, use tools/final_test.py for the')
        print('hardware acceptance test.')
        return 0
    print(f'{len(failed)} CHECK(S) FAILED')
    print()
    for script, detail in failed:
        print(f'  --- {script}')
        for line in detail.splitlines()[-12:]:
            print(f'      {line}')
    return 1


if __name__ == '__main__':
    sys.exit(main())
