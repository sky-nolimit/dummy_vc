#!/usr/bin/env python3
"""离线自查 FollowJointTrajectory 的接口契约。

它要防的故障从外面看只有一句话：

    Action client not connected to action server:
    joint_trajectory_controller/follow_joint_trajectory

而这句话**不代表它字面上的任何意思**：MoveIt 是活的，驱动是活的，网络也是
通的。缺的只是驱动侧一个 action server，且它的名字要和 MoveIt 拼出来的那个
逐字相同：

    /<controller_name>/<action_ns>

注意这里**只有两段**。`action_ns` 通常写成 `follow_joint_trajectory`，所以
名字看起来像 `/joint_trajectory_controller/follow_joint_trajectory` —— 于是
很容易以为还要再补一个 `follow_joint_trajectory`，写成
`.../follow_joint_trajectory/follow_joint_trajectory`。两种写法报的错一模一样，
本脚本因此把"重复了一段"单独判一次。

下面每一条检查都是这条路走不通的一种方式。它们都不需要 ROS：驱动源码走
**AST** 而不是正则——正则的话，一个只出现在注释里的名字也会被当成"已实现"，
而这正是我上一次踩的坑。

    python tools/selftest_follow_joint_trajectory.py
"""
import ast
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

DRIVER = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_controller')
DRIVER_PY = os.path.join(DRIVER, 'dummy_controller', 'dummy_controller.py')
DRIVER_CFG = os.path.join(DRIVER, 'config', 'dummy_controller.yaml')
DRIVER_LAUNCH = os.path.join(DRIVER, 'launch', 'dummy_controller.launch.py')
DRIVER_XML = os.path.join(DRIVER, 'package.xml')
MOVEIT_PKG = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_moveit_config')
CONTROLLERS = os.path.join(MOVEIT_PKG, 'config', 'moveit_controllers.yaml')

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


def read(path):
    """Read a file, or die trying. A missing file is a failure, not a crash."""
    try:
        with open(path, encoding='utf-8') as fh:
            return fh.read()
    except OSError as exc:
        fail(f'cannot read {path}: {exc}')
        return ''


def classes_and_funcs(tree):
    """{name: node} for every top-level class and every method inside one.

    Methods are keyed by their bare name. The driver has no two methods with
    the same name in different classes, and this keeps lookups readable.
    """
    found = {}
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            found.setdefault(node.name, node)
    return found


def assigned_names(func):
    """Names bound with `self.<name> = ...` anywhere inside a function."""
    names = set()
    for node in ast.walk(func):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, (ast.AugAssign, ast.AnnAssign)):
            targets = [node.target]
        for tgt in targets:
            if (isinstance(tgt, ast.Attribute)
                    and isinstance(tgt.value, ast.Name)
                    and tgt.value.id == 'self'):
                names.add(tgt.attr)
    return names


def yields_anywhere(func):
    return any(isinstance(n, (ast.Yield, ast.YieldFrom)) for n in ast.walk(func))


def calls_named(func, name):
    """True if `name` appears as a call target or as a callback argument.

    The second half matters: the interesting line is

        ActionServer(self, FollowJointTrajectory, self.trajectory_action_name,
                     execute_callback=self._on_traj_execute, ...)

    and there `self._on_traj_execute` is an Attribute in a keyword *value*,
    not a Call. Checking only call targets made this test report "no execute
    callback" on a server that has one - a false FAIL is worse than no test,
    because it teaches you to ignore the output.
    """
    for node in ast.walk(func):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        if isinstance(fn, ast.Name) and fn.id == name:
            return True
        if isinstance(fn, ast.Attribute) and fn.attr == name:
            return True
        for kw in node.keywords:
            val = kw.value
            if isinstance(val, ast.Name) and val.id == name:
                return True
            if isinstance(val, ast.Attribute) and val.attr == name:
                return True
    return False


def clears_flag_in_finally(func, flag):
    """True if `try: ... finally:` inside func assigns `self.<flag> = False`.

    Looks for a flag that is *set* but only cleared on the happy path. That is
    not a style question: a guard like

        if self._traj_executing: reject
        self._traj_executing = True

    latches on forever when the path that set it never clears it, and the only
    symptom is that unrelated later code starts refusing to work.
    """
    for node in ast.walk(func):
        if not isinstance(node, ast.Try):
            continue
        for stmt in node.finalbody:
            for sub in ast.walk(stmt):
                if not isinstance(sub, ast.Assign):
                    continue
                if not (isinstance(sub.value, ast.Constant)
                        and sub.value.value is False):
                    continue
                for tgt in sub.targets:
                    if isinstance(tgt, ast.Attribute) and tgt.attr == flag:
                        return True
    return False


def yaml_scalar(text, key):
    """Value of a top-level `key: scalar` line, or None.

    Deliberately not yaml_min.load(): dummy_controller.yaml contains a
    multi-line flow list (`movej_accepted_range_deg`), which the repository's
    minimal parser refuses. That refusal is correct for a full parser, but it
    would make this script fail for a reason that has nothing to do with the
    action contract. Every parameter read here is a plain scalar, so a
    line-wise read is both sufficient and immune to that.
    """
    m = re.search(rf"^\s*{re.escape(key)}\s*:\s*(.+?)\s*$", text, re.M)
    if not m:
        return None
    value = m.group(1).split('#')[0].strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in '\'"':
        value = value[1:-1]
    return value


# ----------------------------------------------------------------------
# 1. Der Name, den der Treiber oeffnet, gegen den Namen, den MoveIt sucht
# ----------------------------------------------------------------------

def check_action_name(src_text):
    print('\n[1] action name: driver default vs moveit_controllers.yaml')

    cfg_text = read(DRIVER_CFG)
    name = yaml_scalar(cfg_text, 'trajectory_action_name')
    if not name:
        fail(f'{DRIVER_CFG}: trajectory_action_name is missing or empty '
             f'(got {name!r})')
        return None
    ok(f'driver trajectory_action_name = {name}')

    if not name.startswith('/'):
        fail(f'{DRIVER_CFG}: trajectory_action_name {name!r} is not absolute; '
             'the MoveIt plugin always looks for /<controller>/<action_ns>')
    # Only two parts: the controller name and the action namespace. The
    # "follow_joint_trajectory" people expect as a third part IS the action_ns
    # - writing it twice is the single easiest way to get this wrong, and the
    # error message reads exactly the same, so it is checked explicitly.
    parts = [p for p in name.split('/') if p]
    if len(parts) == 3 and parts[1] == parts[2]:
        fail(f'{DRIVER_CFG}: {name!r} repeats {parts[2]!r} twice. The action '
             'name is /<controller_name>/<action_ns> - it is NOT suffixed with '
             f'follow_joint_trajectory again. Use /{parts[0]}/{parts[1]}')
    elif len(parts) != 2:
        fail(f'{DRIVER_CFG}: {name!r} has {len(parts)} name parts, expected 2 '
             '(/<controller_name>/<action_ns>)')

    ctl = read(CONTROLLERS)
    # The list is written as a YAML block sequence:
    #     controller_names:
    #       - joint_trajectory_controller
    m = re.search(r'^\s*controller_names\s*:\s*\n((?:\s*-\s*\S+\n?)+)',
                  ctl, re.M)
    if m:
        controller = m.group(1).strip().splitlines()[0].lstrip('- ').strip()
    else:
        m = re.search(r'^\s*controller_names\s*:\s*\[([^\]]*)\]', ctl, re.M)
        controller = (m.group(1).split(',')[0].strip().strip('\'"')
                      if m else None)
    if not controller:
        fail(f'{CONTROLLERS}: no controller_names entry found')
        return name
    ok(f'moveit names its controller {controller!r}')

    m = re.search(r'^\s*action_ns\s*:\s*(\S+)', ctl, re.M)
    if not m:
        fail(f'{CONTROLLERS}: no action_ns found')
        return name
    action_ns = m.group(1).strip().strip('\'"')

    expected = f'/{controller}/{action_ns}'
    if name == expected:
        ok(f'name matches MoveIt exactly: {expected}')
    else:
        fail(f'name mismatch. driver opens {name!r}, MoveIt looks for '
             f'{expected!r} (controller_names[0] + action_ns). The error text '
             'is identical for every kind of mismatch, so this is the first '
             'thing to check.')
    return name


# ----------------------------------------------------------------------
# 2. Ist der Server wirklich da, und zwar im Quelltext (AST), nicht im Kommentar
# ----------------------------------------------------------------------

def check_server_construction(src_text, tree, funcs, name):
    print('\n[2] action server is actually constructed')

    init = funcs.get('init_trajectory_action_server')
    if init is None:
        fail('dummy_controller.py defines no init_trajectory_action_server()')
        return
    ok('init_trajectory_action_server() exists')

    if not calls_named(init, 'ActionServer'):
        fail('init_trajectory_action_server() never calls ActionServer(...)')
    else:
        ok('it calls ActionServer(...)')

    # The real bug this catches: a version of this file that imports
    # ActionServer, mentions the action name everywhere and never builds one.
    if 'FollowJointTrajectory' not in src_text:
        fail('FollowJointTrajectory is not mentioned in the driver at all')

    activate = funcs.get('on_activate')
    if activate is None:
        fail('on_activate() not found')
    elif not calls_named(activate, 'init_trajectory_action_server'):
        fail('on_activate() never calls init_trajectory_action_server(), so the '
             'server only exists if something else builds it')
    else:
        ok('on_activate() builds the server')

    # A server that is created but never given an execute callback accepts
    # goals and then hangs, which from MoveIt looks like a timeout.
    if not calls_named(init, '_on_traj_execute'):
        fail('the ActionServer is created without execute_callback')
    else:
        ok('the ActionServer gets execute_callback')

    deactivate_cleanup = funcs.get('reset_pub_sub')
    if deactivate_cleanup is None:
        note('reset_pub_sub() not found; cannot check that the server is '
             'destroyed on deactivate')
    elif not calls_named(deactivate_cleanup, 'destroy'):
        fail('reset_pub_sub() never calls destroy(); a same-named server left '
             'in DDS makes the next activate create a second one')
    else:
        ok('reset_pub_sub() destroys the server')


# ----------------------------------------------------------------------
# 3. Die drei Fallen, die aus "laeuft" ein "haengt" machen
# ----------------------------------------------------------------------

def check_execute_path(funcs):
    print('\n[3] execute path: return values, locking, cancellation')

    exec_fn = funcs.get('_on_traj_execute')
    if exec_fn is None:
        fail('_on_traj_execute() not found')
        return

    if not yields_anywhere(exec_fn):
        ok('execute callback is a plain function (no `yield`)')
    else:
        fail('execute callback contains `yield`. rclpy then treats it as a '
             'coroutine and the goal never reaches a terminal state; this is '
             'the classic "Execute does nothing" version of this bug.')

    # Busy flag handling. `_traj_busy()` must read the flag under the lock,
    # otherwise the two entry points (action and topic) race.
    busy = funcs.get('_traj_busy')
    if busy is None:
        fail('_traj_busy() not found')
    elif not calls_named(busy, '_traj_lock'):
        # `with self._traj_lock:` is a With statement, not a Call.
        withs = [n for n in ast.walk(busy) if isinstance(n, ast.With)]
        held = any(
            isinstance(item.context_expr, ast.Attribute)
            and item.context_expr.attr == '_traj_lock'
            for w in withs for item in w.items)
        if held:
            ok('_traj_busy() reads the flag under _traj_lock')
        else:
            fail('_traj_busy() reads _traj_executing without holding '
                 '_traj_lock; the action and topic entry points can then both '
                 'think they own the arm')
    else:
        ok('_traj_busy() touches _traj_lock')

    if not any(isinstance(n, ast.With) for n in ast.walk(exec_fn)):
        fail('_on_traj_execute() never takes a lock; the busy flag is then '
             'read-modify-write raced against the topic entry point')
    else:
        ok('_on_traj_execute() takes a lock around the busy flag')

    if not calls_named(exec_fn, '_traj_goal_was_cancelled'):
        note('_on_traj_execute() does not check for a cancel request; a '
             'cancel arriving before the thread starts is then ignored')
    else:
        ok('_on_traj_execute() checks for cancellation')

    if not calls_named(exec_fn, 'join'):
        fail('_on_traj_execute() does not join its worker thread; the result '
             'is then reported before the motion is finished')
    else:
        ok('_on_traj_execute() joins the worker thread')

    # The busy flag has to be cleared on EVERY exit, not only on the happy path.
    # This exact omission shipped: the action entry point set _traj_executing
    # and never cleared it, so after the first Execute the node was "busy"
    # forever -- later goals were rejected, and home/reset were silently
    # refused, which is what stopped the arm retracting at shutdown before the
    # motors were switched off.
    for fn_name, fn in (('_on_traj_execute', exec_fn),
                        ('joints_callback', funcs.get('joints_callback'))):
        if fn is None:
            continue
        if clears_flag_in_finally(fn, '_traj_executing'):
            ok(f'{fn_name}() clears _traj_executing in a `finally`')
        else:
            fail(f'{fn_name}() sets _traj_executing but never clears it in a '
                 '`finally`. The flag then latches on after the first '
                 'trajectory: every later goal is rejected as "already '
                 'executing", and home/reset/named poses are silently refused '
                 'by _send_named_pose -- which is what stops the arm from '
                 'retracting at shutdown before the motors are switched off.')

    # --- _execute_trajectory: every exit must carry an error code ---------
    ex = funcs.get('_execute_trajectory')
    if ex is None:
        fail('_execute_trajectory() not found')
        return

    bare = [n for n in ast.walk(ex)
            if isinstance(n, ast.Return) and n.value is None]
    if bare:
        line = min(n.lineno for n in bare)
        fail(f'_execute_trajectory() has a bare `return` at line {line}. A '
             'bare return gives the caller None, which unpacks into the '
             'wrong number of values or raises; the caller then cannot tell '
             '"finished" from "refused" and the goal aborts with a bogus code.')
    else:
        ok('_execute_trajectory() returns a value on every path')

    if not calls_named(ex, 'Result'):
        # Result constants are looked up as FollowJointTrajectory.Result.X,
        # which is an Attribute, not a Call, so look for the attribute chain.
        has_result = any(
            isinstance(n, ast.Attribute) and n.attr == 'Result'
            for n in ast.walk(ex))
        if has_result:
            ok('_execute_trajectory() uses FollowJointTrajectory.Result codes')
        else:
            fail('_execute_trajectory() never uses a FollowJointTrajectory.'
                 'Result constant; hand-written numbers do not match the '
                 'action definition and MoveIt reports the wrong failure')
    else:
        ok('_execute_trajectory() uses FollowJointTrajectory.Result codes')

    # Cancel handling must not assume an attribute that may not exist.
    cancel = funcs.get('_traj_goal_was_cancelled')
    if cancel is None:
        fail('_traj_goal_was_cancelled() not found')
    elif not calls_named(cancel, 'getattr'):
        fail('_traj_goal_was_cancelled() does not use getattr(); the attribute '
             'name differs between rclpy versions and a direct access raises '
             'AttributeError inside the execute callback')


# ----------------------------------------------------------------------
# 4. Der Executor. Ohne zwei Threads steht /joint_states waehrend der Fahrt.
# ----------------------------------------------------------------------

def check_executor(tree, src_text):
    print('\n[4] executor: the execute callback must not starve /joint_states')

    main = None
    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == 'main':
            main = node
            break
    if main is None:
        fail('main() not found')
        return

    if calls_named(main, 'MultiThreadedExecutor'):
        ok('main() uses MultiThreadedExecutor')
    elif calls_named(main, 'spin'):
        fail('main() uses rclpy.spin(node) / a single-threaded executor. The '
             'execute callback blocks that one thread for the whole motion, '
             'so /joint_states is not published while the arm moves - exactly '
             'when MoveIt and RViz need it.')
    else:
        note('main() neither spins a single-threaded executor nor builds a '
             'MultiThreadedExecutor; check it by hand')

    # num_threads must be >= 2, or MultiThreadedExecutor behaves like the
    # single-threaded one for this purpose.
    for node in ast.walk(main):
        if not isinstance(node, ast.Call):
            continue
        if not (isinstance(node.func, ast.Attribute)
                and node.func.attr == 'MultiThreadedExecutor'):
            continue
        for kw in node.keywords:
            if kw.arg != 'num_threads':
                continue
            if isinstance(kw.value, ast.Constant) and isinstance(
                    kw.value.value, int) and kw.value.value < 2:
                fail(f'MultiThreadedExecutor(num_threads={kw.value.value}) is '
                     'the same as single-threaded here; use at least 2')

    if calls_named(main, 'shutdown'):
        ok('main() shuts the executor down')
    else:
        note('main() does not call executor.shutdown(); check it by hand')


# ----------------------------------------------------------------------
# 5. Die Abhaengigkeiten, ohne die der Knoten gar nicht startet
# ----------------------------------------------------------------------

def check_dependencies():
    print('\n[5] package.xml / launch file')

    xml = read(DRIVER_XML)
    for dep in ('control_msgs', 'action_msgs'):
        if re.search(rf'<depend>\s*{dep}\s*</depend>', xml):
            ok(f'package.xml declares {dep}')
        else:
            fail(f'package.xml does not declare <depend>{dep}</depend>. '
                 f'Without control_msgs the node dies on import; without '
                 f'action_msgs the Result constants are missing.')

    launch = read(DRIVER_LAUNCH)
    # load_config_defaults() raises when a YAML parameter has no description,
    # so a parameter added to the YAML but not to DESCRIPTIONS stops the
    # launch file from starting at all.
    cfg_text = read(DRIVER_CFG)
    for key in ('trajectory_action_enable', 'trajectory_action_name',
                'trajectory_feedback_hz'):
        if key not in cfg_text:
            note(f'{key} is not in the driver YAML')
            continue
        if re.search(rf"['\"]{key}['\"]\s*:", launch):
            ok(f'launch DESCRIPTIONS documents {key}')
        else:
            fail(f'{key} is in dummy_controller.yaml but not in the launch '
                 'file DESCRIPTIONS; load_config_defaults() raises '
                 '"missing descriptions" and the driver will not start')


# ----------------------------------------------------------------------
# 6. Toter Parameter. Ein Parameter, den niemand liest, ist schlimmer als
#    keiner: er sieht aus wie eine Einstellung, die etwas bewirkt.
# ----------------------------------------------------------------------

def check_feedback_param(src_text, funcs):
    print('\n[6] trajectory_feedback_hz is not a dead parameter')

    reads = re.search(r"get_parameter\(\s*'trajectory_feedback_hz'", src_text)
    if not reads:
        if 'trajectory_feedback_hz' in src_text:
            fail('trajectory_feedback_hz is mentioned but never read with '
                 'get_parameter(); it looks like a knob and is not one')
        else:
            ok('trajectory_feedback_hz is not declared at all')
        return

    if not re.search(r"get_parameter\(\s*'trajectory_feedback_hz'",
                     src_text):
        fail('trajectory_feedback_hz is not read with get_parameter()')
        return

    # Where self.trajectory_feedback_hz is *assigned* is not the point; what
    # matters is that some function reads it. Searching the raw text for
    # "publish_feedback" would have matched the comment in this very file that
    # explains why feedback is not implemented, so the text form of this test
    # passed while nothing was wired up.
    attr_read = False
    for fn in funcs.values():
        for node in ast.walk(fn):
            if (isinstance(node, ast.Attribute)
                    and node.attr == 'trajectory_feedback_hz'
                    and isinstance(node.ctx, ast.Load)):
                attr_read = True
                break
        if attr_read:
            break

    if attr_read:
        ok('trajectory_feedback_hz is read from a parameter and used')
    else:
        # Deliberate: MoveIt waits for the result, not for feedback.
        note('trajectory_feedback_hz is declared but no code reads '
             'self.trajectory_feedback_hz; it is a dead knob. Either use it or '
             'drop it.')


def main():
    print(f'reading {os.path.relpath(DRIVER_PY, REPO)}')
    src_text = read(DRIVER_PY)
    if not src_text:
        print('\nFAILED (driver source unreadable)')
        return 1

    try:
        tree = ast.parse(src_text, filename=DRIVER_PY)
    except SyntaxError as exc:
        print(f'  [FAIL] cannot parse the driver: {exc}')
        print('\nFAILED (1 problem)')
        return 1
    ok('driver parses')

    funcs = classes_and_funcs(tree)

    name = check_action_name(src_text)
    if name:
        check_server_construction(src_text, tree, funcs, name)
    check_execute_path(funcs)
    check_executor(tree, src_text)
    check_dependencies()
    check_feedback_param(src_text, funcs)

    print()
    if FAILURES:
        print(f'FAILED ({len(FAILURES)} problem(s))')
        for item in FAILURES:
            print(f'  - {item}')
        return 1
    print(f'ALL CHECKS PASSED ({len(NOTES)} note(s))')
    return 0


if __name__ == '__main__':
    sys.exit(main())
