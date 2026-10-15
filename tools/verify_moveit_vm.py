#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""在虚拟机（或任何装了 ROS 2 + MoveIt 2 的机器）上验证 dummy_vc_moveit_config。

为什么需要这个脚本
------------------
`ros2 launch ... demo.launch.py` 跑起来、RViz 里能拖动机械臂、点 Plan 能看到一条
轨迹——这些都**不能证明配置是对的**。反过来，Plan 偶尔失败也不能证明配置是错的，
外行很难分辨。人眼判断不了的东西，让 move_group 自己回答。

这个脚本只做一件事：**真的向 move_group 发一个规划请求，把结果和原因打出来。**

判据（脚本会自动判定，不用你判断）
----------------------------------
  1. move_group 起来了，而且它的参数里真的有机器人模型
  2. `dummy_arm` 这条规划链能从当前位置规划到一个明确可达的位姿
  3. `gripper` 组也能规划（手指是 prismatic，不能和 arm 混在一起测）
  4. 规划出来的轨迹里每个关节都有限位、且都在限位内

任何一条不成立，脚本以非 0 退出，并说明是哪一条、可能的原因是什么。

不会动硬件
----------
只调用 planning 接口，不调用 execute。demo.launch.py 本来就没有 controller，
这里的 execute 也只会发给空实现。**跑之前请确认机器没上电也不要紧。**

用法
----
    # 终端 1
    ros2 launch dummy_vc_moveit_config demo.launch.py

    # 终端 2（等 demo 打印完 banner）
    python3 tools/verify_moveit_vm.py

可选参数：
    --group      只测某个组（默认两个都测）
    --timeout    等待 move_group 的秒数（默认 60）
    --verbose    打印 move_group 的参数内容，排查"配置没加载"时用
"""

import argparse
import sys
import time

# ---------------------------------------------------------------------------
# 期望值。这些数字来自 tools/check_moveit_config.py 核对过的实测关节行程，
# 单位和 URDF 一样是**度**（MoveIt 打印时也是度）。
# 如果这里和机器实际不符，说明 URDF/配置被动过，不是"配置加载失败"。
# ---------------------------------------------------------------------------
EXPECTED_LIMITS_DEG = {
    'dummy2_joint1': (-165.0, 165.0),
    'dummy2_joint2': (-90.0, 75.0),
    'dummy2_joint3': (-90.0, 45.0),
    'dummy2_joint4': (-180.0, 165.0),
    'dummy2_joint5': (-90.0, 90.0),
    'dummy2_joint6': (-180.0, 180.0),
}

# 规划目标。选的是**远离限位边界、且离 home 有可观距离**的位姿：
# 太靠近边界会让规划器偶尔失败，太靠近 home 会让"规划成功"没有说服力
# （可能只是原地不动）。
#
# ★ 这里的单位是 **URDF 角度**，不是真机的 move_j。
# 两者差一个绕圈：move_j j3 = 90（home）对应 URDF j3 = -90，而 URDF 的 j3
# 上限只有 45。曾经在这里填了 move_j 口径的 90，那是个越限的位姿。
# 每个值都过了 tools/fk_dummy.py 的限位判定才写进来。
ARM_GOAL_DEG = {
    'dummy2_joint1': 0.0,
    'dummy2_joint2': -60.0,      # URDF 限位 -90..75，余量 30°
    'dummy2_joint3': 0.0,        # URDF 限位 -90..45，余量 45°
    'dummy2_joint4': 0.0,
    'dummy2_joint5': 0.0,
    'dummy2_joint6': 0.0,
}

# 手指行程 0~0.023 m（米，prismatic）。取中间值，不碰两端。
FINGER_GOAL_M = 0.0115

# 规划超时。OMPL 在空场景里通常几十毫秒，5 s 已经很宽松；
# 真超时说明是配置/运动学问题，不是算得慢。
PLAN_TIMEOUT_SEC = 5.0


def _selfcheck_goals():
    """先验自己：规划目标本身必须落在实测限位内。

    这一步不是多余的 —— 目标越限时 MoveIt 会返回 -14 (GOAL_CONSTRAINTS_
    VIOLATED)，看起来像"配置坏了"，实际是这个脚本填错了数。填目标时用的是
    真机 move_j 口径还是 URDF 口径，差一个绕圈，很容易搞混。所以让脚本自己
    先判一次，而不是等到虚拟机上报一个含糊的失败。
    """
    problems = []

    for jname, target in ARM_GOAL_DEG.items():
        if jname not in EXPECTED_LIMITS_DEG:
            problems.append('{} 不是 dummy_arm 的关节'.format(jname))
            continue
        lo, hi = EXPECTED_LIMITS_DEG[jname]
        if not (lo <= target <= hi):
            problems.append('{}={}° 越出限位 {:.1f}..{:.1f}°'.format(
                jname, target, lo, hi))

    # 手指是 prismatic，限位在 URDF 里写的是 0~0.023 m
    if not (0.0 <= FINGER_GOAL_M <= 0.023):
        problems.append('手指目标 {} m 越出 0..0.023 m'.format(FINGER_GOAL_M))

    # 目标不能等于 home（0,0,0,0,0,0），否则"规划成功"可能只是原地不动
    if all(abs(v) < 1e-9 for v in ARM_GOAL_DEG.values()):
        problems.append('机械臂目标就是 home，规划成功无法说明问题')

    return problems


def _deg(v):
    """弧度转度。MoveIt 的轨迹点用弧度，人看限位用度。"""
    import math
    return math.degrees(v)


class Check:
    """记录一条判据的结果。任何一条 failed 都会让脚本非 0 退出。"""

    def __init__(self):
        self.results = []

    def ok(self, name, detail=''):
        self.results.append((True, name, detail))
        print('  [ok  ] {}{}'.format(name, ('  — ' + detail) if detail else ''))

    def fail(self, name, detail=''):
        self.results.append((False, name, detail))
        print('  [FAIL] {}{}'.format(name, ('  — ' + detail) if detail else ''))

    def skip(self, name, detail=''):
        print('  [skip] {}{}'.format(name, ('  — ' + detail) if detail else ''))

    @property
    def failed(self):
        return [r for r in self.results if not r[0]]


def get_param(node, remote_node, name, timeout=5.0):
    """读远端节点的参数。读不到返回 (False, None) 而不是抛异常。

    为什么不用 get_parameter：move_group 的参数在它自己的节点上，
    从客户端读要走参数服务；而且有些发行版里 robot_description 是
    "未声明但可读"的，声明与否会影响结果。所以三条路都试。
    """
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter

    client = node.create_client(
        GetParameters, '{}/get_parameters'.format(remote_node))
    if not client.wait_for_service(timeout_sec=timeout):
        return False, None

    req = GetParameters.Request()
    req.names = [name]
    fut = client.call_async(req)

    deadline = time.time() + timeout
    while not fut.done() and time.time() < deadline:
        time.sleep(0.05)
    if not fut.done():
        return False, None

    resp = fut.result()
    if resp is None or not resp.values:
        return False, None
    val = resp.values[0]
    if val.type == Parameter.Type.NOT_SET.value:
        return False, None
    return True, val


def param_to_python(val):
    """把 rcl_interfaces ParameterValue 转成 python 值。"""
    from rclpy.parameter import Parameter
    t = val.type
    if t == Parameter.Type.BOOL.value:
        return bool(val.bool_value)
    if t == Parameter.Type.INTEGER.value:
        return int(val.integer_value)
    if t == Parameter.Type.DOUBLE.value:
        return float(val.double_value)
    if t == Parameter.Type.STRING.value:
        return val.string_value
    if t == Parameter.Type.BYTE_ARRAY.value:
        return bytes(val.byte_array_value)
    if t == Parameter.Type.BOOL_ARRAY.value:
        return list(val.bool_array_value)
    if t == Parameter.Type.INTEGER_ARRAY.value:
        return list(val.integer_array_value)
    if t == Parameter.Type.DOUBLE_ARRAY.value:
        return list(val.double_array_value)
    if t == Parameter.Type.STRING_ARRAY.value:
        return list(val.string_array_value)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--group', action='append', default=None,
                    help='只测这个规划组，可重复（默认 dummy_arm 和 gripper 都测）')
    ap.add_argument('--timeout', type=float, default=60.0,
                    help='等待 move_group 的秒数（默认 60）')
    ap.add_argument('--verbose', action='store_true',
                    help='打印 move_group 参数内容')
    args = ap.parse_args()

    groups = args.group or ['dummy_arm', 'gripper']
    check = Check()

    print('=' * 74)
    print('MoveIt 配置自检 —— 直接问 move_group 能不能规划')
    print('=' * 74)

    # -- 0. 先验自己：目标位姿不能越限 -------------------------------------
    goal_problems = _selfcheck_goals()
    if goal_problems:
        print('\n脚本自身的规划目标是错的，先修脚本，不用看 ROS：')
        for p in goal_problems:
            print('  - {}'.format(p))
        print('\n（单位口径：URDF 角度。真机 move_j 的 j3=90 对应 URDF j3=-90。）')
        return 3
    print('\n脚本自身的规划目标已按实测限位核过。')

    # -- 0. 导入依赖 --------------------------------------------------------
    try:
        import rclpy
        from rclpy.node import Node
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest
        from rclpy.action import ActionClient
    except ImportError as e:
        print('\n缺少 ROS 2 / MoveIt 的 Python 包: {}'.format(e))
        print('说明这个终端没有 source ROS 环境。先执行：')
        print('    source /opt/ros/<发行版>/setup.bash')
        print('    source ~/dummy_vc_ws/install/setup.bash')
        return 2

    rclpy.init()
    node = Node('verify_moveit_vm')
    try:
        # -- 1. move_group 在不在 -------------------------------------------
        print('\n' + '-' * 74)
        print('1. move_group 是否在运行')
        print('-' * 74)

        names = [n for n in node.get_node_names() if 'move_group' in n]
        if names:
            check.ok('找到 move_group 节点', ', '.join(names))
        else:
            check.fail('找不到 move_group 节点',
                       'demo.launch.py 没起来，或者起来后崩了')

        client = ActionClient(node, MoveGroup, 'move_action')
        if not client.wait_for_server(timeout_sec=args.timeout):
            check.fail('move_action action server 不可用',
                       '{} s 内没等到。看终端 1 有没有报错'.format(args.timeout))
            return _finish(node, check)
        check.ok('move_action action server 可用')

        remote = names[0] if names else 'move_group'

        # -- 2. 参数里真的有模型吗 ------------------------------------------
        print('\n' + '-' * 74)
        print('2. move_group 加载到的参数（这是"配置没生效"的判定处）')
        print('-' * 74)

        ok, val = get_param(node, remote, 'robot_description')
        if ok:
            desc = param_to_python(val) or ''
            if len(desc) > 1000:
                check.ok('robot_description 非空', '{} 字符'.format(len(desc)))
            else:
                check.fail('robot_description 太短',
                           '只有 {} 字符，URDF 基本没加载'.format(len(desc)))
            if args.verbose:
                print('        开头 200 字符: {}'.format(desc[:200].replace('\n', ' ')))
        else:
            check.fail('读不到 robot_description 参数',
                       'move_group 参数没加载 —— 检查 .setup_assistant 指向的 URDF 路径')

        ok, val = get_param(node, remote, 'robot_description_semantic')
        if ok:
            srdf = param_to_python(val) or ''
            missing = [g for g in groups if '<group name="{}"'.format(g) not in srdf]
            if srdf and not missing:
                check.ok('robot_description_semantic 含目标规划组',
                         '{} 字符'.format(len(srdf)))
            else:
                check.fail('SRDF 里缺少规划组: {}'.format(missing),
                           'SRDF 加载了但组名不对 —— 检查 config/dummy2.srdf')
        else:
            check.fail('读不到 robot_description_semantic 参数',
                       'SRDF 没加载')

        ok, val = get_param(node, remote, 'robot_description_kinematics')
        if ok:
            kin = param_to_python(val)
            solver_groups = sorted(kin.keys()) if hasattr(kin, 'keys') else []
            if solver_groups:
                check.ok('robot_description_kinematics 有求解器',
                         ', '.join(solver_groups))
            else:
                check.fail('robot_description_kinematics 为空',
                           '没有 IK 求解器 —— 检查 config/kinematics.yaml')
        else:
            check.fail('读不到 robot_description_kinematics 参数',
                       'kinematics.yaml 没加载')

        # -- 3/4. 真的规划一次 ----------------------------------------------
        for group in groups:
            print('\n' + '-' * 74)
            print('3. 规划测试：组 "{}"'.format(group))
            print('-' * 74)

            if group == 'dummy_arm':
                goal = ARM_GOAL_DEG
            else:
                goal = {'dummy2_finger_joint1': FINGER_GOAL_M,
                        'dummy2_finger_joint2': FINGER_GOAL_M}

            req = MotionPlanRequest()
            req.group_name = group
            req.num_planning_attempts = 5
            req.allowed_planning_time = PLAN_TIMEOUT_SEC
            req.max_velocity_scaling_factor = 0.2
            req.max_acceleration_scaling_factor = 0.2

            c = Constraints()
            for jname, target in goal.items():
                jc = JointConstraint()
                jc.joint_name = jname
                jc.position = float(target)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                c.joint_constraints.append(jc)
            req.goal_constraints.append(c)

            g = MoveGroup.Goal()
            g.request = req
            g.planning_options.plan_only = True          # 关键：绝不下发
            g.planning_options.look_around = False
            g.planning_options.replan = False

            print('  目标: {}'.format(
                ', '.join('{}={}'.format(k, v) for k, v in goal.items())))

            send = client.send_goal_async(g)
            t0 = time.time()
            while not send.done() and time.time() - t0 < args.timeout:
                time.sleep(0.05)
            if not send.done():
                check.fail('组 "{}" 发送规划请求超时'.format(group))
                continue
            gh = send.result()
            if gh is None or not gh.accepted:
                check.fail('组 "{}" 的规划请求被拒绝'.format(group),
                           '通常是组名不存在，或者没有运动学求解器')
                continue

            res_fut = gh.get_result_async()
            t0 = time.time()
            while not res_fut.done() and time.time() - t0 < args.timeout:
                time.sleep(0.05)
            if not res_fut.done():
                check.fail('组 "{}" 规划结果超时'.format(group))
                continue

            wrapped = res_fut.result()
            result = wrapped.result
            ec = result.error_code.val

            if ec != 1:                                   # 1 == SUCCESS
                check.fail('组 "{}" 规划失败'.format(group),
                           'error_code={}  {}'.format(ec, _explain_error(ec)))
                continue

            traj = result.planned_trajectory.joint_trajectory
            if not traj.points:
                check.fail('组 "{}" 返回了成功但轨迹是空的'.format(group),
                           '这说明 planning pipeline 配置有问题')
                continue

            check.ok('组 "{}" 规划成功'.format(group),
                     '{} 个关节, {} 个轨迹点'.format(
                         len(traj.joint_names), len(traj.points)))

            # -- 4. 轨迹里的关节名和限位 ------------------------------------
            if group == 'dummy_arm':
                _check_trajectory_joints(traj, check)

        # -- 5. 手指和机械臂的限位不能混在一起 ------------------------------
        print('\n' + '-' * 74)
        print('5. 结论')
        print('-' * 74)
        return _finish(node, check)

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def _check_trajectory_joints(traj, check):
    """轨迹里的关节名必须正好是那 6 个，而且轨迹点都在限位内。"""
    names = list(traj.joint_names)
    unexpected = [n for n in names if n not in EXPECTED_LIMITS_DEG]
    missing = [n for n in EXPECTED_LIMITS_DEG if n not in names]

    if unexpected or missing:
        check.fail('轨迹关节名不是那 6 个',
                   '多出来: {}  缺少: {}'.format(unexpected, missing))
        return
    check.ok('轨迹关节名正好是 dummy2_joint1..6')

    worst = []
    for jname in names:
        lo, hi = EXPECTED_LIMITS_DEG[jname]
        idx = names.index(jname)
        vals = [_deg(p.positions[idx]) for p in traj.points]
        w = max(abs(min(vals) - lo), abs(max(vals) - hi))
        worst.append((w, jname, min(vals), max(vals), lo, hi))

    out_of_range = [w for w in worst if w[2] < w[4] - 0.01 or w[3] > w[5] + 0.01]
    if out_of_range:
        for _, jname, vmin, vmax, lo, hi in out_of_range:
            check.fail('{} 的轨迹越界'.format(jname),
                       '轨迹 {:.2f}..{:.2f}°，限位 {:.2f}..{:.2f}°'.format(
                           vmin, vmax, lo, hi))
    else:
        check.ok('轨迹所有点都在实测限位内',
                 '离限位最近的关节: {} 余量 {:.1f}°'.format(
                     min(worst)[1], min(worst)[0]))


def _explain_error(ec):
    """把 MoveIt 的 error_code 翻译成人能看懂的话。

    数值来自 moveit_msgs/msg/MoveItErrorCodes.msg。
    """
    table = {
        -1: 'FAILURE（笼统失败）',
        -2: 'PLANNING_FAILED —— 规划器算不出路径，最常见：目标不可达 / 有碰撞 / 关节限位不对',
        -3: 'INVALID_MOTION_PLAN',
        -4: 'MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE',
        -5: 'CONTROL_FAILED',
        -6: 'UNABLE_TO_AQUIRE_SENSOR_DATA',
        -7: 'TIMED_OUT —— 规划超时，多半是目标不可达',
        -10: 'START_STATE_IN_COLLISION —— 起点就和物体碰撞，检查场景里有没有多余物体',
        -11: 'START_STATE_VIOLATES_PATH_CONSTRAINTS',
        -12: 'GOAL_IN_COLLISION',
        -13: 'GOAL_VIOLATES_PATH_CONSTRAINTS',
        -14: 'GOAL_CONSTRAINTS_VIOLATED —— 目标本身违反限位/约束',
        -15: 'INVALID_GROUP_NAME —— 组名不对',
        -16: 'INVALID_GOAL_CONSTRAINTS',
        -17: 'INVALID_ROBOT_STATE',
        -18: 'INVALID_LINK_NAME',
        -19: 'INVALID_OBJECT_NAME',
        -21: 'FRAME_TRANSFORM_FAILURE',
        -22: 'COLLISION_CHECKING_UNAVAILABLE',
        -23: 'ROBOT_STATE_STALE',
        -24: 'SENSOR_INFO_STALE',
        -25: 'COMMUNICATION_FAILURE',
        -26: 'CRASH',
        -27: 'ABORT',
        -31: 'INVALID_LINK_NAME',
        -32: 'NO_IK_SOLUTION —— 有运动学配置但解不出逆解',
    }
    return table.get(ec, '未收录的 error_code')


def _finish(node, check):
    failed = check.failed
    print()
    if failed:
        print('RESULT: {} 条判据没通过'.format(len(failed)))
        for _, name, detail in failed:
            print('  - {}{}'.format(name, ('  — ' + detail) if detail else ''))
        print()
        print('把上面整段输出贴回来即可（不需要截图）。')
        return 1
    print('RESULT: MoveIt 配置在这台机器上可用 —— 规划链、运动学、限位都对')
    print()
    print('下一步可以测真机接管（需要机械臂上电）：')
    print('    ros2 launch dummy_controller dummy_controller.launch.py')
    print('    另开终端: ros2 launch dummy_vc_moveit_config moveit_hardware.launch.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
