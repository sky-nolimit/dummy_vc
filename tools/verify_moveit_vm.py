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

# ★ 这个 import 必须在模块顶层。`main()` 里的 `import rclpy` 是**局部**名字，
#   而 `_wait_future()` 是模块级函数 —— 它里面的 `rclpy` 会 NameError。这个
#   异常当时被 `except Exception` 吞掉、退化成 time.sleep，于是 executor 一次
#   都没转过、action 回包永远收不到、只能等超时；而 move_group 那边照样算出
#   了计划。症状就是"规划器看着坏了，其实脚本没在收消息"。
try:
    import rclpy
except ImportError:               # 没 source ROS 时也要能 import 本模块
    rclpy = None

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

    def note(self, name, detail=''):
        """提示，不算失败。用于"不影响判据但值得记一笔"的情况。"""
        print('  [note] {}{}'.format(name, ('  — ' + detail) if detail else ''))

    @property
    def failed(self):
        return [r for r in self.results if not r[0]]


def service_name_for(remote_node, suffix='get_parameters'):
    """把节点名拼成参数服务的全名。

    Remote node names are FULLY QUALIFIED and start with a slash:
    get_node_names() returns '/move_group', not 'move_group'. Formatting that
    straight into '{}/get_parameters' produces '//move_group/get_parameters',
    which rclpy treats as an absolute name and keeps -- so the client waits on a
    service that does not exist, wait_for_service() times out, and EVERY
    parameter looks unset. That is a false alarm: the config is fine and only
    the client is looking in the wrong place.

    Stripping the leading slash and re-adding exactly one makes both
    '/move_group' and 'move_group' produce '/move_group/get_parameters'.
    """
    return '/{}/{}'.format(remote_node.lstrip('/'), suffix)


def get_param(node, remote_node, name, timeout=5.0):
    """读远端节点的参数。读不到返回 (False, None) 而不是抛异常。

    为什么不用 get_parameter：move_group 的参数在它自己的节点上，
    从客户端读要走参数服务；而且有些发行版里 robot_description 是
    "未声明但可读"的，声明与否会影响结果。
    """
    from rcl_interfaces.srv import GetParameters
    from rclpy.parameter import Parameter

    client = node.create_client(
        GetParameters, service_name_for(remote_node))
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


def _one_line(text, limit=120):
    """把多行输出压成一行，便于塞进详情里。"""
    if text is None:
        return '<无输出>'
    text = ' '.join(str(text).split())
    return text if len(text) <= limit else text[:limit] + '…'


def parse_param_cli_output(out, returncode):
    """把 `ros2 param get` 的输出解析成 python 值。解析不了返回 (False, None)。

    输出形如（值本身含换行，所以只能按第一个 ': ' 切一次）：
        String value is: <robot ...>
        Integer value is: 42
        Parameter not set
    """
    if returncode != 0 or 'not set' in out or ': ' not in out:
        return False, None

    kind, body = out.split(': ', 1)
    kind = kind.strip()
    if body.endswith('\n'):
        body = body[:-1]

    if 'Integer' in kind:
        try:
            return True, int(body)
        except ValueError:
            return False, None
    if 'Double' in kind:
        try:
            return True, float(body)
        except ValueError:
            return False, None
    if 'Boolean' in kind:
        return True, body.strip().lower() in ('true', '1')
    # String / Array：原样返回字符串，调用方自己判断内容
    if 'String' in kind or 'Array' in kind:
        return True, body
    return False, None


def param_via_cli(node_name, name, timeout=20.0):
    """用 `ros2 param get` 读参数。返回 (ok, value, raw)。

    为什么要有这条路：直接调参数服务在某些环境上读不到（服务名/发现时机/发行版
    差异都可能），而 `ros2 param get` 走的是 CLI 自己那套，实测可靠。代价是要起
    一个子进程，所以只在前一条路失败时才用。

    为什么第三个值是 raw：解析失败时**必须能看到原文**，否则"读到了但没解析出来"
    和"根本没读到"在输出里长得一样，又要多花一个来回。
    """
    import shutil
    import subprocess

    if shutil.which('ros2') is None:
        return False, None, 'PATH 里没有 ros2'
    try:
        proc = subprocess.run(
            ['ros2', 'param', 'get', node_name, name],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout, text=True, encoding='utf-8', errors='replace')
    except Exception as exc:
        return False, None, '起子进程失败: {}'.format(exc)

    raw = proc.stdout or ''
    ok, val = parse_param_cli_output(raw, proc.returncode)
    return ok, val, raw


def get_param_any(node, remote_node, name, timeout=5.0):
    """先试参数服务，失败再用 `ros2 param get`。返回 (ok, value, via)。

    现在 main 走的是下面的 get_param_typed()（它多了类型检查）；这个函数保留
    着是因为"读参数"这件事本身仍然需要一条服务+CLI 的兜底路径，将来别处要用。
    """
    ok, val = get_param(node, remote_node, name, timeout=timeout)
    if ok:
        return True, param_to_python(val), 'service'
    ok, py, _ = param_via_cli(remote_node, name)
    if ok:
        return True, py, 'ros2 param get'
    return False, None, ''


def get_param_typed(node, remote_node, name, want, timeout=5.0):
    """读一个参数，并检查它的 python 类型是不是 `want`。

    为什么不能只判 None：实测在虚拟机上再走一次参数服务时，
    `robot_description_semantic` 被转成了 `True`（bool），于是
    `'<group name="..."' in srdf` 报 `TypeError: argument of type 'bool'
    is not iterable` —— 一个纯字符串判定的地方冒出个 bool，很难看出是读取
    环节的问题。走 `ros2 param get` 那一路类型是对的，所以类型不对就当这条
    路失败，让调用方回退到 CLI。

    返回 (ok, value, via, detail)；类型不对时 ok=False，detail 说明实际类型。
    """
    ok, val = get_param(node, remote_node, name, timeout=timeout)
    if ok:
        try:
            py = param_to_python(val)
        except Exception as exc:      # 服务返回了古怪东西，不该把脚本带走
            py, ok = None, False
            svc_detail = '参数服务读到的值转不了: {}'.format(exc)
        else:
            if isinstance(py, want):
                return True, py, 'service', ''
            # 类型不对就别要了 —— 换 CLI 再试一次，它给的类型是对的。
            # 实测这台机器上 robot_description_semantic 走服务读回来是 bool。
            svc_detail = '参数服务读到的类型是 {}，期望 {}'.format(
                type(py).__name__, want.__name__)
    else:
        svc_detail = '参数服务读不到'

    ok, py, raw = param_via_cli(remote_node, name)
    if ok and isinstance(py, want):
        return True, py, 'ros2 param get', ''
    if ok:
        return False, None, 'ros2 param get', '{}；`ros2 param get` 读到 {}，期望 {}'.format(
            svc_detail, type(py).__name__, want.__name__)
    # 解析失败时把 CLI 原文附上 —— 否则"读到但没解析出来"和"没读到"长得一样
    return False, None, '', '{}；`ros2 param get` 也没读出来: {}'.format(
        svc_detail, _one_line(raw)[:160])


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
        # 注意：rclpy 已在模块顶层 import（_wait_future 要用它），这里只补
        # 其余几个。**不要再写 `import rclpy`** —— 那只是 main() 的局部名，
        # 模块级函数看不到，正是之前 _wait_future 静默失败的原因。
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

        # 节点名和 action server 是两回事：get_node_names() 只列出"正在广播"
        # 的节点，会因为发现机制/DDS 而漏掉活着的节点（实测就漏过一次，
        # 报出"找不到 move_group"紧接着又说 action server 可用，自相矛盾）。
        # 真正的硬判据是 move_action 能不能连上。
        names = [n for n in node.get_node_names() if 'move_group' in n]

        client = ActionClient(node, MoveGroup, 'move_action')
        action_ok = client.wait_for_server(timeout_sec=args.timeout)

        if names:
            check.ok('找到 move_group 节点', ', '.join(names))
        elif action_ok:
            check.note('节点名列表里没有 move_group',
                       '但 action server 可用，说明它确实在运行 —— '
                       '这多半是 DDS 发现延迟，不影响判据')
        else:
            check.fail('找不到 move_group 节点',
                       'demo.launch.py 没起来，或者起来后崩了')

        if not action_ok:
            check.fail('move_action action server 不可用',
                       '{} s 内没等到。看终端 1 有没有报错'.format(args.timeout))
            return _finish(node, check)
        check.ok('move_action action server 可用')

        # 候选节点名。get_node_names() 给的是 '/move_group'，而参数服务在
        # /move_group/get_parameters；这里把可能的写法都试一遍，避免因为
        # 名字拼错而误报"参数没加载"。
        candidates = names or ['/move_group']
        if 'move_group' not in candidates:
            candidates.append('move_group')
        remote = candidates[0]

        # -- 2. 参数里真的有模型吗 ------------------------------------------
        print('\n' + '-' * 74)
        print('2. move_group 加载到的参数（这是"配置没生效"的判定处）')
        print('-' * 74)

        # 先确认参数本身能不能读到。读不到就说清楚是"读不到"而不是"没有"，
        # 两者指向的问题完全不同：前者查脚本/节点名，后者才查配置。
        desc, desc_via, desc_detail = None, '', ''
        for cand in candidates:
            ok_probe, py, via, detail = get_param_typed(
                node, cand, 'robot_description', str)
            if ok_probe:
                desc, desc_via, remote = py, via, cand
                break
            desc_detail = detail
        if desc is None:
            print('  试过的节点名: {}'.format(', '.join(candidates)))
            print('  [!!] 读不到 robot_description：{}'.format(desc_detail))
            print('       先用 ROS 自带工具确认参数到底在不在:')
            print('           ros2 param get {} robot_description | head -c 200'
                  .format(remote))
            print('       （能打印出 <robot ... 就说明是脚本的锅，不是配置）')
            print()

        if desc is not None:
            if desc_via != 'service':
                print('  (参数服务读不到，改用 `{}` 读到的)'.format(desc_via))
            if len(desc) > 1000:
                check.ok('robot_description 非空', '{} 字符'.format(len(desc)))
            else:
                check.fail('robot_description 太短',
                           '只有 {} 字符，URDF 基本没加载'.format(len(desc)))
            if args.verbose:
                print('        开头 200 字符: {}'.format(
                    desc[:200].replace('\n', ' ')))
        else:
            check.fail('读不到 robot_description 参数',
                       '不是"配置没加载"，而是两种读法都读不到 —— 先跑 '
                       '`ros2 param get {} robot_description` 对比'.format(remote))

        srdf, srdf_via, srdf_detail = None, '', ''
        for cand in candidates:
            ok_s, py, via, detail = get_param_typed(
                node, cand, 'robot_description_semantic', str)
            if ok_s:
                srdf, srdf_via = py, via
                break
            srdf_detail = detail       # 记住最后一次失败的原因
        if srdf is not None:
            if srdf_via != 'service':
                print('  (SRDF 也是改用 `{}` 读到的)'.format(srdf_via))
            missing = [g for g in groups if '<group name="{}"'.format(g) not in srdf]
            if missing:
                check.fail('SRDF 里缺少规划组: {}'.format(missing),
                           'SRDF 加载了但组名不对 —— 检查 config/dummy2.srdf')
            else:
                check.ok('robot_description_semantic 含目标规划组',
                         '{} 字符'.format(len(srdf)))
        else:
            check.fail('读不到 robot_description_semantic 参数',
                       srdf_detail or '两种读法都读不到，先对照 '
                       '`ros2 param get` 的输出')

        # kinematics 在 demo 里是 rviz2 的节点参数，**不在 move_group 节点上**
        # （实测 `ros2 param get /move_group robot_description_kinematics` 报
        # `Parameter not set`）。所以读不到不能判红 —— move_group 用不用它，
        # 由后面的真实规划说了算。能读到就顺手核一下组名。
        kin = None
        for cand in candidates:
            ok_k, py, _, _ = get_param_typed(
                node, cand, 'robot_description_kinematics', dict)
            if ok_k:
                kin = py
                break
        if kin is None:
            ok_cli, py, raw = param_via_cli(remote, 'robot_description_kinematics')
            if ok_cli and isinstance(py, dict) and 'dummy_arm' in py:
                kin = py
                print('  (kinematics 只有 CLI 读得到 —— move_group 节点上没这个'
                      '参数，它给 rviz2 用)')
        if kin is not None:
            solver_groups = sorted(kin.keys()) if hasattr(kin, 'keys') else []
            # 只有"链"组能有求解器：gripper 是并联手指的关节列表，没有 IK，
            # 所以这里期望的是 ['dummy_arm'] 而不是两个组都在。
            if solver_groups:
                check.ok('robot_description_kinematics 有求解器',
                         '{}（链组才有 IK；gripper 是关节组，'
                         '本来就不该有）'.format(', '.join(solver_groups)))
            else:
                check.fail('robot_description_kinematics 为空',
                           '没有 IK 求解器 —— 检查 config/kinematics.yaml')
            if 'dummy_arm' not in solver_groups:
                check.fail('dummy_arm 没有 IK 求解器',
                           '链组 dummy_arm 必须有条目，否则规划只能靠'
                           '关节空间目标，笛卡尔目标全部失败')
        else:
            # 不判红：move_group 上本来就没有这个参数。真正能证明运动学可用的
            # 只有"规划算得出来"，那是下一节的事。
            check.note('move_group 节点上没有 robot_description_kinematics 参数',
                       '这在本配置里是正常的（它只喂给 rviz2）；'
                       '运动学到底能不能用，看第 3 节的规划结果')

        # -- 2b. 起始状态 ----------------------------------------------
        # move_group 日志里的 "Found empty JointState message" 表示没有起点。
        # 那会让规划失败，而且看起来像配置坏了，所以提前查一次。
        js_ok, js_count, js_what = _joint_state_summary()
        if js_ok:
            check.ok('/joint_states 有起始状态', '{} 个关节'.format(js_count))
        else:
            check.fail('/joint_states 读不到起始状态', js_what)
            print('       demo 的起始状态由 joint_state_publisher_gui 提供 ——')
            print('       那个窗口必须在、并且要发布关节值；报错里会看到')
            print('       "Found empty JointState message"。')

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
            # ★ 这里必须转 executor，否则回包永远收不到（见 _wait_future 的注释）
            sent_ok, spin_err = _wait_future(node, send, args.timeout)
            if not sent_ok:
                check.fail('组 "{}" 发送规划请求超时'.format(group),
                           spin_err or '{} s 内没等到 move_group 的响应'.format(
                               args.timeout))
                continue
            gh = send.result()
            if gh is None or not gh.accepted:
                check.fail('组 "{}" 的规划请求被拒绝'.format(group),
                           '通常是组名不存在，或者没有运动学求解器')
                continue

            res_fut = gh.get_result_async()
            res_ok, spin_err2 = _wait_future(node, res_fut, args.timeout)
            if not res_ok:
                check.fail('组 "{}" 规划结果超时'.format(group),
                           spin_err2 or 'move_group 接下了请求但没回结果，'
                           '看它的终端输出')
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


def _wait_future(node, fut, timeout, poll=0.05):
    """等一个 rclpy future 完成。

    ★ 必须一边转 executor 一边等。之前这里只写 `while not fut.done():
    time.sleep(...)`，结果 move_group 明明已经把计划算出来了（它日志里写着
    "Motion plan was computed successfully"），脚本却报"发送规划请求超时" ——
    action 的回包要靠 executor 收，节点不 spin 就永远收不到，future 永远是
    pending。`wait_for_server()` 能返回 True 是因为它内部自己用了 waitable，
    只有 action 的回包需要 executor。

    返回 (完成了吗, 第一次异常信息)。第二个值非空表示"没能转 executor" ——
    这种情况必须报出来，否则又是一次"看着像规划器坏了"。
    """
    t0 = time.time()
    first_error = ''
    while not fut.done() and time.time() - t0 < timeout:
        if rclpy is None:
            first_error = first_error or 'rclpy 没导入（没 source ROS？）'
            time.sleep(poll)
            continue
        try:
            rclpy.spin_once(node, timeout_sec=poll)
        except Exception as exc:
            first_error = first_error or '{}: {}'.format(type(exc).__name__, exc)
            time.sleep(poll)
    return fut.done(), first_error


def _joint_state_summary(timeout=5.0):
    """读一次 /joint_states，返回 (ok, 关节数, 名字) 或 (False, 0, 原因)。

    demo 里起始状态由 joint_state_publisher_gui 提供，没有它 move_group 会报
    "Found empty JointState message"，规划就没有起点（那行 ERROR 在用户日志里
    出现过四次）。这里提前查一次，把"规划失败"和"没有起点"区分开。
    """
    import shutil
    import subprocess
    if shutil.which('ros2') is None:
        return False, 0, 'PATH 里没有 ros2'
    try:
        proc = subprocess.run(
            ['ros2', 'topic', 'echo', '/joint_states', '--once',
             '--field', 'name'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            timeout=timeout, text=True, encoding='utf-8', errors='replace')
    except Exception as exc:
        return False, 0, '读 /joint_states 失败: {}'.format(exc)
    raw = proc.stdout or ''
    if proc.returncode != 0 or 'does not exist' in raw.lower():
        return False, 0, '/joint_states 话题不存在'
    # --field name 打印的是 yaml 列表：['a', 'b', ...]
    import re
    names = re.findall(r"[A-Za-z0-9_]*(?:joint|finger)[A-Za-z0-9_]*", raw)
    names = [n for n in names if n and n not in ('joint', 'finger')]
    if not names:
        return False, 0, '/joint_states 是空的: {}'.format(_one_line(raw)[:80])
    return True, len(names), _one_line(raw)[:120]


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
