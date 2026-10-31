#!/usr/bin/env python3
"""离线自查阶段 5：抓取规划 + 任务状态机。

它要防的故障是这一句：

    真机上某一步失败了，但代码不知道该怎么办 —— 卡住、静默重来、或者把
    "夹爪没标定"当成普通失败继续往下走

真机上试一次失败要重开相机、重摆方块，一天试不了几次；而"哪一步失败之后该去
哪"是纯逻辑，可以在这里把每条边都走一遍。所以这个自测的重点不是几何（那在
`tools/selftest_vision_geometry.py` 里），而是：

  * 每个状态都有动作、除终态外都有前进方向（没有死胡同）
  * 可重试的步骤重试到上限才认输；夹爪那两步**不重试**
  * 夹爪映射没实测时，前面几步照走，到 CLOSE_GRIPPER 必须停
  * ABORT 从任何状态都能进 FAULT，START 能把 FAULT/DONE 拉回流程

    python tools/selftest_grasp_task.py
"""
import ast
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PERCEPTION = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_perception')
GRASP = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_grasp')

#: 纯层允许 import 的东西。多一个 ROS/cv2 进来，离线自测就没了。
PURE_IMPORTS = {'math', 'enum', 'dataclasses', 'typing', 'numpy',
                'dummy_vc_perception', 'dummy_vc_grasp'}

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


def close(label, got, want, tol=1e-9):
    got_arr = np.asarray(got, float)
    want_arr = np.asarray(want, float)
    if got_arr.shape != want_arr.shape:
        fail(f'{label}: 形状 {got_arr.shape} != {want_arr.shape}')
        return False
    err = float(np.max(np.abs(got_arr - want_arr)))
    if err <= tol:
        ok(f'{label}（最大误差 {err:.3e} <= {tol:.1e}）')
        return True
    fail(f'{label}: 最大误差 {err:.3e} > {tol:.1e}；got={got_arr} want={want_arr}')
    return False


def expect_error(label, exc_types, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except exc_types as exc:
        ok(f'{label}（{type(exc).__name__}）')
        return True
    except Exception as exc:            # noqa: BLE001 - 报出来比咽下去强
        fail(f'{label}: 抛了意料外的 {type(exc).__name__}: {exc}')
        return False
    fail(f'{label}: 没有拒绝，返回了一个看起来正常的值')
    return False


def load_modules():
    for path in (PERCEPTION, GRASP):
        if path not in sys.path:
            sys.path.insert(0, path)
    try:
        from dummy_vc_grasp import planner, scene_objects, task
        from dummy_vc_perception import geometry, gripper
    except ImportError as exc:
        fail(f'import 失败：{exc}（这两层只依赖 numpy）')
        return None, None, None, None, None
    return geometry, gripper, planner, scene_objects, task


def check_offline_imports():
    print('\n0. 纯层不许 import ROS / OpenCV')
    for name in ('planner.py', 'scene_objects.py', 'task.py'):
        path = os.path.join(GRASP, 'dummy_vc_grasp', name)
        try:
            with open(path, encoding='utf-8') as fh:
                tree = ast.parse(fh.read())
        except OSError as exc:
            fail(f'读不到 {name}：{exc}')
            continue
        bad = []
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                mods = [a.name.split('.')[0] for a in node.names]
            elif isinstance(node, ast.ImportFrom):
                mods = [(node.module or '').split('.')[0]]
            else:
                continue
            bad += [m for m in mods if m and m not in PURE_IMPORTS]
        if bad:
            fail(f'{name} import 了不该 import 的：{sorted(set(bad))}')
        else:
            ok(f'{name} 只 import 了 {sorted(PURE_IMPORTS & set()) or "允许的模块"}')


class FakeDetection:
    """`dummy_vc_perception.detection.SceneDetection` 的替身。

    规划层只读这四个字段，所以这里不必构造真的一帧图像。
    """

    def __init__(self, position, yaw_rad=0.0, extent=(0.024, 0.024),
                 distance_m=0.3):
        self.position = np.asarray(position, float)
        self.yaw_rad = float(yaw_rad)
        self.extent_m = (float(extent[0]), float(extent[1]))
        self.edge_error_m = 0.0
        self.distance_m = float(distance_m)
        self.metres_per_pixel = (0.001, 0.001)


def measured_model(gripper):
    """四个端点都标成 measured 的夹爪模型（0 mm / 46 mm <-> 30 deg / 0 deg）。"""
    return gripper.opening_endpoints_from_measurements(
        closed_width_m=0.0, closed_angle_deg=30.0,
        open_width_m=0.046, open_angle_deg=0.0,
        where_closed='自测', where_open='自测')


def check_planner(gripper, planner):
    print('\n1. 规划：能抓的给出路点，不能抓的说清理由')
    model = measured_model(gripper)
    ground = -0.0405
    good = FakeDetection([0.0, 0.30, ground + 0.012], yaw_rad=0.0)
    plan = planner.plan_grasp(good, model)
    wp = plan.waypoints
    ok(f'正常目标：{plan.describe()}')
    # 方块中心 z = 地面 + 12 mm；夹持中心要再抬高 3 + 17.5 - 12 = 8.5 mm，
    # 否则 17.5 mm 长的指头会插进桌面。
    close('夹持中心比方块中心高 8.5 mm', wp.grasp,
          [0.0, 0.30, ground + 0.012 + 0.0085], 1e-9)
    close('plan 记下了这个抬高量', plan.tcp_offset_m, 0.0085, 1e-12)
    close('起手位在抓取点上方 80 mm', wp.pre_grasp - wp.grasp, [0, 0, 0.08], 1e-12)
    close('抬升位在抓取点上方 100 mm', wp.lift - wp.grasp, [0, 0, 0.10], 1e-12)
    close('开口 = 边长 + 2 mm', plan.open_width_m, 0.024 + 0.002, 1e-12)
    # 2026-10-22 真机日志里的拟合尺寸：长边是方块投到地面的"影子"，被低机位
    # 拉长到方块的 3.6 倍，只有短边挨着真实尺寸。vision 现在发的 bbox.size 是
    # 标称方框（不是这个拟合尺寸），这条断言守的是"哪天把拟合尺寸接进来"。
    shadow = FakeDetection([0.0, 0.302, ground + 0.012],
                           extent=(0.0862, 0.0295))
    shadow_plan = planner.plan_grasp(shadow, model)
    close('短边当边长：开口 = 短边 + 2 mm', shadow_plan.open_width_m,
          0.0295 + 0.002, 1e-12)
    ok(f'长边 86.2 mm 不再顶爆开口：{shadow_plan.describe()}')
    if plan.gripper_ready:
        ok('已实测的模型被认成 ready')
    else:
        fail('已实测的模型没被认成 ready')

    # -- 拒的理由 ---------------------------------------------------------
    far = FakeDetection([0.0, 0.60, ground + 0.012])
    expect_error('太远', planner.NotGraspable, planner.plan_grasp, far, model)
    near = FakeDetection([0.0, 0.05, ground + 0.012])
    expect_error('太近', planner.NotGraspable, planner.plan_grasp, near, model)
    high = FakeDetection([0.0, 0.30, 0.35])
    expect_error('太高（顶抓不成立）', planner.NotGraspable,
                 planner.plan_grasp, high, model)
    below = FakeDetection([0.0, 0.30, -0.20])
    expect_error('低于工作面', planner.NotGraspable, planner.plan_grasp, below, model)
    bad_extent = FakeDetection([0.0, 0.30, ground + 0.012], extent=(0.0, 0.024))
    expect_error('边长不合理', planner.NotGraspable,
                 planner.plan_grasp, bad_extent, model)

    # -- 多个候选：跳过不能抓的，理由一条条留着 ---------------------------
    cands = [far, high, good]
    picked, reasons = planner.first_graspable(cands, model)
    if picked is None:
        fail('三个候选里有一个能抓，却一个都没选出来')
    else:
        ok(f'跳过 {len(reasons)} 个候选后选中第 3 个')
        close('选中的是同一个抓取点', picked.waypoints.grasp,
              [0.0, 0.30, ground + 0.012 + 0.0085], 1e-9)
    if len(reasons) != 2:
        fail(f'拒绝理由应该有 2 条，实际 {len(reasons)} 条')
    none, reasons = planner.first_graspable([far, high], model)
    if none is None and len(reasons) == 2:
        ok('全都不能抓时返回 None 并带两条理由')
    else:
        fail(f'全不能抓的情形不对：plan={none}，理由 {len(reasons)} 条')

    # -- 夹爪没实测：规划照做，但计划上标着不许合爪 -----------------------
    placeholder = gripper.unverified_placeholder_model()
    plan2 = planner.plan_grasp(good, placeholder)
    if plan2.gripper_ready:
        fail('未实测的夹爪模型被标成了 ready')
    else:
        ok('未实测的夹爪模型：规划成功但 gripper_ready=False')
    expect_error('未实测模型拒绝给出角度指令值', gripper.UnverifiedGripperModel,
                 placeholder.command_for_width, plan2.open_width_m)

    # -- 2026-10-27 真机反馈的三件事 --------------------------------------
    # (a) 腕部不要为了迁就视觉给的任意 yaw 多转半圈：正方形方块 90 度就够。
    yaw = np.deg2rad(55.3)
    snapped = planner.snap_yaw(yaw)
    close('55.3° 量化到 90°', snapped, np.pi / 2, 1e-12)
    close('4.3° 量化到 0°', planner.snap_yaw(np.deg2rad(4.3)), 0.0, 1e-12)
    close('−40° 量化到 0°', planner.snap_yaw(np.deg2rad(-40.0)), 0.0, 1e-12)
    close('snap=0 时原样返回', planner.snap_yaw(yaw, 0.0), yaw, 1e-12)
    tilted = FakeDetection([0.0, 0.30, ground + 0.012], yaw_rad=yaw)
    keep = planner.plan_grasp(tilted, model, yaw_snap_deg=0.0)
    close('snap=0：计划里还是原始 yaw', keep.yaw_rad, yaw, 1e-12)
    snap_plan = planner.plan_grasp(tilted, model)
    close('默认 90°：计划里的 yaw 被量化', snap_plan.yaw_rad, np.pi / 2, 1e-12)
    q_keep = keep.waypoints.quaternion
    q_snap = snap_plan.waypoints.quaternion
    if tuple(q_keep) != tuple(q_snap) and abs(q_snap[2]) > 1e-6:
        ok('量化改变了目标姿态（四元数 z: {:.3f} → {:.3f}）'.format(
            q_keep[2], q_snap[2]))
    else:
        fail('量化没改变目标姿态，可能没接上')

    # (b) 抓不准：偏差直接加在视觉给的抓取点上。
    shifted = planner.plan_grasp(good, model, object_offset=(0.013, -0.029))
    close('抓取点被平移了 (13, -29) mm', shifted.waypoints.grasp,
          [0.0 + 0.013, 0.30 - 0.029, ground + 0.012 + 0.0085], 1e-9)
    close('平移不影响起手高度',
          shifted.waypoints.pre_grasp[2] - shifted.waypoints.grasp[2], 0.08, 1e-12)
    picked_offset, _ = planner.first_graspable(
        [good], model, object_offset=(0.01, 0.0))
    close('first_graspable 也认 object_offset', picked_offset.waypoints.grasp[0],
          0.01, 1e-9)

    # (c) 放置点：place_x/place_y 以前声明了却没用，计划里的 place 就是抓取点。
    wp_x = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                        approach_m=0.08, lift_m=0.10)
    close('放置点用参数给的 xy', wp_x.place, [-0.175, 0.090, ground + 0.08], 1e-12)
    if not np.allclose(wp_x.place, plan.waypoints.grasp):
        ok('放置点不再等于抓取点')
    else:
        fail('放置点仍然等于抓取点（place_x/place_y 没生效）')
    wp_z = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                        place_z=-0.0405 + 0.055,
                                        place_z_frame='base')
    close('place_z + frame=base 直接用（base_link 的 z）', wp_z.place[2],
          -0.0405 + 0.055, 1e-12)
    wp_t = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                        place_z=0.055, place_z_frame='table')
    close('place_z + frame=table 加 table_z（"桌面上方 55 mm"）', wp_t.place[2],
          -0.0405 + 0.055, 1e-12)
    if abs(wp_t.place[2] - wp_z.place[2]) > 1e-12:
        fail('frame=table 与 frame=base 对同一高度算出了不同的 z')
    else:
        ok('两种 frame 写法能对上（省得再自己减 table_z）')
    # (d) 放置高度还能"再抬高几个边长"（2026-10-31：松手时物块蹭台面）
    wp_s = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                        place_surface_m=0.055,
                                        place_clearance_m=0.003)
    close('松手时物块底面停在台面上方 3 mm',
          wp_s.place[2] - (ground + 0.055),
          plan.tcp_offset_m + 0.5 * plan.object_edge_m + 0.003, 1e-12)
    wp_l = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                        place_surface_m=0.055,
                                        place_clearance_m=0.003,
                                        place_lift_edges=0.5)
    close('再抬 0.5 个边长 = 再高半个边长', wp_l.place[2] - wp_s.place[2],
          0.5 * plan.object_edge_m, 1e-12)
    wp_l1 = planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                         place_z=0.055, place_z_frame='table',
                                         place_lift_edges=1.0)
    close('place_z 的写法也吃这个抬高', wp_l1.place[2] - wp_t.place[2],
          plan.object_edge_m, 1e-12)

    try:
        planner.pick_place_waypoints(plan, (-0.175, 0.090), ground,
                                     place_z=0.055, place_z_frame='floor')
    except ValueError as exc:
        if 'table' in str(exc) and 'base' in str(exc):
            ok('frame 写错立刻报错，并说明只能是 table/base')
        else:
            fail(f'frame 报错没讲清楚两种取值：{exc}')
    else:
        fail('place_z_frame 写成 floor 竟然没报错')


def run_machine(task, gripper_ready, events):
    """按事件序列跑一遍，返回经过的状态列表。"""
    machine = task.TaskMachine(max_retries=2, gripper_ready=gripper_ready)
    trail = [machine.state]
    out = []
    for ev in events:
        dec = machine.decide(ev)
        trail.append(machine.state)
        out.append(dec)
    return machine, trail, out


def check_task(task, gripper):
    print('\n2. 状态机：正常路径')
    machine, trail, out = run_machine(task, True, [
        task.Event.START, task.Event.TARGET,
        task.Event.SUCCEEDED, task.Event.SUCCEEDED,   # 起手位 -> 下探
        task.Event.SUCCEEDED,                          # 合爪
        task.Event.SUCCEEDED,                          # 抬升
        task.Event.SUCCEEDED,                          # 移到放置点
        task.Event.SUCCEEDED,                          # 开爪
        task.Event.SUCCEEDED,                          # 退回
    ])
    want = [task.State.IDLE, task.State.OBSERVE, task.State.MOVE_PRE_GRASP,
            task.State.APPROACH, task.State.CLOSE_GRIPPER, task.State.LIFT,
            task.State.MOVE_PLACE, task.State.OPEN_GRIPPER, task.State.RETREAT,
            task.State.DONE]
    if trail == want:
        ok('一路走到 DONE：' + ' -> '.join(s.value for s in trail[1:]))
    else:
        fail('路径不对：\n  got  ' + ' -> '.join(s.value for s in trail) +
             '\n  want ' + ' -> '.join(s.value for s in want))
    if all(d.action for d in out):
        ok('每一步都给了动作：' + ', '.join(d.action for d in out))

    print('\n3. 状态机：夹爪没标定时在合爪前停住')
    machine, trail, out = run_machine(task, False, [
        task.Event.START, task.Event.TARGET,
        task.Event.SUCCEEDED, task.Event.SUCCEEDED])
    last = out[-1]
    if trail[-1] is task.State.FAULT:
        ok(f'进 FAULT：{last.note}')
    else:
        fail(f'夹爪没标定却继续往下走到了 {trail[-1].value}')
    if task.State.CLOSE_GRIPPER not in trail:
        ok('确实没进 CLOSE_GRIPPER')
    else:
        fail('进了 CLOSE_GRIPPER —— 那会发 hand 命令')
    # 前四步必须还能验（否则夹爪没标定之前什么都测不了）
    if task.State.APPROACH in trail:
        ok('起手位与下探这两步照常执行')

    print('\n4. 状态机：重试与放弃')
    machine = task.TaskMachine(max_retries=2, gripper_ready=True)
    machine.decide(task.Event.START)
    machine.decide(task.Event.TARGET)
    d = machine.decide(task.Event.FAILED)
    if d.state is task.State.MOVE_PRE_GRASP and d.retries == 1:
        ok(f'第一次失败：原地重试（{d.note}）')
    else:
        fail(f'第一次失败应原地重试，实际 {d.state.value} retries={d.retries}')
    machine.decide(task.Event.FAILED)
    d = machine.decide(task.Event.FAILED)
    if d.state is task.State.FAULT:
        ok(f'重试用完进 FAULT（{d.note}）')
    else:
        fail(f'重试用完应进 FAULT，实际 {d.state.value}')

    print('\n5. 状态机：夹爪那两步失败不重试')
    machine = task.TaskMachine(max_retries=5, gripper_ready=True)
    for ev in (task.Event.START, task.Event.TARGET,
               task.Event.SUCCEEDED, task.Event.SUCCEEDED):
        machine.decide(ev)          # 到 CLOSE_GRIPPER
    if machine.state is not task.State.CLOSE_GRIPPER:
        fail(f'没走到 CLOSE_GRIPPER（现在 {machine.state.value}）')
    d = machine.decide(task.Event.FAILED)
    if d.state is task.State.FAULT:
        ok(f'合爪失败直接认输（{d.note}）')
    else:
        fail(f'合爪失败不该重试，实际去了 {d.state.value}')

    print('\n6. 状态机：超时、叫停、重来')
    machine = task.TaskMachine(max_retries=2, gripper_ready=True)
    machine.decide(task.Event.START)
    d = machine.decide(task.Event.TIMEOUT)
    if d.state is task.State.FAULT:
        ok(f'等目标超时进 FAULT（{d.note}）')
    else:
        fail(f'等目标超时该进 FAULT，实际 {d.state.value}')
    d = machine.decide(task.Event.START)
    if d.state is task.State.OBSERVE:
        ok('FAULT 之后 START 能回到 OBSERVE')
    else:
        fail(f'FAULT 之后 START 应回 OBSERVE，实际 {d.state.value}')

    machine = task.TaskMachine(max_retries=2, gripper_ready=True)
    for ev in (task.Event.START, task.Event.TARGET, task.Event.SUCCEEDED):
        machine.decide(ev)          # 到 APPROACH
    d = machine.decide(task.Event.ABORT)
    if d.state is task.State.FAULT:
        ok(f'半路叫停进 FAULT（{d.note}）')
    else:
        fail(f'半路叫停应进 FAULT，实际 {d.state.value}')

    machine = task.TaskMachine(max_retries=2, gripper_ready=True)
    d = machine.decide(task.Event.SUCCEEDED)
    if d.state is task.State.IDLE:
        note(f'IDLE 收到 SUCCEEDED 被忽略（{d.note}）')
    else:
        fail(f'IDLE 收到 SUCCEEDED 应忽略，实际 {d.state.value}')

    print('\n7. 状态机：没有死胡同')
    missing_action = [s.value for s in task.State if s not in task.ACTIONS]
    if missing_action:
        fail(f'这些状态没有动作：{missing_action}')
    else:
        ok(f'{len(task.State)} 个状态都有动作')
    terminal = {task.State.IDLE, task.State.DONE, task.State.FAULT, task.State.OBSERVE}
    dead = [s.value for s in task.State
            if s not in terminal and s not in task.NEXT]
    if dead:
        fail(f'这些状态没有下一站：{dead}')
    else:
        ok('所有动作状态都在 NEXT 里有下一站')

    # 每个状态都要能被 ABORT 打断（安全侧的要求）
    bad = []
    for state in task.State:
        m = task.TaskMachine(max_retries=2, gripper_ready=True)
        m.state = state
        if m.decide(task.Event.ABORT).state is not task.State.FAULT:
            bad.append(state.value)
    if bad:
        fail(f'这些状态不吃 ABORT：{bad}')
    else:
        ok(f'{len(task.State)} 个状态都能被 ABORT 打断')


def check_node_api(geometry, gripper, planner, scene, task):
    """pick_place_node 不能离线跑，但至少盯着它别把接口名写错。

    写错属性名是这类代码最常见的翻车方式：离线看着挺好，到 VM 上一 import 就炸，
    而那一炸要占掉一次开机、一次摆方块的机会。
    """
    print('\n8. pick_place_node 引用的接口都真的存在')
    path = os.path.join(GRASP, 'dummy_vc_grasp', 'pick_place_node.py')
    if not os.path.exists(path):
        fail('pick_place_node.py 不在了')
        return
    try:
        with open(path, encoding='utf-8') as fh:
            text = fh.read()
        tree = ast.parse(text, filename=path)
    except (OSError, SyntaxError) as exc:
        fail(f'解析 pick_place_node.py 失败：{exc}')
        return
    modules = {'g': geometry, 'gr': gripper, 'grasp_planner': planner,
               'scene': scene, 'task_mod': task}
    referenced = set()
    unknown = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute) or not isinstance(node.value, ast.Name):
            continue
        module = modules.get(node.value.id)
        if module is None:
            continue
        name = f'{node.value.id}.{node.attr}'
        referenced.add(name)
        if not hasattr(module, node.attr):
            unknown.append(f'{name}（第 {node.lineno} 行）')
    for item in unknown:
        fail(f'pick_place_node.py 用了不存在的接口 {item}')
    if not unknown:
        ok(f'引用的 {len(referenced)} 个接口都在：{sorted(referenced)}')
    if not referenced:
        fail('一个规划/状态机接口都没引用？节点大概没在干正事')
    # 四个"必须留在代码里"的安全点：光写在文档里，下一个人会忘。
    #
    # 2026-10-27 改了第三条：以前要求节点必须问 `gripper_model.command_for_width`
    # 要角度 —— 那条路在真机上走不通（`hand.set_angle` 发的是 0x382，而 0x382 在
    # 这块板子上是**触发标定**）。现在夹爪走旁路话题，所以守卫反过来：不许再回头
    # 去调那个模型。
    for needle, why in (
            ('gripper_gate', '合爪的判定（闭到多宽 / 夹住没有）必须走纯函数，别现场算'),
            ('plan_only', 'plan_only 必须跟着 execute 参数走'),
            ('execute', '默认不执行这件事必须写在代码里，不能只写在文档里'),
            ('GetCartesianPath', '下降必须是笛卡尔直线，退回关节空间只能作为兜底'),
    ):
        if needle in text:
            ok(f'代码里提到 {needle}（{why}）')
        else:
            fail(f'代码里找不到 {needle}（{why}）')
    for banned, why in (
            ('command_for_width', '夹爪不许再回头走驱动里的 hand.*（0x382 = 触发标定）'),
            ('set_angle', '夹爪不许再回头走驱动里的 hand.*（0x382 = 触发标定）'),
    ):
        if banned in text:
            fail(f'代码里还有 {banned}（{why}）')
        else:
            ok(f'代码里没有 {banned}（{why}）')

    # error_code 得配翻译：真机上只会给一个 -4，没翻译就只能猜。
    funcs = {node.name: node for node in ast.walk(tree)
             if isinstance(node, ast.FunctionDef)}
    if 'describe_moveit_error' not in funcs:
        fail('pick_place_node 里没有 describe_moveit_error')
        return
    env = {}
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.Assign, ast.AnnAssign)) and (
                getattr(node, 'name', None) == 'describe_moveit_error'
                or any(getattr(t, 'id', None) == '_MOVEIT_ERROR_TEXT'
                       for t in getattr(node, 'targets', []))):
            exec(compile(ast.Module([node], []), path, 'exec'), env)  # noqa: S102
    describe = env.get('describe_moveit_error')
    if describe is None:
        fail('describe_moveit_error 抠不出来（依赖了模块里的别的东西？）')
        return
    if describe(1) != '':
        fail('成功码不该带解释：{!r}'.format(describe(1)))
    else:
        ok('成功码（1）不带解释')
    for code in (-1, -4, -9, 99999):
        text = describe(code)
        if not text.startswith(': '):
            fail(f'{code} 的解释格式不对：{text!r}')
        else:
            ok(f'{code} → {text[2:][:28]}…')
    if 'move_group' not in describe(-4):
        fail('-4（CONTROL_FAILED）的解释没说去看 move_group 日志',
             '这个码恰恰是"规划好了但没执行"，线索在 move_group 那边')
    else:
        ok('-4 指了该去看哪段日志')


def check_scene(scene, geometry):
    """碰撞体数值：场景里最容易被当成"随便填个大盒子"糊过去的一层。"""
    print('\n9. 碰撞体（地面 / 地台）')
    table_z = geometry.project_scene_measurements().table_z.value
    close('scene_objects 的地面高度与 perception 的实测值一致',
          scene.DEFAULT_TABLE_Z, table_z, 1e-12)

    boxes, notes = scene.workspace_boxes()
    by_name = {box.name: box for box in boxes}
    if 'ground' not in by_name:
        fail('workspace_boxes() 里没有 ground')
    else:
        ground = by_name['ground']
        close('地面盒子的顶面正好落在 table_z 上', ground.top_z, table_z, 1e-12)
        if ground.bottom_z >= ground.top_z:
            fail('地面盒子厚度不为正，规划器会看不见它')
        else:
            ok('地面盒子有厚度（{:.0f} mm）'.format(
                (ground.top_z - ground.bottom_z) * 1000.0))
    # 地台是两层（2026-10-21 实测）：上层和底座一样大、高 1 cm；下层 20x20 cm，
    # 中心在 +y 4 cm（前沿 +14、后沿 -6）。
    if 'platform_upper' not in by_name:
        fail('workspace_boxes() 里没有 platform_upper（第二层）')
    else:
        upper = by_name['platform_upper']
        if upper.top_z > scene.DEFAULT_BASE_BOTTOM_Z_M:
            fail('第二层顶面 {:.4f} 高过了 base_link 的底面 {:.4f}'.format(
                upper.top_z, scene.DEFAULT_BASE_BOTTOM_Z_M))
        else:
            ok('第二层顶面 {:.4f} 在 base_link 底面之下，不会和底座面接触'.format(
                upper.top_z))
        close('第二层顶面离 base_link 底面正好留了 1 mm',
              scene.DEFAULT_BASE_BOTTOM_Z_M - upper.top_z, 0.001, 1e-9)
        close('第二层和底座一样大（10x10 cm）', upper.size[0], 0.10, 1e-12)
        close('第二层高 1 cm（那 1 mm 的缝从这里扣）', upper.size[2], 0.010 - 0.001, 1e-12)
        close('第二层居中（x）', upper.center[0], 0.0, 1e-12)
        close('第二层居中（y）', upper.center[1], 0.0, 1e-12)
    if 'platform_lower' not in by_name:
        fail('workspace_boxes() 里没有 platform_lower（第一层）')
    else:
        lower = by_name['platform_lower']
        close('第一层 20x20 cm', lower.size[0], 0.20, 1e-12)
        close('第一层高 1.5 cm', lower.size[2], 0.015, 1e-12)
        close('第一层中心在 +y 4 cm', lower.center[1], 0.04, 1e-12)
        close('第一层顶面接住第二层底面', lower.top_z, upper.bottom_z, 1e-12)
        close('第一层底面正好落在工作面上', lower.bottom_z, table_z, 1e-12)
    for msg in notes:
        note(msg)

    if scene.base_platform_boxes(base_bottom_z=-0.5, table_z=-0.5):
        fail('地台与工作面同高时还返回了盒子（会是一个零厚度的碰撞体）')
    else:
        ok('地台与工作面同高时不产生碰撞体')
    if expect_error('地面尺寸非正时抛 ValueError', ValueError,
                    scene.table_box, size_x=0.0):
        pass


def main():
    geometry, gripper, planner, scene, task = load_modules()
    check_offline_imports()
    if planner is None:
        print('\nFAILED（连模块都 import 不了）')
        return 1
    check_planner(gripper, planner)
    check_task(task, gripper)
    check_node_api(geometry, gripper, planner, scene, task)
    check_scene(scene, geometry)

    print()
    if FAILURES:
        print(f'FAILED ({len(FAILURES)} problem(s))')
        for msg in FAILURES:
            print(f'  - {msg}')
        return 1
    print(f'ALL CHECKS PASSED ({len(NOTES)} note(s))')
    for msg in NOTES:
        print(f'  - {msg}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
