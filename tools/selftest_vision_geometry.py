#!/usr/bin/env python3
"""离线自查视觉几何：像素 -> base_link 下的抓取点。

它要防的故障是这一句：

    抓取点看着挺合理，但整体偏了十几毫米（或者方向反了）

而这句故障在现场极难定位，因为它的三个来源长得一模一样：坐标轴约定写错、
去畸变没做、把有高度的物体当成贴在桌面上的平面图形。三者都会给出一个
"数值正常、位置偏掉"的点。所以这里把每一条都用合成数据钉死 ——
比如"相机在 0.6 m 高、朝下看，桌面上的一个已知点"这种可以手算的情形。

它盯的第二件事是**这一层不许 import ROS / OpenCV**：几何能在没有 ROS 的
机器上跑，是这个包唯一能离线验收的原因。顺手 import 一个 cv2 进来，
离线自测就没了。

    python tools/selftest_vision_geometry.py
"""
import ast
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PKG_ROOT = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy_vc_perception')
XACRO = os.path.join(REPO, 'dummy_vc_ws', 'src', 'dummy2_urdf',
                     'urdf', 'dummy2_urdf.xacro')

# 相机内参来自 dummy_controller/config/fingerprinter_camera.yaml —— 就是
# driver 里那个 USB 相机的实测值，不是编的。
FX, FY, CX, CY = 495.231697, 494.590030, 303.808212, 249.027693
DISTORTION = (0.077718, -0.127016, -0.000550, -0.001023, 0.0)

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
    """数值比对。tol 就是这条检查的判据，写在调用处，别藏进函数里。"""
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
    """这类检查断言的是"它会拒绝"，而不是"它算得对"。"""
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
    if PKG_ROOT not in sys.path:
        sys.path.insert(0, PKG_ROOT)
    try:
        from dummy_vc_perception import geometry, gripper
    except ImportError as exc:
        fail(f'import dummy_vc_perception 失败：{exc}（几何层只依赖 numpy）')
        return None, None
    return geometry, gripper


# --------------------------------------------------------------------------
# 0. 这一层必须是离线的
# --------------------------------------------------------------------------

ALLOWED_IMPORTS = {'math', 'dataclasses', 'typing', 'numpy'}


def check_offline_imports():
    print('--- 0. 几何层不许 import ROS / OpenCV')
    for name in ('geometry.py', 'gripper.py'):
        path = os.path.join(PKG_ROOT, 'dummy_vc_perception', name)
        with open(path, encoding='utf-8') as fh:
            tree = ast.parse(fh.read(), filename=path)
        found = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    found.add(alias.name.split('.')[0])
            elif isinstance(node, ast.ImportFrom):
                if node.level == 0 and node.module:
                    found.add(node.module.split('.')[0])
        extra = sorted(found - ALLOWED_IMPORTS)
        if extra:
            fail(f'{name} import 了 {extra}；这一层必须只有 {sorted(ALLOWED_IMPORTS)}，'
                 f'否则离线自测跑不起来')
        else:
            ok(f'{name} 只 import 了 {sorted(found)}')


# --------------------------------------------------------------------------
# 1. 相机模型与去畸变
# --------------------------------------------------------------------------

def check_camera_model(geometry):
    print('--- 1. 相机模型')
    cam = geometry.PinholeCamera(FX, FY, CX, CY, width=640, height=480,
                                 distortion=DISTORTION)
    from_info = geometry.PinholeCamera.from_camera_info(
        [FX, 0.0, CX, 0.0, FY, CY, 0.0, 0.0, 1.0], width=640, height=480,
        d=DISTORTION)
    close('from_camera_info 与直接构造等价', from_info.K, cam.K, 0.0)

    # 主点上的射线必然是光轴本身
    close('主点像素的射线 = 光轴 (0, 0, 1)', cam.ray_optical(CX, CY), [0, 0, 1], 1e-12)

    expect_error('fx <= 0 被拒绝', geometry.GeometryError,
                 geometry.PinholeCamera, 0.0, FY, CX, CY)
    expect_error('distortion 少一个数被拒绝', geometry.GeometryError,
                 geometry.PinholeCamera, FX, FY, CX, CY, None, None, (0.0, 0.0, 0.0))

    dx, dy = cam.metres_per_pixel(0.6)
    print(f'    0.6 m 处 1 像素 = {dx * 1000:.3f} mm (x) / {dy * 1000:.3f} mm (y)')
    # docs/01 说"0.6 m 桌面 1 像素 ≈ 1 mm"，实际是 1.21 mm。这个数决定了
    # 视觉的精度上限：想做到 5 mm，分割质心就得准到 4 个像素。
    if 1.0e-3 < dx < 1.5e-3:
        ok('0.6 m 处 1 像素的量级与 docs/01 的估算一致（1.2 mm，不是 1.0 mm）')
    else:
        fail(f'0.6 m 处 1 像素 = {dx:.4e} m，与 docs/01 的估算差得太远')


def check_project_roundtrip(geometry):
    print('--- 2. 投影 <-> 反投影（含畸变）')
    cam = geometry.PinholeCamera(FX, FY, CX, CY, width=640, height=480,
                                 distortion=DISTORTION)
    pixels = [(CX, CY), (0.0, 0.0), (639.0, 0.0), (0.0, 479.0), (639.0, 479.0),
              (100.0, 400.0)]
    worst = 0.0
    for u, v in pixels:
        depth = 0.6
        point = cam.point_optical(u, v, depth)
        back = cam.project(point)
        worst = max(worst, abs(back[0] - u), abs(back[1] - v))
    if worst <= 1e-6:
        ok(f'6 个像素（含四角）投影->反投影往返最大误差 {worst:.3e} px')
    else:
        fail(f'投影往返误差 {worst:.3e} px，超过 1e-6')

    # 畸变真的被用上了：偏角处的像素，去畸变前后差多少像素
    u, v = 639.0, 479.0
    x, y = cam.undistort_pixel(u, v)
    u_raw = FX * (u - CX) / FX + CX
    u_undistorted = FX * x + CX
    shift = abs(u_undistorted - u_raw)
    if shift > 1.0:
        ok(f'角落像素去畸变移动了 {shift:.2f} px（畸变没有被忽略）')
    else:
        fail(f'角落像素去畸变只移动 {shift:.3f} px，畸变像是没生效')


def check_plane_and_backproject(geometry):
    print('--- 3. 桌面反投影（相机 0.6 m 高、正朝下看）')
    cam = geometry.PinholeCamera(FX, FY, CX, CY, width=640, height=480,
                                 distortion=DISTORTION)
    # 朝下看：光学 z（前）-> base -z，光学 x（右）-> base +x，光学 y（下）-> base +y
    R = np.array([[1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.0, 0.0, 0.6]

    hit = geometry.intersect_plane_z([0.0, 0.0, 0.6], [0.0, 0.0, -1.0], 0.0)
    close('竖直射线的交点', hit, [0.0, 0.0, 0.0], 1e-12)
    if geometry.intersect_plane_z([0.0, 0.0, 0.6], [1.0, 0.0, 0.0], 0.0) is None:
        ok('与桌面平行的射线返回 None')
    else:
        fail('与桌面平行的射线没有返回 None')
    if geometry.intersect_plane_z([0.0, 0.0, 0.6], [0.0, 0.0, 1.0], 0.0) is None:
        ok('朝上的射线（交点在背后）返回 None')
    else:
        fail('朝上的射线没有返回 None')

    # 端到端：桌面上的已知点 -> 像素 -> 反投影回桌面上的点
    for target in ([0.12, -0.05, 0.0], [-0.09, 0.11, 0.0]):
        p_optical = R.T @ (np.asarray(target) - np.array([0.0, 0.0, 0.6]))
        u, v = cam.project(p_optical)
        recovered = geometry.backproject_pixel_to_plane(cam, u, v, T, 0.0)
        close(f'桌面点 {target[:2]} 往返', recovered, target, 1e-9)

    expect_error('相机在平面下方时拒绝', geometry.GeometryError,
                 geometry.backproject_pixel_to_plane, cam, CX, CY,
                 np.diag([1.0, -1.0, -1.0, 1.0]) @ np.eye(4), 0.0)
    expect_error('非 4x4 变换被拒绝', geometry.GeometryError,
                 geometry.backproject_pixel_to_plane, cam, 10.0, 10.0,
                 np.eye(3), 0.0)


def check_monocular_bias(geometry):
    print('--- 4. 单目兜底的固有偏差（物体有高度）')
    cam = geometry.PinholeCamera(FX, FY, CX, CY, distortion=DISTORTION)
    R = np.array([[1.0, 0.0, 0.0],
                  [0.0, -1.0, 0.0],
                  [0.0, 0.0, -1.0]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [0.0, 0.0, 0.6]

    cube = 0.04
    true_center = geometry.object_center_on_table(0.10, 0.0, 0.0, cube)
    close('物体中心高度 = 桌面 + 边长/2', true_center, [0.10, 0.0, cube / 2.0], 1e-12)

    top = np.array([0.10, 0.0, cube])          # 顶面中心（相机真正看到的点）
    p_optical = R.T @ (top - np.array([0.0, 0.0, 0.6]))
    u, v = cam.project(p_optical)
    recovered = geometry.backproject_pixel_to_plane(cam, u, v, T, 0.0)
    inflation = geometry.monocular_planar_inflation(cube, 0.6)
    close('反投影落点 = 真实半径 x (z/(z-h))',
          recovered, [0.10 * inflation, 0.0, 0.0], 1e-9)
    error_mm = abs(recovered[0] - 0.10) * 1000.0
    print(f'    40 mm 方块在 0.6 m 处：单目把它算远了 {error_mm:.1f} mm'
          f'（放大 {inflation:.4f} 倍）')
    if error_mm > 5.0:
        note(f'单目兜底在 40 mm 方块上就偏 {error_mm:.1f} mm —— 阶段 4 的验收线'
             f'是 10 mm，只剩一半余量；深度相机才谈得上精度')
    expect_error('物体高过相机时拒绝', geometry.GeometryError,
                 geometry.monocular_planar_inflation, 0.7, 0.6)


# --------------------------------------------------------------------------
# 5. 顶抓位姿
# --------------------------------------------------------------------------

def check_top_down_pose(geometry):
    print('--- 5. 顶抓位姿的轴向')
    yaw = 0.0
    R = geometry.top_down_grasp_matrix(yaw)
    close('+y（接近方向）朝下', R @ np.array([0.0, 1.0, 0.0]), [0.0, 0.0, -1.0], 1e-12)
    close('+x（开合方向）指向 yaw', R @ np.array([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], 1e-12)
    close('+z = x × y', R @ np.array([0.0, 0.0, 1.0]), [0.0, 1.0, 0.0], 1e-12)
    close('正交', R.T @ R, np.eye(3), 1e-12)
    close('行列式 = 1（右手系，不是镜像）', np.linalg.det(R), 1.0, 1e-12)

    yaws = (0.0, math.pi / 4.0, math.pi / 2.0, math.pi, -0.3)
    bad = []
    for yaw in yaws:
        R = geometry.top_down_grasp_matrix(yaw)
        closing = R @ np.array([1.0, 0.0, 0.0])
        want = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        if abs(np.linalg.det(R) - 1.0) > 1e-12:
            bad.append(f'yaw={yaw} 行列式 {np.linalg.det(R)}')
        elif np.max(np.abs(closing - want)) > 1e-12:
            bad.append(f'yaw={yaw} 开合方向 {closing} != {want}')
    if bad:
        for item in bad:
            fail(item)
    else:
        ok(f'{len(yaws)} 个 yaw 下开合方向都在桌面内、且等于 (cos yaw, sin yaw, 0)')

    q = geometry.top_down_grasp_quaternion(0.7)
    close('四元数长度为 1', np.linalg.norm(q), 1.0, 1e-12)
    if q[3] >= 0.0:
        ok(f'四元数取 w >= 0 的表示（w={q[3]:.6f}）')
    else:
        fail(f'四元数 w={q[3]} 为负，同一个姿态会有两种日志写法')
    close('四元数 -> 矩阵与原始矩阵一致',
          geometry.matrix_from_quaternion(q),
          geometry.top_down_grasp_matrix(0.7), 1e-12)
    expect_error('零四元数被拒绝', geometry.GeometryError,
                 geometry.matrix_from_quaternion, (0.0, 0.0, 0.0, 0.0))


def check_waypoints(geometry):
    print('--- 6. 抓取路点')
    target = [0.20, -0.05, 0.02]
    wp = geometry.grasp_waypoints(target, yaw_rad=math.pi / 2.0)
    close('pre_grasp = 抓取点 + 8 cm', wp.pre_grasp, [0.20, -0.05, 0.10], 1e-12)
    close('grasp = 抓取点', wp.grasp, target, 1e-12)
    close('lift = 抓取点 + 10 cm', wp.lift, [0.20, -0.05, 0.12], 1e-12)
    close('place 缺省等于抓取点', wp.place, target, 1e-12)
    close('四个路点共用顶抓姿态',
          wp.quaternion, geometry.top_down_grasp_quaternion(math.pi / 2.0), 1e-12)
    close('路点与 yaw 无关（抬升都在 base_link 的 +z 上）',
          geometry.grasp_waypoints(target, yaw_rad=0.0).pre_grasp,
          wp.pre_grasp, 1e-12)
    expect_error('approach <= 0 被拒绝', geometry.GeometryError,
                 geometry.grasp_waypoints, target, 0.0, 0.0)
    expect_error('抓取点不是 3 维被拒绝', geometry.GeometryError,
                 geometry.grasp_waypoints, [0.1, 0.2], 0.0)


def check_xacro_camera_note(geometry):
    print('--- 7. xacro 里的相机占位姿态（只提示，不判红）')
    try:
        with open(XACRO, encoding='utf-8') as fh:
            text = fh.read()
    except OSError as exc:
        fail(f'读不到 {XACRO}: {exc}')
        return

    def prop(name, default):
        hit = re.search(r'name="%s"\s+value="([^"]+)"' % name, text)
        return float(hit.group(1)) if hit else default

    cam_xyz = (prop('camera_offset_x', 0.30), prop('camera_offset_y', 0.0),
               prop('camera_offset_z', 0.60))
    mount = tuple(math.radians(prop('camera_mount_%s_deg' % axis, 0.0))
                  for axis in ('roll', 'pitch', 'yaw'))
    T_base_cam = geometry.transform_from_xyz_rpy(cam_xyz, mount)
    # camera_link 的 +x 是镜头方向（xacro 自己的注释这么写的），
    # 而 optical 的 z（前）在 camera_link 里指向 +x。
    T_cam_opt = geometry.transform_from_xyz_rpy((0.0, 0.0, 0.0),
                                                (-math.pi / 2.0, 0.0, -math.pi / 2.0))
    T = T_base_cam @ T_cam_opt
    axis = T[:3, 2]
    print(f'    占位相机：位置 {tuple(round(v, 3) for v in cam_xyz)}，'
          f'光轴在 base_link 里指向 ({axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f})')
    if axis[2] > -0.9:
        note('xacro 里那个相机固定关节是"标定前的兜底"，按它自己的说明'
             '（camera_link 的 +x 是镜头方向）算出来并没有朝下 —— '
             '现在不要拿它做反投影，等手眼标定或装好支架后改这里')
    close('变换求逆 = 反向变换', geometry.invert_transform(T) @ T, np.eye(4), 1e-12)


# --------------------------------------------------------------------------
# 8. 夹爪映射：没实测就必须拒绝
# --------------------------------------------------------------------------

def check_gripper(gripper, geometry):
    print('--- 8. 夹爪开口映射')
    placeholder = gripper.unverified_placeholder_model()
    if placeholder.verified:
        fail('占位映射自称已实测')
    else:
        ok('占位映射 verified=False')
    if len(placeholder.unmeasured()) == 4:
        ok('四个端点都标着未实测')
    else:
        fail(f'占位映射未实测的端点有 {len(placeholder.unmeasured())} 个，应该是 4 个')
    expect_error('未实测时 command_for_width 拒绝下发',
                 gripper.UnverifiedGripperModel,
                 placeholder.command_for_width, 0.03)

    measured = gripper.opening_endpoints_from_measurements(
        closed_width_m=0.0, closed_angle_deg=30.0,
        open_width_m=0.046, open_angle_deg=0.0,
        where_closed='阶段 0.5 合到底', where_open='阶段 0.5 全开')
    if measured.verified:
        ok('四个端点都是实测值时 verified=True')
    else:
        fail('实测模型仍被当成未验证')
    close('开口 46 mm -> 角度 0', measured.command_for_width(0.046), 0.0, 1e-12)
    close('开口 0 mm -> 角度 30', measured.command_for_width(0.0), 30.0, 1e-12)
    close('开口 23 mm -> 角度 15（线性插值）', measured.command_for_width(0.023), 15.0, 1e-12)
    close('反向换算', measured.width_for_angle(15.0), 0.023, 1e-12)
    close('夹 40 mm 方块需要 42 mm 开口', measured.grasp_width_for(0.04), 0.042, 1e-12)

    expect_error('超出实测范围的开口被拒绝', geometry.GeometryError,
                 measured.command_for_width, 0.05)
    expect_error('开口端点写反被拒绝', geometry.GeometryError,
                 gripper.opening_endpoints_from_measurements,
                 0.046, 0.0, 0.0, 30.0, 'x', 'y')
    expect_error('非 Measurement 被拒绝', geometry.GeometryError,
                 gripper.FingerOpeningModel, 0.0, 0.046, 30.0, 0.0)
    print(f'    {measured.describe()}')


def check_measurements(geometry):
    print('--- 9. 还没实测的数被标出来了')
    scene = geometry.default_scene_measurements()
    unmeasured = scene.unmeasured()
    close('默认三个数都没实测', len(unmeasured), 3.0, 0.0)
    print(f'    未实测：{scene.describe_unmeasured()}')
    measured = geometry.Measurement(0.123, 'm', 'measured', '尺子')
    if measured.is_measured:
        ok("source='measured' 的 Measurement 认作已实测")
    else:
        fail("source='measured' 仍被当成未实测")
    expect_error('source 拼错被拒绝', geometry.GeometryError,
                 geometry.Measurement, 0.1, 'm', 'measured-ish', '尺子')


def main():
    print(f'reading {os.path.relpath(PKG_ROOT, REPO)}')
    geometry, gripper = load_modules()
    if geometry is None:
        print('\nFAILED (geometry layer not importable)')
        return 1

    check_offline_imports()
    check_camera_model(geometry)
    check_project_roundtrip(geometry)
    check_plane_and_backproject(geometry)
    check_monocular_bias(geometry)
    check_top_down_pose(geometry)
    check_waypoints(geometry)
    check_xacro_camera_note(geometry)
    check_gripper(gripper, geometry)
    check_measurements(geometry)

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
