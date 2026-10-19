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

# Gemini 335L 实测（2026-10-16 在 VM 上 echo 到的真实 CameraInfo）：
# distortion_model 写 plumb_bob，d 却是 8 个数——那是 rational_polynomial
# 的 (k1,k2,p1,p2,k3,k4,k5,k6)。只该用前 5 个，并且把丢掉的个数记下来。
# 这台相机的 k4..k6 全是 0，丢掉没代价；但代码不能因为 8 个数就整条挂掉。
GEMINI_K = (614.2484741210938, 0.0, 641.4732666015625,
            0.0, 614.148193359375, 365.4802551269531,
            0.0, 0.0, 1.0)
GEMINI_D = (-0.02929498627781868, 0.03310345113277435,
            -9.030981891555712e-05, 9.581100312061608e-05,
            -0.01202486827969551, 0.0, 0.0, 0.0)

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


def xacro_property(name, default):
    """从 xacro 里读一个 <xacro:property> 的数值；读不到/不是数就返回 default。

    xacro 是 URDF 的权威，所以这里直接读源文件，不另抄一份常量。
    """
    try:
        with open(XACRO, encoding='utf-8') as fh:
            text = fh.read()
    except OSError:
        return default
    hit = re.search(r'name="%s"\s+value="([^"]+)"' % name, text)
    if not hit:
        return default
    try:
        return float(hit.group(1))
    except ValueError:
        return default


def load_modules():
    if PKG_ROOT not in sys.path:
        sys.path.insert(0, PKG_ROOT)
    try:
        from dummy_vc_perception import depth as depth3d
        from dummy_vc_perception import detection, geometry, gripper
    except ImportError as exc:
        fail(f'import dummy_vc_perception 失败：{exc}（几何层只依赖 numpy）')
        return None, None, None, None
    return geometry, gripper, detection, depth3d


# --------------------------------------------------------------------------
# 0. 这一层必须是离线的
# --------------------------------------------------------------------------

ALLOWED_IMPORTS = {'math', 'dataclasses', 'typing', 'numpy'}


def check_offline_imports():
    print('--- 0. 几何层不许 import ROS / OpenCV')
    for name in ('geometry.py', 'gripper.py', 'detection.py', 'depth.py'):
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
# 0b. 节点用到的接口必须真的存在
# --------------------------------------------------------------------------

NODE_PATH = os.path.join(PKG_ROOT, 'dummy_vc_perception', 'vision_node.py')


def check_node_api(geometry, gripper, detection, depth3d):
    """节点不能离线跑，所以至少盯着它别把接口名写错。

    这是这一层能做的"最接近集成测试"的检查：`geometry.foo` / `detection.bar`
    里的属性名是唯一能在没有 ROS 的机器上验的东西，而写错属性名正是这类代码
    最常见的翻车方式（要到 VM 上 import 才炸）。
    """
    print('--- 0b. vision_node 引用的 geometry / detection 接口都真的存在')
    if not os.path.exists(NODE_PATH):
        fail(f'{os.path.relpath(NODE_PATH, REPO)} 不在了')
        return
    with open(NODE_PATH, encoding='utf-8') as fh:
        tree = ast.parse(fh.read(), filename=NODE_PATH)
    modules = {'geometry': geometry, 'gripper': gripper,
               'detection': detection, 'depth3d': depth3d}
    referenced = set()
    unknown = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        if not isinstance(node.value, ast.Name):
            continue
        module = modules.get(node.value.id)
        if module is None:
            continue
        name = f'{node.value.id}.{node.attr}'
        referenced.add(name)
        if not hasattr(module, node.attr):
            unknown.append(f'{name}（第 {node.lineno} 行）')
    for item in unknown:
        fail(f'vision_node.py 用了不存在的接口 {item}')
    if not unknown:
        ok(f'引用的 {len(referenced)} 个接口都在：{sorted(referenced)}')
    if not referenced:
        fail('一个 geometry/detection 接口都没引用？节点大概没在干正事')


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

    gemini = geometry.PinholeCamera.from_camera_info(
        GEMINI_K, width=1280, height=720, d=GEMINI_D)
    close('8 个畸变系数只取前 5 个', gemini.distortion, GEMINI_D[:5], 0.0)
    if gemini.ignored_distortion == 3:
        ok('并记下丢掉了 3 个系数（k4,k5,k6）')
    else:
        fail(f'ignored_distortion 应为 3，实际 {gemini.ignored_distortion}')
    close('Gemini 的 fx/fy 解析正确', [gemini.fx, gemini.fy],
          [614.2484741210938, 614.148193359375], 0.0)

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
    print('--- 7. xacro 里的相机安装姿态')
    cam_xyz = (xacro_property('camera_offset_x', 0.30),
               xacro_property('camera_offset_y', 0.0),
               xacro_property('camera_offset_z', 0.60))
    mount = tuple(math.radians(xacro_property('camera_mount_%s_deg' % axis, 0.0))
                  for axis in ('roll', 'pitch', 'yaw'))
    T_base_cam = geometry.transform_from_xyz_rpy(cam_xyz, mount)
    # camera_link 的 +x 是镜头方向（xacro 自己的注释这么写的），
    # 而 optical 的 z（前）在 camera_link 里指向 +x。
    T_cam_opt = geometry.transform_from_xyz_rpy((0.0, 0.0, 0.0),
                                                (-math.pi / 2.0, 0.0, -math.pi / 2.0))
    T = T_base_cam @ T_cam_opt
    axis = T[:3, 2]
    depression = math.degrees(math.asin(min(1.0, max(-1.0, -axis[2]))))
    print(f'    相机：位置 {tuple(round(v, 3) for v in cam_xyz)}，'
          f'光轴在 base_link 里指向 ({axis[0]:.3f}, {axis[1]:.3f}, {axis[2]:.3f})'
          f'（低于水平面 {depression:.1f}°）')
    # 这条是真检查，不是提示：镜头必须"明显朝下"，否则平面反投影拿到的
    # 是墙或者无限远，而且不会报错、只会给出离谱的抓取点。
    # 实装的俯角是地面拟合量出来的 33.3°（竖直分量 -0.55），
    # 所以门限放到 -0.15（约 8.6°）—— 只要"明显朝下"就算过，具体角度不判。
    if axis[2] > -0.15:
        fail('xacro 里相机光轴几乎是水平的（竖直分量 {:.2f}）—— 平面反投影'
             '拿不到工作面。camera_link 的 +x 才是镜头方向，要让镜头朝下必须'
             '绕 y 轴转 pitch，不是绕 x 轴转 roll'.format(axis[2]))
    if axis[2] > -0.9:
        note('相机是斜下 {:.0f}°，不是垂直俯视：平面反投影在倾斜方向上误差被 '
             '1/sin 放大（33° 时约 1.8 倍），画面远端更差 —— 方块要放在画面'
             '中心附近'.format(depression))
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


def check_measurements(geometry, gripper):
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

    print('--- 9b. 本机的实测值，以及它必须和 xacro 一致')
    real = geometry.project_scene_measurements()
    if real.unmeasured():
        fail('本机实测值里还有没实测的：' + real.describe_unmeasured())
    else:
        ok('本机三个数都是 measured：' + '; '.join(
            '{} = {}'.format(name, m.describe())
            for name, m in real.as_dict().items()))
    # 同一个数写在两个地方（xacro 给 MoveIt 用，代码给抓取点用），
    # 改一处不改另一处，规划用的 TCP 和视觉算的抓取点就差几毫米 ——
    # 而这正好是"对准了但没夹住"的样子。
    xacro_tcp = xacro_property('tcp_offset_y', float('nan'))
    close('xacro 的 tcp_offset_y 与代码里的实测值一致',
          real.tcp_offset_y.value, xacro_tcp, 1e-12)
    # 方块是放在工作面上的，抓取点取它的中心，所以高度是边长的一半。
    close('24 mm 方块的中心在工作面上方 12 mm',
          geometry.object_center_on_table(0.0, 0.0, real.table_z.value,
                                          real.object_edge.value),
          np.array([0.0, 0.0, real.table_z.value + real.object_edge.value / 2.0]),
          1e-12)
    # 开口需求 = 边长 + clearance（clearance 是总余量，两侧各 1 mm），
    # 与 gripper.grasp_width_for() 的算法保持一致。
    need = real.object_edge.value + gripper.DEFAULT_GRASP_CLEARANCE_M
    close('夹 24 mm 方块需要 26 mm 开口', need, 0.026, 1e-12)
    close('grasp_width_for() 与上面那个式子一致',
          gripper.unverified_placeholder_model().grasp_width_for(
              real.object_edge.value),
          need, 1e-12)
    print(f'    抓取点高度 {real.object_edge.value / 2.0 * 1000:.1f} mm，'
          f'需要的开口 {need * 1000:.1f} mm（实机最大开口还没量过）')


def yaw_error_deg(got, want):
    """朝向差多少度（正方形是 90 度对称的，所以按 mod 90 比）。"""
    quarter = math.pi / 2.0
    diff = (got - want) % quarter
    return math.degrees(min(diff, quarter - diff))


def check_detection(geometry, gripper, detection):
    print('--- 10. 掩膜 -> 抓取点（合成数据，端到端）')
    camera = geometry.PinholeCamera(FX, FY, CX, CY, width=640, height=480,
                                    distortion=DISTORTION)
    table_z = 0.0
    edge = 0.024
    # 相机在 base_link 正上方 0.6 m、垂直朝下，图像上方是 base 的 +y：
    #   x_opt = +x、y_opt = -y（图像 y 朝下）、z_opt = -z（朝下看）
    T = np.eye(4)
    T[:3, :3] = np.array([[1.0, 0.0, 0.0],
                          [0.0, -1.0, 0.0],
                          [0.0, 0.0, -1.0]])
    T[:3, 3] = (0.0, 0.0, 0.6)
    T_optical_base = geometry.invert_transform(T)

    def pixel_of(point_base):
        p = T_optical_base @ np.append(np.asarray(point_base, float), 1.0)
        return camera.project(p[:3])

    def mask_of_square(centre, yaw_rad, half_px=18):
        """画出 base 平面上一个 24 mm 正方形所占的像素（含畸变、含透视）。"""
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        mask = np.zeros((480, 640), bool)
        cu, cv = pixel_of((centre[0], centre[1], table_z))
        for v in range(int(round(cv)) - half_px, int(round(cv)) + half_px + 1):
            for u in range(int(round(cu)) - half_px, int(round(cu)) + half_px + 1):
                if not (0 <= u < 640 and 0 <= v < 480):
                    continue
                hit = geometry.backproject_pixel_to_plane(
                    camera, u + 0.5, v + 0.5, T, table_z)
                dx = hit[0] - centre[0]
                dy = hit[1] - centre[1]
                if abs(c * dx + s * dy) <= edge / 2.0 and \
                        abs(-s * dx + c * dy) <= edge / 2.0:
                    mask[v, u] = True
        return mask

    for centre, yaw_deg in (((0.05, -0.02), 0.0), ((0.05, -0.02), 30.0),
                            ((-0.03, 0.04), 45.0)):
        yaw_rad = math.radians(yaw_deg)
        mask = mask_of_square(centre, yaw_rad)
        if int(mask.sum()) < 100:
            fail('合成掩膜只有 {} 个像素 —— 测试本身写错了'.format(int(mask.sum())))
            continue
        blobs = detection.blobs_from_labels(mask.astype(np.int32))
        if len(blobs) != 1:
            fail('标签图里应该只有一个 blob，拿到 {} 个'.format(len(blobs)))
            continue
        det = detection.detection_from_blob(blobs[0], camera, T, table_z, edge)
        tag = '{:.0f} 度'.format(yaw_deg)
        # 1.5 mm 的容差 = 一个像素多一点（0.6 m 处 1 px = 1.212 mm），
        # 因为掩膜本身就是按像素量化出来的。
        close('{}：抓取点 xy'.format(tag), det.position[:2], centre, 0.0015)
        close('{}：抓取点 z = 边长一半'.format(tag), det.position[2],
              table_z + edge / 2.0, 1e-12)
        err = yaw_error_deg(det.yaw_rad, yaw_rad)
        if err <= 2.0:
            ok('{}：拟合出的朝向差 {:.2f} 度'.format(tag, err))
        else:
            fail('{}：拟合出的朝向差了 {:.2f} 度（> 2.0）'.format(tag, err))
        # 3 mm 容差：掩膜是按像素量化出来的，一条 20 px 的边本来就允许差
        # 1~2 个像素（0.6 m 处 1 px = 1.212 mm）。
        close('{}：拟合出的两条边长与 24 mm 的差'.format(tag),
              det.extent_m, (edge, edge), 0.0030)
        close('{}：四元数与顶抓姿态一致'.format(tag), det.quaternion,
              geometry.top_down_grasp_quaternion(det.yaw_rad), 1e-12)
    print('    相机距离 {:.3f} m，该深度 1 像素 = {:.3f} mm'.format(
        det.distance_m, det.metres_per_pixel[0] * 1000.0))

    # 朝向为什么必须拟合：顶抓时两指要贴着方块的一条边进去。偏 45 度时
    # 需要的开口是边长 × sqrt(2)，24 mm 方块就是 34 mm —— 而实测手爪能
    # 给出的是 26 mm 开口，夹不住。
    diagonal = edge * math.sqrt(2.0)
    note('方块转 45 度时需要的开口是 {:.1f} mm，比 26 mm 大 —— '
         '所以朝向不能糊弄，偏了就是夹不住'.format(diagonal * 1000.0))

    # 反向：相机在平面下方 / 工作面在相机上方，都必须拒绝而不是算个假点
    below = T.copy()
    below[2, 3] = -0.6
    blob = detection.Blob.from_points(
        np.array([[u, v] for u in range(310, 330) for v in range(230, 250)],
                 float))
    expect_error('相机在平面下方时拒绝给出抓取点', geometry.GeometryError,
                 detection.detection_from_blob, blob, camera, below, table_z, edge)
    expect_error('工作面在相机上方时拒绝', geometry.GeometryError,
                 detection.detection_from_blob, blob, camera, T, 1.0, edge)

    # 小噪点要被门限挡掉；点数太少时不许硬凑出一个朝向
    tiny = np.zeros((480, 640), np.int32)
    tiny[100:103, 100:103] = 1
    if detection.blobs_from_labels(tiny):
        fail('9 个像素的噪点没被门限挡住')
    else:
        ok('小于面积门限的 blob 被丢掉')
    expect_error('点数太少时拒绝拟合朝向', geometry.GeometryError,
                 detection.fit_edge_yaw, np.zeros((5, 2)))


# --------------------------------------------------------------------------
# 11. 深度这一层（合成场景，端到端）
# --------------------------------------------------------------------------

DEPTH_DOWNSCALE = 4.0
DEPTH_W = int(1280 / DEPTH_DOWNSCALE)
DEPTH_H = int(720 / DEPTH_DOWNSCALE)
CUBE_EDGE = 0.024
CUBE_CENTRE = (0.0, 0.30, CUBE_EDGE / 2.0)
MOUNT_XYZ = (-0.21, 0.30, 0.15)


def synthetic_camera(geometry):
    """把 Gemini 的真实内参等比缩小，做一张便宜的合成图。

    真实图 1280x720 有 92 万像素，为了一个自测渲染它没必要——缩小 4 倍之后
    精度差 4 倍，但足以看出"几毫米 vs 几十毫米"的区别。畸变照原样带进来，
    这样去畸变那条路真的走到。
    """
    return geometry.PinholeCamera(
        GEMINI_K[0] / DEPTH_DOWNSCALE, GEMINI_K[4] / DEPTH_DOWNSCALE,
        GEMINI_K[2] / DEPTH_DOWNSCALE, GEMINI_K[5] / DEPTH_DOWNSCALE,
        width=DEPTH_W, height=DEPTH_H, distortion=GEMINI_D[:5])


def mount_transform(geometry, pitch_deg):
    """base_link -> camera_color_optical_frame，跟 xacro 里那两个关节一致。"""
    mount = geometry.transform_from_xyz_rpy(
        MOUNT_XYZ, (0.0, math.radians(pitch_deg), 0.0))
    optical = geometry.transform_from_xyz_rpy(
        (0.0, 0.0, 0.0), (math.radians(-90.0), 0.0, math.radians(-90.0)))
    return mount @ optical


def ray_box(origin, direction, lower, upper):
    """slab 法求射线与轴对齐立方体的最近交点参数 t；打不到就是 inf。"""
    near = np.full(direction.shape[0], -np.inf)
    far = np.full(direction.shape[0], np.inf)
    for k in range(3):
        d = direction[:, k]
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (lower[k] - origin[k]) / d
            t2 = (upper[k] - origin[k]) / d
        near = np.maximum(near, np.minimum(t1, t2))
        far = np.minimum(far, np.maximum(t1, t2))
    hit = (far > near) & (far > 0.0)
    t = np.where(near > 0.0, near, far)
    return np.where(hit, t, np.inf)


def render_scene(geometry, depth3d, camera, T):
    """合成一张深度图：24 mm 立方体站在 z=0 的地面上。

    返回 (depth 图, labels 图, 每个像素的光学系单位射线)。labels 里 1=方块、
    2=地面、0=什么都没有（地平线以上）。
    """
    grid_u, grid_v = np.meshgrid(np.arange(DEPTH_W, dtype=float),
                                 np.arange(DEPTH_H, dtype=float))
    xs, ys = depth3d.undistort_pixels(camera, grid_u.ravel(), grid_v.ravel())
    ray = np.stack([xs, ys, np.ones_like(xs)], axis=1)
    ray /= np.linalg.norm(ray, axis=1, keepdims=True)
    origin = T[:3, 3]
    direction = ray @ T[:3, :3].T

    half = CUBE_EDGE / 2.0
    lower = np.array([CUBE_CENTRE[0] - half, CUBE_CENTRE[1] - half, 0.0])
    upper = np.array([CUBE_CENTRE[0] + half, CUBE_CENTRE[1] + half, CUBE_EDGE])
    t_box = ray_box(origin, direction, lower, upper)

    with np.errstate(divide='ignore', invalid='ignore'):
        t_ground = (0.0 - origin[2]) / direction[:, 2]
    t_ground = np.where((t_ground > 0.0) & (direction[:, 2] < 0.0),
                        t_ground, np.inf)

    t = np.minimum(t_box, t_ground)
    labels = np.zeros(t.shape, dtype=np.int32)
    labels[t_box <= t_ground] = 1
    labels[(t_ground < t_box) & np.isfinite(t_ground)] = 2
    labels[~np.isfinite(t)] = 0

    depth = np.zeros(t.shape, dtype=np.float32)
    finite = np.isfinite(t)
    depth[finite] = (t[finite] * ray[finite, 2]).astype(np.float32)
    return (depth.reshape(DEPTH_H, DEPTH_W),
            labels.reshape(DEPTH_H, DEPTH_W), ray, direction, t)


def check_depth(geometry, depth3d):
    print('--- 11. 深度这一层（合成场景）')
    camera = synthetic_camera(geometry)

    # 11a. 向量版去畸变必须正好是投影的逆
    us = np.array([0.0, 40.0, 160.0, 300.0, DEPTH_W - 1.0])
    vs = np.array([0.0, 30.0, 90.0, 150.0, DEPTH_H - 1.0])
    x, y = depth3d.undistort_pixels(camera, us, vs)
    projected = [camera.project((xi, yi, 1.0)) for xi, yi in zip(x, y)]
    got_u = np.array([p[0] for p in projected])
    got_v = np.array([p[1] for p in projected])
    close('向量版去畸变是投影的逆（畸变真的生效）',
          [got_u, got_v], [us, vs], 1e-9)

    # 11b. 深度编码 -> 比例尺
    for encoding, want in (('16UC1', 0.001), ('mono16', 0.001),
                           ('32FC1', 1.0), ('64FC1', 1.0)):
        close(f'depth_scale_for({encoding})', depth3d.depth_scale_for(encoding),
              want, 0.0)
    expect_error('不认识的编码宁可报错也不猜', geometry.GeometryError,
                 depth3d.depth_scale_for, 'rgb8')

    # 11c. 端到端：合成深度图 -> 3D 点 -> 抓取点
    true_pitch = 20.0
    T = mount_transform(geometry, true_pitch)
    depth, labels, ray, direction, t = render_scene(geometry, depth3d, camera, T)
    box_px = int(np.count_nonzero(labels == 1))
    ground_px = int(np.count_nonzero(labels == 2))
    if box_px < 40:
        fail(f'合成图里方块只有 {box_px} 个像素，场景没搭对')
    else:
        ok(f'合成图：方块 {box_px} 个像素、地面 {ground_px} 个像素')

    points, point_labels = depth3d.points_from_labels(
        labels, depth, camera, T, 1.0, step=1)
    if box_px and np.count_nonzero(point_labels == 1) < 0.5 * box_px:
        fail('方块像素有一半以上没变成 3D 点')
    else:
        ok('方块的像素都变成了 base_link 下的 3D 点')
    scene = depth3d.detection_from_points(
        points[point_labels == 1], CUBE_EDGE, T, camera)
    err = np.linalg.norm(np.asarray(scene.position) - np.array(CUBE_CENTRE))
    print(f'    报出 ({scene.position[0]:+.4f}, {scene.position[1]:+.4f}, '
          f'{scene.position[2]:+.4f}) m，真值 ({CUBE_CENTRE[0]:+.4f}, '
          f'{CUBE_CENTRE[1]:+.4f}, {CUBE_CENTRE[2]:+.4f}) m，差 '
          f'{err * 1000:.2f} mm（像素 {1000 * float(np.mean(camera.metres_per_pixel(0.3))):.1f} mm 宽）')
    if err < 8e-3:
        ok(f'抓取点误差 {err * 1000:.2f} mm < 8 mm')
    else:
        fail(f'抓取点误差 {err * 1000:.2f} mm 超了 8 mm')
    if abs(scene.position[2] - CUBE_EDGE / 2.0) < 0.4e-3:
        ok('z 落在边长一半（顶面减半个边长）')
    else:
        fail(f'z = {scene.position[2] * 1000:.2f} mm，应该是 '
             f'{CUBE_EDGE / 2.0 * 1000:.1f} mm')
    # 斜下 20° 看这个方块，拟合出的外接矩形必然是长条：短边≈真实边长，长边被
    # 侧面拉长。尺寸判据只认短边，否则真方块永远过不了 —— 虚拟机上的实测就是
    # 34.9x25.5 和 46.6x25.8 两种，长边把 10 mm 的容差顶穿了。
    short_m, long_m = sorted(scene.extent_m)
    print(f'    外接矩形 {long_m * 1e3:.1f} x {short_m * 1e3:.1f} mm'
          f'（长边/短边 = {long_m / short_m:.2f}）')
    if abs(short_m - CUBE_EDGE) < 3e-3:
        ok(f'短边 {short_m * 1e3:.1f} mm 贴住真实边长 '
           f'{CUBE_EDGE * 1e3:.0f} mm（斜视下短边也会被压短一点，所以是 3 mm）')
    else:
        fail(f'短边 {short_m * 1e3:.1f} mm 与真实边长 '
             f'{CUBE_EDGE * 1e3:.0f} mm 不符')
    if scene.edge_error_m <= 0.01:
        ok(f'尺寸判据用短边，斜视下误差 {scene.edge_error_m * 1e3:.1f} mm '
           f'在 10 mm 以内（若用长边是 '
           f'{abs(long_m - CUBE_EDGE) * 1e3:.1f} mm，会被拒）')
    else:
        fail(f'短边误差 {scene.edge_error_m * 1e3:.1f} mm 超了 10 mm')

    # 11i. 掩膜里混进背景时：同一块掩膜的深度会分成两堆。方块在近处、桌面在后，
    #      取中位数就选了背景（真机日志：中位数 0.478，而方块在 0.339）。
    rng = np.random.default_rng(20261019)
    near = 0.34 + 0.002 * rng.standard_normal(80)
    far = 0.49 + 0.004 * rng.standard_normal(400)
    both = np.concatenate([near, far])
    keep = depth3d.near_depth_cluster(both)
    kept = both[keep]
    if kept.size and float(np.max(np.abs(kept - 0.34))) < 0.012:
        ok(f'两簇深度（0.34 m 有 80 点、0.49 m 有 400 点）挑出近处那簇 '
           f'{kept.size} 点，没被中位数带跑')
    else:
        fail(f'两簇深度挑错了：留下 {kept.size} 点，'
             f'中位数 {float(np.median(kept)) if kept.size else float("nan"):.3f} m')
    if depth3d.near_depth_cluster(np.array([0.4] * 20)).all():
        ok('只有一簇深度时不做裁剪（不该把唯一点集切碎）')
    else:
        fail('只有一簇深度时被裁掉了')

    # 11d. 用地面拟合反解相机倾角：真实 20°，URDF 里写的 16° -> 应补 +4°
    ground_opt = (ray[labels.ravel() == 2]
                  * t[labels.ravel() == 2][:, None])
    normal_opt, offset = depth3d.plane_from_points(ground_opt)
    assumed = mount_transform(geometry, 16.0)
    report = depth3d.plane_report(assumed[:3, :3] @ normal_opt, offset)
    print(f'    地面法向：光学系 ({normal_opt[0]:+.4f}, {normal_opt[1]:+.4f}, '
          f'{normal_opt[2]:+.4f})；标定报告 pitch 要补 '
          f'{report["pitch_correction_deg"]:+.2f}°、离地 '
          f'{report["camera_height_m"]:.4f} m')
    if abs(report['pitch_correction_deg'] - (true_pitch - 16.0)) < 0.4:
        ok(f'俯角补 {report["pitch_correction_deg"]:+.2f}° ≈ 真实-写的 = '
           f'{true_pitch - 16.0:+.1f}°（符号也对）')
    else:
        fail(f'俯角补 {report["pitch_correction_deg"]:+.2f}°，应该是 '
             f'{true_pitch - 16.0:+.1f}°——符号错或法向约定错')
    if abs(report['camera_height_m'] - MOUNT_XYZ[2]) < 2e-3:
        ok(f'相机离地 {report["camera_height_m"]:.4f} m ≈ xacro 里的 '
           f'{MOUNT_XYZ[2]:.2f} m')
    else:
        fail(f'相机离地 {report["camera_height_m"]:.4f} m，xacro 写 '
             f'{MOUNT_XYZ[2]:.2f} m')

    # 11e. 超量程/零深度的像素必须被丢掉，而不是当成 0 米
    holes = np.array(depth, copy=True)
    holes[labels == 1] = 0.0
    pts, plt = depth3d.points_from_labels(labels, holes, camera, T, 1.0, step=1)
    if np.count_nonzero(plt == 1) == 0:
        ok('深度为 0 的像素被丢掉（不会变成 base_link 原点上的点）')
    else:
        fail(f'深度为 0 的位置还是造出了 {np.count_nonzero(plt == 1)} 个点')
    far = np.full(depth.shape, 5.0, dtype=np.float32)
    expect_error('5 m 远（超出 max_range）的掩膜整个被拒',
                 geometry.GeometryError, depth3d.points_from_labels,
                 labels, far, camera, T, 1.0)



    # 11f. 拟合质量：真平面残差该是毫米级，弯的/混了东西的必须变大
    exact = np.array([[u, v, 0.4] for u in (-0.2, 0.0, 0.2) for v in (-0.2, 0.2)],
                     dtype=float)
    n_exact, total_exact, rms_exact = depth3d.plane_quality(exact, (0.0, 0.0, 1.0),
                                                           0.4)
    if rms_exact < 1e-9 and n_exact == total_exact:
        ok(f'精确平面：{n_exact}/{total_exact} 个点在内，残差 RMS '
           f'{rms_exact * 1e3:.3f} mm')
    else:
        fail(f'精确平面的残差 RMS 应该是 0，实得 {rms_exact:.6f} m（{n_exact}/'
             f'{total_exact} 个点）')
    bowed = exact.copy()
    bowed[:, 2] += 0.02 * (bowed[:, 0] ** 2 + bowed[:, 1] ** 2) / 0.04
    _, _, rms_bowed = depth3d.plane_quality(bowed, (0.0, 0.0, 1.0), 0.4)
    if rms_bowed > rms_exact + 1e-3:
        ok(f'弯掉的点云残差变大（{rms_bowed * 1e3:.2f} mm）—— 这个数能把'
           f'"深度值其实是斜距"这类问题暴露出来')
    else:
        fail(f'弯掉的点云残差没变大（{rms_bowed * 1e3:.3f} mm），这个检查是摆设')

    # 11g. 画面里大半不是地面时，RANSAC 必须还能找到地面。
    #      2026-10-19 的现场就是这样：整幅采样，机械臂、底座、方块、线缆都在画面
    #      里，地面只占三成。那时用整堆点做最小二乘，报出 0.206 m / RMS 11.2 mm，
    #      两个数都是错的，而且看起来很正常。
    junk = np.random.default_rng(7)
    grid = np.linspace(-0.25, 0.25, 20)
    gu, gv = np.meshgrid(grid, grid)
    floor_pts = np.stack([gu.ravel(), gv.ravel(), np.full(gu.size, 0.4)], axis=1)
    clutter = np.column_stack([
        junk.uniform(-0.25, 0.25, 400),
        junk.uniform(-0.25, 0.25, 400),
        junk.uniform(0.30, 0.60, 400)])
    mixed = np.vstack([floor_pts, clutter])
    junk.shuffle(mixed)
    n_mixed, off_mixed = depth3d.plane_from_points(mixed)
    kept, total_mixed, rms_mixed = depth3d.plane_quality(mixed, n_mixed, off_mixed)
    print(f'    混了 {clutter.shape[0]} 个非地面点（占 '
          f'{clutter.shape[0] / mixed.shape[0] * 100:.0f}%）：拟合出的地面过 '
          f'z = {-off_mixed:+.4f} m，法向 z 分量 {n_mixed[2]:+.4f}，内点 '
          f'{kept}/{total_mixed}，残差 RMS {rms_mixed * 1e3:.2f} mm')
    if abs(-off_mixed - 0.4) < 2e-3 and n_mixed[2] < -0.99:
        ok(f'非地面点占一半时仍然拟合到 z = 0.4 m（差 '
           f'{abs(-off_mixed - 0.4) * 1e3:.2f} mm）')
    else:
        fail(f'混进一半非地面点后拟合出 z = {-off_mixed:.4f} m、法向 '
             f'({n_mixed[0]:+.3f}, {n_mixed[1]:+.3f}, {n_mixed[2]:+.3f})，'
             f'应该还是 z = 0.4 m、法向 (0, 0, -1)')
    # 这里的 RMS 不可能像 11f 那样是 0：2 cm 的容差带里必然混进了几个非地面点，
    # 它们贡献的残差就是那几毫米。能要求的是"带宽的零头"—— 说明内点确实都贴着
    # 这个平面，而不是散在整条带子里。
    if kept >= 380 and rms_mixed < 0.5 * 0.02:
        ok(f'内点 {kept}/{total_mixed}、RMS {rms_mixed * 1e3:.2f} mm（不到容差'
           f' 2 cm 的零头）—— 这两个数能当"这次标定可不可信"的判据')
    else:
        fail(f'内点 {kept}/{total_mixed}、RMS {rms_mixed * 1e3:.2f} mm：'
             f'内点该接近 400、RMS 该在毫米级')

    # 11h. 画面里有一面墙比地面还大时，靠"地面是水平的"这条先验把它排掉。
    #      2026-10-19 真机上一帧就锁到了 85.7° 的竖直面，把"相机到地面"报成
    #      1.336 m（真实 0.15 m）。地面按重力必然水平，这条先验是免费的。
    wall = np.column_stack([
        np.full(600, 1.0),                       # x = 1.0 的墙面
        junk.uniform(-0.4, 0.4, 600),
        junk.uniform(-0.4, 0.4, 600)])
    floor_small = np.column_stack([
        junk.uniform(-0.2, 0.2, 300),
        junk.uniform(-0.2, 0.2, 300),
        np.full(300, 0.0)])
    room = np.vstack([wall, floor_small])
    junk.shuffle(room)
    n_free, off_free = depth3d.plane_from_points(room)
    # 相机在 base 里朝下看（和上面合成场景的约定一致），所以 T 的旋转部分把
    # 光学系的 z 轴映到 base 的 -z。
    T_down = np.array([[1.0, 0.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0, 0.0],
                       [0.0, 0.0, -1.0, 0.6],
                       [0.0, 0.0, 0.0, 1.0]])
    rot_t = T_down[:3, :3]
    room_opt = (room - T_down[:3, 3]) @ rot_t     # base -> optical（R^T 乘）
    n_pri, off_pri = depth3d.plane_from_points(
        room_opt, T_base_optical=T_down, max_tilt_deg=25.0)

    def plane_z_in_base(n_opt, off_opt):
        """把光学系里的平面搬到 base_link，取它在 z 轴上的截距。

        法向几乎水平（锁到了墙）时截距没有意义，返回 nan 而不是 inf。
        """
        n_base = rot_t @ n_opt
        point_base = rot_t @ (n_opt * off_opt) + T_down[:3, 3]
        if abs(n_base[2]) < 1e-6:
            return float('nan')
        return float(n_base @ point_base / n_base[2])

    free_z = plane_z_in_base(n_free, off_free)
    pri_z = plane_z_in_base(n_pri, off_pri)
    print(f'    墙 600 点 + 地面 300 点：不加先验锁到 z = {free_z:+.3f} m、'
          f'法向竖直分量 {abs((rot_t @ n_free)[2]):.3f}；加先验后 '
          f'z = {pri_z:+.3f} m、竖直分量 {abs((rot_t @ n_pri)[2]):.3f}')
    if abs((rot_t @ n_free)[2]) < 0.3 and abs(pri_z) < 5e-3:
        ok('一面大墙时，先验把拟合从墙面拉回了地面（+0.000 m）')
    else:
        fail(f'先验没起作用：不加先验竖直分量 {abs((rot_t @ n_free)[2]):.3f}'
             f'（该接近 0，即锁到墙），加先验后 z = {pri_z:+.4f}（该是 0）')


def main():
    print(f'reading {os.path.relpath(PKG_ROOT, REPO)}')
    geometry, gripper, detection, depth3d = load_modules()
    if geometry is None:
        print('\nFAILED (geometry layer not importable)')
        return 1

    check_offline_imports()
    check_node_api(geometry, gripper, detection, depth3d)
    check_camera_model(geometry)
    check_project_roundtrip(geometry)
    check_plane_and_backproject(geometry)
    check_monocular_bias(geometry)
    check_top_down_pose(geometry)
    check_waypoints(geometry)
    check_xacro_camera_note(geometry)
    check_gripper(gripper, geometry)
    check_measurements(geometry, gripper)
    check_detection(geometry, gripper, detection)
    check_depth(geometry, depth3d)

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
