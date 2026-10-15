#!/usr/bin/env python3
"""On the VM: show how MoveItConfigsBuilder resolves each config file.

Why this exists
---------------
`MoveItConfigsBuilder('dummy2', package_name='dummy_vc_moveit_config')` has to
find two different packages:

  * `dummy_vc_moveit_config` itself, for the SRDF / limits / kinematics /
    controllers / rviz config
  * `dummy2_urdf_description`, for the URDF. This package is NOT the same as the
    directory it lives in (`dummy_vc_ws/src/dummy2_urdf/`), which is exactly the
    kind of thing that resolves differently on a different machine.

When any of those lookups fails, the symptom is an EMPTY `robot_description`
plus downstream noise from `robot_state_publisher` ("robot_description
parameter must not be empty"), `move_group` ("Could not find parameter
robot_description") and `rviz2` ("Error document empty"). None of those say
which file was missing, and the launch banner that is supposed to catch it can
itself be misleading.

This script answers the only question that matters: **for every file the builder
loads, what absolute path does it resolve to, and does that path exist?**

It needs ROS (for `get_package_share_directory`) but no hardware.

    python3 tools/check_moveit_vm_paths.py

Every line starts with [ok] or [!!]; the summary at the end says whether the
configuration can possibly load.
"""
import os
import sys


# ---------------------------------------------------------------- static part

# name -> (package that owns it, path inside that package)
# `None` as the package means "inside dummy_vc_moveit_config itself".
CONFIG_FILES = [
    ('.setup_assistant',              None,                         'config/.setup_assistant'),
    ('SRDF',                          None,                         'config/dummy2.srdf'),
    ('joint limits',                  None,                         'config/joint_limits.yaml'),
    ('kinematics',                    None,                         'config/kinematics.yaml'),
    ('controllers',                   None,                         'config/moveit_controllers.yaml'),
    ('ompl planning',                 None,                         'config/ompl_planning.yaml'),
    ('pilz cartesian limits',         None,                         'config/pilz_cartesian_limits.yaml'),
    ('planning scene monitor',        None,                         'config/planning_scene_monitor_parameters.yaml'),
    ('rviz',                          None,                         'config/moveit.rviz'),
]

MOVEIT_PACKAGE = 'dummy_vc_moveit_config'
URDF_PACKAGE = 'dummy2_urdf_description'
EXPECTED_GROUPS = ('dummy_arm', 'gripper')


def _ok(msg):
    print('  [ok] ' + msg)


def _bad(msg):
    print('  [!!] ' + msg)


class _NeverRaised(Exception):
    """Stand-in when PackageNotFoundError cannot be imported.

    `ament_index_python.packages` has exported `PackageNotFoundError` for a long
    time, but importing it in the same statement as `get_package_share_directory`
    means that if it ever moves, the script reports "you did not source ROS" --
    a wrong diagnosis for a working ROS install. Import the function first
    (that one is guaranteed), then the exception class separately.
    """


def _try_import():
    try:
        from ament_index_python.packages import get_package_share_directory
        try:
            from ament_index_python.packages import PackageNotFoundError
        except ImportError:
            PackageNotFoundError = _NeverRaised
        return get_package_share_directory, PackageNotFoundError
    except ImportError as exc:
        print('=' * 70)
        print('这台机器上没有 ROS 的 Python 环境')
        print('=' * 70)
        print(f'  import ament_index_python 失败: {exc}')
        print()
        print('  先 source 一下再跑：')
        print('      source /opt/ros/humble/setup.bash')
        print('      source install/setup.bash')
        print()
        print('  如果 source 过了还这样，是 ament_index_python 没装：')
        print('      sudo apt install -y python3-ament-index-python')
        return None, None


def _share(get_share, PackageNotFoundError, package):
    """Return the share dir of `package`, or None with a printed reason."""
    try:
        return get_share(package)
    except PackageNotFoundError:
        _bad(f'找不到包 {package}！')
        return None
    except Exception as exc:                                  # noqa: BLE001
        _bad(f'查 {package} 的 share 目录时出错: {exc!r}')
        return None


def check_packages(get_share, PackageNotFoundError):
    print('=' * 70)
    print('1. 两个包的 share 目录')
    print('=' * 70)
    found = {}
    for package in (MOVEIT_PACKAGE, URDF_PACKAGE):
        share = _share(get_share, PackageNotFoundError, package)
        if share:
            _ok(f'{package:<24} -> {share}')
            found[package] = share
        else:
            print(f'       {package} 的目录本该是 install/{package}/share/{package}/')
            print('       没有它说明这个包没构建，或者 build/install 被清掉过。')
    return found


def check_files(get_share, PackageNotFoundError, found):
    print()
    print('=' * 70)
    print('2. builder 要加载的每个文件')
    print('=' * 70)
    missing = []
    for label, package, rel in CONFIG_FILES:
        owner = package or MOVEIT_PACKAGE
        share = found.get(owner)
        if not share:
            _bad(f'{label:<24} 无法判断（{owner} 的 share 目录没找到）')
            missing.append(label)
            continue
        path = os.path.join(share, rel)
        if os.path.isfile(path):
            size = os.path.getsize(path)
            _ok(f'{label:<24} {path}  ({size} 字节)')
        else:
            _bad(f'{label:<24} 不存在: {path}')
            missing.append(label)
    return missing


def read_setup_assistant(share):
    """Read the URDF/SRDF locations out of .setup_assistant without PyYAML."""
    path = os.path.join(share, 'config', '.setup_assistant')
    if not os.path.isfile(path):
        return None, None, None
    urdf_pkg = urdf_rel = srdf_rel = None
    section = None
    with open(path, encoding='utf-8') as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            if stripped.startswith('urdf:'):
                section = 'urdf'
                continue
            if stripped.startswith('srdf:'):
                section = 'srdf'
                continue
            if stripped.startswith('configuration:'):
                section = 'configuration'
                continue
            if ':' not in stripped:
                continue
            key, _, value = stripped.partition(':')
            key, value = key.strip(), value.strip()
            if section == 'urdf' and key == 'package':
                urdf_pkg = value
            elif section == 'urdf' and key == 'relative_path':
                urdf_rel = value
            elif section == 'srdf' and key == 'relative_path':
                srdf_rel = value
    return urdf_pkg, urdf_rel, srdf_rel


def check_urdf(found):
    print()
    print('=' * 70)
    print('3. URDF 到底在哪（robot_description 为空的真正原因）')
    print('=' * 70)
    moveit_share = found.get(MOVEIT_PACKAGE)
    if not moveit_share:
        _bad('dummy_vc_moveit_config 的 share 目录没找到，跳过')
        return False

    urdf_pkg, urdf_rel, srdf_rel = read_setup_assistant(moveit_share)
    if urdf_pkg is None and urdf_rel is None:
        _bad('.setup_assistant 读不出内容（文件缺失或格式不对）')
        print('       builder 找不到它就会退回 config/dummy2.urdf —— 那个文件不存在，')
        print('       于是 robot_description 为空。')
        return False

    _ok(f'.setup_assistant 声明: urdf 包={urdf_pkg!r} 路径={urdf_rel!r} srdf={srdf_rel!r}')

    if urdf_pkg != URDF_PACKAGE:
        _bad(f'.setup_assistant 里写的包是 {urdf_pkg!r}，但仓库里的包叫 {URDF_PACKAGE!r}')
        print('       两者必须一致，否则 builder 去一个不存在的包里找 URDF。')
        return False

    share = found.get(URDF_PACKAGE)
    if not share:
        _bad(f'{URDF_PACKAGE} 没构建出来 —— 这就是 robot_description 为空的原因')
        return False

    path = os.path.join(share, urdf_rel)
    if not os.path.isfile(path):
        _bad(f'URDF 文件不存在: {path}')
        print(f'       看看这个目录里到底有什么: ls -la {os.path.join(share, "urdf")}')
        return False
    size = os.path.getsize(path)
    _ok(f'URDF 文件在: {path}  ({size} 字节)')

    # The xacro must contain the links the SRDF and MoveIt groups refer to.
    with open(path, encoding='utf-8') as handle:
        text = handle.read()
    for token in ('base_link', 'gripper_tcp', 'dummy2_joint1', 'dummy2_finger_joint1'):
        if token in text:
            _ok(f'URDF 里含 {token}')
        else:
            _bad(f'URDF 里没有 {token} —— SRDF 会引用到不存在的名字')
    return True


def check_python_read(get_share, PackageNotFoundError, found):
    """Load the two packages the way the builder does, and show the shapes."""
    print()
    print('=' * 70)
    print('4. 用 Python 真读一遍（这一节最接近 launch 里的实际行为）')
    print('=' * 70)
    import yaml

    try:
        import xacro
    except ImportError:
        xacro = None
        _bad('没有 python3-xacro，无法展开 xacro（URDF 是 xacro 文件）')
        print('       sudo apt install -y ros-humble-xacro')

    moveit_share = found.get(MOVEIT_PACKAGE)
    urdf_share = found.get(URDF_PACKAGE)
    if not moveit_share or not urdf_share:
        _bad('包目录不全，跳过')
        return

    _urdf_pkg, urdf_rel, _srdf_rel = read_setup_assistant(moveit_share)
    urdf_path = os.path.join(urdf_share, urdf_rel or 'urdf/dummy2_urdf.xacro')

    raw = None
    if xacro is not None and os.path.isfile(urdf_path):
        try:
            raw = xacro.process_file(urdf_path).toxml()
            _ok(f'xacro 展开成功，得到 {len(raw)} 字符的 URDF')
        except Exception as exc:                              # noqa: BLE001
            _bad(f'xacro 展开失败: {exc!r}')
            print('       这一条会让 robot_description 为空。上面那句 "Error document empty"')
            print('       通常就是这么来的。')
    if raw:
        for token in ('<robot', 'base_link', 'gripper_tcp'):
            if token in raw:
                _ok(f'展开后的 URDF 里含 {token}')
            else:
                _bad(f'展开后的 URDF 里没有 {token}')

    # The kinematics file must become {'dummy_arm': {...}, 'gripper': {...}}.
    # If it shows up as a single key named after the FILE, the loader wrapped it.
    kin_path = os.path.join(moveit_share, 'config', 'kinematics.yaml')
    if os.path.isfile(kin_path):
        with open(kin_path, encoding='utf-8') as handle:
            kin = yaml.safe_load(handle)
        keys = list(kin.keys()) if isinstance(kin, dict) else kin
        if isinstance(kin, dict) and set(EXPECTED_GROUPS).issubset(kin.keys()):
            _ok(f'kinematics.yaml 顶层键: {keys}  <- 正确')
        else:
            _bad(f'kinematics.yaml 顶层键: {keys}')
            print(f'       期望顶层键正好是 {list(EXPECTED_GROUPS)}。')
    _check_simple_yaml(moveit_share, 'config/joint_limits.yaml', 'joint limits')
    _check_simple_yaml(moveit_share, 'config/ompl_planning.yaml', 'ompl')


def _check_simple_yaml(share, rel, label):
    import yaml
    path = os.path.join(share, rel)
    if not os.path.isfile(path):
        return
    try:
        with open(path, encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except Exception as exc:                                  # noqa: BLE001
        _bad(f'{label}: YAML 解析失败 {exc!r}')
        return
    keys = list(data.keys()) if isinstance(data, dict) else type(data).__name__
    _ok(f'{label}: 顶层键 {keys}')


def main():
    get_share, PackageNotFoundError = _try_import()
    if get_share is None:
        return 3

    print('=' * 70)
    print('MoveIt 配置路径自检（在虚拟机上跑）')
    print('=' * 70)
    print(f'  ROS_DISTRO       = {os.environ.get("ROS_DISTRO", "(未设置)")}')
    print(f'  AMENT_PREFIX_PATH 有 {len(os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep))} 项')
    print()

    found = check_packages(get_share, PackageNotFoundError)
    missing = check_files(get_share, PackageNotFoundError, found)
    urdf_ok = check_urdf(found)
    try:
        check_python_read(get_share, PackageNotFoundError, found)
    except ImportError as exc:
        _bad(f'缺 Python 模块: {exc}')

    print()
    print('=' * 70)
    if missing:
        print('结论: 有配置文件缺失，launch 不可能加载出正确的 robot_description')
        print('  缺的: ' + ', '.join(missing))
        print('  处理: 拉最新代码后清掉这个包的 build/install 再构建')
        print('        rm -rf install/dummy_vc_moveit_config build/dummy_vc_moveit_config')
        print('        colcon build --symlink-install --packages-select '
              'dummy2_urdf dummy_vc_moveit_config')
        return 1
    if not urdf_ok:
        print('结论: 配置文件都在，但 URDF 找不到或读不出来 —— robot_description 会是空的')
        return 1
    print('结论: 所有路径都能解析。如果 launch 里 robot_description 仍为空，')
    print('      把第 4 节的输出发回来 —— 说明问题在 xacro 展开或参数形状，不在路径。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
