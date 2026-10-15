#!/usr/bin/env python3
"""On the VM: ask MoveItConfigsBuilder what it ACTUALLY returned.

Why this exists
---------------
`check_moveit_vm_paths.py` proves the files are all present, that the URDF is
where `.setup_assistant` says it is, and that `xacro` can expand it to a
non-empty document. It stops there on purpose -- it re-implements the lookup,
so it cannot see what the *real* builder produces.

This script runs the real thing and dumps the answer:

    MoveItConfigsBuilder('dummy2', package_name='dummy_vc_moveit_config')
        .robot_description_semantic(file_path='config/dummy2.srdf')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')

-- byte for byte the same chain `launch/demo.launch.py` uses.

It prints the RAW value of each attribute (its Python type and repr), then
judges the three things the launch banner prints as counts:

  * robot_description           must be a dict with a NON-EMPTY
                                `robot_description` string inside it
  * robot_description_semantic  must be a non-empty string / dict
  * robot_description_kinematics must be a dict keyed by GROUP NAME
                                (`dummy_arm`, `gripper`) -- a single key equal to
                                the attribute name means the loader wrapped the
                                file instead of reading it into parameters

It also prints the MoveIt version, because file resolution changed across
releases and a version mismatch is the usual reason this works on one machine
and not another.

    python3 tools/check_moveit_builder.py

Exit codes: 0 all three judgements pass, 1 something the launch cannot survive,
3 the script could not run (no ROS / no moveit_configs_utils).
"""
import os
import sys


# The exact chain from launch/demo.launch.py. Keep the two in step.
def build():
    from moveit_configs_utils import MoveItConfigsBuilder
    return (MoveItConfigsBuilder('dummy2',
                                 package_name='dummy_vc_moveit_config')
            .robot_description_semantic(file_path='config/dummy2.srdf')
            .joint_limits(file_path='config/joint_limits.yaml')
            .trajectory_execution(file_path='config/moveit_controllers.yaml'))


# Paths this script hard-codes above. A copy that drifts from the launch file
# would make every later verdict describe a chain the launch does not use.
CHAIN_PATHS = ('config/dummy2.srdf',
               'config/joint_limits.yaml',
               'config/moveit_controllers.yaml')
LAUNCH_REL = os.path.join('dummy_vc_ws', 'src', 'dummy_vc_moveit_config',
                          'launch', 'demo.launch.py')


def check_chain_matches_launch():
    """Compare the hard-coded chain above against the real launch file.

    Runs through the AST, so it needs neither ROS nor a working
    moveit_configs_utils -- it still works on the machine where the launch is
    broken, which is exactly when this script gets used.
    """
    import ast

    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(os.path.dirname(here), LAUNCH_REL)
    if not os.path.isfile(path):
        print(f'  [--] 找不到 {path}，跳过这项（不影响下面的判定）')
        return None
    try:
        with open(path, encoding='utf-8') as handle:
            tree = ast.parse(handle.read())
    except (OSError, SyntaxError) as exc:
        print(f'  [--] 读不了 launch 文件: {exc!r}，跳过这项')
        return None

    text = open(path, encoding='utf-8').read()
    found = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not (isinstance(func, ast.Attribute) and func.attr in
                ('robot_description_semantic', 'joint_limits',
                 'trajectory_execution')):
            continue
        for keyword in node.keywords:
            if keyword.arg == 'file_path' and isinstance(keyword.value,
                                                         ast.Constant):
                found.append((func.attr, keyword.value.value))

    if not found:
        print('  [--] launch 文件里没找到带 file_path 的 builder 调用，跳过这项')
        return None

    missing = [p for p in CHAIN_PATHS if p not in text]
    if missing:
        print(f'  [!!] launch 文件里没有这些路径 {missing} —— 两边已经不一致了！')
        return False
    print(f'  [ok] launch 里的 builder 链与本脚本一致（{len(found)} 个 file_path）')
    for name, value in found:
        print(f'      {name}(file_path={value!r})')
    return True


def _describe(value, limit=160):
    """One line describing a returned attribute: type plus a short repr."""
    kind = type(value).__name__
    try:
        text = repr(value)
    except Exception as exc:                                  # noqa: BLE001
        return f'{kind} (repr failed: {exc!r})'
    if len(text) > limit:
        text = text[:limit] + f'... (+{len(repr(value)) - limit} chars)'
    return f'{kind}: {text}'


def _dump_dict(label, value):
    print(f'  {label}:')
    print(f'      type = {type(value).__name__}')
    if isinstance(value, dict):
        if not value:
            print('      (空字典)')
        for key in sorted(value.keys()):
            item = value[key]
            if isinstance(item, str):
                print(f'      [{key}] str, {len(item)} 字符')
                if len(item) < 200:
                    print(f'          {item!r}')
                else:
                    print(f'          开头: {item[:100]!r}')
                    print(f'          结尾: {item[-60:]!r}')
            elif isinstance(item, dict):
                print(f'      [{key}] dict, 顶层键 = {sorted(item.keys())}')
            else:
                print(f'      [{key}] {_describe(item)}')
    else:
        print(f'      {_describe(value)}')


def judge_robot_description(desc):
    print()
    print('=' * 70)
    print('判定 1: robot_description')
    print('=' * 70)
    _dump_dict('robot_description', desc)
    print()
    if not isinstance(desc, dict):
        print('  [!!] 不是 dict —— launch 里 `robot_description.get(...)` 会崩或拿到空值')
        return False
    urdf = desc.get('robot_description', '')
    if not isinstance(urdf, str) or len(urdf) < 100:
        print(f'  [!!] 里面没有可用的 URDF（拿到 {len(str(urdf))} 字符）')
        print('       文件都在、xacro 也能展开，但 builder 没把 URDF 放进来。')
        print('       下一步看上面打印的键名 —— 如果键名不是 robot_description，')
        print('       而是以文件名/包名为名的键，那就是加载路径走错了。')
        return False
    print(f'  [ok] URDF 有 {len(urdf)} 字符')
    for token in ('<robot', 'base_link', 'gripper_tcp'):
        if token in urdf:
            print(f'  [ok] 含 {token}')
        else:
            print(f'  [!!] 不含 {token}')
            return False
    return True


def judge_semantic(sem):
    print()
    print('=' * 70)
    print('判定 2: robot_description_semantic')
    print('=' * 70)
    _dump_dict('robot_description_semantic', sem)
    print()
    text = sem.get('robot_description_semantic', '') if isinstance(sem, dict) else sem
    if not isinstance(text, str) or len(text) < 100:
        print('  [!!] 没有可用的 SRDF')
        return False
    if '<group name="dummy_arm"' not in text:
        print('  [!!] SRDF 里没有 dummy_arm 组')
        return False
    print(f'  [ok] SRDF 有 {len(text)} 字符，含 dummy_arm 组')
    return True


def judge_kinematics(kin, groups=('dummy_arm', 'gripper')):
    print()
    print('=' * 70)
    print('判定 3: robot_description_kinematics（看键名）')
    print('=' * 70)
    _dump_dict('robot_description_kinematics', kin)
    print()
    if not isinstance(kin, dict):
        print('  [!!] 不是 dict')
        return False
    keys = sorted(kin.keys())
    if set(groups).issubset(keys):
        print(f'  [ok] 顶层键是规划组名: {keys}')
        return True
    print(f'  [!!] 顶层键是 {keys}，期望含 {list(groups)}')
    if len(keys) == 1:
        print(f'       只有一个键 {keys[0]!r} —— 加载器把整个文件包了一层。')
        print('       这会让 move_group 找不到任何规划组的运动学解算器。')
    return False


def main():
    print('=' * 70)
    print('MoveItConfigsBuilder 真实返回值（在虚拟机上跑）')
    print('=' * 70)
    print(f'  python    = {sys.version.split()[0]}')
    print(f'  ROS_DISTRO= {os.environ.get("ROS_DISTRO", "(未设置)")}')

    try:
        import ament_index_python
        print(f'  ament_index_python = {getattr(ament_index_python, "__file__", "?")}')
    except ImportError:
        pass

    try:
        from moveit_configs_utils import MoveItConfigsBuilder  # noqa: F401
    except ImportError as exc:
        print()
        print(f'  [!!] 导入 moveit_configs_utils 失败: {exc}')
        print('       先 source ROS 和工作空间：')
        print('           source /opt/ros/humble/setup.bash')
        print('           source install/setup.bash')
        return 3

    try:
        import importlib.metadata as md
        try:
            version = md.version('moveit_configs_utils')
        except Exception:                                     # noqa: BLE001
            version = '(查不到)'
    except ImportError:
        version = '(无 importlib.metadata)'
    print(f'  moveit_configs_utils 版本 = {version}')

    # MoveIt itself, for the release-to-release comparison.
    for pkg in ('moveit_core', 'moveit_ros_planning'):
        try:
            import importlib.metadata as md
            print(f'  {pkg} 版本 = {md.version(pkg)}')
        except Exception:                                     # noqa: BLE001
            pass

    print()
    print('=' * 70)
    print('0. 走 launch 里那条一模一样的 builder 链')
    print('=' * 70)
    print('  先确认这条链真的和 launch 文件一致:')
    check_chain_matches_launch()
    print()
    try:
        builder = build()
        print('  [ok] builder 构造成功')
    except Exception as exc:                                  # noqa: BLE001
        print(f'  [!!] builder 构造失败: {type(exc).__name__}: {exc}')
        import traceback
        traceback.print_exc()
        return 1

    try:
        config = builder.to_moveit_configs()
        print(f'  [ok] to_moveit_configs() 返回 {type(config).__name__}')
    except Exception as exc:                                  # noqa: BLE001
        print(f'  [!!] to_moveit_configs() 失败: {type(exc).__name__}: {exc}')
        import traceback
        traceback.print_exc()
        return 1

    print()
    print('  这个对象有哪些属性:')
    attrs = [a for a in dir(config) if not a.startswith('_')]
    print(f'      {attrs}')
    print()
    print('  其中和 robot_description 相关的，原样打印:')
    for attr in attrs:
        if 'description' in attr or 'kinematic' in attr or 'pipeline' in attr:
            try:
                value = getattr(config, attr)
            except Exception as exc:                          # noqa: BLE001
                print(f'      {attr}: 取值失败 {exc!r}')
                continue
            if isinstance(value, dict):
                print(f'      {attr}: dict 顶层键 = {sorted(value.keys())}')
            elif isinstance(value, str):
                print(f'      {attr}: str, {len(value)} 字符')
            else:
                print(f'      {attr}: {_describe(value)}')

    ok = True
    ok &= judge_robot_description(getattr(config, 'robot_description', {}))
    ok &= judge_semantic(getattr(config, 'robot_description_semantic', {}))
    ok &= judge_kinematics(getattr(config, 'robot_description_kinematics', {}))

    print()
    print('=' * 70)
    if ok:
        print('结论: builder 三个返回值都正常。')
        print('      如果 launch 仍然报 robot_description 为空，那问题在 launch 之后 ——')
        print('      把 demo.launch.py 的完整输出发回来对比。')
        return 0
    print('结论: builder 返回的东西本身就是坏的，与 launch 无关。')
    print('      上面第一个 [!!] 就是根因；把它连同这几行版本号发回来。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
