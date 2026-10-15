#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线自测 tools/check_rviz_plugin.py：四种场景，全部在假 ROS 前缀上跑真脚本。

为什么要留着它：check_rviz_plugin.py 只能在装好 ROS 的机器上跑，而它第一次
真被执行就是在那台虚拟机上。一旦它自己出错，报出来的是"一个 panel 都没有"，
这跟要排查的故障长得一模一样 —— 最该信的那条线索反而把人带偏。这个自测用假
前缀把这四种情况在本地跑一遍：

  case 1  索引标记 + XML + 真 .so 都在（有 gcc 才真编译，没有就跳过 dlopen 断言）
  case 2  只有 XML、没有 ament 索引标记 —— 就是 "Declared types are <EMPTY>" 的
          真实成因：pluginlib 只读索引，不读 share 目录里躺着什么文件
  case 3  索引标记指向一个不存在的文件（装包装了一半）
  case 4  XML 根本没装

它抓到过的真 bug：插件的 XML 根元素就是 <library>（moveit_ros_visualization 和
rviz_default_plugins 都是这个形状），而脚本原来写的是 root.findall('library')，
在根元素底下找不到 <library> 子节点，于是"声明的类"永远是空的。那个 bug 在
虚拟机上会伪装成"MoveIt 的 panel 一个都没装"。

假前缀放在系统临时目录里（不是 tools/ 下面），所以中途被打断也不会往仓库里
留 .tmp_* 垃圾。
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, 'check_rviz_plugin.py')

XML_BODY = '''<library path="libmoveit_motion_planning_rviz_plugin.so">
  <class name="moveit_rviz_plugin/MotionPlanning"
         type="moveit_rviz_plugin::MotionPlanningDisplay"
         base_class_type="rviz_common::Panel">
    <description>MoveIt Motion Planning Panel</description>
  </class>
  <class name="moveit_rviz_plugin/PlanningScene"
         type="moveit_rviz_plugin::PlanningSceneDisplay"
         base_class_type="rviz_common::Display">
    <description>MoveIt Planning Scene</description>
  </class>
</library>
'''

# NOTE: tempfile.mkdtemp() is deliberately not used here. Under the DSH Windows
# sandbox a directory created by mkdtemp() rejects the very next os.mkdir()
# inside it (WinError 5, access denied), while a directory made by os.makedirs()
# accepts it. Build the fixture root by hand instead.
ROOT = os.path.join(tempfile.gettempdir(),
                    'dummy_vc_rviz_selftest_{}'.format(os.getpid()))
shutil.rmtree(ROOT, ignore_errors=True)
os.makedirs(ROOT)
FAKE = os.path.join(ROOT, 'prefix')
SRC = os.path.join(ROOT, 'plugin_marker.c')


def make_lib(target):
    """Compile a tiny real shared library so the dlopen step is exercised."""
    gcc = shutil.which('gcc') or shutil.which('cc')
    if not gcc:
        return False
    with open(SRC, 'w') as fh:
        fh.write('int dummy_plugin_marker(void) { return 42; }\n')
    p = subprocess.run([gcc, '-shared', '-o', target, SRC],
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       text=True, encoding='utf-8', errors='replace')
    return p.returncode == 0 and os.path.isfile(target)


def build_prefix(with_xml=True, with_marker=True, with_lib=True,
                 marker_body='share/moveit_ros_visualization/moveit_rviz_plugin_description.xml'):
    if os.path.isdir(FAKE):
        shutil.rmtree(FAKE)
    share = os.path.join(FAKE, 'share', 'moveit_ros_visualization')
    os.makedirs(share)
    if with_xml:
        with open(os.path.join(share, 'moveit_rviz_plugin_description.xml'), 'w') as fh:
            fh.write(XML_BODY)
    libdir = os.path.join(FAKE, 'lib')
    os.makedirs(libdir)
    made = make_lib(os.path.join(libdir, 'libmoveit_motion_planning_rviz_plugin.so')) \
        if with_lib else False
    if with_marker:
        idx = os.path.join(FAKE, 'share', 'ament_index', 'resource_index',
                           'rviz_common__pluginlib__plugin')
        os.makedirs(idx)
        with open(os.path.join(idx, 'moveit_ros_visualization'), 'w') as fh:
            fh.write(marker_body)
    # a minimal rviz_common so section 4 has something to count
    rv = os.path.join(FAKE, 'share', 'rviz_common')
    os.makedirs(rv)
    with open(os.path.join(rv, 'plugins_description.xml'), 'w') as fh:
        fh.write('<library path="librviz_common.so">\n'
                 '  <class name="rviz_common/Displays"\n'
                 '         type="rviz_common::Displays"\n'
                 '         base_class_type="rviz_common::Panel"/>\n'
                 '</library>\n')
    return made


def run_script(prefix):
    """Run the real diagnostic against the fake prefix and capture its report."""
    env = dict(os.environ)
    env['ROS_DISTRO'] = 'humble'
    env['AMENT_PREFIX_PATH'] = prefix
    env['PATH'] = '/usr/bin:/bin'
    p = subprocess.run([sys.executable, SCRIPT], stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, env=env, text=True,
                       encoding='utf-8', errors='replace', timeout=180)
    return p.returncode, p.stdout or ''


fails = []


def check(label, cond, extra=''):
    print('  {} {}'.format('[ok  ]' if cond else '[FAIL]', label))
    if not cond:
        fails.append(label)
        if extra:
            print('      --- tail of output ---')
            print(extra)


print('假 ROS 前缀: {}'.format(FAKE))
print('')
print('=== case 1: XML + marker + real .so all present ===')
made_lib = build_prefix()
print('  (compiled a real shared library: {})'.format(made_lib))
rc, out = run_script(FAKE)
check('reached section 2b', '2b. what pluginlib itself sees' in out, out[-2000:])
check('marker detected', '[ok] marker present:' in out)
check('index -> XML resolved',
      'moveit_rviz_plugin_description.xml' in out, out[-2000:])
check('index and on-disk XML agree',
      '[ok] the ament index and the on-disk XML agree' in out, out[-2000:])
check('no "empty list" complaint',
      'pluginlib therefore finds ZERO panel' not in out, out[-2000:])
check('MotionPlanning found in the XML',
      'this is the one RViz is failing on' in out, out[-2000:])
check('base class matched', '[ok] base class matches what RViz asks for' in out,
      out[-2000:])
if made_lib:
    check('dlopen succeeded', 'dlopen succeeded' in out, out[-2000:])

print('')
print('=== case 2: XML present, marker MISSING (the failure mode) ===')
build_prefix(with_marker=False)
rc, out = run_script(FAKE)
check('reached section 2b', '2b. what pluginlib itself sees' in out, out[-2000:])
check('told the user the index marker is absent',
      'no ament plugin-index marker' in out, out[-2000:])
check('explains the empty Declared types', 'Declared types are' in out,
      out[-2000:])

print('')
print('=== case 3: marker present but points at a missing file ===')
build_prefix(marker_body='share/moveit_ros_visualization/does_not_exist.xml')
rc, out = run_script(FAKE)
check('detects the dangling index entry', 'but that file is not there' in out,
      out[-2000:])
check('flags index/files disagree',
      'the index and the installed files disagree' in out, out[-2000:])

print('')
print('=== case 4: XML missing entirely ===')
build_prefix(with_xml=False)
rc, out = run_script(FAKE)
check('flags the missing XML',
      'the plugin description XML is NOT installed' in out, out[-2000:])

shutil.rmtree(ROOT, ignore_errors=True)
print('')
print('临时前缀已清理: {}'.format(not os.path.exists(ROOT)))
if fails:
    print('RESULT: {} FAILURE(S) -- check_rviz_plugin.py would mislead on the VM'
          .format(len(fails)))
    for f in fails:
        print('  - {}'.format(f))
else:
    print('RESULT: the RViz plugin diagnostic reports all four cases correctly')
sys.exit(1 if fails else 0)
