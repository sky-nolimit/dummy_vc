#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz still cannot load moveit_rviz_plugin/MotionPlanning? Run THIS.

Why this exists
---------------
The error says: "the class moveit_rviz_plugin/MotionPlanning with base class
type rviz_common::Panel does not exist. Declared types are <EMPTY>".

That "Declared types are <EMPTY>" is the interesting part: RViz found **zero**
panel plugins at all -- not just the MoveIt one. So the question is not "is
ros-humble-moveit-ros-visualization installed" (you checked, it is). The
question is "where does pluginlib look, and what does it find there".

This script prints that chain, step by step, with a verdict on each step.
It does NOT need RViz to be running, and it changes nothing on the system.

Usage
-----
    python3 tools/check_rviz_plugin.py

Exit codes: 0 = the chain looks complete, 1 = a step is broken, 3 = no ROS.
"""
import os
import shutil
import subprocess
import sys
import glob


def head(title):
    print('')
    print('=' * 72)
    print(title)
    print('=' * 72)


def run(cmd, timeout=20):
    """Run a command, return (returncode, stdout+stderr). Never raises."""
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           timeout=timeout, text=True, encoding='utf-8',
                           errors='replace')
        return p.returncode, p.stdout or ''
    except Exception as exc:
        return 127, 'could not run {}: {}'.format(cmd, exc)


def glob_posix(pattern):
    """glob.glob, but with a pattern that survives Windows backslashes.

    glob treats '\\' as an escape character rather than a separator, so a
    pattern built with os.path.join matches nothing on Windows and the caller
    silently sees "no files found". Always hand glob a forward-slash pattern.
    """
    return glob.glob(str(pattern).replace(os.sep, '/'))


def split_paths(value):
    """Split a ROS path-list env var the way ROS itself does.

    ROS uses ':' on Linux, which is os.pathsep there -- so this is the same
    thing on the target platform. Do NOT hardcode ':': on Windows an absolute
    path starts with a drive letter, so splitting on ':' turns
    'C:\\ros' into ['C', '\\ros'] and every later check silently runs against
    a bogus prefix (this exact bug made the offline harness fail).
    """
    return [p for p in (value or '').split(os.pathsep) if p]


problems = []


def bad(msg):
    problems.append(msg)
    print('  [!!] {}'.format(msg))


# ---------------------------------------------------------------------------
head('0. environment')
# ---------------------------------------------------------------------------
distro = os.environ.get('ROS_DISTRO', '')
print('  ROS_DISTRO      = {}'.format(distro or '(not set!)'))
print('  ROS_VERSION     = {}'.format(os.environ.get('ROS_VERSION', '(not set)')))
print('  LD_LIBRARY_PATH =')
for part in split_paths(os.environ.get('LD_LIBRARY_PATH')):
    if part:
        print('      {}'.format(part))
print('  AMENT_PREFIX_PATH =')
for part in split_paths(os.environ.get('AMENT_PREFIX_PATH')):
    if part:
        print('      {}'.format(part))
print('  rviz2           = {}'.format(shutil.which('rviz2') or '(not on PATH!)'))
if not distro:
    print('')
    print('  ROS is not sourced. Run: source /opt/ros/humble/setup.bash')
    sys.exit(3)
if not shutil.which('ros2'):
    bad('ros2 is not on PATH')

# ---------------------------------------------------------------------------
head('1. does the package show up in the ament index?')
# ---------------------------------------------------------------------------
prefixes = split_paths(os.environ.get('AMENT_PREFIX_PATH'))
share = None
for prefix in prefixes + ['/opt/ros/{}'.format(distro)]:
    cand = os.path.join(prefix, 'share', 'moveit_ros_visualization')
    if os.path.isdir(cand):
        share = cand
        print('  [ok] share dir: {}'.format(cand))
        break
if share is None:
    bad('no share/moveit_ros_visualization under any AMENT_PREFIX_PATH entry '
        '-- the apt package may be missing after all')
    print('       checked: {}'.format(
        [os.path.join(p, 'share', 'moveit_ros_visualization') for p in prefixes]))

# ---------------------------------------------------------------------------
head('2. the plugin description XML(s) and what they declare')
# ---------------------------------------------------------------------------
xmls = []
for prefix in prefixes + ['/opt/ros/{}'.format(distro)]:
    hit = sorted(glob_posix('{}/share/moveit_ros_visualization/*.xml'.format(prefix)))
    for f in hit:
        if f not in xmls:
            xmls.append(f)
print('  found {} xml file(s)'.format(len(xmls)))
for f in xmls:
    print('    {}'.format(f))
if not xmls:
    bad('the plugin description XML is NOT installed -- pluginlib has nothing '
        'to read, which alone explains "Declared types are <EMPTY>"')

declared = []       # (class_name, base_class, library, xml_path)
for path in xmls:
    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(path).getroot()
    except Exception as exc:
        bad('cannot parse {}: {}'.format(path, exc))
        continue
    # pluginlib XMLs come in two shapes: `<library>` AS the root element (that
    # is how moveit_ros_visualization and rviz_default_plugins ship them), or
    # several `<library>` children under a wrapper element. `findall('library')`
    # finds nothing in the first shape -- and the first shape is exactly the
    # one that matters here. Silently declaring zero classes is the worst
    # possible answer from this script, so handle both.
    libs = [root] if root.tag == 'library' else root.findall('library')
    if not libs:
        print('    ({}: no <library> element at all -- not a pluginlib XML)'
              .format(path))
    for lib in libs:
        libpath = lib.get('path', '')
        for cls in lib.findall('class'):
            name = cls.get('name', '')
            base = cls.get('base_class_type', '')
            declared.append((name, base, libpath, path))
            print('    class {!r}'.format(name))
            print('        base_class_type = {!r}'.format(base))
            print('        library         = {!r}'.format(libpath))
            if name == 'moveit_rviz_plugin/MotionPlanning':
                print('        ^^^ this is the one RViz is failing on')

if declared:
    motions = [d for d in declared if d[0] == 'moveit_rviz_plugin/MotionPlanning']
    if not motions:
        bad('the XML does not declare moveit_rviz_plugin/MotionPlanning at all')
    else:
        base = motions[0][1]
        print('')
        print('  MotionPlanning declares base_class_type = {!r}'.format(base))
        print('  RViz looks for base class rviz_common::Panel')
        if base != 'rviz_common::Panel':
            bad('base class mismatch: XML says {!r}, RViz asks for '
                '"rviz_common::Panel" -- versions of rviz_common and '
                'moveit_ros_visualization do not match'.format(base))
        else:
            print('  [ok] base class matches what RViz asks for')

# ---------------------------------------------------------------------------
head('2b. what pluginlib itself sees (the ament plugin index)')
# ---------------------------------------------------------------------------
# This is the step that was missing the first time round, and it is the one
# that matters. "Declared types are <EMPTY>" does not come from the files on
# disk, it comes from the list pluginlib built by reading the ament index.
# An XML can sit there perfectly installed and still be invisible, because
# ament_index_python.get_resource() reads a marker under the *prefix*:
#
#   <prefix>/share/ament_index/resource_index/rviz_common__pluginlib__plugin/<pkg>
#
# and the content of that marker says which XML to load. Two things break it:
#   - the marker is not installed / not indexed,
#   - AMENT_PREFIX_PATH that pluginlib is using does not contain the prefix.
#
# So this section compares those two views and reports when they disagree.
index_marker_rel = os.path.join(
    'share', 'ament_index', 'resource_index',
    'rviz_common__pluginlib__plugin', 'moveit_ros_visualization')

index_view = []          # (prefix, marker path, parsed xml paths)
for prefix in prefixes + ['/opt/ros/{}'.format(distro)]:
    marker = os.path.join(prefix, index_marker_rel)
    if not os.path.isfile(marker):
        continue
    print('  [ok] marker present: {}'.format(marker))
    try:
        with open(marker, 'r') as fh:
            body = fh.read()
    except Exception as exc:
        bad('cannot read the index marker {}: {}'.format(marker, exc))
        continue
    for rel in body.split():
        # normalise instead of plain join: the marker content is a POSIX path
        # (share/.../x.xml) and os.path.join would happily produce a Windows
        # path with backslashes, which then silently fails to match anything.
        full = os.path.normpath(os.path.join(prefix, rel.replace('/', os.sep)))
        index_view.append(full)
        exists = os.path.isfile(full)
        print('       -> {}{}'.format(full, '' if exists else '   <-- MISSING'))
        if not exists:
            bad('the ament index points at {} but that file is not there '
                '-- the index and the installed files disagree'.format(full))

if not index_view:
    bad('no ament plugin-index marker for moveit_ros_visualization in any '
        'prefix on AMENT_PREFIX_PATH -- pluginlib therefore finds ZERO panel '
        'plugins and prints "Declared types are" with an empty list. The XML '
        'being on disk (section 2) is NOT enough; it has to be indexed.')
else:
    # Compare normalised paths: section 2's glob() may hand back forward
    # slashes while this section builds native ones, and a plain string
    # comparison would then report a mismatch between two identical files.
    index_set = {os.path.normpath(p) for p in index_view}
    if xmls and not any(os.path.normpath(f) in index_set for f in xmls):
        bad('the XML found by globbing the share dir is NOT among the files '
            'the ament index hands to pluginlib -- they are different files. '
            'Compare the two lists above.')
    else:
        print('  [ok] the ament index and the on-disk XML agree')

# The same index also feeds rviz_common's own panels. When "Declared types" is
# empty the question is whether it is empty for MoveIt alone (one package's
# install is broken) or for every package (the index itself is not being read)
# -- those are different bugs with different fixes. Ask it directly rather than
# inferring it from section 4. Print only: section 4 owns the verdict.
print('')
print('  index markers for the packages that feed this index:')
for pkg in ('rviz_common', 'rviz_default_plugins', 'moveit_ros_visualization'):
    rel = os.path.join('share', 'ament_index', 'resource_index',
                       'rviz_common__pluginlib__plugin', pkg)
    hits = [p for p in prefixes + ['/opt/ros/{}'.format(distro)]
            if os.path.isfile(os.path.join(p, rel))]
    if hits:
        print('    [ok] {}: {}'.format(pkg, hits[0]))
    else:
        print('    [!!] {}: no marker under any prefix'.format(pkg))

# Also ask the API directly, when it is importable, because that is the exact
# call pluginlib makes. A disagreement with the walk above means the prefix
# list the process uses is not the one printed in section 0.
try:
    from ament_index_python.packages import get_package_share_directory
    from ament_index_python.resources import get_resource

    share = get_package_share_directory('moveit_ros_visualization')
    print('  get_package_share_directory() -> {}'.format(share))
    try:
        _ctype, content = get_resource('rviz_common__pluginlib__plugin',
                                       'moveit_ros_visualization')
        print('  get_resource() -> {!r}'.format(content))
    except Exception as exc:
        bad('ament_index_python.get_resource() raised {} -- this is the '
            'ground truth for what pluginlib sees'.format(exc))
except ImportError:
    print('  (ament_index_python not importable; skipped the API check)')
except Exception as exc:
    bad('get_package_share_directory(moveit_ros_visualization) raised {} '
        '-- the running environment cannot find the package even though apt '
        'says it is installed'.format(exc))

# ---------------------------------------------------------------------------
head('3. does the plugin .so exist, and can it be loaded?')
# ---------------------------------------------------------------------------
import ctypes
libs = sorted({d[2] for d in declared if d[2]})
for libname in libs:
    found = None
    for prefix in prefixes + ['/opt/ros/{}'.format(distro)]:
        cand = os.path.join(prefix, 'lib', libname)
        if os.path.isfile(cand):
            found = cand
            break
    if found is None:
        cand2 = glob.glob('/opt/ros/{}/lib/**/{}'.format(distro, libname),
                          recursive=True)
        found = cand2[0] if cand2 else None
    if found is None:
        bad('library {!r} is NOT on disk -- the XML points at a .so that is '
            'not installed'.format(libname))
        continue
    print('  [ok] {}  ({} bytes)'.format(found, os.path.getsize(found)))
    try:
        ctypes.CDLL(found)
        print('       [ok] dlopen succeeded (all link deps resolve)')
    except OSError as exc:
        bad('dlopen({}) failed: {}'.format(found, exc))
        print('       ^ that is the usual cause: a dependency .so is missing '
              'or a symbol is unresolved')
        rc, out = run(['ldd', found])
        if rc == 0:
            missing = [ln.strip() for ln in out.splitlines()
                       if 'not found' in ln]
            for m in missing[:10]:
                print('       ldd: {}'.format(m))

# ---------------------------------------------------------------------------
head('4. does rviz_common itself declare any panel plugins?')
# ---------------------------------------------------------------------------
rviz_xmls = []
for prefix in prefixes + ['/opt/ros/{}'.format(distro)]:
    rviz_xmls += glob_posix(os.path.join(prefix, 'share', 'rviz_common',
                                         '*.xml'))
    rviz_xmls += glob_posix(os.path.join(prefix, 'share', 'rviz_default_plugins',
                                         '*.xml'))
rviz_xmls = sorted(set(rviz_xmls))
print('  {} xml file(s) under rviz_common / rviz_default_plugins'.format(
    len(rviz_xmls)))
panel_count = 0
for path in rviz_xmls:
    try:
        import xml.etree.ElementTree as ET
        root = ET.parse(path).getroot()
    except Exception:
        continue
    for cls in root.iter('class'):
        base = cls.get('base_class_type', '')
        if base == 'rviz_common::Panel':
            panel_count += 1
            print('    {!r}  ({})'.format(cls.get('name', ''), base))
if panel_count == 0:
    bad('rviz_common declares NO panel plugins either -- so the plugin index '
        'itself is not being found. That is an environment/index problem, not '
        'a MoveIt problem')
else:
    print('  [ok] rviz_common declares {} panel plugin(s)'.format(panel_count))

# ---------------------------------------------------------------------------
head('5. is anything restricting library loading?')
# ---------------------------------------------------------------------------
preload = os.environ.get('LD_PRELOAD', '')
if preload:
    bad('LD_PRELOAD is set to {!r} -- this can break dlopen'.format(preload))
else:
    print('  [ok] LD_PRELOAD is not set')
print('  RViz version:')
rc, out = run(['ros2', 'pkg', 'executables', 'rviz2'])
if rc == 0:
    for ln in out.splitlines()[:5]:
        if ln.strip():
            print('      {}'.format(ln.strip()))
else:
    print('      (could not query rviz2 executables)')

# ---------------------------------------------------------------------------
head('6. what RViz is actually running with')
# ---------------------------------------------------------------------------
rc, out = run(['pgrep', '-af', 'rviz'])
if rc == 0 and out.strip():
    lines = [ln for ln in out.splitlines() if ln.strip()]
    for ln in lines:
        print('  {}'.format(ln[:160]))
    print('')
    print('  environment of the running rviz2 (this is what matters -- if it')
    print('  does not contain the prefix that has the plugin, that is the bug):')
    for ln in lines[:3]:
        pid = ln.split()[0]
        try:
            with open('/proc/{}/environ'.format(pid), 'rb') as fh:
                raw = fh.read().decode('utf-8', 'replace')
            for kv in raw.split('\0'):
                if kv.startswith(('AMENT_PREFIX_PATH=', 'LD_LIBRARY_PATH=',
                                  'ROS_DISTRO=')):
                    print('    [{}] {}'.format(pid, kv[:150]))
        except Exception as exc:
            print('    [{}] cannot read environ: {}'.format(pid, exc))
else:
    print('  rviz2 is not running right now.')

# ---------------------------------------------------------------------------
head('VERDICT')
# ---------------------------------------------------------------------------
if problems:
    print('  {} problem(s):'.format(len(problems)))
    for p in problems:
        print('    - {}'.format(p))
    print('')
    print('  Send this whole output back. The step that failed tells us which')
    print('  of these it is: package not installed / .so missing / .so cannot')
    print('  dlopen / plugin index not found / version mismatch.')
    sys.exit(1)
print('  Every step looks fine. In that case the plugin IS loadable, and the')
print('  RViz error is coming from a running rviz2 that started before the')
print('  package was installed -- fully quit RViz (close the window, make sure')
print('  no rviz2 process is left) and launch again.')
sys.exit(0)
