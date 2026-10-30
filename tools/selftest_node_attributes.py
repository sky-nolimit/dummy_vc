#!/usr/bin/env python3
"""静态自查：ROS 节点的 `self.xxx` 属性和参数名有没有写错。

为什么非要有这一条：2026-10-30 给 vision_node 加 `shadow_bias_scale` 时，
参数声明了、也用了 `self.shadow_bias_scale`，**就是忘了把参数读进属性**。
所有离线自查全过 —— 因为没有任何一条会真去 new 一个 Node —— 直到上真机，
第一帧就抛 `AttributeError: 'VisionNode' object has no attribute ...`，
整条视觉链直接停。这种错编译器和现有自查都看不见，只能静态查。

两条规则（只看 Node 子类的类体）：

1. 读到的 `self.x` 必须在类里被赋值过（`self.x = ...`），或者是个方法，
   或者是被调用着的 rclpy 接口（`self.get_logger()` 这种）；
2. `declare_parameter('p')` 声明的 `p`，必须真的被 `get_parameter('p')`
   读过 —— 否则 launch 里怎么改都不起作用，白白让人以为参数生效了。

自查本身也要有反面用例：喂一段"读了没赋值"的代码必须报错，
喂一段正常的必须不报 —— 不然这条检查自己坏了也没人知道。

用法::

    python tools/selftest_node_attributes.py
"""
import ast
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)

#: 要查的节点源码（相对仓库根）
NODE_FILES = (
    'dummy_vc_ws/src/dummy_vc_perception/dummy_vc_perception/vision_node.py',
    'dummy_vc_ws/src/dummy_vc_grasp/dummy_vc_grasp/pick_place_node.py',
    'dummy_vc_ws/src/dummy_vc_grasp/dummy_vc_grasp/gripper_can_node.py',
)

FAILURES = []


def fail(msg):
    FAILURES.append(msg)
    print(f'  [FAIL] {msg}')


def ok(msg):
    print(f'  [ok  ] {msg}')


def is_self(node):
    return isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) \
        and node.value.id == 'self'


def node_classes(tree):
    """类定义里基类带 Node 的那些（`class X(Node)`、`class X(SomeNode)`）。"""
    out = []
    for cls in [n for n in ast.walk(tree)
                if isinstance(n, ast.ClassDef)]:
        bases = []
        for b in cls.bases:
            if isinstance(b, ast.Name):
                bases.append(b.id)
            elif isinstance(b, ast.Attribute):
                bases.append(b.attr)
        if any(b.endswith('Node') for b in bases):
            out.append(cls)
    return out


def scan_class(cls):
    """返回 (读到的属性, 赋值过的属性, 方法名, 被调用的属性)。"""
    assigned, loaded, methods, called = set(), set(), set(), set()
    for n in ast.walk(cls):
        if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef)):
            methods.add(n.name)
        elif isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and is_self(n.func):
            called.add(n.func.attr)
    for n in ast.walk(cls):
        if not is_self(n):
            continue
        if isinstance(n.ctx, (ast.Store, ast.Del)):
            assigned.add(n.attr)
        elif isinstance(n.ctx, ast.Load):
            loaded.add(n.attr)
    return loaded, assigned, methods, called


def declared_params(cls):
    out = []
    for n in ast.walk(cls):
        if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) \
                and n.func.attr == 'declare_parameter' and n.args:
            arg = n.args[0]
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                out.append(arg.value)
    return out


def read_params(cls):
    """读了哪些参数名。`params = self.get_parameter` 这种别名也算。"""
    aliases = set()
    for n in ast.walk(cls):
        if isinstance(n, ast.Assign) and isinstance(n.value, ast.Attribute) \
                and n.value.attr == 'get_parameter' and is_self(n.value):
            for t in n.targets:
                if isinstance(t, ast.Name):
                    aliases.add(t.id)
    out = set()
    for n in ast.walk(cls):
        if not isinstance(n, ast.Call) or not n.args:
            continue
        arg = n.args[0]
        if not (isinstance(arg, ast.Constant) and isinstance(arg.value, str)):
            continue
        func = n.func
        if isinstance(func, ast.Attribute) and func.attr == 'get_parameter' \
                and is_self(func):
            out.add(arg.value)
        elif isinstance(func, ast.Name) and func.id in aliases:
            out.add(arg.value)
    return out


def audit(tree, label):
    """查一棵语法树，返回 [(问题描述), ...]。"""
    problems = []
    classes = node_classes(tree)
    if not classes:
        return [f'{label}：找不到 Node 子类，这条检查等于没查']
    for cls in classes:
        loaded, assigned, methods, called = scan_class(cls)
        for name in sorted(loaded):
            if name in assigned or name in methods or name in called:
                continue
            problems.append(
                f'{label}::{cls.name}：读了 self.{name}，但类里没有 '
                f'`self.{name} = ...`（也没有同名方法）')
        declared = declared_params(cls)
        read = read_params(cls)
        for name in declared:
            if name not in read:
                problems.append(
                    f'{label}::{cls.name}：参数 {name} 声明了，但没有 '
                    f'get_parameter({name!r}) 去读它 —— 改 launch 不会生效')
    return problems


def check_real_files():
    print('\n1. 仓库里的节点：读的属性都有赋值、声明的参数都有读')
    clean = 0
    for rel in NODE_FILES:
        path = os.path.join(REPO, rel)
        if not os.path.isfile(path):
            fail(f'找不到 {rel}')
            continue
        with open(path, encoding='utf-8') as fh:
            tree = ast.parse(fh.read(), filename=rel)
        problems = audit(tree, os.path.basename(rel))
        for p in problems:
            fail(p)
        if not problems:
            clean += 1
    if clean == len([f for f in NODE_FILES
                     if os.path.isfile(os.path.join(REPO, f))]):
        ok(f'{clean} 个节点文件都干净')


def check_detector_catches():
    print('\n2. 自查自己：喂一段"读了没赋值"必须报，喂正常的必须不报')
    bad = ast.parse(
        'class B(Node):\n'
        '    def __init__(self):\n'
        '        self.alpha = 1\n'
        '        self.declare_parameter("p", 1.0)\n'
        '    def f(self):\n'
        '        return self.alpha + self.beta + self.log()\n'
        '    def log(self):\n'
        '        return 0\n')
    problems = audit(bad, 'fixture')
    joined = '\n'.join(problems)
    if 'beta' in joined:
        ok('认出"读了 self.beta 但没赋值"')
    else:
        fail('漏了没赋值的属性 beta')
    if '参数 p 声明了' in joined:
        ok('认出"声明了参数 p 却没读"')
    else:
        fail('漏了声明了却没读的参数 p')
    good = ast.parse(
        'class G(Node):\n'
        '    def __init__(self):\n'
        '        self.declare_parameter("p", 1.0)\n'
        '        self.p = float(self.get_parameter("p").value)\n'
        '    def f(self):\n'
        '        return self.p + self.get_logger_check()\n'
        '    def get_logger_check(self):\n'
        '        return 0\n')
    if audit(good, 'fixture'):
        fail(f'正常代码被误报：{audit(good, "fixture")}')
    else:
        ok('正常代码不误报')


def check_real_bug_would_be_caught():
    """把 2026-10-30 那个真错原样造一遍：删掉赋值，必须报出来。"""
    print('\n3. 复现 2026-10-30 那个真错（声明了参数但忘了读进属性）')
    rel = 'dummy_vc_ws/src/dummy_vc_perception/dummy_vc_perception/vision_node.py'
    path = os.path.join(REPO, rel)
    with open(path, encoding='utf-8') as fh:
        lines = fh.readlines()
    broken = [ln for ln in lines
              if 'self.shadow_bias_scale =' not in ln]
    if len(broken) == len(lines):
        fail('vision_node 里找不到 shadow_bias_scale 的赋值，这个自查失去意义')
        return
    problems = audit(ast.parse(''.join(broken), filename=rel),
                     'vision_node.py(去掉赋值)')
    joined = '\n'.join(problems)
    if 'shadow_bias_scale' in joined:
        ok('删掉 `self.shadow_bias_scale = ...` 之后立刻报出来')
    else:
        fail('把赋值删了都没报出来，这条检查挡不住那类错')


def main():
    print('=' * 68)
    print('SELFTEST  node attributes')
    print('=' * 68)
    check_real_files()
    check_detector_catches()
    check_real_bug_would_be_caught()
    print()
    print('=' * 68)
    if FAILURES:
        print(f'{len(FAILURES)} 项失败')
        return 1
    print('RESULT: 每个节点读的 self.x 都有赋值，声明的参数都真的读了')
    return 0


if __name__ == '__main__':
    sys.exit(main())
