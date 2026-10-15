"""静态自查 verify_moveit_vm.py：这次两个 bug 都能被它抓到。

1. get_param_any / param_via_cli / parse_param_cli_output 的调用点必须按
   它们实际返回的元数解包（上一个 bug 就是漏改了一处，运行时才 ValueError）。
2. check.<method>() 必须是 Check 上真的存在的方法，参数个数也得对得上
   （新增 check.note 时最容易漏）。
3. 顶层 def 之间没有重名。
"""
import ast
import sys

SRC = 'tools/verify_moveit_vm.py'
src = open(SRC, encoding='utf-8').read()
tree = ast.parse(src)


def returns_arity(fn):
    """函数体里所有 return 返回的元数；不一致就报错。"""
    arities = set()
    for node in ast.walk(fn):
        if isinstance(node, ast.Return):
            if node.value is None:
                arities.add(0)
            elif isinstance(node.value, ast.Tuple):
                arities.add(len(node.value.elts))
            else:
                arities.add(1)
    return arities


funcs = {}
for node in tree.body:
    if isinstance(node, ast.FunctionDef):
        funcs[node.name] = node

check_cls = next(n for n in tree.body
                 if isinstance(n, ast.ClassDef) and n.name == 'Check')
check_methods = {m.name: m for m in check_cls.body
                 if isinstance(m, ast.FunctionDef)}
print('Check 的方法:', sorted(check_methods))

problems = []


def arity_of_return_value(value, known):
    """一个 return 表达式返回几个值。

    `return a, b`      -> 2
    `return f()`       -> f 的返回元数（不知道就算 1）
    `return x`         -> 1
    `return`           -> 0
    """
    if value is None:
        return 0
    if isinstance(value, ast.Tuple):
        return len(value.elts)
    if isinstance(value, ast.Call) and isinstance(value.func, ast.Name):
        if value.func.id in known and len(known[value.func.id]) == 1:
            return max(known[value.func.id])
        return None      # 转调未知函数：元数不确定，由调用点负责
    return 1


# --- 1. 元数 ---
TRACKED = ('get_param_typed', 'get_param_any', 'param_via_cli',
           'parse_param_cli_output', 'param_to_python', 'get_param',
           'service_name_for')
arities = {}
unresolved = {}
# 先算一次粗略值，好让转调能解析到被调函数的元数
for name in TRACKED:
    if name in funcs:
        arities[name] = {arity_of_return_value(n.value, {})
                         for n in ast.walk(funcs[name])
                         if isinstance(n, ast.Return)}
for name in TRACKED:
    if name not in funcs:
        problems.append('找不到函数 %s' % name)
        continue
    a = {arity_of_return_value(n.value, arities)
         for n in ast.walk(funcs[name]) if isinstance(n, ast.Return)}
    arities[name] = a
    if None in a:
        unresolved[name] = True
        a = {x for x in a if x is not None}
    if len(a) > 1:
        problems.append('%s 的 return 元数不一致: %s' % (name, sorted(a)))
    print('  %-24s 返回元数 %s%s'
          % (name, sorted(a) or [0], ' (含不确定的转调)' if name in unresolved else ''))

for node in ast.walk(tree):
    if not isinstance(node, ast.Call):
        continue
    f = node.func
    if not isinstance(f, ast.Name) or f.id not in arities:
        continue
    vals = [x for x in arities[f.id] if x is not None]
    want = max(vals) if vals else 0
    # 找出这个调用被赋给了几个目标。注意不能只看 Assign：
    # `a, b = f()` 和 `return f()` 都要能认出来，而且同一个调用表达式在树里
    # 只应被算一次，否则会重复报同一个问题。
    got = None
    for parent in ast.walk(tree):
        if isinstance(parent, ast.Assign) and parent.value is node:
            if len(parent.targets) == 1 and isinstance(parent.targets[0], ast.Tuple):
                got = len(parent.targets[0].elts)
            else:
                got = 1
            break
        if isinstance(parent, ast.Return) and parent.value is node:
            got = None      # 由上面"元数是否一致"那条负责，不重复报
            break
    if isinstance(got, int) and got != want:
        problems.append('第 %d 行 %s() 返回 %d 个值，却按 %d 个解包'
                        % (node.lineno, f.id, want, got))
    elif isinstance(got, int):
        print('  L%-4d %s() -> 解包 %d 个' % (node.lineno, f.id, got))

# --- 2. Check 方法 ---
for node in ast.walk(tree):
    if not isinstance(node, ast.Call):
        continue
    f = node.func
    if (isinstance(f, ast.Attribute) and isinstance(f.value, ast.Name)
            and f.value.id == 'check'):
        m = f.attr
        if m not in check_methods:
            problems.append('第 %d 行 check.%s() 不存在' % (node.lineno, m))
            continue
        sig = check_methods[m]
        nargs = len(sig.args.args) - 1          # 去掉 self
        ndef = len(sig.args.defaults)
        ncall = len(node.args)
        if not (nargs - ndef) <= ncall <= nargs:
            problems.append('第 %d 行 check.%s() 传了 %d 个参数，'
                            '接受 %d..%d 个'
                            % (node.lineno, m, ncall, nargs - ndef, nargs))

# --- 3. 顶层 def 重名 ---
seen = {}
for node in tree.body:
    if isinstance(node, ast.FunctionDef):
        if node.name in seen:
            problems.append('顶层函数重名: %s (L%d 与 L%d)'
                            % (node.name, seen[node.name], node.lineno))
        seen[node.name] = node.lineno

# --- 4. get_param_typed 的类型门 ---
# 这条是这次真机上踩到的：参数服务把 robot_description_semantic 读成了 bool，
# 于是 `'<group name="..."' in srdf` 报
# `TypeError: argument of type 'bool' is not iterable`。
# 静态检查看不出这个，所以真跑一遍逻辑：把三个依赖函数换成桩，
# 确认"服务给了错的类型"时会退到 CLI，而 CLI 也不对时才判失败。
try:
    import importlib.util

    spec = importlib.util.spec_from_file_location('verify_moveit_vm', SRC)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)          # 这个模块顶层不导入 rclpy，能安全执行
except Exception as exc:                  # pragma: no cover
    print('  [skip] 类型门测试没能加载模块: %s' % exc)
else:
    def _probe(service_ok, service_val, cli_ok, cli_val, want, label):
        mod.get_param = lambda *a, **k: (service_ok, service_val)
        mod.param_to_python = lambda v: v
        # param_via_cli 返回 (ok, value, raw)，桩也要给三个
        mod.param_via_cli = lambda *a, **k: (cli_ok, cli_val, repr(cli_val))
        ok, val, via, detail = mod.get_param_typed(None, '/move_group', 'x', want)
        print('    %-46s -> ok=%-5s via=%-15s val=%-9s %s'
              % (label, ok, via or '-', repr(val)[:9], detail))
        return ok, via, detail

    print()
    print('  类型门:')
    # 服务给了 bool（就是真机上的情况）→ 必须退到 CLI，且用 CLI 的字符串
    ok, via, _ = _probe(True, True, True, '<robot/>', str, '服务给 bool，CLI 给 str')
    if not (ok and via == 'ros2 param get'):
        problems.append('服务类型不对时没有退到 CLI（ok=%s via=%s）' % (ok, via))
    # 两条路都给了错的类型 → 必须失败，而且 detail 要说清两种类型
    ok, _, detail = _probe(True, True, True, 42, str, '两条路类型都不对')
    if ok or 'bool' not in detail or 'int' not in detail:
        problems.append('两条路类型都不对时判定有误: ok=%s detail=%s' % (ok, detail))
    # CLI 读到了但解析不出来 → detail 里必须带原文，否则排查看不出区别
    ok, _, detail = _probe(True, True, False, None, str, 'CLI 解析失败（要看得到原文）')
    if ok or '也没读出来' not in detail:
        problems.append('CLI 解析失败时没把原文带出来: %s' % detail)
    # 服务类型正确 → 直接用服务的，绝不调 CLI。这里把 CLI 设成"一被调用就抛"，
    # 用异常来证明它没被走到；抛出来就说明我们的判定逻辑错了。
    def _must_not_call(*a, **k):
        raise AssertionError('服务类型正确时不该走 CLI')
    mod.get_param = lambda *a, **k: (True, '<robot/>')
    mod.param_to_python = lambda v: v
    mod.param_via_cli = _must_not_call
    try:
        ok, val, via, _ = mod.get_param_typed(None, '/move_group', 'x', str)
    except AssertionError as exc:
        problems.append(str(exc))
        ok, val, via = False, None, ''
    print('    %-46s -> ok=%-5s via=%-15s val=%s'
          % ('服务类型正确（不该走 CLI）', ok, via, repr(val)))
    if not (ok and via == 'service' and val == '<robot/>'):
        problems.append('服务类型正确时判定有误: ok=%s via=%s val=%s'
                        % (ok, via, val))

print()
if problems:
    for p in problems:
        print('  [FAIL]', p)
else:
    print('  [ok  ] 元数、Check 方法、顶层函数名 全部一致')
print()
print('问题数:', len(problems))
sys.exit(1 if problems else 0)
