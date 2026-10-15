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
TRACKED = ('get_param_any', 'param_via_cli', 'parse_param_cli_output',
           'param_to_python', 'get_param', 'service_name_for')
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

print()
if problems:
    for p in problems:
        print('  [FAIL]', p)
else:
    print('  [ok  ] 元数、Check 方法、顶层函数名 全部一致')
print()
print('问题数:', len(problems))
sys.exit(1 if problems else 0)
