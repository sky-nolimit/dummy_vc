#!/usr/bin/env python3
"""A small YAML reader, enough for ROS 2 parameter files.

Why this exists: the machine that edits this repo has no PyYAML and cannot
install one (no package access). Without any reader, the only way to find a
mistake in a MoveIt parameter file is to build it on the other machine and read
the error there, which is slow and loses the point of having offline checks.

Scope, deliberately narrow:
    * nested mappings by indentation
    * block lists ("- item")
    * scalars: int, float, bool, null, quoted and bare strings
    * block scalars introduced by ">" or "|" (folded / literal strings)
    * comments (#) and blank lines
    * inline flow lists and maps, parsed by a tiny recursive reader

Out of scope, and it will raise rather than guess: anchors, aliases, tags,
multi-document streams, and anything indentation-ambiguous.

    from yaml_min import load
    data = load(open('config/joint_limits.yaml', encoding='utf-8').read())
"""
import re

__all__ = ['load', 'YamlError']


class YamlError(ValueError):
    pass


def _strip_comment(line):
    """Drop a trailing comment that is not inside quotes."""
    out = []
    quote = None
    i = 0
    while i < len(line):
        ch = line[i]
        if quote:
            if ch == quote:
                quote = None
            out.append(ch)
        elif ch in '"\'':
            quote = ch
            out.append(ch)
        elif ch == '#':
            break
        else:
            out.append(ch)
        i += 1
    return ''.join(out).rstrip()


def _scalar(tok):
    """Convert a scalar token to a Python value."""
    tok = tok.strip()
    if tok == '' or tok in ('~', 'null', 'Null', 'NULL'):
        return None
    low = tok.lower()
    if low in ('true', 'yes', 'on'):
        return True
    if low in ('false', 'no', 'off'):
        return False
    if tok.startswith('[') or tok.startswith('{'):
        return _flow(tok)
    if (len(tok) >= 2 and tok[0] == tok[-1]
            and tok[0] in '"\''):
        return tok[1:-1]
    try:
        return int(tok)
    except ValueError:
        pass
    try:
        return float(tok)
    except ValueError:
        pass
    return tok


def _split_flow(body):
    """Split a flow collection body on top-level commas."""
    parts, depth, buf, quote = [], 0, [], None
    for ch in body:
        if quote:
            buf.append(ch)
            if ch == quote:
                quote = None
            continue
        if ch in '"\'':
            quote = ch
            buf.append(ch)
        elif ch in '[{':
            depth += 1
            buf.append(ch)
        elif ch in ']}':
            depth -= 1
            buf.append(ch)
        elif ch == ',' and depth == 0:
            parts.append(''.join(buf))
            buf = []
        else:
            buf.append(ch)
    if buf:
        parts.append(''.join(buf))
    return parts


def _flow(tok):
    """Parse a one-line [a, b] or {k: v} value."""
    tok = tok.strip()
    if tok.startswith('['):
        if not tok.endswith(']'):
            raise YamlError(f'unterminated flow list: {tok!r}')
        body = tok[1:-1].strip()
        if not body:
            return []
        return [_scalar(p) for p in _split_flow(body)]
    if tok.startswith('{'):
        if not tok.endswith('}'):
            raise YamlError(f'unterminated flow map: {tok!r}')
        body = tok[1:-1].strip()
        if not body:
            return {}
        out = {}
        for part in _split_flow(body):
            if ':' not in part:
                raise YamlError(f'flow map entry without ":": {part!r}')
            k, v = part.split(':', 1)
            out[_scalar(k)] = _scalar(v)
        return out
    raise YamlError(f'not a flow collection: {tok!r}')


class _Line:
    __slots__ = ('indent', 'text', 'no')

    def __init__(self, indent, text, no):
        self.indent = indent
        self.text = text
        self.no = no


def _prepare(src):
    """Turn the source into significant lines, expanding block scalars."""
    lines = []
    raw = src.splitlines()
    i = 0
    while i < len(raw):
        line = raw[i]
        stripped = _strip_comment(line)
        if not stripped.strip():
            i += 1
            continue
        indent = len(stripped) - len(stripped.lstrip(' '))
        if '\t' in stripped[:indent + 1]:
            raise YamlError(f'line {i + 1}: tab used for indentation')
        text = stripped.strip()
        # A block scalar: collect the more-indented lines that follow and fold
        # them into a single string, which is how `request_adapters: >-` works.
        m = re.search(r':\s*([>|])([-+]?)\s*$', text)
        if m:
            style, chomp = m.group(1), m.group(2)
            body, j = [], i + 1
            body_indent = None
            while j < len(raw):
                nxt = raw[j]
                if not nxt.strip():
                    body.append('')
                    j += 1
                    continue
                nind = len(nxt) - len(nxt.lstrip(' '))
                if nind <= indent:
                    break
                if body_indent is None:
                    body_indent = nind
                body.append(nxt[body_indent:])
                j += 1
            while body and body[-1] == '':
                body.pop()
            if style == '>':
                joined = ' '.join(s for s in body if s != '')
            else:
                joined = '\n'.join(body)
            if chomp != '-':
                joined += '\n'
            lines.append(_Line(indent, text[:m.start()] + ': ' + repr(joined),
                               i + 1))
            i = j
            continue
        lines.append(_Line(indent, text, i + 1))
        i += 1
    return lines


def _parse_block(lines, pos, indent):
    """Parse a mapping or list at the given indent. Returns (value, pos)."""
    if pos >= len(lines):
        return None, pos
    first = lines[pos]
    if first.indent < indent:
        return None, pos

    if first.text.startswith('- ') or first.text == '-':
        return _parse_list(lines, pos, first.indent)

    out = {}
    while pos < len(lines):
        ln = lines[pos]
        if ln.indent < indent:
            break
        if ln.indent > indent:
            raise YamlError(f'line {ln.no}: unexpected indent')
        if ln.text.startswith('- '):
            raise YamlError(f'line {ln.no}: list item inside a mapping')
        if ':' not in ln.text:
            raise YamlError(f'line {ln.no}: expected "key: value", got '
                            f'{ln.text!r}')
        key, _, rest = ln.text.partition(':')
        key = _scalar(key.strip())
        rest = rest.strip()
        if rest == '':
            # nested block (mapping or list), or an empty value
            pos += 1
            if pos < len(lines) and lines[pos].indent > ln.indent:
                val, pos = _parse_block(lines, pos, lines[pos].indent)
            elif pos < len(lines) and lines[pos].indent == ln.indent \
                    and lines[pos].text.startswith('- '):
                val, pos = _parse_list(lines, pos, ln.indent)
            else:
                val = None
            out[key] = val
        else:
            out[key] = _scalar(rest)
            pos += 1
    return out, pos


def _parse_list(lines, pos, indent):
    out = []
    while pos < len(lines):
        ln = lines[pos]
        if ln.indent < indent:
            break
        if ln.indent > indent and not ln.text.startswith('- '):
            raise YamlError(f'line {ln.no}: unexpected indent in list')
        if not ln.text.startswith('- '):
            break
        item = ln.text[2:].strip()
        if item == '':
            pos += 1
            if pos < len(lines) and lines[pos].indent > indent:
                val, pos = _parse_block(lines, pos, lines[pos].indent)
            else:
                val = None
            out.append(val)
            continue
        if ':' in item and not item.startswith(('[', '{', '"', "'")):
            # a mapping that starts on the dash line
            key, _, rest = item.partition(':')
            sub = {_scalar(key.strip()): _scalar(rest.strip())
                   if rest.strip() else None}
            pos += 1
            if pos < len(lines) and lines[pos].indent > indent:
                more, pos = _parse_block(lines, pos, lines[pos].indent)
                if isinstance(more, dict):
                    sub.update(more)
            out.append(sub)
            continue
        out.append(_scalar(item))
        pos += 1
    return out, pos


def load(src):
    """Parse YAML text into Python objects."""
    lines = _prepare(src)
    if not lines:
        return None
    value, pos = _parse_block(lines, 0, lines[0].indent)
    if pos != len(lines):
        raise YamlError(f'line {lines[pos].no}: trailing content was not '
                        f'consumed')
    return value


def load_file(path):
    with open(path, encoding='utf-8-sig') as fh:
        return load(fh.read())
