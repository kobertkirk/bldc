"""Minimal S-expression reader/writer for KiCad files."""
import re

_tok = re.compile(r'\(|\)|"(?:\\.|[^"\\])*"|[^\s()"]+')


class Sym(str):
    """Bare (unquoted) atom."""


def parse(text):
    stack = [[]]
    for m in _tok.finditer(text):
        t = m.group(0)
        if t == '(':
            stack.append([])
        elif t == ')':
            v = stack.pop()
            stack[-1].append(v)
        elif t[0] == '"':
            stack[-1].append(t[1:-1].replace('\\"', '"').replace('\\\\', '\\'))
        else:
            stack[-1].append(Sym(t))
    return stack[0][0]


def q(s):
    return '"' + str(s).replace('\\', '\\\\').replace('"', '\\"').replace('\n', '\\n') + '"'


def dump(node, indent=0):
    if not isinstance(node, list):
        if isinstance(node, Sym):
            return str(node)
        if isinstance(node, (int, float)):
            return fmt_num(node)
        return q(node)
    pad = '  ' * indent
    simple = all(not isinstance(c, list) for c in node)
    if simple:
        return '(' + ' '.join(dump(c) for c in node) + ')'
    out = '('
    first = True
    for c in node:
        if isinstance(c, list):
            out += '\n' + pad + '  ' + dump(c, indent + 1)
        else:
            out += ('' if first else ' ') + dump(c)
        first = False
    return out + ')'


def fmt_num(v):
    if isinstance(v, int):
        return str(v)
    s = ('%.4f' % v).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def find(node, key):
    return [c for c in node if isinstance(c, list) and c and c[0] == key]


def find1(node, key):
    r = find(node, key)
    return r[0] if r else None
