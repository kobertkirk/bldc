#!/usr/bin/env python3
"""
Generate the hierarchical KiCad 7 schematic for the controller from circuit.py.

Symbols are copied from the installed KiCad symbol libraries (so pin-outs are
the official library ones).  Every pin gets a short wire stub that ends in a
net label / power symbol / no-connect flag, which keeps the drawing readable
and makes the connectivity unambiguous.

usage: gen_schematic.py [--libdir /usr/share/kicad/symbols] [--out ../kicad]
"""
import argparse
import copy
import math
import os
import sys
import uuid as uuidlib

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import circuit  # noqa: E402
from sexp import Sym, dump, find, find1, parse, q  # noqa: E402

PROJECT = 'bldc48'
STUB = 2.54
GRID = 2.54
CHAR_W = 1.05          # approx width of a 1.27 mm label character
NS = uuidlib.UUID('7a1c1f50-8c2d-4b8e-9f00-b1dc48000000')

POWER_SYMBOL = {'GND': 'power:GND', '+48V': 'power:+48V', '+12V': 'power:+12V',
                '+5V': 'power:+5V', '+3V3': 'power:+3V3'}


def uid(*parts):
    """Deterministic UUIDs so regenerating doesn't churn the files."""
    return str(uuidlib.uuid5(NS, '/'.join(str(p) for p in parts)))


def n(v):
    s = ('%.4f' % v).rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


# ----------------------------------------------------------------------------
# Library handling
class Libs:
    def __init__(self, libdir):
        self.libdir = libdir
        self.cache = {}

    def _lib(self, lib):
        if lib not in self.cache:
            root = parse(open(os.path.join(self.libdir, lib + '.kicad_sym')).read())
            self.cache[lib] = {s[1]: s for s in find(root, 'symbol')}
        return self.cache[lib]

    def symbol(self, lib_id):
        """Return flattened symbol node named 'Lib:Name'."""
        lib, name = lib_id.split(':')
        syms = self._lib(lib)
        node = copy.deepcopy(syms[name])
        ext = find1(node, 'extends')
        if ext:
            parent = copy.deepcopy(syms[ext[1]])
            child_props = find(node, 'property')
            out = [Sym('symbol'), name]
            for item in parent[2:]:
                if isinstance(item, list) and item[0] == 'property':
                    continue
                if isinstance(item, list) and item[0] == 'symbol':
                    item[1] = name + item[1][len(ext[1]):]
                    out.append(item)
                    continue
                out.append(item)
            # properties go right after the header items, before sub-symbols
            idx = next(i for i, it in enumerate(out)
                       if isinstance(it, list) and it[0] == 'symbol')
            out[idx:idx] = child_props
            node = out
        node[1] = lib_id
        return node

    def pins(self, lib_id):
        node = self.symbol(lib_id)
        res = []
        for u in find(node, 'symbol'):
            for p in find(u, 'pin'):
                at = find1(p, 'at')
                res.append(dict(
                    type=str(p[1]), number=find1(p, 'number')[1], name=find1(p, 'name')[1],
                    x=float(at[1]), y=float(at[2]), angle=int(float(at[3])),
                    hidden='hide' in p[3:] or any(x == Sym('hide') for x in p)))
        return res

    def body_bbox(self, lib_id):
        node = self.symbol(lib_id)
        xs, ys = [0.0], [0.0]
        for u in find(node, 'symbol'):
            for item in u[2:]:
                if not isinstance(item, list):
                    continue
                if item[0] == 'rectangle':
                    for k in ('start', 'end'):
                        e = find1(item, k)
                        xs.append(float(e[1])); ys.append(float(e[2]))
                elif item[0] in ('polyline',):
                    for xy in find(find1(item, 'pts'), 'xy'):
                        xs.append(float(xy[1])); ys.append(float(xy[2]))
                elif item[0] == 'circle':
                    c = find1(item, 'center'); r = float(find1(item, 'radius')[1])
                    xs += [float(c[1]) - r, float(c[1]) + r]; ys += [float(c[2]) - r, float(c[2]) + r]
                elif item[0] == 'pin':
                    at = find1(item, 'at')
                    xs.append(float(at[1])); ys.append(float(at[2]))
        # convert to schematic (Y down)
        return min(xs), -max(ys), max(xs), -min(ys)


# ----------------------------------------------------------------------------
OUTWARD = {0: (-1, 0), 180: (1, 0), 90: (0, 1), 270: (0, -1)}  # schematic coords


def resolve_pins(comp, pins):
    """Map every visible pin to a net (or None = no-connect)."""
    by_num = {p['number']: p for p in pins}
    mapping = {}
    for key, net in comp['pins'].items():
        if key in by_num:
            mapping[key] = net
            continue
        hits = [p for p in pins if p['name'] == key]
        if not hits:
            raise SystemExit(f"{comp['ref']}: no pin '{key}' in {comp['lib_id']}")
        for p in hits:
            mapping.setdefault(p['number'], net)
    for p in pins:
        if p['hidden']:
            continue
        if p['number'] not in mapping and comp['lib_id'] != 'Mechanical:MountingHole':
            raise SystemExit(f"{comp['ref']}: pin {p['number']} ({p['name']}) unassigned")
    return mapping


def net_sheets():
    use = {}
    for c in circuit.components:
        for net in c['pins'].values():
            if net:
                use.setdefault(net, set()).add(c['sheet'])
    return use


class SheetWriter:
    def __init__(self, libs, sheet, title, root_uuid, sheet_uuid, page, global_nets):
        self.libs = libs
        self.sheet = sheet
        self.title = title
        self.root_uuid = root_uuid
        self.sheet_uuid = sheet_uuid
        self.page = page
        self.global_nets = global_nets
        self.items = []
        self.lib_ids = set()
        self.pwr_count = 0

    # -- primitive emitters ---------------------------------------------------
    def wire(self, x1, y1, x2, y2):
        self.items.append(
            f'(wire (pts (xy {n(x1)} {n(y1)}) (xy {n(x2)} {n(y2)})) '
            f'(stroke (width 0) (type default)) (uuid {uid(self.sheet, "w", x1, y1, x2, y2)}))')

    def label(self, net, x, y, ang):
        just = 'left' if ang in (0, 90) else 'right'
        if net in self.global_nets:
            self.items.append(
                f'(global_label {q(net)} (shape passive) (at {n(x)} {n(y)} {ang}) (fields_autoplaced) '
                f'(effects (font (size 1.27 1.27)) (justify {just})) (uuid {uid(self.sheet, "gl", net, x, y)}) '
                f'(property "Intersheetrefs" "${{INTERSHEET_REFS}}" (at {n(x)} {n(y)} 0) '
                f'(effects (font (size 1.27 1.27)) hide)))')
        else:
            self.items.append(
                f'(label {q(net)} (at {n(x)} {n(y)} {ang}) (fields_autoplaced) '
                f'(effects (font (size 1.27 1.27)) (justify {just} bottom)) '
                f'(uuid {uid(self.sheet, "l", net, x, y)}))')

    def no_connect(self, x, y):
        self.items.append(f'(no_connect (at {n(x)} {n(y)}) (uuid {uid(self.sheet, "nc", x, y)}))')

    def text(self, s, x, y, size=1.27):
        self.items.append(f'(text {q(s)} (at {n(x)} {n(y)} 0) '
                          f'(effects (font (size {size} {size})) (justify left top)) '
                          f'(uuid {uid(self.sheet, "t", x, y)}))')

    def symbol(self, lib_id, ref, value, x, y, rot=0, footprint='', fields=None,
               ref_pos=None, val_pos=None, hide_ref=False, pin_numbers=(),
               val_angle=0, val_center=False):
        self.lib_ids.add(lib_id)
        u = uid(self.sheet, 'sym', ref)
        rp = ref_pos or (x, y - 2)
        vp = val_pos or (x, y + 2)
        props = [
            ('Reference', ref, rp, hide_ref, 0, False),
            ('Value', value, vp, False, val_angle, val_center),
            ('Footprint', footprint, (x, y), True, 0, False),
            ('Datasheet', '~', (x, y), True, 0, False),
        ]
        for k, v in (fields or {}).items():
            props.append((k, v, (x, y), True, 0, False))
        s = (f'(symbol (lib_id {q(lib_id)}) (at {n(x)} {n(y)} {rot}) (unit 1)\n'
             f'    (in_bom yes) (on_board yes) (dnp no)\n    (uuid {u})\n')
        for name, val, (px, py), hide, ang, center in props:
            just = '' if (hide or center) else ' (justify left)'
            s += (f'    (property {q(name)} {q(val)} (at {n(px)} {n(py)} {ang}) '
                  f'(effects (font (size 1.27 1.27)){" hide" if hide else ""}{just}))\n')
        for pn in pin_numbers:
            s += f'    (pin {q(pn)} (uuid {uid(self.sheet, "pin", ref, pn)}))\n'
        s += (f'    (instances (project {q(PROJECT)} (path "/{self.root_uuid}/{self.sheet_uuid}" '
              f'(reference {q(ref)}) (unit 1))))\n  )')
        self.items.append(s)

    def power_symbol(self, net, x, y, outward, dense=False):
        self.pwr_count += 1
        ref = f'#PWR_{self.sheet}_{self.pwr_count:03d}'
        lib_id = POWER_SYMBOL[net]
        dx, dy = outward
        if net == 'GND':        # graphic points down at rot 0
            rot = {(0, 1): 0, (0, -1): 180, (1, 0): 90, (-1, 0): 270}[(dx, dy)]
        else:                   # graphic points up at rot 0
            rot = {(0, -1): 0, (0, 1): 180, (-1, 0): 90, (1, 0): 270}[(dx, dy)]
        # value text: horizontal, except on vertical stubs of dense ICs where
        # neighbouring pins are only 2.54 mm apart -> vertical text
        vertical = bool(dy) and dense
        tlen = len(net) * CHAR_W
        reach = 4.0 + (tlen / 2 if (vertical or dx) else 0.8)
        vx, vy = x + dx * reach, y + dy * reach
        # KiCad shows field angle = stored angle + symbol rotation
        shown = 90 if vertical else 0
        stored = (shown - rot) % 180
        self.symbol(lib_id, ref, net, x, y, rot, val_pos=(vx, vy), hide_ref=True,
                    pin_numbers=['1'], val_angle=stored, val_center=True)

    # -- component placement ---------------------------------------------------
    def footprint_box(self, comp):
        pins = self.libs.pins(comp['lib_id'])
        mapping = resolve_pins(comp, pins)
        x0, y0, x1, y1 = self.libs.body_bbox(comp['lib_id'])
        for p in pins:
            if p['hidden'] or p['number'] not in mapping:
                continue
            net = mapping[p['number']]
            dx, dy = OUTWARD[p['angle']]
            px, py = p['x'], -p['y']
            if net is None:
                ln = 0
            elif net in POWER_SYMBOL:
                ln = STUB + 4 + (len(net) * CHAR_W if dx else 0)
            else:
                ln = STUB + len(net) * CHAR_W + (3 if net in self.global_nets else 1)
            ex, ey = px + dx * ln, py + dy * ln
            # labels running vertically still need some horizontal room
            wpad = 1.5 if dy else 0
            x0 = min(x0, ex - wpad); x1 = max(x1, ex + wpad)
            y0 = min(y0, ey); y1 = max(y1, ey)
        # room for ref / value text
        tw = max(len(comp['ref']), len(comp['value'])) * CHAR_W
        x1 = max(x1, x0 + tw)
        return x0, y0 - 5, x1, y1 + 5, pins, mapping

    def place(self, comp, ox, oy, box, pins, mapping):
        x0, y0, x1, y1 = box
        fields = dict(comp['fields'])
        bx0, by0, bx1, by1 = self.libs.body_bbox(comp['lib_id'])
        self.symbol(comp['lib_id'], comp['ref'], comp['value'], ox, oy,
                    footprint=comp['footprint'], fields=fields,
                    ref_pos=(ox + bx1 + 1.0, oy + by0 - 1.0),
                    val_pos=(ox + bx1 + 1.0, oy + by1 + 2.0),
                    pin_numbers=[p['number'] for p in pins])
        done = set()
        for p in pins:
            if p['hidden'] or p['number'] not in mapping:
                continue
            px, py = ox + p['x'], oy - p['y']
            key = (round(px, 2), round(py, 2))
            if key in done:
                continue
            done.add(key)
            net = mapping[p['number']]
            if net is None:
                self.no_connect(px, py)
                continue
            dx, dy = OUTWARD[p['angle']]
            ex, ey = px + dx * STUB, py + dy * STUB
            self.wire(px, py, ex, ey)
            if net in POWER_SYMBOL:
                self.power_symbol(net, ex, ey, (dx, dy), dense=len(pins) > 3)
            else:
                ang = {(1, 0): 0, (-1, 0): 180, (0, -1): 90, (0, 1): 270}[(dx, dy)]
                self.label(net, ex, ey, ang)

    def layout(self, comps):
        boxes = [(c,) + self.footprint_box(c) for c in comps]
        for paper, (pw, ph) in (('A3', (420, 297)), ('A2', (594, 420)), ('A1', (841, 594))):
            placed = self._flow(boxes, pw, ph)
            if placed is not None:
                break
        self.paper = paper
        for c, ox, oy, box, pins, mapping in placed:
            self.place(c, ox, oy, box, pins, mapping)
        self.text(self.title, 15, 12, size=2.5)

    @staticmethod
    def _flow(boxes, pw, ph):
        left, top, right, bottom = 15.0, 22.0, pw - 15.0, ph - 40.0
        gap = 5.08
        x, y, row_h = left, top, 0.0
        out = []
        for c, x0, y0, x1, y1, pins, mapping in boxes:
            w, h = x1 - x0, y1 - y0
            if x + w > right and x > left:
                x = left
                y += row_h + gap
                row_h = 0.0
            ox = math.ceil((x - x0) / GRID) * GRID
            oy = math.ceil((y - y0) / GRID) * GRID
            if oy + y1 > bottom:
                return None
            out.append((c, ox, oy, (x0, y0, x1, y1), pins, mapping))
            x = ox + x1 + gap
            row_h = max(row_h, oy + y1 - y)
        return out

    def render(self, title_block):
        libsyms = '\n'.join('    ' + dump(self.libs.symbol(l), 2) for l in sorted(self.lib_ids))
        body = '\n  '.join(self.items)
        return (f'(kicad_sch (version 20230121) (generator eeschema)\n\n'
                f'  (uuid {self.sheet_uuid})\n\n  (paper "{self.paper}")\n\n{title_block}\n'
                f'  (lib_symbols\n{libsyms}\n  )\n\n  {body}\n)\n')


def title_block(title, page_comment=''):
    return (f'  (title_block\n    (title {q(title)})\n    (date "2026-10-06")\n    (rev "A")\n'
            f'    (company "48V 35A hub motor controller")\n'
            f'    (comment 1 "STM32G431 + LM5109B x3 + IPT015N10N5 x6 + INA240A2 x3")\n'
            f'    (comment 2 {q(page_comment)})\n  )\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--libdir', default='/usr/share/kicad/symbols')
    ap.add_argument('--out', default=os.path.join(os.path.dirname(__file__), '..', 'kicad'))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)
    libs = Libs(args.libdir)

    use = net_sheets()
    global_nets = {net for net, sheets in use.items() if len(sheets) > 1}
    root_uuid = uid('root')

    sheet_entries = []
    for page, (sheet, title) in enumerate(circuit.SHEETS, start=2):
        su = uid('sheet', sheet)
        w = SheetWriter(libs, sheet, title, root_uuid, su, page, global_nets)
        comps = [c for c in circuit.components if c['sheet'] == sheet]
        w.layout(comps)
        with open(os.path.join(args.out, f'{sheet}.kicad_sch'), 'w') as f:
            f.write(w.render(title_block(title, f'Sheet: {sheet}')))
        sheet_entries.append((sheet, title, su, page))
        print(f'{sheet}: {len(comps)} components, paper {w.paper}')

    # root sheet
    items = []
    for i, (sheet, title, su, page) in enumerate(sheet_entries):
        x, y = 30 + (i % 2) * 120, 50 + (i // 2) * 60
        items.append(
            f'(sheet (at {x} {y}) (size 90 35) (fields_autoplaced)\n'
            f'    (stroke (width 0.1524) (type solid)) (fill (color 0 0 0 0.0000))\n'
            f'    (uuid {su})\n'
            f'    (property "Sheetname" {q(title)} (at {x} {n(y - 0.7)} 0) '
            f'(effects (font (size 1.5 1.5)) (justify left bottom)))\n'
            f'    (property "Sheetfile" "{sheet}.kicad_sch" (at {x} {n(y + 35.6)} 0) '
            f'(effects (font (size 1.27 1.27)) (justify left top)))\n'
            f'    (instances (project {q(PROJECT)} (path "/{root_uuid}" (page "{page}"))))\n  )')
    notes = open(os.path.join(os.path.dirname(__file__), 'root_notes.txt')).read().strip()
    items.append(f'(text {q(notes)} (at 30 185 0) (effects (font (size 1.6 1.6)) (justify left top)) '
                 f'(uuid {uid("root", "notes")}))')
    root = (f'(kicad_sch (version 20230121) (generator eeschema)\n\n  (uuid {root_uuid})\n\n'
            f'  (paper "A3")\n\n{title_block("48V 35A BLDC hub motor controller", "Top level")}\n'
            f'  (lib_symbols)\n\n  ' + '\n  '.join(items) +
            f'\n\n  (sheet_instances (path "/" (page "1")))\n)\n')
    with open(os.path.join(args.out, f'{PROJECT}.kicad_sch'), 'w') as f:
        f.write(root)


if __name__ == '__main__':
    main()
