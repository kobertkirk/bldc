#!/usr/bin/env python3
"""
Create bldc48.kicad_pcb from circuit.py using KiCad's pcbnew Python API.

The power stage is placed by hand (coordinates below); small parts are packed
into regions next to the part they serve.  Nets are assigned to every pad and
each footprint is linked to its schematic symbol, so "Update PCB from
Schematic" works afterwards without losing placement.  Tracks are not routed
here: route_pcb.py routes this placement and adds planes/pours.

Board plan (mm, origin top-left, power flows left -> right -> down):

  x 0..20     battery strip: BAT+ / BAT- M5 bolt terminals on the left edge,
              36 mm apart, TVS between them, bus-voltage divider below
  x 20..134   bulk capacitors along the top edge, then one 38 mm column per
              phase:  [gate driver | high FET over low FET | Kelvin shunt]
              - both FETs drain-tab up: the switch node is the short gap
                between the high-side source leads and the low-side tab
              - DC-link ceramics directly under the low-side source leads
              - motor terminal on the bottom edge straight below the shunt
  x 136..188  controller: bucks, LDO, power latch, MCU, connectors

usage: gen_pcb.py  (needs `import pcbnew`, i.e. run with KiCad's python)
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import circuit  # noqa: E402
from gen_schematic import Libs, resolve_pins, uid  # noqa: E402

FPDIR = '/usr/share/kicad/footprints'
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'kicad', 'bldc48.kicad_pcb')
W, H = 188.0, 74.0           # board size, mm
GAP = 1.5                    # min spacing between packed courtyards (room for fan-out vias)

# power-stage geometry (shared with route_pcb.py)
COL0, COLW = 20.0, 38.0      # first phase column x, column pitch
POWER_X = COL0 + 3 * COLW    # 134: power stage left of this, controller right
Y_HI, Y_LO = 25.0, 40.0      # FET centres (switch-node gap between them)
Y_DRV = (Y_HI + Y_LO) / 2    # gate driver centre
Y_SHUNT = 33.0
Y_INA = 46.0
Y_CER = 52.5                 # DC-link ceramics under the low-side sources
Y_TERM = H - 9.5             # motor terminal centres (bottom edge)
# x offsets inside a phase column
DRV_X, FET_X, SHUNT_X, INA_X, TERM_X, CER_DX = 5.5, 16.5, 28.0, 26.0, 32.5, 3.4
BAT_POS = {'BAT+': (10.0, 12.0), 'BAT-': (10.0, 42.0)}
TVS_POS = (10.0, 27.0)


def col_x(ph):
    return COL0 + COLW * 'ABC'.index(ph)


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def load_fp(fpid):
    lib, name = fpid.split(':')
    fp = pcbnew.FootprintLoad(os.path.join(FPDIR, lib + '.pretty'), name)
    if fp is None:
        raise SystemExit(f'footprint {fpid} not found')
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def size_of(fp):
    """(width, height, cx, cy): courtyard size and its centre relative to the origin."""
    fp.SetPosition(mm(0, 0))
    fp.BuildCourtyardCaches()
    poly = fp.GetCourtyard(pcbnew.F_CrtYd)
    bb = poly.BBox() if poly.OutlineCount() else fp.GetBoundingBox(False, False)
    c = bb.GetCenter()
    return (pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight()),
            pcbnew.ToMM(c.x), pcbnew.ToMM(c.y))


def orient(fp, pad, direction):
    """Rotate so `pad` points in `direction` ('up','down','left','right') from the origin."""
    vec = {'up': (0, -1), 'down': (0, 1), 'left': (-1, 0), 'right': (1, 0)}[direction]
    fp.SetPosition(mm(0, 0))
    best = None
    for rot in (0, 90, 180, 270):
        fp.SetOrientationDegrees(rot)
        p = next(q for q in fp.Pads() if q.GetNumber() == pad).GetPosition()
        score = vec[0] * p.x + vec[1] * p.y
        if best is None or score > best[0]:
            best = (score, rot)
    fp.SetOrientationDegrees(best[1])


def put(fp, x, y, pad=None, direction=None):
    if pad:
        orient(fp, pad, direction)
    fp.SetPosition(mm(x, y))


def role_of(c, ina_caps):
    """Placement role: an explicit power-stage slot or a packing region."""
    s, ref, v, lib = c['sheet'], c['ref'], c['value'], c['lib_id']
    pins = set(n for n in c['pins'].values() if n)
    if ref.startswith('#'):
        return None
    if s == 'bridge':
        if lib == 'Device:Thermistor_NTC':
            return 'ntc'
        for ph in 'ABC':
            if any(n.endswith('_' + ph) for n in pins):
                if lib.startswith('Transistor_FET'):
                    return f'hifet{ph}' if '+48V' in pins else f'lofet{ph}'
                if lib == 'Device:R_Shunt':
                    return f'shunt{ph}'
                if ref.startswith('J'):
                    return f'term{ph}'
                if v.startswith('INA240'):
                    return f'ina{ph}'
                if lib.startswith('Driver_FET'):
                    return f'driver{ph}'
                if any(n.startswith(('ISO_', 'ISENSE_')) for n in pins):
                    return f'sense{ph}'
                return f'drv{ph}'
        if v == '100n' and pins == {'+3V3', 'GND'}:        # INA240 decouplers, in phase order
            ina_caps.append(ref)
            return 'sense' + 'ABC'[len(ina_caps) - 1]
        if lib == 'Device:C_Polarized':
            return 'bulk'
        if v == '2.2u/100V':
            return 'ceramic'
        return 'bus_sense'
    if s == 'power':
        if v in ('BAT+', 'BAT-'):
            return 'batterm'
        if v == 'SMCJ60CA':
            return 'tvs'
        if any(n.endswith('12') for n in pins) or '+12V' in pins:
            return 'buck12'
        if any(n.endswith('5') and n != '+5V' for n in pins) or '+5V' in pins and v != 'AP2112K-3.3' \
                and '+3V3' not in pins:
            return 'buck5'
        if any(n in pins for n in ('LATCH_B', 'LATCH_PULL', 'HOLD_B', 'PWR_BTN', 'EN_RAW', 'BUCK_EN')):
            return 'latch'
        return 'ldo'
    if s == 'mcu':
        return 'conn' if ref.startswith('J') else 'mcu'
    if s == 'io':
        if v == 'M3':
            return 'holes'
        return 'conn' if ref.startswith('J') else 'io'
    return 'misc'


# Packing regions: list of (x0, y0, x1, y1) filled in order
REGIONS = {
    'bulk':      [(COL0, 1.5, POWER_X, 16.5)],
    'bus_sense': [(0.5, 49, 19.5, 66)],
    'buck12':    [(136, 1.5, 160, 22.5)],
    'buck5':     [(136, 23, 160, 44)],
    'ldo':       [(136, 44.5, 160, 53)],
    'latch':     [(136, 53.5, 160, 72.5)],
    'mcu':       [(160.5, 1.5, 180, 24), (180, 8.5, 187.5, 24)],
    'io':        [(160.5, 24.5, 187.5, 42)],
    'conn':      [(160.5, 42.5, 187.5, 66), (160.5, 66, 180, 72.5)],
}
GROUP_GAP = {'bulk': 1.0}
for _ph in 'ABC':
    _x = col_x(_ph)
    REGIONS[f'drv{_ph}'] = [(_x + 0.3, 17, _x + 10.8, Y_DRV - 2.9), (_x + 0.3, Y_DRV + 2.9, _x + 10.8, H - 1)]
    REGIONS[f'sense{_ph}'] = [(_x + 22.0, Y_INA + 3.5, _x + 30.0, Y_TERM - 6.0)]


def pack(items, regions, gap=GAP):
    """Pack footprints left-to-right, top-to-bottom into the regions in turn."""
    ri = 0
    x0, y0, x1, y1 = regions[ri]
    x, y, row = x0, y0, 0.0
    for fp, (w, h, cx, cy) in items:
        w += gap
        h += gap
        while True:
            if x + w > x1 and x > x0:
                x, y, row = x0, y + row, 0.0
            if y + h <= y1 + 0.01 or ri == len(regions) - 1:
                break
            ri += 1
            x0, y0, x1, y1 = regions[ri]
            x, y, row = x0, y0, 0.0
        if y + h > y1 + 0.01:
            print(f'warning: regions {regions} overflow at {fp.GetReference()}')
        fp.SetPosition(mm(x + w / 2 - cx, y + h / 2 - cy))
        x += w
        row = max(row, h)


def place_power(groups):
    def one(role):
        return groups.pop(role)[0][0]

    for v, (x, y) in BAT_POS.items():
        fp = next(f for f, _ in groups['batterm'] if f.GetValue() == v)
        put(fp, x, y)
    groups.pop('batterm')
    put(one('tvs'), *TVS_POS, pad='1', direction='up')    # +48V end towards BAT+
    ceramics = [f for f, _ in groups.pop('ceramic')]
    for ph in 'ABC':
        x = col_x(ph)
        xf = x + FET_X
        put(one(f'hifet{ph}'), xf, Y_HI, pad='3', direction='up')     # drain tab to +48V bus
        put(one(f'lofet{ph}'), xf, Y_LO, pad='3', direction='up')     # drain tab to switch node
        put(one(f'driver{ph}'), x + DRV_X, Y_DRV, pad='7', direction='right')
        put(one(f'shunt{ph}'), x + SHUNT_X, Y_SHUNT, pad='1', direction='left')
        put(one(f'ina{ph}'), x + INA_X, Y_INA, pad='1', direction='left')
        put(one(f'term{ph}'), x + TERM_X, Y_TERM)
        for k, dx in enumerate((-CER_DX, CER_DX)):                    # GND pad up, at the sources
            put(ceramics[2 * 'ABC'.index(ph) + k], xf + dx, Y_CER, pad='2', direction='up')
    put(one('ntc'), col_x('B') + FET_X, Y_CER + 5.0)                  # next to the middle low-side FET



def unclash_refs(board):
    """Move a reference off a neighbour's silkscreen or reference (copper is
    untouched, so routing stays valid); hide it if no nearby spot is free."""
    silk = [(f, g.GetBoundingBox()) for f in board.GetFootprints() for g in f.GraphicalItems()
            if g.GetLayer() == pcbnew.F_SilkS and g.GetClass() != 'PCB_TEXT']

    def clashes(fp):
        bb = fp.Reference().GetBoundingBox()
        return (any(f is not fp and bb.Intersects(b) for f, b in silk) or
                any(f is not fp and f.Reference().IsVisible() and bb.Intersects(f.Reference().GetBoundingBox())
                    for f in board.GetFootprints()))

    for fp in board.GetFootprints():
        ref = fp.Reference()
        if not ref.IsVisible() or not clashes(fp):
            continue
        p, r = fp.GetPosition(), ref.GetPosition()
        ys = (r.y, 2 * p.y - r.y)
        dxs = [pcbnew.FromMM(d) for d in (0, -1, 1, -2, 2, -3, 3)]
        for cand in [pcbnew.VECTOR2I(r.x + dx, y) for dx in dxs for y in ys][1:]:
            ref.SetPosition(cand)
            if not clashes(fp):
                break
        else:
            ref.SetPosition(r)
            ref.SetVisible(False)
            print(f'note: hid crowded reference {fp.GetReference()}')


def main():
    libs = Libs('/usr/share/kicad/symbols')
    board = pcbnew.BOARD()
    ds = board.GetDesignSettings()
    board.SetCopperLayerCount(4)
    ds.SetCopperLayerCount(4)
    ds.m_TrackMinWidth = pcbnew.FromMM(0.15)
    ds.m_MinClearance = pcbnew.FromMM(0.15)
    ds.m_MinThroughDrill = pcbnew.FromMM(0.2)   # LM5164 thermal vias

    nets = {}

    def net(name):
        if name not in nets:
            ni = pcbnew.NETINFO_ITEM(board, name)
            board.Add(ni)
            nets[name] = ni
        return nets[name]

    groups = {}
    ina_caps = []
    for c in circuit.components:
        g = role_of(c, ina_caps)
        if g is None or not c['footprint']:
            continue
        fp = load_fp(c['footprint'])
        if 'PinHeader' in c['footprint']:
            fp.SetOrientationDegrees(90)          # lay the SWD header flat
        fp.SetReference(c['ref'])
        fp.SetValue(c['value'])
        sheet_uuid = uid('sheet', c['sheet'])
        sym_uuid = uid(c['sheet'], 'sym', c['ref'])
        fp.SetPath(pcbnew.KIID_PATH(f'/{sheet_uuid}/{sym_uuid}'))
        board.Add(fp)
        if c['pins']:
            pins = libs.pins(c['lib_id'])
            mapping = resolve_pins(c, pins)
            # hidden power pins (e.g. INA240 pin 4 GND)
            for p in pins:
                if p['hidden'] and p['number'] not in mapping and p['type'] == 'power_in':
                    mapping[p['number']] = p['name']
                if p['hidden'] and p['number'] not in mapping:
                    twin = [q for q in pins if q['name'] == p['name'] and not q['hidden']]
                    if twin:
                        mapping[p['number']] = mapping.get(twin[0]['number'])
            for pad in fp.Pads():
                nm = mapping.get(pad.GetNumber())
                if nm:
                    pad.SetNet(net(nm))
        groups.setdefault(g, []).append((fp, size_of(fp)))

    place_power(groups)
    holes = groups.pop('holes')
    for (fp, _), pos in zip(holes, [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]):
        fp.SetPosition(mm(*pos))
    for g, items in groups.items():
        if g.startswith(('drv', 'sense')):
            items.sort(key=lambda it: -(it[1][0] * it[1][1]))     # biggest first
        elif g != 'bulk':
            items.sort(key=lambda it: (-round(it[1][1], 1), -it[1][0]))   # tallest first: tighter rows
        pack(items, REGIONS[g], GROUP_GAP.get(g, GAP))

    # terminals carry big silkscreen labels instead; hole refs would sit off the edge
    for fp in board.GetFootprints():
        if fp.GetValue() in ('BAT+', 'BAT-', 'M3') or fp.GetValue().startswith('MOTOR_'):
            fp.Reference().SetVisible(False)
        if fp.GetValue() == 'ESDA6V1-5SC6':            # ref clashes with the neighbour's pin-1 mark
            p = fp.GetPosition()
            fp.Reference().SetPosition(pcbnew.VECTOR2I(p.x, p.y - pcbnew.FromMM(2.9)))
    unclash_refs(board)

    rect = pcbnew.PCB_SHAPE(board)
    rect.SetShape(pcbnew.SHAPE_T_RECT)
    rect.SetStart(mm(0, 0))
    rect.SetEnd(mm(W, H))
    rect.SetLayer(pcbnew.Edge_Cuts)
    rect.SetWidth(pcbnew.FromMM(0.1))
    board.Add(rect)

    # silkscreen: polarity and phase labels next to the terminals
    labels = [('48V 35A BLDC  rev C', 170, H - 1.5, 1.0),
              ('BAT+', BAT_POS['BAT+'][0] + 0.5, BAT_POS['BAT+'][1] + 7.5, 1.5),
              ('BAT-', BAT_POS['BAT-'][0] + 0.5, BAT_POS['BAT-'][1] - 7.5, 1.5)]
    labels += [(f'MOTOR {ph}', col_x(ph) + TERM_X, Y_TERM + 7.3, 1.2) for ph in 'ABC']
    for txt, x, y, size in labels:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetPosition(mm(x, y))
        t.SetTextSize(pcbnew.VECTOR2I(pcbnew.FromMM(size), pcbnew.FromMM(size)))
        t.SetTextThickness(pcbnew.FromMM(size * 0.15))
        t.SetLayer(pcbnew.F_SilkS)
        board.Add(t)

    board.Save(OUT)
    print(f'wrote {OUT}: {len(board.GetFootprints())} footprints, {len(nets)} nets')


if __name__ == '__main__':
    main()
