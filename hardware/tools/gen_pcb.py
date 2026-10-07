#!/usr/bin/env python3
"""
Create bldc48.kicad_pcb from circuit.py using KiCad's pcbnew Python API.

The power stage is placed by hand (coordinates below); small parts are packed
into regions next to the part they serve.  Nets are assigned to every pad and
each footprint is linked to its schematic symbol, so "Update PCB from
Schematic" works afterwards without losing placement.  Tracks are not routed
here: route_pcb.py routes this placement and adds planes/pours.

Board plan, rev F (mm, origin top-left, top view), 90 x 68.5 mm:

  x 0.5..62     power array: one 20.5 mm column per phase
                  +48 V bus along the top edge (high-side drain tabs)
                  high FET over low FET, drain tabs up: the switch node is the
                  short gap between them, and the motor wire pad sits right
                  beside it; the low-side Kelvin shunt under the low FET; the
                  DC-link ceramics bottom-right of each column; a 2 mm channel
                  on the left of each column carries the gate / sense traces
  x 62.5..69.5  battery wire pads (BAT+ top, BAT- bottom), TVS, two M3 holes
  x 70..89.5    three 18 x 35 mm bulk capacitors standing in a column
  y 41.8..60.5  DRV8353RS (top) with its passives, the 5 V buck, the latch;
                MCU, I/O conditioning and the LDO on the BOTTOM side here and
                beside the bulk caps (the heat plate only covers the array)
  y 61..68.5    JST-GH connector row along the bottom edge

usage: gen_pcb.py  (needs `import pcbnew`, i.e. run with KiCad's python)
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import circuit  # noqa: E402
from gen_schematic import KICAD_DIR, PROJECT_LIB, Libs, resolve_pins, uid  # noqa: E402

FPDIR = '/usr/share/kicad/footprints'
OUT = os.environ.get('BLDC_PCB') or \
    os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'kicad', 'bldc48.kicad_pcb')
W, H = 90.0, float(os.environ.get('BLDC_H', 68.5))   # board size, mm (9.6 sq in)
GAP = 1.0                    # min spacing between packed courtyards

# power-array geometry (shared with route_pcb.py)
COL0, COLW = 0.5, 20.5       # first phase column x, column pitch
POWER_X = COL0 + 3 * COLW    # 62: power array left of this
ARRAY_Y = 41.5               # power array above this (heat-plate area)
CH_W = 2.2                   # gate / sense channel on the left of each column
FET_X = CH_W + 5.3           # FET centre x inside a column
TERM_X = 16.6                # motor wire pad centre x inside a column
SHUNT_X = FET_X
Y_HI, Y_LO = 11.4, 25.6      # FET centres (drain tabs up)
Y_SHUNT = 36.8               # low-side shunt centre
Y_TERM = 20.0                # motor wire pad centre, beside the switch node
Y_CER = (31.0, 37.5)         # DC-link ceramics (two per column), bottom-right
CER_X = 17.0
BAT_X = 66.0
BAT_POS = {'BAT+': (BAT_X, 4.2), 'BAT-': (BAT_X, 38.3)}
TVS_POS = (BAT_X, 21.25)
HOLES = [(BAT_X, 12.3), (BAT_X, 30.2), (3.6, H - 3.6), (W - 3.6, H - 3.6)]
CAP_X = 79.75                # bulk capacitor centres (column of three)
CAP_YS = (9.5, 28.5, 47.5)
DRV_POS = (31.5, 51.0)
BOTTOM = ('mcu', 'io')            # groups placed on the bottom side


def col_x(ph):
    return COL0 + COLW * 'ABC'.index(ph)


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def load_fp(fpid):
    lib, name = fpid.split(':')
    libdir = os.path.join(KICAD_DIR, lib + '.pretty') if lib == PROJECT_LIB else os.path.join(FPDIR, lib + '.pretty')
    fp = pcbnew.FootprintLoad(libdir, name)
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


def role_of(c, state):
    """Placement role: an explicit power-array slot or a packing region."""
    s, ref, v, lib = c['sheet'], c['ref'], c['value'], c['lib_id']
    pins = set(n for n in c['pins'].values() if n)
    if ref.startswith('#'):
        return None
    if s == 'bridge':
        if lib == 'Device:Thermistor_NTC':
            return 'ntc'
        if v.startswith('DRV8353'):
            return 'drv_ic'
        if lib == 'Device:C_Polarized':
            return 'bulk'
        if v == '2.2u/100V':
            state['cer'] = state.get('cer', 0) + 1
            return 'ceramic' if state['cer'] <= 6 else 'drv'
        for ph in 'ABC':
            if lib.startswith('Transistor_FET') and f'GH_{ph}' in pins:
                return f'hifet{ph}'
            if lib.startswith('Transistor_FET') and f'GL_{ph}' in pins:
                return f'lofet{ph}'
            if lib == 'Device:R_Shunt' and f'SP_{ph}' in pins:
                return f'shunt{ph}'
            if ref.startswith('J') and f'SW_{ph}' in pins:
                return f'term{ph}'
        if pins & {'VBUS_SENSE', 'ISENSE_A', 'ISENSE_B', 'ISENSE_C'}:
            return 'mcu'                      # ADC-side filters / divider next to the MCU
        return 'drv'                          # DRV passives
    if s == 'power':
        if v in ('BAT+', 'BAT-'):
            return 'batterm'
        if v == 'SMCJ60CA':
            return 'tvs'
        if pins & {'BST5', 'FB5', 'RIP5', 'RT_SD', 'RCL', 'VCC_B'} and lib != 'Device:L' \
                and 'EN_RAW' not in pins:
            return 'drv'                      # tied to DRV8353 buck pins: keep them at the pins
        if pins & {'SW5'} or (v == '1u/100V') or \
                (pins <= {'+5V', 'GND'} and lib != 'Device:LED' and v != '1u'):
            return 'buck'
        if any(n in pins for n in ('LATCH_B', 'LATCH_PULL', 'HOLD_B', 'PWR_BTN', 'EN_RAW')):
            return 'latch'
        if lib == 'Device:LED' or 'PWR_LED_A' in pins:
            return 'leds'
        return 'io'                           # LDO: bottom side with the I/O parts
    if s == 'mcu':
        if ref.startswith('J'):
            return 'conn'
        if lib == 'Device:LED' or pins & {'LED_STATUS_A', 'LED_FAULT_A'}:
            return 'leds'
        return 'mcu'
    if s == 'io':
        if v == 'M3':
            return 'holes'
        return 'conn' if ref.startswith('J') else 'io'
    return 'misc'


# Packing regions: list of (x0, y0, x1, y1) filled in order.  The DRV8353 sits
# at DRV_POS; its passives pack in the ring around it.
_DX, _DY = DRV_POS
_K = 4.15 + 2.5              # DRV courtyard half-size + a fan-out ring for its 0.5 mm pins
_YD = H - 8.0                # bottom of the DRV / supply area
REGIONS = {
    'latch': [(0.5, 41.8, 15.5, _YD)],
    'drv':   [(15.5, 41.8, _DX - _K, _YD), (_DX + _K, 41.8, 47.5, _YD),
              (_DX - _K, 41.8, _DX + _K, _DY - _K), (_DX - _K, _DY + _K, _DX + _K, _YD)],
    'buck':  [(47.5, 41.8, 69.6, _YD)],
    'conn':  [(7.4, H - 7.5, 82.6, H - 0.2)],
    # bottom side (outside the heat-plate area), clear of the DRV fan-out
    'mcu':   [(40.0, 41.8, 69.6, _YD)],
    # I/O conditioning right under the connectors it serves, then the left corner
    'io':    [(7.4, H - 7.5, 82.6, H - 0.4), (0.5, 41.8, _DX - _K - 1.0, _YD),
              (70.6, 12.0, 89.5, 26.5), (70.6, 31.2, 89.5, 46.0)],
}
GROUP_GAP = {'conn': 0.6, 'drv': 1.3}


def _skyline_fit(sky, x0, x1, y1, w, h):
    """Lowest-then-leftmost spot for a w x h box on a skyline [(x, width, y)], or None."""
    best = None
    for i, (sx, _, _) in enumerate(sky):
        if sx + w > x1 + 1e-6:
            break
        top, span, j = 0.0, 0.0, i
        while span < w - 1e-6:
            top = max(top, sky[j][2])
            span += sky[j][1]
            j += 1
        if top + h <= y1 + 1e-6 and (best is None or (top, sx) < (best[0], best[1])):
            best = (top, sx)
    return best


def _skyline_add(sky, x, w, top):
    out = []
    for sx, sw, sy in sky:
        ex = sx + sw
        if ex <= x or sx >= x + w:
            out.append((sx, sw, sy))
            continue
        if sx < x:
            out.append((sx, x - sx, sy))
        if ex > x + w:
            out.append((x + w, ex - x - w, sy))
    out.append((x, w, top))
    out.sort()
    merged = []
    for seg in out:                      # merge equal-height neighbours
        if merged and abs(merged[-1][2] - seg[2]) < 1e-6 and \
                abs(merged[-1][0] + merged[-1][1] - seg[0]) < 1e-6:
            merged[-1] = (merged[-1][0], merged[-1][1] + seg[1], seg[2])
        else:
            merged.append(seg)
    sky[:] = merged


def pack(items, regions, gap=GAP, rotate=True):
    """Bottom-left skyline packing into the regions in turn (first region that fits).
    Two-pin parts and ICs may be turned 90 degrees to fill gaps; connectors are not."""
    skies = [[(x0, x1 - x0, y0)] for x0, y0, x1, y1 in regions]
    for fp, size in items:
        rots = [fp.GetOrientationDegrees()]
        if rotate and not fp.GetReference().startswith('J'):
            rots.append(rots[0] + 90)
        placed = None
        for k, (x0, y0, x1, y1) in enumerate(regions):
            for rot in rots:
                fp.SetOrientationDegrees(rot)
                w, h, cx, cy = size_of(fp)
                spot = _skyline_fit(skies[k], x0, x1, y1, w + gap, h + gap)
                if spot and (placed is None or (spot[0] + h, spot[1]) < placed[0]):
                    placed = ((spot[0] + h, spot[1]), k, rot, spot, (w, h, cx, cy))
            if placed:
                break
        if placed is None:
            print(f'warning: regions {regions} overflow at {fp.GetReference()}')
            continue
        _, k, rot, (top, x), (w, h, cx, cy) = placed
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(mm(x + (w + gap) / 2 - cx, top + (h + gap) / 2 - cy))
        _skyline_add(skies[k], x, w + gap, top + h + gap)


def place_power(groups):
    def one(role):
        return groups.pop(role)[0][0]

    for v, (x, y) in BAT_POS.items():
        fp = next(f for f, _ in groups['batterm'] if f.GetValue() == v)
        put(fp, x, y)
    groups.pop('batterm')
    put(one('tvs'), *TVS_POS, pad='1', direction='up')    # +48V end towards BAT+
    for fp, y in zip([f for f, _ in groups.pop('bulk')], CAP_YS):
        put(fp, CAP_X - 3.75, y, pad='2', direction='right')   # pad 1 (+) at the origin
    ceramics = [f for f, _ in groups.pop('ceramic')]
    gates = {}
    for ph in 'ABC':
        x = col_x(ph)
        put(one(f'hifet{ph}'), x + FET_X, Y_HI, pad='3', direction='up')   # drain tab to +48V bus
        lo = one(f'lofet{ph}')
        put(lo, x + FET_X, Y_LO, pad='3', direction='up')                  # drain tab to switch node
        put(one(f'shunt{ph}'), x + SHUNT_X, Y_SHUNT, pad='1', direction='left')
        put(one(f'term{ph}'), x + TERM_X, Y_TERM)
        for k, y in enumerate(Y_CER):                 # GND pad down, +48V pad up (via to In2)
            put(ceramics[2 * 'ABC'.index(ph) + k], x + CER_X, y, pad='2', direction='down')
        gates[ph] = x
    th = one('ntc')                                   # beside the middle low FET's drain
    th.SetOrientationDegrees(90)
    th.SetPosition(mm(col_x('B') + 14.1, Y_CER[0] + 3.0))

    # DRV8353: the rotation that puts each phase's gate/sense pins nearest its column
    drv = one('drv_ic')
    best = None
    for rot in (0, 90, 180, 270):
        drv.SetOrientationDegrees(rot)
        drv.SetPosition(mm(*DRV_POS))
        cost = 0.0
        for p in drv.Pads():
            n = p.GetNetname()
            for ph in 'ABC':
                if n in (f'GH_{ph}', f'GL_{ph}', f'SW_{ph}', f'SP_{ph}', f'SN_{ph}'):
                    cost += abs(pcbnew.ToMM(p.GetPosition().x) - (gates[ph] + CH_W / 2))
                    cost += 3.0 * max(0.0, pcbnew.ToMM(p.GetPosition().y) - DRV_POS[1])   # face up
            if n in ('SW5', 'BST5', 'FB5', 'RT_SD', 'RCL', 'VCC_B'):
                cost += 3.0 * max(0.0, DRV_POS[0] + 3.0 - pcbnew.ToMM(p.GetPosition().x))  # face right
        if best is None or cost < best[0]:
            best = (cost, rot)
    drv.SetOrientationDegrees(best[1])
    drv.SetPosition(mm(*DRV_POS))


def unclash_refs(board):
    """Move a reference off a neighbour's silkscreen, a pad or another reference
    on the same side (copper is untouched, so routing stays valid); hide it if
    no nearby spot is free."""
    edge = pcbnew.BOX2I(mm(0.5, 0.5), mm(W - 1.0, H - 1.0))
    for silk_layer, cu in ((pcbnew.F_SilkS, pcbnew.F_Cu), (pcbnew.B_SilkS, pcbnew.B_Cu)):
        # (SWIG hands out a new proxy per call, so footprints are compared by reference)
        silk = [g.GetBoundingBox() for f in board.GetFootprints() for g in f.GraphicalItems()
                if g.GetLayer() == silk_layer and g.GetClass() != 'PCB_TEXT']
        pads = [p.GetBoundingBox() for f in board.GetFootprints() for p in f.Pads() if p.IsOnLayer(cu)]
        mine = [f for f in board.GetFootprints() if f.Reference().GetLayer() == silk_layer]

        def clashes(fp):
            me, bb = fp.GetReference(), fp.Reference().GetBoundingBox()
            return (not edge.Contains(bb) or any(bb.Intersects(b) for b in pads) or
                    any(bb.Intersects(b) for b in silk) or
                    any(f.GetReference() != me and f.Reference().IsVisible() and
                        bb.Intersects(f.Reference().GetBoundingBox()) for f in mine))

        for fp in mine:
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
    state = {}
    for c in circuit.components:
        g = role_of(c, state)
        if g is None or not c['footprint']:
            continue
        fp = load_fp(c['footprint'])
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
    for (fp, _), pos in zip(holes, HOLES):
        fp.SetPosition(mm(*pos))
    # latch and LEDs share a region: pack them as one group
    groups['latch'] = groups.get('latch', []) + groups.pop('leds', [])
    for g in ['latch', 'drv', 'buck', 'conn', 'mcu', 'io']:
        items = groups.pop(g, [])
        items.sort(key=lambda it: (-round(it[1][1], 1), -it[1][0]))   # tallest first: tighter rows
        regions = REGIONS[g]
        if g == 'io':                      # skip what ldo used of the shared band
            regions = [rg for rg in regions]
        pack(items, regions, GROUP_GAP.get(g, GAP))
        if g in BOTTOM:
            for fp, (w, h, cx, cy) in items:
                p = fp.GetPosition()
                fp.Flip(p, True)                       # mirror left/right onto B.Cu
                fp.SetPosition(pcbnew.VECTOR2I(p.x + pcbnew.FromMM(2 * cx), p.y))
    if groups:
        raise SystemExit(f'unplaced groups: {sorted(groups)}')

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
    labels = [('BLDC48 rev F', 45.0, H - 0.9, 0.8),
              ('+', BAT_POS['BAT+'][0] - 4.6, BAT_POS['BAT+'][1], 2.0),
              ('-', BAT_POS['BAT-'][0] - 4.6, BAT_POS['BAT-'][1], 2.0)]
    labels += [(ph, col_x(ph) + TERM_X, Y_TERM - 4.4, 1.6) for ph in 'ABC']
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
