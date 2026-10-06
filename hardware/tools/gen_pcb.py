#!/usr/bin/env python3
"""
Create bldc48.kicad_pcb from circuit.py using KiCad's pcbnew Python API.

Footprints are placed by function (power stage on the left two thirds, logic
and connectors on the right), nets are assigned to every pad and each
footprint is linked to its schematic symbol, so "Update PCB from Schematic"
works afterwards without losing placement.  Tracks are not routed here:
route_pcb.py routes this placement and adds planes/pours.

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
W, H = 190.0, 120.0          # board size, mm
GAP = 2.0                    # min spacing between courtyards (room for fan-out vias)


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


_ina_caps = []


def group_of(c):
    """Assign each part to a placement region."""
    s, ref, v = c['sheet'], c['ref'], c['value']
    pins = set(n for n in c['pins'].values() if n)
    if ref.startswith('#'):
        return None
    if s == 'bridge':
        for ph in 'ABC':
            if any(n.endswith('_' + ph) for n in pins):
                if c['lib_id'].startswith('Transistor_FET'):
                    return f'hifet{ph}' if '+48V' in pins else f'lofet{ph}'
                if v == '0.5m':
                    return f'shunt{ph}'
                if ref.startswith('J'):
                    return f'pad{ph}'
                if 'INA240' in v or any(n.startswith(('ISO_', 'ISENSE_')) for n in pins):
                    return f'sense{ph}'
                return f'drv{ph}'
        if v == '100n':                       # INA240 decouplers (only +3V3/GND pins)
            return 'senseA' if not _ina_caps.append(1) and len(_ina_caps) == 1 else \
                'senseB' if len(_ina_caps) == 2 else 'senseC'
        if 'u/100V' in v and c['lib_id'] == 'Device:C_Polarized':
            return 'bulk'
        if v == '2.2u/100V':
            return 'ceramic'
        return 'sense' 
        if 'u/100V' in v and c['lib_id'] == 'Device:C_Polarized':
            return 'bulk'
        if v == '2.2u/100V':
            return 'ceramic'
        return 'sense'
    if s == 'power':
        if v in ('BAT+', 'BAT-', 'SMCJ60CA'):
            return 'battery'
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
        if ref.startswith('J'):
            return 'conn'
        return 'io'
    return 'misc'


# Placement regions: (x0, y0, x1, y1) in mm
REGIONS = {
    'bulk':    (10, 2, 112, 18),
    'ceramic': (26, 18, 112, 26),
    'battery': (2, 20, 24, 76),
    'sense':   (2, 78, 24, 110),
    'buck12':  (120, 2, 160, 34),
    'buck5':   (120, 36, 160, 68),
    'ldo':     (162, 7, 184, 24),
    'latch':   (162, 25, 188, 46),
    'mcu':     (120, 70, 155, 117),
    'io':      (157, 48, 188, 78),
    'conn':    (157, 80, 183, 117),
    'holes':   None,
}
for _i, _ph in enumerate('ABC'):
    _x = 26 + 29 * _i
    REGIONS.update({
        f'hifet{_ph}': (_x, 27, _x + 15, 45),
        f'lofet{_ph}': (_x, 46, _x + 15, 64),
        f'shunt{_ph}': (_x, 65, _x + 15, 74),
        f'pad{_ph}':   (_x, 100, _x + 15, 117),
        f'drv{_ph}':   (_x + 15, 27, _x + 28.5, 64),
        f'sense{_ph}': (_x + 15, 66, _x + 28.5, 98),
    })


def pack(fps, region):
    x0, y0, x1, y1 = region
    x, y, row = x0, y0, 0.0
    for fp, (w, h, cx, cy) in fps:
        w += GAP
        h += GAP
        if x + w > x1 and x > x0:
            x, y, row = x0, y + row, 0.0
        if y + h > y1:
            print(f'warning: region {region} overflow at {fp.GetReference()}')
        fp.SetPosition(mm(x + w / 2 - cx, y + h / 2 - cy))
        x += w
        row = max(row, h)


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
    root = uid('root')
    for c in circuit.components:
        g = group_of(c)
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

    # bulk caps etc. pack left-to-right; phases pack top-down
    for g, items in groups.items():
        if g == 'holes':
            for fp, pos in zip([f for f, _ in items], [(4, 4), (W - 4, 4), (4, H - 4), (W - 4, H - 4)]):
                fp.SetPosition(mm(*pos))
            continue
        if g.startswith(('drv', 'sense')):
            items.sort(key=lambda it: -(it[1][0] * it[1][1]))     # IC first
        pack(items, REGIONS[g])

    # move mounting-hole-clashing bulk region a little: holes sit in corners
    # board outline
    rect = pcbnew.PCB_SHAPE(board)
    rect.SetShape(pcbnew.SHAPE_T_RECT)
    rect.SetStart(mm(0, 0))
    rect.SetEnd(mm(W, H))
    rect.SetLayer(pcbnew.Edge_Cuts)
    rect.SetWidth(pcbnew.FromMM(0.1))
    board.Add(rect)

    # silkscreen notes
    for txt, x, y in [('48V 35A BLDC  rev A', 140, 116), ('BAT+ / BAT-', 12, 77)]:
        t = pcbnew.PCB_TEXT(board)
        t.SetText(txt)
        t.SetPosition(mm(x, y))
        t.SetLayer(pcbnew.F_SilkS)
        board.Add(t)

    board.Save(OUT)
    print(f'wrote {OUT}: {len(board.GetFootprints())} footprints, {len(nets)} nets')


if __name__ == '__main__':
    main()
