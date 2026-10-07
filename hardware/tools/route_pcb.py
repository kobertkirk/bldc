#!/usr/bin/env python3
"""
Autoroute bldc48.kicad_pcb with Freerouting and add the high-current copper.

  1. export Specctra DSN and route every net on all four layers with
     Freerouting (headless).  Planes are NOT exported: Freerouting treats a
     plane of another net as an obstacle, which blocks almost every via.
  2. read the SES back (KiCad 7 can only import SES from the GUI, so this
     script parses it itself)
  3. a rule area keeps tracks off the inner layers under the power stage, so
     add the planes - In1.Cu = GND, In2.Cu = +48V (power stage) / +3V3
     (logic) - and outer-layer pours for the 35 A paths: +48V bus, each switch node
     SW_x, each phase output PHASE_x, GND around the low-side FETs, plus a
     GND fill on both outer layers everywhere else
  4. fill zones, save, write a DRC report

usage: route_pcb.py --freerouting /path/freerouting.jar [--passes 40]
"""
import argparse
import os
import re
import subprocess
import sys

import pcbnew

# lets DRC check footprints against the stock libraries via ../kicad/fp-lib-table
os.environ.setdefault('KICAD7_FOOTPRINT_DIR', '/usr/share/kicad/footprints')

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sexp import find, find1, parse  # noqa: E402
import gen_pcb  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.join(HERE, '..', 'kicad', 'bldc48.kicad_pcb')
from gen_pcb import COLW, POWER_X, H, W, Y_HI, Y_LO, col_x  # noqa: E402  (board geometry)

KEEPOUT_Y = Y_LO + 23.0     # inner-layer track keepout ends below the shunts/INA240s


def mm(x, y):
    return pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))


def add_zone(board, net, layer, pts, priority=0, clearance=0.3, min_w=0.25, thermal=True,
             solid=False):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(board.FindNet(net))
    outline = z.Outline()
    outline.NewOutline()
    for x, y in pts:
        outline.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(pcbnew.FromMM(clearance))
    z.SetMinThickness(pcbnew.FromMM(min_w))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(pcbnew.FromMM(0.4))
    z.SetThermalReliefSpokeWidth(pcbnew.FromMM(0.6))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetIsFilled(False)
    board.Add(z)
    return z


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def plane_keepout(board):
    """No tracks on In1/In2 under the FETs, DC-link caps and shunts: the GND and
    +48V planes there carry the DC-link and switching currents and must stay
    unbroken.  Vias may pass through.  Signals may still use the inner layers
    under the controller and in a channel along the bottom of the power stage
    (below KEEPOUT_Y), where the planes only carry logic return current."""
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(False)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowCopperPour(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    ls.AddLayer(pcbnew.In1_Cu)
    ls.AddLayer(pcbnew.In2_Cu)
    z.SetLayerSet(ls)
    z.SetZoneName('power-stage plane keepout')
    o = z.Outline()
    o.NewOutline()
    for x, y in rect(0.2, 0.2, POWER_X + 1.0, KEEPOUT_Y):
        o.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
    board.Add(z)


def inner_planes(board):
    # solid pad connections: the M5 power terminals and FET via arrays must not
    # be throttled by thermal spokes
    add_zone(board, 'GND', pcbnew.In1_Cu, rect(0.5, 0.5, W - 0.5, H - 0.5), solid=True)
    add_zone(board, '+48V', pcbnew.In2_Cu, rect(0.5, 0.5, POWER_X - 0.5, H - 0.5), solid=True)
    add_zone(board, '+3V3', pcbnew.In2_Cu, rect(POWER_X + 0.5, 0.5, W - 0.5, H - 0.5), solid=True)


def outer_pours(board, hv_clear=0.5):
    """High-current copper on both outer layers (geometry from gen_pcb.py).

    Per phase column at x = X:
      SW_x     high-side source leads + the whole low-side drain tab, out to
               the shunt's current pad                  (priority 5)
      PHASE_x  shunt current pad down to the motor terminal (priority 5)
      GND      low-side source leads + DC-link ceramics  (priority 4)
    +48V bus along the top: battery terminal, bulk caps, high-side drain tabs
    (priority 3); GND fills the rest (priority 1).  48-60 V pours keep 0.5 mm
    from other nets.
    """
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        for ph in 'ABC':
            x = col_x(ph)
            add_zone(board, f'SW_{ph}', layer, rect(x + 14.5, Y_HI + 3.0, x + 33.0, Y_LO + 2.0),
                     priority=5, clearance=hv_clear, solid=True)
            add_zone(board, f'PHASE_{ph}', layer, rect(x + 36.2, Y_HI + 3.5, x + COLW - 0.4, H - 0.6),
                     priority=5, clearance=hv_clear, solid=True)
            add_zone(board, 'GND', layer, rect(x + 13.5, Y_LO + 3.3, x + 27.0, Y_LO + 16.0),
                     priority=4, solid=True)
        add_zone(board, '+48V', layer, rect(0.6, 0.6, POWER_X - 0.5, Y_HI + 2.3),
                 priority=3, clearance=hv_clear, solid=True)
        add_zone(board, '+48V', layer, rect(0.6, 0.6, 21.0, 30.5),
                 priority=3, clearance=hv_clear, solid=True)
        add_zone(board, 'GND', layer, rect(0.5, 0.5, W - 0.5, H - 0.5), priority=1, solid=True)


POWER_PAD_NETS = ('GND', '+48V', 'SW_', 'PHASE_')


def via_arrays(board, pitch=1.3, inset=0.65, dia=0.6, drill=0.3):
    """Via arrays in the big power pads (FET tabs/sources, shunts, motor pads)
    and a via in each DC-link ceramic pad: ties F.Cu/B.Cu pours and the inner
    planes together for current sharing and heat spreading."""
    import math
    segs, holes = [], []
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':
            c = t.GetPosition()
            holes.append((pcbnew.ToMM(c.x), pcbnew.ToMM(c.y), pcbnew.ToMM(t.GetWidth()) / 2, t.GetNetCode()))
        else:
            a, b = t.GetStart(), t.GetEnd()
            segs.append((pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y),
                         pcbnew.ToMM(t.GetWidth()) / 2, t.GetNetCode()))

    def seg_dist(px, py, x0, y0, x1, y1):
        dx, dy = x1 - x0, y1 - y0
        L = dx * dx + dy * dy
        u = 0 if L == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L))
        return math.hypot(px - x0 - u * dx, py - y0 - u * dy)

    def free(x, y, netcode):
        rv = dia / 2
        for hx, hy, hr, hn in holes:
            if math.hypot(x - hx, y - hy) < rv + hr + 0.3:
                return False
        for x0, y0, x1, y1, hw, tn in segs:
            if tn != netcode and seg_dist(x, y, x0, y0, x1, y1) < rv + hw + 0.3:
                return False
        return True

    n = 0
    for fp in board.GetFootprints():
        is_link_cap = fp.GetValue() == '2.2u/100V'
        for pad in fp.Pads():
            net = pad.GetNetname()
            if pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or not net.startswith(POWER_PAD_NETS):
                continue
            if pcbnew.ToMM(pad.GetPosition().x) > POWER_X:      # power stage only
                continue
            bb = pad.GetBoundingBox()
            w, h = pcbnew.ToMM(bb.GetWidth()), pcbnew.ToMM(bb.GetHeight())
            x0, y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
            if is_link_cap:
                pts = [(x0 + w / 2, y0 + h / 2)]
            elif w * h >= 6.0:
                nx, ny = max(1, int((w - 2 * inset) / pitch) + 1), max(1, int((h - 2 * inset) / pitch) + 1)
                ox = x0 + (w - (nx - 1) * pitch) / 2
                oy = y0 + (h - (ny - 1) * pitch) / 2
                pts = [(ox + i * pitch, oy + j * pitch) for i in range(nx) for j in range(ny)]
            else:
                continue
            r = pcbnew.FromMM(dia / 2 + 0.05)
            for x, y in pts:
                c = mm(x, y)
                # whole via must sit on this pad's copper
                if not all(pad.HitTest(pcbnew.VECTOR2I(c.x + dx, c.y + dy))
                           for dx, dy in ((r, 0), (-r, 0), (0, r), (0, -r))):
                    continue
                if not free(x, y, pad.GetNetCode()):
                    continue
                holes.append((x, y, dia / 2, pad.GetNetCode()))
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(c)
                v.SetWidth(pcbnew.FromMM(dia))
                v.SetDrill(pcbnew.FromMM(drill))
                v.SetNet(pad.GetNet())
                board.Add(v)
                n += 1
    return n


def fill(board):
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())


# ---------------------------------------------------------------- Specctra
# Hand routes for connections Freerouting 1.9 leaves open on this placement.
# Each is only added if no track of the net already reaches the pad, and was
# verified with a full zone refill + DRC (0 violations).
#   (net, (ref, pad), [segments]); segment = ('F'|'B', [points]) or ('V', point)
HAND_ROUTES = [
    ('HB_B', ('U6', '8'), [            # bootstrap cap C39 -> driver HB pin, over the driver
        ('F', [(70.25, 28.475), (70.25, 29.9)]), ('V', (70.25, 29.9)),
        ('B', [(70.25, 29.9), (77.475, 29.9), (77.475, 34.0)]), ('V', (77.475, 34.0)),
        ('F', [(77.475, 34.0), (77.475, 35.6)]),
    ]),
]


def hand_routes(board):
    added = 0
    for netname, (ref, padnum), segs in HAND_ROUTES:
        ni = board.FindNet(netname)
        pad = next(p for p in board.FindFootprintByReference(ref).Pads() if p.GetNumber() == padnum)
        reached = any(t.GetNetCode() == ni.GetNetCode() and t.GetClass() == 'PCB_TRACK' and
                      (pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd())) for t in board.GetTracks())
        if reached:
            continue
        for kind, data in segs:
            if kind == 'V':
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(mm(*data))
                v.SetWidth(pcbnew.FromMM(0.6))
                v.SetDrill(pcbnew.FromMM(0.3))
                v.SetNet(ni)
                board.Add(v)
            else:
                for a, b in zip(data, data[1:]):
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(mm(*a))
                    t.SetEnd(mm(*b))
                    t.SetWidth(pcbnew.FromMM(0.25))
                    t.SetLayer(pcbnew.F_Cu if kind == 'F' else pcbnew.B_Cu)
                    t.SetNet(ni)
                    board.Add(t)
        added += 1
    return added


def through_hole_at(board, q, ni):
    pos = mm(*q)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == ni.GetNetCode() and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH \
                    and p.HitTest(pos):
                return True
    return False


def import_ses(board, ses_path):
    root = parse(open(ses_path).read())
    routes = find1(root, 'routes')
    res = find1(routes, 'resolution')
    unit, per = str(res[1]), float(res[2])
    scale = {'um': 1e-3, 'mm': 1.0, 'mil': 0.0254, 'inch': 25.4}[unit] / per   # -> mm
    lib = find1(routes, 'library_out')
    via_sizes = {}
    if lib:
        for ps in find(lib, 'padstack'):
            name = ps[1]
            shp = find1(ps, 'shape')
            circ = find1(shp, 'circle') if shp else None
            dia = float(circ[2]) * scale if circ else 0.6
            mo = re.search(r':(\d+)_um', name)
            drill = int(mo.group(1)) / 1000 if mo else 0.3
            via_sizes[name] = (dia, drill)
    layers = {board.GetLayerName(l): l for l in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu)}
    n_tracks = n_vias = n_fixed = 0
    default_via = next(iter(via_sizes.values()), (0.6, 0.3))

    def add_via(ni, x, y, size):
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(mm(x, y))
        v.SetWidth(pcbnew.FromMM(size[0]))
        v.SetDrill(pcbnew.FromMM(size[1]))
        v.SetNet(ni)
        board.Add(v)

    for net in find(find1(routes, 'network_out'), 'net'):
        ni = board.FindNet(net[1])
        ends = {}                                  # point -> set of layers
        for wire in find(net, 'wire'):
            path = find1(wire, 'path')
            layer = layers[path[1]]
            width = float(path[2]) * scale
            coords = [float(v) * scale for v in path[3:]]
            pts = [(round(coords[i], 4), round(-coords[i + 1], 4)) for i in range(0, len(coords), 2)]
            if len(pts) < 2:
                continue                           # zero-length stub
            for q in (pts[0], pts[-1]):
                ends.setdefault(q, set()).add(layer)
            for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(mm(x0, y0))
                t.SetEnd(mm(x1, y1))
                t.SetWidth(pcbnew.FromMM(width))
                t.SetLayer(layer)
                t.SetNet(ni)
                board.Add(t)
                n_tracks += 1
        vias = set()
        for via in find(net, 'via'):
            q = (round(float(via[2]) * scale, 4), round(-float(via[3]) * scale, 4))
            vias.add(q)
            add_via(ni, q[0], q[1], via_sizes.get(via[1], default_via))
            n_vias += 1
        # some Freerouting builds drop vias from the SES: re-insert one wherever
        # the route changes layer and there is no via (or through-hole pad)
        for q, ls in ends.items():
            if len(ls) > 1 and q not in vias and not through_hole_at(board, q, ni):
                add_via(ni, q[0], q[1], default_via)
                n_fixed += 1
    if n_fixed:
        print(f'added {n_fixed} vias missing from the SES')
    return n_tracks, n_vias


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--freerouting')
    ap.add_argument('--passes', type=int, default=40)
    ap.add_argument('--workdir', default='/tmp/bldc48-route')
    ap.add_argument('--legacy', action='store_true',
                    help='Freerouting 1.x command line (needs a display: runs under xvfb-run)')
    ap.add_argument('--ses', help='skip routing and import this SES file')
    ap.add_argument('--no-optimize', action='store_true',
                    help='skip Freerouting route optimisation (1.9 can hang in it)')
    args = ap.parse_args()
    os.makedirs(args.workdir, exist_ok=True)
    dsn = os.path.join(args.workdir, 'bldc48.dsn')
    ses = os.path.join(args.workdir, 'bldc48.ses')

    gen_pcb.main()                       # fresh, deterministic placement
    board = pcbnew.LoadBoard(PCB)
    ds = board.GetDesignSettings()
    ds.m_MinThroughDrill = pcbnew.FromMM(0.2)
    ds.SetCustomViaSize(True)
    plane_keepout(board)
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        raise SystemExit('DSN export failed')
    print('DSN written')

    if args.ses:
        ses = args.ses
    else:
        if os.path.exists(ses):
            os.remove(ses)
        if args.legacy:
            cmd = ['xvfb-run', '-a', 'java', '-jar', args.freerouting, '-de', dsn, '-do', ses,
                   '-mp', str(args.passes)]
            if args.no_optimize:
                cmd += ['-mt', '0']
        else:
            cmd = ['java', '-jar', args.freerouting, '--gui.enabled=false', '-de', dsn, '-do', ses,
                   f'--router.max_passes={args.passes}']
        print(' '.join(cmd))
        subprocess.run(cmd, check=False)
    if not os.path.exists(ses):
        raise SystemExit('freerouting produced no SES')

    nt, nv = import_ses(board, ses)
    print(f'imported {nt} track segments, {nv} vias')
    print(f'added {hand_routes(board)} hand route(s)')
    print(f'added {via_arrays(board)} power vias')
    inner_planes(board)
    outer_pours(board)
    fill(board)
    board.Save(PCB)
    rpt = os.path.join(args.workdir, 'drc.rpt')
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    print(f'saved {PCB}; DRC report {rpt}')
    for line in open(rpt):
        if line.startswith('** Found'):
            print(line.strip())


if __name__ == '__main__':
    main()
