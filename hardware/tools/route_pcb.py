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
     SW_x (with the motor wire pad), each low-side source LS_x, GND around the shunts, plus a
     GND fill on both outer layers everywhere else
  4. fill zones, save, write a DRC report

usage: route_pcb.py --freerouting /path/freerouting.jar [--passes 40]
"""
import argparse
import math
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
PCB = os.environ.get('BLDC_PCB') or os.path.join(HERE, '..', 'kicad', 'bldc48.kicad_pcb')
from gen_pcb import (ARRAY_Y, CAP_X, CAP_YS, CH_W, COLW, H, POWER_X, TVS_POS, W, Y_HI, Y_LO,  # noqa: E402
                     col_x)  # board geometry

KEEPOUT_Y = ARRAY_Y          # inner-layer track keepout: the power array and its DC path
PLUS_X = CAP_X               # +48 V side of the bulk-cap column (pads 1 at CAP_X - 3.75)
SW_Y0, SW_Y1 = Y_HI + 2.5, Y_LO + 0.7      # switch-node band between the FETs
LS_Y0, LS_Y1 = Y_LO + 2.5, Y_LO + 7.0      # low-side source leads
OPEN_X = 5.5                 # left end of each column left open for the DRV sense / gate taps


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


def plane_keepout(board, solid_in1=False):
    """In1 is a solid GND plane: no tracks anywhere.  In2 carries +48 V under the
    power array, the battery strip and the + side of the bulk caps (no tracks
    there either); elsewhere In2 is a signal layer for the controller."""
    def area(layer, pts, name):
        z = pcbnew.ZONE(board)
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(False)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowCopperPour(False)
        z.SetDoNotAllowFootprints(False)
        ls = pcbnew.LSET()
        ls.AddLayer(layer)
        z.SetLayerSet(ls)
        z.SetZoneName(name)
        o = z.Outline()
        o.NewOutline()
        for x, y in pts:
            o.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        board.Add(z)

    if solid_in1:      # added only after routing: Freerouting's wire keep-outs also stop vias
        area(pcbnew.In1_Cu, rect(0.2, 0.2, W - 0.2, H - 0.2), 'solid GND plane')
        return
    area(pcbnew.In2_Cu, [(0.2, 0.2), (PLUS_X, 0.2), (PLUS_X, CAP_YS[-1] + 9.4), (POWER_X + 7.8, CAP_YS[-1] + 9.4),
                         (POWER_X + 7.8, KEEPOUT_Y), (0.2, KEEPOUT_Y)], '+48V plane under the power array')


def phase_keepout(board):
    """No tracks on the outer layers across a switch node (high-side source ->
    low-side drain -> motor wire) or a low-side source band (-> shunt): a
    signal crossing one splits the pour, and that layer then carries none of
    the motor current.  The left end of each column stays open so the DRV8353
    gate and SHx sense traces can reach the FETs."""
    ls = pcbnew.LSET()
    ls.AddLayer(pcbnew.F_Cu)          # B.Cu stays open for the gate / sense traces
    for ph in 'ABC':
        x = col_x(ph)
        for x0, y0, x1, y1 in ((x + OPEN_X, SW_Y0, x + COLW - 0.4, SW_Y1),
                               (x + OPEN_X, LS_Y0, x + 12.9, LS_Y1)):
            z = pcbnew.ZONE(board)
            z.SetIsRuleArea(True)
            z.SetDoNotAllowTracks(True)
            z.SetDoNotAllowVias(False)
            z.SetDoNotAllowPads(False)
            z.SetDoNotAllowCopperPour(False)
            z.SetDoNotAllowFootprints(False)
            z.SetLayerSet(ls)
            z.SetZoneName(f'phase {ph} power copper keepout')
            o = z.Outline()
            o.NewOutline()
            for px, py in rect(x0, y0, x1, y1):
                o.Append(pcbnew.FromMM(px), pcbnew.FromMM(py))
            board.Add(z)


PLANE_NETS = ('GND',)            # reaches the solid In1 plane through fan-out vias


def fanout(board, dist=(0.9, 1.25, 1.6, 2.0, 2.5, 3.0), via_d=0.6, drill=0.3, stub_w=0.3):
    """Give every GND / +3V3 SMD pad outside the power array its own via into
    the solid In1 GND plane, with a short stub.  The autorouter
    then never has to draw these nets, which is most of the congestion on a
    small two-sided board.  Candidate spots go outward from the part first."""
    clear, hole_clear = 0.22, 0.27
    pads = [p for f in board.GetFootprints() for p in f.Pads()]
    rule_areas = [z for z in board.Zones() if z.GetIsRuleArea() and z.GetDoNotAllowVias()]
    vias = []

    def ok_via(x, y, code):
        q = mm(x, y)
        r = pcbnew.FromMM(via_d / 2 + clear)
        if x < 1.3 or y < 1.3 or x > W - 1.3 or y > H - 1.3:
            return False
        if any(z.Outline().Contains(q) for z in rule_areas):
            return False
        for p in pads:
            if p.HitTest(q, r + (pcbnew.FromMM(hole_clear - clear) if p.GetDrillSize().x else 0)):
                return False                       # no via in any pad, own net included
        return all(math.hypot(x - vx, y - vy) >= via_d + clear for vx, vy in vias)

    def ok_stub(a, b, layer, code):
        n = max(2, int(math.dist(a, b) / 0.1))
        acc = pcbnew.FromMM(stub_w / 2 + clear)
        for k in range(1, n):
            q = mm(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n)
            for p in pads:
                if p.GetNetCode() != code and p.IsOnLayer(layer) and p.HitTest(q, acc):
                    return False
        return True

    added = 0
    for fp in board.GetFootprints():
        c = fp.GetPosition()
        cx, cy = pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)
        plated = {p.GetNumber() for p in fp.Pads() if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH}
        for pad in fp.Pads():
            if pad.GetNetname() not in PLANE_NETS or pad.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            if pad.GetNumber() in plated:           # e.g. the DRV8353 thermal pad
                continue
            px, py = pcbnew.ToMM(pad.GetPosition().x), pcbnew.ToMM(pad.GetPosition().y)
            if px < POWER_X + 8.0 and py < ARRAY_Y:  # the power array has pours + via arrays
                continue
            layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            ang0 = math.atan2(py - cy, px - cx) if math.hypot(px - cx, py - cy) > 0.2 else 0.0
            done = False
            for d in dist:
                for da in (0, 45, -45, 90, -90, 135, -135, 180):
                    a = ang0 + math.radians(da)
                    vx, vy = round(px + d * math.cos(a), 3), round(py + d * math.sin(a), 3)
                    if ok_via(vx, vy, pad.GetNetCode()) and ok_stub((px, py), (vx, vy), layer,
                                                                   pad.GetNetCode()):
                        t = pcbnew.PCB_TRACK(board)
                        t.SetStart(mm(px, py))
                        t.SetEnd(mm(vx, vy))
                        t.SetWidth(pcbnew.FromMM(stub_w))
                        t.SetLayer(layer)
                        t.SetNet(pad.GetNet())
                        board.Add(t)
                        v = pcbnew.PCB_VIA(board)
                        v.SetPosition(mm(vx, vy))
                        v.SetWidth(pcbnew.FromMM(via_d))
                        v.SetDrill(pcbnew.FromMM(drill))
                        v.SetNet(pad.GetNet())
                        board.Add(v)
                        vias.append((vx, vy))
                        added += 1
                        done = True
                        break
                if done:
                    break
            if not done:
                print(f'  no fan-out spot for {fp.GetReference()} pad {pad.GetNumber()} ({pad.GetNetname()})')
    return added


def four_layers():
    ls = pcbnew.LSET()
    for layer in (pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu):
        ls.AddLayer(layer)
    return ls


def edge_keepout(board, margin=1.0):
    """Freerouting does not know KiCad's board-edge clearance: keep tracks 1 mm
    in.  Four strips, because a ring (outline with a hole) loses its hole in
    the Specctra export and would block the whole board."""
    for x0, y0, x1, y1 in ((0, 0, W, margin), (0, H - margin, W, H),
                           (0, 0, margin, H), (W - margin, 0, W, H)):
        z = pcbnew.ZONE(board)
        z.SetIsRuleArea(True)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(False)
        z.SetDoNotAllowCopperPour(False)
        z.SetDoNotAllowFootprints(False)
        z.SetLayerSet(four_layers())
        z.SetZoneName('board-edge track keepout')
        o = z.Outline()
        o.NewOutline()
        for x, y in rect(x0, y0, x1, y1):
            o.Append(pcbnew.FromMM(x), pcbnew.FromMM(y))
        board.Add(z)


def patch_dsn(path, board, clearance_um=210):
    """Give the router a little clearance margin over KiCad's 0.2 mm rule
    (Specctra coordinates are rounded on the way back), and leave out the
    connections the pours make inside the power keep-outs: the low-side
    source nets (FET source -> shunt) and the motor wire pads on the switch
    nodes.  The router still joins each switch node to the DRV8353 SHx pin."""
    txt = open(path).read()
    txt = re.sub(r'\(clearance 200\.1\)', f'(clearance {clearance_um})', txt)
    # In1 is the solid GND plane: a power layer for the router (vias pass, no wires)
    txt = re.sub(r'(\(layer In1\.Cu\s*\(type )signal\)', r'\1power)', txt)
    txt = re.sub(r'\n\s*\(net LS_[ABC]\n\s*\(pins [^)]*\)\n\s*\)', '', txt)
    txt = re.sub(r' LS_[ABC](?=[\s)])', '', txt)
    for pn in PLANE_NETS:          # nothing left to route: the fan-out vias reach the planes
        txt = re.sub(r'(\(net %s\n\s*\(pins )(\S+)[^)]*\)' % re.escape(pn), r'\1\2)', txt)
        # and the router must leave the fan-out stubs and vias where they are
        txt = txt.replace(f'(net {pn})(type route)', f'(net {pn})(type fix)')
    motor = [fp.GetReference() for fp in board.GetFootprints() if fp.GetValue().startswith('MOTOR_')]
    for ref in motor:
        txt = re.sub(r' %s-[0-9@]+(?=[\s)])' % re.escape(ref), '', txt)
    open(path, 'w').write(txt)


def inner_planes(board):
    """In1: GND everywhere.  In2: +48 V under the power array, the battery strip
    and the + side of the bulk caps; elsewhere it is a signal layer, and a
    +3.3 V fill takes the space the signals leave.  Solid
    pad connections: the wire pads and FET via arrays must not be throttled."""
    add_zone(board, 'GND', pcbnew.In1_Cu, rect(0.5, 0.5, W - 0.5, H - 0.5), solid=True)
    plus = [(0.5, 0.5), (PLUS_X, 0.5), (PLUS_X, CAP_YS[-1] + 9.4), (POWER_X + 7.8, CAP_YS[-1] + 9.4),
            (POWER_X + 7.8, ARRAY_Y), (0.5, ARRAY_Y)]
    add_zone(board, '+48V', pcbnew.In2_Cu, plus, priority=2, clearance=0.5, solid=True)
    add_zone(board, '+3V3', pcbnew.In2_Cu, rect(0.5, 0.5, W - 0.5, H - 0.5), priority=1)   # fills what is left


def outer_pours(board, hv_clear=0.5):
    """High-current copper on both outer layers (geometry from gen_pcb.py).

    Per phase column at x = X:
      SW_x   high-side source leads + low-side drain tab + motor wire pad (5)
      LS_x   low-side source leads down to the shunt's current pad       (6)
      GND    shunt's GND pad, DC-link ceramics, rest of the column       (4)
    +48V bus along the top edge, the battery strip's upper half and the + side
    of the bulk caps (3); GND on the strip's lower half and the caps' - side (3);
    GND fills the rest (1).  48-60 V pours keep 0.5 mm from other nets.
    """
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        for ph in 'ABC':
            x = col_x(ph)
            add_zone(board, f'SW_{ph}', layer, rect(x + CH_W - 0.3, SW_Y0, x + COLW - 0.4, SW_Y1),
                     priority=5, clearance=hv_clear, solid=True)
            ls = [(x + CH_W - 0.3, LS_Y0), (x + 12.9, LS_Y0), (x + 12.9, LS_Y1), (x + 5.2, LS_Y1),
                  (x + 5.2, ARRAY_Y - 2.3), (x + CH_W - 0.3, ARRAY_Y - 2.3)]
            add_zone(board, f'LS_{ph}', layer, ls, priority=6, clearance=hv_clear, solid=True)
            gnd = [(x + 13.3, SW_Y1 + 0.6), (x + COLW - 0.4, SW_Y1 + 0.6), (x + COLW - 0.4, ARRAY_Y),
                   (x + 5.7, ARRAY_Y), (x + 5.7, LS_Y1 + 0.4), (x + 13.3, LS_Y1 + 0.4)]
            add_zone(board, 'GND', layer, gnd, priority=4, solid=True)
        add_zone(board, '+48V', layer, rect(0.6, 0.6, POWER_X + 7.6, Y_HI + 1.0),
                 priority=3, clearance=hv_clear, solid=True)
        add_zone(board, '+48V', layer, rect(POWER_X + 0.4, 0.6, POWER_X + 7.6, TVS_POS[1]),
                 priority=3, clearance=hv_clear, solid=True)
        add_zone(board, '+48V', layer, rect(POWER_X + 7.9, 0.6, PLUS_X, CAP_YS[-1] + 9.3),
                 priority=3, clearance=hv_clear, solid=True)
        add_zone(board, 'GND', layer, rect(POWER_X + 0.4, TVS_POS[1], POWER_X + 7.6, ARRAY_Y),
                 priority=3, solid=True)
        add_zone(board, 'GND', layer, rect(0.5, 0.5, W - 0.5, H - 0.5), priority=1, solid=True)


POWER_PAD_NETS = ('GND', '+48V', 'SW_', 'LS_')


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
# nets whose Freerouting wiring is thrown away on import (bad detours), re-made by HAND_ROUTES
RIP_UP = {          # {sha1 of the SES file: {net, ...}}
    '604e5ab3ee39b391a8f89548bc6738a9e444be09': {'GL_C'},    # 139 mm loop for a gate drive
}

HAND_ROUTES = {     # {sha1 of the SES file: [routes]}, filled per routing session
    # rev E: the INA240 C supply tap the router left open (astar_route.py)
    '604e5ab3ee39b391a8f89548bc6738a9e444be09': [
        ('+3V3', None, [('F', [(121.475, 48.865), (120.75, 50.4)]), ('V', (120.75, 50.4)),
                        ('B', [(120.75, 50.4), (119.75, 53.4), (119.25, 62.4), (118.5, 63.4), (118.5, 64.4),
                               (127.75, 65.65), (133.25, 64.9)]), ('V', (133.25, 64.9))]),   # via into In2 +3V3
    ],
    # rev C: LATCH_B (Q1 base) is boxed in by inner-layer tracks; go over Q1 pin 3 on F.Cu
    '3ea9f4de44a089cb82b0a6ff3ac9a04d7fcae6f2': [
        ('LATCH_B', ('Q1', '1'), [('F', [(137.73, 55.0), (138.3, 54.4), (141.25, 54.4), (141.25, 59.88)])]),
    ],
    # rev D: the router left the INA240 B/C supply taps open (paths found by a grid search)
    'fb745afb3a85808af48dd11c94372f2fe518c995': [
        ('+3V3', None, [('V', (78.25, 50.48)), ('B', [(78.25, 50.48), (77.2, 57.6), (77.45, 58.09)])]),
        # phase C: under its motor terminal on B.Cu to a via into the +3V3 plane (In2, x > POWER_X)
        ('+3V3', None, [('V', (116.25, 50.48)), ('B', [(116.25, 50.48), (120.8, 58.8), (122.4, 60.4),
                                                       (132.8, 64.6)]), ('V', (132.8, 64.6))]),
    ],
}


def hand_routes(board, ses_path):
    import hashlib
    digest = hashlib.sha1(open(ses_path, 'rb').read()).hexdigest()
    added = 0
    for netname, ref, segs in HAND_ROUTES.get(digest, []):
        ni = board.FindNet(netname)
        if ref is not None:                   # skip if the router did reach that pad
            pad = next(p for p in board.FindFootprintByReference(ref[0]).Pads() if p.GetNumber() == ref[1])
            if any(t.GetNetCode() == ni.GetNetCode() and t.GetClass() == 'PCB_TRACK' and
                   (pad.HitTest(t.GetStart()) or pad.HitTest(t.GetEnd())) for t in board.GetTracks()):
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
                    t.SetLayer({'F': pcbnew.F_Cu, '1': pcbnew.In1_Cu, '2': pcbnew.In2_Cu,
                                'B': pcbnew.B_Cu}[kind])
                    t.SetNet(ni)
                    board.Add(t)
        added += 1
    return added


def via_fits(board, q, ni, radius, clearance=0.21):
    """True if a via at q keeps clearance to every pad, track and via of other nets.
    (A layer change the router made through a through-hole pin looks like a
    via-less layer change at a nearby bend point; such a spot fails here.)"""
    pos = mm(*q)
    acc = pcbnew.FromMM(radius + clearance)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() != ni.GetNetCode() and p.HitTest(pos, acc):
                return False
    for t in board.GetTracks():
        if t.GetNetCode() != ni.GetNetCode() and t.HitTest(pos, acc):
            return False
    return True


def through_hole_at(board, q, ni):
    """True if a pad of this net at q already joins the layers: a plated pad, or
    an SMD pad whose pin also has plated holes (e.g. an exposed pad with
    thermal vias, which the router treats as a through-hole pin)."""
    pos = mm(*q)
    for fp in board.GetFootprints():
        pads = list(fp.Pads())
        plated = {p.GetNumber() for p in pads if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH}
        for p in pads:
            if p.GetNetCode() == ni.GetNetCode() and p.HitTest(pos) and \
                    (p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH or p.GetNumber() in plated):
                return True
    return False


REPAIR_VIAS = []
COSMETIC = ('unconnected_items', 'silk_overlap', 'silk_over_copper', 'silk_edge_clearance',
            'lib_footprint_issues', 'lib_footprint_mismatch')
_ITEM = re.compile(r'@\(([-0-9.]+) mm, ([-0-9.]+) mm\): (.*)')


def drc(board, path=f'/tmp/bldc48-drc-probe-{os.getpid()}.rpt'):
    """Run KiCad DRC; return (list of (type, [(x, y, desc), ...]), n_unconnected)."""
    pcbnew.WriteDRCReport(board, path, pcbnew.EDA_UNITS_MILLIMETRES, True)
    items, cur = [], None
    for line in open(path):
        m = re.match(r'\[(\w+)\]:', line)
        if m:
            cur = (m.group(1), [])
            items.append(cur)
            continue
        m = _ITEM.search(line)
        if m and cur:
            cur[1].append((float(m.group(1)), float(m.group(2)), m.group(3)))
    unconnected = sum(1 for t, _ in items if t == 'unconnected_items')
    return items, unconnected


def n_errors(items):
    return sum(1 for t, _ in items if t not in COSMETIC)


def veto_repair_vias(board):
    """Drop any via the SES repair added that DRC finds in conflict with another net."""
    items, _ = drc(board)
    bad = {(round(x, 3), round(y, 3)) for t, locs in items if t not in COSMETIC
           for x, y, d in locs if d.startswith('Via')}
    removed = 0
    for v in list(REPAIR_VIAS):
        p = v.GetPosition()
        if (round(pcbnew.ToMM(p.x), 3), round(pcbnew.ToMM(p.y), 3)) in bad:
            board.Remove(v)
            REPAIR_VIAS.remove(v)
            removed += 1
    return removed


def _endpoints(board, x, y, desc):
    """Candidate (point, layers) to start a repair route from a DRC item."""
    pos = mm(x, y)
    both = {pcbnew.F_Cu, pcbnew.B_Cu}
    if desc.startswith(('Pad', 'PTH pad')):
        m = re.match(r'(?:PTH pad|Pad) (\S+) \[.*\] of (\S+)', desc)
        fp = board.FindFootprintByReference(m.group(2))
        pad = next(p for p in fp.Pads() if p.GetNumber() == m.group(1) and p.HitTest(pos))
        c = pad.GetPosition()
        layers = both if pad.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else {pad.GetLayer()}
        return [((pcbnew.ToMM(c.x), pcbnew.ToMM(c.y)), layers)]
    if desc.startswith('Via'):
        return [((x, y), both)]
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_TRACK' and t.HitTest(pos):
            return [((pcbnew.ToMM(e.x), pcbnew.ToMM(e.y)), {t.GetLayer()})
                    for e in (t.GetStart(), t.GetEnd())]
    return []


def _shapes(a, b):
    (ax, ay), (bx, by) = a, b
    dx, dy = bx - ax, by - ay
    m = min(abs(dx), abs(dy))
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    return [[a, b], [a, (bx, ay), b], [a, (ax, by), b],
            [a, (ax + sx * m, ay + sy * m), b], [a, (bx - sx * m, by - sy * m), b]]


def _pour_joined(net, locs):
    """Connections the outer pours make: the low-side source nets, and a motor
    wire pad on its switch node."""
    return (net in PLANE_NETS or net.startswith('LS_') or
            (net.startswith('SW_') and any(' of J' in d for _, _, d in locs)))


def _add_items(board, net, items):
    ni = board.FindNet(net)
    layer = {'F': pcbnew.F_Cu, '1': pcbnew.In1_Cu, '2': pcbnew.In2_Cu, 'B': pcbnew.B_Cu}
    for kind, data in items:
        if kind == 'V':
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(mm(*data))
            v.SetWidth(pcbnew.FromMM(0.6))
            v.SetDrill(pcbnew.FromMM(0.3))
            v.SetNet(ni)
            board.Add(v)
            continue
        for a, b in zip(data, data[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(mm(*a))
            t.SetEnd(mm(*b))
            t.SetWidth(pcbnew.FromMM(0.25))
            t.SetLayer(layer[kind])
            t.SetNet(ni)
            board.Add(t)


def grid_finish(board):
    """Route what is still open with the 4-layer grid router (astar_route.py):
    first kept off the power array's outer layers, then without that fence for
    nets that live inside it (gate drive, sense).  Deterministic, so a rebuild
    from the same SES gives the same board."""
    import astar_route
    items, _ = drc(board)
    done = 0
    for t, locs in [it for it in items if it[0] == 'unconnected_items']:
        if len(locs) < 2:
            continue
        net = re.search(r'\[(.*?)\]', locs[0][2]).group(1)
        if _pour_joined(net, locs):
            continue
        a, b = locs[0][:2], locs[1][:2]
        for fence in (True, False):
            try:
                g, path = astar_route.route(board, net, a, b, fence=fence)
            except SystemExit:
                continue
            _add_items(board, net, astar_route.to_hand_route(g, path, a, b))
            done += 1
            break
        else:
            print(f'  grid router found no path for {net}')
    return done


def auto_repair(board, max_tries=400):
    """Close connections the router left open: try short F/B routes (direct,
    L, 45-degree, or via-to-the-other-layer) and keep the first that DRC
    accepts with no new violations."""
    fixed = 0
    items, base_unc = drc(board)
    base_err = n_errors(items)
    for t, locs in [it for it in items if it[0] == 'unconnected_items']:
        if len(locs) < 2:
            continue
        net = re.search(r'\[(.*?)\]', locs[0][2]).group(1)
        if _pour_joined(net, locs):           # made by the pours added afterwards
            continue
        ni = board.FindNet(net)
        ea, eb = _endpoints(board, *locs[0]), _endpoints(board, *locs[1])
        if not ea or not eb:
            continue
        (a, la), (b, lb) = min(((p, q) for p in ea for q in eb),
                               key=lambda pq: math.dist(pq[0][0], pq[1][0]))
        cands = []
        for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
            if layer in la and layer in lb:
                cands += [[(layer, path)] for path in _shapes(a, b)]
        other = {pcbnew.F_Cu: pcbnew.B_Cu, pcbnew.B_Cu: pcbnew.F_Cu}
        la1, lb1 = next(iter(la)), next(iter(lb))
        offs = [(d * ux, d * uy) for d in (0.9, 1.4, 2.0)
                for ux, uy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                               (0.707, 0.707), (0.707, -0.707), (-0.707, 0.707), (-0.707, -0.707))]
        mids = [other[la1] if la1 == lb1 else lb1]

        def outside_keepout(q):
            return q[0] > POWER_X + 1.0 or q[1] > KEEPOUT_Y

        for oa in offs:
            for ob in offs:
                va = (round(a[0] + oa[0], 3), round(a[1] + oa[1], 3))
                vb = (round(b[0] + ob[0], 3), round(b[1] + ob[1], 3))
                inner = [pcbnew.In1_Cu, pcbnew.In2_Cu] if outside_keepout(va) and outside_keepout(vb) else []
                for mid in mids + inner:
                    for path in _shapes(va, vb)[:3]:
                        cands.append([(la1, [a, va]), ('via', va), (mid, path), ('via', vb), (lb1, [vb, b])])

        def length(c):
            return sum(math.dist(p, q) for layer, d in c if layer != 'via' for p, q in zip(d, d[1:])) + \
                2.0 * sum(1 for layer, _ in c if layer == 'via')
        cands.sort(key=length)
        for cand in cands[:max_tries]:
            added = []
            for layer, data in cand:
                if layer == 'via':
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(mm(*data))
                    v.SetWidth(pcbnew.FromMM(0.6))
                    v.SetDrill(pcbnew.FromMM(0.3))
                    v.SetNet(ni)
                    board.Add(v)
                    added.append(v)
                    continue
                for p0, p1 in zip(data, data[1:]):
                    if p0 == p1:
                        continue
                    tr = pcbnew.PCB_TRACK(board)
                    tr.SetStart(mm(*p0))
                    tr.SetEnd(mm(*p1))
                    tr.SetWidth(pcbnew.FromMM(0.25))
                    tr.SetLayer(layer)
                    tr.SetNet(ni)
                    board.Add(tr)
                    added.append(tr)
            its, unc = drc(board)
            if unc < base_unc and n_errors(its) <= base_err:
                base_unc = unc
                fixed += 1
                print(f'  repaired {net} with {len(added)} items')
                break
            for it in added:
                board.Remove(it)
        else:
            print(f'  could not repair {net}')
    return fixed


def import_ses(board, ses_path):
    import hashlib
    rip = RIP_UP.get(hashlib.sha1(open(ses_path, 'rb').read()).hexdigest(), set())
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
        return v

    for net in find(find1(routes, 'network_out'), 'net'):
        if net[1] in rip:
            print(f'ripped up the router wiring of {net[1]}')
            continue
        if net[1] in PLANE_NETS:              # only our own fan-out, already on the board
            continue
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
            if len(ls) > 1 and q not in vias and not through_hole_at(board, q, ni) and \
                    via_fits(board, q, ni, default_via[0] / 2):
                REPAIR_VIAS.append(add_via(ni, q[0], q[1], default_via))
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
    ap.add_argument('--no-short-repair', action='store_true',
                    help='skip the DRC-checked short repairs (slow on a dense board) and go '
                         'straight to the grid router')
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
    phase_keepout(board)
    edge_keepout(board)
    print(f'fan-out: {fanout(board)} plane vias')
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        raise SystemExit('DSN export failed')
    patch_dsn(dsn, board)
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
    print(f'added {hand_routes(board, ses)} hand route(s)')
    print(f'removed {veto_repair_vias(board)} repair via(s) that DRC rejected')
    if not args.no_short_repair:
        print(f'auto-repaired {auto_repair(board)} open connection(s)')
    plane_keepout(board, solid_in1=True)
    print(f'grid-routed {grid_finish(board)} more')
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
