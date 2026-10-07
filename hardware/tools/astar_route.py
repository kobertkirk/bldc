#!/usr/bin/env python3
"""
Grid router for the few connections Freerouting leaves open.

    astar_route.py NET x1 y1 x2 y2 [--board FILE]
    astar_route.py +3V3 x1 y1 --to-plane      (end on a via into the In2 +3V3 plane)

Finds a path on the four copper layers from the item at (x1, y1) to the item
at (x2, y2) (both of NET), or to a via anywhere over the controller's +3V3
plane.  It keeps the 0.2 mm clearance to other nets, stays
out of track keep-outs and the high-current pours, and changes layers with
0.6/0.3 mm vias.  It prints a HAND_ROUTES entry for route_pcb.py; DRC there
is the final check.
"""
import argparse
import heapq
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_pcb import ARRAY_Y, CAP_X, CAP_YS, H, POWER_X, W  # noqa: E402

G = 0.25                         # grid pitch, mm
OY = 0.15                        # grid y offset: puts the 0.5 mm MCU pin rows on grid
TRACK_W, VIA_D, CLEAR = 0.25, 0.6, 0.21
EDGE = 1.0
LAYERS = [pcbnew.F_Cu, pcbnew.In1_Cu, pcbnew.In2_Cu, pcbnew.B_Cu]
CODES = {pcbnew.F_Cu: 'F', pcbnew.In1_Cu: '1', pcbnew.In2_Cu: '2', pcbnew.B_Cu: 'B'}
VIA_COST, DIAG = 12.0, math.sqrt(2)
POUR_NETS = ('+48V', 'SW_', 'LS_')   # outer pours a track must not cut


def to_mm(v):
    return pcbnew.ToMM(v)


class Grid:
    def __init__(self):
        self.nx = int(W / G) + 1
        self.ny = int((H - OY) / G) + 1
        n = self.nx * self.ny
        self.track = [bytearray(n) for _ in LAYERS]     # 1 = a track centre here clashes
        self.via = bytearray(n)                         # 1 = a via centre here clashes

    def xy(self, i, j):
        return i * G, OY + j * G

    def ij(self, x, y):
        return round(x / G), round((y - OY) / G)

    def mark_disc(self, grid, x, y, r):
        i0, j0 = self.ij(x - r, y - r)
        i1, j1 = self.ij(x + r, y + r)
        for j in range(max(j0, 0), min(j1 + 1, self.ny)):
            for i in range(max(i0, 0), min(i1 + 1, self.nx)):
                cx, cy = self.xy(i, j)
                if (cx - x) ** 2 + (cy - y) ** 2 <= r * r:
                    grid[j * self.nx + i] = 1

    def mark_segment(self, grid, ax, ay, bx, by, r):
        i0, j0 = self.ij(min(ax, bx) - r, min(ay, by) - r)
        i1, j1 = self.ij(max(ax, bx) + r, max(ay, by) + r)
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy or 1e-12
        for j in range(max(j0, 0), min(j1 + 1, self.ny)):
            for i in range(max(i0, 0), min(i1 + 1, self.nx)):
                cx, cy = self.xy(i, j)
                t = max(0.0, min(1.0, ((cx - ax) * dx + (cy - ay) * dy) / L2))
                px, py = ax + t * dx - cx, ay + t * dy - cy
                if px * px + py * py <= r * r:
                    grid[j * self.nx + i] = 1

    def mark_box(self, grid, x0, y0, x1, y1, r):
        """Cells whose centre lies within r of the box (exact bounds, no rounding out)."""
        i0, i1 = math.ceil((x0 - r) / G - 1e-9), math.floor((x1 + r) / G + 1e-9)
        j0, j1 = math.ceil((y0 - r - OY) / G - 1e-9), math.floor((y1 + r - OY) / G + 1e-9)
        for j in range(max(j0, 0), min(j1 + 1, self.ny)):
            for i in range(max(i0, 0), min(i1 + 1, self.nx)):
                grid[j * self.nx + i] = 1

    def mark_poly(self, grid, outline):
        bb = outline.BBox()
        i0, j0 = self.ij(to_mm(bb.GetX()), to_mm(bb.GetY()))
        i1, j1 = self.ij(to_mm(bb.GetRight()), to_mm(bb.GetBottom()))
        for j in range(max(j0 - 1, 0), min(j1 + 2, self.ny)):
            for i in range(max(i0 - 1, 0), min(i1 + 2, self.nx)):
                x, y = self.xy(i, j)
                if outline.Contains(pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))):
                    grid[j * self.nx + i] = 1


def build(board, net, fence=True):
    g = Grid()
    code = board.FindNet(net).GetNetCode()
    rt, rv = TRACK_W / 2 + CLEAR, VIA_D / 2 + CLEAR
    for t in board.GetTracks():
        if t.GetClass() == 'PCB_VIA':             # drill-to-drill spacing, any net
            x, y = to_mm(t.GetPosition().x), to_mm(t.GetPosition().y)
            g.mark_disc(g.via, x, y, VIA_D + 0.26)
        if t.GetNetCode() == code:
            continue
        if t.GetClass() == 'PCB_VIA':
            x, y = to_mm(t.GetPosition().x), to_mm(t.GetPosition().y)
            r = to_mm(t.GetWidth()) / 2
            for k in range(len(LAYERS)):
                g.mark_disc(g.track[k], x, y, r + rt)
            g.mark_disc(g.via, x, y, r + rv)
            continue
        if t.GetLayer() not in LAYERS:
            continue
        k = LAYERS.index(t.GetLayer())
        s, e, hw = t.GetStart(), t.GetEnd(), to_mm(t.GetWidth()) / 2
        g.mark_segment(g.track[k], to_mm(s.x), to_mm(s.y), to_mm(e.x), to_mm(e.y), hw + rt)
        g.mark_segment(g.via, to_mm(s.x), to_mm(s.y), to_mm(e.x), to_mm(e.y), hw + rv)
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == code and code > 0:
                continue
            bb = p.GetBoundingBox()
            box = (to_mm(bb.GetX()), to_mm(bb.GetY()), to_mm(bb.GetRight()), to_mm(bb.GetBottom()))
            for k, layer in enumerate(LAYERS):
                if p.IsOnLayer(layer) or p.GetDrillSize().x > 0:
                    g.mark_box(g.track[k], *box, rt)
            g.mark_box(g.via, *box, rv + (0.05 if p.GetDrillSize().x > 0 else 0))
    for z in board.Zones():
        o = z.Outline()
        if z.GetIsRuleArea():
            for k, layer in enumerate(LAYERS):
                if z.IsOnLayer(layer) and z.GetDoNotAllowTracks():
                    g.mark_poly(g.track[k], o)
            if z.GetDoNotAllowVias() or 'phase' in z.GetZoneName():
                g.mark_poly(g.via, o)
        elif fence and z.GetNetname().startswith(POUR_NETS) and z.GetNetCode() != code:
            for k, layer in enumerate(LAYERS):
                if layer in (pcbnew.F_Cu, pcbnew.B_Cu) and z.IsOnLayer(layer):
                    g.mark_poly(g.track[k], o)
    # outer layers stay off the power array and battery strip: a track there would
    # slice the +48 V / switch-node copper; inner layers have their own rule area
    for k in ((0, len(LAYERS) - 1) if fence else ()):
        g.mark_box(g.track[k], 0, 0, POWER_X + 7.8, ARRAY_Y, 0)
    for grid in g.track + [g.via]:                    # board-edge margin
        m = EDGE + (VIA_D / 2 if grid is g.via else TRACK_W / 2)
        for j in range(g.ny):
            for i in range(g.nx):
                x, y = g.xy(i, j)
                if x < m or y < m or x > W - m or y > H - m:
                    grid[j * g.nx + i] = 1
    return g


def item_layers(board, net, x, y):
    """Layers on which the net's pad / track / via at (x, y) can be reached."""
    q = pcbnew.VECTOR2I(pcbnew.FromMM(x), pcbnew.FromMM(y))
    code = board.FindNet(net).GetNetCode()
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == code and p.HitTest(q):
                return [k for k, layer in enumerate(LAYERS) if p.IsOnLayer(layer)]
    for t in board.GetTracks():
        if t.GetNetCode() == code and t.HitTest(q):
            if t.GetClass() == 'PCB_VIA':
                return list(range(len(LAYERS)))
            return [LAYERS.index(t.GetLayer())]
    raise SystemExit(f'no {net} item at ({x}, {y})')


def route(board, net, a, b, to_plane=False, fence=True):
    g = build(board, net, fence)
    sl = item_layers(board, net, *a)
    gl = [] if to_plane else item_layers(board, net, *b)
    si, sj = g.ij(*a)
    ti, tj = g.ij(*b)
    def on_3v3(i, j):                              # In2 is +3V3 outside the +48 V island
        x, y = g.xy(i, j)
        m = 1.0
        in48 = (x < CAP_X + m and y < ARRAY_Y + m) or \
               (POWER_X + 7.8 - m < x < CAP_X + m and y < CAP_YS[-1] + 9.4 + m)
        return not in48
    nx = g.nx
    for k in sl:                                   # the end cells sit on our own net's item
        g.track[k][sj * nx + si] = 0
    for k in gl:
        g.track[k][tj * nx + ti] = 0

    def h(i, j):
        if to_plane:
            return 0.0
        return math.hypot(i - ti, j - tj)

    start = [(h(si, sj), 0.0, si, sj, k) for k in sl]
    heapq.heapify(start)
    pq, cost, came = start, {}, {}
    for _, c, i, j, k in start:
        cost[(i, j, k)] = 0.0
        came[(i, j, k)] = None
    goal = None
    while pq:
        _, c, i, j, k = heapq.heappop(pq)
        if c > cost.get((i, j, k), 1e18):
            continue
        if to_plane and on_3v3(i, j) and not g.via[j * nx + i]:
            goal = (i, j, k)
            break
        if not to_plane and (i, j) == (ti, tj) and k in gl:
            goal = (i, j, k)
            break
        steps = []
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
            a2, b2 = i + di, j + dj
            if 0 <= a2 < nx and 0 <= b2 < g.ny and not g.track[k][b2 * nx + a2]:
                if di and dj and (g.track[k][j * nx + a2] or g.track[k][b2 * nx + i]):
                    continue                       # no corner cutting
                steps.append((a2, b2, k, DIAG if di and dj else 1.0))
        if not g.via[j * nx + i]:
            for k2 in range(len(LAYERS)):
                if k2 != k and not g.track[k2][j * nx + i]:
                    steps.append((i, j, k2, VIA_COST))
        for a2, b2, k2, w in steps:
            nc = c + w
            if nc < cost.get((a2, b2, k2), 1e18):
                cost[(a2, b2, k2)] = nc
                came[(a2, b2, k2)] = (i, j, k)
                heapq.heappush(pq, (nc + h(a2, b2), nc, a2, b2, k2))
    if goal is None:
        raise SystemExit(f'{net}: no path')
    path, n = [], goal
    if to_plane:                                   # finish with the via into the plane
        path.append((goal[0], goal[1], LAYERS.index(pcbnew.In2_Cu)))
        path.append(goal)
    while n:
        path.append(n)
        n = came[n]
    path.reverse()
    return g, path


def clear_line(g, k, p, q):
    """True if every grid cell under the straight segment p-q is free on layer k."""
    (ax, ay), (bx, by) = g.xy(*p), g.xy(*q)
    n = max(1, int(math.hypot(bx - ax, by - ay) / (G / 4)))
    for t in range(n + 1):
        x, y = ax + (bx - ax) * t / n, ay + (by - ay) * t / n
        for dx in (-G / 2, 0, G / 2):              # the cells either side of the line too
            for dy in (-G / 2, 0, G / 2):
                i, j = g.ij(x + dx, y + dy)
                cx, cy = g.xy(i, j)
                if math.hypot(cx - x, cy - y) > G * 0.75:
                    continue
                if g.track[k][j * g.nx + i] and (i, j) not in (p, q):
                    return False
    return True


def to_hand_route(g, path, a, b):
    """Corner points per layer run (line-of-sight smoothed), vias at layer changes."""
    items, run = [], []

    def flush(k):
        simp_c = [run[0]]
        idx = 0
        while idx < len(run) - 1:
            far = len(run) - 1
            while far > idx + 1 and not clear_line(g, k, run[idx], run[far]):
                far -= 1
            simp_c.append(run[far])
            idx = far
        simp = [g.xy(i, j) for i, j in simp_c]
        if len(simp) > 1:
            items.append((CODES[LAYERS[k]], [(round(x, 3), round(y, 3)) for x, y in simp]))

    k0 = path[0][2]
    for i, j, k in path:
        if k != k0:
            flush(k0)
            x, y = g.xy(i, j)
            items.append(('V', (round(x, 3), round(y, 3))))
            run, k0 = [], k
        run.append((i, j))
    flush(k0)
    tracks = [it for it in items if it[0] != 'V']
    if tracks:                                     # snap the ends onto the real items
        tracks[0][1][0] = a
        if b is not None:
            tracks[-1][1][-1] = b
    if items and items[0][0] == 'V':
        items[0] = ('V', a)
    if items and items[-1][0] == 'V' and b is not None:
        items[-1] = ('V', b)
    return items


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('net')
    ap.add_argument('coords', nargs='+', type=float, help='x1 y1 [x2 y2]')
    ap.add_argument('--to-plane', action='store_true')
    ap.add_argument('--no-fence', action='store_true', help='allow the outer layers over the bridges '
                    '(for nets local to a half bridge, e.g. a gate drive)')
    ap.add_argument('--board', default=os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                                    '..', 'kicad', 'bldc48.kicad_pcb'))
    args = ap.parse_args()
    board = pcbnew.LoadBoard(args.board)
    a = tuple(args.coords[:2])
    b = tuple(args.coords[2:4]) if len(args.coords) >= 4 else None
    g, path = route(board, args.net, a, b or a, to_plane=args.to_plane, fence=not args.no_fence)
    print(f"        ('{args.net}', None, {to_hand_route(g, path, a, b)!r}),")


if __name__ == '__main__':
    main()
