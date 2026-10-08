"""Tracks-only view of the routed board (pads, tracks coloured by layer, vias,
outer-layer keep-outs) as an SVG: routing_svg.py board.kicad_pcb out.svg x0 y0 x1 y1"""
import sys, pcbnew
pcb, out, x0, y0, x1, y1 = sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7])
S = 40
b = pcbnew.LoadBoard(pcb)
el = []
P = lambda x, y: ((x-x0)*S, (y-y0)*S)
col = {pcbnew.F_Cu: 'rgba(210,0,0,.6)', pcbnew.B_Cu: 'rgba(0,0,230,.6)', pcbnew.In2_Cu: 'rgba(0,150,0,.6)', pcbnew.In1_Cu: 'rgba(200,150,0,.6)'}
for z in b.Zones():
    if z.GetIsRuleArea() and z.GetDoNotAllowTracks() and z.IsOnLayer(pcbnew.F_Cu) and 'board' not in z.GetZoneName() and 'plane' not in z.GetZoneName():
        o = z.Outline().Outline(0)
        pts = ' '.join('%.1f,%.1f' % P(pcbnew.ToMM(o.CPoint(i).x), pcbnew.ToMM(o.CPoint(i).y)) for i in range(o.PointCount()))
        el.append(f'<polygon points="{pts}" fill="rgba(255,150,150,.25)"/>')
for f in b.GetFootprints():
    for p in f.Pads():
        bb = p.GetBoundingBox(); ax, ay = P(pcbnew.ToMM(bb.GetX()), pcbnew.ToMM(bb.GetY())); w, h = pcbnew.ToMM(bb.GetWidth())*S, pcbnew.ToMM(bb.GetHeight())*S
        fill = 'rgba(255,140,0,.45)' if p.IsOnLayer(pcbnew.F_Cu) else 'rgba(120,120,255,.45)'
        el.append(f'<rect x="{ax:.1f}" y="{ay:.1f}" width="{w:.1f}" height="{h:.1f}" fill="{fill}" stroke="#333" stroke-width=".5"/>')
        if f.GetReference() == 'U2' and p.GetNetname() not in ('GND',):
            el.append(f'<text x="{ax:.1f}" y="{ay:.1f}" font-size="8">{p.GetNetname()}</text>')
    x, y = P(*pcbnew.ToMM(f.GetPosition())); el.append(f'<text x="{x:.1f}" y="{y:.1f}" font-size="11" fill="purple">{f.GetReference()}</text>')
for t in b.GetTracks():
    if t.GetClass() == 'PCB_VIA':
        x, y = P(*pcbnew.ToMM(t.GetPosition())); el.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{.3*S}" fill="#999" stroke="#000"/>'); continue
    (ax, ay), (bx, by) = P(*pcbnew.ToMM(t.GetStart())), P(*pcbnew.ToMM(t.GetEnd()))
    el.append(f'<line x1="{ax:.1f}" y1="{ay:.1f}" x2="{bx:.1f}" y2="{by:.1f}" stroke="{col.get(t.GetLayer(), "#000")}" stroke-width="{max(2, pcbnew.ToMM(t.GetWidth())*S):.1f}" stroke-linecap="round"><title>{t.GetNetname()}</title></line>')
for x in range(int(x0), int(x1)+1):
    if x % 2 == 0: el.append(f'<text x="{P(x,y0)[0]}" y="10" font-size="10">{x}</text>')
for y in range(int(y0), int(y1)+1):
    if y % 2 == 0: el.append(f'<text x="2" y="{P(x0,y)[1]}" font-size="10">{y}</text>')
W, H = (x1-x0)*S, (y1-y0)*S
open(out, 'w').write(f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}"><rect width="100%" height="100%" fill="white"/>' + ''.join(el) + '</svg>')
