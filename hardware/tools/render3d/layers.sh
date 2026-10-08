#!/bin/sh
# Copper-layer images (each layer + board outline) and a tracks-only routing view
# into hardware/renders/.  Needs KiCad 7 (kicad-cli, pcbnew python) and node.
set -e
cd "$(dirname "$0")"
[ -d node_modules ] || npm install --silent
PCB=../../kicad/bldc48.kicad_pcb
OUT=../../renders
T=$(mktemp -d)
for spec in top:F.Cu in1:In1.Cu in2:In2.Cu bottom:B.Cu; do
  name=${spec%%:*}; layer=${spec#*:}
  kicad-cli pcb export svg -l "$layer,Edge.Cuts" --exclude-drawing-sheet --page-size-mode 2 \
    -o "$T/$name.svg" "$PCB" >/dev/null
  node svg2png.mjs "$T/$name.svg" "$OUT/bldc48-layer-$name.png"
done
python3 routing_svg.py "$PCB" "$T/routing.svg" 0 0 90 95
node svg2png.mjs "$T/routing.svg" "$OUT/bldc48-routing.png"
rm -rf "$T"
echo "wrote $OUT/bldc48-layer-*.png and bldc48-routing.png"
