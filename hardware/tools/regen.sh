#!/bin/sh
# Regenerate schematic, PCB, BOM and schematic PDF from circuit.py and verify
# connectivity.  Needs KiCad 7+ (kicad-cli and the pcbnew python module).
set -e
cd "$(dirname "$0")"
python3 gen_schematic.py
python3 check_netlist.py ../kicad/bldc48.kicad_sch
python3 gen_pcb.py
python3 gen_bom.py
kicad-cli sch export pdf -o ../bldc48-schematic.pdf ../kicad/bldc48.kicad_sch >/dev/null
echo "done"
