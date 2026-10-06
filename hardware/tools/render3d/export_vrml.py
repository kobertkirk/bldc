#!/usr/bin/env python3
"""Export the board (with embedded 3D component models) to VRML.

usage: KICAD7_3DMODEL_DIR=/usr/share/kicad/3dmodels python3 export_vrml.py out.wrl
"""
import os
import sys

import pcbnew

os.environ.setdefault('KICAD7_3DMODEL_DIR', '/usr/share/kicad/3dmodels')
os.environ.setdefault('KICAD6_3DMODEL_DIR', os.environ['KICAD7_3DMODEL_DIR'])
here = os.path.dirname(os.path.abspath(__file__))
board = pcbnew.LoadBoard(os.path.join(here, '..', '..', 'kicad', 'bldc48.kicad_pcb'))
ok = pcbnew.EXPORTER_VRML(board).ExportVRML_File(board.GetProject(), '', os.path.abspath(sys.argv[1]),
                                                 1.0, False, False, '', 0.0, 0.0)
sys.exit(0 if ok else 1)
