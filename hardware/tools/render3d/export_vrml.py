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
board = pcbnew.LoadBoard(os.environ.get('BLDC_PCB', os.path.join(here, '..', '..', 'kicad', 'bldc48.kicad_pcb')))
# stand-in models (extra_models/) for footprints whose stock 3D model is missing
for fp in board.GetFootprints():
    for m in list(fp.Models()):
        path = m.m_Filename.replace('${KICAD7_3DMODEL_DIR}', os.environ['KICAD7_3DMODEL_DIR']) \
                           .replace('${KICAD6_3DMODEL_DIR}', os.environ['KICAD6_3DMODEL_DIR'])
        extra = os.path.join(here, 'extra_models', os.path.basename(path))
        if not os.path.exists(path) and os.path.exists(extra):
            new = pcbnew.FP_3DMODEL()          # Models() yields copies: replace the entry
            new.m_Filename = extra
            new.m_Show = True
            fp.Models().clear()
            fp.Add3DModel(new)
            print('stand-in model for', fp.GetReference())
# footprints of our own library that carry no model at all
for fp in board.GetFootprints():
    extra = os.path.join(here, 'extra_models', str(fp.GetFPID().GetLibItemName()) + '.wrl')
    if not list(fp.Models()) and os.path.exists(extra):
        new = pcbnew.FP_3DMODEL()
        new.m_Filename = extra
        new.m_Show = True
        fp.Add3DModel(new)
        print('stand-in model for', fp.GetReference())
ok = pcbnew.EXPORTER_VRML(board).ExportVRML_File(board.GetProject(), '', os.path.abspath(sys.argv[1]),
                                                 1.0, False, False, '', 0.0, 0.0)
sys.exit(0 if ok else 1)
