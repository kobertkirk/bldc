#!/bin/sh
# Render 3D views of the PCB into hardware/renders/.
# Needs: KiCad 7 (pcbnew python), the KiCad 3D models, node + a Chromium
# (set CHROMIUM=/path/to/chrome if not /opt/pw-browsers/chromium).
set -e
cd "$(dirname "$0")"
[ -d node_modules ] || npm install --silent
python3 export_vrml.py board.wrl
python3 -m http.server 8765 --bind 127.0.0.1 >/dev/null 2>&1 & SRV=$!
trap 'kill $SRV' EXIT
sleep 1
mkdir -p ../../renders
node render.mjs board.wrl ../../renders/bldc48 \
  iso:-28:40:1.95:0:4 top:0:89.9:2.05 power:-20:38:0.95:-8:-22 terminals:-35:32:0.62:10:-16 rear:155:38:1.9:0:-4
