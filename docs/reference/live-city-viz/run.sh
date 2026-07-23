#!/usr/bin/env bash
# Regenerate the 2D live-city HTML from the sample data. Requires SUMO (sumo + sumolib) + python3.
set -euo pipefail
export SUMO_HOME="${SUMO_HOME:-/usr/local/lib/python3.11/dist-packages/sumo}"
export PYTHONPATH="$SUMO_HOME/tools:${PYTHONPATH:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"
D="$HERE/sample_data"
# 1) vehicle FCD from the served scenario (moderate duration -> shareable HTML).
( cd "$D" && sumo -c scenario.sumocfg --fcd-output "$HERE/box.fcd.xml" --end 350 --no-step-log true )
# 2) build the self-contained HTML with all static overlays.
python3 "$HERE/renderer/sim_viz.py" \
  --net "$D/net.xml" --fcd "$HERE/box.fcd.xml" --rou "$D/scenario.rou.xml" \
  --pois "$D/pois.json" --zones "$D/zones.json" --buildings "$D/buildings.json" \
  --out "$HERE/sample.html" --name "Live-city 2D sample" --desc "demo-city box, cars + overlays"
echo "open $HERE/sample.html"
# For pedestrians: run a person-carrying sim and add its FCD via --ped-fcd (see DESIGN §6).
