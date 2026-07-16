#!/usr/bin/env bash
# City3D demo — performance ladder (task T3.2). Drives Sim.Host.App headless across the committed
# _bench scale rungs and reports peak concurrent vehicles + wall time, showing the same build spans a
# tiny scenario up to a ~4k-vehicle city with no code change (the scenario is the only dial).
#
# This is a HOST-side measurement (engine step + replication publish); it needs no GPU and no viewer.
# Reconstruction/rendering scale is separate (the viewer draws cars/roads via MultiMesh + one ArrayMesh
# per lane — see the README).
#
# Usage: demos/City3D/perf-ladder.sh   [STEPS]   (default 240 sim steps per rung, paced at --hz 40)
set -euo pipefail
ROOT="$(git rev-parse --show-toplevel)"
STEPS="${1:-240}"
HZ=40

echo "==> building Sim.Host.App"
dotnet build "$ROOT/src/Sim.Host.App/Sim.Host.App.csproj" -c Release >/dev/null

printf '%-16s %14s %10s %s\n' "scenario" "peak vehicles" "wall(s)" "(steps=$STEPS, hz=$HZ, inmem)"
for SC in city-30 city-300 city-mixed-1k city-3000 city-15000; do
  dir="$ROOT/scenarios/_bench/$SC"
  [[ -d "$dir" ]] || { printf '%-16s %14s\n' "$SC" "(missing)"; continue; }
  t0=$(date +%s.%N)
  out=$(dotnet run --project "$ROOT/src/Sim.Host.App" -c Release --no-build -- \
        --scenario "$dir" --transport inmem --hz "$HZ" --steps "$STEPS" 2>&1)
  t1=$(date +%s.%N)
  el=$(printf "%.1f" "$(echo "$t1 - $t0" | bc)")
  peak=$(echo "$out" | grep -oE "vehicles=[0-9]+" | grep -oE "[0-9]+$" | sort -rn | head -1)
  printf '%-16s %14s %10s\n' "$SC" "${peak:-?}" "$el"
done
