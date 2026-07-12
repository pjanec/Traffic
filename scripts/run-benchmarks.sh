#!/usr/bin/env bash
# Build the engine (Release) and run the benchmarks: the highway determinism/micro-benchmark, then
# the scaled-city ladder with engine-vs-SUMO aggregate parity (where a SUMO reference ships).
#
#   scripts/run-benchmarks.sh          # highway + city-30 / city-300 / city-3000
#   scripts/run-benchmarks.sh full     # …also city-15000 (heavy, ~15k concurrent vehicles)
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
MODE="${1:-quick}"

echo "==> Building (Release)…"
dotnet build -c Release -v q

echo ""
echo "==> Determinism + micro-benchmark (highway-dense: steps/s, alloc/veh-step, determinism hash):"
dotnet run -c Release --no-build --project src/Sim.Bench

CITIES=(city-30 city-300 city-3000)
[ "$MODE" = "full" ] && CITIES+=(city-15000)

echo ""
echo "==> Scaled-city benchmarks (RTF / peak RSS / stuck, + engine-vs-SUMO aggregate parity):"
for s in "${CITIES[@]}"; do
  d="scenarios/_bench/$s"
  [ -d "$d" ] || { echo "  skip  $s (not found)"; continue; }
  args=("$d")
  [ -f "$d/summary.xml" ]            && args+=(--sumo-summary "$d/summary.xml")
  [ -f "$d/tripinfo.xml" ]           && args+=(--sumo-tripinfo "$d/tripinfo.xml")
  [ -f "$d/aggregate-tolerance.json" ] && args+=(--aggregate-tolerance "$d/aggregate-tolerance.json")
  echo ""
  echo "--- $s ---"
  dotnet run -c Release --no-build --project src/Sim.BenchCity -- "${args[@]}"
done

echo ""
echo "Tip: for the 1→N-thread core-scaling curve vs SUMO, run scripts/bench-scaling.ps1 (Windows)."
