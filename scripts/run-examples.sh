#!/usr/bin/env bash
# Build the engine (Release) and render a self-contained replay.html for the showcase scenarios.
# Parity scenarios render straight from their committed golden FCD; the external-agent demos are
# run through the engine first. Open any replay.html in a browser afterwards.
#
#   scripts/run-examples.sh            # the ~15 showcase scenarios + the external-agent demos
#   scripts/run-examples.sh all        # every parity scenario (all 60+)
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
MODE="${1:-showcase}"

echo "==> Building (Release)…"
dotnet build -c Release -v q

run() { dotnet run -c Release --no-build --project "$1" -- "${@:2}"; }

SHOWCASE=(09-traffic-light 11-priority-junction 12-overtake 26-right-before-left 27-allway-stop
  32-roundabout 35-actuated-tls 43-continuous-lanechange 44-multilane-junction-turn 16-emergency-red
  53-giveway-single 55-giveway-drift 47-rail-free-flow 49-rail-bidi-meet 51-rail-crossing)

if [ "$MODE" = "all" ]; then
  SCEN=(); for d in scenarios/*/; do b=$(basename "$d"); [[ "$b" == _* ]] || SCEN+=("$b"); done
else
  SCEN=("${SHOWCASE[@]}")
fi

ok=0; fail=0
echo "==> Rendering ${#SCEN[@]} scenario replay(s)…"
for s in "${SCEN[@]}"; do
  d="scenarios/$s"
  [ -d "$d" ] || { echo "  skip  $s (not found)"; continue; }
  render_ok=1
  if [ -f "$d/golden.fcd.xml" ]; then
    # Parity scenario: render straight from the committed golden FCD.
    run src/Sim.Viz "$d" >/dev/null 2>&1 || render_ok=0
  else
    # Behavioral scenario (no golden, e.g. give-way / opposite-overtake): run the engine to produce
    # an FCD first, then render that.
    { run src/Sim.Run "$d" >/dev/null 2>&1 && run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml" >/dev/null 2>&1; } || render_ok=0
  fi
  if [ "$render_ok" = 1 ]; then echo "  ok    $d/replay.html"; ok=$((ok+1)); else echo "  FAIL  $s"; fail=$((fail+1)); fi
done

echo "==> External-agent demos (engine → combined FCD → replay)…"
for s in ext-showcase ext-swerve-demo ext-agents-demo; do
  d="scenarios/_bench/$s"
  [ -d "$d" ] || continue
  if run src/Sim.ExtDemo "$d" >/dev/null 2>&1 && run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml" >/dev/null 2>&1; then
    echo "  ok    $d/replay.html"; ok=$((ok+1))
  else echo "  FAIL  $s"; fail=$((fail+1)); fi
done

echo ""
echo "Done: $ok rendered, $fail failed. Open any of the replay.html files in a browser."
