#!/usr/bin/env bash
#
# gen-benchmark.sh <targetConcurrency> [gridNumber] [gridLength] [end]
# -------------------------------------
# VB-5 (VIZ_BENCH_TASKS.md Phase 2 / BENCHMARK_SPEC.md): generates the net + routes + config for
# one rung of the scaled-city benchmark, from the pinned pip SUMO install ALONE (no external
# scenario download) -- netgenerate for the network, randomTrips.py + duarouter for demand. The
# rung is a single argument: target PEAK CONCURRENT vehicles (not total trips). Everything else is
# derived: the insertion `period` is tuned via Little's law (concurrent ~= insertion_rate *
# mean_trip_time) against a short pilot SUMO run, then refined 1-2 more times by actually
# measuring peak/mean `running` from --summary-output, per BENCHMARK_SPEC.md's tuning heuristic.
#
# This is a [net] step: NETWORK-ENABLED VM, deliberate, ends in a commit of the small/aggregate
# outputs (net/rou/cfg/provenance -- see VB-8 for which outputs are committed vs regenerated).
# NOT part of `dotnet test` -- SUMO is never required for the offline parity loop.
#
# Usage:
#   scripts/gen-benchmark.sh <targetConcurrency> [gridNumber] [gridLength] [end]
#   scripts/gen-benchmark.sh 30
#   scripts/gen-benchmark.sh 15000 22 200 1800   # larger rungs share ONE net (see NET-SIZING NOTE)
#
# gridNumber/gridLength/end are OPTIONAL and default to the original bring-up-rung values (3, 200,
# 600) so `scripts/gen-benchmark.sh 30` is byte-for-byte the same invocation/behavior as before
# this was parameterized (city-30 stays reproducible from the same one-arg call). Rungs 2-4
# (~300/~3,000/~15,000) pass explicit gridNumber/gridLength/end so they share ONE larger net --
# see the "NET-SIZING NOTE" below and each city-<N>/NOTES.md for the density reasoning.
#
# NET-SIZING NOTE (BENCHMARK_SPEC.md "Network generation" -- option (a), the spec's stated
# preference): size ONE net at the largest (15k) rung and reuse it for smaller rungs too, so only
# demand (the insertion period) changes between rungs -- NOT a net grown per rung. netgenerate is
# deterministic given the same command+seed, so calling this script with the SAME gridNumber/
# gridLength for the 300/3,000/15,000 rungs regenerates a byte-identical net.net.xml each time
# (verified via provenance.txt sha256) without needing to plumb a shared-file path through the
# script. Chosen size: gridNumber=22, gridLength=200 -> ~343 lane-km at LANES=1, which at 15,000
# concurrent vehicles is a mean spacing of ~23 m/vehicle (SUMO jam spacing is ~7.5 m; free-flow
# arterial spacing is 50m+) -- "typical urban density, not bumper-to-bumper, not near-empty" per
# BENCHMARK_SPEC.md. The 300/3,000 rungs on this SAME (fixed) net are consequently much sparser
# (0.9 and 8.7 veh/km respectively) -- an accepted tradeoff of option (a), documented per-rung in
# NOTES.md rather than growing the net per rung (which would be option (b), NOT chosen here).
#
# Output: scenarios/_bench/city-<targetConcurrency>/{net.net.xml,rou.rou.xml,config.sumocfg,
#         provenance.txt}, plus a pilot summary/tripinfo pair left in the work dir for inspection
#         (VB-8 re-runs the FINAL SUMO reference pass itself and commits ITS summary/tripinfo).
#
# ============================================================================================
# ENGINE CAPABILITY FINDING (read before changing LANES below):
#
# netgenerate's default multi-lane connection assignment (-L 2+) does not give every lane a
# <connection> to every route-reachable next edge (e.g. a straight-only lane vs a turn-only
# lane). SUMO itself resolves this via *dynamic* strategic lane-changing while a vehicle
# travels. The engine's ported strategic lane-change (C2-ii, see Sim.Core/Engine.cs
# TryStrategicLaneChange) is explicitly a "single-look-ahead scoped port" -- it only resolves
# the FIRST edge transition of a route at insertion (NetworkModel.ResolveLaneSequence /
# ComputeBestLanes) and only handles a same-edge drop-lane convergence at runtime, not a full
# multi-hop strategic replan. A multi-edge route through a multi-lane grid net WILL throw
# "No <connection> found from edge '...' lane N to edge '...'" at insertion whenever the
# lane the greedy walk lands on doesn't happen to continue.
#
# This was reproduced directly: a 3x3 grid, -L 2, --tls.guess, randomTrips --fringe-factor 5
# demand reliably hits this on ~1/200 vehicles. The SAME net/demand pipeline at -L 1 (single
# lane per edge -- every lane trivially has exactly one outgoing connection per direction, so
# route-to-lane resolution can never be ambiguous) runs 600 simulated seconds / 200 vehicles to
# completion with zero errors. LANES is therefore pinned at 1 for now -- this is the
# "simplify the generated net/demand until the engine runs it" scoping BENCHMARK bring-up calls
# for, not a Sim.Core change. Bumping LANES back up (to actually exercise lane-changing/
# overtaking, as BENCHMARK_SPEC.md's network-generation section wants) is future engine work:
# extend ComputeBestLanes/ResolveLaneSequence to a real multi-hop lookahead (or dynamically
# re-resolve the lane sequence at each junction instead of once at insertion).
# ============================================================================================
#
# 15k-RUNG NOTE (also read before scaling up): BENCHMARK_SPEC.md says to size the net ONCE at the
# 15k rung so the same net serves every rung (only demand/period changes). This script currently
# regenerates a FIXED small net every call (sized for the ~30..~300 bring-up rungs) -- that is
# intentional for now (tune the pipeline small first, per the spec's own ladder). Scaling to
# thousands/15k concurrent on a single-lane net means a much longer/larger net (more lane-km) is
# needed to avoid artificial gridlock at high density; that requires either a bigger --grid.number
# or switching to `netgenerate --rand` with a controlled node count, sized once by measuring
# density at the 15k rung, and is left as a follow-up (VB-8's own per-rung task) rather than done
# here.

set -euo pipefail

if [[ $# -lt 1 || $# -gt 4 ]]; then
  echo "usage: $0 <targetConcurrency> [gridNumber] [gridLength] [end]" >&2
  exit 2
fi

TARGET_CONCURRENCY="$1"
if ! [[ "$TARGET_CONCURRENCY" =~ ^[0-9]+$ ]] || [[ "$TARGET_CONCURRENCY" -le 0 ]]; then
  echo "ERROR: targetConcurrency must be a positive integer, got '$TARGET_CONCURRENCY'" >&2
  exit 2
fi

ARG_GRID_NUMBER="${2:-3}"
ARG_GRID_LENGTH="${3:-200}"
ARG_END="${4:-600}"

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# shellcheck disable=SC1091
source "$REPO_ROOT/SUMO_VERSION"
: "${SUMO_VERSION:?Set SUMO_VERSION in $REPO_ROOT/SUMO_VERSION}"

: "${SUMO_HOME:=/usr/local/lib/python3.11/dist-packages/sumo}"
export SUMO_HOME
if [[ ! -d "$SUMO_HOME" ]]; then
  echo "ERROR: SUMO_HOME ($SUMO_HOME) not found -- run scripts/install-sumo.sh first." >&2
  exit 1
fi
if ! command -v sumo >/dev/null 2>&1 || ! command -v netgenerate >/dev/null 2>&1 \
    || ! command -v duarouter >/dev/null 2>&1; then
  echo "ERROR: sumo/netgenerate/duarouter not on PATH -- run scripts/install-sumo.sh first." >&2
  exit 1
fi

OUT_DIR="$REPO_ROOT/scenarios/_bench/city-${TARGET_CONCURRENCY}"
mkdir -p "$OUT_DIR"

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
cd "$WORK"

# ---- knobs (bring-up defaults unless overridden by the optional positional args) -----------
SEED=42
END="$ARG_END"              # simulated seconds (fill time + steady-state plateau window)
GRID_NUMBER="$ARG_GRID_NUMBER"
GRID_LENGTH="$ARG_GRID_LENGTH"
LANES=1            # see ENGINE CAPABILITY FINDING above -- do not bump without re-validating
                    # multi-edge routes against the engine first (dotnet run --project src/Sim.Run)
FRINGE_FACTOR=5
# Larger nets/demand need more tuning headroom and a bigger pilot period (a low pilot period on a
# huge net can make the very first pilot run itself already badly congested, giving a useless mean
# trip-time estimate) -- scale both with gridNumber relative to the bring-up default (3).
PILOT_PERIOD=$(python3 -c "print(max(1.0, 5.0 * (${GRID_NUMBER} / 3.0)))")
TUNE_ITERS=$([[ "$GRID_NUMBER" -gt 3 ]] && echo 4 || echo 2)

hash_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

# ---- 1. network (shared by every tuning iteration below) -----------------------------------
NETGEN_CMD=(netgenerate --grid --grid.number="$GRID_NUMBER" --grid.length="$GRID_LENGTH" \
  -L "$LANES" --tls.guess --seed "$SEED" -o city.net.xml)
echo "==> ${NETGEN_CMD[*]}"
"${NETGEN_CMD[@]}"

write_config() {
  local cfg="$1"
  local net_file="${2:-city.net.xml}"
  local route_file="${3:-city.rou.xml}"
  cat > "$cfg" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<configuration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">
    <input>
        <net-file value="${net_file}"/>
        <route-files value="${route_file}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="${END}"/>
        <step-length value="1"/>
    </time>
    <processing>
        <step-method.ballistic value="false"/>
        <time-to-teleport value="-1"/>
        <default.action-step-length value="1"/>
        <default.speeddev value="0"/>
    </processing>
    <random_number>
        <seed value="${SEED}"/>
    </random_number>
</configuration>
EOF
}

# Adds an explicit DEFAULT_VEHTYPE (sigma=0 -- phase-1 determinism convention, see CLAUDE.md
# "Determinism (phase 1)") and references it from every <vehicle>, since randomTrips.py/duarouter
# omit a <vType> and the engine's DemandParser (unlike SUMO) does not synthesize an implicit
# default vType -- see gen-benchmark.sh's own header note if this throws KeyNotFoundException.
add_default_vtype() {
  local rou="$1"
  python3 - "$rou" <<'PYEOF'
import re
import sys

path = sys.argv[1]
with open(path) as f:
    content = f.read()

content = re.sub(
    r'(<routes[^>]*>)',
    r'\1\n    <vType id="DEFAULT_VEHTYPE" vClass="passenger" sigma="0"/>',
    content, count=1)
content = re.sub(
    r'<vehicle id="([^"]+)" depart=',
    r'<vehicle id="\1" type="DEFAULT_VEHTYPE" depart=',
    content)

with open(path, 'w') as f:
    f.write(content)
PYEOF
}

# Generates demand at a given insertion `period`, pre-routes with duarouter --named-routes (the
# format DemandParser needs: <route id=.../> + <vehicle route="id"/>, NOT duarouter's default
# embedded <route> child -- see gen-benchmark.sh's ENGINE CAPABILITY FINDING section for the other
# format gap this avoids), and patches in DEFAULT_VEHTYPE. Writes city.rou.xml in $WORK.
generate_demand() {
  local period="$1"
  python3 "$SUMO_HOME/tools/randomTrips.py" -n city.net.xml -e "$END" -p "$period" \
    --fringe-factor "$FRINGE_FACTOR" --seed "$SEED" -o city.trips.xml -r city.trips.rou.xml \
    >/dev/null
  duarouter -n city.net.xml -r city.trips.xml -o city.rou.xml --seed "$SEED" \
    --ignore-errors --named-routes >/dev/null
  add_default_vtype city.rou.xml
}

# Runs SUMO on the CURRENT city.net.xml/city.rou.xml, returns (via globals) the measured mean
# trip duration, peak/mean-steady running count, and a COLLAPSE flag from --summary-output.
#
# COLLAPSE DETECTION (added after directly reproducing runaway network gridlock at the 3k/15k
# rungs' scale): a naive Little's-law period computed from a FREE-FLOW pilot's mean trip time
# badly overestimates the sustainable insertion rate once the target concurrency approaches this
# net's critical (max-throughput) density -- beyond that point, `running` grows ~unboundedly
# instead of leveling off (classic macroscopic-fundamental-diagram gridlock: queues build faster
# than junctions can discharge them, which is throttled harder still, a runaway positive
# feedback). Signature straight off the FINAL step of a collapsing run: halting/running ratio
# high (most vehicles stopped) AND meanSpeedRelative very low (near-zero net speed) AND
# `arrived` a tiny fraction of `loaded` (almost nothing is completing trips). Any one of these
# triggers COLLAPSED=1, which the tuning loop below treats specially (multiplicative period
# backoff, NOT the usual proportional mean/target rescale -- an average over a still-diverging
# tail is not a valid steady-state estimate to rescale from).
measure_concurrency() {
  write_config city.sumocfg
  sumo -c city.sumocfg --summary-output measure.summary.xml --tripinfo-output measure.tripinfo.xml \
    --no-step-log true >/dev/null
  read -r MEAN_TRIP_TIME PEAK_RUNNING MEAN_RUNNING_STEADY N_ARRIVED COLLAPSED < <(python3 - <<'PYEOF'
import xml.etree.ElementTree as ET

summary = ET.parse("measure.summary.xml").getroot()
steps = summary.findall("step")
running = [int(s.get("running")) for s in steps]
peak = max(running) if running else 0
tail = running[len(running)//3:] if running else []
mean_steady = (sum(tail) / len(tail)) if tail else 0.0

trips = ET.parse("measure.tripinfo.xml").getroot()
durations = [float(t.get("duration")) for t in trips.findall("tripinfo")]
mean_duration = (sum(durations) / len(durations)) if durations else 0.0

collapsed = 0
if steps:
    last = steps[-1]
    last_running = int(last.get("running"))
    last_halting = int(last.get("halting"))
    last_loaded = int(last.get("loaded"))
    last_arrived = int(last.get("arrived"))
    speed_rel = float(last.get("meanSpeedRelative"))
    halting_frac = (last_halting / last_running) if last_running > 0 else 0.0
    arrived_frac = (last_arrived / last_loaded) if last_loaded > 0 else 1.0
    if (halting_frac > 0.5 and speed_rel < 0.2) or (last_loaded >= 200 and arrived_frac < 0.05):
        collapsed = 1

print(f"{mean_duration:.6f} {peak} {mean_steady:.6f} {len(durations)} {collapsed}")
PYEOF
)
}

# ---- 2. pilot run: rough period, just to estimate mean trip time ---------------------------
echo "==> pilot demand at period=${PILOT_PERIOD}s (estimating mean trip time)"
generate_demand "$PILOT_PERIOD"
measure_concurrency
echo "    pilot: mean_trip_time=${MEAN_TRIP_TIME}s peak_running=${PEAK_RUNNING} mean_running=${MEAN_RUNNING_STEADY} arrived=${N_ARRIVED}"

# ---- 3. Little's law: period ~= mean_trip_time / target, then measure + refine (2x bring-up
#         rungs, up to 4x for scaled-net rungs -- bigger nets/insertion-rate swings converge
#         slower; see TUNE_ITERS above). COLLAPSE-AWARE: track the best (highest,
#         non-collapsing) steady mean seen across iterations -- if the target band is never
#         reached without the network collapsing into runaway gridlock (see measure_concurrency's
#         COLLAPSE DETECTION comment), fall back to that best-stable period rather than whatever
#         the last (possibly collapsed) iteration produced. This is a legitimate network-capacity
#         finding (BENCHMARK_SPEC.md / the benchmark briefing both explicitly anticipate it), not
#         a bug to paper over. ---------------------------------------------------------------
PERIOD=$(python3 -c "print(max(0.001, ${MEAN_TRIP_TIME} / ${TARGET_CONCURRENCY}))")
BEST_STABLE_PERIOD=""
BEST_STABLE_MEAN="0"
REACHED_BAND=0
for ITER in $(seq 1 "$TUNE_ITERS"); do
  echo "==> tuning iteration ${ITER}: period=${PERIOD}s (target concurrency=${TARGET_CONCURRENCY})"
  generate_demand "$PERIOD"
  measure_concurrency
  echo "    measured: mean_trip_time=${MEAN_TRIP_TIME}s peak_running=${PEAK_RUNNING} mean_running=${MEAN_RUNNING_STEADY} arrived=${N_ARRIVED} collapsed=${COLLAPSED}"

  if [[ "$COLLAPSED" == "1" ]]; then
    echo "    COLLAPSE detected (runaway gridlock signature) -- backing off period (x1.7), NOT trusting this iteration's steady-mean for rescaling."
    PERIOD=$(python3 -c "print(${PERIOD} * 1.7)")
    continue
  fi

  # This iteration is a genuine (non-collapsing) data point -- remember it if it's the best
  # (highest stable concurrency) seen so far, regardless of whether it lands in-band.
  IS_BEST=$(python3 -c "print(1 if ${MEAN_RUNNING_STEADY} > ${BEST_STABLE_MEAN} else 0)")
  if [[ "$IS_BEST" == "1" ]]; then
    BEST_STABLE_PERIOD="$PERIOD"
    BEST_STABLE_MEAN="$MEAN_RUNNING_STEADY"
  fi

  # Close enough (within ~35% of target on the steady-state mean)? Stop early.
  WITHIN_BAND=$(python3 -c "
target = ${TARGET_CONCURRENCY}
mean = ${MEAN_RUNNING_STEADY}
print(1 if target * 0.65 <= mean <= target * 1.35 else 0)
")
  if [[ "$WITHIN_BAND" == "1" ]]; then
    echo "    within target band -- stopping tuning."
    REACHED_BAND=1
    break
  fi

  # Re-derive period from the just-measured mean_trip_time and steady concurrency (Little's
  # law again, using the ACTUAL measured relationship rate=1/period -> concurrent=mean_running
  # to rescale proportionally toward the target).
  PERIOD=$(python3 -c "
period = ${PERIOD}
mean = ${MEAN_RUNNING_STEADY}
target = ${TARGET_CONCURRENCY}
print(max(0.001, period * (mean / target) if mean > 0 else period))
")
done

if [[ "$REACHED_BAND" == "1" ]]; then
  FINAL_PERIOD="$PERIOD"
  NET_CAPACITY_FINDING="target concurrency reached within tolerance band"
elif [[ -n "$BEST_STABLE_PERIOD" ]]; then
  echo "==> target band NOT reached without collapse after ${TUNE_ITERS} iterations -- falling back to the best STABLE (non-collapsing) period found (mean_running=${BEST_STABLE_MEAN})."
  FINAL_PERIOD="$BEST_STABLE_PERIOD"
  generate_demand "$FINAL_PERIOD"
  measure_concurrency
  NET_CAPACITY_FINDING="target concurrency ${TARGET_CONCURRENCY} NOT reached without runaway gridlock on this net; falling back to best stable period (mean_running_steady=${MEAN_RUNNING_STEADY}, peak_running=${PEAK_RUNNING}) -- see NOTES.md for the collapse trajectory"
else
  echo "==> WARNING: every tuning iteration collapsed -- using the last (collapsed) period; this rung's net is undersized for its demand." >&2
  FINAL_PERIOD="$PERIOD"
  NET_CAPACITY_FINDING="every tuning iteration collapsed into runaway gridlock; net is undersized for target concurrency ${TARGET_CONCURRENCY}"
fi

echo "==> final tuned period=${FINAL_PERIOD}s -> peak_running=${PEAK_RUNNING} mean_running=${MEAN_RUNNING_STEADY} arrived=${N_ARRIVED}"
echo "==> net-capacity finding: ${NET_CAPACITY_FINDING}"

# ---- 4. install into the committed scenario dir ---------------------------------------------
cp city.net.xml "$OUT_DIR/net.net.xml"
cp city.rou.xml "$OUT_DIR/rou.rou.xml"
write_config "$OUT_DIR/config.sumocfg" "net.net.xml" "rou.rou.xml"

RANDOMTRIPS_CMD="python3 \$SUMO_HOME/tools/randomTrips.py -n city.net.xml -e ${END} -p ${FINAL_PERIOD} --fringe-factor ${FRINGE_FACTOR} --seed ${SEED} -o city.trips.xml -r city.trips.rou.xml"
DUAROUTER_CMD="duarouter -n city.net.xml -r city.trips.xml -o city.rou.xml --seed ${SEED} --ignore-errors --named-routes"

{
  echo "sumo_version=${SUMO_VERSION}"
  echo "sumo_version_reported=$(sumo --version 2>&1 | head -n 1)"
  echo "generated_utc=$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
  echo "target_concurrency=${TARGET_CONCURRENCY}"
  echo "seed=${SEED}"
  echo "begin=0 end=${END} step-length=1"
  echo "net_command=${NETGEN_CMD[*]}"
  echo "demand_command_randomtrips=${RANDOMTRIPS_CMD}"
  echo "demand_command_duarouter=${DUAROUTER_CMD}"
  echo "demand_postprocess=add DEFAULT_VEHTYPE (vClass=passenger sigma=0) vType + type= reference on every <vehicle> (randomTrips/duarouter emit neither; DemandParser requires an explicit vType -- see gen-benchmark.sh add_default_vtype)"
  echo "tuned_insertion_period_s=${FINAL_PERIOD}"
  echo "measured_mean_trip_time_s=${MEAN_TRIP_TIME}"
  echo "measured_peak_running=${PEAK_RUNNING}"
  echo "measured_mean_running_steady=${MEAN_RUNNING_STEADY}"
  echo "measured_arrived=${N_ARRIVED}"
  echo "net_grid_number=${GRID_NUMBER} net_grid_length=${GRID_LENGTH} net_lanes=${LANES}"
  echo "net_capacity_finding=${NET_CAPACITY_FINDING}"
  echo "engine_capability_note=LANES pinned at ${LANES} -- see gen-benchmark.sh header 'ENGINE CAPABILITY FINDING' (multi-lane multi-hop route-to-lane resolution is a single-look-ahead scoped port in Sim.Core/Engine.cs, C2-ii; -L 2+ throws 'No connection found' at insertion for some multi-edge routes)"
  echo "# input file hashes (sha256):"
  for f in "$OUT_DIR"/*.net.xml "$OUT_DIR"/*.rou.xml "$OUT_DIR"/*.sumocfg; do
    [[ -e "$f" ]] || continue
    echo "input=$(basename "$f") sha256=$(hash_file "$f")"
  done
} > "$OUT_DIR/provenance.txt"

echo
echo "==> wrote $OUT_DIR/{net.net.xml,rou.rou.xml,config.sumocfg,provenance.txt}"
echo "==> next: run the SUMO reference pass + engine run + comparator + viz (see VIZ_BENCH_TASKS.md VB-7/VB-8)"
