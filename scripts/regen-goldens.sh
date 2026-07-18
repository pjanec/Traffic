#!/usr/bin/env bash
#
# regen-goldens.sh
# ----------------
# (Re)generates the committed golden test data for every scenario, from the pinned
# SUMO version. This is the ONLY thing that needs SUMO. Run it deliberately, on a
# network-enabled VM, then COMMIT the produced goldens.
#
# For each scenario directory under /scenarios that contains a config.sumocfg it
# produces, next to the inputs:
#   golden.fcd.xml       -- full trajectory dump (the behavioral ground truth)
#   golden.state.xml     -- fully-resolved vehicle/vType parameters at t=1
#                           (the initialization ground truth: catches vType-default
#                            bugs directly instead of via drifting trajectories)
#   golden.tripinfo.xml  -- GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2): per-vehicle arrival
#                           record (arrivalLane/arrivalPos/duration/routeLength/waitingTime/
#                           timeLoss/...), ONLY for a scenario that opts in via a committed
#                           sentinel file `.wants-tripinfo` in its directory -- every OTHER
#                           scenario's golden set is UNCHANGED by this addition (verified by
#                           `git status` showing no diff on any pre-existing golden.*).
#   provenance.txt       -- SUMO version, exact command, date, input file hashes
#                           (so goldens are trustworthy and staleness is detectable)
#
# DETERMINISM: goldens for phase-1 parity are generated with randomness stripped.
# Each scenario's config is expected to set sigma=0, fixed depart, Euler stepping,
# and teleport off. This script does not override the config; it trusts the
# committed scenario. See DESIGN.md "determinism ladder".
#
# The VM is volatile: the SUMO install here does not persist and does not need to.
# The committed goldens carry all ground truth forward to the offline test loop.
#
# USAGE: scripts/regen-goldens.sh [scenario-dir-or-glob-root]
#   With no argument, regenerates EVERY scenario under scenarios/ (the default, full sweep).
#   With an argument, scopes the sweep to just that directory (e.g.
#   `scripts/regen-goldens.sh scenarios/66-tripinfo-arrivallane`) -- use this for a SINGLE new
#   scenario so `generated_utc`/hashes in every OTHER scenario's provenance.txt are left
#   untouched (a full-sweep re-run is safe/idempotent for fcd/state/tripinfo content, but
#   provenance.txt's timestamp always changes, which would needlessly dirty `git status` for
#   scenarios that did not actually change).

set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

# shellcheck disable=SC1091
source "$REPO_ROOT/SUMO_VERSION"
: "${SUMO_VERSION:?Set SUMO_VERSION in $REPO_ROOT/SUMO_VERSION}"

# Step 1: ensure SUMO is present (volatile install).
"$REPO_ROOT/scripts/install-sumo.sh"

RAW_SCENARIOS_DIR="${1:-$REPO_ROOT/scenarios}"
if [[ ! -d "$RAW_SCENARIOS_DIR" ]]; then
  echo "ERROR: no scenarios directory at $RAW_SCENARIOS_DIR" >&2
  exit 1
fi
# Canonicalize to absolute -- the per-scenario loop below `cd`s into each $SCEN_DIR before
# invoking sumo, so a relative $CFG (derived from a relative SCENARIOS_DIR) would resolve
# against the WRONG directory after that cd. An absolute SCENARIOS_DIR keeps every derived path
# absolute throughout.
SCENARIOS_DIR="$(cd "$RAW_SCENARIOS_DIR" && pwd)"

# Portable-ish sha256 helper.
hash_file() {
  if command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    shasum -a 256 "$1" | awk '{print $1}'
  fi
}

STATE_TIME="${STATE_TIME:-1}"   # step at which to dump resolved parameters

# FCD output precision (decimal places). SUMO's default is 2, which is COARSER than the
# per-scenario parity tolerance (e.g. 1e-3) and would make the golden a lossy, 2-decimal
# truncation of SUMO's full-precision internal trajectory -- capping real parity sensitivity
# at ~5e-3 no matter what tolerance.json says. We raise it well above the tolerance so the
# committed golden carries enough digits for the tolerance to be a genuine bar. The engine
# emits full double precision and must NOT round to match a coarse golden.
FCD_PRECISION="${FCD_PRECISION:-6}"
GENERATED_ANY=0

# A scenario is any directory containing config.sumocfg.
while IFS= read -r -d '' CFG; do
  SCEN_DIR="$(dirname "$CFG")"
  SCEN_NAME="$(basename "$SCEN_DIR")"
  echo "==> Scenario: ${SCEN_NAME}"

  FCD_OUT="$SCEN_DIR/golden.fcd.xml"
  STATE_OUT="$SCEN_DIR/golden.state.xml"
  PROV_OUT="$SCEN_DIR/provenance.txt"

  # FCD includes lane-relative pos + speed AND global x/y/angle so the harness can
  # compare at either fidelity (see DESIGN.md "layered comparison metric").
  SUMO_CMD=(sumo
    -c "$CFG"
    --fcd-output "$FCD_OUT"
    --fcd-output.acceleration
    --precision "$FCD_PRECISION"
    --save-state.times "$STATE_TIME"
    --save-state.files "$STATE_OUT"
    --no-step-log true
  )

  # GAP-2: opt-in tripinfo golden, gated on the scenario's own committed `.wants-tripinfo`
  # sentinel -- keeps every scenario that does NOT carry the sentinel byte-identical (no new
  # --tripinfo-output flag added to its SUMO_CMD at all).
  TRIPINFO_OUT=""
  if [[ -f "$SCEN_DIR/.wants-tripinfo" ]]; then
    TRIPINFO_OUT="$SCEN_DIR/golden.tripinfo.xml"
    SUMO_CMD+=(--tripinfo-output "$TRIPINFO_OUT")
  fi

  # Phase 2 (sublane): SUMO does NOT emit posLat in the default FCD attribute set, even with the
  # sublane model active. A scenario with <lateral-resolution value="R"/> (R > 0) is a sublane
  # scenario whose golden must carry the lateral position, so request the explicit attribute list
  # (which REPLACES the default set -- hence it must re-list x/y/angle/speed/pos/lane and
  # acceleration alongside posLat). Phase-1 scenarios (no lateral-resolution, or 0) keep the
  # default set unchanged.
  LATRES="$(grep -oE 'lateral-resolution[^>]*value="[^"]*"' "$CFG" | grep -oE 'value="[^"]*"' | grep -oE '[0-9.]+' | head -1 || true)"
  if [[ -n "${LATRES:-}" ]] && awk "BEGIN{exit !(${LATRES} > 0)}"; then
    SUMO_CMD+=(--fcd-output.attributes x,y,angle,speed,pos,lane,posLat,acceleration)
  fi

  echo "    ${SUMO_CMD[*]}"
  ( cd "$SCEN_DIR" && "${SUMO_CMD[@]}" )

  # Provenance: what produced these goldens, so a future reader can trust/reproduce
  # them and detect staleness (input changed but goldens not regenerated).
  {
    echo "sumo_version=${SUMO_VERSION}"
    echo "sumo_version_reported=$(sumo --version 2>&1 | head -n 1)"
    echo "generated_utc=$(date -u +"%Y-%m-%dT%H:%M:%SZ")"
    echo "command=${SUMO_CMD[*]}"
    echo "state_time=${STATE_TIME}"
    echo "# input file hashes (sha256):"
    for f in "$SCEN_DIR"/*.net.xml "$SCEN_DIR"/*.rou.xml "$SCEN_DIR"/*.sumocfg; do
      [[ -e "$f" ]] || continue
      echo "input=$(basename "$f") sha256=$(hash_file "$f")"
    done
  } > "$PROV_OUT"

  if [[ -n "$TRIPINFO_OUT" ]]; then
    echo "    wrote: $(basename "$FCD_OUT"), $(basename "$STATE_OUT"), $(basename "$TRIPINFO_OUT"), $(basename "$PROV_OUT")"
  else
    echo "    wrote: $(basename "$FCD_OUT"), $(basename "$STATE_OUT"), $(basename "$PROV_OUT")"
  fi
  GENERATED_ANY=1
done < <(find "$SCENARIOS_DIR" -name config.sumocfg -print0 | sort -z)

if [[ "$GENERATED_ANY" -eq 0 ]]; then
  echo "WARNING: no scenarios with config.sumocfg found under $SCENARIOS_DIR." >&2
fi

echo
echo "==> Done. Review diffs, then COMMIT the golden files:"
echo "      git add scenarios/**/golden.fcd.xml scenarios/**/golden.state.xml scenarios/**/golden.tripinfo.xml scenarios/**/provenance.txt"
echo "      git commit -m 'Regenerate goldens (SUMO ${SUMO_VERSION})'"
