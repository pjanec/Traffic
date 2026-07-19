#!/usr/bin/env bash
# gen-demos.sh
# ------------
# Single source of truth for the auto-generated interactive-demo gallery: builds a CURATED,
# CATEGORIZED "one per feature" set of self-contained `replay.html`-style pages (vanilla Canvas
# 2D, no server, no SUMO — see `src/Sim.Viz`) into `site/<slug>.html`, plus a `site/index.html`
# landing page grouped by category. Both the `demos` GitHub Actions workflow (deploy to Pages)
# and humans run exactly this script — see docs/DEMOS.md.
#
#   scripts/gen-demos.sh
#   open site/index.html
#
# Never modifies committed scenario files: any `replay.html` the tools write into a scenario dir
# is copied out to `site/` and then restored — `git checkout --` if that path is git-tracked,
# otherwise `rm -f` (it was never committed there); any other regenerated scenario artifact
# (`engine.fcd.xml`, `combined.fcd.xml`, ...) is deleted afterwards. Resilient: a single broken
# demo is SKIPped (logged, not faked) and does not abort the rest of the gallery.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

SITE="$ROOT/site"
LOGDIR="$SITE/.logs"
rm -rf "$SITE"
mkdir -p "$SITE" "$LOGDIR"

echo "==> Building Sim.Viz / Sim.Run / Sim.ExtDemo (Release)…"
dotnet build src/Sim.Viz -c Release -v q
dotnet build src/Sim.Run -c Release -v q
dotnet build src/Sim.ExtDemo -c Release -v q

run() { dotnet run -c Release --no-build --project "$1" -- "${@:2}"; }

# restore_replay <scenarioDir>
# The tools always write <scenarioDir>/replay.html. If that path is already git-tracked (a few
# scenarios ship a committed sample replay), put the committed content back; otherwise the file
# never belonged there, so just remove it. Either way scenarios/ ends up pristine.
restore_replay() {
  local d="$1"
  if git ls-files --error-unmatch "$d/replay.html" >/dev/null 2>&1; then
    git checkout -- "$d/replay.html"
  else
    rm -f "$d/replay.html"
  fi
}

produced_slugs=()
produced_titles=()
produced_descs=()
produced_cats=()
skipped_slugs=()
skipped_reasons=()

# try <slug> <title> <description> <category> <fn> [fn-args...]
# Runs "<fn> [fn-args...]"; on success records slug/title/desc/category as produced, on failure
# records a SKIP. The command runs under `set -e` internally, but as the tested command of this
# `if`, a failure partway through does not abort the whole script — only that one demo is skipped.
try() {
  local slug="$1" title="$2" desc="$3" cat="$4"
  shift 4
  local log="$LOGDIR/$slug.log"
  if "$@" >"$log" 2>&1; then
    echo "OK   $slug"
    produced_slugs+=("$slug")
    produced_titles+=("$title")
    produced_descs+=("$desc")
    produced_cats+=("$cat")
  else
    echo "SKIP $slug (generation failed — see $log)"
    skipped_slugs+=("$slug")
    skipped_reasons+=("generation failed")
  fi
}

# --- generic demo bodies, one per command pattern -------------------------------------------
# Each renders straight from committed scenario inputs (net + golden FCD, or the engine run
# fresh) and never leaves the scenarios/ tree modified.

# demo_golden <scenarioDir> <slug>
# Sim.Viz reads the scenario's committed golden.fcd.xml directly.
demo_golden() {
  local d="$1" slug="$2"
  run src/Sim.Viz "$d"
  cp "$d/replay.html" "$SITE/$slug.html"
  restore_replay "$d"
}

# demo_run <scenarioDir> <slug>
# Sim.Run generates a fresh engine.fcd.xml, then Sim.Viz renders it.
demo_run() {
  local d="$1" slug="$2"
  run src/Sim.Run "$d"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/$slug.html"
  restore_replay "$d"
  rm -f "$d/engine.fcd.xml"
}

# demo_run_warmup <scenarioDir> <slug> <warmupSteps>
# Same as demo_run, but passes --warmup N to Sim.Run first: the recorded FCD's frame 0 starts from
# an already-populated timeline (Engine.WarmUp -- see src/Sim.Run/Program.cs) instead of ramping up
# from empty.
demo_run_warmup() {
  local d="$1" slug="$2" warmup="$3"
  run src/Sim.Run "$d" --warmup "$warmup"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/$slug.html"
  restore_replay "$d"
  rm -f "$d/engine.fcd.xml"
}

# demo_ext <scenarioDir> <slug>
# Sim.ExtDemo generates a combined (engine + external-agent) engine.fcd.xml, then Sim.Viz renders it.
demo_ext() {
  local d="$1" slug="$2"
  run src/Sim.ExtDemo "$d"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/$slug.html"
  restore_replay "$d"
  rm -f "$d/engine.fcd.xml" "$d/combined.fcd.xml" "$d/playwright_screenshot.png"
}

# demo_ext_args <scenarioDir> <slug> [extra Sim.ExtDemo args...]
# Same as demo_ext, but forwards extra CLI flags to Sim.ExtDemo -- e.g. --reroute-threshold N to
# opt the B3 live-reroute trigger in for a scenario whose external-agents.json places a persistent
# obstacle on the vehicle's nominal route (the flag is additive/CLI-only, see Sim.ExtDemo/Program.cs;
# omitting it reproduces demo_ext's own inert-by-default behavior).
demo_ext_args() {
  local d="$1" slug="$2"
  shift 2
  run src/Sim.ExtDemo "$d" "$@"
  run src/Sim.Viz "$d" --fcd "$d/engine.fcd.xml"
  cp "$d/replay.html" "$SITE/$slug.html"
  restore_replay "$d"
  rm -f "$d/engine.fcd.xml" "$d/combined.fcd.xml" "$d/playwright_screenshot.png"
}

# demo_evac <slug>
# Sim.Viz's dedicated --evac-organic mode writes straight to the site dir; no scenario dir at all.
demo_evac() {
  local slug="$1"
  run src/Sim.Viz --evac-organic "$SITE/$slug.html"
}

# demo_evac_district <slug>
# P5-1(B): Sim.Viz's dedicated --evac-district mode (pedestrian panic evac routed onto
# Sim.Pedestrians over the P5-PRE walkable net) writes straight to the site dir; no scenario dir,
# no vehicles.
demo_evac_district() {
  local slug="$1"
  run src/Sim.Viz --evac-district "$SITE/$slug.html"
}

# demo_ped <mode> <slug>
# Sim.Viz's dedicated --ped-<mode> modes build a self-contained pedestrian showcase (crowd + LOD +
# routing + parking) straight to the site dir; no scenario-dir round-trip, no external SUMO run.
demo_ped() {
  local mode="$1" slug="$2"
  run src/Sim.Viz "--ped-$mode" "$SITE/$slug.html"
}

# --- the curated set: category | slug | title | description | scenario dir | pattern ---------
# One row per feature. Categories are emitted as section headings on the gallery index in this
# same order; demos within a category keep this order too.

echo "==> Generating curated, categorized demo gallery…"

# Car-following
try krauss-free-flow "Krauss free-flow" \
  "A single vehicle cruising free-flow on an open road under SUMO's default Krauss car-following model — the simplest parity scenario." \
  "Car-following" demo_golden scenarios/01-single-free-flow krauss-free-flow
try platoon-shockwave "Platoon shockwave" \
  "A dense platoon under Krauss car-following propagates a braking shockwave back through the queue." \
  "Car-following" demo_golden scenarios/05-platoon-shockwave platoon-shockwave
try idm "IDM car-following" \
  "The Intelligent Driver Model's smooth, desired-gap-based acceleration profile in free flow and approach." \
  "Car-following" demo_golden scenarios/22-idm-carfollow idm
try idmm "IDMM (memory)" \
  "IDM with driver memory: past following experience biases the desired-gap parameter over time." \
  "Car-following" demo_golden scenarios/25-idmm-carfollow idmm
try acc "ACC" \
  "Adaptive Cruise Control car-following — a fixed time-gap radar-follower model." \
  "Car-following" demo_golden scenarios/23-acc-carfollow acc
try cacc "CACC (cooperative)" \
  "Cooperative Adaptive Cruise Control — vehicle-to-vehicle coordination tightens the following gap beyond plain ACC." \
  "Car-following" demo_golden scenarios/24-cacc-carfollow cacc

# Lane changing & overtaking
try keep-right "Keep-right lane change" \
  "A faster vehicle merges back to the right-hand lane once past a slower one, per the keep-right lane-change strategy." \
  "Lane changing & overtaking" demo_golden scenarios/07-keep-right-change keep-right
try overtake "Same-direction overtaking" \
  "A trailing vehicle changes lanes to overtake a slower leader ahead of it." \
  "Lane changing & overtaking" demo_golden scenarios/12-overtake overtake
try continuous-lane-change "Continuous lane change" \
  "Sublane-resolution continuous lateral motion during a lane change, instead of an instantaneous lane snap." \
  "Lane changing & overtaking" demo_golden scenarios/43-continuous-lanechange continuous-lane-change
try multilane-keep-right "Multilane keep-right on arrival" \
  "Vehicles across several lanes converge toward the right-hand lane as they approach their arrival edge." \
  "Lane changing & overtaking" demo_golden scenarios/45-multilane-keepright-arrival multilane-keep-right
try sublane-mixed "Sublane / laneless mixed" \
  "Sublane and laneless vehicles share the same road, each governed by its own lateral-motion model." \
  "Lane changing & overtaking" demo_golden scenarios/65-mixed-sublane sublane-mixed
try overtake-opposite "Opposite-direction overtaking (lcOpposite)" \
  "A fast lcOpposite vehicle held up by a slow leader spills across the road centerline into the oncoming lane, passes it, and returns -- overtaking via oncoming traffic's lane rather than a same-direction second lane." \
  "Lane changing & overtaking" demo_run scenarios/_bench/overtake-opposite-demo overtake-opposite

# Junctions & right-of-way
try priority-junction "Priority junction" \
  "Right-of-way negotiation at an unsignalized priority junction." \
  "Junctions & right-of-way" demo_golden scenarios/11-priority-junction priority-junction
try right-before-left "Right-before-left" \
  "An unsignalized junction resolved by the right-before-left rule instead of an explicit priority road." \
  "Junctions & right-of-way" demo_golden scenarios/26-right-before-left right-before-left
try all-way-stop "All-way stop" \
  "Every approach yields in turn at an all-way-stop-controlled junction." \
  "Junctions & right-of-way" demo_golden scenarios/27-allway-stop all-way-stop
try roundabout "Roundabout" \
  "Circulating traffic holds priority over entering traffic at a roundabout." \
  "Junctions & right-of-way" demo_golden scenarios/32-roundabout roundabout
try on-ramp-merge "On-ramp merge" \
  "A merging vehicle from an on-ramp negotiates a gap into mainline through traffic." \
  "Junctions & right-of-way" demo_golden scenarios/19-onramp-merge on-ramp-merge
try multilane-junction-turn "Multilane junction turn" \
  "Turning movements across a multilane junction with lane-to-lane connections." \
  "Junctions & right-of-way" demo_golden scenarios/44-multilane-junction-turn multilane-junction-turn

# Traffic lights
try traffic-light "Static traffic light" \
  "Vehicles queuing and releasing at a signalized intersection with SUMO-native signal heads on a fixed program." \
  "Traffic lights" demo_golden scenarios/09-traffic-light traffic-light
try traffic-light-actuated "Actuated (detector) traffic light" \
  "A detector-actuated signal program extends or truncates phases in response to arriving traffic." \
  "Traffic lights" demo_golden scenarios/35-actuated-tls traffic-light-actuated

# Emergency & priority vehicles
try emergency-corridor "Emergency rescue lane (bluelight)" \
  "Traffic forms a rescue lane ahead of an approaching emergency vehicle running its bluelight device." \
  "Emergency & priority vehicles" demo_run scenarios/_bench/emergency-corridor-demo emergency-corridor
try emergency-run-red "Emergency vehicle runs red" \
  "An emergency vehicle passes a red signal while ordinary traffic yields right-of-way to it." \
  "Emergency & priority vehicles" demo_golden scenarios/16-emergency-red emergency-run-red

# Rail
try rail-free-flow "Rail free-run" \
  "A train running free on open track under rail-specific kinematics." \
  "Rail" demo_golden scenarios/47-rail-free-flow rail-free-flow
try rail-bidirectional "Rail bidirectional single track" \
  "Two trains meet on a bidirectional single track, one yielding at a passing point." \
  "Rail" demo_golden scenarios/49-rail-bidi-meet rail-bidirectional
try rail-signal-block "Rail signal block reservation" \
  "A rail signal reserves a track block ahead of a train and holds a following train clear of it." \
  "Rail" demo_golden scenarios/50-rail-signal-meet rail-signal-block
try rail-level-crossing "Rail level crossing" \
  "A road-rail level crossing barrier closes to road traffic as a train approaches." \
  "Rail" demo_run scenarios/_bench/rail-crossing-demo rail-level-crossing
try rail-traction "Rail traction curve" \
  "A train's speed-dependent traction/braking curve shapes its acceleration profile, unlike a road vehicle's flat bounds." \
  "Rail" demo_golden scenarios/52-rail-traction rail-traction

# Reactive & external agents
try external-agents "External non-SUMO agents (5 reactions)" \
  "Five external (non-SUMO) agent reactions — stop, swerve, spill, follow, junction-yield — injected alongside engine traffic." \
  "Reactive & external agents" demo_ext scenarios/_bench/ext-showcase external-agents

# Panic evacuation
try evac-organic "Panic evacuation (organic town)" \
  "A realistic organic town under panic evacuation: congestion plus a large local foot exodus." \
  "Panic evacuation" demo_evac evac-organic
try evac-district "Panic evacuation (district, routed on foot)" \
  "Panicked pedestrians on a synthetic sidewalk district route to their nearest safe-zone corner ALONG the real sidewalk grid -- bending around the blocks, never radially through them -- forced high-power (reactive full-ORCA) via the Sim.Pedestrians PedestrianWorld facade (EvacDistrictDirector, P5-1(B))." \
  "Panic evacuation" demo_evac_district evac-district

# Pedestrians
try ped-crossing-gate "Crossing gate (car stops for pedestrian)" \
  "Pedestrians queue at a signalized crosswalk while the light is red, then surge across on the real walk phase; a car crossing the junction on its own green halts for a pedestrian in its lane -- emergent ORCA vehicle/pedestrian avoidance, not a scripted stop (CrossingGate + Engine.CrowdSource)." \
  "Pedestrians" demo_ped crossing-gate ped-crossing-gate
try ped-lod-promotion "Sim-LOD promotion (low-power to full ORCA)" \
  "Low-power PathArc pedestrians walk fixed sidewalk routes at O(1) cost; a moving interest source promotes any it nears to reactive full-ORCA high-power agents and demotes them once it passes (PedLodManager + InterestField)." \
  "Pedestrians" demo_ped lod-promotion ped-lod-promotion
try ped-od-routing "Origin-destination routed crowd" \
  "PedDemand spawns pedestrians on a Poisson process, each routed origin-to-destination across the junction's real sidewalks, crossings, and walkingareas via SumoNavMesh, then despawned on arrival (PedDemand + PedLodManager)." \
  "Pedestrians" demo_ped od-routing ped-od-routing
try ped-dodge "Obstacle dodge" \
  "A bidirectional pedestrian stream is routed through the clear corridor beside a static box obstacle by an off-line waypoint, while ORCA keeps each ped clear of the box and of oncoming peds -- the stream arcs around the box and re-forms past it (OrcaCrowd BoxObstacle + PedRouteController)." \
  "Pedestrians" demo_ped dodge ped-dodge
try ped-reroute "Crossing reroute" \
  "A pedestrian pair walks a signalized crossing back and forth; a blocker box appears over the crossing partway through and RerouteDriver recomputes a detour through the walkingarea ring for exactly the affected pedestrian, then the blocker clears (PedRouteController + BlockerRegistry + RerouteDriver)." \
  "Pedestrians" demo_ped reroute ped-reroute
try ped-parking "Parking lot (car/pedestrian mutual avoidance)" \
  "A non-holonomic car maneuvers into a parking slot among static parked cars while pedestrians weave the drive aisle; one walker boards the car once it parks, another alights once it returns to the exit (LotCoupling)." \
  "Pedestrians" demo_ped parking ped-parking
try ped-liveliness "Liveliness (activity timeline replay)" \
  "Low-power pedestrians walk, pause to sip a drink, sit at a table, then vanish into a building and re-emerge -- every pose and animation tag is a pure deterministic function of (ActivityTimeline, now), so server and IG replay identically (LIVE-POC-1, ActivityTimeline.PoseAt)." \
  "Pedestrians" demo_ped liveliness ped-liveliness
try ped-social "Meet & talk (pre-scheduled two-ped interaction)" \
  "Pairs of walkers on converging approaches are paired up front by SocialPlanner: both step aside to opposite sides of the flow, face each other, and talk for a shared time window, then resume their own onward route -- authored together at schedule time with no runtime negotiation, so it stays exactly as low-power and IG-reproducible as a solo walker (LIVE-POC-2, ActivityTimeline Interact)." \
  "Pedestrians" demo_ped social ped-social
try ped-waiter "Waiter (micro-scenario actor)" \
  "A waiter emerges from a restaurant's service door, walks to a table in a seed-varied rotation, dwells to serve it, walks back, and dwells inside (no disc) before the next round -- a single templated, looping ActivityTimeline, so the scripted actor stays exactly as low-power and server==IG-reconstructable as a solo walker (LIVE-POC-3, WaiterScenario)." \
  "Pedestrians" demo_ped waiter ped-waiter
try ped-lively-crowd "Lively crowd (routed + activity timelines)" \
  "The same Poisson-demand O-D crowd as the plain OD-routing demo, now graduated to LIVE-PROD-1b: PedDemand seeds Pause (\"sip\") beats into each spawn's route as an ActivityTimeline, so the routed crowd occasionally stops in place mid-transit -- still low-power, still server==IG, still routed across the junction's real sidewalks/crossings/walkingareas (PedDemand + PedLodManager.AddPedLively)." \
  "Pedestrians" demo_ped lively-crowd ped-lively-crowd
try ped-remote "Remote (over the wire)" \
  "The same sweeping-interest-source LOD promotion demo, but every disc is drawn from a PedRemoteReconstructor fed by the real DR-error-gated PedReplicationPublisher over an InMemoryPedReplicationBus -- each path is sent once, high-power positions are gated onto the wire only when dead-reckoning would drift out of tolerance, and a playout-delay render clock plus capped-correction smoothing reconstruct every pose (including the promotion/demotion DR-switch) with no visible pop: this crowd is literally reconstructed from the one multicast stream, not the sim (P3-3)." \
  "Pedestrians" demo_ped remote ped-remote

# Integration & driver behavior
try ballistic-integration "Ballistic integration" \
  "Ballistic (exact-kinematics) position integration compared to SUMO's default Euler stepping." \
  "Integration & driver behavior" demo_golden scenarios/21-ballistic-freeflow ballistic-integration
try action-step-length "Reaction time (actionStepLength)" \
  "A driver re-evaluates its car-following decision only every actionStepLength seconds instead of every simulation step." \
  "Integration & driver behavior" demo_golden scenarios/28-actionstep action-step-length
try dawdle-stochastic "Dawdle / sigma stochasticity" \
  "Krauss's sigma-driven random dawdle perturbs following speed from step to step." \
  "Integration & driver behavior" demo_run scenarios/17-dawdle-freeflow dawdle-stochastic
try probabilistic-flow "Probabilistic flow insertion" \
  "Vehicles are inserted stochastically by per-step probability instead of on a fixed period." \
  "Integration & driver behavior" demo_run scenarios/58-flow-probability probabilistic-flow
try reroute "Rerouting around a blockage" \
  "A vehicle's routing device detects a persistent obstacle ahead on its assigned route and recomputes a different path around it (diamond detour), instead of queuing behind it forever." \
  "Integration & driver behavior" demo_ext_args scenarios/_bench/reroute-demo reroute --reroute-threshold 5
try warm-start "Warm-start snapshot" \
  "A scaled town rendered from an already-populated timeline (Engine.WarmUp) so frame 0 shows dozens of vehicles already moving, instead of ramping up from an empty network." \
  "Integration & driver behavior" demo_run_warmup scenarios/_bench/city-30 warm-start 60

# City scale
try city-town "Scaled town (~30 vehicles)" \
  "A 3x3-grid town at ~30 concurrent vehicles — engine run rendered against the SUMO aggregate-parity reference." \
  "City scale" demo_run scenarios/_bench/city-30 city-town
try city-multilane "Large multilane city (~400 vehicles)" \
  "A larger organic multilane city network under ~400 concurrent vehicles." \
  "City scale" demo_run scenarios/_bench/city-organic-L2 city-multilane
try city-signalized "Signalized city (~1000 vehicles)" \
  "A mixed signalized city network at ~1000 concurrent vehicles, exercising traffic lights at city scale." \
  "City scale" demo_run scenarios/_bench/city-mixed-1k city-signalized

# --- summary ---------------------------------------------------------------------------------

echo ""
echo "==> Summary: ${#produced_slugs[@]} produced, ${#skipped_slugs[@]} skipped."
for i in "${!produced_slugs[@]}"; do echo "  OK   ${produced_slugs[$i]}"; done
for i in "${!skipped_slugs[@]}"; do echo "  SKIP ${skipped_slugs[$i]} (${skipped_reasons[$i]})"; done

if [ "${#produced_slugs[@]}" -eq 0 ]; then
  echo "::error::no demos were produced — refusing to write an empty gallery"
  exit 1
fi

# --- index.html --------------------------------------------------------------------------------

echo "==> Writing site/index.html…"

{
  cat <<'HTML_HEAD'
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>SumoSharp — interactive demo gallery</title>
<style>
  :root { color-scheme: light dark; }
  body {
    margin: 0; padding: 2.5rem 1.5rem 4rem; min-height: 100vh;
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    background: #f6f7f9; color: #1b1f24;
  }
  @media (prefers-color-scheme: dark) {
    body { background: #14171a; color: #e6e8eb; }
  }
  header { max-width: 1100px; margin: 0 auto 2.5rem; }
  h1 { font-size: 1.9rem; margin: 0 0 0.5rem; }
  p.lede { margin: 0; opacity: 0.75; line-height: 1.5; }
  main { max-width: 1100px; margin: 0 auto; }
  section.category { margin: 0 0 2.4rem; }
  section.category h2 {
    font-size: 1.15rem; margin: 0 0 0.9rem; padding-bottom: 0.4rem;
    border-bottom: 1px solid rgba(0,0,0,0.1);
  }
  @media (prefers-color-scheme: dark) {
    section.category h2 { border-bottom-color: rgba(255,255,255,0.12); }
  }
  .grid {
    display: grid; grid-template-columns: repeat(auto-fill, minmax(260px, 1fr)); gap: 1.1rem;
  }
  a.card {
    display: block; padding: 1.25rem 1.35rem; border-radius: 0.75rem; text-decoration: none;
    color: inherit; background: #ffffff; border: 1px solid rgba(0,0,0,0.08);
    box-shadow: 0 1px 2px rgba(0,0,0,0.04); transition: transform 0.12s ease, box-shadow 0.12s ease;
  }
  @media (prefers-color-scheme: dark) {
    a.card { background: #1e2226; border-color: rgba(255,255,255,0.08); box-shadow: none; }
  }
  a.card:hover { transform: translateY(-2px); box-shadow: 0 6px 16px rgba(0,0,0,0.12); }
  a.card h3 { margin: 0 0 0.4rem; font-size: 1.05rem; }
  a.card p { margin: 0; font-size: 0.9rem; opacity: 0.75; line-height: 1.4; }
  footer { max-width: 1100px; margin: 2.5rem auto 0; font-size: 0.8rem; opacity: 0.6; }
  footer a { color: inherit; }
</style>
</head>
<body>
<header>
  <h1>SumoSharp — interactive demo gallery</h1>
  <p class="lede">
    Self-contained, browser-only traffic-simulation replays (vanilla Canvas 2D — no install, no
    server, no SUMO required). Click a demo to open it; each page has its own play / pause / scrub
    / speed / zoom &amp; pan controls.
  </p>
</header>
<main>
HTML_HEAD

  # Emit one <section> per category, in first-seen order, each with its own card grid.
  declare -a seen_cats=()
  for i in "${!produced_slugs[@]}"; do
    cat="${produced_cats[$i]}"
    already=0
    for sc in "${seen_cats[@]:-}"; do [ "$sc" = "$cat" ] && already=1 && break; done
    [ "$already" -eq 1 ] && continue
    seen_cats+=("$cat")

    printf '  <section class="category">\n    <h2>%s</h2>\n    <div class="grid">\n' "$cat"
    for j in "${!produced_slugs[@]}"; do
      [ "${produced_cats[$j]}" = "$cat" ] || continue
      slug="${produced_slugs[$j]}"
      title="${produced_titles[$j]}"
      desc="${produced_descs[$j]}"
      printf '      <a class="card" href="%s.html">\n        <h3>%s</h3>\n        <p>%s</p>\n      </a>\n' "$slug" "$title" "$desc"
    done
    printf '    </div>\n  </section>\n'
  done

  cat <<'HTML_TAIL'
</main>
<footer>
  Generated by <code>scripts/gen-demos.sh</code> — see <code>docs/DEMOS.md</code> in the repo.
</footer>
</body>
</html>
HTML_TAIL
} > "$SITE/index.html"

echo ""
echo "Done. Produced ${#produced_slugs[@]} demo(s) in $SITE/."
echo "open $SITE/index.html"
echo "(CI deploys this same directory to GitHub Pages — see .github/workflows/demos.yml)"
