# Interactive demo gallery

An auto-generated gallery of interactive, self-contained traffic-simulation replays. Each demo is a
single `replay.html`-style page (vanilla Canvas 2D — width-accurate lanes, junction fills,
SUMO-native signal heads, true-size oriented vehicle boxes, play/pause/scrub/speed/zoom/pan). No
install, no server, no SUMO: it runs entirely in the browser once the page is loaded.

<!-- Pages base URL: update ONLY if the repo is not named "SumoSharp". Everything else is relative. -->
https://pjanec.github.io/SumoSharp/

## Demos

One demo per feature, grouped by category (the gallery's landing page uses the same grouping).

| Category | Demo | Shows |
|---|---|---|
| Car-following | Krauss free-flow | A single vehicle cruising free-flow on an open road under SUMO's default Krauss car-following model — the simplest parity scenario. |
| Car-following | Platoon shockwave | A dense platoon under Krauss car-following propagates a braking shockwave back through the queue. |
| Car-following | IDM car-following | The Intelligent Driver Model's smooth, desired-gap-based acceleration profile in free flow and approach. |
| Car-following | IDMM (memory) | IDM with driver memory: past following experience biases the desired-gap parameter over time. |
| Car-following | ACC | Adaptive Cruise Control car-following — a fixed time-gap radar-follower model. |
| Car-following | CACC (cooperative) | Cooperative Adaptive Cruise Control — vehicle-to-vehicle coordination tightens the following gap beyond plain ACC. |
| Lane changing & overtaking | Keep-right lane change | A faster vehicle merges back to the right-hand lane once past a slower one, per the keep-right lane-change strategy. |
| Lane changing & overtaking | Same-direction overtaking | A trailing vehicle changes lanes to overtake a slower leader ahead of it. |
| Lane changing & overtaking | Continuous lane change | Sublane-resolution continuous lateral motion during a lane change, instead of an instantaneous lane snap. |
| Lane changing & overtaking | Multilane keep-right on arrival | Vehicles across several lanes converge toward the right-hand lane as they approach their arrival edge. |
| Lane changing & overtaking | Sublane / laneless mixed | Sublane and laneless vehicles share the same road, each governed by its own lateral-motion model. |
| Lane changing & overtaking | Opposite-direction overtaking (lcOpposite) | A fast lcOpposite vehicle held up by a slow leader spills across the road centerline into the oncoming lane, passes it, and returns — overtaking via oncoming traffic's lane rather than a same-direction second lane. |
| Junctions & right-of-way | Priority junction | Right-of-way negotiation at an unsignalized priority junction. |
| Junctions & right-of-way | Right-before-left | An unsignalized junction resolved by the right-before-left rule instead of an explicit priority road. |
| Junctions & right-of-way | All-way stop | Every approach yields in turn at an all-way-stop-controlled junction. |
| Junctions & right-of-way | Roundabout | Circulating traffic holds priority over entering traffic at a roundabout. |
| Junctions & right-of-way | On-ramp merge | A merging vehicle from an on-ramp negotiates a gap into mainline through traffic. |
| Junctions & right-of-way | Multilane junction turn | Turning movements across a multilane junction with lane-to-lane connections. |
| Traffic lights | Static traffic light | Vehicles queuing and releasing at a signalized intersection with SUMO-native signal heads on a fixed program. |
| Traffic lights | Actuated (detector) traffic light | A detector-actuated signal program extends or truncates phases in response to arriving traffic. |
| Emergency & priority vehicles | Emergency rescue lane (bluelight) | Traffic forms a rescue lane ahead of an approaching emergency vehicle running its bluelight device. |
| Emergency & priority vehicles | Emergency vehicle runs red | An emergency vehicle passes a red signal while ordinary traffic yields right-of-way to it. |
| Rail | Rail free-run | A train running free on open track under rail-specific kinematics. |
| Rail | Rail bidirectional single track | Two trains meet on a bidirectional single track, one yielding at a passing point. |
| Rail | Rail signal block reservation | A rail signal reserves a track block ahead of a train and holds a following train clear of it. |
| Rail | Rail level crossing | A road-rail level crossing barrier closes to road traffic as a train approaches. |
| Rail | Rail traction curve | A train's speed-dependent traction/braking curve shapes its acceleration profile, unlike a road vehicle's flat bounds. |
| Reactive & external agents | External non-SUMO agents (5 reactions) | Five external (non-SUMO) agent reactions — stop, swerve, spill, follow, junction-yield — injected alongside engine traffic. |
| Panic evacuation | Panic evacuation (organic town) | A realistic organic town under panic evacuation: congestion plus a large local foot exodus. |
| Integration & driver behavior | Ballistic integration | Ballistic (exact-kinematics) position integration compared to SUMO's default Euler stepping. |
| Integration & driver behavior | Reaction time (actionStepLength) | A driver re-evaluates its car-following decision only every actionStepLength seconds instead of every simulation step. |
| Integration & driver behavior | Dawdle / sigma stochasticity | Krauss's sigma-driven random dawdle perturbs following speed from step to step. |
| Integration & driver behavior | Probabilistic flow insertion | Vehicles are inserted stochastically by per-step probability instead of on a fixed period. |
| Integration & driver behavior | Rerouting around a blockage | A vehicle's routing device detects a persistent obstacle ahead on its assigned route and recomputes a different path around it (diamond detour), instead of queuing behind it forever. |
| Integration & driver behavior | Warm-start snapshot | A scaled town rendered from an already-populated timeline (`Engine.WarmUp`) so frame 0 shows dozens of vehicles already moving, instead of ramping up from an empty network. |
| City scale | Scaled town (~30 vehicles) | A 3x3-grid town at ~30 concurrent vehicles — engine run rendered against the SUMO aggregate-parity reference. |
| City scale | Large multilane city (~400 vehicles) | A larger organic multilane city network under ~400 concurrent vehicles. |
| City scale | Signalized city (~1000 vehicles) | A mixed signalized city network at ~1000 concurrent vehicles, exercising traffic lights at city scale. |
| Pedestrians | Crossing gate (car stops for pedestrian) | Pedestrians queue at a signalized crosswalk while the light is red, then surge across on the walk phase; a car halts for a pedestrian in its lane — emergent ORCA avoidance, not a scripted stop. |
| Pedestrians | Sim-LOD promotion (low-power to full ORCA) | A moving interest source promotes nearby low-power dead-reckoned pedestrians to reactive full-ORCA agents and demotes them once it passes. |
| Pedestrians | Origin-destination routed crowd | A Poisson O-D crowd routed across the junction's real sidewalks, crossings, and walkingareas on the baked navmesh. |
| Pedestrians | Obstacle dodge | A bidirectional pedestrian stream swerves around a static box obstacle via reciprocal ORCA avoidance. |
| Pedestrians | Crossing reroute | A blocker appears over a crossing and the affected pedestrians recompute a detour through the walkingarea ring. |
| Pedestrians | Parking lot (car/pedestrian mutual avoidance) | A car maneuvers among parked cars while pedestrians weave and board/alight — mutual car↔pedestrian yielding. |
| Pedestrians | Liveliness (activity timeline replay) | Deterministic activity timelines: each pedestrian walks, pauses (sips), sits at a table, and steps inside a building (rendering no disc while hidden). |
| Pedestrians | Meet & talk (pre-scheduled two-ped interaction) | Converging pairs meet, step aside, and talk — authored into both timelines up front, with no runtime negotiation. |
| Pedestrians | Waiter (micro-scenario actor) | A looping templated waiter emerges from a door, serves open-air tables, and disappears inside between rounds. |
| Pedestrians | Lively crowd (routed + activity timelines) | The routed O-D crowd with seeded pause beats spliced into each trip, so it occasionally stops in place — still low-power, still server==IG. |
| Pedestrians | Remote (over the wire) | The crowd rendered purely from the replication stream (DR-error-gated publish → byte loopback → reconstructor), server==IG with no promotion pop. |
| Pedestrians | Deterministic weave — no pass-through (on/off) | Two counterflowing streams share one sidewalk; toggle the deterministic weave to see opposing flows separate versus collapse onto the centreline and pass through each other. |
| Pedestrians | City: cars + weaving pedestrians | A routed weaving crowd on a real intersection's sidewalks/crossings sharing the scene with cars driving cross-traffic through the junction. |
| Pedestrians | Dense city: cars + weaving pedestrians | A busy ~900 m block of the synthetic demo-city: hundreds of weaving pedestrians on the real sidewalks plus a dense car flow on the signalized road grid. |
| Panic evacuation | Panic evacuation (district, routed on foot) | A deterministic ambient crowd panics on incident proximity and routes to its nearest safe zone along the real sidewalk grid (bending around blocks, not radially through them). |

The curated set is defined in `scripts/gen-demos.sh`; only demos that actually generate are listed
on the gallery's landing page (a broken demo is skipped and logged, never faked).

## Run the demos locally

```bash
scripts/gen-demos.sh
open site/index.html      # or just double-click it in a file browser
```

Windows users: run it via WSL or Git Bash (the script is bash-only; there is no `.ps1` twin).

`site/` is git-ignored — it is always regenerated, never committed.

## Maintainer setup

1. **Settings → Pages → Source: "GitHub Actions"** (one-time, per repo).
2. The gallery deploys automatically from `main` whenever `scenarios/**`, `src/**`, or
   `scripts/gen-demos.sh` change (see `.github/workflows/demos.yml`), or on demand via
   **Actions → demos → Run workflow**.
3. Feature branches never publish: the push trigger is restricted to `main`.

## Not part of the web gallery

The native desktop viewer (raylib + Dear ImGui, 10k-scale) is a separate desktop application, not
a browser page — see `docs/PACKAGES.md` and the "Live & native viewers" section of the README.

Also not part of the web gallery: [`demos/City3D`](../demos/City3D) — a Godot 4 (.NET) 3D city
viewer that consumes the SumoSharp packages from a local feed (local co-hosted + remote/DDS + a CLI
video wall); see [`demos/City3D/README.md`](../demos/City3D/README.md). It's a desktop/GPU Godot
app, not a browser page.
