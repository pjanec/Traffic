# Traffic — SUMO's microscopic traffic simulation, reimplemented in C# / .NET 8

A behaviorally-faithful, data-oriented reimplementation of [Eclipse SUMO](https://eclipse.dev/sumo/)
**1.20.0**'s microscopic mobility core in C# / .NET 8. The goal is **exact trajectory parity with
SUMO** on a cache-friendly, allocation-light, parallel-ready ECS engine — plus a few capabilities SUMO
doesn't have. Highlights below; precise scope after that.

> **Correctness bar:** every deterministic scenario matches a committed SUMO golden trajectory to
> **1e-3** on `[lane, pos, speed]`, every step. Stochastic features use a statistical/ensemble bar
> instead. The committed goldens *are* the executable spec — `dotnet test` never needs SUMO.

## Highlights

- **⚡ Faster than SUMO — bit-for-bit identical.** The simulation tick runs **3.6× single-threaded
  SUMO at 8 cores (3.1× at 4)** on a 7,600-vehicle city, and every parallel run reproduces the serial
  run's *exact* trajectories (same determinism hash). A faster **identical** answer, not an
  approximation. (Full core-scaling curve in *Scale*.)
- **🧩 Self-balancing spatial parallelism.** An opt-in domain decomposition splits the road network into
  a grid of regions that each own a **disjoint set of lanes** — lock-free by construction. Dynamic
  work-stealing over the regions **auto-balances load** as congestion moves around the map, and a
  vehicle crossing a region boundary is handed off *for free* (simply regrouped by its current lane the
  next step), with no explicit state transfer.
- **🏗️ Data-oriented ECS.** SUMO's algorithms on a cache-friendly struct-of-arrays layout: a
  zero-allocation hot path, a plan/execute double buffer, and a deferred command buffer — parallel-ready
  by design, not retrofitted.
- **🖥️ Runs anywhere .NET 8 runs.** Pure managed C# with no native dependencies — the engine and its
  offline parity suite build and run on **Linux and Windows** alike (any .NET 8 target); the only
  prerequisite is the `dotnet` SDK, never a SUMO install.
- **🚗 Broad, faithful coverage.** Six car-following models (Krauss / IDM / IDMM / ACC / CACC / Rail),
  full lane-changing, junction right-of-way (priority, right-before-left, all-way-stop, roundabouts,
  zipper merge), static **and** actuated traffic lights, and **first-class rail** (signals, level
  crossings, bidirectional single track, traction).
- **➕ Beyond SUMO.** Live external agents (pedestrians / crowds / detections that cars *react* to),
  emergency-vehicle give-way, opposite-direction overtaking, laneless/shaped mixed traffic, and a full
  **panic-evacuation** model (localized incident → flee → jam → abandon car → foot exodus, layered on
  the unchanged parity core).
- **🚶 A from-scratch pedestrian crowd layer.** Not a port of SUMO's person model — an independent,
  **network-distributable** crowd subsystem: a navmesh baked from the SUMO net, Poisson O-D demand, a
  two-level LOD (cheap dead-reckoned "ambient" peds vs. reactive full-ORCA ones, promoted on demand),
  activity-timeline liveliness (pause / sit / step inside), crosswalk signals, and a **deterministic
  lateral weave** so dense crowds thread shared sidewalks without passing through each other. Every
  low-power pedestrian's pose is a **pure function of `(route, seed, width, time)`**, so the server and
  every remote image-generator reconstruct it **bit-for-bit identically** — a route is broadcast *once*
  and thousands of ambient peds cost ~O(1) each over a tiny bandwidth budget. Default-off, so the
  vehicle parity goldens are byte-unchanged. Full index: [`docs/PEDESTRIANS.md`](docs/PEDESTRIANS.md).
- **👁️ Three ways to watch it run.** A self-contained **offline HTML replay** (Canvas 2D, no server, no
  SUMO); a **live dead-reckoned browser viewer** (`Sim.LiveHost` — the zero-install shareable demo,
  poses extrapolated between low-rate updates); and a **native 10k-scale C# viewer** (`Sim.Viewer` —
  raylib + Dear ImGui) that renders either the authoritative snapshot locally or a **dead-reckoned
  stream over DDS** (RTPS pub/sub, durable network geometry + low-rate traffic-light topics, view-only
  late-joining remote clients).

## Documentation & live demos

- **▶ Live demo gallery — interactive, in-browser, zero install:** **https://pjanec.github.io/SumoSharp/**
  — 50+ self-contained replays, **one per feature** (car-following, same- & opposite-direction
  overtaking, roundabouts, traffic lights, emergency vehicles, rail, rerouting, panic evacuation,
  **pedestrian crowds & the deterministic weave**, city-scale, …). One click each; regenerated
  automatically by CI. Index + local-run script
  (`scripts/gen-demos.sh`): [`docs/DEMOS.md`](docs/DEMOS.md).
- **📦 The NuGet packages & how they fit together:** [`docs/PACKAGES.md`](docs/PACKAGES.md) — the
  à-la-carte package map, "which packages do I install?", and composition diagrams. Runnable
  consumption examples: [`samples/`](samples/) (`HelloTraffic`, `StreamingLoopback`,
  `MotionReconstruction`, `EvacDemo`, `GameHostSample`).
- **📐 Design of record:** [`docs/DESIGN.md`](docs/DESIGN.md) (engine & parity) ·
  [`docs/SUMOSHARP-PACKAGING-DESIGN.md`](docs/SUMOSHARP-PACKAGING-DESIGN.md) (packaging) ·
  [`docs/SUMOSHARP-VIEWER-DR-SMOOTHING.md`](docs/SUMOSHARP-VIEWER-DR-SMOOTHING.md) (dead-reckoning &
  smoothing for custom/3D viewers) ·
  [`docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`](docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md) (the **kinematic
  vehicle-motion reconstruction** both viewers now smooth with).
- **🏙️ [`demos/City3D`](demos/City3D)** — a Godot 4 (.NET) 3D city viewer consuming the SumoSharp
  packages from a local feed.
- **🛰️ IgBridge** ([`docs/IGBRIDGE-DESIGN.md`](docs/IGBRIDGE-DESIGN.md)) — a producer-side feed for an
  **external 3D image generator** that has **no protocol for sophisticated predictive dead-reckoning** and
  consumes only plain `position / orientation / timestamp` samples (interpolating between its two most recent).
  IgBridge bakes all the smoothing in *before* the wire: it reconstructs SUMO's stepwise motion (junction
  facet-snaps, instant lane changes) into a continuous curve with the **same kinematic reconstructor the
  viewers use** and resamples it densely, so an IG that does no prediction of its own still shows artifact-free
  motion. Tuning methodology + metrics: [`docs/IGBRIDGE-METHODOLOGY.md`](docs/IGBRIDGE-METHODOLOGY.md).
- **🔧 Build & run it from a fresh clone** (build, tests, the demo viewer, the live browser viewer):
  see [Install → *See it run*](#install).

## Scope

- **What it ports:** the per-step simulation algorithms (car-following, lane-changing, junction
  right-of-way, traffic lights, rail), copied close to line-for-line from the vendored SUMO source.
- **What it deliberately rebuilds:** the *data structures*. SUMO's deep `MSVehicle` inheritance and
  pointer-linked leaders become struct-of-arrays with a plan/execute double buffer and a deferred
  command buffer — the point of the project, not a risky deviation.
- **What it does *not* do:** `netconvert` / OSM import / routing preprocessing / emissions / persons /
  mesoscopic model / TraCI / GUI. It consumes SUMO's *file formats* (`.net.xml`, `.rou.xml`,
  `.sumocfg`) and ports only the core.

---

## Install

The only prerequisite is the **.NET 8 SDK** — no SUMO, no other native dependencies. Verify with
`dotnet --version` (expect `8.x`).

**Linux** (Debian/Ubuntu — what the offline test loop itself uses):
```bash
sudo apt-get update && sudo apt-get install -y dotnet-sdk-8.0
# Fedora: sudo dnf install dotnet-sdk-8.0   ·   Arch: sudo pacman -S dotnet-sdk
# or the official installer: https://dotnet.microsoft.com/download/dotnet/8.0
```

**Windows** (PowerShell):
```powershell
winget install Microsoft.DotNet.SDK.8
# or the installer: https://dotnet.microsoft.com/download/dotnet/8.0
```

Then build and run the offline parity suite (no SUMO, no network):
```bash
git clone https://github.com/pjanec/SumoSharp && cd SumoSharp
dotnet build -c Release
dotnet test                     # 465 passed, 3 skipped  (offline; no SUMO, no network)
```

### See it run (from a fresh checkout)

```bash
# 1) Interactive gallery — 50+ self-contained replays, one per feature. Any machine, no GPU.
scripts/gen-demos.sh            # builds Sim.Viz/Sim.Run/Sim.ExtDemo, writes site/index.html
#   then open site/index.html in a browser  (or just visit https://pjanec.github.io/SumoSharp/)

# 2) The native desktop demo viewer — raylib + Dear ImGui, in-window scenario picker.
#    Needs a desktop/GPU; it builds on first run (pulls the raylib native package). It is out of
#    Traffic.sln, so run the project directly (this builds it):
dotnet run -c Release --project src/Sim.Viewer -- --mode local --demo "Roundabout"
#   switch demos live from the in-window "Demos" panel · drag = pan · wheel = zoom · click a road = drop an obstacle

# 3) The zero-install live browser viewer — streams a running engine over WebSocket:
dotnet run -c Release --project src/Sim.LiveHost -- scenarios/_bench/city-organic-L2   # open the printed http URL
```

More viewer modes (loopback / DDS publish+remote), controls, and a headless screenshot mode are in
[**Live & native viewers**](#live--native-viewers) below.

---

## Quick start

Two convenience scripts build the engine and run everything end-to-end — a `.sh` (Linux/macOS/Git Bash)
and a `.ps1` (Windows PowerShell) of each:

| What | Linux / macOS | Windows |
|---|---|---|
| Render `replay.html` for the showcase scenarios + external-agent demos | `scripts/run-examples.sh` | `powershell -File scripts/run-examples.ps1` |
| …for **every** parity scenario | `scripts/run-examples.sh all` | `… run-examples.ps1 all` |
| Determinism + scaled-city benchmarks (with engine-vs-SUMO aggregate parity) | `scripts/run-benchmarks.sh` | `powershell -File scripts/run-benchmarks.ps1` |
| Core-scaling curve (1→N threads vs SUMO) | — | `powershell -File scripts/bench-scaling.ps1` |

`run-examples` writes a self-contained `replay.html` into each scenario directory (open any in a
browser); `run-benchmarks` prints the determinism hash, throughput/RTF, peak RSS, stuck count, and the
engine-vs-SUMO aggregate PASS/FAIL across the city ladder.

Or drive the pieces directly:
```bash
# run the engine on one scenario → SUMO-schema FCD, then an interactive replay
dotnet run -c Release --project src/Sim.Run -- scenarios/11-priority-junction
dotnet run -c Release --project src/Sim.Viz -- scenarios/11-priority-junction
open scenarios/11-priority-junction/replay.html      # any web browser
```

---

## What is simulated

Parity bar is **exact @1e-3** unless noted. Car-following models are selected per-vehicle by the
`carFollowModel` vType attribute; everything feeds one multi-constraint speed reducer
(`nextSpeed = min over {lane leader, junction foes, red light, obstacle, …}`).

### Car-following models

| Model | Ports | Notes |
|---|---|---|
| **Krauss** (default) | `MSCFModel_Krauss` + `MSCFModel` | full safe-speed / brake-gap / accel-decel bounds |
| **IDM** | `MSCFModel_IDM` | δ=4, 4 sub-iterations |
| **IDMM** (IDM w/ memory) | `MSCFModel_IDM` (idmm arm) | per-vehicle level-of-service headway memory |
| **ACC** | `MSCFModel_ACC` | stateful speed/gap/CA control-mode machine |
| **CACC** | `MSCFModel_CACC` | cooperative time-gap law, ACC fallback |
| **Rail** | `MSCFModel_Rail` | parametric traction/resistance curves |

### Lane changing (`MSLCM_LC2013`, discrete lanes)

Decisions run in SUMO's post-move `changeLanes` phase order.

- ✅ **Strategic** (route/connectivity-forced, `getBestLanes`/`bestLaneOffset`, multi-hop route→lane)
- ✅ **Keep-right** (Rechtsfahrgebot accumulator)
- ✅ **Speed-gain / overtaking** (with target-lane safety veto)
- ✅ **Continuous lane change over time** (`lanechange.duration > 0` — label timing spread across steps)
- ✅ **Opposite-direction overtaking** (`lcOpposite` — spill into the oncoming `bidi` lane past a slow
  leader, closing-speed gap acceptance, cooperative oncoming shift) — see *Extras* below.
- ⏸ Not yet: cooperative LC, physical **lateral position** / sublane model (`MSLCM_SL2015`) — see
  *Not simulated*.

### Junctions & right-of-way

Driven by the static netconvert `<request>` response/foe matrix over a **frozen start-of-step foe
snapshot** (order-independent, no arrival-time race).

- ✅ **Priority** yielding (`MSRightOfWayJunction` / `MSLink` / `adaptToJunctionLeader`)
- ✅ **Right-before-left** and **all-way-stop** (`updateWaitingTime` + longest-waiter order)
- ✅ **Roundabouts** (single-lane priority; circulating priority + merge + successive-lane speed cap)
- ✅ **Merging / on-ramp / zipper** (`sameTarget` merge, conflict-width geometry)
- ✅ **keepClear / don't-block-the-box** + cross-junction leader following
- ✅ **willPass** ordering pre-pass for multi-lane saturation
- ⚠️ Box-blocking on *pathological* tight unmarked rings is diagnosed but not fixed (see roadmap).

### Traffic lights

- ✅ **Static** `<tlLogic>` programs (`MSSimpleTrafficLightLogic`) + yellow "stop-if-you-can-brake" decision
- ✅ **Actuated / detector-driven** (`MSActuatedTrafficLightLogic` + `MSInductLoop`, gap-based algorithm)

### Rail (all exact @1e-3)

- ✅ Rail vClass defaults + free-running train, two-train following (rail Krauss gap)
- ✅ **Bidirectional single track** (`<edge bidi=…>`) with deadlock insertion guard
- ✅ **Rail signals + driveway/block reservation** (`MSRailSignal` — hold until shared block clears)
- ✅ **Level crossings** (`MSRailCrossing` — close road links while a train occupies)
- ✅ **Traction model** (`MSCFModel_Rail` — speed-dependent accel from parametric curves)

### Integration, routing & determinism

- ✅ **Euler** (default) and **ballistic** integration (config flag)
- ✅ **actionStepLength** (reaction time > 1 step)
- ✅ **sigma / dawdle** (`MSCFModel_Krauss::dawdle2`) and **speedFactor distribution** (`normc(…)`),
  both validated against SUMO **ensembles** (statistical parity)
- ✅ **Route → lane resolution** through each `<connection>`'s `via` internal lanes, multi-hop,
  intra-edge mid-route lane change, multi-internal-lane junction "cont" chains
- ✅ **Rerouting** — Dijkstra over the connection-turn graph (`DijkstraRouter`/`MSDevice_Routing`),
  validated against `duarouter`
- ✅ **Per-entity seeded RNG** (SplitMix64, hashed from entity id) — thread-order-independent, never
  `System.Random`
- ✅ **Demand insertion** — `<flow>` (period / vehsPerHour / number) expanded deterministically at
  ingest (exact parity), and `<flow probability=>` inserted per step by a **per-flow seeded** RNG
  (deterministic & reproducible; statistical parity with SUMO — see *Not simulated → \[net\] tail*)
- ✅ **Warm-start snapshot** — `WarmUp(steps)` deterministically pre-populates a live, already-running
  network in memory; `SaveSnapshot` / `LoadSnapshot` round-trips **all** vehicles (incl. trains) plus
  the engine state machines, so a run can start from a live-traffic snapshot (in-memory or from file)

---

## What is *not* simulated

**Hard out of scope** — consumed as pre-processed input or simply not a goal:
`netconvert` / OSM parsing / route import · emissions / electric / fuel models · SUMO's own **person
model** (`MSPModel_Striping` / `<person>` walk-parity) · **containers** · public transport (`<busStop>`,
dwell, schedules) · mesoscopic model · **TraCI** runtime control · GUI internals.

> **Pedestrians are simulated — just not as a SUMO port.** SumoSharp does not reproduce SUMO's person
> trajectories; instead it has an independent, network-distributable **pedestrian crowd layer** (navmesh
> + O-D demand + LOD + liveliness + a deterministic weave), on the live-reactivity axis and validated by
> behavioral tests. See *Extras → Pedestrians* below and [`docs/PEDESTRIANS.md`](docs/PEDESTRIANS.md).

**Deferred within scope** (characterized, intentionally not built yet):
cooperative lane changes · the full **sublane / lateral model** (`MSLCM_SL2015`, continuous lateral
position, hexagonal footprints, spatial hash, SIMD — this is "phase 2") · U-turns · station dwell /
train reversal · advanced actuated-TLS features (switching rules, TraCI overrides, jam logic). See
`docs/TASKS.md` and `docs/DESIGN.md` for the full ledger and the phase-1/phase-2 seam analysis.

The probabilistic-`<flow>` arrival stream and the warm-start snapshot also carry **SUMO cross-checks**:
an ensemble statistical parity of the insertion-count distribution vs a 50-seed SUMO golden
(`scenarios/58-flow-probability`), and a `SaveSnapshot`-vs-SUMO-`--save-state` mid-run cross-check
(`golden.state.mid.xml`) — both green.

---

## Extras & differences from SUMO

### Architecture (the deliberate divergence)

The *algorithms* are copied faithfully; the *data structures* are rebuilt data-oriented. The only
behavioral deviations permitted are those ECS parallelism structurally forces — memory layout and the
*timing* of structural mutations — neither of which touches the numbers.

- **Plan/execute double buffer** — followers read start-of-step neighbor state; this is faithful to
  SUMO's `planMovements → executeMove` split, not an optimization.
- **Deferred command buffer** — lane swaps, reroute, insertion and arrival are recorded during
  planning and flushed at step end (`ICommandBuffer`: `ChangeLane`/`ReplaceRoute`/`Destroy`/`Flush`).
- **Zero-allocation hot path** — the emit path is a plain struct append (no per-point heap record or
  red-black-tree node), and insertion route-resolution + the per-step junction constraints are
  allocation-free (route-wide best-lanes in **one** backward pass instead of O(N²), `ReadOnlySpan` +
  by-value struct callbacks, pooled / `[ThreadStatic]` scratch). Total engine allocation at city scale
  fell **−69 % (city-mixed-1k: 3.78 → 1.16 GiB over a 700-step run)**, all byte-identical.
- **Parallel by default at scale & deterministic** — the plan, export *and* post-move phases read only
  frozen start-of-step state and write only their own vehicle's intent/frame (structural changes go
  through the deferred command buffer), so they are **race-free by construction** and
  **auto-parallelize above 256 concurrent vehicles** (tiny parity scenarios stay serial); single-threaded
  and parallel runs are byte-identical. The measured scaling curve and the specific optimizations that
  produced it live in one place — *Scale*, below.
- **FastDataPlane-shaped** — int-handle identity, value-type components, immutable network blueprints,
  phased systems and an `IWorld`/`ICommandBuffer` seam make the engine droppable into an external ECS
  later *without* adding a dependency now. "The gap is representation, not architecture."

### External (non-SUMO) agents — the "live reactivity" seam

SUMO has no concept of agents it doesn't control. This engine adds **external obstacles/agents**
(pedestrians, navmesh/RVO crowd agents, live detections) injected lane-relative via
`AddObstacle`/`AddMovingObstacle`/`UpdateObstacle`; SUMO cars *react* to them. Frozen once per step,
so order-independent. Validated by **behavioral/property tests** (no-overlap, resume-within-N, correct
dodge side), not goldens.

- **B1 — stop:** brake to a Krauss gap behind a blocking agent.
- **B5 — follow / veto / junction-yield:** treat a moving agent as a dynamic leader, a lane-change
  blocker, or a junction foe.
- **B6 — swerve / spill:** when braking alone can't stop, **swerve within the lane**, else **spill
  into a gap-safe adjacent lane**, else stop; a *lunging* agent is dodged **predictively** toward the
  side it is vacating.

### Emergency vehicles & give-way (`MSDevice_Bluelight`, adapted)

SUMO's bluelight *pushes* a lateral alignment onto neighbours — incompatible with plan/execute — so
it's re-expressed as a per-vehicle **pull**: each car detects an approaching blue-light EV from the
frozen snapshot and forms its own "clear the way" intent.

- **Ignore-red** (`jmDriveAfterRedTime`) and **ignore-foe at junctions**
  (`jmIgnoreJunctionFoeProb` / `jmIgnoreFoeProb` / `jmIgnoreFoeSpeed`) — EVs push through (parity-tested).
- **Give-way:** detect the EV → **multi-lane vacate** (change to an adjacent lane, suppressing own
  keep-right/strategic LC), or on a single lane **drift to the edge and let the EV pass by** (a leader
  whose footprint no longer overlaps imposes no car-following constraint), then recenter. Behavioral
  parity (SUMO's own give-way is stochastic sublane alignment).

### Opposite-direction overtaking (`lcOpposite`, adapted)

A fast car held up behind a slow leader on a two-way (`bidi`) road overtakes **through the oncoming
lane**, reusing the give-way lateral primitives. Behavioral (no golden), gated on `lcOpposite` so
every other scenario is byte-identical.

- **Detect → gap-accept → execute:** form the intent when held up with the oncoming lane clear, then
  a **closing-speed gap acceptance** commits only if the pass can finish before the head-on arrives
  (`requiredClear = (egoSpeed + oncomingSpeed)·overtakeTime + safety`); ego **spills** into the
  oncoming lane, passes the leader (the same *footprint-no-longer-overlaps* leader bypass as give-way),
  and **recenters** once past or if the gap acceptance drops — collision-free including a tested
  adversarial *abort-mid-spill*.
- **Cooperative oncoming shift:** an oncoming driver that sees a spilled overtaker closing head-on
  **pulls to its own outer edge** to widen the corridor, then recenters — the mirror of the give-way
  drift.
- Deferred (diagnosed in `docs/OV-REMAINING.md`): a cross-lane hard-brake backstop, return-gap enforcement,
  and a coupled OV2/OV4 decision enabling a true reduced-clearance side-by-side pass.

### Pedestrians (a from-scratch, network-distributable crowd layer)

An independent pedestrian subsystem — **not** a port of SUMO's `MSPModel` person model, and on the
**live-reactivity axis, not the parity axis** (validated by behavioral/property tests, never golden
FCD). Every pedestrian feature is **default-off / inert-when-absent**, so the vehicle parity goldens and
the determinism hash are byte-unchanged when peds are unused. Suite: **214 tests, green**.

- **Navmesh from the SUMO net** — sidewalks / crossings / walkingAreas bake into walkable polygons with
  per-vertex half-width (`WalkablePolygonBaker` → `SumoNavMesh`); A* routing; three additive connectivity
  passes (area-overlap, sidewalk continuation, and a stitch from the net's declared ped `<connection>`s)
  fold a real crop's split junctions into one connected component. Alternate DotRecast provider, cross-checked.
- **Poisson O-D demand** routed over the navmesh, seeded (SplitMix64, never `System.Random`), with a
  **density knob** (peds-per-walkable-km, LoS-C-capped) and **weighted POI + fringe endpoints** so peds
  appear/vanish only at legitimate edges.
- **Two-level LOD** — cheap **dead-reckoned "ambient"** peds (`PathArcMotion`, O(1)) vs. reactive
  **full-ORCA** peds, promoted/demoted on demand by a movable multi-source **interest field**.
- **Liveliness** — activity timelines (walk / pause / sit / step inside a building), pre-scheduled two-ped
  meet-and-talk, and templated actors (a looping waiter) — all still low-power.
- **Deterministic lateral weave** — dense low-power crowds are offset onto their own half of each
  sidewalk (a pure function of `route, seed, width, time`), so opposing/overtaking peds thread shared
  sidewalks **without passing through each other** — at O(1) per ped, no neighbour queries.
- **Reacts with traffic** — signalized crosswalk gates (queue → surge on the walk phase), cars stopping
  for a ped in their lane (`Engine.CrowdSource`), obstacle dodge + dynamic-blocker reroute, and parking-lot
  board/alight with mutual car↔ped avoidance.
- **`server == IG` by construction, over the wire.** Because a low-power ped's pose is a pure function of
  its inputs, the sim server and every remote **image-generator** reconstruct it **bit-for-bit** — a
  route/timeline is broadcast *once*, ambient peds emit **zero per-step bytes**, and a DR-error gate +
  global bandwidth governor keep even the reactive peds within a fixed budget. Proven bit-identical across
  an in-process byte loopback **and** real CycloneDDS (`Sim.Replication.Dds` / `Sim.PedDdsLoopback`, both
  out of `Traffic.sln`). The native raylib viewer draws the crowd reconstructed **from the wire** with
  ground-truth rings so the parity is literally visible.
- **Full feature index, code map, API, how-to-run, and what's parked: [`docs/PEDESTRIANS.md`](docs/PEDESTRIANS.md).**

### Panic evacuation (localized incident → foot exodus)

A complete **panic-evacuation** model layered *entirely* on top of the unchanged driving core (a
`Sim.Evac` external layer that only *drives* the engine through public seams — parity-exempt, so with
panic off the determinism hash is unmoved). On a localized security incident: nearby cars panic
(local information only — contagion + occlusion-gated line-of-sight + jam-unease, never a global
broadcast), switch to an aggressive flee preset and reroute toward exits; the streets jam; a boxed-in
panicked driver noses onto the shoulder as a shaped non-holonomic mover, then **abandons the car** and
its occupants flee **on foot**; the pedestrian crowd (reciprocal collision avoidance, confined to a
"known-world" navmesh) streams outward to safety, and cars react to both pedestrians and abandoning
cars as obstacles.

- **The evacuation is LOCAL** — the layer auto-attaches only to a bounded **working region** around the
  incident, so its cost scales with the *local* affected population, not the city size. Demonstrated at
  **10k-vehicle-class scale**: a central catastrophe with a low-thousands foot exodus while the rest of
  the city keeps flowing (distant traffic is pure, untouched parity lane traffic).
- **Bit-identical, opt-in scale knobs** — the two crowd solvers gained spatial hashes proven
  byte-identical to brute force (exact position/heading-equality tests); default off, per-scenario opt-in.
- Behavioral/property-tested (cascade emerges, containment, locality, determinism), not goldens.
- **Full feature index, per-phase design docs, and how to run the demos: `docs/PANIC-EVAC-OVERVIEW.md`.**

### Live & native viewers (dead-reckoned streaming)

Three visualizers share one engine; all are **out of the parity solution** (`Traffic.sln`), so the
hermetic `dotnet test` gate never touches them and the determinism hash is unaffected.

- **Offline HTML replay** (`Sim.Viz`) — reads a network + FCD trajectory and writes a single
  self-contained `replay.html` (Canvas 2D). No server, no SUMO. The everything-committed-scenario path.
- **Live browser viewer** (`Sim.LiveHost`) — a running engine streamed to a browser over WebSocket at a
  low update rate, with **client-side dead reckoning** (poses extrapolated/interpolated between packets)
  for smooth 60 fps playback. The zero-install shareable demo.
- **Native C# viewer** (`Sim.Viewer` + `Sim.Viewer.Core`) — a desktop window (**raylib-cs** for the
  world, **Dear ImGui** for the controls + a perf-diagnostics overlay) built for **10k-vehicle** scale.
  Four modes: `local` (renders the authoritative `Snapshot` every frame — no transport, no jitter),
  `loopback` (one process: publish → DDS → subscribe → dead-reckon → render), `publish` (headless
  engine + DDS writer), and `remote` (**view-only** DDS subscriber + renderer; a late joiner gets the
  road network via durable QoS). `local` mode also doubles as a **demo tool**: an in-window ImGui
  **scenario picker** (`--demo "<name>"`) switches between curated demos live — junctions, traffic
  lights, lane-changing, rail, city-scale, sandbox nets, **and** the **panic-evacuation** feature
  rendered *live* (incident zone, fear-tinted cars, pedestrian crowd, abandoned cars; click to place the
  incident).

The remote transport is **binary DDS** (RTPS), not JSON — a `Sim.Replication` layer (transport-agnostic
frame codec + an **adaptive publish-rate policy** that sends fast-changing vehicles more often and
predictable ones less) feeds `Sim.Replication.Dds` topics: high-rate vehicle state, a **durable,
chunked network-geometry** topic (so a remote client draws roads without the net file), and a
**low-rate traffic-light** topic. The subscriber's `DrClock` paces the render clock off each sample's
source timestamp with playout-delay interpolation and an extrapolation fallback, so a 2 Hz feed renders
as smooth motion. Design: `docs/SUMOSHARP-NATIVE-VIEWER.md`, `docs/SUMOSHARP-DEADRECKONING.md`.

Per render frame that reconstructed pose is then turned into **believable vehicle motion by a shared
kinematic reconstructor** (`Sim.Viewer.Motion.KinematicReconstructor`): a no-slip rear-axle (bicycle) model
so the body pivots at the rear like a real car, low-passed lane-heading prediction, a **spatial look-ahead**
that anticipates the connecting lane through a junction (the front rides the connecting-lane centerline
instead of turning in late), a bounded lane-change ease, and coarse-feed robustness for sparse (~1–3 Hz)
DDS packets. **Both** the native raylib viewer and the Godot City3D viewer use it (fix-once-fixes-both); it
supersedes the earlier capped-correction + heading-tilt smoother on the vehicle path. Design + the numeric
tuning story: `docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`, `docs/IGBRIDGE-METHODOLOGY.md`.

### Demand insertion & warm-start snapshots

- **Flow demand:** `<flow>` (period/vehsPerHour/number, exact parity) and probabilistic
  `<flow probability=>` (per-flow seeded RNG, deterministic & reproducible).
- **Warm-start:** `WarmUp(steps)` deterministically precomputes a populated, already-driving network;
  `SaveSnapshot`/`LoadSnapshot` persists the whole live state — every vehicle (cars **and** trains)
  plus the engine's rail-crossing/clock machines — so a simulation can begin from a live-traffic
  snapshot rather than an empty map.

### Rail

Rail was previously out of scope entirely; it's now a first-class citizen (trains, rail signals, level
crossings, bidirectional single-track, traction model) — see *What is simulated → Rail*.

### Determinism & the parity harness

- Phase-1 scenarios strip randomness (`sigma=0`, fixed depart, `actionStepLength=1`, teleport off,
  Euler) so parity is asserted **tightly**.
- Goldens are regenerated by pinned SUMO 1.20.0 (`scripts/regen-goldens.sh`) and **committed**:
  `golden.fcd.xml` (trajectory) + `golden.state.xml` (resolved state) + `golden.vtype.json` (resolved
  vType defaults) + `tolerance.json` (per-scenario `parityMode` + attribute tolerances) +
  `provenance.txt` (hashes + SUMO version, so staleness is detectable).
- **All external/live/rail/emergency features are inert-when-absent** (default flags off / empty
  obstacle store), so the deterministic parity scenarios and the `Sim.Bench` hash stay byte-identical.

### Scale

Struct-of-arrays + a zero-alloc hot path target large scenarios. The scaling ladder
(`scenarios/_bench/SCALING.md`) exercises the engine against SUMO up to **~15,000 peak concurrent**
vehicles.

**Hot-path speed vs single-threaded SUMO — byte-identical.** On the target box (16-core / 24-thread,
Windows 11), `city-3000` (7,632 vehicles, 1,200 steps, ~4,300 concurrent) runs its **simulation tick**
against single-threaded **SUMO 1.20.0's 17.19 s** on the identical net. The spatial region-parallel
path (`--region`) scales cleanly at low core counts and saturates the memory subsystem around 8 threads
(the plan/junction phases are bandwidth-bound on random neighbour access):

| Cores | `city-3000` sim tick | vs 1-thread SUMO (17.19 s) |
|---|---|---|
| 1 | 14.32 s | 1.24× |
| 2 |  8.20 s | 2.10× |
| 4 |  5.61 s | **3.07×** |
| 8 |  4.81 s | **3.57×** |

**4 cores already captures most of the win** (3.07×); 8 cores adds the last ~16 %, and beyond 8 threads
hyper-threading oversubscription regresses. Every point is **byte-identical** to the committed goldens
(single == parallel determinism hash `909605E965BFFE59`) — a faster *identical* answer, not an
approximation. The single-thread **1.24×** is the data-oriented engine being leaner than SUMO per tick;
the rest is the region-parallel scaling on top.

The **`sumosharp` drop-in serve CLI** (the `sumo`-compatible binary the SumoData pipeline calls via
`SUMO_BINARY`) exposes this same worker-thread cap as **`--max-parallelism N`** — `N<=0` (or omitted) =
all cores, `N>=1` = at most N. It is parsed order-independently (works before or after `-c`, so it can
ride in a `SUMO_BINARY="dotnet …/sumosharp.dll --max-parallelism 4"` prefix), letting a batch of
concurrent probe sims cap per-sim threads instead of each grabbing every core. It is a **performance
knob only**: output is byte-identical for any value (invariance is asserted by
`RungHDgap4MaxParallelismTests`). See `docs/SUMOSHARP-CLI-MAXPARALLELISM.md`.

**Optimizations that built the curve.** Every one is **byte-identical** (holds that same hash) — gated
so any special-feature scenario falls back to the exact serial path, and each keeps the committed
goldens unchanged. In roughly the order they landed:

- **O(N²) → indexed junction scans** — closed the early ~39× gap (~28 min → minutes). The per-step
  right-of-way and keepClear space-accounting used to scan every vehicle per lane; per-lane indices made
  them O(vehicles-on-that-lane). (The same investigation fixed a cont-turn **U-turn distance bug** that
  froze fringe vehicles and seeded a gridlock cascade — see the aggregate-parity note below.)
- **Plan-pass fusion** — the engine used to run *two* full plan passes over every near-junction vehicle
  (a `willPass` pre-pass, then the real plan); the pre-pass intent is now cached and reused inline,
  bringing the **true single-threaded** path (`--serial`) to ~parity with single-threaded SUMO.
- **Parallel export** — each vehicle's frame geometry is computed concurrently into a reusable
  index-keyed buffer; the comparator/hash are (vehicle, time)-keyed, so emission order is irrelevant.
- **Insert route pre-resolution** — a vehicle's lane sequence is a pure function of its route + the
  immutable network, so it is resolved for *every* vehicle **in parallel at load** instead of in the hot
  insertion loop (−72 % of the insert phase).
- **Handle-array emit** — the emit path indexes lanes by their dense integer handle instead of hashing
  the `LaneId` string per vehicle per frame (−28 % emit).
- **Spatial domain decomposition** (`--region`) — the network is partitioned into a grid of regions that
  each **own a disjoint set of lanes**, so one task per region needs no locks. **Dynamic scheduling over
  the regions auto-balances load** — as congestion concentrates, busy regions are simply picked up by
  free threads (finer grids give smaller working sets and better balance) — and vehicle handoff between
  regions is free: a vehicle that crosses a boundary is regrouped by its current lane the next step, with
  no explicit state transfer. This region-parallelizes the plan, junction `willPass`, movement execute,
  neighbour refill and post-move speed-gain phases.

The dominant plan/junction phases are **memory-bandwidth-bound on random neighbour access**, which caps
byte-identical parallel scaling near ~3× (hence the 4→8-core tail-off above). The full ledger — the
ceiling analysis, and the blind alleys that were tried and reverted (per-field SoA, a flat spatial
reorder, locked parallel foe-index) — is in **`docs/PERF-HANDOVER.md`** and **`docs/SPATIAL-OPT.md`**.

**Engine vs real SUMO on the same scenario.** `Sim.BenchCity` compares an engine run against a
committed SUMO 1.20.0 reference (`--sumo-summary`/`--sumo-tripinfo`/`--aggregate-tolerance`) on
arrivals, mean trip duration, mean speed, and the trip-time distribution (KS). On `city-300`
(724 vehicles, seed 42) the aggregates land **within 0.5 % of SUMO on every metric** (arrived
238→237 = 0.42 %, mean duration 424.65→425.15 s = 0.12 %, mean speed 13.27→13.22 m/s = 0.36 %,
KS distance 0.027 — all well inside the 35 % statistical band):

```bash
dotnet run -c Release --project src/Sim.BenchCity -- scenarios/_bench/city-300 \
  --sumo-summary   scenarios/_bench/city-300/summary.xml \
  --sumo-tripinfo  scenarios/_bench/city-300/tripinfo.xml \
  --aggregate-tolerance scenarios/_bench/city-300/aggregate-tolerance.json
# … AGGREGATE COMPARISON: PASS
```

The same command runs against `city-30` / `city-3000` / `city-15000` (each ships the SUMO reference
trio), and the engine now tracks SUMO across the ladder with **0 stuck / 0 teleport**: `city-3000`
(7,632 vehicles, ~4,300 concurrent) arrives **3,446 vs SUMO's 3,260** (relError 5.7 %), mean duration
within 1.3 %, mean speed within 3.1 % — a full aggregate PASS with margin. The former extreme-saturation
gridlock (~2,200 stuck) was root-caused by an engine-vs-SUMO FCD trace to a **cont-turn (U-turn)
distance bug** in the junction merge arm — a U-turn split across two internal lanes measured its
distance-to-merge from a 1 m intermediate lane, freezing the vehicle hundreds of metres early; since
randomTrips fills the fringe with U-turns (~47 % of that demand), the frozen vehicles seeded a whole
gridlock cascade. The one-line distance fix (byte-identical on every committed golden) cleared it; see
`scenarios/_diag/uturn-contturn-freeze` (`UturnContTurnFreezeDiagTests`).

---

## Visual demos & how to run them

The visualizer is **offline and browser-only** — no SUMO, no server needed for committed scenarios.
`Sim.Viz` reads a scenario's network + an FCD trajectory and writes a **single self-contained
`replay.html`** (vanilla Canvas 2D: width-accurate lanes, junction fills, SUMO-native signal heads,
true-size oriented vehicle boxes colored by vClass; play/pause/scrub/speed, zoom & pan).

### Live demos

A curated, auto-generated gallery of these replays — **one per feature, 50+ in all** — is deployed to
GitHub Pages: **https://pjanec.github.io/SumoSharp/**. CI regenerates it on every change to the
scenarios or the engine (`src/**`). The full categorized list is in [`docs/DEMOS.md`](docs/DEMOS.md);
generate the same gallery locally with `scripts/gen-demos.sh`, then open `site/index.html`.

> **Phone caveat:** opening a local `replay.html` from the iOS Files app runs it with **no JavaScript**
> (black canvas). On a phone, serve it over **https** (e.g. a hosted artifact URL), not `file://`.

### Any parity scenario (has a committed golden — nothing to generate)

```bash
dotnet run --project src/Sim.Viz -- scenarios/09-traffic-light
# then open scenarios/09-traffic-light/replay.html
```

Worth a look: `11-priority-junction`, `12-overtake`, `26-right-before-left`, `27-allway-stop`,
`32-roundabout`, `35-actuated-tls`, `43-continuous-lanechange`, `44-multilane-junction-turn`,
`16-emergency-red` (EV runs red), `53-giveway-single` / `55-giveway-drift` (cars pull aside for an EV),
`47-rail-free-flow` / `49-rail-bidi-meet` / `51-rail-crossing` (trains, bidi track, level crossing).
*(A few scenarios ship a pre-rendered `replay.html`: `01`, `09`, `11`, and the `city-30`/`city-300`/
`city-organic` benchmarks.)*

### External-agent demos (inject non-SUMO agents, then visualize both together)

`Sim.ExtDemo` runs the engine with external agents from a JSON script and writes a **combined** FCD
(SUMO cars + agents). Feed that to `Sim.Viz` with `--fcd`.

```bash
# all five reactions (stop / swerve / spill / follow / junction-yield) in one replay:
dotnet run --project src/Sim.ExtDemo -- scenarios/_bench/ext-showcase
dotnet run --project src/Sim.Viz    -- scenarios/_bench/ext-showcase --fcd scenarios/_bench/ext-showcase/engine.fcd.xml
```

| Demo | Shows |
|---|---|
| `scenarios/_bench/ext-showcase` | all five external-agent reactions sequentially |
| `scenarios/_bench/ext-swerve-demo` | pure B6 lateral swerve around a lunging pedestrian (watch at 0.25×) |
| `scenarios/_bench/ext-agents-demo` | simplest with/without proof: stop-behind + follow-slow-agent |

### Scaled-city demos

```bash
# run the engine to completion (Release), then render (city FCD can be large — downsample if needed)
dotnet run -c Release --project src/Sim.Run -- scenarios/_bench/city-organic-L2
dotnet run           --project src/Sim.Viz -- scenarios/_bench/city-organic-L2 --fcd scenarios/_bench/city-organic-L2/engine.fcd.xml
```

| Demo | What it stresses |
|---|---|
| `scenarios/_bench/city-organic` | organic town, 49 junctions incl. a roundabout, ~180 concurrent (pre-rendered) |
| `scenarios/_bench/city-organic-L2` | **2-lane** town, 274 junctions, ~406 concurrent — multi-lane at scale |
| `scenarios/_bench/city-mixed-1k` | 1,064 junctions (262 TLS), ~1,000 concurrent, ~98% flowing |

### Pedestrian demos

Self-contained HTML replays of the pedestrian crowd layer (see *Pedestrians* above and
[`docs/PEDESTRIANS.md`](docs/PEDESTRIANS.md)). No SUMO, no server, no GPU.

```bash
# dense demo-city block: hundreds of weaving pedestrians + a car flow on the real signalized grid
dotnet run -c Release --project src/Sim.Viz -- --ped-dense-city ped-dense-city.html
# one intersection: routed weaving crowd + cross-traffic cars
dotnet run -c Release --project src/Sim.Viz -- --ped-weave-city ped-weave-city.html
# the crowd rendered purely from the replication wire (server == IG), promotion pop-free
dotnet run -c Release --project src/Sim.Viz -- --ped-remote ped-remote.html
# more: --ped-od-routing --ped-lively-crowd --ped-crossing-gate --ped-lod-promotion
#       --ped-dodge --ped-reroute --ped-parking --ped-liveliness --ped-social --ped-waiter
```

The 14 pedestrian gallery entries are registered under the **Pedestrians** category in
`scripts/gen-demos.sh`; the offline pedestrian test suite is `dotnet test tests/Sim.Pedestrians.Tests`.

### Panic-evacuation demos

Self-contained HTML replays of the evacuation model (see *Panic evacuation* above and
`docs/PANIC-EVAC-OVERVIEW.md`). No SUMO, no server.

```bash
# realistic organic town — congestion + a large local foot exodus
dotnet run -c Release --project src/Sim.Viz -- --evac-organic evac-organic.html
# 10k-vehicle-class city — a local catastrophe while the rest of the city keeps flowing
dotnet run -c Release --project src/Sim.Viz -- --evac-city evac-city.html
# per-phase cost profile (organic; --city for 10k off-vs-on; --microbench for the crowd-hash speedup)
dotnet run -c Release --project src/Sim.EvacProfile
```

### Live & native viewers

The **live browser viewer** streams a running engine over WebSocket with client-side dead reckoning:
```bash
dotnet run -c Release --project src/Sim.LiveHost -- scenarios/_bench/city-organic-L2   # then open the printed http URL
```

The **native desktop viewer** (raylib + Dear ImGui, 10k-scale; both it and `Sim.Viewer.Core` are out of
`Traffic.sln`, so build/run them directly):
```bash
# local: owns the engine, renders the authoritative snapshot (no transport, no jitter)
dotnet run -c Release --project src/Sim.Viewer -- --mode local samples/junctions/cross/net.net.xml
# loopback: one process — publish → DDS → subscribe → dead-reckon → render (delay 0 = extrapolate, >0 = interpolate)
dotnet run -c Release --project src/Sim.Viewer -- --mode loopback scenarios/15-reroute --delay 0.4
# two processes: a headless publisher, then a late-joining view-only remote client
dotnet run -c Release --project src/Sim.Viewer -- --mode publish samples/junctions/cross/net.net.xml   # terminal A
dotnet run -c Release --project src/Sim.Viewer -- --mode remote                                        # terminal B
# demo tool: pick an initial demo by name; switch live from the in-window ImGui "Demos" picker
dotnet run -c Release --project src/Sim.Viewer -- --mode local --demo "Roundabout"
dotnet run -c Release --project src/Sim.Viewer -- --mode local --demo "Evacuation (organic town)"      # live panic-evac
```
Controls (local/loopback): drag = pan, wheel = zoom, click a road = drop an obstacle, `d` = toggle the
diagnostics overlay; the ImGui panel has restart / clear-obstacles / inject-traffic / DR-delay. On a
headless box, render to a PNG with `--screenshot out.png --frames 120` (software GL under `xvfb-run`).

### Benchmarks (metrics, not pictures)

```bash
dotnet run -c Release --project src/Sim.Bench            # highway-dense: steps/s, alloc/veh-step, determinism hash
dotnet run -c Release --project src/Sim.BenchCity -- scenarios/_bench/city-organic-L2   # RTF, peak RSS, stuck count
```

---

## Repository layout

```
src/
  Sim.Core/       the engine: car-following models, lane change, junction RoW, TLS, rail,
                  external-obstacle & give-way reactions, ECS command buffer / phases
  Sim.Ingest/     .net.xml / .rou.xml / .sumocfg parsing, immutable network + demand model, router
  Sim.Harness/    FCD parser/writer, trajectory & ensemble comparators, tolerance config
  Sim.ParityTests/ the offline test suite (engine vs committed goldens)
  Sim.Run/        engine → FCD dumper (feeds the visualizer)
  Sim.Viz/        offline HTML replay generator (Canvas 2D; incl. the evac scenes)
  Sim.LiveHost/   live dead-reckoned browser viewer (WebSocket; the zero-install demo)
  Sim.Replication/     transport-agnostic frame codec + adaptive publish policy + DR model
  Sim.Replication.Dds/ CycloneDDS topic types (vehicle state / geometry / traffic-light / pedestrian) — out of Traffic.sln
  Sim.Viewer.Core/ native-viewer engine host + DDS pub/sub + DrClock (headless-testable)
  Sim.Viewer/     native 10k-scale desktop viewer (raylib-cs + Dear ImGui) — out of Traffic.sln
  Sim.ExtDemo/    external-agent demo runner (combined FCD)
  Sim.Evac/       panic-evacuation layer (parity-exempt; drives the core via public seams)
  Sim.EvacProfile/ evac per-phase cost profiler + crowd-solver micro-benchmark
  Sim.Pedestrians/ the from-scratch pedestrian crowd layer — navmesh bake, O-D demand, two-level LOD,
                  activity-timeline liveliness, deterministic weave, and the server==IG replication seam
  Sim.Pedestrians.Nav.DotRecast/ alternate DotRecast navmesh provider (cross-checked against SumoNavMesh)
  Sim.PedDdsLoopback/ live CycloneDDS server==IG proof for pedestrians — out of Traffic.sln
  Sim.BenchPedLod/ · Sim.BenchPedNet/ · Sim.BenchCrowd/  pedestrian micro-benchmarks
  Sim.Bench/      determinism + micro-benchmark oracle
  Sim.BenchCity/  scaled-city benchmark runner (RTF / RSS / stuck detector)
scenarios/        committed parity scenarios (inputs + goldens + tolerance + provenance), _bench/ demos,
                  and _ped/ pedestrian scenarios (poc0-crossing-plaza, demo_city, sub-area/evac nets)
sumo/             vendored SUMO 1.20.0 source (read-only algorithm reference; never edited)
scripts/          golden regeneration & benchmark generation (network side; needs SUMO)
docs/             design, specs, the perf/optimization ledger, and open-issue notes
LICENSE           EPL-2.0 OR GPL-2.0-or-later (dual); version.json — project version (SemVer)
```

Key docs: **`docs/DESIGN.md`** (architecture of record — read this for the "why"), **`docs/TASKS.md`** (the work
queue / feature ledger), **`CLAUDE.md`** (contributor rules), **`docs/RAIL-SUPPORT.md`**,
**`docs/EXTERNAL-AGENTS-VIZ.md`**, **`docs/BENCHMARK_SPEC.md`**, **`docs/C4-VII-REMAINING.md`** (open junction work),
**`docs/PANIC-EVAC-OVERVIEW.md`** (the panic-evacuation feature index), **`docs/PEDESTRIANS.md`** (the
pedestrian-subsystem front door — capabilities, code map, API, what's parked), **`docs/SUMOSHARP-NATIVE-VIEWER.md`**
(native raylib + DDS viewer) and **`docs/SUMOSHARP-DEADRECKONING.md`** (the dead-reckoning / replication stack).

---

## Status

The car-following, lane-change, junction/right-of-way, traffic-light, rail, emergency-vehicle and
external-agent subsystems are implemented and parity-tested (**649 passing parity checks**, +3 skipped).
The **pedestrian crowd layer** (navmesh, O-D demand, two-level LOD, liveliness, deterministic weave,
server==IG replication) is on the live-reactivity axis and behavior/property-tested (**214 passing**);
because it is default-off, the vehicle parity goldens and determinism hash are unaffected. Known open
item: box-blocking on *pathological* tight unmarked single-lane rings under saturation (diagnosed,
deferred — normal roundabouts flow fine). Phase 2 (the laneless/sublane heterogeneous model) is
designed-for but not built; the pedestrian navmesh connectivity thresholds are provisional pending
real-net (non-synthetic) re-validation.

---

## Versioning

[Semantic Versioning 2.0.0](https://semver.org). The current version lives in **`version.json`** (and
is stamped into every built assembly via `Directory.Build.props`). Pre-1.0, so behavior and public API
may still change between minor versions. The SUMO version this engine targets for parity is tracked
separately in **`SUMO_VERSION`** (currently `1.20.0`).

## License

**`EPL-2.0 OR GPL-2.0-or-later`** (dual) — you may use this project under *either* license, at your
option; see **`LICENSE`**. This project ports algorithms from, and vendors the source of,
[Eclipse SUMO](https://eclipse.dev/sumo/), which is itself licensed `EPL-2.0 OR GPL-2.0-or-later`; the
port is a derivative work, so it carries the same dual license. **Commercial use is fine** — under the
EPL-2.0 arm (weak, file-level copyleft) you can build proprietary software around this engine and need
only share modifications to the EPL-covered files themselves.

The vendored **`sumo/`** directory is Eclipse SUMO's own upstream source, used read-only as an algorithm
reference; it keeps SUMO's own copyright and license (`sumo/LICENSE`, `sumo/NOTICE.md`) and is not
modified.
