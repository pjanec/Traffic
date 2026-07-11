# Traffic — SUMO's microscopic traffic simulation, reimplemented in C# / .NET 8

A behaviorally-faithful, data-oriented reimplementation of [Eclipse SUMO](https://eclipse.dev/sumo/)
**1.20.0**'s microscopic mobility core in C# / .NET 8. The goal is **exact trajectory parity with
SUMO** on a cache-friendly, allocation-light, parallel-ready ECS engine — plus a few capabilities SUMO
doesn't have (live external agents, emergency-vehicle give-way, an offline HTML visualizer).

> **Correctness bar:** every deterministic scenario matches a committed SUMO golden trajectory to
> **1e-3** on `[lane, pos, speed]`, every step. Stochastic features use a statistical/ensemble bar
> instead. The committed goldens *are* the executable spec — `dotnet test` never needs SUMO.

- **What it ports:** the per-step simulation algorithms (car-following, lane-changing, junction
  right-of-way, traffic lights, rail), copied close to line-for-line from the vendored SUMO source.
- **What it deliberately rebuilds:** the *data structures*. SUMO's deep `MSVehicle` inheritance and
  pointer-linked leaders become struct-of-arrays with a plan/execute double buffer and a deferred
  command buffer — the point of the project, not a risky deviation.
- **What it does *not* do:** `netconvert` / OSM import / routing preprocessing / emissions / persons /
  mesoscopic model / TraCI / GUI. It consumes SUMO's *file formats* (`.net.xml`, `.rou.xml`,
  `.sumocfg`) and ports only the core.

---

## Quick start

Requires only the **.NET 8 SDK**. SUMO itself is **not** needed to build, test, run, or visualize —
only to regenerate goldens (rare, human-triggered).

```bash
# build + run the full parity suite (offline, no SUMO, no network)
dotnet build
dotnet test          # 202 passed, 1 skipped

# run the engine on a scenario and dump a SUMO-schema FCD trajectory
dotnet run --project src/Sim.Run -- scenarios/11-priority-junction

# render an interactive replay (writes a self-contained replay.html into the scenario dir)
dotnet run --project src/Sim.Viz -- scenarios/11-priority-junction
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
`netconvert` / OSM parsing / route import · emissions / electric / fuel models · **persons /
pedestrians / containers** (peds appear only as junction foes) · public transport (`<busStop>`, dwell,
schedules) · mesoscopic model · **TraCI** runtime control · GUI internals.

**Deferred within scope** (characterized, intentionally not built yet):
cooperative lane changes · the full **sublane / lateral model** (`MSLCM_SL2015`, continuous lateral
position, hexagonal footprints, spatial hash, SIMD — this is "phase 2") · U-turns · station dwell /
train reversal · advanced actuated-TLS features (switching rules, TraCI overrides, jam logic). See
`TASKS.md` and `DESIGN.md` for the full ledger and the phase-1/phase-2 seam analysis.

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
- **Zero-allocation hot path** — **735.8 → 207.1 B/veh-step (−71.9%)**; the remainder is the FCD emit.
- **Parallel-ready & deterministic** — the plan phase reads only frozen state and writes only its own
  intent, so it is race-free by construction; single-threaded and parallel runs produce a
  byte-identical determinism hash (`909605E965BFFE59`).
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
- Deferred (diagnosed in `OV-REMAINING.md`): a cross-lane hard-brake backstop, return-gap enforcement,
  and a coupled OV2/OV4 decision enabling a true reduced-clearance side-by-side pass.

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

Struct-of-arrays + zero-alloc hot path targets large scenarios. The scaling ladder
(`scenarios/_bench/SCALING.md`) validates statistical agreement with SUMO up to **~15,000 peak
concurrent** vehicles; `city-3000` (≈4,300 concurrent) passes the distribution (KS) and stability
(0 stuck / 0 teleport) checks. Perf figures there are provisional (measured with FCD writing on under
CPU contention); the correctness/stability columns are deterministic.

---

## Visual demos & how to run them

The visualizer is **offline and browser-only** — no SUMO, no server needed for committed scenarios.
`Sim.Viz` reads a scenario's network + an FCD trajectory and writes a **single self-contained
`replay.html`** (vanilla Canvas 2D: width-accurate lanes, junction fills, SUMO-native signal heads,
true-size oriented vehicle boxes colored by vClass; play/pause/scrub/speed, zoom & pan).

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
  Sim.Viz/        offline HTML replay generator (Canvas 2D)
  Sim.ExtDemo/    external-agent demo runner (combined FCD)
  Sim.Bench/      determinism + micro-benchmark oracle
  Sim.BenchCity/  scaled-city benchmark runner (RTF / RSS / stuck detector)
scenarios/        committed parity scenarios (inputs + goldens + tolerance + provenance) and _bench/ demos
sumo/             vendored SUMO 1.20.0 source (read-only algorithm reference; never edited)
scripts/          golden regeneration & benchmark generation (network side; needs SUMO)
```

Key docs: **`DESIGN.md`** (architecture of record — read this for the "why"), **`TASKS.md`** (the work
queue / feature ledger), **`CLAUDE.md`** (contributor rules), **`RAIL-SUPPORT.md`**,
**`EXTERNAL-AGENTS-VIZ.md`**, **`BENCHMARK_SPEC.md`**, **`C4-VII-REMAINING.md`** (open junction work).

---

## Status

The car-following, lane-change, junction/right-of-way, traffic-light, rail, emergency-vehicle and
external-agent subsystems are implemented and parity-tested (**202 passing scenarios/checks**). Known
open item: box-blocking on *pathological* tight unmarked single-lane rings under saturation (diagnosed,
deferred — normal roundabouts flow fine). Phase 2 (the laneless/sublane heterogeneous model) is
designed-for but not built.

*This project ports algorithms from Eclipse SUMO (EPL-2.0). The vendored `sumo/` tree is the upstream
source, used as a read-only reference.*
