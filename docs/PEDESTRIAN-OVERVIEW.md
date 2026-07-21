# PEDESTRIAN-OVERVIEW.md — the WHAT of the pedestrian subsystem

**Status: LARGELY IMPLEMENTED & tested — this doc is the original requirements/scope (the WHAT), kept
as the rationale of record.** The subsystem described here is built and behavior/property-tested (214
green); for the *current* state — what exists, where the code is, how to run it, what is parked — read
**[`PEDESTRIANS.md`](PEDESTRIANS.md)** (the front door) and **`PEDESTRIAN-TRACKER.md`** (the done/parked
map). This doc still holds the WHAT; `PEDESTRIAN-DESIGN.md` holds the HOW (mechanisms, architecture, the
determinism & networking argument); `PEDESTRIAN-POC-PLAN.md` holds the de-risking experiments.

This subsystem is on the **live-reactivity axis**, not the SUMO-parity axis (see `DESIGN.md` §"Two
futures"). It is validated by **behavioral / property tests**, never by golden FCD. The parity lane
core is untouched and stays byte-exact (determinism hash `909605E965BFFE59`).

---

## 1. The goal

Simulate pedestrians that are **performant at scale**, **believable in dense areas**, **interactive
with traffic and external entities**, **evacuation-capable**, and **distributable to many image-
generator (IG) render channels over a network** — on a single 16+‑core Windows server.

Concretely, the target is **~10 000 fully-interactive ("high-power") pedestrians + ~100 000 total**
(the balance being cheap "low-power" ambient pedestrians), alongside ~10 000 vehicles, streamed to
many IG channels within a bounded network budget.

## 2. Requirements

1. **Performance — large pedestrian counts.** Scale to the 10k-high / 100k-total target on the 16+‑core
   box. Ambient pedestrians must be cheap; the interactive ones must still be affordable in bulk.
2. **Believability — crowds, not lanes.** In dense areas pedestrians must move as a *crowd* (continuous,
   laneless, emergent flows), not shuffle down discrete stripes. Emergent bidirectional lane-formation
   is desirable realism, not a lane constraint.
3. **Interactivity.** Cars stop when a pedestrian is crossing on a crosswalk. Pedestrians dodge/avoid
   blockers — including **externally-controlled entities** (another externally-driven pedestrian walking
   among the simulated crowd, or an externally-controlled car parked partly on the sidewalk) — and route
   around them when a way is blocked.
4. **Evacuation compatibility.** On an incident, pedestrians panic and flee to safety. This must reuse
   the general pedestrian machinery, not fork it — evac is a *specialization* (destination = safe zone +
   panic parameter overrides), building on the existing `Sim.Evac` work.
5. **Disjoint crowds.** Pedestrians accumulate at crosswalk edges and cross on the walk signal as a
   larger group; they accumulate before a narrow shopping-mall entrance and stream out of a mall exit.
6. **Parking-lot situations (cars + pedestrians cooperating).** Pedestrians move toward their car across
   a parking lot, often *between* parked cars, forming moving obstacles on the inner drive lanes for cars
   trying to park or leave. This requires:
   - **Cars switchable between two modes** — *SUMO lane travel* and *parking-lot maneuvering* — switching
     at parking-lot **entry/exit points**.
   - **Pedestrians boarding/alighting** — a pedestrian moves up to a car and disappears (enters it), or
     appears next to a car (leaves it).
7. **Network distribution to many IG channels.** One server drives the simulation; many IG channels
   render it over the network. Bandwidth is the hard limit. Bundling (as already done for cars) is
   required; ideally the server sends **precomputed paths** that IGs follow locally via dead-reckoning,
   switching to per-frame position streaming only when a non-deterministic event forces it.

## 3. Scope and non-goals

- **We do NOT port SUMO's striping pedestrian model** (`MSPModel_Striping`). It walks pedestrians on
  discrete stripes/sublanes of footpaths — lane-like, exactly what Requirement 2/5/6 argue against.
  SUMO's only real crowd model is **JuPedSim**, an external C++ framework it merely proxies — not a port
  target either.
- **We DO consume SUMO's pedestrian network *topology*.** `netconvert` already emits sidewalks (lanes
  with `allow="pedestrian"`), pedestrian **crossings**, and **walkingAreas** in `net.xml`, which we
  already ingest. We copy SUMO's *map*, not its *movement*: motion runs on our continuous ORCA layer.
- **No golden-FCD parity for pedestrians.** Validation is behavioral/statistical (no-overlap, arrives-
  within-N, never-leaves-walkable-area, flux/throughput distributions), consistent with how the existing
  `OrcaCrowd` / `MixedTrafficCrowd` layers are already validated. Parity is reserved for sub-behaviors
  that *do* have a SUMO analog (e.g. a car braking for a stopped blocker).
- **Parking-lot maneuvering is a new free-space feature**, not a SUMO-parity feature (SUMO does not
  simulate in-lot maneuvering; `DemandModel` does not model `parkingArea`).
- Core stays **parity-inert**: the pedestrian subsystem is an additive layer that drives Core through its
  public seams, exactly as `Sim.Evac` does. With no pedestrians present, every scenario is byte-identical.

## 4. Key decisions (rationale in `PEDESTRIAN-DESIGN.md`)

| # | Decision | Why |
|---|---|---|
| D1 | **Build on the existing ORCA continuous-crowd layer, not SUMO striping.** | Requirements 2/5/6 need laneless crowd dynamics; `OrcaCrowd` already provides them, deterministically and cache-efficiently. |
| D2 | **Two orthogonal LOD axes: sim-LOD (compute) and net-LOD (bandwidth).** | Sim-LOD (low/high power) decides ORCA-vs-path compute globally; net-LOD (per-channel AoI) decides who streams. A ped can be high-power but off-camera, or low-power but visible. |
| D3 | **A unified regime/lifecycle state machine** for all entities (car lane/park/parked, person walk/ride, ped low/high power, DR-model). | Requirements 4 and 6 are all regime transitions; one command-buffer + lifecycle-event mechanism serves all of them. |
| D4 | **Navigation behind interfaces; DotRecast as the dev navmesh.** | The owner has a production navmesh to plug in later. `IPedNavigation`/steering/walkable-space are interfaces; DotRecast (MIT C# Recast/Detour port) is the dev provider, alongside a provider that bakes walkable polygons from SUMO ped-network geometry. |
| D5 | **Rule-based crosswalk gates** (read TLS state; hold at portal, release on walk phase), with ORCA avoidance as backstop. | Real crossings are rule-governed; gates also produce the "accumulate then surge" crowd behavior of Requirement 5 for free. |
| D6 | **`PathArc` dead-reckoning model for low-power peds** (send path once, IG follows locally), `FreeKinematic` for high-power. | The pedestrian analog of the car's lane-window path-DR. Makes the ~90k ambient peds nearly free on the wire; only the interactive set streams positions. |
| D7 | **Pedestrians live in a new `Sim.Pedestrians` package** layered on Core (like `Sim.Evac`). | Keeps Core parity-inert; evac becomes a consumer/specialization of the pedestrian layer. |

## 5. What we already own (the substrate — ~70% of the engine)

- **Operational crowd solver — `OrcaCrowd`** (`src/Sim.Core/Orca/`): a faithful RVO2/ORCA port —
  continuous 2D holonomic disc agents, struct-of-arrays, plan/execute double buffer (deterministic,
  parallel-ready), static walls (`AddObstacle`), external moving discs (`SetExternalObstacles`), opt-in
  uniform spatial hash (bit-identical, ~6× at n≥400), nearest-k, removal-on-arrival, deterministic
  symmetry-break. Value-type contiguous SoA — it does **not** suffer the lane core's memory-bandwidth
  wall.
- **Shaped free-space vehicle solver — `MixedTrafficCrowd`** (`src/Sim.Core/Mixed/`): oriented-polygon
  agents, non-holonomic (kinematic-bicycle) steering, soft asymmetric priority, wall confinement/box
  obstacles. This is the **parking-lot maneuvering** solver.
- **Car ↔ crowd coupling — the bridge** (`src/Sim.Core/Bridge/`): `WorldDisc` + `ICrowdFootprintSource`
  neutral seam; `Engine.CrowdSource`; `CrossRegimeCoupling`. Cars brake/swerve/spill/yield for
  pedestrians (`ComputeLateralEvasion`, `CrowdLongitudinalConstraint`); pedestrians avoid vehicles. All
  parity-safe (byte-identical when unused).
- **External-entity injection — `ExternalObstacle` / `ObstacleStore`** (`src/Sim.Core/`): lane-relative
  externally-controlled entities (dead-reckoned, dodgeable footprints) with the full car-reaction suite
  already wired. Covers the "external car parked across a sidewalk" and "walking blocker" cases.
- **Evac layer — `Sim.Evac`**: pedestrians already run as `OrcaCrowd` agents that mutually avoid live and
  abandoned cars over a `FakeNavMesh`, with `FearField`/panic, `LineOfSight`, `BlockedDetector`, and the
  lane→push→pedestrian cascade (`EnterOrcaPush`). The template for the regime state machine.
- **Networking / dead-reckoning — `Sim.Replication` (+ `.Dds`)**: `DrModel` enum
  (`LaneArc|FreeKinematic|Stationary`), `FrameCodec` packed blob, `FrameChunker` bundling,
  `PublishScheduler`+`DrErrorPublishPolicy` (publish-on-predicted-error), `PoseResolver`,
  `DdsWireFrame`/lifecycle topics, and a client DR pipeline (`DrClock`/`DrPoseSmoother`). A `CrowdRecord`
  (32 B) already exists in the codec.
- **Engine lifecycle API**: `SpawnVehicle(..., departPos, departSpeed, departLane)`, `Despawn`,
  `SetDestination`, slot recycling — the mechanics for regime transitions (parking entry/exit, board/
  alight).
- **Perf substrate**: `Parallel.For` scheduling, region decomposition (`ComputeLaneRegions`), thread-safe
  `CommandBuffer`, per-entity seeded RNG / hashed determinism.

## 6. The gaps (what this project builds)

1. **Strategic + tactical navigation** (the biggest gap): a real walkable-space representation, O→D
   routing, and portal/funnel steering. Today it is a bounding box + radial vector (`FakeNavMesh`).
2. **Sim-LOD low/high-power split** and the **promotion/demotion** machinery (with hysteresis).
3. **Crosswalk gates** tied to TLS state; **accumulate-then-surge** portal behavior.
4. **Parking lots**: the car lane↔free-space mode switch at entry/exit; ped board/alight; the full
   assembly of `MixedTrafficCrowd` cars + `OrcaCrowd` peds + static parked-car boxes — never assembled.
5. **`OrcaCrowd` box obstacles + a static-obstacle spatial index** (obstacle queries are O(n) today —
   won't scale to many buildings/parked cars).
6. **Parallelize `OrcaCrowd.Step`** (serial today; embarrassingly parallel by construction).
7. **Networking**: the `PathArc` DR model; end-to-end **crowd transport wiring** (`CrowdRecord` is codec-
   only); **quantization** (32 B → ~16 B); **net-LOD / per-channel AoI culling**; lifecycle events for
   regime transitions; and the **missing scale measurements** (nothing above ~450 concurrent movers is
   measured in-repo).
8. **Evac generalization**: replace radial-flee/exit-reroute with real destination routing so evac
   becomes a pedestrian-layer specialization.

## 7. Document map

- `PEDESTRIAN-OVERVIEW.md` — this file (WHAT).
- `PEDESTRIAN-DESIGN.md` — HOW: layered architecture, regime/LOD state machine, navigation interfaces,
  interactivity mechanisms, networking/DR, determinism, performance, packaging.
- `PEDESTRIAN-POC-PLAN.md` — the de-risking experiment ladder with explicit success conditions.
- Later, once the design converges: `PEDESTRIAN-TASKS.md` (staged tasks + success conditions) and
  `PEDESTRIAN-TRACKER.md` (checklist).
