# PEDESTRIAN-TASKS.md — production task breakdown

**Status: planning, for review.** The POC phase (0–7) validated every mechanism of the pedestrian design
(see `PEDESTRIAN-OVERVIEW.md`, `PEDESTRIAN-DESIGN.md`, and the `PEDESTRIAN-POC*-FINDINGS.md` files). This
document converts the validated design into an **executable production work queue**. `PEDESTRIAN-TRACKER.md`
is the at-a-glance checklist over these task IDs.

Each task names its **design reference** (a section, not a copy), the **files** it touches, its
**dependencies**, and — mandatory — **success conditions** (specific, measurable: tests / assertions /
benchmark numbers). A task closes only when its success conditions pass, verified first-hand.

**Invariants that hold for every task below** (from the POC phase; do not regress):
- The SUMO **parity lane core stays untouched** — determinism hash `909605E965BFFE59` unchanged; full
  `Sim.ParityTests` green. Pedestrians are a separate engine (Principle 6).
- Crowd changes are **behaviorally validated**, and any parallel/perf change is **bit-identical to serial**
  (the POC-7a gate style) or gated behind an explicit fast-mode flag.
- **Coordinate on `Engine.cs`/routing** with the parallel lane-engine session — Stage P4 is the only stage
  that touches the lane Engine, and it must be scheduled with that session.

The current POC code (`src/Sim.Pedestrians/{Lod,Navigation,Crossing,Obstacles,Parking,Density}`,
`src/Sim.Pedestrians.Nav.DotRecast`) is proof-of-concept quality: correct and tested, but built for
demonstration. Production hardening is folded into the stages below, not treated as a separate pass.

---

## Stage P0 — Crowd-store `Add`/`Remove` (PRIORITY)

**Why first:** POC-7c measured churn (moving interest sources continuously promoting/demoting) at **3.6×**
the stable step cost at 100k, entirely because `OrcaCrowd`/`MixedTrafficCrowd` have no agent removal and
`PedLodManager` rebuilds the whole high-power crowd on every membership change (O(high) per switch). A
real `Add`/`Remove` (O(1) per switch) removes most of that gap and is the single highest-value follow-up.
Design ref: `PEDESTRIAN-DESIGN.md` §3(d), `PEDESTRIAN-POC7C-FINDINGS.md` Q2.

### P0-1 — `OrcaCrowd` stable-handle `Add`/`Remove`
- **Design ref:** §3(d). **Files:** `src/Sim.Core/Orca/OrcaCrowd.cs` (+ a handle type). **Deps:** none.
- Add a real removal path: agents addressed by a **stable handle** (index + generation), a **free-list** of
  vacated slots recycled on `Add`, and a `Remove(handle)` that vacates a slot without disturbing others'
  handles. Keep the SoA/plan-execute/spatial-hash design; removed slots are skipped (like `_active`) and
  reused. Preserve deterministic iteration order (iterate live slots in a fixed order; document it).
- **Success conditions:**
  1. A dedicated test adds N, removes an arbitrary subset, adds more, and asserts surviving agents' handles
     still resolve to the correct positions and stepping is unaffected.
  2. **Determinism gate:** a fixed add/remove script produces bit-identical trajectories run-to-run, and
     (with `UseParallelStep`) parallel == serial bit-identical (extend `OrcaParallelStepTests`).
  3. Full `Sim.ParityTests` + `Sim.Pedestrians.Tests` green (existing OrcaCrowd behavior unchanged when
     `Remove` is never called — byte-identical to today for the no-remove path).

### P0-2 — `MixedTrafficCrowd` `Add`/`Remove` + dynamic external-disc input
- **Design ref:** §3(d), §6; `PEDESTRIAN-POC6*`. **Files:** `src/Sim.Core/Mixed/MixedTrafficCrowd.cs`.
  **Deps:** P0-1 (share the handle/free-list pattern).
- Mirror P0-1's `Add`/`Remove` on `MixedTrafficCrowd`, and add a **`SetExternalObstacles(WorldDisc[])`**
  equivalent (POC-6b found it has only permanent `AddWall`/`AddBlock`) so a maneuvering car can avoid
  *moving* pedestrians without a per-step crowd rebuild.
- **Success conditions:**
  1. Remove/re-add a maneuvering agent mid-run; survivors unaffected; deterministic.
  2. `SetExternalObstacles` makes a car avoid a moving disc with velocity awareness; a test shows the car
     yields to a *moving* ped better than the POC-6b per-step-`AddBlock` approximation (measurable earlier
     avoidance), no overlap.
  3. Parity + ped suites green.

### P0-3 — `PedLodManager` / `LotCoupling` use `Add`/`Remove` (retire the rebuild)
- **Design ref:** §5. **Files:** `src/Sim.Pedestrians/Lod/PedLodManager.cs`,
  `src/Sim.Pedestrians/Parking/LotCoupling.cs`. **Deps:** P0-1, P0-2.
- Replace `RebuildHighCrowd` (and `LotCoupling`'s per-step car-crowd rebuild) with incremental
  `Add`/`Remove` on promotion/demotion. Keep the demotion-reroute-from-current-position behavior.
- **Success conditions:**
  1. All existing POC-3 / POC-6 tests still pass unchanged.
  2. **Churn benchmark (`Sim.BenchPedLod`) Scenario B is now near-flat vs Scenario A** — the per-switch cost
     is O(1), not O(high). Target: at 100k, B's parallel ms/step within ~1.3× of A's *for equal high-power
     counts* (isolate the "more high-power agents" effect from the rebuild effect — the finding to erase is
     the rebuild overhead). Record before/after in `PEDESTRIAN-POC7C-FINDINGS.md`.

---

## Stage P1 — Consolidate `Sim.Pedestrians` into a production API + interest-source system

### P1-1 — Interest-source field (movable, multi-source, spatial)
- **Design ref:** §5 (movable interest sources; use cases 1–4). **Files:** `src/Sim.Pedestrians/Lod/`.
  **Deps:** P0-3.
- Promote the POC-3 single-source promotion into a production **interest-source field**: an updatable set of
  movable sources (entity/avatar-attached, static AoI, intrinsic crosswalk/parking/incident), queried per
  low-power ped via the crowd spatial hash (cheap because sources are few). Deterministic, hysteretic.
- **Success conditions:** a test with several independently-moving sources promotes exactly the peds inside
  any promote-radius, demotes correctly with hysteresis (no flap), and the per-step interest scan is
  sub-linear in ped count (measured: adding sources or peds does not blow up the stable-scenario ms/step).

### P1-2 — API surface + packaging *(scheduled BEFORE P5 — evac (P5-B) consumes it)*
- **Design ref:** §10. **Files:** `src/Sim.Pedestrians/*`, new `README.md`, `.csproj` packaging. **Deps:** P1-1.
- A coherent `PedestrianWorld` **facade** over the common wiring (`PedLodManager` + `InterestField` +
  `PedPublisher`): `AddWalker`/`AddLivelyWalker`, `SetForcedHighPower(id,on)` (the panic/pin control),
  `AddInterestSource`/`MoveInterestSource`/`RemoveInterestSource`, `SetExternalObstacles`, `Step(now,dt)`,
  `PositionOf`/`ModelOf`/`Remove`/`LiveIds` — so a consumer never hand-wires the internals. Add
  `PedLodManager.SetForcedHighPower` (a pinned ped promotes immediately and never demotes while pinned;
  default off → bit-identical). Make `Sim.Pedestrians` a packable library (packaging metadata on the
  `.csproj`); DotRecast provider stays its own optional package.
- **Success conditions:** the package builds/packs; a short sample drives a scenario through the facade
  only (no internals); `SetForcedHighPower` promotes a ped regardless of interest field and holds it high;
  the unpinned path stays bit-identical (existing ped tests unchanged); README documents the seams;
  hermetic `dotnet test` unaffected.

### P5-PRE — Synthetic pedestrian evac district (walkable net) *(prerequisite for P5-B)*
- **Design ref:** §4 (bake), §6. **Files:** new `scenarios/_ped/evac-district/` (`*.net.xml` +
  `walkable.add.xml` + safe-zone anchors). **Deps:** P2-1.
- Author/generate a synthetic SUMO net with real pedestrian infrastructure (sidewalks + crossings +
  walkingareas) over a few blocks, an incident-able interior, and **safe-zone anchor points** at the
  fringe (the flee destinations). It must bake cleanly through `SumoNavMesh`/`WalkablePolygonBaker`.
- **Success conditions:** `SumoNavMesh` bakes a connected walkable graph; `FindPath(interiorPoint,
  safeZone)` returns a route along sidewalks/crossings (not a straight line through buildings); committed
  as a reusable evac scenario.

---

## Stage P2 — Navigation productionization (behind the seam)

### P2-1 — Harden the SUMO-geometry bake
- **Design ref:** §4; POC-1a notes. **Files:** `src/Sim.Pedestrians/Navigation/Bake/`. **Deps:** none.
- Replace the per-segment sidewalk quad approximation with **whole-polyline mitred/rounded strips**, and add
  **vertex-proximity adjacency** for bent sidewalks (POC-1a disabled it to fix a corner bug; do it correctly)
  so multi-segment sidewalks connect. Add a **static-obstacle spatial index** to `OrcaCrowd` (obstacle
  queries are O(n) — §3(b)) for many buildings/parked cars.
- **Success conditions:** a network with bent multi-segment sidewalks routes correctly (path stays walkable,
  no false disconnection); a scene with hundreds of static boxes/buildings steps at a rate that scales
  sub-linearly in obstacle count (benchmark before/after the index).

### P2-2 — Dynamic blockers + reroute in the navigation API
- **Design ref:** §4, §6; POC-5. **Files:** `src/Sim.Pedestrians/Navigation/`. **Deps:** P2-1.
- Generalize POC-5's blocked-polygon reroute into the production `IPedNavigation` (dynamic obstacle
  registration → affected agents reroute), with hysteresis so a transient blocker doesn't thrash routes.
- **Success conditions:** registering/unregistering a blocker reroutes only affected peds; no route thrash
  when a blocker flickers; deterministic.

### P2-3 — Production navmesh integration contract + OD demand
- **Design ref:** §4, §10. **Files:** `src/Sim.Pedestrians/Navigation/`. **Deps:** P1-2.
- Document and example the `IWalkableSpace`/`IPedNavigation`/`ILocalSteering` contract the **owner's
  production navmesh** implements; provide an adapter template. Add pedestrian **origin→destination demand**
  (spawn/route/despawn at scale) so a scenario populates itself.
- **Success conditions:** a second real `IWalkableSpace` implementation (or a documented adapter stub for
  the owner's navmesh) drives the same pedestrian layer unchanged; an OD-demand scenario sustains a target
  population with continuous spawn/arrival.

---

## Stage P3 — Networking productionization (DDS multicast, end-to-end)

### P3-1 — Pedestrian wire codec + transport-neutral replication surface (hermetic)
- **Design ref:** §7 (multicast, one stream); POC-7b; the vehicle `IReplicationSink`/`Source` +
  `InMemoryReplicationBus` pattern. **Files:** `src/Sim.Replication/` (Records, FrameCodec, a ped
  replication surface + InMemory binding). **Deps:** none (POC-7b landed `PedFreeKinematicRecord` + wire
  `PathArcRecord`); LIVE-POC-1 added `ActivityTimeline`.
- Add the missing pedestrian wire records: an **`ActivityTimeline` codec** (T0 + segment list: Walk/Pause/
  Dwell/Interact, each length-prefixed, anim tags as length-prefixed UTF-8) and a **ped lifecycle record**
  (spawn/despawn + DR-model switch promote/demote). Build a **transport-neutral `IPedReplicationSink`/
  `IPedReplicationSource`** (mirroring the vehicle pair) with an **InMemory binding** so the hermetic
  `dotnet test` loop can round-trip the whole stream with NO DDS. The real CycloneDDS binding is a
  separate, out-of-`Traffic.sln` concern (like the existing vehicle DDS binding) — NOT part of this task's
  hermetic gate.
- **Success conditions:** a round-trip test runs a real `PedLodManager` population (low-power PathArc +
  lively `ActivityTimeline` + a promoted `FreeKinematic` ped), serializes the full `PedEvent` stream through
  the InMemory ped bus, and reconstructs on the receiver: **server==IG EXACT** for low-power (path/timeline
  sent once as doubles, reconstructed by the same `PoseAt`/`PathArcMotion`) and **within cm quantization
  tolerance** for high-power (`PedFreeKinematicRecord` int32-cm); a DR-switch on the wire flips the receiver's
  reconstruction model at the right time; every byte-level codec round-trips (encode→decode identity for
  each record type); `dotnet test` stays hermetic (no DDS, no network) and parity is untouched.

### P3-2 — Publisher + global bandwidth governor
- **Design ref:** §7. **Files:** `src/Sim.Replication/PublishPolicy.cs` (+ a ped publisher). **Deps:** P3-1, P0-3.
- Productionize the POC-3 `PedPublisher`: DR-error-gated `FreeKinematic` for high-power, `PathArc`-once +
  heartbeat for low-power, DR-model switch on promotion, all on the single stream. Add a **global bandwidth
  governor** `IPublishPolicy` that throttles high-power send-rate under spike load (there is no per-channel
  culling — multicast).
- **Success conditions:** at the target mix, the measured single-stream rate stays under 500 Mbit/s even in a
  mass-promotion spike (governor engages); low-power peds emit zero per-step samples (POC-3 invariant holds
  at scale).

### P3-3 — IG-side reconstruction
- **Design ref:** §7; POC-3 `HeadlessIg`. **Files:** `src/Sim.Viewer.Motion/` (or a ped viewer consumer).
  **Deps:** P3-1.
- Add the `FreeKinematic` extrapolator and the `PathArc` follower to the viewer DR pipeline
  (`DrClock`/`PoseResolver`), applying regime lifecycle + DR-switch events at their event time.
- **Success conditions:** an end-to-end test/demo streams a mixed crowd and the IG reconstructs low-power
  peds from the one-time path (server==IG within render tolerance, the POC-3 invariant, over DDS) and
  high-power peds from the position stream, switching correctly on promotion.

---

## Stage P4 — Engine coordination seams (Core; schedule with the lane-engine session)

### P4-1 — Engine TLS-crossing signal projection
- **Design ref:** §6; POC-2 finding. **Files:** `src/Sim.Core/Engine.cs` (a new read-only projection).
  **Deps:** coordinate with the parallel Engine session.
- POC-2 found `Engine.TlStates` excludes pedestrian crossings (internal→internal links). Add a read-only
  projection exposing each crossing's controlling `tlLogic` + link state so the crossing gate reads the
  **live** Engine signal instead of re-deriving phase timing from the net XML.
- **Success conditions:** the crossing gate, fed the live projection, opens/closes exactly with the Engine's
  crossing signal; parity hash unchanged (Step-only projection, off the golden path); coordinated merge with
  the Engine session.

---

## Stage P5 — Evac generalization

### P5-1 (B) — `Sim.Evac` consumes `Sim.Pedestrians` on a real walkable net
- **Design ref:** §6 (evac = specialization). **Files:** `src/Sim.Evac/*` (a new evac scenario on the P5-PRE
  net). **Deps:** P1-2 (the facade + `SetForcedHighPower`), P5-PRE (the synthetic walkable net), P2-3.
- Rebuild the pedestrian side of evac on the `PedestrianWorld` facade over the P5-PRE walkable net: panic =
  `SetForcedHighPower` (forced promotion to reactive ORCA) + the flee param override; the fleeing ped's
  destination = the **nearest safe zone routed via `IPedNavigation`** (`SumoNavMesh`) along real sidewalks/
  crossings, replacing `FleeGoalFor` radial steering (and, for the ped side, the fake-navmesh band). Abandoned
  cars feed in as external obstacles (`SetExternalObstacles`). `FearField`/`LineOfSight`/`BlockedDetector`
  reused unchanged. Keep the legacy `FakeNavMesh`/radial path available for the existing (netless) evac
  scenarios so their aggregate-property tests stay green; the NEW walkable-net evac scenario exercises the
  routed flee.
- **Success conditions:** a new evac-district scenario runs end-to-end; panicked peds route to the nearest
  safe zone **along walkable space** (a ped's path bends around a block, not straight through it) and reach
  it (escaped); existing evac demo tests (aggregate-property) still pass; a `--evac-district`/Sim.Viz demo
  shows the routed foot-exodus; hermetic `dotnet test` unaffected.

---

## Stage LIVE-PROD — Graduate liveliness into the demand path

The LIVE-POCs proved the mechanisms in isolation (`ActivityTimeline`, `SocialPlanner`, `WaiterScenario`);
this stage makes the *routed ambient crowd* actually lively. Design refs: `PEDESTRIAN-LIVELINESS-DESIGN.md`
§4 (schedule generator extends `PedDemand`) + §10 (LOD integration). Iron rule: strictly additive — with
liveliness disabled the population is **bit-identical** to today's PathArc behaviour.

### LIVE-PROD-1a — `PedLodManager` low-power `ActivityTimeline` path
- **Design ref:** liveliness §10; `PedLodManager.cs`, `ActivityTimeline.cs`. **Deps:** LIVE-POC-1.
- A low-power ped may carry an `ActivityTimeline` (`Model = PedDrModel.ActivityTimeline`) instead of a bare
  PathArc leg: `AddPedLively(id, timeline, radius, now)` publishes `ActivityTimelineRecord` + the switch;
  `PositionOf` evaluates `PoseAt`; promotion carries the timeline's pose+velocity forward into the crowd
  (add `ActivityTimeline.VelocityAt`); demotion returns to a plain PathArc walk-to-destination. Low-power =
  PathArc **or** ActivityTimeline throughout the promote/demote state machine.
- **Success conditions:** every existing `PedLodManager`/`PedDemand` test stays **bit-identical** (null
  timeline path unchanged); a new test drives a lively low-power ped through a `PedPublisher`→`HeadlessIg`
  round-trip and asserts exact server==IG over a sweep, and asserts a lively ped inside a promote radius
  promotes to FreeKinematic and demotes back; serial==parallel; 589 parity green.

### LIVE-PROD-1b — `PedDemand` schedule generator + lively-crowd demo
- **Design ref:** liveliness §4, §8; `PedDemand.cs`. **Deps:** LIVE-PROD-1a.
- `PedDemandConfig` gains an optional liveliness block (per-ped probability of a Pause/Dwell beat, a POI/
  dwell-spot set, seeded from `config.Seed`+id). `TrySpawnOne` builds an `ActivityTimeline` = the route as
  Walk segments with occasional deterministic Pause("sip"/"phone")/Dwell(at a nearby POI) inserted, and
  calls `AddPedLively`. A `--ped-lively-crowd` Sim.Viz scene shows the routed crowd sipping/sitting/dwelling
  as it moves.
- **Success conditions:** determinism (same seed → identical spawn/pose stream); with the liveliness block
  omitted the demand is bit-identical to today's `PedDemand`; the demo visibly shows lively beats along
  real routes; arrivals still despawn at destination (a dwelling ped does not despawn mid-route); ped tests
  green.

---

## Stage P6 — Scale hardening + on-target validation

### P6-1 — On-target benchmark run
- **Design ref:** §9; POC-7 findings (all measured on a 4-core VM). **Files:** benchmarks. **Deps:** P0-3.
- Run `Sim.BenchCrowd` / `Sim.BenchPedLod` / `Sim.BenchPedNet` on the **16+‑core Windows target** and record
  real steps/s + Mbit/s; update the findings docs with on-target numbers replacing the 4-core estimates.
- **Success conditions:** measured 100k-ped stable steps/s is interactive on the target box; the churn case
  (post-P0) is within the interactive band; bandwidth confirmed under budget on real DDS multicast.

### P6-2 — Region decomposition (only if P6-1 shows the flat parallel `Step` plateaus)
- **Design ref:** §9; `DOMAIN-DECOMP.md`. **Files:** `src/Sim.Core/Orca/`. **Deps:** P6-1.
- If flat `Parallel.For` plateaus below target on the 16-core box, region-decompose the crowd (the
  `ComputeLaneRegions` pattern) with free cross-region handoff.
- **Success conditions:** measurable speedup over flat parallel at 100k on the target box, still bit-identical.

### P6-3 — Full property-test hardening
- **Design ref:** §8. **Files:** `tests/Sim.Pedestrians.Tests/`. **Deps:** all above.
- Consolidate the per-POC property tests into a maintained production suite: no-overlap, arrives-within-N,
  never-leaves-walkable, no-flap, server==IG, determinism — run across the promoted-to-production scenarios.
- **Success conditions:** the suite covers every requirement (1–7) with a named test; all green; parity
  untouched.

---

## Stage P7 — Pedestrian visualization (existing 3D viewers, in-process AND remote-with-DR)

**Why a stage:** the sim is only useful if you can *see* the pedestrians. Two viewers already render cars
(and evac has a pedestrian precedent), and each supports two data paths — **in-process** (the viewer
co-hosts the sim and renders live state directly, no DR) and **remote** (the viewer consumes the DDS
stream and reconstructs via dead-reckoning). Pedestrians must render in **both viewers** over **both
paths**. Design ref: §7 (DR / multicast), §5 (regime rendering); precedent: `src/Sim.Viewer/EvacOverlay.cs`,
`src/Sim.Viewer/EvacRenderSnapshot.cs`, `demos/City3D/CityLib/{SimSource,Reconstructor}.cs`.

### P7-1 — Native viewer (`Sim.Viewer` / `Sim.Viewer.Raylib`): in-process pedestrian render
- **Design ref:** §5. **Files:** `src/Sim.Viewer/`, `src/Sim.Viewer.Raylib/RenderHelpers.cs`. **Deps:** P1-2.
- Generalize the existing evac ped **overlay** into a first-class pedestrian render driven by live
  `PedLodManager`/crowd state: draw each ped as an oriented agent (heading from velocity), **regime-aware** —
  low-power and high-power peds both drawn (optionally tinted for debugging), and `board`/`alight` shown as
  appear/disappear at the vehicle. Reuse the evac snapshot pattern (`EvacRenderSnapshot`) so reading ped
  state never blocks the sim step.
- **Success conditions:** a co-hosted run renders a moving pedestrian crowd (crossing, parking, corridor)
  at interactive fps; board/alight visibly appears/disappears beside cars; a headless smoke test asserts the
  ped render snapshot is populated and non-blocking.

### P7-2 — Native viewer: remote pedestrian render over DR/DDS
- **Design ref:** §7. **Files:** `src/Sim.Viewer.Motion/`, `src/Sim.Viewer/`, `src/Sim.Viewer.Core/DdsPublisher.cs`.
  **Deps:** P3-1, P3-3.
- Feed the same ped render from the **DDS stream** instead of live state: `FreeKinematic` high-power peds
  extrapolated and `PathArc` low-power peds followed via the P3-3 reconstruction, with regime/DR-switch and
  board/alight lifecycle events applied. Same avatar rendering as P7-1 — only the data source differs.
- **Success conditions:** a publisher→multicast→viewer run renders the crowd smoothly between sparse
  updates; **server==IG visual parity** — a low-power ped's rendered position matches the co-hosted render
  within render tolerance (the POC-3 invariant, now on-screen); no pop/teleport on promotion (DR-switch
  reconciled by the smoother).

### P7-3 — City3D (Godot 3D): pedestrian render, in-process + remote
- **Design ref:** §5, §7. **Files:** `demos/City3D/CityLib/{SimSource,Reconstructor}.cs`,
  `demos/City3D/Viewer/`. **Deps:** P7-1, P7-2 (shares the render model), P3-1/P3-3.
- Add pedestrian avatars to the Godot 3D city viewer along the paths it already uses for cars: **in-process**
  via `SimSource` (live ped state) and **remote** via `Reconstructor` + the ped DDS topics (`run-remote.sh`
  path). 3D-specific: ground-plane placement, a simple ped mesh/billboard, heading from velocity, LOD-cull
  distant peds for the GPU (render-only culling — independent of sim/net LOD).
- **Success conditions:** the 3D viewer shows the pedestrian crowd in both `local` and `remote` (DDS) launch
  modes; peds move smoothly under DR in remote mode; large counts render without stalling (GPU LOD engaged).

---

## Stage P8 — Subarea integration (SumoData compatibility)

Realizes the 7 compatibility requirements in `SUBAREA-FOR-PEDESTRIAN-SESSION.md` §3. The design consequences
are in `PEDESTRIAN-LIVELINESS-DESIGN.md` §11; the full per-requirement mapping + serve-path touchpoint is in
`COORDINATION-pedestrian-x-subarea.md`. Standing invariant: all of P8 is **additive and inert by default** —
an empty camera visible set → fully permissive → every existing pedestrian scenario/golden unchanged
(mirrors the engine's null-`RealismMask` default).

### P8-1 — Verify the bake against a real cropped sub-area net
- **Design ref:** coord §1 req 1. **Files:** test-only (a cropped box `net.xml` fixture under
  `scenarios/_ped/`), `SumoNavMesh` (read-only verification, fix only if a crop breaks it). **Deps:** P2-1;
  needs a real cropped box `net.xml` from the SumoData pipeline.
- Run the SUMO-geometry bake on an actual cropped box (crossings/walkingAreas cut by the boundary produce
  dangling "fringe" stubs) and confirm the walkable graph is sane (fringe edges identified, no spurious
  adjacency across the cut, coordinate frame matches the vehicle net).
- **Success conditions:** bake of a cropped net produces a connected walkable graph whose fringe edge set
  equals the boundary-cut walkable edges; a golden-style assertion pins the fringe set for the fixture; no
  change to any existing (uncropped) scenario.

### P8-2 — Appearance-legitimacy layer (the no-cheating gate) — CORE
- **Design ref:** coord §1 req 2+4, coord §2, liveliness §11. **Files:** new
  `src/Sim.Pedestrians/Legitimacy/PedSpawnPolicy.cs` (+ a visible-walkable-edge set type), wiring in
  `PedDemand` (spawn) and `PedLodManager` (despawn/end-of-route). **Deps:** P1-1 (camera interest source),
  P2-3 (`PedDemand`), P8-1 (fringe set).
- Add the axis the earlier design conflated with sim-LOD: `MaySpawnOrDespawn(ped, walkableEdge) = isFringe(e)
  OR hostsLegitimateSink(e, ped) OR isOffCamera(e)`, where `isOffCamera` reads the **same** host camera
  signal the vehicle side uses (analogue of `RealismMask.MayPop`) mapped to the visible **walkable**-edge
  set; `hostsLegitimateSink` = a building-entrance / transit / parking board-alight POI the ped is using
  (liveliness §6/§8). A denied on-camera despawn routes the ped to the nearest sink/fringe or holds it
  low-power until off-camera; a denied on-camera spawn defers. Camera remains a sim-LOD interest source
  (unchanged); it *additionally* drives this gate. Pure function of (seed, ped, edge, visible set); visible
  set captured once per host tick.
- **Success conditions:** with a visible-edge set active, a deterministic scenario produces **zero**
  ped appear/despawn events on a visible walkable edge that is not a fringe/sink (assert over the event log);
  with an empty visible set the run is **bit-identical** to the pre-P8-2 baseline (inert-default gate);
  serial == parallel; 585 parity + ped tests green.

### P8-3 — Auto-deduced pedestrian demand
- **Design ref:** coord §1 req 3, liveliness §4+§8. **Files:** `PedDemand` (deduction pass), POI ingest for
  the liveliness §8 schema. **Deps:** P8-1, P8-2, P2-3.
- Deduce O→D + liveliness POIs from walkable-space + land-use/POI net-data (sidewalk density, building
  entrances, transit/parking, plazas), mirroring the vehicle side's topology deduction (their
  `deduce_weights.py` as a template). Spawns land at fringe/doors per the P8-2 gate. Hand-authored
  `personFlows` remain the stopgap until this lands.
- **Success conditions:** on the cropped fixture, deduced demand yields fringe/door-legitimate O→D with a
  reproducible per-(seed, box) distribution; a probe run is deterministic across repeats; no on-camera pops
  (via P8-2).

### P8-4 — Pedestrian density knob + crossing-throughput guard
- **Design ref:** coord §1 req 5, liveliness §11. **Files:** `PedDemand` (density target),
  `Crossing/CrossingGate` (occupancy cap). **Deps:** P8-3.
- Expose a pedestrian density knob (peds/area or peds/sidewalk-m) analogous to the vehicle density level and
  document its safe range; cap crossing occupancy so crowds never deadlock a signalized crossing hard enough
  to gridlock the (separately calibrated) cars.
- **Success conditions:** the knob linearly targets measured density within a documented range; a
  crossing-saturation scenario shows crossings drain (no permanent deadlock) and the coupled cars keep
  flowing; documented safe range committed.

### P8-5 — Scenario/manifest slot-in + shared-replay contract
- **Design ref:** coord §1 req 6, coord §3. **Files:** ped demand emitter (references/attaches to
  `scenario.sumocfg` + `manifest.json`), `Sim.Viz` (shared FCD-style ped stream — already renders discs,
  P7-0). **Deps:** P8-3.
- Emit pedestrian demand as **additional** route/person input referenced by (or alongside) the produced
  `scenario.sumocfg`, record the ped-density knob + safe range in the manifest, and confirm one replay can
  render cars and peds from the same trajectory stream.
- **Success conditions:** a produced `scenario.sumocfg` + `manifest.json` round-trips with ped inputs
  attached; a single Sim.Viz replay shows vehicles and pedestrians together from one stream; outputs stay
  self-contained/offline.

---

## Sequencing summary

**P0 first (Add/Remove — the priority).** Then P1 (API/interest-source) and P2 (navigation) can proceed in
parallel; P3 (networking) depends on P0-3; **P7 (visualization) follows its data sources — P7-1 in-process
after P1, P7-2/P7-3 remote after P3-1/P3-3** (it is the payoff that makes the whole system watchable, so
schedule P7-1 early for a visible in-process demo); P4 (Engine TLS seam) is scheduled with the lane-engine
session whenever convenient; P5 (evac) waits on P1–P2; P6 (on-target scale) waits on P0 and closes the loop
with the real hardware numbers. **P8 (subarea integration) waits on P2-3 + P1-1 and a real cropped box
`net.xml` from the SumoData pipeline; P8-1 (crop verification) is cheap and can run the moment a crop is
available, P8-2 (the appearance-legitimacy gate) is the load-bearing new piece.** The design is fixed; any
P-stage finding that contradicts it updates `PEDESTRIAN-DESIGN.md` before that stage closes.
