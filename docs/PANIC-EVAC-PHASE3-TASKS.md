# PANIC-EVAC-PHASE3-TASKS.md — stages, tasks, success conditions (Option A)

Task breakdown for **Phase 3, Option A** (external shaped-mover handoff — `PANIC-EVAC-PHASE3-DESIGN.md`
§3–§4, signed off). Design refs point at that doc; requirements at `PANIC-EVAC.md` (§6.3, R2, R8); the
shaped substrate is `Sim.Core.Mixed.MixedTrafficCrowd` (`INDIA-TRAFFIC.md`). Checklist:
`PANIC-EVAC-PHASE3-TRACKER.md`. Each task lists checkable success conditions; a task closes only when all
pass. Loop: Sonnet implements a batch, Opus reviews hard before ticking.

The new lifecycle stage: `Organized → Flee → [blocked] → Orca-push (shaped free-space) → [wedged] →
Pedestrian`. Orca-push is an evac-owned `MixedTrafficCrowd` mover; the lane engine is untouched.

---

## Stage S1 — VehicleMover + config

### T1.1 — `EvacConfig` Phase-3 tunables
- **Design:** DESIGN §4, §9.3. **Files:** `src/Sim.Evac/EvacConfig.cs`.
- Add (Option-A defaults ON so the demo pushes): `EnableOrcaPush=true`; `OrcaWedgeSpeed=0.1` (m/s below
  which a mover counts as not progressing); `OrcaWedgeDwellSeconds=3.0`; `OrcaPushSafetyMargin=0.3`
  (shaped-VO tracking inflation); `OrcaCrowdSubSteps=10`.
- **Success:** fields present with those defaults; `EnableOrcaPush=false` must recover exact Phase-1/2
  behaviour (verified in T3.1 cond. 3).

### T1.2 — `VehicleMover` (wraps `MixedTrafficCrowd`)
- **Design:** DESIGN §1, §4. **Files:** `src/Sim.Evac/VehicleMover.cs` (new).
- A `Sim.Evac`-owned manager over a `MixedTrafficCrowd`: `AddCar(Vec2 pos, double headingRad, Vec2 goal)
  → int index` (uses `VehicleClass.Car`, `SafetyMargin=OrcaPushSafetyMargin`); `SetGoal(i, goal)`;
  `Step(dt)` (sub-stepped `OrcaCrowdSubSteps`); `Position(i)/Heading(i)/Velocity(i)/Count`;
  `Deactivate(i)`; wall setup `AddWall/AddBlock` passthrough; per-mover **wedge tracking** (a mover whose
  `Velocity(i).Abs < OrcaWedgeSpeed` for `OrcaWedgeDwellSeconds` reports wedged, via an internal dwell
  timer keyed by mover index; resets when it moves).
- **Success (unit tests, no Engine):** (a) a car added in a walled box with a reachable goal moves toward
  it (position closes on goal over N steps); (b) a car boxed with no reachable exit reports wedged after
  the dwell and not before; (c) heading is non-holonomic — the car does not translate purely sideways in
  one step (displacement is within the bounded-turn cone of its heading), reusing the `MixedTrafficCrowd`
  guarantees.

---

## Stage S2 — Band walls (the push region)

### T2.1 — `FakeNavMesh` mover-band walls
- **Design:** DESIGN §4 (band, first cut = outer hard edge). **Files:** `src/Sim.Evac/FakeNavMesh.cs`.
- Expose the band boundary as shaped walls for the mover crowd: first cut reuses the existing
  bounding-box `BoundaryLoop` as four `AddWall` segments (a helper the director calls to arm a
  `VehicleMover`/`MixedTrafficCrowd`). (The richer per-lane buffered band is the follow-up sub-task
  T2.2, deferred unless the push doesn't read well.)
- **Success:** a `VehicleMover` armed with the band walls **confines** a mover — a car driven toward a
  goal outside the boundary stops at/inside the wall and never crosses it (containment invariant, the
  shaped analogue of the pedestrian no-cross-edge test).

---

## Stage S3 — EvacDirector integration

### T3.1 — Orca-push stage in the lifecycle
- **Design:** DESIGN §2, §4. **Files:** `src/Sim.Evac/EvacDirector.cs`.
- Add a `VehicleMover` to the director (armed with the band walls in the ctor). Extend `VehState` with a
  `Pushing` flag + `MoverIndex`.
- **PostStep change:** `blocked && panicked` →
  - if `EnableOrcaPush` and not already pushing/converted: **EnterOrcaPush** — `Despawn(handle)`,
    `mover.AddCar(pose, heading, FleeGoalFor(pose))` (heading from last `VehicleState.Angle`), set
    `Pushing`, record `MoverIndex`; the lane car is gone, the shaped mover now carries it.
  - else (push disabled): the existing `Convert`→pedestrian (unchanged Phase-1 path).
- **Each tick:** set every pushing mover's goal to the away-from-incident direction; `mover.Step(dt)`;
  a mover that reports **wedged** → run the pedestrian conversion from its current pose (spawn
  pedestrian(s), add its footprint to `_abandoned` as a static block/disc, `mover.Deactivate`), mark
  `Converted`.
- **Cross-avoidance:** (a) feed pushing-mover footprints into `FeedVehicleDiscsToPeds` so pedestrians
  avoid them; (b) abandoned cars added as mover blocks so pushers avoid wrecks; (c) make lane vehicles
  avoid pushers too — replace the single `Engine.CrowdSource = _peds` with a **composite
  `ICrowdFootprintSource`** that queries both the pedestrian crowd and the pushing movers (new small
  `Sim.Evac` class; still one seam into the engine, still parity-inert when empty).
- Observability: `OrcaPushCount`, `PushingCarPose(i)` (x,y,heading), `IsPushing(handle)`.
- **Success conditions:**
  1. On the grid (defaults), some cars enter Orca-push (`OrcaPushCount` peaks > 0) before any becomes a
     pedestrian; the cascade still completes (pedestrians eventually spawn, some escape).
  2. Wedged pushers convert to pedestrians (a pushing mover that stops for the dwell yields a pedestrian).
  3. **Backwards compatibility:** with `EnableOrcaPush=false`, the director behaves exactly as Phase-2 —
     the existing `EvacSpineTests` and `EvacPhase2Tests` pass unchanged (blocked→pedestrian directly).

---

## Stage S4 — Behavioural / determinism / parity validation

All in `tests/Sim.ParityTests/EvacPhase3Tests.cs` (new).

### T4.1 — Push precedes foot exodus
- **Success:** with `EnableOrcaPush=true`, over a grid run `OrcaPushCount` reaches > 0 and at least one
  car transitions lane→push→pedestrian (push is really an intermediate stage, not skipped); with
  `EnableOrcaPush=false` on the same setup, `OrcaPushCount==0` and the run still cascades (pedestrians > 0).

### T4.2 — Shaped confinement (band wall)
- **Success:** across a grid run, every pushing mover stays within the navmesh bounds every tick (no
  shaped car crosses the hard edge) — the vehicle analogue of the pedestrian containment invariant.

### T4.3 — Wedge → pedestrian
- **Success:** a `VehicleMover`-level or director-level test where a pushed car is boxed (goal
  unreachable) converts to a pedestrian after `OrcaWedgeDwellSeconds` and not before.

### T4.4 — No shaped interpenetration
- **Success:** during the push, no two active movers overlap beyond the safety margin (sample min
  centre-distance vs summed footprint extents across the run stays ≥ 0 within tolerance), leaning on the
  `MixedTrafficCrowd` SAT/margin guarantee.

### T4.5 — Determinism
- **Success:** two grid runs fold to a bit-identical signature that INCLUDES pushing-mover poses +
  `OrcaPushCount` (extends the Phase-2 determinism check).

### T4.6 — Parity / inertness + gate
- **Success:** `NoIncident` ⇒ 0 pushers, 0 panicked, fear 0; full `dotnet test` green; `Sim.Bench`
  `hashA==hashPar==909605E965BFFE59` (Option A adds no Engine code, so this must hold trivially).

---

## Stage S5 — Viz: cars mounting the shoulder

### T5.1 — Emit pushing cars as oriented shaped boxes
- **Design:** DESIGN §7. **Files:** `src/Sim.Viz/SceneGen.cs` (+ `Payload.cs` only if a new disc kind
  needs a label).
- **Success:** `BuildEvacGrid` reads `director` pushing-mover poses and emits them as **shaped** disc
  entries (`[x,y,radius,kind,headingDeg,shape,halfLen,halfWid]`, the format `drawShaped` already
  consumes) with a distinct kind/tint; lane cars + pedestrians + abandoned cars unchanged.

### T5.2 — Render + confirm
- **Files:** `src/Sim.Viz/template.js` (palette/label for the new kind if needed; `drawShaped` reused).
- **Success:** the bundle renders and a blocked fleeing car is visibly seen **nosing off the lane into
  the shoulder** (oriented, turning like a car), then — when wedged — becoming a pedestrian. Opus renders
  the bundle at several timesteps to confirm; suite green; hash unmoved.
