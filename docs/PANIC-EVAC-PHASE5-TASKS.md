# PANIC-EVAC-PHASE5-TASKS.md — scale: stages, tasks, success conditions

Task breakdown for Phase 5 (`PANIC-EVAC-PHASE5-DESIGN.md`). **Tier 1 is fully specified below; Tier 2 is
outlined** and will get exact tasks in a Tier-2 addendum written against Tier-1's measured cost profile
(design §6). Loop as usual: Sonnet implements a batch, Opus reviews hard before ticking.

---

## TIER 1 — realistic organic town, local auto-attach (buildable now)

### Stage S1 — working-region auto-attach
#### T1.1 — `EvacConfig.WorkingRadius`
- **Design:** §3. **Files:** `src/Sim.Evac/EvacConfig.cs`.
- Add `WorkingRadius` (m, default e.g. 250) — the disc around the incident inside which vehicles become
  evac-relevant. **Success:** field present; documented as ≥ incident radius + jam margin.

#### T1.2 — `EvacDirector` auto-track mode
- **Design:** §3. **Files:** `src/Sim.Evac/EvacDirector.cs`.
- Add an opt-in auto-track: each tick (in `PreStep`, before the fear update) scan the engine read surface
  (`engine.VehicleHandles`/`PosX`/`PosY`) in **ascending EntityIndex order**; `Track` any active,
  not-yet-tracked vehicle whose position is within `WorkingRadius` of the incident. Vehicles outside are
  never tracked. Explicit `Track` (runtime-spawn demos) still works; auto-track is additive.
- **Success conditions:**
  1. With auto-track on, only vehicles that enter the working region are ever tracked (a vehicle that
     stays outside the region for a whole run is never in `_veh`).
  2. Deterministic: two runs track the same vehicles in the same order (fixed EntityIndex iteration).
  3. The existing grid/TLS demos (explicit `Track`) are unaffected — auto-track is off unless enabled.

### Stage S2 — organic scenario
#### T2.1 — `EvacOrganicScenario`
- **Design:** §4. **Files:** `src/Sim.Evac/EvacOrganicScenario.cs` (new).
- `LoadScenario(scenarios/_bench/city-organic-L2/{net,rou,cfg})`; build the director in auto-track mode
  with a `WorkingRadius`; incident at a busy interior junction (choose one with high through-traffic from
  the net), `StartTime` mid-run (while congestion is present); step length = the scenario config's.
  Return `(engine, director)`. `ExitEdges` = the net's boundary edges (for flee reroute).
- **Success conditions:** loads the committed organic scenario (no SUMO needed at run time); the director
  ticks; vehicles insert from the loaded demand and auto-attach in-region; the run is deterministic.

#### T2.2 — demand-under-director confirmation
- **Design:** §8. **Success:** a test confirms loaded-demand vehicles insert and are driven under the
  director's `Tick()` loop (peak concurrent > 0; some vehicles reach the region), i.e. `engine.Step()`
  inside `Tick` advances demand insertion.

### Stage S3 — behavioural tests
All in `tests/Sim.ParityTests/EvacOrganicDemoTests.cs` (new).
- **T3.1 — cascade on the organic net:** over a run, `PanickedCount > 0`, `OrcaPushCount` peak > 0,
  `PedestrianCount > 0`, some escape (maxPedDist ≥ 0.8·SafeRadius).
- **T3.2 — locality:** at least one active vehicle far outside the working region is never tracked
  (proves the evac stays local — the core Phase-5 property).
- **T3.3 — containment + determinism:** no pedestrian/pusher leaves the navmesh; two runs bit-identical.
- **T3.4 — gate:** full `dotnet test` green; `Sim.Bench` hash `909605E965BFFE59` unmoved; existing
  grid/TLS evac tests unchanged.

### Stage S4 — viz + measurement
- **T4.1 — organic viz scene** (`src/Sim.Viz/SceneGen.cs`): render the town + local evac (reuse the fear
  tint / incident overlay / pushers / pedestrians / hard edge). Camera fits the net extent. Opus renders
  to confirm congestion + foot exodus read on a realistic mesh.
- **T4.2 — cost profile (deliverable):** a small harness (e.g. in `Sim.BenchCity` or a scratch runner)
  that reports generation time + per-phase evac cost (fear update, disc feeds, ped step, pusher step) at
  the ~400-vehicle scale. **Success:** a reported breakdown identifying the dominant evac hotspot — the
  input that scopes the Tier-2 task list.

---

## TIER 2 — 10k city (outline; exact tasks deferred to a Tier-2 addendum)
Built only after Tier-1 measurement. Candidates (design §6), each parity-exempt, determinism-preserving,
and gated by measured need:
- FearField uniform grid (bit-identical, mirrors OrcaCrowd Q3).
- Spatial composite `CrowdSource.QueryNear` + spatial disc feeds (per-query O(local)).
- Enable `OrcaCrowd.UseSpatialHash`; add a `MixedTrafficCrowd` hash if pushers grow large.
- 10k-city demo scenario (big grid or big organic) + viz payload management (region-crop / decimation /
  caps, with explicit logging of anything dropped — no silent truncation).
- Working-region scan optimization only if the full read-buffer scan proves too costly at 10k.
Each will carry its own success conditions (incl. a before/after generation-time measurement and the
determinism/hash gates) in the addendum.
