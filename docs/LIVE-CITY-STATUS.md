# Live-city demo — status & handoff (resume point)

Working state of the "live city" effort (dense cars + weaving pedestrians + cars yielding to crossing
peds) on branch `claude/pedestrian-dds-transport-c8w2gf`. Read this first after a context compaction, then
the phase design docs. Gate is green throughout unless noted.

## The deliverable
`Sim.Viz --live-city out.html` (also gallery slug `live-city`): a downtown block of the demo-city box with
dense car traffic + a large weaving low-power crowd, where **cars stop for pedestrians on crossings**.
Substrate: `scenarios/_ped/demo_city/box` (bakes to `components=1`). Crop pinned to SumoData's downtown
hero bbox **`[2055,2055,2895,2895]`** in `SceneGen.BuildLiveCity`.

## Contracts / decisions (frozen)
- `docs/SUMOSHARP-LIVE-CITY-DECISIONS.md` — Q1 crossing class→yield table (signalized / unsignalized /
  discouraged), Q2–Q9, hero crop bbox, v1 behavior set.
- **Option B** (cars yield to any crossing ped, no promotion) — accepted, supersedes the spec's Option A.
- **Option A ped signal compliance** (peds wait at red) — chosen; **all 51 box TLs are `type="static"`**
  (deterministic), so waits are analytic and server==IG holds. No actuated TLs desired.

## Phases
- **Phase 1 — builder** (`docs/LIVE-CITY-2D-BUILDER-DESIGN.md`): DONE. `SceneGen.BuildLiveCity` +
  `--live-city` + `RunLiveCity`; `PedLodManager.HighPowerFootprints` accessor; W-A/W-B/W-C.
- **Phase 2 — cheap crossing-yield** (`docs/LIVE-CITY-CROSSING-YIELD-DESIGN.md`): DONE.
  `CompositeFootprintSource` (Sim.Core/Bridge), `CrossingOccupancySource` (Sim.Pedestrians/Crossing).
  `CrowdSource = Composite(HighPowerFootprints, crossingOccupancy)`. Per-crossing promotion dropped; pocket
  anchored on the central intersection. **Vehicle sim not slowed** (engine.Step ≈ 5 ms/step, occupancy
  Update ≈ 0.3 ms/step; CrowdSource null for goldens → parity byte-identical).
- **Phase 2b — peds respect the crosswalk signal** (`docs/LIVE-CITY-CROSSWALK-SIGNAL-DESIGN.md`): IN
  PROGRESS. Fixes the observed "car drives over a grey ped crossing on the car's green" — peds jaywalk
  because the low-power routed crowd has no crosswalk-signal awareness.

## Phase 2b task state
- **P2b-T1 — `CrosswalkSignalSchedule`**: DONE (committed). `src/Sim.Pedestrians/Crossing/
  CrosswalkSignalSchedule.cs`. `NextWalkStart(crossingId, tArrive, crossTime)` = earliest time ≥ tArrive a
  ped may step on and clear within one green window, from the static `<tlLogic>`. `ForCrossing(id, program,
  linkIndex)`, `FromNet(netPath, ids, actuated?)`, `IsSignalized`. 5 tests green
  (`tests/Sim.Pedestrians.Tests/Crossing/CrosswalkSignalScheduleTests.cs`).
- **P2b-T2 — splice the kerb wait into the ped timeline**: NEXT (in progress).
- **P2b-T3 — gate class-awareness**: TODO.
- **P2b-T4 — wire into BuildLiveCity + verify**: TODO.

## P2b-T2 plan (resume here) + the code facts already gathered
Goal: a low-power ped waits at the kerb until its walk phase, then crosses. Behind an opt-in flag
(default off → every committed ped test byte-identical).

1. **New `CrosswalkSignals` provider** (`src/Sim.Pedestrians/Crossing/CrosswalkSignals.cs`): wraps a
   `CrosswalkSignalSchedule` + the **signalized** crossing polygons. API:
   `bool TryLocate(Vec2 p, out string crossingId)` (bbox + point-in-polygon over signalized crossings);
   `double NextWalkStart(string id, double tArrive, double speed)` (computes crossTime = crossLen/speed,
   where crossLen ≈ the crossing polygon's bbox diagonal, an over-estimate → ped waits conservatively);
   `int SignalizedCount`; `bool IsSignalized(id)`; `static FromNet(netPath, IEnumerable<BakedPolygon>)`.
   A baked `Crossing` polygon's `.Id` **is** the crossing edge id (e.g. `:d_0_0_c0`) that
   `CrossingTlReader.FindCrossingLink` keys on — confirmed.
2. **`PedDemandConfig.CrosswalkSignals`** (new `CrosswalkSignals?`, default null → inert).
3. **`PedDemand.BuildLivelyTimeline`** (`src/Sim.Pedestrians/Demand/PedDemand.cs` ~line 272–355): after the
   existing pause/segment build, if signals != null, run a post-pass `InsertCrosswalkWaits(segments, now,
   speed, signals, makeWalk)`:
   - Track clock `t = now`. Every `ActivitySegment` has `.Duration`; `WalkSegment.Duration =
     PathArcMotion.PathLength(Path)/Speed` — so the clock is exact (matches ActivityTimeline's own timing).
   - For each `WalkSegment`, walk its `Path` vertex-by-vertex; when a sub-segment ENTERS a signalized
     crossing (`TryLocate(b)!=null && TryLocate(a)==null`): emit the walk-to-kerb (advance `t` by its
     Duration), emit `PauseSegment(NextWalkStart(id, t, speed) - t, "wait")` (advance `t`), start a fresh
     walk at the kerb. Non-walk segments pass through (`t += seg.Duration`).
   - The post-pass draws NO rng → deterministic order preserved. Waiting peds render yellow (pause tag
     != Walk → `KindPedPaused`) — you can SEE them wait at the kerb, then cross.
   - Test (`tests/…/Crossing/…`): a ped path across a POC-0 signalized crossing gets a Pause before the
     crossing and its on-crossing interval lies within a walk window; an unsignalized crossing gets none;
     signals==null → byte-identical timeline.

## P2b-T3 plan
`BuildLiveCity` passes only **unsignalized** crossing polygons to `CrossingOccupancySource` (filter via
`signals.IsSignalized(poly.Id)`), because signalized crossings are now handled by ped compliance (peds only
on them during walk = car red). Point-disc gate kept for unsignalized; the full tunneling-proof stop-line
(needs `crossingEdges`→lane mapping) is a later refinement — note if cars still clip peds at unsignalized.

## P2b-T4 plan (verify)
Build a `CrosswalkSignals` from the net + crop crossing polys in `BuildLiveCity`; pass into the
`PedDemandConfig`. Verify: (1) peds visibly wait (yellow) at signalized kerbs; (2) sample cars vs
ped-occupied crossings → ~zero cars traverse a crossing while a ped is on it; (3) traffic still flows;
(4) two runs byte-identical + full `dotnet test` green + hash unmoved.

## Key files
- Scene: `src/Sim.Viz/SceneGen.cs` (`BuildLiveCity` ~1806, `BuildDenseCity` ~1578, helpers `ReadDrivable-
  Edges`, `CropNetwork`); `src/Sim.Viz/Program.cs` (`RunLiveCity`).
- Ped: `src/Sim.Pedestrians/Crossing/` (`CrosswalkSignalSchedule`, `CrossingOccupancySource`,
  `CrossingTlReader`, `CrossingGate`), `src/Sim.Pedestrians/Demand/PedDemand.cs`,
  `src/Sim.Pedestrians/Lod/{PedLodManager,ActivityTimeline,PathArcMotion}.cs`.
- Engine seam: `src/Sim.Core/Bridge/{WorldDisc(ICrowdFootprintSource),CompositeFootprintSource}.cs`;
  `Engine.CrowdSource` (Engine.cs:716), `CrowdLongitudinalConstraint` (~7790, gated/inert for goldens).
- Sidewalk fix (landed on main): `Lane.AllowsRoadVehicle` (Sim.Ingest) excludes ped lanes from
  neighbors; `LanePayload.Ped` + template.js "concrete" shading.

## Run / verify
```
dotnet run -c Release --project src/Sim.Viz --no-build -- --live-city <out>.html   # prints diagnostics + vehicle cost
dotnet test Traffic.sln -c Release                                                  # full gate
```
Determinism check: generate twice, `cmp -s`. Current gate: ParityTests 654/+3, Pedestrians 224 (incl. the
P2b-T1 5), DotRecast 2, Host 1 (recount after each add).

## Phase 3 (later): City3D combined cars+peds + semantic (enter buildings, dine at terraces, meet). Data is
in the box (buildings.json, entrances, venue table_cluster/service_door). Not started.
