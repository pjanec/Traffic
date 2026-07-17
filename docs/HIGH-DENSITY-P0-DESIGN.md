# HIGH-DENSITY-P0-DESIGN.md — P0 plumbing (design + tasks)

Design + task-description (folded, per CLAUDE.md "small features may fold the first two together")
for the four P0 plumbing prerequisites. Reference for the WHAT/WHY: `docs/HIGH-DENSITY-FEATURES`
(`SUMOSHARP-HIGH-DENSITY-FEATURES.md` §2) and the verified findings in `docs/HIGH-DENSITY-PLAN.md`
§1. This doc is the HOW. Order: **P0-A → P0-C → P0-B → P0-D**.

## Invariants (all four tasks)
- **Additive only.** Existing `LoadScenario(net, rou, cfg)` and all 466 committed goldens must
  stay byte-identical green. New capability rides on new overloads / new optional config fields
  that default to today's behaviour.
- **Each task lands with its own parity scenario** (`scenarios/NN-name/`) whose golden is
  regenerated from vanilla SUMO 1.20.0 (`scripts/regen-goldens.sh`) and wired into
  `tests/Sim.ParityTests`, passing within `tolerance.json`.
- **RNG-insensitive parity** (owner Q1b): parity scenarios are deterministic; no PRNG-order port.

---

## P0-A — multi-file `.sumocfg <input>` (net-file / route-files / additional-files)

**Mechanism.** SUMO is driven by `sumo -c config.sumocfg`; the cfg's `<input>` names the inputs:
```xml
<input>
  <net-file value="net.net.xml"/>
  <route-files value="vtypes.rou.xml,demand.rou.xml"/>
  <additional-files value="extra.add.xml"/>
</input>
```
Paths are resolved **relative to the cfg's directory** (SUMO semantics). route-files and
additional-files are comma-lists (SUMO also allows spaces; accept both).

**Design.**
1. `ScenarioConfig` (src/Sim.Ingest/ScenarioConfig.cs): add optional fields
   `string? NetFile`, `IReadOnlyList<string> RouteFiles`, `IReadOnlyList<string> AdditionalFiles`
   (default null / empty — today's scenarios omit `<input>`, so behaviour is unchanged).
2. `ScenarioConfigParser`: read `root.Element("input")` and its `net-file`/`route-files`/
   `additional-files` `value=` attributes; split comma/space lists; trim.
3. `DemandParser`: add `Parse(IEnumerable<string> rouPaths)` that parses each file and **merges**
   into one `DemandModel` — union VTypes (later files override same-id? SUMO errors on dup vType
   id; match by erroring, or last-wins — pick SUMO's behaviour, verify), concat Routes, concat
   Vehicles. The existing single-path `Parse` delegates to the multi-file path with one element.
4. `Engine`: add SUMO-faithful `LoadScenario(string sumocfgPath)` overload that parses the cfg,
   resolves `<input>` paths against the cfg dir, and loads net + all route-files (+ additional-
   files; for P0-A the additional-file may be behaviourally idle, e.g. a defined-but-unused
   parkingArea — it only has to *load*). Keep the 3-arg overload.
5. `Sim.Run/Program.cs`: when the cfg has an `<input>` section, drive off it instead of the glob;
   otherwise keep the glob fallback (back-compat with existing scenario dirs).

**Files touched:** `ScenarioConfig.cs`, `ScenarioConfigParser.cs`, `DemandParser.cs`,
`Engine.cs` (LoadScenario overload), `Sim.Run/Program.cs`.

**Success conditions (P0-A):**
- New unit test: `ScenarioConfigParser` parses an `<input>` with a 2-file `route-files` and a
  1-file `additional-files` into the new fields (comma + space forms).
- New unit test: `DemandParser.Parse([vtypesFile, demandFile])` merges vTypes-from-one +
  vehicles-from-another into a `DemandModel` equivalent to the single-file version.
- New parity scenario `scenarios/41-multifile-cfg` (next free NN): a small free-flow run whose
  `config.sumocfg` splits the vType into `vtypes.rou.xml`, routes+vehicles into `demand.rou.xml`,
  and references an idle `extra.add.xml`. Golden from SUMO 1.20.0. `LoadScenario(cfgPath)`
  reproduces `golden.fcd.xml` within tolerance.
- `dotnet test` fully green (466 existing + new).

---

## P0-C — symbolic depart attributes (`departSpeed`, `departLane`, `departPos`)

Grounded against the vendored SUMO 1.20.0 source (`sumo/src/...`) and the actual SumoSharp
insertion path. **Split into P0-C1 (the three symbolic specs, lane-stop only) and P0-C2 (the
parkingArea subsystem)** — the investigation showed a parkingArea-based `departPos="stop"` is a
whole subsystem, not plumbing.

**SUMO semantics (verified, cite sumo/src):**
- `departSpeed="max"` (`SUMOVehicleParameter.h` `DepartSpeedDefinition::MAX`): `MSLane::getDepartSpeed`
  (`MSLane.cpp:588-590`) sets speed = `getVehicleMaxSpeed` = `MIN2(vType.maxSpeed, laneSpeed ×
  speedFactor)` with **`patchSpeed=true`**. The leader-safety clamp happens downstream in
  `isInsertionSuccess`/`checkFailure` (`MSLane.cpp:780-808`): with `patchSpeed=true` an unreachable
  requested speed is **clamped** (`speed = MIN2(nspeed, speed)`), never fails — only an actual
  `gap<0` overlap fails. (Differs from `"desired"`, which is `patchSpeed=false` → fails/retries.)
- `departLane="best"` (`DepartLaneDefinition::BEST_FREE`): `MSEdge::getDepartLane` (`MSEdge.cpp:628-654`)
  ranks lanes by **route-continuation length** (`LaneQ.length`, capped at `BEST_LANE_LOOKAHEAD`=3000),
  then breaks ties by **least `getBruttoOccupancy`** = unnormalized `Σ(length+minGap)` over the lane's
  vehicles (`MSEdge::getFreeLane`, `MSLane.cpp:441`). Length first, occupancy second.
- `departPos="stop"` (`DepartPosDefinition::STOP`): `MSLane::insertVehicle` (`MSLane.cpp:692-698`) —
  if the vehicle's first stop is on the insertion lane, `pos = MAX2(0, stop.getEndPos())`; else
  **silently falls back to BASE** (no error). `MSStop::getEndPos` (`MSStop.cpp:34-51`) resolves a
  `parkingArea=` stop through `MSParkingArea::getLastFreePos` (dynamic, occupancy-aware) — this is
  the subsystem deferred to P0-C2.

**Byte-identical-safety strategy (critical).** Every new behaviour is **gated on the spec kind**:
`Given` (a numeric literal — what every existing scenario uses) takes *exactly today's code path*;
only `Max`/`Best`/`Stop` take new branches. So all 474 goldens stay byte-identical by construction
(no need to prove the new safe-speed clamp degenerates to the old raw-gap test — the old path is
literally kept for `Given`).

**Spec types (mirror SUMO's enums; unsupported members throw a clear error for now):**
```csharp
enum DepartSpeedSpec { Given, Max }   enum DepartLaneSpec { Given, Best }   enum DepartPosSpec { Given, Stop }
```
Replace `VehicleDef.DepartSpeed/DepartPos` (double) and `DepartLaneIndex` (int) — and the same three
on `ProbabilisticFlow` — with `(Kind, Literal)` pairs (`Literal` meaningful only when `Kind==Given`).

**Consumers to update (verified list):** `DemandParser.cs` (symbolic-aware parse helpers replacing
the `double.Parse`/`int.Parse` crash; both `<vehicle>` and `<flow>` paths), demand-model types,
`Engine.cs` insertion (`InsertDepartingVehicles` ~2356 for lane resolution; `TryInsertOnLane` ~2456
for speed + pos), **and `EngineSnapshot.cs:103-105,237-239`** (serialize the discriminant+literal for
flow-generated `--save-state` round-tripping — was missing from the file list).

### P0-C1 — the three specs, lane-stop only

**Resolution points (Engine.cs), each gated on `Kind`:**
- `departLane=Best`: resolve in `InsertDepartingVehicles` **before** the lane-index scan and before
  `PrewarmInsertRouteCache`/`RailBidiTrackOccupied` (which key off a concrete int) — call
  `NetworkModel.ComputeBestLanes(route.Edges, route.Edges[0])` (reuse; it is a pure topology fn,
  callable pre-placement), keep the max-`Length` set (Δ cap 3000), tiebreak by a **new** occupancy
  scan: `Σ(VType.Length+VType.MinGap)` over `ActiveVehicles()` on each candidate lane.
- `departSpeed=Max`: in `TryInsertOnLane`, cap = `KraussModel.LaneVehicleMaxSpeed(lane.Speed,
  speedFactor, vType)`, then clamp to `KraussModel.MaximumSafeFollowSpeed` vs same-lane and
  cross-junction leaders, and **succeed with the clamped speed** (patchSpeed=true), failing only on
  real `gap<0`.
- `departPos=Stop`: in `TryInsertOnLane`, if `Stops.Count>0 && Stops[0].LaneId==lane.Id` →
  `insertPos = Max(0, Stops[0].EndPos)`; else fall back to base. **Lane `<stop>` only** — already
  representable in today's `StopDef`, no new parsing.

**Success conditions (P0-C1):**
- Unit tests: parser maps `"max"/"best"/"stop"` to the right spec kinds; a number still parses as
  `Given`+literal; an unsupported keyword (e.g. `"free"`) throws a clear `InvalidDataException`
  (not `FormatException`). `Given` scenarios unchanged.
- Parity scenario `scenarios/42-symbolic-depart`: a multi-lane net; vehicles inserted with
  `departSpeed="max" departLane="best"` (and at least one `departPos` given as a lane `<stop>` if
  it can be made deterministic), onto lanes with differing route-continuation + occupancy so `best`
  is exercised non-trivially. Golden from SUMO 1.20.0; SumoSharp matches insertion speed/lane/pos +
  trajectory within tolerance.
- Full `dotnet test` green (474 + new), all pre-existing goldens byte-identical.

### P0-C2 — parkingArea subsystem (follow-on; needed for no-cheating parked origins)

The product's parked-origin cars use `departPos="stop"` into a `parkingArea` (feature doc §0:
"off-road inside a parkingArea"). This needs: (1) parse `<parkingArea id lane startPos endPos
roadsideCapacity>` from additional-files into a new small model (today `Engine.LoadScenario(cfg)`
loads-and-discards additional-files); (2) allow `<stop parkingArea="..."/>` without `lane=` in
`DemandParser`; (3) a minimal port of `MSParkingArea::getLastFreePos` (`MSParkingArea.cpp:187-225`)
— scope the parity scenario to the **degenerate empty-area single-vehicle** case so `getLastFreePos`
collapses to "first lot's endPos" without the full reservation/egress machinery. Flag the general
multi-occupant parkingArea occupancy as its own later task.

**Success conditions (P0-C2):** parity scenario `scenarios/NN-parking-depart` — one car with
`departPos="stop"` into a `parkingArea` additional-file; golden from SUMO 1.20.0; SumoSharp places
it at the same position and reproduces the trajectory. Full suite green.

---

## P0-B — `<vTypeDistribution>` resolution (RNG-insensitive parity)

**Mechanism.** `<vTypeDistribution id="civ_vehicle" vTypes="car:0.7 van:0.3">` (or child
`<vType .../>` with `probability=`). A `<vehicle type="civ_vehicle">` gets a member vType sampled
by probability. SUMO samples from its RNG per vehicle.

**SUMO syntax (both forms must parse) — VERIFIED against `sumo/src/microsim/MSRouteHandler.cpp:248-286`:**
- Attribute form: `<vTypeDistribution id="civ" vTypes="carA carB" probabilities="0.5 0.5"/>` —
  **two separate space-separated attributes**: `vTypes` (member ids declared elsewhere) and an
  optional parallel `probabilities`. NOT colon-weight syntax (`carA:0.5` is read as one id and
  errors). If `probabilities` is omitted, each member uses its default probability (1.0). Weights
  are normalised by the total.
- Nested form: `<vTypeDistribution id="civ"> <vType id="carA" .../ probability="0.5"> ... </>` —
  members are inline vType definitions carrying a `probability=` attribute.
A `<vehicle type="civ">`/`<flow type="civ">` whose `type=` names a distribution id draws a member.

**Design (owner Q1b — RNG-insensitive).**
1. Parse `<vTypeDistribution>` (in route-files or additional-files) into a registry:
   id → normalised [(vTypeId, probability)]. Nested inline vTypes are also added to the vType set.
2. **Resolution timing.** Keep the distribution in the demand model; resolve a vehicle's concrete
   member vType **at vehicle-creation time in `Engine`** (where the per-entity seeded RNG lives),
   drawing from a **per-entity hashed RNG** (hash of entity id / vehicle id — never `System.Random`,
   never a shared stream), so the assignment is independent of thread/scheduling order (DESIGN.md).
3. **RNG-insensitive parity gate (the key trick).** The parity comparator compares only
   `(lane, pos, speed)` (per `tolerance.json` `comparedAttributes`), **not** the FCD `type=`
   attribute. So a distribution whose members are **behaviourally identical** (same accel/decel/tau/
   maxSpeed/length/minGap; different ids only) yields a golden trajectory that is **invariant to
   which member each vehicle draws** — SumoSharp matches the golden without reproducing SUMO's PRNG
   draw. The scenario also asserts (functionally) that every vehicle resolved to *a* declared member.
4. **Sampling correctness** is a separate statistical/functional test: a multi-member *weighted*
   distribution, 500+ draws via SumoSharp's own seeded RNG, assert the member-frequency histogram
   matches the declared weights within tolerance. (Not a SUMO golden comparison — different RNG.)

**Files touched:** new `VTypeDistribution` type + parse in `DemandParser`/`DemandModel`, distribution
registry on `DemandModel`, `Engine` type-resolution at vehicle creation (survey the existing
per-entity dawdle-RNG seeding to reuse its hashing).

**Success conditions (P0-B):**
- Unit tests: parse both the attribute (`vTypes="a b" probabilities="0.7 0.3"`) and nested (`<vType
  probability=>`) forms → normalised registry; omitted `probabilities` defaults each member to 1.
- Parity scenario `scenarios/43-vtypedist`: several vehicles with `type="<distId>"` over a
  **behaviourally-identical 2-member** distribution; golden from SUMO 1.20.0; SumoSharp reproduces
  `(lane,pos,speed)` within tolerance AND every vehicle's resolved type ∈ the members.
- Statistical test: 500+ vehicles over a weighted (e.g. 0.7/0.3) distribution → frequencies within
  statistical tolerance of the weights.
- `dotnet test` green; all prior goldens byte-identical (a plain `type=` that is NOT a distribution
  id keeps the exact direct-lookup path).

---

## P0-D — engine writers for `--summary-output` + `--statistic-output`

**Mechanism (VERIFIED against `sumo/src/microsim/MSNet.cpp:607-647` + `MSVehicleControl.cpp:516-543`
+ `StdDefs.h:58`).** `--summary-output` writes one `<step>` per sim step with (P0-D subset):
- `time` — sim time of the step.
- `running` — `getRunningVehicleNo()` (inserted, not yet ended).
- `halting` — count of **on-road** vehicles with `speed < 0.1` (`SUMO_const_haltingSpeed`).
- `stopped` — `getStoppedVehiclesCount()` (vehicles currently at a `<stop>`).
- `meanSpeed` — mean `speed` over **on-road AND non-stopped** vehicles; **`-1` if that count is 0**.
- `meanSpeedRelative` — mean of `speed / edge.speedLimit` over the same set; **`-1` if count 0**.
  (Relative to the **edge speed limit**, NOT vType maxSpeed — verified: scenario 02 gives
  5.0/13.89 = 0.36.)
`--statistic-output` writes `<statistics>` with (P0-D subset) `<teleports total= jam= yield=
wrongLane= />` — all 0 until P1-F.

**Design.**
1. Engine per-step aggregates computed from the **same per-step state the FCD export uses** (so a
   scenario whose FCD already matches golden yields matching summary values). A `SummaryWriter`
   emitting `<step time running halting stopped meanSpeed meanSpeedRelative/>` each step aligned to
   the FCD emission point. Reuse/extend whatever `Sim.BenchCity`'s partial `WriteSummary` computes
   for running/meanSpeed; add halting/stopped/meanSpeedRelative with the exact definitions above.
   **Emit `-1` for meanSpeed/meanSpeedRelative when the on-road-non-stopped count is 0.**
2. `StatisticWriter` emitting `<statistics><teleports total="0" jam="0" yield="0" wrongLane="0"/>…`
   — teleport counter surfaced now (0), so P1-F only has to increment it. (A teleport counter field
   on Engine, 0 in phase 1.)
3. **Harness parity side:** extend `Sim.Harness/SummaryOutputParser.cs` + `SummaryStepRecord.cs` to
   read `halting/stopped/meanSpeedRelative` (currently only `time/running/arrived/meanSpeed`); add a
   new `StatisticOutputParser` for `<teleports total>`. Add a summary comparator (per-attribute
   numeric tolerance over the step series, mirroring the FCD comparator's shape).
4. `Sim.Run` CLI flags `--summary-output PATH` / `--statistic-output PATH` (additive; absent = no
   writer, unchanged behaviour).

**Files touched:** `Sim.Run/Program.cs`, new `SummaryWriterObserver`/`StatisticWriterObserver`
(likely in Sim.Core or Sim.Run export seam), `Sim.Harness/SummaryOutputParser.cs`,
`SummaryStepRecord.cs`, new `StatisticOutputParser.cs`, aggregate comparator.

**Success conditions (P0-D):**
- Unit tests: extended `SummaryOutputParser` reads all five attributes; `StatisticOutputParser`
  reads `<teleports total>`.
- Parity scenario (reuse `41`/`42` or a dedicated `44-summary-output`): engine
  `--summary-output`/`--statistic-output` are numerically within tolerance of golden's
  `summary.xml`/`statistic.xml` (teleports total = 0 pre-P1-F).
- `dotnet test` green.

---

## Tracker (P0)
- [x] P0-A multi-file cfg — parser + DemandParser merge + LoadScenario(cfg) + Sim.Run + scenario 41 ✅ (474 green)
- [x] P0-C1 symbolic departs (max/best, lane-stop) — specs + gated insertion resolution + scenario 42 ✅ (497 green)
- [ ] P0-C2 parkingArea subsystem (parkingArea-stop departPos) — parse + getLastFreePos + scenario NN
- [x] P0-B vTypeDistribution — parse (attribute vTypes=/probabilities=, colon shorthand, nested <vType probability=>) + per-entity salted-RNG resolution at BuildRuntime + scenario 43 ✅ (507 passed + 3 pre-existing skips, 510 total)
- [x] P0-D summary/statistic writers + harness parsers + comparator + scenario 44 ✅ (521 passed + 3 pre-existing skips, 524 total)

### P0-C2 — grounded design (verified against SUMO 1.20.0 source + empirical run)

**Scope confirmed SMALL** (no off-road/parked state needed for the single-car parked-origin case):
a `departPos="stop"` car into a parkingArea starts AT the lot position ON-lane, parks for the stop
duration, then drives off — mechanically identical to a lane `<stop>` (P0-C1), only the position is
resolved from the parkingArea. Off-road/lateral state matters only for a FOLLOWER past a parked car
and the `y` coord (not compared) — deferred to a later rung.

**Lot-0 position (empty area, straight lane)** = `startPos + (endPos - startPos) / roadsideCapacity`
(`MSParkingArea.cpp:72,106` + `getLastFreePos:196-204`). Verified: `195 + (210-195)/5 = 198.0`
(scenarios/48 golden: parked at 198.0 from t=0..duration, then Krauss accel). Constraints:
`roadsideCapacity >= 1` (capacity 0 hits the `begPos - minGap` branch), straight/unscaled lane
(so `interpolateLanePosToGeometryPos` is identity), `gModelParkingManoeuver` off (default).

**Minimal pieces:** (1) parse `<parkingArea id lane startPos endPos roadsideCapacity>` from
additional-files (defaults per `NLTriggerBuilder.cpp:566-569`) into a registry, at the
`Engine.LoadScenario(cfg)` additional-file hook (`Engine.cs:~1099`, currently loads-and-discards);
(2) allow `<stop parkingArea="..."/>` with no `lane=` in `DemandParser` (`:215-220`); resolve it —
given the parkingArea registry — to `StopDef{ LaneId = pa.lane, StartPos = pa.startPos, EndPos =
lot0Pos, Duration }`; (3) `departPos="stop"` (`Engine.cs:2820-2827`) then works UNCHANGED (reads
`Stops[0].EndPos`); (4) `ProcessNextStop` reused unchanged (stop reached at insertion since
`pa.startPos <= insertPos`). Wiring: `LoadScenario(cfg)` parses parkingAreas before/with demand and
resolves parkingArea-stops (registry available to the resolution).

**Acceptance:** `scenarios/48-parking-depart` (built): straight net + `pa0` (cap 5) + one
`departPos="stop"` car with `<stop parkingArea="pa0" duration="10">`. Golden from SUMO 1.20.0
(parked at 198.0 t0..10, drives off). SumoSharp reproduces `(lane,pos,speed)` within tolerance.
Unit test: parse a `<parkingArea>` + resolve lot-0 position formula.

