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

**Design (owner Q1b — RNG-insensitive).**
1. Parse `<vTypeDistribution>` (in route-files or additional-files) into a distribution registry:
   id → [(vTypeId, probability)], normalised.
2. Resolution: when a vehicle's `type=` names a distribution, assign a member.
   - **Parity gate scenario uses a single-member or `probability`-degenerate distribution**
     (or an assignment rule we can match deterministically) so the golden's type-per-vehicle is
     reproducible **without** cloning SUMO's PRNG stream — this is the RNG-insensitive gate.
   - **Sampling correctness** (multi-member weighted draw) is validated in a **separate
     statistical** test (`parityMode:"statistical"`): over many vehicles the assigned-type
     histogram matches the declared probabilities within tolerance — not vehicle-for-vehicle.
   - Seed via per-entity hashed RNG (never `System.Random`), per DESIGN.md.

**Files touched:** new `VTypeDistribution` parse in ingest, demand model, `Engine`/type resolution.

**Success conditions (P0-B):**
- Unit test: parse a `<vTypeDistribution>` with weights → normalised registry.
- Parity scenario `scenarios/43-vtypedist` with a degenerate/deterministic distribution: golden
  type assignment + trajectory reproduced exactly.
- Statistical test: 500+ vehicles over a weighted distribution → member-type frequencies within
  statistical tolerance of the declared weights.
- `dotnet test` green.

---

## P0-D — engine writers for `--summary-output` + `--statistic-output`

**Mechanism.** `--summary-output` writes per-step `<step time= running= halting= stopped=
meanSpeed= meanSpeedRelative= .../>`; `--statistic-output` writes a `<statistics>` doc including
`<teleports total= .../>`. These are SumoData's calibration signals.

**Design.**
1. Engine writer (new observer in `Sim.Run` / a `SummaryWriterObserver`, mirroring
   `FcdWriterObserver`) emitting all five per-step attributes. Aggregates largely exist; add
   `halting` (speed < 0.1 m/s threshold, SUMO's `haltingSpeedThreshold`), `stopped`,
   `meanSpeedRelative` (mean of v/vLimit).
2. `--statistic-output` writer emitting `<teleports total=…>` (0 until P1-F exists; wired now so
   P1-F just increments the counter).
3. **Harness parity side (bigger than a writer):** extend `Sim.Harness/SummaryOutputParser.cs` +
   `SummaryStepRecord.cs` to read `halting/stopped/meanSpeedRelative`; add a new
   `StatisticOutputParser` for `<teleports total>`. Add a comparator so the parity test can diff
   engine summary vs golden summary.
4. `Sim.Run` CLI flags `--summary-output PATH` / `--statistic-output PATH`.

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
- [ ] P0-C1 symbolic departs (max/best, lane-stop) — specs + gated insertion resolution + scenario 42
- [ ] P0-C2 parkingArea subsystem (parkingArea-stop departPos) — parse + getLastFreePos + scenario NN
- [ ] P0-B vTypeDistribution — parse + deterministic parity 43 + statistical sampling test
- [ ] P0-D summary/statistic writers + harness parsers + comparator + scenario 44
