# SERVE-PATH-PLAN — making SumoSharp a drop-in `sumo` for the SumoData serve/replay path

**Status:** analysis + sequencing, pre-implementation. Source of truth for the *what*:
`docs/SUMOSHARP-SERVE-PATH-DROP-IN.md`. This document is the verified *how/when*, to be agreed with
the owner before any large implementation begins (CLAUDE.md "design-first").

**Baseline at this checkout** (`claude/sumosharp-drop-in-binary-vq7u9p`, `dotnet test`):
`Sim.ParityTests` 589 passed / 3 skipped (pre-existing sublane/multilane skips), `Sim.Pedestrians`
72 passed, `Nav.DotRecast` 2 passed. **Green.** Every finding below is cited at this checkout.

---

## 0. Verification of the three gaps (file:line evidence)

### GAP-1 — `sumo`-compatible CLI shim — CONFIRMED OPEN
- `src/Sim.Run/Program.cs:45` takes a **positional scenario directory** (`args[0]`), not `-c <cfg>`.
- Accepted flags: `--steps` (`:65`), `--fcd-out` (`:68`, note **`-out` ≠ `--fcd-output`**), `--warmup`
  (`:71`), `--summary-output` (`:74`), `--statistic-output` (`:77`), `--parity`/`--coordinated-lc`
  (`:80`,`:83`). **No** `-c`, `--begin`, `--end`, `--tripinfo-output`, `--no-step-log`.
- Unknown flags **abort**: `default:` → `error: unrecognized argument` + `return 2` (`:86-88`). The doc
  requires unknown flags be *tolerated* (warn, don't abort).
- Sim end is derived from the cfg, not a flag: `steps = round((End - Begin)/StepLength)` (`:101`).
- **The engine capability already exists** — `engine.LoadScenario(cfg)` with `<input>` multi-file
  resolution (`:111`), `FcdWriterObserver`/`SummaryWriterObserver`/`StatisticWriter` all wired
  (`:135-149`). So GAP-1 is **argv parsing + wiring + a discoverable `sumo` entrypoint**, no engine
  change.

### GAP-2 — `--tripinfo-output` with `arrivalLane` — CONFIRMED OPEN
- `src/Sim.Harness/TripInfoRecord.cs:11-15` carries only `Id, Depart, Duration, ArrivalSpeed` — **no**
  `ArrivalLane / arrivalPos / waitingTime / timeLoss / routeLength`.
- A tripinfo *writer* exists **only** in the benchmark tool `src/Sim.BenchCity/Program.cs:596-619`
  (`WriteTripInfo`), and it is **reconstructed post-hoc from FCD trajectory points**
  (`Program.cs:251-296`) with an explicit heuristic `arrivalCutoff` (`:263`) because — its own comment
  — it "cannot tell [arrival] apart … without a new Sim.Core arrival event". **There is no engine-CLI
  tripinfo writer, and no real arrival record.**
- **The real arrival seam exists and is clean:** route-end arrival is `_commandBuffer.Destroy(v)` at
  `src/Sim.Core/Engine.cs:8345`. At that point the vehicle still holds `LaneId` (→ `arrivalLane`),
  `Kinematics.Pos` (→ `arrivalPos`), `Kinematics` speed (→ `arrivalSpeed`), and `WaitingTime`
  (`VehicleRuntime.cs:235`); `Def.Depart` gives depart, sim time gives arrival. Arrival is **already
  deferred through the CommandBuffer**, so capturing a record here is thread-safe by construction.
- So GAP-2 = (a) capture a real per-vehicle arrival record at `:8345` (and the despawn/fringe path,
  `:2259`, if fringe exits must appear); (b) compute `routeLength` and `timeLoss` to **SUMO's
  `MSDevice_Tripinfo` formula** (the parity-sensitive part); (c) a new writer + `--tripinfo-output`
  wiring on the CLI.

### GAP-3 — multi-occupant `parkingArea` — CONFIRMED OPEN (degenerate today)
- `src/Sim.Ingest/ParkingArea.cs:24-35` `Lot0Position()` returns **one** position (lot-0 only):
  `startPos + (endPos-startPos)/roadsideCapacity`. There is **no per-occupant distinct slot**.
- `Engine.ResolveParkingAreaStop` (`src/Sim.Core/Engine.cs:1256-1277`) rewrites **every**
  `<stop parkingArea=X>` to a plain **on-lane** `StopRuntime` (`StopRuntime.cs`) at `EndPos =
  pa.Lot0Position()` — i.e. **all occupants collapse onto the same lot-0 spot**, and a parked car sits
  **on the running lane** (it is an ordinary longitudinal stop, so a follower is **blocked**, not
  passing).
- `departPos="stop"` is parsed (`DemandParser.cs:573-576` → `DepartPosValue.Stop`) and scenario
  `scenarios/48-parking-depart/` exercises it — but with exactly **one** vehicle (`rou.rou.xml`), the
  degenerate case the doc calls out.
- **Parity nuance (important):** scenario 48's *golden* already encodes SUMO's **off-lane** parking —
  in `golden.fcd.xml` the parked `veh0` has `y=-4.8` while parked vs lane-centre `y=-1.6`, snapping
  back to `-1.6` on pull-out. SumoSharp's on-lane approximation passes **only because
  `scenarios/48-parking-depart/tolerance.json` compares `["lane","pos","speed"]` and NOT `x/y`**. So
  today's parking parity does **not** actually verify the lateral off-lane placement. GAP-3's new
  scenario must be the thing that pins it down (via the *follower's* longitudinal trajectory, which is
  the real discriminator of "not blocked").
- So GAP-3 = real parking semantics: N≤`roadsideCapacity` residents with **distinct slots**, parked
  car **removed from the running lane** (not a leader/blocker for followers), `<stop parkingArea=…
  duration=…>` parks a moving car into a free slot, `departPos="stop"` inserts an already-parked car
  that pulls out into a gap, and an off-lane lateral position for FCD/replay.

### Minor / conditional (verified, likely deferrable)
- `departLane="free"/"random"` **throw** (`DemandParser.cs:535-554`, only numeric + `"best"`);
  `departPos` non-`"stop"` symbolic throws (`:557-579`). Add **iff** the real `.rou.xml` uses them.
- `<rerouter>` / `parkingAreaReroute` — not present; skip unless served scenarios use finite-dwell
  turnover (the doc says they don't: destinations park-and-stay, origins `departPos="stop"`).

### "What's DONE" claims (P0/P1/X1) — SPOT-CHECKED TRUE
`.sumocfg` multi-file input (`Engine.cs:111`, scenario `41-multifile-cfg`), `vTypeDistribution`
(`43-vtypedist`), symbolic departs (`42-symbolic-depart`), `--summary-output`/`--statistic-output`
(writers wired `Program.cs:135-149`, scenario `44-summary-output`), `device.rerouting`
(`45-reroute-congestion`, `RerouteEquipped`/`NextRerouteTime` `VehicleRuntime.cs:391-392`),
bounded `time-to-teleport` + counter (`47-teleport-jam`, `TeleportCount`/`TeleportCountJam` on Engine),
FCD (SUMO schema, `FcdWriterObserver`), X1 RealismMask (`MayPop`/`MayTeleport`, `Engine.cs:9671`). All
present and golden-backed. No regressions to the DONE surface are expected from GAP-1→3.

---

## 1. Sequencing, effort, risk

| Gap | Effort | Parity risk | Engine change? | Recommend |
|-----|--------|-------------|----------------|-----------|
| **GAP-1** CLI shim | Low | ~None (additive argv only) | No | **Do first** |
| **GAP-2** tripinfo+arrivalLane | Moderate | Moderate (`timeLoss`/`waitingTime`/`routeLength` must match `MSDevice_Tripinfo` exactly) | Small (arrival-record capture at an existing seam) | Second |
| **GAP-3** multi-occupant parking | High | High (off-lane placement, follower-not-blocked, pull-out gap accept) | Substantial (real parking manoeuvre semantics) | Third |

Order **GAP-1 → GAP-2 → GAP-3** matches the doc and is dependency-correct: GAP-1 unblocks the tools
calling SumoSharp at all; GAP-2 unblocks the audit; GAP-3 unblocks running the served scenarios.

Each gap lands with its own `scenarios/NN-*` parity case, golden regenerated from **pinned vanilla
SUMO 1.20.0**, wired into `dotnet test`:
- GAP-1: a multi-file-cfg CLI script test producing the four output files.
- GAP-2: a mix of through + parked-destination vehicles; assert `arrivalLane`/timing vs golden.
- GAP-3: N>1 sharing one `parkingArea` (some staying, some `departPos="stop"`) with through-traffic
  passing; assert the **follower's** trajectory matches (proves not-blocked).

---

## 2. Iron-law / determinism guardrails (apply to every gap)
- **Every prior golden byte-identical.** GAP-1 is additive argv (no engine touch). GAP-2/GAP-3 engine
  changes must be **gated** so no existing scenario's trajectory moves: the arrival-record capture is
  read-only w.r.t. movement; parking changes must be a no-op for any scenario without a multi-occupant
  parkingArea (fast-path preserved, as `ResolveParkingAreaStops` already does at `Engine.cs:1220`).
- **Determinism hash unchanged** — `tests/Sim.ParityTests/RungD1BenchmarkDeterminismTests.cs` and
  `RungD8ParallelDeterminismTests.cs` must stay green (same hash, same peak).
- **Thread-safety** — new per-vehicle mutations (parking manoeuvre state, arrival record) route through
  the existing `CommandBuffer` pattern; arrival already does (`Engine.cs:8345`).
- **Offline `dotnet test` never invokes SUMO** — SUMO only for golden regen (`scripts/regen-goldens.sh`)
  and investigation.

---

## 3. Design questions for the owner (need answers before GAP-1 / GAP-3)

1. **`sumo` entrypoint form (GAP-1).** How does the SumoData side expect to find SumoSharp? Options:
   (a) `dotnet publish` a self-contained binary named `sumo` on `PATH`; (b) a tiny `sumo` shell wrapper
   that execs `dotnet Sim.Run.dll …`; (c) SumoData sets a `SUMO_BINARY` env at the launcher. The doc
   mentions both PATH and `SUMO_BINARY`. **Recommendation: (a) a published `sumo` binary**, with (c)
   documented as the override. Which do you want as the committed deliverable?
2. **Where the shim lives (GAP-1).** Extend `Sim.Run` to accept **both** the legacy positional form
   (keeps `Sim.Viz`/benches working) **and** the `sumo -c` form, or add a dedicated thin `Sim.Sumo`
   CLI project that shares the same engine wiring? **Recommendation: a dedicated `Sim.Sumo` shim** so
   the viz-oriented `Sim.Run` flags don't tangle with the sumo contract. OK?
3. **Definitive integration test inputs.** The end-to-end acceptance needs a real
   `scenario.sumocfg` + `audit_nocheat.py` from the **SumoData** repo (not in scope here). Can you
   provide (i) one produced `scenario.sumocfg` (or the small Geneva box coords from the doc) and (ii)
   `audit_nocheat.py`? Without them I can build the three per-gap parity scenarios and a synthesized
   stand-in, but the *definitive* green needs the real artifacts.
4. **GAP-3 turnover semantics.** Confirm served scenarios are **park-and-stay sinks + `departPos="stop"`
   origins only** (no `parkingAreaReroute`, no finite-dwell overflow). If so I can skip rerouters.
5. **GAP-3 lateral-parity bar.** Should the new scenario's golden compare the off-lane `y` (tighten
   beyond scenario 48's `lane/pos/speed`-only tolerance), or is presence + the follower's longitudinal
   pass sufficient? This decides how faithfully the lateral slot offset must match `MSParkingArea`.
6. **Minor flags.** Do the real `.rou.xml` files use `departLane="free"/"random"` (currently throw)? If
   yes I'll fold the ~small parse addition into GAP-1.

---

## 3a. Progress log

- **GAP-1 — DONE** (owner-steered choices applied). New `src/Sim.Sumo` project builds the drop-in
  binary **`sumosharp`** (distinct name — never shadows vanilla `netconvert`/`randomTrips`/`duarouter`
  on PATH; SumoData points `SUMO_BINARY` at it). Core is a testable `SumoShim.Run(args, out, err)`;
  `Program.cs` is a one-line delegate. Flags: `-c/--configuration`, `-b/--begin`, `-e/--end` (a *time*
  → `round((end-begin)/step-length)` steps), `--fcd-output`, `--summary-output`, `--statistic-output`,
  `--tripinfo-output` (accepted; writer is GAP-2), `--no-step-log` (accepted+ignored); unknown flags
  tolerated (warn, not abort); `--flag=value` and `--flag value` both parse. Published self-contained
  single-file exe via `scripts/publish-sumosharp.sh` — **cold start + full 120-step multi-file run in
  0.19 s**, addressing the calibration per-invocation-cost requirement. Parity gate:
  `tests/Sim.ParityTests/RungHDgap1SumoCliTests.cs` (4 tests) drives the shim in-process over
  `41-multifile-cfg` and asserts the CLI-produced FCD matches the golden within tolerance, plus flag
  contract + `--end` run-length + error handling. **Full suite green: 593 parity (+4) / 3 skipped, 72
  pedestrian, 2 nav, 1 host; every prior golden byte-identical, determinism tests unchanged.**
  - *Note:* the owner-referenced served fixture (real `scenario.sumocfg` + `audit_nocheat.py` +
    README, Geneva box `-109298,-138987,-107798,-137487`) is **not yet present** in the repo/branch —
    needed for GAP-2's realistic scenario and the definitive end-to-end audit. GAP-1 used
    `41-multifile-cfg` (already committed) as its acceptance scenario, so it is unblocked.
- **GAP-2 — DONE** (Opus-reviewed hard, not taken on the implementor's word). Engine now captures a
  real per-vehicle arrival record at the single-threaded post-`Flush()` seam
  (`CommandBuffer.DestroyWithArrival` → `ArrivedThisFlush` → `Engine.CaptureCompletedTrips`, sorted by
  `EntityIndex` for RegionPlan determinism), exposed as `Engine.CompletedTrips`. Two **new never-reset**
  trip-total accumulators `TripWaitingTime`/`TripTimeLoss` (distinct from the existing consecutive
  `WaitingTime`, which is left byte-identical — the shared predicate was refactored into named locals
  with the *same* result; the SUMO-synchronous-stop correction reading `Intent.StopUpdate?.Reached` is
  scoped **only** to the two new fields). `routeLength = Σ(edges before arrival) − departPos +
  arrivalPos`, `arrivalPos` = arrival lane length, per `MSDevice_Tripinfo`. New `TripInfoWriter`
  (SUMO-schema, arrival order) + `TripInfoComparator` (exact `arrivalLane`, 0.05 abs on numerics,
  absent-vs-present → +∞ so an omitted field fails loud); `TripInfoRecord`/parser extended (nullable,
  back-compat). `--tripinfo-output` now really writes on the CLI. Acceptance:
  `scenarios/66-tripinfo-arrivallane` (leader with a brief `<stop>` + a follower that queues behind it
  → **nonzero** waitingTime 2.0 and timeLoss 5.993/8.333, both arrive), golden regenerated from **SUMO
  1.20.0** (provenance-verified), sentinel-gated (`.wants-tripinfo`) in `regen-goldens.sh` so no other
  golden changed. `RungHDgap2TripinfoTests` asserts both the engine path and the CLI path vs golden,
  and guards the null-vs-null vacuity trap. **Re-ran the full suite first-hand: 595 parity (+2) / 3
  skipped, 72 pedestrian, 2 nav, 1 host; determinism (D1/D8) green; `git status scenarios/` shows only
  the new `66-*`.**
- **GAP-3 — DONE** (Opus-reviewed hard). Real multi-occupant parkingArea semantics, all gated on a new
  `StopRuntime.IsParking` (true iff the stop came from `<stop parkingArea>`) / `VehicleRuntime.IsParked`
  so **plain `<stop lane>` stays on-lane blocking and every existing golden is byte-identical**:
  distinct per-occupant slots (`ParkingArea.LotPosition(i) = startPos + spaceDim·(i+1)`, two-pass
  load-time occupant→lot assignment — origins first, then moving cars — faithful to
  `MSParkingArea::computeLastFreePos` for the park-and-stay + `departPos="stop"` shapes, no reroute/
  turnover); a parked vehicle is lifted OFF the running lane (lateral bay offset via `LatOffset`, and
  **excluded from the leader search** in `LaneNeighborQuery.Refill`/`RefillRegion` — SUMO's
  `MSLane.cpp:2212 isParking()` — so a follower passes it, both serial and region-parallel paths);
  `departPos="stop"` origins start parked and pull out into a gap. A real diagnosed gap was fixed, not
  papered over: a parkingArea stop drops the generic `+NUMERICAL_EPS` braking term
  (`MSVehicle.cpp:2477-2481`), gated on `IsParking` — this reproduced SUMO's 204.999-vs-205.000 settle
  exactly (tolerance stayed strict at 0.001, **not loosened**). Acceptance:
  `scenarios/67-multi-parking` (parkStay0/pullOut0/driveInPark0 sharing one bay + through0 passing —
  golden from **SUMO 1.20.0**, asserts follower stays at cruise ≥13.88 while crossing the bays,
  co-occupants >5 m apart in distinct lots, parked cars off-lane) and `scenarios/68-serve-nocheat` (a
  synthetic fringe→interior→fringe box with `pa_eB`, through/origin/sink vehicles). `audit_nocheat.py`
  is ported to a **C# offline assertion** (`RungHDgap3NoCheatTests`) that runs the engine and checks 0
  birth/0 death/0 FCD-first-appearance violations from the engine's own tripinfo+FCD — no SUMO/sumolib,
  so it guards the no-cheating rule on every fresh VM. **Re-ran the full suite first-hand: 600 parity
  (+5) / 3 skipped, 72 pedestrian, 2 nav, 1 host; determinism (D1/D8) green; scenario 48 unchanged;
  `git status scenarios/` shows only new `67-*`/`68-*`.**

## Post-acceptance: Issue 1 (park-and-stay residency) — FIXED

The definitive acceptance run (spec §7) passed the no-cheating audit but found two aggregate-parity
divergences. Per the owner's split, **Issue 1 (serve-path) is fixed here; Issue 2 (junction
deadlock/jam-teleport, sumo-core) is delegated** to the core session (`docs/CORE-SESSION-ISSUE2-PROMPT.md`).

**Issue 1 — a park-and-stay sink (`<stop parkingArea duration=100000>`) on a 2-lane edge (parkingArea
on lane 0) wrongly "arrived"** because the car reached the parking edge on lane 1, never
strategically changed to lane 0 (so `StopLineConstraint` — which brakes only when `stop.LaneId ==
v.LaneId` — never engaged), drove off the final edge end, and was removed. **Fix (Opus-reviewed hard):**
- **Strategic LC toward the stop's lane** in `TryStrategicLaneChange` (Engine.cs) — when the front
  unreached stop is on the current edge with a different lane index, the strategic target is overridden
  to the stop's lane and bound by distance-to-stop (`stop.EndPos − pos`), porting SUMO's
  `updateBestLanes` stop-lane fold + `MSLCM_LC2013::driveToNextStop`. **Gated**: inert unless there is
  an unreached same-edge stop on a *different* lane, so every existing stop scenario (03/13/44/48, car
  already on the stop lane) and non-stop LC scenario falls through the byte-identical original path.
- **Residency guard** at the final-edge arrival site — a vehicle with an unreached `IsParking` stop is
  clamped at the lane end instead of arrived (defense-in-depth; rarely fires once the LC works).
- Once parked, GAP-3's `IsParked` keeps a `duration=100000` car resident off-lane, in FCD every step,
  never in tripinfo — exactly the sink semantics.

Acceptance: `scenarios/69-parkandstay-lc` (2-lane edge, parkingArea on lane 0, a car departing on lane
1 that must LC to lane 0 and stay resident), golden from **SUMO 1.20.0** (empty tripinfo — never
arrives). `RungHDgap3ParkAndStayResidencyTests` asserts it reaches lane 0 at the golden slot, is absent
from `CompletedTrips`, and is present in the trajectory every step to sim-end. Pre-fix (git-stash
verified) the car wrongly arrived at t=11 on lane e0_1; post-fix it LCs at t=2, parks at pos 47.499,
stays resident — matching vanilla. **Full suite re-run first-hand: 604 parity (+4) / 3 skipped, 72
pedestrian, 2 nav, 1 host; determinism (D1/D8) green; keep-right/overtake/strategic-turn + 03/13/44/48
byte-identical; only `scenarios/69-*` new.**

*Caveat (honest):* the residency guard is defense-in-depth — if a car genuinely cannot reach lane 0
(blocked), it clamps at the edge end on-lane rather than in the lot. The strategic LC handles the normal
case; the true validation is the SumoData re-run of the definitive acceptance test.

## Post-acceptance: Issue 2 (junction jam-teleports) — FIXED (it was the parking path all along)

The sumo-core session diagnosed (`docs/ISSUE2-JUNCTION-KEEPCLEAR-DESIGN.md`) that Issue 2 is **not** a
junction right-of-way defect: stripping the `<stop parkingArea>` from the sink cars drops jam-teleports
21→0. The root cause is the **same parking path as Issue 1** — GAP-3's `IsParked` off-lane exclusion was
**incomplete**: it removed a parked car only from the plan-phase neighbor query
(`LaneNeighborQuery`), but the `ActiveVehicles()`-based cross-junction/merge/foe/occupancy scans in
`Engine.cs` still treated it as an on-lane blocker, so through-traffic wedged the junction → deadlock →
jam-teleport. So the core session stood down and this work folded here.

**Fix (Opus-reviewed hard):** a parked (`IsParked`) vehicle is now skipped in every scan that can hold
back a *moving* vehicle by lane position (SUMO's `MSVehicleTransfer` off-lane semantics), while it stays
in the FCD and the `running` count. The load-bearing sites: `RearmostOnLaneAmongActive` (cross-junction
insertion leader), `FindRearmostOnLane` (`SameTargetMergeConstraint`'s merge-target leader — the actual
live-traffic wedge), `BuildFoeApproachIndex` (phantom-foe registration), the insertion-leader/`getFreeLane`
occupancy tallies, and the teleport-reinsertion gap scan — all **gated on `IsParked`** (byte-identical
without a parked car). FCD/export/`running` untouched. keepClear and `TryStrategicLaneChange` untouched.

Acceptance: `scenarios/70-parked-passable` (a priority cross with a roadside parkingArea on the major
exit lane, through-traffic that must pass + a genuine `time-to-teleport=60` so a block *would* teleport),
golden from **SUMO 1.20.0** with `teleports jam="0"`. `RungHDgap3ParkedPassableTests` asserts FCD parity,
through-speed >1 m/s at the parked car's position, residency, and jam-count == golden (0). **Verified
first-hand: full suite 608 parity (+4) / 3 skipped; junction 08/11/26/27/34/38/39/40 + parking
48/66/67/68/69 + determinism byte-identical; only `scenarios/70-*` new. Independently re-ran the synthetic
grid through `sumosharp`: jam-teleports 21 → 0 (vanilla 0), mean rel-speed now 0.73 (was 0.41).**

## Post-acceptance: Issue 1 residency tail (cross-edge strategic LC) — FIXED → definitive grid parity

The off-lane fix exposed the last gap: on the free-flow grid the sinks reach the short parking edge too
fast to lane-change, so they overshoot and wrongly complete (111/120). SUMO pre-positions them onto the
connecting lane on the *approach* edge via `updateBestLanes` propagating the stop-lane requirement
backward. **Fix (Opus-reviewed hard):** `NetworkModel.BuildTerminalLaneQ` folds a park-and-stay stop's
lane into the base case of the best-lanes backward pass (truncating every other last-edge lane to the
stop position, keeping the stop's lane full length — a port of `MSVehicle::updateBestLanes`
`MSVehicle.cpp:5913-5933`); the existing `BackwardPassEdge` recursion then steers every earlier route
edge's pool lane toward the connecting lane, so the *existing* pool-driven strategic LC pre-positions the
car across edges. `ComputeBestLanes`/`ResolveLaneSequence` gained an optional `stopOverride` (null-default
→ every prior call byte-identical). Caching hazard handled: a qualifying vehicle (`ParkStopFinalEdgeOverride`
— parking stop on its route's final edge) **bypasses** the route-keyed pool/bestLanes caches and resolves
per-vehicle, so a same-route non-parking vehicle is unaffected. Two additional bugs found + fixed (both
gated, byte-identical for non-parked): a parked vehicle now skips periodic reroute (`isStopped` guard,
`MSDevice_Routing.cpp:279`) and skips the keep-right/strategic/speed-gain LC decision (`IsParked` guard —
without it, a spurious speed-gain lane swap off the parkingArea lane un-held the stop and let the car
resume/arrive).

Acceptance: `scenarios/71-parkandstay-crossedge` (2-edge route through a junction, parkingArea near the
start of the 2nd edge so the car MUST pre-position on the approach edge), golden from **SUMO 1.20.0**,
empty tripinfo. **Verified first-hand — the definitive grid parity (Geneva's own done-condition):**

| synthetic grid `--end 1000` | vanilla | before | after |
|---|--:|--:|--:|
| tripinfo total | 1240 | 1351 | **1240** |
| sinks wrongly completed (of 120) | 0 | 111 | **0** |
| `ids(ss.ti) − ids(van.ti)` | — | 111 | **0** |
| jam-teleports | 0 | 21→0 | **0** |
| no-cheating audit | — | — | **PASS** |

Full suite 613 parity (+5) / 3 skipped; only `scenarios/71-*` new — every existing golden byte-identical
(LC 07/12/18/41/45/46/49, routing 15, junctions 08/11/26/38/39/40, parking 48/66/67/68/69/70, Issue-1
69, determinism D1/D8 all green). **Issue 1 and Issue 2 are both fully closed and match vanilla.**

## Issue 2 (junction teleports) — RESOLVED, and it was NEVER a right-of-way bug

Chasing the teleport count gap (SumoSharp 75 vs vanilla 3 on `scenarios/_repro/synthetic-junction`) led
to a decisive course-correction. The core session's on-junction-RoW hypothesis was investigated and
**disproved by deeper tracing**: every over-teleported car is not stuck in a right-of-way contest — it
is blocked by cars that are **permanently frozen** by a bug in *this session's own Issue-1 residency
guard*. Chain: a `departPos="stop"` + `departSpeed="max"` origin was inserted **moving** (at lane max
speed), immediately lane-changed off its stop lane before `ProcessNextStop` (which needs speed ≤ halting
*on the stop lane*) could mark the stop `Reached`, leaving a permanent unreached "zombie" stop in its
queue. When the car later reached its real arrival edge, the Issue-1 residency guard misread the zombie
as a pending parking obligation and **froze the car forever at the lane end** (Engine.cs `if
(frontStopAtArrival is { Reached: false, IsParking: true }) { Pos = laneLength; Speed = 0; break; }`).
149 such frozen cars on the junction grid blocked junctions → cascaded into 75 jam/yield teleports.

**Fix (surgical, in the parkingArea path):** SUMO inserts a `departPos="stop"` vehicle **stopped**
(`MSLane::insertVehicle` STOP case, MSLane.cpp:692-698 — a car at its stop has speed 0), regardless of
`departSpeed`. `TryInsertOnLane` now forces the insertion speed to 0 for `departPos="stop"`, so
`ProcessNextStop` marks the origin stop `Reached` on step 1 exactly as vanilla does — no zombie.
**Byte-identical for every committed `departPos="stop"` golden (48/67/68 all use `departSpeed="0"`, so
the speed was already 0); only the `departSpeed!="0"` case changes.**

**Verified first-hand** — one line closed both the never-arrived gap AND the entire teleport count:

| synthetic-junction `--end 1000` | vanilla | before | after |
|---|--:|--:|--:|
| arrived (tripinfo) | 475 | 326 | **475** (exact) |
| teleports total / jam / yield | 3 / 0 / 3 | 75 / 31 / 44 | **0 / 0 / 0** |

`synthetic-parity` (Issue 1) unchanged: sinks wrongly completed 0, `ids(ss.ti)−ids(van.ti)=0`, teleports
0. Full suite **613 parity byte-identical** (no committed golden moved). The Part-1 classification stays
(correct jam/yield split for any genuine future case), but on these scenarios the count is now 0 because
the frozen blockers are gone. Credit: the on-junction-RoW subagent correctly **refused to build the
assigned (ineffective) fix** and traced the real root instead.

## Post-acceptance round 4 (SumoData reaccept4): routeLength-across-reroutes FIXED; one real-box residual

SumoData's 4th definitive re-run: **Issue 1 green on the real box + both synthetics**; Issue 2 cleared on
both synthetics (teleports → 0). Two items remained:

**(a) routeLength lost distance across a `device.rerouting` reroute — FIXED.** The internal-junction-lane
routeLength fix summed the vehicle's *current* lane-sequence pool; a reroute (`ReplaceRoute`) rebuilds
that pool for only the *remaining* route, so all pre-reroute distance was dropped (rerouted trips
reported 0.33–0.49× — SumoData's synthetic-parity ids 2/113/697). Replaced with a **running accumulator**
`VehicleRuntime.RouteDistanceTraveled` (SUMO's `MSDevice_Tripinfo::myRouteLength`): −departPos at
insertion, `+= left-lane length` at the one `ExecuteMoveVehicle` lane-crossing site, `+ arrivalPos` at
arrival — so it survives reroutes. Equals the old pool-sum exactly for non-rerouted trips ⇒ scenarios
66/72 byte-identical. Verified on synthetic-parity: ids 2/113/697 went 0.37/0.49/0.33× → ~1.0× vanilla
(id 113 exact). Regression golden `scenarios/73-reroute-routelength` (a `blocker` congests the short
path so the tracked vehicle's *periodic* reroute fires mid-trip at t=8, `replacedOnEdge="e_src2"`,
`rerouteNo=1`); `RungHDgap2RerouteRouteLengthTests` asserts routeLength=756.21 == golden and, as the
guard, `> 696.11 + 30` (the old pool-sum-only value — proving it fails pre-fix). Golden from SUMO 1.20.0.

**(b) waitingTime — NOT a bug (withdrawn).** For matched-duration ids waitingTime matches exactly;
id-108 differs only because its duration differs hugely (ss flowed through in 124.6 s vs vanilla's
601 s) — real faster-flow dynamics, not an accumulator error.

**Open: real-box Issue-2 residual — 36 teleports (18 jam + 18 yield) vs vanilla 2** (down from 105).
No committed synthetic reproduces it (the `departPos="stop"` fix took `synthetic-junction` to 0), so the
residual is a *different* real-net junction pattern with no witness. Needs a repro (a sharper synthetic
or a geometry-free real-net capture of the stalling junctions/vehicles) to fix faithfully, or ship as a
documented follow-up (audit passes, Issue 1 green, RealismMask hides off-camera pops). Owner decision
pending.

## Real-box teleport residual (synthetic-junction2 witness): partial fix 42→17, root-caused

Geneva's geometry-free witness `scenarios/_repro/synthetic-junction2` reproduces the residual real-box
teleports (vanilla 0 vs SumoSharp 42 = 13 jam + 29 yield). Root cause pinned (via per-constraint-arm
instrumentation): a **stale distance formula** in `JunctionYieldConstraint`'s `JunctionCycleHold`
(right-before-left tie-break hold) arm. For a **cont-turn** chain (a 2-stage turn split across two
internal lanes), the arm computed the stop-line distance as `approachLane.Length − pos` where
`approachLane` is the SHORT intermediate internal lane (~7.8 m) — going deeply negative (e.g. 7.80 − 16.92
= −9.14), so `StopSpeedFor` collapses to ≈0 and **permanently freezes** the vehicle regardless of real
gap → it hits `time-to-teleport=120` (the Y1 pattern). Fix: use `egoDistToEntry` (the pool-walk distance
C4-vii-a already established for the cautious-approach arm and SameTargetMerge) — mathematically identical
to the old formula for every ordinary (non-cont) link, so **byte-identical for every committed golden**;
it only changes a cont-chain link (rare, RBL-tie-break-gated). Result: synthetic-junction2 **42→17** (jam
13→3, yield 29→14); synthetic-parity + synthetic-junction stay at 0; all 27 named TL/junction goldens +
determinism byte-identical; full suite green.

**Residual 17 not yet closed:** the same stale formula exists at two other sites in the function
(`ExternalAgentOnFoeLane` arm, foe-loop approaching-branch); applying the same substitution there further
cut teleports BUT gridlocked two saturated-grid stress tests (`WillPassSaturationDiagTests`,
`RungHDp2g2CoordinatedLaneChangeTests`) — so it was correctly NOT forced. Those two sites need a separate,
carefully-verified fix (they feed the saturated-grid path, so the naive substitution over-shoots). Owner
decision: attempt that delicate 2-site pass, or ship with the residual (down from 105→~14 est. on the real
box, audit passes, Issue 1 green, RealismMask hides off-camera pops) as a documented follow-up.

## SHIP acceptance: NOT green — real-box PROGRESSIVE GRIDLOCK (the teleport count was hiding it)

SumoData's ship-acceptance run correctly rejected it, and the real signal is not the teleport count.
Hard gates hold — completes, no-cheating audit PASS, Issue 1 residency green, routeLength accumulator
confirmed fixed. But on the real box SumoSharp **progressively gridlocks**: same vehicles inserted
(identical `running` each step), yet `halting` climbs monotonically to **~89% of on-lane vehicles at
t=999 vs vanilla's ~11%** (fair mean rel-speed 0.545 vs 0.838). Only ~36 of ~393 stalled cars ever cross
`time-to-teleport=120`, so the teleport count (36, unchanged across the last two fix passes) massively
UNDERSTATES the failure. The `JunctionCycleHold` fix helped the synthetic teleport count (42→17) but did
**nothing** to the real-box gridlock.

**Correction to the "synthetic has diverged / no witness" conclusion:** the committed
`scenarios/_repro/synthetic-junction2` DOES reproduce the gridlock — on the **halting curve**, not the
teleport count. Measured ss halting 79/97/…/35 at t=300/500/…/999 vs vanilla 34/36/…/0 (≈35 of 36 running
halted at end, mean rel-speed ~0.23). So it remains a valid witness **if gated on halting, not teleports**.

**Root category:** a systematic **junction-throughput / gap-acceptance / yield-release deficit** —
SumoSharp halts vehicles vanilla keeps moving, and the per-decision lag accumulates into whole-box
gridlock. This is the P2-G "junction saturation" work (flagged long ago as investigate-then-fix), a
substantial CORE fix — NOT the "small residual on pathological topologies" earlier accepted. **Decision
(owner): this is a PRE-MERGE BLOCKER, not a follow-up.** A diagnose-first pass is underway to localize the
seed-stall mechanism on synthetic-junction2 (which junction/movement, and which of gap-acceptance /
yield-release / junction-blocking / internal-lane-speed), with the **halting curve** (ss halting → vanilla
halting) as the gate. Fix follows the diagnosis, gated hard against the saturation stress tests +
TL/junction goldens.

## All three gaps landed — definitive acceptance status

GAP-1→GAP-3 are complete and golden-verified against vanilla SUMO 1.20.0. The `sumosharp` binary now
accepts the full serve/replay CLI contract, emits `--tripinfo-output` with `arrivalLane`, and runs
multi-occupant-parkingArea served scenarios. The no-cheating rule is guarded offline (C# port) and
available at the network tier (`scripts/audit_nocheat.py`). The one piece that still needs the owner is
a **real** SumoData-produced `scenario.sumocfg` to run the *definitive* end-to-end audit against a live
vanilla-SUMO reference — the Geneva box is company-restricted, so `68-serve-nocheat` stands in as a
synthetic equivalent built from the documented contract.

## 4. Recommendation

Proceed with **GAP-1 first** — it is the safe, enabling, near-zero-parity-risk step, and it lets us
point the SumoData tools at SumoSharp immediately. I'll wait for your answers to §3 (especially Q1/Q2
for GAP-1, and Q3 so the definitive test is real) before writing code.
