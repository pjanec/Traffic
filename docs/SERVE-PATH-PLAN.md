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
- **GAP-3 — next** (multi-occupant parkingArea). Will reuse `scripts/audit_nocheat.py` (now committed)
  as the network-tier no-cheating check + a C# offline port of its logic as a `dotnet test` assertion
  on a synthetic served scenario (parkingArea id `pa_<edge>`, fringe births/deaths), since the real
  Geneva box is company-restricted — a synthetic box is built from the documented contract instead.

## 4. Recommendation

Proceed with **GAP-1 first** — it is the safe, enabling, near-zero-parity-risk step, and it lets us
point the SumoData tools at SumoSharp immediately. I'll wait for your answers to §3 (especially Q1/Q2
for GAP-1, and Q3 so the definitive test is real) before writing code.
