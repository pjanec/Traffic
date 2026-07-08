# VIZ_BENCH_TASKS.md — Work queue for `Sim.Viz` + the scaled-city benchmark

A separate task tracker for the two utility tracks described in `VIZ_SPEC.md` and
`BENCHMARK_SPEC.md`. Same discipline as `TASKS.md`: each task is a **self-contained
briefing** naming its inputs (source files, target files, command, numeric done-condition)
so a fresh subagent can pick it up cold. Read `CLAUDE.md` (rules), `DESIGN.md`
(architecture), `HANDOFF.md` (current state), and the two spec files before starting.

**These two tracks are OFF the `dotnet test` parity path.** Neither is a parity gate; both
are utilities (`VIZ_SPEC.md` and `BENCHMARK_SPEC.md` both say so). They are additive and
low-risk to the parity iron law — but they must still honor the committed-vs-ephemeral rule,
the plan/execute invariant, and the "extend through the D-seams, don't route around them"
rule from `HANDOFF.md`.

Legend: **[net]** = needs a network-enabled VM + SUMO (netgenerate/randomTrips/duarouter or
golden regen); everything else runs offline.

---

## Ground-truth reconciliation (READ FIRST — the specs are stale about project maturity)

Both spec docs were written assuming a much earlier project. Verified against the checkout
at `main` (`498d3e3`), the following are **already true** and must NOT be re-derived from the
specs' hedged "if that isn't done yet" language:

- **The engine already emits real global `X`/`Y`/`Angle`.** `Engine.EmitTrajectory`
  (`src/Sim.Core/Engine.cs`) calls `LaneGeometry.PositionAtOffset(lane.Shape, pos)`
  (`src/Sim.Ingest/LaneGeometry.cs`) per vehicle per step. `Angle` is already SUMO's
  **naviDegree** convention (0°=north, increasing clockwise) — identical to FCD. `(x,y)` is
  the lane arc-length position (front-reference, matching SUMO's FCD). So the viz can render
  **engine output** faithfully, not only SUMO goldens; the box back-offset logic is identical
  for both.
- **`Sim.Ingest` already parses almost everything the viz needs from `.net.xml`/`.rou.xml`:**
  - `NetworkModel` (`src/Sim.Ingest/NetworkModel.cs`): `Lane(Shape, Width)` (width defaults to
    `SUMO_const_laneWidth = 3.2`), `Edge`, `Connection(from, fromLane, to, toLane, via, tl,
    linkIndex)`, `TlLogic(Id, Offset, Phases)` + `TlPhase(Duration, State)`, `Junction`.
  - `DemandModel`/`VType` (`src/Sim.Ingest/DemandModel.cs`): `VClass`, `Length`, `Width`
    (nullable overrides) + the vClass default table in `VTypeDefaults.cs` (length/width/etc.
    per vClass). The FCD→type→dimensions join is a lookup over these.
  - **Reuse `NetworkParser.Parse` / `DemandParser.Parse` from the viz exporter.** These are
    `Sim.Ingest` public parsers, NOT engine internal component structs — reusing them is what
    the viz spec's "reuse `Sim.Ingest` where it parses net/rou" means, and it does NOT violate
    the "never reach into engine structs" rule.
- **The engine's capability gates are already cleared.** Traffic lights (rung 10), priority
  junctions (9b), lane-changing (A2 speed-gain + keep-right), Dijkstra routing (B2), reroute
  (B3) are all done and green. As of `main` (`2178654`) the C-track has also landed — C1
  (sigma>0), **C2 (strategic LC + lane continuity)**, C3, C7, C8, C11 (IDM/ACC/CACC/IDMM) —
  and B5 (external agents). `dotnet test` = **104 green**. The benchmark's engine-capability
  gate is fully cleared for all rungs (see Phase 2).

Two **real gaps** the tasks below close:

1. **No FCD *writer* exists.** The engine produces an in-memory `TrajectorySet`;
   `Sim.Harness/FcdParser.cs` only *reads* FCD (SUMO goldens). "Wire the engine to emit FCD"
   is genuine new work — Phase 0.
2. **`Junction` has no `Shape` polygon.** `NetworkModel.Junction(Id, Type, IntLanes, Links,
   Requests, Conflicts)` omits the junction `shape` the viz fills as a solid area.
   `NetworkParser.ParseJunction` reads the `<junction>` element but drops `shape`. Adding it
   is inert for the engine (engine never uses it) — done in VB-1.

**Do NOT confuse `Sim.Bench` with the benchmark spec.** `src/Sim.Bench` already exists — it is
the **Group-D perf/determinism harness** on a hand-built dense highway (`scenarios/_bench/
highway-dense`), measuring steps/sec, alloc/veh-step, and the determinism hash
`909605E965BFFE59`. `BENCHMARK_SPEC.md` wants a **new** netgenerate-based synthetic-city
scaling ladder with statistical SUMO comparison. Reuse `Sim.Bench`'s *measurement code*; do
not overload that scenario or that project's purpose.

---

## Build order & dependency graph

```
Phase 0 (offline) ──> Phase 1 Sim.Viz (offline) ──────────────┐
   FCD writer seam        VB-1 ─ VB-2 ─ VB-3 ─ VB-4            │
        │                                                     ▼
        └────────────────────────> Phase 2 Benchmark [net]  (feeds viz)
                                       VB-5 ─ VB-6 ─ VB-7 ─ VB-8
```

- **Critical path is Phase 0 → Phase 1**, 100% offline (no SUMO, no network). Highest
  immediate payoff; delegatable to a separate Sonnet session per `HANDOFF.md`.
- **Phase 2 is `[net]`** (netgenerate/randomTrips/duarouter + SUMO aggregate goldens) and
  reuses the Phase 0 FCD writer + `Sim.Bench` measurement code.

---

# PHASE 0 — shared FCD-writer seam (offline)

## VB-0 — `FcdWriterObserver` + a run-and-dump entry point — **DONE**

**Status: DONE** (`Sim.Harness/FcdWriterObserver.cs`, `src/Sim.Run/`,
`VehicleExportSnapshot.VehicleType`, `tests/Sim.ParityTests/Vb0FcdWriterTests.cs`). `dotnet
test` = **105 green**; `Sim.Bench` hash **byte-identical** (`42F875C2662DB78E`, single ==
parallel, unchanged with/without the edit). The writer is lossless (round-trippable "R"
formatting) and the emitted engine FCD lands within `01`'s tolerance vs the SUMO golden.

**Goal.** Let any engine run write a SUMO-schema FCD file, so both `Sim.Viz` and the benchmark
can consume engine trajectories the same way they consume `golden.fcd.xml`.

**Why the D9 seam.** `HANDOFF.md` mandates extending the engine *through* the D-seams. The D9
export seam (`src/Sim.Core/ISimExportObserver.cs` + `VehicleExportSnapshot.cs`) is exactly a
per-frame, per-vehicle export hook — the FCD writer is its first real consumer, not a reason
to touch `EmitTrajectory`.

**Implement in `Sim.Harness`** (it already owns FCD parsing — keep read+write together):
- `FcdWriterObserver : ISimExportObserver` — on `OnFrameBegin(time)` open a `<timestep
  time="...">`; on `OnVehicleExported(in snapshot)` write `<vehicle id x y angle speed pos
  lane type/>` from the snapshot; on `OnFrameEnd` close the timestep. Root `<fcd-export>`.
  Match SUMO's element/attribute names and `--precision 6` formatting exactly (see
  `scripts/regen-goldens.sh` for the golden's schema; the engine emits full double precision —
  do NOT round).
  - `VehicleExportSnapshot` currently lacks the vehicle **type** string. Add `VehicleType`
    (the vType id) to the snapshot (populated in `EmitTrajectory` from the vehicle def) so the
    FCD `type=` attribute is real. This is inert for existing observers (the default list is
    empty) and byte-identical for the `TrajectorySet` path.
- Register it via the existing `Engine.AddExportObserver(...)` — no change to `EmitTrajectory`.

**Entry point.** A small runner so a scenario dir → `engine.fcd.xml`. Cleanest: a new
`src/Sim.Run` console project (`dotnet run --project src/Sim.Run -- <scenarioDir> [--steps N]
[--fcd-out engine.fcd.xml]`) that loads net/rou/cfg via `Engine.LoadScenario`, attaches
`FcdWriterObserver`, runs, and writes the file. (Alternatively extend `Sim.Bench` with a
`--fcd-out` flag; a dedicated `Sim.Run` keeps Bench's perf purpose clean — prefer `Sim.Run`.)

**Done-condition.** `dotnet run --project src/Sim.Run -- scenarios/01-single-free-flow
--fcd-out /tmp/engine.fcd.xml` writes a well-formed FCD file; feeding it and
`scenarios/01-single-free-flow/golden.fcd.xml` through `Sim.Harness.TrajectoryComparator`
reports **within `tolerance.json`** (a free parity re-confirmation via the new path). Commit
`Sim.Run` + `FcdWriterObserver` + the snapshot `VehicleType` addition. `dotnet test` stays
green (still 64; no parity scenario changed).

**Validation bar.** Byte-identical for all existing tests (empty observer list default). Gate
through parity-reviewer only if `EmitTrajectory` or the snapshot changes touch the hot path.

---

# PHASE 1 — `Sim.Viz` offline replay tool — **DONE (VB-1…VB-4)**

**Status: DONE.** `src/Sim.Viz` (exporter `Program.cs` + committed `template.html`/`template.js`),
`Junction.Shape` added to `Sim.Ingest` (inert), and committed `replay.html` for
`01-single-free-flow`, `09-traffic-light`, `11-priority-junction`. `dotnet test` = **105
green**; `Sim.Bench` hash **byte-identical** (`42F875C2662DB78E`); every `replay.html` is
self-contained (no external refs). Verified with headless Chromium/Playwright (no JS errors,
canvas draws, TLS head red/green at the stop line, junction fill distinct, play/pause/restart/
speed/pan/keyboard all functional). Run: `dotnet run --project src/Sim.Viz -- <scenarioDir>`
(defaults FCD to `golden.fcd.xml`; `--fcd <path>` for an engine `engine.fcd.xml` from VB-0).

Front-end is a **committed static template** (`src/Sim.Viz/template.html` + `template.js`);
the C# exporter injects one `<script>const REPLAY_DATA = {...}</script>` and writes
`scenarios/<name>/replay.html`. Canvas 2D, vanilla JS, no external libs, self-contained.
One command: `dotnet run --project src/Sim.Viz -- <scenarioDir>`.

## VB-1 — Exporter: scenario dir → `REPLAY_DATA` JSON → `replay.html`

**Goal.** The `Sim.Viz` C# console tool that reads net + fcd + rou, builds the compact JSON
payload, injects it into the template, and writes `replay.html`.

**Inputs per scenario dir** (all committed): `*.net.xml`, an FCD file (prefer
`golden.fcd.xml`; accept `--fcd <path>` to point at an engine `engine.fcd.xml` from VB-0),
`*.rou.xml`.

**Reuse** `Sim.Ingest.NetworkParser.Parse` and `Sim.Ingest.DemandParser.Parse`, and
`Sim.Harness.FcdParser` for the FCD. **New parsing gap to close:** add `Shape` (the polygon
point list) to `NetworkModel.Junction` and populate it in `NetworkParser.ParseJunction` from
the `<junction shape="...">` attribute (inert for the engine). Lane markings need adjacent
lanes of the same edge — `Edge` already lists its lanes with index.

**Payload shape (pre-flatten to arrays; keep it compact):**
- `network`: per lane `{shape:[x,y,...], width}`; per junction `{shape:[x,y,...]}`; per edge
  its lane ids+indices (for markings); `tls`: the `<tlLogic>` programs (`offset`, phases as
  `{duration, state}`) and the controlled `<connection>` list (`tl`, `linkIndex`, from-lane id
  + the stop-line end point of that lane, computed from lane shape).
- `vehicles`: per FCD vehicle id → `{type, vClass, length, width, color}` (join FCD `type` →
  `VType`/`VTypeDefaults` for length/width/vClass; if the FCD row already carries length/width,
  prefer it).
- `trajectory`: per timestep an array of `{id, x, y, angle, speed}` (parallel arrays are fine
  and smaller). Record `simStart`, `simEnd`, `stepSize`.
- `bbox`: network bounding box for fit-to-view (do NOT bake offsets into geometry — keep world
  coords, camera transforms them).

**Template injection.** Read `template.html`, replace a `/*REPLAY_DATA*/` placeholder (or a
marked `<script>` block) with the serialized payload, write `replay.html` next to the inputs.
Do NOT string-concatenate the whole HTML in C# — fill the template.

**Done-condition.** `dotnet run --project src/Sim.Viz -- scenarios/01-single-free-flow`
writes `scenarios/01-single-free-flow/replay.html` containing a `REPLAY_DATA` block with
non-empty network + trajectory. (Rendering correctness is VB-2..VB-4.)

## VB-2 — Front-end spine: world/camera + real-time clock (build this correctly first)

**Goal.** The camera transform and the wall-clock replay engine — everything else hangs off
these. No rendering of vehicles yet beyond a placeholder dot to prove the clock.

- **Camera:** one screen↔world transform (`scale`, `translateX/Y`). On load, fit-to-view from
  `bbox` (centered, margin). **Flip Y** (canvas Y is screen-down, SUMO Y is up). Zoom on wheel
  + two-finger pinch, **anchored at cursor / pinch midpoint** (world point under cursor stays
  fixed). Pan on left-drag + one-finger drag. All zoom/pan are pure matrix ops.
- **Clock:** `requestAnimationFrame` loop decoupled from FCD step size. Maintain `simTime`;
  each frame when playing `simTime += realDelta * speedMultiplier`. Map `simTime` to the
  bracketing FCD steps and **interpolate** position + angle (shortest-arc for angle). A vehicle
  present at step k but not k+1 stops drawing. Play/Pause freezes accumulation; Restart sets
  `simTime = simStart`; speed multiplier set {0.25,0.5,1,2,4,8}, default 1×.

**Done-condition.** Open `replay.html` for scenario 01: a placeholder marker moves smoothly
along the road in real time start→end; wheel/drag zoom-pan works, cursor-anchored.

## VB-3 — Rendering layers (draw back-to-front)

**Goal.** Width-accurate network + native-look TLS + oriented true-size vehicle boxes.

1. **Junctions** — fill each junction `shape` polygon (dark gray).
2. **Lanes** — each lane centerline `shape` as a filled band of `width` world-units (thick
   stroke = lane width, or offset polyline both sides). Mid-gray.
3. **Lane markings** — dashed white between adjacent same-edge lanes.
4. **Traffic lights (SUMO-native look)** — replay `<tlLogic>` directly: from `simTime`, program
   `offset`, and cumulative phase `duration`s, find the active phase's `state` string. For each
   controlled `<connection>` draw a small signal head at the **stop-line end of the `from`
   lane**, colored by `state[linkIndex]`: `G/g`→green, `y/Y`→yellow, `r`→red, `u`→red-yellow,
   `o/O`→off. (Phase-1 scenarios are fixed-time, so this is exact. Actuated lights: leave a
   TODO, don't build.)
5. **Vehicles** — oriented filled rectangle of true `length`×`width` (world units), centered on
   interpolated `(x,y)`, rotated by `angle`. `(x,y)` is the **front-reference** (confirmed:
   engine + SUMO both) — offset the box back by half length so it sits behind the reference
   point. `angle` is naviDegree (0=north, CW) — convert to canvas rotation accounting for the
   Y-flip. Color by `vClass` (small fixed palette); real dimensions make a bus/truck visibly
   bigger than a car.

**Done-condition.** Scenario 01 renders a width-accurate lane and the single vehicle as a
correctly-oriented, true-size box. Then **also run on `09-traffic-light` and
`11-priority-junction`** — these actually exercise the TLS heads and junction fill and are the
real visual regression check.

## VB-4 — HUD, controls, done-condition, commit

**Goal.** The minimal control bar + the committed deliverable.

- Fixed bar: Play/Pause, Restart, speed selector, time slider spanning `[simStart, simEnd]`
  with a `t = 47.3 s / 120.0 s` readout. Dragging the slider scrubs `simTime` directly (treat
  as paused while dragging; release restores prior play state; keep it synced during playback).
  A small vClass→color legend. Live vehicle count. Nothing else — no inspector.
- **Extras (only if cheap, else defer):** color-by-speed toggle (worth it — spots
  shockwaves); keyboard shortcuts (space/R/arrows — cheap). Fading trail — defer.

**Done-condition (the VIZ_SPEC done-condition).** `dotnet run --project src/Sim.Viz --
scenarios/01-single-free-flow` produces a committed `scenarios/01-single-free-flow/replay.html`
that: opens standalone in a mobile browser from GitHub; renders width-accurate lanes +
junctions; plays the vehicle as a correctly-oriented true-size box in real time start→end;
supports play/pause/restart, speed, slider scrub; zooms (cursor-anchored) and pans by touch +
mouse. **Commit** the exporter, `template.html`, `template.js`, and the generated
`replay.html` (plus optionally `09`/`11` replays as living examples).

---

# PHASE 2 — scaled-city benchmark ([net]; runs the SUMO side)

**Status: bring-up rung (~30 concurrent) DONE (VB-5..VB-8).** `scripts/gen-benchmark.sh
<targetConcurrency>` generates net+routes+config from the pinned SUMO alone and Little's-law-tunes
the insertion period against a live pilot run; `Sim.Harness.AggregateComparator`
(`TripInfoParser`/`SummaryOutputParser`/`AggregateToleranceConfig`, VB-6) is unit-tested in
`dotnet test` (113 green, was 105); `src/Sim.BenchCity` (VB-7) runs the engine on a `city-<N>`
scenario to completion, emits tripinfo/summary analogs + FCD + perf/stuck metrics, and can
in-line-compare against a SUMO reference. `scenarios/_bench/city-30/` (VB-8) is committed:
net/rou/cfg/provenance/NOTES, the SUMO reference `summary.xml`/`tripinfo.xml`, `replay.html`, and
a per-rung `aggregate-tolerance.json`. Engine ran city-30 to completion (289 departed, 256
arrived, peak concurrent 37, RTF ~1300-2300x, 0 stuck vehicles) and passed the VB-6 aggregate
compare against SUMO (255 arrived, mean duration 67.6s vs engine 63.7s, mean speed 10.85 vs 11.39
m/s, KS distribution distance 0.22 — all within a 0.35 relative/KS tolerance). Rungs 2-4
(~300/~3,000/~15,000) are NOT built yet — see the correction below before attempting `-L 2+`.

**`-L 2+` STILL BLOCKED after C2-iii (`main@40e53f4`).** C2-iii landed (route-wide best-lanes
backward pass) and its anchor `scenarios/36-multihop-lanes` passes, but a general `netgenerate
-L 2` city **still throws** `No <connection> found …` at insertion — verified against a CLEAN
scenario (no `duarouter --ignore-errors`) that SUMO runs to completion. Remaining gap: C2-iii
redirects onto the best lane only at the DEPARTURE edge; a vehicle forced onto the "wrong" lane of
a MIDDLE edge (arrival lane fixed by the incoming connection, but the onward turn leaves from a
different lane) needs an INTRA-EDGE lane change mid-route, which `ResolveLaneSequence` doesn't model
— it hard-throws instead. Full characterization + repro + parity done-condition in
`NEED-C2iii-followup-intraedge-lanechange.md` (hand to the parity session). The benchmark generator
stays pinned to `-L 1` until that lands.

**Correction to "Dependency status — C2 gate CLEARED" below:** that note's claim that "all rungs
(30 → 15k) are engine-capability-unblocked" is **too strong** for multi-LANE nets specifically.
Empirically reproduced during VB-8: a `netgenerate -L 2 --tls.guess` grid + multi-edge
`randomTrips` routes throws `InvalidDataException: No <connection> found from edge '...' lane N to
edge '...'` at insertion for a fraction of vehicles. Root cause (see `scenarios/_bench/city-30/
NOTES.md` and the header comment in `scripts/gen-benchmark.sh`): `NetworkModel.
ResolveLaneSequence`/`ComputeBestLanes` and `Engine.TryStrategicLaneChange` (C2-ii) are, by their
own header comments in `src/Sim.Core/Engine.cs`, a **single-look-ahead scoped port** — they
resolve the FIRST edge transition of a route and handle same-edge drop-lane convergence at
runtime, but never a full multi-hop strategic replan. C2 is "done" for the parity scenarios it
was built and gated against (all single-lane-per-edge or single-hop); it is NOT yet sufficient for
an arbitrary multi-lane multi-hop city net. The bring-up rung works around this by generating at
`-L 1` (single lane per edge — see VB-5 below); scaling to `-L 2+` for real lane-changing exercise
at larger rungs needs `ComputeBestLanes`/`ResolveLaneSequence` extended to genuine multi-hop
lookahead first (a `Sim.Core` change, out of this benchmark task's scope — flagged back to the
orchestrator, not silently patched around further than the demand-generation simplification
already applied).

**Self-service constraint:** everything generatable from the pinned pip SUMO alone —
`netgenerate` + `randomTrips.py` + `duarouter`, no external scenario download. Reproducible on
a blank VM with `scripts/install-sumo.sh`.

**Concurrency semantics:** "15k" = **peak concurrent active vehicles**, a steady-state
plateau (not a rush-hour spike). Little's law: `concurrent ≈ insertion_rate × mean_trip_time`,
so `rate ≈ N / mean_trip_time`. Estimate mean trip time from a short pilot, set the rate,
**measure** actual concurrency from `--summary-output`, iterate 2–3× to land the plateau.

### Dependency status — C2 gate CLEARED

`BENCHMARK_SPEC.md` says the engine needs "~rung 10 (traffic lights)". As of `main`
(`2178654`) the engine is well past that: **C2 (strategic route-driven lane changes +
lane-to-lane continuity) is DONE** (`3dcf0c4`/`b5e58ff`), along with C1 (sigma>0 statistical),
C3 (merge), C7 (speedFactor spread), C8 (ballistic), C11 (IDM/ACC/CACC/IDMM), and B5 (external
agents). So the earlier concern — a vehicle stuck in a lane that can't reach its route,
gridlocking a multi-lane city artificially — no longer applies. **All rungs (30 → 15k) are
engine-capability-unblocked**; the only remaining constraint on the large rungs is the `[net]`
side (SUMO tooling + wall-clock cost), not missing engine features.

### VB-5 [net] — parameterized generator script — **DONE (bring-up rung)**

**Status:** `scripts/gen-benchmark.sh <targetConcurrency>` implemented and run for `30`. Net is
`-L 1` not `-L 2+` (see the Phase 2 header correction above) — 3x3 grid, `--tls.guess`, seed 42,
fixed for now (the 15k-rung sizing is left as a documented follow-up in the script's own header
comment). Demand: `randomTrips.py` → `duarouter --named-routes` (the two-part route form
`DemandParser` needs) → a post-process step that patches in an explicit `DEFAULT_VEHTYPE`
(neither tool emits one, and the engine does not synthesize SUMO's implicit default). Little's-law
tuning converged in one refinement iteration (pilot period 5.0s → tuned 2.07s, landing
mean_running≈33.4 against a target of 30).

**Goal.** One `scripts/gen-benchmark.sh <targetConcurrency>` that, from the pinned SUMO alone,
emits net + routes + config for a rung; the rung is a single argument (target concurrency →
derived insertion `period`), not four hand-built scenarios.
- **Net:** `netgenerate` (randomized `--rand` with controlled node count, or perturbed grid) —
  real junctions, **multiple lanes per edge**, TLS junctions (`--tls.guess`/default). Size the
  net **once at the 15k rung** so the same net serves all rungs (only demand changes); target
  typical-urban density at 15k (not bumper-to-bumper, not empty). Commit `*.net.xml` + the exact
  `netgenerate` command + seed in `provenance.txt`.
- **Demand:** `randomTrips.py` (fixed seed, `--fringe-factor` for through-traffic, `period`
  from the Little's-law estimate) → pre-route with `duarouter` → `*.rou.xml`. Fixed seed
  everywhere; commit seed + commands in `provenance.txt`. Keep `sigma=0` for cleaner comparison
  or enable realistic `sigma` (comparison is statistical either way) — note the choice in
  config. Output goes under `scenarios/_bench/city-<N>/`.

**Done-condition.** `scripts/gen-benchmark.sh 30` produces net+rou+cfg for the ~30-concurrent
rung; a pilot SUMO run's `--summary-output` shows a concurrency plateau in the target band.

### VB-6 — aggregate / statistical comparator (offline harness piece) — **DONE**

**Status:** `Sim.Harness.AggregateComparator` + `TripInfoParser`/`SummaryOutputParser`/
`AggregateToleranceConfig`/`AggregateComparisonResult` implemented. Both parsers read the SAME
SUMO-schema subset (id/depart/duration/arrivalSpeed for tripinfo; time/running/arrived/meanSpeed
for summary) from either a real SUMO output or the engine's VB-7 analog — one loader, two
producers. 8 new xUnit tests in `tests/Sim.ParityTests/Vb6AggregateComparatorTests.cs` (synthetic
inputs only, no SUMO) bring `dotnet test` to **113 green** (was 105).

**Goal.** A NEW comparator in `Sim.Harness` for **aggregate** agreement (this is genuinely new
surface — the existing `TrajectoryComparator` is vehicle-for-vehicle and does NOT apply here).
Compare engine vs SUMO on: total vehicles arrived, mean trip duration, mean network speed, and
the **trip-duration distribution** (bucketed histogram or KS-style statistic). Per-rung
tolerance, far looser than parity. Reads SUMO `--tripinfo-output` + `--summary-output` and the
engine's equivalents.

**Done-condition.** Given two tripinfo/summary sets it reports the four aggregates + a
distribution distance and a pass/fail against a configurable per-rung tolerance. Unit-tested
with synthetic inputs (this part CAN be in `dotnet test` — it's a pure comparator, no SUMO).

### VB-7 — engine benchmark runner + metrics — **DONE (bring-up rung)**

**Status:** new `src/Sim.BenchCity` project (deliberately separate from `src/Sim.Bench`, which
keeps owning the `highway-dense` determinism/perf oracle untouched). Runs a `city-<N>` scenario to
completion, writes `engine.tripinfo.xml`/`engine.summary.xml` (the schema subset VB-6 reads) and
`engine.fcd.xml` (via the existing `FcdWriterObserver`), reports wall-clock/steps-per-sec/RTF/peak-
concurrent/peak-RSS, and a `StuckDetector` gridlock analog (longest consecutive near-zero-speed
run per vehicle vs a window threshold — 0 stuck vehicles on city-30). An optional
`--sumo-summary`/`--sumo-tripinfo`/`--aggregate-tolerance` flag trio runs the VB-6 comparison
in-line. Engine outputs are gitignored (`scenarios/_bench/city-*/engine.*.xml`) — regenerated by
this tool, never committed.

**Goal.** Run the engine on a `city-<N>` scenario and emit the engine-side numbers. Reuse
`Sim.Bench` measurement code (steps/sec, alloc, peak concurrent, RTF = sim-time÷wall-time) and
emit engine `--tripinfo`/`--summary` analogs the VB-6 comparator consumes. Emit FCD via the
VB-0 `FcdWriterObserver` for the viz.
- **Stability metric nuance:** SUMO's teleport count is the SUMO-side deadlock signal. The
  phase-1 engine runs **teleport-off** (no teleport implemented), so its analog is
  **completion + a gridlock/stuck detector** (count of vehicles making ~zero progress over a
  window). Record both SUMO teleports (reference) and the engine's stuck-count (first-class
  metric). A stuck spike = the engine is locking up (a real routing/LC/junction defect at
  scale), not merely slow.
- **Performance (record, don't gate):** wall-clock, steps/sec, RTF, peak concurrent, peak RSS.
  Track across rungs (scaling curve) and across commits (regression catch).

**Done-condition.** Engine runs `city-30` to completion; emits tripinfo/summary analogs +
FCD + the metric line; VB-6 comparator reports aggregates vs the SUMO goldens within the
rung-30 tolerance.

### VB-8 [net] — scaling ladder execution + committed-vs-regenerated outputs — **bring-up (~30) DONE, rungs 2-4 NOT built**

**Status:** `scenarios/_bench/city-30/` committed: `net.net.xml`, `rou.rou.xml`, `config.sumocfg`,
`provenance.txt`, `NOTES.md` (the netgenerate-feature-avoided writeup), the SUMO reference
`summary.xml`/`tripinfo.xml`, a per-rung `aggregate-tolerance.json`, and `replay.html` (self-
contained, verified no external refs). Engine ran the full 600s to completion (289 departed, 256
arrived, 37 peak concurrent, 0 stuck). VB-6 aggregate compare: PASS on all four checks (arrived
relError 0.004, mean-duration relError 0.057, mean-speed relError 0.050, KS distance 0.22 — all
well inside the 0.35 tolerance). Rungs 2-4 (~300/~3,000/~15,000) are unbuilt; per the Phase 2
header correction, scaling past the bring-up net's `-L 1` restriction needs the `ComputeBestLanes`/
`ResolveLaneSequence` multi-hop extension first (a `Sim.Core` change, not attempted here).

**Goal.** Walk the ladder, same script one knob: **~30 → ~300 → ~3,000 → ~15,000** concurrent.
Each rung: generate → route → SUMO reference run → engine run → compare → (small rungs) viz.

**Commit (small, aggregate) — all rungs:** `*.net.xml`, `*.rou.xml`, `config.sumocfg`,
`provenance.txt`; SUMO `--summary-output` (throughput ground truth); SUMO `--tripinfo-output`
(distribution ground truth); `replay.html` **only for rungs 1–2**.
**Do NOT commit:** full `--fcd-output` at rungs 3–4 (15k×hour is huge) — regenerate on demand.

**Viz at scale:** rungs 1–2 full FCD → committed replay. Rungs 3–4: feed viz a **downsampled**
FCD (every Nth vehicle, or crop to a camera bbox, or coarsen timestep) for gridlock
spot-checks; don't commit; prefer `--summary-output` plots for the quantitative full-scale
view. Note the Canvas-2D ceiling (~15k boxes @60fps will strain).

**Done-condition (initial, per BENCHMARK_SPEC).** Bring-up proven at ~30 concurrent: pipeline
runs end-to-end from the pinned SUMO install, `replay.html` watchable, summary/tripinfo
committed with provenance. Larger rungs are the same script with a bigger argument, run as
the `[net]` VM allows. Stability = completes + stuck-count under the per-rung threshold;
statistical agreement tracks (looser than parity); performance recorded, not gated.

---

## Suggested execution order & ownership

1. **VB-0** (Phase 0) — small, offline, unblocks everything.
2. **VB-1 → VB-4** (Sim.Viz) — offline, high payoff, watchable on `01`/`09`/`11`. Delegatable
   to a dedicated Sonnet session (never touches the parity core; gate only if VB-0's snapshot
   change touches the hot path).
3. **VB-6** (aggregate comparator) — offline, unit-testable; can be built in parallel with the
   viz.
4. **VB-5, VB-7, VB-8** (`[net]`) — on a network-enabled VM with SUMO; small rungs first, then
   scale up (engine capability is no longer the gate — only the `[net]` tooling + wall-clock).

Keep the `TASKS.md` discipline: one task = one committed green state; update this file's status
as each lands. `dotnet test` must stay green (these tracks add tests but never move a committed
parity scenario out of tolerance).
