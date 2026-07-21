# IgBridge ↔ IG binding — TRACKER

At-a-glance to-do for the tasks in `IGBRIDGE-TASKS.md`. A box is ticked only when its
stated success conditions are verified first-hand (per CLAUDE.md: read the diff, read the test, re-run
`dotnet test` — do not trust a "done" report). Code name **IgBridge**; no `"IgBridge"` in code.

**Status:** design phase — awaiting owner sign-off on the trio (DECISIONS / TASKS / TRACKER). No code yet.

## Stage 0 — retarget, scaffold, prove .NET 6
- [x] **T0.1** Retarget `Sim.Pedestrians` → `netstandard2.1;net8.0`; `dotnet test` green + goldens byte-identical
      <!-- done: both TFMs build; net8 tests byte-identical (peds 227, parity 654/4skip). Gaps closed:
           System.Text.Json + System.Memory pkgrefs (ns2.1), NetstandardPolyfills link, ISet<int> for
           IReadOnlySet<int> (per Sim.Ingest/NetworkRouter precedent), Int64-bits double bridge in
           ActivityTimelineWire, manual null-guards for ArgumentNullException.ThrowIfNull. -->

- [x] **T0.2** Scaffold `Sim.IgBridge` (`netstandard2.1;net6.0;net8.0`) + `Sim.IgBridge.Host` (net8); `Sim.IgBridge` builds for net6
      <!-- done: producer builds all 3 TFMs (net6 embed proof: consumes ns2.1 deps incl Sim.Pedestrians);
           Host builds+runs on net8; solution green; dotnet test byte-identical (peds 227, parity 654/4).
           Polyfill scoped to ns2.1 only (net6 has IsExternalInit); IgSample schema (New/Upd/Del, planar)
           added. -->
- [x] **T0.3** Fixed-10 Hz core loop + per-entity ring buffers + sim clock; determinism test
      <!-- done: IgBridgeRunner replays box's real 819-route demand demand-less (RouteDemand bypasses
           DemandParser's departPos="base"/parking reject) at exact 0.1s ticks; PedStream synthesizes a
           deterministic 40-ped PathArc crowd (monotonic trip cursor, no System.Random) buffered in a
           SEPARATE PedSampleHistory (not DrClock). 5/5 IgBridge tests: exact cadence + non-vacuous +
           two-run byte-identical for BOTH vehicle and ped streams. Full gate green, parity 654/4
           byte-identical. Note: nav-bake makes IgBridge tests ~1m16s. -->


## Stage 1 — reconstruct + emit + FakeIg + baseline
- [x] **T1.1** `DrClock.ResolveAt(history, sampleT, lanes)` deterministic seam (pure refactor)
      <!-- done: Resolve now delegates to ResolveAt(history, _renderSim-delay, lanes); EffectiveDelay
           semantics preserved (unset on empty-history throw). 4-branch equivalence theory + 3 existing
           regression pins green (7/7); solution builds; DrClock is render-side, not in parity path. -->

- [x] **T1.2** Vehicle reconstruct (ResolveAt + PoseResolver + DrPoseSmoother) + emit JSONL trace; byte-identical across runs
      <!-- done: IgBridgeSession resamples the reused stack at 20 Hz (shared tQuery/instant, coherent),
           vehicles via ResolveAt->PoseResolver(ChordHeading)->DrPoseSmoother (straddle skipped, = viewer
           Stage 1; T2.1 adds the ease), peds via linear resample of PedSampleHistory (velocity heading).
           IgTraceWriter = deterministic JSONL (new/upd/del, planar x,y,h). Host writes a 269k-record
           120s trace; 2 runs byte-identical. Tests 7/7: emit determinism + lifecycle-correct
           (0 upd-before-new / new-after-upd / upd-after-del, new==del) non-vacuous JSONL. -->

- [x] **T1.3** `FakeIg` replay: 2-most-recent interp @ `clock−igDelay`, shortest-arc heading, threshold-jump
      <!-- done: FakeIg reconstructs each entity's displayed pose (2-sample linear pos, shortest-arc
           heading, jump-threshold snap for teleports, extrapolate when delay too small). 3 unit tests
           (interp, shortest-arc-through-north, jump-not-smear). -->
- [ ] **T1.4** Side-by-side render (raw vs FakeIg-reconstructed) via `Sim.Viz` two-scene payload
- [x] **T1.5** Baseline metrics pass (yaw-rate, yaw-jerk, lateral-accel, C1 gap) raw vs reconstructed
      <!-- done: raw engine @10Hz vs IgBridge-smoothed @20Hz, both reconstructed via FakeIg, compared.
           Honest finding at 10Hz: JUNCTION TURNS -> among turning vehicles 88% (68/77) improve, turner-
           median yaw-rate 2.3x lower; LANE CHANGES -> all-vehicle mean peak lat-accel ~1.6-3.2x lower.
           max/p95 dominated by genuine hairpin U-turns present in raw too (median/mean are the robust
           story). Metric measures on FakeIg-reconstructed streams (measured, not eyeballed). Regression
           test (paired by id, turner-filtered) seeds T3.2. Lane-change DURATION deferred to T2.1 (needs
           the ease to exist). -->


## Stage 2 — lane-change ease + lifecycle
- [ ] **T2.1** Lane-change ease over ~1.3 s in `Sim.Viewer.Motion` (detect perpendicular snap → smoothstep); duration ∈ [1.2,1.5] s
- [ ] **T2.2** Lifecycle (`new`/`del`) from SimEvent/ped add-remove + non-lane-change discontinuity handling (no 60 m/s slide)

## Stage 3 — tune & prove
- [ ] **T3.1** Sweep igDelay/emit-rate/W; lock latency budget; confirm no extrapolation at igDelay ≥ 0.5 s
- [ ] **T3.2** Commit offline metrics regression check (no SUMO, no network)
- [ ] **T3.3** Final determinism + parity gate (byte-identical trace + goldens; net6 build)
