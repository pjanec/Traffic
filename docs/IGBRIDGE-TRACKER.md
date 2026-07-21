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

- [ ] **T0.2** Scaffold `Sim.IgBridge` (`netstandard2.1;net6.0;net8.0`) + `Sim.IgBridge.Host` (net8); `Sim.IgBridge` builds for net6
- [ ] **T0.3** Fixed-10 Hz core loop + per-entity ring buffers + sim clock; determinism test

## Stage 1 — reconstruct + emit + FakeIg + baseline
- [ ] **T1.1** `DrClock.ResolveAt(history, sampleT, lanes)` deterministic seam (pure refactor)
- [ ] **T1.2** Vehicle reconstruct (ResolveAt + PoseResolver + DrPoseSmoother) + emit JSONL trace; byte-identical across runs
- [ ] **T1.3** `FakeIg` replay: 2-most-recent interp @ `clock−igDelay`, shortest-arc heading, threshold-jump
- [ ] **T1.4** Side-by-side render (raw vs FakeIg-reconstructed) via `Sim.Viz` two-scene payload
- [ ] **T1.5** Baseline metrics pass (yaw-rate, yaw-jerk, lateral-accel, C1 gap, lane-change duration) raw vs reconstructed

## Stage 2 — lane-change ease + lifecycle
- [ ] **T2.1** Lane-change ease over ~1.3 s in `Sim.Viewer.Motion` (detect perpendicular snap → smoothstep); duration ∈ [1.2,1.5] s
- [ ] **T2.2** Lifecycle (`new`/`del`) from SimEvent/ped add-remove + non-lane-change discontinuity handling (no 60 m/s slide)

## Stage 3 — tune & prove
- [ ] **T3.1** Sweep igDelay/emit-rate/W; lock latency budget; confirm no extrapolation at igDelay ≥ 0.5 s
- [ ] **T3.2** Commit offline metrics regression check (no SUMO, no network)
- [ ] **T3.3** Final determinism + parity gate (byte-identical trace + goldens; net6 build)
