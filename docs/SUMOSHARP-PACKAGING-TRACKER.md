# SUMOSHARP-PACKAGING-TRACKER.md — at-a-glance to-do

Checklist for the packaging rethink. Task IDs → `SUMOSHARP-PACKAGING-TASKS.md`; design →
`SUMOSHARP-PACKAGING-DESIGN.md`. A box is ticked only when the task's success conditions are
verified first-hand (build / `dotnet pack` / `dotnet test`), per the CLAUDE.md accept gate.

## Baseline (integrated this session)
- [x] Fast-forward the Windows-GPU viewer branch (`native-viewer-perf-gpu`, main + 8 commits) into
      the packaging branch — brings DR/smoothing **code** (DrClock arc-window, lane-change straddle,
      ChordHeading, auto-delay, extrapolation low-pass) + **docs**.
- [x] DR/smoothing reimplementation guide present: `SUMOSHARP-VIEWER-DR-SMOOTHING.md` (+ lane-change
      design/tasks, DR-motion-jitter investigation).
- [x] Rebased onto updated `main` (main + gpu branch converged; +11 commits): brings the
      **DR-error-based publishing** feature — `Sim.Replication` now owns `DrExtrapolation` (shared DR
      math), `DrErrorPublishPolicy`, and `PublishScheduler` per-vehicle reference state. Its design
      (`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md §8`) already adopts the "data model is the API"
      layering → reinforces D8/D9; part of the motion extraction (arc extrapolation) is already done.
- [x] Offline parity gate green after rebase: **446 passed, 0 failed, 3 skipped** (was 440; +6 from
      the new DR-error tests).

## Stage P0 — Reconcile docs with reality
- [x] P0.1 — Packaging design/tasks/tracker docs landed.
- [x] P0.2 — `SUMOSHARP-API.md §1` points here; two-package reality + retired `Runtime` recorded.

## Stage P1 — Replication transport contract + neutral sample (D8, D9) — critical path
- [ ] P1.1 — `IReplicationSink`/`IReplicationSource` (4-channel contract) added in `Sim.Replication`.
- [ ] P1.2 — `TimestampedSample` + `IVehicleSampleHistory` added in `Sim.Replication` (+ history test).
- [ ] P1.3 — `Sim.Replication.Dds` refactored to *implement* the contract (channel model no longer
      defined DDS-side; `DdsF32Col` columns stay as encoding).
- [ ] P1.4 — in-memory-transport round-trip test proves a second binding (hermetic, no CycloneDDS).

## Stage P2 — `SumoSharp.Viewer.Motion`
- [ ] P2.1 — `DrClock` decoupled from `DdsSubscriber` onto the P1 sample/history; branch/arc
      regression test.
- [ ] P2.2 — `Sim.Viewer.Motion` project created: net8+ns2.1, `IsPackable`, `PackageId=SumoSharp.Viewer.Motion`,
      no DDS/raylib deps; packs `lib/net8.0` + `lib/netstandard2.1`.
- [ ] P2.3 — DR/smoothing guide shipped as the package README (+ license/disclaimer).

## Stage P3 — Generic `SumoSharp.Viewer.Raylib` + demo-tool separation (D5, D10)
- [ ] P3.1 — render-overlay seam (D10) on the generic viewer (domain draws plug in, no viewer dep).
- [ ] P3.2 — relocate demo/evac code (`DemoCatalog`, `DemoSession`, `EvacRenderSnapshot`, evac path)
      out of `Sim.Viewer.Core`; move the `→ Sim.Evac` edge to the demo layer; evac draws via the seam.
- [ ] P3.3 — generic viewer packable (`PackageId=SumoSharp.Viewer.Raylib`, native assets, NO evac/demo);
      demo tool stays a sample; viewer modes + picker still run.

## Stage P4 — Dev-time & domain packages
- [ ] P4.1 — `SumoSharp.Testing` from `Sim.Harness`.
- [ ] P4.2 — `SumoSharp.Evac` from `Sim.Evac`.

## Stage P5 — Convenience & CI
- [ ] P5.1 — `SumoSharp` meta-package (Core + Ingest + Replication + Viewer.Motion).
- [ ] P5.2 — packaging guard test extended (targets/packability, no-native-leak, contract-in-Replication).
- [ ] P5.3 — publish CI packs the full shipped set on a `v*` tag.

## Already shipped before this session (context)
- [x] `SumoSharp.Core`, `SumoSharp.Ingest` — packable, net8+ns2.1, publish CI, B13 guard.
- [x] `SumoSharp.Replication`, `SumoSharp.Replication.Dds` — packable.
