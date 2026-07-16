# 3D city demo — tracker

Checklist for `docs/DEMO-CITY3D-TASKS.md` (design: `docs/DEMO-CITY3D-DESIGN.md`). A box is ticked ONLY
after Opus confirms the task's success conditions first-hand (diff read + gate/self-test re-run; the
desktop-only parts are confirmed by the user and noted as such — never faked from a headless run).

**Standing gate on every `src/`-touching tick:** `dotnet test Traffic.sln -c Release` at the committed
baseline + `Sim.Bench` determinism hash unchanged (single + parallel). Capture the baseline numbers on
the first `src/` task and repeat them here as each box is ticked.

## Stage 0 — Foundations (publisher package + local feed)
- [ ] **T0.1** `SumoSharp.Host` (`ReplicationPublisher`) — additive lib, packs, self-test green, gate unchanged
- [ ] **T0.2** DRY rewire: `DdsReplicationSink` → `Replication.Dds`, `DdsPublisher` delegates — loopback self-test green, no duplicated publisher, gate unchanged *(fallback: ship T0.1 alone if risky)*
- [ ] **T0.3** local feed: `build.sh` + `nuget.config` (packageSourceMapping) + `.gitignore` — packs, resolves from local feed, fails without it

## Stage 1 — Local single-viewport demo (M1, public-facing) — no `src/` changes
- [ ] **T1.1** Godot skeleton consuming packages + `--headless` N-frames smoke
- [ ] **T1.2** in-proc engine → `InMemoryReplicationBus` → DR reconstruction; `ReplicationLaneShapeSource`; local↔remote seam proven
- [ ] **T1.3** procedural roads (width-accurate, `LaneShapeZ`) — *desktop: looks right*
- [ ] **T1.4** procedural buildings (deterministic, believable heights, set-back) — *desktop: box-city*
- [ ] **T1.5** cars (MultiMesh, true vType dims, heading yaw, smooth) — *desktop: true-size & smooth*
- [ ] **T1.6** simplified traffic lights (head colour == replicated signal) — *desktop: colours match, queues*
- [ ] **T1.7** scenario switch + scale polish (`09-traffic-light` / `city-mixed-1k` / `city-organic-L2`) — *desktop: believable scale*

## Stage 2 — Remote (M2)
- [ ] **T2.1** `src/Sim.Host.App` generic headless DDS host — publishes (dds + inmem), self-test green, gate unchanged
- [ ] **T2.2** viewer remote mode (DDS source), single channel — headless receive+reconstruct, render code unchanged vs local — *desktop: live remote render*

## Stage 3 — Video wall + performance (M3)
- [ ] **T3.1** auto-detect video wall (N windows/cameras, 1 subscription; default 1) — *desktop: spans monitors; N independent viewers*
- [ ] **T3.2** performance ladder (`city-30…15000`, `city-mixed-1k`) — host rates logged — *desktop: ~1k at interactive fps*

## Stage 4 — Close-out
- [ ] **T4.1** demo README + `PACKAGES.md`/`DEMOS.md`/root README wiring + fresh-clone dry run + final gate

Status: **DESIGN DRAFTED — awaiting sign-off. No code yet.**
