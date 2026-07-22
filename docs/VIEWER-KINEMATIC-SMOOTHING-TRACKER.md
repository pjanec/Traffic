# Viewer kinematic smoothing — TRACKER

At-a-glance status. Design: `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`. Tasks + success conditions:
`VIEWER-KINEMATIC-SMOOTHING-TASKS.md`. Tick a box only when its stated success conditions are verified
first-hand.

Legend: [ ] todo · [~] in progress · [x] done (success conditions met)

## S0 — baseline + facade extraction
- [ ] T0.1 Capture DrPoseSmoother metric baseline (both viewers)
- [x] T0.2 Extract `KinematicReconstructor` into `Sim.Viewer.Motion`; IgBridge trace byte-identical to v5 ✓ (verified: byte-diff clean, deterministic, Motion 11/11, IgBridge 11/11, Parity 654/4)
- [ ] T0.3 Facade unit tests (straight / junction / lane-change / stop / coarse-vs-dense)

## S1 — Raylib 2D swap
- [ ] T1.1 Confirm box pivot; wire the facade (loopback + remote)
- [ ] T1.2 Metrics beat the T0.1 baseline (committed before/after table)
- [ ] T1.3 Unify `--mode local` onto the kinematics
- [ ] T1.4 Delete `DrPoseSmoother` (file + tests + all call sites)

## S2 — City3D 3D swap
- [ ] T2.1 Package the facade; bump City3D local feed
- [ ] T2.2 Reconstructor: straddle-lerp + facade + center-pivot; CityLib.Tests green
- [ ] T2.3 Godot headless smoke

## S3 — tune, prove, sign-off
- [ ] T3.1 Render-rate + stutter robustness (30/60/144 Hz + stall)
- [ ] T3.2 Lock defaults + commit metric regression check
- [ ] T3.3 Docs updated + owner desktop sign-off; decide DrPoseSmoother deletion

## Standing gates (re-verify every src/-touching task)
- [ ] `dotnet test Traffic.sln` 654 pass / 4 skip, byte-identical
- [ ] `Sim.Bench` determinism hash unchanged
- [ ] IgBridge emitted trace byte-identical to v5
- [ ] No ProjectReference from demos/City3D into src/

## Owner decisions (Design §7) — RESOLVED, green to go
- [x] Q1 DrPoseSmoother → DELETE outright (no toggle)
- [x] Q2 `--mode local` → UNIFY onto the kinematics
- [x] Q3 City3D via `SumoSharp.Viewer.Motion` package bump → OK
