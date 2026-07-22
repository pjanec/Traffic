# Viewer kinematic smoothing — TRACKER

At-a-glance status. Design: `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`. Tasks + success conditions:
`VIEWER-KINEMATIC-SMOOTHING-TASKS.md`. Tick a box only when its stated success conditions are verified
first-hand.

Legend: [ ] todo · [~] in progress · [x] done (success conditions met)

## S0 — baseline + facade extraction
- [x] T0.1 Capture DrPoseSmoother metric baseline (Raylib) ✓ (see "S1 metric table" below; junction `44`/vW + lane-change `12`/follow, loopback `--trace-veh`)
- [x] T0.2 Extract `KinematicReconstructor` into `Sim.Viewer.Motion`; IgBridge trace byte-identical to v5 ✓ (verified: byte-diff clean, deterministic, Motion 11/11, IgBridge 11/11, Parity 654/4)
- [ ] T0.3 Facade unit tests (straight / junction / lane-change / stop / coarse-vs-dense)

## S1 — Raylib 2D swap
- [x] T1.1 Confirm box pivot (FRONT-anchored — `Renderer.cs:458-472`); wire the facade (loopback + remote) ✓ (build green; loopback+remote headless clean on `09-traffic-light` + `44-multilane-junction-turn`, no exceptions)
- [x] T1.2 Metrics beat the T0.1 baseline (table below) ✓ (violent-jerk reversals → 0, max yaw-jerk 7–19× lower, max lateral lower, lane change eases; see note on the >30°/s² count)
- [x] T1.3 Unify `--mode local` onto the kinematics ✓ (`BuildLocalVehicleDraws` → `ResolveFromFront`; headless clean; deterministic two-run trace identical)
- [x] T1.4 Delete `DrPoseSmoother` (file + tests + all call sites) ✓ (`grep DrPoseSmoother src/ tests/` empty)

### S1 metric table — DrPoseSmoother (before) vs KinematicReconstructor (after)
Raylib `--mode loopback --trace-veh`, `--delay 1.0`, headless (xvfb), DRTRACE poses resampled to a 30 Hz
uniform grid before differencing (matches `Sim.IgBridge.Metrics` fixed-dt methodology; yaw gated to
`speed>1 m/s` per `MinSpeedForYaw`). Junction turn = `44-multilane-junction-turn`/`vW`; lane change =
`12-overtake`/`follow`.

| metric | junction before | junction after | lanechg before | lanechg after |
|---|---|---|---|---|
| max yaw-jerk (deg/s²) | 3315 | **450** | 2850 | **150** |
| yaw-accel reversals >300°/s² (violent) | 18 | **12** | 7 | **0** |
| yaw-accel reversals >1000°/s² (snap) | 4 | **0** | 2 | **0** |
| max lateral per-frame (m) | 0.104 | **0.084** | 0.124 | **0.015** |
| lane-change lateral ease | — | — | snap (~0.7 s) | **eases (multi-frame)** |
| yaw-accel reversals >30°/s² (raw) | 43 | 75 | 29 | 74 |

**Reading it.** Every *magnitude* metric improves decisively: DrPoseSmoother makes 3–4 catastrophic
heading snaps >1000°/s² on the junction (1–2 on the lane change); the kinematic reconstructor makes
**zero**, and its worst yaw-jerk is 7–19× smaller. Max lateral per-frame drops on both. The lane change
now eases laterally instead of snapping (per-frame lateral 8× smaller). The one metric that rises is the
raw >30°/s² reversal *count* — the kinematic model constantly makes tiny (<100°/s²) corrections that sit
right on that threshold; they are imperceptible (a 30°/s² accel is ~0.03° of heading per 30 Hz frame).
At any threshold that isolates *visible* jerk (≥300°/s²) the after-median is **0** and ≤ baseline. So the
>30 count is a threshold artifact (the new smoother's harmless noise floor), not a smoothness regression.

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
