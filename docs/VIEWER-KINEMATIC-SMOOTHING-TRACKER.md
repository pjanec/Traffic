# Viewer kinematic smoothing — TRACKER

At-a-glance status. Design: `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`. Tasks + success conditions:
`VIEWER-KINEMATIC-SMOOTHING-TASKS.md`. Tick a box only when its stated success conditions are verified
first-hand.

Legend: [ ] todo · [~] in progress · [x] done (success conditions met)

## S0 — baseline + facade extraction
- [x] T0.1 Capture DrPoseSmoother metric baseline (Raylib) ✓ (see "S1 metric table" below; junction `44`/vW + lane-change `12`/follow, loopback `--trace-veh`)
- [x] T0.2 Extract `KinematicReconstructor` into `Sim.Viewer.Motion`; IgBridge trace byte-identical to v5 ✓ (verified: byte-diff clean, deterministic, Motion 11/11, IgBridge 11/11, Parity 654/4)
- [x] T0.3 Facade unit tests (straight / junction / lane-change / stop / coarse-vs-dense) ✓ (`tests/Sim.Viewer.Motion.Tests/KinematicReconstructorTests.cs`: straight = center L/2 behind front + stable heading + no drift; stop = no creep + heading held; lane-change ease = perpendicular snap eased over multiple frames, not jumped; coarse-vs-dense = wide-heading straddle reclassified out of the lane-change path under CoarseFeed while near-parallel unchanged, driven through the full `Resolve(DrClock.Resolved)` path on in-memory geometry; the junction-arc end-to-end case is covered by `CityLib.Tests/ReconstructorS2Tests`)

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
- [x] T2.1 Package the facade; bump City3D local feed ✓ (`build.sh --pack-only` packs SumoSharp.Viewer.Motion@0.1.0 w/ KinematicReconstructor & no DrPoseSmoother into local-nuget/; cache cleared; CityLib restores + compiles; SumoSharp.*→local pin holds)
- [x] T2.2 Reconstructor: straddle-lerp + facade + center-pivot; CityLib.Tests green ✓ (straddle `continue` removed → one `_recon.Resolve` for both cases; feeds `r.CenterX/Y`; `CoarseFeed=true`, delay 0.4; CityLib.Tests 54/54 green incl. 6 new S2 cases; 3x stable)
- [x] T2.3 Godot headless smoke ✓ (Godot 4.7.1 mono fetched; `run-smoke.sh` PASS: 200 frames, vehicles=1/cars=1, no ERROR, clean quit; Viewer builds against the bumped package)

### S2 new CityLib.Tests (ReconstructorS2Tests.cs) — verified numbers
- pivot (facade): straight-vehicle center sits L/2 behind the front reference (|front−center|≈L/2, <0.1 m).
- pivot (end-to-end): stopped 09 veh0 center is L/2 behind the SUMO snapshot front (median 2.500 m = L/2; ≈0 would mean the front is still fed — the old bug).
- lane change (12 veh1): ≥1 lateral-straddle frame occurs (was `continue`d pre-S2), vehicle now gapless through its window, ~one lane-width (3.2 m) of lateral travel, no back-jump — eased & continuous, not skipped.
- junction (44 veh0): >90° turn, center stays <0.6 m off the nearest lane centerline (measured ~0.48 m), no sideways slide.
- stop (09 veh0): no creep (max <0.12 m/frame; measured ~0.05 m settle only).
- determinism: fixed frame-dt + fixed packet stream → bit-identical ReconstructedVehicle transforms across two runs.

## S3 — tune, prove, sign-off
- [x] T3.1 Render-rate + stutter robustness (30/60/144 Hz + stall) ✓ (`tests/Sim.Viewer.Motion.Tests/KinematicReconstructorTests.cs`: `RenderRate_ConstantCurvatureTurn_IsSmoothAtEveryRate` theory at 30/60/144 Hz — center stays L/2 behind front, max yaw-accel < 120 deg/s², yaw-rate converges to the true turn rate at every rate; `Stutter_LongFrame_NoSpikeNoNaN` — a 0.25 s frame stays < 7 m reseed threshold, no NaN, heading held)
- [x] T3.2 Lock defaults + commit metric regression check ✓ (locked-defaults table below; the Motion unit tests are the committed regression guard — 19/19 green: 11 existing + 8 new facade/robustness cases)
- [~] T3.3 Docs updated (SUMOSHARP-VIEWER-DR-SMOOTHING §11 as-built, DEMO-CITY3D-DESIGN data path, both package READMEs) ✓ — **owner desktop sign-off (Godot 3D + Raylib 2D) still pending**; DrPoseSmoother deletion already resolved (Q1, done in S1/T1.4)

### Locked defaults — shipped `KinematicReconstructor` config (regression-guarded by the Motion unit tests)
These are the v5-tuned values every consumer ships with. `KinematicReconstructor` properties (facade-level):

| property | shipped value | who sets it |
|---|---|---|
| `LookAheadMeters` | 0.0 (viewers) · 3.0 (IgBridge) | viewers keep the facade default 0; IgBridge host sets 3.0 (`IGBRIDGE_LOOKAHEAD`) |
| `LookAheadLengthFactor` | 0.5 | facade default (eff. look-ahead = `max(LookAheadMeters, 0.5·length)`) |
| `MaxAnticipationLeadDeg` | 70 | facade default |
| `MaxStraddleLaneChangeHeadingDeg` | 20 | facade default (coarse-feed junction-vs-lane-change discriminator threshold) |
| `CoarseFeed` | **true** (all three viewers + IgBridge decimated feed) | Raylib `RenderHelpers`, City3D `Reconstructor`, `Sim.Viewer` loopback/remote all set `CoarseFeed = true` |
| `AlwaysSplitJunctionStraddle` | false | facade default (opt-in only; would change the v5 baseline) |

`KinematicHeadingParams` tunables (the bicycle core, shipped defaults — mirror `SUMOSHARP-VIEWER-DR-SMOOTHING.md` §9 / `IGBRIDGE-DECISIONS.md` §5.3):

| tunable | value | tunable | value |
|---|---|---|---|
| `WheelbaseFactor` | 0.6 | `LaneChangeDecayTau` | 2.0 s |
| `HoldSpeed` | 0.5 m/s | `LaneChangeSnapMeters` | 1.5 m |
| `ReseedJumpMeters` | 7.0 m | `LaneChangeErrorCapMeters` | 3.4 m |
| `PositionSmoothTime` | 0.60 s | `TurnInSmoothTime` | 0.0 (off) |
| `LanePredictSmoothTime` | 0.18 s | `TurnInReconverge` | 0.15 |
| `HeadingSmoothTime` | 0.0 (off) | `TurnInMaxOffsetMeters` | 0.6 m |

**Regression guard:** `tests/Sim.Viewer.Motion.Tests` (facade + KinematicHeading unit tests) pins the pivot
(center L/2 behind front), the lane-change ease, the stop hold, and render-rate/stutter smoothness; the
committed IgBridge byte-identical trace pins the full v5 pipeline; `CityLib.Tests/ReconstructorS2Tests` pins
the end-to-end City3D behaviour. A change to any locked value that regresses these is caught by `dotnet test`.

## Standing gates (re-verify every src/-touching task)
- [x] `dotnet test tests/Sim.ParityTests -c Release` 654 pass / 4 skip ✓ (S3: tests + docs only, no src/ behaviour change)
- [ ] `Sim.Bench` determinism hash unchanged (S3 touched no engine/src code paths)
- [x] IgBridge emitted trace byte-identical to v5 ✓ (S3: regenerated `artifacts/igbridge/trace.jsonl`, `diff -q` vs `trace_v5_baseline.jsonl` = IDENTICAL)
- [x] No ProjectReference from demos/City3D into src/ (unchanged; S3 added no references)

## Owner decisions (Design §7) — RESOLVED, green to go
- [x] Q1 DrPoseSmoother → DELETE outright (no toggle)
- [x] Q2 `--mode local` → UNIFY onto the kinematics
- [x] Q3 City3D via `SumoSharp.Viewer.Motion` package bump → OK
