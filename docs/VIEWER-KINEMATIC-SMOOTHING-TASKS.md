# Viewer kinematic smoothing — TASKS (stages + success conditions)

Design reference: `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md` (sections cited per task). Tracker:
`VIEWER-KINEMATIC-SMOOTHING-TRACKER.md`. Every `src/`-touching task **captures the standing-gate baseline
fresh** (`dotnet test Traffic.sln` counts + `Sim.Bench` determinism hash) at its start, and restores it at its
end — other sessions may be editing the engine concurrently (Design §4).

Global invariants (every task): `Sim.Core` untouched; parity 654/4 byte-identical; determinism (no
`System.Random`, per-entity state); no ProjectReference from `demos/City3D` into `src/`.

---

## Stage S0 — baseline capture + shared facade extraction

### T0.1 — Capture the DrPoseSmoother metric baseline (both viewers)
Design §5. Run the Raylib loopback with `--trace-veh` on a junction-turn + a lane-change vehicle
(`scenarios/44-multilane-junction-turn` and a lane-change scenario); record the IgBridge metric suite
(yaw-accel reversals, no-slip, offset-from-centerline, max lateral per-frame deviation) for the **current
`DrPoseSmoother`** path. Do the same headlessly for City3D via `CityLib.Tests` fixtures.
**Success:** a committed `docs/` note (or test fixture constants) with the baseline numbers, so S1/S2 can prove
"beats baseline".

### T0.2 — Extract `KinematicReconstructor` into `Sim.Viewer.Motion`
Design §1.1. Move `TryLookAheadHeading` + the straddle-lerp + look-ahead-guard/lead-bound + `KinematicHeading`
call out of `IgBridgeSession` into a new `Sim.Viewer.Motion/KinematicReconstructor.cs` (verbatim logic).
Expose the tuned v5 config; default = v5 values. `IgBridgeSession.EmitVehicles` calls the facade.
**Success:** (a) `dotnet build` green; (b) **IgBridge emitted trace byte-identical to v5** (two-run + vs a saved
v5 trace); (c) IgBridge + Motion unit tests green; (d) `dotnet test` 654/4, `Sim.Bench` hash unchanged.

### T0.3 — Facade unit tests
Add `tests/Sim.Viewer.Motion.Tests` cases for `KinematicReconstructor`: straight track (center ½·length behind
front, heading stable), a scripted junction straddle (follows arc, offset < 0.6 m), a scripted lane-change
straddle (eases over ~1.2–1.5 s, bounded yaw), a stop (no creep), coarse-feed vs dense-feed straddle
classification.
**Success:** new tests green; assertions check the real numeric conditions (not vacuous).

---

## Stage S1 — Raylib 2D swap (src/, has the trace harness)

### T1.1 — Confirm Raylib box pivot; wire the facade
Design §2.2. Determine whether the Raylib renderer draws the vehicle box centered on the pose point; if so,
feed it the facade **center** (front→center fix). Replace the straddle branch + `smoother.Smooth` in
`RenderHelpers.cs` with one `_recon.Resolve(...)`; `CoarseFeed=true`.
**Success:** loopback + remote build & run; vehicles render; no exceptions over a full `09-traffic-light` +
`44-multilane-junction-turn` run.

### T1.2 — Metrics beat the T0.1 baseline
Design §5.2. `--trace-veh` on the same vehicles: yaw-accel reversals **median 0**; max lateral per-frame
deviation ≤ the DrPoseSmoother baseline; offset-from-connecting-lane-centerline through a junction improved; a
lane change now eases (measured duration ~1.2–1.5 s, was a skip/snap); no stopped-vehicle creep.
**Success:** a committed before/after table; every metric ≥ baseline (no regression), lane-change + junction
strictly better. Standing gates green.

### T1.3 — Unify `--mode local` onto the kinematics
Design §1.3. Route `BuildLocalVehicleDraws` through the facade's pose-level entry (`ResolveFromFront`,
`predictHeadingDeg=null`); draw the center. Lane changes ease via the step-based detector.
**Success:** local mode renders; a lane change eases (was a heading low-pass only); no creep at a stop;
determinism.

### T1.4 — Delete `DrPoseSmoother`
Design §1.2. Remove `src/Sim.Viewer.Motion/DrPoseSmoother.cs`, its tests, and every remaining call site.
**Success:** `dotnet build` + `dotnet test` green with the file gone; no references remain (`grep`).

---

## Stage S2 — City3D 3D swap (demos/, package-consumed)

### T2.1 — Package the facade; bump City3D's local feed
Design §2.1. `dotnet pack` `SumoSharp.Viewer.Motion` (now containing `KinematicReconstructor`) to
`demos/City3D/local-nuget/`; restore.
**Success:** `demos/City3D/build.sh` packs + restores; CityLib compiles against the new package; the
`SumoSharp.*`-only local-feed pin still holds.

### T2.2 — Reconstructor: straddle-lerp + facade + center-pivot
Design §2.1. Remove the straddle `continue` (`Reconstructor.cs:90-96`); replace `_smoother.Smooth` (`:118`)
with `_recon.Resolve(...)`; feed `CarTransform` the **center**; keep z + `ComputePitchRad`. `CoarseFeed=true`,
delay stays 0.4 s.
**Success:** `CityLib.Tests` green including new cases: lane change eases (was skipped), box center is ½·length
behind the front reference, junction turn follows the arc, no back-jumps, no creep at a stop, determinism
(two runs identical instance transforms).

### T2.3 — Godot headless smoke
Design §5.4. `fetch-godot.sh` + `godot --headless` scene smoke on `09-traffic-light` (local) — N frames, log,
quit, no exceptions; optional Xvfb software screenshot committed to the PR description (not the repo).
**Success:** headless scene runs clean; screenshot (if produced) shows cars on roads.

---

## Stage S3 — tune, prove, document, sign-off

### T3.1 — Render-rate + stutter robustness
Design §3. Headless facade sweep at 30/60/144 Hz `frameDt` and an injected long-frame; assert smoothness
(bounded yaw-accel) and no reseed spikes across all.
**Success:** metrics stable across rates; committed as a regression test.

### T3.2 — Lock defaults + commit the metric regression check
**Success:** the before/after metric tables (both viewers) committed; a headless regression test guards the
key metrics; tunables documented (mirror DR doc §9 / IgBridge methodology §8).

### T3.3 — Docs + owner desktop sign-off
Update `SUMOSHARP-VIEWER-DR-SMOOTHING.md` (note the kinematic smoother supersedes §10.2/§10.3 on the vehicle
path) and `DEMO-CITY3D-*`. Owner runs Godot 3D + Raylib 2D on a desktop and confirms motion looks right.
**Success:** docs updated; owner sign-off recorded; decide `DrPoseSmoother` deletion (Open Q1).
