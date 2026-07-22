# SumoSharp.Viewer.Motion

Portable, engine-agnostic **render-side motion reconstruction** for a SumoSharp-driven simulation.
Targets `net8.0` and `netstandard2.1` — no renderer, no wire transport, no native dependency, so
Unity (Mono/IL2CPP), Godot, and any custom 3D engine can consume it directly.

**What it solves.** The simulation engine advances in discrete steps (as slow as 1 Hz in phase-1
parity mode), far below a display's 60 Hz, and a decoupled renderer typically only sees *sparse*,
adaptively-published samples (lane + arc-position + speed/accel, no `x,y`, no heading). Naively
drawing "the latest sample" teleports vehicles. This package turns that sparse stream into a smooth,
continuous per-frame pose `(x, y, heading)`.

This package is the shipped implementation of
[`SUMOSHARP-VIEWER-DR-SMOOTHING.md`](../../docs/SUMOSHARP-VIEWER-DR-SMOOTHING.md) — read it for the
full design rationale, the bugs each mechanism fixes, and the tunables table (§9). This README is the
condensed reimplementation guide (its §8 and §10).

## What's in the box

- **`DrClock`** — the render clock + sample resolver.
  - `Pump(newestSampleTime, hold)`: advances a strictly-monotonic `renderSim` clock from a
    long-baseline wall\<-\>sim rate fit, immune to per-packet jitter. Never steps backward; holds
    instead. `hold: true` (paused) freezes it. Restart detection re-anchors the fit.
  - `Resolve(history, delay, laneSource)`: given a vehicle's buffered per-vehicle history (newest
    last) and a playout `delay`, returns a `DrState` at render time — extrapolating forward past the
    newest sample, extrapolating backward past the oldest, or interpolating between the two
    bracketing samples. A same-lane bracket is a plain arc-length lerp; a lane-crossing bracket uses
    an **arc-window walk** (`ArcInWindow`) so a turn follows the real (possibly curved) internal-lane
    geometry instead of snapping; a lane-change (sideways) straddle returns *both* bracketing states
    for the caller to resolve independently and Cartesian-lerp.
  - Extrapolation delegates to `Sim.Replication.DrExtrapolation.Arc` — the SAME curve the publisher
    uses for its DR-error publish decision, so viewer prediction and publish-side prediction never
    diverge (see "Reuse DR-error-based publishing" below).
- **`KinematicReconstructor`** — the shared per-vehicle, per-frame render-smoothing facade you run *after*
  `DrClock.Resolve` has produced the render-time `DrState` bracket. It resolves a continuous front pose
  (straddle-aware), applies the spatial look-ahead + anticipation guards + the coarse-feed junction-straddle
  discriminator, and drives a **kinematic no-slip rear-axle (bicycle) reconstruction** (`KinematicHeading`)
  to a rigid-body pose:
  - `Resolve(handle, resolved, lanes, dims, frameDt)` — the full pipeline from a `DrClock.Resolved` bracket
    (used by the DR viewers + IgBridge). Returns a `KinematicReconResult` carrying the vehicle **center**
    (½·length behind the front reference — pivot your box on this), `Z`, and body heading; `Ok == false`
    means skip the vehicle this frame (degenerate/missing geometry).
  - `ResolveFromFront(handle, frontX, frontY, laneHeadingDeg, speed, dims, frameDt, lateralEvent, predictHeadingDeg)`
    — the pose-level entry for a caller that already owns the front reference (e.g. `--mode local`, or a
    snapshot interpolator), with `predictHeadingDeg = null` to skip the look-ahead.
  Stateful per vehicle, deterministic (no `System.Random`, per-entity state, tau/`frameDt`-based gains), and
  render-side only — it never touches parity. It replaces the earlier `DrPoseSmoother` (deleted); its config
  ships at the tuned v5 defaults (see `VIEWER-KINEMATIC-SMOOTHING-TRACKER.md` "Locked defaults").

Not in this package (by design, D2 of the packaging doc): `PoseResolver`, `ILaneShapeSource`, `Pose`,
`DrState` stay in `SumoSharp.Core` — they are dependency-light and shared by the engine's own opt-in
render mode as well as every viewer. `DrClock`/`KinematicReconstructor` consume them; they don't redefine them.

## The reconstruction pipeline

Per render frame, per vehicle:

```
drClock.Pump(newestSampleTime, hold: paused)                 // advance the render clock
resolved = drClock.Resolve(history, delay, laneSource)        // pick/interpolate a DrState bracket
r        = recon.Resolve(handle, resolved, laneSource,        // straddle-aware front + look-ahead +
               (length, width), frameDt)                      //   kinematic no-slip reconstruction
if (!r.Ok) skip this vehicle this frame
emit draw(r.CenterX, r.CenterY, r.Z, r.HeadingDeg, length, width, speed)  // pivot the box on the CENTER
```

`KinematicReconstructor` calls `PoseResolver.Resolve(...ChordHeading, dt: 0)` internally (`Resolve` has
already advanced the arc to render time; `ChordHeading` is the correct chord heading with no lateral
"swing-wide" bow). The returned **center** sits ½·length behind the front reference — pivot your vehicle box
on it. A caller that already owns the front reference (no `DrState`/lane window, e.g. `--mode local`) calls
`ResolveFromFront(...)` with `predictHeadingDeg = null` instead.

### `KinematicReconstructor` internals (as-built — supersedes the earlier `DrPoseSmoother`)

The reconstruction is a **kinematic no-slip rear-axle (bicycle) model** (`KinematicHeading`), not a
capped-correction pose smoother. In brief (full detail + tunables in `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`
and `IGBRIDGE-DECISIONS.md` §5.3):

1. **Straddle-aware front resolve.** A non-straddle bracket resolves one front pose; a lateral straddle
   resolves both bracketing states on their own lanes and Cartesian-lerps them, so the front path is
   continuous through a lane change (no hole in the stream).
2. **Spatial look-ahead + guards.** The front predictor aims at a point a few metres ahead on the *upcoming*
   lane centerline (so a junction turn-in tracks the connecting lane), gated by a per-frame jump-guard and an
   anticipation lead-bound; a **coarse-feed** straddle whose two lane poses diverge widely is treated as a
   junction turn, not absorbed as a lane change.
3. **No-slip rear-axle drag.** The front tows a rear axle that cannot slip sideways (real off-tracking); body
   heading = rear→front, which integrates faceted-polyline kinks away. A stopped vehicle (`speed < HoldSpeed`)
   holds heading (no spin at a red light).
4. **Lane-change ease.** SUMO changes lane instantly (a ~lane-width sideways snap); the cross-track part is
   absorbed into a bounded, critically-decaying lateral error, so the drawn body slides into the new lane over
   ~`LaneChangeDecayTau` instead of teleporting.

All gains are `1 − e^(−frameDt/τ)` (tau-based), so they adapt to any display frame rate; the reconstruction
is deterministic and render-side only. The deleted `DrPoseSmoother`'s capped-correction + heading-tilt
mechanisms are recorded historically in `SUMOSHARP-VIEWER-DR-SMOOTHING.md` §10.2/§10.3.

### Playout delay

Use a **stable, manual** delay (recommended default ~1.0 s for a real-time feed, kept under 1 s).
Do **not** auto-drive it from the measured packet interval — since the sample instant is
`renderSim - delay`, a per-frame delay change is a per-frame position teleport, which showed up as
audible/visible speed pulsing. Interpolation (delay > the packet gap) is strictly smoother than
extrapolation (delay = 0); size the delay against your transport's actual publish cadence.

### Reuse DR-error-based publishing — the biggest lever, and it's not in this package

The single most effective fix for reconciliation snaps lives on the **publish** side, not here: have
your publisher run the same `Sim.Replication.DrExtrapolation.Arc` curve this package's `DrClock` uses
for extrapolation, and only emit a new sample when the true state diverges from that prediction beyond
a tolerance (`Sim.Replication.PublishScheduler` / `DrErrorPublishPolicy`). That bounds this package's
own extrapolation error at the source, so motion stays smooth at low delay with no bandwidth increase.
`KinematicReconstructor` then only reconciles a small residual. See
`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md` for the full design.

### Pitfalls carried forward from the native viewer's own history

- **Under-sampling at junctions is the norm, not an edge case**, at low step/publish rates — implement
  the arc-window bracket (`DrClock.Resolve` does this for you), or turns will visibly snap.
- **Faceted lane geometry + per-segment tangents** produce a stair-stepping heading; `ChordHeading`
  avoids the artifact without densifying geometry.
- **Never let the render clock step backward** on jitter — `DrClock.Pump` holds instead; don't bypass
  this if you reimplement pacing yourself.
- **Clamp deceleration extrapolation at the predicted stop time** — an unclamped constant-accel
  extrapolation drives a stopped/braking vehicle backward past its stop. `DrExtrapolation.Arc` (in
  `SumoSharp.Replication`) already guards this.

### 3D-specific notes

- **Elevation**: sample `ILaneShapeSource.LaneShapeZ`; `Pose` carries `Z`. Interpolate it with the
  same fraction as the arc-length position.
- **Yaw/pitch/roll**: yaw = the navi-heading (convert to your engine's convention, e.g. negate for a
  counter-clockwise-positive engine); derive pitch from the Z gradient along travel; roll ≈ 0 unless
  you fake banking. Use shortest-arc interpolation (or a quaternion slerp) for yaw — never a raw linear
  lerp, which rotates the long way across a 350°→10° wrap.
- **Wheels / steering / brake lights**: `Speed`, `Accel`, and the frame's `Δheading/Δt` are already
  available from the resolved state; no additional data is needed.

## License & disclaimer

Dual-licensed **EPL-2.0 OR GPL-2.0-or-later** (SumoSharp is a derivative of Eclipse SUMO and cannot be
relicensed). Practical read: EPL-2.0 is a weak, file-level copyleft — a proprietary game may link this
package and keep its own source closed, but must keep SUMO-derived files under EPL and publish changes
to *those* files. This is not legal advice; get counsel for commercial use.

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
