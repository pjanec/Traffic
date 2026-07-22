# Viewer motion: dead-reckoning & smoothing across viewers

**Status:** living design note. **Audience:** anyone re-implementing a viewer (2D native, web,
or a future 3D image-generator / "IG") that has to turn the engine's discrete state into smooth
on-screen motion. **Companion docs:** `SUMOSHARP-DEADRECKONING.md` (the DR *wire contract* — §4.2
records, §7/§8 pacing) and `SUMOSHARP-NATIVE-VIEWER.md` (the native viewer's mode/architecture).
This note is the *HOW motion is reconstructed and smoothed*, the reasoning behind it, and the bug
fixes that shaped it. **§8 is the reimplementation checklist for a new (e.g. 3D IG) viewer** —
start there, it points back into the mechanism sections. §6 covers the junction-turn pass (commit
`befd5ed`); **§10 records the later as-built** — position error-smoothing, motion-derived heading tilt,
auto-delay OFF by default, and **dead-reckoning-error-based publishing** (the publish-side root fix;
its own doc `SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`) — and supersedes the §5.4/§5.5 details.
**§11 is the current as-built for the VEHICLE path**: it now smooths via the shared
`Sim.Viewer.Motion.KinematicReconstructor` (kinematic no-slip reconstruction), which **supersedes the
§10.2/§10.3 `DrPoseSmoother` mechanisms** — see `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`.

---

## 1. The core problem

The engine advances in discrete steps. In phase-1 parity mode a step is **1 simulated second**
(`actionStepLength=1`, Euler), and even in faster configs the step rate (1–10 Hz) is far below a
display's 60 Hz. Naively drawing "the latest state each frame" makes vehicles **teleport** 1–10×
per second. Every viewer therefore has to *reconstruct* a continuous pose `(x, y, heading[, z])`
per vehicle per render frame from sparse authoritative samples.

There are **two fundamentally different reconstruction models**, and which one a viewer uses is
dictated by *what data it can see*, not by preference:

| Model | Used by | Input it has | Reconstruction |
|---|---|---|---|
| **Authoritative-snapshot interpolation** | `--mode local` | The full `SimulationSnapshot` every step: exact `(x, y, angle, speed)` per vehicle | Blend the two latest snapshots (render-behind lerp) |
| **Dead reckoning (DR)** | `--mode loopback`, `--mode remote` | Only sparse DDS packets: `lane + arc-pos + posLat + speed + accel + upcoming-lanes` (no `x,y`) | Rebuild `(x,y,heading)` from lane geometry, then interpolate/extrapolate in time |

A 3D IG viewer that consumes the **DDS stream** (the expected case for a decoupled renderer) is a
**DR viewer** and must implement §5. A viewer co-hosted with the engine (has the snapshot) can use
the simpler §4. The two share the *same smoothing philosophy* but almost none of the code.

---

## 2. Data contracts

### 2.1 Local (authoritative snapshot)
`SimulationSnapshot` is a struct-of-arrays (columnar) view, double-buffered by `EngineHost` as an
atomic `(cur, prev)` pair (`host.SnapshotPair`). Per vehicle, per column: `Handles[i]`,
`PosX[i]`, `PosY[i]`, `Angle[i]` (navi-degrees, 0 = North, clockwise), `SpeedExact[i]`,
`Length[i]`, `Width[i]`, plus `Time`. Column index is **not** stable across frames (spawn/despawn
shuffles it) — always match by `Handles[i]`.

### 2.2 DR (DDS stream) — see `SUMOSHARP-DEADRECKONING.md` §4 for the exact wire layout
Three kinds of topic, at three cadences:

- **Geometry, once** (durable / `TRANSIENT_LOCAL`): lane polylines by dense lane handle. This is the
  static network a DR viewer walks. Interface: `ILaneShapeSource` — `LaneLength(handle)`,
  `LaneShape(handle)` (world polyline), `LaneShapeZ(handle)`.
- **Lifecycle / dims, once per vehicle** (durable): physical `Length`/`Width` per handle. Not on the
  per-frame record (bandwidth), so a DR viewer keeps a `handle → (length,width)` registry.
- **Per-frame `VehicleRecord`** (`Sim.Replication/Records.cs`): `LaneHandle`, `Pos` (arc-length along
  the lane), `PosLat`, `Speed`, `Accel`, `LatSpeed`, and `UpcomingLanes` — the current lane first
  then **K=4** lanes ahead (padded with −1). This is deliberately small; there is **no `x,y` and no
  heading** on the wire. K=4 is the render horizon: a vehicle rarely crosses >4 lanes within one
  packet gap (but see §6.1 — at 1 Hz it *can*, and that's exactly the junction case).
- **Status / command** (durable, reverse channel): engine sim-time / paused / speed so a view-only
  remote can reflect real state and grey-out inapplicable controls; commands (pause/restart/speed/
  inject) flow the other way. Orthogonal to motion; mentioned only for completeness.

**Publishing is adaptive** (`PublishScheduler`): every step while accelerating (`|accel|` above a
threshold), every 3rd step while steady. So the packet interval a DR viewer sees is **not constant**
— roughly `1×step` near junctions (accel/decel) and `3×step` on open road. At a 1 Hz step that is a
**1–3 second** inter-arrival gap. Everything in §5 exists to cope with that.

---

## 3. Shared vocabulary

- **navi-degrees**: heading where 0 = North (+Y), increasing clockwise. `VectorFromNavi(deg) =
  (sin, cos)`. Both viewers and `PoseResolver` use this convention end-to-end.
- **arc-pos / `Pos`**: distance travelled along the *current lane's* polyline from its start.
- **`PosLat`**: signed lateral offset from lane centre (sublane / lane-change in progress).
- **render-behind**: deliberately render a little in the *past* so the two samples that bracket the
  render instant already exist — turning prediction (hard) into interpolation (exact).
- **shortest-arc heading lerp** (`LerpAngleDeg`): interpolate angles the short way so `350°→10°`
  rotates +20°, not −340°. Used by *both* viewers; a 3D viewer needs the same for yaw.

---

## 4. Local viewer — render-behind snapshot interpolation

Code: `RunLocal` + `BuildLocalVehicleDraws` in `src/Sim.Viewer/Program.cs`.

Because it has exact `(x,y,angle)` every step, the local viewer never touches lane geometry. It
keeps a **render clock** ~one sim-step behind the newest snapshot and blends the `(prev, cur)` pair:

1. **Clock** (`RunLocal`, ~line 349): `renderClock` advances in *sim-time* driven by the measured
   **wall** interval between snapshot *arrivals* (not the nominal step), so a jittery producer still
   yields smooth playback. On a new snapshot, `renderClock` is nudged toward `prev.Time` (one step
   behind `cur`).
2. **Blend fraction** (`BuildLocalVehicleDraws`, line 1150): `a = (renderClock − prev.Time) / span`,
   clamped to `[0, 1.25]`. The `>1` head-room is a *small bounded extrapolation* so a late-arriving
   next snapshot doesn't cause a once-per-step freeze — it predicts slightly ahead instead.
3. **Per vehicle** (matched by handle): `pos = lerp(prev, cur, a)`, `heading = LerpAngleDeg(prev, cur,
   a)`, `speed = lerp(...)`. A vehicle absent from `prev` (just departed) draws at its raw `cur` pose.
4. **Heading low-pass** (line 1179, `τ = 0.25 s`, `headAlpha = 1−e^(−dt/τ)`): a junction internal
   lane is a coarse ~5-point arc, so even the *interpolated* heading rotates in ~5 discrete bursts; a
   ~0.25 s low-pass on the render heading spreads them into one continuous rotation. Straights
   (constant heading) converge instantly → no lag. Jumps >100° (handle reuse / respawn) snap.

The local path is smooth because it interpolates **real coordinates**; it never sees the faceted
lane geometry as a position source, so it is immune to §6.2.

---

## 5. DR viewer — the dead-reckoning pipeline

Code: `PumpAndBuildVehicleDraws` in `src/Sim.Viewer/Program.cs` (shared by loopback & remote),
`DrClock` in `src/Sim.Viewer.Core/DrClock.cs`, `PoseResolver` in `src/Sim.Core/PoseResolver.cs`.

Per render frame:

```
DrClock.Pump(newestSampleTime, hold: paused)          // advance the render clock (§5.1)
delay = autoDelay()                                     // how far behind to render (§5.4)
for each vehicle with history H:
    resolved = DrClock.Resolve(H, delay, laneSource)    // pick/interpolate a DrState (§5.2)
    state    = resolved.State with { Length, Width }    // add dims from the registry
    pose     = PoseResolver.Resolve(laneSource, state,  // DrState -> (x,y,heading) (§5.3)
                    resolved.Upcoming, dt: 0, ChordHeading)
    if smoothing && resolved.Extrapolated:              // low-pass only the predicted frames (§5.5)
        pose = lowPass(prevPose, pose)
    emit draw(pose, length, width, speed)
```

### 5.1 `DrClock.Pump` — the render clock (`DrClock.cs:90`)
Ported from the web viewer's `HtmlPage.cs` pacing. Turns "packets tagged with a sim-axis timestamp,
arriving on a jittery wall clock" into one smooth, **strictly-monotonic** `renderSim`:

- **Long-baseline rate fit**: `simRate = (sim − firstSim) / (wall − firstWall)`. The growing
  denominator makes it immune to per-packet burst jitter — the clock runs at steady velocity instead
  of racing-and-snapping. A placeholder `simRate = 1.0` is used until the first second of data.
- **Forward-only**: `renderSim` advances toward the estimate but is *never decreased*. If the estimate
  dips (jitter) it **holds** (invisible) rather than steps back (a visible back-jump), counting a
  `BackSteps` health tick. Catch-up is capped at `3× frameDt × simRate`.
- **`hold` (paused)**: freeze `renderSim` so a paused publisher's vehicles stop *immediately* instead
  of coasting ~3 s on extrapolation. Ingest still runs so the rate fit & restart detection stay live.
- **Restart detection**: a sim-time jump *backward* > 2 s (publisher rebuilt at t=0) re-anchors the fit
  and clock, else `renderSim` would strand far ahead and extrapolate wildly ("vehicles racing off").
- **`AvgSampleInterval`**: EMA (α=0.2) of the packet inter-arrival gap, guarded to `(0.001, 5) s`.
  This is the measured cadence the auto-delay (§5.4) sizes itself against.

### 5.2 `DrClock.Resolve` — choose a render-time `DrState` (`DrClock.cs:195`)
`sampleT = renderSim − delay` is the instant we want to show. Given a vehicle's buffered history
(newest last):

- `sampleT ≥ newest` → **extrapolate forward** from the newest packet (`dt` clamped ≤ 3 s).
- `sampleT ≤ oldest` → **extrapolate backward** from the oldest (`dt` clamped ≥ −3 s).
- otherwise **interpolate** between the bracketing packets `a` (last with `t ≤ sampleT`) and `b`:
  - **same lane** → arc-length lerp: `arc = a.Pos + (b.Pos − a.Pos)·f`, `posLat` lerped likewise.
  - **different lane (a straddle)** → **arc-window interpolation** (§6.1 — the fix): express `b`'s
    position in `a`'s *forward* lane window via `ArcInWindow(a, b.lane, laneSource)` (sum lane lengths
    along `a.Upcoming` to the start of `b`'s lane), then interpolate the arc in that single frame:
    `arc = a.Pos + (arcB + b.Pos − a.Pos)·f`. Because the arc is expressed along `a.Upcoming`,
    `PoseResolver` walks the *real* lane polyline (through the curved internal junction lanes) — no
    corner-cut, no snap. If `b`'s lane isn't within `a`'s K=4 window (crossed too far in one gap),
    fall back to extrapolating from `a`.

`Resolve` returns a `DrState` (lane + render-time arc + posLat + kinematics), the `Upcoming` lane
path to walk, and an `Extrapolated` flag (drives §5.5). It does **not** produce `x,y` — that is
`PoseResolver`'s job, deliberately kept separate so the pose maths is shared and testable.

**Guard — no backward driving** (`ExtrapolateArc`, `DrClock.cs:~330`): constant-accel extrapolation
`pos + v·dt + ½·a·dt²` is a *downward* parabola when decelerating; past the stop time it predicts the
car reversing. It is clamped to freeze at the predicted stop. This is why a car braking at a red light
holds instead of creeping backward while its packets are sparse.

### 5.3 `PoseResolver.Resolve` — `DrState → Pose` (`PoseResolver.cs:79`)
Walks the lane window to a world pose. Feed it `dt = 0` from the DR path (Resolve already advanced the
arc to render time). Mechanics:
- **Position**: `SampleForward` walks `upcomingLanes` from the current lane, crossing lane boundaries
  when `arc` exceeds a lane's length, and samples the polyline at `(arc, posLat)`.
- **Heading realism** (`RenderRealism`): `ParityTangent` (lane tangent at the point — byte-identical
  to the parity Angle column), `ChordHeading` (back→front chord over the body length — correct heading
  for long vehicles on curves), `CornerCutCorrected` (chord **+** a swept-path "trucks-swing-wide"
  lateral bow). **The DR path uses `ChordHeading`** — see §6.2. `PoseResolver` is renderer-only
  (it is *not* in the parity path except `ParityTangent`) but lives in `Sim.Core`, so treat it as
  read-only from a viewer and select behaviour via the `realism` argument.

### 5.4 Auto-delay — "always interpolate" (`Program.cs`, both loopback & remote sites) — ⚠️ SUPERSEDED by §10.1 (now OFF by default; it caused speed pulsing)
The delay is the single knob that decides interpolate vs extrapolate. Sitting ~1.5 sample-intervals
behind the newest packet keeps `Resolve` in the interpolate branch through junctions (where the
arc-window makes turns follow real geometry). It is **auto-driven and stabilised** (§6.3):

```
target = clamp(AvgSampleInterval · 1.5, 0.3, 2.0)
if not seeded and AvgSampleInterval > 0:  delay = target; seeded = true   // snap once at startup
else if |target − delay| > 0.05:          delay += clamp(target−delay, ±0.008)  // slew, dead-banded
```

`sampleT = renderSim − delay`, so **any change in delay shifts the sample point and teleports every
vehicle**. Seeding once at startup (nothing is moving yet) skips a long ramp; the dead-band ignores
EMA jitter; the ±0.008 s/frame slew keeps any genuine drift well under one frame of real travel.
A user can uncheck "always interpolate" and drive the delay slider by hand (larger delay = smoother
but laggier).

### 5.5 Extrapolation low-pass (`Program.cs:1050`) — ⚠️ SUPERSEDED by §10.2 (replaced with capped-correction error-smoothing) and §10.3 (motion-derived heading tilt)
Interpolated poses are already exact/continuous, so smoothing runs **only when `Extrapolated`**. A
first-order low-pass toward the raw pose: position `τ = 0.07 s`, heading `τ = 0.06 s`
(`α = 1−e^(−dt/τ)`, `dt` = measured frame time). A >7 m position jump or >50° heading jump *snaps*
(handle reuse / genuine correction) rather than smearing. When not extrapolating, the smoother's state
is kept synced to the current pose so toggling it on later is seamless.

---

## 6. The three bugs fixed in the junction-turn pass (commit `befd5ed`)

Vehicles "jumped laterally" through junctions in the DR viewers. Diagnosis was **data-first** (§7):
the `--trace-veh` harness logged the reconstructed `DRTRACE` pose against the authoritative `AUTHTRACE`
ground truth, revealing **three independent causes**, two of them genuinely lateral.

### 6.1 Lane-transition snap (~2.2 m lateral) → arc-window interpolation
At 1 Hz + adaptive publish, a ~13 m/s vehicle crosses a whole junction (several short internal lanes)
**between two packets**, so its bracketing packets `a,b` sit on *different* lanes every frame. The old
straddle fallback **extrapolated along `a`'s old lane**, overshot the turn, then snapped onto `b`'s
lane. **Fix:** port the web viewer's `arcInWindow` (§5.2) — interpolate the arc *within `a`'s forward
lane window* so `PoseResolver` walks the real internal-lane polylines. A single reconstructed pose that
follows the true curve; no snap. `DrClock.Resolve` gained an `ILaneShapeSource` parameter for the
lane-length walk.

### 6.2 Off-tracking "swing-wide" bow (~1 m lateral) → use `ChordHeading`
`CornerCutCorrected` shifts the body sideways by `length·|Δheading|/2`. On a **faceted 5-point**
internal-lane polyline the per-segment tangent is piecewise-constant, so `Δheading` (and the bow) jumps
at every vertex → a ~1 m lateral hop mid-turn. **Fix:** the DR path renders with `ChordHeading`
(correct chord heading, *no* lateral bow). The swing-wide effect is a deliberate visual approximation,
not parity, and is not worth the artefact on faceted junction geometry. `PoseResolver` itself was **not**
edited (it is in `Sim.Core`); only the `realism` argument the viewer passes changed.

### 6.3 Delay volatility (~4 m *longitudinal*) → seed + slew + dead-band
The first "always-interpolate" implementation recomputed the delay from the jittery `AvgSampleInterval`
EMA **every frame**. Since `sampleT = renderSim − delay`, each wobble (e.g. +0.35 s) teleported every
vehicle (≈ 0.35 s × 13 m/s ≈ 4.4 m backward). **Fix:** §5.4 — seed once, then slew within a dead-band so
steady state is jitter-free.

**Result** (scenario `44-multilane-junction-turn`, vehicle `vN` crossing two internal lanes): max
lateral per-frame deviation **0.089 m**, down from ~2.2 m. A residual *longitudinal* decel/extrapolation
reconciliation remains (a car braking at a light, packets sparse, extrapolation then corrected on the
next packet) — separate from the lateral turn issue and softened by delay + §5.5.

---

## 7. The diagnostic harness (keep it; use it before "fixing")

`--trace-veh <id>` (loopback) logs two lines per frame for one vehicle:
- `AUTHTRACE …` — the **authoritative** snapshot pose (smooth ground truth). Loopback only (remote has
  no local engine).
- `DRTRACE …` — the **DR-reconstructed** render pose, plus the `Extrapolated` flag, resolved lane,
  effective `delay`, and `AvgSampleInterval`.

It is opt-in (no-op without the flag) and intentionally left in the tree for the lane-change work. The
governing rule (see `CLAUDE.md`): **no blind fixing** — record coordinate data and localise the cause
before touching pose maths. The analysis that found §6 split raw per-frame deltas into **lateral**
(perpendicular to heading) vs **longitudinal**, and normalised by the actual `rsim` timestep (headless
runs have no vsync, so a large delta can just be a long frame, not a jump). Reuse that decomposition.

---

## 8. Reimplementing in a future viewer (e.g. 3D IG)

**If it consumes the DDS stream, it is a DR viewer — reuse §5 wholesale.** The clean seam is:

1. Provide an `ILaneShapeSource` from the received geometry (3D: keep `LaneShapeZ` for elevation).
2. Reuse `DrClock` **unchanged** — it is transport-agnostic (feed it the newest sample timestamp and
   its own wall clock) and geometry-agnostic except the `ILaneShapeSource` handed to `Resolve`.
3. Reuse `PoseResolver` for `DrState → (x,y,z,heading)`. Pick `realism`:
   - default **`ChordHeading`** (see §6.2);
   - `CornerCutCorrected` only if you also **densify** internal-lane geometry (higher netconvert
     `internal-link-detail`) so the tangent stops stair-stepping — otherwise you reintroduce §6.2.
4. **Playout delay:** use a STABLE delay, kept **< 1 s** for a real-time feed. Do NOT auto-drive it from
   the packet interval — that fluctuates and pulses speed (§10.1); the auto "always interpolate" now
   defaults OFF. Error-smoothing + DR-error publishing (below) are what make a low, stable delay smooth.
5. **Reuse the §10.2 position error-smoothing** (capped, forward-biased correction — zero lag on smooth
   motion, absorbs a reconciliation snap as a gentle ≤50 % slowdown, never a freeze or a reverse) and the
   §10.3 **motion-derived heading tilt** (lean into any lateral slide, both directions; ~0 on cruise and
   turns). These REPLACE the old §5.5 low-pass — plain scalar maths, transport-agnostic.
6. **Reuse DR-error-based PUBLISHING — the biggest lever, and it lives on the PUBLISH side, not the
   viewer.** The publisher (`Sim.Replication`: `PublishScheduler` + `DrErrorPublishPolicy`) runs the SAME
   `DrExtrapolation.Arc` the viewer does and emits a packet only when the true state diverges from that
   prediction beyond a tolerance — bounding the viewer's extrapolation error at the source, so motion is
   smooth at low delay with **no bandwidth increase** (steady vehicles go silent). A non-DDS transport
   reuses the identical scheduler/policy — they operate on the transport-agnostic `VehicleRecord`, never a
   DDS type. Full design + as-built: **`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`** (essential reading for
   a new transport). The shared curve is `Sim.Replication.DrExtrapolation.Arc` (§10.5) — the viewer's
   extrapolation and the publisher's prediction MUST be the same function.

**3D-specific additions (not in the 2D viewers):**
- **Elevation**: sample `LaneShapeZ`; `Pose` already carries `Z`. Interpolate Z with the same fraction.
- **Model yaw/pitch/roll**: yaw = the navi-heading (convert to your engine's convention); derive pitch
  from the Z gradient along travel; roll ≈ 0 unless you fake banking. Use **shortest-arc** interpolation
  for yaw (§3) or a quaternion slerp — never a raw angle lerp.
- **Wheels / steering / brake lights**: `Speed`, `Accel`, and `Δheading/Δt` are all available per frame;
  no new data needed.
- **Sub-frame motion blur / temporal AA**: the render clock is continuous, so you can query poses at
  arbitrary sub-frame times by adjusting `sampleT`.

**Pitfalls to carry forward (each is a bug that already bit us):**
- **Under-sampling at junctions** (§6.1): the straddle/arc-window case is *not* an edge case at low step
  rates — it is the norm for any fast vehicle through any junction. Implement it, or turns will snap.
- **Faceted geometry + lateral corrections** (§6.2): any position offset derived from a per-segment
  tangent will jump on coarse polylines. Smooth the tangent over a baseline, or don't apply the offset.
- **Publish on prediction-error, not a timer** (§10.4) — the single most important lesson: an interval
  scheduler with delay < packet-gap *guarantees* extrapolation overshoot the moment a vehicle's accel
  changes, and viewer-side smoothing can only repaint that as a slowdown. Fix it at the **publisher**
  (DR-error publishing); then the viewer smoothing only mops up a sub-tolerance residual.
- **Delay must be stable** (§10.1): a fluctuating (auto-driven) delay pulses speed — since
  `sampleT = renderSim − delay`, a per-frame delay change *is* a per-frame teleport. Keep it fixed/off.
- **Monotonic clock only**: never let the render clock step backward on jitter — hold (§5.1).
- **Backward-driving guard** (§5.2): clamp decel extrapolation at the stop time.
- **Correct the smoothing, don't just low-pass** (§10.2/§10.3): position — chase the target with a
  *capped, forward-biased* correction (never freeze, never reverse), not a lag-inducing low-pass on the
  absolute position; heading — derive the lean from *actual render motion* and **negate the `atan2`**
  (navi is clockwise, `atan2` is CCW — adding leans the car the wrong way; this bit us).

---

## 9. Tunables (all in the viewer, none affect parity)

| Parameter | Where | Value | Effect |
|---|---|---|---|
| Playout delay target | `Program.cs` auto-delay | `clamp(avgInterval·1.5, 0.3, 2.0)` s | interpolate vs extrapolate; latency vs smoothness |
| Delay slew / dead-band | `Program.cs` auto-delay | `±0.008` s/frame, `0.05` s band | how gently delay adapts without teleporting |
| DR position smoothing τ | `Program.cs:1054` | `0.07` s | low-pass on *extrapolated* position |
| DR heading smoothing τ | `Program.cs:1055` | `0.06` s | low-pass on *extrapolated* heading |
| DR smoothing snap thresholds | `Program.cs:1061,1070` | `7` m, `50°` | correction/respawn → snap not smear |
| Local heading low-pass τ | `BuildLocalVehicleDraws:1143` | `0.25` s | spreads faceted-junction heading bursts |
| Local blend clamp | `BuildLocalVehicleDraws:1152` | `[0, 1.25]` | bounded prediction for a late snapshot |
| DR realism | `PumpAndBuildVehicleDraws` | `ChordHeading` | heading model; avoids off-tracking artefact |
| Upcoming-lane horizon K | `Records.cs:UpcomingLanes.Count` | `4` | how far the arc-window can bridge a straddle |
| Extrapolation clamp | `DrClock.ExtrapolateArc` | `±3` s, stop-time | bounds prediction; no backward driving |
| Rate-fit baseline | `DrClock.Pump` | `1.0` s | when the long-baseline `simRate` goes live |

## 10. As-built updates (2026-07-15) — supersedes parts of §5.4/§5.5

The DR smoothing evolved substantially after the junction-turn pass (§6). The mechanisms below are the
**shipped** state (on `main`); where they conflict with §5.4/§5.5, these win. Companion docs:
`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md` (the root-cause publish-side fix) and
`SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md` (the lateral-straddle reconstruction).

### 10.1 Auto-delay ("always interpolate") is OFF by default — supersedes §5.4
Auto-driving the delay from the measured packet interval made the delay **fluctuate**, and since
`sampleT = renderSim − delay`, that visibly **pulsed** rendered vehicle speed. It now defaults **off**;
the viewer uses a **stable manual delay** (default 1.0 s, slider-adjustable). The checkbox remains for
the rare latency-minimising case. (`Program.cs`, both loopback + remote sites.)

### 10.2 Position error-smoothing (capped correction) — supersedes the §5.5 low-pass
Instead of an exponential low-pass on extrapolated poses, the rendered position **chases the DR target
with a CAPPED correction speed** (netcode "projective" style), always on:
- Smooth constant-speed motion passes through with **zero lag** (per-frame move == real travel).
- A reconciliation **snap** (extrapolation overshoot corrected when a packet lands) is **absorbed over a
  few frames** instead of teleporting.
- **Forward-biased:** a backward correction (the common case) is a gentle **~50 % slowdown** (forward
  move floored at `0.5·trueStep`, capped at `trueStep + 6 m/s`), never a freeze and never a reverse.
  Lateral catch-up is capped (~4 m/s) both ways. A >7 m gap snaps (respawn / handle reuse).
Decomposed in the lane-heading frame; runs in `PumpAndBuildVehicleDraws` after `Resolve`/`PoseResolver`.

### 10.3 Motion-derived heading tilt — supersedes the straddle chord heading
Heading = the lane-forward heading (`PoseResolver` `ChordHeading`, or the straddle's lane-lerp) **rotated
by the tilt of the vehicle's ACTUAL per-frame render motion**: decompose the final render displacement in
the lane-heading frame, `tilt = atan2(perp, along)`, **subtract** it (navi is clockwise, `atan2` is CCW —
adding leans the wrong way), clamp ±25°, then the heading low-pass (τ = 0.18 s, >100° snaps). This makes
the car **lean into any lateral slide uniformly** — outbound *and* return lane changes — and is ~0 on
straight cruise and on turns (motion follows the lane, so no spurious tilt). It replaces the earlier
straddle-only chord heading, which left the *return* change (rendered via extrapolation, not the straddle
bracket) sliding flat.

### 10.4 DR-error-based publishing is the ROOT fix for reconciliation snaps
The frequent micro-slowdowns and the "pass hiccup" were **extrapolation overshoot**: the publisher kept a
"steady" vehicle on a 3 s cadence, but constant-accel extrapolation overshoots when the vehicle's accel
changes in that gap; error-smoothing (§10.2) can only convert the correction into a slowdown. The fix is
**publish-on-prediction-error** (publisher runs the same `DrExtrapolation.Arc` the viewer does and sends
only when the true state diverges > tolerance) — bounding the viewer's overshoot at the source so the
motion is smooth at low delay with no bandwidth increase. Full design + as-built:
`SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`. With it, §10.2/§10.3 handle only the small residual.

### 10.5 One source of truth for the DR curve
`DrClock.ExtrapolateArc` now delegates to `Sim.Replication.DrExtrapolation.Arc`, so the viewer's
extrapolation and the publisher's DR-error prediction are byte-identical.

---

## 11. As-built update (2026-07) — the VEHICLE path now smooths via `KinematicReconstructor` (supersedes §10.2/§10.3)

The **vehicle** render path in every viewer — 2D Raylib (`--mode loopback`/`remote`/`local`), 3D City3D
(Godot), and the IgBridge feed — no longer uses `DrPoseSmoother`. It now runs the shared
`Sim.Viewer.Motion.KinematicReconstructor` facade: a **no-slip rear-axle (bicycle) reconstruction** with a
low-passed **lane-heading predictor**, a **spatial look-ahead** onto the connecting lane, an anticipation
**lead-bound**, a **coarse-feed junction-straddle discriminator**, and a **lane-change ease** (a lane-width
sideways snap is absorbed into a bounded, decaying lateral error instead of teleporting). The vehicle box is
now pivoted on the returned **center** (½·length behind the front reference), fixing a latent forward offset.

This **supersedes §10.2 (capped-correction position error-smoothing) and §10.3 (motion-derived heading tilt)
on the vehicle path** — those were the `DrPoseSmoother` mechanisms, which is deleted. §10.2/§10.3 remain here
as the historical record of what the vehicle path did before this change (and still describe the general
DR-smoothing philosophy). §10.1 (stable manual delay), §10.4 (DR-error-based publishing), and §10.5 (one DR
curve) are unchanged and still apply. Any non-vehicle path (e.g. pedestrians) keeps its own reconstructor.

**The full design, tunables, and parity/determinism argument live in
[`VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`](VIEWER-KINEMATIC-SMOOTHING-DESIGN.md)** (tasks +
success conditions: `VIEWER-KINEMATIC-SMOOTHING-TASKS.md`; tracker: `VIEWER-KINEMATIC-SMOOTHING-TRACKER.md`,
whose "Locked defaults" table lists the shipped config). The reconstruction internals mirror
`IGBRIDGE-DECISIONS.md` §5.3. Regression is guarded by `tests/Sim.Viewer.Motion.Tests`,
`CityLib.Tests/ReconstructorS2Tests`, and the IgBridge byte-identical trace.
