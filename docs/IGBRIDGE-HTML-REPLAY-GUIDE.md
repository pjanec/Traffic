# How the smoothed HTML replays are generated (reproduction guide)

**Goal of this doc.** Explain exactly how the side-by-side HTML replays (the ones showing *smooth, realistic
vehicle kinematics*) are produced, so another session can reproduce the result. If you're trying to rebuild
this and the motion looks wrong (rear "skidding", corner snaps, cars off-lane), **§5 (the gotchas) is almost
certainly why** — start there. The fuller method + the general 2D-viz recipe live in
`IGBRIDGE-METHODOLOGY.md` (§5 metrics, §10 the visualization recipe); this doc is the focused
"how the replay is made and why it's smooth" distillation.

Two different HTML-replay generators exist in the repo — this is about the **first**:
1. **IgBridge side-by-side** (`Sim.IgBridge.Host/VizExport.cs`) — raw-engine vs kinematically-smoothed, the
   smooth one. **← this doc.**
2. `Sim.Viz` `SceneGen`/`WriteHtml` — the demo gallery `replay.html` (engine-FCD based). Different producer,
   same `template.js` front-end.

Run it:
```
dotnet run --project src/Sim.IgBridge.Host        # -> artifacts/igbridge/sidebyside.html
# knobs (env): IGBRIDGE_SCENARIO, IGBRIDGE_FEED_HZ (1=coarse feed), IGBRIDGE_EMIT_HZ, IGBRIDGE_BUS_IDS, ...
```

---

## 1. The pipeline (five stages, producer → player)

```
[1] Engine @ fixed 10 Hz            IgBridgeRunner.Tick()      per-entity DR ring buffers (lane+arc+speed+UPCOMING lanes)
        │  (deterministic sim-time; no wall clock)
        ▼
[2] Reconstruction  (the smoothing) IgBridgeSession.EmitVehicles
        │  DrClock.ResolveAt(history, tau, lanes)   -> a DrState at query time (interpolate/extrapolate)
        │  KinematicReconstructor.Resolve(...)       -> smooth CENTER (x,y,z) + body heading
        ▼
[3] Emit IG-native samples @ 20 Hz  IgSample new/upd/del  [id, t, model, x, y, z, headingDeg]
        │  (all entities at ONE coherent tau per instant; dense, ON a smooth curve)
        ▼
[4] "Dumb" IG replay                 FakeIg (2-sample interpolator + playout delay)  -> the poses an IG shows
        │  VizExport.BuildScene samples FakeIg at a fixed fps over a window
        ▼
[5] Self-contained HTML              template.html + template.js (Canvas 2D)  -> sidebyside.html
```

The load-bearing idea: **all the smoothing is baked into stage [2]/[3]** — the emitted stream is dense and
lies on a smooth curve, so the "dumb" 2-sample interpolation in [4] and the Catmull-Rom in the player [5]
between those points already look right. The player does **no** smart smoothing; it just interpolates points
that are already good. **If you feed raw engine samples straight to the player, it looks bad** — that's the
whole point of stage [2].

---

## 2. Where the algorithms come from (and yes — they use the engine's prediction DR data)

The smoothing is **`src/Sim.Viewer.Motion/KinematicReconstructor.cs`** (shared with the 2D/3D viewers). Per
entity, per query instant it:
- resolves a **continuous front** from the DR history (`DrClock.ResolveAt`), straddle-aware (junction
  lane-cross / lane change);
- runs a **no-slip rear-axle (bicycle) model** so the body pivots at the rear like a real car (rear tracks
  inside on a turn) — this is what makes it look like a *vehicle*, not a sprite sliding along a line;
- runs a **spatial look-ahead** that aims the front down the **upcoming connecting lane** so it anticipates a
  junction turn instead of turning in late;
- eases lane changes (absorbs SUMO's instant ~3.2 m lateral snap into a bounded decaying error);
- emits the vehicle **CENTER** + body heading.

**The look-ahead IS the engine's prediction DR data.** `TryLookAheadHeading` advances the DR arc position
`Pos` by `LookAheadMeters` and re-resolves the pose down the **`UpcomingLanes`** — the forward-path lane
window the engine ships as part of its dead-reckoning record (`VehicleRecord.Upcoming`, K=4 lanes ahead).
That is exactly "using the prediction data the engine produces": the front rides the connecting-lane
centerline through a junction because the reconstruction *knows the upcoming lanes*. Set `CoarseFeed = true`
when the feed rate is below the core rate (junction-straddle discriminator). This is the same class both real
viewers use — so reproducing it = reusing this class, not re-deriving the math.

---

## 3. How time is driven (this is where "smooth and nice" comes from)

Three clocks, each turning discrete data into continuous motion:

1. **Producer emit cadence (stage [3]).** `IgBridgeSession.Advance()` drains every emit instant that has
   become resolvable: `horizon = SimTime − LookaheadSeconds; while (nextEmit <= horizon) { EmitInstant(nextEmit); nextEmit += 1/EmitHz; }`.
   All entities are reconstructed at the **same `tau`** each instant (time-coherent scene). `LookaheadSeconds`
   (~one core tick, 0.1 s; at a coarse feed ~one feed interval) keeps the query *behind* the newest buffered
   sample so `DrClock.ResolveAt` stays in the **interpolate** branch (arc-window junction turns) rather than
   extrapolating a turn. Emit at **20 Hz** — dense enough that linear interp between samples is negligible.
2. **IG playout (stage [4], `FakeIg`).** The "dumb" IG plays at `tPlay = clock − DelaySeconds` (0.75 s) and
   linearly interpolates the **two emitted samples bracketing `tPlay`** (shortest-arc for heading). The delay
   is what lets it *interpolate* instead of *extrapolate a turn* — **delay must be ≥ one emit interval + jitter**.
   `VizExport.BuildScene` samples this FakeIg at a fixed **fps** (15) over a **window** (startT 20 s → endT
   80 s) to produce the per-frame data.
3. **Player render clock (stage [5], `template.js`).** Interpolates between the fixed-fps frames with a
   **centripetal Catmull-Rom** on position (clamped to each segment's endpoint bbox so it can't overshoot onto
   a sidewalk) and shortest-arc interpolation on heading. A monotonic render clock advances in real time.

Net: engine 10 Hz → reconstruction 20 Hz (smooth) → IG delay-interpolated → player Catmull-Rom at 60 fps.
The realism is in stage [2]; every later stage just interpolates points already on the smooth curve.

---

## 4. The data on the wire (schemas you must match)

- **Emitted sample** — `src/Sim.IgBridge/IgSample.cs`: `Kind ∈ {New, Upd, Del}`, `Id`, `T` (sim time — the
  sole timing authority), `Model ∈ {Car, Ped}` (meaningful on `New`), `X, Y, Z, HeadingDeg` (navi-degrees:
  0=north, clockwise). `New` = entity's first sample; `Upd` = pose thereafter; `Del` = history frozen
  (despawned) and the query time passed it. Stable id per entity.
- **Scene payload vehicle row** — `VizExport.BuildScene` writes per frame, per slot:
  `[x, y, headingDeg, length, width]` (a **5-tuple**; a 3-tuple is City3D-FCD, a 4-tuple is panic-evac fear —
  the arity is the type tag). Plus `network` (lane polylines + junctions), `view` (crop bbox),
  `vehIds` (slot→id, for click-to-identify), and the crucial **`useDataHeading: true`** flag (see §5).
  Injected into `template.html` by replacing `/*REPLAY_DATA*/` (JSON) and `/*TEMPLATE_JS*/` (the JS file).

---

## 5. The gotchas — why a reproduction attempt looks wrong

Almost every "it doesn't look as smooth as yours" comes from one of these (each was a real bug we hit):

1. **Draw the EMITTED heading, NOT the path tangent.** This is the #1 cause of "the rear skids". `template.js`
   must orient the box from the emitted body heading (`scene.useDataHeading = true`, the branch at
   `template.js ~line 480`), NOT from the tangent of the front-anchor path. The tangent is jittery (median
   ~158 yaw-accel reversals/turner) and makes the rear look like it's steering. Set `useDataHeading` in the
   scene payload and honor it in the player.
2. **Feed the reconstruction, not the raw engine stream.** The smooth scene must be built from the
   `KinematicReconstructor` output. If you point the player at raw engine FCD (x,y,angle @ 10 Hz), you get
   junction facet-snaps + instant lane-change teleports — that's literally the "raw" scene in the side-by-side.
3. **The look-ahead needs the upcoming-lanes.** If you don't pass `UpcomingLanes` (the engine's DR prediction)
   into the reconstruction, the front turns in late/overshoots at junctions. This is the prediction data doing
   the work.
4. **Draw the CENTER, at the true per-vehicle length.** The reconstruction emits the vehicle center (the IG
   pivot). Draw the box centered on it (or, for a front-anchored renderer, shift by ½·length). Emit the true
   length so long vehicles draw long; a wrong pivot puts the box ½·length off.
5. **Interpolate, don't extrapolate.** Keep a playout delay ≥ one emit interval so the player always brackets
   the render instant with two real samples. Use **shortest-arc** heading interpolation (350°→10° is +20°),
   and clamp position interpolation to the segment endpoints (no spline overshoot onto sidewalks).
6. **Coarse feed:** if the engine/feed rate is below the sample rate (e.g. 1 Hz), set `CoarseFeed = true` and
   size the lookahead ≈ the feed interval, or live vehicles get retired / turns ride between lanes. (This is
   the validated "1 Hz feed still looks smooth" regime.)

---

## 5b. Pedestrians (the other half of the scene — same principle, a different reconstructor)

Peds get the SAME "all smoothing baked into the reconstruction, only dumb interpolation in the player"
treatment as vehicles — but through a **different reconstruction path**, because ped motion is *analytic*
(a walked path / activity timeline), not lane-arc dead-reckoning. Get this wrong and peds move like a
**"caterpillar"**: a fast lurch once per sim tick, then slow — the ped analogue of the vehicle facet-snap.

- **Source:** the ped replication WIRE — `LiveCitySim.PedSource` (an `IPedReplicationSource`), the same
  in-memory wire the remote/DDS ped path consumes. (Peds do NOT ride the vehicle `VehicleSource`.)
- **Reconstructor:** `Sim.Pedestrians.Lod.PedRemoteReconstructor` (wraps a `HeadlessIg`). Per id,
  `HeadlessIg.ReconstructSample(id, renderTime)` evaluates the ped's **PathArc / ActivityTimeline leg as an
  analytic function of time** — sampleable at ANY instant, no snapshot brackets, no tick-boundary kink. This
  is what makes peds continuous between the (2 Hz) sim ticks.
- **Playout clock:** the reconstructor keeps its own: `RenderTime = latestServerTime − PlayoutDelay`
  (`DefaultPlayoutDelaySeconds` = 0.15 s), advanced by `Pump(latestServerTime)`. To render peds AT instant
  `tau`, call `Pump(tau + PlayoutDelay)` so `RenderTime == tau`.
- **Read poses:** `foreach (id in recon.KnownIds) if (recon.TryGetRenderPose(id, out pos, out visible, out
  animTag) && visible) …`. LOD colour = `recon.Ig.ModelOf(id) == FreeKinematic` (promoted ORCA) else the
  `animTag` (walk vs paused).
- **Disc payload row:** `[x, y, radius, kind]` (a 4-tuple; `kind` = the LOD colour index). The player's
  `interpolatedDiscs` linearly interpolates disc positions between frames (and holds an absent slot), so —
  exactly like vehicles — the emitted points must already lie on the smooth curve.

### The ped gotchas
1. **Reconstruct off the wire; do NOT call the LOD manager's `PositionOf(id, t)` directly at a lagged/arbitrary
   time.** `PositionOf` reads the ped's CURRENT arc, only valid around the latest tick; at a lagged `tau`
   (e.g. behind by the vehicle playout delay) it evaluates before the arc's start → clamp-then-jump = the
   caterpillar. And for **promoted (FreeKinematic/ORCA) peds `PositionOf` ignores `t` entirely** (returns the
   live ORCA position) → held-then-jump. `ReconstructSample(id, RenderTime)` is time-correct for every regime.
2. **One coherent `tau` per frame.** Pump the ped reconstructor to the SAME render instant the vehicles
   resolve against — cars: `DrClock.ResolveAt(history, tau, lanes)`; peds: `pedRecon.Pump(tau + pedDelay)` so
   its `RenderTime == tau`. Otherwise cars and peds drift out of lockstep on turns/crossings.
3. **Colour by the emitted LOD regime, not a guess.** grey = low-power walk, orange = promoted full-ORCA,
   yellow = paused — read it from `Ig.ModelOf`/`animTag`, so a promotion/demotion is visible as a colour flip.

### Worked example (the faithful live-city replay)
`src/Sim.Viz/SceneGen.cs → BuildLiveCityDemo`: one loop, one `tau` per emitted frame, cars via
`DrClock.ResolveAt` + `KinematicReconstructor` (center + emitted heading, §2/§5), peds via
`PedRemoteReconstructor.Pump(tau + pedDelay)` + `TryGetRenderPose`. Both write into the same `template.js`
scene (`useDataHeading` for the 5-tuple cars; 4-tuple discs for peds).

---

## 6. Files + entry points (the map to copy from)
- **Producer/host:** `src/Sim.IgBridge.Host/Program.cs` (drives it), `src/Sim.IgBridge.Host/VizExport.cs`
  (`WriteSideBySide` / `BuildScene` — the scene payload + the `useDataHeading`/`vehIds`/5-tuple emit).
- **Reconstruction (the smoothing):** `src/Sim.Viewer.Motion/KinematicReconstructor.cs` (+ `KinematicHeading.cs`),
  `src/Sim.Viewer.Motion/DrClock.cs` (`ResolveAt`), `src/Sim.Core/PoseResolver.cs`.
- **Emit + IG:** `src/Sim.IgBridge/IgBridgeSession.cs` (`EmitVehicles`/`EmitPeds`), `src/Sim.IgBridge/IgSample.cs`,
  `src/Sim.IgBridge/FakeIg.cs` (the 2-sample interpolator + playout delay + optional zero-latency extrapolate).
- **Player (shared front-end):** `src/Sim.Viz/template.html` (markers) + `src/Sim.Viz/template.js`
  (`interpolatedVehicles` Catmull-Rom + the `useDataHeading` branch + `drawVehicle`).
- **Diagnose motion numerically:** `IGBRIDGE_DEBUG_VEH=v49 dotnet run …` dumps per-vehicle CSV
  (`artifacts/igbridge/debug_v49.csv`, columns `t,prX,prY,prH,smX,smY,cX,cY,cH,rearX,rearY,speed,laH`); the
  metric suite + method are in `IGBRIDGE-METHODOLOGY.md` §5/§10. Slider-time = real − 20 s (the render window).

---

## 7. TL;DR for the other session
Reuse `KinematicReconstructor` fed with the engine's DR history **including upcoming-lanes** (that's the
prediction that anticipates junctions); emit its **center + emitted heading** as dense (~20 Hz) time-coherent
samples; play them back with a small **interpolation delay** (never extrapolate a turn); and in the player
**draw the emitted heading, not the path tangent** (`useDataHeading`) and Catmull-Rom the positions. The
smoothing is in the reconstruction, not the player — the player only interpolates points already on a smooth,
prediction-anticipated curve.

**Peds:** same principle, different reconstructor (§5b). Reconstruct off the ped WIRE with
`PedRemoteReconstructor` (`Pump(tau + pedDelay)` → `TryGetRenderPose`), NOT the LOD manager's `PositionOf`
at a lagged time (that's the "caterpillar"). One coherent `tau` per frame for cars and peds together.
