# IgBridge ↔ IG binding — DECISIONS & design addendum

Records the owner's answers to the design's §12 open questions and the survey findings that refine
scope. This is the **HOW addendum**: it amends `IGBRIDGE-DESIGN.md` where the answers
narrow or correct it. Read the requirements (`IGBRIDGE-REQUIREMENTS.md`) and design first;
this doc is the delta, not a restatement.

> **Naming.** The binding — the external **.NET 6 host** that embeds SumoSharp, ticks the core at
> 10 Hz, and feeds the proprietary IG — is called **IgBridge** everywhere: docs (`IGBRIDGE-*` family)
> and code (`Sim.IgBridge` producer lib, `Sim.IgBridge.Host` verification console).

---

## 1. Locked decisions (owner answers to design §12 + the two confirmations)

| # | Question | Decision |
|---|---|---|
| **Q1** | Orientation on the wire | **`x, y, headingDeg` (navi-degrees).** No quaternion, no z, no pitch/roll from us. Real IgBridge converts to the IG-native quaternion later; the **IG does conformal ground-clamping itself**. FakeIg interpolates heading with **shortest-arc** angle interp (never a raw lerp). |
| **Q2** | Emit cadence & delay | The IG is a **standalone async network process** fed timestamped samples; it makes **no delay assumptions**. Its playout **delay is the IG's own config: ~500 ms–1 s** (longer harms human-in-the-loop perceived latency). Real IgBridge emits at its own (variable) frame rate. **IgBridge's job: emit samples whose timestamp correctly correlates with the position increment**, so the IG's DR never jumps. PoC emits at a **fixed, deterministic cadence** (default 20 Hz) for a reproducible trace; delay is a FakeIg replay knob swept over 0.5–1.0 s. |
| **Q3** | Despawn / teleport | The IG has a **lifecycle API**: `entity-created(id, model, initialPos)`, `entity-removed(id)`, `entity-updated(sample)`. The IG smooths **small** teleports via its DR and does an **immediate jump above a spatial threshold**. So IgBridge emits three record kinds, not a bare sample stream; genuine discontinuities need no special smoothing (let the IG threshold handle them, or re-key via remove+create). |
| **Q4** | Input stream | **(a) direct 10 Hz engine state.** DDS-DR wire (b) deferred (may later drive the City3D viewer off the full DR protocol). |
| **Q5** | Terrain | **The IG owns its terrain.** Production will use a road net matching the IG's terrain. **No City3D, no z, no 2D→3D.** IgBridge emits **planar `x, y, headingDeg` only.** (This also matches the survey finding that City3D has *no* procedural terrain field — ground-z there comes only from lane `ShapeZ`.) |
| **Q6** | Targeting | Producer lib **`Sim.IgBridge`** multi-targets `netstandard2.1;net6.0;net8.0` (proves the .NET 6 embed + Unity/IL2CPP reuse). Verification console **`Sim.IgBridge.Host`** is `net8.0`. **`Sim.Pedestrians` is retargeted `netstandard2.1;net8.0`** (same as the vehicle stack) so peds are reusable from .NET 6 / Unity too. |
| **C1** | Lane-change ease location | Lives in **`Sim.Viewer.Motion`**, not IgBridge — no binding-private smoothing math. Fixing it fixes the 2D/3D viewers too. Outside the parity path → goldens & `dotnet test` stay byte-identical. |
| **C2** | Metrics gate | Objective smoothness metrics (yaw-rate, yaw-jerk, lateral accel, C1 gap, lane-change duration) raw-vs-reconstructed, committed as a regression check. Concrete targets in `IGBRIDGE-TASKS.md`. |

### Scope changes these answers force (vs the original design)
- **R5 (2D→3D terrain) is dropped.** IgBridge emits planar pose + heading; the IG clamps to its own
  ground. Removes the terrain-height/pitch/roll work entirely.
- **Lifecycle is an explicit 3-verb protocol** (created/removed/updated), replacing the design §8
  "the 4-tuple has no teleport bit" framing. The IG's own jump-threshold handles genuine
  discontinuities; IgBridge just emits create/remove correctly and does **not** smooth a supra-threshold
  jump as motion.
- **Orientation is scalar heading**, so the "quaternion vs yaw-only" branch collapses to yaw-only.

### Revision (post-Q5): z IS on the wire, for multi-level disambiguation
The owner later clarified the IG **needs z** for **multi-level roads and tunnels**: where a bridge and a
tunnel share `(x, y)`, only z places the entity on the correct deck. This does **not** revive
terrain-following (still the IG's job) — z is for *disambiguation*, not ground contact. So the wire is
`[id, x, y, z, headingDeg, t]`. Sourcing (no engine-core change on the direct-10 Hz path):
- **Vehicles:** z = `PoseResolver.Resolve`'s `Pose.Z`, sampled from `NetworkLaneSource.LaneShapeZ` — real
  elevation on a 3-D net, `0.0` on a flat net (the box). Flows automatically; already available.
- **Peds:** `z = 0` for now — the crowd is 2-D (holonomic). Multi-level ped z needs a future surface-z
  mapping (see open items).

**Open items (do NOT touch engine core — other sessions are editing lane-change / teleport there):**
1. Pedestrian surface z on multi-level structures (needs a ped-position→deck-z map; possibly engine or
   ped-nav support).
2. Whether an elevated production net's lane z is fully carried by `NetworkLaneSource` (verify on a real
   3-D net), and whether the DDS-DR wire path (option b) needs z added (`ReplicationLaneShapeSource`
   currently returns null z).
3. Coordinate any engine API need for z with the in-flight engine-core sessions rather than editing it here.

---

## 2. Refined architecture

```
 [1] Engine @ fixed 10 Hz (StepLength=0.1)         Sim.IgBridge  (netstandard2.1;net6.0;net8.0)
     read SoA spans each step                       ├─ per-entity VehicleSampleHistory (ring)
       ▼                                            ├─ reconstruct: DrClock.ResolveAt + PoseResolver
 [2] per-entity ring buffers (sim-time)             │              + DrPoseSmoother   (REUSED, unchanged*)
       ▼                                            ├─ peds: PedestrianWorld → PedPublisher → HeadlessIg
 [3] resample @ fixed emit cadence (20 Hz)          │         + PedRemoteReconstructor (REUSED ped stack)
       ▼                                            └─ emit IgSample records → trace + in-memory ring
 [4] IG-native records: created / updated / removed
       ▼
 Sim.IgBridge.Host (net8.0, verification only)
     ├─ FakeIg: replay trace, 2-most-recent interp @ (clock − igDelay), threshold-jump (Q3)
     ├─ Sim.Viz side-by-side render (raw scene vs FakeIg-reconstructed scene)
     └─ metrics pass (raw vs reconstructed) + committed regression check
```
\* Two **non-behaviour-changing** additions to `Sim.Viewer.Motion` (see §5), both outside parity.

**Reuse split (both are shared-with-the-viewer, not binding-private):**
- **Vehicles** → `Sim.Viewer.Motion` (`DrClock`/`DrPoseSmoother`) + `Sim.Core.PoseResolver`. This is
  the "fix once, fix both" seam with the City3D viewer.
- **Pedestrians** → `Sim.Pedestrians/Lod` (`HeadlessIg` + `PedRemoteReconstructor`). Peds deliberately
  do **not** run through `DrClock` (they're holonomic, lane-free); their shared stack is the ped Lod
  reconstructor, which the native ped viewer uses. Peds are smooth by construction; only corner
  headings + LOD promote/demote route through smoothing.

---

## 3. The wire/trace record schema (PoC)

Three record kinds, one JSONL line each, ordered by `t` then by kind (created < updated < removed at
equal `t`). All positions planar; heading navi-degrees.

```jsonc
{"k":"new", "id":"veh0", "t":12.30, "model":"car",  "x":100.4, "y":22.1, "z":0.0, "h":271.3}  // entity-created
{"k":"upd", "id":"veh0", "t":12.35, "x":101.1, "y":22.0, "z":0.0, "h":271.0}                   // entity-updated
{"k":"del", "id":"veh0", "t":40.10}                                                    // entity-removed
```
- `t` is **sim time** (seconds). It is the sole timing authority; a sample's `(x,y)` is the pose **at
  `t`**. Temporal correctness (Q2) means: consecutive `upd` for an entity have `Δposition ≈ speed·Δt`,
  so the IG's linear DR between them is near-exact.
- `model` on `new` selects the IG 3D model (Q3). PoC values: `car`, `ped` (extend as needed).
- Vehicle ids are the engine `VehicleId` string; ped ids are `ped:<intId>` — globally unique, stable
  for the entity's life, never reused within a run.
- The **in-memory ring** carries the identical records (struct form) for a live consumer; the trace is
  the same stream flushed to disk, replayable/diffable.

---

## 4. Timing model (two distinct offsets — keep them separate)

1. **Reconstruction lookahead** (`Sim.IgBridge`, ~1 core tick ≈ 100 ms): the emit query time is held
   ~one 10 Hz tick behind the newest core sample so `DrClock.ResolveAt` always has a bracketing sample
   *ahead* (interpolate branch: arc-window turns, straddle lane-change, ChordHeading). The emitted
   sample's `t` = the query time. This is IgBridge-internal and fixed.
2. **IG playout delay** (`FakeIg` / real IG, 500 ms–1 s, Q2): applied by the **consumer**, not at emit.
   FakeIg plays at `tPlay = clock − igDelay` and interpolates the two most recent emitted samples that
   bracket `tPlay`. With 20 Hz emit (50 ms spacing) and ≥500 ms delay, `tPlay` is always bracketed →
   the IG always interpolates, never extrapolates a turn.

**End-to-end latency budget** = lookahead (~0.1 s) + igDelay (0.5–1.0 s). Explicit and tunable
(requirement §7). No auto-driven delay (DR doc §10.1 "auto-delay pulses speed" — we never do it).

---

## 5. Additions to `Sim.Viewer.Motion` (the only new code outside IgBridge; both outside parity)

### 5.1 Deterministic resolve seam — `DrClock.ResolveAt(history, sampleT, lanes)`
`DrClock.Resolve` computes `sampleT = _renderSim − delay`, where `_renderSim` is advanced by
`Pump(...)` from a **real `Stopwatch` wall clock** — correct for a live 60 Hz viewer, but non-deterministic
for an offline trace generator. Extract `Resolve`'s body into `ResolveAt(history, double sampleT,
lanes)` and make the existing `Resolve` call `ResolveAt(history, _renderSim − delay, lanes)`. This is a
**pure refactor** (byte-identical for the viewer, which keeps calling `Resolve`/`Pump`) that lets
IgBridge drive an **explicit sim-time query** — no `Stopwatch`, fully deterministic. IgBridge never
calls `Pump`, so the `Stopwatch` stays inert.

### 5.2 Lane-change ease over ~1.2–1.5 s (C1 — the one likely gap)
Today an instant lane change reconstructs as a lateral move over the **straddle bracket interval** only
(one packet gap), which at 10 Hz can be a single 100 ms slide. Spread it over a fixed **ease window**
`W ≈ 1.3 s` as an S-curve in the lateral axis. Placement and preference order:
- **Preferred (free) path:** if the sublane port lands (`LANE-CHANGE-OVERLAP-SPEC.md`),
  `TrajectoryPoint.PosLat` becomes real lateral offset and the ease is already in the data — no
  detector needed.
- **PoC path (add now):** a small stateful ease in `Sim.Viewer.Motion` keyed by entity handle. Detect a
  one-tick displacement that is **mostly perpendicular to heading** (the lane-snap signature — a full
  lane width ≈ 3.2 m sideways in one tick, not a turn). On detect, latch the lateral delta and release
  it as a **smoothstep over `W`**, so the emitted lateral offset ramps 0→Δ over ~1.3 s. Straight cruise
  and genuine turns (motion follows the lane) produce ~0 perpendicular delta → ease is inert.
- Lives in `Sim.Viewer.Motion` (new small `LaneChangeEaser` or folded into a resolve-side helper), so
  the 2D native/web viewers inherit it. **Not** in IgBridge. `W` is a tunable (§7).

### 5.3 Kinematic reconstruction — rear-axle drag (the "vehicle on rails" fix)
**Problem (owner-reported).** The reconstruction emits SUMO's **front reference point** as the position and
derives heading from the back→front chord *sampled on the lane polyline*. The renderer draws the body
backward from the front, so the **front is pinned to the polyline ("on rails") and the rear swings/jumps**
around the front pivot at every faceted internal-lane vertex / route-segment boundary — as if the rear
wheels steered. The same front-pivot rotation makes lane changes look like the rear slews sideways.

**Fix — a kinematic single-track (bicycle) model with the rear axle as a *towed* point.** The front
reference (reliable, follows the lane) tows a rear axle that **cannot slip sideways** — exactly a car's
unsteered rear wheels. Per vehicle, integrated at the emit cadence Δ:
```
Lwb        = wheelbaseFactor · Length            // ~0.6·Length; axles inset like an ordinary car
Fa         = Pfront − frontOverhang · dir(θ_prev) // front axle, inset from the front ref by ~0.15·Length
Ra(t)      = Fa − Lwb · unit( Fa − Ra(t−Δ) )      // rear axle DRAGGED, stays Lwb behind, no lateral slip
θ          = navi( Fa − Ra )                        // body heading = rear→front (forward)
Center     = (Fa + Ra) / 2                          // ≈ vehicle geometric center
```
Properties (why it's right): the rear follows a **smooth path and cuts inside the corner** (real
off-tracking) while the front rides the geometry — **no rear swing, no jump**; heading changes
*continuously* across polyline facets because the drag integrates them away; and on a lane change the
towed rear lags → the body **yaws into the change and back** (natural S-curve). 10 Hz core + 20 Hz emit is
ample — the drag integrates at 50 ms steps, far finer than the turn dynamics.

**Guards** (from the DR doc's hard-won lessons): below a small speed **hold heading** (no spin at a red
light); a supra-threshold front jump (teleport/handle reuse) **reseeds** `Ra` from the lane heading rather
than dragging across the gap; deterministic seeding at spawn (`Ra = Fa − Lwb·dir(laneHeading)`), no
`System.Random`, per-entity state → order-independent.

**Where it lives / reference point.** A new stateful per-handle component in **`Sim.Viewer.Motion`**
(`KinematicHeading`), so the 2D web/native and 3D City3D viewers inherit the identical model — fix once,
fix all. It **replaces** the chord-heading + motion-tilt heuristic (§10.3 of the DR doc) for this path,
kept selectable so the viewers can A/B. IgBridge emits the **vehicle Center + θ** (the owner's IG models
pivot at center; `z` = lane-surface/ground z, wheels-on-ground is the IG's job). The 2D `Sim.Viz` render
converts Center→front-ref only if the template anchors at the front (a fixed `±Length/2·dir(θ)` offset).

**Composition with §5.2 (ease).** Complementary: the **drag model fixes body orientation** (the rear
jump) for turns *and* lane changes; the **ease spreads the front's instant lateral jump** over ~1.3 s so
the front *path* is smooth. Together a lane change is a front sliding over ~1.3 s with the rear dragging
kinematically behind — a real-looking maneuver.

**Tunables:** `wheelbaseFactor` (0.6), `frontOverhangFactor` (0.15), `holdSpeed` (~0.5 m/s), `reseedJump`
(~7 m). All render-side; none affect parity.

### 5.4 As-built (smoothness pass — measured, not eyeballed)
The naive drag (§5.3) smooths *heading* but the rear POSITION still tracked the faceted-polyline kinks and
straddle gaps → a visible tail wiggle (measured on a per-vehicle rear-bumper reversal/jerk gate). The
shipped pipeline that drove the fleet to a **median of 0 visible rear-accel reversals** (≥2 m/s²; 67/85
vehicles clean, 80/85 at ≥4 m/s²):
1. **No emit gaps.** A lateral straddle is no longer skipped — both bracketing states are resolved on their
   own lanes and Cartesian-lerped, so the front path is continuous (a skip punched a hole the IG
   interpolated across and desynced the kinematic state).
2. **Real elapsed dt.** The kinematic step is fed the actual time since the vehicle's last emit (≥ one emit
   step, larger after a straddle), so SmoothDamp/drag integrate correctly instead of over/undershooting.
3. **Projective error-blending** of the front position (replaces the absolute-position low-pass): predict
   the front moving at `speed` along its heading, and critically-damp only the *deviation* of the
   authoritative front from that prediction. A genuine turn has ~no deviation → **zero lag**; facet kinks
   and lateral slides are deviations → absorbed and eased. Decay time `PositionSmoothTime = 0.40 s`.
4. **Lane-change ease (§5.2, as-built).** The engine's lateral-straddle signal latches a
   `LaneChangeEaseWindow = 1.3 s` during which the deviation decays over the longer
   `LaneChangeSmoothTime = 0.55 s`, spreading SUMO's instant lane change into a gentle ~1.3 s slide (a real
   car yaws ~10° into a lane change, not ~25). Turns never set this, so they stay crisp.
A residual tail (~5/85: departure/complex-swerve edge cases) remains for later refinement.

### 5.5 As-built (no-slip fidelity — owner: "rear wheels look steered, front sticks / rear skids")
§5.4 fixed heading smoothness, but the owner still read the *rear* as skidding — the visual signature of a
rear that swings **outward** instead of tracking inside. Two geometry bugs caused it:
1. **The single-step drag under-tracked at high yaw rate.** Pulling the rear to `Lwb` behind the front in one
   discrete jump per frame lets the rear cut across the corner (an apparent rear-steer). Fixed by
   **substepping** the tow: the rear is integrated along the front's *within-frame* motion in `N = 8` fine
   steps (`PrevFa → Fa`), so the discrete no-slip constraint holds even on a hairpin.
2. **A laterally-offset front axle.** The old front-overhang inset displaced the drag's front pivot sideways
   (it used the previous heading), injecting artificial slip. Dropped — the drag now pivots on the
   error-blended front reference directly. Vehicle **center** is placed half a length behind the front
   bumper along the body heading, so the front bumper rides the lane while the rear off-tracks.

**Verified against an ideal bicycle (not eyeballed).** For all 28 clean ~90° junction turns in the box run,
the emitted body's drawn rear-bumper was compared to a ground-truth no-slip bicycle integrated from the same
front path. They match to **<0.15°**: rear-bumper slip median 10.12° (emitted) vs 10.25° (ideal), max
72.74° vs 72.74°. So the residual rear-bumper motion is the *physically-correct tail swing* every real car
has on a 90° turn, not skid. Rear tracks inside (rear/front path-length ratio median 0.97) and heading stays
smooth (median 0 yaw-accel reversals). `PositionSmoothTime` retuned to 0.40 s (matching §5.4); heading
low-pass off (`HeadingSmoothTime = 0`, the substepped drag already yields a clean heading).

### 5.6 As-built (post-turn overshoot — owner: "after the 90° turn it oscillates like a beginner driver")
The front reference from PoseResolver is a **staircase** in heading — at a junction it jumps between polyline
facets (e.g. −64° in one step) then holds flat for ~1 s. §5.4's projective blend predicted the front moving
along the **body heading** `s.Deg`; while that heading lagged the travel direction (exactly the post-turn
transient), the prediction sat off to one side and injected a *cross-track* push. That coupled front-position
smoothing to the lagged heading and rang: the drawn heading sailed ~11° past the settled value and swung
back — the owner's wobble. Fleet-wide it made reconstructed peaks *worse than raw* (max yaw-jerk 127 k vs
raw 107 k).

**Fix — track the front with a constant-velocity g-h filter keyed to its OWN velocity, not the body
heading.** `pred = F + Fvel·dt`; correct toward the authoritative front by critically-damped gains
(`g = 1 − e^(−dt/τ)`, `h = g²/(2−g)`). The velocity term carries the front through a turn (zero lag) and the
prediction no longer depends on `s.Deg`, so the resonance is gone; cross-track jitter is still low-passed by
the gains, and heading smoothing is left to the rear-axle drag (a stable follower that cannot overshoot).
The lane-change ease just swaps τ (`PositionSmoothTime` → `LaneChangeSmoothTime`).

**Measured (box, 120 s).** Overshoot on the representative turner fell 11° → ~3° with a *monotonic* settle
(no sign reversals). Fleet-wide the pathological peaks collapsed below raw: reconstructed **max yaw-jerk
127 k → 7.9 k** (raw 107 k), **max yaw-rate 2128 → 173** (raw 1790), **max lat-accel 3128 → 163** (raw 1922);
medians held (yaw-jerk 41×, yaw-rate 4.2×, lat-accel 7.7× below raw). No-slip fidelity unchanged (drawn
rear-bumper slip 10.77° vs ideal-bicycle 10.87° over 68 clean 90° turns), rear still tracks inside (0.96),
heading still smooth (median 0 yaw-accel reversals). Motion 11/11, IgBridge 11/11, parity 654 / 4-skip
byte-identical.

**Viewer: click-to-identify.** To make "which car is misbehaving?" answerable, the side-by-side viewer now
labels a clicked vehicle with its id (yellow ring + id, latched across the raw/smoothed toggle so the *same*
car is compared). Guarded on a `vehIds` slot map the IgBridge export emits — inert (a no-op) for every other
`Sim.Viz` scene, so it is a shared, zero-cost addition.

All four additions are renderer-side, outside `Sim.Core`'s parity path; the offline `dotnet test` gate and
every golden stay byte-identical (§6).

### 5.7 Test bed — clean grid (owner: "almost no one is turning; cars born mid-junction; erratic peds")
The `demo_city/box` scenario is a poor smoothness bed: irregular topology (few clean turns, spawns near
junctions) and — the "erratic pedestrians" — were **our own** synthetic PathArc crowd, not scenario peds.
The Host now defaults to `scenarios/_ped/subarea-box`, a clean **6×6 Manhattan grid** (36 priority
intersections, 11 k vehicles with explicit turning routes, clean edge-start spawns). Knobs:
`IGBRIDGE_SCENARIO=<path under scenarios/>` picks any scenario with `<vehicle><route edges>` demand;
`IGBRIDGE_PEDS=1` re-enables the synthetic crowd (off by default — `IgBridgeConfig.EnablePeds`). The grid
gives **253 clean ~90° turns** (vs 68 in box) so the *median* vehicle turns, with bounded maxes (no U-turn
outliers): the overshoot fix (§5.6) verifies here at worst-case **1** heading yaw-accel reversal across all
253 turns, no-slip fidelity intact (drawn rear-bumper slip 11.51° vs ideal 11.57°), rear inside (0.98).
Scenario selection and the ped toggle are Host/harness only — no `Sim.Core`, no parity impact.

### 5.8 As-built (lane-change "rotation jump" — owner: four cars wobbly / "very sharply entering the lane change")
On the grid the owner flagged four cars still wobbling. Instrumenting each showed the identical cause: a
**3.2 m (one lane width) instantaneous lateral jump of the raw front in a single 0.05 s step** — SUMO's
instant lane change. The engine's lateral-straddle signal misses these (in-junction), so no ease fired and
the drag turned the snap into a **70–130 °/s "rotation jump."** Earlier attempts to hide it by low-passing
the whole front (longer g-h τ, or a SmoothDamp) made the front lag the *fast* along-track motion so far it
tripped the teleport reseed and produced 500–1000 °/s spikes — worse.

**Fix — a bounded, decaying LATERAL error (netcode-style), + a short g-h jitter low-pass.** A single-step
front jump beyond `LaneChangeSnapMeters` (1.5 m; a turn only advances ~½ m/step, so it never trips) has its
**cross-track** component absorbed into an error `E`; the tracker input is `raw − E`, which is *continuous*
(the drawn front never jumps). `E` then decays with `LaneChangeDecayTau` (2.0 s) and is capped at
`LaneChangeErrorCapMeters` (3.4 m ≈ one lane), so it can never approach the teleport threshold and the reseed
cannot misfire. **Along-track motion passes through untouched → zero lag → turns stay crisp.** A short-τ
(0.40 s) g-h then low-passes the faceted-polyline micro-kinks on that continuous input (its velocity term
keeps zero lag; it no longer sees the snap, so no overshoot). The substepped no-slip drag is unchanged.

**Measured (grid, 120 s).** Lane-change yaw fell from 70–130 °/s to a gentle **~10–25 °/s** (pure lane
changes); the four flagged cars no longer spike (whole-life max-yaw 70–73 °/s = their genuine 90° turns).
Fleet-wide the reconstructed maxes dropped further — **yaw-jerk max 16.8 k → 2.6 k, yaw-rate 281 → 74,
lat-accel 1985 → 101** (all far below raw); the 253-turn no-slip fidelity is intact (drawn rear-bumper slip
11.17° vs ideal-bicycle 11.28°), rear tracks inside (0.97), and heading yaw-accel reversals are back to
**median 0 / max 1** (the passthrough-jitter regression from dropping the g-h was fixed by re-adding it on
the corrected input). A multi-vehicle debug hook (`IGBRIDGE_DEBUG_VEH=v18,v98,…`) captured all four at once.
Render-side only. Motion 11/11, IgBridge 11/11, parity 654 / 4-skip byte-identical.

---

## 6. Determinism & parity

- **Deterministic trace.** Core ticks a fixed `StepLength=0.1` (no wall dt); peds tick
  `PedestrianWorld.Step(now, dt)` with fixed `now`/`dt` from the sim clock; emit is a fixed cadence
  driven by sim time, not wall jitter; reconstruction uses `ResolveAt(sampleT)` (no `Stopwatch`);
  `DrPoseSmoother.Smooth` gets a fixed `frameDt` = emit interval. No `System.Random`. → two runs of the
  same (scenario, seed, 10 Hz schedule, tunables) produce **byte-identical** traces (acceptance §9.5).
- **Parity untouched.** IgBridge and both `Sim.Viewer.Motion` additions are **consumers** of engine
  output; nothing touches `Sim.Core`'s simulation/parity code. Committed goldens and `dotnet test` stay
  byte-identical. The `Sim.Pedestrians` retarget (Q6) is a TFM-only change — verified by re-running
  `dotnet test` green, not by trusting the edit.

---

## 7. Tunables (all IgBridge/FakeIg/Motion; none affect parity)

| Parameter | Where | Default | Effect |
|---|---|---|---|
| Core step | scenario config `StepLength` | `0.1` s (10 Hz) | source sample rate |
| Emit cadence | IgBridge | `20` Hz | trace density; IG interp error between samples |
| Reconstruction lookahead | IgBridge | ~`0.1` s (1 tick) | keeps ResolveAt in the interpolate branch |
| IG playout delay | FakeIg (IG-side in prod) | swept `0.5–1.0` s | interpolate vs extrapolate; latency vs smoothness |
| IG jump threshold | FakeIg (Q3) | TBD (~lane width) | above → immediate jump, no DR smear |
| Lane-change ease window `W` | `Sim.Viewer.Motion` | `1.3` s (range 1.2–1.5) | lane change reads as a smooth ease, not a slide |
| DR realism | PoseResolver arg | `ChordHeading` | heading model (avoids swing-wide artifact) |
| Heading low-pass τ | `DrPoseSmoother` | `0.18` s (existing) | eases heading; >100° snaps |

---

## 8. What the PoC proves without a real IG (acceptance mapping)

- **§9.1** IgBridge runs `scenarios/_ped/demo_city/box` (vehicles + peds) at 10 Hz, writes an IG-sample
  trace for all entities.
- **§9.2** FakeIg replay (2-most-recent @ `clock − delay`, 0.5–1.0 s) shows **no lane-change teleports,
  no junction yaw snaps** in the `Sim.Viz` side-by-side.
- **§9.3** Reconstructed max yaw-rate / yaw-jerk / lateral-accel bounded and far below the raw stream;
  lane changes reconstruct as a ~1.3 s ease (targets fixed in the tasks doc).
- **§9.4** Reuse proven: the smoothing is `Sim.Viewer.Motion`/`PoseResolver` (+ ped Lod) — a fix there
  changes both the IG trace and the City3D viewer.
- **§9.5** Deterministic: two runs → identical traces; latency budget (§4) documented and tunable.
