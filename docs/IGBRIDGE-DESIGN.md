# IgBridge ↔ external 3D image-generator (IG) binding — DESIGN (the HOW)

How to feed an **external 3D image generator (IG)** from SumoSharp with all core motion artifacts smoothed,
and build a PoC that proves it without the real IG. The target is any IG that has **no protocol for
predictive dead-reckoning** and consumes only plain position/orientation/timestamp samples — nothing
IG-specific is assumed. Requirements (the WHAT) are in `IGBRIDGE-REQUIREMENTS.md`; read it first. This doc
references, and deliberately **reuses**, the existing viewer reconstruction stack.

> **AS-BUILT (2026-07) — read this first.** This is the original plan. The PoC shipped and was tuned to an
> owner-signed-off **v5** baseline. Its reconstruction outgrew the old `DrPoseSmoother`: it became a no-slip
> **rear-axle (bicycle) model** with low-passed lane-heading prediction, a **spatial look-ahead** that
> anticipates the connecting lane through a junction, an anticipation lead-bound, a bounded lane-change ease,
> and coarse-feed junction-straddle handling. That reconstruction was extracted into the shared
> `Sim.Viewer.Motion.KinematicReconstructor` (over `KinematicHeading`) and now smooths **both** the raylib 2D
> and Godot City3D viewers as well — `DrPoseSmoother` was **deleted**. So **wherever this doc says
> `DrPoseSmoother`, read `KinematicReconstructor`.** Current state lives in `IGBRIDGE-VERSIONS.md` (v1–v5
> milestones), `IGBRIDGE-DECISIONS.md` (§5.x as-built), `IGBRIDGE-METHODOLOGY.md` (how we tuned + the
> 2D-visualization recipe), and `VIEWER-KINEMATIC-SMOOTHING-DESIGN.md` (the viewer swap).

## 0. The load-bearing insight
The IG does **no prediction**: it consumes plain position/orientation/timestamp samples and interpolates
between its two most recent — there are no velocity/acceleration/curvature prediction fields. It cannot fix
anything.
Therefore **all smoothing must be baked into the sample stream IgBridge emits** — the emitted samples must
lie on a smooth curve, dense enough that the IG's linear/slerp between any two consecutive samples already
looks right. IgBridge is a **reconstruction + resampling stage**, not a passthrough. In particular a
one-tick lane change must be **expanded into many small emitted samples over ~1–1.5 s**, or the IG will
linear-interpolate a full lane width across one send interval (a teleport).

## 1. Reuse — do NOT write new smoothing math
The Godot City3D / native viewers already solve exactly these artifacts. Reuse:
- **`Sim.Viewer.Motion`** (`net8.0`+`netstandard2.1`, no renderer/wire/native deps):
  - `DrClock.Pump(newestSampleTime, hold)` — a strictly-monotonic, jitter-immune render/sim clock from a
    long-baseline wall↔sim rate fit.
  - `DrClock.Resolve(history, delay, laneSource)` — interpolate/extrapolate a `DrState` at query time:
    same-lane arc-length lerp; **lane-crossing turns via an arc-window walk** (follows the real curved
    internal-lane geometry instead of snapping); **lane-change (sideways) straddle returns both bracketing
    states** for a Cartesian lerp.
  - `KinematicReconstructor.Resolve(handle, resolved, lanes, dims, dt)` — the shared per-entity render-side
    reconstruction (no-slip rear-axle drag + lane-heading prediction + spatial look-ahead + lane-change ease).
    (Was `DrPoseSmoother.Smooth` in the original plan — see the AS-BUILT banner.)
- **`Sim.Core.PoseResolver.Resolve(...)`** — `DrState → (x, y[, z], heading)`, with `RenderRealism.ChordHeading`
  (no "swing-wide" bow) and the motion-derived heading tilt.
- **`Sim.Replication.DrExtrapolation.Arc`** — the single source-of-truth DR curve (publisher and viewer share
  it so prediction never diverges).
- Design + tunables + the junction-turn fixes: **`SUMOSHARP-VIEWER-DR-SMOOTHING.md`** — especially **§8
  "Reimplementing in a future viewer (e.g. 3D IG)"** (written for exactly this), §6 (lane-transition snap →
  arc-window; swing-wide → ChordHeading; delay volatility → seed+slew+dead-band), §9 tunables, §10 as-built.
- Ped side: `HeadlessIg` / `PedRemoteReconstructor` (`Sim.Pedestrians/Lod`) reconstruct low-power peds from
  the pure-function-of-time timeline; peds are far less artifact-prone (smooth by construction) but their
  corner headings and LOD promote/demote transitions still route through the same smoothing.

**Fix-once-fixes-both is structural:** keep every smoothing decision inside `Sim.Viewer.Motion`/`PoseResolver`.
IgBridge adds *transport + resampling + 2D→3D + lifecycle*, never new curve/heading math.

## 2. Architecture — three decoupled stages
```
 [1] SumoSharp core @ fixed 10 Hz  ──samples──▶  [2] shared reconstruction  ──pose(t)──▶  [3] IgBridge emit
     (Engine + PedestrianWorld)     (per-entity     (Sim.Viewer.Motion +                     + FakeIg + trace
      deterministic, sim-time)       ring buffer)    PoseResolver, REUSED)                    + metrics/render)
```
- **[1] Fixed-rate core.** Own 100 ms sim clock; never driven by IgBridge's variable wall tick. Each step
  appends per-entity timestamped state to a **per-entity ring buffer** keyed by sim time. Feed the buffer
  the same shape the viewer expects (lane + arc-pos + speed, and/or x,y+heading — see §6 on which stream).
- **[2] Reconstruction.** `DrClock.Resolve` + `PoseResolver.Resolve` (+ `KinematicReconstructor`) → a smooth
  `pose(entityId, tQuery)` for any query time inside the buffered window. This is the reused stack, unchanged.
- **[3] IgBridge stage.** Advances a **sim clock** (§4), and at the IG send cadence queries [2] at a single
  coherent `tQuery` for all entities, applies **2D→3D** (§5), and emits `[id, pos, quat, t]` to a trace log +
  in-memory ring. A bundled **FakeIg** (§7) replays that trace with the real IG's 2-sample rule.

## 3. Why not let the IG do the smoothing
Because it only linear/slerps the last two samples. Feeding it raw 10 Hz SUMO samples reproduces every
artifact (a lane change becomes a 3.2 m / 100 ms slide; a junction turn becomes a sequence of yaw snaps).
The reconstruction in [2] produces a *continuous* pose(t); IgBridge samples that continuous curve at the IG
cadence, so the IG interpolates between points that are already on the smooth curve. Dense enough cadence
(≥ ~20–30 Hz emit) makes the linear error between smooth-curve samples negligible.

## 4. Timing — sim clock, jitter buffer, interpolation delay
Standard game-netcode **entity interpolation**:
- Decouple **sim time** from wall time. IgBridge plays at `simClock = f(wall)` (1× real-time by default),
  advanced by `DrClock.Pump` (jitter-immune, monotonic). The emitted sample's `timestamp` is **sim time**.
- Emit on a **steady cadence**; the IG plays at `tPlay = latest_sim_time − interpDelay`. Choose
  `interpDelay ≥ one emit interval + jitter margin` so the IG always brackets `tPlay` with two real samples →
  **always interpolates, never extrapolates a turn**. This is the owner's "delay a bit to avoid extrapolation".
- The reconstruction ([2]) itself needs one sample *ahead* of its query for arc-window/centripetal behaviour,
  so it runs ~1 core tick behind newest. **Total latency = core-tick lookahead + IG interp delay** — a few
  hundred ms, invisible for viz. Make it one explicit, tunable budget (mirror the §9 tunables of the DR doc;
  note §10.1 "auto-delay OFF by default — it caused speed pulsing" and §5.4/§6.3 delay seed+slew+dead-band).

## 5. 2D → 3D (IgBridge-added, not in the 2D viewers)
- `z` = City3D **procedural terrain height** at `(x, y)` — reuse the Godot City3D terrain height field so the
  IG feed and the 3D viewer agree on the ground. Pitch/roll from the local slope along heading (car banks with
  the hill). These inherit position smoothing; sample the terrain at the smoothed `(x, y)`.
- Orientation on the wire: **quaternion** (yaw from smoothed heading, pitch/roll from terrain) is the safe
  default for a 3D IG; confirm with the owner whether the IG wants full quaternion or yaw-only (Q1 below).
- Peds: upright, yaw from motion direction; no bank.

## 6. Which input stream feeds the reconstruction
Two options; the design recommends starting with (a) and keeping (b) as the faithful end-state:
- **(a) Direct 10 Hz engine state** (x,y,heading,lane,arc,speed available in-process). Simplest; still runs
  the SAME `DrClock.Resolve`/`PoseResolver` so the smoothing (arc-window turns, ChordHeading, lane-straddle,
  heading tilt) is identical to the viewer. Recommended for the PoC.
- **(b) The DDS DR-published sparse stream** (lane+arc+speed, DR-error-gated) that the City3D viewer already
  consumes. Maximum "one stream, fix once", but adds the publish/subscribe path. Move here only if the owner
  wants the IG fed off the identical wire the 3D viewer uses.
Either way the reconstruction routines are the same — that is what makes the fix shared.

## 7. Artifact-by-artifact plan (each maps to an existing solution)
- **Junction turns** → `DrClock.Resolve`'s **arc-window walk** across lane-crossing brackets + `ChordHeading`
  (DR doc §6.1/§6.2). Already solved for the 2D viewer; IgBridge inherits it.
- **Orientation jumps / stops** → motion-derived heading (DR doc §10.3) + hold-last-heading below a small
  speed (no spinning at red) + a max yaw-rate limiter for the emitted quaternion.
- **Instant lane change** → the straddle Cartesian-lerp gives a lateral move, but over the *bracket* interval
  only. **Gap to close:** ensure the lane change is spread over a fixed **ease window (~1.2–1.5 s)**, not one
  100 ms tick. Options, in order of preference: (i) if the sublane port lands (see `LANE-CHANGE-OVERLAP-SPEC.md`),
  `TrajectoryPoint.PosLat` becomes real lateral offset and the ease is free; (ii) otherwise add a detect + ease
  in the reconstruction: a one-tick displacement mostly **perpendicular to heading** is a lane snap → S-curve
  the lateral component over the ease window (a post-hoc sublane). Put this in `Sim.Viewer.Motion` so the 2D
  viewer benefits too.
- **Overlaps** → NOT smoothed here (a sim defect; separate spec). IgBridge just renders faithfully.

## 8. Entity lifecycle (the 4-tuple has no teleport bit)
- **Spawn:** first sample appears; the IG holds until it has two. Emit a stable `entityId`.
- **Despawn:** stop emitting on route end/removal; the IG times the entity out. Confirm the real protocol's
  despawn semantics (Q3).
- **Genuine discontinuities** (LOD promote/demote position nudge; SUMO jam-teleport — off in our scenarios)
  must NOT be smoothed as motion: detect a supra-threshold jump that is NOT a lane change and handle by a
  brief hide/reappear or an id re-key, never a 60 m/s slide. Enumerate every discontinuity source up front.
- **Coherence:** emit all entities at one shared `tQuery` per frame so the IG scene is time-consistent.

## 9. The PoC — `Sim.IgBridge` (proposed)
- A console/host app that: loads a scenario, ticks `Engine` + `PedestrianWorld` at fixed 10 Hz, feeds the
  per-entity ring buffers, runs the reused reconstruction at the emit cadence, applies 2D→3D, and writes an
  **IG-sample trace** (JSONL: `{id, t, x, y, z, qx, qy, qz, qw}` + kind) and an in-memory ring.
- **`FakeIg`** — replays the trace doing the real IG's job: per entity keep the 2 most recent samples, at
  `tPlay = clock − delay` linear-interp position and slerp orientation (and extrapolate when forced, to
  exercise the delay tuning). Output: the poses the IG would show.
- **Analysis (R8):** (1) side-by-side render — reuse the `Sim.Viz` HTML replay pipeline to draw *raw SUMO*
  vs *FakeIg-reconstructed* from the same run; (2) a metrics pass over both streams: per-entity max
  yaw-rate, yaw-jerk (dω/dt), lateral acceleration, C1 position gap, and measured lane-change duration —
  raw vs reconstructed, as a table + as a committed regression check.
- Reuse the DR doc's **§7 diagnostic harness** rather than inventing a new one.

## 10. Determinism & parity
- Emitted trace is a pure function of (scenario, seed, fixed 10 Hz schedule, tunables). The wall↔sim fit is
  `DrClock`'s jitter-immune monotonic clock; no `System.Random`; per-entity state is order-independent.
- Nothing here touches `Sim.Core`'s parity path — IgBridge/`Sim.IgBridge` are consumers. The offline
  `dotnet test` gate and every golden stay byte-identical. Any smoothing change lands in `Sim.Viewer.Motion`
  (already outside parity) with its own before/after metric check.

## 11. Staged plan (independent session)
- **Stage 0 — survey & wire.** Read `SUMOSHARP-VIEWER-DR-SMOOTHING.md` (esp §8) + `Sim.Viewer.Motion`. Stand up
  `Sim.IgBridge`: fixed-10 Hz core, per-entity ring buffers, sim clock. Confirm `Sim.Viewer.Motion` is
  consumable from .NET 6 (it targets netstandard2.1 — should be).
- **Stage 1 — reconstruct + emit + FakeIg.** Wire `DrClock.Resolve`+`PoseResolver` (option (a) input) → emit
  trace → `FakeIg` replay. Side-by-side render. Baseline metrics (expect junction turns already smooth via §6).
- **Stage 2 — lane-change ease + 2D→3D + lifecycle.** Add the perpendicular-displacement ease window (in
  `Sim.Viewer.Motion`, so the 2D viewer gains it too), terrain z/pitch/roll, spawn/despawn/discontinuity
  handling. Re-measure: lane changes reconstruct as a smooth ~1.3 s ease; yaw-jerk bounded.
- **Stage 3 — tune & prove.** Sweep interpDelay / emit-rate / ease-window; lock the latency budget; commit
  the metrics regression check; document the tunables (mirror DR doc §9). Confirm determinism + parity gate.

## 12. Open questions for the owner
1. **Orientation on the wire:** full quaternion (pitch/roll from terrain) or yaw-only?
2. **Emit cadence & interp delay:** does the IG fix the delay, or may IgBridge choose the cadence? (Sets the
   jitter-buffer math and the §4 budget.)
3. **Despawn / teleport semantics** in the real protocol — how does the IG learn an entity is gone / has
   legitimately jumped?
4. **Input stream:** feed the reconstruction from the direct 10 Hz engine state (a) or the DDS DR wire (b)?
5. **Terrain source of truth:** is the City3D procedural terrain the authority for `z`/slope the IG should
   match, or does the IG own its own terrain (then IgBridge sends only planar pose + yaw)?
