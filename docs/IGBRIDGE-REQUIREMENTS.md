# IgBridge ↔ external 3D image-generator (IG) binding — REQUIREMENTS (the WHAT)

Reference/spec for a new SumoSharp output binding: feeding an **external 3D image generator (IG)** — one with
no protocol for predictive dead-reckoning, consuming only plain position/orientation/timestamp samples — from
an external .NET simulation host called **IgBridge**, with all SUMO-core motion artifacts smoothed out. This
document is the WHAT (requirements + acceptance). The HOW is in `IGBRIDGE-DESIGN.md`.

## 1. Context
- **SumoSharp** (this repo) is embedded as a library inside **IgBridge**, an existing **.NET 6** C#
  simulation host that already knows the full IG protocol and how to transmit to the real IG.
- IgBridge is **tick-based, ~20 Hz, variable Δt** (it uses the previous frame's duration as the next
  frame's delta; fluctuates on Windows 11). It is **not** a fixed-step loop.
- SumoSharp's core is to be **ticked at a fixed 10 Hz** to provide smooth-enough source samples.
- Entity counts are **low for this use case: up to ~1000 vehicles + pedestrians combined.** The highly
  optimized deterministic DR/prediction DDS protocol is therefore **not mandatory**; a simple per-sample
  protocol suffices.

## 2. The IG protocol (the hard constraint)
- The IG accepts only **`[entityId, position, orientation, timestamp]`** samples (simplified here — the
  real protocol has more properties, but the shape and its limits are what matter).
- **No dead-reckoning / prediction fields** are carried. The IG has no velocity, no curve, no lane info.
- The IG reconstructs motion itself from the **2 most recent samples** it has received per entity: it
  **interpolates/extrapolates** position **and orientation** between/beyond them.
- There is a **configurable time offset (delay)**; in practice the IG is run **delayed a bit** so it
  **interpolates rather than extrapolates** (extrapolation is what makes SUMO output look bad).
- Consequence (load-bearing): **the IG cannot fix any artifact.** Whatever smoothing is needed must be
  **baked into the sample stream IgBridge emits**, such that naive linear/slerp interpolation between any
  two consecutive emitted samples already looks correct.

## 3. The artifacts to smooth (why this is non-trivial)
SumoSharp's core (SUMO-parity) produces motion that is correct as *simulation* but has *rendering*
artifacts a non-predictive 2-sample interpolator exposes:
- **Instant lane changes** — position steps a full lane width (~3.2 m) in one core tick (no sublane lateral
  motion is modelled). Emitted raw, that is a teleport / 60 m/s lateral slide.
- **Non-smooth junction turns** — internal-junction edges are coarse linear-segment polylines; heading is a
  per-segment tangent, so orientation **jumps** at each vertex.
- **Orientation jumps generally** from coarse polyline sampling and from lane-relative heading snaps.
- (Related, tracked separately: dense multi-lane car *overlaps* — see `LANE-CHANGE-OVERLAP-SPEC.md`. Out of
  scope here; IgBridge smooths *motion*, it does not fix the sim.)

## 4. Reuse mandate (fix once, fix both)
The Godot **City3D** viewer and any native viewer already smooth these artifacts with a shipped,
engine-agnostic reconstruction stack (`Sim.Viewer.Motion` = `DrClock`/`DrPoseSmoother`, `Sim.Core.PoseResolver`,
documented in `SUMOSHARP-VIEWER-DR-SMOOTHING.md`). **IgBridge must reuse the same reconstruction routines**
so that a fix to a junction-turn / lane-change / heading glitch fixes the IG feed *and* the City3D viewer
together. No forked, IgBridge-private smoothing math.

## 5. Deliverable — a PoC, no real IG required
Build a **IgBridge-like proof-of-concept app** that:
1. Embeds SumoSharp; ticks the core at fixed 10 Hz.
2. Converts each vehicle's and pedestrian's motion into **IG-native `[id, position, orientation, timestamp]`
   samples**, applying all smoothing needed to hide the §3 artifacts, **reusing the §4 routines**.
3. Produces the emitted samples **in memory** and **logged to a file** (a replayable trace).
4. Is **renderable / analyzable WITHOUT the real IG**: a bundled *fake IG* consumes the logged samples using
   the **same 2-most-recent-sample interpolation** the real IG does, so one can *see and measure* what the IG
   would show. (This is the part that can be done well here with no dependency on the IG's own protocol.)

## 6. Functional requirements
- **R1** — Fixed 10 Hz SumoSharp core, decoupled from IgBridge's variable ~20 Hz wall-clock tick
  (the sim clock must not be driven by Windows frame jitter; output must stay deterministic given the same
  scenario + seed + 10 Hz schedule).
- **R2** — Per-entity, per-emit **smooth pose** `(position, orientation)` for both vehicles and pedestrians,
  free of the §3 artifacts, produced via the reused reconstruction stack.
- **R3** — **IG-cadence sample emission** with an explicit **interpolation-delay / jitter-buffer** model so
  the (simulated and real) IG always **interpolates between two real samples**, never extrapolates a turn.
- **R4** — **Entity lifecycle** handling for a protocol with no teleport bit: spawn (first sample), despawn
  (route end / stop emitting), and genuine discontinuities (LOD promote/demote nudges; SUMO jam-teleport —
  off in our scenarios) must NOT be smoothed as motion.
- **R5** — **2D → 3D**: map SUMO planar `(x, y, heading)` to the IG's 3D pose — `z` and pitch/roll from the
  City3D procedural terrain (reuse its height field), smoothed so cars sit on and bank with the ground.
- **R6** — **Trace log** (e.g. JSONL/CSV) of every emitted IG sample, replayable and diffable.
- **R7** — **Fake-IG consumer** implementing the 2-most-recent-sample interpolate/extrapolate rule + the
  configurable delay, producing the reconstructed poses the real IG would show.
- **R8** — **Analysis output**: side-by-side (raw SUMO vs fake-IG-reconstructed) render, plus **objective
  smoothness metrics** (per entity: max yaw-rate, yaw-jerk, lateral acceleration, C1 position gaps,
  lane-change duration) for raw vs reconstructed, so "no artifacts" is *measured*, not eyeballed.

## 7. Non-functional
- **Determinism**: given the scenario + seed + fixed 10 Hz schedule, the emitted sample trace is
  reproducible. No `System.Random`; the wall-clock→sim-clock mapping must be explicit and jitter-immune.
- **Latency budget**: end-to-end (core tick → reconstruction lookahead → IG interp delay) on the order of a
  few hundred ms is acceptable for visualization; it must be **explicit and tunable**, not accidental.
- **Scale**: correct and smooth at up to ~1000 concurrent entities; the per-entity reconstruction cost must
  fit a 10 Hz produce + ~20–60 Hz emit budget on a Windows workstation. (Optimized DR is *not* required at
  this scale, but the reconstruction must not be O(N²) or allocate per entity per frame.)
- **Parity untouched**: this is a consumer of engine output; it must not change any committed golden or the
  offline `dotnet test` gate. All smoothing lives outside `Sim.Core`'s parity path.
- **.NET compatibility**: the reused reconstruction library already targets `net8.0` + `netstandard2.1`;
  confirm it (and the PoC's shared parts) are consumable from IgBridge's **.NET 6**.

## 8. Out of scope
- The real IG, its wire transport, and its non-simplified property set (IgBridge owns those).
- Fixing simulation-level defects (e.g. the dense multi-lane overlap — `LANE-CHANGE-OVERLAP-SPEC.md`).
- The optimized DDS DR protocol itself (this binding deliberately uses the simple per-sample protocol).

## 9. Acceptance criteria (PoC)
1. The PoC runs a committed scenario (e.g. the live-city box) with SumoSharp at 10 Hz and writes an IG-sample
   trace for all vehicles + pedestrians.
2. The **fake IG**, replaying that trace with the 2-most-recent-sample rule at `now − delay`, shows **no
   visible lane-change teleports and no junction-turn yaw snaps** in the side-by-side render.
3. **Metrics**: reconstructed max yaw-rate / yaw-jerk / lateral-accel are bounded and dramatically lower than
   the raw SUMO stream's (target numbers set in the design doc); lane changes reconstruct as a smooth ease of
   a stated duration, not a one-interval jump.
4. **Reuse proven**: the smoothing is the shared `Sim.Viewer.Motion`/`PoseResolver` stack — a fix there
   changes both the IG trace and the City3D viewer.
5. Deterministic: two runs produce identical traces; the latency budget is documented and tunable.
