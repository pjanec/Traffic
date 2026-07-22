# IgBridge — tuning methodology (how we tune the reconstruction, and how to return to it)

This is the durable **method** behind the IgBridge motion-reconstruction work: how we diagnose a
visual-realism problem, turn it into numbers, fix it without breaking anything, and prove the fix.
It is deliberately separate from the *as-built story* (`IGBRIDGE-DECISIONS.md` §5.x), the *milestone
record* (`IGBRIDGE-VERSIONS.md`), and the *live working state* (`IGBRIDGE-RESUME.md`). Read this when
you pick the work back up and want to remember **how we work**, not just what we built.

## 0. The one rule: measure, don't eyeball

Every change is driven by a **number with a goal**, not by "looks better". The owner watches the render
and reports a specific artifact ("v90 dances at ~16 s"); we reproduce it, **instrument the exact vehicle**,
localize which pipeline stage produces the artifact, form a hypothesis, change **one** thing, and re-measure
the same number. We never accept a change because a run "seemed smoother" — we accept it because a defined
metric moved the right way *and* the standing invariants still hold. The single biggest lesson of this
project (the "skidding" saga) was that **the eyes and the metrics can disagree, and when they do, one of
them is instrumented wrong** — reconcile them before touching the algorithm (see §7).

## 1. Standing invariants — every change must preserve ALL of these

A change is **rejected or reverted** unless it keeps every one of these true. Check them before accepting:

1. **Parity byte-identical.** `dotnet test tests/Sim.ParityTests` stays **654 pass / 4 skip**, byte-for-byte.
   IgBridge is render-side; it must never perturb the engine's committed trajectories. This is the iron law.
2. **Engine core untouched.** No edits under `Sim.Core` (car-following, lane-change, junctions, teleport).
   Other sessions own those. IgBridge only *calls* public engine APIs (`DefineVType`, `SpawnVehicle`,
   `GetUpcomingLanes`, the SoA read spans) and reconstructs render-side.
3. **Determinism.** Two runs of the same config produce a **byte-identical** emitted trace. No `System.Random`,
   no wall-clock, no thread-order dependence. Verify with a two-run `diff` on `trace.jsonl`.
4. **Default = the current baseline, byte-identical.** Any new knob must default to the previous accepted
   baseline's behavior, proven by a byte-identical `trace.jsonl` diff against a saved baseline trace. New
   behavior is *opt-in* (an env knob / a length threshold), never a silent change to the shipped default.
5. **Cars unchanged when tuning something else.** When adding buses, coarse-feed, etc., the passenger-car
   stream must stay byte-identical (e.g. length-scaled look-ahead uses `max(3, 0.5·len)` so a 5 m car stays
   on the flat 3 m). Prove it with a trace diff.
6. **Motion + IgBridge unit gates green.** `tests/Sim.Viewer.Motion.Tests` (11) and `tests/Sim.IgBridge.Tests`
   (11) pass.
7. **Shared viewer stays back-compatible.** `Sim.Viz/template.js`/`.html` are shared with City3D and the ped
   test beds; guard every IgBridge-specific addition (e.g. `scene.useDataHeading`, 5-tuple vehicle rows,
   `scene.vehIds`) so the 3-tuple (City3D) and 4-tuple (panic-evac fear) paths are untouched. `node --check`
   the JS.

## 2. The gate commands (run these; trust these)

```
dotnet build src/Sim.IgBridge.Host -c Release
dotnet run   --project src/Sim.IgBridge.Host -c Release   # -> artifacts/igbridge/sidebyside.html
dotnet test  tests/Sim.Viewer.Motion.Tests -c Release     # 11
dotnet test  tests/Sim.IgBridge.Tests       -c Release     # 11
dotnet test  tests/Sim.ParityTests          -c Release     # 654 / 4  (iron law)
```
Determinism / default-unchanged check (the workhorse):
```
dotnet run ... >/dev/null; cp artifacts/igbridge/trace.jsonl /tmp/A.jsonl
dotnet run ... >/dev/null; diff -q /tmp/A.jsonl artifacts/igbridge/trace.jsonl   # must be identical
```
**Never** run SUMO inside the offline test loop. SUMO is only for *authoring* inputs (net/demand
generation, golden regen) and for direct engine-vs-SUMO diagnosis — see `CLAUDE.md`.

## 3. The instrumentation

- **Per-vehicle debug CSV**: `IGBRIDGE_DEBUG_VEH=v90` (or a comma list) dumps `artifacts/igbridge/debug_<id>.csv`
  with columns `t,prX,prY,prH,smX,smY,cX,cY,cH,rearX,rearY,speed,laH`:
  - `prX,prY,prH` — the **raw** front pose from `PoseResolver` (arc position + lane/chord heading). The
    reactive ground truth; if `prH` is smooth the *input* is fine and the artifact is downstream.
  - `smX,smY` — the g-h / lane-predicted smoothed **front** tracker.
  - `cX,cY,cH` — the emitted **center** + body heading (what the IG receives; center is ½·length behind front).
  - `rearX,rearY` — the drawn **rear bumper** (`center − ½·length·dir(cH)`), for slip/tail-swing analysis.
  - `laH` — the **applied predictor heading**: the look-ahead bearing when used, or the lane heading `prH`
    when the look-ahead was rejected. `laH == prH` ⇒ look-ahead fell back that frame.
- **Slider-time convention**: the render window is real **20–80 s**; the owner's slider time = **real − 20 s**.
  Always convert before dumping (owner "v90 at 16 s" ⇒ inspect real t ≈ 36 s).
- **Multi-vehicle sweep**: dump 50–100 ids at once to get fleet distributions of a quantity (e.g. the legit
  look-ahead lead we bounded in §6 was found by dumping `v40..v139` and taking the p99.9/max of `|laH−prH|`).

## 4. The metric suite (definitions + what each catches)

Two levels — pick the right one:

**A. Emitted-stream metrics** (over `trace.jsonl`, per-vehicle heading series) — diagnose the *reconstruction*:
- **Yaw-accel reversals** — count of `>30°/s²` sign flips in the heading's second derivative over a track.
  The primary **smoothness gate**. Target: fleet **median 0**; a clean ~90° turn should show 0. This is what
  "beginner-driver oscillation" and "the dance" show up as.
- **Peak yaw-rate** — max `|dH/dt|`. A smooth junction turn ramps to ~50–80°/s; a *snap* (facet or dance)
  spikes to 200–1000°/s. The dance fix drove v90 from 243 → 64°/s.
- **Rear-axle no-slip** — lateral (perpendicular-to-heading) speed of the **rear axle** (`center − 0.1·len·dir`,
  since axle = front − 0.6·len·dir and center = front − 0.5·len·dir). A no-slip bicycle keeps this ≈ 0;
  in-turn median holds **0.03–0.10 m/s** across car and bus. NB: measure the **axle**, not the rear **bumper** —
  the bumper legitimately sweeps (`≈ ω·0.4·len`, real rear-overhang tail-swing that *should* scale with length).
- **Offset-from-connecting-lane-centerline** — emitted front vs the raw front through a junction. The look-ahead
  drove this from ~1.1–1.5 m (turn-in late) to ~0.3–0.5 m (rides the connecting lane).
- **Backward-drift-while-stopped** — center forward-step at speed < 0.5 m/s. Must be ~0 (the "dancing on the
  spot" bug; fixed by clamping the tracked front velocity to the known speed).

**B. Display-level metrics** (both raw and smoothed streams pushed through the **FakeIg** 2-sample
interpolator, then `Metrics.Compute`) — this is the **fair A/B** and what `Program.cs` prints. Use it to
compare *raw-fed-IG vs IgBridge-fed-IG*, and to compare feed/emit rates:
`yaw-rate`, `yaw-jerk`, `lat-accel`, `C1 vel-gap`, each as median/mean/p95/max with a "median reduction".
The **median** is the typical vehicle; **p95/max** expose tail transients (a few vehicles at a hard junction).
Reconstruction-vs-raw median reductions run ~5× (yaw-rate), ~35–55× (yaw-jerk), and the coarse-feed tests
showed **71–188×** on lat-accel / vel-gap.

> Pitfall: never compare a reversal count between streams of **different sample rates** (a 1 Hz raw series has
> ~20× fewer points than a 20 Hz reconstruction, so it *looks* smoother by raw count). Cross-rate comparisons
> must go through the FakeIg to a common display rate (level B).

## 5. The diagnostic loop (the actual workflow)

1. **Reproduce** the owner's artifact from the exact config (scenario + env knobs), convert slider→real time.
2. **Instrument** the named vehicle(s): `IGBRIDGE_DEBUG_VEH`, dump the CSV around the event window.
3. **Localize the stage.** Walk the CSV columns front-to-back: is `prH` (raw input) already bad → it's an
   input/geometry issue; is `prH` smooth but `laH` diverging → look-ahead; is `cH` lagging/oscillating vs a
   clean `laH` → the kinematic front tracker / smoothing. (For "skidding", the stages were all clean and the
   bug was in the **viewer** — see §7.)
4. **Hypothesize a mechanism** in one sentence and predict which number will move.
5. **Change one thing** (ideally behind a new knob defaulting to baseline).
6. **Re-measure** that number *and* re-run the invariants (§1). Sweep the knob (e.g. lead-bound 40/50/70) to
   see the tradeoff and pick the value that fixes the artifact while keeping the baseline byte-identical.
7. **Accept** only if the metric moved and all invariants hold; else revert. Commit with the numbers in the
   message; append to `IGBRIDGE-VERSIONS.md` at each owner-approved baseline.

## 6. Worked example — the coarse-feed "dance" (v90 @ 1 Hz)

Textbook application of §5. Owner: "v90 ~16 s, smooth right turn then a dance." Real t ≈ 36 s.
- Dumped `debug_v90.csv`: `prH` ramped 90→180 **cleanly** (input fine); `laH` first led correctly (+48°),
  then after the apex **drifted gradually** to −142° (pointing back the way it came) — each 0.05 s step under
  the 250°/s frame-to-frame guard, so the guard **missed the slow drift**. The front tracker follows `laH`,
  so `cH` dipped 133→108° then **snapped back at 243°/s** when the look-ahead reset. That snap *is* the dance.
- Mechanism (one sentence): *the per-frame jump guard catches sudden look-ahead hops but not gradual divergence,
  so on a coarse feed the look-ahead can drift to a wrong-way bearing and drag the body.*
- Measured the legit ceiling: dumped `v40..v139` at 10 Hz, `|laH−prH|` on used frames maxes at **60.0°**.
- Fix: reject the look-ahead when it diverges `> MaxAnticipationLeadDeg` (`IGBRIDGE_MAX_LEAD`, default **70°**)
  from the reactive lane heading. 70 > 60 ⇒ **v5 baseline byte-identical**; v90 peak yaw **243 → 64°/s**;
  fleet 1 Hz tail yaw-rate max 252 → 107, lat-accel p95 55 → 38. Swept 40/50/70 to confirm 70 is the tightest
  value that leaves the baseline untouched.

## 7. Eyes-vs-metrics reconciliation (do this before blaming the algorithm)

The longest-running bug ("rear still skids") had **all reconstruction metrics green** — because the **viewer**
was drawing box orientation from the front-anchor *path tangent* (median 158 yaw-accel reversals/turner),
ignoring the emitted heading entirely. Fix was a viewer flag (`useDataHeading`), not the algorithm. Rule:
when the owner sees an artifact the metrics say isn't there, **suspect the render path** (how the box is
oriented/sized/anchored) before re-tuning the reconstruction. The viewer draws the box center-pivoted from a
front anchor computed with the *same* length used to draw, so a mismatch there fakes a motion artifact.

## 8. The tuning surface (env knobs — the whole control panel)

All read in `Sim.IgBridge.Host/Program.cs`; all default to the v5 baseline:
- `IGBRIDGE_SCENARIO` (`_ped/subarea-box`) — scenario dir under `scenarios/`.
- `IGBRIDGE_PEDS` (0), `IGBRIDGE_STEPS` (1200), `IGBRIDGE_DEBUG_VEH` (csv ids).
- `IGBRIDGE_LOOKAHEAD` (3.0 m), `IGBRIDGE_LOOKAHEAD_LENFAC` (0.5), `IGBRIDGE_MAX_LEAD` (70°).
- `IGBRIDGE_POS_SMOOTH` (0.60), `IGBRIDGE_LANEPRED` (0.18), `IGBRIDGE_TURNIN` (0 = off), `IGBRIDGE_HEAD_SMOOTH`
  (0), `IGBRIDGE_LC_TAU` (2.0).
- `IGBRIDGE_BUS_IDS` (∅), `IGBRIDGE_BUS_LEN` (12), `IGBRIDGE_BUS_VCLASS` (bus).
- `IGBRIDGE_FEED_HZ` (10) — feed decimation; `IGBRIDGE_EMIT_HZ` (20) — output rate.
When tuning: change **one** knob, keep the rest at default, and always confirm the default run is still
byte-identical to the saved baseline trace.

## 9. Adding a scenario / test bed

Put non-parity viz beds under `scenarios/_ig/<name>/` (net `net.xml` + `scenario.rou.xml` with **inline**
routes — `RouteDemand` only parses `<vehicle><route edges=…/></vehicle>`, not named routes). These have **no
goldens** and are never touched by `dotnet test`. Generate organic geometry with SUMO
(`netgenerate`/`netconvert`/`duarouter`/`randomTrips.py`); the tool version is irrelevant here since there are
no goldens (version pinning only governs parity goldens). Point the host with `IGBRIDGE_SCENARIO=_ig/<name>`.

## 10. The 2D visualization recipe (precious — how the side-by-side player works)

This is the reusable render pipeline (what **F1** in `IGBRIDGE-RESUME.md` would harden into a standalone
tool). It turns any SUMO net + any SumoSharp entity stream into a single **self-contained HTML file** that
plays back and A/B-compares scenes with pan/zoom/scrub — no server, no external JS, vanilla Canvas 2D.

### 10.1 The three pieces
- **`src/Sim.Viz/template.html`** — the shell + HUD (scene `<select>`, play/pause, restart, speed, color-by-speed,
  the **find** box, the time slider) and two markers: `const REPLAY_DATA = /*REPLAY_DATA*/;` and `/*TEMPLATE_JS*/`.
- **`src/Sim.Viz/template.js`** — the whole front-end (camera, interpolation, rendering, HUD wiring). **Shared**
  with City3D and the ped test beds, so every IgBridge-specific feature is guarded (see §1.7). `node --check` it.
- **A writer** that builds the JSON payload and does the two string-replaces to emit the final HTML. For IgBridge
  that is **`src/Sim.IgBridge.Host/VizExport.cs`** (`WriteSideBySide`); City3D/ped beds use `Sim.Viz`'s own
  `Program.cs`/`SceneGen.cs`. F1 = one writer + input adapters feeding this same template.

### 10.2 The payload data model (what the writer emits)
```
{ scenes: [ sceneA, sceneB, ... ] }              // >1 scene => the HUD shows a toggle
scene = {
  name, desc,                                    // caption text
  view:   [minX, minY, maxX, maxY],              // initial world crop (see 10.5)
  network:{ lanes:[{id,edgeId,index,width,shape:[x0,y0,x1,y1,...],ped}],
            junctions:[{id,shape:[...]}], tls, signals, crossings, pedSignals },
  vdim:   [len, wid],                            // FALLBACK vehicle box when a row carries no dims
  vehIds: [ idForSlot0, idForSlot1, ... ],       // slot-indexed; enables click-to-identify + find/follow
  useDataHeading: true,                          // draw the EMITTED heading, not the path tangent (see 10.4)
  dt,                                            // seconds between frames
  frames: [ frame0, frame1, ... ]
}
frame = {
  v: [ vehRowOrNull per slot ],                  // FIXED slots across frames (same slot == same vehicle)
  d: [ discRow, ... ]                            // pedestrians / obstacles / markers
}
```
**Vehicle row arity is the type tag** (disjoint, so the shared template can tell scenes apart):
- `[x, y, heading]` — 3-tuple, City3D FCD.
- `[x, y, heading, fear]` — 4-tuple, panic-evac (fear tint).
- `[x, y, heading, length, width]` — **5-tuple, IgBridge** mixed traffic: true per-vehicle footprint, so a
  12 m bus draws long and a 5 m car short, each **center-pivoted**. The writer front-anchors the box by
  shifting the emitted **center** forward `½·length` along the heading; the template draws the rect *back*
  by that same `length`, so rotating the rigid rect about the front anchor reproduces an identical
  center-pivoted box at any length.
**Disc row**: `[x, y, radius, kind]` (kind 2 = pedestrian), optionally `…, headingDeg, shape, halfLen, halfWid]`
for oriented shapes (buses/obstacles via `drawShaped`). Discs occupy fixed slots too (stable index-matching).

### 10.3 Conventions (get these right or everything mis-renders)
- **Heading = navi degrees**: 0 = north (+Y), increasing **clockwise**. Unit vector `(sin θ, cos θ)`.
  `worldToScreen` flips Y (`[wx·s+ox, −wy·s+oy]`); box rotation is `ctx.rotate(angleRad − π/2)`.
- **Emit the CENTER**; the IG models pivot on center. z = ground (lane-surface) for vehicles, 0 for peds.
- **Camera** = `{scale, offsetX, offsetY}`; to center world `(wx,wy)`: `offsetX = cw/2 − wx·s`,
  `offsetY = ch/2 + wy·s` (this is what `fitToView` and the find/follow use).

### 10.4 Interpolation & the two heading modes
- **Vehicles**: `interpolatedVehicles(t)` finds the bracketing frames and runs a **centripetal Catmull-Rom**
  through the 4 surrounding centers, **clamped to the segment's own endpoint bbox** (kills spline overshoot —
  a lane-change snap can't bulge onto the sidewalk). Heading has two modes:
  - `useDataHeading:true` (IgBridge) → interpolate the **emitted** heading along the shortest arc. Use this
    whenever the stream's heading is authoritative (our kinematic body heading). Drawing the *path tangent*
    instead was the entire "skidding" bug (§7).
  - default (City3D) → heading from the path delta, with a lateral-snap guard that keeps the reported heading
    during a sideways lane-change slide.
- **Discs**: linear interpolation (crowd frames are dense); heading from segment delta.

### 10.5 The viewer features (what the HUD gives you)
- **Scene toggle A/B** — the raw-fed-IG vs IgBridge-fed-IG comparison. **Camera pan/zoom AND playback position
  are preserved across the toggle** (`loadScene(idx, preserveView=true)`) so you compare the same place at the
  same moment; only the initial load auto-fits.
- **find box** — type a vehicle id (forgiving: `213`→`v213`); it latches + **follows** (re-centers every frame
  so the vehicle can't hide outside the crop) and rings/labels it. Clearing or clicking a vehicle releases it.
- **click-to-identify** — nearest-vehicle hit-test → latched id, survives the scene toggle.
- **color** — passenger blue; **long vehicles (len ≥ 7 m) amber** so buses are findable at a glance; optional
  color-by-speed heatmap; panic-evac fear tint. Highlight ring scales with the vehicle's own length.
- **transport** — play/pause (space), restart (r), arrow-step, speed 0.25–8×, scrub slider.
- **auto-crop view** — `view` box is the ~median-centered activity window (`±160 m`) so cars aren't specks;
  entities outside simply draw off-view (the find/follow box is how you chase one down).

### 10.6 The FakeIg (why the comparison is honest)
Both scenes are **`FakeIg` reconstructions** — the deterministic 2-sample linear interpolator that models a
non-predictive downstream IG (a playout `DelaySeconds` ≈ 0.75 s + a `JumpThresholdMeters` teleport cutoff, resampled
at `RenderHz`). The **only** difference between the two scenes is the *input stream* (raw engine @ feed-rate
vs IgBridge-smoothed @ emit-rate), so toggling shows exactly what IgBridge buys through an unchanged, honest
consumer. This is also why emit rate matters (§4B): the IG only linear-interpolates, so denser emit rounds
curves better (20 Hz is the sweet spot).

### 10.7 Reproduce / extend
`dotnet run --project src/Sim.IgBridge.Host -c Release` writes `artifacts/igbridge/sidebyside.html`
(startT 20 s, endT 80 s, 15 fps). To add a layer (POIs, parking, crosswalks — the F1 wishlist): parse the
scenario's `*.add.xml`, add arrays to the `network`/`frame` payload, and draw them in `template.js` behind a
`scene.<flag>` guard so the other consumers stay byte-identical. Keep the payload self-contained (no external
fetches) so the HTML stays a single portable file.

## 11. Where the rest lives
- **As-built mechanisms** (why each stage exists, the algorithms): `IGBRIDGE-DECISIONS.md` §5.3–5.13.
- **Milestones / baselines** (v1–v5, owner sign-offs, metrics per version): `IGBRIDGE-VERSIONS.md`.
- **Live state + next items** (F1 reusable viz tool, F2 articulated vehicles, F3 look-ahead on curves):
  `IGBRIDGE-RESUME.md`.
