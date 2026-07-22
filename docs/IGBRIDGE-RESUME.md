# IgBridge — RESUME / working state (for continuing after compaction)

**Point here to resume.** This captures exactly where the IgBridge PoC stands, how to drive it, and the two
next exploration directions with concrete plans.

## Where we are

Branch `claude/spectacle-ig-binding-poc-cf3fm4`. Current baseline **`igbridge-v5-usable`** (commit
`c663996`) — owner-signed-off ("very nice long vehicle turning, realistic … a new baseline"). Local git tags
`igbridge-v1..v5-usable` exist; **the remote git proxy rejects non-branch refs (tags 403)**, so milestones
live in `docs/IGBRIDGE-VERSIONS.md` (the durable record) — keep tagging locally AND appending there.

**Direction 1 (longer vehicles) is DONE and shipped in v5** — see the v5 entry in VERSIONS.md. Env:
`IGBRIDGE_BUS_IDS=213,239` (promote demand ids to a long vehicle), `IGBRIDGE_BUS_LEN` (12),
`IGBRIDGE_BUS_VCLASS` (bus|coach|truck|trailer), `IGBRIDGE_LOOKAHEAD_LENFAC` (0.5). Viewer: amber long
vehicles + a find-by-id follow box + camera/time sync across the A/B toggle.

**Direction 2 (organic nets) is DONE** — test beds under `scenarios/_ig/` (`organic` = 112 irregular
junctions, `round12` = 12-gon roundabout, `roundabout` = diamond). Result: reconstruction generalizes off
the grid — fleet yaw-accel reversals median 0 (85% clean on organic, 67% on the roundabout), rear-axle
no-slip holds, deterministic. **Finding → F3 below** (look-ahead under-used on sustained curvature). Next
open item is F3 (optional) or whatever the owner picks; the PoC's requested directions are all delivered.

### F3 — Look-ahead on sustained curvature (roundabouts): temporal-smooth instead of hard-reject
On roundabout rings the spatial look-ahead is REJECTED ~45% of the time: the ring is a chain of short
per-node junction connectors, and the look-ahead point hops onto them, jumping the predictor bearing past
the 250°/s frame-to-frame guard, so it falls back to the reactive lane heading. This does NOT visibly hurt
(the reactive predictor is smooth on continuous curvature — the look-ahead's real win is *faceted* junctions,
which a ring doesn't have), so it's parked, not urgent. Fix per DECISIONS §5.13: instead of a hard reject,
**temporally smooth** the look-ahead heading (low-pass / rate-limit toward the new bearing) so a genuine
sustained curve is followed while transient cross-connector blips are still damped. Gate: the ring
look-ahead-rejection rate should drop well below 45% with fleet reversals staying median 0.

The IgBridge subsystem: embeds SumoSharp, ticks the core at 10 Hz over the clean **6×6 grid `subarea-box`**,
reconstructs each vehicle into a smooth IG-native `[id, x, y, z, headingDeg, t]` stream with all SUMO motion
artifacts baked out, replays it through a FakeIg 2-sample interpolator, and renders a side-by-side
`Sim.Viz` HTML. **All render-side; `dotnet test` parity stays 654 pass / 4 skip byte-identical.**

### The reconstruction pipeline (per emitted sample), all in `KinematicHeading.Update`
1. **Lane-change lateral snap** absorbed into a bounded decaying error `E` (input = raw − E is continuous).
2. **Front position tracker**: predict the front advancing at `speed` along a **low-passed lane DIRECTION**
   (EMA `LanePredictSmoothTime`), correct by a critically-damped position gain (`PositionSmoothTime`). The
   predictor direction is the **spatial look-ahead heading** when available (see below) — that's what makes
   the front ride the connecting-lane centerline through junctions.
3. **Substepped no-slip rear-axle drag**: front tows a rear axle that can't slip → rear tracks inside like a
   real car; body heading = rear→front. Center placed ½·length behind the front bumper.
4. **Anticipatory turn-in stage** — present but **OFF by default** (`TurnInSmoothTime = 0`); the look-ahead
   predictor already gives the driver's line.

**Spatial look-ahead** (`IgBridgeSession`): resolve a second front pose with the front arc position advanced
by `LookAheadMeters` (3 m) — `PoseResolver` walks the upcoming lanes, so the point lands down the connecting
lane; the chord front→point is the predictor direction passed to `KinematicHeading` as `predictHeadingDeg`.
Guarded by a **frame-to-frame jump limit** (250°/s vs the previously-used heading) that rejects spurious
cross-junction resolves.

**Render heading fix** (`Sim.Viz/template.js`): the viewer draws the **emitted heading** (per-scene
`useDataHeading` flag), NOT the front-anchor path tangent — the tangent was jittery (median 158 yaw-accel
reversals/turner) and was the real "skidding". Guarded; other Sim.Viz consumers unchanged.

### Key files
- `src/Sim.Viewer.Motion/KinematicHeading.cs` — the whole kinematic reconstruction + tunables
  (`KinematicHeadingParams`). Shared with the City3D viewer ("fix once, fix both").
- `src/Sim.IgBridge/IgBridgeSession.cs` — emit loop, lane-change resolve, **look-ahead** (`LookAheadMeters`,
  `TryLookAheadHeading`, jump guard `_lookAheadPrev`), debug CSV rows.
- `src/Sim.IgBridge/IgBridgeRunner.cs` — 10 Hz core loop, demand (`RouteDemand`), `EnablePeds`.
- `src/Sim.IgBridge.Host/Program.cs` — env knobs, debug dump, metrics, render.
- `src/Sim.IgBridge.Host/VizExport.cs` — two-scene REPLAY_DATA (`useDataHeading`, `vehIds` click-to-id).
- `src/Sim.Viz/template.js` — shared viewer (the `useDataHeading` branch ~line 479).
- Docs: `IGBRIDGE-DECISIONS.md` (§5.3–5.13 = the as-built story), `-TASKS.md`, `-TRACKER.md`, `-VERSIONS.md`.

### Feed/emit-rate tests (owner: "run at 1 Hz", "emit 10 Hz")
`IGBRIDGE_FEED_HZ` (default 10) decimates the rate state is sampled into the reconstruction + raw stream
(core still steps at 10 Hz, so dynamics are correct); `IGBRIDGE_EMIT_HZ` (default 20) sets the output rate.
Result (grid, display-level FakeIg metrics): a **1 Hz feed** is catastrophic for the *raw* dumb-IG path
(lat-accel median 1533, vel-gap 833 m/s — it teleports across 1 s gaps) but the prediction-driven
reconstruction from the SAME feed stays smooth (lat-accel 21, vel-gap 4.5 — 71–184× better, near the 10 Hz
baseline). **Emit 10 Hz** stays smooth (median yaw-rate unchanged) but ~doubles lat-accel/yaw-jerk vs 20 Hz
(the FakeIg only linear-interpolates 2 samples, so denser emit rounds curves better; 20 Hz is the sweet spot).
**Combined 1 Hz feed + 10 Hz emit** (the coarsest realistic case) still holds: lat-accel median 39 (vs raw
1533 = 39×), vel-gap 4.4 (188×), median yaw-rate 55, dance-free (the lead-bound holds), deterministic.
Fixed a latent bug the coarse feed exposed: the emit lookahead now tracks the feed interval (was fixed 0.1 s),
else at 1 Hz `tau > newest` spuriously retired live vehicles (emitted 0).

**Look-ahead lead-bound (`IGBRIDGE_MAX_LEAD`, default 70°):** the per-frame jump guard only catches SUDDEN
look-ahead hops; on a coarse feed the look-ahead can DRIFT gradually (each step under the guard) to a bearing
pointing the wrong way through a junction, dragging the body off then snapping back (the "dance", owner saw
v90 @1 Hz). Rejecting a look-ahead that diverges > MaxAnticipationLeadDeg from the reactive lane heading kills
it (v90 peak yaw 243→64°/s). Legit anticipation ≤ ~60° at 10 Hz, so 70° leaves the v5 baseline byte-identical.
This is the same family of fix F3 wants for roundabouts (drift on sustained curvature).

**Coarse-feed junction-straddle absorption (`CoarseFeed` + `MaxStraddleLaneChangeHeadingDeg`=20°):** owner
saw v250 @1 Hz ride the lane-division line for ~5 s after a left turn. A junction-turn straddle spans ~1 s of
travel at a coarse feed, so its lateral arc motion was absorbed into the lane-change error E (τ=2 s), parking
the front ~1.8 m off-lane. Fix (coarse-feed only, so v5 byte-identical): a straddle whose incoming/outgoing
lane poses diverge > 20° is a turn, not a lane change → not absorbed (v250 off-lane integral 6.30→2.02 m·s).
**Latent v5 finding (→ possible F4):** at the dense feed the SAME junction straddles are absorbed too, parking
the front ~0.3 m off-lane through *every* junction (84% of samples move ~0.32 m if the discriminator is
applied at 10 Hz). Removing that would be a real junction-fidelity improvement but is a **new baseline (v6)** —
needs owner sign-off, not silent. Parked as F4 in case we want it.

### Env knobs (all read in `Program.cs`)
`IGBRIDGE_SCENARIO` (default `_ped/subarea-box`), `IGBRIDGE_PEDS` (0), `IGBRIDGE_STEPS` (1200 = 120 s),
`IGBRIDGE_LOOKAHEAD` (3.0; 0 = v3), `IGBRIDGE_POS_SMOOTH` (0.60), `IGBRIDGE_LANEPRED` (0.18),
`IGBRIDGE_TURNIN` (0 = off), `IGBRIDGE_DEBUG_VEH` (comma list of ids → per-id CSV in `artifacts/igbridge/`).

### Commands
- Build/run + render: `dotnet run --project src/Sim.IgBridge.Host -c Release` → `artifacts/igbridge/sidebyside.html`.
- Gates: `dotnet test tests/Sim.Viewer.Motion.Tests` (11), `tests/Sim.IgBridge.Tests` (11),
  `tests/Sim.ParityTests` (654/4 byte-identical — the iron law).
- Debug CSV columns: `t,prX,prY,prH,smX,smY,cX,cY,cH,rearX,rearY,speed,laH` (prH=raw lane heading,
  cH=emitted body heading, laH=look-ahead predictor heading, sm*=g-h front, c*=emitted center).

### Diagnostic recipes (the working method: measure, don't eyeball)
- Times the owner quotes are **slider time = real − 20 s** (render window is 20–80 s).
- Per-vehicle: `IGBRIDGE_DEBUG_VEH=v49 dotnet run …` then read `artifacts/igbridge/debug_v49.csv`.
- Standard metrics (python over `trace.jsonl` / debug CSVs): fleet **yaw-accel reversals** (>30°/s² sign
  flips over clean ~90° turns — the smoothness gate), **no-slip** = drawn rear-bumper slip vs an ideal
  bicycle, **off-track** = rear/front path-length ratio, **offset-from-centerline** = emitted front vs raw
  front, **backward-while-stopped** = center fwd-step at speed<0.5. Keep median-0 reversals + parity green.

## Next: two exploration directions (owner-requested)

### Direction 1 — longer vehicles (a bus) on a sharp turn
**Goal:** verify/tune the kinematics for a long vehicle (~12 m bus) making a sharp junction turn — it should
need more off-tracking / lateral shift and effectively a later turn-in, like a real bus swinging wide.
**Why it may need work:** wheelbase `lwb = WheelbaseFactor·length` and the center placement scale with
`length`, so off-tracking scales correctly in principle — but `LookAheadMeters` is a *fixed 3 m* and the
lane-change/ reseed thresholds are fixed metres; a 12 m bus wants look-ahead ≈ its own length and may bump
lane geometry (a bus won't fit a 4.9 m-radius grid corner — SUMO itself may refuse/adjust).
**Plan:**
1. Add a longer vType (bus, length ~12 m, appropriate width) — either in the scenario's vType config or a
   `DefineVType` in `IgBridgeRunner`, and spawn one on a route with a sharp turn (reuse an existing turning
   route; the runner's `SpawnVehicle` takes explicit edges). Capture its dims (already done via `VehicleDims`).
2. Instrument it; check: rear off-tracking scales with length (rear cuts further inside), no-slip still
   matches an ideal bicycle *at that length*, the turn-in isn't clipping/oscillating.
3. Likely tune: make `LookAheadMeters` scale with vehicle length (e.g. `max(3, 0.5·length)`) so a bus
   anticipates proportionally; re-check the jump guard still holds. Keep passenger cars unchanged.
4. Consider whether the grid corner is even geometrically drivable by a 12 m bus (may need Direction-2 net).
**Watch for:** a long body amplifies any heading wiggle (the tail lever is longer) — the reversal gate is the
guard. And the IG car models pivot at CENTER (owner's constraint) — the bus box must stay center-pivoted.

### Direction 2 — organic road net (roundabouts, irregular junctions)
**Goal:** stress the reconstruction on non-grid geometry — roundabouts, Y-junctions, curved/irregular
connectors — to see what breaks (the grid is very regular; real nets aren't).
**Plan:**
1. Pick/build a net. Committed candidate: `scenarios/32-roundabout` (has a roundabout) but only ~2 vehicles
   — too sparse. Better: **generate one with SUMO** (available per CLAUDE.md; `netgenerate --rand` or a
   spider/roundabout net + `randomTrips`/`duarouter` for explicit-route demand the runner can parse). Put it
   under `scenarios/` and point `IGBRIDGE_SCENARIO` at it. (The runner needs `<vehicle><route edges>` demand;
   duarouter emits that.)
2. Run and eyeball + gate: reversals, no-slip, offset-from-centerline, look-ahead jump-guard rejection rate.
   Roundabouts = sustained curvature (not facet steps) → the lane-heading predictor and look-ahead should
   love them; irregular junctions may expose new spurious-look-ahead-point cases (tune the jump guard /
   `LookAheadMeters`).
3. Specifically check: continuous-curvature tracking (roundabout), multi-way junctions (look-ahead picking
   the right connecting lane), very short connectors (look-ahead point overshooting the connector).
**Watch for:** the look-ahead point stability on tight/curved connectors (the jump guard is the safety net;
may need the temporal-smoothing upgrade noted in §5.13 if the guard rejects too often and the front reverts
to reactive tracking on a curve).

### IG consumption models: interpolate (buffered) vs extrapolate (zero-latency DR)
`IGBRIDGE_EXTRAP=1` renders the buffered **interpolating** IG (0.75 s playout delay) against a zero-latency
**extrapolating** IG on the same stream (`FakeIgConfig.Extrapolate`): the extrapolating IG dead-reckons
forward from the last received sample with a constant-speed constant-yaw-rate coordinated-turn model
(speed+yaw estimated from the last two samples). Consumer-side only — emitted trace byte-identical. Finding:
zero latency is nearly free at a high emit rate and pays at a low one; the DR lead/snap error ≈ speed·emit
interval, concentrated at turns (straight-line DR is exact). DR-error max **0.30 m @20 Hz, 0.57 @10 Hz, 1.22
@5 Hz, 2.87 @2 Hz**; extrap lat-accel runs ~2–4× the interpolated (the per-update correction snaps). So for
the low-latency default, keep the emit rate up (≥10–20 Hz) and the snaps stay sub-metre / invisible.

## Future work (owner-requested, parked — do when scheduled, not now)

### F1 — One reusable SumoSharp visualization tool (consolidate the ad-hoc players)
Owner (v5): *"This player would be nice to have as a separate tool … it would need to show pedestrians,
crosswalks, POIs, parking places etc. … taking all the ad-hoc players and make one reusable tool from them
(something that consumes all the road nets and SumoSharp outputs and produces the HTML)."*
**Scope:** promote the side-by-side `Sim.Viz` HTML player (currently driven ad-hoc by `Sim.IgBridge.Host`
`VizExport` and separately by `Sim.Viz` `SceneGen`/`Program` for the ped test beds) into ONE reusable
command-line tool: input = a SUMO net + SumoSharp outputs (vehicle + ped streams, and the scenario's
`*.add.xml` for POIs / parking areas / crossings); output = the self-contained HTML. It must render
**pedestrians, crosswalks/zebra, POIs, parking areas** (the ped test bed already draws peds + crossings;
`scenario.add.xml`/`pois.add.xml` carry POIs + parking) as well as the vehicle layer. Design-first per
CLAUDE.md: a design doc for the unified scene payload + input adapters, then tasks. Note `template.js`/
`template.html` are already the shared front-end; the work is mostly a clean **input/adapter layer** + a CLI,
not new rendering.

### F2 — Render-side articulated vehicles (tractor + trailer hinge)
Neither SUMO nor SumoSharp simulate articulation — a vehicle is ONE rigid `length` (vClass `trailer` is just
16.5 m rigid; SUMO's GUI "carriages" for trains rigidly follow the lead path, no joint dynamics). If a
believable **articulated** truck/bus is wanted, model it **render-side** as two coupled no-slip bicycle
segments (tractor + trailer joined at a fifth-wheel/hinge: the trailer's front pinned to the tractor's rear
axle, each segment no-slip). Extends `KinematicHeading`; emit two boxes (or a hinged shape) to the viewer.
This would show the trailer cutting further inside than a rigid body on a sharp turn (classic off-tracking).
Not needed for the current PoC; a rigid 16.5 m `trailer` already off-tracks (just without the mid-body bend).

## How to resume
Read this file + **`IGBRIDGE-METHODOLOGY.md`** (how we tune — invariants, metrics, the diagnostic loop, and
the 2D-visualization recipe) + `IGBRIDGE-DECISIONS.md` §5.11–5.13 + `IGBRIDGE-TRACKER.md` (T2.x). Confirm on branch
`claude/spectacle-ig-binding-poc-cf3fm4` at/after `87418e4`. Run the render + gates once to re-baseline, then
pick Direction 1 or 2. Keep the discipline: measure numerically, gate median-0 reversals + parity byte-
identical, commit + append to VERSIONS.md at each usable step, ask the owner in plain chat (no widgets).
