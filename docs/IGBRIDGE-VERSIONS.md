# IgBridge — version markers

Milestone tags for the IgBridge PoC. The git tag `igbridge-v1-usable` was created locally on the
commit below; the remote git proxy only accepts pushes to the working branch (tag/other refs 403),
so this file is the durable, pushed marker.

## `igbridge-v1-usable` — commit `c593558`
**First owner-signed-off usable version.** Owner: *"non-sharp turns now looking very real! mark this
version pls, first one usable."*

What it delivers (all render-side; parity 654 pass / 4 skip byte-identical):
- Deterministic 10 Hz SumoSharp → IG-native sample stream `[entityId, x, y, z, headingDeg, t]` with
  SUMO motion artifacts baked out.
- Kinematic no-slip rear-axle drag — rear tracks *inside* the front arc like a real car (drawn
  rear-bumper slip matches an ideal bicycle to <0.15°); substepped for high-yaw fidelity (§5.5).
- Zero-lag constant-velocity (g-h) front tracking — no post-turn "beginner-driver" overshoot (§5.6).
- Gentle lane changes via a bounded decaying-lateral-error model — SUMO's instant ~3.2 m lateral snap
  becomes a ~10–25 °/s yaw instead of a 70–130 °/s rotation jump, with no reseed spikes (§5.8).
- Clean 6×6 grid test bed (`subarea-box`), FakeIg 2-sample replay, side-by-side `Sim.Viz` render with
  click-to-identify.

Known residuals at this tag (see §5.9):
- Sharp low-radius turns: the rear off-tracks aggressively (faithful to the bicycle model, since the
  front follows the lane centerline — a real driver would take a wider turn-in line). Refinement:
  anticipatory turn-in.
- Fixed just after the tag (T2.0f): stationary vehicles drifted the center ~6 cm ("dancing on the
  spot" / "backward movement") — the g-h velocity overshooting a hard decel. Resolved by clamping the
  tracked front velocity to the known vehicle speed.

## `igbridge-v2-usable` — commit `eab3ddc`
**Second usable baseline.** Owner: *"dance is gone. Another one usable."* Adds to v1 the
stopped-vehicle fix (front-velocity clamp → a halted car holds position exactly, no center drift);
fleet maxes improved (lat-accel max 101 → 58). Render-side only; parity 654/4-skip byte-identical.
Known residual carried forward: sharp low-radius turn-in line (see §5.9 / the experimental turn-in below).

## `igbridge-v3-usable` — commit `d7ee8c6`
**Third usable baseline.** Owner: *"best result so far! very good candidate!"* Two decisive fixes:
- **Render draws the emitted kinematic heading, not the path tangent** (§5.11) — the real cause of the
  long-running "skidding": the viewer derived orientation from a jittery front-anchor tangent (median 158
  yaw-accel reversals/turner vs the emitted stream's 0). All the kinematic work became visible.
- **Front stays in its lane through turns via lane-heading prediction** (§5.12) — predicting along the lane
  arc (not a straight tracked velocity) removes the corner-overshoot that drifted the front ~1.9 m toward the
  parallel lane; now ~1.1 m (in-lane). Anticipatory turn-in off by default (redundant with this).
Render-side only; parity 654/4-skip byte-identical; fleet reversals median 0, no-slip 10.5° vs ideal.
Known residual: the front tracks ~1.1–1.5 m off the CONNECTING-lane centerline through a junction (turns in a
touch late / overshoots, then compensates over ~3 s) — the reactive predictor doesn't yet read the path
*through* the junction. Addressed by spatial look-ahead (§5.13).

## `igbridge-v4-usable` — commit `87418e4`
**Fourth usable baseline — prediction-driven anticipation.** Owner: *"wow… looks realistic for the cars and
for the junctions."* Adds to v3 the **spatial look-ahead** (§5.13): the front predictor aims at a point 3 m
ahead ON the upcoming lane centerline — free from `PoseResolver` (advance the front arc position; the chord
to it is the predictor direction), the same forward path a DDS producer ships to IGs. A frame-to-frame jump
guard rejects the occasional spurious cross-junction resolve. Cars now ride the connecting-lane centerline
through junctions (v49 offset 1.15 → 0.49 m, v229 1.52 → 0.33 m — no late/overshoot-and-compensate) AND the
fleet is smoother than v3 (mean yaw-accel reversals 0.48 → 0.31, lat-accel max 70 → 32, yaw-jerk max 2244 →
1629). Deterministic (two runs byte-identical); render-side only; parity 654/4-skip byte-identical.
`LookAheadMeters = 0` falls back to exactly v3.

## `igbridge-v5-usable` — commit `c663996`
**Fifth usable baseline — long vehicles + a usable A/B viewer.** Owner: *"very nice long vehicle turning,
realistic, very good job! … current state is very good, a new baseline."* Adds Direction 1 (longer
vehicles) plus viewer polish, all render-side; parity 654/4-skip byte-identical, cars byte-identical to v4.

Long-vehicle probe (`IGBRIDGE_BUS_IDS` promotes named demand ids to a bus/coach/truck/trailer vType;
`IGBRIDGE_BUS_LEN`, `IGBRIDGE_BUS_VCLASS`):
- The kinematic reconstruction needed **no core change** — wheelbase (0.6·length) and center offset
  (length/2 behind front) already scale with length, so a 12 m body off-tracks correctly. Verified on
  promoted buses v213/v239: rear-**axle** no-slip holds (median 0.05–0.06 m/s lateral, tighter than a 5 m
  car's 0.095); turns smooth (0–2 yaw-accel reversals); the larger rear-**bumper** sweep is real
  rear-overhang tail-swing (bumper 0.4·length behind the axle) and scales with length.
- **Length-scaled look-ahead**: effective look-ahead = `max(LookAheadMeters, LookAheadLengthFactor·length)`
  (0.5). A ~5 m car stays on the flat 3 m (byte-identical v4); a 12 m bus anticipates ~6 m and starts its
  swing earlier (in-turn lead 37–55° over the reactive lane heading).
- **True per-vehicle footprint in the render**: VizExport emits a 5-tuple `[x,y,heading,length,width]`;
  `template.js` draws each at its own length (center-pivoted; long vehicles in amber). 3-/4-tuple (City3D /
  panic-evac fear) paths untouched.

Viewer: **find-by-id follow box** (type `v213` → pans + rings + follows the vehicle so it can't hide
off-view) and **camera+time sync across the raw↔IgBridge toggle** (A/B compares at the same place & moment).

`IGBRIDGE_BUS_IDS` empty ⇒ exactly v4.
