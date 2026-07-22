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
