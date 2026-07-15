using Sim.Core;

namespace Sim.Viewer.Motion;

// Packaging P2-C (docs/SUMOSHARP-PACKAGING-DESIGN.md §5): extracted VERBATIM from the per-vehicle
// smoothing block in `PumpAndBuildVehicleDraws` (src/Sim.Viewer/Program.cs) so a game/3D engine can reuse
// the as-built DR pose-smoothing pipeline without the viewer's raylib/DDS plumbing. See
// docs/SUMOSHARP-VIEWER-DR-SMOOTHING.md §10.2 (position error-smoothing) and §10.3 (motion-derived
// heading tilt) for the design rationale; this is a pure code move, no behaviour change.
//
// Per-vehicle, per-frame: the rendered pose chases the resolved+posed DR target with (1) a CAPPED,
// forward-biased position correction (netcode-style: zero lag on smooth motion, a reconciliation snap is
// absorbed over a few frames, never a freeze or a reverse), (2) a heading tilt derived from the vehicle's
// ACTUAL render motion this frame (leans into a lateral slide, both directions), then (3) a heading
// low-pass toward the tilt-adjusted target. The first observation of a handle returns the target pose
// unchanged (and remembers it) -- there is no previous frame to correct from.
public sealed class DrPoseSmoother
{
    private readonly Dictionary<VehicleHandle, (float X, float Y, float Deg)> _smoothed = new();

    // Returns the smoothed render pose for `handle` this frame. `targetX/Y` + `targetDeg` are the
    // resolved+posed target (targetDeg = the lane-forward heading, e.g. PoseResolver's ChordHeading or the
    // lateral-straddle lane-lerp); `speed` is resolved.State.Speed; `frameDt` the frame delta (seconds).
    public (double X, double Y, float Deg) Smooth(
        VehicleHandle handle, double targetX, double targetY, float targetDeg, double speed, float frameDt)
    {
        double px = targetX, py = targetY;
        float pdeg = targetDeg;

        if (_smoothed.TryGetValue(handle, out var prevPose))
        {
            var laneHr = pdeg * MathF.PI / 180f;               // lane-forward heading (PoseResolver / lane lerp)
            float lhx = MathF.Sin(laneHr), lhy = MathF.Cos(laneHr);

            // (1) POSITION error-smoothing (netcode-style, capped correction): the rendered position chases the
            // DR target but its correction speed is CAPPED, so smooth constant-speed motion passes through with
            // ZERO lag while a reconciliation snap is absorbed over a few frames instead of teleporting. Forward-
            // biased: a backward correction is a gentle ~50% slowdown (floor 0.5*trueStep), never a reverse or a
            // freeze. Lateral catch-up eases both ways, capped. A >7 m gap snaps (respawn). Moved AHEAD of the
            // heading step so the motion-tilt below sees the FINAL rendered displacement.
            var tx = (float)px - prevPose.X;
            var ty = (float)py - prevPose.Y;
            if (tx * tx + ty * ty <= 49f)
            {
                var along = tx * lhx + ty * lhy;               // longitudinal part of the needed correction
                var perp = tx * -lhy + ty * lhx;                // lateral part
                var trueStep = (float)speed * frameDt;
                var fwdCap = trueStep + 6f * frameDt;          // real travel + 6 m/s catch-up
                var latCap = 4f * frameDt;                     // 4 m/s lateral catch-up
                along = Math.Clamp(along, 0.5f * trueStep, fwdCap);
                perp = Math.Clamp(perp, -latCap, latCap);
                px = prevPose.X + along * lhx + perp * -lhy;
                py = prevPose.Y + along * lhy + perp * lhx;
            }

            // (2) MOTION-DERIVED heading tilt: lean toward the vehicle's ACTUAL render motion this frame, so a
            // lane-change slide leans into the change in BOTH directions (the outbound straddle AND the return,
            // which is rendered via extrapolation + the lateral error-smoothing above and otherwise stays pinned
            // to the lane tangent). Decompose the final render displacement in the lane-heading frame; tilt =
            // atan2(perp, along), capped. ~0 on straight cruise and on turns (motion follows the lane).
            var mdx = (float)px - prevPose.X;
            var mdy = (float)py - prevPose.Y;
            var mAlong = mdx * lhx + mdy * lhy;
            var mPerp = mdx * -lhy + mdy * lhx;
            if (mAlong > 0.01f)
            {
                var tiltDeg = (float)(Math.Atan2(mPerp, mAlong) * 180.0 / Math.PI);
                tiltDeg = Math.Clamp(tiltDeg, -25f, 25f);
                // SUBTRACT: navi heading is clockwise (0=N, 90=E) but atan2 is counter-clockwise, so the
                // lateral tilt must be negated or the lean shows the wrong way (car pointing right while
                // sliding left, and vice versa).
                pdeg = (pdeg - tiltDeg + 360f) % 360f;
            }

            // (3) heading low-pass toward the (tilt-adjusted) target: ease over ~0.18 s; a >100 deg jump snaps.
            var aHead = 1f - MathF.Exp(-frameDt / 0.18f);
            var dh = ((pdeg - prevPose.Deg + 540f) % 360f) - 180f;
            pdeg = MathF.Abs(dh) > 100f ? pdeg : (prevPose.Deg + dh * aHead + 360f) % 360f;
        }

        _smoothed[handle] = ((float)px, (float)py, pdeg);
        return (px, py, pdeg);
    }

    public void Clear() => _smoothed.Clear();
}
