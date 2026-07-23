using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// The PathArc dead-reckoning model (docs/PEDESTRIAN-DESIGN.md §7; docs/PEDESTRIAN-POC-PLAN.md POC-3):
// a pedestrian's position along a fixed polyline is a PURE function of (path, startTime, speed, now) --
// no neighbour state, no System.Random, no hidden mutable state. This is the load-bearing identity the
// whole POC rests on: the server calls THIS SAME function to advance a low-power ped, and the headless
// IG calls it again to reconstruct that ped from the one-time path broadcast -- so "server == IG for
// low-power" follows from sharing the code, not from independently matching two implementations.
//
// Allocation-light: no LINQ, no per-call allocation, a single forward walk of the polyline.
public static class PathArcMotion
{
    // World position at arc-length speed * max(0, now - startTime) along `path`, clamped at the final
    // vertex once that arc-length exceeds the path's total length.
    public static Vec2 PositionAt(IReadOnlyList<Vec2> path, double startTime, double speed, double now)
    {
        var s = ArcLength(startTime, speed, now);
        return Walk(path, s, out _);
    }

    // Direction of the segment the walk currently sits on, times `speed` -- Vec2.Zero once clamped at
    // the final vertex (the agent has arrived and stopped).
    public static Vec2 VelocityAt(IReadOnlyList<Vec2> path, double startTime, double speed, double now)
    {
        var s = ArcLength(startTime, speed, now);
        Walk(path, s, out var direction);
        return direction * speed;
    }

    // W1 (docs/PEDESTRIAN-WEAVE-PRODUCTION-DESIGN.md §1, §3): the weave-aware sample. One forward walk of
    // `path` to arc-length `s` returns the centreline point AND the unit tangent AND the interpolated
    // half-width there -- everything LateralWeave needs to place `centre + tangent.PerpCW * offset`, without a
    // second walk. `halfWidths` is per-vertex (parallel to `path`); when null the corridor half-width is 0
    // (weave OFF -> pose is exactly the centreline, byte-identical to PositionAt). This is the SHARED evaluator
    // both the server (PedLodManager.PositionOf -> ActivityTimeline.PoseAt) and the IG
    // (HeadlessIg.ReconstructSample -> the decoded PoseAt) call, so server==IG holds by construction.
    public static Vec2 SampleAt(
        IReadOnlyList<Vec2> path, IReadOnlyList<double>? halfWidths, double s, out Vec2 tangent, out double halfWidth)
    {
        tangent = Vec2.Zero;
        halfWidth = 0.0;

        if (path.Count == 0)
        {
            return Vec2.Zero;
        }

        if (path.Count == 1)
        {
            halfWidth = halfWidths is { Count: > 0 } ? halfWidths[0] : 0.0;
            return path[0];
        }

        var remaining = s < 0.0 ? 0.0 : s;
        for (var i = 0; i + 1 < path.Count; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            var seg = b - a;
            var len = seg.Abs;
            if (len <= 1e-12)
            {
                continue; // degenerate (duplicate-point) segment: no arc-length, skip it
            }

            if (remaining <= len)
            {
                var t = remaining / len;
                tangent = seg / len;
                if (halfWidths != null && halfWidths.Count == path.Count)
                {
                    halfWidth = halfWidths[i] + ((halfWidths[i + 1] - halfWidths[i]) * t);
                }

                return new Vec2(a.X + (seg.X * t), a.Y + (seg.Y * t));
            }

            remaining -= len;
        }

        // clamped at the final vertex (arrived): tangent stays Zero, half-width is the last vertex's.
        halfWidth = halfWidths is { Count: > 0 } ? halfWidths[^1] : 0.0;
        return path[^1];
    }

    // Pedestrian junction-turn back-step fix (docs/LIVE-CITY-VISUALS-NOTES.md-adjacent): the weave
    // PROJECTION AXIS at arc-length `s`, blended continuously across each interior polyline VERTEX over a
    // `blendMeters`-wide window straddling it, instead of the raw per-segment tangent SampleAt/Walk return
    // (which flips the instant `s` crosses a vertex). Used ONLY by ActivityTimeline.Evaluate to pick the
    // lateral weave's axis (`centre + WeaveAxisAt(...).PerpCW * (c+off)`) -- NEVER for the walked centreline
    // position (still the exact piecewise-linear SampleAt/Walk result, byte-identical) NOR the reported
    // PoseSample.Heading (still the sharp per-segment SampleAt tangent, unaffected).
    //
    // Why this matters: the weave places the pose off the centreline by a roughly-constant lateral distance
    // `c+off` (metres), projected along `tangent.PerpCW`. At a route CORNER the raw tangent rotates
    // instantaneously (one polyline vertex, zero arc-length), so the SAME lateral distance gets re-projected
    // onto a suddenly-rotated axis in a single sample -- a discontinuous Cartesian pop whose direction
    // (relative to the pre-corner heading) can be backward, which is exactly the observed "pauses and steps
    // back at a turn waypoint" glitch. This function fixes that at the root (the shared server==IG pose
    // evaluator both the real low-power ped and its render reconstruction call), two ways at once:
    //
    //   1. CONTINUOUS ROTATION: each interior vertex gets its own bisector axis
    //      ((incomingSegTangent + outgoingSegTangent) / 2), and a segment's own tangent smoothsteps into/out
    //      of its two bounding vertices' bisectors over `blendMeters`. Approaching a vertex from EITHER side
    //      converges to the SAME bisector value (proof: at distFromStart/distFromEnd -> 0 the smoothstep -> 0,
    //      both sides evaluate to Bisector(prevSegTangent, nextSegTangent)), so the axis direction is
    //      C0-continuous through every corner -- no more instantaneous flip.
    //   2. SELF-SHRINKING AMPLITUDE AT SHARP TURNS: the bisector is deliberately left UN-normalized. For two
    //      UNIT vectors `a`,`b` separated by turn angle theta, `(a+b)/2` has magnitude cos(theta/2): 1 at
    //      theta=0 (straight through -- see point 3), shrinking smoothly to 0 as theta -> 180 degrees (a
    //      near-reversal). This matters because #1 alone (a length-PRESERVING rotation) cannot avoid *some*
    //      backward-facing sweep on a sharp-enough turn -- continuously rotating a fixed-length vector through
    //      a large angle necessarily has intermediate directions pointing backward relative to the pre-turn
    //      heading. Shrinking the vector's length in lockstep with how sharp the turn is (down to exactly 0 at
    //      a full reversal, where no rotation could possibly avoid it) removes the backward motion at its
    //      source instead of merely spreading it out. A gentle bend (small theta) is barely shrunk at all.
    //      Because a straight-line interpolation of two vectors each of magnitude <= 1 has magnitude <= 1
    //      (triangle inequality), the blended axis returned here NEVER exceeds unit length either -- so this
    //      only ever REDUCES the weave's lateral magnitude relative to the pre-fix `tangent.PerpCW*(c+off)`,
    //      it never increases it, which is why the sidewalk-half-width clamp invariant
    //      (PedDemandWeaveTests.WeaveOn_StaysWithinSidewalkHalfWidth_AndLeavesCentreline) still holds:
    //      |woven - centre| = |WeaveAxisAt(...)| * |c+off| <= |c+off| <= halfWidth.
    //   3. NO-OP AWAY FROM CORNERS: a single-segment path (no interior vertex), a query more than
    //      `blendMeters` from any vertex, or a vertex that is actually COLLINEAR (theta=0, e.g. a navmesh
    //      path's own straight-line subdivision points) all return the raw segment tangent UNCHANGED (unit
    //      length) -- so straight-line motion, and every existing straight-leg weave test, is untouched
    //      (byte-identical).
    public static Vec2 WeaveAxisAt(IReadOnlyList<Vec2> path, double s, double blendMeters)
    {
        if (path.Count < 2)
        {
            return Vec2.Zero;
        }

        var remaining = s < 0.0 ? 0.0 : s;
        var prevTangent = Vec2.Zero;

        for (var i = 0; i + 1 < path.Count; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            var seg = b - a;
            var len = seg.Abs;
            if (len <= 1e-12)
            {
                continue; // degenerate (duplicate-point) segment -- no arc-length, skip it (mirrors Walk/SampleAt)
            }

            var tangent = seg / len;
            if (remaining <= len)
            {
                // Never let the two ends' blend windows overlap in the interior of a short segment.
                var half = blendMeters < len * 0.5 ? blendMeters : len * 0.5;
                if (half > 1e-9)
                {
                    var distFromStart = remaining;
                    if (distFromStart < half && prevTangent.AbsSq > 0.0)
                    {
                        var vertexAxis = Bisector(prevTangent, tangent);
                        var u = SmoothStep01(distFromStart / half);
                        return Lerp(vertexAxis, tangent, u);
                    }

                    var distFromEnd = len - remaining;
                    if (distFromEnd < half)
                    {
                        var nextTangent = NextNonDegenerateTangent(path, i + 1);
                        if (nextTangent.AbsSq > 0.0)
                        {
                            var vertexAxis = Bisector(tangent, nextTangent);
                            var u = SmoothStep01(distFromEnd / half);
                            return Lerp(vertexAxis, tangent, u);
                        }
                    }
                }

                return tangent; // interior of the segment, away from both ends -- unchanged
            }

            remaining -= len;
            prevTangent = tangent;
        }

        return prevTangent; // clamped beyond the path end (fp edge case) -- last known tangent, else Zero
    }

    // The unit tangent of the first non-degenerate segment at or after `fromIndex`, or Vec2.Zero if `path`
    // has no more segments (fromIndex is at/past the last vertex) -- the "what comes next" half of
    // WeaveAxisAt's corner blend.
    private static Vec2 NextNonDegenerateTangent(IReadOnlyList<Vec2> path, int fromIndex)
    {
        for (var i = fromIndex; i + 1 < path.Count; i++)
        {
            var seg = path[i + 1] - path[i];
            var len = seg.Abs;
            if (len > 1e-12)
            {
                return seg / len;
            }
        }

        return Vec2.Zero;
    }

    // The vertex axis two bounding UNIT segment tangents `a`,`b` converge to, from EITHER side, as
    // WeaveAxisAt approaches their shared vertex. DELIBERATELY the plain (un-normalized) average, not a
    // normalized bisector -- see WeaveAxisAt's remark #2: its magnitude (cos(turnAngle/2)) is exactly the
    // self-shrinking factor that removes the backward sweep on a sharp turn, so re-normalizing here would
    // defeat the whole mechanism.
    private static Vec2 Bisector(Vec2 a, Vec2 b) => (a + b) * 0.5;

    private static Vec2 Lerp(Vec2 a, Vec2 b, double t) => a + ((b - a) * t);

    private static double SmoothStep01(double x)
    {
        var t = x < 0.0 ? 0.0 : (x > 1.0 ? 1.0 : x);
        return t * t * (3.0 - (2.0 * t));
    }

    private static double ArcLength(double startTime, double speed, double now) =>
        speed * Math.Max(0.0, now - startTime);

    // Total arc-length of `path` -- the sum of segment lengths, skipping degenerate (duplicate-point)
    // segments exactly like Walk does. ActivityTimeline (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §2) uses
    // this to size a Walk segment's duration (pathLength / speed) without duplicating this walk.
    public static double PathLength(IReadOnlyList<Vec2> path)
    {
        if (path.Count < 2)
        {
            return 0.0;
        }

        var total = 0.0;
        for (var i = 0; i + 1 < path.Count; i++)
        {
            var len = (path[i + 1] - path[i]).Abs;
            if (len > 1e-12)
            {
                total += len;
            }
        }

        return total;
    }

    // Walks `path` to arc-length `s`, returning the world point and (via `direction`) the unit
    // direction of the segment it landed on. `direction` is Vec2.Zero once `s` reaches or exceeds the
    // path's total length (clamped at the final vertex -- no more motion).
    private static Vec2 Walk(IReadOnlyList<Vec2> path, double s, out Vec2 direction)
    {
        if (path.Count == 0)
        {
            direction = Vec2.Zero;
            return Vec2.Zero;
        }

        if (path.Count == 1)
        {
            direction = Vec2.Zero;
            return path[0];
        }

        var remaining = s;
        for (var i = 0; i + 1 < path.Count; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            var seg = b - a;
            var len = seg.Abs;
            if (len <= 1e-12)
            {
                continue; // degenerate (duplicate-point) segment: no arc-length, skip it
            }

            if (remaining <= len)
            {
                var t = remaining / len;
                direction = seg / len;
                return new Vec2(a.X + (seg.X * t), a.Y + (seg.Y * t));
            }

            remaining -= len;
        }

        direction = Vec2.Zero; // clamped at the final vertex
        return path[^1];
    }
}
