namespace Sim.Ingest;

// Rung 9b-i: polyline-polyline intersection used to compute junction conflict geometry (the
// crossing point of two internal lanes' shapes, ported conceptually from how
// sumo/src/microsim/MSLink.cpp's myConflicts/getLengthBehindCrossing locate where a link's
// internal lane crosses a foe link's internal lane -- SUMO computes this once at network build
// time (NBRequest/NBNode), we do the equivalent here at ingest time over the already-parsed
// lane shapes).
//
// Only PROPER (transversal) crossings count: two segments that merely touch at a shared
// endpoint -- e.g. two internal lanes that merge into the same downstream lane, ending at the
// same (x,y) -- are foes in the request/foes bitstring (right-of-way still matters at a merge)
// but are not a "crossing" in the geometric sense this helper models; callers skip a foe pair
// when TryIntersect returns false. The standard cross-product parametric test naturally
// distinguishes the two: a proper crossing has both intersection parameters strictly inside
// (0, 1); a shared-endpoint touch lands one (or both) parameters exactly on 0 or 1.
public static class PolylineGeometry
{
    private const double Epsilon = 1e-9;

    public readonly record struct Intersection(double ArcA, double ArcB, (double X, double Y) Point);

    // Scans every segment of `a` against every segment of `b` and returns the first proper
    // crossing found, together with the cumulative arc-length along each polyline to that
    // point. Our callers only ever have at most one true crossing between two internal lanes,
    // so "first" and "only" coincide in practice; segments are scanned in polyline order so a
    // caller relying on "first" gets a stable, well-defined result even in the general case.
    public static bool TryIntersect(
        IReadOnlyList<(double X, double Y)> a,
        IReadOnlyList<(double X, double Y)> b,
        out Intersection intersection)
    {
        var arcA = 0.0;
        for (var i = 0; i < a.Count - 1; i++)
        {
            var a1 = a[i];
            var a2 = a[i + 1];
            var segLenA = Distance(a1, a2);

            var arcB = 0.0;
            for (var j = 0; j < b.Count - 1; j++)
            {
                var b1 = b[j];
                var b2 = b[j + 1];
                var segLenB = Distance(b1, b2);

                if (TrySegmentIntersect(a1, a2, b1, b2, out var t, out var u))
                {
                    var point = (X: a1.X + (a2.X - a1.X) * t, Y: a1.Y + (a2.Y - a1.Y) * t);
                    intersection = new Intersection(arcA + t * segLenA, arcB + u * segLenB, point);
                    return true;
                }

                arcB += segLenB;
            }

            arcA += segLenA;
        }

        intersection = default;
        return false;
    }

    // Standard cross-product segment intersection (p = a1 + t*r, q = b1 + u*s). Only a proper
    // (strictly interior) crossing is reported: t and u must both lie strictly inside (0, 1),
    // which excludes endpoint touches (shared-endpoint merges) and collinear/parallel segments
    // (rxs ~= 0) alike -- neither is a "crossing" for this helper's purposes.
    private static bool TrySegmentIntersect(
        (double X, double Y) a1, (double X, double Y) a2,
        (double X, double Y) b1, (double X, double Y) b2,
        out double t, out double u)
    {
        var rX = a2.X - a1.X;
        var rY = a2.Y - a1.Y;
        var sX = b2.X - b1.X;
        var sY = b2.Y - b1.Y;

        var rxs = rX * sY - rY * sX;
        if (Math.Abs(rxs) < Epsilon)
        {
            // Parallel (or collinear) segments -- no proper crossing.
            t = 0;
            u = 0;
            return false;
        }

        var qpX = b1.X - a1.X;
        var qpY = b1.Y - a1.Y;

        t = (qpX * sY - qpY * sX) / rxs;
        u = (qpX * rY - qpY * rX) / rxs;

        return t > Epsilon && t < 1.0 - Epsilon && u > Epsilon && u < 1.0 - Epsilon;
    }

    private static double Distance((double X, double Y) p1, (double X, double Y) p2)
    {
        var dx = p2.X - p1.X;
        var dy = p2.Y - p1.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }

    // C4-v: total 2D length of a polyline shape (PositionVector::length2D).
    public static double PolylineLength(IReadOnlyList<(double X, double Y)> shape)
    {
        var total = 0.0;
        for (var i = 0; i < shape.Count - 1; i++)
        {
            total += Distance(shape[i], shape[i + 1]);
        }

        return total;
    }

    // C4-v: minimum 2D distance from a point to a polyline (the nearest point on any segment,
    // clamped to segment endpoints) -- PositionVector::distance2D(point, perpendicular=false).
    private static double PointToPolylineDistance((double X, double Y) p, IReadOnlyList<(double X, double Y)> poly)
    {
        var best = double.PositiveInfinity;
        for (var i = 0; i < poly.Count - 1; i++)
        {
            best = Math.Min(best, PointToSegmentDistance(p, poly[i], poly[i + 1]));
        }

        return best;
    }

    private static double PointToSegmentDistance((double X, double Y) p, (double X, double Y) a, (double X, double Y) b)
    {
        var dx = b.X - a.X;
        var dy = b.Y - a.Y;
        var lenSq = dx * dx + dy * dy;
        if (lenSq == 0.0)
        {
            return Distance(p, a);
        }

        var t = ((p.X - a.X) * dx + (p.Y - a.Y) * dy) / lenSq;
        t = Math.Max(0.0, Math.Min(1.0, t));
        var footX = a.X + t * dx;
        var footY = a.Y + t * dy;
        var ex = p.X - footX;
        var ey = p.Y - footY;
        return Math.Sqrt(ex * ex + ey * ey);
    }

    // C4-v (TASKS.md sameTarget-merge conflict geometry): the distance-to-divergence for a
    // sameTarget MERGE, ported VERBATIM from MSLink::computeDistToDivergence
    // (sumo/src/microsim/MSLink.cpp:561-620, the `sameSource=false` arm). Two internal lanes that
    // merge into the same downstream lane end at (nearly) the same point and diverge going
    // backward; this returns the geometric arc-length from the lane's END back to the point where
    // the two lane shapes are `minDist` apart -- the merge's crossing point, from which each lane's
    // `lengthBehindCrossing` is derived (caller divides by the lane's length/shape factor via
    // InterpolateGeometryPosToLanePos). Verified byte-exact against the vendored v1_20_0
    // DEBUG trace lbc/flbc for scenarios/29 (10.827662/10.822642) and 31 (14.747385/14.748960).
    // The `lane`/`sibling` here are the two internal lane SHAPES; `laneLen`/`sibLen` are their LANE
    // lengths (myLength, which may differ slightly from the shape length -- the source uses
    // getLength() for distToDivergence1/2 but the polyline length for the final MIN3 clamp).
    public static double ComputeDistToDivergence(
        IReadOnlyList<(double X, double Y)> laneShape,
        IReadOnlyList<(double X, double Y)> sibShape,
        double laneLen, double sibLen, double minDist)
    {
        var lbcSibling = 0.0;
        var lbcLane = 0.0;

        // sameSource=false: reverse both shapes so they start at the merge point.
        var l = new List<(double X, double Y)>(laneShape);
        l.Reverse();
        var s = new List<(double X, double Y)>(sibShape);
        s.Reverse();
        var length = PolylineLength(laneShape);
        var sibLength = PolylineLength(sibShape);

        if (Distance(l[^1], s[^1]) > minDist)
        {
            // distances: for each vertex of l -> nearest distance to polyline s; then each vertex
            // of s -> nearest distance to l (PositionVector::distances). Size == l.Count + s.Count.
            var distances = new double[l.Count + s.Count];
            for (var i = 0; i < l.Count; i++)
            {
                distances[i] = PointToPolylineDistance(l[i], s);
            }

            for (var i = 0; i < s.Count; i++)
            {
                distances[l.Count + i] = PointToPolylineDistance(s[i], l);
            }

            if (distances[^1] > minDist && distances[l.Count - 1] > minDist)
            {
                // Walk the sibling backward from near-start toward the merge, accumulating segment
                // length until the lanes are within minDist, then interpolate the last segment.
                for (var j = s.Count - 2; j >= 0; j--)
                {
                    var i = j + l.Count;
                    var segLength = Distance(s[j], s[j + 1]);
                    if (distances[i] > minDist)
                    {
                        lbcSibling += segLength;
                    }
                    else
                    {
                        lbcSibling += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                        break;
                    }
                }

                for (var i = l.Count - 2; i >= 0; i--)
                {
                    var segLength = Distance(l[i], l[i + 1]);
                    if (distances[i] > minDist)
                    {
                        lbcLane += segLength;
                    }
                    else
                    {
                        lbcLane += segLength - (minDist - distances[i]) * segLength / (distances[i + 1] - distances[i]);
                        break;
                    }
                }
            }
        }

        var distToDivergence1 = sibLen - lbcSibling;
        var distToDivergence2 = laneLen - lbcLane;
        return Math.Min(Math.Min(Math.Max(distToDivergence1, distToDivergence2), sibLength), length);
    }
}
