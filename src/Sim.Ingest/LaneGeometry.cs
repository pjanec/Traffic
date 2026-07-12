namespace Sim.Ingest;

// Ported from sumo/src/utils/geom/PositionVector.cpp (positionAtOffset) and the naviDegree
// angle convention used by MSVehicle when writing FCD output (GeomHelper::naviDegree): 0 deg
// is north, increasing clockwise, vs. the mathematical atan2 convention (0 deg = +x, CCW).
//
// This is the seam-2 "derive global x/y from lane-relative pos" step (DESIGN.md). Lane-relative
// (lane id, pos) stays the source of truth; this is purely an output-side derivation and must
// never feed back into the kinematic state.
public static class LaneGeometry
{
    // `latOffset` (B6, default 0) shifts the returned point PERPENDICULAR to the lane direction --
    // positive = LEFT of travel (the +90 deg / CCW normal of the segment direction, matching
    // Kinematics.LatOffset's convention). Purely output-side (like the whole method): it moves the
    // rendered x/y so a lateral swerve is visible, and never feeds back into kinematic state. With
    // latOffset == 0 (every vehicle in lane-centred mode) the added terms are exactly 0, so this is
    // byte-identical to the pre-B6 two-argument form.
    public static (double X, double Y, double AngleDeg) PositionAtOffset(
        IReadOnlyList<(double X, double Y)> shape,
        double offset,
        double latOffset = 0.0)
    {
        if (shape.Count == 0)
        {
            throw new InvalidOperationException("Lane shape must contain at least one point.");
        }

        if (shape.Count == 1)
        {
            return (shape[0].X, shape[0].Y, 0.0);
        }

        var remaining = offset;

        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var segmentLength = Math.Sqrt(dx * dx + dy * dy);
            var isLastSegment = i == shape.Count - 2;

            if (remaining <= segmentLength || isLastSegment)
            {
                // Clamp so an offset beyond the polyline's total length (or a negative
                // departPos) still resolves to an endpoint instead of extrapolating.
                var t = segmentLength > 0 ? Math.Clamp(remaining / segmentLength, 0.0, 1.0) : 0.0;
                var x = x1 + dx * t;
                var y = y1 + dy * t;
                if (latOffset != 0.0 && segmentLength > 0)
                {
                    // Left-normal unit vector of the segment direction (dx,dy): rotate +90 deg CCW ->
                    // (-dy, dx), normalized. Positive latOffset moves the point to the left of travel.
                    x += latOffset * (-dy / segmentLength);
                    y += latOffset * (dx / segmentLength);
                }

                var angleRad = Math.Atan2(dy, dx);
                var naviDeg = NormalizeDegrees(90.0 - angleRad * 180.0 / Math.PI);
                return (x, y, naviDeg);
            }

            remaining -= segmentLength;
        }

        var last = shape[^1];
        return (last.X, last.Y, 0.0);
    }

    // SUMOSHARP-API.md §6: the elevation (z) at longitudinal `offset` along the lane, interpolated with the
    // SAME segment walk and clamp as PositionAtOffset (2-D segment lengths, same `t`), so z is consistent
    // with the derived x/y. Purely output-side, like the whole class -- never feeds kinematic state. Used
    // only for the read surface's PosZ; a 2-D lane (ShapeZ == null) never calls this and reports 0.
    public static double ElevationAtOffset(
        IReadOnlyList<(double X, double Y)> shape,
        IReadOnlyList<double> shapeZ,
        double offset)
    {
        if (shape.Count == 0 || shapeZ.Count == 0)
        {
            return 0.0;
        }

        if (shape.Count == 1)
        {
            return shapeZ[0];
        }

        var remaining = offset;
        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var segmentLength = Math.Sqrt(dx * dx + dy * dy);
            var isLastSegment = i == shape.Count - 2;

            if (remaining <= segmentLength || isLastSegment)
            {
                var t = segmentLength > 0 ? Math.Clamp(remaining / segmentLength, 0.0, 1.0) : 0.0;
                var z1 = i < shapeZ.Count ? shapeZ[i] : 0.0;
                var z2 = i + 1 < shapeZ.Count ? shapeZ[i + 1] : 0.0;
                return z1 + (z2 - z1) * t;
            }

            remaining -= segmentLength;
        }

        return shapeZ[^1];
    }

    private static double NormalizeDegrees(double degrees)
    {
        var normalized = degrees % 360.0;
        return normalized < 0 ? normalized + 360.0 : normalized;
    }
}
