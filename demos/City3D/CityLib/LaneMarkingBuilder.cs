namespace CityLib;

// docs/LIVE-CITY-VISUALS-NOTES.md "Lane markings" row / docs/reference/live-city-viz/DESIGN-live-city-2d-
// viz.md §2 layer 3 (reference template.js precomputeNetwork()/offsetPolyline + drawLaneMarkings): a dashed
// white line laid on the SEAM between two same-edge adjacent lanes -- a lane's LEFT edge (offset its
// centerline by `+width/2` along the left-pointing normal), drawn for every lane that HAS a left neighbour
// (same edge, `Index+1` present). This is lane-DIVIDER striping ("driving lane separation"), NOT a per-lane
// centreline. Pure dash-quad math, no Godot type (mirrors CrosswalkBuilder's own split); reuses
// RoadMeshBuilder.Build per dash (each dash IS a tiny ribbon) and CrosswalkBuilder.MergeRibbons to
// concatenate the per-dash ribbons into one mesh.
public static class LaneMarkingBuilder
{
    private static readonly RibbonMesh Empty =
        new(Array.Empty<float>(), Array.Empty<int>(), Array.Empty<float>(), 0.0);

    // `laneWidth` is the LANE's own width (not the marking's) -- the seam sits `laneWidth/2` to the left of
    // the lane's centerline, exactly on the shared edge with its left neighbour. `dashLength`/`gapLength`
    // are the on/off run lengths (metres) of the dashed pattern; `markWidth` is the painted stripe's own
    // (thin) width. Elevated `elevationOffsetSumoZ` (SUMO z, additive) above the lane's own RoadMeshBuilder
    // ribbon to avoid z-fighting.
    public static (RibbonMesh Mesh, int DashCount) Build(
        IReadOnlyList<(double X, double Y)> shape,
        double laneWidth,
        double dashLength = 1.0,
        double gapLength = 1.0,
        double markWidth = 0.15,
        double elevationOffsetSumoZ = 0.02)
    {
        if (shape.Count < 2 || laneWidth <= 0.0 || dashLength <= 0.0 || gapLength < 0.0)
        {
            return (Empty, 0);
        }

        var offset = OffsetLeft(shape, laneWidth / 2.0);
        var cumulative = CumulativeLengths(offset);
        var total = cumulative[^1];
        if (total < 1e-6)
        {
            return (Empty, 0);
        }

        var parts = new List<RibbonMesh>();
        var dashCount = 0;
        var period = dashLength + gapLength;
        for (var d = 0.0; d < total; d += period)
        {
            var dashEnd = Math.Min(d + dashLength, total);
            var sub = SubPolyline(offset, cumulative, d, dashEnd);
            if (sub.Count < 2)
            {
                continue;
            }

            var subZ = new double[sub.Count];
            for (var i = 0; i < subZ.Length; i++)
            {
                subZ[i] = elevationOffsetSumoZ;
            }

            parts.Add(RoadMeshBuilder.Build(sub, subZ, markWidth));
            dashCount++;
        }

        return (CrosswalkBuilder.MergeRibbons(parts), dashCount);
    }

    // Offsets `shape` +dist along the LEFT-pointing normal -- the SAME per-segment normal convention
    // RoadMeshBuilder itself uses for a lane's own left edge (rotate travel direction +90deg CCW in SUMO's
    // x/y plane: (dx,dy) -> (-dy,dx), normalized; positive `dist` = left, matching the reference's own
    // offsetPolyline() "positive = LEFT" convention). Every ORIGINAL vertex maps 1:1 to one offset vertex,
    // using the (unweighted) average of its adjacent segments' normals at interior vertices -- not a full
    // miter -- so consecutive dash ribbons still share endpoints across a shape vertex instead of leaving a
    // visible gap, while staying simple (a lane-divider's thin width does not need RoadMeshBuilder's
    // miter-limit precision).
    private static (double X, double Y)[] OffsetLeft(IReadOnlyList<(double X, double Y)> shape, double dist)
    {
        var n = shape.Count;
        var segNormals = new (double X, double Y)[n - 1];
        for (var i = 0; i < n - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var len = Math.Sqrt(dx * dx + dy * dy);
            segNormals[i] = len > 1e-9 ? (-dy / len, dx / len) : (0.0, 0.0);
        }

        var result = new (double X, double Y)[n];
        for (var i = 0; i < n; i++)
        {
            (double X, double Y) nrm;
            if (i == 0)
            {
                nrm = segNormals[0];
            }
            else if (i == n - 1)
            {
                nrm = segNormals[n - 2];
            }
            else
            {
                var a = segNormals[i - 1];
                var b = segNormals[i];
                var sx = a.X + b.X;
                var sy = a.Y + b.Y;
                var slen = Math.Sqrt(sx * sx + sy * sy);
                nrm = slen > 1e-6 ? (sx / slen, sy / slen) : a;
            }

            var (cx, cy) = shape[i];
            result[i] = (cx + nrm.X * dist, cy + nrm.Y * dist);
        }

        return result;
    }

    private static double[] CumulativeLengths(IReadOnlyList<(double X, double Y)> shape)
    {
        var cum = new double[shape.Count];
        for (var i = 1; i < shape.Count; i++)
        {
            var (x1, y1) = shape[i - 1];
            var (x2, y2) = shape[i];
            var dx = x2 - x1;
            var dy = y2 - y1;
            cum[i] = cum[i - 1] + Math.Sqrt(dx * dx + dy * dy);
        }

        return cum;
    }

    // Extracts the sub-polyline of `shape` (arc-length parametrized by `cumulative`) between [from, to],
    // interpolating the two endpoints and keeping any original vertices strictly between them -- so a dash
    // that spans a shape bend still follows the bend rather than cutting a straight chord across it.
    private static List<(double X, double Y)> SubPolyline(
        IReadOnlyList<(double X, double Y)> shape, double[] cumulative, double from, double to)
    {
        var result = new List<(double X, double Y)>();
        var n = shape.Count;
        for (var i = 0; i < n - 1; i++)
        {
            var segStart = cumulative[i];
            var segEnd = cumulative[i + 1];
            if (segEnd < from)
            {
                continue;
            }

            if (segStart > to)
            {
                break;
            }

            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var segLen = segEnd - segStart;

            (double X, double Y) startPt;
            if (segStart >= from)
            {
                startPt = (x1, y1);
            }
            else
            {
                var t = segLen > 1e-9 ? (from - segStart) / segLen : 0.0;
                startPt = (x1 + (x2 - x1) * t, y1 + (y2 - y1) * t);
            }

            if (result.Count == 0 || SqDist(result[^1], startPt) > 1e-12)
            {
                result.Add(startPt);
            }

            if (segEnd <= to)
            {
                result.Add((x2, y2));
            }
            else
            {
                var t = segLen > 1e-9 ? (to - segStart) / segLen : 0.0;
                result.Add((x1 + (x2 - x1) * t, y1 + (y2 - y1) * t));
                break;
            }
        }

        return result;
    }

    private static double SqDist((double X, double Y) a, (double X, double Y) b)
    {
        var dx = a.X - b.X;
        var dy = a.Y - b.Y;
        return dx * dx + dy * dy;
    }
}
