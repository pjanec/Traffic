using System.Text.RegularExpressions;

namespace CityLib;

// docs/LIVE-CITY-VISUALS-NOTES.md "Crosswalk zebra" row / docs/reference/live-city-viz/DESIGN-live-city-2d-
// viz.md §2 layer 9 (reference template.js `drawCrossings`): a SUMO internal crossing lane
// (net.xml edge id ":<node>_c<idx>", function="crossing"; its lane id appends "_<laneIndex>", typically
// "_0") gets perpendicular white "zebra" stripes stepped along its length, instead of a plain lane ribbon.
// Pure stripe-quad math, no Godot type anywhere (mirrors RoadMeshBuilder/ZoneGroundBuilder's own
// "CityLib stays engine-agnostic" split -- Main.cs turns the result into a MeshInstance3D). Each stripe is
// built by calling RoadMeshBuilder.Build on a tiny 2-point "shape" (the stripe's along-crossing centreline)
// with the stripe's across-crossing extent as the ribbon `width` -- so the triangulation + SUMO->Godot
// transform is exercised through the SAME already-tested code path, not duplicated.
public static class CrosswalkBuilder
{
    // The task's own regex (`^:.+_c\d+$`) matches a crossing EDGE id exactly (the reference renderer tests
    // `lane.edgeId`, docs/reference/live-city-viz/renderer/templates/template.js:998 `CROSSING_RE`). This
    // pattern additionally tolerates an optional trailing "_<laneIndex>" so it ALSO matches the crossing's
    // LANE id directly (e.g. ":d_0_0_c0_0") -- "works from the lane id alone" (no edge/id lookup needed) for
    // a caller that only has a lane id string (Sim.Ingest.Lane.Id, or the 2D LanePayload's Id).
    private static readonly Regex CrossingIdPattern = new(@"^:.+_c\d+(_\d+)?$", RegexOptions.Compiled);

    public static bool IsCrossingLaneId(string? laneOrEdgeId) =>
        !string.IsNullOrEmpty(laneOrEdgeId) && CrossingIdPattern.IsMatch(laneOrEdgeId);

    // Builds one crossing lane's zebra mesh: stripes spaced `stripeSpacing` metres centre-to-centre (first
    // stripe centred at stripeSpacing/2, matching the reference's own `d = stripeGap*0.5; d < len; d +=
    // stripeGap` walk), each `stripeThickness` metres long ALONG the crossing and `width * widthFraction`
    // metres long ACROSS it (a small inset from the lane's full width so a stripe doesn't overhang the
    // crossing's outline/kerb). Elevated `elevationOffsetSumoZ` (SUMO z, additive) above the crossing lane's
    // own RoadMeshBuilder ribbon so the stripes never z-fight with the surface underneath them.
    public static (RibbonMesh Mesh, int StripeCount) Build(
        IReadOnlyList<(double X, double Y)> shape,
        double width,
        double stripeSpacing = 1.0,
        double stripeThickness = 0.5,
        double widthFraction = 0.8,
        double elevationOffsetSumoZ = 0.02)
    {
        if (shape.Count < 2 || width <= 0.0 || stripeSpacing <= 0.0)
        {
            return (Empty, 0);
        }

        var cumulative = CumulativeLengths(shape);
        var total = cumulative[^1];
        if (total < 1e-6)
        {
            return (Empty, 0);
        }

        var halfAlong = stripeThickness / 2.0;
        var acrossWidth = Math.Max(width * widthFraction, 0.1);

        var parts = new List<RibbonMesh>();
        var stripeCount = 0;
        for (var d = stripeSpacing * 0.5; d < total; d += stripeSpacing)
        {
            var (point, tangent) = PointAndTangentAt(shape, cumulative, d);
            var p0 = (point.X - tangent.X * halfAlong, point.Y - tangent.Y * halfAlong);
            var p1 = (point.X + tangent.X * halfAlong, point.Y + tangent.Y * halfAlong);
            var stripeShape = new[] { p0, p1 };
            var stripeZ = new[] { elevationOffsetSumoZ, elevationOffsetSumoZ };
            parts.Add(RoadMeshBuilder.Build(stripeShape, stripeZ, acrossWidth));
            stripeCount++;
        }

        return (MergeRibbons(parts), stripeCount);
    }

    private static readonly RibbonMesh Empty =
        new(Array.Empty<float>(), Array.Empty<int>(), Array.Empty<float>(), 0.0);

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

    // Locates the point + unit travel-direction tangent at arc-length `distance` along `shape`.
    private static ((double X, double Y) Point, (double X, double Y) Tangent) PointAndTangentAt(
        IReadOnlyList<(double X, double Y)> shape, double[] cumulative, double distance)
    {
        var segCount = shape.Count - 1;
        for (var i = 0; i < segCount; i++)
        {
            var segStart = cumulative[i];
            var segEnd = cumulative[i + 1];
            if (distance > segEnd && i != segCount - 1)
            {
                continue;
            }

            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var segLen = segEnd - segStart;
            var t = segLen > 1e-9 ? Math.Clamp((distance - segStart) / segLen, 0.0, 1.0) : 0.0;
            var px = x1 + (x2 - x1) * t;
            var py = y1 + (y2 - y1) * t;
            var dx = x2 - x1;
            var dy = y2 - y1;
            var len = Math.Sqrt(dx * dx + dy * dy);
            var tx = len > 1e-9 ? dx / len : 1.0;
            var ty = len > 1e-9 ? dy / len : 0.0;
            return ((px, py), (tx, ty));
        }

        var (lx, ly) = shape[^1];
        return ((lx, ly), (1.0, 0.0));
    }

    // Concatenates several independently-built RibbonMesh parts into one (index-offset merge) -- shared by
    // CrosswalkBuilder (per-stripe ribbons) and LaneMarkingBuilder (per-dash ribbons) so both draw as a
    // single MeshInstance3D/Godot draw call rather than one per stripe/dash. Public: the Viewer project
    // (Main.cs, a separate assembly) also calls this directly to merge its own per-lane crosswalk/marking
    // ribbons into the ONE MeshInstance3D each overlay layer uses.
    public static RibbonMesh MergeRibbons(IReadOnlyList<RibbonMesh> parts)
    {
        var vertexCount = 0;
        var indexCount = 0;
        var area = 0.0;
        foreach (var part in parts)
        {
            vertexCount += part.Vertices.Length;
            indexCount += part.Indices.Length;
            area += part.Area;
        }

        var vertices = new float[vertexCount];
        var normals = new float[vertexCount];
        var indices = new int[indexCount];

        var offset = 0;
        var iOffset = 0;
        var vertBase = 0;
        foreach (var part in parts)
        {
            Array.Copy(part.Vertices, 0, vertices, offset, part.Vertices.Length);
            Array.Copy(part.Normals, 0, normals, offset, part.Normals.Length);
            for (var k = 0; k < part.Indices.Length; k++)
            {
                indices[iOffset + k] = part.Indices[k] + vertBase;
            }

            offset += part.Vertices.Length;
            iOffset += part.Indices.Length;
            vertBase += part.Vertices.Length / 3;
        }

        return new RibbonMesh(vertices, indices, normals, area);
    }
}
