using Sim.LiveCity;

namespace CityLib;

// docs/LIVE-CITY-VISUALS-NOTES.md "Buildings (data-driven)" row / docs/reference/live-city-viz/DESIGN-
// live-city-2d-viz.md §7 "Buildings: buildings[].polygon + type -> extruded massing" -- the PRIMARY
// building renderer (the "data over defaults" standing directive: a real footprint + real HeightM from
// buildings.json beats a synthetic BuildingPlacer box). Pure polygon -> mesh math, no Godot type anywhere
// (mirrors ZoneGroundBuilder's own "CityLib stays engine-agnostic" split -- Main.cs turns this into an
// ArrayMesh/MeshInstance3D per building).
public readonly struct ExtrudedBuildingMesh
{
    public ExtrudedBuildingMesh(
        float[] vertices, int[] indices, float[] normals, double footprintArea, int roofVertexCount, int wallQuadCount)
    {
        Vertices = vertices;
        Indices = indices;
        Normals = normals;
        FootprintArea = footprintArea;
        RoofVertexCount = roofVertexCount;
        WallQuadCount = wallQuadCount;
    }

    // xyz triples, already in GODOT space (CoordinateTransform.SumoToGodot applied). The first
    // `RoofVertexCount` vertices are the roof cap (in footprint order, all at Godot Y = HeightM); the
    // remaining `WallQuadCount * 4` vertices are the wall quads, four fresh (non-shared) vertices per
    // footprint edge so each wall face gets its own flat outward normal.
    public float[] Vertices { get; }

    // Triangle indices (into Vertices/3): the roof cap's ear-clip triangulation first, then two triangles
    // per wall quad.
    public int[] Indices { get; }

    // Per-vertex normals, Godot space, parallel to Vertices. Roof vertices: (0,1,0) (straight up). Wall
    // vertices: the edge's outward horizontal normal (same for all 4 corners of that edge's quad, giving
    // flat-shaded walls -- a building silhouette reads far better flat-shaded than smooth-shaded).
    public float[] Normals { get; }

    // The footprint's planar (SUMO x/y ground-plane) area in square metres -- shoelace formula, so it is
    // invariant to the footprint's authored winding order (CW or CCW both feed the same |area|).
    public double FootprintArea { get; }

    // Vertex count of the roof cap == the (deduplicated) footprint point count. 0 for a degenerate
    // (<3-point, zero-length-edge-collapsed, or non-positive height) footprint.
    public int RoofVertexCount { get; }

    // Number of wall quads emitted == the number of non-degenerate (non-zero-length) footprint edges. For
    // every building in the committed demo_city/box dataset this equals RoofVertexCount (every quad
    // footprint has 4 distinct corners), but a footprint with a repeated/coincident vertex would emit one
    // fewer wall than roof vertices -- callers that assert "wall count == edge count" should compare
    // against RoofVertexCount, not a hardcoded literal.
    public int WallQuadCount { get; }

    public static readonly ExtrudedBuildingMesh Empty =
        new(Array.Empty<float>(), Array.Empty<int>(), Array.Empty<float>(), 0.0, 0, 0);
}

// Extrudes one SceneBuilding footprint (SUMO x/y polygon) up to HeightM: a roof cap (ear-clip
// triangulated -- correct for both the convex quads the committed demo_city/box dataset actually ships
// AND any non-convex footprint a future dataset might add, per the task's explicit ear-clip requirement)
// plus one flat-shaded wall quad (2 triangles) per footprint edge, from the ground (SUMO z=0) up to
// HeightM.
public static class BuildingFromDataBuilder
{
    // Builds the extruded prism for one building. `footprint` is read as-authored (any winding); the
    // first step normalizes it to CCW in the SUMO (x,y) ground plane (viewed from above, +Y sumo-north
    // "up" in the 2-D sense) so the ear-clip convexity test and the per-edge outward-normal formula both
    // have a single, known orientation to work from -- every footprint in the committed dataset is already
    // CCW (verified against buildings.json), so this is a defensive no-op there, but keeps the builder
    // correct for hand-authored or differently-wound future data.
    public static ExtrudedBuildingMesh Build(IReadOnlyList<(double X, double Y)> footprint, double heightM)
    {
        var pts = DedupeAndNormalizeCcw(footprint);
        var n = pts.Count;
        if (n < 3 || heightM <= 0.0)
        {
            return ExtrudedBuildingMesh.Empty;
        }

        var footprintArea = PlanarArea(pts);

        var roofTris = EarClipTriangulate(pts);

        var vertices = new List<float>((n + n * 4) * 3);
        var normals = new List<float>((n + n * 4) * 3);
        var indices = new List<int>((roofTris.Count + n) * 3);

        // ---- Roof cap: n vertices at Godot Y = heightM, normal straight up. ----
        for (var i = 0; i < n; i++)
        {
            var (gx, gy, gz) = CoordinateTransform.SumoToGodot(pts[i].X, pts[i].Y, heightM);
            vertices.Add(gx);
            vertices.Add(gy);
            vertices.Add(gz);
            normals.Add(0f);
            normals.Add(1f);
            normals.Add(0f);
        }

        foreach (var (a, b, c) in roofTris)
        {
            indices.Add(a);
            indices.Add(b);
            indices.Add(c);
        }

        // ---- Walls: one quad (4 fresh vertices, 2 triangles) per non-degenerate footprint edge. ----
        var wallQuadCount = 0;
        for (var i = 0; i < n; i++)
        {
            var p0 = pts[i];
            var p1 = pts[(i + 1) % n];
            var dx = p1.X - p0.X;
            var dy = p1.Y - p0.Y;
            var len = Math.Sqrt(dx * dx + dy * dy);
            if (len < 1e-9)
            {
                continue; // degenerate (repeated) vertex pair -- no edge to wall.
            }

            // Outward unit normal for a CCW polygon: rotate the edge direction -90 degrees, i.e.
            // (dx,dy) -> (dy,-dx) (normalized). Sanity check on a CCW unit square (0,0)->(1,0)->(1,1)->
            // (0,1): edge0 direction (1,0) has interior above it (+y); the outward normal (0,-1) points
            // away from the interior, matching (dy,-dx)/len = (0,-1)/1.
            var nx = dy / len;
            var ny = -dx / len;

            var b0 = CoordinateTransform.SumoToGodot(p0.X, p0.Y, 0.0);
            var b1 = CoordinateTransform.SumoToGodot(p1.X, p1.Y, 0.0);
            var t1 = CoordinateTransform.SumoToGodot(p1.X, p1.Y, heightM);
            var t0 = CoordinateTransform.SumoToGodot(p0.X, p0.Y, heightM);

            // SUMO (nx,ny,0) -> Godot (nx, 0, -ny), the same position mapping CoordinateTransform.
            // SumoToGodot applies to a direction vector (drop the z term, negate y).
            var wallNormalX = (float)nx;
            var wallNormalZ = (float)-ny;

            var baseIdx = vertices.Count / 3;
            AddVertex(vertices, normals, b0, wallNormalX, wallNormalZ);
            AddVertex(vertices, normals, b1, wallNormalX, wallNormalZ);
            AddVertex(vertices, normals, t1, wallNormalX, wallNormalZ);
            AddVertex(vertices, normals, t0, wallNormalX, wallNormalZ);

            // Two triangles (b0,b1,t1) and (b0,t1,t0) -- CCW when viewed FROM the outward side (the wall's
            // front face), matching the outward normal computed above.
            indices.Add(baseIdx + 0);
            indices.Add(baseIdx + 1);
            indices.Add(baseIdx + 2);
            indices.Add(baseIdx + 0);
            indices.Add(baseIdx + 2);
            indices.Add(baseIdx + 3);

            wallQuadCount++;
        }

        return new ExtrudedBuildingMesh(
            vertices.ToArray(), indices.ToArray(), normals.ToArray(), footprintArea, n, wallQuadCount);
    }

    // Convenience overload over the render-neutral scene record (Sim.LiveCity.SceneBuilding).
    public static ExtrudedBuildingMesh Build(SceneBuilding building)
        => Build(building.Footprint, building.HeightM);

    private static void AddVertex(
        List<float> vertices, List<float> normals, (float X, float Y, float Z) pos, float normalX, float normalZ)
    {
        vertices.Add(pos.X);
        vertices.Add(pos.Y);
        vertices.Add(pos.Z);
        normals.Add(normalX);
        normals.Add(0f);
        normals.Add(normalZ);
    }

    // Drops consecutive coincident points (defensive against a footprint with a repeated vertex), then
    // reverses the ring if it is authored clockwise so every downstream computation (ear-clip, outward
    // wall normals) can assume CCW. Uses the SAME shoelace sign convention PlanarArea below does.
    private static List<(double X, double Y)> DedupeAndNormalizeCcw(IReadOnlyList<(double X, double Y)> footprint)
    {
        var deduped = new List<(double X, double Y)>(footprint.Count);
        foreach (var p in footprint)
        {
            if (deduped.Count == 0)
            {
                deduped.Add(p);
                continue;
            }

            var last = deduped[^1];
            var dx = p.X - last.X;
            var dy = p.Y - last.Y;
            if (Math.Sqrt(dx * dx + dy * dy) >= 1e-9)
            {
                deduped.Add(p);
            }
        }

        // Drop a trailing point that coincides with the first (a closed-ring authoring convention).
        if (deduped.Count > 1)
        {
            var first = deduped[0];
            var last = deduped[^1];
            var dx = last.X - first.X;
            var dy = last.Y - first.Y;
            if (Math.Sqrt(dx * dx + dy * dy) < 1e-9)
            {
                deduped.RemoveAt(deduped.Count - 1);
            }
        }

        if (deduped.Count < 3)
        {
            return deduped;
        }

        var signedArea2 = 0.0;
        for (var i = 0; i < deduped.Count; i++)
        {
            var (x1, y1) = deduped[i];
            var (x2, y2) = deduped[(i + 1) % deduped.Count];
            signedArea2 += (x1 * y2) - (x2 * y1);
        }

        if (signedArea2 < 0.0)
        {
            deduped.Reverse();
        }

        return deduped;
    }

    // Shoelace formula, SUMO (x,y) plane -- same technique ZoneGroundBuilder.PlanarArea uses. Absolute
    // value: callers only need magnitude.
    private static double PlanarArea(IReadOnlyList<(double X, double Y)> polygon)
    {
        var n = polygon.Count;
        var sum = 0.0;
        for (var i = 0; i < n; i++)
        {
            var (x1, y1) = polygon[i];
            var (x2, y2) = polygon[(i + 1) % n];
            sum += (x1 * y2) - (x2 * y1);
        }

        return Math.Abs(sum) * 0.5;
    }

    // Standard ear-clipping triangulation over a CCW-wound simple polygon (no self-intersections) --
    // handles BOTH the convex quads the committed dataset ships (every candidate ear is found on the
    // first pass) and a non-convex footprint (a future dataset might ship an L-shaped or notched
    // building), per the task's explicit "if footprints are non-convex, ear-clip is needed" call-out.
    // Returns triangles as index triples into `polygonCcw`.
    private static List<(int A, int B, int C)> EarClipTriangulate(IReadOnlyList<(double X, double Y)> polygonCcw)
    {
        var result = new List<(int, int, int)>();
        var n = polygonCcw.Count;
        if (n < 3)
        {
            return result;
        }

        if (n == 3)
        {
            result.Add((0, 1, 2));
            return result;
        }

        var remaining = new List<int>(n);
        for (var i = 0; i < n; i++)
        {
            remaining.Add(i);
        }

        // Bounded retry loop: each successful clip removes one vertex, so at most n-3 clips are needed to
        // get down to a final triangle. The outer guard (n*n) is a defensive bound only reached by
        // degenerate/self-intersecting input, in which case the loop simply stops early and returns
        // whatever valid ears it already found (never throws -- matches ZoneGroundBuilder/BuildingPlacer's
        // "never throw on odd geometry" convention).
        var guard = 0;
        while (remaining.Count > 3 && guard++ < n * n)
        {
            var earFound = false;
            for (var ii = 0; ii < remaining.Count; ii++)
            {
                var count = remaining.Count;
                var iPrev = remaining[(ii - 1 + count) % count];
                var iCurr = remaining[ii];
                var iNext = remaining[(ii + 1) % count];

                var a = polygonCcw[iPrev];
                var b = polygonCcw[iCurr];
                var c = polygonCcw[iNext];

                if (!IsConvexTurn(a, b, c))
                {
                    continue; // reflex vertex -- cannot be an ear tip.
                }

                var anyOtherPointInside = false;
                for (var jj = 0; jj < remaining.Count; jj++)
                {
                    var idx = remaining[jj];
                    if (idx == iPrev || idx == iCurr || idx == iNext)
                    {
                        continue;
                    }

                    if (PointInTriangle(polygonCcw[idx], a, b, c))
                    {
                        anyOtherPointInside = true;
                        break;
                    }
                }

                if (anyOtherPointInside)
                {
                    continue;
                }

                result.Add((iPrev, iCurr, iNext));
                remaining.RemoveAt(ii);
                earFound = true;
                break;
            }

            if (!earFound)
            {
                break; // degenerate/self-intersecting polygon -- stop rather than spin.
            }
        }

        if (remaining.Count == 3)
        {
            result.Add((remaining[0], remaining[1], remaining[2]));
        }

        return result;
    }

    // Positive (or straight) cross product at b == a left turn / convex vertex for a CCW polygon.
    private static bool IsConvexTurn((double X, double Y) a, (double X, double Y) b, (double X, double Y) c)
    {
        var cross = ((b.X - a.X) * (c.Y - b.Y)) - ((b.Y - a.Y) * (c.X - b.X));
        return cross >= -1e-9;
    }

    private static bool PointInTriangle(
        (double X, double Y) p, (double X, double Y) a, (double X, double Y) b, (double X, double Y) c)
    {
        var d1 = Cross(a, b, p);
        var d2 = Cross(b, c, p);
        var d3 = Cross(c, a, p);

        var hasNeg = d1 < 0 || d2 < 0 || d3 < 0;
        var hasPos = d1 > 0 || d2 > 0 || d3 > 0;
        return !(hasNeg && hasPos);
    }

    private static double Cross((double X, double Y) a, (double X, double Y) b, (double X, double Y) p)
        => ((b.X - a.X) * (p.Y - a.Y)) - ((b.Y - a.Y) * (p.X - a.X));
}
