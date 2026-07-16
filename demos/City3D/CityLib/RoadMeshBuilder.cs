using Sim.Ingest;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Roads" -- pure ribbon-mesh math, no Godot
// type anywhere (CityLib stays engine-agnostic; the Viewer project turns this into an ArrayMesh). Static
// geometry only, built once at load (design: "two clocks: static geometry ... built once at load").
public readonly struct RibbonMesh
{
    public RibbonMesh(float[] vertices, int[] indices, float[] normals, double area)
    {
        Vertices = vertices;
        Indices = indices;
        Normals = normals;
        Area = area;
    }

    // xyz triples, already in GODOT space (CoordinateTransform.SumoToGodot applied).
    public float[] Vertices { get; }
    public int[] Indices { get; }
    public float[] Normals { get; }
    // Sum of the per-quad planar (ground-plane) areas, in square metres.
    public double Area { get; }
}

public static class RoadMeshBuilder
{
    // Turns ONE lane's centerline (+ width, + optional per-vertex elevation) into a flat ribbon mesh.
    // Algorithm (design "Roads"):
    //   1. per segment, the 2-D unit normal (perpendicular to the segment direction);
    //   2. offset centerline vertices +/-width/2 along the (miter-averaged at interior vertices) normal to
    //      get left/right edges;
    //   3. emit 2 triangles per segment.
    // Every emitted vertex is transformed through CoordinateTransform.SumoToGodot (Y = elevation from
    // shapeZ, or 0 when null).
    public static RibbonMesh Build(IReadOnlyList<(double X, double Y)> shape, IReadOnlyList<double>? shapeZ, double width)
    {
        var n = shape.Count;
        if (n == 0)
        {
            return new RibbonMesh(Array.Empty<float>(), Array.Empty<int>(), Array.Empty<float>(), 0.0);
        }

        if (n == 1)
        {
            // Degenerate single-point lane: nothing to ribbon (no segment to derive a normal from).
            return new RibbonMesh(Array.Empty<float>(), Array.Empty<int>(), Array.Empty<float>(), 0.0);
        }

        var halfWidth = width / 2.0;

        // Per-segment unit normals (perpendicular to travel direction, i.e. rotate the segment direction
        // +90deg CCW in SUMO's (x,y) plane: (dx,dy) -> (-dy,dx), normalized). n-1 segments.
        var segCount = n - 1;
        var segNormals = new (double X, double Y)[segCount];
        for (var i = 0; i < segCount; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var len = Math.Sqrt(dx * dx + dy * dy);
            segNormals[i] = len > 1e-9 ? (-dy / len, dx / len) : (0.0, 0.0);
        }

        // Per-vertex normal: the miter (normalized average) of the adjacent segment normals at interior
        // vertices; the lone adjacent segment's normal at the two endpoints. A length cap on the miter
        // (design "Roads" item 3) avoids spikes at very sharp bends -- clamp the averaged (pre-normalize)
        // vector's magnitude-derived scale so the offset never exceeds a small multiple of halfWidth.
        var vertNormals = new (double X, double Y)[n];
        for (var i = 0; i < n; i++)
        {
            (double X, double Y) nrm;
            if (i == 0)
            {
                nrm = segNormals[0];
            }
            else if (i == n - 1)
            {
                nrm = segNormals[segCount - 1];
            }
            else
            {
                var a = segNormals[i - 1];
                var b = segNormals[i];
                var sx = a.X + b.X;
                var sy = a.Y + b.Y;
                var slen = Math.Sqrt(sx * sx + sy * sy);
                if (slen < 1e-6)
                {
                    // Near-180deg reversal: fall back to the incoming segment's normal rather than a
                    // near-zero (or undefined) average.
                    nrm = a;
                }
                else
                {
                    // Miter scale = 1/cos(halfAngle) so the offset edge still lands `halfWidth` away from
                    // the centerline measured perpendicular to each segment; cap it (miter-limit) so a very
                    // sharp bend doesn't spike the offset vertex far off the road.
                    var cosHalfAngle = (a.X * sx + a.Y * sy) / slen; // = dot(a, normalized sum)
                    var miterScale = cosHalfAngle > 1e-3 ? 1.0 / cosHalfAngle : 4.0;
                    miterScale = Math.Min(miterScale, 4.0);
                    nrm = (sx / slen * miterScale, sy / slen * miterScale);
                }
            }

            vertNormals[i] = nrm;
        }

        var left = new (double X, double Y)[n];
        var right = new (double X, double Y)[n];
        for (var i = 0; i < n; i++)
        {
            var (cx, cy) = shape[i];
            var (nx, ny) = vertNormals[i];
            left[i] = (cx + nx * halfWidth, cy + ny * halfWidth);
            right[i] = (cx - nx * halfWidth, cy - ny * halfWidth);
        }

        // 2 vertices per centerline point (left, right), transformed to Godot space. Up-normal (0,1,0) for
        // a flat ribbon (a road surface faces straight up).
        var vertices = new float[n * 2 * 3];
        var normals = new float[n * 2 * 3];
        for (var i = 0; i < n; i++)
        {
            var z0 = shapeZ is not null && i < shapeZ.Count ? shapeZ[i] : 0.0;

            var (lgx, lgy, lgz) = CoordinateTransform.SumoToGodot(left[i].X, left[i].Y, z0);
            var (rgx, rgy, rgz) = CoordinateTransform.SumoToGodot(right[i].X, right[i].Y, z0);

            var leftBase = i * 2 * 3;
            vertices[leftBase + 0] = lgx;
            vertices[leftBase + 1] = lgy;
            vertices[leftBase + 2] = lgz;
            normals[leftBase + 0] = 0f;
            normals[leftBase + 1] = 1f;
            normals[leftBase + 2] = 0f;

            var rightBase = leftBase + 3;
            vertices[rightBase + 0] = rgx;
            vertices[rightBase + 1] = rgy;
            vertices[rightBase + 2] = rgz;
            normals[rightBase + 0] = 0f;
            normals[rightBase + 1] = 1f;
            normals[rightBase + 2] = 0f;
        }

        // Two triangles per segment: left[i], right[i], right[i+1] / left[i], right[i+1], left[i+1]
        // (consistent CCW winding when viewed from above, +Y up).
        var indices = new int[segCount * 6];
        var area = 0.0;
        for (var i = 0; i < segCount; i++)
        {
            var li = i * 2;      // left[i] vertex index
            var ri = li + 1;     // right[i]
            var li1 = li + 2;    // left[i+1]
            var ri1 = li + 3;    // right[i+1]

            var idxBase = i * 6;
            indices[idxBase + 0] = li;
            indices[idxBase + 1] = ri;
            indices[idxBase + 2] = ri1;
            indices[idxBase + 3] = li;
            indices[idxBase + 4] = ri1;
            indices[idxBase + 5] = li1;

            // Planar (ground-plane, i.e. SUMO x/y) quad area via the shoelace formula over the 4 corners
            // left[i], right[i], right[i+1], left[i+1] -- independent of elevation, matching the design's
            // sanity check "total ribbon area ~ Sum(lane length * lane width)".
            area += QuadArea(left[i], right[i], right[i + 1], left[i + 1]);
        }

        return new RibbonMesh(vertices, indices, normals, area);
    }

    // Iterates every lane in `network` (LanesByHandle, D2 dense handle order) and builds its ribbon mesh.
    // `includeInternal` (default true, design "Junctions are just the internal (':'-prefixed) lanes"):
    // when false, skips lanes whose Id starts with ':'.
    public static IEnumerable<(int Handle, RibbonMesh Mesh)> BuildAll(NetworkModel network, bool includeInternal = true)
    {
        foreach (var lane in network.LanesByHandle)
        {
            if (!includeInternal && lane.Id.StartsWith(':'))
            {
                continue;
            }

            yield return (lane.Handle, Build(lane.Shape, lane.ShapeZ, lane.Width));
        }
    }

    // Shoelace-formula area of a (possibly non-planar-in-3D but here purely 2-D) quad given in order
    // around its boundary.
    private static double QuadArea(
        (double X, double Y) a, (double X, double Y) b, (double X, double Y) c, (double X, double Y) d)
    {
        var sum = (a.X * b.Y - b.X * a.Y)
                 + (b.X * c.Y - c.X * b.Y)
                 + (c.X * d.Y - d.X * c.Y)
                 + (d.X * a.Y - a.X * d.Y);
        return Math.Abs(sum) / 2.0;
    }
}
