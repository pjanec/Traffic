using Sim.Core.Orca;

namespace Sim.Pedestrians.Navigation.Bake;

// Small deterministic 2D polygon helpers shared by the SUMO-geometry-bake navigation provider
// (docs/PEDESTRIAN-DESIGN.md §4, POC-1a). A "polygon" here is always an ORDERED vertex list
// treated as an IMPLICITLY CLOSED ring -- the edge from Vertices[^1] back to Vertices[0] is part
// of the boundary even though it is not stored twice. This matches the convention
// OrcaCrowd.AddObstacle already uses for its wall loops, so a baked polygon's vertex list can be
// hand^ed straight to AddObstacle with no re-closing.
internal static class PolygonGeometry
{
    // Used only for degenerate-length checks (zero-length segments), not for equality tolerance.
    public const double DegenerateLengthSq = 1e-12;

    // Point-in-polygon via the standard even-odd ray-casting test (Sedgewick). Runs on the
    // implicitly-closed ring described above.
    public static bool Contains(IReadOnlyList<Vec2> vertices, Vec2 p)
    {
        var inside = false;
        var n = vertices.Count;
        for (int i = 0, j = n - 1; i < n; j = i++)
        {
            var vi = vertices[i];
            var vj = vertices[j];
            var crosses = (vi.Y > p.Y) != (vj.Y > p.Y);
            if (crosses && p.X < ((vj.X - vi.X) * (p.Y - vi.Y) / (vj.Y - vi.Y)) + vi.X)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    // Nearest point to `p` lying ON the polygon's boundary (its edge set), plus the squared
    // distance to it. Used to clamp an off-mesh point onto walkable space.
    public static Vec2 NearestPointOnBoundary(IReadOnlyList<Vec2> vertices, Vec2 p, out double distSq)
    {
        var n = vertices.Count;
        var best = vertices[0];
        var bestDistSq = double.MaxValue;

        for (var i = 0; i < n; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % n];
            var candidate = NearestPointOnSegment(a, b, p);
            var dSq = (candidate - p).AbsSq;
            if (dSq < bestDistSq)
            {
                bestDistSq = dSq;
                best = candidate;
            }
        }

        distSq = bestDistSq;
        return best;
    }

    public static Vec2 NearestPointOnSegment(Vec2 a, Vec2 b, Vec2 p)
    {
        var ab = b - a;
        var abLenSq = ab.AbsSq;
        if (abLenSq <= DegenerateLengthSq)
        {
            return a;
        }

        var t = Vec2.Dot(p - a, ab) / abLenSq;
        t = Math.Clamp(t, 0.0, 1.0);
        return a + (t * ab);
    }

    // Vertex-average "centroid" (not the true area centroid) -- adequate as an A* distance
    // heuristic (PEDESTRIAN-POC-PLAN.md POC-1 task: "Euclidean centroid heuristic") without the
    // extra complexity of a signed-area centroid computation.
    public static Vec2 VertexAverage(IReadOnlyList<Vec2> vertices)
    {
        double sx = 0.0, sy = 0.0;
        foreach (var v in vertices)
        {
            sx += v.X;
            sy += v.Y;
        }

        return new Vec2(sx / vertices.Count, sy / vertices.Count);
    }

    public static bool NearlyEqual(Vec2 a, Vec2 b, double epsilon) => (a - b).AbsSq <= epsilon * epsilon;

    // P8-1b (docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): `p` is inside the polygon AND at least
    // `margin` away from its boundary -- STRICTLY inside. A boundary or corner touch (distance ~0) is NOT
    // strictly inside, which is exactly what keeps a shared-corner touch (the POC-0 3-polygon-corner shape)
    // from reading as an area overlap.
    public static bool ContainsStrict(IReadOnlyList<Vec2> vertices, Vec2 p, double margin)
    {
        if (!Contains(vertices, p))
        {
            return false;
        }

        NearestPointOnBoundary(vertices, p, out var distSq);
        return distSq > margin * margin;
    }

    // Proper (transversal) segment intersection: the two open segments cross at an interior point of BOTH.
    // A shared endpoint or a collinear overlap is NOT a proper intersection (strict sign test), so two
    // polygons that merely meet at a corner do not register as crossing. Returns the crossing point in
    // `point` when true.
    public static bool SegmentsProperlyIntersect(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1, out Vec2 point)
    {
        point = default;
        var r = a1 - a0;
        var s = b1 - b0;
        var denom = Vec2.Det(r, s);
        if (denom == 0.0)
        {
            return false; // parallel or collinear -- not a proper crossing
        }

        var qp = b0 - a0;
        var t = Vec2.Det(qp, s) / denom;
        var u = Vec2.Det(qp, r) / denom;
        if (t <= 0.0 || t >= 1.0 || u <= 0.0 || u >= 1.0)
        {
            return false; // intersection at/beyond an endpoint -- not interior to both
        }

        point = a0 + (t * r);
        return true;
    }

    // Do two polygons genuinely OVERLAP in 2D area (not merely touch at a corner)? Evidence collected:
    // any vertex of one strictly inside the other, and any proper edge crossing. When they overlap, a
    // representative interior portal point (the average of that evidence -- all points inside the walkable
    // union) is returned in `portal`. `margin` is the strict-inside slack (see ContainsStrict). This is the
    // invariant-safe adjacency signal for real buffered geometry: abutting sidewalk/crossing strips overlap
    // the junction walkingArea by a sliver, but a sidewalk and a crossing (which only meet the area, not
    // each other) do not overlap -- so no illegitimate shortcut portal is ever produced.
    public static bool TryFindOverlapPortal(
        IReadOnlyList<Vec2> a, IReadOnlyList<Vec2> b, double margin, out Vec2 portal)
    {
        portal = default;
        double sx = 0.0, sy = 0.0;
        var count = 0;

        foreach (var v in a)
        {
            if (ContainsStrict(b, v, margin))
            {
                sx += v.X;
                sy += v.Y;
                count++;
            }
        }

        foreach (var v in b)
        {
            if (ContainsStrict(a, v, margin))
            {
                sx += v.X;
                sy += v.Y;
                count++;
            }
        }

        for (var i = 0; i < a.Count; i++)
        {
            var a0 = a[i];
            var a1 = a[(i + 1) % a.Count];
            for (var j = 0; j < b.Count; j++)
            {
                var b0 = b[j];
                var b1 = b[(j + 1) % b.Count];
                if (SegmentsProperlyIntersect(a0, a1, b0, b1, out var x))
                {
                    sx += x.X;
                    sy += x.Y;
                    count++;
                }
            }
        }

        if (count == 0)
        {
            return false;
        }

        portal = new Vec2(sx / count, sy / count);
        return true;
    }

    // Signed area via the shoelace formula, over the implicitly-closed ring. Used to detect and
    // drop DEGENERATE polygons: SUMO emits a zero-area "walkingarea" at a network-boundary dead
    // end (all its shape points collinear, e.g. POC-0's ":n_w0_0" / ":e_w0_0" / ":s_w0_0" /
    // ":w_w0_0") to keep its lane-adjacency bookkeeping uniform, even though there is no real
    // walkable AREA there. Left in the baked set, such a polygon has no interior (Contains() is
    // false almost everywhere on it) yet still gets adjacency portals to its neighbours -- which
    // would let A* route a path "through" it as an illegitimate zero-width teleport. Baking must
    // filter these out.
    public static double SignedArea(IReadOnlyList<Vec2> vertices)
    {
        double area = 0.0;
        var n = vertices.Count;
        for (var i = 0; i < n; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % n];
            area += (a.X * b.Y) - (b.X * a.Y);
        }

        return area / 2.0;
    }
}
