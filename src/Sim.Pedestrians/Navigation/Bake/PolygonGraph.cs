using Sim.Core.Orca;

namespace Sim.Pedestrians.Navigation.Bake;

// A portal-adjacency edge from a polygon to one of its neighbours, carrying the shared-boundary
// portal point SumoNavMesh.FindPath threads the path through.
internal readonly record struct PolygonPortal(int Neighbor, Vec2 Point);

// Builds and holds the polygon-adjacency graph over a baked polygon set (docs/PEDESTRIAN-DESIGN.md
// §4, POC-1a). Two polygons are adjacent when their boundaries share a segment (an edge with the
// same two endpoints, in either order, within AdjacencyEpsilon) -- that segment's midpoint is the
// portal. Where more than one boundary segment is shared between the same pair (SUMO's corner
// geometry commonly abuts along a short multi-segment "staircase" chain, not a single edge), the
// portal point is the average of every shared segment's midpoint, giving one representative
// doorway point roughly centred on the shared chain.
//
// P2-1 (docs/PEDESTRIAN-TASKS.md): a SECOND pass restores vertex-proximity adjacency for polygon
// pairs that touch at a corner point without sharing a whole edge (e.g. two sidewalk polygons that
// meet a third lane's corner at a junction where widths differ, or a bent sidewalk's single mitred
// polygon meeting a neighbour only at its end-cap vertex). An EARLIER version of this connected ANY
// pair sharing just one endpoint, unconditionally, and that caused a real routing bug in POC-0: a
// crossing polygon, its walkingarea, and the far sidewalk quad all met at one shared CORNER VERTEX
// (three polygons, one point), and the unconditional fallback connected the crossing directly to
// the sidewalk there -- skipping the walkingarea whose actual AREA is the only real path between
// them. A* then routed a straight line across that corner that briefly left the walkable union.
//
// THE FIX applied here: cluster vertices across ALL polygons by mutual proximity (union-find, same
// AdjacencyEpsilon as the edge test), then, per cluster, only add a vertex portal when the cluster
// touches EXACTLY TWO distinct polygons. A cluster touched by 3+ polygons (the POC-0 bug's shape,
// "three polygons, one point") is skipped entirely -- a single shared vertex is ambiguous evidence
// of a doorway once more than two polygons meet there, exactly the case the bug came from, so it is
// never connected by this pass (any REAL connection through such a corner must go through the
// area polygon, via the shared-edge pass above). A cluster touched by exactly two polygons has no
// such ambiguity: it is a genuine two-party corner touch, so it gets a portal (the point is the
// cluster's vertex average). Pairs already connected by the shared-edge pass are left alone (the
// edge portal wins; a vertex portal is only added where no edge portal already exists for that
// pair), so this pass strictly ADDS missing corner-only connections, it never duplicates or
// overrides an edge-based one.
internal sealed class PolygonGraph
{
    // A "small epsilon" for SUMO net coordinates (meters): generous enough to tolerate minor
    // floating-point drift in shared boundary geometry, tight enough that unrelated polygons
    // several metres apart never falsely connect.
    private const double AdjacencyEpsilon = 1e-3;

    // P8-1b (docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): strict-inside slack for the area-overlap
    // pass. A real buffered sidewalk/crossing overlaps the junction walkingArea by ~0.05-0.2 m, far more than
    // this; a shared-corner touch penetrates ~0, so it never reads as an overlap. 1 mm matches AdjacencyEpsilon.
    private const double OverlapMargin = 1e-3;

    // Max ABUTMENT gap for the area-overlap pass: two polygons whose boundaries approach within this are
    // connected even if they do not overlap. netconvert's independent buffering leaves sub-mm-to-few-mm gaps
    // between a walkingArea and the sidewalks/crossings that meet it (the pedfrag witness gaps are <= ~2 mm);
    // 5 cm catches those with wide margin while staying far below the metre-scale spacing between genuinely
    // distinct walkable surfaces, so it never bridges unrelated polygons. Area-anchoring (below) keeps it from
    // connecting two non-area polygons regardless.
    private const double AbutProximityEps = 0.05;

    // P8-1c (docs/PEDESTRIAN-P8-1C-NAVMESH-CONTINUATION-DESIGN.md): the continuation-angle gate for the
    // sidewalk<->sidewalk near-abutment pass. Two buffered sidewalk strips are bridged only when their
    // outward end-tangents at the abutting ends are anti-parallel to within this angle -- a straight or
    // gently-bending CONTINUATION (~180 deg) -- and never when they meet at a junction CORNER (~90 deg),
    // which must still route through its walkingArea (the POC-0 no-shortcut invariant).
    //
    // PROVISIONAL: 135 deg is validated only on the synthetic straight-stub witness (subarea-pedfrag2),
    // where continuations sit at ~180 deg and corners at ~90 deg -- trivially separable. REAL sidewalks
    // curve, so a legit curving continuation may sit lower and a shallow corner higher; this awaits the
    // real-net seam end-tangent-angle distribution from the sub-area session (design doc Section 4). Kept a
    // single named constant so tuning is a one-line change.
    private const double ContinuationMinAngleDeg = 135.0;

    // theta >= ContinuationMinAngleDeg  <=>  cos(theta) <= cos(threshold); compared against Dot of the two
    // (unit) outward end-tangents, avoiding a trig call per pair.
    private static readonly double ContinuationMaxTangentDot =
        Math.Cos(ContinuationMinAngleDeg * Math.PI / 180.0);

    private readonly List<PolygonPortal>[] _adjacency;

    public PolygonGraph(IReadOnlyList<BakedPolygon> polygons)
        : this(polygons, pedConnections: null)
    {
    }

    // R1 (docs/PEDESTRIAN-R1-CONNECTION-STITCH-DESIGN.md): `pedConnections` (from PedNetwork) lets the graph
    // stitch portals the purely-geometric passes conservatively miss -- polygons the NET declares a pedestrian
    // walks between (split walkingArea pieces, corner-abutting sidewalks). Null / empty == the geometric-only
    // behaviour, so every existing caller and committed witness is unaffected.
    public PolygonGraph(IReadOnlyList<BakedPolygon> polygons, IReadOnlyList<Sim.Pedestrians.PedConnection>? pedConnections)
    {
        _adjacency = BuildAdjacency(polygons, pedConnections);
    }

    public IReadOnlyList<PolygonPortal> Neighbors(int polygonIndex) => _adjacency[polygonIndex];

    private static List<PolygonPortal>[] BuildAdjacency(
        IReadOnlyList<BakedPolygon> polygons, IReadOnlyList<Sim.Pedestrians.PedConnection>? pedConnections)
    {
        var n = polygons.Count;
        var adjacency = new List<PolygonPortal>[n];
        for (var i = 0; i < n; i++)
        {
            adjacency[i] = new List<PolygonPortal>();
        }

        for (var i = 0; i < n; i++)
        {
            for (var j = i + 1; j < n; j++)
            {
                var portal = FindPortal(polygons[i].Vertices, polygons[j].Vertices);
                if (portal is { } point)
                {
                    adjacency[i].Add(new PolygonPortal(j, point));
                    adjacency[j].Add(new PolygonPortal(i, point));
                }
            }
        }

        AddVertexProximityAdjacency(polygons, adjacency);
        AddAreaOverlapAdjacency(polygons, adjacency);
        AddSidewalkContinuationAdjacency(polygons, adjacency);
        AddNetConnectionAdjacency(polygons, pedConnections, adjacency);

        // Fixed iteration order per node: sort each neighbour list ascending by neighbour index,
        // so graph traversal (and hence A*) never depends on the O(n^2) build order above.
        for (var i = 0; i < n; i++)
        {
            adjacency[i].Sort((a, b) => a.Neighbor.CompareTo(b.Neighbor));
        }

        return adjacency;
    }

    // R1 pass (docs/PEDESTRIAN-R1-CONNECTION-STITCH-DESIGN.md): stitch portals for polygon pairs the NET
    // declares a pedestrian walks between (PedNetwork.PedConnections), where the geometric passes above did not
    // already connect them. Shortcut-SAFE by construction: it only adds an edge the net vouches for, so it can
    // never manufacture the POC-0 forbidden shortcut (a pair with no declared ped connection stays unlinked).
    // Portal point = the midpoint of the closest vertex pair between the two polygons (they abut in netconvert
    // geometry; the geometric pass merely wouldn't COMMIT to their ambiguous corner). A declared connection
    // whose polygons are implausibly far apart (> a size-RELATIVE threshold -- large roundabout/arterial
    // walkingAreas are legitimate, an absolute cap would false-positive on them) is skipped as a data smell.
    private static void AddNetConnectionAdjacency(
        IReadOnlyList<BakedPolygon> polygons,
        IReadOnlyList<Sim.Pedestrians.PedConnection>? pedConnections,
        List<PolygonPortal>[] adjacency)
    {
        if (pedConnections is null || pedConnections.Count == 0)
        {
            return;
        }

        var indexOfId = new Dictionary<string, int>(StringComparer.Ordinal);
        for (var i = 0; i < polygons.Count; i++)
        {
            indexOfId[polygons[i].Id] = i; // baked-polygon ids are lane ids -> unique
        }

        foreach (var conn in pedConnections)
        {
            if (!indexOfId.TryGetValue(conn.AId, out var a) || !indexOfId.TryGetValue(conn.BId, out var b) || a == b)
            {
                continue;
            }

            if (adjacency[a].Any(p => p.Neighbor == b))
            {
                continue; // already connected by a geometric pass -- the geometric portal wins
            }

            var (dist, mid) = ClosestVertexMidpoint(polygons[a].Vertices, polygons[b].Vertices);
            var extent = Math.Max(BoundingDiagonal(polygons[a].Vertices), BoundingDiagonal(polygons[b].Vertices));
            var farThreshold = Math.Max(2.0, 0.25 * extent); // size-relative (Q2): generous for large junctions
            if (dist > farThreshold)
            {
                continue; // declared-but-distant -> a data smell, not a real abutment; do not bridge across space
            }

            adjacency[a].Add(new PolygonPortal(b, mid));
            adjacency[b].Add(new PolygonPortal(a, mid));
        }
    }

    // Closest vertex pair between two polygons -> (distance, midpoint). Sufficient for netconvert output where
    // connected polygons abut at (near-)shared vertices; O(Vi*Vj), an offline bake cost.
    private static (double Dist, Vec2 Mid) ClosestVertexMidpoint(IReadOnlyList<Vec2> a, IReadOnlyList<Vec2> b)
    {
        var best = double.MaxValue;
        var mid = Vec2.Zero;
        foreach (var pa in a)
        {
            foreach (var pb in b)
            {
                var d = (pa - pb).AbsSq;
                if (d < best)
                {
                    best = d;
                    mid = new Vec2((pa.X + pb.X) * 0.5, (pa.Y + pb.Y) * 0.5);
                }
            }
        }

        return (Math.Sqrt(best), mid);
    }

    private static double BoundingDiagonal(IReadOnlyList<Vec2> verts)
    {
        if (verts.Count == 0)
        {
            return 0.0;
        }

        double minX = verts[0].X, minY = verts[0].Y, maxX = minX, maxY = minY;
        foreach (var v in verts)
        {
            if (v.X < minX) minX = v.X;
            if (v.X > maxX) maxX = v.X;
            if (v.Y < minY) minY = v.Y;
            if (v.Y > maxY) maxY = v.Y;
        }

        return new Vec2(maxX - minX, maxY - minY).Abs;
    }

    // Second adjacency pass (see class remarks): connects polygon pairs that share ONLY a corner
    // vertex, restricted to clusters touched by EXACTLY two distinct polygons so the POC-0
    // 3-polygon-corner bug can never recur. Union-find over every (polygon, vertex) pair is O(V^2)
    // in the total vertex count across the bake -- fine for an offline navmesh bake, not a per-step
    // cost.
    private static void AddVertexProximityAdjacency(
        IReadOnlyList<BakedPolygon> polygons, List<PolygonPortal>[] adjacency)
    {
        var verts = new List<(int Polygon, Vec2 Point)>();
        for (var i = 0; i < polygons.Count; i++)
        {
            foreach (var v in polygons[i].Vertices)
            {
                verts.Add((i, v));
            }
        }

        var parent = new int[verts.Count];
        for (var v = 0; v < verts.Count; v++)
        {
            parent[v] = v;
        }

        int Find(int x)
        {
            while (parent[x] != x)
            {
                parent[x] = parent[parent[x]]; // path halving
                x = parent[x];
            }

            return x;
        }

        void Union(int a, int b)
        {
            var ra = Find(a);
            var rb = Find(b);
            if (ra != rb)
            {
                parent[ra] = rb;
            }
        }

        for (var a = 0; a < verts.Count; a++)
        {
            for (var b = a + 1; b < verts.Count; b++)
            {
                if (PolygonGeometry.NearlyEqual(verts[a].Point, verts[b].Point, AdjacencyEpsilon))
                {
                    Union(a, b);
                }
            }
        }

        var clusters = new Dictionary<int, List<int>>();
        for (var v = 0; v < verts.Count; v++)
        {
            var root = Find(v);
            if (!clusters.TryGetValue(root, out var members))
            {
                clusters[root] = members = new List<int>();
            }

            members.Add(v);
        }

        foreach (var members in clusters.Values)
        {
            var distinctPolygons = new SortedSet<int>();
            foreach (var v in members)
            {
                distinctPolygons.Add(verts[v].Polygon);
            }

            if (distinctPolygons.Count != 2)
            {
                continue; // 1 (self-touch only) or 3+ (ambiguous corner, the POC-0 bug shape) -- skip
            }

            var it = distinctPolygons.GetEnumerator();
            it.MoveNext();
            var pi = it.Current;
            it.MoveNext();
            var pj = it.Current;

            if (adjacency[pi].Exists(p => p.Neighbor == pj))
            {
                continue; // already connected by the shared-edge pass (or an earlier cluster) -- edge/first wins
            }

            var point = Average(members.Select(v => verts[v].Point).ToList());
            adjacency[pi].Add(new PolygonPortal(pj, point));
            adjacency[pj].Add(new PolygonPortal(pi, point));
        }
    }

    // THIRD adjacency pass (P8-1b, docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): connects polygon
    // pairs that GENUINELY OVERLAP in 2D area but share no exact edge or coincident vertex -- the abutment
    // real netconvert geometry has, where independently-buffered sidewalk/crossing strips overlap the
    // junction walkingArea by a ~0.05-0.2 m sliver. Without this the surface fragments into thousands of
    // components on real crops (SUMOSHARP-P8-1-REAL-NET-NAVMESH.md); the synthetic grid shares exact edges
    // so it is unaffected.
    //
    // TWO safety properties keep this parity-neutral and invariant-preserving:
    //  1. AREA-ANCHORED: a portal is added only when at least one polygon is an AREA kind (WalkingArea /
    //     WalkablePolygon). Junction connectivity in SUMO always runs THROUGH a walkingArea (or plaza), so a
    //     sidewalk and a crossing are connected via the area, never directly -- this structurally preserves
    //     the POC-0 no-shortcut invariant (crossing<->sidewalk is never bridged here) and cannot add a
    //     sidewalk<->crossing / sidewalk<->sidewalk / crossing<->crossing portal.
    //  2. ADDITIVE + DEDUP: only pairs with no existing portal are considered, so every pair the shared-edge
    //     / vertex passes already connected (all of the synthetic-grid and POC-0 pairs) is left byte-identical
    //     -- this pass strictly ADDS the missing sliver-overlap portals that appear only on irregular geometry.
    // Genuine area overlap (not a corner touch) is required (PolygonGeometry.TryFindOverlapPortal), and the
    // portal sits inside the overlap region (inside the walkable union), so A* never routes across a
    // non-walkable notch -- the exact failure the vertex pass's 3-polygon-corner skip guards against.
    private static void AddAreaOverlapAdjacency(
        IReadOnlyList<BakedPolygon> polygons, List<PolygonPortal>[] adjacency)
    {
        var n = polygons.Count;
        for (var i = 0; i < n; i++)
        {
            for (var j = i + 1; j < n; j++)
            {
                if (!IsArea(polygons[i].Kind) && !IsArea(polygons[j].Kind))
                {
                    continue; // area-anchored: never bridge two non-area polygons (invariant safety)
                }

                if (adjacency[i].Exists(p => p.Neighbor == j))
                {
                    continue; // already connected by an earlier pass -- edge/vertex portal wins (dedup)
                }

                if (PolygonGeometry.TryFindOverlapPortal(polygons[i].Vertices, polygons[j].Vertices, OverlapMargin, AbutProximityEps, out var point))
                {
                    adjacency[i].Add(new PolygonPortal(j, point));
                    adjacency[j].Add(new PolygonPortal(i, point));
                }
            }
        }
    }

    // FOURTH adjacency pass (P8-1c, docs/PEDESTRIAN-P8-1C-NAVMESH-CONTINUATION-DESIGN.md): bridges
    // SidewalkSegment<->SidewalkSegment near-abutment at genuine CONTINUATIONS -- the residual real-net seams
    // the AREA-ANCHORED third pass structurally cannot connect (two buffered sidewalk strips meet within
    // AbutProximityEps with NO walkingArea between them: SUMO's dropped zero-area continuation walkingArea, or
    // an irregular seam with no area at all). Both leave the same signature -- two collinear sidewalk strips
    // <= 5 cm apart -- so this catches both the "dropped walkingArea" and "no walkingArea" cases (the witness
    // subarea-pedfrag2 embeds the latter, which approach B -- rehabilitating the dropped walkingArea -- could
    // not connect).
    //
    // THREE safety properties keep it invariant-preserving (POC-0 no-shortcut) and parity-neutral:
    //  1. CONTINUATION-GATED: only bridges when the two strips' outward end-tangents at their abutting ends
    //     are anti-parallel within ContinuationMinAngleDeg (IsCollinearContinuation) -- a straight/gently-
    //     bending continuation. A junction CORNER (~90 deg turn) is rejected, so A* is never handed a portal
    //     that would let it cut across the non-walkable junction interior (the exact failure the area-anchored
    //     pass avoided by refusing sidewalk<->sidewalk entirely). Corner connectivity still runs through the
    //     walkingArea via the third pass.
    //  2. ADDITIVE + DEDUP: only pairs with no existing portal are considered, so every pair the earlier
    //     passes connected (all synthetic-grid / POC-0 pairs, which share exact edges and have no <=5 cm gaps)
    //     is left byte-identical -- this pass strictly ADDS the missing collinear-sidewalk portals that appear
    //     only on irregular real geometry.
    //  3. SIDEWALK-ONLY: restricted to SidewalkSegment<->SidewalkSegment; crossing<->sidewalk and
    //     crossing<->crossing are never touched here.
    // The seam portal (TryFindOverlapPortal at the <=5 cm abutment) sits on the seam between the two strips,
    // physically within a ped's own body width across a <=5 cm gap -- the same seam portal the P8-1b area
    // abutment already places.
    private static void AddSidewalkContinuationAdjacency(
        IReadOnlyList<BakedPolygon> polygons, List<PolygonPortal>[] adjacency)
    {
        var n = polygons.Count;
        for (var i = 0; i < n; i++)
        {
            if (polygons[i].Kind != BakedPolygonKind.SidewalkSegment)
            {
                continue;
            }

            for (var j = i + 1; j < n; j++)
            {
                if (polygons[j].Kind != BakedPolygonKind.SidewalkSegment)
                {
                    continue;
                }

                if (adjacency[i].Exists(p => p.Neighbor == j))
                {
                    continue; // already connected by an earlier pass -- dedup
                }

                if (!IsCollinearContinuation(polygons[i].Spine, polygons[j].Spine))
                {
                    continue; // corner / non-continuation -- never bridge (no-shortcut invariant)
                }

                if (PolygonGeometry.TryFindOverlapPortal(
                        polygons[i].Vertices, polygons[j].Vertices, OverlapMargin, AbutProximityEps, out var point))
                {
                    adjacency[i].Add(new PolygonPortal(j, point));
                    adjacency[j].Add(new PolygonPortal(i, point));
                }
            }
        }
    }

    // True when two sidewalk spines meet end-to-end as a COLLINEAR continuation (P8-1c): the outward
    // end-tangents at their NEAREST ends point substantially at each other (angle >= ContinuationMinAngleDeg,
    // i.e. Dot <= ContinuationMaxTangentDot). A junction corner turns ~90 deg (Dot ~0) and is rejected. The
    // nearest-end pairing localizes the tangent to the abutting end, so a strip that bends far from the seam
    // does not skew the angle. Requires both spines to have >= 2 points; returns false otherwise (no tangent
    // -> not a continuation -> leave the pair unbridged, the safe default).
    private static bool IsCollinearContinuation(IReadOnlyList<Vec2>? spineA, IReadOnlyList<Vec2>? spineB)
    {
        if (spineA is not { Count: >= 2 } || spineB is not { Count: >= 2 })
        {
            return false;
        }

        // Each spine's two endpoints and the outward tangent there (endpoint - its inner neighbour, so it
        // points OUT of the strip toward a potential seam).
        var a0 = spineA[0];
        var a0T = Normalize(spineA[0] - spineA[1]);
        var a1 = spineA[^1];
        var a1T = Normalize(spineA[^1] - spineA[^2]);
        var b0 = spineB[0];
        var b0T = Normalize(spineB[0] - spineB[1]);
        var b1 = spineB[^1];
        var b1T = Normalize(spineB[^1] - spineB[^2]);

        // Pick the nearest end pair (the ends that actually abut) and take those two outward tangents.
        var best = double.MaxValue;
        var ta = Vec2.Zero;
        var tb = Vec2.Zero;
        void Consider(Vec2 pa, Vec2 tpa, Vec2 pb, Vec2 tpb)
        {
            var d = (pa - pb).AbsSq;
            if (d < best)
            {
                best = d;
                ta = tpa;
                tb = tpb;
            }
        }

        Consider(a0, a0T, b0, b0T);
        Consider(a0, a0T, b1, b1T);
        Consider(a1, a1T, b0, b0T);
        Consider(a1, a1T, b1, b1T);

        // Anti-parallel outward tangents == the two strips continue each other in a line. A zero-length
        // tangent (degenerate spine end) yields Dot 0, which fails the <= negative-cos threshold -> false.
        return Vec2.Dot(ta, tb) <= ContinuationMaxTangentDot;
    }

    private static Vec2 Normalize(Vec2 v)
    {
        var m = v.Abs;
        return m > 1e-12 ? v / m : Vec2.Zero;
    }

    private static bool IsArea(BakedPolygonKind kind) =>
        kind is BakedPolygonKind.WalkingArea or BakedPolygonKind.WalkablePolygon;

    private static Vec2? FindPortal(IReadOnlyList<Vec2> a, IReadOnlyList<Vec2> b)
    {
        var edgeMidpoints = new List<Vec2>();
        for (var i = 0; i < a.Count; i++)
        {
            var a0 = a[i];
            var a1 = a[(i + 1) % a.Count];
            for (var j = 0; j < b.Count; j++)
            {
                var b0 = b[j];
                var b1 = b[(j + 1) % b.Count];
                var sameOrder = PolygonGeometry.NearlyEqual(a0, b0, AdjacencyEpsilon)
                    && PolygonGeometry.NearlyEqual(a1, b1, AdjacencyEpsilon);
                var reversed = PolygonGeometry.NearlyEqual(a0, b1, AdjacencyEpsilon)
                    && PolygonGeometry.NearlyEqual(a1, b0, AdjacencyEpsilon);
                if (sameOrder || reversed)
                {
                    edgeMidpoints.Add(0.5 * (a0 + a1));
                }
            }
        }

        return edgeMidpoints.Count > 0 ? Average(edgeMidpoints) : null;
    }

    private static Vec2 Average(List<Vec2> points)
    {
        double sx = 0.0, sy = 0.0;
        foreach (var p in points)
        {
            sx += p.X;
            sy += p.Y;
        }

        return new Vec2(sx / points.Count, sy / points.Count);
    }
}
