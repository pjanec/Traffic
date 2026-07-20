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

    private readonly List<PolygonPortal>[] _adjacency;

    public PolygonGraph(IReadOnlyList<BakedPolygon> polygons)
    {
        _adjacency = BuildAdjacency(polygons);
    }

    public IReadOnlyList<PolygonPortal> Neighbors(int polygonIndex) => _adjacency[polygonIndex];

    private static List<PolygonPortal>[] BuildAdjacency(IReadOnlyList<BakedPolygon> polygons)
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

        // Fixed iteration order per node: sort each neighbour list ascending by neighbour index,
        // so graph traversal (and hence A*) never depends on the O(n^2) build order above.
        for (var i = 0; i < n; i++)
        {
            adjacency[i].Sort((a, b) => a.Neighbor.CompareTo(b.Neighbor));
        }

        return adjacency;
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

                if (PolygonGeometry.TryFindOverlapPortal(polygons[i].Vertices, polygons[j].Vertices, OverlapMargin, out var point))
                {
                    adjacency[i].Add(new PolygonPortal(j, point));
                    adjacency[j].Add(new PolygonPortal(i, point));
                }
            }
        }
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
