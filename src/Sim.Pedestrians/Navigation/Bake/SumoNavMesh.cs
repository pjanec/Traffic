using Sim.Core.Orca;

namespace Sim.Pedestrians.Navigation.Bake;

// IPedNavigation over a WalkablePolygonBaker.Bake() polygon set (docs/PEDESTRIAN-DESIGN.md §4,
// PEDESTRIAN-POC-PLAN.md POC-1a "SUMO-geometry bake" provider). Strategic routing is A* over the
// polygon-adjacency graph (PolygonGraph): each polygon is a node, each shared boundary a portal
// edge, edge cost and the search heuristic are both the Euclidean distance between polygon
// VERTEX-AVERAGE centroids (BakedPolygon.Centroid) -- using the same metric for cost and heuristic
// keeps the heuristic consistent (never overestimates), so the search is optimal over this graph.
// Deterministic: fixed adjacency iteration order (PolygonGraph), ties broken by polygon Index (the
// task's "deterministic tie-break by polygon index").
public sealed class SumoNavMesh : IPedNavigation
{
    private readonly IReadOnlyList<BakedPolygon> _polygons;
    private readonly PolygonGraph _graph;
    private readonly SumoWalkableSpace _space;

    public SumoNavMesh(IReadOnlyList<BakedPolygon> polygons)
        : this(polygons, new SumoWalkableSpace(polygons))
    {
    }

    // Overload for callers that already built (and want to share) a SumoWalkableSpace over the
    // same polygon set, instead of this constructing its own.
    public SumoNavMesh(IReadOnlyList<BakedPolygon> polygons, SumoWalkableSpace space)
    {
        _polygons = polygons;
        _graph = new PolygonGraph(polygons);
        _space = space;
    }

    public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => FindPath(start, goal, blockedPolygonIndices: null);

    /// P8-1b (docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): number of connected components in the
    /// portal-adjacency graph -- a direct diagnostic for the real-geometry fragmentation bug. A well-connected
    /// crop is 1 (or a few large) components; ~1000 means the surface shattered and O/D routing will fail.
    /// Deterministic (BFS over the fixed adjacency); offline, not a per-step cost.
    public int ConnectedComponentCount()
    {
        var n = _polygons.Count;
        var seen = new bool[n];
        var components = 0;
        var stack = new Stack<int>();
        for (var start = 0; start < n; start++)
        {
            if (seen[start])
            {
                continue;
            }

            components++;
            seen[start] = true;
            stack.Push(start);
            while (stack.Count > 0)
            {
                var cur = stack.Pop();
                foreach (var portal in _graph.Neighbors(cur))
                {
                    if (!seen[portal.Neighbor])
                    {
                        seen[portal.Neighbor] = true;
                        stack.Push(portal.Neighbor);
                    }
                }
            }
        }

        return components;
    }

    // ADDITIVE (POC-5, docs/PEDESTRIAN-POC-PLAN.md POC-5 "reroute"; docs/PEDESTRIAN-DESIGN.md §6
    // "full occlusion of a portal triggers a strategic reroute"): same A* as the two-argument
    // overload above, but polygons in `blockedPolygonIndices` are excluded from the adjacency graph
    // entirely -- neither traversable nor a valid start/goal location -- so a caller whose portal
    // (e.g. a crossing) is fully occluded can re-query for a path that routes around it. Passing
    // `null` or an empty set reproduces the two-argument overload's result exactly (same code path,
    // same deterministic tie-breaks), so that overload's behaviour for every existing caller
    // (POC-1/POC-3) is unchanged byte-for-byte.
    public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal, IReadOnlySet<int>? blockedPolygonIndices)
    {
        var blocked = (blockedPolygonIndices is { Count: > 0 }) ? blockedPolygonIndices : null;

        var startPolygon = LocatePolygon(start, blocked);
        var goalPolygon = LocatePolygon(goal, blocked);
        if (startPolygon < 0 || goalPolygon < 0)
        {
            return null; // walkable space is empty, every candidate polygon is blocked, or the
                         // point cannot be located/clamped at all
        }

        if (startPolygon == goalPolygon)
        {
            // Same local region: for a small convex-ish polygon (crossing/walkingarea/walkable
            // polygon, or a straight sidewalk quad) a direct segment is a valid corridor path. A
            // BENT sidewalk's whole-lane polygon (P2-1, PolylineBuffer) is generally NON-CONVEX,
            // so a naive direct segment between two points on different arms can cut outside the
            // strip across the elbow -- thread through the lane's Spine instead whenever it has a
            // genuine bend (3+ points); see ThreadThroughSpine / BakedPolygon.Spine remarks. A
            // straight (2-point) Spine, or no Spine at all, falls through to the pre-P2-1 direct
            // segment unchanged.
            var spine = _polygons[startPolygon].Spine;
            if (spine is { Count: >= 3 })
            {
                return ThreadThroughSpine(spine, start, goal);
            }

            return new[] { start, goal };
        }

        var nodePath = FindNodePath(startPolygon, goalPolygon, blocked);
        if (nodePath is null)
        {
            return null; // disconnected in the (possibly blocked) adjacency graph -> unreachable
        }

        var waypoints = new List<Vec2> { start };
        for (var i = 0; i + 1 < nodePath.Count; i++)
        {
            var from = nodePath[i];
            var to = nodePath[i + 1];
            var portal = _graph.Neighbors(from).First(p => p.Neighbor == to).Point;
            waypoints.Add(portal);
        }

        waypoints.Add(goal);
        return waypoints;
    }

    // Locates the polygon containing `p`; if none does, snaps `p` onto walkable space first (the
    // interface's documented off-mesh spawn/goal case) and locates from there, falling back to the
    // nearest polygon by boundary distance if even the clamped point lands exactly on a seam.
    // `blocked` (nullable -- the common, unblocked case skips the containment check entirely) polygons
    // are never returned: a blocked polygon is not a legal start/goal location either, not just a
    // non-traversable graph node.
    private int LocatePolygon(Vec2 p, IReadOnlySet<int>? blocked)
    {
        var direct = IndexOfContaining(p, blocked);
        if (direct >= 0)
        {
            return direct;
        }

        if (_polygons.Count == 0)
        {
            return -1;
        }

        var clamped = _space.ClampToWalkable(p);
        var afterClamp = IndexOfContaining(clamped, blocked);
        if (afterClamp >= 0)
        {
            return afterClamp;
        }

        var best = -1;
        var bestDistSq = double.MaxValue;
        for (var i = 0; i < _polygons.Count; i++)
        {
            if (blocked is not null && blocked.Contains(i))
            {
                continue;
            }

            PolygonGeometry.NearestPointOnBoundary(_polygons[i].Vertices, clamped, out var distSq);
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                best = i;
            }
        }

        return best;
    }

    private int IndexOfContaining(Vec2 p, IReadOnlySet<int>? blocked)
    {
        for (var i = 0; i < _polygons.Count; i++)
        {
            if (blocked is not null && blocked.Contains(i))
            {
                continue;
            }

            if (PolygonGeometry.Contains(_polygons[i].Vertices, p))
            {
                return i;
            }
        }

        return -1;
    }

    private List<int>? FindNodePath(int start, int goal, IReadOnlySet<int>? blocked)
    {
        var open = new List<int> { start };
        var inOpen = new HashSet<int> { start };
        var closed = new HashSet<int>();
        var cameFrom = new Dictionary<int, int>();
        var gScore = new Dictionary<int, double> { [start] = 0.0 };

        double FScore(int node) => gScore[node] + Heuristic(node, goal);

        while (open.Count > 0)
        {
            // Deterministic tie-break: lowest f-score, then lowest polygon index.
            open.Sort((a, b) =>
            {
                var cmp = FScore(a).CompareTo(FScore(b));
                return cmp != 0 ? cmp : a.CompareTo(b);
            });

            var current = open[0];
            open.RemoveAt(0);
            inOpen.Remove(current);

            if (current == goal)
            {
                return ReconstructPath(cameFrom, current);
            }

            closed.Add(current);

            foreach (var portal in _graph.Neighbors(current))
            {
                if (closed.Contains(portal.Neighbor))
                {
                    continue;
                }

                if (blocked is not null && blocked.Contains(portal.Neighbor))
                {
                    continue; // blocked polygon: excluded from the graph entirely (POC-5 reroute)
                }

                var tentativeG = gScore[current] + CentroidDistance(current, portal.Neighbor);
                if (!gScore.TryGetValue(portal.Neighbor, out var existingG) || tentativeG < existingG)
                {
                    cameFrom[portal.Neighbor] = current;
                    gScore[portal.Neighbor] = tentativeG;
                    if (inOpen.Add(portal.Neighbor))
                    {
                        open.Add(portal.Neighbor);
                    }
                }
            }
        }

        return null;
    }

    // P2-1: builds a path from `start` to `goal` (both known to lie in the same bent-sidewalk
    // polygon) that threads through `spine`'s interior vertices between the two points' projections,
    // instead of a direct segment that can cut outside the strip across a bend's elbow. `spine` is
    // the lane's ORIGINAL centreline polyline (BakedPolygon.Spine) -- following it (offset by at
    // most the strip's own half-width from `start`/`goal` to the nearest spine vertices actually
    // used) stays inside the buffered strip as long as the strip's width covers that offset, which
    // holds for any point actually placed on/near the lane the way SumoWalkableSpace.ClampToWalkable
    // and normal spawn/goal placement do.
    private static IReadOnlyList<Vec2> ThreadThroughSpine(IReadOnlyList<Vec2> spine, Vec2 start, Vec2 goal)
    {
        var startPos = NearestPositionOnSpine(spine, start);
        var goalPos = NearestPositionOnSpine(spine, goal);

        var waypoints = new List<Vec2> { start };
        if (startPos <= goalPos)
        {
            for (var idx = 1; idx < spine.Count - 1; idx++)
            {
                if (idx > startPos && idx < goalPos)
                {
                    waypoints.Add(spine[idx]);
                }
            }
        }
        else
        {
            for (var idx = spine.Count - 2; idx >= 1; idx--)
            {
                if (idx < startPos && idx > goalPos)
                {
                    waypoints.Add(spine[idx]);
                }
            }
        }

        waypoints.Add(goal);
        return waypoints;
    }

    // Position of `p`'s nearest point on the polyline `spine`, expressed as `segmentIndex + t`
    // (t in [0,1], the fraction along that segment) -- monotonically increasing along the spine's
    // own vertex order, which is all ThreadThroughSpine needs to pick out (and correctly order) the
    // interior vertices strictly between two projected positions. Not true arc length, but that is
    // never required here: a spine VERTEX `idx` has position exactly `idx` (t=0 at its own segment),
    // so the "idx strictly between startPos and goalPos" comparisons above are exact regardless of
    // per-segment length.
    private static double NearestPositionOnSpine(IReadOnlyList<Vec2> spine, Vec2 p)
    {
        var bestDistSq = double.MaxValue;
        var bestPos = 0.0;
        for (var s = 0; s + 1 < spine.Count; s++)
        {
            var a = spine[s];
            var b = spine[s + 1];
            var candidate = PolygonGeometry.NearestPointOnSegment(a, b, p);
            var distSq = (candidate - p).AbsSq;
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                var segLenSq = (b - a).AbsSq;
                var t = segLenSq > PolygonGeometry.DegenerateLengthSq
                    ? Math.Clamp(Vec2.Dot(candidate - a, b - a) / segLenSq, 0.0, 1.0)
                    : 0.0;
                bestPos = s + t;
            }
        }

        return bestPos;
    }

    private double Heuristic(int node, int goal) => CentroidDistance(node, goal);

    private double CentroidDistance(int a, int b) => (_polygons[a].Centroid - _polygons[b].Centroid).Abs;

    private static List<int> ReconstructPath(Dictionary<int, int> cameFrom, int current)
    {
        var path = new List<int> { current };
        while (cameFrom.TryGetValue(current, out var prev))
        {
            current = prev;
            path.Add(current);
        }

        path.Reverse();
        return path;
    }
}
