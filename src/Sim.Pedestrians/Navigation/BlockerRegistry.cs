using Sim.Core.Orca;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;

namespace Sim.Pedestrians.Navigation;

// P2-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §4/§6): generalizes POC-5's one-off
// "blocked = a hand-picked HashSet<int>" (RerouteTests) into a managed, stable-identity registry of
// DYNAMIC obstacles, each mapped to the set of baked walkable polygons (WalkablePolygonBaker.Bake)
// it occludes. This is the production input to SumoNavMesh.FindPath(start, goal, blocked) -- see
// BlockedPolygons() below -- and to RerouteDriver, which watches this registry's output for changes.
//
// Shape/style mirrors InterestField (P1-1, src/Sim.Pedestrians/Lod/InterestField.cs): an opaque,
// never-reused id minted on Register, O(1) Remove-by-id, and a deterministic ascending-id Snapshot.
// Unlike InterestField this registry is not spatially indexed -- blockers, like interest sources,
// are "few" (a handful of parked cars / temporary obstructions at once), and the O(blockers *
// polygons) recompute below is cheap at that scale; see Recompute's remarks if that stops holding.
//
// A registered blocker is an arbitrary CONVEX polygon, vertices wound COUNTERCLOCKWISE (the same
// convention BoxObstacle.Corners already produces for an oriented box -- "interior on the left of
// each directed edge", docs BoxObstacle.cs remarks). Convexity is required only of the BLOCKER (used
// as the Sutherland-Hodgman clip window below); the walkable polygon being tested against it may be
// non-convex (a bent sidewalk strip, P2-1) with no loss of correctness -- see PolygonOverlapArea.
public readonly record struct BlockerId(long Value)
{
    public static readonly BlockerId Invalid = new(0);

    public bool IsValid => Value != 0;
}

public sealed class BlockerRegistry
{
    // Default overlap-area threshold (m^2): a blocker must occlude more than this much of a walkable
    // polygon's interior to count as "blocking" it. Chosen well above the shallow shared-corner
    // overlap between independently-baked ADJACENT polygons (documented in RerouteTests.cs as
    // ~0.05-0.2 m of BOUNDARY distance, not area, but kept here as the same order-of-magnitude
    // "ignore a sliver" floor) so a blocker merely grazing a polygon's edge does not spuriously mark
    // that whole polygon impassable.
    public const double DefaultOverlapAreaThreshold = 0.05;

    private sealed class Entry
    {
        public required IReadOnlyList<Vec2> Vertices;
    }

    private readonly IReadOnlyList<BakedPolygon> _polygons;
    private readonly double _overlapAreaThreshold;

    private readonly Dictionary<BlockerId, Entry> _blockers = new();
    private long _nextId = 1;

    // Recomputed by Recompute() on every Register/Unregister -- see BlockedPolygons() remarks for why
    // this is push-on-write rather than computed lazily per query.
    private HashSet<int> _blockedPolygons = new();

    public BlockerRegistry(IReadOnlyList<BakedPolygon> polygons, double overlapAreaThreshold = DefaultOverlapAreaThreshold)
    {
        _polygons = polygons;
        _overlapAreaThreshold = overlapAreaThreshold;
    }

    public int Count => _blockers.Count;

    // Registers a new convex blocker polygon (CCW-wound, see class remarks) under a fresh,
    // never-reused id, and immediately recomputes the blocked-polygon set. O(blockers * polygons).
    public BlockerId Register(IReadOnlyList<Vec2> vertices)
    {
        ArgumentNullException.ThrowIfNull(vertices);
        if (vertices.Count < 3)
        {
            throw new ArgumentException("A blocker polygon needs at least 3 vertices.", nameof(vertices));
        }

        var id = new BlockerId(_nextId++);
        _blockers.Add(id, new Entry { Vertices = vertices });
        Recompute();
        return id;
    }

    // Convenience overload for the common "parked car" case (POC-5 Req 3): builds the CCW box corners
    // via BoxObstacle.Corners and registers those directly.
    public BlockerId RegisterBox(Vec2 center, double halfExtentX, double halfExtentY, double angleRadians)
        => Register(BoxObstacle.Corners(center, halfExtentX, halfExtentY, angleRadians));

    // Inert (returns false) if `id` is already gone -- mirrors OrcaCrowd.Remove / InterestField.Remove's
    // established "removing something already gone is harmless" convention.
    public bool Unregister(BlockerId id)
    {
        if (!_blockers.Remove(id))
        {
            return false;
        }

        Recompute();
        return true;
    }

    public bool Contains(BlockerId id) => _blockers.ContainsKey(id);

    public IReadOnlyList<Vec2> VerticesOf(BlockerId id) => _blockers[id].Vertices;

    // A deterministic (ascending-id) snapshot of every registered blocker -- for telemetry/tests, not
    // consumed by RerouteDriver itself (it only calls BlockedPolygons()).
    public IReadOnlyList<(BlockerId Id, IReadOnlyList<Vec2> Vertices)> Snapshot()
    {
        var ids = new List<BlockerId>(_blockers.Keys);
        ids.Sort((a, b) => a.Value.CompareTo(b.Value));

        var result = new List<(BlockerId, IReadOnlyList<Vec2>)>(ids.Count);
        foreach (var id in ids)
        {
            result.Add((id, _blockers[id].Vertices));
        }

        return result;
    }

    // The set of baked-polygon indices currently occluded by ANY registered blocker -- feeds directly
    // into SumoNavMesh.FindPath(start, goal, blockedPolygonIndices) (POC-5's blocked-set overload).
    // Returned set is a live reference recomputed on every Register/Unregister; callers must not
    // mutate it (RerouteDriver treats it as read-only, matching IReadOnlySet's contract).
    public IReadOnlySet<int> BlockedPolygons() => _blockedPolygons;

    // Recomputes the full blocked set from scratch: for every (blocker, polygon) pair, a polygon is
    // blocked iff its overlap area with THAT SINGLE blocker exceeds the threshold (no accumulation
    // across multiple partially-overlapping blockers -- the design's "a blocker polygon/box that
    // overlaps a walkable polygon by more than a threshold marks it blocked" is a per-blocker test).
    // This makes the result independent of registration order (a pure OR across blockers), which is
    // what makes Unregister-then-Register-elsewhere and flicker (Register/Unregister in any order)
    // deterministic and order-independent -- important for RerouteDriver's hysteresis to reason about
    // "is polygon i blocked" as a single, well-defined boolean per step.
    //
    // Cost: O(blockers * polygons), each pair a small Sutherland-Hodgman clip (O(polygon vertices)).
    // Fine at the design's stated scale ("a handful of parked cars / temporary obstructions", not
    // thousands); if that stops holding, bound blocker/polygon pairs with a bounding-box prefilter or
    // adopt InterestField's grid-bucket scheme (P1-1) before falling back to the full clip.
    private void Recompute()
    {
        var blocked = new HashSet<int>();
        if (_blockers.Count == 0)
        {
            _blockedPolygons = blocked;
            return;
        }

        var ids = new List<BlockerId>(_blockers.Keys);
        ids.Sort((a, b) => a.Value.CompareTo(b.Value)); // deterministic iteration; result itself is order-independent (see above)

        foreach (var id in ids)
        {
            var blockerVerts = _blockers[id].Vertices;
            for (var i = 0; i < _polygons.Count; i++)
            {
                if (blocked.Contains(i))
                {
                    continue; // already blocked by an earlier blocker in this pass; still order-independent overall
                }

                var overlap = PolygonOverlapArea(blockerVerts, _polygons[i].Vertices);
                if (overlap > _overlapAreaThreshold)
                {
                    blocked.Add(i);
                }
            }
        }

        _blockedPolygons = blocked;
    }

    // Overlap area of a CONVEX clip polygon (the blocker) against an arbitrary (possibly non-convex,
    // e.g. a bent-sidewalk P2-1 strip) subject polygon (the walkable polygon), via Sutherland-Hodgman
    // clipping of `subject` against each half-plane of `convexClip` in turn. Sutherland-Hodgman only
    // requires the CLIP polygon to be convex for the result's boundary to be exact; the subject may
    // be non-convex -- each single half-plane clip is an exact cut of an arbitrary polygon, and any
    // "bridge" edges the algorithm introduces run exactly along the clip line, contributing zero net
    // signed area (out-and-back), so the shoelace area of the final (possibly self-touching) ring is
    // still the true intersection area for this purpose.
    internal static double PolygonOverlapArea(IReadOnlyList<Vec2> convexClip, IReadOnlyList<Vec2> subject)
    {
        var clipped = ClipPolygon(subject, convexClip);
        return clipped.Count < 3 ? 0.0 : Math.Abs(PolygonGeometry.SignedArea(clipped));
    }

    private static List<Vec2> ClipPolygon(IReadOnlyList<Vec2> subject, IReadOnlyList<Vec2> convexClip)
    {
        var output = new List<Vec2>(subject);
        var n = convexClip.Count;
        for (var i = 0; i < n && output.Count > 0; i++)
        {
            var a = convexClip[i];
            var b = convexClip[(i + 1) % n];
            output = ClipAgainstEdge(output, a, b);
        }

        return output;
    }

    // One Sutherland-Hodgman pass against the half-plane of directed edge a->b (interior = left of
    // a->b, the CCW convention this whole codebase uses -- see class remarks).
    private static List<Vec2> ClipAgainstEdge(List<Vec2> polygon, Vec2 a, Vec2 b)
    {
        var output = new List<Vec2>();
        var n = polygon.Count;
        if (n == 0)
        {
            return output;
        }

        for (var i = 0; i < n; i++)
        {
            var current = polygon[i];
            var previous = polygon[(i - 1 + n) % n];

            var currentInside = IsLeftOf(a, b, current);
            var previousInside = IsLeftOf(a, b, previous);

            if (currentInside)
            {
                if (!previousInside)
                {
                    output.Add(LineIntersection(previous, current, a, b));
                }

                output.Add(current);
            }
            else if (previousInside)
            {
                output.Add(LineIntersection(previous, current, a, b));
            }
        }

        return output;
    }

    private static bool IsLeftOf(Vec2 a, Vec2 b, Vec2 p) => Vec2.Det(b - a, p - a) >= 0.0;

    // Intersection of infinite lines (p1,p2) and (a,b). Both segments passed in are always
    // non-degenerate here (adjacent polygon vertices / a real clip edge), and this is only ever
    // called when p1/p2 straddle line (a,b) (one inside, one outside), so the lines are guaranteed
    // non-parallel in every real call; the parallel fallback below only guards float edge cases.
    private static Vec2 LineIntersection(Vec2 p1, Vec2 p2, Vec2 a, Vec2 b)
    {
        var d1 = p2 - p1;
        var d2 = b - a;
        var denom = Vec2.Det(d1, d2);
        if (Math.Abs(denom) < 1e-12)
        {
            return p1;
        }

        var t = Vec2.Det(a - p1, d2) / denom;
        return p1 + (t * d1);
    }
}
