using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Sim.Core.Orca;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Navigation;

// P8-1b (docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md): the walkable bake must connect REAL
// netconvert geometry, where independently-buffered sidewalk/crossing strips OVERLAP the junction
// walkingArea by a ~0.05-0.2 m sliver instead of sharing exact edges/vertices. The prior bake connected
// only exact shared edges / 2-polygon shared corners, so real crops fragmented into ~1000 components and
// O/D routing failed (SUMOSHARP-P8-1-REAL-NET-NAVMESH.md). These tests reproduce the fragmentation from
// HAND-BUILT polygons (no SUMO/real geometry needed) and pin the area-overlap fix + its invariants.
public class NavmeshConnectivityTests
{
    private readonly ITestOutputHelper _out;

    public NavmeshConnectivityTests(ITestOutputHelper output) => _out = output;

    // Axis-aligned rectangle (CCW), as one baked polygon.
    private static BakedPolygon Rect(int index, string id, BakedPolygonKind kind, double x0, double y0, double x1, double y1)
        => new(index, id, kind, new[] { new Vec2(x0, y0), new Vec2(x1, y0), new Vec2(x1, y1), new Vec2(x0, y1) });

    // A mini-junction the way netconvert emits it: a central walkingArea, two sidewalk strips and a
    // crossing that each OVERLAP the walkingArea by 0.1 m (vertices NON-coincident, no shared exact edge),
    // and do NOT overlap one another (they meet the area from different sides). WA = [0,4]x[0,4].
    private static List<BakedPolygon> MiniJunction(bool includeWalkingArea)
    {
        var polys = new List<BakedPolygon>();
        var i = 0;
        if (includeWalkingArea)
        {
            polys.Add(Rect(i++, "wa", BakedPolygonKind.WalkingArea, 0, 0, 4, 4));
        }

        polys.Add(Rect(i++, "sa", BakedPolygonKind.SidewalkSegment, -10, 1, 0.1, 3));   // left, overlaps WA x in [0,0.1]
        polys.Add(Rect(i++, "sb", BakedPolygonKind.SidewalkSegment, 3.9, 1, 14, 3));    // right, overlaps WA x in [3.9,4]
        polys.Add(Rect(i++, "xc", BakedPolygonKind.Crossing, 1, -10, 3, 0.1));          // below, overlaps WA y in [0,0.1]
        return polys;
    }

    [Fact]
    public void AreaOverlap_ConnectsAbuttingStrips_ThroughTheWalkingArea_OneComponent()
    {
        var polys = MiniJunction(includeWalkingArea: true);
        var nav = new SumoNavMesh(polys);

        // The whole junction is one connected component (was 4 before the area-overlap pass: no strip shares
        // an exact edge/vertex with the walkingArea, so the shared-edge/vertex passes connected nothing).
        Assert.Equal(1, nav.ConnectedComponentCount());

        // A ped can route from the crossing across the junction to the far sidewalk...
        var path = nav.FindPath(new Vec2(2, -5), new Vec2(-5, 2));
        Assert.NotNull(path);

        // ...and the route goes THROUGH the walkingArea (a waypoint lands inside WA=[0,4]x[0,4]), never a
        // direct crossing->sidewalk shortcut -- the POC-0 no-shortcut invariant, preserved by construction.
        Assert.Contains(path!, wp => wp.X is > 0.0 and < 4.0 && wp.Y is > 0.0 and < 4.0);

        _out.WriteLine($"[P8-1b] mini-junction: 1 component, cross-junction path = {path!.Count} waypoints via the area");
    }

    [Fact]
    public void AreaOverlap_NeverBridgesTwoNonAreaPolygons()
    {
        // Same layout but with the walkingArea REMOVED: the crossing and the two sidewalks overlap only the
        // (now absent) area, never each other, and all three are non-area kinds -- so the area-anchored
        // overlap pass connects nothing. They stay 3 separate components. This is the invariant that keeps a
        // crossing from being bridged directly to a sidewalk (the POC-0 shortcut bug).
        var polys = MiniJunction(includeWalkingArea: false);
        var nav = new SumoNavMesh(polys);

        Assert.Equal(3, nav.ConnectedComponentCount());
        Assert.Null(nav.FindPath(new Vec2(2, -5), new Vec2(-5, 2))); // crossing -> far sidewalk unreachable
        _out.WriteLine("[P8-1b] non-area polygons are never overlap-bridged (crossing<->sidewalk stays disconnected)");
    }

    [Fact]
    public void SeparatedNonTouchingPolygons_AreNotOverlapConnected()
    {
        // A walkingArea and a crossing that neither touch nor overlap (a 0.5 m gap) must stay two components:
        // the overlap pass connects only genuine area overlaps, never a near-miss. (Guards against the pass
        // over-connecting once it stopped requiring an exact shared edge/vertex.)
        var wa = Rect(0, "wa", BakedPolygonKind.WalkingArea, 0, 0, 4, 4);
        var xc = Rect(1, "xc", BakedPolygonKind.Crossing, 4.5, 0, 8, 4); // 0.5 m gap from WA
        var nav = new SumoNavMesh(new List<BakedPolygon> { wa, xc });

        Assert.Equal(2, nav.ConnectedComponentCount());
        _out.WriteLine("[P8-1b] a non-touching gap is not an overlap -> not bridged");
    }

    [Fact]
    public void ConnectedComponentCount_CountsDisjointIslands()
    {
        var far = new List<BakedPolygon>
        {
            Rect(0, "a", BakedPolygonKind.WalkablePolygon, 0, 0, 2, 2),
            Rect(1, "b", BakedPolygonKind.WalkablePolygon, 100, 100, 102, 102),
        };
        Assert.Equal(2, new SumoNavMesh(far).ConnectedComponentCount());
    }

    [Fact]
    public void SyntheticBox_StaysOneComponent()
    {
        // The committed synthetic box already connected (shared exact edges) -> 1 component; the additive
        // overlap pass must not change that. Anchors the report's "synthetic grid -> 1 component" baseline.
        var dir = AppContext.BaseDirectory;
        while (dir is not null && !File.Exists(Path.Combine(dir, "Traffic.sln")))
        {
            dir = Path.GetDirectoryName(dir);
        }

        Assert.NotNull(dir);
        var boxNet = Path.Combine(dir!, "scenarios", "_ped", "subarea-box", "net.xml");
        var network = PedNetworkParser.Load(boxNet);
        var polygons = WalkablePolygonBaker.Bake(network);
        var nav = new SumoNavMesh(polygons);

        Assert.Equal(1, nav.ConnectedComponentCount());
        _out.WriteLine($"[P8-1b] synthetic box: {polygons.Count} polygons, {nav.ConnectedComponentCount()} component");
    }
}
