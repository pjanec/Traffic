using System.Collections.Generic;
using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;

namespace Sim.Pedestrians.Tests.Navigation;

// R1 (docs/PEDESTRIAN-R1-CONNECTION-STITCH-DESIGN.md): the net-connection navmesh stitch. Three ped polygons
// meet at ONE shared corner point and share no edge -- a 3-party corner cluster the geometric adjacency passes
// deliberately skip (the POC-0 anti-shortcut guard), so geometry alone leaves them as THREE components. When
// the net DECLARES pedestrian connections between them (PedConnection), the stitch bridges those pairs into one
// component -- and a declared-but-DISTANT connection is refused by the size-relative far guard. This is the
// "necessary, not sufficient" proof that the pass actually merges (independent of any real net).
public class NavConnectionStitchTests
{
    // Three triangles meeting only at the origin, pairwise sharing no edge (so the only contact is the
    // 3-party corner the geometric passes skip).
    private static IReadOnlyList<BakedPolygon> ThreeCornerPolys()
    {
        var a = new[] { new Vec2(0, 0), new Vec2(-2, 0), new Vec2(-2, -2) };
        var b = new[] { new Vec2(0, 0), new Vec2(2, -2), new Vec2(2, 0) };
        var c = new[] { new Vec2(0, 0), new Vec2(0, 2), new Vec2(-2, 2) };
        return new[]
        {
            new BakedPolygon(0, "e_a_0", BakedPolygonKind.SidewalkSegment, a),
            new BakedPolygon(1, "e_b_0", BakedPolygonKind.SidewalkSegment, b),
            new BakedPolygon(2, "e_c_0", BakedPolygonKind.SidewalkSegment, c),
        };
    }

    private static int Components(IReadOnlyList<BakedPolygon> polys, IReadOnlyList<PedConnection>? conns) =>
        new SumoNavMesh(polys, new SumoWalkableSpace(polys), conns).ConnectedComponentCount();

    [Fact]
    public void GeometryAlone_LeavesTheThreePartyCorner_Fragmented()
    {
        // No declared connections: the 3-party corner is skipped -> three separate components.
        Assert.Equal(3, Components(ThreeCornerPolys(), null));
    }

    [Fact]
    public void NetConnections_StitchTheCornerIntoOneComponent()
    {
        var conns = new[] { new PedConnection("e_a_0", "e_b_0"), new PedConnection("e_b_0", "e_c_0") };
        Assert.Equal(1, Components(ThreeCornerPolys(), conns));
    }

    [Fact]
    public void PartialConnections_MergeOnlyTheDeclaredPair()
    {
        // Only a<->b declared -> {a,b} + {c} = two components (c stays isolated: no shortcut invented).
        var conns = new[] { new PedConnection("e_a_0", "e_b_0") };
        Assert.Equal(2, Components(ThreeCornerPolys(), conns));
    }

    [Fact]
    public void DistantDeclaredConnection_IsRefusedByTheFarGuard()
    {
        // A 4th polygon far away, declared connected to a -- but geometrically distant, so the size-relative
        // far guard refuses to bridge across space (a data smell, not a real abutment).
        var polys = new List<BakedPolygon>(ThreeCornerPolys())
        {
            new BakedPolygon(3, "e_far_0", BakedPolygonKind.SidewalkSegment,
                new[] { new Vec2(100, 100), new Vec2(102, 100), new Vec2(102, 102) }),
        };
        var conns = new[] { new PedConnection("e_a_0", "e_far_0") };
        // a,b,c still 3 (no a<->b/c conn here) and e_far stays its own -> 4 components (nothing bridged).
        Assert.Equal(4, Components(polys, conns));
    }

    [Fact]
    public void UnknownConnectionIds_AreIgnored()
    {
        var conns = new[] { new PedConnection("e_a_0", "e_nonexistent_0") };
        Assert.Equal(3, Components(ThreeCornerPolys(), conns)); // inert
    }
}
