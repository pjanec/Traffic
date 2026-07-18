using Sim.Core.Orca;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;
using Xunit;

namespace Sim.Pedestrians.Tests.Navigation;

// P2-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §4/§6): unit-level coverage of
// BlockerRegistry in isolation (stable ids, overlap-area thresholding, order-independence) using
// small hand-built polygons for precise control -- DynamicBlockerRerouteTests.cs covers the
// registry wired into a full RerouteDriver scenario over the real POC-0 fixture.
public class BlockerRegistryTests
{
    // A single 10x10 walkable square, (0,0)-(10,0)-(10,10)-(0,10) -- CCW, area 100.
    private static BakedPolygon Square(int index) => new(
        index,
        $"square{index}",
        BakedPolygonKind.WalkablePolygon,
        new Vec2[] { new(0, 0), new(10, 0), new(10, 10), new(0, 10) });

    [Fact]
    public void Register_BoxSubstantiallyOverlappingPolygon_MarksItBlocked()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons);

        Assert.Empty(registry.BlockedPolygons());

        // A 4x4 box centered inside the square: area 16, well over the default 0.05 threshold.
        var id = registry.RegisterBox(new Vec2(5, 5), 2.0, 2.0, angleRadians: 0.0);

        Assert.True(id.IsValid);
        Assert.Contains(0, registry.BlockedPolygons());
        Assert.Equal(1, registry.Count);
    }

    [Fact]
    public void Unregister_ClearsTheBlock()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons);
        var id = registry.RegisterBox(new Vec2(5, 5), 2.0, 2.0, angleRadians: 0.0);
        Assert.Contains(0, registry.BlockedPolygons());

        Assert.True(registry.Unregister(id));

        Assert.Empty(registry.BlockedPolygons());
        Assert.Equal(0, registry.Count);
        Assert.False(registry.Contains(id));
    }

    [Fact]
    public void Unregister_UnknownOrAlreadyRemovedId_IsInertAndReturnsFalse()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons);
        var id = registry.RegisterBox(new Vec2(5, 5), 2.0, 2.0, angleRadians: 0.0);

        Assert.True(registry.Unregister(id));
        Assert.False(registry.Unregister(id)); // already gone: inert, not an error
        Assert.False(registry.Unregister(new BlockerId(999_999))); // never existed: inert
    }

    [Fact]
    public void TinyCornerOverlap_BelowThreshold_DoesNotBlock()
    {
        var polygons = new[] { Square(0) };
        // A generous 1.0 m^2 threshold so a small grazing sliver at the corner doesn't count.
        var registry = new BlockerRegistry(polygons, overlapAreaThreshold: 1.0);

        // A 0.2 x 0.2 box (area 0.04) straddling the square's corner (9.9,9.9): overlap with the
        // square is at most 0.01 m^2 -- far under the 1.0 threshold.
        registry.RegisterBox(new Vec2(9.9, 9.9), 0.1, 0.1, angleRadians: 0.0);

        Assert.Empty(registry.BlockedPolygons());
    }

    [Fact]
    public void OverlapAboveThreshold_Blocks_BelowThreshold_DoesNot()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons, overlapAreaThreshold: 5.0);

        // A 2x2 box (area 4) fully inside the square: overlap = 4 < 5 -> not blocked.
        var small = registry.RegisterBox(new Vec2(5, 5), 1.0, 1.0, angleRadians: 0.0);
        Assert.Empty(registry.BlockedPolygons());

        registry.Unregister(small);

        // A 4x4 box (area 16) fully inside the square: overlap = 16 > 5 -> blocked.
        registry.RegisterBox(new Vec2(5, 5), 2.0, 2.0, angleRadians: 0.0);
        Assert.Contains(0, registry.BlockedPolygons());
    }

    [Fact]
    public void RegisterArbitraryConvexPolygon_NotJustABox_AlsoBlocks()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons);

        // A CCW triangle straddling most of the square's interior.
        var triangle = new Vec2[] { new(2, 2), new(8, 2), new(5, 8) };
        registry.Register(triangle);

        Assert.Contains(0, registry.BlockedPolygons());
    }

    [Fact]
    public void Ids_AreStableAndNeverReused()
    {
        var polygons = new[] { Square(0) };
        var registry = new BlockerRegistry(polygons);

        var a = registry.RegisterBox(new Vec2(1, 1), 0.5, 0.5, 0.0);
        var b = registry.RegisterBox(new Vec2(2, 2), 0.5, 0.5, 0.0);
        registry.Unregister(a);
        var c = registry.RegisterBox(new Vec2(3, 3), 0.5, 0.5, 0.0);

        Assert.NotEqual(a, b);
        Assert.NotEqual(a, c);
        Assert.NotEqual(b, c);
        Assert.False(registry.Contains(a));
        Assert.True(registry.Contains(b));
        Assert.True(registry.Contains(c));

        var snapshotIds = registry.Snapshot().Select(s => s.Id).ToList();
        Assert.Equal(new[] { b, c }, snapshotIds); // ascending id order
    }

    [Fact]
    public void MultipleBlockers_UnionOfBlockedPolygons_IsOrderIndependent()
    {
        var polygons = new[]
        {
            Square(0), // (0,0)-(10,10)
            new BakedPolygon(1, "square1", BakedPolygonKind.WalkablePolygon,
                new Vec2[] { new(20, 0), new(30, 0), new(30, 10), new(20, 10) }),
        };

        var registryAB = new BlockerRegistry(polygons);
        var registryBA = new BlockerRegistry(polygons);

        // Register in one order for the first registry, the reverse order for the second.
        var a1 = registryAB.RegisterBox(new Vec2(5, 5), 2.0, 2.0, 0.0);
        var b1 = registryAB.RegisterBox(new Vec2(25, 5), 2.0, 2.0, 0.0);

        var b2 = registryBA.RegisterBox(new Vec2(25, 5), 2.0, 2.0, 0.0);
        var a2 = registryBA.RegisterBox(new Vec2(5, 5), 2.0, 2.0, 0.0);

        Assert.True(registryAB.BlockedPolygons().SetEquals(registryBA.BlockedPolygons()));
        Assert.Contains(0, registryAB.BlockedPolygons());
        Assert.Contains(1, registryAB.BlockedPolygons());
    }
}
