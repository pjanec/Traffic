using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;

namespace Sim.Pedestrians.Tests;

// P1-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §10): drives every scenario through the
// PedestrianWorld FACADE ONLY -- no PedLodManager/InterestField/OrcaCrowd internals referenced below
// this file, mirroring the facade's own contract ("a consumer never hand-wires the internals").
// Reuses the same committed POC-0 fixture net + navigation pieces PedLodManagerTests uses.
public class PedestrianWorldTests
{
    private const double MaxSpeed = 1.4;   // m/s
    private const double Radius = 0.3;     // m
    private const double Dt = 0.1;         // s
    private const double DwellSeconds = 1.0; // s
    private const double PromoteRadius = 3.0; // m
    private const double DemoteRadius = 6.0;  // m

    // Same POC-0 junction points PedLodManagerTests/SumoBakeNavigationTests use.
    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);

    private static SumoNavMesh BuildNav()
    {
        var polygons = WalkablePolygonBaker.Bake(LoadPoc0Network());
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static PedNetwork LoadPoc0Network() => PedNetworkParser.Load(
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml"));

    // A navigation provider that never finds a path -- used only to exercise AddWalker's "false,
    // no throw" unroutable contract; a legitimate implementation of the public IPedNavigation seam,
    // not a peek at any internal.
    private sealed class NullNavigation : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    [Fact]
    public void AddWalker_RoutesOverNavMesh_TracksLiveIds_AndArrivesAtDestination()
    {
        var world = new PedestrianWorld(BuildNav(), arriveRadius: 0.3, dwellSeconds: DwellSeconds);

        Assert.Empty(world.LiveIds);

        var added = world.AddWalker(id: 1, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0);
        Assert.True(added);
        Assert.Contains(1, world.LiveIds);
        Assert.Equal(PedDrModel.PathArc, world.ModelOf(1));

        var startPos = world.PositionOf(1, 0.0);

        // The routed path bends through the junction (south, across, then back north) rather than
        // running straight toward the destination, so straight-line distance-to-goal is not monotone
        // early on -- only ARC-LENGTH progress along the path is. A few steps in, the walker must at
        // least have left its start position.
        var now = 0.0;
        for (var i = 0; i < 10; i++)
        {
            world.Step(now, Dt);
            now += Dt;
        }

        Assert.True((world.PositionOf(1, now) - startPos).Abs > 0.1, "walker should have moved from its start position");

        // Same generous slack convention PedLodManagerTests uses (straight-line time * 3) -- comfortably
        // enough for the actual (longer, junction-routed) path to complete and arrive.
        var straightLineDist = (EastNorthArm - WestNorthArm).Abs;
        var totalSteps = (int)((straightLineDist / MaxSpeed) * 3.0 / Dt);
        for (var i = 0; i < totalSteps; i++)
        {
            world.Step(now, Dt);
            now += Dt;
        }

        var finalDist = (world.PositionOf(1, now) - EastNorthArm).Abs;
        Assert.True(finalDist < 0.5, $"walker should have arrived at its destination, final distance {finalDist:F3} m");

        world.Remove(1);
        Assert.DoesNotContain(1, world.LiveIds);
    }

    [Fact]
    public void AddWalker_ReturnsFalse_AndDoesNotRegister_WhenUnroutable()
    {
        var world = new PedestrianWorld(new NullNavigation());

        var added = world.AddWalker(id: 1, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0);

        Assert.False(added);
        Assert.DoesNotContain(1, world.LiveIds);
    }

    [Fact]
    public void MovingInterestSource_PromotesNearbyWalker_ToFreeKinematic()
    {
        var world = new PedestrianWorld(BuildNav(), arriveRadius: 0.3, dwellSeconds: DwellSeconds);
        Assert.True(world.AddWalker(id: 1, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0));

        // A moving interest source (an avatar/camera bubble) that chases the walker, exactly like a
        // real caller would move a source every step (docs/PEDESTRIAN-DESIGN.md §5 use case 1).
        var source = world.AddInterestSource(WestNorthArm, PromoteRadius, DemoteRadius, InterestSourceKind.EntityAttached);

        Assert.Equal(0, world.HighPowerCount);

        var everPromoted = false;
        var now = 0.0;
        for (var i = 0; i < 100 && !everPromoted; i++)
        {
            world.Step(now, Dt);
            now += Dt;

            if (world.ModelOf(1) == PedDrModel.FreeKinematic)
            {
                everPromoted = true;
            }

            world.MoveInterestSource(source, world.PositionOf(1, now) + new Vec2(1.0, 0.0));
        }

        Assert.True(everPromoted, "walker never promoted despite a chasing interest source");
        Assert.Equal(PedDrModel.FreeKinematic, world.ModelOf(1));
        Assert.True(world.HighPowerCount > 0);
    }

    [Fact]
    public void RemoveInterestSource_StopsItFromDrivingFurtherPromotions()
    {
        var world = new PedestrianWorld(BuildNav(), arriveRadius: 0.3, dwellSeconds: DwellSeconds);
        Assert.True(world.AddWalker(id: 1, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0));
        Assert.True(world.AddWalker(id: 2, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0));

        var source = world.AddInterestSource(WestNorthArm, PromoteRadius, DemoteRadius, InterestSourceKind.StaticAoI);
        world.RemoveInterestSource(source);

        var now = 0.0;
        for (var i = 0; i < 50; i++)
        {
            world.Step(now, Dt);
            now += Dt;
        }

        // With the source removed before any Step ever ran, neither walker had anything to promote it.
        Assert.Equal(PedDrModel.PathArc, world.ModelOf(1));
        Assert.Equal(PedDrModel.PathArc, world.ModelOf(2));
        Assert.Equal(0, world.HighPowerCount);
    }

    [Fact]
    public void SetForcedHighPower_PromotesWithNoInterestSourceNearby_HoldsHigh_ThenDemotesWhenCleared()
    {
        var world = new PedestrianWorld(BuildNav(), arriveRadius: 0.3, dwellSeconds: DwellSeconds);
        Assert.True(world.AddWalker(id: 1, WestNorthArm, EastNorthArm, MaxSpeed, Radius, now: 0.0));

        // An interest source parked far away -- nothing would ever promote this walker by proximity.
        var source = world.AddInterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0);

        world.SetForcedHighPower(1, true);
        var now = 0.0;
        world.Step(now, Dt);
        now += Dt;

        Assert.Equal(PedDrModel.FreeKinematic, world.ModelOf(1));
        Assert.True(world.HighPowerCount > 0);

        // Stays high across many steps while pinned, still with no source nearby.
        for (var i = 0; i < 100; i++)
        {
            world.Step(now, Dt);
            now += Dt;
        }

        Assert.Equal(PedDrModel.FreeKinematic, world.ModelOf(1));

        // Unpin -> demotes back to low-power once past the dwell/hysteresis window.
        world.SetForcedHighPower(1, false);
        var demoted = false;
        for (var i = 0; i < 100 && !demoted; i++)
        {
            world.Step(now, Dt);
            now += Dt;
            if (world.ModelOf(1) == PedDrModel.PathArc)
            {
                demoted = true;
            }
        }

        Assert.True(demoted, "an unpinned walker far from every source must demote back to low-power");
        Assert.Equal(0, world.HighPowerCount);

        // Source registration itself is inert here -- it was never near the walker.
        world.RemoveInterestSource(source);
    }

    [Fact]
    public void Sample_RunsThroughFacadeOnly_DemonstratingPromotionArrivalAndForcedHighPower()
    {
        var result = PedestrianWorldSample.Run();

        Assert.True(result.Walker1EverPromoted, "sample walker 1 never promoted under the chasing interest source");
        Assert.True(result.Walker2Arrived, "sample walker 2 never reached its destination");
        Assert.True(result.PinnedPedHadNoNearbySource, "sample pinned walker was not actually free of nearby sources");
        Assert.True(result.PinnedPedIsHighPower, "sample pinned walker did not hold high-power under SetForcedHighPower");
        Assert.True(result.HighPowerCountAtEnd > 0);
        Assert.True(result.StepsRun > 0);
    }
}
