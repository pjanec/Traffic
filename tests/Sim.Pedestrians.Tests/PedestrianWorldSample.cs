using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians.Tests;

// P1-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §10): the "sample" success condition --
// a short scenario driven through the PedestrianWorld FACADE ONLY (no PedLodManager/InterestField/
// OrcaCrowd internals touched below Build/construction). Chose the lighter of the two allowed options
// (a static method + a test that runs it) over a new samples/ console project: it needs no new
// project reference, no .sln entry, and no duplicated POC-0 fixture copy -- it reuses the Fixtures/
// net.xml this test project already commits (PedLodManagerTests' BuildNav convention).
//
// Constructing the navigation provider (SumoNavMesh, over the baked POC-0 walkable polygons) is not an
// "internal" in the sense the task means -- it is the IPedNavigation seam PedestrianWorld's constructor
// takes as a parameter, exactly like a real host would construct its own navmesh once and hand it in.
public static class PedestrianWorldSample
{
    public readonly record struct Result(
        int StepsRun,
        bool Walker1EverPromoted,
        bool Walker2Arrived,
        int HighPowerCountAtEnd,
        bool PinnedPedIsHighPower,
        bool PinnedPedHadNoNearbySource);

    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);

    public static Result Run()
    {
        // 1) Build a real navmesh from the committed POC-0 SUMO-geometry fixture and hand it to the
        //    facade -- everything from here on goes through PedestrianWorld only.
        var polygons = WalkablePolygonBaker.Bake(
            PedNetworkParser.Load(
                Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
                Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml")));
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        var world = new PedestrianWorld(nav, arriveRadius: 0.3, dwellSeconds: 1.0);

        const double maxSpeed = 1.4;
        const double radius = 0.3;
        const double dt = 0.1;

        // 2) A few walkers, routed origin -> destination by the facade itself.
        var walker1Added = world.AddWalker(id: 1, WestNorthArm, EastNorthArm, maxSpeed, radius, now: 0.0);
        var walker2Added = world.AddWalker(id: 2, EastNorthArm, WestNorthArm, maxSpeed, radius, now: 0.0);
        if (!walker1Added || !walker2Added)
        {
            throw new InvalidOperationException("sample setup expected both walkers to route successfully");
        }

        // 3) A moving interest source -- an avatar/camera bubble that chases walker 1, promoting it.
        var source = world.AddInterestSource(WestNorthArm, promoteRadius: 3.0, demoteRadius: 6.0, InterestSourceKind.EntityAttached);

        // Generous slack (PedLodManagerTests uses a 3x factor over straight-line time for the same
        // junction-routed path; this sample uses 6x for extra margin since it also has to leave time
        // for walker 2's full arrival, not just a promotion event).
        var straightLineDist = (EastNorthArm - WestNorthArm).Abs;
        var steps = (int)((straightLineDist / maxSpeed) * 6.0 / dt);

        var everPromoted = false;
        var now = 0.0;
        for (var i = 0; i < steps; i++)
        {
            world.Step(now, dt);
            now += dt;

            if (world.ModelOf(1) == PedDrModel.FreeKinematic)
            {
                everPromoted = true;
            }

            // Chase walker 1 so the source stays a live promotion stimulus.
            world.MoveInterestSource(source, world.PositionOf(1, now) + new Vec2(1.0, 0.0));
        }

        var walker2Arrived = (world.PositionOf(2, now) - WestNorthArm).Abs < 0.5;

        // 4) Move the interest source far away, THEN add ped 3 -- it is added fresh, after the source
        //    has left, so it has never been within any source's radius before we pin it. SetForcedHighPower
        //    then promotes it purely from the pin, with no interest source anywhere near it.
        var farAway = new Vec2(-10_000, -10_000);
        world.MoveInterestSource(source, farAway);
        world.Step(now, dt);
        now += dt;

        var pinnedAdded = world.AddWalker(id: 3, WestNorthArm, EastNorthArm, maxSpeed, radius, now);
        if (!pinnedAdded)
        {
            throw new InvalidOperationException("sample setup expected the pinned walker to route successfully");
        }

        // Well outside even the largest demote radius used above (6.0 m) -- confirms the pin below
        // cannot be attributed to ambient interest-field proximity.
        var pinnedHadNoNearbySource = (world.PositionOf(3, now) - farAway).Abs > 100.0;

        world.SetForcedHighPower(3, true);
        world.Step(now, dt);
        now += dt;

        var pinnedIsHighPower = world.ModelOf(3) == PedDrModel.FreeKinematic;
        var totalSteps = steps + 1;

        // Hold across several more steps to demonstrate the pin (never demotes while pinned).
        for (var i = 0; i < 20; i++)
        {
            world.Step(now, dt);
            now += dt;
            totalSteps++;
        }

        pinnedIsHighPower &= world.ModelOf(3) == PedDrModel.FreeKinematic;

        return new Result(
            StepsRun: totalSteps,
            Walker1EverPromoted: everPromoted,
            Walker2Arrived: walker2Arrived,
            HighPowerCountAtEnd: world.HighPowerCount,
            PinnedPedIsHighPower: pinnedIsHighPower,
            PinnedPedHadNoNearbySource: pinnedHadNoNearbySource);
    }
}
