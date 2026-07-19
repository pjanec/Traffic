using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Demand;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Demand;

// P2-3 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §4/§10; docs/PEDESTRIAN-NAVMESH-CONTRACT.md):
// pedestrian origin->destination demand -- a scenario that populates and sustains itself instead of a
// test hand-registering one ped at a time. Reuses the same POC-0 fixture net + SUMO-bake navmesh
// (PedNetworkParser / WalkablePolygonBaker / SumoNavMesh) as PedLodManagerTests/SumoBakeNavigationTests,
// so PedDemand is exercised against a REAL, already-proven IPedNavigation, not a stub.
public class PedDemandTests
{
    private readonly ITestOutputHelper _output;

    public PedDemandTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double MaxSpeed = 1.4;    // m/s
    private const double Radius = 0.3;      // m
    private const double ArriveRadius = 0.3; // m (PedLodManager waypoint-arrival radius)
    private const double ArrivalRadius = 0.5; // m (PedDemand OD-arrival radius)
    private const double Dt = 0.1;          // s
    private const double DwellSeconds = 0.5; // s

    // Same POC-0 junction points PedLodManagerTests/SumoBakeNavigationTests use: routing between them
    // forces a real, multi-waypoint path through the junction's walkingarea/crossing/walkingarea chain.
    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);
    private static readonly double StraightLineDist = (EastNorthArm - WestNorthArm).Abs;

    // Generous slack over straight-line transit time (mirrors the *3.0 convention already used by
    // SumoBakeNavigationTests/PedLodManagerTests for this same fixture's bent-path routes).
    private const double TransitSlackFactor = 5.0;

    private static SumoNavMesh BuildNav()
    {
        var polygons = WalkablePolygonBaker.Bake(LoadPoc0Network());
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static PedNetwork LoadPoc0Network() => PedNetworkParser.Load(
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml"));

    private static PedDemandConfig BuildConfig(double spawnRate, int cap, ulong seed) => new()
    {
        Origins = new[] { WestNorthArm, EastNorthArm },
        Destinations = new[] { WestNorthArm, EastNorthArm },
        SpawnRatePerSecond = spawnRate,
        PopulationCap = cap,
        Seed = seed,
        MaxSpeed = MaxSpeed,
        Radius = Radius,
        ArrivalRadius = ArrivalRadius,
    };

    // ---- Success condition 1: sustained population + real O->D routing & arrival -------------------

    [Fact]
    public void SustainedRun_PopulationRisesToAndStaysNearCap_AndPedsRouteAndArrive()
    {
        var nav = BuildNav();
        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        const int cap = 10;
        var demand = new PedDemand(BuildConfig(spawnRate: 3.0, cap: cap, seed: 12345UL), nav, manager);

        var field = new InterestField(); // no interest sources -> whole run stays low-power (PathArc)
        var noEntities = Array.Empty<WorldDisc>();

        var spawnTimeById = new Dictionary<int, double>();
        var liveHistory = new List<int>();

        var now = 0.0;
        const int steps = 1500; // 150 s of sim time
        for (var i = 0; i < steps; i++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;
            liveHistory.Add(demand.LiveCount);
        }

        foreach (var e in demand.SpawnEvents)
        {
            spawnTimeById[e.Id] = e.Time;
        }

        var peakLive = liveHistory.Max();
        _output.WriteLine($"[P2-3 measured] peak live population: {peakLive} (cap {cap}), spawns={demand.SpawnCount}, arrivals={demand.ArrivalCount}");
        Assert.Equal(cap, peakLive);

        // Over the back half of the run the population should sit AT the cap almost always -- allow a
        // small tolerance for the one-step gap between an arrival freeing a slot and the next Step's
        // spawn refilling it (see PedDemand.SpawnDue remarks).
        var backHalf = liveHistory.Skip(steps / 2).ToList();
        var meanBackHalf = backHalf.Average();
        _output.WriteLine($"[P2-3 measured] mean live population over back half: {meanBackHalf:F2} (cap {cap})");
        Assert.True(meanBackHalf >= cap * 0.85, $"expected sustained population near cap, got mean {meanBackHalf:F2} vs cap {cap}");

        Assert.True(demand.ArrivalCount > 0, "expected at least one ped to actually arrive");
        Assert.Equal(0, demand.UnreachableSkipCount);

        // Every arrival happened within a generous nav-distance/speed slack of its own spawn time --
        // i.e. peds actually walked their route, not teleported or stalled forever.
        foreach (var arrival in demand.ArrivalEvents)
        {
            Assert.True(spawnTimeById.TryGetValue(arrival.Id, out var spawnedAt), $"ped {arrival.Id} arrived without a recorded spawn");
            var transit = arrival.Time - spawnedAt;
            var maxAllowed = (StraightLineDist / MaxSpeed) * TransitSlackFactor;
            Assert.True(transit >= 0, $"ped {arrival.Id} arrived before it spawned ({transit:F2}s)");
            Assert.True(transit <= maxAllowed, $"ped {arrival.Id} took {transit:F2}s to arrive, exceeding slack bound {maxAllowed:F2}s");
        }

        // Population accounting invariant: nobody vanished or duplicated.
        Assert.Equal(demand.SpawnCount - demand.ArrivalCount, demand.LiveCount);
    }

    // ---- Success condition 2: spawn/despawn hygiene -- no unbounded growth, no crowd-slot leak -------

    [Fact]
    public void SustainedRun_WithPromotion_DoesNotLeakOrcaCrowdSlots()
    {
        var nav = BuildNav();
        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        const int cap = 6;
        var demand = new PedDemand(BuildConfig(spawnRate: 2.0, cap: cap, seed: 999UL), nav, manager);

        // A single static interest source spanning the whole corridor's midpoint with a promote
        // radius covering the entire West<->East span: every spawned ped is promoted to high-power
        // (FreeKinematic) for at least part of its transit, so arrivals routinely happen WHILE
        // high-power -- exactly the case that would leak an OrcaCrowd slot if RemovePed forgot to
        // release the handle.
        var midpoint = new Vec2((WestNorthArm.X + EastNorthArm.X) / 2.0, WestNorthArm.Y);
        var field = new InterestField();
        field.Register(new InterestSource(midpoint, promoteRadius: StraightLineDist, demoteRadius: StraightLineDist * 1.5));
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;
        const int steps = 3000; // 300 s of sim time -- many spawn/arrive cycles at cap=6
        for (var i = 0; i < steps; i++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        _output.WriteLine(
            $"[P2-3 measured] spawns={demand.SpawnCount}, arrivals={demand.ArrivalCount}, live={demand.LiveCount}, "
            + $"HighPowerCount(live)={manager.HighPowerCount}, HighCrowdSlotHighWater={manager.HighCrowdSlotHighWater}");

        // The total number of trips over a 300s run at cap=6 should vastly exceed the cap -- otherwise
        // this test isn't actually exercising churn.
        Assert.True(demand.SpawnCount > cap * 5, $"expected substantial churn (>{cap * 5} spawns), got {demand.SpawnCount}");
        Assert.True(demand.ArrivalCount > cap * 3, $"expected substantial churn (>{cap * 3} arrivals), got {demand.ArrivalCount}");

        // The leak check: if RemovePed failed to release a high-power ped's OrcaCrowd handle on
        // despawn, every arrival-while-high-power would permanently burn a NEW slot (OrcaCrowd never
        // reuses a handle nobody freed), so the high-water mark would grow roughly linearly with
        // SpawnCount. With a correct Remove, the high-water mark is bounded by the PEAK concurrent
        // high-power occupancy (at most `cap`, plus a small slack for slots vacated-and-not-yet-
        // recycled at the moment of measurement) -- nowhere near SpawnCount.
        Assert.True(manager.HighCrowdSlotHighWater <= cap + 4,
            $"HighCrowdSlotHighWater ({manager.HighCrowdSlotHighWater}) grew far past the population cap ({cap}) "
            + $"across {demand.SpawnCount} spawns -- suspected OrcaCrowd slot leak on despawn");

        // Live occupancy is never inflated either -- HighPowerCount (the live, not high-water, count)
        // never exceeds the current live population.
        Assert.True(manager.HighPowerCount <= demand.LiveCount,
            $"HighPowerCount ({manager.HighPowerCount}) exceeded live ped count ({demand.LiveCount})");

        // No unbounded growth of PedDemand's own bookkeeping either.
        Assert.Equal(demand.SpawnCount - demand.ArrivalCount, demand.LiveCount);
    }

    // ---- Success condition 3: determinism -----------------------------------------------------------

    [Fact]
    public void SustainedRun_IsDeterministic_AcrossIndependentRuns()
    {
        var (trajectory1, spawns1, arrivals1) = RunFullDemandScenario();
        var (trajectory2, spawns2, arrivals2) = RunFullDemandScenario();

        Assert.Equal(spawns1.Count, spawns2.Count);
        for (var i = 0; i < spawns1.Count; i++)
        {
            Assert.Equal(spawns1[i].Id, spawns2[i].Id);
            Assert.Equal(spawns1[i].Time, spawns2[i].Time, precision: 12);
            Assert.Equal(spawns1[i].Origin.X, spawns2[i].Origin.X, precision: 12);
            Assert.Equal(spawns1[i].Origin.Y, spawns2[i].Origin.Y, precision: 12);
            Assert.Equal(spawns1[i].Destination.X, spawns2[i].Destination.X, precision: 12);
            Assert.Equal(spawns1[i].Destination.Y, spawns2[i].Destination.Y, precision: 12);
        }

        Assert.Equal(arrivals1.Count, arrivals2.Count);
        for (var i = 0; i < arrivals1.Count; i++)
        {
            Assert.Equal(arrivals1[i].Id, arrivals2[i].Id);
            Assert.Equal(arrivals1[i].Time, arrivals2[i].Time, precision: 12);
        }

        Assert.True(trajectory1.Count > 0);
        Assert.Equal(trajectory1.Count, trajectory2.Count);
        for (var i = 0; i < trajectory1.Count; i++)
        {
            Assert.Equal(trajectory1[i].Id, trajectory2[i].Id);
            Assert.Equal(trajectory1[i].Time, trajectory2[i].Time, precision: 12);
            Assert.Equal(trajectory1[i].Pos.X, trajectory2[i].Pos.X, precision: 12);
            Assert.Equal(trajectory1[i].Pos.Y, trajectory2[i].Pos.Y, precision: 12);
        }

        _output.WriteLine(
            $"[P2-3 measured] determinism gate: {spawns1.Count} spawns, {arrivals1.Count} arrivals, "
            + $"{trajectory1.Count} trajectory samples, all bit-identical across two independent runs");
    }

    private static (List<(int Id, double Time, Vec2 Pos)> Trajectory, IReadOnlyList<PedSpawnEvent> Spawns, IReadOnlyList<PedArrivalEvent> Arrivals)
        RunFullDemandScenario()
    {
        var nav = BuildNav();
        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        const int cap = 5;
        var demand = new PedDemand(BuildConfig(spawnRate: 2.5, cap: cap, seed: 424242UL), nav, manager);

        var midpoint = new Vec2((WestNorthArm.X + EastNorthArm.X) / 2.0, WestNorthArm.Y);
        var field = new InterestField();
        field.Register(new InterestSource(midpoint, promoteRadius: StraightLineDist * 0.5, demoteRadius: StraightLineDist));
        var noEntities = Array.Empty<WorldDisc>();

        var trajectory = new List<(int, double, Vec2)>();
        var now = 0.0;
        for (var i = 0; i < 800; i++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;

            foreach (var id in demand.LiveIds)
            {
                trajectory.Add((id, now, manager.PositionOf(id, now)));
            }
        }

        return (trajectory, new List<PedSpawnEvent>(demand.SpawnEvents), new List<PedArrivalEvent>(demand.ArrivalEvents));
    }
}
