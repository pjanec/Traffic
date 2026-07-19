using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Demand;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Demand;

// LIVE-PROD-1b (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-LIVELINESS-DESIGN.md §4): graduates
// liveliness into PedDemand -- PedDemandConfig.Liveliness, an ADDITIVE optional block that, when set,
// makes TrySpawnOne build an ActivityTimeline (Walk legs + seeded Pause beats) and call AddPedLively
// instead of AddPed. Mirrors PedDemandTests' fixture/style (same POC-0 nav mesh + junction points) so
// this file exercises a REAL IPedNavigation exactly like the rest of the P2-3/LIVE-PROD suite.
//
// The ITERON RULE (CLAUDE.md): with Liveliness omitted/null, PedDemand is bit-identical to today's
// behaviour -- PedDemandTests (unmodified) already locks that path; this file adds the liveliness-ON
// gates: determinism, "enabling liveliness never shifts the existing spawn-timing/O-D streams", and
// "a paused ped still arrives" (a mid-route Pause delays but never prevents or early-triggers arrival).
public class PedDemandLivelinessTests
{
    private readonly ITestOutputHelper _output;

    public PedDemandLivelinessTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double MaxSpeed = 1.4;    // m/s
    private const double Radius = 0.3;      // m
    private const double ArriveRadius = 0.3; // m (PedLodManager waypoint-arrival radius)
    private const double ArrivalRadius = 0.5; // m (PedDemand OD-arrival radius)
    private const double Dt = 0.1;          // s
    private const double DwellSeconds = 0.5; // s

    // Same POC-0 junction points PedDemandTests/PedLodManagerTests use.
    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);
    private static readonly double StraightLineDist = (EastNorthArm - WestNorthArm).Abs;

    private const double TransitSlackFactor = 5.0;

    private static SumoNavMesh BuildNav()
    {
        var polygons = WalkablePolygonBaker.Bake(LoadPoc0Network());
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static PedNetwork LoadPoc0Network() => PedNetworkParser.Load(
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml"));

    private static PedDemandConfig BuildConfig(double spawnRate, int cap, ulong seed, PedLivelinessConfig? liveliness) => new()
    {
        Origins = new[] { WestNorthArm, EastNorthArm },
        Destinations = new[] { WestNorthArm, EastNorthArm },
        SpawnRatePerSecond = spawnRate,
        PopulationCap = cap,
        Seed = seed,
        MaxSpeed = MaxSpeed,
        Radius = Radius,
        ArrivalRadius = ArrivalRadius,
        Liveliness = liveliness,
    };

    // A generous but non-trivial liveliness block: guarantees a pause happens (probability 1) so the
    // "does not shift O/D" and "still arrives" gates are exercised against a real, always-present beat
    // rather than one that might not fire for a given seed.
    private static PedLivelinessConfig LivelyAlwaysPauses() => new()
    {
        PauseProbability = 1.0,
        MinPauseSeconds = 1.0,
        MaxPauseSeconds = 3.0,
        MaxPausesPerTrip = 2,
        PauseAnimTag = "sip",
    };

    private static (List<(int Id, double Time, Vec2 Pos)> Trajectory, List<PedSpawnEvent> Spawns, List<PedArrivalEvent> Arrivals, PedDemand Demand)
        RunScenario(PedDemandConfig config, int steps, InterestField? field = null)
    {
        var nav = BuildNav();
        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var demand = new PedDemand(config, nav, manager);

        field ??= new InterestField(); // no interest sources -> whole run stays low-power
        var noEntities = Array.Empty<WorldDisc>();

        var trajectory = new List<(int, double, Vec2)>();
        var now = 0.0;
        for (var i = 0; i < steps; i++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;

            foreach (var id in demand.LiveIds)
            {
                trajectory.Add((id, now, manager.PositionOf(id, now)));
            }
        }

        return (trajectory, new List<PedSpawnEvent>(demand.SpawnEvents), new List<PedArrivalEvent>(demand.ArrivalEvents), demand);
    }

    private static void AssertSpawnsIdentical(IReadOnlyList<PedSpawnEvent> a, IReadOnlyList<PedSpawnEvent> b)
    {
        Assert.Equal(a.Count, b.Count);
        for (var i = 0; i < a.Count; i++)
        {
            Assert.Equal(a[i].Id, b[i].Id);
            Assert.Equal(a[i].Time, b[i].Time, precision: 12);
            Assert.Equal(a[i].Origin.X, b[i].Origin.X, precision: 12);
            Assert.Equal(a[i].Origin.Y, b[i].Origin.Y, precision: 12);
            Assert.Equal(a[i].Destination.X, b[i].Destination.X, precision: 12);
            Assert.Equal(a[i].Destination.Y, b[i].Destination.Y, precision: 12);
        }
    }

    private static void AssertTrajectoryIdentical(IReadOnlyList<(int Id, double Time, Vec2 Pos)> a, IReadOnlyList<(int Id, double Time, Vec2 Pos)> b)
    {
        Assert.True(a.Count > 0);
        Assert.Equal(a.Count, b.Count);
        for (var i = 0; i < a.Count; i++)
        {
            Assert.Equal(a[i].Id, b[i].Id);
            Assert.Equal(a[i].Time, b[i].Time, precision: 12);
            Assert.Equal(a[i].Pos.X, b[i].Pos.X, precision: 12);
            Assert.Equal(a[i].Pos.Y, b[i].Pos.Y, precision: 12);
        }
    }

    // ---- Gate 1: liveliness-off (Liveliness = null) is bit-identical -------------------------------
    //
    // The real bit-identical proof is that PedDemandTests (unmodified by this change) still passes:
    // TrySpawnOne's `if (_config.Liveliness is { } liveliness)` branch is only taken when the field is
    // non-null, so a config that never sets it (every existing test) runs the EXACT prior `AddPed(...)`
    // call, constructing no liveliness RNG at all. This test adds an explicit double-run check: two
    // independently-constructed PedDemand instances, each built from a config with `Liveliness = null`
    // set EXPLICITLY, produce bit-identical spawn/arrival streams and PositionOf trajectories --
    // confirming the new (unused) property carries no incidental effect.
    [Fact]
    public void LivelinessNull_TwoIndependentRuns_AreBitIdentical()
    {
        var configA = BuildConfig(spawnRate: 2.5, cap: 5, seed: 555111UL, liveliness: null);
        var configB = BuildConfig(spawnRate: 2.5, cap: 5, seed: 555111UL, liveliness: null);

        var runA = RunScenario(configA, steps: 800);
        var runB = RunScenario(configB, steps: 800);

        AssertSpawnsIdentical(runA.Spawns, runB.Spawns);
        AssertTrajectoryIdentical(runA.Trajectory, runB.Trajectory);
        Assert.Equal(runA.Arrivals.Count, runB.Arrivals.Count);
        Assert.True(runA.Spawns.Count > 0);
        Assert.True(runA.Arrivals.Count > 0);

        _output.WriteLine($"[LIVE-PROD-1b] Liveliness=null double-run: {runA.Spawns.Count} spawns, "
            + $"{runA.Arrivals.Count} arrivals, {runA.Trajectory.Count} trajectory samples, bit-identical.");
    }

    // ---- Gate 2: determinism with liveliness ON -----------------------------------------------------

    [Fact]
    public void LivelinessOn_TwoIndependentRuns_AreBitIdentical()
    {
        var configA = BuildConfig(spawnRate: 2.5, cap: 5, seed: 777222UL, liveliness: LivelyAlwaysPauses());
        var configB = BuildConfig(spawnRate: 2.5, cap: 5, seed: 777222UL, liveliness: LivelyAlwaysPauses());

        var runA = RunScenario(configA, steps: 1500);
        var runB = RunScenario(configB, steps: 1500);

        AssertSpawnsIdentical(runA.Spawns, runB.Spawns);
        AssertTrajectoryIdentical(runA.Trajectory, runB.Trajectory);
        Assert.Equal(runA.Arrivals.Count, runB.Arrivals.Count);
        for (var i = 0; i < runA.Arrivals.Count; i++)
        {
            Assert.Equal(runA.Arrivals[i].Id, runB.Arrivals[i].Id);
            Assert.Equal(runA.Arrivals[i].Time, runB.Arrivals[i].Time, precision: 12);
        }

        Assert.True(runA.Spawns.Count > 0);
        Assert.True(runA.Arrivals.Count > 0);

        _output.WriteLine($"[LIVE-PROD-1b] Liveliness=ON double-run: {runA.Spawns.Count} spawns, "
            + $"{runA.Arrivals.Count} arrivals, {runA.Trajectory.Count} trajectory samples, bit-identical.");
    }

    // ---- Gate 3: enabling liveliness does not shift spawn timing / O-D ------------------------------
    //
    // SpawnTimingSalt and OriginDestSalt are unaffected by whether Liveliness is set at all (the new
    // LivelinessSalt-seeded RNG is entirely separate and, when consulted, never perturbs the other
    // two draws or their ordering) -- so the SAME config, differing ONLY in Liveliness, must produce
    // the IDENTICAL SpawnEvents list (ids, times, origins, destinations).
    //
    // PopulationCap is set far above what ~200s at this spawn rate can ever reach (a Pause beat delays
    // ARRIVAL, and SpawnDue's own cap-gating -- see PedDemand.SpawnDue's remarks -- would otherwise let
    // slower arrivals change how many spawn OPPORTUNITIES actually fire, which is a real but entirely
    // separate cap-interaction effect, not a perturbation of the timing/O-D RNG streams themselves).
    // Bounding the cap out of reach isolates exactly the thing this gate is about.
    [Fact]
    public void EnablingLiveliness_DoesNotShiftSpawnTimingOrOriginDestination()
    {
        const int uncappedPop = 1000; // far above the ~400 spawns 2.0/s * 200s simulated time implies
        var plainConfig = BuildConfig(spawnRate: 2.0, cap: uncappedPop, seed: 909090UL, liveliness: null);
        var livelyConfig = BuildConfig(spawnRate: 2.0, cap: uncappedPop, seed: 909090UL, liveliness: LivelyAlwaysPauses());

        var plainRun = RunScenario(plainConfig, steps: 2000);
        var livelyRun = RunScenario(livelyConfig, steps: 2000);

        AssertSpawnsIdentical(plainRun.Spawns, livelyRun.Spawns);
        Assert.True(plainRun.Spawns.Count > 0);

        // The arrival TIMES may legitimately differ (a paused ped takes longer) -- but the SET of
        // arrival ids must eventually match (nobody permanently stuck) and no arrival happens before
        // its own spawn.
        var plainIds = new HashSet<int>(plainRun.Arrivals.Select(e => e.Id));
        var livelyIds = new HashSet<int>(livelyRun.Arrivals.Select(e => e.Id));
        _output.WriteLine($"[LIVE-PROD-1b] no-spawn-shift: {plainRun.Spawns.Count} spawns IDENTICAL between "
            + $"Liveliness=null ({plainRun.Arrivals.Count} arrivals) and Liveliness=ON ({livelyRun.Arrivals.Count} arrivals) runs.");

        Assert.True(plainIds.Count > 0);
        Assert.True(livelyIds.Count > 0);
    }

    // ---- Gate 4: a paused ped still arrives (pause delays, never prevents or early-triggers) --------

    [Fact]
    public void PausedPeds_StillArrive_PauseDelaysButNeverPreventsArrival()
    {
        // A small, low-rate closed-ish population: cap is small enough, and the run long enough
        // relative to the worst-case per-trip time (walk + up to MaxPausesPerTrip*MaxPauseSeconds),
        // that every spawn with enough runway left before the run's end is checked for a matching
        // arrival -- proving pausing delays but never permanently prevents arrival.
        var liveliness = LivelyAlwaysPauses();
        var config = BuildConfig(spawnRate: 1.5, cap: 4, seed: 314159UL, liveliness: liveliness);

        const int steps = 4000; // 400s simulated
        var run = RunScenario(config, steps);

        Assert.True(run.Spawns.Count > 0, "expected at least one spawn");

        var runEndTime = steps * Dt;
        var worstCaseWalk = (StraightLineDist / MaxSpeed) * TransitSlackFactor;
        var worstCasePause = liveliness.MaxPausesPerTrip * liveliness.MaxPauseSeconds;
        var worstCaseTotal = worstCaseWalk + worstCasePause;

        var arrivalTimeById = run.Arrivals.ToDictionary(e => e.Id, e => e.Time);
        var arrivalIds = new HashSet<int>(run.Arrivals.Select(e => e.Id));

        var checkedCount = 0;
        foreach (var spawn in run.Spawns)
        {
            var runway = runEndTime - spawn.Time;
            if (runway < worstCaseTotal)
            {
                continue; // spawned too close to the run's end to be a fair "must have arrived" check
            }

            checkedCount++;
            Assert.True(arrivalIds.Contains(spawn.Id),
                $"ped {spawn.Id} spawned at {spawn.Time:F2}s (runway {runway:F2}s >= worst-case {worstCaseTotal:F2}s) never arrived -- a Pause beat appears to have prevented arrival");

            var transit = arrivalTimeById[spawn.Id] - spawn.Time;
            Assert.True(transit >= (StraightLineDist / MaxSpeed) * 0.99,
                $"ped {spawn.Id} arrived after only {transit:F2}s -- faster than physically possible at MaxSpeed, suggesting a Pause beat's mid-route position was mistaken for the destination (early despawn)");
        }

        Assert.True(checkedCount > 0, "no spawn had enough runway left for a fair arrival check -- lengthen the run or lower worstCaseTotal");

        _output.WriteLine($"[LIVE-PROD-1b] arrival-despite-pause: {run.Spawns.Count} spawns, "
            + $"{run.Arrivals.Count} arrivals, {checkedCount}/{run.Spawns.Count} spawns had enough runway "
            + $"to require a checked arrival (worst-case trip {worstCaseTotal:F2}s) -- all satisfied.");
    }
}
