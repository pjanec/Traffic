using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Crossing;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;
using Sim.Pedestrians.Parking;
using Sim.Pedestrians.Tests.Parking;
using Sim.Replication;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Requirements;

// P6-3 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-OVERVIEW.md §2 "Requirements"): a requirement-indexed
// PROPERTY-test suite. Each ReqN test below names one of the 7 core pedestrian requirements from
// PEDESTRIAN-OVERVIEW.md §2 and asserts its single load-bearing invariant over a DETERMINISTIC SEEDED
// RANGE of configs (>= 5 distinct seeds/configs per test) -- not one hand-picked case. Every test
// reuses an existing PoC/task test's setup machinery (named in each test's own header comment) rather
// than inventing new production code; this suite is additive-only (new test files), touches no
// src/ and no existing test, and needs neither SUMO nor the network (hermetic, deterministic --
// VehicleRng or fixed config arrays only, never System.Random). Requirement 4 (Evacuation) needs
// Sim.Evac's EvacDistrictDirector, which this project does not reference -- see
// tests/Sim.ParityTests/Req4EvacuationRequirementTests.cs for that one test.
public class RequirementPropertyTests
{
    private readonly ITestOutputHelper _out;
    public RequirementPropertyTests(ITestOutputHelper output) => _out = output;

    private static string NetPath => Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml");

    // ================================================================================================
    // Req 1 -- Performance (the LOD scale-enabler): docs/PEDESTRIAN-OVERVIEW.md §2.1.
    //   (a) OrcaCrowd.UseParallelStep is bit-identical to serial over seeded crowds (the parallelism
    //       that enables scale) -- setup mirrors tests/Sim.ParityTests/OrcaParallelStepTests.cs.
    //   (b) A low-power (PathArc/ActivityTimeline) PedLodManager population emits ZERO per-step
    //       FreeKinematic samples over a run -- setup mirrors PedPublishGovernorTests'
    //       LowPowerOnlyPopulation_EmitsZeroPerStepCrowdFrameBytes_OverTheRun, measured directly against
    //       PedPublisher.FreeKinematicSamplesSent instead of going over the wire.
    // ================================================================================================

    private const double ParallelDt = 0.2;
    private const int ParallelAgentGx = 26;
    private const int ParallelAgentGy = 10; // 260 agents -- clears OrcaCrowd's ParallelStepThreshold (256)

    // Builds a crowd whose EVERY tunable that touches Plan()'s scratch buffers (spatial hash, static
    // obstacles, external cross-regime discs, MaxNeighbours-bounded insertion, SymmetryBreak jitter,
    // and now per-agent start-position jitter) varies deterministically with `seed` -- a genuinely
    // different scene per seed, not just a relabeled copy of one fixed scene. Positions are perturbed by
    // a VehicleRng stream seeded from `seed` alone: the stream is drawn from sequentially, in a fixed
    // single-threaded build-time loop (never during Step()), so it is fully reproducible and
    // independent of any later parallel scheduling.
    private static OrcaCrowd BuildSeededParallelCrowd(int seed, bool useParallel)
    {
        var rng = VehicleRng.SeedFor(globalSeed: 0xF00D_BEEF_0000_0001UL, entityIndex: seed);
        const int agentCount = ParallelAgentGx * ParallelAgentGy;

        var crowd = new OrcaCrowd(agentCount)
        {
            UseSpatialHash = true,
            MaxNeighbours = 6 + (seed % 5),
            SymmetryBreak = 0.02 + 0.01 * seed,
            UseParallelStep = useParallel,
        };

        var spacing = 1.2 + 0.05 * seed;
        var originX = -(ParallelAgentGx - 1) * spacing / 2.0;
        var originY = -(ParallelAgentGy - 1) * spacing / 2.0;
        var jitter = 0.05 + 0.01 * seed;

        for (var iy = 0; iy < ParallelAgentGy; iy++)
        {
            for (var ix = 0; ix < ParallelAgentGx; ix++)
            {
                var jx = (rng.NextDouble() - 0.5) * jitter;
                var jy = (rng.NextDouble() - 0.5) * jitter;
                var p = new Vec2(originX + ix * spacing + jx, originY + iy * spacing + jy);
                crowd.Add(p, radius: 0.35, maxSpeed: 1.4, goal: -p);
            }
        }

        Assert.Equal(agentCount, crowd.Count);

        var wallY = 4.0 + seed;
        crowd.AddObstacle(new[] { new Vec2(-9.0, wallY), new Vec2(9.0, wallY) });
        crowd.AddObstacle(new[] { new Vec2(-9.0, -wallY), new Vec2(9.0, -wallY) });

        var discs = new[]
        {
            new WorldDisc(-18.0 - seed, 0.0, 1.0, 0.1 * seed, 1.0),
            new WorldDisc(18.0 + seed, 1.0, -0.6, 0.2 * seed, 0.9),
        };
        crowd.SetExternalObstacles(discs);

        return crowd;
    }

    [Fact]
    public void Req1_Performance_ParallelStepIsBitIdenticalToSerial_AcrossSeededCrowds()
    {
        const int steps = 60;
        var comparisons = 0L;

        for (var seed = 0; seed < 5; seed++)
        {
            var serial = BuildSeededParallelCrowd(seed, useParallel: false);
            var parallel = BuildSeededParallelCrowd(seed, useParallel: true);

            Assert.False(serial.UseParallelStep);
            Assert.True(parallel.UseParallelStep);
            Assert.True(serial.Count >= 256, "seeded crowd must clear the parallel-step size gate");

            for (var step = 0; step < steps; step++)
            {
                serial.Step(ParallelDt);
                parallel.Step(ParallelDt);

                for (var i = 0; i < serial.Count; i++)
                {
                    var ps = serial.Position(i);
                    var pp = parallel.Position(i);
                    var vs = serial.Velocity(i);
                    var vp = parallel.Velocity(i);
                    comparisons++;

                    Assert.True(ps.X == pp.X && ps.Y == pp.Y && vs.X == vp.X && vs.Y == vp.Y,
                        $"seed {seed}, step {step}, agent {i}: position/velocity diverged between serial and parallel " +
                        $"(serial pos=({ps.X:R},{ps.Y:R}) parallel pos=({pp.X:R},{pp.Y:R}))");
                }
            }
        }

        _out.WriteLine($"[Req1a measured] 5 seeds x {steps} steps x {ParallelAgentGx * ParallelAgentGy} agents = " +
            $"{comparisons} position/velocity comparisons, ALL bit-identical (max bit-diff = 0).");
    }

    private sealed class StraightLineNav : IPedNavigation
    {
        // No navmesh needed: these peds never promote/demote (no interest sources registered), so
        // PedLodManager never calls FindPath for them after spawn -- mirrors PedPublishGovernorTests'
        // own stub.
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    // (PathArc leg count, ActivityTimeline leg count) per seeded config -- deliberately varies the MIX
    // (all-PathArc, all-lively, and blends) across 5 configs, not just population SIZE.
    private static readonly (int PathArcCount, int LivelyCount)[] LowPowerMixConfigs =
    {
        (2, 1),
        (3, 2),
        (1, 3),
        (4, 0),
        (0, 4),
    };

    [Fact]
    public void Req1_Performance_LowPowerPopulation_EmitsZeroPerStepFreeKinematicSamples()
    {
        const double dt = 0.1;
        const int steps = 400; // 40s -- several multiples of the 3s heartbeat interval
        var totalPedsChecked = 0;

        for (var seed = 0; seed < LowPowerMixConfigs.Length; seed++)
        {
            var (pathArcCount, livelyCount) = LowPowerMixConfigs[seed];
            var publisher = new PedPublisher();
            var manager = new PedLodManager(new StraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);
            var field = new InterestField(); // zero sources registered -> nothing ever promotes
            var noEntities = Array.Empty<WorldDisc>();

            for (var k = 0; k < pathArcCount; k++)
            {
                var id = 1000 * seed + k + 1;
                manager.AddPed(id, new[] { new Vec2(0.0, k * 5.0), new Vec2(50.0, k * 5.0) }, maxSpeed: 1.4, radius: 0.3, now: 0.0);
                totalPedsChecked++;
            }

            for (var k = 0; k < livelyCount; k++)
            {
                var id = 1000 * seed + 100 + k;
                var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
                {
                    new WalkSegment(new[] { new Vec2(0.0, 20.0 + k), new Vec2(30.0, 20.0 + k) }, 1.4),
                });
                manager.AddPedLively(id, timeline, maxSpeed: 1.4, radius: 0.3, now: 0.0);
                totalPedsChecked++;
            }

            var now = 0.0;
            for (var i = 0; i < steps; i++)
            {
                manager.Step(now, dt, field, noEntities);
                now += dt;
            }

            Assert.Equal(0, publisher.FreeKinematicSamplesSent.Count);
        }

        _out.WriteLine($"[Req1b measured] {LowPowerMixConfigs.Length} seeded mixes, {totalPedsChecked} low-power peds " +
            "total, over 400 steps (40s) each: FreeKinematicSamplesSent.Count == 0 in every config " +
            "(zero per-step samples -- only the spawn-time PathArc/ActivityTimeline leg + heartbeats).");
    }

    // ================================================================================================
    // Req 2 -- Believability (crowds, not lanes): docs/PEDESTRIAN-OVERVIEW.md §2.2. In seeded DENSE
    // OrcaCrowd scenes, no two agents overlap -- min pairwise centre distance >= (r_i + r_j) - eps at
    // EVERY step. Setup mirrors tests/Sim.ParityTests/OrcaConvergenceTests.cs'
    // AntipodalCircle_WithAids_StillSafe_DoesNotConverge (a dense antipodal circle -- every agent's
    // goal is directly through the crowd's centre, the densest crossing pattern a crowd sim can pose).
    // ================================================================================================

    private const double CircleDt = 0.25;
    private const double CircleOverlapEps = 0.02;

    // (agent count, circle radius, base per-agent radius) -- 5 distinct dense scenes, all close
    // variants of OrcaConvergenceTests.AntipodalCircle_WithAids_StillSafe_DoesNotConverge's own
    // MEASURED-safe recipe (circleR=8.0, radius=0.5, SymmetryBreak up to 0.5, n in {8,12}): every
    // agent's goal is diametrically opposite (through the crowd's packed centre), the densest crossing
    // pattern a laneless crowd sim can pose.
    private static readonly (int N, double CircleR, double BaseRadius)[] DenseCircleConfigs =
    {
        (8, 8.0, 0.50),
        (9, 8.0, 0.48),
        (10, 8.0, 0.50),
        (11, 8.0, 0.47),
        (12, 8.0, 0.50),
    };

    [Fact]
    public void Req2_Believability_DenseCrowd_NoTwoAgentsEverOverlap()
    {
        const int steps = 400;
        var globalMinMargin = double.MaxValue;

        for (var cfgIndex = 0; cfgIndex < DenseCircleConfigs.Length; cfgIndex++)
        {
            var (n, circleR, baseRadius) = DenseCircleConfigs[cfgIndex];
            // Mirrors OrcaConvergenceTests' proven-safe antipodal-circle aids: SymmetryBreak
            // desymmetrises the perfectly-balanced centre jam, RemoveOnArrival+ArrivalRadius retires
            // agents that settle so they stop constraining (and cannot be run into by) the rest,
            // MaxNeighbours bounds work without breaking the safety guarantee.
            var crowd = new OrcaCrowd(n)
            {
                SymmetryBreak = 0.3,
                MaxNeighbours = 6,
                RemoveOnArrival = true,
                ArrivalRadius = 0.2,
            };
            var radii = new double[n];

            for (var i = 0; i < n; i++)
            {
                var theta = 2.0 * Math.PI * i / n;
                var pos = new Vec2(circleR * Math.Cos(theta), circleR * Math.Sin(theta));
                // Slight deterministic per-agent radius variation (no two agents identical), still a
                // pure, fixed function of (cfgIndex, i) -- exercises the general r_i + r_j sum, not just
                // a uniform 2*radius special case.
                radii[i] = baseRadius + 0.01 * (i % 3);
                crowd.Add(pos, radii[i], maxSpeed: 1.5, goal: -pos);
            }

            for (var step = 0; step < steps; step++)
            {
                crowd.Step(CircleDt);

                for (var i = 0; i < n; i++)
                {
                    for (var j = i + 1; j < n; j++)
                    {
                        var sep = (crowd.Position(i) - crowd.Position(j)).Abs;
                        var minAllowed = radii[i] + radii[j];
                        var margin = sep - minAllowed;
                        globalMinMargin = Math.Min(globalMinMargin, margin);

                        Assert.True(sep >= minAllowed - CircleOverlapEps,
                            $"config {cfgIndex} (n={n}), step {step}: agents {i},{j} overlapped -- " +
                            $"sep {sep:F4} < r_i+r_j {minAllowed:F4}");
                    }
                }
            }
        }

        _out.WriteLine($"[Req2 measured] {DenseCircleConfigs.Length} seeded dense antipodal-circle configs x " +
            $"{steps} steps: worst-case observed margin (sep - (r_i+r_j)) = {globalMinMargin:F4} m " +
            $"(never below -{CircleOverlapEps} m tolerance -- ORCA reciprocal avoidance held throughout).");
    }

    // ================================================================================================
    // Req 3 -- Interactivity: docs/PEDESTRIAN-OVERVIEW.md §2.3.
    //   (a) A car crossing a junction halts for / does not run over a pedestrian in its path -- setup
    //       mirrors tests/Sim.Pedestrians.Tests/Crossing/CarStopsForPedestrianTests.cs.
    //   (b) A pedestrian avoids an external moving blocker -- setup mirrors
    //       tests/Sim.Pedestrians.Tests/Obstacles/ObstacleDodgeTests.cs' MovingBlocker_IsAvoided test.
    // ================================================================================================

    private const double CarPedDt = 1.0; // Engine's default StepLength

    // Seeded pedestrian Y positions -- all "south of the junction, on the car's own nc->cs lane, well
    // short of the cs edge" (same regime CarStopsForPedestrianTests' header comment proves safe for
    // 111.6), varied +-1.6 m around that proven point.
    private static readonly double[] CarStopPedYSeeds = { 110.0, 110.8, 111.6, 112.4, 113.2 };

    private static VehicleState SettleOntoVehicleLane(Engine engine, VehicleHandle handle, out bool inserted)
    {
        var state = default(VehicleState);
        inserted = false;
        for (var i = 0; i < 6; i++)
        {
            engine.Step();
            if (engine.TryGetVehicle(handle, out state))
            {
                inserted = true;
                if (!state.LaneId.EndsWith("_0", StringComparison.Ordinal))
                {
                    break;
                }
            }
        }

        return state;
    }

    [Fact]
    public void Req3_Interactivity_CarHaltsForPedestrianInItsPath_AcrossSeededPedPositions()
    {
        var minGaps = new double[CarStopPedYSeeds.Length];
        var minSpeedsNearPed = new double[CarStopPedYSeeds.Length];

        for (var seed = 0; seed < CarStopPedYSeeds.Length; seed++)
        {
            var pedY = CarStopPedYSeeds[seed];

            var engine = new Engine();
            engine.LoadNetwork(NetPath);
            var vtype = engine.DefineVType(new VTypeParams { VClass = "passenger", Sigma = 0.0 });
            var carHandle = engine.SpawnVehicle(vtype, "nc", "cs", departPos: 5.0);

            var carState = SettleOntoVehicleLane(engine, carHandle, out var inserted);
            Assert.True(inserted, $"seed {seed}: car should have inserted within a few steps of spawning");

            var crowd = new OrcaCrowd();
            var pedPos = new Vec2(carState.X, pedY);
            crowd.Add(pedPos, radius: 0.3, maxSpeed: 0.0, goal: pedPos);
            engine.CrowdSource = crowd;

            var minGap = double.MaxValue;
            var minSpeedNearPed = double.MaxValue;
            const double nearPedThreshold = 8.0;

            for (var i = 0; i < 40; i++)
            {
                engine.Step();
                crowd.Step(CarPedDt);

                if (!engine.TryGetVehicle(carHandle, out carState))
                {
                    break;
                }

                var gap = carState.Y - pedY;
                minGap = Math.Min(minGap, gap);
                if (gap < nearPedThreshold)
                {
                    minSpeedNearPed = Math.Min(minSpeedNearPed, carState.Speed);
                }
            }

            minGaps[seed] = minGap;
            minSpeedsNearPed[seed] = minSpeedNearPed;

            Assert.True(minGap >= 0.0, $"seed {seed} (pedY={pedY}): car's front reached/passed the pedestrian, min gap = {minGap:F3} m");
            Assert.True(minSpeedNearPed < 0.3, $"seed {seed} (pedY={pedY}): expected the car to nearly stop near the pedestrian, min speed = {minSpeedNearPed:F3} m/s");
            Assert.True(minGap < nearPedThreshold, $"seed {seed} (pedY={pedY}): car never got close to the pedestrian, min gap = {minGap:F3} m");
        }

        _out.WriteLine($"[Req3a measured] {CarStopPedYSeeds.Length} seeded ped Y positions " +
            $"[{string.Join(", ", CarStopPedYSeeds)}]: min longitudinal gaps = " +
            $"[{string.Join(", ", Array.ConvertAll(minGaps, g => g.ToString("F3")))}] m (all >= 0); " +
            $"min speeds near ped = [{string.Join(", ", Array.ConvertAll(minSpeedsNearPed, s => s.ToString("F3")))}] m/s (all < 0.3).");
    }

    private const double BlockerPedMaxSpeed = 1.4;
    private const double BlockerPedRadius = 0.3;
    private const double BlockerRadius = 0.35;
    private const double BlockerArriveRadius = 0.3;
    private const double BlockerOverlapEps = 1e-3;
    private const int BlockerMaxSteps = 3000;
    private const double BlockerDt = 0.1;

    // Seeded genuine-intercept configs: (interceptX along the ped's fixed [0,20] straight path, blocker
    // descent speed m/s). For each, the blocker's start Y is computed so it crosses y=0 at interceptX at
    // EXACTLY the instant the (non-avoiding) ped would also be there -- a real collision course, not
    // just "nearby", mirroring ObstacleDodgeTests.MovingBlocker_IsAvoided_NoOverlap_PedArrives.
    private static readonly (double InterceptX, double BlockerSpeed)[] MovingBlockerConfigs =
    {
        (8.0, 1.0),
        (10.0, 1.2),
        (12.0, 1.4),
        (14.0, 1.1),
        (16.0, 0.9),
    };

    [Fact]
    public void Req3_Interactivity_PedestrianAvoidsExternalMovingBlocker_AcrossSeededInterceptConfigs()
    {
        var start = new Vec2(0.0, 0.0);
        var goal = new Vec2(20.0, 0.0);
        var minSeparations = new double[MovingBlockerConfigs.Length];

        for (var seed = 0; seed < MovingBlockerConfigs.Length; seed++)
        {
            var (interceptX, blockerSpeed) = MovingBlockerConfigs[seed];
            var tIntercept = interceptX / BlockerPedMaxSpeed;
            var y0 = blockerSpeed * tIntercept;
            var blocker = new MovingBlocker(new Vec2(interceptX, y0), new Vec2(0.0, -blockerSpeed), BlockerRadius);

            var crowd = new OrcaCrowd();
            var pedIndex = crowd.Add(start, BlockerPedRadius, BlockerPedMaxSpeed, goal);

            var sumOfRadii = BlockerPedRadius + BlockerRadius;
            var minSeparation = double.MaxValue;
            var arrived = false;

            for (var step = 0; step < BlockerMaxSteps; step++)
            {
                crowd.SetExternalObstacles(new[] { blocker.ToWorldDisc() });
                crowd.Step(BlockerDt);
                blocker.Advance(BlockerDt);

                var pedPos = crowd.Position(pedIndex);
                var separation = (pedPos - blocker.Position).Abs;
                minSeparation = Math.Min(minSeparation, separation);

                if ((pedPos - goal).Abs <= BlockerArriveRadius)
                {
                    arrived = true;
                    break;
                }
            }

            minSeparations[seed] = minSeparation;

            Assert.True(arrived, $"seed {seed} (interceptX={interceptX}, blockerSpeed={blockerSpeed}): ped never reached its goal while avoiding the moving blocker");
            Assert.True(minSeparation >= sumOfRadii - BlockerOverlapEps,
                $"seed {seed}: ped overlapped the moving blocker -- min separation {minSeparation:F4} < sum-of-radii {sumOfRadii:F4} - eps");
        }

        _out.WriteLine($"[Req3b measured] {MovingBlockerConfigs.Length} seeded genuine-intercept configs: " +
            $"min ped-blocker separations = [{string.Join(", ", Array.ConvertAll(minSeparations, s => s.ToString("F4")))}] m " +
            $"(all >= sum-of-radii {BlockerPedRadius + BlockerRadius:F2} m - eps); every ped arrived.");
    }

    // ================================================================================================
    // Req 5 -- Disjoint crowds (accumulate + release): docs/PEDESTRIAN-OVERVIEW.md §2.5. With a
    // crossing signal, peds accumulate at the portal while the signal is closed (none crosses the stop
    // line) and cross once it opens. Setup mirrors
    // tests/Sim.Pedestrians.Tests/Crossing/CrossingGateCrowdTests.cs (POC-0's real west crossing tlLogic).
    // ================================================================================================

    private static readonly Vec2 CrossingPortalNear = new(111.6, 126.4);
    private static readonly Vec2 CrossingPortalFar = new(111.6, 113.6);
    private static readonly Vec2 CrossingDestinationBase = new(111.6, 108.0);
    private const double CrossingNearLineY = 126.4;
    private const double CrossingQueueSpacing = 2.0;
    private const int CrossingFrontSlotBuffer = 2; // mirrors CrossingGate's own private FrontSlotBuffer
    private const double CrossingMaxSpeed = 1.4;
    private const double CrossingPedRadius = 0.3;
    private const double CrossingArriveRadius = 0.4;
    private const double CrossingDt = 0.2;

    private static ICrossingSignal BuildWestSignal()
    {
        var network = PedNetworkParser.Load(NetPath);
        var westCrossing = network.Crossings.Single(c => c.Id == ":c_c3_0");
        return CrossingSignalFactory.ForCrossing(NetPath, westCrossing);
    }

    private static Vec2 CrossingDestinationFor(int registrationIndex) =>
        CrossingDestinationBase + new Vec2(registrationIndex * 0.7, 0.0);

    private static Vec2 CrossingExpectedHoldSlot(int registrationIndex) =>
        CrossingPortalNear + new Vec2(0.0, (registrationIndex + CrossingFrontSlotBuffer) * CrossingQueueSpacing);

    private static (OrcaCrowd Crowd, CrossingGate Gate, OrcaHandle[] Indices) BuildQueueScenario(
        int count, double approachExtra, ICrossingSignal signal, double startTime)
    {
        var crowd = new OrcaCrowd { MaxNeighbours = 8 };
        var gate = new CrossingGate(
            crowd, new WaypointFollower(), signal, CrossingArriveRadius,
            queueDirection: new Vec2(0.0, 1.0), CrossingQueueSpacing);

        var indices = new OrcaHandle[count];
        for (var k = 0; k < count; k++)
        {
            var path = new Vec2[] { CrossingPortalNear, CrossingPortalFar, CrossingDestinationFor(k) };
            var spawn = CrossingExpectedHoldSlot(k) + new Vec2(0.0, approachExtra);
            var idx = crowd.Add(spawn, CrossingPedRadius, CrossingMaxSpeed, goal: spawn);
            gate.AddRoute(idx, path, portalIndex: 0, CrossingMaxSpeed, startTime);
            indices[k] = idx;
        }

        return (crowd, gate, indices);
    }

    // (pedCount, approach distance behind own queue slot, spawn time) -- all spawn times fall inside
    // the west crossing's don't-walk window ([37,90) mod 90, CrossingTlReaderTests), varying both the
    // queue's population and how far out each ped starts (a different "arrival pattern" per seed).
    private static readonly (int PedCount, double ApproachExtra, double StartTime)[] CrossingArrivalConfigs =
    {
        (4, 3.0, 40.0),
        (5, 4.0, 45.0),
        (6, 2.0, 55.0),
        (7, 5.0, 60.0),
        (8, 3.5, 65.0),
    };

    [Fact]
    public void Req5_DisjointCrowds_AccumulateOnRed_DrainOnGreen_AcrossSeededArrivalPatterns()
    {
        const double walkOpensAt = 90.0;
        const double endTime = 175.0;

        for (var seed = 0; seed < CrossingArrivalConfigs.Length; seed++)
        {
            var (pedCount, approachExtra, startTime) = CrossingArrivalConfigs[seed];
            var signal = BuildWestSignal();
            var (crowd, gate, indices) = BuildQueueScenario(pedCount, approachExtra, signal, startTime);

            var crossedAt = new double?[pedCount];
            var wasAboveLine = new bool[pedCount];
            for (var k = 0; k < pedCount; k++)
            {
                wasAboveLine[k] = crowd.Position(indices[k]).Y >= CrossingNearLineY;
            }

            var now = startTime;
            while (now < endTime)
            {
                gate.Update(now);
                crowd.Step(CrossingDt);
                now += CrossingDt;

                for (var k = 0; k < pedCount; k++)
                {
                    var y = crowd.Position(indices[k]).Y;
                    if (wasAboveLine[k] && y < CrossingNearLineY && crossedAt[k] is null)
                    {
                        crossedAt[k] = now;
                    }

                    wasAboveLine[k] = y >= CrossingNearLineY;
                }
            }

            // Invariant part 1: zero crossings while the signal was ever closed -- i.e. every recorded
            // crossing time falls at/after the signal genuinely allowed it.
            var crossedDuringHold = crossedAt.Count(t => t is not null && !signal.WalkAllowed(t.Value));
            Assert.Equal(0, crossedDuringHold);

            // Invariant part 2: the queue actually drains once the walk phase opens -- every ped
            // eventually crosses and completes its route.
            Assert.All(crossedAt, t => Assert.NotNull(t));
            for (var k = 0; k < pedCount; k++)
            {
                Assert.True(gate.IsRouteComplete(indices[k]), $"seed {seed}: ped {k} should have completed its route by t={endTime}");
            }

            var crossedAfterOpen = crossedAt.Count(t => t is not null && t.Value >= walkOpensAt);
            Assert.True(crossedAfterOpen == pedCount, $"seed {seed}: expected all {pedCount} peds to cross only after the walk phase opened at t={walkOpensAt}");

            _out.WriteLine($"[Req5 measured] seed {seed} (pedCount={pedCount}, approachExtra={approachExtra}, startTime={startTime}): " +
                $"crossedDuringHold={crossedDuringHold}, all {pedCount} peds crossed after walkOpensAt={walkOpensAt} and completed their route.");
        }
    }

    // ================================================================================================
    // Req 6 -- Parking-lot situations: docs/PEDESTRIAN-OVERVIEW.md §2.6.
    //   (a) Over seeded lot configs, peds weaving a lot maintain min separation from parked + maneuvering
    //       cars -- setup mirrors tests/Sim.Pedestrians.Tests/Parking/LotCouplingScenario.cs +
    //       LotCouplingMutualAvoidanceTests.cs.
    //   (b) Board/alight lifecycle events fire -- setup mirrors
    //       tests/Sim.Pedestrians.Tests/Parking/BoardAlightTests.cs.
    // ================================================================================================

    private const double LotOverlapEps = 1e-2;
    private const double LotPedRadius = 0.3;
    private const double LotPedMaxSpeed = 1.4;
    private const double LotCarMaxSpeed = 3.0;
    private const double LotDt = 0.05;
    private const int LotMaxSteps = 1000;

    // Uniformly scales an entire lot layout (car start/goal, parked-box centres + half-extents, ped
    // start/goal) by `scale` -- a genuinely different concrete geometry per seed while preserving the
    // qualitative "weave through two row gaps, cross the car's lane" relationship exactly (everything,
    // including the gaps, scales together).
    private static (LotCoupling Coupling, int CarId, OrcaHandle[] PedIds) BuildSeededLot(double scale)
    {
        var coupling = new LotCoupling();
        var boxCentersX = new[] { 8.0, 14.0, 20.0 };
        foreach (var cx in boxCentersX)
        {
            coupling.AddParkedCarBox(new Vec2(cx * scale, 5.0 * scale), 2.15 * scale, 0.9 * scale);
        }

        var carId = coupling.AddCar(new Vec2(0.0, 0.0), new Vec2(40.0 * scale, 0.0), LotCarMaxSpeed);

        var pedRoutes = new (Vec2 Start, Vec2 Goal)[]
        {
            (new Vec2(11.0 * scale, 9.0 * scale), new Vec2(11.0 * scale, -9.0 * scale)),
            (new Vec2(17.0 * scale, 9.0 * scale), new Vec2(17.0 * scale, -9.0 * scale)),
            (new Vec2(25.0 * scale, 9.0 * scale), new Vec2(25.0 * scale, -9.0 * scale)),
        };

        var pedIds = new OrcaHandle[pedRoutes.Length];
        for (var i = 0; i < pedRoutes.Length; i++)
        {
            pedIds[i] = coupling.AddPedestrian(pedRoutes[i].Start, LotPedRadius, LotPedMaxSpeed, pedRoutes[i].Goal);
        }

        return (coupling, carId, pedIds);
    }

    private static readonly double[] LotScaleConfigs = { 1.0, 1.05, 1.1, 1.15, 1.2 };

    [Fact]
    public void Req6_Parking_PedsWeavingLot_MaintainMinSeparationFromParkedAndManeuveringCars()
    {
        var globalMinPedCarSep = double.MaxValue;
        var globalMinPedBoxSep = double.MaxValue;

        for (var seed = 0; seed < LotScaleConfigs.Length; seed++)
        {
            var scale = LotScaleConfigs[seed];
            var (coupling, carId, pedIds) = BuildSeededLot(scale);

            // LotCouplingScenario's own remarks: Dt=0.05 was tuned so a fixed-speed car/ped pair closes
            // no more than a proven-safe fraction of the (unscaled) gap between coupling rebuilds.
            // Scaling every DISTANCE by `scale` without also scaling the per-step displacement would
            // silently erode that margin (a smaller lot at the same speed/dt covers relatively MORE
            // ground per step) -- scaling Dt by the same factor keeps the displacement-vs-gap ratio, and
            // therefore the proven safety margin, invariant across seeds.
            var dt = LotDt * scale;

            for (var step = 0; step < LotMaxSteps; step++)
            {
                coupling.Step(dt);

                var carFootprint = coupling.CarFootprint(carId);
                var boxFootprints = coupling.ParkedCarFootprints;

                for (var i = 0; i < pedIds.Length; i++)
                {
                    var pedPos = coupling.PedPosition(pedIds[i]);
                    var pedRadius = coupling.PedRadius(pedIds[i]);

                    var distToCar = LotCoupling.DistanceToBox(pedPos, carFootprint);
                    globalMinPedCarSep = Math.Min(globalMinPedCarSep, distToCar);
                    Assert.True(distToCar >= pedRadius - LotOverlapEps,
                        $"seed {seed} (scale={scale}), step {step}: pedestrian {i} overlaps the car footprint " +
                        $"(distance {distToCar:F3} < radius {pedRadius:F3})");

                    foreach (var box in boxFootprints)
                    {
                        var distToBox = LotCoupling.DistanceToBox(pedPos, box);
                        globalMinPedBoxSep = Math.Min(globalMinPedBoxSep, distToBox);
                        Assert.True(distToBox >= pedRadius - LotOverlapEps,
                            $"seed {seed} (scale={scale}), step {step}: pedestrian {i} overlaps a parked-car box " +
                            $"(distance {distToBox:F3} < radius {pedRadius:F3})");
                    }
                }
            }

            // Sanity: this seed's peds genuinely reached (or came very close to) their goals -- avoidance
            // did not deadlock anyone.
            for (var i = 0; i < pedIds.Length; i++)
            {
                var finalDist = (coupling.PedGoal(pedIds[i]) - coupling.PedPosition(pedIds[i])).Abs;
                Assert.True(finalDist <= 2.0, $"seed {seed}: pedestrian {i} did not make progress toward its goal (final distance {finalDist:F2})");
            }
        }

        _out.WriteLine($"[Req6a measured] {LotScaleConfigs.Length} seeded lot scales " +
            $"[{string.Join(", ", LotScaleConfigs)}]: min ped-car separation = {globalMinPedCarSep:F3} m, " +
            $"min ped-box separation = {globalMinPedBoxSep:F3} m (both >= ped radius {LotPedRadius:F2} m - eps throughout).");
    }

    private const double BoardRadius = 2.5;

    private static Vec2 ParkOneCar()
    {
        var engine = ParkingScenarioFixture.NewEngine();
        var vType = ParkingScenarioFixture.DefineCarType(engine);
        var handle = ParkingScenarioFixture.SpawnAndSettleCar(engine, vType);

        var controller = new ParkingController(engine, ParkingScenarioFixture.ManeuverArriveRadius);
        var carId = controller.Track(handle);

        var pedNet = ParkingScenarioFixture.LoadPedNetwork();
        var entryPos = ParkingScenarioFixture.EntryPoi(pedNet);
        controller.EnterLot(carId, entryPos, ParkingScenarioFixture.Slot, ParkingScenarioFixture.ManeuverMaxSpeed);

        for (var i = 0; i < 60 && controller.RegimeOf(carId) != CarRegime.Parked; i++)
        {
            engine.Step();
            controller.Step(1.0);
        }

        Assert.Equal(CarRegime.Parked, controller.RegimeOf(carId));
        return controller.ParkedPositionOf(carId)!.Value;
    }

    // (walk-start offset from the parked car, alight offset, alight destination offset) -- 5 distinct
    // approach/alight geometries, all with |walk-start offset| > BoardRadius (so boarding is a real walk-
    // in, not an instant board) and used as-is (an alight offset's own magnitude bounds how far the
    // alighted ped may spawn from the car, per PersonRideController.Alight's contract).
    private static readonly (Vec2 WalkStartOffset, Vec2 AlightOffset, Vec2 AlightDestOffset)[] BoardAlightConfigs =
    {
        (new Vec2(-5.0, -10.0), new Vec2(2.0, 0.0), new Vec2(15.0, 0.0)),
        (new Vec2(-6.0, -8.0), new Vec2(-2.0, 0.0), new Vec2(12.0, 3.0)),
        (new Vec2(-4.0, -12.0), new Vec2(0.0, 2.0), new Vec2(10.0, -5.0)),
        (new Vec2(-8.0, -6.0), new Vec2(1.5, 1.5), new Vec2(18.0, 2.0)),
        (new Vec2(-3.0, -9.5), new Vec2(-1.5, -1.5), new Vec2(14.0, -3.0)),
    };

    [Fact]
    public void Req6_Parking_BoardAndAlightLifecycleEvents_FireAcrossSeededApproachGeometries()
    {
        for (var seed = 0; seed < BoardAlightConfigs.Length; seed++)
        {
            var (walkStartOffset, alightOffset, alightDestOffset) = BoardAlightConfigs[seed];
            var carPos = ParkOneCar();

            // ----- board -----
            var walkStart = carPos + walkStartOffset;
            Assert.True(walkStartOffset.Abs > BoardRadius, $"seed {seed}: walk-start offset must start outside BoardRadius");

            var ride = new PersonRideController(BoardRadius);
            var personId = ride.AddWalking(walkStart, LotPedRadius, LotPedMaxSpeed, goal: carPos);
            Assert.Equal(PersonRegime.Walking, ride.RegimeOf(personId));

            var parkedCars = new (int CarId, Vec2 Position)[] { (0, carPos) };
            IReadOnlyList<int> boardedThisStep = Array.Empty<int>();
            var now = 0.0;
            for (var i = 0; i < 60 && boardedThisStep.Count == 0; i++)
            {
                ride.Crowd.Step(1.0);
                now += 1.0;
                boardedThisStep = ride.TryBoard(parkedCars, now);
            }

            Assert.Single(boardedThisStep);
            Assert.Equal(personId, boardedThisStep[0]);
            Assert.Equal(PersonRegime.Riding, ride.RegimeOf(personId));
            Assert.Equal(0, ride.Crowd.Count);

            var boardEvents = ride.Events.Where(e => e.Kind == PersonLifecycleEventKind.Boarded).ToList();
            Assert.Single(boardEvents);
            Assert.Equal(personId, boardEvents[0].PersonId);

            // ----- alight -----
            var alightRide = new PersonRideController(BoardRadius);
            Assert.Equal(0, alightRide.Crowd.Count);

            var alightDestination = carPos + alightDestOffset;
            var alightPersonId = alightRide.Alight(carPos, alightOffset, LotPedRadius, LotPedMaxSpeed, alightDestination, now: 0.0);

            Assert.Equal(PersonRegime.Walking, alightRide.RegimeOf(alightPersonId));
            Assert.Equal(1, alightRide.Crowd.Count);

            var spawnPos = alightRide.PositionOf(alightPersonId);
            var distToCar = (spawnPos - carPos).Abs;
            Assert.True(distToCar <= alightOffset.Abs + 1e-6, $"seed {seed}: alighted pedestrian spawned {distToCar} m from the car");

            var alightEvents = alightRide.Events.Where(e => e.Kind == PersonLifecycleEventKind.Alighted).ToList();
            Assert.Single(alightEvents);
            Assert.Equal(alightPersonId, alightEvents[0].PersonId);

            _out.WriteLine($"[Req6b measured] seed {seed}: board fired at t={now:F0}s (1 BoardEvent for person {personId}); " +
                $"alight spawned {distToCar:F3} m from car (1 AlightEvent for person {alightPersonId}).");
        }
    }

    // ================================================================================================
    // Req 7 -- Networking to many IG channels: docs/PEDESTRIAN-OVERVIEW.md §2.7.
    //   (a) server == IG survives the serialized wire across seeded mixed populations (exact for
    //       low-power PathArc/ActivityTimeline, <= cm-tolerance for quantized FreeKinematic) -- setup
    //       mirrors PedReplicationRoundTripTests.cs.
    //   (b) the single-stream rate measured by PedBandwidthMeter stays < 500 Mbit/s at (seeded
    //       variations of) the target mix -- setup mirrors PedPublishGovernorTests'
    //       TargetMix_100kAllHighPower_StaysUnder500MbitPerSecond.
    // ================================================================================================

    private sealed class ReplicationStraightLineNav : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    private readonly record struct MixConfig(int PathArcCount, int LivelyCount, int PromotedCount);

    // 5 seeded population MIXES -- every combination of all-PathArc / all-lively / blended, always
    // including at least one ped that promotes to FreeKinematic mid-run (so every seed exercises all
    // three wire paths).
    private static readonly MixConfig[] ReplicationMixConfigs =
    {
        new(2, 1, 1),
        new(3, 2, 1),
        new(1, 3, 2),
        new(4, 0, 1),
        new(0, 4, 2),
    };

    // Quantization-safe speed pool: whole/half values are exactly representable through FrameCodec's
    // cm-quantization AND the float32 downcast (mirrors PedReplicationRoundTripTests' Ped1 remarks).
    private static readonly double[] SafeSpeeds = { 1.0, 1.5, 2.0, 2.5 };

    [Fact]
    public void Req7_Networking_ServerEqualsIgOverTheWire_AcrossSeededMixedPopulations()
    {
        const double dt = 0.1;
        const int steps = 250;
        var maxHighPowerErrorAcrossSeeds = 0.0;

        for (var seed = 0; seed < ReplicationMixConfigs.Length; seed++)
        {
            var cfg = ReplicationMixConfigs[seed];
            var publisher = new PedPublisher();
            var manager = new PedLodManager(new ReplicationStraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);
            var field = new InterestField();

            var nextId = 1;
            var pathArcIds = new List<int>();
            var livelyIds = new List<int>();
            var promotedIds = new List<int>();

            for (var k = 0; k < cfg.PathArcCount; k++)
            {
                var id = nextId++;
                var speed = SafeSpeeds[k % SafeSpeeds.Length];
                var path = new[] { new Vec2(0.0, k * 10.0), new Vec2(20.0, k * 10.0) }; // whole-metre points
                manager.AddPed(id, path, speed, radius: 0.3, now: 0.0);
                pathArcIds.Add(id);
            }

            for (var k = 0; k < cfg.LivelyCount; k++)
            {
                var id = nextId++;
                var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
                {
                    new WalkSegment(new[] { new Vec2(0.0, 100.0 + k * 7.3), new Vec2(11.7 + k, 100.0 + k * 7.3) }, 1.53),
                    new PauseSegment(1.25, "sip"),
                    new WalkSegment(new[] { new Vec2(11.7 + k, 100.0 + k * 7.3), new Vec2(11.7 + k, 111.4 + k * 7.3) }, 1.53),
                });
                manager.AddPedLively(id, timeline, maxSpeed: 1.53, radius: 0.3, now: 0.0);
                livelyIds.Add(id);
            }

            for (var k = 0; k < cfg.PromotedCount; k++)
            {
                var id = nextId++;
                var start = new Vec2(200.0 + k * 30.0, 0.0);
                var path = new[] { start, new Vec2(220.0 + k * 30.0, 0.0) };
                manager.AddPed(id, path, maxSpeed: 1.4, radius: 0.3, now: 0.0);
                field.Register(new InterestSource(start, promoteRadius: 3.0, demoteRadius: 6.0), InterestSourceKind.EntityAttached);
                promotedIds.Add(id);
            }

            var bus = new InMemoryPedReplicationBus();
            var replicationPublisher = new PedReplicationPublisher(bus.Sink);
            var receiver = new PedReplicationReceiver(bus.Source);

            replicationPublisher.Publish(publisher.Events);
            bus.Source.Pump();
            receiver.Drain();

            var now = 0.0;
            var anyPromotedObservedOnWire = false;
            var seedMaxHighPowerError = 0.0;

            for (var i = 0; i < steps; i++)
            {
                var before = publisher.Events.Count;
                manager.Step(now, dt, field, Array.Empty<WorldDisc>());
                now += dt;

                var fresh = new List<PedEvent>(publisher.Events.Count - before);
                for (var e = before; e < publisher.Events.Count; e++)
                {
                    fresh.Add(publisher.Events[e]);
                }

                replicationPublisher.Publish(fresh);
                bus.Source.Pump();
                receiver.Drain();

                foreach (var id in promotedIds)
                {
                    if (manager.ModelOf(id) == PedDrModel.FreeKinematic
                        && receiver.Ig.Knows(id) && receiver.Ig.ModelOf(id) == PedDrModel.FreeKinematic)
                    {
                        var server = manager.PositionOf(id, now);
                        var ig = receiver.Ig.Reconstruct(id, now);
                        var err = (server - ig).Abs;
                        seedMaxHighPowerError = Math.Max(seedMaxHighPowerError, err);
                        anyPromotedObservedOnWire = true;
                    }
                }
            }

            Assert.True(anyPromotedObservedOnWire, $"seed {seed}: no promoted ped was ever observed as FreeKinematic on the wire");
            Assert.True(seedMaxHighPowerError <= 0.02, $"seed {seed}: high-power reconstruction error {seedMaxHighPowerError:E4} m exceeded the 0.02 m budget");
            maxHighPowerErrorAcrossSeeds = Math.Max(maxHighPowerErrorAcrossSeeds, seedMaxHighPowerError);

            foreach (var id in pathArcIds)
            {
                Assert.Equal(PedDrModel.PathArc, manager.ModelOf(id));
                Assert.Equal(PedDrModel.PathArc, receiver.Ig.ModelOf(id));
                for (var t = 0.0; t <= now; t += dt * 4.0)
                {
                    var server = manager.PositionOf(id, t);
                    var ig = receiver.Ig.Reconstruct(id, t);
                    Assert.Equal(server.X, ig.X);
                    Assert.Equal(server.Y, ig.Y);
                }
            }

            foreach (var id in livelyIds)
            {
                Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(id));
                Assert.Equal(PedDrModel.ActivityTimeline, receiver.Ig.ModelOf(id));
                for (var t = 0.0; t <= now; t += dt * 4.0)
                {
                    var server = manager.PositionOf(id, t);
                    var ig = receiver.Ig.ReconstructSample(id, t).Pos;
                    Assert.Equal(server.X, ig.X);
                    Assert.Equal(server.Y, ig.Y);
                }
            }

            _out.WriteLine($"[Req7a measured] seed {seed} (PathArc={cfg.PathArcCount}, Lively={cfg.LivelyCount}, Promoted={cfg.PromotedCount}): " +
                $"exact match for all {pathArcIds.Count} PathArc + {livelyIds.Count} ActivityTimeline peds; " +
                $"max high-power (quantized) error = {seedMaxHighPowerError:E4} m.");
        }

        _out.WriteLine($"[Req7a measured] worst-case high-power error across all seeds: {maxHighPowerErrorAcrossSeeds:E4} m (budget 0.02 m).");
    }

    // Seeded high-power FRACTIONS of a 100k-total population -- the design's target mix is ~10k
    // high/~90k low of a 100k total; this sweeps around that ratio (8%-12%) rather than testing one
    // hand-picked split. Only high-power ids emit a per-step FreeKinematicSample (the true zero-per-step
    // property for the low-power remainder, proven directly by Req1b, holds here too by construction).
    private static readonly double[] TargetMixHighPowerFractions = { 0.08, 0.09, 0.10, 0.11, 0.12 };

    [Fact]
    public void Req7_Networking_BandwidthStaysUnder500MbitPerSecond_AcrossSeededTargetMixes()
    {
        const int population = 100_000;
        const int steps = 5;
        const double dt = 0.1;
        var peaks = new double[TargetMixHighPowerFractions.Length];

        for (var seed = 0; seed < TargetMixHighPowerFractions.Length; seed++)
        {
            var fraction = TargetMixHighPowerFractions[seed];
            var highCount = (int)(population * fraction);

            var meter = new PedBandwidthMeter();
            var bus = new InMemoryPedReplicationBus();
            var publisher = new PedReplicationPublisher(bus.Sink, scheduler: null, governor: null, meter: meter, stepDt: dt);

            var time = 0.0;
            for (var step = 0; step < steps; step++)
            {
                time += dt;
                var events = new List<PedEvent>(highCount);
                for (var id = 0; id < highCount; id++)
                {
                    var pos = new Vec2(id * 0.01, step * 0.1);
                    var vel = new Vec2(1.0, 0.0);
                    events.Add(new FreeKinematicSample(id, time, pos, vel));
                }

                publisher.Publish(events);
                bus.Source.Pump();
            }

            var peakMbit = meter.PeakStepMbitPerSecond(dt);
            peaks[seed] = peakMbit;

            Assert.True(peakMbit < 500.0, $"seed {seed} (fraction={fraction:P0}, highCount={highCount}): peak rate {peakMbit:F2} Mbit/s exceeded the 500 Mbit/s budget");
            Assert.Equal(highCount, bus.Source.LatestCrowdFrame.Count);
        }

        _out.WriteLine($"[Req7b measured] {TargetMixHighPowerFractions.Length} seeded target-mix fractions of a " +
            $"{population}-total population: peak Mbit/s = [{string.Join(", ", Array.ConvertAll(peaks, p => p.ToString("F2")))}] " +
            "(all < 500 Mbit/s budget).");
    }
}
