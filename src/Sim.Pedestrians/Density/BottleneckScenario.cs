using Sim.Core.Orca;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians.Density;

// POC-4 (docs/PEDESTRIAN-POC-PLAN.md POC-4; docs/PEDESTRIAN-DESIGN.md §3 "Believability"): a
// "mall entrance" bottleneck -- a wall with a narrow gap that a large group must funnel through --
// built entirely from committed pieces (OrcaCrowd.AddObstacle for the wall,
// PedRouteController/WaypointFollower for the hand-placed gate waypoint, exactly as POC-5's
// ObstacleDodgeTests already does for a box detour). No Engine, no navmesh: each agent's path is
// [start, gate-waypoint, far-side goal] -- the gate waypoint is the "strategic layer" stand-in that
// gives pure-reactive ORCA a persistent pull TOWARD the opening instead of stalling flat against
// the wall (see ObstacleDodgeTests' BuildPedPath remarks for why a bare same-row goal alone is not
// enough once an obstacle blocks the direct line).
public sealed class BottleneckScenario
{
    // Wall geometry: a single vertical wall at x = GateX spanning y in [-WallHalfHeight,
    // WallHalfHeight], broken by a centered gap of width GateWidth. Approach side is x < GateX
    // (where the queue forms); far side is x > GateX (where the drained stream appears).
    public double GateX { get; init; } = 20.0;
    public double GateWidth { get; init; } = 1.6;
    public double WallHalfHeight { get; init; } = 10.0;

    public int AgentCount { get; init; } = 64;
    public double Radius { get; init; } = 0.3;
    public double MaxSpeed { get; init; } = 1.4;
    public double ArriveRadius { get; init; } = 0.3;
    public double Dt { get; init; } = 0.1;
    public int MaxSteps { get; init; } = 6000;

    public int MaxNeighbours { get; init; } = 10;
    public double SymmetryBreak { get; init; } = 0.03;
    public bool UseSpatialHash { get; init; } = true;

    // Approach-side "queue region" used for the queue/no-permanent-jam measurement: a NARROW band
    // immediately before the gate, spanning the wall's whole height. Deliberately narrow (not the
    // whole approach side) and, critically, agents spawn well OUTSIDE it (see ApproachSpawnDepth
    // below) -- otherwise "peak queue count" would trivially equal the spawn count from step 0 and
    // prove nothing about a queue actually FORMING at the choke point.
    public double QueueRegionDepth { get; init; } = 3.0;

    // How far back from the queue region the group spawns (so it must walk toward the gate before
    // any of it can register as "queued" -- makes the accumulate-then-drain dynamic observable).
    public double ApproachSpawnDepth { get; init; } = 9.0;

    public BottleneckResult Run()
    {
        var crowd = new OrcaCrowd
        {
            UseSpatialHash = UseSpatialHash,
            MaxNeighbours = MaxNeighbours,
            RemoveOnArrival = true,
            ArrivalRadius = ArriveRadius,
            SymmetryBreak = SymmetryBreak,
        };

        var gapHalf = GateWidth / 2.0;
        // Two thin wall segments with a centered gap between them.
        crowd.AddObstacle(new[] { new Vec2(GateX, -WallHalfHeight), new Vec2(GateX, -gapHalf) });
        crowd.AddObstacle(new[] { new Vec2(GateX, gapHalf), new Vec2(GateX, WallHalfHeight) });

        var spawnYHalf = WallHalfHeight - 1.0;
        var spawnFar = GateX - QueueRegionDepth - 1.0;             // 1 m clear gap before the queue band
        var spawnNear = spawnFar - ApproachSpawnDepth;
        var spawnPositions = CorridorScenario.GridPositions(spawnNear, spawnFar, -spawnYHalf, spawnYHalf, AgentCount);

        var indices = new int[AgentCount];
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var farGoalX = GateX + QueueRegionDepth + ApproachSpawnDepth;

        for (var i = 0; i < AgentCount; i++)
        {
            var start = spawnPositions[i];

            // Small deterministic per-agent spread within the gap (index-derived, no RNG) so agents
            // don't all target the exact same point -- realistic "more than one person can fit
            // through a 1.6 m gap side by side" while still forcing genuine convergence. Margin to
            // the wall is generous (gapHalf - Radius - 0.15) so an agent at the extreme lane offset
            // still clears the wall edge comfortably instead of grinding along it. The waypoint
            // itself sits just PAST the gate (not exactly in the doorway plane), so a passing agent
            // only needs to get within ArriveRadius of a point it is already walking through, rather
            // than having to converge precisely on a point sitting in the doorway itself -- the
            // latter turns the gap into a second, tighter bottleneck than the wall itself.
            var laneOffset = ((i % 3) - 1) * (gapHalf - Radius - 0.15);
            var gateWaypoint = new Vec2(GateX + 2.0, laneOffset);
            var farGoal = new Vec2(farGoalX, start.Y);

            var path = new[] { start, gateWaypoint, farGoal };
            var index = crowd.Add(start, Radius, MaxSpeed, goal: start);
            indices[i] = index;
            controller.AddRoute(index, path, MaxSpeed);
        }

        var trajectory = new List<Vec2[]>();
        var arrivedStep = new int?[AgentCount];
        var queueCountByStep = new List<int>();
        var gapCrossingStepBySide = new List<int>(); // step at which each agent's x first exceeds GateX

        var crossedGate = new bool[AgentCount];
        int steps;
        for (steps = 0; steps < MaxSteps; steps++)
        {
            controller.Update();
            crowd.Step(Dt);

            var frame = new Vec2[AgentCount];
            var queueCount = 0;
            for (var i = 0; i < AgentCount; i++)
            {
                var pos = crowd.Position(indices[i]);
                frame[i] = pos;

                if (pos.X < GateX && pos.X >= GateX - QueueRegionDepth)
                {
                    queueCount++;
                }

                if (!crossedGate[i] && pos.X >= GateX)
                {
                    crossedGate[i] = true;
                    gapCrossingStepBySide.Add(steps);
                }

                if (arrivedStep[i] is null && !crowd.IsActive(indices[i]))
                {
                    arrivedStep[i] = steps;
                }
            }

            trajectory.Add(frame);
            queueCountByStep.Add(queueCount);

            var allArrived = true;
            for (var i = 0; i < AgentCount; i++)
            {
                if (arrivedStep[i] is null)
                {
                    allArrived = false;
                    break;
                }
            }

            if (allArrived)
            {
                steps++;
                break;
            }
        }

        return Measure(trajectory, arrivedStep, queueCountByStep, gapCrossingStepBySide, steps);
    }

    private BottleneckResult Measure(
        List<Vec2[]> trajectory,
        int?[] arrivedStep,
        List<int> queueCountByStep,
        List<int> gapCrossingSteps,
        int steps)
    {
        var arrived = arrivedStep.Count(s => s.HasValue);
        var totalTime = steps * Dt;

        var peakQueue = queueCountByStep.Count > 0 ? queueCountByStep.Max() : 0;
        var finalQueue = queueCountByStep.Count > 0 ? queueCountByStep[^1] : 0;

        // Queue count shortly after the start, before even the FRONT spawn row has had time to
        // reach the queue band (front row is 1 m short of the band, i.e. well under a second's
        // walk at MaxSpeed) -- should be near 0, so PeakQueueCount vs EarlyQueueCount shows the
        // queue actually FORMING over the run, not just reporting the spawn count.
        var earlyIndex = Math.Min(queueCountByStep.Count - 1, (int)Math.Round(0.3 / Dt));
        var earlyQueue = queueCountByStep.Count > 0 ? queueCountByStep[Math.Max(0, earlyIndex)] : 0;

        // Gap through-rate, fine-grained: bucket cumulative gap-crossings into 5 s windows purely
        // for display -- at a mean rate on the order of 1 ped every ~2-3 s, a 5 s bucket holds only
        // a handful of events, so its own Poisson-ish sampling noise (0 vs 2 crossings in a bucket)
        // swings the instantaneous rate wildly even for a genuinely STATIONARY process. It is kept
        // in the result for illustration, not as the stability gate.
        gapCrossingSteps.Sort();
        var windowSeconds = 5.0;
        var windowSteps = Math.Max(1, (int)Math.Round(windowSeconds / Dt));
        var windowRates = new List<double>();
        if (gapCrossingSteps.Count > 0)
        {
            var maxStep = gapCrossingSteps[^1];
            for (var w0 = 0; w0 <= maxStep; w0 += windowSteps)
            {
                var w1 = w0 + windowSteps;
                var count = gapCrossingSteps.Count(s => s >= w0 && s < w1);
                windowRates.Add(count / windowSeconds);
            }
        }

        // Gap through-rate, coarse (the actual stability measurement): split the flowing phase --
        // from the first to the last gap crossing -- into quarters BY TIME and report each quarter's
        // average rate. Each quarter spans ~AgentCount/4 crossings, which is a large-enough sample
        // that its rate reflects the underlying process rather than small-window sampling noise.
        // "Stable" means these quarters stay within the same rough order of magnitude of each other
        // (no monotonic collapse toward zero while agents are still queued, which would signal a jam).
        var quarterRates = new List<double>();
        double meanThroughRate;
        double minSteadyRate;
        double maxSteadyRate;
        if (gapCrossingSteps.Count >= 4)
        {
            var firstStep = gapCrossingSteps[0];
            var lastStep = gapCrossingSteps[^1];
            var flowSteps = Math.Max(1, lastStep - firstStep);
            var quarterSteps = Math.Max(1, flowSteps / 4);
            for (var q = 0; q < 4; q++)
            {
                var q0 = firstStep + (q * quarterSteps);
                var q1 = q == 3 ? lastStep + 1 : q0 + quarterSteps;
                var count = gapCrossingSteps.Count(s => s >= q0 && s < q1);
                var durationSeconds = Math.Max(1, q1 - q0) * Dt;
                quarterRates.Add(count / durationSeconds);
            }

            meanThroughRate = gapCrossingSteps.Count / (flowSteps * Dt);
            minSteadyRate = quarterRates.Min();
            maxSteadyRate = quarterRates.Max();
        }
        else
        {
            meanThroughRate = gapCrossingSteps.Count > 0 ? gapCrossingSteps.Count / totalTime : 0.0;
            minSteadyRate = meanThroughRate;
            maxSteadyRate = meanThroughRate;
        }

        return new BottleneckResult(
            Steps: steps,
            SimulatedSeconds: totalTime,
            Arrived: arrived,
            Total: AgentCount,
            PeakQueueCount: peakQueue,
            EarlyQueueCount: earlyQueue,
            FinalQueueCount: finalQueue,
            MeanGateThroughRate: meanThroughRate,
            MinSteadyGateThroughRate: minSteadyRate,
            MaxSteadyGateThroughRate: maxSteadyRate,
            QuarterThroughRates: quarterRates,
            WindowThroughRates: windowRates,
            Trajectory: trajectory);
    }
}

// Measurements from one BottleneckScenario.Run() -- see POC-4 success conditions
// (docs/PEDESTRIAN-POC-PLAN.md POC-4 §2): queue formation, gate through-rate stability, and
// whether the approach queue clears by the end of the run (no permanent jam). MeanGateThroughRate /
// Min|MaxSteadyGateThroughRate / QuarterThroughRates are the coarse (time-quartile) stability
// measurement; WindowThroughRates is the finer 5 s-bucket series kept only for illustration (see
// Measure() remarks on small-window sampling noise).
public sealed record BottleneckResult(
    int Steps,
    double SimulatedSeconds,
    int Arrived,
    int Total,
    int PeakQueueCount,
    int EarlyQueueCount,
    int FinalQueueCount,
    double MeanGateThroughRate,
    double MinSteadyGateThroughRate,
    double MaxSteadyGateThroughRate,
    List<double> QuarterThroughRates,
    List<double> WindowThroughRates,
    List<Vec2[]> Trajectory);
