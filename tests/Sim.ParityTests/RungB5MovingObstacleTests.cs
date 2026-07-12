using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B5-i (TASKS.md "The external-agent interop" -- generalizes B1's STATIC external obstacle
// to a MOVING one driven outside SUMO, e.g. a navmesh/RVO agent). Like B1, the moving obstacle is
// not in any SUMO run, so these are BEHAVIORAL/PROPERTY tests against the engine's own
// TrajectorySet, not golden-FCD comparisons -- mirrors RungB1ExternalObstacleTests's structure
// and comment idiom, reusing the same scenarios/14-external-obstacle fixture (a `follower`
// vehicle on lane `e0_0`, LaneMaxSpeed=13.89, dt=1s per Engine.AdvanceObstacles's per-step
// dead-reckoning contract).
public class RungB5MovingObstacleTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle");
    private const double LaneMaxSpeed = 13.89;

    // Engine.AdvanceObstacles dead-reckons FrontPos += Speed*dt once per step, in the Input phase
    // BEFORE this step's PlanMovements -- so the obstacle position PlanMovements(time=T-dt) reads
    // (and therefore the ego position ExecuteMoves produces, emitted at time=T) is already
    // advanced to T. That means the obstacle's back position AT THE SAME emitted trajectory time
    // T is exactly initialBack + speed*T -- reconstructing it this way (rather than trying to
    // separately record it) is exact, not an approximation, given the dead-reckoning is linear.
    private static double ObstacleBackAtExact(double initialFrontPos, double length, double speed, double time) =>
        (initialFrontPos - length) + speed * time;

    [Fact]
    public void MovingObstacleAhead_NeverOverlapsAndSettlesToTrailingAtObstacleSpeed()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double obstacleFrontPos = 150.0;
        const double obstacleLength = 5.0;
        const double obstacleSpeed = 5.0;
        const double obstacleMaxDecel = 4.5;

        engine.AddMovingObstacle(
            engine.GetLane("e0_0"), frontPos: obstacleFrontPos, length: obstacleLength,
            speed: obstacleSpeed, maxDecel: obstacleMaxDecel);

        var trajectory = engine.Run(45);
        var points = trajectory.PointsFor("follower");

        Assert.True(points.Count > 1, "expected the follower to be present at multiple timesteps");

        // Never overlaps the obstacle's (moving) back at ANY step -- property test: no-overlap,
        // exactly like B1's static check, just against a per-step-advanced back position.
        foreach (var point in points.Values)
        {
            var back = ObstacleBackAtExact(obstacleFrontPos, obstacleLength, obstacleSpeed, point.Time);
            Assert.True(
                point.Pos <= back,
                $"follower overlapped the moving obstacle at t={point.Time}: pos={point.Pos} > back={back}");
        }

        // Late/steady state: trailing a MOVING leader, not stopping (contrast B1's stop-at-rest).
        // Speed settles near the obstacle's own speed (the Krauss constant-speed-leader
        // equilibrium), strictly positive, with a bounded (small, non-runaway) trailing gap.
        var last = points.Values.Last();
        Assert.True(last.Speed > 0.0, $"follower should be cruising behind a moving leader, not stopped: speed={last.Speed}");
        Assert.True(
            Math.Abs(last.Speed - obstacleSpeed) < 0.5,
            $"follower speed should settle near the obstacle's speed: speed={last.Speed}, obstacleSpeed={obstacleSpeed}");

        var lastBack = ObstacleBackAtExact(obstacleFrontPos, obstacleLength, obstacleSpeed, last.Time);
        var gap = lastBack - last.Pos;
        Assert.True(gap >= 0.0, $"gap must never go negative (no collision): gap={gap}");
        Assert.True(gap < 20.0, $"trailing gap should be bounded/small, not a free-flow-distance runaway: gap={gap}");
    }

    [Fact]
    public void MovingObstacleDeactivates_FollowerResumesToFreeFlow()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double obstacleFrontPos = 60.0;
        const double obstacleLength = 5.0;
        const double obstacleSpeed = 5.0;
        const double obstacleMaxDecel = 4.5;
        const double deactivateTime = 20.0;

        engine.AddMovingObstacle(
            engine.GetLane("e0_0"), frontPos: obstacleFrontPos, length: obstacleLength,
            speed: obstacleSpeed, maxDecel: obstacleMaxDecel,
            startTime: double.NegativeInfinity, endTime: deactivateTime);

        var trajectory = engine.Run(45);
        var points = trajectory.PointsFor("follower");

        // Trailing (positive, near obstacle speed) at a step just before deactivation.
        Assert.True(trajectory.TryGet("follower", deactivateTime - 1.0, out var justBefore));
        Assert.True(justBefore.Speed > 0.0, "follower should still be moving (trailing), not stopped, before deactivation");
        Assert.True(
            Math.Abs(justBefore.Speed - obstacleSpeed) < 1.0,
            $"follower should be trailing near the obstacle's speed just before deactivation: speed={justBefore.Speed}");

        // After deactivation, speed increases again and reaches free-flow max within a bounded
        // number of steps (mirrors RungB1ExternalObstacleTests.ObstacleRemoval...'s ~6-step bound).
        var reachedMaxTime = -1.0;
        for (var t = deactivateTime; t <= deactivateTime + 10.0; t += 1.0)
        {
            if (!trajectory.TryGet("follower", t, out var point))
            {
                continue;
            }

            if (Math.Abs(point.Speed - LaneMaxSpeed) < 1e-2)
            {
                reachedMaxTime = t;
                break;
            }
        }

        Assert.True(reachedMaxTime >= 0, "follower never reached free-flow max speed after obstacle deactivation");
        Assert.True(
            reachedMaxTime <= deactivateTime + 8.0,
            $"follower took too long to reaccelerate to max speed: reached at t={reachedMaxTime}");

        var last = points.Values.Last();
        Assert.Equal(LaneMaxSpeed, last.Speed, precision: 2);
    }

    // Static regression cross-check: a Speed=0 AddMovingObstacle must degenerate EXACTLY to B1's
    // stop-at-rest behavior (same scenario/parameters as
    // RungB1ExternalObstacleTests.SteadyState_HoldsAtKraussGapAndNeverOverlapsObstacle) -- proving
    // the moving code path (AdvanceObstacles' Speed==0 skip + ObstacleConstraint's predMaxDecel
    // conditional) is a no-op for a static obstacle, byte-identical to B1's own AddObstacle path.
    [Fact]
    public void ZeroSpeedMovingObstacle_DegeneratesExactlyToB1SteadyState()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double obstacleFrontPos = 250.0;
        const double obstacleLength = 5.0;
        const double expectedSteadyPos = 242.499; // == RungB1ExternalObstacleTests.ExpectedSteadyPos

        engine.AddMovingObstacle(
            engine.GetLane("e0_0"), frontPos: obstacleFrontPos, length: obstacleLength,
            speed: 0.0, maxDecel: 4.5);

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("follower");

        var last = points.Values.Last();
        Assert.Equal(expectedSteadyPos, last.Pos, precision: 3);
        Assert.Equal(0.0, last.Speed, precision: 3);
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot() / RungB1ExternalObstacleTests.RepoRoot().
    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
