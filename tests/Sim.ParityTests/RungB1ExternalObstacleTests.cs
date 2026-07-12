using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B1 (DESIGN.md "Two futures" -- live-reactivity, a DIFFERENT bar than the parity rungs):
// the external obstacle is not in any SUMO run, so these are BEHAVIORAL/PROPERTY tests against
// the engine's own TrajectorySet, not golden-FCD comparisons. Test 1's steady-gap value is a
// cross-check against scenarios/13-stopped-leader/golden.fcd.xml (a REAL stopped leader at the
// same lane position), proving the virtual-obstacle constraint produces the identical Krauss
// steady gap as a real vehicle leader would.
public class RungB1ExternalObstacleTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle");

    // scenarios/13-stopped-leader/golden.fcd.xml: follower settles at pos=242.499 behind a real
    // leader whose front=250, length=5 (back=245) -- 245 - minGap(2.5) - NUMERICAL_EPS(0.001).
    // Cited rather than reparsed here since this file only needs the one scalar; the parity
    // rung's own test (Rung's *ParityTests) is what actually re-derives it from the golden.
    private const double ExpectedSteadyPos = 242.499;
    private const double ObstacleFrontPos = 250.0;
    private const double ObstacleLength = 5.0;
    private const double ObstacleBackPos = ObstacleFrontPos - ObstacleLength; // 245
    private const double LaneMaxSpeed = 13.89;

    [Fact]
    public void SteadyState_HoldsAtKraussGapAndNeverOverlapsObstacle()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: ObstacleFrontPos, length: ObstacleLength);

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("follower");

        Assert.True(points.Count > 1, "expected the follower to be present at multiple timesteps");

        // Never overlaps the obstacle's back at ANY step (property test: no-overlap).
        foreach (var point in points.Values)
        {
            Assert.True(
                point.Pos <= ObstacleBackPos,
                $"follower overlapped the obstacle at t={point.Time}: pos={point.Pos} > back={ObstacleBackPos}");
        }

        // Late/steady state: holds at the Krauss steady gap, at rest.
        var last = points.Values.Last();
        Assert.Equal(ExpectedSteadyPos, last.Pos, precision: 3);
        Assert.Equal(0.0, last.Speed, precision: 3);
    }

    [Fact]
    public void ObstacleRemoval_FollowerResumesAndPassesThroughFormerObstaclePosition()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double removalTime = 25.0;
        engine.AddObstacle(
            engine.GetLane("e0_0"), frontPos: ObstacleFrontPos, length: ObstacleLength,
            startTime: double.NegativeInfinity, endTime: removalTime);

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("follower");

        // Stopped (steady gap, ~zero speed) at a step just before the obstacle is removed.
        Assert.True(trajectory.TryGet("follower", removalTime - 1.0, out var justBefore));
        Assert.Equal(ExpectedSteadyPos, justBefore.Pos, precision: 3);
        Assert.Equal(0.0, justBefore.Speed, precision: 3);

        // After removal, speed increases again and reaches free-flow max within ~6 steps.
        var reachedMaxTime = -1.0;
        for (var t = removalTime; t <= removalTime + 10.0; t += 1.0)
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

        Assert.True(reachedMaxTime >= 0, "follower never reached free-flow max speed after obstacle removal");
        Assert.True(
            reachedMaxTime <= removalTime + 6.0,
            $"follower took too long to reaccelerate to max speed: reached at t={reachedMaxTime}");

        // Advances PAST the former obstacle's back position, proving it actually resumed through
        // where the obstacle used to be (not just accelerated in place).
        var last = points.Values.Last();
        Assert.True(last.Pos > ObstacleBackPos, $"follower never passed the former obstacle position: pos={last.Pos}");
    }

    [Fact]
    public void NoObstacle_IsInertAndFollowerReachesFreeFlow()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        // Add then clear, to also exercise ClearObstacles/RemoveObstacle as no-ops on the result.
        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: ObstacleFrontPos, length: ObstacleLength);
        engine.ClearObstacles();

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("follower");

        var last = points.Values.Last();
        Assert.Equal(LaneMaxSpeed, last.Speed, precision: 2);
        Assert.True(last.Pos > ObstacleBackPos, $"follower should have driven well past the former obstacle back: pos={last.Pos}");

        // Never stops: speed is always positive once inserted (free-flow acceleration only).
        Assert.All(points.Values.Skip(1), point => Assert.True(point.Speed > 0.0));
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot().
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
