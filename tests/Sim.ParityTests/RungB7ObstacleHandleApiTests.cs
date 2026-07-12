using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-API.md §4 (D5): the handle-based external-obstacle API and its generational ObstacleStore.
// The B1/B5/B6 rungs prove the BEHAVIOUR (now via the handle API directly); this file adds the
// generational stale-handle contract and the absolute steady-state anchor. Behavioural/property tests.
public class RungB7ObstacleHandleApiTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle");

    private const double ObstacleFrontPos = 250.0;
    private const double ObstacleLength = 5.0;
    private const double ObstacleBackPos = ObstacleFrontPos - ObstacleLength; // 245
    private const double ExpectedSteadyPos = 242.499;
    private const double LaneMaxSpeed = 13.89;

    private static Engine Load()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        return engine;
    }

    // The handle-based AddObstacle holds the follower at the Krauss steady gap behind the obstacle (the
    // absolute-behaviour anchor cross-checked against scenarios/13-stopped-leader).
    [Fact]
    public void HandleApi_HoldsAtKraussSteadyGap()
    {
        var engine = Load();
        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: ObstacleFrontPos, length: ObstacleLength);

        var points = engine.Run(60).PointsFor("follower");
        foreach (var p in points.Values)
        {
            Assert.True(p.Pos <= ObstacleBackPos, $"overlapped obstacle at t={p.Time}: {p.Pos} > {ObstacleBackPos}");
        }

        var last = points.Values.Last();
        Assert.Equal(ExpectedSteadyPos, last.Pos, precision: 3);
        Assert.Equal(0.0, last.Speed, precision: 3);
    }

    // Removing an obstacle by handle, then calling UpdateObstacle on that STALE handle, is an inert
    // no-op (the generation no longer matches) -- the moved obstacle never reappears, so the follower
    // runs free. Proves the generational invalidation, not just "removed from the map".
    [Fact]
    public void StaleHandleAfterRemove_UpdateIsInert()
    {
        var engine = Load();
        var lane = engine.GetLane("e0_0");
        var h = engine.AddObstacle(lane, frontPos: ObstacleFrontPos, length: ObstacleLength);

        engine.RemoveObstacle(h);
        // A stale correction that tries to move the (removed) obstacle somewhere else must NOT resurrect
        // it or write into a recycled slot.
        engine.UpdateObstacle(h, frontPos: 100.0, speed: 0.0);

        var points = engine.Run(60).PointsFor("follower");
        var last = points.Values.Last();

        // No obstacle anywhere -> free-flow, driving well past both 100 and the former 245.
        Assert.Equal(LaneMaxSpeed, last.Speed, precision: 2);
        Assert.True(last.Pos > ObstacleBackPos, $"follower should be past {ObstacleBackPos}: pos={last.Pos}");
        Assert.All(points.Values.Skip(1), p => Assert.True(p.Speed > 0.0, $"unexpected stop at t={p.Time}"));
    }

    // A recycled slot gets a fresh generation, so a re-added obstacle's handle never equals the removed
    // one; and after ClearObstacles an outstanding handle is stale (its Update is inert).
    [Fact]
    public void RecycledSlot_GetsFreshGeneration_AndClearInvalidatesHandles()
    {
        var engine = Load();
        var lane = engine.GetLane("e0_0");

        var h1 = engine.AddObstacle(lane, frontPos: ObstacleFrontPos, length: ObstacleLength);
        engine.RemoveObstacle(h1);
        var h2 = engine.AddObstacle(lane, frontPos: ObstacleFrontPos, length: ObstacleLength);

        // Same slot index reused, but a bumped generation -> distinct handle.
        Assert.NotEqual(h1, h2);

        engine.ClearObstacles();
        // h2 is now stale; this correction must be inert (no obstacle in the store at all).
        engine.UpdateObstacle(h2, frontPos: 100.0, speed: 0.0);

        var last = engine.Run(60).PointsFor("follower").Values.Last();
        Assert.Equal(LaneMaxSpeed, last.Speed, precision: 2);
        Assert.True(last.Pos > ObstacleBackPos, $"follower should be past {ObstacleBackPos}: pos={last.Pos}");
    }


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
