using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B3 (DESIGN.md "Two futures" -- live-reactivity, a DIFFERENT bar than the parity rungs):
// composes B1 (external obstacles) + B2 (NetworkRouter) into a live reroute-around-prolonged-
// blockage trigger. The blocking obstacle is a live, non-SUMO input, so these are BEHAVIORAL/
// PROPERTY tests against the engine's own TrajectorySet, not golden-FCD comparisons -- there is
// no scenarios/15-reroute/golden.fcd.xml.
//
// Fixture: scenarios/15-reroute/ -- the same diamond net as scenarios/_fixtures/routing-diamond
// (SA -> {AB,AC} -> {BD,CD} -> DE; top path AB+BD 505.07 each, bottom path AC+CD 634.63 each).
// Vehicle "veh" is routed the TOP path (SA AB BD DE) in rou.rou.xml. Tests inject a live obstacle
// on BD and set Engine.RerouteThresholdSeconds to exercise the reroute trigger.
public class RungB3RerouteTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "15-reroute");

    private static Engine LoadEngine()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        return engine;
    }

    // Test 1: a persistent obstacle on BD, threshold=5 -- veh must reroute around it (never
    // enters BD), divert onto the bottom detour (AC/CD), and still reach DE.
    [Fact]
    public void PersistentBlockage_ReroutesAroundAndReachesDestination()
    {
        var engine = LoadEngine();
        engine.RerouteThresholdSeconds = 5;
        engine.AddObstacle("blk", "BD_0", frontPos: 250, length: 5);

        var trajectory = engine.Run(300);
        var points = trajectory.PointsFor("veh").Values.OrderBy(p => p.Time).ToList();

        Assert.True(points.Count > 1, "expected veh to be present at multiple timesteps");

        // (a) Never appears on a BD_* lane at any step.
        Assert.DoesNotContain(points, p => p.Lane.StartsWith("BD"));

        // (b) Does traverse the bottom detour (AC_0 and CD_0).
        Assert.Contains(points, p => p.Lane == "AC_0");
        Assert.Contains(points, p => p.Lane == "CD_0");

        // (c) Reaches DE (a DE_0 row appears).
        Assert.Contains(points, p => p.Lane == "DE_0");
    }

    // Test 2: the obstacle clears BEFORE the threshold (endTime=3 < threshold=5) -- veh must
    // never reroute, taking the original top path (AB then BD) and still reaching DE.
    [Fact]
    public void BlockageClearsBeforeThreshold_NoRerouteTakesOriginalPath()
    {
        var engine = LoadEngine();
        engine.RerouteThresholdSeconds = 5;
        engine.AddObstacle("blk", "BD_0", frontPos: 250, length: 5, endTime: 3);

        var trajectory = engine.Run(300);
        var points = trajectory.PointsFor("veh").Values.OrderBy(p => p.Time).ToList();

        Assert.True(points.Count > 1, "expected veh to be present at multiple timesteps");

        // Original top path taken: appears on both AB_0 and BD_0.
        Assert.Contains(points, p => p.Lane == "AB_0");
        Assert.Contains(points, p => p.Lane == "BD_0");

        // Never diverts onto the bottom detour.
        Assert.DoesNotContain(points, p => p.Lane == "AC_0");
        Assert.DoesNotContain(points, p => p.Lane == "CD_0");

        // Still reaches DE.
        Assert.Contains(points, p => p.Lane == "DE_0");
    }

    // Test 3: RerouteThresholdSeconds left at its default (+infinity) -- reroute must stay
    // completely inert even WITH a persistent obstacle on BD injected: veh follows the top path
    // onto AB (heading toward BD) and never diverts to AC, confirming reroute is strictly opt-in.
    [Fact]
    public void Disabled_ByDefault_IsInertEvenWithObstaclePresent()
    {
        var engine = LoadEngine();
        // RerouteThresholdSeconds left at its default (+infinity) -- not set here.
        engine.AddObstacle("blk", "BD_0", frontPos: 250, length: 5);

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("veh").Values.OrderBy(p => p.Time).ToList();

        Assert.True(points.Count > 1, "expected veh to be present at multiple timesteps");

        // Follows the top path onto AB_0 toward BD (no reroute).
        Assert.Contains(points, p => p.Lane == "AB_0");

        // Never diverts onto the bottom detour.
        Assert.DoesNotContain(points, p => p.Lane == "AC_0");
        Assert.DoesNotContain(points, p => p.Lane == "CD_0");
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
