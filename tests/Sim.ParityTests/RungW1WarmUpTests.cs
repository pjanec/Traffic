using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung W1 test: WarmUp(W) leaves the engine in the exact state step W of a normal run would, so
// WarmUp(W); Run(N) produces exactly the TAIL (steps W..W+N-1) of a single Run(W+N). Uses the F1
// flow scenario (56-flow-equiv) so there is ongoing, still-inserting traffic across the warm-up
// boundary (cars depart at 0/6/12/18/24). Deterministic, offline, no SUMO golden.
public class RungW1WarmUpTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "56-flow-equiv");

    private static Engine Load()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "flow.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        return engine;
    }

    [Fact]
    public void WarmUpThenRun_EqualsTailOfSingleRun()
    {
        const int warm = 20;
        const int run = 20;

        // Reference: one continuous run of warm+run steps.
        var full = Load().Run(warm + run);

        // Warm-started: advance `warm` steps silently, then run `run` steps.
        var warmed = Load();
        warmed.WarmUp(warm);
        var tail = warmed.Run(run);

        // The warm-started run must emit exactly the tail times (>= warm) of the full run, with
        // identical per-vehicle state at every one of them.
        var tailTimes = tail.AllPoints.Select(p => p.Time).Distinct().OrderBy(t => t).ToList();
        Assert.NotEmpty(tailTimes);
        Assert.True(tailTimes.Min() >= warm, $"warm-started run emitted a time {tailTimes.Min()} before the warm-up boundary {warm}");

        var comparedPoints = 0;
        foreach (var wp in tail.AllPoints)
        {
            Assert.True(full.TryGet(wp.VehicleId, wp.Time, out var fp),
                $"full run has no point for {wp.VehicleId} at t={wp.Time}");
            Assert.Equal(fp.Lane, wp.Lane);
            Assert.Equal(fp.Pos, wp.Pos, precision: 9);
            Assert.Equal(fp.Speed, wp.Speed, precision: 9);
            comparedPoints++;
        }

        // Non-vacuous: the warm-up actually crossed the boundary with live traffic present, and a
        // vehicle that departs AFTER the warm-up boundary (f.4 @ t=24) still appears -- i.e. the
        // warm-started engine kept inserting, not just replayed a frozen set.
        Assert.True(comparedPoints > 0);
        Assert.Contains(tail.VehicleIds, id => id == "f.4");
        Assert.Contains(tail.VehicleIds, id => id == "f.0"); // an early car is still on the road at the boundary
    }

    [Fact]
    public void WarmUpAlone_EmitsNothing_ButPopulatesState()
    {
        // WarmUp returns no trajectory; a subsequent Run(1) immediately emits the populated state,
        // proving the warm-up left vehicles in the sim rather than a blank engine.
        var e = Load();
        e.WarmUp(20);
        var oneStep = e.Run(1);
        Assert.NotEmpty(oneStep.VehicleIds);          // vehicles are present right away
        Assert.All(oneStep.AllPoints, p => Assert.True(p.Time >= 20)); // clock continued past the warm-up
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
