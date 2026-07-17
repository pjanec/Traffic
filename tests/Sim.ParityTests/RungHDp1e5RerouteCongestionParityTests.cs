using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// P1E-5 acceptance (docs/HIGH-DENSITY-P1E-DESIGN.md §6 Tier 2a): the SUMO-faithful reroute-congestion
// anchor. scenarios/45-reroute-congestion has device.rerouting on (probability=1, period=30,
// adaptation-steps=18, routing-algorithm=astar, jitter OFF); vanilla SUMO 1.20.0 diverts all flow
// vehicles from the decisively-slower single-lane short path to the fast 2-lane detour. SumoSharp
// (periodic reroute device) must reproduce golden.fcd.xml within tolerance. The O->J common prefix
// (~43s) exceeds the 30s period, so SUMO's pre-insertion reroute vs SumoSharp's first periodic
// reroute are masked on the shared prefix (both divert at J before ever entering the short edge).
public class RungHDp1e5RerouteCongestionParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "45-reroute-congestion");

    [Fact]
    public void RerouteCongestion_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(300);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"P1E-5 reroute-congestion parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
        };
        foreach (var attribute in result.Attributes)
        {
            lines.Add($"  attribute={attribute.Attribute} maxAbsError={attribute.MaxAbsError} rmse={attribute.Rmse} withinTolerance={attribute.WithinTolerance}");
        }
        if (result.PresenceMismatches.Count > 0)
        {
            lines.Add("  presence mismatches (first 10):");
            foreach (var mismatch in result.PresenceMismatches.Take(10))
            {
                lines.Add($"    {mismatch.Kind} vehicle={mismatch.VehicleId} time={mismatch.Time?.ToString() ?? "n/a"}");
            }
        }
        return string.Join(Environment.NewLine, lines);
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
