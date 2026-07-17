using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// P0-C1 acceptance gate (docs/HIGH-DENSITY-P0-DESIGN.md "P0-C1"): scenarios/42-symbolic-depart
// exercises departSpeed="max" + departLane="best" end-to-end against the vanilla SUMO 1.20.0
// golden. The net (from 18-strategic-turnlane) has E1 (2 lanes) -> E2 (1 lane) where ONLY E1_1
// continues onward, so departLane="best" must resolve to lane index 1 by route-continuation length
// (unambiguous, occupancy-tiebreak-free, hence deterministic/RNG-insensitive per owner Q1b), and
// departSpeed="max" must resolve to the lane speed limit (13.89 m/s, no leader clamp). SumoSharp
// must reproduce golden.fcd.xml within tolerance.json.
public class RungHDp0c1SymbolicDepartScenarioParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "42-symbolic-depart");

    [Fact]
    public void SymbolicDepart_MaxSpeedBestLane_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        // Load via the SUMO-faithful cfg overload (also confirms P0-A + P0-C1 compose).
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(90);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"P0-C1 symbolic-depart parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
        };

        foreach (var attribute in result.Attributes)
        {
            lines.Add(
                $"  attribute={attribute.Attribute} maxAbsError={attribute.MaxAbsError} rmse={attribute.Rmse} withinTolerance={attribute.WithinTolerance}");
        }

        if (result.PresenceMismatches.Count > 0)
        {
            lines.Add("  presence mismatches:");
            foreach (var mismatch in result.PresenceMismatches)
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
