using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// P0-A acceptance gate (docs/HIGH-DENSITY-P0-DESIGN.md "P0-A"): the scenarios/41-multifile-cfg
// scenario is loaded through the NEW SUMO-faithful Engine.LoadScenario(sumocfgPath) overload --
// which reads the cfg's <input> section (net-file + a 2-entry route-files comma-list + an
// additional-file), resolves those paths relative to the cfg dir, and merges the two route-files
// into one DemandModel. The resulting trajectory must match the vanilla SUMO 1.20.0 golden
// (golden.fcd.xml) within tolerance.json. This proves the multi-file loading path is behaviourally
// identical to a single-file load (the scenario mirrors scenarios/02-two-vehicle-following).
public class RungHDp0aMultiFileScenarioParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "41-multifile-cfg");

    [Fact]
    public void MultiFileCfg_LoadsViaInputSection_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        // The whole point of P0-A: drive off the cfg alone (like `sumo -c config.sumocfg`), NOT the
        // 3-arg overload. This exercises <input> parsing + multi-file DemandParser merge + the
        // additional-file loader.
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(120);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"P0-A multi-file parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
