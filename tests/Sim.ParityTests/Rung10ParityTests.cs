using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung 10's real parity test: a single vehicle (sigma=0) drives route "WJ JE" through a
// traffic-light-controlled junction J. The signal is red for sim time [0, 30) then green
// (100s cycle, only 'r'/'G' occur -- no yellow). The vehicle approaches during red, brakes to
// a halt at the stop line (WJ_0 pos 298.999, laneLength - DIST_TO_STOPLINE_EXPECT_PRIORITY),
// holds through the remainder of red, then accelerates through the junction at green (t=30,
// crossing WJ_0 -> :J_0_0 -> JE_0 via rung 9a's lane-sequence traversal). Runs the scenario's
// full length (end=50, step-length=1 => 50 steps) and compares against golden.fcd.xml within
// tolerance.
public class Rung10ParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light");

    [Fact]
    public void Run50Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(50);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-10 parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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

    // Mirrors EngineRung1PlumbingTests.RepoRoot(): resolve the repo root by walking up from
    // the test assembly's location until Traffic.sln is found.
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
