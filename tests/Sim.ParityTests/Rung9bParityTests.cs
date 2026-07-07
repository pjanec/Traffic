using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung 9b-ii/iii's real parity test: two vehicles at a priority junction --
// scenarios/11-priority-junction, route WJ->JE (vMajor, priority=10, no yielding) crossing
// route SJ->JN (vMinor, priority=1, yields to vMajor via link1's RespondsTo(2)). Exercises
// the full mechanism: vMajor cruises through with NO junction constraint at all (never has a
// foe it must yield to); vMinor brakes for the stop line while vMajor is still on its
// approach lane (WJ_0), keeps braking via adaptToJunctionLeader once vMajor enters its own
// internal lane (:J_2_0), then free-accelerates again once vMajor has cleared the junction
// (JE_0+). Runs 40 steps (scenario end=60, but vMinor arrives at t=35 and vMajor at t=31 --
// 40 comfortably covers brake-hold-accelerate-clear for both).
public class Rung9bParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "11-priority-junction");

    [Fact]
    public void Run40Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(40);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-9b parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
