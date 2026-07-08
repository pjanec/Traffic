using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// C11-iii: CACC (Cooperative Adaptive Cruise Control) car-following model + carFollowModel
// dispatch -- scenarios/24-cacc-carfollow, both vTypes carFollowModel="CACC" on scenario 01's
// 1000m single lane. "lead" (maxSpeed=6) never has its own leader (single-lane, front of route),
// so its own CACC state is only ever driven by the free-flow/speed-control arm. "follow" (default
// desired speed) free-accelerates under speed control (time_gap>2) until the gap to "lead" tightens
// through the 1.5-2.0 time-gap HYSTERESIS band (reading the per-vehicle CaccControlMode state),
// then under 1.5 (COOPERATIVE gap control, since the leader IS also CACC -- reads the ego's own
// last-step acceleration), settling at the tight CACC cooperative gap. sigma=0, Euler,
// actionStepLength=1.
public class RungC11CaccParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "24-cacc-carfollow");

    [Fact]
    public void Run70Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(70);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"RungC11 (CACC) parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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

    // Mirrors RungC11AccParityTests.RepoRoot(): resolve the repo root by walking up from the test
    // assembly's location until Traffic.sln is found.
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
