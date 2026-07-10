using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung R2 (rail support) trajectory parity test: two vClass="rail" trains following on a single
// straight track (carFollowModel="Krauss"). The capped leader (maxSpeed=5) is caught by the
// faster follower, which settles into the Krauss steady-state gap with RAIL params
// (minGap=5 + tau=1.0 * v=5 => 10 m at v=5; see golden.fcd.xml). This is the rail analog of
// Rung4ParityTests (passenger two-vehicle following): it proves the existing leader vsafe
// constraint + deceleration bound reproduce SUMO exactly once the leader/follower are rail
// vehicles with rail defaults (decel=1.3, length=135, minGap=5). Exact @1e-3 on lane,pos,speed.
// Mirrors RungA1ParityTests.cs, pointed at scenarios/48-two-train-following.
public class RungR2TwoTrainFollowingParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "48-two-train-following");

    [Fact]
    public void Run120Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

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
            $"Rung-R2 two-train parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
