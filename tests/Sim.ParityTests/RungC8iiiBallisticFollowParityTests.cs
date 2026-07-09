using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C8-iii (ballistic car-following): under the BALLISTIC integration update
// (step-method.ballistic=true), the follower's safe speed must come from the ballistic
// safe-speed branches, not the Euler ones. scenarios/42-ballistic-follow: a slow leader (maxSpeed 5,
// departPos 100) and a fast follower (13.89, departPos 0) on a single 500 m lane. The follower
// free-flows to t=7, then car-follows the leader down to ~5 m/s. C8-i already made the POSITION
// update ballistic (free-flow parity, scenario 21); this rung makes the SAFE-SPEED ballistic too:
// KraussModel.MaximumSafeStopSpeedBallistic (MSCFModel.cpp:855-910) + the ballistic brakeGap,
// threaded through FollowSpeed via a `ballistic` flag gated on config.Ballistic (so every Euler
// scenario is byte-identical). Pre-C8-iii the engine used the Euler safe-speed and diverged from t=8
// (13.89 vs golden 13.24). Runs 40 steps (t=0..39; neither vehicle exits the 500 m lane).
public class RungC8iiiBallisticFollowParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "42-ballistic-follow");

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
            $"Rung-C8-iii ballistic-follow parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
