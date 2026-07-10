using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung ER1's parity test (the `!canBrake` ignore-red arm). An emergency vehicle
// (vClass=emergency, sigma=0, maxSpeed=25, jmDriveAfterRedTime="0") cruises WJ at 25 m/s toward
// the traffic-light junction J. The signal is GREEN for [0,11) then turns RED directly (no
// yellow). At the step planning t=10->11 the vehicle is at WJ_0 pos 255 (seen 45 from the stop
// line < brakeDist ~57.5): it is physically too close to stop, so canBrakeBeforeStopLine is
// false and it PROCEEDS through the red, crossing onto JE_0 by t=12 at free-flow (25).
//
// This exercises the RED dilemma-zone "go" -- the sibling of scenario 30's YELLOW go. In SUMO
// the emergency vehicle's jmDriveAfterRedTime="0" isolates MSVehicle::ignoreRed's `!canBrake`
// arm (MSVehicle.cpp:7302): ignoreRedTime=0 is >= 0 so the red-arm branch is taken, but
// `0 > redDuration` is always false (the time-based privilege never fires), so the only reason
// ignoreRed returns true is `!canBrake`. The engine models MSVehicle.cpp:2754's outer
// `&& canBrakeBeforeStopLine` gate (RedLightConstraint returns +inf when the vehicle cannot
// brake before the stop line), which yields the identical trajectory. Non-vacuous: a
// stop-at-red engine would halt at ~WJ_0 298.999 instead of crossing.
//
// Runs the scenario's full length (end=30, step-length=1 => 30 steps) and compares against
// golden.fcd.xml within tolerance.
public class RungER1ParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "50-emergency-red-dilemma");

    [Fact]
    public void Run30Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(30);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-ER1 parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
