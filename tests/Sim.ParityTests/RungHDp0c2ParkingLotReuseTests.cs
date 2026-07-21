using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// GAP-2 acceptance gate (docs/HIGH-DENSITY-CALIBRATION-DESIGN.md section 3): a roadsideCapacity=1
// parkingArea referenced by two vehicles that do NOT overlap in time must LOAD (the pre-GAP-2 static
// load-time assignment threw "lot index 1 out of range") and REUSE the single lot -- runtime
// lowest-free-lot turnover (MSParkingArea::computeLastFreePos: free on pull-out, reclaim on the next
// arrival). scenarios/76-parking-lot-reuse pins this against the vanilla SUMO 1.20.0 golden: veh0
// (departPos="stop" origin) parks in lot 0 at pos 210 and pulls out at t~=11; veh1 (moving) arrives
// at t~=40 and (park-and-stays) claims the now-free lot 0 (same pos ~210). SumoSharp must reproduce
// golden.fcd.xml within tolerance.json (exact, pos/speed @ 1e-3).
public class RungHDp0c2ParkingLotReuseTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "76-parking-lot-reuse");

    [Fact]
    public void CapacityOneParkingArea_ReusedAcrossTime_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(60);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"GAP-2 parking-lot-reuse parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
