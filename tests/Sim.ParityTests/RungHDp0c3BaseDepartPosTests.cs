using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// GAP-3 acceptance gate (docs/HIGH-DENSITY-CALIBRATION-DESIGN.md section 4): departPos="base" must
// resolve to SUMO's DepartPosDefinition::BASE == MSBaseVehicle::basePos, NOT a hardcoded 0.
// scenarios/75-base-depart pins the two branches of basePos against the vanilla SUMO 1.20.0 golden:
//   veh0 (no first-edge stop): basePos = MIN(vType.Length(7) + POSITION_EPS(0.1), laneLength) = 7.1
//   veh1 (first-edge stop, endPos=3): basePos capped to MIN(7.1, MAX(0, 3)) = 3.0
// A "base -> 0" shortcut would place both at 0 and fail pos parity at step 0. SumoSharp must
// reproduce golden.fcd.xml within tolerance.json (exact, pos/speed @ 1e-3).
public class RungHDp0c3BaseDepartPosTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "75-base-depart");

    [Fact]
    public void BaseDepartPos_ResolvesToBasePos_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(100);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"GAP-3 base-departPos parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
