using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C4-vi (priority-junction far-routed-foe false positive): a minor-road vehicle must NOT yield
// to a major-road foe that is still far up its own route. scenarios/40-farrouted-foe is a priority
// junction J where the minor ego (SJ->JN) responds to the major link WJ->JE in the request matrix,
// but the single major vehicle foeFar departs 496 m up WJ -- far beyond its approach-reservation
// range (SPEED2DIST(maxV)+brakeGap(maxV) ~ 27 m) when ego reaches the stop line. SUMO does not
// register foeFar as approaching (MSLink::setApproaching's lookahead window), so ego proceeds: it
// does the cautious minor-link approach slowdown (13.89 -> 8.64 by t=9) then crosses onto :J_0_0 at
// t=10 and JN_0 at t=11; foeFar reaches the junction only at t=38.
//
// The pre-fix engine's FindFoeVehicle matched foeFar purely because its route lane-sequence includes
// the major internal lane :J_1_0 anywhere ahead -- no proximity bound -- so ego braked to a full
// stop at the stop line (pos 92.699) and stayed stuck. Port = the reservation-distance gate
// (SPEED2DIST(maxV)+brakeGap(maxV), already used by the sameTarget-merge arm) added to
// JunctionYieldConstraint's crossing approaching-foe arm: a foe farther than that has not reserved
// the link, so ego is not blocked. Inert for a genuinely-close foe (scenario 11 / 19 / all committed
// junction scenarios stay green), so only this scenario's far-foe case changes.
//
// Runs 46 steps (t=0..45, foeFar's last golden frame): ego crosses by t=11 and completes JN; foeFar crosses at t=38.
public class RungC4viFarRoutedFoeParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "40-farrouted-foe");

    [Fact]
    public void Run46Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(46);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C4-vi far-routed-foe parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
