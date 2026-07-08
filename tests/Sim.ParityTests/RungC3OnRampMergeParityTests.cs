using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C3 (TASKS.md "on-ramp merge" / minor-link CAUTIOUS APPROACH): scenarios/19-onramp-merge --
// mainline M (A->J, priority 10, major) and ramp R (B->J, priority 1, minor) BOTH feed the SAME
// downstream lane D_0 (sameTarget merge). Vehicles: mA (mainline, depart 0) and rA (ramp,
// depart 2), both at 13.89, sigma=0. The junction link R->D (:J_0_0) is minor and yields (request
// index 0, response="10").
//
// The decisive property this rung adds (and why the pre-C3 engine failed it at firstDiv=t=8):
// mA is ~390m away when rA nears the junction (pos 111 at t=8) -- a HUGE gap -- so rA is NOT
// gap-blocked by any foe. Yet SUMO's rA DECELERATES on approach (11.906333 at t=8, 7.406333 at
// t=9), enters :J_0_0 at t=10 (10.006333), merges to D_0 at t=11 and re-accelerates. That slowdown
// is SUMO's minor-link cautious approach: brake toward the stop line while `seen` exceeds the
// link's foe-visibility distance (4.5), then release and enter once within visibility (the gap is
// confirmed clear). This is Engine.JunctionYieldConstraint's C3 arm, ported from the vendored
// v1_20_0 DEBUG_PLAN_MOVE trace for rA (every per-step vLinkWait reproduced to <1e-3). mA itself
// crosses on the MAJOR link with no yielding at all -- a pure free-flow cruise -- so its whole
// trajectory is the ordinary single-lane free-flow that every prior rung already matches.
//
// Runs 72 steps to cover the golden's FULL extent: rA departs at 2, brakes/enters/merges over
// t=8..12, and clears the network by ~t=47; mA (the mainline major vehicle) departs at 0 and
// cruises the whole ~1000m route (M + :J_1_0 + D), still present on D_0 at the golden's last
// populated step (t=71) and gone by t=72. 72 therefore compares every populated golden row for
// BOTH vehicles (a shorter run would flag mA's later rows as presence mismatches).
public class RungC3OnRampMergeParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "19-onramp-merge");

    [Fact]
    public void Run72Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(72);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C3 parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
