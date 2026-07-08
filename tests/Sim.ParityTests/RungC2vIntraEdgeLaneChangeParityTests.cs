using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C2-v (intra-edge mid-route lane change): a vehicle enters a middle edge on the arrival lane
// fixed by the incoming connection, but the only onward connection to its next route edge leaves
// from a SIBLING lane, so it must lane-change across that edge before the junction.
// scenarios/37-intraedge-lanechange: E0 (1 lane) -> E1 (2 lanes) -> E2 (1 lane); E0_0 connects only
// to E1_1 (forced arrival on the "wrong" middle lane), and E1->E2 leaves only from E1_0. SUMO
// golden: E0_0 (t<=14), arrive E1_1 at t=15, STAY on E1_1 through t=17, change to E1_0 at t=18,
// :N2_0_0 at t=29, E2_0 at t=30 -- i.e. the FCD shows the arrival lane E1_1 for three steps before
// the strategic change, then the exit lane E1_0.
//
// Port = the (Exit, Arrival) lane-sequence split (NetworkModel.ResolveSequenceCore /
// _laneSeqArrival): the routing pool holds the EXIT lane (E1_0, the strategic-LC target + onward
// connection source) while the crossing lands the vehicle on the ARRIVAL lane (E1_1); the existing
// strategic lane change (TryStrategicLaneChange, C2-ii) then converges arrival->exit at t=18, and
// the convergence guard holds the E1->E2 crossing until it does. This is the multi-lane
// generalization of the departure-edge redirect; for every route with no intra-edge change
// Arrival == Exit everywhere, so all pre-C2-v scenarios stay byte-identical.
//
// Runs 44 steps (t=0..43, v0's last golden frame).
public class RungC2vIntraEdgeLaneChangeParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "37-intraedge-lanechange");

    [Fact]
    public void Run44Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(44);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C2-v intra-edge-lane-change parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
