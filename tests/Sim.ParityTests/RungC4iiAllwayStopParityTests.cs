using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C4-ii (TASKS.md "Remaining right-of-way" -- the ALL-WAY-STOP sub-rung). Unlike priority
// (9b) and right-before-left (C4-i) junctions, an `type="allway_stop"` junction requires EVERY
// approach to come to a full stop first, then proceed in arrival order (longest waiter goes
// first). scenarios/27-allway-stop is the same uncontrolled cross as C4-i but node type
// allway_stop: link 0 (SJ->JN, vSouth) and link 1 (WJ->JE, vWest) have a MUTUAL <request> matrix
// (each yields to the other, state 'w'). vSouth departs at t=0, vWest at t=3.
//
// The mechanism this rung adds (and why the pre-C4-ii engine DEADLOCKED here, leaving both
// vehicles halted forever): with a mutual response matrix the priority-junction "yield to a
// present/approaching foe" rule is symmetric -- each yields to the other indefinitely. SUMO breaks
// it with the all-way-stop rule (MSLink::opened/blockedByFoe): a link is never open until the
// vehicle has actually stopped (WaitingTime > 0), and once stopped it proceeds unless a foe has
// waited strictly longer. So vSouth brakes to a full stop (9.433 -> 4.933 -> 0.433 -> 0.0 at t=19)
// then goes at t=20 (vWest, still moving, has not waited at all); vWest brakes to a full stop
// (0.0 at t=22) and goes at t=23 once vSouth has cleared the crossing. Ported as
// Engine.AllwayStopConstraint + VehicleRuntime.WaitingTime (MSVehicle::updateWaitingTime), gated
// on junction.Type == "allway_stop" so every priority/RBL scenario is byte-identical.
//
// Runs 39 steps: vSouth clears the network by t=36 and vWest (later, and delayed by its stop) by
// t=38 in the golden; 39 compares every populated golden row for both vehicles.
public class RungC4iiAllwayStopParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "27-allway-stop");

    [Fact]
    public void Run39Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(39);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C4-ii parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
