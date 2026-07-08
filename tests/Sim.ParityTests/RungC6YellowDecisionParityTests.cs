using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C6 (yellow decision) -- the smaller, separable first bite of "actuated / adaptive traffic
// lights + yellow decision" (the actuated/detector half is its own later rung). Rung 10 modeled a
// static TLS and always braked for red/yellow. But SUMO only brakes for a yellow (or red) light if
// the vehicle can STILL STOP before the stop line (canBrakeBeforeStopLine, MSVehicle.cpp:2648/2754);
// if it is too close -- inside the "dilemma zone" where stopping would require harder-than-comfort
// braking -- it PROCEEDS through instead.
//
// scenarios/30-yellow-decision is scenario 09's TLS net with lane speeds raised to 25 m/s and a
// Green(11)/yellow(3)/red static program. veh0 cruises WJ at 25; when the light turns yellow at
// t=11 it is at pos 255 (seen 44 m from the stop line < its 57.5 m braking distance), so it CANNOT
// stop and proceeds through the junction at full speed (t=12 on JE_0). The pre-C6 engine wrongly
// emergency-braked it to a halt at the stop line. Ported as the canBrakeBeforeStopLine gate in
// Engine.RedLightConstraint (return non-binding when the vehicle cannot brake in time); byte-
// identical for rung 10 (scenario 09) and the emergency-red scenario (16), where the vehicle always
// approaches from far enough to stop.
//
// Runs 25 steps: covers the approach, the yellow decision at t=10->11, and the pass-through.
public class RungC6YellowDecisionParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "30-yellow-decision");

    [Fact]
    public void Run25Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(25);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C6-yellow parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
