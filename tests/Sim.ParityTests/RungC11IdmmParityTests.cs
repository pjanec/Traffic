using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// C11-iv: IDMM (IDM with Memory / "Improved IDM") -- scenarios/25-idmm-carfollow, both vTypes
// carFollowModel="IDMM" on a LONG 3000m single lane (net.net.xml) so the follower stays in
// sustained congestion behind the slow leader (maxSpeed=6) long enough for the per-vehicle
// levelOfService memory to actually drift (adaptationTime=600s -- see provenance.txt). At
// levelOfService=1.0 (the ctor default) IDMM collapses to plain IDM exactly; as the follower
// settles into congestion (vNext/maxSpeed ~0.43), levelOfService drifts down and the
// steady-state gap GROWS over the run (golden.fcd.xml: ~8.81m@t60 -> ~9.52m@t249), the
// discriminating memory effect this test exercises. sigma=0 (irrelevant -- IDMM never dawdles,
// same as plain IDM), Euler, actionStepLength=1. Runs the full 250s (scenario end=250).
public class RungC11IdmmParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "25-idmm-carfollow");

    [Fact]
    public void Run250Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(250);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"RungC11Idmm (IDMM) parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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

    // Mirrors EngineRung1PlumbingTests.RepoRoot(): resolve the repo root by walking up from
    // the test assembly's location until Traffic.sln is found.
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
