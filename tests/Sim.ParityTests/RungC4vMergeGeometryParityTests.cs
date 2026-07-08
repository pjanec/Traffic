using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Rung C4-v (TASKS.md "sameTarget-merge conflict geometry") -- the ASYMMETRIC merge anchor that
// C4-iv's symmetric scenario 31 could not exercise. scenarios/29-merge-yield is an asymmetric
// on-ramp merge: a fast ramp vehicle rA (R->J, 13.89) converges onto the mainline exit D behind a
// slow mainline vehicle mA (M->J, 6.0) whose two internal lanes are NOT mirror images, so the
// merge conflict's per-link lengthBehindCrossing terms do NOT cancel. C4-iv (symmetric) was exact
// because those terms cancelled; here they carry a real residual that only the ported merge
// geometry resolves.
//
// The residual comes from MSLink::computeDistToDivergence (MSLink.cpp:561, sameSource=false arm),
// which walks the two internal lane shapes backward from the merge point to where they are minDist
// (2.5) apart, giving each link its lengthBehindCrossing; getLeaderInfo then forms the merge gap as
// distToCrossing - minGap - leaderBackDist2, where leaderBackDist2 applies the MSLink.cpp:1633-1638
// asymmetry correction (add (flbc-lbc) when leaderBackDist<0). With the geometry ported
// (PolylineGeometry.ComputeDistToDivergence + Engine.SameTargetMergeConstraint) rA yields, dips to
// ~2.26 as it enters the junction behind mA, then follows mA onto D converging to 6.0 -- exact to
// 1e-3. Verified byte-exact against the vendored v1_20_0 DEBUG trace (lbc/flbc 10.827662/10.822642).
//
// Runs the golden's full 90 steps (t=0..89): the merge decision resolves by t=15 (rA settled on D
// at 6.0), after which both vehicles cruise the long exit edge D without ever leaving it.
public class RungC4vMergeGeometryParityTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "29-merge-yield");

    [Fact]
    public void Run90Steps_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(90);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-C4-v parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
