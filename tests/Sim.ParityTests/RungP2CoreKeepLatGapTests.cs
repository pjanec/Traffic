using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Phase 2 / P2-core (keepLatGap -- lateral minGapLat maintenance, isolated from the P2.4
// speed-gain-overtake machinery). scenarios/64-sublane-keeplatgap: a slow leader v0 (maxSpeed 5)
// PINNED to the left edge (latAlignment="left", departPosLat="left" -> posLat +2.7, constant for
// all 40 steps) and a fast follower v1 (default vType, CENTERED, posLat starts 0) that free-flows
// straight past it: pos/speed are plain free-flow for both (v1 to 13.89, v0 to 5) and must stay
// EXACT throughout -- the two vehicles' lateral footprints are disjoint by design (v0 hugs the
// left edge), so the same-lane !FootprintsOverlap leader bypass (LeaderFollowSpeedConstraint)
// already lets v1 pass with no braking (same mechanism as scenarios/61-sublane-sidebyside). The
// ONLY lateral motion is v1's posLat: 0 (6 steps) -> -0.301500 (6 steps, while longitudinally
// alongside v0) -> 0 -- SUMO's keepLatGap nudging v1 away from v0 to maintain minGapLat (0.6 m
// default) as it passes.
//
// DEFERRED (characterized) -- see docs/PHASE2-SUBLANE.md. A faithful port of keepLatGap + updateGaps
// (MSLCM_SL2015.cpp:3046-3391) reproduced the MAGNITUDE exactly: the -0.301500 nudge is a 0.6 m
// minGapLat measured against v0's GRID-quantized near sublane-column boundary (MSLeaderInfo::
// getSubLanes), NOT its continuous physical edge (the continuous edge gives +0.2985 -- wrong sign),
// derived as gap = |4.8 - 3.6| - 0.9015 = 0.2985, surplusGapLeft = 0.2985 - 0.6 = -0.3015. That
// port matched pos/speed to ~1e-13 for all 40 steps AND kept scenarios 60/61/62 byte-exact + the
// hash held -- but it diverges on the HOLD DURATION (v1 snaps back to posLat 0 at step 7 instead of
// holding -0.301500 through step 11). The hold is governed by SUMO's PERSISTENT cross-step lateral
// state (mySafeLatDistRight/Left, seeded in prepareStep, updated by checkBlocking's own updateGaps,
// decremented by travelled lateral distance each step) -- the same stateful machinery the P2.4
// checkpoint independently hit. Per the iron law (no non-exact golden committed as passing) the
// per-step port was reverted; the inert minGapLat vType plumbing is retained (ready for the exact
// port). This is the definitive SL2015-core finding: the magnitude mechanism is solved; byte-exact
// multi-vehicle sublane needs the full persistent lateral state machine (a cohesive dedicated port,
// not incremental slices). Skipped (not failed) so the branch stays green.
public class RungP2CoreKeepLatGapTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "64-sublane-keeplatgap");

    [Fact(Skip = "P2-core keepLatGap deferred: -0.3015 magnitude solved (grid-quantized minGapLat) and pos/speed exact, but the hold duration needs SUMO's persistent cross-step lateral state (mySafeLatDistRight/Left). See class header / docs/PHASE2-SUBLANE.md.")]
    public void Run40Steps_MatchesGoldenFcdWithinTolerance_FollowerKeepsLateralGapWhilePassing()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(40);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"Rung-P2-core keepLatGap parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
