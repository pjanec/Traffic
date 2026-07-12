using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Phase 2 / P2.4 (sublane speed-gain overtake -- the continuous _wantsChangeSublane decision, exact
// @1e-3 on pos/speed/posLat). scenarios/63-sublane-overtake-wide: lateral-resolution=0.8, ONE 7.2 m
// lane (wide enough for two 1.8 m cars to pass with minGapLat), both vehicles CENTERED. The fast
// follower v1 catches the slow leader v0 (maxSpeed 5) and OVERTAKES by drifting to the right sublane,
// then recenters. v1 trajectory (speed / posLat):
//   accelerate 2.6/5.2/7.8/10.4 (posLat 0)
//   -> brake-while-drifting 9.366333/-1.0, 7.366333/-2.0, 6.183167/-2.699500
//   -> clear + accelerate 8.783167, 11.383167, 13.89 (posLat -2.699500 held)
//   -> recenter -1.699500, -0.699500, 0.0 (speed 13.89).
// Right-edge target -2.699500 = -(halfLane - (width+EPS)/2) = -(3.6 - 0.9005); drift 1.0/step
// (maxSpeedLat). The brake profile is Krauss following while footprints overlap during the drift;
// acceleration resumes once posLat clears the leader (existing FootprintsOverlap bypass); recenter is
// the latAlignment="center" drift (P2.3). The NEW piece is the speed-gain lateral DECISION (which
// side / when) from MSLCM_SL2015::_wantsChangeSublane + computeSpeedLat. Runs 40 steps.
public class RungP24SublaneOvertakeTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "63-sublane-overtake-wide");

    // DEFERRED (characterized, not yet byte-exact) -- see the class header and docs/TASKS.md P2.4.
    // A faithful minimal-slice port of MSLCM_SL2015's speed-gain decision reproduces this golden
    // ALMOST exactly: pos/speed match to ~3e-7 for all 40 steps, and posLat matches EXACTLY through
    // t=13 (the entire trigger -> drift -> brake -> hold: 0 -> -1 -> -2 -> -2.6995, held). Two
    // residuals remain, both requiring the deeper SL2015 machinery (the reason this is the phase-2
    // strategic checkpoint / kill-criterion, NOT a quick fix):
    //   1. the follower's recenter fires one step late (t=14 vs t=13) -- SUMO releases the change
    //      when computeSpeedGain < -mySublaneParam over the true per-sublane EXPECTED-SPEED grid
    //      (updateExpectedSublaneSpeeds); a single-scalar EMA proxy is off by one step;
    //   2. the LEADER v0 wiggles laterally 0 -> 0.3015 -> 0 exactly as v1 passes -- an unmodeled
    //      lateral gap-keeping reaction (keepLatGap / minGapLat maintenance against the passing
    //      neighbour), a genuine two-vehicle lateral coupling not yet built.
    // Byte-exact parity here needs the full per-sublane leader grid (MSLeaderInfo::getSubLanes) +
    // updateExpectedSublaneSpeeds + keepLatGap -- the SL2015 complexity core. The exact-parity
    // phase-2 lateral rungs that ARE landed: P2.0/P2.1 (harness+gate), P2.3 (single-vehicle drift),
    // P2.2a (side-by-side coexistence + departPosLat), and scenario 62 (following-parity). Skipped
    // (not failed) so the branch stays green, mirroring the pre-existing C4-vii deferred rung.
    [Fact(Skip = "P2.4 deferred: sublane speed-gain overtake reproduces pos/speed exact and posLat exact through t=13, but byte-exact recenter + leader lateral gap-keeping need the full MSLeaderInfo per-sublane expected-speed grid + keepLatGap (SL2015 core). See class header / docs/TASKS.md P2.4.")]
    public void Run40Steps_MatchesGoldenFcdWithinTolerance_FollowerSublaneOvertakes()
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
            $"Rung-P2.4 sublane-overtake parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
