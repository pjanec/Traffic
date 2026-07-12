using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Laneless direction PoC (docs/LANELESS-DIRECTION.md): the opt-in continuous footprint / velocity-
// obstacle (RVO-lite) lateral layer, validated BEHAVIOURALLY (no golden), not byte-exact. Reuses
// scenarios/63-sublane-overtake-wide's inputs (one 7.2 m lane; a fast follower behind a slow leader,
// both centred) but runs with Engine.LanelessRvo = true: the follower should EMERGE an overtake from
// local avoidance -- drift laterally to clear the slower leader by minGapLat, accelerate past via the
// existing !FootprintsOverlap leader bypass, then recentre. We assert the properties that matter for
// a laneless model, NOT SUMO's exact posLat/timing:
//   (1) no-overlap: the two footprints never intersect (whenever they overlap longitudinally, their
//       lateral separation is >= the sum of half-widths);
//   (2) overtake-completes: the fast follower ends AHEAD of the slow leader and reaches free-flow
//       (13.89), which is impossible without the lateral manoeuvre (centred it would settle at 5);
//   (3) it actually manoeuvred laterally (peak |posLat| well off centre) and then recentred (~0).
//
// Byte-identity is covered elsewhere: LanelessRvo defaults false, so every committed golden (incl.
// the exact sublane rungs 60/61/62) and the determinism hash are unaffected (the full suite proves
// this). This test is the ONLY place the flag is set.
public class RungRvoLateralPocTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "63-sublane-overtake-wide");

    // Passenger defaults (VTypeDefaults): length 5.0 m, width 1.8 m.
    private const double VehLength = 5.0;
    private const double HalfWidth = 0.9;

    [Fact]
    public void RvoLateral_EmergentOvertake_NoOverlap_Completes_Recenters()
    {
        var engine = new Engine { LanelessRvo = true };
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(40);

        double peakLat = 0.0, peakV0Lat = 0.0;
        double lastV1Lat = 0.0, lastV1Pos = 0.0, lastV0Pos = 0.0, lastV1Speed = 0.0, lastV0Lat = 0.0;
        var sawBoth = false;

        for (var t = 0; t <= 39; t++)
        {
            var haveV0 = traj.TryGet("v0", t, out var v0);
            var haveV1 = traj.TryGet("v1", t, out var v1);
            if (!haveV1)
            {
                continue;
            }

            peakLat = Math.Max(peakLat, Math.Abs(v1.PosLat));
            lastV1Lat = v1.PosLat;
            lastV1Pos = v1.Pos;
            lastV1Speed = v1.Speed;

            if (haveV0)
            {
                sawBoth = true;
                lastV0Pos = v0.Pos;
                lastV0Lat = v0.PosLat;
                peakV0Lat = Math.Max(peakV0Lat, Math.Abs(v0.PosLat));

                // Footprints (pos is the FRONT): longitudinal [pos-length, pos]; lateral [posLat ± half].
                var longOverlap = v0.Pos - VehLength < v1.Pos && v1.Pos - VehLength < v0.Pos;
                var latSeparation = Math.Abs(v0.PosLat - v1.PosLat);
                if (longOverlap)
                {
                    Assert.True(latSeparation >= 2 * HalfWidth - 1e-6,
                        $"footprint overlap at t={t}: longitudinal overlap AND lateral separation " +
                        $"{latSeparation:F3} < {2 * HalfWidth} (v0 pos={v0.Pos:F2} lat={v0.PosLat:F3}, " +
                        $"v1 pos={v1.Pos:F2} lat={v1.PosLat:F3})");
                }
            }
        }

        Assert.True(sawBoth, "expected both vehicles present together at some step");
        // (2b) ONE-SIDED clearing (Stage 2b-ii feasible-interval): the overtaker clears the slower
        // leader FULLY by minGapLat, so the leader holds its line -- it only steps aside if the overtaker
        // cannot clear in time. (Stage 2's push-sum produced a small emergent leader wiggle, but it could
        // strand ego between conflicting neighbours; 2b-ii trades that cosmetic wiggle for correct
        // conflict resolution -- see RvoConflictingNeighbors below. Reciprocal SHARING is the open-space
        // full-ORCA layer, docs/LANELESS-DIRECTION.md.) Assert the leader stays near its line.
        Assert.True(peakV0Lat < 0.2, $"leader should hold its line under the one-sided feasible-interval solve (peak |posLat| = {peakV0Lat:F3})");
        Assert.True(Math.Abs(lastV0Lat) < 0.2, $"leader did not stay centred (ended at posLat {lastV0Lat:F3})");
        // (2) overtake completed: fast follower ended ahead of the slow leader, at free-flow.
        Assert.True(lastV1Pos > lastV0Pos, $"follower did not pass leader (v1 {lastV1Pos:F1} <= v0 {lastV0Pos:F1})");
        Assert.True(lastV1Speed > 13.0, $"follower did not reach free-flow (ended at {lastV1Speed:F2}, would be ~5 if blocked)");
        // (3) it actually manoeuvred laterally, then recentred.
        Assert.True(peakLat > 1.0, $"follower never manoeuvred laterally (peak |posLat| = {peakLat:F3})");
        Assert.True(Math.Abs(lastV1Lat) < 0.2, $"follower did not recentre (ended at posLat {lastV1Lat:F3})");
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
