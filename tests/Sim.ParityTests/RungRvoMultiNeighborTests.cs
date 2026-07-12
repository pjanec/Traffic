using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Laneless direction (docs/LANELESS-DIRECTION.md), Stage 2b: the RVO solve gathers ALL near
// footprint agents within a fixed radius from ONE query (Seam-1's phase-2 form), rather than the
// lane-structured single leader + single follower + separate obstacle loop. This test exercises the
// N-neighbour gather: a fast car overtakes a QUEUE of three slow cars on a 7.2 m lane, staying clear
// of every one of them (the old leader/follower pair could only see one leader at a time). Asserted
// behaviourally (no golden): the fast car never overlaps ANY of the three, drifts aside, passes the
// whole queue (ends ahead of all at free-flow), and recentres.
//
// Byte-identity: LanelessRvo defaults false, so committed goldens + hash are unaffected.
public class RungRvoMultiNeighborTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "_fixtures", "rvo-multi");
    private const double VehLength = 5.0;
    private const double TwoHalfWidths = 1.8;   // 0.9 + 0.9

    [Fact]
    public void RvoVehicle_OvertakesQueueOfThree_NoOverlapWithAny_Passes_Recenters()
    {
        var engine = new Engine { LanelessRvo = true };
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(50);
        var slow = new[] { "s0", "s1", "s2" };

        double peakLat = 0.0, lastLat = 0.0, lastFastPos = 0.0, lastFastSpeed = 0.0, maxSlowPos = 0.0;
        for (var t = 0; t <= 49; t++)
        {
            if (!traj.TryGet("f", t, out var f))
            {
                continue;
            }

            peakLat = Math.Max(peakLat, Math.Abs(f.PosLat));
            lastLat = f.PosLat;
            lastFastPos = f.Pos;
            lastFastSpeed = f.Speed;

            foreach (var id in slow)
            {
                if (!traj.TryGet(id, t, out var s))
                {
                    continue;
                }

                maxSlowPos = Math.Max(maxSlowPos, s.Pos);

                // No-overlap with EACH queued car: whenever they overlap longitudinally, their lateral
                // footprints must be disjoint.
                var longOverlap = f.Pos - VehLength < s.Pos && s.Pos - VehLength < f.Pos;
                if (longOverlap)
                {
                    Assert.True(Math.Abs(f.PosLat - s.PosLat) >= TwoHalfWidths - 1e-6,
                        $"fast car overlapped {id} at t={t}: lateral sep {Math.Abs(f.PosLat - s.PosLat):F3} " +
                        $"< {TwoHalfWidths} (f pos={f.Pos:F2} lat={f.PosLat:F3}, {id} pos={s.Pos:F2} lat={s.PosLat:F3})");
                }
            }
        }

        Assert.True(peakLat > 1.0, $"fast car never manoeuvred (peak |posLat| = {peakLat:F3})");
        Assert.True(lastFastPos > maxSlowPos, $"fast car did not pass the whole queue (ended pos {lastFastPos:F1}, frontmost slow {maxSlowPos:F1})");
        Assert.True(lastFastSpeed > 13.0, $"fast car did not reach free-flow (ended at {lastFastSpeed:F2})");
        Assert.True(Math.Abs(lastLat) < 0.2, $"fast car did not recentre (ended at posLat {lastLat:F3})");
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
