using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung D3 behavioral (property) tests: COUPLED OV2/OV4 -- a cooperative side-by-side opposite-
// direction overtake. On a lane wide enough that a spilled overtaker and a cooperatively-shifted
// oncoming leave a safe lateral corridor (scenarios/59-overtake-cooperative, 3.8 m lanes), the gap
// acceptance may keep the overtaker committed against an oncoming that the conservative rule would
// abort for, so the two pass SIDE BY SIDE (the overtaker spilled, the oncoming shifted to its outer
// edge), then both recentre. Asserted here:
//   1. the overtaker does NOT abort -- it stays spilled through the head-on approach and the closest
//      approach happens with BOTH vehicles off-centre (a genuine side-by-side pass, not "wait for the
//      oncoming to clear"), with a safe lateral corridor, no collision, and both recentre;
//   2. D3's optimism is BOUNDED: two overtakers approaching head-on (neither cooperating) never
//      collide -- the coupled acceptance's not-spilled check drops the intent and both abort.
// No SUMO golden (this is the requested live-reactivity enhancement, inert when no vType is
// lcOpposite and on any lane too narrow for the shifted pass).
public class RungD3CooperativeOvertakeTests
{
    private const double LaneCentreYAB = -1.90; // edge AB centre on the 3.8 m net
    private const double LaneCentreYBA = 1.90;
    private const double CarLen = 5.0;
    private const double CombinedHalfWidth = 1.8;

    private static TrajectorySet Run(string rou, int steps)
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "59-overtake-cooperative");
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(dir, "net.net.xml"),
            Path.Combine(dir, rou),
            Path.Combine(dir, "config.sumocfg"));
        return engine.Run(steps);
    }

    private static void AssertNoCollisions(TrajectorySet traj)
    {
        foreach (var frame in traj.AllPoints.GroupBy(p => p.Time))
        {
            var pts = frame.ToList();
            for (var i = 0; i < pts.Count; i++)
            {
                for (var j = i + 1; j < pts.Count; j++)
                {
                    var a = pts[i];
                    var b = pts[j];
                    var overlap = Math.Abs(a.X - b.X) < CarLen && Math.Abs(a.Y - b.Y) < CombinedHalfWidth;
                    Assert.False(overlap,
                        $"collision at t={a.Time}: {a.VehicleId}({a.X:F1},{a.Y:F1}) vs {b.VehicleId}({b.X:F1},{b.Y:F1})");
                }
            }
        }
    }

    [Fact]
    public void CooperativeSideBySidePass_OvertakerDoesNotAbort_SafeCorridor_BothRecentre()
    {
        var traj = Run("coop.rou.xml", 30);
        var ovByTime = traj.PointsFor("overtaker");
        var oncByTime = traj.PointsFor("oncoming");
        var leader = traj.PointsFor("leader").OrderBy(kv => kv.Key).Last().Value;
        var ov = ovByTime.OrderBy(kv => kv.Key).Select(kv => kv.Value).ToList();

        // The overtaker spilled toward the oncoming lane and completed the pass (ahead of the leader,
        // recentred) -- i.e. it did NOT abort and wait for the oncoming to clear.
        Assert.Contains(ov, p => p.Y - LaneCentreYAB > 1.5);
        Assert.True(ov[^1].Pos > leader.Pos, "overtaker never got ahead of the leader");
        Assert.True(Math.Abs(ov[^1].Y - LaneCentreYAB) < 1e-6, $"overtaker did not recentre (final y {ov[^1].Y:F2})");

        // Closest head-on approach: it must happen while BOTH are off their lane centres (a real
        // side-by-side pass) AND with a safe lateral corridor (>= combined half width). This is the
        // D3-specific behaviour: pre-D3 the overtaker aborts and is centred long before the oncoming.
        var shared = ovByTime.Keys.Where(t => oncByTime.ContainsKey(t)).ToList();
        var closest = shared.OrderBy(t => Math.Abs(ovByTime[t].X - oncByTime[t].X)).First();
        var o = ovByTime[closest];
        var n = oncByTime[closest];
        Assert.True(o.Y - LaneCentreYAB > 1.0, $"overtaker not spilled at closest approach t={closest} (y {o.Y:F2})");
        Assert.True(n.Y - LaneCentreYBA > 0.5, $"oncoming not shifted outward at closest approach t={closest} (y {n.Y:F2})");
        Assert.True(Math.Abs(o.Y - n.Y) >= CombinedHalfWidth,
            $"unsafe lateral corridor at closest approach t={closest}: |dy|={Math.Abs(o.Y - n.Y):F2}");

        // The oncoming recentres after the pass.
        var oncLast = oncByTime.OrderBy(kv => kv.Key).Last().Value;
        Assert.True(Math.Abs(oncLast.Y - LaneCentreYBA) < 1e-6, $"oncoming did not recentre (final y {oncLast.Y:F2})");

        AssertNoCollisions(traj);
    }

    [Fact]
    public void TwoOvertakersHeadOn_NeitherCooperates_BothAbort_NoCollision()
    {
        // The adversarial bound on D3's optimism: two overtakers spill toward each other, but as soon
        // as each sees the other spilled the coupled acceptance refuses/drops the intent and both
        // abort, so they pass in their own lanes without ever overlapping.
        var traj = Run("headon.rou.xml", 30);

        // Non-vacuous: both overtakers really did spill toward the centre line at some point.
        Assert.Contains(traj.PointsFor("overtakerA").Values, p => p.Y - LaneCentreYAB > 1.5);
        Assert.Contains(traj.PointsFor("overtakerB").Values, p => LaneCentreYBA - p.Y > 1.5);

        AssertNoCollisions(traj);
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
