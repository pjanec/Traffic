using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Laneless direction (docs/LANELESS-DIRECTION.md), Stage 2b-ii: the lateral FEASIBLE-INTERVAL solve
// correctly resolves CONFLICTING neighbours -- the case the Stage-2 push-sum could strand. One
// centred vehicle on a 7.2 m lane meets two agents at once: agent A blocks the CENTRE (forcing a
// lateral move) and agent B blocks the RIGHT side. A naive push-sum (A pushes ego right, B pushes ego
// left) can leave ego stuck in the middle, overlapping both. The feasible-interval solve instead finds
// the only open gap -- the LEFT -- and threads it. Asserted behaviourally (no golden): ego moves to
// the LEFT (not the default keep-right), never overlaps EITHER agent, and passes both.
//
// Byte-identity: LanelessRvo defaults false + agents injected only here, so goldens + hash unaffected.
public class RungRvoConflictingNeighborsTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "_fixtures", "rvo-agent");
    private const string Lane = "e0_0";
    private const double VehLength = 5.0;
    private const double VehHalfWidth = 0.9;
    private const double AgentHalfWidth = 0.4;   // width 0.8

    [Fact]
    public void RvoVehicle_BetweenConflictingAgents_FindsTheOpenGap_NoOverlapWithEither()
    {
        var engine = new Engine { LanelessRvo = true };
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        // A blocks the centre; B blocks the right (negative posLat). The only feasible clearance is LEFT.
        engine.AddObstacle("A", Lane, frontPos: 60.0, length: 1.0, latPos: 0.0, width: 2 * AgentHalfWidth);
        engine.AddObstacle("B", Lane, frontPos: 60.0, length: 1.0, latPos: -1.8, width: 2 * AgentHalfWidth);

        var traj = engine.Run(40);

        double signedPeakLeft = double.NegativeInfinity, lastPos = 0.0;
        var agents = new (string Id, double Front, double Lat)[] { ("A", 60.0, 0.0), ("B", 60.0, -1.8) };

        for (var t = 0; t <= 39; t++)
        {
            if (!traj.TryGet("v0", t, out var p))
            {
                continue;
            }

            signedPeakLeft = Math.Max(signedPeakLeft, p.PosLat);   // most-LEFT (positive) offset reached
            lastPos = p.Pos;

            foreach (var (id, front, lat) in agents)
            {
                var longOverlap = p.Pos - VehLength < front && front - 1.0 < p.Pos;
                if (longOverlap)
                {
                    Assert.True(Math.Abs(p.PosLat - lat) >= VehHalfWidth + AgentHalfWidth - 1e-6,
                        $"vehicle overlapped agent {id} at t={t}: lateral sep {Math.Abs(p.PosLat - lat):F3} " +
                        $"< {VehHalfWidth + AgentHalfWidth} (pos={p.Pos:F2}, posLat={p.PosLat:F3})");
                }
            }
        }

        // It resolved the conflict by going LEFT (the open side), not the default keep-right.
        Assert.True(signedPeakLeft > 1.0,
            $"vehicle did not thread the LEFT gap (max left posLat = {signedPeakLeft:F3}); a push-sum would strand it centred");
        Assert.True(lastPos > 60.0, $"vehicle did not get past the agents (ended at pos {lastPos:F1})");
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
