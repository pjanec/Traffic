using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// C4-vii-c CRASH REGRESSION (diagnostic, not an exact-parity anchor). scenarios/_diag/
// multilane-internal-lc-crash is a committed minimal 3x3 -L2 priority grid (10 duarouter trips) that
// deterministically reproduces the strategic-lane-change-on-an-internal-edge crash the convergence
// fix (Engine.TryReResolveFromActualLane) EXPOSED:
//
//   Before the internal-lane guard, a vehicle that stopped/transited a MULTI-LANE junction's internal
//   lane (a 2-lane ':'-edge, e.g. :A0_2) while its route pool wanted the sibling internal lane reached
//   DecideSpeedGainChanges' strategic-LC path -> ComputeBestLanes(route, ":A0_2") -> throws
//   "Edge ':A0_2' is not part of the given route" (an internal edge is never on an edge-only route).
//   The convergence fix rescues vehicles that used to CLAMP at a lane end (never entering the
//   junction); they now proceed onto the internal lane, so this latent crash became reachable on any
//   dense -L2 net. FIX: skip ALL lane-change decisions (keep-right/strategic/speed-gain) on internal
//   lanes -- SUMO's MSLaneChanger runs only on normal edges (Engine.DecideSpeedGainChanges guard,
//   mirroring commit eac0a5b's keep-right guard hoisted to cover the strategic path).
//
// Regen recipe (network side): netgenerate --grid --grid.number=3 --grid.length=200 -L 2
// --no-turnarounds --seed 7 ; randomTrips.py -n net -e 40 -p 4 --fringe-factor 5 --min-distance 300
// --seed 7 ; duarouter --named-routes --ignore-errors. SUMO runs this net at free flow (0 of 10 stuck).
public class MultilaneInternalLcCrashDiagTests
{
    private static readonly string Dir = System.IO.Path.Combine(
        RepoRoot(), "scenarios", "_diag", "multilane-internal-lc-crash");

    [Fact]
    public void Grid_RunsWithoutThrowing_AndFlows()
    {
        var engine = new Engine();
        engine.LoadScenario(
            System.IO.Path.Combine(Dir, "net.net.xml"),
            System.IO.Path.Combine(Dir, "rou.rou.xml"),
            System.IO.Path.Combine(Dir, "config.sumocfg"));

        // Must not throw. Without the internal-lane guard this threw "Edge ':A0_2' is not part of the
        // given route" once a vehicle reached a multi-lane junction interior with a pending strategic
        // lane change.
        var traj = engine.Run(300);

        var last = new Dictionary<string, (double T, double Speed)>();
        var maxT = 0.0;
        foreach (var p in traj.AllPoints)
        {
            maxT = System.Math.Max(maxT, p.Time);
            last[p.VehicleId] = (p.Time, p.Speed);
        }

        var stuck = last.Count(kv => kv.Value.T >= maxT - 1 && kv.Value.Speed < 0.1);

        // SUMO: 0 stuck / 10; engine (post-fix): 0. A tiny margin absorbs a vehicle legitimately still
        // in transit at the run end without letting a real gridlock regression slip through.
        Assert.True(stuck <= 1, $"multilane-internal-lc-crash: {stuck} stuck (post-fix baseline 0, SUMO 0).");
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
