using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// C4-vii-a PART 1 (cont-link internal via-chain SEQUENCE) -- diagnostic anchor. scenarios/_diag/
// cont-turn-sequence is a LONE left-turner (vN: NC->CE) on the scenarios/44 symmetric 2-lane priority
// crossroads. NC->CE is a `cont` link: the turn is split by an INTERNAL JUNCTION into TWO internal
// lanes -- NC_1 -> :C_3_0 -> :C_16_0 -> CE_1 (SUMO reference lane order, verified). Only :C_16_0 is the
// lane in junction C's <request>/IntLanes for this link; :C_3_0 is the intermediate internal lane
// before the internal junction :C_16.
//
// THE BUG this pins (baseline 8bd4beb): NetworkModel.ResolveSequenceCore followed only the FIRST via
// (:C_3_0) listed on the NC->CE connection and jumped straight to the exit edge CE, COLLAPSING the
// path to NC_1 -> CE_1 and dropping :C_16_0. Because :C_16_0 is the junction link's own internal lane,
// JunctionYieldConstraint's egoInternalLane scan (which keys off LinkByInternalLane) could not even
// find ego's link, so no cautious approach / RoW could ever fire for a cont turn. FIX: follow the full
// internal via-chain (:C_3_0 then :C_16_0) so the pool includes every internal lane crossed -- SUMO's
// getViaLaneOrLane / MSLane internal-following chain. Inert for single-internal-lane junctions (every
// committed green scenario + the -L2 diag grids; suite byte-identical, Sim.Bench hash unchanged).
//
// SCOPE: this asserts only the internal-lane SEQUENCE (that the engine now OCCUPIES :C_16_0), NOT exact
// FCD -- the cont-turn SPEED still diverges (the cautious-approach relocation onto the internal
// junction :C_16's conflict zone, the turn-speed cap, and :C_16's own RoW/foes are the remaining
// C4-vii-a parts, tracked in TASKS.md; scenario 44 stays skip-gated until they land). :C_3_0 is
// crossed but too short (5 m) to be FCD-sampled at the engine's approach speed, so only :C_16_0
// (14.3 m) is asserted.
public class ContTurnSequenceDiagTests
{
    private static readonly string Dir = System.IO.Path.Combine(
        RepoRoot(), "scenarios", "_diag", "cont-turn-sequence");

    [Fact]
    public void ContTurn_TraversesInternalJunctionLane()
    {
        var engine = new Engine();
        engine.LoadScenario(
            System.IO.Path.Combine(Dir, "net.net.xml"),
            System.IO.Path.Combine(Dir, "rou.rou.xml"),
            System.IO.Path.Combine(Dir, "config.sumocfg"));

        var traj = engine.Run(45);

        var lanes = new HashSet<string>();
        var arrived = false;
        var maxT = 0.0;
        foreach (var p in traj.AllPoints)
        {
            lanes.Add(p.Lane);
            maxT = System.Math.Max(maxT, p.Time);
        }

        // vN must OCCUPY the internal-junction lane :C_16_0 -- before the via-chain fix the engine
        // collapsed the turn to NC_1 -> CE_1 and never entered it (SUMO: NC_1 :C_3_0 :C_16_0 CE_1).
        Assert.True(lanes.Contains(":C_16_0"),
            $"cont turn collapsed: vN never occupied :C_16_0. Visited: {string.Join(",", lanes.OrderBy(x => x))}");

        // And it still completes the turn onto the exit edge (the collapse fix must not strand it).
        var last = traj.AllPoints.Where(p => p.VehicleId == "vN").OrderBy(p => p.Time).Last();
        arrived = last.Lane == "CE_1" || !traj.AllPoints.Any(p => p.VehicleId == "vN" && p.Time >= maxT - 0.5);
        Assert.True(arrived, $"vN did not complete the turn (last lane {last.Lane} @ t={last.Time}).");
    }

    // C4-vii-a PART 2 (cont-turn SPEED) -- EXACT @1e-3 pos/speed parity. SUMO cautiously brakes the
    // minor `cont` turn (golden dips 13.89 -> 10.036 -> 5.536 on :C_3_0 -> 8.136 on :C_16_0 -> 9.260);
    // the baseline engine measured the minor-link cautious-approach `seen` from the wrong (intermediate
    // internal) lane, so it never braked and entered :C_3_0 at ~9.26. FIX (Engine.cs 2a): when the
    // approach lane is itself internal (the cont-turn signature), measure `seen` to the junction-link
    // internal lane :C_16_0 through the intermediate internal lane. Reproduces the golden EXACTLY.
    // See scenarios/_diag/cont-turn-sequence/provenance.txt.
    [Fact]
    public void ContTurn_SpeedMatchesGoldenExact()
    {
        var engine = new Engine();
        engine.LoadScenario(
            System.IO.Path.Combine(Dir, "net.net.xml"),
            System.IO.Path.Combine(Dir, "rou.rou.xml"),
            System.IO.Path.Combine(Dir, "config.sumocfg"));

        var actual = engine.Run(45);
        var golden = FcdParser.Parse(System.IO.Path.Combine(Dir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(System.IO.Path.Combine(Dir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch,
            $"cont-turn speed parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}");
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
