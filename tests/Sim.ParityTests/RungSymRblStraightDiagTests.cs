using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Bug C (SYMMETRIC arrival-time right-of-way) -- diagnostic anchor, SKIP-GATED until the rung lands.
// scenarios/_diag/sym-rbl-straight is one single-lane right_before_left crossroads with four
// STRAIGHT-through vehicles (ns/sn/ew/we), one per approach, all departing t=0 from rest 100 m out --
// perfectly symmetric, arriving together. Distinct from scenario 26 (asymmetric 2-vehicle
// right-before-left, already green with a clear priority winner): here the four movements form a
// directed right-before-left CONFLICT CYCLE (each yields to the movement on its right) with NO clear
// winner.
//
// SUMO (golden): flows all four -- the N-S axis crosses first (t=17..20), then E-W (t=19..24); 0 stuck.
// ENGINE (baseline): DEADLOCKS -- all four drive onto the junction (t=17..19) and stop mid-junction,
// each blocking the next; none exits (4/4 permanently stuck). The willPass pre-pass resolves
// WillPass=false for all four (each braking-to-yield), so the crossing gate's
// `foeYieldsThisStep = !foe.WillPass` is true for every ego and all four proceed simultaneously.
//
// SUMO breaks the tie with an EXPLICIT right-before-left deadlock detector (MSVehicle.cpp:2818-2839):
// for a LINKSTATE_EQUAL link with waitingTime>0 it walks the getFirstApproachingFoe blocker chain, and
// if it wraps back to ego it aborts the request RANDOMLY (RandHelper::rand < 0.25 straight / 0.75 turn).
// Because that tie-break is RNG-keyed and the engine's RNG stream is its own (VehicleRng, never SUMO's
// per the C1/determinism policy), EXACT @1e-3 FCD parity is NOT achievable -- only stuck-count /
// statistical parity, so this test asserts stuck==0 (the _diag convention), not FCD parity.
// FIX (LANDED -- Engine.ResolveRightBeforeLeftCycles): a DETERMINISTIC, order-independent equivalent of
// SUMO's RNG abort -- detect the directed response cycle among approaching vehicles and select a maximal
// non-conflicting set greedily by ascending link index to pass, the rest yield. INERT wherever no cycle
// exists (whole committed suite byte-identical, Sim.Bench hash unchanged). Full diagnosis:
// scenarios/_diag/sym-rbl-straight/provenance.txt and C4-VII-REMAINING.md "#2".
public class RungSymRblStraightDiagTests
{
    private static readonly string Dir = System.IO.Path.Combine(
        RepoRoot(), "scenarios", "_diag", "sym-rbl-straight");

    [Fact]
    public void SymmetricRightBeforeLeft_AllFourClear_NotGridlocked()
    {
        var engine = new Engine();
        engine.LoadScenario(
            System.IO.Path.Combine(Dir, "net.net.xml"),
            System.IO.Path.Combine(Dir, "rou.rou.xml"),
            System.IO.Path.Combine(Dir, "config.sumocfg"));

        var traj = engine.Run(60);

        var last = new Dictionary<string, (double T, double Speed)>();
        var maxT = 0.0;
        foreach (var p in traj.AllPoints)
        {
            maxT = System.Math.Max(maxT, p.Time);
            last[p.VehicleId] = (p.Time, p.Speed);
        }

        var stuck = last.Count(kv => kv.Value.T >= maxT - 1 && kv.Value.Speed < 0.1);

        // SUMO: 0 stuck / 4 (all cleared). Engine WITH the tie-break fix: 0. Baseline: 4 (mid-junction
        // gridlock -- this assertion FAILS today, which is why the test is Skip-gated).
        Assert.Equal(0, stuck);
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
