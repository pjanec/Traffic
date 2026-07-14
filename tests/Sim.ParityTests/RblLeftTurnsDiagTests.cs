using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Bug C, the "hold" arm -- behavioral regression for the right-before-left LEFT-TURN conflict cycle.
// scenarios/_diag/rbl-left-turns is one single-lane right_before_left crossroads with four LEFT-turning
// vehicles (nl/el/sl/wl), one per approach, all departing t=0 from rest 100 m out. netconvert emits a
// COMPLETE (K4) conflict graph -- every pair conflicts, so only one may cross at a time -- while the
// right-before-left response forms a rock-paper-scissors CYCLE (nl>el>sl>nl) with no global winner.
//
// Distinct from sym-rbl-straight (the STRAIGHT version), which already resolved with only the
// foe-relative crossing gate: there the resolver grants BOTH opposite-axis movements the pass at once,
// and they cover the "right" of both yielders. Here only ONE movement can pass, so two of the three
// yielders yield to foes that are THEMSELVES yielders (WillPass=false) and, pre-fix, saw no passing foe
// -- they drove onto the junction and locked mid-box (the user-reported "all cars stuck at junctions
// with no apparent blocker" on a TL-less grid; all 4/4 permanently stuck, no physical blocker).
//
// FIX (VehicleRuntime.JunctionCycleHold + JunctionYieldConstraint hold arm):
// Engine.ResolveRightBeforeLeftCycles marks every NON-selected cycle member JunctionCycleHold=true; the
// real-pass JunctionYieldConstraint then stops it AT the junction stop line before it commits onto its
// internal lane -- the deterministic analogue of SUMO's RNG abort clearing mySetRequest (MSVehicle.cpp:
// 2818-2839). The resolver re-runs each step and admits the next lowest-index cycle member, staggering
// the four movements out one at a time. INERT wherever no cycle exists (committed suite byte-identical,
// Sim.Bench hash 909605E965BFFE59 unchanged).
//
// This is a BEHAVIORAL assertion (stuck==0), NOT FCD parity: SUMO's tie-break is RNG-keyed and the
// engine's RNG stream is its own (VehicleRng, never SUMO's), so exact @1e-3 parity is unachievable --
// only stuck-count parity. Both engines break the deadlock and clear all four with 0 stuck and no
// teleport; the exact order/timing differs by design. See the scenario's provenance.txt.
public class RblLeftTurnsDiagTests
{
    private static readonly string Dir = System.IO.Path.Combine(
        RepoRoot(), "scenarios", "_diag", "rbl-left-turns");

    [Fact]
    public void FourLeftTurns_RightBeforeLeftCycle_AllClear_NotGridlocked()
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

        // All four must actually appear (guards against a "0 stuck because 0 departed" false pass).
        Assert.Equal(4, last.Count);

        var stuck = last.Count(kv => kv.Value.T >= maxT - 1 && kv.Value.Speed < 0.1);

        // SUMO: 0 stuck / 4 (all cleared, no teleport). Engine WITH the hold fix: 0. Baseline (pre-fix):
        // all four locked mid-junction and none exited -- this assertion FAILED before the fix.
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
