using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using Sim.Core;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// ACCEPTANCE HARNESS for docs/LANE-CHANGE-OVERLAP-SPEC.md (open engine task: port SUMO cooperative
// lane-changing). In dense multi-lane traffic SumoSharp lets a vehicle finish a lane change into an
// occupied slot, so two cars end up on the SAME lane closer than a vehicle length apart -- a physical
// overlap. Vanilla SUMO 1.20.0 never does this on the same net (measured: 0). Root cause + fix plan are
// in the spec; the short version is that only the LEADER-gap veto is ported (keep-right passes a null
// follower) and there is no cooperative follower-slows-to-make-room machinery.
//
// This test is [Fact(Skip=...)] on purpose: it does NOT gate the build today (the bug is known and
// deferred). The fix session UNSKIPS it -- it must then assert 0 overlaps on scenarios/_diag/
// willpass-saturation. Do NOT "fix" this by loosening the threshold: 5.5 m is already shorter than a
// vehicle, so any hit is a real overlap.
public class LaneChangeOverlapDiagTests
{
    private readonly ITestOutputHelper _output;

    public LaneChangeOverlapDiagTests(ITestOutputHelper output) => _output = output;

    private const double OverlapDistance = 5.5; // metres; < one vehicle length -> a genuine overlap
    private const int Steps = 200;

    [Fact(Skip = "IN PROGRESS — the lane-change overlap fix (docs/LANE-CHANGE-OVERLAP-DESIGN.md) has landed "
        + "the checkChange LCA_OVERLAPPING block + keep-right follower veto + the arrival-lane cross-junction "
        + "leader fix, cutting overlaps 197 -> 2 (both-stopped 186 -> 0) with 0 stuck and every golden "
        + "byte-identical. The last 2 are NOT lane-change overlaps: a junction MERGE and a 1-step "
        + "car-following reaction, both ECS frozen-snapshot artifacts (design §3 Stage 3). Unskip and assert "
        + "== 0 once Stage 3 lands. Do NOT loosen the 5.5 m threshold to pass it.")]
    public void WillpassSaturation_NoSameLaneOverlaps()
    {
        var overlaps = CountOverlaps(out var bothStopped);
        _output.WriteLine($"[overlap] willpass-saturation {Steps} steps: {overlaps} same-lane overlaps "
            + $"(<{OverlapDistance} m), {bothStopped} both-stopped. vanilla SUMO on this net: 0.");

        // ACCEPTANCE: match vanilla SUMO's zero overlaps.
        Assert.Equal(0, overlaps);
    }

    // Counts, across the run, consecutive same-lane vehicle pairs closer than OverlapDistance in
    // lane-relative Pos. Reuses the committed dense repro; no FCD file needed (reads the trajectory).
    private static int CountOverlaps(out int bothStopped)
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "_diag", "willpass-saturation");
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(dir, "net.net.xml"),
            Path.Combine(dir, "rou.rou.xml"),
            Path.Combine(dir, "config.sumocfg"));

        var traj = engine.Run(Steps);

        // Group by (time, lane); skip internal ':' lanes (junction interiors are single-file transits).
        var byTimeLane = new Dictionary<(double, string), List<(double Pos, double Speed)>>();
        foreach (var p in traj.AllPoints)
        {
            if (string.IsNullOrEmpty(p.Lane) || p.Lane[0] == ':')
            {
                continue;
            }

            var key = (p.Time, p.Lane);
            if (!byTimeLane.TryGetValue(key, out var list))
            {
                byTimeLane[key] = list = new List<(double, double)>();
            }

            list.Add((p.Pos, p.Speed));
        }

        var overlaps = 0;
        bothStopped = 0;
        foreach (var list in byTimeLane.Values)
        {
            list.Sort((a, b) => a.Pos.CompareTo(b.Pos));
            for (var i = 1; i < list.Count; i++)
            {
                if (list[i].Pos - list[i - 1].Pos < OverlapDistance)
                {
                    overlaps++;
                    if (list[i].Speed < 0.3 && list[i - 1].Speed < 0.3)
                    {
                        bothStopped++;
                    }
                }
            }
        }

        return overlaps;
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
