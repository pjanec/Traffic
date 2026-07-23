using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.LiveCity.Tests;

// THROWAWAY diagnostic probe (docs/ task: pedestrian junction-turn back-step). Not part of the
// permanent suite's assertions story -- it exists to trace one low-power ped through a real junction
// turn, exactly mirroring City3D's ProcessLiveCity / Raylib's RunLiveCity render-time construction
// (sim.Time + accumulator, fed into PedRemoteReconstructor.Pump), and report where a backward step
// appears (raw PoseAt vs the capped-correction smoothing layer on top).
public class PedBackstepProbeTests
{
    private readonly ITestOutputHelper _output;

    public PedBackstepProbeTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private static string RepoRoot()
    {
        try
        {
            var psi = new ProcessStartInfo("git", "rev-parse --show-toplevel")
            {
                RedirectStandardOutput = true,
                UseShellExecute = false,
                WorkingDirectory = AppContext.BaseDirectory,
            };
            using var proc = Process.Start(psi)!;
            var outp = proc.StandardOutput.ReadToEnd().Trim();
            proc.WaitForExit();
            if (proc.ExitCode == 0 && Directory.Exists(Path.Combine(outp, "scenarios")))
            {
                return outp;
            }
        }
        catch
        {
        }

        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null)
        {
            if (Directory.Exists(Path.Combine(dir.FullName, "scenarios")) && File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
            {
                return dir.FullName;
            }

            dir = dir.Parent;
        }

        throw new InvalidOperationException("could not resolve repo root");
    }

    [Fact]
    public void Probe_LowPowerPedThroughJunctionTurn_TraceBackSteps()
    {
        var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
        cfg.YieldEnabled = true;
        using var sim = new LiveCitySim(cfg);

        // Advance the coupled sim far enough to get a healthy population, exactly like LiveCitySimTests.
        for (var i = 0; i < 60; i++)
        {
            sim.Step();
        }

        // Find a candidate low-power ped whose ActivityTimeline contains >=2 Walk segments with a real
        // heading change between them (a "turn waypoint") -- i.e. multiple WalkSegments chained by
        // ActivityTimeline's own anchor-inheritance (junction corners show up as a heading kink either
        // WITHIN one WalkSegment's polyline, or AT a WalkSegment boundary). We inspect the underlying
        // ActivityTimeline objects directly (server-side truth) via the ped source wire's IG copy is not
        // exposed, so we use the reconstructor's own Ig after pumping.
        var recon = new PedRemoteReconstructor(sim.PedSource);

        // Continuous render clock, mirroring City3D Main.cs ProcessLiveCity / Sim.Viewer Program.cs
        // RunLiveCity EXACTLY: `_liveCitySource.Time + _liveCityAccumulator` / `sim.Time + accumWall`.
        // We drive the coupled sim's own Dt-paced ticks while advancing a wall-clock-like accumulator in
        // small sub-steps between ticks, then Pump the reconstructor with (simTime + accumulator) each
        // "frame" -- reproducing the real per-frame call pattern.
        const double renderFps = 60.0;
        const double renderDt = 1.0 / renderFps;
        var accumulator = 0.0;
        double simNow = sim.Time;

        int? targetId = null;
        var candidates = new List<(int Id, ActivityTimeline Timeline)>();

        // Drain what's already on the wire so recon.Ig has every spawned ped's timeline, then inspect.
        recon.Pump(sim.Time);
        foreach (var id in recon.KnownIds)
        {
            if (recon.Ig.ModelOf(id) != PedDrModel.ActivityTimeline)
            {
                continue;
            }

            // Reach into the private timeline via reflection-free approach: reconstruct segments by
            // sampling PoseAt at many times and looking for a heading discontinuity ("turn") within the
            // still-untraveled portion of the trip (now .. now+30s), so the probe below still has time to
            // observe it live.
            candidates.Add((id, default!)); // timeline object not exposed; will discover turns via PoseAt scan below.
        }

        // For every low-power candidate, scan its future PoseAt (raw, from HeadlessIg) for a heading
        // flip -- i.e. sample every 0.1s from now to now+40s, compute velocity-direction via finite
        // difference, and flag a big turn (dot(prevDir, nextDir) < 0.3, both segments actually walking).
        (int Id, double TurnTime)? found = null;
        foreach (var (id, _) in candidates)
        {
            Vec2? prevPos = null;
            Vec2? prevDir = null;
            for (var dtProbe = 0.0; dtProbe <= 40.0; dtProbe += 0.1)
            {
                var t = sim.Time + dtProbe;
                var sample = recon.Ig.ReconstructSample(id, t);
                if (prevPos is { } pp)
                {
                    var disp = sample.Pos - pp;
                    if (disp.Abs > 1e-6)
                    {
                        var dir = disp / disp.Abs;
                        if (prevDir is { } pd && pd.Abs > 1e-6)
                        {
                            var dot = (pd.X * dir.X) + (pd.Y * dir.Y);
                            if (dot < 0.3 && dtProbe > 1.0)
                            {
                                found = (id, t);
                                break;
                            }
                        }

                        prevDir = dir;
                    }
                }

                prevPos = sample.Pos;
            }

            if (found is not null)
            {
                targetId = id;
                break;
            }
        }

        Assert.True(targetId is not null, "no low-power ped with an upcoming turn was found in this population -- widen the search.");
        var turnApproxTime = found!.Value.TurnTime;
        _output.WriteLine($"PROBE: tracking ped id={targetId} through a turn near renderTime~{turnApproxTime:F2} (sim.Time={sim.Time:F2})");

        // Now actually run the render loop for real: advance sim.Step() on cfg.Dt ticks, and every
        // "render frame" (renderDt) call recon.Pump(simTime + accumulator) then TryGetRenderPose, EXACTLY
        // like ProcessLiveCity/RunLiveCity. Keep going until well past the observed turn or the ped
        // despawns.
        var rows = new List<(double RenderTime, Vec2 Smoothed, Vec2 Raw, Vec2 Heading, bool BackStep, double Disp, PedDrModel Model)>();
        Vec2? lastSmoothed = null;
        Vec2? lastRawHeadingDir = null;

        var endTime = turnApproxTime + 8.0;
        var safetyFrames = 0;
        while (simNow < endTime && safetyFrames < 20000)
        {
            safetyFrames++;
            // Advance the coupled sim on its own Dt ticks (mirrors ProcessLiveCity's accumulator loop).
            accumulator += renderDt;
            while (accumulator >= cfg.Dt)
            {
                sim.Step();
                accumulator -= cfg.Dt;
                simNow = sim.Time;
            }

            var pedNow = simNow + accumulator;
            recon.Pump(pedNow);

            if (!recon.TryGetRenderPose(targetId.Value, out var smoothedPos, out var visible, out _))
            {
                break; // despawned
            }

            var rawSample = recon.Ig.Knows(targetId.Value) ? recon.Ig.ReconstructSample(targetId.Value, recon.RenderTime) : default;

            Vec2 heading;
            bool backStep = false;
            double disp = 0.0;
            if (lastSmoothed is { } prev)
            {
                var d = smoothedPos - prev;
                disp = d.Abs;
                heading = lastRawHeadingDir ?? Vec2.Zero;
                if (heading.Abs > 1e-9 && d.Abs > 1e-9)
                {
                    var dot = (heading.X * d.X) + (heading.Y * d.Y);
                    if (dot < -1e-6)
                    {
                        backStep = true;
                    }
                }
            }
            else
            {
                heading = Vec2.Zero;
            }

            // Track a smoothed "current heading" direction off the RAW sample's own velocity if
            // available (approx via recent raw displacement), else off the last smoothed displacement.
            if (lastSmoothed is { } p2)
            {
                var dd = smoothedPos - p2;
                if (dd.Abs > 1e-9)
                {
                    lastRawHeadingDir = dd / dd.Abs;
                }
            }

            var model = recon.Ig.Knows(targetId.Value) ? recon.Ig.ModelOf(targetId.Value) : PedDrModel.PathArc;
            rows.Add((recon.RenderTime, smoothedPos, rawSample.Pos, heading, backStep, disp, model));
            lastSmoothed = smoothedPos;
        }

        // Report.
        var backSteps = rows.Where(r => r.BackStep).ToList();
        _output.WriteLine($"PROBE: total frames={rows.Count}, back-step frames={backSteps.Count}");
        foreach (var r in backSteps)
        {
            _output.WriteLine($"  BACKSTEP renderTime={r.RenderTime:F3} smoothed=({r.Smoothed.X:F3},{r.Smoothed.Y:F3}) raw=({r.Raw.X:F3},{r.Raw.Y:F3}) disp={r.Disp:F4}");
        }

        // Dump a window around the first backstep (or the whole trace if none) for inspection.
        var firstBad = backSteps.Count > 0 ? rows.IndexOf(backSteps[0]) : -1;
        var startIdx = firstBad >= 0 ? Math.Max(0, firstBad - 30) : 0;
        var endIdx = firstBad >= 0 ? Math.Min(rows.Count - 1, firstBad + 30) : Math.Min(rows.Count - 1, 40);
        _output.WriteLine("PROBE window:");
        for (var i = startIdx; i <= endIdx; i++)
        {
            var r = rows[i];
            _output.WriteLine($"  [{i}] t={r.RenderTime:F3} model={r.Model} smoothed=({r.Smoothed.X:F4},{r.Smoothed.Y:F4}) raw=({r.Raw.X:F4},{r.Raw.Y:F4}) rawEqualsSmoothed={(r.Smoothed - r.Raw).Abs < 1e-9} disp={r.Disp:F5} back={r.BackStep}");
        }

        // Also directly diff raw vs smoothed at each backstep frame to attribute cause.
        _output.WriteLine("PROBE attribution (raw-vs-smoothed at backstep frames):");
        foreach (var r in backSteps)
        {
            var rawSmoothedGap = (r.Smoothed - r.Raw).Abs;
            _output.WriteLine($"  t={r.RenderTime:F3} |smoothed-raw|={rawSmoothedGap:F5}  => {(rawSmoothedGap > 1e-6 ? "SMOOTHING differs from raw at this frame" : "raw==smoothed here (raw itself moved backward, or first-sight snap)")}");
        }
    }

    // AFTER-FIX regression pin: the EXACT same ped id and corner (id=1, corner near renderTime~56.35,
    // sim.Time=30 seed) as the diagnostic trace that originally localized this bug (see the class remarks'
    // BEFORE-fix numbers: a 0.235 m raw jump in a single 16.7 ms frame at t=56.350). Forces targetId=1 (no
    // auto-search, since the auto-search's own turn-detection heuristic shifts slightly once the fix changes
    // pose values) and traces the SAME window for a clean before/after comparison.
    [Fact]
    public void Probe_Ped1ThroughItsKnownJunctionTurn_AfterFix_NoBackSteps()
    {
        var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
        cfg.YieldEnabled = true;
        using var sim = new LiveCitySim(cfg);

        for (var i = 0; i < 60; i++)
        {
            sim.Step();
        }

        var recon = new PedRemoteReconstructor(sim.PedSource);
        const double renderFps = 60.0;
        const double renderDt = 1.0 / renderFps;
        var accumulator = 0.0;
        double simNow = sim.Time;

        const int targetId = 1;
        var rows = new List<(double RenderTime, Vec2 Smoothed, Vec2 Raw, bool BackStep, double Disp)>();
        Vec2? lastSmoothed = null;
        Vec2? lastDir = null;

        var endTime = 64.0; // comfortably past the known ~56.35 corner
        var safetyFrames = 0;
        while (simNow < endTime && safetyFrames < 20000)
        {
            safetyFrames++;
            accumulator += renderDt;
            while (accumulator >= cfg.Dt)
            {
                sim.Step();
                accumulator -= cfg.Dt;
                simNow = sim.Time;
            }

            var pedNow = simNow + accumulator;
            recon.Pump(pedNow);

            if (!recon.TryGetRenderPose(targetId, out var smoothedPos, out _, out _))
            {
                break;
            }

            var rawSample = recon.Ig.Knows(targetId) ? recon.Ig.ReconstructSample(targetId, recon.RenderTime) : default;

            bool backStep = false;
            double disp = 0.0;
            if (lastSmoothed is { } prev)
            {
                var d = smoothedPos - prev;
                disp = d.Abs;
                if (lastDir is { } dir && dir.Abs > 1e-9 && d.Abs > 1e-9)
                {
                    var dot = (dir.X * d.X) + (dir.Y * d.Y);
                    if (dot < -1e-6)
                    {
                        backStep = true;
                    }
                }

                if (d.Abs > 1e-9)
                {
                    lastDir = d / d.Abs;
                }
            }

            rows.Add((recon.RenderTime, smoothedPos, rawSample.Pos, backStep, disp));
            lastSmoothed = smoothedPos;
        }

        var backSteps = rows.Where(r => r.BackStep).ToList();
        var maxBackDisp = backSteps.Count > 0 ? backSteps.Max(r => r.Disp) : 0.0;
        _output.WriteLine($"AFTER-FIX: ped id=1, frames={rows.Count}, back-step count={backSteps.Count}, max backward displacement={maxBackDisp:F5}");
        foreach (var r in rows.Where(r => r.RenderTime >= 56.1 && r.RenderTime <= 56.7))
        {
            _output.WriteLine($"  t={r.RenderTime:F3} smoothed=({r.Smoothed.X:F4},{r.Smoothed.Y:F4}) raw=({r.Raw.X:F4},{r.Raw.Y:F4}) disp={r.Disp:F5} back={r.BackStep}");
        }

        Assert.Empty(backSteps);
    }
}
