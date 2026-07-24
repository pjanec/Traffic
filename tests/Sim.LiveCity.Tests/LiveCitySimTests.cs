using System;
using System.Diagnostics;
using System.IO;
using Sim.LiveCity;
using Xunit;

namespace Sim.LiveCity.Tests;

public class LiveCitySimTests
{
    // Resolve the repo root the same way CLAUDE.md prescribes ("git rev-parse --show-toplevel"), with a
    // walk-up-from-AppContext.BaseDirectory fallback for an environment without git on PATH.
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
            var output = proc.StandardOutput.ReadToEnd().Trim();
            proc.WaitForExit();
            if (proc.ExitCode == 0 && Directory.Exists(Path.Combine(output, "scenarios")))
            {
                return output;
            }
        }
        catch
        {
            // fall through to the walk-up fallback
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

        throw new InvalidOperationException("could not resolve the SumoSharp repo root.");
    }

    private static LiveCityConfig MakeConfig(bool yield = true, double? dt = null)
    {
        var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
        cfg.YieldEnabled = yield;
        if (dt is { } d)
        {
            cfg.Dt = d;
        }

        return cfg;
    }

    [Fact]
    public void CoupledSim_OverAFewMinutes_ProducesCarsPedsAndYieldEvents()
    {
        using var sim = new LiveCitySim(MakeConfig(yield: true));

        for (var i = 0; i < 120; i++)
        {
            sim.Step();
        }

        Assert.True(sim.PeakCars > 0, $"expected PeakCars > 0, got {sim.PeakCars}");
        Assert.True(sim.PeakPeds > 0, $"expected PeakPeds > 0, got {sim.PeakPeds}");
        Assert.True(sim.PeakOccupiedCrossings > 0, $"expected PeakOccupiedCrossings > 0, got {sim.PeakOccupiedCrossings}");
        Assert.True(sim.CarYieldObservations > 0, $"expected CarYieldObservations > 0, got {sim.CarYieldObservations}");

        // Wire non-vacuousness: pump both sources and assert something real arrived.
        sim.VehicleSource.Pump();
        Assert.True(sim.VehicleSource.History.Count > 0, "expected >=1 vehicle in the replicated History");

        sim.PedSource.Pump();
        Assert.True(sim.PedSource.LatestCrowdFrame.Count > 0, "expected >=1 ped in the latest crowd frame");
    }

    [Fact]
    public void TwoRuns_SameConfig_AreByteExactDeterministic()
    {
        using var simA = new LiveCitySim(MakeConfig(yield: true));
        using var simB = new LiveCitySim(MakeConfig(yield: true));

        for (var step = 0; step < 120; step++)
        {
            simA.Step();
            simB.Step();

            var snapA = simA.Sample();
            var snapB = simB.Sample();

            Assert.Equal(snapA.Cars.Count, snapB.Cars.Count);
            for (var i = 0; i < snapA.Cars.Count; i++)
            {
                var a = snapA.Cars[i];
                var b = snapB.Cars[i];
                Assert.Equal(a.Handle, b.Handle);
                Assert.Equal(a.X, b.X);
                Assert.Equal(a.Y, b.Y);
                Assert.Equal(a.Z, b.Z);
                Assert.Equal(a.AngleDeg, b.AngleDeg);
            }

            Assert.Equal(snapA.Peds.Count, snapB.Peds.Count);
            for (var i = 0; i < snapA.Peds.Count; i++)
            {
                var a = snapA.Peds[i];
                var b = snapB.Peds[i];
                Assert.Equal(a.Id, b.Id);
                Assert.Equal(a.X, b.X);
                Assert.Equal(a.Y, b.Y);
                Assert.Equal(a.Regime, b.Regime);
            }
        }
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task), deliverable 1: the SAME determinism proof as
    // TwoRuns_SameConfig_AreByteExactDeterministic above, but at Dt=0.1 (10 Hz, cfg.SimHz's non-default
    // side) instead of the 0.5 (2 Hz) default -- proves LiveCityConfig.Dt/SimHz plumbs all the way through
    // to LiveCitySim's engine step-length (via the InvariantCulture-formatted config XML) and the ped
    // demand's stepDt without breaking either the coupled sim's liveness (cars>0 && peds>0) or its
    // byte-exact determinism (same seed+Dt => identical run).
    [Fact]
    public void TwoRuns_AtTenHz_AreByteExactDeterministic_AndProduceCarsAndPeds()
    {
        using var simA = new LiveCitySim(MakeConfig(yield: true, dt: 0.1));
        using var simB = new LiveCitySim(MakeConfig(yield: true, dt: 0.1));

        for (var step = 0; step < 120; step++)
        {
            simA.Step();
            simB.Step();

            var snapA = simA.Sample();
            var snapB = simB.Sample();

            Assert.Equal(snapA.Cars.Count, snapB.Cars.Count);
            for (var i = 0; i < snapA.Cars.Count; i++)
            {
                var a = snapA.Cars[i];
                var b = snapB.Cars[i];
                Assert.Equal(a.Handle, b.Handle);
                Assert.Equal(a.X, b.X);
                Assert.Equal(a.Y, b.Y);
                Assert.Equal(a.Z, b.Z);
                Assert.Equal(a.AngleDeg, b.AngleDeg);
            }

            Assert.Equal(snapA.Peds.Count, snapB.Peds.Count);
            for (var i = 0; i < snapA.Peds.Count; i++)
            {
                var a = snapA.Peds[i];
                var b = snapB.Peds[i];
                Assert.Equal(a.Id, b.Id);
                Assert.Equal(a.X, b.X);
                Assert.Equal(a.Y, b.Y);
                Assert.Equal(a.Regime, b.Regime);
            }
        }

        Assert.True(simA.PeakCars > 0, $"expected PeakCars > 0 at Dt=0.1, got {simA.PeakCars}");
        Assert.True(simA.PeakPeds > 0, $"expected PeakPeds > 0 at Dt=0.1, got {simA.PeakPeds}");
    }

    // #15 LIVENESS / THROUGHPUT regression guard (docs/LIVE-CITY-15-RESUME.md §2 item 3).
    // Fixture + run/re-baseline instructions: scenarios/_ped/demo_city/box/README.md (the committed dataset
    // this and several other tests pin -- change it only deliberately, re-baseline all dependants together).
    // The parity gate structurally CANNOT catch the #15 junction-gridlock class of bug: every #15 fix is
    // demo-gated and INERT on every golden, so a change that silently reforms the dense-flow gridlock passes
    // parity byte-for-byte. This test is the missing guard: it runs the coupled live-city sim ~1000 s
    // headless (no SUMO, committed demo inputs) with the shipped dense-flow config PINNED (immune to stray
    // LIVECITY_* env vars) and asserts the sim keeps DISCHARGING -- arrivals keep climbing to the end and the
    // late stopped fraction never pins near 1.0. It is deterministic (same seed+config => identical run, see
    // TwoRuns_SameConfig_AreByteExactDeterministic), so the thresholds are stable, not statistical.
    //
    // Measured separation (this branch, first-hand) at 2000 steps = 1000 s:
    //   healthy (shipped)      : final arrivals ~736, last-400-step growth ~+145, late stoppedFrac avg ~0.35
    //   a #15 gridlock regress : final arrivals ~361 (flatlined by t~900), last-window growth ~+2, frac ~1.0
    // The thresholds below sit with wide margin on BOTH sides of that gap, so healthy flow never flakes red
    // while any gridlock regression (the arrivals flatline is the sharpest signal) trips it.
    [Fact]
    public void DenseFlow_OverAThousandSeconds_KeepsDischarging_NoGridlock()
    {
        var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
        // Pin the scenario so the assertion is about ENGINE behaviour, not config/env drift: dense-flow
        // demand (160 cars), teleport OFF (a teleport would mask a jam by removing stuck cars), 2 Hz, and the
        // shipped #15 dense-flow knobs explicitly ON (cooperative LC + the into-occupied vetoes).
        cfg.CarTargetConcurrent = 160;
        cfg.TimeToTeleportSeconds = 0.0;
        cfg.Dt = 0.5;
        cfg.YieldEnabled = true;
        cfg.CooperativeLaneChange = true;
        cfg.MergeStoppedMinGap = 5.0;
        cfg.MergeStoppedStrategicDeferDist = 15.0;

        using var sim = new LiveCitySim(cfg);

        const int totalSteps = 2000;       // 1000 s at dt=0.5 -- long enough that a gridlock has fully set in
        const int lateWindow = 400;        // final 200 s used for the anti-flatline + stopped-fraction checks
        var arrivalsAtLateWindowStart = 0L;
        var lastPos = new System.Collections.Generic.Dictionary<Sim.Core.VehicleHandle, (double X, double Y)>();
        var lateMatched = 0;
        var lateStopped = 0;

        for (var i = 0; i < totalSteps; i++)
        {
            sim.Step();
            var snap = sim.Sample();

            var inLateWindow = i >= totalSteps - lateWindow;
            if (i == totalSteps - lateWindow)
            {
                arrivalsAtLateWindowStart = sim.ArrivedTotal;
            }

            // Displacement-based stopped fraction, computed EXACTLY as the LIVECITY-GRIDLOCK smoke probe does
            // (per-handle frame-to-frame move < 0.05 m => "stopped"), accumulated over the late window only.
            var cur = new System.Collections.Generic.Dictionary<Sim.Core.VehicleHandle, (double X, double Y)>(snap.Cars.Count);
            foreach (var c in snap.Cars)
            {
                cur[c.Handle] = (c.X, c.Y);
                if (inLateWindow && lastPos.TryGetValue(c.Handle, out var prev))
                {
                    var d = Math.Sqrt(((c.X - prev.X) * (c.X - prev.X)) + ((c.Y - prev.Y) * (c.Y - prev.Y)));
                    lateMatched++;
                    if (d < 0.05) lateStopped++;
                }
            }

            lastPos = cur;
        }

        var finalArrivals = sim.ArrivedTotal;
        var lateGrowth = finalArrivals - arrivalsAtLateWindowStart;
        var lateStoppedFrac = lateMatched > 0 ? (double)lateStopped / lateMatched : 1.0;

        Assert.True(sim.PeakCars > 0, $"expected PeakCars > 0, got {sim.PeakCars}");
        // (1) Total throughput: healthy ~736; a gridlock flatlines ~361. 450 sits between with margin.
        Assert.True(finalArrivals >= 450,
            $"THROUGHPUT regression: expected >= 450 arrivals over 1000 s, got {finalArrivals} (gridlock reforms at ~360)");
        // (2) Anti-flatline (sharpest gridlock signal): arrivals must KEEP climbing to the end. Healthy
        //     grows ~+145 in the last 200 s; a gridlock grows ~+2. 40 firmly separates them.
        Assert.True(lateGrowth >= 40,
            $"GRIDLOCK: arrivals flatlined -- only {lateGrowth} arrivals in the last {lateWindow} steps (healthy ~145, gridlock ~2)");
        // (3) The sim is not frozen: late stopped fraction must not pin near 1.0. Healthy avg ~0.35;
        //     a terminal gridlock ~1.0. 0.85 catches the freeze while tolerating heavy-but-flowing jams.
        Assert.True(lateStoppedFrac <= 0.85,
            $"GRIDLOCK: late stopped fraction {lateStoppedFrac:F2} pinned high (healthy ~0.35, frozen ~1.0)");
    }

    // #15 per-area realism LOD (docs/LIVE-CITY-15-PER-AREA-LOD-DESIGN.md), T2 success condition: the
    // classification predicate is a pure function of position -- inside the pocket radius => HIGH realism
    // (false), strictly outside => LOW realism (true), on the boundary => high (<=), and a non-positive
    // radius disables the gate (all high realism). Same inputs => same output (determinism).
    [Fact]
    public void PerAreaLod_Classification_IsPurePositionPredicate()
    {
        const double px = 100.0, py = 200.0, r = 70.0;

        // Inside the pocket -> high realism (not low).
        Assert.False(LiveCitySim.IsLowRealismLaneChangePos(px, py, px, py, r));                 // centre
        Assert.False(LiveCitySim.IsLowRealismLaneChangePos(px + 50.0, py, px, py, r));          // well inside
        // On the boundary (distance == radius) -> high realism (strict >).
        Assert.False(LiveCitySim.IsLowRealismLaneChangePos(px + r, py, px, py, r));
        // Outside the pocket -> low realism.
        Assert.True(LiveCitySim.IsLowRealismLaneChangePos(px + r + 0.01, py, px, py, r));
        Assert.True(LiveCitySim.IsLowRealismLaneChangePos(px + 200.0, py + 200.0, px, py, r));
        // Disabled gate (radius <= 0) -> everyone high realism.
        Assert.False(LiveCitySim.IsLowRealismLaneChangePos(px + 999.0, py, px, py, 0.0));
        // Determinism: identical inputs -> identical output.
        Assert.Equal(
            LiveCitySim.IsLowRealismLaneChangePos(px + 80.0, py + 10.0, px, py, r),
            LiveCitySim.IsLowRealismLaneChangePos(px + 80.0, py + 10.0, px, py, r));
    }

    // #15 per-area LOD, T2 end-to-end: after stepping the real coupled sim, EVERY live car's classification
    // (recomputed from its current render position vs the sim's own exposed pocket) partitions the population
    // -- some cars inside the pocket (high realism), some outside (low) -- proving the pocket actually splits
    // the dense flow rather than trivially classifying all one way. (The engine-side flag itself is set from
    // the previous snapshot; this asserts the predicate + pocket wiring, not the one-step-stale flag value.)
    [Fact]
    public void PerAreaLod_OverAFewMinutes_PocketSplitsThePopulation()
    {
        using var sim = new LiveCitySim(MakeConfig(yield: true));

        for (var i = 0; i < 240; i++) // 120 s -- long enough to spread cars across and beyond the pocket
        {
            sim.Step();
        }

        var snap = sim.Sample();
        Assert.True(snap.Cars.Count > 0, "expected live cars to classify");

        var inside = 0;
        var outside = 0;
        foreach (var c in snap.Cars)
        {
            if (LiveCitySim.IsLowRealismLaneChangePos(c.X, c.Y, sim.HighRealismPocketX, sim.HighRealismPocketY, sim.HighRealismPromoteRadius))
            {
                outside++;
            }
            else
            {
                inside++;
            }
        }

        Assert.True(sim.HighRealismPromoteRadius > 0.0, "expected a positive pocket radius");
        Assert.True(outside > 0, $"expected some LOW-realism (outside-pocket) cars, got {outside} of {snap.Cars.Count}");
        Assert.True(inside > 0, $"expected some HIGH-realism (inside-pocket) cars, got {inside} of {snap.Cars.Count}");
    }

    // #15 camera-driven LC-realism zone (docs/LIVE-CITY-CAMERA-REALISM-ZONE-DESIGN.md): the settable zone
    // defaults to the static pocket (so Central mode == prior behaviour) and, once moved, is what the
    // per-area classification keys on -- a position at the moved centre is HIGH realism and one back at the
    // old pocket is LOW.
    [Fact]
    public void LcRealismZone_DefaultsToPocket_AndIsSettable()
    {
        using var sim = new LiveCitySim(MakeConfig(yield: true));

        Assert.Equal(sim.HighRealismPocketX, sim.LcZoneX, 6);
        Assert.Equal(sim.HighRealismPocketY, sim.LcZoneY, 6);
        Assert.Equal(sim.HighRealismPromoteRadius, sim.LcZoneRadius, 6);

        var newX = sim.HighRealismPocketX + 500.0;
        var newY = sim.HighRealismPocketY + 500.0;
        const double newR = 80.0;
        sim.SetLcRealismZone(newX, newY, newR);
        Assert.Equal(newX, sim.LcZoneX, 6);
        Assert.Equal(newY, sim.LcZoneY, 6);
        Assert.Equal(newR, sim.LcZoneRadius, 6);

        // Classification now keys on the MOVED zone: new centre is inside (high), old pocket is outside (low).
        Assert.False(LiveCitySim.IsLowRealismLaneChangePos(newX, newY, sim.LcZoneX, sim.LcZoneY, sim.LcZoneRadius));
        Assert.True(LiveCitySim.IsLowRealismLaneChangePos(
            sim.HighRealismPocketX, sim.HighRealismPocketY, sim.LcZoneX, sim.LcZoneY, sim.LcZoneRadius));
    }

    [Fact]
    public void YieldOnVsOff_ProduceDifferentCoupling_AndYieldOnIsPositive()
    {
        using var simOn = new LiveCitySim(MakeConfig(yield: true));
        using var simOff = new LiveCitySim(MakeConfig(yield: false));

        var trajectoryDiffers = false;

        for (var step = 0; step < 120; step++)
        {
            simOn.Step();
            simOff.Step();

            var onSnap = simOn.Sample();
            var offSnap = simOff.Sample();

            if (onSnap.Cars.Count != offSnap.Cars.Count)
            {
                trajectoryDiffers = true;
                continue;
            }

            for (var i = 0; i < onSnap.Cars.Count; i++)
            {
                if (Math.Abs(onSnap.Cars[i].X - offSnap.Cars[i].X) > 1e-9
                    || Math.Abs(onSnap.Cars[i].Y - offSnap.Cars[i].Y) > 1e-9)
                {
                    trajectoryDiffers = true;
                    break;
                }
            }
        }

        Assert.True(simOn.CarYieldObservations > 0, $"expected yield-ON CarYieldObservations > 0, got {simOn.CarYieldObservations}");
        Assert.True(
            trajectoryDiffers || simOn.CarYieldObservations != simOff.CarYieldObservations,
            $"expected yield ON/OFF to differ: onObs={simOn.CarYieldObservations} offObs={simOff.CarYieldObservations} trajectoryDiffers={trajectoryDiffers}");
    }

    // Count "crossing nose-in" ticks over a fixed headless run: a MOVING car whose front bumper is on/over a
    // pedestrian who is inside a real crossing polygon (the visible "car drives over a ped on the zebra"
    // event, defect #1). Motion direction + speed are finite-differenced from Sample() poses (no engine
    // internals), and "on a crossing" is the true point-in-polygon test (LiveCitySim.IsOnCrossingPolygon),
    // NOT a loose centroid circle -- the same metric the --live-city-yieldtrace diagnostic reports.
    private static int CountCrossingNoseIns(LiveCityConfig cfg, int steps)
    {
        const double pedR = 0.3;
        using var sim = new LiveCitySim(cfg);
        var prev = new System.Collections.Generic.Dictionary<Sim.Core.VehicleHandle, (double X, double Y)>();
        var noseIn = 0;

        for (var s = 0; s < steps; s++)
        {
            sim.Step();
            var snap = sim.Sample();

            var onCross = new System.Collections.Generic.List<(double X, double Y)>();
            foreach (var p in snap.Peds)
            {
                if (sim.IsOnCrossingPolygon(p.X, p.Y)) onCross.Add((p.X, p.Y));
            }

            if (onCross.Count > 0)
            {
                foreach (var c in snap.Cars)
                {
                    if (!prev.TryGetValue(c.Handle, out var pp)) continue;
                    var fx = c.X - pp.X; var fy = c.Y - pp.Y;
                    var fn = Math.Sqrt((fx * fx) + (fy * fy));
                    var spd = fn / cfg.Dt;
                    if (spd <= 0.5 || fn < 1e-3) continue;         // stopped/yielded cars are fine
                    var ux = fx / fn; var uy = fy / fn;
                    var halfW = c.Width * 0.5; var halfLen = c.Length * 0.5;
                    foreach (var (px, py) in onCross)
                    {
                        var vx = px - c.X; var vy = py - c.Y;
                        var along = (vx * ux) + (vy * uy);
                        var lat = Math.Abs((vx * (-uy)) + (vy * ux));
                        if (along > 0 && along < halfLen + pedR && lat < halfW + pedR) { noseIn++; break; }
                    }
                }
            }

            prev.Clear();
            foreach (var c in snap.Cars) prev[c.Handle] = (c.X, c.Y);
        }

        return noseIn;
    }

    // Realism #1/#2 (docs/LIVE-CITY-REALISM-1-2-DESIGN.md) regression guard -- and, like the dense-flow test
    // above, a guard the PARITY gate structurally cannot provide: the crossing-yield fix is demo-gated
    // (Engine.CrowdSource is null on every golden), so a regression that reverts cars to driving over
    // crossing pedestrians passes parity byte-for-byte. This test pins the fix: with the shipped defaults
    // (CrossingGateRadius=1.5, GatePausedPedsOnCrossing=true) a moving car noses over a ped on a crossing
    // FAR less often than with the stock 0.3 m point disc. Deterministic (same seed+config => identical run),
    // so the thresholds are stable, not statistical.
    //
    // Measured (this branch, 400 steps = 200 s): STOCK nose-in=10, FIXED nose-in=1 (residual is an ORCA
    // promotion-timing edge, defect #3/#4). The thresholds below sit with wide margin on both sides.
    [Fact]
    public void CrossingYield_FixedGate_NosesOverFarFewerCrossingPeds_ThanStockPointDisc()
    {
        const int steps = 400;

        LiveCityConfig Base()
        {
            var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
            // Pin the scenario so the assertion is about the GATE, not config/env drift.
            cfg.CarTargetConcurrent = 160;
            cfg.TimeToTeleportSeconds = 0.0;
            cfg.Dt = 0.5;
            cfg.YieldEnabled = true;
            return cfg;
        }

        var stockCfg = Base();
        stockCfg.CrossingGateRadius = 0.3;              // stock point disc
        stockCfg.GatePausedPedsOnCrossing = false;      // stock walking-only feed

        var fixedCfg = Base();                          // shipped defaults: r=1.5, paused feed on
        Assert.Equal(1.5, fixedCfg.CrossingGateRadius);
        Assert.True(fixedCfg.GatePausedPedsOnCrossing);

        var stockNose = CountCrossingNoseIns(stockCfg, steps);
        var fixedNose = CountCrossingNoseIns(fixedCfg, steps);

        // (1) The stock behaviour genuinely exhibits the defect (else the test proves nothing).
        Assert.True(stockNose >= 5,
            $"expected the STOCK point-disc to nose over crossing peds (>=5), got {stockNose} -- defect not reproduced");
        // (2) The fix drives it near zero.
        Assert.True(fixedNose <= 3,
            $"crossing-yield REGRESSION: fixed-gate nose-ins {fixedNose} (expected <=3; stock was {stockNose})");
        // (3) A clear, non-marginal improvement (guards against both drifting toward stock).
        Assert.True(fixedNose * 2 < stockNose,
            $"crossing-yield fix lost its margin: fixed={fixedNose} stock={stockNose} (want fixed*2 < stock)");
    }

    // Realism #1/#2 at HIGH ped density -- the guard for the two density-only bugs the 1x test can't see:
    //  (i)  the engine's crowd-disc query cap (was 16): at 10x density a car has a median ~39 / max ~131 crowd
    //       discs in range, so the in-path disc was truncated away and the car drove THROUGH the ped;
    //  (ii) ORCA peds on a crossing were fed only as their 0.3 m physics footprint (narrow wheel corridor).
    // With MaxCrowdDiscs=256 + ORCA peds fed to the wide crossing gate, nose-ins at 10x drop from 28 (15 of
    // them FAST >=4 m/s) to a handful of slow (~1.3 m/s) tail cases. Deterministic (seed+config fixed).
    // One 400-step run at 1600 peds -- heavier than the 1x test, but still a few seconds headless.
    [Fact]
    public void CrossingYield_HoldsUnderHighPedDensity_NoMassDriveThrough()
    {
        var cfg = LiveCityConfig.ForRepoRoot(RepoRoot());
        cfg.CarTargetConcurrent = 160;
        cfg.TimeToTeleportSeconds = 0.0;
        cfg.Dt = 0.5;
        cfg.YieldEnabled = true;
        cfg.PedPopulationCap = 1600;              // 10x the shipped 160
        cfg.PedSpawnRatePerSecond = 80.0;         // matches ForRepoRoot's LIVECITY_PEDS scaling (8 * peds/160)
        // shipped fix defaults (assert they're what we think, so a default change can't silently weaken this)
        Assert.Equal(1.5, cfg.CrossingGateRadius);
        Assert.True(cfg.GatePausedPedsOnCrossing);

        var nose = CountCrossingNoseIns(cfg, 400);

        // At 10x, the pre-fix engine truncated in-path discs and drove through ~28 crossing peds (15 fast).
        // The fix keeps it to a small slow tail; 12 sits well below the broken regime with margin for the
        // finite-difference metric's noise.
        Assert.True(nose <= 12,
            $"HIGH-DENSITY crossing-yield regression: {nose} nose-ins at 10x ped density (fixed target <=12; the "
            + "pre-fix engine 16-disc truncation + ORCA point-disc drove through ~28, 15 of them fast)");
    }
}
