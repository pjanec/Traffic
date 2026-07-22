using Sim.IgBridge;
using Xunit;

namespace Sim.IgBridge.Tests;

// T1.5 / seeds T3.2 (docs/IGBRIDGE-TASKS.md): the objective smoothness win, MEASURED. Reconstruct the raw
// engine stream and the IgBridge-smoothed stream through the same FakeIg and compare. The robust wins at
// 10 Hz are: junction-turn faceting -> median yaw-rate/yaw-jerk sharply lower; instant lane changes ->
// mean lateral-accel sharply lower. (max/p95 are dominated by genuine hairpin U-turns present in BOTH
// streams, so the asserted bounds use the outlier-robust median/mean.)
public sealed class MetricsTests
{
    private const int Steps = 700;       // 70 s @ 10 Hz -- enough turning vehicles for a stable turner set
    private const double RenderHz = 60.0;

    // A vehicle "turns" if its RAW reconstructed heading has real angular motion (max yaw-rate above this);
    // straight-driving vehicles (yaw ~0 in both streams) would only dilute the junction-turn comparison.
    private const double TurnerYawRateThreshold = 60.0; // deg/s

    [Fact]
    public void Reconstructed_SmoothsJunctionTurns_AndLaneChanges()
    {
        var (raw, recon) = RunAndMeasure();

        // Pair by entity id; isolate the vehicles that actually turn (per the RAW stream).
        var ids = new List<string>();
        foreach (var id in raw.Keys)
        {
            if (recon.ContainsKey(id) && raw[id].YawRateMax >= TurnerYawRateThreshold)
            {
                ids.Add(id);
            }
        }

        Assert.True(ids.Count >= 20, $"only {ids.Count} turning vehicles matched");

        // JUNCTION TURNS: the large majority of turners have LOWER reconstructed yaw-rate AND yaw-jerk --
        // the faceted per-segment heading snaps are replaced by the arc-window smooth curve.
        int yawRateBetter = 0, yawJerkBetter = 0;
        var rawYawRate = new List<double>();
        var reconYawRate = new List<double>();
        foreach (var id in ids)
        {
            if (recon[id].YawRateMax < raw[id].YawRateMax) yawRateBetter++;
            if (recon[id].YawJerkMax < raw[id].YawJerkMax) yawJerkBetter++;
            rawYawRate.Add(raw[id].YawRateMax);
            reconYawRate.Add(recon[id].YawRateMax);
        }

        Assert.True(yawRateBetter >= 0.75 * ids.Count,
            $"only {yawRateBetter}/{ids.Count} turners improved yaw-rate");
        Assert.True(yawJerkBetter >= 0.70 * ids.Count,
            $"only {yawJerkBetter}/{ids.Count} turners improved yaw-jerk");

        // And the turner-median yaw-rate is materially lower (not just marginally more-often-lower).
        Assert.True(Median(reconYawRate) < 0.7 * Median(rawYawRate),
            $"turner-median yaw-rate raw={Median(rawYawRate):F1} recon={Median(reconYawRate):F1}");

        // LANE CHANGES: across ALL matched vehicles the mean peak lateral acceleration is lower -- the
        // instant one-tick lane snap (a huge lateral spike in raw) is smoothed. (Among pure turners the
        // smooth curve legitimately carries MORE centripetal accel than the faceted path, so this is an
        // all-vehicle mean, not a turner metric.)
        double rawLat = 0, reconLat = 0;
        int n = 0;
        foreach (var id in raw.Keys)
        {
            if (!recon.ContainsKey(id)) continue;
            rawLat += raw[id].LatAccelMax;
            reconLat += recon[id].LatAccelMax;
            n++;
        }

        Assert.True(reconLat / n < 0.75 * (rawLat / n),
            $"mean peak lat-accel not improved: raw={rawLat / n:F1} recon={reconLat / n:F1}");

        // Finite/bounded.
        foreach (var id in ids)
        {
            Assert.True(double.IsFinite(recon[id].YawRateMax) && double.IsFinite(recon[id].YawJerkMax),
                $"non-finite recon metric for {id}");
        }
    }

    private static double Median(List<double> xs)
    {
        xs.Sort();
        return xs.Count == 0 ? 0 : xs[xs.Count / 2];
    }

    private static (IReadOnlyDictionary<string, EntityMetrics> raw, IReadOnlyDictionary<string, EntityMetrics> recon)
        RunAndMeasure()
    {
        var box = Path.Combine(RepoRoot(), "scenarios", "_ped", "demo_city", "box");
        var cfg = new IgBridgeConfig(Path.Combine(box, "net.xml"), Path.Combine(box, "scenario.rou.xml"))
        {
            StepLength = 0.1,
            Seed = 42,
        };
        var igCfg = new FakeIgConfig { DelaySeconds = 0.75, JumpThresholdMeters = 8.0, RenderHz = RenderHz };

        var runner = new IgBridgeRunner(cfg);
        var rawSamples = new List<IgSample>();
        var seen = new HashSet<string>();
        var prevLive = new HashSet<string>();

        using var sw = new StringWriter();
        var session = new IgBridgeSession(runner, new IgEmitConfig { EmitHz = 20.0, LookaheadSeconds = 0.1 },
            new IgTraceWriter(sw), retainAll: true);

        for (var i = 0; i < Steps; i++)
        {
            runner.Tick();
            var t = runner.SimTime;
            var live = new HashSet<string>();
            foreach (var r in runner.RawVehiclePosesThisTick)
            {
                live.Add(r.Id);
                rawSamples.Add(seen.Add(r.Id)
                    ? IgSample.Created(r.Id, t, IgEntityModel.Car, r.X, r.Y, r.Z, r.HeadingDeg)
                    : IgSample.Updated(r.Id, t, r.X, r.Y, r.Z, r.HeadingDeg));
            }

            foreach (var id in prevLive)
            {
                if (!live.Contains(id))
                {
                    rawSamples.Add(IgSample.Removed(id, t));
                }
            }

            prevLive = live;
            session.Advance();
        }

        session.Finish();

        var rawIg = new FakeIg(rawSamples, igCfg);
        var smIg = new FakeIg(session.AllEmitted, igCfg);
        var rawStreams = rawIg.Reconstruct();
        var smStreams = new Dictionary<string, IReadOnlyList<ReconPose>>();
        foreach (var kv in smIg.Reconstruct())
        {
            if (smIg.ModelOf(kv.Key) == IgEntityModel.Car)
            {
                smStreams[kv.Key] = kv.Value;
            }
        }

        return (Metrics.ComputePerEntity(rawStreams, RenderHz), Metrics.ComputePerEntity(smStreams, RenderHz));
    }

    private static string RepoRoot()
    {
        var d = new DirectoryInfo(AppContext.BaseDirectory);
        while (d is not null && !File.Exists(Path.Combine(d.FullName, "Traffic.sln")))
        {
            d = d.Parent;
        }

        return d?.FullName ?? throw new InvalidOperationException("repo root not found");
    }
}
