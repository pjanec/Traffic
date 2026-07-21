using System.Text;

namespace Sim.IgBridge;

// Aggregate of one smoothness metric across entities: the worst-case (max), the 95th percentile of the
// per-entity maxima (robust to a single outlier), and the mean of the per-entity maxima.
public readonly struct MetricAgg
{
    public MetricAgg(double max, double p95, double median, double mean)
    {
        Max = max;
        P95 = p95;
        Median = median;
        Mean = mean;
    }

    public double Max { get; }
    public double P95 { get; }
    public double Median { get; } // robust central tendency: the typical vehicle's worst, immune to a few genuine hairpins
    public double Mean { get; }
}

public sealed class SmoothnessStats
{
    public int Entities { get; init; }
    public MetricAgg YawRateDegPerSec { get; init; }    // |dHeading/dt|
    public MetricAgg YawJerkDegPerSec2 { get; init; }   // |d2Heading/dt2|
    public MetricAgg LateralAccelMPerS2 { get; init; }  // acceleration perpendicular to travel
    public MetricAgg C1VelGapMPerS { get; init; }       // |dv| between consecutive segments (C1 discontinuity proxy)
}

// The objective smoothness metrics (docs/IGBRIDGE-REQUIREMENTS.md R8/§9.3, DECISIONS §7). Computed on the
// FakeIg-reconstructed (IG-displayed) pose timelines, so "no artifacts" is MEASURED, not eyeballed. The
// same routine runs on the raw-engine stream and the IgBridge-smoothed stream; the reconstructed-smoothed
// yaw-rate / yaw-jerk / lateral-accel must be bounded and far below the raw stream's. (Lane-change duration
// is added in T2.1, when the ease that produces a measurable duration exists.)
// Per-entity worst-case of each metric (the max over that entity's reconstructed timeline).
public readonly struct EntityMetrics
{
    public EntityMetrics(double yawRate, double yawJerk, double latAccel, double c1Gap)
    {
        YawRateMax = yawRate;
        YawJerkMax = yawJerk;
        LatAccelMax = latAccel;
        C1GapMax = c1Gap;
    }

    public double YawRateMax { get; }
    public double YawJerkMax { get; }
    public double LatAccelMax { get; }
    public double C1GapMax { get; }
}

public static class Metrics
{
    // Yaw/lateral metrics are meaningless at a standstill (heading noise at ~0 speed), so they are gated to
    // segments moving faster than this.
    public const double MinSpeedForYaw = 1.0; // m/s

    public static SmoothnessStats Compute(
        IReadOnlyDictionary<string, IReadOnlyList<ReconPose>> streams, double renderHz)
    {
        var perEntity = ComputePerEntity(streams, renderHz);
        return new SmoothnessStats
        {
            Entities = perEntity.Count,
            YawRateDegPerSec = Aggregate(Select(perEntity, m => m.YawRateMax)),
            YawJerkDegPerSec2 = Aggregate(Select(perEntity, m => m.YawJerkMax)),
            LateralAccelMPerS2 = Aggregate(Select(perEntity, m => m.LatAccelMax)),
            C1VelGapMPerS = Aggregate(Select(perEntity, m => m.C1GapMax)),
        };
    }

    // Per-entity metrics keyed by id, so a caller can pair raw vs reconstructed by entity and filter (e.g.
    // to vehicles that actually turn) -- the robust way to isolate the junction-turn win from the many
    // straight-driving vehicles whose yaw is ~0 in both streams.
    public static IReadOnlyDictionary<string, EntityMetrics> ComputePerEntity(
        IReadOnlyDictionary<string, IReadOnlyList<ReconPose>> streams, double renderHz)
    {
        var dt = 1.0 / renderHz;
        var result = new Dictionary<string, EntityMetrics>(streams.Count);

        foreach (var kv in streams)
        {
            var p = kv.Value;
            if (p.Count < 4)
            {
                continue;
            }

            double maxYawRate = 0, maxYawJerk = 0, maxLat = 0, maxC1 = 0;
            double prevYawRate = 0;
            var havePrevYawRate = false;
            double prevVx = 0, prevVy = 0;
            var havePrevV = false;

            for (var i = 1; i < p.Count; i++)
            {
                var vx = (p[i].X - p[i - 1].X) / dt;
                var vy = (p[i].Y - p[i - 1].Y) / dt;
                var speed = Math.Sqrt(vx * vx + vy * vy);

                if (havePrevV)
                {
                    // C1 gap: magnitude of the velocity change between consecutive segments.
                    var dvx = vx - prevVx;
                    var dvy = vy - prevVy;
                    maxC1 = Math.Max(maxC1, Math.Sqrt(dvx * dvx + dvy * dvy));

                    // Lateral acceleration: component of the acceleration perpendicular to travel.
                    if (speed > MinSpeedForYaw)
                    {
                        var ax = dvx / dt;
                        var ay = dvy / dt;
                        var nx = -vy / speed; // left-normal of the velocity direction
                        var ny = vx / speed;
                        maxLat = Math.Max(maxLat, Math.Abs(ax * nx + ay * ny));
                    }
                }

                if (speed > MinSpeedForYaw)
                {
                    var yr = Math.Abs(ShortestArcDeg(p[i - 1].HeadingDeg, p[i].HeadingDeg)) / dt;
                    maxYawRate = Math.Max(maxYawRate, yr);
                    if (havePrevYawRate)
                    {
                        maxYawJerk = Math.Max(maxYawJerk, Math.Abs(yr - prevYawRate) / dt);
                    }

                    prevYawRate = yr;
                    havePrevYawRate = true;
                }
                else
                {
                    havePrevYawRate = false;
                }

                prevVx = vx;
                prevVy = vy;
                havePrevV = true;
            }

            result[kv.Key] = new EntityMetrics(maxYawRate, maxYawJerk, maxLat, maxC1);
        }

        return result;
    }

    private static List<double> Select(
        IReadOnlyDictionary<string, EntityMetrics> perEntity, Func<EntityMetrics, double> pick)
    {
        var list = new List<double>(perEntity.Count);
        foreach (var kv in perEntity)
        {
            list.Add(pick(kv.Value));
        }

        return list;
    }

    private static MetricAgg Aggregate(List<double> perEntityMax)
    {
        if (perEntityMax.Count == 0)
        {
            return new MetricAgg(0, 0, 0, 0);
        }

        perEntityMax.Sort();
        var max = perEntityMax[perEntityMax.Count - 1];
        var p95 = perEntityMax[(int)Math.Floor(0.95 * (perEntityMax.Count - 1))];
        var median = perEntityMax[perEntityMax.Count / 2];
        double sum = 0;
        foreach (var v in perEntityMax)
        {
            sum += v;
        }

        return new MetricAgg(max, p95, median, sum / perEntityMax.Count);
    }

    private static double ShortestArcDeg(float a, float b)
        => ((b - a + 540f) % 360f) - 180f;

    // A side-by-side raw-vs-reconstructed table. The reduction factor is on the MEDIAN of per-entity
    // maxima (the typical vehicle), which is robust to the handful of genuine hairpin/U-turn junctions
    // whose real ~180-degree reversals dominate the raw AND reconstructed max/p95 alike.
    public static string FormatComparison(SmoothnessStats raw, SmoothnessStats smoothed)
    {
        var sb = new StringBuilder();
        sb.AppendLine($"entities: raw={raw.Entities} reconstructed={smoothed.Entities}");
        sb.AppendLine("metric                    raw(median/mean /p95  /max)   recon(median/mean /p95  /max)  median-red");
        Row(sb, "yaw-rate  (deg/s)", raw.YawRateDegPerSec, smoothed.YawRateDegPerSec);
        Row(sb, "yaw-jerk  (deg/s^2)", raw.YawJerkDegPerSec2, smoothed.YawJerkDegPerSec2);
        Row(sb, "lat-accel (m/s^2)", raw.LateralAccelMPerS2, smoothed.LateralAccelMPerS2);
        Row(sb, "C1 vel-gap(m/s)", raw.C1VelGapMPerS, smoothed.C1VelGapMPerS);
        return sb.ToString();
    }

    private static void Row(StringBuilder sb, string name, MetricAgg raw, MetricAgg recon)
    {
        var reduction = recon.Median > 1e-9 ? raw.Median / recon.Median : double.PositiveInfinity;
        sb.AppendLine($"{name,-24} {raw.Median,7:F1}/{raw.Mean,6:F1}/{raw.P95,6:F1}/{raw.Max,7:F0}   "
            + $"{recon.Median,7:F1}/{recon.Mean,6:F1}/{recon.P95,6:F1}/{recon.Max,7:F0}   {reduction,5:F1}x");
    }
}
