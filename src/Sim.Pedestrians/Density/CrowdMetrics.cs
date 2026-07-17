namespace Sim.Pedestrians.Density;

// POC-4 (docs/PEDESTRIAN-POC-PLAN.md POC-4; docs/PEDESTRIAN-DESIGN.md §3 "Believability"): small,
// dependency-free statistics helpers shared by the corridor and bottleneck measurement scenarios.
// Pure functions only -- no state, no System.Random -- so callers stay deterministic by
// construction.
public static class CrowdMetrics
{
    // Pearson correlation coefficient between two equal-length samples. Used as the corridor's
    // quantitative lane-formation metric: correlate each agent's lateral (cross-channel) offset
    // with a +1/-1 sign for its travel direction. If the two directions self-organize into lateral
    // lanes (direction correlates with which side of the channel an agent is on), |r| grows well
    // above 0; a fully-mixed, laneless flow stays near 0. Returns 0.0 (not NaN) when either sample
    // has zero variance -- "no measurable structure" is the right degenerate answer for a metric
    // that otherwise reports a de-facto boolean "lanes formed or not."
    public static double PearsonCorrelation(IReadOnlyList<double> xs, IReadOnlyList<double> ys)
    {
        if (xs.Count != ys.Count)
        {
            throw new ArgumentException("Sample lists must be the same length.");
        }

        var n = xs.Count;
        if (n < 2)
        {
            return 0.0;
        }

        var meanX = 0.0;
        var meanY = 0.0;
        for (var i = 0; i < n; i++)
        {
            meanX += xs[i];
            meanY += ys[i];
        }

        meanX /= n;
        meanY /= n;

        var cov = 0.0;
        var varX = 0.0;
        var varY = 0.0;
        for (var i = 0; i < n; i++)
        {
            var dx = xs[i] - meanX;
            var dy = ys[i] - meanY;
            cov += dx * dy;
            varX += dx * dx;
            varY += dy * dy;
        }

        if (varX <= 1e-12 || varY <= 1e-12)
        {
            return 0.0;
        }

        return cov / Math.Sqrt(varX * varY);
    }
}
