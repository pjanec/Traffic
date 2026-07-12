using Sim.Core;

namespace Sim.Harness;

public static class TrajectoryComparator
{
    public static ComparisonResult Compare(TrajectorySet actual, TrajectorySet expected, ToleranceConfig tolerance)
    {
        var attributes = tolerance.ComparedAttributes.Count > 0
            ? tolerance.ComparedAttributes
            : ToleranceConfig.DefaultComparedAttributes;

        var presenceMismatches = new List<PresenceMismatch>();

        var actualVehicles = actual.VehicleIds.ToHashSet();
        var expectedVehicles = expected.VehicleIds.ToHashSet();

        foreach (var vehicleId in expectedVehicles.Except(actualVehicles).OrderBy(id => id, StringComparer.Ordinal))
            presenceMismatches.Add(PresenceMismatch.MissingVehicle(vehicleId));
        foreach (var vehicleId in actualVehicles.Except(expectedVehicles).OrderBy(id => id, StringComparer.Ordinal))
            presenceMismatches.Add(PresenceMismatch.ExtraVehicle(vehicleId));

        var commonVehicles = actualVehicles.Intersect(expectedVehicles).OrderBy(id => id, StringComparer.Ordinal);

        var commonPoints = new List<(double Time, string VehicleId, TrajectoryPoint Actual, TrajectoryPoint Expected)>();

        foreach (var vehicleId in commonVehicles)
        {
            var actualPoints = actual.PointsFor(vehicleId);
            var expectedPoints = expected.PointsFor(vehicleId);
            var actualTimes = actualPoints.Keys.ToHashSet();
            var expectedTimes = expectedPoints.Keys.ToHashSet();

            foreach (var time in expectedTimes.Except(actualTimes).OrderBy(t => t))
                presenceMismatches.Add(PresenceMismatch.MissingStep(vehicleId, time));
            foreach (var time in actualTimes.Except(expectedTimes).OrderBy(t => t))
                presenceMismatches.Add(PresenceMismatch.ExtraStep(vehicleId, time));

            foreach (var time in actualTimes.Intersect(expectedTimes))
                commonPoints.Add((time, vehicleId, actualPoints[time], expectedPoints[time]));
        }

        commonPoints.Sort((a, b) =>
        {
            var byTime = a.Time.CompareTo(b.Time);
            return byTime != 0 ? byTime : string.CompareOrdinal(a.VehicleId, b.VehicleId);
        });

        var errorsByAttribute = attributes.ToDictionary(a => a, _ => new List<double>());
        double? firstDivergence = null;

        foreach (var (time, _, actualPoint, expectedPoint) in commonPoints)
        {
            foreach (var attribute in attributes)
            {
                var error = ComputeError(attribute, actualPoint, expectedPoint);
                errorsByAttribute[attribute].Add(error);

                if (firstDivergence is null && error > tolerance.ToleranceFor(attribute))
                    firstDivergence = time;
            }
        }

        var attributeResults = attributes.Select(attribute =>
        {
            var errors = errorsByAttribute[attribute];
            var maxAbs = errors.Count > 0 ? errors.Max() : 0.0;
            var rmse = errors.Count > 0 ? Math.Sqrt(errors.Average(e => e * e)) : 0.0;
            return new AttributeComparisonResult(attribute, maxAbs, rmse, maxAbs <= tolerance.ToleranceFor(attribute));
        }).ToList();

        return new ComparisonResult
        {
            Attributes = attributeResults,
            PresenceMismatches = presenceMismatches,
            FirstDivergenceStep = firstDivergence,
        };
    }

    /// <summary>
    /// ENSEMBLE STATISTICAL comparison (C1-ii): compares aggregate distribution statistics of an
    /// ensemble of engine runs (different seeds) against an ensemble of expected runs, instead of
    /// matching individual (vehicle, time) points exactly. This is the right comparator once
    /// <c>sigma&gt;0</c> — with dawdling there is no single trajectory to match to 1e-3, but the
    /// POOLED distribution of speeds/positions across many seeds should still land in the same
    /// place SUMO's does.
    ///
    /// "Pooled over all runs" means every (run, vehicle, time) sample of an attribute across the
    /// whole ensemble is thrown into one flat list before computing statistics — runs are not
    /// weighted or averaged individually first, and no attempt is made to align same-seed runs
    /// between actual/expected (statistical parity does not require it: only the ensembles' shapes
    /// need to agree, not which vehicle produced which point). For each attribute this computes:
    ///   - mean: the arithmetic mean of the pooled samples.
    ///   - std: the POPULATION standard deviation of the pooled samples, i.e.
    ///     sqrt(mean((x - mean(x))^2)) — not the Bessel-corrected sample std, since we are treating
    ///     the pooled ensemble as the full population we care about, not an estimate of a larger one.
    /// An empty ensemble (no runs, or no points of that attribute) yields mean=std=0 by convention
    /// (guarded explicitly below) rather than throwing or producing NaN.
    ///
    /// "lane" is intentionally never compared here: it is categorical (a string), and averaging or
    /// taking a standard deviation of a lane id is not meaningful. Statistical tolerance configs
    /// should list only numeric attributes (typically "speed", "pos") in comparedAttributes; if
    /// "lane" is present it is silently skipped rather than rejected, so exact-mode's default
    /// comparedAttributes list can be reused without modification.
    ///
    /// DEFERRED (not built here): a per-time-bin flow/density (fundamental-diagram) variant, which
    /// would bucket samples by time (or by space) before aggregating instead of pooling everything
    /// into one flat distribution. That is a richer, later extension — this method only supports
    /// the whole-ensemble pooled mean/std bar decided for C1.
    /// </summary>
    public static EnsembleComparisonResult CompareEnsemble(
        IReadOnlyList<TrajectorySet> actualRuns,
        IReadOnlyList<TrajectorySet> expectedRuns,
        ToleranceConfig tolerance)
    {
        var attributes = (tolerance.ComparedAttributes.Count > 0
                ? tolerance.ComparedAttributes
                : ToleranceConfig.DefaultComparedAttributes)
            .Where(attribute => attribute != "lane")
            .ToList();

        var attributeResults = attributes.Select(attribute =>
        {
            var actualValues = PooledValues(actualRuns, attribute);
            var expectedValues = PooledValues(expectedRuns, attribute);

            var meanActual = Mean(actualValues);
            var meanExpected = Mean(expectedValues);
            var stdActual = PopulationStd(actualValues, meanActual);
            var stdExpected = PopulationStd(expectedValues, meanExpected);

            var meanError = Math.Abs(meanActual - meanExpected);
            var stdError = Math.Abs(stdActual - stdExpected);

            var meanTolerance = tolerance.MeanToleranceFor(attribute);
            var stdTolerance = tolerance.StdToleranceFor(attribute);

            return new AttributeEnsembleComparisonResult(
                attribute,
                meanActual, meanExpected, meanError, meanError <= meanTolerance,
                stdActual, stdExpected, stdError, stdError <= stdTolerance);
        }).ToList();

        return new EnsembleComparisonResult { Attributes = attributeResults };
    }

    private static List<double> PooledValues(IReadOnlyList<TrajectorySet> runs, string attribute) =>
        runs
            .SelectMany(run => run.AllPoints)
            .Select(point => GetValue(attribute, point))
            .Where(value => value.HasValue)
            .Select(value => value!.Value)
            .ToList();

    private static double Mean(IReadOnlyList<double> values) =>
        values.Count > 0 ? values.Average() : 0.0;

    private static double PopulationStd(IReadOnlyList<double> values, double mean) =>
        values.Count > 0 ? Math.Sqrt(values.Average(v => (v - mean) * (v - mean))) : 0.0;

    private static double? GetValue(string attribute, TrajectoryPoint point) => attribute switch
    {
        "pos" => point.Pos,
        "speed" => point.Speed,
        "posLat" => point.PosLat,
        "x" => point.X,
        "y" => point.Y,
        "angle" => point.Angle,
        "acceleration" => point.Acceleration,
        _ => throw new ArgumentException($"Unknown comparison attribute '{attribute}'.", nameof(attribute)),
    };

    private static double ComputeError(string attribute, TrajectoryPoint actual, TrajectoryPoint expected) => attribute switch
    {
        "lane" => string.Equals(actual.Lane, expected.Lane, StringComparison.Ordinal) ? 0.0 : 1.0,
        "pos" => Math.Abs(actual.Pos - expected.Pos),
        "speed" => Math.Abs(actual.Speed - expected.Speed),
        "posLat" => Math.Abs(actual.PosLat - expected.PosLat),
        "x" => Math.Abs(actual.X - expected.X),
        "y" => Math.Abs(actual.Y - expected.Y),
        "angle" => Math.Abs(actual.Angle - expected.Angle),
        "acceleration" => AccelerationError(actual.Acceleration, expected.Acceleration),
        _ => throw new ArgumentException($"Unknown comparison attribute '{attribute}'.", nameof(attribute)),
    };

    private static double AccelerationError(double? actual, double? expected)
    {
        if (actual is null || expected is null)
            return actual == expected ? 0.0 : double.PositiveInfinity;
        return Math.Abs(actual.Value - expected.Value);
    }
}
