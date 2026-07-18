namespace Sim.Harness;

/// <summary>
/// GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md): compares an engine-
/// produced tripinfo set against a SUMO golden's, keyed by vehicle id (a tripinfo has no per-step
/// series -- one row per completed trip -- so this mirrors <see cref="SummaryComparator"/>'s
/// shape, keyed by id instead of time).
///
/// <c>ArrivalLane</c> is EXACT string equality (SumoData's audit_nocheat.py requires it verbatim,
/// not a numeric-tolerance field -- docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2). Every other numeric
/// field (Depart/ArrivalTime/ArrivalPos/ArrivalSpeed/Duration/RouteLength/WaitingTime/TimeLoss)
/// gets ONE small absolute tolerance (not per-field) -- the GAP-2 formulas are all seconds/metres-
/// scale quantities with the same order of residual rounding noise (SUMO's own tripinfo file is
/// itself rounded to 2-3 decimals), so a single scalar is simpler than SummaryComparator's
/// per-attribute lookup and is exactly what docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2 asks for ("a
/// small abs tolerance").
/// </summary>
public static class TripInfoComparator
{
    public const double DefaultAbsTolerance = 0.05;

    private static readonly string[] NumericAttributes =
        { "depart", "arrival", "arrivalPos", "arrivalSpeed", "duration", "routeLength", "waitingTime", "timeLoss" };

    public static TripInfoComparisonResult Compare(
        IReadOnlyList<TripInfoRecord> actual,
        IReadOnlyList<TripInfoRecord> expected,
        double absTolerance = DefaultAbsTolerance)
    {
        // "last wins" on a duplicate id is not expected from a real SUMO tripinfo file or the
        // engine's own writer (one <tripinfo> per completed vehicle) -- a plain dictionary keyed
        // by Id is the simplest correct join, mirroring SummaryComparator's Time-keyed join.
        var actualById = new Dictionary<string, TripInfoRecord>(StringComparer.Ordinal);
        foreach (var t in actual)
        {
            actualById[t.Id] = t;
        }

        var expectedById = new Dictionary<string, TripInfoRecord>(StringComparer.Ordinal);
        foreach (var t in expected)
        {
            expectedById[t.Id] = t;
        }

        var missingIds = expectedById.Keys.Except(actualById.Keys).OrderBy(id => id, StringComparer.Ordinal).ToList();
        var extraIds = actualById.Keys.Except(expectedById.Keys).OrderBy(id => id, StringComparer.Ordinal).ToList();
        var commonIds = actualById.Keys.Intersect(expectedById.Keys).OrderBy(id => id, StringComparer.Ordinal).ToList();

        var errorsByAttribute = NumericAttributes.ToDictionary(a => a, _ => new List<double>());
        var arrivalLaneMismatches = new List<string>();

        foreach (var id in commonIds)
        {
            var a = actualById[id];
            var e = expectedById[id];

            if (!string.Equals(a.ArrivalLane, e.ArrivalLane, StringComparison.Ordinal))
            {
                arrivalLaneMismatches.Add($"{id}: actual='{a.ArrivalLane}' expected='{e.ArrivalLane}'");
            }

            foreach (var attribute in NumericAttributes)
            {
                errorsByAttribute[attribute].Add(ComputeError(attribute, a, e));
            }
        }

        var attributeResults = NumericAttributes.Select(attribute =>
        {
            var errors = errorsByAttribute[attribute];
            var maxAbs = errors.Count > 0 ? errors.Max() : 0.0;
            var rmse = errors.Count > 0 ? Math.Sqrt(errors.Average(err => err * err)) : 0.0;
            return new AttributeComparisonResult(attribute, maxAbs, rmse, maxAbs <= absTolerance);
        }).ToList();

        var isMatch = missingIds.Count == 0 && extraIds.Count == 0 && arrivalLaneMismatches.Count == 0
            && attributeResults.All(a => a.WithinTolerance);

        return new TripInfoComparisonResult
        {
            IsMatch = isMatch,
            Attributes = attributeResults,
            MissingIds = missingIds,
            ExtraIds = extraIds,
            ArrivalLaneMismatches = arrivalLaneMismatches,
        };
    }

    // Absent-vs-absent is an exact match (both null); absent-vs-present is a hard divergence
    // (+Infinity, guaranteed outside any finite tolerance), matching SummaryComparator.NullableError's
    // convention for the same reason: a producer that silently OMITS a required field must fail loud,
    // not be scored as a 0 residual.
    private static double ComputeError(string attribute, TripInfoRecord actual, TripInfoRecord expected) => attribute switch
    {
        "depart" => Math.Abs(actual.Depart - expected.Depart),
        "arrival" => NullableError(actual.ArrivalTime, expected.ArrivalTime),
        "arrivalPos" => NullableError(actual.ArrivalPos, expected.ArrivalPos),
        "arrivalSpeed" => NullableError(actual.ArrivalSpeed, expected.ArrivalSpeed),
        "duration" => Math.Abs(actual.Duration - expected.Duration),
        "routeLength" => NullableError(actual.RouteLength, expected.RouteLength),
        "waitingTime" => NullableError(actual.WaitingTime, expected.WaitingTime),
        "timeLoss" => NullableError(actual.TimeLoss, expected.TimeLoss),
        _ => throw new ArgumentException($"Unknown comparison attribute '{attribute}'.", nameof(attribute)),
    };

    private static double NullableError(double? actual, double? expected)
    {
        if (actual is null && expected is null)
        {
            return 0.0;
        }

        if (actual is null || expected is null)
        {
            return double.PositiveInfinity;
        }

        return Math.Abs(actual.Value - expected.Value);
    }
}

public sealed class TripInfoComparisonResult
{
    public required bool IsMatch { get; init; }
    public required IReadOnlyList<AttributeComparisonResult> Attributes { get; init; }
    public required IReadOnlyList<string> MissingIds { get; init; }
    public required IReadOnlyList<string> ExtraIds { get; init; }
    public required IReadOnlyList<string> ArrivalLaneMismatches { get; init; }
}
