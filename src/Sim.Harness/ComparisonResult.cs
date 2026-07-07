namespace Sim.Harness;

public sealed record AttributeComparisonResult(string Attribute, double MaxAbsError, double Rmse, bool WithinTolerance);

public sealed class ComparisonResult
{
    public required IReadOnlyList<AttributeComparisonResult> Attributes { get; init; }
    public required IReadOnlyList<PresenceMismatch> PresenceMismatches { get; init; }
    public required double? FirstDivergenceStep { get; init; }

    public bool IsMatch => PresenceMismatches.Count == 0 && Attributes.All(a => a.WithinTolerance);
}
