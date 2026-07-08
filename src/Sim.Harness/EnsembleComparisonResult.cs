namespace Sim.Harness;

/// <summary>
/// Ensemble (mean, std) comparison outcome for one attribute, mirroring the reporting shape of
/// <see cref="AttributeComparisonResult"/> so a failing statistical test can print a useful
/// diagnostic (which of mean/std failed, by how much, against which tolerance).
/// </summary>
public sealed record AttributeEnsembleComparisonResult(
    string Attribute,
    double MeanActual,
    double MeanExpected,
    double MeanError,
    bool MeanWithinTolerance,
    double StdActual,
    double StdExpected,
    double StdError,
    bool StdWithinTolerance)
{
    public bool WithinTolerance => MeanWithinTolerance && StdWithinTolerance;
}

/// <summary>
/// Result of <see cref="TrajectoryComparator.CompareEnsemble"/>: pooled ensemble-aggregate
/// statistics compared per attribute. There is no per-(vehicle,time) presence/divergence
/// tracking here (unlike <see cref="ComparisonResult"/>) — statistical parity only claims the
/// pooled distribution matches, not that any individual trajectory does.
/// </summary>
public sealed class EnsembleComparisonResult
{
    public required IReadOnlyList<AttributeEnsembleComparisonResult> Attributes { get; init; }

    public bool IsMatch => Attributes.All(a => a.WithinTolerance);
}
