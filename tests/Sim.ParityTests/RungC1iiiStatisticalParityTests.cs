using Sim.Core;
using Sim.Harness;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// Rung C1-iii (TASKS.md "Statistical parity / driver imperfection" -- the [net] validation step):
// the ENSEMBLE statistical parity bar (owner decision, C1) exercised end-to-end against SUMO.
// scenarios/17-dawdle-freeflow is a single vehicle at free flow with sigma=0.5 (Krauss dawdling on).
// With randomness there is no single trajectory to match 1e-3; instead we compare the AGGREGATE
// speed distribution of an ENSEMBLE of engine runs (N seeds) against an ensemble of SUMO runs (the
// committed golden.ensemble/*.fcd.xml, one per SUMO seed) via TrajectoryComparator.CompareEnsemble
// (C1-ii). This is a REAL test, not a loose one: our engine ported SUMO's dawdle2 ALGORITHM
// faithfully (C1-i), so even though our RNG stream differs from SUMO's, both ensembles draw
// uniform[0,1) and apply the identical reduction formula -- their pooled mean/std of speed must
// converge to the same values (a sigma factor-of-2 bug, or a dropped MAX2(vMin,..)/min(speed,accel)
// term, would shift the ensemble mean/std well past the committed tolerance). The RNG stream itself
// is NOT compared (that is the ensemble-not-RNG-exact decision).
public class RungC1iiiStatisticalParityTests
{
    private const int EnsembleSize = 24; // matches golden.ensemble/ seed count (provenance.txt)
    private const int Steps = 80;        // config.sumocfg end=80

    private readonly ITestOutputHelper _out;

    public RungC1iiiStatisticalParityTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void DawdleFreeFlow_EngineEnsembleMatchesSumoEnsembleStatistically()
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "17-dawdle-freeflow");
        var tolerance = ToleranceConfig.Load(Path.Combine(dir, "tolerance.json"));

        // Expected ensemble: the committed SUMO runs, one TrajectorySet per seed.
        var expected = Directory
            .EnumerateFiles(Path.Combine(dir, "golden.ensemble"), "*.fcd.xml")
            .OrderBy(p => p, StringComparer.Ordinal)
            .Select(FcdParser.Parse)
            .ToList();
        Assert.Equal(EnsembleSize, expected.Count);

        // Actual ensemble: N independent engine runs, one per seed. Seeds 1..N are OUR engine's
        // RNG seeds (Engine.Seed), NOT SUMO's -- ensemble stats must match regardless of the
        // specific seed values (both are samples of the same dawdle distribution).
        var actual = new List<TrajectorySet>(EnsembleSize);
        for (var seed = 1; seed <= EnsembleSize; seed++)
        {
            var engine = new Engine { Seed = (ulong)seed };
            engine.LoadScenario(
                Path.Combine(dir, "net.net.xml"),
                Path.Combine(dir, "rou.rou.xml"),
                Path.Combine(dir, "config.sumocfg"));
            actual.Add(engine.Run(Steps));
        }

        var result = TrajectoryComparator.CompareEnsemble(actual, expected, tolerance);

        foreach (var a in result.Attributes)
        {
            _out.WriteLine(
                $"[{a.Attribute}] mean: engine={a.MeanActual:F6} sumo={a.MeanExpected:F6} " +
                $"|Δ|={a.MeanError:F6} (tol {tolerance.MeanToleranceFor(a.Attribute)}) ok={a.MeanWithinTolerance}");
            _out.WriteLine(
                $"[{a.Attribute}] std : engine={a.StdActual:F6} sumo={a.StdExpected:F6} " +
                $"|Δ|={a.StdError:F6} (tol {tolerance.StdToleranceFor(a.Attribute)}) ok={a.StdWithinTolerance}");
        }

        Assert.True(result.IsMatch, BuildFailureMessage(result, tolerance));
    }

    private static string BuildFailureMessage(EnsembleComparisonResult result, ToleranceConfig tol)
    {
        var lines = new List<string> { "Ensemble statistical parity FAILED:" };
        foreach (var a in result.Attributes)
        {
            lines.Add(
                $"  {a.Attribute}: meanΔ={a.MeanError:F6}/{tol.MeanToleranceFor(a.Attribute)} " +
                $"(engine {a.MeanActual:F6} vs sumo {a.MeanExpected:F6}); " +
                $"stdΔ={a.StdError:F6}/{tol.StdToleranceFor(a.Attribute)} " +
                $"(engine {a.StdActual:F6} vs sumo {a.StdExpected:F6})");
        }

        return string.Join(Environment.NewLine, lines);
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
