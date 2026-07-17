using Sim.Pedestrians.Density;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Density;

// POC-4 (docs/PEDESTRIAN-POC-PLAN.md POC-4 success condition 2; docs/PEDESTRIAN-DESIGN.md §3
// "Believability"): mall-entrance bottleneck -- measures queue formation, gate through-rate
// stability, and drain behavior for PURE ORCA (no density term).
public class BottleneckScenarioTests
{
    private readonly ITestOutputHelper _out;

    public BottleneckScenarioTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void Bottleneck_PureOrca_QueueForms_OutflowStable_NoPermanentJam()
    {
        var scenario = new BottleneckScenario();
        var result = scenario.Run();

        _out.WriteLine("[POC-4 bottleneck, pure ORCA] "
            + $"steps={result.Steps} ({result.SimulatedSeconds:F1}s); "
            + $"arrived={result.Arrived}/{result.Total}; "
            + $"queue at t=0.3s={result.EarlyQueueCount}, peak queue={result.PeakQueueCount}, "
            + $"final queue={result.FinalQueueCount}; "
            + $"gate through-rate: mean={result.MeanGateThroughRate:F3} ped/s "
            + $"(quarter-of-flow range {result.MinSteadyGateThroughRate:F3}..{result.MaxSteadyGateThroughRate:F3} ped/s); "
            + $"quarter rates=[{string.Join(", ", result.QuarterThroughRates.Select(r => r.ToString("F2")))}]; "
            + $"per-5s-window rates (noisy, illustration only)=[{string.Join(", ", result.WindowThroughRates.Select(r => r.ToString("F2")))}]");

        // Success condition: a queue measurably FORMS during the run -- it starts near-empty (the
        // group spawns well back from the gate) and grows to a real peak as agents arrive faster
        // than the gate can pass them, not just a restatement of the spawn count.
        Assert.True(result.EarlyQueueCount <= result.Total / 10,
            $"queue region was already populated at t=2s ({result.EarlyQueueCount} agents) -- "
            + "the measurement isn't capturing queue FORMATION, just the spawn layout");
        Assert.True(result.PeakQueueCount >= result.Total / 4,
            $"no measurable queue formed: peak queue count {result.PeakQueueCount} out of {result.Total} agents");
        Assert.True(result.PeakQueueCount > result.EarlyQueueCount * 3,
            $"queue count did not grow meaningfully: early={result.EarlyQueueCount}, peak={result.PeakQueueCount}");

        // Success condition: the far side drains -- almost everyone eventually arrives.
        Assert.True(result.Arrived >= result.Total * 0.9,
            $"crowd did not drain: {result.Arrived}/{result.Total} arrived within {result.Steps} steps ({result.SimulatedSeconds:F1}s)");

        // Success condition: the gap through-rate is computed and reported (a real positive number),
        // and it is roughly STABLE across the flowing phase -- no quarter of the flow collapses to a
        // near-zero rate while the others are flowing (a lenient stability bound: each quarter stays
        // within 5x of the overall mean).
        Assert.True(result.MeanGateThroughRate > 0.0, "gate through-rate was zero -- nothing got through");
        foreach (var quarterRate in result.QuarterThroughRates)
        {
            Assert.True(quarterRate > result.MeanGateThroughRate / 5.0,
                $"gate through-rate collapsed in one quarter of the flow ({quarterRate:F3} ped/s vs mean "
                + $"{result.MeanGateThroughRate:F3} ped/s) -- looks like an intermittent jam, not a stable capacity limit");
        }

        // Success condition: no permanent jam -- the approach queue region empties by the end.
        Assert.True(result.FinalQueueCount <= result.Total / 10,
            $"approach queue never cleared: {result.FinalQueueCount} agents still queued at the end of the run");
    }

    [Fact]
    public void Bottleneck_PureOrca_DeterministicAcrossIndependentRuns()
    {
        var run1 = new BottleneckScenario().Run();
        var run2 = new BottleneckScenario().Run();

        Assert.Equal(run1.Steps, run2.Steps);
        Assert.Equal(run1.Trajectory.Count, run2.Trajectory.Count);
        for (var f = 0; f < run1.Trajectory.Count; f++)
        {
            Assert.Equal(run1.Trajectory[f].Length, run2.Trajectory[f].Length);
            for (var i = 0; i < run1.Trajectory[f].Length; i++)
            {
                Assert.Equal(run1.Trajectory[f][i].X, run2.Trajectory[f][i].X, precision: 12);
                Assert.Equal(run1.Trajectory[f][i].Y, run2.Trajectory[f][i].Y, precision: 12);
            }
        }

        Assert.Equal(run1.Arrived, run2.Arrived);
        Assert.Equal(run1.PeakQueueCount, run2.PeakQueueCount);
    }
}
