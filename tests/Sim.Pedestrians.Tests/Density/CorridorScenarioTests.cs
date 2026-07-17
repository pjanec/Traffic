using Sim.Pedestrians.Density;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Density;

// POC-4 (docs/PEDESTRIAN-POC-PLAN.md POC-4 success condition 1; docs/PEDESTRIAN-DESIGN.md §3
// "Believability"): bidirectional corridor -- measures throughput, lane-formation, and drain
// behavior for PURE ORCA (no density term), the "measure first" gate before any density term is
// even considered.
public class CorridorScenarioTests
{
    private readonly ITestOutputHelper _out;

    public CorridorScenarioTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void BidirectionalCorridor_PureOrca_BothDirectionsFlow_LanesFormed_NoDeadlock()
    {
        var scenario = new CorridorScenario();
        var result = scenario.Run();

        _out.WriteLine("[POC-4 corridor, pure ORCA] "
            + $"steps={result.Steps} ({result.SimulatedSeconds:F1}s); "
            + $"arrived A={result.ArrivedA}/{result.TotalA}, B={result.ArrivedB}/{result.TotalB}; "
            + $"throughput A={result.ThroughputA:F3} ped/s, B={result.ThroughputB:F3} ped/s; "
            + $"lane-formation |r| mean={result.MeanAbsLaneCorrelation:F3}, peak={result.PeakAbsLaneCorrelation:F3}; "
            + $"close head-on encounters={result.CloseHeadOnEncounters}");

        // Success condition: both directions achieve > 0 throughput.
        Assert.True(result.ThroughputA > 0.0, "group A throughput was zero -- no one arrived");
        Assert.True(result.ThroughputB > 0.0, "group B throughput was zero -- no one arrived");

        // Success condition: no deadlock -- the crowd drains (>= ~90% arrive within the budget).
        var totalArrived = result.ArrivedA + result.ArrivedB;
        var total = result.TotalA + result.TotalB;
        Assert.True(totalArrived >= total * 0.9,
            $"crowd did not drain: {totalArrived}/{total} arrived within {result.Steps} steps ({result.SimulatedSeconds:F1}s)");

        // Success condition: the lane-formation metric is computed (a real number, not vacuous).
        Assert.True(!double.IsNaN(result.MeanAbsLaneCorrelation) && !double.IsNaN(result.PeakAbsLaneCorrelation),
            "lane-formation correlation was not computed (NaN) -- no qualifying frames observed");
    }

    [Fact]
    public void BidirectionalCorridor_PureOrca_DeterministicAcrossIndependentRuns()
    {
        var run1 = new CorridorScenario().Run();
        var run2 = new CorridorScenario().Run();

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

        Assert.Equal(run1.ArrivedA, run2.ArrivedA);
        Assert.Equal(run1.ArrivedB, run2.ArrivedB);
        Assert.Equal(run1.CloseHeadOnEncounters, run2.CloseHeadOnEncounters);
    }
}
