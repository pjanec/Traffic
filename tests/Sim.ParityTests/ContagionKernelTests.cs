using Sim.Evac;
using Xunit;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE2-DESIGN.md §4 / PANIC-EVAC-PHASE2-TASKS.md T1.2: pure contagion proximity kernel.
public class ContagionKernelTests
{
    [Fact]
    public void AtZeroDistance_WeightIsOne()
    {
        Assert.Equal(1.0, FearField.ContagionWeight(0.0, 25.0));
    }

    [Fact]
    public void AtRadius_WeightIsZero()
    {
        Assert.Equal(0.0, FearField.ContagionWeight(25.0, 25.0));
    }

    [Fact]
    public void BeyondRadius_WeightIsZero()
    {
        Assert.Equal(0.0, FearField.ContagionWeight(30.0, 25.0));
    }

    [Fact]
    public void AtHalfRadius_WeightIsHalf()
    {
        Assert.Equal(0.5, FearField.ContagionWeight(12.5, 25.0), 6);
    }

    [Fact]
    public void MonotoneDecreasing_OnZeroToRadius()
    {
        const double radius = 25.0;
        var prev = FearField.ContagionWeight(0.0, radius);
        for (var d = 1.0; d <= radius; d += 1.0)
        {
            var w = FearField.ContagionWeight(d, radius);
            Assert.True(w <= prev, $"weight not monotone decreasing at d={d}: {w} > {prev}");
            prev = w;
        }
    }

    [Fact]
    public void NonPositiveRadius_IsZero()
    {
        Assert.Equal(0.0, FearField.ContagionWeight(0.0, 0.0));
        Assert.Equal(0.0, FearField.ContagionWeight(5.0, -1.0));
    }
}
