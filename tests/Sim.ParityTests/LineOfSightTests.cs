using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Evac;
using Xunit;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE2-DESIGN.md §3 / PANIC-EVAC-PHASE2-TASKS.md T1.1: pure segment-vs-disc occlusion.
public class LineOfSightTests
{
    private static readonly Vec2 From = new(0.0, 0.0);
    private static readonly Vec2 Target = new(100.0, 0.0);

    [Fact]
    public void NoOccluders_IsVisible()
    {
        Assert.True(LineOfSight.IsVisible(From, Target, Array.Empty<WorldDisc>()));
        Assert.True(LineOfSight.IsVisible(From, Target, null!));
    }

    [Fact]
    public void OccluderOnSegmentMidpoint_Blocks()
    {
        var occluders = new List<WorldDisc> { new(50.0, 0.0, 0.0, 0.0, 2.0) };
        Assert.False(LineOfSight.IsVisible(From, Target, occluders));
    }

    [Fact]
    public void OccluderNearButFarEnoughOff_DoesNotBlock()
    {
        var occluders = new List<WorldDisc> { new(50.0, 5.0, 0.0, 0.0, 2.0) };
        Assert.True(LineOfSight.IsVisible(From, Target, occluders));
    }

    [Fact]
    public void OccluderBeyondTarget_DoesNotBlockEvenIfLarge()
    {
        var occluders = new List<WorldDisc> { new(150.0, 0.0, 0.0, 0.0, 40.0) };
        Assert.True(LineOfSight.IsVisible(From, Target, occluders));
    }

    [Fact]
    public void OccluderBehindFrom_DoesNotBlockEvenIfLarge()
    {
        var occluders = new List<WorldDisc> { new(-50.0, 0.0, 0.0, 0.0, 40.0) };
        Assert.True(LineOfSight.IsVisible(From, Target, occluders));
    }
}
