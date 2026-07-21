using Sim.IgBridge;
using Xunit;

namespace Sim.IgBridge.Tests;

// T1.3 (docs/IGBRIDGE-TASKS.md): the FakeIg replays a trace exactly as the real IG would -- 2-sample
// linear position interpolation, shortest-arc heading, and an immediate jump (not a smear) when the
// bracketing pair is farther apart than the jump threshold.
public sealed class FakeIgTests
{
    private static IgSample New(string id, double t, double x, double y, float h)
        => IgSample.Created(id, t, IgEntityModel.Car, x, y, 0.0, h);

    private static IgSample Upd(string id, double t, double x, double y, float h)
        => IgSample.Updated(id, t, x, y, 0.0, h);

    [Fact]
    public void Reconstruct_InterpolatesPositionLinearlyBetweenSamples()
    {
        var trace = new[]
        {
            New("v1", 0.0, 0.0, 0.0, 90f),
            Upd("v1", 1.0, 10.0, 0.0, 90f),
            Upd("v1", 2.0, 20.0, 0.0, 90f),
            IgSample.Removed("v1", 2.0),
        };
        var ig = new FakeIg(trace, new FakeIgConfig { RenderHz = 10.0, JumpThresholdMeters = 100.0 });
        var poses = ig.Reconstruct()["v1"];

        // At t=0.5 the reconstructed x is the midpoint of the [0,10] segment.
        var mid = poses.First(p => Math.Abs(p.T - 0.5) < 1e-6);
        Assert.Equal(5.0, mid.X, precision: 6);
        Assert.False(mid.Jumped);
        Assert.All(poses, p => Assert.False(p.Jumped));
    }

    [Fact]
    public void Reconstruct_ShortestArcHeading_CrossesNorthNotTheLongWay()
    {
        var trace = new[]
        {
            New("v1", 0.0, 0.0, 0.0, 350f),
            Upd("v1", 1.0, 1.0, 0.0, 10f),
            IgSample.Removed("v1", 1.0),
        };
        var ig = new FakeIg(trace, new FakeIgConfig { RenderHz = 10.0, JumpThresholdMeters = 100.0 });
        var poses = ig.Reconstruct()["v1"];

        // 350 -> 10 the short way passes through 0 (=360), so the midpoint is ~0, NOT ~180.
        var mid = poses.First(p => Math.Abs(p.T - 0.5) < 1e-6);
        var h = mid.HeadingDeg;
        Assert.True(h < 5f || h > 355f, $"expected heading near 0, got {h}");
    }

    [Fact]
    public void Reconstruct_JumpsInsteadOfSmearing_WhenPairExceedsThreshold()
    {
        var trace = new[]
        {
            New("v1", 0.0, 0.0, 0.0, 90f),
            Upd("v1", 1.0, 100.0, 0.0, 90f), // 100 m gap between adjacent samples
            IgSample.Removed("v1", 1.0),
        };
        var ig = new FakeIg(trace, new FakeIgConfig { RenderHz = 10.0, JumpThresholdMeters = 8.0 });
        var poses = ig.Reconstruct()["v1"];

        // No reconstructed pose may sit in the smeared middle of the teleport (e.g. x≈50); each is snapped
        // to one endpoint and flagged Jumped.
        foreach (var p in poses)
        {
            Assert.True(p.Jumped);
            Assert.True(p.X < 1e-6 || p.X > 100.0 - 1e-6, $"jump smeared to x={p.X}");
        }
    }
}
