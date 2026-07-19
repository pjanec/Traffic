using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- Encode -> Decode round-trips a rich
// ActivityTimeline (all four segment kinds, a hidden Dwell, an Interact with a partner id, and non-ASCII
// anim tags) such that the decoded timeline's PoseAt reproduces the original's BIT-FOR-BIT over a fine
// sweep, proving the wire format (full IEEE-754 doubles, no quantization) preserves the "server == IG"
// identity across serialization.
public class ActivityTimelineWireTests
{
    private readonly ITestOutputHelper _output;

    public ActivityTimelineWireTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private static ActivityTimeline BuildRichTimeline()
    {
        return new ActivityTimeline(t0: 5.0, new ActivitySegment[]
        {
            new WalkSegment(new[] { new Vec2(0.0, 0.0), new Vec2(7.3, -2.125), new Vec2(12.0, 4.75) }, 1.37),
            new PauseSegment(1.5, "⏸ pausé"), // non-ASCII anim tag
            new DwellSegment(new Vec2(20.5, 8.125), new Vec2(0.0, 1.0), 2.25, "睡眠 (hidden)", false),
            new InteractSegment(new Vec2(25.0, 8.125), new Vec2(1.0, 0.0), 3.0, "talk-π", 42),
            new WalkSegment(new[] { new Vec2(25.0, 8.125), new Vec2(25.0, 30.0) }, 0.9),
        });
    }

    [Fact]
    public void EncodeDecode_RoundTrips_AllSegmentKinds_ExactlyOverASweep()
    {
        var original = BuildRichTimeline();
        var bytes = ActivityTimelineWire.Encode(original);
        var decoded = ActivityTimelineWire.Decode(bytes);

        Assert.Equal(original.T0, decoded.T0);
        Assert.Equal(original.Segments.Count, decoded.Segments.Count);
        Assert.Equal(original.TotalDuration, decoded.TotalDuration);

        var samples = 0;
        for (var t = original.T0 - 2.0; t <= original.EndTime + 2.0; t += 0.013)
        {
            var a = original.PoseAt(t);
            var b = decoded.PoseAt(t);

            Assert.Equal(a.Pos.X, b.Pos.X);
            Assert.Equal(a.Pos.Y, b.Pos.Y);
            Assert.Equal(a.Heading.X, b.Heading.X);
            Assert.Equal(a.Heading.Y, b.Heading.Y);
            Assert.Equal(a.AnimTag, b.AnimTag);
            Assert.Equal(a.Visible, b.Visible);
            samples++;
        }

        _output.WriteLine($"[P3-1 measured] ActivityTimelineWire exact-match samples over sweep: {samples}");
        Assert.True(samples > 100, $"expected a fine sweep (>100 samples), got {samples}");
    }

    [Fact]
    public void EncodeDecode_PreservesSegmentPayloads_PerKind()
    {
        var original = BuildRichTimeline();
        var decoded = ActivityTimelineWire.Decode(ActivityTimelineWire.Encode(original));

        var w0 = Assert.IsType<WalkSegment>(decoded.Segments[0]);
        Assert.Equal(1.37, w0.Speed);
        Assert.Equal(3, w0.Path.Count);
        Assert.Equal(7.3, w0.Path[1].X);
        Assert.Equal(-2.125, w0.Path[1].Y);

        var p1 = Assert.IsType<PauseSegment>(decoded.Segments[1]);
        Assert.Equal(1.5, p1.Dur);
        Assert.Equal("⏸ pausé", p1.AnimTag);

        var d2 = Assert.IsType<DwellSegment>(decoded.Segments[2]);
        Assert.Equal(20.5, d2.Pose.X);
        Assert.Equal(8.125, d2.Pose.Y);
        Assert.Equal(0.0, d2.Heading.X);
        Assert.Equal(1.0, d2.Heading.Y);
        Assert.Equal(2.25, d2.Dur);
        Assert.False(d2.Visible);
        Assert.Equal("睡眠 (hidden)", d2.AnimTag);

        var i3 = Assert.IsType<InteractSegment>(decoded.Segments[3]);
        Assert.Equal(25.0, i3.Pose.X);
        Assert.Equal(8.125, i3.Pose.Y);
        Assert.Equal(1.0, i3.Heading.X);
        Assert.Equal(0.0, i3.Heading.Y);
        Assert.Equal(3.0, i3.Dur);
        Assert.Equal(42, i3.PartnerId);
        Assert.Equal("talk-π", i3.AnimTag);

        var w4 = Assert.IsType<WalkSegment>(decoded.Segments[4]);
        Assert.Equal(0.9, w4.Speed);
        Assert.Equal(2, w4.Path.Count);
    }

    [Fact]
    public void EncodeDecode_RoundTrips_ASingleSegmentTimeline()
    {
        var original = new ActivityTimeline(0.0, new ActivitySegment[]
        {
            new WalkSegment(new[] { new Vec2(0.0, 0.0), new Vec2(1.0, 0.0) }, 1.0),
        });

        var decoded = ActivityTimelineWire.Decode(ActivityTimelineWire.Encode(original));
        Assert.Equal(original.EndTime, decoded.EndTime);
        Assert.Equal(original.PoseAt(0.5).Pos.X, decoded.PoseAt(0.5).Pos.X);
    }
}
