using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// LIVE-POC-1 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §1, §2, §12): the ActivityTimeline generalization
// of PathArcMotion, exercised the same way PedLodManagerTests exercises PathArc -- via a real
// PedPublisher -> HeadlessIg round-trip -- but for the richer Walk/Pause/Dwell schedule, and asserting
// EXACT (not tolerance-bounded) equality since both sides call the literal same PoseAt.
public class ActivityTimelineTests
{
    private readonly ITestOutputHelper _output;

    public ActivityTimelineTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double Speed = 1.3;
    private const int PedId = 1;

    // A representative Walk -> Pause -> Walk -> Dwell(visible) -> Walk -> Dwell(hidden) -> Walk
    // timeline, matching the LIVE-POC-1 success-condition story (sips, sits at a table, enters a
    // building, re-emerges, arrives). Each Walk's authored last waypoint equals the NEXT segment's own
    // anchor coordinate exactly (same Vec2 values), so the chain is continuous by construction.
    private static ActivityTimeline BuildTimeline(double t0)
    {
        var spawn = new Vec2(-20.0, 0.0);
        var sipPoint = new Vec2(-10.0, 0.0);
        var tablePoint = new Vec2(-2.0, 3.0);
        var doorPoint = new Vec2(6.0, -2.5);
        var exit = new Vec2(18.0, 1.0);

        var segments = new ActivitySegment[]
        {
            new WalkSegment(new[] { spawn, sipPoint }, Speed),
            new PauseSegment(2.5, "sip"),
            new WalkSegment(new[] { sipPoint, tablePoint }, Speed),
            new DwellSegment(tablePoint, new Vec2(0.0, 1.0), 4.0, "sit", Visible: true),
            new WalkSegment(new[] { tablePoint, doorPoint }, Speed),
            new DwellSegment(doorPoint, new Vec2(1.0, 0.0), 3.0, "enter", Visible: false),
            new WalkSegment(new[] { doorPoint, exit }, Speed),
        };

        return new ActivityTimeline(t0, segments);
    }

    // ---- Success condition: server == IG, bit-identical, fine-dt sweep -------------------------

    [Fact]
    public void ServerAndHeadlessIg_ReconstructExactlyIdenticalPose_OverFineDtSweep()
    {
        const double t0 = 5.0;
        var timeline = BuildTimeline(t0);

        var publisher = new PedPublisher();
        publisher.PublishActivityTimeline(PedId, timeline, time: t0);
        publisher.PublishSwitch(PedId, PedDrModel.PathArc, PedDrModel.ActivityTimeline, time: t0);

        var ig = new HeadlessIg();
        ig.ApplyAll(publisher.Events);
        Assert.Equal(PedDrModel.ActivityTimeline, ig.ModelOf(PedId));

        const double dt = 0.05;
        var start = t0 - 2.0;             // sweep from BEFORE t0...
        var end = timeline.EndTime + 2.0;  // ...to PAST the very end

        var samples = 0;
        for (var now = start; now <= end; now += dt)
        {
            var serverSample = timeline.PoseAt(now);
            var igSample = ig.ReconstructSample(PedId, now);

            // EXACT equality (not a tolerance) -- both sides call the literal same ActivityTimeline
            // instance's PoseAt, so they can only ever agree, never merely "usually match".
            Assert.Equal(serverSample.Pos.X, igSample.Pos.X);
            Assert.Equal(serverSample.Pos.Y, igSample.Pos.Y);
            Assert.Equal(serverSample.Heading.X, igSample.Heading.X);
            Assert.Equal(serverSample.Heading.Y, igSample.Heading.Y);
            Assert.Equal(serverSample.AnimTag, igSample.AnimTag);
            Assert.Equal(serverSample.Visible, igSample.Visible);

            // Also exercise the position-only Reconstruct seam (same underlying model branch).
            Assert.Equal(serverSample.Pos.X, ig.Reconstruct(PedId, now).X);
            Assert.Equal(serverSample.Pos.Y, ig.Reconstruct(PedId, now).Y);

            samples++;
        }

        _output.WriteLine($"[LIVE-POC-1 measured] server-vs-IG exact-equality sweep: {samples} samples, dt={dt}s, " +
            $"window=[{start:F2},{end:F2}]s");
        Assert.True(samples > 200, $"expected a fine-dt sweep of hundreds of samples, got {samples}");
    }

    // ---- Success condition: visibility window ---------------------------------------------------

    [Fact]
    public void PoseAt_ReportsInvisible_StrictlyInsideHiddenDwell_AndVisibleOutsideIt()
    {
        const double t0 = 0.0;
        var timeline = BuildTimeline(t0);

        // Locate the hidden Dwell segment (AnimTag "enter", Visible=false) and its [t0,t1) window.
        var hiddenIndex = -1;
        for (var i = 0; i < timeline.Segments.Count; i++)
        {
            if (timeline.Segments[i] is DwellSegment { Visible: false })
            {
                hiddenIndex = i;
                break;
            }
        }

        Assert.True(hiddenIndex >= 0, "test timeline must contain a hidden Dwell segment");

        var windowStart = timeline.StartOffsetOf(hiddenIndex) + t0;
        var windowDuration = ((DwellSegment)timeline.Segments[hiddenIndex]).Dur;
        var windowEnd = windowStart + windowDuration;

        // Strictly inside: invisible.
        Assert.False(timeline.PoseAt(windowStart + (windowDuration / 2.0)).Visible);
        Assert.False(timeline.PoseAt(windowStart + 1e-6).Visible);
        Assert.False(timeline.PoseAt(windowEnd - 1e-6).Visible);

        // Outside (before the window, and at/after it ends): visible.
        Assert.True(timeline.PoseAt(windowStart - 1e-6).Visible);
        Assert.True(timeline.PoseAt(windowEnd).Visible);
        Assert.True(timeline.PoseAt(windowEnd + 0.5).Visible);

        // Before T0 and past the very end of the whole timeline: also visible (clamp).
        Assert.True(timeline.PoseAt(t0 - 10.0).Visible);
        Assert.True(timeline.PoseAt(timeline.EndTime + 10.0).Visible);
    }

    // ---- Success condition: continuity + no NaN --------------------------------------------------

    [Fact]
    public void PoseAt_IsContinuousAtSegmentBoundaries_AndNeverNaN_OverTheFullSweep()
    {
        const double t0 = 3.0;
        var timeline = BuildTimeline(t0);

        // Structural continuity: every Walk segment's OWN authored end waypoint equals the timeline's
        // OWN precomputed anchor (start pose) for the segment right after it -- the exact chain PoseAt
        // relies on, checked against the real computed anchors (not re-derived by the test).
        for (var i = 0; i < timeline.Segments.Count - 1; i++)
        {
            if (timeline.Segments[i] is WalkSegment w)
            {
                var end = w.Path[^1];
                var nextStart = timeline.StartPoseOf(i + 1);
                Assert.Equal(end.X, nextStart.X);
                Assert.Equal(end.Y, nextStart.Y);
            }
        }

        // No NaN anywhere over a fine sweep spanning before T0 to past the end.
        const double dt = 0.05;
        var start = t0 - 1.0;
        var end2 = timeline.EndTime + 1.0;
        for (var now = start; now <= end2; now += dt)
        {
            var sample = timeline.PoseAt(now);
            Assert.False(double.IsNaN(sample.Pos.X) || double.IsNaN(sample.Pos.Y), $"NaN position at t={now}");
            Assert.False(double.IsNaN(sample.Heading.X) || double.IsNaN(sample.Heading.Y), $"NaN heading at t={now}");
            Assert.NotNull(sample.AnimTag);
        }
    }

    // ---- Walk-only timeline stays byte-identical to plain PathArc ---------------------------------

    [Fact]
    public void WalkOnlyTimeline_MatchesPathArcMotion_Exactly()
    {
        var path = new[] { new Vec2(0.0, 0.0), new Vec2(10.0, 0.0), new Vec2(10.0, 10.0) };
        const double t0 = 2.0;
        var timeline = new ActivityTimeline(t0, new ActivitySegment[] { new WalkSegment(path, Speed) });

        // Strictly before arrival: ActivityTimeline.PoseAt's Walk branch calls PathArcMotion.PositionAt
        // with the identical (path, relative-zero start, speed, elapsed) arithmetic PathArcMotion.
        // PositionAt(path, t0, speed, now) computes directly -- same expression, same operands, so the
        // two calls are bit-for-bit identical for every `now` up to (not including) EndTime.
        for (var now = t0 - 1.0; now < timeline.EndTime; now += 0.05)
        {
            var expected = PathArcMotion.PositionAt(path, t0, Speed, Math.Max(now, t0));
            var sample = timeline.PoseAt(now);
            Assert.Equal(expected.X, sample.Pos.X);
            Assert.Equal(expected.Y, sample.Pos.Y);
        }

        // At/after arrival: clamped to the path's own final vertex exactly (no further motion) --
        // ActivityTimeline stores this vertex directly rather than re-deriving it via arc length, so
        // this is checked against the vertex itself rather than a possibly-rounded PathArcMotion call
        // evaluated exactly at the boundary.
        Assert.Equal(path[^1].X, timeline.PoseAt(timeline.EndTime).Pos.X);
        Assert.Equal(path[^1].Y, timeline.PoseAt(timeline.EndTime).Pos.Y);
        Assert.Equal(path[^1].X, timeline.PoseAt(timeline.EndTime + 5.0).Pos.X);
        Assert.Equal(path[^1].Y, timeline.PoseAt(timeline.EndTime + 5.0).Pos.Y);
    }
}
