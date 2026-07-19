using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// LIVE-POC-2 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §5, §12): SocialPlanner pairs two peds' nominal
// Walk-only plans into matching Walk->Interact->Walk timelines, agreed ENTIRELY at schedule time (no
// runtime negotiation between the two peds). Mirrors ActivityTimelineTests's style -- a real
// PedPublisher -> HeadlessIg round-trip, exact (not tolerance-bounded) equality for the server==IG
// claim -- plus the LIVE-POC-2-specific coordination/geometry assertions.
public class SocialPlannerTests
{
    private readonly ITestOutputHelper _output;

    public SocialPlannerTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double Speed = 1.3;
    private const double MeetOffset = 0.6;
    private const double TalkDuration = 4.0;
    private const int PedAId = 1;
    private const int PedBId = 2;

    // Two peds on CROSSING approaches (pedA west->east, pedB south->north), meeting near the crossing
    // point (0,0) -- a clean, unique nearest-approach point (the two segments actually intersect), so
    // the planner's geometry is exercised on a well-conditioned, non-degenerate case.
    private static (PedPlan A, PedPlan B) BuildPlans()
    {
        var pedA = new PedPlan(PedAId, new[] { new Vec2(-10.0, 0.0), new Vec2(10.0, 0.0) }, 0.0, Speed);
        var pedB = new PedPlan(PedBId, new[] { new Vec2(0.0, -10.0), new Vec2(0.0, 10.0) }, 0.0, Speed);
        return (pedA, pedB);
    }

    // ---- Success condition: server == IG, bit-identical, fine-dt sweep, for BOTH interacting peds ----

    [Fact]
    public void ServerAndHeadlessIg_ReconstructExactlyIdenticalPose_ForBothPeds_OverFineDtSweep()
    {
        var (pedA, pedB) = BuildPlans();
        var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        var publisher = new PedPublisher();
        publisher.PublishActivityTimeline(PedAId, timelineA, time: 0.0);
        publisher.PublishSwitch(PedAId, PedDrModel.PathArc, PedDrModel.ActivityTimeline, time: 0.0);
        publisher.PublishActivityTimeline(PedBId, timelineB, time: 0.0);
        publisher.PublishSwitch(PedBId, PedDrModel.PathArc, PedDrModel.ActivityTimeline, time: 0.0);

        var ig = new HeadlessIg();
        ig.ApplyAll(publisher.Events);
        Assert.Equal(PedDrModel.ActivityTimeline, ig.ModelOf(PedAId));
        Assert.Equal(PedDrModel.ActivityTimeline, ig.ModelOf(PedBId));

        const double dt = 0.05;
        var start = Math.Min(timelineA.T0, timelineB.T0) - 2.0;
        var end = Math.Max(timelineA.EndTime, timelineB.EndTime) + 2.0;

        var samplesA = 0;
        var samplesB = 0;
        for (var now = start; now <= end; now += dt)
        {
            AssertExactMatch(timelineA, ig, PedAId, now);
            samplesA++;
            AssertExactMatch(timelineB, ig, PedBId, now);
            samplesB++;
        }

        _output.WriteLine(
            $"[LIVE-POC-2 measured] server-vs-IG exact-equality sweep: pedA={samplesA} samples, " +
            $"pedB={samplesB} samples, dt={dt}s, window=[{start:F2},{end:F2}]s");
        Assert.True(samplesA > 200, $"expected a fine-dt sweep of hundreds of samples for pedA, got {samplesA}");
        Assert.True(samplesB > 200, $"expected a fine-dt sweep of hundreds of samples for pedB, got {samplesB}");
    }

    private static void AssertExactMatch(ActivityTimeline timeline, HeadlessIg ig, int id, double now)
    {
        var serverSample = timeline.PoseAt(now);
        var igSample = ig.ReconstructSample(id, now);

        // EXACT equality (not a tolerance) -- both sides call the literal same ActivityTimeline
        // instance's PoseAt, so they can only ever agree, never merely "usually match".
        Assert.Equal(serverSample.Pos.X, igSample.Pos.X);
        Assert.Equal(serverSample.Pos.Y, igSample.Pos.Y);
        Assert.Equal(serverSample.Heading.X, igSample.Heading.X);
        Assert.Equal(serverSample.Heading.Y, igSample.Heading.Y);
        Assert.Equal(serverSample.AnimTag, igSample.AnimTag);
        Assert.Equal(serverSample.Visible, igSample.Visible);
    }

    // ---- Success condition: temporal coordination (load-bearing) ---------------------------------

    [Fact]
    public void BothPeds_InteractOnMatchingAbsoluteTimeWindow_AndBothTalk_AtASharedInstant()
    {
        var (pedA, pedB) = BuildPlans();
        var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        var (windowA, interactA) = FindInteractWindow(timelineA);
        var (windowB, interactB) = FindInteractWindow(timelineB);

        // The planner authors BOTH peds onto the SAME absolute [T, T+D] window -- this is the crux of
        // "agreed at schedule time, no runtime negotiation": the two windows don't just overlap, they
        // are identical.
        Assert.Equal(windowA.Start, windowB.Start, 9);
        Assert.Equal(windowA.End, windowB.End, 9);
        Assert.True(windowA.End > windowA.Start, "the Interact window must have positive duration");

        // Overlap check stated independently of the equality above (the load-bearing coordination
        // claim): the two windows overlap.
        var overlapStart = Math.Max(windowA.Start, windowB.Start);
        var overlapEnd = Math.Min(windowA.End, windowB.End);
        Assert.True(overlapEnd > overlapStart, "pedA and pedB's Interact windows must overlap");

        // At an instant inside the overlap, BOTH peds report AnimTag == "talk" and each is at its OWN
        // meet pose (the exact Pose/Heading the InteractSegment carries), within a tiny epsilon.
        var mid = (overlapStart + overlapEnd) / 2.0;
        var sampleA = timelineA.PoseAt(mid);
        var sampleB = timelineB.PoseAt(mid);

        Assert.Equal(SocialPlanner.TalkAnimTag, sampleA.AnimTag);
        Assert.Equal(SocialPlanner.TalkAnimTag, sampleB.AnimTag);
        Assert.True(sampleA.Visible);
        Assert.True(sampleB.Visible);

        const double eps = 1e-9;
        Assert.True((sampleA.Pos - interactA.Pose).Abs < eps, "pedA must be at its own authored meet pose");
        Assert.True((sampleB.Pos - interactB.Pose).Abs < eps, "pedB must be at its own authored meet pose");

        _output.WriteLine(
            $"[LIVE-POC-2 measured] Interact windows: A=[{windowA.Start:F3},{windowA.End:F3}] " +
            $"B=[{windowB.Start:F3},{windowB.End:F3}] overlap=[{overlapStart:F3},{overlapEnd:F3}] mid={mid:F3}");
    }

    private static ((double Start, double End) Window, InteractSegment Segment) FindInteractWindow(
        ActivityTimeline timeline)
    {
        for (var i = 0; i < timeline.Segments.Count; i++)
        {
            if (timeline.Segments[i] is InteractSegment interact)
            {
                var start = timeline.T0 + timeline.StartOffsetOf(i);
                return ((start, start + interact.Dur), interact);
            }
        }

        throw new InvalidOperationException("timeline has no InteractSegment");
    }

    // ---- Success condition: stepped aside ---------------------------------------------------------

    [Fact]
    public void InteractPose_IsSteppedAsideFromTheNominalCenterline_ForBothPeds()
    {
        var (pedA, pedB) = BuildPlans();
        var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        var (_, interactA) = FindInteractWindow(timelineA);
        var (_, interactB) = FindInteractWindow(timelineB);

        // pedA's nominal centerline is its own original path, y=0 from x=-10 to x=10; pedB's is x=0
        // from y=-10 to y=10. The two paths cross at a right angle at (0,0), the offset axis is the
        // PERPENDICULAR BISECTOR of the two approach directions (SocialPlanner's flow axis), so BOTH
        // peds land off their own centerline by EXACTLY meetOffset * sqrt(2)/2 -- an exact geometric
        // identity for a 90-degree crossing, not just "nonzero".
        var expectedOffset = MeetOffset * Math.Sqrt(2.0) / 2.0;

        var distFromA = DistancePointToLine(interactA.Pose, new Vec2(-10.0, 0.0), new Vec2(10.0, 0.0));
        var distFromB = DistancePointToLine(interactB.Pose, new Vec2(0.0, -10.0), new Vec2(0.0, 10.0));

        _output.WriteLine(
            $"[LIVE-POC-2 measured] stepped-aside distance: pedA={distFromA:F6} pedB={distFromB:F6} " +
            $"expected={expectedOffset:F6}");

        Assert.True(distFromA > 1e-6, "pedA's meet pose must NOT be on its own nominal centerline");
        Assert.True(distFromB > 1e-6, "pedB's meet pose must NOT be on its own nominal centerline");
        Assert.Equal(expectedOffset, distFromA, 6);
        Assert.Equal(expectedOffset, distFromB, 6);
    }

    private static double DistancePointToLine(Vec2 p, Vec2 lineA, Vec2 lineB)
    {
        var dir = (lineB - lineA).Normalized();
        var rel = p - lineA;
        var cross = (dir.X * rel.Y) - (dir.Y * rel.X);
        return Math.Abs(cross);
    }

    // ---- Success condition: facing ------------------------------------------------------------------

    [Fact]
    public void InteractHeadings_PointRoughlyTowardEachOther()
    {
        var (pedA, pedB) = BuildPlans();
        var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        var (_, interactA) = FindInteractWindow(timelineA);
        var (_, interactB) = FindInteractWindow(timelineB);

        var dot = Vec2.Dot(interactA.Heading, interactB.Heading);
        _output.WriteLine($"[LIVE-POC-2 measured] dot(headingA, headingB) = {dot:F6}");

        Assert.True(dot < 0.0, "the two Interact headings must point roughly toward each other");
        // For this construction the two headings are exactly opposite (each looks straight at the
        // other's meet pose), so the dot product is exactly -1.
        Assert.Equal(-1.0, dot, 9);
    }

    // ---- Success condition: partner identity ---------------------------------------------------------

    [Fact]
    public void InteractSegments_NamePartnerIdCorrectly()
    {
        var (pedA, pedB) = BuildPlans();
        var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        var (_, interactA) = FindInteractWindow(timelineA);
        var (_, interactB) = FindInteractWindow(timelineB);

        Assert.Equal(PedBId, interactA.PartnerId);
        Assert.Equal(PedAId, interactB.PartnerId);
    }

    // ---- Success condition: determinism ---------------------------------------------------------------

    [Fact]
    public void ScheduleInteraction_IsDeterministic_SamePlansYieldIdenticalTimelines()
    {
        var (pedA, pedB) = BuildPlans();

        var (timelineA1, timelineB1) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);
        var (timelineA2, timelineB2) = SocialPlanner.ScheduleInteraction(pedA, pedB, MeetOffset, TalkDuration);

        const double dt = 0.05;
        var start = Math.Min(timelineA1.T0, timelineB1.T0) - 1.0;
        var end = Math.Max(timelineA1.EndTime, timelineB1.EndTime) + 1.0;

        var samples = 0;
        for (var now = start; now <= end; now += dt)
        {
            var a1 = timelineA1.PoseAt(now);
            var a2 = timelineA2.PoseAt(now);
            Assert.Equal(a1.Pos.X, a2.Pos.X);
            Assert.Equal(a1.Pos.Y, a2.Pos.Y);
            Assert.Equal(a1.Heading.X, a2.Heading.X);
            Assert.Equal(a1.Heading.Y, a2.Heading.Y);
            Assert.Equal(a1.AnimTag, a2.AnimTag);
            Assert.Equal(a1.Visible, a2.Visible);

            var b1 = timelineB1.PoseAt(now);
            var b2 = timelineB2.PoseAt(now);
            Assert.Equal(b1.Pos.X, b2.Pos.X);
            Assert.Equal(b1.Pos.Y, b2.Pos.Y);
            Assert.Equal(b1.AnimTag, b2.AnimTag);
            Assert.Equal(b1.Visible, b2.Visible);

            samples++;
        }

        Assert.True(samples > 50, $"expected a meaningful determinism sweep, got {samples}");

        // Also directly compare the two InteractSegments' authored poses/times (not just PoseAt).
        var (windowA1, interactA1) = FindInteractWindow(timelineA1);
        var (windowA2, interactA2) = FindInteractWindow(timelineA2);
        Assert.Equal(interactA1.Pose.X, interactA2.Pose.X);
        Assert.Equal(interactA1.Pose.Y, interactA2.Pose.Y);
        Assert.Equal(windowA1.Start, windowA2.Start);
        Assert.Equal(windowA1.End, windowA2.End);
    }
}
