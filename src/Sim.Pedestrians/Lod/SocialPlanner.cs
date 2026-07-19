using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// LIVE-POC-2 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §5, §12): the pre-scheduled social planner. "Two
// peds meet, step aside, and talk" is the one liveliness beat that needs TWO agents to agree, and the
// whole point of this planner is that the agreement happens ONCE, at schedule-generation time, as a
// pure function of the two peds' nominal plans -- never at runtime. It writes MATCHING `Interact`
// segments into BOTH timelines (same absolute [T, T+D] window, headings aimed at each other, meet
// poses offset to opposite sides of the flow) and hands back two ordinary `ActivityTimeline`s. Nothing
// downstream (PedPublisher, HeadlessIg, ActivityTimeline.PoseAt) needs to know a planner was involved --
// this is exactly why it stays low-power: authored-together + deterministic => self-consistent =>
// IG-reproducible => no promotion needed (§5's "no runtime negotiation" argument).
//
// No System.Random anywhere: every number here is derived from the two input PedPlans by closed-form
// geometry, so calling ScheduleInteraction twice with the same inputs always returns bit-identical
// timelines.
public readonly record struct PedPlan(int Id, IReadOnlyList<Vec2> Path, double StartTime, double Speed);

public static class SocialPlanner
{
    // Animation tag played by both peds for the whole Interact window (docs §3 tag vocabulary --
    // "talk" is the LIVE-POC-2 addition alongside "sip"/"sit"/"enter" from LIVE-POC-1).
    public const string TalkAnimTag = "talk";

    // Default half-separation (metres) each ped stands from the shared meet point -- the two peds end
    // up 2*meetOffset apart, comfortably clear of typical ped radii (~0.3m) so they never overlap
    // (LIVE-POC-2 success condition, docs §12).
    public const double DefaultMeetOffset = 0.6;

    // Default conversation length (seconds).
    public const double DefaultDuration = 4.0;

    // Pairs pedA and pedB and rewrites their nominal Walk-only plans into a Walk -> Interact -> Walk
    // timeline EACH, sharing one agreed meet point/time/duration (docs §5):
    //
    //   1. Find where the two paths pass closest to each other (the "meet region") -- a plain
    //      segment-segment nearest-approach over the two polylines, no search/iteration needed.
    //   2. The meet point is the midpoint of that closest pair; each ped's own meet POSE is offset
    //      `meetOffset` to one side of it, perpendicular to pedA's direction of travel through the meet
    //      segment (the "step aside from the flow" beat) -- pedA and pedB land on OPPOSITE sides, so
    //      they end up facing each other across a small gap.
    //   3. The meet TIME T is the LATER of the two peds' own natural (unsynchronized) arrival times at
    //      their meet pose (walking their nominal Speed) -- the earlier-arriving ped's approach leg is
    //      re-timed (a new, still-constant approach speed, NOT its nominal Speed) to land at exactly T,
    //      so neither ped waits or teleports. This re-timing is itself a pure function of the two
    //      nominal plans, so it costs nothing to reproduce on the IG.
    //   4. Both peds get IDENTICAL [T, T+duration) Interact windows and resume their ORIGINAL onward
    //      route (the tail of their own input Path) afterward at their own nominal Speed.
    public static (ActivityTimeline TimelineA, ActivityTimeline TimelineB) ScheduleInteraction(
        PedPlan pedA, PedPlan pedB, double meetOffset = DefaultMeetOffset, double duration = DefaultDuration)
    {
        if (pedA.Path.Count < 2)
        {
            throw new ArgumentException("pedA's path must have at least two waypoints.", nameof(pedA));
        }

        if (pedB.Path.Count < 2)
        {
            throw new ArgumentException("pedB's path must have at least two waypoints.", nameof(pedB));
        }

        var (pointOnA, pointOnB, segA, segB) = ClosestApproach(pedA.Path, pedB.Path);
        var meetPoint = (pointOnA + pointOnB) * 0.5;

        // Offset axis: perpendicular to the BISECTOR of the two peds' own directions of travel through
        // the meet segment ("step off the line of the flow", not off the axis connecting the two
        // peds). Using the bisector (dirA - dirB), rather than either ped's direction alone, keeps the
        // step-aside symmetric between A and B for the common case of two peds converging almost
        // head-on (dirA ~= -dirB, bisector ~= 2*dirA, so this reduces to "perpendicular to the shared
        // corridor") while still giving BOTH peds a genuine (if unequal for very oblique crossings)
        // deviation off their own line when the two approaches cross at an angle -- perpendicular to
        // pedA's direction ALONE would give pedB zero deviation in the perpendicular-crossing case,
        // since pedA's perpendicular is then exactly pedB's own direction of travel.
        var dirA = Direction(pedA.Path[segA], pedA.Path[segA + 1]);
        var dirB = Direction(pedB.Path[segB], pedB.Path[segB + 1]);
        var bisector = dirA - dirB;
        var flowAxis = bisector.AbsSq > 1e-12
            ? bisector.Normalized()
            : (dirA.AbsSq > 1e-12 ? dirA : new Vec2(1.0, 0.0));
        var offsetAxis = flowAxis.PerpCW;
        var offsetVec = offsetAxis * meetOffset;

        var meetPosA = meetPoint - offsetVec;
        var meetPosB = meetPoint + offsetVec;

        // Each faces the OTHER's meet pose -- by construction these are opposite unit vectors, so
        // dot(headingA, headingB) == -1 (they look straight at each other).
        var headingA = (meetPosB - meetPosA).Normalized();
        var headingB = (meetPosA - meetPosB).Normalized();

        var approachLenA = ApproachLength(pedA.Path, segA, meetPosA);
        var approachLenB = ApproachLength(pedB.Path, segB, meetPosB);

        var naturalArrivalA = pedA.StartTime + (pedA.Speed > 0.0 ? approachLenA / pedA.Speed : 0.0);
        var naturalArrivalB = pedB.StartTime + (pedB.Speed > 0.0 ? approachLenB / pedB.Speed : 0.0);
        var meetTime = Math.Max(naturalArrivalA, naturalArrivalB);

        var timelineA = BuildTimeline(pedA, segA, meetPosA, headingA, meetTime, duration, approachLenA, pedB.Id);
        var timelineB = BuildTimeline(pedB, segB, meetPosB, headingB, meetTime, duration, approachLenB, pedA.Id);

        return (timelineA, timelineB);
    }

    private static ActivityTimeline BuildTimeline(
        PedPlan ped,
        int segIndex,
        Vec2 meetPos,
        Vec2 heading,
        double meetTime,
        double duration,
        double approachLength,
        int partnerId)
    {
        // Re-time the approach so this ped lands on meetPos at exactly `meetTime` -- a pure function of
        // (ped.StartTime, meetTime, approachLength), never the nominal Speed once the two agree on T.
        var approachDuration = Math.Max(meetTime - ped.StartTime, 1e-6);
        var approachSpeed = approachLength / approachDuration;

        var approachPath = new List<Vec2>(segIndex + 2);
        for (var i = 0; i <= segIndex; i++)
        {
            approachPath.Add(ped.Path[i]);
        }

        approachPath.Add(meetPos);

        var onwardPath = new List<Vec2> { meetPos };
        for (var i = segIndex + 1; i < ped.Path.Count; i++)
        {
            onwardPath.Add(ped.Path[i]);
        }

        var segments = new ActivitySegment[]
        {
            new WalkSegment(approachPath, approachSpeed),
            new InteractSegment(meetPos, heading, duration, TalkAnimTag, partnerId),
            new WalkSegment(onwardPath, ped.Speed),
        };

        return new ActivityTimeline(ped.StartTime, segments);
    }

    // Arc-length from `path[0]` to `meetPos`, where `meetPos` replaces `path[segIndex + 1]` as the
    // detour target off segment `segIndex` (mirrors PathArcMotion.PathLength's summation).
    private static double ApproachLength(IReadOnlyList<Vec2> path, int segIndex, Vec2 meetPos)
    {
        var total = 0.0;
        for (var i = 0; i < segIndex; i++)
        {
            total += (path[i + 1] - path[i]).Abs;
        }

        total += (meetPos - path[segIndex]).Abs;
        return total;
    }

    private static Vec2 Direction(Vec2 a, Vec2 b)
    {
        var d = b - a;
        return d.Abs > 1e-12 ? d.Normalized() : Vec2.Zero;
    }

    // Nearest-approach point between two polylines: the closest pair of points across every
    // segment-vs-segment combination. Paths are short (a handful of waypoints), so the plain O(n*m)
    // scan is deliberately simple and robust rather than clever -- the sophisticated pairing rule is an
    // explicitly open item (docs §13); this is the "clean, deterministic, visibly-correct" POC bar.
    private static (Vec2 PointOnA, Vec2 PointOnB, int SegA, int SegB) ClosestApproach(
        IReadOnlyList<Vec2> a, IReadOnlyList<Vec2> b)
    {
        var best = double.PositiveInfinity;
        var bestPa = a[0];
        var bestPb = b[0];
        var bestSegA = 0;
        var bestSegB = 0;

        for (var i = 0; i + 1 < a.Count; i++)
        {
            for (var j = 0; j + 1 < b.Count; j++)
            {
                var (pa, pb) = ClosestSegmentPoints(a[i], a[i + 1], b[j], b[j + 1]);
                var d = (pa - pb).AbsSq;
                if (d < best)
                {
                    best = d;
                    bestPa = pa;
                    bestPb = pb;
                    bestSegA = i;
                    bestSegB = j;
                }
            }
        }

        return (bestPa, bestPb, bestSegA, bestSegB);
    }

    // Closest points between two finite segments [p1,p2] and [p3,p4] -- the standard clamped-parametric
    // segment-segment distance construction (closed-form, no iteration).
    private static (Vec2 OnFirst, Vec2 OnSecond) ClosestSegmentPoints(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4)
    {
        var d1 = p2 - p1;
        var d2 = p4 - p3;
        var r = p1 - p3;

        var a = Vec2.Dot(d1, d1);
        var e = Vec2.Dot(d2, d2);
        var f = Vec2.Dot(d2, r);

        double s, t;
        if (a <= 1e-12 && e <= 1e-12)
        {
            s = 0.0;
            t = 0.0;
        }
        else if (a <= 1e-12)
        {
            s = 0.0;
            t = Clamp01(f / e);
        }
        else
        {
            var c = Vec2.Dot(d1, r);
            if (e <= 1e-12)
            {
                t = 0.0;
                s = Clamp01(-c / a);
            }
            else
            {
                var b = Vec2.Dot(d1, d2);
                var denom = (a * e) - (b * b);
                s = denom > 1e-12 ? Clamp01(((b * f) - (c * e)) / denom) : 0.0;
                t = Clamp01(((b * s) + f) / e);
                s = Clamp01(((b * t) - c) / a);
            }
        }

        var onFirst = p1 + (d1 * s);
        var onSecond = p3 + (d2 * t);
        return (onFirst, onSecond);
    }

    private static double Clamp01(double v) => Math.Max(0.0, Math.Min(1.0, v));
}
