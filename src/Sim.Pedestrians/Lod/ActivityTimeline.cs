using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// LIVE-POC-1 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §1, §2, §12): generalizes PathArcMotion's pure-
// function identity to a RICHER function so low-power pedestrians can look "alive" -- sipping,
// sitting at a table, ducking into a building -- without ever becoming a per-step behavior loop.
//
// An ActivityTimeline is an ordered, precomputed list of ActivitySegments (Walk/Pause/Dwell/Interact),
// each covering a [t0, t1) slice of the ped's life. `PoseAt(now)` is the ONE evaluator: a PURE function
// of (this timeline, now) -- no mutable state, no System.Random, no neighbour queries. This is exactly
// PathArcMotion's load-bearing identity ("server == IG because they share the code"), just applied to
// a richer schedule: the server calls THIS SAME method to advance/expose a low-power ped, and the
// headless IG calls it again to reconstruct that ped from the one-time ActivityTimelineRecord
// broadcast -- so "server == IG for liveliness" follows from sharing the code, not from independently
// matching two implementations. A pure Walk-only timeline delegates its motion to PathArcMotion
// segment-by-segment, so it reproduces today's PathArc pose exactly. LIVE-POC-2 (§5, §12) adds
// `Interact`: pose-wise it IS a Dwell (its own Pose/Heading, always visible) that additionally names
// the conversation `PartnerId` -- see `InteractSegment` below -- authored PAIRWISE by SocialPlanner so
// two peds' timelines agree at schedule time rather than negotiating at runtime.
//
// Allocation-light: the per-segment start offsets/poses are computed ONCE in the constructor (a single
// forward pass); `PoseAt` itself is a linear scan over the (small, n) segment list with no allocation.
public readonly struct PoseSample
{
    public readonly Vec2 Pos;
    public readonly Vec2 Heading;
    public readonly string AnimTag;
    public readonly bool Visible;

    public PoseSample(Vec2 pos, Vec2 heading, string animTag, bool visible)
    {
        Pos = pos;
        Heading = heading;
        AnimTag = animTag;
        Visible = visible;
    }
}

public enum ActivitySegmentKind
{
    Walk,
    Pause,
    Dwell,
    Interact,
}

// One [t0, t1) slice of an ActivityTimeline (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §2 table). `Duration`
// is fixed at construction time -- Walk derives it from arc-length / speed, Pause/Dwell take it
// directly -- so ActivityTimeline can precompute every segment's cumulative start offset once, rather
// than re-deriving it on every PoseAt call.
public abstract record ActivitySegment(ActivitySegmentKind Kind, double Duration);

// Move along `Path` (today's PathArc) at `Speed`. Duration = arc-length(Path) / Speed (0 if Speed <= 0
// or the path has no length -- a degenerate but harmless Walk). AnimTag is always Walk's own
// "walk" (see ActivityTimeline.WalkAnimTag); Interact is a LATER POC, not modelled here.
public sealed record WalkSegment(IReadOnlyList<Vec2> Path, double Speed)
    : ActivitySegment(ActivitySegmentKind.Walk, ComputeDuration(Path, Speed))
{
    private static double ComputeDuration(IReadOnlyList<Vec2> path, double speed) =>
        speed > 0.0 ? PathArcMotion.PathLength(path) / speed : 0.0;
}

// Stop in place for `Dur` seconds and play `AnimTag` (e.g. "sip", "phone", "look"). Pause carries NO
// position of its own -- it holds at wherever the timeline reached when it started (almost always the
// end of the preceding Walk), which is exactly what keeps a Walk->Pause->Walk chain continuous for
// free: the anchor is inherited, never re-specified.
public sealed record PauseSegment(double Dur, string AnimTag) : ActivitySegment(ActivitySegmentKind.Pause, Dur);

// Occupy a spot (table/bench) or go inside a building for `Dur` seconds, playing `AnimTag` (e.g.
// "sit", "enter"). Unlike Pause, Dwell carries its OWN `Pose`/`Heading` (a POI may be off the walked
// path -- a detour to a table, a building's door). `Visible = false` models "inside" (§6 hidden dwell):
// PoseAt still returns a pose (so re-emergence has somewhere to resume from) but flags it non-visible,
// so the caller (HeadlessIg/SceneGen) must emit no disc for that ped while inside.
public sealed record DwellSegment(Vec2 Pose, Vec2 Heading, double Dur, string AnimTag, bool Visible)
    : ActivitySegment(ActivitySegmentKind.Dwell, Dur);

// LIVE-POC-2 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §5, §12): the "step aside and talk" beat. Exactly a
// VISIBLE Dwell for pose purposes -- it carries its OWN `Pose`/`Heading` (the meet spot, just off the
// nominal walked centerline, facing the partner) -- plus one extra field, `PartnerId`, naming the OTHER
// ped in the conversation. `PartnerId` is carried for the animation contract / tests (so an IG or a
// test can confirm which two peds are talking to each other); it plays NO role in PoseAt's pose math,
// which is why Interact can otherwise reuse Dwell's anchor/EndOf handling verbatim. Two Interact
// segments only ever "agree" because SocialPlanner authors them together at schedule time (matching
// meetPos +/- offset, matching [T, T+D] window, headings aimed at each other) -- there is no runtime
// negotiation between the two peds' timelines.
public sealed record InteractSegment(Vec2 Pose, Vec2 Heading, double Dur, string AnimTag, int PartnerId)
    : ActivitySegment(ActivitySegmentKind.Interact, Dur);

public sealed class ActivityTimeline
{
    // Animation tag Walk always plays, and the clamp tag used before T0 / after the timeline ends
    // (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §3 "missing-tag fallback = Idle/Walk").
    public const string WalkAnimTag = "walk";
    public const string IdleAnimTag = "idle";

    private readonly double[] _startOffset; // cumulative start offset from T0, per segment
    private readonly Vec2[] _startPos;      // precomputed anchor position, per segment
    private readonly Vec2[] _startHeading;  // precomputed anchor heading, per segment
    private readonly Vec2 _endPos;          // pose after the last segment ends (clamped)

    public double T0 { get; }
    public IReadOnlyList<ActivitySegment> Segments { get; }
    public double TotalDuration { get; }
    public double EndTime => T0 + TotalDuration;

    public ActivityTimeline(double t0, IReadOnlyList<ActivitySegment> segments)
    {
        if (segments.Count == 0)
        {
            throw new ArgumentException("ActivityTimeline requires at least one segment.", nameof(segments));
        }

        T0 = t0;
        Segments = segments;

        _startOffset = new double[segments.Count];
        _startPos = new Vec2[segments.Count];
        _startHeading = new Vec2[segments.Count];

        // A single forward pass: each segment's anchor (start pose/heading) is the previous segment's
        // END pose/heading -- the chain that makes Walk->Pause->Walk and Walk->Dwell->Walk continuous
        // without any segment needing to restate where the previous one left off (except Dwell, which
        // deliberately carries its own POI pose -- a detour off the walked path).
        var offset = 0.0;
        var pos = AnchorOf(segments[0]);
        var heading = Vec2.Zero;
        for (var i = 0; i < segments.Count; i++)
        {
            _startOffset[i] = offset;
            _startPos[i] = pos;
            _startHeading[i] = heading;

            (pos, heading) = EndOf(segments[i], pos, heading);
            offset += segments[i].Duration;
        }

        TotalDuration = offset;
        _endPos = pos;
    }

    // The pose at Segments[0]'s own start -- Walk's first waypoint, Dwell's own Pose. A leading Pause
    // has no anchor of its own (edge case: a timeline should not start with a Pause); Vec2.Zero is the
    // harmless fallback.
    private static Vec2 AnchorOf(ActivitySegment segment) => segment switch
    {
        WalkSegment w when w.Path.Count > 0 => w.Path[0],
        DwellSegment d => d.Pose,
        InteractSegment i => i.Pose,
        _ => Vec2.Zero,
    };

    // The pose/heading a segment leaves the ped in once it finishes -- becomes the NEXT segment's
    // anchor.
    private static (Vec2 Pos, Vec2 Heading) EndOf(ActivitySegment segment, Vec2 startPos, Vec2 startHeading) =>
        segment switch
        {
            WalkSegment w when w.Path.Count > 0 => (w.Path[^1], FinalWalkHeading(w.Path)),
            PauseSegment => (startPos, startHeading), // no movement -- keep facing the same way
            DwellSegment d => (d.Pose, d.Heading),
            InteractSegment i => (i.Pose, i.Heading),
            _ => (startPos, startHeading),
        };

    // Unit direction of the last non-degenerate polyline segment (mirrors PathArcMotion's own
    // degenerate-segment skipping), or Vec2.Zero for a single-point/empty path.
    private static Vec2 FinalWalkHeading(IReadOnlyList<Vec2> path)
    {
        for (var i = path.Count - 2; i >= 0; i--)
        {
            var seg = path[i + 1] - path[i];
            var len = seg.Abs;
            if (len > 1e-12)
            {
                return seg / len;
            }
        }

        return Vec2.Zero;
    }

    // Start offset (seconds since T0) of Segments[index] -- exposed for diagnostics/tests.
    public double StartOffsetOf(int index) => _startOffset[index];

    // Precomputed anchor pose/heading of Segments[index] -- exposed so a Walk segment's authored end
    // pose can be checked against the NEXT segment's anchor (continuity), without recomputing anything
    // PoseAt didn't already compute once at construction.
    public Vec2 StartPoseOf(int index) => _startPos[index];
    public Vec2 StartHeadingOf(int index) => _startHeading[index];

    // THE evaluator (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §2): PURE, deterministic, allocation-free.
    // Before T0 -> the first segment's start pose, Idle. At/after EndTime -> the final pose, heading
    // zeroed, Idle (arrived and stopped -- always considered visible, even if the very last segment
    // happened to be a hidden Dwell: the clamp models "already re-emerged/moved on"). Otherwise: find
    // the segment containing `now` (linear scan -- n is small, O(1) amortized per docs) and evaluate it
    // relative to its own start.
    public PoseSample PoseAt(double now)
    {
        if (now < T0)
        {
            return new PoseSample(_startPos[0], Vec2.Zero, IdleAnimTag, true);
        }

        var elapsed = now - T0;
        if (elapsed >= TotalDuration)
        {
            return new PoseSample(_endPos, Vec2.Zero, IdleAnimTag, true);
        }

        var idx = Segments.Count - 1;
        for (var i = 1; i < Segments.Count; i++)
        {
            if (_startOffset[i] > elapsed)
            {
                idx = i - 1;
                break;
            }
        }

        var segElapsed = elapsed - _startOffset[idx];
        return Evaluate(Segments[idx], _startPos[idx], _startHeading[idx], segElapsed);
    }

    // The ped's world velocity at `now` -- the segment direction * speed while Walking, Vec2.Zero while
    // Pausing/Dwelling/Interacting (standing still) and outside the timeline (clamped). Used when a
    // low-power ActivityTimeline ped is PROMOTED into the high-power OrcaCrowd (PedLodManager): the crowd
    // agent is seeded with this velocity so it continues in the same direction rather than jerking to a
    // stop. Mirrors PoseAt's segment scan exactly (same idx, same clamps) so pose and velocity agree.
    public Vec2 VelocityAt(double now)
    {
        if (now < T0)
        {
            return Vec2.Zero;
        }

        var elapsed = now - T0;
        if (elapsed >= TotalDuration)
        {
            return Vec2.Zero;
        }

        var idx = Segments.Count - 1;
        for (var i = 1; i < Segments.Count; i++)
        {
            if (_startOffset[i] > elapsed)
            {
                idx = i - 1;
                break;
            }
        }

        return Segments[idx] is WalkSegment w
            ? PathArcMotion.VelocityAt(w.Path, 0.0, w.Speed, elapsed - _startOffset[idx])
            : Vec2.Zero;
    }

    private static PoseSample Evaluate(ActivitySegment segment, Vec2 startPos, Vec2 startHeading, double segElapsed)
    {
        switch (segment)
        {
            case WalkSegment w:
                var pos = PathArcMotion.PositionAt(w.Path, 0.0, w.Speed, segElapsed);
                var vel = PathArcMotion.VelocityAt(w.Path, 0.0, w.Speed, segElapsed);
                var normalized = vel.Normalized();
                var heading = normalized.AbsSq > 0.0 ? normalized : startHeading;
                return new PoseSample(pos, heading, WalkAnimTag, true);

            case PauseSegment p:
                return new PoseSample(startPos, startHeading, p.AnimTag, true);

            case DwellSegment d:
                return new PoseSample(d.Pose, d.Heading, d.AnimTag, d.Visible);

            case InteractSegment i:
                return new PoseSample(i.Pose, i.Heading, i.AnimTag, true);

            default:
                throw new InvalidOperationException($"Unknown ActivitySegment type: {segment.GetType()}");
        }
    }
}
