using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// The in-memory event stream a real DDS lifecycle/high-rate topic pair would carry (docs/PEDESTRIAN-
// DESIGN.md §7; docs/PEDESTRIAN-POC-PLAN.md POC-3). This POC proves the MECHANISM -- what gets sent,
// and how rarely, for low-power vs high-power -- not the transport, so there is no DDS wiring here at
// all: PedPublisher just appends to an ordered in-process list a HeadlessIg can drain.
public abstract record PedEvent(int Id, double Time);

// Emitted once per PathArc "leg": on spawn, and again on every demotion (with a fresh re-routed path).
// The path is sent ONCE, never repeated per step -- exactly what makes the low-power population near-
// free on the wire (docs/PEDESTRIAN-DESIGN.md §7 bandwidth math).
public sealed record PathArcRecord(int Id, double Time, IReadOnlyList<Vec2> Path, double StartTime, double Speed)
    : PedEvent(Id, Time);

// A DR-model switch on the lifecycle topic -- the promotion/demotion broadcast. The IG applies this at
// its Time: before it the ped is reconstructed under `From`, from it onward under `To`.
public sealed record DrSwitchEvent(int Id, double Time, PedDrModel From, PedDrModel To) : PedEvent(Id, Time);

// LIVE-POC-1 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §1, §12): the ActivityTimeline analogue of
// PathArcRecord -- broadcast ONCE (on spawn, or again on a re-plan after a mid-activity promotion/
// demotion, §10), never repeated per step. The IG stores the whole timeline and reconstructs pose +
// animation + visibility by calling the exact same ActivityTimeline.PoseAt the server calls.
public sealed record ActivityTimelineRecord(int Id, double Time, ActivityTimeline Timeline) : PedEvent(Id, Time);

// One high-power position/velocity sample, streamed every step a ped is FreeKinematic. POC-3 success
// condition 4 only requires this to be silent while low-power; a publish-on-predicted-error gate (the
// car stack's DrErrorPublishPolicy) is explicitly optional polish here, so every high-power step emits
// one -- see PedLodManager remarks for why that is still a faithful "silent when low-power" story.
public sealed record FreeKinematicSample(int Id, double Time, Vec2 Pos, Vec2 Vel) : PedEvent(Id, Time);

// Low-rate liveness signal for a PathArc ped. Carries no pose -- the IG already has the path -- so it
// costs almost nothing on the wire and lets a late/lossy channel confirm the ped still exists.
public sealed record HeartbeatEvent(int Id, double Time) : PedEvent(Id, Time);

// In-memory publisher: appends every emitted event to `Events` in wire order and tracks per-id send
// counters -- the numbers POC-3 success conditions 1 and 4 are measured against
// (FreeKinematicSamplesSent, PathArcRecordsSent, HeartbeatsSent).
public sealed class PedPublisher
{
    private readonly List<PedEvent> _events = new();
    private readonly Dictionary<int, double> _lastHeartbeatAt = new();
    private readonly Dictionary<int, int> _freeKinematicSamplesSent = new();
    private readonly Dictionary<int, int> _pathArcRecordsSent = new();
    private readonly Dictionary<int, int> _activityTimelineRecordsSent = new();
    private readonly Dictionary<int, int> _heartbeatsSent = new();

    public PedPublisher(double heartbeatInterval = 3.0)
    {
        HeartbeatInterval = heartbeatInterval;
    }

    public double HeartbeatInterval { get; }

    public IReadOnlyList<PedEvent> Events => _events;

    public IReadOnlyDictionary<int, int> FreeKinematicSamplesSent => _freeKinematicSamplesSent;
    public IReadOnlyDictionary<int, int> PathArcRecordsSent => _pathArcRecordsSent;
    public IReadOnlyDictionary<int, int> ActivityTimelineRecordsSent => _activityTimelineRecordsSent;
    public IReadOnlyDictionary<int, int> HeartbeatsSent => _heartbeatsSent;

    public void PublishPathArc(int id, IReadOnlyList<Vec2> path, double startTime, double speed, double time)
    {
        _events.Add(new PathArcRecord(id, time, path, startTime, speed));
        Increment(_pathArcRecordsSent, id);
    }

    public void PublishSwitch(int id, PedDrModel from, PedDrModel to, double time)
    {
        _events.Add(new DrSwitchEvent(id, time, from, to));
    }

    // Broadcast-once (LIVE-POC-1): sends the whole ActivityTimeline exactly once, mirroring
    // PublishPathArc's "path sent once" discipline.
    public void PublishActivityTimeline(int id, ActivityTimeline timeline, double time)
    {
        _events.Add(new ActivityTimelineRecord(id, time, timeline));
        Increment(_activityTimelineRecordsSent, id);
    }

    public void PublishSample(int id, double time, Vec2 pos, Vec2 vel)
    {
        _events.Add(new FreeKinematicSample(id, time, pos, vel));
        Increment(_freeKinematicSamplesSent, id);
    }

    // Emits at most once per HeartbeatInterval seconds per id -- a no-op otherwise, so calling this
    // every step for every low-power ped is the correct, cheap usage pattern (PedLodManager does).
    public void MaybePublishHeartbeat(int id, double time)
    {
        if (_lastHeartbeatAt.TryGetValue(id, out var last) && time - last < HeartbeatInterval)
        {
            return;
        }

        _lastHeartbeatAt[id] = time;
        _events.Add(new HeartbeatEvent(id, time));
        Increment(_heartbeatsSent, id);
    }

    private static void Increment(Dictionary<int, int> counts, int id)
    {
        counts.TryGetValue(id, out var n);
        counts[id] = n + 1;
    }
}
