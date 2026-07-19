using Sim.Core;
using Sim.Core.Orca;
using Sim.Replication;

namespace Sim.Pedestrians.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- maps a PedPublisher's in-process
// PedEvent stream onto the transport-neutral IPedReplicationSink (Sim.Replication). This is the ONE
// place allowed to reference both Sim.Pedestrians and Sim.Replication (see PedReplication.cs's header
// comment for why Sim.Replication itself must not reference back).
//
// P3-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- ADDITIVE DR-error gating + a global
// bandwidth governor for the FreeKinematic (high-power) stream, both strictly OPT-IN via an optional
// constructor overload: the original single-argument constructor (`new
// PedReplicationPublisher(sink)`) leaves `_scheduler`/`_governor`/`_meter` null and behaves BYTE-FOR-BYTE
// like the P3-1 version -- every FreeKinematicSample is forwarded unconditionally. This is deliberate
// (see the task note): the existing PedReplicationRoundTripTests asserts a tight (<= 0.02 m) FreeKinematic
// reconstruction bound measured against a run that publishes every step; rather than retuning that bound
// to also hold under gating (which would couple an unrelated test's tolerance to this task's default
// tolerances), the default path is left completely ungated and P3-2's own tests construct the gated
// overload explicitly. Low-power peds (PathArc/ActivityTimeline) are UNCHANGED either way -- they never
// produce a FreeKinematicSample in the first place (PedLodManager.Step only calls PublishSample for a
// FreeKinematic-model ped), so gating on/off has zero effect on the "zero per-step crowd-frame bytes for
// low-power" invariant.
public sealed class PedReplicationPublisher
{
    private readonly IPedReplicationSink _sink;
    private readonly PedPublishScheduler? _scheduler;
    private readonly PedBandwidthGovernor? _governor;
    private readonly PedBandwidthMeter? _meter;
    private readonly double _stepDt;

    // FreeKinematicSample events are published once per currently-high-power ped per PedLodManager.Step
    // call, all stamped with that step's own `newNow` (see PedLodManager.Step's final loop) -- grouping
    // consecutive same-Time samples into one crowd frame therefore reconstructs "one crowd frame per
    // step" without this class needing the caller to mark step boundaries explicitly.
    private uint _nextStep;

    private readonly List<(int Id, Vec2 Pos, Vec2 Vel)> _candidateScratch = new();

    // Back-compat/default: fully ungated, exactly the P3-1 behaviour (every FreeKinematicSample is
    // forwarded). See the class remarks for why this stays the default.
    public PedReplicationPublisher(IPedReplicationSink sink) : this(sink, scheduler: null)
    {
    }

    // Gated overload (P3-2): when `scheduler` is non-null, each step's high-power samples are run through
    // it (and, if supplied, `governor`) before hitting the wire; only survivors are batched into that
    // step's crowd frame. `meter`, if supplied, is fed the ACTUAL bytes emitted for every topic (crowd
    // frame, path-arc, activity-timeline, lifecycle, heartbeat) -- independent of whether gating is
    // enabled, so a caller can measure bandwidth even on the ungated path. `stepDt` is the fixed seconds
    // each Publish() step-batch advances (needed by `governor` to convert a byte budget into a rate); it
    // is a caller-supplied constant, never inferred from timestamp deltas, so behaviour cannot depend on
    // guessing a step length from the first observed frame.
    public PedReplicationPublisher(
        IPedReplicationSink sink,
        PedPublishScheduler? scheduler,
        PedBandwidthGovernor? governor = null,
        PedBandwidthMeter? meter = null,
        double stepDt = 0.1)
    {
        if (governor is not null && scheduler is null)
        {
            throw new ArgumentException("a bandwidth governor requires a scheduler.", nameof(governor));
        }

        _sink = sink;
        _scheduler = scheduler;
        _governor = governor;
        _meter = meter;
        _stepDt = stepDt;
    }

    // Publishes a batch of PedEvents (e.g. everything PedPublisher appended during one Step call, or the
    // whole run at once) onto the wire, in order.
    public void Publish(IEnumerable<PedEvent> events)
    {
        var pending = new List<FreeKinematicSample>();

        foreach (var evt in events)
        {
            if (evt is FreeKinematicSample sample)
            {
                if (pending.Count > 0 && Math.Abs(sample.Time - pending[0].Time) > 1e-9)
                {
                    FlushCrowdFrame(pending);
                }

                pending.Add(sample);
                continue;
            }

            FlushCrowdFrame(pending);
            PublishOne(evt);
        }

        FlushCrowdFrame(pending);
    }

    private void FlushCrowdFrame(List<FreeKinematicSample> pending)
    {
        if (pending.Count == 0)
        {
            return;
        }

        var time = pending[0].Time;
        IReadOnlyList<FreeKinematicSample> toSend = pending;

        if (_scheduler is not null)
        {
            _candidateScratch.Clear();
            foreach (var s in pending)
            {
                _candidateScratch.Add((s.Id, s.Pos, s.Vel));
            }

            List<(int Id, Vec2 Pos, Vec2 Vel)> selected;
            if (_governor is not null)
            {
                selected = new List<(int Id, Vec2 Pos, Vec2 Vel)>(_governor.SelectForPublish(_candidateScratch, time, _stepDt));
            }
            else
            {
                selected = new List<(int Id, Vec2 Pos, Vec2 Vel)>(pending.Count);
                foreach (var c in _candidateScratch)
                {
                    if (_scheduler.ShouldPublish(c.Id, c.Pos, c.Vel, time))
                    {
                        selected.Add(c);
                    }
                }
            }

            _scheduler.EndStep();

            if (selected.Count == 0)
            {
                pending.Clear();
                return;
            }

            var selectedIds = new HashSet<int>(selected.Count);
            foreach (var s in selected)
            {
                selectedIds.Add(s.Id);
            }

            var filtered = new List<FreeKinematicSample>(selected.Count);
            foreach (var s in pending)
            {
                if (selectedIds.Contains(s.Id))
                {
                    filtered.Add(s);
                }
            }

            toSend = filtered;
        }

        var recs = new PedFreeKinematicRecord[toSend.Count];
        for (var i = 0; i < toSend.Count; i++)
        {
            var s = toSend[i];
            // Radius is not modeled here: FreeKinematicSample (PedPublisher.cs) carries only Id/Time/
            // Pos/Vel -- no footprint radius -- so this bridge cannot forward a real value. 0.0 is a
            // harmless placeholder: none of the P3-1 success conditions (position/velocity
            // reconstruction, DR-switch timing) depend on the wire radius field.
            recs[i] = new PedFreeKinematicRecord(new VehicleHandle((uint)s.Id, 0), s.Pos.X, s.Pos.Y, s.Vel.X, s.Vel.Y, radius: 0.0);
        }

        _sink.PublishCrowdFrame(_nextStep++, (float)time, recs);
        _meter?.RecordCrowdFrame(time, recs.Length);
        pending.Clear();
    }

    private void PublishOne(PedEvent evt)
    {
        switch (evt)
        {
            case PathArcRecord r:
                var wire = new Sim.Replication.PathArcRecord(new VehicleHandle((uint)r.Id, 0), r.Speed, r.StartTime, r.Path);
                _sink.PublishPathArc(wire);
                _meter?.RecordPathArc(evt.Time, r.Path.Count);
                break;

            case ActivityTimelineRecord a:
                var bytes = ActivityTimelineWire.Encode(a.Timeline);
                _sink.PublishActivityTimeline(new VehicleHandle((uint)a.Id, 0), bytes);
                _meter?.RecordActivityTimeline(evt.Time, bytes.Length);
                break;

            case DrSwitchEvent sw:
                _sink.PublishPedLifecycle(new PedLifecycleRecord(new VehicleHandle((uint)sw.Id, 0), MapSwitch(sw.To), sw.Time));
                _meter?.RecordLifecycle(sw.Time);
                // A demotion away from FreeKinematic (or any DR-switch touching this id) drops the
                // scheduler's tracked last-published reference: a later re-promotion is then a fresh
                // first sighting rather than dead-reckoned against a stale reference from a previous
                // high-power spell. Harmless no-op for an id the scheduler never tracked (e.g. a
                // promotion switch, or when gating is disabled).
                _scheduler?.Forget(sw.Id);
                break;

            case HeartbeatEvent hb:
                // Liveness only -- no pose information, and P3-1's task note explicitly allows skipping
                // wire bytes for it (a production heartbeat would still ride the lifecycle topic, but
                // nothing in HeadlessIg's reconstruction depends on it). Still metered (near-zero cost)
                // so the low-power invariant's per-topic breakdown accounts for it explicitly.
                _meter?.RecordHeartbeat(hb.Time);
                break;

            case FreeKinematicSample:
                throw new InvalidOperationException("FreeKinematicSample must be routed through the crowd-frame batching path.");

            default:
                throw new InvalidOperationException($"Unknown PedEvent type: {evt.GetType()}");
        }
    }

    private static PedLifecycleKind MapSwitch(PedDrModel to) => to switch
    {
        PedDrModel.FreeKinematic => PedLifecycleKind.PromoteToFreeKinematic,
        PedDrModel.PathArc => PedLifecycleKind.DemoteToPathArc,
        PedDrModel.ActivityTimeline => PedLifecycleKind.DemoteToActivityTimeline,
        _ => throw new ArgumentOutOfRangeException(nameof(to), to, "unsupported DR-switch target for the wire lifecycle mapping"),
    };
}
