using Sim.Core;
using Sim.Replication;

namespace Sim.Pedestrians.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- maps a PedPublisher's in-process
// PedEvent stream onto the transport-neutral IPedReplicationSink (Sim.Replication). This is the ONE
// place allowed to reference both Sim.Pedestrians and Sim.Replication (see PedReplication.cs's header
// comment for why Sim.Replication itself must not reference back).
public sealed class PedReplicationPublisher
{
    private readonly IPedReplicationSink _sink;

    // FreeKinematicSample events are published once per currently-high-power ped per PedLodManager.Step
    // call, all stamped with that step's own `newNow` (see PedLodManager.Step's final loop) -- grouping
    // consecutive same-Time samples into one crowd frame therefore reconstructs "one crowd frame per
    // step" without this class needing the caller to mark step boundaries explicitly.
    private uint _nextStep;

    public PedReplicationPublisher(IPedReplicationSink sink) => _sink = sink;

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
        var recs = new PedFreeKinematicRecord[pending.Count];
        for (var i = 0; i < pending.Count; i++)
        {
            var s = pending[i];
            // Radius is not modeled here: FreeKinematicSample (PedPublisher.cs) carries only Id/Time/
            // Pos/Vel -- no footprint radius -- so this bridge cannot forward a real value. 0.0 is a
            // harmless placeholder: none of the P3-1 success conditions (position/velocity
            // reconstruction, DR-switch timing) depend on the wire radius field.
            recs[i] = new PedFreeKinematicRecord(new VehicleHandle((uint)s.Id, 0), s.Pos.X, s.Pos.Y, s.Vel.X, s.Vel.Y, radius: 0.0);
        }

        _sink.PublishCrowdFrame(_nextStep++, (float)time, recs);
        pending.Clear();
    }

    private void PublishOne(PedEvent evt)
    {
        switch (evt)
        {
            case PathArcRecord r:
                var wire = new Sim.Replication.PathArcRecord(new VehicleHandle((uint)r.Id, 0), r.Speed, r.StartTime, r.Path);
                _sink.PublishPathArc(wire);
                break;

            case ActivityTimelineRecord a:
                var bytes = ActivityTimelineWire.Encode(a.Timeline);
                _sink.PublishActivityTimeline(new VehicleHandle((uint)a.Id, 0), bytes);
                break;

            case DrSwitchEvent sw:
                _sink.PublishPedLifecycle(new PedLifecycleRecord(new VehicleHandle((uint)sw.Id, 0), MapSwitch(sw.To), sw.Time));
                break;

            case HeartbeatEvent:
                // Liveness only -- no pose information, and P3-1's task note explicitly allows skipping
                // wire bytes for it (a production heartbeat would still ride the lifecycle topic, but
                // nothing in HeadlessIg's reconstruction depends on it).
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
