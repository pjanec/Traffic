using Sim.Core.Orca;
using Sim.Replication;

namespace Sim.Pedestrians.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7/§8) -- drains an IPedReplicationSource
// (after Pump()) and feeds a HeadlessIg with the decoded wire records, REUSING HeadlessIg unchanged as
// the reconstruction engine: this is the receiving half of the "server == IG survives serialization"
// proof (PedReplicationRoundTripTests). Tracks how far it has consumed each of the source's
// monotonically-growing lists so repeated Drain() calls (once per simulated step, in the round-trip
// test) only apply NEWLY arrived wire records.
public sealed class PedReplicationReceiver
{
    private readonly IPedReplicationSource _source;
    private readonly HeadlessIg _ig = new();

    private int _pathArcCursor;
    private int _timelineCursor;
    private int _lifecycleCursor;
    private uint? _lastAppliedCrowdStep;

    public PedReplicationReceiver(IPedReplicationSource source) => _source = source;

    // Exposes HeadlessIg directly (per the P3-1 task note) so a test can call Reconstruct /
    // ReconstructSample / ModelOf against it exactly like the in-process HeadlessIg tests do.
    public HeadlessIg Ig => _ig;

    // Applies every wire record that has arrived (via the source's Pump()) since the last Drain() call,
    // in the natural order: PathArc legs, ActivityTimeline legs, lifecycle (DR-switch) events, then the
    // latest crowd frame -- matching PedLodManager.Step's own publish order within a step (switches/
    // path-arcs are emitted before that step's FreeKinematicSample batch).
    public void Drain()
    {
        for (; _pathArcCursor < _source.PathArcs.Count; _pathArcCursor++)
        {
            var r = _source.PathArcs[_pathArcCursor];
            // Sim.Replication.PathArcRecord has no separate "Time" field (only StartTime); every
            // publisher in this codebase calls PedPublisher.PublishPathArc with Time == StartTime (see
            // PedLodManager.AddPed / the demotion branch of Step), so StartTime doubles for both here.
            _ig.Apply(new PathArcRecord((int)r.Handle.Index, r.StartTime, r.Path, r.StartTime, r.Speed));
        }

        for (; _timelineCursor < _source.ActivityTimelines.Count; _timelineCursor++)
        {
            var (handle, bytes) = _source.ActivityTimelines[_timelineCursor];
            var timeline = ActivityTimelineWire.Decode(bytes);
            _ig.Apply(new ActivityTimelineRecord((int)handle.Index, timeline.T0, timeline));
        }

        for (; _lifecycleCursor < _source.Lifecycles.Count; _lifecycleCursor++)
        {
            var lc = _source.Lifecycles[_lifecycleCursor];
            var to = ToModelOf(lc.Kind);
            if (to is null)
            {
                continue; // Spawn/Despawn: no DR-model switch to replay (see PedLifecycleKind remarks).
            }

            // `From` is a placeholder: HeadlessIg.Apply(DrSwitchEvent) only ever reads `.To` (see
            // HeadlessIg.cs), so which value we pass here is inert.
            _ig.Apply(new DrSwitchEvent((int)lc.Handle.Index, lc.Time, PedDrModel.PathArc, to.Value));
        }

        if (_source.LatestCrowdFrame.Count > 0 &&
            (_lastAppliedCrowdStep is null || _source.LatestCrowdStep != _lastAppliedCrowdStep.Value))
        {
            foreach (var rec in _source.LatestCrowdFrame)
            {
                _ig.Apply(new FreeKinematicSample((int)rec.Handle.Index, _source.LatestCrowdTime, new Vec2(rec.X, rec.Y), new Vec2(rec.Vx, rec.Vy)));
            }

            _lastAppliedCrowdStep = _source.LatestCrowdStep;
        }
    }

    private static PedDrModel? ToModelOf(PedLifecycleKind kind) => kind switch
    {
        PedLifecycleKind.PromoteToFreeKinematic => PedDrModel.FreeKinematic,
        PedLifecycleKind.DemoteToPathArc => PedDrModel.PathArc,
        PedLifecycleKind.DemoteToActivityTimeline => PedDrModel.ActivityTimeline,
        _ => null,
    };
}
