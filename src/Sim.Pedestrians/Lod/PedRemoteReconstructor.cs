using Sim.Core.Orca;
using Sim.Replication;

namespace Sim.Pedestrians.Lod;

// P3-3 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- the render-side consumer that
// closes the server -> wire -> IG -> render loop for pedestrians. Wraps a PedReplicationReceiver
// (P3-1: decodes the wire into a HeadlessIg, the same reconstruction engine the server itself
// calls) with:
//
//   1. A playout-delay RENDER CLOCK: renderTime = latestServerTime - PlayoutDelay. Mirrors the IDEA
//      behind Sim.Viewer.Motion.DrClock's wall<->sim rate fit (a small delay keeps the render clock
//      behind the newest data so a sparse/late high-power sample is usually already on hand rather
//      than racing ahead into pure extrapolation) -- reimplemented trivially here because this
//      reconstructor is always pumped with the SAME sim-time axis the publisher stamped its events
//      with (no cross-process wall-clock/sim-rate fit to do), so the "clock" is just the one
//      subtraction. Never negative.
//
//   2. Per-ped CAPPED-CORRECTION position smoothing: the rendered position chases the reconstructed
//      target but its correction speed is capped, so constant-velocity motion (the common case)
//      passes through with ~zero added lag, while a reconciliation snap -- a DR-error-gated
//      correction landing after a quiet spell, or a P3-2 bandwidth-governor deferred sample finally
//      catching up -- is absorbed over a frame or few instead of teleporting. A genuinely large gap
//      (first sighting, a real teleport/respawn) snaps directly rather than crawling across the map.
//      Mirrors the IDEA behind Sim.Viewer.Motion.DrPoseSmoother's capped, forward-biased position
//      correction, reimplemented as a plain isotropic (holonomic) cap: a pedestrian has no
//      lane-forward heading to decompose the correction against, so there is no along/lateral split,
//      just a single magnitude cap on the correction vector.
//
// Deliberately does NOT reuse Sim.Viewer.Motion.DrClock/DrPoseSmoother directly (see the P3-3 task's
// design-decision note): both are lane/VehicleHandle-specific (arc-length lane windows, lane-forward
// heading tilt) and Sim.Viewer.Motion pulls in the vehicle-only Sim.Ingest. Only the IDEAS are reused
// here, reimplemented simply for a holonomic 2D pedestrian pose.
public sealed class PedRemoteReconstructor
{
    // Small enough that a ped whose next high-power sample is only slightly late still renders from
    // real (already-received) data rather than extrapolating past it; large enough to smooth over a
    // typical DR-error-gated publish gap (docs/PEDESTRIAN-DESIGN.md §7).
    public const double DefaultPlayoutDelaySeconds = 0.15;

    // Generous ceiling above any ped's modeled walking/high-power speed in this codebase's demos and
    // tests (~1.3-1.5 m/s): constant-speed motion never hits this cap, so it passes through with
    // zero added lag.
    private const double MaxTrackedSpeedMetersPerSecond = 3.0;

    // Extra correction budget beyond the tracked-speed allowance: bounds how fast a reconciliation
    // snap (a DR-error correction, a governor catch-up) is absorbed -- over a frame or few, never an
    // instantaneous teleport, but also never a multi-second crawl.
    private const double CatchUpMetersPerSecond = 2.0;

    // A gap larger than this is treated as a genuine discontinuity (a real teleport/respawn) rather
    // than a reconciliation snap -- render position jumps directly to the target instead of chasing
    // it, per the class remarks ("respawn", not "crawl across the map").
    private const double SnapDistanceMeters = 3.0;

    private sealed class SmoothState
    {
        public Vec2 RenderPos;
        public double LastRenderTime;
    }

    private readonly IPedReplicationSource _source;
    private readonly PedReplicationReceiver _receiver;
    private readonly double _playoutDelay;

    private readonly HashSet<int> _knownIds = new();
    private readonly Dictionary<int, SmoothState> _smoothed = new();

    private int _pathArcIdCursor;
    private int _timelineIdCursor;
    private int _lifecycleIdCursor;

    public PedRemoteReconstructor(IPedReplicationSource source, double playoutDelaySeconds = DefaultPlayoutDelaySeconds)
    {
        _source = source;
        _receiver = new PedReplicationReceiver(source);
        _playoutDelay = playoutDelaySeconds;
    }

    // The underlying reconstruction engine (P3-1), exposed for a caller/test that wants the raw,
    // UNSMOOTHED reconstruction for comparison (e.g. to quote "smoothed vs plain" render error/step
    // around a DR-switch).
    public HeadlessIg Ig => _receiver.Ig;

    // renderTime = latestServerTime - PlayoutDelay (never negative). Updated by Pump.
    public double RenderTime { get; private set; }

    // Every ped id ever observed on the wire (spawned via a PathArc/ActivityTimeline leg, or named by
    // a lifecycle record) and not since despawned.
    public IReadOnlyCollection<int> KnownIds => _knownIds;

    // Drains the source (via the wrapped receiver) and advances the render clock. Call once per
    // rendered frame/step with the newest sim time known to have been published (e.g. the sim's own
    // `now` after that step -- matching every FreeKinematicSample's Time that step).
    public void Pump(double latestServerTime)
    {
        _source.Pump();
        _receiver.Drain();
        TrackLifecycle();
        RenderTime = Math.Max(0.0, latestServerTime - _playoutDelay);
    }

    // Reconstructs `id`'s render-time pose (position, visibility, animation tag) from the wire alone,
    // applying capped-correction smoothing to the position. Returns false if `id` has never been
    // observed on the wire (or has since despawned) -- there is nothing to render.
    public bool TryGetRenderPose(int id, out Vec2 pos, out bool visible, out string animTag)
    {
        if (!_knownIds.Contains(id) || !_receiver.Ig.Knows(id))
        {
            pos = Vec2.Zero;
            visible = false;
            animTag = ActivityTimeline.IdleAnimTag;
            return false;
        }

        var sample = _receiver.Ig.ReconstructSample(id, RenderTime);
        pos = Smooth(id, sample.Pos);
        visible = sample.Visible;
        animTag = sample.AnimTag;
        return true;
    }

    // The capped-correction chase (class remarks). The first observation of `id` returns the target
    // pose unchanged (and remembers it as the smoothing anchor) -- there is no previous render frame
    // to correct from, mirroring DrPoseSmoother's own "first sighting" behaviour.
    private Vec2 Smooth(int id, Vec2 target)
    {
        if (!_smoothed.TryGetValue(id, out var state))
        {
            _smoothed[id] = new SmoothState { RenderPos = target, LastRenderTime = RenderTime };
            return target;
        }

        var dt = RenderTime - state.LastRenderTime;
        state.LastRenderTime = RenderTime;

        if (dt <= 0.0)
        {
            // Render clock held or (defensively) stepped backward this call -- nothing to advance;
            // hold the last rendered position rather than dividing by a non-positive dt below.
            return state.RenderPos;
        }

        var delta = target - state.RenderPos;
        var dist = delta.Abs;

        if (dist > SnapDistanceMeters)
        {
            state.RenderPos = target; // genuine large discontinuity -- respawn, don't crawl toward it
            return state.RenderPos;
        }

        var cap = (MaxTrackedSpeedMetersPerSecond + CatchUpMetersPerSecond) * dt;
        state.RenderPos = dist <= cap ? target : state.RenderPos + (delta * (cap / dist));
        return state.RenderPos;
    }

    // Watches the source's PathArc/ActivityTimeline/Lifecycle lists (in arrival order, the same
    // cursor-tracking discipline PedReplicationReceiver.Drain uses) purely to maintain `KnownIds` and
    // to drop a despawned ped's smoothing state. IPedReplicationSource exposes no direct "known ids"
    // query and HeadlessIg deliberately exposes only per-id lookups (Knows/ModelOf/ReconstructSample),
    // so this reconstructs the id set from the same wire evidence a real IG would see a ped
    // appear/disappear from.
    private void TrackLifecycle()
    {
        for (; _pathArcIdCursor < _source.PathArcs.Count; _pathArcIdCursor++)
        {
            _knownIds.Add((int)_source.PathArcs[_pathArcIdCursor].Handle.Index);
        }

        for (; _timelineIdCursor < _source.ActivityTimelines.Count; _timelineIdCursor++)
        {
            _knownIds.Add((int)_source.ActivityTimelines[_timelineIdCursor].Handle.Index);
        }

        for (; _lifecycleIdCursor < _source.Lifecycles.Count; _lifecycleIdCursor++)
        {
            var lc = _source.Lifecycles[_lifecycleIdCursor];
            var id = (int)lc.Handle.Index;
            if (lc.Kind == PedLifecycleKind.Despawn)
            {
                _knownIds.Remove(id);
                _smoothed.Remove(id);
            }
            else
            {
                _knownIds.Add(id);
            }
        }
    }
}
