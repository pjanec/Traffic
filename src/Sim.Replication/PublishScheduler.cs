using Sim.Core;

namespace Sim.Replication;

// SUMOSHARP-DEADRECKONING.md §7 — the transport-agnostic scheduling loop that turns a per-vehicle
// IPublishPolicy (a stateless predicate) into a per-step "which movers do I send this step?" decision.
// It owns the only state the policy needs but does not keep: each mover's last-published sim time. A
// publisher (DDS / TCP / the LiveHost demo) drives it once per sim step -- ask ShouldPublish per candidate,
// then EndStep to forget despawned movers.
//
// Keyed by VehicleHandle (NO strings -- matches the wire packet). Stateful and single-threaded: this is the
// publish side, which already runs on one thread reading the async snapshot. The policy stays swappable
// (camera-distance weighting, bandwidth governor, ...); the scheduler is the reusable bookkeeping around it,
// previously duplicated in the demo host.
public sealed class PublishScheduler
{
    private readonly IPublishPolicy _policy;
    private readonly Dictionary<VehicleHandle, Ref> _lastSent = new();
    private readonly HashSet<VehicleHandle> _seen = new();
    private List<VehicleHandle>? _pruneScratch;

    private readonly struct Ref
    {
        public Ref(double pos, double posLat, double speed, double accel, double latSpeed, int lane, double time)
        { Pos = pos; PosLat = posLat; Speed = speed; Accel = accel; LatSpeed = latSpeed; Lane = lane; Time = time; }
        public double Pos { get; } public double PosLat { get; } public double Speed { get; }
        public double Accel { get; } public double LatSpeed { get; } public int Lane { get; } public double Time { get; }
    }

    public PublishScheduler(IPublishPolicy policy)
    {
        if (policy is null)
        {
            throw new ArgumentNullException(nameof(policy));
        }

        _policy = policy;
    }

    // Movers currently remembered (i.e. published at least once and not yet pruned). Test/telemetry hook.
    public int TrackedCount => _lastSent.Count;

    // Decide for ONE candidate mover. Computes the receiver's DR prediction from this mover's last-PUBLISHED
    // state (via the shared DrExtrapolation.Arc, so it matches the viewer exactly) and hands the policy both
    // the time-since-last (for DefaultPublishPolicy) and the prediction error (for DrErrorPublishPolicy). On
    // publish, re-bases the stored reference to the current state.
    public bool ShouldPublish(
        VehicleHandle handle, DrModel model, double pos, double posLat, double speed, double accel,
        double latSpeed, int laneHandle, double time, bool laneChangingOrManoeuvring)
    {
        _seen.Add(handle);
        double since;
        double posError = 0.0, latError = 0.0;
        var laneChanged = false;
        if (_lastSent.TryGetValue(handle, out var r))
        {
            since = time - r.Time;
            laneChanged = laneHandle != r.Lane;
            if (!laneChanged) // arc-pos is only comparable within the same lane; a lane change publishes anyway
            {
                var predPos = DrExtrapolation.Arc(r.Pos, r.Speed, r.Accel, since);
                posError = Math.Abs(pos - predPos);
                var predLat = r.PosLat + r.LatSpeed * since;
                latError = Math.Abs(posLat - predLat);
            }
        }
        else
        {
            since = double.PositiveInfinity; // first sighting -> always publish
        }

        var signals = new PublishSignals(
            handle, model, speed, accel, since, laneChangingOrManoeuvring, posError, latError, laneChanged);
        if (!_policy.ShouldPublish(signals))
        {
            return false;
        }

        _lastSent[handle] = new Ref(pos, posLat, speed, accel, latSpeed, laneHandle, time);
        return true;
    }

    // Call once after a step's candidates have all been offered to ShouldPublish. Forgets any tracked mover
    // not seen this step (it despawned), keeping memory O(live movers), then resets the per-step seen set.
    public void EndStep()
    {
        if (_lastSent.Count > _seen.Count)
        {
            _pruneScratch ??= new List<VehicleHandle>();
            _pruneScratch.Clear();
            foreach (var kv in _lastSent)
            {
                if (!_seen.Contains(kv.Key))
                {
                    _pruneScratch.Add(kv.Key);
                }
            }

            foreach (var stale in _pruneScratch)
            {
                _lastSent.Remove(stale);
            }
        }

        _seen.Clear();
    }

    // Drop all bookkeeping (e.g. on a scenario reload). The policy is retained.
    public void Reset()
    {
        _lastSent.Clear();
        _seen.Clear();
    }
}
