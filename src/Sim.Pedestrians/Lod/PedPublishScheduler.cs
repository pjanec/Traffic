using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// P3-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- the pedestrian analog of
// Sim.Replication.PublishScheduler/DrErrorPublishPolicy for the FreeKinematic (high-power) stream. The
// car stack's scheduler predicts via DrExtrapolation.Arc (lane-window arc-length + speed/accel); the ped
// analog predicts via the SAME LINEAR extrapolation HeadlessIg.Reconstruct(FreeKinematic) uses
// (`lastPubPos + lastPubVel * (now - lastPubTime)`, HeadlessIg.cs) -- so "would the receiver's dead-
// reckoning be wrong" is computed with the exact formula the receiver itself will apply, never an
// approximation of it. A steadily-moving ped (near-constant velocity) diverges from that prediction by
// ~0 every step, so it is sent only every MaxInterval (a liveliness heartbeat); a maneuvering/avoiding
// ped diverges immediately and is sent promptly, keeping the IG within PosTol.
//
// Deliberately pure/deterministic: no System.Random, no wall-clock, no thread affinity. The scheduler
// owns the only state the policy needs but does not keep itself: each ped's last-PUBLISHED (pos, vel,
// time). Evaluate/Commit are split (rather than fused, like the vehicle PublishScheduler.ShouldPublish)
// specifically so PedBandwidthGovernor can ask "would this candidate want to be sent?" for EVERY
// candidate in a step, rank them, and commit only the ones that actually make it onto the wire --
// a candidate the governor defers must NOT have its tracked reference rebased, so its predicted error
// keeps growing next step and it is naturally re-prioritized (bounded, self-correcting).

public readonly struct PedPublishSignals
{
    public PedPublishSignals(int id, double time, double posError, double velDelta, double secondsSinceLastSent)
    {
        Id = id;
        Time = time;
        PosError = posError;
        VelDelta = velDelta;
        SecondsSinceLastSent = secondsSinceLastSent;
    }

    public int Id { get; }
    public double Time { get; }

    // |truePos - (lastPubPos + lastPubVel * secondsSinceLastSent)| -- the receiver's linear-extrapolation
    // error if nothing more is sent (HeadlessIg's exact formula). +inf on first sighting (never predictable).
    public double PosError { get; }

    // |trueVel - lastPubVel| -- a direction/speed change too, even one that has not YET produced enough
    // positional drift to trip PosTol (catches "just started a hard turn" a step earlier than pure
    // position error would).
    public double VelDelta { get; }

    public double SecondsSinceLastSent { get; }
}

public interface IPedPublishPolicy
{
    // True to include this ped's FreeKinematic sample in the current frame; false to keep dead-reckoning
    // it on the IG.
    bool ShouldPublish(in PedPublishSignals s);

    // A monotonically-increasing "how overdue is this" score used ONLY by a bandwidth governor to rank
    // candidates for thinning under a spike -- NOT consulted by the scheduler itself. 0 means "not urdent
    // at all"; >= 1 means "at or past the threshold that already makes ShouldPublish true"; +inf means
    // "must never be deferred" (first sighting). A policy that does not care how the governor ranks its
    // candidates may just return 0.0/1.0 for false/true.
    double Urgency(in PedPublishSignals s);
}

// The default ped policy (mirrors Sim.Replication.DrErrorPublishPolicy): publish only when the IG's
// linear extrapolation would be wrong by more than PosTol, or the velocity changed by more than VelTol
// (a maneuver that has not yet produced enough positional drift to trip PosTol), or a liveliness
// MaxInterval heartbeat elapsed. First sighting: SecondsSinceLastSent is +inf >= MaxInterval -> always
// publish.
public sealed class PedDrErrorPublishPolicy : IPedPublishPolicy
{
    public double PosTol { get; init; } = 0.1;      // m of linear-extrapolation position error
    public double VelTol { get; init; } = 0.5;      // m/s of velocity change
    public double MaxInterval { get; init; } = 3.0; // s liveliness heartbeat

    public bool ShouldPublish(in PedPublishSignals s) =>
        s.PosError > PosTol
        || s.VelDelta > VelTol
        || s.SecondsSinceLastSent >= MaxInterval;

    public double Urgency(in PedPublishSignals s)
    {
        if (double.IsPositiveInfinity(s.SecondsSinceLastSent))
        {
            return double.PositiveInfinity; // first sighting -- must never be deferred
        }

        var posScore = s.PosError / PosTol;
        var velScore = s.VelDelta / VelTol;
        var ageScore = s.SecondsSinceLastSent / MaxInterval;
        return Math.Max(posScore, Math.Max(velScore, ageScore));
    }
}

public sealed class PedPublishScheduler
{
    private readonly IPedPublishPolicy _policy;
    private readonly Dictionary<int, Ref> _lastPublished = new();
    private readonly HashSet<int> _seen = new();
    private List<int>? _pruneScratch;

    private readonly struct Ref
    {
        public Ref(Vec2 pos, Vec2 vel, double time) { Pos = pos; Vel = vel; Time = time; }
        public Vec2 Pos { get; }
        public Vec2 Vel { get; }
        public double Time { get; }
    }

    public PedPublishScheduler(IPedPublishPolicy policy)
    {
        _policy = policy ?? throw new ArgumentNullException(nameof(policy));
    }

    public IPedPublishPolicy Policy => _policy;

    // Peds currently remembered (published at least once, not yet pruned/forgotten). Test/telemetry hook.
    public int TrackedCount => _lastPublished.Count;

    // Computes this candidate's prediction-error signals against its last-PUBLISHED state, and marks it
    // "seen" this step (for EndStep's despawn pruning) -- but does NOT decide or mutate the tracked
    // reference. Pure/read-mostly (the only side effect is the seen-set bookkeeping every candidate needs
    // regardless of the outcome).
    public PedPublishSignals Signals(int id, Vec2 pos, Vec2 vel, double time)
    {
        _seen.Add(id);
        if (_lastPublished.TryGetValue(id, out var r))
        {
            var since = time - r.Time;
            var predPos = r.Pos + r.Vel * since;
            var posError = (pos - predPos).Abs;
            var velDelta = (vel - r.Vel).Abs;
            return new PedPublishSignals(id, time, posError, velDelta, since);
        }

        return new PedPublishSignals(id, time, double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
    }

    // Rebase the tracked last-published reference to the current (pos, vel, time) -- call this ONLY when
    // the sample is actually going out on the wire this step (a governor may evaluate a candidate via
    // Signals/Policy.ShouldPublish and then choose NOT to call Commit if it defers the candidate for
    // bandwidth reasons; the candidate is then re-evaluated fresh, with a larger error, next step).
    public void Commit(int id, Vec2 pos, Vec2 vel, double time) => _lastPublished[id] = new Ref(pos, vel, time);

    // Convenience for the ungoverned (no bandwidth governor) path: evaluate the policy and commit
    // atomically if it says yes. Mirrors Sim.Replication.PublishScheduler.ShouldPublish.
    public bool ShouldPublish(int id, Vec2 pos, Vec2 vel, double time)
    {
        var signals = Signals(id, pos, vel, time);
        if (!_policy.ShouldPublish(signals))
        {
            return false;
        }

        Commit(id, pos, vel, time);
        return true;
    }

    // Call once after a step's candidates have all been offered to Signals/ShouldPublish. Forgets any
    // tracked ped not seen this step (it demoted/despawned without an explicit Forget call), keeping
    // memory O(live high-power peds), then resets the per-step seen set.
    public void EndStep()
    {
        if (_lastPublished.Count > _seen.Count)
        {
            _pruneScratch ??= new List<int>();
            _pruneScratch.Clear();
            foreach (var kv in _lastPublished)
            {
                if (!_seen.Contains(kv.Key))
                {
                    _pruneScratch.Add(kv.Key);
                }
            }

            foreach (var stale in _pruneScratch)
            {
                _lastPublished.Remove(stale);
            }
        }

        _seen.Clear();
    }

    // Explicit per-ped forget (demotion/despawn): drops the tracked last-published state immediately, so
    // a later re-promotion of the same id is treated as a fresh first sighting rather than dead-reckoned
    // against a stale reference from a previous high-power spell.
    public void Forget(int id)
    {
        _lastPublished.Remove(id);
        _seen.Remove(id);
    }

    // Drop all bookkeeping (e.g. on a scenario reload). The policy is retained.
    public void Reset()
    {
        _lastPublished.Clear();
        _seen.Clear();
    }
}
