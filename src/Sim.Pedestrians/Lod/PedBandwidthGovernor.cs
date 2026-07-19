using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// P3-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- a GLOBAL bandwidth governor wrapping
// PedPublishScheduler. Per §7 there is exactly ONE multicast stream and NO per-channel/AoI culling, so the
// only lever under a spike is a global cap on how many FreeKinematic samples go out THIS step: when the
// DR-error scheduler's candidates for a step would push the single stream's projected rate over
// MaxMbitPerSecond, this defers the least-urgent candidates (smallest predicted error / longest until
// their next mandatory heartbeat) rather than sending all of them.
//
// The trade this makes explicit (per the task note): a deferred ped is simply dead-reckoned one MORE
// step on every IG (its tracked last-published reference is left untouched, so PedPublishScheduler
// naturally re-evaluates it next step with a LARGER error/urgency) -- bounded and self-correcting, at the
// cost of a little extra transient extrapolation error on that one ped for one step, in exchange for the
// single stream never blowing through its ceiling even during a mass-promotion spike. First-sighting
// candidates (Urgency == +inf) are never deferred: a never-seen ped must always get its first sample.
public sealed class PedBandwidthGovernor
{
    private readonly PedPublishScheduler _scheduler;
    private readonly PedBandwidthMeter _meter;

    private readonly List<(int Id, Vec2 Pos, Vec2 Vel, double Urgency)> _wantedScratch = new();
    private readonly List<(int Id, Vec2 Pos, Vec2 Vel)> _sentScratch = new();

    public PedBandwidthGovernor(PedPublishScheduler scheduler, PedBandwidthMeter meter, double maxMbitPerSecond = 500.0)
    {
        _scheduler = scheduler ?? throw new ArgumentNullException(nameof(scheduler));
        _meter = meter ?? throw new ArgumentNullException(nameof(meter));
        MaxMbitPerSecond = maxMbitPerSecond;
    }

    public double MaxMbitPerSecond { get; init; }

    // How many of this step's candidates the DR-error policy wanted sent, before any governor thinning
    // (i.e. what an ungated scheduler would have sent). Diagnostic/test hook.
    public int LastWantedCount { get; private set; }

    // How many of LastWantedCount were actually sent this step (<= LastWantedCount).
    public int LastSentCount { get; private set; }

    // LastWantedCount - LastSentCount: how many were deferred purely for bandwidth reasons this step.
    public int LastDeferredCount { get; private set; }

    // True iff this step's candidate set would have exceeded MaxMbitPerSecond and the governor had to
    // thin it (i.e. LastDeferredCount > 0).
    public bool LastStepGoverned { get; private set; }

    // `candidates`: this step's high-power (id, pos, vel) population, in the caller's iteration order
    // (PedLodManager iterates ascending id -- deterministic). `time`: this step's stamp (matches every
    // FreeKinematicSample's Time this step). `dt`: the seconds this step advanced (the denominator for
    // converting this step's byte budget to a rate) -- NOT inferred, so behaviour never depends on
    // guessing a step length from timestamp deltas.
    //
    // Returns the subset to actually publish, in the ORIGINAL candidate order (so a caller building a
    // wire frame from it gets a stable, deterministic record order). Every returned candidate has been
    // Commit()ed into the scheduler; every deferred one has not (see class remarks). The returned list is
    // a reused internal buffer -- consume it (e.g. copy into a wire frame) before calling
    // SelectForPublish again.
    public IReadOnlyList<(int Id, Vec2 Pos, Vec2 Vel)> SelectForPublish(
        IReadOnlyList<(int Id, Vec2 Pos, Vec2 Vel)> candidates, double time, double dt)
    {
        _wantedScratch.Clear();
        _sentScratch.Clear();

        foreach (var c in candidates)
        {
            var signals = _scheduler.Signals(c.Id, c.Pos, c.Vel, time);
            if (!_scheduler.Policy.ShouldPublish(signals))
            {
                continue;
            }

            var urgency = _scheduler.Policy.Urgency(signals);
            _wantedScratch.Add((c.Id, c.Pos, c.Vel, urgency));
        }

        LastWantedCount = _wantedScratch.Count;

        // How many crowd-frame records still fit this step's byte budget, after whatever this step's
        // other topics (lifecycle switches, path-arc re-routes) have already put on the meter at this
        // exact `time`. Subtracts a tiny SafetyMarginBytes slack before flooring so the result always
        // lands STRICTLY under MaxMbitPerSecond, never exactly at it (both a hedge against the double
        // arithmetic above being off by a representable-fraction epsilon, e.g. dt=0.1, and a deliberate
        // "spike insurance leaves insurance" choice: the whole point of the ceiling is headroom, so
        // landing exactly on the wire is not good enough).
        const double SafetyMarginBytes = 1.0;
        var budgetBytes = MaxMbitPerSecond * 1_000_000.0 / 8.0 * dt;
        var spentBytes = _meter.BytesAtTime(time);
        var remainingBytes = budgetBytes - spentBytes - Sim.Replication.FrameCodec.HeaderSize - SafetyMarginBytes;
        var maxRecords = remainingBytes <= 0
            ? 0
            : (int)Math.Min(int.MaxValue, Math.Floor(remainingBytes / Sim.Replication.FrameCodec.PedFreeKinematicRecordSize));

        List<(int Id, Vec2 Pos, Vec2 Vel, double Urgency)> selected;
        if (_wantedScratch.Count <= maxRecords)
        {
            selected = _wantedScratch;
            LastStepGoverned = false;
        }
        else
        {
            // Rank by urgency descending (most-overdue first), stable tie-break by id so the choice is
            // deterministic regardless of candidate arrival order. Keep the top `maxRecords`; every other
            // wanted candidate is deferred (its scheduler state is left untouched -- see class remarks).
            var ranked = new List<(int Id, Vec2 Pos, Vec2 Vel, double Urgency)>(_wantedScratch);
            ranked.Sort(static (a, b) =>
            {
                var byUrgency = b.Urgency.CompareTo(a.Urgency); // descending
                return byUrgency != 0 ? byUrgency : a.Id.CompareTo(b.Id);
            });

            selected = ranked.GetRange(0, maxRecords);
            LastStepGoverned = true;
        }

        var selectedIds = new HashSet<int>(selected.Count);
        foreach (var s in selected)
        {
            selectedIds.Add(s.Id);
        }

        // Preserve the ORIGINAL candidate order in the output, committing each sent candidate's tracked
        // state as we go.
        foreach (var c in candidates)
        {
            if (!selectedIds.Contains(c.Id))
            {
                continue;
            }

            _scheduler.Commit(c.Id, c.Pos, c.Vel, time);
            _sentScratch.Add(c);
        }

        LastSentCount = _sentScratch.Count;
        LastDeferredCount = LastWantedCount - LastSentCount;
        return _sentScratch;
    }
}
