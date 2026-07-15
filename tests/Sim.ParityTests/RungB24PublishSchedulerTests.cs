using Sim.Core;
using Sim.Replication;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-DEADRECKONING.md §7 — the PublishScheduler wraps a stateless IPublishPolicy with the per-vehicle
// last-sent bookkeeping the adaptive rate needs, and prunes despawned movers. Transport-agnostic, keyed by
// VehicleHandle, no external deps -> safe in the hermetic gate. (The policy predicate itself is RungB22.)
public class RungB24PublishSchedulerTests
{
    // Demo-style intervals over a 1 s step: uncertain movers every step, predictable every 3rd step.
    private static PublishScheduler NewScheduler() => new(new DefaultPublishPolicy
    {
        FastInterval = 1.0,
        SlowInterval = 3.0,
        AccelThreshold = 0.3,
    });

    [Fact]
    public void FirstSighting_AlwaysPublishes_ThenDefersPredictable_ButNotActive()
    {
        var sched = NewScheduler();
        var steady = new VehicleHandle(1, 1);   // |accel| below threshold -> predictable
        var active = new VehicleHandle(2, 1);   // hard accel -> full rate

        // Step at t=0: neither seen before -> both sent (secondsSinceLastSent = +inf).
        Assert.True(Offer(sched, steady, accel: 0.05, t: 0.0));
        Assert.True(Offer(sched, active, accel: 2.0, t: 0.0));
        sched.EndStep();
        Assert.Equal(2, sched.TrackedCount);

        // t=1: predictable deferred (1 s < 3 s slow interval); active resent (1 s >= 1 s fast interval).
        Assert.False(Offer(sched, steady, accel: 0.05, t: 1.0));
        Assert.True(Offer(sched, active, accel: 2.0, t: 1.0));
        sched.EndStep();

        // t=2: predictable still deferred (2 s < 3 s).
        Assert.False(Offer(sched, steady, accel: 0.05, t: 2.0));
        sched.EndStep();

        // t=3: predictable reaches the slow keep-alive interval (3 s >= 3 s) -> resent.
        Assert.True(Offer(sched, steady, accel: 0.05, t: 3.0));
        sched.EndStep();

        // Having just been resent at t=3, it defers again at t=4 (only 1 s elapsed).
        Assert.False(Offer(sched, steady, accel: 0.05, t: 4.0));
        sched.EndStep();
    }

    [Fact]
    public void Manoeuvring_ForcesFullRate_EvenWhenAccelIsLow()
    {
        var sched = NewScheduler();
        var h = new VehicleHandle(5, 2);

        Assert.True(Offer(sched, h, accel: 0.0, t: 0.0));       // first sighting
        sched.EndStep();
        // Low accel would normally defer, but a mid-manoeuvre (lane change / lateral dodge) forces full rate.
        Assert.True(Offer(sched, h, accel: 0.0, t: 1.0, manoeuvring: true));
        sched.EndStep();
    }

    [Fact]
    public void EndStep_PrunesDespawnedMovers()
    {
        var sched = NewScheduler();
        var a = new VehicleHandle(1, 1);
        var b = new VehicleHandle(2, 1);

        Offer(sched, a, accel: 2.0, t: 0.0);
        Offer(sched, b, accel: 2.0, t: 0.0);
        sched.EndStep();
        Assert.Equal(2, sched.TrackedCount);

        // Next step only `a` is offered -> `b` despawned; EndStep forgets it.
        Offer(sched, a, accel: 2.0, t: 1.0);
        sched.EndStep();
        Assert.Equal(1, sched.TrackedCount);

        sched.Reset();
        Assert.Equal(0, sched.TrackedCount);
    }

    [Fact]
    public void NullPolicy_Throws() =>
        Assert.Throws<ArgumentNullException>(() => new PublishScheduler(null!));

    private static bool Offer(
        PublishScheduler sched, VehicleHandle h, double accel, double t, bool manoeuvring = false) =>
        sched.ShouldPublish(h, DrModel.LaneArc, pos: 10.0 * t, posLat: 0.0, speed: 10.0, accel: accel,
            latSpeed: 0.0, laneHandle: 1, time: t, laneChangingOrManoeuvring: manoeuvring);
}

// SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md §8 (Stage A) — DrErrorPublishPolicy.ShouldPublish exercised
// directly with hand-built PublishSignals; no scheduler involved.
public class DrErrorPublishPolicyTests
{
    private static readonly VehicleHandle H = new(1, 1);

    [Fact]
    public void Steady_InTolerance_BeforeHeartbeat_DoesNotPublish()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: 1.0, false,
            posError: 0.05, latError: 0.0, laneChanged: false);
        Assert.False(policy.ShouldPublish(s));
    }

    [Fact]
    public void PosErrorOverTolerance_Publishes()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: 1.0, false,
            posError: 0.5, latError: 0.0, laneChanged: false);
        Assert.True(policy.ShouldPublish(s));
    }

    [Fact]
    public void LatErrorOverTolerance_Publishes()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: 1.0, false,
            posError: 0.0, latError: 0.3, laneChanged: false);
        Assert.True(policy.ShouldPublish(s));
    }

    [Fact]
    public void LaneChanged_Publishes()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: 1.0, false,
            posError: 0.0, latError: 0.0, laneChanged: true);
        Assert.True(policy.ShouldPublish(s));
    }

    [Fact]
    public void HeartbeatReached_Publishes()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: 3.0, false,
            posError: 0.0, latError: 0.0, laneChanged: false);
        Assert.True(policy.ShouldPublish(s));
    }

    [Fact]
    public void FirstSighting_Publishes()
    {
        var policy = new DrErrorPublishPolicy();
        var s = new PublishSignals(H, DrModel.LaneArc, 10, 0, secondsSinceLastSent: double.PositiveInfinity, false,
            posError: 0.0, latError: 0.0, laneChanged: false);
        Assert.True(policy.ShouldPublish(s));
    }
}
