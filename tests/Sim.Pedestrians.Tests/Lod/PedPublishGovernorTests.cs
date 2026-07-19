using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Replication;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// P3-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- the pedestrian analog of the vehicle
// stack's DrErrorPublishPolicy/PublishScheduler tests, plus the NEW global bandwidth governor and the
// bandwidth meter that proves the §7 budget empirically. See PedReplicationRoundTripTests.cs for the
// (unchanged, still-ungated) P3-1 round-trip coverage this task must not disturb.
public class PedPublishGovernorTests
{
    private readonly ITestOutputHelper _output;

    public PedPublishGovernorTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double Dt = 0.1;
    private const int Steps = 200; // 20 s run, long enough to span several MaxInterval heartbeats

    // ------------------------------------------------------------------------------------------------
    // 1. DR-error gating cuts steady traffic + IG stays within tolerance.
    // ------------------------------------------------------------------------------------------------

    [Fact]
    public void SteadyPed_IsPublishedNearHeartbeatRate_AndIgStaysWithinPosTol()
    {
        var policy = new PedDrErrorPublishPolicy(); // PosTol=0.1, VelTol=0.5, MaxInterval=3.0
        var scheduler = new PedPublishScheduler(policy);
        var ig = new HeadlessIg();
        ig.Apply(new DrSwitchEvent(1, 0.0, PedDrModel.PathArc, PedDrModel.FreeKinematic));

        var pos = new Vec2(0.0, 0.0);
        var vel = new Vec2(1.2, 0.0); // constant velocity -- perfectly predictable
        var sendCount = 0;
        var maxErr = 0.0;
        var time = 0.0;

        for (var i = 0; i < Steps; i++)
        {
            time += Dt;
            pos += vel * Dt;

            if (scheduler.ShouldPublish(1, pos, vel, time))
            {
                ig.Apply(new FreeKinematicSample(1, time, pos, vel));
                sendCount++;
            }

            var err = (pos - ig.Reconstruct(1, time)).Abs;
            maxErr = Math.Max(maxErr, err);
        }

        // Expected sends: only the MaxInterval heartbeat -- ceil(20 s / 3 s) plus the first-sighting
        // send already counted in that cadence, i.e. a small single-digit count, NOT one per step.
        var expectedHeartbeats = (int)Math.Ceiling(Steps * Dt / policy.MaxInterval);
        _output.WriteLine($"[P3-2 measured] steady ped: sent {sendCount}/{Steps} steps (heartbeat cadence ~{expectedHeartbeats}), max IG error {maxErr:E4} m");

        Assert.True(sendCount <= expectedHeartbeats + 2, $"expected steady ped sends near the heartbeat cadence (~{expectedHeartbeats}), got {sendCount}");
        Assert.True(sendCount * 20 < Steps, $"expected sends to be << step count, got {sendCount}/{Steps}");
        Assert.True(maxErr <= policy.PosTol + 1e-9, $"IG reconstruction error {maxErr:E4} m exceeded PosTol {policy.PosTol} m for a steady mover");
    }

    [Fact]
    public void ManeuveringPed_IsSentOften_AndIgStaysWithinPosTol()
    {
        var policy = new PedDrErrorPublishPolicy();
        var scheduler = new PedPublishScheduler(policy);
        var ig = new HeadlessIg();
        ig.Apply(new DrSwitchEvent(2, 0.0, PedDrModel.PathArc, PedDrModel.FreeKinematic));

        // A hard, continuous maneuver (avoiding/dodging): constant-speed circular motion, so both the
        // heading (velocity direction) and the linear-extrapolation position error are CONTINUOUSLY
        // diverging -- not a single one-off event -- exactly the "avoiding" case the scheduler must
        // catch promptly rather than only reacting to isolated spikes.
        const double radius = 5.0;
        const double speed = 1.2;
        var omega = speed / radius;
        Vec2 CircPos(double t) => new(radius * Math.Cos(omega * t), radius * Math.Sin(omega * t));
        Vec2 CircVel(double t) => new(-speed * Math.Sin(omega * t), speed * Math.Cos(omega * t));

        var sendCount = 0;
        var maxErr = 0.0;
        var time = 0.0;

        for (var i = 0; i < Steps; i++)
        {
            time += Dt;
            var pos = CircPos(time);
            var vel = CircVel(time);

            if (scheduler.ShouldPublish(2, pos, vel, time))
            {
                ig.Apply(new FreeKinematicSample(2, time, pos, vel));
                sendCount++;
            }

            var err = (pos - ig.Reconstruct(2, time)).Abs;
            maxErr = Math.Max(maxErr, err);
        }

        _output.WriteLine($"[P3-2 measured] maneuvering ped: sent {sendCount}/{Steps} steps, max IG error {maxErr:E4} m");

        Assert.True(sendCount > Steps / 10, $"expected the maneuvering ped to be sent far more often than the steady one, got {sendCount}/{Steps}");
        Assert.True(maxErr <= policy.PosTol + 1e-9, $"IG reconstruction error {maxErr:E4} m exceeded PosTol {policy.PosTol} m for a maneuvering mover");
    }

    // ------------------------------------------------------------------------------------------------
    // 2. Bandwidth under budget at the target mix.
    // ------------------------------------------------------------------------------------------------

    [Fact]
    public void TargetMix_100kAllHighPower_StaysUnder500MbitPerSecond()
    {
        const int population = 100_000;
        const int steps = 5;

        var meter = new PedBandwidthMeter();
        var bus = new InMemoryPedReplicationBus();
        // Ungated (scheduler: null) -- this test measures the worst-case "every ped every step"
        // bandwidth the design doc's §7 estimate is about, not the DR-error-reduced figure.
        var publisher = new PedReplicationPublisher(bus.Sink, scheduler: null, governor: null, meter: meter, stepDt: Dt);

        var time = 0.0;
        for (var step = 0; step < steps; step++)
        {
            time += Dt;
            var events = new List<PedEvent>(population);
            for (var id = 0; id < population; id++)
            {
                var pos = new Vec2(id * 0.01, step * 0.1);
                var vel = new Vec2(1.0, 0.0);
                events.Add(new FreeKinematicSample(id, time, pos, vel));
            }

            publisher.Publish(events);
            bus.Source.Pump();
        }

        var peakMbit = meter.PeakStepMbitPerSecond(Dt);
        var wholeRunMbit = meter.MbitPerSecond(steps * Dt);
        _output.WriteLine($"[P3-2 measured] target-mix {population} all-high-power quantized: peak {peakMbit:F2} Mbit/s, whole-run avg {wholeRunMbit:F2} Mbit/s (budget 500 Mbit/s)");

        Assert.True(peakMbit < 500.0, $"target-mix peak rate {peakMbit:F2} Mbit/s exceeded the 500 Mbit/s budget");
        Assert.True(bus.Source.LatestCrowdFrame.Count == population, "the bus did not decode the full crowd frame back");
    }

    // ------------------------------------------------------------------------------------------------
    // 3. Mass-promotion spike stays under budget WITH the governor.
    // ------------------------------------------------------------------------------------------------

    [Fact]
    public void MassPromotionSpike_GovernorKeepsPeakUnderBudget_AndCatchesUpWithinFewSteps()
    {
        const int population = 400_000;
        const double ceilingMbitPerSecond = 500.0;

        var meter = new PedBandwidthMeter();
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, meter, ceilingMbitPerSecond);
        var bus = new InMemoryPedReplicationBus();
        var publisher = new PedReplicationPublisher(bus.Sink, scheduler, governor, meter, stepDt: Dt);
        var receiver = new PedReplicationReceiver(bus.Source);

        // A genuine constant-velocity mover: TRUE position actually advances by Vel*t (analytic, not
        // just a fixed point relabeled each step) so the "true vs reconstructed" comparison below is a
        // real linear-extrapolation check, not an artifact of a stale fixture.
        const double vel = 1.0;
        static Vec2 VelOf(int id) => new(vel, 0.0);
        static Vec2 PosOf(int id, double t) => new(id * 0.001 + vel * t, 0.0);

        // --- Step 1: the spike -- every ped promotes AND sends its first sample in the SAME step. ---
        var time = Dt;
        var spikeEvents = new List<PedEvent>(population * 2);
        for (var id = 0; id < population; id++)
        {
            spikeEvents.Add(new DrSwitchEvent(id, time, PedDrModel.PathArc, PedDrModel.FreeKinematic));
        }

        for (var id = 0; id < population; id++)
        {
            spikeEvents.Add(new FreeKinematicSample(id, time, PosOf(id, time), VelOf(id)));
        }

        publisher.Publish(spikeEvents);
        bus.Source.Pump();
        receiver.Drain();

        var spikeWanted = governor.LastWantedCount;
        var spikeSent = governor.LastSentCount;
        var spikeDeferred = governor.LastDeferredCount;
        var spikeGoverned = governor.LastStepGoverned;
        var spikePeakMbit = meter.PeakStepMbitPerSecond(Dt);

        _output.WriteLine($"[P3-2 measured] mass-promotion spike ({population} peds, ceiling {ceilingMbitPerSecond} Mbit/s): "
            + $"wanted={spikeWanted}, sent={spikeSent}, deferred={spikeDeferred}, governed={spikeGoverned}, peak={spikePeakMbit:F2} Mbit/s");

        Assert.True(spikeGoverned, "expected the governor to engage (thin the send set) during the mass-promotion spike");
        Assert.True(spikeDeferred > 0, "expected some peds to be deferred by the governor during the spike");
        Assert.True(spikePeakMbit < ceilingMbitPerSecond, $"peak single-step rate {spikePeakMbit:F2} Mbit/s was not kept under the {ceilingMbitPerSecond} Mbit/s ceiling");
        Assert.True(scheduler.TrackedCount == spikeSent, "the scheduler should have committed exactly the sent (not the deferred) candidates");

        // --- A few more steps of the SAME population (no further lifecycle switches): catch-up. ---
        var caughtUpWithinSteps = -1;
        for (var followUp = 1; followUp <= 10 && caughtUpWithinSteps < 0; followUp++)
        {
            time += Dt;
            var events = new List<PedEvent>(population);
            for (var id = 0; id < population; id++)
            {
                events.Add(new FreeKinematicSample(id, time, PosOf(id, time), VelOf(id)));
            }

            publisher.Publish(events);
            bus.Source.Pump();
            receiver.Drain();

            if (scheduler.TrackedCount == population)
            {
                caughtUpWithinSteps = followUp;
            }
        }

        _output.WriteLine($"[P3-2 measured] backlog ({population - spikeSent} deferred peds) fully caught up within {caughtUpWithinSteps} follow-up step(s)");

        Assert.True(caughtUpWithinSteps > 0 && caughtUpWithinSteps <= 10, "the deferred backlog was not fully caught up within a few follow-up steps");

        // The IG (over the real wire, via the InMemory bus) now knows every ped, including whichever
        // ones were deferred during the spike, and reconstructs them within a slightly looser bound
        // than the steady-state PosTol (0.1 m) -- these peds never actually maneuvered, so in practice
        // the bound is met with a wide margin; the looser bound simply accounts for the extra
        // extrapolation step(s) a deferred ped is dead-reckoned through before it is finally sent.
        const double catchUpBound = 0.05;
        var maxCatchUpError = 0.0;
        for (var id = 0; id < population; id += 37_000) // a spread sample, not an exhaustive scan
        {
            Assert.True(receiver.Ig.Knows(id), $"ped {id} was never observed on the wire after catch-up");
            Assert.Equal(PedDrModel.FreeKinematic, receiver.Ig.ModelOf(id));
            var err = (PosOf(id, time) - receiver.Ig.Reconstruct(id, time)).Abs;
            maxCatchUpError = Math.Max(maxCatchUpError, err);
        }

        _output.WriteLine($"[P3-2 measured] max IG error across sampled peds after catch-up: {maxCatchUpError:E4} m (bound {catchUpBound} m)");
        Assert.True(maxCatchUpError <= catchUpBound, $"post-catch-up IG error {maxCatchUpError:E4} m exceeded the looser {catchUpBound} m bound");
    }

    // ------------------------------------------------------------------------------------------------
    // 4. Low-power invariant: zero per-step crowd-frame bytes, only the one-time PathArc/timeline +
    //    heartbeats -- unchanged by P3-2 (gating/governor are irrelevant since a low-power ped never
    //    produces a FreeKinematicSample in the first place).
    // ------------------------------------------------------------------------------------------------

    private sealed class StraightLineNav : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    [Fact]
    public void LowPowerOnlyPopulation_EmitsZeroPerStepCrowdFrameBytes_OverTheRun()
    {
        var publisherEvents = new PedPublisher();
        // dwellSeconds huge + no interest sources registered at all -> Query() always reports
        // AnyWithinPromote=false, so nothing in this population ever promotes for the whole run.
        var manager = new PedLodManager(new StraightLineNav(), publisherEvents, arriveRadius: 0.3, dwellSeconds: 1.0);

        manager.AddPed(id: 1, new[] { new Vec2(0, 0), new Vec2(50, 0) }, maxSpeed: 1.4, radius: 0.3, now: 0.0);
        manager.AddPed(id: 2, new[] { new Vec2(0, 5), new Vec2(50, 5) }, maxSpeed: 1.3, radius: 0.3, now: 0.0);
        manager.AddPed(id: 3, new[] { new Vec2(0, 10), new Vec2(50, 10) }, maxSpeed: 1.5, radius: 0.3, now: 0.0);

        var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
        {
            new WalkSegment(new[] { new Vec2(0.0, 20.0), new Vec2(30.0, 20.0) }, 1.4),
        });
        manager.AddPedLively(id: 4, timeline, maxSpeed: 1.4, radius: 0.3, now: 0.0);
        manager.AddPedLively(id: 5, timeline, maxSpeed: 1.4, radius: 0.3, now: 0.0);

        var meter = new PedBandwidthMeter();
        var bus = new InMemoryPedReplicationBus();
        var wirePublisher = new PedReplicationPublisher(bus.Sink, scheduler: null, governor: null, meter: meter, stepDt: Dt);

        var field = new InterestField(); // zero sources registered
        var noEntities = Array.Empty<WorldDisc>();

        wirePublisher.Publish(publisherEvents.Events); // the spawn-time PathArc/ActivityTimeline legs
        bus.Source.Pump();

        var now = 0.0;
        const int steps = 400; // 40 s -- several multiples of the 3 s heartbeat interval
        for (var i = 0; i < steps; i++)
        {
            var before = publisherEvents.Events.Count;
            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            var fresh = new List<PedEvent>(publisherEvents.Events.Count - before);
            for (var e = before; e < publisherEvents.Events.Count; e++)
            {
                fresh.Add(publisherEvents.Events[e]);
            }

            wirePublisher.Publish(fresh);
            bus.Source.Pump();
        }

        _output.WriteLine($"[P3-2 measured] low-power-only run ({steps} steps, {steps * Dt:F0}s): "
            + $"CrowdFrame bytes={meter.TotalBytes(PedBandwidthTopic.CrowdFrame)}, "
            + $"PathArc count={meter.TotalCount(PedBandwidthTopic.PathArc)}, "
            + $"ActivityTimeline count={meter.TotalCount(PedBandwidthTopic.ActivityTimeline)}, "
            + $"Lifecycle count={meter.TotalCount(PedBandwidthTopic.Lifecycle)}, "
            + $"Heartbeat count={meter.TotalCount(PedBandwidthTopic.Heartbeat)}");

        Assert.Equal(0, meter.TotalBytes(PedBandwidthTopic.CrowdFrame));
        Assert.Equal(0, meter.TotalCount(PedBandwidthTopic.CrowdFrame));
        Assert.Equal(0.0, meter.MbitPerSecondForTopic(PedBandwidthTopic.CrowdFrame, steps * Dt));

        Assert.Equal(3, meter.TotalCount(PedBandwidthTopic.PathArc)); // one per plain PathArc ped, sent once
        Assert.Equal(2, meter.TotalCount(PedBandwidthTopic.ActivityTimeline)); // one per lively ped, sent once
        Assert.Equal(2, meter.TotalCount(PedBandwidthTopic.Lifecycle)); // the PathArc->ActivityTimeline switch per lively ped
        Assert.True(meter.TotalCount(PedBandwidthTopic.Heartbeat) > 0, "expected at least some liveness heartbeats over a 40 s run");

        Assert.Equal(0, bus.Source.LatestCrowdFrame.Count);
    }
}
