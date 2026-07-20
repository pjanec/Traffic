using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Replication;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- proves "server == IG survives
// serialization": a REAL PedLodManager population (a low-power PathArc ped, a lively ActivityTimeline
// ped, and a ped that gets promoted to FreeKinematic by an interest source) is stepped while its
// PedPublisher events are pushed through PedReplicationPublisher -> InMemoryPedReplicationBus (a true
// byte loopback: every publish call is serialized to bytes and deserialized back on Pump()) ->
// PedReplicationReceiver, which reconstructs a HeadlessIg from the wire alone. The receiver's
// reconstruction is then compared against the server's own PositionOf/ModelOf.
public class PedReplicationRoundTripTests
{
    private readonly ITestOutputHelper _output;

    public PedReplicationRoundTripTests(ITestOutputHelper output)
    {
        _output = output;
    }

    // No real navmesh is needed for this test: PedLodManager only calls IPedNavigation.FindPath when a
    // ped promotes/demotes, and a null result falls back to a straight line from the ped's current
    // (frozen) position to its destination (see PedLodManager.Step) -- exactly what this stub provides.
    // Using this (rather than the POC-0 fixture net) also keeps the low-power ped's authored path under
    // this test's own control, at coordinates chosen to survive FrameCodec's PathArc wire quantization
    // losslessly (see the exactness remarks on Ped1 below).
    private sealed class StraightLineNav : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    // W4: a nav that DOES route (straight start->goal) and reports a real 2 m half-width, so a demote's
    // resume leg has room to weave -- lets the test see the weave actually resume after coming back low-power.
    private sealed class StraightWideNav : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => new[] { start, goal };

        public IReadOnlyList<double> HalfWidthsAlong(IReadOnlyList<Vec2> path)
        {
            var w = new double[path.Count];
            Array.Fill(w, 2.0);
            return w;
        }
    }

    private const double Dt = 0.1;

    [Fact]
    public void ServerEqualsIg_OverTheWire_ForPathArc_ActivityTimeline_AndPromotedFreeKinematic()
    {
        var publisher = new PedPublisher();
        var manager = new PedLodManager(new StraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);

        // --- Ped 1: low-power PathArc, exact reconstruction expected -------------------------------
        // Coordinates/speed/startTime are all chosen to be exactly representable after FrameCodec's
        // PathArc wire packing (int32-cm positions, float32 speed/startTime): whole-meter points and a
        // speed of 2.0 m/s round-trip through cm-quantization and the float32 downcast with ZERO loss
        // (both are exact binary values), so the "sent-once, EXACT" identity holds bit-for-bit for this
        // ped even though the underlying wire codec is technically lossy for arbitrary doubles -- exactly
        // the situation a network of on-grid sidewalk coordinates produces in practice.
        var pathArcPath = new[] { new Vec2(0.0, 0.0), new Vec2(20.0, 0.0) };
        const double pathArcSpeed = 2.0;
        manager.AddPed(id: 1, pathArcPath, pathArcSpeed, radius: 0.3, now: 0.0);

        // --- Ped 2: lively low-power ActivityTimeline, exact reconstruction expected ---------------
        // ActivityTimelineWire preserves full double precision regardless of the chosen values (raw
        // IEEE-754 doubles on the wire, no quantization), so these points/speeds are deliberately NOT
        // round numbers -- proving exactness does not depend on lucky inputs the way Ped 1's does.
        var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
        {
            new WalkSegment(new[] { new Vec2(0.0, 10.0), new Vec2(11.7, 10.0) }, 1.53),
            new PauseSegment(1.25, "sip"),
            new WalkSegment(new[] { new Vec2(11.7, 10.0), new Vec2(11.7, 21.4) }, 1.53),
        });
        manager.AddPedLively(id: 2, timeline, maxSpeed: 1.53, radius: 0.3, now: 0.0);

        // --- Ped 3: starts low-power PathArc, promotes to FreeKinematic under an interest source ----
        var pathArc3 = new[] { new Vec2(100.0, 0.0), new Vec2(120.0, 0.0) };
        manager.AddPed(id: 3, pathArc3, maxSpeed: 1.4, radius: 0.3, now: 0.0);

        var field = new InterestField();
        // A dummy, far-away source keeps Ped 1/Ped 2 low-power for the whole run.
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        // Sits exactly on Ped 3's start position -> promotes quickly.
        field.Register(new InterestSource(new Vec2(100.0, 0.0), promoteRadius: 3.0, demoteRadius: 6.0), InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        var bus = new InMemoryPedReplicationBus();
        var replicationPublisher = new PedReplicationPublisher(bus.Sink);
        var receiver = new PedReplicationReceiver(bus.Source);

        // AddPed/AddPedLively above already appended each ped's spawn-time PathArcRecord /
        // ActivityTimelineRecord (+ switch) to publisher.Events BEFORE the stepping loop starts --
        // publish those onto the wire now so the receiver learns every ped's initial leg, exactly like
        // the loop below publishes each subsequent step's new events.
        replicationPublisher.Publish(publisher.Events);
        bus.Source.Pump();
        receiver.Drain();

        var everPromoted = false;
        var promotionObservedOnWire = false;
        var highPowerSampleCount = 0;
        var maxHighPowerError = 0.0;

        var now = 0.0;
        const int steps = 200; // 20 s @ Dt=0.1: covers Ped1's 10 s route and Ped2's 17.25 s timeline
        for (var i = 0; i < steps; i++)
        {
            var beforeCount = publisher.Events.Count;
            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            var newEvents = new List<PedEvent>(publisher.Events.Count - beforeCount);
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            replicationPublisher.Publish(newEvents);
            bus.Source.Pump();
            receiver.Drain();

            if (manager.ModelOf(3) == PedDrModel.FreeKinematic)
            {
                everPromoted = true;
                if (receiver.Ig.Knows(3) && receiver.Ig.ModelOf(3) == PedDrModel.FreeKinematic)
                {
                    promotionObservedOnWire = true;
                    var server3 = manager.PositionOf(3, now);
                    var ig3 = receiver.Ig.Reconstruct(3, now);
                    var err = (server3 - ig3).Abs;
                    maxHighPowerError = Math.Max(maxHighPowerError, err);
                    highPowerSampleCount++;
                }
            }
        }

        Assert.True(everPromoted, "Ped 3 never promoted to FreeKinematic despite sitting on the interest source");
        Assert.True(promotionObservedOnWire, "the receiver's ModelOf(3) never flipped to FreeKinematic after the promote lifecycle event arrived");
        Assert.True(highPowerSampleCount > 0, "expected at least one high-power (quantized) sample comparison");

        // ---- Exact low-power sweep (Ped 1: PathArc; Ped 2: ActivityTimeline) ----------------------
        Assert.Equal(PedDrModel.PathArc, manager.ModelOf(1));
        Assert.Equal(PedDrModel.PathArc, receiver.Ig.ModelOf(1));
        Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(2));
        Assert.Equal(PedDrModel.ActivityTimeline, receiver.Ig.ModelOf(2));

        var exactSamples1 = 0;
        var exactSamples2 = 0;
        for (var t = 0.0; t <= now; t += Dt / 4.0)
        {
            var server1 = manager.PositionOf(1, t);
            var ig1 = receiver.Ig.Reconstruct(1, t);
            Assert.Equal(server1.X, ig1.X);
            Assert.Equal(server1.Y, ig1.Y);
            exactSamples1++;

            var server2 = manager.PositionOf(2, t);
            var ig2 = receiver.Ig.ReconstructSample(2, t).Pos;
            Assert.Equal(server2.X, ig2.X);
            Assert.Equal(server2.Y, ig2.Y);
            exactSamples2++;
        }

        _output.WriteLine($"[P3-1 measured] exact low-power sweep samples: Ped1(PathArc)={exactSamples1}, Ped2(ActivityTimeline)={exactSamples2}");
        _output.WriteLine($"[P3-1 measured] high-power (quantized+extrapolated) samples: {highPowerSampleCount}, max error: {maxHighPowerError:E4} m");

        Assert.True(exactSamples1 > 30, $"expected a fine exact sweep for Ped1, got {exactSamples1}");
        Assert.True(exactSamples2 > 30, $"expected a fine exact sweep for Ped2, got {exactSamples2}");
        Assert.True(maxHighPowerError <= 0.02, $"high-power reconstruction error {maxHighPowerError:E4} m exceeded the 0.02 m cm-quantization + one-step-extrapolation budget");

        // Confirms this run genuinely exercised the "one PathArcRecord + heartbeats, never a
        // FreeKinematicSample" invariant for the low-power peds (sanity check the bridge didn't silently
        // drop or duplicate anything on the way to the wire).
        Assert.Equal(1, publisher.PathArcRecordsSent[1]);
        Assert.Equal(1, publisher.ActivityTimelineRecordsSent[2]);
        Assert.True(publisher.FreeKinematicSamplesSent.GetValueOrDefault(3) > 0);
    }

    // W3 (docs/PEDESTRIAN-WEAVE-PRODUCTION-DESIGN.md): server==IG for a WEAVING low-power ped, END TO END
    // over the real publisher -> byte-loopback bus -> receiver -> HeadlessIg path (not a direct
    // Encode/Decode). The weave seed + global seed + per-vertex half-widths ride the ActivityTimelineWire
    // blob the bridge ships, so the decoded timeline's PoseAt must reproduce the server's weaving pose
    // bit-for-bit -- AND the weave must actually be active (the reconstructed track leaves the centreline),
    // so this is not a vacuous "weave off" pass.
    [Fact]
    public void ServerEqualsIg_OverTheWire_ForWeavingActivityTimeline()
    {
        var publisher = new PedPublisher();
        var manager = new PedLodManager(new StraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);

        // A straight eastbound leg at y=10 with a 2 m half-width per vertex, a non-zero per-ped weave seed,
        // and a non-zero global seed -> the weave is ON. Deliberately non-round coordinates: exactness does
        // not depend on lucky inputs (ActivityTimelineWire carries raw doubles).
        var widths = new[] { 2.0, 2.0 };
        var timeline = new ActivityTimeline(
            t0: 0.0,
            new ActivitySegment[] { new WalkSegment(new[] { new Vec2(0.3, 10.0), new Vec2(41.7, 10.0) }, 1.37, widths) },
            seed: 0xC0FFEE1234UL,
            globalSeed: 42UL);
        manager.AddPedLively(id: 7, timeline, maxSpeed: 1.37, radius: 0.3, now: 0.0);

        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0)); // keep it low-power
        var noEntities = Array.Empty<WorldDisc>();

        var bus = new InMemoryPedReplicationBus();
        var replicationPublisher = new PedReplicationPublisher(bus.Sink);
        var receiver = new PedReplicationReceiver(bus.Source);

        replicationPublisher.Publish(publisher.Events);
        bus.Source.Pump();
        receiver.Drain();

        var now = 0.0;
        for (var i = 0; i < 320; i++) // ~30 m at 1.37 m/s + slack
        {
            var beforeCount = publisher.Events.Count;
            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            var newEvents = new List<PedEvent>();
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            replicationPublisher.Publish(newEvents);
            bus.Source.Pump();
            receiver.Drain();
        }

        Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(7));
        Assert.Equal(PedDrModel.ActivityTimeline, receiver.Ig.ModelOf(7));

        var maxLateral = 0.0;
        var samples = 0;
        for (var t = 0.0; t <= now; t += Dt / 4.0)
        {
            var server = manager.PositionOf(7, t);
            var ig = receiver.Ig.ReconstructSample(7, t).Pos;
            Assert.Equal(server.X, ig.X);   // bit-for-bit -- server==IG over the wire
            Assert.Equal(server.Y, ig.Y);
            maxLateral = Math.Max(maxLateral, Math.Abs(server.Y - 10.0)); // centreline is y=10
            samples++;
        }

        Assert.True(samples > 30, $"expected a fine sweep, got {samples}");
        Assert.True(maxLateral > 0.2, $"the weave must be active over the wire (reconstructed track leaves the centreline); max lateral was {maxLateral:F3} m");
        Assert.Equal(1, publisher.ActivityTimelineRecordsSent[7]);
        _output.WriteLine($"[W3 measured] weaving ped: {samples} exact over-the-wire samples, max lateral {maxLateral:F3} m");
    }

    // W4 (docs/PEDESTRIAN-WEAVE-PRODUCTION-DESIGN.md): a weaving ped that PROMOTES to reactive high-power and
    // then DEMOTES must RESUME the deterministic weave (not fall back to a flat PathArc leg), with no pop
    // across the LOD switch and server==IG exact again after the demote -- end to end over the real wire.
    // The production analog of Prototype D2's restore (no l_r needed: the resume leg is re-anchored exactly at
    // the frozen pose, so the Offset start-taper gives continuity).
    [Fact]
    public void WeavingPed_PromotesThenDemotes_ResumesWeave_NoPop_ServerEqualsIgOverTheWire()
    {
        var publisher = new PedPublisher();
        var manager = new PedLodManager(new StraightWideNav(), publisher, arriveRadius: 0.3, dwellSeconds: 0.5);

        var widths = new[] { 2.0, 2.0 };
        var timeline = new ActivityTimeline(
            t0: 0.0,
            new ActivitySegment[] { new WalkSegment(new[] { new Vec2(0.0, 10.0), new Vec2(60.0, 10.0) }, 1.3, widths) },
            seed: 0xBEEF77UL, globalSeed: 42UL);
        manager.AddPedLively(id: 9, timeline, maxSpeed: 1.3, radius: 0.3, now: 0.0);

        // An entity-attached source sitting at the start promotes the ped early; as it walks east it leaves
        // the demote radius and comes back low-power (resuming the weave) partway along the leg.
        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(2.0, 10.0), promoteRadius: 3.0, demoteRadius: 6.0), InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        var bus = new InMemoryPedReplicationBus();
        var replicationPublisher = new PedReplicationPublisher(bus.Sink);
        var receiver = new PedReplicationReceiver(bus.Source);
        replicationPublisher.Publish(publisher.Events);
        bus.Source.Pump();
        receiver.Drain();

        var everPromoted = false;
        var demotedToWeave = false;
        var demoteNow = -1.0;
        var lastPosBeforeDemote = Vec2.Zero;
        var prevModel = PedDrModel.ActivityTimeline;
        var prevServerPos = manager.PositionOf(9, 0.0);

        var now = 0.0;
        for (var i = 0; i < 700; i++)
        {
            var beforeCount = publisher.Events.Count;
            manager.Step(now, Dt, field, noEntities);
            var stepNow = now;
            now += Dt;

            var newEvents = new List<PedEvent>();
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            replicationPublisher.Publish(newEvents);
            bus.Source.Pump();
            receiver.Drain();

            var model = manager.ModelOf(9);
            if (model == PedDrModel.FreeKinematic)
            {
                everPromoted = true;
            }

            // Detect the FreeKinematic -> ActivityTimeline demote transition (the weave resume).
            if (everPromoted && !demotedToWeave && prevModel == PedDrModel.FreeKinematic && model == PedDrModel.ActivityTimeline)
            {
                demotedToWeave = true;
                demoteNow = stepNow;
                // No teleport across the switch: the first low-power pose is within one step of normal motion
                // of the last high-power pose (the resume leg is re-anchored at the frozen pose).
                var here = manager.PositionOf(9, stepNow);
                Assert.True((here - lastPosBeforeDemote).Abs < (1.3 * Dt) + 1e-6,
                    $"demote teleported: |Δ| = {(here - lastPosBeforeDemote).Abs:F4} m across the LOD switch");
            }

            if (model == PedDrModel.FreeKinematic)
            {
                lastPosBeforeDemote = manager.PositionOf(9, now);
            }

            prevModel = model;
            prevServerPos = manager.PositionOf(9, now);
        }

        Assert.True(everPromoted, "ped never promoted");
        Assert.True(demotedToWeave, "ped never demoted back to a weaving ActivityTimeline");
        Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(9));
        Assert.Equal(PedDrModel.ActivityTimeline, receiver.Ig.ModelOf(9)); // the demote->timeline switch arrived over the wire

        // server==IG EXACT after the demote (the resume leg reconstructs bit-for-bit), and the weave is active.
        var exact = 0;
        var maxLateralAfter = 0.0;
        for (var t = demoteNow + Dt; t <= now; t += Dt / 2.0)
        {
            var server = manager.PositionOf(9, t);
            var ig = receiver.Ig.ReconstructSample(9, t).Pos;
            Assert.Equal(server.X, ig.X);
            Assert.Equal(server.Y, ig.Y);
            maxLateralAfter = Math.Max(maxLateralAfter, Math.Abs(server.Y - 10.0));
            exact++;
        }

        Assert.True(exact > 20, $"expected a fine post-demote sweep, got {exact}");
        Assert.True(maxLateralAfter > 0.2, $"the weave should be active AFTER demote; max lateral was {maxLateralAfter:F3} m");
        _output.WriteLine($"[W4 measured] demote@{demoteNow:F1}s -> resumed weave: {exact} exact over-the-wire samples, max lateral after {maxLateralAfter:F3} m");
    }
}
