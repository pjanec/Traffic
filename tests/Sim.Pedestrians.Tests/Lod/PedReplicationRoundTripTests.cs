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
}
