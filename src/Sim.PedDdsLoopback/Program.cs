using System.Diagnostics;
using CycloneDDS.Runtime;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Replication.Dds;

// docs/PEDESTRIAN-DDS-TRANSPORT-DESIGN.md §6 -- the LIVE CycloneDDS loopback proof for the ped wire, the
// ped mirror of src/Sim.Viewer/LoopbackSelfTest.cs. Same real PedLodManager population and same
// reconstruction bounds as the hermetic tests/Sim.Pedestrians.Tests/Lod/PedReplicationRoundTripTests.cs,
// but the InMemory byte bus is swapped for DdsPedReplicationSink/Source over a real DdsParticipant -- so
// this proves "server == IG survives REAL DDS serialization + transport + async discovery", not just an
// in-process struct hand-off. Returns 0 on PASS, 1 on any violation (LoopbackSelfTest's console contract).

const double dt = 0.1;
const int steps = 200;      // 20 s @ dt=0.1: covers Ped1's 10 s route and Ped2's 17.25 s timeline
const int stepSleepMs = 20; // per-step settle so each step's DDS samples deliver before the pump

var ok = true;
void Check(bool cond, string label)
{
    Console.WriteLine((cond ? "  OK   " : "  FAIL ") + label);
    if (!cond)
    {
        ok = false;
    }
}

Console.WriteLine("PED-DDS-LOOPBACK: live CycloneDDS server==IG proof");

var publisher = new PedPublisher();
var manager = new PedLodManager(new StraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);

// --- Ped 1: low-power PathArc, exact reconstruction expected (whole-meter points + 2.0 m/s survive the
// int32-cm + float32 wire packing losslessly, same as the hermetic test) -----------------------------------
var pathArcPath = new[] { new Vec2(0.0, 0.0), new Vec2(20.0, 0.0) };
manager.AddPed(id: 1, pathArcPath, maxSpeed: 2.0, radius: 0.3, now: 0.0);

// --- Ped 2: lively low-power ActivityTimeline, exact reconstruction (double-precision wire, arbitrary
// values) --------------------------------------------------------------------------------------------------
var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
{
    new WalkSegment(new[] { new Vec2(0.0, 10.0), new Vec2(11.7, 10.0) }, 1.53),
    new PauseSegment(1.25, "sip"),
    new WalkSegment(new[] { new Vec2(11.7, 10.0), new Vec2(11.7, 21.4) }, 1.53),
});
manager.AddPedLively(id: 2, timeline, maxSpeed: 1.53, radius: 0.3, now: 0.0);

// --- Ped 3: starts low-power PathArc, promotes to FreeKinematic under an interest source -------------------
var pathArc3 = new[] { new Vec2(100.0, 0.0), new Vec2(120.0, 0.0) };
manager.AddPed(id: 3, pathArc3, maxSpeed: 1.4, radius: 0.3, now: 0.0);

var field = new InterestField();
field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
field.Register(new InterestSource(new Vec2(100.0, 0.0), promoteRadius: 3.0, demoteRadius: 6.0), InterestSourceKind.EntityAttached);
var noEntities = Array.Empty<WorldDisc>();

using var participant = new DdsParticipant();
using var sink = new DdsPedReplicationSink(participant);
using var source = new DdsPedReplicationSource(participant);
var replicationPublisher = new PedReplicationPublisher(sink);
var receiver = new PedReplicationReceiver(source);

// DDS discovery is async -- let the intra-process writer/reader pairs match before the first publish
// (LoopbackSelfTest's proven pattern).
Thread.Sleep(500);

// Spawn-time PathArc/ActivityTimeline legs were already appended by AddPed/AddPedLively -- push them now.
replicationPublisher.Publish(publisher.Events);
Thread.Sleep(stepSleepMs);
source.Pump();
receiver.Drain();

var everPromoted = false;
var promotionObservedOnWire = false;
var highPowerSampleCount = 0;
var maxHighPowerError = 0.0;

var now = 0.0;
var sw = Stopwatch.StartNew();
for (var i = 0; i < steps; i++)
{
    var beforeCount = publisher.Events.Count;
    manager.Step(now, dt, field, noEntities);
    now += dt;

    var newEvents = new List<PedEvent>(publisher.Events.Count - beforeCount);
    for (var e = beforeCount; e < publisher.Events.Count; e++)
    {
        newEvents.Add(publisher.Events[e]);
    }

    replicationPublisher.Publish(newEvents);
    Thread.Sleep(stepSleepMs);
    source.Pump();
    receiver.Drain();

    if (manager.ModelOf(3) == PedDrModel.FreeKinematic)
    {
        everPromoted = true;
        if (receiver.Ig.Knows(3) && receiver.Ig.ModelOf(3) == PedDrModel.FreeKinematic)
        {
            promotionObservedOnWire = true;
            var server3 = manager.PositionOf(3, now);
            var ig3 = receiver.Ig.Reconstruct(3, now);
            maxHighPowerError = Math.Max(maxHighPowerError, (server3 - ig3).Abs);
            highPowerSampleCount++;
        }
    }
}

sw.Stop();

Check(everPromoted, "Ped 3 promoted to FreeKinematic (server side)");
Check(promotionObservedOnWire, "receiver.ModelOf(3) flipped to FreeKinematic over the wire");
Check(highPowerSampleCount > 0, $"at least one high-power (quantized) sample compared ({highPowerSampleCount})");

Check(manager.ModelOf(1) == PedDrModel.PathArc, "server Ped 1 == PathArc");
Check(receiver.Ig.ModelOf(1) == PedDrModel.PathArc, "receiver Ped 1 == PathArc");
Check(manager.ModelOf(2) == PedDrModel.ActivityTimeline, "server Ped 2 == ActivityTimeline");
Check(receiver.Ig.ModelOf(2) == PedDrModel.ActivityTimeline, "receiver Ped 2 == ActivityTimeline");

// Exact low-power sweep (reliable/durable topics -> lossless bytes -> bit-identical reconstruction).
var exactSamples = 0;
var mismatches1 = 0;
var mismatches2 = 0;
for (var t = 0.0; t <= now; t += dt / 4.0)
{
    var server1 = manager.PositionOf(1, t);
    var ig1 = receiver.Ig.Reconstruct(1, t);
    if (server1.X != ig1.X || server1.Y != ig1.Y)
    {
        mismatches1++;
    }

    var server2 = manager.PositionOf(2, t);
    var ig2 = receiver.Ig.ReconstructSample(2, t).Pos;
    if (server2.X != ig2.X || server2.Y != ig2.Y)
    {
        mismatches2++;
    }

    exactSamples++;
}

Check(exactSamples > 30, $"fine exact sweep taken ({exactSamples} samples)");
Check(mismatches1 == 0, $"Ped 1 (PathArc) exact reconstruction over live DDS ({mismatches1} mismatches)");
Check(mismatches2 == 0, $"Ped 2 (ActivityTimeline) exact reconstruction over live DDS ({mismatches2} mismatches)");
Check(maxHighPowerError <= 0.02, $"high-power reconstruction error {maxHighPowerError:E4} m <= 0.02 m budget");

// --- Live single-stream bandwidth readout (docs §6; P6-1 SC3 live-transport half) -------------------------
var simSeconds = steps * dt;
var totalBytes = sink.CrowdBytesPublished + sink.PathArcBytesPublished + sink.ActivityBytesPublished + sink.LifecycleBytesPublished;
Console.WriteLine();
Console.WriteLine($"[bandwidth] measured over {simSeconds:0.0} s sim ({sw.Elapsed.TotalSeconds:0.0} s wall), 3-ped mix (1 promoted high-power):");
Console.WriteLine($"[bandwidth]   crowd     : {sink.CrowdBytesPublished,8} B  ({sink.CrowdBytesPublished / simSeconds,8:0.0} B/sim-s)");
Console.WriteLine($"[bandwidth]   patharc   : {sink.PathArcBytesPublished,8} B  ({sink.PathArcBytesPublished / simSeconds,8:0.0} B/sim-s)");
Console.WriteLine($"[bandwidth]   activity  : {sink.ActivityBytesPublished,8} B  ({sink.ActivityBytesPublished / simSeconds,8:0.0} B/sim-s)");
Console.WriteLine($"[bandwidth]   lifecycle : {sink.LifecycleBytesPublished,8} B  ({sink.LifecycleBytesPublished / simSeconds,8:0.0} B/sim-s)");
Console.WriteLine($"[bandwidth]   TOTAL     : {totalBytes,8} B  ({totalBytes / simSeconds,8:0.0} B/sim-s)");
Console.WriteLine($"[measured] high-power samples: {highPowerSampleCount}, max error: {maxHighPowerError:E4} m");

Console.WriteLine();
Console.WriteLine(ok ? "PED-DDS-LOOPBACK: PASS" : "PED-DDS-LOOPBACK: FAIL");
return ok ? 0 : 1;

// No real navmesh needed: PedLodManager only calls FindPath on a promote/demote, and a null result falls
// back to a straight line from the ped's current position to its destination -- exactly this stub.
sealed class StraightLineNav : IPedNavigation
{
    public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
}
