using CycloneDDS.Runtime;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2 ("Done: a headless console DDS+DR harness ... asserts round-trip") --
// a headless, no-window proof that the DDS data path actually round-trips end to end in this process:
// EngineHost -> DdsPublisher -> CycloneDDS -> DdsSubscriber, for all four topics. No rendering, no DrClock
// yet (that is a later phase) -- this only proves the wire is live and decodes correctly.
public static class LoopbackSelfTest
{
    // `netPath` is a resolved *.net.xml path (Program.cs resolves a scenario/sandbox directory to one
    // before calling this, exactly like `--mode local` already does).
    public static int Run(string netPath)
    {
        using var host = new EngineHost(netPath);
        using var participant = new DdsParticipant();
        using var publisher = new DdsPublisher(host, participant);
        using var subscriber = new DdsSubscriber(participant);

        // DDS discovery is async -- give the intra-process writer/reader pairs time to match before
        // anything is published (the proven loopback probe's pattern).
        Thread.Sleep(500);

        publisher.PublishGeometryOnce();

        for (var i = 0; i < 30; i++)
        {
            publisher.PublishStep();
            Thread.Sleep(100);
            subscriber.Pump();
        }

        // A short drain window in case the last few writes are still in flight.
        for (var i = 0; i < 5 && !subscriber.GeometryComplete; i++)
        {
            Thread.Sleep(100);
            subscriber.Pump();
        }

        var expectedLanes = host.Network.LanesByHandle.Count;
        var gotLanes = subscriber.Geometry.Count;
        var geometryOk = subscriber.GeometryComplete && gotLanes == expectedLanes;

        var vehicleOk = false;
        var examplePos = double.NaN;
        var exampleHandle = default(Sim.Core.VehicleHandle);
        foreach (var kv in subscriber.History)
        {
            if (kv.Value.Count == 0)
            {
                continue;
            }

            var last = kv.Value[^1];
            if (double.IsFinite(last.Record.Pos))
            {
                vehicleOk = true;
                examplePos = last.Record.Pos;
                exampleHandle = kv.Key;
                break;
            }
        }

        var expectedTl = host.Snapshot.TlCount;
        var gotTl = subscriber.TlStateByLane.Count;
        var tlOk = expectedTl == 0 || gotTl == expectedTl;

        var pass = geometryOk && vehicleOk && tlOk;

        Console.WriteLine($"SELFTEST geometry: expectedLanes={expectedLanes} gotLanes={gotLanes} complete={subscriber.GeometryComplete}");
        Console.WriteLine($"SELFTEST vehicles: trackedHandles={subscriber.History.Count} firstFinitePos={vehicleOk} exampleHandle={exampleHandle} examplePos={examplePos}");
        Console.WriteLine($"SELFTEST tl: expectedTl={expectedTl} gotTl={gotTl}");
        Console.WriteLine(pass ? "SELFTEST: PASS" : "SELFTEST: FAIL");

        return pass ? 0 : 1;
    }
}
