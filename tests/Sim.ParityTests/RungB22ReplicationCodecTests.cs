using Sim.Core;
using Sim.Ingest;
using Sim.Replication;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-DEADRECKONING.md §4.3 / §7 — the transport-agnostic replication layer: the canonical packed
// blob codec (round-trips vehicle + crowd frames), and the pluggable adaptive publish policy. Plus an
// end-to-end check that encoding the engine's read columns and decoding them reconstructs the engine's
// own pose via PoseResolver. No external deps -> safe in the hermetic gate.
public class RungB22ReplicationCodecTests
{
    private static string Net14 => Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle", "net.net.xml");

    [Fact]
    public void VehicleFrame_RoundTrips()
    {
        var recs = new[]
        {
            new VehicleRecord(new VehicleHandle(7, 2), DrModel.LaneArc, 3,
                pos: 12.5, posLat: -0.25, speed: 8.5, accel: 1.25, latSpeed: 0.0,
                new UpcomingLanes(stackalloc int[] { 3, 4, 5 })),
            new VehicleRecord(new VehicleHandle(9, 1), DrModel.Stationary, 6,
                pos: 100.0, posLat: 0.5, speed: 0.0, accel: 0.0, latSpeed: 0.0,
                new UpcomingLanes(stackalloc int[] { 6 })),
        };

        var buf = new byte[FrameCodec.VehicleFrameSize(recs.Length)];
        var written = FrameCodec.WriteVehicleFrame(buf, step: 42, time: 4.2f, recs);
        Assert.Equal(buf.Length, written);

        var header = FrameCodec.ReadHeader(buf);
        Assert.Equal(FrameCodec.KindVehicle, header.Kind);
        Assert.Equal(42u, header.Step);
        Assert.Equal(4.2f, header.Time, 0.0001f);
        Assert.Equal(2, header.Count);

        var outp = new VehicleRecord[2];
        Assert.Equal(2, FrameCodec.ReadVehicleFrame(buf, outp));
        for (var i = 0; i < 2; i++)
        {
            Assert.Equal(recs[i].Handle, outp[i].Handle);
            Assert.Equal(recs[i].Model, outp[i].Model);
            Assert.Equal(recs[i].LaneHandle, outp[i].LaneHandle);
            Assert.Equal(recs[i].Pos, outp[i].Pos);          // float-exact values chosen above
            Assert.Equal(recs[i].PosLat, outp[i].PosLat);
            Assert.Equal(recs[i].Speed, outp[i].Speed);
            Assert.Equal(recs[i].Accel, outp[i].Accel);
            for (var k = 0; k < UpcomingLanes.Count; k++)
                Assert.Equal(recs[i].Upcoming[k], outp[i].Upcoming[k]);
        }
    }

    [Fact]
    public void CrowdFrame_RoundTrips()
    {
        var recs = new[]
        {
            new CrowdRecord(new VehicleHandle(1, 1), x: 3.5, y: -2.25, z: 0.0, vx: 1.5, vy: -0.5, radius: 0.35),
            new CrowdRecord(new VehicleHandle(2, 3), x: 10.0, y: 20.0, z: 0.0, vx: 0.0, vy: 2.0, radius: 0.4),
        };
        var buf = new byte[FrameCodec.CrowdFrameSize(recs.Length)];
        FrameCodec.WriteCrowdFrame(buf, 5, 0.5f, recs);

        Assert.Equal(FrameCodec.KindCrowd, FrameCodec.ReadHeader(buf).Kind);
        var outp = new CrowdRecord[2];
        Assert.Equal(2, FrameCodec.ReadCrowdFrame(buf, outp));
        for (var i = 0; i < 2; i++)
        {
            Assert.Equal(recs[i].Handle, outp[i].Handle);
            Assert.Equal(recs[i].X, outp[i].X, 4);      // float-quantized on the wire
            Assert.Equal(recs[i].Y, outp[i].Y, 4);
            Assert.Equal(recs[i].Vx, outp[i].Vx, 4);
            Assert.Equal(recs[i].Vy, outp[i].Vy, 4);
            Assert.Equal(recs[i].Radius, outp[i].Radius, 4);
        }
    }

    [Fact]
    public void WriteVehicleFrame_TooSmall_Throws_And_PartialRead_Clamps()
    {
        var recs = new[]
        {
            new VehicleRecord(new VehicleHandle(1, 1), DrModel.LaneArc, 0, 1, 0, 1, 0, 0, default),
            new VehicleRecord(new VehicleHandle(2, 1), DrModel.LaneArc, 0, 2, 0, 1, 0, 0, default),
        };
        Assert.Throws<ArgumentException>(() => FrameCodec.WriteVehicleFrame(new byte[8], 0, 0, recs));

        var buf = new byte[FrameCodec.VehicleFrameSize(2)];
        FrameCodec.WriteVehicleFrame(buf, 0, 0, recs);
        var one = new VehicleRecord[1];
        Assert.Equal(1, FrameCodec.ReadVehicleFrame(buf, one)); // clamps to dst.Length
        Assert.Equal(recs[0].Handle, one[0].Handle);
    }

    [Fact]
    public void DefaultPublishPolicy_DefersPredictable_SendsUncertain()
    {
        var p = new DefaultPublishPolicy();

        // Predictable steady LaneArc follower: deferred until the slow keep-alive interval.
        Assert.False(p.ShouldPublish(new PublishSignals(new VehicleHandle(1, 1), DrModel.LaneArc,
            speed: 10, accel: 0.05, secondsSinceLastSent: 0.5, laneChangingOrManoeuvring: false)));
        Assert.True(p.ShouldPublish(new PublishSignals(new VehicleHandle(1, 1), DrModel.LaneArc,
            speed: 10, accel: 0.05, secondsSinceLastSent: 1.0, laneChangingOrManoeuvring: false)));

        // Stationary (stopped at a light) is predictable too -> deferred to the slow interval (issue #4).
        Assert.False(p.ShouldPublish(new PublishSignals(new VehicleHandle(3, 1), DrModel.Stationary,
            speed: 0, accel: 0.0, secondsSinceLastSent: 0.5, laneChangingOrManoeuvring: false)));
        Assert.True(p.ShouldPublish(new PublishSignals(new VehicleHandle(3, 1), DrModel.Stationary,
            speed: 0, accel: 0.0, secondsSinceLastSent: 1.0, laneChangingOrManoeuvring: false)));

        // Hard braking (high |accel|) -> full rate.
        Assert.True(p.ShouldPublish(new PublishSignals(new VehicleHandle(1, 1), DrModel.LaneArc,
            speed: 10, accel: -3.0, secondsSinceLastSent: 0.1, laneChangingOrManoeuvring: false)));

        // FreeKinematic (crowd / swerving) -> full rate.
        Assert.True(p.ShouldPublish(new PublishSignals(new VehicleHandle(2, 1), DrModel.FreeKinematic,
            speed: 1, accel: 0.0, secondsSinceLastSent: 0.1, laneChangingOrManoeuvring: false)));
    }

    // End-to-end: encode the engine's live read columns, decode them, and reconstruct the engine's OWN pose.
    [Fact]
    public void EncodeDecode_ReconstructsEnginePose()
    {
        var e = new Engine();
        e.LoadNetwork(Net14);
        var h = e.SpawnVehicle(e.DefaultVType, new[] { "e0" });
        for (var k = 0; k < 5; k++) e.Step();
        Assert.True(e.TryGetVehicle(h, out var s));

        Span<int> up = stackalloc int[UpcomingLanes.Count];
        var n = e.GetUpcomingLanes(h, up);
        var recIn = new[]
        {
            new VehicleRecord(h, DrModel.LaneArc, s.LaneHandle, s.Pos, s.PosLat, s.Speed,
                accel: 0.0, latSpeed: 0.0, new UpcomingLanes(up[..n])),
        };

        var buf = new byte[FrameCodec.VehicleFrameSize(1)];
        FrameCodec.WriteVehicleFrame(buf, (uint)e.StepCount, (float)e.CurrentTime, recIn);
        var recOut = new VehicleRecord[1];
        FrameCodec.ReadVehicleFrame(buf, recOut);
        var r = recOut[0];

        var src = new NetworkLaneSource(NetworkParser.Parse(Net14));
        Span<int> upOut = stackalloc int[UpcomingLanes.Count];
        var m = r.Upcoming.CopyTo(upOut);
        var state = new DrState { Model = r.Model, LaneHandle = r.LaneHandle, Pos = r.Pos, PosLat = r.PosLat };
        var pose = PoseResolver.Resolve(src, state, upOut[..m], default, dt: 0.0, RenderRealism.ParityTangent);

        // Float-quantized on the wire, so match the engine's (float) render columns within a small tolerance.
        Assert.Equal((double)s.X, pose.X, 1);
        Assert.Equal((double)s.Y, pose.Y, 1);
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
