using Sim.Core;
using Sim.Replication;
using Xunit;

namespace Sim.ParityTests;

// docs/SUMOSHARP-PACKAGING-DESIGN.md P1 (D8) — hermetic proof that DDS is one binding among several: this
// test exercises IReplicationSink/IReplicationSource entirely through InMemoryReplicationBus, using ONLY
// Sim.Replication (no CycloneDDS, no Sim.Viewer.Core). If a consumer's code were coded against the
// interfaces, it could not tell it was talking to this bus rather than DdsPublisher/DdsSubscriber.
public class ReplicationInMemoryTransportTests
{
    [Fact]
    public void Geometry_PublishesAndCompletes()
    {
        var bus = new InMemoryReplicationBus();
        var lanes = new[]
        {
            new GeometryCodec.LaneGeo(1, false, 3.2f, 50.0f, new[] { (0f, 0f), (50f, 0f) }),
            new GeometryCodec.LaneGeo(2, true, 2.5f, 8.0f, new[] { (50f, 0f), (58f, 0f) }),
        };

        bus.Sink.PublishGeometry(lanes);
        bus.Source.Pump();

        Assert.True(bus.Source.GeometryComplete);
        Assert.Equal(2, bus.Source.Geometry.Count);
        Assert.Equal(lanes[0].Width, bus.Source.Geometry[1].Width);
        Assert.Equal(lanes[1].Length, bus.Source.Geometry[2].Length);
    }

    [Fact]
    public void Lifecycle_Spawn_PopulatesDims()
    {
        var bus = new InMemoryReplicationBus();
        var handle = new VehicleHandle(3, 1);
        var record = new LifecycleRecord(handle, isSpawn: true, vTypeId: 0, length: 4.5f, width: 1.8f);

        bus.Sink.PublishLifecycle(record);
        bus.Source.Pump();

        Assert.True(bus.Source.Dims.TryGetValue(handle, out var dims));
        Assert.Equal(4.5f, dims.Length);
        Assert.Equal(1.8f, dims.Width);
    }

    [Fact]
    public void Lifecycle_Despawn_RemovesDimsAndHistory()
    {
        var bus = new InMemoryReplicationBus();
        var handle = new VehicleHandle(4, 1);
        bus.Sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: true, vTypeId: 0, length: 4.5f, width: 1.8f));
        var recIn = new VehicleRecord(handle, DrModel.LaneArc, laneHandle: 2,
            pos: 10.0, posLat: 0.0, speed: 5.0, accel: 0.0, latSpeed: 0.0, default);
        bus.Sink.PublishFrame(1, 1.0, new[] { recIn });
        bus.Source.Pump();
        Assert.True(bus.Source.Dims.ContainsKey(handle));
        Assert.True(bus.Source.History.ContainsKey(handle));

        bus.Sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: false, vTypeId: 0, length: 0f, width: 0f));
        bus.Source.Pump();

        Assert.False(bus.Source.Dims.ContainsKey(handle));
        Assert.False(bus.Source.History.ContainsKey(handle));
    }

    [Fact]
    public void Frame_AppendsHistory_TracksLatestTime_TryGetLatestMatches()
    {
        var bus = new InMemoryReplicationBus();
        var handle = new VehicleHandle(5, 1);
        var upcoming = new UpcomingLanes(stackalloc int[] { 6, 7 });
        var recIn = new VehicleRecord(handle, DrModel.LaneArc, laneHandle: 6,
            pos: 12.5, posLat: -0.25, speed: 8.5, accel: 1.25, latSpeed: 0.1, upcoming);

        bus.Sink.PublishFrame(step: 10, time: 3.5, new[] { recIn });
        bus.Source.Pump();

        Assert.True(bus.Source.History.TryGetValue(handle, out var hist));
        Assert.True(hist.Count >= 1);
        var newest = hist[hist.Count - 1];
        Assert.Equal(3.5, newest.TimestampSeconds);
        Assert.Equal(recIn.Handle, newest.Record.Handle);
        Assert.Equal(recIn.Model, newest.Record.Model);
        Assert.Equal(recIn.LaneHandle, newest.Record.LaneHandle);
        Assert.Equal(recIn.Pos, newest.Record.Pos);
        Assert.Equal(recIn.PosLat, newest.Record.PosLat);
        Assert.Equal(recIn.Speed, newest.Record.Speed);
        Assert.Equal(recIn.Accel, newest.Record.Accel);
        Assert.Equal(recIn.LatSpeed, newest.Record.LatSpeed);
        for (var k = 0; k < UpcomingLanes.Count; k++)
        {
            Assert.Equal(recIn.Upcoming[k], newest.Record.Upcoming[k]);
        }

        Assert.Equal(3.5, bus.Source.LatestVehicleSampleTime);

        Assert.True(bus.Source.TryGetLatest(handle, out var latest));
        Assert.Equal(newest.TimestampSeconds, latest.TimestampSeconds);
        Assert.Equal(newest.Record.Handle, latest.Record.Handle);
        Assert.Equal(newest.Record.Pos, latest.Record.Pos);
    }

    [Fact]
    public void TrafficLights_PublishesLatestSignalPerLane()
    {
        var bus = new InMemoryReplicationBus();
        var entries = new[] { new TlCodec.TlEntry(laneHandle: 9, signal: (byte)'G') };

        bus.Sink.PublishTrafficLights(step: 1, time: 1.0, entries);
        bus.Source.Pump();

        Assert.True(bus.Source.TlStateByLane.TryGetValue(9, out var signal));
        Assert.Equal((byte)'G', signal);
    }

    [Fact]
    public void Connected_BecomesTrueAfterFirstPumpWithData()
    {
        var bus = new InMemoryReplicationBus();
        Assert.False(bus.Source.Connected);

        bus.Sink.PublishGeometry(Array.Empty<GeometryCodec.LaneGeo>());
        bus.Source.Pump();

        Assert.True(bus.Source.Connected);
    }
}
