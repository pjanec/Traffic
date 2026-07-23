using System.Linq;
using Sim.Core;
using Sim.Replication;
using Xunit;

namespace Sim.Host.Tests;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §3, -TASKS.md Stage E (E1, E2) — codec-level round-trip coverage for
// the two ADDITIVE wire fields this stage lands: per-point Z on GeometryCodec.LaneGeo (E1) and the SUMO
// vehicle id on LifecycleRecord.Name (E2). GeometryCodec/IReplication already had round-trip coverage in
// Sim.ParityTests before this task (RungB27GeometryTlCodecTests / ReplicationInMemoryTransportTests) —
// those files are part of the "654 passed / 4 skipped" parity-gate baseline and are deliberately left
// untouched here; this NEW file lives in Sim.Host.Tests (which already references Sim.Replication and
// exercises ReplicationPublisher) instead, so the parity gate's pinned count never moves.
public class GeometryZAndLifecycleNameTests
{
    // ---- E1: GeometryCodec Z round-trip ----

    [Fact]
    public void GeometryCodec_RoundTrips_NonZeroPerPointZ()
    {
        var lanes = new[]
        {
            // A synthetic elevated lane (the same shape the demos/City3D/CityLib.Tests fixtures/
            // elevated.net.xml ramp lane E0_0 uses: 0m -> 4m -> 8m over 3 points).
            new GeometryCodec.LaneGeo(0, isInternal: false, width: 3.2f, length: 100f,
                new (float, float)[] { (0f, 0f), (50f, 0f), (100f, 0f) },
                new[] { 0f, 4f, 8f }),
            // A flat lane on the SAME wire frame (no Z) -- proves per-lane hasZ, not a global net flag.
            new GeometryCodec.LaneGeo(1, isInternal: false, width: 3.2f, length: 60f,
                new (float, float)[] { (200f, 0f), (260f, 0f) }),
        };

        var buf = new byte[GeometryCodec.GeometrySize(lanes)];
        var written = GeometryCodec.WriteGeometry(buf, lanes);
        Assert.Equal(buf.Length, written);
        Assert.Equal(GeometryCodec.Version, buf[0]); // current format version (2) always written

        var outp = new List<GeometryCodec.LaneGeo>();
        Assert.Equal(2, GeometryCodec.ReadGeometry(buf, outp));

        var ramp = outp.Single(l => l.Handle == 0);
        Assert.NotNull(ramp.Z);
        Assert.Equal(3, ramp.Z!.Length);
        Assert.Equal(0f, ramp.Z[0]);
        Assert.Equal(4f, ramp.Z[1]);
        Assert.Equal(8f, ramp.Z[2]);
        // Points themselves are untouched by the additive Z block.
        Assert.Equal(3, ramp.Points.Length);
        Assert.Equal((100f, 0f), ramp.Points[2]);

        var flat = outp.Single(l => l.Handle == 1);
        Assert.Null(flat.Z); // Z absent -- exactly like every committed (flat) net today
    }

    [Fact]
    public void GeometryCodec_2DLaneGeo_RoundTrips_WithZAbsentOrZero()
    {
        // No `z` argument at all -- the pre-Stage-E call shape (LaneGeo's ctor keeps it optional/additive).
        var lanes = new[]
        {
            new GeometryCodec.LaneGeo(7, isInternal: true, width: 3.0f, length: 12.5f,
                new (float, float)[] { (100f, 0f), (105f, 5f) }),
        };

        var buf = new byte[GeometryCodec.GeometrySize(lanes)];
        GeometryCodec.WriteGeometry(buf, lanes);

        var outp = new List<GeometryCodec.LaneGeo>();
        GeometryCodec.ReadGeometry(buf, outp);

        Assert.Single(outp);
        Assert.Null(outp[0].Z);
        Assert.Equal(2, outp[0].Points.Length);
        Assert.Equal((105f, 5f), outp[0].Points[1]);
    }

    [Fact]
    public void GeometryCodec_ReadGeometry_AcceptsLegacyVersion1Bytes_WithNoZBlock()
    {
        // Hand-encode a Version-1 frame (no hasZ byte, no Z block per lane) -- proves an old .simrec /
        // pre-Stage-E stream still parses, with Z simply absent (never a hard failure or misread).
        var lanes = new[]
        {
            new GeometryCodec.LaneGeo(3, isInternal: false, width: 3.2f, length: 50f,
                new (float, float)[] { (0f, 0f), (50f, 0f) }),
        };

        const int v1FixedSize = 4 + 1 + 4 + 4 + 2; // handle+isInternal+width+length+pointCount, NO hasZ byte
        var pointBytes = lanes[0].Points.Length * 8;
        var buf = new byte[4 + v1FixedSize + pointBytes]; // header(4) + one v1 lane record

        buf[0] = 1; // legacy version
        buf[1] = 0;
        System.Buffers.Binary.BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(2, 2), 1); // count=1

        var o = 4;
        System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(buf.AsSpan(o, 4), lanes[0].Handle); o += 4;
        buf[o++] = 0; // isInternal=false
        WriteF32(buf, ref o, lanes[0].Width);
        WriteF32(buf, ref o, lanes[0].Length);
        System.Buffers.Binary.BinaryPrimitives.WriteUInt16LittleEndian(buf.AsSpan(o, 2), (ushort)lanes[0].Points.Length); o += 2;
        foreach (var (x, y) in lanes[0].Points)
        {
            WriteF32(buf, ref o, x);
            WriteF32(buf, ref o, y);
        }

        Assert.Equal(buf.Length, o); // sanity: hand-encoded exactly the v1 layout, no trailing hasZ/Z bytes

        var outp = new List<GeometryCodec.LaneGeo>();
        Assert.Equal(1, GeometryCodec.ReadGeometry(buf, outp));
        Assert.Equal(3, outp[0].Handle);
        Assert.Null(outp[0].Z);
        Assert.Equal(2, outp[0].Points.Length);
        Assert.Equal((50f, 0f), outp[0].Points[1]);
    }

    private static void WriteF32(byte[] dst, ref int o, float value)
    {
        System.Buffers.Binary.BinaryPrimitives.WriteInt32LittleEndian(dst.AsSpan(o, 4), System.BitConverter.SingleToInt32Bits(value));
        o += 4;
    }

    // ---- E2: LifecycleRecord.Name round-trip (transport-neutral, via InMemoryReplicationBus) ----

    [Fact]
    public void InMemoryReplicationBus_Names_ResolvesSpawnedVehicleId_AndForgetsOnDespawn()
    {
        var bus = new InMemoryReplicationBus();
        var handle = new VehicleHandle(1, 1);

        bus.Sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: true, vTypeId: 0, length: 4.5f, width: 1.8f, name: "veh0.0"));
        bus.Source.Pump();

        Assert.True(bus.Source.Names.TryGetValue(handle, out var name));
        Assert.Equal("veh0.0", name);

        bus.Sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: false, vTypeId: 0, length: 0f, width: 0f));
        bus.Source.Pump();

        Assert.False(bus.Source.Names.ContainsKey(handle));
    }

    [Fact]
    public void LifecycleRecord_DefaultName_IsEmpty_ForPreExistingCallSites()
    {
        // The pre-Stage-E 5-arg call shape (no `name` argument) must still compile and default to "" --
        // ReplicationPublisher's despawn call site is exactly this shape.
        var rec = new LifecycleRecord(new VehicleHandle(2, 1), isSpawn: false, vTypeId: 0, length: 0f, width: 0f);
        Assert.Equal(string.Empty, rec.Name);
    }
}
