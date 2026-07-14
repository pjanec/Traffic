using Sim.Replication;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-NATIVE-VIEWER.md DDS topics: GeometryCodec packs the one-time static lane geometry a remote
// viewer needs to draw roads; TlCodec packs the low-rate traffic-light state. Both ship in the hermetic
// SumoSharp.Replication package (like FrameCodec/FrameChunker), so they get hermetic round-trip coverage
// here. No external deps.
public class RungB27GeometryTlCodecTests
{
    [Fact]
    public void GeometryFrame_RoundTrips_VariableLengthLanes()
    {
        var lanes = new[]
        {
            new GeometryCodec.LaneGeo(0, isInternal: false, width: 3.2f, length: 100.0f,
                new (float, float)[] { (0f, 0f), (50f, 0f), (100f, 0f) }),
            new GeometryCodec.LaneGeo(7, isInternal: true, width: 3.0f, length: 12.5f,
                new (float, float)[] { (100f, 0f), (105f, 5f) }),
            new GeometryCodec.LaneGeo(42, isInternal: false, width: 3.5f, length: 3.0f,
                new (float, float)[] { (105f, 5f) }),
        };

        var buf = new byte[GeometryCodec.GeometrySize(lanes)];
        var written = GeometryCodec.WriteGeometry(buf, lanes);
        Assert.Equal(buf.Length, written);

        var outp = new List<GeometryCodec.LaneGeo>();
        Assert.Equal(3, GeometryCodec.ReadGeometry(buf, outp));
        for (var i = 0; i < 3; i++)
        {
            Assert.Equal(lanes[i].Handle, outp[i].Handle);
            Assert.Equal(lanes[i].IsInternal, outp[i].IsInternal);
            Assert.Equal(lanes[i].Width, outp[i].Width);           // float-exact values chosen above
            Assert.Equal(lanes[i].Length, outp[i].Length);
            Assert.Equal(lanes[i].Points.Length, outp[i].Points.Length);
            for (var p = 0; p < lanes[i].Points.Length; p++)
            {
                Assert.Equal(lanes[i].Points[p], outp[i].Points[p]);
            }
        }
    }

    [Fact]
    public void GeometryReadGeometry_IsAdditive_AcrossChunks()
    {
        // Build many lanes, plan chunks under a small byte budget, write each chunk, and reassemble by
        // accumulating into ONE list (ReadGeometry appends) -- how the DDS subscriber rebuilds a big net.
        var lanes = new GeometryCodec.LaneGeo[50];
        for (var i = 0; i < lanes.Length; i++)
        {
            lanes[i] = new GeometryCodec.LaneGeo(i, i % 2 == 0, 3.2f, i * 1.0f,
                new (float, float)[] { (i, 0f), (i, 1f), (i, 2f) });
        }

        var chunks = GeometryCodec.PlanChunks(lanes, maxPayloadBytes: 256);
        Assert.True(chunks.Count > 1); // 256 B budget forces several chunks

        var reassembled = new List<GeometryCodec.LaneGeo>();
        foreach (var (start, count) in chunks)
        {
            var slice = lanes.AsSpan(start, count);
            var buf = new byte[GeometryCodec.GeometrySize(slice)];
            Assert.True(buf.Length <= 256);
            GeometryCodec.WriteGeometry(buf, slice);
            GeometryCodec.ReadGeometry(buf, reassembled);
        }

        Assert.Equal(lanes.Length, reassembled.Count);
        for (var i = 0; i < lanes.Length; i++)
        {
            Assert.Equal(lanes[i].Handle, reassembled[i].Handle);
            Assert.Equal(lanes[i].Points.Length, reassembled[i].Points.Length);
        }
    }

    [Fact]
    public void TlFrame_RoundTrips_AndPartialReadClamps()
    {
        var entries = new[]
        {
            new TlCodec.TlEntry(3, (byte)'G'),
            new TlCodec.TlEntry(4, (byte)'r'),
            new TlCodec.TlEntry(9, (byte)'y'),
        };

        var buf = new byte[TlCodec.TlSize(entries.Length)];
        Assert.Equal(buf.Length, TlCodec.WriteTl(buf, entries));

        var outp = new TlCodec.TlEntry[3];
        Assert.Equal(3, TlCodec.ReadTl(buf, outp));
        for (var i = 0; i < 3; i++)
        {
            Assert.Equal(entries[i].LaneHandle, outp[i].LaneHandle);
            Assert.Equal(entries[i].Signal, outp[i].Signal);
        }

        var one = new TlCodec.TlEntry[1];
        Assert.Equal(1, TlCodec.ReadTl(buf, one)); // clamps to dst.Length
        Assert.Equal(3, one[0].LaneHandle);
    }
}
