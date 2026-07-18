using System.Buffers.Binary;
using System.Text;
using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7 "PathArc DR model" / LIVE-POC-1's
// ActivityTimelineRecord) -- the wire form of the "timeline sent once" broadcast: an ActivityTimeline is
// broadcast ONCE per lively-low-power leg (spawn, or a later re-plan), exactly like PathArcRecord's
// "path sent once" discipline, and the IG reconstructs pose/animation/visibility by calling the SAME
// ActivityTimeline.PoseAt the server calls (docs/PEDESTRIAN-DESIGN.md §8 "server == IG").
//
// Deliberately DIFFERENT from Sim.Replication.FrameCodec's PathArc wire record: that one quantizes
// positions to int32-cm and speed/startTime to float32 (a compactness tradeoff acceptable there because
// PathArc reconstruction is a purely deterministic replay of clean navmesh coordinates). Here every
// scalar is written as a full IEEE-754 double, because P3-1's whole point is that decoding this wire
// form and calling PoseAt must reproduce the server's pose BIT-FOR-BIT (see ActivityTimelineWireTests) --
// any lossy quantization would undermine that identity for the animation-rich Dwell/Interact poses, which
// (unlike a Walk leg's polyline) are often authored off-path and don't have PathArc's "coordinates are
// already at network resolution" luck.
//
// Lives in Sim.Pedestrians (not Sim.Replication) because ActivityTimeline/ActivitySegment are pedestrian-
// layer types and Sim.Replication must stay free of a Sim.Pedestrians reference -- see
// PedReplicationPublisher.cs / PedReplicationReceiver.cs, the one place allowed to bridge the two.
//
// Little-endian, matching FrameCodec's own convention (BinaryPrimitives.Write*LittleEndian). Unlike
// FrameCodec (which multi-targets netstandard2.1 and so bit-casts doubles/floats through Int64/Int32),
// Sim.Pedestrians targets net8.0 only, so this uses BinaryPrimitives.WriteDoubleLittleEndian /
// ReadDoubleLittleEndian directly.
//
// Format (all integers/doubles little-endian):
//   double T0
//   int32  segmentCount
//   segmentCount * {
//     byte kind (0 = Walk, 1 = Pause, 2 = Dwell, 3 = Interact)
//     Walk:     double speed, int32 pointCount, pointCount * (double X, double Y)
//     Pause:    double dur, string animTag
//     Dwell:    double poseX, poseY, headingX, headingY, dur, byte visible, string animTag
//     Interact: double poseX, poseY, headingX, headingY, dur, int32 partnerId, string animTag
//   }
//   string := int32 utf8ByteLen + utf8 bytes
public static class ActivityTimelineWire
{
    private const byte KindWalk = 0;
    private const byte KindPause = 1;
    private const byte KindDwell = 2;
    private const byte KindInteract = 3;

    public static byte[] Encode(ActivityTimeline timeline)
    {
        using var buffer = new MemoryStream();
        Span<byte> scratch = stackalloc byte[8];

        WriteDouble(buffer, scratch, timeline.T0);
        WriteInt32(buffer, scratch, timeline.Segments.Count);

        foreach (var segment in timeline.Segments)
        {
            switch (segment)
            {
                case WalkSegment w:
                    buffer.WriteByte(KindWalk);
                    WriteDouble(buffer, scratch, w.Speed);
                    WriteInt32(buffer, scratch, w.Path.Count);
                    foreach (var p in w.Path)
                    {
                        WriteDouble(buffer, scratch, p.X);
                        WriteDouble(buffer, scratch, p.Y);
                    }

                    break;

                case PauseSegment p:
                    buffer.WriteByte(KindPause);
                    WriteDouble(buffer, scratch, p.Dur);
                    WriteString(buffer, scratch, p.AnimTag);
                    break;

                case DwellSegment d:
                    buffer.WriteByte(KindDwell);
                    WriteDouble(buffer, scratch, d.Pose.X);
                    WriteDouble(buffer, scratch, d.Pose.Y);
                    WriteDouble(buffer, scratch, d.Heading.X);
                    WriteDouble(buffer, scratch, d.Heading.Y);
                    WriteDouble(buffer, scratch, d.Dur);
                    buffer.WriteByte((byte)(d.Visible ? 1 : 0));
                    WriteString(buffer, scratch, d.AnimTag);
                    break;

                case InteractSegment i:
                    buffer.WriteByte(KindInteract);
                    WriteDouble(buffer, scratch, i.Pose.X);
                    WriteDouble(buffer, scratch, i.Pose.Y);
                    WriteDouble(buffer, scratch, i.Heading.X);
                    WriteDouble(buffer, scratch, i.Heading.Y);
                    WriteDouble(buffer, scratch, i.Dur);
                    WriteInt32(buffer, scratch, i.PartnerId);
                    WriteString(buffer, scratch, i.AnimTag);
                    break;

                default:
                    throw new InvalidOperationException($"Unknown ActivitySegment type: {segment.GetType()}");
            }
        }

        return buffer.ToArray();
    }

    public static ActivityTimeline Decode(ReadOnlySpan<byte> src)
    {
        var o = 0;
        var t0 = ReadDouble(src, ref o);
        var count = ReadInt32(src, ref o);
        var segments = new ActivitySegment[count];
        for (var i = 0; i < count; i++)
        {
            var kind = src[o];
            o += 1;
            segments[i] = kind switch
            {
                KindWalk => ReadWalk(src, ref o),
                KindPause => ReadPause(src, ref o),
                KindDwell => ReadDwell(src, ref o),
                KindInteract => ReadInteract(src, ref o),
                _ => throw new ArgumentException($"unknown ActivitySegment kind byte {kind}", nameof(src)),
            };
        }

        return new ActivityTimeline(t0, segments);
    }

    private static WalkSegment ReadWalk(ReadOnlySpan<byte> src, ref int o)
    {
        var speed = ReadDouble(src, ref o);
        var n = ReadInt32(src, ref o);
        var path = new Vec2[n];
        for (var k = 0; k < n; k++)
        {
            var x = ReadDouble(src, ref o);
            var y = ReadDouble(src, ref o);
            path[k] = new Vec2(x, y);
        }

        return new WalkSegment(path, speed);
    }

    private static PauseSegment ReadPause(ReadOnlySpan<byte> src, ref int o)
    {
        var dur = ReadDouble(src, ref o);
        var tag = ReadString(src, ref o);
        return new PauseSegment(dur, tag);
    }

    private static DwellSegment ReadDwell(ReadOnlySpan<byte> src, ref int o)
    {
        var px = ReadDouble(src, ref o);
        var py = ReadDouble(src, ref o);
        var hx = ReadDouble(src, ref o);
        var hy = ReadDouble(src, ref o);
        var dur = ReadDouble(src, ref o);
        var visible = src[o] != 0;
        o += 1;
        var tag = ReadString(src, ref o);
        return new DwellSegment(new Vec2(px, py), new Vec2(hx, hy), dur, tag, visible);
    }

    private static InteractSegment ReadInteract(ReadOnlySpan<byte> src, ref int o)
    {
        var px = ReadDouble(src, ref o);
        var py = ReadDouble(src, ref o);
        var hx = ReadDouble(src, ref o);
        var hy = ReadDouble(src, ref o);
        var dur = ReadDouble(src, ref o);
        var partnerId = ReadInt32(src, ref o);
        var tag = ReadString(src, ref o);
        return new InteractSegment(new Vec2(px, py), new Vec2(hx, hy), dur, tag, partnerId);
    }

    private static void WriteDouble(MemoryStream buffer, Span<byte> scratch, double value)
    {
        BinaryPrimitives.WriteDoubleLittleEndian(scratch[..8], value);
        buffer.Write(scratch[..8]);
    }

    private static void WriteInt32(MemoryStream buffer, Span<byte> scratch, int value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(scratch[..4], value);
        buffer.Write(scratch[..4]);
    }

    private static void WriteString(MemoryStream buffer, Span<byte> scratch, string value)
    {
        var bytes = Encoding.UTF8.GetBytes(value);
        WriteInt32(buffer, scratch, bytes.Length);
        buffer.Write(bytes, 0, bytes.Length);
    }

    private static double ReadDouble(ReadOnlySpan<byte> src, ref int o)
    {
        var v = BinaryPrimitives.ReadDoubleLittleEndian(src.Slice(o, 8));
        o += 8;
        return v;
    }

    private static int ReadInt32(ReadOnlySpan<byte> src, ref int o)
    {
        var v = BinaryPrimitives.ReadInt32LittleEndian(src.Slice(o, 4));
        o += 4;
        return v;
    }

    private static string ReadString(ReadOnlySpan<byte> src, ref int o)
    {
        var len = ReadInt32(src, ref o);
        var s = Encoding.UTF8.GetString(src.Slice(o, len));
        o += len;
        return s;
    }
}
