using System.Buffers.Binary;

namespace Sim.Replication;

// SUMOSHARP-NATIVE-VIEWER.md's "DDS topics" section / SUMOSHARP-DEADRECKONING.md §4.3 (durable geometry
// topic) — the canonical packed wire format for the ONE-TIME static lane geometry a remote viewer needs to
// draw roads without the net file. Same idea as FrameCodec (one little-endian byte layout, allocation-free
// write/read into caller spans), but each record is VARIABLE length (a lane's point count varies), so there
// is no fixed RecordSize the way FrameCodec's VehicleRecord has one — callers use LaneSize/GeometrySize to
// size buffers, and PlanChunks to split a large network across multiple <=64 KiB DDS payloads (mirroring
// FrameChunker's byte-budget chunking for the DdsWireFrame blob).
//
// Header (4 B): version(1) reserved(1) count(u16). Per-lane record, VERSION-dependent layout:
//
//   Version 1 (legacy, still READ, never WRITTEN): 15 B fixed + points*8 B --
//     handle(i32) isInternal(u8) width(f32) length(f32) pointCount(u16) then pointCount*(x,y)(f32,f32).
//
//   Version 2 (current, ADDITIVE over v1 -- docs/LIVE-CITY-VIEWERS-DESIGN.md §3.2 / -TASKS.md E1): 16 B
//   fixed + points*8 B + (hasZ ? points*4 B : 0) -- the SAME v1 prefix plus one `hasZ` flag byte, plus an
//   APPENDED per-point Z block (not interleaved with x/y, so a v1-shaped reader's field layout for
//   handle/isInternal/width/length/pointCount/points is untouched -- only a version-aware reader looks
//   past the point block for Z):
//     handle(i32) isInternal(u8) width(f32) length(f32) pointCount(u16) hasZ(u8)
//     then pointCount*(x,y)(f32,f32)
//     then [only if hasZ==1] pointCount*z(f32)
//   `hasZ` is per-lane (not a global net property) because a mixed net can have some lanes with real
//   elevation data (Lane.ShapeZ from a 3-tuple net.xml shape) and others without.
public static class GeometryCodec
{
    // The WRITER always emits the current version; ReadGeometry dispatches on the version byte so a v1
    // stream (no hasZ byte, no Z block -- e.g. an old .simrec recorded before this task) still parses,
    // with every decoded LaneGeo.Z left null (== "Z absent"), exactly like a v2 lane with hasZ==0.
    public const byte Version = 2;

    public const int HeaderSize = 4;
    public const int LaneFixedSizeV1 = 4 + 1 + 4 + 4 + 2; // handle + isInternal + width + length + pointCount
    public const int LaneFixedSize = LaneFixedSizeV1 + 1; // + hasZ
    public const int PointSize = 8; // (float x, float y)
    public const int ZSize = 4; // (float z), appended per point only when hasZ

    // One lane's static geometry: a dense lane handle, whether it is an internal (":"-prefixed id) lane,
    // its width/length, and its polyline points (already float — the wire precision). `Points` is an array
    // (not a span) so a decoded LaneGeo can outlive the source buffer, e.g. held in a subscriber's registry.
    // `Z` is the ADDITIVE per-point elevation (index-aligned with `Points`, mirroring Sim.Ingest's
    // `Lane.Shape`/`Lane.ShapeZ` pairing) -- null when the source lane carried no elevation (the common
    // case today; every committed net is flat), never a breaking change to the `Points` tuple shape itself.
    public readonly struct LaneGeo
    {
        public LaneGeo(int handle, bool isInternal, float width, float length, (float X, float Y)[] points, float[]? z = null)
        {
            Handle = handle;
            IsInternal = isInternal;
            Width = width;
            Length = length;
            Points = points;
            Z = z is { Length: > 0 } && z.Length == points.Length ? z : null;
        }

        public int Handle { get; }
        public bool IsInternal { get; }
        public float Width { get; }
        public float Length { get; }
        public (float X, float Y)[] Points { get; }
        // Null when absent; otherwise index-aligned 1:1 with Points (same length).
        public float[]? Z { get; }
    }

    public static int LaneSize(int pointCount, bool hasZ = false) =>
        LaneFixedSize + pointCount * PointSize + (hasZ ? pointCount * ZSize : 0);

    public static int LaneSize(in LaneGeo lane) => LaneSize(lane.Points.Length, lane.Z is not null);

    public static int GeometrySize(ReadOnlySpan<LaneGeo> lanes)
    {
        var size = HeaderSize;
        for (var i = 0; i < lanes.Length; i++)
        {
            size += LaneSize(lanes[i]);
        }

        return size;
    }

    public static int WriteGeometry(Span<byte> dst, ReadOnlySpan<LaneGeo> lanes)
    {
        var size = GeometrySize(lanes);
        if (dst.Length < size)
        {
            throw new ArgumentException("destination too small for the geometry frame.", nameof(dst));
        }

        dst[0] = Version;
        dst[1] = 0; // reserved
        BinaryPrimitives.WriteUInt16LittleEndian(dst.Slice(2, 2), (ushort)lanes.Length);

        var o = HeaderSize;
        for (var i = 0; i < lanes.Length; i++)
        {
            ref readonly var lane = ref lanes[i];
            BinaryPrimitives.WriteInt32LittleEndian(dst.Slice(o, 4), lane.Handle); o += 4;
            dst[o++] = (byte)(lane.IsInternal ? 1 : 0);
            WriteF32(dst.Slice(o, 4), lane.Width); o += 4;
            WriteF32(dst.Slice(o, 4), lane.Length); o += 4;
            var points = lane.Points;
            BinaryPrimitives.WriteUInt16LittleEndian(dst.Slice(o, 2), (ushort)points.Length); o += 2;
            var z = lane.Z; // already validated non-empty + length-matched by the LaneGeo ctor
            dst[o++] = (byte)(z is not null ? 1 : 0);
            for (var p = 0; p < points.Length; p++)
            {
                WriteF32(dst.Slice(o, 4), points[p].X); o += 4;
                WriteF32(dst.Slice(o, 4), points[p].Y); o += 4;
            }

            if (z is not null)
            {
                for (var p = 0; p < z.Length; p++)
                {
                    WriteF32(dst.Slice(o, 4), z[p]); o += 4;
                }
            }
        }

        return size;
    }

    // Decodes every lane record in `src` and appends it to `dst`; returns the count appended. Additive
    // (does not clear `dst`) so a subscriber can accumulate lanes across several chunked DDS samples.
    public static int ReadGeometry(ReadOnlySpan<byte> src, List<LaneGeo> dst)
    {
        if (src.Length < HeaderSize)
        {
            throw new ArgumentException("geometry frame shorter than header.", nameof(src));
        }

        // src[0] version (1 = legacy 2-D-only, 2 = current, additive Z); src[1] reserved.
        var version = src[0];
        var count = BinaryPrimitives.ReadUInt16LittleEndian(src.Slice(2, 2));
        var o = HeaderSize;
        for (var i = 0; i < count; i++)
        {
            var handle = BinaryPrimitives.ReadInt32LittleEndian(src.Slice(o, 4)); o += 4;
            var isInternal = src[o++] != 0;
            var width = ReadF32(src.Slice(o, 4)); o += 4;
            var length = ReadF32(src.Slice(o, 4)); o += 4;
            var pointCount = BinaryPrimitives.ReadUInt16LittleEndian(src.Slice(o, 2)); o += 2;

            var hasZ = false;
            if (version >= 2)
            {
                hasZ = src[o++] != 0;
            }

            var points = new (float X, float Y)[pointCount];
            for (var p = 0; p < pointCount; p++)
            {
                var x = ReadF32(src.Slice(o, 4)); o += 4;
                var y = ReadF32(src.Slice(o, 4)); o += 4;
                points[p] = (x, y);
            }

            float[]? z = null;
            if (hasZ)
            {
                z = new float[pointCount];
                for (var p = 0; p < pointCount; p++)
                {
                    z[p] = ReadF32(src.Slice(o, 4)); o += 4;
                }
            }

            dst.Add(new LaneGeo(handle, isInternal, width, length, points, z));
        }

        return count;
    }

    // Splits `lanes` into contiguous [Start, Count) ranges so that GeometrySize(lanes[Start..Start+Count])
    // never exceeds `maxPayloadBytes` (the DDS blob's fixed payload cap) — the geometry analog of
    // FrameChunker, needed here (rather than reusing FrameChunker) because a lane record's size is
    // variable (point count varies), not the uniform per-record size FrameChunker assumes. Always makes
    // progress (a single oversized lane still gets its own one-lane chunk) so this never infinite-loops.
    public static List<(int Start, int Count)> PlanChunks(IReadOnlyList<LaneGeo> lanes, int maxPayloadBytes)
    {
        var chunks = new List<(int, int)>();
        var start = 0;
        while (start < lanes.Count)
        {
            var size = HeaderSize;
            var count = 0;
            while (start + count < lanes.Count)
            {
                var next = LaneSize(lanes[start + count]);
                if (count > 0 && size + next > maxPayloadBytes)
                {
                    break;
                }

                size += next;
                count++;
            }

            chunks.Add((start, count));
            start += count;
        }

        return chunks;
    }

    // float <-> LE bytes via int bits (BinaryPrimitives.Write/ReadSingleLittleEndian is net5+, absent on
    // ns2.1) — mirrors FrameCodec's private helpers.
    private static void WriteF32(Span<byte> dst, float value) =>
        BinaryPrimitives.WriteInt32LittleEndian(dst, BitConverter.SingleToInt32Bits(value));

    private static float ReadF32(ReadOnlySpan<byte> src) =>
        BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(src));
}
