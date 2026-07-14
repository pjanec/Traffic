using System.Buffers.Binary;

namespace Sim.Replication;

// SUMOSHARP-NATIVE-VIEWER.md's "DDS topics" section / SUMOSHARP-DEADRECKONING.md §5.2 — the low-rate
// traffic-light state packet: a controlled approach lane handle + its current signal char, mirroring
// SimulationSnapshot's TlLaneHandle/TlState columns. Small and fixed-size per entry, so unlike
// GeometryCodec this always fits one chunk for any scenario committed today; still exposed as a codec
// (not baked into the DDS type) so the same bytes could ride TCP/UDP too, matching FrameCodec's pattern.
//
// Header (2 B): count(u16). Per-entry record (5 B): laneHandle(i32) signal(u8).
public static class TlCodec
{
    public const int HeaderSize = 2;
    public const int EntrySize = 4 + 1;

    public readonly struct TlEntry
    {
        public TlEntry(int laneHandle, byte signal)
        {
            LaneHandle = laneHandle;
            Signal = signal;
        }

        public int LaneHandle { get; }
        public byte Signal { get; } // e.g. 'G'/'g'/'y'/'r'
    }

    public static int TlSize(int count) => HeaderSize + count * EntrySize;

    public static int WriteTl(Span<byte> dst, ReadOnlySpan<TlEntry> entries)
    {
        var size = TlSize(entries.Length);
        if (dst.Length < size)
        {
            throw new ArgumentException("destination too small for the TL frame.", nameof(dst));
        }

        BinaryPrimitives.WriteUInt16LittleEndian(dst.Slice(0, 2), (ushort)entries.Length);
        var o = HeaderSize;
        for (var i = 0; i < entries.Length; i++)
        {
            ref readonly var e = ref entries[i];
            BinaryPrimitives.WriteInt32LittleEndian(dst.Slice(o, 4), e.LaneHandle); o += 4;
            dst[o++] = e.Signal;
        }

        return size;
    }

    // Reads up to dst.Length entries; returns the number read (== min(header count, dst.Length)).
    public static int ReadTl(ReadOnlySpan<byte> src, Span<TlEntry> dst)
    {
        if (src.Length < HeaderSize)
        {
            throw new ArgumentException("TL frame shorter than header.", nameof(src));
        }

        var count = BinaryPrimitives.ReadUInt16LittleEndian(src.Slice(0, 2));
        var n = Math.Min(count, dst.Length);
        var o = HeaderSize;
        for (var i = 0; i < n; i++)
        {
            var laneHandle = BinaryPrimitives.ReadInt32LittleEndian(src.Slice(o, 4)); o += 4;
            var signal = src[o++];
            dst[i] = new TlEntry(laneHandle, signal);
        }

        return n;
    }
}
