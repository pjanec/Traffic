using CycloneDDS.Schema;

namespace Sim.Replication.Dds;

// SUMOSHARP-DEADRECKONING.md §4.3 — the DDS topic types for dead-reckoning replication over CycloneDDS.NET.
// The high-rate path is the CANONICAL opaque blob (DdsWireFrame): it carries the exact bytes
// SumoSharp.Replication.FrameCodec produces, so one codec/wire format serves DDS + TCP + UDP, and 10k+
// movers ride ONE batched sample (per chunk) instead of one keyed sample per mover (which DDS per-sample
// overhead makes untenable). Keyed by (Kind, ChunkIndex) so a tick's chunks are distinct instances.
//
// Publish/read is a thin host wrapper (see README): a DdsWriter<DdsWireFrame> written at <=10 Hz, a
// DdsReader taking the latest; QoS BEST_EFFORT/KEEP_LAST for state, TRANSIENT_LOCAL for lifecycle/geometry.
// Kept OUT of the [DdsTopic] struct: the CycloneDDS code generator enumerates public members of a topic as
// serializable fields, so a public const there is mis-generated as an instance field.
public static class DdsWire
{
    // 64 KiB caps a chunk at ~1360 vehicle records (48 B) / ~2000 crowd records (32 B); chunk beyond that.
    public const int MaxPayload = 64 * 1024;
}

[DdsTopic]
public unsafe partial struct DdsWireFrame
{
    [DdsKey, DdsId(0)] public byte Kind;        // FrameCodec.KindVehicle / KindCrowd (+ a geometry kind)
    [DdsKey, DdsId(1)] public int ChunkIndex;   // 0-based chunk within this tick
    [DdsId(2)] public uint Step;
    [DdsId(3)] public float Time;
    [DdsId(4)] public int ByteCount;            // valid prefix length of Payload
    [DdsId(5)] public fixed byte Payload[DdsWire.MaxPayload];

    // Blob copy helpers (pure; no DDS runtime dependency) so a publisher fills the sample straight from a
    // FrameCodec-written buffer, and a reader hands the valid prefix back to FrameCodec.
    public void SetPayload(byte kind, uint step, float time, ReadOnlySpan<byte> bytes, int chunkIndex = 0)
    {
        if (bytes.Length > DdsWire.MaxPayload)
        {
            throw new ArgumentException($"payload {bytes.Length} exceeds DdsWire.MaxPayload {DdsWire.MaxPayload}; chunk it.", nameof(bytes));
        }

        Kind = kind;
        ChunkIndex = chunkIndex;
        Step = step;
        Time = time;
        ByteCount = bytes.Length;
        fixed (byte* p = Payload)
        {
            bytes.CopyTo(new Span<byte>(p, DdsWire.MaxPayload));
        }
    }

    // Copy the valid payload prefix into `dst`; returns its length. Feed dst[..len] to FrameCodec.Read*.
    public readonly int ReadPayload(Span<byte> dst)
    {
        var n = ByteCount;
        fixed (byte* p = Payload)
        {
            new ReadOnlySpan<byte>(p, n).CopyTo(dst);
        }

        return n;
    }
}

// Low-rate, KEYED (by vehicle index) lifecycle/registry topic — DDS's "track many objects" sweet spot,
// affordable because it is low-rate. Spawn carries the immutable dims the renderer needs (the high-rate
// state topic then never repeats them); despawn tombstones the instance. TRANSIENT_LOCAL so late joiners
// get the current fleet.
[DdsTopic]
public partial struct DdsVehicleLifecycle
{
    [DdsKey, DdsId(0)] public uint Index;
    [DdsId(1)] public ushort Generation;
    [DdsId(2)] public byte Event;      // 0 = spawned, 1 = despawned
    [DdsId(3)] public int VTypeId;
    [DdsId(4)] public float Length;
    [DdsId(5)] public float Width;
}
