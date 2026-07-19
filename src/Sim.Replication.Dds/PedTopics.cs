using CycloneDDS.Schema;

namespace Sim.Replication.Dds;

// docs/PEDESTRIAN-DDS-TRANSPORT-DESIGN.md §4/§5 -- the NEW DDS topic types for the live pedestrian wire, the
// ped mirror of the shipped vehicle binding (DdsTopics.cs / GeometryTlTopics.cs). The high-rate crowd frame
// REUSES the canonical DdsWireFrame (Kind = FrameCodec.KindPedFreeKinematic) -- it is exactly the same
// "opaque FrameCodec blob" pattern -- so only the two DURABLE, per-ped, keyed topics need new types here:
//
//  * DdsPedLeg      -- one keyed opaque blob per ped, sent once per leg. Carries EITHER the FrameCodec
//                      PathArc frame bytes (sumo/ped-patharc) OR the ActivityTimelineWire blob
//                      (sumo/ped-activity); the two topics share this one type (the "one struct, several
//                      topic names" pattern DdsWireFrame itself uses). Keyed by the ped handle Index so a
//                      late-joining IG gets the latest leg per ped under TRANSIENT_LOCAL durability.
//  * DdsPedLifecycle -- the low-rate keyed spawn/despawn + DR-model-switch event, the ped analog of
//                      DdsVehicleLifecycle.
//
// CRITICAL (see README.md): every writer/reader on these topics MUST be constructed with an explicit
// DdsTopicNames.Ped* string (the [DdsTopic] attribute's own name is NOT usable through CycloneDDS.NET
// 0.3.2's generator), and a writer/reader pair MUST use the identical string.

// Durable, keyed, per-ped opaque leg blob (PathArc frame OR ActivityTimeline blob). Same
// SetPayload/ReadPayload bridge as DdsNetworkGeometry; keyed by the ped handle Index (the Generation is
// carried too so the source can present a full VehicleHandle, though the current receiver reads only Index).
[DdsTopic]
public unsafe partial struct DdsPedLeg
{
    [DdsKey, DdsId(0)] public uint Index;      // ped handle index -- the DDS instance key (latest leg per ped)
    [DdsId(1)] public ushort Generation;       // ped handle generation (faithful; receiver currently reads only Index)
    [DdsId(2)] public int ByteCount;           // valid prefix length of Payload
    [DdsId(3)] public fixed byte Payload[DdsWire.MaxPayload];

    public void SetPayload(uint index, ushort generation, ReadOnlySpan<byte> bytes)
    {
        if (bytes.Length > DdsWire.MaxPayload)
        {
            throw new ArgumentException($"payload {bytes.Length} exceeds DdsWire.MaxPayload {DdsWire.MaxPayload}; chunk it.", nameof(bytes));
        }

        Index = index;
        Generation = generation;
        ByteCount = bytes.Length;
        fixed (byte* p = Payload)
        {
            bytes.CopyTo(new Span<byte>(p, DdsWire.MaxPayload));
        }
    }

    // Copy the valid payload prefix into `dst`; returns its length. Feed dst[..len] to the matching decoder
    // (FrameCodec.ReadPathArcFrame for a patharc leg; ActivityTimelineWire.Decode -- in Sim.Pedestrians --
    // for an activity leg).
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

// Low-rate, KEYED (by ped index) lifecycle topic -- spawn/despawn + DR-model switch, the ped analog of
// DdsVehicleLifecycle. TRANSIENT_LOCAL so a late-joining IG gets each known ped's latest DR-model state.
// `Kind` is the byte-backed Sim.Replication.PedLifecycleKind; `Time` is the event's sim time.
[DdsTopic]
public partial struct DdsPedLifecycle
{
    [DdsKey, DdsId(0)] public uint Index;
    [DdsId(1)] public ushort Generation;
    [DdsId(2)] public byte Kind;       // Sim.Replication.PedLifecycleKind
    [DdsId(3)] public double Time;
}
