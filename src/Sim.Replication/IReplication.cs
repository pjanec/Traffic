using Sim.Core;

namespace Sim.Replication;

// docs/SUMOSHARP-PACKAGING-DESIGN.md P1 (D8) — the transport-neutral replication contract. DDS is one
// binding among several: DdsPublisher/DdsSubscriber (Sim.Viewer.Core) implement these two interfaces
// today, and InMemoryReplicationBus (this project) implements them for a same-process, non-DDS binding
// used by the hermetic test loop. The interfaces reference ONLY data-model types already defined in this
// project (VehicleRecord, GeometryCodec.LaneGeo, TlCodec.TlEntry, TimestampedSample,
// IVehicleSampleHistory, VehicleHandle) -- never a DDS type -- so a consumer coded against
// IReplicationSink/IReplicationSource never needs to know CycloneDDS exists.

// A vehicle spawn/despawn + physical dims announcement (durable, once per vehicle) -- the transport-
// neutral analog of DdsVehicleLifecycle (Sim.Replication.Dds), minus the wire-specific Index/Generation
// split (VehicleHandle already carries both).
//
// `Name` (docs/LIVE-CITY-VIEWERS-DESIGN.md §3.1, -TASKS.md E2): the real SUMO vehicle id, ADDITIVE and
// populated on SPAWN only (empty string on despawn -- a despawn tombstone needs no identity, the consumer
// already has it from the matching spawn). Travels ONCE per entity on this lifecycle event, never on the
// per-frame VehicleRecord/DdsWireFrame -- at 100k entities the hot per-step path pays nothing extra for it.
public readonly struct LifecycleRecord
{
    public LifecycleRecord(VehicleHandle handle, bool isSpawn, int vTypeId, float length, float width, string name = "")
    {
        Handle = handle;
        IsSpawn = isSpawn;
        VTypeId = vTypeId;
        Length = length;
        Width = width;
        Name = name ?? "";
    }

    public VehicleHandle Handle { get; }
    public bool IsSpawn { get; } // false = despawn
    public int VTypeId { get; }
    public float Length { get; }
    public float Width { get; }
    // The real SUMO vehicle id, set on spawn; "" on despawn (see class remark).
    public string Name { get; }
}

// Transport-neutral SEND contract. DDS is one binding; in-memory (InMemoryReplicationBus) is another.
public interface IReplicationSink : IDisposable
{
    void PublishGeometry(IReadOnlyList<GeometryCodec.LaneGeo> lanes);                     // durable, once
    void PublishLifecycle(in LifecycleRecord record);                                     // durable, per spawn/despawn
    void PublishFrame(uint step, double time, ReadOnlySpan<VehicleRecord> movers);         // volatile, per frame
    void PublishTrafficLights(uint step, double time, IReadOnlyList<TlCodec.TlEntry> lights); // low-rate
}

// Transport-neutral RECEIVE contract. Mirrors DdsSubscriber's existing public surface exactly.
public interface IReplicationSource : IDisposable
{
    void Pump();
    IReadOnlyDictionary<int, GeometryCodec.LaneGeo> Geometry { get; }
    bool GeometryComplete { get; }
    IReadOnlyDictionary<VehicleHandle, IVehicleSampleHistory> History { get; }
    IReadOnlyDictionary<VehicleHandle, (float Length, float Width)> Dims { get; }
    // docs/LIVE-CITY-VIEWERS-DESIGN.md §3.1, -TASKS.md E2: the handle->SUMO-id table accumulated from
    // LifecycleRecord.Name on spawn (removed on despawn, mirroring Dims). Built purely from the lifecycle
    // stream, never the per-frame record, so every IReplicationSource binding (in-memory, DDS, file replay)
    // gets it "for free" the same way Dims already is.
    IReadOnlyDictionary<VehicleHandle, string> Names { get; }
    IReadOnlyDictionary<int, byte> TlStateByLane { get; }
    double? LatestVehicleSampleTime { get; }
    bool Connected { get; }
    void ResetVehicles();
    bool TryGetLatest(VehicleHandle handle, out TimestampedSample sample);
}
