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
public readonly struct LifecycleRecord
{
    public LifecycleRecord(VehicleHandle handle, bool isSpawn, int vTypeId, float length, float width)
    {
        Handle = handle;
        IsSpawn = isSpawn;
        VTypeId = vTypeId;
        Length = length;
        Width = width;
    }

    public VehicleHandle Handle { get; }
    public bool IsSpawn { get; } // false = despawn
    public int VTypeId { get; }
    public float Length { get; }
    public float Width { get; }
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
    IReadOnlyDictionary<int, byte> TlStateByLane { get; }
    double? LatestVehicleSampleTime { get; }
    bool Connected { get; }
    void ResetVehicles();
    bool TryGetLatest(VehicleHandle handle, out TimestampedSample sample);
}
