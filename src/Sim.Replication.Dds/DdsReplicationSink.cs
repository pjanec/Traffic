using CycloneDDS.Runtime;
using CycloneDDS.Schema;
using Sim.Replication;

namespace Sim.Replication.Dds;

// docs/DEMO-CITY3D-DESIGN.md "DRY rewire (parity-neutral)": the DDS binding's own IReplicationSink,
// moved here verbatim from src/Sim.Viewer.Core/DdsPublisher.cs (which conflated this DDS-writing half
// with the EngineHost-snapshot-reading half). This class does ONLY encode + DDS-write: chunk-planning,
// codec calls, and writer bytes are byte-identical to DdsPublisher's pre-rewire implementation --
// nothing about topics, QoS, chunk sizes, or payload layout changed by this move. The snapshot -> wire
// record TRANSLATION now lives in Sim.Host.ReplicationPublisher, which drives this sink (or any other
// IReplicationSink) via the interface below.
//
// CRITICAL (see README.md): every writer below is constructed with an EXPLICIT DdsTopicNames.* string --
// the [DdsTopic] attribute's own name is NOT usable (the 1-arg ctor throws ArgumentNullException), and a
// DdsSubscriber reading the SAME topic must pass the identical string.
//
// docs/SUMOSHARP-NATIVE-VIEWER.md P3 QoS: geometry + lifecycle are RELIABLE/TRANSIENT_LOCAL (durable --
// a late-joining remote reader gets the last-published network + fleet); Vehicles + Tl are
// BEST_EFFORT/VOLATILE (perishable state, already re-published periodically) -- see DdsQos's doc
// comment for the two-process durability verification behind this split.
public sealed class DdsReplicationSink : IDisposable, IReplicationSink
{
    private readonly DdsWriter<DdsWireFrame> _vehicleWriter;
    private readonly DdsWriter<DdsNetworkGeometry> _geometryWriter;
    private readonly DdsWriter<DdsVehicleLifecycle> _lifecycleWriter;
    private readonly DdsWriter<DdsTlState> _tlWriter;

    private byte[] _vehicleScratch = new byte[DdsWire.MaxPayload];
    private byte[] _tlScratch = new byte[DdsWire.MaxPayload];

    public DdsReplicationSink(DdsParticipant participant)
    {
        _vehicleWriter = new DdsWriter<DdsWireFrame>(participant, DdsTopicNames.Vehicles, DdsQos.VolatileLatest());
        _geometryWriter = new DdsWriter<DdsNetworkGeometry>(participant, DdsTopicNames.Geometry, DdsQos.DurableLatest());
        _lifecycleWriter = new DdsWriter<DdsVehicleLifecycle>(participant, DdsTopicNames.Lifecycle, DdsQos.DurableLatest());
        _tlWriter = new DdsWriter<DdsTlState>(participant, DdsTopicNames.Tl, DdsQos.VolatileLatest());
    }

    // IReplicationSink.PublishGeometry — chunk + DDS-write the network's static lane geometry. Same
    // chunk-planning and payload bytes as DdsPublisher's pre-rewire implementation.
    public void PublishGeometry(IReadOnlyList<GeometryCodec.LaneGeo> lanes)
    {
        var chunks = GeometryCodec.PlanChunks(lanes, DdsWire.MaxPayload);
        var laneArray = lanes as GeometryCodec.LaneGeo[] ?? ToArray(lanes);
        var laneSpan = laneArray.AsSpan();
        var scratch = new byte[DdsWire.MaxPayload];
        for (var c = 0; c < chunks.Count; c++)
        {
            var (start, count) = chunks[c];
            var size = GeometryCodec.WriteGeometry(scratch, laneSpan.Slice(start, count));
            var frame = new DdsNetworkGeometry();
            frame.SetPayload(c, chunks.Count, step: 0, time: 0f, scratch.AsSpan(0, size));
            _geometryWriter.Write(frame);
        }
    }

    private static GeometryCodec.LaneGeo[] ToArray(IReadOnlyList<GeometryCodec.LaneGeo> lanes)
    {
        var arr = new GeometryCodec.LaneGeo[lanes.Count];
        for (var i = 0; i < lanes.Count; i++)
        {
            arr[i] = lanes[i];
        }

        return arr;
    }

    // IReplicationSink.PublishLifecycle — encode + DDS-write a single spawn/despawn record. Same
    // DdsVehicleLifecycle fields/Event encoding as DdsPublisher's pre-rewire implementation.
    public void PublishLifecycle(in LifecycleRecord record)
    {
        var rec = new DdsVehicleLifecycle
        {
            Index = record.Handle.Index,
            Generation = record.Handle.Generation,
            Event = (byte)(record.IsSpawn ? 0 : 1),
            VTypeId = record.VTypeId,
            Length = record.Length,
            Width = record.Width,
            Name = ToFixedName(record.Name),
        };
        _lifecycleWriter.Write(rec);
    }

    // FixedString64's ctor THROWS if the UTF-8 encoding exceeds its 64-byte capacity -- defensively clamp
    // a pathologically long SUMO id (well beyond any real route/flow-generated id) rather than let one
    // vehicle's lifecycle announcement crash the whole publish loop. Every id seen in this repo's
    // scenarios/tests fits comfortably; this is a belt-and-braces guard, not the expected path.
    private static FixedString64 ToFixedName(string? name)
    {
        var s = name ?? string.Empty;
        if (FixedString64.TryFrom(s, out var fs))
        {
            return fs;
        }

        // Trim from the front (chars, not bytes -- conservative for multi-byte UTF-8) until it fits.
        var trimmed = s.Length > 63 ? s.Substring(0, 63) : s;
        while (trimmed.Length > 0 && !FixedString64.TryFrom(trimmed, out fs))
        {
            trimmed = trimmed.Substring(0, trimmed.Length - 1);
        }

        return fs;
    }

    // IReplicationSink.PublishFrame — chunk + DDS-write the selected movers. Same chunk-planning and
    // payload bytes as DdsPublisher's pre-rewire implementation.
    public void PublishFrame(uint step, double time, ReadOnlySpan<VehicleRecord> movers)
    {
        if (movers.Length == 0)
        {
            return;
        }

        var maxPerChunk = Math.Max(1, FrameChunker.MaxRecordsForPayload(DdsWire.MaxPayload, FrameCodec.VehicleRecordSize));
        var totalChunks = FrameChunker.ChunkCount(movers.Length, maxPerChunk);
        for (var c = 0; c < totalChunks; c++)
        {
            var (start, count) = FrameChunker.ChunkRange(c, movers.Length, maxPerChunk);
            var chunk = movers.Slice(start, count);
            var size = FrameCodec.WriteVehicleFrame(_vehicleScratch, step, (float)time, chunk);
            var frame = new DdsWireFrame();
            frame.SetPayload(FrameCodec.KindVehicle, step, (float)time, _vehicleScratch.AsSpan(0, size), c);
            _vehicleWriter.Write(frame);
        }
    }

    // IReplicationSink.PublishTrafficLights — encode + DDS-write the low-rate TL state packet. Same bytes
    // and cadence as DdsPublisher's pre-rewire implementation.
    public void PublishTrafficLights(uint step, double time, IReadOnlyList<TlCodec.TlEntry> lights)
    {
        if (lights.Count == 0)
        {
            return;
        }

        var entries = lights as TlCodec.TlEntry[] ?? ToArray(lights);
        var size = TlCodec.WriteTl(_tlScratch, entries);
        var frame = new DdsTlState();
        frame.SetPayload(step, (float)time, _tlScratch.AsSpan(0, size));
        _tlWriter.Write(frame);
    }

    private static TlCodec.TlEntry[] ToArray(IReadOnlyList<TlCodec.TlEntry> lights)
    {
        var arr = new TlCodec.TlEntry[lights.Count];
        for (var i = 0; i < lights.Count; i++)
        {
            arr[i] = lights[i];
        }

        return arr;
    }

    public void Dispose()
    {
        _vehicleWriter.Dispose();
        _geometryWriter.Dispose();
        _lifecycleWriter.Dispose();
        _tlWriter.Dispose();
    }
}
