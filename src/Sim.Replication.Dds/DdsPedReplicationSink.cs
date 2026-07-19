using CycloneDDS.Runtime;
using Sim.Core;
using Sim.Replication;

namespace Sim.Replication.Dds;

// docs/PEDESTRIAN-DDS-TRANSPORT-DESIGN.md §4/§5 -- the live CycloneDDS binding of IPedReplicationSink, the
// ped mirror of DdsReplicationSink (the vehicle SEND side). Encode + DDS-write ONLY: every FrameCodec /
// ActivityTimelineWire byte here is byte-identical to InMemoryPedReplicationBus.SinkImpl -- the DDS layer
// transports the exact bytes the InMemory bus round-trips, adding no new codec or quantization.
//
// QoS (docs §4): the crowd frame is perishable per-step state -> VolatileLatest (BEST_EFFORT); the three
// durable per-ped topics (patharc, activity, lifecycle) -> DurableLatest (RELIABLE/TRANSIENT_LOCAL) so a
// late-joining IG gets the latest leg / DR-model per ped. A reader MUST use the matching profile (see
// DdsPedReplicationSource) -- a RELIABLE reader will not match a BEST_EFFORT writer.
//
// CRITICAL (see README.md): every writer is constructed with an EXPLICIT DdsTopicNames.Ped* string.
public sealed class DdsPedReplicationSink : IPedReplicationSink
{
    // Durable per-ped topics keep a small history depth so a rare burst of >1 leg per ped between a
    // subscriber's Pump() calls is not collapsed to a single sample (docs §4 "accumulation semantics").
    private const int DurableDepth = 16;

    private readonly DdsWriter<DdsWireFrame> _crowdWriter;
    private readonly DdsWriter<DdsPedLeg> _pathArcWriter;
    private readonly DdsWriter<DdsPedLeg> _activityWriter;
    private readonly DdsWriter<DdsPedLifecycle> _lifecycleWriter;

    private byte[] _crowdScratch = new byte[DdsWire.MaxPayload];
    private byte[] _pathArcScratch = new byte[DdsWire.MaxPayload];

    // Cumulative bytes written per topic (valid payload prefixes only) -- a pure diagnostic for the
    // live-bandwidth readout (docs §6, P6-1 SC3), never read on the reconstruction path.
    public long CrowdBytesPublished { get; private set; }
    public long PathArcBytesPublished { get; private set; }
    public long ActivityBytesPublished { get; private set; }
    public long LifecycleBytesPublished { get; private set; }

    public DdsPedReplicationSink(DdsParticipant participant)
    {
        _crowdWriter = new DdsWriter<DdsWireFrame>(participant, DdsTopicNames.PedCrowd, DdsQos.VolatileLatest());
        _pathArcWriter = new DdsWriter<DdsPedLeg>(participant, DdsTopicNames.PedPathArc, DdsQos.DurableLatest(DurableDepth));
        _activityWriter = new DdsWriter<DdsPedLeg>(participant, DdsTopicNames.PedActivity, DdsQos.DurableLatest(DurableDepth));
        _lifecycleWriter = new DdsWriter<DdsPedLifecycle>(participant, DdsTopicNames.PedLifecycle, DdsQos.DurableLatest(DurableDepth));
    }

    // High-rate crowd frame: chunk by the shared byte budget (3640 peds / 64 KiB chunk at 18 B/record) and
    // DDS-write each chunk as a DdsWireFrame (Kind = KindPedFreeKinematic). Same chunk planning as the
    // vehicle sink; same FrameCodec bytes as the InMemory bus.
    public void PublishCrowdFrame(uint step, float time, ReadOnlySpan<PedFreeKinematicRecord> records)
    {
        if (records.Length == 0)
        {
            return;
        }

        var maxPerChunk = Math.Max(1, FrameChunker.MaxRecordsForPayload(DdsWire.MaxPayload, FrameCodec.PedFreeKinematicRecordSize));
        var totalChunks = FrameChunker.ChunkCount(records.Length, maxPerChunk);
        for (var c = 0; c < totalChunks; c++)
        {
            var (start, count) = FrameChunker.ChunkRange(c, records.Length, maxPerChunk);
            var chunk = records.Slice(start, count);
            var size = FrameCodec.WritePedFreeKinematicFrame(_crowdScratch, step, time, chunk);
            var frame = new DdsWireFrame();
            frame.SetPayload(FrameCodec.KindPedFreeKinematic, step, time, _crowdScratch.AsSpan(0, size), c);
            _crowdWriter.Write(frame);
            CrowdBytesPublished += size;
        }
    }

    // Durable, once per PathArc leg: serialize the single record via FrameCodec (same bytes as the InMemory
    // bus) and DDS-write it keyed by the ped handle Index (latest leg per ped for late joiners).
    public void PublishPathArc(in PathArcRecord record)
    {
        var recs = new[] { record };
        var size = FrameCodec.WritePathArcFrame(_pathArcScratch, step: 0, time: 0f, recs);
        var leg = new DdsPedLeg();
        leg.SetPayload(record.Handle.Index, record.Handle.Generation, _pathArcScratch.AsSpan(0, size));
        _pathArcWriter.Write(leg);
        PathArcBytesPublished += size;
    }

    // Durable, once per ActivityTimeline leg: the payload is ALREADY the opaque ActivityTimelineWire blob
    // (Sim.Replication never interprets it) -- just tag it with the owning ped's handle and DDS-write.
    public void PublishActivityTimeline(VehicleHandle handle, ReadOnlySpan<byte> timelineBytes)
    {
        var leg = new DdsPedLeg();
        leg.SetPayload(handle.Index, handle.Generation, timelineBytes);
        _activityWriter.Write(leg);
        ActivityBytesPublished += timelineBytes.Length;
    }

    // Durable, keyed: spawn/despawn + DR-model-switch lifecycle event, the ped analog of the vehicle
    // lifecycle write.
    public void PublishPedLifecycle(in PedLifecycleRecord record)
    {
        var lc = new DdsPedLifecycle
        {
            Index = record.Handle.Index,
            Generation = record.Handle.Generation,
            Kind = (byte)record.Kind,
            Time = record.Time,
        };
        _lifecycleWriter.Write(lc);
        LifecycleBytesPublished += 15; // index(4)+gen(2)+kind(1)+time(8), matching the InMemory bus record
    }

    public void Dispose()
    {
        _crowdWriter.Dispose();
        _pathArcWriter.Dispose();
        _activityWriter.Dispose();
        _lifecycleWriter.Dispose();
    }
}
