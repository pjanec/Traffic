using CycloneDDS.Runtime.Interop;

namespace Sim.Replication.Dds;

// docs/SUMOSHARP-NATIVE-VIEWER.md P3 ("remote mode + QoS") -- native QoS handles for the two profiles this
// track uses, built via CycloneDDS.Runtime.Interop.DdsApi (reflected off CycloneDDS.NET 0.3.2's managed
// assemblies: dds_create_qos / dds_qset_reliability / dds_qset_durability / dds_qset_history are public
// static P/Invoke wrappers, and DdsWriter<T>/DdsReader<T> already take a `qos` IntPtr as their 3rd ctor
// argument -- see Sim.Replication.Dds/README.md's recommended-QoS line). This is the DURABLE-QOS path, not
// the periodic-republish fallback: verified with a throwaway two-PROCESS probe (a genuinely separate OS
// process started well after a DurableLatest writer had already written its samples and gone idle still
// received the writer's last sample per key), so TRANSIENT_LOCAL late-join semantics are real cross-process
// in this sandbox, not just an intra-process replay artifact.
//
// - DurableLatest: RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(depth) -- geometry (chunk-keyed) + lifecycle
//   (vehicle-handle-keyed), so a remote reader that starts AFTER the writer has already published gets the
//   last sample per key (the whole network's geometry chunks; each known vehicle's latest spawn/despawn)
//   without waiting for a fresh publish. DdsSubscriber's lifecycle handling is idempotent to a replayed
//   stale despawn (removing an already-absent key is a no-op), so per-key KEEP_LAST(1) is sufficient even
//   though a vehicle's full spawn/despawn history isn't kept.
// - VolatileLatest: BEST_EFFORT + KEEP_LAST(depth) -- the high-rate Vehicles state topic and the low-rate Tl
//   topic (both already re-published periodically by DdsPublisher, so a late joiner just waits for the next
//   live sample rather than paying reliable-retransmit overhead for perishable state).
//
// A writer and its matching reader on the SAME topic MUST use the SAME profile: a RELIABLE reader will not
// match a BEST_EFFORT writer (DDS QoS-compatibility rules), so DdsPublisher and DdsSubscriber below build
// identical profiles per topic name, and LoopbackSelfTest (same-process publisher+subscriber) gets the same
// treatment automatically since both go through this class.
//
// Qos handles are intentionally never dds_delete_qos'd: dds_create_writer/dds_create_reader (invoked inside
// the DdsWriter<T>/DdsReader<T> ctor) copies the QoS content into the entity at creation time per the DDS
// spec, so the handle could be freed right after construction -- but there are at most 8 of these for the
// life of a process (4 writers + 4 readers), so the trivial one-time native alloc is left alive for process
// lifetime rather than adding a use-after-free risk for no measurable benefit.
// Public (not internal): consumers (DdsSubscriber/DdsPublisher/DdsCommandWriter/DdsCommandReader/
// DdsStatusWriter/DdsStatusReader) live in OTHER assemblies (Sim.Viewer.Core, Sim.Viewer.Raylib) after
// the P3.3 packaging split -- internal visibility would not cross the assembly boundary.
public static class DdsQos
{
    public static IntPtr DurableLatest(int depth = 1)
    {
        var qos = DdsApi.dds_create_qos();
        DdsApi.dds_qset_reliability(qos, DdsApi.DDS_RELIABILITY_RELIABLE, DdsApi.DDS_INFINITY);
        DdsApi.dds_qset_durability(qos, DdsApi.DDS_DURABILITY_TRANSIENT_LOCAL);
        DdsApi.dds_qset_history(qos, DdsApi.DDS_HISTORY_KEEP_LAST, depth);
        return qos;
    }

    public static IntPtr VolatileLatest(int depth = 1)
    {
        var qos = DdsApi.dds_create_qos();
        DdsApi.dds_qset_reliability(qos, DdsApi.DDS_RELIABILITY_BEST_EFFORT, 0);
        DdsApi.dds_qset_durability(qos, DdsApi.DDS_DURABILITY_VOLATILE);
        DdsApi.dds_qset_history(qos, DdsApi.DDS_HISTORY_KEEP_LAST, depth);
        return qos;
    }

    // NB: the remote-command channel's QoS (reliable + transient-local + deep keep-last) is declared on the
    // topic TYPE via [DdsQos] on DdsViewerCommand (CycloneDDS.NET code-first DSL), applied automatically -- so
    // it needs no hand-built handle here.
}
