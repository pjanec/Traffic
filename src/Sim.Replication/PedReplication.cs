using System.Buffers.Binary;
using Sim.Core;

namespace Sim.Replication;

// P3-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- the transport-neutral pedestrian
// replication surface, mirroring IReplication.cs's vehicle IReplicationSink/IReplicationSource pair
// exactly, so a caller coded against IPedReplicationSink/IPedReplicationSource never needs to know
// whether it holds DDS or an in-process binding. Stays FREE of Sim.Pedestrians: every type referenced
// here is already defined in this project (VehicleHandle from Sim.Core; PathArcRecord and
// PedFreeKinematicRecord from Records.cs; FrameCodec for the byte-level packing). The bridge that maps
// Sim.Pedestrians.Lod.PedEvent onto/from this surface lives in Sim.Pedestrians
// (PedReplicationPublisher.cs / PedReplicationReceiver.cs) -- the one place allowed to reference both
// projects.

// A ped spawn/despawn/DR-model-switch broadcast (durable, keyed by Handle) -- the transport-neutral
// analog of Sim.Pedestrians.Lod.DrSwitchEvent, plus the spawn/despawn cases the same lifecycle topic
// also carries per docs/PEDESTRIAN-DESIGN.md §7 ("Regime transitions = lifecycle events"). Kept
// deliberately small (CLAUDE.md P3-1 task note): DemoteToActivityTimeline also covers the ONE-TIME
// PathArc -> ActivityTimeline switch a lively ped's spawn emits (PedLodManager.AddPedLively) -- there is
// no separate "PromoteToActivityTimeline" kind because that switch is never itself a demotion FROM
// FreeKinematic; consumers reconstruct purely from the switch's `To` model (see
// Sim.Pedestrians.Lod.HeadlessIg.Apply(DrSwitchEvent), which never reads `From` either).
public enum PedLifecycleKind
{
    Spawn,
    Despawn,
    PromoteToFreeKinematic,
    DemoteToPathArc,
    DemoteToActivityTimeline,
}

public readonly struct PedLifecycleRecord
{
    public PedLifecycleRecord(VehicleHandle handle, PedLifecycleKind kind, double time)
    {
        Handle = handle;
        Kind = kind;
        Time = time;
    }

    public VehicleHandle Handle { get; }
    public PedLifecycleKind Kind { get; }
    public double Time { get; }
}

// Transport-neutral SEND contract for the pedestrian stream (mirrors IReplicationSink).
public interface IPedReplicationSink : IDisposable
{
    // Volatile, per-step: the high-power (FreeKinematic) population's positions/velocities, quantized
    // int32-cm on the wire via FrameCodec.WritePedFreeKinematicFrame.
    void PublishCrowdFrame(uint step, float time, ReadOnlySpan<PedFreeKinematicRecord> records);

    // Durable/transient-local, sent ONCE per PathArc leg (spawn + every demotion): serialized via
    // FrameCodec.WritePathArcFrame.
    void PublishPathArc(in PathArcRecord record);

    // Durable/transient-local, sent ONCE per ActivityTimeline leg: OPAQUE bytes (already encoded by
    // Sim.Pedestrians.Lod.ActivityTimelineWire.Encode) -- Sim.Replication never interprets them, only
    // tags them with the owning ped's handle.
    void PublishActivityTimeline(VehicleHandle handle, ReadOnlySpan<byte> timelineBytes);

    // Durable/keyed: spawn/despawn + DR-model-switch lifecycle events.
    void PublishPedLifecycle(in PedLifecycleRecord record);
}

// Transport-neutral RECEIVE contract (mirrors IReplicationSource's Pump-then-read discipline).
public interface IPedReplicationSource : IDisposable
{
    void Pump();

    // The most recently DECODED crowd frame only -- a receiver reconstructing a FreeKinematic ped needs
    // just the newest sample plus its own last-applied state (HeadlessIg.Reconstruct's linear
    // extrapolation), exactly like the vehicle stack's DR model, so older frames are not retained.
    uint LatestCrowdStep { get; }
    float LatestCrowdTime { get; }
    IReadOnlyList<PedFreeKinematicRecord> LatestCrowdFrame { get; }

    // Every PathArcRecord decoded so far, in arrival order (a ped demoting mid-run adds a second entry
    // for the same handle with a freshly re-routed path).
    IReadOnlyList<PathArcRecord> PathArcs { get; }

    // Every ActivityTimeline blob decoded so far, in arrival order -- still opaque bytes; only the
    // Sim.Pedestrians bridge (which owns ActivityTimelineWire) can decode them into an ActivityTimeline.
    IReadOnlyList<(VehicleHandle Handle, byte[] TimelineBytes)> ActivityTimelines { get; }

    // Every lifecycle record decoded so far, in arrival order.
    IReadOnlyList<PedLifecycleRecord> Lifecycles { get; }
}

// Byte-loopback InMemory binding (P3-1). UNLIKE InMemoryReplicationBus (the vehicle analog in
// InMemoryReplication.cs), which queues plain structs straight across with no codec at all (there is no
// wire to cross in-process), this bus genuinely serializes every publish call to a byte[] tagged with a
// topic and DESERIALIZES it back on Pump() -- so the hermetic round-trip test actually exercises the
// wire codecs (int32-cm quantization via FrameCodec, the ActivityTimeline double-precision format) end
// to end, proving server==IG survives serialization rather than merely an in-process struct hand-off.
public sealed class InMemoryPedReplicationBus
{
    private enum Topic
    {
        CrowdFrame,
        PathArc,
        ActivityTimeline,
        Lifecycle,
    }

    private readonly struct Entry
    {
        public Entry(Topic topic, byte[] bytes)
        {
            Topic = topic;
            Bytes = bytes;
        }

        public Topic Topic { get; }
        public byte[] Bytes { get; }
    }

    private readonly Queue<Entry> _queue = new();

    private uint _latestCrowdStep;
    private float _latestCrowdTime;
    private PedFreeKinematicRecord[] _latestCrowdFrame = Array.Empty<PedFreeKinematicRecord>();
    private readonly List<PathArcRecord> _pathArcs = new();
    private readonly List<(VehicleHandle Handle, byte[] TimelineBytes)> _timelines = new();
    private readonly List<PedLifecycleRecord> _lifecycles = new();

    public InMemoryPedReplicationBus()
    {
        Sink = new SinkImpl(this);
        Source = new SourceImpl(this);
    }

    public IPedReplicationSink Sink { get; }
    public IPedReplicationSource Source { get; }

    private sealed class SinkImpl : IPedReplicationSink
    {
        private readonly InMemoryPedReplicationBus _bus;
        public SinkImpl(InMemoryPedReplicationBus bus) => _bus = bus;

        public void PublishCrowdFrame(uint step, float time, ReadOnlySpan<PedFreeKinematicRecord> records)
        {
            var bytes = new byte[FrameCodec.PedFreeKinematicFrameSize(records.Length)];
            FrameCodec.WritePedFreeKinematicFrame(bytes, step, time, records);
            _bus._queue.Enqueue(new Entry(Topic.CrowdFrame, bytes));
        }

        public void PublishPathArc(in PathArcRecord record)
        {
            var recs = new[] { record };
            var bytes = new byte[FrameCodec.PathArcFrameSize(recs)];
            FrameCodec.WritePathArcFrame(bytes, step: 0, time: 0f, recs);
            _bus._queue.Enqueue(new Entry(Topic.PathArc, bytes));
        }

        public void PublishActivityTimeline(VehicleHandle handle, ReadOnlySpan<byte> timelineBytes)
        {
            // Sim.Replication has no dedicated ActivityTimeline wire header (the payload IS already the
            // ActivityTimelineWire-encoded blob) -- this bus only needs to know WHICH ped it belongs to,
            // so it prefixes the opaque payload with the handle (index + generation), mirroring the
            // handle-first layout every other wire record in FrameCodec.cs uses.
            var bytes = new byte[6 + timelineBytes.Length];
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(0, 4), handle.Index);
            BinaryPrimitives.WriteUInt16LittleEndian(bytes.AsSpan(4, 2), handle.Generation);
            timelineBytes.CopyTo(bytes.AsSpan(6));
            _bus._queue.Enqueue(new Entry(Topic.ActivityTimeline, bytes));
        }

        public void PublishPedLifecycle(in PedLifecycleRecord record)
        {
            // index(4) + generation(2) + kind(1) + time(8), little-endian -- mirrors the vehicle stack's
            // LifecycleRecord in spirit (IReplication.cs), which similarly has no FrameCodec entry of its
            // own since it is a low-rate keyed event, not a per-frame mover record.
            var bytes = new byte[15];
            BinaryPrimitives.WriteUInt32LittleEndian(bytes.AsSpan(0, 4), record.Handle.Index);
            BinaryPrimitives.WriteUInt16LittleEndian(bytes.AsSpan(4, 2), record.Handle.Generation);
            bytes[6] = (byte)record.Kind;
            WriteF64(bytes.AsSpan(7, 8), record.Time);
            _bus._queue.Enqueue(new Entry(Topic.Lifecycle, bytes));
        }

        public void Dispose()
        {
        }
    }

    private sealed class SourceImpl : IPedReplicationSource
    {
        private readonly InMemoryPedReplicationBus _bus;
        public SourceImpl(InMemoryPedReplicationBus bus) => _bus = bus;

        public void Pump() => _bus.PumpCore();

        public uint LatestCrowdStep => _bus._latestCrowdStep;
        public float LatestCrowdTime => _bus._latestCrowdTime;
        public IReadOnlyList<PedFreeKinematicRecord> LatestCrowdFrame => _bus._latestCrowdFrame;
        public IReadOnlyList<PathArcRecord> PathArcs => _bus._pathArcs;
        public IReadOnlyList<(VehicleHandle Handle, byte[] TimelineBytes)> ActivityTimelines => _bus._timelines;
        public IReadOnlyList<PedLifecycleRecord> Lifecycles => _bus._lifecycles;

        public void Dispose()
        {
        }
    }

    private void PumpCore()
    {
        while (_queue.Count > 0)
        {
            var e = _queue.Dequeue();
            switch (e.Topic)
            {
                case Topic.CrowdFrame:
                    var header = FrameCodec.ReadHeader(e.Bytes);
                    var frame = new PedFreeKinematicRecord[header.Count];
                    FrameCodec.ReadPedFreeKinematicFrame(e.Bytes, frame);
                    _latestCrowdStep = header.Step;
                    _latestCrowdTime = header.Time;
                    _latestCrowdFrame = frame;
                    break;

                case Topic.PathArc:
                    var recs = FrameCodec.ReadPathArcFrame(e.Bytes);
                    _pathArcs.AddRange(recs);
                    break;

                case Topic.ActivityTimeline:
                    var index = BinaryPrimitives.ReadUInt32LittleEndian(e.Bytes.AsSpan(0, 4));
                    var gen = BinaryPrimitives.ReadUInt16LittleEndian(e.Bytes.AsSpan(4, 2));
                    var payload = e.Bytes.AsSpan(6).ToArray();
                    _timelines.Add((new VehicleHandle(index, gen), payload));
                    break;

                case Topic.Lifecycle:
                    var lcIndex = BinaryPrimitives.ReadUInt32LittleEndian(e.Bytes.AsSpan(0, 4));
                    var lcGen = BinaryPrimitives.ReadUInt16LittleEndian(e.Bytes.AsSpan(4, 2));
                    var kind = (PedLifecycleKind)e.Bytes[6];
                    var time = ReadF64(e.Bytes.AsSpan(7, 8));
                    _lifecycles.Add(new PedLifecycleRecord(new VehicleHandle(lcIndex, lcGen), kind, time));
                    break;
            }
        }
    }

    // double <-> LE bytes via long bits (BinaryPrimitives.Write/ReadDoubleLittleEndian is net5+, absent
    // on netstandard2.1 -- same reasoning as FrameCodec's WriteF32/ReadF32).
    private static void WriteF64(Span<byte> dst, double value) =>
        BinaryPrimitives.WriteInt64LittleEndian(dst, BitConverter.DoubleToInt64Bits(value));

    private static double ReadF64(ReadOnlySpan<byte> src) =>
        BitConverter.Int64BitsToDouble(BinaryPrimitives.ReadInt64LittleEndian(src));
}
