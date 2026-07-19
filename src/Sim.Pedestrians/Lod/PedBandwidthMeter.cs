using Sim.Replication;

namespace Sim.Pedestrians.Lod;

// P3-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- a transport-neutral accountant of the
// SINGLE multicast stream's byte rate. This is what proves the §7 bandwidth argument empirically rather
// than by estimate: it tallies the REAL bytes a publisher actually put on the wire, per topic, using the
// real FrameCodec record/frame sizes (not an approximation of them), and reports Mbit/s so a test can
// assert the measured figure stays under the ~500 Mbit/s budget.
//
// Deliberately frame/call-granular, not per-record: one Record call per PublishCrowdFrame/PublishPathArc/
// PublishActivityTimeline/PublishPedLifecycle/heartbeat call (mirroring PedReplicationPublisher's actual
// wire calls one-for-one), so the sample list stays O(steps * topics), never O(peds), even at a 100k-ped
// scale -- it is the wire TRAFFIC being measured, not the population.
public enum PedBandwidthTopic
{
    CrowdFrame,        // per-step FreeKinematic samples (FrameCodec.PedFreeKinematicFrameSize)
    PathArc,           // sent once per PathArc leg (FrameCodec.PathArcRecordSize + header)
    ActivityTimeline,  // sent once per timeline leg (opaque blob + the InMemory bus's 6 B handle prefix)
    Lifecycle,         // spawn/despawn/DR-switch (InMemoryPedReplicationBus's 15 B lifecycle record)
    Heartbeat,         // low-power liveness -- near-zero, nominal cost
}

public sealed class PedBandwidthMeter
{
    // Mirrors InMemoryPedReplicationBus's actual on-wire encodings (PedReplication.cs) so this meter's
    // byte accounting matches what the hermetic bus really serializes, not just FrameCodec's frame sizes.
    private const int ActivityTimelineHandlePrefixBytes = 6; // index(4) + generation(2)
    private const int LifecycleRecordBytes = 15;             // index(4) + generation(2) + kind(1) + time(8)
    private const int HeartbeatRecordBytes = 1;               // liveness-only: nominal, near-zero cost

    private readonly struct Entry
    {
        public Entry(PedBandwidthTopic topic, double time, long bytes)
        {
            Topic = topic;
            Time = time;
            Bytes = bytes;
        }

        public PedBandwidthTopic Topic { get; }
        public double Time { get; }
        public long Bytes { get; }
    }

    private readonly List<Entry> _entries = new();
    private readonly Dictionary<PedBandwidthTopic, long> _totalBytes = new();
    private readonly Dictionary<PedBandwidthTopic, long> _totalCount = new();
    private double _lastTime = double.NaN;

    // --- Recorders (real record/frame sizes -- FrameCodec is the single source of truth) ---

    public void RecordCrowdFrame(double time, int recordCount) =>
        Record(PedBandwidthTopic.CrowdFrame, time, FrameCodec.PedFreeKinematicFrameSize(recordCount));

    public void RecordPathArc(double time, int pointCount) =>
        Record(PedBandwidthTopic.PathArc, time, FrameCodec.HeaderSize + FrameCodec.PathArcRecordSize(pointCount));

    public void RecordActivityTimeline(double time, int payloadBytes) =>
        Record(PedBandwidthTopic.ActivityTimeline, time, ActivityTimelineHandlePrefixBytes + payloadBytes);

    public void RecordLifecycle(double time) =>
        Record(PedBandwidthTopic.Lifecycle, time, LifecycleRecordBytes);

    public void RecordHeartbeat(double time) =>
        Record(PedBandwidthTopic.Heartbeat, time, HeartbeatRecordBytes);

    public void Record(PedBandwidthTopic topic, double time, int bytes)
    {
        _entries.Add(new Entry(topic, time, bytes));
        _totalBytes[topic] = _totalBytes.GetValueOrDefault(topic) + bytes;
        _totalCount[topic] = _totalCount.GetValueOrDefault(topic) + 1;
        if (double.IsNaN(_lastTime) || time > _lastTime)
        {
            _lastTime = time;
        }
    }

    // --- Queries ---

    public long TotalBytes(PedBandwidthTopic topic) => _totalBytes.GetValueOrDefault(topic);
    public long TotalCount(PedBandwidthTopic topic) => _totalCount.GetValueOrDefault(topic);

    public long TotalBytesAllTopics
    {
        get
        {
            long total = 0;
            foreach (var kv in _totalBytes)
            {
                total += kv.Value;
            }

            return total;
        }
    }

    // Whole-run or rolling average Mbit/s across ALL topics (the single multicast stream): sums bytes
    // whose Time falls within (latestTime - windowSeconds, latestTime], divided by windowSeconds. Pass
    // the whole run's elapsed seconds for a whole-run average, or a short window for a rolling figure.
    public double MbitPerSecond(double windowSeconds) => MbitPerSecondCore(windowSeconds, topic: null);

    // Same, restricted to one topic -- the per-topic breakdown the low-power invariant / target-mix
    // tests assert against.
    public double MbitPerSecondForTopic(PedBandwidthTopic topic, double windowSeconds) =>
        MbitPerSecondCore(windowSeconds, topic);

    private double MbitPerSecondCore(double windowSeconds, PedBandwidthTopic? topic)
    {
        if (_entries.Count == 0 || windowSeconds <= 0.0)
        {
            return 0.0;
        }

        var since = _lastTime - windowSeconds;
        long bytes = 0;
        foreach (var e in _entries)
        {
            if (e.Time > since && (topic is null || e.Topic == topic.Value))
            {
                bytes += e.Bytes;
            }
        }

        return BytesToMbitPerSecond(bytes, windowSeconds);
    }

    // The largest single-step projected rate seen so far: groups entries by exact Time (every record a
    // publisher emits for one step is stamped with that step's own time, matching
    // PedReplicationPublisher's batching discipline) and reports max(bucketBytes) converted to Mbit/s
    // over `stepSeconds`. This is the figure a mass-promotion-spike test checks against the ceiling.
    public double PeakStepMbitPerSecond(double stepSeconds)
    {
        if (_entries.Count == 0 || stepSeconds <= 0.0)
        {
            return 0.0;
        }

        var perTime = new Dictionary<double, long>();
        foreach (var e in _entries)
        {
            perTime[e.Time] = perTime.GetValueOrDefault(e.Time) + e.Bytes;
        }

        long max = 0;
        foreach (var bytes in perTime.Values)
        {
            if (bytes > max)
            {
                max = bytes;
            }
        }

        return BytesToMbitPerSecond(max, stepSeconds);
    }

    // Sum of bytes already recorded (any topic) at EXACTLY `time` -- used by PedBandwidthGovernor to know
    // how much of this step's budget other topics (lifecycle switches, path-arc re-routes) have already
    // spent before it decides how many crowd-frame records still fit.
    public long BytesAtTime(double time)
    {
        long bytes = 0;
        foreach (var e in _entries)
        {
            if (e.Time == time)
            {
                bytes += e.Bytes;
            }
        }

        return bytes;
    }

    private static double BytesToMbitPerSecond(long bytes, double seconds) => bytes * 8.0 / 1_000_000.0 / seconds;

    public void Reset()
    {
        _entries.Clear();
        _totalBytes.Clear();
        _totalCount.Clear();
        _lastTime = double.NaN;
    }
}
