using CycloneDDS.Runtime;
using Sim.Core;
using Sim.Replication;
using Sim.Replication.Dds;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2 — the read side of the native viewer's DDS data path. Decodes the four
// topics DdsPublisher writes into plain in-memory state a renderer (P2b) or this phase's LoopbackSelfTest
// can inspect: static lane geometry (once), a short per-vehicle history (newest last, for later
// interpolation/DR), TL state, and lifecycle dims. No rendering, no DrClock/extrapolation here.
//
// CRITICAL (see Sim.Replication.Dds/README.md): every reader below is constructed with the SAME explicit
// DdsTopicNames.* string the matching DdsPublisher writer used.
public sealed class DdsSubscriber : IDisposable
{
    private const int HistoryCap = 8;

    // One decoded per-vehicle sample, timestamped with the DDS sample's own source timestamp when the
    // reader exposes one (CycloneDDS.NET's DdsSampleInfo.SourceTimestamp, nanoseconds since the Unix
    // epoch); falls back to the frame's own sim-time field on the (untested-in-practice) chance the
    // timestamp comes back <= 0. This is what a later DrClock paces its render clock off (§8 of
    // SUMOSHARP-DEADRECKONING.md): wall-clock arrival time, not sim time, is what a REMOTE subscriber
    // actually has to reconstruct a rate from.
    public readonly struct VehicleSample
    {
        public VehicleSample(double timestampSeconds, VehicleRecord record)
        {
            TimestampSeconds = timestampSeconds;
            Record = record;
        }

        public double TimestampSeconds { get; }
        public VehicleRecord Record { get; }
    }

    private readonly DdsReader<DdsWireFrame> _vehicleReader;
    private readonly DdsReader<DdsNetworkGeometry> _geometryReader;
    private readonly DdsReader<DdsVehicleLifecycle> _lifecycleReader;
    private readonly DdsReader<DdsTlState> _tlReader;

    private readonly Dictionary<int, GeometryCodec.LaneGeo> _geometryByHandle = new();
    private readonly HashSet<int> _geometryChunksReceived = new();
    private int _geometryTotalChunks = -1;

    private readonly Dictionary<VehicleHandle, List<VehicleSample>> _history = new();
    private readonly Dictionary<VehicleHandle, (float Length, float Width)> _dims = new();
    private readonly Dictionary<int, byte> _tlState = new();

    private byte[] _vehicleBytes = new byte[DdsWire.MaxPayload];
    private VehicleRecord[] _vehicleRecs = new VehicleRecord[2048];
    private byte[] _geometryBytes = new byte[DdsWire.MaxPayload];
    private byte[] _tlBytes = new byte[DdsWire.MaxPayload];

    // Must mirror DdsPublisher's per-topic QoS profile exactly (a RELIABLE reader will not match a
    // BEST_EFFORT writer) -- see DdsQos's doc comment.
    public DdsSubscriber(DdsParticipant participant)
    {
        _vehicleReader = new DdsReader<DdsWireFrame>(participant, DdsTopicNames.Vehicles, DdsQos.VolatileLatest());
        _geometryReader = new DdsReader<DdsNetworkGeometry>(participant, DdsTopicNames.Geometry, DdsQos.DurableLatest());
        _lifecycleReader = new DdsReader<DdsVehicleLifecycle>(participant, DdsTopicNames.Lifecycle, DdsQos.DurableLatest());
        _tlReader = new DdsReader<DdsTlState>(participant, DdsTopicNames.Tl, DdsQos.VolatileLatest());
    }

    // Drop all per-vehicle state (history + dims + latest-sample-time) so it rebuilds from the live stream.
    // Called on a publisher RESTART: the publisher rebuilds at t=0 and REUSES vehicle handle indices, so a
    // reused handle's buffer would otherwise mix old-timeline samples (large Pos) with new ones (small Pos)
    // -> the DR flings that vehicle backward instead of the old one vanishing. Geometry + TL are kept (the
    // network didn't change).
    public void ResetVehicles()
    {
        _history.Clear();
        _dims.Clear();
        LatestVehicleSampleTime = null;
    }

    // Decoded static lane geometry, keyed by lane handle. Empty until PublishGeometryOnce's chunk(s) have
    // arrived; see GeometryComplete.
    public IReadOnlyDictionary<int, GeometryCodec.LaneGeo> Geometry => _geometryByHandle;

    // True once every announced chunk (0..TotalChunks-1) has been seen at least once.
    public bool GeometryComplete => _geometryTotalChunks >= 0 && _geometryChunksReceived.Count >= _geometryTotalChunks;

    // Per-vehicle history, newest sample last, capped at HistoryCap -- the raw material a later DR/
    // interpolation consumer (P2b) walks.
    public IReadOnlyDictionary<VehicleHandle, List<VehicleSample>> History => _history;

    // Dims announced over the lifecycle topic (spawn), removed on despawn.
    public IReadOnlyDictionary<VehicleHandle, (float Length, float Width)> Dims => _dims;

    // Latest TL signal char per controlled lane handle.
    public IReadOnlyDictionary<int, byte> TlStateByLane => _tlState;

    // P2b (DrClock) — the newest per-vehicle sample timestamp seen so far across every decoded vehicle
    // frame (max of VehicleSample.TimestampSeconds); null until the first vehicle frame arrives. All
    // vehicle records decoded from the SAME DDS sample share one timestamp, so this is exactly the
    // "current frame's sim-axis time" DrClock.Pump paces its render clock off.
    public double? LatestVehicleSampleTime { get; private set; }

    // P2b diagnostics — total vehicle records decoded across every Pump call so far (a monotonically
    // increasing counter; divide by elapsed wall time for a "DDS samples/s" HUD readout).
    public long TotalVehicleSamplesReceived { get; private set; }

    // P3 ("remote mode + QoS") — true once THIS reader is currently matched with at least one live writer
    // on the Vehicles topic (DDS's own publication-matched bookkeeping, not an inference from whether data
    // has arrived yet), so a remote viewer's "connected: yes/no" indicator reflects the transport itself,
    // not just "have I decoded a frame". A publisher that starts, publishes once, then is killed will show
    // CurrentCount drop back to 0 here even though TotalVehicleSamplesReceived stays nonzero.
    public bool Connected => _vehicleReader.CurrentStatus.CurrentCount > 0;

    public bool TryGetLatest(VehicleHandle handle, out VehicleSample sample)
    {
        if (_history.TryGetValue(handle, out var list) && list.Count > 0)
        {
            sample = list[^1];
            return true;
        }

        sample = default;
        return false;
    }

    // Pump every topic once. Call this at whatever cadence the host polls DDS (the loopback self-test polls
    // once per publish step).
    public void Pump()
    {
        PumpGeometry();
        PumpLifecycle();
        PumpVehicles();
        PumpTl();
    }

    private void PumpGeometry()
    {
        using var loan = _geometryReader.Take(maxSamples: 16);
        var decoded = new List<GeometryCodec.LaneGeo>();
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var f = sample.Data;
            var n = f.ReadPayload(_geometryBytes);
            decoded.Clear();
            GeometryCodec.ReadGeometry(_geometryBytes.AsSpan(0, n), decoded);
            foreach (var lane in decoded)
            {
                _geometryByHandle[lane.Handle] = lane;
            }

            _geometryTotalChunks = f.TotalChunks;
            _geometryChunksReceived.Add(f.ChunkIndex);
        }
    }

    private void PumpLifecycle()
    {
        using var loan = _lifecycleReader.Take(maxSamples: 256);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var d = sample.Data;
            var handle = new VehicleHandle(d.Index, d.Generation);
            if (d.Event == 0)
            {
                _dims[handle] = (d.Length, d.Width);
            }
            else
            {
                _dims.Remove(handle);
                _history.Remove(handle);
            }
        }
    }

    private void PumpVehicles()
    {
        using var loan = _vehicleReader.Take(maxSamples: 64);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var f = sample.Data;
            var n = f.ReadPayload(_vehicleBytes);
            var count = FrameCodec.ReadVehicleFrame(_vehicleBytes.AsSpan(0, n), _vehicleRecs);

            var ts = sample.Info.SourceTimestamp;
            var timestampSeconds = ts > 0 ? ts / 1_000_000_000.0 : (double)f.Time;

            if (LatestVehicleSampleTime is null || timestampSeconds > LatestVehicleSampleTime.Value)
            {
                LatestVehicleSampleTime = timestampSeconds;
            }

            for (var i = 0; i < count; i++)
            {
                var rec = _vehicleRecs[i];
                if (!_history.TryGetValue(rec.Handle, out var list))
                {
                    list = new List<VehicleSample>(HistoryCap);
                    _history[rec.Handle] = list;
                }

                list.Add(new VehicleSample(timestampSeconds, rec));
                if (list.Count > HistoryCap)
                {
                    list.RemoveAt(0);
                }
            }

            TotalVehicleSamplesReceived += count;
        }
    }

    private void PumpTl()
    {
        using var loan = _tlReader.Take(maxSamples: 8);
        foreach (var sample in loan)
        {
            if (!sample.IsValid)
            {
                continue;
            }

            var f = sample.Data;
            var n = f.ReadPayload(_tlBytes);
            var maxEntries = Math.Max(1, (n - TlCodec.HeaderSize) / TlCodec.EntrySize);
            var entries = new TlCodec.TlEntry[maxEntries];
            var count = TlCodec.ReadTl(_tlBytes.AsSpan(0, n), entries);
            for (var i = 0; i < count; i++)
            {
                _tlState[entries[i].LaneHandle] = entries[i].Signal;
            }
        }
    }

    public void Dispose()
    {
        _vehicleReader.Dispose();
        _geometryReader.Dispose();
        _lifecycleReader.Dispose();
        _tlReader.Dispose();
    }
}
