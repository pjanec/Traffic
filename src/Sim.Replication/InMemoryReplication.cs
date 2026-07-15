using Sim.Core;

namespace Sim.Replication;

// docs/SUMOSHARP-PACKAGING-DESIGN.md P1 (D8) — a same-process, non-DDS binding of IReplicationSink /
// IReplicationSource, proving the contract in IReplication.cs is transport-neutral rather than an
// after-the-fact abstraction over DDS. No codec, no bytes: records are queued and handed straight
// across, since there is no wire to cross in-process. A publisher writes through Sink; a consumer reads
// through Source after calling Pump() to drain the queue -- mirroring DdsSubscriber's own Pump-then-read
// pattern so a caller coded against the interfaces cannot tell which binding it holds.
public sealed class InMemoryReplicationBus
{
    private const int HistoryCapacity = 8;

    // Discriminated queue entry -- Kind selects which payload field is meaningful. A plain struct (not a
    // closure/delegate) keeps this allocation-light per publish call.
    private enum EntryKind { Geometry, Lifecycle, Frame, TrafficLights }

    private readonly struct Entry
    {
        public Entry(IReadOnlyList<GeometryCodec.LaneGeo> lanes)
        { Kind = EntryKind.Geometry; Lanes = lanes; Lifecycle = default; Movers = default; Lights = default; Step = 0; Time = 0; }

        public Entry(in LifecycleRecord lifecycle)
        { Kind = EntryKind.Lifecycle; Lanes = default; Lifecycle = lifecycle; Movers = default; Lights = default; Step = 0; Time = 0; }

        public Entry(uint step, double time, VehicleRecord[] movers)
        { Kind = EntryKind.Frame; Lanes = default; Lifecycle = default; Movers = movers; Lights = default; Step = step; Time = time; }

        public Entry(uint step, double time, IReadOnlyList<TlCodec.TlEntry> lights, bool isTl)
        { Kind = EntryKind.TrafficLights; Lanes = default; Lifecycle = default; Movers = default; Lights = lights; Step = step; Time = time; }

        public EntryKind Kind { get; }
        public IReadOnlyList<GeometryCodec.LaneGeo>? Lanes { get; }
        public LifecycleRecord Lifecycle { get; }
        public VehicleRecord[]? Movers { get; }
        public IReadOnlyList<TlCodec.TlEntry>? Lights { get; }
        public uint Step { get; }
        public double Time { get; }
    }

    private readonly Queue<Entry> _queue = new();

    private readonly Dictionary<int, GeometryCodec.LaneGeo> _geometry = new();
    private bool _geometryComplete;

    private readonly Dictionary<VehicleHandle, VehicleSampleHistory> _history = new();
    private readonly Dictionary<VehicleHandle, (float Length, float Width)> _dims = new();
    private readonly Dictionary<int, byte> _tlState = new();
    private bool _pumpedAfterPublish;

    private sealed class HistoryView : IReadOnlyDictionary<VehicleHandle, IVehicleSampleHistory>
    {
        private readonly Dictionary<VehicleHandle, VehicleSampleHistory> _inner;

        public HistoryView(Dictionary<VehicleHandle, VehicleSampleHistory> inner) => _inner = inner;

        public IVehicleSampleHistory this[VehicleHandle key] => _inner[key];
        public IEnumerable<VehicleHandle> Keys => _inner.Keys;
        public IEnumerable<IVehicleSampleHistory> Values => _inner.Values;
        public int Count => _inner.Count;
        public bool ContainsKey(VehicleHandle key) => _inner.ContainsKey(key);

        public bool TryGetValue(VehicleHandle key, out IVehicleSampleHistory value)
        {
            if (_inner.TryGetValue(key, out var v))
            {
                value = v;
                return true;
            }

            value = default!;
            return false;
        }

        public IEnumerator<KeyValuePair<VehicleHandle, IVehicleSampleHistory>> GetEnumerator()
        {
            foreach (var kv in _inner)
            {
                yield return new KeyValuePair<VehicleHandle, IVehicleSampleHistory>(kv.Key, kv.Value);
            }
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();
    }

    private readonly HistoryView _historyView;

    public InMemoryReplicationBus()
    {
        _historyView = new HistoryView(_history);
        Sink = new SinkImpl(this);
        Source = new SourceImpl(this);
    }

    public IReplicationSink Sink { get; }
    public IReplicationSource Source { get; }

    private sealed class SinkImpl : IReplicationSink
    {
        private readonly InMemoryReplicationBus _bus;
        public SinkImpl(InMemoryReplicationBus bus) => _bus = bus;

        public void PublishGeometry(IReadOnlyList<GeometryCodec.LaneGeo> lanes) =>
            _bus._queue.Enqueue(new Entry(lanes));

        public void PublishLifecycle(in LifecycleRecord record) =>
            _bus._queue.Enqueue(new Entry(record));

        public void PublishFrame(uint step, double time, ReadOnlySpan<VehicleRecord> movers) =>
            _bus._queue.Enqueue(new Entry(step, time, movers.ToArray()));

        public void PublishTrafficLights(uint step, double time, IReadOnlyList<TlCodec.TlEntry> lights) =>
            _bus._queue.Enqueue(new Entry(step, time, lights, isTl: true));

        public void Dispose() { }
    }

    private sealed class SourceImpl : IReplicationSource
    {
        private readonly InMemoryReplicationBus _bus;
        public SourceImpl(InMemoryReplicationBus bus) => _bus = bus;

        public void Pump() => _bus.PumpCore();

        public IReadOnlyDictionary<int, GeometryCodec.LaneGeo> Geometry => _bus._geometry;
        public bool GeometryComplete => _bus._geometryComplete;
        public IReadOnlyDictionary<VehicleHandle, IVehicleSampleHistory> History => _bus._historyView;
        public IReadOnlyDictionary<VehicleHandle, (float Length, float Width)> Dims => _bus._dims;
        public IReadOnlyDictionary<int, byte> TlStateByLane => _bus._tlState;
        public double? LatestVehicleSampleTime { get; internal set; }
        public bool Connected => _bus._pumpedAfterPublish;

        public void ResetVehicles()
        {
            _bus._history.Clear();
            _bus._dims.Clear();
            LatestVehicleSampleTime = null;
        }

        public bool TryGetLatest(VehicleHandle handle, out TimestampedSample sample)
        {
            if (_bus._history.TryGetValue(handle, out var hist) && hist.Count > 0)
            {
                sample = hist[hist.Count - 1];
                return true;
            }

            sample = default;
            return false;
        }

        public void Dispose() { }
    }

    private void PumpCore()
    {
        var sawAny = _queue.Count > 0;
        while (_queue.Count > 0)
        {
            var e = _queue.Dequeue();
            switch (e.Kind)
            {
                case EntryKind.Geometry:
                    foreach (var lane in e.Lanes!)
                    {
                        _geometry[lane.Handle] = lane;
                    }

                    _geometryComplete = true;
                    break;

                case EntryKind.Lifecycle:
                    var lc = e.Lifecycle;
                    if (lc.IsSpawn)
                    {
                        _dims[lc.Handle] = (lc.Length, lc.Width);
                    }
                    else
                    {
                        _dims.Remove(lc.Handle);
                        _history.Remove(lc.Handle);
                    }

                    break;

                case EntryKind.Frame:
                    var movers = e.Movers!;
                    for (var i = 0; i < movers.Length; i++)
                    {
                        var rec = movers[i];
                        if (!_history.TryGetValue(rec.Handle, out var hist))
                        {
                            hist = new VehicleSampleHistory(HistoryCapacity);
                            _history[rec.Handle] = hist;
                        }

                        hist.Append(new TimestampedSample(e.Time, rec));
                    }

                    var srcImpl = (SourceImpl)Source;
                    if (srcImpl.LatestVehicleSampleTime is null || e.Time > srcImpl.LatestVehicleSampleTime.Value)
                    {
                        srcImpl.LatestVehicleSampleTime = e.Time;
                    }

                    break;

                case EntryKind.TrafficLights:
                    foreach (var entry in e.Lights!)
                    {
                        _tlState[entry.LaneHandle] = entry.Signal;
                    }

                    break;
            }
        }

        if (sawAny)
        {
            _pumpedAfterPublish = true;
        }
    }
}
