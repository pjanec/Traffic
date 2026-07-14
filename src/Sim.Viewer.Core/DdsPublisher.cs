using System.Runtime.InteropServices;
using CycloneDDS.Runtime;
using Sim.Core;
using Sim.Replication;
using Sim.Replication.Dds;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2 ("DDS topics + loopback DR") — the publish side of the native
// viewer's DDS data path. Reads an EngineHost's authoritative Snapshot each step and writes it out over
// four topics (vehicles/geometry/lifecycle/TL), reusing the same FrameCodec/GeometryCodec/TlCodec bytes a
// TCP/UDP host would send. No rendering, no dead reckoning here — this is the write side only; DdsSubscriber
// (+ a later DrClock) is the read side.
//
// CRITICAL (see Sim.Replication.Dds/README.md): every writer below is constructed with an EXPLICIT
// DdsTopicNames.* string — the [DdsTopic] attribute's own name is NOT usable (the 1-arg ctor throws
// ArgumentNullException), and a DdsSubscriber reading the SAME topic must pass the identical string.
public sealed class DdsPublisher : IDisposable
{
    // SimulationSnapshot.LaneWindow layout (see SimulationSnapshot.cs's doc comment): flattened
    // [p2,p1,cur,n1,n2,n3] per vehicle, current lane at offset 2 -- so [2,3,4,5] is exactly "current lane
    // first, then the next Sim.Replication.UpcomingLanes.Count-1 lanes ahead" the wire record wants.
    private const int LaneWindowCurOffset = 2;

    private readonly EngineHost _host;
    private readonly DdsWriter<DdsWireFrame> _vehicleWriter;
    private readonly DdsWriter<DdsNetworkGeometry> _geometryWriter;
    private readonly DdsWriter<DdsVehicleLifecycle> _lifecycleWriter;
    private readonly DdsWriter<DdsTlState> _tlWriter;

    // SUMOSHARP-DEADRECKONING.md §7: the adaptive publish-rate scheduler. FastInterval/SlowInterval are in
    // SIM-TIME units (SimulationSnapshot.Time), not wall-clock seconds -- this repo's scenarios step in
    // whole seconds (SUMOSHARP-LIVEVIZ-OUTCOMES.md's demo scaling), so "1" = every step, "3" = at most
    // every third step for a steady/predictable mover.
    private readonly PublishScheduler _scheduler = new(new DefaultPublishPolicy
    {
        FastInterval = 1,
        SlowInterval = 3,
        AccelThreshold = 0.3,
    });

    // Lifecycle bookkeeping: the dims we've already told subscribers about, so a spawn is only announced
    // once and a despawn is announced exactly once when a previously-known handle drops off the snapshot.
    private readonly Dictionary<VehicleHandle, (float Length, float Width)> _knownVehicles = new();
    private readonly HashSet<VehicleHandle> _currentScratch = new();
    private readonly List<VehicleHandle> _despawnScratch = new();

    private readonly List<VehicleRecord> _includeScratch = new();
    private byte[] _vehicleScratch = new byte[DdsWire.MaxPayload];
    private byte[] _tlScratch = new byte[DdsWire.MaxPayload];
    private double _lastTlTime = double.NegativeInfinity;

    // docs/SUMOSHARP-NATIVE-VIEWER.md P3 QoS: geometry + lifecycle are RELIABLE/TRANSIENT_LOCAL (durable --
    // a late-joining remote reader gets the last-published network + fleet); Vehicles + Tl are
    // BEST_EFFORT/VOLATILE (perishable state, already re-published periodically) -- see DdsQos's doc
    // comment for the two-process durability verification behind this split.
    public DdsPublisher(EngineHost host, DdsParticipant participant)
    {
        _host = host;
        _vehicleWriter = new DdsWriter<DdsWireFrame>(participant, DdsTopicNames.Vehicles, DdsQos.VolatileLatest());
        _geometryWriter = new DdsWriter<DdsNetworkGeometry>(participant, DdsTopicNames.Geometry, DdsQos.DurableLatest());
        _lifecycleWriter = new DdsWriter<DdsVehicleLifecycle>(participant, DdsTopicNames.Lifecycle, DdsQos.DurableLatest());
        _tlWriter = new DdsWriter<DdsTlState>(participant, DdsTopicNames.Tl, DdsQos.VolatileLatest());
    }

    // Publish the network's static lane geometry ONCE (durable-intent: see the topic's own comment for the
    // QoS caveat this phase doesn't yet set). Call this once, before the step loop starts, after readers
    // have had time to discover the writer (DDS discovery is async — the caller sleeps first).
    public void PublishGeometryOnce()
    {
        var lanes = BuildLaneGeos(_host.Network);
        var chunks = GeometryCodec.PlanChunks(lanes, DdsWire.MaxPayload);
        var laneSpan = CollectionsMarshal.AsSpan(lanes);
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

    // Call once per sim step: publishes lifecycle deltas, the adaptive-rate-gated vehicle state, and (at a
    // low rate) TL state.
    public void PublishStep()
    {
        var snap = _host.Snapshot;
        PublishLifecycle(snap);
        PublishVehicles(snap);
        PublishTl(snap);
        _scheduler.EndStep();
    }

    private void PublishLifecycle(SimulationSnapshot snap)
    {
        _currentScratch.Clear();
        for (var i = 0; i < snap.Count; i++)
        {
            var handle = snap.Handles[i];
            _currentScratch.Add(handle);
            if (_knownVehicles.ContainsKey(handle))
            {
                continue;
            }

            var length = snap.Length[i];
            var width = snap.Width[i];
            var rec = new DdsVehicleLifecycle
            {
                Index = handle.Index,
                Generation = handle.Generation,
                Event = 0,
                VTypeId = 0, // no handle-based vType registry yet (viewer-side dims only need Length/Width)
                Length = length,
                Width = width,
            };
            _lifecycleWriter.Write(rec);
            _knownVehicles[handle] = (length, width);
        }

        _despawnScratch.Clear();
        foreach (var handle in _knownVehicles.Keys)
        {
            if (!_currentScratch.Contains(handle))
            {
                _despawnScratch.Add(handle);
            }
        }

        foreach (var handle in _despawnScratch)
        {
            var rec = new DdsVehicleLifecycle { Index = handle.Index, Generation = handle.Generation, Event = 1 };
            _lifecycleWriter.Write(rec);
            _knownVehicles.Remove(handle);
        }
    }

    private void PublishVehicles(SimulationSnapshot snap)
    {
        _includeScratch.Clear();
        var stride = snap.LaneWindowStride;
        Span<int> up = stackalloc int[UpcomingLanes.Count];

        for (var i = 0; i < snap.Count; i++)
        {
            var handle = snap.Handles[i];
            var model = (DrModel)snap.DrModels[i];
            var manoeuvring = snap.Manoeuvring[i];
            var speed = snap.SpeedExact[i];
            var accel = snap.Accel[i];

            if (!_scheduler.ShouldPublish(handle, model, speed, accel, snap.Time, manoeuvring))
            {
                continue;
            }

            var baseIdx = i * stride;
            for (var k = 0; k < UpcomingLanes.Count; k++)
            {
                var srcIdx = baseIdx + LaneWindowCurOffset + k;
                up[k] = srcIdx < baseIdx + stride ? snap.LaneWindow[srcIdx] : -1;
            }

            // latSpeed: no SimulationSnapshot column yet (SUMOSHARP-DEADRECKONING.md §4.1 — lands at the
            // laneless merge); 0.0 is exact for every lane-centred vehicle in this phase's scenarios.
            _includeScratch.Add(new VehicleRecord(
                handle, model, snap.LaneHandle[i],
                snap.Pos[i], snap.PosLat[i], speed, accel, latSpeed: 0.0, new UpcomingLanes(up)));
        }

        if (_includeScratch.Count == 0)
        {
            return;
        }

        var recs = CollectionsMarshal.AsSpan(_includeScratch);
        var maxPerChunk = Math.Max(1, FrameChunker.MaxRecordsForPayload(DdsWire.MaxPayload, FrameCodec.VehicleRecordSize));
        var totalChunks = FrameChunker.ChunkCount(recs.Length, maxPerChunk);
        for (var c = 0; c < totalChunks; c++)
        {
            var (start, count) = FrameChunker.ChunkRange(c, recs.Length, maxPerChunk);
            var chunk = recs.Slice(start, count);
            var size = FrameCodec.WriteVehicleFrame(_vehicleScratch, (uint)snap.StepCount, (float)snap.Time, chunk);
            var frame = new DdsWireFrame();
            frame.SetPayload(FrameCodec.KindVehicle, (uint)snap.StepCount, (float)snap.Time, _vehicleScratch.AsSpan(0, size), c);
            _vehicleWriter.Write(frame);
        }
    }

    private void PublishTl(SimulationSnapshot snap)
    {
        if (!double.IsNegativeInfinity(_lastTlTime) && snap.Time - _lastTlTime < 1.0)
        {
            return;
        }

        _lastTlTime = snap.Time;

        if (snap.TlCount == 0)
        {
            return;
        }

        var entries = new TlCodec.TlEntry[snap.TlCount];
        for (var i = 0; i < snap.TlCount; i++)
        {
            entries[i] = new TlCodec.TlEntry(snap.TlLaneHandle[i], snap.TlState[i]);
        }

        var size = TlCodec.WriteTl(_tlScratch, entries);
        var frame = new DdsTlState();
        frame.SetPayload((uint)snap.StepCount, (float)snap.Time, _tlScratch.AsSpan(0, size));
        _tlWriter.Write(frame);
    }

    private static List<GeometryCodec.LaneGeo> BuildLaneGeos(Sim.Ingest.NetworkModel network)
    {
        var lanes = network.LanesByHandle;
        var result = new List<GeometryCodec.LaneGeo>(lanes.Count);
        foreach (var lane in lanes)
        {
            var shape = lane.Shape;
            var points = new (float X, float Y)[shape.Count];
            for (var i = 0; i < shape.Count; i++)
            {
                points[i] = ((float)shape[i].X, (float)shape[i].Y);
            }

            result.Add(new GeometryCodec.LaneGeo(
                lane.Handle, lane.Id.StartsWith(':'), (float)lane.Width, (float)lane.Length, points));
        }

        return result;
    }

    public void Dispose()
    {
        _vehicleWriter.Dispose();
        _geometryWriter.Dispose();
        _lifecycleWriter.Dispose();
        _tlWriter.Dispose();
    }
}
