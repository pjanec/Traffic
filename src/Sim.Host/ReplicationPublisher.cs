using Sim.Core;
using Sim.Ingest;
using Sim.Replication;

namespace Sim.Host;

// docs/DEMO-CITY3D-DESIGN.md "SumoSharp.Host — the missing reusable publisher": the transport-neutral
// half of the engine-to-wire publisher, extracted from src/Sim.Viewer.Core/DdsPublisher.cs. That class
// reads an EngineHost's Snapshot and writes DDS bytes; ReplicationPublisher keeps ONLY the
// snapshot/network -> wire-record translation (lifecycle bookkeeping, the DR-error publish gate, the
// LaneWindow -> UpcomingLanes projection, TL low-rate gating) and hands the records to whatever
// IReplicationSink the caller supplies -- DDS, in-memory, or any future transport. No DDS type, no
// Sim.Viewer.Core/EngineHost dependency here; a caller owns the Engine/SimulationRunner and the sink.
//
// Ported 1:1 from DdsPublisher's PublishGeometryOnce/PublishStep/PublishLifecycle/PublishVehicles/
// PublishTl/BuildLaneGeos/Reset -- same calculation and iteration order, so the records handed to a sink
// are byte-equivalent to what DdsPublisher would encode from the same snapshot. The one intentional
// difference is mechanical, not behavioral: DdsPublisher uses
// System.Runtime.InteropServices.CollectionsMarshal.AsSpan (net8.0-only, that project is net8.0-only) to
// view its scratch List<VehicleRecord> as a span; this class targets net8.0 *and* netstandard2.1, so it
// copies the scratch list into a reusable array instead -- same records, same order, same span contents.
public sealed class ReplicationPublisher
{
    // SimulationSnapshot.LaneWindow layout (see SimulationSnapshot.cs's doc comment): flattened
    // [p2,p1,cur,n1,n2,n3] per vehicle, current lane at offset 2 -- so [2,3,4,5] is exactly "current lane
    // first, then the next Sim.Replication.UpcomingLanes.Count-1 lanes ahead" the wire record wants.
    private const int LaneWindowCurOffset = 2;

    // SUMOSHARP-DEADRECKONING.md §7: the adaptive publish-rate scheduler, identical policy/tolerances to
    // DdsPublisher's own instance.
    private readonly PublishScheduler _scheduler = new(new DrErrorPublishPolicy
    {
        PosTol = 0.3,
        LatTol = 0.2,
        MaxInterval = 3,
    });

    // Lifecycle bookkeeping: the dims we've already told the sink about, so a spawn is only announced
    // once and a despawn is announced exactly once when a previously-known handle drops off the snapshot.
    private readonly Dictionary<VehicleHandle, (float Length, float Width)> _knownVehicles = new();
    private readonly HashSet<VehicleHandle> _currentScratch = new();
    private readonly List<VehicleHandle> _despawnScratch = new();

    private readonly List<VehicleRecord> _includeScratch = new();
    private VehicleRecord[] _includeArray = Array.Empty<VehicleRecord>();

    private double _lastTlTime = double.NegativeInfinity;

    // Publish the network's static lane geometry ONCE (durable-intent). Call this once, before the step
    // loop starts, after readers have had time to discover the sink's transport (if any).
    public void PublishGeometryOnce(NetworkModel network, IReplicationSink sink) =>
        sink.PublishGeometry(BuildLaneGeos(network));

    // Call once per sim step: publishes lifecycle deltas, the adaptive-rate-gated vehicle frame, and (at a
    // low rate) TL state.
    public void PublishStep(SimulationSnapshot snap, IReplicationSink sink)
    {
        PublishLifecycle(snap, sink);
        PublishVehicles(snap, sink);
        PublishTl(snap, sink);
        _scheduler.EndStep();
    }

    private void PublishLifecycle(SimulationSnapshot snap, IReplicationSink sink)
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
            // no handle-based vType registry yet (viewer-side dims only need Length/Width) -> vTypeId 0.
            // Name (docs/LIVE-CITY-VIEWERS-DESIGN.md §3.1, -TASKS.md E2): the real SUMO id for this handle,
            // from the SAME snapshot index -- SimulationSnapshot.VehicleId[] is parallel to Handles[] (both
            // indexed by `i`), so this is the id belonging to THIS spawn, not a stale/mismatched lookup.
            sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: true, vTypeId: 0, length, width, snap.VehicleId[i]));
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
            sink.PublishLifecycle(new LifecycleRecord(handle, isSpawn: false, vTypeId: 0, length: 0f, width: 0f));
            _knownVehicles.Remove(handle);
        }
    }

    private void PublishVehicles(SimulationSnapshot snap, IReplicationSink sink)
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

            if (!_scheduler.ShouldPublish(
                    handle, model, snap.Pos[i], snap.PosLat[i], speed, accel,
                    latSpeed: 0.0, snap.LaneHandle[i], snap.Time, manoeuvring))
            {
                continue;
            }

            var baseIdx = i * stride;
            for (var k = 0; k < UpcomingLanes.Count; k++)
            {
                var srcIdx = baseIdx + LaneWindowCurOffset + k;
                up[k] = srcIdx < baseIdx + stride ? snap.LaneWindow[srcIdx] : -1;
            }

            // latSpeed: no SimulationSnapshot column yet (SUMOSHARP-DEADRECKONING.md §4.1 -- lands at the
            // laneless merge); 0.0 is exact for every lane-centred vehicle in this phase's scenarios.
            _includeScratch.Add(new VehicleRecord(
                handle, model, snap.LaneHandle[i],
                snap.Pos[i], snap.PosLat[i], speed, accel, latSpeed: 0.0, new UpcomingLanes(up)));
        }

        if (_includeScratch.Count == 0)
        {
            return;
        }

        // netstandard2.1-clean span view of the scratch list (CollectionsMarshal.AsSpan is net5+-only):
        // copy into a reusable array, then hand the sink the exact same records DdsPublisher would.
        if (_includeArray.Length < _includeScratch.Count)
        {
            _includeArray = new VehicleRecord[_includeScratch.Count];
        }

        _includeScratch.CopyTo(_includeArray);
        sink.PublishFrame((uint)snap.StepCount, snap.Time, new ReadOnlySpan<VehicleRecord>(_includeArray, 0, _includeScratch.Count));
    }

    private void PublishTl(SimulationSnapshot snap, IReplicationSink sink)
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

        sink.PublishTrafficLights((uint)snap.StepCount, snap.Time, entries);
    }

    private static List<GeometryCodec.LaneGeo> BuildLaneGeos(NetworkModel network)
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

            // docs/LIVE-CITY-VIEWERS-DESIGN.md §3.2, -TASKS.md E1: thread Lane.ShapeZ (index-aligned with
            // Shape, null on every flat net -- the common case today) into the wire's additive per-point Z.
            // GeometryCodec.LaneGeo's ctor itself null-guards a length mismatch, but ShapeZ is parsed
            // 1:1 with Shape (Sim.Ingest.NetworkParser), so a mismatch never happens on real data.
            float[]? z = null;
            var shapeZ = lane.ShapeZ;
            if (shapeZ is not null && shapeZ.Count == shape.Count)
            {
                z = new float[shapeZ.Count];
                for (var i = 0; i < shapeZ.Count; i++)
                {
                    z[i] = (float)shapeZ[i];
                }
            }

            result.Add(new GeometryCodec.LaneGeo(
                lane.Handle, lane.Id.StartsWith(':'), (float)lane.Width, (float)lane.Length, points, z));
        }

        return result;
    }

    // Forget all per-vehicle publish state, for when the driving engine restarts at t=0. Mirrors
    // DdsPublisher.Reset(): the adaptive scheduler and the known-vehicle registry are cleared so the
    // fresh timeline's vehicles re-announce their lifecycle/dims and are not suppressed by stale
    // last-sent bookkeeping from the old (now-larger) sim time.
    public void Reset()
    {
        _scheduler.Reset();
        _knownVehicles.Clear();
    }
}
