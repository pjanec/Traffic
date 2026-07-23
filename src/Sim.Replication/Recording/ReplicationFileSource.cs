using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using Sim.Core;

namespace Sim.Replication.Recording;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §2.3, docs/LIVE-CITY-VIEWERS-TASKS.md Stage C (C2) — an IReplicationSource
// over the `.simrec` CAR track, driven by a PlaybackClock instead of wall time. Implemented by WRAPPING an
// internal InMemoryReplicationBus (reusing ALL of its decode/History/Dims/geometry bookkeeping verbatim) --
// Pump() replays every not-yet-applied record with time <= Clock.Now through `bus.Sink.Publish*`, then
// `bus.Source.Pump()` drains it, exactly mirroring how LiveCitySim/SimSource feed the SAME bus type live.
// Because this implements the EXISTING IReplicationSource contract, RenderHelpers.PumpAndBuildVehicleDraws
// (and therefore KinematicReconstructor/DrClock) consumes it completely unchanged -- design tenet 2 made
// concrete for the replay path.
public sealed class ReplicationFileSource : IReplicationSource
{
    private readonly PlaybackClock _clock;
    private readonly string _path;
    private SimRecReader _reader;
    private readonly InMemoryReplicationBus _bus = new();
    private SimRecEntry? _pending;
    private double _appliedUpTo = double.NegativeInfinity;

    public ReplicationFileSource(string path, PlaybackClock clock)
    {
        _path = path ?? throw new ArgumentNullException(nameof(path));
        _clock = clock ?? throw new ArgumentNullException(nameof(clock));

        // One up-front linear pass to learn the recording's total length (no footer time-index -- see
        // SimRecFormat.cs's remark; acceptable at the minute-scale recordings this feature targets).
        double maxTime = 0.0;
        double dt;
        string datasetId;
        using (var scan = new SimRecReader(path))
        {
            dt = scan.Dt;
            datasetId = scan.DatasetId;
            while (scan.TryReadNext(out var e))
            {
                if (e.Kind != SimRecFormat.RecordType.Geometry && e.Time > maxTime)
                {
                    maxTime = e.Time;
                }
            }
        }

        Dt = dt;
        DatasetId = datasetId;
        Duration = maxTime;

        _reader = new SimRecReader(path);
    }

    public double Duration { get; }

    public double Dt { get; }

    public string DatasetId { get; }

    // Forward advance (or no-op if Clock.Now has not moved): reads every not-yet-applied record with
    // time <= Clock.Now. Automatically rewinds + replays from the start when Clock.Now has moved BACKWARD
    // since the last call (the slider/Restart case) -- so a caller only ever needs to move the clock and
    // call Pump(); SeekTo below is the same logic exposed directly for callers (and tests) that want to
    // seek without going through a clock at all.
    public void Pump() => SeekTo(_clock.Now);

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.3 "Seek": forward just advances the cursor and pumps; backward
    // (or an arbitrary earlier target) resets the bus's per-vehicle state (History/Dims -- geometry, being
    // durable/static, is left alone) and replays every record from the start up to `t`. Because ALL prior
    // records are always replayed (not just those after the last keyframe), TL/lifecycle state ends up
    // correct too even with no explicit keyframe index -- see the class remark above.
    public void SeekTo(double t)
    {
        if (t < _appliedUpTo - 1e-9)
        {
            _bus.Source.ResetVehicles();
            _reader.Rewind();
            _pending = null;
            _appliedUpTo = double.NegativeInfinity;
        }

        AdvanceTo(t);
    }

    private void AdvanceTo(double t)
    {
        var appliedAny = false;

        while (true)
        {
            SimRecEntry entry;
            if (_pending is not null)
            {
                entry = _pending;
                _pending = null;
            }
            else if (!_reader.TryReadNext(out entry))
            {
                break;
            }

            // Geometry is durable/once -- apply unconditionally, never gated on time.
            var recordTime = entry.Kind == SimRecFormat.RecordType.Geometry ? double.NegativeInfinity : entry.Time;
            if (recordTime > t)
            {
                _pending = entry;
                break;
            }

            Apply(entry);
            appliedAny = true;
            if (recordTime > _appliedUpTo)
            {
                _appliedUpTo = recordTime;
            }
        }

        if (appliedAny)
        {
            _bus.Source.Pump();
        }
    }

    private void Apply(SimRecEntry entry)
    {
        switch (entry.Kind)
        {
            case SimRecFormat.RecordType.Geometry:
            {
                var lanes = new List<GeometryCodec.LaneGeo>();
                GeometryCodec.ReadGeometry(entry.GeometryBytes, lanes);
                _bus.Sink.PublishGeometry(lanes);
                break;
            }

            case SimRecFormat.RecordType.Lifecycle:
                _bus.Sink.PublishLifecycle(entry.Lifecycle);
                break;

            case SimRecFormat.RecordType.VehicleFrame:
            {
                var header = FrameCodec.ReadHeader(entry.VehicleFrameBytes);
                var recs = new VehicleRecord[header.Count];
                FrameCodec.ReadVehicleFrame(entry.VehicleFrameBytes, recs);
                // Use the record's own OUTER (double, full-precision) time, not the wrapped FrameCodec
                // header's float32 one -- see SimRecFormat.cs's remark.
                _bus.Sink.PublishFrame(header.Step, entry.Time, recs);
                break;
            }

            case SimRecFormat.RecordType.TrafficLights:
            {
                var count = BinaryPrimitives.ReadUInt16LittleEndian(entry.TlBytes.AsSpan(0, 2));
                var entries = new TlCodec.TlEntry[count];
                TlCodec.ReadTl(entry.TlBytes, entries);
                _bus.Sink.PublishTrafficLights(entry.TlStep, entry.Time, entries);
                break;
            }

            case SimRecFormat.RecordType.PedFrame:
                // Not part of the car bus -- PedFrameTrack reads these itself, from its own pass over the
                // same file.
                break;
        }
    }

    public IReadOnlyDictionary<int, GeometryCodec.LaneGeo> Geometry => _bus.Source.Geometry;
    public bool GeometryComplete => _bus.Source.GeometryComplete;
    public IReadOnlyDictionary<VehicleHandle, IVehicleSampleHistory> History => _bus.Source.History;
    public IReadOnlyDictionary<VehicleHandle, (float Length, float Width)> Dims => _bus.Source.Dims;
    public IReadOnlyDictionary<VehicleHandle, string> Names => _bus.Source.Names;
    public IReadOnlyDictionary<int, byte> TlStateByLane => _bus.Source.TlStateByLane;
    public double? LatestVehicleSampleTime => _bus.Source.LatestVehicleSampleTime;
    public bool Connected => _bus.Source.Connected;

    public void ResetVehicles() => _bus.Source.ResetVehicles();

    public bool TryGetLatest(VehicleHandle handle, out TimestampedSample sample) =>
        _bus.Source.TryGetLatest(handle, out sample);

    public void Dispose()
    {
        _reader.Dispose();
        _bus.Sink.Dispose();
        _bus.Source.Dispose();
    }
}
