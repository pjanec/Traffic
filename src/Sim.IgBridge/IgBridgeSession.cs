using Sim.Core;
using Sim.Replication;
using Sim.Viewer.Motion;

namespace Sim.IgBridge;

// Emit-stage configuration (docs/IGBRIDGE-DECISIONS.md §4). EmitHz is the sample cadence Spectacle/IgBridge
// resamples the reconstructed continuous curve at; LookaheadSeconds is the reconstruction lookahead (~1
// core tick) that keeps the query instant behind the newest core sample so DrClock.ResolveAt stays in the
// INTERPOLATE branch (arc-window junction turns, not extrapolation). This is NOT the IG playout delay --
// that is applied consumer-side by the FakeIg (T1.3), 0.5-1.0 s.
public sealed class IgEmitConfig
{
    public double EmitHz { get; init; } = 20.0;
    public double LookaheadSeconds { get; init; } = 0.1; // ~1 core tick at 10 Hz
}

// Stage [3] of the pipeline (docs/IGBRIDGE-DECISIONS.md §2/§4): resample the REUSED reconstruction stack
// at a fixed emit cadence and write IG-native new/upd/del records. Per emit instant tau (shared across ALL
// entities so the IG scene is time-coherent, §8), vehicles go through the exact viewer recipe
// (CityLib.Reconstructor) -- DrClock.ResolveAt -> PoseResolver.Resolve(ChordHeading) -> DrPoseSmoother --
// but fed the DETERMINISTIC ResolveAt(sampleT) instead of the wall-clock Resolve, and emitting PLANAR
// (x, y, headingDeg) only. The lateral-straddle (lane-change) case is SKIPPED here exactly as the viewer
// skips it in Stage 1; T2.1 adds the lane-change ease in Sim.Viewer.Motion and both inherit it. Pedestrians
// are smooth by construction (PathArc): their buffered history is linearly resampled at tau, heading
// velocity-derived. Lifecycle: `new` on an entity's first emitted sample, `del` once tau passes its frozen
// (despawned) history -- a stable id per entity, per §3.
public sealed class IgBridgeSession
{
    private readonly IgBridgeRunner _runner;
    private readonly IgEmitConfig _emit;
    private readonly IgTraceWriter _trace;
    private readonly double _emitDt;
    private readonly float _frameDt;

    private readonly DrClock _clock = new();          // used ONLY via ResolveAt (deterministic; no Pump)
    private readonly DrPoseSmoother _smoother = new(); // per-handle chase state, shared across vehicles

    private readonly HashSet<VehicleHandle> _vehNewEmitted = new();
    private readonly HashSet<VehicleHandle> _vehDone = new();
    private readonly HashSet<int> _pedNewEmitted = new();
    private readonly HashSet<int> _pedDone = new();

    private readonly Queue<IgSample> _ring = new();
    private readonly int _ringCapacity;
    private readonly List<IgSample>? _retained; // full in-memory stream for offline analysis (opt-in)
    private double _nextEmit;

    // `retainAll`: keep every emitted record in memory (AllEmitted) for the analysis/metrics pass. Off by
    // default -- the real IgBridge streams to the network and keeps only the bounded ring.
    public IgBridgeSession(IgBridgeRunner runner, IgEmitConfig emit, IgTraceWriter trace,
        int ringCapacity = 20000, bool retainAll = false)
    {
        _runner = runner;
        _emit = emit;
        _trace = trace;
        _ringCapacity = ringCapacity;
        _retained = retainAll ? new List<IgSample>() : null;
        _emitDt = 1.0 / emit.EmitHz;
        _frameDt = (float)_emitDt;
        _nextEmit = _emitDt; // first emit instant is one emit-step in (t=0 has no motion yet)
    }

    public int EmittedCount { get; private set; }
    public IReadOnlyCollection<IgSample> Ring => _ring;

    // The full emitted stream (only when constructed with retainAll: true; empty otherwise).
    public IReadOnlyList<IgSample> AllEmitted => (IReadOnlyList<IgSample>?)_retained ?? Array.Empty<IgSample>();

    // Drain every emit instant that has become resolvable (tau <= newestSampleTime - lookahead). Call once
    // after each IgBridgeRunner.Tick().
    public void Advance()
    {
        var horizon = _runner.SimTime - _emit.LookaheadSeconds;
        while (_nextEmit <= horizon + 1e-9)
        {
            EmitInstant(_nextEmit);
            _nextEmit += _emitDt;
        }
    }

    // Close out the trace: emit a `del` for every entity still open at the final horizon so the IG times
    // nothing out implicitly.
    public void Finish()
    {
        var tEnd = _runner.SimTime - _emit.LookaheadSeconds;
        foreach (var kv in _runner.VehicleHistories)
        {
            if (_vehNewEmitted.Contains(kv.Key) && !_vehDone.Contains(kv.Key))
            {
                Emit(IgSample.Removed(_runner.IdOf(kv.Key), tEnd));
                _vehDone.Add(kv.Key);
            }
        }

        foreach (var kv in _runner.PedHistories)
        {
            if (_pedNewEmitted.Contains(kv.Key) && !_pedDone.Contains(kv.Key))
            {
                Emit(IgSample.Removed(IgBridgeRunner.PedIdOf(kv.Key), tEnd));
                _pedDone.Add(kv.Key);
            }
        }

        _trace.Flush();
    }

    private void EmitInstant(double tau)
    {
        EmitVehicles(tau);
        EmitPeds(tau);
    }

    private void EmitVehicles(double tau)
    {
        Span<int> upcoming = stackalloc int[UpcomingLanes.Count];

        foreach (var kv in _runner.VehicleHistories)
        {
            var handle = kv.Key;
            if (_vehDone.Contains(handle))
            {
                continue;
            }

            var history = kv.Value;
            if (history.Count == 0)
            {
                continue;
            }

            var newestT = history[history.Count - 1].TimestampSeconds;
            if (tau > newestT + 1e-9)
            {
                // History frozen (vehicle despawned) and tau has passed it -> retire. Del only if the IG
                // ever saw this entity (a vehicle that never became interpolatable emits neither new nor del).
                if (_vehNewEmitted.Contains(handle))
                {
                    Emit(IgSample.Removed(_runner.IdOf(handle), tau));
                }

                _vehDone.Add(handle);
                continue;
            }

            if (history.Count < 2 || tau < history[0].TimestampSeconds)
            {
                continue; // not yet interpolatable
            }

            var resolved = _clock.ResolveAt(history, tau, _runner.Lanes);
            if (resolved.IsLateralStraddle)
            {
                continue; // Stage-1 skip: the lane-change ease (T2.1) handles the straddle in Sim.Viewer.Motion
            }

            if (!_runner.VehicleDims.TryGetValue(handle, out var dims))
            {
                continue;
            }

            var state = resolved.State with { Length = dims.Length, Width = dims.Width };
            var n = resolved.Upcoming.CopyTo(upcoming);
            if (n == 0)
            {
                upcoming[0] = state.LaneHandle;
                n = 1;
            }

            Pose pose;
            try
            {
                pose = PoseResolver.Resolve(
                    _runner.Lanes, state, upcoming[..n], ReadOnlySpan<int>.Empty, dt: 0.0,
                    RenderRealism.ChordHeading);
            }
            catch (KeyNotFoundException)
            {
                continue;
            }

            var (sx, sy, sdeg) = _smoother.Smooth(handle, pose.X, pose.Y, pose.HeadingDeg, state.Speed, _frameDt);

            // z (multi-level disambiguation, Q5-revised): from lane geometry via PoseResolver's Pose.Z
            // (0 on a flat net). The smoother corrects x,y only; z rides the arc geometry, which is smooth.
            var id = _runner.IdOf(handle);
            Emit(_vehNewEmitted.Add(handle)
                ? IgSample.Created(id, tau, IgEntityModel.Car, sx, sy, pose.Z, sdeg)
                : IgSample.Updated(id, tau, sx, sy, pose.Z, sdeg));
        }
    }

    private void EmitPeds(double tau)
    {
        foreach (var kv in _runner.PedHistories)
        {
            var id = kv.Key;
            if (_pedDone.Contains(id))
            {
                continue;
            }

            var history = kv.Value;
            if (history.Count == 0)
            {
                continue;
            }

            var newestT = history[history.Count - 1].TimestampSeconds;
            if (tau > newestT + 1e-9)
            {
                if (_pedNewEmitted.Contains(id))
                {
                    Emit(IgSample.Removed(IgBridgeRunner.PedIdOf(id), tau));
                }

                _pedDone.Add(id);
                continue;
            }

            if (history.Count < 2 || tau < history[0].TimestampSeconds)
            {
                continue;
            }

            // Linear resample of the (smooth-by-construction) PathArc history at tau; heading is the
            // velocity direction of the bracketing segment, in navi-degrees to match the vehicle wire.
            if (!TryInterpPed(history, tau, out var x, out var y, out var headingDeg))
            {
                continue;
            }

            // Ped z = 0: the crowd is 2-D (holonomic, no surface elevation). Multi-level ped z is a future
            // surface-mapping item (§ open); on a flat net it is 0 anyway.
            var sid = IgBridgeRunner.PedIdOf(id);
            Emit(_pedNewEmitted.Add(id)
                ? IgSample.Created(sid, tau, IgEntityModel.Ped, x, y, 0.0, headingDeg)
                : IgSample.Updated(sid, tau, x, y, 0.0, headingDeg));
        }
    }

    private static bool TryInterpPed(PedSampleHistory history, double tau, out double x, out double y, out float headingDeg)
    {
        x = y = 0.0;
        headingDeg = 0f;

        for (var i = 0; i < history.Count - 1; i++)
        {
            var a = history[i];
            var b = history[i + 1];
            if (tau < a.TimestampSeconds || tau > b.TimestampSeconds)
            {
                continue;
            }

            var span = b.TimestampSeconds - a.TimestampSeconds;
            var f = span > 1e-9 ? (tau - a.TimestampSeconds) / span : 0.0;
            x = a.X + (b.X - a.X) * f;
            y = a.Y + (b.Y - a.Y) * f;
            headingDeg = NaviFromVector(b.X - a.X, b.Y - a.Y, out var moved);
            if (!moved)
            {
                headingDeg = 0f; // stationary this segment -> hold due-north rather than a NaN direction
            }

            return true;
        }

        return false;
    }

    // Navi-degrees (0 = north, clockwise) of a direction vector -- same convention as PoseResolver.
    private static float NaviFromVector(double dx, double dy, out bool moved)
    {
        moved = (dx * dx + dy * dy) > 1e-12;
        if (!moved)
        {
            return 0f;
        }

        var deg = 90.0 - Math.Atan2(dy, dx) * 180.0 / Math.PI;
        deg %= 360.0;
        if (deg < 0.0)
        {
            deg += 360.0;
        }

        return (float)deg;
    }

    private void Emit(in IgSample s)
    {
        _trace.Write(s);
        _retained?.Add(s);
        _ring.Enqueue(s);
        if (_ring.Count > _ringCapacity)
        {
            _ring.Dequeue();
        }

        EmittedCount++;
    }
}
