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

    private readonly DrClock _clock = new();               // used ONLY via ResolveAt (deterministic; no Pump)
    private readonly KinematicReconstructor _recon;        // per-vehicle reconstruction core (front smoothing + rear-axle drag, docs §5.3)

    private readonly HashSet<VehicleHandle> _vehNewEmitted = new();
    private readonly HashSet<VehicleHandle> _vehDone = new();
    private readonly Dictionary<VehicleHandle, double> _lastEmitTau = new(); // for real elapsed dt across straddle gaps
    private readonly HashSet<int> _pedNewEmitted = new();
    private readonly HashSet<int> _pedDone = new();

    private readonly Queue<IgSample> _ring = new();
    private readonly int _ringCapacity;
    private readonly List<IgSample>? _retained; // full in-memory stream for offline analysis (opt-in)
    private double _nextEmit;

    // `retainAll`: keep every emitted record in memory (AllEmitted) for the analysis/metrics pass. Off by
    // default -- the real IgBridge streams to the network and keeps only the bounded ring.
    public IgBridgeSession(IgBridgeRunner runner, IgEmitConfig emit, IgTraceWriter trace,
        int ringCapacity = 20000, bool retainAll = false, KinematicHeadingParams? kinematics = null)
    {
        _runner = runner;
        _emit = emit;
        _trace = trace;
        _ringCapacity = ringCapacity;
        _retained = retainAll ? new List<IgSample>() : null;
        _recon = new KinematicReconstructor(kinematics);
        _emitDt = 1.0 / emit.EmitHz;
        _frameDt = (float)_emitDt;
        _nextEmit = _emitDt; // first emit instant is one emit-step in (t=0 has no motion yet)
    }

    public int EmittedCount { get; private set; }
    public IReadOnlyCollection<IgSample> Ring => _ring;

    // Emit-stage config now lives on the shared KinematicReconstructor (docs §1.1); these forward to it so
    // Program.cs still sets them on the session. See KinematicReconstructor for the per-field semantics.

    // Spatial look-ahead (docs/IGBRIDGE-DECISIONS.md §5.13): metres ahead ON THE UPCOMING LANE CENTERLINE to
    // aim the front predictor at. 0 disables (front follows the reactive current lane heading).
    public double LookAheadMeters { get => _recon.LookAheadMeters; set => _recon.LookAheadMeters = value; }

    // Effective look-ahead per vehicle = max(LookAheadMeters, LookAheadLengthFactor * length).
    public double LookAheadLengthFactor { get => _recon.LookAheadLengthFactor; set => _recon.LookAheadLengthFactor = value; }

    // Max ANGLE the look-ahead heading may lead the reactive lane heading by before it is rejected.
    public float MaxAnticipationLeadDeg { get => _recon.MaxAnticipationLeadDeg; set => _recon.MaxAnticipationLeadDeg = value; }

    // COARSE-FEED ONLY: the junction-turn-straddle discriminator threshold (see CoarseFeed).
    public float MaxStraddleLaneChangeHeadingDeg { get => _recon.MaxStraddleLaneChangeHeadingDeg; set => _recon.MaxStraddleLaneChangeHeadingDeg = value; }

    // Set when the feed is decimated (feedHz < core rate). Enables the junction-turn-straddle discriminator.
    public bool CoarseFeed { get => _recon.CoarseFeed; set => _recon.CoarseFeed = value; }

    // v6 opt-in: apply the junction-turn-straddle discriminator at the DENSE feed too (changes the v5 baseline).
    public bool AlwaysSplitJunctionStraddle { get => _recon.AlwaysSplitJunctionStraddle; set => _recon.AlwaysSplitJunctionStraddle = value; }

    // Diagnostics (T2.0 smoothness investigation): when DebugVehicleId is set, every emit instant for that
    // vehicle records a per-STAGE row so the jitter source can be localised (PoseResolver front -> position
    // smoother -> kinematic center/heading -> drawn rear bumper). Off (null) by default.
    public string? DebugVehicleId { get; set; }
    // Multiple ids (comma-set), each accumulating its own row list, so several suspects are captured in one run.
    public ISet<string>? DebugVehicleIds { get; set; }
    private readonly List<string> _debugRows = new();
    private readonly Dictionary<string, List<string>> _debugRowsById = new();
    public IReadOnlyList<string> DebugRows => _debugRows;
    public IReadOnlyDictionary<string, List<string>> DebugRowsById => _debugRowsById;

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

            if (!_runner.VehicleDims.TryGetValue(handle, out var dims))
            {
                continue;
            }

            var resolved = _clock.ResolveAt(history, tau, _runner.Lanes);

            // Real elapsed dt for this vehicle (>= one emit step; larger after a gap) so the kinematic
            // SmoothDamp/drag integrate correctly instead of over/undershooting.
            var realDt = _lastEmitTau.TryGetValue(handle, out var last)
                ? (float)Math.Min(0.5, Math.Max(_emitDt, tau - last))
                : _frameDt;
            _lastEmitTau[handle] = tau;

            // The full per-vehicle reconstruction (front resolve + straddle-lerp + look-ahead guards +
            // kinematic rear-axle drag) is the shared KinematicReconstructor (docs §1.1). A false Ok means the
            // PoseResolver resolve was unavailable this frame -> skip exactly as the old inline `continue` did.
            var result = _recon.Resolve(handle, resolved, _runner.Lanes, dims, realDt);
            if (!result.Ok)
            {
                continue;
            }

            var id = _runner.IdOf(handle);

            if ((DebugVehicleId is not null && id == DebugVehicleId)
                || (DebugVehicleIds is not null && DebugVehicleIds.Contains(id)))
            {
                // rear bumper as the front-anchored template would draw it: center - (Length/2)*dir(heading)
                var hr = result.HeadingDeg * Math.PI / 180.0;
                var dx = Math.Sin(hr);
                var dy = Math.Cos(hr);
                var rearBx = result.CenterX - dims.Length * 0.5 * dx;
                var rearBy = result.CenterY - dims.Length * 0.5 * dy;
                var row = string.Join(",", new[]
                {
                    tau.ToString("F4"), result.FrontX.ToString("F4"), result.FrontY.ToString("F4"), result.LaneHeadingDeg.ToString("F4"),
                    result.SmoothedFrontX.ToString("F4"), result.SmoothedFrontY.ToString("F4"),
                    result.CenterX.ToString("F4"), result.CenterY.ToString("F4"), result.HeadingDeg.ToString("F4"),
                    rearBx.ToString("F4"), rearBy.ToString("F4"), result.Speed.ToString("F4"),
                    result.PredictHeadingUsed.ToString("F4"),
                });
                if (id == DebugVehicleId)
                {
                    _debugRows.Add(row);
                }

                if (!_debugRowsById.TryGetValue(id, out var list))
                {
                    _debugRowsById[id] = list = new List<string>();
                }

                list.Add(row);
            }

            Emit(_vehNewEmitted.Add(handle)
                ? IgSample.Created(id, tau, IgEntityModel.Car, result.CenterX, result.CenterY, result.Z, result.HeadingDeg)
                : IgSample.Updated(id, tau, result.CenterX, result.CenterY, result.Z, result.HeadingDeg));
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
