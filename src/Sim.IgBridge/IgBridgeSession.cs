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
    private readonly KinematicHeading _kinematic;           // front-position smoothing + rear-axle drag (docs §5.3)

    private readonly HashSet<VehicleHandle> _vehNewEmitted = new();
    private readonly HashSet<VehicleHandle> _vehDone = new();
    private readonly Dictionary<VehicleHandle, double> _lastEmitTau = new(); // for real elapsed dt across straddle gaps
    private readonly Dictionary<VehicleHandle, float> _lookAheadPrev = new(); // previously-USED predictor heading (jump guard)
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
        _kinematic = new KinematicHeading(kinematics);
        _emitDt = 1.0 / emit.EmitHz;
        _frameDt = (float)_emitDt;
        _nextEmit = _emitDt; // first emit instant is one emit-step in (t=0 has no motion yet)
    }

    public int EmittedCount { get; private set; }
    public IReadOnlyCollection<IgSample> Ring => _ring;

    // Spatial look-ahead (docs/IGBRIDGE-DECISIONS.md §5.13): metres ahead ON THE UPCOMING LANE CENTERLINE to
    // aim the front predictor at, so through a junction it anticipates the connecting lane instead of turning
    // in late. 0 disables (front follows the reactive current lane heading). Computed by re-resolving the
    // front pose with an extended length (PoseResolver already walks the upcoming lanes for the bumper).
    public double LookAheadMeters { get; set; }

    // A longer vehicle should anticipate proportionally further ahead (a bus starts its swing earlier than a
    // car). The effective look-ahead per vehicle is max(LookAheadMeters, LookAheadLengthFactor * length), so a
    // ~5 m passenger stays at LookAheadMeters (0.5*5 = 2.5 < 3 -> unchanged, byte-identical to v4) while a 12 m
    // bus aims ~6 m ahead. 0 keeps every vehicle on the flat LookAheadMeters.
    public double LookAheadLengthFactor { get; set; } = 0.5;

    // Max plausible frame-to-frame change (deg/s) of the look-ahead predictor heading; a bigger jump is a
    // spurious cross-junction resolve and is rejected (falls back to the lane heading). A real 90° junction
    // turn ramps at ~100°/s, so this passes turns and rejects the ~tens-of-degrees blips.
    private const float _lookAheadMaxJumpDegPerSec = 250f;

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

            if (!_runner.VehicleDims.TryGetValue(handle, out var dims))
            {
                continue;
            }

            var resolved = _clock.ResolveAt(history, tau, _runner.Lanes);

            // Resolve a CONTINUOUS front pose. A lateral straddle (junction lane-cross or lane change) is
            // NOT skipped -- skipping punches a hole in the emitted stream that the IG interpolates across,
            // and (fed a constant dt) desyncs the kinematic state -> tail wobble. Instead resolve both
            // bracketing states on their own lanes and Cartesian-lerp (the shipped lane-change reconstruction),
            // so the front path is unbroken.
            double frontX, frontY, frontZ;
            float laneHeading;
            double speed;
            var lateralEvent = resolved.IsLateralStraddle;
            if (resolved.IsLateralStraddle && resolved.SecondState is { } stateBraw)
            {
                if (!TryResolveFront(resolved.State, resolved.Upcoming, dims, upcoming, out var pa) ||
                    !TryResolveFront(stateBraw, resolved.SecondUpcoming, dims, upcoming, out var pb))
                {
                    continue;
                }

                var f = resolved.Blend;
                frontX = pa.X + (pb.X - pa.X) * f;
                frontY = pa.Y + (pb.Y - pa.Y) * f;
                frontZ = pa.Z + (pb.Z - pa.Z) * f;
                laneHeading = LerpHeadingDeg(pa.HeadingDeg, pb.HeadingDeg, f);
                speed = resolved.State.Speed + (stateBraw.Speed - resolved.State.Speed) * f;
            }
            else
            {
                if (!TryResolveFront(resolved.State, resolved.Upcoming, dims, upcoming, out var pose))
                {
                    continue;
                }

                frontX = pose.X;
                frontY = pose.Y;
                frontZ = pose.Z;
                laneHeading = pose.HeadingDeg;
                speed = resolved.State.Speed;
            }

            // Real elapsed dt for this vehicle (>= one emit step; larger after a gap) so the kinematic
            // SmoothDamp/drag integrate correctly instead of over/undershooting.
            var realDt = _lastEmitTau.TryGetValue(handle, out var last)
                ? (float)Math.Min(0.5, Math.Max(_emitDt, tau - last))
                : _frameDt;
            _lastEmitTau[handle] = tau;

            // Kinematic rear-axle drag (§5.3): the front reference tows a no-slip rear axle so the body
            // pivots at the rear like a real car. KinematicHeading also critically-damps the front POSITION
            // (removes the faceted-polyline kinks that would ride through as a tail wiggle). Emit the vehicle
            // CENTER (the IG's models pivot on center) + the drag heading; z (Q5) = the lane-surface ground z.
            // Spatial look-ahead heading (aim the front down the upcoming connecting lane, not the current
            // lane) so junction turn-ins hit the connecting-lane centerline instead of lagging off it.
            //
            // Guard against a bad resolve: the look-ahead point is occasionally unstable across a junction
            // (it briefly lands on a crossing/internal lane, giving a bearing tens of degrees off for ~0.2 s
            // then snapping back). A REAL turn's anticipation instead RAMPS smoothly. So reject the look-ahead
            // whenever it JUMPS from the previously-used predictor heading faster than a plausible yaw rate —
            // that kills the transient spurious excursions while passing the smooth turn ramp. On reject we
            // fall back to the lane heading (and remember that), so a sustained bad reading stays rejected.
            var effLookAhead = Math.Max(LookAheadMeters, LookAheadLengthFactor * dims.Length);
            float? predictHeading = null;
            if (effLookAhead > 0.0
                && TryLookAheadHeading(resolved.State, resolved.Upcoming, dims, frontX, frontY, effLookAhead, upcoming, out var lah))
            {
                var prevUsed = _lookAheadPrev.TryGetValue(handle, out var pv) ? pv : laneHeading;
                var jump = Math.Abs(((lah - prevUsed + 540f) % 360f) - 180f);
                if (jump <= _lookAheadMaxJumpDegPerSec * Math.Max(realDt, 1e-3f))
                {
                    predictHeading = lah;
                }
            }

            _lookAheadPrev[handle] = predictHeading ?? laneHeading;

            var kp = _kinematic.Update(handle, frontX, frontY, laneHeading, speed, dims.Length, realDt,
                lateralEvent, predictHeading);
            var id = _runner.IdOf(handle);

            if ((DebugVehicleId is not null && id == DebugVehicleId)
                || (DebugVehicleIds is not null && DebugVehicleIds.Contains(id)))
            {
                // rear bumper as the front-anchored template would draw it: center - (Length/2)*dir(heading)
                var hr = kp.HeadingDeg * Math.PI / 180.0;
                var dx = Math.Sin(hr);
                var dy = Math.Cos(hr);
                var rearBx = kp.CenterX - dims.Length * 0.5 * dx;
                var rearBy = kp.CenterY - dims.Length * 0.5 * dy;
                var row = string.Join(",", new[]
                {
                    tau.ToString("F4"), frontX.ToString("F4"), frontY.ToString("F4"), laneHeading.ToString("F4"),
                    kp.FrontX.ToString("F4"), kp.FrontY.ToString("F4"),
                    kp.CenterX.ToString("F4"), kp.CenterY.ToString("F4"), kp.HeadingDeg.ToString("F4"),
                    rearBx.ToString("F4"), rearBy.ToString("F4"), speed.ToString("F4"),
                    (predictHeading ?? laneHeading).ToString("F4"),
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
                ? IgSample.Created(id, tau, IgEntityModel.Car, kp.CenterX, kp.CenterY, frontZ, kp.HeadingDeg)
                : IgSample.Updated(id, tau, kp.CenterX, kp.CenterY, frontZ, kp.HeadingDeg));
        }
    }

    // Resolve one bracketing state to a front pose via PoseResolver (ChordHeading). Returns false if the
    // lane geometry isn't available.
    private bool TryResolveFront(DrState rawState, UpcomingLanes upcomingLanes,
        (double Length, double Width) dims, Span<int> scratch, out Pose pose)
    {
        var state = rawState with { Length = dims.Length, Width = dims.Width };
        var n = upcomingLanes.CopyTo(scratch);
        if (n == 0)
        {
            scratch[0] = state.LaneHandle;
            n = 1;
        }

        try
        {
            pose = PoseResolver.Resolve(
                _runner.Lanes, state, scratch[..n], ReadOnlySpan<int>.Empty, dt: 0.0, RenderRealism.ChordHeading);
            return true;
        }
        catch (KeyNotFoundException)
        {
            pose = default;
            return false;
        }
    }

    // Look-ahead heading: resolve a "front" pose with the length extended by LookAheadMeters — PoseResolver
    // walks the same upcoming lanes, so the extra length lands the point that far further along the path (down
    // the connecting lane through a junction). The chord from the real front to that point is the anticipatory
    // predictor direction. Returns false (caller falls back to the lane heading) if it can't be resolved or
    // the point is degenerate.
    private bool TryLookAheadHeading(DrState rawState, UpcomingLanes upcomingLanes,
        (double Length, double Width) dims, double frontX, double frontY, double lookAheadMeters,
        Span<int> scratch, out float headingDeg)
    {
        headingDeg = 0f;
        if (lookAheadMeters <= 0.0)
        {
            return false;
        }

        // Advance the front ARC position by lookAheadMeters (Pos is the front reference; Length only sets the
        // chord's back point, so extending it would NOT move the front forward). SampleForward walks into the
        // upcoming lanes, so past the junction this lands on the connecting-lane centerline.
        var state = rawState with { Length = dims.Length, Width = dims.Width, Pos = rawState.Pos + lookAheadMeters };
        var n = upcomingLanes.CopyTo(scratch);
        if (n == 0)
        {
            scratch[0] = state.LaneHandle;
            n = 1;
        }

        try
        {
            var ahead = PoseResolver.Resolve(
                _runner.Lanes, state, scratch[..n], ReadOnlySpan<int>.Empty, dt: 0.0, RenderRealism.ChordHeading);
            headingDeg = NaviFromVector(ahead.X - frontX, ahead.Y - frontY, out var moved);
            return moved;
        }
        catch (KeyNotFoundException)
        {
            return false;
        }
    }

    // Shortest-arc heading interpolation (navi-degrees).
    private static float LerpHeadingDeg(float a, float b, double f)
    {
        var d = ((b - a + 540f) % 360f) - 180f;
        var h = a + (float)(d * f);
        h %= 360f;
        return h < 0f ? h + 360f : h;
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
