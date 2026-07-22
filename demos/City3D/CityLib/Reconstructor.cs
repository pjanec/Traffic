using System.Diagnostics;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;
using Sim.Viewer.Motion;

namespace CityLib;

// One vehicle's fully-reconstructed render pose, in GODOT coordinates/yaw (CoordinateTransform already
// applied) -- the plain struct a later Godot-glue layer turns into a MultiMesh per-instance transform. No
// Godot type here (CityLib stays engine-agnostic); see docs/DEMO-CITY3D-DESIGN.md "Cars".
public readonly struct ReconstructedVehicle
{
    public ReconstructedVehicle(
        VehicleHandle handle, float x, float y, float z, float yawRad, float pitchRad,
        float length, float width, float speed)
    {
        Handle = handle;
        X = x; Y = y; Z = z;
        YawRad = yawRad; PitchRad = pitchRad;
        Length = length; Width = width;
        Speed = speed;
    }

    public VehicleHandle Handle { get; }
    public float X { get; }
    public float Y { get; }
    public float Z { get; }
    public float YawRad { get; }
    public float PitchRad { get; }
    public float Length { get; }
    public float Width { get; }
    public float Speed { get; }
}

// docs/DEMO-CITY3D-DESIGN.md "Data path" + VIEWER-KINEMATIC-SMOOTHING-DESIGN.md §2.1 -- the per-render-frame
// reconstruction pipeline. One DrClock + one shared KinematicReconstructor across all vehicles (the facade is
// per-handle INTERNALLY -- Sim.Viewer.Motion.KinematicReconstructor keys its KinematicHeading + look-ahead
// state by VehicleHandle), driven purely off an IReplicationSource + an ILaneShapeSource. Neither type here is
// transport- or origin-specific, so the SAME Reconstructor instance/logic runs unchanged whether `lanes`
// is a local Sim.Core.NetworkLaneSource or a wire-fed ReplicationLaneShapeSource (T1.2 condition 3), and
// whether `source` is an in-process InMemoryReplicationBus.Source or (later) a DDS IReplicationSource.
//
// VIEWER-KINEMATIC-SMOOTHING §2.1 (S2): the older DrPoseSmoother is retired; every vehicle render frame now
// runs through the shared KinematicReconstructor facade (straddle-aware continuous front + spatial look-ahead
// + no-slip rear-axle KinematicHeading drag). `CoarseFeed=true` because the viewer is a sparse (~1-3 Hz) DR
// consumer -- it enables the junction-turn-straddle discriminator. The facade returns the vehicle geometric
// CENTER (half a length behind the front), which is exactly what City3D's center-anchored box wants.
public sealed class Reconstructor
{
    private readonly DrClock _clock = new();
    private readonly KinematicReconstructor _recon = new() { CoarseFeed = true };
    private readonly List<ReconstructedVehicle> _scratch = new();
    private readonly Stopwatch _wall = Stopwatch.StartNew();
    private double _lastWallSec = -1.0;

    // Call once per render frame. Mirrors the design's data-path recipe exactly:
    //   source.Pump() -> DrClock.Pump(newest) -> per vehicle: DrClock.Resolve -> KinematicReconstructor.Resolve
    // `delaySeconds` is the playout delay (design "Playout delay": a stable manual knob, ~0.3-0.5s).
    // `frameDtOverride`: normally null -- the real per-frame dt is measured off the wall Stopwatch and the
    // DrClock's own Stopwatch-driven render clock (`Pump`+`Resolve`) times the playout, exactly as a live 60 Hz
    // viewer needs. When a FIXED dt is passed the reconstruction is switched to a DETERMINISTIC clock: the per-
    // frame dt is that fixed value AND the DrClock query instant is derived purely from the packet stream
    // (`LatestVehicleSampleTime - delay` via `ResolveAt`, the same deterministic seam IgBridge uses), never the
    // Stopwatch. That makes the whole pass a pure function of (packet stream, dt schedule) -- the only way to
    // assert determinism (identical transforms across two runs) without a wall clock to diverge.
    public IReadOnlyList<ReconstructedVehicle> Reconstruct(
        IReplicationSource source, ILaneShapeSource lanes, double delaySeconds, float? frameDtOverride = null)
    {
        source.Pump();

        float frameDt;
        double? deterministicSampleT = null;
        if (frameDtOverride is { } fixedDt)
        {
            frameDt = fixedDt;
            // Deterministic query instant: `delay` behind the newest sample seen, from the packet stream alone
            // (no Stopwatch). Analogous to IgBridge's `tau`. Null (no samples yet) -> nothing resolves anyway.
            deterministicSampleT = source.LatestVehicleSampleTime is { } newest ? newest - delaySeconds : null;
        }
        else
        {
            _clock.Pump(source.LatestVehicleSampleTime);
            var nowWall = _wall.Elapsed.TotalSeconds;
            frameDt = _lastWallSec >= 0.0 ? (float)(nowWall - _lastWallSec) : 0f;
            _lastWallSec = nowWall;
        }

        _scratch.Clear();

        Span<int> upcomingBuf = stackalloc int[UpcomingLanes.Count];

        foreach (var kv in source.History)
        {
            var handle = kv.Key;
            var history = kv.Value;
            if (history.Count == 0 || !source.Dims.TryGetValue(handle, out var dims))
            {
                continue;
            }

            DrClock.Resolved resolved;
            try
            {
                resolved = deterministicSampleT is { } sampleT
                    ? _clock.ResolveAt(history, sampleT, lanes)
                    : _clock.Resolve(history, delaySeconds, lanes);
            }
            catch (KeyNotFoundException)
            {
                // Geometry for this vehicle's lane hasn't arrived on this source yet (only possible on a
                // remote/wire-fed lane source with real transport latency) -- skip this vehicle this frame
                // rather than throw out of the whole reconstruction pass.
                continue;
            }

            // §2.1 (S2): one shared reconstruction for BOTH straddle and non-straddle. A lateral straddle
            // (lane change or junction lane-cross) is NO LONGER skipped -- the facade resolves both bracketing
            // states and Cartesian-lerps a continuous front, then runs the look-ahead + no-slip rear-axle
            // KinematicHeading drag, returning the vehicle geometric CENTER. Ok=false == the old inline
            // `continue` (missing/degenerate geometry this frame) -> skip the vehicle this frame.
            var r = _recon.Resolve(handle, resolved, lanes, (dims.Length, dims.Width), frameDt);
            if (!r.Ok)
            {
                continue;
            }

            // Feed the box the CENTER (not the front): City3D's box is centered on the point, so before this
            // fix it drew the body at the front reference (~half a length too far forward). r.CenterX/Y is the
            // true geometric center KinematicHeading tows behind the front, which lands the box correctly.
            var (gx, gy, gz) = CoordinateTransform.SumoToGodot(r.CenterX, r.CenterY, r.Z);
            var yawRad = CoordinateTransform.NaviDegToGodotYawRad(r.HeadingDeg);

            // Pitch (tilt on ramps) still comes from the lane's own z-gradient along travel, walked down the
            // primary bracket's current + upcoming lane window (unchanged from before the facade swap).
            var state = resolved.State with { Length = dims.Length, Width = dims.Width };
            var n = resolved.Upcoming.CopyTo(upcomingBuf);
            if (n == 0)
            {
                upcomingBuf[0] = state.LaneHandle;
                n = 1;
            }

            var pitchRad = ComputePitchRad(lanes, state, upcomingBuf[..n]);

            _scratch.Add(new ReconstructedVehicle(
                handle, gx, gy, gz, yawRad, pitchRad, dims.Length, dims.Width, (float)r.Speed));
        }

        return _scratch;
    }

    // docs/DEMO-CITY3D-DESIGN.md "Cars": "pitch = z-gradient along travel (tilt on ramps)". Approximated as
    // the elevation slope 1 m ahead of the vehicle's current arc position, walked along its own upcoming
    // lane window (so it correctly crosses into the next lane near an edge boundary); 0 when the lane
    // source carries no elevation at all (a 2-D net, or any wire-fed ILaneShapeSource -- LaneShapeZ is
    // always null on the wire today, see ReplicationLaneShapeSource).
    private static float ComputePitchRad(ILaneShapeSource lanes, DrState state, ReadOnlySpan<int> path)
    {
        if (path.IsEmpty || lanes.LaneShapeZ(state.LaneHandle) is null)
        {
            return 0f;
        }

        const double stepMetres = 1.0;
        var z0 = WalkElevation(lanes, path, state.Pos);
        var z1 = WalkElevation(lanes, path, state.Pos + stepMetres);
        return (float)Math.Atan2(z1 - z0, stepMetres);
    }

    // Walk `arc` metres forward along `path` (current lane first, mirrors PoseResolver's own SampleForward
    // walk) and sample elevation on whichever lane the walk lands on; 0 if that lane has no elevation data.
    private static double WalkElevation(ILaneShapeSource lanes, ReadOnlySpan<int> path, double arc)
    {
        var remaining = arc < 0.0 ? 0.0 : arc;
        for (var i = 0; i < path.Length; i++)
        {
            var h = path[i];
            var len = lanes.LaneLength(h);
            if (remaining <= len || i == path.Length - 1)
            {
                var shapeZ = lanes.LaneShapeZ(h);
                return shapeZ is null ? 0.0 : LaneGeometry.ElevationAtOffset(lanes.LaneShape(h), shapeZ, remaining);
            }

            remaining -= len;
        }

        return 0.0;
    }
}
