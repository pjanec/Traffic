using Sim.Core;
using Sim.Replication;

namespace Sim.Viewer.Motion;

// The per-vehicle render-side reconstruction core (docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md §1.1), extracted
// VERBATIM from the block that used to live inline in Sim.IgBridge.IgBridgeSession.EmitVehicles so the 2D/3D
// viewers and the IG feed all inherit one pipeline (fix once, fix all). Given a DrClock.Resolved bracket it
// resolves a CONTINUOUS front pose (straddle-aware Cartesian-lerp of two PoseResolver.Resolve calls, or a
// single resolve), applies the coarse-feed junction-straddle discriminator, computes the anticipatory
// look-ahead heading with its jump-guard + lead-bound, and drives KinematicHeading to a rigid-body pose.
// Stateful per vehicle (the look-ahead jump-guard memory here, the bicycle state inside KinematicHeading);
// deterministic and order-independent per handle (no System.Random). Render-side only -- never touches parity.
public sealed class KinematicReconstructor
{
    private readonly KinematicHeading _kinematic;                              // front smoothing + rear-axle drag (docs §5.3)
    private readonly Dictionary<VehicleHandle, float> _lookAheadPrev = new();  // previously-USED predictor heading (jump guard)

    public KinematicReconstructor(KinematicHeadingParams? kinematics = null)
        => _kinematic = new KinematicHeading(kinematics);

    // Spatial look-ahead (metres ahead on the upcoming lane centerline to aim the front predictor at). 0
    // disables (front follows the reactive current lane heading). Default 0; the IgBridge caller sets 3.0.
    public double LookAheadMeters { get; set; }

    // Effective look-ahead per vehicle = max(LookAheadMeters, LookAheadLengthFactor * length), so a longer
    // vehicle anticipates proportionally further ahead. 0 keeps every vehicle on the flat LookAheadMeters.
    public double LookAheadLengthFactor { get; set; } = 0.5;

    // Max ANGLE the look-ahead heading may lead the reactive (raw) lane heading by before it is rejected
    // (catches gradual cross-junction drift the per-frame jump guard misses -- the coarse-feed "dance").
    public float MaxAnticipationLeadDeg { get; set; } = 70f;

    // COARSE-FEED ONLY: a straddle whose two lane poses diverge in heading by more than this is a junction
    // TURN sampled across the bracket and is NOT absorbed as a lane change; a near-parallel straddle still is.
    public float MaxStraddleLaneChangeHeadingDeg { get; set; } = 20f;

    // Set when the feed is decimated (feedHz < core rate). Enables the junction-turn-straddle discriminator.
    public bool CoarseFeed { get; set; }

    // Opt-in: apply the junction-turn-straddle discriminator at the DENSE feed too (changes the v5 baseline).
    public bool AlwaysSplitJunctionStraddle { get; set; }

    // Max plausible frame-to-frame change (deg/s) of the look-ahead predictor heading; a bigger jump is a
    // spurious cross-junction resolve and is rejected (falls back to the lane heading).
    private const float LookAheadMaxJumpDegPerSec = 250f;

    public void Clear()
    {
        _lookAheadPrev.Clear();
        _kinematic.Clear();
    }

    // Full pipeline: resolve a continuous front pose from the DrClock bracket, run the look-ahead predictor
    // + guards, and drive KinematicHeading. `frameDt` is the caller's real elapsed dt for this vehicle (>= one
    // emit step). Returns Ok=false (the caller skips this vehicle this frame) when a PoseResolver resolve is
    // unavailable, matching the old inline `continue` paths.
    public KinematicReconResult Resolve(
        VehicleHandle handle, in DrClock.Resolved resolved, ILaneShapeSource lanes,
        (double Length, double Width) dims, float frameDt)
    {
        Span<int> upcoming = stackalloc int[UpcomingLanes.Count];

        // Resolve a CONTINUOUS front pose. A lateral straddle (junction lane-cross or lane change) is NOT
        // skipped -- skipping punches a hole in the emitted stream and (fed a constant dt) desyncs the
        // kinematic state -> tail wobble. Instead resolve both bracketing states on their own lanes and
        // Cartesian-lerp (the shipped lane-change reconstruction), so the front path is unbroken.
        double frontX, frontY, frontZ;
        float laneHeading;
        double speed;
        var lateralEvent = resolved.IsLateralStraddle;
        if (resolved.IsLateralStraddle && resolved.SecondState is { } stateBraw)
        {
            if (!TryResolveFront(lanes, resolved.State, resolved.Upcoming, dims, upcoming, out var pa) ||
                !TryResolveFront(lanes, stateBraw, resolved.SecondUpcoming, dims, upcoming, out var pb))
            {
                return KinematicReconResult.Skipped;
            }

            var f = resolved.Blend;
            frontX = pa.X + (pb.X - pa.X) * f;
            frontY = pa.Y + (pb.Y - pa.Y) * f;
            frontZ = pa.Z + (pb.Z - pa.Z) * f;
            laneHeading = LerpHeadingDeg(pa.HeadingDeg, pb.HeadingDeg, f);
            speed = resolved.State.Speed + (stateBraw.Speed - resolved.State.Speed) * f;

            // Only a NEAR-PARALLEL straddle is a real lane change whose lateral snap should be absorbed into
            // the decaying error E. A straddle whose two lane poses diverge in heading is a JUNCTION TURN
            // sampled across the bracket; absorbing its lateral ARC motion as a lane-change error is what makes
            // the front ride between lanes for seconds after a turn on a coarse feed. At a dense feed the two
            // bracketing poses are ~0.1 s apart (a few degrees even mid-turn), so this stays below the
            // threshold and the default is byte-identical; it only trips when a coarse feed spans a large turn.
            if (CoarseFeed || AlwaysSplitJunctionStraddle)
            {
                var straddleHeadingDiff = Math.Abs(((pb.HeadingDeg - pa.HeadingDeg + 540f) % 360f) - 180f);
                lateralEvent = straddleHeadingDiff < MaxStraddleLaneChangeHeadingDeg;
            }
        }
        else
        {
            if (!TryResolveFront(lanes, resolved.State, resolved.Upcoming, dims, upcoming, out var pose))
            {
                return KinematicReconResult.Skipped;
            }

            frontX = pose.X;
            frontY = pose.Y;
            frontZ = pose.Z;
            laneHeading = pose.HeadingDeg;
            speed = resolved.State.Speed;
        }

        // Spatial look-ahead heading (aim the front down the upcoming connecting lane, not the current lane) so
        // junction turn-ins hit the connecting-lane centerline instead of lagging off it. Guard against a bad
        // resolve: reject the look-ahead whenever it JUMPS from the previously-used predictor heading faster
        // than a plausible yaw rate (kills transient spurious excursions while passing the smooth turn ramp),
        // or when it DIVERGES from the reactive lane heading past the lead bound (gradual drift). On reject we
        // fall back to the lane heading (and remember that), so a sustained bad reading stays rejected.
        var effLookAhead = Math.Max(LookAheadMeters, LookAheadLengthFactor * dims.Length);
        float? predictHeading = null;
        if (effLookAhead > 0.0
            && TryLookAheadHeading(lanes, resolved.State, resolved.Upcoming, dims, frontX, frontY, effLookAhead, upcoming, out var lah))
        {
            var prevUsed = _lookAheadPrev.TryGetValue(handle, out var pv) ? pv : laneHeading;
            var jump = Math.Abs(((lah - prevUsed + 540f) % 360f) - 180f);
            var lead = Math.Abs(((lah - laneHeading + 540f) % 360f) - 180f);
            if (jump <= LookAheadMaxJumpDegPerSec * Math.Max(frameDt, 1e-3f)
                && lead <= MaxAnticipationLeadDeg)
            {
                predictHeading = lah;
            }
        }

        _lookAheadPrev[handle] = predictHeading ?? laneHeading;

        return ResolveFromFront(handle, frontX, frontY, laneHeading, speed, dims, frameDt,
            lateralEvent, predictHeading, frontZ);
    }

    // Tail of the pipeline: drive KinematicHeading from an already-resolved front pose. For callers that own
    // the front reference themselves (2D/3D viewers with predictHeadingDeg=null); Resolve delegates here.
    public KinematicReconResult ResolveFromFront(
        VehicleHandle handle, double frontX, double frontY, float laneHeadingDeg, double speed,
        (double Length, double Width) dims, float frameDt, bool lateralEvent, float? predictHeadingDeg,
        double frontZ = 0.0)
    {
        // Kinematic rear-axle drag (§5.3): the front reference tows a no-slip rear axle so the body pivots at
        // the rear like a real car; it also critically-damps the front POSITION (removes faceted-polyline
        // kinks). The vehicle CENTER (what the IG's models pivot on) is half a length behind the front bumper.
        var kp = _kinematic.Update(handle, frontX, frontY, laneHeadingDeg, speed, dims.Length, frameDt,
            lateralEvent, predictHeadingDeg);
        var predictUsed = predictHeadingDeg ?? laneHeadingDeg;
        return new KinematicReconResult(
            ok: true,
            centerX: kp.CenterX, centerY: kp.CenterY, z: frontZ, headingDeg: kp.HeadingDeg,
            frontX: frontX, frontY: frontY, laneHeadingDeg: laneHeadingDeg, speed: speed,
            smoothedFrontX: kp.FrontX, smoothedFrontY: kp.FrontY, predictHeadingUsed: predictUsed);
    }

    // Resolve one bracketing state to a front pose via PoseResolver (ChordHeading). Returns false if the lane
    // geometry isn't available.
    private static bool TryResolveFront(
        ILaneShapeSource lanes, DrState rawState, UpcomingLanes upcomingLanes,
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
                lanes, state, scratch[..n], ReadOnlySpan<int>.Empty, dt: 0.0, RenderRealism.ChordHeading);
            return true;
        }
        catch (KeyNotFoundException)
        {
            pose = default;
            return false;
        }
    }

    // Look-ahead heading: resolve a "front" pose with the front ARC position advanced by lookAheadMeters --
    // PoseResolver walks the same upcoming lanes, so past the junction this lands on the connecting-lane
    // centerline. The chord from the real front to that point is the anticipatory predictor direction. Returns
    // false (caller falls back to the lane heading) if it can't be resolved or the point is degenerate.
    private static bool TryLookAheadHeading(
        ILaneShapeSource lanes, DrState rawState, UpcomingLanes upcomingLanes,
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
                lanes, state, scratch[..n], ReadOnlySpan<int>.Empty, dt: 0.0, RenderRealism.ChordHeading);
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
}

// One reconstructed rigid-body result, exposing every value a caller's emit + debug pass needs. `Ok` is false
// when the underlying PoseResolver resolve was unavailable (a degenerate/missing-geometry frame) and the
// caller should skip the vehicle this frame. `Z` is the resolved front z (lane-surface ground elevation).
public readonly struct KinematicReconResult
{
    public KinematicReconResult(
        bool ok, double centerX, double centerY, double z, float headingDeg,
        double frontX, double frontY, float laneHeadingDeg, double speed,
        double smoothedFrontX, double smoothedFrontY, float predictHeadingUsed)
    {
        Ok = ok;
        CenterX = centerX;
        CenterY = centerY;
        Z = z;
        HeadingDeg = headingDeg;
        FrontX = frontX;
        FrontY = frontY;
        LaneHeadingDeg = laneHeadingDeg;
        Speed = speed;
        SmoothedFrontX = smoothedFrontX;
        SmoothedFrontY = smoothedFrontY;
        PredictHeadingUsed = predictHeadingUsed;
    }

    // Skip sentinel: a frame with no resolvable front pose (old inline `continue`).
    public static KinematicReconResult Skipped => default;

    public bool Ok { get; }

    // Emit surface: the vehicle geometric center (what the IG's models pivot on), the ground z, and the body
    // heading (navi-degrees).
    public double CenterX { get; }
    public double CenterY { get; }
    public double Z { get; }
    public float HeadingDeg { get; }

    // Debug/diagnostic surface: the raw resolved front (prX/prY), its lane/chord heading (prH), the resolved
    // speed, the smoothed front the kinematic tracker produced (smX/smY), and the predictor heading actually
    // used (laH == look-ahead heading when accepted, else the lane heading).
    public double FrontX { get; }
    public double FrontY { get; }
    public float LaneHeadingDeg { get; }
    public double Speed { get; }
    public double SmoothedFrontX { get; }
    public double SmoothedFrontY { get; }
    public float PredictHeadingUsed { get; }
}
