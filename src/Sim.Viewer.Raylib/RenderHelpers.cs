using Sim.Core;
using Sim.Replication;
using Sim.Replication.Dds;
using Sim.Viewer.Motion;

namespace Sim.Viewer.Raylib;

// docs/SUMOSHARP-PACKAGING-DESIGN.md D5/§5 (P3.3): the shared per-frame draw-pose builders, moved
// VERBATIM out of Sim.Viewer/Program.cs so the packable Sim.Viewer.Raylib library carries the logic a
// consumer needs to turn a decoded DDS/dead-reckoned stream (or a local authoritative snapshot) into
// the Renderer.DrVehicleDraw list the render loop draws -- not just the draw calls themselves.
public static class RenderHelpers
{
    // P3 refactor / VIEWER-KINEMATIC-SMOOTHING §2.2 (T1.1): the DDS-pump + dead-reckoned-pose-resolve step
    // shared by `--mode loopback` and `--mode remote`. The per-vehicle straddle-lerp + look-ahead +
    // KinematicHeading smoothing that used to live inline here (and the older per-vehicle pose smoother) is one
    // shared `KinematicReconstructor.Resolve(...)` facade (Sim.Viewer.Motion) -- the same pipeline IgBridge and
    // City3D use, so all consumers move as one. `CoarseFeed=true` is set on the passed `recon` by the caller
    // (the viewers are ~1-3 Hz DR consumers). Mutates `vehicleDraws` (cleared and repopulated) and `recon` (its
    // per-vehicle kinematic + look-ahead state) in place.
    public static void PumpAndBuildVehicleDraws(
        DdsSubscriber subscriber,
        DrClock drClock,
        float delaySeconds,
        bool smooth,
        FrameStats frameStats,
        KinematicReconstructor recon,
        List<Renderer.DrVehicleDraw> vehicleDraws,
        bool paused,
        VehicleHandle? traceHandle = null)
    {
        subscriber.Pump();
        drClock.Pump(subscriber.LatestVehicleSampleTime, hold: paused);

        vehicleDraws.Clear();
        var geoSource = new DdsGeometryLaneSource(subscriber.Geometry);

        var (_, avgFrame, _) = frameStats.Compute();
        var frameDt = avgFrame > 0f ? avgFrame : 1f / 60f;

        foreach (var (handle, history) in subscriber.History)
        {
            if (history.Count == 0)
            {
                continue;
            }

            var resolved = drClock.Resolve(history, delaySeconds, geoSource);
            var (length, width) = subscriber.Dims.TryGetValue(handle, out var dims) ? dims : (5.0f, 1.8f);

            // One shared reconstruction: straddle-aware continuous front + spatial look-ahead + no-slip
            // rear-axle KinematicHeading drag, all inside the facade (junction vs lane-change straddle is
            // discriminated by CoarseFeed internally). Ok=false == the old `continue` (missing/degenerate
            // geometry this frame) -> skip the vehicle.
            var result = recon.Resolve(handle, resolved, geoSource, (length, width), frameDt);
            if (!result.Ok)
            {
                continue;
            }

            // The Raylib box is FRONT-anchored (Renderer.DrawVehicleList: the pose point is the rectangle's
            // (length, width/2) origin, so the body trails back from it), so feed it the SMOOTHED FRONT the
            // kinematic tracker produced -- not the geometric center -- to keep the drawn footprint true.
            var px = result.SmoothedFrontX;
            var py = result.SmoothedFrontY;
            var pdeg = result.HeadingDeg;

            // Diagnostic (opt-in via --trace-veh): dump the FINAL drawn pose for the traced vehicle -- exactly
            // what vehicleDraws renders (placed after the reconstruction so the trace observes the smoothing) --
            // plus the extrapolated flag, resolved lane, and effective playout delay, vs the AUTHTRACE truth.
            if (traceHandle is { } th && handle == th)
            {
                Console.WriteLine($"DRTRACE rsim={drClock.RenderSim:F2} x={px:F2} y={py:F2} deg={pdeg:F1} " +
                    $"laneH={resolved.State.LaneHandle} pos={resolved.State.Pos:F2} extrap={resolved.Extrapolated} " +
                    $"spd={resolved.State.Speed:F1} " +
                    $"delay={drClock.EffectiveDelay:F3} avgint={drClock.AvgSampleInterval:F3} hist={history.Count}");
            }

            vehicleDraws.Add(new Renderer.DrVehicleDraw(px, py, pdeg, length, width, resolved.State.Speed));
        }
    }

    // P3: remote has no local NetworkModel to size its initial camera fit from (unlike loopback's
    // host.MinX/MinY/MaxX/MaxY) -- computed once, the first time geometry arrives, from the received lane
    // polylines' own bounding box. Falls back to a small placeholder box if called with no geometry yet (should
    // not happen given the caller's `Count > 0` guard, but degrades safely rather than throwing on empty input).
    public static (double MinX, double MinY, double MaxX, double MaxY) ComputeGeometryBounds(
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
    {
        var minX = double.PositiveInfinity;
        var minY = double.PositiveInfinity;
        var maxX = double.NegativeInfinity;
        var maxY = double.NegativeInfinity;

        foreach (var lane in geometry.Values)
        {
            foreach (var (x, y) in lane.Points)
            {
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }
        }

        if (double.IsPositiveInfinity(minX))
        {
            return (-50, -50, 50, 50);
        }

        return (minX, minY, maxX, maxY);
    }

    // Render-behind interpolation for `--mode local`: fill `outDraws` with each current vehicle's draw pose,
    // blended from the matching vehicle in the previous authoritative frame toward the current one by the clock
    // fraction `a`. O(n) via a reused handle->index map (SimulationSnapshot.TryGetVehicle is a linear scan, so
    // per-vehicle lookups would be O(n^2) at 10k). Reuses `outDraws`/`prevIndex` (Clear keeps capacity) so a
    // warmed steady state allocates nothing. Falls back to the raw current frame when there's no distinct prior
    // frame to blend from (first frame / just after a restart / smoothing off).
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.2): no more evac-specific `Fear` param -- each draw pose
    // carries only its generic VehicleHandle (Commit 1's DrVehicleDraw.Handle) so a render overlay (e.g. the
    // evac layer's fear overpaint) can key its OWN per-vehicle state off it, without this generic builder
    // knowing why. This function is now domain-agnostic.
    public static void BuildLocalVehicleDraws(
        SimulationSnapshot cur, SimulationSnapshot prev, double renderClock, bool smooth,
        List<Renderer.DrVehicleDraw> outDraws, Dictionary<VehicleHandle, int> prevIndex,
        KinematicReconstructor recon, float frameDtWall)
    {
        outDraws.Clear();

        var span = cur.Time - prev.Time;
        var interp = smooth && span > 1e-9 && prev.Count > 0 && !ReferenceEquals(prev, cur);

        var a = 0f;
        if (interp)
        {
            // [0, 1.25]: >1 is the small bounded extrapolation the render clock uses to bridge a late snapshot
            // (see the clock in RunLocal) -- lerp beyond `cur` predicts slightly ahead instead of freezing.
            a = (float)((renderClock - prev.Time) / span);
            if (a < 0f) a = 0f;
            else if (a > 1.25f) a = 1.25f;

            prevIndex.Clear();
            for (var j = 0; j < prev.Count; j++)
            {
                prevIndex[prev.Handles[j]] = j;
            }
        }

        for (var i = 0; i < cur.Count; i++)
        {
            var handle = cur.Handles[i];
            float fx = cur.PosX[i], fy = cur.PosY[i], deg = cur.Angle[i];
            double spd = cur.SpeedExact[i];

            // Match by handle (spawn/despawn shuffles column indices between frames). A vehicle absent from the
            // previous frame (just departed) is drawn at its raw current pose.
            if (interp && prevIndex.TryGetValue(handle, out var j))
            {
                fx = prev.PosX[j] + (cur.PosX[i] - prev.PosX[j]) * a;
                fy = prev.PosY[j] + (cur.PosY[i] - prev.PosY[j]) * a;
                deg = LerpAngleDeg(prev.Angle[j], cur.Angle[i], a);
                spd = prev.SpeedExact[j] + (cur.SpeedExact[i] - prev.SpeedExact[j]) * a;
            }

            float length = cur.Length[i], width = cur.Width[i];

            // VIEWER-KINEMATIC-SMOOTHING §1.3 (T1.3): unify `--mode local` onto the SAME kinematic
            // reconstruction the DR viewers use, replacing the old render-heading low-pass. This path has no
            // DrState / upcoming-lane window, so it calls the pose-level entry with predictHeadingDeg=null (no
            // spatial look-ahead) and lateralEvent=false -- a lane change appears here as a perpendicular x,y
            // step that KinematicHeading's step-based detector eases on its own. Fed the INTERPOLATED front
            // (fx,fy,deg,spd); the no-slip rear-axle body + lane-heading low-pass + lane-change ease apply. The
            // Raylib box is FRONT-anchored, so the drawn point is the SMOOTHED FRONT.
            if (smooth)
            {
                var res = recon.ResolveFromFront(handle, fx, fy, deg, spd, (length, width), frameDtWall,
                    lateralEvent: false, predictHeadingDeg: null);
                if (res.Ok)
                {
                    fx = (float)res.SmoothedFrontX;
                    fy = (float)res.SmoothedFrontY;
                    deg = res.HeadingDeg;
                }
            }

            outDraws.Add(new Renderer.DrVehicleDraw(fx, fy, deg, length, width, spd, handle));
        }
    }

    // Interpolate degrees along the shortest arc, so a heading crossing 0/360 (e.g. 350 -> 10) turns the short
    // way -- mirrors SimulationRunner.LerpAngleDeg (kept local; the runner's is internal). Straight lerp of
    // position + shortest-arc lerp of heading renders a smooth turn through a 90-deg junction; at 10 Hz the
    // chord-vs-arc position error between adjacent frames is sub-metre, so long vehicles round corners cleanly.
    private static float LerpAngleDeg(float from, float to, float t)
    {
        var delta = (to - from) % 360f;
        if (delta > 180f) delta -= 360f;
        else if (delta < -180f) delta += 360f;
        var r = (from + delta * t) % 360f;
        return r < 0f ? r + 360f : r;
    }
}
