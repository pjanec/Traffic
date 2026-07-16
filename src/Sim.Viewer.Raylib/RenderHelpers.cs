using Sim.Core;
using Sim.Replication;
using Sim.Viewer.Motion;

namespace Sim.Viewer.Raylib;

// docs/SUMOSHARP-PACKAGING-DESIGN.md D5/§5 (P3.3): the shared per-frame draw-pose builders, moved
// VERBATIM out of Sim.Viewer/Program.cs so the packable Sim.Viewer.Raylib library carries the logic a
// consumer needs to turn a decoded DDS/dead-reckoned stream (or a local authoritative snapshot) into
// the Renderer.DrVehicleDraw list the render loop draws -- not just the draw calls themselves.
public static class RenderHelpers
{
    // P3 refactor: the DDS-pump + dead-reckoned-pose-resolve step shared by `--mode loopback` and `--mode
    // remote` -- identical math to the block this replaces in each (DrClock.Resolve + PoseResolver.Resolve(dt=0)
    // + the extrapolation-only smoothing low-pass), just called from one place. Mutates `vehicleDraws` (cleared
    // and repopulated) and `smoother` (the DrPoseSmoother's per-vehicle running state, Sim.Viewer.Motion) in
    // place.
    public static void PumpAndBuildVehicleDraws(
        DdsSubscriber subscriber,
        DrClock drClock,
        float delaySeconds,
        bool smooth,
        FrameStats frameStats,
        DrPoseSmoother smoother,
        List<Renderer.DrVehicleDraw> vehicleDraws,
        bool paused,
        VehicleHandle? traceHandle = null)
    {
        subscriber.Pump();
        drClock.Pump(subscriber.LatestVehicleSampleTime, hold: paused);

        vehicleDraws.Clear();
        var geoSource = new DdsGeometryLaneSource(subscriber.Geometry);
        Span<int> upcomingScratch = stackalloc int[UpcomingLanes.Count];

        foreach (var (handle, history) in subscriber.History)
        {
            if (history.Count == 0)
            {
                continue;
            }

            var resolved = drClock.Resolve(history, delaySeconds, geoSource);
            var (length, width) = subscriber.Dims.TryGetValue(handle, out var dims) ? dims : (5.0f, 1.8f);

            // Lateral straddle (lane change, SUMOSHARP-LANE-CHANGE-SMOOTHING-DESIGN.md §4.2): resolve BOTH
            // bracketing packets on their own lanes and Cartesian-lerp the two world poses -- the straight
            // chord between two sibling lanes IS the lane-change diagonal (unlike a downstream/junction
            // straddle, which walks the curved geometry via ArcInWindow instead and never reaches here).
            double px, py;
            float pdeg;
            if (resolved.IsLateralStraddle)
            {
                var stateA = resolved.State with { Length = length, Width = width };
                var upA = resolved.Upcoming.CopyTo(upcomingScratch);
                var poseA = PoseResolver.Resolve(
                    geoSource, stateA, upcomingScratch[..upA], default, 0.0, RenderRealism.ChordHeading);

                var stateB = resolved.SecondState!.Value with { Length = length, Width = width };
                var upB = resolved.SecondUpcoming.CopyTo(upcomingScratch); // reuse scratch: poseA already computed
                var poseB = PoseResolver.Resolve(
                    geoSource, stateB, upcomingScratch[..upB], default, 0.0, RenderRealism.ChordHeading);

                var f = resolved.Blend;
                var dxw = poseB.X - poseA.X;
                var dyw = poseB.Y - poseA.Y;
                var chordLen = Math.Sqrt(dxw * dxw + dyw * dyw);

                // Sanity guard (design §4.2 point 4): an implausibly large gap for ONE PACKET INTERVAL (handle
                // reuse, despawn/respawn, or a genuine teleport) snaps to `poseB` instead of drawing a long
                // diagonal across the map. Gated on THIS bracket's own real span (resolved.PacketSpan), not the
                // smoothed average sample interval -- a lane-change bracket's actual gap can run well above the
                // EMA average (observed on 12-overtake: avgint~1.2s vs. this pair's real ~2.9s), so using the
                // average would under-estimate a perfectly normal slow-sampled slide's chord length and wrongly
                // snap it (the ~3.2 m single-frame jump the T2/T3 numeric bars rule out).
                var maxSlide = Math.Max(3.0 * width /* ~3 lane widths */,
                    resolved.State.Speed * Math.Max(resolved.PacketSpan, 0.1) * 1.5);

                if (chordLen > maxSlide)
                {
                    px = poseB.X;
                    py = poseB.Y;
                    pdeg = poseB.HeadingDeg;
                }
                else
                {
                    px = poseA.X + dxw * f;
                    py = poseA.Y + dyw * f;
                    // Base lane heading; the motion-tilt in the smoothing block below leans it toward the actual
                    // slide direction -- uniform with the return change, which never enters this straddle branch.
                    pdeg = LerpAngleDeg(poseA.HeadingDeg, poseB.HeadingDeg, f);
                }
            }
            else
            {
                var state = resolved.State with { Length = length, Width = width };
                var upCount = resolved.Upcoming.CopyTo(upcomingScratch);
                var pose = PoseResolver.Resolve(
                    geoSource, state, upcomingScratch[..upCount], default, 0.0, RenderRealism.ChordHeading);

                px = pose.X;
                py = pose.Y;
                pdeg = pose.HeadingDeg;
            }

            var (_, avgFrame, _) = frameStats.Compute();
            var frameDt = avgFrame > 0f ? avgFrame : 1f / 60f;

            var (sx, sy, sdeg) = smoother.Smooth(handle, px, py, pdeg, resolved.State.Speed, frameDt);
            px = sx;
            py = sy;
            pdeg = sdeg;

            // Diagnostic (opt-in via --trace-veh): dump the FINAL drawn pose for the traced vehicle -- AFTER the
            // heading low-pass + position filter, i.e. exactly what vehicleDraws renders (placed here, not before
            // the smoothing block, so the trace observes the smoothing) -- plus the extrapolated flag, resolved
            // lane, and effective playout delay, to compare against the AUTHTRACE ground truth.
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
        Dictionary<VehicleHandle, float> headingPrev, Dictionary<VehicleHandle, float> headingCur,
        float frameDtWall)
    {
        outDraws.Clear();
        headingCur.Clear(); // rebuilt from current vehicles only -> prunes despawned, stays bounded

        var span = cur.Time - prev.Time;
        var interp = smooth && span > 1e-9 && prev.Count > 0 && !ReferenceEquals(prev, cur);

        // Heading low-pass coefficient (this frame). A junction's internal lane is a coarse ~5-point arc
        // (netconvert internal-link-detail=5), so the raw/interpolated heading rotates in ~5 discrete bursts; a
        // ~0.18s low-pass on the RENDER heading spreads them into one continuous rotation. Straights (constant
        // heading) converge instantly -> no lag; only actual rotation is smoothed.
        var headAlpha = smooth ? 1f - MathF.Exp(-frameDtWall / 0.25f) : 1f;

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

            // Low-pass the render heading toward `deg` from last frame's smoothed value (shortest arc). A big
            // jump (respawn / opposite-direction reuse of a handle) snaps rather than spins.
            if (smooth && headingPrev.TryGetValue(handle, out var prevDeg))
            {
                var d = ((deg - prevDeg + 540f) % 360f) - 180f;
                deg = MathF.Abs(d) > 100f ? deg : (prevDeg + d * headAlpha + 360f) % 360f;
            }

            headingCur[handle] = deg;
            outDraws.Add(new Renderer.DrVehicleDraw(fx, fy, deg, cur.Length[i], cur.Width[i], spd, handle));
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
