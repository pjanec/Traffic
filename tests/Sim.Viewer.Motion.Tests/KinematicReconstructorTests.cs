using System;
using System.Collections.Generic;
using Sim.Core;
using Sim.Replication;
using Sim.Viewer.Motion;
using Xunit;

namespace Sim.Viewer.Motion.Tests;

// docs/VIEWER-KINEMATIC-SMOOTHING-TASKS.md T0.3 (facade unit tests) + T3.1 (render-rate / stutter robustness).
// These drive the shared render-smoothing facade `KinematicReconstructor` through its PUBLIC API and assert the
// real numeric behaviour the viewers (IgBridge, Raylib 2D, City3D) depend on:
//   * a straight track resolves a center exactly half a length behind the front, stable heading, no drift;
//   * a stopped vehicle (speed < HoldSpeed) does not creep and holds its heading;
//   * a one-frame perpendicular lane-change snap is NOT emitted as a jump -- it eases over multiple frames;
//   * the tau-based gains adapt so the motion stays smooth (bounded per-frame yaw-accel, center still L/2 behind
//     the front) at 30 / 60 / 144 Hz;
//   * an injected long (stutter) frame produces no reseed spike and no NaN;
//   * the coarse-feed junction-straddle discriminator flips a wide-heading straddle out of the lane-change
//     (absorb) path while leaving a near-parallel straddle absorbed -- observable through the emitted path.
//
// Pose-level cases use ResolveFromFront (no lane geometry needed); the straddle-classification case builds a
// minimal in-memory ILaneShapeSource and drives the full DrClock.Resolved -> Resolve path. The end-to-end
// junction/lane-change behaviour on real SUMO nets is additionally covered by CityLib.Tests/ReconstructorS2Tests.
public sealed class KinematicReconstructorTests
{
    private const double Len = 4.5;   // m
    private const double Wid = 1.8;   // m
    private const double V = 10.0;    // m/s
    private static (double Length, double Width) Dims => (Len, Wid);

    private static VehicleHandle H(int id) => new((uint)id, 0);

    // Navi-degrees (0 = north, clockwise) -> unit vector, identical to KinematicHeading's convention.
    private static (double X, double Y) Dir(double naviDeg)
    {
        var m = (90.0 - naviDeg) * Math.PI / 180.0;
        return (Math.Cos(m), Math.Sin(m));
    }

    private static float NaviFromVector(double dx, double dy)
    {
        var deg = 90.0 - Math.Atan2(dy, dx) * 180.0 / Math.PI;
        deg %= 360.0;
        if (deg < 0.0) deg += 360.0;
        return (float)deg;
    }

    // Shortest signed angular difference a - b in degrees, in (-180, 180].
    private static double AngDiff(double a, double b) => ((a - b + 540.0) % 360.0) - 180.0;

    // ---------------------------------------------------------------------------------------------------------
    // STRAIGHT: constant-speed front due east. The center settles exactly half a length behind the front along
    // the heading, the heading holds the lane direction, and there is no lateral drift.
    // ---------------------------------------------------------------------------------------------------------
    [Fact]
    public void Straight_CenterHalfLengthBehindFront_HeadingStable_NoDrift()
    {
        var recon = new KinematicReconstructor();
        var h = H(1);
        const float dt = 1f / 60f;

        double fx = 0, fy = 0;
        KinematicReconResult r = default;
        var headings = new List<float>();
        for (var i = 0; i < 300; i++)
        {
            r = recon.ResolveFromFront(h, fx, fy, 90f, V, Dims, dt, lateralEvent: false, predictHeadingDeg: null);
            headings.Add(r.HeadingDeg);
            fx += V * dt;
        }

        // Center sits ~L/2 behind the front reference, west of (behind) it, no lateral offset.
        var dist = Math.Sqrt((r.FrontX - r.CenterX) * (r.FrontX - r.CenterX) +
                             (r.FrontY - r.CenterY) * (r.FrontY - r.CenterY));
        Assert.True(Math.Abs(dist - Len / 2) < 0.05, $"center should sit L/2 ({Len / 2:F2} m) behind the front, was {dist:F3} m");
        Assert.True(r.FrontX > r.CenterX, $"center must be behind (west of) the front: front={r.FrontX:F3} center={r.CenterX:F3}");
        Assert.True(Math.Abs(r.CenterY) < 0.01, $"no lateral drift expected on a straight, centerY={r.CenterY:F4}");
        Assert.InRange(r.HeadingDeg, 89.9f, 90.1f);

        // Heading holds: after warmup no frame changes the body heading by more than a hair.
        double maxStep = 0;
        for (var i = 60; i < headings.Count; i++)
        {
            maxStep = Math.Max(maxStep, Math.Abs(AngDiff(headings[i], headings[i - 1])));
        }
        Assert.True(maxStep < 0.01, $"straight heading wobbled {maxStep:F4} deg/frame (expected dead-stable)");
    }

    // ---------------------------------------------------------------------------------------------------------
    // STOP: once moving establishes a heading, feeding speed 0 with a held front must not creep the center and
    // must hold the heading (no spin at a red light).
    // ---------------------------------------------------------------------------------------------------------
    [Fact]
    public void Stop_NoCreep_HeadingHeld()
    {
        var recon = new KinematicReconstructor();
        var h = H(2);
        const float dt = 1f / 60f;

        // Establish steady motion (heading ~90) over several seconds -- long enough that the front smoother's
        // init transient has fully decayed -- then hold the stop at the exact last front fed (no artificial gap).
        double fx = 0, lastFx = 0;
        KinematicReconResult moving = default;
        for (var i = 0; i < 400; i++)
        {
            moving = recon.ResolveFromFront(h, fx, 0.0, 90f, V, Dims, dt, lateralEvent: false, predictHeadingDeg: null);
            lastFx = fx;
            fx += V * dt;
        }
        fx = lastFx; // hold the stop at the last moving front

        // Now STOP: speed 0, front held constant. The lane heading even flips to 270 to prove it is ignored at a
        // standstill (HoldSpeed = 0.5). Collect the center path and heading.
        var heldHeading = moving.HeadingDeg;
        double maxOngoingCreep = 0;   // frame-to-frame motion AFTER the vehicle has settled onto the stop point
        double firstSettle = 0;       // the one-time ease onto the stop point on the moving->stopped boundary
        double totalDrift = 0;        // net displacement across the whole stopped window (continuous creep would grow this)
        double prevX = moving.CenterX, prevY = moving.CenterY;
        double firstStopX = 0, firstStopY = 0;
        KinematicReconResult stopped = default;
        for (var i = 0; i < 120; i++)
        {
            stopped = recon.ResolveFromFront(h, fx, 0.0, 270f, 0.0, Dims, dt, lateralEvent: false, predictHeadingDeg: null);
            var d = Math.Sqrt((stopped.CenterX - prevX) * (stopped.CenterX - prevX) +
                              (stopped.CenterY - prevY) * (stopped.CenterY - prevY));
            if (i == 0) { firstSettle = d; firstStopX = stopped.CenterX; firstStopY = stopped.CenterY; }
            else maxOngoingCreep = Math.Max(maxOngoingCreep, d);
            prevX = stopped.CenterX;
            prevY = stopped.CenterY;
        }
        totalDrift = Math.Sqrt((stopped.CenterX - firstStopX) * (stopped.CenterX - firstStopX) +
                               (stopped.CenterY - firstStopY) * (stopped.CenterY - firstStopY));

        // Once steady and stopped, the center holds: no perceptible per-frame motion and no net drift across the
        // whole 2 s stop. A continuous-creep bug would grow totalDrift without bound.
        Assert.True(firstSettle < 1e-3, $"stop settle {firstSettle:E3} m was larger than a gentle ease onto the point");
        Assert.True(maxOngoingCreep < 1e-4, $"stopped center kept creeping {maxOngoingCreep:E3} m/frame (expected it to hold)");
        Assert.True(totalDrift < 1e-3, $"stopped center drifted {totalDrift:E3} m net over 2 s (expected it to hold)");
        Assert.True(Math.Abs(AngDiff(stopped.HeadingDeg, heldHeading)) < 0.5,
            $"heading drifted {AngDiff(stopped.HeadingDeg, heldHeading):F2} deg at a stop (expected it to hold ~{heldHeading:F1})");
    }

    // ---------------------------------------------------------------------------------------------------------
    // LANE-CHANGE EASE: drive straight, then inject a single-frame ~one-lane-width perpendicular front jump with
    // lateralEvent = true. The emitted center must NOT jump -- it eases across over many frames (bounded per-frame
    // lateral), monotonically, and absorbs the offset over ~1-2 s.
    // ---------------------------------------------------------------------------------------------------------
    [Fact]
    public void LaneChangeEase_PerpendicularSnap_EasesNotJumps()
    {
        var recon = new KinematicReconstructor();
        var h = H(3);
        const float dt = 1f / 60f;
        const double laneWidth = 3.2;
        const int jumpFrame = 120;      // ~2 s in
        const int frames = 120 + 360;   // + 6 s to absorb

        double fx = 0;
        double laneY = 0;               // the lane the front rides on (jumps +laneWidth at jumpFrame)
        var centerY = new List<double>();
        var jumpFrameCenterStep = double.NaN;
        double prevCenterY = 0;
        double maxLateralStep = 0;

        for (var i = 0; i < frames; i++)
        {
            var lateral = false;
            if (i == jumpFrame)
            {
                laneY = laneWidth;      // SUMO changes lane instantly: raw front leaps one lane width sideways
                lateral = true;
            }

            var r = recon.ResolveFromFront(h, fx, laneY, 90f, V, Dims, dt, lateralEvent: lateral, predictHeadingDeg: null);
            fx += V * dt;

            if (i > 0)
            {
                var step = Math.Abs(r.CenterY - prevCenterY);
                if (i >= jumpFrame) maxLateralStep = Math.Max(maxLateralStep, step);
                if (i == jumpFrame) jumpFrameCenterStep = step;
            }
            prevCenterY = r.CenterY;
            centerY.Add(r.CenterY);
        }

        // (1) No jump: the frame carrying the raw 3.2 m sideways snap moves the emitted center only a little.
        Assert.True(jumpFrameCenterStep < 0.5,
            $"emitted center jumped {jumpFrameCenterStep:F3} m on the snap frame (expected it absorbed, raw was {laneWidth} m)");

        // (2) Bounded per-frame lateral throughout the ease -- no single frame snaps a large fraction across.
        Assert.True(maxLateralStep < 0.3, $"per-frame lateral reached {maxLateralStep:F3} m (expected a gentle bounded ease)");

        // (3) Multi-frame ease (not a snap, not a stall): ~1.3 s after the snap it is partway across.
        var midY = centerY[jumpFrame + 78]; // 78 frames ~= 1.3 s at 60 Hz
        Assert.InRange(midY, 0.5, 3.0);

        // (4) Monotonic approach to the new lane, and the offset is (nearly) fully absorbed by the end.
        for (var i = jumpFrame + 1; i < frames; i++)
        {
            Assert.True(centerY[i] >= centerY[i - 1] - 1e-6, $"center Y went backwards at frame {i} ({centerY[i]:F3} < {centerY[i - 1]:F3})");
        }
        Assert.True(centerY[^1] > 0.8 * laneWidth, $"lane change only reached centerY={centerY[^1]:F3} of {laneWidth} m (not absorbed)");
    }

    // ---------------------------------------------------------------------------------------------------------
    // RENDER-RATE ROBUSTNESS (T3.1): the same constant-curvature turn fed at 30 / 60 / 144 Hz. The tau-based
    // gains adapt to frameDt, so at EVERY rate the reconstructed motion is smooth (bounded per-frame yaw-accel),
    // converges to the true turn rate, and keeps the center exactly L/2 behind the front.
    // ---------------------------------------------------------------------------------------------------------
    [Theory]
    [InlineData(30)]
    [InlineData(60)]
    [InlineData(144)]
    public void RenderRate_ConstantCurvatureTurn_IsSmoothAtEveryRate(int hz)
    {
        var recon = new KinematicReconstructor();
        var h = H(100 + hz);
        var dt = 1f / hz;
        const double radius = 50.0;         // gentle constant-radius arc
        var omega = V / radius;             // true (constant) yaw rate, rad/s
        const double seconds = 6.0;
        var steps = (int)(seconds * hz);

        double theta = 0;                   // arc angle traversed
        var headings = new List<float>();
        var behind = new List<double>();    // |front - center| each frame
        for (var i = 0; i < steps; i++)
        {
            // Front on a circle centered at (0, radius): starts at origin heading due east, curves left (north).
            var fx = radius * Math.Sin(theta);
            var fy = radius * (1.0 - Math.Cos(theta));
            var laneHeading = NaviFromVector(Math.Cos(theta), Math.Sin(theta)); // analytic tangent
            var r = recon.ResolveFromFront(h, fx, fy, laneHeading, V, Dims, dt, lateralEvent: false, predictHeadingDeg: null);
            headings.Add(r.HeadingDeg);
            behind.Add(Math.Sqrt((r.FrontX - r.CenterX) * (r.FrontX - r.CenterX) +
                                 (r.FrontY - r.CenterY) * (r.FrontY - r.CenterY)));
            theta += omega * dt;
        }

        // Warmup skip: the first ~1.5 s while the no-slip rear seeds and the front smoother converges.
        var warm = (int)(1.5 * hz);

        // (a) Center stays L/2 behind the front at this rate (the pivot the viewers rely on, unaffected by dt).
        double maxBehindErr = 0;
        for (var i = warm; i < steps; i++) maxBehindErr = Math.Max(maxBehindErr, Math.Abs(behind[i] - Len / 2));
        Assert.True(maxBehindErr < 0.1, $"[{hz}Hz] center drifted {maxBehindErr:F3} m off L/2 behind the front");

        // (b) Bounded per-frame yaw-accel: a constant-curvature arc has ~zero true yaw-accel, so the
        // reconstruction must not manufacture jerk. Measured in deg/s^2 (rate-independent units) after warmup.
        double maxYawAccel = 0;
        double prevRate = double.NaN;
        for (var i = warm + 1; i < steps; i++)
        {
            var rate = AngDiff(headings[i], headings[i - 1]) / dt;      // deg/s
            if (!double.IsNaN(prevRate))
            {
                var accel = Math.Abs(rate - prevRate) / dt;            // deg/s^2
                maxYawAccel = Math.Max(maxYawAccel, accel);
            }
            prevRate = rate;
        }
        Assert.True(maxYawAccel < 120.0, $"[{hz}Hz] yaw-accel spiked to {maxYawAccel:F1} deg/s^2 (expected smooth, gains should adapt to dt)");

        // (c) The reconstruction actually tracks the turn: steady-state yaw-rate MAGNITUDE ~= the true omega.
        // (The arc curves left, so navi heading decreases -> the signed rate is negative; magnitude is the claim.)
        var trueRateDeg = omega * 180.0 / Math.PI;
        var lastRate = Math.Abs(AngDiff(headings[^1], headings[^2]) / dt);
        Assert.True(Math.Abs(lastRate - trueRateDeg) < 1.5,
            $"[{hz}Hz] reconstructed yaw rate {lastRate:F2} deg/s did not converge to the true {trueRateDeg:F2} deg/s");
    }

    // ---------------------------------------------------------------------------------------------------------
    // STUTTER: one long frame (0.25 s) mid-run on a straight. The front advances consistently for that dt, so the
    // reconstruction must absorb it smoothly -- no reseed-threshold spike, no NaN, heading unchanged.
    // ---------------------------------------------------------------------------------------------------------
    [Fact]
    public void Stutter_LongFrame_NoSpikeNoNaN()
    {
        var recon = new KinematicReconstructor();
        var h = H(4);
        const float normalDt = 1f / 60f;
        const int stutterAt = 100;
        const float longDt = 0.25f;

        double fx = 0;
        double prevCx = double.NaN, prevCy = 0;
        double stutterStep = double.NaN;
        double maxOtherStep = 0;
        for (var i = 0; i < 200; i++)
        {
            var dt = i == stutterAt ? longDt : normalDt;
            fx += V * dt;
            var r = recon.ResolveFromFront(h, fx, 0.0, 90f, V, Dims, dt, lateralEvent: false, predictHeadingDeg: null);

            Assert.False(double.IsNaN(r.CenterX) || double.IsNaN(r.CenterY) || double.IsNaN(r.HeadingDeg),
                $"NaN in reconstructed pose at frame {i}");
            Assert.InRange(r.HeadingDeg, 89.5f, 90.5f);

            if (!double.IsNaN(prevCx))
            {
                var step = Math.Sqrt((r.CenterX - prevCx) * (r.CenterX - prevCx) + (r.CenterY - prevCy) * (r.CenterY - prevCy));
                if (i == stutterAt) stutterStep = step;
                else maxOtherStep = Math.Max(maxOtherStep, step);
            }
            prevCx = r.CenterX;
            prevCy = r.CenterY;
        }

        // The long frame moves ~speed*0.25 = 2.5 m -- well below the 7 m reseed threshold, and consistent with
        // the real travel (no teleport/spike). Other frames move ~speed/60 = 0.17 m.
        Assert.True(stutterStep < 7.0, $"stutter frame stepped {stutterStep:F3} m (>= reseed threshold -> spike)");
        Assert.True(Math.Abs(stutterStep - V * longDt) < 0.3, $"stutter step {stutterStep:F3} m not consistent with speed*dt={V * longDt:F3} m");
        Assert.True(maxOtherStep < 0.5, $"a normal frame stepped {maxOtherStep:F3} m (expected ~{V * normalDt:F3} m)");
    }

    // ---------------------------------------------------------------------------------------------------------
    // COARSE-FEED STRADDLE CLASSIFICATION: drives the FULL Resolve(DrClock.Resolved) path over a minimal
    // two-lane geometry. A straddle whose two lane poses diverge WIDELY in heading is, under CoarseFeed, a
    // junction turn (NOT absorbed as a lane change) -- so toggling CoarseFeed changes the emitted path. A
    // near-parallel straddle stays a lane change either way -- CoarseFeed leaves it untouched. The end-to-end
    // junction-turn behaviour on a real SUMO net is covered by CityLib.Tests/ReconstructorS2Tests.
    // ---------------------------------------------------------------------------------------------------------
    [Fact]
    public void CoarseFeed_WideStraddleReclassified_NearParallelUnchanged()
    {
        // Wide-heading straddle (laneB @ navi 45, 45 deg off laneA's navi 90): CoarseFeed must change the path.
        var wideCoarse = RunStraddle(coarseFeed: true, laneBHeadingDeg: 45.0);
        var wideDense = RunStraddle(coarseFeed: false, laneBHeadingDeg: 45.0);
        double wideMaxDiff = 0;
        for (var i = 0; i < wideCoarse.Count; i++)
        {
            wideMaxDiff = Math.Max(wideMaxDiff, Dist(wideCoarse[i], wideDense[i]));
        }
        Assert.True(wideMaxDiff > 0.2,
            $"CoarseFeed did not reclassify the wide (45 deg) straddle: emitted paths differ by only {wideMaxDiff:F3} m " +
            "(expected the junction-turn discriminator to stop absorbing the lateral arc)");

        // Near-parallel straddle (laneB @ navi 95, 5 deg off): below MaxStraddleLaneChangeHeadingDeg (20), so it
        // is a lane change either way -- CoarseFeed changes nothing.
        var parCoarse = RunStraddle(coarseFeed: true, laneBHeadingDeg: 95.0);
        var parDense = RunStraddle(coarseFeed: false, laneBHeadingDeg: 95.0);
        double parMaxDiff = 0;
        for (var i = 0; i < parCoarse.Count; i++)
        {
            parMaxDiff = Math.Max(parMaxDiff, Dist(parCoarse[i], parDense[i]));
        }
        Assert.True(parMaxDiff < 1e-9,
            $"CoarseFeed wrongly altered a near-parallel (5 deg) straddle by {parMaxDiff:E3} m (should stay a lane change)");
    }

    private static double Dist((double X, double Y) a, (double X, double Y) b) =>
        Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));

    // Run a persistent lateral straddle for N frames, advancing only the blend fraction so the lerped front
    // makes small (< LaneChangeSnapMeters) perpendicular steps -- small enough that the ONLY thing that can flip
    // KinematicHeading's lane-change absorption is the straddle classifier's `lateralEvent` (i.e. CoarseFeed).
    // Look-ahead is disabled so the straddle classification is the sole variable. Returns the emitted centers.
    private static List<(double X, double Y)> RunStraddle(bool coarseFeed, double laneBHeadingDeg)
    {
        const int laneA = 10;   // due east (navi 90)
        var laneB = laneBHeadingDeg == 45.0 ? 11 : 12;
        var lanes = new PolyLanes(new Dictionary<int, (double, double)[]>
        {
            [laneA] = Line(90.0, 200.0),
            [laneB] = Line(laneBHeadingDeg, 200.0),
        });

        var recon = new KinematicReconstructor
        {
            CoarseFeed = coarseFeed,
            LookAheadMeters = 0.0,
            LookAheadLengthFactor = 0.0, // fully disable the spatial look-ahead so only the straddle differs
        };
        var h = H(200);
        const float dt = 1f / 60f;
        var upA = new UpcomingLanes(new[] { laneA });
        var upB = new UpcomingLanes(new[] { laneB });

        var centers = new List<(double X, double Y)>();
        for (var i = 0; i < 30; i++)
        {
            var f = 0.2f + 0.01f * i; // 0.2 -> 0.49; per-frame front step stays well under LaneChangeSnapMeters
            var stateA = new DrState { Model = DrModel.LaneArc, LaneHandle = laneA, Pos = 50.0, PosLat = 0.0, Speed = V };
            var stateB = new DrState { Model = DrModel.LaneArc, LaneHandle = laneB, Pos = 50.0, PosLat = 0.0, Speed = V };
            var resolved = new DrClock.Resolved(stateA, upA, stateB, upB, f);
            var r = recon.Resolve(h, resolved, lanes, Dims, dt);
            Assert.True(r.Ok, "straddle front should resolve on the in-memory geometry");
            centers.Add((r.CenterX, r.CenterY));
        }

        return centers;
    }

    // A 2-point straight polyline from the origin along a navi heading, `length` metres long.
    private static (double, double)[] Line(double naviDeg, double length)
    {
        var (dx, dy) = Dir(naviDeg);
        return new[] { (0.0, 0.0), (dx * length, dy * length) };
    }

    // Minimal ILaneShapeSource over straight polylines (flat: no z). LaneLength = summed segment lengths.
    private sealed class PolyLanes : ILaneShapeSource
    {
        private readonly Dictionary<int, (double, double)[]> _shapes;
        public PolyLanes(Dictionary<int, (double, double)[]> shapes) => _shapes = shapes;

        public double LaneLength(int laneHandle)
        {
            var s = _shapes[laneHandle];
            double len = 0;
            for (var i = 0; i + 1 < s.Length; i++)
            {
                len += Math.Sqrt((s[i + 1].Item1 - s[i].Item1) * (s[i + 1].Item1 - s[i].Item1) +
                                 (s[i + 1].Item2 - s[i].Item2) * (s[i + 1].Item2 - s[i].Item2));
            }
            return len;
        }

        public IReadOnlyList<(double X, double Y)> LaneShape(int laneHandle) => _shapes[laneHandle];
        public IReadOnlyList<double>? LaneShapeZ(int laneHandle) => null;
    }
}
