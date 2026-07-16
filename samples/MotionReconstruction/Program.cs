// MotionReconstruction -- a tutorial-style walkthrough of SumoSharp.Viewer.Motion: turning a SPARSE,
// low-rate stream of vehicle samples into a SMOOTH, continuous per-frame render pose, with no renderer
// and no wire transport involved (see src/Sim.Viewer.Motion/README.md's pipeline pseudocode -- this
// sample is that pseudocode, made runnable). Run it with:
//   dotnet run --project samples/MotionReconstruction
using Sim.Core;
using Sim.Replication;
using Sim.Viewer.Motion;

internal static class Program
{
    // The vehicle's own physical length (m) -- feeds PoseResolver's ChordHeading back-point offset.
    private const double VehicleLength = 4.5;
    private const double VehicleWidth = 1.8;

    // Render loop: ~10 Hz for a few real seconds. DrClock.Pump ties its render clock to REAL wall-clock
    // time (a Stopwatch) -- exactly as a live game/viewer loop would call it once per app frame -- so
    // this sample actually sleeps between frames rather than faking elapsed time.
    private const int FrameCount = 30;
    private const int FrameMillis = 100;

    // Playout delay (s): render `delay` seconds "in the past" relative to the render clock, so most
    // frames land BETWEEN two buffered samples (interpolation) instead of racing ahead of the newest
    // one (extrapolation). See "interpolate vs extrapolate", below.
    private const double Delay = 0.4;

    private static int Main()
    {
        Console.WriteLine("MotionReconstruction -- SumoSharp.Viewer.Motion: sparse samples -> smooth poses.");
        Console.WriteLine();

        // 1) ILaneShapeSource: the read-only static lane geometry PoseResolver/DrClock walk. A real
        //    viewer gets this from the parsed .net.xml (NetworkLaneSource); here we hand-build the
        //    smallest possible one -- a single straight lane, a 2-point polyline (0,0) -> (100,0).
        //    LaneShapeZ returning null means "flat" (PoseResolver.Resolve skips elevation).
        var lanes = new StraightLaneSource(laneHandle: 0, length: 100.0, from: (0.0, 0.0), to: (100.0, 0.0));

        // 2) A SPARSE, 1 Hz history of one vehicle driving down that lane at a constant 10 m/s (arc
        //    position advances 0 -> 10 -> 20 over 3 samples, 1 second apart). VehicleSampleHistory is
        //    the transport-neutral, capacity-bounded, newest-last buffer DrClock.Resolve reads --
        //    exactly what a DDS/TCP subscriber would append to as packets arrive, pre-filled here since
        //    we already have all 3 samples up front. Upcoming=[0] (just the current lane; our lane is
        //    long enough that the vehicle never needs a downstream lane within this run).
        var handle = new VehicleHandle(1, 0);
        var history = new VehicleSampleHistory(capacity: 8);
        for (var i = 0; i < 3; i++)
        {
            var record = new VehicleRecord(
                handle, DrModel.LaneArc, laneHandle: 0,
                pos: i * 10.0, posLat: 0.0, speed: 10.0, accel: 0.0, latSpeed: 0.0,
                upcoming: new UpcomingLanes(stackalloc[] { 0 }));
            history.Append(new TimestampedSample(timestampSeconds: i * 1.0, record));
        }

        Console.WriteLine($"history: 3 samples @ 1 Hz -- t=0.0s pos=0m, t=1.0s pos=10m, t=2.0s pos=20m (10 m/s)");
        Console.WriteLine($"render loop: {FrameCount} frames @ ~{1000.0 / FrameMillis:F0} Hz, playout delay={Delay:F1}s");
        Console.WriteLine();

        // 3) The reconstruction pipeline (README's pseudocode, verbatim): a DrClock per vehicle (its
        //    render clock + interpolate/extrapolate resolver) and a DrPoseSmoother (the optional
        //    per-frame position/heading smoothing pass a real renderer runs after resolving a pose).
        var clock = new DrClock();
        var smoother = new DrPoseSmoother();

        var sw = System.Diagnostics.Stopwatch.StartNew();
        var lastWall = 0.0;

        for (var frame = 0; frame < FrameCount; frame++)
        {
            if (frame > 0)
            {
                Thread.Sleep(FrameMillis);
            }

            var nowWall = sw.Elapsed.TotalSeconds;
            var frameDt = (float)(frame == 0 ? 0.0 : nowWall - lastWall);
            lastWall = nowWall;

            // Pump once per app frame with the newest sample time this "transport" has ever delivered
            // (here a fixed 2.0s -- the whole history was built up front). DrClock advances its OWN
            // render clock (RenderSim) from a long-baseline wall<->sim rate fit, immune to per-packet
            // jitter, and NEVER steps it backward (it holds instead) -- see DrClock.cs's Pump comment.
            clock.Pump(newestSampleTime: 2.0);

            // Resolve picks (or interpolates between) the two buffered samples bracketing
            // `RenderSim - delay`: EXTRAPOLATE forward past the newest sample (delay small/zero, or the
            // render clock has run past the last known sample), EXTRAPOLATE backward past the oldest
            // (not reachable here), or INTERPOLATE between the bracketing pair with a plain arc-length
            // lerp (same lane throughout, so no arc-window walk is needed).
            var resolved = clock.Resolve(history, delay: Delay, lanes);

            // The caller fills in physical dims (sent once per vType/handle in a real system, not per
            // frame) before handing the state to PoseResolver. dt=0 because Resolve already advanced
            // the arc to render time.
            var state = resolved.State with { Length = VehicleLength, Width = VehicleWidth };
            var pose = PoseResolver.Resolve(
                lanes, state, upcomingLanes: stackalloc[] { state.LaneHandle },
                precedingLanes: ReadOnlySpan<int>.Empty, dt: 0.0, RenderRealism.ChordHeading);

            // 4) DrPoseSmoother.Smooth: an optional per-vehicle, per-frame chase filter applied AFTER
            //    resolving a target pose -- capped position error-smoothing + motion-derived heading
            //    tilt + a heading low-pass. The first observation of a handle returns the target
            //    unchanged (nothing to smooth from yet).
            var (sx, sy, sdeg) = smoother.Smooth(handle, pose.X, pose.Y, pose.HeadingDeg, state.Speed, frameDt);

            var regime = resolved.Extrapolated ? "extrapolate" : "interpolate";
            Console.WriteLine(
                $"frame {frame,2}  wall={nowWall,4:F2}s  renderSim={clock.RenderSim,5:F2}s  " +
                $"sampleT={clock.RenderSim - Delay,5:F2}s  [{regime,-11}]  " +
                $"pos={pose.X,6:F2},{pose.Y,5:F2}  heading={pose.HeadingDeg,5:F1}deg  " +
                $"smoothed=({sx,6:F2},{sy,5:F2})");
        }

        Console.WriteLine();
        Console.WriteLine("what to notice:");
        Console.WriteLine(
            " - the underlying history only has 3 samples a full second apart, yet `pos` advances");
        Console.WriteLine(
            "   frame-by-frame -- that gap is exactly what DrClock.Resolve bridges.");
        Console.WriteLine(
            " - frames ~0-9: DrClock.Pump's long-baseline wall<->sim rate fit needs ~1 real second of");
        Console.WriteLine(
            "   data before it engages (RenderSim holds steady meanwhile -- see Pump's \"pre-baseline");
        Console.WriteLine(
            "   guard\" comment); `pos` sits still, but DrPoseSmoother keeps creeping the SMOOTHED pose");
        Console.WriteLine(
            "   forward anyway (its forward-biased catch-up cap never lets a moving vehicle's render");
        Console.WriteLine(
            "   pose visibly freeze) -- watch `smoothed` keep climbing even while `pos` does not.");
        Console.WriteLine(
            " - frame ~10 on: the fit engages, RenderSim starts advancing at ~1 sim-second per real");
        Console.WriteLine(
            "   second. Frame 10 still INTERPOLATES (sampleT=1.90s is inside the buffered [0,2]s");
        Console.WriteLine(
            "   window); from frame 11 sampleT runs past the newest sample (2.0s), so DrClock.Resolve");
        Console.WriteLine(
            "   switches to EXTRAPOLATE -- see the [interpolate]/[extrapolate] column above.");
        Console.WriteLine(
            " - `delay` (here 0.4s) is the knob: raise it and more frames land inside the interpolated");
        Console.WriteLine(
            "   window (smoother, but laggier); lower it toward 0 and DrClock extrapolates almost");
        Console.WriteLine("   immediately (lower latency, more exposed to prediction error).");
        return 0;
    }

    // The smallest possible ILaneShapeSource: one straight lane, a 2-point polyline. LaneShapeZ
    // returning null means "flat" -- PoseResolver.Resolve skips the elevation sample entirely.
    private sealed class StraightLaneSource : ILaneShapeSource
    {
        private readonly int _laneHandle;
        private readonly double _length;
        private readonly (double X, double Y)[] _shape;

        public StraightLaneSource(int laneHandle, double length, (double X, double Y) from, (double X, double Y) to)
        {
            _laneHandle = laneHandle;
            _length = length;
            _shape = new[] { from, to };
        }

        public double LaneLength(int laneHandle) => laneHandle == _laneHandle
            ? _length
            : throw new ArgumentOutOfRangeException(nameof(laneHandle));

        public IReadOnlyList<(double X, double Y)> LaneShape(int laneHandle) => laneHandle == _laneHandle
            ? _shape
            : throw new ArgumentOutOfRangeException(nameof(laneHandle));

        public IReadOnlyList<double>? LaneShapeZ(int laneHandle) => null;
    }
}
