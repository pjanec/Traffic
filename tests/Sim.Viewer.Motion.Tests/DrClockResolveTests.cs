using Sim.Core;
using Sim.Replication;
using Sim.Viewer.Motion;
using Xunit;

namespace Sim.Viewer.Motion.Tests;

// Packaging P2-B regression pin (docs/SUMOSHARP-PACKAGING-DESIGN.md §5 / P2.1): DrClock.Resolve's first
// parameter was swapped from the DDS-local `DdsSubscriber.VehicleSample` to the transport-neutral
// `Sim.Replication.TimestampedSample` / `IVehicleSampleHistory`. That was meant to be a PURE type
// substitution -- these tests pin that the same branch (extrapolate / same-lane interpolate / downstream
// ArcInWindow interpolate / lateral-straddle two-state) fires and the same arc is computed as before the
// swap, for a fixed set of hand-built histories.
public sealed class DrClockResolveTests
{
    // Minimal ILaneShapeSource stub: only LaneLength is exercised by DrClock.Resolve (via ArcInWindow);
    // LaneShape/LaneShapeZ are never called from that path, so they intentionally throw if they ever are --
    // that would mean this test's assumptions about DrClock's internals are stale.
    private sealed class FixedLengthLaneSource : ILaneShapeSource
    {
        private readonly Dictionary<int, double> _lengths;

        public FixedLengthLaneSource(Dictionary<int, double> lengths) => _lengths = lengths;

        public double LaneLength(int laneHandle) => _lengths[laneHandle];

        public IReadOnlyList<(double X, double Y)> LaneShape(int laneHandle) =>
            throw new NotSupportedException("DrClock.Resolve never calls LaneShape.");

        public IReadOnlyList<double>? LaneShapeZ(int laneHandle) =>
            throw new NotSupportedException("DrClock.Resolve never calls LaneShapeZ.");
    }

    private static VehicleRecord Rec(
        VehicleHandle handle, int laneHandle, double pos, double posLat, double speed, int[] upcoming) =>
        new(handle, DrModel.LaneArc, laneHandle, pos, posLat, speed, accel: 0.0, latSpeed: 0.0,
            new UpcomingLanes(upcoming));

    // DrClock.Pump's very first call always jumps `_renderSim` straight to `newestSampleTime` (renderSim
    // starts at 0.0, and Pump's "init/restart" branch fires whenever renderSim == 0.0). So one
    // `Pump(newestSampleTime)` call deterministically sets RenderSim == newestSampleTime, letting a test
    // pick `delay` directly to land `sampleT = RenderSim - delay` wherever it wants within the bracket.
    private static DrClock PumpedTo(double renderSim)
    {
        var clock = new DrClock();
        clock.Pump(renderSim);
        Assert.Equal(renderSim, clock.RenderSim, 6);
        return clock;
    }

    [Fact]
    public void Resolve_SameLaneBracket_InterpolatesStraightArcLerp()
    {
        var handle = new VehicleHandle(1, 0);
        const int lane = 10;
        var lanes = new FixedLengthLaneSource(new Dictionary<int, double> { [lane] = 100.0 });

        var history = new VehicleSampleHistory(capacity: 8);
        history.Append(new TimestampedSample(0.0, Rec(handle, lane, pos: 0.0, posLat: 0.0, speed: 10.0, upcoming: new[] { lane })));
        history.Append(new TimestampedSample(1.0, Rec(handle, lane, pos: 10.0, posLat: 0.0, speed: 10.0, upcoming: new[] { lane })));

        var clock = PumpedTo(1.0);
        var resolved = clock.Resolve(history, delay: 0.5, lanes); // sampleT = 1.0 - 0.5 = 0.5 -> mid-bracket

        Assert.False(resolved.Extrapolated);
        Assert.False(resolved.IsLateralStraddle);
        Assert.Equal(lane, resolved.State.LaneHandle);
        Assert.Equal(5.0, resolved.State.Pos, precision: 6); // exact arc lerp: 0 + (10-0)*0.5
    }

    [Fact]
    public void Resolve_DownstreamStraddleWithinWindow_UsesArcInWindowOffset()
    {
        var handle = new VehicleHandle(2, 0);
        const int laneA = 10;
        const int laneB = 11;
        var lanes = new FixedLengthLaneSource(new Dictionary<int, double> { [laneA] = 20.0, [laneB] = 30.0 });

        // `a`'s forward window includes laneB immediately downstream of laneA -> ArcInWindow finds it at
        // cumulative offset = LaneLength(laneA) = 20.
        var history = new VehicleSampleHistory(capacity: 8);
        history.Append(new TimestampedSample(0.0, Rec(handle, laneA, pos: 15.0, posLat: 0.0, speed: 10.0, upcoming: new[] { laneA, laneB })));
        history.Append(new TimestampedSample(1.0, Rec(handle, laneB, pos: 3.0, posLat: 0.0, speed: 10.0, upcoming: new[] { laneB })));

        var clock = PumpedTo(1.0);
        var resolved = clock.Resolve(history, delay: 0.5, lanes); // sampleT = 0.5, f = 0.5

        Assert.False(resolved.Extrapolated);
        Assert.False(resolved.IsLateralStraddle);
        // Anchored on `a`'s lane (LaneArc's arc-length frame), NOT snapped to `b`'s lane yet.
        Assert.Equal(laneA, resolved.State.LaneHandle);
        // arc = a.Pos + (ArcInWindow(20) + b.Pos(3) - a.Pos(15)) * f(0.5) = 15 + 8*0.5 = 19.
        Assert.Equal(19.0, resolved.State.Pos, precision: 6);
        // The point of ArcInWindow: it advances PAST a raw same-lane lerp between the two packets' raw Pos
        // values (which would give 15 + (3-15)*0.5 = 9, i.e. backward) -- it must land ahead of a.Pos.
        Assert.True(resolved.State.Pos > 15.0);
    }

    [Fact]
    public void Resolve_LateralStraddleOutsideWindow_ReturnsTwoStateBlend()
    {
        var handle = new VehicleHandle(3, 0);
        const int laneA = 10;
        const int laneB = 11; // a sibling lane NOT reachable from laneA's forward window below.
        var lanes = new FixedLengthLaneSource(new Dictionary<int, double> { [laneA] = 20.0, [laneB] = 30.0 });

        // `a`'s forward window is JUST laneA (no downstream entries) -> ArcInWindow can never find laneB.
        var history = new VehicleSampleHistory(capacity: 8);
        history.Append(new TimestampedSample(0.0, Rec(handle, laneA, pos: 15.0, posLat: 0.0, speed: 10.0, upcoming: new[] { laneA })));
        history.Append(new TimestampedSample(1.0, Rec(handle, laneB, pos: 3.0, posLat: 1.5, speed: 10.0, upcoming: new[] { laneB })));

        var clock = PumpedTo(1.0);
        var resolved = clock.Resolve(history, delay: 0.5, lanes); // sampleT = 0.5, f = 0.5

        Assert.False(resolved.Extrapolated);
        Assert.True(resolved.IsLateralStraddle);
        Assert.Equal(laneA, resolved.State.LaneHandle);
        Assert.NotNull(resolved.SecondState);
        Assert.Equal(laneB, resolved.SecondState!.Value.LaneHandle);
        Assert.Equal(0.5f, resolved.Blend, precision: 5);
        Assert.Equal(1.0f, resolved.PacketSpan, precision: 5); // b.TimestampSeconds - a.TimestampSeconds
    }
}
