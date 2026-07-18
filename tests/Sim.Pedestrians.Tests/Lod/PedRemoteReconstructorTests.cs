using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Replication;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// P3-3 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §7) -- closes the server -> wire -> IG ->
// render loop: a real PedLodManager mix (a plain PathArc ped, a lively ActivityTimeline ped, and a ped
// that promotes to FreeKinematic under an interest source) is stepped while its events are pushed
// through the GATED PedReplicationPublisher (PedPublishScheduler + PedBandwidthGovernor, P3-2) ->
// InMemoryPedReplicationBus (a true byte loopback, P3-1) -> PedRemoteReconstructor (P3-3, NEW). The
// reconstructor's render pose is compared against the server's own PositionOf AT THE SAME (render)
// sim time, exercising the real sparse high-power stream (not the ungated P3-1 round-trip path).
public class PedRemoteReconstructorTests
{
    private readonly ITestOutputHelper _output;

    public PedRemoteReconstructorTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private sealed class StraightLineNav : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => null;
    }

    private const double Dt = 0.1;
    private const int Steps = 200; // 20 s @ Dt=0.1: covers Ped1's route and Ped2's timeline

    // Ped1 (PathArc): whole-metre points + a round speed survive FrameCodec's int32-cm position
    // quantization and float32 speed/startTime downcast losslessly (see
    // PedReplicationRoundTripTests's Ped1 remarks for the identical reasoning) -- so the "server==IG,
    // essentially exact" low-power claim isn't accidentally flattered by a lucky rounding.
    private static readonly Vec2[] PathArcPath = { new(0.0, 0.0), new(20.0, 0.0) };
    private const double PathArcSpeed = 2.0;

    // Ped3 (promotes under an interest source sitting on its spawn point).
    private static readonly Vec2[] PathArc3 = { new(100.0, 0.0), new(120.0, 0.0) };
    private const double Ped3MaxSpeed = 1.4;

    private sealed record Trace(
        int Step, double RenderTime,
        bool Ped1Known, Vec2 Ped1Render, Vec2 Ped1Server,
        bool Ped2Known, Vec2 Ped2Render, Vec2 Ped2Server,
        bool Ped3Known, bool Ped3HighPower, Vec2 Ped3Render, Vec2 Ped3RawUnsmoothed, Vec2 Ped3Server);

    // Builds the full manager + gated wire + reconstructor pipeline and returns a per-step trace.
    // Shared by the tolerance test, the no-pop test, and the determinism test so all three exercise
    // literally the same pipeline construction.
    private static List<Trace> RunScenario()
    {
        var publisher = new PedPublisher();
        var manager = new PedLodManager(new StraightLineNav(), publisher, arriveRadius: 0.3, dwellSeconds: 1.0);

        manager.AddPed(id: 1, PathArcPath, PathArcSpeed, radius: 0.3, now: 0.0);

        var timeline = new ActivityTimeline(t0: 0.0, new ActivitySegment[]
        {
            new WalkSegment(new[] { new Vec2(0.0, 10.0), new Vec2(11.7, 10.0) }, 1.53),
            new PauseSegment(1.25, "sip"),
            new WalkSegment(new[] { new Vec2(11.7, 10.0), new Vec2(11.7, 21.4) }, 1.53),
        });
        manager.AddPedLively(id: 2, timeline, maxSpeed: 1.53, radius: 0.3, now: 0.0);

        manager.AddPed(id: 3, PathArc3, Ped3MaxSpeed, radius: 0.3, now: 0.0);

        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        field.Register(new InterestSource(new Vec2(100.0, 0.0), promoteRadius: 3.0, demoteRadius: 6.0), InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        var meter = new PedBandwidthMeter();
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);
        var bus = new InMemoryPedReplicationBus();
        var wirePublisher = new PedReplicationPublisher(bus.Sink, scheduler, governor, meter, stepDt: Dt);
        var reconstructor = new PedRemoteReconstructor(bus.Source);

        // Publish/drain the spawn-time PathArc/ActivityTimeline legs before the stepping loop, exactly
        // like PedReplicationRoundTripTests.
        wirePublisher.Publish(publisher.Events);
        reconstructor.Pump(0.0);

        var trace = new List<Trace>(Steps);
        var now = 0.0;
        for (var i = 0; i < Steps; i++)
        {
            var beforeCount = publisher.Events.Count;
            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            var newEvents = new List<PedEvent>(publisher.Events.Count - beforeCount);
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            wirePublisher.Publish(newEvents);
            reconstructor.Pump(now);

            var rt = reconstructor.RenderTime;

            var p1Known = reconstructor.TryGetRenderPose(1, out var p1r, out _, out _);
            var p2Known = reconstructor.TryGetRenderPose(2, out var p2r, out _, out _);
            var p3Known = reconstructor.TryGetRenderPose(3, out var p3r, out _, out _);
            var p3High = reconstructor.Ig.Knows(3) && reconstructor.Ig.ModelOf(3) == PedDrModel.FreeKinematic;
            var p3Raw = reconstructor.Ig.Knows(3) ? reconstructor.Ig.ReconstructSample(3, rt).Pos : Vec2.Zero;

            trace.Add(new Trace(
                i, rt,
                p1Known, p1r, manager.PositionOf(1, rt),
                p2Known, p2r, manager.PositionOf(2, rt),
                p3Known, p3High, p3r, p3Raw, manager.PositionOf(3, rt)));
        }

        return trace;
    }

    [Fact]
    public void ServerEqualsIg_OverTheGatedWire_WithinRenderTolerance()
    {
        var trace = RunScenario();

        var exact1 = 0;
        var maxErr1 = 0.0;
        var exact2 = 0;
        var maxErr2 = 0.0;
        var highSamples = 0;
        var maxErr3 = 0.0;
        var everHigh = false;

        foreach (var t in trace)
        {
            if (t.Ped1Known)
            {
                var err = (t.Ped1Render - t.Ped1Server).Abs;
                maxErr1 = Math.Max(maxErr1, err);
                exact1++;
            }

            if (t.Ped2Known)
            {
                var err = (t.Ped2Render - t.Ped2Server).Abs;
                maxErr2 = Math.Max(maxErr2, err);
                exact2++;
            }

            if (t.Ped3Known && t.Ped3HighPower)
            {
                everHigh = true;
                var err = (t.Ped3Render - t.Ped3Server).Abs;
                maxErr3 = Math.Max(maxErr3, err);
                highSamples++;
            }
        }

        _output.WriteLine($"[P3-3 measured] low-power render samples: Ped1(PathArc)={exact1} maxErr={maxErr1:E4} m, "
            + $"Ped2(ActivityTimeline)={exact2} maxErr={maxErr2:E4} m");
        _output.WriteLine($"[P3-3 measured] high-power (gated wire) render samples: {highSamples}, maxErr={maxErr3:E4} m");

        Assert.True(everHigh, "Ped 3 never promoted to FreeKinematic despite sitting on the interest source");
        Assert.True(exact1 > 100, $"expected a full low-power sweep for Ped1, got {exact1}");
        Assert.True(exact2 > 100, $"expected a full low-power sweep for Ped2, got {exact2}");
        Assert.True(highSamples > 0, "expected at least one high-power render comparison");

        // Low-power: the reconstructed target is a pure function of (path/timeline, renderTime) --
        // identical to what the server itself evaluates at that same renderTime -- and the smoothing
        // cap (5 m/s) is far above these peds' walking speed (<=1.53 m/s), so every frame's correction
        // is within cap and the smoother snaps directly to the target (see PedRemoteReconstructor.Smooth):
        // essentially exact once the playout delay is accounted for (both sides evaluated at the SAME
        // renderTime).
        Assert.True(maxErr1 <= 1e-6, $"Ped1 (PathArc) render error {maxErr1:E4} m exceeded the near-exact bound");
        Assert.True(maxErr2 <= 1e-9, $"Ped2 (ActivityTimeline) render error {maxErr2:E4} m exceeded the exact bound");

        // High-power: DR-error gating (PosTol=0.1m) + cm quantization + the smoothing catch-up bound.
        Assert.True(maxErr3 <= 0.25, $"Ped3 (FreeKinematic, gated) render error {maxErr3:E4} m exceeded the 0.25 m render tolerance");
    }

    [Fact]
    public void PromotionReconstructsWithNoPop()
    {
        var trace = RunScenario();

        var promoteStep = -1;
        for (var i = 0; i < trace.Count; i++)
        {
            if (trace[i].Ped3Known && trace[i].Ped3HighPower)
            {
                promoteStep = i;
                break;
            }
        }

        Assert.True(promoteStep > 0, "Ped 3 never promoted");

        // Per-frame displacement of the SMOOTHED render position vs. the RAW (unsmoothed) reconstructed
        // target, across the whole run (the bound must hold everywhere, not just conveniently near the
        // switch) -- with the window right around the promotion reported separately for comparison.
        double maxSmoothedStep = 0.0, maxRawStep = 0.0;
        double windowMaxSmoothedStep = 0.0, windowMaxRawStep = 0.0;
        Vec2? prevSmoothed = null, prevRaw = null;

        for (var i = 0; i < trace.Count; i++)
        {
            var t = trace[i];
            if (!t.Ped3Known)
            {
                prevSmoothed = null;
                prevRaw = null;
                continue;
            }

            if (prevSmoothed is { } ps)
            {
                var step = (t.Ped3Render - ps).Abs;
                maxSmoothedStep = Math.Max(maxSmoothedStep, step);
                if (Math.Abs(i - promoteStep) <= 2)
                {
                    windowMaxSmoothedStep = Math.Max(windowMaxSmoothedStep, step);
                }
            }

            if (prevRaw is { } pr)
            {
                var step = (t.Ped3RawUnsmoothed - pr).Abs;
                maxRawStep = Math.Max(maxRawStep, step);
                if (Math.Abs(i - promoteStep) <= 2)
                {
                    windowMaxRawStep = Math.Max(windowMaxRawStep, step);
                }
            }

            prevSmoothed = t.Ped3Render;
            prevRaw = t.Ped3RawUnsmoothed;
        }

        // maxSpeed*Dt (real travel) + the smoothing cap (5 m/s * Dt) -- the same bound
        // PedRemoteReconstructor's Smooth() enforces by construction (a per-frame correction is never
        // allowed past this, short of the >3m snap-to-target branch, which a continuous promotion never
        // triggers here).
        const double bound = (3.0 + 2.0) * Dt + Ped3MaxSpeed * Dt;

        _output.WriteLine($"[P3-3 measured] Ped3 promotion at step {promoteStep}: "
            + $"max single-frame render step over whole run: smoothed={maxSmoothedStep:E4} m, unsmoothed={maxRawStep:E4} m "
            + $"(bound {bound:E4} m); around the switch (+/-2 steps): smoothed={windowMaxSmoothedStep:E4} m, unsmoothed={windowMaxRawStep:E4} m");

        Assert.True(maxSmoothedStep <= bound + 1e-9, $"smoothed max single-frame step {maxSmoothedStep:E4} m exceeded the no-pop bound {bound:E4} m");
        Assert.True(windowMaxSmoothedStep <= bound + 1e-9, $"smoothed max single-frame step AROUND THE SWITCH {windowMaxSmoothedStep:E4} m exceeded the no-pop bound {bound:E4} m");
    }

    // A focused, low-level exercise of the capped-correction branch itself (Smooth's `dist > cap` path):
    // the LOD-driven scenario above never happens to demand a correction bigger than the cap (its
    // measured max single-frame step, ~0.14-0.21 m, always fits under the ~0.5 m cap at Dt=0.1), so this
    // drives PedRemoteReconstructor directly off a raw IPedReplicationSource with a deliberately large
    // (1.0 m) single-step reconciliation -- the kind of jump a P3-2 governor-deferred catch-up or a
    // reconnect could produce -- and confirms it is absorbed over TWO frames (capped at ~0.5 m each),
    // never a one-frame teleport, while a plain/unsmoothed reconstruction would show the whole 1.0 m in
    // one step.
    [Fact]
    public void LargeSingleStepCorrection_IsAbsorbedOverAFewFrames_NotTeleported()
    {
        var bus = new InMemoryPedReplicationBus();
        var reconstructor = new PedRemoteReconstructor(bus.Source, playoutDelaySeconds: 0.0);

        bus.Sink.PublishPedLifecycle(new PedLifecycleRecord(new VehicleHandle(99, 0), PedLifecycleKind.PromoteToFreeKinematic, 0.0));
        bus.Sink.PublishCrowdFrame(0, 0.0f, new[] { new PedFreeKinematicRecord(new VehicleHandle(99, 0), 0.0, 0.0, 0.0, 0.0, 0.0) });
        reconstructor.Pump(0.0);
        Assert.True(reconstructor.TryGetRenderPose(99, out var pos0, out _, out _));
        Assert.Equal(0.0, pos0.X, 6);

        // The unsmoothed reconstruction's own jump: a full 1.0 m in one wire update (this IS what
        // Ig.ReconstructSample already shows, unsmoothed, once the sample lands).
        bus.Sink.PublishCrowdFrame(1, (float)Dt, new[] { new PedFreeKinematicRecord(new VehicleHandle(99, 0), 1.0, 0.0, 0.0, 0.0, 0.0) });
        reconstructor.Pump(Dt);
        var rawJump = reconstructor.Ig.ReconstructSample(99, reconstructor.RenderTime).Pos;
        Assert.Equal(1.0, rawJump.X, 6);

        Assert.True(reconstructor.TryGetRenderPose(99, out var pos1, out _, out _));
        var step1 = (pos1 - pos0).Abs;

        // Steady (no further correction needed) -- the smoother should finish closing the gap.
        bus.Sink.PublishCrowdFrame(2, (float)(2 * Dt), new[] { new PedFreeKinematicRecord(new VehicleHandle(99, 0), 1.0, 0.0, 0.0, 0.0, 0.0) });
        reconstructor.Pump(2 * Dt);
        Assert.True(reconstructor.TryGetRenderPose(99, out var pos2, out _, out _));
        var step2 = (pos2 - pos1).Abs;

        const double capPerFrame = (3.0 + 2.0) * Dt; // 0.5 m at Dt=0.1 -- see PedRemoteReconstructor.Smooth

        _output.WriteLine($"[P3-3 measured] large 1.0 m single-step correction: unsmoothed jump=1.0000 m in one frame; "
            + $"smoothed frame steps: {step1:F4} m, {step2:F4} m (cap {capPerFrame:F4} m/frame); "
            + $"smoothed render x after frame 1/2: {pos1.X:F4}/{pos2.X:F4} m (target 1.0 m)");

        Assert.True(step1 <= capPerFrame + 1e-9, $"frame-1 smoothed step {step1:E4} m exceeded the cap {capPerFrame:E4} m");
        Assert.True(step1 < 1.0 - 1e-9, $"frame-1 smoothed step {step1:E4} m should NOT be the full 1.0 m unsmoothed jump (that would be a teleport)");
        Assert.Equal(1.0, pos2.X, 6); // fully caught up by frame 2
        Assert.True(step2 <= capPerFrame + 1e-9, $"frame-2 smoothed step {step2:E4} m exceeded the cap {capPerFrame:E4} m");
    }

    [Fact]
    public void SameRunTwice_ProducesIdenticalReconstructedRenderPoseStream()
    {
        var traceA = RunScenario();
        var traceB = RunScenario();

        Assert.Equal(traceA.Count, traceB.Count);
        for (var i = 0; i < traceA.Count; i++)
        {
            var a = traceA[i];
            var b = traceB[i];

            Assert.Equal(a.RenderTime, b.RenderTime);
            Assert.Equal(a.Ped1Known, b.Ped1Known);
            Assert.Equal(a.Ped1Render.X, b.Ped1Render.X);
            Assert.Equal(a.Ped1Render.Y, b.Ped1Render.Y);
            Assert.Equal(a.Ped2Known, b.Ped2Known);
            Assert.Equal(a.Ped2Render.X, b.Ped2Render.X);
            Assert.Equal(a.Ped2Render.Y, b.Ped2Render.Y);
            Assert.Equal(a.Ped3Known, b.Ped3Known);
            Assert.Equal(a.Ped3HighPower, b.Ped3HighPower);
            Assert.Equal(a.Ped3Render.X, b.Ped3Render.X);
            Assert.Equal(a.Ped3Render.Y, b.Ped3Render.Y);
        }

        _output.WriteLine($"[P3-3 measured] determinism: {traceA.Count} steps, identical reconstructed render-pose stream across two independent runs.");
    }
}
