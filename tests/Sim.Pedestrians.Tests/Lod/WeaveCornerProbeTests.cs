using System;
using System.Collections.Generic;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// Diagnostic-turned-regression-guard for the junction-turn back-step fix (PathArcMotion.WeaveAxisAt /
// ActivityTimeline.Evaluate's WalkSegment weave projection). Root cause (found by tracing a real
// LiveCitySim ped through a junction, see PedBackstepProbeTests in Sim.LiveCity.Tests): at a polyline
// VERTEX the raw per-segment tangent SampleAt returns flips discontinuously the instant the walked arc-
// length crosses it, and the weave places the pose at `centre + tangent.PerpCW * (c+off)` -- so the SAME
// roughly-constant lateral distance gets re-projected onto a suddenly-rotated axis in a single sample,
// popping the pose sideways/backward. This is NOT a render-side extrapolation-past-the-waypoint bug (the
// hypothesis in the task write-up): the raw ActivityTimeline.PoseAt/HeadlessIg.ReconstructSample itself
// (the shared server==IG evaluator) produces the discontinuity; the render-side capped-correction smoother
// only partially cushions it over 2-3 frames.
//
// A path that turns sharply (> ~90 degrees) inherently has a large frame-to-frame heading change even with
// the weave OFF (pure centreline) -- that is expected turning motion, not a bug. Also, the fix legitimately
// makes the WOVEN pose sweep a little wider/faster through a corner (a smooth, continuous, slightly-larger
// per-frame displacement while the weave axis blends across the vertex) -- confirmed by hand-inspecting a
// frame-by-frame trace: displacement magnitude rises and falls smoothly, like a shallow bump, never in a
// single-frame spike. That is a real, intentional, harmless side effect, NOT the bug being fixed -- and
// critically, a "the bump's peak is a bit taller than the flat steady-state" heuristic can't tell a smooth
// bump apart from a genuine jump discontinuity (both raise the observed per-frame delta), so the tests below
// use the actual MATHEMATICAL definition instead: (a)/(b) below rule out any backward motion, and (c) is a
// true continuity test -- sample the same corner at a MUCH finer dt (100x) and confirm the max frame-to-
// frame delta shrinks roughly proportionally (a continuous curve's per-sample delta -> 0 as dt -> 0). A real
// discontinuity (the pre-fix bug: a ~0.235 m jump in one 16.7 ms frame in the real LiveCitySim trace, see
// PedBackstepProbeTests) has a FIXED jump height independent of sampling rate -- it would NOT shrink when
// resampled 100x finer, which is exactly what distinguishes "smooth sweep" from "pop" here.
public class WeaveCornerProbeTests
{
    private readonly ITestOutputHelper _output;
    public WeaveCornerProbeTests(ITestOutputHelper output) => _output = output;

    public static IEnumerable<object[]> Corners()
    {
        var seeds = new (ulong Seed, ulong Global)[]
        {
            (0xABCDEF12UL, 42UL),
            (0x1122334455UL, 7UL),
            (0x99AA88BBUL, 12345UL),
            (0xDEADBEEFUL, 999UL),
        };

        foreach (var deg in new[] { 15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 120.0, 150.0 })
        {
            foreach (var (seed, global) in seeds)
            {
                yield return new object[] { deg, seed, global };
            }
        }
    }

    private static (double MaxJump, double MaxBackDot) TraceCorner(ActivityTimeline tl, double cornerTime, double dt = 0.02)
    {
        double maxBackDot = 0.0;
        double maxJump = 0.0;
        Vec2? prevPos = null;
        Vec2? prevDir = null;

        for (var t = Math.Max(0.0, cornerTime - 2.0); t <= cornerTime + 2.0; t += dt)
        {
            var pose = tl.PoseAt(t);
            if (prevPos is { } pp)
            {
                var disp = pose.Pos - pp;
                if (disp.Abs > maxJump)
                {
                    maxJump = disp.Abs;
                }

                if (prevDir is { } pd && pd.Abs > 1e-9 && disp.Abs > 1e-9)
                {
                    var dot = (pd.X * (disp.X / disp.Abs)) + (pd.Y * (disp.Y / disp.Abs));
                    if (dot < maxBackDot)
                    {
                        maxBackDot = dot;
                    }
                }

                if (disp.Abs > 1e-9)
                {
                    prevDir = disp / disp.Abs;
                }
            }

            prevPos = pose.Pos;
        }

        return (maxJump, maxBackDot);
    }

    [Theory]
    [MemberData(nameof(Corners))]
    public void Probe_TurnAtVariousAngles_WeaveDoesNotAddExtraPopBeyondTheTurnItself(double turnDegrees, ulong seed, ulong globalSeed)
    {
        var rad = turnDegrees * Math.PI / 180.0;
        var dir2 = new Vec2(Math.Cos(rad), Math.Sin(rad));
        var path = new List<Vec2> { new(0, 0), new(20, 0), new Vec2(20, 0) + (dir2 * 20.0) };
        var widths = new List<double> { 2.0, 2.0, 2.0 };
        const double speed = 1.3;

        var tlOn = new ActivityTimeline(0.0, new ActivitySegment[] { new WalkSegment(path, speed, widths) }, seed, globalSeed);
        var tlOff = new ActivityTimeline(0.0, new ActivitySegment[] { new WalkSegment(path, speed) }); // no HalfWidths -> weave inactive

        var cornerTime = 20.0 / speed;
        const double coarseDt = 0.02;
        const double fineDt = 0.0002; // 100x finer
        var (jumpOn, backDotOn) = TraceCorner(tlOn, cornerTime, coarseDt);
        var (jumpOff, backDotOff) = TraceCorner(tlOff, cornerTime, coarseDt);
        var (jumpOnFine, _) = TraceCorner(tlOn, cornerTime, fineDt);

        _output.WriteLine(
            $"deg={turnDegrees} seed={seed:X} global={globalSeed} corner t={cornerTime:F3}; " +
            $"weaveOn(jump@{coarseDt}={jumpOn:F5} jump@{fineDt}={jumpOnFine:F5} backDot={backDotOn:F4}) weaveOff/turnOnly(jump={jumpOff:F5} backDot={backDotOff:F4})");

        // (a) realistic street-corner turns (<=90 deg): weave-OFF never backsteps (cos(theta)>=0), so
        // weave-ON must not either -- any negative dot here is purely weave-induced.
        if (turnDegrees <= 90.0)
        {
            Assert.True(backDotOn >= -1e-6, $"AFTER-FIX regression: weave-ON backsteps at a <=90deg corner (deg={turnDegrees}, seed={seed:X}): backDotOn={backDotOn:F4}");
        }

        // (b) the corner must never be WORSE (more negative) than the path's own inherent turn -- i.e. the
        // weave is not allowed to make an already-sharp turn look even more like it stepped backward.
        Assert.True(backDotOn >= backDotOff - 0.05,
            $"AFTER-FIX regression: weave-ON backsteps MORE than the turn itself (deg={turnDegrees}, seed={seed:X}): backDotOn={backDotOn:F4} vs turn-only backDotOff={backDotOff:F4}");

        // (c) TRUE continuity, not a magnitude heuristic: resampling 100x finer must shrink the max
        // frame-to-frame delta by close to the same 100x (a continuous/smooth curve's per-sample delta is
        // ~proportional to dt). A genuine discontinuity has a FIXED jump height independent of dt, so it
        // would NOT shrink -- that is precisely the pre-fix bug's signature (see PedBackstepProbeTests'
        // real trace: a ~0.235 m jump regardless of the render frame rate). Allow a generous margin
        // (shrink by at least 10x, not the full 100x) for floating-point/adaptive-window edge effects.
        var shrinkFactor = jumpOnFine > 1e-12 ? jumpOn / jumpOnFine : double.PositiveInfinity;
        Assert.True(shrinkFactor >= 10.0,
            $"AFTER-FIX regression: corner jump does NOT shrink under finer sampling (coarse={jumpOn:F5} @dt={coarseDt}, fine={jumpOnFine:F5} @dt={fineDt}, shrink={shrinkFactor:F1}x) -- looks like a genuine discontinuity, not a smooth sweep (deg={turnDegrees}, seed={seed:X})");
    }
}
