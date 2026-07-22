using Sim.Core;
using Sim.Viewer.Motion;
using Xunit;

namespace Sim.Viewer.Motion.Tests;

// docs/IGBRIDGE-DECISIONS.md §5.3: the kinematic rear-axle drag. The front reference tows a no-slip rear
// axle so the body pivots at the rear (real car) instead of swinging the rear around a front pivot. These
// pin: straight -> rear trails directly behind; a hard front corner -> heading rotates CONTINUOUSLY (no
// one-step snap) with the rear cutting inside; hold at a standstill; determinism.
public sealed class KinematicHeadingTests
{
    private const double V = 10.0;    // m/s
    private const double Len = 5.0;   // m
    private const float Dt = 0.05f;   // 20 Hz

    private static VehicleHandle H => new(1, 0);

    private static (double, double) Dir(float naviDeg)
    {
        var m = (90.0 - naviDeg) * System.Math.PI / 180.0;
        return (System.Math.Cos(m), System.Math.Sin(m));
    }

    [Fact]
    public void Straight_RearTrailsDirectlyBehindFront_HeadingHoldsLaneDirection()
    {
        var k = new KinematicHeading();
        KinematicPose p = default;
        // Drive due east (navi 90 = +X) for 2 s.
        for (var i = 1; i <= 40; i++)
        {
            p = k.Update(H, i * V * Dt, 0.0, 90f, V, Len, Dt);
        }

        Assert.InRange(p.HeadingDeg, 89f, 91f);          // heading == lane direction
        Assert.True(p.RearX < p.CenterX && p.CenterX < p.FrontX, "rear should trail behind the front (west)");
        Assert.InRange(p.RearY, -0.01, 0.01);            // no lateral offset on a straight
    }

    [Fact]
    public void HardCorner_HeadingRotatesContinuously_NoOneStepSnap()
    {
        var k = new KinematicHeading();
        // Front path: east along y=0 to x=30, then a HARD 90-degree corner north (x=30, y increasing).
        // The raw front tangent would snap 90 deg in one step here; the drag must spread it.
        var headings = new System.Collections.Generic.List<float>();
        double x = 0, y = 0;
        // 200 steps: ~60 to reach the corner (x=30 at 0.5 m/step), then long enough north for the heading to
        // fully settle to north — the anticipatory turn-in (on by default) adds a little convergence lag.
        for (var i = 0; i < 200; i++)
        {
            if (x < 30.0)
            {
                x += V * Dt;                 // heading in the path: due east
            }
            else
            {
                y += V * Dt;                 // due north after the corner
            }

            var laneHeading = x < 30.0 ? 90f : 0f;
            var p = k.Update(H, x, y, laneHeading, V, Len, Dt);
            headings.Add(p.HeadingDeg);
        }

        // No single emit step rotates the body more than a bounded amount (the drag integrates the corner
        // over the wheelbase distance instead of snapping it). A raw tangent would show a ~90 deg step.
        float maxStep = 0;
        for (var i = 1; i < headings.Count; i++)
        {
            var d = System.Math.Abs(((headings[i] - headings[i - 1] + 540f) % 360f) - 180f);
            maxStep = System.Math.Max(maxStep, d);
        }

        Assert.True(maxStep < 30f, $"body heading snapped {maxStep:F1} deg in one step (expected a smooth drag)");
        // And the turn actually completed: end heading is ~north (0/360).
        Assert.True(headings[^1] < 3f || headings[^1] > 357f, $"end heading not ~north: {headings[^1]}");
    }

    [Fact]
    public void BelowHoldSpeed_HoldsHeading()
    {
        var k = new KinematicHeading(new KinematicHeadingParams { HoldSpeed = 0.5 });
        var p0 = k.Update(H, 0.0, 0.0, 90f, V, Len, Dt);           // moving: establishes heading ~90
        var p1 = k.Update(H, 0.001, 0.0, 270f, 0.0, Len, Dt);      // stopped, lane says 270 -> must HOLD ~90
        Assert.InRange(p1.HeadingDeg, 88f, 92f);
    }

    [Fact]
    public void IsDeterministic()
    {
        KinematicPose Run()
        {
            var k = new KinematicHeading();
            KinematicPose p = default;
            for (var i = 1; i <= 50; i++)
            {
                var x = i * V * Dt;
                var (dx, dy) = Dir(45f);
                p = k.Update(H, x * dx, x * dy, 45f, V, Len, Dt);
            }

            return p;
        }

        var a = Run();
        var b = Run();
        Assert.Equal(a.HeadingDeg, b.HeadingDeg);
        Assert.Equal(a.CenterX, b.CenterX, precision: 12);
        Assert.Equal(a.CenterY, b.CenterY, precision: 12);
    }
}
