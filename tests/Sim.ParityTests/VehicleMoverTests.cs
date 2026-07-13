using Sim.Core.Orca;
using Sim.Evac;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE3-TASKS.md T1.2 / T2.1: standalone unit tests for VehicleMover (wraps
// Sim.Core.Mixed.MixedTrafficCrowd) and FakeNavMesh.BandWalls -- no Engine, no EvacDirector. Verifies
// the shaped, non-holonomic free-space push moves toward a reachable goal, is confined by band walls,
// reports wedged only after the configured dwell, and steers rather than teleports sideways.
public class VehicleMoverTests
{
    private readonly ITestOutputHelper _out;
    public VehicleMoverTests(ITestOutputHelper output) => _out = output;

    private static readonly (Vec2 A, Vec2 B)[] BigBoxWalls =
    {
        (new Vec2(0, 0), new Vec2(0, 100)),
        (new Vec2(0, 100), new Vec2(100, 100)),
        (new Vec2(100, 100), new Vec2(100, 0)),
        (new Vec2(100, 0), new Vec2(0, 0)),
    };

    private static (Vec2 A, Vec2 B)[] BoxWalls(double minX, double minY, double maxX, double maxY) => new[]
    {
        (new Vec2(minX, minY), new Vec2(minX, maxY)),
        (new Vec2(minX, maxY), new Vec2(maxX, maxY)),
        (new Vec2(maxX, maxY), new Vec2(maxX, minY)),
        (new Vec2(maxX, minY), new Vec2(minX, minY)),
    };

    // ----- T1.2(a) / DESIGN §4: moves toward a reachable goal, stays inside the box -----
    [Fact]
    public void MovesTowardReachableGoal_WithoutCrossingWalls()
    {
        var mover = new VehicleMover(new EvacConfig());
        mover.ArmWalls(BigBoxWalls);

        var start = new Vec2(10, 50);
        var goal = new Vec2(90, 50);
        var i = mover.AddCar(start, headingRad: 0.0, goal);

        var startDist = (goal - start).Abs;

        for (var step = 0; step < 40; step++)
        {
            mover.Step(1.0);
            var p = mover.Position(i);
            Assert.InRange(p.X, 0.0, 100.0);
            Assert.InRange(p.Y, 0.0, 100.0);
        }

        var finalDist = (goal - mover.Position(i)).Abs;
        _out.WriteLine($"startDist={startDist:F2} finalDist={finalDist:F2} pos={mover.Position(i).X:F2},{mover.Position(i).Y:F2}");

        Assert.True(startDist - finalDist > 50.0,
            $"expected the car to close by > 50 m, closed by {startDist - finalDist:F2}");
    }

    // ----- T2.1 / DESIGN §4: band walls confine a mover driven toward an outside goal -----
    [Fact]
    public void BandWalls_ConfineMoverDrivenOutsideTheBox()
    {
        var mover = new VehicleMover(new EvacConfig());
        mover.ArmWalls(BigBoxWalls);

        var i = mover.AddCar(new Vec2(50, 50), headingRad: 0.0, goal: new Vec2(500, 50));

        double maxX = double.NegativeInfinity;
        for (var step = 0; step < 60; step++)
        {
            mover.Step(1.0);
            var p = mover.Position(i);
            maxX = Math.Max(maxX, p.X);
        }

        _out.WriteLine($"maxX={maxX:F3}");

        // Wall thickness (default 1.0) is centred on the boundary line plus the shaped-VO tracking
        // safety margin (OrcaPushSafetyMargin=0.3) means the true car body stops shy of x=100; allow a
        // small tolerance for wall thickness / integration overshoot, well short of the far goal (500).
        Assert.True(maxX <= 100.0 + 2.5, $"mover crossed the band wall: maxX={maxX:F3}");
    }

    // ----- T1.2(b) / T4.3: wedge reports false before the dwell, true after -----
    [Fact]
    public void Wedge_FalseBeforeDwell_TrueAfter()
    {
        var cfg = new EvacConfig(); // OrcaWedgeDwellSeconds=3.0, OrcaWedgeSpeed=0.1
        var mover = new VehicleMover(cfg);
        // Tiny box: a ~4.3 m long car barely fits and cannot reach a goal through the wall.
        mover.ArmWalls(BoxWalls(0, 0, 6, 6));

        var i = mover.AddCar(new Vec2(3, 3), headingRad: 0.0, goal: new Vec2(100, 3));

        var wedgedAtStep = -1;
        for (var step = 1; step <= 8; step++)
        {
            mover.Step(1.0);
            var wedged = mover.IsWedged(i);
            _out.WriteLine($"step={step} pos={mover.Position(i).X:F3},{mover.Position(i).Y:F3} vel={mover.Velocity(i).Abs:F4} wedged={wedged}");

            if (step == 1)
            {
                Assert.False(wedged, "should not be wedged after just 1 step (dwell is 3s)");
            }

            if (wedged && wedgedAtStep < 0)
            {
                wedgedAtStep = step;
            }
        }

        _out.WriteLine($"wedgedAtStep={wedgedAtStep}");
        Assert.True(wedgedAtStep > 0 && wedgedAtStep <= 8, $"expected wedged by step 8, got {wedgedAtStep}");
    }

    // ----- T1.2(c) / T4: non-holonomic -- no pure sideways teleport for a 90-degree goal -----
    [Fact]
    public void NonHolonomic_DoesNotTeleportSideways()
    {
        var mover = new VehicleMover(new EvacConfig());
        mover.ArmWalls(BigBoxWalls);

        var start = new Vec2(50, 50);
        // Goal is directly "ahead-left" of the car's heading (facing +x), 90 degrees away.
        var goal = new Vec2(50, 90);
        var i = mover.AddCar(start, headingRad: 0.0, goal);
        var headingBefore = mover.Heading(i);

        mover.Step(1.0);

        var p = mover.Position(i);
        var dx = p.X - start.X;
        var dy = p.Y - start.Y;
        var dispMag = Math.Sqrt(dx * dx + dy * dy);
        var headingAfter = mover.Heading(i);

        _out.WriteLine($"dx={dx:F4} dy={dy:F4} disp={dispMag:F4} headingBefore={headingBefore:F4} headingAfter={headingAfter:F4}");

        // Non-holonomic kinematic-bicycle steering: a car cannot pivot in place nor slide sideways.
        // Displacement this single step is bounded by what the class can physically cover, and the
        // car must not have translated straight toward the goal without turning (a pure holonomic
        // sideways jump would put nearly all displacement into dy with ~zero dx and no heading change).
        Assert.True(dispMag <= VehicleClassCarMaxSpeed * 1.0 + 1e-6,
            $"displacement {dispMag:F4} exceeds maxSpeed*dt");
        Assert.False(dy > 0.5 && Math.Abs(dx) < 1e-6 && Math.Abs(headingAfter - headingBefore) < 1e-6,
            "car appears to have translated sideways without turning or moving forward at all");
    }

    private const double VehicleClassCarMaxSpeed = 14.0; // Sim.Core.Mixed.VehicleClass.Car.MaxSpeed

    // ----- PRELIM (CreepSpeed fix, B1 review carry-over): creep breaks the lateral-goal deadlock -----
    // MixedTrafficCrowd.SteerNonholonomic sets targetSpeed = max(min(CreepSpeed, desired),
    // desired*cos(headingErr)); with CreepSpeed=0 a pusher whose goal is ~90 deg off its heading gets
    // targetSpeed 0 and DEADLOCKS (it can never nudge-and-turn onto the shoulder, since turn rate is
    // tied to forward speed). EvacConfig.OrcaCreepSpeed (default 0.5) fixes this. This test supersedes
    // the degenerate "stays put" outcome noted in the B1 review: with creep, over many steps the car
    // should make real progress toward a 90-deg-lateral goal AND its heading should visibly rotate
    // toward it.
    [Fact]
    public void ReorientsTowardLateralGoal_WithCreep()
    {
        var mover = new VehicleMover(new EvacConfig());   // OrcaCreepSpeed defaults to 0.5
        mover.ArmWalls(BigBoxWalls);

        var start = new Vec2(50, 50);
        // Goal is 90 degrees to the car's initial heading (facing +x, goal straight "up" in +y).
        var goal = new Vec2(50, 90);
        var i = mover.AddCar(start, headingRad: 0.0, goal);
        var headingBefore = mover.Heading(i);

        for (var step = 0; step < 30; step++)
        {
            mover.Step(1.0);
        }

        var p = mover.Position(i);
        var headingAfter = mover.Heading(i);
        var deltaY = p.Y - start.Y;

        _out.WriteLine(
            $"pos={p.X:F2},{p.Y:F2} deltaY={deltaY:F2} headingBefore={headingBefore:F4} headingAfter={headingAfter:F4} " +
            $"targetHeading={Math.PI / 2:F4}");

        Assert.True(deltaY > 10.0, $"expected Y to advance by > 10 m via creep, only advanced {deltaY:F2}");

        // Heading should have rotated toward +y (pi/2), i.e. it is now strictly closer to pi/2 than the
        // starting heading (0.0) was -- proving creep let the car steer, not just crawl straight.
        var distBefore = Math.Abs(NormalizeAngle(headingBefore - Math.PI / 2.0));
        var distAfter = Math.Abs(NormalizeAngle(headingAfter - Math.PI / 2.0));
        Assert.True(distAfter < distBefore,
            $"expected heading to rotate toward +y (pi/2): before-dist={distBefore:F4} after-dist={distAfter:F4}");
    }

    private static double NormalizeAngle(double a)
    {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
