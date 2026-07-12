using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B6 (DESIGN.md "Two futures" -- live reactivity, a BEHAVIORAL bar, not golden-FCD parity):
// emergency LATERAL EVASION. When an external agent (a pedestrian who jumped off the sidewalk into
// the driving lane) is ahead in the car's path and the car CANNOT stop in time, the car SWERVES --
// drifting its lateral offset toward the open side of its lane, spilling into a safe adjacent lane
// if the ego lane can't clear the agent, and braking to a stop only as a last resort. Property tests
// against the engine's own TrajectorySet (no SUMO golden), mirroring RungB1/RungB5's idiom.
//
// Lateral convention: Kinematics.LatOffset is metres from the lane centre, positive = LEFT of travel;
// EmitTrajectory renders it into x/y (perpendicular to the lane), so a swerve is visible as the
// vehicle's y drifting off its lane-centre value. scenarios/14-external-obstacle is a single lane e0_0
// centred at y=-1.60 (width 3.2 m); the car (vClass passenger) is 1.8 m wide, 5 m long.
public class RungB6LateralEvasionTests
{
    private static readonly string OneLaneDir = Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle");
    private static readonly string TwoLaneDir = Path.Combine(RepoRoot(), "scenarios", "06-two-lane-cruise");
    private static readonly string WideLaneDir = Path.Combine(RepoRoot(), "scenarios", "_diag", "wide-swerve");
    private const double CarWidth = 1.8;
    private const double CarLength = 5.0;

    // ---- Test 1: pedestrian jumps in at the lane edge, too close to stop -> swerve WITHIN the lane. ----
    [Fact]
    public void SuddenPedestrianAtEdge_CarSwervesWithinLaneAndPassesWithoutStopping()
    {
        var engine = new Engine();
        Load(engine, OneLaneDir);
        const double laneCentreY = -1.60;

        // Jumps in at t=15 (car ~178 @ 13.89 m/s), ~8 m ahead, hugging the right edge:
        // latPos -1.2, width 0.8 -> footprint [-1.6,-0.8]. Too close to stop (brakeGap ~21 m >> gap).
        const double pedFront = 186.0, pedLen = 0.5, pedLat = -1.2, pedWidth = 0.8;
        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: pedFront, length: pedLen,
            startTime: 15.0, endTime: double.PositiveInfinity, latPos: pedLat, width: pedWidth);

        var traj = engine.Run(45);
        var pts = engine_PointsInOrder(traj);

        // Swerved: at some point the car is clearly off the lane centre (toward the LEFT, +y here).
        var maxLeftShift = pts.Max(p => (p.Y - laneCentreY));
        Assert.True(maxLeftShift > 0.4, $"car never swerved left off centre (max shift {maxLeftShift:F2} m)");

        // Passed the pedestrian while still moving (did NOT stop dead behind it).
        var passed = pts.Any(p => p.Pos > pedFront + CarLength && p.Speed > 1.0);
        Assert.True(passed, "car never got past the pedestrian while moving -- it stopped instead of swerving");

        // Recovered to free-flow by the end (proving it cleared and drove on).
        Assert.True(pts.Last().Speed > 13.0, $"car did not recover to free-flow (end speed {pts.Last().Speed:F2})");

        // NEVER collided: whenever the car and pedestrian overlap LONGITUDINALLY, their lateral
        // footprints must be disjoint.
        AssertNoCollision(pts, laneCentreY, pedFront, pedLen, pedLat, pedWidth, activeFrom: 15.0);
    }

    // ---- Test 2: pedestrian FILLS the lane, no room to dodge -> brake to a stop (pre-B6 behaviour). ----
    [Fact]
    public void PedestrianFillsLane_NoEscape_CarBrakesToStopAndStaysCentred()
    {
        var engine = new Engine();
        Load(engine, OneLaneDir);
        const double laneCentreY = -1.60;

        // Static, visible from the start, spanning the whole 3.2 m lane (latPos 0, width 3.0 ->
        // footprint [-1.5,1.5]): the car cannot fit past on either side, so it must stop behind it.
        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: 250.0, length: 5.0, latPos: 0.0, width: 3.0);

        var traj = engine.Run(60);
        var pts = engine_PointsInOrder(traj);

        var last = pts.Last();
        Assert.Equal(0.0, last.Speed, precision: 2);                 // stopped
        Assert.True(last.Pos <= 245.0 + 1e-6, $"overlapped obstacle back (pos {last.Pos:F2} > 245)");
        // Stayed centred (never swerved -- there was nowhere to go).
        var maxShift = pts.Max(p => Math.Abs(p.Y - laneCentreY));
        Assert.True(maxShift < 0.05, $"car swerved when it should have stopped centred (max shift {maxShift:F2} m)");
    }

    // ---- Test 3: pedestrian fills the RIGHT lane, car spills into the empty adjacent LEFT lane. ----
    [Fact]
    public void SuddenPedestrianFillsLane_CarSpillsIntoSafeAdjacentLane()
    {
        const double rightLaneCentreY = -4.80; // e0_0 (car starts here); left lane e0_1 centre is -1.60
        const double laneHalfWidth = 1.6;
        const double injectTime = 22.0;

        // Calibrate on a throwaway engine: where is veh0 (at full speed) at t=injectTime?
        var cal = new Engine();
        Load(cal, TwoLaneDir);
        var calPos = engine_PointsInOrder(cal.Run(60)).First(p => p.VehicleId == "veh0" && p.Time == injectTime).Pos;
        double pedFront = calPos + 8.0; // lane-filling pedestrian jumps in ~8 m ahead

        var engine = new Engine();
        Load(engine, TwoLaneDir);
        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: pedFront, length: 0.5,
            startTime: injectTime, endTime: double.PositiveInfinity, latPos: 0.0, width: 2.8);

        var traj = engine.Run(60);
        var pts = engine_PointsInOrder(traj).Where(p => p.VehicleId == "veh0").ToList();

        // Spilled LEFT past its own lane's boundary (LatOffset > half lane width) -> into e0_1.
        var maxLeftShift = pts.Max(p => (p.Y - rightLaneCentreY));
        Assert.True(maxLeftShift > laneHalfWidth,
            $"car did not spill into the adjacent lane (max left shift {maxLeftShift:F2} m <= {laneHalfWidth})");

        // Got past the pedestrian while moving (did not gridlock behind it).
        Assert.True(pts.Any(p => p.Pos > pedFront + CarLength && p.Speed > 1.0),
            "car never passed the pedestrian while moving");

        AssertNoCollision(pts, rightLaneCentreY, pedFront, 0.5, 0.0, 2.8, activeFrom: injectTime);
    }

    // ---- Test 4 (inertness): a Width=0 obstacle is a FULL-LANE block -> stops dead, exactly as B1. ----
    [Fact]
    public void ZeroWidthObstacle_BehavesAsFullLaneBlock_StopsDead()
    {
        var engine = new Engine();
        Load(engine, OneLaneDir);
        const double laneCentreY = -1.60;

        engine.AddObstacle(engine.GetLane("e0_0"), frontPos: 250.0, length: 5.0); // no latPos/width -> full lane

        var traj = engine.Run(60);
        var pts = engine_PointsInOrder(traj);

        var last = pts.Last();
        Assert.Equal(242.499, last.Pos, precision: 3); // the exact B1 Krauss steady gap
        Assert.Equal(0.0, last.Speed, precision: 3);
        Assert.True(pts.All(p => Math.Abs(p.Y - laneCentreY) < 1e-9), "a full-lane obstacle must never induce a swerve");
    }

    // ---- Test 5: LUNGING pedestrian -> car dodges to the side the ped is VACATING (predictive). ----
    // A pedestrian in the car's path lunges LEFT faster than the car's own 2 m/s swerve. A naive
    // snapshot dodge (ped currently dead-centre) would steer LEFT, straight into the ped's path; the
    // predictive evasion sees the ped heading left and steers RIGHT instead, behind its motion.
    [Fact]
    public void LungingPedestrian_CarDodgesToTheSideItIsVacating_NoCollision()
    {
        const double injectTime = 10.0, laneCentreY = -3.00;
        const double pedLat0 = 0.0, pedLatSpeed = 2.5, pedWidth = 1.2, pedLen = 0.5;
        var engine = new Engine();
        Load(engine, WideLaneDir);
        double pedFront = 13.89 * injectTime + 8.0; // wide-swerve car cruises at departSpeed from pos 0

        engine.AddMovingObstacle(engine.GetLane("e0_0"), frontPos: pedFront, length: pedLen, speed: 0.0, maxDecel: 0.0,
            startTime: injectTime, endTime: double.PositiveInfinity, latPos: pedLat0, width: pedWidth, latSpeed: pedLatSpeed);

        var pts = engine_PointsInOrder(engine.Run(30)).Where(p => p.VehicleId == "car0").ToList();

        // Dodged RIGHT (negative offset) -- AWAY from the leftward-lunging ped, NOT left into it.
        var minOffset = pts.Min(p => p.Y - laneCentreY);
        var maxOffset = pts.Max(p => p.Y - laneCentreY);
        Assert.True(minOffset < -0.3, $"car did not dodge right away from the lunging ped (min offset {minOffset:F2})");
        Assert.True(maxOffset < 0.1, $"car steered LEFT into the ped's path (max offset {maxOffset:F2} should stay <= 0)");

        // Never collided: the ped's lateral centre moves at pedLatSpeed once active (dead-reckoned).
        foreach (var p in pts)
        {
            var pedLatNow = p.Time <= injectTime ? pedLat0 : pedLat0 + pedLatSpeed * (p.Time - injectTime);
            var carBack = p.Pos - CarLength;
            if (!(carBack < pedFront && p.Pos > pedFront - pedLen))
            {
                continue; // no longitudinal overlap this step
            }

            var off = p.Y - laneCentreY;
            var carLo = off - CarWidth / 2.0;
            var carHi = off + CarWidth / 2.0;
            var pedLo = pedLatNow - pedWidth / 2.0;
            var pedHi = pedLatNow + pedWidth / 2.0;
            Assert.False(carLo < pedHi - 1e-6 && carHi > pedLo + 1e-6,
                $"COLLISION at t={p.Time}: car [{carLo:F2},{carHi:F2}] overlaps ped [{pedLo:F2},{pedHi:F2}]");
        }
    }

    // --- helpers ---

    private static void Load(Engine engine, string dir) => engine.LoadScenario(
        Path.Combine(dir, "net.net.xml"), Path.Combine(dir, "rou.rou.xml"), Path.Combine(dir, "config.sumocfg"));

    private sealed record Pt(string VehicleId, double Time, double Pos, double Speed, double X, double Y);

    private static List<Pt> engine_PointsInOrder(TrajectorySet traj) =>
        traj.AllPoints.Select(p => new Pt(p.VehicleId, p.Time, p.Pos, p.Speed, p.X, p.Y))
            .OrderBy(p => p.Time).ToList();

    // Whenever the car and pedestrian overlap LONGITUDINALLY (their [back,front] spans intersect), the
    // lateral footprints (car [latOffset +/- 0.9], ped [pedLat +/- pedWidth/2]) must be DISJOINT.
    private static void AssertNoCollision(
        List<Pt> pts, double laneCentreY, double pedFront, double pedLen, double pedLat, double pedWidth, double activeFrom)
    {
        var pedBack = pedFront - pedLen;
        var pedLo = pedLat - pedWidth / 2.0;
        var pedHi = pedLat + pedWidth / 2.0;
        foreach (var p in pts)
        {
            if (p.Time < activeFrom)
            {
                continue;
            }

            var carBack = p.Pos - CarLength;
            var longitudinalOverlap = carBack < pedFront && p.Pos > pedBack;
            if (!longitudinalOverlap)
            {
                continue;
            }

            var latOffset = p.Y - laneCentreY;
            var carLo = latOffset - CarWidth / 2.0;
            var carHi = latOffset + CarWidth / 2.0;
            var lateralOverlap = carLo < pedHi - 1e-6 && carHi > pedLo + 1e-6;
            Assert.False(lateralOverlap,
                $"COLLISION at t={p.Time}: car lat [{carLo:F2},{carHi:F2}] overlaps ped [{pedLo:F2},{pedHi:F2}] while longitudinally overlapping (car pos {p.Pos:F2}, ped [{pedBack:F2},{pedFront:F2}])");
        }
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
