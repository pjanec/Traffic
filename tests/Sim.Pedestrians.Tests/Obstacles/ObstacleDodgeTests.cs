using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Obstacles;

// POC-5 (docs/PEDESTRIAN-POC-PLAN.md POC-5; docs/PEDESTRIAN-DESIGN.md §3a/§6): pedestrians dodge a
// static box obstacle (a parked-car footprint) and a moving external entity, driven entirely through
// OrcaCrowd's EXISTING AddObstacle/SetExternalObstacles inputs plus the new BoxObstacle/MovingBlocker
// helpers (src/Sim.Pedestrians/Obstacles/) -- no change to OrcaCrowd itself. Asserts POC-5 success
// conditions 1 and 2.
public class ObstacleDodgeTests
{
    private readonly ITestOutputHelper _out;

    public ObstacleDodgeTests(ITestOutputHelper output) => _out = output;

    private const double MaxSpeed = 1.4;   // m/s
    private const double Radius = 0.3;     // m
    private const double ArriveRadius = 0.3; // m
    private const double Dt = 0.1;         // s
    private const int MaxSteps = 3000;

    // Same "small eps" convention used throughout the pedestrian POCs for no-overlap assertions
    // (see SumoBakeNavigationTests / PedLodManagerTests).
    private const double OverlapEps = 1e-3;

    // A straight sidewalk strip: x in [0,40], y in [0,6.2]. Wide enough to walk 8 peds abreast with
    // margin (spacing kept > the peds' combined radii so the baseline stream itself never starts
    // pre-overlapped -- see StartYs below), narrow enough that a mid-width box obstacle forces a
    // real detour for about half the stream.
    private static readonly Vec2 SidewalkMin = new(0.0, 0.0);
    private static readonly Vec2 SidewalkMax = new(40.0, 6.2);

    // A parked-car-sized box centered on the sidewalk's midline, occupying its middle band (y in
    // [1.8,4.4]) -- leaves a 1.8 m clear gap on each side (y in [0,1.8] and [4.4,6.2]), enough for a
    // 0.3 m-radius ped to pass, while about half the stream's starting lanes sit inside the blocked
    // band and must detour around it.
    private static readonly Vec2 BoxCenter = new(20.0, 3.1);
    private const double BoxHalfX = 1.5;
    private const double BoxHalfY = 1.3;

    private static Vec2[] BuildBox() => BoxObstacle.Corners(BoxCenter, BoxHalfX, BoxHalfY, angleRadians: 0.0);

    // 8 peds spread evenly across the sidewalk's width, walking straight across (same y as their
    // start) so half of them have the box directly in their path and must dodge; the rest pass the
    // box's y-band cleanly. Spacing (0.8 m) is kept comfortably above the peds' combined radii
    // (2*Radius = 0.6 m) so the baseline (no-box) stream starts and stays collision-free by
    // construction -- this scenario is about the BOX, not incidental ped-ped crowding.
    private static readonly double[] StartYs = Linspace(0.3, 5.9, 8);

    private static double[] Linspace(double a, double b, int n)
    {
        var result = new double[n];
        for (var i = 0; i < n; i++)
        {
            result[i] = a + (b - a) * i / (n - 1);
        }

        return result;
    }

    // Builds each ped's path: a direct 2-point [start, goal] for lanes that never cross the box's
    // y-band, or a 3-point [start, dodge-waypoint, goal] for lanes that do -- the dodge waypoint sits
    // just past the box in x, at the nearest CLEAR gap's mid-y. This is exactly the tactical-steering
    // role WaypointFollower/PedRouteController already play in POC-1 (reused here, not reinvented):
    // a bare same-y goal gives OrcaCrowd's local reactive solve no PERSISTENT lateral pull once it
    // reaches the box's flat near edge, which is a known, already-documented ORCA characteristic (see
    // tests/Sim.ParityTests/OrcaStaticObstacleTests.cs "local avoidance is not routing" remarks) --
    // not a defect in the obstacle avoidance itself (confirmed separately: MinBoxDistance below stays
    // clean because the AVOIDANCE constraint is honoured; it is the STEERING TARGET, supplied here
    // exactly like a strategic layer would, that keeps the flow moving instead of freezing at the edge).
    private static IReadOnlyList<Vec2> BuildPedPath(double startY, Vec2 goal, bool withBox)
    {
        var start = new Vec2(1.0, startY);
        if (!withBox)
        {
            return new[] { start, goal }; // no box in this run -- nothing to dodge, straight line
        }

        var bandMin = BoxCenter.Y - BoxHalfY;
        var bandMax = BoxCenter.Y + BoxHalfY;

        if (startY <= bandMin || startY >= bandMax)
        {
            return new[] { start, goal }; // never in the box's y-band -- no dodge needed
        }

        var lowerGapMid = bandMin / 2.0;
        var upperGapMid = (bandMax + SidewalkMax.Y) / 2.0;
        var dodgeY = Math.Abs(startY - lowerGapMid) <= Math.Abs(startY - upperGapMid) ? lowerGapMid : upperGapMid;
        var dodgeWaypoint = new Vec2(BoxCenter.X + BoxHalfX + 2.0, dodgeY); // clear of the box in x
        return new[] { start, dodgeWaypoint, goal };
    }

    // Runs the N-ped crossing stream, optionally with the box obstacle, until every ped arrives or
    // MaxSteps is hit. Returns the step count (throughput proxy: fewer steps = faster drain) and the
    // minimum ped-center-to-box-interior distance observed (withBox only).
    private static (int Steps, bool AllArrived, double MinBoxDistance, List<Vec2[]> Trajectory) RunStream(bool withBox)
    {
        var crowd = new OrcaCrowd();
        var box = BuildBox();
        if (withBox)
        {
            crowd.AddObstacle(box);
        }

        var indices = new OrcaHandle[StartYs.Length];
        var goals = new Vec2[StartYs.Length];
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        for (var i = 0; i < StartYs.Length; i++)
        {
            var goal = new Vec2(39.0, StartYs[i]);
            goals[i] = goal;
            var path = BuildPedPath(StartYs[i], goal, withBox);
            var index = crowd.Add(path[0], Radius, MaxSpeed, goal: path[0]);
            indices[i] = index;
            controller.AddRoute(index, path, MaxSpeed);
        }

        var trajectory = new List<Vec2[]>();
        var minBoxDistance = double.MaxValue;
        var steps = 0;
        var allArrived = false;
        for (; steps < MaxSteps; steps++)
        {
            controller.Update();
            crowd.Step(Dt);
            var frame = new Vec2[indices.Length];
            var arrivedCount = 0;
            for (var i = 0; i < indices.Length; i++)
            {
                var pos = crowd.Position(indices[i]);
                frame[i] = pos;
                if (withBox)
                {
                    var nearest = BoxObstacle.NearestPoint(box, pos);
                    var dist = (nearest - pos).Abs;
                    // NearestPoint returns `pos` itself (distance 0) when pos is INSIDE the box, which
                    // correctly registers as a violation via the assertion below (distance 0 < radius).
                    minBoxDistance = Math.Min(minBoxDistance, dist);
                }

                if ((pos - goals[i]).Abs <= ArriveRadius && controller.IsRouteComplete(indices[i]))
                {
                    arrivedCount++;
                }
            }

            trajectory.Add(frame);
            if (arrivedCount == indices.Length)
            {
                allArrived = true;
                steps++; // count this step
                break;
            }
        }

        return (steps, allArrived, minBoxDistance, trajectory);
    }

    [Fact]
    public void PedStream_DodgesStaticBox_NoOverlap_AllArrive_ThroughputDropsButNonzero()
    {
        var baseline = RunStream(withBox: false);
        Assert.True(baseline.AllArrived, "baseline (no box) stream never fully arrived");

        var withBox = RunStream(withBox: true);

        Assert.True(withBox.AllArrived, "ped stream jammed at the box and never fully arrived (zero throughput)");

        _out.WriteLine($"[POC-5 measured] baseline drain: {baseline.Steps} steps ({baseline.Steps * Dt:F1}s); "
            + $"withBox drain: {withBox.Steps} steps ({withBox.Steps * Dt:F1}s); "
            + $"min ped-to-box distance: {withBox.MinBoxDistance:F4} m (radius {Radius:F2} m)");

        // Success condition 1a: no ped ever overlaps the box's interior.
        Assert.True(withBox.MinBoxDistance >= Radius - OverlapEps,
            $"a ped overlapped the box: min ped-to-box distance {withBox.MinBoxDistance:F4} < radius {Radius:F4} - eps");

        // Success condition 1b: throughput (time to drain the stream) drops vs. the no-box baseline,
        // but stays finite/positive -- peds still get through, they are just slower.
        Assert.True(withBox.Steps > baseline.Steps,
            $"expected the box to cost throughput (more steps to drain): baseline={baseline.Steps} steps, withBox={withBox.Steps} steps");

        // Also confirm every ped in the box run individually reached its own goal (not just an
        // aggregate arrived-count coincidence): recompute final distances from the last frame.
        var lastFrame = withBox.Trajectory[^1];
        for (var i = 0; i < StartYs.Length; i++)
        {
            var goal = new Vec2(39.0, StartYs[i]);
            Assert.True((lastFrame[i] - goal).Abs <= ArriveRadius,
                $"ped {i} (start y={StartYs[i]:F2}) did not arrive: final={lastFrame[i]}, goal={goal}");
        }
    }

    [Fact]
    public void MovingBlocker_IsAvoided_NoOverlap_PedArrives()
    {
        // A single ped walking straight east; a moving blocker crosses its path north-to-south,
        // timed so a naive (non-avoiding) straight-line walker would actually collide with it -- not
        // just pass nearby. Ped covers 20 m at 1.4 m/s -> reaches the path midpoint (x=10) at
        // t = 10/1.4 = 7.143s. Blocker starts directly above x=10 and walks straight down at 1.2 m/s,
        // timed to cross y=0 at that SAME instant (y0 = 1.2 * 7.143 = 8.571), i.e. a genuine intercept
        // course: without avoidance the two footprints would overlap almost exactly there.
        var start = new Vec2(0.0, 0.0);
        var goal = new Vec2(20.0, 0.0);
        const double pedRadius = Radius;
        const double blockerRadius = 0.35;
        var blocker = new MovingBlocker(new Vec2(10.0, 8.571), new Vec2(0.0, -1.2), blockerRadius);

        var crowd = new OrcaCrowd();
        var pedIndex = crowd.Add(start, pedRadius, MaxSpeed, goal);

        var sumOfRadii = pedRadius + blockerRadius;
        var minSeparation = double.MaxValue;
        var steps = 0;
        var arrived = false;
        for (; steps < MaxSteps; steps++)
        {
            crowd.SetExternalObstacles(new[] { blocker.ToWorldDisc() });
            crowd.Step(Dt);
            blocker.Advance(Dt);

            var pedPos = crowd.Position(pedIndex);
            var separation = (pedPos - blocker.Position).Abs;
            minSeparation = Math.Min(minSeparation, separation);

            if ((pedPos - goal).Abs <= ArriveRadius)
            {
                arrived = true;
                steps++;
                break;
            }
        }

        Assert.True(arrived, "ped never reached its goal while avoiding the moving blocker");

        _out.WriteLine($"[POC-5 measured] min ped-blocker separation: {minSeparation:F4} m (sum-of-radii {sumOfRadii:F4} m); "
            + $"arrived in {steps} steps ({steps * Dt:F1}s)");

        // Success condition 2: no ped-center-to-disc overlap over the whole run.
        Assert.True(minSeparation >= sumOfRadii - OverlapEps,
            $"ped overlapped the moving blocker: min separation {minSeparation:F4} < sum-of-radii {sumOfRadii:F4} - eps");
    }

    [Fact]
    public void Determinism_BoxStreamAndMovingBlocker_AreIdenticalAcrossIndependentRuns()
    {
        var run1 = RunStream(withBox: true);
        var run2 = RunStream(withBox: true);

        Assert.Equal(run1.Steps, run2.Steps);
        Assert.Equal(run1.Trajectory.Count, run2.Trajectory.Count);
        for (var f = 0; f < run1.Trajectory.Count; f++)
        {
            for (var i = 0; i < run1.Trajectory[f].Length; i++)
            {
                Assert.Equal(run1.Trajectory[f][i].X, run2.Trajectory[f][i].X, precision: 12);
                Assert.Equal(run1.Trajectory[f][i].Y, run2.Trajectory[f][i].Y, precision: 12);
            }
        }

        var (steps1, traj1) = RunMovingBlockerTrajectory();
        var (steps2, traj2) = RunMovingBlockerTrajectory();
        Assert.Equal(steps1, steps2);
        Assert.Equal(traj1.Count, traj2.Count);
        for (var i = 0; i < traj1.Count; i++)
        {
            Assert.Equal(traj1[i].X, traj2[i].X, precision: 12);
            Assert.Equal(traj1[i].Y, traj2[i].Y, precision: 12);
        }
    }

    private static (int Steps, List<Vec2> Trajectory) RunMovingBlockerTrajectory()
    {
        var start = new Vec2(0.0, 0.0);
        var goal = new Vec2(20.0, 0.0);
        var blocker = new MovingBlocker(new Vec2(10.0, 8.571), new Vec2(0.0, -1.2), 0.35);

        var crowd = new OrcaCrowd();
        var pedIndex = crowd.Add(start, Radius, MaxSpeed, goal);

        var trajectory = new List<Vec2>();
        var steps = 0;
        for (; steps < MaxSteps; steps++)
        {
            crowd.SetExternalObstacles(new[] { blocker.ToWorldDisc() });
            crowd.Step(Dt);
            blocker.Advance(Dt);
            var pos = crowd.Position(pedIndex);
            trajectory.Add(pos);
            if ((pos - goal).Abs <= ArriveRadius)
            {
                break;
            }
        }

        return (steps, trajectory);
    }
}
