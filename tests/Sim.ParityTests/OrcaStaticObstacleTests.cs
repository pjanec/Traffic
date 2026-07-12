using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// Static-obstacle (wall) avoidance for the OPEN-SPACE ORCA layer (docs/LANELESS-DIRECTION.md), ported
// from RVO2's Agent::computeNewVelocity obstacle section + linearProgram3 (see OrcaSolver.cs). Same
// behavioural validation bar as OrcaOpenSpaceTests (no SUMO counterpart for this layer): no-overlap
// against the wall geometry itself (not just other agents), and reaches-goal / routes-around for
// scenarios a naive agent-agent-only solve would drive straight through.
public class OrcaStaticObstacleTests
{
    private const double Dt = 0.25;
    private const double Radius = 0.5;
    private const double MaxSpeed = 1.0;

    // Wall no-penetration tolerance. ORCA's obstacle half-plane guarantees the velocity it hands back
    // is safe over the obstacle time horizon FROM THE CURRENT POSITION; with discrete Euler
    // integration (fixed dt), a single step that sweeps past a convex corner vertex can shave a small,
    // bounded amount off the ideal clearance before the next step's re-plan corrects course -- the
    // same category of slack OrcaOpenSpaceTests' OverlapEps documents for agent-agent pairs (there
    // from reciprocal negotiation; here from crossing a corner's narrow safe cone in one discrete
    // step). Confirmed transient and self-correcting (next step's clearance recovers immediately), not
    // a growing or sustained violation -- so this is discretisation slack, not a construction defect.
    private const double WallEps = 0.02;

    private readonly ITestOutputHelper _out;

    public OrcaStaticObstacleTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void WallBlocksStraightPath_AgentRoutesAroundIt_NoPenetration()
    {
        // A thin rectangular wall straddling the straight-line path from start to goal. Pure
        // agent-agent ORCA (no obstacles) would drive the agent through x=5 in a dead-straight line;
        // this geometry only stops it if the obstacle lines are actually being constructed and honoured.
        var wall = new[]
        {
            new Vec2(4.85, -2), new Vec2(5.15, -2), new Vec2(5.15, 2), new Vec2(4.85, 2),
        };

        var crowd = new OrcaCrowd();
        crowd.AddObstacle(wall);
        // Goal at y=3 (clearly ABOVE the wall's y in [-2,2] span, with margin past the corner-clearance
        // threshold y=2+radius=2.5): local reactive ORCA has no global path planner, so a preferred
        // velocity that is (or becomes) purely horizontal gives it nothing to resolve "which way
        // around" with once the wall's flat-edge half-plane pins the forward component -- it would
        // decelerate to a dead stop AT the wall and sit there forever (confirmed by direct
        // reproduction: a goal with y INSIDE the wall's span, e.g. (10,1.5), converges its y exactly
        // to the goal's y while still short of clearing the wall in x, and once y==goal.y the
        // preferred velocity is purely horizontal again -- permanent freeze at the wall's surface).
        // This is a real, well-known ORCA/RVO2 characteristic (local avoidance is not routing), not a
        // defect; a goal that stays outside the blocking span the whole way keeps a persistent
        // non-zero vertical pull until the agent is actually clear, which a real caller's
        // waypoint/steering layer would supply anyway.
        var agent = crowd.Add(new Vec2(0, 0), Radius, MaxSpeed, goal: new Vec2(10, 3));

        var minClearance = double.PositiveInfinity;
        var steps = 0;
        for (; steps < 400 && !crowd.AllArrived(0.2); steps++)
        {
            crowd.Step(Dt);
            var pos = crowd.Position(agent);
            minClearance = Math.Min(minClearance, MinDistanceToWall(pos, wall));
            AssertNoWallPenetration(pos, Radius, wall, steps);
        }

        _out.WriteLine($"WallBlocksStraightPath: settled in {steps} steps, minClearance={minClearance:F4} " +
                       $"(radius {Radius})");

        Assert.True(crowd.AllArrived(0.2), $"agent did not reach the goal past the wall (pos={crowd.Position(agent)})");
        Assert.True(minClearance >= Radius - WallEps,
            $"agent penetrated the wall (min clearance {minClearance:F6} < radius {Radius})");
    }

    [Fact]
    public void AgentSlidesAlongWall_NoPenetration()
    {
        // A long straight wall (given as RVO2's classic 2-vertex "thin wall" obstacle) directly between
        // the agent and its goal on the far side. The goal sits past the wall's LEFT end (x=0) and well
        // to the left of the start (x=5), so the preferred velocity carries a strong, PERSISTENT lateral
        // (-x) component throughout the approach -- unlike a goal directly below the start (zero lateral
        // component), which gives local reactive ORCA nothing to resolve "which way around" with once
        // the flat wall pins the vertical component (see WallBlocksStraightPath_... for the same
        // point). With that persistent bias, the agent must approach the wall, slide along hugging its
        // boundary (the flat-edge half-plane only constrains the perpendicular component and leaves it
        // free to keep drifting left), round the end vertex, then descend to goal.
        var wallStart = new Vec2(0, 0);
        var wallEnd = new Vec2(15, 0);
        var wall = new[] { wallStart, wallEnd };

        var crowd = new OrcaCrowd();
        crowd.AddObstacle(wall);
        var agent = crowd.Add(new Vec2(5, 3), Radius, MaxSpeed, goal: new Vec2(-3, -3));

        var minClearance = double.PositiveInfinity;
        var minX = double.PositiveInfinity;
        var steps = 0;
        for (; steps < 600 && !crowd.AllArrived(0.3); steps++)
        {
            crowd.Step(Dt);
            var pos = crowd.Position(agent);
            minX = Math.Min(minX, pos.X);
            var clearance = PointSegmentDistance(pos, wallStart, wallEnd);
            minClearance = Math.Min(minClearance, clearance);
            Assert.True(clearance >= Radius - WallEps,
                $"agent penetrated the wall at step {steps}: clearance {clearance:F6} < radius {Radius} (pos={pos})");
        }

        _out.WriteLine($"AgentSlidesAlongWall: settled in {steps} steps, minClearance={minClearance:F4}, " +
                       $"minX={minX:F2} (start x=5, wall left end x=0, goal x=-3)");

        Assert.True(crowd.AllArrived(0.3), $"agent did not reach the goal on the far side (pos={crowd.Position(agent)})");
        // Real, sustained hugging of the wall while sliding toward its near end proves it actually
        // avoided the wall rather than the obstacle being inert (which would let it cut straight
        // through the segment's interior at y=0).
        Assert.True(minClearance < Radius + 0.1,
            $"agent never actually hugged the wall while sliding past it (min clearance {minClearance:F3})");
    }

    [Fact]
    public void NoObstacles_IdenticalToBaseline()
    {
        // Zero obstacles added: obstacles span is empty at every Plan() call, so numObstLines == 0 and
        // the solver's obstacle-line construction loop never executes -- this must reproduce the exact
        // pre-obstacle trajectory (OrcaOpenSpaceTests.SingleAgent_NoNeighbours_GoesStraightToGoal): a
        // lone unobstructed agent goes dead-straight to its goal, never leaving the axis.
        var crowd = new OrcaCrowd();
        var agent = crowd.Add(new Vec2(0, 0), Radius, 1.5, goal: new Vec2(10, 0));

        var steps = 0;
        for (; steps < 200 && !crowd.AllArrived(0.05); steps++)
        {
            crowd.Step(Dt);
            Assert.True(Math.Abs(crowd.Position(agent).Y) < 1e-9,
                $"unobstructed agent drifted off-axis with zero obstacles: y={crowd.Position(agent).Y}");
        }

        _out.WriteLine($"NoObstacles baseline: settled in {steps} steps");
        Assert.True(crowd.AllArrived(0.05), "single agent (no obstacles) did not reach its goal");
    }

    // --- geometry helpers -------------------------------------------------------------------

    // Distance from point `p` to the segment [a, b].
    private static double PointSegmentDistance(Vec2 p, Vec2 a, Vec2 b)
    {
        var ab = b - a;
        var abAbsSq = ab.AbsSq;
        if (abAbsSq <= 1e-15)
        {
            return (p - a).Abs;
        }

        var t = Math.Clamp(Vec2.Dot(p - a, ab) / abAbsSq, 0.0, 1.0);
        var closest = a + t * ab;
        return (p - closest).Abs;
    }

    // Minimum distance from `p` to any edge of the CLOSED polyline `vertices` (mirrors how AddObstacle
    // closes the loop: vertex n-1 connects back to vertex 0).
    private static double MinDistanceToWall(Vec2 p, IReadOnlyList<Vec2> vertices)
    {
        var min = double.PositiveInfinity;
        var n = vertices.Count;
        for (var i = 0; i < n; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % n];
            min = Math.Min(min, PointSegmentDistance(p, a, b));
        }

        return min;
    }

    private static void AssertNoWallPenetration(Vec2 pos, double radius, IReadOnlyList<Vec2> wall, int step)
    {
        var clearance = MinDistanceToWall(pos, wall);
        Assert.True(clearance >= radius - WallEps,
            $"agent penetrated the wall at step {step}: clearance {clearance:F6} < radius {radius} (pos={pos})");
    }
}
