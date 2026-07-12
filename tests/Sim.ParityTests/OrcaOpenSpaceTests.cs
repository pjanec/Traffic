using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// OPEN-SPACE full 2D reciprocal ORCA layer (docs/LANELESS-DIRECTION.md, the second of the two
// regimes -- holonomic disc crowd/navmesh agents, NOT lane-derived vehicles). ORCA has no SUMO
// counterpart, so it is validated BEHAVIOURALLY, exactly the bar the laneless direction set:
//   (1) no-overlap: disc agents never interpenetrate (centre distance >= sum of radii, within eps) --
//       ORCA's hard guarantee, which must hold EVEN WHEN the crowd deadlocks;
//   (2) reaches-goal: agents converge in open space and realistic dense flows (counter-flow, a
//       many-agent crossing permutation);
//   (3) reciprocal-symmetry: a mirror-symmetric setup produces mirror-symmetric trajectories;
//   (4) deterministic: identical runs are bit-identical (no RNG / wall-clock / order dependence).
//
// A note on the famous "antipodal circle": at maximal packing (every path crossing one point) pure
// reciprocal ORCA can DEADLOCK -- agents jam in a stable touching ring rather than pass through. NB
// (measured in Q2, OrcaConvergenceTests): adding nearest-maxNeighbours culling AND removal-on-arrival
// does NOT resolve this here, at any symmetry-break magnitude up to 0.5 -- the perfect symmetry
// re-forms the jam. Those aids' proven value is elsewhere (faster draining of dense NON-symmetric
// flows + a per-agent work bound). The antipodal deadlock is a convergence limitation, never a
// SAFETY one: the no-overlap guarantee still holds. So this suite asserts CONVERGENCE on the flows
// ORCA robustly solves (counter-flow, permutation scatter, crossings) and uses the antipodal circle
// only to prove the collision guarantee survives the deadlock.
//
// This subsystem does not touch the lane-parity core or the string ExternalObstacle API, so the
// determinism hash and every golden are unaffected by construction (nothing here is reachable from
// the lane engine).
public class OrcaOpenSpaceTests
{
    private const double Dt = 0.25;
    private const double Radius = 0.5;
    private const double MaxSpeed = 1.5;
    // Overlap tolerance: ORCA guarantees avoidance over a horizon but a discrete step can shave a
    // hair off the combined radius in tight packing; a few mm is well within "did not collide".
    private const double OverlapEps = 0.02;

    private readonly ITestOutputHelper _out;

    public OrcaOpenSpaceTests(ITestOutputHelper output) => _out = output;

    [Fact]
    public void SingleAgent_NoNeighbours_GoesStraightToGoal()
    {
        var crowd = new OrcaCrowd();
        crowd.Add(new Vec2(0, 0), Radius, MaxSpeed, goal: new Vec2(10, 0));

        for (var step = 0; step < 200 && !crowd.AllArrived(0.05); step++)
        {
            crowd.Step(Dt);
            // Unobstructed: it should never leave the x-axis (pure straight-line pursuit).
            Assert.True(Math.Abs(crowd.Position(0).Y) < 1e-9, $"unobstructed agent drifted off-axis: y={crowd.Position(0).Y}");
        }

        Assert.True(crowd.AllArrived(0.05), "single agent did not reach its goal");
    }

    [Fact]
    public void HeadOn_TwoAgents_PassWithoutCollision_BothReachGoals()
    {
        var crowd = new OrcaCrowd();
        var a = crowd.Add(new Vec2(-8, 0), Radius, MaxSpeed, goal: new Vec2(8, 0));
        var b = crowd.Add(new Vec2(8, 0), Radius, MaxSpeed, goal: new Vec2(-8, 0));

        var minSep = double.PositiveInfinity;
        for (var step = 0; step < 400 && !crowd.AllArrived(0.1); step++)
        {
            crowd.Step(Dt);
            minSep = Math.Min(minSep, (crowd.Position(a) - crowd.Position(b)).Abs);
            AssertNoOverlap(crowd, step);
        }

        _out.WriteLine($"head-on min separation = {minSep:F4} (combined radius {2 * Radius})");
        Assert.True(crowd.AllArrived(0.1), "head-on agents did not both reach their goals");
        Assert.True(minSep >= 2 * Radius - OverlapEps, $"head-on agents overlapped (min sep {minSep:F4})");
    }

    [Fact]
    public void PerpendicularCrossing_NoCollision_BothReachGoals()
    {
        // Two agents crossing at the origin are symmetric about y=x -- another degenerate case pure
        // ORCA stalls on (they stop at the mutual point); the deterministic symmetry break resolves
        // it (converges in ~45 steps), as it does any real, slightly-irregular crossing.
        var crowd = new OrcaCrowd() { SymmetryBreak = 0.05 };
        crowd.Add(new Vec2(-8, 0), Radius, MaxSpeed, goal: new Vec2(8, 0));
        crowd.Add(new Vec2(0, -8), Radius, MaxSpeed, goal: new Vec2(0, 8));

        for (var step = 0; step < 400 && !crowd.AllArrived(0.1); step++)
        {
            crowd.Step(Dt);
            AssertNoOverlap(crowd, step);
        }

        Assert.True(crowd.AllArrived(0.1), "crossing agents did not both reach their goals");
    }

    // Bidirectional pedestrian flow -- the canonical realistic crowd scenario: two streams walking
    // head-on through each other, offset so they must interleave. Every agent must reach the far
    // side and no two may ever overlap. ORCA solves this robustly (self-organising lanes emerge).
    [Theory]
    [InlineData(3)]
    [InlineData(4)]
    public void CounterFlow_TwoStreams_AllReachGoals_NeverOverlap(int perStream)
    {
        var crowd = new OrcaCrowd(2 * perStream) { SymmetryBreak = 0.05 };
        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(-12, i * 1.5), Radius, MaxSpeed, goal: new Vec2(12, i * 1.5));          // →
        }

        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(12, i * 1.5 + 0.75), Radius, MaxSpeed, goal: new Vec2(-12, i * 1.5 + 0.75)); // ←
        }

        var steps = 0;
        for (; steps < 800 && !crowd.AllArrived(0.2); steps++)
        {
            crowd.Step(Dt);
            AssertNoOverlap(crowd, steps);
        }

        _out.WriteLine($"counter-flow({perStream}+{perStream}): settled in {steps} steps");
        Assert.True(crowd.AllArrived(0.2), $"counter-flow({perStream}) did not all reach goals");
    }

    // Many agents on a grid, each targeting the grid point of another (a reversing permutation, so
    // paths cross densely in the middle). A larger no-overlap + all-reach-goal stress that ORCA
    // handles well because the configuration is not perfectly symmetric.
    [Fact]
    public void PermutationScatter_ManyAgents_AllReachGoals_NeverOverlap()
    {
        const int n = 16;
        const int cols = 4;
        var crowd = new OrcaCrowd(n) { SymmetryBreak = 0.05 };
        var pts = new Vec2[n];
        for (var i = 0; i < n; i++)
        {
            pts[i] = new Vec2(i % cols * 3.0, i / cols * 3.0);
        }

        for (var i = 0; i < n; i++)
        {
            crowd.Add(pts[i], Radius, MaxSpeed, goal: pts[n - 1 - i]);   // reversed -> dense central crossing
        }

        var steps = 0;
        for (; steps < 1200 && !crowd.AllArrived(0.2); steps++)
        {
            crowd.Step(Dt);
            AssertNoOverlap(crowd, steps);
        }

        _out.WriteLine($"permutation scatter({n}): settled in {steps} steps");
        Assert.True(crowd.AllArrived(0.2), $"permutation scatter({n}) did not all reach goals");
    }

    // The antipodal circle used HONESTLY: at maximal packing pure ORCA may deadlock (agents jam in a
    // touching ring), so this asserts only the SAFETY guarantee -- across the whole run, no two agents
    // ever interpenetrate, even as they crush together at the centre. Convergence is deliberately NOT
    // asserted here (it is covered by the flows above); this pins the collision guarantee under the
    // worst symmetric load. Run pure (SymmetryBreak = 0) so the jam is maximal.
    [Theory]
    [InlineData(8)]
    [InlineData(12)]
    public void AntipodalCircle_NeverOverlap_EvenUnderDeadlock(int n)
    {
        const double circleR = 8.0;
        var crowd = new OrcaCrowd(n);   // no symmetry break: provoke the worst-case jam on purpose
        for (var i = 0; i < n; i++)
        {
            var theta = 2.0 * Math.PI * i / n;
            var pos = new Vec2(circleR * Math.Cos(theta), circleR * Math.Sin(theta));
            crowd.Add(pos, Radius, MaxSpeed, goal: -pos);
        }

        var minSep = double.PositiveInfinity;
        for (var step = 0; step < 400; step++)
        {
            crowd.Step(Dt);
            for (var i = 0; i < n; i++)
            {
                for (var j = i + 1; j < n; j++)
                {
                    var sep = (crowd.Position(i) - crowd.Position(j)).Abs;
                    minSep = Math.Min(minSep, sep);
                    Assert.True(sep >= 2 * Radius - OverlapEps,
                        $"antipodal({n}) COLLISION at step {step}: agents {i},{j} sep {sep:F4} < {2 * Radius}");
                }
            }
        }

        _out.WriteLine($"antipodal({n}) safety: min separation over run = {minSep:F4} (combined radius {2 * Radius})");
    }

    [Fact]
    public void MirrorSymmetricSetup_ProducesMirrorSymmetricTrajectories()
    {
        // Two agents symmetric about the x-axis, both heading +x. By the determinism + reciprocity of
        // the solve their y-trajectories must stay exact mirror images (yA == -yB every step). Run
        // pure (no jitter, which would deliberately desymmetrise).
        var crowd = new OrcaCrowd();
        var a = crowd.Add(new Vec2(-8, 1.5), Radius, MaxSpeed, goal: new Vec2(8, 1.5));
        var b = crowd.Add(new Vec2(-8, -1.5), Radius, MaxSpeed, goal: new Vec2(8, -1.5));

        for (var step = 0; step < 200 && !crowd.AllArrived(0.1); step++)
        {
            crowd.Step(Dt);
            var pa = crowd.Position(a);
            var pb = crowd.Position(b);
            Assert.True(Math.Abs(pa.X - pb.X) < 1e-12, $"symmetry broke in x at step {step}: {pa.X} vs {pb.X}");
            Assert.True(Math.Abs(pa.Y + pb.Y) < 1e-12, $"symmetry broke in y at step {step}: {pa.Y} vs {pb.Y}");
        }
    }

    [Fact]
    public void Deterministic_IdenticalRuns_AreBitIdentical()
    {
        var runA = RunCounterFlow();
        var runB = RunCounterFlow();

        Assert.Equal(runA.Count, runB.Count);
        for (var i = 0; i < runA.Count; i++)
        {
            Assert.Equal(runA[i].X, runB[i].X);   // exact bit equality -- no RNG, no wall-clock, fixed order
            Assert.Equal(runA[i].Y, runB[i].Y);
        }
    }

    private static List<Vec2> RunCounterFlow()
    {
        const int perStream = 3;
        var crowd = new OrcaCrowd(2 * perStream) { SymmetryBreak = 0.05 };
        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(-12, i * 1.5), Radius, MaxSpeed, goal: new Vec2(12, i * 1.5));
        }

        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(12, i * 1.5 + 0.75), Radius, MaxSpeed, goal: new Vec2(-12, i * 1.5 + 0.75));
        }

        var trace = new List<Vec2>();
        for (var step = 0; step < 120; step++)
        {
            crowd.Step(Dt);
            for (var i = 0; i < crowd.Count; i++)
            {
                trace.Add(crowd.Position(i));
            }
        }

        return trace;
    }

    private static void AssertNoOverlap(OrcaCrowd crowd, int step)
    {
        for (var i = 0; i < crowd.Count; i++)
        {
            for (var j = i + 1; j < crowd.Count; j++)
            {
                var sep = (crowd.Position(i) - crowd.Position(j)).Abs;
                Assert.True(sep >= crowd.Radius(i) + crowd.Radius(j) - OverlapEps,
                    $"overlap at step {step}: agents {i},{j} separation {sep:F4}");
            }
        }
    }
}
