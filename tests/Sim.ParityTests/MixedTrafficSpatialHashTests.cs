using System;
using Sim.Core.Mixed;
using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2a / TASKS T2.1: MixedTrafficCrowd's uniform spatial hash,
// mirroring OrcaSpatialHashTests exactly. The hard requirement is BIT-IDENTITY: grid-on must produce
// exactly the same trajectory (position AND heading -- MixedTrafficCrowd is shaped/oriented, unlike
// the disc OrcaCrowd) as grid-off. The grid is only a pre-filter that skips out-of-range vehicles;
// candidates are sorted to the same order the brute-force scan visits, so the neighbour set, order,
// every LP and every position/heading match to the last bit. Default is off, so every existing demo
// is byte-identical; callers opt in for the perf win.
public class MixedTrafficSpatialHashTests
{
    private const double Dt = 0.25;

    private readonly ITestOutputHelper _out;

    public MixedTrafficSpatialHashTests(ITestOutputHelper output) => _out = output;

    // Run `build` twice -- grid off, then grid on -- and assert the two crowds stay bit-identical
    // (exact double equality on every mover's position AND heading) at every step for `steps` steps.
    private void AssertGridMatchesBruteForce(Func<MixedTrafficCrowd> build, int steps, string label)
    {
        var brute = build();
        var grid = build();
        grid.UseSpatialHash = true;

        var n = brute.Count;
        Assert.True(n > 0 && n == grid.Count);

        for (var step = 0; step < steps; step++)
        {
            brute.Step(Dt);
            grid.Step(Dt);
            for (var i = 0; i < n; i++)
            {
                var pb = brute.Position(i);
                var pg = grid.Position(i);
                Assert.True(pb.X == pg.X && pb.Y == pg.Y,
                    $"{label}: grid diverged from brute-force at step {step}, vehicle {i} (position): " +
                    $"brute=({pb.X:R},{pb.Y:R}) grid=({pg.X:R},{pg.Y:R})");

                var hb = brute.Heading(i);
                var hg = grid.Heading(i);
                Assert.True(hb == hg,
                    $"{label}: grid diverged from brute-force at step {step}, vehicle {i} (heading): " +
                    $"brute={hb:R} grid={hg:R}");
            }
        }

        _out.WriteLine($"{label}: grid bit-identical to brute-force over {steps} steps, {n} vehicles");
    }

    // (a) A dense many-cell crowd: vehicles span many grid cells (cell edge == NeighbourDist == 22.0),
    // so the 3x3 block genuinely prunes, and paths cross in the middle.
    [Fact]
    public void SpreadCrowd_ManyCells_GridBitIdentical()
    {
        AssertGridMatchesBruteForce(() =>
        {
            var crowd = new MixedTrafficCrowd(64) { SymmetryBreak = 0.05 };
            for (var gx = 0; gx < 6; gx++)
            {
                for (var gy = 0; gy < 6; gy++)
                {
                    var p = new Vec2(gx * 12.0 - 33.0, gy * 12.0 - 33.0);
                    crowd.Add(VehicleClass.Car, p, goal: -p);
                }
            }

            return crowd;
        }, steps: 80, "spread-crowd-many-cells");
    }

    // (b) MaxNeighbours cap actually binding: a tight cluster where each vehicle has more than
    // MaxNeighbours in-range neighbours -- the ordering-sensitive nearest-k bounded-insertion path.
    [Fact]
    public void TightCluster_MaxNeighboursBinding_GridBitIdentical()
    {
        AssertGridMatchesBruteForce(() =>
        {
            var crowd = new MixedTrafficCrowd(40) { SymmetryBreak = 0.05, MaxNeighbours = 4 };
            var n = 20;
            for (var i = 0; i < n; i++)
            {
                var angle = 2.0 * Math.PI * i / n;
                var r = 8.0;
                var p = new Vec2(r * Math.Cos(angle), r * Math.Sin(angle));
                // Send each vehicle to the opposite side of the circle: every vehicle sees every other
                // vehicle within NeighbourDist (22.0), so with n=20 and MaxNeighbours=4 the cap binds
                // for every single vehicle at every step.
                crowd.Add(VehicleClass.Car, p, goal: -p);
            }

            return crowd;
        }, steps: 60, "tight-cluster-maxN");
    }

    // (c) The non-holonomic steering path (kinematic-bicycle integration, movers turning) -- the other
    // ordering-sensitive combination named by the design.
    [Fact]
    public void Nonholonomic_Turning_GridBitIdentical()
    {
        AssertGridMatchesBruteForce(() =>
        {
            var crowd = new MixedTrafficCrowd(48)
            {
                SymmetryBreak = 0.05,
                MaxNeighbours = 6,
                Nonholonomic = true,
                CreepSpeed = 0.5,
                SafetyMargin = 0.3,
            };

            // A grid of vehicles each heading to a goal well off their initial heading, so the
            // non-holonomic steering path genuinely turns them (not just straight-line cruise).
            for (var gx = 0; gx < 5; gx++)
            {
                for (var gy = 0; gy < 5; gy++)
                {
                    var p = new Vec2(gx * 10.0 - 20.0, gy * 10.0 - 20.0);
                    var goal = new Vec2(-p.Y, p.X);   // 90-degree-rotated goal: forces a turn
                    crowd.Add(VehicleClass.Car, p, goal, headingRad: 0.0);
                }
            }

            return crowd;
        }, steps: 100, "nonholonomic-turning");
    }

    // Run-to-run reproducibility of the grid path itself (grid == grid twice), as in
    // OrcaSpatialHashTests.SpatialHash_IsDeterministic_RunToRun.
    [Fact]
    public void SpatialHash_IsDeterministic_RunToRun()
    {
        MixedTrafficCrowd Build()
        {
            var crowd = new MixedTrafficCrowd(64)
            {
                SymmetryBreak = 0.05,
                UseSpatialHash = true,
                MaxNeighbours = 6,
                Nonholonomic = true,
                CreepSpeed = 0.5,
            };
            for (var gx = 0; gx < 6; gx++)
            {
                for (var gy = 0; gy < 6; gy++)
                {
                    var p = new Vec2(gx * 12.0 - 33.0, gy * 12.0 - 33.0);
                    crowd.Add(VehicleClass.Car, p, goal: -p);
                }
            }

            return crowd;
        }

        var a = Build();
        var b = Build();
        for (var step = 0; step < 100; step++)
        {
            a.Step(Dt);
            b.Step(Dt);
            for (var i = 0; i < a.Count; i++)
            {
                Assert.Equal(a.Position(i).X, b.Position(i).X);
                Assert.Equal(a.Position(i).Y, b.Position(i).Y);
                Assert.Equal(a.Heading(i), b.Heading(i));
            }
        }
    }
}
