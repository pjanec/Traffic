using System;
using Sim.Core.Orca;
using Sim.Core.Unified;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// Q5 (docs/UNIFIED-SOLVER.md): the unified two-population solver PROTOTYPE. Validates the design's two
// load-bearing claims on the canonical hard case -- the close-range perpendicular crossing the shipped
// bridge documents as its honest residual (a fast vehicle grazing a crossing pedestrian at dt=1):
//
//   1. The single JOINT plan/execute (both regimes read the same frozen start-of-step snapshot) plus
//   2. sub-stepping the vehicle's crowd-REACTION (the parity escape hatch)
//
// together turn that grazing crossing collision-free. This subsystem is standalone (not wired into the
// committed Engine/bridge path), so it cannot affect any golden or the determinism hash -- proven by the
// full suite + hash gate staying green with it present.
public class UnifiedSolverTests
{
    private const double Dt = 1.0;   // the lane sim's fixed step -- the regime the bridge cannot sub-step

    private readonly ITestOutputHelper _out;

    public UnifiedSolverTests(ITestOutputHelper output) => _out = output;

    // The canonical crossing: a 13.9 m/s vehicle down the lane centreline; a pedestrian crossing
    // perpendicular (1.0 m/s, +Y) timed to reach the lane just as the vehicle arrives (xCross=34).
    private static (UnifiedWorld world, OrcaCrowd crowd, int ped, UnifiedWorld.Vehicle veh) BuildCrossing(int subSteps)
    {
        var crowd = new OrcaCrowd(2) { NeighbourDist = 30.0 };
        var ped = crowd.Add(new Vec2(34.0, -2.0), radius: 0.3, maxSpeed: 1.0, goal: new Vec2(34.0, 6.0));
        var world = new UnifiedWorld(crowd) { SubSteps = subSteps };
        var veh = world.AddVehicle(new UnifiedWorld.Vehicle
        {
            X = 0, LatOffset = 0, Speed = 13.9, DesiredSpeed = 13.9,
            Length = 5.0, HalfWidth = 0.9, MaxSpeedLat = 1.0, LaneHalfWidth = 3.6,
        });
        return (world, crowd, ped, veh);
    }

    private double RunMinClearance(int subSteps, out double vehEndX)
    {
        var (world, crowd, ped, veh) = BuildCrossing(subSteps);
        var minClear = double.PositiveInfinity;
        for (var step = 0; step < 12; step++)
        {
            world.Step(Dt);
            var clear = UnifiedWorld.VehicleFootprintDistance(veh, crowd.Position(ped)) - crowd.Radius(ped);
            minClear = Math.Min(minClear, clear);
        }

        vehEndX = veh.X;
        return minClear;
    }

    // The headline result: a SINGLE per-step re-plan (SubSteps=1, the bridge's resolution) grazes the
    // crossing pedestrian, while sub-stepping the reaction (SubSteps=8) keeps them collision-free AND the
    // vehicle still drives on through. This is the guarantee the shipped bridge lacks.
    [Fact]
    public void SubSteppedReaction_TurnsGrazingCrossingCollisionFree()
    {
        var single = RunMinClearance(subSteps: 1, out var singleEndX);
        var subbed = RunMinClearance(subSteps: 8, out var subbedEndX);

        _out.WriteLine($"crossing min clearance: SubSteps=1 -> {single:F3} m (overlap), SubSteps=8 -> {subbed:F3} m (safe)");
        _out.WriteLine($"vehicle end X: SubSteps=1 -> {singleEndX:F1}, SubSteps=8 -> {subbedEndX:F1}");

        // Single re-plan per step grazes (negative clearance == footprint overlaps the pedestrian).
        Assert.True(single < -0.05, $"expected the single-re-plan case to graze, but min clearance was {single:F3}");

        // Sub-stepped reaction stays collision-free.
        Assert.True(subbed >= -1e-6, $"sub-stepped reaction still grazed: min clearance {subbed:F3}");

        // And the vehicle is not simply frozen -- it drives on through the crossing.
        Assert.True(subbedEndX > 50.0, $"vehicle did not proceed through the crossing (ended at X={subbedEndX:F1})");
    }

    // The joint solve is mutual: the pedestrian is also nudged by the passing vehicle (Direction A live in
    // the same step as Direction B), and never ends up inside the vehicle footprint.
    [Fact]
    public void JointSolve_PedestrianAlsoDeflects_AndNeverInsideFootprint()
    {
        var (world, crowd, ped, veh) = BuildCrossing(subSteps: 8);
        var startX = crowd.Position(ped).X;
        var deflected = false;

        for (var step = 0; step < 12; step++)
        {
            world.Step(Dt);
            if (Math.Abs(crowd.Position(ped).X - startX) > 0.05)
            {
                deflected = true;
            }

            Assert.True(UnifiedWorld.VehicleFootprintDistance(veh, crowd.Position(ped)) >= crowd.Radius(ped) - 1e-6,
                $"pedestrian entered the vehicle footprint at step {step}");
        }

        Assert.True(deflected, "pedestrian was not deflected at all by the passing vehicle (expected mutual avoidance)");
    }

    // Determinism: identical setups produce bit-identical trajectories (no RNG / wall-clock / order
    // dependence) -- the plan/execute discipline the whole project requires.
    [Fact]
    public void UnifiedWorld_IsDeterministic_RunToRun()
    {
        var (wa, ca, pa, va) = BuildCrossing(subSteps: 8);
        var (wb, cb, pb, vb) = BuildCrossing(subSteps: 8);

        for (var step = 0; step < 12; step++)
        {
            wa.Step(Dt);
            wb.Step(Dt);
            Assert.Equal(va.X, vb.X);
            Assert.Equal(va.LatOffset, vb.LatOffset);
            Assert.Equal(va.Speed, vb.Speed);
            Assert.Equal(ca.Position(pa).X, cb.Position(pb).X);
            Assert.Equal(ca.Position(pa).Y, cb.Position(pb).Y);
        }
    }
}
