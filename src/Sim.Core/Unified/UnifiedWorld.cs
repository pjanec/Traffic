using System;
using System.Collections.Generic;
using Sim.Core.Bridge;
using Sim.Core.Orca;

namespace Sim.Core.Unified;

// PROTOTYPE (docs/UNIFIED-SOLVER.md) — the unified two-population solver, built as a self-contained
// subsystem exactly as the ORCA and Bridge layers were, and deliberately NOT wired into the committed
// lane/bridge path (nothing here is reachable from a golden, so the determinism hash is structurally
// unaffected). It exists to prove the design's two load-bearing claims on the canonical hard case
// before the owner authorises the real Engine integration:
//
//   1. SINGLE JOINT PLAN/EXECUTE (no asymmetric latency). Every mover -- holonomic crowd agents and
//      lane-constrained vehicles -- plans from the SAME frozen start-of-step snapshot, then all commit.
//      (The shipped bridge instead runs Engine.Run(1) THEN crowd.Step(), a half-step skew.)
//   2. THE PARITY ESCAPE HATCH: sub-stepping the vehicle's REACTION to the crowd. Parity pins only the
//      no-crowd trajectory; a crowd-coupled run has no golden, so the vehicle may re-evaluate its
//      lateral swerve + longitudinal brake K times per lane step against the dead-reckoned crowd,
//      tracking a crosser continuously instead of once. This is what turns the close-range perpendicular
//      crossing (which a single per-step re-plan grazes) collision-free -- the guarantee the bridge lacks.
//
// SIMPLIFICATIONS (honest, because this is a prototype of the COUPLING, not of car-following): the lane
// is a straight segment along +X with centreline y == 0; the vehicle's longitudinal model is a simple
// safe-speed follower (approach desiredSpeed, brake to stop within the gap to a blocking crowd agent),
// NOT the full validated Krauss reduction the real Engine already owns. The real integration reuses the
// Engine's exact longitudinal reduction; here the point is the coupling structure + the sub-stepped
// reaction, so a faithful-enough vehicle suffices. The crowd side is the REAL OrcaCrowd (so this inherits
// its Q1 static obstacles and Q3 spatial hash for free), avoided one-sided by the vehicle discs.
public sealed class UnifiedWorld
{
    // A lane-constrained vehicle: front-centre at (X, LatOffset), moving +X at Speed, footprint
    // Length x (2*HalfWidth). Restricted DOF: longitudinal (Speed, braking) + lateral (LatOffset,
    // bounded by MaxSpeedLat) within the lane's lateral bounds [-LaneHalfWidth, +LaneHalfWidth].
    public sealed class Vehicle
    {
        public double X;
        public double LatOffset;
        public double Speed;
        public double Length;
        public double HalfWidth;
        public double DesiredSpeed;
        public double MaxSpeedLat = 1.0;
        public double MaxAccel = 2.6;
        public double MaxDecel = 4.5;
        public double LaneHalfWidth = 3.6;
        public double MinLatGap = 0.6;   // SUMO minGapLat analogue
    }

    private readonly OrcaCrowd _crowd;
    private readonly List<Vehicle> _vehicles = new();

    // Frozen start-of-step crowd state (so the vehicle plans against the SAME instant the crowd does).
    private Vec2[] _frozenPos = Array.Empty<Vec2>();
    private Vec2[] _frozenVel = Array.Empty<Vec2>();
    private double[] _frozenRadius = Array.Empty<double>();
    private int _frozenCount;

    // Vehicle discs handed to the crowd each step (Direction A: crowd avoids vehicles).
    private WorldDisc[] _discBuf = new WorldDisc[16];

    // Sub-steps for the vehicle's crowd-reaction (the escape hatch). 1 == a single per-step re-plan
    // (models the bridge's resolution, which grazes a close crossing); >1 sub-steps the reaction.
    public int SubSteps { get; set; } = 8;

    // Longitudinal reaction range: a crowd agent farther ahead than this does not affect speed.
    public double ReactionRange { get; set; } = 40.0;

    public UnifiedWorld(OrcaCrowd crowd) => _crowd = crowd;

    public OrcaCrowd Crowd => _crowd;
    public IReadOnlyList<Vehicle> Vehicles => _vehicles;

    public Vehicle AddVehicle(Vehicle v)
    {
        _vehicles.Add(v);
        return v;
    }

    // One joint lockstep. BOTH regimes read the frozen start-of-step snapshot of the other, so there is
    // no asymmetric one-step latency. The crowd is the real OrcaCrowd (avoids the frozen vehicle discs);
    // each vehicle avoids the frozen crowd with its reaction sub-stepped SubSteps times.
    public void Step(double dt)
    {
        FreezeCrowd();
        var discCount = BuildVehicleDiscs();

        // Crowd plans/moves against the frozen vehicle discs (Direction A).
        _crowd.SetExternalObstacles(_discBuf.AsSpan(0, discCount));
        _crowd.Step(dt);

        // Vehicles plan/move against the FROZEN crowd (Direction B), reaction sub-stepped.
        foreach (var v in _vehicles)
        {
            StepVehicle(v, dt);
        }
    }

    private void FreezeCrowd()
    {
        _frozenCount = _crowd.Count;
        if (_frozenPos.Length < _frozenCount)
        {
            _frozenPos = new Vec2[_frozenCount];
            _frozenVel = new Vec2[_frozenCount];
            _frozenRadius = new double[_frozenCount];
        }

        for (var i = 0; i < _frozenCount; i++)
        {
            _frozenPos[i] = _crowd.Position(i);
            _frozenVel[i] = _crowd.Velocity(i);
            _frozenRadius[i] = _crowd.Radius(i);
        }
    }

    private int BuildVehicleDiscs()
    {
        // A vehicle covers its footprint spine [X-Length, X] with a short chain of discs (like the
        // bridge's BuildVehicleDiscs), moving at (Speed, 0) along the lane.
        var needed = 0;
        foreach (var v in _vehicles)
        {
            needed += Math.Clamp((int)Math.Ceiling(v.Length / v.HalfWidth), 1, 6);
        }

        if (_discBuf.Length < needed)
        {
            _discBuf = new WorldDisc[Math.Max(needed, _discBuf.Length * 2)];
        }

        var n = 0;
        foreach (var v in _vehicles)
        {
            var count = Math.Clamp((int)Math.Ceiling(v.Length / v.HalfWidth), 1, 6);
            var spacing = count > 1 ? v.Length / (count - 1) : 0.0;
            for (var d = 0; d < count; d++)
            {
                var back = d * spacing;
                _discBuf[n++] = new WorldDisc(v.X - back, v.LatOffset, v.Speed, 0.0, v.HalfWidth);
            }
        }

        return n;
    }

    // Advance one vehicle over dt, re-evaluating its lateral swerve + longitudinal brake SubSteps times
    // against the dead-reckoned frozen crowd. Net longitudinal displacement stays Speed-consistent; what
    // the sub-stepping buys is that the reaction TRACKS a crossing agent continuously.
    private void StepVehicle(Vehicle v, double dt)
    {
        var k = Math.Max(1, SubSteps);
        var subDt = dt / k;

        for (var s = 0; s < k; s++)
        {
            var elapsed = s * subDt;

            // Longitudinal: safe speed = min(accelerate toward desired, brake to stop within the gap to
            // the nearest crowd agent that laterally overlaps ego's swath and is ahead).
            var vFree = Math.Min(v.DesiredSpeed, v.Speed + v.MaxAccel * subDt);
            var vSafe = vFree;

            // Lateral: feasible interval avoiding every near crowd agent's forbidden band.
            var latTarget = 0.0;                 // prefer centre
            var haveForbidden = false;
            double forbLo = 0, forbHi = 0;       // single-band prototype (one dominant crosser)

            for (var i = 0; i < _frozenCount; i++)
            {
                // Dead-reckon the crowd agent to this sub-step.
                var px = _frozenPos[i].X + _frozenVel[i].X * elapsed;
                var py = _frozenPos[i].Y + _frozenVel[i].Y * elapsed;
                var pr = _frozenRadius[i];

                var gapAhead = px - v.X;                 // >0: agent ahead of ego's front
                var latSep = py - v.LatOffset;
                var combinedHalf = v.HalfWidth + pr + v.MinLatGap;

                // Longitudinal: does the agent (at its predicted lateral) overlap ego's lateral swath?
                if (Math.Abs(py) < v.LaneHalfWidth && gapAhead > -v.Length && gapAhead < ReactionRange)
                {
                    var laterallyOverlaps = Math.Abs(latSep) < v.HalfWidth + pr;
                    if (laterallyOverlaps && gapAhead > 0.0)
                    {
                        // Brake to stop within the gap (simple safe-speed; NOT the parity Krauss model).
                        var stopGap = Math.Max(0.0, gapAhead - pr);
                        var vBrake = Math.Sqrt(2.0 * v.MaxDecel * stopGap);
                        vSafe = Math.Min(vSafe, vBrake);
                    }

                    // Lateral: forbid the agent's band if it (predicted) sits within the lane near ego.
                    if (gapAhead < ReactionRange && py > -v.LaneHalfWidth && py < v.LaneHalfWidth)
                    {
                        forbLo = py - combinedHalf;
                        forbHi = py + combinedHalf;
                        haveForbidden = true;
                    }
                }
            }

            if (haveForbidden)
            {
                latTarget = FeasibleClosestToCentre(forbLo, forbHi, v.LaneHalfWidth - v.HalfWidth);
            }

            // Also allow braking below the floor (never negative).
            vSafe = Math.Max(0.0, Math.Min(vSafe, v.Speed + v.MaxAccel * subDt));
            // Bound deceleration.
            vSafe = Math.Max(vSafe, v.Speed - v.MaxDecel * subDt);
            v.Speed = vSafe;

            // Lateral move bounded by MaxSpeedLat, clamped to the lane.
            var latStep = Math.Clamp(latTarget - v.LatOffset, -v.MaxSpeedLat * subDt, v.MaxSpeedLat * subDt);
            v.LatOffset = Math.Clamp(v.LatOffset + latStep, -(v.LaneHalfWidth - v.HalfWidth), v.LaneHalfWidth - v.HalfWidth);

            // Advance longitudinally.
            v.X += v.Speed * subDt;
        }
    }

    // The lateral offset closest to the centre (0) that is OUTSIDE the forbidden band [lo, hi], within
    // the lane's usable half-width. If the centre itself is feasible, keep it; else pick the nearer edge
    // that still fits inside the lane; if neither side fits, hold at the band edge nearer centre (the
    // longitudinal brake then does the work -- the "can't swerve, so stop" fallback).
    private static double FeasibleClosestToCentre(double lo, double hi, double usableHalf)
    {
        if (0.0 <= lo || 0.0 >= hi)
        {
            return 0.0;   // centre is already outside the band
        }

        var leftFeasible = lo >= -usableHalf ? lo : double.NaN;    // shift toward -Y to just clear the band
        var rightFeasible = hi <= usableHalf ? hi : double.NaN;

        var canLeft = !double.IsNaN(leftFeasible);
        var canRight = !double.IsNaN(rightFeasible);

        if (canLeft && canRight)
        {
            return Math.Abs(leftFeasible) <= Math.Abs(rightFeasible) ? leftFeasible : rightFeasible;
        }

        if (canLeft)
        {
            return leftFeasible;
        }

        if (canRight)
        {
            return rightFeasible;
        }

        // Fully blocked: aim for the edge nearer centre; the longitudinal brake stops ego.
        return Math.Abs(lo) <= Math.Abs(hi) ? Math.Max(lo, -usableHalf) : Math.Min(hi, usableHalf);
    }

    // Minimum distance from a crowd agent's centre to a vehicle's axis-aligned footprint rectangle
    // [X-Length, X] x [LatOffset-HalfWidth, LatOffset+HalfWidth] -- for the no-overlap assertion in tests.
    public static double VehicleFootprintDistance(Vehicle v, Vec2 p)
    {
        var dx = Math.Max(Math.Max((v.X - v.Length) - p.X, p.X - v.X), 0.0);
        var dy = Math.Max(Math.Max((v.LatOffset - v.HalfWidth) - p.Y, p.Y - (v.LatOffset + v.HalfWidth)), 0.0);
        return Math.Sqrt(dx * dx + dy * dy);
    }
}
