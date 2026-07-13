using Sim.Core.Bridge;
using Sim.Core.Mixed;
using Sim.Core.Orca;

namespace Sim.Evac;

// PANIC-EVAC-PHASE3-DESIGN.md §1, §4 (Option A, T1.2): a Sim.Evac-owned manager wrapping ONE
// Sim.Core.Mixed.MixedTrafficCrowd -- the shaped, non-holonomic free-space driver for cars that have
// mounted the shoulder (the "Orca-push" stage). This class owns nothing lane-related; it is a thin,
// standalone, deterministic wrapper: index bookkeeping (per-mover wedge dwell + active flag) around
// the core crowd. Parity-exempt (Sim.Evac never touches Sim.Core's parity-critical Engine seams via
// this class), reachable only from the evac layer / viz, so it cannot move the determinism hash.
//
// Also implements ICrowdFootprintSource (T3.1 §3) so pushing cars are a first-class obstacle to the
// LANE engine too, via a CompositeFootprintSource alongside the pedestrian crowd -- mirroring
// OrcaCrowd.QueryNear exactly (brute-force, deterministic index order, ACTIVE movers only).
public sealed class VehicleMover : ICrowdFootprintSource
{
    private readonly EvacConfig _cfg;
    private readonly MixedTrafficCrowd _crowd;

    // Footprint radius (m) this mover presents to OTHER regimes (pedestrians via FeedVehicleDiscsToPeds,
    // lane vehicles via QueryNear) -- half of a ~5 m car, independent of the shaped-VO solve footprint.
    private const double CarFootprintRadius = 2.5;

    // Per-mover wedge tracking, indexed by the SAME index the underlying crowd assigns (AddCar always
    // appends, so this list and the crowd's index space stay in lockstep). Wedge is judged on PROGRESS
    // toward the goal (best distance-to-goal ever seen this "episode"), not on instantaneous speed: a
    // mover deflected by a wall/corner can keep a non-trivial instantaneous speed indefinitely while
    // going in circles alongside the obstacle (net progress ~0) -- a raw |velocity| < threshold check
    // never latches for that case. Distance-to-goal strictly improving (beyond ProgressEpsilon) resets
    // the dwell clock and records the new best; failing to improve for the dwell accrues it.
    private readonly List<double> _dwell = new();
    private readonly List<double> _bestDistToGoal = new();
    private readonly List<bool> _active = new();

    // Minimum improvement (m) in distance-to-goal that counts as "real progress" and resets the wedge
    // dwell clock. Small relative to a car's own length, comfortably above position noise from the
    // shaped-VO solve, well below any genuine forward step at OrcaPushMaxSpeed.
    private const double ProgressEpsilon = 0.25;

    // Defensive containment bounds, inferred from the wall segments ArmWalls is given (the band's own
    // bounding rectangle). The shaped-VO wall is the PRIMARY confinement (a zero-velocity, full-yield
    // neighbour every mover's Plan() sees every step); this is a last-resort backstop for the rare
    // degenerate case where non-holonomic steering's overlap-recovery step (ShapedVoSolver's "already
    // overlapping" branch, sized for a SHORT recovery horizon) produces a burst that carries a mover
    // through a thin wall in one step before the VO can react again next step. Unset (+-infinity) until
    // ArmWalls is called, so a VehicleMover with no walls armed is never clamped.
    private double _boundsMinX = double.NegativeInfinity;
    private double _boundsMinY = double.NegativeInfinity;
    private double _boundsMaxX = double.PositiveInfinity;
    private double _boundsMaxY = double.PositiveInfinity;

    public VehicleMover(EvacConfig cfg)
    {
        _cfg = cfg;
        // Nonholonomic=true: PANIC-EVAC-PHASE3-DESIGN.md is explicit that Orca-push is a SHAPED,
        // NON-HOLONOMIC free-space push (kinematic-bicycle steering, no sideways teleport, no
        // pivot-in-place) -- the whole point of driving a car with MixedTrafficCrowd instead of the
        // holonomic pedestrian OrcaCrowd. CreepSpeed (PRELIM fix): without a creep floor, a pusher whose
        // goal sits ~90 deg off its heading gets a steering target speed of exactly 0 (see
        // MixedTrafficCrowd.SteerNonholonomic) and deadlocks -- it can never roll forward far enough to
        // turn. A small creep lets it nudge-and-turn onto the shoulder instead of freezing in place.
        _crowd = new MixedTrafficCrowd
        {
            SafetyMargin = cfg.OrcaPushSafetyMargin,
            Nonholonomic = true,
            CreepSpeed = cfg.OrcaCreepSpeed,
        };
    }

    public int Count => _crowd.Count;

    // Arm the band walls (PANIC-EVAC-PHASE3-DESIGN.md §4 / FakeNavMesh.BandWalls) -- confines pushers
    // to the known world, the shaped analogue of the pedestrian BoundaryLoop. Also records the
    // segments' bounding rectangle as the defensive containment backstop (see _boundsMinX etc.) --
    // BandWalls is exactly the navmesh's own bounding rectangle, so this reconstructs it exactly.
    public void ArmWalls(IEnumerable<(Vec2 A, Vec2 B)> walls, double thickness = 1.0)
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;

        foreach (var (a, b) in walls)
        {
            _crowd.AddWall(a, b, thickness);
            minX = Math.Min(minX, Math.Min(a.X, b.X));
            minY = Math.Min(minY, Math.Min(a.Y, b.Y));
            maxX = Math.Max(maxX, Math.Max(a.X, b.X));
            maxY = Math.Max(maxY, Math.Max(a.Y, b.Y));
        }

        if (!double.IsPositiveInfinity(minX))
        {
            _boundsMinX = minX;
            _boundsMinY = minY;
            _boundsMaxX = maxX;
            _boundsMaxY = maxY;
        }
    }

    // Passthrough for interior obstacles (abandoned cars, jammed lane vehicles) fed as blocks later.
    public void AddBlock(double minX, double minY, double maxX, double maxY, double thickness = 1.0)
        => _crowd.AddBlock(minX, minY, maxX, maxY, thickness);

    // Add a car that has just mounted the shoulder; returns its stable mover index. Wedge dwell for
    // this index starts at zero and the mover is active.
    public int AddCar(Vec2 pos, double headingRad, Vec2 goal)
    {
        var i = _crowd.Add(VehicleClass.Car, pos, goal, headingRad, maxSpeedOverride: _cfg.OrcaPushMaxSpeed);
        var initialDist = (goal - pos).Abs;
        // AddCar always appends (mirrors the crowd's own append-only Add), so the dwell/active lists
        // stay index-aligned with the crowd without any remapping.
        if (i == _dwell.Count)
        {
            _dwell.Add(0.0);
            _bestDistToGoal.Add(initialDist);
            _active.Add(true);
        }
        else
        {
            // Defensive: should not happen given append-only Add, but keep the parallel lists valid.
            while (_dwell.Count <= i)
            {
                _dwell.Add(0.0);
                _bestDistToGoal.Add(double.PositiveInfinity);
                _active.Add(false);
            }

            _dwell[i] = 0.0;
            _bestDistToGoal[i] = initialDist;
            _active[i] = true;
        }

        return i;
    }

    public void SetGoal(int i, Vec2 goal) => _crowd.SetGoal(i, goal);

    // Sub-stepped like the pedestrian crowd for smoothness (OrcaCrowdSubSteps). Wedge dwell is updated
    // ONCE per engine step (not per sub-step) from the post-step velocity, so the dwell threshold is in
    // real seconds regardless of the sub-step count.
    public void Step(double dt)
    {
        var subSteps = Math.Max(1, _cfg.OrcaCrowdSubSteps);
        var subDt = dt / subSteps;
        for (var s = 0; s < subSteps; s++)
        {
            _crowd.Step(subDt);
        }

        for (var i = 0; i < _crowd.Count; i++)
        {
            if (!_active[i])
            {
                continue;
            }

            // Defensive containment backstop: clamp back inside the band bounds and kill velocity if a
            // degenerate step carried this mover past the (primary) shaped-VO wall. A car pinned here
            // creeps in place against the wall and correctly starts accruing wedge dwell below. Two
            // independently-escaping movers can otherwise clamp to the SAME point (most often a
            // corner, where both axes saturate) and coincide; a small deterministic per-index nudge
            // (golden-angle spread, same trick as the evac layer's pedestrian-spawn offset) keeps
            // distinct movers apart without touching the ordinary (non-escaped) path at all.
            var p = _crowd.Position(i);
            var cx = Math.Min(Math.Max(p.X, _boundsMinX), _boundsMaxX);
            var cy = Math.Min(Math.Max(p.Y, _boundsMinY), _boundsMaxY);
            if (cx != p.X || cy != p.Y)
            {
                var nudge = DeterministicNudge(i);
                cx = Math.Min(Math.Max(cx + nudge.X, _boundsMinX), _boundsMaxX);
                cy = Math.Min(Math.Max(cy + nudge.Y, _boundsMinY), _boundsMaxY);
                _crowd.SetPose(i, new Vec2(cx, cy), Vec2.Zero);
            }

            // Progress-based wedge (see _bestDistToGoal doc comment): distance to the CURRENT goal
            // (SetGoal is called fresh each tick by the caller, but the away-goal direction is stable
            // tick to tick, so this tracks real progress, not goal churn). Strictly improving beyond
            // ProgressEpsilon resets the clock; failing to improve accrues it -- this catches a mover
            // pinned/deflected along a wall that keeps meaningful instantaneous speed while going
            // nowhere, which a raw speed threshold alone would never latch.
            var dist = (_crowd.Goal(i) - _crowd.Position(i)).Abs;
            if (dist < _bestDistToGoal[i] - ProgressEpsilon)
            {
                _bestDistToGoal[i] = dist;
                _dwell[i] = 0.0;
            }
            else
            {
                _dwell[i] += dt;
            }
        }
    }

    public Vec2 Position(int i) => _crowd.Position(i);
    public double Heading(int i) => _crowd.Heading(i);
    public Vec2 Velocity(int i) => _crowd.Velocity(i);

    public bool IsActive(int i) => _active[i];

    public void Deactivate(int i)
    {
        _crowd.Deactivate(i);
        _active[i] = false;
    }

    public bool IsWedged(int i) => _active[i] && _dwell[i] >= _cfg.OrcaWedgeDwellSeconds;

    // A tiny deterministic per-index spread (no RNG) so two movers whose escape clamp lands on the
    // same boundary point (most often a shared corner) don't sit exactly stacked. A Vogel/sunflower
    // spiral (r = c*sqrt(i+1), theta = i*goldenAngle) rather than a fixed-radius golden-angle ring:
    // a fixed ring can put two SPECIFIC indices arbitrarily close (the angle step alone gives no
    // distance floor), whereas the growing-radius spiral keeps EVERY pair of distinct indices at
    // roughly the same minimum spacing (~c*sqrt(pi)) regardless of how many indices collide here.
    private static Vec2 DeterministicNudge(int i)
    {
        const double goldenAngle = 2.399963;
        const double c = 1.5;   // ~2.4 m guaranteed min spacing between any two distinct indices
        var r = c * Math.Sqrt(i + 1);
        var a = i * goldenAngle;
        return new Vec2(Math.Cos(a) * r, Math.Sin(a) * r);
    }

    // ICrowdFootprintSource (T3.1 §3): expose ACTIVE pushers as world discs so the lane engine can
    // discover and avoid them too (via a CompositeFootprintSource alongside the pedestrian crowd).
    // Brute-force, deterministic index order -- mirrors OrcaCrowd.QueryNear exactly.
    public int QueryNear(double x, double y, double radius, Span<WorldDisc> into)
    {
        var rSq = radius * radius;
        var n = 0;
        for (var i = 0; i < _crowd.Count && n < into.Length; i++)
        {
            if (!_active[i])
            {
                continue;
            }

            var p = _crowd.Position(i);
            var dx = p.X - x;
            var dy = p.Y - y;
            if (dx * dx + dy * dy > rSq)
            {
                continue;
            }

            var v = _crowd.Velocity(i);
            into[n++] = new WorldDisc(p.X, p.Y, v.X, v.Y, CarFootprintRadius);
        }

        return n;
    }
}
