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

    // PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2a/§2b: pass-through so EvacConfig.UseCrowdSpatialHash can
    // reach the inner MixedTrafficCrowd's opt-in uniform spatial hash (T2.1). Default false (unset) ->
    // the inner crowd's own default -> brute-force, byte-identical.
    public bool UseSpatialHash
    {
        get => _crowd.UseSpatialHash;
        set => _crowd.UseSpatialHash = value;
    }

    // Arm the band walls (PANIC-EVAC-PHASE3-DESIGN.md §4 / FakeNavMesh.BandWalls) -- confines pushers
    // to the known world, the shaped analogue of the pedestrian BoundaryLoop. Confinement is now
    // guaranteed by the solver's own swept wall clip (docs/MIXED-WALL-CONTAINMENT.md W1); this just
    // feeds the wall segments into the crowd.
    public void ArmWalls(IEnumerable<(Vec2 A, Vec2 B)> walls, double thickness = 1.0)
    {
        foreach (var (a, b) in walls)
        {
            _crowd.AddWall(a, b, thickness);
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
