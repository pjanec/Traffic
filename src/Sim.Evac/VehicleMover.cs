using Sim.Core.Mixed;
using Sim.Core.Orca;

namespace Sim.Evac;

// PANIC-EVAC-PHASE3-DESIGN.md §1, §4 (Option A, T1.2): a Sim.Evac-owned manager wrapping ONE
// Sim.Core.Mixed.MixedTrafficCrowd -- the shaped, non-holonomic free-space driver for cars that have
// mounted the shoulder (the "Orca-push" stage). This class owns nothing lane-related; it is a thin,
// standalone, deterministic wrapper: index bookkeeping (per-mover wedge dwell + active flag) around
// the core crowd. Parity-exempt (Sim.Evac never touches Sim.Core's parity-critical Engine seams via
// this class), reachable only from the evac layer / viz, so it cannot move the determinism hash.
public sealed class VehicleMover
{
    private readonly EvacConfig _cfg;
    private readonly MixedTrafficCrowd _crowd;

    // Per-mover wedge tracking, indexed by the SAME index the underlying crowd assigns (AddCar always
    // appends, so this list and the crowd's index space stay in lockstep).
    private readonly List<double> _dwell = new();
    private readonly List<bool> _active = new();

    public VehicleMover(EvacConfig cfg)
    {
        _cfg = cfg;
        // Nonholonomic=true: PANIC-EVAC-PHASE3-DESIGN.md is explicit that Orca-push is a SHAPED,
        // NON-HOLONOMIC free-space push (kinematic-bicycle steering, no sideways teleport, no
        // pivot-in-place) -- the whole point of driving a car with MixedTrafficCrowd instead of the
        // holonomic pedestrian OrcaCrowd.
        _crowd = new MixedTrafficCrowd { SafetyMargin = cfg.OrcaPushSafetyMargin, Nonholonomic = true };
    }

    public int Count => _crowd.Count;

    // Arm the band walls (PANIC-EVAC-PHASE3-DESIGN.md §4 / FakeNavMesh.BandWalls) -- confines pushers
    // to the known world, the shaped analogue of the pedestrian BoundaryLoop.
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
        var i = _crowd.Add(VehicleClass.Car, pos, goal, headingRad);
        // AddCar always appends (mirrors the crowd's own append-only Add), so the dwell/active lists
        // stay index-aligned with the crowd without any remapping.
        if (i == _dwell.Count)
        {
            _dwell.Add(0.0);
            _active.Add(true);
        }
        else
        {
            // Defensive: should not happen given append-only Add, but keep the parallel lists valid.
            while (_dwell.Count <= i)
            {
                _dwell.Add(0.0);
                _active.Add(false);
            }

            _dwell[i] = 0.0;
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

            if (_crowd.Velocity(i).Abs < _cfg.OrcaWedgeSpeed)
            {
                _dwell[i] += dt;
            }
            else
            {
                _dwell[i] = 0.0;
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
}
