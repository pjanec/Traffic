namespace Sim.Core.Orca;

// Open-space holonomic crowd driver for the ORCA layer (docs/LANELESS-DIRECTION.md, second regime).
// Owns a struct-of-arrays agent store (positions / velocities / radii / goals / maxSpeeds) so it
// scales to many agents with no per-agent allocation, and steps them with a strict PLAN/EXECUTE
// double buffer -- exactly the discipline the lane engine uses: every agent's new velocity is
// computed from the FROZEN start-of-step state of all agents, then positions/velocities are all
// committed together. That makes a step order-independent and trivially parallelisable, and keeps
// the result deterministic (fixed agent order, no RNG, no wall-clock).
//
// This subsystem is deliberately INDEPENDENT of the lane-parity core and of the string-keyed
// ExternalObstacle API (the owner's constraint: the external-agent API must scale -- no string ids,
// high agent counts). Agents are addressed by int index; a future entity-handle/SoA store (the
// SUMOSHARP-API redesign) can back these arrays without changing the solve. Bridging open-space
// agents and lane vehicles into one another's neighbourhoods (so SUMO traffic and crowd agents
// mutually avoid) is the cross-regime step and is intentionally NOT wired here yet.
public sealed class OrcaCrowd
{
    private Vec2[] _position;
    private Vec2[] _velocity;
    private Vec2[] _goal;
    private double[] _radius;
    private double[] _maxSpeed;
    private int _count;

    // Scratch reused across steps (plan/execute needs the frozen snapshot + a place for new values).
    private Vec2[] _newVelocity;
    private OrcaSolver.Agent[] _neighbourScratch;
    private OrcaLine[] _lineScratch;

    // Planning horizon (s): how far ahead ORCA guarantees collision-freedom. Larger -> earlier,
    // smoother avoidance; the RVO2 default is 10 s for agents.
    public double TimeHorizon { get; set; } = 10.0;

    // Agents beyond this range impose no constraint (the near-neighbourhood cutoff). Brute-force
    // O(n^2) filtered scan for correctness; a uniform grid / spatial hash is the perf axis (noted,
    // not done here -- same "correctness first, gated perf later" split as the lane RVO layer).
    public double NeighbourDist { get; set; } = 15.0;

    // Symmetry-breaking magnitude (m/s) added to each agent's preferred velocity. PURE ORCA
    // deadlocks on measure-zero PERFECTLY-symmetric inputs (e.g. N agents antipodal on a circle:
    // every agent's situation is a rotation of every other's, so the reciprocal solve leaves them
    // jammed at the centre touching but unable to advance -- min separation pins to the combined
    // radius and nobody progresses). RVO2's own circle demo breaks this with a tiny RANDOM
    // perturbation RE-DRAWN EACH STEP; we use a DETERMINISTIC per-(agent, step) jitter -- the
    // direction is a hash of the agent index and an internal step counter (no RNG, no wall-clock),
    // so it varies each step like RVO2's noise yet is fully reproducible run-to-run. Default 0.0
    // keeps the driver perfectly pure (real navmesh/crowd inputs are never exactly symmetric, so
    // they never need it); set a small value (~0.05 m/s) for degenerate symmetric setups.
    public double SymmetryBreak { get; set; }

    private int _stepIndex;

    public OrcaCrowd(int capacity = 16)
    {
        capacity = Math.Max(1, capacity);
        _position = new Vec2[capacity];
        _velocity = new Vec2[capacity];
        _goal = new Vec2[capacity];
        _radius = new double[capacity];
        _maxSpeed = new double[capacity];
        _newVelocity = new Vec2[capacity];
        _neighbourScratch = new OrcaSolver.Agent[capacity];
        _lineScratch = new OrcaLine[capacity];
    }

    public int Count => _count;

    public Vec2 Position(int i) => _position[i];
    public Vec2 Velocity(int i) => _velocity[i];
    public Vec2 Goal(int i) => _goal[i];
    public double Radius(int i) => _radius[i];

    public void SetGoal(int i, Vec2 goal) => _goal[i] = goal;

    // Add an agent; returns its stable int index. Grows the SoA arrays as needed.
    public int Add(Vec2 position, double radius, double maxSpeed, Vec2 goal, Vec2 velocity = default)
    {
        if (_count == _position.Length)
        {
            Grow(_position.Length * 2);
        }

        var i = _count++;
        _position[i] = position;
        _velocity[i] = velocity;
        _goal[i] = goal;
        _radius[i] = radius;
        _maxSpeed[i] = maxSpeed;
        _newVelocity[i] = Vec2.Zero;
        return i;
    }

    // Advance the whole crowd by dt using the plan/execute double buffer.
    public void Step(double dt)
    {
        // PLAN: every new velocity is a pure function of the frozen start-of-step state.
        for (var i = 0; i < _count; i++)
        {
            _newVelocity[i] = Plan(i, dt);
        }

        // EXECUTE: commit velocities and integrate positions together.
        for (var i = 0; i < _count; i++)
        {
            _velocity[i] = _newVelocity[i];
            _position[i] += _velocity[i] * dt;
        }

        _stepIndex++;   // advances the deterministic symmetry-break jitter (no effect when it is 0)
    }

    // True once every agent is within `epsilon` of its goal (the crowd has settled). Handy as a
    // behavioural stop condition in tests.
    public bool AllArrived(double epsilon)
    {
        var epsSq = epsilon * epsilon;
        for (var i = 0; i < _count; i++)
        {
            if ((_goal[i] - _position[i]).AbsSq > epsSq)
            {
                return false;
            }
        }

        return true;
    }

    private Vec2 Plan(int i, double dt)
    {
        var self = new OrcaSolver.Agent(_position[i], _velocity[i], _radius[i]);
        var maxSpeed = _maxSpeed[i];

        // Preferred velocity: straight at the goal at maxSpeed, but never overshoot it in one step.
        var toGoal = _goal[i] - _position[i];
        var dist = toGoal.Abs;
        Vec2 pref;
        if (dist < 1e-9)
        {
            pref = Vec2.Zero;
        }
        else
        {
            var speed = Math.Min(maxSpeed, dist / dt);   // ease into the goal instead of oscillating
            pref = toGoal / dist * speed;

            if (SymmetryBreak > 0.0)
            {
                // Deterministic tie-break, re-drawn each step (mirrors RVO2's per-step random nudge
                // but hashed from (agent index, step counter) so it is reproducible, not random).
                // A fixed-per-agent push re-symmetrises; varying it each step reliably shakes the jam
                // loose while averaging to ~0 so it does not bias straight-line travel.
                var h = unchecked((uint)(i * 73856093) ^ (uint)(_stepIndex * 19349663));
                var a = h * (2.0 * Math.PI / 4294967296.0);
                pref += new Vec2(Math.Cos(a), Math.Sin(a)) * SymmetryBreak;
            }
        }

        // Gather near neighbours (frozen state), excluding self and anything out of range.
        var near = _neighbourScratch.AsSpan(0, _count);
        var k = 0;
        var rangeSq = NeighbourDist * NeighbourDist;
        for (var j = 0; j < _count; j++)
        {
            if (j == i)
            {
                continue;
            }

            if ((_position[j] - _position[i]).AbsSq > rangeSq)
            {
                continue;
            }

            near[k++] = new OrcaSolver.Agent(_position[j], _velocity[j], _radius[j]);
        }

        return OrcaSolver.ComputeNewVelocity(
            self, near[..k], pref, maxSpeed, TimeHorizon, dt, _lineScratch.AsSpan(0, k));
    }

    private void Grow(int newCapacity)
    {
        Array.Resize(ref _position, newCapacity);
        Array.Resize(ref _velocity, newCapacity);
        Array.Resize(ref _goal, newCapacity);
        Array.Resize(ref _radius, newCapacity);
        Array.Resize(ref _maxSpeed, newCapacity);
        Array.Resize(ref _newVelocity, newCapacity);
        Array.Resize(ref _neighbourScratch, newCapacity);
        Array.Resize(ref _lineScratch, newCapacity);
    }
}
