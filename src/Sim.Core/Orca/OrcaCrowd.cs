using Sim.Core.Bridge;

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
// mutually avoid) is the cross-regime bridge: this driver participates in it from BOTH sides --
// it exposes its agents as world discs (ICrowdFootprintSource, so the lane engine can avoid them)
// and it consumes external world discs (SetExternalObstacles, so its agents avoid vehicles).
public sealed class OrcaCrowd : ICrowdFootprintSource
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

    // Static obstacles (walls): a flat array of vertices, each closed polyline appended by
    // AddObstacle. Empty by default, so a stand-alone crowd (no AddObstacle call) is byte-identical
    // to the pre-obstacle behaviour -- obstacles span is empty, numObstLines == 0, nothing changes.
    private OrcaObstacle[] _obstacles = Array.Empty<OrcaObstacle>();
    private int _obstacleCount;
    private ObstacleSegment[] _obstacleSegmentScratch = Array.Empty<ObstacleSegment>();

    // Cross-regime bridge (Direction A -- crowd avoids vehicles): external world-space discs from the
    // OTHER regime (lane vehicles, projected to discs) that every agent also avoids, ONE-SIDED
    // (responsibility 1.0: the crowd yields fully; the vehicle avoids the crowd through its own lane
    // solve -- a conservative mutual double-yield that is always collision-safe). Empty by default,
    // so a stand-alone crowd is unaffected. Updated each step by the coupling before Step().
    private WorldDisc[] _externalDiscs = System.Array.Empty<WorldDisc>();
    private int _externalDiscCount;

    // Planning horizon (s): how far ahead ORCA guarantees collision-freedom. Larger -> earlier,
    // smoother avoidance; the RVO2 default is 10 s for agents.
    public double TimeHorizon { get; set; } = 10.0;

    // Planning horizon (s) for STATIC obstacles specifically (RVO2 uses a separate, shorter horizon
    // for walls than for other agents by default).
    public double TimeHorizonObst { get; set; } = 5.0;

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

    // Add one static-obstacle polyline (a "wall"), ported EXACTLY from RVO2's
    // RVOSimulator::addObstacle: the vertex list is always treated as a CLOSED loop (vertex n-1
    // connects back to vertex 0), each vertex becomes one OrcaObstacle node linked to its neighbours
    // in the loop, with UnitDir pointing to the next vertex and IsConvex computed from the interior
    // angle at that vertex (both vertices are convex when the loop has exactly 2, i.e. a thin
    // two-sided wall). May be called multiple times; each call appends one more closed loop. Returns
    // the crowd-array index of the loop's first vertex (RVO2's obstacle-loop handle).
    public int AddObstacle(IReadOnlyList<Vec2> vertices)
    {
        if (vertices.Count < 2)
        {
            throw new ArgumentException("An obstacle polyline needs at least 2 vertices.", nameof(vertices));
        }

        var n = vertices.Count;
        var baseIndex = _obstacleCount;
        EnsureObstacleCapacity(baseIndex + n);

        for (var i = 0; i < n; i++)
        {
            var point = vertices[i];
            var nextPoint = vertices[(i + 1) % n];
            var unitDir = (nextPoint - point).Normalized();

            bool isConvex;
            if (n == 2)
            {
                isConvex = true;
            }
            else
            {
                var prevPoint = vertices[(i - 1 + n) % n];
                isConvex = OrcaObstacle.LeftOf(prevPoint, point, nextPoint) >= 0.0;
            }

            var nextIndex = baseIndex + (i + 1) % n;
            var prevIndex = baseIndex + (i - 1 + n) % n;
            _obstacles[baseIndex + i] = new OrcaObstacle(point, unitDir, isConvex, prevIndex, nextIndex);
        }

        _obstacleCount += n;
        return baseIndex;
    }

    private void EnsureObstacleCapacity(int needed)
    {
        if (_obstacles.Length >= needed)
        {
            return;
        }

        var newCapacity = Math.Max(needed, Math.Max(8, _obstacles.Length * 2));
        Array.Resize(ref _obstacles, newCapacity);
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

        // Gather near neighbours (frozen state), excluding self and anything out of range. Room for
        // every other agent PLUS every external (cross-regime) disc.
        var cap = _count + _externalDiscCount;
        if (_neighbourScratch.Length < cap)
        {
            Array.Resize(ref _neighbourScratch, cap);
        }

        // Line scratch holds obstacle lines FIRST, then agent lines, so it must fit both.
        var lineCap = _obstacleCount + cap;
        if (_lineScratch.Length < lineCap)
        {
            Array.Resize(ref _lineScratch, lineCap);
        }

        var near = _neighbourScratch.AsSpan(0, cap);
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

            near[k++] = new OrcaSolver.Agent(_position[j], _velocity[j], _radius[j]);   // reciprocal 0.5
        }

        // Cross-regime discs (vehicles): avoided ONE-SIDED (responsibility 1.0) -- the crowd yields
        // fully, the vehicle avoids the crowd via its own lane solve. Same range cutoff.
        for (var d = 0; d < _externalDiscCount; d++)
        {
            var disc = _externalDiscs[d];
            var ex = disc.X - _position[i].X;
            var ey = disc.Y - _position[i].Y;
            if (ex * ex + ey * ey > rangeSq)
            {
                continue;
            }

            near[k++] = new OrcaSolver.Agent(
                new Vec2(disc.X, disc.Y), new Vec2(disc.Vx, disc.Vy), disc.Radius, responsibility: 1.0);
        }

        // Gather obstacle (wall) neighbours: brute-force scan (same "correctness first" cutoff style
        // as the agent scan above -- RVO2 uses a kd-tree here, purely for speed, not for a different
        // result). Mirrors RVO2's Agent::computeNeighbors obstacle range exactly: rangeSq =
        // (timeHorizonObst * maxSpeed + radius)^2, and membership is by distance to the obstacle
        // EDGE (vertex -> next vertex), not to the vertex point.
        //
        // Critically, this ALSO reproduces RVO2's KdTree::queryObstacleTreeRecursive occlusion gate:
        // an edge is only a candidate neighbour when the agent is on its RIGHT side (leftOf < 0) --
        // i.e. on the exterior side of that directed edge (obstacle vertices wind so the polygon
        // interior is on the LEFT, RVO2 convention: counterclockwise for solid obstacles). This is
        // NOT a redundant pruning optimisation the kd-tree needs and a brute-force scan can skip --
        // for any polygon with more than one edge in reach, omitting it lets the FAR side of the
        // obstacle (which the agent hasn't reached and isn't "outside" of) also contribute an ORCA
        // line, and a near + far edge together can force a jointly-infeasible half-plane pair (the
        // near edge wants the agent to keep clear of it from the outside, the far edge -- seen from
        // its own "outside", which is the near edge's "inside" -- wants the opposite), corrupting the
        // solve. Confirmed by direct reproduction: a solid rectangle without this filter froze/ate a
        // straight-walking agent from ~4.85 units away, well before any real contact; adding the
        // filter fixed it. (A 2-vertex "thin wall" is the degenerate case: its two directed edges are
        // the same segment reversed, so exactly one of them ever passes this test -- the other side.)
        if (_obstacleSegmentScratch.Length < _obstacleCount)
        {
            Array.Resize(ref _obstacleSegmentScratch, Math.Max(_obstacleCount, 8));
        }

        var obst = _obstacleSegmentScratch.AsSpan(0, _obstacleCount);
        var oCount = 0;
        var obstRange = TimeHorizonObst * maxSpeed + _radius[i];
        var rangeSqObst = obstRange * obstRange;
        for (var v = 0; v < _obstacleCount; v++)
        {
            var node = _obstacles[v];
            var next = _obstacles[node.NextIndex];

            if (OrcaObstacle.LeftOf(node.Point, next.Point, _position[i]) >= 0.0)
            {
                continue;   // agent is on the polygon-interior side of this directed edge -- not a candidate
            }

            var distSq = OrcaObstacle.DistanceSquaredToSegment(node.Point, next.Point, _position[i]);
            if (distSq >= rangeSqObst)
            {
                continue;
            }

            var prev = _obstacles[node.PrevIndex];
            obst[oCount++] = new ObstacleSegment(
                node.Point, next.Point, node.UnitDir, next.UnitDir, prev.UnitDir, node.IsConvex, next.IsConvex);
        }

        return OrcaSolver.ComputeNewVelocity(
            self, near[..k], obst[..oCount], pref, maxSpeed, TimeHorizon, TimeHorizonObst, dt,
            _lineScratch.AsSpan(0, oCount + k));
    }

    // Cross-regime bridge (Direction A): replace the external world-disc list every agent avoids this
    // step (the lane vehicles, projected to discs by the coupling). Copied into an internal buffer so
    // the caller's span need not outlive the call; cleared by passing an empty span.
    public void SetExternalObstacles(ReadOnlySpan<WorldDisc> discs)
    {
        if (_externalDiscs.Length < discs.Length)
        {
            _externalDiscs = new WorldDisc[Math.Max(discs.Length, 8)];
        }

        discs.CopyTo(_externalDiscs);
        _externalDiscCount = discs.Length;
    }

    // Cross-regime bridge (Direction B): expose this crowd's agents as world discs so the OTHER regime
    // (the lane engine) can discover and avoid them. Brute-force radius scan (the spatial hash is the
    // shared perf axis, not built here); fills `into` up to its length and returns the count written.
    public int QueryNear(double x, double y, double radius, Span<WorldDisc> into)
    {
        var rSq = radius * radius;
        var n = 0;
        for (var i = 0; i < _count && n < into.Length; i++)
        {
            var dx = _position[i].X - x;
            var dy = _position[i].Y - y;
            if (dx * dx + dy * dy > rSq)
            {
                continue;
            }

            into[n++] = new WorldDisc(_position[i].X, _position[i].Y, _velocity[i].X, _velocity[i].Y, _radius[i]);
        }

        return n;
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
