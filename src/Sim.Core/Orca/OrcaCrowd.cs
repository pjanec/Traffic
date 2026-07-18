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

    // POC-7a: every buffer Plan() writes while computing one agent's new velocity (neighbour list,
    // its parallel distance-sq array, the ORCA line list, the obstacle-segment list, the spatial-hash
    // candidate list) is now bundled into one ScratchSet instead of five separate instance fields, so
    // it can be OWNED PER PARALLEL WORKER instead of shared. The serial path keeps exactly one
    // instance-owned ScratchSet (`_scratch`, grown the same way the old fields were) and reuses it
    // across steps and agents just like before -- so with UseParallelStep=false the memory-reuse
    // pattern, and therefore every result, is byte-identical to pre-POC-7a. The parallel path (see
    // ParallelPlan) hands each Parallel.For worker its OWN ScratchSet via localInit, so concurrent
    // Plan(i) calls never touch each other's buffers -- no lock, no false sharing beyond normal cache
    // effects, and the PER-AGENT math is untouched (same reads of frozen state, same writes to
    // `_newVelocity[i]` only), so parallel output is bit-identical to serial (see
    // OrcaParallelStepTests).
    private sealed class ScratchSet
    {
        public OrcaSolver.Agent[] NeighbourScratch;
        public double[] NeighbourDistSq;   // parallel to NeighbourScratch, only used when MaxNeighbours > 0
        public OrcaLine[] LineScratch;
        public ObstacleSegment[] ObstacleSegmentScratch = Array.Empty<ObstacleSegment>();
        public int[] CandidateScratch = Array.Empty<int>();

        // Minimal scratch, grown lazily on first use -- what a fresh Parallel.For worker starts with.
        public ScratchSet()
            : this(1)
        {
        }

        public ScratchSet(int capacity)
        {
            NeighbourScratch = new OrcaSolver.Agent[capacity];
            NeighbourDistSq = new double[capacity];
            LineScratch = new OrcaLine[capacity];
        }
    }

    private readonly ScratchSet _scratch;

    // Removal-on-arrival: an arrived agent (RemoveOnArrival, within ArrivalRadius of its goal) is
    // deactivated -- it stops moving AND stops constraining others. All-true until deactivated; the
    // default (RemoveOnArrival == false) never flips one, so the whole mechanism is inert (a stand-
    // alone crowd is byte-identical to the pre-Q2 behaviour).
    private bool[] _active;

    // P0-1 (docs/PEDESTRIAN-TASKS.md, PEDESTRIAN-DESIGN.md §3(d)): real Add/Remove via a free-list of
    // recycled slots + a per-slot generation counter, mirroring ObstacleStore/ObstacleHandle exactly.
    //
    // `_slotAlive` is DISTINCT from `_active` above: `_active` is the RemoveOnArrival "parked" state (the
    // agent is still a live slot, just not moving/constraining -- Remove is never involved); `_slotAlive`
    // is whether the slot holds an agent AT ALL. A slot that is `!_slotAlive` is also always `!_active`
    // (Remove clears both), so every existing "skip when !_active" scan already skips a removed slot for
    // free -- the explicit `!_slotAlive` checks added alongside them (Step, RebuildGrid, AllArrived,
    // QueryNear) are belt-and-suspenders correctness for the NEW removal path, not a behavior change: with
    // `_slotAlive` all-true (Remove never called) they are always false and never taken, so the no-remove
    // path stays byte-identical to pre-P0-1.
    //
    // `_generation` starts at 1 for every never-used slot (0 is reserved for OrcaHandle.Invalid, exactly
    // ObstacleStore's convention) and is bumped again each time its slot is vacated, so any OrcaHandle
    // captured before a Remove is stale afterwards even if the slot is immediately recycled by a later
    // Add. `_freeSlots` is the LIFO of vacated slot indices Add pops from before ever growing/appending;
    // recycling does not disturb any OTHER agent's slot or handle, and does not touch iteration order
    // (iteration is always ascending slot index 0.._count-1, `_count` being the high-water mark of
    // slots ever allocated -- NOT the live agent count once Remove has been used).
    private uint[] _generation;
    private bool[] _slotAlive;
    private readonly Stack<int> _freeSlots = new();

    // Q3 uniform spatial hash (opt-in via UseSpatialHash; default off -> brute-force, byte-identical).
    // Rebuilt once per Step from the frozen positions: agents are bucketed by their cell (cell size ==
    // NeighbourDist, so an agent's whole neighbourhood lies in its own cell + the 8 around it). Each
    // per-agent query gathers those 3x3 cells' members, SORTS them ascending by index, then runs the
    // IDENTICAL gather the brute-force path runs -- so the neighbour set AND order (hence every LP, hence
    // every trajectory) are bit-identical to brute-force; the grid is a pure pre-filter that only skips
    // out-of-range agents cheaply. Pooled per-bucket arrays are reused across steps (no per-step alloc
    // once warm). Agent-agent only; the few external discs / obstacles stay brute-force.
    // Rebuilt SERIALLY once per Step (frozen positions, single writer) before the plan loop; read-only
    // for the rest of the step, including from every parallel worker in ParallelPlan -- concurrent
    // READS of a Dictionary and of the pooled bucket arrays are safe with no concurrent writer.
    private readonly Dictionary<long, int> _cellToBucket = new();
    private int[][] _bucketAgents = Array.Empty<int[]>();
    private int[] _bucketFill = Array.Empty<int>();
    private int _bucketCount;
    private double _cellSize;

    // Static obstacles (walls): a flat array of vertices, each closed polyline appended by
    // AddObstacle. Empty by default, so a stand-alone crowd (no AddObstacle call) is byte-identical
    // to the pre-obstacle behaviour -- obstacles span is empty, numObstLines == 0, nothing changes.
    private OrcaObstacle[] _obstacles = Array.Empty<OrcaObstacle>();
    private int _obstacleCount;

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

    // Nearest-neighbour cap (RVO2's maxNeighbors). 0 == unlimited (every in-range agent constrains
    // ego, in index order -- the pre-Q2 default, kept byte-identical). When > 0, ego keeps only its
    // MaxNeighbours nearest agents, ordered nearest-first (RVO2's insertAgentNeighbor). Beyond the
    // obvious perf win this is a *convergence* aid: at maximal symmetric packing (the antipodal
    // circle) considering ALL neighbours pins every agent in a perfectly balanced jam, whereas the
    // nearest-k selection -- broken by a deterministic distance/index tie-break -- desymmetrises the
    // constraint set enough for agents to slip past. External (cross-regime) discs and obstacles are
    // NOT capped by this (they are the few-count bridge/wall inputs).
    public int MaxNeighbours { get; set; }

    // Removal-on-arrival (RVO2's demo convention): when true, Step() deactivates any agent within
    // ArrivalRadius of its goal -- it parks and stops constraining others, freeing the space its
    // neighbours need to reach their own goals (the cascade that lets a dense crossing drain instead
    // of jamming). Default false -> no agent is ever deactivated -> byte-identical to pre-Q2.
    public bool RemoveOnArrival { get; set; }

    // Distance to goal at which RemoveOnArrival parks an agent. Only consulted when RemoveOnArrival.
    public double ArrivalRadius { get; set; } = 0.1;

    // Q3: use the uniform spatial hash for neighbour queries instead of the O(n^2) brute-force scan.
    // Default false (brute-force -> byte-identical). When true, results are bit-identical (the grid is a
    // pure pre-filter, candidates sorted to the same order); the win is O(n + neighbours) per agent
    // instead of O(n), which matters for large, spatially-spread crowds.
    public bool UseSpatialHash { get; set; }

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

    // POC-7a (docs/PEDESTRIAN-POC-PLAN.md POC-7, PEDESTRIAN-DESIGN.md §3c/§9): opt-in parallel PLAN
    // phase. Default false -> the code path is the ORIGINAL serial for-loop over the one instance-
    // owned `_scratch`, byte-for-byte unchanged, so every existing (default-serial) test stays
    // byte-identical. When true (and the crowd is big enough, see ParallelStepThreshold), Step() plans
    // agents with System.Threading.Tasks.Parallel.For instead, each worker holding its OWN ScratchSet
    // (Parallel.For's localInit/localFinally overload) so there is no shared-buffer race. The PLAN is
    // embarrassingly parallel by construction -- every _newVelocity[i] is a pure function of the
    // frozen start-of-step state, and each iteration writes ONLY its own slot -- so results are
    // bit-identical to serial regardless of thread count or scheduling (see OrcaParallelStepTests).
    public bool UseParallelStep { get; set; }

    // Caps the degree of parallelism of the Parallel.For plan/execute loops when UseParallelStep is
    // set. Default -1 == unlimited (TPL's own default). Mirrors Engine.MaxParallelism; never changes
    // the trajectory, only the wall-clock -- cached as a ParallelOptions so the hot loop allocates
    // nothing per step.
    private System.Threading.Tasks.ParallelOptions _parallelOptions = new() { MaxDegreeOfParallelism = -1 };
    public int MaxParallelism
    {
        get => _parallelOptions.MaxDegreeOfParallelism;
        set => _parallelOptions = new System.Threading.Tasks.ParallelOptions
        {
            MaxDegreeOfParallelism = value > 0 ? value : -1,
        };
    }

    // Below this agent count, Step() plans serially even when UseParallelStep is set -- the
    // Parallel.For dispatch/localInit overhead is not worth it for small crowds. Mirrors the lane
    // engine's ParallelPlanThreshold (Engine.cs), including its "gate on total count, a cheap O(1)
    // proxy for crowd size, not the exact active count" convention.
    private const int ParallelStepThreshold = 256;

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
        _active = new bool[capacity];
        _generation = new uint[capacity];
        _slotAlive = new bool[capacity];
        for (var i = 0; i < capacity; i++)
        {
            _generation[i] = 1;   // never-used slot starts at 1; 0 is reserved for OrcaHandle.Invalid
        }

        _scratch = new ScratchSet(capacity);
    }

    // High-water mark of slots ever allocated -- NOT the live agent count once Remove has been used (a
    // vacated, not-yet-recycled slot is still < _count but is neither active nor alive). Byte-identical
    // to "number of agents added" for any crowd that never calls Remove (the pre-P0-1 meaning).
    public int Count => _count;

    public Vec2 Position(int i) => _position[i];
    public Vec2 Position(OrcaHandle h) => _position[ResolveOrThrow(h)];

    public Vec2 Velocity(int i) => _velocity[i];
    public Vec2 Velocity(OrcaHandle h) => _velocity[ResolveOrThrow(h)];

    public Vec2 Goal(int i) => _goal[i];
    public Vec2 Goal(OrcaHandle h) => _goal[ResolveOrThrow(h)];

    public double Radius(int i) => _radius[i];
    public double Radius(OrcaHandle h) => _radius[ResolveOrThrow(h)];

    // False once RemoveOnArrival has parked agent i at its goal (it no longer moves or constrains).
    public bool IsActive(int i) => _active[i];
    public bool IsActive(OrcaHandle h) => _active[ResolveOrThrow(h)];

    public void SetGoal(int i, Vec2 goal) => _goal[i] = goal;
    public void SetGoal(OrcaHandle h, Vec2 goal) => _goal[ResolveOrThrow(h)] = goal;

    // True iff `h` still addresses a live (added, not yet removed) slot -- the non-throwing counterpart
    // to the accessors above, mirroring ObstacleStore.IsAlive. Handy for tests/callers that want to check
    // "is this handle still good" without a try/catch.
    public bool IsAlive(OrcaHandle h) =>
        h.Index >= 0 && h.Index < _count && _slotAlive[h.Index] && _generation[h.Index] == h.Generation;

    // Resolves a handle to its live slot index, or throws if `h` is stale (already Removed, possibly
    // recycled to a DIFFERENT agent since) or was never valid. A caller holding a stale handle is a bug
    // -- unlike Remove (see below), which is an inert no-op on a stale handle, matching this codebase's
    // established "removing something already gone is harmless" convention (ObstacleStore.Remove) while
    // still catching "read through a dangling reference" fast.
    private int ResolveOrThrow(OrcaHandle h)
    {
        if (!IsAlive(h))
        {
            throw new InvalidOperationException(
                $"OrcaHandle {h} is stale or invalid (crowd currently has {_count} slot(s) allocated).");
        }

        return h.Index;
    }

    // Add an agent; returns a stable OrcaHandle (index + generation). Pops a vacated slot off the free
    // list left by a prior Remove (reusing its index, generation already bumped there) before ever
    // growing/appending a brand new one -- O(1) either way. Grows the SoA arrays as needed.
    public OrcaHandle Add(Vec2 position, double radius, double maxSpeed, Vec2 goal, Vec2 velocity = default)
    {
        int i;
        if (_freeSlots.Count > 0)
        {
            i = _freeSlots.Pop();
        }
        else
        {
            if (_count == _position.Length)
            {
                Grow(_position.Length * 2);
            }

            i = _count++;
        }

        _position[i] = position;
        _velocity[i] = velocity;
        _goal[i] = goal;
        _radius[i] = radius;
        _maxSpeed[i] = maxSpeed;
        _newVelocity[i] = Vec2.Zero;
        _active[i] = true;
        _slotAlive[i] = true;
        return new OrcaHandle(i, _generation[i]);
    }

    // Removes an agent in O(1): the slot stops being planned/executed (Step), stops constraining other
    // agents' neighbour gathers, and stops being exposed via QueryNear -- from the NEXT Step onward (a
    // Remove mid-step, i.e. between Plan and Execute, is not a supported call pattern; Remove is meant to
    // be called from the same single-threaded input phase as Add, never concurrently with Step). Its
    // slot is pushed on the free list for a later Add to recycle; every OTHER agent's slot/handle is
    // completely undisturbed (no shifting, no shuffling -- the whole point of P0-1). Inert no-op if `h`
    // is already stale/invalid (mirrors ObstacleStore.Remove's "inert-when-absent" contract) -- see
    // ResolveOrThrow's remarks for why Remove and the read accessors deliberately differ here.
    public void Remove(OrcaHandle h)
    {
        if (!IsAlive(h))
        {
            return;
        }

        var i = h.Index;
        _slotAlive[i] = false;
        _active[i] = false;      // belt-and-suspenders: a dead slot is never "active" either
        _velocity[i] = Vec2.Zero;
        _generation[i]++;         // invalidates every handle to this slot, including this one
        _freeSlots.Push(i);
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
        // Removal-on-arrival: park agents that have reached their goal BEFORE the plan freezes state,
        // so this step's plan already ignores them as constraints (freeing the jam). Inert unless
        // RemoveOnArrival is set. Done in index order -> deterministic.
        if (RemoveOnArrival)
        {
            var arriveSq = ArrivalRadius * ArrivalRadius;
            for (var i = 0; i < _count; i++)
            {
                if (_active[i] && (_goal[i] - _position[i]).AbsSq <= arriveSq)
                {
                    _active[i] = false;
                    _velocity[i] = Vec2.Zero;   // parked: contributes zero relative velocity to others
                }
            }
        }

        // Rebuild the spatial hash from the frozen (post-removal) positions before planning, so every
        // per-agent query this step reads a consistent index. Inert unless UseSpatialHash.
        if (UseSpatialHash)
        {
            RebuildGrid();
        }

        // POC-7a: parallelize only when opted in AND the crowd is big enough to amortize dispatch
        // overhead (ParallelStepThreshold) -- default/small-crowd path is the untouched serial loop.
        var parallel = UseParallelStep && _count >= ParallelStepThreshold;

        // PLAN: every new velocity is a pure function of the frozen start-of-step state.
        if (parallel)
        {
            PlanParallel(dt);
        }
        else
        {
            for (var i = 0; i < _count; i++)
            {
                if (!_active[i] || !_slotAlive[i])
                {
                    continue;   // parked agent (stays put) or a removed/vacated slot (P0-1: contributes nothing)
                }

                _newVelocity[i] = Plan(i, dt, _scratch);
            }
        }

        // EXECUTE: commit velocities and integrate positions together. Trivially parallel (each
        // iteration reads/writes only its own index, no scratch needed) -- gated the same way as PLAN
        // purely to keep one threshold/knob for the whole step; bit-identical either way.
        if (parallel)
        {
            var velocity = _velocity;
            var position = _position;
            var newVelocity = _newVelocity;
            var active = _active;
            var slotAlive = _slotAlive;
            System.Threading.Tasks.Parallel.For(0, _count, _parallelOptions, i =>
            {
                if (!active[i] || !slotAlive[i])
                {
                    return;
                }

                velocity[i] = newVelocity[i];
                position[i] += velocity[i] * dt;
            });
        }
        else
        {
            for (var i = 0; i < _count; i++)
            {
                if (!_active[i] || !_slotAlive[i])
                {
                    continue;
                }

                _velocity[i] = _newVelocity[i];
                _position[i] += _velocity[i] * dt;
            }
        }

        _stepIndex++;   // advances the deterministic symmetry-break jitter (no effect when it is 0)
    }

    // POC-7a parallel PLAN: each Parallel.For worker gets its OWN ScratchSet via localInit (never
    // shared, so no race on the buffers Plan() writes) and reuses it across the iterations the TPL
    // partitions to that worker (localFinally is a no-op -- nothing to flush, ScratchSet is pure
    // scratch). Each iteration writes only `_newVelocity[i]`, its own slot -- order-independent,
    // bit-identical to the serial loop above regardless of partitioning or thread count.
    private void PlanParallel(double dt)
    {
        var active = _active;
        var slotAlive = _slotAlive;
        var newVelocity = _newVelocity;

        System.Threading.Tasks.Parallel.For(
            0, _count, _parallelOptions,
            () => new ScratchSet(),
            (i, _, local) =>
            {
                if (active[i] && slotAlive[i])
                {
                    newVelocity[i] = Plan(i, dt, local);
                }

                return local;
            },
            _ => { });
    }

    // True once every agent is within `epsilon` of its goal (the crowd has settled). Handy as a
    // behavioural stop condition in tests. A removed/vacated slot (P0-1) is skipped -- its stale
    // leftover position/goal must never hold this false forever.
    public bool AllArrived(double epsilon)
    {
        var epsSq = epsilon * epsilon;
        for (var i = 0; i < _count; i++)
        {
            if (!_slotAlive[i])
            {
                continue;
            }

            if ((_goal[i] - _position[i]).AbsSq > epsSq)
            {
                return false;
            }
        }

        return true;
    }

    // `scratch` is the caller's ScratchSet -- the one instance-owned `_scratch` on the serial path, a
    // per-worker one (from PlanParallel's Parallel.For localInit) on the parallel path. Everything
    // this method reads besides `scratch` is either frozen start-of-step state (positions/velocities/
    // obstacles/external discs) or the read-only-during-this-phase spatial-hash buckets -- never
    // mutated here -- so concurrent calls for different `i` (each with their own `scratch`) never
    // race.
    private Vec2 Plan(int i, double dt, ScratchSet scratch)
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
        if (scratch.NeighbourScratch.Length < cap)
        {
            Array.Resize(ref scratch.NeighbourScratch, cap);
            Array.Resize(ref scratch.NeighbourDistSq, cap);
        }

        // Line scratch holds obstacle lines FIRST, then agent lines, so it must fit both.
        var lineCap = _obstacleCount + cap;
        if (scratch.LineScratch.Length < lineCap)
        {
            Array.Resize(ref scratch.LineScratch, lineCap);
        }

        var near = scratch.NeighbourScratch.AsSpan(0, cap);
        var rangeSq = NeighbourDist * NeighbourDist;
        var maxN = MaxNeighbours;

        // Gather agent neighbours -- brute-force over all agents, or (opt-in) only the spatial-hash
        // candidates from the 3x3 cell neighbourhood, SORTED ascending so the result is bit-identical.
        int k;
        if (UseSpatialHash)
        {
            var candidates = GridCandidates(i, scratch);
            k = GatherAgentNeighbours(i, near, maxN, rangeSq, candidates, useAll: false, scratch);
        }
        else
        {
            k = GatherAgentNeighbours(i, near, maxN, rangeSq, default, useAll: true, scratch);
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
        if (scratch.ObstacleSegmentScratch.Length < _obstacleCount)
        {
            Array.Resize(ref scratch.ObstacleSegmentScratch, Math.Max(_obstacleCount, 8));
        }

        var obst = scratch.ObstacleSegmentScratch.AsSpan(0, _obstacleCount);
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
            scratch.LineScratch.AsSpan(0, oCount + k));
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
    // Deliberately does NOT check `_active` (a RemoveOnArrival-parked agent still occupies its spot and
    // is still exposed here, unchanged pre-P0-1 behavior) but DOES check `_slotAlive` (P0-1: a genuinely
    // Removed slot must never appear as a live footprint to the other regime) -- inert (no-op difference)
    // for any crowd that never calls Remove, since `_slotAlive` is then all-true.
    public int QueryNear(double x, double y, double radius, Span<WorldDisc> into)
    {
        var rSq = radius * radius;
        var n = 0;
        for (var i = 0; i < _count && n < into.Length; i++)
        {
            if (!_slotAlive[i])
            {
                continue;
            }

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

    // The single agent-neighbour gather used by BOTH the brute-force and spatial-hash paths (so they
    // are bit-identical by construction). Iterates either every agent (useAll) or the caller's
    // candidate indices (already sorted ascending), applying the exact same self/active/range filter
    // and the exact same unlimited-append or nearest-k bounded insertion. Returns the count written.
    private int GatherAgentNeighbours(
        int i, Span<OrcaSolver.Agent> near, int maxN, double rangeSq, ReadOnlySpan<int> candidates, bool useAll,
        ScratchSet scratch)
    {
        var k = 0;
        var m = useAll ? _count : candidates.Length;
        for (var idx = 0; idx < m; idx++)
        {
            var j = useAll ? idx : candidates[idx];
            // `!_active[j]` alone already excludes a removed slot (Remove clears `_active` too); the
            // `!_slotAlive[j]` is explicit belt-and-suspenders for the P0-1 removal path and is a no-op
            // when Remove is never called.
            if (j == i || !_active[j] || !_slotAlive[j])
            {
                continue;
            }

            var dsq = (_position[j] - _position[i]).AbsSq;
            if (dsq > rangeSq)
            {
                continue;
            }

            if (maxN <= 0)
            {
                near[k++] = new OrcaSolver.Agent(_position[j], _velocity[j], _radius[j]);   // reciprocal 0.5
                continue;
            }

            // Nearest-k (RVO2's insertAgentNeighbor): keep the maxN closest, sorted nearest-first.
            var neighbourDistSq = scratch.NeighbourDistSq;
            if (k == maxN && dsq >= neighbourDistSq[k - 1])
            {
                continue;   // full and no closer than the current farthest kept
            }

            var pos = k < maxN ? k : k - 1;   // when full, overwrite the farthest slot
            while (pos > 0 && neighbourDistSq[pos - 1] > dsq)
            {
                near[pos] = near[pos - 1];
                neighbourDistSq[pos] = neighbourDistSq[pos - 1];
                pos--;
            }

            near[pos] = new OrcaSolver.Agent(_position[j], _velocity[j], _radius[j]);   // reciprocal 0.5
            neighbourDistSq[pos] = dsq;
            if (k < maxN)
            {
                k++;
            }
        }

        return k;
    }

    // Rebuild the uniform spatial hash from the frozen (post-removal) positions. Cell size ==
    // NeighbourDist so an agent's whole in-range neighbourhood is within its cell + the 8 around it.
    // Pooled per-bucket arrays are cleared (fill reset) and reused; only growth allocates.
    private void RebuildGrid()
    {
        _cellSize = NeighbourDist;
        _cellToBucket.Clear();
        _bucketCount = 0;
        for (var i = 0; i < _count; i++)
        {
            if (!_active[i] || !_slotAlive[i])
            {
                continue;
            }

            var key = CellKey(_position[i]);
            if (!_cellToBucket.TryGetValue(key, out var bi))
            {
                bi = _bucketCount++;
                EnsureBucket(bi);
                _bucketFill[bi] = 0;
                _cellToBucket[key] = bi;
            }

            var arr = _bucketAgents[bi];
            var f = _bucketFill[bi];
            if (f == arr.Length)
            {
                Array.Resize(ref arr, arr.Length * 2);
                _bucketAgents[bi] = arr;
            }

            arr[f] = i;
            _bucketFill[bi] = f + 1;
        }
    }

    // Candidate agent indices for ego `i`: every agent in the 3x3 cell block around ego's cell, SORTED
    // ascending so the downstream gather matches brute-force order exactly. Reuses _candidateScratch.
    private ReadOnlySpan<int> GridCandidates(int i, ScratchSet scratch)
    {
        if (scratch.CandidateScratch.Length < _count)
        {
            Array.Resize(ref scratch.CandidateScratch, _count);
        }

        var candidateScratch = scratch.CandidateScratch;
        var cx = FloorDiv(_position[i].X, _cellSize);
        var cy = FloorDiv(_position[i].Y, _cellSize);
        var n = 0;
        for (var dx = -1; dx <= 1; dx++)
        {
            for (var dy = -1; dy <= 1; dy++)
            {
                if (_cellToBucket.TryGetValue(PackCell(cx + dx, cy + dy), out var bi))
                {
                    var fill = _bucketFill[bi];
                    var arr = _bucketAgents[bi];
                    for (var t = 0; t < fill; t++)
                    {
                        candidateScratch[n++] = arr[t];
                    }
                }
            }
        }

        Array.Sort(candidateScratch, 0, n);
        return candidateScratch.AsSpan(0, n);
    }

    private void EnsureBucket(int bi)
    {
        if (_bucketAgents.Length <= bi)
        {
            var newLen = Math.Max(bi + 1, Math.Max(8, _bucketAgents.Length * 2));
            Array.Resize(ref _bucketAgents, newLen);
            Array.Resize(ref _bucketFill, newLen);
        }

        _bucketAgents[bi] ??= new int[8];
    }

    private int FloorDiv(double v, double cell) => (int)Math.Floor(v / cell);

    private static long PackCell(int cx, int cy) => ((long)cx << 32) | (uint)cy;

    private long CellKey(Vec2 p) => PackCell(FloorDiv(p.X, _cellSize), FloorDiv(p.Y, _cellSize));

    private void Grow(int newCapacity)
    {
        var oldCapacity = _position.Length;
        Array.Resize(ref _position, newCapacity);
        Array.Resize(ref _velocity, newCapacity);
        Array.Resize(ref _goal, newCapacity);
        Array.Resize(ref _radius, newCapacity);
        Array.Resize(ref _maxSpeed, newCapacity);
        Array.Resize(ref _newVelocity, newCapacity);
        Array.Resize(ref _active, newCapacity);
        Array.Resize(ref _generation, newCapacity);
        Array.Resize(ref _slotAlive, newCapacity);
        for (var i = oldCapacity; i < newCapacity; i++)
        {
            _generation[i] = 1;   // never-used slot starts at 1; 0 is reserved for OrcaHandle.Invalid
        }

        Array.Resize(ref _scratch.NeighbourScratch, newCapacity);
        Array.Resize(ref _scratch.NeighbourDistSq, newCapacity);
        Array.Resize(ref _scratch.LineScratch, newCapacity);
    }
}
