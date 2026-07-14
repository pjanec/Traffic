using Sim.Core.Orca;

namespace Sim.Core.Mixed;

// Open-space driver for believable Indian mixed traffic (docs/INDIA-TRAFFIC.md). It is the shaped,
// asymmetric-priority sibling of OrcaCrowd: a struct-of-arrays store of heterogeneous vehicles
// (VehicleClass), stepped with the SAME strict PLAN/EXECUTE double buffer (every new velocity is a
// pure function of the frozen start-of-step state, then all positions commit together) so a step is
// order-independent, deterministic (fixed order, hashed symmetry-break, no RNG/wall-clock), and
// parallel-ready. Three things distinguish it from the disc crowd:
//
//   1. SHAPE. Each vehicle carries an oriented convex footprint (ConvexShape), and avoidance runs
//      through ShapedVoSolver -- anisotropic, so a long bus is awkward broadside and a small
//      motorcycle threads gaps.
//   2. SOFT PRIORITY. There is no hard right of way. For an ordered pair (ego i, neighbour j) ego's
//      responsibility for avoiding j is assert_j / (assert_i + assert_j): equal assertiveness -> 0.5
//      (mutual jostle); a bus (high assert) makes a motorcycle (low assert) yield most of the way,
//      while the bus barely deviates -- and the two reciprocal shares always sum to 1, so the pair's
//      total avoidance still fully clears the collision (collision-safe, just asymmetrically shared).
//      Dominant streams emerge from this alone, with no signs or signals.
//   3. ROAD CONFINEMENT. Static walls (curbs / building edges) are modelled as zero-velocity shaped
//      "neighbours" with responsibility 1.0 (the vehicle yields fully to the wall). This reuses the
//      shaped VO for confinement instead of a separate obstacle path; it is isotropic-per-wall but
//      keeps vehicles on the drivable surface (see INDIA-TRAFFIC.md section 5 for the honest limit).
//
// Parity-exempt: reachable only from the Mixed layer / viz, never from the lane Engine, so it cannot
// move the determinism hash. Validated behaviourally (no-interpenetration via SAT, on-road,
// deterministic, emergent priority) -- there is no SUMO golden for this regime.
public sealed class MixedTrafficCrowd
{
    private Vec2[] _position;
    private Vec2[] _velocity;
    private Vec2[] _goal;
    private double[] _heading;        // radians; footprint orientation + travel direction
    private double[] _maxSpeed;
    private double[] _assert;
    private VehicleClass[] _class;
    private ConvexShape[] _shapeProto; // LOCAL-frame footprint (rotated to heading each plan)
    private ConvexShape[] _solveProto; // LOCAL-frame footprint inflated by SafetyMargin, used in the VO
    private bool[] _active;
    private int _count;

    private Vec2[] _newVelocity;

    // Static walls: world centre + a world-oriented, origin-centred footprint. Fed to the solver as
    // zero-velocity, responsibility-1.0 shaped neighbours.
    private readonly List<(Vec2 Centre, ConvexShape Shape)> _walls = new();

    // Scratch for a plan's neighbour set (vehicles in range + walls in range).
    private ShapedVoSolver.ShapedAgent[] _nbScratch = new ShapedVoSolver.ShapedAgent[16];
    private double[] _nbDistSq = new double[16];
    private OrcaLine[] _lineScratch = new OrcaLine[16];

    // Planning horizon (s): how far ahead avoidance is guaranteed. Shorter than the pedestrian ORCA
    // default -- vehicles with large footprints and a long horizon over-constrain and freeze; traffic
    // reacts on a few seconds.
    public double TimeHorizon { get; set; } = 3.0;

    // Neighbours beyond this centre-distance impose no constraint.
    public double NeighbourDist { get; set; } = 22.0;

    // Nearest-neighbour cap (0 = unlimited). Bounds per-agent work and, as in the disc crowd,
    // desymmetrises dense packing so vehicles slip past instead of pinning.
    public int MaxNeighbours { get; set; } = 12;

    // Deterministic per-(agent, step) preferred-velocity jitter magnitude (m/s), same mechanism as
    // OrcaCrowd.SymmetryBreak. A small value shakes loose perfectly-symmetric standoffs; 0 keeps it
    // pure. Real Indian streets are never symmetric, so a tiny value is plenty.
    public double SymmetryBreak { get; set; }

    // Park + stop-constraining a vehicle once within ArrivalRadius of its goal (frees the space it
    // occupies so the flow drains rather than jams). Off by default.
    public bool RemoveOnArrival { get; set; }
    public double ArrivalRadius { get; set; } = 1.5;

    // Speed below which heading is held rather than re-derived from velocity (avoids spin at a stop).
    public double HeadingHoldSpeed { get; set; } = 0.3;

    // Creep-and-turn floor (m/s, non-holonomic only). Because turn rate is tied to forward speed, a
    // vehicle that brakes fully to turn can no longer steer -- it deadlocks. A small creep keeps it
    // rolling (so it can keep steering out of a jam) WHENEVER the avoidance solve still permits
    // forward motion; if the solve says stop (blocked by an obstacle/vehicle), the creep is clamped to
    // that and the vehicle genuinely stops (no in-place spin). Default 0 (strict model); scenes set a
    // small value (~0.8) to keep dense junctions flowing.
    public double CreepSpeed { get; set; }

    // NON-HOLONOMIC steering (docs/INDIA-TRAFFIC.md). When true, the holonomic velocity the shaped
    // ORCA solve produces is treated as a STEERING TARGET, not the actual motion: the vehicle turns
    // toward it at a rate bounded by its speed and minimum turning radius (so it cannot pivot in
    // place), never reverses (a backward-pointing target becomes braking), and changes speed within
    // its acceleration limits. This removes the "bacteria" pathology of the pure holonomic model --
    // 180-degree heading flips and backward darts in congestion -- and produces car-like motion.
    // Default false keeps the pure-holonomic behaviour (and the existing holonomic tests) intact.
    public bool Nonholonomic { get; set; }

    // Tracking-error safety margin (m): the footprint is grown by this much FOR THE AVOIDANCE SOLVE
    // only, so real bodies stay apart even when bounded (non-holonomic) steering cannot perfectly
    // track the ORCA velocity (NH-ORCA). Rendering and overlap use the TRUE footprint. Must be set
    // before Add(); default 0 (no inflation -> pure holonomic behaviour and tests unchanged).
    public double SafetyMargin { get; set; }

    private double[] _newHeading;

    private int _stepIndex;

    // PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2a: uniform spatial hash for the vehicle-neighbour query,
    // mirroring OrcaCrowd's proven Q3 grid EXACTLY (own copies -- not shared with OrcaCrowd's grid).
    // Default false -> brute-force, byte-identical to pre-Tier2 behaviour. When true, RebuildGrid()
    // buckets the frozen start-of-step positions (cell edge == NeighbourDist, so a vehicle's whole
    // neighbourhood lies within its cell + the 8 around it); GridCandidates(i) gathers that 3x3 block
    // and SORTS it ascending by index before handing it to GatherVehicleNeighbours -- the SAME method
    // the brute path calls (useAll:true) -- so the neighbour set AND order (hence every LP, hence
    // every position/heading) are bit-identical to brute-force. Walls are NOT gridded (unchanged, few
    // in number, appended after the capped vehicle set).
    public bool UseSpatialHash { get; set; }

    private readonly Dictionary<long, int> _cellToBucket = new();
    private int[][] _bucketAgents = Array.Empty<int[]>();
    private int[] _bucketFill = Array.Empty<int>();
    private int _bucketCount;
    private double _cellSize;
    private int[] _candidateScratch = Array.Empty<int>();

    public MixedTrafficCrowd(int capacity = 16)
    {
        capacity = Math.Max(1, capacity);
        _position = new Vec2[capacity];
        _velocity = new Vec2[capacity];
        _goal = new Vec2[capacity];
        _heading = new double[capacity];
        _maxSpeed = new double[capacity];
        _assert = new double[capacity];
        _class = new VehicleClass[capacity];
        _shapeProto = new ConvexShape[capacity];
        _solveProto = new ConvexShape[capacity];
        _active = new bool[capacity];
        _newVelocity = new Vec2[capacity];
        _newHeading = new double[capacity];
    }

    public int Count => _count;
    public int WallCount => _walls.Count;
    public (Vec2 Centre, ConvexShape Shape) Wall(int i) => _walls[i];

    public Vec2 Position(int i) => _position[i];
    public Vec2 Velocity(int i) => _velocity[i];
    public Vec2 Goal(int i) => _goal[i];
    public double Heading(int i) => _heading[i];
    public VehicleClass Class(int i) => _class[i];
    public bool IsActive(int i) => _active[i];
    public void SetGoal(int i, Vec2 goal) => _goal[i] = goal;

    // Add a vehicle; returns its stable index. Initial heading defaults to face the goal.
    // maxSpeedOverride lets a scene run a congested crawl below the class free-flow speed.
    public int Add(
        VehicleClass cls, Vec2 position, Vec2 goal,
        double? headingRad = null, Vec2 velocity = default, double? maxSpeedOverride = null)
    {
        if (_count == _position.Length)
        {
            Grow(_position.Length * 2);
        }

        var i = _count++;
        _position[i] = position;
        _velocity[i] = velocity;
        _goal[i] = goal;
        var toGoal = goal - position;
        _heading[i] = headingRad ?? (toGoal.AbsSq > 1e-9 ? Math.Atan2(toGoal.Y, toGoal.X) : 0.0);
        _maxSpeed[i] = maxSpeedOverride ?? cls.MaxSpeed;
        _assert[i] = cls.Assertiveness;
        _class[i] = cls;
        _shapeProto[i] = cls.Shape();
        _solveProto[i] = _shapeProto[i].Inflate(SafetyMargin);
        _active[i] = true;
        _newVelocity[i] = Vec2.Zero;
        return i;
    }

    // Target length of a single wall obstacle box. A long wall is TILED into segments of about this
    // length rather than kept as one huge box: the shaped VO from a small LOCAL box (the segment near
    // the vehicle) is well-conditioned, whereas one 80 m box -- whose centroid sits far to the side of
    // a vehicle beside its middle -- yields a poor half-plane that leaks (mirrors why RVO2 handles
    // walls as short per-edge obstacles, not one giant polygon).
    public double WallSegmentLength { get; set; } = 3.5;

    // Add a straight wall of the given thickness as static shaped obstacle(s), tiled into
    // WallSegmentLength-ish boxes so confinement stays local and robust.
    public void AddWall(Vec2 a, Vec2 b, double thickness)
    {
        var dir = b - a;
        var len = dir.Abs;
        if (len < 1e-9)
        {
            return;
        }

        var angle = Math.Atan2(dir.Y, dir.X);
        var segs = Math.Max(1, (int)Math.Round(len / WallSegmentLength));
        var segLen = len / segs;
        var unit = dir / len;
        // Each box slightly overlaps its neighbour (segLen + thickness) so there is no seam gap.
        var boxShape = ConvexShape.Rectangle(segLen + thickness, thickness).RotatedTo(angle);
        for (var s = 0; s < segs; s++)
        {
            var centre = a + unit * (segLen * (s + 0.5));
            _walls.Add((centre, boxShape));
        }
    }

    // Add a closed rectangular block (building) as four walls.
    public void AddBlock(double minX, double minY, double maxX, double maxY, double thickness)
    {
        var bl = new Vec2(minX, minY);
        var br = new Vec2(maxX, minY);
        var tr = new Vec2(maxX, maxY);
        var tl = new Vec2(minX, maxY);
        AddWall(bl, br, thickness);
        AddWall(br, tr, thickness);
        AddWall(tr, tl, thickness);
        AddWall(tl, bl, thickness);
    }

    public void Step(double dt)
    {
        if (RemoveOnArrival)
        {
            var arriveSq = ArrivalRadius * ArrivalRadius;
            for (var i = 0; i < _count; i++)
            {
                if (_active[i] && (_goal[i] - _position[i]).AbsSq <= arriveSq)
                {
                    _active[i] = false;
                    _velocity[i] = Vec2.Zero;
                }
            }
        }

        // Rebuild the spatial hash from the frozen (post-removal) positions before planning, so every
        // per-vehicle query this step reads a consistent index. Inert unless UseSpatialHash (mirrors
        // OrcaCrowd.Step exactly). Safe/order-independent because Step is already PLAN/EXECUTE double-
        // buffered: the plan loop below only reads frozen _position/_velocity/_heading.
        if (UseSpatialHash)
        {
            RebuildGrid();
        }

        for (var i = 0; i < _count; i++)
        {
            if (!_active[i])
            {
                continue;
            }

            var desired = Plan(i, dt);
            if (Nonholonomic)
            {
                _newVelocity[i] = SteerNonholonomic(i, desired, dt, out _newHeading[i]);
            }
            else
            {
                _newVelocity[i] = desired;
            }
        }

        for (var i = 0; i < _count; i++)
        {
            if (!_active[i])
            {
                continue;
            }

            if (Nonholonomic)
            {
                // Kinematic-bicycle integration: the body pivots about its REAR AXLE, not its centre.
                // The rear axle advances along the (mean) heading while the heading rotates, so the
                // rear tracks inside the turn and the front/body sweeps wide -- real off-tracking. For
                // a rigid body the orientation IS the back->front chord (SUMO computeAngle), so this
                // also yields the correct long-vehicle heading with no separate approximation.
                var theta0 = _heading[i];
                var theta1 = _newHeading[i];
                var turn = theta1 - theta0;                 // bounded per step -> no wrap
                var speed = _newVelocity[i].Abs;
                var lc = 0.5 * _class[i].Wheelbase;          // centre-to-rear-axle offset
                var c0 = _position[i];
                var rear0 = new Vec2(c0.X - lc * Math.Cos(theta0), c0.Y - lc * Math.Sin(theta0));
                var thetaMid = theta0 + 0.5 * turn;
                var rear1 = new Vec2(
                    rear0.X + speed * dt * Math.Cos(thetaMid),
                    rear0.Y + speed * dt * Math.Sin(thetaMid));
                var c1 = new Vec2(rear1.X + lc * Math.Cos(theta1), rear1.Y + lc * Math.Sin(theta1));

                // Swept wall clip (docs/MIXED-WALL-CONTAINMENT.md W1): the ORCA wall half-plane only
                // constrains the HOLONOMIC target velocity for this step, not the non-holonomic-steered
                // commit above -- a thin wall / overlap-recovery burst can otherwise tunnel it in one
                // step. Clip the committed centre displacement c0->c1 at the first wall it would cross.
                var clipped = ClipToWalls(c0, c1);
                _position[i] = clipped;
                _velocity[i] = (clipped - c0) / dt;          // actual chord velocity (used by neighbours)
                _heading[i] = theta1;
            }
            else
            {
                var c0 = _position[i];
                _velocity[i] = _newVelocity[i];
                var c1 = c0 + _velocity[i] * dt;

                var clipped = ClipToWalls(c0, c1);
                _position[i] = clipped;
                _velocity[i] = (clipped - c0) / dt;
                if (_velocity[i].Abs > HeadingHoldSpeed)
                {
                    _heading[i] = Math.Atan2(_velocity[i].Y, _velocity[i].X);
                }
            }
        }

        _stepIndex++;
    }

    // Deactivate a vehicle explicitly (it parks and stops constraining others). Used by a scene to
    // despawn a mover that has left the field -- including the rare straggler that a dense jam pushes
    // backward out of the domain (the holonomic dense-packing limit noted in INDIA-TRAFFIC.md).
    public void Deactivate(int i)
    {
        _active[i] = false;
        _velocity[i] = Vec2.Zero;
    }

    public bool AllArrived(double epsilon)
    {
        var epsSq = epsilon * epsilon;
        for (var i = 0; i < _count; i++)
        {
            if (_active[i] && (_goal[i] - _position[i]).AbsSq > epsSq)
            {
                return false;
            }
        }

        return true;
    }

    private Vec2 Plan(int i, double dt)
    {
        var pos = _position[i];
        var maxSpeed = _maxSpeed[i];
        var selfShape = _solveProto[i].RotatedTo(_heading[i]);
        var self = new ShapedVoSolver.ShapedAgent(pos, _velocity[i], selfShape);

        // Preferred velocity: straight at the goal at maxSpeed, easing in near it, with an optional
        // deterministic symmetry-break jitter (hashed from agent index + step, like OrcaCrowd).
        var toGoal = _goal[i] - pos;
        var dist = toGoal.Abs;
        Vec2 pref;
        if (dist < 1e-9)
        {
            pref = Vec2.Zero;
        }
        else
        {
            var speed = Math.Min(maxSpeed, dist / dt);
            pref = toGoal / dist * speed;
            if (SymmetryBreak > 0.0)
            {
                var h = unchecked((uint)(i * 73856093) ^ (uint)(_stepIndex * 19349663));
                var a = h * (2.0 * Math.PI / 4294967296.0);
                pref += new Vec2(Math.Cos(a), Math.Sin(a)) * SymmetryBreak;
            }
        }

        var cap = _count + _walls.Count;
        if (_nbScratch.Length < cap)
        {
            Array.Resize(ref _nbScratch, cap);
            Array.Resize(ref _nbDistSq, cap);
        }

        if (_lineScratch.Length < cap)
        {
            Array.Resize(ref _lineScratch, cap);
        }

        var rangeSq = NeighbourDist * NeighbourDist;
        var maxN = MaxNeighbours;
        var assertI = _assert[i];

        // Vehicle neighbours: reciprocal but asymmetric (ego's responsibility for j is
        // assert_j / (assert_i + assert_j) -- ego yields more to a more assertive neighbour), gathered
        // through the SAME method for both the brute-force scan (useAll:true, iterate 0.._count) and
        // the spatial-hash path (useAll:false, iterate the sorted 3x3-cell candidates) -- so the two
        // are bit-identical by construction (PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2a).
        int k;
        if (UseSpatialHash)
        {
            var candidates = GridCandidates(i);
            k = GatherVehicleNeighbours(i, pos, assertI, rangeSq, maxN, 0, candidates, useAll: false);
        }
        else
        {
            k = GatherVehicleNeighbours(i, pos, assertI, rangeSq, maxN, 0, default, useAll: true);
        }

        // Walls: full-yield static shaped obstacles (responsibility 1.0). Range test is by centre
        // distance to the wall's reference point; walls are not subject to the nearest-k cap (append
        // after the capped vehicle set so confinement is never dropped in a dense jam).
        for (var w = 0; w < _walls.Count; w++)
        {
            var (centre, shape) = _walls[w];
            var dsq = (centre - pos).AbsSq;
            // A wall's footprint can be long; admit it if its bounding circle reaches the range.
            var reach = NeighbourDist + shape.CircumRadius();
            if (dsq > reach * reach)
            {
                continue;
            }

            if (k == _nbScratch.Length)
            {
                Array.Resize(ref _nbScratch, k * 2);
                Array.Resize(ref _nbDistSq, k * 2);
                Array.Resize(ref _lineScratch, k * 2);
            }

            _nbScratch[k] = new ShapedVoSolver.ShapedAgent(centre, Vec2.Zero, shape, responsibility: 1.0);
            _nbDistSq[k] = dsq;
            k++;
        }

        return ShapedVoSolver.ComputeNewVelocity(
            self, _nbScratch.AsSpan(0, k), pref, maxSpeed, TimeHorizon, dt, _lineScratch.AsSpan(0, k));
    }

    // Map the holonomic ORCA target velocity onto a car-like motion the vehicle can actually execute
    // this step, given its FROZEN start-of-step heading and speed (deterministic). Three limits:
    //   * no reversing        -- a target pointing behind the heading becomes braking, never reverse;
    //   * bounded turn        -- heading changes at most (speed*dt / MinTurnRadius) rad, so a slow or
    //                            stopped vehicle cannot pivot in place (a small crawl term keeps a
    //                            little steering authority so it can ease into a turn, not lock up);
    //   * bounded accel/brake -- speed changes within [MaxDecel, MaxAccel] * dt, clamped to [0, max].
    // The vehicle also slows for a sharp maneuver: the target speed is scaled by cos(headingError),
    // so a target 90 deg off the heading brakes toward a stop rather than sliding sideways.
    private Vec2 SteerNonholonomic(int i, Vec2 desired, double dt, out double newHeading)
    {
        var cls = _class[i];
        var theta = _heading[i];
        var curSpeed = _velocity[i].Abs;

        var desiredSpeed = desired.Abs;
        var desiredHeading = desiredSpeed > 1e-9 ? Math.Atan2(desired.Y, desired.X) : theta;
        var headingErr = Math.Atan2(Math.Sin(desiredHeading - theta), Math.Cos(desiredHeading - theta));

        // Target forward speed: brake for the turn (cos falloff), never negative (no reverse). A creep
        // floor keeps the vehicle rolling so it can still steer out of a jam -- but only up to what the
        // avoidance solve permits (min with desiredSpeed), so a genuinely blocked vehicle still stops.
        var alignment = Math.Max(0.0, Math.Cos(headingErr));
        var targetSpeed = Math.Max(Math.Min(CreepSpeed, desiredSpeed), desiredSpeed * alignment);

        // Longitudinal accel/brake limit, then the physical speed range.
        var newSpeed = Math.Clamp(targetSpeed, curSpeed - cls.MaxDecel * dt, curSpeed + cls.MaxAccel * dt);
        newSpeed = Math.Clamp(newSpeed, 0.0, cls.MaxSpeed);

        // Turn rate is bounded by the arc actually swept this step: heading can change at most
        // (distance travelled) / MinTurnRadius = newSpeed*dt / R. This ties steering strictly to
        // forward motion -- a stopped or crawling vehicle CANNOT rotate in place (no turntable spin);
        // it must roll forward to change heading, exactly like a real wheeled vehicle.
        var maxTurn = newSpeed * dt / cls.MinTurnRadius;
        var turn = Math.Clamp(headingErr, -maxTurn, maxTurn);
        newHeading = theta + turn;

        return new Vec2(Math.Cos(newHeading), Math.Sin(newHeading)) * newSpeed;
    }

    // Skin distance (m) the swept wall clip pulls the entry point back by, so the body stops just
    // short of the wall's true boundary on the interior side rather than resting exactly on it.
    private const double WallClipSkin = 0.05;

    // W1 (docs/MIXED-WALL-CONTAINMENT.md): clip the committed centre displacement c0->c1 at the
    // FIRST wall (in fixed list order) whose convex box the segment enters, pulled back by a small
    // skin. Deterministic: walls are tested in their fixed AddWall/AddBlock order and the EARLIEST
    // entry (smallest t) wins regardless of iteration order, so ties resolve the same way every run.
    // Returns c1 unchanged if the segment crosses no wall.
    private Vec2 ClipToWalls(Vec2 c0, Vec2 c1)
    {
        var d = c1 - c0;
        var lenSq = d.AbsSq;
        if (lenSq < 1e-18)
        {
            return c1;
        }

        var len = Math.Sqrt(lenSq);
        var bestT = double.PositiveInfinity;
        for (var w = 0; w < _walls.Count; w++)
        {
            var (centre, shape) = _walls[w];

            // Safe cull: if the closest point on the segment to the wall's centre is farther than the
            // wall's bounding-circle radius, the (centre-bounded) polygon cannot touch the segment --
            // no false negatives, just skips clearly-uninvolved walls.
            var reach = shape.CircumRadius();
            if (PointSegDistSq(centre, c0, d, lenSq) > reach * reach)
            {
                continue;
            }

            if (TrySegmentWallEntry(c0, d, centre, shape, out var t) && t < bestT)
            {
                bestT = t;
            }
        }

        if (double.IsPositiveInfinity(bestT))
        {
            return c1;
        }

        var pulledBackT = Math.Max(0.0, bestT - WallClipSkin / len);
        return c0 + d * pulledBackT;
    }

    // Squared distance from point p to the closest point on segment c0->(c0+d), d.AbsSq == dLenSq.
    private static double PointSegDistSq(Vec2 p, Vec2 c0, Vec2 d, double dLenSq)
    {
        var t = Vec2.Dot(p - c0, d) / Math.Max(dLenSq, 1e-18);
        t = Math.Clamp(t, 0.0, 1.0);
        var closest = c0 + d * t;
        return (p - closest).AbsSq;
    }

    // Cyrus-Beck segment-vs-convex-polygon clip: does the segment c0->c0+d enter the wall's convex
    // box (world verts = shape.Verts + centre)? Walks the polygon's edges as inward half-planes
    // (interior on the LEFT of each CCW edge, matching ConvexShape.ContainsPoint) and narrows
    // [tMin, tMax] = 0..1 to the sub-interval that satisfies every half-plane. tEnter (tMin on return)
    // is the earliest parameter at which the segment is inside ALL half-planes, i.e. inside the
    // polygon. Returns false if the segment never enters the polygon within [0, 1].
    private static bool TrySegmentWallEntry(Vec2 c0, Vec2 d, Vec2 centre, ConvexShape shape, out double tEnter)
    {
        var verts = shape.Verts;
        var n = verts.Length;
        var tMin = 0.0;
        var tMax = 1.0;
        for (var e = 0; e < n; e++)
        {
            var a = centre + verts[e];
            var b = centre + verts[(e + 1) % n];
            var edge = b - a;
            var inwardNormal = new Vec2(-edge.Y, edge.X);   // CCW edge -> interior on the left
            var numerator = Vec2.Dot(inwardNormal, a - c0);
            var denominator = Vec2.Dot(inwardNormal, d);

            if (Math.Abs(denominator) < 1e-12)
            {
                // Segment runs parallel to this edge: if c0 is already outside this half-plane, no t
                // satisfies it (the segment never enters the polygon).
                if (numerator > 1e-9)
                {
                    tEnter = 0.0;
                    return false;
                }

                continue;
            }

            var t = numerator / denominator;
            if (denominator > 0.0)
            {
                if (t > tMin)
                {
                    tMin = t;
                }
            }
            else if (t < tMax)
            {
                tMax = t;
            }

            if (tMin > tMax)
            {
                tEnter = 0.0;
                return false;
            }
        }

        tEnter = tMin;
        return true;
    }

    // The single vehicle-neighbour gather used by BOTH the brute-force and spatial-hash paths (so they
    // are bit-identical by construction, mirroring OrcaCrowd.GatherAgentNeighbours). Iterates either
    // every vehicle (useAll) or the caller's candidate indices (already sorted ascending), applying the
    // exact same self/active/range filter and the exact same nearest-k bounded Insert. Returns the
    // updated running count k.
    private int GatherVehicleNeighbours(
        int i, Vec2 pos, double assertI, double rangeSq, int maxN, int k, ReadOnlySpan<int> candidates, bool useAll)
    {
        var m = useAll ? _count : candidates.Length;
        for (var idx = 0; idx < m; idx++)
        {
            var j = useAll ? idx : candidates[idx];
            if (j == i || !_active[j])
            {
                continue;
            }

            var dsq = (_position[j] - pos).AbsSq;
            if (dsq > rangeSq)
            {
                continue;
            }

            var assertJ = _assert[j];
            var resp = assertJ / (assertI + assertJ);
            var agent = new ShapedVoSolver.ShapedAgent(
                _position[j], _velocity[j], _solveProto[j].RotatedTo(_heading[j]), resp);
            k = Insert(agent, dsq, k, maxN);
        }

        return k;
    }

    // Rebuild the uniform spatial hash from the frozen (post-removal) positions. Cell size ==
    // NeighbourDist so a vehicle's whole in-range neighbourhood is within its cell + the 8 around it.
    // Pooled per-bucket arrays are cleared (fill reset) and reused; only growth allocates. Mirrors
    // OrcaCrowd.RebuildGrid exactly (own copies, not shared).
    private void RebuildGrid()
    {
        _cellSize = NeighbourDist;
        _cellToBucket.Clear();
        _bucketCount = 0;
        for (var i = 0; i < _count; i++)
        {
            if (!_active[i])
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

    // Candidate vehicle indices for ego `i`: every vehicle in the 3x3 cell block around ego's cell,
    // SORTED ascending so the downstream gather matches brute-force order exactly. Reuses
    // _candidateScratch. Mirrors OrcaCrowd.GridCandidates exactly.
    private ReadOnlySpan<int> GridCandidates(int i)
    {
        if (_candidateScratch.Length < _count)
        {
            Array.Resize(ref _candidateScratch, _count);
        }

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
                        _candidateScratch[n++] = arr[t];
                    }
                }
            }
        }

        Array.Sort(_candidateScratch, 0, n);
        return _candidateScratch.AsSpan(0, n);
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

    // Nearest-k bounded insertion (sorted nearest-first), identical in spirit to OrcaCrowd's. maxN<=0
    // means unlimited (simple append). Returns the new count.
    private int Insert(ShapedVoSolver.ShapedAgent agent, double dsq, int k, int maxN)
    {
        if (maxN <= 0)
        {
            if (k == _nbScratch.Length)
            {
                Array.Resize(ref _nbScratch, k * 2);
                Array.Resize(ref _nbDistSq, k * 2);
                Array.Resize(ref _lineScratch, k * 2);
            }

            _nbScratch[k] = agent;
            _nbDistSq[k] = dsq;
            return k + 1;
        }

        if (k == maxN && dsq >= _nbDistSq[k - 1])
        {
            return k;   // full and no closer than the current farthest kept
        }

        var pos = k < maxN ? k : k - 1;
        while (pos > 0 && _nbDistSq[pos - 1] > dsq)
        {
            _nbScratch[pos] = _nbScratch[pos - 1];
            _nbDistSq[pos] = _nbDistSq[pos - 1];
            pos--;
        }

        _nbScratch[pos] = agent;
        _nbDistSq[pos] = dsq;
        return k < maxN ? k + 1 : k;
    }

    private void Grow(int newCapacity)
    {
        Array.Resize(ref _position, newCapacity);
        Array.Resize(ref _velocity, newCapacity);
        Array.Resize(ref _goal, newCapacity);
        Array.Resize(ref _heading, newCapacity);
        Array.Resize(ref _maxSpeed, newCapacity);
        Array.Resize(ref _assert, newCapacity);
        Array.Resize(ref _class, newCapacity);
        Array.Resize(ref _shapeProto, newCapacity);
        Array.Resize(ref _solveProto, newCapacity);
        Array.Resize(ref _active, newCapacity);
        Array.Resize(ref _newVelocity, newCapacity);
        Array.Resize(ref _newHeading, newCapacity);
    }
}
