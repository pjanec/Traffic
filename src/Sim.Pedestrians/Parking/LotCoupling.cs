using Sim.Core.Bridge;
using Sim.Core.Mixed;
using Sim.Core.Orca;
using Sim.Pedestrians.Obstacles;

namespace Sim.Pedestrians.Parking;

// POC-6b (docs/PEDESTRIAN-POC-PLAN.md POC-6, success condition 3; docs/PEDESTRIAN-DESIGN.md §6 "Parking
// lots ... NEW assembly of existing parts" and §2): the free-space CAR<->PEDESTRIAN mutual-avoidance
// bridge inside a parking lot -- one or more `MixedTrafficCrowd` maneuvering cars and an `OrcaCrowd` of
// pedestrians, stepped in lockstep with BIDIRECTIONAL disc exchange, plus static parked-car boxes both
// populations avoid. Completes POC-6 (conditions 1/2 are POC-6a's `ParkingController`/
// `PersonRideController`; this is condition 3, "peds crossing the inner drive lane are yielded to by a
// maneuvering car").
//
// ===== Direction A: pedestrians avoid the car(s) (BUILT, straightforward) =====
// Each step every maneuvering car is turned into a short CHAIN OF DISCS along its oriented footprint
// spine (front-to-back, at the car's true half-width) and fed to `OrcaCrowd.SetExternalObstacles` --
// EXACTLY the `EvacDirector.FeedVehicleDiscsToPeds` / `CrossRegimeCoupling.BuildVehicleDiscs` pattern.
// A single bounding disc would either overshrink a long car's flanks or overshoot its ends; a chain
// (count derived from Length/HalfWidth, capped, mirroring CrossRegimeCoupling's own formula) tracks the
// car's rectangle tightly enough that a pedestrian's ORCA genuinely clears the car's real footprint, not
// a padded circle.
//
// ===== Direction B: the car(s) avoid the pedestrians (NEW -- documented API friction) =====
// `Sim.Core.Mixed.MixedTrafficCrowd` was checked for a `SetExternalObstacles`-equivalent (the task's
// starting assumption) and has NONE: its only obstacle inputs are `AddWall`/`AddBlock`, both PERMANENT
// (no removal, no update), and `Add()` only ever appends a fully self-driven vehicle -- there is no way
// to hand it a per-step external moving disc, nor to override a vehicle's position from outside once
// added. (`Sim.Core.Unified.UnifiedWorld` and `Sim.Core.Bridge.CrossRegimeCoupling` both coordinate an
// `OrcaCrowd` against a DIFFERENT vehicle model -- a hand-rolled straight-lane follower and the lane
// `Engine`, respectively -- neither touches `MixedTrafficCrowd`, so there is no existing precedent to
// copy either.) Given the hard constraint against touching Core, this class instead REBUILDS the
// underlying `MixedTrafficCrowd` every step -- exactly the "no add/remove -> rebuild carrying survivors
// forward" idiom this codebase already uses for the identical class of problem (POC-3's
// `PedLodManager.RebuildHighCrowd`; POC-6a's `PersonRideController.RebuildCrowdExcluding`) -- and reuses
// the ALREADY-established "represent a small disc-ish obstacle as an `AddBlock` square" idiom
// (`EvacDirector.DriveOrcaPushers`'s wrecked-pusher obstacle) to turn each pedestrian into a per-step
// static box:
//   1. a FRESH `MixedTrafficCrowd` is constructed;
//   2. the permanent parked-car boxes are replayed as walls (their 4 edges via `AddWall`, so an
//      arbitrary orientation is honoured, not just axis-aligned `AddBlock`);
//   3. EACH pedestrian's CURRENT (frozen, start-of-step -- the ped `Step` has not run yet this tick) is
//      added as a small AABB via `AddBlock` (a full-yield, zero-velocity static obstacle);
//   4. every car is re-`Add`ed at its EXACT carried-forward position/velocity/heading/goal;
//   5. `Step(dt)` runs once and the committed position/velocity/heading are read back.
// This is a ONE-SIDED, one-step-latency yield (the car treats peds as static for the step it plans
// against) -- collision-safe, and consistent with the codebase's own precedent that a one-sided
// conservative yield is an acceptable half of a mutual double-yield
// (`CrossRegimeCoupling`'s own doc comment: "Both sides yield fully ... always collision-safe"). The
// non-holonomic solve's own `SafetyMargin`-inflated solve shape (Plan()) keeps the car's TRUE footprint
// comfortably clear of the ped's box before `ClipToWalls`'s point-swept backstop would ever matter.
//
// Exchange discipline: BOTH directions read the OTHER side's snapshot as of the START of the tick (cars'
// pre-step state feeds the peds; peds' pre-step state feeds the rebuilt car crowd; ped `Step` runs last)
// -- a SYMMETRIC one-step-latency lockstep, mirroring `EvacDirector`/`CrossRegimeCoupling` exactly.
//
// Determinism: no `System.Random`; cars are iterated in ascending CarId (`AddCar` call order),
// pedestrians in ascending crowd index (`OrcaCrowd.Add` call order), parked boxes in `AddParkedCarBox`
// call order -- every rebuild replays these in the SAME fixed order, so the rebuilt crowd's internal
// index assignment (hence every LP, hence every position) is a pure function of frozen state + that
// order, independent of thread/dictionary iteration.
public sealed class LotCoupling
{
    // ----- car-side tuning (mirrors ParkingController / VehicleMover's Orca-push setup) -----
    public double CarSafetyMargin { get; set; } = 0.5;
    public double CarTimeHorizon { get; set; } = 3.0;
    public int CarMaxNeighbours { get; set; } = 8;

    // A small creep floor (matches VehicleMover/EvacDirector's Orca-push setup, and MixedTrafficCrowd's
    // own doc comment: "scenes set a small value (~0.8) to keep dense junctions flowing"). Without it a
    // vehicle whose steering target sits ~90 deg off its current heading gets a target speed of exactly
    // zero (SteerNonholonomic) and deadlocks -- exactly what a large pedestrian-avoidance detour (see
    // StepCars' maxCarReach inflation) can demand of a car that must swerve hard around a crosser.
    public double CarCreepSpeed { get; set; } = 0.8;

    // Cap on the disc-chain length used to represent a car to the pedestrian crowd (Direction A).
    public int MaxDiscsPerVehicle { get; set; } = 6;

    // Extra half-extent margin (m) added around a pedestrian's radius when it becomes a static AABB
    // obstacle for the car-avoids-ped direction (Direction B) -- gives the rebuild's wall-vs-solve-shape
    // gap some headroom beyond the raw disc radius.
    public double PedObstacleMargin { get; set; } = 0.3;

    // Wall thickness (m) for the replayed parked-car box edges and the per-step pedestrian AABB edges
    // in the rebuilt car crowd.
    public double ParkedBoxWallThickness { get; set; } = 0.2;
    public double PedObstacleWallThickness { get; set; } = 0.15;

    // Distance to goal at which a car ARRIVES and holds (frozen position, zero velocity, excluded from
    // the next rebuild -- mirrors ParkingController's own `_arriveRadius` / `MixedTrafficCrowd.Deactivate`
    // convention for "reached the slot"). Mandatory, not optional: a non-holonomic vehicle that can never
    // reverse and is never told to stop will fly past a point goal and re-approach from the far side
    // forever (a classic non-holonomic pursuit pathology, confirmed empirically while tuning this class --
    // without this, a car orbits its goal indefinitely instead of settling).
    public double CarArriveRadius { get; set; } = 1.0;

    private readonly OrcaCrowd _peds;
    private readonly List<ParkedBox> _parkedBoxes = new();
    private readonly List<CarState> _cars = new();
    private WorldDisc[] _carDiscScratch = new WorldDisc[16];
    private double _time;

    private readonly record struct ParkedBox(Vec2 Center, double HalfLength, double HalfWidth, double AngleRad)
    {
        public Vec2[] Corners() => BoxObstacle.Corners(Center, HalfLength, HalfWidth, AngleRad);
    }

    private sealed class CarState
    {
        public required int CarId;
        public required VehicleClass Class;
        public Vec2 Position;
        public Vec2 Velocity;
        public double Heading;
        public Vec2 Goal;
        public double MaxSpeed;
        public bool Arrived;
    }

    public LotCoupling(OrcaCrowd? peds = null)
    {
        _peds = peds ?? new OrcaCrowd();
    }

    public OrcaCrowd Peds => _peds;
    public double Time => _time;
    public int CarCount => _cars.Count;

    // ----- setup -----

    // A static parked-car footprint BOTH populations must avoid: added once (permanently) to the
    // pedestrian `OrcaCrowd` via `BoxObstacle.Corners` -> `AddObstacle`, and replayed (its 4 edges, via
    // `AddWall`) into the maneuvering car's crowd on every rebuild (see `StepCars`).
    public void AddParkedCarBox(Vec2 center, double halfLength, double halfWidth, double angleRad = 0.0)
    {
        var box = new ParkedBox(center, halfLength, halfWidth, angleRad);
        _parkedBoxes.Add(box);
        _peds.AddObstacle(box.Corners());
    }

    public IReadOnlyList<Vec2[]> ParkedCarFootprints
    {
        get
        {
            var list = new List<Vec2[]>(_parkedBoxes.Count);
            foreach (var b in _parkedBoxes)
            {
                list.Add(b.Corners());
            }

            return list;
        }
    }

    // Register a maneuvering car; returns its stable CarId (== ascending AddCar call order, matching the
    // `_cars` index -- append-only, never remapped).
    public int AddCar(
        Vec2 position, Vec2 goal, double maxSpeed, double? headingRad = null, VehicleClass? vehicleClass = null)
    {
        var cls = vehicleClass ?? VehicleClass.Car;
        var toGoal = goal - position;
        var heading = headingRad ?? (toGoal.AbsSq > 1e-9 ? Math.Atan2(toGoal.Y, toGoal.X) : 0.0);
        var id = _cars.Count;
        _cars.Add(new CarState
        {
            CarId = id,
            Class = cls,
            Position = position,
            Velocity = Vec2.Zero,
            Heading = heading,
            Goal = goal,
            MaxSpeed = maxSpeed,
        });
        return id;
    }

    public OrcaHandle AddPedestrian(Vec2 position, double radius, double maxSpeed, Vec2 goal) =>
        _peds.Add(position, radius, maxSpeed, goal);

    // Setting a new goal re-arms an ARRIVED car (a caller-driven "depart" -- not exercised by the
    // condition-3 tests, but keeps the state machine consistent rather than permanently wedging a car
    // that already parked).
    public void SetCarGoal(int carId, Vec2 goal)
    {
        var c = _cars[carId];
        c.Goal = goal;
        c.Arrived = false;
    }

    public void SetPedGoal(OrcaHandle pedIndex, Vec2 goal) => _peds.SetGoal(pedIndex, goal);

    // ----- observability (tests + viz) -----

    public Vec2 CarPosition(int carId) => _cars[carId].Position;
    public Vec2 CarVelocity(int carId) => _cars[carId].Velocity;
    public double CarSpeed(int carId) => _cars[carId].Velocity.Abs;
    public double CarHeading(int carId) => _cars[carId].Heading;
    public bool CarArrived(int carId) => _cars[carId].Arrived;
    public Vec2 CarGoal(int carId) => _cars[carId].Goal;
    public VehicleClass CarClass(int carId) => _cars[carId].Class;

    // The car's TRUE oriented-box footprint (its real Length x Width, NOT the SafetyMargin-inflated
    // solve shape) -- for exact ped-vs-car overlap tests via `BoxObstacle.NearestPoint`/`Contains`.
    public Vec2[] CarFootprint(int carId)
    {
        var c = _cars[carId];
        return BoxObstacle.Corners(c.Position, c.Class.Length * 0.5, c.Class.Width * 0.5, c.Heading);
    }

    public int PedCount => _peds.Count;
    public Vec2 PedPosition(OrcaHandle i) => _peds.Position(i);
    public double PedRadius(OrcaHandle i) => _peds.Radius(i);
    public Vec2 PedGoal(OrcaHandle i) => _peds.Goal(i);

    // Nearest distance from `point` to an oriented-box footprint (thin convenience wrapper around
    // `BoxObstacle.NearestPoint`, used by both `CarFootprint` overlap checks and `ParkedCarFootprints`
    // overlap checks).
    public static double DistanceToBox(Vec2 point, IReadOnlyList<Vec2> corners) =>
        (point - BoxObstacle.NearestPoint(corners, point)).Abs;

    // ----- the coupled step -----

    // Advance BOTH populations one lockstep tick, exchanging FROZEN start-of-step snapshots in both
    // directions (see class remarks). Order: feed the cars' pre-step discs to the peds, then rebuild-
    // and-step the car crowd against the peds' still-pre-step positions, then step the peds against the
    // discs fed above. Both sides therefore react to the OTHER side's state as of the START of this
    // tick -- symmetric, order-independent, matching `EvacDirector`/`CrossRegimeCoupling`.
    public void Step(double dt)
    {
        _time += dt;
        FeedCarDiscsToPeds();
        StepCars(dt);
        _peds.Step(dt);
    }

    private void FeedCarDiscsToPeds()
    {
        var needed = _cars.Count * Math.Max(1, MaxDiscsPerVehicle);
        if (_carDiscScratch.Length < needed)
        {
            _carDiscScratch = new WorldDisc[Math.Max(needed, 16)];
        }

        var n = 0;
        foreach (var c in _cars)
        {
            var halfLen = c.Class.Length * 0.5;
            var halfWidth = c.Class.Width * 0.5;
            var hx = Math.Cos(c.Heading);
            var hy = Math.Sin(c.Heading);

            // Disc count from Length/HalfWidth (mirrors CrossRegimeCoupling.BuildVehicleDiscs' formula
            // exactly), capped -- a chain tight enough to track a rectangle without exploding neighbour
            // count for a long vehicle.
            var count = Math.Clamp((int)Math.Ceiling(c.Class.Length / halfWidth), 1, MaxDiscsPerVehicle);
            var spacing = count > 1 ? (2.0 * halfLen) / (count - 1) : 0.0;
            for (var d = 0; d < count; d++)
            {
                var back = -halfLen + (d * spacing);   // -halfLen .. +halfLen along heading, centred on Position
                _carDiscScratch[n++] = new WorldDisc(
                    c.Position.X + (hx * back), c.Position.Y + (hy * back), c.Velocity.X, c.Velocity.Y, halfWidth);
            }
        }

        _peds.SetExternalObstacles(_carDiscScratch.AsSpan(0, n));
    }

    private void StepCars(double dt)
    {
        if (_cars.Count == 0)
        {
            return;
        }

        // See class remarks: a FRESH MixedTrafficCrowd every step is the only way to inject dynamic
        // (pedestrian-position-tracking) obstacles given MixedTrafficCrowd's append-only/no-removal
        // public surface.
        var crowd = new MixedTrafficCrowd
        {
            Nonholonomic = true,
            SafetyMargin = CarSafetyMargin,
            TimeHorizon = CarTimeHorizon,
            MaxNeighbours = CarMaxNeighbours,
            CreepSpeed = CarCreepSpeed,
        };

        // Permanent parked-car boxes, replayed in AddParkedCarBox order every rebuild.
        foreach (var box in _parkedBoxes)
        {
            var corners = box.Corners();
            for (var e = 0; e < 4; e++)
            {
                crowd.AddWall(corners[e], corners[(e + 1) % 4], ParkedBoxWallThickness);
            }
        }

        // Each pedestrian's CURRENT (pre-step) position becomes a small static AABB this step, in
        // ascending crowd-index order. INFLATION, beyond the raw ped radius + PedObstacleMargin, must
        // cover `maxCarReach` -- see remarks below: MixedTrafficCrowd.ClipToWalls (the wall-containment
        // backstop; Core, unmodifiable) clips only the vehicle's CENTRE point against a wall polygon, not
        // its true oriented body. Under normal operation ShapedVoSolver.Plan already keeps the car's
        // SafetyMargin-inflated solve SHAPE clear of the wall, so ClipToWalls's point check never binds;
        // but a sharp non-holonomic maneuver (turn-rate-limited SteerNonholonomic tracking error) can
        // leave Plan's chosen velocity un-achieved for a step, at which point ClipToWalls is the ONLY
        // thing stopping the car, and it only stops the CENTRE -- letting the car's real body (up to its
        // footprint's circumscribing radius from centre) swing into whatever the centre just missed.
        // Measured without this term: a crossing pedestrian's true separation from the car's true
        // footprint went slightly NEGATIVE (a ~4 cm body-corner clip) in exactly this "car mid-maneuver,
        // pedestrian crossing" situation. Compensating by the worst-case (orientation-agnostic) body
        // reach at the LotCoupling layer -- the only layer this class is allowed to touch -- restores a
        // real guarantee without assuming anything about the car's instantaneous heading.
        var maxCarReach = 0.0;
        foreach (var c in _cars)
        {
            var reach = c.Class.Shape().CircumRadius();
            if (reach > maxCarReach)
            {
                maxCarReach = reach;
            }
        }

        var pedCount = _peds.Count;
        for (var p = 0; p < pedCount; p++)
        {
            var pos = _peds.Position(p);
            var r = _peds.Radius(p) + PedObstacleMargin + maxCarReach;
            crowd.AddBlock(pos.X - r, pos.Y - r, pos.X + r, pos.Y + r, PedObstacleWallThickness);
        }

        // Cars, in ascending CarId order, carrying their exact prior state forward. A car that has
        // ARRIVED (within CarArriveRadius of its goal, checked against the FROZEN pre-step position) is
        // held out of the rebuild entirely -- frozen position, zero velocity -- exactly like
        // ParkingController's Deactivate-on-arrival (see CarArriveRadius remarks: without this, a
        // non-holonomic vehicle that can never reverse and is never told "you're done" orbits a point
        // goal forever instead of settling).
        var indices = new int[_cars.Count];
        for (var i = 0; i < _cars.Count; i++)
        {
            var c = _cars[i];
            if (!c.Arrived && (c.Goal - c.Position).Abs <= CarArriveRadius)
            {
                c.Arrived = true;
                c.Velocity = Vec2.Zero;
            }

            indices[i] = c.Arrived ? -1 : crowd.Add(c.Class, c.Position, c.Goal, c.Heading, c.Velocity, c.MaxSpeed);
        }

        crowd.Step(dt);

        for (var i = 0; i < _cars.Count; i++)
        {
            var idx = indices[i];
            if (idx < 0)
            {
                continue;   // arrived this rebuild or earlier: stays frozen exactly where it stopped
            }

            var c = _cars[i];
            c.Position = crowd.Position(idx);
            c.Velocity = crowd.Velocity(idx);
            c.Heading = crowd.Heading(idx);
        }
    }
}
