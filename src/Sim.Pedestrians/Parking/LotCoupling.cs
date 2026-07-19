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
// ===== Direction A: pedestrians avoid the car(s) (unchanged since POC-6b) =====
// Each step every maneuvering car is turned into a short CHAIN OF DISCS along its oriented footprint
// spine (front-to-back, at the car's true half-width) and fed to `OrcaCrowd.SetExternalObstacles` --
// EXACTLY the `EvacDirector.FeedVehicleDiscsToPeds` / `CrossRegimeCoupling.BuildVehicleDiscs` pattern.
// A single bounding disc would either overshrink a long car's flanks or overshoot its ends; a chain
// (count derived from Length/HalfWidth, capped, mirroring CrossRegimeCoupling's own formula) tracks the
// car's rectangle tightly enough that a pedestrian's ORCA genuinely clears the car's real footprint, not
// a padded circle.
//
// ===== Direction B: the car(s) avoid the pedestrians (P0-3: persistent crowd, no per-step rebuild) =====
// POC-6b found `MixedTrafficCrowd` had only permanent `AddWall`/`AddBlock` and no way to hand it a
// per-step external moving disc, so this class used to REBUILD the underlying `MixedTrafficCrowd` from
// scratch every step (replaying parked-box walls, turning each pedestrian's frozen position into a
// static `AddBlock`, and re-`Add`ing every car at its carried-forward state) -- the same
// no-add/remove-so-rebuild idiom `PedLodManager.RebuildHighCrowd` used before P0-3, and exactly the cost
// P0-3 (docs/PEDESTRIAN-TASKS.md) retires. P0-2 added `MixedTrafficCrowd.Add`/`Remove` (a real O(1)
// handle-based store, mirroring P0-1's `OrcaCrowd`) and `SetExternalObstacles` (a velocity-aware moving
// disc input, mirroring `OrcaCrowd.SetExternalObstacles`), so this class now:
//   1. adds the permanent parked-car boxes as walls ONCE, in `AddParkedCarBox` (not replayed per step);
//   2. `Add`s each car ONCE, in `AddCar`, keeping its `MixedTrafficHandle` (`CarState.Handle`); a car is
//      `Remove`d from the crowd only when it ARRIVES (frozen thereafter, see `CarArriveRadius` remarks)
//      and re-`Add`ed only if a caller re-arms it via `SetCarGoal` -- Add/Remove on lot entry/exit, not
//      every tick;
//   3. every step, `StepCars` feeds the pedestrians' CURRENT (frozen, start-of-step) positions AND
//      VELOCITIES to the persistent car crowd via `SetExternalObstacles` -- the SAME mechanism Direction
//      A already used for cars-avoiding-peds, now used symmetrically -- instead of approximating each
//      ped as a momentary zero-velocity static box. This is a strict accuracy improvement, not just a
//      speed one: a car can react to a ped's velocity (e.g. someone crossing FAST is treated differently
//      from someone dawdling) instead of only ever seeing "a wall happens to be here right now".
// The old `AddBlock`-specific `maxCarReach` inflation (compensating for `MixedTrafficCrowd.ClipToWalls`
// clipping only a vehicle's CENTRE point against a WALL polygon) no longer applies -- pedestrians are no
// longer walls, they are moving neighbours in the SAME shaped-VO avoidance solve Direction A already
// relies on, with its own already-proven no-overlap guarantee (P0-2's own tests). `PedObstacleMargin`
// still exists as an optional extra clearance added to a pedestrian's true radius before it becomes a
// disc.
//
// Exchange discipline: BOTH directions read the OTHER side's snapshot as of the START of the tick (cars'
// pre-step state feeds the peds; peds' pre-step state feeds the car crowd; ped `Step` runs last) -- a
// SYMMETRIC one-step-latency lockstep, mirroring `EvacDirector`/`CrossRegimeCoupling` exactly.
//
// Determinism: no `System.Random`; cars are iterated in ascending CarId (`AddCar` call order),
// pedestrians in ascending crowd index (`OrcaCrowd.Add` call order), parked boxes in `AddParkedCarBox`
// call order. The car crowd is now persistent (P0-3), so a car's `MixedTrafficHandle` -- like an
// `OrcaHandle` -- stays valid and stable across steps instead of being reassigned by a fresh rebuild;
// iteration is still the fixed ascending `_cars` order, so every position is still a pure function of
// frozen state + that order, independent of thread/dictionary iteration.
public sealed class LotCoupling
{
    // ----- car-side tuning (mirrors ParkingController / VehicleMover's Orca-push setup) -----
    public double CarSafetyMargin { get; set; } = 0.5;
    public double CarTimeHorizon { get; set; } = 3.0;
    public int CarMaxNeighbours { get; set; } = 8;

    // A small creep floor (matches VehicleMover/EvacDirector's Orca-push setup, and MixedTrafficCrowd's
    // own doc comment: "scenes set a small value (~0.8) to keep dense junctions flowing"). Without it a
    // vehicle whose steering target sits ~90 deg off its current heading gets a target speed of exactly
    // zero (SteerNonholonomic) and deadlocks -- exactly what a large pedestrian-avoidance detour can
    // demand of a car that must swerve hard around a crosser.
    public double CarCreepSpeed { get; set; } = 0.8;

    // Cap on the disc-chain length used to represent a car to the pedestrian crowd (Direction A).
    public int MaxDiscsPerVehicle { get; set; } = 6;

    // Extra half-extent margin (m) added around a pedestrian's radius when it becomes an external disc
    // for the car-avoids-ped direction (Direction B) -- optional extra clearance beyond the raw disc
    // radius (see class remarks: P0-3 dropped the old AddBlock-era `maxCarReach` wall-clip compensation,
    // which does not apply to the disc-based avoidance path).
    public double PedObstacleMargin { get; set; } = 0.3;

    // Wall thickness (m) for the parked-car box edges in the car crowd (added once, see AddParkedCarBox).
    public double ParkedBoxWallThickness { get; set; } = 0.2;

    // Distance to goal at which a car ARRIVES and holds (frozen position, zero velocity, Removed from
    // the car crowd -- mirrors ParkingController's own `_arriveRadius` / `MixedTrafficCrowd.Deactivate`
    // convention for "reached the slot"). Mandatory, not optional: a non-holonomic vehicle that can never
    // reverse and is never told to stop will fly past a point goal and re-approach from the far side
    // forever (a classic non-holonomic pursuit pathology, confirmed empirically while tuning this class --
    // without this, a car orbits its goal indefinitely instead of settling).
    public double CarArriveRadius { get; set; } = 1.0;

    private readonly OrcaCrowd _peds;
    private readonly MixedTrafficCrowd _carCrowd;
    private readonly List<ParkedBox> _parkedBoxes = new();
    private readonly List<CarState> _cars = new();
    private WorldDisc[] _carDiscScratch = new WorldDisc[16];   // Direction A: car discs fed to the peds
    private WorldDisc[] _pedDiscScratch = new WorldDisc[16];   // Direction B: ped discs fed to the cars
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

        // P0-3: this car's stable handle into the persistent `_carCrowd`, or Invalid while Arrived (the
        // car has been Removed from the crowd and is held out until re-armed via SetCarGoal).
        public MixedTrafficHandle Handle;
    }

    public LotCoupling(OrcaCrowd? peds = null)
    {
        // P0-4 (docs/PEDESTRIAN-POC7C-FINDINGS.md follow-up; docs/PEDESTRIAN-DESIGN.md §9): both
        // crowds here were constructed bare (spatial hash off -> brute-force O(n^2) neighbour gather).
        // UseSpatialHash is a proven bit-identical pre-filter on both OrcaCrowd (OrcaSpatialHashTests)
        // and MixedTrafficCrowd (mirrors the same pattern) -- turning it on changes only candidate
        // discovery, never the neighbour SET or its order, so every trajectory is unchanged. The ped
        // crowd also carries static parked-car-box obstacles (AddParkedCarBox -> _peds.AddObstacle), so
        // UseObstacleSpatialIndex (P2-1, also bit-identical) is turned on for it too -- there is a real
        // static-obstacle index for it to accelerate, unlike PedLodManager's high crowd.
        //
        // NOTE: a caller-supplied `peds` (the `peds` parameter) is used AS-IS and not mutated here --
        // only the crowd THIS constructor creates gets the flag, matching "configure what you own, not
        // what a caller handed you" convention elsewhere in this codebase.
        _peds = peds ?? new OrcaCrowd { UseSpatialHash = true, UseObstacleSpatialIndex = true };
        _carCrowd = new MixedTrafficCrowd
        {
            Nonholonomic = true,
            SafetyMargin = CarSafetyMargin,
            TimeHorizon = CarTimeHorizon,
            MaxNeighbours = CarMaxNeighbours,
            CreepSpeed = CarCreepSpeed,
            UseSpatialHash = true,
        };
    }

    public OrcaCrowd Peds => _peds;
    public double Time => _time;
    public int CarCount => _cars.Count;

    // ----- setup -----

    // A static parked-car footprint BOTH populations must avoid: added once (permanently) to the
    // pedestrian `OrcaCrowd` via `BoxObstacle.Corners` -> `AddObstacle`, and (P0-3) ONCE to the
    // persistent car crowd via its 4 edges (`AddWall`, so an arbitrary orientation is honoured) --
    // no longer replayed on every step (see class remarks).
    public void AddParkedCarBox(Vec2 center, double halfLength, double halfWidth, double angleRad = 0.0)
    {
        var box = new ParkedBox(center, halfLength, halfWidth, angleRad);
        _parkedBoxes.Add(box);
        _peds.AddObstacle(box.Corners());

        var corners = box.Corners();
        for (var e = 0; e < 4; e++)
        {
            _carCrowd.AddWall(corners[e], corners[(e + 1) % 4], ParkedBoxWallThickness);
        }
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
    // `_cars` index -- append-only, never remapped). P0-3: also `Add`s it to the persistent car crowd
    // immediately, keeping the returned `MixedTrafficHandle` (not just appending to `_cars` and letting a
    // future per-step rebuild pick it up).
    public int AddCar(
        Vec2 position, Vec2 goal, double maxSpeed, double? headingRad = null, VehicleClass? vehicleClass = null)
    {
        var cls = vehicleClass ?? VehicleClass.Car;
        var toGoal = goal - position;
        var heading = headingRad ?? (toGoal.AbsSq > 1e-9 ? Math.Atan2(toGoal.Y, toGoal.X) : 0.0);
        var id = _cars.Count;
        var handle = _carCrowd.Add(cls, position, goal, heading, Vec2.Zero, maxSpeed);
        _cars.Add(new CarState
        {
            CarId = id,
            Class = cls,
            Position = position,
            Velocity = Vec2.Zero,
            Heading = heading,
            Goal = goal,
            MaxSpeed = maxSpeed,
            Handle = handle,
        });
        return id;
    }

    public OrcaHandle AddPedestrian(Vec2 position, double radius, double maxSpeed, Vec2 goal) =>
        _peds.Add(position, radius, maxSpeed, goal);

    // Setting a new goal re-arms an ARRIVED car (a caller-driven "depart" -- not exercised by the
    // condition-3 tests, but keeps the state machine consistent rather than permanently wedging a car
    // that already parked). P0-3: an arrived car was `Remove`d from the persistent crowd, so re-arming
    // must `Add` it back (Add/Remove on lot entry/exit, per the class remarks); a still-live car just
    // gets its crowd goal updated in place.
    public void SetCarGoal(int carId, Vec2 goal)
    {
        var c = _cars[carId];
        c.Goal = goal;
        if (c.Arrived)
        {
            c.Arrived = false;
            c.Handle = _carCrowd.Add(c.Class, c.Position, goal, c.Heading, c.Velocity, c.MaxSpeed);
        }
        else
        {
            _carCrowd.SetGoal(c.Handle, goal);
        }
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
    // directions (see class remarks). Order: feed the cars' pre-step discs to the peds, then step the
    // persistent car crowd against the peds' still-pre-step positions/velocities, then step the peds
    // against the discs fed above. Both sides therefore react to the OTHER side's state as of the START
    // of this tick -- symmetric, order-independent, matching `EvacDirector`/`CrossRegimeCoupling`.
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

        // Retire any car that has ARRIVED (within CarArriveRadius of its goal, checked against its
        // FROZEN pre-step position) from the persistent crowd -- P0-3's Add/Remove replaces the old
        // "exclude from this step's rebuild" idiom: frozen position, zero velocity, no longer
        // constrains or is constrained by anyone (see CarArriveRadius remarks: without this, a
        // non-holonomic vehicle that can never reverse and is never told "you're done" orbits a point
        // goal forever instead of settling).
        foreach (var c in _cars)
        {
            if (!c.Arrived && (c.Goal - c.Position).Abs <= CarArriveRadius)
            {
                c.Arrived = true;
                c.Velocity = Vec2.Zero;
                _carCrowd.Remove(c.Handle);
                c.Handle = MixedTrafficHandle.Invalid;
            }
        }

        // Each pedestrian's CURRENT (frozen, start-of-step -- the ped Step has not run yet this tick)
        // position AND velocity becomes a moving external disc this step (P0-2's SetExternalObstacles,
        // Direction B) -- velocity-aware, no rebuild (see class remarks for why this replaces the old
        // per-step AddBlock-as-static-box approximation). PedObstacleMargin is an optional extra
        // clearance beyond the ped's true radius.
        var pedCount = _peds.Count;
        if (_pedDiscScratch.Length < pedCount)
        {
            _pedDiscScratch = new WorldDisc[Math.Max(pedCount, 16)];
        }

        for (var p = 0; p < pedCount; p++)
        {
            var pos = _peds.Position(p);
            var vel = _peds.Velocity(p);
            var r = _peds.Radius(p) + PedObstacleMargin;
            _pedDiscScratch[p] = new WorldDisc(pos.X, pos.Y, vel.X, vel.Y, r);
        }

        _carCrowd.SetExternalObstacles(_pedDiscScratch.AsSpan(0, pedCount));
        _carCrowd.Step(dt);

        // Read back committed state for every still-live (non-arrived) car, in ascending CarId order.
        foreach (var c in _cars)
        {
            if (c.Arrived)
            {
                continue;   // stays frozen exactly where it stopped
            }

            c.Position = _carCrowd.Position(c.Handle);
            c.Velocity = _carCrowd.Velocity(c.Handle);
            c.Heading = _carCrowd.Heading(c.Handle);
        }
    }
}
