using Sim.Core.Orca;

namespace Sim.Pedestrians.Parking;

// POC-6a (docs/PEDESTRIAN-POC-PLAN.md POC-6, success condition 2; docs/PEDESTRIAN-DESIGN.md §2 "The
// unified regime/lifecycle state machine"): the PERSON side. A person is `Walking` (a live `OrcaCrowd`
// agent) or `Riding` (despawned from the crowd, conceptually inside a vehicle -- not simulated as a
// ped at all, per §2's "`Walking -> Riding` (board): ... remove it from the OrcaCrowd").
//
// Removal choice (§2 explicitly leaves this open -- "OrcaCrowd has no public remove; use the same
// 'rebuild the crowd carrying survivors' approach POC-3's PedLodManager used, OR mark it arrived/
// parked so it stops constraining -- document your choice"): this controller REBUILDS the crowd,
// carrying every surviving Walking agent's position/velocity/goal/radius/maxSpeed forward into a fresh
// OrcaCrowd, exactly the technique src/Sim.Pedestrians/Lod/PedLodManager.cs:257 (RebuildHighCrowd)
// already uses and already proved deterministic in POC-3. Rationale over "mark inactive": a boarded
// person is not merely done moving, it VANISHES (a real despawn -- the whole point of §2's "the IG
// shows it vanish next to the car" framing), and OrcaCrowd's own `IsActive`/`RemoveOnArrival` still
// counts an inactive agent when sizing/iterating the store (Count includes it) -- rebuilding is the
// only way to make a boarded person genuinely absent from the crowd's read surface (Count, Position
// iteration), which is exactly what the POC-6 success condition asks the test to observe. The
// trade-off (documented in PedLodManager's own remarks, unchanged here): O(surviving count) per
// boarding, and any obstacles/walls added directly to a prior crowd instance are NOT carried into the
// rebuilt one -- acceptable at POC scale; production needs real crowd add/remove (POC-3/§3d backlog).
//
// Determinism: no System.Random; people are iterated in ascending PersonId (Track()/AddWalking() call
// order); boarding candidates within one Step are resolved in that same fixed order, and a single
// rebuild replays every survivor in that order too, so the crowd's post-rebuild index assignment is a
// pure function of (surviving id set, that fixed order) -- independent of dictionary enumeration.
public enum PersonRegime
{
    Walking,
    Riding,
}

public enum PersonLifecycleEventKind
{
    Boarded,    // Walking -> Riding (despawn from the OrcaCrowd)
    Alighted,   // Riding -> Walking (spawn into the OrcaCrowd beside a parked car)
}

public readonly record struct PersonLifecycleEvent(int PersonId, PersonLifecycleEventKind Kind, double Time, Vec2 Position);

public sealed class PersonRideController
{
    private sealed class PersonEntry
    {
        public required int PersonId;
        public PersonRegime Regime;
        public OrcaHandle CrowdIndex = OrcaHandle.Invalid;   // handle into the CURRENT crowd when Walking; Invalid when Riding
        public double MaxSpeed;
        public double Radius;
        public Vec2 LastPosition;     // last known world position (kept across Board/Alight for observability)
    }

    private readonly double _boardRadius;
    private readonly Dictionary<int, PersonEntry> _people = new();
    private readonly List<int> _order = new();   // Track()/AddWalking() call order -- deterministic
    private readonly List<PersonLifecycleEvent> _events = new();
    private OrcaCrowd _crowd;
    private int _nextPersonId;
    private double _time;

    public PersonRideController(double boardRadius = 1.0, OrcaCrowd? crowd = null)
    {
        _boardRadius = boardRadius;
        _crowd = crowd ?? new OrcaCrowd();
    }

    public OrcaCrowd Crowd => _crowd;
    public IReadOnlyList<PersonLifecycleEvent> Events => _events;
    public double Time => _time;

    // Add a new Walking person directly into the crowd. Returns a STABLE PersonId (independent of the
    // crowd's own int index, which is invalidated on every rebuild).
    public int AddWalking(Vec2 position, double radius, double maxSpeed, Vec2 goal)
    {
        var id = _nextPersonId++;
        var idx = _crowd.Add(position, radius, maxSpeed, goal);
        _people[id] = new PersonEntry
        {
            PersonId = id,
            Regime = PersonRegime.Walking,
            CrowdIndex = idx,
            MaxSpeed = maxSpeed,
            Radius = radius,
            LastPosition = position,
        };
        _order.Add(id);
        return id;
    }

    public PersonRegime RegimeOf(int personId) => _people[personId].Regime;

    // The person's last known world position: the LIVE crowd position while Walking, the position at
    // the moment of boarding while Riding (there is no "true" position for a person inside a vehicle
    // at this POC's fidelity -- the car's own pose stands in for it).
    public Vec2 PositionOf(int personId)
    {
        var e = _people[personId];
        if (e.Regime == PersonRegime.Walking)
        {
            e.LastPosition = _crowd.Position(e.CrowdIndex);
        }

        return e.LastPosition;
    }

    // Board any Walking person within `_boardRadius` of ANY listed parked-car footprint. Evaluated
    // against the FROZEN pre-step positions (both peds and cars are read once, up front), in ascending
    // PersonId order, so the boarding SET is independent of iteration/dictionary order. A single
    // rebuild removes every boarder from the crowd in one pass (matches PedLodManager's "rebuild once
    // per membership-changing step", not once per boarder).
    public IReadOnlyList<int> TryBoard(IReadOnlyList<(int CarId, Vec2 Position)> parkedCars, double now)
    {
        _time = now;
        if (parkedCars.Count == 0)
        {
            return Array.Empty<int>();
        }

        var toBoard = new List<int>();
        foreach (var id in _order)
        {
            var e = _people[id];
            if (e.Regime != PersonRegime.Walking)
            {
                continue;
            }

            var pos = _crowd.Position(e.CrowdIndex);
            foreach (var (_, carPos) in parkedCars)
            {
                if ((pos - carPos).Abs <= _boardRadius)
                {
                    toBoard.Add(id);
                    break;
                }
            }
        }

        if (toBoard.Count == 0)
        {
            return Array.Empty<int>();
        }

        RebuildCrowdExcluding(toBoard);

        foreach (var id in toBoard)
        {
            var e = _people[id];
            e.Regime = PersonRegime.Riding;
            e.CrowdIndex = OrcaHandle.Invalid;
            _events.Add(new PersonLifecycleEvent(id, PersonLifecycleEventKind.Boarded, now, e.LastPosition));
        }

        return toBoard;
    }

    // Alight: spawn a NEW Walking person into the crowd at `carPosition + offset` (a deterministic
    // caller-chosen offset -- no RNG), with a walking destination. Returns the new PersonId.
    public int Alight(Vec2 carPosition, Vec2 offset, double radius, double maxSpeed, Vec2 destination, double now)
    {
        _time = now;
        var spawnPos = carPosition + offset;
        var id = AddWalking(spawnPos, radius, maxSpeed, destination);
        _events.Add(new PersonLifecycleEvent(id, PersonLifecycleEventKind.Alighted, now, spawnPos));
        return id;
    }

    // Rebuilds the crowd from scratch, carrying every surviving Walking person's LIVE position/
    // velocity/goal/radius/maxSpeed forward (mirrors PedLodManager.RebuildHighCrowd). Riding people
    // (already out of the crowd) and the just-boarded set are simply not re-added. Iterates `_order`
    // (ascending PersonId / call order) so the rebuilt crowd's index assignment is deterministic.
    private void RebuildCrowdExcluding(IReadOnlyList<int> boardingIds)
    {
        var boarding = new HashSet<int>(boardingIds);
        var oldCrowd = _crowd;
        var newCrowd = new OrcaCrowd();

        foreach (var id in _order)
        {
            var e = _people[id];
            if (e.Regime != PersonRegime.Walking)
            {
                continue;
            }

            var pos = oldCrowd.Position(e.CrowdIndex);
            if (boarding.Contains(id))
            {
                e.LastPosition = pos;   // stash for the Boarded event / post-board PositionOf reads
                continue;                // do not carry a boarder into the new crowd
            }

            var vel = oldCrowd.Velocity(e.CrowdIndex);
            var goal = oldCrowd.Goal(e.CrowdIndex);
            var newIdx = newCrowd.Add(pos, e.Radius, e.MaxSpeed, goal, vel);
            e.CrowdIndex = newIdx;
        }

        _crowd = newCrowd;
    }
}
