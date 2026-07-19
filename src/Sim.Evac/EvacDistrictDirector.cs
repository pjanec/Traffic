using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;

namespace Sim.Evac;

// P5-1(B) (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §6 "Evac -- a SPECIALIZATION, not a
// fork"): a pedestrian panic-evacuation over a REAL walkable net (scenarios/_ped/evac-district,
// P5-PRE), built ENTIRELY on the Sim.Pedestrians PedestrianWorld facade -- no new PedLodManager /
// PedestrianWorld core method, per the design decision recorded in the task brief. Panicked peds
// route to their NEAREST safe zone ALONG the sidewalk grid (via IPedNavigation / SumoNavMesh -- the
// same routing EvacDistrictNetTests proves bends around blocks rather than cutting through them),
// which is what replaces the legacy EvacDirector's radial FleeGoalFor/FakeNavMesh flee for this NEW
// walkable-net scenario. EvacDirector itself is UNTOUCHED: its own car-centric, radial-flee,
// FakeNavMesh-band tests stay exactly as they were; this is a wholly separate, additive class living
// alongside it.
//
// PANIC DECISION -- simple incident-proximity, not FearField (docs/PEDESTRIAN-DESIGN.md §6: "if
// FearField is VehicleHandle-keyed and awkward for peds, a simple incident-proximity panic ... is
// acceptable"). FearField (src/Sim.Evac/FearField.cs) is keyed on VehicleHandle and threads a
// per-vehicle DR-model / line-of-sight / contagion bookkeeping the car-centric director already owns
// end-to-end (fed by TryGetVehicle, GetDrModel, ...); retrofitting it for an int ped id would mean
// either duplicating that per-entity state or forking the class, neither of which is "cleanly
// reusable" for a first walkable-net ped evac. This class instead panics any calm ped within
// Incident.Radius of the epicentre once Incident.IsActive(time) -- Incident itself (FearAt,
// IsActive, DistanceTo, src/Sim.Evac/Incident.cs) IS reused unchanged; only its distance/activity
// primitives are used (not FearAt's linear falloff -- panic here is a hard radius latch, matching
// EvacConfig.ThetaPanic's "essentially anyone inside the incident radius panics" Phase-1 model).
//
// PANIC / REROUTE / ESCAPE -- exactly the three PedestrianWorld facade calls the design prescribes,
// nothing else:
//   panic:   world.Remove(id);
//            world.AddWalker(id, frozenPos, nearestSafeZone, maxSpeed, radius, now);
//            world.SetForcedHighPower(id, true);
//   escape:  once within ArriveRadius of its assigned safe zone, world.Remove(id) and mark Escaped.
// Re-adding at the SAME (frozen, pre-panic) position with a route to the safe zone is visually
// continuous (PedLodManager.AddPed publishes the new PathArc leg starting exactly there) and the
// SetForcedHighPower pin promotes the ped on the very next Step regardless of any InterestField (none
// is registered by this director -- every ped panics via this class's own incident-proximity check,
// never via the facade's interest-source promotion path) and never demotes while pinned.
public sealed class EvacDistrictDirector
{
    private readonly PedestrianWorld _world;
    private readonly IPedNavigation _navigation;
    private readonly Incident _incident;
    private readonly IReadOnlyList<Vec2> _safeZones;
    private readonly EvacDistrictConfig _cfg;

    private sealed class PedState
    {
        // Sticky: true once panicked, even after Escaped -- mirrors EvacDirector's own
        // PanickedCount/ConvertedCount convention (cumulative, never decreases), which is exactly
        // what lets EscapedCount "reach" PanickedCount over a long-enough run.
        public bool Panicked;
        public bool Escaped;

        public Vec2 SafeZone;        // assigned once, at panic time (the NEAREST zone to PanicPosition)
        public Vec2 PanicPosition;   // the ped's frozen position AT the moment it panicked

        // Positions recorded every Step() from the moment this ped panicked to (and including) its
        // last position before escaping -- used to measure routed (bends around blocks) vs radial
        // (straight-line) arc length.
        public readonly List<Vec2> Trace = new();
    }

    private readonly Dictionary<int, PedState> _peds = new();
    private readonly List<int> _order = new();   // deterministic iteration order (ascending id, insertion order)
    private int _nextId = 1;
    private double _time;

    public EvacDistrictDirector(
        IPedNavigation navigation,
        Incident incident,
        IReadOnlyList<Vec2> safeZones,
        EvacDistrictConfig? config = null)
    {
        if (safeZones is null || safeZones.Count == 0)
        {
            throw new ArgumentException("At least one safe zone is required.", nameof(safeZones));
        }

        _navigation = navigation ?? throw new ArgumentNullException(nameof(navigation));
        _incident = incident;
        _safeZones = safeZones;
        _cfg = config ?? new EvacDistrictConfig();

        _world = new PedestrianWorld(_navigation, _cfg.ArriveRadius, _cfg.DwellSeconds);

        SpawnAmbientPopulation();
    }

    // Deterministic ambient population: a hash-like (golden-angle) spread of origins across the
    // district's bounding box -- no System.Random, mirroring EvacDirector.DeterministicOffset's own
    // "a*index (golden-angle-ish)" construction -- each routed to another such point elsewhere in the
    // district as its initial calm destination (some ambient walk, per the task brief). Every origin/
    // destination is snapped onto the real sidewalk network by IPedNavigation.FindPath's own "clamp
    // off-mesh points onto walkable space" contract (SumoNavMesh.LocatePolygon), so the exact input
    // coordinates here need not themselves lie on a sidewalk -- only be inside the district.
    private void SpawnAmbientPopulation()
    {
        for (var i = 0; i < _cfg.PedCount; i++)
        {
            var origin = DeterministicDistrictPoint(i);
            var destination = DeterministicDistrictPoint(i + _cfg.PedCount);
            var id = _nextId++;

            if (!_world.AddWalker(id, origin, destination, _cfg.MaxSpeed, _cfg.PedRadius, now: 0.0))
            {
                continue;   // unroutable (not expected on this connected net) -- skip, no ghost id
            }

            _peds[id] = new PedState();
            _order.Add(id);
        }
    }

    // A deterministic, non-RNG spread of points across the district's ~[0,200]x[0,200] extent (see
    // scenarios/_ped/evac-district/NOTES.md: a 3x3 grid, 100 m spacing, incident at the centre
    // (100,100)), golden-angle spaced so consecutive indices land far apart in angle (avoiding an
    // accidental alignment with the sidewalk lattice's own symmetry or the incident's diagonal).
    private static Vec2 DeterministicDistrictPoint(int i)
    {
        const double GoldenAngle = 2.399963; // radians -- same constant EvacDirector.DeterministicOffset uses
        var angle = i * GoldenAngle;
        var radius = 15.0 + (i % 9) * 15.0; // 15..135 m from the district centre
        var x = 100.0 + (radius * Math.Cos(angle));
        var y = 100.0 + (radius * Math.Sin(angle));
        return new Vec2(Math.Clamp(x, 2.0, 198.0), Math.Clamp(y, 2.0, 198.0));
    }

    // Advances the district by `dt`:
    //   1. Panic-check every still-calm ped against the incident, using FROZEN start-of-step
    //      positions (deterministic, thread-order-independent, mirroring PedLodManager.Step's own
    //      "freeze positions, then decide transitions" discipline) -- react/reroute panicked ones.
    //   2. Step the pedestrian world (advances low-power motion and the high-power ORCA crowd).
    //   3. Arrival-check every panicked-and-not-yet-escaped ped against ITS assigned safe zone, using
    //      the just-advanced (post-step) positions, and retire any that arrived.
    public void Step(double dt)
    {
        if (_incident.IsActive(_time))
        {
            foreach (var id in _order)
            {
                var st = _peds[id];
                if (st.Panicked)
                {
                    continue;
                }

                var pos = _world.PositionOf(id, _time);
                if (_incident.DistanceTo(pos.X, pos.Y) <= _cfg.PanicRadius)
                {
                    Panic(id, st, pos);
                }
            }
        }

        _world.Step(_time, dt);
        _time += dt;

        foreach (var id in _order)
        {
            var st = _peds[id];
            if (!st.Panicked || st.Escaped)
            {
                continue;
            }

            var pos = _world.PositionOf(id, _time);
            st.Trace.Add(pos);

            if ((pos - st.SafeZone).Abs <= _cfg.ArriveRadius)
            {
                st.Escaped = true;
                _world.Remove(id);
            }
        }
    }

    private void Panic(int id, PedState st, Vec2 pos)
    {
        var safeZone = NearestSafeZone(pos, _safeZones);

        _world.Remove(id);
        var rerouted = _world.AddWalker(id, pos, safeZone, _cfg.MaxSpeed, _cfg.PedRadius, _time);
        if (!rerouted)
        {
            // Every safe zone is routable from anywhere on this connected net
            // (EvacDistrictNetTests.EvacDistrict_AllFourCornerSafeZones_AreRoutableFromTheCentre), so
            // this should never trigger; fail safe by re-registering the ped in place rather than
            // leaving it deregistered and silently dropped from the population.
            _world.AddWalker(id, pos, pos, _cfg.MaxSpeed, _cfg.PedRadius, _time);
            return;
        }

        _world.SetForcedHighPower(id, true);

        st.Panicked = true;
        st.SafeZone = safeZone;
        st.PanicPosition = pos;
        st.Trace.Clear();
        st.Trace.Add(pos);
    }

    // The nearest of `safeZones` to `pos` (fixed tie-break: first-in-list wins an exact tie -- never
    // exercised on this net's asymmetric ambient spawn points, but keeps the choice a pure,
    // deterministic function of its inputs). Exposed (static, public) so tests can validate the
    // nearest-zone rule directly, independent of any particular ped's spawn/panic position.
    public static Vec2 NearestSafeZone(Vec2 pos, IReadOnlyList<Vec2> safeZones)
    {
        var best = safeZones[0];
        var bestDistSq = double.MaxValue;
        foreach (var zone in safeZones)
        {
            var distSq = (zone - pos).AbsSq;
            if (distSq < bestDistSq)
            {
                bestDistSq = distSq;
                best = zone;
            }
        }

        return best;
    }

    // ----- observability (tests + viz) -----

    public double Time => _time;
    public Incident Incident => _incident;
    public IReadOnlyList<Vec2> SafeZones => _safeZones;

    // Total ambient population ever spawned (constant after construction -- this scenario has no
    // demand generator; the panic/flee/escape lifecycle governs its members instead).
    public int PedestrianCount => _peds.Count;

    public int PanickedCount => _peds.Values.Count(s => s.Panicked);
    public int EscapedCount => _peds.Values.Count(s => s.Escaped);

    public bool IsPanicked(int id) => _peds.TryGetValue(id, out var s) && s.Panicked;
    public bool IsEscaped(int id) => _peds.TryGetValue(id, out var s) && s.Escaped;

    // The ped's current world position. Valid only while NOT escaped -- an escaped ped has been
    // Remove()d from the facade, so querying it afterward throws (mirrors PedLodManager's own
    // "unregistered id" contract; check IsEscaped first).
    public Vec2 PositionOf(int id) => _world.PositionOf(id, _time);

    // Valid only while NOT escaped, for the same reason as PositionOf.
    public PedDrModel ModelOf(int id) => _world.ModelOf(id);

    public Vec2 SafeZoneOf(int id) => _peds[id].SafeZone;
    public Vec2 PanicPositionOf(int id) => _peds[id].PanicPosition;

    // The recorded position trace from the moment `id` panicked to (and including) its last position
    // before escaping (or "so far", if it has not escaped yet) -- used to measure routed (bends
    // around blocks) vs radial (straight-line) arc length.
    public IReadOnlyList<Vec2> TraceOf(int id) => _peds[id].Trace;

    public int HighPowerCount => _world.HighPowerCount;
}
