using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Ingest;

namespace Sim.Evac;

// PANIC-EVAC.md §2 / §8: the EXTERNAL evac system, orchestrating the whole transition on top of the
// unchanged driving core. Each coordinated tick:
//   PRE  (before the engine step):
//     • feed current car footprints (live + abandoned) to the pedestrian crowd, so pedestrians avoid
//       cars (R8 interior obstacles);
//     • compute radius fear (R3); newly-panicked cars get the flee preset (SetVehicleParams, R2) and a
//       reroute toward an away-exit (SetDestination).
//   engine.Step()  — vehicles drive (and avoid pedestrians via Engine.CrowdSource = the ped crowd, R5).
//   POST (after the engine step):
//     • update the blocked signal (R4); a panicked + blocked car is converted (R6): Despawn it, leave
//       an abandoned-car obstacle, and spawn its occupants as pedestrians into the crowd;
//     • drive all pedestrians radially away from the incident within the fake-navmesh (ORCA + hard
//       edge), marking each Escaped once beyond the safe radius (R6b).
//
// The core is only ever DRIVEN through its public seams; nothing here is on the golden/parity path, so
// the determinism hash cannot move (R9). Iteration is over a fixed insertion order and uses no RNG /
// wall-clock, so a run is deterministic.
public sealed class EvacDirector
{
    private readonly Engine _engine;
    private readonly EvacConfig _cfg;
    private readonly Incident _incident;
    private readonly FakeNavMesh _navmesh;
    private readonly OrcaCrowd _peds = new();
    private readonly BlockedDetector _blocked;
    private readonly FearField _fearField = new();
    private readonly double _stepLength;

    private sealed class VehState
    {
        public bool Panicked;
        public bool Converted;
        public bool Alive = true;

        // Set once TryGetVehicle has EVER succeeded for this handle. A vehicle can be legitimately
        // absent from the engine's live table for a tick or two after SpawnVehicle (insertion happens
        // inside Engine.Step(), so a PreStep TryGetVehicle check on the very first tick -- before
        // Step() has run even once -- can miss a not-yet-inserted vehicle). That is "not here YET", not
        // "gone for good": only a TryGetVehicle failure AFTER at least one success means the vehicle
        // has genuinely arrived/despawned.
        public bool Seen;
    }

    private readonly Dictionary<VehicleHandle, VehState> _veh = new();
    private readonly List<VehicleHandle> _order = new();          // deterministic iteration order

    private sealed class PedState { public bool Escaped; }
    private readonly List<PedState> _pedState = new();

    // Abandoned cars: static discs pedestrians keep avoiding after the vehicle leaves the sim.
    private readonly List<WorldDisc> _abandoned = new();

    // Exit edges + the world point of their far end (for away-from-incident reroute ranking).
    private readonly List<(string Edge, Vec2 End)> _exits = new();

    private WorldDisc[] _discScratch = new WorldDisc[16];
    private double _time;

    public EvacDirector(Engine engine, NetworkModel net, Incident incident, EvacConfig cfg, double stepLength)
    {
        _engine = engine;
        _cfg = cfg;
        _incident = incident;
        _stepLength = stepLength;
        _navmesh = new FakeNavMesh(net, cfg.VicinityWidth);
        _blocked = new BlockedDetector(cfg.BlockedDwellSeconds);

        _peds.AddObstacle(_navmesh.BoundaryLoop);   // R7 hard outer edge
        _peds.MaxNeighbours = 8;                     // convergence aid for dense foot-exodus
        _engine.CrowdSource = _peds;                 // R5: vehicles avoid pedestrians

        foreach (var edgeId in cfg.ExitEdges)
        {
            if (net.EdgesById.TryGetValue(edgeId, out var edge) && edge.Lanes.Count > 0)
            {
                var shape = edge.Lanes[0].Shape;
                var end = shape[^1];
                _exits.Add((edgeId, new Vec2(end.X, end.Y)));
            }
        }
    }

    // ----- setup -----

    // Put a vehicle (spawned by the caller) under the evac layer's watch.
    public void Track(VehicleHandle handle)
    {
        if (_veh.ContainsKey(handle))
        {
            return;
        }

        _veh[handle] = new VehState();
        _order.Add(handle);
    }

    // ----- the coordinated tick -----

    public void Tick()
    {
        _time += _stepLength;
        PreStep();
        _engine.Step();
        PostStep();
    }

    // PANIC-EVAC-PHASE2-DESIGN.md §5: instead of a radius-only instant latch, drive the per-vehicle
    // fear field every tick (it is inert pre-incident: FearField gates the direct term on incident
    // activity/radius internally, so nothing seeds without a live, in-range, visible incident).
    private void PreStep()
    {
        FeedVehicleDiscsToPeds();

        var obs = new List<FearField.VehicleObs>();
        foreach (var handle in _order)
        {
            var st = _veh[handle];
            if (!st.Alive || st.Converted)
            {
                continue;
            }

            if (!_engine.TryGetVehicle(handle, out var v))
            {
                // Only a failure AFTER having been seen at least once is a real despawn/arrival; a
                // miss before the vehicle's first successful observation is pre-insertion, not death
                // (see VehState.Seen) -- retry next tick instead of latching it dead prematurely.
                if (st.Seen)
                {
                    st.Alive = false;
                }

                continue;
            }

            st.Seen = true;
            obs.Add(new FearField.VehicleObs(handle, v.X, v.Y, _engine.GetDrModel(handle) == DrModel.Stationary));
        }

        var newlyLatched = _fearField.Update(obs, _incident, _time, _abandoned, _cfg, _stepLength);

        foreach (var handle in newlyLatched)
        {
            var st = _veh[handle];
            if (st.Alive && !st.Converted && !st.Panicked)
            {
                st.Panicked = true;
                _engine.SetVehicleParams(handle, _cfg.FleePreset);   // R2 flee preset
                if (_engine.TryGetVehicle(handle, out var v2))
                {
                    RerouteAway(handle, v2);                          // R2 flee route
                }
            }
        }
    }

    private void PostStep()
    {
        foreach (var handle in _order)
        {
            var st = _veh[handle];
            if (!st.Alive || st.Converted)
            {
                continue;
            }

            if (!_engine.TryGetVehicle(handle, out var v))
            {
                st.Alive = false;
                _blocked.Forget(handle);
                continue;
            }

            var blocked = _blocked.Update(_engine, handle, _stepLength);   // R4
            if (blocked && st.Panicked)
            {
                Convert(handle, v);                                        // R6
                st.Converted = true;
                st.Alive = false;
            }
        }

        DrivePedestrians();
    }

    // R6: a panicked, blocked car — its occupants abandon it. Freeze the car as an obstacle, remove it
    // from the driving sim, and spawn its occupants as pedestrians the external layer now drives.
    private void Convert(VehicleHandle handle, VehicleState v)
    {
        _abandoned.Add(new WorldDisc(v.X, v.Y, 0.0, 0.0, _cfg.VehicleDiscRadius));
        _engine.Despawn(handle);
        _blocked.Forget(handle);
        _fearField.Forget(handle);

        for (var k = 0; k < _cfg.PedestriansPerCar; k++)
        {
            var offset = DeterministicOffset(handle, k);
            var pos = _navmesh.ClampInterior(new Vec2(v.X + offset.X, v.Y + offset.Y));
            _peds.Add(pos, _cfg.PedRadius, _cfg.PedMaxSpeed, FleeGoalFor(pos));
            _pedState.Add(new PedState());
        }
    }

    private void DrivePedestrians()
    {
        for (var i = 0; i < _peds.Count; i++)
        {
            var p = _peds.Position(i);
            var st = _pedState[i];
            if (!st.Escaped && _incident.DistanceTo(p.X, p.Y) >= _cfg.SafeRadius)
            {
                st.Escaped = true;   // R6b
            }

            // Escaped pedestrians hold position (goal = current point); the rest keep fleeing radially.
            _peds.SetGoal(i, st.Escaped ? p : FleeGoalFor(p));
        }

        var sub = Math.Max(1, _cfg.CrowdSubSteps);
        var subDt = _stepLength / sub;
        for (var s = 0; s < sub; s++)
        {
            _peds.Step(subDt);
        }
    }

    // A radial away-from-incident goal, clamped inside the navmesh (so it lands on the hard edge).
    private Vec2 FleeGoalFor(Vec2 p)
    {
        var away = new Vec2(p.X - _incident.X, p.Y - _incident.Y).Normalized();
        if (away.AbsSq < 1e-9)
        {
            away = new Vec2(1.0, 0.0);   // exactly at the epicentre: pick a fixed direction (+x)
        }

        return _navmesh.ClampInterior(p + away * _cfg.FleeGoalDistance);
    }

    private void FeedVehicleDiscsToPeds()
    {
        var count = _abandoned.Count;
        foreach (var handle in _order)
        {
            var st = _veh[handle];
            if (st.Alive && !st.Converted && _engine.TryGetVehicle(handle, out _))
            {
                count++;
            }
        }

        if (_discScratch.Length < count)
        {
            _discScratch = new WorldDisc[Math.Max(count, 16)];
        }

        var n = 0;
        foreach (var handle in _order)
        {
            var st = _veh[handle];
            if (!st.Alive || st.Converted || !_engine.TryGetVehicle(handle, out var v))
            {
                continue;
            }

            // SUMO angle: degrees, 0 = north, increasing clockwise.
            var a = v.Angle * Math.PI / 180.0;
            var vx = v.Speed * Math.Sin(a);
            var vy = v.Speed * Math.Cos(a);
            _discScratch[n++] = new WorldDisc(v.X, v.Y, vx, vy, _cfg.VehicleDiscRadius);
        }

        foreach (var d in _abandoned)
        {
            _discScratch[n++] = d;
        }

        _peds.SetExternalObstacles(_discScratch.AsSpan(0, n));
    }

    private void RerouteAway(VehicleHandle handle, VehicleState v)
    {
        // Try exits from farthest-from-incident to nearest until one is routable; keep the current
        // destination if none is (SetDestination returns false, e.g. mid-junction — retry next panic).
        foreach (var (edge, _) in ExitsFarthestFirst())
        {
            if (_engine.SetDestination(handle, edge))
            {
                return;
            }
        }
    }

    private IEnumerable<(string Edge, Vec2 End)> ExitsFarthestFirst()
    {
        var list = new List<(string Edge, Vec2 End)>(_exits);
        list.Sort((x, y) =>
        {
            var byDist = _incident.DistanceTo(y.End.X, y.End.Y)
                .CompareTo(_incident.DistanceTo(x.End.X, x.End.Y));
            return byDist != 0 ? byDist : string.CompareOrdinal(x.Edge, y.Edge);
        });
        return list;
    }

    // A tiny deterministic spread (no RNG) so multiple occupants of one car don't spawn exactly stacked.
    private static Vec2 DeterministicOffset(VehicleHandle handle, int k)
    {
        var a = (int)handle.Index * 2.399963 + k * 1.107149;   // golden-angle-ish, stable per (car, k)
        return new Vec2(Math.Cos(a) * 0.6, Math.Sin(a) * 0.6);
    }

    // ----- observability (tests + viz) -----

    public double Time => _time;
    public Incident Incident => _incident;
    public FakeNavMesh NavMesh => _navmesh;
    public IReadOnlyList<Vec2> BoundaryLoop => _navmesh.BoundaryLoop;

    public int PedestrianCount => _peds.Count;
    public Vec2 PedestrianPosition(int i) => _peds.Position(i);
    public bool PedestrianEscaped(int i) => _pedState[i].Escaped;

    public int AbandonedCarCount => _abandoned.Count;
    public WorldDisc AbandonedCar(int i) => _abandoned[i];

    public double Fear(VehicleHandle h) => _fearField.Fear(h);

    public bool IsPanicked(VehicleHandle h) => _veh.TryGetValue(h, out var s) && s.Panicked;
    public bool IsConverted(VehicleHandle h) => _veh.TryGetValue(h, out var s) && s.Converted;
    public bool IsAlive(VehicleHandle h) => _veh.TryGetValue(h, out var s) && s.Alive;

    public int PanickedCount
    {
        get { var n = 0; foreach (var s in _veh.Values) if (s.Panicked) n++; return n; }
    }

    public int ConvertedCount
    {
        get { var n = 0; foreach (var s in _veh.Values) if (s.Converted) n++; return n; }
    }

    public int EscapedCount
    {
        get { var n = 0; foreach (var s in _pedState) if (s.Escaped) n++; return n; }
    }
}
