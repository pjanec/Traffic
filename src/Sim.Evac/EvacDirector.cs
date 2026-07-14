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
    private readonly VehicleMover _mover;
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

        // PANIC-EVAC-PHASE3-DESIGN.md §2, §4 (T3.1): the car has left the lane and is now a shaped
        // free-space mover in the band, still trying to flee. MoverIndex indexes the VehicleMover /
        // _pushers entry that carries it.
        public bool Pushing;
        public int MoverIndex = -1;
    }

    private readonly Dictionary<VehicleHandle, VehState> _veh = new();
    private readonly List<VehicleHandle> _order = new();          // deterministic iteration order

    // Pushing cars, in deterministic insertion (transition) order. A pusher stays in this list even
    // after it wedges/deactivates -- OrcaPushCount / ActivePushers() filter on VehicleMover.IsActive.
    private readonly List<(VehicleHandle Handle, int MoverIndex)> _pushers = new();

    private sealed class PedState { public bool Escaped; }
    private readonly List<PedState> _pedState = new();

    // Abandoned cars: static discs pedestrians keep avoiding after the vehicle leaves the sim.
    private readonly List<WorldDisc> _abandoned = new();

    // Exit edges + the world point of their far end (for away-from-incident reroute ranking).
    private readonly List<(string Edge, Vec2 End)> _exits = new();

    private WorldDisc[] _discScratch = new WorldDisc[16];
    private readonly List<VehicleHandle> _autoTrackScratch = new();
    private double _time;

    // PANIC-EVAC-PHASE5-DESIGN.md §4 (T4.2): opt-in per-phase profiler, null unless EnableProfiling()
    // is called. Every use below is a `_profiler?.` conditional-access no-op when null, so profiling
    // OFF (every existing demo/test) is zero-cost and byte-identical to before this field existed.
    private EvacProfiler? _profiler;

    public EvacDirector(Engine engine, NetworkModel net, Incident incident, EvacConfig cfg, double stepLength)
    {
        _engine = engine;
        _cfg = cfg;
        _incident = incident;
        _stepLength = stepLength;
        _navmesh = new FakeNavMesh(net, cfg.VicinityWidth);
        _blocked = new BlockedDetector(cfg.BlockedDwellSeconds);

        _mover = new VehicleMover(cfg);
        _mover.ArmWalls(_navmesh.BandWalls);         // PHASE3-DESIGN.md §4: confine pushers to the band

        _peds.AddObstacle(_navmesh.BoundaryLoop);   // R7 hard outer edge
        _peds.MaxNeighbours = 8;                     // convergence aid for dense foot-exodus

        // PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2b: opt-in spatial hash for BOTH crowd solvers (pedestrian
        // OrcaCrowd + pusher MixedTrafficCrowd via VehicleMover's pass-through). Default off -> every
        // existing demo/test byte-identical; proven bit-identical to brute-force when on.
        if (cfg.UseCrowdSpatialHash)
        {
            _peds.UseSpatialHash = true;
            _mover.UseSpatialHash = true;
        }
        // T3.1 §3: composite source -- pedestrians AND pushing cars are both obstacles to the lane
        // engine. Still one seam, inert when both are empty, so parity is unaffected.
        _engine.CrowdSource = new CompositeFootprintSource(_peds, _mover);   // R5

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

    // ----- profiling (T4.2, opt-in, off by default) -----

    // Turn on per-phase wall-time accounting for Tick(); returns the profiler so a caller (e.g.
    // Sim.EvacProfile) can Snapshot() it after running some ticks. Idempotent -- a second call
    // returns the SAME profiler instance rather than resetting accumulated timings.
    public EvacProfiler EnableProfiling() => _profiler ??= new EvacProfiler();

    // Cumulative per-phase timings since EnableProfiling() was called; all-zero if profiling was
    // never enabled (never throws -- a read-only, always-safe observability seam).
    public ProfileSnapshot Profile => _profiler?.Snapshot() ?? default;

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
        var tickStart = _profiler?.Begin() ?? 0L;

        _time += _stepLength;
        PreStep();

        var engineStart = _profiler?.Begin() ?? 0L;
        _engine.Step();
        _profiler?.End(engineStart, EvacProfiler.Phase.EngineStep);

        PostStep();

        _profiler?.EndTick(tickStart);
    }

    // PANIC-EVAC-PHASE5-DESIGN.md §3 (T1.2): opt-in auto-attach for `LoadScenario` demand -- vehicles
    // are never individually `Track`ed by the caller on a realistic town, so the director scans the
    // engine's published read surface itself and starts watching anything that drives into the working
    // region. Runs every tick (a vehicle can drive INTO the region mid-run); vehicles that never enter
    // stay pure parity traffic at zero evac cost (design §1). Off unless AutoTrackInWorkingRegion is set,
    // so the grid/TLS demos (explicit Track only) are unaffected.
    private void AutoTrackWorkingRegion()
    {
        if (!_cfg.AutoTrackInWorkingRegion)
        {
            return;
        }

        var handles = _engine.VehicleHandles;
        var px = _engine.PosX;
        var py = _engine.PosY;

        _autoTrackScratch.Clear();
        for (var i = 0; i < handles.Length; i++)
        {
            var handle = handles[i];
            if (_veh.ContainsKey(handle))
            {
                continue;
            }

            if (_incident.DistanceTo(px[i], py[i]) <= _cfg.WorkingRadius)
            {
                _autoTrackScratch.Add(handle);
            }
        }

        // Deterministic regardless of read-buffer slot order: sort by handle.Index before Track()ing.
        _autoTrackScratch.Sort((a, b) => a.Index.CompareTo(b.Index));
        foreach (var handle in _autoTrackScratch)
        {
            Track(handle);
        }
    }

    // PANIC-EVAC-PHASE2-DESIGN.md §5: instead of a radius-only instant latch, drive the per-vehicle
    // fear field every tick (it is inert pre-incident: FearField gates the direct term on incident
    // activity/radius internally, so nothing seeds without a live, in-range, visible incident).
    private void PreStep()
    {
        AutoTrackWorkingRegion();

        var discFeedsStart = _profiler?.Begin() ?? 0L;
        FeedVehicleDiscsToPeds();
        _profiler?.End(discFeedsStart, EvacProfiler.Phase.DiscFeeds);

        var fearUpdateStart = _profiler?.Begin() ?? 0L;
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
        _profiler?.End(fearUpdateStart, EvacProfiler.Phase.FearUpdate);

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
                if (_cfg.EnableOrcaPush && !st.Pushing)
                {
                    EnterOrcaPush(handle, v, st);                          // PHASE3: shaped-mover handoff
                }
                else
                {
                    Convert(handle, v);                                    // R6 (Phase-1/2, unchanged)
                    st.Converted = true;
                    st.Alive = false;
                }
            }
        }

        var pusherStepStart = _profiler?.Begin() ?? 0L;
        DriveOrcaPushers();
        _profiler?.End(pusherStepStart, EvacProfiler.Phase.PusherStep);

        var pedestrianStepStart = _profiler?.Begin() ?? 0L;
        DrivePedestrians();
        _profiler?.End(pedestrianStepStart, EvacProfiler.Phase.PedestrianStep);
    }

    // R6: a panicked, blocked car — its occupants abandon it. Freeze the car as an obstacle, remove it
    // from the driving sim, and spawn its occupants as pedestrians the external layer now drives.
    private void Convert(VehicleHandle handle, VehicleState v)
    {
        var pos = new Vec2(v.X, v.Y);
        _abandoned.Add(new WorldDisc(pos.X, pos.Y, 0.0, 0.0, _cfg.VehicleDiscRadius));
        _engine.Despawn(handle);
        _blocked.Forget(handle);
        _fearField.Forget(handle);
        SpawnPedestriansAt(handle, pos);
    }

    // Shared by Convert (direct blocked->pedestrian, EnableOrcaPush=false) and the wedge->pedestrian
    // handoff below (EnableOrcaPush=true): spawn PedestriansPerCar pedestrians around `pos`, exactly
    // the loop Convert always ran, now factored out so both paths spawn identically.
    private void SpawnPedestriansAt(VehicleHandle handle, Vec2 pos)
    {
        for (var k = 0; k < _cfg.PedestriansPerCar; k++)
        {
            var offset = DeterministicOffset(handle, k);
            var spawnPos = _navmesh.ClampInterior(new Vec2(pos.X + offset.X, pos.Y + offset.Y));
            _peds.Add(spawnPos, _cfg.PedRadius, _cfg.PedMaxSpeed, FleeGoalFor(spawnPos));
            _pedState.Add(new PedState());
        }
    }

    // PANIC-EVAC-PHASE3-DESIGN.md §2, §4 (T3.1): a panicked, blocked car mounts the shoulder instead of
    // instantly abandoning the lane. Despawn from the lane engine and hand it to the VehicleMover as a
    // shaped, non-holonomic free-space pusher, still fleeing away from the incident. Not Converted --
    // the entity is still in play (as a pusher), just off the lane.
    private void EnterOrcaPush(VehicleHandle handle, VehicleState v, VehState st)
    {
        // SUMO angle: degrees, 0 = north, increasing clockwise. MixedTrafficCrowd expects radians,
        // 0 = +x, counter-clockwise (standard math heading) -- convert once at the handoff.
        var headingRad = (90.0 - v.Angle) * Math.PI / 180.0;
        var pos = new Vec2(v.X, v.Y);

        _engine.Despawn(handle);
        var idx = _mover.AddCar(pos, headingRad, FleeGoalForPusher(pos));

        st.Pushing = true;
        st.MoverIndex = idx;
        st.Alive = false;

        _blocked.Forget(handle);
        _fearField.Forget(handle);
        _pushers.Add((handle, idx));
    }

    // Drive every pushing mover one tick: re-aim its away-goal, advance the shaped crowd, then hand
    // off any mover that has wedged (no progress for the dwell) to the pedestrian stage -- the same
    // Phase-1/2 conversion, now sourced from the Orca-push stage instead of directly from the lane.
    private void DriveOrcaPushers()
    {
        foreach (var (_, idx) in _pushers)
        {
            if (_mover.IsActive(idx))
            {
                _mover.SetGoal(idx, FleeGoalForPusher(_mover.Position(idx)));
            }
        }

        _mover.Step(_stepLength);   // VehicleMover sub-steps internally (OrcaCrowdSubSteps)

        foreach (var (handle, idx) in _pushers)
        {
            if (!_mover.IsActive(idx) || !_mover.IsWedged(idx))
            {
                continue;
            }

            var pos = _mover.Position(idx);
            SpawnPedestriansAt(handle, pos);
            _abandoned.Add(new WorldDisc(pos.X, pos.Y, 0.0, 0.0, _cfg.VehicleDiscRadius));
            // A small box around the wreck so other pushers avoid it too (mirrors the abandoned-car
            // obstacle fed to pedestrians, now fed to the shaped-mover crowd as a static block).
            var r = _cfg.VehicleDiscRadius;
            _mover.AddBlock(pos.X - r, pos.Y - r, pos.X + r, pos.Y + r);
            _mover.Deactivate(idx);

            if (_veh.TryGetValue(handle, out var st))
            {
                st.Converted = true;
                st.Pushing = false;
            }
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
    private Vec2 FleeGoalFor(Vec2 p) => FleeGoalFor(p, PedGoalInset);

    // A shaped pusher's away-goal needs MORE clearance from the wall than a pedestrian point does: a
    // pedestrian is a disc that genuinely comes to rest right at the (holonomic) boundary, but a
    // pusher's goal sitting pinned in/near a CORNER (where two band walls meet) asks the shaped,
    // non-holonomic mover to occupy a spot two intersecting walls actually make unreachable -- it
    // never settles, it grinds along both walls in a persistent limit cycle instead (never registering
    // as wedged, since instantaneous speed during the cycle stays above OrcaWedgeSpeed). A generous
    // inset keeps the goal comfortably off both the wall AND the corner, so the mover's own attractor
    // is a reachable point, not a mathematically-clamped one two walls are already occupying.
    private const double PusherGoalInset = 5.0;
    private const double PedGoalInset = 0.5;   // FakeNavMesh.ClampInterior's own default, made explicit

    private Vec2 FleeGoalForPusher(Vec2 p) => FleeGoalFor(p, PusherGoalInset);

    private Vec2 FleeGoalFor(Vec2 p, double inset)
    {
        var away = new Vec2(p.X - _incident.X, p.Y - _incident.Y).Normalized();
        if (away.AbsSq < 1e-9)
        {
            away = new Vec2(1.0, 0.0);   // exactly at the epicentre: pick a fixed direction (+x)
        }

        return _navmesh.ClampInterior(p + away * _cfg.FleeGoalDistance, inset);
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

        foreach (var (_, idx) in _pushers)
        {
            if (_mover.IsActive(idx))
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

        // Pushing (Orca-push) cars are moving obstacles too -- pedestrians must avoid them exactly
        // like lane vehicles (PHASE3-DESIGN.md §4, cross-avoidance).
        foreach (var (_, idx) in _pushers)
        {
            if (!_mover.IsActive(idx))
            {
                continue;
            }

            var p = _mover.Position(idx);
            var v = _mover.Velocity(idx);
            _discScratch[n++] = new WorldDisc(p.X, p.Y, v.X, v.Y, _cfg.VehicleDiscRadius);
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

    // PANIC-EVAC-PHASE5-DESIGN.md §3: whether `h` is under the evac layer's watch at all (explicitly
    // `Track`ed, or auto-tracked into the working region) -- distinguishes "never entered the region"
    // from "tracked, then despawned/converted" (IsAlive is false for both).
    public bool IsTracked(VehicleHandle h) => _veh.ContainsKey(h);

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

    // ----- Phase-3 (Orca-push) observability -----

    public int OrcaPushCount
    {
        get { var n = 0; foreach (var (_, idx) in _pushers) if (_mover.IsActive(idx)) n++; return n; }
    }

    public bool IsPushing(VehicleHandle h) => _veh.TryGetValue(h, out var s) && s.Pushing;

    public int PusherCount => _mover.Count;

    // Active pusher poses, for viz/tests -- iterates _pushers in deterministic (transition) order.
    public IEnumerable<(double X, double Y, double HeadingRad)> ActivePushers()
    {
        foreach (var (_, idx) in _pushers)
        {
            if (_mover.IsActive(idx))
            {
                var p = _mover.Position(idx);
                yield return (p.X, p.Y, _mover.Heading(idx));
            }
        }
    }

    // As ActivePushers, but also yields the pusher's originating VehicleHandle -- a STABLE identity the
    // viz keys a fixed disc slot on, so the front-end's index-matched disc interpolation never smears
    // one entity's position into another (see SceneGen's stable-disc-slot assembly).
    public IEnumerable<(VehicleHandle Handle, double X, double Y, double HeadingRad)> ActivePushersWithHandle()
    {
        foreach (var (handle, idx) in _pushers)
        {
            if (_mover.IsActive(idx))
            {
                var p = _mover.Position(idx);
                yield return (handle, p.X, p.Y, _mover.Heading(idx));
            }
        }
    }
}
