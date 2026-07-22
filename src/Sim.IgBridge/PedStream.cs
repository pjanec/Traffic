using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.IgBridge;

// One buffered pose for a pedestrian, mirroring TimestampedSample's (t, position) shape but for the
// SEPARATE ped reconstruction stack (docs task: peds do NOT go through VehicleSampleHistory/DrClock --
// PathArc dead-reckoning is its own pure function of (path, startTime, speed, now), so it needs none
// of DrClock's lane/upcoming-lanes machinery). Heading is deliberately NOT stored here: it is
// velocity-derived from two consecutive samples (atan2(y2 - y1, x2 - x1)), never fabricated at
// sample time -- see PedStream.HeadingRad.
public readonly struct PedSample
{
    public PedSample(double timestampSeconds, double x, double y)
    {
        TimestampSeconds = timestampSeconds;
        X = x;
        Y = y;
    }

    public double TimestampSeconds { get; }
    public double X { get; }
    public double Y { get; }
}

// A capacity-bounded, newest-last ring of PedSample -- the ped-side counterpart of
// Sim.Replication.VehicleSampleHistory (same "index 0 == oldest, index Count-1 == newest" contract,
// same drop-oldest-on-overflow behaviour), kept as its own type because peds carry no VehicleRecord
// (lane/upcoming-lanes/DR-model) fields at all.
public sealed class PedSampleHistory
{
    private readonly int _capacity;
    private readonly List<PedSample> _samples;

    public PedSampleHistory(int capacity = 8)
    {
        _capacity = capacity;
        _samples = new List<PedSample>(capacity);
    }

    public void Append(PedSample sample)
    {
        if (_samples.Count >= _capacity)
        {
            _samples.RemoveAt(0);
        }

        _samples.Add(sample);
    }

    public int Count => _samples.Count;

    public PedSample this[int index] => _samples[index];
}

public readonly struct PedSpawnInfo
{
    public PedSpawnInfo(int id, string stringId, Vec2 origin, Vec2 destination)
    {
        Id = id;
        StringId = stringId;
        Origin = origin;
        Destination = destination;
    }

    public int Id { get; }
    public string StringId { get; }
    public Vec2 Origin { get; }
    public Vec2 Destination { get; }
}

public readonly struct PedDespawnInfo
{
    public PedDespawnInfo(int id, string stringId)
    {
        Id = id;
        StringId = stringId;
    }

    public int Id { get; }
    public string StringId { get; }
}

// Synthesizes and drives a DETERMINISTIC pedestrian population over the box net, mirroring
// SceneGen.BuildLiveCity's ped setup (docs/LIVE-CITY-2D-BUILDER-DESIGN.md) but without any of that
// demo's randomness or promotion/crossing-yield machinery -- IgBridge only needs a steadily-populated,
// byte-identical-across-runs walker stream:
//   - Navigation: PedNetworkParser.Load -> WalkablePolygonBaker.Bake -> SumoNavMesh, exactly as
//     BuildLiveCity wires it (same ctor overload, same PedConnections passthrough).
//   - O-D endpoints: sidewalk-segment SPINE MIDPOINTS inside the box (same "spine[mid]" choice
//     BuildLiveCity/BuildDenseCity make), capped and strided the same way if there are more than
//     MaxOdEndpoints.
//   - Population: instead of BuildLiveCity's PedDemand (Poisson spawn-rate + System.Random-seeded
//     lifecycle, docs/PEDESTRIAN-DEMAND-DESIGN.md), this walks the O-D list with a fixed, monotonic
//     integer cursor (`_tripCounter`) -- a pure function of "how many trips have started so far", so
//     two runs that start the same number of trips pick EXACTLY the same O-D pairs in the same order.
//     No System.Random anywhere in this class.
//   - Every walker is a plain PathArc ped (PedestrianWorld.AddWalker); none are ever promoted (no
//     InterestSource is ever registered), so the whole population stays O(1)/step low-power motion.
public sealed class PedStream
{
    // Trips arrive with the geometric fudge PathArcMotion's clamp-at-final-vertex already guarantees
    // (arc length can never exceed path length), so this only needs to cover floating-point rounding
    // of the (length / speed) division -- not a real "grace period".
    private const double ArrivalEpsilon = 1e-6;

    private readonly PedestrianWorld _peds;
    private readonly SumoNavMesh _nav;
    private readonly IReadOnlyList<Vec2> _odPoints;
    private readonly int _populationTarget;
    private readonly double _maxSpeed;
    private readonly double _radius;
    private readonly int _historyCapacity;

    private readonly Dictionary<int, PedSampleHistory> _histories = new();
    private readonly Dictionary<int, double> _arrivalTime = new();
    private readonly HashSet<int> _live = new();
    private readonly List<PedSpawnInfo> _spawnedThisTick = new();
    private readonly List<PedDespawnInfo> _despawnedThisTick = new();

    private int _nextId = 1;
    private int _tripCounter;

    public PedStream(string netXmlPath, IgBridgeConfig config)
    {
        var pedNetwork = PedNetworkParser.Load(netXmlPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        _nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons), pedNetwork.PedConnections);
        _peds = new PedestrianWorld(_nav, config.PedArriveRadius, config.PedDwellSeconds);

        _odPoints = BuildOdPoints(polygons, config.PedMaxOdEndpoints);
        _populationTarget = config.PedPopulationTarget;
        _maxSpeed = config.PedMaxSpeed;
        _radius = config.PedRadius;
        _historyCapacity = config.PedHistoryCapacity;
    }

    public IReadOnlyDictionary<int, PedSampleHistory> Histories => _histories;
    public IReadOnlyCollection<int> LiveIds => _live;
    public IReadOnlyList<PedSpawnInfo> SpawnedThisTick => _spawnedThisTick;
    public IReadOnlyList<PedDespawnInfo> DespawnedThisTick => _despawnedThisTick;

    public static string StringId(int id) => "p" + id.ToString(System.Globalization.CultureInfo.InvariantCulture);

    // Velocity-derived heading (radians): atan2(y2 - y1, x2 - x1) between two consecutive buffered
    // samples. Never stored on PedSample itself -- a caller with only one sample has no heading yet,
    // exactly as the task requires ("do not fabricate a heading source").
    public static double HeadingRad(PedSample from, PedSample to) => Math.Atan2(to.Y - from.Y, to.X - from.X);

    // Sidewalk-segment SPINE MIDPOINTS across the whole box net (the box's net.xml spans a real
    // ~4750x4750 m city, not BuildLiveCity's 840x840 m crop -- see convBoundary in net.xml), then
    // strided down to `maxEndpoints` if there are more (same "even stride" the golden uses for
    // MaxEndpoints -- see BuildLiveCity), same "spine[mid]" point choice as the golden.
    //
    // ORDERING: sorted into a boustrophedon (serpentine row-major) raster over a fixed grid, NOT the
    // polygons' Id order. This is an OD-selection-only concern -- TrySpawnNext walks OUTWARD from an
    // origin's LIST index to find a nearby destination (see MaxTripSearchRadius below), which only
    // produces geographically-close pairs if nearby list indices are geographically close. Across a
    // city this size, an Id-ordered (or otherwise geometry-blind) list would let two "adjacent" indices
    // sit kilometres apart, giving PathArc trips many times longer than this run's 120 s window -- i.e.
    // a population that fills up once and then never cycles (observed: totalPedDespawned=0 over 1200
    // ticks before this fix). The serpentine raster keeps consecutive indices spatially close (each row
    // is walked in alternating direction so a row's last point is near the next row's first), so trips
    // built from nearby indices stay short and the population actually turns over.
    private static IReadOnlyList<Vec2> BuildOdPoints(IReadOnlyList<BakedPolygon> polygons, int maxEndpoints)
    {
        var all = new List<Vec2>();
        foreach (var poly in polygons)
        {
            if (poly.Kind != BakedPolygonKind.SidewalkSegment)
            {
                continue;
            }

            var spine = poly.Spine;
            var pt = spine is { Count: > 0 } ? spine[spine.Count / 2] : poly.Centroid;
            all.Add(pt);
        }

        const double CellSize = 100.0; // metres -- a few city blocks per row
        all.Sort((a, b) =>
        {
            var rowA = Math.Floor(a.Y / CellSize);
            var rowB = Math.Floor(b.Y / CellSize);
            if (rowA != rowB)
            {
                return rowA.CompareTo(rowB);
            }

            // Serpentine: even rows left-to-right, odd rows right-to-left.
            var forward = ((long)rowA % 2L) == 0L;
            var cmpX = a.X.CompareTo(b.X);
            if (!forward)
            {
                cmpX = -cmpX;
            }

            return cmpX != 0 ? cmpX : a.Y.CompareTo(b.Y);
        });

        if (all.Count <= maxEndpoints || maxEndpoints <= 0)
        {
            return all;
        }

        var stride = (double)all.Count / maxEndpoints;
        var picked = new List<Vec2>(maxEndpoints);
        for (var k = 0; k < maxEndpoints; k++)
        {
            picked.Add(all[(int)(k * stride)]);
        }

        return picked;
    }

    // Advances the ped population by exactly one tick, sharing the vehicle engine's (now, dt) clock:
    //   1) top up the population to `_populationTarget` with the next deterministic O-D trip(s);
    //   2) step the low-power LOD population (golden step ORDER: peds step BEFORE the vehicle engine
    //      steps -- see SceneGen.BuildLiveCity's "Step ORDER" comment: `demand.Step(now, dt, ...)`
    //      runs, THEN `engine.Step()`);
    //   3) buffer each live ped's pose at the POST-step time `now + dt` -- the same clock value the
    //      vehicle side timestamps its samples with (IgBridgeRunner.Tick reads `_engine.CurrentTime`
    //      after `Step()`), so ped and vehicle histories line up tick-for-tick;
    //   4) despawn any ped whose PathArc has reached its destination (arc length >= path length) and
    //      let the next tick's top-up (step 1) replace it with a fresh deterministic trip.
    public void Tick(double now, double dt)
    {
        _spawnedThisTick.Clear();
        _despawnedThisTick.Clear();

        while (_live.Count < _populationTarget && TrySpawnNext(now))
        {
        }

        _peds.Step(now, dt);

        var t = now + dt;
        foreach (var id in _live.ToArray())
        {
            var pos = _peds.PositionOf(id, t);
            _histories[id].Append(new PedSample(t, pos.X, pos.Y));

            if (t >= _arrivalTime[id] - ArrivalEpsilon)
            {
                _peds.Remove(id);
                _live.Remove(id);
                _arrivalTime.Remove(id);
                _despawnedThisTick.Add(new PedDespawnInfo(id, StringId(id)));
            }
        }
    }

    // Trips are kept geographically short -- searched outward from the origin's list index (see
    // BuildOdPoints' serpentine ordering) until a candidate at or under this radius is found -- so a
    // PathArc leg's duration (path length / PedMaxSpeed) stays well under this run's 120 s window and
    // the population actually turns over (arrives -> despawns -> the next Tick's top-up respawns it),
    // rather than filling up once with cross-city trips and then sitting static for the rest of the run.
    private const double MaxTripSearchRadius = 250.0; // metres

    // Tries the next deterministic O-D trip: origin = _odPoints[_tripCounter % n] (so consecutive
    // calls walk the origin steadily through the whole endpoint list); destination = the nearest-by-
    // LIST-INDEX candidate (see BuildOdPoints) that is both within MaxTripSearchRadius and routable,
    // widening the search to the whole list only if nothing nearby works. Skips (without registering
    // anything) any candidate the navmesh cannot route between -- e.g. a geometrically disconnected
    // sidewalk island -- and tries the next nearest one in the SAME deterministic order. Returns false
    // only if EVERY other point in the list failed to route from this origin (the origin is isolated);
    // the trip cursor still advances so the next call tries a different origin instead of retrying the
    // same isolated one forever.
    private bool TrySpawnNext(double now)
    {
        var n = _odPoints.Count;
        if (n < 2)
        {
            return false;
        }

        var oi = _tripCounter % n;
        var origin = _odPoints[oi];

        for (var delta = 1; delta < n; delta++)
        {
            var di = (oi + delta) % n;
            var destination = _odPoints[di];

            // Try nearby candidates (within the radius) first; only once every index has been tried
            // does the radius cap stop mattering (delta == n - 1 is the last, farthest-by-index one).
            if ((destination - origin).Abs > MaxTripSearchRadius && delta < n - 1)
            {
                continue;
            }

            var path = _nav.FindPath(origin, destination);
            if (path is null || path.Count < 2)
            {
                continue; // unroutable this pair -- try the next deterministic candidate
            }

            var id = _nextId;
            if (!_peds.AddWalker(id, origin, destination, _maxSpeed, _radius, now))
            {
                continue; // PedestrianWorld re-derives the same path; false here mirrors FindPath's own null
            }

            _nextId++;
            _tripCounter++;

            var duration = PathArcMotion.PathLength(path) / _maxSpeed;
            _arrivalTime[id] = now + duration;
            _live.Add(id);
            _histories[id] = new PedSampleHistory(_historyCapacity);
            _spawnedThisTick.Add(new PedSpawnInfo(id, StringId(id), origin, destination));
            return true;
        }

        _tripCounter++; // this origin has no usable destination at all -- move on, don't retry it forever
        return false;
    }
}
