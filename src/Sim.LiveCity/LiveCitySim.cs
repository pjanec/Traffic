using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Host;
using Sim.Ingest;
using Sim.Pedestrians;
using Sim.Pedestrians.Crossing;
using Sim.Pedestrians.Demand;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Replication;

namespace Sim.LiveCity;

// docs/LIVE-CITY-VIEWERS-DESIGN.md §1: `BuildLiveCity`'s wiring (src/Sim.Viz/SceneGen.cs) turned into a
// real-time, steppable, publish-ready host. Constructs the SAME coupled sim (net parsed twice, navmesh
// baked, CrosswalkSignals, CrossingOccupancySource, PedDemand/PedLodManager, InterestField pocket, Engine
// tuned for the demo's step-length/lanechange/speeddev, CrowdSource = Composite(HighPowerFootprints,
// crossingOccupancy)) and reproduces the reference's exact per-tick order in Step(). `LiveCitySim` does
// not render; it only steps, samples (Sample()), and -- as of this task -- publishes onto the same
// in-memory replication wire the local viewers already consume (CityLib/SimSource.cs, PedSimSource.cs).
public sealed class LiveCitySim : IDisposable
{
    private readonly LiveCityConfig _cfg;
    private readonly double _x0, _y0, _x1, _y1;

    private readonly Engine _engine;
    private readonly VTypeHandle _vtype;
    private readonly List<(string Id, int Lane)> _cropEdges;
    private ulong _rng;

    private readonly PedPublisher _pedPublisher;
    private readonly PedLodManager _manager;
    private readonly PedDemand _demand;
    private readonly InterestField _field;
    // The single ped-ORCA promotion source, re-centred AND re-sized on the live high-realism zone by
    // SetLcRealismZone so peds promote to full ORCA across the WHOLE highlighted zone wherever the viewer
    // looks (Follow/Locked), not only at the static crop-centre crossing. Position is mutated in place
    // (InterestSource doc: "an IG camera frustum carries its bubble with it"); radius is readonly on
    // InterestSource, so a radius change rebuilds the source (Remove + Register) -- the promoted-ped count
    // therefore scales with the zone radius by design (owner: "honor the zone radius, no matter perf").
    private InterestSource _orcaSource;
    private InterestSourceId _orcaSourceId;
    private readonly Sim.Pedestrians.Crossing.CrossingOccupancySource _crossingOccupancy;
    private readonly List<Vec2> _movingLowPowerPositions = new();
    private static readonly WorldDisc[] NoEntities = Array.Empty<WorldDisc>();

    // The car publish wire (mirrors CityLib/SimSource.cs).
    private readonly ReplicationPublisher _vehPublisher = new();
    private readonly InMemoryReplicationBus _vehBus = new();
    private bool _vehGeometryPublished;

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §2.2, -TASKS.md Stage C (C1): an OPTIONAL tee onto a caller-supplied
    // sink (e.g. a RecordingReplicationSink writing a .simrec) -- purely additive, a null sink is the
    // default and costs nothing beyond the two null checks in Step(). A SEPARATE ReplicationPublisher
    // instance (never the live `_vehPublisher` above) drives it: ReplicationPublisher's own per-vehicle
    // lifecycle/adaptive-publish bookkeeping is STATEFUL, so sharing one instance across the live bus and
    // the record sink would make the second PublishStep call each tick see "already known" vehicles the
    // first call just announced, silently dropping spawn/despawn events from the recording. LiveCitySim
    // owns no file handle here and never disposes `_recordVehSink` -- the caller (RunLiveCity) constructs
    // and disposes it, exactly as the design's "LiveCitySim does not know about files" tenet requires.
    private readonly IReplicationSink? _recordVehSink;
    private readonly ReplicationPublisher? _recordPublisher;
    private bool _recordGeometryPublished;

    // The ped publish wire (mirrors CityLib/PedSimSource.cs).
    private readonly PedReplicationPublisher _pedWirePublisher;
    private readonly InMemoryPedReplicationBus _pedBus = new();

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §7, -TASKS.md Stage E (E3): the ped-side twin of `_recordVehSink`/
    // `_recordPublisher` above -- an OPTIONAL tee onto a caller-supplied ped sink (e.g. a
    // DdsPedReplicationSink for the combined cars+peds DDS producer), purely additive (null = unchanged
    // Stage A/C behaviour, the two extra null checks in Step() cost nothing). Exactly like the car tee, a
    // DEDICATED `PedReplicationPublisher` instance (never the live `_pedWirePublisher` above) drives it,
    // with its OWN scheduler/governor/meter -- mirrors the in-mem ped publish's own setup in the
    // constructor below so the record/DDS tee gates/measures its own stream independently of the live
    // in-mem wire (sharing gating state across the two would let one stream's suppression decisions leak
    // into the other's). LiveCitySim owns no DDS participant/file handle here and never disposes
    // `_recordPedSink` -- the caller constructs and disposes it, exactly as the vehicle tee's own remark
    // states.
    private readonly IPedReplicationSink? _recordPedSink;
    private readonly PedReplicationPublisher? _recordPedPublisher;

    private double _now;
    private SimulationSnapshot _lastSnapshot = SimulationSnapshot.Empty;

    public LiveCitySim(LiveCityConfig cfg, IReplicationSink? recordVehSink = null, IPedReplicationSink? recordPedSink = null)
    {
        _cfg = cfg ?? throw new ArgumentNullException(nameof(cfg));
        _recordVehSink = recordVehSink;
        _recordPublisher = recordVehSink is not null ? new ReplicationPublisher() : null;
        _recordPedSink = recordPedSink;
        _x0 = cfg.X0; _y0 = cfg.Y0; _x1 = cfg.X1; _y1 = cfg.Y1;

        // docs/LIVE-CITY-VISUALS-NOTES.md "Shared foundation": load the static world-overlay scene
        // (zones/buildings/pois, all optional) once here so both viewers get it for free off `Scene`
        // instead of each re-parsing the dataset dir's JSON companions themselves.
        Scene = LiveCityScene.Load(cfg.DatasetDir);

        var netPath = Path.Combine(cfg.DatasetDir, "net.xml");

        // net parsed twice (once for the vehicle-side NetworkModel, once for the ped-side PedNetwork) --
        // exactly as SceneGen.BuildLiveCity does; the two readers own disjoint models.
        var model = NetworkParser.Parse(netPath);
        Network = model;
        LocalLanes = new NetworkLaneSource(model);

        var pedNetwork = PedNetworkParser.Load(netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons), pedNetwork.PedConnections);

        bool In(double x, double y) => x >= _x0 && x <= _x1 && y >= _y0 && y <= _y1;
        bool InV(Vec2 p) => In(p.X, p.Y);

        var cx = (_x0 + _x1) / 2.0;
        var cy = (_y0 + _y1) / 2.0;

        // Pedestrian O-D endpoints = sidewalk spine midpoints inside the crop.
        var allEndpoints = new List<Vec2>();
        foreach (var poly in polygons)
        {
            if (poly.Kind != BakedPolygonKind.SidewalkSegment) continue;
            if (!InV(poly.Centroid)) continue;
            var spine = poly.Spine;
            var pt = spine is { Count: > 0 } ? spine[spine.Count / 2] : poly.Centroid;
            if (InV(pt)) allEndpoints.Add(pt);
        }

        const int MaxEndpoints = 90;
        var odPoints = new List<Vec2>();
        if (allEndpoints.Count <= MaxEndpoints)
        {
            odPoints.AddRange(allEndpoints);
        }
        else
        {
            var stride = (double)allEndpoints.Count / MaxEndpoints;
            for (var k = 0; k < MaxEndpoints; k++) odPoints.Add(allEndpoints[(int)(k * stride)]);
        }

        // Crop crossings, split by signalization -- SceneGen.BuildLiveCity's Phase 2b split.
        var cropCrossingPolys = new List<BakedPolygon>();
        foreach (var poly in polygons)
        {
            if (poly.Kind == BakedPolygonKind.Crossing && InV(poly.Centroid)) cropCrossingPolys.Add(poly);
        }

        var crosswalkSignals = CrosswalkSignals.FromNet(netPath, cropCrossingPolys);

        // #realism-1 diagnostic: expose each in-crop crossing's centroid + an approximate half-extent (max
        // vertex distance from the centroid), so a car-vs-crossing-ped yield analysis can tell which crossing
        // a ped is on. Same polygons CrossingOccupancySource marks. Read-only; no behavioural effect.
        var crossCentroids = new List<(double X, double Y, double HalfW)>(cropCrossingPolys.Count);
        foreach (var poly in cropCrossingPolys)
        {
            var maxr = 0.0;
            foreach (var vtx in poly.Vertices)
            {
                var dx = vtx.X - poly.Centroid.X; var dy = vtx.Y - poly.Centroid.Y;
                var r = Math.Sqrt((dx * dx) + (dy * dy));
                if (r > maxr) maxr = r;
            }
            crossCentroids.Add((poly.Centroid.X, poly.Centroid.Y, maxr));
        }
        CrossingCentroids = crossCentroids;

        var config = new PedDemandConfig
        {
            Origins = odPoints,
            Destinations = odPoints,
            SpawnRatePerSecond = cfg.PedSpawnRatePerSecond, // LIVECITY_PEDS scales this (default 8.0)
            PopulationCap = cfg.PedPopulationCap,           // LIVECITY_PEDS overrides this (default 160)
            Seed = cfg.PedSeed,
            MaxSpeed = 1.3,
            Radius = 0.3,
            ArrivalRadius = 0.6,
            Liveliness = new PedLivelinessConfig
            {
                PauseProbability = 0.15,
                MinPauseSeconds = 2.0,
                MaxPauseSeconds = 5.0,
                MaxPausesPerTrip = 1,
                PauseAnimTag = "idle",
            },
            EnableWeave = true,
            CrosswalkSignals = cfg.YieldEnabled ? crosswalkSignals : null,
        };

        _pedPublisher = new PedPublisher();
        _manager = new PedLodManager(nav, _pedPublisher, arriveRadius: 0.3, dwellSeconds: 1.0);
        _demand = new PedDemand(config, nav, _manager, startTime: 0.0);

        // The high-realism pocket, anchored on the crop crossing nearest the crop centre (the same
        // "peds actually walk here" anchoring SceneGen.BuildLiveCity uses).
        var pocketCentre = new Vec2(cx, cy);
        var bestD2 = double.PositiveInfinity;
        foreach (var poly in cropCrossingPolys)
        {
            var d2 = (poly.Centroid.X - cx) * (poly.Centroid.X - cx) + (poly.Centroid.Y - cy) * (poly.Centroid.Y - cy);
            if (d2 < bestD2) { bestD2 = d2; pocketCentre = poly.Centroid; }
        }

        const double promoteRadius = 70.0, demoteRadius = 100.0;
        _field = new InterestField();
        _orcaSource = new InterestSource(pocketCentre, promoteRadius, demoteRadius);
        _orcaSourceId = _field.Register(_orcaSource);

        // Expose the high-realism (ORCA-promotion) pocket so a viewer can render it: peds within
        // PromoteRadius of this centre are promoted to full ORCA; beyond DemoteRadius they fall back to
        // low-power dead-reckoning (hysteresis band in between). Centre is in SUMO x/y (world) coords.
        HighRealismPocketX = pocketCentre.X;
        HighRealismPocketY = pocketCentre.Y;
        HighRealismPromoteRadius = promoteRadius;
        HighRealismDemoteRadius = demoteRadius;

        // #15 camera-driven LC-realism zone (docs/LIVE-CITY-CAMERA-REALISM-ZONE-DESIGN.md): the per-area
        // lane-change realism gate starts ON the static pocket, so Central mode == the prior behaviour;
        // a viewer can later move/lock it to the camera via SetLcRealismZone.
        _lcZoneX = pocketCentre.X;
        _lcZoneY = pocketCentre.Y;
        _lcZoneR = promoteRadius;

        _crossingOccupancy = new Sim.Pedestrians.Crossing.CrossingOccupancySource(cropCrossingPolys, pedRadius: cfg.CrossingGateRadius);

        // ---- cars: real Engine on the full net; a dense LOCAL flow on the crop's drivable edges ----
        _engine = new Engine();
        // docs/LIVE-CITY-VISUALS-NOTES.md (tick-rate task): step-length now tracks cfg.Dt instead of the
        // old hardcoded "0.5" literal -- the live-city coupling invariant (car Dt == ped Dt) requires the
        // engine's own resolution to move with LiveCityConfig.Dt, not just the ped publisher (which
        // already read cfg.Dt, see `stepDt: cfg.Dt` below). InvariantCulture is mandatory here: this
        // string is spliced into XML the engine re-parses with double.Parse -- a locale that renders
        // '.' as ',' (ToString() under a non-invariant thread culture) would corrupt the XML attribute
        // (e.g. "0,1" splitting into two malformed tokens), never a locale-dependent path.
        var stepLengthText = cfg.Dt.ToString(CultureInfo.InvariantCulture);
        // SUMO jam escape valve: splice <time-to-teleport> only when the demo enables it (>0); at 0 the
        // parser stores -1 (off), byte-identical to the pre-knob config.
        var teleportXml = cfg.TimeToTeleportSeconds > 0.0
            ? "<time-to-teleport value=\"" + cfg.TimeToTeleportSeconds.ToString(CultureInfo.InvariantCulture) + "\"/>"
            : string.Empty;
        var engineConfig = ScenarioConfigParser.ParseXml(
            "<configuration><time><begin value=\"0\"/><end value=\"1000000000\"/><step-length value=\""
            + stepLengthText + "\"/></time>"
            + "<processing><lanechange.duration value=\"2.0\"/><default.speeddev value=\"0.0\"/>" + teleportXml + "</processing></configuration>");
        _engine.LoadNetwork(netPath, engineConfig);
        _engine.LaneChangeMinSpeed = cfg.LaneChangeMinSpeed;
        // #15 into-occupied: active only under cooperative (high-realism) LC; low realism keeps the cheap
        // tight merge. The engine helper is also caller-gated on CooperativeInformFollower, so this is
        // belt-and-suspenders (0 => the veto is fully inert).
        _engine.MergeStoppedMinGap = cfg.CooperativeLaneChange ? cfg.MergeStoppedMinGap : 0.0;
        _engine.MergeStoppedStrategicDeferDist = cfg.CooperativeLaneChange ? cfg.MergeStoppedStrategicDeferDist : 0.0;
        _engine.JunctionYieldTimeoutSeconds = cfg.JunctionYieldTimeoutSeconds;
        _engine.DeadLaneDriveThrough = cfg.DeadLaneDriveThrough;
        _engine.WrongLaneRerouteAtApproach = cfg.WrongLaneRerouteAtApproach;
        // docs/LIVE-CITY-15-COOPERATIVE-LC-DESIGN.md: cooperative lane change -- both flags gate together,
        // the informFollower is inert unless CoordinatedLaneChange is also on.
        _engine.CoordinatedLaneChange = cfg.CooperativeLaneChange;
        _engine.CooperativeInformFollower = cfg.CooperativeLaneChange;
        _engine.DiagSeqDesync = Environment.GetEnvironmentVariable("LIVECITY_SEQDESYNC") == "1"; // #15 prong-1
        _engine.DiagLaneChangeLog = Environment.GetEnvironmentVariable("LIVECITY_LCLOG") == "1"; // #15 float/swap analysis
        _vtype = _engine.DefineVType(new VTypeParams { VClass = "passenger", Sigma = 0.0 });

        _engine.CrowdSource = cfg.YieldEnabled
            ? new CompositeFootprintSource(_manager.HighPowerFootprints, _crossingOccupancy)
            : _manager.HighPowerFootprints;

        var routeEdges = ReadDrivableEdges(Path.Combine(cfg.DatasetDir, "scenario.rou.xml"));
        _cropEdges = new List<(string Id, int Lane)>();
        foreach (var eid in routeEdges)
        {
            if (!model.EdgesById.TryGetValue(eid, out var edge) || edge.Lanes.Count == 0) continue;
            var carLane = edge.Lanes[^1];
            if (carLane.Shape.Count == 0) continue;
            var mid = carLane.Shape[carLane.Shape.Count / 2];
            if (In(mid.X, mid.Y)) _cropEdges.Add((eid, carLane.Index));
        }

        _rng = cfg.CarRngSeed;

        // ---- publish wires (mirrors CityLib/SimSource.cs + CityLib/PedSimSource.cs) ----
        VehicleSource = _vehBus.Source;

        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var meter = new PedBandwidthMeter();
        var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);
        _pedWirePublisher = new PedReplicationPublisher(_pedBus.Sink, scheduler, governor, meter, stepDt: cfg.Dt);
        PedSource = _pedBus.Source;

        // Stage E (E3) tee: an entirely SEPARATE scheduler/meter/governor triple, wired to the caller's
        // sink -- see `_recordPedSink`'s field remark for why this must not share state with the live
        // in-mem publisher above.
        if (_recordPedSink is not null)
        {
            var recordScheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
            var recordMeter = new PedBandwidthMeter();
            var recordGovernor = new PedBandwidthGovernor(recordScheduler, recordMeter, maxMbitPerSecond: 500.0);
            _recordPedPublisher = new PedReplicationPublisher(
                _recordPedSink, recordScheduler, recordGovernor, recordMeter, stepDt: cfg.Dt);
        }
    }

    public NetworkModel Network { get; }

    // #realism-1: in-crop pedestrian-crossing centroids + approx half-extent (diagnostic; the polys
    // CrossingOccupancySource marks). Used by the car-vs-crossing-ped yield analysis.
    public IReadOnlyList<(double X, double Y, double HalfW)> CrossingCentroids { get; }

    // The static world-overlay scene (zones/buildings/pois) loaded once from cfg.DatasetDir in the ctor.
    public LiveCityScene Scene { get; }

    public NetworkLaneSource LocalLanes { get; }

    public IReplicationSource VehicleSource { get; }

    public IPedReplicationSource PedSource { get; }

    public double Time => _now;

    public int PeakCars { get; private set; }

    public int PeakPeds { get; private set; }

    // Cumulative count of vehicles that finished their route and left the sim (Engine.Events, kind
    // Arrived, tallied each Step). The #15 gridlock signal: in free flow this climbs steadily; under the
    // junction-discharge deadlock it flatlines near zero even while cars are still on the road (cars have
    // destinations but never reach them). Host-side read-only metric only -- never feeds the engine.
    public long ArrivedTotal { get; private set; }

    // DIAGNOSTIC (#15 residual): how many vehicles hit the wrong-lane dead-end clamp in the engine's
    // last step (a turner that could not merge into its turn lane and stranded at the stop line with no
    // onward connection). This is the strand that upstream lane-change cooperation would PREVENT --
    // measuring it against the total stuck-on-green count decides whether that fix is the right lever.
    public int StrandedOffRouteLastStep => _engine.StrandedOffRouteThisStep;

    // #15 diagnostic passthrough: cumulative histogram of WHY wrong-lane cars resolved as they did at a
    // lane end (indices per Engine.StrandReasonHistogram). Read as deltas across samples to see the live
    // mix of recovered-vs-stranded and, among strands, the dominant cause.
    public System.ReadOnlySpan<long> StrandReasonHistogram => _engine.StrandReasonHistogram;

    // #15 float/swap analysis passthrough: committed lane changes by [path][changer-speed] (flattened
    // path*3+spd; path 0 overtake 1 speedGain 2 strategic 3 keepRight; spd 0 stopped<0.5 1 slow<2 2 moving)
    // and, per path, commits where a target-lane car <20m is stopped (swap into an occupied stretch).
    public System.ReadOnlySpan<long> LaneChangeByPathChangerSpeed => _engine.LaneChangeByPathChangerSpeed;
    public System.ReadOnlySpan<long> LaneChangeTargetNearStopped => _engine.LaneChangeTargetNearStopped;
    public System.ReadOnlySpan<long> LaneChangeIntoStoppedDetail => _engine.LaneChangeIntoStoppedDetail;

    // #15 cooperative-LC diagnostic passthrough: cumulative count of SpeedAdvice writes issued from the
    // STRATEGIC informFollower path (Engine.TryStrategicLaneChange). >0 confirms cooperation actually
    // fires; 0 on every parity/bench golden (both underlying Engine flags default false there).
    public long CoopAdviceIssued => _engine.CoopAdviceIssued;

    // DIAGNOSTIC (#15 SUMO cross-check): when non-null, every successful car spawn is appended here
    // (departTime, fromEdge, toEdge) so the exact procedural demand can be exported to a SUMO .rou.xml
    // and run through vanilla SUMO for an apples-to-apples throughput comparison. Null (default) = no
    // recording, no cost.
    public List<(double Depart, string From, string To)>? SpawnLog { get; set; }

    public int OccupiedCrossings => _crossingOccupancy.OccupiedCount;

    public int PeakOccupiedCrossings { get; private set; }

    public int CarYieldObservations { get; private set; }

    // The high-realism (ORCA-promotion) InterestField pocket, for viewers to render (SUMO world coords).
    public double HighRealismPocketX { get; private set; }
    public double HighRealismPocketY { get; private set; }
    public double HighRealismPromoteRadius { get; private set; }
    public double HighRealismDemoteRadius { get; private set; }

    // #15 camera-driven LC-realism zone (docs/LIVE-CITY-CAMERA-REALISM-ZONE-DESIGN.md). The per-area
    // lane-change realism gate in Step() tests against THIS zone (not the static ped-ORCA pocket above),
    // so the viewer can move it to the camera look-at (Follow) or freeze it (Locked). SUMO world coords;
    // radius <= 0 disables the gate (all cars high realism). Initialised to the static pocket (Central).
    private double _lcZoneX;
    private double _lcZoneY;
    private double _lcZoneR;
    public double LcZoneX => _lcZoneX;
    public double LcZoneY => _lcZoneY;
    public double LcZoneRadius => _lcZoneR;

    // Set the LC-realism zone (the viewer pushes this once per step BEFORE Step(), for Follow/Locked
    // modes). Demo-only: parity/bench drive Engine directly, never LiveCitySim, so goldens never call this
    // and the classification stays byte-identical (Central mode leaves the zone on the static pocket).
    public void SetLcRealismZone(double centreX, double centreY, double radius)
    {
        _lcZoneX = centreX;
        _lcZoneY = centreY;
        _lcZoneR = radius;

        // Unify ped ORCA with the high-realism zone: peds within the zone promote to full ORCA (turn
        // high-power) wherever the viewer looks. The promote radius follows the zone radius (owner
        // requirement: honor the zone radius regardless of perf), with a proportional demote-radius
        // hysteresis band. Position mutates in place; a radius change rebuilds the (readonly-radius) source.
        var centre = new Vec2(centreX, centreY);
        if (radius > 0.0 && Math.Abs(radius - _orcaSource.PromoteRadius) > 0.5)
        {
            _field.Remove(_orcaSourceId);
            _orcaSource = new InterestSource(centre, radius, radius * 1.3);
            _orcaSourceId = _field.Register(_orcaSource);
        }
        else
        {
            _orcaSource.Position = centre;
        }
    }

    // Deterministic SplitMix64, seeded from LiveCityConfig.CarRngSeed -- identical constants/order to
    // SceneGen.BuildLiveCity's `NextRng`, so two LiveCitySim instances with the same seed spawn the same
    // sequence of cars.
    private uint NextRng()
    {
        _rng += 0x9E3779B97F4A7C15UL;
        var z = _rng;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBUL;
        return (uint)(z ^ (z >> 31));
    }

    // #15 per-area realism LOD (docs/LIVE-CITY-15-PER-AREA-LOD-DESIGN.md): a car at (x,y) is LOW realism for
    // lane changing iff it is strictly OUTSIDE the high-realism pocket (distance from the pocket centre >
    // promoteRadius). A non-positive radius disables the gate (all cars high realism). Pure function of
    // position => deterministic, order-independent; unit-tested directly.
    public static bool IsLowRealismLaneChangePos(double x, double y, double pocketX, double pocketY, double promoteRadius)
    {
        if (promoteRadius <= 0.0)
        {
            return false;
        }

        var dx = x - pocketX;
        var dy = y - pocketY;
        return (dx * dx) + (dy * dy) > promoteRadius * promoteRadius;
    }

    // Advances the coupled sim by one tick (Dt seconds, per LiveCityConfig.Dt), then publishes the
    // resulting frame onto both wires. Reproduces SceneGen.BuildLiveCity's per-tick order exactly:
    // (a) spawn cars up to the cap on crop drivable edges -> (b) step the ped demand -> (c) gather this
    // tick's WALKING low-power ped positions -> (d) refresh the crossing-occupancy gate -> (e) step the
    // engine (which queries the now-current CrowdSource).
    public void Step()
    {
        var dt = _cfg.Dt;

        // (a) spawn cars up to the cap on crop drivable edges.
        if (_cropEdges.Count >= 2)
        {
            var live = _engine.VehicleHandles.Length;
            for (var s = 0; s < _cfg.CarSpawnPerStep && live < _cfg.CarTargetConcurrent; s++)
            {
                var (fromId, _) = _cropEdges[(int)(NextRng() % (uint)_cropEdges.Count)];
                var (toId, _) = _cropEdges[(int)(NextRng() % (uint)_cropEdges.Count)];
                if (fromId == toId) continue;
                try
                {
                    _engine.SpawnVehicle(_vtype, fromId, toId, departPos: 5.0, departSpeed: 0.0, departBestLane: true);
                    SpawnLog?.Add((_now, fromId, toId));
                    live++;
                }
                catch (InvalidOperationException)
                {
                }
            }
        }

        // (b) step the ped demand; capture the wire-event cursor first so the batch published this tick
        // includes exactly what this Step call emits (mirrors PedSimSource.Tick).
        var beforeCount = _pedPublisher.Events.Count;
        _demand.Step(_now, dt, _field, NoEntities);
        var tNext = _now + dt;

        // (c) gather this tick's ped positions for the crossing-occupancy gate. The occupancy source's
        // point-in-polygon test restricts discs to peds actually ON a crossing, so this over-feeds cheaply
        // (one bbox reject per off-crossing ped) and the gate stays correct. Two realism #1/#2 fixes:
        //  (A) include peds PAUSED on a crossing (AnimTag != Walk), not just walking ones -- a ped standing on
        //      a crosswalk is more reason to yield, not less.
        //  (ORCA) ALSO feed high-power (promoted/ORCA) peds. They gate cars via HighPowerFootprints too, but
        //      that disc is the ped's 0.3 m PHYSICS radius, which only enters the car's ~1.2 m wheel corridor
        //      when the ped is nearly in front -> the car noses over an ORCA ped on a crossing (rampant at
        //      high density). Feeding them here gives an ORCA ped on a crossing the same wide CrossingGateRadius
        //      gate disc as a low-power one, so the car brakes for it in time. Off-crossing ORCA peds still
        //      only carry their footprint (no gate disc), so free-road ORCA avoidance is unchanged.
        _movingLowPowerPositions.Clear();
        foreach (var id in _demand.LiveIds)
        {
            var isOrca = _manager.ModelOf(id) == PedDrModel.FreeKinematic;
            if (isOrca)
            {
                if (!_cfg.GateOrcaPedsOnCrossing) continue;   // ORCA peds gate via footprint only
            }
            else if (!_cfg.GatePausedPedsOnCrossing
                && _manager.AnimTagOf(id, tNext) != ActivityTimeline.WalkAnimTag)
            {
                continue;   // stock behaviour: walking low-power peds only
            }

            _movingLowPowerPositions.Add(_manager.PositionOf(id, tNext));
        }

        // (d) refresh the crossing-occupancy gate from the current walking peds.
        _crossingOccupancy.Update(_movingLowPowerPositions);
        if (_crossingOccupancy.OccupiedCount > PeakOccupiedCrossings) PeakOccupiedCrossings = _crossingOccupancy.OccupiedCount;

        // (d2) #15 per-area realism LOD (docs/LIVE-CITY-15-PER-AREA-LOD-DESIGN.md): classify each live car's
        // lane-change realism from its PREVIOUS-step position vs the static high-realism pocket, BEFORE the
        // engine steps. Only under cooperative LC (otherwise the global cheap-swap path already applies to
        // all). A car inside the pocket cooperates (no pure-lateral float, into-occupied vetoes on); a car
        // outside takes the cheap flow-preserving swap (float permitted -- distant/unobserved). Cars not yet
        // in the previous snapshot (spawned this step) stay high-realism (cooperative) by default. Pure
        // function of the frozen previous snapshot + the static pocket => deterministic, order-independent.
        // Never runs on a golden (parity/bench drive Engine directly, not LiveCitySim) => flag stays false.
        if (_cfg.CooperativeLaneChange && _lcZoneR > 0.0)
        {
            for (var i = 0; i < _lastSnapshot.Count; i++)
            {
                var low = IsLowRealismLaneChangePos(
                    _lastSnapshot.PosX[i], _lastSnapshot.PosY[i],
                    _lcZoneX, _lcZoneY, _lcZoneR);
                _engine.SetLowRealismLaneChange(_lastSnapshot.Handles[i], low);
            }
        }

        // (e) step the engine -- its CrowdSource query now sees the current gates + promoted peds.
        _engine.Step();
        _now = tNext;

        // Tally trip completions this step (Engine.Events is fresh each Step) -- the #15 arrival signal.
        foreach (var ev in _engine.Events)
        {
            if (ev.Kind == SimEventKind.Arrived) ArrivedTotal++;
        }

        if (_engine.VehicleHandles.Length > PeakCars) PeakCars = _engine.VehicleHandles.Length;
        if (_demand.LiveCount > PeakPeds) PeakPeds = _demand.LiveCount;

        // Car-yield metric: for each occupied crossing disc, count it once if any car within 10 m has
        // Speed < 2.0 m/s -- a car braking beside a ped-occupied crossing.
        CarYieldObservations += CountYieldObservationsThisStep();

        // ---- publish: capture the engine snapshot, then publish both wires ----
        var snap = SimulationSnapshot.Capture(_engine);
        _lastSnapshot = snap;

        if (!_vehGeometryPublished)
        {
            _vehPublisher.PublishGeometryOnce(Network, _vehBus.Sink);
            _vehGeometryPublished = true;
        }

        _vehPublisher.PublishStep(snap, _vehBus.Sink);
        _vehBus.Source.Pump();

        // Stage C (C1) tee: also publish this step onto the record sink, if one was supplied -- geometry
        // once (its own publish-once latch, independent of `_vehGeometryPublished` above), then the frame,
        // through the DEDICATED `_recordPublisher` (see its field comment for why it must not be shared).
        if (_recordVehSink is not null && _recordPublisher is not null)
        {
            if (!_recordGeometryPublished)
            {
                _recordPublisher.PublishGeometryOnce(Network, _recordVehSink);
                _recordGeometryPublished = true;
            }

            _recordPublisher.PublishStep(snap, _recordVehSink);
        }

        var newEvents = new List<PedEvent>(_pedPublisher.Events.Count - beforeCount);
        for (var e = beforeCount; e < _pedPublisher.Events.Count; e++)
        {
            newEvents.Add(_pedPublisher.Events[e]);
        }

        _pedWirePublisher.Publish(newEvents);

        // Stage E (E3) tee: also publish this tick's ped event batch through the DEDICATED
        // `_recordPedPublisher`, if a ped record/DDS sink was supplied -- mirrors the car tee just above.
        _recordPedPublisher?.Publish(newEvents);
    }

    private readonly WorldDisc[] _gateProbeScratch = new WorldDisc[4];

    // #realism-1 diagnostic: is world point (x,y) currently a marked occupied-crossing gate disc (i.e. is a
    // ped there being fed to the vehicle CrowdSource this tick)? A tiny-radius self-query returns >=1 iff that
    // exact point was gated. Read-only; used by the car-vs-crossing-ped yield trace to tell whether a nosed-
    // over ped was actually in the feed (corridor-gate miss) or absent from it (feed gap).
    public bool IsOccupancyMarkedAt(double x, double y, double radius = 0.5)
        => _crossingOccupancy.QueryNear(x, y, radius, _gateProbeScratch) > 0;

    // #realism-1 diagnostic: is world point (x,y) actually INSIDE a crossing polygon (independent of the
    // occupancy feed)? Distinguishes a ped genuinely on a crosswalk from one merely near a junction.
    public bool IsOnCrossingPolygon(double x, double y) => _crossingOccupancy.IsInsideAnyCrossing(x, y);

    // #realism-1 density diagnostic: how many crowd discs (ORCA footprints + crossing-occupancy gates) sit
    // within `radius` of (x,y)? The engine's CrowdLongitudinalConstraint reads at most 16 into a stackalloc
    // span, so if the TRUE count here exceeds 16 the in-path disc can be truncated away -> the car never
    // brakes for it. Returns (highPowerFootprints, occupancyGates); sum > 16 flags the truncation risk.
    public (int High, int Occ) CrowdDiscCountsNear(double x, double y, double radius)
    {
        Span<WorldDisc> buf = stackalloc WorldDisc[512];
        var high = _manager.HighPowerFootprints.QueryNear(x, y, radius, buf);
        var occ = _crossingOccupancy.QueryNear(x, y, radius, buf);
        return (high, occ);
    }

    // Increment once per occupied crossing disc that has at least one car within 10 m braking (Speed <
    // 2.0 m/s) beside it -- the "car stopped for a ped on a crosswalk" proxy. A ped's own moving-low-power
    // position is confirmed to actually BE an occupied-crossing gate disc via crossingOccupancy's public
    // QueryNear (a tiny-radius self-query returns >=1 iff that exact point was gated this tick), so this
    // never double-counts a walking ped that is merely near -- not on -- a crossing.
    private int CountYieldObservationsThisStep()
    {
        if (_crossingOccupancy.OccupiedCount == 0) return 0;

        var count = 0;
        var cpx = _engine.PosX;
        var cpy = _engine.PosY;
        var speed = _engine.Speed;
        var carN = cpx.Length;

        foreach (var p in _movingLowPowerPositions)
        {
            // Is this exact ped position an occupied-crossing gate disc (i.e. is the ped ON a crossing)?
            var onCrossing = _crossingOccupancy.QueryNear(p.X, p.Y, 0.01, _gateProbeScratch) > 0;
            if (!onCrossing) continue;

            var near = false;
            for (var i = 0; i < carN; i++)
            {
                var dx = cpx[i] - p.X;
                var dy = cpy[i] - p.Y;
                if ((dx * dx) + (dy * dy) > 100.0) continue; // 10 m radius
                if (speed[i] < 2.0) { near = true; break; }
            }

            if (near) count++;
        }

        return count;
    }

    // Cars-only readback into a REUSED buffer, for callers that need just the vehicle Handle->Name/pose
    // table every frame (e.g. the viewer's click-select name map) and must NOT pay to materialise the whole
    // ped crowd. Sample() below builds a fresh cars+peds snapshot each call -- at a large LIVECITY_PEDS that
    // per-frame ped-list allocation is the dominant GC pressure (measured), so this avoids it entirely.
    private readonly List<LiveCityCar> _carSampleScratch = new();
    public IReadOnlyList<LiveCityCar> SampleCars()
    {
        _carSampleScratch.Clear();
        for (var i = 0; i < _lastSnapshot.Count; i++)
        {
            var x = _lastSnapshot.PosX[i];
            var y = _lastSnapshot.PosY[i];
            if (x < _x0 || x > _x1 || y < _y0 || y > _y1) continue;
            _carSampleScratch.Add(new LiveCityCar(
                _lastSnapshot.Handles[i], x, y, _lastSnapshot.PosZ[i], _lastSnapshot.Angle[i],
                _lastSnapshot.Length[i], _lastSnapshot.Width[i], _lastSnapshot.VehicleId[i]));
        }

        return _carSampleScratch;
    }

    // issue #15 residual chase (docs/LIVE-CITY-15-RESIDUAL-REPRO.md): an ENGINE-AUTHORITATIVE per-vehicle
    // witness for confirming the turn-lane-segregation hypothesis -- LiveCityCar carries no lane/pos/posLat/
    // speed/TL, so this reaches straight into the live Engine's read columns. Diagnostic accessor only
    // (host-side, read-only, never mutates the engine -> parity-untouched); the smoke witness gates it on
    // an env flag so normal runs pay nothing. GapAhead = longitudinal distance to the nearest same-lane
    // car ahead (PositiveInfinity if none); Tl = the controlling TL link's state char for the car's lane
    // ('\0' if the lane is not TL-controlled).
    // Tl = the "any-green wins" summary char for the lane; TlLinks = the DISTINCT states of every TL link
    // controlling this lane (e.g. "Gr" == one movement green, another red -> a car held by its own red
    // turn-arrow under a lane that reads green for a different movement). NextMouthGap = pos of the nearest
    // car on this car's NEXT lane (across the junction) measured from that lane's start (+inf if the exit
    // lane is empty or unknown) -- a small value means the junction EXIT is occupied at its mouth, so the
    // car holds even though its OWN lane is clear ahead (keep-clear / cross-junction car-following, which
    // the same-lane GapAhead cannot see).
    // TlWire = the state char the VIEWER actually renders for this car's lane, read from the published
    // wire (VehicleSource.TlStateByLane) rather than the engine -- so `Tl != TlWire` means the rendered
    // signal head disagrees with the engine's authoritative phase (a "stopped under a green-rendered
    // head while the engine has it red" render bug).
    public readonly record struct CarAuthWitness(
        VehicleHandle Handle, string LaneId, double Pos, double PosLat, double Speed, char Tl, double GapAhead,
        string TlLinks, double NextMouthGap, char TlWire, byte Binder, byte JyArm, float JyFoeSpeed);

    public IReadOnlyList<CarAuthWitness> WitnessAuthoritative()
    {
        var handles = _engine.VehicleHandles;
        var laneH = _engine.LaneHandles;
        var laneIds = _engine.LaneIds;
        var pos = _engine.Pos;
        var posLat = _engine.PosLat;
        var speed = _engine.Speed;
        var tlLaneH = _engine.TlLaneHandles;
        var tlStates = _engine.TlStates;
        var nextLaneH = _engine.NextLaneHandles;
        var binders = _engine.BindingConstraints;   // which speed constraint bound each car
        var jyArms = _engine.JunctionYieldArms;      // which junction-yield arm bound (+0x80 priority)
        var jyFoeSpd = _engine.JunctionYieldFoeSpeeds; // bound junction foe's speed (-1 none)
        var wireTl = _vehBus.Source.TlStateByLane; // what the viewer renders
        var n = handles.Length;

        var outList = new List<CarAuthWitness>(n);
        for (var i = 0; i < n; i++)
        {
            // GapAhead: nearest same-lane car with a greater longitudinal pos.
            var gap = double.PositiveInfinity;
            for (var j = 0; j < n; j++)
            {
                if (j == i || laneH[j] != laneH[i]) continue;
                var d = pos[j] - pos[i];
                if (d > 0.0 && d < gap) gap = d;
            }

            // TL for the car's lane: `tl` = any-green-wins summary; `tlLinks` = the distinct states of
            // every link controlling this lane (so a car held by its OWN movement's red under a lane that
            // is green for another movement is visible as e.g. "Gr").
            var tl = '\0';
            var links = string.Empty;
            for (var k = 0; k < tlLaneH.Length; k++)
            {
                if (tlLaneH[k] != laneH[i]) continue;
                var c = (char)tlStates[k];
                if (links.IndexOf(c) < 0) links += c;
                if ((c is 'G' or 'g') && tl is not ('G' or 'g')) tl = c;
                else if (tl == '\0') tl = c;
            }

            // NextMouthGap: nearest car on the car's NEXT lane (across the junction), measured from that
            // lane's start -- a small value => the exit is occupied at its mouth (keep-clear / cross-
            // junction leader), which the same-lane GapAhead misses.
            var nextMouthGap = double.PositiveInfinity;
            var nl = i < nextLaneH.Length ? nextLaneH[i] : -1;
            if (nl >= 0)
            {
                for (var j = 0; j < n; j++)
                {
                    if (laneH[j] != nl) continue;
                    if (pos[j] < nextMouthGap) nextMouthGap = pos[j];
                }
            }

            var tlWire = wireTl.TryGetValue(laneH[i], out var wb) ? (char)wb : '\0';

            outList.Add(new CarAuthWitness(
                handles[i], i < laneIds.Length ? laneIds[i] : string.Empty,
                pos[i], posLat[i], speed[i], tl, gap, links, nextMouthGap, tlWire,
                i < binders.Length ? binders[i] : (byte)0,
                i < jyArms.Length ? jyArms[i] : (byte)0,
                i < jyFoeSpd.Length ? jyFoeSpd[i] : -1f));
        }

        return outList;
    }

    // Reads back one frame of the coupled scene: cars from the last captured snapshot (crop-filtered),
    // peds from the demand's live ids (crop-filtered), and the crossing-occupancy peak.
    public LiveCitySnapshot Sample()
    {
        var cars = new List<LiveCityCar>(_lastSnapshot.Count);
        for (var i = 0; i < _lastSnapshot.Count; i++)
        {
            var x = _lastSnapshot.PosX[i];
            var y = _lastSnapshot.PosY[i];
            if (x < _x0 || x > _x1 || y < _y0 || y > _y1) continue;
            cars.Add(new LiveCityCar(
                _lastSnapshot.Handles[i], x, y, _lastSnapshot.PosZ[i], _lastSnapshot.Angle[i],
                _lastSnapshot.Length[i], _lastSnapshot.Width[i], _lastSnapshot.VehicleId[i]));
        }

        var peds = new List<LiveCityPed>(_demand.LiveCount);
        foreach (var id in _demand.LiveIds)
        {
            var p = _manager.PositionOf(id, _now);
            if (p.X < _x0 || p.X > _x1 || p.Y < _y0 || p.Y > _y1) continue;
            var model = _manager.ModelOf(id);
            var animTag = _manager.AnimTagOf(id, _now);
            var regime = model == PedDrModel.FreeKinematic ? PedRegime.HighPower
                : animTag == ActivityTimeline.WalkAnimTag ? PedRegime.LowPowerWalking
                : PedRegime.Paused;
            peds.Add(new LiveCityPed(id, p.X, p.Y, 0.0, regime, animTag));
        }

        return new LiveCitySnapshot(cars, peds, _crossingOccupancy.OccupiedCount);
    }

    // Sample pedestrians at an ARBITRARY (continuous) time `t`, not just the current sim step. Ped DR is a
    // pure function of time (PathArc / ActivityTimeline), so this yields the smooth intermediate pose the
    // interactive viewers render between sim ticks. Used by the faithful HTML replay exporter to emit
    // dense (render-rate) ped frames aligned with the DR-reconstructed vehicle frames -- same as the
    // viewers show. `t` should be within ~one step of the latest `Step()` (the arc it interpolates along).
    public void SamplePedsAt(double t, List<LiveCityPed> into)
    {
        into.Clear();
        foreach (var id in _demand.LiveIds)
        {
            var p = _manager.PositionOf(id, t);
            if (p.X < _x0 || p.X > _x1 || p.Y < _y0 || p.Y > _y1) continue;
            var model = _manager.ModelOf(id);
            var animTag = _manager.AnimTagOf(id, t);
            var regime = model == PedDrModel.FreeKinematic ? PedRegime.HighPower
                : animTag == ActivityTimeline.WalkAnimTag ? PedRegime.LowPowerWalking
                : PedRegime.Paused;
            into.Add(new LiveCityPed(id, p.X, p.Y, 0.0, regime, animTag));
        }
    }

    // Read the union of drivable edge ids from a committed car route file (every `edges="..."` token).
    // Copied from SceneGen.ReadDrivableEdges.
    private static IReadOnlyList<string> ReadDrivableEdges(string rouPath)
    {
        var edges = new List<string>();
        var seen = new HashSet<string>(StringComparer.Ordinal);
        if (!File.Exists(rouPath)) return edges;
        foreach (Match m in Regex.Matches(File.ReadAllText(rouPath), "edges=\"([^\"]*)\""))
        {
            foreach (var tok in m.Groups[1].Value.Split(' ', StringSplitOptions.RemoveEmptyEntries))
            {
                if (seen.Add(tok)) edges.Add(tok);
            }
        }

        return edges;
    }

    public void Dispose()
    {
        _vehBus.Sink.Dispose();
        _vehBus.Source.Dispose();
        _pedBus.Sink.Dispose();
        _pedBus.Source.Dispose();
    }
}
