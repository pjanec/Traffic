using System.Globalization;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;

namespace Sim.IgBridge;

// Immutable run configuration. Deterministic by construction: the emitted motion is a pure function of
// (net, demand, step length, seed) -- no wall clock, no System.Random (the engine's dawdle RNG is
// per-entity seeded off Engine.Seed).
public sealed class IgBridgeConfig
{
    public IgBridgeConfig(string netXmlPath, string rouXmlPath)
    {
        NetXmlPath = netXmlPath;
        RouXmlPath = rouXmlPath;
    }

    public string NetXmlPath { get; }
    public string RouXmlPath { get; }
    public double StepLength { get; init; } = 0.1;   // 10 Hz core (docs/IGBRIDGE-DECISIONS.md §1 Q4/R1)
    public int Seed { get; init; } = 42;
    public int HistoryCapacity { get; init; } = 8;    // per-vehicle DR ring depth

    // ---- pedestrian stream (PedStream.cs): a deterministic, always-populated PathArc walker crowd
    // synthesized over the same box net, mirroring SceneGen.BuildLiveCity's ped setup minus its
    // System.Random-seeded demand generator -- see PedStream's class remarks for why a monotonic trip
    // cursor replaces that RNG. ----
    // Synthetic ped crowd on/off. Off = a clean vehicle-only stream (e.g. the grid test bed, which has no
    // scenario peds of its own). On = the deterministic PathArc crowd over the net's sidewalks.
    public bool EnablePeds { get; init; } = true;
    public double PedMaxSpeed { get; init; } = 1.3;      // matches BuildLiveCity's PedDemandConfig.MaxSpeed
    public double PedRadius { get; init; } = 0.3;        // matches BuildLiveCity's PedDemandConfig.Radius
    public double PedArriveRadius { get; init; } = 0.3;  // PedestrianWorld ctor default
    public double PedDwellSeconds { get; init; } = 1.0;  // PedestrianWorld ctor default (LOD promotion dwell; inert here -- no InterestSource is ever registered)
    public int PedPopulationTarget { get; init; } = 40;  // steady-state alive count (task target: >= ~30)
    public int PedHistoryCapacity { get; init; } = 8;    // per-ped sample ring depth (mirrors HistoryCapacity)
    public int PedMaxOdEndpoints { get; init; } = 600;   // cap on synthesized sidewalk-midpoint O-D points (box net has ~400)
}

public readonly struct SpawnInfo
{
    public SpawnInfo(VehicleHandle handle, string id, IgEntityModel model)
    {
        Handle = handle;
        Id = id;
        Model = model;
    }

    public VehicleHandle Handle { get; }
    public string Id { get; }
    public IgEntityModel Model { get; }
}

public readonly struct DespawnInfo
{
    public DespawnInfo(VehicleHandle handle, string id)
    {
        Handle = handle;
        Id = id;
    }

    public VehicleHandle Handle { get; }
    public string Id { get; }
}

// The RAW (unsmoothed) engine pose of one vehicle this tick -- SUMO's own x/y/z + navi Angle, straight
// from the SoA spans. This is the "before" baseline the metrics pass (T1.5) reconstructs and compares the
// IgBridge-smoothed stream against: fed to the IG raw, it carries the instant lane changes and
// junction-heading snaps IgBridge exists to remove.
public readonly struct RawVehiclePose
{
    public RawVehiclePose(VehicleHandle handle, string id, double x, double y, double z, float headingDeg)
    {
        Handle = handle;
        Id = id;
        X = x;
        Y = y;
        Z = z;
        HeadingDeg = headingDeg;
    }

    public VehicleHandle Handle { get; }
    public string Id { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    public float HeadingDeg { get; }
}

// Stage [1]/[2] of the pipeline (docs/IGBRIDGE-DECISIONS.md §2): the fixed-10 Hz core loop + per-entity
// ring buffers. Loads the box network demand-less (Sim.Ingest.DemandParser rejects the box demand's
// departPos="base"/parking stops -- see RouteDemand), replays the real routes via explicit-edge
// SpawnVehicle, and each 0.1 s Tick() appends one TimestampedSample per active vehicle to that vehicle's
// VehicleSampleHistory. Vehicle lifecycle (spawn/despawn) is detected by first/last presence in the
// engine's active set and surfaced for the emit stage (T1.2). Reconstruction/emit are NOT here -- this
// stage only produces the buffered sample source DrClock.ResolveAt consumes.
public sealed class IgBridgeRunner
{
    private readonly Engine _engine;
    private readonly VTypeHandle _vtype;
    private readonly IReadOnlyList<RouteDemandEntry> _demand;
    private readonly int _historyCapacity;
    private readonly double _stepLength;

    private readonly Dictionary<VehicleHandle, VehicleSampleHistory> _histories = new();
    private readonly Dictionary<VehicleHandle, string> _idByHandle = new();
    private readonly Dictionary<VehicleHandle, (double Length, double Width)> _dims = new();
    private readonly List<SpawnInfo> _spawnedThisTick = new();
    private readonly List<DespawnInfo> _despawnedThisTick = new();
    private readonly List<RawVehiclePose> _rawPosesThisTick = new();
    private HashSet<VehicleHandle> _live = new();
    private int _cursor;

    // Separate ped lifecycle/reconstruction stack (docs task: peds never touch VehicleSampleHistory/
    // DrClock) -- see PedStream.cs. Stepped in lockstep with the vehicle engine inside Tick().
    private readonly PedStream? _pedStream;
    private static readonly Dictionary<int, PedSampleHistory> EmptyPedHistories = new();

    public IgBridgeRunner(IgBridgeConfig config)
    {
        Network = NetworkParser.Parse(config.NetXmlPath);
        Lanes = new NetworkLaneSource(Network);
        _historyCapacity = config.HistoryCapacity;
        _stepLength = config.StepLength;

        _engine = new Engine { Seed = (ulong)config.Seed };
        // StepLength ONLY comes from the config's <step-length>; teleport OFF and sigma=0 for a clean,
        // deterministic motion source (matches the --live-city golden's demand-less setup).
        var step = config.StepLength.ToString(CultureInfo.InvariantCulture);
        var xml =
            "<configuration>"
            + "<time><begin value=\"0\"/><end value=\"1000000000\"/><step-length value=\"" + step + "\"/></time>"
            + "<processing><time-to-teleport value=\"-1\"/><default.speeddev value=\"0.0\"/></processing>"
            + "</configuration>";
        _engine.LoadNetwork(config.NetXmlPath, ScenarioConfigParser.ParseXml(xml));
        _vtype = _engine.DefineVType(new VTypeParams { VClass = "passenger", Sigma = 0.0 });
        _demand = RouteDemand.Parse(config.RouXmlPath);

        _pedStream = config.EnablePeds ? new PedStream(config.NetXmlPath, config) : null;
    }

    public NetworkModel Network { get; }
    public NetworkLaneSource Lanes { get; }
    public double SimTime => _engine.CurrentTime;
    public int StepCount => _engine.StepCount;
    public int PendingDemand => _demand.Count - _cursor;

    public IReadOnlyDictionary<VehicleHandle, VehicleSampleHistory> VehicleHistories => _histories;
    public IReadOnlyDictionary<VehicleHandle, (double Length, double Width)> VehicleDims => _dims;
    public IReadOnlyCollection<VehicleHandle> LiveVehicles => _live;
    public IReadOnlyList<SpawnInfo> SpawnedThisTick => _spawnedThisTick;
    public IReadOnlyList<DespawnInfo> DespawnedThisTick => _despawnedThisTick;
    public IReadOnlyList<RawVehiclePose> RawVehiclePosesThisTick => _rawPosesThisTick;

    public string IdOf(VehicleHandle handle)
        => _idByHandle.TryGetValue(handle, out var id) ? id : handle.ToString();

    // ---- pedestrian side (PedStream.cs): mirrors the vehicle read API above one-for-one. ----
    public IReadOnlyDictionary<int, PedSampleHistory> PedHistories => _pedStream?.Histories ?? EmptyPedHistories;
    public IReadOnlyCollection<int> LivePeds => _pedStream?.LiveIds ?? Array.Empty<int>();
    public IReadOnlyList<PedSpawnInfo> PedSpawnedThisTick => _pedStream?.SpawnedThisTick ?? Array.Empty<PedSpawnInfo>();
    public IReadOnlyList<PedDespawnInfo> PedDespawnedThisTick => _pedStream?.DespawnedThisTick ?? Array.Empty<PedDespawnInfo>();

    public static string PedIdOf(int id) => PedStream.StringId(id);

    // Advance exactly one fixed 0.1 s tick: spawn everything due, Step() once, append per-vehicle
    // samples, and diff the active set for spawn/despawn lifecycle. Order matches the golden loop
    // (spawn-before-step); the sim clock advances by exactly StepLength (no wall-clock coupling).
    public void Tick()
    {
        _spawnedThisTick.Clear();
        _despawnedThisTick.Clear();
        _rawPosesThisTick.Clear();

        // 1) spawn all vehicles whose depart time has arrived (demand is depart-sorted).
        var now = _engine.CurrentTime;
        while (_cursor < _demand.Count && _demand[_cursor].Depart <= now)
        {
            var entry = _demand[_cursor++];
            VehicleHandle handle;
            try
            {
                handle = _engine.SpawnVehicle(_vtype, entry.Edges, departPos: 0.0, departSpeed: 0.0,
                    departBestLane: true);
            }
            catch (Exception)
            {
                // Unroutable/invalid edge in the demand -> skip (the scenario runs with
                // ignore-route-errors; IgBridge mirrors that leniency rather than aborting the run).
                continue;
            }

            _idByHandle[handle] = "v" + entry.Id;
        }

        // 1b) advance the ped stream FIRST, sharing this tick's (now, dt) clock with the vehicle
        // engine -- golden step ORDER (SceneGen.BuildLiveCity: `demand.Step(now, dt, ...)` runs before
        // `engine.Step()`). PedStream buffers its own samples internally (see PedStream.Tick).
        _pedStream?.Tick(now, _stepLength);

        // 2) one fixed tick.
        _engine.Step();

        // 3) append one sample per ACTIVE vehicle; first appearance == spawn.
        var current = new HashSet<VehicleHandle>();
        var handles = _engine.VehicleHandles;
        var laneHandles = _engine.LaneHandles;
        var pos = _engine.Pos;
        var posLat = _engine.PosLat;
        var speed = _engine.Speed;
        var accel = _engine.Acceleration;
        var drModels = _engine.DrModels;
        var lengths = _engine.VehicleLengths;
        var widths = _engine.VehicleWidths;
        var rawX = _engine.PosX;
        var rawY = _engine.PosY;
        var rawZ = _engine.PosZ;
        var rawAngle = _engine.Angle;
        var t = _engine.CurrentTime;
        Span<int> upcoming = stackalloc int[UpcomingLanes.Count];

        for (var i = 0; i < handles.Length; i++)
        {
            var handle = handles[i];
            current.Add(handle);

            if (!_histories.TryGetValue(handle, out var history))
            {
                history = new VehicleSampleHistory(_historyCapacity);
                _histories[handle] = history;
                _dims[handle] = (lengths[i], widths[i]); // once per vehicle -- for PoseResolver ChordHeading
                _spawnedThisTick.Add(new SpawnInfo(handle, IdOf(handle), IgEntityModel.Car));
            }

            var n = _engine.GetUpcomingLanes(handle, upcoming);
            var record = new VehicleRecord(
                handle, (DrModel)drModels[i], laneHandles[i], pos[i], posLat[i], speed[i], accel[i],
                latSpeed: 0.0, new UpcomingLanes(upcoming[..n]));
            history.Append(new TimestampedSample(t, record));

            // Raw (unsmoothed) engine pose for this tick -- the metrics "before" baseline (T1.5).
            _rawPosesThisTick.Add(new RawVehiclePose(handle, IdOf(handle), rawX[i], rawY[i], rawZ[i], rawAngle[i]));
        }

        // 4) despawn: present last tick, gone now.
        foreach (var handle in _live)
        {
            if (!current.Contains(handle))
            {
                _despawnedThisTick.Add(new DespawnInfo(handle, IdOf(handle)));
            }
        }

        _live = current;
    }
}
