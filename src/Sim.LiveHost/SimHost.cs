using System.Text.Json;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;

namespace Sim.LiveHost;

// SUMOSHARP-API.md §11: the live-demo brain. Owns one Engine driven by a SimulationRunner, the parsed
// network (for drawing + the screen->lane projection), and the list of injected obstacles. Two modes,
// auto-selected from the input:
//   * SCENARIO mode -- the input dir has a rou.rou.xml (+ .sumocfg): LoadScenario drives the scenario's OWN
//     demand (the exact vehicles that produce the golden trajectory), rendered live with dead reckoning.
//   * SANDBOX mode -- a bare net.net.xml (no demand, e.g. samples/junctions/*): LoadNetwork + a runtime
//     random-traffic spawner keeps the roads busy.
// A "restart" rebuilds the sim from t=0; a "random traffic" toggle turns the spawner on/off in either mode
// (default: on in sandbox, off in scenario). The engine+runner are rebuilt on restart, so they are mutable
// and guarded by _lock; every cross-thread read (frame build, spawn, obstacle) takes that lock.
public sealed class SimHost : IDisposable
{
    private readonly string _netPath;
    private readonly string? _rouPath;
    private readonly string? _cfgPath;
    private readonly bool _scenarioMode;
    private readonly RenderRealism _renderMode;
    private readonly NetworkModel _network;   // parsed once; the net never changes across a restart
    private readonly string[] _normalEdges;

    private readonly object _lock = new();
    private Engine _engine = null!;
    private SimulationRunner _runner = null!;
    private VTypeHandle _vType;
    private VTypeHandle _truckType;
    private Random _rng = new(12345);

    private readonly object _obsLock = new();
    private readonly List<(double X, double Y)> _obstacles = new();

    private Timer? _spawnTimer;
    private volatile bool _randomTraffic;

    // §7 adaptive publish rate: the PublishScheduler runs the pluggable policy ONCE per sim step
    // (BuildFrameJson is called ~20x/s but the sim ticks slower, and the client dedupes by step). The
    // per-step frame is cached so repeated sends of the same step return identical bytes. Reset on restart.
    private readonly PublishScheduler _scheduler = new(new DefaultPublishPolicy
    {
        FastInterval = 1.0,
        SlowInterval = 3.0,
        AccelThreshold = 0.3,
    });
    private int _lastFrameStep = -1;
    private string _lastFrameJson = "{\"type\":\"frame\",\"time\":0,\"step\":-1,\"alive\":[],\"vehicles\":[],\"obstacles\":[],\"tl\":[],\"npub\":0,\"nalive\":0}";

    public string NetworkJson { get; }

    public SimHost(string netPath, RenderRealism renderMode = RenderRealism.ParityTangent)
    {
        _netPath = netPath;
        _renderMode = renderMode;
        _network = NetworkParser.Parse(netPath);
        _normalEdges = _network.EdgesById.Keys.Where(e => !e.StartsWith(':')).ToArray();

        // Scenario mode iff the net sits beside a rou.rou.xml AND a .sumocfg (a committed scenario dir).
        var dir = Path.GetDirectoryName(Path.GetFullPath(netPath));
        if (dir is not null)
        {
            _rouPath = Directory.EnumerateFiles(dir, "*.rou.xml").FirstOrDefault();
            _cfgPath = Directory.EnumerateFiles(dir, "*.sumocfg").FirstOrDefault();
        }

        _scenarioMode = _rouPath is not null && _cfgPath is not null;
        _randomTraffic = !_scenarioMode; // sandbox: random traffic is the traffic; scenario: off by default

        NetworkJson = BuildNetworkJson();

        BuildSim();

        // Always running; SpawnOne is a no-op while _randomTraffic is off.
        _spawnTimer = new Timer(_ => SpawnOne(), null, dueTime: 500, period: 900);
    }

    public bool ScenarioMode => _scenarioMode;

    // (Re)build the engine + runner from scratch, at t=0. Under _lock so no frame/spawn races the swap.
    private void BuildSim()
    {
        lock (_lock)
        {
            var old = _runner;

            var engine = new Engine();
            if (_scenarioMode)
            {
                engine.LoadScenario(_netPath, _rouPath!, _cfgPath!); // drives the scenario's OWN demand
            }
            else
            {
                engine.LoadNetwork(_netPath);
            }

            // §6.3 production render mode: the streamed world poses carry the SUMO chord heading +
            // swept-path off-tracking when set. Off by default (SUMO-exact tangent). Set before Start.
            engine.RenderMode = _renderMode;
            _vType = engine.DefaultVType;
            // A long vehicle so the random spawner can show swept-path off-tracking ("swing wide") on turns.
            _truckType = engine.DefineVType(new VTypeParams { VClass = "truck", Length = 12.0 });

            var runner = new SimulationRunner(engine);
            runner.EnableSnapshotPool(capacity: 3);
            // Deliberately a LOW sim/publish rate (2 Hz) so the browser's lane-relative dead reckoning is
            // what produces smooth 60 fps motion between the sparse updates -- the point of the demo.
            runner.Start(targetHz: 2.0);

            _engine = engine;
            _runner = runner;
            _rng = new Random(12345);
            _scheduler.Reset();
            _lastFrameStep = -1;

            old?.Dispose();
        }

        lock (_obsLock)
        {
            _obstacles.Clear();
        }
    }

    // Rebuild the sim from t=0 (re-queues the scenario demand / empties the sandbox). Obstacles cleared.
    public void Restart() => BuildSim();

    // Turn the runtime random-traffic spawner on/off (independent of mode).
    public void SetRandomTraffic(bool on) => _randomTraffic = on;

    // Current frame: LANE-RELATIVE dead-reckoning state per vehicle (SUMOSHARP-DEADRECKONING.md §5.1). The
    // browser reconstructs + extrapolates world pose by walking the once-sent lane geometry, so it renders
    // smoothly at 60 fps from these sparse updates. Runs the publish policy once per sim step.
    public string BuildFrameJson()
    {
        lock (_lock)
        {
            var snap = _runner.Snapshot;
            // Once per sim step only: repeated 20 Hz sends between ticks return the cached bytes (the client
            // dedupes by step anyway), so the publish policy advances exactly one step per snapshot.
            if (snap.StepCount == _lastFrameStep)
            {
                return _lastFrameJson;
            }

            _lastFrameStep = snap.StepCount;
            var stride = snap.LaneWindowStride;

            // `alive` (every current id -- cheap) is the client's liveness/despawn signal; `vehicles` (full
            // DR state) carries only the ids the policy chose to publish this step.
            var alive = new List<string>(snap.Count);
            var vehicles = new List<object>(snap.Count);
            for (var i = 0; i < snap.Count; i++)
            {
                var id = snap.VehicleId[i];
                alive.Add(id);

                // DR regime + mid-manoeuvre bit from the engine's own columns (issue #3/#4 seam): DrModel
                // picks the extrapolator, Manoeuvring forces full rate during a reactive lateral dodge.
                if (!_scheduler.ShouldPublish(
                        snap.Handles[i], (DrModel)snap.DrModels[i], snap.SpeedExact[i], snap.Accel[i], snap.Time,
                        snap.Manoeuvring[i]))
                {
                    continue; // predictable + recently sent -> let the client keep dead-reckoning it
                }

                var lw = new int[stride];
                Array.Copy(snap.LaneWindow, i * stride, lw, 0, stride);

                vehicles.Add(new
                {
                    id,
                    lw,
                    p = Math.Round(snap.Pos[i], 3),
                    pl = Math.Round(snap.PosLat[i], 3),
                    s = Math.Round(snap.SpeedExact[i], 3),
                    a = Math.Round(snap.Accel[i], 3),
                    l = Math.Round(snap.Length[i], 2),
                    w = Math.Round(snap.Width[i], 2),
                });
            }

            _scheduler.EndStep();

            object[] obstacles;
            lock (_obsLock)
            {
                obstacles = _obstacles.Select(o => (object)new { x = Math.Round(o.X, 2), y = Math.Round(o.Y, 2) }).ToArray();
            }

            var tl = new List<object>(snap.TlCount);
            for (var i = 0; i < snap.TlCount; i++)
            {
                tl.Add(new { ln = snap.TlLaneHandle[i], st = ((char)snap.TlState[i]).ToString() });
            }

            _lastFrameJson = JsonSerializer.Serialize(new
            {
                type = "frame",
                time = Math.Round(snap.Time, 3),
                step = snap.StepCount,
                alive,
                vehicles,
                obstacles,
                tl,
                npub = vehicles.Count,
                nalive = alive.Count,
            });
            return _lastFrameJson;
        }
    }

    // A canvas click (already WORLD coordinates) -> project to the nearest lane and inject a full-lane
    // obstacle; cars queue behind it. Ignored if the click is far from any lane.
    public void InjectObstacleAtWorld(double wx, double wy)
    {
        if (!TryProjectToLane(wx, wy, out var laneId, out var pos, out var sx, out var sy, out var dist)
            || dist > 15.0)
        {
            return;
        }

        SimulationRunner runner;
        lock (_lock)
        {
            runner = _runner;
        }

        try
        {
            runner.Invoke(e => e.AddObstacle(e.GetLane(laneId), frontPos: pos, length: 2.0));
        }
        catch
        {
            return; // runner disposed mid-restart -> drop this click
        }

        lock (_obsLock)
        {
            _obstacles.Add((sx, sy));
        }
    }

    public void ClearObstacles()
    {
        SimulationRunner runner;
        lock (_lock)
        {
            runner = _runner;
        }

        try
        {
            runner.Invoke<object?>(e => { e.ClearObstacles(); return null; });
        }
        catch
        {
            // runner disposed mid-restart -> the rebuild already cleared obstacles
        }

        lock (_obsLock)
        {
            _obstacles.Clear();
        }
    }

    private void SpawnOne()
    {
        if (!_randomTraffic || _normalEdges.Length < 2)
        {
            return;
        }

        lock (_lock)
        {
            if (_runner.Snapshot.Count > 80)
            {
                return;
            }

            var from = _normalEdges[_rng.Next(_normalEdges.Length)];
            var to = _normalEdges[_rng.Next(_normalEdges.Length)];
            if (from == to)
            {
                return;
            }

            var vt = _rng.Next(3) == 0 ? _truckType : _vType; // ~1/3 trucks, to show off-tracking on turns
            _runner.Post(e =>
            {
                try
                {
                    e.SpawnVehicle(vt, from, to, departPos: 0.0, departSpeed: 0.0, departLane: 0);
                }
                catch
                {
                    // No route between this random pair -> skip (the next tick tries a fresh pair).
                }
            });
        }
    }

    // Nearest lane to a world point, plus the along-lane position and the projected point.
    private bool TryProjectToLane(double wx, double wy,
        out string laneId, out double pos, out double sx, out double sy, out double dist)
    {
        laneId = string.Empty;
        pos = 0.0;
        sx = 0.0;
        sy = 0.0;
        dist = double.PositiveInfinity;
        var bestD2 = double.PositiveInfinity;

        foreach (var lane in _network.LanesByHandle)
        {
            var shape = lane.Shape;
            if (shape.Count < 2)
            {
                continue;
            }

            var acc = 0.0;
            for (var i = 0; i < shape.Count - 1; i++)
            {
                var (px, py) = shape[i];
                var (qx, qy) = shape[i + 1];
                var dx = qx - px;
                var dy = qy - py;
                var segLen2 = dx * dx + dy * dy;
                var t = segLen2 > 0 ? Math.Clamp(((wx - px) * dx + (wy - py) * dy) / segLen2, 0.0, 1.0) : 0.0;
                var cx = px + dx * t;
                var cy = py + dy * t;
                var d2 = (wx - cx) * (wx - cx) + (wy - cy) * (wy - cy);
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    laneId = lane.Id;
                    pos = acc + t * Math.Sqrt(segLen2);
                    sx = cx;
                    sy = cy;
                }

                acc += Math.Sqrt(segLen2);
            }
        }

        dist = Math.Sqrt(bestD2);
        return !double.IsPositiveInfinity(bestD2);
    }

    private string BuildNetworkJson()
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;

        var lanes = new List<object>(_network.LanesByHandle.Count);
        foreach (var lane in _network.LanesByHandle)
        {
            var pts = new double[lane.Shape.Count * 2];
            for (var i = 0; i < lane.Shape.Count; i++)
            {
                var (x, y) = lane.Shape[i];
                pts[i * 2] = Math.Round(x, 2);
                pts[i * 2 + 1] = Math.Round(y, 2);
                minX = Math.Min(minX, x);
                minY = Math.Min(minY, y);
                maxX = Math.Max(maxX, x);
                maxY = Math.Max(maxY, y);
            }

            // The array index == lane handle (LanesByHandle is dense, index==Handle), and `len` is the
            // lane's arc length -- both needed by the client's lane-relative dead-reckoning walk.
            lanes.Add(new { pts, len = Math.Round(lane.Length, 3), w = lane.Width, internalLane = lane.Id.StartsWith(':') });
        }

        // `mode` + `randomTraffic` let the client label the run and initialise the traffic-toggle control.
        return JsonSerializer.Serialize(new
        {
            type = "network",
            mode = _scenarioMode ? "scenario" : "sandbox",
            randomTraffic = _randomTraffic,
            lanes,
            bounds = new { minX, minY, maxX, maxY },
        });
    }

    public void Dispose()
    {
        _spawnTimer?.Dispose();
        lock (_lock)
        {
            _runner?.Dispose();
        }
    }
}
