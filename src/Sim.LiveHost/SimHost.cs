using System.Text.Json;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;

namespace Sim.LiveHost;

// SUMOSHARP-API.md §11: the live-demo brain. Owns one Engine driven by a SimulationRunner, the parsed
// network (for drawing + the screen->lane projection), a runtime traffic spawner, and the list of
// injected obstacles. The web layer (Program.cs) only asks it for JSON messages and forwards clicks.
public sealed class SimHost : IDisposable
{
    private readonly Engine _engine;
    private readonly SimulationRunner _runner;
    private readonly NetworkModel _network;
    private readonly VTypeHandle _vType;
    private readonly VTypeHandle _truckType;
    private readonly string[] _normalEdges;
    private readonly Random _rng = new(12345);

    private readonly object _obsLock = new();
    private readonly List<(double X, double Y)> _obstacles = new();

    private Timer? _spawnTimer;

    // §7 adaptive publish rate: the PublishScheduler runs the pluggable policy ONCE per sim step
    // (BuildFrameJson is called ~20x/s but the sim ticks slower, and the client dedupes by step). Predictable
    // steady followers are re-sent only at the slow keep-alive interval; the client keeps dead-reckoning the
    // ones we skip. The per-step frame is cached so repeated sends of the same step return identical bytes
    // (and don't re-run the scheduler). Intervals are scaled to THIS demo's 1 s sim step (the library
    // defaults 0.1/1.0 target a 10 Hz publisher, where they'd defer 10:1): uncertain movers every step (1 s),
    // predictable steady followers only every 3rd step (~1 send / 3 s).
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
        _network = NetworkParser.Parse(netPath);

        _engine = new Engine();
        _engine.LoadNetwork(netPath);
        // §6.3 production render mode: when set (e.g. CornerCutCorrected), the streamed world poses carry
        // the SUMO chord heading + swept-path off-tracking so long vehicles on curves look right. Off by
        // default (SUMO-exact tangent). Set before Start so the engine thread never races it.
        _engine.RenderMode = renderMode;
        _vType = _engine.DefaultVType;
        // A long vehicle so the renderer's swept-path off-tracking ("swing wide") is visible on turns.
        _truckType = _engine.DefineVType(new VTypeParams { VClass = "truck", Length = 12.0 });

        _normalEdges = _network.EdgesById.Keys.Where(e => !e.StartsWith(':')).ToArray();

        NetworkJson = BuildNetworkJson();

        _runner = new SimulationRunner(_engine);
        // Reuse snapshot backing arrays across ticks (SUMOSHARP-API.md §7) -- the per-frame render read is
        // then allocation-free in steady state. The published snapshot stays valid well beyond the brief
        // window BuildFrameJson holds it (capacity-1 ticks at 30 Hz), so the cross-thread read is safe.
        _runner.EnableSnapshotPool(capacity: 3);
        // Deliberately a LOW sim/publish rate (2 Hz) so the browser's lane-relative dead reckoning is the
        // thing producing smooth 60 fps motion between the sparse updates -- the whole point of the demo.
        _runner.Start(targetHz: 2.0);

        // Keep traffic flowing by spawning routable trips at runtime (demonstrates SpawnVehicle).
        _spawnTimer = new Timer(_ => SpawnOne(), null, dueTime: 500, period: 900);
    }

    // Current frame: LANE-RELATIVE dead-reckoning state per vehicle (SUMOSHARP-DEADRECKONING.md §5.1) --
    // NOT world x/y. The browser reconstructs and extrapolates world pose by walking the once-sent lane
    // geometry (see HtmlPage), so it renders smoothly at 60 fps from these sparse updates and follows the
    // real lane curves (no corner-cutting). This is the demo of the dead-reckoning design end to end.
    //   ln = current lane handle, nx = next lane handle (-1 if none), p = arc pos, pl = lat offset,
    //   s = speed (m/s), a = longitudinal accel (m/s^2).
    public string BuildFrameJson()
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

        // `alive` (every current id -- cheap) is the client's liveness/despawn signal; `vehicles` (full DR
        // state) carries only the ids the policy chose to publish this step. Bandwidth is saved on the
        // expensive per-vehicle state, not on the id list.
        var alive = new List<string>(snap.Count);
        var vehicles = new List<object>(snap.Count);
        for (var i = 0; i < snap.Count; i++)
        {
            var id = snap.VehicleId[i];
            alive.Add(id);

            // The DR regime and the mid-manoeuvre bit now come from the engine's OWN columns (the issue
            // #3/#4 seam), not a stand-in: DrModel picks the extrapolator (LaneArc/Stationary for vehicles),
            // Manoeuvring is the adaptive-rate signal that forces full rate during a reactive lateral dodge.
            if (!_scheduler.ShouldPublish(
                    snap.Handles[i], (DrModel)snap.DrModels[i], snap.SpeedExact[i], snap.Accel[i], snap.Time,
                    snap.Manoeuvring[i]))
            {
                continue; // predictable + recently sent -> let the client keep dead-reckoning it
            }

            // The lane window [prev2,prev1,current,next1,next2,next3] for multi-lane DR walks.
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
                l = Math.Round(snap.Length[i], 2),   // body dims for a correctly-sized rectangle
                w = Math.Round(snap.Width[i], 2),
            });
        }

        // Forget vehicles that despawned this step so the scheduler's memory stays O(live vehicles).
        _scheduler.EndStep();

        object[] obstacles;
        lock (_obsLock)
        {
            obstacles = _obstacles.Select(o => (object)new { x = Math.Round(o.X, 2), y = Math.Round(o.Y, 2) }).ToArray();
        }

        // Traffic-light state (SUMOSHARP-DEADRECKONING.md §5.2): the current signal char per controlled
        // approach lane, so the renderer can draw junction signals. `ln` = lane handle, `st` = signal char.
        var tl = new List<object>(snap.TlCount);
        for (var i = 0; i < snap.TlCount; i++)
        {
            tl.Add(new { ln = snap.TlLaneHandle[i], st = ((char)snap.TlState[i]).ToString() });
        }

        // `time` is the sim clock (seconds); the client measures the sim rate from consecutive frames and
        // extrapolates each vehicle by (renderSimTime - itsOwnLastPublishTime). `npub`/`nalive` let the HUD
        // show the live bandwidth saving (state records sent vs vehicles alive).
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

    // A canvas click (already converted to WORLD coordinates by the browser) -> project to the nearest
    // lane and inject a full-lane obstacle there; cars queue behind it. Ignored if the click is far from
    // any lane. Applied on the engine thread via the runner's command dispatcher.
    public void InjectObstacleAtWorld(double wx, double wy)
    {
        if (!TryProjectToLane(wx, wy, out var laneId, out var pos, out var sx, out var sy, out var dist)
            || dist > 15.0)
        {
            return;
        }

        _runner.Invoke(e => e.AddObstacle(e.GetLane(laneId), frontPos: pos, length: 2.0));
        lock (_obsLock)
        {
            _obstacles.Add((sx, sy));
        }
    }

    public void ClearObstacles()
    {
        _runner.Invoke<object?>(e => { e.ClearObstacles(); return null; });
        lock (_obsLock)
        {
            _obstacles.Clear();
        }
    }

    private void SpawnOne()
    {
        if (_normalEdges.Length < 2 || _runner.Snapshot.Count > 80)
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

    // Nearest lane to a world point, plus the along-lane position and the projected point. Mirrors the
    // point-to-polyline projection PolylineGeometry uses, returning the arc-length of the nearest point.
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

        return JsonSerializer.Serialize(new
        {
            type = "network",
            lanes,
            bounds = new { minX, minY, maxX, maxY },
        });
    }

    public void Dispose()
    {
        _spawnTimer?.Dispose();
        _runner.Dispose();
    }
}
