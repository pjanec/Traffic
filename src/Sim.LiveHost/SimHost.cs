using System.Text.Json;
using Sim.Core;
using Sim.Ingest;

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
    private readonly string[] _normalEdges;
    private readonly Random _rng = new(12345);

    private readonly object _obsLock = new();
    private readonly List<(double X, double Y)> _obstacles = new();

    private Timer? _spawnTimer;

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

        _normalEdges = _network.EdgesById.Keys.Where(e => !e.StartsWith(':')).ToArray();

        NetworkJson = BuildNetworkJson();

        _runner = new SimulationRunner(_engine);
        // Reuse snapshot backing arrays across ticks (SUMOSHARP-API.md §7) -- the per-frame render read is
        // then allocation-free in steady state. The published snapshot stays valid well beyond the brief
        // window BuildFrameJson holds it (capacity-1 ticks at 30 Hz), so the cross-thread read is safe.
        _runner.EnableSnapshotPool(capacity: 3);
        _runner.Start(targetHz: 30.0);

        // Keep traffic flowing by spawning routable trips at runtime (demonstrates SpawnVehicle).
        _spawnTimer = new Timer(_ => SpawnOne(), null, dueTime: 500, period: 900);
    }

    // Current frame: live vehicle state (from the immutable published snapshot) + injected obstacles.
    public string BuildFrameJson()
    {
        var snap = _runner.Snapshot;
        var vehicles = new List<object>(snap.Count);
        for (var i = 0; i < snap.Count; i++)
        {
            vehicles.Add(new
            {
                id = snap.VehicleId[i],
                x = Math.Round(snap.PosX[i], 2),
                y = Math.Round(snap.PosY[i], 2),
                a = Math.Round(snap.Angle[i], 1),
                s = Math.Round(snap.Speed[i], 2),
            });
        }

        object[] obstacles;
        lock (_obsLock)
        {
            obstacles = _obstacles.Select(o => (object)new { x = Math.Round(o.X, 2), y = Math.Round(o.Y, 2) }).ToArray();
        }

        return JsonSerializer.Serialize(new { type = "frame", time = Math.Round(snap.Time, 1), vehicles, obstacles });
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

        _runner.Post(e =>
        {
            try
            {
                e.SpawnVehicle(_vType, from, to, departPos: 0.0, departSpeed: 0.0, departLane: 0);
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

            lanes.Add(new { pts, w = lane.Width, internalLane = lane.Id.StartsWith(':') });
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
