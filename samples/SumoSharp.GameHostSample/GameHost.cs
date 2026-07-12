using Sim.Core;

namespace SumoSharp.GameHostSample;

// One vehicle's render state, handed to the game's renderer each frame. Render-facing floats (matches the
// SoA read columns / InterpolatedVehicle -- "where to draw it").
public readonly struct RenderVehicle
{
    public RenderVehicle(VehicleHandle handle, string id, float x, float y, float z, float angle, float speed)
    {
        Handle = handle; Id = id; X = x; Y = y; Z = z; Angle = angle; Speed = speed;
    }

    public VehicleHandle Handle { get; }
    public string Id { get; }
    public float X { get; }
    public float Y { get; }
    public float Z { get; }
    public float Angle { get; }
    public float Speed { get; }
}

// The reusable SumoSharp <-> game-engine integration brain (SUMOSHARP-API.md §3, §7, §9). This class is
// deliberately netstandard2.1-clean, so it is exactly what you would drop into a Unity MonoBehaviour or a
// Godot Node: it wraps an Engine + a SimulationRunner and exposes only game-shaped calls --
//   * Tick()            -> advance the sim one step (call from FixedUpdate / _physics_process).
//   * GetRenderVehicles([renderTime]) -> the current (or render-time-interpolated) vehicle state to draw
//                          (call from Update / _process, which usually runs faster than the sim).
//   * SpawnAmbient(n)    -> inject routable traffic at runtime (dense edge handles + queued insertion).
//   * AddObstacleOnLane  -> drop a blocker cars steer/stop around (the command dispatcher).
// Nothing here touches an edge-id or lane-id string on the per-frame path: ids are resolved to int handles
// ONCE up front (GetEdge / GetLane), the currency of the hot path (D4).
public sealed class GameHost : IDisposable
{
    private readonly Engine _engine;
    private readonly SimulationRunner _runner;
    private readonly VTypeHandle _vType;
    private readonly int[] _normalEdgeHandles;   // dense edge handles for spawnable (non-internal) edges
    private readonly Random _rng;
    private readonly List<RenderVehicle> _renderScratch = new();

    public GameHost(string netPath, int seed = 12345)
    {
        _engine = new Engine();
        _engine.LoadNetwork(netPath);
        _vType = _engine.DefaultVType;
        _rng = new Random(seed);

        // Resolve the spawnable edges to dense handles once (SUMOSHARP-API.md §9). Internal (":"-prefixed)
        // edges are junction interiors, never a trip origin/destination.
        var normal = new List<int>();
        for (var h = 0; h < _engine.EdgeCount; h++)
        {
            if (!_engine.GetEdgeId(h).StartsWith(":", StringComparison.Ordinal))
            {
                normal.Add(h);
            }
        }
        _normalEdgeHandles = normal.ToArray();

        // Drive the engine through the async runner in MANUAL mode (we call Tick() ourselves -> fully
        // deterministic, the reproducible contract). The opt-in snapshot pool keeps the per-frame render
        // read allocation-free in steady state (SUMOSHARP-API.md §7).
        _runner = new SimulationRunner(_engine);
        _runner.EnableSnapshotPool(capacity: 3);
    }

    public int VehicleCount => _runner.Snapshot.Count;
    public double SimTime => _runner.Snapshot.Time;
    public int SpawnableEdgeCount => _normalEdgeHandles.Length;

    // The string id of the `index`-th spawnable edge (for building a lane id like "<edge>_0"). Setup/debug
    // convenience only -- the per-frame path uses the int handles.
    public string GetSpawnableEdgeId(int index) => _engine.GetEdgeId(_normalEdgeHandles[index]);

    // Advance the simulation one step. In a game this is your fixed-timestep callback (FixedUpdate /
    // _physics_process). Commands posted via SpawnAmbient / AddObstacleOnLane are applied at this boundary.
    public void Tick() => _runner.Tick();

    // Inject up to `count` routable trips between random spawnable edges (unroutable pairs are skipped).
    // Uses the dense-edge-handle SpawnVehicle overload + SUMO-parity queued insertion (the vehicle enters
    // the road when a gap opens at its depart position).
    public int SpawnAmbient(int count)
    {
        var spawned = 0;
        for (var i = 0; i < count && _normalEdgeHandles.Length >= 2; i++)
        {
            var from = _normalEdgeHandles[_rng.Next(_normalEdgeHandles.Length)];
            var to = _normalEdgeHandles[_rng.Next(_normalEdgeHandles.Length)];
            if (from == to)
            {
                continue;
            }

            // Route via the command dispatcher (Invoke runs inline in manual mode). Skip unroutable pairs.
            var ok = _runner.Invoke(e =>
            {
                try { e.SpawnVehicle(_vType, from, to); return true; }
                catch (InvalidOperationException) { return false; } // no route between this pair
            });
            if (ok)
            {
                spawned++;
            }
        }

        return spawned;
    }

    // Drop a static blocker on a lane (resolve the lane id to a handle once; the LiveHost sample shows the
    // screen->lane projection that turns a mouse click into this call). Returns false if the lane is unknown.
    public bool AddObstacleOnLane(string laneId, double frontPos, double length = 5.0)
    {
        int laneHandle;
        try { laneHandle = _engine.GetLane(laneId); }
        catch (KeyNotFoundException) { return false; }

        _runner.Invoke(e => e.AddObstacle(laneHandle, frontPos, length));
        return true;
    }

    // The current frame's vehicles to render (read straight off the published SoA snapshot).
    public IReadOnlyList<RenderVehicle> GetRenderVehicles()
    {
        var snap = _runner.Snapshot;
        _renderScratch.Clear();
        for (var i = 0; i < snap.Count; i++)
        {
            _renderScratch.Add(new RenderVehicle(
                snap.Handles[i], snap.VehicleId[i],
                snap.PosX[i], snap.PosY[i], snap.PosZ[i], snap.Angle[i], snap.Speed[i]));
        }

        return _renderScratch;
    }

    // The current frame's vehicles, each interpolated to `renderTime` between the last two published frames
    // (SUMOSHARP-API.md §7). Call this from your render tick (which runs faster than the sim) with your
    // render clock in sim-time seconds, for smooth motion instead of stepping.
    public IReadOnlyList<RenderVehicle> GetRenderVehicles(double renderTime)
    {
        var snap = _runner.Snapshot;
        _renderScratch.Clear();
        for (var i = 0; i < snap.Count; i++)
        {
            var handle = snap.Handles[i];
            if (_runner.TryInterpolateVehicle(handle, renderTime, out var v))
            {
                _renderScratch.Add(new RenderVehicle(
                    handle, snap.VehicleId[i], v.PosX, v.PosY, v.PosZ, v.Angle, v.Speed));
            }
        }

        return _renderScratch;
    }

    public void Dispose() => _runner.Dispose();
}
