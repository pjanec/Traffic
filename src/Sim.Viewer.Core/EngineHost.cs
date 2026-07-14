using System.Globalization;
using Sim.Core;
using Sim.Ingest;

namespace Sim.Viewer.Core;

// docs/SUMOSHARP-NATIVE-VIEWER.md P0: the render-agnostic engine driver for the native viewer, lifted
// from Sim.LiveHost/SimHost.cs (SUMOSHARP-API.md §11) with the web/JSON/ImGui plumbing stripped out. Owns
// one Engine driven by a SimulationRunner, and the parsed NetworkModel the renderer needs for road
// geometry. Two modes, auto-selected from the input path exactly like SimHost:
//   * SCENARIO mode -- the input dir has a *.rou.xml AND a *.sumocfg (a committed scenario dir):
//     Engine.LoadScenario drives the scenario's OWN demand.
//   * SANDBOX mode -- a bare net.net.xml (no demand): Engine.LoadNetwork + a runtime random-traffic
//     spawner keeps the roads busy so a bare net still shows traffic.
public sealed class EngineHost : IDisposable
{
    private readonly string _netPath;
    private readonly string? _rouPath;
    private readonly string? _cfgPath;
    private readonly bool _scenarioMode;
    private readonly string[] _normalEdges;

    // Native-viewer perf pass (docs/SUMOSHARP-NATIVE-VIEWER-TESTING.md TASK 1): the sandbox random-traffic
    // spawner's concurrency ceiling. Was a hardcoded 80 (a demo-sized fleet); now the `--fleet N` CLI raises
    // it so a large grid net (e.g. scenarios/_bench/city-15000) can be filled to ~10k for the 60 fps pass.
    private readonly int _spawnCap;
    // Per-timer-fire spawn batch: 1 preserves the original demo cadence for small fleets; a large fleet needs
    // a batch so a warm-up (and replenishment of despawned vehicles) reaches the cap in seconds, not minutes.
    private readonly int _spawnBatch;

    // P1: the runner is rebuilt in place on Restart() (SimHost's BuildSim pattern), so every
    // cross-thread read/rebuild of _engine/_runner is guarded by this lock exactly like SimHost.
    private readonly object _lock = new();
    private Engine _engine = null!;
    private SimulationRunner _runner = null!;
    private VTypeHandle _vType;
    private VTypeHandle _truckType;
    private Random _rng = new(12345);

    // P1: injected obstacle world-points, for the renderer's red-X marker -- mirrors SimHost's
    // _obsLock/_obstacles split (obstacle bookkeeping is independent of the engine rebuild lock).
    private readonly object _obsLock = new();
    private readonly List<(double X, double Y)> _obstacles = new();

    private Timer? _spawnTimer;
    private volatile bool _randomTraffic;
    private volatile bool _paused;
    private string? _stepCfgPath; // temp .sumocfg for a non-default sandbox step length (see WriteStepLengthConfig)

    // Two decoupled viewer knobs (native-viewer controls panel):
    //   * _speed  -- playback speed as a multiple of real time (sim-seconds advanced per wall-second).
    //   * _stepLength -- the engine's integration granularity (sim-seconds per Step), SUMO's step-length.
    // The runner ticks at BaseTickHz*SpeedMultiplier wall-ticks/s and advances _stepLength sim-s per tick, so
    //   real-time factor = wallTickRate * _stepLength  and  wallTickRate (== snapshots/s) = _speed/_stepLength.
    // => SpeedMultiplier = _speed / (BaseTickHz * _stepLength). _speed is applied live via SpeedMultiplier (no
    // restart); _stepLength is baked into the loaded config, so changing it rebuilds the sim (see SetStepLength).
    // Both are remembered so a Restart() re-applies them.
    private const double BaseTickHz = 10.0;
    private double _speed = 1.0;
    private double _stepLength = 1.0;

    public NetworkModel Network { get; }

    public double MinX { get; }
    public double MinY { get; }
    public double MaxX { get; }
    public double MaxY { get; }

    public bool ScenarioMode => _scenarioMode;

    public SimulationSnapshot Snapshot
    {
        get
        {
            lock (_lock)
            {
                return _runner.Snapshot;
            }
        }
    }

    // The two latest published frames as an ATOMIC pair (both read under one lock), for render-behind
    // interpolation in the viewer -- reading Snapshot and PreviousSnapshot separately could straddle a tick
    // and pair frames from different moments. Cur is the newest; Prev is the one before (== Cur on the very
    // first frame, before two exist). The pool is sized (EnableSnapshotPool below) so both stay valid for a
    // render frame even if the sim ticks once meanwhile.
    public (SimulationSnapshot Cur, SimulationSnapshot Prev) SnapshotPair
    {
        get
        {
            lock (_lock)
            {
                return (_runner.Snapshot, _runner.PreviousSnapshot);
            }
        }
    }

    // Current state of the runtime random-traffic spawner, for the ImGui checkbox binding.
    public bool RandomTraffic => _randomTraffic;

    // Turn the runtime random-traffic spawner on/off (independent of mode) -- SimHost's SetRandomTraffic.
    public void SetRandomTraffic(bool on) => _randomTraffic = on;

    // Pause/resume the background sim tick (SimulationRunner.Pause/Resume). Remembered so a Restart's fresh
    // runner re-applies it. Rendering keeps running while paused -- the view just holds the last frame.
    public bool IsPaused => _paused;

    public void SetPaused(bool paused)
    {
        _paused = paused;
        lock (_lock)
        {
            if (paused) _runner.Pause();
            else _runner.Resume();
        }
    }

    // Playback speed (x real-time) and sim step length (s), for the UI + diagnostics.
    public double Speed => _speed;
    public double StepLength => _stepLength;

    // Set playback speed (x real-time). Live: only scales SpeedMultiplier, no rebuild. Clamped to a sane
    // positive range so the slider can't stall (0) or ask for an absurd catch-up. NB: the sim may not sustain
    // a high speed at a large vehicle count -- the render clock paces to the ACTUAL delivered rate, so an
    // unsustainable request just plays at what the engine can manage rather than stuttering.
    public void SetSpeed(double speed)
    {
        _speed = Math.Clamp(speed, 0.1, 20.0);
        lock (_lock)
        {
            _runner.SpeedMultiplier = _speed / (BaseTickHz * _stepLength);
        }
    }

    // Set the engine step length (sim-seconds per Step) = SUMO's step-length / the sim's time resolution.
    // Smaller = finer physics + more snapshots/s (smoother, better turns) but proportionally more compute, so
    // an unsustainably small value at a large fleet just lowers the achievable real-time factor. Baked into
    // the loaded config, so this REBUILDS the sim from t=0 (like Restart). Only meaningful in sandbox mode
    // (scenario mode keeps its own .sumocfg step-length). No-op if unchanged.
    public void SetStepLength(double stepLength)
    {
        // Scenario mode's step-length is fixed by its own .sumocfg -- don't override it (and keep _stepLength
        // at the value the speed formula assumes). The resolution knob is a sandbox-only control.
        if (_scenarioMode)
        {
            return;
        }

        var clamped = Math.Clamp(stepLength, 0.05, 1.0);
        if (Math.Abs(clamped - _stepLength) < 1e-9)
        {
            return;
        }

        _stepLength = clamped;
        BuildSim();
    }

    // Thread-safe snapshot of injected obstacle world-points, for the renderer's red-X marker.
    public IReadOnlyList<(double X, double Y)> ObstaclePoints
    {
        get
        {
            lock (_obsLock)
            {
                return _obstacles.ToArray();
            }
        }
    }

    // `spawnCap` is the sandbox random-traffic concurrency ceiling (default 80 = the original demo fleet;
    // `--fleet N` passes a large value for the 10k perf pass). `forceSandbox` ignores an adjacent
    // rou.rou.xml/.sumocfg and drives the net as a random-traffic sandbox even inside a committed scenario
    // dir -- the perf pass points at scenarios/_bench/city-15000 (a large grid) purely for its geometry and
    // fills it with a controllable `--fleet` count rather than replaying that scenario's fixed demand.
    public EngineHost(string netPath, int spawnCap = 80, bool forceSandbox = false, double stepLength = 1.0)
    {
        _netPath = netPath;
        _stepLength = Math.Clamp(stepLength, 0.05, 1.0); // set before BuildSim/front-load so both use it
        Network = NetworkParser.Parse(netPath);
        _normalEdges = Network.EdgesById.Keys.Where(e => !e.StartsWith(':')).ToArray();

        // Scenario mode iff the net sits beside a rou.rou.xml AND a .sumocfg (a committed scenario dir) --
        // same detection SimHost uses. `forceSandbox` overrides this to sandbox regardless (see ctor doc).
        var dir = Path.GetDirectoryName(Path.GetFullPath(netPath));
        if (dir is not null && !forceSandbox)
        {
            _rouPath = Directory.EnumerateFiles(dir, "*.rou.xml").FirstOrDefault();
            _cfgPath = Directory.EnumerateFiles(dir, "*.sumocfg").FirstOrDefault();
        }

        _scenarioMode = _rouPath is not null && _cfgPath is not null;
        _randomTraffic = !_scenarioMode; // sandbox: random traffic is the traffic; scenario: off by default

        _spawnCap = spawnCap;
        // A big fleet fills via bounded per-fire batches rather than one huge burst: each SpawnVehicle routes
        // on the (13k-edge) grid, so draining thousands in a single tick stalls the sim for seconds (the old
        // ~5s warm-up freeze). A capped batch keeps every tick light; the frequent timer (period below) still
        // fills ~10k in ~15s. The default 80-vehicle demo keeps the original one-at-a-time cadence.
        _spawnBatch = spawnCap > 80 ? Math.Clamp(spawnCap / 20, 1, 150) : 1;

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        foreach (var lane in Network.LanesByHandle)
        {
            foreach (var (x, y) in lane.Shape)
            {
                minX = Math.Min(minX, x);
                minY = Math.Min(minY, y);
                maxX = Math.Max(maxX, x);
                maxY = Math.Max(maxY, y);
            }
        }

        MinX = minX;
        MinY = minY;
        MaxX = maxX;
        MaxY = maxY;

        // Build the engine/runner (and front-load the sandbox burst -- BuildSim does that itself now, so a
        // Restart repopulates too, not just the ctor).
        BuildSim();

        // Keep replenishing traffic thereafter (vehicles that reach their destination despawn), and finish
        // the initial fill. Large fleets fire a bounded batch frequently (light ticks, ~15s to fill); the
        // small demo keeps its original one-at-a-time / 900ms cadence.
        var spawnPeriod = _spawnCap > 80 ? 200 : 900;
        _spawnTimer = new Timer(
            _ => { for (var i = 0; i < _spawnBatch; i++) SpawnOne(); },
            null, dueTime: 400, period: spawnPeriod);
    }

    // (Re)build the engine + runner from scratch, at t=0 -- SimHost's BuildSim pattern. Under _lock so
    // no frame/spawn/obstacle-inject races the swap; the old runner is disposed once swapped out.
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
            else if (Math.Abs(_stepLength - 1.0) < 1e-9)
            {
                engine.LoadNetwork(_netPath); // default 1s step -> Engine's own DefaultNetworkConfig
            }
            else
            {
                // Sandbox with a non-default step length: LoadNetwork only reads the sumocfg for its config
                // (timeline/flags), never its net/route refs, so a generated cfg carrying our step-length --
                // every other field pinned to Engine.DefaultNetworkConfig's values -- changes ONLY the sim
                // resolution and nothing else about sandbox behaviour.
                engine.LoadNetwork(_netPath, WriteStepLengthConfig(_stepLength));
            }

            _vType = engine.DefaultVType;
            // A long vehicle so the random spawner can show swept-path off-tracking ("swing wide") on turns.
            _truckType = engine.DefineVType(new VTypeParams { VClass = "truck", Length = 12.0 });

            var runner = new SimulationRunner(engine);
            // Capacity 4 (was 3): the viewer holds BOTH the newest and previous frame across a render frame
            // for interpolation (SnapshotPair), so the pool needs enough spare buffers that a sim tick (or
            // two, at a high sim rate + a slow render frame) can't recycle the buffer we're still reading.
            runner.EnableSnapshotPool(capacity: 4);
            // Local mode has no dead reckoning to smooth over sparse updates, so a higher rate than the web
            // demo's 2 Hz is fine -- the renderer draws the authoritative Snapshot every frame. The live
            // sim-rate slider drives SpeedMultiplier on top of this base (re-applied here so a Restart keeps
            // the user's chosen rate).
            runner.Start(targetHz: BaseTickHz);
            runner.SpeedMultiplier = _speed / (BaseTickHz * _stepLength);
            if (_paused) runner.Pause();

            _engine = engine;
            _runner = runner;
            _rng = new Random(12345);

            old?.Dispose();
        }

        lock (_obsLock)
        {
            _obstacles.Clear();
        }

        // Front-load a burst of random traffic (sandbox only) so the network has traffic from the very first
        // published Snapshot -- on EVERY build, including a Restart (this used to be ctor-only, so a restarted
        // sandbox started empty and refilled only via the slow replenish timer -> "vehicles: 0" for ~30s).
        // Capped (not the whole fleet) so it doesn't route thousands in one tick; the timer tops up the rest.
        // No-op in scenario mode (_randomTraffic false) -- the scenario re-departs its own demand. SpawnOne
        // gates on the cap and Posts each spawn, applied at the next tick.
        if (_randomTraffic)
        {
            var burst = _spawnCap > 80 ? Math.Min(_spawnCap, 500) : 60;
            for (var i = 0; i < burst; i++)
            {
                SpawnOne();
            }
        }
    }

    // Write a minimal .sumocfg carrying `stepLength`, every OTHER field pinned to the exact values in
    // Engine.DefaultNetworkConfig (Begin 0, End 1e9, Euler, teleport off, action-step 0, speeddev 0, seed 42)
    // -- the parser's own defaults differ (end 0, speeddev 0.1), so they must be stated explicitly or sandbox
    // behaviour would change. Reused per instance (overwritten on each step-length change). Returns the path.
    private string WriteStepLengthConfig(double stepLength)
    {
        _stepCfgPath ??= Path.Combine(
            Path.GetTempPath(), $"sumosharp-viewer-{Environment.ProcessId}-{GetHashCode():x}.sumocfg");

        var sl = stepLength.ToString("R", CultureInfo.InvariantCulture);
        File.WriteAllText(_stepCfgPath,
            "<configuration>\n" +
            "  <time>\n" +
            "    <begin value=\"0\"/>\n" +
            "    <end value=\"1000000000\"/>\n" +
            $"    <step-length value=\"{sl}\"/>\n" +
            "  </time>\n" +
            "  <processing>\n" +
            "    <step-method.ballistic value=\"false\"/>\n" +
            "    <time-to-teleport value=\"-1\"/>\n" +
            "    <default.action-step-length value=\"0\"/>\n" +
            "    <default.speeddev value=\"0\"/>\n" +
            "  </processing>\n" +
            "  <random_number>\n" +
            "    <seed value=\"42\"/>\n" +
            "  </random_number>\n" +
            "</configuration>\n");
        return _stepCfgPath;
    }

    // Rebuild the sim from t=0 (re-queues the scenario demand / empties the sandbox). Obstacles cleared, and
    // pause is released -- a restart means "run it again from the start", so it resumes even if paused (a
    // step-length rebuild via SetStepLength keeps pause, since that's not a user "restart").
    public void Restart()
    {
        _paused = false;
        BuildSim();
    }

    // A world-point click (already WORLD coordinates) -> project to the nearest lane and inject a
    // full-lane obstacle; vehicles queue behind it. Ignored if the click is far from any lane. Ported
    // from Sim.LiveHost/SimHost.cs's InjectObstacleAtWorld.
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
            // Post (fire-and-forget), NOT Invoke: Invoke blocks until the next Tick, which never comes while
            // the sim is PAUSED -> it would deadlock the caller (e.g. the remote-command pump) forever. Post
            // just queues the mutation; it applies at the next tick (immediately if running, on resume if
            // paused). The obstacle marker below is optimistic, same as before.
            runner.Post(e => e.AddObstacle(e.GetLane(laneId), frontPos: pos, length: 2.0));
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
            // Post, not Invoke -- see InjectObstacleAtWorld (Invoke deadlocks while paused).
            runner.Post(e => e.ClearObstacles());
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

    // Nearest lane to a world point, plus the along-lane position and the projected point. Ported from
    // Sim.LiveHost/SimHost.cs's TryProjectToLane; Network is parsed once and never mutated across a
    // restart, so this needs no lock.
    private bool TryProjectToLane(double wx, double wy,
        out string laneId, out double pos, out double sx, out double sy, out double dist)
    {
        laneId = string.Empty;
        pos = 0.0;
        sx = 0.0;
        sy = 0.0;
        dist = double.PositiveInfinity;
        var bestD2 = double.PositiveInfinity;

        foreach (var lane in Network.LanesByHandle)
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

    private void SpawnOne()
    {
        if (!_randomTraffic || _normalEdges.Length < 2)
        {
            return;
        }

        lock (_lock)
        {
            // Snapshot is null until the runner's first Tick publishes one; a spawn fired in that window
            // (front-load right after Start, or an early timer fire) must treat the fleet as empty, not NRE.
            var snap = _runner.Snapshot;
            if (snap is not null && snap.Count > _spawnCap)
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

    public void Dispose()
    {
        _spawnTimer?.Dispose();
        lock (_lock)
        {
            _runner?.Dispose();
        }

        if (_stepCfgPath is not null)
        {
            try { File.Delete(_stepCfgPath); } catch { /* temp file, best-effort */ }
        }
    }
}
