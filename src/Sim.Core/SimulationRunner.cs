using System.Diagnostics;

namespace Sim.Core;

// SUMOSHARP-API.md §7: the ASYNC execution wrapper. The Engine stays single-threaded and stepped; this
// runner owns it plus a background thread, and mediates ALL host access through two lock-free-for-the-host
// structures so the host never touches the Engine directly:
//   * host -> engine: a command dispatcher (Post / Invoke), drained at the START of each Tick, FIFO. This
//     is the "apply mutations at the step boundary" contract, so async and stepped produce identical runs.
//   * engine -> host: an immutable SimulationSnapshot, published after each Step and read via `Snapshot`.
//
// Determinism: with a SINGLE command producer, FIFO drain order is deterministic -> same trajectory as a
// plain Step loop. Multiple producers make the merged order timing-dependent (the sim math stays
// deterministic; only WHEN inputs land varies).
//
// Two ways to drive it:
//   * manual: call Tick() yourself (deterministic, used by tests). Invoke() runs inline (no thread).
//   * threaded: Start(hz) runs Tick() on a background thread at a fixed rate (Pause/Resume, SpeedMultiplier).
public sealed class SimulationRunner : IDisposable
{
    private readonly Engine _engine;

    private readonly object _cmdLock = new();
    private List<Action<Engine>> _incoming = new();
    private List<Action<Engine>> _draining = new();

    private volatile SimulationSnapshot _published = SimulationSnapshot.Empty;
    private volatile SimulationSnapshot _previous = SimulationSnapshot.Empty;

    private Thread? _thread;
    private volatile bool _running;
    private volatile bool _paused;

    // Opt-in snapshot pool (SUMOSHARP-API.md §7). Off by default -> Capture allocates a fresh, exact-size,
    // immutable-forever snapshot each Tick (the original contract). When enabled, a ring of reusable
    // backing-array sets is rotated so the big columnar arrays are NOT re-allocated every Tick; only a
    // small wrapper object is. Tightened contract: a pooled snapshot's arrays are recycled after
    // `capacity - 1` further Ticks, so a retained reference is valid for that many frames (the runner's own
    // Snapshot/PreviousSnapshot are always within one Tick, so the interpolation hook is unaffected).
    private SnapshotBuffers[]? _pool;
    private int _poolCursor;

    // >1 runs faster than real time (digital-twin catch-up / training warm-up); <1 slows it down.
    public double SpeedMultiplier { get; set; } = 1.0;

    // Set (and the loop stopped) if a Tick throws on the background thread, so a host/test can observe it.
    public Exception? LastError { get; private set; }

    public SimulationRunner(Engine engine) => _engine = engine;

    // The most recently published frame. Read it fresh each host frame; it is immutable, so a retained
    // reference stays valid (a newer frame is published as a new object).
    public SimulationSnapshot Snapshot => _published;

    // The frame published BEFORE `Snapshot` (SUMOSHARP-API.md §7 interpolation hook). A host rendering
    // faster than the sim ticks lerps between `PreviousSnapshot` and `Snapshot` using their `.Time`
    // stamps, so motion is smooth instead of stepping. `Empty` until at least two frames have published.
    public SimulationSnapshot PreviousSnapshot => _previous;

    public bool IsRunning => _running;
    public bool IsPaused => _paused;
    public bool IsSnapshotPoolEnabled => _pool is not null;

    // Enable the opt-in snapshot pool (SUMOSHARP-API.md §7): reuse `capacity` sets of backing arrays across
    // Ticks instead of allocating them fresh each frame, cutting the per-frame GC pressure of a live render
    // loop. `capacity` must be >= 2 (the pool retains at least the current + previous frame for the
    // interpolation hook); the default 3 leaves a one-frame margin for a host reading slightly behind. Call
    // before Start()/the first Tick(); a no-op if already enabled. Reminder: with the pool on, a snapshot
    // reference is only valid for `capacity - 1` further Ticks, and its columnar arrays may be longer than
    // `Count` (read `[0, Count)` -- the shipped consumers and TryGetVehicle already do).
    public void EnableSnapshotPool(int capacity = 3)
    {
        if (_running)
        {
            throw new InvalidOperationException("Enable the snapshot pool before starting the runner.");
        }

        if (capacity < 2)
        {
            throw new ArgumentOutOfRangeException(
                nameof(capacity), capacity, "Snapshot-pool capacity must be at least 2 (current + previous frame).");
        }

        if (_pool is not null)
        {
            return;
        }

        var pool = new SnapshotBuffers[capacity];
        for (var i = 0; i < capacity; i++)
        {
            pool[i] = new SnapshotBuffers();
        }

        _pool = pool;
        _poolCursor = 0;
    }

    // Enqueue a fire-and-forget mutation applied at the next Tick boundary (UpdateObstacle, Despawn,
    // SetDestination, Reroute, a batched obstacle update, ...).
    public void Post(Action<Engine> action)
    {
        if (action is null) throw new ArgumentNullException(nameof(action));
        lock (_cmdLock)
        {
            _incoming.Add(action);
        }
    }

    // Enqueue and return the result (SpawnVehicle / DefineVType / AddObstacle -- the ops that return a
    // handle). In manual mode (no background thread) it runs inline. In threaded mode it blocks until the
    // engine thread applies it at the next boundary. Do not call while Paused in threaded mode.
    public T Invoke<T>(Func<Engine, T> func)
    {
        if (func is null) throw new ArgumentNullException(nameof(func));

        if (!_running || (_thread is not null && Thread.CurrentThread == _thread))
        {
            return func(_engine); // manual mode, or reentrant call from the engine thread
        }

        T result = default!;
        Exception? error = null;
        using var done = new ManualResetEventSlim(false);
        Post(e =>
        {
            try { result = func(e); }
            catch (Exception ex) { error = ex; }
            finally { done.Set(); }
        });

        done.Wait();
        if (error is not null)
        {
            throw error;
        }

        return result;
    }

    // One unit of work: drain queued commands (boundary), Step the engine, publish an immutable snapshot.
    // The prior published frame is retained as `PreviousSnapshot` for the interpolation hook.
    public void Tick()
    {
        DrainCommands();
        _engine.Step();

        SimulationSnapshot snap;
        if (_pool is { } pool)
        {
            var buf = pool[_poolCursor];
            _poolCursor = (_poolCursor + 1) % pool.Length;
            snap = buf.Fill(_engine);
        }
        else
        {
            snap = SimulationSnapshot.Capture(_engine);
        }

        _previous = _published;
        _published = snap;
    }

    // Render-time blend factor in [0,1] between PreviousSnapshot (0) and Snapshot (1) for a host clock at
    // `renderTime` (same units as SimulationSnapshot.Time -- seconds of sim time). Clamped, and safe before
    // two frames exist (returns 1 -> "just use the latest") and when the two stamps coincide.
    public double InterpolationAlpha(double renderTime)
    {
        var prev = _previous;
        var cur = _published;
        var span = cur.Time - prev.Time;
        if (prev.StepCount == cur.StepCount || span <= 0.0)
        {
            return 1.0; // no distinct earlier frame to blend from
        }

        var a = (renderTime - prev.Time) / span;
        return a < 0.0 ? 0.0 : (a > 1.0 ? 1.0 : a);
    }

    // Interpolate one vehicle's RENDER state between the two published frames at `renderTime`. Returns false
    // if the vehicle is not in the latest frame (it arrived/despawned). If it is only in the latest frame
    // (just departed), its current state is returned unblended. Angle is blended along the shortest arc.
    // Reads the two volatile frames once; a rare torn pair only widens the blend for a single host frame
    // (harmless for rendering) and is guarded by the alpha checks above.
    public bool TryInterpolateVehicle(VehicleHandle handle, double renderTime, out InterpolatedVehicle result)
    {
        var prev = _previous;
        var cur = _published;

        if (!cur.TryGetVehicle(handle, out var now))
        {
            result = default;
            return false;
        }

        var a = (float)InterpolationAlpha(renderTime);
        if (a >= 1.0f || !prev.TryGetVehicle(handle, out var before))
        {
            result = new InterpolatedVehicle(
                handle, now.X, now.Y, now.Z, now.Angle, (float)now.Speed);
            return true;
        }

        result = new InterpolatedVehicle(
            handle,
            Lerp(before.X, now.X, a),
            Lerp(before.Y, now.Y, a),
            Lerp(before.Z, now.Z, a),
            LerpAngleDeg(before.Angle, now.Angle, a),
            Lerp((float)before.Speed, (float)now.Speed, a));
        return true;
    }

    private static float Lerp(float from, float to, float t) => from + (to - from) * t;

    // Interpolate degrees along the shortest arc (so 350deg -> 10deg crosses 0, not the long way round).
    private static float LerpAngleDeg(float from, float to, float t)
    {
        var delta = (to - from) % 360f;
        if (delta > 180f) delta -= 360f;
        else if (delta < -180f) delta += 360f;

        var r = from + delta * t;
        r %= 360f;
        return r < 0f ? r + 360f : r;
    }

    private void DrainCommands()
    {
        lock (_cmdLock)
        {
            (_incoming, _draining) = (_draining, _incoming);
        }

        for (var i = 0; i < _draining.Count; i++)
        {
            _draining[i](_engine);
        }

        _draining.Clear();
    }

    public void Start(double targetHz = 60.0)
    {
        if (_running)
        {
            return;
        }

        LastError = null;
        _running = true;
        _paused = false;
        _thread = new Thread(() => RunLoop(targetHz))
        {
            IsBackground = true,
            Name = "SumoSharp-SimulationRunner",
        };
        _thread.Start();
    }

    private void RunLoop(double targetHz)
    {
        var period = TimeSpan.FromSeconds(1.0 / Math.Max(targetHz, 1e-6));
        var sw = Stopwatch.StartNew();
        var next = sw.Elapsed;

        try
        {
            while (_running)
            {
                if (_paused)
                {
                    Thread.Sleep(1);
                    next = sw.Elapsed;
                    continue;
                }

                Tick();

                // Fixed-rate pacing, scaled by SpeedMultiplier. If we fell behind, resync instead of
                // spiralling (never try to "catch up" an unbounded backlog of ticks).
                var effPeriod = period / Math.Max(SpeedMultiplier, 1e-6);
                next += effPeriod;
                var delay = next - sw.Elapsed;
                if (delay > TimeSpan.Zero)
                {
                    Thread.Sleep(delay);
                }
                else
                {
                    next = sw.Elapsed;
                }
            }
        }
        catch (Exception ex)
        {
            LastError = ex;
            _running = false;
        }
    }

    public void Pause() => _paused = true;

    public void Resume() => _paused = false;

    public void Stop()
    {
        _running = false;
        _thread?.Join(2000);
        _thread = null;
    }

    public void Dispose() => Stop();

    // One reusable set of snapshot backing arrays (SUMOSHARP-API.md §7). Fill() copies the engine's current
    // read state into these arrays -- growing an array only when the vehicle/event count exceeds its current
    // capacity -- and returns a fresh, per-instance-immutable SimulationSnapshot that ALIASES them. So the
    // big columnar arrays survive across Ticks (no per-frame allocation in steady state); only the small
    // wrapper is allocated. Arrays may be longer than Count; the snapshot's Count/EventCount bound the valid
    // region. Reused round-robin by SimulationRunner, so the caller must respect the capacity-frame validity
    // window documented on EnableSnapshotPool.
    private sealed class SnapshotBuffers
    {
        private VehicleHandle[] _handles = Array.Empty<VehicleHandle>();
        private float[] _posX = Array.Empty<float>();
        private float[] _posY = Array.Empty<float>();
        private float[] _posZ = Array.Empty<float>();
        private float[] _angle = Array.Empty<float>();
        private float[] _speed = Array.Empty<float>();
        private float[] _length = Array.Empty<float>();
        private float[] _width = Array.Empty<float>();
        private double[] _speedExact = Array.Empty<double>();
        private double[] _accel = Array.Empty<double>();
        private int[] _laneHandle = Array.Empty<int>();
        private int[] _nextLane = Array.Empty<int>();
        private int[] _prevLane = Array.Empty<int>();
        private int[] _laneWindow = Array.Empty<int>();
        private double[] _pos = Array.Empty<double>();
        private double[] _posLat = Array.Empty<double>();
        private byte[] _drModels = Array.Empty<byte>();
        private bool[] _manoeuvring = Array.Empty<bool>();
        private string[] _vehicleId = Array.Empty<string>();
        private string[] _vehicleType = Array.Empty<string>();
        private string[] _laneId = Array.Empty<string>();
        private SimEvent[] _events = Array.Empty<SimEvent>();
        private int[] _tlLane = Array.Empty<int>();
        private byte[] _tlState = Array.Empty<byte>();

        public SimulationSnapshot Fill(Engine engine)
        {
            var n = engine.VehicleCount;
            Copy(engine.VehicleHandles, ref _handles, n);
            Copy(engine.PosX, ref _posX, n);
            Copy(engine.PosY, ref _posY, n);
            Copy(engine.PosZ, ref _posZ, n);
            Copy(engine.Angle, ref _angle, n);
            Copy(engine.Speed, ref _speed, n);
            Copy(engine.VehicleLengths, ref _length, n);
            Copy(engine.VehicleWidths, ref _width, n);
            Copy(engine.SpeedExactColumn, ref _speedExact, n);
            Copy(engine.Acceleration, ref _accel, n);
            Copy(engine.LaneHandles, ref _laneHandle, n);
            Copy(engine.NextLaneHandles, ref _nextLane, n);
            Copy(engine.PrevLaneHandles, ref _prevLane, n);
            Copy(engine.LaneWindows, ref _laneWindow, n * Engine.LaneWindowStride);
            Copy(engine.Pos, ref _pos, n);
            Copy(engine.PosLat, ref _posLat, n);
            Copy(engine.DrModels, ref _drModels, n);
            Copy(engine.Manoeuvring, ref _manoeuvring, n);
            Copy(engine.VehicleIds, ref _vehicleId, n);
            Copy(engine.VehicleTypes, ref _vehicleType, n);
            Copy(engine.LaneIds, ref _laneId, n);

            var events = engine.Events;
            Copy(events, ref _events, events.Length);

            var tl = engine.TlStates;
            Copy(engine.TlLaneHandles, ref _tlLane, tl.Length);
            Copy(tl, ref _tlState, tl.Length);

            return new SimulationSnapshot
            {
                Count = n,
                Time = engine.CurrentTime,
                StepCount = engine.StepCount,
                Handles = _handles,
                PosX = _posX,
                PosY = _posY,
                PosZ = _posZ,
                Angle = _angle,
                Speed = _speed,
                Length = _length,
                Width = _width,
                SpeedExact = _speedExact,
                Accel = _accel,
                LaneHandle = _laneHandle,
                NextLaneHandle = _nextLane,
                PrevLaneHandle = _prevLane,
                LaneWindow = _laneWindow,
                LaneWindowStride = Engine.LaneWindowStride,
                Pos = _pos,
                PosLat = _posLat,
                DrModels = _drModels,
                Manoeuvring = _manoeuvring,
                VehicleId = _vehicleId,
                VehicleType = _vehicleType,
                LaneId = _laneId,
                Events = _events,
                EventCount = events.Length,
                TlLaneHandle = _tlLane,
                TlState = _tlState,
                TlCount = tl.Length,
            };
        }

        // Copy `count` elements of `src` into `dst`, allocating a new (capacity) array only when `dst` is too
        // small. `dst` may end up longer than `count`; the stale tail is never read (Count bounds it).
        private static void Copy<T>(ReadOnlySpan<T> src, ref T[] dst, int count)
        {
            if (dst.Length < count)
            {
                dst = new T[count];
            }

            src.Slice(0, count).CopyTo(dst);
        }
    }
}
