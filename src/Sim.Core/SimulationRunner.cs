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

    private Thread? _thread;
    private volatile bool _running;
    private volatile bool _paused;

    // >1 runs faster than real time (digital-twin catch-up / training warm-up); <1 slows it down.
    public double SpeedMultiplier { get; set; } = 1.0;

    // Set (and the loop stopped) if a Tick throws on the background thread, so a host/test can observe it.
    public Exception? LastError { get; private set; }

    public SimulationRunner(Engine engine) => _engine = engine;

    // The most recently published frame. Read it fresh each host frame; it is immutable, so a retained
    // reference stays valid (a newer frame is published as a new object).
    public SimulationSnapshot Snapshot => _published;

    public bool IsRunning => _running;
    public bool IsPaused => _paused;

    // Enqueue a fire-and-forget mutation applied at the next Tick boundary (UpdateObstacle, Despawn,
    // SetDestination, Reroute, a batched obstacle update, ...).
    public void Post(Action<Engine> action)
    {
        ArgumentNullException.ThrowIfNull(action);
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
        ArgumentNullException.ThrowIfNull(func);

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
    public void Tick()
    {
        DrainCommands();
        _engine.Step();
        _published = SimulationSnapshot.Capture(_engine);
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
}
