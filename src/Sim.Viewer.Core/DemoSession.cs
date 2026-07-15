namespace Sim.Viewer.Core;

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5: the swappable session the native viewer's render loop
// reads through, so a demo switch never lets the render loop observe a half-built host. Deliberately
// Raylib-free (lives in Sim.Viewer.Core, not Sim.Viewer) -- Camera2D re-fit and the RoadLayerCache
// invalidation are Raylib-side concerns and stay in Program.cs's RunLocal, which reacts to a switch this
// class reports.
//
// Two switch APIs, per §5's "simple lock or a queued top-of-frame latch" guard:
//   * RequestSwitch(entry) + TryApplyPending(...) -- queue-then-apply, safe to call RequestSwitch from
//     ANY point in a frame (e.g. a mid-frame ImGui click in T5's Demos panel) because the actual
//     dispose-old/build-new/swap-in work is deferred to TryApplyPending, which the render loop calls once
//     at the TOP of its next frame, before anything else that frame reads Host -- race-free by ordering,
//     no lock needed for correctness on the single-threaded UI thread (a lock is still used below purely
//     so Host/Current are safe to read from a diagnostic/console thread too).
//   * SwitchTo(entry) -- immediate synchronous switch (RequestSwitch then apply inline) for headless
//     callers with no render loop pumping TryApplyPending each frame (the demo-catalog switch smoke test,
//     `--demo-smoke`).
public sealed class DemoSession : IDisposable
{
    private readonly object _lock = new();
    private EngineHost _host;
    private DemoEntry? _current;
    private DemoEntry? _pending;

    public string RepoRoot { get; }

    public DemoSession(EngineHost initialHost, DemoEntry? initialEntry, string repoRoot)
    {
        _host = initialHost;
        _current = initialEntry;
        RepoRoot = repoRoot;
    }

    // The live host. Not `null` between switches -- TryApplyPending/SwitchTo dispose the OLD host only
    // after the new one is fully built and swapped in, so a reader never sees a torn/disposed host.
    public EngineHost Host
    {
        get { lock (_lock) return _host; }
    }

    // The DemoEntry that produced the current Host, or null for an ad-hoc "(custom)" `--mode local <path>`
    // host that was never built from the catalog (§5) -- T5's Demos panel shows "(custom)" for null.
    public DemoEntry? Current
    {
        get { lock (_lock) return _current; }
    }

    // Queue a switch to `entry` -- wired to the ImGui Demos picker in T5 (a row click calls this). Does
    // NOT build or dispose anything itself; see the class doc for why that's deferred to TryApplyPending.
    public void RequestSwitch(DemoEntry entry)
    {
        lock (_lock)
        {
            _pending = entry;
        }
    }

    // Apply a queued switch, if any. Call this ONCE, at the very top of the render loop's frame (before
    // reading Host/Snapshot/camera for that frame). Builds the new EngineHost, disposes the OLD one, and
    // swaps Current/Host in atomically. Returns true (with the new host + entry) iff a switch was applied
    // this call -- the caller MUST then re-fit its camera to the new host's bounds and invalidate its
    // static road-layer cache (both Raylib-side; see Program.cs's RunLocal), and should reset any
    // render-behind interpolation state that assumed continuity with the OLD host's timeline.
    public bool TryApplyPending(out EngineHost host, out DemoEntry? entry)
    {
        DemoEntry? toApply;
        lock (_lock)
        {
            toApply = _pending;
            _pending = null;
        }

        if (toApply is null)
        {
            host = Host;
            entry = Current;
            return false;
        }

        var newHost = BuildHost(toApply, RepoRoot);
        EngineHost old;
        lock (_lock)
        {
            old = _host;
            _host = newHost;
            _current = toApply;
        }

        old.Dispose();
        host = newHost;
        entry = toApply;
        return true;
    }

    // Immediate synchronous switch (queue then apply inline) for callers with no render loop pumping
    // TryApplyPending -- e.g. a headless switch smoke test. Returns the newly-built host.
    public EngineHost SwitchTo(DemoEntry entry)
    {
        RequestSwitch(entry);
        TryApplyPending(out var host, out _);
        return host;
    }

    // The DemoEntry -> EngineHost factory (§5):
    //   * Scenario -- the resolved net path in that dir; EngineHost auto-detects scenario mode from the
    //     adjacent *.rou.xml/*.sumocfg.
    //   * Sandbox -- the net path, forced into sandbox mode with SandboxFleet as the spawn cap (falling
    //     back to EngineHost's own default of 80 when SandboxFleet is 0, i.e. not raised by the entry).
    //   * Evac -- EngineHost.CreateEvac(EvacKind, repoRoot), which resolves its own net path internally.
    public static EngineHost BuildHost(DemoEntry entry, string repoRoot) => entry.Kind switch
    {
        DemoKind.Evac => EngineHost.CreateEvac(
            entry.EvacKind ?? throw new InvalidOperationException($"Demo '{entry.Name}' has Kind.Evac but no EvacKind."),
            repoRoot),
        DemoKind.Sandbox => new EngineHost(
            ResolveNetPath(Path.Combine(repoRoot, entry.PathRelToRepo)),
            spawnCap: entry.SandboxFleet > 0 ? entry.SandboxFleet : 80,
            forceSandbox: true),
        DemoKind.Scenario => new EngineHost(ResolveNetPath(Path.Combine(repoRoot, entry.PathRelToRepo))),
        _ => throw new ArgumentOutOfRangeException(nameof(entry), entry.Kind, "Unknown DemoKind."),
    };

    // Mirrors Program.cs's local ResolveNetPath helper (accept either a *.net.xml file directly, or a
    // directory containing exactly one) -- duplicated rather than shared because Program.cs's copy is a
    // top-level local function in the Raylib-referencing Sim.Viewer project, and this class must stay
    // Raylib-free in Sim.Viewer.Core.
    private static string ResolveNetPath(string path)
    {
        if (Directory.Exists(path))
        {
            return Directory.EnumerateFiles(path, "*.net.xml").FirstOrDefault()
                ?? throw new FileNotFoundException($"No *.net.xml found in directory '{path}'.");
        }

        return path;
    }

    public void Dispose()
    {
        lock (_lock)
        {
            _host.Dispose();
        }
    }
}
