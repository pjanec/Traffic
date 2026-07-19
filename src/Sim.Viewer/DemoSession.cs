using Sim.Viewer.Core;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5: the swappable session the native viewer's render loop
// reads through, so a demo switch never lets the render loop observe a half-built host.
//
// docs/SUMOSHARP-PACKAGING-DESIGN.md D5/D10 (P3.2): this class now lives in Sim.Viewer (the demo tool),
// not Sim.Viewer.Core (the generic viewer brain) -- it is demo-tool wiring (DemoEntry -> host + overlay),
// not part of the packageable seam. It now also carries the IRenderOverlay a Kind.Evac entry's host
// pairs with (built via EngineHost.CreateCustom + EvacOverlay, §G of the packaging refactor), so a
// switch swaps BOTH atomically -- the render loop must never see a host from one demo paired with an
// overlay from another. Camera2D re-fit and the RoadLayerCache invalidation are Raylib-side concerns and
// stay in Program.cs's RunLocal, which reacts to a switch this class reports.
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
    private IRenderOverlay? _overlay;
    private DemoEntry? _current;
    private DemoEntry? _pending;

    public string RepoRoot { get; }

    public DemoSession(EngineHost initialHost, DemoEntry? initialEntry, string repoRoot)
        : this(initialHost, null, initialEntry, repoRoot)
    {
    }

    public DemoSession(EngineHost initialHost, IRenderOverlay? initialOverlay, DemoEntry? initialEntry, string repoRoot)
    {
        _host = initialHost;
        _overlay = initialOverlay;
        _current = initialEntry;
        RepoRoot = repoRoot;
    }

    // The live host. Not `null` between switches -- TryApplyPending/SwitchTo dispose the OLD host only
    // after the new one is fully built and swapped in, so a reader never sees a torn/disposed host.
    public EngineHost Host
    {
        get { lock (_lock) return _host; }
    }

    // The overlay paired with the current Host (null for a Scenario/Sandbox demo, or an ad-hoc path --
    // only a Kind.Evac entry has one today). Always consistent with Host: both are swapped together.
    public IRenderOverlay? CurrentOverlay
    {
        get { lock (_lock) return _overlay; }
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
    // reading Host/Snapshot/camera for that frame). Builds the new EngineHost (+ overlay), disposes the
    // OLD host, and swaps Current/Host/CurrentOverlay in atomically. Returns true (with the new host +
    // entry) iff a switch was applied this call -- the caller MUST then re-fit its camera to the new
    // host's bounds, invalidate its static road-layer cache (both Raylib-side; see Program.cs's
    // RunLocal), refresh its local `overlay` variable from CurrentOverlay, and should reset any
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

        var (newHost, newOverlay) = BuildHost(toApply, RepoRoot);
        EngineHost old;
        lock (_lock)
        {
            old = _host;
            _host = newHost;
            _overlay = newOverlay;
            _current = toApply;
        }

        old.Dispose(); // overlays need no disposal (they own no unmanaged/engine resources of their own)
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

    // The DemoEntry -> (EngineHost, IRenderOverlay?) factory (§5, extended by the D10 overlay seam):
    //   * Scenario -- the resolved net path in that dir; EngineHost auto-detects scenario mode from the
    //     adjacent *.rou.xml/*.sumocfg. No overlay.
    //   * Sandbox -- the net path, forced into sandbox mode with SandboxFleet as the spawn cap (falling
    //     back to EngineHost's own default of 80 when SandboxFleet is 0, i.e. not raised by the entry).
    //     No overlay.
    //   * Evac -- an EvacOverlay (the ONLY place in the demo tool that names Sim.Evac) drives its own
    //     engine + per-step hook through EngineHost.CreateCustom, then is Bind()-ed to the host it just
    //     produced so its OnWorldClick can call host.Rebuild(). Sim.Viewer.Core never sees any of this.
    public static (EngineHost Host, IRenderOverlay? Overlay) BuildHost(DemoEntry entry, string repoRoot)
    {
        switch (entry.Kind)
        {
            case DemoKind.Evac:
                var evacKind = entry.EvacKind
                    ?? throw new InvalidOperationException($"Demo '{entry.Name}' has Kind.Evac but no EvacKind.");
                var overlay = new EvacOverlay(evacKind, repoRoot);
                var host = EngineHost.CreateCustom(overlay.NetPath, overlay.Build);
                overlay.Bind(host);
                return (host, overlay);
            case DemoKind.Pedestrian:
                // P7-1: mirrors the Evac case above, minus Bind() -- PedOverlay has no HandlesWorldClick
                // seam (no incident-placement/click behaviour is defined for a pedestrian-only demo yet),
                // so it never needs a host reference back.
                var pedKind = entry.PedKind
                    ?? throw new InvalidOperationException($"Demo '{entry.Name}' has Kind.Pedestrian but no PedKind.");
                // P7-2: pick the overlay by pedKind -- "lod-remote" reconstructs its crowd from the wire
                // (RemotePedOverlay), every other kind is the in-process PedOverlay. Both implement
                // IPedDemoOverlay, so the host is built uniformly through that seam.
                IPedDemoOverlay pedOverlay = pedKind switch
                {
                    "lod-remote" => new RemotePedOverlay(repoRoot),
                    // D3a: same reconstruct-from-the-wire overlay, but over the live CycloneDDS transport
                    // (server + subscriber on one participant, in this process).
                    "lod-remote-dds" => new RemotePedOverlay(repoRoot, PedWireTransport.Dds),
                    // D3b: PURE remote IG -- no local sim; subscribe to a separate `--mode ped-publish`
                    // process's live DDS ped stream and render it (the two-process, cross-process topology).
                    "lod-remote-dds-sub" => new RemotePedOverlay(repoRoot, PedWireTransport.DdsSubscribeOnly),
                    _ => new PedOverlay(pedKind, repoRoot),
                };
                var pedHost = EngineHost.CreateCustom(pedOverlay.NetPath, pedOverlay.Build);
                return (pedHost, pedOverlay);
            case DemoKind.Sandbox:
                return (new EngineHost(
                    ResolveNetPath(Path.Combine(repoRoot, entry.PathRelToRepo)),
                    spawnCap: entry.SandboxFleet > 0 ? entry.SandboxFleet : 80,
                    forceSandbox: true), null);
            case DemoKind.Scenario:
                return (new EngineHost(ResolveNetPath(Path.Combine(repoRoot, entry.PathRelToRepo))), null);
            default:
                throw new ArgumentOutOfRangeException(nameof(entry), entry.Kind, "Unknown DemoKind.");
        }
    }

    // Mirrors Program.cs's local ResolveNetPath helper (accept either a *.net.xml file directly, or a
    // directory containing exactly one).
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
