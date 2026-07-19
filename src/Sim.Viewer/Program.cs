using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Runtime;
using CycloneDDS.Runtime;
using Raylib_cs;
using rlImGui_cs;
using ImGuiNET;
using Sim.Core;
using Sim.Pedestrians.Lod;
using Sim.Replication;
using Sim.Replication.Dds;
using Sim.Viewer;
using Sim.Viewer.Core;
using Sim.Viewer.Motion;
using Sim.Viewer.Raylib;

// docs/SUMOSHARP-NATIVE-VIEWER.md P0/P1/P2b/P3: the native desktop viewer entry point.
//   dotnet run --project src/Sim.Viewer -- --mode <local|loopback|publish> <scenarioDir|net.xml> [opts]
//   dotnet run --project src/Sim.Viewer -- --mode remote [opts]                 (no scenario arg -- P3)
// `--mode local` renders the authoritative SimulationSnapshot every frame (no transport, no dead
// reckoning -- EngineHost owns the Engine + SimulationRunner directly). `--mode loopback` (P2b) runs a
// DdsPublisher + DdsSubscriber in-process over DDS and renders the DEAD-RECKONED poses coming through DDS
// (DrClock + PoseResolver), not the local Snapshot -- SUMOSHARP-NATIVE-VIEWER.md's "Modes" section.
// `--mode publish` (P3) is `loopback`'s publish half split into its OWN process: headless (no window), owns
// EngineHost + DdsPublisher, loops PublishStep() at the sim cadence forever (or until `--seconds` / Ctrl-C).
// `--mode remote` (P3) is `loopback`'s subscribe/render half split into its OWN process: DdsSubscriber +
// DrClock + the render loop, but NO EngineHost/publisher anywhere in this process -- a `--mode publish`
// process (this one or a genuinely separate machine) must already be running, or eventually start, for a
// remote viewer to show anything. VIEW-ONLY: no obstacle/restart/random-traffic controls (nothing here can
// command an engine it doesn't own).
// `--screenshot`/`--frames` renders headless (no interactive loop) for the Xvfb verification recipe in
// the design doc: render `frames` frames then TakeScreenshot and exit.
// `--drop-obstacle <wx>,<wy>` (P1): a headless test hook -- inject one obstacle at the given WORLD point
// right after startup, so an obstacle + the resulting queue are visible in a `--screenshot` without
// needing real mouse input under Xvfb.
// `--delay <seconds>` (P2b): presets the loopback/remote DR playout delay (0 = extrapolate) so the
// interactive slider's effect can be verified headlessly (can't be driven by mouse input under Xvfb).
// `--seconds <n>` (P3): caps `--mode publish`'s otherwise-infinite loop to `n` wall-clock seconds, for
// scripted/CI-style runs; omit it for a real long-lived headless publisher (Ctrl-C to stop).

// Interactive viewer: prefer short, non-blocking GC pauses over throughput. SustainedLowLatency defers
// blocking gen2 collections (trading some memory) so a background gen2 doesn't stall a render frame -- one
// of the two things being chased for the ~2s frame hiccup (the other being per-frame allocations, now
// removed on the render path). Applies to every mode; all are latency-sensitive.
GCSettings.LatencyMode = GCLatencyMode.SustainedLowLatency;

string? mode = null;
string? inputPath = null;
string? screenshotPath = null;
string? selftestPath = null;
// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5/§1: pick the INITIAL demo from DemoCatalog.Resolve by name
// (case-insensitive substring match), for `--mode local`. Omit it (with an ad-hoc <path> instead) to keep
// today's behaviour exactly -- see RunLocal.
string? demoName = null;
// Hidden headless smoke test (T4 success condition): build a demo, DemoSession.SwitchTo a second one, then
// SwitchTo an Evac demo, and exit -- no window, no --mode needed. Proves a live switch doesn't leak/hang
// across scenario/sandbox/evac boundaries.
var demoSmoke = false;
var frames = 150;
var delaySeconds = 1.0f; // default DR playout delay (s). The auto "always interpolate" delay is OFF by
                         // default -- it fluctuated and made rendered speed visibly pulse; a stable manual
                         // delay reads far smoother (user 2026-07-15). Override with --delay / the slider.
(double X, double Y)? dropObstacle = null;
double? secondsCap = null;
int? fleet = null;
var perf = false;
double? simRate = null;
double? stepLen = null;
string? traceVeh = null;
// docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.1): hidden proof-of-seam flag -- installs a trivial
// MarkerOverlay (a bright magenta dot at the net centre) through the generic IRenderOverlay hook, so a
// screenshot can confirm a domain-agnostic overlay renders through the seam without the render loop
// knowing its concrete type. `--mode local` only; harmless everywhere else (RunLocal ignores it unless
// set).
var overlayTest = false;

for (var i = 0; i < args.Length; i++)
{
    switch (args[i])
    {
        case "--mode":
            mode = args[++i];
            break;
        case "--selftest":
            selftestPath = args[++i];
            break;
        case "--screenshot":
            screenshotPath = args[++i];
            break;
        case "--frames":
            frames = int.Parse(args[++i]);
            break;
        case "--delay":
            delaySeconds = float.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        case "--seconds":
            secondsCap = double.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        // docs/SUMOSHARP-NATIVE-VIEWER-TESTING.md TASK 1: raise the sandbox spawn cap so a large net can be
        // filled to ~10k for the 60 fps perf pass. `--fleet` and `--spawn-cap` are aliases.
        case "--fleet":
        case "--spawn-cap":
            fleet = int.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        // Perf-measurement mode for `--mode local`: uncaps the frame rate (so the true render frame time is
        // measured, not clamped to 16.6 ms) and prints a periodic PERF line (wall / sim time / veh count /
        // fps / frame-ms avg+p99). Pair with `--seconds N` to auto-exit after the sweep.
        case "--perf":
            perf = true;
            break;
        // Preset the live sim rate (Steps == 1s-of-sim-time each, per wall-clock second) for `--mode local`,
        // the same knob the controls-panel slider drives -- handy for scripted perf runs at a fixed rate.
        case "--sim-rate":
            simRate = double.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        // Preset the sim step length (s) for `--mode local` (SUMO's step-length / the sim resolution), the
        // same knob the controls-panel "sim resolution" buttons drive. Smaller = finer/smoother, heavier.
        case "--step":
            stepLen = double.Parse(args[++i], CultureInfo.InvariantCulture);
            break;
        // Diagnostic: trace one vehicle (by id) — logs its AUTHORITATIVE pose (AUTHTRACE, the smooth ground
        // truth) and its DR-reconstructed render pose (DRTRACE) each frame, for the DR/lane-change analysis.
        case "--trace-veh":
            traceVeh = args[++i];
            break;
        case "--drop-obstacle":
            var parts = args[++i].Split(',');
            dropObstacle = (
                double.Parse(parts[0], CultureInfo.InvariantCulture),
                double.Parse(parts[1], CultureInfo.InvariantCulture));
            break;
        case "--demo":
            demoName = args[++i];
            break;
        case "--demo-smoke":
            demoSmoke = true;
            break;
        case "--overlay-test":
            overlayTest = true;
            break;
        default:
            inputPath ??= args[i];
            break;
    }
}

// T4 headless smoke test: no --mode, no window -- dispatched before every other guard.
if (demoSmoke)
{
    return RunDemoSmoke();
}

// docs/SUMOSHARP-NATIVE-VIEWER.md P2 — a headless (no window, no raylib) proof that the DDS data path
// round-trips: EngineHost -> DdsPublisher -> CycloneDDS -> DdsSubscriber. Accepts either a direct net.xml
// path or a scenario/sandbox directory, resolved exactly like `--mode local` does below.
if (selftestPath is not null)
{
    return LoopbackSelfTest.Run(ResolveNetPath(selftestPath));
}

// P3: the only mode with no local EngineHost, so it's the only one that takes no scenario/net argument --
// dispatch it before the `inputPath is null` guard below.
if (mode == "remote")
{
    return RunRemote(screenshotPath, frames, delaySeconds);
}

// D3b (docs/PEDESTRIAN-DDS-TRANSPORT-TASKS.md): the pedestrian analog of `--mode publish` -- a headless
// process (no window) that runs the crossing-plaza ped server-sim (RemotePedServer) and publishes its
// wire over the LIVE CycloneDDS ped binding (DdsPedReplicationSink). A separate `--mode local --demo
// "Pedestrian remote (DDS subscribe)"` process (or a Godot `--transport=dds --peds`) reconstructs + renders
// it. No scenario arg -- the plaza net is fixed -- so dispatch before the inputPath guard.
if (mode == "ped-publish")
{
    return RunPedPublish(secondsCap);
}

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5: `--mode local --demo "<name>"` needs NO <path> at all --
// the DemoCatalog entry supplies it -- so the `inputPath is null` guard below (still required for
// loopback/publish, which have no --demo path yet) is skipped for this one case.
if (mode == "local" && demoName is not null)
{
    return RunLocal(null, demoName, screenshotPath, frames, dropObstacle, fleet, perf, secondsCap, simRate, stepLen, overlayTest);
}

if (inputPath is null)
{
    Console.Error.WriteLine("Sim.Viewer: missing <scenarioDir|net.xml> argument.");
    return 1;
}

if (mode == "local")
{
    return RunLocal(ResolveNetPath(inputPath), demoName, screenshotPath, frames, dropObstacle, fleet, perf, secondsCap, simRate, stepLen, overlayTest);
}

if (mode == "loopback")
{
    return RunLoopback(ResolveNetPath(inputPath), screenshotPath, frames, delaySeconds, dropObstacle, traceVeh, fleet);
}

if (mode == "publish")
{
    return RunPublish(ResolveNetPath(inputPath), secondsCap, fleet, stepLen);
}

Console.Error.WriteLine($"Sim.Viewer: unknown --mode '{mode ?? "(none)"}' (expected local|loopback|publish|remote).");
return 1;

// Accept either a scenario/sandbox directory (resolve its *.net.xml) or a direct net.xml path --
// EngineHost itself does the scenario-vs-sandbox detection from the resolved net path's directory.
static string ResolveNetPath(string path)
{
    if (Directory.Exists(path))
    {
        return Directory.EnumerateFiles(path, "*.net.xml").FirstOrDefault()
            ?? throw new FileNotFoundException($"No *.net.xml found in directory '{path}'.");
    }

    return path;
}

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5: `netPath` is the ad-hoc "(custom)" path (today's
// `--mode local <path>`, no catalog entry backing it); `demoName` is the §1 catalog selector (`--demo
// "<name>"`). Exactly one is non-null (enforced by the two call sites in the top-level dispatch above).
// A demo build ignores `fleet`/`stepLen` (the catalog entry's own SandboxFleet/scenario config decides
// those); an ad-hoc path keeps using them exactly as before -- this is the "keep working exactly as
// today" invariant §5 requires for `--mode local <path>` with no `--demo`.
static int RunLocal(string? netPath, string? demoName, string? screenshotPath, int frames,
    (double X, double Y)? dropObstacle, int? fleet, bool perf, double? secondsCap, double? simRate, double? stepLen,
    bool overlayTest = false)
{
    var step = stepLen ?? 1.0;
    var repoRoot = DemoCatalog.RepoRoot();
    // docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §6 / -TASKS.md T5: resolved ONCE, up front, so both the
    // initial `--demo` lookup below and the ImGui Demos panel (drawn every frame) share the same list
    // instead of re-walking the filesystem per frame.
    var resolvedCatalog = DemoCatalog.Resolve(repoRoot);

    EngineHost initialHost;
    IRenderOverlay? initialOverlay;
    DemoEntry? initialEntry;

    if (demoName is not null)
    {
        var catalog = resolvedCatalog;
        initialEntry = catalog.FirstOrDefault(e => e.Name.Contains(demoName, StringComparison.OrdinalIgnoreCase))
            ?? catalog.FirstOrDefault(e => e.Category == DemoCategory.Junctions)
            ?? catalog.FirstOrDefault();
        if (initialEntry is null)
        {
            Console.Error.WriteLine("Sim.Viewer: --demo given but DemoCatalog.Resolve found no usable entries.");
            return 1;
        }

        if (!string.Equals(initialEntry.Name, demoName, StringComparison.OrdinalIgnoreCase)
            && !initialEntry.Name.Contains(demoName, StringComparison.OrdinalIgnoreCase))
        {
            Console.Error.WriteLine(
                $"Sim.Viewer: --demo '{demoName}' not found -- falling back to '{initialEntry.Name}'.");
        }

        (initialHost, initialOverlay) = DemoSession.BuildHost(initialEntry, repoRoot);
    }
    else
    {
        initialEntry = null; // ad-hoc "(custom)" -- no catalog entry backs this host (§5)
        initialOverlay = null; // ad-hoc paths never carry a domain overlay
        initialHost = fleet is { } fleetCap
            ? new EngineHost(netPath!, spawnCap: fleetCap, forceSandbox: true, stepLength: step)
            : new EngineHost(netPath!, stepLength: step);
    }

    using var session = new DemoSession(initialHost, initialOverlay, initialEntry, repoRoot);
    var host = session.Host;

    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.1/P3.2): the generic render-overlay seam. Sourced from
    // the session's CurrentOverlay (e.g. an EvacOverlay for a Kind.Evac demo, null otherwise), except the
    // hidden `--overlay-test` proof flag, which takes priority for its own diagnostic purpose. RunLocal
    // never names IRenderOverlay's concrete type.
    IRenderOverlay? overlay = overlayTest
        ? new MarkerOverlay((host.MinX + host.MaxX) / 2.0, (host.MinY + host.MaxY) / 2.0)
        : session.CurrentOverlay;

    // Default interactive speed to 1x real-time (what a viewer expects); perf runs keep the 10x fast-forward
    // so the fleet warms up quickly. Either way the panel's "speed" slider overrides live.
    host.SetSpeed(simRate ?? (perf ? 10.0 : 1.0));

    if (dropObstacle is { } drop)
    {
        host.InjectObstacleAtWorld(drop.X, drop.Y);
    }

    // Render FPS cap, live-adjustable from the controls panel (30 / 60 / unlimited). Default 60 -- rendering
    // at the hundreds of fps the GPU allows for a 10 Hz sim is wasted work. `--perf` starts unlimited (0) so
    // the measurement sees the true render frame time instead of a flat 16.6 ms. 0 = no cap.
    var fpsCap = perf ? 0 : 60;

    // TASK 1: cache the static road layer in a RenderTexture, re-baked only on camera change (see
    // RoadLayerCache) -- the ~13k-edge net is otherwise re-stroked every frame. RoadLayerCache's constructor
    // calls Raylib.LoadRenderTexture, which needs a live GL context -- ViewerHost.Run doesn't create the
    // window (InitWindow) until inside Run(cfg), so this can't be constructed here (before the call) like the
    // pre-refactor code did; it's lazily constructed on DrawWorld's first invocation instead (which always
    // runs after the window/context exist), sized off the actual framebuffer, and disposed in the `finally`
    // below since ViewerHost.Run's own return is this method's last statement.
    RoadLayerCache? roadLayer = null;

    var frameStats = new FrameStats();
    var perfWall = Stopwatch.StartNew();
    var lastPerfLog = 0.0;

    // Render-behind interpolation state. renderClock is a sim-time cursor kept ~one sim-step behind the
    // newest snapshot so each frame blends the two latest authoritative frames (10 Hz sim -> 60 Hz render
    // without teleporting). prevIndex is reused every frame (Clear keeps capacity) so the smoothing adds no
    // steady-state allocation. `smooth` is the panel toggle (default on).
    var renderClock = 0.0;
    var smooth = true;
    // Pre-size the reused draw list to the fleet so it neither grows (and reallocates) while the sim warms up
    // from 0 to ~10k -- that per-warmup-frame growth was itself a source of early allocations/stutter.
    var interpCap = Math.Max(fleet ?? 0, 256);
    var prevIndex = new Dictionary<VehicleHandle, int>(interpCap);
    // Per-vehicle rendered-heading low-pass state, double-buffered (swapped each frame) so it prunes
    // despawned vehicles and stays bounded. Smooths the ~5 discrete headings of a junction's coarse internal
    // lane (netconvert internal-link-detail=5) into a continuous rotation through the turn.
    var headingPrev = new Dictionary<VehicleHandle, float>(interpCap);
    var headingCur = new Dictionary<VehicleHandle, float>(interpCap);
    // Interpolation clock driven by the measured WALL interval between snapshot arrivals: interpolate
    // prev->cur by the fraction of that interval elapsed, which sweeps the pose at CONSTANT velocity across
    // each snapshot (matching the actual production cadence). lastSnapSimTime/Wall mark the newest arrival;
    // intervalEma smooths the arrival period.
    var lastSnapSimTime = double.NaN;
    var lastSnapWall = 0.0;
    var intervalEma = 0.0;

    // Set by PumpFrame the first time it runs, so OnFrameStart (a demo switch) can clear the SAME reused draw
    // list ViewerHost owns and passes into PumpFrame/DrawWorld each frame.
    List<Renderer.DrVehicleDraw>? sharedDraws = null;

    // Written by PumpFrame, read by DrawWorld/DrawImGui -- must be method-scope so the callbacks share them.
    SimulationSnapshot snapshot = host.Snapshot;
    var prevSnapshot = snapshot;
    var actualSpeed = host.Speed;

    var cfg = new ViewerHostConfig
    {
        WindowTitle = "SumoSharp - native viewer (local)",
        TargetFps = fpsCap,
        ScreenshotPath = screenshotPath,
        Frames = frames,
        DrawCapacity = interpCap,
        ResizableWindow = true,

        InitialCameraBounds = () => (host.MinX, host.MinY, host.MaxX, host.MaxY),

        // docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5: apply a queued demo switch (T5's ImGui picker calls
        // session.RequestSwitch(entry) from inside DrawImGui below) at the TOP of the frame, before anything
        // this frame reads `host`. On a switch: re-fit the camera to the new host's bounds (via the returned
        // bounds -- ViewerHost does the FitCamera), force the static road-layer cache to re-bake (it is keyed
        // on the OLD host.Network -- without this it keeps drawing the old net's roads, per the design doc's
        // explicit warning), and reset the render-behind interpolation/heading state so it doesn't blend
        // across the switch's timeline jump.
        OnFrameStart = () =>
        {
            if (session.TryApplyPending(out var switchedHost, out _))
            {
                host = switchedHost;
                // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.2): refresh the overlay from the session too --
                // it and Host are swapped atomically by TryApplyPending, so this always matches the new host
                // (e.g. switching INTO an evac demo installs its EvacOverlay; switching OUT of one drops it).
                // `--overlay-test` keeps its own diagnostic marker regardless of demo switches.
                if (!overlayTest)
                {
                    overlay = session.CurrentOverlay;
                }

                roadLayer?.Invalidate();
                renderClock = 0.0;
                lastSnapSimTime = double.NaN;
                lastSnapWall = 0.0;
                intervalEma = 0.0;
                sharedDraws?.Clear();
                prevIndex.Clear();
                headingPrev.Clear();
                headingCur.Clear();
                host.SetSpeed(simRate ?? (perf ? 10.0 : 1.0));
                return (host.MinX, host.MinY, host.MaxX, host.MaxY);
            }

            return null;
        },

        OnResize = (w, h) => roadLayer?.Resize(w, h),

        // Atomic (cur, prev) pair. Interpolate by the fraction of the measured snapshot-arrival interval
        // elapsed since `cur` arrived -> the render pose sweeps prev->cur at constant velocity over the
        // interval, matching however fast the engine is actually producing frames. (Estimating an
        // instantaneous rate from the impulsive per-frame sim-time deltas produced a sawtooth: race to the
        // newest frame then freeze until the next arrived.)
        PumpFrame = (dt, draws) =>
        {
            frameStats.Add(dt);
            sharedDraws = draws;

            (snapshot, prevSnapshot) = host.SnapshotPair;
            var wallClock = perfWall.Elapsed.TotalSeconds;

            if (double.IsNaN(lastSnapSimTime) || snapshot.Time < lastSnapSimTime)
            {
                // first frame, or a restart / step-length rebuild reset the timeline
                lastSnapSimTime = snapshot.Time;
                lastSnapWall = wallClock;
                intervalEma = 0.0;
            }
            else if (snapshot.Time > lastSnapSimTime)
            {
                var interval = wallClock - lastSnapWall;
                if (interval > 1e-4)
                {
                    intervalEma = intervalEma <= 0.0 ? interval : intervalEma + (interval - intervalEma) * 0.2;
                }

                lastSnapSimTime = snapshot.Time;
                lastSnapWall = wallClock;
            }

            var span = snapshot.Time - prevSnapshot.Time;
            if (smooth && span > 1e-9 && intervalEma > 1e-6)
            {
                // frac past 1.0 means the next snapshot is running late (variable per-tick compute time).
                // Rather than freeze at `cur` until it lands (a once-per-snapshot stall), allow a small
                // bounded extrapolation beyond it (frac up to 1.25 => 0.25 step ahead) to bridge the gap
                // smoothly; it only engages when the sim is late, so on-time frames (incl. turns) never
                // extrapolate.
                var frac = (wallClock - lastSnapWall) / intervalEma;
                if (frac < 0.0) frac = 0.0; else if (frac > 1.25) frac = 1.25;
                renderClock = prevSnapshot.Time + frac * span;
            }
            else
            {
                renderClock = snapshot.Time; // smoothing off / not enough info yet -> draw the newest frame
            }

            // Actual delivered speed (x real-time) for the diagnostics readout: span (== step length) per
            // measured arrival interval.
            actualSpeed = intervalEma > 1e-6 ? span / intervalEma : host.Speed;
            // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.2): no more evac-specific `Fear` threading here --
            // fear tinting is now the overlay's own DrawWorldOver overpaint, keyed off each vehicle's generic
            // Handle. This just builds plain speed-coloured draw poses (+ each vehicle's Handle).
            RenderHelpers.BuildLocalVehicleDraws(snapshot, prevSnapshot, renderClock, smooth, draws, prevIndex,
                headingPrev, headingCur, dt);
            (headingPrev, headingCur) = (headingCur, headingPrev); // swap: headingCur becomes next frame's prior
        },

        DrawWorld = (camera, draws) =>
        {
            // Lazily constructed here (see the declaration above): this is the first callback guaranteed to
            // run after ViewerHost has created the window/GL context, sized off the actual framebuffer.
            roadLayer ??= new RoadLayerCache(Raylib.GetScreenWidth(), Raylib.GetScreenHeight());
            roadLayer.EnsureAndBlit(camera, cam => Renderer.DrawStaticWorld(cam, host.Network));

            // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.1/P3.2): the generic overlay's UNDER layer,
            // immediately before the vehicles it may want to draw beneath/around (e.g. the evac layer's
            // boundary/incident/abandoned-cars/pushers).
            overlay?.DrawWorldUnder(camera, snapshot, draws);

            Renderer.DrawDynamicWorld(camera, host.Network, snapshot, host.ObstaclePoints, draws);

            // ...and its OVER layer, immediately after (e.g. the evac layer's pedestrians + fear overpaint).
            overlay?.DrawWorldOver(camera, snapshot, draws);
        },

        // docs/SUMOSHARP-PACKAGING-DESIGN.md D10: an overlay that wants clicks (e.g. the evac layer's
        // incident placement, via EvacOverlay.OnWorldClick) gets first refusal; otherwise the generic default
        // is to drop an obstacle. RunLocal never special-cases a domain here -- the routing decision is
        // entirely `overlay.HandlesWorldClick`.
        OnWorldClick = (wx, wy) =>
        {
            if (overlay is { HandlesWorldClick: true })
            {
                overlay.OnWorldClick(wx, wy);
            }
            else
            {
                host.InjectObstacleAtWorld(wx, wy);
            }
        },

        DrawImGui = showDiagnostics =>
        {
            DemoUi.DrawDemosPanel(session, resolvedCatalog);
            ViewerControlsPanels.DrawControlsPanel(host, ref fpsCap, ref smooth);
            overlay?.DrawUi();
            if (showDiagnostics)
            {
                Renderer.DrawDiagnosticsPanel(snapshot, frameStats, host.Speed, actualSpeed, host.StepLength);
            }
        },

        OnFrameEnd = () =>
        {
            if (perf)
            {
                var wall = perfWall.Elapsed.TotalSeconds;
                if (wall - lastPerfLog >= 1.0)
                {
                    var (min, avg, p99) = frameStats.Compute();
                    var snap = host.Snapshot;
                    Console.WriteLine(
                        $"PERF wall={wall,6:F1}s sim={snap.Time,6:F1}s veh={snap.Count,6} " +
                        $"fps={Raylib.GetFPS(),4} ms[min={min * 1000f,6:F2} avg={avg * 1000f,6:F2} p99={p99 * 1000f,6:F2}]");
                    lastPerfLog = wall;
                }

                if (secondsCap is { } capS && wall >= capS)
                {
                    return true;
                }
            }

            return false;
        },
    };

    try
    {
        return ViewerHost.Run(cfg);
    }
    finally
    {
        roadLayer?.Dispose();
    }
}

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §5 / -TASKS.md T4 success condition: a HEADLESS (no window,
// no raylib) proof that DemoSession.SwitchTo can move a live host from one pure-SUMO demo to another and
// then into an Evac demo -- disposing the old host each time -- without leaking or hanging. `--demo-smoke`
// dispatches here before any --mode is even looked at.
static int RunDemoSmoke()
{
    var repoRoot = DemoCatalog.RepoRoot();
    var catalog = DemoCatalog.Resolve(repoRoot);

    var demoA = catalog.FirstOrDefault(e => e.Category == DemoCategory.Junctions)
        ?? throw new InvalidOperationException("DEMO-SMOKE: no Junctions entry resolved.");
    var demoB = catalog.FirstOrDefault(e => e.Category == DemoCategory.TrafficLights)
        ?? throw new InvalidOperationException("DEMO-SMOKE: no TrafficLights entry resolved.");
    var demoEvac = catalog.FirstOrDefault(e => e.Kind == DemoKind.Evac)
        ?? throw new InvalidOperationException("DEMO-SMOKE: no Evac entry resolved.");

    Console.WriteLine($"DEMO-SMOKE: building '{demoA.Name}' ({demoA.Kind})...");
    var (hostA, overlayA) = DemoSession.BuildHost(demoA, repoRoot);
    using var session = new DemoSession(hostA, overlayA, demoA, repoRoot);
    Thread.Sleep(300); // let the runner/spawner actually tick at least once
    Console.WriteLine($"DEMO-SMOKE: built. edges={session.Host.Network.EdgesById.Count} snap.time={session.Host.Snapshot.Time:F2}");

    Console.WriteLine($"DEMO-SMOKE: switching to '{demoB.Name}' ({demoB.Kind})...");
    session.SwitchTo(demoB);
    Thread.Sleep(300);
    Console.WriteLine($"DEMO-SMOKE: switched. current='{session.Current?.Name}' edges={session.Host.Network.EdgesById.Count} snap.time={session.Host.Snapshot.Time:F2}");

    Console.WriteLine($"DEMO-SMOKE: switching to evac '{demoEvac.Name}' ({demoEvac.Kind}, EvacKind={demoEvac.EvacKind})...");
    session.SwitchTo(demoEvac);
    Thread.Sleep(300);
    Console.WriteLine($"DEMO-SMOKE: switched. current='{session.Current?.Name}' edges={session.Host.Network.EdgesById.Count} snap.time={session.Host.Snapshot.Time:F2} evac={(session.CurrentOverlay is not null ? "present" : "null")}");

    Console.WriteLine("DEMO-SMOKE: OK (no exception, no hang).");
    return 0;
}

// docs/SUMOSHARP-NATIVE-VIEWER.md P3 ("remote mode + QoS") — the publish HALF of loopback, split into its
// OWN process: headless (no window, no raylib/rlImGui at all), owns EngineHost + DdsPublisher directly, and
// just keeps calling PublishStep() at the sim cadence forever -- a real second process for `--mode remote`
// to late-join against. `secondsCap` (wall-clock seconds, from `--seconds`) is an optional cap for scripted
// runs; omit it (null) for a real long-lived publisher, stopped with Ctrl-C.
static int RunPublish(string netPath, double? secondsCap, int? fleet, double? stepLen)
{
    // `--fleet N` runs the net as a random-traffic SANDBOX (forceSandbox), which also makes the sim-tick-rate
    // control changeable (a scenario's step-length is fixed) -- useful for exercising remote controls +
    // lane-changes on a multilane net. Without it, a scenario dir drives its own committed demand.
    var step = stepLen ?? 1.0;
    using var host = fleet is { } fleetCap
        ? new EngineHost(netPath, spawnCap: fleetCap, forceSandbox: true, stepLength: step)
        : new EngineHost(netPath, stepLength: step);
    using var participant = new DdsParticipant();
    using var publisher = new DdsPublisher(host, participant);
    // Reverse channel: apply commands a view-only remote sends (pause/speed/restart/inject/...).
    using var commandReader = new DdsCommandReader(participant);
    // Forward channel: publish engine state so remotes reflect it + disable inapplicable controls.
    using var statusWriter = new DdsStatusWriter(participant);

    // DDS discovery is async -- give any already-running readers time to match before the durable geometry
    // publish (LoopbackSelfTest's proven pattern). A LATE reader's TRANSIENT_LOCAL durability means it does
    // NOT need to be listening yet at this point -- it will get this exact geometry sample whenever it
    // starts, per DdsQos's two-process verification -- this sleep is only courtesy for readers already up.
    Thread.Sleep(500);
    publisher.PublishGeometryOnce();

    var stopRequested = false;
    Console.CancelKeyPress += (_, e) =>
    {
        e.Cancel = true; // let the loop below exit cleanly (Dispose the DDS writers/EngineHost) instead of
                          // the process dying mid-write.
        stopRequested = true;
    };

    Console.WriteLine($"Sim.Viewer: publishing headless from '{netPath}' (Ctrl-C to stop" +
        (secondsCap is { } cap ? $"; capped at {cap:F0}s" : "") + ").");

    var startWall = Stopwatch.StartNew();
    var lastPublishedSimTime = double.NaN;
    var lastGeneration = host.Generation;
    var lastHeartbeatWall = -2.0; // force an immediate first heartbeat

    while (!stopRequested)
    {
        if (secondsCap is { } capSeconds && startWall.Elapsed.TotalSeconds >= capSeconds)
        {
            break;
        }

        // Apply any pending remote commands before reading the snapshot this iteration, then publish the
        // resulting engine state (cheap; KEEP_LAST(1) durable -> a late remote gets it immediately).
        commandReader.PumpApply(host);
        statusWriter.Publish(host);

        // Same gate RunLoopback uses: EngineHost's SimulationRunner ticks on its own real-time-paced
        // background thread, so most polls of this loop see an unchanged snapshot and would otherwise
        // re-publish identical state.
        var snap = host.Snapshot;
        // Restart (remote command) rebuilds the sim at t=0: sim time drops, so the `> lastPublishedSimTime`
        // gate would suppress all publishing until it climbed back up -> the remote stays empty after a
        // restart. Detect the rebuild DETERMINISTICALLY via EngineHost.Generation (not a fragile backward-
        // time-jump threshold, which misses a small pre-restart time) and reset the gate so the fresh
        // timeline republishes immediately.
        if (host.Generation != lastGeneration)
        {
            lastGeneration = host.Generation;
            publisher.Reset();
            lastPublishedSimTime = double.NaN;
        }

        if (double.IsNaN(lastPublishedSimTime) || snap.Time > lastPublishedSimTime)
        {
            publisher.PublishStep();
            lastPublishedSimTime = snap.Time;
        }

        var nowWall = startWall.Elapsed.TotalSeconds;
        if (nowWall - lastHeartbeatWall >= 2.0)
        {
            Console.WriteLine($"PUBLISH step={snap.StepCount} time={snap.Time:F1} vehicles={snap.Count}");
            lastHeartbeatWall = nowWall;
        }

        Thread.Sleep(50);
    }

    Console.WriteLine("Sim.Viewer: publish loop stopped.");
    return 0;
}

// D3b (docs/PEDESTRIAN-DDS-TRANSPORT-TASKS.md): the headless PEDESTRIAN publisher process. Runs the
// crossing-plaza ped server-sim (RemotePedServer) and publishes its DR-error-gated wire over the live
// CycloneDDS ped binding (DdsPedReplicationSink) at the ped-sim cadence. A separate subscriber process --
// the native `--demo "Pedestrian remote (DDS subscribe)"` viewer, or a Godot `--transport=dds --peds`
// client -- reconstructs + renders the crowd purely from this stream (server == IG over the real wire).
// No window; capped by `--seconds` or Ctrl-C, mirroring RunPublish's shape.
static int RunPedPublish(double? secondsCap)
{
    var repoRoot = DemoCatalog.RepoRoot();

    using var participant = new DdsParticipant();
    using var sink = new DdsPedReplicationSink(participant);
    var meter = new PedBandwidthMeter();
    var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
    var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);
    var wirePublisher = new PedReplicationPublisher(sink, scheduler, governor, meter, stepDt: RemotePedServer.Dt);
    var server = new RemotePedServer(repoRoot);

    // DDS discovery is async -- settle before the first publish. Durable topics (patharc/activity/lifecycle)
    // are TRANSIENT_LOCAL, so a subscriber that starts later still gets each ped's latest leg regardless.
    Thread.Sleep(500);

    var stopRequested = false;
    Console.CancelKeyPress += (_, e) =>
    {
        e.Cancel = true;
        stopRequested = true;
    };

    Console.WriteLine("Sim.Viewer: publishing pedestrians headless over DDS (Ctrl-C to stop" +
        (secondsCap is { } cap ? $"; capped at {cap:F0}s" : "") + ").");

    var startWall = Stopwatch.StartNew();
    var publishedSteps = 0;
    while (!stopRequested)
    {
        var events = server.Step();
        wirePublisher.Publish(events);
        publishedSteps++;

        if (secondsCap is { } capSeconds && startWall.Elapsed.TotalSeconds >= capSeconds)
        {
            break;
        }

        // ~2x real-time publish cadence (Dt = 0.2 s): fast enough to feed a smooth render, slow enough not to
        // spin. Correctness does not depend on it -- durable legs guarantee delivery, crowd is latest-wins.
        Thread.Sleep(100);
    }

    Console.WriteLine($"Sim.Viewer: ped publish loop stopped after {publishedSteps} steps " +
        $"({sink.CrowdBytesPublished + sink.PathArcBytesPublished + sink.ActivityBytesPublished + sink.LifecycleBytesPublished} wire bytes).");
    return 0;
}

// docs/SUMOSHARP-NATIVE-VIEWER.md P2b — one process runs BOTH the publisher (EngineHost -> DdsPublisher)
// and the subscriber/renderer, over DDS intra-host. Renders the DEAD-RECKONED poses coming through DDS
// (DrClock + PoseResolver against the SUBSCRIBER's decoded geometry/history), not the local Snapshot --
// the single-app DR test the design doc's "loopback" mode exists for.
static int RunLoopback(string netPath, string? screenshotPath, int frames, float initialDelaySeconds, (double X, double Y)? dropObstacle, string? traceVeh = null, int? fleet = null)
{
    // `--fleet N` runs the net as a random-traffic SANDBOX (like --mode publish) so a demand-less net
    // (e.g. the evac-grid samples) can be filled for a loopback DR review; without it, a scenario dir
    // drives its own committed demand.
    using var host = fleet is { } fleetCap
        ? new EngineHost(netPath, spawnCap: fleetCap, forceSandbox: true)
        : new EngineHost(netPath);
    using var participant = new DdsParticipant();
    using var publisher = new DdsPublisher(host, participant);
    using var subscriber = new DdsSubscriber(participant);

    if (dropObstacle is { } drop)
    {
        host.InjectObstacleAtWorld(drop.X, drop.Y);
    }

    // DDS discovery is async -- give the intra-process writer/reader pairs time to match before anything
    // is published (LoopbackSelfTest's proven pattern).
    Thread.Sleep(500);
    publisher.PublishGeometryOnce();

    // Drain until the whole network's geometry has arrived (or a short timeout), so the very first
    // rendered frames already have roads to draw instead of a blank world for a few frames.
    for (var i = 0; i < 50 && !subscriber.GeometryComplete; i++)
    {
        subscriber.Pump();
        Thread.Sleep(20);
    }

    var frameStats = new FrameStats();

    var drClock = new DrClock();
    var delaySlider = initialDelaySeconds;
    var delaySeeded = false;
    var smooth = false;
    // Default OFF: auto-driving delaySlider from the measured packet interval makes the delay FLUCTUATE,
    // which shifts sampleT and visibly pulses rendered vehicle speed (user 2026-07-15: "definitely a bad
    // idea"). A stable manual delay reads far smoother. Still available via the checkbox for the rare case
    // where minimising latency matters more than steadiness.
    var alwaysInterpolate = false;
    var smoother = new DrPoseSmoother();
    var lastPublishedSimTime = double.NaN;
    var lastGeneration = host.Generation;
    VehicleHandle? traceHandle = null; // diagnostic: resolved once from --trace-veh id
    var startWall = Stopwatch.StartNew();

    // Set at the end of PumpFrame each frame so DrawImGui's diagnostics panel can report the draw count
    // without the reused `draws` list being in its own scope.
    var lastDrawCount = 0;

    var cfg = new ViewerHostConfig
    {
        WindowTitle = "SumoSharp - native viewer (loopback DR)",
        ScreenshotPath = screenshotPath,
        Frames = frames,

        // Same net -> same bounds whether read locally (EngineHost.Network) or over DDS; local bounds are
        // already available without waiting on the subscriber, so the camera fit doesn't need to block
        // further.
        InitialCameraBounds = () => (host.MinX, host.MinY, host.MaxX, host.MaxY),

        PumpFrame = (dt, draws) =>
        {
            frameStats.Add(dt);

            // Publish at the SIM cadence (gated on the snapshot's own Time advancing), not the 60 Hz render
            // cadence -- EngineHost's SimulationRunner ticks in the background at its own targetHz, so most
            // render frames see an unchanged snapshot and would otherwise re-publish identical state.
            var snapTimeNow = host.Snapshot.Time;
            // Restart detection: a bumped EngineHost.Generation means Restart rebuilt the sim at t=0. This is
            // DETERMINISTIC -- the old heuristic (a >2 s backward jump in Snapshot.Time) silently missed when
            // the pre-restart sim time was small, leaving the publish gate wedged (`> lastPublishedSimTime`
            // forever false) so nothing republished: the "restart does nothing / no cars re-appear" bug. On a
            // restart: drop stale vehicle state, re-anchor the DR clock, reset the publish gate so the fresh
            // timeline republishes from t=0, and re-seed the auto-delay for the new stream.
            if (host.Generation != lastGeneration)
            {
                lastGeneration = host.Generation;
                publisher.Reset();
                subscriber.ResetVehicles();
                drClock.Reset();
                lastPublishedSimTime = double.NaN;
                delaySeeded = false;
            }

            if (double.IsNaN(lastPublishedSimTime) || snapTimeNow > lastPublishedSimTime)
            {
                publisher.PublishStep();
                lastPublishedSimTime = snapTimeNow;
            }

            // "always interpolate": override the manual slider so the render clock stays ~1.5 sample-intervals
            // behind the newest DDS packet and Resolve interpolates (not extrapolates) through junctions, where
            // the arc-window interpolation makes turns follow the real lane geometry. SLEW toward the target
            // instead of snapping: sampleT = renderSim - delay, so a step change in delay shifts the sample
            // point and teleports every vehicle. The small per-frame cap keeps the induced shift well under one
            // frame of real travel (no visible jump); steady-state the target is stable, so the slew is ~0.
            if (alwaysInterpolate)
            {
                var target = (float)Math.Clamp(drClock.AvgSampleInterval * 1.5, 0.3, 2.0);
                if (!delaySeeded && drClock.AvgSampleInterval > 0.0)
                {
                    delaySlider = target; // snap once at startup (nothing is moving yet) to skip the slew ramp
                    delaySeeded = true;
                }
                else if (Math.Abs(target - delaySlider) > 0.05f) // dead-band: ignore EMA jitter, slew real drift
                {
                    delaySlider += Math.Clamp(target - delaySlider, -0.008f, 0.008f);
                }
            }

            // Diagnostic (opt-in via --trace-veh): resolve the traced vehicle's handle by id, then log its
            // AUTHORITATIVE pose each frame (the smooth ground truth) alongside the DR-reconstructed DRTRACE
            // from PumpAndBuildVehicleDraws -- the harness used to diagnose junction/lane-change DR smoothness.
            if (traceVeh is not null)
            {
                var authSnap = host.Snapshot;
                if (traceHandle is null)
                {
                    for (var ti = 0; ti < authSnap.Count; ti++)
                    {
                        if (authSnap.VehicleId[ti] == traceVeh) { traceHandle = authSnap.Handles[ti]; break; }
                    }
                }

                if (traceHandle is { } ath && authSnap.TryGetVehicle(ath, out var avs))
                {
                    Console.WriteLine($"AUTHTRACE t={authSnap.Time:F1} x={avs.X:F2} y={avs.Y:F2} lane={avs.LaneId} pos={avs.Pos:F2} posLat={avs.PosLat:F2} spd={avs.Speed:F1}");
                }
            }

            // P3 refactor: pumping DDS + resolving each tracked vehicle's dead-reckoned draw pose doesn't
            // depend on this frame's camera input, so it's hoisted before the input/drawing block into a
            // function shared with `--mode remote` (PumpAndBuildVehicleDraws below) -- same math, same DR
            // resolve, same smoothing, just called from one place instead of duplicated per mode.
            RenderHelpers.PumpAndBuildVehicleDraws(subscriber, drClock, delaySlider, smooth, frameStats, smoother, draws,
                paused: host.IsPaused, traceHandle: traceHandle);
            lastDrawCount = draws.Count;
        },

        DrawWorld = (camera, draws) => Renderer.DrawWorldDds(camera, subscriber.Geometry, subscriber.TlStateByLane, draws),

        OnWorldClick = (wx, wy) => host.InjectObstacleAtWorld(wx, wy),

        DrawImGui = showDiagnostics =>
        {
            ViewerControlsPanels.DrawLoopbackControlsPanel(host, ref delaySlider, ref smooth, ref alwaysInterpolate);
            if (showDiagnostics)
            {
                var wallElapsed = startWall.Elapsed.TotalSeconds;
                var ddsSamplesPerSecond = wallElapsed > 0 ? subscriber.TotalVehicleSamplesReceived / wallElapsed : 0.0;
                Renderer.DrawDdsDiagnosticsPanel(frameStats, drClock, ddsSamplesPerSecond, lastDrawCount);
            }
        },

        OnHeadlessExit = () =>
            Console.WriteLine($"DRCLOCK: renderSim={drClock.RenderSim:F3} simRate={drClock.SimRate:F3} backSteps={drClock.BackSteps}"),
    };

    return ViewerHost.Run(cfg);
}

// docs/SUMOSHARP-NATIVE-VIEWER.md P3 ("remote mode + QoS") — the subscribe/render HALF of loopback, split
// into its OWN process with NO EngineHost/publisher anywhere in it: a DdsSubscriber + DrClock + the render
// loop only, sharing PumpAndBuildVehicleDraws below with RunLoopback. VIEW-ONLY (design doc's "Delegation
// model"): no restart/clear-obstacles/random-traffic controls, and a plain click in the world does nothing
// (there is no engine here to command, unlike local/loopback's InjectObstacleAtWorld).
//
// May start BEFORE a publisher exists at all, or long AFTER one has already been running (the late-join
// case this phase exists to prove): the window opens immediately, shows a "waiting for publisher..." banner
// until the durable geometry topic (RELIABLE/TRANSIENT_LOCAL -- see DdsQos) has delivered the whole
// network, and only fits the camera once that first happens -- which happens regardless of whether the
// publisher started before or after this process, exactly because that topic is durable.
static int RunRemote(string? screenshotPath, int frames, float initialDelaySeconds)
{
    using var participant = new DdsParticipant();
    using var subscriber = new DdsSubscriber(participant);
    // Reverse channel: drive the publisher's engine from this view-only remote (remote control).
    using var commandWriter = new DdsCommandWriter(participant);
    // Forward channel: the publisher's engine state, so this remote reflects real state (authoritative
    // pause/speed), disables inapplicable controls (sim-tick-rate on a fixed scenario), and freezes the DR
    // playout when the host is paused (no coast).
    using var statusReader = new DdsStatusReader(participant);

    var frameStats = new FrameStats();

    // Optimistic local mirror of the commanded engine state (a view-only remote gets no state feedback, so
    // the control widgets reflect what we've SENT). Initialised to the publisher's own interactive defaults.
    var remoteSpeed = 1f;
    var remoteRandom = false;
    var lastRemoteSimTime = 0.0;

    var drClock = new DrClock();
    var delaySlider = initialDelaySeconds;
    var delaySeeded = false;
    var smooth = false;
    // Default OFF: auto-driving delaySlider from the measured packet interval makes the delay FLUCTUATE,
    // which shifts sampleT and visibly pulses rendered vehicle speed (user 2026-07-15: "definitely a bad
    // idea"). A stable manual delay reads far smoother. Still available via the checkbox for the rare case
    // where minimising latency matters more than steadiness.
    var alwaysInterpolate = false;
    var smoother = new DrPoseSmoother();
    var startWall = Stopwatch.StartNew();

    // No local NetworkModel to size the camera from (unlike loopback's host.MinX/MinY/MaxX/MaxY, read
    // straight off EngineHost) -- ViewerHost falls back to an arbitrary placeholder view and this refits
    // ONCE from the received geometry's own bounds the first time it arrives; pan/zoom after that is left to
    // the user.
    var cameraFitted = false;

    // ViewerStatus is a readonly struct -- default is Present=false, which is exactly the "no status sample
    // yet" state DrawImGui/PumpFrame already treat correctly.
    var status = default(ViewerStatus);

    // Set at the end of PumpFrame each frame so DrawImGui's diagnostics panel can report the draw count
    // without the reused `draws` list being in its own scope.
    var lastDrawCount = 0;

    var cfg = new ViewerHostConfig
    {
        WindowTitle = "SumoSharp - native viewer (remote)",
        ScreenshotPath = screenshotPath,
        Frames = frames,

        InitialCameraBounds = () => null,

        PumpFrame = (dt, draws) =>
        {
            frameStats.Add(dt);

            statusReader.Pump();
            status = statusReader.Current;
            // Restart detection: the host's sim time jumping backward means it rebuilt at t=0. Sim time is
            // authoritative and monotonic WITHIN a run, so any real decrease is a restart -- a small threshold
            // (0.5 s, tolerating only minor DDS status reordering) catches restarts from a short-running sim
            // too, where the old 2 s threshold missed. Drop stale vehicle state (reused handles would
            // otherwise mix old + new timelines), re-anchor the DR clock, and re-seed the auto-delay.
            if (status.Present && status.SimTime < lastRemoteSimTime - 0.5)
            {
                subscriber.ResetVehicles();
                drClock.Reset();
                delaySeeded = false;
            }

            lastRemoteSimTime = status.SimTime;

            // "always interpolate" (see the loopback site for the rationale): slew the auto delay toward ~1.5
            // sample-intervals so Resolve interpolates through junctions, without snapping the sample point.
            if (alwaysInterpolate)
            {
                var target = (float)Math.Clamp(drClock.AvgSampleInterval * 1.5, 0.3, 2.0);
                if (!delaySeeded && drClock.AvgSampleInterval > 0.0)
                {
                    delaySlider = target; // snap once at startup (nothing is moving yet) to skip the slew ramp
                    delaySeeded = true;
                }
                else if (Math.Abs(target - delaySlider) > 0.05f) // dead-band: ignore EMA jitter, slew real drift
                {
                    delaySlider += Math.Clamp(target - delaySlider, -0.008f, 0.008f);
                }
            }

            RenderHelpers.PumpAndBuildVehicleDraws(subscriber, drClock, delaySlider, smooth, frameStats, smoother, draws,
                paused: status.Paused);
            lastDrawCount = draws.Count;
        },

        RefitCameraBounds = () =>
        {
            if (!cameraFitted && subscriber.Geometry.Count > 0)
            {
                cameraFitted = true;
                return RenderHelpers.ComputeGeometryBounds(subscriber.Geometry);
            }

            return null;
        },

        DrawWorld = (camera, draws) =>
        {
            Renderer.DrawWorldDds(camera, subscriber.Geometry, subscriber.TlStateByLane, draws);

            if (!subscriber.GeometryComplete)
            {
                Renderer.DrawWaitingOverlay(Raylib.GetScreenWidth(), Raylib.GetScreenHeight(), "waiting for publisher... (no geometry yet)");
            }
        },

        // Click (not a pan) -> command the publisher to drop an obstacle at this world point. With the
        // reverse command channel, a plain click now commands the publisher, so (like local/loopback) a
        // click must be distinguished from a pan-drag -- ViewerHost already does that distinguishing.
        OnWorldClick = (wx, wy) => commandWriter.Send(ViewerCommandKind.InjectObstacle, x: wx, y: wy),

        DrawImGui = showDiagnostics =>
        {
            ViewerControlsPanels.DrawRemoteControlsPanel(commandWriter, status, ref remoteSpeed, ref remoteRandom,
                ref delaySlider, ref smooth, ref alwaysInterpolate, subscriber.Connected, subscriber.GeometryComplete);
            if (showDiagnostics)
            {
                var wallElapsed = startWall.Elapsed.TotalSeconds;
                var ddsSamplesPerSecond = wallElapsed > 0 ? subscriber.TotalVehicleSamplesReceived / wallElapsed : 0.0;
                Renderer.DrawDdsDiagnosticsPanel(frameStats, drClock, ddsSamplesPerSecond, lastDrawCount);
            }
        },

        OnHeadlessExit = () =>
            Console.WriteLine($"DRCLOCK: renderSim={drClock.RenderSim:F3} simRate={drClock.SimRate:F3} backSteps={drClock.BackSteps}"),
    };

    return ViewerHost.Run(cfg);
}
