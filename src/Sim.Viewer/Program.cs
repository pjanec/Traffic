using System.Diagnostics;
using System.Globalization;
using System.Numerics;
using System.Runtime;
using CycloneDDS.Runtime;
using Raylib_cs;
using rlImGui_cs;
using ImGuiNET;
using Sim.Core;
using Sim.Replication;
using Sim.Viewer;
using Sim.Viewer.Core;

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
var frames = 150;
var delaySeconds = 0.0f;
(double X, double Y)? dropObstacle = null;
double? secondsCap = null;
int? fleet = null;
var perf = false;
double? simRate = null;
double? stepLen = null;

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
        case "--drop-obstacle":
            var parts = args[++i].Split(',');
            dropObstacle = (
                double.Parse(parts[0], CultureInfo.InvariantCulture),
                double.Parse(parts[1], CultureInfo.InvariantCulture));
            break;
        default:
            inputPath ??= args[i];
            break;
    }
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

if (inputPath is null)
{
    Console.Error.WriteLine("Sim.Viewer: missing <scenarioDir|net.xml> argument.");
    return 1;
}

if (mode == "local")
{
    return RunLocal(ResolveNetPath(inputPath), screenshotPath, frames, dropObstacle, fleet, perf, secondsCap, simRate, stepLen);
}

if (mode == "loopback")
{
    return RunLoopback(ResolveNetPath(inputPath), screenshotPath, frames, delaySeconds, dropObstacle);
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

static int RunLocal(string netPath, string? screenshotPath, int frames, (double X, double Y)? dropObstacle,
    int? fleet, bool perf, double? secondsCap, double? simRate, double? stepLen)
{
    var step = stepLen ?? 1.0;
    using var host = fleet is { } fleetCap
        ? new EngineHost(netPath, spawnCap: fleetCap, forceSandbox: true, stepLength: step)
        : new EngineHost(netPath, stepLength: step);

    // Default interactive speed to 1x real-time (what a viewer expects); perf runs keep the 10x fast-forward
    // so the fleet warms up quickly. Either way the panel's "speed" slider overrides live.
    host.SetSpeed(simRate ?? (perf ? 10.0 : 1.0));

    if (dropObstacle is { } drop)
    {
        host.InjectObstacleAtWorld(drop.X, drop.Y);
    }

    var screenW = 1280;
    var screenH = 800;

    Raylib.SetTraceLogLevel(TraceLogLevel.Warning);
    Raylib.SetConfigFlags(ConfigFlags.ResizableWindow); // user-resizable (docs/testing handoff request)
    Raylib.InitWindow(screenW, screenH, "SumoSharp - native viewer (local)");
    if (!Raylib.IsWindowReady())
    {
        Console.Error.WriteLine("Sim.Viewer: window not ready (no display?).");
        return 1;
    }

    // Cap the draw loop at 60 fps. EngineHost's SimulationRunner ticks on its own real-time-paced background
    // thread (targetHz 10) and the random-traffic spawner fires on a real wall-clock Timer (dueTime 500ms,
    // period 900ms) -- an unthrottled headless draw loop can blast through `--frames` in well under a second
    // under Xvfb/software GL, finishing before either has had a chance to run even once. Pacing the render
    // loop to a real frame rate gives both wall-clock-driven systems time to actually produce traffic before
    // the screenshot is taken.
    // Render FPS cap, live-adjustable from the controls panel (30 / 60 / unlimited). Default 60 -- rendering
    // at the hundreds of fps the GPU allows for a 10 Hz sim is wasted work. `--perf` starts unlimited (0) so
    // the measurement sees the true render frame time instead of a flat 16.6 ms. 0 = no cap.
    var fpsCap = perf ? 0 : 60;
    Raylib.SetTargetFPS(fpsCap);

    rlImGui.Setup(darkTheme: true, enableDocking: false);
    var io = ImGui.GetIO();
    io.Fonts.Clear();
    var fontPath = Path.Combine(AppContext.BaseDirectory, "assets", "DejaVuSans.ttf");
    io.Fonts.AddFontFromFileTTF(fontPath, 18f);
    rlImGui.ReloadFonts();

    var camera = Renderer.FitCamera(host.MinX, host.MinY, host.MaxX, host.MaxY, screenW, screenH);

    // TASK 1: cache the static road layer in a RenderTexture, re-baked only on camera change (see
    // RoadLayerCache) -- the ~13k-edge net is otherwise re-stroked every frame.
    using var roadLayer = new RoadLayerCache(screenW, screenH);

    var headless = screenshotPath is not null;
    var frameCount = 0;
    var frameStats = new FrameStats();
    var showDiagnostics = true; // P1: diagnostics panel default ON, toggled with 'd'
    var perfWall = Stopwatch.StartNew();
    var lastPerfLog = 0.0;

    // Render-behind interpolation state. renderClock is a sim-time cursor kept ~one sim-step behind the
    // newest snapshot so each frame blends the two latest authoritative frames (10 Hz sim -> 60 Hz render
    // without teleporting). vehicleDraws/prevIndex are reused every frame (Clear keeps capacity) so the
    // smoothing adds no steady-state allocation. `smooth` is the panel toggle (default on).
    var renderClock = 0.0;
    var smooth = true;
    // Pre-size to the fleet so neither grows (and reallocates) while the sim warms up from 0 to ~10k -- that
    // per-warmup-frame growth was itself a source of early allocations/stutter.
    var interpCap = Math.Max(fleet ?? 0, 256);
    var vehicleDraws = new List<Renderer.DrVehicleDraw>(interpCap);
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

    // P1 drag-vs-click bookkeeping for the world camera (Camera2D pan/zoom/pick -- see Renderer.Flip's
    // doc comment for the world<->screen convention this camera operates in).
    var dragging = false;
    var dragMoved = false;
    var dragStartMouse = Vector2.Zero;
    var dragStartTarget = Vector2.Zero;
    const float DragMoveThreshold = 3f; // px: below this, mouseup is a CLICK (pick), not a pan.

    while (!Raylib.WindowShouldClose())
    {
        frameStats.Add(Raylib.GetFrameTime());

        if (Raylib.IsKeyPressed(KeyboardKey.D))
        {
            showDiagnostics = !showDiagnostics;
        }

        // Window resized: match the offscreen road layer to the new framebuffer and re-centre the camera
        // offset so the current view stays put (pan/zoom otherwise unchanged). Guard against a minimized /
        // degenerate 0-size framebuffer -- LoadRenderTexture(0,0) segfaults the GL driver.
        if (Raylib.IsWindowResized() && !Raylib.IsWindowMinimized())
        {
            var w = Raylib.GetScreenWidth();
            var h = Raylib.GetScreenHeight();
            if (w > 0 && h > 0)
            {
                screenW = w;
                screenH = h;
                roadLayer.Resize(w, h);
                camera.Offset = new Vector2(w / 2f, h / 2f);
            }
        }

        Raylib.BeginDrawing();
        Raylib.ClearBackground(Renderer.BackgroundColor);

        // rlImGui.Begin() (ImGui NewFrame) must run before reading io.WantCaptureMouse for this frame's
        // world-input gate (an ImGui window/button under the cursor should eat clicks/drags/wheel rather
        // than also panning/picking the world underneath it).
        rlImGui.Begin();
        var wantMouse = ImGui.GetIO().WantCaptureMouse;

        if (!wantMouse)
        {
            var mouse = Raylib.GetMousePosition();

            if (Raylib.IsMouseButtonPressed(MouseButton.Left))
            {
                dragging = true;
                dragMoved = false;
                dragStartMouse = mouse;
                dragStartTarget = camera.Target;
            }

            if (dragging && Raylib.IsMouseButtonDown(MouseButton.Left))
            {
                var delta = mouse - dragStartMouse;
                if (delta.Length() > DragMoveThreshold)
                {
                    dragMoved = true;
                }

                // Pan: drag the WORLD with the cursor (moving the mouse right reveals content to the
                // left), so Target moves opposite the screen-space drag delta, scaled back to world units.
                camera.Target = dragStartTarget - delta / camera.Zoom;
            }

            if (Raylib.IsMouseButtonReleased(MouseButton.Left))
            {
                if (dragging && !dragMoved)
                {
                    // A click, not a pan -> invert the camera (then Flip) to get the WORLD point under the
                    // cursor and inject an obstacle there.
                    var flipSpace = Raylib.GetScreenToWorld2D(mouse, camera);
                    host.InjectObstacleAtWorld(flipSpace.X, -flipSpace.Y);
                }

                dragging = false;
            }

            var wheel = Raylib.GetMouseWheelMove();
            if (wheel != 0f)
            {
                // Zoom about the cursor: the world point under the cursor (in Flip-space) must land back
                // under the cursor after the zoom, so re-derive Target from the pre-zoom world point.
                var beforeZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                var zoomFactor = wheel > 0 ? 1.1f : 1f / 1.1f;
                camera.Zoom *= zoomFactor;
                var afterZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                camera.Target += beforeZoom - afterZoom;
            }
        }

        // Atomic (cur, prev) pair. Interpolate by the fraction of the measured snapshot-arrival interval
        // elapsed since `cur` arrived -> the render pose sweeps prev->cur at constant velocity over the
        // interval, matching however fast the engine is actually producing frames. (Estimating an
        // instantaneous rate from the impulsive per-frame sim-time deltas produced a sawtooth: race to the
        // newest frame then freeze until the next arrived.)
        var (snapshot, prevSnapshot) = host.SnapshotPair;
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
            // frac past 1.0 means the next snapshot is running late (variable per-tick compute time). Rather
            // than freeze at `cur` until it lands (a once-per-snapshot stall), allow a small bounded
            // extrapolation beyond it (frac up to 1.25 => 0.25 step ahead) to bridge the gap smoothly; it
            // only engages when the sim is late, so on-time frames (incl. turns) never extrapolate.
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
        var actualSpeed = intervalEma > 1e-6 ? span / intervalEma : host.Speed;
        BuildLocalVehicleDraws(snapshot, prevSnapshot, renderClock, smooth, vehicleDraws, prevIndex,
            headingPrev, headingCur, Raylib.GetFrameTime());
        (headingPrev, headingCur) = (headingCur, headingPrev); // swap: headingCur becomes next frame's prior

        roadLayer.EnsureAndBlit(camera, cam => Renderer.DrawStaticWorld(cam, host.Network));
        Renderer.DrawDynamicWorld(camera, host.Network, snapshot, host, vehicleDraws);

        Renderer.DrawControlsPanel(host, ref fpsCap, ref smooth);
        if (showDiagnostics)
        {
            Renderer.DrawDiagnosticsPanel(snapshot, frameStats, host.Speed, actualSpeed, host.StepLength);
        }

        rlImGui.End();

        Raylib.EndDrawing();

        frameCount++;
        if (headless && frameCount >= frames)
        {
            ExportScreenshot(screenshotPath!);
            break;
        }

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
                break;
            }
        }
    }

    rlImGui.Shutdown();
    Raylib.CloseWindow();
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
        // Restart (remote command) rebuilds at t=0: sim time drops, so the `> lastPublishedSimTime` gate
        // would suppress all publishing until it climbed back up -> the remote stays empty after a restart.
        // Reset the gate so the fresh timeline republishes immediately.
        if (!double.IsNaN(lastPublishedSimTime) && snap.Time < lastPublishedSimTime - 2.0)
        {
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

// docs/SUMOSHARP-NATIVE-VIEWER.md P2b — one process runs BOTH the publisher (EngineHost -> DdsPublisher)
// and the subscriber/renderer, over DDS intra-host. Renders the DEAD-RECKONED poses coming through DDS
// (DrClock + PoseResolver against the SUBSCRIBER's decoded geometry/history), not the local Snapshot --
// the single-app DR test the design doc's "loopback" mode exists for.
static int RunLoopback(string netPath, string? screenshotPath, int frames, float initialDelaySeconds, (double X, double Y)? dropObstacle)
{
    using var host = new EngineHost(netPath);
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

    var screenW = 1280;
    var screenH = 800;

    Raylib.SetTraceLogLevel(TraceLogLevel.Warning);
    Raylib.SetConfigFlags(ConfigFlags.ResizableWindow);
    Raylib.InitWindow(screenW, screenH, "SumoSharp - native viewer (loopback DR)");
    if (!Raylib.IsWindowReady())
    {
        Console.Error.WriteLine("Sim.Viewer: window not ready (no display?).");
        return 1;
    }

    Raylib.SetTargetFPS(60);

    rlImGui.Setup(darkTheme: true, enableDocking: false);
    var io = ImGui.GetIO();
    io.Fonts.Clear();
    var fontPath = Path.Combine(AppContext.BaseDirectory, "assets", "DejaVuSans.ttf");
    io.Fonts.AddFontFromFileTTF(fontPath, 18f);
    rlImGui.ReloadFonts();

    // Same net -> same bounds whether read locally (EngineHost.Network) or over DDS; local bounds are
    // already available without waiting on the subscriber, so the camera fit doesn't need to block further.
    var camera = Renderer.FitCamera(host.MinX, host.MinY, host.MaxX, host.MaxY, screenW, screenH);

    var headless = screenshotPath is not null;
    var frameCount = 0;
    var frameStats = new FrameStats();
    var showDiagnostics = true;

    var dragging = false;
    var dragMoved = false;
    var dragStartMouse = Vector2.Zero;
    var dragStartTarget = Vector2.Zero;
    const float DragMoveThreshold = 3f;

    var drClock = new DrClock();
    var delaySlider = initialDelaySeconds;
    var smooth = false;
    var smoothed = new Dictionary<VehicleHandle, (float X, float Y, float Deg)>();
    var lastPublishedSimTime = double.NaN;
    var lastLoopSimTime = 0.0;
    var startWall = Stopwatch.StartNew();

    var vehicleDraws = new List<Renderer.DrVehicleDraw>();

    while (!Raylib.WindowShouldClose())
    {
        frameStats.Add(Raylib.GetFrameTime());

        if (Raylib.IsKeyPressed(KeyboardKey.D))
        {
            showDiagnostics = !showDiagnostics;
        }

        if (Raylib.IsWindowResized() && !Raylib.IsWindowMinimized())
        {
            var w = Raylib.GetScreenWidth();
            var h = Raylib.GetScreenHeight();
            if (w > 0 && h > 0) { screenW = w; screenH = h; camera.Offset = new Vector2(w / 2f, h / 2f); }
        }

        // Publish at the SIM cadence (gated on the snapshot's own Time advancing), not the 60 Hz render
        // cadence -- EngineHost's SimulationRunner ticks in the background at its own targetHz, so most
        // render frames see an unchanged snapshot and would otherwise re-publish identical state.
        var snapTimeNow = host.Snapshot.Time;
        // Restart detection (local): sim time jumping backward means Restart rebuilt at t=0. Drop stale
        // vehicle state + re-anchor the clock, and reset the publish gate so the fresh timeline republishes
        // (otherwise `> lastPublishedSimTime` would suppress it until sim time climbed back up).
        if (snapTimeNow < lastLoopSimTime - 2.0)
        {
            subscriber.ResetVehicles();
            drClock.Reset();
            lastPublishedSimTime = double.NaN;
        }

        lastLoopSimTime = snapTimeNow;
        if (double.IsNaN(lastPublishedSimTime) || snapTimeNow > lastPublishedSimTime)
        {
            publisher.PublishStep();
            lastPublishedSimTime = snapTimeNow;
        }

        // P3 refactor: pumping DDS + resolving each tracked vehicle's dead-reckoned draw pose doesn't
        // depend on this frame's camera input, so it's hoisted before the input/drawing block into a
        // function shared with `--mode remote` (PumpAndBuildVehicleDraws below) -- same math, same DR
        // resolve, same smoothing, just called from one place instead of duplicated per mode.
        PumpAndBuildVehicleDraws(subscriber, drClock, delaySlider, smooth, frameStats, smoothed, vehicleDraws,
            paused: host.IsPaused);

        Raylib.BeginDrawing();
        Raylib.ClearBackground(Renderer.BackgroundColor);

        rlImGui.Begin();
        var wantMouse = ImGui.GetIO().WantCaptureMouse;

        if (!wantMouse)
        {
            var mouse = Raylib.GetMousePosition();

            if (Raylib.IsMouseButtonPressed(MouseButton.Left))
            {
                dragging = true;
                dragMoved = false;
                dragStartMouse = mouse;
                dragStartTarget = camera.Target;
            }

            if (dragging && Raylib.IsMouseButtonDown(MouseButton.Left))
            {
                var delta = mouse - dragStartMouse;
                if (delta.Length() > DragMoveThreshold)
                {
                    dragMoved = true;
                }

                camera.Target = dragStartTarget - delta / camera.Zoom;
            }

            if (Raylib.IsMouseButtonReleased(MouseButton.Left))
            {
                if (dragging && !dragMoved)
                {
                    var flipSpace = Raylib.GetScreenToWorld2D(mouse, camera);
                    host.InjectObstacleAtWorld(flipSpace.X, -flipSpace.Y);
                }

                dragging = false;
            }

            var wheel = Raylib.GetMouseWheelMove();
            if (wheel != 0f)
            {
                var beforeZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                var zoomFactor = wheel > 0 ? 1.1f : 1f / 1.1f;
                camera.Zoom *= zoomFactor;
                var afterZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                camera.Target += beforeZoom - afterZoom;
            }
        }

        Renderer.DrawWorldDds(camera, subscriber.Geometry, subscriber.TlStateByLane, vehicleDraws);

        Renderer.DrawLoopbackControlsPanel(host, ref delaySlider, ref smooth);
        if (showDiagnostics)
        {
            var wallElapsed = startWall.Elapsed.TotalSeconds;
            var ddsSamplesPerSecond = wallElapsed > 0 ? subscriber.TotalVehicleSamplesReceived / wallElapsed : 0.0;
            Renderer.DrawDdsDiagnosticsPanel(frameStats, drClock, ddsSamplesPerSecond, vehicleDraws.Count);
        }

        rlImGui.End();

        Raylib.EndDrawing();

        frameCount++;
        if (headless && frameCount >= frames)
        {
            ExportScreenshot(screenshotPath!);
            Console.WriteLine($"DRCLOCK: renderSim={drClock.RenderSim:F3} simRate={drClock.SimRate:F3} backSteps={drClock.BackSteps}");
            break;
        }
    }

    rlImGui.Shutdown();
    Raylib.CloseWindow();
    return 0;
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

    var screenW = 1280;
    var screenH = 800;

    Raylib.SetTraceLogLevel(TraceLogLevel.Warning);
    Raylib.SetConfigFlags(ConfigFlags.ResizableWindow);
    Raylib.InitWindow(screenW, screenH, "SumoSharp - native viewer (remote)");
    if (!Raylib.IsWindowReady())
    {
        Console.Error.WriteLine("Sim.Viewer: window not ready (no display?).");
        return 1;
    }

    Raylib.SetTargetFPS(60);

    rlImGui.Setup(darkTheme: true, enableDocking: false);
    var io = ImGui.GetIO();
    io.Fonts.Clear();
    var fontPath = Path.Combine(AppContext.BaseDirectory, "assets", "DejaVuSans.ttf");
    io.Fonts.AddFontFromFileTTF(fontPath, 18f);
    rlImGui.ReloadFonts();

    // No local NetworkModel to size the camera from (unlike loopback's host.MinX/MinY/MaxX/MaxY, read
    // straight off EngineHost) -- start on an arbitrary placeholder view and re-fit ONCE from the received
    // geometry's own bounds the first time it arrives; pan/zoom after that is left to the user.
    var camera = Renderer.FitCamera(-50, -50, 50, 50, screenW, screenH);
    var cameraFitted = false;

    var headless = screenshotPath is not null;
    var frameCount = 0;
    var frameStats = new FrameStats();
    var showDiagnostics = true;

    // With the reverse command channel, a plain click now commands the publisher to drop an obstacle at that
    // world point, so (like local/loopback) a click must be distinguished from a pan-drag.
    var dragging = false;
    var dragMoved = false;
    var dragStartMouse = Vector2.Zero;
    var dragStartTarget = Vector2.Zero;
    const float DragMoveThreshold = 3f;

    // Optimistic local mirror of the commanded engine state (a view-only remote gets no state feedback, so
    // the control widgets reflect what we've SENT). Initialised to the publisher's own interactive defaults.
    var remoteSpeed = 1f;
    var remoteRandom = false;
    var lastRemoteSimTime = 0.0;

    var drClock = new DrClock();
    var delaySlider = initialDelaySeconds;
    var smooth = false;
    var smoothed = new Dictionary<VehicleHandle, (float X, float Y, float Deg)>();
    var startWall = Stopwatch.StartNew();

    var vehicleDraws = new List<Renderer.DrVehicleDraw>();

    while (!Raylib.WindowShouldClose())
    {
        frameStats.Add(Raylib.GetFrameTime());

        if (Raylib.IsKeyPressed(KeyboardKey.D))
        {
            showDiagnostics = !showDiagnostics;
        }

        if (Raylib.IsWindowResized() && !Raylib.IsWindowMinimized())
        {
            var w = Raylib.GetScreenWidth();
            var h = Raylib.GetScreenHeight();
            if (w > 0 && h > 0) { screenW = w; screenH = h; camera.Offset = new Vector2(w / 2f, h / 2f); }
        }

        statusReader.Pump();
        var status = statusReader.Current;
        // Restart detection: the host's sim time jumping backward means it rebuilt at t=0. Drop stale vehicle
        // state (reused handles would otherwise mix old + new timelines) and re-anchor the DR clock.
        if (status.Present && status.SimTime < lastRemoteSimTime - 2.0)
        {
            subscriber.ResetVehicles();
            drClock.Reset();
        }

        lastRemoteSimTime = status.SimTime;
        PumpAndBuildVehicleDraws(subscriber, drClock, delaySlider, smooth, frameStats, smoothed, vehicleDraws,
            paused: status.Paused);

        if (!cameraFitted && subscriber.Geometry.Count > 0)
        {
            var (minX, minY, maxX, maxY) = ComputeGeometryBounds(subscriber.Geometry);
            camera = Renderer.FitCamera(minX, minY, maxX, maxY, screenW, screenH);
            cameraFitted = true;
        }

        Raylib.BeginDrawing();
        Raylib.ClearBackground(Renderer.BackgroundColor);

        rlImGui.Begin();
        var wantMouse = ImGui.GetIO().WantCaptureMouse;

        if (!wantMouse)
        {
            var mouse = Raylib.GetMousePosition();

            if (Raylib.IsMouseButtonPressed(MouseButton.Left))
            {
                dragging = true;
                dragMoved = false;
                dragStartMouse = mouse;
                dragStartTarget = camera.Target;
            }

            if (dragging && Raylib.IsMouseButtonDown(MouseButton.Left))
            {
                var delta = mouse - dragStartMouse;
                if (delta.Length() > DragMoveThreshold)
                {
                    dragMoved = true;
                }

                camera.Target = dragStartTarget - delta / camera.Zoom;
            }

            if (Raylib.IsMouseButtonReleased(MouseButton.Left))
            {
                if (dragging && !dragMoved)
                {
                    // Click (not a pan) -> command the publisher to drop an obstacle at this world point.
                    var flipSpace = Raylib.GetScreenToWorld2D(mouse, camera);
                    commandWriter.Send(ViewerCommandKind.InjectObstacle, x: flipSpace.X, y: -flipSpace.Y);
                }

                dragging = false;
            }

            var wheel = Raylib.GetMouseWheelMove();
            if (wheel != 0f)
            {
                var beforeZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                var zoomFactor = wheel > 0 ? 1.1f : 1f / 1.1f;
                camera.Zoom *= zoomFactor;
                var afterZoom = Raylib.GetScreenToWorld2D(mouse, camera);
                camera.Target += beforeZoom - afterZoom;
            }
        }

        Renderer.DrawWorldDds(camera, subscriber.Geometry, subscriber.TlStateByLane, vehicleDraws);

        if (!subscriber.GeometryComplete)
        {
            Renderer.DrawWaitingOverlay(screenW, screenH, "waiting for publisher... (no geometry yet)");
        }

        Renderer.DrawRemoteControlsPanel(commandWriter, status, ref remoteSpeed, ref remoteRandom,
            ref delaySlider, ref smooth, subscriber.Connected, subscriber.GeometryComplete);
        if (showDiagnostics)
        {
            var wallElapsed = startWall.Elapsed.TotalSeconds;
            var ddsSamplesPerSecond = wallElapsed > 0 ? subscriber.TotalVehicleSamplesReceived / wallElapsed : 0.0;
            Renderer.DrawDdsDiagnosticsPanel(frameStats, drClock, ddsSamplesPerSecond, vehicleDraws.Count);
        }

        rlImGui.End();

        Raylib.EndDrawing();

        frameCount++;
        if (headless && frameCount >= frames)
        {
            ExportScreenshot(screenshotPath!);
            Console.WriteLine($"DRCLOCK: renderSim={drClock.RenderSim:F3} simRate={drClock.SimRate:F3} backSteps={drClock.BackSteps}");
            break;
        }
    }

    rlImGui.Shutdown();
    Raylib.CloseWindow();
    return 0;
}

// P3 refactor: the DDS-pump + dead-reckoned-pose-resolve step shared by `--mode loopback` and `--mode
// remote` -- identical math to the block this replaces in each (DrClock.Resolve + PoseResolver.Resolve(dt=0)
// + the extrapolation-only smoothing low-pass), just called from one place. Mutates `vehicleDraws` (cleared
// and repopulated) and `smoothed` (the low-pass's per-vehicle running state) in place.
static void PumpAndBuildVehicleDraws(
    DdsSubscriber subscriber,
    DrClock drClock,
    float delaySeconds,
    bool smooth,
    FrameStats frameStats,
    Dictionary<VehicleHandle, (float X, float Y, float Deg)> smoothed,
    List<Renderer.DrVehicleDraw> vehicleDraws,
    bool paused)
{
    subscriber.Pump();
    drClock.Pump(subscriber.LatestVehicleSampleTime, hold: paused);

    vehicleDraws.Clear();
    var geoSource = new DdsGeometryLaneSource(subscriber.Geometry);
    Span<int> upcomingScratch = stackalloc int[UpcomingLanes.Count];

    foreach (var (handle, history) in subscriber.History)
    {
        if (history.Count == 0)
        {
            continue;
        }

        var resolved = drClock.Resolve(history, delaySeconds);
        var (length, width) = subscriber.Dims.TryGetValue(handle, out var dims) ? dims : (5.0f, 1.8f);
        var state = resolved.State with { Length = length, Width = width };

        var upCount = resolved.Upcoming.CopyTo(upcomingScratch);
        var pose = PoseResolver.Resolve(
            geoSource, state, upcomingScratch[..upCount], default, 0.0, RenderRealism.CornerCutCorrected);

        var px = pose.X;
        var py = pose.Y;
        var pdeg = pose.HeadingDeg;

        // Optional low-pass, extrapolation-only (HtmlPage.cs's `smooth`): interpolated poses are already
        // smooth/exact, so only extrapolated ones are filtered.
        if (smooth && resolved.Extrapolated)
        {
            var (min, avg, _) = frameStats.Compute();
            var frameDt = avg > 0f ? avg : 1f / 60f;
            var aPos = 1f - MathF.Exp(-frameDt / 0.07f);
            var aDeg = 1f - MathF.Exp(-frameDt / 0.06f);

            if (smoothed.TryGetValue(handle, out var prev))
            {
                var ex = (float)px - prev.X;
                var ey = (float)py - prev.Y;
                if (ex * ex + ey * ey > 49f)
                {
                    smoothed[handle] = ((float)px, (float)py, pdeg);
                }
                else
                {
                    var nx = prev.X + ex * aPos;
                    var ny = prev.Y + ey * aPos;
                    var dd = ((pdeg - prev.Deg + 540f) % 360f) - 180f;
                    var nd = Math.Abs(dd) > 50f ? pdeg : (prev.Deg + dd * aDeg + 360f) % 360f;
                    smoothed[handle] = (nx, ny, nd);
                }
            }
            else
            {
                smoothed[handle] = ((float)px, (float)py, pdeg);
            }

            (px, py, pdeg) = smoothed[handle];
        }
        else
        {
            smoothed[handle] = ((float)px, (float)py, pdeg);
        }

        vehicleDraws.Add(new Renderer.DrVehicleDraw(px, py, pdeg, length, width, state.Speed));
    }
}

// P3: remote has no local NetworkModel to size its initial camera fit from (unlike loopback's
// host.MinX/MinY/MaxX/MaxY) -- computed once, the first time geometry arrives, from the received lane
// polylines' own bounding box. Falls back to a small placeholder box if called with no geometry yet (should
// not happen given the caller's `Count > 0` guard, but degrades safely rather than throwing on empty input).
static (double MinX, double MinY, double MaxX, double MaxY) ComputeGeometryBounds(
    IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
{
    var minX = double.PositiveInfinity;
    var minY = double.PositiveInfinity;
    var maxX = double.NegativeInfinity;
    var maxY = double.NegativeInfinity;

    foreach (var lane in geometry.Values)
    {
        foreach (var (x, y) in lane.Points)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }
    }

    if (double.IsPositiveInfinity(minX))
    {
        return (-50, -50, 50, 50);
    }

    return (minX, minY, maxX, maxY);
}

// Render-behind interpolation for `--mode local`: fill `outDraws` with each current vehicle's draw pose,
// blended from the matching vehicle in the previous authoritative frame toward the current one by the clock
// fraction `a`. O(n) via a reused handle->index map (SimulationSnapshot.TryGetVehicle is a linear scan, so
// per-vehicle lookups would be O(n^2) at 10k). Reuses `outDraws`/`prevIndex` (Clear keeps capacity) so a
// warmed steady state allocates nothing. Falls back to the raw current frame when there's no distinct prior
// frame to blend from (first frame / just after a restart / smoothing off).
static void BuildLocalVehicleDraws(
    SimulationSnapshot cur, SimulationSnapshot prev, double renderClock, bool smooth,
    List<Renderer.DrVehicleDraw> outDraws, Dictionary<VehicleHandle, int> prevIndex,
    Dictionary<VehicleHandle, float> headingPrev, Dictionary<VehicleHandle, float> headingCur,
    float frameDtWall)
{
    outDraws.Clear();
    headingCur.Clear(); // rebuilt from current vehicles only -> prunes despawned, stays bounded

    var span = cur.Time - prev.Time;
    var interp = smooth && span > 1e-9 && prev.Count > 0 && !ReferenceEquals(prev, cur);

    // Heading low-pass coefficient (this frame). A junction's internal lane is a coarse ~5-point arc
    // (netconvert internal-link-detail=5), so the raw/interpolated heading rotates in ~5 discrete bursts; a
    // ~0.18s low-pass on the RENDER heading spreads them into one continuous rotation. Straights (constant
    // heading) converge instantly -> no lag; only actual rotation is smoothed.
    var headAlpha = smooth ? 1f - MathF.Exp(-frameDtWall / 0.25f) : 1f;

    var a = 0f;
    if (interp)
    {
        // [0, 1.25]: >1 is the small bounded extrapolation the render clock uses to bridge a late snapshot
        // (see the clock in RunLocal) -- lerp beyond `cur` predicts slightly ahead instead of freezing.
        a = (float)((renderClock - prev.Time) / span);
        if (a < 0f) a = 0f;
        else if (a > 1.25f) a = 1.25f;

        prevIndex.Clear();
        for (var j = 0; j < prev.Count; j++)
        {
            prevIndex[prev.Handles[j]] = j;
        }
    }

    for (var i = 0; i < cur.Count; i++)
    {
        var handle = cur.Handles[i];
        float fx = cur.PosX[i], fy = cur.PosY[i], deg = cur.Angle[i];
        double spd = cur.SpeedExact[i];

        // Match by handle (spawn/despawn shuffles column indices between frames). A vehicle absent from the
        // previous frame (just departed) is drawn at its raw current pose.
        if (interp && prevIndex.TryGetValue(handle, out var j))
        {
            fx = prev.PosX[j] + (cur.PosX[i] - prev.PosX[j]) * a;
            fy = prev.PosY[j] + (cur.PosY[i] - prev.PosY[j]) * a;
            deg = LerpAngleDeg(prev.Angle[j], cur.Angle[i], a);
            spd = prev.SpeedExact[j] + (cur.SpeedExact[i] - prev.SpeedExact[j]) * a;
        }

        // Low-pass the render heading toward `deg` from last frame's smoothed value (shortest arc). A big
        // jump (respawn / opposite-direction reuse of a handle) snaps rather than spins.
        if (smooth && headingPrev.TryGetValue(handle, out var prevDeg))
        {
            var d = ((deg - prevDeg + 540f) % 360f) - 180f;
            deg = MathF.Abs(d) > 100f ? deg : (prevDeg + d * headAlpha + 360f) % 360f;
        }

        headingCur[handle] = deg;
        outDraws.Add(new Renderer.DrVehicleDraw(fx, fy, deg, cur.Length[i], cur.Width[i], spd));
    }
}

// Interpolate degrees along the shortest arc, so a heading crossing 0/360 (e.g. 350 -> 10) turns the short
// way -- mirrors SimulationRunner.LerpAngleDeg (kept local; the runner's is internal). Straight lerp of
// position + shortest-arc lerp of heading renders a smooth turn through a 90-deg junction; at 10 Hz the
// chord-vs-arc position error between adjacent frames is sub-metre, so long vehicles round corners cleanly.
static float LerpAngleDeg(float from, float to, float t)
{
    var delta = (to - from) % 360f;
    if (delta > 180f) delta -= 360f;
    else if (delta < -180f) delta += 360f;
    var r = (from + delta * t) % 360f;
    return r < 0f ? r + 360f : r;
}

// NOT Raylib.TakeScreenshot(path): raylib's TakeScreenshot silently drops the directory portion of `path`
// (it saves GetFileName(path) under its internal storage/base path, i.e. the process's working directory
// at InitWindow time) -- confirmed experimentally: an absolute path like "/tmp/p0-cross.png" landed at
// "<cwd>/p0-cross.png" instead. LoadImageFromScreen + ExportImage writes to the exact path given, honoring
// an absolute `--screenshot` path as the CLI promises.
static void ExportScreenshot(string screenshotPath)
{
    var absolutePath = Path.GetFullPath(screenshotPath);
    var screenImage = Raylib.LoadImageFromScreen();
    Raylib.ExportImage(screenImage, absolutePath);
    Raylib.UnloadImage(screenImage);
}
