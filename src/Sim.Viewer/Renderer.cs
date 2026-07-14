using System.Numerics;
using Raylib_cs;
using rlImGui_cs;
using ImGuiNET;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;
using Sim.Viewer.Core;

namespace Sim.Viewer;

// docs/SUMOSHARP-NATIVE-VIEWER.md P0: draws the LOCAL-mode world (roads + oriented vehicles from the
// authoritative SimulationSnapshot -- no dead reckoning, no jitter) plus a minimal ImGui HUD.
//
// Coordinate convention: SUMO world Y is UP; raylib/canvas screen space is Y-DOWN (SUMOSHARP-NATIVE-
// VIEWER.md's "Read first" item 5). Every world point fed to a raylib draw call is passed through
// `Flip` (negates Y) before being handed to raylib; the returned Camera2D's Target/Offset are computed in
// that SAME negated-Y space, so BeginMode2D's pan/zoom (translate by -Target, scale by Zoom, translate by
// +Offset) reproduces HtmlPage.cs's `w2s(x,y) = [x*scale+ox, -y*scale+oy]` exactly, while still using a
// real Camera2D (rather than a hand-rolled screen transform) as instructed. Because thickness/size values
// passed to DrawLineEx/DrawRectanglePro are also given in this same world-unit space, the camera's Zoom
// scales road width and vehicle size for free.
public static class Renderer
{
    private static readonly Color Background = new(14, 17, 22, 255);
    private static readonly Color RoadCasing = new(10, 12, 16, 255);
    private static readonly Color RoadSurface = new(69, 78, 90, 255);
    private static readonly Color RoadInternal = new(42, 48, 56, 255);
    // HtmlPage.cs draw(): 'rgba(200,210,225,0.14)' dashed centreline, 'rgba(150,170,190,0.30)' chevron.
    private static readonly Color LaneDash = new(200, 210, 225, 36);
    private static readonly Color LaneChevron = new(150, 170, 190, 76);
    private static readonly Color ObstacleColor = new(255, 92, 92, 255); // HtmlPage.cs: '#ff5c5c'

    public static Color BackgroundColor => Background;

    // HtmlPage.cs draw()'s speedColor(s): 0 = stopped (red) .. free-flow 13.9 m/s (~50 km/h) = green.
    private static Color SpeedColor(double speedExact)
    {
        var t = Math.Clamp(speedExact / 13.9, 0.0, 1.0);
        var r = (int)Math.Round(230 * (1 - t) + 40 * t);
        var g = (int)Math.Round(70 * (1 - t) + 200 * t);
        return new Color(r, g, 80, 255);
    }

    // HtmlPage.cs draw()'s tlColor(st): SUMO signal char -> colour.
    private static Color TlColor(char state) => state switch
    {
        'G' or 'g' => new Color(63, 185, 80, 255),
        'y' or 'Y' => new Color(227, 179, 65, 255),
        'r' => new Color(248, 81, 73, 255),
        'o' or 'O' or 'u' => new Color(227, 179, 65, 255),
        _ => new Color(139, 148, 158, 255),
    };

    // World (x,y, SUMO Y-up) -> the Y-negated space fed to every raylib draw call under BeginMode2D.
    private static Vector2 Flip(double x, double y) => new((float)x, (float)-y);

    // Fit a Camera2D to the network bounds with a margin, exactly like HtmlPage.cs's `fit(b)`. Target and
    // Offset are expressed in the SAME Y-negated space `Flip` produces, so the Y-flip lives entirely in
    // this one conversion (negate the bounds centre's Y here; negate every drawn point's Y via `Flip`).
    public static Camera2D FitCamera(double minX, double minY, double maxX, double maxY, int screenW, int screenH)
    {
        var boundsW = (float)Math.Max(maxX - minX, 1.0);
        var boundsH = (float)Math.Max(maxY - minY, 1.0);
        var zoom = Math.Min(screenW / boundsW, screenH / boundsH) * 0.9f;
        var centerX = (float)((minX + maxX) / 2.0);
        var centerY = (float)((minY + maxY) / 2.0);

        return new Camera2D
        {
            Target = new Vector2(centerX, -centerY),
            Offset = new Vector2(screenW / 2f, screenH / 2f),
            Rotation = 0f,
            Zoom = zoom,
        };
    }

    // The world is drawn in two halves so a huge static net (e.g. scenarios/_bench/city-15000, ~13k edges)
    // is not re-stroked every frame -- docs/SUMOSHARP-NATIVE-VIEWER-TESTING.md TASK 1. The interactive/perf
    // loop calls DrawStaticWorld once per camera change (baked into a RoadLayerCache RenderTexture) plus
    // DrawDynamicWorld every frame.

    // The camera-static half of the world: background + roads + lane markings. Factored out so RoadLayerCache
    // can bake it into a RenderTexture and re-run it ONLY when the camera changes (pan/zoom), not every frame.
    // Clears the background itself so the baked texture is opaque and its blit fully replaces ClearBackground.
    public static void DrawStaticWorld(Camera2D camera, NetworkModel network)
    {
        Raylib.ClearBackground(Background);
        Raylib.BeginMode2D(camera);

        // Roads: a dark casing under a lighter lane fill, each lane drawn as a stroked polyline along its
        // Shape (HtmlPage.cs draw()'s two-pass casing/surface loop). Thickness is in WORLD units (metres)
        // so the camera's Zoom scales it automatically; the 2.5 px floor is converted to world units by
        // dividing by Zoom, mirroring the browser's pixel-space clamp (`Math.max(1.5, lane.w*cam.scale)`)
        // but bumped from 1.5 to 2.5 px so lanes stay visibly a ROAD (not a hairline) when zoomed way out
        // on a large net (e.g. scenarios/15-reroute).
        foreach (var lane in network.LanesByHandle)
        {
            var shape = lane.Shape;
            if (shape.Count < 2)
            {
                continue;
            }

            var surfaceThick = Math.Max(2.5f / camera.Zoom, (float)lane.Width);
            var casingThick = surfaceThick + 2.5f / camera.Zoom;
            var surfaceColor = lane.Id.StartsWith(':') ? RoadInternal : RoadSurface;

            for (var i = 0; i < shape.Count - 1; i++)
            {
                var (x1, y1) = shape[i];
                var (x2, y2) = shape[i + 1];
                var p1 = Flip(x1, y1);
                var p2 = Flip(x2, y2);
                Raylib.DrawLineEx(p1, p2, casingThick, RoadCasing);
            }

            for (var i = 0; i < shape.Count - 1; i++)
            {
                var (x1, y1) = shape[i];
                var (x2, y2) = shape[i + 1];
                var p1 = Flip(x1, y1);
                var p2 = Flip(x2, y2);
                Raylib.DrawLineEx(p1, p2, surfaceThick, surfaceColor);
            }
        }

        // Lane markings (HtmlPage.cs draw()'s "lane markings" section): a dashed centreline + a travel-
        // direction chevron per DRIVABLE (non-internal) lane -- internal (junction) lanes get neither, same
        // as the browser. The dash pattern [6,7]px and chevron size floor 3px are pixel-space constants in
        // HtmlPage.cs; converted to world units by dividing by Zoom so they read as fixed screen sizes.
        var dashOnWorld = 6f / camera.Zoom;
        var dashOffWorld = 7f / camera.Zoom;
        var dashThickWorld = 1f / camera.Zoom;
        var drawChevrons = camera.Zoom > 1.2f; // HtmlPage.cs: `if(cam.scale > 1.2)`
        var chevronSize = Math.Max(3f / camera.Zoom, 1.3f);

        foreach (var lane in network.LanesByHandle)
        {
            if (lane.Id.StartsWith(':'))
            {
                continue;
            }

            var shape = lane.Shape;
            if (shape.Count < 2)
            {
                continue;
            }

            DrawDashedPolyline(shape, dashOnWorld, dashOffWorld, dashThickWorld, LaneDash);

            if (drawChevrons)
            {
                DrawLaneChevron(shape, chevronSize, LaneChevron);
            }
        }

        Raylib.EndMode2D();
    }

    // The per-frame-dynamic half of the world: traffic-light dots (colour changes every step), vehicles, and
    // injected obstacles. Drawn live every frame over the baked static road layer. Vehicles are passed in as
    // an already-resolved list (`vehicles`) rather than read straight off the snapshot, so the caller can
    // interpolate 10 Hz sim frames up to the 60 Hz render rate -- Program.cs builds it (raw or render-behind
    // interpolated); TL dots and obstacles still come live from the authoritative snapshot/host.
    public static void DrawDynamicWorld(
        Camera2D camera, NetworkModel network, SimulationSnapshot snapshot, EngineHost host,
        IReadOnlyList<DrVehicleDraw> vehicles)
    {
        Raylib.BeginMode2D(camera);

        // Traffic-light signals (HtmlPage.cs draw()'s "traffic-light signals" section): a coloured dot at
        // the end (stop line) of each controlled approach lane, index-aligned over [0, TlCount).
        var tlRadius = Math.Max(2.5f / camera.Zoom, 0.9f);
        for (var i = 0; i < snapshot.TlCount; i++)
        {
            var laneHandle = snapshot.TlLaneHandle[i];
            if (laneHandle < 0 || laneHandle >= network.LanesByHandle.Count)
            {
                continue;
            }

            var tlLane = network.LanesByHandle[laneHandle];
            var tlShape = tlLane.Shape;
            if (tlShape.Count < 1)
            {
                continue;
            }

            var (ex, ey) = tlShape[^1];
            Raylib.DrawCircleV(Flip(ex, ey), tlRadius, TlColor((char)snapshot.TlState[i]));
        }

        DrawVehicleList(camera, vehicles);

        // Obstacles: a red X at each injected obstacle's projected world point (HtmlPage.cs draw()'s
        // "obstacles" section).
        var obstacleHalf = Math.Max(4f / camera.Zoom, 3f);
        var obstacleThick = 2.5f / camera.Zoom;
        foreach (var (ox, oy) in host.ObstaclePoints)
        {
            var c = Flip(ox, oy);
            Raylib.DrawLineEx(new Vector2(c.X - obstacleHalf, c.Y - obstacleHalf), new Vector2(c.X + obstacleHalf, c.Y + obstacleHalf), obstacleThick, ObstacleColor);
            Raylib.DrawLineEx(new Vector2(c.X + obstacleHalf, c.Y - obstacleHalf), new Vector2(c.X - obstacleHalf, c.Y + obstacleHalf), obstacleThick, ObstacleColor);
        }

        Raylib.EndMode2D();
    }

    // Dashed stroke along a polyline's arc length -- ported from HtmlPage.cs draw()'s
    // `ctx.setLineDash([6,7])` centreline pass. The dash phase resets to "on" at the start of each lane
    // (canvas resets phase per beginPath(), and each lane is stroked as its own path there), so this
    // walks `shape` fresh with a local on/off counter rather than a running counter across lanes.
    private static void DrawDashedPolyline(IReadOnlyList<(double X, double Y)> shape, float dashOn, float dashOff, float thick, Color color)
    {
        var onPhase = true;
        var remaining = dashOn;

        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = (float)(x2 - x1);
            var dy = (float)(y2 - y1);
            var segLen = MathF.Sqrt(dx * dx + dy * dy);
            if (segLen < 1e-6f)
            {
                continue;
            }

            var t0 = 0f;
            while (t0 < segLen)
            {
                var take = Math.Min(remaining, segLen - t0);
                if (onPhase)
                {
                    var t1 = t0 + take;
                    var a = Flip(x1 + dx * (t0 / segLen), y1 + dy * (t0 / segLen));
                    var b = Flip(x1 + dx * (t1 / segLen), y1 + dy * (t1 / segLen));
                    Raylib.DrawLineEx(a, b, thick, color);
                }

                t0 += take;
                remaining -= take;
                if (remaining <= 1e-6f)
                {
                    onPhase = !onPhase;
                    remaining = onPhase ? dashOn : dashOff;
                }
            }
        }
    }

    // A small direction chevron at a lane's midpoint, pointing along travel -- ported from
    // HtmlPage.cs draw()'s drawLaneArrow(lane).
    private static void DrawLaneChevron(IReadOnlyList<(double X, double Y)> shape, float size, Color color)
    {
        var n = shape.Count;
        var mi = Math.Max(0, n / 2 - 1);
        var (x1, y1) = shape[mi];
        var (x2, y2) = shape[mi + 1];
        var dx = x2 - x1;
        var dy = y2 - y1;
        if (dx * dx + dy * dy < 1e-9)
        {
            return;
        }

        var mid = Flip((x1 + x2) / 2.0, (y1 + y2) / 2.0);
        // World dir (dx,dy) -> screen dir (dx,-dy) [Flip] -> screen angle, matching HtmlPage.cs's
        // `a = Math.atan2(-(dy/len), dx/len)` (atan2 is scale-invariant, so the /len normalization there
        // is inert here).
        var angle = MathF.Atan2((float)-dy, (float)dx);
        var cosA = MathF.Cos(angle);
        var sinA = MathF.Sin(angle);

        // Local (pre-rotation) triangle pointing along +x, matching HtmlPage.cs's
        // `moveTo(sz,0); lineTo(-sz*0.6,-sz*0.7); lineTo(-sz*0.6,sz*0.7)` -- already in the same Y-down
        // screen convention this renderer's Flip-space uses, so no further axis flip is needed.
        var tip = Rotate(new Vector2(size, 0f), cosA, sinA) + mid;
        var backRight = Rotate(new Vector2(-size * 0.6f, size * 0.7f), cosA, sinA) + mid;
        var backLeft = Rotate(new Vector2(-size * 0.6f, -size * 0.7f), cosA, sinA) + mid;
        Raylib.DrawTriangle(tip, backRight, backLeft, color);
    }

    private static Vector2 Rotate(Vector2 v, float cosA, float sinA) =>
        new(v.X * cosA - v.Y * sinA, v.X * sinA + v.Y * cosA);

    // P1 controls panel (SUMOSHARP-NATIVE-VIEWER.md P1): mode label, restart, clear obstacles, and the
    // random-traffic toggle. Sized explicitly (SetNextWindowSize) so its text is never clipped -- P0's HUD
    // was cut off at the default auto-size. Must be called between rlImGui.Begin()/End() (see Program.cs).
    // `fpsCap` is the render frame-rate cap in fps, with 0 meaning "unlimited"; the radio here mutates it and
    // pushes the change straight to Raylib.SetTargetFPS so the choice takes effect on the next frame. It's a
    // ref because Program.cs owns the value (it sets the initial cap before the loop) -- see RunLocal.
    public static void DrawControlsPanel(EngineHost host, ref int fpsCap, ref bool smooth)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(360, 390), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls");
        ImGui.Text(host.ScenarioMode ? "mode: SCENARIO" : "mode: SANDBOX");
        ImGui.Separator();
        if (ImGui.Button("restart"))
        {
            host.Restart();
        }

        ImGui.SameLine();
        if (ImGui.Button("clear obstacles"))
        {
            host.ClearObstacles();
        }

        ImGui.SameLine();
        if (ImGui.Button(host.IsPaused ? "resume" : "pause"))
        {
            host.SetPaused(!host.IsPaused);
        }

        var randomTraffic = host.RandomTraffic;
        if (ImGui.Checkbox("inject random traffic", ref randomTraffic))
        {
            host.SetRandomTraffic(randomTraffic);
        }

        ImGui.Separator();

        // Playback speed relative to real time (LIVE, no rebuild): 1x = real speed, 3x = triple, etc. The
        // engine may not sustain a high factor at a large fleet (CPU-bound, ~3x max at 10k) -- the render
        // clock paces to the ACTUAL delivered rate (see the diagnostics "actual" line), so the slider is a
        // target, not a promise.
        var speed = (float)host.Speed;
        if (ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time"))
        {
            host.SetSpeed(speed);
        }

        // Sim resolution = 1 / step-length = how finely the engine integrates (SUMO's step-length). Higher Hz
        // = smoother physics + more snapshots to interpolate + better turns, but proportionally MORE compute,
        // so a high resolution at a large fleet just lowers the achievable speed. Changing it REBUILDS the sim
        // from t=0 (step-length is baked into the loaded config), hence discrete buttons, not a live slider.
        var hz = (int)Math.Round(1.0 / host.StepLength);
        ImGui.Text("sim resolution (restarts):");
        if (ImGui.RadioButton("1Hz", hz == 1)) host.SetStepLength(1.0);
        ImGui.SameLine();
        if (ImGui.RadioButton("2Hz", hz == 2)) host.SetStepLength(0.5);
        ImGui.SameLine();
        if (ImGui.RadioButton("5Hz", hz == 5)) host.SetStepLength(0.2);
        ImGui.SameLine();
        if (ImGui.RadioButton("10Hz", hz == 10)) host.SetStepLength(0.1);

        // Render-behind interpolation: blend the two latest authoritative snapshots up to the render rate so
        // vehicles glide instead of teleporting once per sim step. Costs ~one sim-step of latency; off = draw
        // the raw newest snapshot (the old jumpy behaviour), useful for an A/B or exact-frame inspection.
        ImGui.Checkbox("smooth (interpolate)", ref smooth);

        // Render FPS cap: 30 / 60 / unlimited. Rendering at the hundreds of fps the GPU allows for a 10 Hz
        // sim is wasted work, so default to a real cap; "unlimited" (0) stays available for perf measurement.
        ImGui.Text("render fps cap:");
        ImGui.SameLine();
        if (ImGui.RadioButton("30", fpsCap == 30)) { fpsCap = 30; Raylib.SetTargetFPS(30); }
        ImGui.SameLine();
        if (ImGui.RadioButton("60", fpsCap == 60)) { fpsCap = 60; Raylib.SetTargetFPS(60); }
        ImGui.SameLine();
        if (ImGui.RadioButton("unlimited", fpsCap == 0)) { fpsCap = 0; Raylib.SetTargetFPS(0); }

        ImGui.TextWrapped("click a road to drop an obstacle - drag to pan - wheel to zoom - 'd' toggles diagnostics");
        ImGui.End();
    }

    // P1 perf-diagnostics panel (SUMOSHARP-NATIVE-VIEWER.md P1): fps, frame-time min/avg/p99 (from the
    // ~120-sample ring buffer Program.cs maintains), vehicle count, sim time + step. Toggled with 'd',
    // default ON. Sized explicitly so text is never clipped. Must be called between rlImGui.Begin()/End().
    public static void DrawDiagnosticsPanel(
        SimulationSnapshot snapshot, FrameStats frameStats,
        double requestedSpeed, double actualSpeed, double stepLength)
    {
        var (min, avg, p99) = frameStats.Compute();
        ImGui.SetNextWindowPos(new Vector2(10, 410), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(360, 200), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - diagnostics");
        ImGui.Text($"fps: {Raylib.GetFPS()}");
        ImGui.Text($"frame ms  min {min * 1000f:F2}  avg {avg * 1000f:F2}  p99 {p99 * 1000f:F2}");
        ImGui.Separator();
        ImGui.Text($"vehicles: {snapshot.Count}");
        ImGui.Text($"sim time: {snapshot.Time:F1}s   step: {snapshot.StepCount}");
        // Requested vs ACTUAL playback speed: if actual << requested, the engine is CPU-bound at this fleet/
        // resolution and can't run as fast as asked (lower the speed or the sim resolution). upd/s is the
        // real snapshot rate the interpolator has to work with (actual speed / step-length).
        ImGui.Text($"speed: {requestedSpeed:F2}x req / {actualSpeed:F2}x actual");
        ImGui.Text($"sim res: {1.0 / stepLength:F0}Hz  ->  {actualSpeed / stepLength:F1} upd/s");
        // GC collection counts (gen0/1/2), to correlate any periodic frame hiccup with a collection. A
        // climbing gen1/gen2 during hiccups points at GC pressure; flat counters rule GC out as the cause.
        ImGui.Text($"GC gen0/1/2: {GC.CollectionCount(0)} / {GC.CollectionCount(1)} / {GC.CollectionCount(2)}");
        ImGui.End();
    }

    // --- P2b: --mode loopback (DDS geometry + dead-reckoned vehicles) ---

    // One vehicle's already-resolved (dead-reckoned) draw pose -- Program.cs builds this from
    // DrClock.Resolve + PoseResolver.Resolve(dt=0) before calling DrawWorldDds; this method only draws.
    public readonly struct DrVehicleDraw
    {
        public DrVehicleDraw(double frontX, double frontY, float headingDeg, float length, float width, double speedExact)
        {
            FrontX = frontX;
            FrontY = frontY;
            HeadingDeg = headingDeg;
            Length = length;
            Width = width;
            SpeedExact = speedExact;
        }

        public double FrontX { get; }
        public double FrontY { get; }
        public float HeadingDeg { get; }
        public float Length { get; }
        public float Width { get; }
        public double SpeedExact { get; }
    }

    // Draws the LOOPBACK-mode world from DECODED DDS state (subscriber geometry + TL-by-lane + already
    // dead-reckoned vehicle poses) rather than the authoritative Snapshot -- mirrors DrawWorld's visual
    // style (same casing/surface/dash/chevron/speed-colour/TL-dot conventions) so a loopback screenshot
    // reads as visually equivalent to `--mode local` on the same net.
    public static void DrawWorldDds(
        Camera2D camera,
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry,
        IReadOnlyDictionary<int, byte> tlByLane,
        IReadOnlyList<DrVehicleDraw> vehicles)
    {
        Raylib.BeginMode2D(camera);

        foreach (var lane in geometry.Values)
        {
            var pts = lane.Points;
            if (pts.Length < 2)
            {
                continue;
            }

            var surfaceThick = Math.Max(2.5f / camera.Zoom, lane.Width);
            var casingThick = surfaceThick + 2.5f / camera.Zoom;
            var surfaceColor = lane.IsInternal ? RoadInternal : RoadSurface;

            for (var i = 0; i < pts.Length - 1; i++)
            {
                var p1 = Flip(pts[i].X, pts[i].Y);
                var p2 = Flip(pts[i + 1].X, pts[i + 1].Y);
                Raylib.DrawLineEx(p1, p2, casingThick, RoadCasing);
            }

            for (var i = 0; i < pts.Length - 1; i++)
            {
                var p1 = Flip(pts[i].X, pts[i].Y);
                var p2 = Flip(pts[i + 1].X, pts[i + 1].Y);
                Raylib.DrawLineEx(p1, p2, surfaceThick, surfaceColor);
            }
        }

        var dashOnWorld = 6f / camera.Zoom;
        var dashOffWorld = 7f / camera.Zoom;
        var dashThickWorld = 1f / camera.Zoom;
        var drawChevrons = camera.Zoom > 1.2f;
        var chevronSize = Math.Max(3f / camera.Zoom, 1.3f);

        foreach (var lane in geometry.Values)
        {
            if (lane.IsInternal || lane.Points.Length < 2)
            {
                continue;
            }

            DrawDashedPolyline(lane.Points, dashOnWorld, dashOffWorld, dashThickWorld, LaneDash);

            if (drawChevrons)
            {
                DrawLaneChevron(lane.Points, chevronSize, LaneChevron);
            }
        }

        var tlRadius = Math.Max(2.5f / camera.Zoom, 0.9f);
        foreach (var (laneHandle, signal) in tlByLane)
        {
            if (!geometry.TryGetValue(laneHandle, out var tlLane) || tlLane.Points.Length < 1)
            {
                continue;
            }

            var (ex, ey) = tlLane.Points[^1];
            Raylib.DrawCircleV(Flip(ex, ey), tlRadius, TlColor((char)signal));
        }

        DrawVehicleList(camera, vehicles);

        Raylib.EndMode2D();
    }

    // Draw a list of already-resolved vehicle poses as oriented rectangles -- shared by local (DrawDynamicWorld)
    // and DDS (DrawWorldDds), which used identical loops. Each vehicle is sized Length x Width (world metres),
    // positioned at the SUMO FRONT reference point, rotated to match its navi-deg heading (0=N, clockwise), and
    // filled by speed-graded colour (red = stopped, green = free-flow). Caller has already called BeginMode2D.
    //   navi-deg -> world dir (sin,cos) -> screen dir (x,-y flip) -> screen rotation, matching HtmlPage.cs's
    //   `nr = deg*PI/180; sa = atan2(-cos(nr), sin(nr))`. The front sits at the rectangle's (length, width/2)
    //   origin so the body trails behind the front along local -x (browser's `rect(-L,-W/2,L,W)` at the front).
    private static void DrawVehicleList(Camera2D camera, IReadOnlyList<DrVehicleDraw> vehicles)
    {
        for (var i = 0; i < vehicles.Count; i++)
        {
            var v = vehicles[i];
            var front = Flip(v.FrontX, v.FrontY);
            var length = Math.Max(0.5f / camera.Zoom, v.Length);
            var width = Math.Max(0.3f / camera.Zoom, v.Width);

            var nr = v.HeadingDeg * MathF.PI / 180f;
            var sa = MathF.Atan2(-MathF.Cos(nr), MathF.Sin(nr));
            var rotationDeg = sa * 180f / MathF.PI;

            var rec = new Rectangle(front.X, front.Y, length, width);
            var origin = new Vector2(length, width / 2f);
            Raylib.DrawRectanglePro(rec, origin, rotationDeg, SpeedColor(v.SpeedExact));
        }
    }

    // (float X, float Y)[] overloads of DrawDashedPolyline/DrawLaneChevron -- GeometryCodec.LaneGeo.Points
    // is a float array (the wire precision), while the local-mode NetworkModel.Shape is
    // IReadOnlyList<(double X, double Y)>; same drawing logic, adapted to avoid a per-lane conversion alloc.
    private static void DrawDashedPolyline((float X, float Y)[] shape, float dashOn, float dashOff, float thick, Color color)
    {
        var onPhase = true;
        var remaining = dashOn;

        for (var i = 0; i < shape.Length - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var segLen = MathF.Sqrt(dx * dx + dy * dy);
            if (segLen < 1e-6f)
            {
                continue;
            }

            var t0 = 0f;
            while (t0 < segLen)
            {
                var take = Math.Min(remaining, segLen - t0);
                if (onPhase)
                {
                    var t1 = t0 + take;
                    var a = Flip(x1 + dx * (t0 / segLen), y1 + dy * (t0 / segLen));
                    var b = Flip(x1 + dx * (t1 / segLen), y1 + dy * (t1 / segLen));
                    Raylib.DrawLineEx(a, b, thick, color);
                }

                t0 += take;
                remaining -= take;
                if (remaining <= 1e-6f)
                {
                    onPhase = !onPhase;
                    remaining = onPhase ? dashOn : dashOff;
                }
            }
        }
    }

    private static void DrawLaneChevron((float X, float Y)[] shape, float size, Color color)
    {
        var n = shape.Length;
        var mi = Math.Max(0, n / 2 - 1);
        var (x1, y1) = shape[mi];
        var (x2, y2) = shape[mi + 1];
        var dx = x2 - x1;
        var dy = y2 - y1;
        if (dx * dx + dy * dy < 1e-9)
        {
            return;
        }

        var mid = Flip((x1 + x2) / 2.0, (y1 + y2) / 2.0);
        var angle = MathF.Atan2(-dy, dx);
        var cosA = MathF.Cos(angle);
        var sinA = MathF.Sin(angle);

        var tip = Rotate(new Vector2(size, 0f), cosA, sinA) + mid;
        var backRight = Rotate(new Vector2(-size * 0.6f, size * 0.7f), cosA, sinA) + mid;
        var backLeft = Rotate(new Vector2(-size * 0.6f, -size * 0.7f), cosA, sinA) + mid;
        Raylib.DrawTriangle(tip, backRight, backLeft, color);
    }

    // P2b controls panel: mode label, restart/clear/random (identical semantics to DrawControlsPanel,
    // driving the SAME publisher-owned EngineHost), plus the DR delay slider (0 = extrapolate, higher =
    // interpolate) and the smoothing toggle (extrapolation-only low-pass, HtmlPage.cs's `smooth`).
    public static void DrawLoopbackControlsPanel(EngineHost host, ref float delaySeconds, ref bool smooth, ref bool alwaysInterpolate)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(380, 370), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls (loopback)");
        ImGui.Text(host.ScenarioMode ? "mode: SCENARIO" : "mode: SANDBOX");
        ImGui.Separator();
        if (ImGui.Button("restart"))
        {
            host.Restart();
        }

        ImGui.SameLine();
        if (ImGui.Button("clear obstacles"))
        {
            host.ClearObstacles();
        }

        ImGui.SameLine();
        if (ImGui.Button(host.IsPaused ? "resume" : "pause"))
        {
            host.SetPaused(!host.IsPaused);
        }

        var randomTraffic = host.RandomTraffic;
        if (ImGui.Checkbox("inject random traffic", ref randomTraffic))
        {
            host.SetRandomTraffic(randomTraffic);
        }

        // Sim controls (loopback owns the engine, so these apply here, unlike view-only remote). See
        // DrawControlsPanel for the semantics of speed (x real-time, live) and sim resolution (step-length,
        // rebuilds).
        var speed = (float)host.Speed;
        if (ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time"))
        {
            host.SetSpeed(speed);
        }

        // Sim resolution is a sandbox-only control (scenario mode's step-length is fixed by its .sumocfg).
        if (host.ScenarioMode)
        {
            ImGui.Text($"sim resolution: {1.0 / host.StepLength:F0}Hz (scenario-fixed)");
        }
        else
        {
            var hz = (int)Math.Round(1.0 / host.StepLength);
            ImGui.Text("sim resolution (restarts):");
            if (ImGui.RadioButton("1Hz", hz == 1)) host.SetStepLength(1.0);
            ImGui.SameLine();
            if (ImGui.RadioButton("2Hz", hz == 2)) host.SetStepLength(0.5);
            ImGui.SameLine();
            if (ImGui.RadioButton("5Hz", hz == 5)) host.SetStepLength(0.2);
            ImGui.SameLine();
            if (ImGui.RadioButton("10Hz", hz == 10)) host.SetStepLength(0.1);
        }

        ImGui.Separator();
        // "always interpolate" auto-sets the delay slider each frame (Program.cs) to ~1.5x the measured DDS
        // packet interval, so the render clock always sits behind the newest packet -> Resolve always
        // interpolates instead of extrapolating. The manual slider is disabled while it's driving the value.
        ImGui.Checkbox("always interpolate (auto delay)", ref alwaysInterpolate);
        ImGui.BeginDisabled(alwaysInterpolate);
        ImGui.SliderFloat("DR delay (s)", ref delaySeconds, 0f, 1.5f, "%.2f");
        ImGui.EndDisabled();
        ImGui.Checkbox("smooth (extrap only)", ref smooth);
        ImGui.TextWrapped("delay 0 = extrapolate (predict ahead, may snap); raise = interpolate between DDS packets (smooth, delayed)");
        ImGui.End();
    }

    // P2b diagnostics panel: fps/frame-time (same ring buffer as local), plus the DR-clock health readout
    // (renderSim / simRate / back-steps -- should stay 0) and the DDS-path counters.
    public static void DrawDdsDiagnosticsPanel(FrameStats frameStats, DrClock clock, double ddsSamplesPerSecond, int vehicleCount)
    {
        var (min, avg, p99) = frameStats.Compute();
        ImGui.SetNextWindowPos(new Vector2(10, 390), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(380, 210), ImGuiCond.FirstUseEver);
        // Shared by loopback and remote (P3 refactor) -- neither the title nor the content depends on
        // which of the two owns a local publisher, so one panel covers both.
        ImGui.Begin("SumoSharp - diagnostics (dds)");
        ImGui.Text($"fps: {Raylib.GetFPS()}");
        ImGui.Text($"frame ms  min {min * 1000f:F2}  avg {avg * 1000f:F2}  p99 {p99 * 1000f:F2}");
        ImGui.Separator();
        ImGui.Text($"dds samples/s: {ddsSamplesPerSecond:F1}");
        ImGui.Text($"renderSim: {clock.RenderSim:F2}s   simRate: {clock.SimRate:F2}/s");
        ImGui.Text($"clock back-steps: {clock.BackSteps}   delay: {clock.EffectiveDelay:F2}s");
        ImGui.Text($"vehicles: {vehicleCount}");
        ImGui.Text($"GC gen0/1/2: {GC.CollectionCount(0)} / {GC.CollectionCount(1)} / {GC.CollectionCount(2)}");
        ImGui.End();
    }

    // P3 ("remote mode + QoS"): the view-only counterpart to DrawLoopbackControlsPanel -- same DR-delay
    // slider + smoothing toggle, but no restart/clear-obstacles/random-traffic buttons (a remote viewer has
    // no EngineHost to command -- docs/SUMOSHARP-NATIVE-VIEWER.md's "Delegation model": remote is
    // view-only). Adds the two indicators a remote viewer needs that a loopback viewer doesn't: whether the
    // Vehicles topic currently has a matched (live) writer, and whether the durable geometry topic has
    // delivered the whole network yet -- both meaningful only when there's no local publisher to fall back on.
    // `status` is the publisher's live engine state (DdsViewerStatus). It makes the command widgets
    // AUTHORITATIVE (pause label, speed value, tick rate reflect the real host) instead of optimistic, and
    // disables what can't act: all commands until a host is present, and the sim-tick-rate radios unless the
    // host is a sandbox (a scenario's step-length is fixed). `speed`/`random` stay refs -- speed is
    // draggable and re-synced to the host value when idle; random has no status field yet.
    public static void DrawRemoteControlsPanel(
        DdsCommandWriter cmd, ViewerStatus status, ref float speed, ref bool random,
        ref float delaySeconds, ref bool smooth, ref bool alwaysInterpolate, bool connected, bool geometryComplete)
    {
        ImGui.SetNextWindowPos(new Vector2(10, 10), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(380, 380), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - controls (remote)");
        ImGui.Text("mode: REMOTE (drives the publisher via DDS)");
        ImGui.Separator();
        ImGui.Text(connected ? "connected: yes" : "connected: NO (waiting for a publisher)");
        ImGui.Text(geometryComplete ? "geometry: received" : "geometry: waiting...");
        ImGui.Text(status.Present
            ? $"host: {(status.Sandbox ? "SANDBOX" : "SCENARIO")}  {(status.Paused ? "PAUSED" : "running")}  sim {status.SimTime:F0}s  veh {status.VehicleCount}"
            : "host: (no status yet)");
        ImGui.Separator();

        // Command controls need a live host -> disabled until status arrives (nothing to drive otherwise).
        ImGui.BeginDisabled(!status.Present);

        if (ImGui.Button("restart")) cmd.Send(ViewerCommandKind.Restart);
        ImGui.SameLine();
        if (ImGui.Button("clear obstacles")) cmd.Send(ViewerCommandKind.ClearObstacles);
        ImGui.SameLine();
        if (ImGui.Button(status.Paused ? "resume" : "pause")) // label from the HOST's real pause state
        {
            cmd.Send(ViewerCommandKind.Pause, flag: !status.Paused);
        }

        if (ImGui.Checkbox("inject random traffic", ref random))
        {
            cmd.Send(ViewerCommandKind.SetRandomTraffic, flag: random);
        }

        var speedChanged = ImGui.SliderFloat("speed", ref speed, 0.25f, 10f, "%.2fx real-time");
        var speedActive = ImGui.IsItemActive();
        if (speedChanged)
        {
            cmd.Send(ViewerCommandKind.SetSpeed, value: speed);
        }

        // Sim tick rate (= 1/step-length): disabled unless the host is a sandbox (scenario step-length is
        // fixed). Reflects the host's actual step, not an optimistic guess.
        var hz = status.StepLength > 1e-6 ? (int)Math.Round(1.0 / status.StepLength) : 1;
        ImGui.BeginDisabled(!status.Sandbox);
        ImGui.Text("sim tick rate:");
        if (ImGui.RadioButton("1Hz", hz == 1)) cmd.Send(ViewerCommandKind.SetStepLength, value: 1.0);
        ImGui.SameLine();
        if (ImGui.RadioButton("2Hz", hz == 2)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.5);
        ImGui.SameLine();
        if (ImGui.RadioButton("5Hz", hz == 5)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.2);
        ImGui.SameLine();
        if (ImGui.RadioButton("10Hz", hz == 10)) cmd.Send(ViewerCommandKind.SetStepLength, value: 0.1);
        ImGui.EndDisabled();

        ImGui.EndDisabled(); // command controls

        // Re-sync the speed slider to the host's actual value when the user isn't actively dragging it.
        if (!speedActive && status.Present)
        {
            speed = (float)status.Speed;
        }

        // --- these are LOCAL to this viewer (client-side dead-reckoning playout, not engine state) ---
        ImGui.Separator();
        // "always interpolate" auto-sets the delay slider each frame (Program.cs) to ~1.5x the measured DDS
        // packet interval, so the render clock always sits behind the newest packet -> Resolve always
        // interpolates instead of extrapolating. The manual slider is disabled while it's driving the value.
        ImGui.Checkbox("always interpolate (auto delay)", ref alwaysInterpolate);
        ImGui.BeginDisabled(alwaysInterpolate);
        ImGui.SliderFloat("DR delay (s)", ref delaySeconds, 0f, 1.5f, "%.2f");
        ImGui.EndDisabled();
        ImGui.Checkbox("smooth (extrap only)", ref smooth);
        ImGui.TextWrapped("click a road to drop an obstacle (remote). delay 0 = extrapolate; raise = interpolate");
        ImGui.End();
    }

    // P3: a large centered banner shown while the remote viewer has no geometry yet, so a screenshot taken
    // during that window reads unambiguously as "still connecting" rather than "broken" (an empty world
    // would otherwise look identical to a bug). Purely cosmetic -- draws over whatever DrawWorldDds already
    // produced (nothing, until the first geometry chunk arrives).
    public static void DrawWaitingOverlay(int screenW, int screenH, string message)
    {
        var textSize = ImGui.CalcTextSize(message);
        ImGui.SetNextWindowPos(new Vector2((screenW - textSize.X) / 2f - 16f, screenH / 2f - 40f), ImGuiCond.Always);
        ImGui.SetNextWindowBgAlpha(0.85f);
        ImGui.Begin(
            "##waiting-overlay",
            ImGuiWindowFlags.NoTitleBar | ImGuiWindowFlags.NoResize | ImGuiWindowFlags.NoMove |
            ImGuiWindowFlags.NoInputs | ImGuiWindowFlags.AlwaysAutoResize);
        ImGui.Text(message);
        ImGui.End();
    }
}
