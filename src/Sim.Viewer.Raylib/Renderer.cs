using System.Numerics;
using rlImGui_cs;
using ImGuiNET;
using Sim.Core;
using Sim.Ingest;
using Sim.Replication;
using Sim.Viewer.Motion;

namespace Sim.Viewer.Raylib;

// `using Raylib_cs;` is deliberately placed HERE (namespace-body level), not above the namespace
// declaration (compilation-unit level): this namespace's own trailing segment is "Raylib", which is a
// nested-namespace member of Sim.Viewer as seen from this file's compilation unit, and C# simple-name
// lookup resolves a same-name nested-namespace member of an ENCLOSING namespace before it ever
// considers a compilation-unit-level using-namespace-directive -- so an unqualified `Raylib.XXX` call
// resolved to the "Sim.Viewer.Raylib" namespace itself (CS0234) rather than the Raylib_cs static class.
// A using-directive declared AT THIS namespace's own body level is checked (and matches) before that
// outer-level lookup is ever reached.
using Raylib_cs;

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
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10/D5 (P3.2/P3.3): `public` (not `private`) so a render
    // overlay in a CONSUMING assembly (e.g. the demo exe's EvacOverlay, in Sim.Viewer, fear-overpaint)
    // can reproduce the EXACT plain speed colour it is blending toward alarm-red, without Renderer
    // knowing why. Renderer now lives in the separate Sim.Viewer.Raylib package, so `internal` would no
    // longer be visible to that overlay.
    public static Color SpeedColor(double speedExact)
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
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10/D5 (P3.2/P3.3): `public` so a render overlay in a
    // CONSUMING assembly (e.g. the demo exe's EvacOverlay) can draw in the exact same world<->screen
    // convention as every generic draw call in this file.
    public static Vector2 Flip(double x, double y) => new((float)x, (float)-y);

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
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D5/§5 (P3.3): takes `obstaclePoints` (plain world points), not an
    // `EngineHost` -- the generic packaged renderer must not depend on Sim.Viewer.Core (the demo-side,
    // non-packable viewer brain; Tier-2 diagram §5 lists this package's deps as Viewer.Motion +
    // Replication.Dds only). Callers that own an EngineHost pass `host.ObstaclePoints`.
    public static void DrawDynamicWorld(
        Camera2D camera, NetworkModel network, SimulationSnapshot snapshot,
        IReadOnlyList<(double X, double Y)> obstaclePoints,
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
        foreach (var (ox, oy) in obstaclePoints)
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
    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10/D5 (P3.2/P3.3): `public` (not `private`) so a render
    // overlay in a CONSUMING assembly (e.g. the demo exe's EvacOverlay dashed boundary rect) can reuse
    // this generic drawing primitive instead of duplicating it.
    public static void DrawDashedPolyline(IReadOnlyList<(double X, double Y)> shape, float dashOn, float dashOff, float thick, Color color)
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
        // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.1/P3.2): `handle` is an OPTIONAL trailing param
        // (default `default(VehicleHandle)`, i.e. VehicleHandle.None) -- generic vehicle identity, for a
        // render-overlay (e.g. the evac layer's fear overpaint) to key its own per-vehicle state off the
        // SAME handle the engine uses, without the renderer knowing why. Every existing call site keeps
        // compiling. (The evac-specific `fear` param this struct briefly carried is gone -- fear tinting
        // is now the overlay's own DrawWorldOver overpaint, drawn on top of this plain speed-coloured rect.)
        public DrVehicleDraw(double frontX, double frontY, float headingDeg, float length, float width, double speedExact, VehicleHandle handle = default)
        {
            FrontX = frontX;
            FrontY = frontY;
            HeadingDeg = headingDeg;
            Length = length;
            Width = width;
            SpeedExact = speedExact;
            Handle = handle;
        }

        public double FrontX { get; }
        public double FrontY { get; }
        public float HeadingDeg { get; }
        public float Length { get; }
        public float Width { get; }
        public double SpeedExact { get; }
        public VehicleHandle Handle { get; }
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
            // docs/SUMOSHARP-PACKAGING-DESIGN.md D10 (P3.2): plain speed colour -- no domain tint here.
            // A domain overlay that wants to highlight a vehicle (e.g. the evac layer's fear overpaint)
            // redraws on top in its own DrawWorldOver, after this generic pass.
            var color = SpeedColor(v.SpeedExact);
            Raylib.DrawRectanglePro(rec, origin, rotationDeg, color);
        }
    }

    // Per-channel colour lerp (t clamped to [0,1]; alpha always opaque). `public` so a render overlay in
    // a CONSUMING assembly (e.g. the evac layer's fear overpaint) can blend toward its own tint colour
    // using this generic primitive instead of duplicating it.
    public static Color LerpColor(Color a, Color b, double t)
    {
        var clamped = Math.Clamp(t, 0.0, 1.0);
        return new Color(
            (int)Math.Round(a.R + (b.R - a.R) * clamped),
            (int)Math.Round(a.G + (b.G - a.G) * clamped),
            (int)Math.Round(a.B + (b.B - a.B) * clamped),
            255);
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
