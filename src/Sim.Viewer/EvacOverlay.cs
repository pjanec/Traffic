using System.Numerics;
using ImGuiNET;
using Raylib_cs;
using Sim.Core;
using Sim.Evac;
using Sim.Viewer.Core;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// docs/SUMOSHARP-PACKAGING-DESIGN.md D5/D10 (P3.2): the ONLY place in the demo tool that references
// Sim.Evac. Wires a live panic-evacuation scenario through EngineHost's GENERIC CreateCustom seam
// (Sim.Viewer.Core/EngineHost.cs) and renders it through the GENERIC IRenderOverlay seam (Commit 1) --
// Sim.Viewer.Core (the packageable viewer brain) never sees this type, or Sim.Evac, at all. This is the
// evac path that used to live inside EngineHost (CreateEvac/ResolveEvacProvider/CaptureEvac/
// SetIncidentAtWorld) and inside Renderer.cs (DrawEvacWorld/DrawEvacPedestrians/the evac legend); all of
// it now lives here, as one client of two generic seams.
public sealed class EvacOverlay : IRenderOverlay
{
    // docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §6: evac draw-pass colours, deliberately matching
    // Sim.Viz.SceneGen's KindFleeing/KindEscaped/KindAbandoned/KindPushingCar hex constants so the live
    // view and the offline HTML replay read identically. Moved here from Renderer.cs -- domain-specific,
    // not part of the generic renderer's palette.
    private static readonly Color BoundaryColor = new(120, 130, 140, 140); // muted grey
    private static readonly Color IncidentFillColor = new(245, 158, 11, 45); // translucent amber
    private static readonly Color IncidentRingColor = new(245, 158, 11, 210); // safe-radius ring
    private static readonly Color IncidentPendingColor = new(245, 158, 11, 100); // before StartTime
    private static readonly Color AbandonedColor = new(185, 28, 28, 255); // #b91c1c KindAbandoned
    private static readonly Color PedFleeingColor = new(56, 189, 248, 255); // #38bdf8 KindFleeing
    private static readonly Color PedEscapedColor = new(52, 211, 153, 255); // #34d399 KindEscaped
    private static readonly Color PusherColor = new(251, 146, 60, 255); // #fb923c KindPushingCar
    private static readonly Color FearAlarmColor = new(220, 38, 38, 255); // alarm red, fear=1 tint target

    private readonly string _evacKind;
    private readonly string _repoRoot;
    private readonly EvacConfig _config;
    private readonly double _defaultRadius;

    private Incident _incident;
    private EngineHost? _host;
    private volatile EvacRenderSnapshot? _snap;

    public EvacOverlay(string evacKind, string repoRoot)
    {
        _evacKind = evacKind;
        _repoRoot = repoRoot;

        var (netPath, config, defaultIncident) = Resolve(evacKind, repoRoot);
        NetPath = netPath;
        _config = config;
        _defaultRadius = defaultIncident.Radius;
        _incident = defaultIncident;
    }

    // The kind's net path -- fed straight to EngineHost.CreateCustom(overlay.NetPath, overlay.Build) by
    // DemoSession.BuildHost, for geometry/bounds only (EngineHost never inspects Build's own output).
    public string NetPath { get; }

    // The current live evac render snapshot, or null before the first engine step has captured one.
    // Exposed for a headless caller (e.g. a future smoke test) that wants to inspect it directly; the
    // draw methods below read the same field.
    public EvacRenderSnapshot? Snapshot => _snap;

    // Wired by DemoSession.BuildHost right after EngineHost.CreateCustom, so OnWorldClick's Rebuild() has
    // a host to call.
    public void Bind(EngineHost host) => _host = host;

    // The buildFn passed to EngineHost.CreateCustom: builds a fresh (engine, director) pair for the
    // current `_incident` via the matching Evac*Scenario.Build overload (the same mapping EngineHost's
    // old ResolveEvacProvider used), and returns the per-step hook that ticks the director and captures
    // the render snapshot -- EngineHost just wires this hook through without knowing what it does.
    public (Engine Engine, Action<Engine>? OnAfterStep) Build()
    {
        EvacDirector director;
        Engine engine;
        switch (_evacKind)
        {
            case "grid-tls":
                (engine, director, _) = EvacTlsScenario.Build(NetPath, _incident, _config);
                break;
            case "organic":
                (engine, director) = EvacOrganicScenario.Build(_repoRoot, _incident, _config);
                break;
            case "city":
                (engine, director) = EvacCityScenario.Build(_repoRoot, _incident, _config);
                break;
            default:
                throw new ArgumentException(
                    $"Unknown evac kind '{_evacKind}' (expected \"grid-tls\", \"organic\", or \"city\").");
        }

        // docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md: unlike EvacDirector.Tick() (the offline/self-driven
        // path, which owns _engine.Step() itself), the native viewer's SimulationRunner is the sole
        // engine driver here (Tick = Step; OnAfterStep) -- calling director.Tick() from OnAfterStep would
        // step the engine a SECOND time per runner tick, desyncing the director's clock from the traffic
        // (see the design doc's "double-step" note). Instead split BeforeStep/AfterStep across the
        // runner's own step: BeforeStep() now (before the runner's first Step()), then per step
        // AfterStep() -> Capture() -> BeforeStep() (prep for the NEXT Step()). Unrolled across ticks this
        // is exactly BeforeStep_N, Step_N, AfterStep_N per N -- the engine steps once per runner tick and
        // director.Time lines up with the just-completed engine step, byte-identical in spirit to
        // EvacDirector.Tick()'s own PreStep, Step, PostStep interleaving.
        director.BeforeStep(); // initial pre-step prep for the runner's FIRST Engine.Step()
        return (engine, e => { director.AfterStep(); _snap = Capture(e, director); director.BeforeStep(); });
    }

    // Maps an evac kind name to its net path (geometry/bounds), the config instance every Build call for
    // that kind shares (so Capture can report SafeRadius without EvacDirector needing to expose its own
    // config), and the kind's default incident (for the radius OnWorldClick reuses).
    private static (string NetPath, EvacConfig Config, Incident DefaultIncident) Resolve(string evacKind, string repoRoot) =>
        evacKind switch
        {
            "grid-tls" => (
                Path.Combine(repoRoot, "scenarios", "evac-grid-tls", "net.net.xml"),
                EvacTlsScenario.DefaultConfig(),
                EvacTlsScenario.DefaultIncident),
            "organic" => (
                Path.Combine(repoRoot, "scenarios", "_bench", "city-organic-L2", "net.net.xml"),
                EvacOrganicScenario.DefaultConfig(),
                EvacOrganicScenario.DefaultIncident),
            "city" => (
                Path.Combine(repoRoot, "scenarios", "_bench", "city-15000", "net.net.xml"),
                EvacCityScenario.DefaultConfig(),
                EvacCityScenario.DefaultIncident),
            _ => throw new ArgumentException(
                $"Unknown evac kind '{evacKind}' (expected \"grid-tls\", \"organic\", or \"city\").", nameof(evacKind)),
        };

    // Capture EvacDirector's public read surface into an immutable EvacRenderSnapshot. Called from
    // inside the engine-thread OnAfterStep hook (Build's closure), right after `director.Tick()`, so the
    // director is quiescent for the duration of this call -- moved verbatim from EngineHost's old
    // CaptureEvac, reading `_config.SafeRadius` instead of a separately-captured field.
    private EvacRenderSnapshot Capture(Engine engine, EvacDirector director)
    {
        var pedCount = director.PedestrianCount;
        var peds = new (double X, double Y, bool Escaped)[pedCount];
        for (var i = 0; i < pedCount; i++)
        {
            var p = director.PedestrianPosition(i);
            peds[i] = (p.X, p.Y, director.PedestrianEscaped(i));
        }

        var carCount = director.AbandonedCarCount;
        var cars = new (double X, double Y, double R)[carCount];
        for (var i = 0; i < carCount; i++)
        {
            var c = director.AbandonedCar(i);
            cars[i] = (c.X, c.Y, c.Radius);
        }

        var pushers = director.ActivePushers()
            .Select(p => (p.X, p.Y, p.HeadingRad))
            .ToArray();

        var fear = new Dictionary<uint, double>();
        foreach (var handle in engine.VehicleHandles)
        {
            fear[handle.Index] = director.Fear(handle);
        }

        var incident = director.Incident;
        var navmesh = director.NavMesh;

        return new EvacRenderSnapshot
        {
            Time = director.Time,
            Peds = peds,
            AbandonedCars = cars,
            Pushers = pushers,
            FearByVehicle = fear,
            Incident = (incident.X, incident.Y, incident.Radius, incident.StartTime, _config.SafeRadius),
            Boundary = (navmesh.MinX, navmesh.MinY, navmesh.MaxX, navmesh.MaxY),
            Panicked = director.PanickedCount,
            Converted = director.ConvertedCount,
            Escaped = director.EscapedCount,
            Abandoned = director.AbandonedCarCount,
        };
    }

    // docs/SUMOSHARP-PACKAGING-DESIGN.md D10: this overlay wants world clicks (re-placing the incident)
    // instead of the generic viewer's default "drop an obstacle" behaviour.
    public bool HandlesWorldClick => true;

    // (Re)place the incident at the clicked world point and rebuild through the SAME EngineHost, via the
    // generic Rebuild() seam -- unlike a user "restart", this PRESERVES pause. Radius is the kind's own
    // DefaultIncident.Radius; StartTime is "now" in the CURRENT run's sim-clock, so a click always plants
    // a live-from-now incident. Moved from EngineHost's old SetIncidentAtWorld.
    public void OnWorldClick(double worldX, double worldY)
    {
        var currentTime = _host?.Snapshot?.Time ?? 0.0;
        _incident = new Incident(worldX, worldY, currentTime, _defaultRadius);
        _host?.Rebuild();
    }

    // docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §6: boundary, incident zone, abandoned cars and pushers --
    // drawn UNDER the vehicles (moved verbatim from Renderer.cs's old DrawEvacWorld).
    public void DrawWorldUnder(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles)
    {
        var snap = _snap;
        if (snap is null)
        {
            return;
        }

        global::Raylib_cs.Raylib.BeginMode2D(camera);

        // 1. Known-world boundary: a dashed rectangle through the hard NavMesh edge.
        var (minX, minY, maxX, maxY) = snap.Boundary;
        var boundaryShape = new (double X, double Y)[]
        {
            (minX, minY), (maxX, minY), (maxX, maxY), (minX, maxY), (minX, minY),
        };
        var dashOn = 6f / camera.Zoom;
        var dashOff = 5f / camera.Zoom;
        var dashThick = Math.Max(1.5f / camera.Zoom, 0.1f);
        Renderer.DrawDashedPolyline(boundaryShape, dashOn, dashOff, dashThick, BoundaryColor);

        // 2. Incident zone: filled translucent amber disc once live (Time >= StartTime) plus a thin ring
        // at the safe radius pedestrians must clear to count as Escaped; before StartTime, just a faint
        // outline at the incident radius so the user can see where it will fire.
        var (ix, iy, radius, startTime, safeRadius) = snap.Incident;
        var center = Renderer.Flip(ix, iy);
        if (snap.Time >= startTime)
        {
            global::Raylib_cs.Raylib.DrawCircleV(center, (float)radius, IncidentFillColor);
            global::Raylib_cs.Raylib.DrawCircleLinesV(center, (float)safeRadius, IncidentRingColor);
        }
        else
        {
            global::Raylib_cs.Raylib.DrawCircleLinesV(center, (float)radius, IncidentPendingColor);
        }

        // 3. Abandoned cars: filled dark-red discs (KindAbandoned). Under vehicles -- a car that has been
        // abandoned sits still on the road, so a moving vehicle drawn on top of it reads correctly.
        foreach (var (x, y, r) in snap.AbandonedCars)
        {
            global::Raylib_cs.Raylib.DrawCircleV(Renderer.Flip(x, y), (float)r, AbandonedColor);
        }

        // 4. Pushers: oriented ~5.0x1.8 m rectangles (KindPushingCar), rotated by HeadingRad. HeadingRad is
        // "math radians, 0 = +x, CCW" (Sim.Viz.SceneGen's own comment on ActivePushersWithHandle) -- rotate
        // the LOCAL rectangle corners in WORLD space by that heading, then Flip each corner to screen space,
        // exactly mirroring SceneGen/template.js's drawShaped (world-space rotate -> worldToScreen) so the
        // live orientation matches the offline HTML replay.
        foreach (var (x, y, headingRad) in snap.Pushers)
        {
            DrawOrientedRectWorld(x, y, headingRad, halfLen: 2.5f, halfWid: 0.9f, PusherColor);
        }

        global::Raylib_cs.Raylib.EndMode2D();
    }

    // Pedestrians (cyan fleeing -> green escaped) AND the fear overpaint (moved from Renderer.cs's old
    // DrawEvacPedestrians + the fear-tint branch that used to live inline in DrawVehicleList) -- drawn
    // OVER the vehicles.
    public void DrawWorldOver(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles)
    {
        var snap = _snap;
        if (snap is null)
        {
            return;
        }

        global::Raylib_cs.Raylib.BeginMode2D(camera);

        const float pedRadius = 1.0f;
        foreach (var (x, y, escaped) in snap.Peds)
        {
            global::Raylib_cs.Raylib.DrawCircleV(Renderer.Flip(x, y), pedRadius, escaped ? PedEscapedColor : PedFleeingColor);
        }

        // Fear overpaint: for every vehicle DrawDynamicWorld already drew (plain speed colour), redraw
        // the SAME oriented rectangle in a fear-red blended colour on top, for whichever ones the evac
        // layer has recorded a nonzero fear for -- keyed by the generic DrVehicleDraw.Handle (Commit 1),
        // so this overlay never needs its own vehicle-identity plumbing.
        for (var i = 0; i < vehicles.Count; i++)
        {
            var v = vehicles[i];
            if (!snap.FearByVehicle.TryGetValue(v.Handle.Index, out var fear) || fear <= 0.0)
            {
                continue;
            }

            var front = Renderer.Flip(v.FrontX, v.FrontY);
            var length = Math.Max(0.5f / camera.Zoom, v.Length);
            var width = Math.Max(0.3f / camera.Zoom, v.Width);

            var nr = v.HeadingDeg * MathF.PI / 180f;
            var sa = MathF.Atan2(-MathF.Cos(nr), MathF.Sin(nr));
            var rotationDeg = sa * 180f / MathF.PI;

            var rec = new Rectangle(front.X, front.Y, length, width);
            var origin = new Vector2(length, width / 2f);
            var baseColor = Renderer.SpeedColor(v.SpeedExact);
            var color = Renderer.LerpColor(baseColor, FearAlarmColor, fear);
            global::Raylib_cs.Raylib.DrawRectanglePro(rec, origin, rotationDeg, color);
        }

        global::Raylib_cs.Raylib.EndMode2D();
    }

    // Rotates a `halfLen`x`halfWid` rectangle centred at world (x,y) by a WORLD-space heading (math
    // radians, 0=+x, CCW) and fills it as two triangles in screen space -- moved verbatim from
    // Renderer.cs's old DrawOrientedRectWorld (mirrors SceneGen/drawShaped's world-rotate-then-flip
    // instead of trying to derive an equivalent DrawRectanglePro screen-rotation angle, since a Y-flip is
    // a reflection, not a pure rotation).
    private static void DrawOrientedRectWorld(double x, double y, double headingRad, float halfLen, float halfWid, Color color)
    {
        var hx = (float)Math.Cos(headingRad);
        var hy = (float)Math.Sin(headingRad);

        Span<float> along = stackalloc float[] { halfLen, halfLen, -halfLen, -halfLen };
        Span<float> perp = stackalloc float[] { halfWid, -halfWid, -halfWid, halfWid };
        Span<Vector2> corners = stackalloc Vector2[4];
        for (var i = 0; i < 4; i++)
        {
            var wx = x + along[i] * hx + perp[i] * -hy;
            var wy = y + along[i] * hy + perp[i] * hx;
            corners[i] = Renderer.Flip(wx, wy);
        }

        global::Raylib_cs.Raylib.DrawTriangle(corners[0], corners[1], corners[2], color);
        global::Raylib_cs.Raylib.DrawTriangle(corners[0], corners[2], corners[3], color);
    }

    // The evac legend + live counters + incident-placement hint -- moved from Renderer.cs's old
    // DrawDemosPanel evac section (+ its LegendLine helper), now a standalone panel instead of tacked
    // onto the generic demos picker.
    public void DrawUi()
    {
        var snap = _snap;

        // Below the generic controls (10,10;360,390) + diagnostics (10,410;360,200) panels, on the same
        // left column, so it fits inside the default 1280x800 window instead of spilling past the bottom
        // edge (the demos panel already owns the right column at x=900).
        ImGui.SetNextWindowPos(new Vector2(10, 610), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(360, 185), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - evac");

        ImGui.Text("evac legend:");
        LegendLine(IncidentRingColor, "incident");
        LegendLine(PedFleeingColor, "fleeing");
        LegendLine(PedEscapedColor, "escaped");
        LegendLine(AbandonedColor, "abandoned");
        LegendLine(PusherColor, "shoulder-push");

        if (snap is { } ev)
        {
            ImGui.Separator();
            ImGui.Text($"panicked: {ev.Panicked}   converted: {ev.Converted}");
            ImGui.Text($"escaped: {ev.Escaped}   abandoned: {ev.Abandoned}");
        }

        ImGui.TextWrapped("click a road to (re)place the incident");
        ImGui.End();
    }

    // One legend row: a coloured swatch glyph + label.
    private static void LegendLine(Color c, string label)
    {
        ImGui.TextColored(new Vector4(c.R / 255f, c.G / 255f, c.B / 255f, 1f), $"■ {label}");
    }
}
