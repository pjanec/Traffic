using System.Numerics;
using ImGuiNET;
using Raylib_cs;
using Sim.Core;
using Sim.Core.Orca;
using Sim.Evac;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Viewer.Core;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// P7-1: generalizes EvacOverlay's ped-drawing (PedFleeingColor/PedEscapedColor, DrawEvacPedestrians) into
// a standalone, domain-agnostic IRenderOverlay for ANY Sim.Pedestrians-driven population -- gated by
// REGIME (low-power PathArc/ActivityTimeline vs high-power FreeKinematic -- promoted, panicked, or
// otherwise reactive), not by any one domain's own vocabulary (EvacOverlay's "panicked"/"converted" stay
// evac-specific; this overlay knows nothing about incidents, cars, or safe zones). The drawing methods
// below consume ONLY a per-frame array of (X, Y, Regime, Visible) tuples -- PedRenderPoint -- published by
// whatever director/world drives the demo; a ped that boards a car / steps inside a building simply stops
// appearing in that array (appear/disappear), it is never a distinct "hidden" draw state.
//
// Like EvacOverlay, this class ALSO owns the demo-tool-side wiring for each concrete pedestrian-only demo
// (a `kind` string dispatch, mirroring EvacOverlay's own evacKind switch) -- Sim.Viewer.Core never
// references Sim.Pedestrians/Sim.Evac at all; this exe (the demo/sample layer) is the only place that
// does, exactly per docs/SUMOSHARP-PACKAGING-DESIGN.md D5/D10.
public enum PedRegime
{
    // PathArc / ActivityTimeline / Stationary -- deterministic, non-reactive, "cheap default" motion
    // (docs/PEDESTRIAN-DESIGN.md §4 "Low-power motion"). Ambient/calm walkers.
    LowPower,

    // FreeKinematic -- promoted into the persistent ORCA crowd, whether by interest-source proximity or a
    // forced pin (evac panic) -- reactive, neighbour-aware motion.
    HighPower,

    // A terminal/arrived ped a caller chose to keep emitting for one extra frame with a distinct colour
    // instead of just omitting it outright. Most callers (e.g. the district-evac demo below) never emit
    // this -- an escaped ped is simply no longer in the array the next frame (appear/disappear) -- but it
    // is here for a future demo that wants a one-frame "arrived" flash before the ped vanishes.
    Escaped,
}

// One pedestrian's render pose for THIS frame. `Visible` (default true) lets a caller keep a stable,
// id-indexed array shape across frames if it wants to; PedOverlay itself just skips entries with
// Visible == false, so "stop being drawn" is exactly as valid via Visible=false as via omitting the tuple.
public readonly record struct PedRenderPoint(double X, double Y, PedRegime Regime, bool Visible = true);

// Live HUD counters for the legend panel -- deliberately generic (no "panicked"/"converted" evac
// vocabulary): every pedestrian demo has a population and a low-power/high-power/escaped split.
public sealed class PedHudCounters
{
    public int Population;
    public int LowPower;
    public int HighPower;
    public int Escaped;
}

public sealed class PedOverlay : IPedDemoOverlay
{
    // Colours deliberately distinct from EvacOverlay's own evac-cascade palette (that overlay's cyan/green
    // mean "fleeing"/"escaped" IN THE CONTEXT of cars+incidents+pushers): here cyan/green mean the more
    // general LOD regime, and a third, muted slate marks low-power/ambient walkers so the two regimes read
    // as clearly different colours side by side (the required screenshot proof).
    private static readonly Color LowPowerColor = new(148, 163, 184, 255); // slate -- calm / ambient (PathArc, ActivityTimeline)
    private static readonly Color HighPowerColor = new(56, 189, 248, 255); // cyan -- promoted / panicked / reactive (FreeKinematic)
    private static readonly Color EscapedColor = new(52, 211, 153, 255); // green -- arrived / escaped (terminal)

    private readonly string _kind;
    private readonly string _repoRoot;

    // Published atomically on the engine thread (inside the OnAfterStep hook Build() returns), read on the
    // UI thread by the draw methods -- same immutable-snapshot discipline EvacRenderSnapshot uses, just a
    // flat point array instead of a class, since a ped population has no other per-frame state to publish.
    private volatile PedRenderPoint[] _snapshot = Array.Empty<PedRenderPoint>();
    private volatile PedHudCounters? _hud;

    public PedOverlay(string kind, string repoRoot)
    {
        _kind = kind;
        _repoRoot = repoRoot;
        NetPath = ResolveNetPath(kind, repoRoot);
    }

    // The kind's net path -- fed straight to EngineHost.CreateCustom(overlay.NetPath, overlay.Build) by
    // DemoSession.BuildHost, for geometry/bounds only (mirrors EvacOverlay.NetPath).
    public string NetPath { get; }

    // Exposed for a headless caller (e.g. a future smoke test) that wants to inspect the live snapshot
    // directly; the draw methods below read the same field.
    public PedRenderPoint[] Snapshot => _snapshot;

    // The buildFn passed to EngineHost.CreateCustom: builds a fresh Engine + per-step hook for the current
    // `_kind`. Unlike EvacOverlay's evac kinds (which also drive vehicle traffic through the SAME Engine
    // the director reads/writes), every pedestrian-only demo here has NO vehicles at all -- the Engine
    // exists solely so EngineHost/SimulationRunner has something to Step() and so the renderer has road
    // geometry to draw under the pedestrians; the population itself lives entirely in Sim.Pedestrians.
    public (Engine Engine, Action<Engine>? OnAfterStep) Build() => _kind switch
    {
        "evac-district" => BuildEvacDistrict(),
        _ => throw new ArgumentException($"Unknown pedestrian demo kind '{_kind}' (expected \"evac-district\")."),
    };

    private static string ResolveNetPath(string kind, string repoRoot) => kind switch
    {
        "evac-district" => Path.Combine(repoRoot, "scenarios", "_ped", "evac-district", "net.net.xml"),
        _ => throw new ArgumentException($"Unknown pedestrian demo kind '{kind}' (expected \"evac-district\")."),
    };

    // docs/PEDESTRIAN-TASKS.md P5-1(B): the routed foot-exodus over the REAL walkable evac-district net --
    // EvacDistrictDirector built exactly as EvacDistrictDemoTests.BuildDirector does (same net, same four
    // corner safe zones), driving the whole ambient population through the Sim.Pedestrians PedestrianWorld
    // facade. No vehicles at all -- an empty-demand Engine.LoadNetwork gives EngineHost/SimulationRunner a
    // real Engine to Step() (and the renderer real road geometry to draw), with zero spawns ever (this
    // custom-source EngineHost ctor forces _randomTraffic false permanently, and no rou.xml/spawner exists
    // here regardless).
    private (Engine, Action<Engine>?) BuildEvacDistrict()
    {
        var net = PedNetworkParser.Load(NetPath);
        var polygons = WalkablePolygonBaker.Bake(net);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        var safeZones = new Vec2[] { new(0, 0), new(200, 0), new(0, 200), new(200, 200) };
        // StartTime=3.0 (vs EvacDistrictDemoTests' 5.0): a quicker fuse so a short --frames screenshot run
        // still lands past the panic onset, showing both regimes rather than only the calm ambient one.
        var incident = new Incident(X: 100.0, Y: 100.0, StartTime: 3.0, Radius: 60.0);
        // Config.PanicRadius deliberately LOCALIZED to the incident's own radius (unlike
        // EvacDistrictDemoTests' PanicRadius=200, which intentionally covers the whole district so its
        // "everyone eventually escapes" test has a deterministic full-population panic). A district-wide
        // radius panics all 40 ambient peds on the SAME step once the incident goes active (every ped is
        // already within 200 units of the centre), which flips LowPower -> 0 almost instantly and leaves
        // nothing to demo -- a live viewer wants the natural, gradual mix a REALISTIC radius gives: peds
        // near the epicentre panic immediately, peds elsewhere stay calm (low-power) until their own
        // ambient route happens to carry them inside the radius, so both regimes stay visible together for
        // most of the run instead of only in a one-frame transition window.
        var director = new EvacDistrictDirector(nav, incident, safeZones, new EvacDistrictConfig { PanicRadius = 60.0 });

        var engine = new Engine();
        engine.LoadNetwork(NetPath); // geometry-only load: EmptyDemand, DefaultNetworkConfig (1.0s step)

        return (engine, e =>
        {
            // Engine's own DefaultNetworkConfig step-length is 1.0s (see Engine.LoadNetwork), matching the
            // dt EvacDistrictDemoTests itself steps by -- keeps the director's clock in lockstep with the
            // Engine's, exactly one Step() per director.Step() call, same discipline as EvacOverlay's own
            // BeforeStep/AfterStep split (this director has no such split -- Step(dt) is self-contained,
            // touches no Engine state, so a single call per tick is all that's needed).
            director.Step(1.0);
            _snapshot = CaptureEvacDistrict(director);
            _hud = new PedHudCounters
            {
                Population = director.PedestrianCount,
                HighPower = director.HighPowerCount,
                Escaped = director.EscapedCount,
                LowPower = director.PedestrianCount - director.HighPowerCount - director.EscapedCount,
            };
        });
    }

    // Captures EvacDistrictDirector's public read surface into the generic PedRenderPoint contract.
    // Regime comes from ModelOf (the facade's OWN LOD state), not from IsPanicked -- this is what keeps
    // the overlay itself domain-agnostic: any Sim.Pedestrians-driven population reports the same
    // PathArc/ActivityTimeline -> LowPower, FreeKinematic -> HighPower mapping regardless of WHY a given
    // ped was promoted. Escaped peds are Remove()d by the director itself (EvacDistrictDirector.Step), so
    // querying PositionOf/ModelOf for one would throw -- they are simply omitted from the array, which is
    // exactly the "stops being drawn" contract.
    //
    // Id iteration mirrors EvacDistrictDemoTests' own convention (`for (var id = 1; id <=
    // director.PedestrianCount; id++)`): ids are assigned 1..PedCount contiguously by
    // SpawnAmbientPopulation on a fully-connected net (EvacDistrictNetTests), so no id in that range is
    // ever missing from the population.
    private static PedRenderPoint[] CaptureEvacDistrict(EvacDistrictDirector director)
    {
        var points = new List<PedRenderPoint>(director.PedestrianCount);
        for (var id = 1; id <= director.PedestrianCount; id++)
        {
            if (director.IsEscaped(id))
            {
                continue; // arrived at its safe zone -- appear/disappear: just stop emitting it
            }

            var pos = director.PositionOf(id);
            var regime = director.ModelOf(id) == PedDrModel.FreeKinematic ? PedRegime.HighPower : PedRegime.LowPower;
            points.Add(new PedRenderPoint(pos.X, pos.Y, regime));
        }

        return points.ToArray();
    }

    // Pedestrian discs, drawn OVER the vehicles/roads (there are none here, but mirrors EvacOverlay's own
    // "peds drawn over" layering so a future demo that DOES mix cars + peds -- e.g. cars queued behind a
    // ped crossing -- reads the same way).
    public void DrawWorldOver(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles)
    {
        var points = _snapshot;
        if (points.Length == 0)
        {
            return;
        }

        global::Raylib_cs.Raylib.BeginMode2D(camera);

        const float pedRadius = 1.0f;
        foreach (var p in points)
        {
            if (!p.Visible)
            {
                continue;
            }

            var color = p.Regime switch
            {
                PedRegime.HighPower => HighPowerColor,
                PedRegime.Escaped => EscapedColor,
                _ => LowPowerColor,
            };
            global::Raylib_cs.Raylib.DrawCircleV(Renderer.Flip(p.X, p.Y), pedRadius, color);
        }

        global::Raylib_cs.Raylib.EndMode2D();
    }

    // The legend + live regime counters -- mirrors EvacOverlay.DrawUi's panel placement/sizing convention
    // (below the generic controls/diagnostics panels, left column) so it fits inside the default 1280x800
    // window alongside them.
    public void DrawUi()
    {
        var hud = _hud;

        ImGui.SetNextWindowPos(new Vector2(10, 610), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(360, 185), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - pedestrians");

        ImGui.Text("pedestrian legend:");
        LegendLine(LowPowerColor, "low-power (calm / ambient)");
        LegendLine(HighPowerColor, "high-power (promoted / panicked)");
        LegendLine(EscapedColor, "escaped / arrived");

        if (hud is { } h)
        {
            ImGui.Separator();
            // One line (not two) so the panel's fixed FirstUseEver height (matched to EvacOverlay's own
            // panel convention, which this sits directly below) never needs a scrollbar to show every
            // counter -- a scrolled-off counter would fail the screenshot verification just as surely as a
            // missing one.
            ImGui.Text($"pop: {h.Population}  low: {h.LowPower}  high: {h.HighPower}  escaped: {h.Escaped}");
        }

        ImGui.End();
    }

    private static void LegendLine(Color c, string label)
    {
        ImGui.TextColored(new Vector4(c.R / 255f, c.G / 255f, c.B / 255f, 1f), $"■ {label}");
    }
}
