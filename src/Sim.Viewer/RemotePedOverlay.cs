using System.Numerics;
using ImGuiNET;
using Raylib_cs;
using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Replication;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// P7-2 (docs/PEDESTRIAN-TASKS.md): the native-viewer analog of the HTML "Remote (over the wire)" scene
// (Sim.Viz/SceneGen.cs BuildPedRemote). Same server ped-sim + gated multicast wire + reconstructor loop --
// only the rendering surface differs (Raylib instead of HTML). The crowd this overlay draws is
// reconstructed PURELY from the DR-error-gated replication stream (PedReplicationPublisher ->
// InMemoryPedReplicationBus -> PedRemoteReconstructor), NOT read from PedLodManager.PositionOf/ModelOf --
// that is the whole point: it proves server == IG within render tolerance, with no promotion "pop".
//
// The parity is made VISIBLE on screen: the reconstructed (IG) poses are drawn as FILLED regime-coloured
// discs (the primary crowd), and the server's own ground-truth positions are drawn as THIN OUTLINE RINGS
// at the same radius. When the filled IG disc sits centred inside its server ring, that coincidence IS the
// parity -- a viewer sees server and IG agree frame-by-frame, including across a promotion's DR-model
// switch. The on-screen max |server - IG| distance (metres) and the achieved wire Mbit/s quantify it.
//
// Structurally this mirrors PedOverlay: a Build() returning (Engine, OnAfterStep) where the Engine is a
// geometry-only clock (zero vehicles) and the ped-sim + wire + reconstructor advance one 0.2s tick inside
// the hook, publishing volatile snapshots read on the UI thread by the draw methods.

// The demo-tool seam DemoSession.BuildHost drives every pedestrian-only demo through, so the two concrete
// pedestrian overlays (PedOverlay, RemotePedOverlay) can be built uniformly: NetPath for the geometry-only
// EngineHost.CreateCustom bounds, Build() for the fresh Engine + per-step hook. Extends the generic
// IRenderOverlay (Sim.Viewer.Raylib) with just the two members the demo-tool wiring needs.
internal interface IPedDemoOverlay : IRenderOverlay
{
    string NetPath { get; }
    (Engine Engine, Action<Engine>? OnAfterStep) Build();
}

public sealed class RemotePedOverlay : IPedDemoOverlay
{
    // Regime colours match PedOverlay's exactly (slate = low-power / ambient PathArc, cyan = high-power /
    // promoted FreeKinematic) so the two pedestrian demos read the same way side by side. Redeclared rather
    // than referenced because PedOverlay's are private -- the values are the contract, not the field.
    private static readonly Color LowPowerColor = new(148, 163, 184, 255);  // slate -- low-power PathArc
    private static readonly Color HighPowerColor = new(56, 189, 248, 255);  // cyan  -- high-power FreeKinematic
    // Neutral, near-white ring for the server ground-truth -- deliberately NOT a regime colour, so a viewer
    // reads the ring as "where the server actually is" independent of which regime the IG disc is showing.
    private static readonly Color ServerRingColor = new(226, 232, 240, 220); // light slate-white
    private static readonly Color SourceColor = new(250, 204, 21, 255);       // amber -- the swept interest source

    // --- ped-sim / wire constants, copied verbatim from SceneGen.BuildPedRemote (its remarks explain the
    // geometry) so this native demo reconstructs the exact same crowd the HTML scene proves. ---
    private const double Cx = 120.0, Cy = 120.0;
    private const double MaxSpeed = 1.3;
    private const double PedRadius = 0.3;
    private const double ArriveRadius = 0.3;
    private const double DwellSeconds = 1.0;
    private const double PromoteRadius = 6.0;
    private const double DemoteRadius = 13.0;
    private const double Dt = 0.2;                 // ped-sim tick -- DECOUPLED from the Engine's 1.0s step
    private const int MaxPeds = 14;
    private const int SpawnEveryNSteps = 20;       // a fresh ped roughly every 4s of ped-sim time
    private const double SweepRadius = 16.0;
    private const double SweepPeriod = 70.0;       // seconds per interest-source revolution

    // On-screen render radius (metres) for both the filled IG disc and its server ring. Larger than the
    // 0.3 m physical PedRadius purely for legibility at plaza scale (mirrors PedOverlay drawing at 1.0 m);
    // both discs and rings use it, so a coincident server/IG pair reads as a ring hugging its filled disc.
    private const float PedRenderRadius = 1.2f;

    private readonly string _repoRoot;

    // Published atomically on the engine thread (inside the OnAfterStep hook), read on the UI thread by the
    // draw methods -- same immutable-snapshot discipline PedOverlay uses.
    private volatile RemotePedPose[] _snapshot = Array.Empty<RemotePedPose>();
    private volatile RemoteHud? _hud;
    private Vec2 _sourcePos;

    public RemotePedOverlay(string repoRoot)
    {
        _repoRoot = repoRoot;
        NetPath = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "net.net.xml");
    }

    // Fed straight to EngineHost.CreateCustom(overlay.NetPath, overlay.Build) by DemoSession.BuildHost, for
    // geometry/bounds only (same as PedOverlay.NetPath).
    public string NetPath { get; }

    // The running MAX across the WHOLE run of the per-frame max |server - IG-render| distance (metres) --
    // exposed public so a headless verifier can read the numeric parity result directly off the overlay.
    public double MaxRenderErrorMeters { get; private set; }

    // Exposed for a headless caller that wants the live snapshot directly; the draw methods read the same.
    public RemotePedPose[] Snapshot => _snapshot;

    // One pedestrian's paired render state for a frame: the reconstructed (IG) pose + regime, and the
    // server ground-truth pose. Either may be absent (a ped not yet observed on the wire has no IG pose;
    // every active ped always has a server pose here since none is ever truly despawned in this demo).
    public readonly record struct RemotePedPose(
        int Id, double IgX, double IgY, PedRegime Regime, bool HasIg, double SrvX, double SrvY, bool HasSrv);

    // Live HUD counters for the legend panel.
    private sealed class RemoteHud
    {
        public int Population;
        public int HighPower;
        public double FrameMaxErr;
        public double RunMaxErr;
        public double MbitPerSecond;
        public double PlayoutDelay;
    }

    // Builds the geometry-only Engine (zero vehicles, exactly like PedOverlay) plus the server ped-sim +
    // wire + reconstructor exactly as SceneGen.BuildPedRemote does, and returns the per-step hook that
    // advances the ped sim ONE 0.2s tick per Engine step and republishes the volatile snapshots.
    public (Engine Engine, Action<Engine>? OnAfterStep) Build()
    {
        var walkableAddPath = Path.Combine(_repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "walkable.add.xml");
        var pedNetwork = PedNetworkParser.Load(NetPath, walkableAddPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // The four arm-crossing candidates, same geometry as BuildPedRemote (see its remarks).
        var candidates = new (Vec2 A, Vec2 B)[]
        {
            (new Vec2(Cx - 7.4, Cy + 20.0), new Vec2(Cx + 7.4, Cy + 20.0)), // north arm
            (new Vec2(Cx + 20.0, Cy - 7.4), new Vec2(Cx + 20.0, Cy + 7.4)), // east arm
            (new Vec2(Cx - 7.4, Cy - 20.0), new Vec2(Cx + 7.4, Cy - 20.0)), // south arm
            (new Vec2(Cx - 20.0, Cy - 7.4), new Vec2(Cx - 20.0, Cy + 7.4)), // west arm
        };

        var validPairs = new List<(Vec2[] Forward, Vec2[] Backward)>();
        foreach (var (a, b) in candidates)
        {
            var path = nav.FindPath(a, b);
            if (path is { Count: >= 2 })
            {
                var fwd = path.ToArray();
                validPairs.Add((fwd, fwd.Reverse().ToArray()));
            }
        }

        if (validPairs.Count == 0)
        {
            throw new InvalidOperationException("RemotePedOverlay: no valid arm-crossing routes found on the net.");
        }

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var field = new InterestField();
        _sourcePos = new Vec2(Cx - 25.0, Cy);
        var source = new InterestSource(_sourcePos, PromoteRadius, DemoteRadius);
        var sourceId = field.Register(source, InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        // The wire: gated publisher (DR-error scheduler + global bandwidth governor) -> InMemory byte
        // loopback -> reconstructor. The crowd this overlay draws is reconstructed from THIS stream.
        var meter = new PedBandwidthMeter();
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);
        var bus = new InMemoryPedReplicationBus();
        var wirePublisher = new PedReplicationPublisher(bus.Sink, scheduler, governor, meter, stepDt: Dt);
        var reconstructor = new PedRemoteReconstructor(bus.Source);

        var pedInfo = new Dictionary<int, (Vec2[] Forward, Vec2[] Backward, bool GoingForward)>();
        var activeIds = new List<int>();
        var nextId = 1;
        var pairCursor = 0;
        var now = 0.0;
        var step = 0;

        void Spawn(double t)
        {
            var (fwd, bwd) = validPairs[pairCursor % validPairs.Count];
            var goingForward = (pairCursor / validPairs.Count) % 2 == 0;
            pairCursor++;

            var id = nextId++;
            var path = goingForward ? fwd : bwd;
            manager.AddPed(id, path, MaxSpeed, PedRadius, t);
            pedInfo[id] = (fwd, bwd, goingForward);
            activeIds.Add(id);
        }

        var engine = new Engine();
        engine.LoadNetwork(NetPath); // geometry-only load: EmptyDemand, DefaultNetworkConfig (1.0s step)

        return (engine, _engineClock =>
        {
            // Whole-step wire batch: capture the event-list cursor BEFORE this step's spawn/promotion so a
            // freshly-spawned ped's PathArcRecord rides the SAME wire batch as everything Step emits this
            // tick (the whole-step publish granularity the reconstructor relies on -- see BuildPedRemote).
            var beforeCount = publisher.Events.Count;

            if (step % SpawnEveryNSteps == 0 && activeIds.Count < MaxPeds)
            {
                Spawn(now);
            }

            // Sweep the interest source in a slow circle so it passes near every arm's route in turn.
            var angle = (now / SweepPeriod) * 2.0 * Math.PI;
            _sourcePos = new Vec2(Cx + (SweepRadius * Math.Cos(angle)), Cy + (SweepRadius * Math.Sin(angle)));
            field.Move(sourceId, _sourcePos);

            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            // Cycle each ped back and forth once it reaches its leg's end -- a sustained flow. ONLY recycles
            // a currently LOW-power ped: RemovePed/AddPed here is a demand-side "reached destination, walk
            // back" reset, not a promote/demote transition, so it publishes no DR-switch; cycling a still-
            // FreeKinematic ped this way would desync the wire (the reconstructor would keep believing it is
            // high-power with no more samples coming). A high-power ped instead demotes via Step's own
            // dwell mechanism (which DOES publish the DR-switch + fresh PathArcRecord) and cycles later.
            foreach (var id in activeIds)
            {
                if (manager.ModelOf(id) != PedDrModel.PathArc)
                {
                    continue;
                }

                var (fwd, bwd, goingForward) = pedInfo[id];
                var dest = goingForward ? fwd[^1] : bwd[^1];
                if ((manager.PositionOf(id, now) - dest).Abs < 0.75)
                {
                    manager.RemovePed(id);
                    var nextPath = goingForward ? bwd : fwd;
                    manager.AddPed(id, nextPath, MaxSpeed, PedRadius, now);
                    pedInfo[id] = (fwd, bwd, !goingForward);
                }
            }

            var newEvents = new List<PedEvent>(publisher.Events.Count - beforeCount);
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            wirePublisher.Publish(newEvents);
            reconstructor.Pump(now);

            // Build the paired snapshot: reconstructed (IG) pose from the wire alone + server ground truth.
            var poses = new List<RemotePedPose>(activeIds.Count);
            var highPower = 0;
            var frameMaxErr = 0.0;
            foreach (var id in activeIds)
            {
                // Server ground truth: PositionOf is safe for every active id -- no ped is ever truly
                // despawned in this demo (Spawn only adds; cycling RemovePed+AddPed keeps the SAME id), so
                // the manager always knows it.
                var srv = manager.PositionOf(id, now);

                var hasIg = reconstructor.TryGetRenderPose(id, out var igPos, out var visible, out _) && visible;
                var regime = PedRegime.LowPower;
                if (hasIg)
                {
                    var high = reconstructor.Ig.ModelOf(id) == PedDrModel.FreeKinematic;
                    regime = high ? PedRegime.HighPower : PedRegime.LowPower;
                    if (high)
                    {
                        highPower++;
                    }

                    var err = (srv - igPos).Abs;
                    if (err > frameMaxErr)
                    {
                        frameMaxErr = err;
                    }
                }

                poses.Add(new RemotePedPose(
                    id, hasIg ? igPos.X : 0.0, hasIg ? igPos.Y : 0.0, regime, hasIg, srv.X, srv.Y, HasSrv: true));
            }

            if (frameMaxErr > MaxRenderErrorMeters)
            {
                MaxRenderErrorMeters = frameMaxErr;
            }

            // Whole-run average over the single multicast stream (a short rolling window reads 0 in the
            // quiet spells the DR gate creates, so the cumulative average is the honest steady figure). A
            // 14-ped DR-error-gated stream is genuinely sub-kilobit -- that IS the §7 bandwidth headline --
            // so it rounds to 0.000 Mbit/s; the Kbit/s companion keeps the on-screen number legible.
            var mbit = meter.MbitPerSecond(Math.Max(now, Dt));
            var kbit = mbit * 1000.0;

            _snapshot = poses.ToArray();
            _hud = new RemoteHud
            {
                Population = reconstructor.KnownIds.Count,
                HighPower = highPower,
                FrameMaxErr = frameMaxErr,
                RunMaxErr = MaxRenderErrorMeters,
                MbitPerSecond = mbit,
                PlayoutDelay = PedRemoteReconstructor.DefaultPlayoutDelaySeconds,
            };

            // The numeric parity gate: one line per tick, read off stderr by the verifier.
            Console.Error.WriteLine(
                $"[remote] t={now:F1} pop={reconstructor.KnownIds.Count} high={highPower} "
                + $"maxErr={frameMaxErr:F3} runMax={MaxRenderErrorMeters:F3} mbit={mbit:F3} kbit={kbit:F2}");

            step++;
        });
    }

    // Reconstructed IG discs (filled, regime-coloured) with the server ground-truth as thin outline rings
    // at the same radius: a filled disc centred inside its ring is the parity, visible frame-by-frame.
    public void DrawWorldOver(Camera2D camera, SimulationSnapshot snapshot, IReadOnlyList<Renderer.DrVehicleDraw> vehicles)
    {
        var poses = _snapshot;

        global::Raylib_cs.Raylib.BeginMode2D(camera);

        // Server rings first (drawn under the filled discs so the disc reads as sitting "inside" the ring).
        foreach (var p in poses)
        {
            if (!p.HasSrv)
            {
                continue;
            }

            global::Raylib_cs.Raylib.DrawCircleLinesV(Renderer.Flip(p.SrvX, p.SrvY), PedRenderRadius, ServerRingColor);
        }

        // Reconstructed (IG) filled discs on top.
        foreach (var p in poses)
        {
            if (!p.HasIg)
            {
                continue;
            }

            var color = p.Regime == PedRegime.HighPower ? HighPowerColor : LowPowerColor;
            global::Raylib_cs.Raylib.DrawCircleV(Renderer.Flip(p.IgX, p.IgY), PedRenderRadius, color);
        }

        // The swept interest source -- a small distinct amber marker (filled dot + a wider promote-radius
        // ring) so a viewer can tie a promotion to the source being near a walker.
        var srcCenter = Renderer.Flip(_sourcePos.X, _sourcePos.Y);
        global::Raylib_cs.Raylib.DrawCircleV(srcCenter, 1.5f, SourceColor);
        global::Raylib_cs.Raylib.DrawCircleLinesV(srcCenter, (float)PromoteRadius, SourceColor);

        global::Raylib_cs.Raylib.EndMode2D();
    }

    // Legend + live HUD -- one line per metric, within a fixed panel height (a scrolled-off counter fails
    // verification), mirroring PedOverlay's panel placement.
    public void DrawUi()
    {
        var hud = _hud;

        ImGui.SetNextWindowPos(new Vector2(10, 606), ImGuiCond.FirstUseEver);
        ImGui.SetNextWindowSize(new Vector2(390, 190), ImGuiCond.FirstUseEver);
        ImGui.Begin("SumoSharp - pedestrians (remote)");

        // No standalone header line -- the panel title already says "remote", and the 800 px window has
        // room for exactly the legend + counters without one (a scrolled-off metric fails verification).
        LegendLine(LowPowerColor, "IG low-power (PathArc)");
        LegendLine(HighPowerColor, "IG high-power (FreeKinematic)");
        LegendLine(ServerRingColor, "server ground-truth ring");
        LegendLine(SourceColor, "interest source (promotes nearby)");

        if (hud is { } h)
        {
            ImGui.Separator();
            ImGui.Text($"pop: {h.Population}   high-power: {h.HighPower}");
            ImGui.Text($"max |server - IG|: {h.FrameMaxErr:F3} m  (run max {h.RunMaxErr:F3} m)");
            ImGui.Text($"wire: {h.MbitPerSecond:F3} Mbit/s ({h.MbitPerSecond * 1000.0:F2} Kbit/s)  playout: {h.PlayoutDelay:F2}s");
        }

        ImGui.End();
    }

    private static void LegendLine(Color c, string label)
    {
        ImGui.TextColored(new Vector4(c.R / 255f, c.G / 255f, c.B / 255f, 1f), $"■ {label}");
    }
}
