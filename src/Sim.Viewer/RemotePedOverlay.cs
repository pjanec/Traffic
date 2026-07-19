using System.Numerics;
using CycloneDDS.Runtime;
using ImGuiNET;
using Raylib_cs;
using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Replication;
using Sim.Replication.Dds;
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

// D3 (docs/PEDESTRIAN-DDS-TRANSPORT-TASKS.md): which transport carries this overlay's ped wire. InMemory
// is the hermetic in-process byte loopback (the P7-2 default); Dds is the live CycloneDDS binding
// (DdsPedReplicationSink/Source), lighting up the real "remote (DDS)" path in the native viewer. The
// IPedReplicationSink/Source pair is identical either way, so the reconstruction + render are unchanged.
public enum PedWireTransport
{
    // In-process server + wire (both halves in ONE process): the server sim + the reconstructor, so the
    // overlay can draw server ground-truth rings coincident with the reconstructed IG discs.
    InMemory, // hermetic byte loopback (P7-2 default)
    Dds,      // live CycloneDDS, one participant (D3a)

    // Pure remote IG (D3b): NO server sim in this process -- subscribe to an EXTERNAL `--mode ped-publish`
    // process's live DDS ped stream and render the reconstructed crowd alone (no server rings). This is the
    // genuine two-process, cross-process multicast topology of Requirement 7.
    DdsSubscribeOnly,
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

    // The ped-sim / scenario constants now live on RemotePedServer (shared with the `--mode ped-publish`
    // process); this overlay references RemotePedServer.Dt / .PromoteRadius where it needs them.

    // On-screen render radius (metres) for both the filled IG disc and its server ring. Larger than the
    // 0.3 m physical PedRadius purely for legibility at plaza scale (mirrors PedOverlay drawing at 1.0 m);
    // both discs and rings use it, so a coincident server/IG pair reads as a ring hugging its filled disc.
    private const float PedRenderRadius = 1.2f;

    private readonly string _repoRoot;
    private readonly PedWireTransport _transport;

    // Held for the overlay's lifetime when _transport == Dds (IRenderOverlay has no Dispose seam, and the
    // participant must outlive the render loop). Null for the in-process InMemory transport.
    private DdsParticipant? _participant;

    // Published atomically on the engine thread (inside the OnAfterStep hook), read on the UI thread by the
    // draw methods -- same immutable-snapshot discipline PedOverlay uses.
    private volatile RemotePedPose[] _snapshot = Array.Empty<RemotePedPose>();
    private volatile RemoteHud? _hud;
    private Vec2 _sourcePos;
    // Only the server role knows the swept interest-source position; subscribe-only IGs never draw it.
    private volatile bool _drawSource;

    public RemotePedOverlay(string repoRoot, PedWireTransport transport = PedWireTransport.InMemory)
    {
        _repoRoot = repoRoot;
        _transport = transport;
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

    // Builds the geometry-only Engine (zero vehicles, exactly like PedOverlay) plus the wire + reconstructor,
    // and returns the per-step hook that advances one 0.2s tick and republishes the volatile snapshots.
    //
    // Two roles, by transport:
    //  * InMemory / Dds -- this process RUNS the server sim (RemotePedServer) AND the reconstructor, so it
    //    draws the reconstructed IG discs coincident with the server ground-truth rings (the P7-2/D3a proof).
    //  * DdsSubscribeOnly (D3b) -- NO server sim here: subscribe to an external `--mode ped-publish` process
    //    over live DDS and render the reconstructed crowd alone (no rings; a real remote IG has only the wire).
    public (Engine Engine, Action<Engine>? OnAfterStep) Build()
    {
        var subscribeOnly = _transport == PedWireTransport.DdsSubscribeOnly;

        var meter = new PedBandwidthMeter();
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);

        // The server sim + its publisher exist only when this process is the server (not subscribe-only).
        RemotePedServer? server = null;
        PedReplicationPublisher? wirePublisher = null;
        IPedReplicationSource wireSource;

        if (subscribeOnly)
        {
            // Pure remote IG: subscribe to the external publisher's live DDS ped stream; no local sim.
            _participant = new DdsParticipant();
            wireSource = new DdsPedReplicationSource(_participant);
            Thread.Sleep(500); // async discovery settle before the first pump
        }
        else
        {
            server = new RemotePedServer(_repoRoot);
            _sourcePos = server.SourcePos;
            _drawSource = true;

            IPedReplicationSink sink;
            if (_transport == PedWireTransport.Dds)
            {
                // Live CycloneDDS: publisher + subscriber on ONE participant (D3a), the D2 loopback proof's
                // path but driving a render. Kept in a field so the participant is not GC-collected mid-run;
                // discovery is async, so settle once before the first publish. sink/wireSource are held alive
                // by wirePublisher/reconstructor below.
                _participant = new DdsParticipant();
                sink = new DdsPedReplicationSink(_participant);
                wireSource = new DdsPedReplicationSource(_participant);
                Thread.Sleep(500);
            }
            else
            {
                var bus = new InMemoryPedReplicationBus();
                sink = bus.Sink;
                wireSource = bus.Source;
            }

            wirePublisher = new PedReplicationPublisher(sink, scheduler, governor, meter, stepDt: RemotePedServer.Dt);
        }

        var reconstructor = new PedRemoteReconstructor(wireSource);

        var engine = new Engine();
        engine.LoadNetwork(NetPath); // geometry-only backdrop (both roles load the committed plaza net locally)

        var now = 0.0;

        return (engine, _engineClock =>
        {
            if (server is not null)
            {
                // Server role: advance the sim one tick, publish its whole-step event batch, mirror the
                // swept source position for the render.
                var events = server.Step();
                wirePublisher!.Publish(events);
                now = server.Now;
                _sourcePos = server.SourcePos;
            }
            else
            {
                // Subscribe-only: advance a local playout clock at the same cadence the publisher uses.
                now += RemotePedServer.Dt;
            }

            reconstructor.Pump(now);

            // Build the snapshot: reconstructed (IG) pose from the wire; server ground-truth rings only when
            // this process HAS a server (subscribe-only shows IG discs alone).
            IReadOnlyCollection<int> ids = server is not null ? server.ActiveIds : reconstructor.KnownIds;
            var poses = new List<RemotePedPose>(ids.Count);
            var highPower = 0;
            var frameMaxErr = 0.0;
            foreach (var id in ids)
            {
                var hasSrv = server is not null;
                var srv = hasSrv ? server!.ServerPositionOf(id) : default;

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

                    if (hasSrv)
                    {
                        var err = (srv - igPos).Abs;
                        if (err > frameMaxErr)
                        {
                            frameMaxErr = err;
                        }
                    }
                }

                poses.Add(new RemotePedPose(
                    id, hasIg ? igPos.X : 0.0, hasIg ? igPos.Y : 0.0, regime, hasIg, srv.X, srv.Y, hasSrv));
            }

            if (frameMaxErr > MaxRenderErrorMeters)
            {
                MaxRenderErrorMeters = frameMaxErr;
            }

            // Whole-run average over the single multicast stream (a short rolling window reads 0 in the DR
            // gate's quiet spells). Meter is publisher-side, so subscribe-only reports 0 (its bytes are the
            // publisher process's concern).
            var mbit = meter.MbitPerSecond(Math.Max(now, RemotePedServer.Dt));
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
        // ring) so a viewer can tie a promotion to the source being near a walker. Only the server role knows
        // it; a subscribe-only remote IG omits it.
        if (_drawSource)
        {
            var srcCenter = Renderer.Flip(_sourcePos.X, _sourcePos.Y);
            global::Raylib_cs.Raylib.DrawCircleV(srcCenter, 1.5f, SourceColor);
            global::Raylib_cs.Raylib.DrawCircleLinesV(srcCenter, (float)RemotePedServer.PromoteRadius, SourceColor);
        }

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
