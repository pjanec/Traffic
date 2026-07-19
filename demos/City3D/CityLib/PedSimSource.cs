using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Replication;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the ped analog of SimSource. Hosts the ped
// SERVER sim (a PedLodManager over a SumoNavMesh, a swept InterestField promoting/demoting nearby walkers,
// peds spawned and cycled) and publishes each tick through the gated PedReplicationPublisher into an
// in-process InMemoryPedReplicationBus (a true byte loopback). The render side (PedReconstructor) only ever
// consumes the transport-neutral IPedReplicationSource, so a future DDS ped source swaps in without touching
// reconstruction/render.
//
// The server ped-sim + wire here is copied verbatim from src/Sim.Viewer/RemotePedOverlay.cs Build() (which
// itself mirrors Sim.Viz/SceneGen.cs BuildPedRemote), MINUS the reconstructor/render (that lives in
// PedReconstructor) and MINUS any Godot/Raylib type (CityLib stays engine-agnostic). Same net
// (scenarios/_ped/poc0-crossing-plaza), same constants, same 4 arm-crossing candidates, same low-power-only
// recycle discipline (only Remove/Add a ped whose ModelOf==PathArc; cycling a still-FreeKinematic one would
// desync the wire).
public sealed class PedSimSource : IDisposable
{
    // --- ped-sim / wire constants, copied verbatim from RemotePedOverlay (its remarks explain the
    // geometry) so this demo reconstructs the exact same crowd. ---
    private const double Cx = 120.0, Cy = 120.0;
    private const double MaxSpeed = 1.3;
    private const double PedRadius = 0.3;
    private const double ArriveRadius = 0.3;
    private const double DwellSeconds = 1.0;
    private const double PromoteRadius = 6.0;
    private const double DemoteRadius = 13.0;
    private const double Dt = 0.2;                 // ped-sim tick -- one Tick() advances this much sim time
    private const int MaxPeds = 14;
    private const int SpawnEveryNSteps = 20;       // a fresh ped roughly every 4s of ped-sim time
    private const double SweepRadius = 16.0;
    private const double SweepPeriod = 70.0;       // seconds per interest-source revolution

    private readonly PedPublisher _publisher = new();
    private readonly PedLodManager _manager;
    private readonly InterestField _field = new();
    private readonly InterestSourceId _sourceId;
    private readonly PedBandwidthMeter _meter = new();
    private readonly PedReplicationPublisher _wirePublisher;
    private readonly InMemoryPedReplicationBus _bus = new();
    private readonly WorldDisc[] _noEntities = Array.Empty<WorldDisc>();

    private readonly List<(Vec2[] Forward, Vec2[] Backward)> _validPairs = new();
    private readonly Dictionary<int, (Vec2[] Forward, Vec2[] Backward, bool GoingForward)> _pedInfo = new();
    private readonly List<int> _activeIds = new();
    private int _nextId = 1;
    private int _pairCursor;
    private double _now;
    private int _step;
    private Vec2 _sourcePos;

    // netXmlPath / walkableAddPath default to scenarios/_ped/poc0-crossing-plaza relative to repoRoot, the
    // same net RemotePedOverlay/SceneGen.BuildPedRemote drive. Main passes the repo root it already resolves
    // via ProjectSettings.GlobalizePath (CityLib is Godot-free and cannot resolve res:// itself).
    public PedSimSource(string repoRoot)
        : this(
            Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "net.net.xml"),
            Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "walkable.add.xml"))
    {
    }

    public PedSimSource(string netXmlPath, string walkableAddPath)
    {
        var pedNetwork = PedNetworkParser.Load(netXmlPath, walkableAddPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // The four arm-crossing candidates, same geometry as RemotePedOverlay/BuildPedRemote.
        var candidates = new (Vec2 A, Vec2 B)[]
        {
            (new Vec2(Cx - 7.4, Cy + 20.0), new Vec2(Cx + 7.4, Cy + 20.0)), // north arm
            (new Vec2(Cx + 20.0, Cy - 7.4), new Vec2(Cx + 20.0, Cy + 7.4)), // east arm
            (new Vec2(Cx - 7.4, Cy - 20.0), new Vec2(Cx + 7.4, Cy - 20.0)), // south arm
            (new Vec2(Cx - 20.0, Cy - 7.4), new Vec2(Cx - 20.0, Cy + 7.4)), // west arm
        };

        foreach (var (a, b) in candidates)
        {
            var path = nav.FindPath(a, b);
            if (path is { Count: >= 2 })
            {
                var fwd = path.ToArray();
                _validPairs.Add((fwd, fwd.Reverse().ToArray()));
            }
        }

        if (_validPairs.Count == 0)
        {
            throw new InvalidOperationException("PedSimSource: no valid arm-crossing routes found on the net.");
        }

        _manager = new PedLodManager(nav, _publisher, ArriveRadius, DwellSeconds);
        _sourcePos = new Vec2(Cx - 25.0, Cy);
        var source = new InterestSource(_sourcePos, PromoteRadius, DemoteRadius);
        _sourceId = _field.Register(source, InterestSourceKind.EntityAttached);

        // The wire: gated publisher (DR-error scheduler + global bandwidth governor) -> InMemory byte
        // loopback. The crowd the render side draws is reconstructed from THIS stream.
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, _meter, maxMbitPerSecond: 500.0);
        _wirePublisher = new PedReplicationPublisher(_bus.Sink, scheduler, governor, _meter, stepDt: Dt);
    }

    // The transport-neutral read side a reconstructor/renderer consumes -- IPedReplicationSource, never the
    // concrete InMemoryPedReplicationBus type, so reconstruction code never knows it's in-process.
    public IPedReplicationSource Source => _bus.Source;

    // The ped-sim's current `now` (advanced by Dt each Tick) -- the server-time axis the reconstructor is
    // pumped with.
    public double Time => _now;

    // The swept interest source position (SUMO coords), for an optional source marker. Trivial to expose.
    public (double X, double Y) SourcePosSumo => (_sourcePos.X, _sourcePos.Y);

    // Advance the ped sim ONE Dt=0.2s tick: spawn/sweep/manager.Step/recycle, then publish the whole step's
    // events as a single wire batch. Same before/after event-cursor batching as RemotePedOverlay: capture
    // the event-list cursor BEFORE this step's spawn/promotion so a freshly-spawned ped's PathArcRecord
    // rides the SAME wire batch as everything Step emits this tick.
    public void Tick()
    {
        var beforeCount = _publisher.Events.Count;

        if (_step % SpawnEveryNSteps == 0 && _activeIds.Count < MaxPeds)
        {
            Spawn(_now);
        }

        // Sweep the interest source in a slow circle so it passes near every arm's route in turn.
        var angle = (_now / SweepPeriod) * 2.0 * Math.PI;
        _sourcePos = new Vec2(Cx + (SweepRadius * Math.Cos(angle)), Cy + (SweepRadius * Math.Sin(angle)));
        _field.Move(_sourceId, _sourcePos);

        _manager.Step(_now, Dt, _field, _noEntities);
        _now += Dt;

        // Cycle each ped back and forth once it reaches its leg's end -- a sustained flow. ONLY recycles a
        // currently LOW-power ped: RemovePed/AddPed here is a demand-side "reached destination, walk back"
        // reset, not a promote/demote transition, so it publishes no DR-switch; cycling a still-FreeKinematic
        // ped this way would desync the wire (the reconstructor would keep believing it is high-power with no
        // more samples coming). A high-power ped instead demotes via Step's own dwell mechanism (which DOES
        // publish the DR-switch + fresh PathArcRecord) and cycles later.
        foreach (var id in _activeIds)
        {
            if (_manager.ModelOf(id) != PedDrModel.PathArc)
            {
                continue;
            }

            var (fwd, bwd, goingForward) = _pedInfo[id];
            var dest = goingForward ? fwd[^1] : bwd[^1];
            if ((_manager.PositionOf(id, _now) - dest).Abs < 0.75)
            {
                _manager.RemovePed(id);
                var nextPath = goingForward ? bwd : fwd;
                _manager.AddPed(id, nextPath, MaxSpeed, PedRadius, _now);
                _pedInfo[id] = (fwd, bwd, !goingForward);
            }
        }

        var newEvents = new List<PedEvent>(_publisher.Events.Count - beforeCount);
        for (var e = beforeCount; e < _publisher.Events.Count; e++)
        {
            newEvents.Add(_publisher.Events[e]);
        }

        _wirePublisher.Publish(newEvents);
        _step++;
    }

    private void Spawn(double t)
    {
        var (fwd, bwd) = _validPairs[_pairCursor % _validPairs.Count];
        var goingForward = (_pairCursor / _validPairs.Count) % 2 == 0;
        _pairCursor++;

        var id = _nextId++;
        var path = goingForward ? fwd : bwd;
        _manager.AddPed(id, path, MaxSpeed, PedRadius, t);
        _pedInfo[id] = (fwd, bwd, goingForward);
        _activeIds.Add(id);
    }

    public void Dispose()
    {
        _bus.Sink.Dispose();
        _bus.Source.Dispose();
    }
}
