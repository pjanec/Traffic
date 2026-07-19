using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Viewer;

// D3b (docs/PEDESTRIAN-DDS-TRANSPORT-TASKS.md): the crossing-plaza pedestrian SERVER-sim, factored out of
// RemotePedOverlay so BOTH the in-process demo (RemotePedOverlay's InMemory/Dds modes) AND the headless
// two-process publisher (`--mode ped-publish`) drive the SAME sim -- one server, no duplication. It owns
// the ped nav + PedLodManager + swept interest source + spawn/cycle demand; each Step() advances one 0.2 s
// tick and returns the PedEvents that tick emitted (to hand to a PedReplicationPublisher). The wire
// (publisher/sink/reconstructor) and the render live in the caller, not here.
public sealed class RemotePedServer
{
    // ped-sim / scenario constants -- copied verbatim from SceneGen.BuildPedRemote (its remarks explain the
    // geometry) so every consumer reconstructs the exact same crowd.
    public const double Cx = 120.0, Cy = 120.0;
    public const double MaxSpeed = 1.3;
    public const double PedRadius = 0.3;
    public const double ArriveRadius = 0.3;
    public const double DwellSeconds = 1.0;
    public const double PromoteRadius = 6.0;
    public const double DemoteRadius = 13.0;
    public const double Dt = 0.2;                 // ped-sim tick -- DECOUPLED from any 1.0 s Engine step
    public const int MaxPeds = 14;
    public const int SpawnEveryNSteps = 20;       // a fresh ped roughly every 4 s of ped-sim time
    public const double SweepRadius = 16.0;
    public const double SweepPeriod = 70.0;       // seconds per interest-source revolution

    public static string ResolveNetPath(string repoRoot) =>
        Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "net.net.xml");

    private readonly PedPublisher _publisher;
    private readonly PedLodManager _manager;
    private readonly InterestField _field;
    private readonly InterestSourceId _sourceId;
    private readonly List<(Vec2[] Forward, Vec2[] Backward)> _validPairs;
    private readonly Dictionary<int, (Vec2[] Forward, Vec2[] Backward, bool GoingForward)> _pedInfo = new();
    private readonly List<int> _activeIds = new();
    private readonly WorldDisc[] _noEntities = Array.Empty<WorldDisc>();

    private int _nextId = 1;
    private int _pairCursor;
    private int _step;

    public RemotePedServer(string repoRoot)
    {
        NetPath = ResolveNetPath(repoRoot);
        var walkableAddPath = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza", "walkable.add.xml");
        var pedNetwork = PedNetworkParser.Load(NetPath, walkableAddPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // The four arm-crossing candidates, same geometry as BuildPedRemote.
        var candidates = new (Vec2 A, Vec2 B)[]
        {
            (new Vec2(Cx - 7.4, Cy + 20.0), new Vec2(Cx + 7.4, Cy + 20.0)), // north arm
            (new Vec2(Cx + 20.0, Cy - 7.4), new Vec2(Cx + 20.0, Cy + 7.4)), // east arm
            (new Vec2(Cx - 7.4, Cy - 20.0), new Vec2(Cx + 7.4, Cy - 20.0)), // south arm
            (new Vec2(Cx - 20.0, Cy - 7.4), new Vec2(Cx - 20.0, Cy + 7.4)), // west arm
        };

        _validPairs = new List<(Vec2[] Forward, Vec2[] Backward)>();
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
            throw new InvalidOperationException("RemotePedServer: no valid arm-crossing routes found on the net.");
        }

        _publisher = new PedPublisher();
        _manager = new PedLodManager(nav, _publisher, ArriveRadius, DwellSeconds);
        _field = new InterestField();
        SourcePos = new Vec2(Cx - 25.0, Cy);
        _sourceId = _field.Register(new InterestSource(SourcePos, PromoteRadius, DemoteRadius), InterestSourceKind.EntityAttached);
    }

    public string NetPath { get; }
    public double Now { get; private set; }
    public Vec2 SourcePos { get; private set; }
    public IReadOnlyList<int> ActiveIds => _activeIds;

    public Vec2 ServerPositionOf(int id) => _manager.PositionOf(id, Now);
    public PedDrModel ServerModelOf(int id) => _manager.ModelOf(id);

    // Advance one 0.2 s tick and return the PedEvents emitted this tick (spawn PathArcs, DR-switches, crowd
    // samples, ...) -- the whole-step batch a PedReplicationPublisher.Publish expects (see BuildPedRemote's
    // whole-step-granularity note). Mirrors RemotePedOverlay's original per-step hook exactly.
    public IReadOnlyList<PedEvent> Step()
    {
        var beforeCount = _publisher.Events.Count;

        if (_step % SpawnEveryNSteps == 0 && _activeIds.Count < MaxPeds)
        {
            Spawn(Now);
        }

        // Sweep the interest source in a slow circle so it passes near every arm's route in turn.
        var angle = (Now / SweepPeriod) * 2.0 * Math.PI;
        SourcePos = new Vec2(Cx + (SweepRadius * Math.Cos(angle)), Cy + (SweepRadius * Math.Sin(angle)));
        _field.Move(_sourceId, SourcePos);

        _manager.Step(Now, Dt, _field, _noEntities);
        Now += Dt;

        // Cycle each LOW-power ped back and forth once it reaches its leg's end -- a sustained flow (see the
        // original overlay remark: only a PathArc ped is recycled, so it never desyncs a high-power stream).
        foreach (var id in _activeIds)
        {
            if (_manager.ModelOf(id) != PedDrModel.PathArc)
            {
                continue;
            }

            var (fwd, bwd, goingForward) = _pedInfo[id];
            var dest = goingForward ? fwd[^1] : bwd[^1];
            if ((_manager.PositionOf(id, Now) - dest).Abs < 0.75)
            {
                _manager.RemovePed(id);
                var nextPath = goingForward ? bwd : fwd;
                _manager.AddPed(id, nextPath, MaxSpeed, PedRadius, Now);
                _pedInfo[id] = (fwd, bwd, !goingForward);
            }
        }

        _step++;

        var newEvents = new List<PedEvent>(_publisher.Events.Count - beforeCount);
        for (var e = beforeCount; e < _publisher.Events.Count; e++)
        {
            newEvents.Add(_publisher.Events[e]);
        }

        return newEvents;
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
}
