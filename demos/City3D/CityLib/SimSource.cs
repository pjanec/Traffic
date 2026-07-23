using Sim.Core;
using Sim.Host;
using Sim.Ingest;
using Sim.Replication;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Data path" / "SimSource" -- the LOCAL, in-process data path: hosts an
// Engine + SimulationRunner + ReplicationPublisher, publishing every step into an in-process
// InMemoryReplicationBus. The render side (Reconstructor, below) only ever consumes the bus's
// IReplicationSource + an ILaneShapeSource, so swapping to a remote DDS source later touches only THIS
// class's construction, never the reconstruction/render code (the local<->remote seam, T1.2 condition 3).
public sealed class SimSource : IDisposable
{
    private readonly Engine _engine;
    private readonly SimulationRunner _runner;
    private readonly ReplicationPublisher _publisher = new();
    private readonly InMemoryReplicationBus _bus = new();
    private bool _geometryPublished;

    public SimSource(string netXmlPath, string rouXmlPath, string sumocfgPath)
    {
        // Parsed independently of the stepping Engine (ReplicationPublisher takes a NetworkModel, not an
        // Engine -- mirrors tests/Sim.Host.Tests/ReplicationPublisherTests.cs's own pattern), so any host
        // (this one, or a future Sim.Host.App) can supply its own parse.
        Network = NetworkParser.Parse(netXmlPath);

        _engine = new Engine();
        _engine.LoadScenario(netXmlPath, rouXmlPath, sumocfgPath);

        // Realism (demo-only; 0 = off = byte-identical parity): a queued/standing car must not snap a full
        // lane sideways -- it sorts into its lane only while moving. The --scenario path set nothing (free
        // SUMO-parity lane changes in queues); mirror LiveCityConfig / SceneGen.BuildLiveCity. Env-tunable
        // via CITY3D_LCMIN; keep <= ~2.0 for deadlock safety.
        _engine.LaneChangeMinSpeed =
            double.TryParse(System.Environment.GetEnvironmentVariable("CITY3D_LCMIN"), out var lcm) ? lcm : 1.5;

        _runner = new SimulationRunner(_engine);

        // The LOCAL, Z-aware lane source (straight off the parsed NetworkModel -- carries Lane.ShapeZ).
        // Used for the "local" half of T1.2 condition 3's seam check; the wire-fed ReplicationLaneShapeSource
        // (built from bus.Source.Geometry) is the "remote-shaped" half, exercised identically.
        LocalLanes = new NetworkLaneSource(Network);
    }

    public NetworkModel Network { get; }

    public NetworkLaneSource LocalLanes { get; }

    // The transport-neutral read side a renderer/reconstructor consumes -- IReplicationSource, never the
    // concrete InMemoryReplicationBus type, so reconstruction code never knows it's in-process.
    public IReplicationSource Source => _bus.Source;

    public double Time { get; private set; }

    // The engine's current published frame (ground truth), for callers/tests that want to correlate a
    // reconstructed render pose against the authoritative parity-side state (e.g. SimulationSnapshot.Angle,
    // the ParityTangent heading) -- mirrors samples/SumoSharp.GameHostSample/GameHost.cs's own
    // `_runner.Snapshot` exposure.
    public SimulationSnapshot Snapshot => _runner.Snapshot;

    // Advance the simulation one step, publish it, and drain the bus so Source's dictionaries are
    // up to date synchronously (in-process: no network latency to wait out). Geometry (durable, static) is
    // published once, on the first call, before the first frame -- readers see lane geometry no later than
    // the first vehicle sample that could reference it.
    public void Tick()
    {
        _runner.Tick();

        if (!_geometryPublished)
        {
            _publisher.PublishGeometryOnce(Network, _bus.Sink);
            _geometryPublished = true;
        }

        _publisher.PublishStep(_runner.Snapshot, _bus.Sink);
        _bus.Source.Pump();
        Time = _runner.Snapshot.Time;
    }

    public void Dispose()
    {
        _runner.Dispose();
        _bus.Sink.Dispose();
        _bus.Source.Dispose();
    }
}
