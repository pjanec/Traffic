using System.IO;
using System.Linq;
using Sim.Core;
using Sim.Host;
using Sim.Ingest;
using Sim.Replication;

namespace Sim.Viz;

// T3 (docs/VIZ-UNIFICATION-DESIGN.md): an IVizReplaySource adapter that runs a REAL deterministic
// Engine on a scenario dir (net + rou + sumocfg) and publishes each step's snapshot onto an
// in-memory replication bus -- the SAME Engine -> ReplicationPublisher -> InMemoryReplicationBus ->
// VehicleSource wiring LiveCitySim uses (LiveCitySim.cs ~lines 53-56, 214, 506-516). Renders the
// REAL engine trajectory (deterministic: same input => byte-identical run), reconstructed smooth by
// VizReplayBuilder -- the same trajectory the committed golden.fcd.xml encodes, not the golden itself.
// Vehicle-only (PedSource = null; a scenario dir has no PedNetwork).
//
// Deliberately uses a plain `new Engine()` (CoordinatedLaneChange defaults to false, the deterministic
// parity/SUMO-anchor mode -- see Sim.Core/Engine.cs's `CoordinatedLaneChange` field), NOT Sim.Run's
// `--coordinated-lc` product default, so the rendered trajectory is the one goldens were generated
// against (see the golden spot-check in T3's success conditions).
internal sealed class EngineScenarioSource : IVizReplaySource, System.IDisposable
{
    private readonly Engine _engine;
    private readonly NetworkModel _model;
    private readonly ReplicationPublisher _publisher = new();
    private readonly InMemoryReplicationBus _bus = new();
    private readonly NetworkLaneSource _lanes;
    private readonly NetworkPayload _network;
    private readonly (double, double, double, double) _view;
    private readonly double _dt;
    private bool _geometryPublished;

    internal int Steps { get; }

    internal EngineScenarioSource(string scenarioDir)
    {
        var cfgPath = Directory.GetFiles(scenarioDir, "*.sumocfg").Single();
        var config = ScenarioConfigParser.Parse(cfgPath);
        _dt = config.StepLength;
        Steps = (int)System.Math.Round((config.End - config.Begin) / config.StepLength);

        var netPath = config.NetFile is not null
            ? Path.Combine(scenarioDir, config.NetFile)
            : Directory.GetFiles(scenarioDir, "*.net.xml").Single();
        _model = NetworkParser.Parse(netPath);
        _lanes = new NetworkLaneSource(_model);
        _network = PayloadBuilder.BuildNetwork(_model);
        _view = WholeNetView(_network);

        _engine = new Engine();
        if (config.RouteFiles.Count > 0)
        {
            _engine.LoadScenario(cfgPath);
        }
        else
        {
            var rouPath = Directory.GetFiles(scenarioDir, "*.rou.xml").Single();
            _engine.LoadScenario(netPath, rouPath, cfgPath);
        }
    }

    public double Dt => _dt;
    public IReplicationSource VehicleSource => _bus.Source;
    public IPedReplicationSource? PedSource => null;
    public ILaneShapeSource Lanes => _lanes;
    public NetworkPayload Network => _network;
    public (double X0, double Y0, double X1, double Y1) View => _view;

    public void Step()
    {
        _engine.Step();
        var snap = SimulationSnapshot.Capture(_engine);
        if (!_geometryPublished)
        {
            _publisher.PublishGeometryOnce(_model, _bus.Sink);
            _geometryPublished = true;
        }

        _publisher.PublishStep(snap, _bus.Sink);
    }

    public void Dispose()
    {
        _bus.Sink.Dispose();
        _bus.Source.Dispose();
    }

    // Whole-network bbox over every lane vertex (rounded 2dp, matching PayloadBuilder.R's convention),
    // so the camera frames the entire scenario -- there is no LiveCityConfig crop box for a scenario dir.
    private static (double, double, double, double) WholeNetView(NetworkPayload net)
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        foreach (var lane in net.Lanes)
        {
            for (var p = 0; p + 1 < lane.Shape.Length; p += 2)
            {
                var x = lane.Shape[p];
                var y = lane.Shape[p + 1];
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
            }
        }

        if (double.IsInfinity(minX))
        {
            return (0, 0, 100, 100);
        }

        double R(double v) => System.Math.Round(v, 2, System.MidpointRounding.AwayFromZero);
        return (R(minX), R(minY), R(maxX), R(maxY));
    }
}
