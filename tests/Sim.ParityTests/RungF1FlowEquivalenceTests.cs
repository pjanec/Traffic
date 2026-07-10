using Sim.Core;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// Rung F1 test: a deterministic <flow> must expand to exactly the vehicles a hand-listed set of
// <vehicle>s would, and produce a byte-identical trajectory. Validated ENGINE-vs-ENGINE (no SUMO
// golden needed): scenarios/56-flow-equiv has a flow.rou.xml (<flow period=6 begin=0 end=30>) and
// an explicit.rou.xml (the same 5 vehicles f.0..f.4 by hand); both run against the same net/config
// and must match point-for-point.
public class RungF1FlowEquivalenceTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "56-flow-equiv");

    private static TrajectorySet Run(string rouFile)
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, rouFile),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        return engine.Run(60);
    }

    [Fact]
    public void DeterministicFlow_ExpandsTo_And_MatchesExplicitVehicles()
    {
        var flow = Run("flow.rou.xml");
        var explicitRun = Run("explicit.rou.xml");

        // Non-vacuous: the flow actually expanded to the expected 5 vehicles f.0..f.4.
        var expected = new[] { "f.0", "f.1", "f.2", "f.3", "f.4" };
        Assert.Equal(expected.OrderBy(x => x), flow.VehicleIds.OrderBy(x => x));
        Assert.Equal(expected.OrderBy(x => x), explicitRun.VehicleIds.OrderBy(x => x));

        // Identical trajectory, point for point.
        foreach (var id in expected)
        {
            var flowPts = flow.PointsFor(id);
            var explicitPts = explicitRun.PointsFor(id);
            Assert.Equal(explicitPts.Count, flowPts.Count);
            foreach (var (time, ep) in explicitPts)
            {
                Assert.True(flowPts.ContainsKey(time), $"{id}: flow missing time {time}");
                var fp = flowPts[time];
                Assert.Equal(ep.Lane, fp.Lane);
                Assert.Equal(ep.Pos, fp.Pos, precision: 9);
                Assert.Equal(ep.Speed, fp.Speed, precision: 9);
            }
        }
    }

    [Fact]
    public void FlowExpansion_HonorsVehsPerHour_And_Number()
    {
        // vehsPerHour: 600/h = one every 6 s. Over [0, 30) that is 5 vehicles.
        var vph = DemandParser.ParseXml(
            "<routes><vType id='c' vClass='passenger' sigma='0'/><route id='r' edges='e0'/>" +
            "<flow id='g' type='c' route='r' begin='0' end='30' vehsPerHour='600'/></routes>");
        Assert.Equal(5, vph.Vehicles.Count);
        Assert.Equal(new[] { 0.0, 6.0, 12.0, 18.0, 24.0 }, vph.Vehicles.Select(v => v.Depart));
        Assert.Equal(new[] { "g.0", "g.1", "g.2", "g.3", "g.4" }, vph.Vehicles.Select(v => v.Id));

        // number+end: 4 vehicles spread evenly over [0, 40) -> period 10 -> departs 0,10,20,30.
        var num = DemandParser.ParseXml(
            "<routes><vType id='c' vClass='passenger' sigma='0'/><route id='r' edges='e0'/>" +
            "<flow id='h' type='c' route='r' begin='0' end='40' number='4'/></routes>");
        Assert.Equal(4, num.Vehicles.Count);
        Assert.Equal(new[] { 0.0, 10.0, 20.0, 30.0 }, num.Vehicles.Select(v => v.Depart));
    }

    [Fact]
    public void ProbabilisticFlow_IsParsedAsATemplate_NotExpanded()
    {
        // F2: a probability= flow is NOT expanded to concrete VehicleDefs at load (its arrivals are
        // decided per-step at runtime); it is carried through as a ProbabilisticFlow template.
        var m = DemandParser.ParseXml(
            "<routes><vType id='c' vClass='passenger' sigma='0'/><route id='r' edges='e0'/>" +
            "<flow id='p' type='c' route='r' begin='0' end='30' probability='0.1'/></routes>");
        Assert.Empty(m.Vehicles);
        var flow = Assert.Single(m.ProbabilisticFlows);
        Assert.Equal("p", flow.Id);
        Assert.Equal(0.1, flow.Probability);
        Assert.Equal(0.0, flow.Begin);
        Assert.Equal(30.0, flow.End);

        // probability= is mutually exclusive with the deterministic rate forms.
        Assert.Throws<InvalidDataException>(() => DemandParser.ParseXml(
            "<routes><vType id='c' vClass='passenger' sigma='0'/><route id='r' edges='e0'/>" +
            "<flow id='p' type='c' route='r' begin='0' end='30' probability='0.1' period='5'/></routes>"));
        // and must be in (0, 1].
        Assert.Throws<InvalidDataException>(() => DemandParser.ParseXml(
            "<routes><vType id='c' vClass='passenger' sigma='0'/><route id='r' edges='e0'/>" +
            "<flow id='p' type='c' route='r' begin='0' end='30' probability='1.5'/></routes>"));
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
