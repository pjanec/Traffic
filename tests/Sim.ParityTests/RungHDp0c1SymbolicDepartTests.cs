using Sim.Core;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// P0-C1 (docs/HIGH-DENSITY-P0-DESIGN.md "P0-C"/"P0-C1"): symbolic depart attributes --
// departSpeed="max", departLane="best", departPos="stop" (lane <stop> only). These are OFFLINE
// unit tests over DemandParser (spec-kind mapping + the loud-reject-on-unsupported-keyword
// contract) plus one small in-engine sanity check reusing the already-committed
// scenarios/18-strategic-turnlane net (its E1 has an unambiguous best-continuation lane, E1_1,
// exactly like scenarios/42-symbolic-depart's own comment documents) -- full engine-vs-SUMO
// trajectory parity is scenario 42's job, not this rung's.
public class RungHDp0c1SymbolicDepartTests
{
    private const string RouHeader = """
        <routes>
            <vType id="car" vClass="passenger" sigma="0"/>
            <route id="r0" edges="E0 E1"/>
        """;

    [Fact]
    public void DepartSpeed_Max_ParsesAsMaxSpec()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0" departSpeed="max"/>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartSpeedSpec.Max, v.DepartSpeed.Kind);
    }

    [Fact]
    public void DepartLane_Best_ParsesAsBestSpec()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0" departLane="best"/>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartLaneSpec.Best, v.DepartLaneIndex.Kind);
    }

    [Fact]
    public void DepartPos_Stop_ParsesAsStopSpec()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0" departPos="stop">
                    <stop lane="E1_0" startPos="10" endPos="20" duration="5"/>
                </vehicle>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartPosSpec.Stop, v.DepartPos.Kind);
    }

    // GAP-3 (docs/HIGH-DENSITY-CALIBRATION-DESIGN.md §4): departPos="base" resolves to SUMO's
    // DepartPosDefinition::BASE (MSBaseVehicle::basePos), NOT a hardcoded 0. Parsed here as the
    // Base spec Kind; the concrete basePos is computed at insertion (Engine.BasePos), verified
    // numerically by RungHDp0c3BaseDepartPosTests.
    [Fact]
    public void DepartPos_Base_ParsesAsBaseSpec()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0" departPos="base"/>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartPosSpec.Base, v.DepartPos.Kind);
    }

    [Fact]
    public void NumericDepartAttributes_StillParseAsGiven_WithTheirLiteral()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0" departPos="12.5" departSpeed="13.89" departLane="1"/>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartPosSpec.Given, v.DepartPos.Kind);
        Assert.Equal(12.5, v.DepartPos.Literal, 9);
        Assert.Equal(DepartSpeedSpec.Given, v.DepartSpeed.Kind);
        Assert.Equal(13.89, v.DepartSpeed.Literal, 9);
        Assert.Equal(DepartLaneSpec.Given, v.DepartLaneIndex.Kind);
        Assert.Equal(1, v.DepartLaneIndex.Literal);
    }

    // Absent attributes must still default exactly like before this rung (Given + 0), not throw or
    // silently pick a symbolic Kind.
    [Fact]
    public void AbsentDepartAttributes_DefaultToGivenZero()
    {
        var demand = DemandParser.ParseXml(RouHeader + """
                <vehicle id="v0" type="car" route="r0" depart="0"/>
            </routes>
            """);

        var v = Assert.Single(demand.Vehicles);
        Assert.Equal(DepartPosValue.Given(0.0), v.DepartPos);
        Assert.Equal(DepartSpeedValue.Given(0.0), v.DepartSpeed);
        Assert.Equal(DepartLaneValue.Given(0), v.DepartLaneIndex);
    }

    [Theory]
    [InlineData("departSpeed", "free")]
    [InlineData("departSpeed", "desired")]
    [InlineData("departSpeed", "random")]
    [InlineData("departSpeed", "avg")]
    public void UnsupportedDepartSpeedKeyword_ThrowsInvalidDataException_NamingTheValue(string attr, string value)
    {
        var rou = RouHeader + $"""
                <vehicle id="v0" type="car" route="r0" depart="0" {attr}="{value}"/>
            </routes>
            """;

        var ex = Assert.Throws<InvalidDataException>(() => DemandParser.ParseXml(rou));
        Assert.Contains(value, ex.Message);
        Assert.Contains(attr, ex.Message);
    }

    [Theory]
    [InlineData("free")]
    [InlineData("random")]
    [InlineData("allowed")]
    [InlineData("first")]
    public void UnsupportedDepartLaneKeyword_ThrowsInvalidDataException_NamingTheValue(string value)
    {
        var rou = RouHeader + $"""
                <vehicle id="v0" type="car" route="r0" depart="0" departLane="{value}"/>
            </routes>
            """;

        var ex = Assert.Throws<InvalidDataException>(() => DemandParser.ParseXml(rou));
        Assert.Contains(value, ex.Message);
        Assert.Contains("departLane", ex.Message);
    }

    [Theory]
    [InlineData("random")]
    [InlineData("free")]
    [InlineData("last")]
    [InlineData("random_free")]
    [InlineData("speedLimit")]
    public void UnsupportedDepartPosKeyword_ThrowsInvalidDataException_NamingTheValue(string value)
    {
        var rou = RouHeader + $"""
                <vehicle id="v0" type="car" route="r0" depart="0" departPos="{value}"/>
            </routes>
            """;

        var ex = Assert.Throws<InvalidDataException>(() => DemandParser.ParseXml(rou));
        Assert.Contains(value, ex.Message);
        Assert.Contains("departPos", ex.Message);
    }

    // <flow> takes the SAME symbolic-aware parse helpers as <vehicle> -- both the deterministic
    // (period/vehsPerHour/number) and probabilistic (probability=) expansion paths read the shared
    // departPos/departSpeed/departLane locals, so one flow form is enough to prove both are wired.
    [Fact]
    public void Flow_DeterministicForm_SymbolicDepartLane_ParsesAsBestSpec()
    {
        var rou = RouHeader + """
                <flow id="f0" type="car" route="r0" begin="0" end="10" period="5" departLane="best"/>
            </routes>
            """;

        var demand = DemandParser.ParseXml(rou);

        Assert.NotEmpty(demand.Vehicles);
        Assert.All(demand.Vehicles, v => Assert.Equal(DepartLaneSpec.Best, v.DepartLaneIndex.Kind));
    }

    [Fact]
    public void Flow_ProbabilisticForm_SymbolicDepartSpeed_ParsesAsMaxSpec()
    {
        var rou = RouHeader + """
                <flow id="f0" type="car" route="r0" begin="0" end="10" probability="0.5" departSpeed="max"/>
            </routes>
            """;

        var demand = DemandParser.ParseXml(rou);

        var flow = Assert.Single(demand.ProbabilisticFlows);
        Assert.Equal(DepartSpeedSpec.Max, flow.DepartSpeed.Kind);
    }

    // In-engine sanity check: departSpeed="max" and departLane="best" together, on
    // scenarios/18-strategic-turnlane's net (E1 has 2 lanes; only E1_1 continues onward to E2, so
    // "best" is unambiguous by route-continuation length alone -- no occupancy tiebreak needed,
    // exactly like scenarios/42-symbolic-depart's own header comment). Proves the engine inserts
    // without crashing, on the CONTINUING lane (not the dead-end lane 0), at the lane's speed limit
    // (13.89 m/s, no leader to clamp against). Full trajectory-vs-SUMO parity is scenario 42's job.
    [Fact]
    public void Engine_DepartSpeedMaxAndDepartLaneBest_InsertsOnBestLane_AtLaneSpeedLimit()
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "18-strategic-turnlane");
        var rouPath = Path.Combine(Path.GetTempPath(), $"p0c1-{Guid.NewGuid():N}.rou.xml");
        File.WriteAllText(rouPath, """
            <routes>
                <vType id="car" vClass="passenger" sigma="0"/>
                <route id="r0" edges="E1 E2"/>
                <vehicle id="veh0" type="car" route="r0" depart="0" departPos="0" departSpeed="max" departLane="best"/>
            </routes>
            """);

        try
        {
            var engine = new Engine();
            engine.LoadScenario(
                Path.Combine(dir, "net.net.xml"),
                rouPath,
                Path.Combine(dir, "config.sumocfg"));

            var traj = engine.Run(1);

            Assert.Contains("veh0", traj.VehicleIds);
            var points = traj.PointsFor("veh0");
            Assert.NotEmpty(points);
            var first = points.OrderBy(kv => kv.Key).First().Value;

            // E1_1 is the only lane that continues onward to E2 (E1_0 dead-ends at junction B) --
            // "best" must resolve to it, not the dead-end lane.
            Assert.Equal("E1_1", first.Lane);
            // departSpeed="max" with no leader: lane speed (13.89) x speedFactor (1.0), well under
            // the passenger default maxSpeed -- no clamp applies.
            Assert.Equal(13.89, first.Speed, 2);
        }
        finally
        {
            File.Delete(rouPath);
        }
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
