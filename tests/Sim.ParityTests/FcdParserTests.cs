using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

public class FcdParserTests
{
    private static string FixturePath(string fileName) => Path.Combine(AppContext.BaseDirectory, "Fixtures", fileName);

    [Fact]
    public void ParsesFixture_FieldValuesRoundTrip()
    {
        var trajectory = FcdParser.Parse(FixturePath("sample.fcd.xml"));

        Assert.Equal(new[] { "veh0" }, trajectory.VehicleIds);

        Assert.True(trajectory.TryGet("veh0", 0.0, out var atZero));
        Assert.Equal("edge0_0", atZero.Lane);
        Assert.Equal(5.10, atZero.Pos, precision: 3);
        Assert.Equal(0.00, atZero.Speed, precision: 3);
        Assert.Equal(5.10, atZero.X, precision: 3);
        Assert.Equal(1.60, atZero.Y, precision: 3);
        Assert.Equal(90.00, atZero.Angle, precision: 3);
        Assert.Null(atZero.Acceleration);

        Assert.True(trajectory.TryGet("veh0", 1.0, out var atOne));
        Assert.Equal(6.10, atOne.Pos, precision: 3);
        Assert.Equal(1.00, atOne.Speed, precision: 3);
        Assert.Equal(1.00, atOne.Acceleration!.Value, precision: 3);

        Assert.False(trajectory.TryGet("veh0", 2.0, out _));
    }

    [Fact]
    public void PosLat_ParsesWhenPresent_DefaultsToZeroWhenAbsent()
    {
        // Phase 2 (sublane): SUMO emits `posLat` only when --lateral-resolution > 0. The parser
        // must read it when present (t=0) and default it to 0 when absent (t=1, a phase-1 row).
        const string xml = """
            <fcd-export>
              <timestep time="0.00">
                <vehicle id="veh0" x="5.0" y="0.4" angle="90.0" speed="10.0" pos="5.0" posLat="-1.6" lane="edge0_0"/>
              </timestep>
              <timestep time="1.00">
                <vehicle id="veh0" x="15.0" y="0.0" angle="90.0" speed="10.0" pos="15.0" lane="edge0_0"/>
              </timestep>
            </fcd-export>
            """;

        var trajectory = FcdParser.ParseXml(xml);

        Assert.True(trajectory.TryGet("veh0", 0.0, out var atZero));
        Assert.Equal(-1.6, atZero.PosLat, precision: 3);

        Assert.True(trajectory.TryGet("veh0", 1.0, out var atOne));
        Assert.Equal(0.0, atOne.PosLat, precision: 3);
    }
}
