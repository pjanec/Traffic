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
}
