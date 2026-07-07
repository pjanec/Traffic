using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

public class ToleranceConfigTests
{
    [Fact]
    public void Parse_ReadsAllFields()
    {
        const string json = """
        {
          "parityMode": "exact",
          "comparedAttributes": ["lane", "pos", "speed"],
          "pos": 0.001,
          "speed": 0.001,
          "x": 0.01,
          "y": 0.01,
          "angle": 0.1,
          "acceleration": 0.01
        }
        """;

        var config = ToleranceConfig.Parse(json);

        Assert.Equal(ParityMode.Exact, config.ParityMode);
        Assert.Equal(new[] { "lane", "pos", "speed" }, config.ComparedAttributes);
        Assert.Equal(0.001, config.ToleranceFor("pos"));
        Assert.Equal(0.001, config.ToleranceFor("speed"));
        Assert.Equal(0.0, config.ToleranceFor("lane"));
    }

    [Fact]
    public void Parse_DefaultsComparedAttributesWhenOmitted()
    {
        const string json = """{ "parityMode": "exact", "pos": 1.0, "speed": 1.0 }""";

        var config = ToleranceConfig.Parse(json);

        Assert.Equal(ToleranceConfig.DefaultComparedAttributes, config.ComparedAttributes);
    }

    [Fact]
    public void ToleranceFor_MissingAttribute_Throws()
    {
        const string json = """{ "parityMode": "exact", "comparedAttributes": ["x"] }""";

        var config = ToleranceConfig.Parse(json);

        Assert.Throws<InvalidOperationException>(() => config.ToleranceFor("x"));
    }
}
