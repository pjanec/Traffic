using System.Text.Json;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// vType defaults resolver init cross-check for rung 10 (traffic light). golden.vtype.json is
// keyed by vType id under "vTypes" with a single entry, "passenger0" -- run BEFORE the
// trajectory parity test, exactly like Rung9aInitCrossCheckTests (CLAUDE.md rule 6: match
// vType/init first when a parity gap appears).
public class Rung10InitCrossCheckTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light");

    [Fact]
    public void ResolvePassenger_Passenger0_MatchesGoldenVTypeJson()
    {
        var demand = DemandParser.Parse(Path.Combine(ScenarioDir, "rou.rou.xml"));
        var rawVType = demand.VTypesById["passenger0"];
        var resolved = VTypeDefaults.ResolvePassenger(rawVType);

        var golden = LoadGoldenVType("passenger0");

        Assert.Equal("passenger", resolved.VClass);
        Assert.Equal(golden.CarFollowModel, resolved.CarFollowModel);
        Assert.Equal(golden.Length, resolved.Length, precision: 9);
        Assert.Equal(golden.MinGap, resolved.MinGap, precision: 9);
        Assert.Equal(golden.MaxSpeed, resolved.MaxSpeed, precision: 9);
        Assert.Equal(golden.Accel, resolved.Accel, precision: 9);
        Assert.Equal(golden.Decel, resolved.Decel, precision: 9);
        Assert.Equal(golden.EmergencyDecel, resolved.EmergencyDecel, precision: 9);
        Assert.Equal(golden.ApparentDecel, resolved.ApparentDecel, precision: 9);
        Assert.Equal(golden.Sigma, resolved.Sigma, precision: 9);
        Assert.Equal(golden.Tau, resolved.Tau, precision: 9);
        Assert.Equal(golden.SpeedFactor, resolved.SpeedFactor, precision: 9);
        Assert.Equal(golden.Width, resolved.Width, precision: 9);
        Assert.Equal(golden.Height, resolved.Height, precision: 9);
    }

    private static GoldenVType LoadGoldenVType(string vTypeId)
    {
        var json = File.ReadAllText(Path.Combine(ScenarioDir, "golden.vtype.json"));
        using var doc = JsonDocument.Parse(json);
        var vType = doc.RootElement.GetProperty("vTypes").GetProperty(vTypeId);

        return new GoldenVType(
            CarFollowModel: vType.GetProperty("carFollowModel").GetString()!,
            Length: vType.GetProperty("length").GetDouble(),
            MinGap: vType.GetProperty("minGap").GetDouble(),
            MaxSpeed: vType.GetProperty("maxSpeed").GetDouble(),
            Accel: vType.GetProperty("accel").GetDouble(),
            Decel: vType.GetProperty("decel").GetDouble(),
            EmergencyDecel: vType.GetProperty("emergencyDecel").GetDouble(),
            ApparentDecel: vType.GetProperty("apparentDecel").GetDouble(),
            Sigma: vType.GetProperty("sigma").GetDouble(),
            Tau: vType.GetProperty("tau").GetDouble(),
            SpeedFactor: vType.GetProperty("speedFactor").GetDouble(),
            Width: vType.GetProperty("width").GetDouble(),
            Height: vType.GetProperty("height").GetDouble());
    }

    private sealed record GoldenVType(
        string CarFollowModel,
        double Length,
        double MinGap,
        double MaxSpeed,
        double Accel,
        double Decel,
        double EmergencyDecel,
        double ApparentDecel,
        double Sigma,
        double Tau,
        double SpeedFactor,
        double Width,
        double Height);

    // Mirrors EngineRung1PlumbingTests.RepoRoot(): resolve the repo root by walking up from
    // the test assembly's location until Traffic.sln is found.
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
