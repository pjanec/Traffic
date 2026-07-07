using System.Text.Json;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// vType defaults resolver init cross-check (CLAUDE.md rule 6: match vType/init first when a
// parity gap appears). Diffs Sim.Ingest.VTypeDefaults.Resolve's output against
// scenarios/01-single-free-flow/golden.vtype.json -- an empirical libsumo/TraCI dump of the
// resolved passenger defaults (cross-checked against the vendored source in
// VTYPE_CROSSCHECK.md; the two agree on every parameter). This is a fast-fail that separates
// init bugs from car-following algorithm bugs, run BEFORE the trajectory parity test.
public class VTypeInitCrossCheckTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "01-single-free-flow");

    [Fact]
    public void Resolve_PureDefaultsPassenger_MatchGoldenVTypeJson()
    {
        var golden = LoadGoldenVType();

        // golden.vtype.json records the PURE vClass defaults (no rou.xml overrides): resolve a
        // vType that declares only vClass="passenger", with no explicit sigma, to compare like
        // for like.
        var pureVType = new VType(Id: "probe", VClass: "passenger", Sigma: null);
        var resolved = VTypeDefaults.Resolve(pureVType);

        Assert.Equal("passenger", resolved.VClass);
        Assert.Equal(golden.CarFollowModel, resolved.CarFollowModel);
        Assert.Equal(golden.Length, resolved.Length, precision: 9);
        Assert.Equal(golden.MinGap, resolved.MinGap, precision: 9);
        Assert.Equal(golden.MaxSpeed, resolved.MaxSpeed, precision: 9);
        Assert.Equal(golden.Accel, resolved.Accel, precision: 9);
        Assert.Equal(golden.Decel, resolved.Decel, precision: 9);
        Assert.Equal(golden.EmergencyDecel, resolved.EmergencyDecel, precision: 9);
        Assert.Equal(golden.ApparentDecel, resolved.ApparentDecel, precision: 9);
        Assert.Equal(golden.Tau, resolved.Tau, precision: 9);
        Assert.Equal(golden.SpeedFactor, resolved.SpeedFactor, precision: 9);
        Assert.Equal(golden.Width, resolved.Width, precision: 9);
        Assert.Equal(golden.Height, resolved.Height, precision: 9);

        // golden.vtype.json's sigma (0.5) is the PURE default -- the resolver must reproduce it
        // exactly when no override is present, matching the source-read cross-check in
        // VTYPE_CROSSCHECK.md.
        Assert.Equal(golden.Sigma, resolved.Sigma, precision: 9);
    }

    [Fact]
    public void Resolve_ScenarioVTypePassenger_AppliesSigmaOverride()
    {
        // The rung-1 scenario's rou.xml explicitly overrides sigma="0" for determinism (see
        // VTYPE_CROSSCHECK.md "Scenario-specific overrides"). The resolved EFFECTIVE sigma must
        // be 0 here, not golden.vtype.json's pure-default 0.5 -- this is the "override applied"
        // half of the init cross-check, everything else must still match the pure defaults.
        var demand = DemandParser.Parse(Path.Combine(ScenarioDir, "rou.rou.xml"));
        var rawVType = demand.VTypesById["passenger0"];
        var resolved = VTypeDefaults.Resolve(rawVType);

        Assert.Equal(0.0, resolved.Sigma, precision: 9);

        var golden = LoadGoldenVType();
        Assert.Equal(golden.Accel, resolved.Accel, precision: 9);
        Assert.Equal(golden.Decel, resolved.Decel, precision: 9);
        Assert.Equal(golden.MaxSpeed, resolved.MaxSpeed, precision: 9);
        Assert.Equal(golden.SpeedFactor, resolved.SpeedFactor, precision: 9);
    }

    private static GoldenVType LoadGoldenVType()
    {
        var json = File.ReadAllText(Path.Combine(ScenarioDir, "golden.vtype.json"));
        using var doc = JsonDocument.Parse(json);
        var vType = doc.RootElement.GetProperty("vType");

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
