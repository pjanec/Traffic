using System.Text.Json;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// Rung 11 -- parameter-extraction cross-check pass. A SINGLE data-driven test that resolves the
// C# vType defaults (VTypeDefaults) for every vType in every scenario and diffs them against that
// scenario's committed golden.vtype.json (the empirical libsumo/TraCI dump of what SUMO actually
// resolved). This is the "fast fail" the ladder called for: a vType-init bug (wrong accel/decel/
// tau/minGap/...) shows up here directly, as a one-line per-(scenario,vType) failure, instead of
// as trajectory drift dozens of steps into a parity test (DESIGN.md "two kinds of ground truth").
//
// Shape: scenarios 02+ commit golden.vtype.json keyed by vType id under "vTypes" (dumped via
// scripts/dump-scenario-vtypes.py, so per-scenario overrides like rung 4's leader maxSpeed=5 and
// the sigma=0 determinism override are reflected). Rung 1's golden.vtype.json is the PURE
// vClass-default reference under a single "vType" key (validated once in VTYPE_CROSSCHECK.md);
// its pure-defaults + override resolution stays covered by the dedicated VTypeInitCrossCheckTests.
// This consolidated pass therefore iterates the "vTypes"-shaped scenarios and subsumes the former
// per-scenario Rung{4,5,6,7,8a,8b,9a,10}InitCrossCheckTests.
public class ParameterCrossCheckTests
{
    private static readonly string ScenariosDir = Path.Combine(RepoRoot(), "scenarios");

    public static IEnumerable<object[]> Cases()
    {
        foreach (var dir in Directory.GetDirectories(ScenariosDir).OrderBy(d => d, StringComparer.Ordinal))
        {
            var vtypeJson = Path.Combine(dir, "golden.vtype.json");
            if (!File.Exists(vtypeJson))
            {
                continue;
            }

            using var doc = JsonDocument.Parse(File.ReadAllText(vtypeJson));
            if (!doc.RootElement.TryGetProperty("vTypes", out var vtypes))
            {
                // Single-"vType" pure-defaults reference (rung 1) -- covered by
                // VTypeInitCrossCheckTests, not this per-scenario-resolution pass.
                continue;
            }

            foreach (var entry in vtypes.EnumerateObject())
            {
                yield return new object[] { Path.GetFileName(dir), entry.Name };
            }
        }
    }

    [Theory]
    [MemberData(nameof(Cases))]
    public void ResolvedVType_MatchesGoldenVTypeJson(string scenario, string vTypeId)
    {
        var scenarioDir = Path.Combine(ScenariosDir, scenario);

        var demand = DemandParser.Parse(Path.Combine(scenarioDir, "rou.rou.xml"));
        Assert.True(
            demand.VTypesById.ContainsKey(vTypeId),
            $"[{scenario}] golden.vtype.json declares vType '{vTypeId}' but rou.rou.xml has no such <vType>.");

        var resolved = VTypeDefaults.Resolve(demand.VTypesById[vTypeId]);
        var golden = LoadGoldenVType(scenarioDir, vTypeId);

        void Check(string attr, double expected, double actual) =>
            Assert.True(
                Math.Abs(expected - actual) < 1e-9,
                $"[{scenario}/{vTypeId}] {attr}: golden={expected} resolved={actual}");

        Assert.Equal(golden.VClass, resolved.VClass);
        Assert.Equal(golden.CarFollowModel, resolved.CarFollowModel);
        Check("length", golden.Length, resolved.Length);
        Check("minGap", golden.MinGap, resolved.MinGap);
        Check("maxSpeed", golden.MaxSpeed, resolved.MaxSpeed);
        Check("accel", golden.Accel, resolved.Accel);
        Check("decel", golden.Decel, resolved.Decel);
        Check("emergencyDecel", golden.EmergencyDecel, resolved.EmergencyDecel);
        Check("apparentDecel", golden.ApparentDecel, resolved.ApparentDecel);
        Check("sigma", golden.Sigma, resolved.Sigma);
        Check("tau", golden.Tau, resolved.Tau);
        Check("speedFactor", golden.SpeedFactor, resolved.SpeedFactor);
        Check("width", golden.Width, resolved.Width);
        Check("height", golden.Height, resolved.Height);
    }

    private static GoldenVType LoadGoldenVType(string scenarioDir, string vTypeId)
    {
        var json = File.ReadAllText(Path.Combine(scenarioDir, "golden.vtype.json"));
        using var doc = JsonDocument.Parse(json);
        var vType = doc.RootElement.GetProperty("vTypes").GetProperty(vTypeId);

        return new GoldenVType(
            VClass: vType.GetProperty("vClass").GetString()!,
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
        string VClass,
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
