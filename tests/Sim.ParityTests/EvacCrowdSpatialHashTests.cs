using Sim.Evac;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2b / TASKS T2.2: EvacConfig.UseCrowdSpatialHash wires the
// already-proven-bit-identical OrcaCrowd/MixedTrafficCrowd spatial hashes into the evac layer's
// pedestrian crowd (_peds) AND pusher crowd (_mover, via VehicleMover.UseSpatialHash, T2.1). The hard
// requirement: at the demo level, flag off vs flag on must produce an IDENTICAL signature (grid ==
// brute), mirroring EvacOrganicDemoTests.ContainmentAndDeterminism's signature exactly.
public class EvacCrowdSpatialHashTests
{
    private readonly ITestOutputHelper _out;
    public EvacCrowdSpatialHashTests(ITestOutputHelper output) => _out = output;

    private static readonly string TestRepoRoot = RepoRoot();

    [Fact]
    public void SpatialHashOn_MatchesBruteForce_AtDemoLevel()
    {
        string RunAndSign(bool useSpatialHash)
        {
            var config = EvacOrganicScenario.DefaultConfig() with { UseCrowdSpatialHash = useSpatialHash };
            var (_, director) = EvacOrganicScenario.Build(TestRepoRoot, config: config);

            for (var step = 0; step < 300; step++)
            {
                director.Tick();
            }

            var sb = new System.Text.StringBuilder();
            sb.Append(director.PanickedCount).Append('|')
              .Append(director.ConvertedCount).Append('|')
              .Append(director.OrcaPushCount).Append('|')
              .Append(director.PedestrianCount).Append(';');

            for (var i = 0; i < director.PedestrianCount; i++)
            {
                var p = director.PedestrianPosition(i);
                sb.Append(p.X.ToString("R")).Append(',').Append(p.Y.ToString("R")).Append(';');
            }

            return sb.ToString();
        }

        var sigOff = RunAndSign(useSpatialHash: false);
        var sigOn = RunAndSign(useSpatialHash: true);

        _out.WriteLine($"sigOff={sigOff}");
        _out.WriteLine($"sigOn ={sigOn}");
        Assert.Equal(sigOff, sigOn);
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
