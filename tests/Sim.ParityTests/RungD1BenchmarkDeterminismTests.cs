using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung D1 (FastDataPlane ECS readiness): the many-vehicle benchmark scenario
// (scenarios/_bench/highway-dense) is the workload the Group-D representation refactor is measured
// against (see src/Sim.Bench for the perf harness + BASELINE.md). This test guards the ONE property
// the whole ECS/parallel program depends on: the engine is DETERMINISTIC -- two independent runs of a
// dense (hundreds of concurrent vehicles) scenario produce byte-identical trajectories. No
// System.Random, no wallclock, plan reads a frozen start-of-step snapshot and writes only its own
// intent, so thread order can never matter (D8 parallelizes on exactly this guarantee). A regression
// here -- an accidental RNG, a shared-state write in the plan phase, a dictionary-iteration-order
// dependence -- would silently break the parallel/FDP work, so it is worth a fast offline check.
//
// Runs a SHORT slice (enough steps to build real multi-lane density with following + lane changes)
// so it stays cheap in the `dotnet test` loop; the full 500-step perf sweep lives in Sim.Bench.
public class RungD1BenchmarkDeterminismTests
{
    private const int Steps = 120;

    [Fact]
    public void DenseScenario_TwoRuns_AreByteIdentical()
    {
        var (hashA, peakA) = RunAndHash();
        var (hashB, peakB) = RunAndHash();

        // Sanity: the workload is actually dense (many concurrent vehicles), else "deterministic"
        // would be a hollow guarantee over an almost-empty road.
        Assert.True(peakA >= 50, $"benchmark scenario not dense enough: peak concurrent = {peakA}");
        Assert.Equal(peakA, peakB);
        Assert.Equal(hashA, hashB);
    }

    private static (ulong Hash, int PeakConcurrent) RunAndHash()
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "_bench", "highway-dense");
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(dir, "net.net.xml"),
            Path.Combine(dir, "rou.rou.xml"),
            Path.Combine(dir, "config.sumocfg"));

        var traj = engine.Run(Steps);

        var perTime = new Dictionary<double, int>();
        var ids = new List<string>(traj.VehicleIds);
        ids.Sort(StringComparer.Ordinal);

        ulong h = 1469598103934665603UL;
        foreach (var id in ids)
        {
            foreach (var kv in traj.PointsFor(id))
            {
                var p = kv.Value;
                perTime[p.Time] = perTime.TryGetValue(p.Time, out var c) ? c + 1 : 1;
                h = Mix(h, id);
                h = Mix(h, p.Time);
                h = Mix(h, p.Lane);
                h = Mix(h, p.Pos);
                h = Mix(h, p.Speed);
            }
        }

        var peak = perTime.Count == 0 ? 0 : perTime.Values.Max();
        return (h, peak);
    }

    private static ulong Mix(ulong h, string s)
    {
        foreach (var c in s) { h ^= c; h *= 1099511628211UL; }
        return h;
    }

    private static ulong Mix(ulong h, double d)
    {
        h ^= (ulong)BitConverter.DoubleToInt64Bits(d);
        h *= 1099511628211UL;
        return h;
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
