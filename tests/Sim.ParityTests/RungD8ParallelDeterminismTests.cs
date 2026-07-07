using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung D8 (FastDataPlane ECS readiness -- parallelize the Simulation phase): PlanMovements'
// per-vehicle plan step reads only start-of-step state (the frozen pre-move LaneNeighborQuery
// snapshot, the immutable NetworkModel, and each vehicle's own + foes' START-OF-STEP
// Kinematics/lane/vType) and writes only its own MoveIntent -- no shared-state write, no lock,
// no cross-entity write (see Engine.UseParallelPlan's own header comment for the full argument).
// That means iterating vehicles concurrently (Engine.UseParallelPlan = true) MUST produce a
// byte-identical trajectory to the existing sequential plan, regardless of thread scheduling --
// this test is the proof, reusing RungD1BenchmarkDeterminismTests' dense highway-dense scenario
// and FNV hash approach so it exercises real lane-change/car-following/junction contention at
// realistic concurrency (peak >= 50 concurrent vehicles), not a near-empty road.
public class RungD8ParallelDeterminismTests
{
    private const int Steps = 120;

    [Fact]
    public void DenseScenario_SequentialAndParallelPlan_AreByteIdentical()
    {
        var (hashSequential, peakSequential) = RunAndHash(useParallelPlan: false);
        var (hashParallel, peakParallel) = RunAndHash(useParallelPlan: true);

        // Sanity: the workload is actually dense (many concurrent vehicles) -- otherwise a
        // "parallel == sequential" match would be a hollow guarantee over an almost-empty road
        // where the plan/execute contract's ordering rules are never actually exercised.
        Assert.True(peakSequential >= 50, $"benchmark scenario not dense enough: peak concurrent = {peakSequential}");
        Assert.Equal(peakSequential, peakParallel);
        Assert.Equal(hashSequential, hashParallel);
    }

    private static (ulong Hash, int PeakConcurrent) RunAndHash(bool useParallelPlan)
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "_bench", "highway-dense");
        var engine = new Engine { UseParallelPlan = useParallelPlan };
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
