using System.Diagnostics;
using Sim.Core;

// D1 benchmark harness (NOT part of `dotnet test`): loads the dense highway scenario, runs the
// engine, and reports the baseline the Group-D ECS refactor improves against -- steps/sec,
// heap allocations per step, GC collections, peak concurrent vehicles, and a determinism hash
// (the ECS/parallel invariant). It is external tooling, so Stopwatch/GC introspection here is
// fine (FDP's no-Stopwatch rule is about *simulation* logic, not a benchmark driver).
//
// Run: dotnet run -c Release --project src/Sim.Bench [steps]
internal static class Program
{
    private static int Main(string[] args)
    {
        var steps = args.Length > 0 ? int.Parse(args[0]) : 500;
        var root = RepoRoot();
        var dir = Path.Combine(root, "scenarios", "_bench", "highway-dense");
        var net = Path.Combine(dir, "net.net.xml");
        var rou = Path.Combine(dir, "rou.rou.xml");
        var cfg = Path.Combine(dir, "config.sumocfg");

        // Warm up: JIT the whole step loop on a short run (not measured).
        RunOnce(net, rou, cfg, Math.Min(20, steps));

        // Determinism: two independent runs must produce identical trajectories (no RNG / no
        // wallclock in sim logic -> byte-identical). This is the invariant the parallel/FDP work
        // relies on; measuring it here makes any future regression loud.
        var hashA = TrajectoryHash(RunOnce(net, rou, cfg, steps, out var trajA));
        var hashB = TrajectoryHash(RunOnce(net, rou, cfg, steps, out _));
        var deterministic = hashA == hashB;

        // Measured run: wall time + allocations + GC.
        var gc0 = GC.CollectionCount(0);
        var gc1 = GC.CollectionCount(1);
        var gc2 = GC.CollectionCount(2);
        var allocBefore = GC.GetTotalAllocatedBytes(precise: true);
        var sw = Stopwatch.StartNew();
        var traj = RunOnce(net, rou, cfg, steps, out _);
        sw.Stop();
        var allocBytes = GC.GetTotalAllocatedBytes(precise: true) - allocBefore;
        gc0 = GC.CollectionCount(0) - gc0;
        gc1 = GC.CollectionCount(1) - gc1;
        gc2 = GC.CollectionCount(2) - gc2;

        // Peak concurrent vehicles = max over times of the count of vehicles present.
        var perTime = new Dictionary<double, int>();
        var totalPoints = 0L;
        foreach (var p in traj.AllPoints)
        {
            perTime[p.Time] = perTime.TryGetValue(p.Time, out var c) ? c + 1 : 1;
            totalPoints++;
        }
        var peak = perTime.Count == 0 ? 0 : perTime.Values.Max();

        var stepsPerSec = steps / sw.Elapsed.TotalSeconds;
        var msPerStep = sw.Elapsed.TotalMilliseconds / steps;
        var allocPerStep = allocBytes / (double)steps;
        var allocPerVehStep = totalPoints == 0 ? 0 : allocBytes / (double)totalPoints;

        Console.WriteLine("=== D1 baseline: scenarios/_bench/highway-dense ===");
        Console.WriteLine($"vehicles(demand)   : {DemandCount(rou)}");
        Console.WriteLine($"peak concurrent    : {peak}");
        Console.WriteLine($"steps              : {steps}");
        Console.WriteLine($"veh-steps emitted  : {totalPoints:N0}");
        Console.WriteLine($"wall time          : {sw.Elapsed.TotalSeconds:F3} s");
        Console.WriteLine($"throughput         : {stepsPerSec:F1} steps/s  ({msPerStep:F3} ms/step)");
        Console.WriteLine($"alloc total        : {allocBytes / (1024.0 * 1024.0):F1} MiB");
        Console.WriteLine($"alloc / step       : {allocPerStep / 1024.0:F1} KiB");
        Console.WriteLine($"alloc / veh-step   : {allocPerVehStep:F1} B");
        Console.WriteLine($"GC gen0/1/2        : {gc0} / {gc1} / {gc2}");
        Console.WriteLine($"deterministic      : {deterministic}  (hashA={hashA:X16})");
        return deterministic ? 0 : 1;
    }

    private static TrajectorySet RunOnce(string net, string rou, string cfg, int steps)
        => RunOnce(net, rou, cfg, steps, out _);

    private static TrajectorySet RunOnce(string net, string rou, string cfg, int steps, out TrajectorySet traj)
    {
        var engine = new Engine();
        engine.LoadScenario(net, rou, cfg);
        traj = engine.Run(steps);
        return traj;
    }

    // Order-independent-but-stable FNV-1a over (vehicleId, time, lane, pos, speed).
    private static ulong TrajectoryHash(TrajectorySet traj)
    {
        var keys = new List<string>(traj.VehicleIds);
        keys.Sort(StringComparer.Ordinal);
        ulong h = 1469598103934665603UL;
        foreach (var id in keys)
        {
            foreach (var kv in traj.PointsFor(id))
            {
                var p = kv.Value;
                h = Mix(h, id);
                h = Mix(h, p.Time);
                h = Mix(h, p.Lane);
                h = Mix(h, p.Pos);
                h = Mix(h, p.Speed);
            }
        }
        return h;
    }

    private static ulong Mix(ulong h, string s)
    {
        foreach (var c in s) { h ^= c; h *= 1099511628211UL; }
        return h;
    }

    private static ulong Mix(ulong h, double d)
    {
        var bits = (ulong)BitConverter.DoubleToInt64Bits(d);
        h ^= bits; h *= 1099511628211UL;
        return h;
    }

    private static int DemandCount(string rouPath)
    {
        var n = 0;
        foreach (var line in File.ReadLines(rouPath))
        {
            if (line.Contains("<vehicle ")) n++;
        }
        return n;
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }
        return dir?.FullName ?? throw new InvalidOperationException("Traffic.sln not found above the bench assembly.");
    }
}
