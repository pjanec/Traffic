using System.Diagnostics;
using System.Globalization;
using Sim.Core;
using Sim.Core.Mixed;
using Sim.Core.Orca;
using Sim.Evac;

// PANIC-EVAC-PHASE5-TASKS.md T4.2 (design §4/§6): the Tier-1 cost-profile deliverable. Runs the
// organic-town evac demo (EvacOrganicScenario -- the same fixture EvacOrganicDemoTests and
// SceneGen.BuildEvacOrganic use: scenarios/_bench/city-organic-L2, incident at junction 415, auto-
// track working region) with EvacDirector's opt-in profiler turned on, and reports a per-phase
// wall-time breakdown -- fear update, disc feeds, pedestrian step, pusher step, engine.Step (the
// parity core, context only), and "other" -- so the Tier-2 optimization list (design §6 candidates:
// FearField grid, spatial disc feeds, OrcaCrowd.UseSpatialHash, etc.) targets the MEASURED dominant
// hotspot rather than a guessed one.
//
// NOT part of `dotnet test` -- a deliberate CLI utility (like Sim.Bench / Sim.BenchCity / Sim.Viz).
// Never touches the parity engine's committed inputs/goldens; EvacDirector's profiler is a pure
// opt-in observability seam (null unless EnableProfiling() is called), so running this tool has zero
// effect on any other demo/test's behaviour or the determinism hash.
//
// PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2a/§2b/§5(1) / TASKS T2.3: `--microbench` is a SEPARATE mode
// (same exe, gated by an arg so the default T4.2 profile run is unaffected) measuring the reason for
// the Tier-2 spatial-hash work directly: for each crowd solver (MixedTrafficCrowd, OrcaCrowd) and a
// few synthetic heavy loads (N ~= 250/1000/2000 agents in a bounded region, goals mirrored across it
// so they genuinely interact), time Step() brute-force vs grid and report the speedup. Not a
// determinism gate -- a measurement (the bit-identity guarantee is proven separately by
// MixedTrafficSpatialHashTests / OrcaSpatialHashTests).
internal static class Program
{
    private const int Ticks = 300;

    private static int Main(string[] args)
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        if (Array.IndexOf(args, "--microbench") >= 0)
        {
            return RunMicrobench();
        }

        var repoRoot = RepoRoot();
        var (engine, director) = EvacOrganicScenario.Build(repoRoot);
        director.EnableProfiling();

        var peakActive = 0;
        var everActive = new HashSet<VehicleHandle>();

        var sw = Stopwatch.StartNew();
        for (var step = 0; step < Ticks; step++)
        {
            director.Tick();

            var handles = engine.VehicleHandles;
            peakActive = Math.Max(peakActive, handles.Length);
            foreach (var h in handles)
            {
                everActive.Add(h);
            }
        }

        sw.Stop();

        var profile = director.Profile;
        var totalMs = sw.Elapsed.TotalMilliseconds;

        var fearMs = profile.FearUpdate.TotalMilliseconds;
        var discFeedsMs = profile.DiscFeeds.TotalMilliseconds;
        var pedStepMs = profile.PedestrianStep.TotalMilliseconds;
        var pusherStepMs = profile.PusherStep.TotalMilliseconds;
        var engineStepMs = profile.EngineStep.TotalMilliseconds;
        var accountedMs = fearMs + discFeedsMs + pedStepMs + pusherStepMs + engineStepMs;
        var otherMs = Math.Max(0.0, profile.TotalTick.TotalMilliseconds - accountedMs);

        Console.WriteLine("=== T4.2 evac cost profile: organic town (EvacOrganicScenario) ===");
        Console.WriteLine($"scenario           : scenarios/_bench/city-organic-L2 (274 junctions, 1186 edges, 618 trips)");
        Console.WriteLine($"ticks              : {Ticks}  (stepLength=1.0s)");
        Console.WriteLine($"peak concurrent    : {peakActive}  (vehicles active in the SAME tick, parity-engine-wide)");
        Console.WriteLine($"ever active        : {everActive.Count}  (distinct vehicles seen over the whole run)");
        Console.WriteLine($"panicked           : {director.PanickedCount}");
        Console.WriteLine($"converted          : {director.ConvertedCount}");
        Console.WriteLine($"pedestrians        : {director.PedestrianCount}");
        Console.WriteLine();
        Console.WriteLine($"total generation wall time : {totalMs:F1} ms  ({sw.Elapsed.TotalSeconds:F3} s)");
        Console.WriteLine();
        Console.WriteLine("per-phase breakdown (of EvacDirector.Tick() wall time):");

        void PrintPhase(string name, double ms) =>
            Console.WriteLine($"  {name,-18} {ms,9:F1} ms   {(totalMs > 0 ? ms / totalMs : 0.0),6:P1}");

        PrintPhase("fear update", fearMs);
        PrintPhase("disc feeds", discFeedsMs);
        PrintPhase("pedestrian step", pedStepMs);
        PrintPhase("pusher step", pusherStepMs);
        PrintPhase("engine.Step", engineStepMs);
        PrintPhase("other", otherMs);

        // The dominant EVAC hotspot -- deliberately excludes engine.Step (the parity core; it is
        // reported as context, not a Tier-2 optimization candidate per design §6) and "other" (not
        // one specific named phase to optimize). This is the input that scopes the Tier-2 task list.
        var evacPhases = new (string Name, double Ms)[]
        {
            ("fear update", fearMs),
            ("disc feeds", discFeedsMs),
            ("pedestrian step", pedStepMs),
            ("pusher step", pusherStepMs),
        };
        var dominant = evacPhases.OrderByDescending(p => p.Ms).First();

        Console.WriteLine();
        Console.WriteLine(
            $"DOMINANT EVAC HOTSPOT: {dominant.Name}  ({dominant.Ms:F1} ms, " +
            $"{(totalMs > 0 ? dominant.Ms / totalMs : 0.0):P1} of total tick time) " +
            "-- this is what Tier 2 should optimize first.");

        return 0;
    }

    // T2.3: synthetic heavy-load microbenchmark for the two Tier-2 spatial-hash solvers. For each
    // solver and each N, builds two crowds -- brute-force and grid (UseSpatialHash) -- placed
    // identically (a square grid layout at a FIXED per-agent spacing, so the region spans sqrt(N),
    // keeping local density -- and hence each agent's REAL neighbour count -- roughly constant across
    // N; only the brute-force scan's "how many agents do I have to LOOK AT before rejecting them as
    // out of range" grows with N, which is exactly the cost the spatial hash targets). Every agent's
    // goal is mirrored to the opposite side of the region so they walk toward and past each other for
    // the whole run (genuine sustained interaction, not a one-off pass). Each configuration is JIT-
    // warmed on a small throwaway crowd first, then timed over `repeats` independent builds and the
    // MEDIAN wall time is reported, to keep the numbers honest against JIT tiering / GC noise.
    private static int RunMicrobench()
    {
        Console.WriteLine("=== T2.3 heavy-load micro-benchmark: MixedTrafficCrowd / OrcaCrowd, brute vs grid ===");
        Console.WriteLine();

        var counts = new[] { 250, 1000, 2000 };
        const int steps = 60;
        const int repeats = 5;
        const double dt = 0.1;

        Console.WriteLine($"{"solver",-18} {"N",6} {"brute ms",10} {"grid ms",10} {"speedup",9}");

        foreach (var n in counts)
        {
            var (bruteMs, gridMs) = BenchmarkOrca(n, steps, repeats, dt);
            PrintRow("OrcaCrowd", n, bruteMs, gridMs);
        }

        Console.WriteLine();

        foreach (var n in counts)
        {
            var (bruteMs, gridMs) = BenchmarkMixed(n, steps, repeats, dt);
            PrintRow("MixedTrafficCrowd", n, bruteMs, gridMs);
        }

        return 0;
    }

    private static void PrintRow(string solver, int n, double bruteMs, double gridMs)
    {
        var speedup = gridMs > 0 ? bruteMs / gridMs : double.PositiveInfinity;
        Console.WriteLine($"{solver,-18} {n,6} {bruteMs,10:F1} {gridMs,10:F1} {speedup,8:F2}x");
    }

    // Square grid layout at a FIXED per-agent spacing: the region side grows as spacing*sqrt(N), so
    // local density (and each agent's real in-range neighbour count) stays roughly constant as N grows
    // -- isolating the ONE thing that genuinely scales with N: the brute-force scan's per-agent "look
    // at every other agent" cost. Goal is the point mirrored through the region's centre, so every
    // agent walks toward (and past) roughly-opposite agents for the whole run.
    private static (Vec2 Pos, Vec2 Goal)[] GridLayout(int n, double spacing)
    {
        var side = (int)Math.Ceiling(Math.Sqrt(n));
        var centre = (side - 1) * spacing * 0.5;
        var layout = new (Vec2 Pos, Vec2 Goal)[n];
        var k = 0;
        for (var gy = 0; gy < side && k < n; gy++)
        {
            for (var gx = 0; gx < side && k < n; gx++)
            {
                var p = new Vec2(gx * spacing - centre, gy * spacing - centre);
                layout[k] = (p, -p);
                k++;
            }
        }

        return layout;
    }

    // Median wall time (ms) of `repeats` independent timed runs of `steps` Step() calls, each run on a
    // FRESH crowd from `build` (so no cross-run state leaks in), after a JIT/tiering warmup pass on a
    // small throwaway crowd built the same way. Median (not mean/min) damps outliers from GC/OS jitter
    // without hand-picking the most favourable sample.
    private static double MedianTimedMs<TCrowd>(
        Func<int, TCrowd> build, Action<TCrowd, double> step, int n, int steps, int repeats, double dt)
    {
        // Warmup: run the SAME code path (same build closure, small N) enough times to clear JIT
        // tiering before any timed sample, on a crowd that is discarded afterwards.
        var warm = build(Math.Min(n, 24));
        for (var s = 0; s < 20; s++)
        {
            step(warm, dt);
        }

        var samples = new double[repeats];
        for (var r = 0; r < repeats; r++)
        {
            var crowd = build(n);
            var sw = Stopwatch.StartNew();
            for (var s = 0; s < steps; s++)
            {
                step(crowd, dt);
            }

            sw.Stop();
            samples[r] = sw.Elapsed.TotalMilliseconds;
        }

        Array.Sort(samples);
        return samples[samples.Length / 2];
    }

    private static (double BruteMs, double GridMs) BenchmarkOrca(int n, int steps, int repeats, double dt)
    {
        OrcaCrowd Build(int count, bool useGrid)
        {
            var crowd = new OrcaCrowd(count) { MaxNeighbours = 8, SymmetryBreak = 0.05, UseSpatialHash = useGrid };
            foreach (var (pos, goal) in GridLayout(count, spacing: 6.0))
            {
                crowd.Add(pos, radius: 0.3, maxSpeed: 1.4, goal);
            }

            return crowd;
        }

        var bruteMs = MedianTimedMs(c => Build(c, useGrid: false), (c, dt) => c.Step(dt), n, steps, repeats, dt);
        var gridMs = MedianTimedMs(c => Build(c, useGrid: true), (c, dt) => c.Step(dt), n, steps, repeats, dt);
        return (bruteMs, gridMs);
    }

    private static (double BruteMs, double GridMs) BenchmarkMixed(int n, int steps, int repeats, double dt)
    {
        MixedTrafficCrowd Build(int count, bool useGrid)
        {
            var crowd = new MixedTrafficCrowd(count)
            {
                MaxNeighbours = 12,
                SymmetryBreak = 0.05,
                UseSpatialHash = useGrid,
            };
            foreach (var (pos, goal) in GridLayout(count, spacing: 25.0))
            {
                crowd.Add(VehicleClass.Car, pos, goal);
            }

            return crowd;
        }

        var bruteMs = MedianTimedMs(c => Build(c, useGrid: false), (c, dt) => c.Step(dt), n, steps, repeats, dt);
        var gridMs = MedianTimedMs(c => Build(c, useGrid: true), (c, dt) => c.Step(dt), n, steps, repeats, dt);
        return (bruteMs, gridMs);
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above the exe).");
    }
}
