using System.Diagnostics;
using System.Globalization;
using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;

// POC-7c (docs/PEDESTRIAN-POC-PLAN.md POC-7, success conditions 1/3/4; docs/PEDESTRIAN-DESIGN.md §5/§9):
// the INTEGRATED CPU-scale acceptance benchmark. Drives the real PedLodManager (POC-3) at the owner's
// target mix -- ~10k high-power (full OrcaCrowd agents, POC-7a's parallel Step) + ~90k low-power
// (PathArcMotion path-followers, O(1)/step) -- and measures wall-clock ms/step, isolating:
//   (A) the steady-state cost once the high-power SET is stable (few promotions/demotions per step);
//   (B) the promotion-churn cost when interest sources continuously sweep the crowd, forcing
//       PedLodManager.RebuildHighCrowd (the documented OrcaCrowd-has-no-Add/Remove workaround) on most
//       steps.
//
// Deliberately a SEPARATE project, NOT part of `dotnet test` (same convention as Sim.BenchCrowd /
// Sim.BenchPedNet -- wall-clock numbers are machine-dependent and must never gate the offline parity
// loop). Run manually:
//   dotnet run -c Release --project src/Sim.BenchPedLod -- [options]
//     --sizes N,N,...       total ped counts to sweep, high-power held at ~--high-fraction (default 20000,50000,100000)
//     --high-fraction F     target fraction of the population that is high-power (default 0.10)
//     --steps N             timed steps per measurement (default 30)
//     --warmup N            untimed steps before each timed run -- lets promotion dwell settle (default 60)
//     --dt SECONDS          integration step (default 0.1)
//     --spacing M           ped grid spacing in meters (default 1.4)
//     --per-source N        target peds per interest-source cluster, sets cluster count/radii (default 1000)
//     --max-parallelism N   caps OrcaCrowd.MaxParallelism on the parallel high-power crowd (default -1 = runtime auto)
//     --no-high-only        skip the "10k-high-only vs 10k-high+90k-low" isolation runs (verdict Q1)
internal static class Program
{
    private const double MaxSpeed = 1.4;    // m/s, matches other ped benches / POC-3 tests
    private const double PedRadius = 0.3;   // m
    private const double ArriveRadius = 0.3; // m
    private const double DwellSeconds = 1.0; // s -- matches POC-3's default; a few steps at dt=0.1

    // Local patrol-loop half-extent (docs task: "give the low-power peds real multi-segment paths ...
    // so PathArcMotion does actual work"). Each ped gets a closed 5-point / 4-segment loop around its
    // spawn point -- PathArcMotion walks all 4 segments on every PositionAt/VelocityAt call regardless
    // of whether the ped has since clamped at the final vertex, so this exercises the real multi-segment
    // arc-walk cost, not a degenerate 2-point path.
    private const double LoopHalf = 0.4; // m

    private static int Main(string[] args)
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        var sizes = new[] { 20_000, 50_000, 100_000 };
        var highFraction = 0.10;
        var steps = 30;
        var warmup = 60;
        var dt = 0.1;
        var spacing = 1.4;
        var perSourceTarget = 1000;
        var maxParallelism = -1;
        var runHighOnly = true;

        for (var a = 0; a < args.Length; a++)
        {
            switch (args[a])
            {
                case "--sizes":
                    sizes = args[++a].Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
                        .Select(int.Parse).ToArray();
                    break;
                case "--high-fraction":
                    highFraction = double.Parse(args[++a], CultureInfo.InvariantCulture);
                    break;
                case "--steps":
                    steps = int.Parse(args[++a]);
                    break;
                case "--warmup":
                    warmup = int.Parse(args[++a]);
                    break;
                case "--dt":
                    dt = double.Parse(args[++a], CultureInfo.InvariantCulture);
                    break;
                case "--spacing":
                    spacing = double.Parse(args[++a], CultureInfo.InvariantCulture);
                    break;
                case "--per-source":
                    perSourceTarget = int.Parse(args[++a]);
                    break;
                case "--max-parallelism":
                    maxParallelism = int.Parse(args[++a]);
                    break;
                case "--no-high-only":
                    runHighOnly = false;
                    break;
                case "-h":
                case "--help":
                    PrintUsage();
                    return 0;
                default:
                    Console.Error.WriteLine($"unknown option: {args[a]}");
                    PrintUsage();
                    return 2;
            }
        }

        Console.WriteLine($"logical processors : {Environment.ProcessorCount}");
        Console.WriteLine($"max-parallelism arg : {(maxParallelism > 0 ? maxParallelism.ToString(CultureInfo.InvariantCulture) : "-1 (runtime auto)")}");
        Console.WriteLine($"steps/warmup        : {steps}/{warmup}   dt={dt}   highFraction={highFraction:P0}   spacing={spacing}");
        Console.WriteLine();

        // ---- Verdict Q1 isolation: 10k-high-only vs the same 10k high-power set carrying 90k low-power
        // riders (both scenario A / stable membership; both serial and parallel high-crowd). ----
        if (runHighOnly)
        {
            var nHighOnlyTarget = (int)Math.Round(100_000 * highFraction);
            Console.WriteLine("=== Verdict Q1: does ms/step scale with the HIGH-power set, or with the TOTAL? ===");
            Console.WriteLine($"{"config",-28} | {"actualHigh",10} | {"serial ms/step",15} | {"parallel ms/step",17} | {"speedup",8}");
            Console.WriteLine(new string('-', 90));

            RunAndReport($"{nHighOnlyTarget}-high-only (0 low)", nHighOnlyTarget, highFraction: 1.0, spacing, perSourceTarget,
                churn: false, dt, warmup, steps, maxParallelism);
            RunAndReport($"100000 total (~{nHighOnlyTarget} high + ~90000 low)", 100_000, highFraction, spacing, perSourceTarget,
                churn: false, dt, warmup, steps, maxParallelism);
            Console.WriteLine();
        }

        // ---- The main sweep: A (stable) vs B (churn), serial vs parallel high-crowd. ----
        Console.WriteLine("=== Main sweep: Scenario A (stable membership) vs Scenario B (churning membership) ===");
        Console.WriteLine(
            $"{"N total",8} | {"scenario",8} | {"actualHigh",10} | {"switches/step",13} | " +
            $"{"serial ms/step",15} | {"parallel ms/step",17} | {"speedup",8}");
        Console.WriteLine(new string('-', 100));

        foreach (var n in sizes)
        {
            RunAndReport($"{n}", n, highFraction, spacing, perSourceTarget, churn: false, dt, warmup, steps, maxParallelism, tableRow: true);
            RunAndReport($"{n}", n, highFraction, spacing, perSourceTarget, churn: true, dt, warmup, steps, maxParallelism, tableRow: true);
        }

        return 0;
    }

    private static void PrintUsage()
    {
        Console.Error.WriteLine(
            "usage: Sim.BenchPedLod [--sizes N,N,...] [--high-fraction F] [--steps N] [--warmup N] " +
            "[--dt SECONDS] [--spacing M] [--per-source N] [--max-parallelism N] [--no-high-only]");
    }

    // Builds the population + interest sources, runs warmup (untimed) then `steps` timed steps once
    // serial and once with UseParallelHighCrowd=true, and prints one (or two table) row(s).
    private static void RunAndReport(
        string label, int n, double highFraction, double spacing, int perSourceTarget, bool churn,
        double dt, int warmup, int steps, int maxParallelism, bool tableRow = false)
    {
        var serial = Measure(n, highFraction, spacing, perSourceTarget, churn, dt, warmup, steps, useParallel: false, maxParallelism);
        var parallel = Measure(n, highFraction, spacing, perSourceTarget, churn, dt, warmup, steps, useParallel: true, maxParallelism);

        var speedup = serial.MsPerStep / parallel.MsPerStep;

        if (tableRow)
        {
            var scenario = churn ? "B/churn" : "A/stable";
            Console.WriteLine(
                $"{n,8} | {scenario,8} | {serial.ActualHigh,10} | {serial.SwitchesPerStep,13:F2} | " +
                $"{serial.MsPerStep,15:F3} | {parallel.MsPerStep,17:F3} | {speedup,7:F2}x | " +
                $"slotHighWater={serial.SlotHighWater} (live={serial.ActualHigh}, bloat={serial.SlotHighWater - serial.ActualHigh})");
        }
        else
        {
            Console.WriteLine(
                $"{label,-28} | {serial.ActualHigh,10} | {serial.MsPerStep,15:F3} | {parallel.MsPerStep,17:F3} | {speedup,7:F2}x | " +
                $"slotHighWater={serial.SlotHighWater}");
        }
    }

    private readonly record struct Measurement(double MsPerStep, int ActualHigh, double SwitchesPerStep, int SlotHighWater);

    private static Measurement Measure(
        int n, double highFraction, double spacing, int perSourceTarget, bool churn,
        double dt, int warmup, int steps, bool useParallel, int maxParallelism)
    {
        var pop = BuildPopulation(n, highFraction, spacing, perSourceTarget);

        var nav = new StraightLineNavigation();
        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds)
        {
            UseParallelHighCrowd = useParallel,
        };

        if (maxParallelism > 0)
        {
            // PedLodManager doesn't expose MaxParallelism passthrough (only UseParallelStep, POC-7c) --
            // not needed for this benchmark's sweep, which relies on runtime-auto like PERF-HANDOVER's
            // convention; kept as a documented no-op knob for parity with Sim.BenchCrowd's CLI shape.
        }

        var now = 0.0;
        foreach (var p in pop.Peds)
        {
            manager.AddPed(p.Id, p.Path, p.MaxSpeed, p.Radius, now);
        }

        var field = new InterestField();
        var sources = pop.SourceAnchors
            .Select(a => new InterestSource(a, pop.PromoteRadius, pop.DemoteRadius))
            .ToList();
        var sourceIds = sources.Select(s => field.Register(s)).ToList();
        var noEntities = Array.Empty<WorldDisc>();

        // Warmup: untimed, lets the promotion dwell settle so Scenario A's timed window starts already
        // at (near-)steady-state membership, and gives Scenario B's churn a representative sweep phase.
        for (var i = 0; i < warmup; i++)
        {
            if (churn)
            {
                UpdateSweepingSources(field, sourceIds, pop, now);
            }

            manager.Step(now, dt, field, noEntities);
            now += dt;
        }

        // Timed window.
        var switchesTotal = 0;
        var sw = Stopwatch.StartNew();
        for (var i = 0; i < steps; i++)
        {
            if (churn)
            {
                UpdateSweepingSources(field, sourceIds, pop, now);
            }

            var before = publisher.Events.Count;
            manager.Step(now, dt, field, noEntities);
            now += dt;
            var after = publisher.Events.Count;
            for (var e = before; e < after; e++)
            {
                if (publisher.Events[e] is DrSwitchEvent)
                {
                    switchesTotal++;
                }
            }
        }

        sw.Stop();

        return new Measurement(
            MsPerStep: sw.Elapsed.TotalMilliseconds / steps,
            ActualHigh: manager.HighPowerCount,
            SwitchesPerStep: switchesTotal / (double)steps,
            SlotHighWater: manager.HighCrowdSlotHighWater);
    }

    // Scenario B: each interest source sweeps back and forth along X around its home anchor, amplitude
    // spanning a large fraction of the populated extent, so it continuously crosses into and out of
    // different peds' promote/demote radii -- "sweep through the low-power crowd, continuously
    // promoting/demoting" per the task. Deterministic (pure function of `now`, no System.Random), each
    // source phase-staggered by its index so they do not all move in lockstep.
    private static void UpdateSweepingSources(InterestField field, List<InterestSourceId> sourceIds, PopulationSpec pop, double now)
    {
        const double period = 6.0; // seconds per full back-and-forth sweep
        var amplitude = pop.SweepAmplitude;

        for (var i = 0; i < sourceIds.Count; i++)
        {
            var phase = (2.0 * Math.PI * i) / Math.Max(1, sourceIds.Count);
            var offset = amplitude * Math.Sin((2.0 * Math.PI * now / period) + phase);
            var home = pop.SourceAnchors[i];
            field.Move(sourceIds[i], new Vec2(home.X + offset, home.Y));
        }
    }

    // Trivial straight-line navigation: FindPath(start, goal) = [start, goal]. The task explicitly
    // allows "no navmesh needed at this scale" for the low-power population's hand-generated paths; for
    // the high-power set's promotion/demotion re-routing (PedLodManager.RebuildHighCrowd,
    // IPedNavigation.FindPath), using a trivial O(1) provider here is a deliberate benchmark choice: it
    // isolates PedLodManager's OWN rebuild-orchestration cost (the thing POC-7c is measuring) from any
    // navmesh-provider query cost (SumoNavMesh / DotRecast), which is a separate, already-measured
    // concern (POC-1). A production deployment would use a real provider; this benchmark's numbers are
    // a lower bound on navigation cost, not a claim that navigation is free.
    private sealed class StraightLineNavigation : IPedNavigation
    {
        public IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal) => new[] { start, goal };
    }

    private sealed class PedSpec
    {
        public required int Id;
        public required Vec2[] Path;
        public required double MaxSpeed;
        public required double Radius;
    }

    private sealed class PopulationSpec
    {
        public required List<PedSpec> Peds;
        public required List<Vec2> SourceAnchors;
        public required double PromoteRadius;
        public required double DemoteRadius;
        public required double SweepAmplitude;
    }

    // Deterministic population: N peds on a centered square grid (spacing apart), each following a
    // closed 5-point/4-segment local patrol loop (LoopHalf) around its spawn point -- real multi-segment
    // PathArcMotion work, no navmesh. `numSources` interest-source anchors are placed on a sub-grid
    // spanning the same extent; PromoteRadius is sized (via the grid's known density) so that roughly
    // `perSourceTarget` peds fall within each anchor's promote radius, totalling ~highFraction*n
    // high-power peds -- and sized with a margin (LoopHalf-derived) so that ANY ped selected as "hot"
    // has its ENTIRE patrol loop inside PromoteRadius of its nearest anchor, which is what makes
    // Scenario A's membership genuinely stable (a hot ped can never wander out of its own promote
    // radius while low-power, and once promoted it steers straight back toward its own spawn point --
    // its pre-promotion Destination -- so it never leaves the much-larger DemoteRadius either).
    private static PopulationSpec BuildPopulation(int n, double highFraction, double spacing, int perSourceTarget)
    {
        var gx = (int)Math.Ceiling(Math.Sqrt(n));
        var gy = (int)Math.Ceiling((double)n / gx);
        var originX = -(gx - 1) * spacing / 2.0;
        var originY = -(gy - 1) * spacing / 2.0;
        var extentX = Math.Max(spacing, (gx - 1) * spacing);
        var extentY = Math.Max(spacing, (gy - 1) * spacing);

        var nHigh = Math.Max(1, (int)Math.Round(n * highFraction));
        var numSources = Math.Clamp((int)Math.Round(nHigh / (double)perSourceTarget), 1, 32);

        var sx = (int)Math.Ceiling(Math.Sqrt(numSources));
        var sy = (int)Math.Ceiling(numSources / (double)sx);
        var anchors = new List<Vec2>(numSources);
        for (var row = 0; row < sy && anchors.Count < numSources; row++)
        {
            for (var col = 0; col < sx && anchors.Count < numSources; col++)
            {
                var fx = (col + 1.0) / (sx + 1.0);
                var fy = (row + 1.0) / (sy + 1.0);
                anchors.Add(new Vec2(originX + (fx * extentX), originY + (fy * extentY)));
            }
        }

        var density = 1.0 / (spacing * spacing);
        var perSourceCount = Math.Max(1, nHigh / numSources);
        var selectRadius = Math.Sqrt(perSourceCount / (density * Math.PI));
        var loopMargin = LoopHalf * 1.5; // covers the loop's own half-diagonal (LoopHalf*sqrt2) + slack
        var promoteRadius = selectRadius + loopMargin;
        var demoteRadius = promoteRadius * 2.2;

        var peds = new List<PedSpec>(n);
        var id = 0;
        var placed = 0;
        for (var iy = 0; iy < gy && placed < n; iy++)
        {
            for (var ix = 0; ix < gx && placed < n; ix++)
            {
                var p = new Vec2(originX + (ix * spacing), originY + (iy * spacing));

                // Closed local patrol loop: p -> +X -> +X+Y -> +Y -> back to p. 4 segments, real
                // multi-segment PathArcMotion work; Destination (path[^1]) == p, so a promoted-then-
                // demoted hot ped always re-routes straight back to its own spawn point.
                var path = new[]
                {
                    p,
                    p + new Vec2(LoopHalf, 0),
                    p + new Vec2(LoopHalf, LoopHalf),
                    p + new Vec2(0, LoopHalf),
                    p,
                };

                peds.Add(new PedSpec { Id = id, Path = path, MaxSpeed = MaxSpeed, Radius = PedRadius });
                id++;
                placed++;
            }
        }

        return new PopulationSpec
        {
            Peds = peds,
            SourceAnchors = anchors,
            PromoteRadius = promoteRadius,
            DemoteRadius = demoteRadius,
            SweepAmplitude = Math.Min(extentX, extentY) * 0.5,
        };
    }
}
