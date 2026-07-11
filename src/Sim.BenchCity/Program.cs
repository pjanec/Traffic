using System.Diagnostics;
using System.Globalization;
using Sim.Core;
using Sim.Harness;
using Sim.Ingest;

// VB-7 (VIZ_BENCH_TASKS.md Phase 2): runs the engine on a generated scaled-city benchmark
// scenario (see VB-5 / scripts/gen-benchmark.sh) to completion and emits:
//   - engine.tripinfo.xml / engine.summary.xml -- SUMO-schema ANALOGS of --tripinfo-output /
//     --summary-output (a subset of attributes -- exactly what Sim.Harness.TripInfoParser /
//     SummaryOutputParser read, see those classes' header comments), so VB-6's AggregateComparator
//     can compare the engine run against a SUMO reference run through ONE shared loader.
//   - engine.fcd.xml (via the VB-0 FcdWriterObserver, D9 export seam) for Sim.Viz.
//   - a metric line: wall-clock, steps/sec, RTF (sim-time / wall-time), peak concurrent
//     vehicles, peak RSS, and the stuck/gridlock count (see StuckDetector below) -- the engine's
//     teleport-off deadlock analog, since phase 1 implements no teleport.
//
// Deliberately a SEPARATE project from Sim.Bench: Sim.Bench owns the Group-D
// perf/determinism harness on the hand-built `highway-dense` scenario (steps/sec, alloc/veh-step,
// the 500-step determinism hash) -- this benchmark's NEW netgenerate-based city is a different
// scenario with a different purpose (statistical SUMO comparison, not a determinism oracle), so
// it gets its own runner rather than overloading Sim.Bench's. It DOES reuse Sim.Bench's
// measurement patterns (Stopwatch, GC/alloc introspection -- fine in a benchmark driver, not sim
// logic, per CLAUDE.md).
//
// NOT part of `dotnet test` -- a deliberate CLI utility, like Sim.Bench and Sim.Run.
//
// Usage:
//   dotnet run -c Release --project src/Sim.BenchCity -- <scenarioDir> [options]
//     --steps N                override step count (default: derived from config.sumocfg)
//     --fcd-out PATH            (default: <scenarioDir>/engine.fcd.xml; "" to skip)
//     --tripinfo-out PATH       (default: <scenarioDir>/engine.tripinfo.xml)
//     --summary-out PATH        (default: <scenarioDir>/engine.summary.xml)
//     --stuck-window SECONDS    consecutive near-zero-speed window to flag "stuck" (default 120)
//     --stuck-speed MPS         speed threshold below which a vehicle counts as "not progressing"
//                               (default 0.1 m/s)
//     --sumo-summary PATH       VB-6: also compare against a SUMO reference --summary-output
//     --sumo-tripinfo PATH      VB-6: also compare against a SUMO reference --tripinfo-output
//     --aggregate-tolerance PATH  VB-6 AggregateToleranceConfig JSON (required with the above two)
internal static class Program
{
    private static int Main(string[] args)
    {
        // Emit all metric numbers with invariant '.' decimals regardless of the OS locale, so the
        // "wall time : 9.231 s" line parses the same on an en-US and a cs-CZ box (which would
        // otherwise print "9,231 s" and break downstream parsers such as scripts/bench-scaling.ps1).
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;

        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            Console.Error.WriteLine(
                "usage: Sim.BenchCity <scenarioDir> [--steps N] [--fcd-out PATH] " +
                "[--tripinfo-out PATH] [--summary-out PATH] [--stuck-window SECONDS] [--stuck-speed MPS]");
            return args.Length == 0 ? 2 : 0;
        }

        var scenarioDir = args[0];
        if (!Directory.Exists(scenarioDir))
        {
            Console.Error.WriteLine($"error: scenario directory not found: {scenarioDir}");
            return 2;
        }

        int? stepsOverride = null;
        string? fcdOut = null;
        string? tripinfoOut = null;
        string? summaryOut = null;
        var stuckWindowSeconds = 120.0;
        var stuckSpeedThreshold = 0.1;
        var forceSerial = false;
        var maxParallelism = -1;
        var noFcd = false;
        var profile = false;
        var parallelExport = false;
        var fast = false;
        var fastGate = false;
        string? sumoSummaryPath = null;
        string? sumoTripinfoPath = null;
        string? aggregateTolerancePath = null;

        for (var i = 1; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--steps" when i + 1 < args.Length:
                    stepsOverride = int.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                case "--fcd-out" when i + 1 < args.Length:
                    fcdOut = args[++i];
                    break;
                case "--tripinfo-out" when i + 1 < args.Length:
                    tripinfoOut = args[++i];
                    break;
                case "--summary-out" when i + 1 < args.Length:
                    summaryOut = args[++i];
                    break;
                case "--stuck-window" when i + 1 < args.Length:
                    stuckWindowSeconds = double.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                case "--stuck-speed" when i + 1 < args.Length:
                    stuckSpeedThreshold = double.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                case "--sumo-summary" when i + 1 < args.Length:
                    sumoSummaryPath = args[++i];
                    break;
                case "--sumo-tripinfo" when i + 1 < args.Length:
                    sumoTripinfoPath = args[++i];
                    break;
                case "--aggregate-tolerance" when i + 1 < args.Length:
                    aggregateTolerancePath = args[++i];
                    break;
                case "--serial":
                    // Force the fully single-threaded path (no Parallel.For over plan/willPass/emit),
                    // for a clean serial-vs-SUMO comparison independent of core count / affinity.
                    forceSerial = true;
                    break;
                case "--max-parallelism" when i + 1 < args.Length:
                    // Cap the engine's Parallel.For degree -- sweep 1..coreCount for a scaling curve.
                    maxParallelism = int.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                case "--no-fcd":
                    // Skip FCD export without the "" empty-string arg (PowerShell drops "", which
                    // silently shifts the following flag into --fcd-out's value). Prefer this in scripts.
                    noFcd = true;
                    break;
                case "--profile":
                    // Turn on the engine's per-phase wall-time accounting and print the breakdown.
                    profile = true;
                    break;
                case "--parallel-export":
                    // Opt into the parallel Export path (off by default -- a net loss on the target
                    // box; byte-identical either way). Here to A/B the schedule on other hardware.
                    parallelExport = true;
                    break;
                case "--fast":
                    // Opt into Engine.FastMode (off by default). NOT SUMO-byte-identical; validate
                    // behavior with --fast-gate, never against the goldens.
                    fast = true;
                    break;
                case "--fast-gate":
                    // Run the deterministic AND the fast engine on this scenario and assert fast mode
                    // is BEHAVIORALLY sound: 0 gridlock, aggregate parity vs the deterministic run
                    // (arrived / mean duration / mean speed / trip-duration KS) within a tight
                    // tolerance, and no vehicle overlaps. Prints PASS/FAIL and returns.
                    fastGate = true;
                    break;
                default:
                    Console.Error.WriteLine($"error: unrecognized argument: {args[i]}");
                    return 2;
            }
        }

        var net = SingleFile(scenarioDir, "*.net.xml");
        var rou = SingleFile(scenarioDir, "*.rou.xml");
        var cfg = SingleFile(scenarioDir, "*.sumocfg");
        if (net is null || rou is null || cfg is null)
        {
            Console.Error.WriteLine(
                $"error: scenario dir must contain exactly one each of *.net.xml, *.rou.xml, " +
                $"*.sumocfg (found net={net}, rou={rou}, cfg={cfg})");
            return 2;
        }

        var config = ScenarioConfigParser.Parse(cfg);
        var steps = stepsOverride ?? (int)Math.Round((config.End - config.Begin) / config.StepLength);

        // Fast-mode behavioral gate: run deterministic + fast on this scenario and assert fast mode
        // is BEHAVIORALLY sound (0 gridlock, aggregate parity vs the deterministic run, no overlaps).
        // Self-contained -- returns before the normal single-run benchmark below.
        if (fastGate)
        {
            return RunFastGate(net, rou, cfg, steps, config, stuckWindowSeconds, stuckSpeedThreshold);
        }

        fcdOut = noFcd ? "" : (fcdOut ?? Path.Combine(scenarioDir, "engine.fcd.xml"));
        tripinfoOut ??= Path.Combine(scenarioDir, "engine.tripinfo.xml");
        summaryOut ??= Path.Combine(scenarioDir, "engine.summary.xml");

        var engine = new Engine();
        if (forceSerial)
        {
            engine.UseParallelPlan = false; // pin the serial path (ShouldParallelizePlan -> false)
        }

        if (maxParallelism > 0)
        {
            engine.MaxParallelism = maxParallelism; // cap worker threads for a scaling sweep
        }

        engine.ProfilePhases = profile; // per-phase wall-time accounting (printed below)
        engine.ParallelExport = parallelExport; // opt-in parallel Export (off = faster on target box)
        engine.FastMode = fast; // opt-in fast mode (off = deterministic/byte-identical path)

        engine.LoadScenario(net, rou, cfg);

        var process = Process.GetCurrentProcess();
        var allocBefore = GC.GetTotalAllocatedBytes(precise: true);
        var sw = Stopwatch.StartNew();

        TrajectorySet traj;
        if (!string.IsNullOrEmpty(fcdOut))
        {
            using var writer = new FcdWriterObserver(fcdOut);
            engine.AddExportObserver(writer);
            traj = engine.Run(steps);
        }
        else
        {
            traj = engine.Run(steps);
        }

        sw.Stop();
        var allocBytes = GC.GetTotalAllocatedBytes(precise: true) - allocBefore;
        process.Refresh();
        var peakRssBytes = process.PeakWorkingSet64;

        // ---- build per-vehicle trip records + per-step summary rows purely from the returned
        // TrajectorySet -- no new Sim.Core API, no touching EmitTrajectory: the D9 export seam
        // (via engine.Run's return value, same as Sim.Bench) is already a complete per-frame,
        // per-vehicle record of everything a "tripinfo"/"summary" analog needs.
        var byVehicle = new Dictionary<string, List<TrajectoryPoint>>();
        foreach (var id in traj.VehicleIds)
        {
            var points = traj.PointsFor(id).Values.OrderBy(p => p.Time).ToList();
            byVehicle[id] = points;
        }

        var finalStepTime = byVehicle.Count > 0 ? byVehicle.Values.Max(pts => pts[^1].Time) : config.End;
        // Half a step of slack: a vehicle whose last frame lands on the run's very last emitted
        // step is still in the network when the run ends (we cannot tell it apart from "arrived
        // in that exact same step" without a new Sim.Core arrival event -- see this file's header
        // comment on why that's fine for a benchmark aggregate/statistical bar, not a parity one).
        var arrivalCutoff = finalStepTime - (config.StepLength / 2.0);

        var trips = new List<TripInfoRecord>();
        var stillRunningCount = 0;
        foreach (var (id, points) in byVehicle)
        {
            if (points.Count == 0)
            {
                continue;
            }

            var depart = points[0].Time;
            var lastSeen = points[^1].Time;
            var arrived = lastSeen < arrivalCutoff;
            if (!arrived)
            {
                stillRunningCount++;
                continue;
            }

            var duration = lastSeen - depart;
            var arrivalSpeed = points[^1].Speed;
            trips.Add(new TripInfoRecord(id, depart, duration, arrivalSpeed));
        }

        trips.Sort((a, b) => a.Depart.CompareTo(b.Depart));

        var summarySteps = BuildSummarySteps(byVehicle, config);

        var stuck = StuckDetector.Detect(byVehicle, config.StepLength, stuckWindowSeconds, stuckSpeedThreshold, finalStepTime);

        WriteTripInfo(tripinfoOut, trips);
        WriteSummary(summaryOut, summarySteps);

        var peakConcurrent = summarySteps.Count > 0 ? summarySteps.Max(s => s.Running) : 0;
        var simDuration = config.End - config.Begin;
        var stepsPerSec = steps / sw.Elapsed.TotalSeconds;
        var rtf = simDuration / sw.Elapsed.TotalSeconds;

        Console.WriteLine($"=== VB-7 engine benchmark: {scenarioDir} ===");
        Console.WriteLine($"steps              : {steps}  (sim [{config.Begin}, {config.End}] @ {config.StepLength}s)");
        Console.WriteLine($"wall time          : {sw.Elapsed.TotalSeconds:F3} s");
        Console.WriteLine($"throughput         : {stepsPerSec:F1} steps/s");
        Console.WriteLine($"RTF (sim/wall)     : {rtf:F2}x");
        Console.WriteLine($"peak concurrent    : {peakConcurrent}");
        Console.WriteLine($"peak RSS           : {peakRssBytes / (1024.0 * 1024.0):F1} MiB");
        Console.WriteLine($"alloc total        : {allocBytes / (1024.0 * 1024.0):F1} MiB");
        Console.WriteLine($"vehicles departed  : {byVehicle.Count}");
        Console.WriteLine($"vehicles arrived   : {trips.Count}");
        Console.WriteLine($"vehicles running@end: {stillRunningCount}");
        Console.WriteLine($"mean trip duration : {(trips.Count > 0 ? trips.Average(t => t.Duration) : 0.0):F2} s");
        Console.WriteLine($"stuck (ever, >= {stuckWindowSeconds:F0}s @<{stuckSpeedThreshold:F2}m/s): {stuck.EverStuckCount}");
        Console.WriteLine($"stuck (still, at sim end)                : {stuck.StillStuckAtEndCount}");
        if (profile && engine.PhaseTicks.Count > 0)
        {
            var totalTicks = (double)engine.PhaseTicks.Values.Sum();
            var toMs = 1000.0 / Stopwatch.Frequency;
            Console.WriteLine("phase breakdown (share of the accounted per-step work):");
            foreach (var kv in engine.PhaseTicks.OrderByDescending(kv => kv.Value))
            {
                Console.WriteLine(
                    $"  {kv.Key,-10} {kv.Value * toMs,9:F1} ms   {kv.Value / totalTicks,6:P1}");
            }
        }

        Console.WriteLine($"wrote {tripinfoOut}");
        Console.WriteLine($"wrote {summaryOut}");
        if (!string.IsNullOrEmpty(fcdOut))
        {
            Console.WriteLine($"wrote {fcdOut}");
        }

        // VB-6/VB-8: optional in-line aggregate comparison against a SUMO reference run, so one
        // Sim.BenchCity invocation can produce the whole "engine vs SUMO" report VB-8 asks for
        // without a separate driver. Reuses the exact records just written (no re-parse needed
        // for the engine side) and TripInfoParser/SummaryOutputParser for the SUMO side.
        if (sumoSummaryPath is not null || sumoTripinfoPath is not null || aggregateTolerancePath is not null)
        {
            if (sumoSummaryPath is null || sumoTripinfoPath is null || aggregateTolerancePath is null)
            {
                Console.Error.WriteLine(
                    "error: --sumo-summary, --sumo-tripinfo, and --aggregate-tolerance must all be given together.");
                return 2;
            }

            var referenceSummary = SummaryOutputParser.Parse(sumoSummaryPath);
            var referenceTrips = TripInfoParser.Parse(sumoTripinfoPath);
            var tolerance = AggregateToleranceConfig.Load(aggregateTolerancePath);

            var comparison = AggregateComparator.Compare(referenceTrips, referenceSummary, trips, summarySteps, tolerance);

            Console.WriteLine();
            Console.WriteLine("=== VB-6 aggregate comparison: engine vs SUMO reference ===");
            PrintMetric(comparison.Arrived);
            PrintMetric(comparison.MeanDuration);
            PrintMetric(comparison.MeanSpeed);
            Console.WriteLine(
                $"distributionDistance (KS): {comparison.DistributionDistance:F4}  " +
                $"(tol={comparison.DistributionDistanceTolerance:F4}, " +
                $"{(comparison.DistributionWithinTolerance ? "PASS" : "FAIL")})");
            Console.WriteLine($"AGGREGATE COMPARISON: {(comparison.IsMatch ? "PASS" : "FAIL")}");
        }

        return 0;
    }

    private static void PrintMetric(AggregateMetricResult m) =>
        Console.WriteLine(
            $"{m.Metric,-12}: reference={m.Reference:F3} candidate={m.Candidate:F3} " +
            $"relError={m.RelError:F4} (tol={m.Tolerance:F4}, {(m.WithinTolerance ? "PASS" : "FAIL")})");

    // A same-lane, same-time longitudinal separation below this (metres) is treated as a vehicle
    // overlap/stacking. It is FAR below any vehicle length + minGap (legitimate bumper-to-bumper
    // spacing is >= ~5 m), so a violation is a genuine collision (e.g. a racy fast-mode lane change
    // dropping two cars into one gap), never legitimate close following.
    private const double OverlapMinSeparation = 1.0;

    // The automated behavioral gate for Engine.FastMode. Fast mode is NOT SUMO-byte-identical, so it
    // is validated behaviorally: it must (1) not gridlock, (2) stay within a TIGHT aggregate
    // tolerance of the deterministic run (the byte-identical-to-SUMO baseline), and (3) never overlap
    // vehicles. Deterministic and fast are each run once on the same scenario and compared.
    private static int RunFastGate(
        string net, string rou, string cfg, int steps, ScenarioConfig config,
        double stuckWindow, double stuckSpeed)
    {
        Console.WriteLine("=== FAST-MODE BEHAVIORAL GATE (deterministic baseline vs --fast) ===");

        var det = ExtractRun(net, rou, cfg, steps, config, stuckWindow, stuckSpeed, fastMode: false);
        var fast = ExtractRun(net, rou, cfg, steps, config, stuckWindow, stuckSpeed, fastMode: true);

        var detOverlaps = CountOverlaps(det.ByVehicle);
        var fastOverlaps = CountOverlaps(fast.ByVehicle);

        // Tight tolerance vs the deterministic run (NOT the loose ~0.35 SUMO bar): fast mode's
        // deterministic-tie-break scheduling should perturb the macroscopic behaviour only slightly.
        var tol = new AggregateToleranceConfig
        {
            ArrivedRelTol = 0.02,
            MeanDurationRelTol = 0.03,
            MeanSpeedRelTol = 0.03,
            DistributionDistanceTol = 0.05,
        };
        var cmp = AggregateComparator.Compare(det.Trips, det.Summary, fast.Trips, fast.Summary, tol);

        Console.WriteLine(
            $"deterministic : departed={det.ByVehicle.Count} arrived={det.Trips.Count} " +
            $"stuck(ever/end)={det.EverStuck}/{det.StillStuck} overlaps={detOverlaps}");
        Console.WriteLine(
            $"fast          : departed={fast.ByVehicle.Count} arrived={fast.Trips.Count} " +
            $"stuck(ever/end)={fast.EverStuck}/{fast.StillStuck} overlaps={fastOverlaps}");
        Console.WriteLine("--- fast vs deterministic (tight tolerance) ---");
        PrintMetric(cmp.Arrived);
        PrintMetric(cmp.MeanDuration);
        PrintMetric(cmp.MeanSpeed);
        Console.WriteLine(
            $"distributionDistance (KS): {cmp.DistributionDistance:F4}  " +
            $"(tol={cmp.DistributionDistanceTolerance:F4}, {(cmp.DistributionWithinTolerance ? "PASS" : "FAIL")})");

        // Sanity: the deterministic baseline must be gridlock-free, else the gate's premise (compare
        // fast to a KNOWN-GOOD baseline) is invalid. (A small non-zero baseline overlap count on
        // normal edges is tolerated -- the criterion below is RELATIVE, so any model-inherent
        // baseline overlaps are the yardstick, not zero.)
        if (det.EverStuck != 0 || det.StillStuck != 0)
        {
            Console.WriteLine(
                $"FAST-GATE: INVALID BASELINE (deterministic gridlocked, stuck={det.EverStuck}/{det.StillStuck}) " +
                "-- fix the scenario before trusting the gate.");
            return 3;
        }

        var gridlockOk = fast.EverStuck == 0 && fast.StillStuck == 0;
        // RELATIVE overlap criterion: fast mode must introduce NO overlaps beyond the deterministic
        // baseline (which may carry a few model-inherent ones). A racy fast-mode lane change that
        // stacks two cars shows up as fastOverlaps > detOverlaps.
        var overlapOk = fastOverlaps <= detOverlaps;
        var aggregateOk = cmp.IsMatch;
        var pass = gridlockOk && overlapOk && aggregateOk;

        Console.WriteLine($"  gridlock : {(gridlockOk ? "PASS" : "FAIL")}  (fast stuck ever/end = {fast.EverStuck}/{fast.StillStuck})");
        Console.WriteLine($"  overlaps : {(overlapOk ? "PASS" : "FAIL")}  (fast={fastOverlaps} vs baseline={detOverlaps}, normal edges only)");
        Console.WriteLine($"  aggregate: {(aggregateOk ? "PASS" : "FAIL")}");
        Console.WriteLine($"FAST-GATE: {(pass ? "PASS" : "FAIL")}");
        return pass ? 0 : 1;
    }

    // Run one engine (deterministic or fast) to completion and extract the behavioral records the
    // gate compares: per-vehicle trajectory, arrived-trip records, per-step summary, and stuck counts.
    // Mirrors Main's inline extraction (same arrival heuristic / stuck detector), factored for reuse.
    private static (List<TripInfoRecord> Trips, List<SummaryStepRecord> Summary,
        Dictionary<string, List<TrajectoryPoint>> ByVehicle, int EverStuck, int StillStuck) ExtractRun(
        string net, string rou, string cfg, int steps, ScenarioConfig config,
        double stuckWindow, double stuckSpeed, bool fastMode)
    {
        var engine = new Engine { FastMode = fastMode };
        engine.LoadScenario(net, rou, cfg);
        var traj = engine.Run(steps);

        var byVehicle = new Dictionary<string, List<TrajectoryPoint>>();
        foreach (var id in traj.VehicleIds)
        {
            byVehicle[id] = traj.PointsFor(id).Values.OrderBy(p => p.Time).ToList();
        }

        var finalStepTime = byVehicle.Count > 0 ? byVehicle.Values.Max(pts => pts[^1].Time) : config.End;
        var arrivalCutoff = finalStepTime - (config.StepLength / 2.0);

        var trips = new List<TripInfoRecord>();
        foreach (var (id, points) in byVehicle)
        {
            if (points.Count == 0)
            {
                continue;
            }

            var depart = points[0].Time;
            var lastSeen = points[^1].Time;
            if (lastSeen >= arrivalCutoff)
            {
                continue; // still running at the run's end -- not an arrival
            }

            trips.Add(new TripInfoRecord(id, depart, lastSeen - depart, points[^1].Speed));
        }

        trips.Sort((a, b) => a.Depart.CompareTo(b.Depart));
        var summary = BuildSummarySteps(byVehicle, config);
        var stuck = StuckDetector.Detect(byVehicle, config.StepLength, stuckWindow, stuckSpeed, finalStepTime);
        return (trips, summary, byVehicle, stuck.EverStuckCount, stuck.StillStuckAtEndCount);
    }

    // Count same-lane, same-time vehicle pairs separated by less than OverlapMinSeparation -- a
    // length-free overlap/collision proxy (see that constant's comment).
    private static int CountOverlaps(Dictionary<string, List<TrajectoryPoint>> byVehicle)
    {
        var byTimeLane = new Dictionary<(double Time, string Lane), List<double>>();
        foreach (var points in byVehicle.Values)
        {
            foreach (var p in points)
            {
                // Skip junction-interior (internal, ':'-prefixed) lanes: their geometry makes a
                // longitudinal (lane, pos) proximity a poor collision proxy (crossing paths, very
                // short lanes). Overlaps that matter for fast mode happen on normal edges.
                if (p.Lane.Length > 0 && p.Lane[0] == ':')
                {
                    continue;
                }

                var key = (p.Time, p.Lane);
                if (!byTimeLane.TryGetValue(key, out var list))
                {
                    list = new List<double>();
                    byTimeLane[key] = list;
                }

                list.Add(p.Pos);
            }
        }

        var overlaps = 0;
        foreach (var list in byTimeLane.Values)
        {
            if (list.Count < 2)
            {
                continue;
            }

            list.Sort();
            for (var i = 1; i < list.Count; i++)
            {
                if (list[i] - list[i - 1] < OverlapMinSeparation)
                {
                    overlaps++;
                }
            }
        }

        return overlaps;
    }

    // Buckets every observed frame by its Time, computing SUMO-summary-schema aggregates:
    // `running` = vehicles present that step, `meanSpeed` = their average speed, `arrived` =
    // cumulative count of vehicles whose LAST frame is strictly before this step (mirrors the
    // arrival heuristic above -- a vehicle vanishing from the trajectory before the run ends is,
    // in this teleport-off engine, only ever explained by a genuine arrival).
    private static List<SummaryStepRecord> BuildSummarySteps(
        Dictionary<string, List<TrajectoryPoint>> byVehicle, ScenarioConfig config)
    {
        var byTime = new SortedDictionary<double, (int Running, double SpeedSum)>();
        var arrivalTimes = new List<double>();

        foreach (var points in byVehicle.Values)
        {
            foreach (var p in points)
            {
                byTime.TryGetValue(p.Time, out var acc);
                byTime[p.Time] = (acc.Running + 1, acc.SpeedSum + p.Speed);
            }

            if (points.Count > 0)
            {
                arrivalTimes.Add(points[^1].Time);
            }
        }

        arrivalTimes.Sort();

        var steps = new List<SummaryStepRecord>();
        var arrivalIndex = 0;
        var finalStepTime = byTime.Count > 0 ? byTime.Keys.Max() : config.End;
        var arrivalCutoff = finalStepTime - (config.StepLength / 2.0);
        // Only count an arrival once its vehicle's last frame is confirmed BEFORE the run's own
        // end (the same "still running at the very last step" guard as the trip-record loop
        // above) so `arrived` in the summary analog and `vehicles arrived` in the console/tripinfo
        // stay mutually consistent.
        var confirmedArrivals = arrivalTimes.Where(t => t < arrivalCutoff).OrderBy(t => t).ToList();

        var cumulativeArrived = 0;
        foreach (var (time, acc) in byTime)
        {
            while (arrivalIndex < confirmedArrivals.Count && confirmedArrivals[arrivalIndex] <= time)
            {
                cumulativeArrived++;
                arrivalIndex++;
            }

            var meanSpeed = acc.Running > 0 ? acc.SpeedSum / acc.Running : (double?)null;
            steps.Add(new SummaryStepRecord(time, acc.Running, cumulativeArrived, meanSpeed));
        }

        return steps;
    }

    private static void WriteTripInfo(string path, IReadOnlyList<TripInfoRecord> trips)
    {
        using var w = new StreamWriter(path, append: false);
        w.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
        w.WriteLine("<tripinfos>");
        foreach (var t in trips)
        {
            w.Write("    <tripinfo id=\"");
            w.Write(Escape(t.Id));
            w.Write("\" depart=\"");
            w.Write(t.Depart.ToString("R", CultureInfo.InvariantCulture));
            w.Write("\" duration=\"");
            w.Write(t.Duration.ToString("R", CultureInfo.InvariantCulture));
            if (t.ArrivalSpeed is { } speed)
            {
                w.Write("\" arrivalSpeed=\"");
                w.Write(speed.ToString("R", CultureInfo.InvariantCulture));
            }

            w.WriteLine("\"/>");
        }

        w.WriteLine("</tripinfos>");
    }

    private static void WriteSummary(string path, IReadOnlyList<SummaryStepRecord> steps)
    {
        using var w = new StreamWriter(path, append: false);
        w.WriteLine("<?xml version=\"1.0\" encoding=\"UTF-8\"?>");
        w.WriteLine("<summary>");
        foreach (var s in steps)
        {
            w.Write("    <step time=\"");
            w.Write(s.Time.ToString("R", CultureInfo.InvariantCulture));
            w.Write("\" running=\"");
            w.Write(s.Running.ToString(CultureInfo.InvariantCulture));
            w.Write("\" arrived=\"");
            w.Write(s.Arrived.ToString(CultureInfo.InvariantCulture));
            w.Write("\" meanSpeed=\"");
            w.Write((s.MeanSpeed ?? -1.0).ToString("R", CultureInfo.InvariantCulture));
            w.WriteLine("\"/>");
        }

        w.WriteLine("</summary>");
    }

    private static string Escape(string s) => s
        .Replace("&", "&amp;")
        .Replace("<", "&lt;")
        .Replace(">", "&gt;")
        .Replace("\"", "&quot;");

    // A scenario dir has exactly one of each input; more than one is ambiguous, so refuse.
    private static string? SingleFile(string dir, string pattern)
    {
        var matches = Directory.GetFiles(dir, pattern);
        return matches.Length == 1 ? matches[0] : null;
    }
}

/// <summary>
/// VB-7 stability metric: the phase-1 engine runs teleport-OFF (CLAUDE.md "Determinism (phase
/// 1)" / no teleport implemented), so there is no SUMO-style teleport count to read off the
/// engine side. The analog VIZ_BENCH_TASKS.md VB-7 asks for is "completion + a stuck/gridlock
/// detector (count vehicles making ~zero progress over a window)". Implemented purely by
/// scanning each vehicle's own (time, speed) series for the longest CONSECUTIVE run of steps
/// with speed below <paramref name="speedThreshold"/> -- "consecutive" meaning back-to-back
/// StepLength-spaced frames with no gap (a gap means the vehicle was not in the network that
/// step, i.e. not yet departed or already arrived, not "stuck").
/// </summary>
internal static class StuckDetector
{
    public static StuckResult Detect(
        Dictionary<string, List<TrajectoryPoint>> byVehicle,
        double stepLength,
        double windowSeconds,
        double speedThreshold,
        double finalStepTime)
    {
        var everStuck = 0;
        var stillStuckAtEnd = 0;
        var windowSteps = Math.Max(1, (int)Math.Round(windowSeconds / stepLength));

        foreach (var points in byVehicle.Values)
        {
            if (points.Count == 0)
            {
                continue;
            }

            var runLength = 0;
            var maxRun = 0;
            var runEndsAtFinalStep = false;

            for (var i = 0; i < points.Count; i++)
            {
                var isSlow = points[i].Speed < speedThreshold;
                var isConsecutive = i == 0 || IsApproximately(points[i].Time - points[i - 1].Time, stepLength);

                if (isSlow && isConsecutive)
                {
                    runLength++;
                }
                else if (isSlow)
                {
                    runLength = 1;
                }
                else
                {
                    runLength = 0;
                }

                if (runLength > maxRun)
                {
                    maxRun = runLength;
                }

                if (i == points.Count - 1 && runLength >= windowSteps && IsApproximately(points[i].Time, finalStepTime))
                {
                    runEndsAtFinalStep = true;
                }
            }

            if (maxRun >= windowSteps)
            {
                everStuck++;
                if (runEndsAtFinalStep)
                {
                    stillStuckAtEnd++;
                }
            }
        }

        return new StuckResult(everStuck, stillStuckAtEnd);
    }

    private static bool IsApproximately(double a, double b) => Math.Abs(a - b) < 1e-6;
}

internal readonly record struct StuckResult(int EverStuckCount, int StillStuckAtEndCount);
