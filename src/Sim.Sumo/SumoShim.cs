using System.Globalization;
using Sim.Core;
using Sim.Harness;
using Sim.Ingest;

namespace Sim.Sumo;

// GAP-1 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §1, docs/SERVE-PATH-PLAN.md): the `sumo`-compatible CLI
// shim -- the drop-in binary the SumoData serve/replay pipeline invokes (via `SUMO_BINARY`) instead
// of vanilla `sumo`. It parses the vanilla flag shape and drives the SAME engine wiring Sim.Run
// already proves out (LoadScenario(cfg) multi-file <input> + Fcd/Summary/Statistic writers); no
// engine change lives here. This class is DELIBERATELY separate from Sim.Run so the sumo-compatible
// contract stays clean and out of Sim.Run's dev/viz flags (--warmup/--fcd-out/--parity/...).
//
// The core is a pure `Run(args, stdout, stderr) -> exit code` so the parity test can drive it in-
// process (no shelling out) and compare the produced FCD against the committed golden -- proving the
// CLI path drives the engine identically. Program.Main is a one-line delegate over Console.Out/Error.
//
// The exact invocation shapes SumoData shells out (all list-form subprocess, so a differently-NAMED
// binary is fine as long as the flags match):
//   sumo -c <cfg> --summary-output S.xml --statistic-output T.xml --end <N> --no-step-log true
//   sumo -c <cfg> --tripinfo-output TI.xml [--summary-output S.xml --statistic-output T.xml] --end <N> --no-step-log true
//   sumo -c <cfg> --fcd-output F.xml --end <N> --no-step-log true
//
// Supported flags (SUMO spellings):
//   -c/--configuration <cfg>   the .sumocfg (its <input> resolves net/route/additional relative to it)
//   -b/--begin <t>             sim begin time  (optional; default = cfg <begin>)
//   -e/--end <t>               sim end   time  (optional; default = cfg <end>) -- run length is
//                              round((end-begin)/step-length) steps, exactly how the parity tests
//                              pick their step count. Vanilla `--end` overrides the cfg's <end>.
//   --fcd-output <path>        SUMO-schema FCD             (FcdWriterObserver)
//   --summary-output <path>    per-step summary            (SummaryWriterObserver)
//   --statistic-output <path>  <teleports total=.. jam=..> (StatisticWriter)
//   --tripinfo-output <path>   SUMO-schema <tripinfo> per ARRIVED vehicle (id/depart/arrival/
//                              arrivalLane/arrivalPos/arrivalSpeed/duration/routeLength/
//                              waitingTime/timeLoss -- GAP-2, docs/SUMOSHARP-SERVE-PATH-DROP-
//                              IN.md §2). Sourced from engine.CompletedTrips (Engine.cs's
//                              CaptureCompletedTrips), written via Sim.Harness.TripInfoWriter.
//   --no-step-log [bool]       accepted and ignored (we never print a per-step log)
// Any OTHER flag is TOLERATED (a warning to stderr, not an abort) so minor extra flags SumoData
// passes never break the run. Both `--flag value` and `--flag=value` forms are accepted.
public static class SumoShim
{
    // Thrown internally by value parsing; caught in Run so a bad numeric never escapes as a raw
    // FormatException (and so the testable Run never calls Environment.Exit).
    private sealed class CliError : Exception
    {
        public CliError(string message) : base(message) { }
    }

    public static int Run(string[] args, TextWriter stdout, TextWriter stderr)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help" or "-?")
        {
            stderr.WriteLine(
                "usage: sumosharp -c <cfg> [--begin t] [--end t] [--fcd-output F] " +
                "[--summary-output S] [--statistic-output T] [--tripinfo-output TI] [--no-step-log]");
            return args.Length == 0 ? 1 : 0;
        }

        string? cfgPath = null;
        double? beginOverride = null;
        double? endOverride = null;
        string? fcdOut = null;
        string? summaryOut = null;
        string? statisticOut = null;
        string? tripinfoOut = null;

        try
        {
            for (var i = 0; i < args.Length; i++)
            {
                var (flag, inlineValue) = SplitInline(args[i]);

                // Read the value for a value-taking flag: the inline `--flag=value` form if present,
                // else the next token. Throws a CliError when neither is available.
                string TakeValue()
                {
                    if (inlineValue is not null)
                    {
                        return inlineValue;
                    }

                    if (i + 1 < args.Length)
                    {
                        return args[++i];
                    }

                    throw new CliError($"{flag} requires a value");
                }

                switch (flag)
                {
                    case "-c":
                    case "--configuration":
                    case "--config-file":
                        cfgPath = TakeValue();
                        break;
                    case "-b":
                    case "--begin":
                        beginOverride = ParseTime(TakeValue(), flag);
                        break;
                    case "-e":
                    case "--end":
                        endOverride = ParseTime(TakeValue(), flag);
                        break;
                    case "--fcd-output":
                        fcdOut = TakeValue();
                        break;
                    case "--summary-output":
                        summaryOut = TakeValue();
                        break;
                    case "--statistic-output":
                        statisticOut = TakeValue();
                        break;
                    case "--tripinfo-output":
                        tripinfoOut = TakeValue();
                        break;
                    case "--no-step-log":
                        // Accept and ignore. SUMO passes `--no-step-log true`; the value (only when it
                        // is a bare boolean token, not the next real flag) is consumed and dropped.
                        if (inlineValue is null && i + 1 < args.Length && IsBooleanLiteral(args[i + 1]))
                        {
                            i++;
                        }

                        break;
                    default:
                        // Tolerate unknown flags: warn, don't abort (the doc's explicit requirement). If
                        // the unknown flag is followed by a non-flag token, treat that as its value and
                        // skip it too, so a `--some-extra value` pair does not desync the parser.
                        stderr.WriteLine($"warning: ignoring unrecognized argument '{args[i]}'");
                        if (inlineValue is null && i + 1 < args.Length && !args[i + 1].StartsWith('-'))
                        {
                            i++;
                        }

                        break;
                }
            }
        }
        catch (CliError ex)
        {
            stderr.WriteLine($"error: {ex.Message}");
            return 1;
        }

        if (cfgPath is null)
        {
            stderr.WriteLine("error: no configuration given (-c <cfg>)");
            return 1;
        }

        if (!File.Exists(cfgPath))
        {
            stderr.WriteLine($"error: configuration file not found: {cfgPath}");
            return 1;
        }

        ScenarioConfig config;
        try
        {
            config = ScenarioConfigParser.Parse(cfgPath);
        }
        catch (Exception ex)
        {
            stderr.WriteLine($"error: failed to parse '{cfgPath}': {ex.Message}");
            return 1;
        }

        if (config.RouteFiles.Count == 0)
        {
            stderr.WriteLine(
                $"error: '{cfgPath}' has no <input><net-file>/<route-files>; the sumo shim requires a " +
                "cfg with an <input> section (every SUMO .sumocfg has one).");
            return 1;
        }

        var beginTime = beginOverride ?? config.Begin;
        var endTime = endOverride ?? config.End;
        if (endTime <= beginTime)
        {
            stderr.WriteLine($"error: end ({endTime}) must be greater than begin ({beginTime}).");
            return 1;
        }

        var steps = (int)Math.Round((endTime - beginTime) / config.StepLength);

        var engine = new Engine();
        try
        {
            engine.LoadScenario(cfgPath);
        }
        catch (Exception ex)
        {
            stderr.WriteLine($"error: failed to load scenario: {ex.Message}");
            return 1;
        }

        // Register only the writers the caller asked for; each is additive and reads the same per-frame
        // export snapshot (see the observer classes' own comments). No flag -> no observer -> no file.
        using (var fcdWriter = fcdOut is not null ? new FcdWriterObserver(fcdOut) : null)
        using (var summaryWriter = summaryOut is not null ? new SummaryWriterObserver(summaryOut) : null)
        {
            if (fcdWriter is not null)
            {
                engine.AddExportObserver(fcdWriter);
            }

            if (summaryWriter is not null)
            {
                engine.AddExportObserver(summaryWriter);
            }

            engine.Run(steps);
        }

        if (statisticOut is not null)
        {
            StatisticWriter.Write(statisticOut, engine.TeleportCount, teleportsJam: engine.TeleportCountJam);
        }

        if (fcdOut is not null)
        {
            stdout.WriteLine($"wrote {fcdOut}");
        }

        if (summaryOut is not null)
        {
            stdout.WriteLine($"wrote {summaryOut}");
        }

        if (statisticOut is not null)
        {
            stdout.WriteLine($"wrote {statisticOut}");
        }

        if (tripinfoOut is not null)
        {
            // GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2): engine.CompletedTrips is the real per-
            // vehicle arrival record (Sim.Core.CompletedTripInfo, captured at the route-end arrival
            // seam). Adapt it to Sim.Harness.TripInfoRecord here -- Sim.Core cannot reference
            // Sim.Harness (Sim.Harness already depends on Sim.Core; see CompletedTripInfo's own header
            // comment for why), so this shim, which references both, is where the two meet.
            var trips = new List<TripInfoRecord>(engine.CompletedTrips.Count);
            foreach (var trip in engine.CompletedTrips)
            {
                trips.Add(new TripInfoRecord(
                    trip.Id, trip.Depart, trip.Duration, trip.ArrivalSpeed,
                    ArrivalLane: trip.ArrivalLane,
                    ArrivalPos: trip.ArrivalPos,
                    ArrivalTime: trip.Arrival,
                    RouteLength: trip.RouteLength,
                    WaitingTime: trip.WaitingTime,
                    TimeLoss: trip.TimeLoss));
            }

            TripInfoWriter.Write(tripinfoOut, trips);
            stdout.WriteLine($"wrote {tripinfoOut}");
        }

        stdout.WriteLine($"ran {steps} steps over [{beginTime}, {endTime}] @ {config.StepLength}s");
        return 0;
    }

    // "--flag=value" -> ("--flag", "value"); "--flag" -> ("--flag", null). Only the FIRST '=' splits,
    // so a value containing '=' survives intact.
    private static (string Flag, string? InlineValue) SplitInline(string arg)
    {
        var eq = arg.IndexOf('=');
        return eq < 0 ? (arg, null) : (arg[..eq], arg[(eq + 1)..]);
    }

    private static double ParseTime(string value, string flag)
    {
        if (!double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var t))
        {
            throw new CliError($"{flag} value '{value}' is not a number");
        }

        return t;
    }

    private static bool IsBooleanLiteral(string s) =>
        s is "true" or "false" or "1" or "0" or "True" or "False";
}
