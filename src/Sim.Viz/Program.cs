using System.Text.Json;
using System.Text.Json.Serialization;
using Sim.Core;
using Sim.Harness;
using Sim.Ingest;
using static Sim.Viz.PayloadBuilder;

namespace Sim.Viz;

// VB-1..VB-4 (VIZ_SPEC.md): builds the compact REPLAY_DATA JSON the committed front-end template
// consumes and writes a fully self-contained HTML replay. REPLAY_DATA is now a UNIFIED multi-scene
// payload `{ scenes: [ SCENE, ... ] }` (see Payload.cs); the template renders one scene at a time
// with a scene selector.
//
// Two modes:
//   dotnet run --project src/Sim.Viz -- <scenarioDir> [--fcd <path>]
//       Single-scenario mode (unchanged behaviour): reads a scenario dir's net+fcd+rou and writes
//       <scenarioDir>/replay.html. The one scenario is wrapped as a one-element scenes array, so it
//       shares the exact same template path as the bundle.
//
//   dotnet run --project src/Sim.Viz -- --bundle <outPath>
//       Bundle mode (NEW): assembles the five showcase scenes (two FCD laneless/sublane scenarios
//       plus three programmatically-generated open-space/cross-regime crowd scenes) into ONE
//       self-contained HTML written to <outPath>.
//
// Not part of `dotnet test` -- a utility, and never touches the parity engine's inputs/goldens.
internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            Console.Error.WriteLine("usage: Sim.Viz <scenarioDir> [--fcd <path>]");
            Console.Error.WriteLine("       Sim.Viz --bundle <outPath>");
            Console.Error.WriteLine("       Sim.Viz --evac-organic <outPath>");
            Console.Error.WriteLine("       Sim.Viz --evac-city <outPath>");
            Console.Error.WriteLine("       Sim.Viz --evac-district <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-crossing-gate <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-lod-promotion <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-od-routing <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-dodge <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-reroute <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-parking <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-liveliness <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-social <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-waiter <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-lively-crowd <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-remote <outPath>");
            Console.Error.WriteLine("       Sim.Viz --ped-subarea-fcd <outPath.fcd.xml> [--dial d] [--seconds s] [--box <dir>]");
            return args.Length == 0 ? 2 : 0;
        }

        return args[0] switch
        {
            "--bundle" => RunBundle(args),
            "--evac-organic" => RunEvacOrganic(args),
            "--evac-city" => RunEvacCity(args),
            "--evac-district" => RunEvacDistrict(args),
            "--ped-crossing-gate" => RunPedCrossingGate(args),
            "--ped-lod-promotion" => RunPedLodPromotion(args),
            "--ped-od-routing" => RunPedOdRouting(args),
            "--ped-dodge" => RunPedScene(args, "--ped-dodge", SceneGen.BuildObstacleDodge),
            "--ped-reroute" => RunPedScene(args, "--ped-reroute", SceneGen.BuildCrossingReroute),
            "--ped-parking" => RunPedParking(args),
            "--ped-liveliness" => RunPedLiveliness(args),
            "--ped-social" => RunPedSocial(args),
            "--ped-waiter" => RunPedWaiter(args),
            "--ped-lively-crowd" => RunPedLivelyCrowd(args),
            "--ped-remote" => RunPedRemote(args),
            "--ped-subarea-fcd" => RunPedSubareaFcd(args),
            _ => RunSingle(args),
        };
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Liveliness" (LIVE-POC-1, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §12).
    // ---------------------------------------------------------------------------------------
    private static int RunPedLiveliness(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-liveliness requires an output path");
            return 2;
        }

        var outPath = args[1];
        var scene = SceneGen.BuildLiveliness();
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxDiscs = 0;
        var minDiscs = int.MaxValue;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxDiscs) maxDiscs = n;
            if (n < minDiscs) minDiscs = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxDiscs} " +
            $"minConcurrentDiscs={minDiscs}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Meet & talk" (LIVE-POC-2, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §5, §12).
    // ---------------------------------------------------------------------------------------
    private static int RunPedSocial(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-social requires an output path");
            return 2;
        }

        var outPath = args[1];
        var scene = SceneGen.BuildSocial();
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxDiscs = 0;
        var everTalked = false;
        var firstTalkTime = -1.0;
        for (var f = 0; f < scene.Frames.Length; f++)
        {
            var frame = scene.Frames[f];
            var n = 0;
            foreach (var d in frame.D)
            {
                if (d is null) continue;
                n++;
                if (d.Length > 3 && d[3] == SceneGen.KindPedTalk)
                {
                    everTalked = true;
                    if (firstTalkTime < 0.0) firstTalkTime = f * scene.Dt;
                }
            }

            if (n > maxDiscs) maxDiscs = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxDiscs} " +
            $"everTalked={everTalked} firstTalkTime={firstTalkTime:F2}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Waiter" (LIVE-POC-3, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §7, §12).
    // ---------------------------------------------------------------------------------------
    private static int RunPedWaiter(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-waiter requires an output path");
            return 2;
        }

        var outPath = args[1];
        var scene = SceneGen.BuildWaiter();
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxDiscs = 0;
        var minDiscs = int.MaxValue;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxDiscs) maxDiscs = n;
            if (n < minDiscs) minDiscs = n;
        }

        // Independent evidence (not the render path): rebuild the SAME waiter ActivityTimeline
        // (SceneGen.BuildWaiterTimeline is the single source of truth both the scene and this report
        // reconstruct from) and scan it directly for a hidden (inside) instant and a visible-serving
        // instant, reporting the first of each found.
        var waiterTimeline = SceneGen.BuildWaiterTimeline();
        var hiddenTime = -1.0;
        var servingTime = -1.0;
        const double reportDt = 0.05;
        for (var now = 0.0; now <= waiterTimeline.EndTime && (hiddenTime < 0.0 || servingTime < 0.0); now += reportDt)
        {
            var sample = waiterTimeline.PoseAt(now);
            if (hiddenTime < 0.0 && !sample.Visible)
            {
                hiddenTime = now;
            }

            if (servingTime < 0.0 && sample.Visible && sample.AnimTag == Sim.Pedestrians.Lod.WaiterScenario.ServeAnimTag)
            {
                servingTime = now;
            }
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxDiscs} " +
            $"minConcurrentDiscs={minDiscs} firstHiddenTime={hiddenTime:F2} firstServingTime={servingTime:F2}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Parking" (LotCoupling).
    // ---------------------------------------------------------------------------------------
    private static int RunPedParking(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-parking requires an output path");
            return 2;
        }

        var outPath = args[1];
        var scene = SceneGen.BuildParking();
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxDiscs = 0;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxDiscs) maxDiscs = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxDiscs}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Dodge / reroute" (local avoidance + BlockerRegistry/RerouteDriver).
    // ---------------------------------------------------------------------------------------
    private static int RunPedScene(string[] args, string flag, Func<string, ScenePayload> build)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine($"error: {flag} requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = build(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxPeds) maxPeds = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxPeds}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "OD routing" (PedDemand + SumoNavMesh).
    // ---------------------------------------------------------------------------------------
    private static int RunPedOdRouting(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-od-routing requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = SceneGen.BuildOdRouting(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxPeds) maxPeds = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentPeds={maxPeds}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Lively crowd" (LIVE-PROD-1b, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §4).
    // Mirrors RunPedOdRouting exactly, plus reports how many recorded frames actually contain a
    // KindPedPaused disc -- direct evidence (not just a claim) that the routed crowd visibly pauses.
    // ---------------------------------------------------------------------------------------
    private static int RunPedLivelyCrowd(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-lively-crowd requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = SceneGen.BuildLivelyCrowd(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        var framesWithPaused = 0;
        var maxConcurrentPaused = 0;
        const double pausedKind = 14.0; // SceneGen.KindPedPaused
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            var pausedThisFrame = 0;
            foreach (var d in frame.D)
            {
                if (d is null)
                {
                    continue;
                }

                n++;
                if (d.Length > 3 && d[3] == pausedKind)
                {
                    pausedThisFrame++;
                }
            }

            if (n > maxPeds) maxPeds = n;
            if (pausedThisFrame > 0) framesWithPaused++;
            if (pausedThisFrame > maxConcurrentPaused) maxConcurrentPaused = pausedThisFrame;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentPeds={maxPeds} "
            + $"framesWithPausedPed={framesWithPaused} maxConcurrentPaused={maxConcurrentPaused}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Remote (over the wire)" (P3-3, docs/PEDESTRIAN-TASKS.md). Reports
    // maxConcurrentPeds like the other ped scenes, plus everPromoted (a DR-switch was actually
    // observed on the wire) -- direct evidence the reconstructed crowd really did promote, not just
    // that the underlying LOD-promotion mechanism ran.
    // ---------------------------------------------------------------------------------------
    private static int RunPedRemote(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-remote requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = SceneGen.BuildPedRemote(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        var everPromoted = false;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D)
            {
                if (d is null) continue;
                n++;
                if (d.Length > 3 && d[3] == SceneGen.KindPedHighPower) everPromoted = true;
            }

            if (n > maxPeds) maxPeds = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxPeds} " +
            $"everPromoted={everPromoted}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Sub-area end-to-end recorder (P8-3 weighted demand + P8-4a density knob -> SUMO <person> FCD).
    // Unlike the other ped modes this emits a SUMO-schema FCD stream (not an HTML replay): the
    // pedestrian half of the shared car+ped replay (P8-5, sub-area session), which sim_viz renders
    // beside the box's vehicle FCD. See docs/PEDESTRIAN-P8-3/-P8-4 designs + PEDESTRIAN-P8-BACKLOG.md.
    // ---------------------------------------------------------------------------------------
    private static int RunPedSubareaFcd(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-subarea-fcd requires an output path (e.g. peds.fcd.xml)");
            return 2;
        }

        var outPath = args[1];
        var dial = 0.03;
        var seconds = 120.0;
        string? boxDir = null;
        for (var i = 2; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--dial" when i + 1 < args.Length:
                    dial = double.Parse(args[++i], System.Globalization.CultureInfo.InvariantCulture);
                    break;
                case "--seconds" when i + 1 < args.Length:
                    seconds = double.Parse(args[++i], System.Globalization.CultureInfo.InvariantCulture);
                    break;
                case "--box" when i + 1 < args.Length:
                    boxDir = args[++i];
                    break;
                default:
                    Console.Error.WriteLine($"error: unrecognized argument: {args[i]}");
                    return 2;
            }
        }

        boxDir ??= Path.Combine(RepoRoot(), "scenarios", "_ped", "subarea-box");
        if (!Directory.Exists(boxDir))
        {
            Console.Error.WriteLine($"error: box directory not found: {boxDir}");
            return 2;
        }

        var options = new Sim.Pedestrians.SubareaFcdRecorder.Options { Dial = dial, Seconds = seconds };
        Sim.Pedestrians.SubareaFcdRecorder.Result result;
        using (var writer = new Sim.Pedestrians.PersonFcdWriter(outPath))
        {
            result = Sim.Pedestrians.SubareaFcdRecorder.Record(boxDir, writer, options);
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={result.Frames} dial={dial} cap={result.PopulationCap} " +
            $"rate={result.SpawnRatePerSecond:F3}/s walkableKm={result.WalkableLengthKm:F3} endpoints={result.Endpoints} " +
            $"peakLive={result.PeakLive} spawns={result.Spawns} arrivals={result.Arrivals}");
        // P8-1b navmesh-connectivity health (should be a few components + ~0 unreachable skips on a good crop).
        Console.WriteLine(
            $"  navmesh: polygons={result.WalkablePolygons} components={result.ConnectedComponents} " +
            $"unreachableSkips={result.UnreachableSkips}" +
            (result.ConnectedComponents > 5
                ? "  [WARN] fragmented navmesh -- crowd will be routing-limited (see docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md)"
                : string.Empty));
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "LOD promotion" (docs/PEDESTRIAN-POC-PLAN.md POC-3).
    // ---------------------------------------------------------------------------------------
    private static int RunPedLodPromotion(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-lod-promotion requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = SceneGen.BuildLodPromotion(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        var everPromoted = false;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D)
            {
                if (d is null) continue;
                n++;
                if (d.Length > 3 && d[3] == SceneGen.KindPedHighPower) everPromoted = true;
            }

            if (n > maxPeds) maxPeds = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentDiscs={maxPeds} " +
            $"everPromoted={everPromoted}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Pedestrian showcase: "Crossing gate" (docs/PEDESTRIAN-POC-PLAN.md POC-2). Emitted standalone
    // (like --evac-organic/--evac-city), not folded into --bundle -- a separate curation track for
    // the pedestrian demo batch (see scripts/gen-demos.sh).
    // ---------------------------------------------------------------------------------------
    private static int RunPedCrossingGate(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-crossing-gate requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_ped", "poc0-crossing-plaza");

        var scene = SceneGen.BuildCrossingGate(scenarioDir);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var vehicleSlots = scene.Frames.Length > 0 ? scene.Frames[0].V.Length : 0;
        var maxPeds = 0;
        foreach (var frame in scene.Frames)
        {
            var n = 0;
            foreach (var d in frame.D) if (d is not null) n++;
            if (n > maxPeds) maxPeds = n;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} vehicleSlots={vehicleSlots} " +
            $"maxConcurrentPeds={maxPeds}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Standalone 10k-city evac mode (PANIC-EVAC-PHASE5-TIER2-TASKS.md T2.6): emits JUST the one
    // scene, kept OUT of --bundle and --evac-organic (SceneGen.BuildEvacCity already bounds its own
    // payload via region-crop -- see the "region-crop:" console line it prints -- but it is still a
    // dedicated multi-MB scene reviewed on its own).
    // ---------------------------------------------------------------------------------------
    private static int RunEvacCity(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --evac-city requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();

        var scene = SceneGen.BuildEvacCity(repoRoot);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var vehicleSlots = scene.Frames.Length > 0 ? scene.Frames[0].V.Length : 0;

        var pedestrianDiscs = 0;
        foreach (var frame in scene.Frames)
        {
            foreach (var d in frame.D)
            {
                if (d is { Length: > 3 } && (d[3] == SceneGen.KindFleeing || d[3] == SceneGen.KindEscaped))
                {
                    pedestrianDiscs++;
                }
            }
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes, {size / 1024.0 / 1024.0:F2} MiB)  frames={scene.Frames.Length} " +
            $"vehicleSlots={vehicleSlots} pedestrianDiscs={pedestrianDiscs}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Standalone organic-town evac mode (PANIC-EVAC-PHASE5-TASKS.md T4.1): emits JUST the one scene,
    // kept OUT of --bundle -- at ~400 vehicle slots x 300 frames the payload is a few MB, which would
    // bloat the showcase bundle for no benefit (this scene is reviewed on its own).
    // ---------------------------------------------------------------------------------------
    private static int RunEvacOrganic(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --evac-organic requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();

        var scene = SceneGen.BuildEvacOrganic(repoRoot);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var vehicleSlots = scene.Frames.Length > 0 ? scene.Frames[0].V.Length : 0;

        var pedestrianDiscs = 0;
        foreach (var frame in scene.Frames)
        {
            foreach (var d in frame.D)
            {
                if (d is { Length: > 3 } && (d[3] == SceneGen.KindFleeing || d[3] == SceneGen.KindEscaped))
                {
                    pedestrianDiscs++;
                }
            }
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} vehicleSlots={vehicleSlots} " +
            $"pedestrianDiscs={pedestrianDiscs}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Standalone evac-district mode (P5-1(B), docs/PEDESTRIAN-TASKS.md): pedestrian panic evac
    // routed onto Sim.Pedestrians over the P5-PRE walkable net -- no vehicles, no scenario dir.
    // ---------------------------------------------------------------------------------------
    private static int RunEvacDistrict(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --evac-district requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();

        var scene = SceneGen.BuildEvacDistrict(repoRoot);
        var payload = new ReplayData(new[] { scene });
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        var maxPeds = 0;
        var maxPanicked = 0;
        var everPanicked = false;
        foreach (var frame in scene.Frames)
        {
            var total = 0;
            var panicked = 0;
            foreach (var d in frame.D)
            {
                if (d is not { Length: > 3 })
                {
                    continue;
                }

                if (d[3] == SceneGen.KindPedLowPower)
                {
                    total++;
                }
                else if (d[3] == SceneGen.KindPedHighPower)
                {
                    total++;
                    panicked++;
                    everPanicked = true;
                }
            }

            if (total > maxPeds) maxPeds = total;
            if (panicked > maxPanicked) maxPanicked = panicked;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine(
            $"wrote {outPath}  ({size} bytes)  frames={scene.Frames.Length} maxConcurrentPeds={maxPeds} " +
            $"maxConcurrentPanicked={maxPanicked} everPanicked={everPanicked}");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Single-scenario mode (backwards compatible).
    // ---------------------------------------------------------------------------------------
    private static int RunSingle(string[] args)
    {
        var scenarioDir = args[0];
        if (!Directory.Exists(scenarioDir))
        {
            Console.Error.WriteLine($"error: scenario directory not found: {scenarioDir}");
            return 2;
        }

        string? fcdOverride = null;
        for (var i = 1; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--fcd" when i + 1 < args.Length:
                    fcdOverride = args[++i];
                    break;
                default:
                    Console.Error.WriteLine($"error: unrecognized argument: {args[i]}");
                    return 2;
            }
        }

        var scene = BuildFcdScene(scenarioDir, fcdOverride, out var err);
        if (scene is null)
        {
            Console.Error.WriteLine(err);
            return 2;
        }

        var payload = new ReplayData(new[] { scene });
        var outPath = Path.Combine(scenarioDir, "replay.html");
        if (!WriteHtml(payload, scene.Name, outPath))
        {
            return 2;
        }

        Console.WriteLine(
            $"wrote {outPath}  (scene='{scene.Name}', lanes={scene.Network?.Lanes.Length ?? 0}, " +
            $"frames={scene.Frames.Length}, dt={scene.Dt:0.###})");
        return 0;
    }

    // ---------------------------------------------------------------------------------------
    // Bundle mode: the five-scene laneless showcase.
    // ---------------------------------------------------------------------------------------
    private static int RunBundle(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --bundle requires an output path");
            return 2;
        }

        var outPath = args[1];
        var repoRoot = RepoRoot();
        var scenarios = Path.Combine(repoRoot, "scenarios");

        var scenes = new List<ScenePayload>();

        // Scene 0 -- the panic-evacuation demo (opens first, docs/PANIC-EVAC-DESIGN.md S6): organized
        // grid traffic transitions to panic/abandonment/foot-flight under a central incident, driven
        // through the real Engine + external Sim.Evac.EvacDirector (the same fixture EvacSpineTests use).
        scenes.Add(SceneGen.BuildEvacGrid(repoRoot));

        // Scene 1 -- the Indian junction: SHAPED mixed traffic (long buses, compact
        // motorcycles) negotiating an uncontrolled crossroads by anisotropic avoidance with SOFT
        // priority (assertive main road vs yielding cross road), from the Sim.Core.Mixed layer.
        scenes.Add(SceneGen.BuildIndianJunction());

        // Scene 1 -- the earlier dense uncontrolled junction (disc agents): many mixed movers on a
        // shared crossroads with no lanes/signals (Egypt/India-style congestion), from the ORCA layer.
        scenes.Add(SceneGen.BuildDenseJunction());

        // Scene A -- FCD "Laneless overtake" (8 vehicles, lateral RVO).
        scenes.Add(RequireFcdScene(
            Path.Combine(scenarios, "65-mixed-sublane"),
            "Laneless overtake",
            "Eight vehicles on a laneless (sub-lane) road overtaking with continuous lateral RVO. "
            + "Replayed from the committed SUMO golden FCD trajectory."));

        // Scene B -- FCD "Sublane overtake" (a follower drifts to pass).
        scenes.Add(RequireFcdScene(
            Path.Combine(scenarios, "63-sublane-overtake-wide"),
            "Sublane overtake",
            "A faster follower drifts laterally within a wide lane to pass the vehicle ahead. "
            + "Replayed from the committed SUMO golden FCD trajectory."));

        // Scenes C/D/E -- programmatically generated from the engine's ORCA layer (no golden FCD).
        scenes.Add(SceneGen.BuildCarAvoidsPedestrian(Path.Combine(scenarios, "_fixtures", "bridge-crossing")));
        scenes.Add(SceneGen.BuildCounterFlow());
        scenes.Add(SceneGen.BuildCrossing());

        var payload = new ReplayData(scenes.ToArray());
        if (!WriteHtml(payload, "Laneless showcase", outPath))
        {
            return 2;
        }

        var size = new FileInfo(outPath).Length;
        Console.WriteLine($"wrote {outPath}  ({size} bytes, {scenes.Count} scenes)");
        foreach (var s in scenes)
        {
            Console.WriteLine(
                $"  - {s.Name}: network={(s.Network is null ? "none" : s.Network.Lanes.Length + " lanes")}, " +
                $"frames={s.Frames.Length}, dt={s.Dt:0.###}");
        }

        return 0;
    }

    private static ScenePayload RequireFcdScene(string scenarioDir, string name, string desc)
    {
        var scene = BuildFcdScene(scenarioDir, null, out var err, name, desc);
        return scene ?? throw new InvalidOperationException(err);
    }

    // ---------------------------------------------------------------------------------------
    // FCD scene builder: reads a scenario dir's net + rou + golden FCD and turns it into a SCENE
    // (network + vehicle-box frames). Reuses the original single-scenario derivation. Returns null
    // (with `err` set) if the inputs are missing, so the single-scenario CLI can report cleanly.
    // ---------------------------------------------------------------------------------------
    private static ScenePayload? BuildFcdScene(
        string scenarioDir,
        string? fcdOverride,
        out string err,
        string? name = null,
        string? desc = null)
    {
        err = string.Empty;

        var netPath = SingleFile(scenarioDir, "*.net.xml");
        var rouPath = SingleFile(scenarioDir, "*.rou.xml");
        var cfgPath = SingleFile(scenarioDir, "*.sumocfg");
        var fcdPath = fcdOverride ?? Path.Combine(scenarioDir, "golden.fcd.xml");

        if (netPath is null || rouPath is null)
        {
            err = $"error: scenario dir must contain exactly one each of *.net.xml, *.rou.xml " +
                  $"(found net={netPath}, rou={rouPath})";
            return null;
        }

        if (!File.Exists(fcdPath))
        {
            err = $"error: FCD file not found: {fcdPath}";
            return null;
        }

        var network = NetworkParser.Parse(netPath);
        var demand = DemandParser.Parse(rouPath);
        var trajectorySet = FcdParser.Parse(fcdPath);
        var config = cfgPath is not null ? ScenarioConfigParser.Parse(cfgPath) : null;

        var sceneName = name ?? Path.GetFileName(Path.GetFullPath(scenarioDir).TrimEnd('/', '\\'));

        var networkPayload = BuildNetwork(network);

        // Camera view = the extent of the network geometry plus every trajectory sample.
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in networkPayload.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in networkPayload.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        // Resolve one shared vehicle box dimension for the scene (VIZ_SPEC unified model uses a
        // single vdim per scene). Use the first vehicle's resolved vType; the committed sublane
        // scenarios are homogeneous passenger traffic, so this is representative.
        var vehicleTypeById = demand.Vehicles.ToDictionary(v => v.Id, v => v.TypeId);
        double vehLength = 0, vehWidth = 0;
        foreach (var vid in trajectorySet.VehicleIds)
        {
            if (vehicleTypeById.TryGetValue(vid, out var t) && demand.VTypesById.TryGetValue(t, out var vType))
            {
                var resolved = VTypeDefaults.Resolve(vType);
                vehLength = resolved.Length;
                vehWidth = resolved.Width;
                break;
            }
        }

        if (vehLength <= 0)
        {
            var fallback = VTypeDefaults.Resolve(new VType("__default__", "passenger", Sigma: null));
            vehLength = fallback.Length;
            vehWidth = fallback.Width;
        }

        // Fixed vehicle slots: a stable index per vehicle id (sorted for determinism), so slot i is
        // always the same vehicle across frames; a vehicle absent in a frame is null in its slot.
        var orderedIds = trajectorySet.VehicleIds.OrderBy(x => x, StringComparer.Ordinal).ToArray();
        var slotById = new Dictionary<string, int>(orderedIds.Length);
        for (var i = 0; i < orderedIds.Length; i++) slotById[orderedIds[i]] = i;

        // Group FCD points by timestep.
        var byTime = new SortedDictionary<double, Dictionary<string, (double X, double Y, double A)>>();
        foreach (var point in trajectorySet.AllPoints)
        {
            if (!byTime.TryGetValue(point.Time, out var atTime))
            {
                atTime = new Dictionary<string, (double, double, double)>();
                byTime[point.Time] = atTime;
            }

            atTime[point.VehicleId] = (point.X, point.Y, point.Angle);
            Track(point.X, point.Y);
        }

        var frames = new FramePayload[byTime.Count];
        var noDiscs = Array.Empty<double[]>();
        var fi = 0;
        foreach (var kv in byTime)
        {
            var v = new double[orderedIds.Length][];
            foreach (var (vid, st) in kv.Value)
            {
                v[slotById[vid]] = new[] { R(st.X), R(st.Y), R(st.A) };
            }

            frames[fi++] = new FramePayload(v, noDiscs);
        }

        var times = byTime.Keys.ToArray();
        var dt = times.Length > 1 ? times[1] - times[0] : config?.StepLength ?? 1.0;

        if (double.IsInfinity(minX))
        {
            minX = minY = 0;
            maxX = maxY = 1;
        }

        return new ScenePayload(
            sceneName,
            desc ?? sceneName,
            new[] { R(minX), R(minY), R(maxX), R(maxY) },
            networkPayload,
            new[] { vehLength, vehWidth },
            dt,
            frames);
    }

    // ---------------------------------------------------------------------------------------
    // Shared HTML writer: serialize the payload and inject it + the template JS into template.html.
    // ---------------------------------------------------------------------------------------
    private static bool WriteHtml(ReplayData payload, string title, string outPath)
    {
        var jsonOptions = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.Never,
            WriteIndented = false,
        };
        var json = JsonSerializer.Serialize(payload, jsonOptions);

        var templateDir = AppContext.BaseDirectory;
        var templateHtmlPath = Path.Combine(templateDir, "template.html");
        var templateJsPath = Path.Combine(templateDir, "template.js");
        if (!File.Exists(templateHtmlPath) || !File.Exists(templateJsPath))
        {
            Console.Error.WriteLine(
                $"error: template files not found next to the built exe ({templateHtmlPath}, {templateJsPath})");
            return false;
        }

        var html = File.ReadAllText(templateHtmlPath);
        var js = File.ReadAllText(templateJsPath);

        html = html.Replace("__SCENARIO_NAME__", title);
        html = html.Replace("/*REPLAY_DATA*/", json);
        html = html.Replace("/*TEMPLATE_JS*/", js);

        File.WriteAllText(outPath, html);
        return true;
    }

    private static string? SingleFile(string dir, string pattern)
    {
        var matches = Directory.GetFiles(dir, pattern);
        return matches.Length == 1 ? matches[0] : null;
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
