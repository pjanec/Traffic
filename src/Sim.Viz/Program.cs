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
            "--ped-weave-csv" => RunPedWeaveCsv(args),
            "--ped-weave-bend-csv" => RunPedWeaveBendCsv(args),
            "--ped-weave-anim-csv" => RunPedWeaveAnimCsv(args),
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
    // PED-REALISM-1 Prototype 1 (docs/PEDESTRIAN-PLANNING-INTENTS.md Section 9): dump low-power ped
    // trajectories with LateralWeave applied, over a straight corridor with two OPPOSING streams, so the
    // weave's LOOK can be plotted (trails) and judged before wiring it into the recorder / synthetic mesh.
    // CSV columns: ped,dir,x,y (dir = +1 eastbound / -1 westbound). A visual prototype tool; off the parity path.
    private static int RunPedWeaveCsv(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-weave-csv requires an output path");
            return 2;
        }

        var outPath = args[1];
        const double length = 60.0;   // corridor length (m)
        const double halfWidth = 2.0; // sidewalk half-width (m) -> 4 m walk
        const int perStream = 40;     // peds per direction
        const double ds = 0.5;        // trajectory sample step (m)
        var wp = Sim.Pedestrians.Lod.WeaveParams.Default;
        var inv = System.Globalization.CultureInfo.InvariantCulture;

        var sb = new System.Text.StringBuilder();
        sb.Append("ped,dir,x,y,t\n");

        // A SHARED, moving interface c(x) between the two counterflowing streams -- one scenario-global seed,
        // evaluated at the CORRIDOR position x (so both directions see the same interface at the same x) --
        // realising the "moving centreline" (docs/PEDESTRIAN-PLANNING-INTENTS.md Section 3 shared field).
        const ulong globalSeed = 777UL;
        var maxShift = 0.35 * halfWidth;

        // Eastbound (+x) keeps below the interface (Y in [-halfWidth, c]); westbound (-x) keeps above it
        // (Y in [c, +halfWidth]). The stream the interface drifts toward is squeezed, the other widens.
        for (var stream = 0; stream < 2; stream++)
        {
            var dir = stream == 0 ? 1 : -1;
            for (var i = 0; i < perStream; i++)
            {
                var seed = (ulong)((stream * 100_000) + i + 1);
                var pedId = (stream * perStream) + i;
                const double speed = 1.3;      // m/s, so the ped experiences the interface EVOLVING as it walks
                var startT = i * 1.0;          // staggered spawn -> peds at the same x see different interface phases
                for (var s = 0.0; s <= length + 1e-9; s += ds)
                {
                    var cx = dir == 1 ? s : length - s; // corridor position by travel direction
                    var now = startT + (s / speed);     // wall time when this ped is at arc-length s
                    var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(cx, now, length, globalSeed, maxShift, wp);
                    double room, y;
                    if (dir == 1)
                    {
                        room = c + halfWidth; // room below the interface
                        y = c - Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, room, wp);
                    }
                    else
                    {
                        room = halfWidth - c; // room above the interface
                        y = c + Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, room, wp);
                    }

                    sb.Append(pedId).Append(',').Append(dir).Append(',')
                      .Append(cx.ToString("F3", inv)).Append(',')
                      .Append(y.ToString("F3", inv)).Append(',')
                      .Append(now.ToString("F2", inv)).Append('\n');
                }
            }
        }

        // Emit the interface c(x, t) at several TIME snapshots (dir=0) so the plot shows it migrate over time.
        foreach (var t in new[] { 0.0, 10.0, 20.0, 30.0, 40.0, 50.0 })
        {
            for (var x = 0.0; x <= length + 1e-9; x += ds)
            {
                var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(x, t, length, globalSeed, maxShift, wp);
                sb.Append(-1).Append(',').Append(0).Append(',')
                  .Append(x.ToString("F3", inv)).Append(',').Append(c.ToString("F3", inv)).Append(',')
                  .Append(t.ToString("F2", inv)).Append('\n');
            }
        }

        System.IO.File.WriteAllText(outPath, sb.ToString());
        Console.WriteLine($"wrote {outPath}  streams=2 perStream={perStream} length={length} halfWidth={halfWidth}");
        return 0;
    }

    // PED-REALISM-1 Prototype 2 (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md Section 8): the SAME weave over a
    // MULTI-SEGMENT route with a rounded bend, to validate the production requirement that the offset flows
    // CONTINUOUSLY across a segment join and anchors ONLY at the true O/D -- never re-zeroed at the bend. The
    // lateral frame (corridor-left) follows the centreline as it turns, so the keep-right bands + moving
    // interface wrap around the corner. World-XY output (columns ped,dir,x,y,t; dir 2 = centreline).
    // PED-REALISM-1 Prototype C (docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md §11): an ANIMATED demo -- a
    // flowing weave crowd (two counterflows, staggered spawns) PLUS low-power live-behaviour actors expressed
    // as lateral-profile overrides (§10.1): "check phone" = drift to the kerb, Pause, rejoin; "enter building"
    // = drift to a doorway, vanish. All deterministic, no ORCA (§9a). Dumps per-frame disc positions
    // (columns frame,x,y,kind) for a GIF. Harness only; off the parity path.
    private static int RunPedWeaveAnimCsv(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-weave-anim-csv requires an output path");
            return 2;
        }

        var outPath = args[1];
        const double length = 50.0, halfWidth = 2.0, speed = 1.3;
        const double spawnEvery = 1.7;             // seconds between spawns per stream
        const double tMax = 34.0, dt = 0.2;        // animation window / frame step
        const ulong globalSeed = 777UL;
        var maxShift = 0.35 * halfWidth;
        var wp = Sim.Pedestrians.Lod.WeaveParams.Default;
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        var sb = new System.Text.StringBuilder();
        sb.Append("frame,x,y,kind\n");

        double Smooth(double x) { var t = x < 0 ? 0 : (x > 1 ? 1 : x); return t * t * (3 - (2 * t)); }

        // Per-ped seeded preferred speed (m/s), ~0.9..1.7 -- breaks the lockstep, and (with the weave's lateral
        // spread) lets a faster ped overtake a slower one on a neighbouring track, no reactivity. Deterministic;
        // in production this is just the PathArc leg's `speed`, already on the wire -> server==IG, zero extra bytes.
        double PedSpeed(ulong seed)
        {
            var z = unchecked(((seed ^ 0x5EED0000BADF00D5UL) * 0x9E3779B97F4A7C15UL) + 0x9E3779B97F4A7C15UL);
            z = unchecked((z ^ (z >> 30)) * 0xBF58476D1CE4E5B9UL);
            z = unchecked((z ^ (z >> 27)) * 0x94D049BB133111EBUL);
            z ^= z >> 31;
            var u = (z >> 11) * (1.0 / 9007199254740992.0);
            return 0.9 + (0.8 * u);
        }

        // One ambient weave ped's lateral offset (signed, world Y) at corridor-arc a, own arc-length s.
        double AmbientY(int dir, double s, double a, double now, ulong seed)
        {
            var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, now, length, globalSeed, maxShift, wp);
            return dir == 1
                ? c - Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, c + halfWidth, wp)
                : c + Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, halfWidth - c, wp);
        }

        for (var f = 0; f * dt <= tMax + 1e-9; f++)
        {
            var now = f * dt;

            // Two ambient counterflows. Spawn from before t=0 so the corridor is pre-populated.
            for (var stream = 0; stream < 2; stream++)
            {
                var dir = stream == 0 ? 1 : -1;
                var k0 = -(int)Math.Ceiling((length / 0.9) / spawnEvery); // slowest ped needs the most lead time
                var kMax = (int)Math.Floor(tMax / spawnEvery);
                for (var k = k0; k <= kMax; k++)
                {
                    var startT = k * spawnEvery;
                    var seed = (ulong)((stream * 100_000) + (k - k0) + 1);
                    var s = PedSpeed(seed) * (now - startT); // per-ped preferred speed (no lockstep)
                    if (s < 0.0 || s > length)
                    {
                        continue;
                    }

                    var a = dir == 1 ? s : length - s;
                    var x = a;
                    var y = AmbientY(dir, s, a, now, seed);
                    sb.Append(f).Append(',').Append(x.ToString("F2", inv)).Append(',')
                      .Append(y.ToString("F2", inv)).Append(',').Append(dir == 1 ? "east" : "west").Append('\n');
                }
            }

            // ACTOR 1 -- "check phone" (eastbound): walk -> step to the kerb -> Pause -> rejoin. §9a.
            AppendPhoneActor(sb, f, now, startT: 3.0, eventArc: 26.0, dwell: 6.0, length, halfWidth, speed, globalSeed, maxShift, wp, inv, Smooth, seed: 424242UL);

            // ACTOR 2 -- "enter building" (westbound): drift to a doorway on the building side, then vanish. §9a.
            AppendDoorwayActor(sb, f, now, startT: 1.0, doorArc: 30.0, length, halfWidth, speed, globalSeed, maxShift, wp, inv, Smooth, seed: 515151UL);

            // ACTOR 3 -- ONE scarce "drunk / distracted" ped: the same weave but with an exaggerated micro-sway
            // (§9b-bis note -- big lateral wander used SPARINGLY adds life; it is NOT the norm). Eastbound.
            {
                const double dStart = 6.0;
                const ulong dSeed = 909090UL;
                if (now >= dStart)
                {
                    var s = 1.1 * (now - dStart); // a slightly slow, meandering walker
                    if (s <= length)
                    {
                        var a = s;
                        var drunkWp = wp with { MicroAmpMeters = 0.55, MicroWavelengthMeters = 4.0 };
                        var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, now, length, globalSeed, maxShift, wp);
                        var y = c - Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, dSeed, c + halfWidth, drunkWp);
                        sb.Append(f).Append(',').Append(a.ToString("F2", inv)).Append(',')
                          .Append(y.ToString("F2", inv)).Append(",drunk\n");
                    }
                }
            }
        }

        System.IO.File.WriteAllText(outPath, sb.ToString());
        Console.WriteLine($"wrote {outPath}  frames={(int)(tMax / dt) + 1} length={length} (ambient weave + phone + doorway actors)");
        return 0;
    }

    private static void AppendPhoneActor(
        System.Text.StringBuilder sb, int f, double now, double startT, double eventArc, double dwell,
        double length, double halfWidth, double speed, ulong globalSeed, double maxShift,
        Sim.Pedestrians.Lod.WeaveParams wp, System.Globalization.CultureInfo inv, Func<double, double> smooth, ulong seed)
    {
        if (now < startT) return;
        var pauseStart = startT + (eventArc / speed);
        var pauseEnd = pauseStart + dwell;
        double s;
        var onPhone = false;
        if (now < pauseStart) s = speed * (now - startT);
        else if (now < pauseEnd) { s = eventArc; onPhone = true; }
        else s = eventArc + (speed * (now - pauseEnd));
        if (s > length) return;

        var a = s; // eastbound
        var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, now, length, globalSeed, maxShift, wp);
        var weaveY = c - Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, c + halfWidth, wp);
        var kerbY = -(halfWidth * 0.92); // eastbound kerb side is -y
        const double blend = 3.0;
        double y;
        if (now < pauseStart) y = Lerp(weaveY, kerbY, smooth(Clamp01((s - (eventArc - blend)) / blend)));
        else if (now < pauseEnd) y = kerbY;
        else y = Lerp(kerbY, weaveY, smooth(Clamp01((s - eventArc) / blend)));

        sb.Append(f).Append(',').Append(a.ToString("F2", inv)).Append(',')
          .Append(y.ToString("F2", inv)).Append(',').Append(onPhone ? "phone" : "actor").Append('\n');
    }

    private static void AppendDoorwayActor(
        System.Text.StringBuilder sb, int f, double now, double startT, double doorArc,
        double length, double halfWidth, double speed, ulong globalSeed, double maxShift,
        Sim.Pedestrians.Lod.WeaveParams wp, System.Globalization.CultureInfo inv, Func<double, double> smooth, ulong seed)
    {
        if (now < startT) return;
        var s = speed * (now - startT);
        var arriveT = startT + (doorArc / speed);
        if (now >= arriveT + 0.6) return; // entered the building -> vanished (visible=false)

        var a = length - s; // westbound corridor arc
        var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, now, length, globalSeed, maxShift, wp);
        var weaveY = c + Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, halfWidth - c, wp);
        var doorY = halfWidth * 0.98; // building side (+y) doorway
        const double blend = 4.0;
        // corridor arc AT the door:
        var y = Lerp(weaveY, doorY, smooth(Clamp01((s - (doorArc - blend)) / blend)));
        var x = length - s;
        sb.Append(f).Append(',').Append(x.ToString("F2", inv)).Append(',')
          .Append(y.ToString("F2", inv)).Append(',').Append("door").Append('\n');
    }

    private static double Lerp(double x, double y, double t) => x + ((y - x) * t);
    private static double Clamp01(double v) => v < 0 ? 0 : (v > 1 ? 1 : v);

    private static int RunPedWeaveBendCsv(string[] args)
    {
        if (args.Length < 2)
        {
            Console.Error.WriteLine("error: --ped-weave-bend-csv requires an output path");
            return 2;
        }

        var outPath = args[1];
        const double halfWidth = 2.0;
        const int perStream = 40;
        const double stepDs = 0.25;
        var wp = Sim.Pedestrians.Lod.WeaveParams.Default;
        var inv = System.Globalization.CultureInfo.InvariantCulture;

        // Dense centreline: straight east, a rounded 60-deg left bend (the walkingArea curve), straight again.
        var px = new List<double>();
        var py = new List<double>();
        double x = 0.0, y = 0.0, h = 0.0; // heading (rad)
        void March(double dist, double curvature)
        {
            var n = (int)Math.Round(dist / stepDs);
            for (var i = 0; i < n; i++)
            {
                px.Add(x); py.Add(y);
                h += curvature * stepDs;
                x += stepDs * Math.Cos(h);
                y += stepDs * Math.Sin(h);
            }
        }

        const double turn = Math.PI / 3.0; // 60 deg
        const double radius = 6.0;
        March(24.0, 0.0);              // straight approach
        March(radius * turn, 1.0 / radius); // rounded left bend (curvature 1/R)
        March(24.0, 0.0);              // straight after the bend
        px.Add(x); py.Add(y);
        var lastIdx = px.Count - 1;
        var length = lastIdx * stepDs; // total route arc-length

        (double tx, double ty) Tangent(int idx)
        {
            var a = Math.Max(0, idx - 1);
            var b = Math.Min(lastIdx, idx + 1);
            var dx = px[b] - px[a];
            var dy = py[b] - py[a];
            var m = Math.Sqrt((dx * dx) + (dy * dy));
            return m > 1e-9 ? (dx / m, dy / m) : (1.0, 0.0);
        }

        const ulong globalSeed = 777UL;
        var maxShift = 0.35 * halfWidth;

        var sb = new System.Text.StringBuilder();
        sb.Append("ped,dir,x,y,t\n");

        // c(a) and corridor-left frame are evaluated at the CORRIDOR arc-length a (shared by both directions);
        // the per-ped lane plan uses the ped's OWN arc-length s. A = s for eastbound, A = length - s for west.
        void Emit(int pedId, int dir, double worldX, double worldY, double t)
            => sb.Append(pedId).Append(',').Append(dir).Append(',')
                 .Append(worldX.ToString("F3", inv)).Append(',')
                 .Append(worldY.ToString("F3", inv)).Append(',')
                 .Append(t.ToString("F2", inv)).Append('\n');

        for (var stream = 0; stream < 2; stream++)
        {
            var dir = stream == 0 ? 1 : -1;
            for (var i = 0; i < perStream; i++)
            {
                var seed = (ulong)((stream * 100_000) + i + 1);
                var pedId = (stream * perStream) + i;
                const double speed = 1.3;
                var startT = i * 1.0;
                for (var s = 0.0; s <= length + 1e-9; s += stepDs)
                {
                    var a = dir == 1 ? s : length - s;            // corridor arc-length (shared frame)
                    var idx = (int)Math.Round(a / stepDs);
                    if (idx < 0) idx = 0; else if (idx > lastIdx) idx = lastIdx;
                    var (tx, ty) = Tangent(idx);
                    var leftX = -ty; var leftY = tx;             // corridor-left = rot(tangent, +90)
                    var now = startT + (s / speed);
                    var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, now, length, globalSeed, maxShift, wp);
                    double lat = dir == 1
                        ? c - Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, c + halfWidth, wp)
                        : c + Sim.Pedestrians.Lod.LateralWeave.Offset(s, length, seed, halfWidth - c, wp);
                    Emit(pedId, dir, px[idx] + (lat * leftX), py[idx] + (lat * leftY), now);
                }
            }
        }

        // Centreline (dir=2) + interface c(a,t) snapshots (dir=0), in world XY.
        for (var idx = 0; idx <= lastIdx; idx++)
        {
            Emit(-2, 2, px[idx], py[idx], 0.0);
        }

        foreach (var t in new[] { 0.0, 15.0, 30.0, 45.0 })
        {
            for (var idx = 0; idx <= lastIdx; idx++)
            {
                var a = idx * stepDs;
                var (tx, ty) = Tangent(idx);
                var c = Sim.Pedestrians.Lod.LateralWeave.CenterShift(a, t, length, globalSeed, maxShift, wp);
                Emit(-1, 0, px[idx] + (c * -ty), py[idx] + (c * tx), t);
            }
        }

        System.IO.File.WriteAllText(outPath, sb.ToString());
        Console.WriteLine($"wrote {outPath}  bent route length={length:F1} m, {2 * perStream} peds");
        return 0;
    }

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
        var reachableFilter = false;
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
                // P8-1c Part 2: draw O/D demand only from the dominant reachable navmesh component(s) -- for a
                // fragmented real crop where unbridgeable island stubs would otherwise starve the crowd. Inert
                // on a connected box (the committed witnesses), so their recorder output is unchanged with it on.
                case "--reachable-filter":
                    reachableFilter = true;
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

        var options = new Sim.Pedestrians.SubareaFcdRecorder.Options { Dial = dial, Seconds = seconds, ReachableFilter = reachableFilter };
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
