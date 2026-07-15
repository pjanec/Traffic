namespace Sim.Viewer.Core;

// docs/SUMOSHARP-VIEWER-DEMO-EVAC-DESIGN.md §1: the single, static, data-driven place to add/curate
// demos for the native viewer's picker. Every entry names a DemoKind (how EngineHost builds it — see
// DemoSession.BuildHost) and a DemoCategory (how the ImGui Demos panel groups it — T5).
public enum DemoKind
{
    Scenario, // a committed scenario dir (net + rou + sumocfg) -- EngineHost auto-detects scenario mode.
    Sandbox,  // a bare net.net.xml (no demand) -- EngineHost's runtime random-traffic spawner fills it.
    Evac,     // a live Sim.Evac scenario -- EngineHost.CreateEvac(EvacKind, repoRoot).
}

public enum DemoCategory
{
    Junctions,
    TrafficLights,
    LaneChange,
    Rail,
    CityScale,
    Sandbox,
    Evacuation,
}

// PathRelToRepo convention (picked here, applied consistently everywhere in `All`): for Scenario/Sandbox
// entries it is a path relative to repoRoot INCLUDING the top-level `scenarios/` or `samples/` prefix,
// e.g. "scenarios/11-priority-junction", "scenarios/_bench/city-15000", "samples/junctions/cross" -- so
// Resolve() and DemoSession.BuildHost can do a plain Path.Combine(repoRoot, entry.PathRelToRepo) with no
// special-casing. It names a DIRECTORY (containing a *.net.xml, plus a *.rou.xml/*.sumocfg for Scenario
// entries); it is never a direct file path in this catalog (EngineHost/ResolveNetPath resolve the
// directory's *.net.xml the same way `--mode local <dir>` already does). Evac entries carry "" here --
// their net path is derived from EvacKind by EngineHost.CreateEvac / DemoCatalog.EvacNetPath instead.
public sealed record DemoEntry(
    string Name,
    string Blurb,
    DemoCategory Category,
    DemoKind Kind,
    string PathRelToRepo,
    string? EvacKind = null,
    int SandboxFleet = 0);

public static class DemoCatalog
{
    // Walk up from the running assembly's directory to the directory containing Traffic.sln -- the same
    // pattern the parity tests use (e.g. tests/Sim.ParityTests/RblLeftTurnsDiagTests.cs's RepoRoot()),
    // copied here so the viewer (which never references the test project) can resolve it independently.
    public static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above assembly).");
    }

    // The curated starter set from §1. An entry whose backing path is missing on disk (a trimmed
    // checkout, e.g. without the `_bench` city nets) is dropped by Resolve() below, logged, not thrown.
    public static IReadOnlyList<DemoEntry> All { get; } = new[]
    {
        // -- Junctions --------------------------------------------------------------------------------
        new DemoEntry("Priority junction", "Right-of-way at an uncontrolled priority junction.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/11-priority-junction"),
        new DemoEntry("Right-before-left", "Unmarked junction resolved by the right-before-left rule.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/26-right-before-left"),
        new DemoEntry("Right-before-left (4 left turns)",
            "Four simultaneous left turns at an RBL junction -- shows the deadlock fix.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/_diag/rbl-left-turns"),
        new DemoEntry("All-way stop", "Four-way stop-sign junction, first-come-first-served.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/27-allway-stop"),
        new DemoEntry("Roundabout", "Circulating traffic yields to entries per SUMO's roundabout rule.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/32-roundabout"),
        new DemoEntry("Multilane junction turn", "Turning movements across a multilane junction.",
            DemoCategory.Junctions, DemoKind.Scenario, "scenarios/44-multilane-junction-turn"),

        // -- Traffic lights -----------------------------------------------------------------------------
        new DemoEntry("Traffic light", "A single fixed-program signalized junction.",
            DemoCategory.TrafficLights, DemoKind.Scenario, "scenarios/09-traffic-light"),
        new DemoEntry("Actuated TLS", "A demand-actuated traffic-light program extending green on detection.",
            DemoCategory.TrafficLights, DemoKind.Scenario, "scenarios/35-actuated-tls"),

        // -- Lane change / overtaking / emergency / give-way ---------------------------------------------
        new DemoEntry("Overtake", "A faster vehicle overtakes a slower leader on a multilane road.",
            DemoCategory.LaneChange, DemoKind.Scenario, "scenarios/12-overtake"),
        new DemoEntry("Continuous lane-change", "A lane change spread smoothly over time (lanechange.duration > 0).",
            DemoCategory.LaneChange, DemoKind.Scenario, "scenarios/43-continuous-lanechange"),
        new DemoEntry("Overtake (oncoming)", "Overtaking that briefly borrows the opposing-direction lane.",
            DemoCategory.LaneChange, DemoKind.Scenario, "scenarios/57-overtake-opposite"),
        new DemoEntry("Emergency vehicle at red", "An emergency vehicle threading a red-signal queue.",
            DemoCategory.LaneChange, DemoKind.Scenario, "scenarios/16-emergency-red"),
        new DemoEntry("Give-way drift", "A vehicle drifts laterally within its lane to give way / let a vehicle pass.",
            DemoCategory.LaneChange, DemoKind.Scenario, "scenarios/55-giveway-drift"),

        // -- Rail ---------------------------------------------------------------------------------------
        new DemoEntry("Rail free-flow", "A single train running free-flow on rail track.",
            DemoCategory.Rail, DemoKind.Scenario, "scenarios/47-rail-free-flow"),
        new DemoEntry("Rail bidirectional meet", "Two trains meeting on bidirectionally-signalled track.",
            DemoCategory.Rail, DemoKind.Scenario, "scenarios/49-rail-bidi-meet"),
        new DemoEntry("Rail crossing", "A train crossing a road at a level crossing.",
            DemoCategory.Rail, DemoKind.Scenario, "scenarios/51-rail-crossing"),

        // -- City scale -----------------------------------------------------------------------------------
        new DemoEntry("City (organic, L2)", "An organically-grown town-scale net with its own committed demand.",
            DemoCategory.CityScale, DemoKind.Scenario, "scenarios/_bench/city-organic-L2"),
        new DemoEntry("City (mixed, 1k)", "A ~1k-vehicle mixed-demand city-scale benchmark scenario.",
            DemoCategory.CityScale, DemoKind.Scenario, "scenarios/_bench/city-mixed-1k"),
        new DemoEntry("City (15k grid, big fleet)",
            "A large 13k-edge grid net driven as a sandbox with a ~4000-vehicle fleet -- the big-net demo.",
            DemoCategory.CityScale, DemoKind.Sandbox, "scenarios/_bench/city-15000", SandboxFleet: 4000),

        // -- Sandbox (bare nets, random-traffic filled) ------------------------------------------------
        new DemoEntry("Sandbox: cross", "A single 4-way cross junction filled with random traffic.",
            DemoCategory.Sandbox, DemoKind.Sandbox, "samples/junctions/cross"),
        new DemoEntry("Sandbox: bend", "A bent (curved) road segment filled with random traffic.",
            DemoCategory.Sandbox, DemoKind.Sandbox, "samples/junctions/bend"),
        new DemoEntry("Sandbox: acute", "An acute-angle junction filled with random traffic.",
            DemoCategory.Sandbox, DemoKind.Sandbox, "samples/junctions/acute"),

        // -- Evacuation (live panic-evacuation -- Sim.Evac, EngineHost.CreateEvac) -----------------------
        new DemoEntry("Evacuation (grid TLS)",
            "Fast, legible grid net: incident -> panic -> jam -> abandon car -> foot exodus.",
            DemoCategory.Evacuation, DemoKind.Evac, "", EvacKind: "grid-tls"),
        new DemoEntry("Evacuation (organic town)",
            "Realistic organically-grown town net running the same live panic-evacuation cascade.",
            DemoCategory.Evacuation, DemoKind.Evac, "", EvacKind: "organic"),
        new DemoEntry("Evacuation (city, 10k-class)",
            "10k-vehicle-class city net: large-scale live panic evacuation and local foot exodus.",
            DemoCategory.Evacuation, DemoKind.Evac, "", EvacKind: "city"),
    };

    // Only the entries whose backing path actually exists under `repoRoot` -- so a trimmed checkout
    // (missing e.g. the `_bench` city nets) still launches with whatever demos it does have. Every
    // dropped entry is logged to stderr with the path that was missing.
    public static IReadOnlyList<DemoEntry> Resolve(string repoRoot)
    {
        var usable = new List<DemoEntry>();
        foreach (var entry in All)
        {
            if (IsUsable(entry, repoRoot, out var missingPath))
            {
                usable.Add(entry);
            }
            else
            {
                Console.Error.WriteLine($"DemoCatalog: dropping '{entry.Name}' -- missing path '{missingPath}'.");
            }
        }

        return usable;
    }

    // The net path an Evac entry's EvacKind resolves to -- MUST match EngineHost.ResolveEvacProvider's
    // own paths exactly (scenarios/evac-grid-tls, scenarios/_bench/city-organic-L2, scenarios/_bench/
    // city-15000), since this is purely a pre-flight existence check for the same file CreateEvac reads.
    public static string EvacNetPath(string evacKind, string repoRoot) => evacKind switch
    {
        "grid-tls" => Path.Combine(repoRoot, "scenarios", "evac-grid-tls", "net.net.xml"),
        "organic" => Path.Combine(repoRoot, "scenarios", "_bench", "city-organic-L2", "net.net.xml"),
        "city" => Path.Combine(repoRoot, "scenarios", "_bench", "city-15000", "net.net.xml"),
        _ => throw new ArgumentException($"Unknown evac kind '{evacKind}' (expected \"grid-tls\", \"organic\", or \"city\").",
            nameof(evacKind)),
    };

    private static bool IsUsable(DemoEntry entry, string repoRoot, out string missingPath)
    {
        if (entry.Kind == DemoKind.Evac)
        {
            var netPath = EvacNetPath(entry.EvacKind ?? throw new InvalidOperationException(
                $"Demo '{entry.Name}' has Kind.Evac but no EvacKind."), repoRoot);
            missingPath = netPath;
            return File.Exists(netPath);
        }

        // Scenario / Sandbox: PathRelToRepo names a directory containing a *.net.xml (see the convention
        // comment on DemoEntry above) -- accept either that, or (belt-and-suspenders) a direct *.net.xml
        // file path, so a future catalog entry pointing straight at a net file still resolves.
        var path = Path.Combine(repoRoot, entry.PathRelToRepo);
        missingPath = path;

        if (File.Exists(path) && path.EndsWith(".net.xml", StringComparison.OrdinalIgnoreCase))
        {
            return true;
        }

        return Directory.Exists(path) && Directory.EnumerateFiles(path, "*.net.xml").Any();
    }
}
