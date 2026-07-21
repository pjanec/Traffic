using Sim.IgBridge;

// IgBridge verification host (docs/IGBRIDGE-TASKS.md). Drives the fixed-10 Hz runner over the box scenario,
// emits the IG trace, and runs the FakeIg + metrics pass: raw engine motion vs IgBridge-smoothed, both
// reconstructed through the IG's 2-sample rule, so "no artifacts" is MEASURED (R8/§9.3). Side-by-side
// render is T1.4.
var repoRoot = FindRepoRoot();
// Scenario is selectable via IGBRIDGE_SCENARIO (a path under scenarios/). Default: the clean 6x6 grid
// "subarea-box" -- a Manhattan grid with explicit turning routes and clean edge-start spawns, a far better
// smoothness test bed than the demo_city "box" (which spawns near junctions and has no clean turns).
var scenarioRel = Environment.GetEnvironmentVariable("IGBRIDGE_SCENARIO") ?? "_ped/subarea-box";
var scDir = Path.Combine(repoRoot, "scenarios", scenarioRel.Replace('/', Path.DirectorySeparatorChar));
var netPath = FindScenarioFile(scDir, "net.xml", "*.net.xml");
var rouPath = FindScenarioFile(scDir, "scenario.rou.xml", "*.rou.xml");
// Synthetic pedestrian crowd off by default (IGBRIDGE_PEDS=1 to enable); the grid has no scenario peds and
// the focus here is vehicle turn smoothness.
var enablePeds = Environment.GetEnvironmentVariable("IGBRIDGE_PEDS") == "1";
var cfg = new IgBridgeConfig(netPath, rouPath)
{
    StepLength = 0.1,
    Seed = 42,
    EnablePeds = enablePeds,
};
Console.WriteLine($"scenario={scenarioRel}  net={Path.GetFileName(netPath)}  rou={Path.GetFileName(rouPath)}  peds={enablePeds}");
var emit = new IgEmitConfig { EmitHz = 20.0, LookaheadSeconds = 0.1 };
var igCfg = new FakeIgConfig { DelaySeconds = 0.75, JumpThresholdMeters = 8.0, RenderHz = 60.0 };

var Steps = int.TryParse(Environment.GetEnvironmentVariable("IGBRIDGE_STEPS"), out var _st) ? _st : 1200; // 120 s @ 10 Hz

var outDir = Path.Combine(repoRoot, "artifacts", "igbridge");
Directory.CreateDirectory(outDir);
var tracePath = Path.Combine(outDir, "trace.jsonl");

Console.WriteLine($"scenarioDir={scDir}");

// One run: write the smoothed trace, retain the smoothed stream in memory, and collect the raw engine
// stream (10 Hz x/y/angle) as the metrics "before" baseline.
var runner = new IgBridgeRunner(cfg);
var raw = new RawStreamCollector();
IReadOnlyList<IgSample> smoothed;
using (var trace = new IgTraceWriter(tracePath))
{
    // Sweep knobs (env): position + heading smoothing times for the kinematic model.
    double EnvD(string k, double def) =>
        double.TryParse(Environment.GetEnvironmentVariable(k), System.Globalization.CultureInfo.InvariantCulture, out var v) ? v : def;
    var kin = new Sim.Viewer.Motion.KinematicHeadingParams
    {
        PositionSmoothTime = EnvD("IGBRIDGE_POS_SMOOTH", 0.40),
        HeadingSmoothTime = EnvD("IGBRIDGE_HEAD_SMOOTH", 0.0),
    };
    var session = new IgBridgeSession(runner, emit, trace, retainAll: true, kinematics: kin);
    session.DebugVehicleId = Environment.GetEnvironmentVariable("IGBRIDGE_DEBUG_VEH"); // e.g. "v2"
    for (var step = 0; step < Steps; step++)
    {
        runner.Tick();
        raw.Ingest(runner);
        session.Advance();
    }

    session.Finish();
    if (session.DebugVehicleId is not null && session.DebugRows.Count > 0)
    {
        var dbgPath = Path.Combine(outDir, $"debug_{session.DebugVehicleId}.csv");
        var header = "t,prX,prY,prH,smX,smY,cX,cY,cH,rearX,rearY,speed";
        File.WriteAllLines(dbgPath, new[] { header }.Concat(session.DebugRows));
        Console.WriteLine($"debug rows for {session.DebugVehicleId}: {session.DebugRows.Count} -> {dbgPath}");
    }
    raw.Finish(runner.SimTime);
    smoothed = session.AllEmitted;

    // Also persist the raw stream for diffing (T1.5 diagnosis).
    using (var rawTrace = new IgTraceWriter(Path.Combine(outDir, "raw.jsonl")))
    {
        foreach (var s in raw.Samples) rawTrace.Write(s);
    }
    Console.WriteLine($"ticks={Steps} simTime={runner.SimTime:F1}s emitted={session.EmittedCount} "
        + $"raw-veh-samples={raw.Samples.Count}");
    Console.WriteLine($"trace: {tracePath}");
}

// Reconstruct both streams through the FakeIg, then compare VEHICLE smoothness (the artifacts are vehicle
// junction turns + lane changes; peds are smooth by construction in both).
var rawIg = new FakeIg(raw.Samples, igCfg);
var smIg = new FakeIg(smoothed, igCfg);

var rawVeh = rawIg.Reconstruct();
var smVeh = OnlyModel(smIg, smIg.Reconstruct(), IgEntityModel.Car);
var smPed = OnlyModel(smIg, smIg.Reconstruct(), IgEntityModel.Ped);

var rawStats = Metrics.Compute(rawVeh, igCfg.RenderHz);
var smStats = Metrics.Compute(smVeh, igCfg.RenderHz);

Console.WriteLine();
Console.WriteLine($"=== VEHICLE smoothness: raw engine @10Hz  vs  IgBridge-smoothed @{emit.EmitHz}Hz "
    + $"(both reconstructed via FakeIg @{igCfg.RenderHz}Hz, delay={igCfg.DelaySeconds}s) ===");
Console.Write(Metrics.FormatComparison(rawStats, smStats));
Console.WriteLine();
Console.WriteLine("interpretation: median = the typical vehicle (junction-turn faceting -> yaw-rate/jerk");
Console.WriteLine("  MEDIAN drops sharply); mean lat-accel drops (instant lane changes smoothed); max/p95");
Console.WriteLine("  reflect genuine hairpin U-turns present in the RAW engine stream too (not an artifact).");
Console.WriteLine($"(pedestrians, reconstructed: {smPed.Count} tracks -- smooth by construction)");

// T1.4: side-by-side HTML render (raw-fed-IG vs IgBridge-fed-IG). Reconstruct for the viz at a display
// frame rate over a focused window so the file stays manageable and the motion is watchable.
var vizCfg = new FakeIgConfig { DelaySeconds = 0.75, JumpThresholdMeters = 8.0, RenderHz = 15.0 };
var rawVizIg = new FakeIg(raw.Samples, vizCfg);
var smVizIg = new FakeIg(smoothed, vizCfg);
var htmlPath = Path.Combine(outDir, "sidebyside.html");
VizExport.WriteSideBySide(
    repoRoot, runner.Network,
    ("raw (IG fed raw 10Hz)", "engine x/y/angle at 10 Hz -- junction snaps + instant lane changes", rawVizIg),
    ("IgBridge (IG fed smoothed 20Hz)", "reused DrClock/PoseResolver/DrPoseSmoother reconstruction", smVizIg),
    startT: 20.0, endT: 80.0, fps: 15.0, htmlPath);
Console.WriteLine($"render: {htmlPath}  (toggle the two scenes to compare)");

return 0;

// Keep only the reconstructed streams whose entity model matches.
static IReadOnlyDictionary<string, IReadOnlyList<ReconPose>> OnlyModel(
    FakeIg ig, IReadOnlyDictionary<string, IReadOnlyList<ReconPose>> all, IgEntityModel model)
{
    var result = new Dictionary<string, IReadOnlyList<ReconPose>>();
    foreach (var kv in all)
    {
        if (ig.ModelOf(kv.Key) == model)
        {
            result[kv.Key] = kv.Value;
        }
    }

    return result;
}

// Resolve a scenario input file: prefer the exact conventional name, else the first glob match.
static string FindScenarioFile(string dir, string exact, string glob)
{
    var p = Path.Combine(dir, exact);
    if (File.Exists(p)) return p;
    var hits = Directory.GetFiles(dir, glob);
    if (hits.Length > 0) return hits[0];
    throw new FileNotFoundException($"no {exact} or {glob} in {dir}");
}

static string FindRepoRoot()
{
    var d = new DirectoryInfo(AppContext.BaseDirectory);
    while (d is not null && !File.Exists(Path.Combine(d.FullName, "Traffic.sln"))) d = d.Parent;
    return d?.FullName ?? throw new InvalidOperationException("repo root not found");
}

// Builds the RAW vehicle IG stream (unsmoothed engine x/y/z/angle at the core 10 Hz cadence) with the same
// new/upd/del lifecycle as the smoothed stream, straight from the runner's per-tick raw poses.
internal sealed class RawStreamCollector
{
    private readonly HashSet<string> _seen = new();
    private HashSet<string> _prevLive = new();

    public List<IgSample> Samples { get; } = new();

    public void Ingest(IgBridgeRunner runner)
    {
        var t = runner.SimTime;
        var live = new HashSet<string>();
        foreach (var r in runner.RawVehiclePosesThisTick)
        {
            live.Add(r.Id);
            Samples.Add(_seen.Add(r.Id)
                ? IgSample.Created(r.Id, t, IgEntityModel.Car, r.X, r.Y, r.Z, r.HeadingDeg)
                : IgSample.Updated(r.Id, t, r.X, r.Y, r.Z, r.HeadingDeg));
        }

        foreach (var id in _prevLive)
        {
            if (!live.Contains(id))
            {
                Samples.Add(IgSample.Removed(id, t));
            }
        }

        _prevLive = live;
    }

    public void Finish(double t)
    {
        foreach (var id in _prevLive)
        {
            Samples.Add(IgSample.Removed(id, t));
        }
    }
}
