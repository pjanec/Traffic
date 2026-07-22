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
// Direction 1 (longer-vehicle probe): IGBRIDGE_BUS_IDS = comma list of demand ids (with or without the "v"
// prefix) to spawn as a bus; IGBRIDGE_BUS_LEN = its length (m). Empty list = byte-identical v4 (all cars).
var busIds = (Environment.GetEnvironmentVariable("IGBRIDGE_BUS_IDS") ?? "")
    .Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
var busLen = double.TryParse(Environment.GetEnvironmentVariable("IGBRIDGE_BUS_LEN"),
    System.Globalization.CultureInfo.InvariantCulture, out var _bl) ? _bl : 12.0;
// IGBRIDGE_BUS_VCLASS: bus (12 m) | coach (14 m) | truck (7.1 m) | trailer (16.5 m rigid). Length still
// from IGBRIDGE_BUS_LEN. (No true articulation -- SUMO/SumoSharp model one rigid length; see docs.)
var busVClass = Environment.GetEnvironmentVariable("IGBRIDGE_BUS_VCLASS")?.Trim() is { Length: > 0 } bvc ? bvc : "bus";
// Feed rate (Hz) sampled into the reconstruction + raw stream. Core always steps at 10 Hz; this decimates
// the FEED. IGBRIDGE_FEED_HZ=1 => sample every 10th tick (a 1 Hz shipped-update test). Default 10 (every tick).
var feedHz = double.TryParse(Environment.GetEnvironmentVariable("IGBRIDGE_FEED_HZ"),
    System.Globalization.CultureInfo.InvariantCulture, out var _fh) && _fh > 0 ? _fh : 10.0;
var feedN = Math.Max(1, (int)Math.Round(10.0 / feedHz));
var cfg = new IgBridgeConfig(netPath, rouPath)
{
    StepLength = 0.1,
    Seed = 42,
    EnablePeds = enablePeds,
    BusVehicleIds = busIds,
    BusLengthMeters = busLen,
    BusVClass = busVClass,
    FeedSampleEveryN = feedN,
};
Console.WriteLine($"scenario={scenarioRel}  net={Path.GetFileName(netPath)}  rou={Path.GetFileName(rouPath)}  peds={enablePeds}"
    + (busIds.Length > 0 ? $"  buses=[{string.Join(",", busIds)}]@{busLen}m" : ""));
// IgBridge output rate to the IG. Default 20 Hz; IGBRIDGE_EMIT_HZ=10 tests a coarser emit into the FakeIg.
var emitHz = double.TryParse(Environment.GetEnvironmentVariable("IGBRIDGE_EMIT_HZ"),
    System.Globalization.CultureInfo.InvariantCulture, out var _eh) && _eh > 0 ? _eh : 20.0;
// The reconstruction query must stay behind the newest BUFFERED sample, which at a decimated feed lags
// SimTime by up to one feed interval. So the emit lookahead tracks the feed interval (0.1 s at 10 Hz -- the
// unchanged default; 1.0 s at a 1 Hz feed). Without this, a coarse-feed query runs past `newest` and the
// emitter spuriously retires still-live vehicles.
var lookaheadSec = Math.Max(0.1, feedN * 0.1);
var emit = new IgEmitConfig { EmitHz = emitHz, LookaheadSeconds = lookaheadSec };
Console.WriteLine($"feedHz={10.0 / feedN:F1} (sample every {feedN} ticks, lookahead {lookaheadSec:F1}s)  emitHz={emitHz:F1}");
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
        LaneChangeDecayTau = EnvD("IGBRIDGE_LC_TAU", 2.0),
        HeadingSmoothTime = EnvD("IGBRIDGE_HEAD_SMOOTH", 0.0),
        // Anticipatory turn-in (wider line on sharp corners); value = heading smoothing time (s). 0 disables.
        // Off by default — the lane-heading predictor already gives an in-lane driver's line.
        TurnInSmoothTime = EnvD("IGBRIDGE_TURNIN", 0.0),
        PositionSmoothTime = EnvD("IGBRIDGE_POS_SMOOTH", 0.60),
        LanePredictSmoothTime = EnvD("IGBRIDGE_LANEPRED", 0.18),
    };
    var session = new IgBridgeSession(runner, emit, trace, retainAll: true, kinematics: kin)
    {
        // Spatial look-ahead: aim the front predictor at a point this far ahead on the upcoming lane
        // centerline, so junction turn-ins hit the connecting lane instead of lagging off it. 0 disables.
        LookAheadMeters = EnvD("IGBRIDGE_LOOKAHEAD", 3.0),
        // Longer vehicles anticipate proportionally further: effective look-ahead = max(LookAheadMeters,
        // factor*length). 0.5 keeps ~5 m cars on the flat 3 m (byte-identical v4) and gives a 12 m bus ~6 m.
        LookAheadLengthFactor = EnvD("IGBRIDGE_LOOKAHEAD_LENFAC", 0.5),
        // Reject a look-ahead that diverges more than this from the reactive lane heading (kills the
        // coarse-feed "dance" where the look-ahead drifts to a wrong bearing). 70° > the ~60° legit max at
        // a 10 Hz feed, so the default is byte-identical to v5.
        MaxAnticipationLeadDeg = (float)EnvD("IGBRIDGE_MAX_LEAD", 70.0),
        // At a decimated feed, don't absorb junction-turn straddles as lane changes (they'd ride between
        // lanes for seconds after a turn). No-op at the dense default, so v5 stays byte-identical.
        CoarseFeed = feedN > 1,
    };
    // IGBRIDGE_DEBUG_VEH: one id or a comma-separated list (e.g. "v18,v98,v213,v321"); each gets its own CSV.
    var dbgIds = (Environment.GetEnvironmentVariable("IGBRIDGE_DEBUG_VEH") ?? "")
        .Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
    if (dbgIds.Length > 0)
    {
        session.DebugVehicleIds = new HashSet<string>(dbgIds);
    }
    for (var step = 0; step < Steps; step++)
    {
        runner.Tick();
        // Decimate the raw comparison stream to the same feed rate as the reconstruction (fair A/B).
        if (runner.IsSampleTick)
        {
            raw.Ingest(runner);
        }
        session.Advance();
    }

    session.Finish();
    {
        var header = "t,prX,prY,prH,smX,smY,cX,cY,cH,rearX,rearY,speed,laH";
        foreach (var kv in session.DebugRowsById)
        {
            var dbgPath = Path.Combine(outDir, $"debug_{kv.Key}.csv");
            File.WriteAllLines(dbgPath, new[] { header }.Concat(kv.Value));
            Console.WriteLine($"debug rows for {kv.Key}: {kv.Value.Count} -> {dbgPath}");
        }
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

// Per-vehicle footprint (id -> length,width) so the viewer draws a bus long and a car short (Direction 1).
var vehDimsById = new Dictionary<string, (double Length, double Width)>();
foreach (var kv in runner.VehicleDims)
{
    vehDimsById[runner.IdOf(kv.Key)] = kv.Value;
}

var htmlPath = Path.Combine(outDir, "sidebyside.html");

// IGBRIDGE_EXTRAP=1: compare the two IG CONSUMPTION models on the SAME IgBridge stream -- a buffered
// INTERPOLATING IG (0.75 s playout delay, needs the next sample) vs a zero-latency EXTRAPOLATING IG (no
// delay, dead-reckons forward from the last received sample). This shows the low-latency default's cost.
if (Environment.GetEnvironmentVariable("IGBRIDGE_EXTRAP") == "1")
{
    var interpCfg = new FakeIgConfig { DelaySeconds = 0.75, JumpThresholdMeters = 8.0, RenderHz = 15.0 };
    var extrapCfg = new FakeIgConfig { JumpThresholdMeters = 8.0, RenderHz = 15.0, Extrapolate = true };
    var igInterp = new FakeIg(smoothed, interpCfg);
    var igExtrap = new FakeIg(smoothed, extrapCfg);

    // DR cost: how far the zero-delay extrapolation sits from the buffered-interpolation pose, per render
    // sample (the lead/snap error). Metric-grade FakeIgs at 60 Hz over the whole run.
    var mInterp = new FakeIg(smoothed, new FakeIgConfig { RenderHz = 60.0 });
    var mExtrap = new FakeIg(smoothed, new FakeIgConfig { RenderHz = 60.0, Extrapolate = true });
    var iRec = OnlyModel(mInterp, mInterp.Reconstruct(), IgEntityModel.Car);
    var eRec = OnlyModel(mExtrap, mExtrap.Reconstruct(), IgEntityModel.Car);
    var drErr = new List<double>();
    foreach (var kv in iRec)
    {
        if (!eRec.TryGetValue(kv.Key, out var ep)) continue;
        var ip = kv.Value;
        var n = Math.Min(ip.Count, ep.Count);
        for (var i = 0; i < n; i++)
        {
            drErr.Add(Math.Sqrt((ip[i].X - ep[i].X) * (ip[i].X - ep[i].X) + (ip[i].Y - ep[i].Y) * (ip[i].Y - ep[i].Y)));
        }
    }

    drErr.Sort();
    var eStats = Metrics.Compute(OnlyModel(igExtrap, igExtrap.Reconstruct(), IgEntityModel.Car), 15.0);
    var iStats = Metrics.Compute(OnlyModel(igInterp, igInterp.Reconstruct(), IgEntityModel.Car), 15.0);
    Console.WriteLine();
    Console.WriteLine($"=== IG consumption: INTERPOLATING (0.75s delay)  vs  EXTRAPOLATING (zero delay, DR) "
        + $"on the same IgBridge @{emit.EmitHz}Hz stream ===");
    Console.Write(Metrics.FormatComparison(iStats, eStats));
    if (drErr.Count > 0)
    {
        Console.WriteLine($"DR position error (extrapolated vs interpolated), m: "
            + $"median={drErr[drErr.Count / 2]:F3} p95={drErr[(int)(0.95 * drErr.Count)]:F3} max={drErr[^1]:F3}");
        Console.WriteLine($"  (this is the zero-latency 'lead'/snap; ~speed*emitInterval on turns -- shrinks with a higher emit rate)");
    }

    VizExport.WriteSideBySide(
        repoRoot, runner.Network,
        ("IgBridge -> interpolating IG (0.75s delay)", $"buffered 2-sample interp of the {emit.EmitHz:F0} Hz stream", igInterp),
        ("IgBridge -> extrapolating IG (zero delay, DR)", $"dead-reckon fwd from last sample, no buffer", igExtrap),
        startT: 20.0, endT: 80.0, fps: 15.0, htmlPath, vehDimsById);
    Console.WriteLine($"render: {htmlPath}  (toggle: interpolating vs extrapolating IG)");
    return 0;
}

// T1.4: side-by-side HTML render (raw-fed-IG vs IgBridge-fed-IG). Reconstruct for the viz at a display
// frame rate over a focused window so the file stays manageable and the motion is watchable.
var vizCfg = new FakeIgConfig { DelaySeconds = 0.75, JumpThresholdMeters = 8.0, RenderHz = 15.0 };
var rawVizIg = new FakeIg(raw.Samples, vizCfg);
var smVizIg = new FakeIg(smoothed, vizCfg);

VizExport.WriteSideBySide(
    repoRoot, runner.Network,
    ("raw (IG fed raw 10Hz)", "engine x/y/angle at 10 Hz -- junction snaps + instant lane changes", rawVizIg),
    ("IgBridge (IG fed smoothed 20Hz)", "reused DrClock/PoseResolver/DrPoseSmoother reconstruction", smVizIg),
    startT: 20.0, endT: 80.0, fps: 15.0, htmlPath, vehDimsById);
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
