using System.Text;
using Sim.Core;
using Sim.IgBridge;
using Sim.Replication;

// IgBridge verification host (docs/IGBRIDGE-TASKS.md). T0.3 smoke: drive the fixed-10 Hz runner over the
// box scenario, report entity/tick stats, and prove the buffered sample stream is deterministic across
// two runs. Reconstruct/emit/FakeIg/render/metrics land in T1.2+.
var repoRoot = FindRepoRoot();
var boxDir = Path.Combine(repoRoot, "scenarios", "_ped", "demo_city", "box");
var cfg = new IgBridgeConfig(Path.Combine(boxDir, "net.xml"), Path.Combine(boxDir, "scenario.rou.xml"))
{
    StepLength = 0.1,
    Seed = 42,
};

const int Steps = 1200; // 120 s @ 10 Hz

Console.WriteLine($"box={boxDir}");
var (fingerprint, stats) = RunAndFingerprint(cfg, Steps, verbose: true);
Console.WriteLine(stats);

// Determinism: a second independent run must produce a byte-identical buffered-sample fingerprint.
var (fingerprint2, _) = RunAndFingerprint(cfg, Steps, verbose: false);
Console.WriteLine(fingerprint == fingerprint2
    ? $"DETERMINISM OK: two runs identical (fingerprint len={fingerprint.Length}, hash={fingerprint.GetHashCode():X8})"
    : "DETERMINISM FAILED: runs diverged");

return 0;

static (string fingerprint, string stats) RunAndFingerprint(IgBridgeConfig cfg, int steps, bool verbose)
{
    var runner = new IgBridgeRunner(cfg);
    var sb = new StringBuilder();
    int maxLive = 0, totalSpawned = 0, totalDespawned = 0;
    int maxLivePeds = 0, totalPedSpawned = 0, totalPedDespawned = 0;
    double firstTickTime = double.NaN, lastTickTime = double.NaN;

    for (var step = 0; step < steps; step++)
    {
        runner.Tick();
        if (step == 0) firstTickTime = runner.SimTime;
        lastTickTime = runner.SimTime;
        maxLive = Math.Max(maxLive, runner.LiveVehicles.Count);
        totalSpawned += runner.SpawnedThisTick.Count;
        totalDespawned += runner.DespawnedThisTick.Count;
        maxLivePeds = Math.Max(maxLivePeds, runner.LivePeds.Count);
        totalPedSpawned += runner.PedSpawnedThisTick.Count;
        totalPedDespawned += runner.PedDespawnedThisTick.Count;
    }

    // Fingerprint: for every buffered sample of every entity (id-sorted), fold in the quantized state.
    // This is what the determinism check compares -- any divergence in timing or motion flips it.
    var fp = new StringBuilder();
    foreach (var kv in runner.VehicleHistories.OrderBy(k => runner.IdOf(k.Key), StringComparer.Ordinal))
    {
        fp.Append(runner.IdOf(kv.Key)).Append('|');
        var h = kv.Value;
        for (var i = 0; i < h.Count; i++)
        {
            var s = h[i];
            fp.Append(s.TimestampSeconds.ToString("F3")).Append(',')
              .Append(s.Record.LaneHandle).Append(',')
              .Append(s.Record.Pos.ToString("F4")).Append(',')
              .Append(s.Record.PosLat.ToString("F4")).Append(',')
              .Append(s.Record.Speed.ToString("F4")).Append(';');
        }
        fp.Append('\n');
    }

    if (verbose)
    {
        sb.AppendLine($"ticks={steps} firstTickTime={firstTickTime:F3} lastTickTime={lastTickTime:F3} "
            + $"(expect first=0.100, last={0.1 * steps:F3})");
        sb.AppendLine($"maxLiveVehicles={maxLive} totalSpawned={totalSpawned} totalDespawned={totalDespawned} "
            + $"pendingDemand={runner.PendingDemand} distinctEntities={runner.VehicleHistories.Count}");
        sb.AppendLine($"livePeds={runner.LivePeds.Count} maxLivePeds={maxLivePeds} "
            + $"totalPedSpawned={totalPedSpawned} totalPedDespawned={totalPedDespawned} "
            + $"distinctPeds={runner.PedHistories.Count}");
    }

    return (fp.ToString(), sb.ToString());
}

static string FindRepoRoot()
{
    var d = new DirectoryInfo(AppContext.BaseDirectory);
    while (d is not null && !File.Exists(Path.Combine(d.FullName, "Traffic.sln"))) d = d.Parent;
    return d?.FullName ?? throw new InvalidOperationException("repo root not found");
}
