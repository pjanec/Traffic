using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung F2b test: a FILE snapshot taken while a probabilistic <flow> is still generating must
// round-trip deterministically -- the restored run continues the SAME arrival stream (same future
// "<flowId>.<k>" vehicles, same kinematics) as an uninterrupted in-memory run. This requires the
// snapshot to carry (a) each flow's per-flow RNG state + arrival counter, and (b) the already-
// generated flow vehicles themselves (they have no template in the demand), reconstructed at their
// original EntityIndex so future arrivals seed identically. Validated engine-vs-engine (no SUMO
// golden): save->fresh load->continue must equal a single WarmUp+Run over the same horizon.
public class RungF2bProbabilisticFlowSnapshotTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "56-flow-equiv");

    private static Engine Load()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "prob.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        return engine;
    }

    [Fact]
    public void Snapshot_RoundTrips_AndContinuesTheFlowStream_Deterministically()
    {
        const int W = 30, N = 50;

        // Reference: one continuous engine -- warm up W steps (populating from the flow), then run N.
        var reference = Load();
        reference.WarmUp(W);
        var refTail = reference.Run(N);

        // Snapshot path: warm up W, save, then restore into a FRESH engine and run N.
        var saver = Load();
        saver.WarmUp(W);
        var path = Path.Combine(Path.GetTempPath(), $"f2b_{Guid.NewGuid():N}.snapshot.xml");
        try
        {
            saver.SaveSnapshot(path);

            var restored = Load();
            restored.LoadSnapshot(path);
            var loadTail = restored.Run(N);

            // Same set of vehicles emitted over the continuation, and it is non-trivial.
            Assert.Equal(refTail.VehicleIds.OrderBy(x => x), loadTail.VehicleIds.OrderBy(x => x));
            Assert.True(refTail.VehicleIds.Count >= 2, $"expected several vehicles in the continuation, got {refTail.VehicleIds.Count}");

            // At least one vehicle carried OVER the snapshot boundary (generated during warm-up), and
            // at least one was generated AFTER restore (a new arrival "<flowId>.<k>"): proves both the
            // restored population and the continued stream.
            var minId = refTail.VehicleIds.Select(id => int.Parse(id.Split('.')[1])).Min();
            var maxId = refTail.VehicleIds.Select(id => int.Parse(id.Split('.')[1])).Max();
            Assert.True(maxId > minId, "expected both carried-over and newly-generated flow vehicles");

            // Point-for-point identical trajectory across the whole continuation.
            foreach (var id in refTail.VehicleIds)
            {
                var rp = refTail.PointsFor(id);
                var lp = loadTail.PointsFor(id);
                Assert.Equal(rp.Count, lp.Count);
                foreach (var (time, p) in rp)
                {
                    Assert.True(lp.ContainsKey(time), $"{id}: restored run missing time {time}");
                    Assert.Equal(p.Lane, lp[time].Lane);
                    Assert.Equal(p.Pos, lp[time].Pos, precision: 9);
                    Assert.Equal(p.Speed, lp[time].Speed, precision: 9);
                }
            }
        }
        finally
        {
            if (File.Exists(path))
            {
                File.Delete(path);
            }
        }
    }

    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
