using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung D9 (FastDataPlane ECS readiness -- info/replication export SEAM, READINESS ONLY,
// TASKS.md line ~651): `Engine.AddExportObserver` registers an `ISimExportObserver` that is
// notified, once per active vehicle, once per Export-phase frame, from the SAME
// `Engine.EmitTrajectory` pass that builds the committed `TrajectorySet` returned by `Run()`
// (see VehicleExportSnapshot.cs/ISimExportObserver.cs's own header comments for the FDP
// `IDescriptorTranslator` shape this mirrors). This is the property the whole seam depends on:
// a subscriber must see a FAITHFUL MIRROR of the sim state `Run()` itself returns -- not a
// stale, reordered, or partial view. No `FastDataPlane`/`Fdp.Core` reference, no
// `NetworkEntityMap`/`INetworkIdAllocator`, no network transport is exercised here; this is an
// entirely in-house recording observer.
//
// Reuses `scenarios/_bench/highway-dense` (RungD1BenchmarkDeterminismTests' dense workload) for
// a short slice so the mirror guarantee is exercised at realistic concurrency (peak >= 50), not
// a near-empty road where "faithful mirror" would be a hollow guarantee.
public class RungD9ExportObserverTests
{
    private const int Steps = 120;

    [Fact]
    public void RegisteredObserver_SeesFaithfulMirrorOfTrajectorySet()
    {
        var dir = Path.Combine(RepoRoot(), "scenarios", "_bench", "highway-dense");
        var engine = new Engine();
        var observer = new RecordingObserver();
        engine.AddExportObserver(observer);

        engine.LoadScenario(
            Path.Combine(dir, "net.net.xml"),
            Path.Combine(dir, "rou.rou.xml"),
            Path.Combine(dir, "config.sumocfg"));

        var traj = engine.Run(Steps);

        // Sanity: the workload is actually dense (many concurrent vehicles), else "faithful
        // mirror" would be a hollow guarantee over an almost-empty road (mirrors RungD1's own
        // density floor).
        var perTime = new Dictionary<double, int>();
        foreach (var id in traj.VehicleIds)
        {
            foreach (var kv in traj.PointsFor(id))
            {
                perTime[kv.Value.Time] = perTime.TryGetValue(kv.Value.Time, out var c) ? c + 1 : 1;
            }
        }

        var peak = perTime.Count == 0 ? 0 : perTime.Values.Max();
        Assert.True(peak >= 50, $"benchmark scenario not dense enough: peak concurrent = {peak}");

        // The observer must have seen frame brackets.
        Assert.True(observer.FrameBeginCount > 0);
        Assert.Equal(observer.FrameBeginCount, observer.FrameEndCount);

        // Set equality: every (VehicleId, Time) pair the TrajectorySet carries was observed
        // exactly once, and the observer recorded nothing extra.
        var expectedKeys = new HashSet<(string VehicleId, double Time)>();
        foreach (var id in traj.VehicleIds)
        {
            foreach (var kv in traj.PointsFor(id))
            {
                expectedKeys.Add((id, kv.Value.Time));
            }
        }

        var observedKeys = new HashSet<(string VehicleId, double Time)>(observer.Snapshots.Keys);

        Assert.Equal(expectedKeys.Count, observer.Snapshots.Count);
        Assert.True(expectedKeys.SetEquals(observedKeys),
            "observer's (VehicleId, Time) set does not match TrajectorySet's -- missing or extra frames");

        // Per-attribute mirror: for every (VehicleId, Time) pair, the observed snapshot's
        // Lane/Pos/Speed/X/Y/Angle must match the committed TrajectoryPoint exactly (same
        // double bits -- this is the same run, so no tolerance is needed).
        foreach (var id in traj.VehicleIds)
        {
            foreach (var kv in traj.PointsFor(id))
            {
                var point = kv.Value;
                var snapshot = observer.Snapshots[(id, point.Time)];

                Assert.Equal(point.Lane, snapshot.Lane);
                Assert.Equal(point.Pos, snapshot.Pos);
                Assert.Equal(point.Speed, snapshot.Speed);
                Assert.Equal(point.X, snapshot.X);
                Assert.Equal(point.Y, snapshot.Y);
                Assert.Equal(point.Angle, snapshot.Angle);
            }
        }
    }

    // In-house recording observer -- NOT an FDP `IDescriptorTranslator`, just a test double
    // proving the seam is genuinely load-bearing (per the briefing: "a subscriber sees a
    // faithful mirror of the sim state").
    private sealed class RecordingObserver : ISimExportObserver
    {
        public readonly Dictionary<(string VehicleId, double Time), VehicleExportSnapshot> Snapshots = new();
        public int FrameBeginCount;
        public int FrameEndCount;

        public void OnVehicleExported(in VehicleExportSnapshot snapshot)
        {
            Snapshots[(snapshot.VehicleId, snapshot.Time)] = snapshot;
        }

        public void OnFrameBegin(double time) => FrameBeginCount++;

        public void OnFrameEnd(double time) => FrameEndCount++;
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
