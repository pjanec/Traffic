using System.Diagnostics;
using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-API.md §7: the async SimulationRunner -- command dispatcher applied at the Tick boundary +
// immutable published SimulationSnapshot. Tested deterministically via manual Tick(), plus one threaded
// smoke test for the background loop.
public class RungB12AsyncRunnerTests
{
    private static string Net14 => Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle", "net.net.xml");

    // A runner Tick()'d N times produces the SAME vehicle trajectory as the bare engine Step()'d N times,
    // and the published snapshot faithfully mirrors the engine's read state -- proving Tick == Step + a
    // correct snapshot copy, and that boundary command application is identity when there are no commands.
    [Fact]
    public void Tick_MatchesBareEngineStep_AndSnapshotIsFaithful()
    {
        var bare = new Engine();
        bare.LoadNetwork(Net14);
        var hb = bare.SpawnVehicle(bare.DefaultVType, new[] { "e0" });

        var eng = new Engine();
        eng.LoadNetwork(Net14);
        var runner = new SimulationRunner(eng);
        var hr = runner.Invoke(e => e.SpawnVehicle(e.DefaultVType, new[] { "e0" })); // inline (not started)

        for (var k = 0; k < 30; k++)
        {
            bare.Step();
            runner.Tick();

            var bareHas = bare.TryGetVehicle(hb, out var sb);
            var snap = runner.Snapshot;
            var snapHas = snap.TryGetVehicle(hr, out var ss);

            Assert.Equal(bareHas, snapHas);
            if (bareHas)
            {
                Assert.Equal(sb.Pos, ss.Pos);       // bit-exact
                Assert.Equal(sb.Speed, ss.Speed);
                Assert.Equal(sb.LaneId, ss.LaneId);
            }

            Assert.Equal(bare.StepCount, snap.StepCount);
        }
    }

    // A command Post()'d before a Tick is applied at that Tick's boundary (before the Step).
    [Fact]
    public void PostedCommand_AppliedAtBoundary()
    {
        var eng = new Engine();
        eng.LoadNetwork(Net14);
        var runner = new SimulationRunner(eng);

        var h = runner.Invoke(e => e.SpawnVehicle(e.DefaultVType, new[] { "e0" }));
        runner.Tick();
        Assert.True(runner.Snapshot.TryGetVehicle(h, out _)); // active

        runner.Post(e => e.Despawn(h));
        runner.Tick(); // drains the despawn, then steps
        Assert.False(runner.Snapshot.TryGetVehicle(h, out _)); // gone
    }

    // The published snapshot carries the per-frame lifecycle events.
    [Fact]
    public void Snapshot_CarriesLifecycleEvents()
    {
        var eng = new Engine();
        eng.LoadNetwork(Net14);
        var runner = new SimulationRunner(eng);

        var h = runner.Invoke(e => e.SpawnVehicle(e.DefaultVType, new[] { "e0" }));
        runner.Tick();

        var snap = runner.Snapshot;
        var departed = false;
        for (var i = 0; i < snap.EventCount; i++)
        {
            if (snap.Events[i].Handle == h && snap.Events[i].Kind == SimEventKind.Departed)
            {
                departed = true;
            }
        }

        Assert.True(departed, "expected a Departed event in the published snapshot");
    }

    // Threaded smoke: the background loop steps the sim; a spawned vehicle appears in the published
    // snapshot, and a posted despawn removes it -- all observed cross-thread.
    [Fact]
    public void BackgroundLoop_SpawnAppears_ThenDespawnRemoves()
    {
        var eng = new Engine();
        eng.LoadNetwork(Net14);
        using var runner = new SimulationRunner(eng);
        runner.Start(targetHz: 500.0);
        try
        {
            var h = runner.Invoke(e => e.SpawnVehicle(e.DefaultVType, new[] { "e0" }));

            Assert.True(WaitUntil(() => runner.Snapshot.TryGetVehicle(h, out _), 3000),
                "spawned vehicle never appeared in the published snapshot");

            runner.Post(e => e.Despawn(h));
            Assert.True(WaitUntil(() => !runner.Snapshot.TryGetVehicle(h, out _), 3000),
                "despawned vehicle never left the published snapshot");

            Assert.Null(runner.LastError);
        }
        finally
        {
            runner.Stop();
        }
    }

    private static bool WaitUntil(Func<bool> condition, int timeoutMs)
    {
        var sw = Stopwatch.StartNew();
        while (sw.ElapsedMilliseconds < timeoutMs)
        {
            if (condition())
            {
                return true;
            }

            Thread.Sleep(5);
        }

        return condition();
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
