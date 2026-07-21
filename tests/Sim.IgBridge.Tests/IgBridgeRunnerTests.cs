using System.Text;
using Sim.Core;
using Sim.IgBridge;
using Sim.Replication;
using Xunit;

namespace Sim.IgBridge.Tests;

// T0.3 (docs/IGBRIDGE-TASKS.md): the fixed-10 Hz core loop + per-entity ring buffers must tick on an
// exact 0.1 s schedule (no wall-clock coupling) and produce a byte-identical buffered sample stream
// across two independent runs (determinism; no System.Random, order-independent).
public sealed class IgBridgeRunnerTests
{
    private const int Steps = 150; // 15 s @ 10 Hz -- enough to spawn a fleet and move through junctions

    private static IgBridgeConfig BoxConfig()
    {
        var box = Path.Combine(RepoRoot(), "scenarios", "_ped", "demo_city", "box");
        return new IgBridgeConfig(Path.Combine(box, "net.xml"), Path.Combine(box, "scenario.rou.xml"))
        {
            StepLength = 0.1,
            Seed = 42,
        };
    }

    [Fact]
    public void Tick_AdvancesSimClockOnExact10HzSchedule()
    {
        var runner = new IgBridgeRunner(BoxConfig());
        for (var step = 1; step <= Steps; step++)
        {
            runner.Tick();
            // CurrentTime is begin + stepCount * stepLength -- exactly step * 0.1, no drift, no wall dt.
            Assert.Equal(step * 0.1, runner.SimTime, precision: 9);
            Assert.Equal(step, runner.StepCount);
        }
    }

    [Fact]
    public void Run_IsDeterministic_TwoRunsProduceIdenticalBufferedStream()
    {
        var a = Fingerprint(RunN(Steps));
        var b = Fingerprint(RunN(Steps));

        Assert.False(string.IsNullOrEmpty(a));
        Assert.Equal(a, b);
    }

    [Fact]
    public void Run_ActuallySpawnsAndBuffersVehicles()
    {
        var runner = RunN(Steps);
        // The box demand departs a vehicle roughly every ~0.8 s, so 15 s must have produced a real fleet
        // with non-trivial buffered histories -- otherwise the "deterministic" stream above is vacuous.
        Assert.True(runner.VehicleHistories.Count >= 10, $"only {runner.VehicleHistories.Count} entities buffered");
        Assert.All(runner.VehicleHistories.Values, h => Assert.True(h.Count >= 1));
        Assert.Contains(runner.VehicleHistories.Values, h => h.Count >= 2); // at least one entity has motion
    }

    [Fact]
    public void Run_ActuallySpawnsAndBuffersPeds()
    {
        var runner = RunN(Steps);
        // PedStream's population target is 40 (IgBridgeConfig.PedPopulationTarget default); after 15 s
        // the steady-state population should be well established -- assert the task's >= ~20 floor.
        Assert.True(runner.LivePeds.Count >= 20, $"only {runner.LivePeds.Count} peds alive");

        // At least one ped must have >= 2 buffered samples with ACTUAL movement (position changed) --
        // otherwise the ped population would be vacuously "alive" while frozen in place.
        var moved = false;
        foreach (var id in runner.LivePeds)
        {
            var h = runner.PedHistories[id];
            if (h.Count < 2)
            {
                continue;
            }

            var first = h[0];
            var last = h[h.Count - 1];
            if (Math.Abs(first.X - last.X) > 1e-9 || Math.Abs(first.Y - last.Y) > 1e-9)
            {
                moved = true;
                break;
            }
        }

        Assert.True(moved, "no live ped showed any movement across its buffered samples");
    }

    [Fact]
    public void Run_IsDeterministic_TwoRunsProduceIdenticalPedStream()
    {
        var a = PedFingerprint(RunN(Steps));
        var b = PedFingerprint(RunN(Steps));

        Assert.False(string.IsNullOrEmpty(a));
        Assert.Equal(a, b);
    }

    private static string PedFingerprint(IgBridgeRunner runner)
    {
        var fp = new StringBuilder();
        // id-sorted (numeric, not string, so "p10" doesn't sort before "p2") for a stable fingerprint.
        foreach (var id in runner.PedHistories.Keys.OrderBy(id => id))
        {
            fp.Append(IgBridgeRunner.PedIdOf(id)).Append('|');
            var h = runner.PedHistories[id];
            for (var i = 0; i < h.Count; i++)
            {
                var s = h[i];
                fp.Append(s.TimestampSeconds.ToString("R")).Append(',')
                  .Append(s.X.ToString("R")).Append(',')
                  .Append(s.Y.ToString("R")).Append(';');
            }

            fp.Append('\n');
        }

        return fp.ToString();
    }

    private static IgBridgeRunner RunN(int steps)
    {
        var runner = new IgBridgeRunner(BoxConfig());
        for (var i = 0; i < steps; i++)
        {
            runner.Tick();
        }

        return runner;
    }

    private static string Fingerprint(IgBridgeRunner runner)
    {
        var fp = new StringBuilder();
        foreach (var kv in runner.VehicleHistories.OrderBy(k => runner.IdOf(k.Key), StringComparer.Ordinal))
        {
            fp.Append(runner.IdOf(kv.Key)).Append('|');
            var h = kv.Value;
            for (var i = 0; i < h.Count; i++)
            {
                var s = h[i];
                fp.Append(s.TimestampSeconds.ToString("F4")).Append(',')
                  .Append(s.Record.LaneHandle).Append(',')
                  .Append(s.Record.Pos.ToString("R")).Append(',')
                  .Append(s.Record.PosLat.ToString("R")).Append(',')
                  .Append(s.Record.Speed.ToString("R")).Append(';');
            }

            fp.Append('\n');
        }

        return fp.ToString();
    }

    private static string RepoRoot()
    {
        var d = new DirectoryInfo(AppContext.BaseDirectory);
        while (d is not null && !File.Exists(Path.Combine(d.FullName, "Traffic.sln")))
        {
            d = d.Parent;
        }

        return d?.FullName ?? throw new InvalidOperationException("repo root not found");
    }
}
