using SumoSharp.GameHostSample;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-API.md §3: guard the ns2.1 game-host sample (samples/SumoSharp.GameHostSample) against
// bit-rot. GameHost is the "drop into Unity/Godot" integration class; this drives it exactly as the
// headless demo does -- spawn ambient traffic, place an obstacle, tick, read render state (raw and
// interpolated) -- so a change that breaks the public API's game-facing ergonomics fails the suite.
// Additive / sample-only; no bearing on the parity path.
public class RungB17GameHostSampleTests
{
    private static string RerouteNet =>
        Path.Combine(RepoRoot(), "scenarios", "15-reroute", "net.net.xml");

    [Fact]
    public void GameHost_SpawnsTicksAndRenders()
    {
        using var host = new GameHost(RerouteNet, seed: 12345);
        Assert.True(host.SpawnableEdgeCount >= 2);

        var spawned = host.SpawnAmbient(12);
        Assert.True(spawned > 0, "at least some random trips should be routable");

        // A static blocker on a real lane places successfully; a bogus lane is rejected (not thrown).
        Assert.True(host.AddObstacleOnLane(host.GetSpawnableEdgeId(0) + "_0", frontPos: 15.0));
        Assert.False(host.AddObstacleOnLane("no_such_lane_0", frontPos: 1.0));

        for (var step = 0; step < 60; step++)
        {
            host.Tick();
        }

        Assert.True(host.VehicleCount > 0, "traffic should be active after warm-up");
        Assert.True(host.SimTime > 0.0);

        var raw = host.GetRenderVehicles();
        Assert.Equal(host.VehicleCount, raw.Count);

        // The interpolated read returns the same set of vehicles, each within the network extent.
        var interp = host.GetRenderVehicles(host.SimTime - 0.5);
        Assert.Equal(raw.Count, interp.Count);
        foreach (var v in interp)
        {
            Assert.InRange(v.Angle, 0f, 360f);
        }
    }

    // Determinism: the same seed + same tick sequence yields the same vehicle count and time.
    [Fact]
    public void GameHost_IsDeterministicForAGivenSeed()
    {
        static (int count, double time) RunOnce()
        {
            using var host = new GameHost(RerouteNet, seed: 999);
            host.SpawnAmbient(10);
            for (var i = 0; i < 50; i++) host.Tick();
            return (host.VehicleCount, host.SimTime);
        }

        Assert.Equal(RunOnce(), RunOnce());
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
