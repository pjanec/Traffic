// net8.0-only runnable demo (the ns2.1 target of this project builds as a library and excludes this entry
// point). Drives GameHost -- the netstandard2.1-clean integration class a Unity/Godot host would use --
// as a headless loop, so the sample is verifiable end-to-end in a plain .NET environment:
//   dotnet run --project samples/SumoSharp.GameHostSample
#if NET8_0_OR_GREATER

using SumoSharp.GameHostSample;

internal static class Program
{
    private static int Main(string[] args)
    {
        var netPath = args.Length > 0 ? args[0] : DefaultNet();
        if (netPath is null || !File.Exists(netPath))
        {
            Console.Error.WriteLine("error: could not find a .net.xml (pass one as the first argument).");
            return 1;
        }

        Console.WriteLine($"SumoSharp GameHost sample — network: {netPath}");
        using var host = new GameHost(netPath);
        Console.WriteLine($"spawnable edges: {host.SpawnableEdgeCount}");

        // Seed some ambient traffic, then run a fixed-step loop, topping up occasionally.
        var spawned = host.SpawnAmbient(12);
        Console.WriteLine($"requested 12 trips, {spawned} were routable");

        // Drop a static blocker on the first spawnable edge's lane 0 (SUMO lane-id convention "<edge>_0").
        var blocked = host.AddObstacleOnLane(host.GetSpawnableEdgeId(0) + "_0", frontPos: 15.0, length: 5.0);
        Console.WriteLine($"obstacle on {host.GetSpawnableEdgeId(0)}_0: {(blocked ? "placed" : "lane not found")}");

        const int steps = 200;
        for (var step = 0; step < steps; step++)
        {
            host.Tick();

            if (step % 40 == 39)
            {
                host.SpawnAmbient(4); // keep traffic flowing
            }

            if (step % 50 == 49)
            {
                Console.WriteLine($"  t={host.SimTime,6:0.0}s  vehicles={host.VehicleCount}");
            }
        }

        // Demonstrate the interpolation hook: render at the midpoint between the last two sim frames (as a
        // 2x-faster-than-sim renderer would) and show one vehicle blended vs its raw latest sample.
        var latest = host.GetRenderVehicles();
        if (latest.Count > 0)
        {
            var midTime = host.SimTime - 0.5; // halfway back toward the previous frame (1 s steps)
            var interp = host.GetRenderVehicles(midTime);
            var raw = latest[0];
            var smooth = FindByHandleId(interp, raw.Id);
            Console.WriteLine(
                $"interp check — {raw.Id}: latest=({raw.X:0.00},{raw.Y:0.00})  " +
                $"render@t-0.5=({smooth.X:0.00},{smooth.Y:0.00})");
        }

        Console.WriteLine($"done — {steps} steps, final vehicle count {host.VehicleCount}.");
        return 0;
    }

    private static RenderVehicle FindByHandleId(IReadOnlyList<RenderVehicle> list, string id)
    {
        foreach (var v in list)
        {
            if (v.Id == id) return v;
        }

        return list.Count > 0 ? list[0] : default;
    }

    // scenarios/15-reroute/net.net.xml, found by walking up to the repo root (Traffic.sln) -- the same
    // routable network the Sim.LiveHost demo defaults to.
    private static string? DefaultNet()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir is null ? null : Path.Combine(dir.FullName, "scenarios", "15-reroute", "net.net.xml");
    }
}

#endif
