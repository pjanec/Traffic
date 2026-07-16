// EvacDemo -- a tutorial-style walkthrough of SumoSharp.Evac, the panic-evacuation layer that sits
// on top of the (unmodified) SumoSharp.Core driving engine. Run it with:
//   dotnet run --project samples/EvacDemo
using Sim.Evac;

internal static class Program
{
    // The full cascade (incident -> panic -> flee -> gridlock -> conversion -> foot exodus) needs a
    // few hundred ticks to play out on the grid fixture -- 300 is comfortably enough to see every
    // stage fire at least once (EvacSpineTests's own spine test runs 600 to build in extra margin).
    private const int Ticks = 300;
    private const int ReportEvery = 25;

    private static int Main()
    {
        Console.WriteLine("EvacDemo -- SumoSharp.Evac panic-evacuation cascade over the evac-grid fixture.");

        // 1) EvacGridScenario is the ONE shared source of truth for the headless demo/test grid (the
        //    same fixture EvacSpineTests and the native viz build from): a hand-built 4x4
        //    priority-junction grid (scenarios/evac-grid/net.net.xml, parity-EXEMPT -- no golden,
        //    since this exercises the external evac layer, not the SUMO-parity core), its 16
        //    straight-across routes, its boundary exit edges, and a default central Incident.
        //    .Build(netPath) does everything HelloTraffic does by hand (new Engine(), LoadNetwork,
        //    DefineVType, SpawnVehicle per route) PLUS constructs the EvacDirector that will drive the
        //    evacuation and Track()s every spawned vehicle so the director watches it.
        var netPath = ResolveNetPath();
        var (engine, director, handles) = EvacGridScenario.Build(netPath);

        var incident = director.Incident;
        Console.WriteLine(
            $"network: {netPath}");
        Console.WriteLine(
            $"incident: centre=({incident.X:F0},{incident.Y:F0}) radius={incident.Radius:F0}m " +
            $"start={incident.StartTime:F0}s");
        Console.WriteLine($"tracked vehicles: {handles.Count} (organized traffic, 2 cars/route x 16 routes)");
        Console.WriteLine();

        // 2) EvacDirector.Tick() is the coordinated tick: PRE (feed vehicle/pedestrian footprints to
        //    each other, update the fear field, panic + reroute newly-latched drivers), engine.Step()
        //    (the unmodified parity core), POST (detect blocked+panicked cars, convert them to
        //    pedestrians -- or hand them to the Orca-push shoulder stage first -- and advance the foot
        //    exodus). Driving the whole layer is exactly this one call per step; nothing else is
        //    required of the host.
        for (var step = 1; step <= Ticks; step++)
        {
            director.Tick();

            if (step % ReportEvery == 0 || step == Ticks)
            {
                // 3) The director's read-only observability surface -- no internal state, just
                //    counters -- is enough to narrate the cascade: PanickedCount (drivers who have
                //    latched fear and switched to the flee preset/route), ConvertedCount (blocked
                //    panicked drivers who have abandoned ship), PedestrianCount (spawned foot
                //    evacuees), EscapedCount (pedestrians who reached the safe radius), and
                //    AbandonedCarCount (the frozen-obstacle cars pedestrians/other traffic now avoid).
                Console.WriteLine(
                    $"  t={director.Time,5:F0}s (step {step,3})  " +
                    $"panicked={director.PanickedCount,2} converted={director.ConvertedCount,2} " +
                    $"pedestrians={director.PedestrianCount,2} escaped={director.EscapedCount,2} " +
                    $"abandonedCars={director.AbandonedCarCount,2} pushers={director.PusherCount,2} " +
                    $"liveVehicles={engine.VehicleHandles.Length,2}");
            }
        }

        Console.WriteLine();
        Console.WriteLine("=== end-of-run summary ===");
        Console.WriteLine($"panicked            : {director.PanickedCount} / {handles.Count} tracked vehicles");
        Console.WriteLine($"converted (abandoned): {director.ConvertedCount}");
        Console.WriteLine($"pedestrians spawned  : {director.PedestrianCount}");
        Console.WriteLine($"pedestrians escaped  : {director.EscapedCount}");
        Console.WriteLine($"abandoned-car obstacles : {director.AbandonedCarCount}");
        Console.WriteLine($"orca-push cars (ever): {director.PusherCount}");
        Console.WriteLine("done.");
        return 0;
    }

    // scenarios/evac-grid/net.net.xml, found by walking up to the repo root (Traffic.sln) -- the same
    // parity-exempt fixture EvacSpineTests builds from (no rou.xml demand, no SUMO needed: vehicles
    // are spawned at runtime by EvacGridScenario.Build).
    private static string ResolveNetPath()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        if (dir is null)
        {
            throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above the exe).");
        }

        return Path.Combine(dir.FullName, "scenarios", "evac-grid", "net.net.xml");
    }
}
