# Sim.LiveHost — browser-live interactive demo

A minimal ASP.NET Core sample (SUMOSHARP-API.md §11) that runs the SumoSharp engine live and streams it
to a browser over a WebSocket. Runtime-spawned traffic flows across the network; **click the road to drop
an obstacle** and watch the cars queue behind it. Not a NuGet package, and not part of `dotnet test`.

## Run

```bash
dotnet run --project src/Sim.LiveHost                       # defaults to scenarios/15-reroute
dotnet run --project src/Sim.LiveHost -- scenarios/39-...   # a scenario dir (uses its net.net.xml)
dotnet run --project src/Sim.LiveHost -- path/to/net.net.xml
dotnet run --project src/Sim.LiveHost -- scenarios/32-roundabout corner   # + production render mode
```

Add `chord` or `corner` (a.k.a. `offtrack`) as an extra argument to enable the production `RenderMode`
(`SUMOSHARP-DEADRECKONING.md` §6.3): `chord` renders SUMO's back→front chord heading, `corner` adds the
swept-path off-tracking bow ("trucks swing wide"). Both are renderer-only (the parity-exact lane-relative
state is unchanged) and are most visible on curvy nets / long vehicles.

Then open <http://localhost:5055>. Wheel = zoom, drag = pan, click = drop an obstacle, and the
**clear obstacles** button removes them.

## How it works

- **`SimHost`** loads the network with `Engine.LoadNetwork`, drives it with a `SimulationRunner` on a
  background thread (30 Hz), and spawns routable trips at runtime with `Engine.SpawnVehicle(from, to)`.
- **`Program.cs`** serves the page and a `/ws` WebSocket: it sends the network geometry once, then streams
  the immutable `SimulationRunner.Snapshot` (vehicle x/y/angle/speed) at ~20 fps, and receives click
  messages.
- A click is converted to **world coordinates** in the browser (the inverse camera transform), then the
  server projects that point to the nearest lane + along-lane position (the *screen→lane projection*) and
  injects a full-lane obstacle with `Engine.AddObstacle` via the runner's command dispatcher.

Everything the host touches goes through the async runner's boundary-applied commands and published
snapshot — the engine itself is never accessed off its own thread.
