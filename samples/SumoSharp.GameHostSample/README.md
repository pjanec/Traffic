# SumoSharp.GameHostSample

The **Unity / Godot reach** sample for SumoSharp — a `netstandard2.1`-consumable integration you can drop
into a game engine, plus a runnable headless demo for plain .NET.

> Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
> affiliated with or endorsed by the Eclipse SUMO project.

## What it shows

`GameHost.cs` is the reusable brain — deliberately written to the `netstandard2.1` API surface, so it is
exactly what a Unity `MonoBehaviour` or a Godot `Node` would contain. It wraps an `Engine` + a
`SimulationRunner` and exposes only game-shaped calls:

| Call | When you call it | SumoSharp API exercised |
|---|---|---|
| `Tick()` | fixed-timestep callback (`FixedUpdate` / `_physics_process`) | `SimulationRunner.Tick` (manual, deterministic) |
| `GetRenderVehicles()` | render callback (`Update` / `_process`) | the published SoA `SimulationSnapshot` |
| `GetRenderVehicles(renderTime)` | render callback, faster than the sim | the **two-frame interpolation hook** (§7) |
| `SpawnAmbient(n)` | whenever you want traffic | **dense edge handles** + `SpawnVehicle` + queued insertion (§9) |
| `AddObstacleOnLane(...)` | drop a blocker | the handle-based obstacle API + command dispatcher (§4, §7) |

It also turns the **opt-in snapshot pool** on (`EnableSnapshotPool`) so the per-frame render read is
allocation-free in steady state. No edge-id or lane-id **string** is touched on the per-frame path — ids
are resolved to `int` handles once up front (`GetEdge` / `GetLane`), the currency of the hot path.

## Multi-targeting (the point of this sample)

The project multi-targets the same pair as the shipped packages:

- **`netstandard2.1` → builds as a library.** Building this target proves the SumoSharp public API is
  genuinely *consumable* from a `netstandard2.1` project (Unity's Mono/IL2CPP scripting runtime, Godot's
  .NET), not merely that the library itself compiles for `ns2.1`.
- **`net8.0` → builds as an exe.** `Program.cs` (net8-only, `#if NET8_0_OR_GREATER`) drives `GameHost`
  as a headless loop, so the sample is verifiable end to end in a normal .NET environment.

## Run the headless demo

```bash
dotnet run --project samples/SumoSharp.GameHostSample
# or point it at any SUMO network:
dotnet run --project samples/SumoSharp.GameHostSample -- path/to/net.net.xml
```

## Using it from Unity

1. Build the two libraries for `netstandard2.1`:
   ```bash
   dotnet build src/Sim.Core/Sim.Core.csproj -c Release -f netstandard2.1
   ```
   Copy `Sim.Core.dll` and `Sim.Ingest.dll` (and `System.Memory.dll`) into `Assets/Plugins/`.
2. Add `GameHost.cs` (and `RenderVehicle`) to your project.
3. In a `MonoBehaviour`: construct `GameHost` in `Start()`, call `Tick()` from `FixedUpdate()`, and read
   `GetRenderVehicles(Time.timeAsDouble)` from `Update()` to position your car GameObjects (interpolated
   for smooth motion between sim ticks).

## Using it from Godot (.NET)

Same shape: construct `GameHost` in `_Ready()`, `Tick()` from `_PhysicsProcess(delta)`, and read
`GetRenderVehicles(...)` from `_Process(delta)` to move your nodes.

## See also

`src/Sim.LiveHost` is the interactive browser demo (ASP.NET + WebSocket) that adds the **screen→lane
projection** turning a mouse click into an `AddObstacle` call — the piece this headless sample references
but does not itself implement.
