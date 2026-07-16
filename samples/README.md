# SumoSharp samples

Runnable, tutorial-style demonstrations of consuming the SumoSharp packages. Every sample uses a
`ProjectReference` into `src/` (the packages aren't published to nuget.org yet), but each sample's
own README shows the equivalent `dotnet add package ...` a real consumer would use.

| Package | Demonstrated in |
|---|---|
| `SumoSharp.Core` | [`HelloTraffic`](HelloTraffic/README.md) (tutorial console app), [`SumoSharp.GameHostSample`](SumoSharp.GameHostSample/README.md) (async runner, obstacles, render interpolation) |
| `SumoSharp.Ingest` | pulled in transitively by `SumoSharp.Core` — exercised (but not separately demonstrated) by `HelloTraffic` and `SumoSharp.GameHostSample`, which both load a `.net.xml` through it |
| `SumoSharp.Replication` | [`StreamingLoopback`](StreamingLoopback/README.md) (in-memory `IReplicationSink`/`IReplicationSource` round trip); also used by `src/Sim.LiveHost` (the interactive browser demo) |
| `SumoSharp.Replication.Dds` | shown in the repo demo tool `src/Sim.Viewer` (via `Sim.Viewer.Core`'s `DdsPublisher` and `Sim.Viewer.Raylib`'s `DdsSubscriber`) — **not yet a standalone sample** |
| `SumoSharp.Viewer.Motion` | [`MotionReconstruction`](MotionReconstruction/README.md) (`DrClock`/`PoseResolver`/`DrPoseSmoother` pose reconstruction, no renderer/transport); also shown in the repo demo tool `src/Sim.Viewer` |
| `SumoSharp.Viewer.Raylib` | shown in the repo demo tool `src/Sim.Viewer` (the Raylib renderer) — **not yet a standalone sample** |
| `SumoSharp.Evac` | [`EvacDemo`](EvacDemo/README.md) (`EvacGridScenario` + `EvacDirector.Tick()`, the panic-evacuation cascade); also shown in the repo demo tool `src/Sim.Viewer` (evacuation demo mode) and `src/Sim.EvacProfile` |
| `SumoSharp.Testing` | the parity-test harness itself (`src/Sim.Harness`, consumed by `tests/Sim.ParityTests`) — **not yet a standalone sample** |
| `SumoSharp` (meta package) | aggregates the packages above for a one-line install; no separate demo needed |

## The newer samples

- **[`HelloTraffic`](HelloTraffic/README.md)** — the smallest possible `SumoSharp.Core` consumer:
  load a network, spawn a couple of vehicles, step, read back live state. Start here if you're new to
  the engine's synchronous API.
- **[`StreamingLoopback`](StreamingLoopback/README.md)** — the smallest possible
  `SumoSharp.Replication` consumer: publish geometry/lifecycle/frames through `IReplicationSink` and
  read them back through `IReplicationSource`, with no DDS or network involved. Start here if you're
  building a remote viewer/replication client and want to understand the wire-neutral contract before
  touching DDS.
- **[`EvacDemo`](EvacDemo/README.md)** — the smallest possible `SumoSharp.Evac` consumer: build the
  committed evac-grid fixture, tick `EvacDirector`, and watch the panic-evacuation cascade (incident
  -> panic -> flee -> gridlock -> driver-to-pedestrian conversion -> foot exodus) play out through
  its read-only counters.
- **[`MotionReconstruction`](MotionReconstruction/README.md)** — the smallest possible
  `SumoSharp.Viewer.Motion` consumer: turn a sparse (1 Hz), hand-built vehicle sample history into a
  smooth, continuous ~10 Hz per-frame pose via `DrClock`/`PoseResolver`/`DrPoseSmoother`, with no
  renderer and no wire transport involved.

## Existing sample

- **[`SumoSharp.GameHostSample`](SumoSharp.GameHostSample/README.md)** — the Unity/Godot-reach sample:
  a `netstandard2.1`-consumable `GameHost` class wrapping `Engine` + `SimulationRunner`, plus a
  runnable `net8.0` headless demo.

## Running the samples

```bash
dotnet run --project samples/HelloTraffic
dotnet run --project samples/StreamingLoopback
dotnet run --project samples/SumoSharp.GameHostSample
dotnet run --project samples/EvacDemo
dotnet run --project samples/MotionReconstruction
```

All five are wired into `Traffic.sln` (`IsPackable=false`, so they never ship to NuGet and never
affect the hermetic offline `dotnet test` gate).
