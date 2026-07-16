# EvacDemo

The smallest possible consumer of **SumoSharp.Evac** — a tutorial-style walkthrough of the
panic-evacuation layer that sits on top of the (unmodified) SumoSharp driving core, turned into a
runnable console program.

> Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
> affiliated with or endorsed by the Eclipse SUMO project.

## What this shows

- Building an evac scenario from the **committed shared fixture**: `EvacGridScenario.Build(netPath)`
  loads the parity-exempt `scenarios/evac-grid/net.net.xml` (a hand-built 4x4 priority-junction
  grid), spawns organized traffic on 16 crossing routes, places a default central `Incident`, and
  constructs the `EvacDirector` that will drive the evacuation — the exact same fixture
  `EvacSpineTests` and the native viz build from.
- Driving the whole layer with a single call per step: `EvacDirector.Tick()` (pre-step fear/panic
  update and reroute, `Engine.Step()`, post-step blocked-detection/conversion/foot-exodus).
- Reading the director's read-only observability surface to narrate the cascade as it plays out:
  `PanickedCount`, `ConvertedCount`, `PedestrianCount`, `EscapedCount`, `AbandonedCarCount`,
  `PusherCount`, and `Incident`/`Time`.

Every call in `Program.cs` is commented inline, in order, as a tutorial.

## Installing the package (in your own project)

This sample uses a `ProjectReference` into `src/Sim.Evac` because SumoSharp isn't published to
nuget.org yet. In a real consumer project you would instead run:

```bash
dotnet add package SumoSharp.Evac
```

which pulls in `SumoSharp.Core` (and, transitively, `SumoSharp.Ingest`) — the driving core the evac
layer is built on top of, without the driving core ever referencing it back.

## Run it

```bash
dotnet run --project samples/EvacDemo
```

It builds the evac-grid fixture (32 tracked vehicles across 16 routes), ticks the director 300
times (`stepLength=1.0s`, incident starts at `t=8s`), prints a progress line every 25 steps, and
ends with a summary of how many drivers panicked, converted to pedestrians, and escaped.

## See also

`src/Sim.EvacProfile` runs the larger `EvacOrganicScenario`/`EvacCityScenario` fixtures with the
director's opt-in profiler turned on, for a per-phase wall-time cost breakdown at city scale.
`tests/Sim.ParityTests/EvacSpineTests.cs` is the behavioral/property test suite this sample's
fixture is shared with (parity-exempt: the evac layer sits outside the SUMO-parity core, so its
determinism is proven separately, not against a golden trajectory).
