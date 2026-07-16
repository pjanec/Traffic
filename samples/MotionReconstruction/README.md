# MotionReconstruction

The smallest possible consumer of **SumoSharp.Viewer.Motion** — a tutorial-style walkthrough of
turning a *sparse*, low-rate stream of vehicle samples into a *smooth*, continuous per-frame render
pose, with **no renderer and no wire transport** involved.

> Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
> affiliated with or endorsed by the Eclipse SUMO project.

## What this shows

This sample is `src/Sim.Viewer.Motion/README.md`'s pipeline pseudocode, made runnable:

```
drClock.Pump(newestSampleTime, hold: paused)                 // advance the render clock
resolved = drClock.Resolve(history, delay, laneSource)        // pick/interpolate a DrState
state    = resolved.State with { Length, Width }               // add dims from your registry
pose     = PoseResolver.Resolve(laneSource, state,             // DrState -> (x, y[, z], heading)
               resolved.Upcoming, dt: 0, RenderRealism.ChordHeading)
(x, y, heading) = smoother.Smooth(handle, pose.X, pose.Y, pose.HeadingDeg, state.Speed, frameDt)
```

Concretely:

- A tiny hand-built `ILaneShapeSource` (`StraightLaneSource`) for one straight lane — a 2-point
  polyline `(0,0) -> (100,0)`. In a real viewer this comes from the parsed `.net.xml`.
- A **sparse** (1 Hz), hand-built `Sim.Replication.VehicleSampleHistory` of one `VehicleRecord`
  advancing at a constant 10 m/s: 3 samples, one second apart (`t=0,1,2` -> `pos=0,10,20`).
- A simulated ~10 Hz render loop (real wall-clock `Thread.Sleep`, since `DrClock.Pump` ties its
  render clock to a real `Stopwatch` exactly like a live game/viewer loop would) that, every frame:
  1. `DrClock.Pump(newestSampleTime)` — advances the render clock.
  2. `DrClock.Resolve(history, delay, lanes)` — resolves a `DrState`, either **interpolating**
     between two buffered samples or **extrapolating** forward from the newest one.
  3. `PoseResolver.Resolve(...)` — turns that `DrState` into a world `(x, y, heading)`.
  4. `DrPoseSmoother.Smooth(...)` — the optional per-frame chase filter a real renderer runs on top.

Every call in `Program.cs` is commented inline, in order, as a tutorial, and the run ends with a
"what to notice" section explaining the interpolate/extrapolate switch and the `delay` knob.

## Installing the package (in your own project)

This sample uses a `ProjectReference` into `src/Sim.Viewer.Motion` because SumoSharp isn't published
to nuget.org yet. In a real consumer project you would instead run:

```bash
dotnet add package SumoSharp.Viewer.Motion
```

which pulls in `SumoSharp.Core` (`PoseResolver`/`ILaneShapeSource`/`DrState`) and
`SumoSharp.Replication` (`TimestampedSample`/`IVehicleSampleHistory`/`DrExtrapolation.Arc`) —
`SumoSharp.Viewer.Motion` has no renderer and no native/DDS dependency of its own.

## Run it

```bash
dotnet run --project samples/MotionReconstruction
```

It prints one line per render frame (position, heading, and whether that frame interpolated or
extrapolated), so you can watch the reconstructed position advance smoothly between the three
sparse samples instead of teleporting from one to the next.

## See also

`src/Sim.Viewer.Motion/README.md` — the full design rationale, the pitfalls each mechanism fixes
(under-sampling at junctions, faceted-geometry heading stair-stepping, decel-extrapolation
reversal), and the tunables table. `src/Sim.Viewer` is the repo demo tool that uses this package for
real, over a live DDS feed and a Raylib renderer.
