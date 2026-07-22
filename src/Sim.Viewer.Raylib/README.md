# SumoSharp.Viewer.Raylib

Generic **raylib/ImGui desktop rendering** for a SumoSharp traffic stream. Targets `net8.0` only
(native GPU dependency; not `netstandard`, unlike the portable packages in this repo).

**What it solves.** Once a simulation is running (locally, via `SumoSharp.Viewer.Motion`'s
dead-reckoning reconstruction, or over a raw DDS feed), someone still has to draw it: roads, lane
markings, traffic-light dots, oriented vehicles, a pan/zoom camera, and the DDS subscriber adapter
that turns a wire stream into the in-memory state a renderer walks. This package is that layer — the
same rendering code the `Sim.Viewer` desktop demo tool uses, extracted so any SumoSharp-based
project can draw its own scene without re-implementing the road/vehicle draw passes or the DDS
decode from scratch.

## What's in the box

- **`Renderer`** — the static-world (roads, lane markings) and dynamic-world (traffic lights,
  vehicles, obstacles) draw passes, plus the ImGui controls/diagnostics panels and the world<->screen
  `Flip` convention (SUMO Y-up -> raylib Y-down).
- **`RoadLayerCache`** — bakes the camera-static road layer into an offscreen `RenderTexture2D`,
  re-stroked only when the camera actually changes (pan/zoom) instead of every frame — the fix for a
  large net's road-redraw cost dominating frame time.
- **`FrameStats`** — a small ring buffer of recent frame times, for an fps/min/avg/p99 HUD readout.
- **`IRenderOverlay`** — the generic render-overlay seam: a consumer plugs extra world-space draw
  passes / UI / click-handling into the render loop without this package (or the loop) knowing the
  concrete overlay type. `MarkerOverlay` is a trivial reference implementation (a single marker dot)
  used to prove the seam end-to-end.
- **`DdsSubscriber`** — the read side of the DDS data path: decodes the four topics a
  `Sim.Replication.Dds`-based publisher writes (vehicle state, static lane geometry, lifecycle,
  traffic-light state) into plain in-memory state, exposed through the transport-neutral
  `Sim.Replication.IReplicationSource`.
- **`DdsGeometryLaneSource`** — adapts the subscriber's decoded geometry into
  `Sim.Core.PoseResolver.ILaneShapeSource`, so `SumoSharp.Viewer.Motion`'s `DrClock.Resolve` can walk
  a remotely-received lane polyline exactly like a local `NetworkModel`.
- **`RenderHelpers`** — the shared per-frame draw-pose builders (`PumpAndBuildVehicleDraws` for a DDS/
  dead-reckoned stream, `BuildLocalVehicleDraws` for a local authoritative snapshot,
  `ComputeGeometryBounds` for fitting a camera to received geometry with no local network model to
  read bounds from).

**Not in this package**: any curated demo catalog, sample scenario selection, or domain-specific
overlay (e.g. a panic-evacuation layer) — those are consumer/demo-tool concerns and stay out so this
package remains a plain, generic rendering leaf for any SumoSharp stream.

## Depends on

- `SumoSharp.Viewer.Motion` (portable render-side motion reconstruction — `DrClock`, `KinematicReconstructor`).
- `SumoSharp.Replication.Dds` (the CycloneDDS binding — topics, wire codecs, QoS profiles).
- `Raylib-cs` + `rlImgui-cs` (the native rendering/UI dependency this package exists to wrap).

## License & disclaimer

Dual-licensed **EPL-2.0 OR GPL-2.0-or-later** (SumoSharp is a derivative of Eclipse SUMO and cannot be
relicensed). Practical read: EPL-2.0 is a weak, file-level copyleft — a proprietary game may link this
package and keep its own source closed, but must keep SUMO-derived files under EPL and publish changes
to *those* files. This is not legal advice; get counsel for commercial use.

Unofficial, independent C# reimplementation of Eclipse SUMO's microscopic simulation core. Not
affiliated with or endorsed by the Eclipse SUMO project. "SUMO" is an Eclipse trademark.
