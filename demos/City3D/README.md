# City3D — a 3D city viewer that *consumes* the SumoSharp NuGet packages

A Godot 4 (.NET / C#) demo that renders a running SumoSharp simulation in 3D — a procedural box-city
generated from the network geometry, width-accurate roads, simplified traffic lights, and true-size cars
moving *smoothly* between sparse sim updates via `SumoSharp.Viewer.Motion` — **local** (engine co-hosted
in-process) and **remote** (a decoupled headless host streaming over DDS to a separate viewer process).

**Why it exists.** Unlike the projects under `samples/` (which `<ProjectReference>` into `src/`), this
demo references `SumoSharp.*` only via `<PackageReference>` resolved from a **local NuGet feed**. It
validates the *packaged consumer experience* end to end — before anything is published to nuget.org — and
showcases the render-side dead-reckoning (DR) motion story in a real open-source engine. Design + tasks +
integration handoff: `docs/DEMO-CITY3D-{DESIGN,TASKS,TRACKER,HANDOFF}.md`.

> **Nothing heavy is committed.** The Godot engine binary (~100 MB) is fetched on demand by
> `fetch-godot.sh`; `local-nuget/`, `.godot/`, and build output are git-ignored. Committed here: only
> `.cs`, `.tscn`, `project.godot`, the scripts, `nuget.config`, and this README.

---

## Layout

| Path | What |
|---|---|
| `CityLib/` | Plain `net8.0` class library, **Godot-free**, consuming `SumoSharp.*` from the local feed. All the logic: `CoordinateTransform`, `SimSource` (engine→`ReplicationPublisher`→`InMemoryReplicationBus`), `Reconstructor` (DrClock→PoseResolver→DrPoseSmoother → Godot-space poses), `ReplicationLaneShapeSource`, `RoadMeshBuilder`, `BuildingPlacer`, `CarTransform`, `TrafficLightPlacer`, `OffAxisFrustum`. |
| `CityLib.Tests/` | xUnit over `CityLib` (**41 tests**) — the headless-provable success conditions. |
| `Viewer/` | The Godot 4 app: thin glue that turns `CityLib`'s plain arrays into `ArrayMesh`/`MultiMeshInstance3D`/cameras. Local by default; `-p:City3DRemote=true` adds the DDS subscriber. |
| `build.sh` `fetch-godot.sh` `run-local.sh` `run-remote.sh` `run-smoke.sh` `screenshot.sh` `perf-ladder.sh` `nuget.config` | the local-feed + run/verify tooling (below). |

The reusable engine→wire publisher (`SumoSharp.Host`) and the generic headless DDS host
(`src/Sim.Host.App`) live in `src/` (product), not here — this demo *consumes* them.

## Requirements
.NET 8 SDK. Network access to nuget.org (Godot SDK, BCL, CycloneDDS) and to `downloads.godotengine.org`
(the engine binary). SUMO is **not** needed. A GPU/desktop is needed only to *see* it — see
"What is verified where".

---

## Quick start (local, single viewport)

```bash
demos/City3D/build.sh                 # pack SumoSharp.* -> ./local-nuget, restore + build the Viewer
demos/City3D/run-local.sh             # launch the viewer (interactive window if you have a display)
demos/City3D/run-local.sh --scenario=_bench/city-mixed-1k --camera=close
```

`build.sh` runs `dotnet pack` for the packages the demo needs into `local-nuget/`; `nuget.config` pins
`SumoSharp.*` to that feed via `packageSourceMapping`, so our packages can **never** be resolved from
nuget.org (everything else — Godot SDK, BCL, CycloneDDS — still comes from nuget.org). `run-local.sh`
opens an interactive window when `DISPLAY` is set, otherwise runs headless under Xvfb + software GL.

**Local mode is pure C#** (no native DDS dependency) — it is the clean "any C# engine consumes SumoSharp"
reference. It hosts the engine in-process, publishes each step into an `InMemoryReplicationBus`, and
reconstructs smooth per-frame poses with `Viewer.Motion` — a genuine DR viewer, just co-hosted.

## Remote (decoupled host → viewer over DDS)

```bash
demos/City3D/run-remote.sh            # end-to-end: Sim.Host.App (publisher) + the DDS-built Viewer
```

Under the hood, two processes:

```bash
# 1) headless host — publishes any scenario over DDS (no GPU); a generic src/ tool:
dotnet run --project src/Sim.Host.App -- --scenario scenarios/_bench/city-mixed-1k --transport dds --hz 10

# 2) the viewer, built with the remote flag, subscribing over DDS:
demos/City3D/build.sh --remote
dotnet build demos/City3D/Viewer -c Debug -p:City3DRemote=true
# then run the Viewer with --transport=dds (run-remote.sh wires this under Xvfb)
```

Only the remote build pulls the native `SumoSharp.Replication.Dds`. The renderer talks to
`IReplicationSource` either way, so local↔remote is a **source swap with the reconstruction/render code
unchanged** — one shared `RenderFrame()`. DDS discovery is zero-config on a LAN/loopback (cross-subnet
needs a Cyclone peer config).

## Video wall (CLI-configured off-axis channels)

One shared eye, N channels, each a possibly **asymmetric (off-axis) frustum** into the same scene — the
standard tiled-wall / CAVE projection (seamless tiles, unlike per-tile rotated cameras). No screen
autodetection; channels are given on the CLI. The frustum math is `CityLib.OffAxisFrustum` (a reusable,
tested Kooima generalized-perspective tool).

```bash
# 3 tiles of one continuous flat screen (shared eye), composited side by side:
demos/City3D/run-local.sh --scenario=09-traffic-light \
  --channel="off=300,45,160;screen=100,0,40,233.3,0,40,100,90,40" \
  --channel="off=300,45,160;screen=233.3,0,40,366.7,0,40,233.3,90,40" \
  --channel="off=300,45,160;screen=366.7,0,40,500,0,40,366.7,90,40"
# or the simple offset+angles+fov form: --channel="off=x,y,z;look=yaw,pitch;fov=60"
```

## Scenario switch & scale (perf)

`--scenario=<path under scenarios/>` is the only dial — dev → huge, same build, no code change:

```bash
demos/City3D/perf-ladder.sh           # host-side step+publish across the _bench rungs
```

Measured here (host-side, `--transport inmem`, 240 sim steps @ hz 40; wall time includes load + dotnet
startup; headless, no GPU):

| rung | peak concurrent vehicles | 240 steps, wall |
|---|--:|--:|
| `_bench/city-30` | 34 | ~7 s |
| `_bench/city-300` | 190 | ~10 s |
| `_bench/city-mixed-1k` (signalized) | 579 | ~9 s |
| `_bench/city-3000` | 1 492 | ~11 s |
| `_bench/city-15000` | 4 017 | ~13 s |

Static procedural scale for the signalized city (`city-mixed-1k`): **7 024 road ribbons · 15 124
buildings · 1 876 signal heads**, all generated from the network geometry.

---

## Verify (headless — no GPU)

```bash
demos/City3D/build.sh            # packs the feed; resolves SumoSharp.* from it (fails NU1101 without it)
dotnet test demos/City3D/CityLib.Tests -c Release   # 41/41: transform, reconstruction, meshes, frustum
demos/City3D/run-smoke.sh        # builds the Godot viewer + runs it headless (scene runs, quits 0)
demos/City3D/screenshot.sh       # Xvfb + llvmpipe software render -> a PNG
demos/City3D/run-remote.sh       # two-process DDS round-trip (host -> viewer)
```

## What is verified where

| Provable headless (here) | Desktop / GPU only (you) |
|---|---|
| feed packs; `SumoSharp.*` resolves from it and **fails without it** (no nuget.org fallback) | the actual rendered image / believable-scale look |
| `CityLib.Tests` 41/41 (math + reconstruction + procedural geometry + frustum seams) | a real **multi-monitor** video wall spanning physical screens |
| Godot viewer builds + `godot --headless` scene smoke; Xvfb software-rendered screenshots | interactive camera / smooth-motion feel at 60 fps |
| two-process DDS round-trip (host → viewer) | |

**On motion in headless screenshots:** `DrClock` is driven by real wall-clock, so a fast headless loop
shows near-frozen render motion (a screenshot is a still, and smooth-motion *fidelity* is proven in the
`CityLib.Tests` real-time tests). On a desktop at real-time 60 fps the motion is smooth.

## Known limitations (honest)
- **Elevation over the wire:** the replication geometry (`GeometryCodec.LaneGeo`) is 2-D — no `Z`. Local
  mode gets elevation from the Z-aware `NetworkLaneSource`; a remote viewer renders flat until a future
  `GeometryCodec` Z-extension. (No committed scenario actually carries Z geometry today anyway —
  `_bench/city-organic-L2` "L2" = 2 *lanes*, not 2 *levels*.)
- **Remote buildings:** buildings need per-edge data the wire doesn't carry, so the remote viewer renders
  roads + cars + signals and skips buildings (logged). Local renders the full city.
- **Feed version:** packages pack at `Directory.Build.props`'s version (0.1.0); `build.sh` re-packs, so
  bump there if you change it.
