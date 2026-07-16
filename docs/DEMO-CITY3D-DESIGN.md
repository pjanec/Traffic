# 3D city demo (Godot 4) consuming the SumoSharp NuGet packages — design

**HOW it works.** The WHAT (from the request): a small **3D city viewer** that renders a running
SumoSharp simulation — a procedural box-city generated from the network geometry, width-accurate roads,
simplified traffic lights, and true-size cars moving *smoothly* between sparse sim updates — built by
**consuming the SumoSharp.\* NuGet packages from a local feed** (never `nuget.org`, never a
`ProjectReference` into `src/`). The point is to validate the *packaged consumer experience* end to end
**and** to showcase the render-side dead-reckoning (DR) motion story in a real open-source engine, first
as a single local viewer and then as a **remote** viewer fed by a decoupled headless host (the
professional use case).

This doc is the mechanism. It does **not** restate: the package map (`docs/PACKAGES.md`), the DR
reconstruction pipeline (`docs/SUMOSHARP-VIEWER-DR-SMOOTHING.md`, esp. **§5** and the **§8**
reimplementation checklist), the replication contract (`docs/SUMOSHARP-PACKAGING-DESIGN.md` D2/D8/D9/D10
and `src/Sim.Replication`), or the DR-error publishing rationale
(`docs/SUMOSHARP-DR-ERROR-PUBLISHING-DESIGN.md`). Read those for the *why* behind the pieces this doc
wires together. Task breakdown + success conditions: `docs/DEMO-CITY3D-TASKS.md`; tracker:
`docs/DEMO-CITY3D-TRACKER.md`.

---

## Design tenets

1. **Consume packages, not source.** The demo references `SumoSharp.*` via `<PackageReference>` resolved
   from a **local NuGet feed**. No `ProjectReference` into `src/`. This is the deliberate difference from
   `samples/` (which use project refs because nothing is published yet).
2. **`nuget.org` is off *only* for `SumoSharp.*`.** Everything else (the Godot SDK, the BCL, CycloneDDS)
   restores from `nuget.org` normally. `packageSourceMapping` pins `SumoSharp.*` to the local feed so our
   packages can **never** silently come from `nuget.org` — that pin is the thing being proved.
3. **Local first, remote second — but the render path is identical.** The local viewer feeds itself
   through an **in-process `InMemoryReplicationBus`** and reconstructs with `Viewer.Motion`, so it is a
   genuine **DR viewer** from day one. Going remote swaps *only* the `IReplicationSource` (in-memory →
   DDS); render, content generation, motion, and smoothing code are untouched. This is why "settle all
   content/graphics in local" actually carries over.
4. **The reusable pieces are packages; the Godot app is a build-from-checkout demo.** A Godot game is not
   NuGet-shaped (it has `project.godot`, scenes, an engine binary). The genuinely reusable parts —
   motion reconstruction (`Viewer.Motion`), the transport + data model (`Replication[.Dds]`), and the
   engine→wire publisher (`SumoSharp.Host`, new) — are packages. The demo *proves those packages are
   sufficient* to build a real 3D viewer. One script gives a clone → build → run experience; nothing
   heavy is committed.
5. **No hacks, DRY.** The engine→sink publisher exists today only as `Sim.Viewer.Core.DdsPublisher`,
   which is hardwired to DDS *and* to the non-packaged `EngineHost` — not reusable. We extract the
   reusable, transport-neutral publisher into a package and **rewire the native viewer to use it**, so
   there is exactly one publisher in the tree.
6. **`src/` parity path and the offline gate are untouched.** New `src/` projects are **additive**
   (no existing file's behaviour changes) or **render-side** (the DRY rewire). `dotnet test Traffic.sln`
   and the `Sim.Bench` determinism hash stay at their committed baseline on every stage that touches
   `src/`. Demo stages don't touch `src/` at all.
7. **Never assume you are the only session.** Other parallel sessions may be modifying the engine/`src/`
   concurrently (bug fixes, etc.), so (a) the standing-gate baseline (`dotnet test` counts + `Sim.Bench`
   determinism hash) is captured **fresh** at the start of each `src/`-touching task, never hardcoded
   from this doc; (b) rebase the demo branch onto its base before pushing; (c) all engine-side work here
   is additive or render-side on the demo's own branch, so a concurrent engine change should merge
   cleanly.

---

## Component map

```
                    ┌─────────────────────── committed src/ (product) ───────────────────────┐
                    │                                                                          │
 SumoSharp.Core ────┤  Engine · SimulationRunner · SimulationSnapshot · PoseResolver ·         │
 (Ingest)           │  NetworkLaneSource                                                       │
                    │                                                                          │
 SumoSharp.Replication ─ InMemoryReplicationBus · IReplicationSink/Source · VehicleRecord ·    │
                    │  GeometryCodec · TlCodec · PublishScheduler · DrExtrapolation             │
                    │                                                                          │
 SumoSharp.Viewer.Motion ─ DrClock · DrPoseSmoother                                            │
                    │                                                                          │
 ★ SumoSharp.Host (NEW, src/Sim.Host) ─ ReplicationPublisher: SimulationSnapshot+NetworkModel  │
                    │      → IReplicationSink (transport-neutral; used in-proc AND for DDS)     │
                    │                                                                          │
 SumoSharp.Replication.Dds ─ ★ DdsReplicationSink (moved here) + DDS source                    │
                    │                                                                          │
 ★ src/Sim.Host.App (NEW) ─ generic headless "publish any --scenario over DDS" tool            │
 Sim.Viewer.Core.DdsPublisher ─ ★ rewired to delegate to ReplicationPublisher + DdsSink        │
                    └──────────────────────────────────────────────────────────────────────────┘

                    ┌────────────── demos/City3D (Godot 4 .NET, consumes packages) ────────────┐
 local feed  ◀──────┤  local-nuget/  ← build.sh: dotnet pack the SumoSharp.* packages           │
 (git-ignored)      │  nuget.config  ← packageSourceMapping: SumoSharp.* → local feed only      │
                    │                                                                          │
                    │  Viewer (Godot app): ReplicationLaneShapeSource · procedural roads/       │
                    │    buildings/TLS · MultiMesh cars · DrClock→PoseResolver→DrPoseSmoother   │
                    │    ├─ local mode  : in-proc Engine + ReplicationPublisher → InMemory bus  │
                    │    └─ remote mode : DDS source ← Sim.Host.App (separate process)          │
                    └──────────────────────────────────────────────────────────────────────────┘
```

★ = new or changed by this work. Everything else already exists and is consumed as-is.

---

## `SumoSharp.Host` — the missing reusable publisher (`src/Sim.Host`, additive, packable)

**Why it exists.** A package consumer today cannot publish a running engine over the replication contract
without hand-rolling the snapshot→`VehicleRecord` translation. The only implementation,
`Sim.Viewer.Core.DdsPublisher`, is coupled to DDS (`DdsParticipant`) and to `EngineHost` (which lives in
the non-packaged `Sim.Viewer.Core`). So we extract the transport-neutral, host-neutral half into a
package.

**Shape.** `ReplicationPublisher` operates on the two Core/Ingest types that both hosts already have:

```
PublishGeometryOnce(NetworkModel network, IReplicationSink sink)   // durable geometry, once
PublishStep(SimulationSnapshot snap, IReplicationSink sink)        // lifecycle + frame + TL per step
```

- **Geometry**: build `GeometryCodec.LaneGeo` per lane from `network` (id, one-way, width, length,
  polyline) — the same bytes `DdsPublisher.BuildLaneGeos` produces today.
- **Frame**: per vehicle, translate `snap` columns → `VehicleRecord` — `Pos`, `PosLat`, `Speed`,
  `Accel`, `LatSpeed`, and `UpcomingLanes` from the flattened `snap.LaneWindow` (`[p2,p1,cur,n1,n2,n3]`),
  gated by the existing `PublishScheduler` / DR-error policy (`SumoSharp.Replication`). **No `x,y`, no
  heading** on the wire (§2.2) — the consumer rebuilds them.
- **Lifecycle / dims**: `Length`/`Width` per handle once on spawn (durable).
- **Traffic lights**: `TlCodec.TlEntry[]` from `snap.TlLaneHandle`/`snap.TlState` (low-rate).

`ReplicationPublisher` depends only on `Core` (+ `Ingest`, transitively) and `Replication` — **no DDS**.
TFMs `net8.0` + `netstandard2.1` (so Unity/Godot can consume it, like `Viewer.Motion`). `IsPackable=true`,
`PackageId=SumoSharp.Host`. It is **packed to the local feed and consumed by the demo**, but **not** added
to `pack-check.yml`'s published-9 assert nor `publish.yml` — it is "packaged and locally consumable, not
yet published." Promotion to published package #10 later is a docs/CI change, not a rewrite.

**DRY rewire (parity-neutral).** With `ReplicationPublisher` in place:
- The pure DDS **sink** (encode + DDS-write, an `IReplicationSink`) moves from `Sim.Viewer.Core` into
  **`SumoSharp.Replication.Dds`** as `DdsReplicationSink` — the DDS binding owns its own sink.
- `Sim.Viewer.Core.DdsPublisher` becomes a thin composition: read `EngineHost.Snapshot` → hand it to
  `ReplicationPublisher` → `DdsReplicationSink`. Its public entry points (`PublishGeometryOnce`,
  `PublishStep`) and the exact bytes on the wire are **unchanged**, so the native viewer behaves
  identically. This is render-side only; `dotnet test` + the determinism hash stay at baseline.

## `src/Sim.Host.App` — the generic headless DDS host (new, `IsPackable=false`)

A standalone reusable tool (sibling to `Sim.LiveHost`), **not** demo-specific:

```
dotnet run --project src/Sim.Host.App -- --scenario scenarios/_bench/city-mixed-1k \
    --spawn 1000 --hz 10 --transport dds
```

- Loads the net + demand with `Engine`/`Ingest`, drives `SimulationRunner` in manual mode, and each tick
  hands the snapshot to `ReplicationPublisher` → a sink chosen by `--transport` (`dds` | `inmem` for
  self-test). `--scenario` + optional `--spawn N` (GameHost-style ambient traffic) is the **switchable**
  scale dial — same build drives `09-traffic-light` (dev) up to `city-15000` (perf). No rendering, no
  GPU: runs on this Linux VM. The City3D **remote** viewer subscribes to whatever it publishes.

---

## The demo (`demos/City3D/`, Godot 4 .NET, C# only)

### Data path (identical for local and remote)
Per app frame the viewer runs the §5/§8 recipe against an `IReplicationSource`:

```
source.Pump()                                       // drain arrived packets into per-vehicle history
DrClock.Pump(newestSampleTime)                      // advance the monotonic render clock (§5.1)
for each vehicle with history H:
    resolved = DrClock.Resolve(H, delay, laneSource) // interpolate/extrapolate a DrState (§5.2)
    pose     = PoseResolver.Resolve(laneSource, state with dims, upcoming, dt:0, ChordHeading)  // (§5.3)
    (x,y,deg)= DrPoseSmoother.Smooth(handle, pose…)  // capped correction + heading tilt (§10.2/§10.3)
    draw car at (x, y, z) yaw=deg                    // z from LaneShapeZ; pitch from z-gradient
apply current TL state to signal-head materials
```

- **Local mode**: an in-process `Engine` + `ReplicationPublisher` → `InMemoryReplicationBus`; the viewer
  reads `bus.Source`. Sparse packets (publish policy gates them), so it is a real DR viewer.
- **Remote mode**: the viewer reads a **DDS** `IReplicationSource` (`SumoSharp.Replication.Dds`); the
  engine + publisher live in `Sim.Host.App`, a separate process. Only this line differs from local.
- **`ReplicationLaneShapeSource`** (consumer-side `ILaneShapeSource`): built from the **received
  geometry** topic, *not* by re-parsing the net — so local and remote consume geometry the same way and
  the decoupling is genuine (§8 step 1). Keeps `LaneShapeZ` for elevation. (First task confirms whether
  `IReplicationSource` exposes geometry as walkable polylines directly or needs a ~30-line adapter; the
  DDS side already has `DdsGeometryLaneSource` as the pattern.)
- **Playout delay**: a stable, small manual delay (default ~0.3–0.5 s), **not** auto-driven (§10.1). DR
  smoothing + DR-error publishing carry the quality.

### Procedural scene generation
All geometry is a pure function of the network the viewer already has, seeded-deterministically (no
`System.Random` without a fixed seed — same repo rule). There are two clocks: static geometry (roads,
buildings, TL poles) is built **once at load** and never moves; only cars update per frame. The viewer
never needs the `.net.xml` on disk — it builds from the `GeometryCodec.LaneGeo` polylines the host
published, which is what makes local and remote identical.

#### Coordinate mapping
SUMO is 2D right-handed with `x`=east, `y`=north, plus navi-heading (0°=North, clockwise); Godot 3D is
Y-up. So there is a single fixed transform SUMO `(x, y, z)` → Godot `(x, z, -y)` and a heading conversion
(navi-deg → Godot yaw radians), applied in exactly one place. Getting it wrong mirrors/rotates everything
together, so it is the first thing the headless self-test pins down (T1.2 checks yaw within 1°).

#### Roads
Each lane arrives as a polyline (list of `(x,y)`) plus a true `Width`. To turn a centerline into a
surface:

1. for each segment compute the 2D unit normal (perpendicular to the segment direction);
2. offset the centerline vertices ±`Width`/2 along the normal to get left/right edges;
3. at interior vertices use the miter normal (normalized average of the two adjacent segment normals) so
   consecutive quads meet without a gap or overlap on curves — a length cap on the miter avoids spikes at
   very sharp bends, fine for a demo;
4. emit two triangles per segment into an `ArrayMesh`.

Junctions are just the internal (`:`-prefixed) lanes — they carry their own curved polylines, so the same
ribbon routine fills the junction interior (turning paths) with no special case. Dark asphalt material;
optional thin lane-edge line ribbons are polish, not core. Ribbon vertices take `z` from `LaneShapeZ`
(flat nets → constant `z`, `city-organic-L2` → real relief). Sanity check (T1.3): total ribbon area ≈
Σ(lane length × lane width) within a few percent.

#### Buildings
Roads give where *not* to build; buildings fill the space beside them. Per non-internal edge:

1. take the edge's lane-group polyline and its total half-width (sum of its lanes' widths / 2);
2. march along the polyline at a stride (~15–25 m); at each step offset outward by `halfWidth + setback`
   on both sides;
3. place a box whose footprint (width along the road, depth away from it) and height come from a seeded
   hash of `(edgeId, stepIndex, side)` — varied but stable across runs; heights in a believable band
   (~6–60 m, i.e. 2–20 storeys);
4. orient the box to the local road tangent so façades face the street;
5. skip steps near junctions (leave corners open) and skip if a footprint would collide with an
   already-placed neighbor on that edge.

Building bases sit on the sampled ground `z` under their footprint (from `LaneShapeZ`), so elevation
follows the terrain. Thousands of buildings go into a `MultiMeshInstance3D` — one unit-cube mesh, one
per-instance transform, one draw call. Variety without assets = scale + a small palette of seeded
materials. Determinism check (T1.4): two runs → identical instance counts and transforms.

#### Traffic lights
The network exposes which connections are TL-controlled and their link indices; the replication stream
carries the live per-approach signal letter (r/y/g/G via `TlCodec`).

1. For each controlled approach (grouped so we get one head per incoming lane, not one per connection),
   find the stop-line point — the downstream end of the incoming lane's polyline, offset to the lane
   edge;
2. place a simple pole (thin cylinder/box) and a head (small box or three stacked spheres) at the top,
   facing back down the approach;
3. each frame set the head's emissive material from that approach's current signal letter → red/amber/
   green.

Only the material param updates per frame. Verification (T1.6): head colour index == replicated letter
every frame, and over a `09-traffic-light` run observe red→green and green→(amber→)red transitions (sim
really drives the lights).

#### Cars
The only per-frame geometry. All vehicles share a single `MultiMeshInstance3D` of a unit box; each frame
resize the instance buffer to the live vehicle count and write one transform per car:

- **Scale** = (`Length`, `Width`, height) with `Length`/`Width` the real vType dims (a truck is visibly
  bigger), height a believable constant (~1.4 m for cars, taller for trucks by vClass);
- **Position** = reconstructed `(x,y)` from `DrClock`→`PoseResolver`→`DrPoseSmoother`, `z` from
  `LaneShapeZ`;
- **Yaw** = reconstructed heading; **pitch** = z-gradient along travel (tilt on ramps); **roll** ≈ 0.

Smoothness is entirely the `Viewer.Motion` story; the `MultiMesh` just draws where reconstruction says.
One draw call lets `city-mixed-1k` (~1k) and the ladder to 15k render at interactive rates. Believable
scale throughout (cars ~4.5×1.8 m, true lane widths, buildings tens of metres).

#### Determinism & scale
No unseeded randomness (repo rule): every "random" footprint/height/material comes from a hash of stable
ids + a single scene seed — reproducible and independent of thread order. Static geometry is built once,
cars per-frame; buildings and cars both use `MultiMesh` so vehicle count and city size scale to the perf
ladder without a draw-call explosion. `LaneShapeZ` is threaded through roads, buildings, and cars alike,
so the multi-level capability is real, not bolted on. Because generation is a pure function of the
received geometry, the generator is scenario-agnostic — swap the scenario and the whole city regenerates;
the perf demo is just "same code, bigger net + more `--spawn`".

### Scale / scenario switching
The viewer (local) and `Sim.Host.App` (remote) both take `--scenario <path>` (+ `--spawn N`). Nothing is
hardcoded. Shipped defaults in the run scripts: **dev** `scenarios/09-traffic-light` (tiny, signalized so
lights do something early); **headline perf** `scenarios/_bench/city-mixed-1k` (~1k vehicles, signalized);
**perf ladder** `city-30 → city-300 → city-3000 → city-15000`; **elevation showcase**
`scenarios/_bench/city-organic-L2` (multi-level → exercises `LaneShapeZ`). All already committed.

### Multi-channel video wall (remote, mandatory second goal)
The remote viewer defaults to a **single channel** (one window/camera) for simplicity. `--video-wall`
auto-detects screens (`DisplayServer.GetScreenCount()`) and opens one borderless `Window` per screen,
each with its own `Camera3D` (different districts / follow different vehicles), all fed by **one** DDS
subscription in **one** process — zero-config on a wide desktop. Because each viewer is *just* a DDS
subscriber, the same build also supports launching **N independent viewer processes** (even on separate
LAN machines) against one `Sim.Host.App` — the "several remote viewers, one headless host" story, for
free. Honest caveat: DDS zero-config discovery is LAN/loopback multicast; cross-subnet needs a Cyclone
peer config (a documented knob, not a redesign).

---

## Local package feed

`demos/City3D/build.sh` (bash; documented in the demo README):

1. `dotnet pack` the needed packages at the repo's `Directory.Build.props` version into
   `demos/City3D/local-nuget/`:
   - **local viewer needs (pure C#)**: `SumoSharp.Core`, `SumoSharp.Ingest`, `SumoSharp.Replication`,
     `SumoSharp.Viewer.Motion`, `SumoSharp.Host`.
   - **remote adds (native)**: `SumoSharp.Replication.Dds`.
2. `dotnet restore` the demo — resolves `SumoSharp.*` from the local feed via `nuget.config`
   `packageSourceMapping`; the Godot SDK / BCL / CycloneDDS come from `nuget.org`.
3. `dotnet build` the demo.

`nuget.config` (committed):

```xml
<configuration>
  <packageSources>
    <add key="nuget.org" value="https://api.nuget.org/v3/index.json" />
    <add key="sumosharp-local" value="./local-nuget" />
  </packageSources>
  <packageSourceMapping>
    <packageSource key="sumosharp-local"><package pattern="SumoSharp.*" /></packageSource>
    <packageSource key="nuget.org"><package pattern="*" /></packageSource>
  </packageSourceMapping>
</configuration>
```

`local-nuget/` and Godot's caches (`.godot/`, export output) are **git-ignored** — ephemeral,
regenerated (committed-vs-ephemeral rule). Committed under `demos/City3D/`: only `.cs`, `.tscn`,
`project.godot`, `build.sh`, `nuget.config`, `README.md`, `.gitignore`.

**Alternative feed (documented, not default):** download the `nupkgs` artifact from a `pack-check.yml`
CI run and point `sumosharp-local` at it. The README notes this "from GitHub Action artifacts" path; the
`dotnet pack` script is the default because it is offline, cred-free, and lets you iterate on the packages
locally without a round-trip through GitHub.

---

## What is verified where (headless VM vs desktop)

| Verifiable **here** (headless Linux, no GPU) | Verifiable **only on a desktop** (GPU) |
|---|---|
| `build.sh` packs; `dotnet restore` resolves `SumoSharp.*` from the local feed and **fails** if the feed is absent (proving no `nuget.org` fallback) | The actual rendered 3D image — roads/buildings/cars/lights on screen |
| `SumoSharp.Host` unit self-test: publish → consume → reconstruct sane records | The video-wall spanning multiple monitors |
| `Sim.Host.App` runs, steps a scenario, publishes over DDS/inmem | Interactive camera / believable-scale eyeballing |
| A **headless** reconstruction self-test: poses advance smoothly, bounded, no back-jumps | |
| Godot C# assembly builds; `godot --headless` runs a scripted "N frames → log → quit" smoke | |
| host↔subscriber **DDS loopback** smoke on the VM *if* container multicast works — else same-process + in-memory fallback (the implementor reports which actually ran) | |
| `dotnet test Traffic.sln` green + `Sim.Bench` hash unchanged after the `src/` stages | |

The design's contract is that **everything except the on-screen pixels is provable in this environment**;
the pixels are the user's desktop check (Godot runs first-class on Linux, so a Linux desktop works too).

---

## Non-goals / future

- **Not a package**, the Godot app: `IsPackable=false`, build-from-checkout. Only the reusable libraries
  are packaged.
- **No DDS in the local viewer**: it drags a native binary into what must read as the clean "any C#
  engine consumes SumoSharp" reference and would mislead an integrator. DDS enters only at remote.
- **No new transport** (WebSocket/pipe/ENet): DDS is the shipped remote binding; a new one is out of
  scope though the contract allows it.
- **Promotion of `SumoSharp.Host` to the published set** and a **two-process DDS demo across machines**
  are natural follow-ups, deliberately left past this work.
