# SUMOSHARP-API.md — Library / NuGet packaging & public-API design of record

This is the design of record for turning the engine into **SumoSharp**, a distributable .NET
library (NuGet) that a hosting application — a game engine, an RL/training pipeline, or a
digital-twin backend — can load, drive, and read from at runtime.

It is a companion to `DESIGN.md` (which owns the *simulation* architecture and the four seams).
Where this doc says "additive" or "isolated," it means: **does not touch the vehicle hot loop,
the car-following / lane-change / junction math, or the internal vehicle SoA**, and therefore
cannot move any scenario out of its committed `tolerance.json`. The parity suite stays the
correctness anchor throughout (`CLAUDE.md` rule 3).

Status: **design agreed; most of Phase 1 LANDED on this branch** (async runner + string-API removal
pending). Implemented and verified (full parity suite green, 241 tests, determinism anchor
`909605E965BFFE59` unchanged single + parallel — so every addition is byte-identical where the new
paths are unused):
- the handle-based struct-of-arrays **obstacle store** (§4.3–4.4);
- the host-facing **stepped read surface** (§5): `Step()`/`Step(int)`, the columnar SoA spans
  (`VehicleHandles`/`PosX`/`PosY`/`PosZ`/`Angle`/`Speed`/`LaneHandles`/`Pos`/`PosLat`), and
  `TryGetVehicle` — a projection published each `Step()`, off the `Run()`/parity path (zero overhead there);
- **runtime demand** (§9): `LoadNetwork`, `DefineVType`/`VTypeParams`/`VTypeHandle`, `SpawnVehicle`
  (edge-list and from→to overloads) with SUMO-parity **queued insertion**, `GetLifecycle`, `Despawn`,
  `SetDestination`, `Reroute` — all over mutable vType/route registries seeded identically from the
  loaded demand;
- **geometry-3D** (§6): lane-shape `z` ingestion (`Lane.ShapeZ`) + the `PosZ` read column via
  `LaneGeometry.ElevationAtOffset` (0 on 2-D nets);
- the **lifecycle event buffer** (§10): `Engine.Events` (`Departed`/`Arrived`), diffed each `Step()`;
- the **async `SimulationRunner`** (§7): background-thread loop, boundary-applied command dispatcher
  (`Post`/`Invoke`), and an immutable double-buffered `SimulationSnapshot` (`Start`/`Stop`/`Pause`/
  `Resume`/`SpeedMultiplier`, or manual `Tick()`).

**Coordinated with `docs/LANELESS-DIRECTION.md`** (the laneless/RVO branch) — see §15 for the shared
obstacle-store ownership split, the lateral-state API requirements folded in, and the merge order.

---

## 0. Hard constraints (the rails this design runs on)

1. **The vehicle internals are frozen.** The SUMO-vehicle data structures underwent heavy
   optimization; reorganizing them is high-risk and out of scope. Every feature below is either
   a *new isolated subsystem* (obstacles), a *facade over fields that already exist* (vehicle /
   lane / vType handles), or a *mirror produced in the Export phase* (the read columns). None
   reshapes vehicle storage.
2. **Parity is the iron law.** Additive, inert-when-absent, byte-identical to the committed
   goldens on every existing scenario.
3. **The current host-obstacle API is not shippable.** It allocates on the hot path (see §4) and
   will be redesigned before any release.
4. **Handles are the currency of the hot path; strings are for setup/debug only.**

---

## 1. Package layout & naming

| Package | Contents | Ships? |
|---|---|---|
| **`SumoSharp.Core`** | `Sim.Core` + `Sim.Ingest` (engine + net/rou/sumocfg parsers) | ✅ the one package most users install |
| **`SumoSharp.Runtime`** *(optional split)* | the async `SimulationRunner` + command queue (§7) | ✅ separate so headless training builds can avoid the threading they don't want |
| **`SumoSharp.Tools`** | SUMO-binary fetch + `netconvert`/`duarouter` wrappers (§2) | ✅ optional, dev-time only |
| `Sim.Harness` | FCD parsing + tolerance comparison | ❌ `IsPackable=false` (test-only; maybe later `SumoSharp.Testing`) |
| `Sim.Run`, `Sim.Viz`, `Sim.ExtDemo`, benches | CLIs / demos | ❌ shipped as **samples**, not packages |

**Name:** `SumoSharp` (verified clear at time of writing). "SUMO" is an Eclipse trademark, so the
package README must carry: *"Unofficial, independent C# reimplementation of Eclipse SUMO's
microscopic simulation core. Not affiliated with or endorsed by the Eclipse SUMO project."*

**Packaging metadata (all packages):** `PackageId`, `Authors`, `Description`,
`PackageTags` (`sumo;traffic;microsimulation;ecs;simulation;games`), `PackageReadmeFile`,
`PackageProjectUrl`, `RepositoryUrl`, package icon, deterministic build,
`Microsoft.SourceLink.GitHub`, symbol package (`.snupkg`).

### License (adoption-critical — read before first publish)

SumoSharp is a **derivative work of SUMO**, so it inherits **`EPL-2.0 OR GPL-2.0-or-later`**
(already set in `Directory.Build.props`). This **cannot** be relicensed to MIT/Apache.

Practical read for a commercial adopter (**not legal advice — get counsel for commercial use**):
EPL-2.0 is **weak, file-level copyleft**. A proprietary game/app **may** link SumoSharp and keep
its *own* source closed; it must keep the **SUMO-derived files** under EPL and publish
modifications *to those files*. State this plainly and high in the README — many game developers
bounce at the bare word "GPL" without realizing the EPL option permits commercial linking.

---

## 2. SUMO binary tooling — download-on-demand, kept out of Core

The engine **never** depends on a SUMO install (that is the whole hermetic-test philosophy).
Network *authoring* (OSM → `.net.xml` via `netconvert`) does need SUMO, so it lives in a separate,
optional package.

- **`SumoSharp.Tools`** (a package, or better a `dotnet sumosharp` global tool) fetches the
  official Eclipse SUMO binaries for the current OS **on first use** and shells out to
  `netconvert` / `duarouter` / `polyconvert`.
- **Download, do not bundle.** Binaries are large and per-platform; redistribution is cleaner
  avoided. Cache into `~/.sumosharp/<SUMO_VERSION>/`, keyed and **checksum-verified** against the
  pinned `SUMO_VERSION`.
- **Multi-platform reality (Windows + Linux):**
  - **Windows:** official self-contained release **zips** — download, unzip, run the `.exe`s. Turnkey.
  - **Linux:** official distribution is the `sumo` apt/PPA package or a source build — there is no
    universal portable tarball across distros. So the Linux path is "use the system-installed
    `sumo` if present, else guide the user to `apt-get install sumo`," **or** a **Docker image**
    wrapping `netconvert` for CI/headless. Be honest in docs that Linux is less turnkey than Windows.
- Concern split: **Core** = "simulate a network at runtime" (no SUMO). **Tools** = "build a
  network" (needs SUMO). A game ships only Core + a pre-baked `.net.xml`; a content pipeline uses Tools.

---

## 3. Framework targeting — net8.0 first, netstandard2.1 phased

- **First releases: `net8.0`.**
- **Then add `netstandard2.1`** to `SumoSharp.Core` to unlock **Unity** (Mono / .NET Standard 2.1)
  and Godot. Expect to guard a handful of net8-only APIs behind `#if NET8_0_OR_GREATER`
  (some `System.Text.Json`, `Half`, certain `Span`/generic-math paths, `[SkipLocalsInit]`).
- **The performance-critical surface needs no net8-only feature.** `Span<T>`/`ReadOnlySpan<T>`/
  `Memory<T>` are available on netstandard2.1 (via `System.Memory`), so the non-allocating obstacle
  and read APIs (§4, §5) are **one API** for Unity and net8 alike — net8 just JITs it faster. No
  API bifurcation. Multi-target *after* the API surface stabilizes, so we multi-target a stable API.

---

## 4. Handles & the obstacle subsystem redesign

### 4.1 Handle model — mirrors the host engine's 48-bit convention

```csharp
public readonly struct ObstacleHandle   // and VehicleHandle, same shape
{
    public readonly uint   Index;        // 32-bit: direct slot into the SoA arrays
    public readonly ushort Generation;   // 16-bit: version counter, matched O(1) vs the live slot
}
```

- **48-bit total (32 index + 16 generation)** — identical layout to the host game engine's entity
  handle, so it reads/stores identically there.
- **Distinct id space.** A SumoSharp handle addresses a slot in the *sim* engine, **not** the host
  ECS. The host maps between the two spaces (the same place the string→id cache lives). Same shape,
  different registry — never silently interchange them.
- **String ids are dropped from the engine hot path entirely.** Any string→handle caching is a
  host-side convenience layer. Strings remain only for scenario authoring / debugging / lookup.
- **16-bit generation wraps at 65,536 slot recycles** — the exact accepted property the host engine
  already lives with. Named here so it is understood, not discovered.

### 4.2 Why the current obstacle API must be replaced

`ExternalObstacle` is a `sealed record` (a class) stored in `Dictionary<string, ExternalObstacle>`;
every `UpdateObstacle` does `obstacle with { … }` → **one heap allocation + one string-hash lookup
per obstacle per step**. At thousands of moving agents that is thousands of allocations/step and
real GC pressure. Unshippable for a high-perf engine.

### 4.3 New store — direct-mapped struct-of-arrays + a dense active-index list

Matches the host convention ("Index points directly to the entity's slot in the internal arrays"):

- **Data columns indexed *directly* by `handle.Index`:** `laneHandle[]`, `frontPos[]`, `length[]`,
  `startTime[]`, `endTime[]`, `speed[]`, `maxDecel[]`, `latPos[]`, `width[]`, `latSpeed[]`,
  `avoidanceClass[]` (see next bullet), plus a `generations[]` (`ushort`) and a free-list of recycled
  indices.
- **Reserve a per-agent `avoidanceClass` byte now (coordinated with the RVO layer, §15 B1):** enum
  `StaticBlocker` / `OneSided` / `Reciprocal`, **default `OneSided`**. It is inert for the lane-based
  engine (a car still just stops/swerves behind any obstacle regardless), but the RVO solve reads it to
  pick the reciprocity `share`: `1.0` (one-sided) for a dumb obstacle, `0.5` for a cooperative
  navmesh/RVO agent that avoids reciprocally. Reserving the column now avoids a store change when the
  RVO layer's Stage 3 lands. Exposed as an optional `AddObstacle`/`AddMovingObstacle` argument
  (default `OneSided`, so every existing call site is unchanged).
- **`UpdateObstacle(handle, …)`** = check `handle.Generation == generations[handle.Index]`, then
  write by index. **O(1), zero-alloc, no hash, no indirection.** A stale/dead handle fails the
  generation check → cheap no-op (preserves the "inert-when-absent" contract).
- **Dead slots leave holes**, so the engine's per-step scan walks a separate **dense
  `activeIndices` list** (append on add, swap-remove on remove) for cache locality. Direct-mapped
  for O(1) host updates; dense list for the sim's own iteration.
- **No `z` on obstacles** — they are lane-relative, so elevation derives from the lane like a vehicle's.

### 4.4 API shape

```csharp
int          GetLane(string laneId);                 // resolve ONCE at setup → int lane handle
ObstacleHandle AddObstacle(int laneHandle, float frontPos, float length, /* + lat/time/speed */ …);
void         UpdateObstacle(ObstacleHandle h, float frontPos, float speed, float latPos, float latSpeed);
void         RemoveObstacle(ObstacleHandle h);       // bumps the slot generation

// Batch path for the many-thousands case (netstandard2.1-clean; one frame of crowd corrections):
void UpdateObstacles(ReadOnlySpan<ObstacleHandle> handles, ReadOnlySpan<ObstacleUpdate> updates);
```

**Isolation:** this swaps only the `_obstacles` backing store and its ~3 read sites
(`AdvanceObstacles`, `ObstacleConstraint`, the neighbor scan). It feeds the existing multi-constraint
reducer (seam 1). The vehicle SoA and all car-following/lane-change/junction code are untouched.

---

## 5. Read API — SoA-primary, mirrored in the Export phase

The read columns are a **projection refreshed each step**, not the engine's source of truth. The
Export phase already builds a `VehicleExportSnapshot` per active vehicle per frame; it *also* writes
these parallel columns. This is why the read surface is additive and never a vehicle-storage refactor.

```csharp
ReadOnlySpan<VehicleHandle> Handles { get; }   // 32-bit index + 16-bit generation
ReadOnlySpan<float>  PosX  { get; }
ReadOnlySpan<float>  PosY  { get; }
ReadOnlySpan<float>  PosZ  { get; }             // present from day one (see §6)
ReadOnlySpan<float>  Angle { get; }             // heading / yaw
ReadOnlySpan<float>  Speed { get; }             // render-facing
ReadOnlySpan<int>    LaneHandle { get; }
ReadOnlySpan<double> PosLat { get; }            // lane-relative lateral offset; parity-exact (see note)
// optional: LatSpeed (double), SlopeDeg/Pitch (from the z-gradient) for tilting vehicles on ramps

bool TryGetVehicle(VehicleHandle h, out VehicleState state);   // random access; readonly struct, no alloc
```

**STATUS: landed** (`src/Sim.Core/VehicleHandle.cs`, `VehicleState.cs`, `VehicleReadBuffer.cs`; the
`Step()`/spans/`TryGetVehicle` surface on `Engine`). `PosLat` is exposed today (from `Kinematics.LatOffset`);
`LatSpeed` is deferred to the laneless-branch merge (it owns `Kinematics.LatSpeed`). `PosZ` ships as an
all-zero column until geometry-3D lands. The columns are populated only by `Step()`, never by `Run()`.

- **`PosLat` is first-class (coordinated with the laneless branch).** The sublane/laneless axis makes
  lateral state load-bearing — that branch already added `VehicleExportSnapshot.PosLat`,
  `TrajectoryPoint.PosLat`, and `Kinematics.LatSpeed`. Lateral position *is* visible in the derived
  `PosX/PosY`, but hosts that consume **lane-relative** state (and the sublane goldens) need `posLat`
  **directly**, so it is an explicit **`double`, parity-exact** column (`LatSpeed` likewise available).
  See §15.

- **SoA is primary** (matches the host engine; GPU-vertex-buffer / DOTS-friendly). A client that
  wants array-of-structures converts on its side.
- **`ReadOnlySpan<T>`** ⇒ one API on Unity (netstandard2.1) and net8.
- **Precision split (agreed):** derived **render** columns (`PosX/Y/Z`, `Angle`) are **`float`**
  (half the bandwidth, wider SIMD, GPU-native). Keep **`double`** accessors for the parity-exact
  lane-relative values (`pos`, `speed`, **`posLat`, `latSpeed`**) for training / digital-twin consumers
  and the sublane goldens that re-ingest state.
  *Float for "where to draw it," double available for "what the sim actually computed."*
- **Validity contract:** spans are valid **until the next `Step()`**. In async mode the runner
  double-buffers these column arrays (§7).

---

## 6. 3D / multi-level roads & tunnels

**Today the network is fully 2D** — `ParseShape` reads `"x,y"` pairs into `(double X, double Y)`;
`LaneGeometry.PositionAtOffset` returns `(X, Y, AngleDeg)`; `z` is dropped at ingestion.

**SUMO does not do volumetric 3D.** Multi-level roads, overpasses, bridges and tunnels are:
1. **Topology** — roads at different levels simply have *no connection* between them (already fully
   expressible today), and
2. **Per-vertex elevation `z`** on the lane/edge shape (`shape="x,y,z …"`), which is **purely
   geometric** — it feeds rendering (and optionally slope), while the dynamics stay lane-relative 1-D
   arc-length. x/y/z are all *derived* by walking the polyline.

### Geometry-3D — additive, safe, do it now
- **Ingest:** parse the optional 3rd shape component; default `z = 0` when absent.
- **Derive:** interpolate `z` along the polyline exactly like x/y; optionally derive slope/pitch.
- **Read:** the `PosZ` column (§5) — one always-derived float; **0 on 2-D nets, real on 3-D nets**.
- **Because `z` never feeds car-following, adding it is byte-identical on existing 2-D scenarios**
  and cannot perturb parity or performance. Tunnels need only geometry + (optionally) a semantic tag
  for renderer culling.

### Dynamics-3D — a separate, future, opt-in extension
Slope-affected acceleration (grade resistance) is a *behavioral* change (today only the rail model
touches grade, pinned flat). Keep it **decoupled** from geometry-3D: gated, opt-in, never bundled
with the free/safe geometry work.

---

## 7. Execution model — one deterministic core, two host-facing modes

**The engine is always deterministic and single-threaded internally**; it only ever advances via
`Step()`. Async is a *wrapper*, not a different engine.

### Stepped mode
Host calls `Step()` and reads the spans on the **same thread, between steps**. Single-threaded
contract. Best for training (reproducibility), lockstep, and deterministic replay.

### Async mode — `SimulationRunner`
Owns a background thread + the engine; the host **never touches `Engine` directly**. Two lock-free
structures mediate access:

- **Host command queue (host → runner):** zero-alloc `readonly struct Command` (a `CommandKind`
  discriminator + packed payload) in a preallocated ring buffer. Drained at the **start of a step**,
  in **FIFO enqueue order**.
- **Published snapshot (runner → host):** after each step the runner publishes/double-buffers the
  read columns (§5) + the event buffer (§9); the host reads the latest lock-free.

**Two buffers at two levels — keep straight:**
- the **host command queue** crosses the *thread* boundary;
- the existing **engine command buffer** (seam 4) handles *intra-step* structural deferral.
  A host command → drained at the boundary → becomes an engine-command-buffer op → flushed in the
  normal Execute phase.

**Determinism contract:**
- **Single-writer is the reproducible contract.** One producer ⇒ FIFO order is deterministic ⇒ same
  inputs, same trajectory, even async. Multiple producers ⇒ the *merged* order depends on thread
  timing (sim math still deterministic; only *when* inputs land varies) — so document "single writer,
  or you lose input-order determinism."
- **Apply-at-boundary in *both* modes.** Route even stepped-mode commands through the queue and drain
  at the boundary, so stepped and async produce **identical** results.

**Time control (async):** target tick rate, pause/resume, single-step, **speed multiplier**
(faster-than-real-time for digital-twin catch-up / training warm-up). Fixed-timestep accumulator with
a **clamp on max steps per wall-frame** (avoid the spiral of death when the host stalls).

**Interpolation hook:** publish the **last two** snapshots + sim timestamps so a host rendering at
120 fps over a 10 Hz sim can lerp position/angle. Cheap, and the difference between "juddery" and
"shippable" for the browser-live demo (§10).

**STATUS: landed** (`src/Sim.Core/SimulationRunner.cs`, `SimulationSnapshot.cs`). `SimulationRunner`
wraps a loaded `Engine`: `Post(Action<Engine>)` / `Invoke<T>(Func<Engine,T>)` enqueue mutations drained
FIFO at the start of each `Tick()` (boundary contract); `Tick()` then `Step`s and publishes an immutable
`SimulationSnapshot` (SoA columns + `SpeedExact` double + events + time/step, self-contained so the host
reads it from any thread). `Start(hz)` runs `Tick()` on a background thread with `Pause`/`Resume` and a
`SpeedMultiplier` (fixed-rate pacing, resync-not-spiral when behind); `Tick()` is also callable manually
for deterministic stepping. Notes: snapshots are **allocated per Tick** for now (a snapshot pool is the
documented perf refinement); the two-frame **interpolation hook** above is not wired yet; `Invoke` runs
inline in manual mode and blocks for the result in threaded mode (avoid calling it while `Paused`).

---

## 8. Threading contract (summary)

| Mode | Writes | Reads |
|---|---|---|
| **Stepped** | same thread as `Step()`, between steps | same thread, spans valid until next `Step()` |
| **Async** | host → thread-safe **command queue** only | host → immutable **published snapshot** only |

The engine thread exclusively owns the `Engine` in async mode ⇒ no shared mutable access, no locks
in the host hot path.

---

## 9. Runtime spawn / reroute / vType — full dynamic control

The network comes from netconvert, but **everything that moves is runtime-constructed** (not just
predefined scenarios). All of this builds on machinery that already exists.

```csharp
VTypeHandle DefineVType(in VTypeParams p);                 // immutable once defined (mirrors ResolvedVType)
                                                           // VTypeParams MUST include the sublane params,
                                                           // runtime-settable, defaulting (see §15):
                                                           //   maxSpeedLat = 1.0, latAlignment = "center",
                                                           //   minGapLat   = 0.6

// Explicit route OR origin→destination (routed via the existing NetworkRouter)
VehicleHandle SpawnVehicle(VTypeHandle t, ReadOnlySpan<int> routeEdges,
                           float departPos, float departSpeed, int departLane);
VehicleHandle SpawnVehicle(VTypeHandle t, int fromEdge, int toEdge, …);

void SetDestination(VehicleHandle v, int toEdge);          // recompute route from current edge
void Reroute(VehicleHandle v, ReadOnlySpan<int> avoidEdges);
void Despawn(VehicleHandle v);                             // structural → command buffer; bumps generation
```

- **`CreateRuntime(VehicleDef)`** (already used by `LoadSnapshot` to materialize flow vehicles) **is
  the spawn primitive**; `SpawnVehicle` is a public, handle-returning wrapper applied at the step
  boundary via the command buffer (seam 4).
- **`NetworkRouter.Route(from, to[, avoid])`** already exists ⇒ routing / reroute is largely built.
- **`SaveSnapshot`/`LoadSnapshot` double as training reset** (`env.reset()` for free).
- **Network-only load path:** add `LoadNetwork(net[, cfg])` (path **and** stream overloads) that
  loads geometry with **zero demand**, so a host starts empty and spawns everything itself. This is
  the "not just predefined scenarios" entry point. (Stream/string overloads also let a game embed
  scenarios from asset bundles rather than loose files.)
- **Surface the lateral engine modes through the facade/config** (coordinated with the laneless
  branch, §15): `ScenarioConfig.LateralResolution` (the sublane master switch) and `Engine.LanelessRvo`
  (the continuous velocity-obstacle avoidance mode). Both are opt-in and byte-identical when off, so
  they belong on the public config surface without perturbing the default lane-based path.
  **Document the coupling (§15 B2):** `LanelessRvo` currently takes effect **only when
  `LateralResolution > 0`** (nested under the sublane gate to keep phase-1 byte-identical). The facade
  must state this dependency; it may later be promoted to an independent continuous-lateral mode.

**STATUS: landed.** `LoadNetwork(net[, cfg])`, `DefineVType(VTypeParams) → VTypeHandle` (+ `DefaultVType`/
`TryGetVType`), `SpawnVehicle(...)` (edge-list and from→to), `GetLifecycle`, `Despawn`, `SetDestination`,
`Reroute` are implemented on `Engine`, backed by mutable `_vTypesById`/`_routesById` registries seeded
from `_demand` at load. Notes on this branch: edges are addressed by **SUMO edge-id string** (the router
and edge model are string-keyed — a dense `GetEdge(string)→int` is a possible later refinement, not
required); `VTypeParams` omits the **sublane** lateral attributes (`maxSpeedLat`/`latAlignment`/`minGapLat`)
until the laneless-branch merge that adds them to `VType`; `SetDestination`/`Reroute` operate on **active**
vehicles (a pending vehicle is respawned with the intended route); the despawn slot is **not** recycled yet
(EntityIndex stays stable), and the lifecycle **event buffer** (§10) is deferred — poll `GetLifecycle` for now.

### Insertion semantics — SUMO-parity queued insertion (decided)

`SpawnVehicle` returns a handle **immediately in a `Pending` state**; the vehicle enters the road
when insertion succeeds (space at the depart position), emitting a `Departed` event — or an
`InsertionFailed` event if it never fits / times out. Handle state (`Pending`/`Active`/`Arrived`) is
queryable. This matches SUMO's own insertion/queuing model and how flow/demand vehicles already
behave, so runtime-spawned vehicles stay parity-consistent. (Rejected alternative: synchronous
immediate try-insert, which diverges from SUMO and pushes retry logic onto the host.)

---

## 10. Lifecycle events — a drained event buffer (not C# events)

The host must know when a vehicle **departed / arrived / insertion-failed / teleported** (to release
its game-side entity, retry a spawn, etc.). C# `event`/delegates allocate and cross threads awkwardly.
Instead publish a **per-step event buffer** alongside the snapshot, drained by the host each frame:

```csharp
ReadOnlySpan<SimEvent> Events { get; }   // { VehicleHandle handle, SimEventKind kind }
```

Zero-alloc, thread-clean (rides the same double-buffer as the read columns).

**STATUS: landed** (`src/Sim.Core/SimEvent.cs`; `Engine.Events`). Emitted by diffing each vehicle's
lifecycle every `Step()` (in `PublishReadState`, so `Run()` never pays for it): **Departed**
(Pending→Active) and **Arrived** (route completion, and despawn). `InsertionFailed`/`Teleported` are
defined in `SimEventKind` for API stability but not emitted yet (the engine queues indefinitely and
teleport is off in phase 1). Despawn surfaces as `Arrived` with a stale-generation handle (the host
initiated it, so it already holds the correlation).

---

## 11. Example applications

- **Browser-live interactive demo (primary showcase).** A minimal `SumoSharp.LiveHost` ASP.NET Core
  sample: each tick calls `Step()`, serializes the live SoA read columns to a compact frame, and
  streams them over a **WebSocket** at ~10–20 Hz (decoupled from sim `dt`). **Reuse `Sim.Viz`'s
  existing `template.html`/`template.js` renderer**, refactored to consume *live* frames instead of a
  baked `REPLAY_DATA` blob: send static network geometry once on connect, per-frame moving state only.
  Cross-platform for free (browser), zero-install, and doubles as a debugging tool.
  - **Killer interaction:** click the canvas → project screen x/y to nearest **lane + pos** → send an
    `AddObstacle`/`SpawnVehicle` command back over the socket. Watching SUMO cars stop/swerve/reroute
    around an obstacle *you just dropped* is the demo that sells the library.
  - **New code required:** the **screen→lane projection** (inverse of the x/y derivation — nearest-lane
    point projection) for obstacle/vehicle placement.
- **Keep the offline `replay.html`** (FCD → self-contained HTML) for docs and golden inspection.
- **Later:** a Unity/Godot sample (gated on the netstandard2.1 multi-target, §3).

Each of the three target users falls out of the *same* core + wrapper:
- **Game engine:** async runner (non-blocking ambient traffic), SoA float reads (thousands of cars/
  frame), runtime spawn/despawn, obstacle injection, netstandard2.1.
- **Training (RL):** stepped deterministic, `LoadSnapshot` as reset, non-alloc columnar obs read,
  seeded per-entity RNG, headless Core-only.
- **Digital-twin:** async runner, feed live detections as obstacles / spawns, runtime reroute,
  snapshot for state injection, real-time or faster-than-real-time stepping.

---

## 12. Decisions log

| # | Decision | Rationale |
|---|---|---|
| D1 | Package name **SumoSharp**; keep `EPL-2.0 OR GPL-2.0-or-later`; "unofficial" disclaimer | Derivative of SUMO; name clear; EPL weak-copyleft usable commercially |
| D2 | SUMO binaries via a **separate optional `SumoSharp.Tools`**, download-on-demand, never bundled | Core stays SUMO-free/hermetic; binaries are large/per-platform |
| D3 | **net8.0 first, netstandard2.1 phased** (Unity/Godot later) | Ship sooner; multi-target a stable API; perf surface needs no net8-only feature |
| D4 | **Handles, not strings**, in the hot path; **32-bit index + 16-bit generation** (48-bit) | Matches host engine convention; zero-alloc; O(1) stale-ref detection |
| D5 | **Redesign the obstacle store** to direct-mapped SoA + dense active list; drop the record-dict | Current path allocates + hashes per update per step; unshippable |
| D6 | **SoA-primary read**, mirrored in Export phase; client converts to AoS | Matches host engine; GPU/DOTS-friendly; no vehicle-storage refactor |
| D7 | **Float** render columns (X/Y/Z/Angle) + **double** accessors for parity-exact pos/speed | Games/GPU want float; training/twin want double faithfulness |
| D8 | **Z in the read API from day one**; geometry-3D additive; dynamics-3D deferred/opt-in | XYZ API stable from 0.1; 3-D nets "just work"; parity-safe |
| D9 | **One deterministic core + async `SimulationRunner`** (command queue + double-buffered snapshot) | Both stepped and async from one engine; determinism preserved via boundary-applied commands |
| D10 | **Apply-at-boundary in both modes**; single-writer = reproducible | Stepped and async produce identical results |
| D11 | **Runtime spawn/reroute/vType + network-only load**; reuse `CreateRuntime` / `NetworkRouter` | "Not just predefined scenarios"; building blocks already exist |
| D12 | **SUMO-parity queued insertion** (`Pending`→`Departed`/`InsertionFailed`) | Matches SUMO + existing flow/demand behavior; parity-consistent |
| D13 | **Drained event buffer**, not C# events | Zero-alloc, thread-clean, rides the snapshot double-buffer |
| D14 | Primary demo = **browser-live WebSocket**, reusing the `Sim.Viz` renderer | Zero-install, multi-platform, interactive obstacle injection |
| D15 | **This (NuGet) session owns the obstacle-store redesign** (§4.3–4.4); the laneless/RVO layer consumes it via the neutral `RvoNeighbor` value abstraction | Avoid building the store twice; both branches independently converged on the same handle-based SoA |
| D16 | **Expose lateral state + sublane params** — `PosLat`/`LatSpeed` read columns, `maxSpeedLat`/`latAlignment`/`minGapLat` in `VTypeParams`, `LateralResolution`/`LanelessRvo` on the config/facade | The laneless branch made lateral state first-class; the public API must carry it or runtime-spawned vehicles can't do lateral behavior |
| D17 | **Reserve an `avoidanceClass` byte** (`StaticBlocker`/`OneSided`/`Reciprocal`, default `OneSided`) in the obstacle store now | The RVO solve needs per-agent reciprocity `share`; reserving now avoids a store reshape at Stage 3 |
| D18 | **Merge order:** store lands first, then RVO Stage 3's `ComputeRvoLateral` loop retargets onto it; RVO Stages 1–2 are order-independent | Agreed with the laneless session; `RvoNeighbor` is the sole seam |

---

## 13. Phased roadmap

**Phase 1 — the shippable 0.1 core (API facade + isolated redesigns).** *No "ship as-is" —
the obstacle API is redesigned first.*
- Handle model (§4.1); obstacle store redesign (§4.3–4.4); SoA read surface + float/double split (§5);
  `PosZ` column + z ingestion/derivation (§6 geometry-3D); public `Step()`; stream/string load
  overloads + `LoadNetwork` (§9); runtime spawn/reroute/vType with queued insertion (§9); event
  buffer (§10). Package metadata, SourceLink, symbols, README + license story (§1).
- Gate: full parity suite green + `Sim.Bench` determinism hash unchanged (everything additive/isolated).

**Phase 2 — the browser-live interactive demo (§11)** on top of the Phase-1 API, incl.
screen→lane projection and the WebSocket host reusing the `Sim.Viz` renderer.

**Phase 3 — reach & polish:** async `SimulationRunner` hardening (§7); `netstandard2.1` multi-target
+ a Unity/Godot sample (§3); `SumoSharp.Tools` binary-fetch tool (§2); publish to nuget.org + release CI.

---

## 14. Open items (not yet decided)

- Exact `VTypeParams` surface (which SUMO vType attributes are runtime-settable vs net-fixed) —
  **must include the sublane params `maxSpeedLat`/`latAlignment`/`minGapLat`** (§15, resolved).
- Whether `SumoSharp.Runtime` (async) is a separate package or folded into Core behind a namespace.
- Multi-writer command-queue policy if a host genuinely needs concurrent producers (per-tick sort key?).
- `SumoSharp.Tools` Linux delivery: system-`sumo` detection vs a pinned Docker image (or both).

---

## 15. Coordination with `LANELESS-DIRECTION.md` (the laneless/RVO branch)

Cross-checked against `docs/LANELESS-DIRECTION.md` on branch
`claude/sumo-phase-2-planning-p3w7kh`. Both branches fork from the same `main` and are design-level,
so **all overlaps are future** (this doc's Phase 1 vs that branch's staged RVO plan). The headline is
**strong alignment, not conflict** — and the shared subsystem has a clean owner.

### Shared conclusion (independently reached)
Both sessions concluded the string-keyed `ExternalObstacle` record-dictionary must be replaced by a
**handle-based struct-of-arrays store** (this doc §4.2–4.4 / D5). That branch's Stage 3 wants exactly
this as its "scalable int-indexed footprint-agent store." Its **lateral columns (`latPos`, `width`,
`latSpeed`)** — already in the §4.4 column list — are **load-bearing for the RVO/ORCA solve; keep them.**

### Ownership split (so the store is built once) — D15
- **This (NuGet) session owns the obstacle-store redesign** (§4.3–4.4, Phase 1).
- **The laneless/RVO layer consumes it** through a neutral value-typed neighbour abstraction
  (`RvoNeighbor`: pos/length/latOffset/halfWidth/speed — no strings, no dictionary), never touching the
  store directly. Until the SoA store lands, that branch reads the *current* `_obstacles` via a thin
  transitional adapter in `ComputeRvoLateral`; when the store lands, **only that adapter changes**, not
  the RVO solve. So neither side blocks the other, and the store is implemented exactly once.

### Incompatibilities this branch must honor (folded into this doc) — D16
1. **Read API (§5) exposes `PosLat`** (double, parity-exact) — added. `LatSpeed` likewise available.
   Lateral is visible via derived `PosX/PosY`, but lane-relative consumers and the sublane goldens
   need `posLat` directly. (That branch already added `VehicleExportSnapshot.PosLat`,
   `TrajectoryPoint.PosLat`, `Kinematics.LatSpeed`.)
2. **`VTypeParams`/`DefineVType` (§9) include the sublane params** `maxSpeedLat`, `latAlignment`,
   `minGapLat` — added. Without them, runtime-spawned vehicles cannot do sublane/laneless lateral behavior.
3. **Lateral engine modes surfaced on the config/facade (§9):** `ScenarioConfig.LateralResolution`
   (sublane master switch) and `Engine.LanelessRvo` — added. Opt-in, byte-identical when off.

### Two items reserved at the laneless session's request (confirmed)
- **B1 — `avoidanceClass` byte in the obstacle store (reserved now).** Enum `StaticBlocker` /
  `OneSided` / `Reciprocal`, default `OneSided`; added to the §4.3 columns and the
  `AddObstacle`/`AddMovingObstacle` signature. The RVO solve reads it to choose the reciprocity `share`
  (`1.0` one-sided for a dumb obstacle, `0.5` for a cooperative navmesh/RVO agent). Reserving the
  column now means the store never changes shape when RVO Stage 3 lands. Inert for the lane-based engine.
- **B2 — `LanelessRvo` is gated under `LateralResolution > 0`.** Documented on the facade (§9): the
  continuous mode currently only takes effect when the sublane master switch is on (keeps phase-1
  byte-identical). May later become an independent continuous-lateral mode.

### Confirmations (from the laneless session)
- The Stage-3 adapter needs `{laneHandle, frontPos, length, latPos, width, speed, startTime, endTime}`
  per active agent — **all already in the §4.3 columns**; `RvoNeighbor` maps directly (plus the new
  `avoidanceClass`).
- `DefineVType` defaults `maxSpeedLat = 1.0`, `latAlignment = "center"`, `minGapLat = 0.6`, all
  runtime-settable (folded into §9).
- `ToleranceConfig.PosLat` lives in **`Sim.Harness` (test-only)**, not `SumoSharp.Core` — so of the 8
  shared files below, `ToleranceConfig.cs` is *not* in the shipped package.
- The RVO layer has **no dependency** on the vehicle read API, the handle types, the spatial hash, or
  the spawn surface. The entire coordination reduces to one seam: **`RvoNeighbor` is the contract** —
  this session's store *produces* them, the laneless solve *consumes* them, neither reaches across.

### Merge order (agreed)
- **RVO Stages 1–2** (vehicle↔vehicle) don't touch the obstacle store → **merge-order-independent**,
  land whenever.
- **RVO Stage 3** is the only piece reading `_obstacles` (one clearly-marked transitional loop in
  `ComputeRvoLateral`). **This session's SoA store lands first; then that one loop retargets** onto it
  (builds `RvoNeighbor` from the SoA columns instead of the record). Until then Stage 3 stays on the
  current `_obstacles` — gated/byte-identical, so it blocks nothing.
- **STATUS: the store has landed** (`src/Sim.Core/ObstacleStore.cs`, `ObstacleHandle.cs`,
  `AvoidanceClass.cs`; `ExternalObstacle` is now a `readonly record struct`). It carries every column the
  Stage-3 adapter needs — `laneHandle/frontPos/length/latPos/width/speed/startTime/endTime` — plus the
  reserved `avoidanceClass` byte (B1/D17). The engine still materialises an `ExternalObstacle` value per
  active slot via `_obstacles.Values` (zero-alloc struct enumerator), so the RVO adapter can read the
  same struct today and switch to raw columns later with no behavioural change. `RvoNeighbor` remains the
  sole seam.

### Merge mechanics
Both branches make **additive** edits to 8 shared files: `Engine.cs`, `VehicleExportSnapshot.cs`,
`DemandModel.cs`, `DemandParser.cs`, `VTypeDefaults.cs`, `ScenarioConfig.cs`, `TrajectoryPoint.cs`,
`ToleranceConfig.cs`. Additions sit in distinct regions; **whoever merges second reconciles the
additive hunks.** Both sides hold the **same parity anchor** (determinism hash `909605E965BFFE59`,
goldens byte-identical); every change on both branches is additive/gated, so the merged result must
reproduce that anchor. That invariant is the shared acceptance gate for the merge.
