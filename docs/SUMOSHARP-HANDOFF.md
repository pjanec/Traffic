# SUMOSHARP-HANDOFF.md — session handoff

Continuation notes for the **SumoSharp** library/packaging effort. Pairs with
`docs/SUMOSHARP-API.md` (the design of record + landed-status per section) and
`docs/LANELESS-DIRECTION.md` (the sibling laneless/RVO branch).

## TL;DR

- **Branch:** `claude/sumo-csharp-nuget-strategy-4vlkki` (all work pushed).
- **Gates (must stay true after every change):**
  - `dotnet test` → **0 failed, 1 skipped**; the pass count grows as new-surface tests are added
    (**250** at the start of the packaging work → **253** after the B13 multi-target guard test).
  - `Sim.Bench` determinism hash → **`909605E965BFFE59`** (single **and** parallel).
- **What exists now:** the whole Phase-1 public API + NuGet packaging + a working browser-live demo.
  Every addition is *additive / inert-when-absent*, so it is byte-identical where the new paths are
  unused (that is why the hash never moved).
- **Prime rule (unchanged):** parity is the iron law (`CLAUDE.md`). The vehicle SoA and the
  car-following / lane-change / junction math are **frozen**; everything below is either a new isolated
  subsystem, a facade over existing fields, or a projection produced in the Export/Step path.

## How to build / test / run

```bash
dotnet build Traffic.sln
dotnet test  Traffic.sln                      # the offline parity gate (no SUMO needed)
dotnet run --project src/Sim.Bench -c Debug   # prints the determinism hash (expect 909605E965BFFE59)

# NuGet packages
dotnet pack src/Sim.Core/Sim.Core.csproj -c Release -o ./nupkgs
dotnet pack src/Sim.Ingest/Sim.Ingest.csproj -c Release -o ./nupkgs

# Browser-live demo (then open http://localhost:5055, click the road to drop an obstacle)
dotnet run --project src/Sim.LiveHost
```

Environment note: the .NET 8 SDK is **not** committed. On a fresh VM install it with
`sudo apt-get update && sudo apt-get install -y dotnet-sdk-8.0` (the pinned point-release debs go stale,
so `apt-get update` first). SUMO is not needed for `dotnet test`.

## What landed this session (commits, newest first)

| Commit | What |
|---|---|
| *(this session)* | **ns2.1 consumer sample** `samples/SumoSharp.GameHostSample` (multi-target net8.0/ns2.1; `GameHost` drop-in + runnable net8 demo; `RungB17`). |
| *(this session)* | **Dense edge handles** `GetEdge`/`GetEdgeId`/`EdgeCount` + int Spawn/route overloads (`0ceeaf0`, `RungB16`). |
| `3ac73c1` | **Async runner — snapshot pool (opt-in):** `EnableSnapshotPool(cap=3)` reuses backing arrays across Ticks (`RungB15`). Default off; contract unchanged when off. |
| `ce37400` | **Async runner — two-frame interpolation hook:** `PreviousSnapshot`, `InterpolationAlpha`, `TryInterpolateVehicle` → `InterpolatedVehicle` (`RungB14`). |
| `1a2d685` | **`netstandard2.1` multi-target** on `Sim.Core` + `Sim.Ingest` (Unity/Godot reach): polyfills (`src/Shared/NetstandardPolyfills.cs`), `System.Memory` on ns2.1, 4 net8-only sites guarded/rewritten, `RungB13` guard test. Gate unchanged (`909605E965BFFE59`; 253/1/0). |
| `958b5ad` | **Phase 2** browser-live demo (`src/Sim.LiveHost/`) |
| `e818cc6` | **Phase 0** NuGet packaging (`SumoSharp.Core` + `SumoSharp.Ingest`) |
| `e3756be` | Removed the transitional **string obstacle API**; handle-only + all callers migrated |
| `d1778dc` | Async **`SimulationRunner`** (command dispatcher + published snapshot) |
| `7699495` | Per-Step **lifecycle event buffer** (`Engine.Events`, Departed/Arrived) |
| `c439002` | **Geometry-3D** `z` ingestion + `PosZ` read column |
| `f08407b` | **Runtime demand**: `LoadNetwork`, `DefineVType`, `SpawnVehicle`, reroute, despawn |
| `f4ca712` | Stepped **SoA read surface** + public `Step()` |
| `c5aedaf` | Handle-based **struct-of-arrays obstacle store** |
| `2ec4062`–`def8d01` | Design doc + laneless-branch coordination |

## Public API surface (all on `Sim.Core.Engine` unless noted)

- **Obstacles (handle-only):** `int GetLane(string)`, `ObstacleHandle AddObstacle(int laneHandle, …)`,
  `AddMovingObstacle(...)`, `UpdateObstacle(ObstacleHandle, …)`, `RemoveObstacle(ObstacleHandle)`,
  `ClearObstacles()`. Store: `ObstacleStore.cs` (direct-mapped SoA + generational handle + dense active
  list + reserved `AvoidanceClass` byte). `ExternalObstacle` is now a `readonly record struct`.
- **Stepped read surface:** `Step()` / `Step(int)`, `VehicleCount`, `StepCount`, `CurrentTime`;
  columnar spans `VehicleHandles`, `PosX/PosY/PosZ`, `Angle`, `Speed` (float), `LaneHandles` (int),
  `Pos`/`PosLat` (double), `VehicleIds/VehicleTypes/LaneIds`; `TryGetVehicle(VehicleHandle, out VehicleState)`.
  Backing: `VehicleReadBuffer.cs`, populated only by `Step()` (Run() pays nothing).
- **Runtime demand:** `LoadNetwork(net[, cfg])`, `DefineVType(VTypeParams) → VTypeHandle` (+ `DefaultVType`,
  `TryGetVType`), `SpawnVehicle(...)` (edge-list and from→to), `GetLifecycle`, `Despawn`,
  `SetDestination`, `Reroute`. Backed by mutable `_vTypesById`/`_routesById` seeded from `_demand`.
  SUMO-parity **queued insertion** (spawned vehicle is `Pending` → `Active`).
- **Lifecycle events:** `ReadOnlySpan<SimEvent> Events` (`SimEvent.cs`), diffed each `Step()`.
- **Async runner:** `SimulationRunner` (`SimulationRunner.cs`) + immutable `SimulationSnapshot`
  (`SimulationSnapshot.cs`): `Post`/`Invoke`/`Tick`/`Start`/`Stop`/`Pause`/`Resume`/`SpeedMultiplier`.
- **Geometry-3D:** `Lane.ShapeZ` + `LaneGeometry.ElevationAtOffset`; `PosZ` is 0 on 2-D nets.

New handle/value types: `ObstacleHandle`, `VehicleHandle` (both 32-bit index + 16-bit generation, the
host game engine's convention), `VTypeHandle`, `AvoidanceClass`, `VehicleLifecycle`, `VTypeParams`,
`VehicleState`, `SimEvent`.

## Tests (new this session)

`RungB7`..`RungB12` cover the new surface; `RungB1/B3/B5/B6` were migrated to the handle obstacle API.
- B7 obstacle handle store (generational contract), B8 stepped read surface (Step == Run bit-for-bit),
  B9 runtime spawn/reroute/vType/LoadNetwork, B10 geometry-3D, B11 lifecycle events, B12 async runner
  (incl. a threaded smoke test).

## Remaining work (prioritized, none blocking)

1. ~~**`netstandard2.1` multi-target** on `Sim.Core` + `Sim.Ingest`~~ — **DONE** (see below / API §3).
   ~~Unity/Godot sample~~ → landed as `samples/SumoSharp.GameHostSample` (a ns2.1-consumable `GameHost`
   integration class + runnable net8 headless demo; `RungB17`). Per steer, this replaces an in-editor
   Unity/Godot project (neither engine can run in this environment).
2. **Publish CI** to nuget.org (a GitHub Actions workflow: pack + push `.nupkg`/`.snupkg`, gated on a
   tag). Pin `RepositoryUrl`/commit for SourceLink (see gotcha below).
3. ~~**Async runner refinements (§7):** two-frame **interpolation hook** + **snapshot pool**~~ — **DONE**
   (commits `ce37400`, `3ac73c1`; see API §7). Both additive/async-only; pool is opt-in.
4. ~~**`GetEdge(string) → int`** dense edge handles (§9)~~ — **DONE** (commit `0ceeaf0`; API §9).
5. **Vehicle-slot recycling** on `Despawn` (§9) — slots are not reused yet (EntityIndex only grows).
   **Deliberately deferred this session (not a blocker).** In-place slot reuse is coupled to a lot of
   `EntityIndex`-keyed state that must all be reset atomically without perturbing the *shared* append path
   the goldens use: `CreateRuntime` seeds `RngState`/`SpeedFactor`/`Entity` off `EntityIndex`; the lifecycle
   event-diff (`_prevLifecycle[idx]`) and the `_stopsByEntity`/`_avoidedByEntity` side tables are all
   index-keyed. Correct recycling is a focused, well-tested change to the vehicle-creation + lifecycle-diff
   code — worth doing deliberately, not bundled with lower-risk work. Until then `_vehicles` grows on each
   `SpawnVehicle` (fine for bounded/most runs; a concern only for very-long-lived spawn/despawn-heavy hosts).
6. **Lifecycle events**: `InsertionFailed`/`Teleported` are defined but not emitted (no insertion
   timeout; teleport off). Wire if/when those engine behaviors exist.
7. **`VTypeParams` sublane fields** (`maxSpeedLat`/`latAlignment`/`minGapLat`) — added at the laneless
   merge, which owns those `VType` additions.

## Coordination with the laneless/RVO branch (`claude/sumo-phase-2-planning-p3w7kh`)

See `SUMOSHARP-API.md` §15 for the full record. Key points for the merge:
- **`RvoNeighbor` is the sole seam.** This branch **owns** the obstacle store; the RVO layer consumes it
  via that neutral value abstraction. The store already carries the columns the Stage-3 adapter needs
  plus the reserved `AvoidanceClass` byte.
- **Merge order:** this branch's SoA store lands first, then their `ComputeRvoLateral` adapter retargets
  onto it. RVO Stages 1–2 are order-independent.
- **Heads-up:** this branch **removed** the string obstacle API — any laneless-branch test/demo that
  still calls `AddObstacle(string id, …)` must migrate to `GetLane` + handles at merge (mechanical).
- **Shared acceptance gate:** determinism anchor `909605E965BFFE59`, byte-identical goldens.
- Both branches additively edit the same 8 files (`Engine.cs`, `VehicleExportSnapshot.cs`,
  `DemandModel.cs`, `DemandParser.cs`, `VTypeDefaults.cs`, `ScenarioConfig.cs`, `TrajectoryPoint.cs`,
  `ToleranceConfig.cs`); whoever merges second reconciles the additive hunks.

## Gotchas / non-obvious decisions

- **`Step()` vs `Run()`:** `Step()` (host loop) publishes the read buffer + events; `Run()` (batch,
  returns `TrajectorySet`) does not — that is what keeps the parity/determinism path zero-overhead. They
  advance the sim identically (B8 proves bit-exact).
- **Snapshot precision:** render columns are `float` (`PosX/Y/Z`, `Angle`, `Speed`); parity-exact values
  are `double` (`Pos`, `PosLat`, and `SpeedExact` on the snapshot / `VehicleState.Speed`).
- **Handles are a distinct id space** from the host ECS; 16-bit generation wraps at 65 536 slot recycles
  (accepted, matches the host engine).
- **`Despawn`** surfaces as an `Arrived` event with a stale-generation handle (the host initiated it).
- **Obstacle `Id`** is now always empty (was only the string-API key); `ComputeLateralEvasion`'s
  tie-break among overlapping obstacles falls to insertion order — deterministic, and no committed
  scenario has such an overlap.
- **SourceLink warning** ("source control information is not available") appears only because this
  environment's git remote is a local proxy URL. On real GitHub it resolves; pin `RepositoryUrl` +
  commit in CI to silence it.
- **`LoadNetwork` default config:** Begin 0, End 1e9, 1 s Euler steps, teleport off, sigma-neutral,
  seed 42 (matches `Engine.Seed`).
- **`Sim.LiveHost`** parses the net itself (via `NetworkParser`) for drawing + the screen→lane
  projection; lane handles match the engine's because both parse the same file in the same order.

## File map (new/changed this session)

Core: `ObstacleHandle/ObstacleStore/AvoidanceClass/ExternalObstacle`, `VehicleHandle/VehicleState/
VehicleReadBuffer`, `VTypeHandle/VTypeParams/VehicleLifecycle`, `SimEvent`, `SimulationRunner/
SimulationSnapshot`, plus additions in `Engine.cs`/`IEngine.cs`.
Ingest: `NetworkModel.cs` (Lane.ShapeZ), `NetworkParser.cs` (ParseShapeZ), `LaneGeometry.cs`
(ElevationAtOffset).
Packaging: `Directory.Build.props`, `Sim.Core.csproj`, `Sim.Ingest.csproj`, package READMEs.
Demo: `src/Sim.LiveHost/` (Program/SimHost/HtmlPage + README).
Tests: `RungB7`..`RungB12`; `RungB1/B3/B5/B6` migrated.
Docs: `SUMOSHARP-API.md` (status per section), this file.
