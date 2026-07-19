# PEDESTRIAN-NAVMESH-CONTRACT.md — the `IWalkableSpace` / `IPedNavigation` / `ILocalSteering` contract

**Status:** normative, P2-3 (`docs/PEDESTRIAN-TASKS.md`). This is a **contract document**, not a
tutorial: it states exactly what the owner's production navmesh must satisfy to plug into
`Sim.Pedestrians` as a drop-in replacement for the two shipped dev providers, with no change to any
code above the seam (`docs/PEDESTRIAN-DESIGN.md` §4, §10, Principle "no double-build"). The three
interfaces themselves live in `src/Sim.Pedestrians/Navigation/INavigation.cs`; read that file's own
doc comments alongside this one — this document explains the semantics and invariants those
signatures only imply.

## 0. Why three interfaces, not one

`docs/PEDESTRIAN-DESIGN.md` §4 splits navigation into three seams because three different providers
can reasonably supply them independently:

| Interface | Question it answers | Called from |
|---|---|---|
| `IWalkableSpace` | "Is this point walkable? Where's the nearest walkable point? What are the confining walls?" | Spawn/goal snapping; `OrcaCrowd.AddObstacle` (confinement) |
| `IPedNavigation` | "Given an origin and a goal, what polyline gets me there?" | `PedLodManager` on promotion/demotion (`Lod/PedLodManager.cs`); `PedDemand` on spawn (`Demand/PedDemand.cs`) |
| `ILocalSteering` | "Given my current pose and my path, what's my preferred velocity right now?" | `PedRouteController.Update()` (high-power tactical steering); low-power `PathArcMotion` is a *different*, deterministic-by-construction implementation of this same role (see §5) |

A production navmesh is free to implement all three with one internal engine, or to implement only
`IWalkableSpace`/`IPedNavigation` and reuse the shipped `WaypointFollower` for `ILocalSteering` — the
seam does not require symmetry. The two shipped providers (§7) deliberately differ in exactly this
way: `SumoWalkableSpace`/`SumoNavMesh` supply their own `WaypointFollower`; `DotRecastWalkableSpace`
does the same. Neither ships a bespoke `ILocalSteering` — there is currently exactly one production
`ILocalSteering` implementation (`Navigation/Bake/WaypointFollower.cs`), and a new navmesh provider is
expected to reuse it rather than reimplement funnel-following steering, unless it specifically needs
different tactical behavior.

## 1. Coordinate convention (binding on every method)

All geometry crossing this seam is `Sim.Core.Orca.Vec2` — a value-type pair of `double X, Y`
(`src/Sim.Core/Orca/Vec2.cs`). This is **planar, 2D, y-up-in-the-plane**: `(X, Y)` is a top-down
ground-plane coordinate, matching the SUMO net's own XY plane and the `OrcaCrowd` operational layer
these interfaces feed (`docs/PEDESTRIAN-DESIGN.md` §3). There is no Z axis anywhere in this seam —
elevation, multi-level structures, and 2.5D navmeshes are entirely a provider-internal concern hidden
behind `IWalkableSpace`; a multi-level production navmesh must collapse to one `(X, Y)` per query
(e.g. by having the caller supply the correct level's `IWalkableSpace` instance, or by baking a single
"current floor" projection) — **never** by smuggling a third coordinate into `Vec2`.

Distances, radii, and speeds are plain doubles in the same linear units as the net (SUMO's meters).
`Vec2.Abs` is Euclidean magnitude; there is no unit conversion at this seam.

Winding convention where it matters: `WallSegment(A, B)` is a **directed** segment with the walkable
interior on its **left** (RVO2's convention, matching `OrcaCrowd.AddObstacle`'s own vertex-loop
convention — see `OrcaCrowd.cs`). A provider emitting `BoundarySegments` must respect this winding or
`OrcaCrowd`'s obstacle confinement will be inside-out.

## 2. `IWalkableSpace` — method semantics

```csharp
bool Contains(Vec2 p);
Vec2 ClampToWalkable(Vec2 p);
IReadOnlyList<WallSegment> BoundarySegments { get; }
```

- **`Contains(p)`** — true iff `p` lies inside walkable space. Must be a pure function of `(provider
  state, p)`: same navmesh, same point, same answer, every call, forever (until the provider's
  geometry is rebuilt/reloaded, which is not a per-query event — see §4 determinism). A provider that
  erodes its walkable area inward by agent radius (as Recast does) may treat "within erosion tolerance
  of the eroded boundary" as contained — `DotRecastWalkableSpace` does exactly this, documented at its
  `_containsTolerance` field — but the tolerance itself must be a fixed build-time constant, not a
  runtime knob that varies the answer call-to-call.
- **`ClampToWalkable(p)`** — the nearest walkable point to `p`; **identity when `p` is already
  inside** (this is binding: a caller must be able to call `ClampToWalkable` unconditionally on every
  spawn/goal point without first checking `Contains`, and get `p` back unchanged when it was already
  fine). Used to snap spawn points and O/D demand points onto the mesh so `FindPath` never has to
  reject a caller-supplied point that is merely a few centimeters off the mesh due to net-import
  rounding.
- **`BoundarySegments`** — **may be empty**. This is not a degenerate case to special-handle away; it
  is one of the two valid answers a provider gives, and every caller in this codebase already
  tolerates it (`DotRecastWalkableSpace.BoundarySegments` is `Array.Empty<WallSegment>()` by design —
  see its own remarks). A provider returns a non-empty set only if it wants `OrcaCrowd.AddObstacle`
  confinement built from *its own* boundary; a provider confining agents some other way (e.g. it has
  its own hard collision walls, or nothing needs confining) returns empty. **Do not** approximate a
  correct union boundary if computing one is expensive — see `SumoWalkableSpace`'s own documented
  caveat (§7) for what "acceptable, documented approximation" looks like here: it emits each baked
  polygon's own edges independently rather than the true union outline, which double-emits shared
  interior portals as walls; that specific approximation is *wrong* to consume for confinement (a
  portal edge is not a wall) but is fine as-is because the one caller that would need the true union
  boundary (`PedRouteController`) does not consume `BoundarySegments` at all in this codebase today. A
  new provider is free to either match that same documented approximation or do better; it must not do
  worse (i.e., must not silently wall off a real portal).

## 3. `IPedNavigation` — method semantics

```csharp
IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal);
```

- Returns an **ordered polyline** from (near) `start` to (near) `goal`, already **funnel/string-pulled
  to a smooth corridor** — i.e. a walkable-space shortest path, not a raw sequence of navmesh-cell
  centroids or polygon-portal midpoints. `SumoNavMesh.FindPath` and `DotRecastNavMesh.FindPath` both
  do this (Detour's own corridor + `DtPathCorridor`/string-pull for DotRecast; `PolygonGraph` A* +
  portal funnel for the SUMO bake) — a provider that returns an un-smoothed portal sequence instead
  breaks every downstream consumer of "this list of points is where I steer toward," most visibly
  `WaypointFollower`/`PathArcMotion`, which walk the polyline's arc length directly with no further
  smoothing of their own.
- Returns **`null`** when `goal` is unreachable from `start` (disconnected components, or either point
  fails to resolve onto the mesh at all) — never an empty list, never a list of length 1 unless
  `start` and `goal` coincide (a length-1 path is a legitimate degenerate answer for that case; both
  `PedLodManager` and `PedDemand` handle it via `PathArcMotion`'s own length-0/1 path handling). Every
  caller in this codebase treats `null` as "this trip cannot happen right now" and reacts accordingly
  (`PedLodManager.Step`'s promotion/demotion paths fall back to a straight two-point path rather than
  crash; `PedDemand.TrySpawnOne` counts it and skips the spawn — see `UnreachableSkipCount`). Throwing
  an exception instead of returning `null` is a contract violation.
- **First point is (near) `start`, last is (near) `goal`** — "near," not necessarily exact, because a
  provider is expected to clamp both endpoints onto the mesh first (effectively `ClampToWalkable`
  followed by a graph search from there); a caller must not assume `path[0] == start` bit-for-bit.
- **Determinism (binding, see §4 below): same `(start, goal)` → same returned polyline, vertex for
  vertex, run after run.**

### 3a. The dynamic-blocker overload (P2-2)

The production SUMO-bake provider additionally exposes:

```csharp
IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal, IReadOnlySet<int>? blockedPolygonIndices);
```

(`SumoNavMesh.cs`; the no-blocker `FindPath(start, goal)` is defined in terms of this overload with
`blockedPolygonIndices: null`.) This is **not** part of the `IPedNavigation` interface itself — it is
an additive, provider-specific extension that `BlockerRegistry`/`RerouteDriver`
(`src/Sim.Pedestrians/Navigation/BlockerRegistry.cs`, `RerouteDriver.cs`) consume when they know they
are talking to a `SumoNavMesh` concretely, to route *around* a set of dynamic obstacles (parked cars,
temporary blockages) by excluding the polygons those obstacles occlude from the path search. A
production navmesh that wants to participate in P2-2-style dynamic rerouting has two options, in order
of preference:
1. Expose an equivalent overload/parameter on its own concrete type and adapt `RerouteDriver`'s
   blocked-set consumer to call it (the registry already produces a provider-agnostic "which regions
   are blocked" signal — `BlockerRegistry.BlockedPolygons()` — the adaptation is in how that signal is
   threaded into the provider's own search, not in the registry).
2. Simulate the same effect at the `IWalkableSpace` boundary — e.g. temporarily marking the blocked
   region non-walkable and forcing a `FindPath` re-query — accepting the cost of whatever
   recomputation that requires internally.
Either way, the **caller-visible contract stays exactly `IPedNavigation.FindPath(start, goal)`** — the
reroute mechanism (registry, debounce/hysteresis in `RerouteDriver`) lives above the seam and does not
require the interface itself to grow a blocker parameter; adding one is a convenience the bake
provider took, not a requirement.

## 4. Determinism requirement (binding, both interfaces)

**Same inputs → same output, forever, independent of thread, call order, or wall-clock time.**
Concretely:
- `Contains`/`ClampToWalkable`/`FindPath` must not consult `System.Random`, `DateTime.Now`, thread-
  static mutable scratch state that leaks between calls, or any other non-reproducible source. If a
  provider's internal search has ties (equal-cost paths, equal-distance nearest points), it must break
  them with a **fixed, documented rule** (index order, insertion order, lexicographic — anything fixed
  and A/A' should be the *same* rule every build), not "whatever the underlying library's iteration
  order happens to be" if that order is undocumented/unstable. `PolygonGraph`'s A* and Detour's own
  `DtNavMeshQuery.FindPath` both satisfy this today.
- Rebuilding the navmesh from the same input geometry must produce a provider that answers identically
  to the previous instance. This is what lets a determinism test construct the provider twice (or
  reuse one instance across two independent simulation runs) and require bit-identical results — see
  `PedDemandTests.SustainedRun_IsDeterministic_AcrossIndependentRuns` and
  `PedLodManagerTests.FullScenario_IsDeterministic_AcrossIndependentRuns` for the shape of that gate.
- This is the **same bar** `CLAUDE.md` sets for the parity lane core and for `PedDemand`'s own random
  draws (SplitMix64, never `System.Random`) — a provider is not exempt just because it lives behind an
  interface seam. A production navmesh built on a non-deterministic underlying library (parallel
  internal search with non-fixed reduction order, floating-point-order-sensitive tie-breaks, etc.) must
  either configure that library into single-threaded/fixed-order mode for path queries or wrap it with
  its own deterministic tie-break layer before it can plug in here.

## 5. Thread-safety expectations

Every method above is queried from the simulation's **plan phase** — the same phase that reads
positions and computes desired velocities, before any command-buffer mutation is applied
(`docs/PEDESTRIAN-DESIGN.md`'s plan/execute split, mirroring the parity-lane core's own convention).
Concretely, in this codebase:
- `PedLodManager.Step` calls `_navigation.FindPath` for every promotion and every demotion **within
  the same single-threaded step**, ascending-ped-id order (see its own remarks on why: "deterministic
  evaluation and application").
- `PedDemand.Step` calls `_navigation.FindPath` once per spawn attempt, also single-threaded.

**Today, nothing in this codebase calls into `IWalkableSpace`/`IPedNavigation` from more than one
thread concurrently, and no shipped provider is required to be thread-safe for concurrent queries.**
A provider MAY be read-only-safe for concurrent queries (both shipped providers happen to be, since
neither mutates any state in `Contains`/`ClampToWalkable`/`FindPath` — they only read an immutable
baked mesh), and a production provider is encouraged to be, since `docs/PEDESTRIAN-TASKS.md` P1-1's
promotion/demotion path and a future parallel-`FindPath` optimization are both plausible future
callers. But **do not assume the interface requires it** — the current binding requirement is
re-entrant-safe-when-called-serially (no hidden per-call scratch state that a later call on the same
thread corrupts), not concurrent-safe. If a production navmesh's query path is *not* safe for
concurrent calls (e.g. it has an internal mutable scratch buffer per query context), it must document
that restriction; callers in this codebase already satisfy it by construction, but a future caller
must know not to violate it.

## 6. Composing with the LOD / PathArc layer

This is the seam's most important non-obvious property, so it gets its own section
(`docs/PEDESTRIAN-DESIGN.md` §4 "Low-power motion", §5, §7):

A **low-power** pedestrian's entire motion is `PathArcMotion.PositionAt(path, startTime, speed, now)`
— a pure function of the polyline `IPedNavigation.FindPath` returned once, at spawn or at the last
promotion/demotion switch. No neighbor query, no re-steering, no per-step call back into navigation.
This is precisely what §4 means by "deterministic path-follower with a speed profile": **the path
itself is the only navmesh-derived state a low-power ped carries forward**. Two consequences a
navmesh provider must respect:

1. **The returned polyline must be stable across the ped's low-power lifetime once handed out.**
   `IPedNavigation.FindPath` is called once per PathArc "leg" (spawn, or fresh re-route on demotion —
   `PedLodManager.Step`'s remarks explain why demotion re-queries rather than resuming the old
   polyline: an ORCA-drifted position is off the old path, so resuming it is wrong). The provider does
   not need to support incremental path updates or notify anyone if its underlying mesh changes later
   — the ped is already committed to its polyline until its *next* promotion/demotion event, which is
   when the caller will re-query. A provider is free to hot-reload its mesh between such queries; it
   must not require the caller to re-validate an already-issued polyline against a changed mesh.
2. **The path must be exactly reproducible for the IG (`docs/PEDESTRIAN-DESIGN.md` §7).** The whole
   "server == IG" invariant for low-power peds (`HeadlessIg`/`PathArcRecord`, proven by
   `PedLodManagerTests.LowPower_ServerPositionMatchesHeadlessIgReconstruction_AtEverySampledTime`) rests
   on the IG reconstructing a ped from the **one-time broadcast polyline**, never re-deriving it. This
   works regardless of which navmesh produced the polyline — the IG never calls `IPedNavigation`
   itself — but it means the *server's* call to `FindPath` must be the determinism (§4) this contract
   already requires: if the server ever called `FindPath(O, D)` twice for the same trip (it doesn't,
   today) and got two different answers, that would silently desync a re-simulated or restarted server
   from an IG that cached the first answer. Determinism here is not just a nice-to-have for tests — it
   is the reason the wire protocol can send the path *once*.

A **high-power** ped instead re-derives a fresh `ILocalSteering.DesiredVelocity` every tick
(`PedRouteController.Update`), but still only calls `IPedNavigation.FindPath` at the same LOD-
transition boundaries (promotion/demotion), not every step — the tactical layer walks the *already-
returned* polyline; it does not re-query the strategic layer per tick. A navmesh provider therefore
sees `FindPath` called at a rate bounded by "trips + LOD transitions," never by "peds × steps/sec" —
this is precisely why `IPedNavigation.FindPath`'s own cost does not need to be sub-millisecond to keep
up with a 100k-ped step budget; it needs to keep up with the much smaller promotion/demotion + spawn
rate (`PedDemand`'s own spawn rate, plus `InterestField`-driven transitions).

## 7. The two shipped reference implementations (read these, don't reinvent)

| Provider | `IWalkableSpace` | `IPedNavigation` | Files |
|---|---|---|---|
| **SUMO-geometry bake** | `SumoWalkableSpace` | `SumoNavMesh` | `src/Sim.Pedestrians/Navigation/Bake/*` |
| **DotRecast** | `DotRecastWalkableSpace` | `DotRecastNavMesh` | `src/Sim.Pedestrians.Nav.DotRecast/*` |

Both exist specifically so this contract is proven against **two independent implementations**, not
inferred from one and hoped to generalize (`docs/PEDESTRIAN-DESIGN.md` §4: "two providers also *prove*
the seam is real, not a shim around one implementation"). A third, production implementation is
expected to look like a peer of these two, not a special case:

- **SUMO-geometry bake** (`WalkablePolygonBaker.Bake` → `BakedPolygon[]` → `PolygonGraph` A* +
  portal-funnel `FindPath`): builds walkable polygons directly from the ingested SUMO pedestrian
  network (sidewalk lanes, crossings, walkingAreas). Read `SumoNavMesh.cs` for the funnel-string-pull
  implementation and `PolygonGraph.cs` for the portal graph — this is the concrete example of "shortest
  path over a polygon-portal graph, funnel-smoothed" the contract in §3 describes abstractly.
- **DotRecast** (`DotRecastNavMeshBuilder.Build` → `DtNavMesh` → Detour's own `DtNavMeshQuery.FindPath`
  + corridor string-pull): a true open-space navmesh (handles plazas/malls/parking lots that are not
  representable as sidewalk-quad polygons at all). Read `DotRecastNavMesh.cs` for how a fully external,
  third-party pathfinding library (MIT-licensed Recast/Detour C# port) is wrapped to satisfy exactly
  the same `IPedNavigation`/`IWalkableSpace` seam — this is the closer structural analog to how an
  owner's production navmesh (also presumably an external engine wrapped at this boundary) should
  integrate: a thin adapter type per interface, translating `Vec2` in and out at the boundary and nowhere
  else, with no `Sim.Pedestrians` internals reaching into the wrapped library's own types.

### Adapter template (what a third provider's file layout should look like)

Following the DotRecast provider's shape (the closer analog for a wrapped external engine):

```
YourNavmesh.Adapter/
  YourNavmeshWalkableSpace.cs   : IWalkableSpace   — wraps your engine's point-containment/nearest-point queries
  YourNavmeshNavigation.cs      : IPedNavigation   — wraps your engine's path query; funnel/string-pull
                                    the raw corridor into the smooth polyline §3 requires if your engine
                                    doesn't already do this internally
  YourNavmeshBuildConfig.cs     : whatever build-time parameters your engine needs (agent radius, cell
                                    size, etc.) — a plain config record, mirrors DotRecastBuildConfig.cs
  YourNavmesh.Adapter.csproj    : its own project, exactly like Sim.Pedestrians.Nav.DotRecast
                                    (docs/PEDESTRIAN-DESIGN.md §10) — kept out of the CORE
                                    Sim.Pedestrians/Sim.Pedestrians.Tests dependency graph either way, so
                                    the hermetic ped test project never references your adapter. Whether
                                    the adapter's OWN test project sits inside Traffic.sln depends on
                                    whether it brings a genuinely native/non-hermetic dependency: DotRecast
                                    is a pure-managed MIT C# port, so its test project (Sim.Pedestrians.
                                    Nav.DotRecast.Tests) DOES sit inside Traffic.sln today; a provider
                                    wrapping a native library should instead follow Sim.Replication.Dds's
                                    convention (excluded from Traffic.sln entirely, its own manually-run
                                    loop) — see docs/PEDESTRIAN-DESIGN.md §11 open item 7.
```

No change to `Sim.Pedestrians/Lod/`, `Sim.Pedestrians/Demand/`, or any consumer is required — every
consumer (`PedLodManager`, `PedRouteController`, `PedDemand`) already programs against the three
interfaces, constructed with whichever provider the caller wires up (see how
`PedLodManagerTests`/`PedDemandTests` construct a `SumoNavMesh` and hand it in as `IPedNavigation`, and
how a DotRecast-backed test would hand in a `DotRecastNavMesh` instead — same call sites, different
concrete type). **This is the success condition for P2-3**: swap the provider, change nothing above
this file.
