# Design — arbitrary SUMO road-net import with route-graph pedestrians

**Status:** DESIGN (for owner agreement). Companion to `LIVE-CITY-ARBITRARY-NET-SCOPING.md` (the WHAT/scope)
and `LIVE-CITY-ARBITRARY-NET-TASKS.md` (the staged work). This document is the HOW: mechanisms, data
structures, algorithms, the exact seams touched, the data flow, and the determinism/parity argument. It does
not restate scope; see the scoping memo.

---

## 0. One-paragraph summary

Add a second `IPedNavigation` implementation, **`SumoRouteGraphNav`**, that routes pedestrians on the SUMO
pedestrian **edge/connection graph** already present in `net.xml` (sidewalk lanes + crossings + walkingareas,
stitched by ped `<connection>`s) and returns a plain `Vec2` polyline — the exact contract the existing
downstream (`PedDemand`, `PedLodManager`, `PathArcMotion`, `WaypointFollower`, ORCA) already consumes. Add a
**road-net-import mode** to `LiveCityConfig`/`LiveCitySim` that selects this provider, **skips the expensive
walkable-polygon bake**, works on the **whole net (no crop)**, detects pedestrian capability and degrades
gracefully, and derives drivable edges from `net.xml`. The demo stays on `SumoNavMesh` (regime unchanged),
so parity, bench, and the demo regression are byte-identical.

---

## 1. Seam and why nothing below it changes

The load-bearing fact (verified): `IPedNavigation` (`src/Sim.Pedestrians/Navigation/INavigation.cs:37-56`)
is a two-member interface —

```csharp
IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal);
IReadOnlyList<double> HalfWidthsAlong(IReadOnlyList<Vec2> path);   // default impl returns 0.5 everywhere
```

`PedDemand` (`PedDemand.cs:55,67,224,249`) and `PedLodManager` (`PedLodManager.cs:82,140-141,406,430,449`)
hold the **interface**. `PathArcMotion`, `WaypointFollower`, and `PedRouteController` consume only an opaque
`Vec2` point-sequence; `Sim.Core`/ORCA never sees navigation at all. A second production implementation
(`DotRecastNavMesh : IPedNavigation`) already proves the seam is real.

**Consequence:** `SumoRouteGraphNav` returning an equivalent polyline drops in with **zero changes** to
`PedDemand`, `PedLodManager`, `PathArcMotion`, `WaypointFollower`, `PedRouteController`, or `Sim.Core`
(iron law R5 — no `Sim.Core` motion-math edits — is structurally satisfied). The only code that changes is
`LiveCitySim`'s construction wiring (which navigation to build) plus the new class and config.

---

## 2. Data source — reuse `PedNetworkParser`, skip the baker

`PedNetworkParser.Load(netPath)` (`src/Sim.Pedestrians/PedNetworkParser.cs`) is **cheap** — it parses XML
into lane records and connections and produces `PedNetwork` (`Sidewalks`, `Crossings`, `WalkingAreas`,
`WalkablePolygons`, `AccessPoints`, `PedConnections`). The **expensive** step we are avoiding is
`WalkablePolygonBaker.Bake` (per-sidewalk `PolylineBuffer.Buffer` mitred strips) + `SumoNavMesh`'s polygon
graph + spatial index.

`SumoRouteGraphNav` consumes the **same `PedNetwork`** that `PedNetworkParser.Load` already returns. No new
parser; the route graph is built from records already in memory. This also gives R3 its probe for free
(inspect `PedNetwork.Sidewalks.Count` / `.Crossings.Count`).

Lane id space is `edge + "_" + laneIndex` and is identical between `PedNetwork` records, `BakedPolygon.Id`,
and `PedConnection(aId,bId)` — so connections index cleanly into lanes.

---

## 3. `SumoRouteGraphNav` — data structures

Namespace: `Sim.Pedestrians.Navigation.RouteGraph` (new). New files under
`src/Sim.Pedestrians/Navigation/RouteGraph/`.

### 3.1 Node model

One **node per pedestrian lane element**:

| Node kind | Source | Geometry role |
|---|---|---|
| `Sidewalk` | `PedNetwork.Sidewalks` (`PedLane`) | traversable **centreline** (lane `shape`), half-width = lane width/2 |
| `Crossing` | `PedNetwork.Crossings` (`PedCrossing`) | traversable **centreline** (lane `shape`), half-width = lane width/2 |
| `WalkingArea` | `PedNetwork.WalkingAreas` (`PedWalkingArea`) | **polygon** cut across between portals, nominal half-width |

Each node stores: stable `int Index` (assigned in a deterministic order — kind group, then id ordinal), `Id`
(lane id), `Kind`, the polyline/polygon vertices, `HalfWidth`, and precomputed `Centroid` + segment AABBs.

### 3.2 Edges (portals)

Graph edges come from `PedNetwork.PedConnections` — each `PedConnection(aId, bId)` links two lane nodes and
is traversable **both directions** (walking is symmetric). For each connection we precompute a **portal
point**: the geometric junction of the two lanes, taken as the midpoint of the closest endpoint pair between
`shape(a)` and `shape(b)` (deterministic). The portal is the anchor where the assembled route hands off from
one node's geometry to the next's.

Adjacency is stored as `int[][]` (node → neighbour node indices) plus a parallel `PortalPoint[][]`, built
once at construction. A node with no connections is an isolated component (still routable start==goal case).

### 3.3 Spatial index (nearest lane)

To map an arbitrary O/D `Vec2` to the graph we need nearest-lane lookup. A **uniform grid** over lane
**segment** AABBs (cell size ≈ median lane length, tunable) — for a query point, scan candidate segments in
the query cell and its 8 neighbours, keep the nearest point-on-segment; widen the ring if empty. This is the
*only* index we build, and it indexes 1-D centrelines (far lighter than the navmesh's polygon index).

Determinism: grid construction and query iteration order are by node/segment index; ties broken by lower
index. No RNG, no floating-point-order hazards beyond standard `<` comparisons.

---

## 4. `SumoRouteGraphNav.FindPath` — algorithm

```
FindPath(start, goal):
  sLane = NearestLane(start)   ; gLane = NearestLane(goal)
  if sLane or gLane is null  -> return null            // net has no ped lanes near the point
  if sLane == gLane          -> return [start, snap(start,sLane) .. snap(goal,sLane), goal]  // same-lane sub-path
  nodePath = AStar(sLane.node, gLane.node)              // over §3.2 adjacency, cost = Euclidean via portals
  if nodePath is null        -> return null             // disconnected ped graph (few/no peds; graceful)
  return AssemblePolyline(start, goal, nodePath)
```

- **A\*** heuristic = Euclidean distance from a node's portal-relevant point to `goal` (admissible). Edge
  cost = distance between successive portal points along the node geometry (see §4.1). Priority queue keyed
  `(fScore, nodeIndex)` — the `nodeIndex` tie-break makes expansion order deterministic.
- **Null contract preserved:** returning `null` on unreachable is exactly what `PedDemand` already handles
  (`PedDemand.cs:225`, `UnreachableSkipCount++`) and `PedLodManager` falls back on
  (`?? new[]{pos,dest}`). A poorly-stitched net therefore yields *few/no peds*, never a crash.

### 4.1 `AssemblePolyline` — threading lanes, crossings, and walkingareas

Walk the node path, appending each node's contribution between its **entry portal** and **exit portal**
(the portals shared with the previous/next node; the first node's entry is `start`, the last node's exit is
`goal`):

- **Sidewalk / Crossing node** (centreline): append the sub-polyline of the lane `shape` between the entry
  and exit portal projections, **oriented** so the near end matches the entry portal (a lane may be traversed
  in either direction). This is a projection onto a polyline + slice — pure `Vec2` math.
- **WalkingArea node** (polygon): append a **straight segment** from entry portal to exit portal (a
  pedestrian cutting across the junction corner). First approximation; if a straight chord exits the polygon
  badly on unusual shapes, fall back to routing via the polygon's nearest boundary vertices. (SUMO itself
  computes walkingarea internal routes; the straight chord matches the common convex-corner case and is
  refined only if a fixture shows it leaving the area.)

The resulting `IReadOnlyList<Vec2>` is the FindPath return — the same shape `SumoNavMesh` returns
(`{start, portal, …, goal}`), so all downstream consumers are byte-compatible.

### 4.2 `HalfWidthsAlong`

Because `AssemblePolyline` knows which node produced each vertex, `HalfWidthsAlong(path)` returns the
per-vertex half-width of the owning node (lane width/2 for sidewalks/crossings; nominal for walkingareas).
This is strictly better than the 0.5 m default and makes the weave (`PedDemand.BuildLivelyTimeline`) and O/D
jitter width-correct. Implementation re-locates each vertex to its nearest node segment (same grid) if the
path was not produced by this instance (defensive), else uses the recorded owner.

---

## 5. `LiveCitySim` / `LiveCityConfig` wiring (road-net mode)

### 5.1 Config

Add to `LiveCityConfig`:

- `enum PedNavMode { Navmesh, RouteGraph }` and `PedNavMode NavMode { get; set; }`.
  - `ForRepoRoot` (demo) → `Navmesh` (unchanged behaviour, locked fixture).
  - `ForDataset(datasetDir)` → `RouteGraph` (road-net import default).
- Surface currently ctor-hardcoded ped-demand knobs (§7): `PedMaxSpeed`, `PedRadius`, `PedArrivalRadius`,
  `PedEnableWeave`, plus the `PedLivelinessConfig` fields, all defaulted to the exact current literals so the
  demo config is byte-identical.
- `ForDataset` factory: `DatasetDir = datasetDir`, `NavMode = RouteGraph`, no crop (see §5.4). `ForRepoRoot`
  delegates to a shared builder then overrides `DatasetDir`/`NavMode` for the demo, preserving every existing
  `LIVECITY_*` env override.

### 5.2 Constructor branch

`LiveCitySim` ctor (`LiveCitySim.cs:92-287`) currently: parse net twice → `WalkablePolygonBaker.Bake` →
`SumoNavMesh` → crop endpoints → crossings/signals → engine. The new branch:

```
model = NetworkParser.Parse(netPath)               // unchanged (vehicles + LocalLanes)
pedNetwork = PedNetworkParser.Load(netPath)         // wrapped in try/catch (R3, §6)
switch cfg.NavMode:
  Navmesh (demo):   polygons = WalkablePolygonBaker.Bake(pedNetwork)   // EXISTING path, untouched
                    nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons), pedNetwork.PedConnections)
                    crossingPolys = crop-filtered Crossing polygons     // existing
  RouteGraph:       nav = new SumoRouteGraphNav(pedNetwork)             // NO sidewalk bake
                    crossingPolys = BakeCrossingsOnly(pedNetwork)       // §5.3, cheap
```

`nav` is typed `IPedNavigation` from here on; `PedLodManager`/`PedDemand` construction is unchanged.

### 5.3 Crossing gate + signals in road-net mode

`CrossingOccupancySource` and `CrosswalkSignals.FromNet` take `BakedPolygon`s of `Kind==Crossing`. Crossings
are few (hundreds, vs tens of thousands of sidewalks), so road-net mode bakes **crossings only** via the
existing baker's crossing branch (`BakeCrossingsOnly` = the `Crossing` arm of `WalkablePolygonBaker`,
extracted into a small helper, or a filtered call). This reuses `CrossingOccupancySource` and
`CrosswalkSignals` **unchanged**. Walk-only degrade (no crossings) → skip both, exactly as `YieldEnabled`
already gates them.

### 5.4 No crop

Road-net mode does not crop. The crop predicates (`In`/`InV`, `LiveCitySim.cs:117-118`) and the endpoint
crop-filter are bypassed when `NavMode==RouteGraph`; O/D is sampled from the whole ped network (§5.5), and
`Sample()`/`SampleCars()` crop-filters become no-ops (return all). The realism-zone/pocket centre defaults
to the net's geometric centre (AABB of lane shapes, or `<location convBoundary>` if present — optional
convenience, not a gate). Demo (`Navmesh`) keeps its hero-block crop exactly as today.

### 5.5 O/D sampling (caller-side, road-net mode)

The navigation seam does **not** supply endpoints — the caller does (verified). In road-net mode
`LiveCitySim` samples O/D from **sidewalk lane centrelines**: pick lanes by a deterministic seeded stride
over `PedNetwork.Sidewalks` (cap at an endpoint budget so a huge net samples a bounded set), take a point on
each lane's centreline (optionally jittered laterally within half-width). Assign to
`PedDemandConfig.Origins`/`.Destinations` — identical shape to the demo's current midpoint sampling, just
sourced from lanes instead of baked polygons and without the crop filter. Deterministic (seeded stride, no
`System.Random`).

### 5.6 Drivable edges from `net.xml` (R1 / vehicles)

`ReadDrivableEdges` (`LiveCitySim.cs:729-743`) regex-scrapes `scenario.rou.xml`. Add a fallback: when the
route file is missing/empty, derive spawn edges from `model.EdgesById` where the edge has ≥1 lane with
`AllowsRoadVehicle==true` (the parser already tags this, `NetworkParser.cs:428`), excluding internal `:`
edges. Demo path unchanged (its `scenario.rou.xml` exists, so the scrape wins and the fallback never runs).

### 5.7 `RerouteDriver`

`RerouteDriver` (`src/Sim.Pedestrians/Navigation/RerouteDriver.cs`) holds the **concrete** `SumoNavMesh` and
its navmesh-only blocked-polygon overload. It is **not** on the `PedDemand`/`PedLodManager` path and
`LiveCitySim` does not construct it. The design keeps it that way: road-net mode never touches `RerouteDriver`
(no code change needed; documented invariant + an assertion in the road-net smoke test that the nav object
is a `SumoRouteGraphNav`).

---

### 5.8 Pedestrian avoidance of vehicles (IN SCOPE — real-world requirement)

Peds must know about SUMO cars: walk **around a car stopped on a crosswalk** (a jammed junction — a real
situation) and, for later evacuation work, around abandoned cars on the street. This is a first-class
requirement, not an enhancement.

**Mechanism (navmesh-independent).** Car-avoidance and pathfinding are separate: pedestrians avoid vehicles
via `OrcaCrowd.SetExternalObstacles(discs)` (`src/Sim.Core/Orca/OrcaCrowd.cs:797-803`) — vehicle positions
projected to `WorldDisc`s and avoided **one-sided** (responsibility 1.0: the crowd yields fully, the vehicle
avoids the crowd through its own lane solve; `OrcaCrowd.cs:731-745`). The discs reach the crowd via
`PedDemand.Step(..., externalEntities)` → `PedLodManager.Step(..., externalEntities)`, which feeds them to
`_highCrowd.SetExternalObstacles` then `_highCrowd.Step(dt)` (`PedLodManager.cs:472-480`) — pure runtime
position data, never the navigation seam. So **route-graph mode does ped-avoids-car identically to navmesh
mode.**

**What road-net mode does:** in `LiveCitySim.Step`, project each live vehicle from the engine snapshot
(`_lastSnapshot`: `PosX/PosY`, velocity, a bounding radius from `Length/Width`) to a `WorldDisc[]` and pass
that as `externalEntities` (replacing the current `NoEntities`, `LiveCitySim.cs:451`). One disc per car
centre for v1 (radius ≈ half the larger body dimension); a short chain of discs along the body is a later
refinement if single-disc corner clipping shows. Deterministic (derived from the snapshot, no RNG).
`OrcaCrowd`'s range cutoff means feeding all live cars is fine — only nearby ones influence a ped.

**LOD boundary (by design, expected):** only **high-power (ORCA) peds** avoid cars (and each other); this is
the whole point of the LOD split. **Low-power PathArc peds avoid nothing** — they are cheap dead-reckoning
that walks the polyline, and that is intended, not a gap. The peds a viewer watches near a jammed crosswalk
are the promoted high-power ones in the realism zone, so they visibly go around a stopped car; distant
low-power peds do not, and are not meant to. No low-power avoidance is in scope or planned for this feature.

**Config + demo safety.** A `LiveCityConfig.PedAvoidVehicles` knob gates it: **on** for road-net
`ForDataset`, **off** for the demo `ForRepoRoot` so the dense-flow liveness regression stays byte-identical
(the demo currently passes `NoEntities`; turning the feed on there changes ped positions that loop back into
the crossing gate → `CrowdSource` → car metrics, so the demo default stays off and the pinned liveness test
is untouched). The feed is additive and never runs in parity/bench (they drive `Engine` directly).

## 6. Capability detection + graceful degrade (R3)

After `PedNetworkParser.Load` (wrapped):

```
try   pedNetwork = PedNetworkParser.Load(netPath)
catch (InvalidOperationException | InvalidDataException)  pedNetwork = PedNetwork.Empty   // malformed ped-net
hasSidewalks = pedNetwork.Sidewalks.Count > 0
hasCrossings = pedNetwork.Crossings.Count > 0
PedestriansEnabled = hasSidewalks
CrossingsEnabled   = hasSidewalks && hasCrossings
```

- **Bare vehicle-only net** (no ped lanes) → `PedestriansEnabled=false`: skip `SumoRouteGraphNav`, ped
  demand, LOD manager, crossing gate, signals; `CrowdSource` stays vehicle-only; cars run. No throw.
- **Sidewalks, no crossings** → `PedestriansEnabled=true`, `CrossingsEnabled=false`: peds walk, no crossing
  gate/signals (walk-only). Accepted degrade (decision 3).
- **Full** → coupled cars+peds+crossings.

Expose `LiveCitySim.PedestriansEnabled` and `LiveCitySim.CrossingsEnabled` (read-only) so the consumer can
show the active mode. `PedSource` remains non-null and simply carries no peds when disabled (contract
preserved).

---

## 7. Config surfacing (R-suggestion)

The ped-demand knobs hardcoded in the ctor (`LiveCitySim.cs:155-175` — `MaxSpeed=1.3`, `Radius=0.3`,
`ArrivalRadius=0.6`, `EnableWeave=true`, the `PedLivelinessConfig` block, `CrosswalkSignals` gate) move to
`LiveCityConfig` fields, each defaulted to its current literal. The ctor reads them from `cfg`. Proof of
no-op: the demo config produced by `ForRepoRoot` yields identical values → the demo `PedDemandConfig` is
byte-identical → the dense-flow liveness + scene tests are unaffected.

---

## 8. Offline preparation (C2)

Runtime never invokes `netconvert`. Making a bare net ped-capable is offline:

```
netconvert --sumo-net-file in.net.xml \
           --sidewalks.guess --crossings.guess --walkingareas.all-nonspecific \
           -o out.net.xml
```

Deliver a `scripts/prep-ped-net.sh` wrapper (documented, not part of the offline test loop) and a short
recipe section. The **regression fixture** (§9) is generated the same offline way and committed.

---

## 9. Test strategy & fixtures

- **Committed road-net fixture (SUMO-free at test time, no proprietary data):** a tiny synthetic
  ped-equipped net generated offline via `netgenerate --grid --grid.number=3 --sidewalks.guess
  --crossings.guess -o <fixture>.net.xml` (a 3×3 grid with sidewalks + crossings + walkingareas), committed
  under `scenarios/_ped/roadnet_min/` with a `README` + `provenance.txt`. Small, deterministic, ped-stitched.
- **Unit tests (`tests/Sim.Pedestrians.Tests`):** `SumoRouteGraphNav` — nearest-lane, connectivity,
  FindPath returns an on-lane polyline on a connected fixture, `null` on a disconnected one, determinism
  (repeat-call identical), `HalfWidthsAlong` matches source widths.
- **Road-net smoke/regression (`tests/Sim.LiveCity.Tests`):** load the fixture via
  `LiveCityConfig.ForDataset`, run N steps headless, assert peds spawn + route (LiveCount>0), cross at
  crossings, the nav object is a `SumoRouteGraphNav` (bake-free proof), and a bare-net variant runs
  vehicles-only with `PedestriansEnabled==false` and no throw.
- **Robustness:** a fixture (or unit) exercising large-magnitude, negative, and 3-D (elevation) coordinates
  through parse → route → sample.
- **Parity/bench gate:** `dotnet test tests/Sim.ParityTests` byte-identical (654/4); `Sim.Bench` hash
  intact; demo dense-flow liveness + scene tests green (proves the `Navmesh` path is untouched).

---

## 10. Determinism & parity argument (the iron laws)

- **Parity & bench drive `Engine` directly, never `LiveCitySim`** → the entire feature is invisible to them.
  New code is additive (new class, new mode branch, new config fields with demo-identical defaults).
- **Demo stays on `Navmesh`** (`ForRepoRoot` → `PedNavMode.Navmesh`), executing the *existing* bake/nav path
  unchanged; surfaced config defaults reproduce the current literals byte-for-byte.
- **`Sim.Core` untouched** — the seam is `IPedNavigation`; the new provider lives entirely in
  `Sim.Pedestrians`.
- **No `System.Random`** — nearest-lane, A*, portal computation, and O/D sampling are deterministic
  (index-ordered tie-breaks, seeded stride).
- **netstandard2.1** — `SumoRouteGraphNav` uses only the same primitives the existing nav code does; no new
  dependency, no TFM change.

---

## 11. What is explicitly NOT built (see scoping §4)

Crop; density calibration; navmesh for road-net mode; ORCA walkable-area confinement; runtime ped-infra
generation; `personFlows`/demand-file O/D (deferred); polygon/TAZ crops.
