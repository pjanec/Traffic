# Tasks — arbitrary SUMO road-net import with route-graph pedestrians

**Status:** TASKS (for owner agreement). References `LIVE-CITY-ARBITRARY-NET-DESIGN.md` (the HOW — cited by
section, never copied) and `LIVE-CITY-ARBITRARY-NET-SCOPING.md` (the WHAT). Each task names its design
reference, the files it touches, its dependencies, and **mandatory success conditions** the implementor must
satisfy before the task is closed. Tracker: `LIVE-CITY-ARBITRARY-NET-TRACKER.md`.

**Global gate (applies to every task):** `dotnet test tests/Sim.ParityTests` stays 654/4 byte-identical and
the `Sim.Bench` determinism hash is unchanged. Any task that moves either is reverted.

**Model routing:** B* and E1/E2 (mechanical port + offline gen) suit a Sonnet implementor; A3/C1/C2/C4 and
the F gate need the Opus accept/reject review. Delegate per `CLAUDE.md`.

---

## Stage A — Dataset parameterization, drivable edges, capability probe

### A1 — `LiveCityConfig.ForDataset` factory  *(design §5.1)*
- **Files:** `src/Sim.LiveCity/LiveCityConfig.cs`.
- **Deps:** none.
- **Do:** add `enum PedNavMode { Navmesh, RouteGraph }`, `NavMode` property; add `ForDataset(datasetDir)`
  (→ `RouteGraph`, no crop); refactor `ForRepoRoot` to delegate to a shared builder then set the demo
  `DatasetDir` + `NavMode=Navmesh`, preserving every `LIVECITY_*` override.
- **Success:**
  1. `ForRepoRoot(root)` returns a config **field-for-field identical** to today (unit test asserts every
     property incl. crop `X0..Y1`, seeds, and `NavMode==Navmesh`).
  2. `ForDataset(dir)` returns `DatasetDir==dir`, `NavMode==RouteGraph`, and leaves crop fields at values the
     road-net path ignores.
  3. All existing `LIVECITY_*` env overrides still apply through `ForRepoRoot`.

### A2 — Drivable edges from `net.xml` fallback  *(design §5.6)*
- **Files:** `src/Sim.LiveCity/LiveCitySim.cs` (`ReadDrivableEdges` + caller at `:254-263`).
- **Deps:** A1.
- **Do:** when `scenario.rou.xml` is absent/empty, derive spawn edges from `model.EdgesById` lanes with
  `AllowsRoadVehicle==true`, excluding internal `:` edges.
- **Success:**
  1. On a dataset with **no** `scenario.rou.xml`, `_cropEdges` (road-net mode: net-wide) is non-empty and
     contains only vehicle-allowed, non-internal edges.
  2. Demo (`scenario.rou.xml` present) yields the **exact same** edge set as today (the scrape wins; fallback
     never runs) — asserted by a test comparing the demo edge list before/after.

### A3 — Capability probe + graceful degrade + `PedestriansEnabled`  *(design §6)*
- **Files:** `src/Sim.LiveCity/LiveCitySim.cs`; possibly `PedNetwork.Empty` helper in
  `src/Sim.Pedestrians/PedNetwork.cs`.
- **Deps:** A1.
- **Do:** wrap `PedNetworkParser.Load` in try/catch → `PedNetwork.Empty` on malformed; compute
  `PedestriansEnabled`/`CrossingsEnabled`; gate ped demand/LOD/crossing wiring accordingly; expose the two
  read-only flags.
- **Success:**
  1. A **bare vehicle-only** net constructs with `PedestriansEnabled==false`, no throw, cars spawn and step;
     `PedSource` non-null with zero peds.
  2. A **malformed** ped-net (crossing edge lacking a ped lane) degrades to `PedestriansEnabled==false`
     rather than throwing (unit test with a crafted tiny malformed net string).
  3. A **sidewalks-but-no-crossings** net → `PedestriansEnabled==true`, `CrossingsEnabled==false`, no
     crossing gate/signals wired.

---

## Stage B — `SumoRouteGraphNav` (the route-graph `IPedNavigation`)

### B1 — Node/edge graph + spatial index  *(design §3)*
- **Files:** new `src/Sim.Pedestrians/Navigation/RouteGraph/SumoRouteGraphNav.cs` (+ small helper types).
- **Deps:** none (consumes `PedNetwork`).
- **Do:** build lane nodes (sidewalk/crossing centrelines, walkingarea polygons) in deterministic order;
  build bidirectional adjacency + portal points from `PedConnections`; build the uniform-grid nearest-lane
  index.
- **Success:**
  1. Constructs from a `PedNetwork` with no exceptions; node count == sidewalks+crossings+walkingareas.
  2. `NearestLane(p)` returns the correct lane for points on and near known lanes (unit test on the fixture).
  3. Adjacency is symmetric and matches the `PedConnections` count (each connection → both directions).

### B2 — `FindPath` (A* + polyline assembly)  *(design §4, §4.1)*
- **Files:** same as B1.
- **Deps:** B1.
- **Do:** implement `FindPath(start, goal)`: nearest-lane endpoints, same-lane sub-path, A* over the graph,
  `AssemblePolyline` threading sidewalk/crossing centrelines (oriented) and walkingarea straight chords.
- **Success:**
  1. On the connected fixture, `FindPath` between two sidewalk points returns a non-null polyline whose every
     vertex lies within its owning lane half-width of a ped lane (assert on-network).
  2. A path that must cross a road returns a polyline that **passes through a crossing** node (assert a
     `Crossing`-owned vertex is present).
  3. On a **disconnected** fixture (two ped islands), `FindPath` across the gap returns `null`.
  4. `start`/`goal` in the same lane returns a 2+-point sub-path along that lane.

### B3 — `HalfWidthsAlong`  *(design §4.2)*
- **Files:** same as B1.
- **Deps:** B2.
- **Success:** `HalfWidthsAlong(path)` returns one value per vertex, each equal to the owning lane's width/2
  (±ε) for sidewalk/crossing vertices; nominal for walkingarea vertices; never the bare 0.5 default when the
  path came from this provider (unit test).

### B4 — Determinism  *(design §3.3, §10)*
- **Files:** same as B1.
- **Deps:** B2.
- **Success:** the same `FindPath(start,goal)` called twice returns identical polylines (sequence-equal); a
  property test over many seeded O/D pairs shows no `System.Random` and stable output. No allocation of
  `System.Random` anywhere in the class (grep-asserted in the test or a code check).

---

## Stage C — Road-net mode wiring in `LiveCitySim`

### C1 — Mode branch: build `SumoRouteGraphNav`, skip the bake  *(design §5.2, §5.4)*
- **Files:** `src/Sim.LiveCity/LiveCitySim.cs`.
- **Deps:** A1, A3, B2.
- **Do:** branch ctor on `cfg.NavMode`; in `RouteGraph` build `SumoRouteGraphNav(pedNetwork)` (no
  `WalkablePolygonBaker.Bake` of sidewalks), bypass crop predicates, centre the realism zone on the net AABB
  centre.
- **Success:**
  1. In `RouteGraph` mode `WalkablePolygonBaker.Bake` is **not** called for sidewalks (assert via a test seam
     / instrumentation, or assert the nav object type is `SumoRouteGraphNav`).
  2. In `Navmesh` mode the construction path is **unchanged** (demo scene + dense-flow liveness tests pass).
  3. Road-net construction on the committed fixture completes and `Step()` runs N steps without throwing.

### C2 — Crossings-only bake + gate/signals + walk-only degrade  *(design §5.3, §6)*
- **Files:** `src/Sim.LiveCity/LiveCitySim.cs`; helper extraction in
  `src/Sim.Pedestrians/Navigation/Bake/WalkablePolygonBaker.cs` (`BakeCrossingsOnly`, additive).
- **Deps:** C1.
- **Success:**
  1. In road-net mode with crossings, `CrossingOccupancySource` + `CrosswalkSignals` are wired from
     crossings-only baked polygons; crossing-occupancy count goes >0 when a ped is on a crossing (fixture
     test).
  2. `BakeCrossingsOnly` returns the **same** `Crossing` polygons the full `Bake` would (unit test comparing
     the Crossing subset).
  3. Sidewalks-but-no-crossings → no gate/signals wired, peds still walk (walk-only), no throw.

### C3 — O/D sampling from sidewalk centrelines  *(design §5.5)*
- **Files:** `src/Sim.LiveCity/LiveCitySim.cs`.
- **Deps:** C1.
- **Success:**
  1. `Origins`/`Destinations` are populated deterministically from `PedNetwork.Sidewalks` (seeded stride,
     bounded count), no `System.Random`; two constructions with the same seed produce identical O/D sets.
  2. Peds spawn and reach destinations on the fixture (`ArrivedTotal`-equivalent for peds > 0 over the run).

### C4 — `RerouteDriver` invariant  *(design §5.7)*
- **Files:** test only.
- **Deps:** C1.
- **Success:** the road-net smoke test asserts the nav object is a `SumoRouteGraphNav` and that no
  `RerouteDriver` / concrete `SumoNavMesh` is constructed in road-net mode (type assertion / no-throw).

---

## Stage D — Config surfacing

### D1 — Promote ped-demand knobs to `LiveCityConfig`  *(design §7)*
- **Files:** `src/Sim.LiveCity/LiveCityConfig.cs`, `src/Sim.LiveCity/LiveCitySim.cs`.
- **Deps:** A1.
- **Success:**
  1. `PedMaxSpeed`, `PedRadius`, `PedArrivalRadius`, `PedEnableWeave`, and the liveliness fields exist on the
     config, defaulted to the current literals; the ctor reads them from `cfg`.
  2. The demo `PedDemandConfig` built via `ForRepoRoot` is **byte-identical** to today (unit test on the
     resulting field values); dense-flow liveness + scene tests unchanged.

---

## Stage E — Offline prep, fixture, tests

### E1 — Offline prep recipe + `scripts/prep-ped-net.sh`  *(design §8)*
- **Files:** `scripts/prep-ped-net.sh`, a recipe section in the fixture README / scoping.
- **Deps:** none.
- **Success:** the script runs `netconvert --sidewalks.guess --crossings.guess --walkingareas.all-nonspecific`
  on an input net and produces a ped-equipped output; documented; **not** referenced by `dotnet test`.

### E2 — Committed synthetic road-net fixture  *(design §9)*
- **Files:** `scenarios/_ped/roadnet_min/{*.net.xml, README.md, provenance.txt}`.
- **Deps:** E1 (uses the same tooling).
- **Success:**
  1. A tiny ped-equipped net (e.g. `netgenerate --grid --grid.number=3 --sidewalks.guess
     --crossings.guess`) is committed; contains sidewalks + crossings + walkingareas + ped connections;
     **no proprietary data**.
  2. Loads via `LiveCityConfig.ForDataset` with **no SUMO present** (offline).
  3. `provenance.txt` records the exact generation command + SUMO version.

### E3 — Unit + smoke/regression tests  *(design §9)*
- **Files:** `tests/Sim.Pedestrians.Tests/*`, `tests/Sim.LiveCity.Tests/*`.
- **Deps:** B4, C1-C4, D1, E2.
- **Success:**
  1. `SumoRouteGraphNav` unit tests (B1-B4 conditions) pass.
  2. Road-net smoke test: fixture → `ForDataset` → N steps → peds route (LiveCount>0), cross a crossing,
     nav is `SumoRouteGraphNav`; deterministic (two runs identical peak/arrival metrics).
  3. Bare-net variant: `PedestriansEnabled==false`, vehicles run, no throw.
  4. Whole suite (`dotnet test`) green **without SUMO installed**.

### E4 — Coordinate robustness  *(design §9)*
- **Files:** a fixture or unit test with large/negative/3-D coords.
- **Deps:** B2.
- **Success:** parse → `SumoRouteGraphNav.FindPath` → O/D sampling all succeed on coordinates that are
  large-magnitude, negative, and carry a z component (no overflow, no NaN, on-network polyline).

---

## Stage F — Final parity/bench/demo gate (Opus review)

### F1 — Accept gate  *(design §10)*
- **Deps:** all above.
- **Success (Opus verifies first-hand, does not trust reports):**
  1. `dotnet test tests/Sim.ParityTests` — 654/4 byte-identical.
  2. `Sim.Bench` determinism hash — unchanged.
  3. `tests/Sim.LiveCity.Tests` — scene + dense-flow liveness green (demo `Navmesh` path proven untouched).
  4. Full `dotnet test` green on a fresh checkout **without SUMO**.
  5. No `Sim.Core` diff; no `System.Random` introduced; netstandard2.1 intact; consumer contract
     (`VehicleSource`/`PedSource`/`LocalLanes`/`Time`/`Step`/`Dispose`) unchanged.
