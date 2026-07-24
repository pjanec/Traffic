# Scoping — run LiveCitySim on an arbitrary SUMO road-net (with routed pedestrians)

**Status:** SCOPING AGREED (owner discussion, 2026-07-24). Boundaries below are settled; the full
`design -> tasks -> tracker` trio (per `CLAUDE.md`) is written against this memo, not the other way round.

This memo captures **what we will and will not build**, the **decisions** taken, and the **evidence** that
the chosen approach is safe. It deliberately does not restate the requirements narrative (that lives in the
originating handoff); it records the *boundaries* the design must respect.

---

## 1. Goal (one sentence)

Let `Sim.LiveCity.LiveCitySim` run on an **arbitrary, already-prepared SUMO road-net** supplied as a dataset
folder — with pedestrians **routed on the network** (walking sidewalks, crossing at crosswalks) rather than
free-wandering — while the existing `demo_city/box` demo keeps working byte-for-byte as the golden dev/test
loop.

The consumer contract (`VehicleSource` / `PedSource` / `LocalLanes` / `Time` / `Step` / `Dispose`, all
netstandard2.1) is **preserved unchanged**; a consumer only ever passes a different dataset dir.

---

## 2. The central architectural decision — two navigation regimes

Pedestrian navigation in this repo is already behind a swappable interface, **`IPedNavigation`**
(`src/Sim.Pedestrians/Navigation/INavigation.cs:37-56`), whose entire surface is:

- `IReadOnlyList<Vec2>? FindPath(Vec2 start, Vec2 goal)`
- `IReadOnlyList<double> HalfWidthsAlong(IReadOnlyList<Vec2> path)`  (default-implemented, 0.5 m)

`PedDemand` and `PedLodManager` depend on the **interface**, never on the concrete `SumoNavMesh`. Everything
downstream (`PathArcMotion`, `WaypointFollower`, `PedRouteController`, all of `Sim.Core`) consumes only an
opaque `Vec2` point-sequence. The seam is proven real by an existing independent production implementation,
`DotRecastNavMesh : IPedNavigation`, plus several stubs/test doubles.

Given that, we split pedestrian navigation into **two regimes sharing the same downstream**:

| Regime | Used for | Navigation | Data it needs |
|---|---|---|---|
| **Road-net import** (THIS task) | arbitrary SUMO road-nets | route on the SUMO **ped edge/connection graph** already in `net.xml`; O/D sampled along sidewalk/crossing lane centrelines within lane width | just `net.xml` (sidewalk lanes + crossings + walkingareas + ped connections) |
| **Navmesh** (existing, unchanged) | live-city demo, parks/plazas, evacuation | `SumoNavMesh` (baked walkable polygons + A*) | extra walkable-polygon / obstacle data a bare road-net does not contain |

**Rationale.** SUMO itself does not use a navmesh for pedestrians — its ped model walks people along
sidewalk lanes and through walkingareas/crossings, routed over the edge/connection graph. Route-graph
navigation is therefore the *faithful* model for a road-net-only import (parity-aligned), and the navmesh is
a SumoSharp enhancement for **free-space** realism (weaving around benches/tables/trees, panicked crowds
avoiding buildings) — behaviour that is neither expressible in, nor derivable from, a bare road-net. A
road-net import needs nothing more than "walk the sidewalks, cross at crosswalks", which is exactly what the
edge graph encodes.

**Scale consequence (the reason this matters).** The dominant bake cost is `WalkablePolygonBaker` buffering
every sidewalk lane into a mitred polygon strip plus a polygon-adjacency graph and spatial index. A
validation target (a large geo-projected real-world SUMO net, ~45 MB, ~21k pedestrian lanes) makes that
prohibitive. Route-graph navigation needs only lane centrelines + a connection adjacency list, so the ped
bake stops being the bottleneck. Crowd size is bounded by `PopulationCap` in either regime.

---

## 3. No crop in SumoSharp

Cropping a subarea out of a larger network — and calibrating it to find the maximum safe traffic density —
is an **offline** concern, handled by the external Python/SUMO pipeline that prepares the dataset. The net
handed to SumoSharp is already the exact subset, already density-tuned.

Therefore:

- **SumoSharp always ingests the whole road-net it is given and never crops it** (road-net-import mode).
- The originating requirement to "default the crop to the net's real extent / parse `convBoundary`" is
  **dropped** as a crop feature. `convBoundary` parsing is at most an optional convenience for centring the
  realism zone / reporting extent, and is derivable from lane shapes if wanted — not a gate.
- Density is a **user-selected config knob**, not computed by SumoSharp. `LiveCityConfig` already exposes
  `CarTargetConcurrent`, `PedPopulationCap`, and spawn rates; road-net mode honours them, and the currently
  ctor-hardcoded ped-demand knobs are surfaced to config.
- The demo's hero-block crop (`demo_city/box`, `X0/Y0/X1/Y1`) is retained **as a demo/navmesh-mode artifact
  only** — it is the locked regression fixture and must not change.

---

## 4. Scope

### In scope
- **R1** — a public dataset factory: `LiveCityConfig.ForDataset(datasetDir)`; `ForRepoRoot` delegates to it
  with the demo path. Demo path unchanged.
- **Road-net-import mode** — a mode on `LiveCityConfig`/`LiveCitySim` that:
  - selects a new `SumoRouteGraphNav : IPedNavigation` (route on the ped edge/connection graph, return the
    concatenated lane-shape polyline threading walkingarea + crossing geometry; `HalfWidthsAlong` returns
    real lane half-widths),
  - **skips the walkable-polygon bake entirely**,
  - operates on the whole net (no crop).
- **R3 capability detection + graceful degrade (no throw):** probe whether the net has pedestrian infra;
  full coupled cars+peds when present, **vehicles-only** when absent, **walk-only** when sidewalks exist but
  no crossings. Wrap the ped load so a malformed ped-net degrades rather than throws. Expose the active mode
  (e.g. `LiveCitySim.PedestriansEnabled`).
- **Drivable edges from `net.xml`** — derive vehicle spawn edges from vehicle-allowed lanes (the parser
  already tags ped-only lanes via `LaneAllowsRoadVehicle`), so an arbitrary net needs no `.rou.xml`.
- **Config surfacing** — promote the ctor-hardcoded ped-demand knobs (spawn rate, cap, max speed, weave,
  signal compliance) to `LiveCityConfig`.
- **Offline prep** — document (+ a `scripts/` helper) the sanctioned `netconvert` recipe that makes a bare
  net ped-capable (`--sidewalks.guess --crossings.guess`, walkingareas as needed).
- **Robustness verification** — confirm the ped/vehicle pipelines tolerate large-magnitude, negative, and
  3-D (elevation) shape coordinates (the demo net exercised none of these).
- **Pedestrians avoid SUMO cars (high-power/ORCA)** — a real-world requirement: peds walk **around a car
  stopped on a crosswalk** (jammed junction) and around abandoned cars. Feed live vehicle discs to the ped
  crowd (`OrcaCrowd.SetExternalObstacles`) — **navmesh-independent**, one-sided (cars already avoid peds via
  the existing yield coupling). Only high-power peds avoid (LOD design); config-gated, **on** for road-net
  import, **off** for the demo so its liveness regression stays byte-identical.

### Out of scope (this round)
- **Any crop feature** — done offline (§3).
- **Density calibration** — done offline in the Python/SUMO pipeline (§3).
- **Navmesh for road-net mode** — reserved for live-city/evacuation datasets that carry walkable-polygon /
  obstacle data.
- **ORCA walkable-area confinement** — the existing "ORCA steers toward navmesh/route waypoints but is not
  walled in" behaviour is kept as-is (see §5 decision 1).
- **Runtime pedestrian-infrastructure generation** — no `netconvert` at runtime; the embedded host has no
  SUMO binary (C2).
- **Reading `personFlows` / demand files for O/D** — deferred; road-net mode uses companion-free centreline
  O/D sampling. Documented as a future enhancement (real edge-based ped demand exists in typical datasets
  and is an attractive later source).
- **Polygon / TAZ crops** — n/a given §3.
- **Low-power pedestrian obstacle avoidance** — not planned. By LOD design, low-power PathArc peds avoid
  nothing; only high-power (ORCA) peds avoid cars/each other (see the in-scope item below).

---

## 5. Decisions taken

1. **ORCA drift** — accept the pre-existing behaviour (ORCA is local collision-avoidance steering toward the
   route/navmesh goal, not walkable-area confinement). Not fixed here. On a road-net with sparse waypoints an
   ORCA ped can transiently drift off the strip during avoidance; acceptable and unchanged from today.
2. **No crop** — SumoSharp always works on the whole net; crop is offline (§3).
3. **Sidewalks but no crossings** — peds walk, never cross; still pedestrian-enabled. Fully acceptable.
4. **Offline `netconvert` prep is the sanctioned path** — no runtime ped-infra generation.
5. **`personFlows` O/D deferred** — centreline O/D sampling this round.

---

## 6. Constraints the design must respect (iron laws)

- **Parity/bench untouched (R5):** `dotnet test tests/Sim.ParityTests` stays byte-identical; the
  `Sim.Bench` determinism hash is intact. Every new path is **additive and demo-inert** — the demo stays on
  the navmesh regime, and parity/bench drive the `Engine` directly, never `LiveCitySim`.
- **No `System.Random`; per-entity seeded RNG only.**
- **Do not edit `Sim.Core` motion math** for this — the seam is `IPedNavigation`; nothing below it changes.
- **netstandard2.1** preserved across the consume chain.
- **`demo_city/box` remains the golden dev/test loop and a committed regression fixture (R6).**
- **Committed-vs-ephemeral split honoured;** the offline test loop never invokes SUMO.

---

## 7. Evidence base (why this is safe)

- **Seam is clean and real:** `IPedNavigation` = `FindPath` + `HalfWidthsAlong`; `PedDemand`/`PedLodManager`
  hold the interface; callers treat the result as an opaque `Vec2` sequence; a second production
  implementation (`DotRecastNavMesh`) already exists. A route-graph `IPedNavigation` drops in with no change
  to `PedDemand`, `PedLodManager`, `PathArcMotion`, `WaypointFollower`, or `Sim.Core`.
- **O/D is caller-supplied**, not from the navigation object — road-net mode controls centreline O/D
  sampling in `LiveCitySim` without touching the seam.
- **Baker is safe on a bare net:** `WalkablePolygonBaker.Bake` returns an empty list (no throw); the throw
  risk lives upstream in `PedNetworkParser` on a *malformed* ped-net (a crossing/walkingarea edge with no
  ped lane, or a malformed internal id) — hence R3 wraps the ped load.
- **Vehicle side already tolerates ped-equipped nets:** `NetworkParser.LaneAllowsRoadVehicle` treats
  `allow="pedestrian"` lanes as non-drivable, so vehicle spawn-edge derivation from `net.xml` is
  straightforward.
- **Validation target confirms feasibility:** a large real-world geo-projected SUMO net (~45 MB, ~21k ped
  lanes, ~220 crossings, ~2.2k walkingareas) is ped-equipped and its ped graph *stitches* (thousands of
  sidewalk->walkingarea and walkingarea->crossing connections) — i.e. edge-graph routing has a connected
  graph to work with. The genuine risk on *poorly-prepared* nets is broken ped connectivity, which manifests
  as `FindPath` returning null and spawns being skipped (few/no peds) — graceful, no crash — and is
  addressed by the offline prep recipe.

---

## 8. Loose ends to resolve in the design doc (not blockers)

1. **Wiring point:** `LiveCitySim.cs:115` instantiates `SumoNavMesh` directly — mode selection lives here.
2. **`RerouteDriver`** holds the *concrete* `SumoNavMesh` (navmesh-only blocked-polygon overload) — confirm
   it is not wired in road-net mode (or gate it to navmesh mode).
3. **Crossing gate / signals geometry:** `CrossingOccupancySource` and `CrosswalkSignals.FromNet` currently
   take `BakedPolygon`s; in road-net mode feed them lightweight crossing geometry built from crossing *lane*
   shapes + width instead of a full bake.
4. **Route through walkingareas:** the route-graph path must thread walkingarea lane shapes (open
   junction-corner polygons) and crossing centrelines, following the ped-connection chain — the core work of
   the new nav provider.

---

## 9. Definition of done (scoped)

A public way to run `LiveCitySim` on an arbitrary **ped-equipped** SUMO road-net folder (whole net, no
crop); peds walk sidewalks and cross at crossings via route-graph navigation (no bake); a net with sidewalks
but no crossings runs walk-only; a bare vehicle-only net runs vehicles-only without throwing; the active mode
is queryable. The `demo_city/box` demo is byte-for-byte unchanged and stays on the navmesh regime. Parity
(654/4) and the bench determinism hash are intact. netstandard2.1 and the consumer contract are preserved.
The offline net-prep recipe is documented.
