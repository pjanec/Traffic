# R1 — net-connection navmesh stitch (design)

**Status:** IMPLEMENTED & tested. **HOW** for `SUMOSHARP-DEMO-CITY-REQUIREMENTS.md` R1
("navmesh baker robustness … stitch portals from the net's own lane/walkingArea/crossing connectivity").
The mechanism landed as `PolygonGraph.AddNetConnectionAdjacency` (`src/Sim.Pedestrians/Navigation/Bake/`)
and is proven by `NavConnectionStitchTests` (a hermetic three-party-corner witness); the Q3 ped-isolated-
component warning ships in `SubareaFcdRecorder`. The **WHAT** is that requirement + the box's split-bake.

## 1. Diagnosis — the box's 6 components are THREE different failures, not one

Baking `scenarios/_ped/demo_city/box` gives `components=6` = `[1027, 36, 7, 1, 1, 1]`. Localized via
`--ped-navmesh-components-csv` (`box-components.csv`) + a scan of the net's 3976 `<connection>` elements:

| component | polys | what the net says (verified at LANE granularity) | verdict |
|---|---|---|---|
| **0** | 1027 | the city | main |
| **1** | 36 (dining-plaza) | walkingAreas connect internally + to **vehicle** edges only; **0** ped-lane connections to any other component | **NET-authoring gap** |
| **5** | 7 (ring-NW) | **0** ped-lane connections to any other component | **NET-authoring gap** |
| **2/3/4** | 1 each (`:q_*_w0`) | isolated single walkingArea pieces that connect nothing pedestrian | **benign orphan artifacts** |

**Correction — verified at LANE granularity (supersedes an earlier edge-level read).** A first scan matched
connections by *edge* id and appeared to show comp 5 with 15 cross-component ped links — implying a baker miss.
That was wrong: SUMO ped connectivity (and the stitch) is **lane**-level, and re-scanning at lane granularity
(`from + "_" + fromLane`) finds **1377 ped-lane connections, ALL within-component, ZERO crossing any boundary**.
The comp-5 "links" were vehicle-lane connections between edges that merely also carry a sidewalk. So **every
residual — plaza, ring-NW, and the stubs — is ped-isolated in the net itself.** There is no declared pedestrian
connectivity to stitch across any boundary, so the connection-stitch (correct + shortcut-safe) is a **no-op on
this box**: it cannot merge what the net does not connect. **All of the box's fragmentation is a citygen
authoring gap, not a baker limitation** (confirmed by SumoData: `build_dining` downgrades the plaza interior to
`allow="pedestrian"` but authors no ped link across the perimeter junctions, unlike the park's `build_ped_hub`
gate edges — and the ring-NW has the same gap).

So R1 delivered two things, and neither is a stitch of *this* box: **(a)** the connection-stitch as a correct,
tested, general mechanism (it merges any net that *does* declare a geometry-missed ped link — proven by
`NavConnectionStitchTests`, inert here); and **(b)** the automatic **ped-isolated-component warning** (§ below)
that turns the by-hand scan into a bake-report signal and correctly flags all 5 residuals as net gaps. The fix
itself is SumoData's citygen gate-edge pass.

## 2. Why geometry alone misses comp 5 (and the fix's premise)

`PolygonGraph.BuildAdjacency` (`PolygonGraph.cs`) is **purely geometric**: shared-edge portals, 2-party
shared-corner portals, and P8-1c's collinear spine-continuation bridge. It deliberately refuses the ambiguous
3+-polygon corner and non-collinear touches (the anti-shortcut invariant). At a heterogeneous-approach junction
netconvert splits the walkingArea into pieces whose baked polygons abut only at ambiguous corners — so the
geometric pass can't safely link them, even though the net's `<connection>` elements say a pedestrian walks
straight through. The premise of the fix: **the net's declared pedestrian connectivity is authoritative and
shortcut-safe** (a `<connection>` is a real, walkable move), so it is the right source to stitch from — exactly
what R1 asks.

## 3. The connection-stitch (HOW)

A new **non-geometric adjacency pass** that adds portals between baked polygons the net declares connected for
pedestrians, composed on top of the existing geometric passes.

1. **Parser (`PedNetworkParser`) reads `<connection>` elements.** Today it reads only `<edge>`s. Add a pass
   over `root.Elements("connection")`, keeping a connection when **both** its `from` and `to` resolve to a
   pedestrian polygon (a sidewalk lane, crossing, or walkingArea in the already-parsed sets). Expose them on
   `PedNetwork` as `IReadOnlyList<PedConnection>(FromPolyId, ToPolyId)` — resolved to the **baked-polygon id
   space** (see §4 id-matching) so the baker can use them directly. Directionality is irrelevant for
   connectivity (pedestrian moves are bidirectional on the navmesh), so store an unordered pair.

2. **Baker threads the connections into `PolygonGraph`.** `PolygonGraph(polygons)` gains an optional
   `IReadOnlyList<PedConnection>` argument; after the geometric passes, a `AddNetConnectionAdjacency` pass adds
   a portal for each connection whose two polygons are not already adjacent. **Portal point:** the closest
   approach between the two polygons' boundaries (they abut in geometry — netconvert built them adjacent — the
   geometric pass just wouldn't *commit* to the ambiguous corner; the nearest-boundary midpoint is the
   doorway). Deterministic: connections applied in net-file order, ties by polygon index.

3. **Shortcut-safety (load-bearing).** A connection portal links only polygons the net **declares** a
   pedestrian walks between, so it cannot manufacture the POC-0 forbidden shortcut (two polygons with no
   pedestrian connection stay unlinked). This pass is *safe where the geometric pass was conservative* precisely
   because it has authoritative evidence the geometry lacked. The existing anti-shortcut guards stay unchanged;
   the connection pass only *adds* edges the net vouches for.

## 4. Seams / files (for the implementation stage)

- `PedNetwork.cs` — new `PedConnection(string AId, string BId)` record + list on `PedNetwork`.
- `PedNetworkParser.cs` — parse `<connection>`; resolve `from`/`to` (+ `fromLane`/`toLane`) to baked-polygon
  ids. **Id-matching detail to nail:** baked sidewalk polygons are keyed by *lane* id (`edge_0`), walkingAreas/
  crossings by their lane id (`:J_w0_0`); a `<connection from="edge" fromLane="0" to=":J_w0" …>` names *edge*
  ids + lane indices, so resolve `from` → `edge + "_" + fromLane` and keep only pairs where both resolve to a
  known pedestrian polygon. Non-pedestrian (vehicle-lane) connections are dropped here — that is what correctly
  leaves the plaza isolated.
- `Navigation/Bake/PolygonGraph.cs` — optional connections arg + `AddNetConnectionAdjacency`; build an
  `polyId → index` map once, add portals, skip already-adjacent pairs.
- `Navigation/Bake/SumoNavMesh.cs` + `WalkablePolygonBaker` callers — thread the connections from the parsed
  `PedNetwork` to `PolygonGraph`. (The recorder/`SubareaFcdRecorder` already has the `PedNetwork` in hand.)
- Tests — a component-count assertion on the committed box + witnesses.

## 5. What this deliberately does NOT fix — coordination with SumoData

- **The plaza-36** stays a separate component after the stitch, because there is **no pedestrian connection to
  stitch**. This is a **net-data item for SumoData**: the dining-plaza interior grid needs pedestrian
  connectivity to the surrounding sidewalks — crossings/walkingArea ped-connections at its boundary junctions
  (or, if it is intended shared-space, that is a separate navmesh feature, not R1). Until then, the plaza's
  meet-&-talk spot won't populate from cross-district demand — a *data* limitation, correctly surfaced, not a
  silent baker hole. **Ask to SumoData:** add ped connections/crossings linking the plaza boundary, or confirm
  shared-space intent.
- **The 1-poly corner stubs** (`:q_*_w0`) connect nothing pedestrian; they are benign (no ped ever routes onto
  them) and cost nothing. Optionally filtered as noise later; not worth a fix.

## 6. Determinism & parity safety

- **Additive & monotone.** The pass only *adds* adjacency, so it can never split a component — the committed
  witnesses already at `components=1` (grid, synthetic_pedfrag, pedfrag2, kernel) **stay 1** by construction.
- **No car parity impact** — this is the pedestrian navmesh; no `tolerance.json` moves.
- **Ped-route risk to check:** a new connection portal can open a *shorter* real route than the geometric graph
  had, which could shift a low-power ped trajectory in a POC ped scenario. Expected inert on the already-
  well-connected committed scenarios (the geometric pass already found their portals), but the offline gate is
  the guard — any shift is caught, and if a committed ped golden moves it is because a real net-declared route
  was previously missing (a fix, to be re-baselined deliberately, not silently).
- **No-shortcut invariant:** the POC-0 no-shortcut witness must stay green — a connection portal is only added
  where the net declares a ped connection, and the no-shortcut pair has none, so it stays unlinked.

## 7. Success conditions

- `box` bakes to **fewer components** — comp 5 merges into main (expect `components` drop from 6 to ~4:
  1 main + plaza-36 + the 3 corner stubs); `unreachableSkips` stays 0.
- kernel + all committed witnesses stay at their current component counts (1).
- POC-0 no-shortcut test stays green; the full offline gate stays green.
- A test asserts the box's post-stitch component count and that the ring-NW edges now share the main
  component's label.
- The plaza + stubs are documented as the SumoData data item (not counted as a baker failure).

## 8. Open questions (for review)

1. **Scope of "pedestrian connection".** Keep a connection only when both ends are already-parsed ped polygons
   (sidewalk/crossing/walkingArea) — recommended, and it's what correctly excludes the plaza's car-only links.
   Confirm no ped path legitimately routes *through* a vehicle lane in these nets (shared-space) that we'd be
   dropping. (If shared-space peds-on-roads is in scope for the plaza, that's a separate, larger design.)
2. **Portal point.** Nearest-boundary midpoint between the two polygons (recommended). If a pair is declared
   connected but geometrically far apart (shouldn't happen for real netconvert output), cap the stitch distance
   and log — a far "connection" is a data smell worth surfacing rather than bridging across space.
3. **Plaza hand-back.** Do we (a) just report the plaza gap to SumoData and leave it, or (b) also add a
   recorder `--reachable-filter`-style acceptance that treats "ped-isolated district" as a first-class warning
   with the junction list? Recommendation: (a) now (report), (b) only if it recurs on real nets.

## 9. Staged tasks (implementation later — success conditions now)

- **R1-a — parser reads ped connections.** `PedConnection` + `PedNetworkParser` connection pass + id
  resolution. **Success:** the box parses N ped connections; a unit test resolves a known
  `sidewalk→walkingArea→sidewalk` chain to the right baked-polygon ids; vehicle-only connections are dropped.
- **R1-b — connection-stitch adjacency pass.** `PolygonGraph.AddNetConnectionAdjacency` threaded from the
  parsed network. **Success:** box `components` drops (comp 5 merges); witnesses stay 1; no-shortcut green;
  offline gate green; a component-count test on the box.
- **R1-c — report the plaza data gap to SumoData** with the junction list (`q_0_1…q_3_2`) and the
  "no ped connection across the boundary" finding, as the coordination item.

Sequencing: R1-a → R1-b → R1-c. Blocked on nothing (the box + witnesses are committed).
