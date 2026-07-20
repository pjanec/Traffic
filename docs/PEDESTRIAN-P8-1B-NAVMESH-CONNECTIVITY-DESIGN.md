# PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md — connect the walkable bake on real geometry (HOW)

**Status: implemented (hermetic gate green); awaiting a real-net fixture for end-to-end acceptance.** Fix for the fragmentation reported in
`docs/SUMOSHARP-P8-1-REAL-NET-NAVMESH.md` (the WHAT): on a real ~2 km crop the walkable bake shatters into
~1000 disconnected components (synthetic grid → 1), so O/D routing fails and the crowd can't populate
(peak 3 vs 203). Tracked as **P8-1b** (`PEDESTRIAN-TRACKER.md` Stage P8).

## 1. Root cause (confirmed by tracing the pipeline)

Adjacency between baked polygons is computed **100% geometrically** in `PolygonGraph.BuildAdjacency`, by two
passes only:
1. **shared-edge** — two polygons share an edge with the same two endpoints within `AdjacencyEpsilon = 1e-3`
   (1 mm), portal = midpoint of the shared edge(s);
2. **shared-vertex** — union-find clusters of coincident vertices (within 1 mm), but a portal is added
   **only for clusters touched by exactly two polygons** — clusters where **3+ polygons meet are skipped**
   (the POC-0 fix: a crossing + walkingArea + far sidewalk meeting at one corner point must *not* connect
   crossing→sidewalk directly, or A* shortcuts across the non-walkable notch outside the corner).

Routing (`SumoNavMesh.FindPath`) depends **only** on this adjacency (A* over the portal graph); a pair with
no portal is unreachable and `FindPath` returns null. Containment (`SumoWalkableSpace`) is a union test and is
unaffected — the surface is *coverable* but not *routable*.

**Why real geometry fragments:** netconvert emits each sidewalk lane as an **independently-buffered strip**,
plus separate `walkingArea`/`crossing` polygons. Where a sidewalk meets a junction's walkingArea they
**abut by overlapping ~0.05–0.2 m** (documented in `RerouteTests`' remarks) rather than sharing an exact edge
or a coincident vertex. Neither geometric pass fires on a sliver overlap, so no portal forms and the surface
splits. The synthetic grid happens to be regular enough that its polygons share exact edges/vertices, so it
showed 1 component and hid the gap.

## 2. The fix — an additive **area-overlap** adjacency pass

Add a THIRD adjacency pass to `PolygonGraph` that connects two polygons when their 2D areas **genuinely
overlap** (not merely touch at a corner). This is exactly the abutment relationship real buffered geometry
has, and it is **invariant-safe by construction**:

- At a junction, a **sidewalk** strip and a **crossing** each overlap the junction's **walkingArea**, but
  **not each other** (they abut the area from different sides). So an area-overlap test connects
  sidewalk↔walkingArea and crossing↔walkingArea (routing *through the area*, the correct path) and leaves
  crossing↔sidewalk unconnected — **the POC-0 no-shortcut invariant holds without any kind-based special
  casing.** (The POC-0 bug came from a shared *corner point*, which is a touch, not an area overlap.)
- The portal lands **inside the overlap region** (inside both polygons ⇒ inside the walkable union), so the
  A* segments through it stay walkable — unlike the corner-point portal the vertex pass rightly refuses.

**Overlap detection (cheap, no full polygon clipping):** two polygons overlap iff
- a vertex of one lies **strictly inside** the other (strict = inside by more than a small margin, so a
  boundary/corner touch does NOT count — this is what preserves the POC-0 corner), **or**
- an edge of one **properly crosses** an edge of the other (proper = interior segment intersection, not a
  shared endpoint).

Portal point:
- if a strictly-contained vertex exists, use it (guaranteed inside both);
- else use the midpoint of a proper edge-crossing (on both boundaries, inside the union).
When several qualify, average the candidates for one representative doorway (mirrors the shared-edge pass),
deterministically.

**Additive + dedup ⇒ parity-safe.** The pass runs AFTER the two existing passes and **only adds a portal for
a pair that has none yet**. On the synthetic box and POC-0 every legitimate pair is already connected by the
shared-edge/vertex passes, so the new pass finds them already-connected and adds nothing → those bakes are
**bit-identical**. It strictly *adds* the missing sliver-overlap portals that only appear on irregular real
geometry. `AdjacencyEpsilon` itself is left at 1 mm (untouched) — we add a relationship, we don't loosen the
existing ones, keeping the existing passes' behaviour exactly.

## 3. Measuring connectivity (new, for acceptance)

There is no connected-components notion today. Add a small deterministic `SumoNavMesh.ConnectedComponentCount()`
(BFS/union-find over the portal graph) so tests and the recorder can assert "1 (or few) components" instead of
inferring it from routing. Pure, offline, no per-step cost.

## 4. Determinism & parity

- The new pass is pure geometry over the deterministically-ordered bake, O(n²) pairs (offline bake cost, not
  per-step), neighbour lists re-sorted by index exactly as today ⇒ A* order unchanged.
- No `System.Random`. No engine/parity-core touch. All new work is in `PolygonGeometry` + `PolygonGraph` +
  a read-only navmesh accessor.
- **Parity bar (must stay green, from the committed test map):** `SumoBakeNavigationTests` (POC-0 route ≥3
  waypoints, stays walkable, deterministic), `BothProvidersAgreeTests`, `RerouteTests` (blocking `:c_c0_0`
  still reroutes and differs), `BentSidewalkBakeTests` (1 polygon per bent lane; 2-lane shared-edge connect),
  `SubareaBoxBakeTests` (>100 polys, lo→hi pathable), `PedNetworkParserTests`.

## 5. Test / repro strategy (hermetic — no real geometry needed)

The fragmentation is geometry-shape, not Geneva-specific, so it reproduces from **hand-built `BakedPolygon`s**
(the type is public; `SumoNavMesh` takes a polygon list directly — no net.xml, no SUMO):

- **`NavmeshConnectivityTests`** builds a mini-junction: a walkingArea square + two sidewalk strips + a
  crossing, all **abutting with ~0.1 m overlap and non-coincident vertices** (real-buffering shape).
  - *Before-fix assertion of the bug:* with only the old passes the component count is > 1 and a cross-junction
    `FindPath` returns null. *(Captured as the motivating case; the fixed graph makes it 1 / routable.)*
  - *After fix:* `ConnectedComponentCount() == 1`; cross-junction `FindPath` non-null and every sampled point
    walkable; **crossing and sidewalk are NOT directly adjacent** (only via the walkingArea) — the POC-0
    invariant, asserted directly on the graph.
- A **corner-touch** case (three polygons meeting at one shared point, POC-0 shape) asserts the new pass adds
  **no** crossing↔sidewalk portal (touch ≠ overlap).
- Re-run the full existing suite to confirm bit-identical behaviour on POC-0 / synthetic / bent fixtures.

(If a real-like irregular fixture is later wanted end-to-end, the sub-area session offered a netgenerate
`--rand --sidewalks.guess --crossings.guess` box; the hand-built repro above is the geometry-free equivalent
and is sufficient to develop and gate the fix.)

## 6. Tasks & success conditions

- [x] **P8-1b-1 — overlap primitives** in `PolygonGeometry` (`ContainsStrict`, `SegmentsProperlyIntersect`,
  `TryFindOverlapPortal`). *Done* — strict-inside excludes boundary/corner touches; proper-cross excludes
  shared endpoints. Covered by the connectivity tests below.
- [x] **P8-1b-2 — area-overlap adjacency pass** in `PolygonGraph` (`AddAreaOverlapAdjacency`; additive, dedup,
  area-anchored, deterministic). *Done* — the hand-built mini-junction goes to **1 component** and routes
  crossing→sidewalk **through the walkingArea**; non-area polygons are never bridged; POC-0/synthetic/bent
  bakes bit-identical (full gate green: 649 parity / 173 ped / 2 DotRecast).
- [x] **P8-1b-3 — `ConnectedComponentCount()`** on `SumoNavMesh`. *Done* — 1 on the synthetic box, 2 on a
  two-island input, 3 on the walkingArea-removed mini-junction.
- [x] **P8-1b-4 — recorder diagnostic**: `SubareaFcdRecorder.Result` now carries `WalkablePolygons` /
  `ConnectedComponents` / `UnreachableSkips`, and `--ped-subarea-fcd` prints them (with a `[WARN] fragmented
  navmesh` note when components > 5) so the next real-crop run self-reports connectivity.
- [ ] **P8-1b-5 — real-net end-to-end acceptance** (awaiting the sub-area session's `netgenerate --rand`
  irregular box). *Success:* commit it as a fixture; `ConnectedComponentCount()` is 1 (or few), the recorder's
  `UnreachableSkips` ≈ 0, and the crowd populates to the dialed density (not 3).

## 7. Invariants

- Additive/dedup: existing goldens & fixtures bit-identical (the new pass only fires where the old passes
  found nothing).
- POC-0 no-shortcut invariant preserved **by geometry** (corner touch is not an area overlap; portals sit
  inside the overlap, never across a non-walkable notch).
- No `System.Random`; deterministic bake; no parity-core / engine change.
