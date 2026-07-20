# SumoSharp requirements — the composed demo-city (Prototype E substrate)

**For the SumoSharp pedestrian/engine session.** SumoData is building a single reusable **synthetic
~5×5 km demo city** (`experiments/subarea/demo_city/`, design: `SUBAREA-DEMO-CITY-DESIGN.md`) to
demonstrate and test *every* vehicle + pedestrian feature and their interactions, in both `sim_viz` 2D
and City3D 3D. It is the composed **V2** successor to the `synthetic_demo` catalog box, and it is the
**Prototype E** acceptance substrate you flagged as still-waiting.

**It is fully synthetic (no real geometry, no place tokens) — it may live in the SumoSharp repo.** We
hand you the net + companion data; this doc lists the engine/ped support that must exist for the box to
demonstrate its features. Ordered by criticality.

The weave (PED-REALISM-1 W1–W4) is done and behind `EnableWeave` — this box is where it gets turned on
in a real showcase, and where the remaining V2 behaviors need productizing.

**Status (2026-07-20): the box is COMPLETE and packaged.** All four stages built + pushed on SumoData
branch `claude/subarea-handoff-docs-a3f1jd`; handed over as a tgz (`demo_city_handoff.tgz` — `kernel/`
+ `box/` + both docs). It is fully synthetic (scanned clean of place tokens). You can bake and drive it
today. **The one thing that blocks you on first contact: your recorder throws
`FormatException: unknown POI kind` on the new `parking_lot`/`park` kinds** — `PedPoiReader.cs` /
`PedNetworkParser` must learn them before it will load `box/pois.json` unfiltered (see R2/R5). Our own
2D replay worked around this by feeding the recorder a v1-kinds-only POI list; that is the temporary
seam, not a fix.

---

## R1 — Navmesh baker robustness on composed geometry *(critical, gates everything)*

The city deliberately contains the shapes that fragmented the baker in P8-1: **roundabouts, multi-lane
junctions with large walkingAreas, irregular arterials, lane-count changes, and a ped-only park path
graph.** You want to "build the navmesh once" — so the baker must bake this net to **few large connected
components** (ideally 1 + the isolated park island bridged at its gates), with unreachable O/D ≈ 0.

- We deliver a **Stage-1 feature kernel** (`demo_city/kernel/`, ~1×1 km, one of every junction type +
  a roundabout + multi-lane + a ped-only park) **specifically as your shareable navmesh stress-test**,
  before the full 5×5 km. Bake it; report `components` / `unreachableSkips`.
- If it fragments: the P8-1 ask stands — stitch portals from the **net's own lane/walkingArea/crossing
  connectivity** (which `PedNetworkParser` reads) rather than 1 mm polygon-corner geometry, and handle
  the 3+-polygon corner. The kernel is the geometry-free repro to fix it against, once.
- **Acceptance:** kernel and full box each bake to ≤ few components; the recorder populates to the
  dialed density (not routing-limited); the park path graph connects to the street sidewalks at its
  gate nodes.

### What the bakes actually showed (concrete R1 datapoints — resolved vs. still on you)

Both artifacts are built and baked through your recorder (`e9ac56c`):

- **`kernel/` → `components=1, unreachableSkips=0`.** Every junction type + roundabout + zipper +
  allway_stop + the ped-only park baked into ONE component, first try. The P8-1 fragmentation is
  **geometry-shape, not present in regular synthetic geometry** — confirmed.
- **`box/` (full 5×5 km) → 1 main component (~96% of ~1049 polygons) + small residuals.** The park (the
  flagged risk) bakes clean into the main component via its 8 gates. Two classes of residual remain, and
  **the split between "our bug" and "your baker" is the useful part:**
  - *Ours, fixed:* several arterial-speed (16.67 m/s) edges — roundabout legs, zipper internals, the
    turn-lane-drop, fringe stubs — had **no explicit sidewalk width**, so netconvert's 15 m/s
    sidewalk-guess threshold silently dropped their sidewalks, isolating the mall district + two
    roundabouts. We now stamp explicit widths; those merged into the main component. (Heads-up for your
    own real-net handling: **arterial-speed edges lose guessed sidewalks** — a silent navmesh hole.)
  - *Yours, R1:* residual fragments — the **dining-plaza interior (36-poly)**, three single-polygon
    dining-corner artifacts, and a **7-poly ring-NW-corner** stretch — trace to netconvert's own
    **walkingArea splitting at heterogeneous-approach junctions**, which `PolygonGraph.cs`'s documented
    anti-shortcut invariant **correctly refuses to bridge**. This is a baker-level limitation, not a
    citygen shortcut. **Demo impact:** the dining-plaza meet-&-talk spot won't populate from
    cross-district demand while its interior is a separate component. This is the concrete, shareable,
    geometry-free repro for the net-connectivity-based stitching in R1 — the plaza interior in `box/` is
    the minimal case. **Update:** we tried the citygen-side mitigation (homogenising the plaza's
    approaches) — it did **not** move the residual (re-bake byte-identical), which *confirms* this is a
    baker-level limitation, not something the net author can paper over. So R1 stands as a real ask.
    Full box re-bake after all Stage-3 edges: **`components=6, unreachableSkips=0`** (1 main ≈ 1027 polys
    + the plaza-36 + three 1-poly + one 7-poly residuals).

## R2 — Data-driven micro-scenario registry *(headline behavior: waiter)*

Replace the hardcoded `WaiterScenario`/`SceneGen` anchors with a registry keyed off our data. A
`venue` record carries `scenario_template` (e.g. `waiter_v1`), `service_door` (a `building_entrance`
id), and `table_cluster` (`[{id,pos,capacity}]`). The registry instantiates the scenario at the venue:
peds arrive via the door, sit at a table, are served, dwell, leave. IG-deterministic like the rest.

- **We provide:** the venue records + door + table geometry + cross-refs in `pois/v2`.
- **You build:** the template→instance registry + `waiter_v1`; **first extend `PedPoiReader.cs` /
  `PedNetworkParser` to parse the new `parking_lot` and `park` POI kinds** — today the recorder throws
  `FormatException: unknown POI kind` on them, so nothing loads the full box until this lands. Confirmed
  empirically against the unmodified recorder.

## R3 — Shared parked-car representation + productized drive-away *(headline behavior: shop & drive off)*

Turn the `LotCoupling` POC into a data-driven, both-directions coupling on our `parking_lot` records:
`polygon`, `lane_seam` (lot↔carriageway), `parked_cars` (static poses), `boardable_car`
(`{id, exit_route:[edge…]}`).

- **Arrive → park → alight → walk:** a car routes to the lot's parkingArea, parks (residency — already
  fixed), the ped alights at the seam and walks to the venue.
- **Walk → board → drive off:** a ped walks to `boardable_car`, boards, the car departs on `exit_route`.
- Needs the **shared parked-car representation** both sessions flagged (so the parked dressing, the
  boardable car, and the SUMO parkingArea occupant are one entity, IG-consistent).
- **We provide:** the lot polygon + seam + car poses + boardable-car id + exit route. The **mall
  surface lot** has a full `boardable_car` + `exit_route` (two-way access, verified in SUMO: 6 shopping
  trips ran the full arrive→park→dwell→drive-away cycle). The **hidden garages + quiet-area lots** are
  dead-end stubs, so their churn is modeled as *separate* arriving/departing vehicles (a same-car round
  trip is impossible on a dead-end); no `boardable_car` there.
- **You build:** the productized coupling + the shared car rep + its IG-determinism contract.

## R4 — Hidden-garage birth/death (occluded) *(the "underground garage" realism requirement)*

A `parking_lot` with `hidden:true` is a garage: a SUMO `parkingArea` on a short off-road access edge
inside a building footprint. Cars route in and legitimately disappear, and a fraction of demand is
*born* there — no pull-in-and-vanish in open view. Residency is already correct; the ask is **City3D
occlusion** (the garage sits inside/under a building, so the birth/death is not visible) and treating
the garage access as a legitimate hidden sink/source in the demo.

## R5 — Park / ped-only region

The park is a green polygon with an internal **pedestrian-only** edge graph (`allow="pedestrian"`),
gated to the surrounding sidewalks, plus `meet_areas`. Support: navmesh over the foot-only edges +
open polygon; strolling/social there; no vehicles. (Ties into R1 — the park island must connect at its
gates.)

## R6 — City3D data-driven world *(3D render surface)*

`BuildingPlacer` is landed but purely geometric (hashed grey boxes). Make City3D consume our data:
- `--zones <zones.json>` → district tint/typed ground.
- `--buildings <buildings.json>` → **footprint-accurate massing** (`footprint` polygon + `levels|height_m`
  + `type`), not hashed boxes; typed models per `type` (mall/restaurant/office/residential/garage).
- `--pois <pois.json>` → `PoiPlacer`: typed props per `venue_type`/kind (mall, restaurant + tables,
  park + benches, lot + parked cars, transit).
- Suppress filler massing near our POIs/buildings; coordinate frames already match (SUMO metres).

## R7 — Turn the weave on + confirm width sourcing

`EnableWeave` on in the showcase scenario. Confirm the weave sources per-edge half-width from this net's
**baked sidewalk width** (our sidewalks carry real 2 m / 4 m widths, so the band should visibly vary),
and that server==IG holds on the composed net at demo density. This box is the visual proof of W1–W4.

**Note — not yet shown.** Our 2D replay ran the recorder at its default (`EnableWeave` **off**), so the
peds in it are the old pass-through low-power peds, *not* the weave. Turning `EnableWeave` on (and
confirming the band varies with our 2/4 m sidewalks) is the first thing that makes this box the visible
proof of the weave — do it early; it needs nothing further from us.

## R8 — Density coupling *(parked; list for completeness)*

The per-class (sidewalk vs crossing) density ceiling + `crossing_rate(ped_density, bias, class)` from
`SUBAREA-PED-PLANNING-RESPONSE.md §2`. Not needed for the demo (low crossing-rate by construction), but
this is the box to co-calibrate on when we pick density back up.

---

## What SumoData delivers to you (so you can build against it)

The full box (`demo_city/box/`, and first the `kernel/`): `net.xml` + `scenario.*` + `vType*` +
`manifest.json` + `pois.json` (`pois/v2`, extended + cross-refs) + `edge_fields.json` +
`zones.json` (`zones/v1`) + `buildings.json` (`buildings/v1`). All synthetic, path-scrubbed,
self-contained, regenerable byte-for-byte. Schemas are in `SUBAREA-DEMO-CITY-DESIGN.md §3`.

## Behavior status (for reference — what fires off which record)

| behavior | record it fires off | status | this box |
|---|---|---|---|
| dwell / pause | `dwell_spot.duration_profile`, `building_entrance.inside_dwell` | LANDED | v1 |
| meet & talk | `dwell_spot.meet_area`, `park.meet_areas` | LANDED | v1 |
| enter / exit buildings | `building_entrance` (+ P8-2 legitimacy) | LANDED | v1 |
| crossing crowds | crossings + classes (`edge_fields`) | LANDED | v1 |
| lateral weave (no pass-through) | baked sidewalk width; `EnableWeave` | **LANDED (W1–W4)** | **v1 (turn on)** |
| restaurant waiter | `venue.scenario_template` + `service_door` + `table_cluster` | POC → **R2** | v2 |
| walk-to-car & drive off | `parking_lot.boardable_car` + `exit_route` | POC → **R3** | v2 |
| hidden-garage birth/death | `parking_lot{hidden:true}` | **R4** | v2 |
| transit board/alight | `transit_stop.linked_vehicle` | DESIGNED | v3 |
