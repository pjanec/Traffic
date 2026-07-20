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

## R2 — Data-driven micro-scenario registry *(headline behavior: waiter)*

Replace the hardcoded `WaiterScenario`/`SceneGen` anchors with a registry keyed off our data. A
`venue` record carries `scenario_template` (e.g. `waiter_v1`), `service_door` (a `building_entrance`
id), and `table_cluster` (`[{id,pos,capacity}]`). The registry instantiates the scenario at the venue:
peds arrive via the door, sit at a table, are served, dwell, leave. IG-deterministic like the rest.

- **We provide:** the venue records + door + table geometry + cross-refs in `pois/v2`.
- **You build:** the template→instance registry + `waiter_v1`; confirm it reads the add-file POIs
  (`PedNetworkParser` `<poi>`/`<param>` path may need extending).

## R3 — Shared parked-car representation + productized drive-away *(headline behavior: shop & drive off)*

Turn the `LotCoupling` POC into a data-driven, both-directions coupling on our `parking_lot` records:
`polygon`, `lane_seam` (lot↔carriageway), `parked_cars` (static poses), `boardable_car`
(`{id, exit_route:[edge…]}`).

- **Arrive → park → alight → walk:** a car routes to the lot's parkingArea, parks (residency — already
  fixed), the ped alights at the seam and walks to the venue.
- **Walk → board → drive off:** a ped walks to `boardable_car`, boards, the car departs on `exit_route`.
- Needs the **shared parked-car representation** both sessions flagged (so the parked dressing, the
  boardable car, and the SUMO parkingArea occupant are one entity, IG-consistent).
- **We provide:** the lot polygon + seam + car poses + boardable-car id + exit route.
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
