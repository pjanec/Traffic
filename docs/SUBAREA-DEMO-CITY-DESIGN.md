# Synthetic demo-city (~5×5 km) — design & build spec

A **purpose-built, deterministic, synthetic** SUMO city that is the single reusable substrate for
demonstrating and testing **every vehicle + pedestrian feature and their interactions**, in both
render surfaces (our `sim_viz` 2D replay and SumoSharp's City3D procedural 3D world). It is the
production successor to `experiments/subarea/synthetic_demo/` (which was a data-contract *catalog* on
a bare 5×5 grid — one marker of each POI kind, no composed scenarios). This is the composed **V2**
layout: zoned districts, real junction/road variety, and the functional set-piece spots
(mall + lot + drive-away, restaurant + tables, hidden garages, park, quiet-area lots) wired together
by cross-referenced companion data.

**Synthetic ⇒ shareable.** No real geometry, no place tokens; the whole box (net + companion data)
may cross to the SumoSharp repo as the Prototype-E acceptance mesh and the navmesh stress-test.

**Design goal — "build the navmesh once."** The net deliberately contains the geometry that fragments
the current baker (roundabouts, multi-lane junctions, irregular arterials — the P8-1 failure shapes).
We validate navmesh connectivity on a small **feature kernel first**, coordinate one baker-robustness
pass with the ped session, then scale — so the full city bakes to *few* components and stays a stable
substrate we do not re-cut per demo.

---

## 0. Directory & artifacts

```
experiments/subarea/demo_city/
  citygen.py        # procedural net generator -> plain XML (nod/edg/con/tll + roundabout/zone patches)
  compose.py        # companion-data composer: zones, buildings, extended POIs + cross-refs, edge_fields
  build.py          # orchestrator: citygen -> netconvert(+ped infra) -> preprocess -> compose -> assemble box
  kernel/           # Stage-1 feature-representative slice (committed; navmesh-validated)
  box/              # the full 5x5 km committed box (Stage 2+)
  README.md
```

Committed `box/` (and `kernel/`) contents — all path-scrubbed, relative, self-contained:

| file | schema | what |
|---|---|---|
| `net.xml` | — | the city net (SUMO XY metres, origin 0,0 → ~5000,5000) |
| `scenario.sumocfg` | — | self-contained, relative paths |
| `scenario.rou.xml` / `scenario.add.xml` | — | calibrated car demand + parkingAreas (incl. off-street lots + hidden garages) |
| `vType*.xml` | — | vType library (cars + peds) |
| `manifest.json` | subarea contract | bbox / fringe / frame + density block |
| `pois.json` | **`pois/v2`** | extended POIs + cross-references (§3) |
| `edge_fields.json` | `edge_fields/v1(+)` | per-edge broadcast-once companion (existing) |
| `zones.json` | **`zones/v1`** (NEW) | district polygons + type (§4) |
| `buildings.json` | **`buildings/v1`** (NEW) | footprints + height/levels + type (§4) |

FCD/trajectories are never committed (size); regenerated on demand for replays.

---

## 1. Road network — hierarchy, junctions, lanes

A believable city is a **hierarchy**, not a uniform grid. `citygen.py` composes:

**Road classes (the multi-lane requirement):**
- **Arterials** — 3 lanes/dir, ~50–70 km/h, the district-connecting skeleton (ring + 2 cross axes).
- **Collectors** — 2 lanes/dir, ~50 km/h, district spines.
- **Local/residential** — 1 lane/dir, ~30 km/h.
- **One-way pairs** — a couple of multi-lane one-way streets in the downtown core.
- **Lane-count changes** — turn-lane additions / drops at major junctions (exercises lane-change +
  junction lane assignment + zipper).

**Junction types (the variety requirement) — every SUMO node type present:**
- `traffic_light` — downtown grid + major arterial crossings (→ signalized ped crossings, TL platoons).
- **roundabout** — 2–3 on the arterial ring (explicit `<roundabout>` rings in plain XML; netconvert
  recognises them). Exercises roundabout car behaviour + ped crossings on approaches.
- `priority` — uncontrolled major/minor (the default; most collector/local junctions).
- `right_before_left` — uncontrolled equal (residential; → discouraged crossings).
- `allway_stop` — at least one (US-style all-way stop).
- `zipper` — at least one merge (lane drop on an arterial on-ramp-like merge).
- `traffic_light` with `left_before_right`/actuated variants where cheap.

**Pedestrian infrastructure:**
- Sidewalks on every urban road (`--sidewalks.guess`, `--default.sidewalk-width 2.0`, wider 4 m on
  arterials/downtown frontage → real per-edge width variation the weave consumes).
- Crossings: all three classes present (signalized at TL, unsignalized at priority, discouraged at
  right_before_left/mid-block) + a couple of explicit mid-block crossings.
- **Park foot-path graph** — pedestrian-only edges (`allow="pedestrian"`, `disallow` all vehicle
  classes) forming an internal path network inside the park polygon, connected to the surrounding
  sidewalks at gate nodes.

**Fringe legitimacy** — arterials terminate in fringe stubs (`--grid.attach-length` analogue);
vehicle/ped births & deaths happen at the fringe or at off-road parking/garages, never mid-road
(the no-cheating audit still applies).

---

## 2. Districts (5×5 km ≈ 25 km²)

Laid out on the 5000×5000 m frame; sizes approximate.

| district | ~area | net character | headline features |
|---|---|---|---|
| **Downtown core** (centre) | 1.5×1.5 km | dense regular grid, TL, 2 lanes/dir, one-way pairs, on-street parking, wide sidewalks | crossing crowds, TL ped platoons, offices/venues (enter-exit), high ped density |
| **Retail / mall** (NE) | ~1 km | multi-lane approach arterial + roundabout entry | **shopping mall** (big venue) + **off-street lot polygon** in front + **hidden underground garage** (occluded birth/death); the drive-away set-piece |
| **Dining quarter / plaza** (near downtown) | ~0.6 km | low-speed shared-space streets, discouraged crossings | **restaurants** (tables + service doors, waiter scenario), a **plaza** for meet & talk |
| **Residential** (SW) | 1.5×1.5 km | quiet grid, `priority` + `right_before_left`, 1 lane/dir | buildings + entrances (enter/exit), **quiet-area parking lots** + local demand so streets aren't dead |
| **Park / green** (a block) | ~0.6×0.6 km | **ped-only foot-path graph** inside a green polygon, no cars, gates to sidewalks | strolling, **meet areas** (social), benches (dwell), a ped-only navmesh region |
| **Arterial ring + cross axes** | — | 3-lane arterials, roundabouts, `zipper` merge, `allway_stop` | through traffic, roundabout behaviour, lane-change, junction variety |

The zoning is explicit (`zones.json`), not derived from betweenness — it drives POI placement, car-demand
weighting, and 3D massing.

---

## 3. Companion data — `pois/v2` with cross-references

Extends today's `pois.json` (kinds building_entrance / venue / dwell_spot / transit_stop /
parking_access; fields id/kind/pos/edge/weight/facing/capacity/land_use/lateral_anchor). **Additive,
back-compatible; a `schema:"pois/v2"` tag gates the new fields.** Records now carry **ids that link
records** — the coherence the catalog box lacked.

- **`venue`** += `venue_type ∈ {mall,restaurant,cafe,office,retail}`, `scenario_template` (e.g.
  `waiter_v1`), `service_door` (a `building_entrance` id), `table_cluster` (`[{id,pos,capacity}]`),
  `building` (a `buildings.json` id).
- **`parking_lot`** (NEW kind) = `polygon` ([x,y]…), `capacity`, `lane_seam` (`[{lane,pos}]` — where the
  lot meets the carriageway), `parked_cars` (`[{pos,angle}]` static dressing), `boardable_car`
  (`{id, exit_route:[edge…]}` — the car a ped walks to and drives off in), `hidden` (bool). A lot with
  `hidden:true` **is a garage**: modelled as a SUMO `parkingArea` on a short off-road access edge inside
  a building footprint, so cars route in and legitimately disappear (residency), and births originate
  there — occluded in 3D (the "underground garage" realism requirement).
- **`park`** (NEW kind) = `polygon`, `path_edges` (ped-only edge ids), `meet_areas`
  (`[{id,pos|polygon,group_size,talk_duration}]`), `dwell_points`.
- **`building_entrance`** += `inside_dwell` (duration profile), `legitimate_sink` (P8-2 gate), `building`.
- **`dwell_spot`** += `meet_area`, `group_size`, `talk_duration`, `duration_profile`.
- **`transit_stop`** (v3) += `headway`, `linked_vehicle`.

**Cross-reference graph** (the set-piece wiring): `venue → service_door → table_cluster`;
`parking_lot → parked_cars → boardable_car`; `park → path_edges → meet_areas`;
`building_entrance ↔ building`. Every referenced id resolves within the box.

`zones.json` (`zones/v1`): `[{id, type, polygon, ...}]` per district.
`buildings.json` (`buildings/v1`): `[{id, footprint:[x,y]…, levels|height_m, type, zone}]` — real massing
for City3D (not typed grey boxes); `type` distinguishes mall/restaurant/office/residential/garage.

---

## 4. Car demand (believable, non-cheating, animating the quiet zones)

`compose.py` writes weighted demand (via `deduce_weights.py` + hand-authored trip classes):
- **Through traffic** — fringe↔fringe on arterials/ring.
- **Home→work** — residential → downtown (AM), reverse (PM) if we want a time profile.
- **Shopping trip** — →mall, **park in the lot**, ped alights, walks to mall entrance, dwells, returns,
  **drives away** (the drive-away coupling; car uses `boardable_car.exit_route`).
- **Dining trip** — →dining quarter, park, walk to restaurant, waiter scenario, leave.
- **Garage births/deaths** — a fraction of demand originates/terminates at hidden garages (legitimate
  occluded birth/death).
- **Quiet-area animation** — residential-internal trips + parking churn at quiet-area lots so those
  streets read as alive, not dead.

Density: a **moderate, believable demo density**, not a full 5×5 km knee sweep (too heavy and not the
point). `build.py` runs `preprocess.py` at a fixed `--percent` chosen per district-mix; per-district knee
calibration is available later if we want the auto-density dial on this box.

---

## 5. Rendering — both surfaces off the one dataset

- **`sim_viz` 2D (ours):** extend the existing POI+crossings overlay with **static polygon layers** —
  district tints from `zones.json`, building footprints from `buildings.json`, lot polygons + park
  polygon + meet areas from `pois.json`. `template.js` gets `drawZone`/`drawFootprint`/`drawPolygon`
  passes under the agents. Additive; payload byte-identical without the new files.
- **City3D 3D (theirs):** `PoiPlacer` + `ZonePlacer` + footprint-accurate `BuildingPlacer` massing from
  `zones.json` + `buildings.json` + `pois.json` v2 (`--pois`/`--zones`/`--buildings` args). Typed models
  per `venue_type`/building `type`; filler suppressed near our POIs. **Ped-side work — see the
  requirements doc.**

---

## 6. Build stages (validate navmesh before scaling)

1. **Stage 1 — feature kernel** (`kernel/`, ~1×1 km): one of *every* junction type + a roundabout +
   a multi-lane arterial + a park (ped-only) + one built district block + all crossing classes.
   **Acceptance: bakes to ≤ few navmesh components, unreachable O/D ≈ 0** (run the SumoSharp recorder).
   If it fragments → that is the navmesh-robustness ask to the ped session, on a *shareable synthetic*
   repro, resolved once. Nothing scales until the kernel is green.
2. **Stage 2 — full 5×5 km** — compose all districts + the arterial ring; re-validate navmesh
   components + the no-cheating car audit at the full scale.
3. **Stage 3 — V2 companion data** — `zones.json`, `buildings.json`, extended `pois/v2` with the
   cross-reference graph + the set-piece spots + the drive-away/waiter demand classes.
4. **Stage 4 — rendering** — the `sim_viz` polygon layers (ours); hand the City3D requirements + the box
   to the ped session for the 3D consumer.

---

## 7. Ownership

- **SumoData (ours):** `citygen.py` + `compose.py` + `build.py`; the net; the companion schema
  (`pois/v2`, `zones/v1`, `buildings/v1`, `edge_fields`); the demand incl. drive-away/garage/quiet-area
  classes; the `sim_viz` polygon layers; navmesh-validating the kernel + full box; keeping it synthetic.
- **SumoSharp (theirs):** navmesh baker robustness on this net; the data-driven micro-scenario registry
  (waiter) + productized drive-away + shared parked-car representation; hidden-garage occlusion; the park
  ped-only region; City3D `PoiPlacer`/`ZonePlacer`/massing; `EnableWeave` in the showcase scenario. See
  `SUMOSHARP-DEMO-CITY-REQUIREMENTS.md`.
