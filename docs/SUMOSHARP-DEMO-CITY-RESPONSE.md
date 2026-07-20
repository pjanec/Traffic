# SumoSharp response — the composed demo-city (Prototype E)

Reply to `SUMOSHARP-DEMO-CITY-REQUIREMENTS.md` / `SUBAREA-DEMO-CITY-DESIGN.md` (both committed here). Maps the
R1–R8 asks onto current SumoSharp state, flags what is already landed vs blocked on artifacts, and names the
gating next action. This is the Prototype-E substrate the weave work (PED-REALISM-1) was building toward.

## Artifact status (what's in this repo right now)
- **Not yet here:** `demo_city/kernel/` and `demo_city/box/` (net + `scenario.*` + `manifest.json` + `pois/v2`
  + `zones.json` + `buildings.json` + `edge_fields.json`). The relay says Stage-1 kernel exists on the
  SumoData side (baked to components=1) and Stage 2/3 are building. **I bake the moment the kernel lands here.**
- **Already here (reusable):** `EnableWeave` (W1–W4, done), the P8-1c navmesh bridge + reachability filter,
  `WaiterScenario` + `LotCoupling` POCs, `SubareaFcdRecorder` (reports `ConnectedComponents` + `UnreachableSkips`
  — R1's exact acceptance signal), the `DEMO-CITY3D-*` doc set (R6 surface), and existing subarea scenarios.

## R-by-R readiness

| R | ask | SumoSharp state | blocker / next action |
|---|---|---|---|
| **R1** | navmesh bakes composed geometry to few components, unreachable≈0 | **Largely landed.** P8-1c already stitches sidewalk↔sidewalk continuations from the net's spine connectivity + a dominant-component reachability filter (witness 83→1). The recorder reports `components`/`unreachableSkips` directly. | **Ready — needs the kernel file.** Drop `kernel/net.xml` here → I bake + report the two numbers. If it fragments on a NEW shape (roundabout walkingArea, 3+-poly corner), that's the next P8-1 pass, against your shareable repro. **This gates everything; it's first.** |
| **R2** | data-driven micro-scenario registry (`waiter_v1`) | POC exists (`WaiterScenario`), but hardcoded anchors. | Build the template→instance registry keyed off `venue.scenario_template`/`service_door`/`table_cluster`; **extend `PedNetworkParser` to read add-file `<poi>`/`<param>`** (confirmed: it does not read POIs today). Needs `pois/v2`. |
| **R3** | shared parked-car rep + productized drive-away | POC exists (`LotCoupling`, both directions in prototype). | Productize on `parking_lot` records; the **shared parked-car representation** (one entity = dressing + boardable + parkingArea occupant) + its IG-determinism contract. Needs `pois/v2` lot records. |
| **R4** | hidden-garage occluded birth/death | Residency correct; garage = `parkingArea` on off-road access edge. | Mostly a **City3D occlusion** ask (R6) + treating the garage access as a legitimate hidden sink/source. Needs `parking_lot{hidden:true}` + `buildings.json`. |
| **R5** | park ped-only region | Navmesh handles sidewalk/crossing/walkingArea/plaza polygons. | Confirm `PedNetworkParser` ingests `allow="pedestrian"` foot-only edges + gates the park island to sidewalks (ties into R1). Needs the kernel's park. |
| **R6** | City3D data-driven world (zones/buildings/pois) | `BuildingPlacer` landed (geometric); `DEMO-CITY3D-*` docs exist. | `--zones`/`--buildings`/`--pois` consumers + typed massing/props; filler suppression near POIs. Needs `zones.json`/`buildings.json`/`pois/v2`. |
| **R7** | **turn the weave on + confirm width sourcing** | **DONE (W1–W4).** `EnableWeave` flag; per-vertex half-width sourced from `BakedPolygon.HalfWidth = lane.Width/2` (W2); server==IG bit-exact over the wire (W3); survives promote/demote (W4). | **Ready — needs the showcase net.** With your 2 m/4 m sidewalks the band should visibly vary. I flip `EnableWeave` on in the showcase config + produce the visual proof + re-confirm `components`/server==IG at demo density. |
| **R8** | density coupling (parked) | Density ceiling measured (~0.3/m² pure-weave); `crossing_rate` owed. | Parked, as you note — co-calibrate on this box when density resumes. |

## Sequencing (my side)
1. **R1 kernel bake** — the instant `kernel/` lands. One command, two numbers back. Gates the rest.
2. **R7 weave-on** — unblocked in parallel the moment a showcase net exists (kernel or box); it's the visual
   payoff of W1–W4 and needs no new engine code, just the scenario config + a render pass.
3. **R2 / R3** — the two headline v2 behaviors; both have POCs to productize, both need `pois/v2`.
4. **R5 / R6 / R4** — park ped-only + City3D data-driven + garage occlusion, as the box + companion data fill in.

## What I need from SumoData (in order)
1. `kernel/net.xml` (+ its `scenario.*`) committed here → unblocks R1 + a first R7 visual.
2. `pois/v2` schema examples (a venue with `scenario_template`/`service_door`/`table_cluster`; a `parking_lot`
   with `lane_seam`/`boardable_car`) → unblocks R2/R3 design.
3. The full `box/` + `zones.json`/`buildings.json` → R6 + R4 + the composed acceptance demo (Prototype E).

**Standing invariant:** everything above keeps server==IG and the offline gate green; `EnableWeave` stays
default-off until the showcase scenario opts in, so nothing regresses while the box fills in.
