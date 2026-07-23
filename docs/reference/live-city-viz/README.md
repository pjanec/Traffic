# `live-city-viz` — vendored reference (read-only)

A **reference** 2D live-city visualization from the SumoData side, kept here so we can port its richer world
visuals (POIs, sidewalks, lane markings, crosswalk zebras, zones, data-driven buildings + entrances) into our
own viewers (Raylib 2D and Godot City3D). **This is reference material, not part of our build or tests** — do
not add it to a `.csproj`/solution; it never runs in CI.

## Contents
- `DESIGN-live-city-2d-viz.md` — the layer spec + data contract. **Read this first.** It documents all 14
  render layers (§2), the `REPLAY_DATA` schema (§3), the source data (§6), and — crucially — **§7 "Reproducing
  in 3D"**, which maps each 2D layer to a 3D concept. This is the blueprint for the visual-enrichment work.
- `renderer/sim_viz.py` — the Python payload builder (SUMO net + FCD + pois/zones/buildings → `REPLAY_DATA`).
- `renderer/templates/template.html` + `template.js` — the vanilla-JS Canvas 2D renderer (how each layer is
  actually drawn; the port source for the 2D visuals).
- `sample.html` — a **rendered** self-contained sample (open in any browser) showing the target look.
- `manifest.viz.json` — the viz package's manifest (the one `sample_data/` file that differs from ours).
- `run.sh` — the reference regen command (Linux; needs SUMO).

## The sample data is ALREADY in our repo
The package shipped a `sample_data/` dir. Every file in it is **byte-identical** to
`scenarios/_ped/demo_city/box/` — `net.xml`, `pois.json`, `buildings.json`, `zones.json`,
`edge_fields.json`, `scenario.rou.xml`, `scenario.sumocfg` — **except** `manifest.json` (kept here as
`manifest.viz.json`). So it is **not re-committed**; the live "box" dataset our viewers already load is the
same world the reference renders. In other words: **we already have all the POI / building / zone / edge-field
data our viewers need — it just isn't rendered yet.**

## How this maps to the requested visual enrichments
| Want | Reference layer (DESIGN §2) | Data (in `demo_city/box`) |
|---|---|---|
| Pedestrian **sidewalks** (distinct shade) | 2b (`lane.ped===true`) | `net.xml` ped lanes |
| Driving-lane **dashed separators** | 3 (derived from adjacent same-edge lanes) | `net.xml` lanes |
| **Crosswalk zebras** | 9 (`:node_cN` crossing lanes) | `net.xml` crossings |
| **Traffic lights** (already in City3D) | 10 (`TrafficLightPlacer`) | `net.xml` tlLogic |
| **POIs** (venues, transit, dwell, parking, **entrances**) | 11 (glyph by kind) | `pois.json` |
| **Zones / districts** | 4 (translucent tint by type) | `zones.json` |
| **Buildings** (data-driven massing) | 7 (footprint by type; §7 → extruded) | `buildings.json` |
| Building **entry/exit doors** | POI kind `building_entrance` (layer 11) | `pois.json` |

See `docs/LIVE-CITY-VISUALS-NOTES.md` for the port plan into our 2D + 3D viewers.
