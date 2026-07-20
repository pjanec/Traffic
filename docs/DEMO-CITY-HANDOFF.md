# SumoData → SumoSharp handoff: the synthetic demo-city (Prototype E substrate)

A fully **synthetic** ~5×5 km composed city + a 1 km feature kernel, for demonstrating and testing every
vehicle + pedestrian feature and their interactions, in both `sim_viz` 2D and City3D 3D. No real
geometry, no place tokens — safe to keep in the SumoSharp repo.

## Contents
- `box/` — the full 5×5 km city (net + calibrated demand + companion data).
- `kernel/` — a ~1 km feature-representative slice: one of every junction type (traffic_light,
  roundabout, priority, right_before_left, allway_stop, zipper) + multi-lane + a ped-only park. **This is
  the shareable navmesh stress-test** (bakes to `components=1` — the P8-1 fragmentation repro that
  *doesn't* fragment).
- `SUMOSHARP-DEMO-CITY-REQUIREMENTS.md` — **read this first.** The prioritized asks (R1–R8): what to
  build to consume the box, what SumoData provides vs what you build, and the concrete bake findings.
- `SUBAREA-DEMO-CITY-DESIGN.md` — the design reference: district layout + the exact companion-data
  schemas (`pois/v2`, `zones/v1`, `buildings/v1`) with the cross-reference graph.

## Companion data (in `box/` and `kernel/`)
- `net.xml`, `scenario.sumocfg`, `scenario.rou.xml`, `scenario.add.xml`, `vType*.xml` — SUMO scenario.
- `pois.json` (`pois/v2`) — 458 POIs incl. the new `parking_lot` / `park` kinds + set-piece cross-refs
  (mall+lot+boardable_car, restaurants+tables+doors, park+meet_areas, hidden garages, quiet-area lots).
- `zones.json` (`zones/v1`) — 6 district polygons.
- `buildings.json` (`buildings/v1`) — 28 footprints with `levels`/`height_m`/`type` (City3D massing).
- `edge_fields.json` — per-edge sidewalk width / keep-side / corridor / aoi / crossing class.
- `manifest.json` — the sub-area contract (bbox / fringe / frame) + density block.

## First contact — the one known blocker
Your recorder throws `FormatException: unknown POI kind` on `parking_lot` / `park`. Extend
`PedPoiReader.cs` / `PedNetworkParser` to parse them (R2/R5) before loading `box/pois.json` unfiltered.
Everything else (net, cars, v1 POIs) loads today.

## Quick start
```
# vanilla SUMO (cars): loads clean, drive-away/garage cycles run
sumo -c box/scenario.sumocfg --end 800 --no-step-log true

# navmesh bake (your recorder): kernel -> components=1; full box -> 1 main + residuals (R1)
dotnet run --project src/Sim.Viz -c Release -- \
  --ped-subarea-fcd ped.fcd.xml --reachable-filter --dial 0.3 --seconds 200 --box <path>/box
```
