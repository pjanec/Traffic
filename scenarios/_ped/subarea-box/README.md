# P8-1 handoff box — synthetic pedestrian sub-area

A **synthetic, geometry-free, shareable** SUMO sub-area produced by the SumoData
`experiments/subarea/preprocess.py` pipeline, built specifically to unblock the
SumoSharp pedestrian **P8 (sub-area integration)** work. It carries **no real road
geometry** — it is a deterministic `netgenerate` grid (seed 42) **with pedestrian
infrastructure guessed on** (sidewalks + crossings + walkingAreas) — so it can live in
the SumoSharp repo. All place labels have been scrubbed to neutral tokens.

This box is the stand-in for the "real cropped box `net.xml`" that P8-1 was blocked on:
you can run the navmesh bake and the fringe/appearance-legitimacy plumbing against it
today, then re-verify against a real crop later.

## Contents

| file | what it is |
|---|---|
| `net.xml` | the cropped box network (SUMO 1.20.0), **with sidewalks (`allow="pedestrian"` lanes), `crossing` edges, and `walkingArea`s**. SUMO network XY metres. |
| `scenario.sumocfg` | self-contained config, **relative paths** — `sumo -c scenario.sumocfg` just works. |
| `scenario.rou.xml` | calibrated car demand for the box (fringe→fringe + park-and-stay + depart-parked). |
| `scenario.add.xml` | parkingAreas (roadside sinks). |
| `vType.config.xml`, `vType_pedestrians.xml`, `vTypeDist.config.xml` | vType library (place tokens scrubbed; IDs kept internally consistent). |
| `manifest.json` | pipeline manifest — **the `subarea` block is the data contract for P8** (below). |

No FCD / trajectory files are shipped (kept small).

## What to consume (maps to `COORDINATION-pedestrian-x-subarea.md` §3)

- **P8-1 — navmesh bake:** feed `net.xml` to the SUMO-geometry bake (`SumoNavMesh`).
  The net has the walkable elements the bake reads (sidewalks / crossings /
  walkingAreas) and is frame-agnostic (SUMO XY metres). This is the crop to verify the
  bake against.

- **P8-2 — appearance-legitimacy fringe:** `manifest.subarea.fringe_edges` is the
  no-cheating FRINGE (edges cut by the crop — the only legitimate on-lane entry/exit),
  using the same definition as `audit_nocheat.py`. Each entry is tagged `car` / `ped`.
  **Filter to `ped: true`** to get the WALKABLE fringe = the pedestrian
  appearance-legitimacy boundary (the walkable-edge analogue of `RealismMask.MayPop`).
  In this box: `fringe_edge_count = 48`, `fringe_walkable_count = 48` (every fringe stub
  carries a sidewalk).

- **shared coordinate frame:** `manifest.subarea.box_bounds`
  (`xmin/ymin/xmax/ymax`, here `0,0 → 800,800` m) and `manifest.subarea.coordinate_frame`
  (`units: metres`, `net_offset`) define the frame — identical to `net.xml`, so peds and
  vehicles share one coordinate system.

- **P8-4 — calibrated density:** `manifest.density` is the calibrated level for the box
  (`knee_veh_lkm`, `served_percent`, `served_veh_lkm`) — the vehicle-side knee to anchor
  the ped-density knob / crossing-throughput guard against so crowds don't gridlock the
  calibrated cars.

## Run it

```bash
export SUMO_HOME=...            # your SUMO 1.20.0
sumo -c scenario.sumocfg --end 1000 --no-step-log true
```

## Provenance / caveats

- Deterministic: `netgenerate` seed 42, pipeline seed 42.
- Synthetic grid — topology is regular (6×6 with fringe attach-stubs), not a real city.
  Good enough to exercise the ped navmesh + the walkable fringe; **re-verify P8-1
  against a real crop before relying on absolute numbers.**
- `manifest.status` is `warn` only because the post-emit believability check saw
  achieved peak density above the sane band around the target — expected for a
  small dense synthetic grid, not an audit failure. The no-cheating audit
  (`manifest.verify.no_cheating`) is the relevant integrity signal.
