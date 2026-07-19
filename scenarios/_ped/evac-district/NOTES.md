# evac-district — synthetic pedestrian evac network (P5-PRE)

A synthetic pedestrian walkable district for the evac unification (`docs/PEDESTRIAN-TASKS.md`
P5-PRE / P5-1(B)). It exists because no real pedestrian net was available for evac development;
this one is committed and verified so the evac routed foot-exodus (P5-B) has real sidewalks/
crossings to route over.

## What it contains
- A **3×3 grid** of `priority` junctions, 100 m spacing (a 200 m × 200 m district).
- **Sidewalks** on every street (`--sidewalks.guess`) + **pedestrian crossings** at every
  junction (`--crossings.guess`) + **walkingareas** (44 crossing/walkingarea elements total).
- **Incident epicentre:** the grid centre `n11` at world **(100, 100)**.
- **Safe zones (flee destinations):** the four corner junctions —
  `n00` (0, 0), `n20` (200, 0), `n02` (0, 200), `n22` (200, 200). A ped fleeing the centre
  routes to the nearest corner **along the sidewalk grid** (bending around the blocks), not
  radially through the buildings — verified in `EvacDistrictNetTests`.

## Regeneration (authoring-side; SUMO required, never in the test loop)
```
netconvert --node-files=nodes.nod.xml --edge-files=edges.edg.xml \
  --sidewalks.guess --crossings.guess --tls.guess --output-file=net.net.xml
```
Generated with Eclipse SUMO 1.20.0 (matches `SUMO_VERSION`). The committed `net.net.xml` is the
hermetic test input; SUMO is not needed to run `dotnet test`.
