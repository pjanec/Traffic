# D1 experiment: land-use-weighted OD demand (zero external data)

**Goal**: show that per-edge origin/destination weights derived from synthetic land-use zones
(a stand-in for OSM land-use polygons) make `randomTrips.py --weights-prefix` produce a
believable home->work commute pattern, and quantify the effect against a uniform baseline.

## Method

- Network: `experiments/subarea/scratch/synth_macro.net.xml` (netgenerate 30x30 grid @300m,
  bounds 0,0..8700,8700; 3480 normal edges, 0 internal edges).
- Zones (script: `experiments/subarea/scratch/landuse_zones.py`): for each edge, take the mean
  of its shape-point coordinates as its center.
  - **COMMERCIAL/WORK core**: center inside the square `3600,3600 .. 5100,5100` -> 100 edges.
  - **RESIDENTIAL**: all other normal edges -> 3380 edges.
  - Area/edge-count baseline: commercial = 100/3480 = **2.87%** of edges (and, since this grid's
    edges are all equal length, also 2.87% of network length); residential = 97.13%.
- Weight files (script: `experiments/subarea/scratch/gen_weights.py`), edgedata format,
  `interval begin="0" end="3600"`:
  - `lu.src.xml` (trip origins): residential = 1.0, commercial = 0.1
  - `lu.dst.xml` (trip destinations): commercial = 1.0, residential = 0.1
- Demand generation, same seed/count settings for a fair A/B:
  - **WEIGHTED**: `randomTrips.py -n synth_macro.net.xml --weights-prefix lu -o lu.trips.xml -r lu.rou.xml --begin 0 --end 3600 --period 1.0 --validate --seed 42`
  - **UNIFORM**: identical command, omitting `--weights-prefix`, outputs `uni.*`
  - Both runs: `randomTrips.py` reported "Success." and duarouter validated with "Success."
    (no warnings in `lu.randomTrips.log` / `uni.randomTrips.log`).
  - Both produced **3600 trips / 3600 routed vehicles** (period 1.0 over the 3600 s window).
- Measurement (script: `experiments/subarea/scratch/measure.py`): for every vehicle in
  `lu.rou.xml` / `uni.rou.xml`, classify the **first edge** of `<route edges="...">` as the
  origin zone and the **last edge** as the destination zone, using the same zone test as above.

## Results

| metric                                      | area baseline | UNIFORM | WEIGHTED |
|----------------------------------------------|:---:|:---:|:---:|
| origin is residential                        | 97.13% | 96.75% | 99.56% |
| origin is commercial                         | 2.87%  | 3.25%  | 0.44%  |
| destination is commercial                    | 2.87%  | 2.81%  | 22.69% |
| destination is residential                   | 97.13% | 97.19% | 77.31% |

n = 3600 vehicles per set.

Sanity check: the UNIFORM set's empirical fractions (96.75% / 2.81%) track the geometric area
baseline (97.13% / 2.87%) almost exactly, confirming unweighted `randomTrips` samples edges
roughly uniformly (proportional to edge count/length) with no spatial bias, as expected.

**Key ratios:**
- **destination-commercial concentration**: 22.69% (weighted) / 2.81% (uniform) = **8.09x**
- **origin-commercial suppression** (symmetric view of the origin effect — more informative
  than the residential fraction, which is ceiling-limited since it starts at ~97%):
  3.25% (uniform) / 0.44% (weighted) = **7.31x** fewer commercial-origin trips
- origin-residential fraction moved 96.75% -> 99.56% (ratio 1.03x) — this ratio is naturally
  compressed because residential origins are already the overwhelming majority under uniform
  sampling (only 2.87% of edges are commercial), so there is little headroom left to climb
  toward 100%. The suppression ratio above is the correct measure of the same effect.

## Pass/fail on the success condition

**PASS.** The required check — weighted commercial-destination fraction / uniform
commercial-destination fraction "clearly > 2x" — comes out to **8.09x** (22.69% vs 2.81%),
well above the 2x bar. The symmetric origin-side effect is confirmed via commercial-origin
suppression at **7.31x** (uniform 3.25% -> weighted 0.44%), i.e., trips overwhelmingly start
outside the commercial core and end inside it under the weighted demand, exactly the
home -> work commute pattern the experiment set out to produce — achieved with zero external
demographic data, purely from synthetic per-edge src/dst weight files.

## Automation path (no manual work, real net)

On a real network the same pipeline needs no manual edge-picking: pull OSM land-use polygons
(`landuse=residential/commercial/retail/industrial`, or `building=*` tags) for the area of
interest, convert them to SUMO polygons with `polyconvert` (which already tags each polygon
with its OSM `type`), then assign each polygon to its nearest edge(s) — e.g. via
`sumolib.net.getNeighboringEdges()` / a spatial index keyed on polygon centroid — accumulating a
per-edge weight bucket by summing polygon area (or building floor area, if available) per
land-use class touching that edge. Those per-edge sums become exactly the
`<edge id=".." value=".."/>` rows in `W.src.xml` (residential-heavy weight) and `W.dst.xml`
(commercial/retail/industrial-heavy weight) that `randomTrips.py --weights-prefix W` consumes —
the whole derivation is polygon-geometry arithmetic, with no demographic survey data required.
The `randomTrips` knobs that matter for realism: `--weights-prefix` itself (the src/dst/via
split demonstrated here), `--fringe-factor` to inflate weights on network-boundary edges so
through-traffic isn't starved relative to internal commute trips, `--vclass` to restrict
generated trips to a vehicle class matching the land-use-appropriate traffic mix (e.g. trucks
weighted toward industrial), and `--period` (or `--insertion-rate`) to control overall demand
density independent of the spatial weighting.

## Files

- `experiments/subarea/scratch/landuse_zones.py` — zone classification (commercial core square
  vs residential) shared by weight generation and measurement.
- `experiments/subarea/scratch/gen_weights.py` — writes `lu.src.xml` / `lu.dst.xml`.
- `experiments/subarea/scratch/measure.py` — classifies route origins/destinations and prints
  the fractions/ratios above.
- `experiments/subarea/scratch/lu.{trips,rou}.xml`, `uni.{trips,rou}.xml` — generated demand
  (gitignored, regenerable from the scripts above).
- `experiments/subarea/scratch/lu.randomTrips.log`, `uni.randomTrips.log` — randomTrips/duarouter
  stderr+stdout for both runs (both "Success.").
