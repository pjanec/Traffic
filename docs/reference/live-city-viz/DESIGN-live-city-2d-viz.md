# Live-City 2D Visualization — Design & Data Contract

A reference spec for the self-contained HTML traffic-replay used in this project (the "live-city" 2D
player), so an independent session can **reproduce a similar 2D (or 3D) visualization** from the same data.
It documents (1) every rendered layer — what it looks like and what data feeds it, (2) the exact
`REPLAY_DATA` JSON schema the player consumes, (3) the upstream source data needed to produce it, and (4)
guidance for a 3D re-implementation.

Reference implementation in this repo: `experiments/subarea/sim_viz.py` (payload builder) +
`experiments/subarea/templates/template.html` + `templates/template.js` (renderer). The sample package
ships all three plus a ready dataset and a generated HTML.

---

## 1. Architecture

**Offline, single-file, no server.** The pipeline is:

```
SUMO net.xml ┐
FCD (veh+ped)├─►  sim_viz.py  ─►  REPLAY_DATA (JSON)  ─┐
pois/zones/  │    (payload builder)                    ├─► one self-contained .html
buildings    ┘                    template.html + template.js (renderer, inlined verbatim) ┘
```

`sim_viz.py` converts inputs into a single `REPLAY_DATA` JSON object and injects it — together with the
verbatim `template.js` — into `template.html` at three tokens (`__SCENARIO_NAME__`, `/*REPLAY_DATA*/`,
`/*TEMPLATE_JS*/`). The result is one HTML file that runs anywhere in a browser (Canvas 2D, vanilla JS, no
libraries, no network). Everything below is rendered client-side from `REPLAY_DATA`.

**Coordinate frame.** All coordinates are raw **SUMO world XY in metres**, rounded to 2 dp; **Y is up**
(north). The renderer flips Y for canvas (`screenY = -worldY*scale + offsetY`) in `worldToScreen()` — the
data is never pre-flipped. Angles are **naviDegrees**: 0 = +Y/north, increasing clockwise (this is SUMO
FCD's own `angle` attribute passed through unchanged).

**Multi-scene.** `REPLAY_DATA = { scenes: [ SCENE, … ] }`; a scene selector switches between them. A
single-scenario export is a one-element array.

---

## 2. Rendered layers — appearance + data source

Drawn strictly **back-to-front** every frame (`render()` in template.js). Each layer is null/empty-guarded,
so a scene that omits a layer's data simply doesn't draw it.

| # | Layer | Looks like | Data source (scene key) |
|---|---|---|---|
| 0 | Background | flat dark slate `#1b1e26` | — |
| 1 | Junctions | filled grey polygons `#33363f` | `network.junctions[].shape` |
| 2 | Lanes (carriageway) | dark asphalt band `#4a4d55`, width = true lane width (px floor ~2.5) | `network.lanes[]` (`shape`,`width`) |
| 2b| Sidewalks / footpaths | lighter "concrete" band `#8a8f99` | `network.lanes[].ped === true` |
| 3 | Lane markings | dashed white line between same-edge adjacent lanes | derived from `network.lanes` (left edge of any lane with a left neighbour) |
| 4 | Zone tints | translucent district fill, colour by type (downtown grey, retail amber, dining pink, residential blue, park green, arterial faint grey) | `zones[]` (`type`,`polygon`) |
| 5 | Parks | green fill `rgba(34,197,94,.30)` + green outline | `parks[].polygon` |
| 6 | Parking lots | grey fill `rgba(148,163,184,.35)` + light outline | `parkingLots[].polygon` |
| 7 | Buildings | footprint fill by type (mall amber, office blue, residential teal, restaurant red, garage grey) + dark edge | `buildings[]` (`type`,`polygon`) |
| 8 | Meet areas | small dashed yellow ring + centre dot, sized by group size | `parks[].meetAreas[]` (`x`,`y`,`groupSize`) |
| 9 | Crossings (zebra) | subtle light base band + white perpendicular stripes | detected in renderer: `network.lanes` whose `edgeId` matches `:<node>_c<idx>` |
| 10| TLS signal heads | small red/yellow/green dots at stop lines, colour = live phase state | `network.signals[]` + `network.tls[]` (see §note) |
| 11| POIs ("places") | glyph by kind (see below), world-scaled with px floor/ceiling; text label when zoomed in | `pois[]` (`kind`,`x`,`y`,`label`) |
| 12| Discs (pedestrians / crowd) | filled circles coloured by `kind` (ped = purple `#c084fc`); optional oriented shapes | `frames[].d[]` |
| 13| Vehicles | oriented rectangles (front-anchored), passenger-blue `#4f8ef7`; optional speed heatmap / fear ramp | `frames[].v[]` + `vdim` |

**POI glyphs** (`kind` → glyph, colour): `venue` → amber square; `building_entrance` → green triangle;
`dwell_spot` → pink circle; `transit_stop` → cyan diamond; `parking_access` → light-blue "P" tile. Unknown
kinds → grey circle. Labels (the POI `id`) show for venue/entrance/dwell/transit when zoomed in.

**Vehicles.** Each `frames[].v[i]` is `[x, y, angleDeg]` (optionally a 4th `fear` element). `(x,y)` is the
**front-centre**; the box extends *back* by `vdim[0]` (length) along the heading, width `vdim[1]`. Colour is
passenger-blue, or a **speed heatmap** (cold→hot, HUD toggle; speed derived from displacement) or a **fear
ramp** (calm-blue→panic-red) when a 4th element is present (panic-evac scenes). Heading is **kinematically
smoothed** — see §4.

**Discs.** Each `frames[].d[i]` is `[x, y, radius, kind]` (pedestrian kind = 2, purple). A disc may be
extended to an **oriented shape** for mixed-traffic vehicle agents: `[x, y, radius, kind, headingDeg, shape,
halfLen, halfWid]` where `shape` 0 = rectangle (car/bus, true aspect), 1 = slim hexagon (motorcycle).

**Note on TLS.** `sim_viz.py` currently stubs `network.tls=[]` and `network.signals=[]` (crossings are still
drawn, detected by edge-id). The renderer fully supports live signals when populated: `tls[]` =
`{id, offset, phases:[{duration, state}]}` (SUMO `tlLogic`), `signals[]` = `{tl, linkIndex, x, y}`; the
player computes the phase at time *t* (`tlLinkState`, mirrors SUMO `TrafficLightState.GetLinkState`) and
colours each head. The C# Sim.Viz tool populates these; a 3D re-impl that wants live signals should emit
them the same way.

---

## 3. `REPLAY_DATA` schema (what the renderer consumes)

CamelCase keys. Optional keys are **omitted entirely** when their source is absent (keeps outputs
byte-identical across feature gates).

```jsonc
{
  "scenes": [{
    "name": "string",
    "desc": "string",
    "view": [minX, minY, maxX, maxY],          // world bbox for initial camera fit
    "dt":   1.0,                                 // seconds between frames
    "vdim": [length, width],                     // shared vehicle box dims (metres)

    "network": {                                 // null for pure open-space crowd scenes
      "lanes": [ { "id":"…", "edgeId":"…", "index":0,
                   "width":3.2, "shape":[x0,y0,x1,y1,…],
                   "ped": true } ],              // "ped" present only on pedestrian-only lanes
      "junctions": [ { "id":"…", "shape":[x0,y0,…] } ],
      "tls":     [ { "id":"…", "offset":0, "phases":[ {"duration":37,"state":"GrGr…"} ] } ],
      "signals": [ { "tl":"…", "linkIndex":0, "x":…, "y":… } ]
    },

    "frames": [                                  // one per timestep, ascending
      { "v": [ [x,y,angleDeg] | [x,y,angleDeg,fear] | null, … ],   // FIXED SLOTS
        "d": [ [x,y,radius,kind] | [x,y,r,kind,headingDeg,shape,halfLen,halfWid] | null, … ] }
    ],

    // OPTIONAL static layers — key present only when supplied:
    "pois":        [ { "kind":"venue", "x":…, "y":…, "label":"…" } ],
    "zones":       [ { "id":"…", "type":"downtown", "polygon":[x0,y0,…] } ],
    "buildings":   [ { "id":"…", "type":"office",   "polygon":[x0,y0,…] } ],
    "parkingLots": [ { "id":"…", "polygon":[x0,y0,…] } ],
    "parks":       [ { "id":"…", "polygon":[x0,y0,…],
                       "meetAreas":[ {"id":"…","x":…,"y":…,"groupSize":4} ] } ]
  }]
}
```

**Fixed slots (important).** In `frames[].v` and `.d`, slot *i* is the **same** entity across every frame; an
entity absent that frame is `null` in its slot. This lets the renderer interpolate/​smooth per-entity without
identity tracking. Build the slot map once (sorted distinct ids → index).

---

## 4. Motion smoothing (vehicles)

Frames are discrete (typically `dt=1 s`); the player interpolates and smooths at render time so cars move
fluidly. The vehicle path uses a **kinematic single-track (bicycle) reconstruction** (ported from SumoSharp
`KinematicHeading`): linear-interpolate the raw front between the two bracketing frames (Stage A), then run a
stateful per-vehicle filter (Stage B) — lane-change lateral-error absorption, a lane-direction-predicted
critically-damped front tracker, and an 8-substep **no-slip rear-axle drag** whose rear→front vector is the
drawn heading. Result: cars round corners on a real arc (rear off-tracks inside), never snap-rotate, and
don't spin at stops (heading held below ~0.5 m/s); teleports reseed. Discs (pedestrians) use plain linear
interpolation (crowd frames are dense). Full detail + params: `template.js` (`khUpdate`) and the SumoSharp
`docs/VIEWER-KINEMATIC-SMOOTHING-DESIGN.md`.

---

## 5. Interaction / camera

- **Camera:** `screenX = worldX*scale + offsetX`, `screenY = -worldY*scale + offsetY`. Auto-fits `view` on
  load; stops auto-fitting once the user zooms/pans.
- **Input:** wheel zoom, drag pan, touch pinch/pan (DPR-aware). Zoom clamps `scale ∈ [0.02, 4000]`.
- **Playback:** play/pause (Space), restart (R), speed multiplier (HUD), scrub slider, ←/→ step one frame.
  Real-time clock advances sim-time by wall-Δt × speed; respects `prefers-reduced-motion` (starts paused).
- **HUD:** scene selector (hidden for single-scene), vehicle/disc counts, time readout, speed-colour toggle,
  a per-scene legend auto-built from the layers/kinds actually present.

---

## 6. Source data needed to PRODUCE `REPLAY_DATA`

| Input | Feeds | Format |
|---|---|---|
| **SUMO `net.xml`** (`withInternal`) | `network.lanes` (shape/width/index, `ped` = allows pedestrian & not passenger), `network.junctions` (node shapes), crossings (internal `:node_cN` lanes), and — if emitted — `tls`/`signals` | standard SUMO net |
| **FCD** (`--fcd-output`) | `frames[].v` (`<vehicle x y angle>`) and `frames[].d` (`<person x y>`), fixed-slotted by id | SUMO FCD XML; persons may be interleaved or in a separate `--ped-fcd` |
| **POIs** `pois.json` (pois/v2) | `pois[]` (each `{id, kind, pos:[x,y]}`), and polygon records `kind:"parking_lot"|"park"` → `parkingLots[]`/`parks[]` (with `meet_areas`) | `deduce_pois.py`/`compose.py` output; bare list or `{pois:[…]}` |
| **Zones** `zones.json` (zones/v1) | `zones[]` (`{id, type, polygon:[[x,y],…]}`) | `{zones:[…]}` |
| **Buildings** `buildings.json` (buildings/v1) | `buildings[]` (`{id, type, footprint|polygon:[[x,y],…]}`) | `{buildings:[…]}` |
| `scenario.rou.xml` (optional) | resolves `vdim` from the first vehicle's `vType` length/width | SUMO routes |
| `edge_fields.json` (optional, not rendered directly) | per-edge attributes for downstream tooling; not consumed by the 2D player | project schema |

**Enum values present in the sample:** POI kinds `venue, transit_stop, building_entrance, dwell_spot,
parking_access, parking_lot, park`; building types `mall, garage, office, residential, restaurant`; zone
types `downtown, retail, dining, residential, park, arterial`. The renderer has explicit styling for these;
unknown values fall back to a neutral grey.

**Regenerate (exact command):**
```bash
export SUMO_HOME=/usr/local/lib/python3.11/dist-packages/sumo
export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"
# 1) produce an FCD from a served scenario:
sumo -c scenario.sumocfg --fcd-output box.fcd.xml --end 400 --no-step-log true
# 2) build the HTML with all overlays:
python3 sim_viz.py --net net.xml --fcd box.fcd.xml --rou scenario.rou.xml \
  --pois pois.json --zones zones.json --buildings buildings.json \
  --out sample.html --name "Live city" --desc "…"
```
Peds: emit `<person>` in the FCD (or pass `--ped-fcd`) and they appear as purple discs (`--ped-radius`).

---

## 7. Reproducing in 3D

The same data drives a 3D scene; the mapping:

- **Ground plane / districts:** `zones[].polygon` → tinted ground regions (SUMO XY, Z=0, metres).
- **Roads:** `network.lanes[].shape`+`width` → extruded road ribbons; `ped:true` lanes → raised/curbed
  sidewalks; `junctions[].shape` → junction slabs; `:node_cN` lanes → crosswalk decals.
- **Buildings:** `buildings[].polygon` + `type` → extruded massing. **Heights are NOT in the data** — supply
  a `type → height (range)` mapping (e.g. mall low+wide, office tall, residential mid, garage low) and,
  for determinism, derive any per-building jitter from a hash of the building `id` (no RNG).
- **Props:** `pois[]` by `kind` → placed props (entrance markers, transit stops, "P" pylons, dwell benches);
  `parkingLots[].polygon` → paved lots; `parks[].polygon` → greenspace; `meetAreas` → gathering props.
- **Agents:** `frames[].v` → vehicle meshes (front-anchored at `(x,y)`, extend back by `vdim` length along
  heading; apply the §4 kinematic smoothing for fluid motion); `frames[].d` → pedestrian billboards/meshes
  by `kind`. Fixed slots → stable per-agent instances across frames.
- **Signals (optional):** emit `network.tls`+`signals` (see §2 note) and colour 3D signal heads by the
  phase-at-time computation.
- **Camera/time:** same world frame (Z up, metres); `dt` between frames; interpolate on a render clock.

This is exactly the data the project's City3D viewer is intended to consume; the box companion files
(`buildings.json`/`zones.json`/`pois.json`/`edge_fields.json`) are the shared, data-driven world description.

---

## 8. Notes / guarantees

- **Determinism:** rounding is `round(v,2, half-away-from-zero)`; ids sorted ordinally for slot assignment;
  no RNG anywhere. Same inputs → byte-identical HTML.
- **Optional-layer gating:** every optional layer key is omitted when its source is absent, so adding an
  overlay never perturbs an existing scene's bytes.
- **Timesteps with zero vehicles are dropped** (empty frames are never emitted); `dt` is inferred from the
  first two emitted times.
- **Payload size** scales with `frames × entities`. For a shareable HTML keep the run short/moderate-density
  (e.g. a few hundred seconds); the sample here is deliberately moderate.
