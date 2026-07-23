# Live-city visual enrichment — port plan (2D + 3D)

**Status: PLAN / for agreement — not yet implemented** (except the camera controller + TL-in-live-city wiring
+ buildings hide toggle, which are being built now as verification-unblockers). This captures *how* to bring
the reference viz's world layers into our two viewers, per the owner's request. Reference:
`docs/reference/live-city-viz/DESIGN-live-city-2d-viz.md` (the 14-layer spec + §7 "Reproducing in 3D").
Nothing here restates that doc — read it for the per-layer appearance + the 3D mapping.

## The situation (from a fresh code inventory)
- **All the data already exists**, in `scenarios/_ped/demo_city/box/`: `pois.json` (473 POIs incl. **51
  `building_entrance`** with a `building` back-ref + `facing` vector), `buildings.json` (31 buildings with
  `footprint` + real **`height_m`** + `type` + `zone`), `zones.json` (6 districts: downtown/retail/dining/
  residential/park/arterial — exactly the reference palette), `edge_fields.json` (per-edge `sidewalk_width_m`,
  corridor, keep-side — not rendered, useful metadata). This is a **rendering gap, not a data gap.**
- **Both viewers render none of the 5 static world layers** (sidewalks, lane markings, crosswalk zebra, zones,
  data-driven buildings, POIs). City3D draws every lane as one asphalt material; Raylib branches only on
  internal-vs-not and has a per-lane dashed centerline (not the reference's between-lane seam).
- **Buildings are 100% procedural** (`demos/City3D/CityLib/BuildingPlacer.cs`, hash-placed boxes off edges) —
  it never reads `buildings.json`.
- **Traffic lights** exist and are correct (`TrafficLightPlacer`) but were **not wired into `--live-city`**
  (only `--scenario`) — being fixed now.
- **Camera:** no interactive controller — being built now (top priority).

## Shared foundation (do this first, both viewers consume it)
Add scene-overlay loaders once, in the shared host, so 2D and 3D don't each re-parse:
- **`SumoSharp.LiveCity`** gains a `LiveCityScene` (or extend `LiveCitySim`/`LiveCityConfig`) that loads +
  exposes the static overlays from the dataset dir: `Pois` (id, kind, x, y, label, building?, facing?),
  `Buildings` (id, type, footprint[], heightM, zone), `Zones` (id, type, polygon[]). Plain records, no render
  types. `System.Text.Json`. Null/empty when a file is absent (every layer optional, like the reference).
- Raylib (`src/Sim.Viewer`, refs `Sim.LiveCity`) and City3D (`CityLib`, refs the package) both read these —
  one parser, two renderers. Determinism: pure load, no RNG; parity untouched (render-side, no golden path).

## Per-layer port

| Layer | 2D (Raylib) — `src/Sim.Viewer.Raylib/Renderer.cs` or a new overlay | 3D (City3D) — `demos/City3D/CityLib/*` + `Main.cs` |
|---|---|---|
| **Sidewalks** | In `DrawStaticWorld`, branch lane stroke color on the ped-lane flag (lighter concrete vs asphalt) — the flag is on `NetworkModel` lanes (as `sim_viz` uses `lane.ped`) | In `BuildRoadMeshesFromRibbons`/`…Cropped`: a 2nd `StandardMaterial3D` (concrete) for ped lanes; optionally raise them a few cm as a curb (§7) |
| **Lane markings** | Align to the reference technique (dashed line on the seam between same-edge adjacent lanes) — we already have a per-lane centerline approximation to build from | New thin white marking quads along lane seams (a `LaneMarkingBuilder`), or a road-texture/decal; low, unlit white |
| **Crosswalk zebra** | Detect `:<node>_cN` crossing lanes, overpaint perpendicular white stripes along the crossing arc | Striped decal/quads on the crossing lane surface (a `CrosswalkBuilder`) |
| **Zones/districts** | New draw pass: translucent polygon fill by `type` (reference `ZONE_FILL` palette), largest-area first | `ZoneMeshBuilder` → tinted flat ground regions at Z=0 (slightly below roads), by type |
| **Buildings (data-driven)** | New draw pass: footprint fill by `type` | **Primary:** `BuildingFromDataPlacer` reading `buildings.json` — extrude each `footprint` to `height_m`, color by `type`, one `MultiMesh` (or per-type). **Fallback:** the existing procedural `BuildingPlacer` only when no `buildings.json`. **Hide toggle** (being added now). Wire into `ReadyLiveCity`. |
| **Building doors** | The `building_entrance` POI glyph (green triangle) | A door prop on the building wall: place at the entrance POI `pos`, orient by its `facing` vector, matched to its `building` back-ref (so doors sit on the right façade) — a small recessed/framed quad on the building mesh |
| **POIs** | New overlay: glyph-by-kind (venue square / entrance triangle / dwell circle / transit diamond / parking "P"), labels when zoomed | `PoiPlacer` (pure, mirrors `BuildingPlacer`'s no-Godot pattern) → placed props by kind (entrance markers, transit-stop posts, "P" pylons, dwell benches); labels via `Label3D` on click/hover |
| **Traffic lights** | already drawn | **wiring into live-city being fixed now** (placer unchanged) |

## Sequencing (proposed)
1. **NOW (unblockers, in flight):** camera controller · TL heads in live-city · buildings hide toggle.
2. **Foundation:** the shared `LiveCityScene` overlay loader (pois/buildings/zones).
3. **Ground & roads:** zones (ground tint) → sidewalks (distinct shade) → crosswalk zebras → lane markings.
4. **Buildings:** data-driven massing from `buildings.json` (height_m + type), procedural fallback, then doors
   from `building_entrance` POIs.
5. **POIs:** the remaining kinds as props/glyphs (venues, transit, dwell, parking).
Each step lands in both viewers (2D first — cheaper/faster to iterate — then 3D), independently demoable,
render-side only (parity untouched). We formalize into design/tasks/tracker before building, per CLAUDE.md.

## Decisions (owner-confirmed)
- **STANDING DIRECTIVE — data over defaults.** Whenever we have real data for something, render/drive it from
  the data, not a synthetic default. Defaults/procedural are a **fallback for when data is absent**, only.
  (This governs every layer below, not just buildings.)
- **Building heights:** use `buildings.json` `height_m` directly. ✔
- **Procedural `BuildingPlacer`:** demoted to the **no-data fallback only** (used when `buildings.json` is
  absent). ✔
- **2D scope:** the static layers land in the **Raylib 2D viewer too** — confirmed important for orientation +
  bug-spotting, not just aesthetics. Port each layer to BOTH viewers (2D first to iterate fast, then 3D). ✔

## POI / area rendering (owner-confirmed — keep it FLAT, no props)
- **Everything is flat on the ground EXCEPT doors.** No modeled props (no posts, benches, blades, pylons,
  shelters, trees) — deliberately, to avoid clutter.
- **POI points** (`venue`, `transit_stop`, `dwell_spot`, `parking_access`): flat colored **ground
  markers/decals** by kind (color = the reference palette). No vertical geometry.
- **POI areas** (`parking_lot`, `park`) and **zones/districts**: flat colored **ground polygons** (translucent
  tint, painted just below the roads). Parking lot = paved gray; park = green; zones = subtle per-type wash.
- **`building_entrance` = the ONE vertical element: a thin, flat, vertical colored box** placed on the
  building wall where the door would be, at the entrance `pos`, oriented by its `facing` vector, matched to its
  `building` id. Different color so it reads as a door.
- **Density:** `parking_access` (351) as flat ground decals is fine (no clutter now that they're flat); still
  give a per-category show/hide toggle.
- 3D and 2D use the same color palette so the two viewers read identically.

## Still open (minor)
- Whether to promote the "data over defaults" directive into `CLAUDE.md` as a standing project rule (happy to,
  on request).
