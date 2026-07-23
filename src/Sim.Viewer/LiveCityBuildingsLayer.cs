using System.Numerics;
using Raylib_cs;
using Sim.LiveCity;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// docs/LIVE-CITY-VISUALS-NOTES.md "Buildings (data-driven)" row / docs/reference/live-city-viz/DESIGN-
// live-city-2d-viz.md §2 layer 7: footprint fill by type, mirrors LiveCityZonesLayer.cs's shape (a static
// helper -- not a Renderer method -- so Sim.Viewer.Raylib stays domain-agnostic and this stays the ONLY
// place in the demo tool that turns Sim.LiveCity.SceneBuilding into raylib draw calls). Unlike the
// opt-in zone tint, buildings are a PRIMARY feature (task: "Default ON") -- the caller draws this
// unconditionally (subject to the `--hide-buildings` gate) with the ground layers, on top of zones, BEFORE
// the road pass (so buildings sit beside/behind the streets, never covering the cars drawn later).
public static class LiveCityBuildingsLayer
{
    // Matches the reference renderer's BUILDING_FILL exactly (docs/reference/live-city-viz/renderer/
    // templates/template.js:114-121): mall amber, office blue, residential teal, restaurant red, garage
    // grey. Alpha is taken straight from the reference (already 0.55-0.7, noticeably more opaque than
    // ZoneFillPalette's ~0.20-0.25 -- LiveCityZonesLayer's own remark explains why a flat 2D fill needs
    // more opacity than the reference's soft browser canvas blend; buildings, being solid massing rather
    // than a translucent ground wash, keep the reference's already-high alpha unchanged) so buildings read
    // as solid fills against the road/background palette.
    private static readonly Dictionary<string, Color> BuildingFillPalette = new(StringComparer.Ordinal)
    {
        ["mall"] = new Color(245, 158, 11, 166),         // ~0.65 alpha
        ["office"] = new Color(59, 130, 246, 153),       // ~0.60 alpha
        ["residential"] = new Color(45, 212, 191, 140),  // ~0.55 alpha
        ["restaurant"] = new Color(248, 113, 113, 166),  // ~0.65 alpha
        ["garage"] = new Color(107, 114, 128, 178),      // ~0.70 alpha
    };

    private static readonly Color BuildingFillDefault = new(156, 163, 175, 140); // ~0.55 alpha (BUILDING_FILL_DEFAULT)

    // Dark edge stroke (reference: `rgba(15,17,22,0.6)`) -- the DESIGN doc's row 7 "+ dark edge" so each
    // footprint reads as a distinct building rather than blending into its neighbours.
    private static readonly Color BuildingEdgeColor = new(15, 17, 22, 153);

    // Draws every building footprint as a filled, edge-outlined polygon coloured by `Type`. No area
    // ordering is needed (unlike zones): real building footprints in the committed demo_city/box dataset
    // don't overlap, so draw order is irrelevant -- callers may pass the scene's buildings in any order.
    public static void Draw(Camera2D camera, IReadOnlyList<SceneBuilding> buildings)
    {
        if (buildings.Count == 0)
        {
            return;
        }

        global::Raylib_cs.Raylib.BeginMode2D(camera);

        foreach (var building in buildings)
        {
            if (building.Footprint.Count < 3)
            {
                continue; // degenerate polygon -- nothing to fill.
            }

            var fill = BuildingFillPalette.TryGetValue(building.Type, out var c) ? c : BuildingFillDefault;

            // Raylib.DrawTriangleFan fans around points[0] -- exact for the convex rectangular footprints
            // the demo_city/box dataset ships (same fan-triangulation choice LiveCityZonesLayer/
            // CityLib.ZoneGroundBuilder make for their own convex polygons); a future non-convex footprint
            // would need ear-clipping here too (mirrors CityLib.BuildingFromDataBuilder's 3D-side handling).
            var pts = new Vector2[building.Footprint.Count];
            for (var i = 0; i < building.Footprint.Count; i++)
            {
                var (x, y) = building.Footprint[i];
                pts[i] = Renderer.Flip(x, y);
            }

            global::Raylib_cs.Raylib.DrawTriangleFan(pts, pts.Length, fill);

            // Dark edge outline, closing back to the first vertex.
            for (var i = 0; i < pts.Length; i++)
            {
                var a = pts[i];
                var b = pts[(i + 1) % pts.Length];
                global::Raylib_cs.Raylib.DrawLineV(a, b, BuildingEdgeColor);
            }
        }

        global::Raylib_cs.Raylib.EndMode2D();
    }
}
