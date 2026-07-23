using System.Numerics;
using Raylib_cs;
using Sim.LiveCity;
using Sim.Viewer.Raylib;

namespace Sim.Viewer;

// docs/LIVE-CITY-VISUALS-NOTES.md OWNER-CONFIRMED "POI / area rendering" block ("keep it FLAT, no props")
// / docs/reference/live-city-viz/DESIGN-live-city-2d-viz.md §2 layers 5/6/11: POI areas (parking_lot/park
// ground tint), POI point markers (flat coloured dots by kind), and building-entrance "doors" (a short
// tick along the entrance's facing vector -- the 2D analogue of City3D's DoorBuilder vertical box).
// Mirrors LiveCityZonesLayer.cs/LiveCityBuildingsLayer.cs's shape (a static helper, not a Renderer method,
// so Sim.Viewer.Raylib stays domain-agnostic) and is the ONE place Sim.Viewer turns Sim.LiveCity.ScenePoi/
// SceneArea into raylib draw calls -- ONE `Draw` call covers all three sub-layers so callers gate the
// whole "POI + doors + areas" group with a single `--hide-pois`/`P` toggle, matching City3D's single
// `_poisNode` grouping.
public static class LiveCityPoiLayer
{
    // "Same color palette in 2D and 3D so they read identically" -- these hex values are duplicated
    // verbatim from demos/City3D/Viewer/Main.cs's PoiMarkerPalette (the two viewers don't share a
    // rendering-side assembly, same cross-viewer duplication LiveCityZonesLayer/LiveCityBuildingsLayer's
    // own palettes already use for ZoneFillPalette/BuildingFillPalette). `building_entrance` is
    // deliberately absent -- that kind renders as a DOOR (DoorColor below), never a ground marker.
    private static readonly Dictionary<string, Color> PoiMarkerPalette = new(StringComparer.Ordinal)
    {
        ["venue"] = new Color(245, 158, 11, 255),
        ["dwell_spot"] = new Color(244, 114, 182, 255),
        ["transit_stop"] = new Color(34, 211, 238, 255),
        ["parking_access"] = new Color(96, 165, 250, 255),
    };

    private static readonly Color PoiMarkerDefault = new(200, 200, 200, 255);

    // "building_entrance = the ONE vertical element ... Different color (green) so it reads as a door" --
    // the 2D analogue is a short green tick along Facing (no vertical geometry in a top-down 2D view).
    private static readonly Color DoorColor = new(34, 197, 94, 255);

    // POI AREAS (parking_lot/park): "parking = grey, park = green" -- matches CityLib's PoiAreaFillPalette
    // channels; alpha raised into the same ~0.30-0.35 band LiveCityZonesLayer/LiveCityBuildingsLayer's own
    // remarks explain (a flat 2D fill needs more opacity than a soft canvas blend to read against the
    // dark road/background palette).
    private static readonly Dictionary<string, Color> PoiAreaFillPalette = new(StringComparer.Ordinal)
    {
        ["parking_lot"] = new Color(148, 163, 184, 89),  // ~0.35 alpha
        ["park"] = new Color(34, 197, 94, 76),           // ~0.30 alpha
    };

    private static readonly Color PoiAreaFillDefault = new(156, 163, 175, 51); // ~0.20 alpha

    // Marker radii (world metres), matching CityLib.PoiGroundBuilder's per-kind constants exactly so the
    // two viewers' markers read the same relative size. `parking_access` (351 records in the committed
    // dataset) kept smallest/subtlest per the task's explicit "keep parking_access subtle".
    private const float VenueRadiusMeters = 0.9f;
    private const float TransitStopRadiusMeters = 0.7f;
    private const float DwellSpotRadiusMeters = 0.6f;
    private const float ParkingAccessRadiusMeters = 0.35f;
    private const float DefaultRadiusMeters = 0.6f;

    // "a short green tick/segment at the entrance point along facing" -- half-length either side of the
    // POI point, world metres.
    private const float DoorTickHalfLengthMeters = 1.0f;
    private const float DoorTickThickness = 2f; // screen-space line thickness (raylib DrawLineEx), constant regardless of zoom.

    private static float RadiusForKind(string kind) => kind switch
    {
        "venue" => VenueRadiusMeters,
        "transit_stop" => TransitStopRadiusMeters,
        "dwell_spot" => DwellSpotRadiusMeters,
        "parking_access" => ParkingAccessRadiusMeters,
        _ => DefaultRadiusMeters,
    };

    // Draws, in order (areas -> markers -> doors, flattest/most ground-hugging first, mirroring City3D's
    // BuildLiveCityPois sub-build order): POI area ground tints, then POI point markers (every kind except
    // `building_entrance`), then a short facing-oriented tick for every `building_entrance` POI. Caller
    // draws this BEFORE Renderer.DrawWorldDds/DrawStaticWorld, same "ground layer under the roads" spot
    // LiveCityZonesLayer/LiveCityBuildingsLayer already occupy.
    public static void Draw(Camera2D camera, IReadOnlyList<SceneArea> areas, IReadOnlyList<ScenePoi> pois)
    {
        global::Raylib_cs.Raylib.BeginMode2D(camera);

        DrawAreas(areas);
        DrawMarkers(pois);
        DrawDoors(pois);

        global::Raylib_cs.Raylib.EndMode2D();
    }

    private static void DrawAreas(IReadOnlyList<SceneArea> areas)
    {
        foreach (var area in areas)
        {
            if (area.Polygon.Count < 3)
            {
                continue; // degenerate polygon -- nothing to fill.
            }

            var fill = PoiAreaFillPalette.TryGetValue(area.Kind, out var c) ? c : PoiAreaFillDefault;

            // Raylib.DrawTriangleFan fans around points[0] -- same convex-polygon fan choice
            // LiveCityZonesLayer/LiveCityBuildingsLayer make for their own polygons (parking_lot/park
            // polygons in the committed demo_city/box dataset are simple rectangles/near-convex).
            var pts = new Vector2[area.Polygon.Count];
            for (var i = 0; i < area.Polygon.Count; i++)
            {
                var (x, y) = area.Polygon[i];
                pts[i] = Renderer.Flip(x, y);
            }

            global::Raylib_cs.Raylib.DrawTriangleFan(pts, pts.Length, fill);
        }
    }

    private static void DrawMarkers(IReadOnlyList<ScenePoi> pois)
    {
        foreach (var poi in pois)
        {
            if (poi.Kind == "building_entrance")
            {
                continue; // DoorBuilder/DrawDoors owns this kind -- never double-drawn as a ground marker.
            }

            var color = PoiMarkerPalette.TryGetValue(poi.Kind, out var c) ? c : PoiMarkerDefault;
            var center = Renderer.Flip(poi.X, poi.Y);
            global::Raylib_cs.Raylib.DrawCircleV(center, RadiusForKind(poi.Kind), color);
        }
    }

    private static void DrawDoors(IReadOnlyList<ScenePoi> pois)
    {
        foreach (var poi in pois)
        {
            if (poi.Kind != "building_entrance")
            {
                continue;
            }

            var fx = poi.FacingX ?? 0.0;
            var fy = poi.FacingY ?? 1.0;
            var len = Math.Sqrt((fx * fx) + (fy * fy));
            if (len < 1e-9)
            {
                fx = 0.0;
                fy = 1.0;
                len = 1.0;
            }

            var ux = fx / len;
            var uy = fy / len;

            // A short segment straddling the entrance point along Facing -- the 2D "door mark" (no
            // vertical geometry available in top-down 2D, unlike City3D's DoorBuilder box).
            var a = Renderer.Flip(poi.X - (ux * DoorTickHalfLengthMeters), poi.Y - (uy * DoorTickHalfLengthMeters));
            var b = Renderer.Flip(poi.X + (ux * DoorTickHalfLengthMeters), poi.Y + (uy * DoorTickHalfLengthMeters));
            global::Raylib_cs.Raylib.DrawLineEx(a, b, DoorTickThickness, DoorColor);
        }
    }
}
