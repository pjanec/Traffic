using Sim.LiveCity;

namespace CityLib;

// docs/LIVE-CITY-VISUALS-NOTES.md OWNER-CONFIRMED "POI / area rendering" block ("keep it FLAT, no props"):
// every POI POINT kind EXCEPT `building_entrance` (which becomes the one vertical element instead -- see
// DoorBuilder.cs) renders as a small flat ground marker/decal, colour-by-kind, sized per-kind (venue a
// touch bigger; `parking_access` -- the dense 351-record kind -- kept small/subtle so it doesn't clutter,
// per the task's explicit "keep parking_access subtle"). Pure math, no Godot type anywhere (mirrors
// ZoneGroundBuilder/BuildingFromDataBuilder's "CityLib stays engine-agnostic" split); Main.cs turns this
// into ONE MultiMeshInstance3D (a thin unit disc, one per-instance transform+colour per marker -- the SAME
// instancing idiom BuildBuildings/BuildCarMultiMesh/BuildPedMultiMesh already use).
public readonly struct PoiMarkerInstance
{
    public PoiMarkerInstance(string kind, float posX, float posY, float posZ, float radiusMeters)
    {
        Kind = kind;
        PosX = posX;
        PosY = posY;
        PosZ = posZ;
        RadiusMeters = radiusMeters;
    }

    // pois.json `kind` (venue/transit_stop/dwell_spot/parking_access) -- the caller's palette lookup key.
    public string Kind { get; }

    // Ground point, already in GODOT space (CoordinateTransform.SumoToGodot applied, raised
    // GroundOffsetSumoZ metres above the road/zone surface so a marker painted on a sidewalk/plaza never
    // z-fights with either).
    public float PosX { get; }
    public float PosY { get; }
    public float PosZ { get; }

    // Marker radius in metres (RadiusForKind(Kind)) -- callers scale their shared unit disc mesh
    // (diameter 1) by 2*RadiusMeters.
    public float RadiusMeters { get; }
}

public static class PoiGroundBuilder
{
    // Sits ABOVE the road surface (SUMO z=0) and above the zone tint (ZoneGroundBuilder's -0.05m default)
    // so a marker never z-fights with either -- imperceptible from the live-city overview camera's
    // altitude, same "small offset wins the depth test" reasoning ZoneGroundBuilder.Build's own remark
    // uses (just the opposite sign, since markers sit ON TOP of the ground layers, not below them).
    public const double GroundOffsetSumoZ = 0.02;

    public const float VenueRadiusMeters = 0.9f;
    public const float TransitStopRadiusMeters = 0.7f;
    public const float DwellSpotRadiusMeters = 0.6f;

    // Dense (351 records in the committed dataset) -- kept small/subtle per the owner note so it doesn't
    // clutter the ground even though every one of them is rendered.
    public const float ParkingAccessRadiusMeters = 0.35f;

    public const float DefaultRadiusMeters = 0.6f;

    public static float RadiusForKind(string kind) => kind switch
    {
        "venue" => VenueRadiusMeters,
        "transit_stop" => TransitStopRadiusMeters,
        "dwell_spot" => DwellSpotRadiusMeters,
        "parking_access" => ParkingAccessRadiusMeters,
        _ => DefaultRadiusMeters,
    };

    // Builds one marker per POI point, EXCEPT `building_entrance` (DoorBuilder owns that kind -- the one
    // vertical element, per the owner-confirmed style). Order is preserved from `pois` (stable, useful for
    // tests); this type stays Godot/palette-free -- callers look up their own colour by Kind.
    public static IReadOnlyList<PoiMarkerInstance> Build(IReadOnlyList<ScenePoi> pois)
    {
        var result = new List<PoiMarkerInstance>(pois.Count);
        foreach (var poi in pois)
        {
            if (poi.Kind == "building_entrance")
            {
                continue;
            }

            var (gx, gy, gz) = CoordinateTransform.SumoToGodot(poi.X, poi.Y, GroundOffsetSumoZ);
            result.Add(new PoiMarkerInstance(poi.Kind, gx, gy, gz, RadiusForKind(poi.Kind)));
        }

        return result;
    }
}
