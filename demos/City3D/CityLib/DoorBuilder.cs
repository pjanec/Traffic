using Sim.LiveCity;

namespace CityLib;

// docs/LIVE-CITY-VISUALS-NOTES.md OWNER-CONFIRMED "POI / area rendering" block: "`building_entrance` = the
// ONE vertical element: a thin, flat, vertical colored box placed on the building wall where the door
// would be, at the entrance `pos`, oriented by its `facing` vector". Pure math, no Godot type anywhere
// (mirrors CarTransform's own "pure per-instance box placement" split -- Main.cs turns this into a
// MultiMeshInstance3D instance transform, same idiom).
public readonly struct DoorInstance
{
    public DoorInstance(float posX, float posY, float posZ, float yawRad)
    {
        PosX = posX;
        PosY = posY;
        PosZ = posZ;
        YawRad = yawRad;
    }

    // Box CENTER, already in GODOT space -- PosY is raised HeightMeters/2 above the POI ground point
    // (CoordinateTransform.SumoToGodot(..., 0.0)) so the door box sits ON the ground rather than being
    // bisected by it, same convention CarTransform.ForVehicle/BuildingPlacer use for their own boxes.
    public float PosX { get; }
    public float PosY { get; }
    public float PosZ { get; }

    // Rotation about +Y (Godot yaw convention, CoordinateTransform.DirectionToGodotYawRad(FacingX,
    // FacingY)) -- the box's local -Z (its THIN, ThicknessMeters axis) points along the entrance's facing
    // direction, so callers scale a unit BoxMesh by (WidthMeters, HeightMeters, ThicknessMeters) and rotate
    // by this yaw (Basis(Up, yaw).ScaledLocal(scale), the same right-multiply CarTransform's own consumer
    // uses) to get a thin door-shaped slab standing edge-on to the wall, face reading along Facing.
    public float YawRad { get; }
}

public static class DoorBuilder
{
    // "a thin, flat, vertical colored box (~2m tall x ~1.2m wide x ~0.2m thick)" -- OWNER-CONFIRMED sizing.
    public const float WidthMeters = 1.2f;
    public const float HeightMeters = 2.0f;
    public const float ThicknessMeters = 0.2f;

    // Builds the placement for one `building_entrance` POI. Facing defaults to due-SUMO-north (0,1) on the
    // rare/defensive chance a caller passes a POI without a Facing vector (every `building_entrance` record
    // in the committed dataset has one, per LiveCityScene's own doc comment -- this is a never-throw
    // fallback, not an expected path).
    public static DoorInstance ForEntrance(ScenePoi poi)
    {
        var (gx, gy, gz) = CoordinateTransform.SumoToGodot(poi.X, poi.Y, 0.0);
        var facingX = poi.FacingX ?? 0.0;
        var facingY = poi.FacingY ?? 1.0;
        var yaw = CoordinateTransform.DirectionToGodotYawRad(facingX, facingY);
        return new DoorInstance(gx, gy + (HeightMeters / 2f), gz, yaw);
    }

    // Builds every door in one pass, skipping any POI that is not `building_entrance` (defensive -- callers
    // are expected to already have filtered, mirroring PoiGroundBuilder.Build's own filter shape).
    public static IReadOnlyList<DoorInstance> Build(IReadOnlyList<ScenePoi> pois)
    {
        var result = new List<DoorInstance>();
        foreach (var poi in pois)
        {
            if (poi.Kind != "building_entrance")
            {
                continue;
            }

            result.Add(ForEntrance(poi));
        }

        return result;
    }
}
