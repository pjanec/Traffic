namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Cars" / task T1.5 -- pure per-vehicle box
// placement math, no Godot type anywhere (CityLib stays engine-agnostic; the Viewer project turns this
// into one MultiMeshInstance3D per-instance transform, same "pure math, no Godot types, tested" pattern
// as RoadMeshBuilder/BuildingPlacer). Unlike those two (static geometry, built once), this is called ONCE
// PER VEHICLE PER RENDER FRAME straight off Reconstructor.Reconstruct's output.
public readonly struct CarInstance
{
    public CarInstance(float posX, float posY, float posZ, float yawRad, float scaleX, float scaleY, float scaleZ)
    {
        PosX = posX;
        PosY = posY;
        PosZ = posZ;
        YawRad = yawRad;
        ScaleX = scaleX;
        ScaleY = scaleY;
        ScaleZ = scaleZ;
    }

    // Box center, already in GODOT space. PosY is raised half the box height above the reconstructed
    // ground point (ReconstructedVehicle.Y already carries LaneShapeZ, per Reconstructor's data path) so
    // the box sits ON the lane rather than being bisected by it -- same convention BuildingPlacer uses for
    // its own boxes.
    public float PosX { get; }
    public float PosY { get; }
    public float PosZ { get; }

    // Rotation about +Y (Godot yaw convention, CoordinateTransform.NaviDegToGodotYawRad) -- passed through
    // unchanged from the reconstructed heading.
    public float YawRad { get; }

    // Unit-BoxMesh scale: (Width, height, Length) in the box's own (unrotated) local axes -- ScaleZ is the
    // vehicle's real Length so the box's LOCAL forward (-Z in Godot, see CoordinateTransform) runs along
    // the car's length, ScaleX is its real Width, ScaleY is the fixed believable height.
    public float ScaleX { get; }
    public float ScaleY { get; }
    public float ScaleZ { get; }
}

public static class CarTransform
{
    // Design "Cars": "height a believable constant (~1.4 m for cars ... taller for trucks by vClass)" --
    // T1.5 keeps a single fixed constant (per-vClass height varies by a later task, not this one).
    public const float DefaultHeightMeters = 1.5f;

    // Pure mapping: one ReconstructedVehicle (already in Godot space/yaw, design "Data path") -> one
    // CarInstance a renderer can turn directly into a Transform3D. Called per vehicle, per render frame.
    public static CarInstance ForVehicle(in ReconstructedVehicle v, float height = DefaultHeightMeters)
        => new(v.X, v.Y + height / 2f, v.Z, v.YawRad, v.Width, height, v.Length);

    // Convenience batch mapping for a whole reconstructed frame.
    public static CarInstance[] Map(IReadOnlyList<ReconstructedVehicle> vehicles, float height = DefaultHeightMeters)
    {
        var result = new CarInstance[vehicles.Count];
        for (var i = 0; i < vehicles.Count; i++)
        {
            result[i] = ForVehicle(vehicles[i], height);
        }

        return result;
    }
}
