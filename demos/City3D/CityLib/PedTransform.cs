namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- pure per-ped avatar placement math, no Godot type
// anywhere (CityLib stays engine-agnostic; the Viewer project turns this into one MultiMeshInstance3D
// per-instance transform). The ped analog of CarTransform/CarInstance: a slim upright capsule/box, ~0.5 m
// wide x ~1.8 m tall, its center raised half its height above the reconstructed ground point so it stands ON
// the ground rather than being bisected by it (the same convention CarTransform/BuildingPlacer use). No yaw
// needed -- a slim upright avatar reads fine at demo scale without heading.
public readonly struct PedInstance
{
    public PedInstance(float posX, float posY, float posZ, float scaleX, float scaleY, float scaleZ, PedRegime regime)
    {
        PosX = posX;
        PosY = posY;
        PosZ = posZ;
        ScaleX = scaleX;
        ScaleY = scaleY;
        ScaleZ = scaleZ;
        Regime = regime;
    }

    // Avatar center, already in GODOT space. PosY is raised half the avatar height above the reconstructed
    // ground point (ReconstructedPed.Y, which is the flat ped net's ground, always 0 in Godot up) so the
    // avatar sits ON the plaza rather than being bisected by it.
    public float PosX { get; }
    public float PosY { get; }
    public float PosZ { get; }

    // Scale of the unit mesh (a unit BoxMesh or a CapsuleMesh sized to 1x1x1 bounds): (Width, Height, Width)
    // -- ScaleX and ScaleZ are the avatar's footprint diameter, ScaleY its standing height.
    public float ScaleX { get; }
    public float ScaleY { get; }
    public float ScaleZ { get; }

    // Regime rides the struct so the Viewer palette can colour low-power vs high-power distinctly.
    public PedRegime Regime { get; }

    public bool IsHighPower => Regime == PedRegime.HighPower;
}

public static class PedTransform
{
    // A believable slim-avatar height (~1.8 m standing person) and footprint width (~0.5 m shoulder-ish).
    public const float DefaultHeightMeters = 1.8f;
    public const float DefaultWidthMeters = 0.5f;

    // Pure mapping: one ReconstructedPed (already in Godot space, CoordinateTransform applied) -> one
    // PedInstance a renderer can turn directly into a Transform3D. Called per ped, per render frame.
    public static PedInstance ForPed(
        in ReconstructedPed p,
        float height = DefaultHeightMeters,
        float width = DefaultWidthMeters)
        => new(p.X, p.Y + height / 2f, p.Z, width, height, width, p.Regime);

    // Convenience batch mapping for a whole reconstructed frame.
    public static PedInstance[] Map(
        IReadOnlyList<ReconstructedPed> peds,
        float height = DefaultHeightMeters,
        float width = DefaultWidthMeters)
    {
        var result = new PedInstance[peds.Count];
        for (var i = 0; i < peds.Count; i++)
        {
            result[i] = ForPed(peds[i], height, width);
        }

        return result;
    }
}
