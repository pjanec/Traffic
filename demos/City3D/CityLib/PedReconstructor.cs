using Sim.Pedestrians.Lod;
using Sim.Replication;

namespace CityLib;

// The ped LOD regime, mirrored into CityLib so neither CityLib nor Viewer depends on Sim.Viewer's own
// PedRegime (the values are the contract, not the type). Low-power = the deterministic PathArc follower;
// high-power = the full FreeKinematic OrcaCrowd agent (docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)":
// "regime from Ig.ModelOf: FreeKinematic -> high-power else low-power").
public enum PedRegime
{
    LowPower = 0,
    HighPower = 1,
}

// One pedestrian's fully-reconstructed render pose, in GODOT coordinates (CoordinateTransform already
// applied) -- the plain struct the Viewer glue turns into a MultiMesh per-instance transform. No Godot type
// here (CityLib stays engine-agnostic); the ped analog of ReconstructedVehicle. The ped net is flat, so Z
// (Godot up = SUMO elevation) is always 0.
public readonly struct ReconstructedPed
{
    public ReconstructedPed(int id, float x, float y, float z, PedRegime regime, bool visible)
    {
        Id = id;
        X = x; Y = y; Z = z;
        Regime = regime;
        Visible = visible;
    }

    public int Id { get; }
    public float X { get; }
    public float Y { get; }
    public float Z { get; }
    public PedRegime Regime { get; }
    public bool Visible { get; }

    public bool IsHighPower => Regime == PedRegime.HighPower;
}

// docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" -- the ped analog of Reconstructor. Wraps a
// Sim.Pedestrians.Lod.PedRemoteReconstructor (the render-side consumer that closes the server -> wire -> IG
// -> render loop off the ped stack, NOT the vehicle DrClock/PoseResolver/DrPoseSmoother stack): Pump(now)
// once per render frame, then for each KnownId TryGetRenderPose -> skip if not visible -> apply the one
// fixed CoordinateTransform.SumoToGodot -> a ReconstructedPed (regime from Ig.ModelOf). All the
// DR/playout-delay/capped-correction smoothing already lives INSIDE PedRemoteReconstructor (the "no
// promotion pop" story), so this layer is thin -- the ped analog of the vehicle Reconstructor being where
// the DR lives.
public sealed class PedReconstructor
{
    private readonly List<ReconstructedPed> _scratch = new();
    private PedRemoteReconstructor? _reconstructor;
    private IPedReplicationSource? _boundSource;

    // Call once per render frame with the ped-sim's current server time (PedSimSource.Time). Constructs the
    // wrapped PedRemoteReconstructor once, from the first source seen, and reuses it every frame after (its
    // smoothing/known-id state is per-instance and must persist across frames, exactly like the vehicle
    // Reconstructor's DrClock/DrPoseSmoother).
    public IReadOnlyList<ReconstructedPed> Reconstruct(IPedReplicationSource source, double serverTime)
    {
        if (_reconstructor is null)
        {
            _reconstructor = new PedRemoteReconstructor(source);
            _boundSource = source;
        }
        else if (!ReferenceEquals(source, _boundSource))
        {
            throw new InvalidOperationException(
                "PedReconstructor is bound to its first IPedReplicationSource; use one PedReconstructor per source.");
        }

        _reconstructor.Pump(serverTime);

        _scratch.Clear();
        foreach (var id in _reconstructor.KnownIds)
        {
            if (!_reconstructor.TryGetRenderPose(id, out var pos, out var visible, out _) || !visible)
            {
                continue;
            }

            // The ped net is flat (z = 0); CoordinateTransform is the ONE fixed SUMO->Godot mapping, reused
            // verbatim from the vehicle path.
            var (gx, gy, gz) = CoordinateTransform.SumoToGodot(pos.X, pos.Y, 0.0);
            var regime = _reconstructor.Ig.ModelOf(id) == PedDrModel.FreeKinematic
                ? PedRegime.HighPower
                : PedRegime.LowPower;

            _scratch.Add(new ReconstructedPed(id, gx, gy, gz, regime, visible: true));
        }

        return _scratch;
    }
}
