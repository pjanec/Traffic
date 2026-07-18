namespace Sim.Core.Orca;

// P0-1 (docs/PEDESTRIAN-TASKS.md, docs/PEDESTRIAN-DESIGN.md §3(d)): a stable, zero-allocation reference
// to an agent slot in OrcaCrowd, laid out to MIRROR the engine's existing handle convention -- see
// Sim.Core.ObstacleHandle (index + generation) and Sim.Core.VehicleHandle. Same 2-field shape, DISTINCT
// id space: never interchange an OrcaHandle with an ObstacleHandle/VehicleHandle, even if the numeric
// Index happens to coincide.
//
// Generation 0 is never handed out for a live slot (OrcaCrowd starts every slot's generation at 1, and
// bumps it again on every Remove), so `default(OrcaHandle)` (== Invalid) never resolves to a live agent
// -- exactly ObstacleHandle's "generation 0 is reserved" convention.
public readonly record struct OrcaHandle(int Index, uint Generation)
{
    // The never-valid sentinel (Index 0, Generation 0). A live slot's generation is always >= 1, so this
    // is never equal to a real handle.
    public static OrcaHandle Invalid => default;

    // Cheap, crowd-independent check: true unless this is the Invalid sentinel. This does NOT confirm
    // the slot is still alive in any particular OrcaCrowd (that requires the generation to match the
    // crowd's live state, e.g. via OrcaCrowd.IsAlive(handle)) -- it only rules out the default/never-
    // assigned value, the same cheap guard ObstacleHandle callers get from comparing against `None`.
    public bool IsValid => this != Invalid;

    public override string ToString() => $"Orca#{Index}.{Generation}";
}
