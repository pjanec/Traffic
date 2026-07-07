namespace Sim.Core;

// D9 (FastDataPlane ECS readiness -- info/replication export SEAM, READINESS ONLY, TASKS.md
// line ~651). This is the "ECS component -> external descriptor" SOURCE shape FDP's own
// `IDescriptorTranslator` consumes (see FastDataPlane Docs/architectural-rules.md: a translator
// reads a component snapshot by value and produces an external/network descriptor from it --
// this project does NOT build that translator or any network descriptor, only the in-house
// shape a later one would be handed). One `VehicleExportSnapshot` is built ONCE per active
// vehicle per Export-phase frame (see Engine.EmitTrajectory) and carries exactly the identity +
// exportable component state a replication/info layer would need: the FDP-shaped `Entity`
// handle (D5) a descriptor translator keys its external id off, the plain `EntityIndex` (the
// same stable slot key the engine's own side tables already use), the SUMO-facing `VehicleId`,
// the frame `Time`, and the lane-relative + derived-global fields `TrajectoryPoint` already
// carries (`Lane`/`Pos`/`Speed`/`X`/`Y`/`Angle`).
//
// `readonly struct`, always passed `in` (see `ISimExportObserver.OnVehicleExported`): this is
// the CLAUDE.md rule 4 zero-alloc discipline / D4's "no `new`/boxing in the hot path" rule
// applied to the export seam -- building this snapshot is a stack copy of doubles + two small
// strings + a struct handle, not a heap allocation, so registering zero observers costs nothing
// beyond the copy already implied by reading `v.Kinematics`/`v.LaneId` (which EmitTrajectory did
// unconditionally before this rung too), and registering N observers costs one extra virtual
// call per observer per vehicle per frame, never an extra allocation.
public readonly struct VehicleExportSnapshot
{
    // D5's FDP-shaped handle -- the id a later IDescriptorTranslator-style consumer would key
    // its external/network descriptor off, instead of the raw VehicleId string.
    public readonly Entity Entity;

    // The plain int slot key (== Entity.Index) -- mirrors the key the engine's own side tables
    // (lane-sequence pool, stop/avoided-edge tables) already use, for a consumer that wants the
    // cheap array-index form rather than the FDP handle.
    public readonly int EntityIndex;

    // SUMO-facing identity -- same string TrajectoryPoint.VehicleId carries (Def.Id).
    public readonly string VehicleId;

    public readonly double Time;
    public readonly string Lane;
    public readonly double Pos;
    public readonly double Speed;
    public readonly double X;
    public readonly double Y;
    public readonly double Angle;

    public VehicleExportSnapshot(
        Entity entity,
        int entityIndex,
        string vehicleId,
        double time,
        string lane,
        double pos,
        double speed,
        double x,
        double y,
        double angle)
    {
        Entity = entity;
        EntityIndex = entityIndex;
        VehicleId = vehicleId;
        Time = time;
        Lane = lane;
        Pos = pos;
        Speed = speed;
        X = x;
        Y = y;
        Angle = angle;
    }
}
