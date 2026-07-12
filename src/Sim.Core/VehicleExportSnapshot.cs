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

    // SUMO-facing vType id (Def.TypeId, e.g. "truck0") -- the FCD `type=` attribute a consumer
    // joins against .rou.xml <vType> to recover length/width/vClass. VB-0: added so an FCD
    // writer built on this seam can round-trip SUMO's FCD `type` field; inert for every other
    // consumer (TrajectoryPoint never carried it and still doesn't).
    public readonly string VehicleType;

    public readonly double Time;
    public readonly string Lane;
    public readonly double Pos;
    public readonly double Speed;
    public readonly double X;
    public readonly double Y;
    public readonly double Angle;

    // Phase 2 (sublane): the lane-relative lateral offset (== Kinematics.LatOffset, +left of
    // travel), SUMO's FCD `posLat`. 0 for every lane-centred vehicle, so inert for phase-1 output.
    public readonly double PosLat;

    // Rung ER3 (give-way): the vehicle's current give-way intent, computed each plan step from
    // the frozen start-of-step snapshot -- 0 = none, -1 = clear toward the right lane edge, +1 =
    // clear toward the left lane edge (see Engine.DetectGiveWaySide). Always 0 for every vehicle
    // in every scenario that has no active blue-light emergency vehicle in range, so it is inert
    // wherever give-way is not triggered. Exposed here so a behavioral observer/test can assert
    // the DETECTION independently of the ER4/ER5 execution (which shows up in Lane / LatOffset).
    public readonly int GiveWaySide;

    // Rung OV1 (opposite-direction overtaking): whether this vehicle currently intends to overtake
    // through the oncoming lane (held up behind a slower leader with the opposite lane clear ahead).
    // Always false wherever no vType sets lcOpposite, so inert for every existing scenario. Exposed
    // so a behavioral test can assert the DECISION independently of the (later) execution.
    public readonly bool OvertakeActive;

    // Rung OV4 (cooperative oncoming shift): whether this vehicle is an oncoming driver currently
    // pulling to its own outer lane edge to make room for a spilled opposite-direction overtaker
    // closing head-on. Always false wherever no vType sets lcOpposite, so inert for every existing
    // scenario. Exposed so a behavioral test can assert the cooperative DECISION independently of the
    // resulting lateral drift (which shows up in Y).
    public readonly bool CooperativeShift;

    public VehicleExportSnapshot(
        Entity entity,
        int entityIndex,
        string vehicleId,
        string vehicleType,
        double time,
        string lane,
        double pos,
        double speed,
        double x,
        double y,
        double angle,
        int giveWaySide = 0,
        bool overtakeActive = false,
        bool cooperativeShift = false,
        double posLat = 0.0)
    {
        Entity = entity;
        EntityIndex = entityIndex;
        VehicleId = vehicleId;
        VehicleType = vehicleType;
        Time = time;
        Lane = lane;
        Pos = pos;
        Speed = speed;
        X = x;
        Y = y;
        Angle = angle;
        GiveWaySide = giveWaySide;
        OvertakeActive = overtakeActive;
        CooperativeShift = cooperativeShift;
        PosLat = posLat;
    }
}
