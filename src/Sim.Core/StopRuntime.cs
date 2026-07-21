namespace Sim.Core;

// Runtime mirror of a single scheduled <stop> (Sim.Ingest.StopDef), ported from the fields
// MSVehicle::MSStop (sumo/src/microsim/MSStop.h) actually reads for a non-waypoint lane stop
// (stop.getSpeed()==0): lane/startPos/endPos/duration plus the `reached` flag and the
// per-step-decremented `duration` countdown (MSVehicle.cpp's processNextStop, ~lines 1613-1897).
// Only ever mutated during Execute (Engine.ExecuteMoves applies a StopTransition computed
// during Plan) -- CLAUDE.md rule 3: the plan phase must not write shared/runtime state, even
// state as narrowly-scoped as "this vehicle's own next stop."
internal sealed class StopRuntime
{
    public required string LaneId { get; init; }
    public required double StartPos { get; init; }

    // The brake target / final stop position. For a plain lane stop this is the load-resolved
    // <stop endPos>, fixed. GAP-2: for a parkingArea stop it is RESOLVED AT RUNTIME to
    // ParkingArea.LotPosition(AssignedLot) once a lot is claimed (Execute), because the lot a
    // vehicle actually gets depends on which lots are free WHEN IT PARKS -- not knowable at load
    // for an area whose occupants turn over across the run (MSParkingArea::computeLastFreePos).
    // While unclaimed (AssignedLot < 0) it holds the load-time placeholder (0) and the Plan phase
    // brakes toward a provisional lot computed from the start-of-step occupancy snapshot instead.
    // Mutable (not init) only for the parking case; plain lane stops set it once and never touch it.
    public double EndPos;

    // MSStop::getMinDuration's fallback (no until/ended modeled): the configured <stop
    // duration="..."/> in seconds, used to (re)initialize RemainingDuration once reached.
    public required double Duration { get; init; }

    // GAP-3 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §3): true iff this stop originated from a
    // `<stop parkingArea="...">` (Sim.Ingest.StopDef.ParkingAreaId != null) rather than a plain
    // `<stop lane="...">`. Distinguishes the two at the ONE seam that matters for behavior: once
    // `Reached`, a parking stop takes the vehicle OFF the running lane (VehicleRuntime.IsParked --
    // lateral offset, excluded from leader queries) while a plain lane stop stays an ordinary
    // on-lane, blocking stop exactly as before GAP-3 (scenarios 03/13/44 are untouched -- this
    // field is false for every StopDef whose ParkingAreaId is null, which is every one of them).
    // Default false so any StopRuntime built without setting it (there are none post-GAP-3, but the
    // property is not `required` to avoid disturbing any other construction site) is a plain stop.
    public bool IsParking { get; init; }

    // GAP-2: the parkingArea this stop belongs to (StopDef.ParkingAreaId), used to claim/free a lot
    // in the Engine's per-area occupancy table at runtime. Null for every plain lane stop (IsParking
    // false), so the lane-stop path is untouched.
    public string? ParkingAreaId { get; init; }

    // GAP-2: the roadside lot index this stop has claimed, or -1 while unclaimed. Set once (Execute)
    // when the vehicle parks (MSParkingArea::computeLastFreePos picks the lowest free lot at that
    // instant); read back to free the exact lot when the vehicle pulls out. -1 for every plain lane
    // stop.
    public int AssignedLot = -1;

    public bool Reached;
    public double RemainingDuration;
}
