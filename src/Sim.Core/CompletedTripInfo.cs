namespace Sim.Core;

// GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md): one completed
// (genuinely ARRIVED) vehicle trip, captured by Engine.CaptureCompletedTrips at the route-end
// arrival seam and exposed via Engine.CompletedTrips. Deliberately a Sim.Core type (not
// Sim.Harness.TripInfoRecord) -- Sim.Harness already depends on Sim.Core (its TripInfoParser/
// TripInfoWriter read/write the SUMO-schema file), so Sim.Core cannot depend back on
// Sim.Harness without a cycle. Sim.Sumo (SumoShim), which references both, adapts one to the
// other when writing --tripinfo-output.
//
// Field values match SUMO's MSDevice_Tripinfo formulas exactly (see VehicleRuntime.
// TripWaitingTime/TripTimeLoss/DepartPosResolved and Engine.CaptureCompletedTrips's own
// comments for the full derivation/citations):
//   Id           -- the vehicle's id.
//   Depart       -- the configured depart time (VehicleDef.Depart).
//   Arrival      -- the sim time the vehicle was removed (this step's time + dt).
//   ArrivalLane  -- the lane id occupied at arrival (mandatory for SumoData's audit_nocheat.py).
//   ArrivalPos   -- the CONFIGURED arrival position (the arrival edge's lane length, since no
//                   scenario sets a literal <vehicle arrivalPos=>), NOT the physical overshoot pos.
//   ArrivalSpeed -- speed at the arrival step.
//   Duration     -- Arrival - Depart.
//   RouteLength  -- sum of full lengths of every route edge BEFORE the arrival edge, minus
//                   DepartPosResolved, plus ArrivalPos.
//   WaitingTime  -- the trip-total halted-time accumulator (VehicleRuntime.TripWaitingTime).
//   TimeLoss     -- the trip-total speed-deficit accumulator (VehicleRuntime.TripTimeLoss).
public sealed record CompletedTripInfo(
    string Id,
    double Depart,
    double Arrival,
    string ArrivalLane,
    double ArrivalPos,
    double ArrivalSpeed,
    double Duration,
    double RouteLength,
    double WaitingTime,
    double TimeLoss);
