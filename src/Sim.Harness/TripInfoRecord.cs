namespace Sim.Harness;

/// <summary>
/// VB-6 (VIZ_BENCH_TASKS.md Phase 2): one completed trip, as read from either a real SUMO
/// <c>--tripinfo-output</c> file or the engine's tripinfo ANALOG (<see cref="TripInfoParser"/>
/// reads both through the same schema subset -- see that class's header comment). Originally
/// carried only the fields the aggregate/statistical comparator needed; GAP-2
/// (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md) adds the six PARITY-grade
/// fields <see cref="TripInfoWriter"/>/the GAP-2 engine writer need to reproduce (and compare
/// against golden) a real SUMO <c>&lt;tripinfo&gt;</c> row: <see cref="ArrivalLane"/>,
/// <see cref="ArrivalPos"/>, <see cref="ArrivalTime"/>, <see cref="RouteLength"/>,
/// <see cref="WaitingTime"/>, <see cref="TimeLoss"/>. All six are nullable with a trailing
/// default of <c>null</c> so every pre-GAP-2 4-argument positional call site (Sim.BenchCity)
/// keeps compiling unchanged.
/// </summary>
public sealed record TripInfoRecord(
    string Id,
    double Depart,
    double Duration,
    double? ArrivalSpeed,
    // GAP-2: the arrival lane id -- mandatory for SumoData's audit_nocheat.py, EXACT string match
    // (not a numeric-tolerance field).
    string? ArrivalLane = null,
    // GAP-2: the CONFIGURED arrival position (MSDevice_Tripinfo.cpp:272's myHolder.getArrivalPos(),
    // NOT the physical overshoot pos -- see Sim.Core.CompletedTripInfo's own header comment).
    double? ArrivalPos = null,
    // GAP-2: the sim time the vehicle was removed (SUMO's tripinfo `arrival` attribute).
    double? ArrivalTime = null,
    // GAP-2: MSDevice_Tripinfo.cpp:297's myRouteLength + arrivalPos.
    double? RouteLength = null,
    // GAP-2: MSDevice_Tripinfo's own trip-total waitingTime (MSDevice_Tripinfo.cpp:179-193) --
    // DISTINCT from SUMO's per-halt myWaitingTime; never resets.
    double? WaitingTime = null,
    // GAP-2: MSVehicle::myTimeLoss (MSVehicle.cpp:4095-4105) at arrival.
    double? TimeLoss = null);
