namespace Sim.Ingest;

// Immutable demand model, parsed once from .rou.xml. Only attributes explicitly present are
// captured for VType (CLAUDE.md/DESIGN.md: --save-state itself does not expand vType defaults,
// so this parser must not invent one either -- resolved defaulting is a separate, later
// cross-check against golden.vtype.json, not this ingest step).
//
// Each field is an optional override on top of the vClass-default table (Sim.Ingest.
// VTypeDefaults): a rou.xml <vType> only ever sets the attributes it explicitly needs (rung 4's
// leader sets maxSpeed="5.00", both rung 4 vTypes set sigma="0"), and everything else is left to
// the resolver's `override ?? default` per attribute -- never invented here.
public sealed record VType(
    string Id,
    string? VClass,
    double? Sigma,
    double? MaxSpeed = null,
    double? Accel = null,
    double? Decel = null,
    double? Tau = null,
    double? MinGap = null,
    double? Length = null,
    double? EmergencyDecel = null,
    double? SpeedFactor = null,
    // Rung A3: sumo/src/microsim/MSVehicle.cpp:7266 ignoreRed's
    // getJMParam(SUMO_ATTR_JM_DRIVE_AFTER_RED_TIME, -1) -- a vType-level (not <param> child)
    // junction-model attribute; null here (no override present) resolves to SUMO's -1 default
    // in VTypeDefaults.Resolve ("never ignore red").
    double? JmDriveAfterRedTime = null,
    // Rung ER2: emergency ignore-FOE junction-model attributes (jmIgnoreFoeProb / jmIgnoreFoeSpeed
    // / jmIgnoreJunctionFoeProb). Null (no override) resolves to SUMO's 0 default ("never ignore a
    // foe") in VTypeDefaults.Resolve, so every vType that omits them stays byte-identical.
    double? JmIgnoreFoeProb = null,
    double? JmIgnoreFoeSpeed = null,
    double? JmIgnoreJunctionFoeProb = null,
    // Rung ER3 (give-way): our opt-in model of SUMO's MSDevice_Bluelight assignment. Null (absent)
    // resolves to false in VTypeDefaults.Resolve, so no give-way is ever induced by default.
    bool? HasBluelight = null,
    // Rung OV1 (opposite-direction overtaking): opt-in analog of SUMO's lcOpposite. Null (absent)
    // resolves to false, so no vehicle ever considers the oncoming lane by default.
    bool? LcOpposite = null,
    // C11-i: sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:331's `carFollowModel` vType
    // attribute (SUMO_TAG_CF_KRAUSS's XML tag name is "Krauss", SUMO_TAG_CF_IDM's is "IDM").
    // null here (no override present) resolves to "Krauss" in VTypeDefaults.Resolve, exactly as
    // before this rung.
    string? CarFollowModel = null,
    // Rung R6: MSCFModel_Rail cf-params (sumo/src/microsim/cfmodels/MSCFModel_Rail.cpp:62-133),
    // read from the <vType>'s own XML attributes (getCFParam/getCFParamString reads the attribute
    // map). Only meaningful when carFollowModel="Rail"; null for every other vType. `trainType`
    // selects the base parameter set (only "custom" is in scope -- the built-in NGT400/ICE/… lookup
    // tables are deferred); maxPower/maxTraction give the parametric traction curve and
    // resCoef_{constant,linear,quadratic} the parametric resistance curve; massFactor/mass override
    // the rotating-mass weight.
    string? TrainType = null,
    double? MaxPower = null,
    double? MaxTraction = null,
    double? ResCoefConstant = null,
    double? ResCoefLinear = null,
    double? ResCoefQuadratic = null,
    double? MassFactor = null,
    double? Mass = null,
    // Phase 2 (sublane, P2.3): SUMO's sublane vType attributes
    // (sumo/src/utils/vehicle/SUMOVTypeParameter.cpp:333-335 defaults). MaxSpeedLat is the max
    // lateral speed (m/s, default 1.0); LatAlignment is the preferred lateral alignment
    // ("center" default | "right" | "left" | "compact" | "nice" | "arbitrary" | a numeric offset).
    // Both null (absent) resolve to their SUMO defaults; inert unless lateral-resolution > 0.
    double? MaxSpeedLat = null,
    string? LatAlignment = null,
    // Rung P2-core (keepLatGap): SUMOVTypeParameter.cpp:62 minGapLat(0.6) -- the desired lateral
    // gap (m, at high speed/closing-speed) a vehicle tries to keep from a same-lane neighbour.
    // Null (absent) resolves to SUMO's 0.6 default in VTypeDefaults.Resolve; inert unless
    // lateral-resolution > 0.
    double? MinGapLat = null);

public sealed record Route(
    string Id,
    IReadOnlyList<string> Edges);

// A scheduled <stop> child of <vehicle> (rung 5). Only the non-waypoint lane-stop subset is
// modeled (lane/startPos/endPos/duration) -- busStop/parkingArea/triggered/until/waypoint
// (speed>0) stops are Sim.Ingest's future-scenario surface, not this rung's.
public sealed record StopDef(
    string LaneId,
    double StartPos,
    double EndPos,
    double Duration);

public sealed record VehicleDef(
    string Id,
    string TypeId,
    string RouteId,
    double Depart,
    double DepartPos,
    double DepartSpeed,
    int DepartLaneIndex,
    IReadOnlyList<StopDef>? Stops = null,
    // Phase 2 (sublane, P2.2): SUMO's departPosLat -- the initial lateral position on the depart
    // lane. "center" (default) | "left" | "right" | a numeric offset (m, +left). null (absent)
    // resolves to centre (0). Applied only when lateral-resolution > 0 (the engine keeps every
    // phase-1 vehicle lane-centred), so byte-identical for phase 1.
    string? DepartPosLat = null)
{
    // Records can't default a reference-type param to a freshly-allocated empty collection
    // (default values must be compile-time constants), so callers that omit Stops get null;
    // this property is what every reader actually uses.
    public IReadOnlyList<StopDef> Stops { get; init; } = Stops ?? Array.Empty<StopDef>();
}

// F2 (probabilistic flow demand): a <flow> with `probability=` is NOT expanded to concrete
// VehicleDefs at load (its arrivals are decided per-step at runtime by a Bernoulli draw), so it is
// carried through as a template. `Probability` is the per-second insertion probability (SUMO's
// SUMO_ATTR_PROB); the engine draws once per active flow per step from a per-flow seeded VehicleRng
// and inserts a vehicle "<Id>.<k>" (running counter k) with probability `Probability * stepLength`.
// This is DETERMINISTIC and reproducible (given the engine seed), which is what serves the
// warm-start "deterministically precompute a populated network" use case; matching SUMO's exact
// insertion STREAM (statistical parity vs a SUMO ensemble) is a separate, network-only cross-check.
public sealed record ProbabilisticFlow(
    string Id,
    string TypeId,
    string RouteId,
    double Begin,
    double End,
    double Probability,
    double DepartPos,
    double DepartSpeed,
    int DepartLaneIndex);

public sealed record DemandModel(
    IReadOnlyList<VType> VTypes,
    IReadOnlyDictionary<string, VType> VTypesById,
    IReadOnlyList<Route> Routes,
    IReadOnlyDictionary<string, Route> RoutesById,
    IReadOnlyList<VehicleDef> Vehicles,
    IReadOnlyList<ProbabilisticFlow>? ProbabilisticFlows = null)
{
    // Same "records can't default a reference-type param to an allocated empty collection" pattern
    // as VehicleDef.Stops: callers that predate F2 omit the arg and get an empty list, never null.
    public IReadOnlyList<ProbabilisticFlow> ProbabilisticFlows { get; init; } = ProbabilisticFlows ?? Array.Empty<ProbabilisticFlow>();
}
