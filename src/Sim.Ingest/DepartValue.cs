namespace Sim.Ingest;

// P0-C1 (docs/HIGH-DENSITY-P0-DESIGN.md "P0-C"/"P0-C1"): symbolic depart attributes. Mirrors
// SUMO's own procedure enums (SUMOVehicleParameter.h's DepartSpeedDefinition/DepartLaneDefinition/
// DepartPosDefinition) but models ONLY the members this rung resolves -- every other symbolic
// keyword (e.g. "desired"/"free"/"random"/"allowed"/"first"/"base"/"last"/"random_free"/
// "speedLimit"/"avg") is rejected loudly by DemandParser rather than silently mishandled or left
// to crash as a FormatException. departPos="stop" is scoped to a lane <stop> only (P0-C2 is the
// parkingArea-based form).
public enum DepartSpeedSpec { Given, Max }
public enum DepartLaneSpec { Given, Best }
public enum DepartPosSpec { Given, Stop, Base }

// (Kind, Literal) pair for departSpeed="...". Literal is meaningful ONLY when Kind == Given (a
// numeric departSpeed, e.g. "0" or "13.89") -- every scenario before this rung is Given, so
// threading Literal straight through as the resolved value stays byte-identical. Max carries no
// literal; it is resolved at insertion time (Engine.TryInsertOnLane) from the lane speed limit /
// speedFactor / leader-safety clamp, per SUMO's MSLane::getDepartSpeed + patchSpeed=true.
public readonly record struct DepartSpeedValue(DepartSpeedSpec Kind, double Literal)
{
    public static DepartSpeedValue Given(double literal) => new(DepartSpeedSpec.Given, literal);
    public static readonly DepartSpeedValue Max = new(DepartSpeedSpec.Max, 0.0);
}

// (Kind, Literal) pair for departLane="...". Literal is meaningful ONLY when Kind == Given (a
// numeric lane index). Best carries no literal; it is resolved at insertion time
// (Engine.InsertDepartingVehicles) from route-continuation length + runtime occupancy, per SUMO's
// MSEdge::getDepartLane's DepartLaneDefinition::BEST_FREE case.
public readonly record struct DepartLaneValue(DepartLaneSpec Kind, int Literal)
{
    public static DepartLaneValue Given(int literal) => new(DepartLaneSpec.Given, literal);
    public static readonly DepartLaneValue Best = new(DepartLaneSpec.Best, 0);
}

// (Kind, Literal) pair for departPos="...". Literal is meaningful ONLY when Kind == Given (a
// numeric position). Stop carries no literal; it is resolved at insertion time
// (Engine.TryInsertOnLane) from the vehicle's first scheduled lane <stop> (its EndPos) when that
// stop is on the insertion lane -- else it falls back to BASE (position 0), per SUMO's
// MSLane::insertVehicle's DepartPosDefinition::STOP case. Lane <stop> only (parkingArea-based
// stops are P0-C2).
//
// Base (DepartPosDefinition::BASE, also SUMO's DEFAULT) carries no literal; it is resolved at
// insertion time to MSBaseVehicle::basePos (MSBaseVehicle.cpp:1117): the vehicle's FRONT bumper
// at MIN(vType.Length + POSITION_EPS, lane.Length), further capped to MAX(0, firstStop.EndPos)
// when the first scheduled stop is on the depart edge. NOT a hardcoded 0 -- the MIN/stop cap only
// collapses to ~0 for very short shapes.
public readonly record struct DepartPosValue(DepartPosSpec Kind, double Literal)
{
    public static DepartPosValue Given(double literal) => new(DepartPosSpec.Given, literal);
    public static readonly DepartPosValue Stop = new(DepartPosSpec.Stop, 0.0);
    public static readonly DepartPosValue Base = new(DepartPosSpec.Base, 0.0);
}
