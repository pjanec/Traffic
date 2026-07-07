using Sim.Ingest;

namespace Sim.Core;

// Per-vehicle mutable runtime state, plus the immutable spawn template (Def) it was created
// from. Kept as one record-per-vehicle for now rather than a struct-of-arrays split: with a
// single rung-1 vehicle there is nothing yet to gain from the SoA reshape, and DESIGN.md's
// struct-of-arrays push is about the *data layout* paying for itself once many vehicles/
// systems exist -- deferring it here blocks nothing (Kinematics/MoveIntent are already
// separable structs, so an eventual SoA split is a mechanical extraction, not a redesign).
internal sealed class VehicleRuntime
{
    public required VehicleDef Def { get; init; }

    // Resolved (fully-defaulted) vType parameters (Sim.Ingest.VTypeDefaults) -- the car-
    // following model reads these, never the raw .rou.xml VType with its optional fields.
    public required ResolvedVType VType { get; init; }

    public bool Inserted;

    // Set once the vehicle runs off the end of LaneSequence (route end) during execute;
    // distinct from Inserted so InsertDepartingVehicles never mistakes "arrived" for "not yet
    // departed" and re-inserts.
    public bool Arrived;
    public string LaneId = string.Empty;

    // Rung 9a: the full ordered lane-id sequence this vehicle's route resolves to (via
    // NetworkModel.ResolveLaneSequence), e.g. ["WJ_0", ":J_2_0", "JE_0"] -- includes internal
    // (junction) lanes between consecutive route edges. Set once at insertion; LaneSeqIndex is
    // the index of the CURRENT lane (LaneId) within this list, advanced by ExecuteMoves as the
    // vehicle's Pos crosses each lane's end. A single-edge route resolves to a one-element
    // sequence, so this collapses to rung 1-8's single-lane "reached the end -> arrived"
    // behavior exactly (CLAUDE.md-mandated regression: unchanged for single-edge routes).
    public IReadOnlyList<string> LaneSequence { get; set; } = Array.Empty<string>();
    public int LaneSeqIndex;
    public Kinematics Kinematics;
    public MoveIntent Intent;

    // Rung 5: this vehicle's scheduled stops (Sim.Ingest.VehicleDef.Stops), in route order.
    // Front-of-queue only is ever consulted (MSVehicle::myStops is a deque with the same
    // front-only access pattern) -- populated once at LoadScenario time from the immutable Def,
    // then only ever mutated (front popped/updated) by Engine.ExecuteMoves.
    public Queue<StopRuntime> Stops { get; } = new();

    // Rung 8b: SUMO's MSLCM_LC2013::myKeepRightProbability -- a stateful per-vehicle accumulator
    // for the keep-right (Rechtsfahrgebot) lane-change incentive. Starts at 0 (SUMO's ctor
    // default); only ever mutated by Engine.ExecuteMoves from the plan phase's MoveIntent
    // (CLAUDE.md rule 3 -- Plan writes only MoveIntent, never this field directly).
    public double KeepRightProbability;

    // Rung A2: SUMO's MSLCM_LC2013::mySpeedGainProbability -- a stateful per-vehicle accumulator
    // for the speed-gain (overtaking) lane-change incentive. Starts at 0 (SUMO's ctor default);
    // unlike KeepRightProbability (plan-phase, pre-move), this is decided/written by the new
    // post-move phase (Engine.DecideSpeedGainChanges) that runs AFTER ExecuteMoves -- SUMO's
    // changeLanes phase reads post-move gaps (MSNet.cpp:784/790/796), so this field is written
    // directly there rather than threaded through MoveIntent (CLAUDE.md rule 3 is still honored:
    // it is written once, after all vehicles' moves are settled, from a single frozen post-move
    // snapshot built at the top of that phase -- not a mid-query shared-state write).
    public double SpeedGainProbability;

    // B3: live reroute-around-blockage bookkeeping (DESIGN.md "Two futures" -- not a SUMO
    // field). BlockedByObstacleSeconds accumulates dt while a FUTURE edge of this vehicle's
    // remaining route is sitting under an active external obstacle; reset to 0 the moment no
    // future edge is blocked. AvoidedEdges is the set of edges this vehicle has already routed
    // around once (so a blockage it has already detoured past never re-triggers a reroute of the
    // same edge). Both start at their zero values (0 / empty), which is exactly the inert-when-
    // absent case: with RerouteThresholdSeconds left at its default (+infinity), Engine.
    // UpdateReroutes returns immediately every step and neither field is ever touched.
    public double BlockedByObstacleSeconds;
    public HashSet<string> AvoidedEdges { get; } = new(StringComparer.Ordinal);
}
