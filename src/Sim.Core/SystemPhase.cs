namespace Sim.Core;

// D6 (FastDataPlane ECS readiness -- phased systems over queries). FDP organizes every unit of
// work as a "system" mapped to one of an ordered set of phases
// (`Fdp.ModuleHost`'s `SystemPhase.{Input,BeforeSync,Simulation,PostSimulation,Export}`,
// scheduled via `[UpdateBefore]`/`[UpdateAfter]`); this enum is the same idea, scoped to exactly
// the phases this engine's step loop actually needs (no `BeforeSync` -- there is no
// cross-module sync barrier here yet; that is a D7/D8 concern, not a D6 one).
//
// This is documentation/structure, not a scheduler: each pass in `Engine.Run` is tagged with a
// `// [SystemPhase.X]` comment naming which phase it belongs to, in the SAME order it already
// ran in before D6 (CLAUDE.md rule 2 / the D6 briefing: preserve calculation order exactly --
// this rung only makes the phase boundaries explicit, it does not reorder or rename any pass).
// A later `IWorld`/`SystemPhase` scheduler (D7) can read these tags to place each system without
// re-deriving the ordering from scratch.
public enum SystemPhase
{
    // FDP: input ingestion (new entities, external events) before simulation touches them.
    // Here: InsertDepartingVehicles (spawns newly-departed vehicles) and UpdateReroutes (reads
    // start-of-step state -- including the live B1 obstacle store -- to redirect a vehicle's
    // route; a structural mutation applied via the command buffer at this phase's own barrier).
    Input,

    // FDP: the deterministic, frozen-snapshot core simulation step; module systems run here
    // (async, in FDP) reading RO components and writing only their own entity's RW state. Here:
    // PlanMovements -- each vehicle reads the frozen pre-move `LaneNeighborQuery` snapshot plus
    // immutable network/vType data and writes ONLY its own `MoveIntent` (CLAUDE.md rule 3).
    Simulation,

    // FDP: settles this frame's structural consequences after Simulation, still before Export.
    // Here: ExecuteMoves (applies every vehicle's own `MoveIntent`, integrates position, and
    // flushes arrival through the command buffer) followed by DecideSpeedGainChanges (keep-right
    // + speed-gain lane-change decisions, run post-move per SUMO's own
    // planMovements->executeMovements->changeLanes ordering, MSNet.cpp:784/790/796; structural
    // lane swaps flushed through the command buffer at this phase's barrier).
    PostSimulation,

    // FDP: read-only, external-facing snapshot of the settled state, safe for an
    // observer/replication layer to consume. Here: EmitTrajectory. Load-bearing timing note (see
    // Engine.Run/EmitTrajectory's own comments): this Export pass runs at the TOP of the loop,
    // emitting the PRIOR step's already-settled PostSimulation result -- do not move it to the
    // end, that would change the traffic-light `time+dt` sampling semantics and desync the
    // trajectory from the golden.
    Export,
}
