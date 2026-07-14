using Sim.Core;

namespace Sim.Evac;

// PANIC-EVAC.md §7: the evac layer's tunables (calibrated against the viz, not fixed by architecture)
// plus the flee-preset param bundle. All defaults are first-cut values for the Phase-1 spine.
public sealed record EvacConfig
{
    // Fear (0..1) at or above which a driver panics and switches to flee (R3 / §8.2). Low by default:
    // in Phase 1 essentially anyone inside the incident radius panics.
    public double ThetaPanic { get; init; } = 0.05;

    // PANIC-EVAC-PHASE2-DESIGN.md §3/§4/§9: Phase-2 fear-field tunables. All ON/first-cut by default
    // so the grid demo still cascades; setting EnableLineOfSight=EnableContagion=EnableJamUnease=false
    // and FearDecay=1 reduces the model exactly to Phase 1 (radius-only instant latch).

    // Gate the direct term on unoccluded line-of-sight to the incident (else always visible in-radius).
    public bool EnableLineOfSight { get; init; } = true;

    // Let panicking neighbours' fear spread to nearby vehicles (contagion term).
    public bool EnableContagion { get; init; } = true;

    // Contagion neighbourhood radius (m): the kernel is 1 at d=0, linear to 0 at this radius.
    public double ContagionRadius { get; init; } = 25.0;

    // Contagion gain (/s): how fast neighbours' fear bleeds into a vehicle's own fear.
    public double ContagionRate { get; init; } = 0.6;

    // Let being stuck (Stationary DR regime) amplify a vehicle's OWN existing fear (never originate it).
    public bool EnableJamUnease { get; init; } = true;

    // Jam amplification gain (/s).
    public double JamUneaseRate { get; init; } = 0.3;

    // Sub-threshold fear decay per step (<1): only matters below ThetaPanic -- a brief contagion blip
    // fades if not sustained. Panicked (latched) vehicles are pinned to 1.0 regardless.
    public double FearDecay { get; init; } = 0.98;

    // Fixed vicinity width (m) added beyond the road geometry to form the known-world's hard outer
    // edge where sidewalks are absent (R7). The pedestrian bounding wall sits this far out.
    public double VicinityWidth { get; init; } = 8.0;

    // A vehicle is 'blocked' (R4) once its DR regime has been Stationary for this dwell (s).
    public double BlockedDwellSeconds { get; init; } = 3.0;

    // A fleeing pedestrian is 'Escaped' (R6b) once it is farther than this (m) from the incident —
    // a far-enough point, not necessarily the world edge.
    public double SafeRadius { get; init; } = 120.0;

    // Pedestrian footprint (m) and panicked jog speed (m/s).
    public double PedRadius { get; init; } = 0.25;
    public double PedMaxSpeed { get; init; } = 3.0;

    // How far ahead (m) the radial away-from-incident goal is placed each step. Clamped into the
    // navmesh, so in practice the goal lands on the hard edge and pedestrians pile there.
    public double FleeGoalDistance { get; init; } = 200.0;

    // Car footprint radius (m) pedestrians (and other cars via the core) avoid — moving while driven,
    // frozen once abandoned.
    public double VehicleDiscRadius { get; init; } = 2.0;

    // Occupants that spill out of each abandoned car as pedestrians (R6).
    public int PedestriansPerCar { get; init; } = 1;

    // Pedestrian ORCA sub-steps per engine step: the lane engine runs at the scenario's (coarse,
    // parity) dt, but ORCA collision-avoidance wants a finer dt, so the crowd is sub-stepped
    // (the same idea as Engine.CrowdReactionSubSteps). Purely internal to the evac layer.
    public int CrowdSubSteps { get; init; } = 10;

    // Boundary edges a fleeing car reroutes toward (R2 flee route). The director picks, per car,
    // the reachable exit whose far end is farthest from the incident.
    public string[] ExitEdges { get; init; } = Array.Empty<string>();

    // PANIC-EVAC-PHASE5-DESIGN.md §3: opt-in working-region auto-attach for `LoadScenario` demand (a
    // large realistic town where cars are never individually `Track`ed by the caller). Off by default
    // so the grid/TLS demos (which rely on explicit `Track`) are unaffected.
    public bool AutoTrackInWorkingRegion { get; init; } = false;

    // PANIC-EVAC-PHASE5-DESIGN.md §3: the working-region disc radius (m) around the incident inside
    // which a vehicle becomes evac-relevant and is auto-`Track`ed. Must be >= the incident radius plus
    // a jam margin (congestion backs up beyond the incident's own influence radius, and those queued
    // vehicles still need fear/flee/blocked treatment once they enter the region).
    public double WorkingRadius { get; init; } = 250.0;

    // PANIC-EVAC-PHASE5-TIER2-DESIGN.md §2b: opt-in uniform spatial hash for the two O(m^2) crowd
    // solvers (the pedestrian OrcaCrowd and the pusher MixedTrafficCrowd via VehicleMover), proven
    // bit-identical to brute-force (OrcaSpatialHashTests / MixedTrafficSpatialHashTests). Default false
    // so every existing demo/test stays byte-identical; a caller opts in for the perf win on a
    // heavily-loaded working region (see Sim.EvacProfile --microbench for the measured speedup).
    public bool UseCrowdSpatialHash { get; init; } = false;

    // PANIC-EVAC-PHASE3-DESIGN.md §4: Orca-push tunables (Option A, external shaped-mover handoff).
    // All ON/first-cut by default so the grid demo actually pushes; EnableOrcaPush=false recovers
    // exact Phase-1/2 behaviour (blocked+panicked converts straight to pedestrian, unchanged).

    // Insert the shaped-mover Orca-push stage between "blocked+panicked" and the pedestrian handoff.
    public bool EnableOrcaPush { get; init; } = true;

    // Speed (m/s) below which a pushing mover counts as "not progressing" toward its away-goal.
    public double OrcaWedgeSpeed { get; init; } = 0.1;

    // Dwell (s) a mover must stay below OrcaWedgeSpeed before it is reported wedged.
    public double OrcaWedgeDwellSeconds { get; init; } = 3.0;

    // Shaped-VO tracking-inflation safety margin (m) for Orca-push movers (MixedTrafficCrowd.SafetyMargin).
    public double OrcaPushSafetyMargin { get; init; } = 0.3;

    // Orca-push mover sub-steps per engine step (mirrors CrowdSubSteps for the pedestrian crowd).
    public int OrcaCrowdSubSteps { get; init; } = 10;

    // Speed cap (m/s) for a pushing mover, overriding VehicleClass.Car.MaxSpeed (14.0). A panicked
    // driver nosing onto the shoulder is manoeuvring at a crawl, not flooring it -- and, just as
    // important, VehicleClass.Car's free-road MaxSpeed combined with the shaped NH steering's
    // accel/decel and turn-rate limits produces a violent, oscillating "arrival" overshoot when the
    // away-goal (recomputed every tick, clamped into the band) sits close to the mover itself (e.g.
    // near a band corner) -- the high-speed swings can outrun the wall's shaped-VO braking in a single
    // step. A modest cap keeps the pusher's own dynamics slow enough for the wall/mutual avoidance to
    // actually track it, matching the believability target (a car nosing off the lane, not drag racing
    // the shoulder).
    public double OrcaPushMaxSpeed { get; init; } = 4.0;

    // PRELIM fix (B1 review carry-over): MixedTrafficCrowd.SteerNonholonomic sets
    // targetSpeed = max(min(CreepSpeed, desired), desired*cos(headingErr)); with CreepSpeed=0 a pusher
    // whose goal is ~90 deg off its heading gets targetSpeed 0 and DEADLOCKS -- it can never nudge
    // forward far enough to turn onto the shoulder (turn rate is tied to forward speed). A small creep
    // floor (m/s) lets it creep forward to reorient toward a lateral away-goal instead of freezing.
    public double OrcaCreepSpeed { get; init; } = 0.5;

    // The aggressive "flee" preset (R2): the bulk override applied to a panicked vehicle. Kept
    // DETERMINISTIC (no jmIgnoreFoe* lever — those consult a per-vehicle RNG); the gridlock is meant
    // to emerge from density + aggression converging on a few exits, not from stochastic gap-running.
    public VehicleParamOverride FleePreset { get; init; } = new()
    {
        SpeedFactor = 1.6,   // want to drive well above the limit
        Tau = 0.5,           // tailgate
        MinGap = 0.5,        // pack close
        Accel = 4.0,         // jump into gaps
        Decel = 6.0,         // and brake hard when they close
    };
}
