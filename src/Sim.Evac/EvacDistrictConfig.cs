namespace Sim.Evac;

// P5-1(B) (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §6): tunables for
// EvacDistrictDirector -- the pedestrian panic-evacuation built on the Sim.Pedestrians
// PedestrianWorld facade over a REAL walkable net (the P5-PRE evac-district). Deliberately small:
// the routing/promotion MECHANICS live in Sim.Pedestrians (PedestrianWorld/PedLodManager); this
// config only tunes the ambient population and the panic/arrival thresholds this director itself
// decides.
public sealed record EvacDistrictConfig
{
    // Ambient (calm) pedestrian population spawned once at construction time. No demand generator --
    // this scenario's population is fixed for the run (P5-B does not require OD churn, only the
    // routed panic/flee/escape property).
    public int PedCount { get; init; } = 40;

    // Shared walking speed (m/s) and footprint radius (m) for every ped, calm or panicked.
    public double MaxSpeed { get; init; } = 1.3;
    public double PedRadius { get; init; } = 0.3;

    // Waypoint/goal arrival tolerance (m): both the facade's own steering settle radius (passed to
    // PedestrianWorld's constructor, so a high-power ped's ORCA goal-seek actually eases to a stop
    // within this radius of its target) and this director's own "has this fleeing ped reached its
    // safe zone" threshold. Kept equal on purpose -- a ped that has visibly settled at the corner
    // under the facade's own steering is exactly the ped this director should also detect as
    // Escaped, not one hovering just outside a separately-tuned director-level radius.
    public double ArriveRadius { get; init; } = 1.0;

    // PedestrianWorld/PedLodManager's minimum promotion/demotion dwell (s). Largely inert for the
    // panic path itself: SetForcedHighPower(true) bypasses the dwell gate on promotion and blocks
    // demotion entirely while pinned, and a panicked ped here is never unpinned (it escapes -- i.e.
    // is removed -- instead of demoting back to calm). Still required by the facade's constructor.
    public double DwellSeconds { get; init; } = 1.0;

    // A calm ped within this distance (m) of the incident epicentre panics once the incident is
    // active (docs/PEDESTRIAN-DESIGN.md §6: "a simple incident-proximity panic ... is acceptable"
    // when FearField's VehicleHandle-keyed bookkeeping is awkward for peds -- see
    // EvacDistrictDirector's remarks for why FearField itself was not reused here). Defaults large
    // enough to cover the whole 200x200 district (max distance from the centre incident to any
    // corner is ~141 m) so a default run panics -- and eventually evacuates -- the entire ambient
    // population.
    public double PanicRadius { get; init; } = 200.0;
}
