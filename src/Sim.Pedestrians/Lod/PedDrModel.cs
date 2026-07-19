namespace Sim.Pedestrians.Lod;

// The pedestrian dead-reckoning model for a ped's CURRENT sim-LOD state (docs/PEDESTRIAN-DESIGN.md §5,
// §7; docs/PEDESTRIAN-POC-PLAN.md POC-3). Deliberately its OWN enum, not an extension of
// Sim.Core.DrModel (the car stack's LaneArc | FreeKinematic | Stationary): Core is a frozen seam a
// parallel session is actively editing (CLAUDE.md Principle 6), and PathArc is a genuinely pedestrian
// concept -- arc-length walked along a navmesh polyline, not a lane-relative window -- so it belongs in
// the pedestrian layer, not bolted onto Core.
public enum PedDrModel
{
    // Deterministic path-follower: pose is a pure function of (path, startTime, speed, now) via
    // PathArcMotion. No neighbour reactivity -- this is what lets the IG reconstruct it from the path
    // alone (docs/PEDESTRIAN-DESIGN.md §4 "Low-power motion", Principle 4).
    PathArc = 0,

    // Full OrcaCrowd agent: position/velocity streamed per step, reactive avoidance included.
    FreeKinematic = 1,

    // Parked / not moving (not exercised by POC-3; reserved for the regime machine's Parked/Riding
    // states, docs/PEDESTRIAN-DESIGN.md §2).
    Stationary = 2,

    // LIVE-POC-1 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §1, §2, §10 "Scheduled"): pose+animation is a
    // pure function of (ActivityTimeline, now) via ActivityTimeline.PoseAt -- the PathArc trick
    // generalized to a richer Walk/Pause/Dwell schedule. Same low-power story: no neighbour
    // reactivity, IG-reconstructable from the one-time ActivityTimelineRecord broadcast.
    ActivityTimeline = 3,
}
