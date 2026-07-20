using Sim.Core.Orca;

namespace Sim.Pedestrians;

// P8-3 (docs/PEDESTRIAN-P8-3-POI-REQUEST.md; docs/PEDESTRIAN-LIVELINESS-DESIGN.md §8): a point-of-interest
// deduced by the SumoData sub-area pipeline (deduce_pois.py) -- an internal source/sink that weights the
// auto-deduced ped O->D demand AND is a legitimate in-view appear/disappear anchor for the P8-2 gate (a ped
// may pop at a building door / transit stop / parking board-alight even on-camera; not on an open sidewalk).
//
// `Edge` is the load-bearing field: the walkable edge id (same id space as manifest.subarea.fringe_edges)
// the POI attaches to -- computed and validated on the pipeline side. It ties the POI to the edge-keyed
// legitimacy gate and to the walkable graph. `Weight` is the O->D attractiveness (deduce_pois' KIND_BASE *
// (0.6*betweenness + 0.4*land-use)).
public enum PedPoiKind
{
    BuildingEntrance,
    Venue,
    DwellSpot,
    TransitStop,
    ParkingAccess,

    // pois/v2 (SUBAREA-DEMO-CITY-DESIGN.md §3) -- the composed demo-city added two polygon-anchored kinds.
    // Read here as ORDINARY base POIs (id/kind/pos/edge/weight) so the v2 box loads unfiltered; the v2-only
    // structure (parking_lot's polygon/lane_seam/boardable_car, park's path_edges/meet_areas) is deliberately
    // NOT consumed yet -- the micro-scenario/drive-away productization (R2/R3) is parked (see
    // docs/PEDESTRIAN-R2-SCENARIO-REGISTRY-DESIGN.md). Present so the reader tolerates them, not so they act.
    ParkingLot,
    Park,
}

public sealed record PedPoi(
    string Id,
    PedPoiKind Kind,
    Vec2 Pos,
    string Edge,
    double Weight,
    // SHOULD fields (present where derivable): outward lane normal (building_entrance), finite slots
    // (venue/dwell_spot), OSM/land-use category. Null when the pipeline could not derive them.
    Vec2? Facing = null,
    int? Capacity = null,
    string? LandUse = null);
