using Sim.Core.Orca;

namespace Sim.Pedestrians;

// The pedestrian-network model produced by PedNetworkParser. Deliberately immutable and
// data-only: this is pure geometry, read from a net.net.xml (+ optional walkable.add.xml),
// consumed later by the navmesh/tactical-routing providers (docs/PEDESTRIAN-DESIGN.md §4).
//
// This is a SEPARATE ingest from the parity src/Sim.Ingest/NetworkParser.cs
// (docs/PEDESTRIAN-DESIGN.md §0 Principle 6) — it must never be merged with, nor replace, the
// lane-parity network model.
public sealed record PedNetwork(
    IReadOnlyList<PedLane> Sidewalks,
    IReadOnlyList<PedCrossing> Crossings,
    IReadOnlyList<PedWalkingArea> WalkingAreas,
    IReadOnlyList<WalkablePolygon> WalkablePolygons,
    IReadOnlyList<WalkableAccessPoint> AccessPoints);

// A pedestrian-usable sidewalk lane on a normal (non-internal, non-crossing, non-walkingarea)
// edge, i.e. a <lane allow="pedestrian" .../> child of an edge with no "function" attribute.
public sealed record PedLane(
    string Id,
    string EdgeId,
    double Width,
    IReadOnlyList<Vec2> Shape);

// A signalized-or-not pedestrian crossing: an edge with function="crossing". TlLogicId is set
// only when the crossing's junction has a matching <tlLogic>, i.e. the crossing is
// TLS-controlled (its lane carries a signal-link mapping via <connection tl="..." .../>).
public sealed record PedCrossing(
    string Id,
    string JunctionId,
    double Width,
    IReadOnlyList<Vec2> Shape,
    IReadOnlyList<Vec2> Outline,
    IReadOnlyList<string> CrossingEdges,
    string? TlLogicId);

// A walkingarea: an edge with function="walkingarea". The lane's shape is the walkable polygon
// covering the junction corner.
public sealed record PedWalkingArea(
    string Id,
    string JunctionId,
    double Width,
    IReadOnlyList<Vec2> Polygon);

// An open walkable surface from walkable.add.xml (e.g. a plaza or parking-lot polygon) that SUMO
// does not model as pedestrian infrastructure but the navmesh providers must still consume.
public sealed record WalkablePolygon(
    string Id,
    string Type,
    IReadOnlyList<Vec2> Shape);

// A point-of-interest marking where a walkable surface connects to the road/lane world (e.g. a
// parking-lot entry/exit), from walkable.add.xml <poi>.
public sealed record WalkableAccessPoint(
    string Id,
    string Type,
    Vec2 Position);
