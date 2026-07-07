namespace Sim.Ingest;

// Immutable network model (DESIGN.md: "immutable arrays, not entities"). Parsed once from
// .net.xml and never mutated afterward; the engine reads it, it does not own it.
//
// Position model (ported from sumo/src/microsim/MSLane.cpp, MSVehicle.cpp): a lane's shape
// is a polyline in global x/y. A vehicle's authoritative position is lane-relative arc-length
// (MSVehicle::myState.myPos) along that polyline; global x/y is *derived* by walking the shape
// (MSLane::geometryPositionAtOffset), never the other way around.
// Rung 9b-ii: `Width` is ported from a <lane>'s optional `width` attribute -- defaults to
// SUMO_const_laneWidth (sumo/src/utils/common/StdDefs.h:48, 3.2m) when absent, exactly like
// MSLane's own default (this net's <lane> elements omit `width` entirely). Only consumed by
// the junction-conflict geometry (MSLink::setRequestInformation) below; every prior rung's
// lane-following/stop-line/TL math never reads it.
public sealed record Lane(
    string Id,
    string EdgeId,
    int Index,
    double Speed,
    double Length,
    IReadOnlyList<(double X, double Y)> Shape,
    double Width);

public sealed record Edge(
    string Id,
    string From,
    string To,
    IReadOnlyList<Lane> Lanes);

// Ported from a net.xml top-level <connection>: from/to are edge ids (which may themselves be
// internal edges, e.g. ":J_0" -> "JE" for the internal lane's own outgoing continuation --
// rung 9a's route-lane-sequence resolution only ever looks up connections whose `from` is a
// normal edge, but all connections are parsed uniformly here since the source file does not
// distinguish them either). `Via` is the internal lane traversed between `from`/`to` at a
// junction; absent (null) when the connection crosses no junction interior (not exercised by
// this scenario, but tolerated). Rung 10: `Tl` is the controlling <tlLogic> id (null when the
// connection is uncontrolled, e.g. minor/major priority-only links) and `LinkIndex` is this
// connection's index into that tlLogic's per-phase state string (MSLink::getTLLinkIndex).
public sealed record Connection(
    string From,
    int FromLane,
    string To,
    int ToLane,
    string? Via,
    string? Tl,
    int? LinkIndex);

// Rung 10: one <phase> of a <tlLogic>, ported from sumo/src/microsim/traffic_lights/
// MSSimpleTrafficLightLogic.cpp's MSPhaseDefinition -- only `duration` and `state` are needed
// for a 'static' program's link-state lookup (min/max duration, next-phase overrides, etc. are
// actuated/adaptive-only concerns, out of scope per the briefing).
public sealed record TlPhase(double Duration, string State);

// Rung 10: a parsed <tlLogic id=".." type="static" offset=".."> -- only `type="static"` is
// supported (see scope note in Sim.Core.TrafficLightState); `Offset` shifts the cycle start
// (MSSimpleTrafficLightLogic's constructor: myStep 0 begins at simulation time `offset`, not 0,
// in the general case -- always 0 in this scenario, but threaded through rather than assumed).
public sealed record TlLogic(string Id, double Offset, IReadOnlyList<TlPhase> Phases)
{
    // MSSimpleTrafficLightLogic::computeCycleTime: sum of every phase's duration.
    public double CycleLength => Phases.Sum(p => p.Duration);
}

// Rung 9b-i: one link (link index) at a junction's internal-lane crossing point, i.e. one
// entry of a <junction>'s intLanes -- ported from MSLink's notion of a numbered "link" leaving
// an incoming lane through the junction interior (MSLink::myIndex). `InternalLaneId` is
// `IntLanes[Index]`; `Connection` is the top-level <connection> whose `via` equals that
// internal lane (the incoming-lane -> outgoing-lane move this link represents).
public sealed record JunctionLink(int Index, string InternalLaneId, Connection Connection);

// Rung 9b-i: one <request> row (right-of-way for a single link index against every other link
// at the junction) -- ported from sumo/src/netbuild/NBRequest.cpp's response/foes bitstrings,
// which MSRightOfWayJunction loads verbatim into MSLink::myResponse (foes) at simulation
// build time. Bitstring convention (NBRequest.cpp): the string is indexed with the RIGHTMOST
// character as link 0, i.e. bit for link `j` is `mask[mask.Length - 1 - j]`. A set bit in
// `Response` at index `j` means this link must yield to link `j`; a set bit in `Foes` at index
// `j` means link `j` physically conflicts with this link (occupies overlapping road space),
// irrespective of who yields. `Cont` ("continuation") marks a link whose internal lane may be
// entered despite a red/yielded state because it leads immediately into another junction
// (MSLink::myAmCont) -- parsed here, not yet consumed (that is a rung 9b-iii concern).
public sealed record JunctionRequest(int Index, string Response, string Foes, bool Cont)
{
    // Rightmost char = link 0 (NBRequest convention).
    public bool RespondsTo(int foeLink) => Bit(Response, foeLink);
    public bool FoeWith(int foeLink) => Bit(Foes, foeLink);
    private static bool Bit(string mask, int link) =>
        link >= 0 && link < mask.Length && mask[mask.Length - 1 - link] == '1';
}

// Rung 9b-i: the crossing of an ego link's internal lane with a foe link's internal lane --
// ported from the geometric crossing point MSLink::myConflicts entries are built around
// (MSLink.cpp's computation of myConflictSize / getLengthBehindCrossing at network load).
// EgoCrossingArc is the arc-length from the ego internal lane's START to the crossing point,
// so a vehicle's remaining distance to the crossing = (distance remaining to reach the internal
// lane's start) + EgoCrossingArc; FoeCrossingArc is the same measured along the foe's internal
// lane. Only links that geometrically cross (not links that merely merge at a shared endpoint)
// get a JunctionConflict -- see PolylineGeometry's doc comment for the exact rule.
//
// Rung 9b-ii: EgoConflictSize/FoeConflictSize/EgoLengthBehindCrossing/FoeLengthBehindCrossing
// are ported from MSLink::setRequestInformation's crossing-width computation
// (sumo/src/microsim/MSLink.cpp:354-382) -- the raw arc-length crossing point above is shifted
// back by half the conflict's width to account for the two lanes' physical extent, and the
// conflict "size" (how much of the foe/ego lane's width actually overlaps the crossing,
// widened for shallow-angle crossings) is precomputed once at network-load time exactly as
// MSLink does, rather than re-derived per constraint evaluation.
public sealed record JunctionConflict(
    int EgoLink, int FoeLink,
    double EgoCrossingArc, double FoeCrossingArc,
    (double X, double Y) CrossingPoint,
    double EgoConflictSize, double FoeConflictSize,
    double EgoLengthBehindCrossing, double FoeLengthBehindCrossing);

// Rung 9b-i: a parsed <junction> -- `IntLanes` is the link-index-ordered list of internal lane
// ids (IntLanes[i] is link i's internal lane, ported from the order netconvert emits
// `intLanes="..."` in, which MSRightOfWayJunction/MSLink rely on being link-index order).
// `Links`/`Requests`/`Conflicts` are populated only for junctions that actually have a
// right-of-way matrix (non-empty `IntLanes` AND at least one child <request>); dead_end/internal
// junctions parse with empty lists, which is harmless since nothing yields at them (rung 9b-iii
// scope, not this rung).
public sealed record Junction(
    string Id, string Type,
    IReadOnlyList<string> IntLanes,
    IReadOnlyList<JunctionLink> Links,
    IReadOnlyList<JunctionRequest> Requests,
    IReadOnlyList<JunctionConflict> Conflicts);

public sealed record NetworkModel(
    IReadOnlyList<Edge> Edges,
    IReadOnlyDictionary<string, Edge> EdgesById,
    IReadOnlyDictionary<string, Lane> LanesById,
    IReadOnlyList<Connection> Connections,
    IReadOnlyDictionary<(string FromEdge, int FromLane, string ToEdge), Connection> ConnectionsByFromLaneTo,
    IReadOnlyDictionary<string, TlLogic> TlLogicsById,
    IReadOnlyDictionary<(string FromEdge, int FromLane), IReadOnlyList<Connection>> ConnectionsByFromEdgeLane,
    IReadOnlyList<Junction> Junctions,
    IReadOnlyDictionary<string, Junction> JunctionsById,
    IReadOnlyDictionary<string, (Junction Junction, JunctionLink Link)> LinkByInternalLane)
{
    // Rung 10: find the (at most one, in this rung's scope) TL-controlled connection leaving a
    // given lane -- i.e. the connection a vehicle currently on this lane would use to exit it,
    // irrespective of which specific destination edge its route picks (this scenario has only
    // one outgoing connection per lane at all, so there is no ambiguity to resolve; a network
    // with multiple turn choices, only some of which are TL-controlled, is out of scope -- see
    // the briefing's scope note on "multi-link/multi-lane TL states").
    public bool TryGetTlControlledConnection(string edgeId, int laneIndex, out Connection connection)
    {
        if (ConnectionsByFromEdgeLane.TryGetValue((edgeId, laneIndex), out var candidates))
        {
            foreach (var candidate in candidates)
            {
                if (candidate.Tl is not null)
                {
                    connection = candidate;
                    return true;
                }
            }
        }

        connection = null!;
        return false;
    }

    // Rung 9a: resolves the ordered lane-id sequence a vehicle traverses along `routeEdges`,
    // starting at `departLaneIndex` on the first edge -- ported from how SUMO's route/lane
    // machinery expands a route's edge sequence through each junction's connection/via lane
    // (MSLane::getInternalFollowingLane / MSEdge's Successors, conceptually: pick the
    // <connection> whose fromLane matches the current lane index, append its via internal lane
    // if present, then the destination toLane). For a single-edge route this degenerates to
    // exactly `[<edge>_<departLaneIndex>]`, matching every prior single-lane rung's behavior
    // (CLAUDE.md-mandated regression: no change for single-edge routes).
    //
    // Scoped out (not needed by rung 9a's single-lane-per-edge, straight-through scenario):
    // multi-lane lane-choice/continuity heuristics when more than one connection matches the
    // same (fromEdge, fromLaneIndex, toEdge) key, and junction right-of-way (rung 9b) -- this
    // purely resolves the lane sequence, it makes no yielding decision.
    public IReadOnlyList<string> ResolveLaneSequence(IReadOnlyList<string> routeEdges, int departLaneIndex)
    {
        var sequence = new List<string>();
        var currentLaneIndex = departLaneIndex;
        var firstEdge = EdgesById[routeEdges[0]];
        var currentLane = firstEdge.Lanes.First(l => l.Index == currentLaneIndex);
        sequence.Add(currentLane.Id);

        for (var i = 0; i < routeEdges.Count - 1; i++)
        {
            var fromEdgeId = routeEdges[i];
            var toEdgeId = routeEdges[i + 1];
            var key = (fromEdgeId, currentLaneIndex, toEdgeId);

            if (!ConnectionsByFromLaneTo.TryGetValue(key, out var connection))
            {
                throw new InvalidDataException(
                    $"No <connection> found from edge '{fromEdgeId}' lane {currentLaneIndex} to edge '{toEdgeId}'.");
            }

            if (connection.Via is { } via)
            {
                sequence.Add(via);
            }

            var toEdge = EdgesById[toEdgeId];
            var toLane = toEdge.Lanes.First(l => l.Index == connection.ToLane);
            sequence.Add(toLane.Id);
            currentLaneIndex = connection.ToLane;
        }

        return sequence;
    }
}
