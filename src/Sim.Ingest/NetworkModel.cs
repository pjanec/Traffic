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
// D2 (FastDataPlane readiness): `Handle` is a GLOBAL dense 0..N-1 index across every lane in
// the network -- including internal (`:`-prefixed) lanes -- assigned in parse order by
// NetworkParser. It exists so the hot per-step path (LaneNeighborQuery buckets, the
// reducer's `_network.LanesByHandle[...]` lookups) can index an array instead of hashing a
// string every vehicle, every step. FDP components are unmanaged structs and cannot hold a
// `string` field at all, so this handle is also what a future VehicleRuntime-as-component
// split (D3) will actually store. `Id` (the string) stays authoritative at I/O boundaries
// (FCD emit, obstacle API, router) -- see NetworkModel.LaneHandleById/LanesByHandle.
// D4 (FDP zero-alloc `OnUpdate` rule): `LeftNeighbor`/`RightNeighbor` are the dense Handle of
// the same-edge lane at Index+1/Index-1 (-1 when absent), precomputed once at ingest
// (NetworkParser) so the per-step keep-right/speed-gain lane-change decision (Engine.cs) is an
// O(1) array read instead of a per-vehicle, per-step `edge.Lanes.FirstOrDefault(l => l.Index ==
// ...)` LINQ scan (which also allocated a closure over `lane`).
public sealed record Lane(
    string Id,
    string EdgeId,
    int Index,
    double Speed,
    double Length,
    IReadOnlyList<(double X, double Y)> Shape,
    double Width,
    int Handle,
    int LeftNeighbor = -1,
    int RightNeighbor = -1);

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
// C4-v (sameTarget-merge conflict geometry): two links whose <connection>s feed the SAME
// downstream lane MERGE (converge) rather than cross, so they get no JunctionConflict -- but the
// merge still has a per-lane `lengthBehindCrossing` (EgoLengthBehindCrossing / FoeLengthBehindCrossing)
// ported from MSLink::setRequestInformation's sameTarget arm (sumo/src/microsim/MSLink.cpp:302-331)
// via PolylineGeometry.ComputeDistToDivergence + InterpolateGeometryPosToLanePos. The C4-iv
// sameTarget-merge yield reads these to place the merge-leader's crossing point exactly (the term
// that cancels for a symmetric merge but is ~0.005 for an asymmetric one). Built only for
// sameTarget link pairs that converge close enough to have a real crossing point; a "dummy merge"
// (lanes ending far apart) gets EgoLengthBehindCrossing 0, matching the source's CONFLICT_DUMMY_MERGE.
public sealed record MergeConflict(
    int EgoLink, int FoeLink,
    double EgoLengthBehindCrossing, double FoeLengthBehindCrossing);

public sealed record Junction(
    string Id, string Type,
    IReadOnlyList<string> IntLanes,
    IReadOnlyList<JunctionLink> Links,
    IReadOnlyList<JunctionRequest> Requests,
    IReadOnlyList<JunctionConflict> Conflicts,
    IReadOnlyList<MergeConflict> Merges);

// C2-i: one lane's route-continuity data for STRATEGIC lane-change planning -- a scoped port of
// `struct LaneQ` (sumo/src/microsim/MSVehicle.h:865-886), the per-lane record
// `MSVehicle::updateBestLanes` builds (sumo/src/microsim/MSVehicle.cpp:5744-6063) and that
// TraCI's `getBestLanes` exposes verbatim. This rung is ADDITIVE ONLY: it produces this data via
// `NetworkModel.ComputeBestLanes` below but nothing yet reads it to make a lane-change decision
// (that consumer is C2-ii). See `ComputeBestLanes`'s doc comment for the exact single-look-ahead
// scope and what is deferred.
//
// Sign convention for `BestLaneOffset`: SIGNED lane count, positive = toward the LEFT -- matches
// this repo's `Lane.LeftNeighbor` convention (NetworkParser.cs: a lane's `LeftNeighbor` is the
// same-edge lane at Index+1). Confirmed against SUMO itself: `(*j).bestLaneOffset = bestThisIndex
// - index` (MSVehicle.cpp:5973) is positive exactly when the target lane has a HIGHER index than
// the source lane, and SUMO's own lane index 0 is the rightmost lane increasing leftward (the same
// left/right sense NetworkParser already assigns), so "higher index = left = positive offset"
// holds in both.
public sealed record LaneContinuation(
    int LaneIndex,
    bool AllowsContinuation,
    int BestLaneOffset,
    double Length);

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
    IReadOnlyDictionary<string, (Junction Junction, JunctionLink Link)> LinkByInternalLane,
    // D2: dense lane-handle identity, parallel to LanesById -- LanesByHandle[lane.Handle] ==
    // lane, for every lane (index == Handle). LaneHandleById is the one-time string->handle
    // resolution point for I/O-boundary callers (e.g. AddObstacle(laneId,...) resolving its
    // handle once at call time, not per step).
    IReadOnlyList<Lane> LanesByHandle,
    IReadOnlyDictionary<string, int> LaneHandleById)
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
    //
    // C2-ii: the pool/route-path sequence is resolved via the CONTINUING (best) lane on the
    // first edge, not necessarily `departLaneIndex` itself -- a vehicle may depart on a lane
    // that does not continue its route at all (a drop lane; scenarios/18-strategic-turnlane's
    // E1_0), converging onto this resolved path later via a strategic lane change
    // (Engine.TryStrategicLaneChange). `ComputeBestLanes` (C2-i) always resolves SOME
    // continuing lane on a reachable route, so the old "no <connection> found from the depart
    // lane" throw below can no longer be reached via a non-continuing depart lane -- it now
    // only fires for a genuinely unreachable route (unchanged scope).
    //
    // Byte-identical guard: for a single-edge route (routeEdges.Count == 1) `ComputeBestLanes`
    // is not even called (see the guard below) -- for a MULTI-edge route where the depart lane
    // already continues (every existing multi-edge parity scenario: 9a/9b/A3/B3's 15-reroute,
    // all single-lane-per-edge, per C2-i's own inert-control test), `ComputeBestLanes` reports
    // `AllowsContinuation=true` for it, so `currentLaneIndex` stays exactly `departLaneIndex`
    // -- this resolves to EXACTLY the prior behavior.
    public IReadOnlyList<string> ResolveLaneSequence(IReadOnlyList<string> routeEdges, int departLaneIndex)
    {
        var sequence = new List<string>();
        var firstEdgeId = routeEdges[0];
        var currentLaneIndex = departLaneIndex;

        if (routeEdges.Count > 1)
        {
            var bestLanes = ComputeBestLanes(routeEdges, firstEdgeId);
            foreach (var continuation in bestLanes)
            {
                if (continuation.LaneIndex != departLaneIndex)
                {
                    continue;
                }

                if (!continuation.AllowsContinuation)
                {
                    currentLaneIndex = departLaneIndex + continuation.BestLaneOffset;
                }

                break;
            }
        }

        var firstEdge = EdgesById[firstEdgeId];
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

    // D2: the handle-parallel form of ResolveLaneSequence -- same traversal, same lane
    // sequence, but returned as dense int handles (LanesByHandle[handle] == the lane at that
    // sequence position) instead of string ids, for the hot-path callers that will walk
    // LaneSequenceHandles every step instead of re-hashing LaneId strings. Kept as a wholly
    // separate method (rather than changing ResolveLaneSequence's signature) so every existing
    // string-boundary caller (router, tests) is untouched -- this is purely additive.
    public int[] ResolveLaneSequenceHandles(IReadOnlyList<string> routeEdges, int departLaneIndex)
    {
        var sequence = ResolveLaneSequence(routeEdges, departLaneIndex);
        var handles = new int[sequence.Count];
        for (var i = 0; i < sequence.Count; i++)
        {
            handles[i] = LaneHandleById[sequence[i]];
        }

        return handles;
    }

    // C2-i: single-look-ahead scoped port of MSVehicle::updateBestLanes / LaneQ (see
    // LaneContinuation's doc comment for the struct citation). Ported pieces:
    //   - the per-edge LaneQ build (MSVehicle.cpp:5896-5918): `allowsContinuation` = whether the
    //     lane has ANY <connection> to the next route edge (SUMO's `allowedLanes()`, here queried
    //     via the existing `ConnectionsByFromEdgeLane` table -- no XML re-parse); `length` = the
    //     lane's own edge length (SUMO's initial `q.length = ce->getLength()`, MSVehicle.cpp:5911).
    //   - the last-route-edge special case (MSVehicle.cpp:5951-5989): with no next edge to miss,
    //     every lane trivially continues and every lane's initial length is identical (no
    //     stop/vClass penalty in this rung's scope), so the "lengths differ" branch that computes
    //     a nonzero offset never fires -- every lane gets `BestLaneOffset = 0`.
    //   - the offset tie-break for a non-continuing lane (MSVehicle.cpp:5970-5976): among the
    //     lanes that DO continue, `bestThisIndex` is the lowest continuing index and
    //     `bestThisMaxIndex` the highest; a non-continuing lane's offset is whichever of
    //     `bestThisIndex - index` / `bestThisMaxIndex - index` is smaller in absolute value,
    //     preferring the higher-index (leftward) target on an exact tie -- SUMO's `else` branch.
    //
    // SCOPE (matches this rung's anchor, scenarios/18-strategic-turnlane -- a single junction):
    // only the transition from `currentEdgeId` to the IMMEDIATELY NEXT route edge is considered.
    // DEFERRED (not built here -- no scenario needs it yet): SUMO's backward pass
    // (MSVehicle.cpp:6003-6063) that, for a CONTINUING lane, recursively ADDS the best downstream
    // lane's own `length` onto this lane's `length`, accumulating a route-wide continuation
    // distance across every remaining edge (and picks the best of several downstream lanes when
    // more than one continues). Here `Length` is always just `currentEdgeId`'s own edge length --
    // this is exact for a single-junction lookahead (scenario 18 needs no more) but is NOT the
    // full multi-edge recursion; a multi-junction scenario would need that deferred piece built
    // out before `Length` could be trusted route-wide.
    //
    // Throws if the route is genuinely unreachable from `currentEdgeId` (no lane on it has a
    // <connection> to the next route edge at all) -- not exercised by any committed scenario
    // (scenario 18's E1 always has E1_1 continuing to E2).
    public IReadOnlyList<LaneContinuation> ComputeBestLanes(IReadOnlyList<string> routeEdges, string currentEdgeId)
    {
        var edgeIndex = -1;
        for (var i = 0; i < routeEdges.Count; i++)
        {
            if (routeEdges[i] == currentEdgeId)
            {
                edgeIndex = i;
                break;
            }
        }

        if (edgeIndex < 0)
        {
            throw new InvalidDataException(
                $"Edge '{currentEdgeId}' is not part of the given route ({string.Join(" ", routeEdges)}).");
        }

        var edge = EdgesById[currentEdgeId];
        var orderedLanes = edge.Lanes.OrderBy(l => l.Index).ToList();

        if (edgeIndex == routeEdges.Count - 1)
        {
            // Last route edge: every lane trivially continues (MSVehicle.cpp:5951-5989 -- no
            // lengths differ, so no lane ever gets a nonzero offset here).
            return orderedLanes
                .Select(l => new LaneContinuation(l.Index, AllowsContinuation: true, BestLaneOffset: 0, l.Length))
                .ToList();
        }

        var nextEdgeId = routeEdges[edgeIndex + 1];
        var allowsByIndex = new Dictionary<int, bool>();
        foreach (var lane in orderedLanes)
        {
            var allows = ConnectionsByFromEdgeLane.TryGetValue((currentEdgeId, lane.Index), out var candidates)
                && candidates.Any(c => c.To == nextEdgeId);
            allowsByIndex[lane.Index] = allows;
        }

        var continuingIndices = allowsByIndex.Where(kv => kv.Value).Select(kv => kv.Key).ToList();
        if (continuingIndices.Count == 0)
        {
            throw new InvalidDataException(
                $"No lane on edge '{currentEdgeId}' has a <connection> to the next route edge '{nextEdgeId}' -- route is unreachable.");
        }

        var bestThisIndex = continuingIndices.Min();
        var bestThisMaxIndex = continuingIndices.Max();

        var result = new List<LaneContinuation>();
        foreach (var lane in orderedLanes)
        {
            var allows = allowsByIndex[lane.Index];
            var offset = 0;
            if (!allows)
            {
                var toLowest = bestThisIndex - lane.Index;
                var toHighest = bestThisMaxIndex - lane.Index;
                offset = Math.Abs(toLowest) < Math.Abs(toHighest) ? toLowest : toHighest;
            }

            result.Add(new LaneContinuation(lane.Index, allows, offset, lane.Length));
        }

        return result;
    }
}
