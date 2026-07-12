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
    int RightNeighbor = -1,
    // SUMOSHARP-API.md §6 (geometry-3D): per-vertex elevation, index-aligned with Shape. Null when the
    // net's lane shape is 2-D (`x,y` pairs) -- the common case, byte-identical to before. Populated only
    // when the shape carries a 3rd `x,y,z` component; consumed only by the read surface's PosZ column
    // (LaneGeometry.ElevationAtOffset), never by any car-following / lane-change / junction math.
    IReadOnlyList<double>? ShapeZ = null);

public sealed record Edge(
    string Id,
    string From,
    string To,
    IReadOnlyList<Lane> Lanes,
    // R3 (rail bidi): the id of the opposing edge that shares this edge's physical track, from the
    // net.xml `<edge ... bidi="...">` attribute (netconvert emits it for reversed-geometry rail
    // edge pairs). null for every non-bidi edge, so this is inert for all road scenarios. A lane on
    // a bidi edge shares its track with the same-index lane on the bidi edge (single-lane rail track
    // in scope; multi-lane bidi lane-pairing is not needed for the committed rail scenarios).
    string? BidiEdgeId = null);

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
// MSSimpleTrafficLightLogic.cpp's MSPhaseDefinition -- `duration` and `state` drive a 'static'
// program's link-state lookup.
// C6-ii: `MinDur`/`MaxDur` are the actuated-program bounds (MSPhaseDefinition::minDuration/
// maxDuration). SUMO defaults an unspecified minDur/maxDur to the phase `duration` itself
// (MSPhaseDefinition's constructor), which is exactly what makes a yellow/all-red phase
// non-actuated (minDur == maxDur == duration -> isActuated() false). We keep them nullable and
// resolve the "== Duration" default at read time (MinDuration/MaxDuration below) so a static
// program (no minDur/maxDur anywhere) is completely unaffected.
public sealed record TlPhase(double Duration, string State, double? MinDur = null, double? MaxDur = null)
{
    // MSPhaseDefinition: an unset minDur/maxDur equals the fixed `duration`.
    public double MinDuration => MinDur ?? Duration;
    public double MaxDuration => MaxDur ?? Duration;

    // MSPhaseDefinition::isActuated(): a phase participates in gap-based extension only when its
    // min and max durations differ (an explicit [minDur, maxDur] window). Yellow/all-red phases,
    // which carry a single fixed duration, return false and are never extended.
    public bool IsActuated => MinDuration != MaxDuration;
}

// Rung 10 / C6-ii: a parsed <tlLogic id=".." type=".." offset=".."> -- `type` selects the phase
// engine: "static" (MSSimpleTrafficLightLogic, a pure function of time; see Sim.Core.
// TrafficLightState) or "actuated" (MSActuatedTrafficLightLogic, a stateful detector-driven phase
// machine; see Sim.Core.ActuatedTrafficLightLogic). `Offset` shifts the cycle start
// (MSSimpleTrafficLightLogic's constructor: myStep 0 begins at simulation time `offset`, not 0).
public sealed record TlLogic(string Id, double Offset, IReadOnlyList<TlPhase> Phases, string Type = "static")
{
    // MSSimpleTrafficLightLogic::computeCycleTime: sum of every phase's duration.
    public double CycleLength => Phases.Sum(p => p.Duration);

    public bool IsActuated => Type == "actuated";
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

// VB-1 (Sim.Viz, VIZ_BENCH_TASKS.md): `Shape` is the junction's `<junction shape="...">`
// polygon (same "x,y x,y ..." polyline format as a lane's shape) -- ported straight from
// netconvert's output, purely for rendering (the viz tool fills this polygon as a solid area
// under the lanes). Empty when the attribute is absent (e.g. some internal junctions omit it).
// Inert for the engine: nothing in Sim.Core reads Junction.Shape.
public sealed record Junction(
    string Id, string Type,
    IReadOnlyList<string> IntLanes,
    IReadOnlyList<JunctionLink> Links,
    IReadOnlyList<JunctionRequest> Requests,
    IReadOnlyList<JunctionConflict> Conflicts,
    IReadOnlyList<MergeConflict> Merges,
    IReadOnlyList<(double X, double Y)> Shape);

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

    // R3 (rail bidi): the id of the lane that shares this lane's physical track in the opposing
    // direction, or null when the lane's edge is not bidi. A lane on a bidi edge shares its track
    // with the SAME-index lane on the edge named by `BidiEdgeId` (single-lane rail track in scope).
    // Returns null for every non-bidi (road) lane, so callers stay inert for road scenarios.
    public string? TryGetBidiLaneId(string laneId)
    {
        if (!LanesById.TryGetValue(laneId, out var lane))
        {
            return null;
        }

        if (!EdgesById.TryGetValue(lane.EdgeId, out var edge) || edge.BidiEdgeId is null)
        {
            return null;
        }

        if (!EdgesById.TryGetValue(edge.BidiEdgeId, out var bidiEdge))
        {
            return null;
        }

        foreach (var bidiLane in bidiEdge.Lanes)
        {
            if (bidiLane.Index == lane.Index)
            {
                return bidiLane.Id;
            }
        }

        return null;
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
    public IReadOnlyList<string> ResolveLaneSequence(IReadOnlyList<string> routeEdges, int departLaneIndex) =>
        ResolveSequenceCore(routeEdges, departLaneIndex).Select(p => p.Exit).ToList();

    // C2-v: the ordered lane sequence, resolved as (Exit, Arrival) pairs per slot.
    //  * Exit  = the lane the vehicle must be on to CONTINUE its route from this edge (the onward
    //            connection's fromLane) -- this is the routing "pool" the strategic lane change
    //            converges the vehicle onto and that the next hop's connection is taken from.
    //  * Arrival = the lane the vehicle physically OCCUPIES on entering this edge (the incoming
    //            connection's toLane; the depart lane on the first edge). For an internal (via)
    //            lane and for any edge whose arrival lane already connects onward, Arrival == Exit.
    //
    // They differ ONLY at an intra-edge mid-route lane change (C2-v): the vehicle arrives on lane A
    // (fixed by the incoming connection) but the only onward connection to the next route edge
    // leaves from a sibling lane B, so it must lane-change A->B while traversing this edge. This is
    // the multi-lane generalization of the departure-edge redirect (a vehicle departing a drop lane
    // that doesn't continue), which SUMO models uniformly: `bestLaneOffset` is a PER-EDGE quantity
    // and the arrival lane and exit lane on one edge legitimately differ (MSVehicle::updateBestLanes,
    // MSVehicle.cpp:5744-6063). For every route whose arrival lane already continues at every hop
    // (every pre-C2-v scenario), Arrival == Exit at every slot, so the resolved Exit sequence -- and
    // therefore the routing pool -- is byte-identical to before.
    // C4-vii-c: `forceFirstExitToArrival` pins the FIRST edge's exit lane to the depart/arrival lane
    // instead of steering toward bestLaneOffset. Used by ExecuteMoves' boundary re-resolution: a
    // vehicle that reached a lane end on a lane that isn't the pool's resolved exit -- but that STILL
    // connects onward -- is, by definition, leaving THIS edge via THAT lane's connection (it is at the
    // boundary; no lane change is possible now). SUMO models exactly this: it follows whatever
    // connection leaves the current lane rather than forcing the resolved exit (MSVehicle::
    // updateBestLanes' bestLaneOffset is a HINT it lane-changes toward opportunistically, not a hard
    // gate). Pinning exit == arrival makes the re-resolved pool start ON the vehicle's actual lane so
    // the crossing advances via that lane's own connection. Only ever passed true for that one
    // boundary case; the default (false) is byte-identical to every existing caller.
    private List<(string Exit, string Arrival)> ResolveSequenceCore(IReadOnlyList<string> routeEdges, int departLaneIndex, bool forceFirstExitToArrival = false)
    {
        var result = new List<(string Exit, string Arrival)>();
        // The lane the vehicle physically occupies on the current edge (depart lane on edge 0, then
        // each hop's incoming connection toLane).
        var arrivalIndex = departLaneIndex;

        // L0c (PERF-ROADMAP.md): compute every edge's route-wide LaneQ in ONE backward pass up front,
        // rather than re-running the whole backward pass inside the loop for each edge and each hop
        // (the former O(N^2) ComputeBestLanes calls, each allocating per-edge collections). Lookups
        // below are byte-identical to ComputeBestLanes(routeEdges, edgeId) -- see ComputeAllBestLanes.
        var bestByEdge = ComputeAllBestLanes(routeEdges);

        for (var i = 0; i < routeEdges.Count; i++)
        {
            var edgeId = routeEdges[i];
            var edge = EdgesById[edgeId];
            var isLast = i == routeEdges.Count - 1;

            // Determine this edge's EXIT lane. THE INVARIANT: the exit lane MUST have a <connection>
            // to the route's next edge (else the vehicle physically cannot make the hop). Among the
            // lanes that connect, steer toward the route-wide best-continuing lane via `bestLaneOffset`
            // (MSVehicle::updateBestLanes) -- the offset is a per-edge downstream HINT, not a hard
            // target. Two failure modes this must reconcile:
            //   * The arrival lane connects to the next edge but that path dead-ends further along, so
            //     the offset points to a BETTER-continuing sibling that ALSO connects (scenario 36:
            //     E0_0->E1_0 connects but dead-ends; E0_1->E1_1 continues; offset +1 -> exit E0_1).
            //   * The offset points to a sibling that does NOT connect to the next edge, because it is
            //     a hint for a change to make on a LATER edge (C2-vi: C1B1 lane 0 is the only lane
            //     connecting to B1B2, yet inherits offset +1 from B1B2's downstream best lane; the
            //     +1 target (lane 1) doesn't connect to B1B2, so the vehicle must STAY on lane 0 here
            //     and change on B1B2). SUMO never resolves a fixed exit lane -- it follows whatever
            //     connection leaves its current lane and lane-changes opportunistically toward
            //     bestLaneOffset; picking the CONNECTING lane closest to (arrival + offset) reproduces
            //     that exit-lane choice.
            // The last edge has no onward connection, so exit == arrival.
            int exitIndex;
            if (isLast)
            {
                exitIndex = arrivalIndex;
            }
            else
            {
                var nextEdgeId = routeEdges[i + 1];
                var best = bestByEdge[edgeId];
                var offset = best.First(q => q.LaneIndex == arrivalIndex).BestLaneOffset;
                var target = arrivalIndex + offset;

                // Lanes on this edge that actually connect to the route's next edge.
                var connecting = edge.Lanes
                    .Where(l => ConnectionsByFromEdgeLane.TryGetValue((edgeId, l.Index), out var cc)
                                && cc.Any(c => c.To == nextEdgeId))
                    .Select(l => l.Index)
                    .ToList();

                if (connecting.Count == 0)
                {
                    throw new InvalidDataException(
                        $"No lane on edge '{edgeId}' has a <connection> to '{nextEdgeId}' (route unroutable).");
                }

                // C4-vii-c: the boundary re-resolution pins the first edge's exit to the vehicle's
                // actual lane (which the caller has already verified connects onward) -- see this
                // method's `forceFirstExitToArrival` header. Otherwise choose the connecting lane
                // nearest the bestLaneOffset target; on a tie, the lower index (SIMPLIFICATION,
                // documented: no committed scenario has two equidistant connecting lanes with
                // differing downstream quality -- when they do, a betterContinuation rank would be
                // more faithful, but every anchor here has a unique nearest connecting lane).
                exitIndex = forceFirstExitToArrival && i == 0
                    ? arrivalIndex
                    : connecting
                        .OrderBy(l => Math.Abs(l - target))
                        .ThenBy(l => l)
                        .First();
            }

            var arrivalLane = edge.Lanes.First(l => l.Index == arrivalIndex);
            var exitLane = edge.Lanes.First(l => l.Index == exitIndex);
            result.Add((exitLane.Id, arrivalLane.Id));

            if (isLast)
            {
                break;
            }

            // Advance to the next edge via the EXIT lane's onward connection.
            var toEdgeId = routeEdges[i + 1];
            var candidates = ConnectionsByFromEdgeLane.TryGetValue((edgeId, exitIndex), out var conns)
                ? conns.Where(c => c.To == toEdgeId).ToList()
                : new List<Connection>();

            if (candidates.Count == 0)
            {
                throw new InvalidDataException(
                    $"No <connection> found from edge '{edgeId}' lane {exitIndex} to edge '{toEdgeId}'.");
            }

            Connection connection;
            if (candidates.Count == 1)
            {
                // Single onward connection: byte-identical to the pre-C2-iii lookup.
                connection = candidates[0];
            }
            else
            {
                // C2-iii: multiple onward connections from this lane -- thread the pool through the
                // ToLane with the best route-wide continuation (SUMO's bestConnectedNext /
                // betterContinuation: longest Length, then least |offset|, then lowest index), so the
                // resolved lane sequence matches updateBestLanes' bestContinuations.
                var toBest = bestByEdge[toEdgeId];
                var toByIndex = toBest.ToDictionary(q => q.LaneIndex);
                connection = candidates
                    .OrderByDescending(c => toByIndex[c.ToLane].Length)
                    .ThenBy(c => Math.Abs(toByIndex[c.ToLane].BestLaneOffset))
                    .ThenBy(c => c.ToLane)
                    .First();
            }

            if (connection.Via is { } via)
            {
                // Internal (via) lane: arrival == exit (a vehicle never intra-edge-changes on a
                // one-lane junction interior in this rung's scope).
                result.Add((via, via));

                // C4-vii-a: follow the internal VIA-CHAIN. A `cont` link (a turn split by an INTERNAL
                // junction) traverses TWO+ internal lanes -- e.g. NC->CE = :C_3_0 then :C_16_0 -- but
                // the NC->CE connection lists only the FIRST via (:C_3_0); the continuation :C_3->CE
                // lists via :C_16_0, and the final :C_16->CE hop has no via. Walk that chain so the pool
                // includes EVERY internal lane the vehicle physically crosses (SUMO's getViaLaneOrLane /
                // MSLane internal-following chain), not just the first. This is load-bearing beyond the
                // trajectory: :C_16_0 is the lane in junction C's <request>/IntLanes for this link, so
                // without it JunctionYieldConstraint's egoInternalLane scan can't even find ego's link
                // (LinkByInternalLane has :C_16_0, not the intermediate :C_3_0) and no cautious approach
                // / RoW fires. INERT for single-internal-lane junctions (every committed green scenario):
                // the first via already reaches the exit edge, so this loop never iterates.
                var chainVia = via;
                for (var guard = 0; guard < 8; guard++)
                {
                    var chainLane = LanesById[chainVia];
                    if (!ConnectionsByFromEdgeLane.TryGetValue((chainLane.EdgeId, chainLane.Index), out var chainConns))
                    {
                        break;
                    }

                    Connection? nextHop = null;
                    foreach (var c in chainConns)
                    {
                        if (c.To == toEdgeId && c.Via is not null)
                        {
                            nextHop = c;
                            break;
                        }
                    }

                    if (nextHop?.Via is not { } nextVia)
                    {
                        break;
                    }

                    result.Add((nextVia, nextVia));
                    chainVia = nextVia;
                }
            }

            arrivalIndex = connection.ToLane;
        }

        return result;
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

    // C2-v: the handle-parallel Exit (routing pool) AND Arrival sequences together. `Pool[k]` is the
    // lane the vehicle must be on to continue (strategic-LC target + onward-connection source);
    // `Arrival[k]` is the lane it physically occupies on entering that slot's edge (== Pool[k] for
    // every slot except an intra-edge mid-route lane change). The crossing code sets the vehicle's
    // actual lane to Arrival[k] on entering a slot and the strategic lane change converges it onto
    // Pool[k] -- so for every route with no intra-edge change (Arrival == Pool everywhere) this is
    // byte-identical to ResolveLaneSequenceHandles.
    public (int[] Pool, int[] Arrival) ResolveLaneSequenceHandlesWithArrival(IReadOnlyList<string> routeEdges, int departLaneIndex, bool forceFirstExitToArrival = false)
    {
        var sequence = ResolveSequenceCore(routeEdges, departLaneIndex, forceFirstExitToArrival);
        var pool = new int[sequence.Count];
        var arrival = new int[sequence.Count];
        for (var i = 0; i < sequence.Count; i++)
        {
            pool[i] = LaneHandleById[sequence[i].Exit];
            arrival[i] = LaneHandleById[sequence[i].Arrival];
        }

        return (pool, arrival);
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

        // C2-iii: the full ROUTE-WIDE backward pass (MSVehicle::updateBestLanes,
        // MSVehicle.cpp:6003-6063), replacing C2-i's single-look-ahead. Build the per-edge LaneQ
        // from the LAST route edge back to `currentEdgeId`: each lane's Length accumulates the best
        // reachable downstream continuation, and BestLaneOffset steers -- across multiple junctions
        // -- toward a lane that stays connected to the END of the route (so a route whose insertion
        // lane is dictated by a connection two+ hops out resolves instead of dead-ending).
        //
        // Base of the recursion -- the last route edge (MSVehicle.cpp:5951-5989): every lane
        // trivially continues, Length = its own length, offset 0. (SIMPLIFICATION: SUMO's terminal
        // edge can assign nonzero offsets when its lanes differ in length; every committed anchor's
        // terminal lanes are equal-length, so offset 0 is exact here.)
        var lastIndex = routeEdges.Count - 1;
        var nextQ = EdgesById[routeEdges[lastIndex]].Lanes
            .OrderBy(l => l.Index)
            .Select(l => new LaneContinuation(l.Index, AllowsContinuation: true, BestLaneOffset: 0, l.Length))
            .ToList();

        for (var e = lastIndex - 1; e >= edgeIndex; e--)
        {
            nextQ = BackwardPassEdge(routeEdges[e], routeEdges[e + 1], nextQ);
        }

        return nextQ;
    }

    // L0c (PERF-ROADMAP.md): the route-wide form of ComputeBestLanes -- ONE backward pass from the
    // last route edge down to the first, CAPTURING every edge's LaneQ instead of discarding the
    // intermediates. ResolveSequenceCore formerly called ComputeBestLanes once per edge (for the
    // exit-lane offset) AND once per multi-connection hop (for the ToLane ranking); every one of
    // those calls re-ran the whole backward pass, so a route of N edges cost O(N^2) BackwardPassEdge
    // invocations at insertion -- and each BackwardPassEdge allocates a fistful of per-edge
    // collections. This runs the pass ONCE (O(N)) and lets ResolveSequenceCore look each edge's
    // LaneQ up by id.
    //
    // BYTE-IDENTICAL to ComputeBestLanes: for an edge id `e`, ComputeBestLanes(routeEdges, e) threads
    // the base LaneQ through BackwardPassEdge from lastIndex-1 down to the FIRST index at which `e`
    // appears and returns that. This single pass threads the exact same nextQ through the exact same
    // BackwardPassEdge calls in the same order, so `all[firstIndexOf(e)]` equals what
    // ComputeBestLanes(routeEdges, e) returns. The dictionary keeps the FIRST occurrence per id,
    // preserving ComputeBestLanes' first-match `edgeIndex` semantics for a route that repeats an edge.
    private Dictionary<string, IReadOnlyList<LaneContinuation>> ComputeAllBestLanes(IReadOnlyList<string> routeEdges)
    {
        // An empty route has no edges to resolve; return an empty map so ResolveSequenceCore's loop
        // simply doesn't run (byte-identical to the pre-L0c behavior, which never entered the loop).
        // Not reached by any vehicle -- a real route always has >=1 edge -- but keeps the degenerate
        // case from indexing routeEdges[-1].
        if (routeEdges.Count == 0)
        {
            return new Dictionary<string, IReadOnlyList<LaneContinuation>>();
        }

        var lastIndex = routeEdges.Count - 1;
        var all = new IReadOnlyList<LaneContinuation>[routeEdges.Count];

        var nextQ = EdgesById[routeEdges[lastIndex]].Lanes
            .OrderBy(l => l.Index)
            .Select(l => new LaneContinuation(l.Index, AllowsContinuation: true, BestLaneOffset: 0, l.Length))
            .ToList();
        all[lastIndex] = nextQ;

        for (var e = lastIndex - 1; e >= 0; e--)
        {
            nextQ = BackwardPassEdge(routeEdges[e], routeEdges[e + 1], nextQ);
            all[e] = nextQ;
        }

        var byEdge = new Dictionary<string, IReadOnlyList<LaneContinuation>>();
        for (var i = 0; i < routeEdges.Count; i++)
        {
            if (!byEdge.ContainsKey(routeEdges[i]))
            {
                byEdge[routeEdges[i]] = all[i];
            }
        }

        return byEdge;
    }

    // C2-iii: one backward step of MSVehicle::updateBestLanes -- compute edge `edgeId`'s route-wide
    // LaneQ from the already-computed downstream edge's LaneQ (`nextQ`). SIMPLIFICATIONS (documented,
    // matching the anchor scenarios/36-multihop-lanes -- none exercised here): no bidi-lane
    // preference (betterContinuation's bidi arms), no `nextLinkPriority` tie-break (only reached when
    // two continuations have EQUAL length AND equal |offset|), no vClass permission-change handling,
    // no elecHybrid overhead-wire preference. The `bestConnectedLength <= 0` disconnected-route arm
    // (MSVehicle.cpp:6078-6098) is likewise not reached by a route SUMO itself routes.
    private List<LaneContinuation> BackwardPassEdge(string edgeId, string nextEdgeId, List<LaneContinuation> nextQ)
    {
        var lanes = EdgesById[edgeId].Lanes.OrderBy(l => l.Index).ToList();
        var nextByIndex = nextQ.ToDictionary(q => q.LaneIndex);

        // Forward init (MSVehicle.cpp:5896-5918): the ToLanes each lane connects to on `nextEdgeId`,
        // `allows` = it has any such connection, initial `length` = the lane's own length.
        var toLanes = new List<int>[lanes.Count];
        var allows = new bool[lanes.Count];
        var length = new double[lanes.Count];
        var offset = new int[lanes.Count];
        var reachableNext = new HashSet<int>();
        for (var i = 0; i < lanes.Count; i++)
        {
            toLanes[i] = ConnectionsByFromEdgeLane.TryGetValue((edgeId, lanes[i].Index), out var cands)
                ? cands.Where(c => c.To == nextEdgeId).Select(c => c.ToLane).Distinct().ToList()
                : new List<int>();
            allows[i] = toLanes[i].Count > 0;
            length[i] = lanes[i].Length;
            offset[i] = 0;
            foreach (var tl in toLanes[i])
            {
                reachableNext.Add(tl);
            }
        }

        // bestConnectedLength = the longest downstream lane reachable from THIS edge; bestLength =
        // the longest downstream lane overall (MSVehicle.cpp:6012-6019).
        var bestConnectedLength = -1.0;
        var bestLength = -1.0;
        foreach (var q in nextQ)
        {
            if (reachableNext.Contains(q.LaneIndex) && bestConnectedLength < q.Length)
            {
                bestConnectedLength = q.Length;
            }

            if (bestLength < q.Length)
            {
                bestLength = q.Length;
            }
        }

        if (bestConnectedLength > 0)
        {
            for (var i = 0; i < lanes.Count; i++)
            {
                if (!allows[i])
                {
                    continue;
                }

                // bestConnectedNext = the best downstream lane this lane connects to
                // (betterContinuation, MSVehicle.cpp:5940-5949: longest length, then least |offset|).
                LaneContinuation? bcn = null;
                foreach (var tl in toLanes[i])
                {
                    var m = nextByIndex[tl];
                    if (bcn is null
                        || bcn.Length < m.Length
                        || (bcn.Length == m.Length && Math.Abs(bcn.BestLaneOffset) > Math.Abs(m.BestLaneOffset)))
                    {
                        bcn = m;
                    }
                }

                if (bcn is not null)
                {
                    // MSVehicle.cpp:6038-6043: keep this lane on the globally-best downstream lane
                    // when it can reach it (length == bestConnectedLength, near-straight offset),
                    // else accumulate its own best continuation's length.
                    length[i] += bcn.Length == bestConnectedLength && Math.Abs(bcn.BestLaneOffset) < 2
                        ? bestLength
                        : bcn.Length;
                    offset[i] = bcn.BestLaneOffset;

                    // MSVehicle.cpp:6046-6050: the continuation dies only if the best downstream lane
                    // neither continues nor has any length of its own.
                    if (!(bcn.AllowsContinuation || bcn.Length > 0))
                    {
                        allows[i] = false;
                    }
                }
                else
                {
                    allows[i] = false;
                }
            }
        }

        // bestThisIndex = the lane with the greatest route-wide length (tie: least |offset|);
        // bestThisMaxIndex = the highest-index lane sharing that length and |offset|
        // (MSVehicle.cpp:6027-6033).
        var bestThis = 0;
        for (var i = 1; i < lanes.Count; i++)
        {
            if (length[i] > length[bestThis]
                || (length[i] == length[bestThis] && Math.Abs(offset[i]) < Math.Abs(offset[bestThis])))
            {
                bestThis = i;
            }
        }

        var bestThisMax = bestThis;
        for (var i = 0; i < lanes.Count; i++)
        {
            if (length[i] == length[bestThis] && Math.Abs(offset[i]) == Math.Abs(offset[bestThis]))
            {
                bestThisMax = i;
            }
        }

        // Final offset (MSVehicle.cpp:6103-6115): every lane worse than the best is steered toward
        // it, choosing the nearer of the lowest/highest best index.
        for (var i = 0; i < lanes.Count; i++)
        {
            if (length[i] < length[bestThis]
                || (length[i] == length[bestThis] && Math.Abs(offset[i]) > Math.Abs(offset[bestThis])))
            {
                var toLow = lanes[bestThis].Index - lanes[i].Index;
                var toHigh = lanes[bestThisMax].Index - lanes[i].Index;
                offset[i] = Math.Abs(toLow) < Math.Abs(toHigh) ? toLow : toHigh;
            }
        }

        var result = new List<LaneContinuation>(lanes.Count);
        for (var i = 0; i < lanes.Count; i++)
        {
            result.Add(new LaneContinuation(lanes[i].Index, allows[i], offset[i], length[i]));
        }

        return result;
    }
}
