using System.Globalization;
using System.Xml.Linq;

namespace Sim.Ingest;

// Parses the rung-1 subset of SUMO's post-netconvert .net.xml: <edge> containing one or more
// <lane>, plus (rung 9a) internal (junction-interior) edges/lanes and top-level <connection>
// elements. Tolerant of missing optional attributes (documented defaults below); required
// attributes throw a clear error rather than silently defaulting, since a missing id/shape
// signals a parser-subset gap, not a legitimate omission.
public static class NetworkParser
{
    // sumo/src/utils/common/StdDefs.h:48 -- #define SUMO_const_laneWidth 3.2. Default lane
    // width when a <lane>'s `width` attribute is absent (rung 9b-ii).
    private const double SumoConstLaneWidth = 3.2;

    public static NetworkModel Parse(string path)
    {
        using var stream = File.OpenRead(path);
        return ParseDocument(XDocument.Load(stream));
    }

    public static NetworkModel ParseXml(string xml) => ParseDocument(XDocument.Parse(xml));

    private static NetworkModel ParseDocument(XDocument doc)
    {
        var root = doc.Root ?? throw new InvalidDataException("net.xml has no root element.");

        var edges = new List<Edge>();
        var edgesById = new Dictionary<string, Edge>();
        var lanesById = new Dictionary<string, Lane>();

        // D2: the global dense lane-handle assignment -- every lane (including internal `:`
        // lanes) gets the next sequential handle in PARSE order, so LanesByHandle[h] == the
        // lane whose Handle is h. nextLaneHandle is shared across every <edge>'s <lane> loop
        // below (it is NOT reset per edge).
        var nextLaneHandle = 0;
        var lanesByHandle = new List<Lane>();

        foreach (var edgeEl in root.Elements("edge"))
        {
            // Rung 9a: internal (junction-interior) edges are now parsed too -- a multi-edge
            // route's lane sequence passes through them (e.g. ":J_2_0"). They have no from/to
            // (tolerated: From/To default to "" below, same as any edge missing the attribute).
            var edgeId = RequireAttribute(edgeEl, "id");
            var from = edgeEl.Attribute("from")?.Value ?? string.Empty;
            var to = edgeEl.Attribute("to")?.Value ?? string.Empty;

            var lanes = new List<Lane>();
            foreach (var laneEl in edgeEl.Elements("lane"))
            {
                // Rung 9b-ii: `width` defaults to SUMO_const_laneWidth (3.2, StdDefs.h:48) when
                // absent -- this net's <lane> elements never specify it.
                var width = laneEl.Attribute("width") is { } widthAttr
                    ? double.Parse(widthAttr.Value, CultureInfo.InvariantCulture)
                    : SumoConstLaneWidth;

                var lane = new Lane(
                    Id: RequireAttribute(laneEl, "id"),
                    EdgeId: edgeId,
                    Index: int.Parse(RequireAttribute(laneEl, "index"), CultureInfo.InvariantCulture),
                    Speed: double.Parse(RequireAttribute(laneEl, "speed"), CultureInfo.InvariantCulture),
                    Length: double.Parse(RequireAttribute(laneEl, "length"), CultureInfo.InvariantCulture),
                    Shape: ParseShape(RequireAttribute(laneEl, "shape")),
                    Width: width,
                    Handle: nextLaneHandle++);

                lanes.Add(lane);
                lanesById[lane.Id] = lane;
                lanesByHandle.Add(lane);
            }

            // D4: precompute each lane's same-edge left/right neighbor HANDLE (Index+1/Index-1)
            // once here, at ingest -- cold path (parse time, O(n^2) over one edge's small lane
            // count), so the per-step keep-right/speed-gain decision (Engine.cs) never has to
            // scan `edge.Lanes` itself. `lanesById`/`lanesByHandle` are updated in place (via
            // `with`) since `Lane` is immutable and neighbor handles are only knowable once every
            // lane on this edge has already been assigned its own Handle above.
            for (var i = 0; i < lanes.Count; i++)
            {
                var lane = lanes[i];
                var leftHandle = -1;
                var rightHandle = -1;
                foreach (var sibling in lanes)
                {
                    if (sibling.Index == lane.Index + 1)
                    {
                        leftHandle = sibling.Handle;
                    }
                    else if (sibling.Index == lane.Index - 1)
                    {
                        rightHandle = sibling.Handle;
                    }
                }

                if (leftHandle != -1 || rightHandle != -1)
                {
                    var updated = lane with { LeftNeighbor = leftHandle, RightNeighbor = rightHandle };
                    lanes[i] = updated;
                    lanesById[updated.Id] = updated;
                    lanesByHandle[updated.Handle] = updated;
                }
            }

            var edge = new Edge(edgeId, from, to, lanes);
            edges.Add(edge);
            edgesById[edgeId] = edge;
        }

        var connections = new List<Connection>();
        var connectionsByFromLaneTo = new Dictionary<(string, int, string), Connection>();
        var connectionsByFromEdgeLane = new Dictionary<(string, int), List<Connection>>();

        foreach (var connEl in root.Elements("connection"))
        {
            // A <connection>'s from/to are always present in netconvert output; fromLane/toLane
            // default to "0" only for parser robustness (every connection in scope for this
            // rung specifies them explicitly). `via` (the internal lane traversed at a
            // junction) is absent for connections that cross no junction interior. Rung 10:
            // `tl`/`linkIndex` are present together only on connections controlled by a
            // <tlLogic> (e.g. this scenario's WJ->JE connection); absent (null) otherwise.
            var from = RequireAttribute(connEl, "from");
            var to = RequireAttribute(connEl, "to");
            var fromLane = int.Parse(connEl.Attribute("fromLane")?.Value ?? "0", CultureInfo.InvariantCulture);
            var toLane = int.Parse(connEl.Attribute("toLane")?.Value ?? "0", CultureInfo.InvariantCulture);
            var via = connEl.Attribute("via")?.Value;
            var tl = connEl.Attribute("tl")?.Value;
            var linkIndex = connEl.Attribute("linkIndex") is { } linkIndexAttr
                ? int.Parse(linkIndexAttr.Value, CultureInfo.InvariantCulture)
                : (int?)null;

            var connection = new Connection(from, fromLane, to, toLane, via, tl, linkIndex);
            connections.Add(connection);
            // Last-wins on a duplicate key is a non-issue for this rung's straight-through,
            // single-connection-per-(fromEdge,fromLane,toEdge) network.
            connectionsByFromLaneTo[(from, fromLane, to)] = connection;

            if (!connectionsByFromEdgeLane.TryGetValue((from, fromLane), out var list))
            {
                list = new List<Connection>();
                connectionsByFromEdgeLane[(from, fromLane)] = list;
            }

            list.Add(connection);
        }

        var tlLogicsById = new Dictionary<string, TlLogic>();
        foreach (var tlLogicEl in root.Elements("tlLogic"))
        {
            var id = RequireAttribute(tlLogicEl, "id");
            var offset = double.Parse(tlLogicEl.Attribute("offset")?.Value ?? "0", CultureInfo.InvariantCulture);

            var phases = new List<TlPhase>();
            foreach (var phaseEl in tlLogicEl.Elements("phase"))
            {
                var duration = double.Parse(RequireAttribute(phaseEl, "duration"), CultureInfo.InvariantCulture);
                var state = RequireAttribute(phaseEl, "state");
                phases.Add(new TlPhase(duration, state));
            }

            // Last-wins on a duplicate id (multiple <tlLogic> programs for the same junction id,
            // e.g. an alternate programID) is a non-issue for this rung's single-program network.
            tlLogicsById[id] = new TlLogic(id, offset, phases);
        }

        var connectionsByFromEdgeLaneReadOnly = connectionsByFromEdgeLane
            .ToDictionary(kvp => kvp.Key, kvp => (IReadOnlyList<Connection>)kvp.Value);

        var junctions = new List<Junction>();
        var junctionsById = new Dictionary<string, Junction>();
        var linkByInternalLane = new Dictionary<string, (Junction Junction, JunctionLink Link)>();

        foreach (var junctionEl in root.Elements("junction"))
        {
            var junction = ParseJunction(junctionEl, connections, lanesById);
            junctions.Add(junction);
            junctionsById[junction.Id] = junction;

            foreach (var link in junction.Links)
            {
                linkByInternalLane[link.InternalLaneId] = (junction, link);
            }
        }

        // D2: LaneHandleById mirrors lanesById's keys 1:1 (every lane got exactly one handle
        // above), just projecting Id -> Handle instead of Id -> Lane.
        var laneHandleById = new Dictionary<string, int>(lanesById.Count, StringComparer.Ordinal);
        foreach (var lane in lanesByHandle)
        {
            laneHandleById[lane.Id] = lane.Handle;
        }

        return new NetworkModel(
            edges,
            edgesById,
            lanesById,
            connections,
            connectionsByFromLaneTo,
            tlLogicsById,
            connectionsByFromEdgeLaneReadOnly,
            junctions,
            junctionsById,
            linkByInternalLane,
            lanesByHandle,
            laneHandleById);
    }

    // Rung 9b-i: parses one <junction> -- id/type/intLanes are always present (netconvert
    // output); only junctions with a nonempty intLanes AND at least one child <request> get a
    // populated Links/Requests/Conflicts (dead_end/internal junctions have neither and parse to
    // empty lists, which is harmless -- see Junction's doc comment).
    private static Junction ParseJunction(
        XElement junctionEl,
        IReadOnlyList<Connection> connections,
        IReadOnlyDictionary<string, Lane> lanesById)
    {
        var id = RequireAttribute(junctionEl, "id");
        var type = RequireAttribute(junctionEl, "type");
        var intLanesAttr = junctionEl.Attribute("intLanes")?.Value ?? string.Empty;
        var intLanes = intLanesAttr.Length == 0
            ? new List<string>()
            : intLanesAttr.Split(' ', StringSplitOptions.RemoveEmptyEntries).ToList();

        var requestEls = junctionEl.Elements("request").ToList();
        if (intLanes.Count == 0 || requestEls.Count == 0)
        {
            return new Junction(id, type, intLanes, Array.Empty<JunctionLink>(), Array.Empty<JunctionRequest>(),
                Array.Empty<JunctionConflict>(), Array.Empty<MergeConflict>());
        }

        // Links: for each link index i, the top-level <connection> whose `via` equals
        // intLanes[i] (the incoming-lane -> outgoing-lane move this link represents). A missing
        // match (shouldn't happen for a real request row) is skipped gracefully -- the request
        // row is kept regardless, so right-of-way bits are never silently dropped.
        var links = new List<JunctionLink>();
        var linksByIndex = new Dictionary<int, JunctionLink>();
        for (var i = 0; i < intLanes.Count; i++)
        {
            var internalLaneId = intLanes[i];
            var connection = connections.FirstOrDefault(c => c.Via == internalLaneId);
            if (connection is null)
            {
                continue;
            }

            var link = new JunctionLink(i, internalLaneId, connection);
            links.Add(link);
            linksByIndex[i] = link;
        }

        var requests = new List<JunctionRequest>();
        var requestsByIndex = new Dictionary<int, JunctionRequest>();
        foreach (var requestEl in requestEls)
        {
            var index = int.Parse(RequireAttribute(requestEl, "index"), CultureInfo.InvariantCulture);
            var response = RequireAttribute(requestEl, "response");
            var foes = RequireAttribute(requestEl, "foes");
            var cont = (requestEl.Attribute("cont")?.Value ?? "0") == "1";

            var request = new JunctionRequest(index, response, foes, cont);
            requests.Add(request);
            requestsByIndex[index] = request;
        }

        // Conflicts: for every ordered pair (i, j), i != j, where link i's request marks link j
        // as a physical foe and both links resolved to an internal lane, compute the crossing
        // of the two internal-lane shapes (see PolylineGeometry's doc comment for what counts
        // as a "crossing" -- merges that only share an endpoint are skipped).
        var conflicts = new List<JunctionConflict>();
        // C4-v: sameTarget MERGE geometry (per ego link) -- the two links whose connections feed the
        // same downstream lane converge instead of crossing, so they produce a MergeConflict (lbc/
        // flbc) rather than a JunctionConflict.
        var merges = new List<MergeConflict>();
        foreach (var request in requests)
        {
            if (!linksByIndex.TryGetValue(request.Index, out var egoLink))
            {
                continue;
            }

            for (var j = 0; j < intLanes.Count; j++)
            {
                if (j == request.Index || !request.FoeWith(j))
                {
                    continue;
                }

                if (!linksByIndex.TryGetValue(j, out var foeLink))
                {
                    continue;
                }

                var egoLane = lanesById[egoLink.InternalLaneId];
                var foeLane = lanesById[foeLink.InternalLaneId];

                // C4-v: a sameTarget MERGE (connections share the destination edge + lane) does NOT
                // cross (TryIntersect below fails, they touch only at the shared end) -- compute its
                // lengthBehindCrossing from MSLink::setRequestInformation's sameTarget arm instead.
                if (egoLink.Connection.To == foeLink.Connection.To
                    && egoLink.Connection.ToLane == foeLink.Connection.ToLane)
                {
                    // minDist = MIN2(DIVERGENCE_MIN_WIDTH=2.5, 0.5*(egoW+foeW)) (MSLink.cpp:306).
                    var minDist = Math.Min(2.5, 0.5 * (egoLane.Width + foeLane.Width));
                    double egoLbc;
                    double foeLbc;
                    // Lanes ending >= minDist apart => CONFLICT_DUMMY_MERGE (lbc 0); else compute the
                    // divergence point (MSLink.cpp:307-330). computeDistToDivergence is symmetric in
                    // its (lane, sibling) roles, so one call serves both lanes' lbc via their own
                    // length/shape factor (InterpolateGeometryPosToLanePos = geomPos * laneLen / shapeLen).
                    var egoEnd = egoLane.Shape[^1];
                    var foeEnd = foeLane.Shape[^1];
                    if (Math.Sqrt(((egoEnd.X - foeEnd.X) * (egoEnd.X - foeEnd.X)) + ((egoEnd.Y - foeEnd.Y) * (egoEnd.Y - foeEnd.Y))) >= minDist)
                    {
                        egoLbc = 0.0;
                        foeLbc = 0.0;
                    }
                    else
                    {
                        var dtd = PolylineGeometry.ComputeDistToDivergence(
                            egoLane.Shape, foeLane.Shape, egoLane.Length, foeLane.Length, minDist);
                        egoLbc = dtd * egoLane.Length / PolylineGeometry.PolylineLength(egoLane.Shape);
                        foeLbc = dtd * foeLane.Length / PolylineGeometry.PolylineLength(foeLane.Shape);
                    }

                    merges.Add(new MergeConflict(egoLink.Index, foeLink.Index, egoLbc, foeLbc));
                    continue;
                }

                if (PolylineGeometry.TryIntersect(egoLane.Shape, foeLane.Shape, out var intersection))
                {
                    // Rung 9b-ii: MSLink.cpp:358-366 -- widthFactor widens (or leaves unchanged)
                    // the conflict size for shallow-angle crossings; angleDiff is the acute angle
                    // between the two internal lanes' travel DIRECTIONS at the crossing
                    // (GeomHelper::getMinAngleDiff, folded to [0,90] for these straight lanes).
                    var egoDirection = LaneDirection(egoLane.Shape);
                    var foeDirection = LaneDirection(foeLane.Shape);
                    var angleDiffDeg = MinAngleDiffDegrees(egoDirection, foeDirection);
                    var widthFactor = (1.0 / Math.Max(Math.Sin(DegToRad(angleDiffDeg)), 0.2) * 2.0) - 1.0;

                    // MSLink.cpp:365-366/380-382: conflictSize = MIN2(foeLane->getWidth() *
                    // widthFactor, lane->getLength()); myConflicts.push_back(ConflictInfo(
                    // lane->getLength() - MAX2(0, crossingArc - conflictSize/2), conflictSize)).
                    // Each conflict record here is built once per (ego, foe) ordered pair, so the
                    // "ego" / "foe" roles below always match this record's own EgoLink/FoeLink.
                    var egoConflictSize = Math.Min(foeLane.Width * widthFactor, egoLane.Length);
                    var foeConflictSize = Math.Min(egoLane.Width * widthFactor, foeLane.Length);
                    var egoLengthBehindCrossing = egoLane.Length - Math.Max(0.0, intersection.ArcA - (egoConflictSize / 2.0));
                    var foeLengthBehindCrossing = foeLane.Length - Math.Max(0.0, intersection.ArcB - (foeConflictSize / 2.0));

                    conflicts.Add(new JunctionConflict(
                        egoLink.Index, foeLink.Index,
                        intersection.ArcA, intersection.ArcB,
                        intersection.Point,
                        egoConflictSize, foeConflictSize,
                        egoLengthBehindCrossing, foeLengthBehindCrossing));
                }
            }
        }

        return new Junction(id, type, intLanes, links, requests, conflicts, merges);
    }

    // Rung 9b-ii: a straight 2-point internal lane's travel direction, normalized -- ported
    // from the (unit-vector) reading of GeomHelper::naviDegree(shape.rotationAtOffset(...)) for
    // the straight-through internal lanes this scenario has (no curved internal-lane shapes are
    // in scope here).
    private static (double X, double Y) LaneDirection(IReadOnlyList<(double X, double Y)> shape)
    {
        var first = shape[0];
        var last = shape[^1];
        var dx = last.X - first.X;
        var dy = last.Y - first.Y;
        var length = Math.Sqrt((dx * dx) + (dy * dy));
        return (dx / length, dy / length);
    }

    // Ported from GeomHelper::getMinAngleDiff (sumo/src/utils/geom/GeomHelper.cpp) for the two
    // straight lanes this rung's net has: the acute angle between two direction vectors, folded
    // to [0, 90] degrees via acos(|dot|/(|a||b|)) -- equivalent to getMinAngleDiff's own
    // fmod/180-wrap for this scenario's perpendicular/straight crossing.
    private static double MinAngleDiffDegrees((double X, double Y) a, (double X, double Y) b)
    {
        var dot = (a.X * b.X) + (a.Y * b.Y);
        var magA = Math.Sqrt((a.X * a.X) + (a.Y * a.Y));
        var magB = Math.Sqrt((b.X * b.X) + (b.Y * b.Y));
        var cos = Math.Clamp(Math.Abs(dot) / (magA * magB), -1.0, 1.0);
        return Math.Acos(cos) * 180.0 / Math.PI;
    }

    private static double DegToRad(double degrees) => degrees * Math.PI / 180.0;

    private static IReadOnlyList<(double X, double Y)> ParseShape(string shape)
    {
        var points = new List<(double, double)>();
        foreach (var pair in shape.Split(' ', StringSplitOptions.RemoveEmptyEntries))
        {
            var coords = pair.Split(',');
            var x = double.Parse(coords[0], CultureInfo.InvariantCulture);
            var y = double.Parse(coords[1], CultureInfo.InvariantCulture);
            points.Add((x, y));
        }

        return points;
    }

    private static string RequireAttribute(XElement element, string name) =>
        element.Attribute(name)?.Value
        ?? throw new InvalidDataException($"<{element.Name}> is missing required attribute '{name}'.");
}
