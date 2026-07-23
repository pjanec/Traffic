using Sim.Ingest;
using Sim.Replication;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Procedural scene generation -> Traffic lights" / task T1.6 -- pure
// signal-head placement math, no Godot type anywhere (same "pure math, no Godot types, tested" pattern as
// RoadMeshBuilder/BuildingPlacer/CarTransform). Static geometry (the pole/head POSITIONS never move once
// placed); only the head's material colour changes per frame, driven by `ColorFor` off the live replicated
// signal byte -- the Viewer project turns this into MeshInstance3D nodes + per-frame material updates.
public readonly struct SignalHead
{
    public SignalHead(int laneHandle, float poleX, float poleY, float poleZ, float headX, float headY, float headZ)
    {
        LaneHandle = laneHandle;
        PoleX = poleX;
        PoleY = poleY;
        PoleZ = poleZ;
        HeadX = headX;
        HeadY = headY;
        HeadZ = headZ;
    }

    // The TL-controlled approach lane this head signals for -- the same key
    // Sim.Replication.IReplicationSource.TlStateByLane is indexed by, so a renderer looks the live signal
    // byte up as `tlStateByLane[head.LaneHandle]`.
    public int LaneHandle { get; }

    // Pole base, already in GODOT space -- sits on the ground under the head (PoleY ~= the lane's own
    // ground elevation; 0 on a flat net).
    public float PoleX { get; }
    public float PoleY { get; }
    public float PoleZ { get; }

    // Signal head, already in GODOT space -- directly above the pole base, raised HeadHeightMeters.
    public float HeadX { get; }
    public float HeadY { get; }
    public float HeadZ { get; }
}

// Pure mapping of a signal byte ('r'/'y'/'g'/'G'/other) to a renderer-agnostic colour bucket -- the
// Viewer project maps each bucket to an actual emissive Color.
public enum SignalColor
{
    Red,
    Yellow,
    Green,
    Off,
}

public static class TrafficLightPlacer
{
    // Design "Traffic lights": "place a simple pole ... and a head ... at the top" -- a believable
    // eye-level-and-above signal mast height.
    public const float HeadHeightMeters = 5f;

    // Pure signal-byte -> colour-bucket mapping (task T1.6 Part A): 'r' -> Red, 'y' -> Yellow, 'g'/'G' ->
    // Green, anything else (an unrecognized/absent state, e.g. between-phase or a non-road rail signal
    // byte) -> Off.
    public static SignalColor ColorFor(byte signalByte) => (char)signalByte switch
    {
        'r' => SignalColor.Red,
        'y' => SignalColor.Yellow,
        'g' or 'G' => SignalColor.Green,
        _ => SignalColor.Off,
    };

    // For each TL-controlled approach lane (identified by its dense Handle, e.g.
    // sim.Source.TlStateByLane.Keys), place one SignalHead at that lane's downstream end (Shape[^1] --
    // the stop-line, ported from the same "lane's own downstream end" convention
    // Sim.Viewer.Raylib.Renderer.DrawWorldDds uses: `tlLane.Points[^1]`), nudged toward the lane's right
    // edge by half its width along the segment normal (same normal/edge convention RoadMeshBuilder's
    // `right[i] = center - normal*halfWidth` uses, so a head sits at the actual road edge rather than
    // straddling the centerline). Unknown/out-of-range handles are skipped rather than throwing (a
    // defensive guard for a caller passing a stale or foreign handle set).
    public static IReadOnlyList<SignalHead> Place(NetworkModel network, IEnumerable<int> controlledLaneHandles)
    {
        var result = new List<SignalHead>();

        foreach (var handle in controlledLaneHandles)
        {
            if (handle < 0 || handle >= network.LanesByHandle.Count)
            {
                continue;
            }

            var lane = network.LanesByHandle[handle];
            var shape = lane.Shape;
            if (shape.Count == 0)
            {
                continue;
            }

            var (endX, endY) = shape[^1];

            // Segment normal at the downstream end (rotate the final segment's direction +90deg CCW in
            // SUMO's (x,y) plane, same convention RoadMeshBuilder/BuildingPlacer both use:
            // (dx,dy) -> (-dy,dx), normalized). Degenerate (single-point shape): no direction to derive a
            // normal from, so no edge nudge -- the head sits on the lane's raw endpoint.
            double nx = 0.0, ny = 0.0;
            if (shape.Count >= 2)
            {
                var (prevX, prevY) = shape[^2];
                var dx = endX - prevX;
                var dy = endY - prevY;
                var len = Math.Sqrt(dx * dx + dy * dy);
                if (len > 1e-9)
                {
                    nx = -dy / len;
                    ny = dx / len;
                }
            }

            var halfWidth = lane.Width / 2.0;
            // RoadMeshBuilder's "right" edge convention: center - normal*halfWidth.
            var baseX = endX - nx * halfWidth;
            var baseY = endY - ny * halfWidth;

            // Ground elevation at the lane's downstream end, same "shapeZ[i] : 0" fallback
            // RoadMeshBuilder/BuildingPlacer both use for a 2-D (ShapeZ == null) lane.
            var lastIndex = shape.Count - 1;
            var groundZ = lane.ShapeZ is not null && lastIndex < lane.ShapeZ.Count ? lane.ShapeZ[lastIndex] : 0.0;

            var (poleX, poleY, poleZ) = CoordinateTransform.SumoToGodot(baseX, baseY, groundZ);

            result.Add(new SignalHead(handle, poleX, poleY, poleZ, poleX, poleY + HeadHeightMeters, poleZ));
        }

        return result;
    }

    // docs/DEMO-CITY3D-DESIGN.md "Data path -> Remote mode" / task T2.2b: the REMOTE-shaped counterpart of
    // Place(NetworkModel, ...) above. A remote viewer never has a NetworkModel (no .net.xml parse), only
    // the received wire geometry (IReplicationSource.Geometry, GeometryCodec.LaneGeo per lane handle) --
    // which DOES carry everything this placement math actually needs (Points, Width), so the identical
    // downstream-end + right-edge-nudge recipe below runs unchanged; only the per-lane lookup source
    // differs (a LaneGeo dictionary vs NetworkModel.LanesByHandle). docs/LIVE-CITY-VIEWERS-TASKS.md Stage E
    // (E1): the wire CAN carry elevation now (GeometryCodec.LaneGeo.Z), but this placer doesn't sample it
    // yet (out of E1's scope -- ReplicationLaneShapeSource/RoadMeshBuilder are the two files that task
    // touches) -- groundZ stays the flat-net fallback of 0 here regardless of the net's real elevation.
    public static IReadOnlyList<SignalHead> Place(
        IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry, IEnumerable<int> controlledLaneHandles)
    {
        var result = new List<SignalHead>();

        foreach (var handle in controlledLaneHandles)
        {
            if (!geometry.TryGetValue(handle, out var lane) || lane.Points.Length == 0)
            {
                continue;
            }

            var shape = lane.Points;
            var (endX, endY) = shape[^1];

            double nx = 0.0, ny = 0.0;
            if (shape.Length >= 2)
            {
                var (prevX, prevY) = shape[^2];
                var dx = endX - prevX;
                var dy = endY - prevY;
                var len = Math.Sqrt(dx * dx + dy * dy);
                if (len > 1e-9)
                {
                    nx = -dy / len;
                    ny = dx / len;
                }
            }

            var halfWidth = lane.Width / 2.0;
            var baseX = endX - nx * halfWidth;
            var baseY = endY - ny * halfWidth;

            // The wire carries no elevation (GeometryCodec.LaneGeo has no z field) -- groundZ is always the
            // flat-net fallback of 0, same convention RoadMeshBuilder's geometry-dictionary overload uses.
            const double groundZ = 0.0;

            var (poleX, poleY, poleZ) = CoordinateTransform.SumoToGodot(baseX, baseY, groundZ);

            result.Add(new SignalHead(handle, poleX, poleY, poleZ, poleX, poleY + HeadHeightMeters, poleZ));
        }

        return result;
    }
}
