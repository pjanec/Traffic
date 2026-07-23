using Sim.Core;
using Sim.Replication;

namespace CityLib;

// docs/DEMO-CITY3D-DESIGN.md "Data path" — the consumer-side ILaneShapeSource: built from the RECEIVED
// geometry topic (bus.Source.Geometry), never by re-parsing the .net.xml, so local and remote consume
// geometry identically (design's "ReplicationLaneShapeSource" bullet). Wraps whatever
// IReadOnlyDictionary<int, GeometryCodec.LaneGeo> a bound IReplicationSource exposes -- local
// (InMemoryReplicationBus) and remote (DDS) both shape it the same way, so this type never needs to know
// which transport is behind it.
public sealed class ReplicationLaneShapeSource : ILaneShapeSource
{
    private readonly IReadOnlyDictionary<int, GeometryCodec.LaneGeo> _geometry;

    // Decoded (double,double) polylines, cached per lane handle so repeated per-frame PoseResolver walks
    // (several vehicles a frame, several frames a second) don't re-allocate the same lane's points every
    // call. GeometryCodec.LaneGeo.Points is immutable once received (durable geometry, published once), so
    // this cache is safe to keep for the lifetime of the source.
    private readonly Dictionary<int, (double X, double Y)[]> _shapeCache = new();

    // Decoded Z, cached the same way as `_shapeCache` -- see LaneShapeZ below. A separate cache (not folded
    // into a 3-tuple) because most lanes have no Z (Value=null is the common, cheap-to-cache case too).
    private readonly Dictionary<int, double[]?> _shapeZCache = new();

    public ReplicationLaneShapeSource(IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
        => _geometry = geometry ?? throw new ArgumentNullException(nameof(geometry));

    public double LaneLength(int laneHandle) => Get(laneHandle).Length;

    public IReadOnlyList<(double X, double Y)> LaneShape(int laneHandle)
    {
        if (_shapeCache.TryGetValue(laneHandle, out var cached))
        {
            return cached;
        }

        var lane = Get(laneHandle);
        var points = new (double X, double Y)[lane.Points.Length];
        for (var i = 0; i < points.Length; i++)
        {
            points[i] = (lane.Points[i].X, lane.Points[i].Y);
        }

        _shapeCache[laneHandle] = points;
        return points;
    }

    // docs/LIVE-CITY-VIEWERS-DESIGN.md §3.2, -TASKS.md E1: GeometryCodec.LaneGeo now carries an ADDITIVE
    // per-point Z (GeometryCodec.cs's header comment), so a wire-fed lane source returns REAL elevation
    // when the publisher had any (ReplicationPublisher.BuildLaneGeos, from NetworkModel's Lane.ShapeZ) --
    // null only when the source lane genuinely has none (every committed net today), matching the LOCAL
    // path's Sim.Core.NetworkLaneSource for the same net (both now agree, not merely both-flat-by-luck).
    public IReadOnlyList<double>? LaneShapeZ(int laneHandle)
    {
        if (_shapeZCache.TryGetValue(laneHandle, out var cached))
        {
            return cached;
        }

        var lane = Get(laneHandle);
        double[]? z = null;
        if (lane.Z is { Length: > 0 } laneZ)
        {
            z = new double[laneZ.Length];
            for (var i = 0; i < laneZ.Length; i++)
            {
                z[i] = laneZ[i];
            }
        }

        _shapeZCache[laneHandle] = z;
        return z;
    }

    private GeometryCodec.LaneGeo Get(int laneHandle)
    {
        if (_geometry.TryGetValue(laneHandle, out var lane))
        {
            return lane;
        }

        throw new KeyNotFoundException(
            $"ReplicationLaneShapeSource: no geometry received yet for lane handle {laneHandle}.");
    }
}
