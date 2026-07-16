using Sim.Core;
using Sim.Replication;

namespace Sim.Viewer.Raylib;

// docs/SUMOSHARP-NATIVE-VIEWER.md P2b — PoseResolver.ILaneShapeSource backed by the subscriber's DECODED
// DDS geometry (GeometryCodec.LaneGeo), so PoseResolver can walk the remotely-received lane polylines (not
// the local NetworkModel) when resolving a loopback/remote vehicle's dead-reckoned pose. 2-D only: the
// geometry wire format (GeometryCodec) carries no elevation yet, so LaneShapeZ is always null.
public sealed class DdsGeometryLaneSource : ILaneShapeSource
{
    private readonly IReadOnlyDictionary<int, GeometryCodec.LaneGeo> _geometry;

    public DdsGeometryLaneSource(IReadOnlyDictionary<int, GeometryCodec.LaneGeo> geometry)
    {
        _geometry = geometry;
    }

    public double LaneLength(int laneHandle) =>
        _geometry.TryGetValue(laneHandle, out var lane) ? lane.Length : 0.0;

    public IReadOnlyList<(double X, double Y)> LaneShape(int laneHandle)
    {
        if (!_geometry.TryGetValue(laneHandle, out var lane))
        {
            return Array.Empty<(double, double)>();
        }

        var points = lane.Points;
        var shape = new (double X, double Y)[points.Length];
        for (var i = 0; i < points.Length; i++)
        {
            shape[i] = (points[i].X, points[i].Y);
        }

        return shape;
    }

    public IReadOnlyList<double>? LaneShapeZ(int laneHandle) => null;
}
