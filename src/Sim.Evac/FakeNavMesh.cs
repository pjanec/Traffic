using Sim.Core.Orca;
using Sim.Ingest;

namespace Sim.Evac;

// PANIC-EVAC.md R7/R8 / §8.4: the minimal Phase-1 fake-navmesh. The "known world" is derived ENTIRELY
// from the SUMO net's own geometry (no external world data): here, the axis-aligned bounding box of
// every lane and junction vertex, expanded outward by the vicinity width. Its outer edge is a single
// hard wall (a ditch/fence/soft-soil boundary) fed to the pedestrian OrcaCrowd as one closed obstacle
// loop, so pedestrians cannot cross it — they pile at it, matching reality.
//
// This is deliberately the CRUDEST region that still gives a real hard edge and a testable containment
// invariant. The Phase-3 refinement (R8) buffers each lane+junction polygon individually so
// pedestrians flee ALONG the streets and block at the true road edge; that drops in behind this same
// "navigable region + boundary loop" interface. Interior car obstacles are supplied separately by the
// director (OrcaCrowd.SetExternalObstacles), not baked into the mesh.
public sealed class FakeNavMesh
{
    public double MinX { get; }
    public double MinY { get; }
    public double MaxX { get; }
    public double MaxY { get; }

    // One closed loop = the hard outer edge. Wound CLOCKWISE so RVO2 keeps agents on the INTERIOR
    // side (verified by the no-pedestrian-crosses-the-edge test; flip the winding if it ever fails).
    public IReadOnlyList<Vec2> BoundaryLoop { get; }

    // PANIC-EVAC-PHASE3-DESIGN.md §4: the same boundary rectangle expressed as four wall segments for
    // a MixedTrafficCrowd (AddWall per segment) -- the shaped-mover analogue of BoundaryLoop, first-cut
    // band for Orca-push cars (outer hard edge; the richer per-lane buffered band is a later sub-task).
    public IReadOnlyList<(Vec2 A, Vec2 B)> BandWalls { get; }

    public FakeNavMesh(NetworkModel net, double vicinityWidth)
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;

        void Accumulate(IReadOnlyList<(double X, double Y)> shape)
        {
            foreach (var (x, y) in shape)
            {
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }
        }

        foreach (var lane in net.LanesByHandle)
        {
            Accumulate(lane.Shape);
        }

        foreach (var junction in net.Junctions)
        {
            if (junction.Shape is { Count: > 0 })
            {
                Accumulate(junction.Shape);
            }
        }

        if (double.IsInfinity(minX))
        {
            // No geometry (shouldn't happen for a loaded net) — degenerate to a point.
            minX = minY = maxX = maxY = 0.0;
        }

        var margin = Math.Max(0.0, vicinityWidth);
        MinX = minX - margin;
        MinY = minY - margin;
        MaxX = maxX + margin;
        MaxY = maxY + margin;

        BoundaryLoop = new List<Vec2>
        {
            new(MinX, MinY),
            new(MinX, MaxY),
            new(MaxX, MaxY),
            new(MaxX, MinY),
        };

        var bl = new Vec2(MinX, MinY);
        var tl = new Vec2(MinX, MaxY);
        var tr = new Vec2(MaxX, MaxY);
        var br = new Vec2(MaxX, MinY);
        BandWalls = new List<(Vec2 A, Vec2 B)>
        {
            (bl, tl),
            (tl, tr),
            (tr, br),
            (br, bl),
        };
    }

    public bool Contains(Vec2 p) => p.X >= MinX && p.X <= MaxX && p.Y >= MinY && p.Y <= MaxY;

    // Clamp a point strictly inside the wall (a small inset) so spawned pedestrians and their goals
    // never sit exactly on the obstacle segment.
    public Vec2 ClampInterior(Vec2 p, double inset = 0.5)
    {
        var x = Math.Min(Math.Max(p.X, MinX + inset), MaxX - inset);
        var y = Math.Min(Math.Max(p.Y, MinY + inset), MaxY - inset);
        return new Vec2(x, y);
    }
}
