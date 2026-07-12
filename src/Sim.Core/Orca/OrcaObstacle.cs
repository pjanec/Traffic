namespace Sim.Core.Orca;

// One VERTEX of a static-obstacle polyline (a "wall"), mirroring RVO2's Obstacle node exactly.
// A polyline of n vertices produces n of these, linked into a CLOSED loop (RVO2's
// RVOSimulator::addObstacle always closes the loop -- see OrcaCrowd.AddObstacle). For a wall meant
// to be traversed as an open segment, supply vertices such that the closing edge sits outside the
// area agents can reach; RVO2 itself has no "open polyline" variant, so neither do we.
public readonly struct OrcaObstacle
{
    public readonly Vec2 Point;
    public readonly Vec2 UnitDir;      // unit vector from Point to the NEXT vertex in the loop
    public readonly bool IsConvex;
    public readonly int PrevIndex;     // index (into the crowd's obstacle array) of the previous vertex
    public readonly int NextIndex;     // index (into the crowd's obstacle array) of the next vertex

    public OrcaObstacle(Vec2 point, Vec2 unitDir, bool isConvex, int prevIndex, int nextIndex)
    {
        Point = point;
        UnitDir = unitDir;
        IsConvex = isConvex;
        PrevIndex = prevIndex;
        NextIndex = nextIndex;
    }

    // RVO2's exact leftOf test: det(a - c, b - a). >= 0 means b is (weakly) to the left of the
    // directed line a->c... concretely: is `b` on the left of the segment from `a` to `c`? Used both
    // to classify vertex convexity at construction time (OrcaCrowd.AddObstacle) and, in RVO2's
    // kd-tree obstacle query, to decide which side of an obstacle edge the agent sits on (we do a
    // brute-force scan instead of a kd-tree, so we don't need the query use here, only construction).
    public static double LeftOf(Vec2 a, Vec2 b, Vec2 c) => Vec2.Det(a - c, b - a);

    // RVO2's distSqPointLineSegment: squared distance from point `c` to the segment [a, b].
    public static double DistanceSquaredToSegment(Vec2 a, Vec2 b, Vec2 c)
    {
        var ab = b - a;
        var abAbsSq = ab.AbsSq;
        if (abAbsSq <= 0.0)
        {
            return (c - a).AbsSq;
        }

        var r = Vec2.Dot(c - a, ab) / abAbsSq;

        if (r < 0.0)
        {
            return (c - a).AbsSq;
        }

        if (r > 1.0)
        {
            return (c - b).AbsSq;
        }

        return (c - (a + r * ab)).AbsSq;
    }
}

// The per-obstacle-VERTEX input the crowd hands the solver for one obstacle-neighbour query result
// (RVO2's "obstacle1" plus everything computeObstacleNeighbors would otherwise pointer-chase for:
// obstacle1's next vertex ("obstacle2") and obstacle1's previous vertex's direction, needed for the
// "foreign leg" clamp). Carrying these by value keeps the solver free of any reference to the
// crowd's obstacle array or its indices -- purely functional, like OrcaSolver.Agent.
public readonly struct ObstacleSegment
{
    public readonly Vec2 Point1;       // obstacle1.Point
    public readonly Vec2 Point2;       // obstacle2.Point (obstacle1's next vertex)
    public readonly Vec2 UnitDir1;     // obstacle1.UnitDir
    public readonly Vec2 UnitDir2;     // obstacle2.UnitDir
    public readonly Vec2 PrevUnitDir;  // obstacle1.prev.UnitDir (the "left neighbour")
    public readonly bool IsConvex1;    // obstacle1.IsConvex
    public readonly bool IsConvex2;    // obstacle2.IsConvex

    public ObstacleSegment(
        Vec2 point1, Vec2 point2, Vec2 unitDir1, Vec2 unitDir2, Vec2 prevUnitDir, bool isConvex1, bool isConvex2)
    {
        Point1 = point1;
        Point2 = point2;
        UnitDir1 = unitDir1;
        UnitDir2 = unitDir2;
        PrevUnitDir = prevUnitDir;
        IsConvex1 = isConvex1;
        IsConvex2 = isConvex2;
    }
}
