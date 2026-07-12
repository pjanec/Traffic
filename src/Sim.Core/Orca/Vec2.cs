namespace Sim.Core.Orca;

// Minimal double-precision 2D vector for the OPEN-SPACE ORCA layer (docs/LANELESS-DIRECTION.md,
// the second of the "two regimes"). Deliberately self-contained and value-typed: this layer does
// not touch the lane-parity core, the string-keyed ExternalObstacle API, or any float math -- it
// is a fresh holonomic 2D crowd subsystem for navmesh/RVO agents, built to scale to many agents.
// Doubles (not RVO2's float) for determinism consistency with the rest of the engine.
public readonly struct Vec2
{
    public readonly double X;
    public readonly double Y;

    public Vec2(double x, double y)
    {
        X = x;
        Y = y;
    }

    public static readonly Vec2 Zero = new(0.0, 0.0);

    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator -(Vec2 a) => new(-a.X, -a.Y);
    public static Vec2 operator *(double s, Vec2 a) => new(s * a.X, s * a.Y);
    public static Vec2 operator *(Vec2 a, double s) => new(a.X * s, a.Y * s);
    public static Vec2 operator /(Vec2 a, double s) => new(a.X / s, a.Y / s);

    // Dot product.
    public static double Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;

    // 2D cross / determinant: det(a, b) = a.x*b.y - a.y*b.x. Sign gives left/right orientation.
    public static double Det(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;

    public double AbsSq => X * X + Y * Y;
    public double Abs => Math.Sqrt(AbsSq);

    public Vec2 Normalized()
    {
        var len = Abs;
        return len > 0.0 ? new Vec2(X / len, Y / len) : Zero;
    }

    // Perpendicular (rotate -90 deg): used to seed direction-optimisation in the LP.
    public Vec2 PerpCW => new(Y, -X);
}
