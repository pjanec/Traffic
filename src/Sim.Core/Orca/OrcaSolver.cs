namespace Sim.Core.Orca;

// A half-plane constraint on the new velocity: the feasible side is LEFT of the directed line
// through Point along Direction (i.e. det(Direction, v - Point) >= 0).
public readonly struct OrcaLine
{
    public readonly Vec2 Point;
    public readonly Vec2 Direction;

    public OrcaLine(Vec2 point, Vec2 direction)
    {
        Point = point;
        Direction = direction;
    }
}

// OPEN-SPACE full 2D reciprocal ORCA (docs/LANELESS-DIRECTION.md, the second regime): holonomic
// disc agents choosing, each independently and reciprocally, a new velocity that provably avoids
// collision with their neighbours for a time horizon while staying closest to a preferred velocity.
// This is the model for external crowd / navmesh agents -- NOT lane-derived vehicles (those are
// elongated and quasi-1D longitudinally, and use the lateral feasible-interval solve +
// car-following in Engine.ComputeRvoLateral). ORCA is our reference here the way SUMO is the
// reference for lane behaviour: a faithful port of van den Berg, Guy, Lin & Manocha, "Reciprocal
// n-Body Collision Avoidance" (2011) -- the RVO2 reference algorithm. Validated BEHAVIOURALLY
// (no-overlap, reaches-goal, reciprocal-symmetry, deterministic), no SUMO golden -- SUMO has no
// such model.
//
// Purely functional and parallel-safe: ComputeNewVelocity reads only frozen agent state and writes
// nothing; the caller integrates. Deterministic: fixed neighbour order, no RNG, no wall-clock,
// double math. There are no static-obstacle lines in this layer (agent-agent only), so the 3D LP's
// numObstLines is always 0.
public static class OrcaSolver
{
    // Guard against division by ~0 when two constraint lines are (almost) parallel.
    private const double Eps = 1e-10;

    // One agent's frozen kinematic state, value-typed (SoA-friendly, no strings, no handles baked
    // in). The crowd driver owns identity/goals; the solver sees pure geometry + velocity.
    public readonly struct Agent
    {
        public readonly Vec2 Position;
        public readonly Vec2 Velocity;
        public readonly double Radius;

        public Agent(Vec2 position, Vec2 velocity, double radius)
        {
            Position = position;
            Velocity = velocity;
            Radius = radius;
        }
    }

    // Compute the collision-avoiding new velocity for `self` given its `neighbours` (already
    // filtered to those within reaction range), the preferred velocity it would take if unobstructed,
    // maxSpeed, the planning time horizon (s), and the integration timeStep (s, used only for the
    // already-colliding recovery branch). `lineScratch` must have length >= neighbours.Length; it
    // holds the ORCA half-planes (caller-supplied to keep the solver allocation-free).
    public static Vec2 ComputeNewVelocity(
        in Agent self,
        ReadOnlySpan<Agent> neighbours,
        Vec2 prefVelocity,
        double maxSpeed,
        double timeHorizon,
        double timeStep,
        Span<OrcaLine> lineScratch)
    {
        var invTimeHorizon = 1.0 / timeHorizon;
        var lineCount = 0;

        for (var i = 0; i < neighbours.Length; i++)
        {
            var other = neighbours[i];
            var relativePosition = other.Position - self.Position;
            var relativeVelocity = self.Velocity - other.Velocity;
            var distSq = relativePosition.AbsSq;
            var combinedRadius = self.Radius + other.Radius;
            var combinedRadiusSq = combinedRadius * combinedRadius;

            Vec2 direction;
            Vec2 u;

            if (distSq > combinedRadiusSq)
            {
                // No collision yet. Velocity obstacle is a cone truncated by the cutoff circle at
                // relativePosition/timeHorizon with radius combinedRadius/timeHorizon.
                var w = relativeVelocity - invTimeHorizon * relativePosition;
                var wLengthSq = w.AbsSq;
                var dotProduct1 = Vec2.Dot(w, relativePosition);

                if (dotProduct1 < 0.0 && dotProduct1 * dotProduct1 > combinedRadiusSq * wLengthSq)
                {
                    // Project on the cutoff circle.
                    var wLength = Math.Sqrt(wLengthSq);
                    var unitW = w / wLength;
                    direction = new Vec2(unitW.Y, -unitW.X);
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    // Project on one of the cone legs.
                    var leg = Math.Sqrt(distSq - combinedRadiusSq);
                    if (Vec2.Det(relativePosition, w) > 0.0)
                    {
                        // Left leg.
                        direction = new Vec2(
                            relativePosition.X * leg - relativePosition.Y * combinedRadius,
                            relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq;
                    }
                    else
                    {
                        // Right leg.
                        direction = -(new Vec2(
                            relativePosition.X * leg + relativePosition.Y * combinedRadius,
                            -relativePosition.X * combinedRadius + relativePosition.Y * leg) / distSq);
                    }

                    var dotProduct2 = Vec2.Dot(relativeVelocity, direction);
                    u = dotProduct2 * direction - relativeVelocity;
                }
            }
            else
            {
                // Already overlapping. Project on the cutoff circle of the (short) time step so the
                // pair separates immediately rather than over the full horizon.
                var invTimeStep = 1.0 / timeStep;
                var w = relativeVelocity - invTimeStep * relativePosition;
                var wLength = w.Abs;
                var unitW = w / wLength;
                direction = new Vec2(unitW.Y, -unitW.X);
                u = (combinedRadius * invTimeStep - wLength) * unitW;
            }

            // Reciprocity: each agent takes HALF the correction; the neighbour (running the same
            // solve) takes the other half -> guaranteed mutual collision avoidance, no oscillation.
            lineScratch[lineCount++] = new OrcaLine(self.Velocity + 0.5 * u, direction);
        }

        var lines = lineScratch[..lineCount];
        var result = Vec2.Zero;
        var lineFail = LinearProgram2(lines, maxSpeed, prefVelocity, false, ref result);
        if (lineFail < lines.Length)
        {
            // The dense-packing fallback: no velocity satisfies every constraint, so minimise the
            // maximum penetration instead of failing hard.
            LinearProgram3(lines, lineFail, maxSpeed, ref result);
        }

        return result;
    }

    // Optimise `result` along a single constraint line `lineNo`, subject to lines [0, lineNo) and the
    // maxSpeed circle. Returns false if the constraints are jointly infeasible for this line.
    private static bool LinearProgram1(
        ReadOnlySpan<OrcaLine> lines, int lineNo, double radius, Vec2 optVelocity, bool directionOpt, ref Vec2 result)
    {
        var line = lines[lineNo];
        var dotProduct = Vec2.Dot(line.Point, line.Direction);
        var discriminant = dotProduct * dotProduct + radius * radius - line.Point.AbsSq;

        if (discriminant < 0.0)
        {
            // The maxSpeed circle does not intersect this line at all.
            return false;
        }

        var sqrtDiscriminant = Math.Sqrt(discriminant);
        var tLeft = -dotProduct - sqrtDiscriminant;
        var tRight = -dotProduct + sqrtDiscriminant;

        for (var i = 0; i < lineNo; i++)
        {
            var denominator = Vec2.Det(line.Direction, lines[i].Direction);
            var numerator = Vec2.Det(lines[i].Direction, line.Point - lines[i].Point);

            if (Math.Abs(denominator) <= Eps)
            {
                // (Almost) parallel. Infeasible if `line` is on the wrong side of line i.
                if (numerator < 0.0)
                {
                    return false;
                }

                continue;
            }

            var t = numerator / denominator;
            if (denominator >= 0.0)
            {
                tRight = Math.Min(tRight, t);   // line i bounds `line` on the right
            }
            else
            {
                tLeft = Math.Max(tLeft, t);     // line i bounds `line` on the left
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        if (directionOpt)
        {
            // Optimise direction (used by the 3D LP): push to the far end along optVelocity's sense.
            result = Vec2.Dot(optVelocity, line.Direction) > 0.0
                ? line.Point + tRight * line.Direction
                : line.Point + tLeft * line.Direction;
        }
        else
        {
            // Optimise the closest point to optVelocity, clamped to [tLeft, tRight].
            var t = Vec2.Dot(line.Direction, optVelocity - line.Point);
            if (t < tLeft)
            {
                result = line.Point + tLeft * line.Direction;
            }
            else if (t > tRight)
            {
                result = line.Point + tRight * line.Direction;
            }
            else
            {
                result = line.Point + t * line.Direction;
            }
        }

        return true;
    }

    // 2D LP: velocity closest to optVelocity satisfying every half-plane, inside the maxSpeed circle.
    // Returns lines.Length on success, or the index of the first line that made it infeasible.
    private static int LinearProgram2(
        ReadOnlySpan<OrcaLine> lines, double radius, Vec2 optVelocity, bool directionOpt, ref Vec2 result)
    {
        if (directionOpt)
        {
            // optVelocity is assumed to be a unit direction here; scale to the speed circle.
            result = optVelocity * radius;
        }
        else if (optVelocity.AbsSq > radius * radius)
        {
            result = optVelocity.Normalized() * radius;
        }
        else
        {
            result = optVelocity;
        }

        for (var i = 0; i < lines.Length; i++)
        {
            if (Vec2.Det(lines[i].Direction, lines[i].Point - result) > 0.0)
            {
                // `result` violates constraint i; re-optimise along line i.
                var tempResult = result;
                if (!LinearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                {
                    result = tempResult;
                    return i;
                }
            }
        }

        return lines.Length;
    }

    // 3D LP fallback for the infeasible case: from `beginLine` on, minimise the maximum constraint
    // violation. With no static-obstacle lines (numObstLines == 0), the projected LP starts empty.
    private static void LinearProgram3(
        ReadOnlySpan<OrcaLine> lines, int beginLine, double radius, ref Vec2 result)
    {
        var distance = 0.0;
        // At most (lines.Length - 1) projected constraints for any single i.
        Span<OrcaLine> projLines = lines.Length <= 64
            ? stackalloc OrcaLine[lines.Length]
            : new OrcaLine[lines.Length];

        for (var i = beginLine; i < lines.Length; i++)
        {
            if (Vec2.Det(lines[i].Direction, lines[i].Point - result) > distance)
            {
                var projCount = 0;   // numObstLines == 0: projected LP begins empty

                for (var j = 0; j < i; j++)
                {
                    OrcaLine projLine;
                    var determinant = Vec2.Det(lines[i].Direction, lines[j].Direction);

                    if (Math.Abs(determinant) <= Eps)
                    {
                        // i and j parallel.
                        if (Vec2.Dot(lines[i].Direction, lines[j].Direction) > 0.0)
                        {
                            continue;   // same direction -> j imposes nothing new
                        }

                        // Opposite directions: split the difference.
                        projLine = new OrcaLine(
                            0.5 * (lines[i].Point + lines[j].Point),
                            (lines[j].Direction - lines[i].Direction).Normalized());
                    }
                    else
                    {
                        var point = lines[i].Point
                            + (Vec2.Det(lines[j].Direction, lines[i].Point - lines[j].Point) / determinant)
                              * lines[i].Direction;
                        projLine = new OrcaLine(point, (lines[j].Direction - lines[i].Direction).Normalized());
                    }

                    projLines[projCount++] = projLine;
                }

                var tempResult = result;
                var dirOpt = new Vec2(-lines[i].Direction.Y, lines[i].Direction.X);
                if (LinearProgram2(projLines[..projCount], radius, dirOpt, true, ref result) < projCount)
                {
                    // Should not happen in principle; keep the safe (feasible) value.
                    result = tempResult;
                }

                distance = Vec2.Det(lines[i].Direction, lines[i].Point - result);
            }
        }
    }
}
