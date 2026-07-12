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
// double math. Static-obstacle (wall) avoidance is ported from RVO2's Agent::computeNewVelocity
// obstacle section: obstacle half-planes are constructed FIRST, into lineScratch[0..numObstLines),
// and the agent-agent ORCA half-planes (unchanged from before) are appended after them; the 3D LP's
// numObstLines threads that split through so LinearProgram3 treats the (already-absolute) obstacle
// lines as a fixed base rather than pairwise-intersecting them with each other.
public static class OrcaSolver
{
    // Guard against division by ~0 when two constraint lines are (almost) parallel.
    private const double Eps = 1e-10;

    // RVO2's RVO_EPSILON: used only in the obstacle-line construction below (the "already covered"
    // test), exactly where RVO2 uses it. The LP's own near-parallel guard above (Eps) is unrelated
    // and unchanged.
    private const double RvoEpsilon = 1e-5;

    // One agent's frozen kinematic state, value-typed (SoA-friendly, no strings, no handles baked
    // in). The crowd driver owns identity/goals; the solver sees pure geometry + velocity.
    public readonly struct Agent
    {
        public readonly Vec2 Position;
        public readonly Vec2 Velocity;
        public readonly double Radius;
        // The fraction of the avoidance correction SELF takes when avoiding THIS neighbour. 0.5 is
        // reciprocal ORCA (the neighbour, running the same solve, takes the other half -> mutual
        // avoidance). 1.0 is one-sided: the neighbour does NOT reciprocate (a static blocker, or a
        // mover from the OTHER regime that avoids self through its own separate solve -- the
        // cross-regime bridge case), so self takes the whole correction. Default 0.5.
        public readonly double Responsibility;

        public Agent(Vec2 position, Vec2 velocity, double radius, double responsibility = 0.5)
        {
            Position = position;
            Velocity = velocity;
            Radius = radius;
            Responsibility = responsibility;
        }
    }

    // Compute the collision-avoiding new velocity for `self` given its `neighbours` (already
    // filtered to those within reaction range), the `obstacles` (static-wall vertices within the
    // obstacle horizon), the preferred velocity it would take if unobstructed, maxSpeed, the agent
    // planning time horizon (s), the obstacle planning time horizon (s, RVO2 uses a shorter one for
    // walls than for agents), and the integration timeStep (s, used only for the already-colliding
    // agent-agent recovery branch). `lineScratch` must have length >= obstacles.Length +
    // neighbours.Length; it holds the ORCA half-planes (caller-supplied to keep the solver
    // allocation-free), obstacle lines first, then agent lines.
    public static Vec2 ComputeNewVelocity(
        in Agent self,
        ReadOnlySpan<Agent> neighbours,
        ReadOnlySpan<ObstacleSegment> obstacles,
        Vec2 prefVelocity,
        double maxSpeed,
        double timeHorizon,
        double timeHorizonObst,
        double timeStep,
        Span<OrcaLine> lineScratch)
    {
        var lineCount = ComputeObstacleLines(self, obstacles, timeHorizonObst, lineScratch);
        var numObstLines = lineCount;

        var invTimeHorizon = 1.0 / timeHorizon;

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

            // Reciprocity: self takes its `Responsibility` fraction of the correction (0.5 = mutual,
            // the neighbour taking the other half; 1.0 = one-sided, self avoids fully -- e.g. a
            // cross-regime mover that avoids self through its own separate solve).
            lineScratch[lineCount++] = new OrcaLine(self.Velocity + other.Responsibility * u, direction);
        }

        var lines = lineScratch[..lineCount];
        var result = Vec2.Zero;
        var lineFail = LinearProgram2(lines, maxSpeed, prefVelocity, false, ref result);
        if (lineFail < lines.Length)
        {
            // The dense-packing fallback: no velocity satisfies every constraint, so minimise the
            // maximum penetration instead of failing hard.
            LinearProgram3(lines, numObstLines, lineFail, maxSpeed, ref result);
        }

        return result;
    }

    // Static-obstacle (wall) half-planes, ported verbatim from RVO2's Agent::computeNewVelocity
    // obstacle section. Writes into lineScratch[0..) in obstacle-array order (deterministic) and
    // returns the count written (== numObstLines for the caller). Self-contained: takes only frozen
    // geometry (self.Position / self.Velocity / self.Radius) and the per-vertex ObstacleSegment the
    // crowd already resolved (no pointer-chasing here, unlike RVO2's linked Obstacle nodes).
    private static int ComputeObstacleLines(
        in Agent self, ReadOnlySpan<ObstacleSegment> obstacles, double timeHorizonObst, Span<OrcaLine> lineScratch)
    {
        var invTimeHorizonObst = 1.0 / timeHorizonObst;
        var lineCount = 0;

        for (var oi = 0; oi < obstacles.Length; oi++)
        {
            var o = obstacles[oi];
            var relativePosition1 = o.Point1 - self.Position;
            var relativePosition2 = o.Point2 - self.Position;

            // Already-covered test: skip this obstacle vertex if a previously constructed obstacle
            // line (from THIS call, indices [0, lineCount) built so far) already forbids the whole
            // velocity-obstacle wedge this vertex would add.
            var alreadyCovered = false;
            for (var j = 0; j < lineCount; j++)
            {
                var lineJ = lineScratch[j];
                if (Vec2.Det(invTimeHorizonObst * relativePosition1 - lineJ.Point, lineJ.Direction)
                        - invTimeHorizonObst * self.Radius >= -RvoEpsilon
                    && Vec2.Det(invTimeHorizonObst * relativePosition2 - lineJ.Point, lineJ.Direction)
                        - invTimeHorizonObst * self.Radius >= -RvoEpsilon)
                {
                    alreadyCovered = true;
                    break;
                }
            }

            if (alreadyCovered)
            {
                continue;
            }

            var distSq1 = relativePosition1.AbsSq;
            var distSq2 = relativePosition2.AbsSq;
            var radiusSq = self.Radius * self.Radius;

            var obstacleVector = o.Point2 - o.Point1;
            var s = Vec2.Dot(-relativePosition1, obstacleVector) / obstacleVector.AbsSq;
            var distSqLine = (-relativePosition1 - s * obstacleVector).AbsSq;

            // Local mutable copies: the "oblique" (near-vertex) cases below reassign these to make
            // obstacle1 and obstacle2 the SAME vertex (RVO2's `obstacle2 = obstacle1;` / `obstacle1 =
            // obstacle2;` pointer reassignment), which point1/point2 mirror so the cutoff points stay
            // exactly right.
            var point1 = o.Point1;
            var point2 = o.Point2;
            var p1 = relativePosition1;
            var p2 = relativePosition2;
            var unitDir1 = o.UnitDir1;
            var unitDir2 = o.UnitDir2;
            var convex1 = o.IsConvex1;
            var convex2 = o.IsConvex2;
            var ds1 = distSq1;
            var ds2 = distSq2;
            var sameVertex = false;

            if (s < 0.0 && distSq1 <= radiusSq)
            {
                if (convex1)
                {
                    lineScratch[lineCount++] = new OrcaLine(Vec2.Zero, new Vec2(-p1.Y, p1.X).Normalized());
                }

                continue;
            }

            if (s > 1.0 && distSq2 <= radiusSq)
            {
                if (convex2 && Vec2.Det(p2, unitDir2) >= 0.0)
                {
                    lineScratch[lineCount++] = new OrcaLine(Vec2.Zero, new Vec2(-p2.Y, p2.X).Normalized());
                }

                continue;
            }

            if (s >= 0.0 && s < 1.0 && distSqLine <= radiusSq)
            {
                lineScratch[lineCount++] = new OrcaLine(Vec2.Zero, -unitDir1);
                continue;
            }

            // Legs of the velocity-obstacle cone from the near vertex/edge.
            Vec2 leftLegDirection;
            Vec2 rightLegDirection;

            if (s < 0.0 && distSqLine <= radiusSq)
            {
                if (!convex1)
                {
                    continue;
                }

                sameVertex = true;
                point2 = point1;
                p2 = p1;
                ds2 = ds1;
                convex2 = convex1;
                unitDir2 = unitDir1;

                var leg1 = Math.Sqrt(ds1 - radiusSq);
                leftLegDirection = new Vec2(p1.X * leg1 - p1.Y * self.Radius, p1.X * self.Radius + p1.Y * leg1) / ds1;
                rightLegDirection = new Vec2(p1.X * leg1 + p1.Y * self.Radius, -p1.X * self.Radius + p1.Y * leg1) / ds1;
            }
            else if (s > 1.0 && distSqLine <= radiusSq)
            {
                if (!convex2)
                {
                    continue;
                }

                sameVertex = true;
                point1 = point2;
                p1 = p2;
                ds1 = ds2;
                convex1 = convex2;
                unitDir1 = unitDir2;

                var leg2 = Math.Sqrt(ds2 - radiusSq);
                leftLegDirection = new Vec2(p2.X * leg2 - p2.Y * self.Radius, p2.X * self.Radius + p2.Y * leg2) / ds2;
                rightLegDirection = new Vec2(p2.X * leg2 + p2.Y * self.Radius, -p2.X * self.Radius + p2.Y * leg2) / ds2;
            }
            else
            {
                if (convex1)
                {
                    var leg1 = Math.Sqrt(ds1 - radiusSq);
                    leftLegDirection = new Vec2(p1.X * leg1 - p1.Y * self.Radius, p1.X * self.Radius + p1.Y * leg1) / ds1;
                }
                else
                {
                    leftLegDirection = -unitDir1;
                }

                if (convex2)
                {
                    var leg2 = Math.Sqrt(ds2 - radiusSq);
                    rightLegDirection = new Vec2(p2.X * leg2 + p2.Y * self.Radius, -p2.X * self.Radius + p2.Y * leg2) / ds2;
                }
                else
                {
                    rightLegDirection = unitDir1;
                }
            }

            // Legs can never point into a neighbouring edge at a convex vertex: clamp to the
            // neighbour's cutoff line instead ("foreign" leg -- if the projected velocity lands on
            // it, no constraint is added for that side).
            var isLeftLegForeign = false;
            var isRightLegForeign = false;

            if (convex1 && Vec2.Det(leftLegDirection, -o.PrevUnitDir) >= 0.0)
            {
                leftLegDirection = -o.PrevUnitDir;
                isLeftLegForeign = true;
            }

            if (convex2 && Vec2.Det(rightLegDirection, unitDir2) <= 0.0)
            {
                rightLegDirection = unitDir2;
                isRightLegForeign = true;
            }

            var leftCutoff = invTimeHorizonObst * (point1 - self.Position);
            var rightCutoff = invTimeHorizonObst * (point2 - self.Position);
            var cutoffVec = rightCutoff - leftCutoff;

            var t = sameVertex ? 0.5 : Vec2.Dot(self.Velocity - leftCutoff, cutoffVec) / cutoffVec.AbsSq;
            var tLeft = Vec2.Dot(self.Velocity - leftCutoff, leftLegDirection);
            var tRight = Vec2.Dot(self.Velocity - rightCutoff, rightLegDirection);

            if ((t < 0.0 && tLeft < 0.0) || (sameVertex && tLeft < 0.0 && tRight < 0.0))
            {
                var unitW = (self.Velocity - leftCutoff).Normalized();
                lineScratch[lineCount++] = new OrcaLine(
                    leftCutoff + self.Radius * invTimeHorizonObst * unitW,
                    new Vec2(unitW.Y, -unitW.X));
                continue;
            }

            if (t > 1.0 && tRight < 0.0)
            {
                var unitW = (self.Velocity - rightCutoff).Normalized();
                lineScratch[lineCount++] = new OrcaLine(
                    rightCutoff + self.Radius * invTimeHorizonObst * unitW,
                    new Vec2(unitW.Y, -unitW.X));
                continue;
            }

            var distSqCutoff = (t < 0.0 || t > 1.0 || sameVertex)
                ? double.PositiveInfinity
                : (self.Velocity - (leftCutoff + t * cutoffVec)).AbsSq;
            var distSqLeft = tLeft < 0.0
                ? double.PositiveInfinity
                : (self.Velocity - (leftCutoff + tLeft * leftLegDirection)).AbsSq;
            var distSqRight = tRight < 0.0
                ? double.PositiveInfinity
                : (self.Velocity - (rightCutoff + tRight * rightLegDirection)).AbsSq;

            if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
            {
                var dir = -unitDir1;
                lineScratch[lineCount++] = new OrcaLine(
                    leftCutoff + self.Radius * invTimeHorizonObst * new Vec2(-dir.Y, dir.X),
                    dir);
                continue;
            }

            if (distSqLeft <= distSqRight)
            {
                if (isLeftLegForeign)
                {
                    continue;
                }

                var dir = leftLegDirection;
                lineScratch[lineCount++] = new OrcaLine(
                    leftCutoff + self.Radius * invTimeHorizonObst * new Vec2(-dir.Y, dir.X),
                    dir);
                continue;
            }

            {
                if (isRightLegForeign)
                {
                    continue;
                }

                var dir = -rightLegDirection;
                lineScratch[lineCount++] = new OrcaLine(
                    rightCutoff + self.Radius * invTimeHorizonObst * new Vec2(-dir.Y, dir.X),
                    dir);
            }
        }

        return lineCount;
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
    // violation. Static-obstacle lines occupy lines[0..numObstLines) and are already absolute
    // half-planes (not derived relative to any other line), so the projected LP for line i starts as
    // a direct COPY of them, and only the agent lines [numObstLines, i) get pairwise-intersected
    // against line i (mirrors RVO2's linearProgram3 exactly).
    private static void LinearProgram3(
        ReadOnlySpan<OrcaLine> lines, int numObstLines, int beginLine, double radius, ref Vec2 result)
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
                // Seed with the obstacle lines verbatim (they need no intersection with line i: they
                // are fixed constraints already expressed in absolute (point, direction) form).
                lines[..numObstLines].CopyTo(projLines);
                var projCount = numObstLines;

                for (var j = numObstLines; j < i; j++)
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
