using Sim.Core.Orca;

namespace Sim.Pedestrians.Density;

// POC-4 (docs/PEDESTRIAN-POC-PLAN.md POC-4; docs/PEDESTRIAN-DESIGN.md §3 "Believability"):
// bidirectional-flow corridor built ENTIRELY from committed pieces (OrcaCrowd +
// OrcaCrowd.AddObstacle thin walls, no changes to OrcaCrowd itself) to MEASURE whether pure ORCA
// self-organizes into believable counter-flow lanes and drains without deadlock -- the "measure
// first" gate the design doc requires before any density term is even considered. Pure ped-layer:
// no Engine, no navmesh -- goals are hand-placed straight-line targets (each agent's goal is the
// mirror point at its own row, on the far end of the channel), which is enough to force genuine
// bidirectional negotiation without needing a strategic path layer.
public sealed class CorridorScenario
{
    // Channel geometry: a straight strip y in [0, Width], x in [0, Length], bounded by two thin
    // walls (OrcaCrowd.AddObstacle 2-vertex "thin wall" convention, docs comment on AddObstacle).
    public double Width { get; init; } = 5.0;
    public double Length { get; init; } = 36.0;

    // Agents per direction (total crowd size is 2x this).
    public int AgentsPerGroup { get; init; } = 30;

    public double Radius { get; init; } = 0.3;
    public double MaxSpeed { get; init; } = 1.4;
    public double ArriveRadius { get; init; } = 0.3;
    public double Dt { get; init; } = 0.1;
    public int MaxSteps { get; init; } = 4000;

    // Dense-crowd knobs the design doc calls out by name (MaxNeighbours/SymmetryBreak for the
    // dense counts, UseSpatialHash for the neighbour-query cost at this agent count).
    public int MaxNeighbours { get; init; } = 10;
    public double SymmetryBreak { get; init; } = 0.03;
    public bool UseSpatialHash { get; init; } = true;

    // A pair-wise proximity threshold (m) for the "close head-on encounter" swap-count metric: two
    // opposite-direction agents whose centers ever come this close while both still under way are
    // counted once each. 2*Radius (contact) + a small margin, so it flags genuinely tight passes,
    // not routine comfortable clearance.
    public double SwapProximity { get; init; } = 0.9;

    public CorridorResult Run()
    {
        var crowd = new OrcaCrowd
        {
            UseSpatialHash = UseSpatialHash,
            MaxNeighbours = MaxNeighbours,
            RemoveOnArrival = true,
            ArrivalRadius = ArriveRadius,
            SymmetryBreak = SymmetryBreak,
        };

        // Two thin walls bounding the channel, extended well past the spawn/goal range so no
        // agent can round the ends within the run.
        crowd.AddObstacle(new[] { new Vec2(-10.0, 0.0), new Vec2(Length + 10.0, 0.0) });
        crowd.AddObstacle(new[] { new Vec2(-10.0, Width), new Vec2(Length + 10.0, Width) });

        var total = AgentsPerGroup * 2;
        var indices = new OrcaHandle[total];
        var dir = new int[total];       // +1 == walks +x (group A); -1 == walks -x (group B)
        var group = new bool[total];    // true == group A

        var marginX = 8.0;
        var marginY = 0.5;
        var gridA = GridPositions(1.0, 1.0 + marginX, marginY, Width - marginY, AgentsPerGroup);
        var gridB = GridPositions(Length - 1.0 - marginX, Length - 1.0, marginY, Width - marginY, AgentsPerGroup);

        for (var i = 0; i < AgentsPerGroup; i++)
        {
            var start = gridA[i];
            var goal = new Vec2(Length - 1.0, start.Y);
            indices[i] = crowd.Add(start, Radius, MaxSpeed, goal);
            dir[i] = +1;
            group[i] = true;
        }

        for (var i = 0; i < AgentsPerGroup; i++)
        {
            var idx = AgentsPerGroup + i;
            var start = gridB[i];
            var goal = new Vec2(1.0, start.Y);
            indices[idx] = crowd.Add(start, Radius, MaxSpeed, goal);
            dir[idx] = -1;
            group[idx] = false;
        }

        var trajectory = new List<Vec2[]>();
        var arrivedStep = new int?[total];
        int steps;
        for (steps = 0; steps < MaxSteps; steps++)
        {
            crowd.Step(Dt);

            var frame = new Vec2[total];
            for (var i = 0; i < total; i++)
            {
                frame[i] = crowd.Position(indices[i]);
                if (arrivedStep[i] is null && !crowd.IsActive(indices[i]))
                {
                    arrivedStep[i] = steps;
                }
            }

            trajectory.Add(frame);

            var allArrived = true;
            for (var i = 0; i < total; i++)
            {
                if (arrivedStep[i] is null)
                {
                    allArrived = false;
                    break;
                }
            }

            if (allArrived)
            {
                steps++; // count this step
                break;
            }
        }

        return Measure(trajectory, dir, group, arrivedStep, steps);
    }

    private CorridorResult Measure(
        List<Vec2[]> trajectory, int[] dir, bool[] group, int?[] arrivedStep, int steps)
    {
        var total = dir.Length;
        var arrivedA = 0;
        var arrivedB = 0;
        for (var i = 0; i < total; i++)
        {
            if (arrivedStep[i] is null)
            {
                continue;
            }

            if (group[i])
            {
                arrivedA++;
            }
            else
            {
                arrivedB++;
            }
        }

        var totalTime = steps * Dt;
        var throughputA = totalTime > 0.0 ? arrivedA / totalTime : 0.0;
        var throughputB = totalTime > 0.0 ? arrivedB / totalTime : 0.0;

        // Lane-formation metric: at each frame, correlate every STILL-WALKING agent's lateral
        // offset from the channel's midline against its direction sign, restricted to the
        // "established flow" interior of the channel (away from spawn/goal edge effects). Report
        // both the mean and the peak |r| observed across qualifying frames.
        var midY = Width / 2.0;
        var xLo = Length * 0.25;
        var xHi = Length * 0.75;
        var correlations = new List<double>();
        var lateralSamples = new List<double>();
        var dirSamples = new List<double>();

        for (var f = 0; f < trajectory.Count; f++)
        {
            lateralSamples.Clear();
            dirSamples.Clear();
            var countA = 0;
            var countB = 0;
            var frame = trajectory[f];
            for (var i = 0; i < total; i++)
            {
                if (arrivedStep[i].HasValue && arrivedStep[i]!.Value <= f)
                {
                    continue; // already parked -- excluded from "still walking" lane structure
                }

                var pos = frame[i];
                if (pos.X < xLo || pos.X > xHi)
                {
                    continue; // outside the established-flow interior
                }

                lateralSamples.Add(pos.Y - midY);
                dirSamples.Add(dir[i]);
                if (group[i])
                {
                    countA++;
                }
                else
                {
                    countB++;
                }
            }

            if (countA >= 4 && countB >= 4)
            {
                correlations.Add(CrowdMetrics.PearsonCorrelation(lateralSamples, dirSamples));
            }
        }

        var meanAbsCorrelation = correlations.Count > 0 ? correlations.Select(Math.Abs).Average() : 0.0;
        var peakAbsCorrelation = correlations.Count > 0 ? correlations.Select(Math.Abs).Max() : 0.0;

        // Close head-on encounter count: distinct opposite-direction pairs that ever come within
        // SwapProximity of each other while both are still under way.
        var swapCount = CountCloseEncounters(trajectory, dir, group, arrivedStep, SwapProximity);

        return new CorridorResult(
            Steps: steps,
            SimulatedSeconds: totalTime,
            ArrivedA: arrivedA,
            ArrivedB: arrivedB,
            TotalA: AgentsPerGroup,
            TotalB: AgentsPerGroup,
            ThroughputA: throughputA,
            ThroughputB: throughputB,
            MeanAbsLaneCorrelation: meanAbsCorrelation,
            PeakAbsLaneCorrelation: peakAbsCorrelation,
            CloseHeadOnEncounters: swapCount,
            Trajectory: trajectory);
    }

    private static int CountCloseEncounters(
        List<Vec2[]> trajectory, int[] dir, bool[] group, int?[] arrivedStep, double proximity)
    {
        var total = dir.Length;
        var proxSq = proximity * proximity;
        var seen = new HashSet<(int, int)>();

        for (var f = 0; f < trajectory.Count; f++)
        {
            var frame = trajectory[f];
            for (var a = 0; a < total; a++)
            {
                if (!group[a])
                {
                    continue; // only iterate group-A x group-B pairs, each pair once
                }

                if (arrivedStep[a].HasValue && arrivedStep[a]!.Value <= f)
                {
                    continue;
                }

                for (var b = 0; b < total; b++)
                {
                    if (group[b])
                    {
                        continue;
                    }

                    if (arrivedStep[b].HasValue && arrivedStep[b]!.Value <= f)
                    {
                        continue;
                    }

                    var key = (a, b);
                    if (seen.Contains(key))
                    {
                        continue;
                    }

                    var dx = frame[a].X - frame[b].X;
                    var dy = frame[a].Y - frame[b].Y;
                    if (dx * dx + dy * dy <= proxSq)
                    {
                        seen.Add(key);
                    }
                }
            }
        }

        return seen.Count;
    }

    // Deterministic near-square row-major grid of exactly `count` points filling
    // [xMin,xMax] x [yMin,yMax] -- the "hand-placed" spawn layout for a spawn box. Pure function of
    // its arguments, so runs are reproducible by construction.
    internal static Vec2[] GridPositions(double xMin, double xMax, double yMin, double yMax, int count)
    {
        if (count <= 0)
        {
            return Array.Empty<Vec2>();
        }

        var cols = (int)Math.Ceiling(Math.Sqrt(count));
        var rows = (int)Math.Ceiling((double)count / cols);
        var pts = new Vec2[count];
        var n = 0;
        for (var r = 0; r < rows && n < count; r++)
        {
            var y = rows == 1 ? (yMin + yMax) / 2.0 : yMin + ((yMax - yMin) * r / (rows - 1));
            for (var c = 0; c < cols && n < count; c++)
            {
                var x = cols == 1 ? (xMin + xMax) / 2.0 : xMin + ((xMax - xMin) * c / (cols - 1));
                pts[n++] = new Vec2(x, y);
            }
        }

        return pts;
    }
}

// Measurements from one CorridorScenario.Run() -- see POC-4 success conditions
// (docs/PEDESTRIAN-POC-PLAN.md POC-4 §1): per-direction throughput, the lane-formation metric, the
// close head-on encounter count, and the full trajectory (for determinism comparison across runs).
public sealed record CorridorResult(
    int Steps,
    double SimulatedSeconds,
    int ArrivedA,
    int ArrivedB,
    int TotalA,
    int TotalB,
    double ThroughputA,
    double ThroughputB,
    double MeanAbsLaneCorrelation,
    double PeakAbsLaneCorrelation,
    int CloseHeadOnEncounters,
    List<Vec2[]> Trajectory);
