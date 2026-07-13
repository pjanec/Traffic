using Sim.Core.Mixed;
using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// Behavioural bar for the believable mixed-traffic module (docs/INDIA-TRAFFIC.md section 3). There is
// no SUMO golden for this regime, so correctness is a set of measurable invariants + emergent-
// behaviour checks:
//   HARD INVARIANTS   -- shaped footprints never meaningfully interpenetrate (SAT); vehicles stay on
//                        the drivable surface (walls); runs are bit-identical (determinism).
//   EMERGENT BEHAVIOUR-- anisotropy (a long vehicle is given a wider berth broadside than end-on);
//                        soft priority (a high-assertiveness stream deviates less than a low one).
// Parity-exempt: nothing here touches the lane engine, goldens, or the determinism hash.
public class MixedTrafficBehaviourTests
{
    private readonly ITestOutputHelper _out;

    public MixedTrafficBehaviourTests(ITestOutputHelper output) => _out = output;

    private const double Dt = 0.2;

    // ------------------------------------------------------------------ SAT overlap of two vehicles

    private static Vec2[] WorldPoly(MixedTrafficCrowd c, int i)
    {
        var proto = c.Class(i).Shape().RotatedTo(c.Heading(i));
        var p = c.Position(i);
        var w = new Vec2[proto.Count];
        for (var v = 0; v < proto.Count; v++)
        {
            w[v] = p + proto.Verts[v];
        }

        return w;
    }

    // Penetration depth of two convex polygons (0 if separated), by the separating-axis theorem over
    // both polygons' edge normals. Positive => they overlap by that many metres on the tightest axis.
    private static double Penetration(Vec2[] a, Vec2[] b)
    {
        var minOverlap = double.PositiveInfinity;
        foreach (var poly in new[] { a, b })
        {
            for (var i = 0; i < poly.Length; i++)
            {
                var e = poly[(i + 1) % poly.Length] - poly[i];
                var axis = new Vec2(-e.Y, e.X).Normalized();
                Project(a, axis, out var aMin, out var aMax);
                Project(b, axis, out var bMin, out var bMax);
                var overlap = Math.Min(aMax, bMax) - Math.Max(aMin, bMin);
                if (overlap <= 0.0)
                {
                    return 0.0;   // a separating axis exists -> no overlap
                }

                minOverlap = Math.Min(minOverlap, overlap);
            }
        }

        return minOverlap;
    }

    private static void Project(Vec2[] poly, Vec2 axis, out double min, out double max)
    {
        min = double.PositiveInfinity;
        max = double.NegativeInfinity;
        foreach (var v in poly)
        {
            var d = Vec2.Dot(v, axis);
            min = Math.Min(min, d);
            max = Math.Max(max, d);
        }
    }

    private double MaxPairPenetration(MixedTrafficCrowd c)
    {
        var worst = 0.0;
        for (var i = 0; i < c.Count; i++)
        {
            if (!c.IsActive(i))
            {
                continue;
            }

            var pi = WorldPoly(c, i);
            for (var j = i + 1; j < c.Count; j++)
            {
                if (!c.IsActive(j))
                {
                    continue;
                }

                worst = Math.Max(worst, Penetration(pi, WorldPoly(c, j)));
            }
        }

        return worst;
    }

    // ------------------------------------------------------------------ hard invariant: no overlap

    [Fact]
    public void MixedCrossing_ShapedVehicles_NeverMeaningfullyInterpenetrate()
    {
        // A heterogeneous crossing: an E->W stream of cars/buses and a N->S stream of autos/motos.
        var c = new MixedTrafficCrowd(32) { SymmetryBreak = 0.05, RemoveOnArrival = true };
        var ew = new[] { VehicleClass.Car, VehicleClass.Bus, VehicleClass.Car, VehicleClass.AutoRickshaw };
        for (var i = 0; i < ew.Length; i++)
        {
            c.Add(ew[i], new Vec2(-40, i * 3.0 - 4.5), new Vec2(60, i * 3.0 - 4.5));
        }

        var ns = new[] { VehicleClass.Motorcycle, VehicleClass.AutoRickshaw, VehicleClass.Motorcycle, VehicleClass.Car };
        for (var i = 0; i < ns.Length; i++)
        {
            c.Add(ns[i], new Vec2(i * 3.0 - 4.5, -40), new Vec2(i * 3.0 - 4.5, 60));
        }

        var worst = 0.0;
        for (var step = 0; step < 500 && !c.AllArrived(2.0); step++)
        {
            c.Step(Dt);
            worst = Math.Max(worst, MaxPairPenetration(c));
        }

        _out.WriteLine($"worst penetration over run = {worst:F4} m");
        // Shaped ORCA + discrete stepping can shave a little in tight passing; a light touch (< 0.35 m
        // on vehicles 2-11 m long) is "did not drive through each other".
        Assert.True(worst < 0.35, $"vehicles interpenetrated by {worst:F4} m");
    }

    // ------------------------------------------------------------------ hard invariant: stay on road

    [Fact]
    public void WalledCorridor_VehiclesStayOnTheRoad()
    {
        // A straight corridor along +x, curbs at y = +/-4. Vehicles must not drive through the curb.
        // Curbs at y=+/-4. Walls run WELL beyond the travelled span so no vehicle can round an open
        // end, and every vehicle STARTS inside the walled region (a vehicle spawned outside the curbs
        // is not a confinement test -- it is just outside the road).
        const double curb = 4.0;
        var c = new MixedTrafficCrowd(16) { RemoveOnArrival = true };
        c.AddWall(new Vec2(-40, curb + 0.5), new Vec2(100, curb + 0.5), 1.0);   // top, inner face y=+4
        c.AddWall(new Vec2(-40, -curb - 0.5), new Vec2(100, -curb - 0.5), 1.0); // bottom, inner face y=-4

        var kinds = new[] { VehicleClass.Car, VehicleClass.Motorcycle, VehicleClass.AutoRickshaw, VehicleClass.Bus };
        for (var i = 0; i < kinds.Length; i++)
        {
            var y = i * 1.6 - 2.4;
            c.Add(kinds[i], new Vec2(i * 6.0, y), new Vec2(90, y), headingRad: 0.0);
        }

        var worstIntoCurb = 0.0;
        for (var step = 0; step < 500 && !c.AllArrived(2.0); step++)
        {
            c.Step(Dt);
            for (var i = 0; i < c.Count; i++)
            {
                if (!c.IsActive(i))
                {
                    continue;
                }

                // Footprint extreme in y must not cross the curb inner face at +/-4.
                var proto = c.Class(i).Shape().RotatedTo(c.Heading(i));
                var py = c.Position(i).Y;
                foreach (var v in proto.Verts)
                {
                    var edge = Math.Abs(py + v.Y) - curb;
                    if (edge > worstIntoCurb)
                    {
                        worstIntoCurb = edge;
                    }
                }
            }
        }

        _out.WriteLine($"worst footprint incursion past the curb = {worstIntoCurb:F4} m");
        Assert.True(worstIntoCurb < 0.5, $"a vehicle drove {worstIntoCurb:F4} m into/through the curb");
    }

    // ------------------------------------------------------------------ emergent: anisotropy

    [Fact]
    public void Anisotropy_WiderBerthPassingBroadsideThanEndOn()
    {
        // A car passes a stationary elongated vehicle (a 6x2 m obstacle). Broadside -- its long axis
        // ACROSS the car's path -- presents a wide barrier the car must arc well around; end-on -- long
        // axis ALONG the path -- presents a narrow frontage the car slips past close. So the lateral
        // berth is much larger broadside. The car is offset 2.5 m off-centre so it commits to one side
        // and clears the obstacle in BOTH orientations (a dead-centre approach to a long obstacle sits
        // in an ORCA local minimum -- the holonomic limit noted in INDIA-TRAFFIC.md, not an anisotropy
        // effect -- so we deliberately avoid it here).
        var broadside = CarPassesElongated(broadside: true);
        var endOn = CarPassesElongated(broadside: false);
        _out.WriteLine($"berth passing 6x2 obstacle: broadside={broadside:F2}  end-on={endOn:F2}");
        Assert.True(broadside > 0 && endOn > 0, "car failed to pass in one orientation");
        Assert.True(broadside > endOn + 1.0,
            $"expected a much wider berth broadside ({broadside:F2}) than end-on ({endOn:F2})");
    }

    // Returns the max lateral berth while passing, or -1 if the car never got through.
    private static double CarPassesElongated(bool broadside)
    {
        const double l = 6.0, w = 2.0;
        var c = new MixedTrafficCrowd(4) { SymmetryBreak = 0.05 };
        if (broadside)
        {
            c.AddBlock(-w / 2, -l / 2, w / 2, l / 2, 0.4);   // long axis across the +x path
        }
        else
        {
            c.AddBlock(-l / 2, -w / 2, l / 2, w / 2, 0.4);   // long axis along the +x path
        }

        var car = c.Add(VehicleClass.Car, new Vec2(-30, 2.5), new Vec2(30, 0), headingRad: 0.0);
        var maxBerth = 0.0;
        for (var step = 0; step < 600; step++)
        {
            c.Step(Dt);
            if (Math.Abs(c.Position(car).X) < 20)
            {
                maxBerth = Math.Max(maxBerth, Math.Abs(c.Position(car).Y));
            }

            if (c.Position(car).X > 25)
            {
                return maxBerth;
            }
        }

        return -1;
    }

    // The defining property of a SHAPED velocity obstacle: rotating an elongated obstacle changes the
    // avoidance response. A disc solver is rotation-invariant (a disc has no orientation), so this
    // difference is exactly what the OBB extension buys.
    [Fact]
    public void Anisotropy_RotatingElongatedObstacle_ChangesResponse()
    {
        var car = ConvexShape.Rectangle(4.3, 1.8);
        var boxAlong = ConvexShape.Rectangle(6, 2);
        var boxAcross = ConvexShape.Rectangle(6, 2).RotatedTo(Math.PI / 2);
        var self = new ShapedVoSolver.ShapedAgent(new Vec2(-6, 1.2), new Vec2(12, 0), car);
        var pref = new Vec2(12, 0);
        Span<OrcaLine> sc = stackalloc OrcaLine[4];

        var along = ShapedVoSolver.ComputeNewVelocity(
            self, new[] { new ShapedVoSolver.ShapedAgent(Vec2.Zero, Vec2.Zero, boxAlong, 1.0) },
            pref, 12, 3, Dt, sc);
        var across = ShapedVoSolver.ComputeNewVelocity(
            self, new[] { new ShapedVoSolver.ShapedAgent(Vec2.Zero, Vec2.Zero, boxAcross, 1.0) },
            pref, 12, 3, Dt, sc);

        var diff = (along - across).Abs;
        _out.WriteLine($"response along=({along.X:F2},{along.Y:F2}) across=({across.X:F2},{across.Y:F2}) diff={diff:F2}");
        Assert.True(diff > 0.3, $"shaped VO barely changed with obstacle orientation (diff {diff:F2}) -- not anisotropic");
    }

    // ------------------------------------------------------------------ emergent: soft priority

    [Fact]
    public void SoftPriority_AssertiveStreamDeviatesLessThanYieldingStream()
    {
        // Two crossing single-file streams through the origin: buses (assertive) E->W, motorcycles
        // (yielding) N->S. The assertive stream should hold its line; the yielding one should bend.
        // RemoveOnArrival so a vehicle that has crossed and reached its goal is parked+dropped, not
        // left resting off-axis to pollute the mean; we sample cross-track only for IN-TRANSIT
        // vehicles that are actually in the conflict zone near the crossing.
        var c = new MixedTrafficCrowd(16) { SymmetryBreak = 0.02, RemoveOnArrival = true };
        var buses = new List<int>();
        var motos = new List<int>();
        for (var i = 0; i < 4; i++)
        {
            buses.Add(c.Add(VehicleClass.Bus, new Vec2(-50 - i * 14.0, 0), new Vec2(60, 0), headingRad: 0.0));
        }

        for (var i = 0; i < 4; i++)
        {
            motos.Add(c.Add(VehicleClass.Motorcycle, new Vec2(0, -50 - i * 6.0), new Vec2(0, 60), headingRad: Math.PI / 2));
        }

        double busCross = 0, motoCross = 0;
        int busSamples = 0, motoSamples = 0;
        for (var step = 0; step < 700 && !c.AllArrived(2.0); step++)
        {
            c.Step(Dt);
            foreach (var b in buses)
            {
                if (c.IsActive(b) && Math.Abs(c.Position(b).X) < 25)   // in the conflict zone
                {
                    busCross += Math.Abs(c.Position(b).Y);
                    busSamples++;
                }
            }

            foreach (var m in motos)
            {
                if (c.IsActive(m) && Math.Abs(c.Position(m).Y) < 25)
                {
                    motoCross += Math.Abs(c.Position(m).X);
                    motoSamples++;
                }
            }
        }

        busCross /= Math.Max(1, busSamples);
        motoCross /= Math.Max(1, motoSamples);
        _out.WriteLine($"mean cross-track in conflict zone: buses={busCross:F3}  motorcycles={motoCross:F3}");
        Assert.True(motoCross > busCross * 1.5,
            $"expected the yielding stream to deviate more (motos {motoCross:F3} vs buses {busCross:F3})");
    }

    // ------------------------------------------------------------------ non-holonomic (car-like) motion

    // The holonomic model lets a vehicle pick any velocity each step -- including backward and
    // sideways -- so in congestion heading snaps 180 deg and the motion looks like "bacteria under a
    // microscope". Non-holonomic steering must produce CAR-LIKE motion: no reversing, bounded per-step
    // heading change (no pivot-in-place), smooth speed. This drives a dense crossing under NH and
    // asserts those invariants directly on the trajectories.
    [Fact]
    public void Nonholonomic_DenseCrossing_NoReverse_NoPivot_SmoothHeading()
    {
        var c = new MixedTrafficCrowd(48)
        {
            Nonholonomic = true,
            SafetyMargin = 0.6,
            SymmetryBreak = 0.03,
            MaxNeighbours = 10,
            RemoveOnArrival = true,
            ArrivalRadius = 3.0,
            NeighbourDist = 18.0,
            TimeHorizon = 3.0,
        };

        // Two crossing streams of mixed vehicles that must interleave through the origin.
        var ew = new[] { VehicleClass.Bus, VehicleClass.Car, VehicleClass.Car, VehicleClass.AutoRickshaw };
        for (var i = 0; i < ew.Length; i++)
        {
            c.Add(ew[i], new Vec2(-45 - i * 12, i * 3.0 - 4.5), new Vec2(60, i * 3.0 - 4.5),
                headingRad: 0.0, maxSpeedOverride: ew[i].MaxSpeed * 0.4);
        }

        var ns = new[] { VehicleClass.Motorcycle, VehicleClass.AutoRickshaw, VehicleClass.Motorcycle, VehicleClass.Car };
        for (var i = 0; i < ns.Length; i++)
        {
            c.Add(ns[i], new Vec2(i * 3.0 - 4.5, -45 - i * 12), new Vec2(i * 3.0 - 4.5, 60),
                headingRad: Math.PI / 2, maxSpeedOverride: ns[i].MaxSpeed * 0.4);
        }

        var n = c.Count;
        var prevHeading = new double[n];
        var prevVel = new Vec2[n];
        var seen = new bool[n];
        var maxHeadingStep = 0.0;
        var reverseFlips = 0;
        var backwardMotion = 0;

        for (var step = 0; step < 500 && !c.AllArrived(2.5); step++)
        {
            c.Step(Dt);
            for (var i = 0; i < n; i++)
            {
                if (!c.IsActive(i))
                {
                    continue;
                }

                var h = c.Heading(i);
                var v = c.Velocity(i);

                // Motion is always forward along the heading (never reverse): v . heading >= -eps.
                var fwd = Vec2.Dot(v, new Vec2(Math.Cos(h), Math.Sin(h)));
                if (v.Abs > 0.05 && fwd < -0.05)
                {
                    backwardMotion++;
                }

                if (seen[i])
                {
                    var dh = Math.Abs(Math.Atan2(Math.Sin(h - prevHeading[i]), Math.Cos(h - prevHeading[i])));
                    maxHeadingStep = Math.Max(maxHeadingStep, dh);
                    if (v.Abs > 0.2 && prevVel[i].Abs > 0.2 && Vec2.Dot(v, prevVel[i]) < 0)
                    {
                        reverseFlips++;
                    }
                }

                prevHeading[i] = h;
                prevVel[i] = v;
                seen[i] = true;
            }
        }

        _out.WriteLine($"NH: max heading change/step = {maxHeadingStep * 180 / Math.PI:F1} deg, "
            + $"reverse-flips = {reverseFlips}, backward-motion steps = {backwardMotion}");
        Assert.Equal(0, backwardMotion);       // never drives backward along its heading
        Assert.Equal(0, reverseFlips);          // velocity never flips ~180 deg step-to-step
        // A single step can never spin more than the nimblest vehicle's speed*dt/Rmin bound (~55 deg
        // for a fast motorcycle); no 180-degree "bacteria" flips.
        Assert.True(maxHeadingStep < 70 * Math.PI / 180,
            $"heading jumped {maxHeadingStep * 180 / Math.PI:F0} deg in one step (pivot/flip)");
    }

    // A wheeled vehicle pivots about its REAR AXLE, so through a turn the rear tracks a tighter arc
    // than the front (off-tracking) -- NOT a body-centre spin where both ends sweep equally. Drive a
    // single bus through a ~90 deg turn and confirm the front bumper travels a measurably LONGER path
    // than the rear axle over the turning phase.
    [Fact]
    public void Nonholonomic_LongVehicle_RearAxleTracksInsideTurn()
    {
        var c = new MixedTrafficCrowd(2) { Nonholonomic = true, TimeHorizon = 3.0 };
        var bus = c.Add(VehicleClass.Bus, new Vec2(-30, 0), new Vec2(0, 30), headingRad: 0.0,
            maxSpeedOverride: 6.0);
        var halfL = VehicleClass.Bus.Length * 0.5;

        double frontPath = 0, rearPath = 0;
        Vec2? prevFront = null, prevRear = null;
        for (var step = 0; step < 120; step++)
        {
            c.Step(Dt);
            var p = c.Position(bus);
            var h = c.Heading(bus);
            var dir = new Vec2(Math.Cos(h), Math.Sin(h));
            var front = new Vec2(p.X + halfL * dir.X, p.Y + halfL * dir.Y);
            var rear = new Vec2(p.X - halfL * dir.X, p.Y - halfL * dir.Y);
            var hd = h * 180 / Math.PI;
            if (prevFront.HasValue && hd > 5 && hd < 85)   // accumulate only during the turn
            {
                frontPath += (front - prevFront.Value).Abs;
                rearPath += (rear - prevRear!.Value).Abs;
            }

            prevFront = front;
            prevRear = rear;
        }

        _out.WriteLine($"through the turn: front path = {frontPath:F2} m, rear path = {rearPath:F2} m");
        Assert.True(frontPath > rearPath + 1.0,
            $"front should sweep a longer arc than the rear (off-tracking); front {frontPath:F2} vs rear {rearPath:F2}");
    }

    // ------------------------------------------------------------------ hard invariant: determinism

    [Fact]
    public void Deterministic_IdenticalRuns_AreBitIdentical()
    {
        var a = RunScenario();
        var b = RunScenario();
        Assert.Equal(a.Count, b.Count);
        for (var i = 0; i < a.Count; i++)
        {
            Assert.Equal(a[i].X, b[i].X);
            Assert.Equal(a[i].Y, b[i].Y);
        }
    }

    private static List<Vec2> RunScenario()
    {
        var c = new MixedTrafficCrowd(12) { SymmetryBreak = 0.05 };
        c.Add(VehicleClass.Bus, new Vec2(-30, 0), new Vec2(30, 0));
        c.Add(VehicleClass.Car, new Vec2(30, 1.5), new Vec2(-30, 1.5));
        c.Add(VehicleClass.Motorcycle, new Vec2(0, -30), new Vec2(0, 30));
        c.Add(VehicleClass.AutoRickshaw, new Vec2(0, 30), new Vec2(0, -30));
        var trace = new List<Vec2>();
        for (var step = 0; step < 120; step++)
        {
            c.Step(Dt);
            for (var i = 0; i < c.Count; i++)
            {
                trace.Add(c.Position(i));
            }
        }

        return trace;
    }
}
