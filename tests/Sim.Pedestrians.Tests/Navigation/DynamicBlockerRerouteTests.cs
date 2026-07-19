using Sim.Core.Orca;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Navigation;

// P2-2 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §4/§6): the production dynamic-blocker
// registry (BlockerRegistry) + reroute driver (RerouteDriver) built on top of POC-5's blocked-set
// FindPath overload (SumoNavMesh.FindPath(start, goal, blocked), covered directly by RerouteTests.cs).
// Reuses the same committed POC-0 fixture and junction geometry RerouteTests/SumoBakeNavigationTests
// already rely on: junction "c"'s north crossing ":c_c0_0" sits on the shortest path between the two
// parallel north-arm sidewalks (nc_0 / cn_0), and the junction's walkingarea/crossing ring provides a
// genuine, longer detour once it is blocked -- verified empirically in RerouteTests.
public class DynamicBlockerRerouteTests
{
    private const double MaxSpeed = 1.4;    // m/s
    private const double Radius = 0.3;      // m
    private const double ArriveRadius = 0.3; // m
    private const double Dt = 0.1;          // s
    private const double DebounceSeconds = 0.5;    // blocker must persist 5 steps before it "counts"
    private const double CommitDwellSeconds = 1.0; // a rerouted ped can't reroute again for 10 steps
    private const int MaxSteps = 4000;

    // Same POC-0 junction points RerouteTests/SumoBakeNavigationTests use: two parallel north-arm
    // sidewalks whose only connection is through the junction's crossing/walkingarea ring.
    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);

    // Two points further up the SAME sidewalk (nc_0, x in [111.6, 113.6]) as WestNorthArm, well clear
    // of the junction -- a ped routing between these never comes near the north crossing at all.
    private static readonly Vec2 FarNorthA = new(112.6, 200.0);
    private static readonly Vec2 FarNorthB = new(112.6, 215.0);

    private readonly ITestOutputHelper _out;

    public DynamicBlockerRerouteTests(ITestOutputHelper output) => _out = output;

    private static PedNetwork LoadPoc0Network() => PedNetworkParser.Load(
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml"));

    private static (IReadOnlyList<BakedPolygon> Polygons, SumoWalkableSpace Space, SumoNavMesh Nav) BuildProvider()
    {
        var polygons = WalkablePolygonBaker.Bake(LoadPoc0Network());
        var space = new SumoWalkableSpace(polygons);
        var nav = new SumoNavMesh(polygons, space);
        return (polygons, space, nav);
    }

    // A blocker box centered on northCrossing's own bounding box, sized to comfortably overlap ITS
    // interior without spilling into the perpendicular east/west crossings that share the same
    // junction corners a few metres away (a crossing polygon here is not a plain rectangle -- it
    // tapers into a funnel shape at both ends where it meets the corner walkingareas, verified
    // empirically for POC-0's junction "c": a box sized off the crossing's FULL bounding box (e.g.
    // 90% of width) reaches past that taper and picks up the neighbouring walkingareas AND the
    // perpendicular crossings too, which can disconnect the whole detour ring. 60% of the
    // bounding-box WIDTH (centered) stays within the crossing's straight middle section; 90% of the
    // (much smaller) HEIGHT is safely inside its own thickness.
    private static Vec2[] BoxCoveringPolygon(BakedPolygon polygon)
    {
        var minX = polygon.Vertices.Min(v => v.X);
        var maxX = polygon.Vertices.Max(v => v.X);
        var minY = polygon.Vertices.Min(v => v.Y);
        var maxY = polygon.Vertices.Max(v => v.Y);
        var center = new Vec2((minX + maxX) / 2.0, (minY + maxY) / 2.0);
        var halfX = (maxX - minX) / 2.0 * 0.6;
        var halfY = (maxY - minY) / 2.0 * 0.9;
        return BoxObstacle.Corners(center, halfX, halfY, angleRadians: 0.0);
    }

    // Dense-sample test, identical in spirit and nuance to RerouteTests.cs's own PathEntersPolygon:
    // true when `path` MEANINGFULLY enters `polygon`'s interior at any point along its length -- not
    // a plain point-in-polygon test. WalkablePolygonBaker bakes each walkable surface (crossings,
    // walkingareas, sidewalk strips) as an INDEPENDENT polygon from SUMO's real junction geometry, and
    // at a busy junction corner these independently-baked shapes legitimately OVERLAP each other by a
    // shallow sliver where they abut (a detour through an ADJACENT crossing/walkingarea necessarily
    // brushes past the shared corner of the very polygon it is routing AROUND). A route that only
    // grazes that shallow shared sliver has not actually traversed the polygon as a crossing; one that
    // goes deep into its interior still is. See RerouteTests.cs's PathEntersPolygon/MeaningfullyInside
    // remarks for the full empirical justification (verified against this same POC-0 fixture).
    private static bool PathEntersPolygon(IReadOnlyList<Vec2> path, BakedPolygon polygon)
    {
        for (var i = 0; i + 1 < path.Count; i++)
        {
            var a = path[i];
            var b = path[i + 1];
            var length = (b - a).Abs;
            var steps = Math.Max(20, (int)(length / 0.1));
            for (var s = 0; s <= steps; s++)
            {
                var t = (double)s / steps;
                var sample = new Vec2(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t));
                if (MeaningfullyInside(polygon, sample))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private static bool MeaningfullyInside(BakedPolygon polygon, Vec2 p)
    {
        var single = new SumoWalkableSpace(new[] { polygon });
        if (!single.Contains(p))
        {
            return false;
        }

        return DistanceToBoundary(polygon.Vertices, p) > Radius;
    }

    private static double DistanceToBoundary(IReadOnlyList<Vec2> vertices, Vec2 p)
    {
        var n = vertices.Count;
        var best = double.MaxValue;
        for (var i = 0; i < n; i++)
        {
            var a = vertices[i];
            var b = vertices[(i + 1) % n];
            var ab = b - a;
            var abLenSq = ab.AbsSq;
            var t = abLenSq > 1e-12 ? Math.Clamp(Vec2.Dot(p - a, ab) / abLenSq, 0.0, 1.0) : 0.0;
            var candidate = a + (t * ab);
            var dist = (candidate - p).Abs;
            if (dist < best)
            {
                best = dist;
            }
        }

        return best;
    }

    private static bool PathsEqual(IReadOnlyList<Vec2> a, IReadOnlyList<Vec2> b)
    {
        if (a.Count != b.Count)
        {
            return false;
        }

        for (var i = 0; i < a.Count; i++)
        {
            if (a[i].X != b[i].X || a[i].Y != b[i].Y)
            {
                return false;
            }
        }

        return true;
    }

    [Fact]
    public void RegisteringBlocker_ReroutesOnlyAffectedPed_UnregisteringRestoresDirectPathOnFreshQuery()
    {
        var (polygons, space, nav) = BuildProvider();
        var northCrossing = polygons.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");

        var directPath = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(directPath);
        Assert.True(PathEntersPolygon(directPath!, northCrossing), "test setup invalid: direct path should use the north crossing");

        var farPath = nav.FindPath(FarNorthA, FarNorthB);
        Assert.NotNull(farPath);
        Assert.False(PathEntersPolygon(farPath!, northCrossing), "test setup invalid: far-north path should never approach the north crossing");

        var registry = new BlockerRegistry(polygons);
        var crowd = new OrcaCrowd();
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var driver = new RerouteDriver(crowd, nav, polygons, DebounceSeconds, CommitDwellSeconds);

        var affected = crowd.Add(WestNorthArm, Radius, MaxSpeed, goal: directPath![0]);
        controller.AddRoute(affected, directPath, MaxSpeed);
        driver.RegisterPed(affected, EastNorthArm, directPath);

        var unaffected = crowd.Add(FarNorthA, Radius, MaxSpeed, goal: farPath![0]);
        controller.AddRoute(unaffected, farPath, MaxSpeed);
        driver.RegisterPed(unaffected, FarNorthB, farPath);

        var time = 0.0;
        var lastEventCount = 0;
        BlockerId? blockerId = null;
        var arrived = false;
        var step = 0;

        for (; step < MaxSteps; step++)
        {
            controller.Update();
            crowd.Step(Dt);
            time += Dt;

            if (step == 2)
            {
                blockerId = registry.Register(BoxCoveringPolygon(northCrossing));
            }

            driver.Update(time, registry.BlockedPolygons());

            if (driver.Events.Count > lastEventCount)
            {
                for (var e = lastEventCount; e < driver.Events.Count; e++)
                {
                    var evt = driver.Events[e];
                    controller.AddRoute(evt.Ped, driver.PathOf(evt.Ped), MaxSpeed);
                    _out.WriteLine($"[t={evt.Time:F2}] reroute ped(idx={evt.Ped.Index}): {evt.OldWaypointCount} -> {evt.NewWaypointCount} waypoints");
                }

                lastEventCount = driver.Events.Count;
            }

            Assert.True(space.Contains(crowd.Position(affected)), $"affected ped left walkable space at step {step}");

            if ((crowd.Position(affected) - EastNorthArm).Abs <= ArriveRadius && controller.IsRouteComplete(affected))
            {
                arrived = true;
                break;
            }
        }

        Assert.True(arrived, "affected ped never arrived at its goal");
        Assert.True(blockerId is { IsValid: true });

        // (1) exactly one reroute happened, and it was for the AFFECTED ped only.
        Assert.Single(driver.Events);
        Assert.Equal(affected, driver.Events[0].Ped);

        // (2) the affected ped's committed path is genuinely different, avoids the blocked crossing,
        // stays walkable end-to-end, and still reaches the goal.
        var reroutedPath = driver.PathOf(affected);
        Assert.False(PathsEqual(directPath, reroutedPath));
        Assert.False(PathEntersPolygon(reroutedPath, northCrossing), "rerouted path still traverses the blocked crossing");
        for (var i = 0; i + 1 < reroutedPath.Count; i++)
        {
            var a = reroutedPath[i];
            var b = reroutedPath[i + 1];
            for (var s = 0; s <= 20; s++)
            {
                var t = s / 20.0;
                var sample = new Vec2(a.X + ((b.X - a.X) * t), a.Y + ((b.Y - a.Y) * t));
                Assert.True(space.Contains(sample), $"rerouted path segment {i}->{i + 1} at t={t:F2} left walkable space");
            }
        }

        Assert.True((reroutedPath[^1] - EastNorthArm).Abs < 1e-6);

        // (3) the unaffected ped's path/handle were never touched.
        Assert.True(ReferenceEquals(farPath, driver.PathOf(unaffected)));

        // (4) unregister -> the registry's blocked set clears, and a FRESH nav query (independent of
        // the already-rerouted ped's own driver-managed state) is the direct path again.
        Assert.True(registry.Unregister(blockerId!.Value));
        Assert.DoesNotContain(northCrossing.Index, registry.BlockedPolygons());

        var freshPath = nav.FindPath(WestNorthArm, EastNorthArm, registry.BlockedPolygons());
        Assert.NotNull(freshPath);
        Assert.True(PathsEqual(directPath, freshPath!), "a fresh route after unregister should be the direct path again");

        _out.WriteLine($"[measured] direct path waypoints: {directPath.Count}; rerouted path waypoints: {reroutedPath.Count}; arrived in {step} steps ({step * Dt:F1}s)");
    }

    [Fact]
    public void FlickerFasterThanDebounce_CausesZeroReroutes()
    {
        var (polygons, space, nav) = BuildProvider();
        var northCrossing = polygons.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");
        var directPath = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(directPath);

        var registry = new BlockerRegistry(polygons);
        var crowd = new OrcaCrowd();
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var driver = new RerouteDriver(crowd, nav, polygons, DebounceSeconds, CommitDwellSeconds);

        var affected = crowd.Add(WestNorthArm, Radius, MaxSpeed, goal: directPath![0]);
        controller.AddRoute(affected, directPath, MaxSpeed);
        driver.RegisterPed(affected, EastNorthArm, directPath);

        var box = BoxCoveringPolygon(northCrossing);

        var time = 0.0;
        // Flicker register/unregister every OTHER step (0.2 s cadence) -- faster than the 0.5 s
        // debounce, so the crossing never accumulates 0.5 s of continuous raw-blocked time.
        for (var step = 0; step < 200; step++)
        {
            controller.Update();
            crowd.Step(Dt);
            time += Dt;

            if (step % 4 == 0)
            {
                registry.Register(box); // note: Register returns a NEW id each flicker-on, deliberately
            }
            else if (step % 4 == 2)
            {
                foreach (var (id, _) in registry.Snapshot())
                {
                    registry.Unregister(id);
                }
            }

            driver.Update(time, registry.BlockedPolygons());
        }

        Assert.Empty(driver.Events);
        Assert.True(PathsEqual(directPath, driver.PathOf(affected)), "flickering blocker (never debounced) must never touch the ped's path");
        _out.WriteLine($"[measured] flicker (period < debounce) reroute count: {driver.Events.Count}");
    }

    [Fact]
    public void FlickerAroundCommitBoundary_WhileMidReroute_BoundsRerouteCount()
    {
        var (polygons, space, nav) = BuildProvider();
        var northCrossing = polygons.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");
        var directPath = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(directPath);

        var registry = new BlockerRegistry(polygons);
        var crowd = new OrcaCrowd();
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var driver = new RerouteDriver(crowd, nav, polygons, DebounceSeconds, CommitDwellSeconds);

        var affected = crowd.Add(WestNorthArm, Radius, MaxSpeed, goal: directPath![0]);
        controller.AddRoute(affected, directPath, MaxSpeed);
        driver.RegisterPed(affected, EastNorthArm, directPath);

        var box = BoxCoveringPolygon(northCrossing);

        var time = 0.0;
        var lastEventCount = 0;
        var arrived = false;
        var step = 0;

        // Register once and leave it registered long enough to clear the debounce -- this SHOULD
        // reroute the ped exactly once, putting it into its commit-dwell window.
        var blockerId = registry.Register(box);

        for (; step < MaxSteps; step++)
        {
            controller.Update();
            crowd.Step(Dt);
            time += Dt;

            // Once the first reroute has committed (ped now mid-commit-dwell), start flickering the
            // SAME blocker on/off right around the debounce boundary every few steps -- exactly the
            // "flickers around the commit boundary while a ped is mid-reroute" scenario.
            if (driver.Events.Count > 0 && step % 3 == 0)
            {
                if (registry.Contains(blockerId))
                {
                    registry.Unregister(blockerId);
                }
                else
                {
                    blockerId = registry.Register(box);
                }
            }

            driver.Update(time, registry.BlockedPolygons());

            if (driver.Events.Count > lastEventCount)
            {
                for (var e = lastEventCount; e < driver.Events.Count; e++)
                {
                    var evt = driver.Events[e];
                    controller.AddRoute(evt.Ped, driver.PathOf(evt.Ped), MaxSpeed);
                }

                lastEventCount = driver.Events.Count;
            }

            if ((crowd.Position(affected) - EastNorthArm).Abs <= ArriveRadius && controller.IsRouteComplete(affected))
            {
                arrived = true;
                break;
            }
        }

        Assert.True(arrived, "affected ped never arrived despite the flickering blocker");
        Assert.True(driver.Events.Count is >= 1 and <= 2, $"expected a bounded (1-2) reroute count, got {driver.Events.Count}");
        _out.WriteLine($"[measured] boundary-flicker reroute count: {driver.Events.Count}");
    }

    // Runs the full "register -> debounce -> reroute -> flicker -> unregister" scenario once, driven
    // by a fixed step-indexed script, and returns the reroute event sequence + the final committed
    // path for the affected ped -- shared by the determinism test below to compare two independent runs.
    private static (IReadOnlyList<RerouteEvent> Events, IReadOnlyList<Vec2> FinalPath) RunScenario(
        IReadOnlyList<BakedPolygon> polygons, SumoWalkableSpace space, SumoNavMesh nav, BakedPolygon northCrossing)
    {
        var directPath = nav.FindPath(WestNorthArm, EastNorthArm)!;
        var registry = new BlockerRegistry(polygons);
        var crowd = new OrcaCrowd();
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var driver = new RerouteDriver(crowd, nav, polygons, DebounceSeconds, CommitDwellSeconds);

        var affected = crowd.Add(WestNorthArm, Radius, MaxSpeed, goal: directPath[0]);
        controller.AddRoute(affected, directPath, MaxSpeed);
        driver.RegisterPed(affected, EastNorthArm, directPath);

        var box = BoxCoveringPolygon(northCrossing);
        var time = 0.0;
        var lastEventCount = 0;

        for (var step = 0; step < MaxSteps; step++)
        {
            controller.Update();
            crowd.Step(Dt);
            time += Dt;

            if (step == 2)
            {
                registry.Register(box);
            }
            else if (step == 60)
            {
                foreach (var (id, _) in registry.Snapshot())
                {
                    registry.Unregister(id);
                }
            }

            driver.Update(time, registry.BlockedPolygons());

            if (driver.Events.Count > lastEventCount)
            {
                for (var e = lastEventCount; e < driver.Events.Count; e++)
                {
                    controller.AddRoute(driver.Events[e].Ped, driver.PathOf(driver.Events[e].Ped), MaxSpeed);
                }

                lastEventCount = driver.Events.Count;
            }

            if ((crowd.Position(affected) - EastNorthArm).Abs <= ArriveRadius && controller.IsRouteComplete(affected))
            {
                break;
            }
        }

        return (driver.Events, driver.PathOf(affected));
    }

    [Fact]
    public void Determinism_WholeScenario_IsIdenticalAcrossRuns()
    {
        var (polygons1, space1, nav1) = BuildProvider();
        var northCrossing1 = polygons1.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");
        var (events1, path1) = RunScenario(polygons1, space1, nav1, northCrossing1);

        var (polygons2, space2, nav2) = BuildProvider();
        var northCrossing2 = polygons2.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");
        var (events2, path2) = RunScenario(polygons2, space2, nav2, northCrossing2);

        Assert.Equal(events1.Count, events2.Count);
        for (var i = 0; i < events1.Count; i++)
        {
            Assert.Equal(events1[i].Time, events2[i].Time, precision: 12);
            Assert.Equal(events1[i].Ped, events2[i].Ped);
            Assert.Equal(events1[i].OldWaypointCount, events2[i].OldWaypointCount);
            Assert.Equal(events1[i].NewWaypointCount, events2[i].NewWaypointCount);
        }

        Assert.True(PathsEqual(path1, path2), "the two runs' final committed paths must be bit-identical");
        _out.WriteLine($"[measured] determinism run: {events1.Count} reroute event(s), final path {path1.Count} waypoints");
    }
}
