using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// POC-3 (docs/PEDESTRIAN-POC-PLAN.md): sim-LOD promotion/demotion + the PathArc<->FreeKinematic DR
// switch, proven end-to-end with an in-process headless "IG" reconstructor. Reuses the committed POC-0
// fixture net + POC-1a navigation pieces (PedNetworkParser / WalkablePolygonBaker / SumoNavMesh) for a
// real navmesh path, exactly like SumoBakeNavigationTests.
public class PedLodManagerTests
{
    private readonly ITestOutputHelper _output;

    public PedLodManagerTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const double MaxSpeed = 1.4;   // m/s
    private const double Radius = 0.3;     // m
    private const double ArriveRadius = 0.3; // m
    private const double Dt = 0.1;         // s
    private const double DwellSeconds = 1.0; // s
    private const double PromoteRadius = 3.0; // m
    private const double DemoteRadius = 6.0;  // m

    // Same POC-0 junction points SumoBakeNavigationTests uses: routing between them forces a real,
    // multi-waypoint path through the junction's walkingarea/crossing/walkingarea chain.
    private static readonly Vec2 WestNorthArm = new(112.6, 140.0);
    private static readonly Vec2 EastNorthArm = new(127.4, 140.0);

    private static SumoNavMesh BuildNav()
    {
        var polygons = WalkablePolygonBaker.Bake(LoadPoc0Network());
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    private static PedNetwork LoadPoc0Network() => PedNetworkParser.Load(
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml"),
        Path.Combine(AppContext.BaseDirectory, "Fixtures", "walkable.add.xml"));

    // ---- Success condition 1: server == IG for low-power --------------------------------------

    [Fact]
    public void LowPower_ServerPositionMatchesHeadlessIgReconstruction_AtEverySampledTime()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        // No interest sources anywhere near the route -> stays low-power (PathArc) the whole run.
        var straightLineDist = (EastNorthArm - WestNorthArm).Abs;
        var totalTime = (straightLineDist / MaxSpeed) * 3.0; // slack, mirrors POC-1a's convention
        var steps = (int)(totalTime / Dt);

        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;
        for (var i = 0; i < steps; i++)
        {
            manager.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        Assert.Equal(PedDrModel.PathArc, manager.ModelOf(1));

        // The IG learns the path from the ONE spawn-time PathArcRecord -- nothing else was published
        // for this ped (asserted separately below), so this exercises exactly the "path sent once"
        // reconstruction path.
        var ig = new HeadlessIg();
        ig.ApplyAll(publisher.Events);
        Assert.Equal(PedDrModel.PathArc, ig.ModelOf(1));

        var maxError = 0.0;
        for (var t = 0.0; t <= now; t += Dt / 3.0) // finer grid than the step size -- a pure-function check
        {
            var serverPos = manager.PositionOf(1, t);
            var igPos = ig.Reconstruct(1, t);
            var error = (serverPos - igPos).Abs;
            maxError = Math.Max(maxError, error);
        }

        _output.WriteLine($"[POC-3 measured] server-vs-IG max position error (low-power): {maxError:E3} m");
        Assert.True(maxError < 1e-9, $"max server-vs-IG position error {maxError:E3} exceeded 1e-9");

        // Success condition 4, checked here too since this run never promotes: silent on the wire.
        Assert.False(publisher.FreeKinematicSamplesSent.ContainsKey(1) && publisher.FreeKinematicSamplesSent[1] > 0);
        Assert.Equal(1, publisher.PathArcRecordsSent[1]);
        Assert.True(publisher.HeartbeatsSent.TryGetValue(1, out var heartbeats) && heartbeats > 0,
            "expected at least one heartbeat over a multi-second low-power run");
    }

    // ---- Success condition 2: promotion + avoidance --------------------------------------------

    [Fact]
    public void Promotion_SwitchesToFreeKinematic_AndPromotedPedAvoidsTheStimulus()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        const double entityRadius = 0.3;
        // Starts ON the ped (promotes fast) and then CHASES it every step (offset just ahead, on its
        // route toward EastNorthArm) -- an avatar walking alongside, per docs/PEDESTRIAN-DESIGN.md §5
        // use case 1. Chasing keeps the entity within the (hysteretic) demote radius for the whole run,
        // so this test exercises promotion AND sustained reactive avoidance, not just a one-shot flip.
        var entity = new InterestSource(WestNorthArm, PromoteRadius, DemoteRadius);
        var field = new InterestField();
        var entityId = field.Register(entity, InterestSourceKind.EntityAttached);

        var ig = new HeadlessIg();
        var sumOfRadii = Radius + entityRadius;
        const double overlapEps = 1e-3;
        var minSeparation = double.MaxValue;
        var everPromoted = false;

        var now = 0.0;
        for (var i = 0; i < 200; i++)
        {
            var beforeCount = publisher.Events.Count;
            manager.Step(now, Dt, field, new[] { new WorldDisc(entity.Position.X, entity.Position.Y, 0, 0, entityRadius) });
            now += Dt;
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                ig.Apply(publisher.Events[e]);
            }

            if (manager.ModelOf(1) == PedDrModel.FreeKinematic)
            {
                everPromoted = true;
                var pedPos = manager.PositionOf(1, now);
                var separation = (pedPos - entity.Position).Abs;
                minSeparation = Math.Min(minSeparation, separation);
            }

            // Chase: keep the entity ~1 m from the ped's current position (well inside PromoteRadius,
            // and always inside DemoteRadius too) so it stays a live avoidance stimulus for the ped's
            // whole promoted run instead of being outrun and passively lapsing out of range.
            field.Move(entityId, manager.PositionOf(1, now) + new Vec2(1.0, 0.0));
        }

        Assert.True(everPromoted, "ped never promoted despite the stimulus sitting on top of it");
        Assert.Equal(PedDrModel.FreeKinematic, manager.ModelOf(1));
        Assert.Equal(PedDrModel.FreeKinematic, ig.ModelOf(1)); // the IG observed the switch on the wire

        Assert.Contains(publisher.Events, e => e is DrSwitchEvent { From: PedDrModel.PathArc, To: PedDrModel.FreeKinematic });

        _output.WriteLine($"[POC-3 measured] min ped-entity separation while promoted: {minSeparation:F4} m (sum-of-radii {sumOfRadii:F4} m)");
        Assert.True(minSeparation >= sumOfRadii - overlapEps,
            $"minimum ped-entity separation ({minSeparation:F4}) violated sum-of-radii ({sumOfRadii:F4}) - eps");
    }

    // ---- Success condition 3a: demotion after the stimulus is removed ---------------------------

    [Fact]
    public void Demotion_ReattachesFreshPathArc_AfterStimulusLeavesAndDwellElapses()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        var entity = new InterestSource(WestNorthArm, PromoteRadius, DemoteRadius);
        var field = new InterestField();
        var entityId = field.Register(entity, InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;

        // Promote: stimulus sits on the ped until it goes high-power.
        while (manager.ModelOf(1) != PedDrModel.FreeKinematic && now < 30.0)
        {
            manager.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        Assert.Equal(PedDrModel.FreeKinematic, manager.ModelOf(1));

        // Remove the stimulus far away permanently; step through the dwell.
        field.Move(entityId, new Vec2(-10_000, -10_000));
        var pathArcRecordsBeforeDemotion = publisher.PathArcRecordsSent.GetValueOrDefault(1);

        var demotedAt = -1.0;
        for (var i = 0; i < 300 && manager.ModelOf(1) == PedDrModel.FreeKinematic; i++)
        {
            manager.Step(now, Dt, field, noEntities);
            now += Dt;
            if (manager.ModelOf(1) == PedDrModel.PathArc)
            {
                demotedAt = now;
            }
        }

        Assert.Equal(PedDrModel.PathArc, manager.ModelOf(1));
        Assert.True(demotedAt >= 0, "ped never demoted");

        Assert.Contains(publisher.Events, e => e is DrSwitchEvent { From: PedDrModel.FreeKinematic, To: PedDrModel.PathArc });
        Assert.True(publisher.PathArcRecordsSent[1] > pathArcRecordsBeforeDemotion,
            "expected a fresh PathArcRecord on demotion");
    }

    // ---- Success condition 3b: no flapping when the stimulus hovers at the demote boundary ------

    [Fact]
    public void Demotion_DoesNotFlap_WhenStimulusHoversAtTheDemoteBoundary()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        var source = new InterestSource(WestNorthArm, PromoteRadius, DemoteRadius);
        var field = new InterestField();
        var sourceId = field.Register(source);
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;
        for (var i = 0; i < 400; i++)
        {
            var pedPos = manager.PositionOf(1, now);
            var nextPos = manager.ModelOf(1) == PedDrModel.PathArc
                // still low: sit well inside the promote radius to force (and keep) the first promotion.
                ? pedPos + new Vec2(PromoteRadius * 0.2, 0.0)
                // once high: bounce the stimulus just inside / just outside the DEMOTE radius every
                // single step -- the classic flapping stimulus -- chasing the ped so it never simply
                // "walks away" from a stationary source.
                : pedPos + new Vec2(i % 2 == 0 ? DemoteRadius - 0.1 : DemoteRadius + 0.1, 0.0);
            field.Move(sourceId, nextPos);

            manager.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        var promotions = publisher.Events.Count(e => e is DrSwitchEvent { To: PedDrModel.FreeKinematic });
        _output.WriteLine($"[POC-3 measured] promotion count under boundary-hovering stimulus (400 steps): {promotions}");
        Assert.True(promotions is >= 1 and <= 2,
            $"expected the promotion count to stay bounded (<=2) under a boundary-hovering stimulus, got {promotions}");
    }

    // ---- Success condition 4: low-power is silent on the wire -----------------------------------

    [Fact]
    public void LowPower_NeverEmitsFreeKinematicSamples_OnlyOnePathArcRecordAndHeartbeats()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher(heartbeatInterval: 3.0);
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;
        for (var i = 0; i < 150; i++) // 15 s at Dt=0.1 -> several heartbeat intervals
        {
            manager.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        Assert.Equal(PedDrModel.PathArc, manager.ModelOf(1));
        _output.WriteLine(
            $"[POC-3 measured] while low-power over 15s: FreeKinematicSamplesSent={publisher.FreeKinematicSamplesSent.GetValueOrDefault(1)}, "
            + $"PathArcRecordsSent={publisher.PathArcRecordsSent[1]}, HeartbeatsSent={publisher.HeartbeatsSent[1]}");
        Assert.Equal(0, publisher.FreeKinematicSamplesSent.GetValueOrDefault(1));
        Assert.Equal(1, publisher.PathArcRecordsSent[1]);
        Assert.True(publisher.HeartbeatsSent[1] >= 4, $"expected >= 4 heartbeats over 15s at a 3s rate, got {publisher.HeartbeatsSent[1]}");
        Assert.DoesNotContain(publisher.Events, e => e is FreeKinematicSample);
    }

    // ---- Success condition 5: determinism ---------------------------------------------------------

    [Fact]
    public void FullScenario_IsDeterministic_AcrossIndependentRuns()
    {
        var (trajectory1, events1) = RunFullScenario();
        var (trajectory2, events2) = RunFullScenario();

        Assert.Equal(trajectory1.Count, trajectory2.Count);
        for (var i = 0; i < trajectory1.Count; i++)
        {
            Assert.Equal(trajectory1[i].X, trajectory2[i].X, precision: 12);
            Assert.Equal(trajectory1[i].Y, trajectory2[i].Y, precision: 12);
        }

        Assert.Equal(events1.Count, events2.Count);
        for (var i = 0; i < events1.Count; i++)
        {
            AssertEventsEqual(events1[i], events2[i]);
        }
    }

    // Promotion (stimulus starts on the ped) + avoidance + demotion (stimulus removed), one full run.
    // Returns the per-step ped trajectory and the full published event stream, for the determinism
    // check above.
    private static (List<Vec2> Trajectory, List<PedEvent> Events) RunFullScenario()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        var entity = new InterestSource(WestNorthArm, PromoteRadius, DemoteRadius);
        var field = new InterestField();
        var entityId = field.Register(entity, InterestSourceKind.EntityAttached);
        const double entityRadius = 0.3;

        var trajectory = new List<Vec2>();
        var now = 0.0;
        for (var i = 0; i < 250; i++)
        {
            // Remove the stimulus permanently once the ped has been high-power for a while, so the run
            // exercises promotion, avoidance, AND demotion.
            if (i == 120)
            {
                field.Move(entityId, new Vec2(-10_000, -10_000));
            }

            var discs = manager.ModelOf(1) == PedDrModel.FreeKinematic && i < 120
                ? new[] { new WorldDisc(entity.Position.X, entity.Position.Y, 0, 0, entityRadius) }
                : Array.Empty<WorldDisc>();

            manager.Step(now, Dt, field, discs);
            now += Dt;
            trajectory.Add(manager.PositionOf(1, now));
        }

        return (trajectory, new List<PedEvent>(publisher.Events));
    }

    private static void AssertEventsEqual(PedEvent a, PedEvent b)
    {
        Assert.Equal(a.GetType(), b.GetType());
        Assert.Equal(a.Id, b.Id);
        Assert.Equal(a.Time, b.Time, precision: 12);

        switch (a)
        {
            case PathArcRecord pa:
                var pb = (PathArcRecord)b;
                Assert.Equal(pa.StartTime, pb.StartTime, precision: 12);
                Assert.Equal(pa.Speed, pb.Speed, precision: 12);
                Assert.Equal(pa.Path.Count, pb.Path.Count);
                for (var i = 0; i < pa.Path.Count; i++)
                {
                    Assert.Equal(pa.Path[i].X, pb.Path[i].X, precision: 12);
                    Assert.Equal(pa.Path[i].Y, pb.Path[i].Y, precision: 12);
                }

                break;

            case DrSwitchEvent sa:
                var sb = (DrSwitchEvent)b;
                Assert.Equal(sa.From, sb.From);
                Assert.Equal(sa.To, sb.To);
                break;

            case FreeKinematicSample fa:
                var fb = (FreeKinematicSample)b;
                Assert.Equal(fa.Pos.X, fb.Pos.X, precision: 12);
                Assert.Equal(fa.Pos.Y, fb.Pos.Y, precision: 12);
                Assert.Equal(fa.Vel.X, fb.Vel.X, precision: 12);
                Assert.Equal(fa.Vel.Y, fb.Vel.Y, precision: 12);
                break;

            case HeartbeatEvent:
                break; // Id + Time already checked above

            default:
                throw new InvalidOperationException($"unhandled PedEvent type {a.GetType()}");
        }
    }

    // ---- LIVE-PROD-1a: lively low-power (ActivityTimeline) peds -----------------------------------

    // A lively low-power ped walks its route with a Pause beat in the middle. Builds a Walk -> Pause ->
    // Walk timeline over a real navmesh path, splitting it at its own midpoint waypoint.
    private static ActivityTimeline BuildLivelyTimeline(IReadOnlyList<Vec2> path, double t0, double speed)
    {
        var mid = path.Count / 2;
        var first = new List<Vec2>();
        for (var i = 0; i <= mid; i++) first.Add(path[i]);
        var second = new List<Vec2>();
        for (var i = mid; i < path.Count; i++) second.Add(path[i]);

        return new ActivityTimeline(t0, new ActivitySegment[]
        {
            new WalkSegment(first, speed),
            new PauseSegment(2.0, "sip"),
            new WalkSegment(second, speed),
        });
    }

    [Fact]
    public void LivelyLowPower_ServerPoseMatchesHeadlessIgReconstruction_OverSweep_AndStaysSilentLowPower()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var timeline = BuildLivelyTimeline(path!, t0: 0.0, MaxSpeed);
        manager.AddPedLively(id: 1, timeline, MaxSpeed, Radius, now: 0.0);

        // No interest source anywhere near -> stays a lively ActivityTimeline ped the whole run.
        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        var noEntities = Array.Empty<WorldDisc>();

        var now = 0.0;
        var steps = (int)((timeline.EndTime + 1.0) / Dt);
        for (var i = 0; i < steps; i++)
        {
            manager.Step(now, Dt, field, noEntities);
            now += Dt;
        }

        Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(1));

        // The IG learns the whole timeline from the ONE spawn-time ActivityTimelineRecord + the switch;
        // it reconstructs pose via the SAME ActivityTimeline.PoseAt, so server==IG is exact.
        var ig = new HeadlessIg();
        ig.ApplyAll(publisher.Events);
        Assert.Equal(PedDrModel.ActivityTimeline, ig.ModelOf(1));

        var maxError = 0.0;
        for (var t = 0.0; t <= now; t += Dt / 3.0)
        {
            var serverPos = manager.PositionOf(1, t);
            var igPos = ig.ReconstructSample(1, t).Pos;
            maxError = Math.Max(maxError, (serverPos - igPos).Abs);
            // exact, not tolerance
            Assert.Equal(serverPos.X, igPos.X);
            Assert.Equal(serverPos.Y, igPos.Y);
        }

        _output.WriteLine($"[LIVE-PROD-1a measured] server-vs-IG max position error (lively low-power): {maxError:E3} m");
        Assert.True(maxError < 1e-12, $"max server-vs-IG error {maxError:E3} exceeded 1e-12");

        // Silent low-power: no per-step FreeKinematic samples, one timeline record, heartbeats flowing.
        Assert.False(publisher.FreeKinematicSamplesSent.ContainsKey(1) && publisher.FreeKinematicSamplesSent[1] > 0);
        Assert.Equal(1, publisher.ActivityTimelineRecordsSent[1]);
        Assert.True(publisher.HeartbeatsSent.TryGetValue(1, out var hb) && hb > 0, "expected heartbeats over a lively low-power run");
    }

    [Fact]
    public void LivelyLowPowerPed_PromotesToFreeKinematic_WhenAnInterestSourceIsPresent()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var timeline = BuildLivelyTimeline(path!, t0: 0.0, MaxSpeed);
        manager.AddPedLively(id: 1, timeline, MaxSpeed, Radius, now: 0.0);
        Assert.Equal(PedDrModel.ActivityTimeline, manager.ModelOf(1));

        // A source sitting on the ped's start promotes the lively ped exactly like it would a PathArc one.
        var field = new InterestField();
        field.Register(new InterestSource(WestNorthArm, PromoteRadius, DemoteRadius), InterestSourceKind.EntityAttached);
        var entity = new[] { new WorldDisc(WestNorthArm.X, WestNorthArm.Y, 0, 0, 0.3) };

        var everPromoted = false;
        var now = 0.0;
        for (var i = 0; i < 60 && !everPromoted; i++)
        {
            manager.Step(now, Dt, field, entity);
            now += Dt;
            if (manager.ModelOf(1) == PedDrModel.FreeKinematic) everPromoted = true;
        }

        Assert.True(everPromoted, "a lively low-power ped inside a promote radius must promote to FreeKinematic");
    }

    // ---- P1-2: forced high-power (evac panic pin) -----------------------------------------------

    [Fact]
    public void SetForcedHighPower_PromotesWithNoInterestSource_AndHoldsHigh_ThenDemotesWhenCleared()
    {
        var nav = BuildNav();
        var path = nav.FindPath(WestNorthArm, EastNorthArm);
        Assert.NotNull(path);

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        manager.AddPed(id: 1, path!, MaxSpeed, Radius, now: 0.0);

        // An interest field with its ONE source parked far away -> nothing would ever promote by proximity.
        var field = new InterestField();
        field.Register(new InterestSource(new Vec2(-10_000, -10_000), promoteRadius: 1.0, demoteRadius: 2.0));
        var noEntities = Array.Empty<WorldDisc>();

        // Pin it high-power (evac panic). It must promote on the next step despite no source nearby.
        manager.SetForcedHighPower(1, true);
        var now = 0.0;
        manager.Step(now, Dt, field, noEntities); now += Dt;
        Assert.Equal(PedDrModel.FreeKinematic, manager.ModelOf(1));

        // And it must STAY high across many steps while pinned (never demotes, even far from any source).
        for (var i = 0; i < 100; i++) { manager.Step(now, Dt, field, noEntities); now += Dt; }
        Assert.Equal(PedDrModel.FreeKinematic, manager.ModelOf(1));

        // Unpin -> it demotes back to low-power once past the dwell/hysteresis (it is far from every
        // demote radius, so the countdown runs immediately).
        manager.SetForcedHighPower(1, false);
        var demoted = false;
        for (var i = 0; i < 100 && !demoted; i++)
        {
            manager.Step(now, Dt, field, noEntities); now += Dt;
            if (manager.ModelOf(1) == PedDrModel.PathArc) demoted = true;
        }

        Assert.True(demoted, "an unpinned ped far from every source must demote back to low-power");
    }
}
