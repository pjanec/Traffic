using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B5-ii (TASKS.md "Cross-lane blocker vetoing lane changes" -- the SECOND of B5's three
// sub-rungs). Like B5-i/B1, the external obstacle is not in any SUMO run, so these are
// BEHAVIORAL/DIFFERENTIAL tests against the engine's own TrajectorySet -- mirrors
// RungB5MovingObstacleTests' structure/comment idiom, reusing scenario 12-overtake (the SAME
// fixture RungA2ParityTests anchors on: `follow` -- fast, no maxSpeed override -- overtakes
// `lead` -- slow, maxSpeed=5 -- on edge `e0`, right lane `e0_0` / left lane `e0_1`).
//
// Baseline anchor (read straight off scenarios/12-overtake/golden.fcd.xml, cross-checked against
// RungA2ParityTests' own header comment "fires the LEFT change at t11->t12"): with NO obstacle,
// `follow` is still on `e0_0` at t=11 (pos=122.340) and has committed to `e0_1` by t=12
// (pos=134.593). Per DecideSpeedGainChanges' post-move/CORRECTED-ORDERING comment, that commit
// is decided during the iteration whose `time == 11` (Run()'s loop variable), AFTER that
// iteration's ExecuteMoves has already advanced `follow`'s arc-length Pos to 134.593 on its
// still-current lane `e0_0` -- the commit is an instant lane-index snap at that SAME Pos, which
// is exactly why TargetLaneBlockedByObstacle projects ego's target-lane slot as
// [Pos-VType.Length, Pos] using the POST-move Pos.
public class RungB5LaneChangeVetoTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "12-overtake");

    // (a) Sanity/baseline: the scenario still overtakes with no obstacle at all -- this is also
    // the behavioral-level "A2 unchanged" check (byte-identical trajectory is separately proven
    // by RungA2ParityTests, which this rung does not touch).
    [Fact]
    public void Baseline_NoObstacle_FollowOvertakesToLeftLane()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var trajectory = engine.Run(20);

        Assert.True(trajectory.TryGet("follow", 11.0, out var beforeChange));
        Assert.Equal("e0_0", beforeChange.Lane);

        Assert.True(trajectory.TryGet("follow", 12.0, out var afterChange));
        Assert.Equal("e0_1", afterChange.Lane);
    }

    // (b)+(c) The core B5-ii property: a target-lane obstacle that is active exactly across the
    // window `follow` would otherwise commit its change in (t=11->12) VETOES the change --
    // `follow` holds `e0_0` for as long as the obstacle is active -- and once the obstacle
    // deactivates, `follow` completes the change shortly after (proving SpeedGainProbability
    // never got reset by the vetoed attempts: MSLCM_LC2013.cpp:1063/1080 only resets on an
    // actually-committed change, exactly like the pre-existing IsTargetLaneSafe veto).
    //
    // The obstacle spans the ENTIRE `e0_1` lane (FrontPos == lane length, Length == lane length,
    // so its back is 0) -- this sidesteps needing to predict `follow`'s exact Pos at every
    // retried attempt while blocked (it decelerates once it starts closing on `lead` on `e0_0`,
    // an ordinary/expected consequence of NOT changing lanes, not something this test needs to
    // predict precisely): ANY ego Pos in [0, laneLength) overlaps a whole-lane obstacle, so the
    // TargetLaneBlockedByObstacle "overlap" branch (hard veto, no secure-gap arithmetic needed)
    // fires deterministically on every retry for as long as the obstacle is active.
    [Fact]
    public void TargetLaneObstacleVetoesChange_ThenClearsOnceObstacleDeactivates()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double laneLength = 2000.0; // net.net.xml: e0_1 length="2000.00"
        const double obstacleEndTime = 20.0; // well after t=11's first attempt, before natural pass-through

        engine.AddObstacle(
            engine.GetLane("e0_1"), frontPos: laneLength, length: laneLength,
            startTime: double.NegativeInfinity, endTime: obstacleEndTime);

        var trajectory = engine.Run(60);
        var points = trajectory.PointsFor("follow");

        // Held on e0_0 for the whole time the obstacle is active, including exactly the step the
        // baseline commits at (t=12) and every retry through t=19 (the last whole second before
        // deactivation at t=20).
        for (var t = 11.0; t < obstacleEndTime; t += 1.0)
        {
            Assert.True(trajectory.TryGet("follow", t, out var point), $"missing trajectory point at t={t}");
            Assert.Equal("e0_0", point.Lane);
        }

        // Never overlaps the (whole-lane) obstacle while it is active -- trivially true since
        // `follow` never even reaches `e0_1` during [11, obstacleEndTime), but asserted directly
        // against the actual recorded lane/time pairs as a general safety invariant (would catch
        // a regression that let a change slip through mid-window).
        foreach (var point in points.Values)
        {
            if (point.Lane == "e0_1" && point.Time < obstacleEndTime)
            {
                Assert.Fail($"follow reached e0_1 at t={point.Time} while the target-lane obstacle was still active");
            }
        }

        // Once deactivated, the change completes promptly (the accumulator kept accruing through
        // every vetoed retry, so it fires again at the very next opportunity -- no re-ramp-up
        // needed, unlike a freshly-triggered speed-gain incentive).
        var completedTime = -1.0;
        for (var t = obstacleEndTime; t <= obstacleEndTime + 5.0; t += 1.0)
        {
            if (trajectory.TryGet("follow", t, out var point) && point.Lane == "e0_1")
            {
                completedTime = t;
                break;
            }
        }

        Assert.True(completedTime >= 0, "follow never completed the change to e0_1 after the obstacle deactivated");

        // Once on e0_1, remains there (the keep-right accumulator's own later return to e0_0,
        // per RungA2ParityTests' header comment, is a separate/expected later event -- not
        // asserted against here since this test's obstacle-driven delay shifts its timing too;
        // only the IMMEDIATE post-clearance change is this rung's concern).
        Assert.True(trajectory.TryGet("follow", completedTime, out var justChanged));
        Assert.Equal("e0_1", justChanged.Lane);
    }

    // Third fact -- the veto's LANE SCOPING, made genuinely differential (not vacuous): the
    // obstacle here has the SAME would-veto geometry as test (b)'s -- a lane-spanning blocker
    // that overlaps `follow`'s projected target-lane slot at every retry -- and differs from it
    // ONLY in which lane it sits on (`e0_0`, `follow`'s own CURRENT lane, instead of the target
    // `e0_1`). Because TargetLaneBlockedByObstacle filters on `obstacle.LaneId == targetLane.Id`,
    // this obstacle is scoped OUT and the change still commits at the exact baseline step
    // (t=11->12). This is the point of the test: delete that LaneId filter and this obstacle WOULD
    // hard-veto (its extent overlaps ego's slot just like (b)'s does), so the assertion below
    // flips -- proving the scoping is load-bearing, not incidental to obstacle position.
    //
    // Geometry: FrontPos=1999, Length=2000 -> extent [back=-1, front=1999]. The strictly-negative
    // back keeps it inert to the SEPARATE, pre-existing ObstacleConstraint car-following check on
    // `follow`'s own lane (that check skips any obstacle whose `back < v.Pos`, true here for every
    // reachable Pos >= 0 -- including Pos=0 at depart, unlike a back=0 whole-lane obstacle which
    // would spuriously stop `follow` at t=0) -- so the run is otherwise identical to the baseline
    // and this test isolates the lane-change veto's scoping alone. Yet front=1999 still overlaps
    // ego's projected slot ([Pos-Length, Pos], Pos ~120-134 across the window), so were it on the
    // TARGET lane it would take TargetLaneBlockedByObstacle's overlap branch exactly like (b).
    [Fact]
    public void ObstacleOnNonTargetLane_DoesNotVetoOrDelayTheChange()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        // Same lane-spanning would-veto geometry as test (b), but on `follow`'s OWN lane e0_0
        // (non-target) instead of the target e0_1 -- so only the LaneId filter distinguishes them.
        engine.AddObstacle(
            engine.GetLane("e0_0"), frontPos: 1999.0, length: 2000.0,
            startTime: double.NegativeInfinity, endTime: double.PositiveInfinity);

        var trajectory = engine.Run(20);

        Assert.True(trajectory.TryGet("follow", 11.0, out var beforeChange));
        Assert.Equal("e0_0", beforeChange.Lane);

        Assert.True(trajectory.TryGet("follow", 12.0, out var afterChange));
        Assert.Equal("e0_1", afterChange.Lane);
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot() / RungB5MovingObstacleTests.RepoRoot().
    private static string RepoRoot()
    {
        var dir = new DirectoryInfo(AppContext.BaseDirectory);
        while (dir is not null && !File.Exists(Path.Combine(dir.FullName, "Traffic.sln")))
        {
            dir = dir.Parent;
        }

        return dir?.FullName
            ?? throw new InvalidOperationException("Could not locate repo root (Traffic.sln not found above test assembly).");
    }
}
