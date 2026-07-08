using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// Rung B5-iii (TASKS.md "Junction foe the reducer yields to" -- the THIRD and FINAL of B5's
// three sub-rungs). Like B5-i/B5-ii/B1, the external agent is not in any SUMO run, so these are
// BEHAVIORAL/DIFFERENTIAL tests against the engine's own TrajectorySet -- mirrors
// RungB5LaneChangeVetoTests' structure/comment idiom, using the new fixture
// scenarios/_fixtures/junction-external-foe (net.net.xml copied verbatim from
// scenarios/11-priority-junction; a SINGLE vehicle `vMinor` on the minor yielding route SJ->JN,
// with NO `vMajor` at all -- so with no external agent present, `vMinor` has no foe of any kind
// and simply crosses the junction, exactly the "9b's inert path" sanity RungB5LaneChangeVetoTests'
// own baseline fact plays for A2).
//
// Junction geometry recap (net.net.xml, junction J): minor route SJ->JN is link 1, internal lane
// `:J_1_0`, whose `<request index="1" response="1100">` (rightmost-bit-is-link-0 convention, see
// JunctionRequest.RespondsTo's own doc comment) responds to links 2 and 3 -- i.e. `vMinor` must
// yield to whatever occupies EITHER major internal lane, `:J_2_0` (WJ->JE, link 2) or `:J_3_0`
// (WJ->JN, link 3). Link 0's own internal lane `:J_0_0` (SJ->JE) is NOT one of link 1's response
// bits, so an agent sitting there must NOT cause `vMinor` to yield -- test (c) below.
//
// Baseline arrival timing (observed by running the engine against this exact fixture with no
// obstacle at all, config.sumocfg's default.action-step-length=1/step-length=1): `vMinor`
// accelerates cleanly off `SJ_0` (departPos=0/departSpeed=0), reaches the lane's cruise speed
// 13.89 by t=6, and cruises until it nears its own minor link. Per C3 (the minor-link CAUTIOUS
// APPROACH now modeled in JunctionYieldConstraint), a lone minor vehicle does NOT sail straight
// through at 13.89 even with no foe present: it briefly decelerates toward the stop line while
// still beyond the link's foe-visibility distance (13.89 -> 9.433 at t=16 -> 4.933 at t=17), then
// -- once within visibility, gap confirmed clear -- re-accelerates and crosses, reading `JN_0`
// by t=19. This is the SAME mechanism (and shape) scenarios/19-onramp-merge's golden-verified rA
// exercises; crucially it NEVER comes to a sustained stop (min speed 4.933), which is exactly what
// separates it from test (b)'s external-agent hold (a full halt to 0 at the stop line).
public class RungB5JunctionFoeTests
{
    private static readonly string ScenarioDir =
        Path.Combine(RepoRoot(), "scenarios", "_fixtures", "junction-external-foe");

    // (a) Baseline sanity: with NO external agent (and no vMajor -- this fixture has only
    // vMinor), JunctionYieldConstraint's foe-link loop never finds a foe of any kind (SUMO or
    // external) on either responded-to internal lane (:J_2_0/:J_3_0), so `vMinor` is never HELD
    // at its stop line -- proving 9b's inert path (and this rung's new ExternalAgentOnFoeLane
    // guard, which the empty `_obstacles` store trivially short-circuits) lets a lone minor
    // vehicle through. It does perform C3's minor-link cautious dip (9.433/4.933 near the stop
    // line, min 4.933) but never a sustained stop, then crosses to JN_0 -- the differential
    // against test (b)'s full agent-forced halt.
    [Fact]
    public void Baseline_NoAgent_CrossesWithoutYielding()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        var trajectory = engine.Run(30);
        var points = trajectory.PointsFor("vMinor");

        // Never comes to a sustained stop while approaching: on `SJ_0` through the cautious
        // approach, speed stays well clear of zero (the C3 dip bottoms out at 4.933) -- no
        // braking-to-halt anywhere, which is what distinguishes this from test (b)'s agent hold.
        for (var t = 2.0; t <= 15.0; t += 1.0)
        {
            Assert.True(trajectory.TryGet("vMinor", t, out var point), $"missing point at t={t}");
            Assert.Equal("SJ_0", point.Lane);
            Assert.True(point.Speed > 0.5, $"vMinor unexpectedly near-stopped at t={t}, speed={point.Speed}");
        }

        // Crosses (reaches JN_0) by t=19 -- one-plus step later than a naive free-cruise would,
        // exactly because of C3's cautious approach (dip to 4.933 at the stop line, then
        // re-accelerate through :J_1_0 into JN_0), matching the observed baseline anchor.
        Assert.True(trajectory.TryGet("vMinor", 19.0, out var crossed));
        Assert.Equal("JN_0", crossed.Lane);
    }

    // (b) The core B5-iii property: an external agent occupying the RESPONDED-TO foe internal
    // lane `:J_2_0` (the major straight-through link `vMinor` must yield to per its own
    // response="1100" row) forces `vMinor` to brake and hold at its stop line on `SJ_0` for as
    // long as the agent is active -- differential vs. the baseline (a), which never stops at
    // all. The agent's window [-inf, 20) is chosen (per the briefing) by first observing (a)'s
    // arrival timing: 20 is well after `vMinor` would have reached its stop line (~t=16-18) and
    // well before it would naturally clear if unobstructed (t=17), giving a clean sustained-hold
    // window before the agent deactivates and `vMinor` resumes.
    [Fact]
    public void ExternalAgentOnRespondedFoeLane_ForcesYieldThenResumeAcrossJunction()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        const double agentEndTime = 20.0;

        // A crossing external agent standing on the major internal lane :J_2_0 -- the exact
        // stand-in for a navmesh/RVO agent currently traversing/occupying that crossing.
        engine.AddObstacle(
            "crossingAgent", ":J_2_0", frontPos: 5.0, length: 5.0,
            startTime: double.NegativeInfinity, endTime: agentEndTime);

        var trajectory = engine.Run(40);

        // (a) vMinor stops at/near its stop line on SJ_0 while the agent is active. Observed:
        // brakes starting ~t=16 (9.433/4.933 profile -- the SAME approaching-foe stop-line
        // formula 9b's own SUMO-foe branch produces, per JunctionYieldConstraint's shared
        // KraussModel.StopSpeed call), fully halted (speed=0) by t=19, held at pos~192.699.
        Assert.True(trajectory.TryGet("vMinor", 19.0, out var stopped));
        Assert.Equal("SJ_0", stopped.Lane);
        Assert.True(stopped.Speed < 0.1, $"vMinor should be halted at t=19, speed={stopped.Speed}");
        Assert.True(stopped.Pos > 190.0 && stopped.Pos < 193.0, $"vMinor should be held near its stop line, pos={stopped.Pos}");

        // Differential: at the SAME t=19 the baseline (a) is already deep into JN_0 (far past
        // the junction) -- proving this is the external agent's doing, not some independent
        // slowdown.
        var baselineEngine = new Engine();
        baselineEngine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));
        var baselineTrajectory = baselineEngine.Run(40);
        Assert.True(baselineTrajectory.TryGet("vMinor", 19.0, out var baselineAtSameTime));
        Assert.Equal("JN_0", baselineAtSameTime.Lane);

        // (c) vMinor never enters its own internal lane :J_1_0 while the agent is active (never
        // runs the crossing while the foe is present) -- checked against every recorded point
        // strictly before the agent's deactivation time.
        foreach (var point in trajectory.PointsFor("vMinor").Values)
        {
            if (point.Time < agentEndTime)
            {
                Assert.NotEqual(":J_1_0", point.Lane);
            }
        }

        // (b) Once the agent deactivates (t=20), vMinor resumes and eventually reaches JN_0
        // (crosses) -- proving yield-then-go, not a permanent block. Observed: resumes
        // accelerating at t=21, reaches JN_0 by t=23.
        var crossedAfterClearance = -1.0;
        for (var t = agentEndTime; t <= agentEndTime + 10.0; t += 1.0)
        {
            if (trajectory.TryGet("vMinor", t, out var point) && point.Lane == "JN_0")
            {
                crossedAfterClearance = t;
                break;
            }
        }

        Assert.True(crossedAfterClearance >= 0, "vMinor never crossed to JN_0 after the external agent deactivated");
    }

    // (c) Scoping fact: an agent on a NON-responded-to internal lane (:J_0_0, link 0 -- link 1's
    // own response="1100" row does NOT set bit 0) must NOT make vMinor yield at all -- proving
    // the <request>-matrix scoping (JunctionRequest.RespondsTo) is load-bearing here, not "any
    // obstacle near the junction stops everyone". Same geometry/placement style as test (b)'s
    // agent (a small object sitting on an internal lane), differing ONLY in which internal lane
    // it occupies, and active for the ENTIRE run (not just a window) -- if the scoping check were
    // missing, this would deterministically stop vMinor forever; instead the trajectory is
    // byte-identical to the no-agent baseline (a).
    [Fact]
    public void ExternalAgentOnNonRespondedFoeLane_DoesNotForceYield()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(ScenarioDir, "net.net.xml"),
            Path.Combine(ScenarioDir, "rou.rou.xml"),
            Path.Combine(ScenarioDir, "config.sumocfg"));

        engine.AddObstacle(
            "irrelevantAgent", ":J_0_0", frontPos: 5.0, length: 5.0,
            startTime: double.NegativeInfinity, endTime: double.PositiveInfinity);

        var trajectory = engine.Run(30);

        // Same never-held profile as the baseline (the C3 cautious dip applies identically; the
        // non-responded-to agent adds nothing), min speed well clear of zero throughout SJ_0.
        for (var t = 2.0; t <= 15.0; t += 1.0)
        {
            Assert.True(trajectory.TryGet("vMinor", t, out var point), $"missing point at t={t}");
            Assert.Equal("SJ_0", point.Lane);
            Assert.True(point.Speed > 0.5, $"vMinor unexpectedly near-stopped at t={t}, speed={point.Speed}");
        }

        // Crosses at the exact same baseline step (t=19), unaffected by the non-responded-to
        // foe-lane agent -- the <request>-matrix scoping holds, so its trajectory is identical to
        // baseline (a) including the cautious dip.
        Assert.True(trajectory.TryGet("vMinor", 19.0, out var crossed));
        Assert.Equal("JN_0", crossed.Lane);
    }

    // Mirrors EngineRung1PlumbingTests.RepoRoot() / RungB5LaneChangeVetoTests.RepoRoot().
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
