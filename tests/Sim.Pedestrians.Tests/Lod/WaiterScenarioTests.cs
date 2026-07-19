using System.Linq;
using Sim.Core.Orca;
using Sim.Pedestrians.Lod;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Lod;

// LIVE-POC-3 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §7, §12): WaiterScenario is a pure generator that
// composes an ordinary ActivityTimeline -- so it inherits ActivityTimelineTests' whole server==IG
// story for free -- plus the micro-scenario-specific success conditions from docs §12: the actor
// loops deterministically, respects table capacity (every table gets served), and the "inside the
// building" hidden-dwell story (§6) applies between rounds, not just once at the start.
public class WaiterScenarioTests
{
    private readonly ITestOutputHelper _output;

    public WaiterScenarioTests(ITestOutputHelper output)
    {
        _output = output;
    }

    private const int WaiterId = 1;
    private const double Speed = 1.3;
    private const double ServeSeconds = 2.0;
    private const double InsideSeconds = 1.5;
    private const int Loops = 6; // >= Tables.Count so every table is served at least once (rotation)

    private static readonly Vec2 DoorPos = new(0.0, 0.0);

    private static readonly IReadOnlyList<Vec2> Tables = new[]
    {
        new Vec2(5.0, 2.0),
        new Vec2(5.0, -2.0),
        new Vec2(8.0, 2.0),
        new Vec2(8.0, -2.0),
    };

    private static WaiterScenarioConfig BuildConfig(double t0 = 5.0, ulong seed = 42UL) => new(
        DoorPos, Tables, t0, Speed, ServeSeconds, InsideSeconds, Loops, seed);

    // ---- Success condition: server == IG, bit-identical, fine-dt sweep -------------------------

    [Fact]
    public void ServerAndHeadlessIg_ReconstructExactlyIdenticalPose_OverFineDtSweep()
    {
        var cfg = BuildConfig();
        var timeline = WaiterScenario.Build(cfg);

        var publisher = new PedPublisher();
        publisher.PublishActivityTimeline(WaiterId, timeline, time: cfg.StartTime);
        publisher.PublishSwitch(WaiterId, PedDrModel.PathArc, PedDrModel.ActivityTimeline, time: cfg.StartTime);

        var ig = new HeadlessIg();
        ig.ApplyAll(publisher.Events);
        Assert.Equal(PedDrModel.ActivityTimeline, ig.ModelOf(WaiterId));

        const double dt = 0.05;
        var start = cfg.StartTime - 2.0;
        var end = timeline.EndTime + 2.0;

        var samples = 0;
        for (var now = start; now <= end; now += dt)
        {
            var serverSample = timeline.PoseAt(now);
            var igSample = ig.ReconstructSample(WaiterId, now);

            // EXACT equality (not a tolerance) -- both sides call the literal same ActivityTimeline
            // instance's PoseAt, so they can only ever agree, never merely "usually match".
            Assert.Equal(serverSample.Pos.X, igSample.Pos.X);
            Assert.Equal(serverSample.Pos.Y, igSample.Pos.Y);
            Assert.Equal(serverSample.Heading.X, igSample.Heading.X);
            Assert.Equal(serverSample.Heading.Y, igSample.Heading.Y);
            Assert.Equal(serverSample.AnimTag, igSample.AnimTag);
            Assert.Equal(serverSample.Visible, igSample.Visible);

            Assert.Equal(serverSample.Pos.X, ig.Reconstruct(WaiterId, now).X);
            Assert.Equal(serverSample.Pos.Y, ig.Reconstruct(WaiterId, now).Y);

            samples++;
        }

        _output.WriteLine($"[LIVE-POC-3 measured] server-vs-IG exact-equality sweep: {samples} samples, dt={dt}s, " +
            $"window=[{start:F2},{end:F2}]s, EndTime={timeline.EndTime:F2}");
        Assert.True(samples > 200, $"expected a fine-dt sweep of hundreds of samples, got {samples}");
    }

    // ---- Success condition: capacity -- every table in the cluster gets served -------------------

    [Fact]
    public void WaiterScenario_ServesEveryTable_AtSomeInstant()
    {
        var cfg = BuildConfig();
        var timeline = WaiterScenario.Build(cfg);

        var servedTable = new bool[Tables.Count];

        const double dt = 0.05;
        var start = cfg.StartTime;
        var end = timeline.EndTime;
        for (var now = start; now <= end; now += dt)
        {
            var sample = timeline.PoseAt(now);
            if (sample.AnimTag != WaiterScenario.ServeAnimTag || !sample.Visible)
            {
                continue;
            }

            for (var t = 0; t < Tables.Count; t++)
            {
                if ((sample.Pos - Tables[t]).Abs < 1e-9)
                {
                    servedTable[t] = true;
                }
            }
        }

        for (var t = 0; t < Tables.Count; t++)
        {
            Assert.True(servedTable[t], $"table {t} at {Tables[t]} was never served (AnimTag=='serve', Visible, Pos==table)");
        }

        _output.WriteLine($"[LIVE-POC-3 measured] tables served: {servedTable.Count(s => s)}/{Tables.Count}");
    }

    // ---- Success condition: inside = hidden, repeatedly (not just the initial dwell) --------------

    [Fact]
    public void WaiterScenario_ReportsInvisible_DuringInitialAndBetweenRoundsInsideDwells_AndVisibleWhileServing()
    {
        var cfg = BuildConfig();
        var timeline = WaiterScenario.Build(cfg);

        var insideWindows = new List<(double Start, double End)>();
        var serveWindows = new List<(double Start, double End)>();
        for (var i = 0; i < timeline.Segments.Count; i++)
        {
            if (timeline.Segments[i] is DwellSegment { Visible: false } inside)
            {
                var windowStart = timeline.T0 + timeline.StartOffsetOf(i);
                insideWindows.Add((windowStart, windowStart + inside.Dur));
            }

            if (timeline.Segments[i] is DwellSegment { Visible: true, AnimTag: WaiterScenario.ServeAnimTag } serve)
            {
                var windowStart = timeline.T0 + timeline.StartOffsetOf(i);
                serveWindows.Add((windowStart, windowStart + serve.Dur));
            }
        }

        // At least the initial inside-dwell PLUS one between-rounds inside-dwell (docs §6: the door
        // dwell repeats every loop, not just once at spawn).
        Assert.True(insideWindows.Count >= 2, $"expected >= 2 inside windows (initial + between-rounds), got {insideWindows.Count}");
        Assert.True(serveWindows.Count >= 1, "expected at least one serve window");

        foreach (var (windowStart, windowEnd) in insideWindows)
        {
            Assert.False(timeline.PoseAt(windowStart + ((windowEnd - windowStart) / 2.0)).Visible);
            Assert.False(timeline.PoseAt(windowStart + 1e-6).Visible);
            Assert.False(timeline.PoseAt(windowEnd - 1e-6).Visible);
        }

        foreach (var (windowStart, windowEnd) in serveWindows)
        {
            var mid = timeline.PoseAt(windowStart + ((windowEnd - windowStart) / 2.0));
            Assert.True(mid.Visible);
            Assert.Equal(WaiterScenario.ServeAnimTag, mid.AnimTag);
        }

        _output.WriteLine($"[LIVE-POC-3 measured] inside windows: {insideWindows.Count}, serve windows: {serveWindows.Count}");
    }

    // ---- Success condition: determinism -----------------------------------------------------------

    [Fact]
    public void Build_IsDeterministic_SameConfigYieldsBitIdenticalTimeline()
    {
        var cfg = BuildConfig();

        var timeline1 = WaiterScenario.Build(cfg);
        var timeline2 = WaiterScenario.Build(cfg);

        Assert.Equal(timeline1.EndTime, timeline2.EndTime);
        Assert.Equal(timeline1.Segments.Count, timeline2.Segments.Count);

        const double dt = 0.05;
        var start = cfg.StartTime - 1.0;
        var end = timeline1.EndTime + 1.0;

        var samples = 0;
        for (var now = start; now <= end; now += dt)
        {
            var s1 = timeline1.PoseAt(now);
            var s2 = timeline2.PoseAt(now);
            Assert.Equal(s1.Pos.X, s2.Pos.X);
            Assert.Equal(s1.Pos.Y, s2.Pos.Y);
            Assert.Equal(s1.Heading.X, s2.Heading.X);
            Assert.Equal(s1.Heading.Y, s2.Heading.Y);
            Assert.Equal(s1.AnimTag, s2.AnimTag);
            Assert.Equal(s1.Visible, s2.Visible);
            samples++;
        }

        Assert.True(samples > 200, $"expected a meaningful determinism sweep, got {samples}");
        _output.WriteLine($"[LIVE-POC-3 measured] determinism sweep: {samples} samples");
    }
}
