using Sim.Core.Orca;
using Sim.Pedestrians.Crossing;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;

namespace Sim.Pedestrians.Tests.Crossing;

// POC-2 (docs/PEDESTRIAN-POC-PLAN.md POC-2, success conditions 1-2): pedestrians accumulate at a
// signalized crossing's near-side portal while the walk signal is closed, then release and clear the
// crossing (a measurable throughput surge) once it opens -- driven by CrossingGate + the REAL
// tlLogic-derived ICrossingSignal for POC-0's west crossing (":c_c3", walk during [0,37) mod 90s,
// docs/PEDESTRIAN-POC-PLAN.md POC-2 / CrossingTlReaderTests).
//
// Geometry: the west crossing's centreline runs north-south at x=111.6, from y=126.4 (near/north
// portal, approached from the NW-corner walkingarea) to y=113.6 (far/south portal, the SW-corner
// walkingarea) -- crossing the west arm's east-west roadway. "The near side of the crossing line"
// (success condition 1) is therefore y >= 126.4: a held pedestrian must never dip below it.
public class CrossingGateCrowdTests
{
    private const double MaxSpeed = 1.4;    // m/s
    private const double Radius = 0.3;      // m
    private const double ArriveRadius = 0.4; // m
    private const double Dt = 0.2;          // s

    private static readonly Vec2 PortalNear = new(111.6, 126.4); // north end of :c_c3's centreline
    private static readonly Vec2 PortalFar = new(111.6, 113.6);  // south end
    private static readonly Vec2 Destination = new(111.6, 108.0);

    // Per-ped destination offset (spread along x): giving every ped the exact same LITERAL
    // destination point reproduces the same shared-goal contention CrossingGate's own hold points
    // avoid (class remarks) -- agents converging on one point jostle each other via ORCA reciprocal
    // avoidance and can settle meaningfully off that exact point. Spreading final destinations avoids
    // the contention entirely, same fix, applied at the far end of the route.
    private static Vec2 DestinationFor(int registrationIndex) => Destination + new Vec2(registrationIndex * 0.7, 0.0);

    private const double NearLineY = 126.4;

    private static string NetPath => Path.Combine(AppContext.BaseDirectory, "Fixtures", "net.net.xml");

    private static ICrossingSignal BuildWestSignal()
    {
        var network = PedNetworkParser.Load(NetPath);
        var westCrossing = network.Crossings.Single(c => c.Id == ":c_c3_0");
        return CrossingSignalFactory.ForCrossing(NetPath, westCrossing);
    }

    // Spacing between queue slots (CrossingGate's own hold-point stagger). CrossingGate reserves a
    // further FrontSlotBuffer (2) slots between the portal and the first registrant -- see
    // CrossingGate's class remarks for the empirical probe behind that number: at this spacing, with
    // agents genuinely walking in from a distance (not starting at rest near their slot), the
    // measured worst-case transient dip stayed ~1.3-1.4 m clear of the crossing line for 6-20 queued
    // agents, which is what this test exercises and asserts.
    private const double QueueSpacing = 2.0;
    private const int FrontSlotBuffer = 2; // mirrors CrossingGate's own private FrontSlotBuffer

    private static Vec2 ExpectedHoldSlot(int registrationIndex) =>
        PortalNear + new Vec2(0.0, (registrationIndex + FrontSlotBuffer) * QueueSpacing);

    // Spawns `count` pedestrians staggered north of the portal (approaching it from a real distance,
    // not starting at rest near their eventual slot), each registered on the SAME
    // [portalNear, portalFar, destination] path (portalIndex 0), and returns the crowd + gate + crowd
    // indices for the caller to drive.
    private static (OrcaCrowd Crowd, CrossingGate Gate, OrcaHandle[] Indices) BuildQueueScenario(
        int count, ICrossingSignal signal, double startTime)
    {
        var crowd = new OrcaCrowd { MaxNeighbours = 8 };
        var gate = new CrossingGate(
            crowd, new WaypointFollower(), signal, ArriveRadius,
            queueDirection: new Vec2(0.0, 1.0), QueueSpacing);

        var indices = new OrcaHandle[count];
        for (var k = 0; k < count; k++)
        {
            var path = new Vec2[] { PortalNear, PortalFar, DestinationFor(k) };

            // 3m further out than this ped's own eventual queue slot, so it genuinely walks in (not
            // already resting near its slot at t=startTime).
            var spawn = ExpectedHoldSlot(k) + new Vec2(0.0, 3.0);
            var idx = crowd.Add(spawn, Radius, MaxSpeed, goal: spawn);
            gate.AddRoute(idx, path, portalIndex: 0, MaxSpeed, startTime);
            indices[k] = idx;
        }

        return (crowd, gate, indices);
    }

    [Fact]
    public void Peds_QueueAtPortal_DuringHoldPhase_NoneCrossTheLine()
    {
        const int pedCount = 6;
        var signal = BuildWestSignal();
        // t=40 is inside the west crossing's don't-walk window ([37,90) mod 90) with 50s of hold
        // remaining before the next walk phase at t=90 -- ample time for every spawned ped (3m behind
        // its own queue slot, maxSpeed 1.4 m/s) to reach and queue at it.
        const double startTime = 40.0;
        var (crowd, gate, indices) = BuildQueueScenario(pedCount, signal, startTime);

        var now = startTime;
        const double holdEnd = 90.0;
        while (now < holdEnd)
        {
            Assert.False(signal.WalkAllowed(now), $"test setup: signal should be closed at t={now}");

            gate.Update(now);
            crowd.Step(Dt);
            now += Dt;

            for (var k = 0; k < pedCount; k++)
            {
                var pos = crowd.Position(indices[k]);
                // Success condition 1: none entering the roadway -- every ped stays on the near
                // (north) side of the crossing line for the whole hold window.
                Assert.True(pos.Y >= NearLineY - 1e-6,
                    $"ped {k} crossed the line during hold at t={now:F1}: y={pos.Y:F3}");
            }
        }

        // Success condition 1: a growing/held count near the portal -- by the end of the hold window
        // every ped has reached and is waiting at its queue slot (none still short of it).
        var heldCount = 0;
        for (var k = 0; k < pedCount; k++)
        {
            Assert.True(gate.IsHeld(indices[k], now - Dt), $"ped {k} should still be held at t={now - Dt:F1}");
            var pos = crowd.Position(indices[k]);
            var expectedSlot = ExpectedHoldSlot(k);
            Assert.True((pos - expectedSlot).Abs <= 0.5,
                $"ped {k} should have reached its queue slot by end of hold: pos=({pos.X:F2},{pos.Y:F2}), " +
                $"expected near ({expectedSlot.X:F2},{expectedSlot.Y:F2})");
            heldCount++;
        }

        Assert.Equal(pedCount, heldCount);
    }

    [Fact]
    public void Peds_ReleaseAndSurge_WhenWalkPhaseOpens_ThroughputExceedsHoldPhase()
    {
        const int pedCount = 6;
        var signal = BuildWestSignal();
        const double startTime = 40.0; // 50s of hold before the walk phase reopens at t=90
        var (crowd, gate, indices) = BuildQueueScenario(pedCount, signal, startTime);

        var crossedAt = new double?[pedCount]; // sim time each ped's y first drops below the line
        var wasAboveLine = new bool[pedCount];
        for (var k = 0; k < pedCount; k++)
        {
            wasAboveLine[k] = crowd.Position(indices[k]).Y >= NearLineY;
        }

        var now = startTime;
        const double endTime = 170.0; // 50s of hold + 37s walk + slack (queue drain measured up to ~104s post-open)
        while (now < endTime)
        {
            gate.Update(now);
            crowd.Step(Dt);
            now += Dt;

            for (var k = 0; k < pedCount; k++)
            {
                var y = crowd.Position(indices[k]).Y;
                if (wasAboveLine[k] && y < NearLineY && crossedAt[k] is null)
                {
                    crossedAt[k] = now;
                }

                wasAboveLine[k] = y >= NearLineY;
            }
        }

        // Success condition 1 (restated): zero crossings while the signal was ever closed.
        var crossedDuringHold = crossedAt.Count(t => t is not null && !signal.WalkAllowed(t.Value));
        Assert.Equal(0, crossedDuringHold);

        // Success condition 2: a measurable throughput surge once the walk phase opens at t=90.
        const double walkOpensAt = 90.0;
        const double walkWindowSeconds = 10.0;
        var crossedInWalkWindow = crossedAt.Count(t => t is >= walkOpensAt and < walkOpensAt + walkWindowSeconds);
        var holdThroughput = 0.0 / (walkOpensAt - startTime); // 0 crossings during the whole hold window
        var walkThroughput = crossedInWalkWindow / walkWindowSeconds;

        Assert.True(crossedInWalkWindow > 0,
            "expected at least one ped to cross within 10s of the walk phase opening (a surge)");
        Assert.True(walkThroughput > holdThroughput,
            $"walk-phase throughput ({walkThroughput:F2}/s) should exceed hold-phase throughput ({holdThroughput:F2}/s)");

        // Every ped eventually crosses and arrives.
        Assert.All(crossedAt, t => Assert.NotNull(t));
        for (var k = 0; k < pedCount; k++)
        {
            Assert.True(gate.IsRouteComplete(indices[k]), $"ped {k} should have completed its route by t={endTime}");
            var pos = crowd.Position(indices[k]);
            Assert.True((pos - DestinationFor(k)).Abs <= 1.0, $"ped {k} should have reached the destination");
        }
    }

    [Fact]
    public void Determinism_QueueAndReleaseTrajectory_IsIdenticalAcrossIndependentRuns()
    {
        List<Vec2[]> RunOnce()
        {
            var signal = BuildWestSignal();
            const double startTime = 40.0;
            var (crowd, gate, indices) = BuildQueueScenario(6, signal, startTime);

            var trajectory = new List<Vec2[]>();
            var now = startTime;
            const double endTime = 170.0;
            while (now < endTime)
            {
                gate.Update(now);
                crowd.Step(Dt);
                now += Dt;

                var snapshot = new Vec2[indices.Length];
                for (var k = 0; k < indices.Length; k++)
                {
                    snapshot[k] = crowd.Position(indices[k]);
                }

                trajectory.Add(snapshot);
            }

            return trajectory;
        }

        var run1 = RunOnce();
        var run2 = RunOnce();

        Assert.Equal(run1.Count, run2.Count);
        for (var step = 0; step < run1.Count; step++)
        {
            for (var k = 0; k < run1[step].Length; k++)
            {
                Assert.Equal(run1[step][k].X, run2[step][k].X, precision: 12);
                Assert.Equal(run1[step][k].Y, run2[step][k].Y, precision: 12);
            }
        }
    }
}
