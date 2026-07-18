using System.Linq;
using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Issue 1 CROSS-EDGE fix (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md par 7, follow-up to
// scenarios/69-parkandstay-lc's SAME-edge case). scenarios/71-parkandstay-crossedge isolates the
// residual bug the acceptance run's synthetic grid found: on the SHORT parking edge (e1, 30m) with
// the parkingArea (pa1) very close to its start (startPos=2, endPos=16), a sink vehicle that reaches
// e1 STILL on the wrong lane cannot possibly same-edge-lane-change into the parkingArea's window
// before running out of it -- one step at the scenario's 20 m/s exceeds the whole 16m window. The
// vehicle must instead PRE-POSITION onto the correct lane on the APPROACH edge (e0, 181m) before ever
// reaching the junction.
//
// The fix: NetworkModel.ComputeBestLanes/ComputeAllBestLanes fold an unreached <stop parkingArea> on
// the route's FINAL edge into the terminal LaneQ base case (BuildTerminalLaneQ) exactly like
// MSVehicle::updateBestLanes' own stop-edge truncation -- every lane but the stop's own is clamped to
// the (clamped) stop position and marked non-continuing. That asymmetry propagates BACKWARD through
// NetworkModel's existing best-lanes recursion, so the route's POOL lane on e0 already targets the
// lane that connects onward to the stop's lane on e1 -- the EXISTING pool-driven strategic
// lane-changer (Engine.TryStrategicLaneChange) then converges onto it automatically, with no new
// same-edge special case. The override is per-VEHICLE (StopDef lives on VehicleDef, not Route) and is
// threaded through every pool-resolution call site (insertion, reroute, boundary re-resolution, jam
// teleport re-insertion) while deliberately bypassing the shared ROUTE-keyed
// _insertRouteSeqCache/_bestLanesCache for a qualifying vehicle, so no other vehicle sharing the same
// route id is ever affected.
public class RungHDgap3ParkAndStayCrossEdgeTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "71-parkandstay-crossedge");
    private const int Steps = 20; // matches config.sumocfg's <end value="20"/> at step-length 1s.

    [Fact]
    public void ParkAndStayCrossEdge_LoadsViaCfg_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(Steps);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    // The core regression: sink0 must lane-change onto e0_0 on the APPROACH edge (well before the
    // junction at pos=181) -- vanilla does this at t=1, ~180m before the junction (golden.fcd.xml) --
    // then cross onto e1_0 already on the parkingArea's lane and settle at the golden's slot
    // (pos=15.999, lane e1_0) rather than overshooting off the end of e1 on the wrong lane.
    [Fact]
    public void ParkAndStayCrossEdge_Vehicle_PrePositionsOnApproachEdgeBeforeJunction()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(Steps);

        // Golden: sink0 is already on e0_0 (the lane that connects onward to the parkingArea's lane)
        // by t=1, while still ~180m from the junction at pos=181 -- proof the CROSS-EDGE pool steered
        // it before it ever reached e1, not a last-moment same-edge scramble.
        var pointsAtT1 = traj.PointsFor("sink0")[1.0];
        Assert.Equal("e0_0", pointsAtT1.Lane);
    }

    [Fact]
    public void ParkAndStayCrossEdge_Vehicle_ReachesParkingLaneAtGoldenSlot()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(Steps);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));

        var lastTime = (double)(Steps - 1);
        var actualLast = traj.PointsFor("sink0")[lastTime];
        var goldenLast = golden.PointsFor("sink0")[lastTime];

        Assert.Equal("e1_0", actualLast.Lane);
        Assert.Equal(goldenLast.Lane, actualLast.Lane);
        Assert.Equal(goldenLast.Pos, actualLast.Pos, 3);
        Assert.Equal(0.0, actualLast.Speed, 3);
    }

    // RESIDENCY: sink0 must never appear in engine.CompletedTrips (it never arrives) and must be
    // present in the trajectory at EVERY step from t=0 through the LAST step -- exactly like the
    // golden (an empty <tripinfos/> for this scenario; vanilla SUMO agrees it never arrives either).
    [Fact]
    public void ParkAndStayCrossEdge_Vehicle_NeverArrivesAndStaysInTrajectoryEveryStep()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(Steps);

        Assert.DoesNotContain(engine.CompletedTrips, t => t.Id == "sink0");

        var points = traj.PointsFor("sink0");
        for (var step = 0; step < Steps; step++)
        {
            var t = (double)step;
            Assert.True(points.ContainsKey(t), $"sink0 missing from trajectory at t={t} -- it dropped out of the running set.");
        }

        var last = points[(double)(Steps - 1)];
        Assert.Equal(0.0, last.Speed, 3);
        Assert.NotEqual(-4.8, last.Y, 3); // parked off the running lane's centreline

        var goldenTripinfo = TripInfoParser.Parse(Path.Combine(ScenarioDir, "golden.tripinfo.xml"));
        Assert.DoesNotContain(goldenTripinfo, t => t.Id == "sink0");
        Assert.Empty(goldenTripinfo);
    }

    // Parked well before the parkingArea's window is overrun: once settled (t=9, see
    // golden.fcd.xml), sink0 stays fully stopped and resident through the last step -- guards
    // against a resume-then-arrive regression (a duration=100000 stop must never resume early).
    [Fact]
    public void ParkAndStayCrossEdge_Vehicle_StaysStoppedFromParkUntilLastStep()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(Steps);
        var points = traj.PointsFor("sink0");

        for (var step = 9; step < Steps; step++)
        {
            var p = points[(double)step];
            Assert.Equal("e1_0", p.Lane);
            Assert.Equal(0.0, p.Speed, 3);
        }
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"scenario 71 park-and-stay cross-edge parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
        };

        foreach (var attribute in result.Attributes)
        {
            lines.Add(
                $"  attribute={attribute.Attribute} maxAbsError={attribute.MaxAbsError} rmse={attribute.Rmse} withinTolerance={attribute.WithinTolerance}");
        }

        if (result.PresenceMismatches.Count > 0)
        {
            lines.Add("  presence mismatches:");
            foreach (var mismatch in result.PresenceMismatches)
            {
                lines.Add($"    {mismatch.Kind} vehicle={mismatch.VehicleId} time={mismatch.Time?.ToString() ?? "n/a"}");
            }
        }

        return string.Join(Environment.NewLine, lines);
    }

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
