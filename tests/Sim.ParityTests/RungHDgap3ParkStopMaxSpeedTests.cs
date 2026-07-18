using System.Linq;
using Sim.Core;
using Sim.Harness;
using Xunit;

namespace Sim.ParityTests;

// Issue-2 zombie-stop regression (docs/SERVE-PATH-PLAN.md "Issue 2 (junction teleports)", fix
// commit c130fdc, Engine.cs TryInsertOnLane's `isStopDepart` branch, lines ~3244-3264). SUMO inserts
// a departPos="stop" vehicle STOPPED (MSLane::insertVehicle STOP case, MSLane.cpp:692-698 -- a car
// placed at its stop has speed 0) regardless of the requested departSpeed. Before the fix,
// TryInsertOnLane only forced speed 0 when departSpeed was already "0" literal -- a departPos="stop"
// + departSpeed="max" origin was inserted MOVING (at lane max speed), so it could lane-change off its
// stop lane before ProcessNextStop (which requires speed<=haltingSpeed ON the stop lane, Engine.cs's
// `stop.LaneId != v.LaneId` guard) ever marked the stop Reached -- leaving a permanent unreached
// "zombie" stop in the vehicle's queue. When the car later reached its real arrival edge, the Issue-1
// residency guard (near DestroyWithArrival: `if (frontStopAtArrival is { Reached: false, IsParking:
// true }) { Pos = laneLength; Speed = 0; break; }`) misread that stale zombie as a still-pending
// parking obligation and froze the car forever at the lane end instead of letting it arrive -- the
// root cause of the synthetic-junction "149 frozen cars -> 75 jam/yield teleports" gap. None of the
// pre-existing departPos="stop" goldens (48/67/68) exercise this: they all use departSpeed="0", so
// resolvedSpeed was already 0 pre-fix. This scenario is the first with departSpeed="max".
//
// scenarios/72-parkstop-maxspeed forces the mechanism deliberately (see rou.rou.xml's own header
// comment): e0 is a 2-lane approach where only lane 0 continues onward to e1 (lane 1 is a
// left-turn-only spur to a dead end, "da", never used by the route); the parkingArea pa0 sits on the
// "wrong" lane e0_1. The route "e0 e1" therefore REQUIRES a lane-change from e0_1 to e0_0 once the
// vehicle's short (duration=8) stop elapses -- exactly the kind of route-continuation lane-change
// that, pre-fix, could fire the moment the vehicle was inserted moving-but-not-yet-Reached, stranding
// the stop forever. Golden from vanilla SUMO 1.20.0 (see golden.tripinfo.xml): origin departs PARKED
// (speed 0.000000 despite the requested departSpeed="max") at pos=40 on e0_1, holds t=0..8, resumes
// and lane-changes onto e0_0 at t=9, crosses the J junction, and ARRIVES on e1 at t=15
// (arrivalLane="e1_0", arrivalSpeed=10, arrivalPos=22.8, duration=15, routeLength=50.57).
public class RungHDgap3ParkStopMaxSpeedTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "72-parkstop-maxspeed");
    private const int Steps = 30; // matches config.sumocfg's <end value="30"/> at step-length 1s.

    [Fact]
    public void ParkStopMaxSpeed_LoadsViaCfg_MatchesGoldenFcdWithinTolerance()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var actual = engine.Run(Steps);
        var golden = FcdParser.Parse(Path.Combine(ScenarioDir, "golden.fcd.xml"));
        var tolerance = ToleranceConfig.Load(Path.Combine(ScenarioDir, "tolerance.json"));

        var result = TrajectoryComparator.Compare(actual, golden, tolerance);

        Assert.True(result.IsMatch, BuildFailureMessage(result));
    }

    // The core regression: pre-fix, "origin" NEVER appeared in engine.CompletedTrips (the zombie stop
    // froze it at the arrival lane end forever instead of arriving it). Post-fix it must arrive, and
    // its arrival record must match the golden tripinfo (real, non-hardcoded values read from
    // golden.tripinfo.xml -- see the header comment for the exact numbers).
    [Fact]
    public void ParkStopMaxSpeed_Origin_ArrivesAndMatchesGoldenTripinfo()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        engine.Run(Steps);

        var completed = engine.CompletedTrips.SingleOrDefault(t => t.Id == "origin");
        Assert.True(completed is not null,
            "origin never arrived (engine.CompletedTrips has no entry for it) -- this is exactly the " +
            "pre-fix zombie-stop freeze this scenario exists to catch.");

        var golden = TripInfoParser.Parse(Path.Combine(ScenarioDir, "golden.tripinfo.xml"));
        var goldenOrigin = Assert.Single(golden, t => t.Id == "origin");

        // Guard against a vacuous pass: pin the golden itself to the real, non-degenerate values
        // captured in provenance (not hardcoded guesses -- read straight from the committed golden).
        Assert.Equal("e1_0", goldenOrigin.ArrivalLane);
        Assert.Equal(15.0, goldenOrigin.ArrivalTime!.Value, 3);
        Assert.Equal(10.0, goldenOrigin.ArrivalSpeed!.Value, 3);
        Assert.Equal(22.8, goldenOrigin.ArrivalPos!.Value, 3);
        Assert.Equal(15.0, goldenOrigin.Duration, 3);
        Assert.Equal(0.0, goldenOrigin.WaitingTime!.Value, 3);

        // Direct field-level parity against the golden's arrival record. This scenario's route
        // crosses one priority junction (internal lane ":J_0_0", 11.77 m), so it also guards the
        // routeLength fix that landed with it: CompletedTripInfo.RouteLength now sums the vehicle's
        // traversed lane SEQUENCE (including internal junction lanes), matching SUMO's own
        // MSDevice_Tripinfo accumulation -- golden routeLength=50.57 (the prior edge-only formula
        // gave 38.80, short by exactly the 11.77 m internal lane). Every arrival field is asserted.
        Assert.Equal(goldenOrigin.ArrivalLane, completed!.ArrivalLane);
        Assert.Equal(goldenOrigin.ArrivalTime!.Value, completed.Arrival, 3);
        Assert.Equal(goldenOrigin.ArrivalSpeed!.Value, completed.ArrivalSpeed, 3);
        Assert.Equal(goldenOrigin.ArrivalPos!.Value, completed.ArrivalPos, 3);
        Assert.Equal(goldenOrigin.Duration, completed.Duration, 3);
        Assert.Equal(goldenOrigin.RouteLength!.Value, completed.RouteLength, 2);
        Assert.Equal(goldenOrigin.WaitingTime!.Value, completed.WaitingTime, 3);
        Assert.Equal(goldenOrigin.TimeLoss!.Value, completed.TimeLoss, 3);
    }

    // Proves the vehicle was genuinely PARKED first (off-lane-equivalent, speed 0, held at its
    // parkingArea slot) for the duration of the stop, before it ever pulls out -- guards against a
    // regression that merely "arrives eventually" without actually exercising the parked-then-resume
    // path this scenario is built to cover. Golden: pos=40 on e0_1 at speed 0 for t=0..8 (holds
    // through the full duration=8 stop; resumes+lane-changes at t=9 -- see golden.fcd.xml).
    [Fact]
    public void ParkStopMaxSpeed_Origin_IsParkedAtSlotBeforePullingOut()
    {
        var engine = new Engine();
        engine.LoadScenario(Path.Combine(ScenarioDir, "config.sumocfg"));

        var traj = engine.Run(Steps);
        var points = traj.PointsFor("origin");

        for (var step = 0; step <= 8; step++)
        {
            var p = points[(double)step];
            Assert.Equal("e0_1", p.Lane);
            Assert.Equal(40.0, p.Pos, 3);
            Assert.Equal(0.0, p.Speed, 3);
        }

        // Confirms the pull-out actually happens (not a permanently-parked sink): by t=9 the vehicle
        // must have left its stop lane, matching the golden's lane-change-on-resume at t=9.
        var afterResume = points[9.0];
        Assert.NotEqual("e0_1", afterResume.Lane);
    }

    private static string BuildFailureMessage(ComparisonResult result)
    {
        var lines = new List<string>
        {
            $"scenario 72 park-stop max-speed parity FAILED. FirstDivergenceStep={result.FirstDivergenceStep?.ToString() ?? "none"}",
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
