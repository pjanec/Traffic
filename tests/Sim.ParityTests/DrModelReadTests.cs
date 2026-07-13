using System.IO;
using Sim.Core;
using Sim.Core.Orca;
using Xunit;
using Xunit.Abstractions;

namespace Sim.ParityTests;

// DR2 (dead-reckoning coordination, issue #3, re-scoped per the NuGet reply). The DR read surface the
// publisher polls, off the Step() projection:
//   - Engine.DrModels (byte column) / Engine.GetDrModel(handle): a VEHICLE is LaneArc while moving,
//     Stationary when stopped -- NEVER FreeKinematic (LaneArc extrapolates along the curved lane; a
//     swerving car stays LaneArc). FreeKinematic is only ever produced by the crowd source.
//   - Engine.Manoeuvring (bool column) / Engine.IsManoeuvring(handle): the separate mid-manoeuvre bit
//     (RVO/ORCA swerve, or a normal-mode obstacle/crowd/overtake/give-way steer) the publisher reads to
//     raise the publish rate. True only during an active lateral manoeuvre.
// Additive/gated: a plain lane vehicle is LaneArc/Stationary with Manoeuvring == false; the columns are
// populated only in Step() (off the golden path); hash 909605E965BFFE59 and the suite are unaffected.
public class DrModelReadTests
{
    private static readonly string LanelessDir = Path.Combine(RepoRoot(), "scenarios", "_fixtures", "bridge-crossing");
    private static readonly string NormalDir = Path.Combine(RepoRoot(), "scenarios", "_fixtures", "bridge-crossing-normal");
    private static readonly string PlainDir = Path.Combine(RepoRoot(), "scenarios", "60-sublane-drift");

    private readonly ITestOutputHelper _out;

    public DrModelReadTests(ITestOutputHelper output) => _out = output;

    // A laneless-RVO vehicle swerving for a pedestrian stays LaneArc throughout (never FreeKinematic),
    // but IsManoeuvring flips true WHILE it is coupled/swerving and false before & after. Column ==
    // accessor every step.
    [Fact]
    public void LanelessVehicleSwervingForCrowd_StaysLaneArc_ManoeuvringFlips()
    {
        var engine = new Engine { LanelessRvo = true };
        engine.LoadScenario(
            Path.Combine(LanelessDir, "net.net.xml"),
            Path.Combine(LanelessDir, "rou.rou.xml"),
            Path.Combine(LanelessDir, "config.sumocfg"));
        var crowd = new OrcaCrowd();
        crowd.Add(new Vec2(30, -3.6), radius: 0.35, maxSpeed: 1.5, goal: new Vec2(30, -3.6));
        engine.CrowdSource = crowd;

        var sawManoeuvring = false;
        var sawSteady = false;

        for (var step = 0; step < 25; step++)
        {
            engine.Step();
            var handles = engine.VehicleHandles;
            var drCol = engine.DrModels;
            var mCol = engine.Manoeuvring;
            Assert.Equal(handles.Length, drCol.Length);
            Assert.Equal(handles.Length, mCol.Length);

            for (var i = 0; i < handles.Length; i++)
            {
                var dr = engine.GetDrModel(handles[i]);
                var m = engine.IsManoeuvring(handles[i]);
                Assert.Equal((byte)dr, drCol[i]);
                Assert.Equal(m, mCol[i]);
                Assert.NotEqual(DrModel.FreeKinematic, dr);   // a VEHICLE is never FreeKinematic

                if (dr == DrModel.LaneArc && m) sawManoeuvring = true;
                if (dr == DrModel.LaneArc && !m) sawSteady = true;
            }
        }

        _out.WriteLine($"laneless: sawManoeuvring={sawManoeuvring} sawSteady={sawSteady}");
        Assert.True(sawManoeuvring, "vehicle never reported Manoeuvring while swerving for the crowd agent");
        Assert.True(sawSteady, "vehicle never reported steady LaneArc (expected before/after the manoeuvre)");
    }

    // A NORMAL-mode vehicle dodging a CrowdSource pedestrian (Q6) is also flagged Manoeuvring, while
    // still classified LaneArc -- confirming the flag now covers the ComputeLateralEvasion path too.
    [Fact]
    public void NormalModeVehicleSwervingForCrowd_StaysLaneArc_IsManoeuvring()
    {
        var engine = new Engine();   // NORMAL mode (no LanelessRvo)
        engine.LoadScenario(
            Path.Combine(NormalDir, "net.net.xml"),
            Path.Combine(NormalDir, "rou.rou.xml"),
            Path.Combine(NormalDir, "config.sumocfg"));
        var crowd = new OrcaCrowd();
        crowd.Add(new Vec2(30, -3.6), 0.35, 1.0, goal: new Vec2(30, -3.6));
        engine.CrowdSource = crowd;

        var sawManoeuvring = false;
        for (var step = 0; step < 25; step++)
        {
            engine.Step();
            var handles = engine.VehicleHandles;
            for (var i = 0; i < handles.Length; i++)
            {
                Assert.NotEqual(DrModel.FreeKinematic, engine.GetDrModel(handles[i]));
                if (engine.IsManoeuvring(handles[i])) sawManoeuvring = true;
            }
        }

        Assert.True(sawManoeuvring, "normal-mode vehicle swerving for a crowd agent was not flagged Manoeuvring");
    }

    // A plain lane vehicle (no crowd, no obstacle) is LaneArc while moving and never Manoeuvring.
    [Fact]
    public void PlainLaneVehicle_LaneArc_NeverManoeuvring()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(PlainDir, "net.net.xml"),
            Path.Combine(PlainDir, "rou.rou.xml"),
            Path.Combine(PlainDir, "config.sumocfg"));

        var sawLaneArc = false;
        for (var step = 0; step < 20; step++)
        {
            engine.Step();
            var handles = engine.VehicleHandles;
            for (var i = 0; i < handles.Length; i++)
            {
                var dr = engine.GetDrModel(handles[i]);
                Assert.NotEqual(DrModel.FreeKinematic, dr);
                Assert.False(engine.IsManoeuvring(handles[i]), "a plain vehicle should never be Manoeuvring");
                if (dr == DrModel.LaneArc) sawLaneArc = true;
            }
        }

        Assert.True(sawLaneArc, "a moving plain vehicle should read LaneArc");
    }

    // A stale handle resolves to Stationary / not-manoeuvring.
    [Fact]
    public void StaleHandle_ResolvesToStationary()
    {
        var engine = new Engine();
        engine.LoadScenario(
            Path.Combine(PlainDir, "net.net.xml"),
            Path.Combine(PlainDir, "rou.rou.xml"),
            Path.Combine(PlainDir, "config.sumocfg"));
        engine.Step();

        Assert.Equal(DrModel.Stationary, engine.GetDrModel(new VehicleHandle(9999, 1)));
        Assert.False(engine.IsManoeuvring(new VehicleHandle(9999, 1)));
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
