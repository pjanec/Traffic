using Sim.Core;
using Xunit;

namespace Sim.ParityTests;

// issue #4 merge follow-ups: (1) the DR regime + manoeuvre bits now surface on SimulationSnapshot
// (DrModels/Manoeuvring), copied from the engine's own columns instead of the publisher assuming LaneArc;
// (2) Core VTypeParams accepts the sublane lateral overrides (maxSpeedLat/latAlignment/minGapLat) and flows
// them through the same resolve. Both additive and inert on the parity path -> hermetic. No external deps.
public class RungB26DrColumnsAndLateralVTypeTests
{
    private static string Net14 => Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle", "net.net.xml");

    [Fact]
    public void Snapshot_CarriesDrRegimeAndManoeuvreColumns_VehiclesNeverFreeKinematic()
    {
        var e = new Engine();
        e.LoadNetwork(Net14);
        e.SpawnVehicle(e.DefaultVType, new[] { "e0" });
        for (var k = 0; k < 6; k++) e.Step();

        var snap = SimulationSnapshot.Capture(e);
        Assert.True(snap.Count > 0);
        // Columns are aligned with Handles (same length window) ...
        Assert.True(snap.DrModels.Length >= snap.Count);
        Assert.True(snap.Manoeuvring.Length >= snap.Count);

        for (var i = 0; i < snap.Count; i++)
        {
            // ... a plain lane-following vehicle is LaneArc or Stationary, NEVER FreeKinematic (the frozen
            // issue #3/#4 contract: FreeKinematic comes only from the laneless crowd source), ...
            var model = (DrModel)snap.DrModels[i];
            Assert.True(model is DrModel.LaneArc or DrModel.Stationary, $"unexpected {model}");
            Assert.NotEqual(DrModel.FreeKinematic, model);
            // ... and not manoeuvring (no sublane/RVO active in this scenario).
            Assert.False(snap.Manoeuvring[i]);
            // The per-handle accessor agrees with the column.
            Assert.Equal(model, e.GetDrModel(snap.Handles[i]));
        }
    }

    [Fact]
    public void DefineVType_AcceptsLateralOverrides_AndRunsInert()
    {
        var e = new Engine();
        e.LoadNetwork(Net14);

        // The three sublane lateral overrides flow through DefineVType -> the resolve pipeline. They are
        // inert here (no lateral-resolution set), so a vehicle defined with them still simulates normally.
        var vt = e.DefineVType(new VTypeParams
        {
            VClass = "passenger",
            MaxSpeedLat = 2.0,
            LatAlignment = "right",
            MinGapLat = 0.4,
        });
        Assert.True(vt.IsValid);

        var h = e.SpawnVehicle(vt, new[] { "e0" });
        for (var k = 0; k < 6; k++) e.Step();

        // It departed and moved like any lane vehicle; the lateral overrides changed nothing with the
        // sublane model off (parity-inert).
        Assert.True(e.TryGetVehicle(h, out var s));
        Assert.Equal(DrModel.LaneArc, e.GetDrModel(h));
        Assert.True(s.Pos >= 0.0);
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
