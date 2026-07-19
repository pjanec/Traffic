using System;
using System.IO;
using CityLib;
using Xunit;

namespace CityLib.Tests;

// docs/DEMO-CITY3D-DESIGN.md "#### Pedestrians (P7-3)" verification: "CityLib.Tests pins the pure ped math
// (coordinate transform of a known ped pose; regime mapping; avatar center/height) with no Godot
// dependency, same as CarTransformTests." Pure-math, hermetic (no SUMO, no Godot) -- mirrors
// CarTransformTests' style. These assert real values, not tautologies.
public class PedTransformTests
{
    // ---- 1: SUMO->Godot coordinate transform of a known ped position ----
    // CoordinateTransform.SumoToGodot(x, y, z) = (x, z, -y). A ped at SUMO (120, 130) on the flat plaza
    // (z = 0) must land at Godot (120, 0, -130).
    [Fact]
    public void CoordinateTransform_MapsKnownPedPosition_SumoToGodot()
    {
        var (gx, gy, gz) = CoordinateTransform.SumoToGodot(120.0, 130.0, 0.0);

        AssertClose(120f, gx, 1e-5f);
        AssertClose(0f, gy, 1e-5f);   // flat ped net -> Godot up is 0
        AssertClose(-130f, gz, 1e-5f); // SUMO north -> Godot -Z
    }

    // ---- 2: avatar center = ground + height/2 (stands ON the ground) ----
    [Fact]
    public void ForPed_SeatsAvatarOnGround_PosYIsGroundPlusHalfHeight()
    {
        // A reconstructed ped already in Godot space; Y = 0 is the flat plaza ground.
        var p = new ReconstructedPed(id: 7, x: 12.5f, y: 0f, z: -30f, regime: PedRegime.LowPower, visible: true);

        var inst = PedTransform.ForPed(p, height: 1.8f, width: 0.5f);

        AssertClose(0f + 1.8f / 2f, inst.PosY, 1e-5f); // 0.9 m -- half the standing height above the ground
        AssertClose(p.X, inst.PosX, 1e-5f);
        AssertClose(p.Z, inst.PosZ, 1e-5f);
    }

    // ---- 3: scale = (width, height, width) ----
    [Fact]
    public void ForPed_MapsWidthHeightWidth_ToScale()
    {
        var p = new ReconstructedPed(id: 3, x: 0f, y: 0f, z: 0f, regime: PedRegime.HighPower, visible: true);

        var inst = PedTransform.ForPed(p, height: 1.8f, width: 0.5f);

        AssertClose(0.5f, inst.ScaleX, 1e-5f);
        AssertClose(1.8f, inst.ScaleY, 1e-5f);
        AssertClose(0.5f, inst.ScaleZ, 1e-5f);
    }

    // ---- 4: regime/IsHighPower passthrough ----
    [Theory]
    [InlineData(PedRegime.LowPower, false)]
    [InlineData(PedRegime.HighPower, true)]
    public void ForPed_PassesRegimeThrough(PedRegime regime, bool expectedHighPower)
    {
        var p = new ReconstructedPed(id: 1, x: 1f, y: 0f, z: 2f, regime: regime, visible: true);

        var inst = PedTransform.ForPed(p);

        Assert.Equal(regime, inst.Regime);
        Assert.Equal(expectedHighPower, inst.IsHighPower);
        Assert.Equal(expectedHighPower, p.IsHighPower);
    }

    // ---- 5: default avatar dims are the believable slim-person band ----
    [Fact]
    public void Defaults_AreSlimUprightAvatar()
    {
        AssertClose(1.8f, PedTransform.DefaultHeightMeters, 1e-6f);
        AssertClose(0.5f, PedTransform.DefaultWidthMeters, 1e-6f);
    }

    // ---- 6: end-to-end through the real PedSimSource/PedReconstructor wire loop ----
    // Drives the actual server ped-sim + byte-loopback wire + PedRemoteReconstructor over the committed
    // poc0-crossing-plaza net (no SUMO, no Godot -- all in-process), and asserts the reconstructed crowd is
    // real (peds appear on the wire), lands in Godot space (flat net -> Godot up == 0), and maps to
    // ground-seated avatars. Mirrors CarTransformTests' Map_OverRealReconstruction end-to-end check.
    [Fact]
    public void Map_OverRealReconstruction_ProducesGroundSeatedAvatars()
    {
        var repoRoot = RepoRoot();
        using var sim = new PedSimSource(repoRoot);
        var reconstructor = new PedReconstructor();

        var sawAnyPed = false;

        // ~40s of ped-sim (200 ticks x 0.2s) -- well past the first spawns (every 20 steps) so the crowd is
        // populated; no wall-clock sleep needed since the loopback bus is synchronous.
        for (var tick = 0; tick < 200; tick++)
        {
            sim.Tick();
            var peds = reconstructor.Reconstruct(sim.Source, sim.Time);
            var avatars = PedTransform.Map(peds);

            Assert.Equal(peds.Count, avatars.Length);

            for (var i = 0; i < peds.Count; i++)
            {
                sawAnyPed = true;
                var p = peds[i];
                var a = avatars[i];

                // Flat ped net: Godot up (Y) is always 0 at the ground point.
                AssertClose(0f, p.Y, 1e-4f);
                // Avatar sits ON the ground: center Y = ground + height/2.
                AssertClose(p.Y + PedTransform.DefaultHeightMeters / 2f, a.PosY, 1e-4f);
                AssertClose(p.X, a.PosX, 1e-4f);
                AssertClose(p.Z, a.PosZ, 1e-4f);
                Assert.Equal(p.Regime, a.Regime);
                // Scale is the slim-avatar footprint x height x footprint.
                AssertClose(PedTransform.DefaultWidthMeters, a.ScaleX, 1e-4f);
                AssertClose(PedTransform.DefaultHeightMeters, a.ScaleY, 1e-4f);
                AssertClose(PedTransform.DefaultWidthMeters, a.ScaleZ, 1e-4f);
            }
        }

        Assert.True(sawAnyPed, "expected at least one reconstructed pedestrian over the run on poc0-crossing-plaza");
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

    // float-tolerant equality (matches CarTransformTests' own AssertClose convention -- Assert.Equal(float,
    // float, int) is ambiguous for float args, CS0121).
    private static void AssertClose(float expected, float actual, float eps)
        => Assert.True(Math.Abs(expected - actual) < eps, $"expected {expected} but got {actual} (eps={eps})");
}
