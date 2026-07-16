using System;
using System.IO;
using System.Threading;
using CityLib;
using Sim.Core;
using Xunit;

namespace CityLib.Tests;

// T1.5 success conditions (docs/DEMO-CITY3D-TASKS.md): "per-instance scale equals each vehicle's vType
// Length x Width (+/-1e-3) x a fixed height; ... yaw equals the reconstructed heading ... within 1deg"
// (the yaw here is a pure passthrough, so exact equality holds, not just within 1deg); plus an end-to-end
// check driven through the real SimSource/Reconstructor pipeline (same recipe as ReconstructionTests).
public class CarTransformTests
{
    // ---- 1: scale mapping ----
    [Fact]
    public void ForVehicle_MapsLengthWidthHeight_ToScale()
    {
        var v = new ReconstructedVehicle(
            handle: new VehicleHandle(1, 1), x: 10f, y: 0f, z: 5f, yawRad: 0.3f, pitchRad: 0f,
            length: 4.5f, width: 1.8f, speed: 12f);

        var car = CarTransform.ForVehicle(v, 1.5f);

        AssertClose(4.5f, car.ScaleZ, 1e-4f);
        AssertClose(1.8f, car.ScaleX, 1e-4f);
        AssertClose(1.5f, car.ScaleY, 1e-4f);
    }

    // ---- 2: ground seating ----
    [Fact]
    public void ForVehicle_SeatsBoxOnGround_PosYIsGroundPlusHalfHeight()
    {
        var v = new ReconstructedVehicle(
            handle: new VehicleHandle(2, 1), x: 0f, y: 3.25f, z: 0f, yawRad: 0f, pitchRad: 0f,
            length: 4.5f, width: 1.8f, speed: 0f);

        var car = CarTransform.ForVehicle(v, 1.5f);

        AssertClose(v.Y + 1.5f / 2f, car.PosY, 1e-5f);
        AssertClose(v.X, car.PosX, 1e-5f);
        AssertClose(v.Z, car.PosZ, 1e-5f);
    }

    // ---- 3: yaw passthrough ----
    [Fact]
    public void ForVehicle_PassesYawThroughUnchanged()
    {
        var v = new ReconstructedVehicle(
            handle: new VehicleHandle(3, 1), x: 0f, y: 0f, z: 0f, yawRad: -1.2345f, pitchRad: 0f,
            length: 4.5f, width: 1.8f, speed: 0f);

        var car = CarTransform.ForVehicle(v, 1.5f);

        Assert.Equal(v.YawRad, car.YawRad);
    }

    // ---- 4: end-to-end via the real SimSource/Reconstructor pipeline ----
    [Fact]
    public void Map_OverRealReconstruction_MatchesRealVTypeDimsAndCount()
    {
        var (netPath, rouPath, cfgPath) = ScenarioPaths();
        using var sim = new SimSource(netPath, rouPath, cfgPath);
        var reconstructor = new Reconstructor();

        var sawAnyVehicle = false;

        for (var tick = 0; tick < 20; tick++)
        {
            sim.Tick();
            Thread.Sleep(20);
            var vehicles = reconstructor.Reconstruct(sim.Source, sim.LocalLanes, delaySeconds: 0.4);
            var cars = CarTransform.Map(vehicles, 1.5f);

            Assert.Equal(vehicles.Count, cars.Length);

            for (var i = 0; i < vehicles.Count; i++)
            {
                sawAnyVehicle = true;
                var v = vehicles[i];
                var car = cars[i];

                // "real vType Length/Width" pulled straight from the reconstruction's own dims (Reconstructor
                // sets ReconstructedVehicle.Length/Width from source.Dims, the actual vType-derived values).
                AssertClose(v.Length, car.ScaleZ, 1e-3f);
                AssertClose(v.Width, car.ScaleX, 1e-3f);
                AssertClose(1.5f, car.ScaleY, 1e-4f);
                Assert.Equal(v.YawRad, car.YawRad);
                AssertClose(v.Y + 0.75f, car.PosY, 1e-4f);
            }
        }

        Assert.True(sawAnyVehicle, "expected at least one reconstructed vehicle over the run on 09-traffic-light");
    }

    private static (string Net, string Rou, string Cfg) ScenarioPaths()
    {
        var scenarioDir = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light");
        return (
            Path.Combine(scenarioDir, "net.net.xml"),
            Path.Combine(scenarioDir, "rou.rou.xml"),
            Path.Combine(scenarioDir, "config.sumocfg"));
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

    // float-tolerant equality (Assert.Equal(float,float,int) is ambiguous with the double/precision
    // overload for float args -- see CS0121 -- so this repo's tests use an explicit epsilon check instead,
    // matching ReconstructionTests' own Math.Abs(...) < eps convention).
    private static void AssertClose(float expected, float actual, float eps)
        => Assert.True(Math.Abs(expected - actual) < eps, $"expected {expected} but got {actual} (eps={eps})");
}
