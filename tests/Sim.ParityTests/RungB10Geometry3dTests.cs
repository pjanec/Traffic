using Sim.Core;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// SUMOSHARP-API.md §6 (geometry-3D): elevation (z) ingestion + the read surface's PosZ column. z is a
// purely output-side derivation (never feeds dynamics), so a 2-D net is byte-identical (PosZ == 0, proven
// in RungB8) and a 3-D net reports interpolated elevation. Additive: no golden/determinism impact.
public class RungB10Geometry3dTests
{
    // Pure interpolation: same segment walk as PositionAtOffset, linear z between vertices, clamped.
    [Fact]
    public void ElevationAtOffset_InterpolatesLinearlyAndClamps()
    {
        var shape = new (double X, double Y)[] { (0, 0), (100, 0) };
        var z = new double[] { 0.0, 10.0 };

        Assert.Equal(0.0, LaneGeometry.ElevationAtOffset(shape, z, 0.0), 9);
        Assert.Equal(5.0, LaneGeometry.ElevationAtOffset(shape, z, 50.0), 9);
        Assert.Equal(10.0, LaneGeometry.ElevationAtOffset(shape, z, 100.0), 9);
        // Clamp beyond the ends.
        Assert.Equal(0.0, LaneGeometry.ElevationAtOffset(shape, z, -20.0), 9);
        Assert.Equal(10.0, LaneGeometry.ElevationAtOffset(shape, z, 250.0), 9);
    }

    // Multi-segment: z accumulates per segment by 2-D arc length.
    [Fact]
    public void ElevationAtOffset_MultiSegment()
    {
        var shape = new (double X, double Y)[] { (0, 0), (100, 0), (200, 0) };
        var z = new double[] { 0.0, 10.0, 30.0 };

        Assert.Equal(10.0, LaneGeometry.ElevationAtOffset(shape, z, 100.0), 9); // end of segment 1
        Assert.Equal(20.0, LaneGeometry.ElevationAtOffset(shape, z, 150.0), 9); // halfway up segment 2
    }

    // End-to-end: a 3-D net (scenario 14 with the driving lane's shape given z 0->25 over its 500 m) makes
    // the read surface report PosZ = 0.05 * pos as the spawned vehicle climbs it.
    [Fact]
    public void ReadSurface_ReportsInterpolatedElevationOn3dNet()
    {
        var src = File.ReadAllText(Path.Combine(RepoRoot(), "scenarios", "14-external-obstacle", "net.net.xml"));
        // Inject a z profile into the e0_0 driving lane only (0 m -> 0, 500 m -> 25).
        var net3d = src.Replace("0.00,-1.60 500.00,-1.60", "0.00,-1.60,0.00 500.00,-1.60,25.00");
        Assert.NotEqual(src, net3d); // the target shape was actually found and rewritten

        var tempNet = Path.Combine(Path.GetTempPath(), $"sumosharp_3d_{Guid.NewGuid():N}.net.xml");
        File.WriteAllText(tempNet, net3d);
        try
        {
            var e = new Engine();
            e.LoadNetwork(tempNet);
            var h = e.SpawnVehicle(e.DefaultVType, new[] { "e0" }, departPos: 0.0, departSpeed: 0.0, departLane: 0);

            var sawElevated = false;
            for (var k = 0; k < 40; k++)
            {
                e.Step();
                if (e.TryGetVehicle(h, out var s))
                {
                    Assert.True(Math.Abs(s.Z - 0.05f * (float)s.Pos) < 0.05f, // z == 25/500 * pos
                        $"z={s.Z} pos={s.Pos} expected≈{0.05f * (float)s.Pos}");
                    if (s.Pos > 1.0)
                    {
                        Assert.True(s.Z > 0.0f, "expected nonzero elevation once past the lane start");
                        sawElevated = true;
                    }
                }
            }

            Assert.True(sawElevated, "vehicle never climbed the 3-D lane");
        }
        finally
        {
            File.Delete(tempNet);
        }
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
