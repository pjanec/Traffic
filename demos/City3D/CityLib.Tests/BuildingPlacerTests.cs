using System;
using System.IO;
using System.Linq;
using CityLib;
using Sim.Ingest;
using Xunit;

namespace CityLib.Tests;

// T1.4 success conditions (docs/DEMO-CITY3D-TASKS.md): "building instances generated deterministically
// (same scenario+seed -> identical count and transforms across two runs); heights in a believable range
// (e.g. 6-60 m); none overlap the road ribbons (set-back respected, spot-checked)".
public class BuildingPlacerTests
{
    // ---- 1: determinism -- same net + seed, called twice, is byte-identical ----
    [Fact]
    public void PlaceAll_SameNetworkAndSeed_CalledTwice_ProducesIdenticalBoxes()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "_bench", "city-organic", "net.net.xml"));

        var first = BuildingPlacer.PlaceAll(network, seed: 42);
        var second = BuildingPlacer.PlaceAll(network, seed: 42);

        Assert.True(first.Count > 0, "expected at least one building on a real multi-edge network");
        Assert.Equal(first.Count, second.Count);

        for (var i = 0; i < first.Count; i++)
        {
            var a = first[i];
            var b = second[i];
            Assert.Equal(a.CX, b.CX); // exact float equality -- same hash inputs, same arithmetic order
            Assert.Equal(a.CY, b.CY);
            Assert.Equal(a.CZ, b.CZ);
            Assert.Equal(a.SizeX, b.SizeX);
            Assert.Equal(a.SizeY, b.SizeY);
            Assert.Equal(a.SizeZ, b.SizeZ);
            Assert.Equal(a.YawRad, b.YawRad);
        }
    }

    // ---- 1b: different seeds vary (proves the seed actually participates in the hash, not just id/step/side) ----
    [Fact]
    public void PlaceAll_DifferentSeeds_ProduceDifferentBoxes()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "_bench", "city-organic", "net.net.xml"));

        var seedA = BuildingPlacer.PlaceAll(network, seed: 1);
        var seedB = BuildingPlacer.PlaceAll(network, seed: 2);

        Assert.Equal(seedA.Count, seedB.Count); // placement steps are seed-independent -- only footprint/height/etc vary

        var anyDifferent = false;
        for (var i = 0; i < seedA.Count; i++)
        {
            if (seedA[i].SizeX != seedB[i].SizeX
                || seedA[i].SizeY != seedB[i].SizeY
                || seedA[i].SizeZ != seedB[i].SizeZ)
            {
                anyDifferent = true;
                break;
            }
        }

        Assert.True(anyDifferent, "expected at least one box to differ in footprint/height between two different seeds");
    }

    // ---- 2: believable heights -- every SizeY within [6, 60] ----
    [Fact]
    public void PlaceAll_RealNetwork_EveryHeightIsBelievable()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "_bench", "city-organic", "net.net.xml"));

        var boxes = BuildingPlacer.PlaceAll(network);

        Assert.True(boxes.Count > 0);
        Assert.All(boxes, b => Assert.InRange(b.SizeY, 6f, 60f));
    }

    // ---- 3a: count sanity on a real network ----
    [Fact]
    public void PlaceAll_CityOrganic_ProducesManyBuildings()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "_bench", "city-organic", "net.net.xml"));

        var boxes = BuildingPlacer.PlaceAll(network);

        Assert.True(boxes.Count > 0, "expected PlaceAll to generate at least one building on city-organic");
    }

    // ---- 3b: placement sanity -- spot-check a single straight synthetic edge: every box's center is at
    // least the edge's half-width away, laterally, from the edge centerline (buildings sit off the road).
    // A synthetic single-edge net (parsed by the real NetworkParser, same pattern RoadMeshTests uses for
    // its elevation case) makes the geometric check exact rather than approximate: the edge runs straight
    // along SUMO's +X axis, so CoordinateTransform.SumoToGodot's Z = -sumoY makes each box's lateral SUMO
    // offset directly recoverable as -box.CZ.
    [Fact]
    public void PlaceAll_SyntheticStraightEdge_EveryBoxIsOffTheRoad()
    {
        const string netXml = """
            <net version="1.20">
              <edge id="E0" from="J0" to="J1">
                <lane id="E0_0" index="0" speed="13.9" length="200.0" width="3.2" shape="0.00,0.00 200.00,0.00"/>
                <lane id="E0_1" index="1" speed="13.9" length="200.0" width="3.2" shape="0.00,3.20 200.00,3.20"/>
              </edge>
            </net>
            """;

        var network = NetworkParser.ParseXml(netXml);
        const double halfWidth = (3.2 + 3.2) / 2.0; // sum of both lanes' widths / 2, per design "Buildings"

        var boxes = BuildingPlacer.PlaceAll(network);

        Assert.True(boxes.Count > 0, "expected at least one building on a 200m straight edge");
        Assert.All(boxes, b =>
        {
            var lateralSumoOffset = Math.Abs(-b.CZ); // SUMO y = -Godot z (CoordinateTransform.SumoToGodot)
            Assert.True(
                lateralSumoOffset >= halfWidth - 1e-3,
                $"box center at lateral offset {lateralSumoOffset:F3} is closer than the edge half-width {halfWidth:F3} -- sits on/over the road");
        });
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
