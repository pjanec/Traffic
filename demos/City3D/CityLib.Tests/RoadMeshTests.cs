using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using CityLib;
using Sim.Ingest;
using Xunit;
using Xunit.Abstractions;

namespace CityLib.Tests;

// T1.3 success conditions (docs/DEMO-CITY3D-TASKS.md): "a road mesh is generated for every lane; total
// ribbon area is within +-5% of Sum(lane length * lane width) ... LaneShapeZ is sampled (city-organic-L2
// produces non-constant vertex Z, a flat net produces constant Z)".
public class RoadMeshTests
{
    private readonly ITestOutputHelper _output;

    public RoadMeshTests(ITestOutputHelper output) => _output = output;

    // ---- 1: synthetic straight, flat lane -- area + flat Y ----
    [Fact]
    public void Build_StraightFlatLane_AreaMatchesLengthTimesWidth_AndYIsZero()
    {
        var shape = new (double X, double Y)[] { (0, 0), (100, 0) };
        const double width = 3.2;

        var mesh = RoadMeshBuilder.Build(shape, null, width);

        Assert.Equal(100.0 * width, mesh.Area, 1e-3);

        for (var i = 0; i < mesh.Vertices.Length; i += 3)
        {
            Assert.Equal(0f, mesh.Vertices[i + 1], 1e-5f); // Y == 0 (no elevation data)
        }
    }

    // ---- 2: synthetic lane WITH shapeZ -- vertex Y non-constant, matches the ramp ----
    [Fact]
    public void Build_RampedLane_VertexY_MatchesElevationRamp()
    {
        var shape = new (double X, double Y)[] { (0, 0), (50, 0), (100, 0) };
        var shapeZ = new double[] { 0.0, 2.5, 5.0 };
        const double width = 3.2;

        var mesh = RoadMeshBuilder.Build(shape, shapeZ, width);

        var ys = new List<float>();
        for (var i = 0; i < mesh.Vertices.Length; i += 3)
        {
            ys.Add(mesh.Vertices[i + 1]);
        }

        Assert.True(ys.Distinct().Count() > 1, "expected non-constant vertex Y with a ramped shapeZ");

        // Endpoints: the FIRST vertex pair (left/right of shape[0]) is at z=0; the LAST pair at z=5.
        Assert.Equal(0.0f, ys[0], 1e-3f);
        Assert.Equal(0.0f, ys[1], 1e-3f);
        Assert.Equal(5.0f, ys[^1], 1e-3f);
        Assert.Equal(5.0f, ys[^2], 1e-3f);

        // Monotonic non-decreasing along the ramp (left-edge and right-edge sequences each, since a
        // straight lane's left/right offsets don't change Y -- shapeZ is per-vertex, shared by both edges
        // at that index).
        for (var i = 2; i < ys.Count; i += 2)
        {
            Assert.True(ys[i] >= ys[i - 2] - 1e-4f, $"left-edge Y decreased at vertex pair {i / 2}");
            Assert.True(ys[i + 1] >= ys[i - 1] - 1e-4f, $"right-edge Y decreased at vertex pair {i / 2}");
        }
    }

    // ---- 3: real network (09-traffic-light) -- mesh for every lane; total area within +-5% ----
    [Fact]
    public void BuildAll_RealNetwork_ProducesMeshPerLane_AndTotalAreaWithinFivePercent()
    {
        var netPath = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light", "net.net.xml");
        var network = NetworkParser.Parse(netPath);

        var meshes = RoadMeshBuilder.BuildAll(network, includeInternal: true).ToList();

        Assert.Equal(network.LanesByHandle.Count, meshes.Count);

        var expected = network.LanesByHandle.Sum(l => l.Length * l.Width);
        var actual = meshes.Sum(m => m.Mesh.Area);

        var pctDiff = Math.Abs(actual - expected) / expected * 100.0;
        _output.WriteLine($"09-traffic-light: expected(sum length*width)={expected:F3} actual(sum ribbon area)={actual:F3} diff={pctDiff:F2}%");

        Assert.True(pctDiff <= 5.0, $"ribbon area diff {pctDiff:F2}% exceeds 5% (expected={expected:F3}, actual={actual:F3})");
    }

    // ---- 4: elevation over a real multi-level net -- at least one lane non-constant Y ----
    //
    // FINDING (verified by direct inspection, reported to the task's author): the task briefing names
    // scenarios/_bench/city-organic-L2 as "a real multi-level net" ("elevation showcase" per
    // docs/DEMO-CITY3D-DESIGN.md's "Scale / scenario switching"). Its committed net.net.xml, however,
    // carries NO 3rd (z) shape component on any lane at all -- `grep -oE 'shape="[-0-9.]+,[-0-9.]+,[-0-9.]+'
    // scenarios/_bench/city-organic-L2/net.net.xml` matches zero lines, and provenance.txt confirms "L2"
    // names "--default.lanenumber 2" (2 lanes/edge), not an elevation level. NetworkParser.ParseShapeZ
    // (src/Sim.Ingest/NetworkParser.cs) returns null the instant any shape vertex lacks a z component, so
    // EVERY lane in this net parses with ShapeZ == null -- there is no elevation data anywhere in the repo
    // today (checked every scenarios/**/net.net.xml; none carry a 3-tuple shape). This is a scenario-data
    // gap, not a RoadMeshBuilder bug; demos/City3D is the only directory this task may touch, so the net
    // itself cannot be regenerated with real elevation here.
    //
    // BuildAll_RealNetwork_Elevation_IsPresentButFlat below still exercises the REAL city-organic-L2 net
    // end-to-end (mesh-per-lane, all constant Y, matching its genuinely-flat ShapeZ==null data).
    // BuildAll_SyntheticElevatedNetwork_NonConstantVertexY proves the actual ask -- that Lane.ShapeZ
    // (parsed by NetworkParser, exactly the production path) threads through BuildAll into non-constant
    // ribbon Y -- via a minimal synthetic net.xml carrying a real 3-tuple (elevation) shape, parsed with
    // the SAME NetworkParser the real scenarios use (NetworkParser.ParseXml, not a hand-built NetworkModel).
    [Fact]
    public void BuildAll_RealNetwork_Elevation_IsPresentButFlat()
    {
        var netPath = Path.Combine(RepoRoot(), "scenarios", "_bench", "city-organic-L2", "net.net.xml");
        var network = NetworkParser.Parse(netPath);

        Assert.True(network.LanesByHandle.Count > 0);
        Assert.All(network.LanesByHandle, l => Assert.Null(l.ShapeZ));

        var meshes = RoadMeshBuilder.BuildAll(network, includeInternal: true).ToList();
        Assert.Equal(network.LanesByHandle.Count, meshes.Count);
    }

    [Fact]
    public void BuildAll_SyntheticElevatedNetwork_NonConstantVertexY()
    {
        // Minimal but real net.xml (parsed by the actual NetworkParser, not hand-built): one edge, one
        // lane, a 3-vertex shape whose z climbs 0m -> 8m -> 8m -- exactly the "3rd x,y,z component"
        // NetworkParser.ParseShapeZ looks for (src/Sim.Ingest/NetworkParser.cs).
        const string netXml = """
            <net version="1.20">
              <edge id="E0" from="J0" to="J1">
                <lane id="E0_0" index="0" speed="13.9" length="100.0" width="3.2"
                      shape="0.00,0.00,0.00 50.00,0.00,8.00 100.00,0.00,8.00"/>
              </edge>
            </net>
            """;

        var network = NetworkParser.ParseXml(netXml);
        Assert.Single(network.LanesByHandle);
        Assert.NotNull(network.LanesByHandle[0].ShapeZ);

        var meshes = RoadMeshBuilder.BuildAll(network, includeInternal: true).ToList();
        Assert.Single(meshes);

        var mesh = meshes[0].Mesh;
        float? first = null;
        var nonConstant = false;
        for (var i = 0; i < mesh.Vertices.Length; i += 3)
        {
            var y = mesh.Vertices[i + 1];
            if (first is null)
            {
                first = y;
            }
            else if (Math.Abs(y - first.Value) > 1e-4f)
            {
                nonConstant = true;
            }
        }

        Assert.True(nonConstant, "expected non-constant vertex Y from a NetworkParser-parsed elevated lane (proves LaneShapeZ threads through BuildAll)");

        // Endpoints match the ramp: first vertex pair at z=0, last at z=8.
        Assert.Equal(0.0f, mesh.Vertices[1], 1e-3f);
        Assert.Equal(8.0f, mesh.Vertices[^2], 1e-3f);
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
