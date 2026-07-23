using System.Collections.Generic;
using CityLib;
using Sim.LiveCity;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VISUALS-NOTES.md "Buildings (data-driven)" row: CityLib.BuildingFromDataBuilder's pure
// footprint+height -> extruded-prism mesh math (ear-clip roof cap + one wall quad per edge).
public class BuildingFromDataBuilderTests
{
    // ---- 1: a square footprint -- roof sits at Godot Y = HeightM, wall count == edge count. ----
    [Fact]
    public void Build_SquareFootprint_RoofAtHeightAndOneWallPerEdge()
    {
        var footprint = new (double X, double Y)[] { (0, 0), (10, 0), (10, 10), (0, 10) };

        var mesh = BuildingFromDataBuilder.Build(footprint, heightM: 14.0);

        Assert.Equal(4, mesh.RoofVertexCount);
        Assert.Equal(4, mesh.WallQuadCount); // one wall quad per footprint edge

        // Roof cap is the FIRST RoofVertexCount vertices; CoordinateTransform.SumoToGodot(x,y,z) puts the
        // extrusion height on Godot's Y axis (Godot Y = Sumo Z).
        for (var i = 0; i < mesh.RoofVertexCount; i++)
        {
            var y = mesh.Vertices[(i * 3) + 1];
            Assert.Equal(14.0f, y, 1e-3f);
        }
    }

    // ---- 2: the committed bldg_mall_0 footprint (a real demo_city/box rectangle) -- exact planar area,
    // preserved regardless of the builder's internal CCW-normalization step. ----
    [Fact]
    public void Build_MallFootprint_AreaMatchesShoelaceByHand()
    {
        var footprint = new (double X, double Y)[]
        {
            (4140.0, 4270.0), (4360.0, 4270.0), (4360.0, 4410.0), (4140.0, 4410.0),
        };

        var mesh = BuildingFromDataBuilder.Build(footprint, heightM: 14.0);

        // 220m x 140m rectangle.
        Assert.Equal(220.0 * 140.0, mesh.FootprintArea, 1e-6);
    }

    // ---- 3: a CLOCKWISE-authored footprint (opposite winding from the CCW demo_city/box data) still
    // yields the same area and the same wall/roof counts -- the builder normalizes winding internally. ----
    [Fact]
    public void Build_ClockwiseFootprint_AreaAndCountsUnaffectedByWinding()
    {
        var ccw = new (double X, double Y)[] { (0, 0), (10, 0), (10, 10), (0, 10) };
        var cw = new (double X, double Y)[] { (0, 0), (0, 10), (10, 10), (10, 0) };

        var meshCcw = BuildingFromDataBuilder.Build(ccw, heightM: 20.0);
        var meshCw = BuildingFromDataBuilder.Build(cw, heightM: 20.0);

        Assert.Equal(meshCcw.FootprintArea, meshCw.FootprintArea, 1e-6);
        Assert.Equal(meshCcw.RoofVertexCount, meshCw.RoofVertexCount);
        Assert.Equal(meshCcw.WallQuadCount, meshCw.WallQuadCount);
    }

    // ---- 4: a non-convex (L-shaped) footprint -- ear-clip triangulates it (not a naive fan, which would
    // produce a wrong/self-intersecting result for a reflex vertex), and the wall count still equals the
    // edge count. ----
    [Fact]
    public void Build_NonConvexLShapeFootprint_TriangulatesAndWallsPerEdge()
    {
        // L-shape: a 10x10 square with a 5x5 notch bitten out of the top-right corner. 6 vertices, 6
        // edges, one reflex vertex (5,5).
        var footprint = new (double X, double Y)[]
        {
            (0, 0), (10, 0), (10, 5), (5, 5), (5, 10), (0, 10),
        };

        var mesh = BuildingFromDataBuilder.Build(footprint, heightM: 12.0);

        Assert.Equal(6, mesh.RoofVertexCount);
        Assert.Equal(6, mesh.WallQuadCount);

        // Roof triangle count: an n-gon triangulates into (n-2) triangles regardless of convexity.
        var roofTriangleIndexCount = (6 - 2) * 3;
        var wallTriangleIndexCount = 6 * 2 * 3;
        Assert.Equal(roofTriangleIndexCount + wallTriangleIndexCount, mesh.Indices.Length);

        // Area: the 10x10 square minus the 5x5 notch = 75.
        Assert.Equal(75.0, mesh.FootprintArea, 1e-6);
    }

    // ---- 5: a degenerate (<3-point) footprint yields the Empty mesh, never throws. ----
    [Fact]
    public void Build_DegenerateFootprint_ReturnsEmptyMesh()
    {
        var mesh = BuildingFromDataBuilder.Build(new (double X, double Y)[] { (0, 0), (1, 1) }, heightM: 10.0);

        Assert.Empty(mesh.Vertices);
        Assert.Empty(mesh.Indices);
        Assert.Equal(0, mesh.RoofVertexCount);
        Assert.Equal(0, mesh.WallQuadCount);
    }

    // ---- 6: a non-positive height yields the Empty mesh, never throws (defensive -- the committed
    // dataset's HeightM values are all > 0). ----
    [Fact]
    public void Build_NonPositiveHeight_ReturnsEmptyMesh()
    {
        var footprint = new (double X, double Y)[] { (0, 0), (10, 0), (10, 10), (0, 10) };

        var mesh = BuildingFromDataBuilder.Build(footprint, heightM: 0.0);

        Assert.Empty(mesh.Vertices);
        Assert.Equal(0, mesh.RoofVertexCount);
    }

    // ---- 7: Godot-space X/Z mapping matches CoordinateTransform.SumoToGodot's (x, z, -y) exactly for the
    // roof cap vertices. ----
    [Fact]
    public void Build_RoofVertices_MatchSumoToGodotMapping()
    {
        var footprint = new (double X, double Y)[] { (0, 0), (10, 0), (10, 10), (0, 10) };

        var mesh = BuildingFromDataBuilder.Build(footprint, heightM: 5.0);

        // Vertex 0 = sumo (0,0) at height 5 -> godot (0, 5, 0).
        Assert.Equal(0f, mesh.Vertices[0], 1e-3f);
        Assert.Equal(5f, mesh.Vertices[1], 1e-3f);
        Assert.Equal(0f, mesh.Vertices[2], 1e-3f);

        // Vertex 1 = sumo (10,0) at height 5 -> godot (10, 5, 0).
        Assert.Equal(10f, mesh.Vertices[3], 1e-3f);
        Assert.Equal(5f, mesh.Vertices[4], 1e-3f);
        Assert.Equal(0f, mesh.Vertices[5], 1e-3f);
    }

    // ---- 8: the SceneBuilding overload delegates to the footprint/height overload identically. ----
    [Fact]
    public void Build_SceneBuildingOverload_MatchesFootprintOverload()
    {
        var footprint = new (double X, double Y)[] { (0, 0), (10, 0), (10, 10), (0, 10) };
        var building = new SceneBuilding("bldg_test", "office", footprint, HeightM: 30.0, Zone: null);

        var fromScene = BuildingFromDataBuilder.Build(building);
        var fromRaw = BuildingFromDataBuilder.Build(footprint, 30.0);

        Assert.Equal(fromRaw.FootprintArea, fromScene.FootprintArea, 1e-9);
        Assert.Equal(fromRaw.RoofVertexCount, fromScene.RoofVertexCount);
        Assert.Equal(fromRaw.WallQuadCount, fromScene.WallQuadCount);
        Assert.Equal(fromRaw.Vertices.Length, fromScene.Vertices.Length);
    }
}
