using System.Collections.Generic;
using System.Linq;
using CityLib;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VISUALS-NOTES.md "Lane markings" row: CityLib.LaneMarkingBuilder's pure seam-offset +
// dash-quad math (dash count/spacing, seam offset direction, elevation).
public class LaneMarkingBuilderTests
{
    // ---- dash count matches the dash/gap period: a 10m straight lane, dash=1 gap=1 -> 5 dashes
    // (on [0,1] [2,3] [4,5] [6,7] [8,9], the [9,10] gap trailing off unfinished). ----
    [Fact]
    public void Build_StraightLane_DashCount_MatchesPeriod()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (mesh, dashCount) = LaneMarkingBuilder.Build(shape, laneWidth: 3.2, dashLength: 1.0, gapLength: 1.0);

        Assert.Equal(5, dashCount);
        Assert.True(mesh.Vertices.Length > 0);
        // Each dash is a RoadMeshBuilder 2-point ribbon: 4 vertices (2 per endpoint) * 3 floats.
        Assert.Equal(5 * 4 * 3, mesh.Vertices.Length);
    }

    // ---- a longer gap yields fewer dashes over the same lane length. ----
    [Fact]
    public void Build_LongerGap_FewerDashes()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (_, tightCount) = LaneMarkingBuilder.Build(shape, laneWidth: 3.2, dashLength: 1.0, gapLength: 1.0);
        var (_, looseCount) = LaneMarkingBuilder.Build(shape, laneWidth: 3.2, dashLength: 1.0, gapLength: 3.0);

        Assert.True(looseCount < tightCount, $"expected fewer dashes with a longer gap (tight={tightCount}, loose={looseCount})");
    }

    // ---- seam offset direction: for a lane travelling along +X (SUMO east), the LEFT edge is +Y (SUMO
    // north) -- matching RoadMeshBuilder's own left/right convention (rotate travel dir +90deg CCW:
    // (dx,dy)->(-dy,dx)) and the reference renderer's offsetPolyline() "positive = LEFT". CoordinateTransform
    // maps SUMO Y -> Godot Z = -sumoY, so a left-of-travel seam offset must land at NEGATIVE Godot Z.
    [Fact]
    public void Build_StraightEastboundLane_SeamOffsetIsOnTheLeft_NegativeGodotZ()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };
        const double laneWidth = 3.2;
        const double elevation = 0.02;

        var (mesh, dashCount) = LaneMarkingBuilder.Build(
            shape, laneWidth, dashLength: 1.0, gapLength: 1.0, elevationOffsetSumoZ: elevation);

        Assert.True(dashCount > 0);

        var ys = new List<float>();
        var zs = new List<float>();
        for (var i = 0; i < mesh.Vertices.Length; i += 3)
        {
            ys.Add(mesh.Vertices[i + 1]);
            zs.Add(mesh.Vertices[i + 2]);
        }

        // Elevated above the road surface (Godot Y == SUMO Z here, both 0-based).
        Assert.All(ys, y => Assert.Equal((float)elevation, y, 1e-4f));

        // The dash's own thin width straddles the seam line at Z = -(laneWidth/2); every vertex must be
        // strictly negative (left-of-travel), never crossing over to the right (positive Z).
        Assert.All(zs, z => Assert.True(z < 0f, $"expected every marking vertex left-of-travel (Z<0), got {z}"));

        var expectedSeamZ = -(float)(laneWidth / 2.0);
        var meanZ = zs.Average();
        Assert.Equal(expectedSeamZ, meanZ, 1e-2f);
    }

    // ---- degenerate inputs produce an empty mesh, never throw. ----
    [Fact]
    public void Build_SinglePointShape_ReturnsEmptyMesh()
    {
        var shape = new (double X, double Y)[] { (5, 5) };

        var (mesh, dashCount) = LaneMarkingBuilder.Build(shape, laneWidth: 3.2);

        Assert.Equal(0, dashCount);
        Assert.Empty(mesh.Vertices);
    }

    [Fact]
    public void Build_ZeroLaneWidth_ReturnsEmptyMesh()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (mesh, dashCount) = LaneMarkingBuilder.Build(shape, laneWidth: 0.0);

        Assert.Equal(0, dashCount);
        Assert.Empty(mesh.Vertices);
    }

    // ---- a dash spanning a bend in the underlying shape still produces valid (non-empty) sub-geometry,
    // proving SubPolyline follows the bend rather than throwing/degenerating. ----
    [Fact]
    public void Build_BentLane_ProducesDashesAcrossTheBend()
    {
        // An L-shaped lane: 5m east, then 5m north -- a dash near the corner (e.g. covering [4,6]) spans
        // both segments.
        var shape = new (double X, double Y)[] { (0, 0), (5, 0), (5, 5) };

        var (mesh, dashCount) = LaneMarkingBuilder.Build(shape, laneWidth: 3.2, dashLength: 1.0, gapLength: 1.0);

        Assert.True(dashCount > 0);
        Assert.True(mesh.Vertices.Length > 0);
    }
}
