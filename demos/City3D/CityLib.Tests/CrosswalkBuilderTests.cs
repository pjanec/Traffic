using System;
using System.Linq;
using CityLib;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VISUALS-NOTES.md "Crosswalk zebra" row: CityLib.CrosswalkBuilder's crossing-lane-id
// detection + pure stripe-quad math (stripe count/spacing, elevation, across-width).
public class CrosswalkBuilderTests
{
    // ---- id detection: matches the task's own regex, both edge-id and lane-id shaped strings. ----
    [Theory]
    [InlineData(":d_0_0_c0", true)]       // crossing EDGE id (reference template.js tests this form)
    [InlineData(":d_0_0_c0_0", true)]     // crossing LANE id (edge id + "_<laneIndex>")
    [InlineData(":axis_w_zip_c0_0", true)]
    [InlineData(":d_0_0_w0_0", false)]    // walkingarea, not a crossing
    [InlineData("E0_0", false)]           // ordinary car lane
    [InlineData(":J_0_0", false)]         // internal junction lane, not a crossing
    [InlineData("", false)]
    [InlineData(null, false)]
    public void IsCrossingLaneId_MatchesExpected(string? id, bool expected)
    {
        Assert.Equal(expected, CrosswalkBuilder.IsCrossingLaneId(id));
    }

    // ---- stripe count matches spacing: a 10m-long straight crossing at 1.0m spacing -> 10 stripes. ----
    [Fact]
    public void Build_StraightCrossing_StripeCount_MatchesSpacing()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (mesh, stripeCount) = CrosswalkBuilder.Build(shape, width: 4.0, stripeSpacing: 1.0);

        Assert.Equal(10, stripeCount);
        Assert.True(mesh.Vertices.Length > 0);
        // Each stripe is a RoadMeshBuilder 2-point ribbon: 4 vertices (2 per endpoint) * 3 floats.
        Assert.Equal(10 * 4 * 3, mesh.Vertices.Length);
    }

    // ---- spacing changes proportionally change the stripe count. ----
    [Fact]
    public void Build_WiderSpacing_FewerStripes()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (_, denseCount) = CrosswalkBuilder.Build(shape, width: 4.0, stripeSpacing: 1.0);
        var (_, sparseCount) = CrosswalkBuilder.Build(shape, width: 4.0, stripeSpacing: 2.0);

        Assert.True(sparseCount < denseCount, $"expected fewer stripes at wider spacing (dense={denseCount}, sparse={sparseCount})");
        Assert.Equal(5, sparseCount);
    }

    // ---- a single stripe's geometry: centred on the crossing, spanning width*widthFraction across it,
    // elevated by elevationOffsetSumoZ (Godot Y == SUMO Z, CoordinateTransform.SumoToGodot). ----
    [Fact]
    public void Build_SingleStripe_IsCenteredAcrossWidthAndElevated()
    {
        // A 1.5m-long crossing at spacing=2 -> exactly one stripe (centred at d=1.0, the first candidate).
        var shape = new (double X, double Y)[] { (0, 0), (1.5, 0) };
        const double width = 4.0;
        const double widthFraction = 0.8;
        const double elevation = 0.02;

        var (mesh, stripeCount) = CrosswalkBuilder.Build(
            shape, width, stripeSpacing: 2.0, widthFraction: widthFraction, elevationOffsetSumoZ: elevation);

        Assert.Equal(1, stripeCount);

        // Travel direction is +X; the stripe's ACROSS extent is perpendicular (SUMO Y), which
        // CoordinateTransform maps to Godot Z = -sumoY. Half-extent = width*widthFraction/2.
        var expectedHalfAcross = width * widthFraction / 2.0;

        var zs = new System.Collections.Generic.List<float>();
        var ys = new System.Collections.Generic.List<float>();
        for (var i = 0; i < mesh.Vertices.Length; i += 3)
        {
            ys.Add(mesh.Vertices[i + 1]);
            zs.Add(mesh.Vertices[i + 2]);
        }

        Assert.All(ys, y => Assert.Equal((float)elevation, y, 1e-4f));
        Assert.Equal(expectedHalfAcross, zs.Max(), 1e-3);
        Assert.Equal(-expectedHalfAcross, zs.Min(), 1e-3);
    }

    // ---- degenerate inputs produce an empty mesh, never throw. ----
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void Build_DegenerateShape_ReturnsEmptyMesh(int pointCount)
    {
        var shape = Enumerable.Range(0, pointCount).Select(i => ((double)i, 0.0)).ToArray();

        var (mesh, stripeCount) = CrosswalkBuilder.Build(shape, width: 4.0);

        Assert.Equal(0, stripeCount);
        Assert.Empty(mesh.Vertices);
    }

    [Fact]
    public void Build_ZeroWidth_ReturnsEmptyMesh()
    {
        var shape = new (double X, double Y)[] { (0, 0), (10, 0) };

        var (mesh, stripeCount) = CrosswalkBuilder.Build(shape, width: 0.0);

        Assert.Equal(0, stripeCount);
        Assert.Empty(mesh.Vertices);
    }
}
