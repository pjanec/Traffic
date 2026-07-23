using System.Collections.Generic;
using CityLib;
using Sim.LiveCity;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VISUALS-NOTES.md OWNER-CONFIRMED "POI / area rendering" block: CityLib.DoorBuilder's pure
// building_entrance POI -> vertical door-box placement math (position + facing-derived yaw).
public class DoorBuilderTests
{
    // ---- 1: position -- Godot-space mapping (x, z=-y) with PosY raised HeightMeters/2 above ground. ----
    [Fact]
    public void ForEntrance_PositionIsGroundPointRaisedHalfHeight()
    {
        var poi = new ScenePoi("e1", "building_entrance", 10.0, 20.0, FacingX: 0.0, FacingY: 1.0);

        var door = DoorBuilder.ForEntrance(poi);

        Assert.Equal(10f, door.PosX, 1e-6f);
        Assert.Equal(DoorBuilder.HeightMeters / 2f, door.PosY, 1e-6f);
        Assert.Equal(-20f, door.PosZ, 1e-6f);
    }

    // ---- 2: yaw -- facing north (SUMO +y) reuses the SAME mapping NaviDegToGodotYawRad(0) uses (yaw=0,
    // forward = -Z), since a navi-heading-0 direction vector IS (0,1) (sin 0, cos 0). ----
    [Fact]
    public void ForEntrance_FacingNorth_YawMatchesNaviZero()
    {
        var poi = new ScenePoi("e1", "building_entrance", 0.0, 0.0, FacingX: 0.0, FacingY: 1.0);

        var door = DoorBuilder.ForEntrance(poi);

        AssertAngleClose(CoordinateTransform.NaviDegToGodotYawRad(0f), door.YawRad, 1e-4f);
    }

    // ---- 3: facing east (SUMO +x, navi 90 deg) -- yaw matches NaviDegToGodotYawRad(90). ----
    [Fact]
    public void ForEntrance_FacingEast_YawMatchesNavi90()
    {
        var poi = new ScenePoi("e1", "building_entrance", 0.0, 0.0, FacingX: 1.0, FacingY: 0.0);

        var door = DoorBuilder.ForEntrance(poi);

        AssertAngleClose(CoordinateTransform.NaviDegToGodotYawRad(90f), door.YawRad, 1e-4f);
    }

    // ---- 4: facing south/west round out the compass, each matching the navi-degree equivalent. ----
    [Theory]
    [InlineData(0.0, -1.0, 180f)]   // facing south (SUMO -y) == navi 180
    [InlineData(-1.0, 0.0, 270f)]   // facing west (SUMO -x) == navi 270
    public void ForEntrance_Yaw_MatchesNaviDegreeEquivalent(double facingX, double facingY, float naviDeg)
    {
        var poi = new ScenePoi("e1", "building_entrance", 0.0, 0.0, FacingX: facingX, FacingY: facingY);

        var door = DoorBuilder.ForEntrance(poi);

        AssertAngleClose(CoordinateTransform.NaviDegToGodotYawRad(naviDeg), door.YawRad, 1e-4f);
    }

    // ---- 5: a POI missing Facing falls back to due-north (never throws). ----
    [Fact]
    public void ForEntrance_MissingFacing_FallsBackToNorth()
    {
        var poi = new ScenePoi("e1", "building_entrance", 5.0, 5.0);

        var door = DoorBuilder.ForEntrance(poi);

        AssertAngleClose(CoordinateTransform.NaviDegToGodotYawRad(0f), door.YawRad, 1e-4f);
    }

    // ---- 6: Build filters to ONLY building_entrance, in input order. ----
    [Fact]
    public void Build_FiltersToBuildingEntranceOnly()
    {
        var pois = new[]
        {
            new ScenePoi("v1", "venue", 0, 0),
            new ScenePoi("e1", "building_entrance", 1, 1, FacingX: 0, FacingY: 1),
            new ScenePoi("e2", "building_entrance", 2, 2, FacingX: 1, FacingY: 0),
        };

        var doors = DoorBuilder.Build(pois);

        Assert.Equal(2, doors.Count);
    }

    private static void AssertAngleClose(float expectedRad, float actualRad, float toleranceRad)
    {
        var diff = System.MathF.Atan2(System.MathF.Sin(expectedRad - actualRad), System.MathF.Cos(expectedRad - actualRad));
        Assert.True(System.MathF.Abs(diff) <= toleranceRad, $"expected {expectedRad} rad, got {actualRad} rad (diff {diff})");
    }
}
