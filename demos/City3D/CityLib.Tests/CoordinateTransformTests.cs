using CityLib;
using Xunit;

namespace CityLib.Tests;

// T1.2 item 1 (docs/DEMO-CITY3D-TASKS.md): "the SUMO->Godot coordinate transform and navi-heading->yaw
// conversion are exercised and asserted once" -- pure math, no sim/engine involved.
public class CoordinateTransformTests
{
    [Fact]
    public void SumoToGodot_MapsAxesPerDesign()
    {
        // SUMO (x=10 east, y=20 north, z=3 up) -> Godot (x=10, y=3 up, z=-20).
        var (gx, gy, gz) = CoordinateTransform.SumoToGodot(10.0, 20.0, 3.0);
        Assert.Equal(10f, gx, 0.001f);
        Assert.Equal(3f, gy, 0.001f);
        Assert.Equal(-20f, gz, 0.001f);
    }

    [Theory]
    // (naviHeadingDeg, expectedGodotYawRad) -- derived in CoordinateTransform.cs's header comment:
    // yaw = -theta, since Basis(Up, yaw) maps local forward (0,0,-1) to (-sin(yaw), 0, -cos(yaw)), which
    // must equal the SUMO direction vector (sin(theta), cos(theta)) reflected through (x, -y) -> (sin(theta), -cos(theta)).
    [InlineData(0f, 0f)] // facing north (SUMO +y) -> Godot -Z -> yaw 0
    [InlineData(90f, -MathF.PI / 2f)] // facing east (SUMO +x) -> Godot +X -> yaw -90deg
    [InlineData(180f, MathF.PI)] // facing south (SUMO -y) -> Godot +Z -> yaw +-180deg
    [InlineData(270f, MathF.PI / 2f)] // facing west (SUMO -x) -> Godot -X -> yaw +90deg
    public void NaviDegToGodotYawRad_MatchesDerivedMapping(float naviDeg, float expectedYawRad)
    {
        var yaw = CoordinateTransform.NaviDegToGodotYawRad(naviDeg);
        AssertAngleClose(expectedYawRad, yaw, toleranceRad: 1e-4f);
    }

    [Fact]
    public void NaviDegToGodotYawRad_ForwardVectorMatchesTransformedSumoDirection()
    {
        // For a range of headings, verify the Godot Basis(Up, yaw) forward vector
        // (-sin(yaw), 0, -cos(yaw)) equals the SUMO direction vector (sin(theta), cos(theta)) transformed
        // through the SAME SumoToGodot position mapping (drop z, negate y) -- i.e. the position transform
        // and the heading transform are mutually consistent, not just individually plausible.
        for (var deg = 0f; deg < 360f; deg += 15f)
        {
            var thetaRad = deg * MathF.PI / 180f;
            var sumoDirX = MathF.Sin(thetaRad);
            var sumoDirY = MathF.Cos(thetaRad);

            // Transform a unit step in the sim direction through the position mapping (z term dropped --
            // heading is planar) to get the expected Godot-plane direction.
            var (godotDirX, _, godotDirZ) = CoordinateTransform.SumoToGodot(sumoDirX, sumoDirY, 0.0);

            var yaw = CoordinateTransform.NaviDegToGodotYawRad(deg);
            var forwardX = -MathF.Sin(yaw);
            var forwardZ = -MathF.Cos(yaw);

            Assert.True(
                MathF.Abs(forwardX - godotDirX) < 1e-4f && MathF.Abs(forwardZ - godotDirZ) < 1e-4f,
                $"heading {deg}deg: forward=({forwardX:F4},{forwardZ:F4}) vs expected=({godotDirX:F4},{godotDirZ:F4})");
        }
    }

    // docs/LIVE-CITY-VISUALS-NOTES.md building-entrance door: DirectionToGodotYawRad is the general form of
    // NaviDegToGodotYawRad for a caller that already has an (x,y) direction vector (e.g. a pois.json
    // `facing`) instead of a navi-degree angle -- for a UNIT direction vector (sin(theta), cos(theta)) the
    // two must agree exactly.
    [Theory]
    [InlineData(0f, 1f, 0f)]      // facing north (SUMO +y) == navi 0
    [InlineData(1f, 0f, 90f)]     // facing east (SUMO +x) == navi 90
    [InlineData(0f, -1f, 180f)]   // facing south (SUMO -y) == navi 180
    [InlineData(-1f, 0f, 270f)]   // facing west (SUMO -x) == navi 270
    public void DirectionToGodotYawRad_UnitVector_MatchesNaviDegEquivalent(float dirX, float dirY, float naviDeg)
    {
        var yaw = CoordinateTransform.DirectionToGodotYawRad(dirX, dirY);
        AssertAngleClose(CoordinateTransform.NaviDegToGodotYawRad(naviDeg), yaw, toleranceRad: 1e-4f);
    }

    [Fact]
    public void DirectionToGodotYawRad_IsInvariantToVectorMagnitude()
    {
        // Only the direction matters -- scaling (dirX,dirY) by any positive factor must yield the same yaw.
        var yawUnit = CoordinateTransform.DirectionToGodotYawRad(0.6, 0.8);
        var yawScaled = CoordinateTransform.DirectionToGodotYawRad(6.0, 8.0);
        AssertAngleClose(yawUnit, yawScaled, toleranceRad: 1e-5f);
    }

    private static void AssertAngleClose(float expectedRad, float actualRad, float toleranceRad)
    {
        var diff = MathF.Atan2(MathF.Sin(expectedRad - actualRad), MathF.Cos(expectedRad - actualRad));
        Assert.True(MathF.Abs(diff) <= toleranceRad, $"expected {expectedRad} rad, got {actualRad} rad (diff {diff})");
    }
}
