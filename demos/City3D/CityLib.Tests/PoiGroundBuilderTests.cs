using System.Collections.Generic;
using System.Linq;
using CityLib;
using Sim.LiveCity;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VISUALS-NOTES.md OWNER-CONFIRMED "POI / area rendering" block: CityLib.PoiGroundBuilder's
// pure POI -> flat-marker math (per-kind radius, the SUMO->Godot ground-offset elevation, and the
// `building_entrance` exclusion -- that kind becomes a door, not a ground marker).
public class PoiGroundBuilderTests
{
    // ---- 1: `building_entrance` POIs are excluded (DoorBuilder owns that kind). ----
    [Fact]
    public void Build_ExcludesBuildingEntrance()
    {
        var pois = new[]
        {
            new ScenePoi("v1", "venue", 10, 20),
            new ScenePoi("e1", "building_entrance", 30, 40, FacingX: 1, FacingY: 0),
            new ScenePoi("t1", "transit_stop", 50, 60),
        };

        var markers = PoiGroundBuilder.Build(pois);

        Assert.Equal(2, markers.Count);
        Assert.DoesNotContain(markers, m => m.Kind == "building_entrance");
        Assert.Contains(markers, m => m.Kind == "venue");
        Assert.Contains(markers, m => m.Kind == "transit_stop");
    }

    // ---- 2: per-kind radius -- venue biggest, parking_access smallest/subtle (owner note). ----
    [Theory]
    [InlineData("venue", PoiGroundBuilder.VenueRadiusMeters)]
    [InlineData("transit_stop", PoiGroundBuilder.TransitStopRadiusMeters)]
    [InlineData("dwell_spot", PoiGroundBuilder.DwellSpotRadiusMeters)]
    [InlineData("parking_access", PoiGroundBuilder.ParkingAccessRadiusMeters)]
    [InlineData("unknown_kind", PoiGroundBuilder.DefaultRadiusMeters)]
    public void RadiusForKind_MatchesPerKindConstant(string kind, float expectedRadius)
    {
        Assert.Equal(expectedRadius, PoiGroundBuilder.RadiusForKind(kind));
    }

    [Fact]
    public void RadiusForKind_ParkingAccessIsSmallestOfTheStyledKinds()
    {
        Assert.True(PoiGroundBuilder.ParkingAccessRadiusMeters < PoiGroundBuilder.DwellSpotRadiusMeters);
        Assert.True(PoiGroundBuilder.ParkingAccessRadiusMeters < PoiGroundBuilder.TransitStopRadiusMeters);
        Assert.True(PoiGroundBuilder.ParkingAccessRadiusMeters < PoiGroundBuilder.VenueRadiusMeters);
        Assert.True(PoiGroundBuilder.VenueRadiusMeters >= PoiGroundBuilder.TransitStopRadiusMeters);
    }

    // ---- 3: Godot-space mapping + ground offset -- SumoToGodot(x,y,GroundOffsetSumoZ), so PosY is exactly
    // the (small, positive) offset, and PosX/PosZ follow the (x, -y) mapping. ----
    [Fact]
    public void Build_MapsToGodotSpaceWithGroundOffset()
    {
        var pois = new[] { new ScenePoi("v1", "venue", 10.0, 20.0) };

        var markers = PoiGroundBuilder.Build(pois);

        var m = Assert.Single(markers);
        Assert.Equal(10f, m.PosX, 1e-6f);
        Assert.Equal((float)PoiGroundBuilder.GroundOffsetSumoZ, m.PosY, 1e-6f);
        Assert.Equal(-20f, m.PosZ, 1e-6f);
    }

    // ---- 4: an empty POI list yields an empty marker list (never throws). ----
    [Fact]
    public void Build_EmptyList_ReturnsEmpty()
    {
        Assert.Empty(PoiGroundBuilder.Build(System.Array.Empty<ScenePoi>()));
    }

    // ---- 5: order is preserved (minus the excluded kind). ----
    [Fact]
    public void Build_PreservesInputOrder()
    {
        var pois = new[]
        {
            new ScenePoi("a", "venue", 0, 0),
            new ScenePoi("b", "dwell_spot", 1, 1),
            new ScenePoi("c", "parking_access", 2, 2),
        };

        var markers = PoiGroundBuilder.Build(pois);

        Assert.Equal(new[] { "venue", "dwell_spot", "parking_access" }, markers.Select(m => m.Kind).ToArray());
    }
}
