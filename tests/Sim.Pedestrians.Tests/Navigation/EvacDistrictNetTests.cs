using Sim.Core.Orca;
using Sim.Pedestrians;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Navigation;

// P5-PRE (docs/PEDESTRIAN-TASKS.md): the synthetic pedestrian evac-district net -- a 3x3 sidewalk grid
// (incident-able centre, four corner SAFE ZONES) -- must bake cleanly through the same
// PedNetworkParser -> WalkablePolygonBaker -> SumoNavMesh chain the real scenarios use, and a route from
// the interior to a safe zone must go ALONG the walkable grid (bending around the blocks), not a
// straight radial line through the buildings. This is the prerequisite the evac (P5-B) routed foot-exodus
// stands on: no real ped net existed, so this synthetic one is committed and verified here.
public class EvacDistrictNetTests
{
    private readonly ITestOutputHelper _output;

    public EvacDistrictNetTests(ITestOutputHelper output)
    {
        _output = output;
    }

    // n11 (grid centre) = incident epicentre; n00 (a corner) = a fringe safe zone.
    private static readonly Vec2 Centre = new(100.0, 100.0);
    private static readonly Vec2 CornerSafeZone = new(0.0, 0.0);

    private static SumoNavMesh BuildNav()
    {
        var net = PedNetworkParser.Load(
            System.IO.Path.Combine(AppContext.BaseDirectory, "Fixtures", "evac-district", "net.net.xml"));
        var polygons = WalkablePolygonBaker.Bake(net);
        return new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));
    }

    [Fact]
    public void EvacDistrict_Bakes_AndRoutesInteriorToSafeZone_AlongWalkableSpace_NotRadially()
    {
        var nav = BuildNav();

        var path = nav.FindPath(Centre, CornerSafeZone);
        Assert.NotNull(path);
        Assert.True(path!.Count >= 2, "a route must have at least two waypoints");

        // Arc length of the routed path vs the straight-line (radial) distance. A real sidewalk route
        // from the grid centre out to a corner threads along the streets around the blocks, so it is
        // meaningfully LONGER than the ~141 m straight line -- the whole point of routing over walkable
        // space rather than fleeing radially through the buildings.
        var arc = 0.0;
        for (var i = 0; i + 1 < path.Count; i++)
        {
            arc += (path[i + 1] - path[i]).Abs;
        }

        var straight = (CornerSafeZone - Centre).Abs;
        _output.WriteLine($"[P5-PRE measured] route waypoints={path.Count} arcLength={arc:F1} m " +
            $"straightLine={straight:F1} m ratio={arc / straight:F2}");

        Assert.True(path.Count > 2, $"expected a multi-segment (bending) route, got {path.Count} waypoints");
        Assert.True(arc > 1.2 * straight,
            $"routed arc {arc:F1} m should exceed 1.2x the straight-line {straight:F1} m (i.e. route AROUND blocks, not through them)");
    }

    [Fact]
    public void EvacDistrict_AllFourCornerSafeZones_AreRoutableFromTheCentre()
    {
        var nav = BuildNav();
        var corners = new[] { new Vec2(0, 0), new Vec2(200, 0), new Vec2(0, 200), new Vec2(200, 200) };

        foreach (var corner in corners)
        {
            var path = nav.FindPath(Centre, corner);
            Assert.True(path is { Count: >= 2 }, $"safe zone {corner} was not routable from the centre");
        }
    }
}
