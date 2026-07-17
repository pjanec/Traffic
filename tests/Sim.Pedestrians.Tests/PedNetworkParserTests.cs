using Xunit;

namespace Sim.Pedestrians.Tests;

// POC-0 success conditions (docs/PEDESTRIAN-POC-PLAN.md): Sim.Pedestrians' own ped-network
// ingest (NOT src/Sim.Ingest) loads scenarios/_ped/poc0-crossing-plaza/net.net.xml +
// walkable.add.xml and exposes pedestrian lanes, crossings, walkingAreas, and walkable
// polygons/access-points as distinct, queryable geometry — hermetically, from committed
// fixtures, with no SUMO dependency.
public class PedNetworkParserTests
{
    private static string FixturePath(string fileName) =>
        Path.Combine(AppContext.BaseDirectory, "Fixtures", fileName);

    private static PedNetwork LoadPoc0Network() =>
        PedNetworkParser.Load(FixturePath("net.net.xml"), FixturePath("walkable.add.xml"));

    [Fact]
    public void Crossings_AreFourSignalizedCrossingsWithShapeAndOutline()
    {
        var net = LoadPoc0Network();

        Assert.Equal(4, net.Crossings.Count);

        foreach (var crossing in net.Crossings)
        {
            Assert.NotEmpty(crossing.Shape);
            Assert.NotEmpty(crossing.Outline);
            Assert.Equal(4.0, crossing.Width, precision: 3);
            Assert.NotEmpty(crossing.CrossingEdges);
            Assert.Equal("c", crossing.TlLogicId);
        }
    }

    [Fact]
    public void WalkingAreas_AreEightWithNonEmptyPolygons()
    {
        var net = LoadPoc0Network();

        Assert.Equal(8, net.WalkingAreas.Count);
        Assert.All(net.WalkingAreas, wa => Assert.NotEmpty(wa.Polygon));
    }

    [Fact]
    public void Sidewalks_ArePresentWithNonEmptyShapes()
    {
        var net = LoadPoc0Network();

        Assert.True(net.Sidewalks.Count > 0);
        Assert.All(net.Sidewalks, sw => Assert.NotEmpty(sw.Shape));
    }

    [Fact]
    public void WalkablePolygons_ContainPlazaAndParkingLot()
    {
        var net = LoadPoc0Network();

        var plaza = Assert.Single(net.WalkablePolygons, p => p.Id == "plaza");
        Assert.NotEmpty(plaza.Shape);

        var parkingLot = Assert.Single(net.WalkablePolygons, p => p.Id == "parkinglot");
        Assert.NotEmpty(parkingLot.Shape);
    }

    [Fact]
    public void AccessPoints_ContainParkingLotEntryAndExit()
    {
        var net = LoadPoc0Network();

        Assert.Contains(net.AccessPoints, p => p.Id == "parkinglot_entry");
        Assert.Contains(net.AccessPoints, p => p.Id == "parkinglot_exit");
    }
}
