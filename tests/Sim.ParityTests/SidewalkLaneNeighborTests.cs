using System.Linq;
using Sim.Ingest;
using Xunit;

namespace Sim.ParityTests;

// Regression for the demo-city "cars on the sidewalk" bug (docs/SUMOSHARP-NEED... cars-on-sidewalk):
// on a [sidewalk, car] edge the car lane's RightNeighbor used to be the pedestrian lane (Index-1), so
// keep-right / lateral placement moved a car onto the footpath. A pedestrian-only lane
// (`allow="pedestrian"`) must never be a road vehicle's lane-change neighbor. Pure-car nets (every
// committed golden) have no such lanes, so this is inert there -- proven by the unchanged parity gate.
public class SidewalkLaneNeighborTests
{
    private const string Net = """
        <net>
          <edge id="road" from="A" to="B">
            <lane id="road_0" index="0" allow="pedestrian" speed="5.56" length="100.0" shape="0.00,0.00 100.00,0.00"/>
            <lane id="road_1" index="1" disallow="pedestrian" speed="13.9" length="100.0" shape="0.00,3.20 100.00,3.20"/>
            <lane id="road_2" index="2" speed="13.9" length="100.0" shape="0.00,6.40 100.00,6.40"/>
          </edge>
          <edge id="plain" from="B" to="C">
            <lane id="plain_0" index="0" speed="13.9" length="50.0" shape="100.00,0.00 150.00,0.00"/>
            <lane id="plain_1" index="1" speed="13.9" length="50.0" shape="100.00,3.20 150.00,3.20"/>
          </edge>
        </net>
        """;

    [Fact]
    public void SidewalkLane_IsNotARoadVehicle_AndIsNeverANeighbor()
    {
        var net = NetworkParser.ParseXml(Net);
        var byId = net.LanesByHandle.ToDictionary(l => l.Id);

        var side = byId["road_0"];   // allow="pedestrian" -> sidewalk
        var carInner = byId["road_1"];
        var carOuter = byId["road_2"];

        // The sidewalk is not usable by road vehicles; the car lanes are.
        Assert.False(side.AllowsRoadVehicle);
        Assert.True(carInner.AllowsRoadVehicle);
        Assert.True(carOuter.AllowsRoadVehicle);

        // The car lane next to the sidewalk has NO right neighbor (the ped lane is excluded), so keep-right
        // can't move a car onto it. Its left neighbor is still the next car lane.
        Assert.Equal(-1, carInner.RightNeighbor);
        Assert.Equal(carOuter.Handle, carInner.LeftNeighbor);

        // The outer car lane keeps its normal right neighbor (the inner car lane).
        Assert.Equal(carInner.Handle, carOuter.RightNeighbor);
    }

    [Fact]
    public void PureCarEdge_NeighborsUnchanged_TheFixIsInertThere()
    {
        var net = NetworkParser.ParseXml(Net);
        var byId = net.LanesByHandle.ToDictionary(l => l.Id);

        // No pedestrian lane on this edge -> the two car lanes are each other's neighbors, exactly as before.
        Assert.True(byId["plain_0"].AllowsRoadVehicle);
        Assert.True(byId["plain_1"].AllowsRoadVehicle);
        Assert.Equal(byId["plain_0"].Handle, byId["plain_1"].RightNeighbor);
        Assert.Equal(byId["plain_1"].Handle, byId["plain_0"].LeftNeighbor);
    }
}
