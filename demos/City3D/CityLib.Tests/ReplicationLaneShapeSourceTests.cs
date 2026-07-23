using System;
using System.Collections.Generic;
using System.IO;
using CityLib;
using Xunit;

namespace CityLib.Tests;

// T1.2 item 2 (docs/DEMO-CITY3D-TASKS.md): "ReplicationLaneShapeSource returns lane polylines matching the
// published geometry (spot-check >=3 lanes: endpoint coords within 1e-3 of the network's)."
// scenarios/09-traffic-light's net has exactly 3 lanes (WJ_0, JE_0, and the internal :J_0_0), so this
// spot-checks all of them.
public class ReplicationLaneShapeSourceTests
{
    private static readonly string ScenarioDir = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light");

    [Fact]
    public void LaneShape_MatchesNetworkEndpoints_ForEveryPublishedLane()
    {
        var netPath = Path.Combine(ScenarioDir, "net.net.xml");
        var rouPath = Path.Combine(ScenarioDir, "rou.rou.xml");
        var cfgPath = Path.Combine(ScenarioDir, "config.sumocfg");

        using var sim = new SimSource(netPath, rouPath, cfgPath);
        sim.Tick(); // publishes geometry (once) + the first frame

        Assert.True(sim.Source.GeometryComplete);
        Assert.True(sim.Source.Geometry.Count >= 3, $"expected >=3 lanes, got {sim.Source.Geometry.Count}");

        var wire = new ReplicationLaneShapeSource(sim.Source.Geometry);

        var checkedLanes = 0;
        foreach (var lane in sim.Network.LanesByHandle)
        {
            var handle = lane.Handle;
            Assert.True(sim.Source.Geometry.ContainsKey(handle), $"lane handle {handle} ({lane.Id}) missing from published geometry");

            var wireShape = wire.LaneShape(handle);
            var netShape = lane.Shape;

            Assert.Equal(netShape.Count, wireShape.Count);

            // Endpoint spot-check (first + last point), within 1e-3 -- the wire encodes points as float32,
            // so a tiny quantization delta is expected and 1e-3 comfortably covers it.
            AssertPointClose(netShape[0], wireShape[0]);
            AssertPointClose(netShape[^1], wireShape[^1]);

            // Also assert LaneLength matches (the other half of ILaneShapeSource PoseResolver relies on).
            Assert.Equal(lane.Length, wire.LaneLength(handle), 3);

            // docs/LIVE-CITY-VIEWERS-TASKS.md Stage E (E1): the wire CAN carry elevation now (GeometryCodec
            // LaneGeo.Z), but this scenario's net is flat (no lane has ShapeZ), so it is still null here --
            // see RoadMeshTests' elevated-fixture coverage for the non-null case.
            Assert.Null(wire.LaneShapeZ(handle));

            checkedLanes++;
        }

        Assert.True(checkedLanes >= 3, $"only checked {checkedLanes} lanes, need >=3");
    }

    [Fact]
    public void LaneShape_UnknownHandle_ThrowsGracefully()
    {
        var netPath = Path.Combine(ScenarioDir, "net.net.xml");
        var rouPath = Path.Combine(ScenarioDir, "rou.rou.xml");
        var cfgPath = Path.Combine(ScenarioDir, "config.sumocfg");

        using var sim = new SimSource(netPath, rouPath, cfgPath);
        sim.Tick();

        var wire = new ReplicationLaneShapeSource(sim.Source.Geometry);
        Assert.Throws<KeyNotFoundException>(() => wire.LaneShape(999_999));
        Assert.Throws<KeyNotFoundException>(() => wire.LaneLength(999_999));
    }

    private static void AssertPointClose((double X, double Y) expected, (double X, double Y) actual)
    {
        Assert.True(Math.Abs(expected.X - actual.X) < 1e-3, $"X: expected {expected.X}, got {actual.X}");
        Assert.True(Math.Abs(expected.Y - actual.Y) < 1e-3, $"Y: expected {expected.Y}, got {actual.Y}");
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
