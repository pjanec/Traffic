using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using CityLib;
using Sim.Host;
using Sim.Ingest;
using Sim.Replication;
using Xunit;

namespace CityLib.Tests;

// docs/LIVE-CITY-VIEWERS-TASKS.md Stage E (E1) success condition: "a two-process inmem/DDS round-trip
// over the synthetic elevated net delivers non-zero Z to the subscriber; remote road/car Y matches the
// local path on that net (parity of the two ILaneShapeSource impls)". Uses the SAME committed fixture
// (fixtures/elevated.net.xml) Stage D's RoadMeshTests already exercises for the LOCAL path, and drives the
// "remote" half over an actual IReplicationSink/IReplicationSource pair (InMemoryReplicationBus is a real
// binding of the transport-neutral contract, exactly what DdsReplicationSink/DdsSubscriber also implement
// -- see GeometryCodec's own header comment: the DDS geometry topic carries these exact codec bytes
// unchanged, so this in-process transport proves the same wire encode/decode path DDS would).
public class RoadMeshWireZParityTests
{
    private static string FixturePath =>
        Path.Combine(AppContext.BaseDirectory, "fixtures", "elevated.net.xml");

    [Fact]
    public void WireGeometry_DeliversNonZeroZ_ForElevatedFixtureRampLane()
    {
        var network = NetworkParser.Parse(FixturePath);
        var rampLane = network.LanesByHandle.Single(l => l.Id == "E0_0");
        Assert.NotNull(rampLane.ShapeZ);

        var bus = new InMemoryReplicationBus();
        var publisher = new ReplicationPublisher();
        publisher.PublishGeometryOnce(network, bus.Sink);
        bus.Source.Pump();

        Assert.True(bus.Source.GeometryComplete);
        Assert.True(bus.Source.Geometry.TryGetValue(rampLane.Handle, out var wireLane));
        Assert.NotNull(wireLane.Z);
        Assert.Equal(rampLane.ShapeZ!.Count, wireLane.Z!.Length);
        for (var i = 0; i < rampLane.ShapeZ.Count; i++)
        {
            Assert.Equal(rampLane.ShapeZ[i], wireLane.Z[i], 3);
        }

        var flatLane = network.LanesByHandle.Single(l => l.Id == "E1_0");
        Assert.Null(flatLane.ShapeZ);
        Assert.True(bus.Source.Geometry.TryGetValue(flatLane.Handle, out var wireFlatLane));
        Assert.Null(wireFlatLane.Z); // per-lane, not a global net flag -- the flat lane on the SAME frame has no Z
    }

    [Fact]
    public void RoadMeshBuilder_FromWireGeometry_MatchesLocalNetworkModel_OnElevatedFixture()
    {
        var network = NetworkParser.Parse(FixturePath);

        var bus = new InMemoryReplicationBus();
        var publisher = new ReplicationPublisher();
        publisher.PublishGeometryOnce(network, bus.Sink);
        bus.Source.Pump();

        // LOCAL half (T1.2's "seam check" pattern): NetworkModel -> RoadMeshBuilder directly.
        var fromNetwork = RoadMeshBuilder.BuildAll(network, includeInternal: true).ToDictionary(x => x.Handle, x => x.Mesh);

        // "REMOTE" half: the wire-fed ILaneShapeSource + the geometry-dictionary RoadMeshBuilder overload,
        // exactly what a DDS-fed City3D remote viewer would build from DdsSubscriber.Geometry.
        var wireLanes = new ReplicationLaneShapeSource(bus.Source.Geometry);
        var fromWire = RoadMeshBuilder.BuildAll(bus.Source.Geometry, includeInternal: true).ToDictionary(x => x.Handle, x => x.Mesh);

        Assert.Equal(fromNetwork.Count, fromWire.Count);

        var rampLane = network.LanesByHandle.Single(l => l.Id == "E0_0");
        var rampFromNetwork = fromNetwork[rampLane.Handle];
        var rampFromWire = fromWire[rampLane.Handle];

        Assert.Equal(rampFromNetwork.Vertices.Length, rampFromWire.Vertices.Length);

        // Vertex Y (elevation, in Godot space) must match within the wire's float32 quantization -- proving
        // "remote road Y matches the local path" for the elevated ramp lane specifically (not just area).
        for (var i = 1; i < rampFromNetwork.Vertices.Length; i += 3)
        {
            Assert.Equal(rampFromNetwork.Vertices[i], rampFromWire.Vertices[i], 1e-3f);
        }

        // Non-constant Y on the wire-built mesh too (the actual "delivers real elevation" proof, not just
        // "happens to be zero on both sides").
        var wireYs = new List<float>();
        for (var i = 1; i < rampFromWire.Vertices.Length; i += 3)
        {
            wireYs.Add(rampFromWire.Vertices[i]);
        }

        Assert.True(wireYs.Distinct().Count() > 1, "expected non-constant vertex Y from the WIRE-fed geometry on the elevated ramp lane");
        Assert.Equal(0.0f, wireYs[0], 1e-3f);
        Assert.Equal(8.0f, wireYs[^1], 1e-3f);

        // ILaneShapeSource.LaneShapeZ agrees with RoadMeshBuilder's own decode too (same source data).
        var wireShapeZ = wireLanes.LaneShapeZ(rampLane.Handle);
        Assert.NotNull(wireShapeZ);
        Assert.Equal(0.0, wireShapeZ![0], 3);
        Assert.Equal(8.0, wireShapeZ[^1], 3);

        // Flat lane: both sides stay ~0.
        var flatLane = network.LanesByHandle.Single(l => l.Id == "E1_0");
        var flatFromWire = fromWire[flatLane.Handle];
        for (var i = 1; i < flatFromWire.Vertices.Length; i += 3)
        {
            Assert.Equal(0.0f, flatFromWire.Vertices[i], 1e-5f);
        }
    }
}
