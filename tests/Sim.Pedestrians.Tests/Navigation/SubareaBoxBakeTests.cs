using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text.Json;
using Sim.Pedestrians;
using Sim.Pedestrians.Navigation.Bake;
using Xunit;
using Xunit.Abstractions;

namespace Sim.Pedestrians.Tests.Navigation;

// P8-1 (docs/PEDESTRIAN-TRACKER.md Stage P8; docs/COORDINATION-pedestrian-x-subarea.md §1 req 1): verify the
// SUMO-geometry bake against the REAL cropped sub-area net the SumoData session shipped
// (scenarios/_ped/subarea-box, unblocking P8). The box is a synthetic-but-real-shaped crop carrying
// pedestrian infrastructure (sidewalks / crossings / walkingAreas) plus a manifest whose
// subarea.fringe_edges is the P8-2 no-cheating boundary. This proves (a) the bake CONSUMES the crop into a
// connected, pathable navmesh, and (b) the walkable-fringe contract the P8-2 spawn gate will read is valid
// against the baked geometry. Hermetic: reads the committed net.xml + manifest.json only (no SUMO).
public class SubareaBoxBakeTests
{
    private readonly ITestOutputHelper _out;

    public SubareaBoxBakeTests(ITestOutputHelper output) => _out = output;

    private static string RepoRoot()
    {
        var dir = AppContext.BaseDirectory;
        while (dir is not null && !File.Exists(Path.Combine(dir, "Traffic.sln")))
        {
            dir = Path.GetDirectoryName(dir);
        }

        Assert.NotNull(dir);
        return dir!;
    }

    private static string BoxDir() => Path.Combine(RepoRoot(), "scenarios", "_ped", "subarea-box");

    [Fact]
    public void Bake_ConsumesTheSubareaCrop_ProducingAConnectedPathableNavmesh()
    {
        var network = PedNetworkParser.Load(Path.Combine(BoxDir(), "net.xml"));

        // The crop carries real pedestrian infrastructure -> the bake must yield walkable geometry.
        Assert.NotEmpty(network.Sidewalks);
        Assert.NotEmpty(network.Crossings);
        Assert.NotEmpty(network.WalkingAreas);

        var polygons = WalkablePolygonBaker.Bake(network);
        Assert.True(polygons.Count > 100,
            $"expected a rich walkable bake from the 6x6+fringe crop, got {polygons.Count} polygons");

        var space = new SumoWalkableSpace(polygons);
        var nav = new SumoNavMesh(polygons, space);

        // Connectivity: a ped can path across the box. Route between the walkable polygons nearest opposite
        // corners of the 0,0 -> 800,800 box (their centroids are guaranteed to lie on walkable surface).
        var lo = polygons.OrderBy(p => p.Centroid.X + p.Centroid.Y).First().Centroid;
        var hi = polygons.OrderByDescending(p => p.Centroid.X + p.Centroid.Y).First().Centroid;
        var path = nav.FindPath(lo, hi);
        Assert.True(path is { Count: >= 2 },
            $"navmesh could not path across the box from ({lo.X:F1},{lo.Y:F1}) to ({hi.X:F1},{hi.Y:F1})");

        _out.WriteLine($"[P8-1] baked {polygons.Count} walkable polygons; " +
                       $"sidewalks={network.Sidewalks.Count} crossings={network.Crossings.Count} " +
                       $"walkingAreas={network.WalkingAreas.Count}; cross-box path = {path!.Count} waypoints");
    }

    [Fact]
    public void WalkableFringe_PinsTheManifestContract()
    {
        var network = PedNetworkParser.Load(Path.Combine(BoxDir(), "net.xml"));
        var sidewalkEdges = network.Sidewalks.Select(s => s.EdgeId).ToHashSet();

        using var manifest = JsonDocument.Parse(File.ReadAllText(Path.Combine(BoxDir(), "manifest.json")));
        var subarea = manifest.RootElement.GetProperty("subarea");
        var fringe = subarea.GetProperty("fringe_edges");

        var pedFringe = new List<string>();
        foreach (var e in fringe.EnumerateArray())
        {
            if (e.GetProperty("ped").GetBoolean())
            {
                pedFringe.Add(e.GetProperty("id").GetString()!);
            }
        }

        // The manifest advertises 48 walkable fringe edges (the P8-2 appearance-legitimacy boundary); pin it.
        Assert.Equal(subarea.GetProperty("fringe_walkable_count").GetInt32(), pedFringe.Count);
        Assert.Equal(48, pedFringe.Count);

        // Every ped-fringe edge must exist as a real sidewalk edge in the baked net, so the fringe contract
        // the P8-2 spawn gate reads is grounded in the actual walkable geometry (no dangling ids).
        foreach (var id in pedFringe)
        {
            Assert.Contains(id, sidewalkEdges);
        }

        _out.WriteLine($"[P8-1] pinned {pedFringe.Count} walkable-fringe edges, all present as baked sidewalks");
    }
}
