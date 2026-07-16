using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using CityLib;
using Sim.Ingest;
using Xunit;

namespace CityLib.Tests;

// T1.6 success conditions (docs/DEMO-CITY3D-TASKS.md / this task's briefing): "one head per TL-controlled
// approach is generated; head colour index each frame equals the replicated signal letter (r/y/g) for
// that approach ... over a 09-traffic-light run observe red->green and green->(amber->)red transitions".
public class TrafficLightTests
{
    // ---- 1: ColorFor -- exhaustive small table (r/y/g/G/other) ----
    [Theory]
    [InlineData((byte)'r', SignalColor.Red)]
    [InlineData((byte)'y', SignalColor.Yellow)]
    [InlineData((byte)'g', SignalColor.Green)]
    [InlineData((byte)'G', SignalColor.Green)]
    [InlineData((byte)'o', SignalColor.Off)]
    [InlineData((byte)'u', SignalColor.Off)]
    [InlineData((byte)'Y', SignalColor.Off)] // only lowercase 'y' maps to Yellow per this task's spec
    [InlineData((byte)0, SignalColor.Off)]
    public void ColorFor_MapsEverySignalByte(byte signalByte, SignalColor expected)
    {
        Assert.Equal(expected, TrafficLightPlacer.ColorFor(signalByte));
    }

    // ---- 2: Place -- head at the controlled lane's downstream end (within tol), HeadY ~= 5, pole base
    // Y ~= ground (0 on this flat net) ----
    [Fact]
    public void Place_RealControlledLane_HeadAtDownstreamEnd_RaisedFiveMetres()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "09-traffic-light", "net.net.xml"));

        // WJ_0 is the one TL-controlled approach lane in 09-traffic-light (net.net.xml: <connection
        // from="WJ" to="JE" ... tl="J" .../>), shape (0.00,-1.60) -> (300.00,-1.60), width defaults to
        // SUMO_const_laneWidth (3.2m, the <lane> omits width entirely -- see NetworkModel.Lane's own doc
        // comment on the default).
        var handle = network.LaneHandleById["WJ_0"];

        var heads = TrafficLightPlacer.Place(network, new[] { handle });

        Assert.Single(heads);
        var head = heads[0];
        Assert.Equal(handle, head.LaneHandle);

        // Independently recomputed expectation (task Part A recipe): downstream end (300, -1.60), nudged
        // toward the right edge by half the default lane width (1.6m) along the segment normal
        // ((-dy,dx)/len = (0,1) for this due-east lane -- RoadMeshBuilder's "right = center -
        // normal*halfWidth" convention), then CoordinateTransform.SumoToGodot.
        const double halfWidth = 3.2 / 2.0;
        var (expX, expY, expZ) = CoordinateTransform.SumoToGodot(300.0, -1.60 - halfWidth, 0.0);

        const float tol = 1e-3f;
        Assert.Equal(expX, head.PoleX, tol);
        Assert.Equal(expY, head.PoleY, tol); // ground elevation on a flat net == 0
        Assert.Equal(expZ, head.PoleZ, tol);

        Assert.Equal(expX, head.HeadX, tol);
        Assert.Equal(expZ, head.HeadZ, tol);
        Assert.Equal(expY + TrafficLightPlacer.HeadHeightMeters, head.HeadY, tol);
        Assert.Equal(5f, head.HeadY - head.PoleY, tol); // HeadY ~= 5 above the pole base
        Assert.Equal(0f, head.PoleY, tol); // pole base ~= ground
    }

    [Fact]
    public void Place_UnknownOrOutOfRangeHandle_IsSkippedNotThrown()
    {
        var network = NetworkParser.Parse(Path.Combine(RepoRoot(), "scenarios", "09-traffic-light", "net.net.xml"));

        var heads = TrafficLightPlacer.Place(network, new[] { -1, 999999 });

        Assert.Empty(heads);
    }

    // ---- 3: end-to-end -- drive SimSource on 09-traffic-light, TlStateByLane becomes non-empty, Place
    // returns one head per controlled lane, and over the run at least one lane's ColorFor is NOT Green
    // (a red/amber is observed -- the light actually changes; this net's tlLogic starts red for 30s then
    // green for the remaining 20s of the 50s run, so both states are genuinely exercised). ----
    [Fact]
    public void EndToEnd_SimSourceOnTrafficLightScenario_PlacesOneHeadPerControlledLane_AndObservesNonGreen()
    {
        var scenarioDir = Path.Combine(RepoRoot(), "scenarios", "09-traffic-light");
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var rouPath = Path.Combine(scenarioDir, "rou.rou.xml");
        var cfgPath = Path.Combine(scenarioDir, "config.sumocfg");

        using var sim = new SimSource(netPath, rouPath, cfgPath);

        var everSawTlState = false;
        var everSawNonGreen = false;
        var everSawGreen = false;
        var maxHeadsSeen = 0;

        for (var tick = 0; tick < 45; tick++)
        {
            sim.Tick();

            var tlStateByLane = sim.Source.TlStateByLane;
            if (tlStateByLane.Count == 0)
            {
                continue;
            }

            everSawTlState = true;

            var heads = TrafficLightPlacer.Place(sim.Network, tlStateByLane.Keys);
            Assert.Equal(tlStateByLane.Count, heads.Count);
            maxHeadsSeen = Math.Max(maxHeadsSeen, heads.Count);

            foreach (var signalByte in tlStateByLane.Values)
            {
                var color = TrafficLightPlacer.ColorFor(signalByte);
                if (color != SignalColor.Green)
                {
                    everSawNonGreen = true;
                }
                else
                {
                    everSawGreen = true;
                }
            }
        }

        Assert.True(everSawTlState, "TlStateByLane never became non-empty over the run");
        Assert.True(maxHeadsSeen > 0, "expected at least one signal head to be placed");
        Assert.True(everSawNonGreen, "expected at least one red/amber (non-Green) signal state to be observed over the run");
        Assert.True(everSawGreen, "expected the light to also reach Green over the run (proves it actually changes, not stuck red)");
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
