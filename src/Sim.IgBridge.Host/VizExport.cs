using System.Text.Json;
using Sim.Core;
using Sim.Ingest;
using Sim.IgBridge;

// T1.4 (docs/IGBRIDGE-TASKS.md): side-by-side HTML render, RAW-fed-IG vs IgBridge-fed-IG, reusing the
// committed Sim.Viz front-end (template.html + template.js) as a two-scene REPLAY_DATA payload -- the same
// marker-injection WriteHtml does, replicated here so the Host builds the payload from public FakeIg output
// without needing Sim.Viz's internal payload types. Both scenes are FakeIg reconstructions (the poses the
// IG would actually display); the ONLY difference is the input stream (raw engine @10Hz vs IgBridge-
// smoothed @20Hz), so toggling scenes shows exactly what IgBridge buys.
internal static class VizExport
{
    private static double R(double v) => Math.Round(v, 2, MidpointRounding.AwayFromZero);

    private const double DefaultLen = 5.0;  // fallback vehicle length when dims are unknown
    private const double DefaultWid = 1.9;  // fallback vehicle width

    // Navi-degree (0 = north, clockwise) unit vector.
    private static (double X, double Y) NaviDir(double naviDeg)
    {
        var m = (90.0 - naviDeg) * Math.PI / 180.0;
        return (Math.Cos(m), Math.Sin(m));
    }

    public static void WriteSideBySide(
        string repoRoot, NetworkModel network,
        (string Name, string Desc, FakeIg Ig) sceneA,
        (string Name, string Desc, FakeIg Ig) sceneB,
        double startT, double endT, double fps, string outPath,
        IReadOnlyDictionary<string, (double Length, double Width)>? vehDims = null)
    {
        vehDims ??= new Dictionary<string, (double, double)>();
        // Global vehicle slot map (union of both scenes -- same "v.." ids in both, so a vehicle keeps its
        // slot/colour across the toggle). Peds are drawn as discs, no slot needed.
        var vehIds = new SortedSet<string>(StringComparer.Ordinal);
        foreach (var ig in new[] { sceneA.Ig, sceneB.Ig })
        {
            foreach (var id in ig.Ids)
            {
                if (ig.ModelOf(id) == IgEntityModel.Car)
                {
                    vehIds.Add(id);
                }
            }
        }

        var slot = new Dictionary<string, int>();
        foreach (var id in vehIds)
        {
            slot[id] = slot.Count;
        }

        // Slot-indexed id list so the viewer can label a clicked vehicle (click-to-identify). Same slot map
        // in both scenes, so a latched id follows the same car across the raw/smoothed toggle.
        var vehIdsBySlot = new string[slot.Count];
        foreach (var kv in slot)
        {
            vehIdsBySlot[kv.Value] = kv.Key;
        }

        var net = BuildNetwork(network);
        var view = ActivityView(sceneA.Ig, sceneB.Ig, startT, endT, fps);
        var dt = 1.0 / fps;

        var scenes = new[]
        {
            BuildScene(sceneA.Name, sceneA.Desc, sceneA.Ig, slot, vehIdsBySlot, vehDims, view, net, dt, startT, endT, fps),
            BuildScene(sceneB.Name, sceneB.Desc, sceneB.Ig, slot, vehIdsBySlot, vehDims, view, net, dt, startT, endT, fps),
        };

        var json = JsonSerializer.Serialize(new { scenes },
            new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase });

        var tdir = Path.Combine(repoRoot, "src", "Sim.Viz");
        var html = File.ReadAllText(Path.Combine(tdir, "template.html"))
            .Replace("__SCENARIO_NAME__", "IgBridge: raw engine vs IgBridge-smoothed (IG-displayed)")
            .Replace("/*REPLAY_DATA*/", json)
            .Replace("/*TEMPLATE_JS*/", File.ReadAllText(Path.Combine(tdir, "template.js")));

        File.WriteAllText(outPath, html);
    }

    private static object BuildScene(
        string name, string desc, FakeIg ig, Dictionary<string, int> slot, string[] vehIdsBySlot,
        IReadOnlyDictionary<string, (double Length, double Width)> vehDims,
        double[] view, object net, double dt, double startT, double endT, double fps)
    {
        var frames = new List<object>();
        for (var t = startT; t <= endT + 1e-9; t += dt)
        {
            var v = new double[]?[slot.Count];
            foreach (var kv in slot)
            {
                if (ig.TryDisplayPose(kv.Key, t, out var p) && ig.ModelOf(kv.Key) == IgEntityModel.Car)
                {
                    // The trace carries the vehicle CENTER (IG pivots on center); the Sim.Viz template
                    // anchors the box at the FRONT (rect extends backward), so shift forward by half THIS
                    // vehicle's own length along the heading. Emitting the true per-vehicle length+width as a
                    // 5-tuple lets the viewer draw a long bus long and a car short (a 3/4-tuple has no dims;
                    // only IgBridge emits 5) -- the shift and the drawn box then agree, so the box stays
                    // center-pivoted at whatever length. Rotating the rigid rect about its front anchor
                    // reproduces the identical box a center-pivot draw would (front = center + halfLen*dir).
                    var hasDim = vehDims.TryGetValue(kv.Key, out var vd);
                    var len = hasDim ? vd.Length : DefaultLen;
                    var wid = hasDim ? vd.Width : DefaultWid;
                    var (dx, dy) = NaviDir(p.HeadingDeg);
                    var fx = p.X + len * 0.5 * dx;
                    var fy = p.Y + len * 0.5 * dy;
                    v[kv.Value] = new[] { R(fx), R(fy), R(p.HeadingDeg), R(len), R(wid) };
                }
            }

            var d = new List<double[]>();
            foreach (var id in ig.Ids)
            {
                if (ig.ModelOf(id) == IgEntityModel.Ped && ig.TryDisplayPose(id, t, out var pp))
                {
                    d.Add(new[] { R(pp.X), R(pp.Y), 0.4, 2.0 }); // kind 2 = pedestrian
                }
            }

            frames.Add(new { v, d = d.ToArray() });
        }

        return new
        {
            name,
            desc,
            view,
            network = net,
            vdim = new[] { 5.0, 1.9 }, // shared vehicle box (representative passenger dims)
            vehIds = vehIdsBySlot,     // slot-indexed ids -> click-to-identify in the viewer
            useDataHeading = true,     // draw the emitted (kinematic) heading, NOT the front-anchor path tangent
            dt,
            frames = frames.ToArray(),
        };
    }

    // View = the 5th..95th percentile bbox of vehicle positions over the window, so the (huge ~4750 m)
    // city crops to where the traffic actually is instead of rendering cars as specks.
    private static double[] ActivityView(FakeIg a, FakeIg b, double startT, double endT, double fps)
    {
        var xs = new List<double>();
        var ys = new List<double>();
        var dt = 1.0 / fps;
        foreach (var ig in new[] { a, b })
        {
            foreach (var id in ig.Ids)
            {
                if (ig.ModelOf(id) != IgEntityModel.Car)
                {
                    continue;
                }

                for (var t = startT; t <= endT + 1e-9; t += dt)
                {
                    if (ig.TryDisplayPose(id, t, out var p))
                    {
                        xs.Add(p.X);
                        ys.Add(p.Y);
                    }
                }
            }
        }

        if (xs.Count == 0)
        {
            return new[] { 0.0, 0.0, 100.0, 100.0 };
        }

        // Center on the MEDIAN vehicle position (the densest area) and crop to a fixed ~halfExtent window,
        // so cars are large enough to watch a junction turn / lane change rather than rendering as specks
        // across the whole ~4750 m city. Entities outside the window simply draw off-view.
        xs.Sort();
        ys.Sort();
        const double HalfExtent = 160.0; // metres -> ~320 m window
        var cx = xs[xs.Count / 2];
        var cy = ys[ys.Count / 2];
        return new[] { R(cx - HalfExtent), R(cy - HalfExtent), R(cx + HalfExtent), R(cy + HalfExtent) };
    }

    private static object BuildNetwork(NetworkModel network)
    {
        var lanes = new List<object>();
        foreach (var lane in network.LanesByHandle)
        {
            var flat = new double[lane.Shape.Count * 2];
            for (var p = 0; p < lane.Shape.Count; p++)
            {
                flat[p * 2] = R(lane.Shape[p].X);
                flat[p * 2 + 1] = R(lane.Shape[p].Y);
            }

            lanes.Add(new
            {
                id = lane.Id,
                edgeId = lane.EdgeId,
                index = lane.Index,
                width = lane.Width,
                shape = flat,
                ped = !lane.AllowsRoadVehicle,
            });
        }

        var junctions = new List<object>();
        foreach (var j in network.Junctions)
        {
            if (j.Shape.Count == 0)
            {
                continue;
            }

            var flat = new double[j.Shape.Count * 2];
            for (var p = 0; p < j.Shape.Count; p++)
            {
                flat[p * 2] = R(j.Shape[p].X);
                flat[p * 2 + 1] = R(j.Shape[p].Y);
            }

            junctions.Add(new { id = j.Id, shape = flat });
        }

        return new
        {
            lanes = lanes.ToArray(),
            junctions = junctions.ToArray(),
            tls = Array.Empty<object>(),
            signals = Array.Empty<object>(),
            crossings = Array.Empty<object>(),
            pedSignals = Array.Empty<object>(),
        };
    }
}
