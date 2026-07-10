using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;
using Sim.Core;
using Sim.Harness;
using Sim.Ingest;

// VB-1..VB-4 (VIZ_BENCH_TASKS.md / VIZ_SPEC.md): reads a scenario directory's net + fcd + rou,
// builds the compact REPLAY_DATA JSON payload the committed front-end template consumes, and
// writes a fully self-contained `replay.html` next to the inputs.
//
// Usage:
//   dotnet run --project src/Sim.Viz -- <scenarioDir> [--fcd <path>]
//
// Default FCD input is <scenarioDir>/golden.fcd.xml; pass --fcd to point at an engine
// engine.fcd.xml (VB-0's Sim.Run output) instead. Not part of `dotnet test` -- a utility, like
// Sim.Run/Sim.Bench.
internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            Console.Error.WriteLine("usage: Sim.Viz <scenarioDir> [--fcd <path>]");
            return args.Length == 0 ? 2 : 0;
        }

        var scenarioDir = args[0];
        if (!Directory.Exists(scenarioDir))
        {
            Console.Error.WriteLine($"error: scenario directory not found: {scenarioDir}");
            return 2;
        }

        string? fcdOverride = null;
        for (var i = 1; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--fcd" when i + 1 < args.Length:
                    fcdOverride = args[++i];
                    break;
                default:
                    Console.Error.WriteLine($"error: unrecognized argument: {args[i]}");
                    return 2;
            }
        }

        var netPath = SingleFile(scenarioDir, "*.net.xml");
        var rouPath = SingleFile(scenarioDir, "*.rou.xml");
        var cfgPath = SingleFile(scenarioDir, "*.sumocfg");
        var fcdPath = fcdOverride ?? Path.Combine(scenarioDir, "golden.fcd.xml");

        if (netPath is null || rouPath is null)
        {
            Console.Error.WriteLine(
                $"error: scenario dir must contain exactly one each of *.net.xml, *.rou.xml " +
                $"(found net={netPath}, rou={rouPath})");
            return 2;
        }

        if (!File.Exists(fcdPath))
        {
            Console.Error.WriteLine($"error: FCD file not found: {fcdPath}");
            return 2;
        }

        var network = NetworkParser.Parse(netPath);
        var demand = DemandParser.Parse(rouPath);
        var trajectorySet = FcdParser.Parse(fcdPath);
        var config = cfgPath is not null ? ScenarioConfigParser.Parse(cfgPath) : null;

        var payload = BuildPayload(Path.GetFileName(Path.GetFullPath(scenarioDir).TrimEnd('/', '\\')),
            network, demand, trajectorySet, config);

        var jsonOptions = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            DefaultIgnoreCondition = JsonIgnoreCondition.Never,
            WriteIndented = false,
        };
        var json = JsonSerializer.Serialize(payload, jsonOptions);

        var templateDir = AppContext.BaseDirectory;
        var templateHtmlPath = Path.Combine(templateDir, "template.html");
        var templateJsPath = Path.Combine(templateDir, "template.js");
        if (!File.Exists(templateHtmlPath) || !File.Exists(templateJsPath))
        {
            Console.Error.WriteLine(
                $"error: template files not found next to the built exe ({templateHtmlPath}, {templateJsPath})");
            return 2;
        }

        var html = File.ReadAllText(templateHtmlPath);
        var js = File.ReadAllText(templateJsPath);

        html = html.Replace("__SCENARIO_NAME__", payload.Scenario);
        html = html.Replace("/*REPLAY_DATA*/", json);
        html = html.Replace("/*TEMPLATE_JS*/", js);

        var outPath = Path.Combine(scenarioDir, "replay.html");
        File.WriteAllText(outPath, html);

        Console.WriteLine(
            $"wrote {outPath}  (lanes={payload.Network.Lanes.Length}, junctions={payload.Network.Junctions.Length}, " +
            $"vehicles={payload.Vehicles.Count}, steps={payload.Trajectory.Length}, " +
            $"t=[{payload.SimStart:0.###},{payload.SimEnd:0.###}])");
        return 0;
    }

    private static string? SingleFile(string dir, string pattern)
    {
        var matches = Directory.GetFiles(dir, pattern);
        return matches.Length == 1 ? matches[0] : null;
    }

    private static ReplayData BuildPayload(
        string scenarioName,
        NetworkModel network,
        DemandModel demand,
        TrajectorySet trajectorySet,
        ScenarioConfig? config)
    {
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;

        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        var lanes = new LanePayload[network.LanesByHandle.Count];
        for (var i = 0; i < network.LanesByHandle.Count; i++)
        {
            var lane = network.LanesByHandle[i];
            var flat = new double[lane.Shape.Count * 2];
            for (var p = 0; p < lane.Shape.Count; p++)
            {
                var (x, y) = lane.Shape[p];
                flat[p * 2] = x;
                flat[p * 2 + 1] = y;
                Track(x, y);
            }

            lanes[i] = new LanePayload(lane.Id, lane.EdgeId, lane.Index, lane.Width, flat);
        }

        var junctions = new List<JunctionPayload>();
        foreach (var junction in network.Junctions)
        {
            if (junction.Shape.Count == 0)
            {
                continue;
            }

            var flat = new double[junction.Shape.Count * 2];
            for (var p = 0; p < junction.Shape.Count; p++)
            {
                var (x, y) = junction.Shape[p];
                flat[p * 2] = x;
                flat[p * 2 + 1] = y;
                Track(x, y);
            }

            junctions.Add(new JunctionPayload(junction.Id, flat));
        }

        var tls = network.TlLogicsById.Values
            .Select(tl => new TlLogicPayload(
                tl.Id,
                tl.Offset,
                tl.Phases.Select(p => new TlPhasePayload(p.Duration, p.State)).ToArray()))
            .ToArray();

        // Signal heads: one per TL-controlled <connection>, placed at the stop-line end (arc
        // length = the from-lane's own Length) of that lane -- VIZ_SPEC.md "Traffic lights"
        // layer. LaneGeometry.PositionAtOffset is the exact same derivation the engine already
        // uses for FCD x/y, so the marker sits precisely at the lane's physical end point.
        var signals = new List<SignalHeadPayload>();
        foreach (var connection in network.Connections)
        {
            if (connection.Tl is null || connection.LinkIndex is null)
            {
                continue;
            }

            if (!network.EdgesById.TryGetValue(connection.From, out var fromEdge))
            {
                continue;
            }

            var fromLane = fromEdge.Lanes.FirstOrDefault(l => l.Index == connection.FromLane);
            if (fromLane is null)
            {
                continue;
            }

            var (x, y, angle) = LaneGeometry.PositionAtOffset(fromLane.Shape, fromLane.Length);
            signals.Add(new SignalHeadPayload(connection.Tl, connection.LinkIndex.Value, x, y, angle));
        }

        var networkPayload = new NetworkPayload(lanes, junctions.ToArray(), tls, signals.ToArray());

        // Vehicle dimension records: join FCD vehicle id -> rou.xml <vehicle type=...> ->
        // <vType> -> VTypeDefaults.Resolve (VIZ_SPEC.md "Inputs" #3). Vehicles that appear in
        // the FCD but have no matching rou.xml <vehicle> (shouldn't happen for the committed
        // scenarios, but tolerated) fall back to a default resolved passenger vType rather than
        // failing the export.
        var vehicleTypeById = demand.Vehicles.ToDictionary(v => v.Id, v => v.TypeId);
        var fallbackVType = VTypeDefaults.Resolve(new VType("__default__", "passenger", Sigma: null));

        var vehicles = new Dictionary<string, VehicleInfoPayload>();
        foreach (var vehicleId in trajectorySet.VehicleIds)
        {
            // External-agent naming convention (Sim.ExtDemo/CombinedFcdObserver.RenderId): an
            // agent injected OUTSIDE SUMO via the B1/B5 obstacle API is never a real vehicle in
            // this scenario's rou.xml (adding one there would make the ENGINE itself simulate it
            // as a normal car -- exactly what it must not be), so the usual rou.xml join below
            // can never resolve it. Recognize it by its "ext_pedestrian_"/"ext_car_" id prefix
            // instead and assign fixed dimensions/vClass directly -- the vClass string doubles as
            // the palette key template.js's VCLASS_COLORS.ext_pedestrian/ext_car resolve against.
            if (vehicleId.StartsWith("ext_pedestrian_", StringComparison.Ordinal))
            {
                vehicles[vehicleId] = new VehicleInfoPayload("ext_pedestrian", "ext_pedestrian", 0.5, 0.5);
                continue;
            }

            if (vehicleId.StartsWith("ext_car_", StringComparison.Ordinal))
            {
                vehicles[vehicleId] = new VehicleInfoPayload("ext_car", "ext_car", 4.5, 1.9);
                continue;
            }

            ResolvedVType resolved;
            string typeId;
            if (vehicleTypeById.TryGetValue(vehicleId, out var t) && demand.VTypesById.TryGetValue(t, out var vType))
            {
                typeId = t;
                resolved = VTypeDefaults.Resolve(vType);
            }
            else
            {
                typeId = vehicleTypeById.GetValueOrDefault(vehicleId, "passenger0");
                resolved = fallbackVType;
            }

            vehicles[vehicleId] = new VehicleInfoPayload(typeId, resolved.VClass, resolved.Length, resolved.Width);
        }

        // Trajectory: one entry per distinct FCD timestep, each carrying every vehicle present
        // at that step (VIZ_SPEC.md "Real-time clock" -- the front end interpolates between
        // bracketing steps and stops drawing a vehicle absent from the next one).
        var byTime = new SortedDictionary<double, Dictionary<string, VehicleStatePayload>>();
        foreach (var point in trajectorySet.AllPoints)
        {
            if (!byTime.TryGetValue(point.Time, out var atTime))
            {
                atTime = new Dictionary<string, VehicleStatePayload>();
                byTime[point.Time] = atTime;
            }

            atTime[point.VehicleId] = new VehicleStatePayload(point.X, point.Y, point.Angle, point.Speed);
        }

        var trajectory = byTime.Select(kv => new StepPayload(kv.Key, kv.Value)).ToArray();
        var simStart = trajectory.Length > 0 ? trajectory[0].T : config?.Begin ?? 0.0;
        var simEnd = trajectory.Length > 0 ? trajectory[^1].T : config?.End ?? 0.0;
        var stepSize = trajectory.Length > 1 ? trajectory[1].T - trajectory[0].T : config?.StepLength ?? 1.0;

        var bbox = new BboxPayload(minX, minY, maxX, maxY);

        return new ReplayData(scenarioName, networkPayload, vehicles, trajectory, simStart, simEnd, stepSize, bbox);
    }

    private sealed record LanePayload(string Id, string EdgeId, int Index, double Width, double[] Shape);

    private sealed record JunctionPayload(string Id, double[] Shape);

    private sealed record TlPhasePayload(double Duration, string State);

    private sealed record TlLogicPayload(string Id, double Offset, TlPhasePayload[] Phases);

    private sealed record SignalHeadPayload(string Tl, int LinkIndex, double X, double Y, double Angle);

    private sealed record NetworkPayload(
        LanePayload[] Lanes,
        JunctionPayload[] Junctions,
        TlLogicPayload[] Tls,
        SignalHeadPayload[] Signals);

    private sealed record VehicleInfoPayload(string Type, string VClass, double Length, double Width);

    private sealed record VehicleStatePayload(double X, double Y, double Angle, double Speed);

    private sealed record StepPayload(double T, Dictionary<string, VehicleStatePayload> V);

    private sealed record BboxPayload(double MinX, double MinY, double MaxX, double MaxY);

    private sealed record ReplayData(
        string Scenario,
        NetworkPayload Network,
        Dictionary<string, VehicleInfoPayload> Vehicles,
        StepPayload[] Trajectory,
        double SimStart,
        double SimEnd,
        double StepSize,
        BboxPayload Bbox);
}
