using System.Globalization;
using Sim.Core;
using Sim.ExtDemo;
using Sim.Ingest;

// EXTERNAL-AGENT visual-test demo: injects external (non-SUMO) pedestrians/cars into the engine
// via the existing B1/B5 obstacle API (IEngine.AddObstacle / AddMovingObstacle -- see
// src/Sim.Core/IEngine.cs), runs the engine, and emits a COMBINED FCD (SUMO vehicles + external
// agents, both as <vehicle> rows) via CombinedFcdObserver so Sim.Viz can replay both together.
//
// This tool does not touch Sim.Core -- it is a caller of the existing obstacle API, exactly like
// RungB1ExternalObstacleTests / RungB5MovingObstacleTests, just wired to a scenario dir + a JSON
// script instead of hardcoded test values. Mirrors src/Sim.Run/Program.cs's shape.
//
// Usage:
//   dotnet run --project src/Sim.ExtDemo -- <scenarioDir> [--agents PATH] [--fcd-out PATH] [--steps N]
//
// Defaults: agents = <scenarioDir>/external-agents.json (missing file => zero agents, which is
// exactly the "WITHOUT" half of the behavioral-proof pair -- see the scenario's NOTES.md);
// fcd-out = <scenarioDir>/engine.fcd.xml; steps = round((end-begin)/step-length) from the
// scenario's *.sumocfg (matches Sim.Run's own default).
internal static class Program
{
    private static int Main(string[] args)
    {
        if (args.Length == 0 || args[0] is "-h" or "--help")
        {
            Console.Error.WriteLine(
                "usage: Sim.ExtDemo <scenarioDir> [--agents PATH] [--fcd-out PATH] [--steps N]");
            return args.Length == 0 ? 2 : 0;
        }

        var scenarioDir = args[0];
        if (!Directory.Exists(scenarioDir))
        {
            Console.Error.WriteLine($"error: scenario directory not found: {scenarioDir}");
            return 2;
        }

        string? agentsPathOverride = null;
        string? fcdOut = null;
        int? stepsOverride = null;
        for (var i = 1; i < args.Length; i++)
        {
            switch (args[i])
            {
                case "--agents" when i + 1 < args.Length:
                    agentsPathOverride = args[++i];
                    break;
                case "--fcd-out" when i + 1 < args.Length:
                    fcdOut = args[++i];
                    break;
                case "--steps" when i + 1 < args.Length:
                    stepsOverride = int.Parse(args[++i], CultureInfo.InvariantCulture);
                    break;
                default:
                    Console.Error.WriteLine($"error: unrecognized argument: {args[i]}");
                    return 2;
            }
        }

        var net = SingleFile(scenarioDir, "*.net.xml");
        var rou = SingleFile(scenarioDir, "*.rou.xml");
        var cfg = SingleFile(scenarioDir, "*.sumocfg");
        if (net is null || rou is null || cfg is null)
        {
            Console.Error.WriteLine(
                $"error: scenario dir must contain exactly one each of *.net.xml, *.rou.xml, " +
                $"*.sumocfg (found net={net}, rou={rou}, cfg={cfg})");
            return 2;
        }

        var agentsPath = agentsPathOverride ?? Path.Combine(scenarioDir, "external-agents.json");
        var agents = ExternalAgentsReader.Read(agentsPath);

        var network = NetworkParser.Parse(net);
        var config = ScenarioConfigParser.Parse(cfg);
        var steps = stepsOverride ?? (int)Math.Round((config.End - config.Begin) / config.StepLength);
        fcdOut ??= Path.Combine(scenarioDir, "engine.fcd.xml");

        var engine = new Engine();
        engine.LoadScenario(net, rou, cfg);

        foreach (var agent in agents)
        {
            // One obstacle per EngineLaneIds entry (LaneId plus any blockLaneIds) -- a crossing
            // pedestrian spanning several lanes registers on every one of them so a SUMO car
            // cannot simply lane-change around a footprint the engine has no lateral field to
            // represent otherwise (see ExternalAgent.cs's blockLaneIds doc comment).
            foreach (var laneId in agent.EngineLaneIds)
            {
                if (!network.LanesById.ContainsKey(laneId))
                {
                    Console.Error.WriteLine($"error: agent '{agent.Id}' references unknown laneId '{laneId}'.");
                    return 2;
                }

                // Handle-based obstacle API (§4.4): resolve the lane once, discard the returned handle
                // (this demo adds obstacles up front and never updates/removes them by id).
                var laneHandle = engine.GetLane(laneId);
                // B6: latPos/width/latSpeed give the agent a LATERAL footprint the engine reacts
                // to -- a car swerves within its lane around a partial-width agent, spills to the
                // next lane if it fills this one, and dodges a lunging (latSpeed!=0) agent
                // predictively. width=0 keeps the pre-B6 full-lane block (car stops dead).
                if (agent.IsPedestrian)
                {
                    engine.AddObstacle(laneHandle, agent.StartPos, agent.Length,
                        agent.StartTime, agent.EndTime, agent.LatPos, agent.Width, agent.LatSpeed);
                }
                else
                {
                    engine.AddMovingObstacle(
                        laneHandle, agent.StartPos, agent.Length,
                        agent.Speed, agent.MaxDecel ?? ExternalAgentDef.DefaultMaxDecel,
                        agent.StartTime, agent.EndTime, agent.LatPos, agent.Width, agent.LatSpeed);
                }
            }
        }

        using (var writer = new CombinedFcdObserver(fcdOut, network, agents))
        {
            engine.AddExportObserver(writer);
            engine.Run(steps);
        }

        Console.WriteLine(
            $"wrote {fcdOut}  ({steps} steps, [{config.Begin}, {config.End}] @ {config.StepLength}s, " +
            $"agents={agents.Count} from {(agents.Count > 0 ? agentsPath : "(none found)")})");
        return 0;
    }

    private static string? SingleFile(string dir, string pattern)
    {
        var matches = Directory.GetFiles(dir, pattern);
        return matches.Length == 1 ? matches[0] : null;
    }
}
