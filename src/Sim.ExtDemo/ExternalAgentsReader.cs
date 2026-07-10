using System.Text.Json;
using System.Text.Json.Serialization;

namespace Sim.ExtDemo;

// Reads external-agents.json (see ExternalAgent.cs's schema doc). A missing file is treated as
// "no external agents" (empty list) rather than an error -- this is what lets the SAME demo
// binary produce the WITH/WITHOUT behavioral-proof pair (VERIFY section of the briefing) just by
// pointing --agents at a real file vs a nonexistent one, with no separate code path.
public static class ExternalAgentsReader
{
    private static readonly JsonSerializerOptions Options = new()
    {
        PropertyNameCaseInsensitive = true,
        ReadCommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
    };

    public static IReadOnlyList<ExternalAgentDef> Read(string path)
    {
        if (!File.Exists(path))
        {
            return Array.Empty<ExternalAgentDef>();
        }

        var json = File.ReadAllText(path);
        var dto = JsonSerializer.Deserialize<FileDto>(json, Options)
            ?? throw new InvalidDataException($"'{path}' did not deserialize to an agents file.");

        var agents = new List<ExternalAgentDef>(dto.Agents.Count);
        foreach (var a in dto.Agents)
        {
            if (a.Id is null || a.Kind is null || a.LaneId is null)
            {
                throw new InvalidDataException(
                    $"'{path}': every agent needs id, kind, and laneId (got id={a.Id}, kind={a.Kind}, laneId={a.LaneId}).");
            }

            if (!string.Equals(a.Kind, "pedestrian", StringComparison.OrdinalIgnoreCase)
                && !string.Equals(a.Kind, "car", StringComparison.OrdinalIgnoreCase))
            {
                throw new InvalidDataException($"'{path}': agent '{a.Id}' has unknown kind '{a.Kind}' (expected pedestrian|car).");
            }

            agents.Add(new ExternalAgentDef(
                Id: a.Id,
                Kind: a.Kind,
                LaneId: a.LaneId,
                BlockLaneIds: a.BlockLaneIds ?? new List<string>(),
                StartPos: a.StartPos,
                Length: a.Length,
                Width: a.Width,
                StartTime: a.StartTime ?? double.NegativeInfinity,
                EndTime: a.EndTime ?? double.PositiveInfinity,
                Speed: a.Speed ?? 0.0,
                MaxDecel: a.MaxDecel,
                LatFrom: a.LatFrom ?? 0.0,
                LatTo: a.LatTo ?? 0.0));
        }

        return agents;
    }

    private sealed class FileDto
    {
        public List<AgentDto> Agents { get; set; } = new();
    }

    private sealed class AgentDto
    {
        public string? Id { get; set; }
        public string? Kind { get; set; }
        public string? LaneId { get; set; }
        public List<string>? BlockLaneIds { get; set; }
        public double StartPos { get; set; }
        public double Length { get; set; }
        public double Width { get; set; }
        public double? StartTime { get; set; }
        public double? EndTime { get; set; }
        public double? Speed { get; set; }
        public double? MaxDecel { get; set; }
        public double? LatFrom { get; set; }
        public double? LatTo { get; set; }
    }
}
