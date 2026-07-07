using System.Text.Json;
using System.Text.Json.Serialization;

namespace Sim.Harness;

public sealed class ToleranceConfig
{
    public static readonly IReadOnlyList<string> DefaultComparedAttributes = new[] { "lane", "pos", "speed" };

    public required ParityMode ParityMode { get; init; }
    public required IReadOnlyList<string> ComparedAttributes { get; init; }
    public double? Pos { get; init; }
    public double? Speed { get; init; }
    public double? X { get; init; }
    public double? Y { get; init; }
    public double? Angle { get; init; }
    public double? Acceleration { get; init; }

    public double ToleranceFor(string attribute) => attribute switch
    {
        "lane" => 0.0,
        "pos" => Pos ?? throw MissingTolerance(attribute),
        "speed" => Speed ?? throw MissingTolerance(attribute),
        "x" => X ?? throw MissingTolerance(attribute),
        "y" => Y ?? throw MissingTolerance(attribute),
        "angle" => Angle ?? throw MissingTolerance(attribute),
        "acceleration" => Acceleration ?? throw MissingTolerance(attribute),
        _ => throw new ArgumentException($"Unknown comparison attribute '{attribute}'.", nameof(attribute)),
    };

    private static InvalidOperationException MissingTolerance(string attribute) =>
        new($"tolerance.json does not define a tolerance for compared attribute '{attribute}'.");

    public static ToleranceConfig Load(string path)
    {
        using var stream = File.OpenRead(path);
        return LoadFrom(stream, path);
    }

    public static ToleranceConfig Parse(string json) =>
        LoadFrom(new MemoryStream(System.Text.Encoding.UTF8.GetBytes(json)), sourceName: "<inline>");

    private static ToleranceConfig LoadFrom(Stream stream, string sourceName)
    {
        var dto = JsonSerializer.Deserialize<ToleranceConfigDto>(stream, JsonOptions)
                  ?? throw new InvalidDataException($"tolerance config is empty: {sourceName}");

        var parityMode = dto.ParityMode.ToLowerInvariant() switch
        {
            "exact" => ParityMode.Exact,
            "statistical" => ParityMode.Statistical,
            _ => throw new InvalidDataException(
                $"tolerance config '{sourceName}' has unknown parityMode '{dto.ParityMode}' (expected 'exact' or 'statistical')."),
        };

        return new ToleranceConfig
        {
            ParityMode = parityMode,
            ComparedAttributes = dto.ComparedAttributes ?? DefaultComparedAttributes,
            Pos = dto.Pos,
            Speed = dto.Speed,
            X = dto.X,
            Y = dto.Y,
            Angle = dto.Angle,
            Acceleration = dto.Acceleration,
        };
    }

    private static readonly JsonSerializerOptions JsonOptions = new()
    {
        PropertyNameCaseInsensitive = true,
    };

    private sealed class ToleranceConfigDto
    {
        [JsonPropertyName("parityMode")]
        public string ParityMode { get; set; } = "exact";

        [JsonPropertyName("comparedAttributes")]
        public List<string>? ComparedAttributes { get; set; }

        public double? Pos { get; set; }
        public double? Speed { get; set; }
        public double? X { get; set; }
        public double? Y { get; set; }
        public double? Angle { get; set; }
        public double? Acceleration { get; set; }
    }
}
