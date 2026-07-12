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

    // Phase 2 (sublane): tolerance for the continuous lateral offset (SUMO's FCD `posLat`).
    // Only required by scenarios that list "posLat" in comparedAttributes; phase-1 goldens omit it.
    public double? PosLat { get; init; }

    /// <summary>
    /// Per-attribute ensemble (mean, std) tolerances for <c>parityMode="statistical"</c> configs
    /// (see <see cref="Harness.TrajectoryComparator.CompareEnsemble"/>). Absent/null for
    /// <c>parityMode="exact"</c> configs — those never populate or read this.
    /// </summary>
    public IReadOnlyDictionary<string, StatisticalAttributeTolerance>? Statistical { get; init; }

    public double ToleranceFor(string attribute) => attribute switch
    {
        "lane" => 0.0,
        "pos" => Pos ?? throw MissingTolerance(attribute),
        "speed" => Speed ?? throw MissingTolerance(attribute),
        "posLat" => PosLat ?? throw MissingTolerance(attribute),
        "x" => X ?? throw MissingTolerance(attribute),
        "y" => Y ?? throw MissingTolerance(attribute),
        "angle" => Angle ?? throw MissingTolerance(attribute),
        "acceleration" => Acceleration ?? throw MissingTolerance(attribute),
        _ => throw new ArgumentException($"Unknown comparison attribute '{attribute}'.", nameof(attribute)),
    };

    /// <summary>Ensemble-mean tolerance for <paramref name="attribute"/> (statistical parity mode).</summary>
    public double MeanToleranceFor(string attribute) => StatisticalToleranceFor(attribute).Mean;

    /// <summary>Ensemble-standard-deviation tolerance for <paramref name="attribute"/> (statistical parity mode).</summary>
    public double StdToleranceFor(string attribute) => StatisticalToleranceFor(attribute).Std;

    private StatisticalAttributeTolerance StatisticalToleranceFor(string attribute)
    {
        if (Statistical is null || !Statistical.TryGetValue(attribute, out var found))
            throw new InvalidOperationException(
                $"tolerance.json does not define a statistical tolerance for compared attribute '{attribute}'.");
        return found;
    }

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
            PosLat = dto.PosLat,
            X = dto.X,
            Y = dto.Y,
            Angle = dto.Angle,
            Acceleration = dto.Acceleration,
            Statistical = dto.Statistical?.ToDictionary(
                kvp => kvp.Key,
                kvp => new StatisticalAttributeTolerance(kvp.Value.Mean, kvp.Value.Std)),
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
        public double? PosLat { get; set; }
        public double? X { get; set; }
        public double? Y { get; set; }
        public double? Angle { get; set; }
        public double? Acceleration { get; set; }

        [JsonPropertyName("statistical")]
        public Dictionary<string, StatisticalAttributeToleranceDto>? Statistical { get; set; }
    }

    private sealed class StatisticalAttributeToleranceDto
    {
        public double Mean { get; set; }
        public double Std { get; set; }
    }
}

/// <summary>
/// Ensemble mean/std tolerance pair for one attribute under <c>parityMode="statistical"</c>.
/// JSON shape (nested under a top-level <c>"statistical"</c> object keyed by attribute name):
/// <code>
/// "statistical": {
///   "speed": { "mean": 0.5, "std": 0.5 },
///   "pos":   { "mean": 5.0, "std": 5.0 }
/// }
/// </code>
/// </summary>
public sealed record StatisticalAttributeTolerance(double Mean, double Std);
