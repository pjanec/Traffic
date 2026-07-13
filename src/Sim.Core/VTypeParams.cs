namespace Sim.Core;

// SUMOSHARP-API.md §9: the input to Engine.DefineVType -- the runtime-settable subset of SUMO's vType
// attributes. Every field is an OPTIONAL override on top of the vClass default table (VTypeDefaults),
// exactly like a parsed rou.xml <vType>: a null leaves SUMO's vClass default in place, so
// `new VTypeParams()` yields the pure `VClass` defaults. Runtime-defined vTypes therefore resolve
// through the SAME VTypeDefaults.Resolve pipeline as loaded ones (parity-consistent).
//
public sealed record VTypeParams
{
    // vClass selects the default table row (passenger/truck/bus/bicycle/…). Defaults to passenger.
    public string VClass { get; init; } = "passenger";

    public double? Accel { get; init; }
    public double? Decel { get; init; }
    public double? EmergencyDecel { get; init; }
    public double? MaxSpeed { get; init; }
    public double? Tau { get; init; }
    public double? MinGap { get; init; }
    public double? Length { get; init; }

    // Driver imperfection. Null -> vClass default (0.5 for passenger). Set 0.0 for deterministic motion.
    public double? Sigma { get; init; }
    public double? SpeedFactor { get; init; }

    // "Krauss" (default), "IDM", "IDMM", "ACC", "CACC", "Rail" -- must match a supported model.
    public string? CarFollowModel { get; init; }

    public bool? HasBluelight { get; init; }
    public bool? LcOpposite { get; init; }

    // Sublane lateral attributes (folded in at the laneless merge -- issue #4). Each is an optional
    // override that resolves through the SAME VTypeDefaults.Resolve as loaded vTypes: null -> SUMO's
    // default (MaxSpeedLat 1.0 m/s, LatAlignment "center", MinGapLat 0.6 m). All three are INERT unless the
    // scenario enables the sublane model (ScenarioConfig.LateralResolution > 0), so they never affect the
    // parity/golden path when the model is off.
    public double? MaxSpeedLat { get; init; }
    // Keyword OR a numeric offset, matching SUMO: "center" (default) | "right" | "left" | "compact" |
    // "nice" | "arbitrary" | a number (metres). String (not an enum) because it must carry the numeric
    // case and stay identical to the Ingest parse type it resolves through. Only right/left/center are
    // ported today; the rest hold the centreline.
    public string? LatAlignment { get; init; }
    public double? MinGapLat { get; init; }
}
