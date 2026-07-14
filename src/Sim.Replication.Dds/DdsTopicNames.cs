namespace Sim.Replication.Dds;

// Explicit topic-name constants. CRITICAL: the [DdsTopic] attribute on these structs does NOT carry a
// usable name through CycloneDDS.NET 0.3.2's generator — the 1-arg `DdsWriter<T>(participant)` /
// `DdsReader<T>(participant)` ctor resolves to a NULL topic name and throws ArgumentNullException. Every
// writer/reader in this track MUST be constructed with an explicit topic-name string (the 2-arg ctor,
// `new DdsWriter<T>(participant, DdsTopicNames.X)`), and a writer/reader pair must use the SAME string to
// match. See README.md.
public static class DdsTopicNames
{
    public const string Vehicles = "sumo/vehicles";
    public const string Lifecycle = "sumo/lifecycle";
    public const string Geometry = "sumo/geometry";
    public const string Tl = "sumo/tl";
    // Reverse channel: a view-only remote publishes commands here; the publisher subscribes and applies them
    // to its EngineHost (pause/speed/restart/inject/...). See DdsViewerCommand + Sim.Viewer.Core command link.
    public const string Commands = "sumo/commands";
    // Forward status: the publisher publishes its engine state (mode/paused/speed/step/time/count) here so a
    // remote can reflect real state and disable inapplicable controls. See DdsViewerStatus.
    public const string Status = "sumo/status";
}
