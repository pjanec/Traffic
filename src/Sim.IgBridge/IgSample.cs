namespace Sim.IgBridge;

// The IG-native record schema (docs/IGBRIDGE-DECISIONS.md §3). The external IG accepts a lifecycle
// triple -- entity-created / entity-updated / entity-removed -- carrying, per the owner's Q1/Q3
// answers, a PLANAR pose only: (x, y, headingDeg) in SUMO metres + navi-degrees. No z / pitch / roll:
// the IG owns its terrain and does its own conformal ground-clamping. `t` is SIM time (seconds) and is
// the sole timing authority; a sample's pose is the pose AT `t`, so consecutive `Upd` for an entity
// satisfy dPos ~= speed*dt and the IG's 2-sample DR never erratically jumps.
public enum IgRecordKind : byte
{
    New = 0, // entity-created (id, model, initial pose)
    Upd = 1, // entity-updated (the per-sample pose)
    Del = 2, // entity-removed (id)
}

// Selects the IG 3D model on `New`. Extend as more entity classes are fed.
public enum IgEntityModel : byte
{
    Car = 0,
    Ped = 1,
}

// One IG-native record. A value type (no allocation per emit); the same shape is flushed to the JSONL
// trace (T1.2) and pushed onto the in-memory ring. `Model` is meaningful only on `New`; `X/Y/Z/H` are
// meaningful on `New`/`Upd` and ignored on `Del`.
//
// `Z` (docs/IGBRIDGE-DECISIONS.md §1 Q5, revised): the IG owns terrain-FOLLOWING, but on MULTI-LEVEL roads
// and tunnels (x, y) is ambiguous -- a bridge and the tunnel beneath share it -- so the IG needs the
// engine's z to place the entity on the correct deck. Sourced from lane geometry via PoseResolver's
// Pose.Z (NetworkLaneSource.LaneShapeZ): real elevation on a 3-D net, 0.0 on a flat net (the box). No
// engine-core change is needed for the direct-10 Hz vehicle path; ped z stays 0 (the crowd is 2-D) pending
// a future surface-z mapping (§ open items).
public readonly struct IgSample
{
    public IgSample(IgRecordKind kind, string id, double t, IgEntityModel model, double x, double y, double z, float headingDeg)
    {
        Kind = kind;
        Id = id;
        T = t;
        Model = model;
        X = x;
        Y = y;
        Z = z;
        HeadingDeg = headingDeg;
    }

    public IgRecordKind Kind { get; }
    public string Id { get; }
    public double T { get; }
    public IgEntityModel Model { get; }
    public double X { get; }
    public double Y { get; }
    public double Z { get; }
    public float HeadingDeg { get; }

    public static IgSample Created(string id, double t, IgEntityModel model, double x, double y, double z, float headingDeg)
        => new(IgRecordKind.New, id, t, model, x, y, z, headingDeg);

    public static IgSample Updated(string id, double t, double x, double y, double z, float headingDeg)
        => new(IgRecordKind.Upd, id, t, default, x, y, z, headingDeg);

    public static IgSample Removed(string id, double t)
        => new(IgRecordKind.Del, id, t, default, 0.0, 0.0, 0.0, 0f);
}
