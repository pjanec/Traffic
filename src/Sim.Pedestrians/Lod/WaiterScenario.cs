using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// LIVE-POC-3 (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §7, §12): a MICRO-SCENARIO template -- "the waiter
// serves the outdoor tables" -- bound to a (building, table-cluster) anchor. Unlike PedDemand's
// probability-driven schedule insertion (§4) or SocialPlanner's pairwise agreement (§5), a
// micro-scenario is authored directly from a small, fixed config: a looping
// Walk(door->table) -> Dwell(serve) -> Walk(table->door) -> Dwell(inside) schedule, repeated `Loops`
// times, visiting the tables in a SEED-VARIED (not random) order. The result is STILL an ordinary
// ActivityTimeline -- WaiterScenario is a pure generator, not a new evaluator or a new segment kind --
// so the waiter is exactly as low-power and server==IG-reconstructable as any other ActivityTimeline
// ped (docs §1: "liveliness does not add a per-step behavior loop; it adds richer one-time data").
//
// The hidden Dwell at the door (Visible:false) is §6's "inside the building" state: PoseAt still
// returns a pose there (so re-emergence has somewhere to resume from) but the caller must render no
// disc while the waiter is inside -- SceneGen.BuildWaiter and the tests both check for this window.
//
// No System.Random anywhere: the table visitation order is a CLOSED-FORM rotation keyed by `Seed` (see
// TableIndexForLoop), so calling Build twice with the same config always returns a bit-identical
// timeline (the same determinism discipline as SocialPlanner.ScheduleInteraction).
public readonly record struct WaiterScenarioConfig(
    Vec2 DoorPos,
    IReadOnlyList<Vec2> Tables,
    double StartTime,
    double Speed,
    double ServeSeconds,
    double InsideSeconds,
    int Loops,
    ulong Seed);

public static class WaiterScenario
{
    // Animation tags (docs §3 tag vocabulary): "serve" is the LIVE-POC-3 addition alongside
    // "sip"/"sit"/"enter"/"talk" from LIVE-POC-1/2. "inside" reuses the §6 hidden-dwell idea but names
    // it distinctly from LIVE-POC-1's "enter" tag since the waiter's inside dwell is a REPEATED
    // between-rounds state, not a one-shot "entered a building" beat.
    public const string ServeAnimTag = "serve";
    public const string InsideAnimTag = "inside";

    // Builds the waiter's whole looping schedule up front (docs §7): a hidden inside-dwell at the
    // door, then `Loops` repetitions of Walk(door->table) -> Dwell(serve) -> Walk(table->door) ->
    // Dwell(inside), each loop's table chosen by TableIndexForLoop. Every Walk's authored path starts
    // exactly where the previous segment left the waiter (door or table), so the chain is continuous
    // by construction -- the same authoring discipline ActivityTimelineTests documents for
    // Walk->Dwell->Walk chains.
    public static ActivityTimeline Build(WaiterScenarioConfig cfg)
    {
        if (cfg.Tables.Count == 0)
        {
            throw new ArgumentException("WaiterScenarioConfig.Tables must contain at least one table.", nameof(cfg));
        }

        if (cfg.Loops <= 0)
        {
            throw new ArgumentException("WaiterScenarioConfig.Loops must be positive.", nameof(cfg));
        }

        var segments = new List<ActivitySegment>(1 + (cfg.Loops * 4));

        // Waiter starts inside (§6 hidden dwell) -- the door pose is the anchor the very first Walk
        // departs from. Heading is irrelevant while hidden (no disc is ever drawn for this segment),
        // so Vec2.Zero is the harmless, deterministic choice (mirrors ActivityTimeline's own Idle-clamp
        // convention of Vec2.Zero for a "facing doesn't matter here" heading).
        segments.Add(new DwellSegment(cfg.DoorPos, Vec2.Zero, cfg.InsideSeconds, InsideAnimTag, Visible: false));

        for (var loop = 0; loop < cfg.Loops; loop++)
        {
            var tableIndex = TableIndexForLoop(loop, cfg.Tables.Count, cfg.Seed);
            var table = cfg.Tables[tableIndex];
            var facing = Facing(cfg.DoorPos, table);

            segments.Add(new WalkSegment(new[] { cfg.DoorPos, table }, cfg.Speed));
            segments.Add(new DwellSegment(table, facing, cfg.ServeSeconds, ServeAnimTag, Visible: true));
            segments.Add(new WalkSegment(new[] { table, cfg.DoorPos }, cfg.Speed));
            segments.Add(new DwellSegment(cfg.DoorPos, Vec2.Zero, cfg.InsideSeconds, InsideAnimTag, Visible: false));
        }

        return new ActivityTimeline(cfg.StartTime, segments);
    }

    // The table visited on a given (0-based) loop iteration: a closed-form ROTATION over the table
    // list, offset by `seed`. `offset = seed % tableCount` picks WHICH table the rotation starts on
    // (the "seeded variation" the design calls for -- two different seeds serve the tables in a
    // different starting order); `(loop + offset) % tableCount` is then a bijection on
    // [0, tableCount) with period `tableCount`, so every table is served exactly once per
    // `tableCount` consecutive loops -- the property the "serves every table" success condition
    // relies on (docs §12: "respects table capacity" -- no table is ever skipped, none is ever
    // double-booked within one rotation). A step-multiplied shuffle (e.g. `loop*step + seed`) was
    // considered but rejected: an arbitrary step sharing a common factor with tableCount can skip
    // tables entirely, which would silently break full-cluster coverage.
    private static int TableIndexForLoop(int loop, int tableCount, ulong seed)
    {
        var offset = (int)(seed % (ulong)tableCount);
        return (loop + offset) % tableCount;
    }

    // The pose the waiter faces while serving a table: the direction of travel FROM the door TO the
    // table (i.e. still facing "into" the table cluster on arrival), a deterministic closed-form
    // choice from the two POI positions alone. Falls back to Vec2.Zero for the degenerate door==table
    // case (zero-length approach).
    private static Vec2 Facing(Vec2 doorPos, Vec2 tablePos)
    {
        var d = tablePos - doorPos;
        return d.Abs > 1e-12 ? d.Normalized() : Vec2.Zero;
    }
}
