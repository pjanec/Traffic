using Sim.Core.Orca;

namespace Sim.Pedestrians.Lod;

// P1-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §5, use cases 1-4): promotes POC-3's
// single caller-owned `IReadOnlyList<InterestSource>` into a first-class, MANAGED, multi-source
// interest field. POC-3's PedLodManager.Step took a bare list and full-double-scanned it against
// every ped every step (O(peds * sources)); with dozens of independently-moving sources (an avatar
// bubble, an IG camera frustum, several static AoIs, a few intrinsic crosswalk/incident sources all
// live at once) that full scan is wasted work -- a given ped is only ever near a handful of sources.
//
// What this type adds over a raw list:
//   1. STABLE IDENTITY: Register/Move/Remove key off an opaque InterestSourceId, so a caller
//      juggling several independently-moving sources (use case 4) never has to reindex "the other
//      N-1 sources" just because one moved or a new one showed up -- exactly the same
//      add/move/remove-by-handle shape OrcaCrowd and PedRouteController already use for pedestrians
//      themselves (P0-1/P0-2/P0-3), applied here to sources instead of agents.
//   2. BOUNDED EVALUATION: sources are bucketed into a uniform grid so a ped queries only the
//      sources that could plausibly reach it, not the whole set. See RebuildIndex/Query remarks
//      below for the exact scheme and why it stays correct as sources move independently.
//
// Kind is pure metadata (telemetry/caller bookkeeping) -- every source, regardless of Kind,
// promotes/demotes via the exact same PromoteRadius/DemoteRadius test (docs §5: "a ped is
// high-power iff it lies within the promotion volume of ANY active source").
public enum InterestSourceKind
{
    EntityAttached, // an avatar / external car / disembarked ped carrying its own bubble (use case 1)
    Camera,         // an IG camera frustum/region (use case 3)
    StaticAoI,      // a designated, scripted area of interest known up front (use case 2)
    Intrinsic,      // a crosswalk/parking/incident source independent of any camera or avatar (use case 2/4)
}

// Opaque handle returned by InterestField.Register. Two ids are equal iff they name the same
// registration -- a Remove-d id is never reissued (the internal counter only increments), so a
// stale id from a removed source can never silently alias a later, unrelated one.
public readonly record struct InterestSourceId(long Value)
{
    public static readonly InterestSourceId Invalid = new(0);

    public bool IsValid => Value != 0;
}

// The result of a single Query: both booleans PedLodManager.Step needs, computed together off one
// grid-cell lookup so a ped that must be tested for BOTH "any promote" (while low-power) and "all
// outside demote" (while high-power) never pays for two separate scans.
public readonly record struct InterestQueryResult(bool AnyWithinPromote, bool AllOutsideDemote);

// The movable, multi-source interest field. One instance is long-lived for the whole population
// (mirrors PedLodManager's own persistent-for-the-manager's-lifetime convention, P0-3) -- sources
// are Registered/Moved/Removed across steps, never rebuilt wholesale by the caller.
public sealed class InterestField
{
    private sealed class Entry
    {
        public required InterestSource Source;
        public required InterestSourceKind Kind;
    }

    private readonly Dictionary<InterestSourceId, Entry> _sources = new();
    private long _nextId = 1;

    // ---- Spatial index (rebuilt once per PedLodManager.Step -- see RebuildIndex remarks) ----
    private readonly Dictionary<long, List<InterestSource>> _cellToBucket = new();
    private double _cellSize = 1.0;

    public int Count => _sources.Count;

    // ---- Diagnostics (not used by PedLodManager itself; for tests/telemetry proving the cost model,
    // P1-1 success condition "adding sources does not blow up the scan") ----

    // Total (source, cell) insertions performed by the most recent RebuildIndex -- scales with
    // sum-of-cells-per-source, NOT with ped count (RebuildIndex never looks at peds at all).
    public int LastRebuildInsertionCount { get; private set; }

    // Number of candidate sources examined by the most recent Query call -- i.e. the size of the one
    // bucket that ped's position landed in. For spread-out sources this stays small and roughly
    // constant as more (spread-out) sources are registered, since a given ped's cell only ever
    // contains the handful of sources whose reach overlaps that specific cell.
    public int LastQueryCandidateCount { get; private set; }

    // Registers a new source under a fresh, never-reused id. `source.Position` is read live on every
    // RebuildIndex -- Move (or direct Position mutation between steps, see Move's remarks) is how a
    // caller relocates it.
    public InterestSourceId Register(InterestSource source, InterestSourceKind kind = InterestSourceKind.StaticAoI)
    {
        if (source is null)
        {
            throw new ArgumentNullException(nameof(source));
        }

        var id = new InterestSourceId(_nextId++);
        _sources.Add(id, new Entry { Source = source, Kind = kind });
        return id;
    }

    // Relocates an already-registered source in place (use cases 1/3/4: an avatar/camera/incident
    // moving independently every step). Equivalent to mutating `Source.Position` directly -- provided
    // as a named, discoverable verb for callers who don't want to hold onto the InterestSource
    // reference, and to make "this is how sources move" unambiguous at call sites.
    public void Move(InterestSourceId id, Vec2 newPosition) => _sources[id].Source.Position = newPosition;

    public bool Remove(InterestSourceId id) => _sources.Remove(id);

    public bool Contains(InterestSourceId id) => _sources.ContainsKey(id);

    public InterestSource SourceOf(InterestSourceId id) => _sources[id].Source;

    public InterestSourceKind KindOf(InterestSourceId id) => _sources[id].Kind;

    // A deterministic (ascending-id) snapshot of every registered source, for callers that need to
    // enumerate the whole set directly (telemetry, or a brute-force reference oracle in tests --
    // PedLodManager itself never enumerates this; it only calls Query per ped).
    public IReadOnlyList<(InterestSourceId Id, InterestSource Source, InterestSourceKind Kind)> Snapshot()
    {
        var ids = new List<InterestSourceId>(_sources.Keys);
        ids.Sort((a, b) => a.Value.CompareTo(b.Value));

        var result = new List<(InterestSourceId, InterestSource, InterestSourceKind)>(ids.Count);
        foreach (var id in ids)
        {
            var e = _sources[id];
            result.Add((id, e.Source, e.Kind));
        }

        return result;
    }

    // Rebuilds the uniform grid from CURRENT source positions/radii. Called ONCE per
    // PedLodManager.Step, before that step's per-ped scan -- exactly like OrcaCrowd.RebuildGrid is
    // called once per OrcaCrowd.Step (Sim.Core.Orca.OrcaCrowd) -- so every ped queried during the same
    // step sees the same frozen, start-of-step source positions (docs §5: "Promotion is a pure
    // function of frozen state (source positions are start-of-step)"). Cheap: O(sources *
    // cells-per-source), and sources are FEW (design: "typically tens, not thousands"), so this is
    // negligible next to the O(peds) scan it enables -- it does not grow with the ped population.
    //
    // Indexing scheme (loose/bucket grid, not the 3x3-neighbourhood scheme OrcaCrowd uses for
    // same-radius crowd agents): interest sources can have wildly different DemoteRadius values (a
    // 2 m static AoI next to a 200 m camera frustum), so a fixed "cell == neighbour distance, query
    // the 3x3 block" scheme would either miss large sources (cell too small) or force every ped to
    // scan a huge neighbourhood (cell too big). Instead:
    //   - cell size is derived each rebuild from the LARGEST current DemoteRadius (2x it, floor 1.0),
    //     so no single source ever needs more than a small, bounded number of cells for itself.
    //   - each source is inserted into EVERY cell its DemoteRadius bounding box overlaps (not just its
    //     own cell) -- a source with a big radius occupies more cells, proportional to its OWN size,
    //     never to the ped count.
    //   - a ped queries ONLY its own single cell (no neighbour search needed): because floor() is
    //     monotonic, any position p with |p - source.Position| <= source.DemoteRadius necessarily has
    //     p's cell within the source's bounding-box cell range, so single-cell lookup cannot miss a
    //     true positive. (DemoteRadius > PromoteRadius always, so this same lookup also covers every
    //     promote-radius test.)
    //   - net effect: adding more sources costs each ped nothing unless that ped's OWN cell now holds
    //     more sources -- i.e. cost tracks local source density, not total source count. Spread-out
    //     sources (the design's stated shape: "several active interest sources at once, each moving
    //     independently") therefore do not multiply the per-ped query cost as more are added.
    public void RebuildIndex()
    {
        _cellToBucket.Clear();
        var insertions = 0;

        var ids = new List<InterestSourceId>(_sources.Keys);
        ids.Sort((a, b) => a.Value.CompareTo(b.Value)); // deterministic bucket-fill order (P1-1: fixed iteration order)

        var maxDemote = 0.0;
        foreach (var id in ids)
        {
            var r = _sources[id].Source.DemoteRadius;
            if (r > maxDemote)
            {
                maxDemote = r;
            }
        }

        _cellSize = maxDemote > 0.0 ? maxDemote * 2.0 : 1.0;

        foreach (var id in ids)
        {
            var src = _sources[id].Source;
            var r = src.DemoteRadius;

            var minCx = FloorDiv(src.Position.X - r, _cellSize);
            var maxCx = FloorDiv(src.Position.X + r, _cellSize);
            var minCy = FloorDiv(src.Position.Y - r, _cellSize);
            var maxCy = FloorDiv(src.Position.Y + r, _cellSize);

            for (var cx = minCx; cx <= maxCx; cx++)
            {
                for (var cy = minCy; cy <= maxCy; cy++)
                {
                    var key = PackCell(cx, cy);
                    if (!_cellToBucket.TryGetValue(key, out var bucket))
                    {
                        bucket = new List<InterestSource>();
                        _cellToBucket[key] = bucket;
                    }

                    bucket.Add(src);
                    insertions++;
                }
            }
        }

        LastRebuildInsertionCount = insertions;
    }

    // Tests `pos` against only the sources bucketed into pos's own cell (see RebuildIndex remarks for
    // why that is sufficient, not just fast). Must be called after RebuildIndex has been run for this
    // step; PedLodManager.Step calls RebuildIndex exactly once at the top of the step and then Query
    // once per ped, so every ped in the same step is tested against the same frozen index.
    public InterestQueryResult Query(Vec2 pos)
    {
        var key = PackCell(FloorDiv(pos.X, _cellSize), FloorDiv(pos.Y, _cellSize));

        var anyPromote = false;
        var allOutsideDemote = true;

        if (_cellToBucket.TryGetValue(key, out var bucket))
        {
            LastQueryCandidateCount = bucket.Count;
            for (var i = 0; i < bucket.Count; i++)
            {
                var src = bucket[i];
                var distSq = (pos - src.Position).AbsSq;

                if (distSq <= src.DemoteRadius * src.DemoteRadius)
                {
                    allOutsideDemote = false;
                    if (distSq <= src.PromoteRadius * src.PromoteRadius)
                    {
                        anyPromote = true;
                    }
                }
            }
        }
        else
        {
            LastQueryCandidateCount = 0;
        }

        return new InterestQueryResult(anyPromote, allOutsideDemote);
    }

    private static int FloorDiv(double v, double cell) => (int)Math.Floor(v / cell);

    private static long PackCell(int cx, int cy) => ((long)cx << 32) | (uint)cy;
}
