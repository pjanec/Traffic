using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians.Lod;

// Sim-LOD promotion/demotion + PathArc<->FreeKinematic switching (docs/PEDESTRIAN-DESIGN.md §5, §7;
// docs/PEDESTRIAN-POC-PLAN.md POC-3). Owns a population of peds, each either:
//   - Low-power (PedDrModel.PathArc): pose is a pure function of (path, startTime, speed, now) via
//     PathArcMotion -- O(1), no neighbour query, no ORCA.
//   - High-power (PedDrModel.FreeKinematic): a real agent in a persistent high-power OrcaCrowd,
//     routed by a persistent PedRouteController + WaypointFollower exactly like POC-1a, reacting to
//     every other high-power ped AND to `externalEntities`.
//
// A ped is high-power iff its (frozen, start-of-step) position lies within ANY active
// InterestSource.PromoteRadius; it demotes once it has been continuously outside EVERY source's
// (larger) DemoteRadius for `dwellSeconds`. `dwellSeconds` ALSO gates how soon a ped may leave the
// state it just entered (both directions) -- the "minimum-dwell in each state" the design calls for,
// collapsed into one knob for this POC (a production version might separate "how long outside before
// demoting" from "minimum time before ANY transition"; see the report for this simplification).
//
// P0-3 (docs/PEDESTRIAN-TASKS.md, PEDESTRIAN-POC7C-FINDINGS.md Q2): the POC-3 version of this class
// had NO agent removal on either the crowd or the route-controller side, so every membership change
// rebuilt the ENTIRE high-power OrcaCrowd from scratch AND re-derived EVERY still-high ped's steering
// route (even peds nothing happened to) -- an O(current-high-power-count) cost per switch, measured
// at 100k as the dominant reason a churning (constantly promoting/demoting) world cost 3.6x a stable
// one. P0-1/P0-2 gave OrcaCrowd a real O(1) Add/Remove and P0-3 (this class) now uses it directly:
// `_highCrowd`/`_highController` are PERSISTENT for the lifetime of this manager -- a promotion Adds
// exactly the one newly-promoted ped and registers exactly its route; a demotion Removes exactly
// that one ped's handle and route. Every OTHER high-power ped's handle, position, velocity, route,
// AND waypoint cursor are completely untouched by someone else's promotion/demotion -- there is
// nothing left to rebuild.
//
// P1-1 (docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md §5): the POC-3 version of Step took a
// bare `IReadOnlyList<InterestSource>` and full-double-scanned it (every ped against every source,
// O(peds * sources)) with no stable identity for a caller juggling several independently-moving
// sources. Step now takes an `InterestField` (see InterestField.cs): a managed, multi-source field
// with stable per-source ids (Register/Move/Remove) and a bounded, grid-indexed per-ped query
// (RebuildIndex once per step, Query once per ped) -- same promotion/demotion semantics and hysteresis
// as POC-3, but the per-step scan no longer multiplies with the source count.
public sealed class PedLodManager
{
    private sealed class PedEntry
    {
        public required int Id;
        public required Vec2 Destination;
        public required double MaxSpeed;
        public required double Radius;

        public PedDrModel Model = PedDrModel.PathArc;

        // The polyline currently being followed: the PathArc leg's polyline when Low, the navmesh
        // steering route (set once, at promotion) when High.
        public IReadOnlyList<Vec2> Path = Array.Empty<Vec2>();
        public double PathStartTime;

        // LIVE-PROD-1a: set when this ped is a LIVELY low-power ped (Model == ActivityTimeline) -- its
        // low-power pose/velocity come from Timeline.PoseAt/VelocityAt instead of PathArcMotion. Null for
        // an ordinary PathArc ped (the whole population before liveliness is enabled), so every branch
        // that special-cases it is inert and the PathArc path stays bit-identical.
        public ActivityTimeline? Timeline;

        // P1-2 (evac panic, docs/PEDESTRIAN-DESIGN.md §6): when true this ped is PINNED high-power --
        // it promotes on the next step regardless of any InterestSource, and never demotes while pinned.
        // Default false -> the interest-field-driven promotion path is exactly as before (bit-identical).
        public bool ForcedHighPower;

        // W4 (docs/PEDESTRIAN-WEAVE-PRODUCTION-DESIGN.md): the ped's deterministic-weave seeds, kept across a
        // promote->demote so a demoted weaving ped RESUMES the weave (emits a weaving ActivityTimeline resume
        // leg) rather than a flat PathArc leg. 0 == not a weaving ped -> demote stays exactly as before.
        public ulong WeaveSeed;
        public ulong WeaveGlobalSeed;

        public OrcaHandle HighIndex = OrcaHandle.Invalid;    // handle into the persistent high-power OrcaCrowd, or Invalid when Low

        public double StateEnteredAt;             // sim time this ped entered its CURRENT LOD state
        public double OutsideSince = double.NaN;   // sim time since continuously outside every demote
                                                    // radius (High only); NaN = currently inside one
    }

    private readonly IPedNavigation _navigation;
    private readonly PedPublisher _publisher;
    private readonly ILocalSteering _steering;
    private readonly double _arriveRadius;
    private readonly double _dwellSeconds;

    private readonly Dictionary<int, PedEntry> _peds = new();

    // Persistent for the manager's whole lifetime (P0-3) -- see class remarks. Never replaced.
    private readonly OrcaCrowd _highCrowd;
    private readonly PedRouteController _highController;
    private bool _useParallelHighCrowd;
    private bool _useRegionDecompHighCrowd;

    // Live high-power ped count. NOT the same as `_highCrowd.Count` any more: OrcaCrowd.Count is a
    // high-water mark of slots ever allocated (P0-1), so it stays at its peak even after every
    // currently-high ped demotes, whereas this is decremented on every demotion -- the accurate
    // "is anyone currently high-power" signal Step() and HighPowerCount both need.
    private int _highPowerLiveCount;

    public bool UseParallelHighCrowd
    {
        get => _useParallelHighCrowd;
        set
        {
            _useParallelHighCrowd = value;
            _highCrowd.UseParallelStep = value;
        }
    }

    // P6-2 (docs/PEDESTRIAN-P6-2-REGION-DESIGN.md): opt in the high-power crowd to spatial region
    // decomposition -- the cache-local parallel plan that raises ped per-core throughput (the combined-load
    // GO). Bit-identical to serial (OrcaRegionDecompositionTests); default off, so the manager's behaviour is
    // unchanged unless a caller enables it. Takes precedence over UseParallelHighCrowd on the underlying crowd.
    public bool UseRegionDecompositionHighCrowd
    {
        get => _useRegionDecompHighCrowd;
        set
        {
            _useRegionDecompHighCrowd = value;
            _highCrowd.UseRegionDecomposition = value;
        }
    }

    // P6-2-4 tuning passthrough: region cell side = this multiple of NeighbourDist on the high-power crowd.
    public double HighCrowdRegionCellSizeMultiplier
    {
        get => _highCrowd.RegionCellSizeMultiplier;
        set => _highCrowd.RegionCellSizeMultiplier = value;
    }

    public PedLodManager(
        IPedNavigation navigation,
        PedPublisher publisher,
        double arriveRadius = 0.3,
        double dwellSeconds = 1.0,
        ILocalSteering? steering = null)
    {
        _navigation = navigation;
        _publisher = publisher;
        _arriveRadius = arriveRadius;
        _dwellSeconds = dwellSeconds;
        _steering = steering ?? new WaypointFollower();

        // P0-4 (docs/PEDESTRIAN-POC7C-FINDINGS.md follow-up hypothesis; docs/PEDESTRIAN-DESIGN.md §9):
        // the persistent high-power crowd was constructed bare (UseSpatialHash defaults to false), so
        // every Plan() neighbour gather brute-force-scanned the WHOLE crowd -- O(n^2) for the ~10k
        // high-power agents this manager is built to hold at scale. UseSpatialHash is a proven
        // bit-identical pre-filter (OrcaSpatialHashTests): it changes candidate discovery from a full
        // scan to a 3x3-cell gather, sorted to the SAME order the brute-force scan would visit, so the
        // neighbour set (and hence every trajectory) is unchanged -- only the wall-clock cost drops.
        // This manager never calls AddObstacle on `_highCrowd` (no static walls in the LOD population),
        // so UseObstacleSpatialIndex is left off (inert either way, but there is nothing for it to
        // accelerate here).
        _highCrowd = new OrcaCrowd { UseSpatialHash = true };
        _highController = new PedRouteController(_highCrowd, _steering, _arriveRadius);
    }

    // Registers a new ped as low-power (PathArc), following `path` at `maxSpeed` from `now`.
    // `path[^1]` is treated as the ped's destination (used to re-route on later promote/demote).
    // Publishes the spawn PathArcRecord (the "path sent once").
    public int AddPed(int id, IReadOnlyList<Vec2> path, double maxSpeed, double radius, double now)
    {
        if (path.Count == 0)
        {
            throw new ArgumentException("A ped's initial path must have at least one point.", nameof(path));
        }

        var entry = new PedEntry
        {
            Id = id,
            Destination = path[^1],
            MaxSpeed = maxSpeed,
            Radius = radius,
            Path = path,
            PathStartTime = now,
            StateEnteredAt = now,
        };

        _peds.Add(id, entry);
        _publisher.PublishPathArc(id, path, now, maxSpeed, now);
        return id;
    }

    // LIVE-PROD-1a (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §4, §10): registers a LIVELY low-power ped whose
    // pose is `timeline` (Walk legs plus Pause/Dwell/Interact beats) evaluated by ActivityTimeline.PoseAt,
    // rather than a bare PathArc leg. It is still low-power and O(1)/step (PoseAt is a pure function of
    // time), and still server==IG: the whole timeline is broadcast ONCE here (ActivityTimelineRecord,
    // mirroring AddPed's "path sent once") and the IG reconstructs pose+visibility by calling the same
    // PoseAt. `timeline`'s final pose is treated as the destination (used to re-route on promote/demote
    // and for demand-side arrival). The ped can promote to a full reactive OrcaCrowd agent exactly like a
    // PathArc ped -- see Step's promotion branch, which carries PoseAt/VelocityAt forward.
    public int AddPedLively(int id, ActivityTimeline timeline, double maxSpeed, double radius, double now)
    {
        var entry = new PedEntry
        {
            Id = id,
            Destination = timeline.PoseAt(timeline.EndTime).Pos,
            MaxSpeed = maxSpeed,
            Radius = radius,
            Model = PedDrModel.ActivityTimeline,
            Timeline = timeline,
            PathStartTime = now,
            StateEnteredAt = now,
            WeaveSeed = timeline.Seed,
            WeaveGlobalSeed = timeline.GlobalSeed,
        };

        _peds.Add(id, entry);
        _publisher.PublishActivityTimeline(id, timeline, now);
        _publisher.PublishSwitch(id, PedDrModel.PathArc, PedDrModel.ActivityTimeline, now);
        return id;
    }

    // W4: force a routed path to START exactly at `anchor` (the frozen demote pose), so a resume leg's
    // pose at arc 0 IS the ped's current position -- machine-precision no-pop across the LOD switch. Drops
    // any leading routed point that coincides with the anchor (avoids a zero-length first segment).
    private static IReadOnlyList<Vec2> ReanchorAt(IReadOnlyList<Vec2> routed, Vec2 anchor)
    {
        var pts = new List<Vec2>(routed.Count + 1) { anchor };
        foreach (var p in routed)
        {
            if ((p - anchor).Abs > 1e-9 || pts.Count > 1)
            {
                pts.Add(p);
            }
        }

        if (pts.Count < 2)
        {
            pts.Add(anchor); // degenerate route -- keep a valid 2-point leg
        }

        return pts;
    }

    // ADDITIVE (P2-3, docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-NAVMESH-CONTRACT.md): removes a ped
    // entirely -- the "arrived at its OD destination, despawn" case a demand generator needs, distinct
    // from a demotion (which keeps the ped, just switches its DR model). If the ped is currently
    // High-power (FreeKinematic), releases its OrcaCrowd handle and route exactly like a demotion's
    // removal side (P0-3) -- every OTHER high-power ped's handle/route/waypoint cursor is untouched --
    // so a despawn never leaks a crowd slot (HighCrowdSlotHighWater may still record the slot as ever-
    // allocated, but HighPowerCount/live occupancy drops immediately and the slot is free-listed for
    // reuse). If Low-power, PathArc motion is a pure function of (path, startTime, speed, now) with no
    // crowd-side state at all, so dropping the dictionary entry is the whole removal. Inert (no-op) if
    // `id` is not currently registered, mirroring OrcaCrowd.Remove / PedRouteController.RemoveRoute's
    // established "removing something already gone is harmless" convention.
    public void RemovePed(int id)
    {
        if (!_peds.TryGetValue(id, out var e))
        {
            return;
        }

        if (e.Model == PedDrModel.FreeKinematic)
        {
            _highController.RemoveRoute(e.HighIndex);
            _highCrowd.Remove(e.HighIndex);
            _highPowerLiveCount--;
        }

        _peds.Remove(id);
    }

    public PedDrModel ModelOf(int id) => _peds[id].Model;

    // P1-2 (docs/PEDESTRIAN-DESIGN.md §6 evac panic; the PedestrianWorld facade's `SetForcedHighPower`):
    // pin/unpin a ped to high-power. While pinned it promotes on the next Step regardless of any
    // InterestSource and never demotes; unpinning lets it demote normally once it is outside every
    // demote radius. Inert (no-op) if `id` is not registered, mirroring RemovePed's convention. The
    // actual PathArc->FreeKinematic Add happens in the next Step's promotion pass (never mid-call),
    // preserving the "structural mutations are deferred to Step" discipline the whole manager relies on.
    public void SetForcedHighPower(int id, bool on)
    {
        if (_peds.TryGetValue(id, out var e))
        {
            e.ForcedHighPower = on;
        }
    }

    // LIVE-PROD-1b (docs/PEDESTRIAN-LIVELINESS-DESIGN.md §4, §10): the ped's current animation tag, so
    // a caller (a Sim.Viz demo, or eventually an IG) can pick a disc kind / anim clip without
    // re-evaluating PoseAt itself or reaching into PedEntry (private). An ActivityTimeline ped reports
    // its live `PoseAt(now).AnimTag` (walk/pause/dwell tag, per the timeline); every other model (plain
    // PathArc, FreeKinematic) has no richer per-step state than "in motion", so it reports
    // ActivityTimeline.WalkAnimTag -- read-only, additive, and touches no existing behavior.
    public string AnimTagOf(int id, double now)
    {
        var e = _peds[id];
        return e.Model == PedDrModel.ActivityTimeline ? e.Timeline!.PoseAt(now).AnimTag : ActivityTimeline.WalkAnimTag;
    }

    public int HighPowerCount => _highPowerLiveCount;

    // Diagnostic-only (P0-4 investigation, docs/PEDESTRIAN-POC7C-FINDINGS.md follow-up hypothesis):
    // the persistent `_highCrowd`'s slot high-water mark (OrcaCrowd.Count -- the number of slots EVER
    // allocated, not the live count), for benchmarks/tests to compare against HighPowerCount and
    // quantify how far the high-water mark has drifted above the live count after a churn spike. Never
    // consulted by Step() itself; purely observability.
    public int HighCrowdSlotHighWater => _highCrowd.Count;

    // The ped's current world position: for Low-power this is the pure PathArcMotion function
    // evaluated AT `now` (so it can be queried for any `now`, not just at a Step boundary); for
    // High-power this is the last-committed OrcaCrowd position (the truth only advances via Step).
    public Vec2 PositionOf(int id, double now)
    {
        var e = _peds[id];
        return e.Model switch
        {
            PedDrModel.FreeKinematic => _highCrowd.Position(e.HighIndex),
            PedDrModel.ActivityTimeline => e.Timeline!.PoseAt(now).Pos,
            _ => PathArcMotion.PositionAt(e.Path, e.PathStartTime, e.MaxSpeed, now),
        };
    }

    // Advances every ped by `dt`, from time `now` to `now + dt`:
    //   1. Evaluate promotion/demotion (pure function of frozen ped/source positions + dwell timers),
    //      in ascending ped-id order.
    //   2. Apply transitions: flip PedDrModel, Add/Remove the ONE affected ped's OrcaCrowd handle and
    //      PedRouteController route (P0-3 -- O(1) per switch, no rebuild), emit lifecycle events
    //      (DrSwitchEvent, and on demotion a fresh PathArcRecord).
    //   3. Advance motion: low-power peds are a pure function of time (nothing to "step"); the
    //      high-power crowd is stepped once, avoiding `externalEntities`.
    //   4. Publish this step's wire traffic: a FreeKinematicSample per high-power ped, a (rate-limited)
    //      HeartbeatEvent per low-power ped.
    public void Step(
        double now,
        double dt,
        InterestField interestField,
        IReadOnlyList<WorldDisc> externalEntities)
    {
        // Freeze the interest field's spatial index for this whole step (P1-1, docs §5: "Promotion is
        // a pure function of frozen state (source positions are start-of-step)") -- every ped queried
        // below sees the exact same source snapshot, regardless of evaluation order. See
        // InterestField.RebuildIndex remarks for why this is O(sources), not O(peds).
        interestField.RebuildIndex();

        var ids = new List<int>(_peds.Keys);
        ids.Sort(); // ascending ped-id order -- deterministic evaluation and application

        var frozenPos = new Dictionary<int, Vec2>(ids.Count);
        foreach (var id in ids)
        {
            frozenPos[id] = PositionOf(id, now);
        }

        var toPromote = new List<int>();
        var toDemote = new List<int>();
        foreach (var id in ids)
        {
            var e = _peds[id];
            var pos = frozenPos[id];
            var stateAge = now - e.StateEnteredAt;

            // Low-power = PathArc OR ActivityTimeline (a lively low-power ped, LIVE-PROD-1a); both promote
            // the same way. FreeKinematic is the only high-power model. Stationary is not used here.
            if (e.Model != PedDrModel.FreeKinematic)
            {
                // A pinned ped (P1-2 SetForcedHighPower, evac panic) promotes immediately -- no interest
                // source and no minimum-dwell gate; otherwise the ordinary interest-field promotion.
                if (e.ForcedHighPower || (stateAge >= _dwellSeconds && interestField.Query(pos).AnyWithinPromote))
                {
                    toPromote.Add(id);
                }
            }
            else if (e.Model == PedDrModel.FreeKinematic)
            {
                // A pinned ped never demotes while pinned.
                if (!e.ForcedHighPower && interestField.Query(pos).AllOutsideDemote)
                {
                    if (double.IsNaN(e.OutsideSince))
                    {
                        e.OutsideSince = now;
                    }

                    if (stateAge >= _dwellSeconds && now - e.OutsideSince >= _dwellSeconds)
                    {
                        toDemote.Add(id);
                    }
                }
                else
                {
                    e.OutsideSince = double.NaN; // back inside someone's demote radius: cancel the countdown
                }
            }
        }

        // Promotions: PathArc -> FreeKinematic. Adds ONLY this ped to the persistent high-power
        // OrcaCrowd (carrying its frozen position + PathArc-derived velocity forward) and registers
        // ONLY its route -- every already-high ped's handle/route is untouched (P0-3).
        foreach (var id in toPromote)
        {
            var e = _peds[id];
            var pos = frozenPos[id];
            var velocity = e.Model == PedDrModel.ActivityTimeline
                ? e.Timeline!.VelocityAt(now)
                : PathArcMotion.VelocityAt(e.Path, e.PathStartTime, e.MaxSpeed, now);
            var steeringPath = _navigation.FindPath(pos, e.Destination) ?? new[] { pos, e.Destination };

            e.Model = PedDrModel.FreeKinematic;
            e.StateEnteredAt = now;
            e.OutsideSince = double.NaN;
            e.Path = steeringPath;
            e.Timeline = null; // now a reactive high-power agent; a later demotion resumes as plain PathArc

            var handle = _highCrowd.Add(pos, e.Radius, e.MaxSpeed, goal: pos, velocity: velocity);
            _highController.AddRoute(handle, steeringPath, e.MaxSpeed);
            e.HighIndex = handle;
            _highPowerLiveCount++;

            _publisher.PublishSwitch(id, PedDrModel.PathArc, PedDrModel.FreeKinematic, now);
        }

        // Demotions: FreeKinematic -> PathArc. Re-routes from the ped's CURRENT (frozen) position to
        // its destination via IPedNavigation (see the class remarks for why re-route rather than
        // resume), then Removes ONLY this ped's OrcaCrowd handle and route -- every other high-power
        // ped's handle/route/waypoint cursor is untouched (P0-3).
        foreach (var id in toDemote)
        {
            var e = _peds[id];
            var pos = frozenPos[id];
            var routed = _navigation.FindPath(pos, e.Destination) ?? new[] { pos, e.Destination };
            // Re-anchor the resume leg EXACTLY at the frozen high-power pose, so the low-power pose at the
            // demote instant is the ped's current position to machine precision (no pop across the LOD switch).
            var newPath = ReanchorAt(routed, pos);

            _highController.RemoveRoute(e.HighIndex);
            _highCrowd.Remove(e.HighIndex);
            _highPowerLiveCount--;

            e.StateEnteredAt = now;
            e.PathStartTime = now;
            e.HighIndex = OrcaHandle.Invalid;

            if (e.WeaveSeed != 0)
            {
                // W4: a weaving ped RESUMES the deterministic weave -- emit a single-Walk ActivityTimeline
                // resume leg (exact ActivityTimelineWire, unlike the quantized PathArc record) carrying the
                // ped's own weave seed + the baked per-vertex half-widths. The Offset start-taper is 0 at the
                // re-anchored start, so the pose leaves `pos` with no pop and weaves back in over the lead-in.
                var widths = _navigation.HalfWidthsAlong(newPath);
                var resume = new ActivityTimeline(
                    now,
                    new ActivitySegment[] { new WalkSegment(newPath, e.MaxSpeed, widths) },
                    e.WeaveSeed, e.WeaveGlobalSeed);

                e.Model = PedDrModel.ActivityTimeline;
                e.Timeline = resume;
                e.Path = newPath;
                _publisher.PublishActivityTimeline(id, resume, now);
                _publisher.PublishSwitch(id, PedDrModel.FreeKinematic, PedDrModel.ActivityTimeline, now);
            }
            else
            {
                e.Model = PedDrModel.PathArc;
                e.Path = newPath;
                _publisher.PublishPathArc(id, newPath, now, e.MaxSpeed, now);
                _publisher.PublishSwitch(id, PedDrModel.FreeKinematic, PedDrModel.PathArc, now);
            }
        }

        if (_highPowerLiveCount > 0)
        {
            var discs = new WorldDisc[externalEntities.Count];
            for (var i = 0; i < discs.Length; i++)
            {
                discs[i] = externalEntities[i];
            }

            _highCrowd.SetExternalObstacles(discs);
            _highController.Update();
            _highCrowd.Step(dt);
        }

        var newNow = now + dt;
        foreach (var id in ids)
        {
            var e = _peds[id];
            if (e.Model == PedDrModel.FreeKinematic)
            {
                _publisher.PublishSample(id, newNow, _highCrowd.Position(e.HighIndex), _highCrowd.Velocity(e.HighIndex));
            }
            else
            {
                _publisher.MaybePublishHeartbeat(id, newNow);
            }
        }
    }

}
