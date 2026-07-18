using Sim.Core.Bridge;
using Sim.Core.Orca;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;

namespace Sim.Pedestrians.Lod;

// Sim-LOD promotion/demotion + PathArc<->FreeKinematic switching (docs/PEDESTRIAN-DESIGN.md §5, §7;
// docs/PEDESTRIAN-POC-PLAN.md POC-3). Owns a population of peds, each either:
//   - Low-power (PedDrModel.PathArc): pose is a pure function of (path, startTime, speed, now) via
//     PathArcMotion -- O(1), no neighbour query, no ORCA.
//   - High-power (PedDrModel.FreeKinematic): a real agent in a high-power OrcaCrowd, routed by a
//     PedRouteController + WaypointFollower exactly like POC-1a, reacting to every other high-power
//     ped AND to `externalEntities`.
//
// A ped is high-power iff its (frozen, start-of-step) position lies within ANY active
// InterestSource.PromoteRadius; it demotes once it has been continuously outside EVERY source's
// (larger) DemoteRadius for `dwellSeconds`. `dwellSeconds` ALSO gates how soon a ped may leave the
// state it just entered (both directions) -- the "minimum-dwell in each state" the design calls for,
// collapsed into one knob for this POC (a production version might separate "how long outside before
// demoting" from "minimum time before ANY transition"; see the report for this simplification).
//
// OrcaCrowd churn (a real design gap, not a POC shortcut): OrcaCrowd has no public agent-removal, so
// membership can't be edited in place. Whenever this step's promotions/demotions change the high-power
// SET, the whole high-power OrcaCrowd is REBUILT from scratch, carrying every still-high ped's current
// position and velocity forward and re-deriving its steering path via IPedNavigation.FindPath(current
// position, destination) (see RebuildHighCrowd). This is O(high-power count) per membership change,
// not per step -- acceptable at POC scale, but "OrcaCrowd needs efficient add/remove" should be
// recorded as a real backlog item for scale (POC-7).
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
        // steering route (refreshed on every high-crowd rebuild) when High.
        public IReadOnlyList<Vec2> Path = Array.Empty<Vec2>();
        public double PathStartTime;

        public OrcaHandle HighIndex = OrcaHandle.Invalid;    // handle into the CURRENT high-power OrcaCrowd, or Invalid when Low
        public Vec2 PendingVelocity;  // velocity-at-promotion, stashed for the rebuild that follows

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

    private OrcaCrowd _highCrowd;
    private PedRouteController _highController;
    private bool _useParallelHighCrowd;

    // Additive POC-7c wiring: forwards OrcaCrowd.UseParallelStep (POC-7a, bit-identical to serial) onto
    // whichever OrcaCrowd currently backs the high-power set -- both the live one and, since
    // RebuildHighCrowd (a real design gap -- see class remarks) replaces it wholesale on every
    // membership change, every crowd created afterwards. Default false, matching OrcaCrowd's own
    // default, so no existing caller's behavior changes.
    public bool UseParallelHighCrowd
    {
        get => _useParallelHighCrowd;
        set
        {
            _useParallelHighCrowd = value;
            _highCrowd.UseParallelStep = value;
        }
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
        _highCrowd = new OrcaCrowd();
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

    public PedDrModel ModelOf(int id) => _peds[id].Model;

    public int HighPowerCount => _highCrowd.Count;

    // The ped's current world position: for Low-power this is the pure PathArcMotion function
    // evaluated AT `now` (so it can be queried for any `now`, not just at a Step boundary); for
    // High-power this is the last-committed OrcaCrowd position (the truth only advances via Step).
    public Vec2 PositionOf(int id, double now)
    {
        var e = _peds[id];
        return e.Model == PedDrModel.FreeKinematic
            ? _highCrowd.Position(e.HighIndex)
            : PathArcMotion.PositionAt(e.Path, e.PathStartTime, e.MaxSpeed, now);
    }

    // Advances every ped by `dt`, from time `now` to `now + dt`:
    //   1. Evaluate promotion/demotion (pure function of frozen ped/source positions + dwell timers),
    //      in ascending ped-id order.
    //   2. Apply transitions: flip PedDrModel, emit lifecycle events (DrSwitchEvent, and on demotion a
    //      fresh PathArcRecord).
    //   3. Rebuild the high-power OrcaCrowd if membership changed this step.
    //   4. Advance motion: low-power peds are a pure function of time (nothing to "step"); the
    //      high-power crowd is stepped once, avoiding `externalEntities`.
    //   5. Publish this step's wire traffic: a FreeKinematicSample per high-power ped, a (rate-limited)
    //      HeartbeatEvent per low-power ped.
    public void Step(
        double now,
        double dt,
        IReadOnlyList<InterestSource> interestSources,
        IReadOnlyList<WorldDisc> externalEntities)
    {
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

            if (e.Model == PedDrModel.PathArc)
            {
                if (stateAge >= _dwellSeconds && AnySourceWithinPromote(interestSources, pos))
                {
                    toPromote.Add(id);
                }
            }
            else if (e.Model == PedDrModel.FreeKinematic)
            {
                if (AllSourcesOutsideDemote(interestSources, pos))
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

        var membershipChanged = toPromote.Count > 0 || toDemote.Count > 0;

        // Promotions: PathArc -> FreeKinematic. Stash the velocity-at-promotion (computed from the
        // STILL-INTACT PathArc fields) for RebuildHighCrowd to pick up below.
        foreach (var id in toPromote)
        {
            var e = _peds[id];
            e.PendingVelocity = PathArcMotion.VelocityAt(e.Path, e.PathStartTime, e.MaxSpeed, now);
            e.Model = PedDrModel.FreeKinematic;
            e.StateEnteredAt = now;
            e.OutsideSince = double.NaN;
            e.HighIndex = OrcaHandle.Invalid;
            _publisher.PublishSwitch(id, PedDrModel.PathArc, PedDrModel.FreeKinematic, now);
        }

        // Demotions: FreeKinematic -> PathArc. Re-route from the ped's CURRENT position to its
        // destination via IPedNavigation (see the class remarks for why re-route rather than resume).
        foreach (var id in toDemote)
        {
            var e = _peds[id];
            var pos = frozenPos[id];
            var newPath = _navigation.FindPath(pos, e.Destination) ?? new[] { pos, e.Destination };
            e.Model = PedDrModel.PathArc;
            e.Path = newPath;
            e.PathStartTime = now;
            e.StateEnteredAt = now;
            e.HighIndex = OrcaHandle.Invalid;
            _publisher.PublishPathArc(id, newPath, now, e.MaxSpeed, now);
            _publisher.PublishSwitch(id, PedDrModel.FreeKinematic, PedDrModel.PathArc, now);
        }

        if (membershipChanged)
        {
            RebuildHighCrowd(frozenPos, now);
        }

        if (_highCrowd.Count > 0)
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

    // Rebuilds the high-power OrcaCrowd from scratch (OrcaCrowd has no agent-removal -- see class
    // remarks). Every currently-high ped (post this step's promotions/demotions) is re-added in
    // ascending ped-id order:
    //   - already-high peds carry over the OLD crowd's current position/velocity;
    //   - freshly-promoted peds carry over their frozen PathArc position + PendingVelocity.
    // Every one of them gets a FRESH steering path via IPedNavigation.FindPath(currentPosition,
    // destination) -- not just the newly-promoted ones -- because the OLD PedRouteController's
    // per-route waypoint cursor lives inside the discarded controller; re-deriving the path from the
    // ped's actual current position is simpler and more robust than trying to carry a waypoint index
    // across the rebuild (documented design choice, see report).
    private void RebuildHighCrowd(IReadOnlyDictionary<int, Vec2> frozenPos, double now)
    {
        _ = now; // kept for symmetry/future use (e.g. time-stamped diagnostics); not needed today

        var oldCrowd = _highCrowd;
        var newCrowd = new OrcaCrowd { UseParallelStep = _useParallelHighCrowd };
        var newController = new PedRouteController(newCrowd, _steering, _arriveRadius);

        var ids = new List<int>(_peds.Keys);
        ids.Sort(); // ascending ped-id order -- deterministic add order into the new crowd

        foreach (var id in ids)
        {
            var e = _peds[id];
            if (e.Model != PedDrModel.FreeKinematic)
            {
                continue;
            }

            Vec2 pos, vel;
            if (e.HighIndex.IsValid)
            {
                pos = oldCrowd.Position(e.HighIndex);
                vel = oldCrowd.Velocity(e.HighIndex);
            }
            else
            {
                pos = frozenPos[id];
                vel = e.PendingVelocity;
            }

            var steeringPath = _navigation.FindPath(pos, e.Destination) ?? new[] { pos, e.Destination };
            e.Path = steeringPath;

            var newIndex = newCrowd.Add(pos, e.Radius, e.MaxSpeed, goal: pos, velocity: vel);
            newController.AddRoute(newIndex, steeringPath, e.MaxSpeed);
            e.HighIndex = newIndex;
        }

        _highCrowd = newCrowd;
        _highController = newController;
    }

    private static bool AnySourceWithinPromote(IReadOnlyList<InterestSource> sources, Vec2 pos)
    {
        for (var i = 0; i < sources.Count; i++)
        {
            var s = sources[i];
            if ((pos - s.Position).Abs <= s.PromoteRadius)
            {
                return true;
            }
        }

        return false;
    }

    private static bool AllSourcesOutsideDemote(IReadOnlyList<InterestSource> sources, Vec2 pos)
    {
        for (var i = 0; i < sources.Count; i++)
        {
            var s = sources[i];
            if ((pos - s.Position).Abs <= s.DemoteRadius)
            {
                return false;
            }
        }

        return true;
    }
}
