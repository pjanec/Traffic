using Sim.Core.Orca;
using Sim.Pedestrians.Navigation;

namespace Sim.Pedestrians.Crossing;

// Rule-based crosswalk gate (docs/PEDESTRIAN-DESIGN.md §6, decision D5; docs/PEDESTRIAN-POC-PLAN.md
// POC-2). Drives registered OrcaCrowd agents along a path, exactly like
// Sim.Pedestrians.Navigation.Bake.PedRouteController, but with one addition: while `ICrossingSignal.
// WalkAllowed(now)` is false, an agent that has not yet passed the crossing's near-side portal
// waypoint holds its crowd goal at a QUEUE SLOT behind that portal instead of advancing past it.
//
// Why a queue slot per agent, not one shared hold point at the portal: OrcaCrowd's own goal-seeking
// (OrcaCrowd.Plan) eases a SOLITARY agent's speed to zero exactly at its goal (no overshoot) -- but
// when several agents are all given the literal same goal point, ORCA's reciprocal avoidance can
// still nudge the FRONT agent well past that point once a trailing agent (still approaching at
// speed, its own distance-to-goal not yet small) presses into its collision radius: the reciprocal
// solve splits the required separating velocity between both agents, including the one that had
// already "arrived", and the push does not stop after one step -- while trailing agents still have
// unspent momentum toward THEIR OWN goals, they keep shoving the one(s) ahead of them, transiently
// (not just by a few cm).
//
// Measured empirically (a standalone probe: N agents, each spawned some distance behind its own,
// evenly-`queueSpacing`-apart hold slot, all walking toward the crossing simultaneously -- i.e. a
// realistic "approaching and queueing" scenario, not agents that start already resting near their
// slot): with a shared goal point the lead agent gets pushed arbitrarily far past the goal (the
// approach never stops shoving it). Giving every agent its own slot removes the unbounded case but a
// residual transient shove remains on the FRONT slot specifically (the one with no slack ahead of
// it) -- e.g. at `queueSpacing=2.0` with only the first-registered route holding right at the portal,
// the front agent was measured dipping ~0.6-0.7 m PAST the portal line regardless of approach
// distance. Reserving TWO slots of buffer ahead of the front-most registrant (below: hold slot =
// `(registrationIndex + 2) * queueSpacing` back from the portal, i.e. even the FIRST registrant holds
// two spacings behind it) pushed the measured margin back to a comfortable 1.3-1.4 m clear of the
// line at `queueSpacing=2.0` for 6-20 queued agents (MaxNeighbours=8) -- callers should pick
// `queueSpacing` generously above sum-of-radii (not just past it: the probe found ~2x the minimum
// separation distance is where the transient shove shrinks enough to matter) and keep this 2-slot
// buffer in mind when placing the portal itself (i.e. leave real-world room behind a crossing's stop
// line). See CrossingGateCrowdTests for the exact values proven safe against POC-0's fixture.
//
// When the signal opens, held agents are simply allowed to resume ordinary waypoint following, which
// walks them from their queue slot to the actual portal point and out the far side; the release
// "surge" is emergent (a bottleneck draining), not scripted.
//
// Deterministic: registration order is fixed insertion order (no RNG) and also fixes each route's
// queue slot for its lifetime; the only external input is the caller-supplied `now` and each
// ICrossingSignal, itself a pure function of `now`.
public sealed class CrossingGate
{
    private sealed class Route
    {
        public required OrcaHandle CrowdIndex;
        public required IReadOnlyList<Vec2> Path;
        public required int PortalIndex;
        public required double MaxSpeed;
        public required Vec2 HoldPoint;
        public int WaypointIndex;
    }

    private readonly OrcaCrowd _crowd;
    private readonly ILocalSteering _steering;
    private readonly ICrossingSignal _signal;
    private readonly double _arriveRadius;
    private readonly Vec2 _queueDirection;
    private readonly double _queueSpacing;
    private readonly List<Route> _routes = new();

    // `queueDirection` points FROM the portal back toward the approaching agents (i.e. away from the
    // crossing); it need not be normalized (this normalizes it). `queueSpacing` is the distance
    // between consecutive queue slots along that direction -- must exceed twice the agents' radius so
    // held agents never contest a slot with each other.
    public CrossingGate(
        OrcaCrowd crowd, ILocalSteering steering, ICrossingSignal signal, double arriveRadius,
        Vec2 queueDirection, double queueSpacing)
    {
        if (queueDirection.AbsSq <= 1e-12)
        {
            throw new ArgumentException("queueDirection must be non-zero.", nameof(queueDirection));
        }

        if (queueSpacing <= 0.0)
        {
            throw new ArgumentOutOfRangeException(nameof(queueSpacing), queueSpacing, "must be positive.");
        }

        _crowd = crowd;
        _steering = steering;
        _signal = signal;
        _arriveRadius = arriveRadius;
        _queueDirection = queueDirection.Normalized();
        _queueSpacing = queueSpacing;
    }

    // Reserved buffer (in queue slots) between the portal itself and the first-registered route's
    // hold point -- see the class remarks for the measured transient-shove margin this buys.
    private const int FrontSlotBuffer = 2;

    // Registers an already-added crowd agent (the handle OrcaCrowd.Add returned) to follow `path`,
    // holding at a queue slot behind `path[portalIndex]` -- the crossing's near-side waypoint --
    // whenever the signal is closed. The slot is this route's position in registration order plus
    // FrontSlotBuffer (the first-registered route holds `FrontSlotBuffer * queueSpacing` behind the
    // portal, not at it -- see the class remarks for why that margin matters; each subsequent route
    // holds one more `queueSpacing` further back along `queueDirection`). `path[portalIndex + 1]`
    // (when present) is the far-side waypoint, only pursued once released. Immediately targets the
    // agent's goal (matching PedRouteController.AddRoute), so a caller does not need to call Update()
    // before the first OrcaCrowd.Step().
    public void AddRoute(OrcaHandle crowdIndex, IReadOnlyList<Vec2> path, int portalIndex, double maxSpeed, double now)
    {
        if (portalIndex < 0 || portalIndex >= path.Count)
        {
            throw new ArgumentOutOfRangeException(nameof(portalIndex), portalIndex, "must index into path.");
        }

        var slot = _routes.Count + FrontSlotBuffer;
        var holdPoint = path[portalIndex] + (_queueDirection * (slot * _queueSpacing));

        var route = new Route
        {
            CrowdIndex = crowdIndex, Path = path, PortalIndex = portalIndex, MaxSpeed = maxSpeed,
            HoldPoint = holdPoint, WaypointIndex = 0,
        };
        _routes.Add(route);
        UpdateGoal(route, _signal.WalkAllowed(now));
    }

    // True once crowdIndex has advanced past the last waypoint of its registered path.
    public bool IsRouteComplete(OrcaHandle crowdIndex)
    {
        foreach (var route in _routes)
        {
            if (route.CrowdIndex == crowdIndex)
            {
                return route.WaypointIndex >= route.Path.Count;
            }
        }

        return true; // not a registered route -> vacuously "complete"
    }

    // True while `crowdIndex` is being held at its portal: registered, has not advanced its waypoint
    // cursor past the portal, and the signal is closed at `now`. Observability hook for "none entering
    // the roadway" (POC-2 success condition 1).
    public bool IsHeld(OrcaHandle crowdIndex, double now)
    {
        foreach (var route in _routes)
        {
            if (route.CrowdIndex == crowdIndex)
            {
                return route.WaypointIndex <= route.PortalIndex && !_signal.WalkAllowed(now);
            }
        }

        return false;
    }

    public int WaypointIndexOf(OrcaHandle crowdIndex)
    {
        foreach (var route in _routes)
        {
            if (route.CrowdIndex == crowdIndex)
            {
                return route.WaypointIndex;
            }
        }

        return -1;
    }

    // Advances every registered route's goal for the current tick. Call before OrcaCrowd.Step(dt).
    public void Update(double now)
    {
        var walk = _signal.WalkAllowed(now);
        foreach (var route in _routes) // fixed insertion order -> deterministic
        {
            UpdateGoal(route, walk);
        }
    }

    private void UpdateGoal(Route route, bool walk)
    {
        if (route.Path.Count == 0)
        {
            return;
        }

        if (route.WaypointIndex <= route.PortalIndex && !walk)
        {
            // HOLD: pin the goal at this route's OWN queue slot (never the shared portal point --
            // see the class remarks) without advancing the waypoint cursor. The agent decelerates
            // into it exactly like any other waypoint arrival (OrcaCrowd.Plan's own goal-seeking
            // eases speed to 0 at the goal), with no cross-agent pressure since no two routes share a
            // goal.
            _crowd.SetGoal(route.CrowdIndex, route.HoldPoint);
            return;
        }

        // Gate open (or already past the portal): ordinary waypoint following, targeting the REAL
        // path (the portal point itself, not the queue slot) -- every held agent, regardless of how
        // many queue slots back it was, now walks forward to the portal and on through it; its
        // WaypointIndex advances past the portal the moment it comes within arriveRadius of it.
        var position = _crowd.Position(route.CrowdIndex);
        _steering.DesiredVelocity(position, route.Path, ref route.WaypointIndex, route.MaxSpeed, _arriveRadius);

        var targetIndex = Math.Min(route.WaypointIndex, route.Path.Count - 1);
        _crowd.SetGoal(route.CrowdIndex, route.Path[targetIndex]);
    }
}
