using Sim.Core.Orca;

namespace Sim.Pedestrians.Navigation.Bake;

// Integration driver (POC-1a): ties the strategic/tactical navigation layers to the operational
// OrcaCrowd (docs/PEDESTRIAN-DESIGN.md §4). Each registered agent holds a path (from
// IPedNavigation.FindPath) and a waypoint cursor; every Update():
//
//   1. Calls ILocalSteering.DesiredVelocity for the agent. OrcaCrowd has no "external preferred
//      velocity" input -- it always steers each agent toward its OWN goal (OrcaCrowd.Plan), so the
//      returned velocity itself is not fed into the crowd. What Update() actually needs from the
//      steering call is its SIDE EFFECT: advancing `waypointIndex` past any waypoint already
//      reached, i.e. deciding WHICH waypoint is now the current steering target.
//   2. Sets the crowd agent's GOAL to that current target (path[waypointIndex]) via
//      OrcaCrowd.SetGoal. OrcaCrowd's own goal-seeking + reciprocal-avoidance solve then turns
//      "walk toward this point" into an actual velocity for the step, negotiating with every other
//      crowd agent exactly as it already does -- this is what gets ORCA avoidance "for free" on
//      top of path-following, per the task description.
//
// Call Update() once per tick BEFORE OrcaCrowd.Step(dt); the caller owns stepping the crowd itself
// (this class only ever reads/sets goals, never calls Step).
//
// Deterministic: agents are updated in a fixed list (registration) order, no System.Random.
public sealed class PedRouteController
{
    private sealed class Route
    {
        public required OrcaHandle CrowdIndex;
        public required IReadOnlyList<Vec2> Path;
        public required double MaxSpeed;
        public int WaypointIndex;
    }

    private readonly OrcaCrowd _crowd;
    private readonly ILocalSteering _steering;
    private readonly double _arriveRadius;
    private readonly List<Route> _routes = new();

    public PedRouteController(OrcaCrowd crowd, ILocalSteering steering, double arriveRadius)
    {
        _crowd = crowd;
        _steering = steering;
        _arriveRadius = arriveRadius;
    }

    // Registers an already-added crowd agent (the handle OrcaCrowd.Add returned) to follow `path`
    // at `maxSpeed`, and immediately targets its goal at the path's first steering waypoint.
    public void AddRoute(OrcaHandle crowdIndex, IReadOnlyList<Vec2> path, double maxSpeed)
    {
        var route = new Route { CrowdIndex = crowdIndex, Path = path, MaxSpeed = maxSpeed, WaypointIndex = 0 };
        _routes.Add(route);
        UpdateGoal(route);
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

    // Advances every registered route's waypoint cursor and (re)targets its crowd goal. Call
    // before OrcaCrowd.Step(dt) each tick.
    public void Update()
    {
        foreach (var route in _routes) // fixed insertion order -> deterministic
        {
            UpdateGoal(route);
        }
    }

    private void UpdateGoal(Route route)
    {
        if (route.Path.Count == 0)
        {
            return;
        }

        var position = _crowd.Position(route.CrowdIndex);
        _steering.DesiredVelocity(position, route.Path, ref route.WaypointIndex, route.MaxSpeed, _arriveRadius);

        var targetIndex = Math.Min(route.WaypointIndex, route.Path.Count - 1);
        _crowd.SetGoal(route.CrowdIndex, route.Path[targetIndex]);
    }
}
