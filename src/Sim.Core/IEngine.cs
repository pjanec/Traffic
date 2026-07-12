namespace Sim.Core;

// The engine seam (DESIGN.md "build order"): implemented starting Task 2. Not implemented here.
public interface IEngine
{
    void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath);

    TrajectorySet Run(int steps);

    // ----- Handle-based external-obstacle API (SUMOSHARP-API.md §4.4, the primary surface) -----
    // B1 external-obstacle input surface (DESIGN.md "Two futures"): a live object (non-SUMO vehicle,
    // pedestrian, detection) injected between the offline model and the reducer. Inert-when-absent, and
    // zero-allocation -- Add returns a generational ObstacleHandle; Update/Remove address it by index.
    //
    // Resolve the lane's string id to an int lane handle ONCE via GetLane, then the per-step path is
    // all handles. B6: latPos/width (default 0/0 == pre-B6 full-lane block) give the obstacle a lateral
    // footprint so a car can swerve around it. D17: avoidanceClass is the reserved RVO reciprocity
    // class, inert for the lane-based engine.
    int GetLane(string laneId);

    ObstacleHandle AddObstacle(int laneHandle, double frontPos, double length,
                               double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity,
                               double latPos = 0.0, double width = 0.0, double latSpeed = 0.0,
                               AvoidanceClass avoidanceClass = AvoidanceClass.OneSided);

    // B5-i: the MOVING generalization -- the obstacle carries a velocity the engine dead-reckons each
    // step (Engine.AdvanceObstacles, Input phase, before PlanMovements) until the owner calls
    // UpdateObstacle with a fresh reading. speed=0 degenerates to a static obstacle.
    ObstacleHandle AddMovingObstacle(int laneHandle, double frontPos, double length,
                                     double speed, double maxDecel,
                                     double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity,
                                     double latPos = 0.0, double width = 0.0, double latSpeed = 0.0,
                                     AvoidanceClass avoidanceClass = AvoidanceClass.OneSided);

    // B5-i: per-step correction from the external layer that owns this obstacle's real motion (navmesh/
    // RVO agent, live detection). No-op on a stale/removed handle (inert-when-absent).
    void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed);

    // B6: per-step correction that also moves the lateral centre (a pedestrian walking across).
    void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed, double latPos);

    // B6-lat: full correction including the lateral velocity, for predicting a lunging pedestrian.
    void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed, double latPos, double latSpeed);

    void RemoveObstacle(ObstacleHandle handle);

    void ClearObstacles();
}
