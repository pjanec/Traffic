namespace Sim.ExtDemo;

using System.Linq;

// The EXTERNAL-AGENT script schema this demo reads (scenarios/*/external-agents.json). These
// agents are driven by a layer OUTSIDE SUMO/the engine -- they are registered with the engine
// only through the existing B1/B5 obstacle API (IEngine.AddObstacle / AddMovingObstacle), which
// is longitudinal-only (lane id + arc-length pos). The engine never sees "kind" or lateral
// offset; those two fields exist purely for THIS demo's visualization layer (see
// CombinedFcdObserver), so that when the engine eventually grows a lateral field, an agent
// record here barely changes.
//
// Schema (documented on scenarios/_bench/ext-agents-demo/NOTES.md too):
//   id         - stable string id (becomes the engine obstacle id AND, prefixed by kind, the
//                combined-FCD/viz vehicle id -- see CombinedFcdObserver).
//   kind       - "pedestrian" (a static, time-windowed lane blocker; AddObstacle) or "car" (a
//                moving obstacle the engine dead-reckons at `speed`; AddMovingObstacle).
//   laneId     - the SUMO lane id the agent occupies (same lane-relative convention as a
//                vehicle's own pos: frontPos is the agent's DOWNSTREAM/front edge). This is also
//                the REFERENCE lane the viz derives x/y from (LaneGeometry.PositionAtOffset).
//   blockLaneIds - OPTIONAL extra lane ids, ALSO registered as an obstacle at the same
//                startPos/length/window (one AddObstacle/AddMovingObstacle call per lane, all
//                sharing this agent's timing). A crossing pedestrian physically blocks EVERY
//                lane it is currently in front of, not just one -- without this, a SUMO car on
//                the lane the pedestrian did NOT register on simply lane-changes around it
//                (there is no lateral field yet, so the engine has no way to know the pedestrian
//                also occupies the next lane over unless told so explicitly). On a multi-lane
//                crossing, list every OTHER lane the pedestrian's footprint currently spans here.
//   startPos   - the agent's initial front position (m along the lane). For a pedestrian this
//                is its (constant) crossing position; for a car it is where dead-reckoning
//                starts from at simulation time 0 (AdvanceObstacles integrates FrontPos +=
//                speed*dt every step from t=0, matching Engine.AdvanceObstacles/B5-i).
//   length     - along-lane footprint (m) fed straight to AddObstacle/AddMovingObstacle.
//   width      - footprint across the lane (m); NOT read by the engine (longitudinal-only API)
//                -- used only to size the drawn marker.
//   startTime / endTime - the obstacle's [startTime, endTime) active window (same convention as
//                ExternalObstacle.StartTime/EndTime -- StartTime <= t < EndTime).
//   speed      - along-lane velocity (m/s). Ignored for "pedestrian" (must be a static blocker
//                in this rung's scope); required for "car".
//   maxDecel   - the moving obstacle's braking capability (m/s^2), fed to AddMovingObstacle.
//                Optional; defaults to a sane passenger-car value (DefaultMaxDecel) when absent.
//                Ignored for "pedestrian" (AddObstacle has no such parameter).
//   latFrom / latTo - lateral offset from the lane CENTERLINE (m, positive = to the right of the
//                direction of travel -- see CombinedFcdObserver's perpendicular-vector comment),
//                linearly interpolated across [startTime, endTime) for the VISUAL x/y only. A
//                crossing pedestrian sets latFrom/latTo to opposite sides of the lane; a car
//                driving in-lane sets both to 0.
public sealed record ExternalAgentDef(
    string Id,
    string Kind,
    string LaneId,
    IReadOnlyList<string> BlockLaneIds,
    double StartPos,
    double Length,
    double Width,
    double StartTime,
    double EndTime,
    double Speed,
    double? MaxDecel,
    double LatFrom,
    double LatTo)
{
    public const double DefaultMaxDecel = 4.5;

    public bool IsPedestrian => string.Equals(Kind, "pedestrian", StringComparison.OrdinalIgnoreCase);

    public bool IsCar => string.Equals(Kind, "car", StringComparison.OrdinalIgnoreCase);

    // Every lane this agent is registered as an obstacle on: the reference lane plus any extra
    // blockLaneIds, deduplicated (LaneId first, so a caller that only wants the primary keeps
    // using LaneId directly).
    public IReadOnlyList<string> EngineLaneIds =>
        BlockLaneIds.Count == 0 ? new[] { LaneId } : new[] { LaneId }.Concat(BlockLaneIds).Distinct().ToList();

    public bool ActiveAt(double time) => StartTime <= time && time < EndTime;

    // Front position at simulation time `time` -- byte-identical to Engine.AdvanceObstacles'
    // own dead-reckoning (FrontPos += Speed*dt every step from t=0), verified against
    // RungB5MovingObstacleTests.ObstacleBackAtExact's same `initialFrontPos + speed*time`
    // formula. A pedestrian (Speed=0 at the engine) never moves along the lane.
    public double FrontPosAt(double time) => IsCar ? StartPos + Speed * time : StartPos;

    // Linear lateral interpolation across the active window, clamped at the edges so a caller
    // that evaluates slightly outside [startTime,endTime) (belt-and-suspenders; ActiveAt already
    // guards real callers) still gets a sane value instead of extrapolating.
    public double LatAt(double time)
    {
        if (EndTime <= StartTime)
        {
            return LatFrom;
        }

        var t = Math.Clamp((time - StartTime) / (EndTime - StartTime), 0.0, 1.0);
        return LatFrom + (LatTo - LatFrom) * t;
    }

    // Combined-FCD / viz vehicle id: prefixed by kind so Sim.Viz's naming-convention resolution
    // (see src/Sim.Viz/Program.cs) can tell an external agent apart from a real SUMO vehicle
    // without needing a rou.xml <vehicle>/<vType> entry that would also make the ENGINE simulate
    // it as a real car -- exactly what an external agent must NOT be.
    public string RenderId => (IsPedestrian ? "ext_pedestrian_" : "ext_car_") + Id;

    public string RenderType => IsPedestrian ? "ext_pedestrian" : "ext_car";
}

public sealed record ExternalAgentsFile(IReadOnlyList<ExternalAgentDef> Agents);
