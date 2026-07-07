using Sim.Ingest;

namespace Sim.Core;

// Task 3: real Krauss/MSCFModel car-following speed law (ported from
// sumo/src/microsim/cfmodels/MSCFModel*.cpp -- see KraussModel.cs) wired into the plan/execute
// contract and lane-relative position model built in Task 2 (DESIGN.md "The plan/execute
// contract", "Seam 2").
public sealed class Engine : IEngine
{
    private NetworkModel? _network;
    private DemandModel? _demand;
    private ScenarioConfig? _config;
    private readonly List<VehicleRuntime> _vehicles = new();

    // B1: external-obstacle store (DESIGN.md "Two futures" -- live-reactivity input surface, not
    // a SUMO concept). Keyed by id so AddObstacle can add-or-replace. Deliberately NOT cleared by
    // LoadScenario (tests inject obstacles after loading, before Run) and empty by default, which
    // is exactly the inert-when-absent guard: with no entries, ObstacleConstraint below is a
    // trivial +infinity no-op and every parity scenario's constraints list is unaffected.
    private readonly Dictionary<string, ExternalObstacle> _obstacles = new();

    // B3: reroute-around-prolonged-blockage (DESIGN.md "Two futures" -- live-reactivity, not a
    // ported SUMO code path; SUMO's analog is a rerouting device / <rerouter> reacting to a
    // closed edge). Left at +infinity by default, which makes UpdateReroutes below an immediate
    // no-op every step -- the inert-when-absent guard: reroute is strictly opt-in, so no existing
    // (obstacle-free or obstacle-present-but-untested) parity scenario is ever affected.
    public double RerouteThresholdSeconds { get; set; } = double.PositiveInfinity;

    // B2 router, built once lazily from the loaded (immutable) network and cached -- cheap to
    // construct, but there is no reason to rebuild it every step. Null until first needed (either
    // LoadScenario has not run yet, or UpdateReroutes never actually reroutes anything).
    private NetworkRouter? _router;

    public void AddObstacle(string id, string laneId, double frontPos, double length,
        double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity)
    {
        _obstacles[id] = new ExternalObstacle(id, laneId, frontPos, length, startTime, endTime);
    }

    public void RemoveObstacle(string id) => _obstacles.Remove(id);

    public void ClearObstacles() => _obstacles.Clear();

    public void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _demand = DemandParser.Parse(rouXmlPath);
        _config = ScenarioConfigParser.Parse(sumocfgPath);

        // B3: the cached router is built from the network being replaced above -- invalidate it
        // here so UpdateReroutes lazily rebuilds against the NEW network the next time it is
        // actually needed (never eagerly, since most scenarios never reroute at all).
        _router = null;

        _vehicles.Clear();
        foreach (var def in _demand.Vehicles)
        {
            var rawVType = _demand.VTypesById[def.TypeId];
            // vType defaults resolver (CLAUDE.md rule 6: match vType/init first): only vClass
            // and any explicit overrides (e.g. rou.xml's sigma="0") come from the raw parse;
            // everything else is a resolved SUMO vClass default (VTypeDefaults.Resolve).
            var vType = VTypeDefaults.Resolve(rawVType);
            var runtime = new VehicleRuntime { Def = def, VType = vType };

            // Rung 5: seed this vehicle's own stop queue (StopRuntime) from its immutable Def.
            // Reached/RemainingDuration start at their defaults (false/0) -- ProcessNextStop only
            // initializes RemainingDuration once the stop is actually reached.
            foreach (var stopDef in def.Stops)
            {
                runtime.Stops.Enqueue(new StopRuntime
                {
                    LaneId = stopDef.LaneId,
                    StartPos = stopDef.StartPos,
                    EndPos = stopDef.EndPos,
                    Duration = stopDef.Duration,
                });
            }

            _vehicles.Add(runtime);
        }
    }

    public TrajectorySet Run(int steps)
    {
        if (_network is null || _demand is null || _config is null)
        {
            throw new InvalidOperationException("LoadScenario must be called before Run.");
        }

        var trajectory = new TrajectorySet();
        var dt = _config.StepLength;

        for (var step = 0; step < steps; step++)
        {
            var time = _config.Begin + step * dt;

            InsertDepartingVehicles(time);
            EmitTrajectory(trajectory, time);

            // B3: reroute-around-prolonged-blockage. Runs ONCE per step, BEFORE PlanMovements,
            // so a vehicle that reroutes this step immediately plans against its NEW route this
            // same step (see UpdateReroutes' own header comment for why this ordering, and why it
            // is still a seam-4 structural mutation rather than a Plan-phase concern).
            UpdateReroutes(time, dt);

            // Plan/execute contract (DESIGN.md): plan reads start-of-step state and writes
            // only MoveIntent; execute applies all intents afterward. A follower must never
            // see a leader's updated position within the same step. The neighbor query is
            // built ONCE per step, here, from the same frozen start-of-step snapshot every
            // vehicle's plan phase reads (Seam 1: neighbor discovery behind an interface).
            var neighbors = LaneNeighborQuery.Build(_vehicles);
            PlanMovements(neighbors, time);
            ExecuteMoves(dt);

            // Rung A2 (speed-gain/overtaking lane change): SUMO's own per-step order is
            // planMovements -> executeMovements -> changeLanes (MSNet.cpp:784/790/796) -- the
            // lane-change decision (MSLCM_LC2013::_wantsChange's speed-gain block) runs AFTER
            // this step's longitudinal move, so it sees POST-move gaps (unlike keep-right, which
            // has no leader-gap dependence and stays entirely in the pre-move Plan phase per
            // rung 8b). This is a NEW phase, not a change to PlanMovements/ExecuteMoves above.
            DecideSpeedGainChanges(dt);
        }

        return trajectory;
    }

    // Rung 6: gap-gated departure insertion, ported from
    // sumo/src/microsim/MSLane.cpp's isInsertionSuccess (leader-gap tail, ~line 1085-1099),
    // safeInsertionSpeed (~line 1328), and checkFailure (~line 780). Vehicles queue at their
    // departLane/departPos until a leader-gap check passes; unconditional insertion (rungs
    // 1-5) was a placeholder this rung replaces.
    //
    // Derivation used here (all four vehicles in this scenario have departPos="0",
    // departSpeed="0" explicitly given, i.e. patchSpeed=false per MSInsertionControl):
    //   gap = leaderBackPos + seen - egoMinGap, called with seen = -pos (MSLane.cpp:1097)
    //       = (leaderPos - leaderLength) - insertPos - egoMinGap
    //   checkFailure(speed=0, nspeed=min(departSpeed, insertionFollowSpeed(...))=0):
    //       nspeed < speed is 0 < 0 = false -> never fails on speed with departSpeed=0.
    //   => insertion fails iff gap < 0 (INVALID_SPEED, COLLISION is in the default
    //      insertionChecks set); succeeds (at departPos/departSpeed unmodified, since
    //      patchSpeed=false leaves `speed` -- not `nspeed` -- as the value actually used) iff
    //      there is no leader, or gap >= 0.
    //
    // Scoped out (not needed by this single-lane, no-stop, no-junction scenario; a literal
    // port would also cover these but they do not exist here): MSInsertionControl's
    // RANDOM/FREE depart procedures and full retry bookkeeping, multi-lane/lane-choice,
    // junction-foe and stop-line insertion checks, follower-gap/pedestrian/shadow-lane
    // checks, rail bidi handling, and the departPos<0 "measured from lane end" convention
    // (we use departPos directly since it is always >=0 here).
    private void InsertDepartingVehicles(double time)
    {
        // Group not-yet-inserted, not-arrived candidates whose depart time has come by their
        // target insertion lane (each candidate resolves independently; grouping is only to
        // process each lane's depart queue in isolation). Ordered by target lane id for
        // deterministic per-step processing order (this scenario has exactly one lane).
        var candidatesByLane = new SortedDictionary<string, List<VehicleRuntime>>(StringComparer.Ordinal);

        foreach (var v in _vehicles)
        {
            if (v.Inserted || v.Arrived || v.Def.Depart > time)
            {
                continue;
            }

            var route = _demand!.RoutesById[v.Def.RouteId];
            var edge = _network!.EdgesById[route.Edges[0]];
            var lane = edge.Lanes.First(l => l.Index == v.Def.DepartLaneIndex);

            if (!candidatesByLane.TryGetValue(lane.Id, out var list))
            {
                list = new List<VehicleRuntime>();
                candidatesByLane[lane.Id] = list;
            }

            list.Add(v);
        }

        foreach (var (laneId, candidates) in candidatesByLane)
        {
            // FIFO order: SUMO's depart queue is processed in departure order (ties broken by
            // the route file's vehicle order); List<T>.OrderBy is a stable sort, so ties
            // preserve _demand.Vehicles/_vehicles enumeration order (rou.xml file order).
            foreach (var v in candidates.OrderBy(c => c.Def.Depart))
            {
                if (!TryInsertOnLane(v, laneId))
                {
                    // MSLane::isInsertionSuccess fails for this candidate this step -> stop
                    // attempting further (later-departing) candidates on this lane this step;
                    // they queue behind it (FIFO). A vehicle inserted earlier in THIS loop
                    // (for an earlier candidate on the same lane) becomes the leader the next
                    // candidate is checked against, since TryInsertOnLane re-scans _vehicles
                    // fresh on each call.
                    break;
                }
            }
        }
    }

    // MSLane::isInsertionSuccess's leader-gap check only (see InsertDepartingVehicles' header
    // comment for the full derivation/scope). Returns true and performs the insertion iff
    // there is no leader on the lane or gap >= 0; otherwise leaves `v` untouched and returns
    // false (queued for a later step).
    private bool TryInsertOnLane(VehicleRuntime v, string laneId)
    {
        var insertPos = v.Def.DepartPos;

        // MSLane::getLastVehicleInformation / getLeader (same-lane branch): nearest already-
        // inserted, not-arrived vehicle with Pos >= insertPos on this lane -- includes any
        // vehicle inserted earlier THIS SAME step, since this re-scans _vehicles (the engine's
        // authoritative list) on every call rather than a stale snapshot.
        VehicleRuntime? leader = null;
        foreach (var other in _vehicles)
        {
            if (!other.Inserted || other.Arrived || other.LaneId != laneId)
            {
                continue;
            }

            if (other.Kinematics.Pos >= insertPos && (leader is null || other.Kinematics.Pos < leader.Kinematics.Pos))
            {
                leader = other;
            }
        }

        if (leader is not null)
        {
            // MSLane.cpp:1097 safeInsertionSpeed(veh, seen=-pos, leaders, speed): gap =
            // leaderBackPos + seen - egoMinGap = leaderBackPos - insertPos - egoMinGap.
            var leaderBackPos = leader.Kinematics.Pos - leader.VType.Length;
            var gap = leaderBackPos - insertPos - v.VType.MinGap;

            if (gap < 0)
            {
                // checkFailure's INVALID_SPEED/COLLISION path (MSLane.cpp:1098): no safe gap
                // yet -- do not insert this step.
                return false;
            }
        }

        var route = _demand!.RoutesById[v.Def.RouteId];
        var edge = _network!.EdgesById[route.Edges[0]];
        var lane = edge.Lanes.First(l => l.Index == v.Def.DepartLaneIndex);

        v.LaneId = lane.Id;
        v.Kinematics = new Kinematics
        {
            // patchSpeed=false (departSpeed explicitly given): the vehicle is inserted at its
            // requested departPos/departSpeed unchanged -- `nspeed` (the safe-insertion-speed
            // computation) is only used for the checkFailure gate above, never applied as the
            // actual insertion speed in this branch.
            Pos = v.Def.DepartPos,
            Speed = v.Def.DepartSpeed,
            LatOffset = 0.0,
        };

        // Rung 9a: resolve the FULL lane sequence for this vehicle's route (spanning internal/
        // junction lanes between edges), not just the departure edge/lane. For a single-edge
        // route this is exactly `[lane.Id]`, matching rungs 1-8 exactly (v.LaneId above already
        // equals LaneSequence[0]).
        v.LaneSequence = _network!.ResolveLaneSequence(route.Edges, v.Def.DepartLaneIndex);
        v.LaneSeqIndex = 0;

        v.Inserted = true;
        return true;
    }

    // B3: reroute-around-prolonged-blockage (DESIGN.md "Two futures" -- live-reactivity, seam-4
    // structural mutation, same discipline as a lane change: reads only start-of-step state (this
    // vehicle's own LaneSequence/LaneSeqIndex/kinematics, the immutable network, and the frozen
    // B1 obstacle store) plus the immutable network router, and mutates only THIS vehicle's own
    // LaneSequence/LaneSeqIndex/BlockedByObstacleSeconds/AvoidedEdges -- never another vehicle's
    // state, so this loop's outcome for one vehicle can never depend on another vehicle's
    // processing order this same step (order-independent, deterministic, parallel-ready even
    // though it runs as a plain sequential loop here). Called once per step, before
    // PlanMovements, so a vehicle that reroutes this step plans this SAME step against its new
    // route (see Run()'s comment).
    //
    // Inert-when-disabled (CLAUDE.md/DESIGN.md "keep every live-reactivity feature optional and
    // inert-when-absent"): returns immediately while RerouteThresholdSeconds is +infinity (the
    // default), so this method costs nothing and changes nothing for any scenario that does not
    // explicitly opt in.
    private void UpdateReroutes(double time, double dt)
    {
        if (double.IsInfinity(RerouteThresholdSeconds))
        {
            return;
        }

        _router ??= new NetworkRouter(_network!);

        foreach (var v in _vehicles)
        {
            if (!v.Inserted || v.Arrived || v.LaneId.StartsWith(':'))
            {
                // Not yet inserted/already arrived, or mid-junction on an internal lane -- a
                // reroute mid-junction has nowhere sensible to redirect from (ego is already
                // committed to the connection it is traversing), so this vehicle is simply
                // skipped this step; it will be reconsidered once it lands on its next normal
                // lane.
                continue;
            }

            var currentEdge = _network!.LanesById[v.LaneId].EdgeId;

            // Distinct FUTURE normal edges (route order, deduplicated), i.e. every normal edge
            // this vehicle's route still has left to traverse AFTER its current position,
            // excluding currentEdge itself and any internal/junction lane's edge id.
            var futureEdges = new List<string>();
            var futureEdgesSeen = new HashSet<string>(StringComparer.Ordinal);
            for (var i = v.LaneSeqIndex + 1; i < v.LaneSequence.Count; i++)
            {
                var seqLaneId = v.LaneSequence[i];
                if (seqLaneId.StartsWith(':'))
                {
                    continue;
                }

                var seqEdgeId = _network.LanesById[seqLaneId].EdgeId;
                if (seqEdgeId == currentEdge)
                {
                    continue;
                }

                if (futureEdgesSeen.Add(seqEdgeId))
                {
                    futureEdges.Add(seqEdgeId);
                }
            }

            // An active obstacle (StartTime <= time < EndTime) sitting on one of those future
            // edges -- reusing the B1 store exactly as ObstacleConstraint does, just asking "is
            // its edge one I still have to cross" instead of "is it ahead of me on my CURRENT
            // lane".
            string? blockedEdge = null;
            foreach (var obstacle in _obstacles.Values)
            {
                if (obstacle.StartTime > time || time >= obstacle.EndTime)
                {
                    continue;
                }

                var obstacleEdge = _network.LanesById[obstacle.LaneId].EdgeId;
                if (futureEdges.Contains(obstacleEdge))
                {
                    blockedEdge = obstacleEdge;
                    break;
                }
            }

            if (blockedEdge is null)
            {
                v.BlockedByObstacleSeconds = 0.0;
                continue;
            }

            v.BlockedByObstacleSeconds += dt;
            if (v.BlockedByObstacleSeconds < RerouteThresholdSeconds)
            {
                continue;
            }

            // Threshold reached -- recompute a route from HERE to the destination, avoiding this
            // blockage plus every edge already routed around earlier (so a blockage this vehicle
            // has already detoured past can never re-trigger a second reroute of the same edge).
            var destEdge = _network.LanesById[v.LaneSequence[^1]].EdgeId;
            var avoid = new HashSet<string>(v.AvoidedEdges, StringComparer.Ordinal) { blockedEdge };
            var newEdges = _router.Route(currentEdge, destEdge, avoid);

            if (newEdges is null)
            {
                // No alternate route exists (B4's dead-end/u-turn case, out of this rung's scope)
                // -- leave the vehicle on its current route; it will stop behind the obstacle via
                // the B1 ObstacleConstraint if/when it actually reaches the blocked edge.
                continue;
            }

            var currentRemainingEdges = new List<string>(futureEdges.Count + 1) { currentEdge };
            currentRemainingEdges.AddRange(futureEdges);
            if (newEdges.SequenceEqual(currentRemainingEdges))
            {
                // Router found no actual detour (cannot happen once blockedEdge is confirmed to
                // be one of currentRemainingEdges, but guarded per the briefing regardless).
                continue;
            }

            // newEdges[0] == currentEdge, and v is already on it at v.Kinematics.Pos -- resetting
            // LaneSeqIndex to 0 on the newly resolved sequence keeps the vehicle exactly where it
            // physically is (Kinematics.Pos untouched), just re-pointed at the new remaining lane
            // sequence from here onward. Structural mutation (route/LaneSequence replacement),
            // applied directly here rather than staged through MoveIntent -- this runs in its own
            // once-per-step phase outside Plan/Execute, exactly like DecideSpeedGainChanges' own
            // direct LaneId/accumulator writes, not mid-query shared state.
            var laneIndex = _network.LanesById[v.LaneId].Index;
            v.LaneSequence = _network.ResolveLaneSequence(newEdges, laneIndex);
            v.LaneSeqIndex = 0;
            v.AvoidedEdges.Add(blockedEdge);
            v.BlockedByObstacleSeconds = 0.0;
        }
    }

    // Plan phase (seam 1, parallel-safe): reads start-of-step world state (including the frozen
    // `neighbors` snapshot), writes only to the owning vehicle's own MoveIntent. No shared-state
    // writes, even single-threaded -- the rung-5 stop-transition decision (see ProcessNextStop)
    // is threaded through MoveIntent.StopUpdate rather than mutating v.Stops here, so this rule
    // holds even though a vehicle's own stop bookkeeping "changes" every step it is stopped.
    private void PlanMovements(LaneNeighborQuery neighbors, double time)
    {
        foreach (var v in _vehicles)
        {
            if (!v.Inserted || v.Arrived)
            {
                continue;
            }

            v.Intent = ComputeMoveIntent(v, neighbors, time);
        }
    }

    // Multi-constraint speed reducer (DESIGN.md seam 1): vPos is the MINIMUM over a collection
    // of constraints (leader car-following, junction/foe, stop line, and later shadow-lane
    // leaders), computed as a real collection/reduce even when the collection has only one
    // binding entry -- junctions/leaders slot in later without restructuring this method.
    // vPos then feeds MSCFModel.cpp's finalizeSpeed (KraussModel.FinalizeSpeed) for the
    // free-flow acceleration/deceleration bounding, exactly mirroring MSVehicle's plan-phase
    // call chain (per-constraint CF calls -> finalizeSpeed's vStop = MIN2(vPos,
    // processNextStop(vPos))).
    //
    // Plan/execute contract (DESIGN.md): this reads only start-of-step state off `v` (including
    // the front of v.Stops, never mutated here), the frozen `neighbors` snapshot, and the
    // immutable network/vType data -- no shared-state writes happen here; the resulting
    // StopTransition is handed back for ExecuteMoves to apply.
    private MoveIntent ComputeMoveIntent(VehicleRuntime v, LaneNeighborQuery neighbors, double time)
    {
        var lane = _network!.LanesById[v.LaneId];
        var dt = _config!.StepLength;
        // default.action-step-length=1 in rung 1's config, equal to dt; kept as its own value
        // (not silently assumed == dt) since MSCFModel.cpp divides by it separately from TS.
        var actionStepLengthSecs = _config.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        var laneVehicleMaxSpeed = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.VType);

        var constraints = new List<double>
        {
            // Leader car-following (MSCFModel_Krauss.cpp followSpeed -> MSCFModel.cpp
            // maximumSafeFollowSpeed): the REAL formula our resolved carFollowModel="Krauss"
            // uses -- NOT MSCFModel_KraussOrig1::vsafe (removed; see rung-4 briefing, that
            // formula is dead code once a real leader exists). No leader => +infinity
            // (non-binding), matching a gap=+infinity KraussOrig1 vsafe call's short-circuit
            // but via the real code path: simply contribute nothing when there is no leader.
            LeaderFollowSpeedConstraint(v, neighbors, dt),

            // Desired free-flow speed (MSLane::getVehicleMaxSpeed): lane speed limit adapted
            // by this vehicle's speedFactor, capped by its vType maxSpeed.
            laneVehicleMaxSpeed,

            // Stop line (rung 5): MSVehicle.cpp's planMoveInternal "process stops" block
            // (~lines 2467-2540), non-waypoint arm only. +infinity (non-binding) once reached
            // (the source's own approach-block condition `!stop.reached || (waypoint &&
            // keepStopping())` is simply false for a non-waypoint stop that IS reached) or when
            // there is no stop at all.
            StopLineConstraint(v, dt, actionStepLengthSecs),

            // Red light (rung 10): MSVehicle.cpp's planMoveInternal per-link loop (~2641-2666,
            // 2734), yellowOrRed arm only. +infinity (non-binding) when this lane's outgoing
            // connection is not TL-controlled, or its light is green, at the time this Plan/
            // Execute cycle's result will be observed (see RedLightConstraint's own comment on
            // why that is `time + dt`, not `time`).
            RedLightConstraint(v, lane, time, dt, actionStepLengthSecs),

            // Priority-junction yielding (rung 9b-ii/iii): MSLink's right-of-way gate (stop-line
            // brake while a higher-priority foe still approaches) plus MSVehicle::
            // adaptToJunctionLeader (car-following against a foe already on the junction).
            // +infinity (non-binding) whenever ego has no upcoming/current junction link, or
            // that link's foes are all cleared/absent -- see JunctionYieldConstraint's own
            // comment for the full derivation and its determinism note.
            JunctionYieldConstraint(v, _vehicles, dt, actionStepLengthSecs),

            // B1: external obstacle (DESIGN.md "Two futures" -- a live, non-SUMO input, not a
            // ported SUMO code path). Modeled as one more virtual stopped leader reusing the same
            // KraussModel.FollowSpeed leader car-following formula as LeaderFollowSpeedConstraint
            // above -- the multi-constraint reducer does not care where a constraint's speed came
            // from. +infinity (non-binding) whenever _obstacles is empty or none is active/ahead
            // on this lane -- this is the inert-when-absent guard: an empty store makes this a
            // no-op Min term, leaving every existing (obstacle-free) parity scenario untouched.
            ObstacleConstraint(v, time),
        };

        var vPos = constraints.Min();

        // MSCFModel.cpp:191 finalizeSpeed: `vStop = MIN2(vPos, veh->processNextStop(vPos))`.
        // ProcessNextStop reads only the front stop's START-OF-STEP snapshot (Reached/
        // RemainingDuration) and returns the transition to apply at Execute -- never mutates.
        var (processedVelocity, stopUpdate) = ProcessNextStop(v, vPos, actionStepLengthSecs);
        var vStop = Math.Min(vPos, processedVelocity);

        var newSpeed = KraussModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs);

        return new MoveIntent
        {
            NewSpeed = newSpeed,
            LatOffset = 0.0,
            StopUpdate = stopUpdate,
        };
    }

    // MSVehicle.cpp's planMoveInternal "process stops" block (~2467-2540), non-waypoint
    // (stop.getSpeed()==0) arm only: newStopDist = seen + endPos - lane->getLength(), which on a
    // single lane (seen = laneLength - pos) collapses to `endPos + NUMERICAL_EPS - pos`;
    // stopSpeed = MAX2(cfModel.stopSpeed(this, getSpeed(), newStopDist), vMinComfortable) where
    // vMinComfortable = cfModel.minNextSpeed(getSpeed()) (line 2191). Non-binding (+infinity)
    // once the stop is reached (matches the source's own approach-block guard) or absent.
    private static double StopLineConstraint(VehicleRuntime v, double dt, double actionStepLengthSecs)
    {
        if (v.Stops.Count == 0)
        {
            return double.PositiveInfinity;
        }

        var stop = v.Stops.Peek();
        if (stop.Reached || stop.LaneId != v.LaneId)
        {
            return double.PositiveInfinity;
        }

        var newStopDist = stop.EndPos + KraussModel.NumericalEps - v.Kinematics.Pos;
        var vMinComfortable = KraussModel.MinNextSpeed(v.Kinematics.Speed, v.VType, dt);
        var stopSpeed = KraussModel.StopSpeed(newStopDist, v.Kinematics.Speed, v.VType, dt, actionStepLengthSecs);

        return Math.Max(stopSpeed, vMinComfortable);
    }

    // Ported from MSVehicle.cpp's planMoveInternal per-link loop, yellowOrRed arm only
    // (~lines 2630-2666 for laneStopOffset/stopDist, ~line 2734 for the stopSpeed call itself).
    // Non-binding (+infinity) when this lane's outgoing connection is not TL-controlled, or the
    // controlling link's state is green, at the time this Plan/Execute cycle's result will
    // actually be observed.
    //
    // Timing note (why `time + dt`, not `time`): SUMO's MSNet::simulationStep processes, for its
    // own internal clock reading T, `myLogics->check2Switch(T)` (the TLS phase switch, if T is a
    // scheduled switch time) THEN `myEdges->planMovements(T)`/`executeMovements(T)` (using that
    // now-current state) BEFORE `writeOutput()` tags the just-computed result as T (MSNet.cpp's
    // postMoveStep, called at the end of the same simulationStep(); myStep is only incremented
    // afterward). Our engine's loop instead EMITS the trajectory point tagged `time` at the TOP
    // of its iteration (using the PREVIOUS iteration's Plan/Execute result), then Plans/Executes
    // for `time` itself, whose result becomes the trajectory emitted at the NEXT iteration's
    // `time + dt`. So the movement this Plan phase is computing right now corresponds exactly to
    // SUMO's internal clock reading `time + dt`, not `time` -- the TL state must be sampled there
    // (this is the one place a scenario boundary (t=29 -> t=30, red -> green) actually falsifies
    // the naive "just use `time`" reading: the golden's t=30 row already shows free-flow
    // acceleration through the junction, i.e. green was already in effect for the Plan/Execute
    // that produced it).
    private double RedLightConstraint(VehicleRuntime v, Lane lane, double time, double dt, double actionStepLengthSecs)
    {
        if (!_network!.TryGetTlControlledConnection(lane.EdgeId, lane.Index, out var connection))
        {
            return double.PositiveInfinity;
        }

        var tlLogic = _network.TlLogicsById[connection.Tl!];
        var linkIndex = connection.LinkIndex!.Value;
        var evalTime = time + dt;
        var state = TrafficLightState.GetLinkState(tlLogic, linkIndex, evalTime);

        if (!TrafficLightState.IsRedOrYellow(state))
        {
            return double.PositiveInfinity;
        }

        var seen = lane.Length - v.Kinematics.Pos;

        // stopDecel (MSVehicle.cpp:2645): yellowOrRed => MAX2(MIN2(gTLSYellowMinDecel,
        // emergencyDecel), maxDecel) -- since MAX2 floors it at maxDecel and this vType's
        // gTLSYellowMinDecel default (3.0) < emergencyDecel (9.0) < ... is always dominated by
        // that floor, stopDecel collapses to exactly vType.Decel for every vType reachable here
        // (see the rung-10 briefing); ported as the resolved constant rather than the unreached
        // MIN2/MAX2 machinery.
        var stopDecel = v.VType.Decel;
        var brakeDist = KraussModel.BrakeGap(v.Kinematics.Speed, stopDecel, headwayTime: 0.0, dt);
        var canBrakeBeforeLaneEnd = seen >= brakeDist;

        // majorStopOffset (MSVehicle.cpp:2642): MAX2(jmStoplineGap default
        // DIST_TO_STOPLINE_EXPECT_PRIORITY=1.0, lane.getVehicleStopOffset(this)=0 -- no
        // vClass-specific stop offset modeled) = 1.0.
        const double majorStopOffset = 1.0;
        const double positionEps = 0.1;

        var laneStopOffset = majorStopOffset;
        if (canBrakeBeforeLaneEnd)
        {
            // MSVehicle.cpp:2661: avoid emergency braking if possible.
            laneStopOffset = Math.Min(laneStopOffset, seen - brakeDist);
        }

        laneStopOffset = Math.Max(positionEps, laneStopOffset);
        var stopDist = Math.Max(0.0, seen - laneStopOffset);

        return KraussModel.StopSpeed(stopDist, v.Kinematics.Speed, v.VType, dt, actionStepLengthSecs);
    }

    // sumo/src/microsim/MSLink.cpp's POSITION_EPS (used throughout its getLeaderInfo/
    // adaptToJunctionLeader call chain, e.g. MSVehicle.cpp:3228's `seen - lane->getLength() -
    // POSITION_EPS`) -- distinct from KraussModel.NumericalEps (0.001, a different constant).
    private const double PositionEps = 0.1;

    // Rung 9b-ii/iii: priority-junction yielding. Ported from two SUMO call sites that only
    // ever fire for a link this vehicle's own request row must yield to
    // (JunctionRequest.RespondsTo, MSLink::myResponse / "myHasFoes"):
    //   - MSLink::opened()'s stop-line gate (approaching foe still on its own approach lane:
    //     the ego link is not yet "open", so ego must be able to stop at the stop line) --
    //     modeled here as a straight stopSpeed brake to the approach lane's end
    //     (approachLen - pos - POSITION_EPS), matching the verified 9.433/4.933 trajectory.
    //   - MSVehicle::adaptToJunctionLeader (MSVehicle.cpp:3205-3307, Euler branch): once the
    //     foe has actually entered its own internal lane, MSLink's opened() check no longer
    //     blocks entry (foe becomes a link-leader instead) -- ego treats it as a car-following
    //     leader superimposed at the junction's crossing point.
    // These are mutually exclusive per foe link (never MIN'd together for the same foe): a
    // foe is classified as exactly one of on-junction / approaching / cleared from its FROZEN
    // start-of-step lane/position (the same `allVehicles` snapshot LaneNeighborQuery.Build
    // reads -- CLAUDE.md rule 2, never a foe's already-updated position this step).
    //
    // Determinism (CLAUDE.md rule 5 / this rung's briefing): the yield decision is derived
    // purely from the STATIC <request> priority matrix (parsed once from net.xml, unaffected
    // by runtime state) plus this frozen start-of-step snapshot -- there is no "first to
    // arrive wins" race and no dependency on _vehicles' iteration/processing order, so the
    // result is identical regardless of parallel/thread scheduling.
    //
    // +infinity (non-binding) when ego has no upcoming/current internal-lane link in its
    // LaneSequence (already past its own junction, or its route crosses none), that link has
    // no <request> row, or every foe link it must yield to either has no geometric conflict
    // recorded (JunctionConflict) or no actual foe vehicle present/still relevant.
    private double JunctionYieldConstraint(VehicleRuntime v, IReadOnlyList<VehicleRuntime> allVehicles, double dt, double actionStepLengthSecs)
    {
        // Step 1: ego's own upcoming/current junction link -- the first internal lane in
        // v.LaneSequence at or after LaneSeqIndex. A lane already passed is simply never found
        // by this forward-only scan (LaneSeqIndex has already advanced beyond it), which is
        // exactly the "already passed -> +infinity" case the briefing calls for.
        var egoLinkSeqIndex = -1;
        string? egoInternalLaneId = null;
        for (var i = v.LaneSeqIndex; i < v.LaneSequence.Count; i++)
        {
            if (_network!.LinkByInternalLane.ContainsKey(v.LaneSequence[i]))
            {
                egoLinkSeqIndex = i;
                egoInternalLaneId = v.LaneSequence[i];
                break;
            }
        }

        if (egoInternalLaneId is null)
        {
            return double.PositiveInfinity;
        }

        var (junction, egoLink) = _network!.LinkByInternalLane[egoInternalLaneId];
        var request = junction.Requests.FirstOrDefault(r => r.Index == egoLink.Index);
        if (request is null)
        {
            return double.PositiveInfinity;
        }

        var egoLane = _network.LanesById[egoInternalLaneId];
        // The lane immediately before ego's internal lane in its route. Null only if the
        // internal lane is the very first element of LaneSequence -- which cannot happen for a
        // vehicle inserted on a normal lane (egoLinkSeqIndex >= 1 then), so it is used only in
        // the !egoOnInternal branches below, where it is always non-null. Guarded so a future
        // laneless/mid-junction insertion can't index -1 here.
        var approachLane = egoLinkSeqIndex >= 1
            ? _network.LanesById[v.LaneSequence[egoLinkSeqIndex - 1]]
            : null;
        var egoOnInternal = v.LaneId == egoInternalLaneId;

        var constraint = double.PositiveInfinity;
        for (var j = 0; j < junction.IntLanes.Count; j++)
        {
            if (j == egoLink.Index || !request.RespondsTo(j))
            {
                continue;
            }

            var conflict = junction.Conflicts.FirstOrDefault(c => c.EgoLink == egoLink.Index && c.FoeLink == j);
            if (conflict is null)
            {
                // No geometric crossing recorded for this foe link -- nothing to yield to.
                continue;
            }

            var foeInternalLaneId = junction.IntLanes[j];
            var foe = FindFoeVehicle(v, allVehicles, foeInternalLaneId);
            if (foe is null)
            {
                continue;
            }

            var foeInternalSeqIndex = IndexOfLane(foe.LaneSequence, foeInternalLaneId);

            double thisConstraint;
            if (foe.LaneId == foeInternalLaneId)
            {
                // On-junction: MSVehicle::adaptToJunctionLeader.
                thisConstraint = AdaptToJunctionLeader(v, egoLane, approachLane, egoOnInternal, conflict, foe, dt, actionStepLengthSecs);
            }
            else if (foeInternalSeqIndex > foe.LaneSeqIndex)
            {
                // Approaching (foe hasn't reached its own internal lane yet): the stop-line
                // yield only guards ENTRY onto ego's own internal lane -- once ego has already
                // been granted entry (egoOnInternal), it is no longer gated by this foe's
                // approach state.
                thisConstraint = egoOnInternal
                    ? double.PositiveInfinity
                    : KraussModel.StopSpeed(
                        approachLane!.Length - v.Kinematics.Pos - PositionEps,
                        v.Kinematics.Speed, v.VType, dt, actionStepLengthSecs);
            }
            else
            {
                // Cleared: foe already past its internal lane.
                thisConstraint = double.PositiveInfinity;
            }

            constraint = Math.Min(constraint, thisConstraint);
        }

        return constraint;
    }

    // MSVehicle::adaptToJunctionLeader (sumo/src/microsim/MSVehicle.cpp:3205-3307), Euler
    // branch only, and the gap formula it is fed (MSLink::getLeaderInfo, MSLink.cpp:1647).
    // `egoLane` is ego's own upcoming/current internal lane (the link this constraint was
    // raised for); `approachLane` is the lane immediately before it in ego's LaneSequence.
    // `foe` is already confirmed to be ON its own internal lane (foe.LaneId equals the
    // conflict's foe internal lane, i.e. `_network.LanesById[foe.LaneId]` below IS that lane)
    // by JunctionYieldConstraint before calling in.
    private double AdaptToJunctionLeader(
        VehicleRuntime ego,
        Lane egoLane,
        Lane? approachLane,
        bool egoOnInternal,
        JunctionConflict conflict,
        VehicleRuntime foe,
        double dt,
        double actionStepLengthSecs)
    {
        var foeLane = _network!.LanesById[foe.LaneId];

        // MSVehicle.cpp:3428/3473's `seen`: distance from ego's front to the end of the exit
        // link it is currently driving toward -- ego's OWN internal lane (egoLane) is that
        // exit link, whether ego is still approaching it or already on it.
        var seen = egoOnInternal
            ? egoLane.Length - ego.Kinematics.Pos
            : (approachLane!.Length - ego.Kinematics.Pos) + egoLane.Length;

        var distToCrossing = seen - conflict.EgoLengthBehindCrossing;
        var foeDistToCrossing = foeLane.Length - conflict.FoeLengthBehindCrossing;

        var leaderBack = foe.Kinematics.Pos - foe.VType.Length;
        var leaderBackDist = foeDistToCrossing - leaderBack;
        var foeCrossingWidth = conflict.FoeConflictSize;

        var gap = distToCrossing - ego.VType.MinGap - leaderBackDist - foeCrossingWidth;

        // MSVehicle.cpp:3219-3222: Euler (gSemiImplicitEulerUpdate=true, phase 1's only
        // integration mode) initializes vsafeLeader to 0, not -DBL_MAX.
        var vsafeLeader = 0.0;
        if (gap >= 0)
        {
            vsafeLeader = KraussModel.FollowSpeed(ego.Kinematics.Speed, gap, foe.Kinematics.Speed, foe.VType.Decel, ego.VType, dt);
        }
        else
        {
            // MSVehicle.cpp:3225-3228: leaderInfo.first != this is always true here (foe is a
            // distinct vehicle, never the ego "pedestrian" self-reference).
            vsafeLeader = KraussModel.StopSpeed(seen - egoLane.Length - PositionEps, ego.Kinematics.Speed, ego.VType, dt, actionStepLengthSecs);
        }

        if (distToCrossing >= 0)
        {
            // MSVehicle.cpp:3240-3280. leaderInfo.first == this (pedestrian) and
            // leaderInfo.second == -DBL_MAX (continuation-lane/opposite-direction foe) never
            // occur for this rung's foe-vehicle-on-a-plain-internal-lane case, so only the
            // final "else" branch (lines 3260-3280) is reachable here.
            var vStop = KraussModel.StopSpeed(distToCrossing - ego.VType.MinGap, ego.Kinematics.Speed, ego.VType, dt, actionStepLengthSecs);
            var leaderDistToCrossing = distToCrossing - gap;
            var leaderPastCPTime = leaderDistToCrossing / Math.Max(foe.Kinematics.Speed, KraussModel.HaltingSpeed);
            var vFinal = Math.Max(ego.Kinematics.Speed, (2.0 * (distToCrossing - ego.VType.MinGap) / leaderPastCPTime) - ego.Kinematics.Speed);
            var v2 = ego.Kinematics.Speed + KraussModel.Accel2Speed((vFinal - ego.Kinematics.Speed) / leaderPastCPTime, dt);
            vsafeLeader = Math.Max(vsafeLeader, Math.Min(v2, vStop));
        }

        return vsafeLeader;
    }

    // MSLane::getInternalFollowingLane-adjacent lookup: the (at most one, in this rung's
    // scope) OTHER vehicle whose route crosses the given internal lane -- ported from
    // MSLink::getLeaderInfo's foeLane vehicle scan (MSLink.cpp's per-foeLane loop), simplified
    // to this rung's single-foe-vehicle-per-link scenario (no queueing/multiple-foes
    // tie-break is modeled; see the briefing's scope note). Excludes ego itself, and any
    // vehicle not yet inserted or already arrived (frozen `allVehicles` snapshot).
    private static VehicleRuntime? FindFoeVehicle(VehicleRuntime ego, IReadOnlyList<VehicleRuntime> allVehicles, string foeInternalLaneId)
    {
        foreach (var other in allVehicles)
        {
            if (ReferenceEquals(other, ego) || !other.Inserted || other.Arrived)
            {
                continue;
            }

            if (other.LaneSequence.Contains(foeInternalLaneId))
            {
                return other;
            }
        }

        return null;
    }

    // IReadOnlyList<string> has no IndexOf overload -- a tiny manual scan avoids materializing
    // a copy just to find the internal lane's position in a vehicle's LaneSequence.
    private static int IndexOfLane(IReadOnlyList<string> laneSequence, string laneId)
    {
        for (var i = 0; i < laneSequence.Count; i++)
        {
            if (laneSequence[i] == laneId)
            {
                return i;
            }
        }

        return -1;
    }

    // MSVehicle::processNextStop (sumo/src/microsim/MSVehicle.cpp:1613-1897),
    // non-waypoint (stop.getSpeed()==0) arm only, Euler branch only (the ballistic
    // `getSpeed() - getMaxDecel()` arm is dead per phase-1 CLAUDE.md/DESIGN.md). Reads only the
    // front stop's START-OF-STEP snapshot; returns (the value processNextStop would have
    // returned, the StopTransition for ExecuteMoves to apply -- null if nothing changes, exactly
    // like the source's implicit "no side effect on stop.reached" paths).
    private static (double ReturnedVelocity, StopTransition? Transition) ProcessNextStop(
        VehicleRuntime v,
        double currentVelocity,
        double actionStepLengthSecs)
    {
        if (v.Stops.Count == 0)
        {
            // MSVehicle.cpp:1614-1617: myStops.empty() -> return currentVelocity.
            return (currentVelocity, null);
        }

        var stop = v.Stops.Peek();
        if (stop.LaneId != v.LaneId)
        {
            // MSVehicle.cpp:1762's `stop.edge == myCurrEdge` guard -- not on the stop's edge/lane
            // yet; rung 5's single-lane scenario never exercises this, guarded for safety.
            return (currentVelocity, null);
        }

        if (stop.Reached)
        {
            // MSVehicle.cpp:1627-1628: stop.duration -= getActionStepLength() (every call while
            // reached, BEFORE the keepStopping() check).
            var remaining = stop.RemainingDuration - actionStepLengthSecs;

            // MSVehicle.cpp:1578-1588 keepStopping(): non-waypoint (getSpeed()==0) simplifies to
            // `duration > 0` (no triggered/collision/parking flags modeled in rung 5).
            var keepStopping = remaining > 0;

            if (!keepStopping)
            {
                // MSVehicle.cpp:1663-1679: resumeFromStopping() pops the stop; not a railway, so
                // falls through to the function's tail `return currentVelocity;` (line 1896)
                // unchanged -- the vehicle plans freely again from here.
                return (currentVelocity, new StopTransition(Resume: true, Reached: false, RemainingDuration: 0.0));
            }

            // MSVehicle.cpp:1731-1739, Euler branch: still holding -> return 0.
            return (0.0, new StopTransition(Resume: false, Reached: true, RemainingDuration: remaining));
        }

        // MSVehicle.cpp:1794-1808: reachedThreshold = stop.getReachedThreshold() - NUMERICAL_EPS;
        // getReachedThreshold() (MSStop.cpp:64) is pars.startPos for a normal (non-opposite) lane
        // stop.
        var reachedThreshold = stop.StartPos - KraussModel.NumericalEps;
        if (v.Kinematics.Pos >= reachedThreshold
            && currentVelocity <= 0.0 + KraussModel.HaltingSpeed
            && v.LaneId == stop.LaneId)
        {
            // MSVehicle.cpp:1808/1824: stop.reached = true; stop.duration = getMinDuration(time)
            // -- no until/ended modeled, so getMinDuration is just the configured duration
            // (MSStop.cpp:134-147's final `else` arm).
            return (currentVelocity, new StopTransition(Resume: false, Reached: true, RemainingDuration: stop.Duration));
        }

        // MSVehicle.cpp:1896: return currentVelocity; (no change to stop.reached this step).
        return (currentVelocity, null);
    }

    // MSLane::getLeader's gap formula (MSLane.cpp:2817/2841): gap = leaderBackPos -
    // egoMinGap - egoPos, where leaderBackPos = leaderPos - leaderLength. predMaxDecel is the
    // leader's OWN decel (MSVehicle::getCurrentApparentDecel(), which for our phase-1 vTypes
    // -- no apparent-decel override beyond the vType default -- equals the leader's vType
    // decel). Returns +infinity (non-binding) when ego has no leader on its lane.
    private static double LeaderFollowSpeedConstraint(VehicleRuntime ego, LaneNeighborQuery neighbors, double dt)
    {
        var leader = neighbors.GetLeader(ego);
        if (leader is null)
        {
            return double.PositiveInfinity;
        }

        var leaderBackPos = leader.Kinematics.Pos - leader.VType.Length;
        var gap = leaderBackPos - ego.VType.MinGap - ego.Kinematics.Pos;

        return KraussModel.FollowSpeed(
            egoSpeed: ego.Kinematics.Speed,
            gap: gap,
            predSpeed: leader.Kinematics.Speed,
            predMaxDecel: leader.VType.Decel,
            vType: ego.VType,
            dt: dt);
    }

    // B1: external-obstacle constraint. Treats the nearest active obstacle ahead of `v` on its
    // current lane as a virtual stopped leader (predSpeed=0), reusing the exact same
    // KraussModel.FollowSpeed leader-following formula LeaderFollowSpeedConstraint uses for a
    // real vehicle leader -- so a follower approaching an obstacle brakes and settles at the same
    // Krauss steady gap it would hold behind a stopped real vehicle (verified against scenario
    // 13-stopped-leader's golden: follower front settles at 242.499 = obstacle back 245 - minGap
    // 2.5 - NUMERICAL_EPS 0.001). predMaxDecel is passed as the ego's OWN decel (`v.VType.Decel`):
    // with predSpeed=0, KraussModel.BrakeGap(0, ...) is 0 regardless of the decel argument, so
    // this value is never actually used by the formula for a static obstacle -- it is supplied
    // only because FollowSpeed's signature requires *some* vType-shaped decel, and the ego's own
    // is a harmless, always-defined choice.
    //
    // Timing: obstacle activity ([StartTime, EndTime)) is evaluated at the SAME `time` this whole
    // Plan phase reads every other piece of start-of-step state (v's own kinematics, the frozen
    // `neighbors` snapshot) -- an obstacle is just another neighbor read from the frozen
    // start-of-step world, never one whose activity window is re-checked mid-step.
    //
    // +infinity (non-binding, the inert-when-absent guard) when `_obstacles` is empty, none is
    // active at `time`, none is on v's current lane, or none is still ahead of v (back position
    // >= v's own position) -- an empty store trivially falls through this loop with the seed
    // value untouched.
    private double ObstacleConstraint(VehicleRuntime v, double time)
    {
        ExternalObstacle? nearest = null;
        var nearestBack = double.PositiveInfinity;

        foreach (var obstacle in _obstacles.Values)
        {
            if (obstacle.StartTime > time || time >= obstacle.EndTime || obstacle.LaneId != v.LaneId)
            {
                continue;
            }

            var back = obstacle.FrontPos - obstacle.Length;
            if (back < v.Kinematics.Pos)
            {
                continue;
            }

            if (back < nearestBack)
            {
                nearestBack = back;
                nearest = obstacle;
            }
        }

        if (nearest is null)
        {
            return double.PositiveInfinity;
        }

        var gap = nearestBack - v.VType.MinGap - v.Kinematics.Pos;

        return KraussModel.FollowSpeed(
            egoSpeed: v.Kinematics.Speed,
            gap: gap,
            predSpeed: 0.0,
            predMaxDecel: v.VType.Decel,
            vType: v.VType,
            dt: _config!.StepLength);
    }

    // Execute phase: apply each vehicle's own MoveIntent and integrate position. Euler per
    // config.sumocfg's step-method.ballistic=false: pos += newSpeed * dt (integration method
    // is a config flag per DESIGN.md, not hard-coded -- Ballistic support is a later task).
    private void ExecuteMoves(double dt)
    {
        foreach (var v in _vehicles)
        {
            if (!v.Inserted || v.Arrived)
            {
                continue;
            }

            v.Kinematics.Speed = v.Intent.NewSpeed;
            v.Kinematics.Pos += v.Intent.NewSpeed * dt;
            v.Kinematics.LatOffset = v.Intent.LatOffset;

            // Rung 5: apply the plan phase's proposed stop-queue update (Engine.ProcessNextStop).
            // This is the only place v.Stops is ever mutated (CLAUDE.md rule 3).
            if (v.Intent.StopUpdate is { } stopUpdate)
            {
                if (stopUpdate.Resume)
                {
                    v.Stops.Dequeue();
                }
                else
                {
                    var stop = v.Stops.Peek();
                    stop.Reached = stopUpdate.Reached;
                    stop.RemainingDuration = stopUpdate.RemainingDuration;
                }
            }

            // Rung 9a: lane-sequence traversal. Generalizes rungs 1-8's single-lane "reached the
            // end -> arrived" check into a route that may cross several lane boundaries
            // (including internal/junction lanes) per step -- a `while` loop rather than a
            // single `if`, since a step could in principle span a very short lane fully (not
            // exercised in this scenario, where each lane boundary is crossed exactly once, but
            // matching the briefing's guard against exactly that). Carries the lane-relative
            // remainder of Pos forward across the boundary (pos -= currentLane.Length) rather
            // than clamping, so downstream lane pos (e.g. golden's :J_2_0 pos=9.68, JE_0
            // pos=12.37) matches exactly. Once there is no next lane in LaneSequence, the
            // vehicle has run off the end of its route and is marked Arrived -- stops being
            // planned/executed/emitted from the NEXT step onward (the step in which it crosses
            // the line is still emitted beforehand, since EmitTrajectory runs at the top of the
            // loop before Plan/Execute -- this reproduces golden.fcd.xml's presence set exactly:
            // present through the last in-bounds step, absent afterward, with no extra "arrival"
            // row). For a single-edge route (LaneSequence.Count == 1) this collapses to exactly
            // the old ArrivalPos check: the first "no next lane" hit marks Arrived immediately.
            while (!v.Arrived)
            {
                var currentLane = _network!.LanesById[v.LaneId];
                if (v.Kinematics.Pos < currentLane.Length)
                {
                    break;
                }

                if (v.LaneSeqIndex + 1 >= v.LaneSequence.Count)
                {
                    v.Arrived = true;
                    break;
                }

                v.Kinematics.Pos -= currentLane.Length;
                v.LaneSeqIndex++;
                v.LaneId = v.LaneSequence[v.LaneSeqIndex];
            }

            // Rung 8b/A2: keep-right and speed-gain lane changes are no longer decided here --
            // both now run in the post-move DecideSpeedGainChanges phase (see Run()'s comment and
            // that method's header comment for why keep-right moved out of Plan/MoveIntent).
        }
    }

    // Rung A2 (+ rung 8b, moved here -- see the CORRECTED-ORDERING note below): the two LC2013
    // lane-change sub-decisions, both ported from
    // sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp's SINGLE `_wantsChange` function -- keep-right
    // (~1721-1794, ~1743-1748's neighLead adjustment) and speed-gain-LEFT (~1548-1549
    // thisLaneVSafe/neighLaneVSafe, ~1682 relativeGain, ~1818-1864 accumulate/decay/threshold).
    // Runs as its OWN post-move phase (see Run()'s comment) over a FRESH neighbor query built
    // from the now-settled post-move kinematics -- this is the one place a "plan" reads state
    // that changed THIS step, which is correct here precisely because it mirrors SUMO's
    // changeLanes phase running after executeMovements (CLAUDE.md rule 2 is about a follower
    // never seeing ITS LEADER'S update mid-CAR-FOLLOWING-step; this is a separate, later phase by
    // design).
    //
    // CORRECTED-ORDERING note (why keep-right moved here from Plan/MoveIntent): rung 8b originally
    // ran keep-right in the pre-move Plan phase, documented as safe because every scenario up to
    // that point had an empty right lane (no neighLead-gap dependence). Rung A2's briefing
    // inherited that assumption for the keep-right RETURN in scenario 12 -- but the passed slow
    // leader is briefly still AHEAD of the follower on the (now-right) lane for a couple of steps
    // after the left change, which DOES bind the :1743-1748 neighLead adjustment, and that
    // adjustment needs the POST-move gap (real SUMO's `_wantsChange` -- for BOTH keep-right and
    // speed-gain -- runs once per vehicle from MSLaneChanger's post-executeMovements
    // `changeLanes()` pass, never from planMovements). Running keep-right in Plan gave the RIGHT
    // answer only by coincidence (position-independent math) for every earlier scenario;
    // scenario 12 exposes the coincidence and CLAUDE.md rule 1 (match what the vendored source
    // actually does, over a flagged-as-unverified briefing guess -- see RUNGA2.md's own "VERIFY...
    // untested" caveat) requires moving it here. Verified NOT to change rung 8a/8b's byte-identical
    // trajectory (see this rung's PR notes): with no right-lane neighbor, neither the accumulator
    // math nor `newSpeed`/`v.Kinematics.Pos` differ between pre- and post-move reads.
    //
    // Target-lane safety veto (A2-iii) is a minimal-but-faithful brake-gap check
    // (MSCFModel::getSecureGap), not the full checkBlocking/blocker-gap machinery -- see
    // IsTargetLaneSafe's own comment.
    private void DecideSpeedGainChanges(double dt)
    {
        const double relGainNormalizationMinSpeed = 10.0; // MSLCM_LC2013.cpp RELGAIN_NORMALIZATION_MIN_SPEED
        const double changeProbThresholdLeft = 0.2; // ctor: (0.2/mySpeedGainParam), default mySpeedGainParam=1
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        // Built ONCE per step, from the now-settled post-move positions every vehicle's
        // keep-right/speed-gain decision below reads -- SUMO's changeLanes phase sees all
        // vehicles already moved this step (MSNet.cpp:784/790/796), so this is the correct (and
        // only) frozen snapshot for this phase, distinct from PlanMovements' pre-move `neighbors`
        // snapshot.
        var postMoveNeighbors = LaneNeighborQuery.Build(_vehicles);

        foreach (var v in _vehicles)
        {
            if (!v.Inserted || v.Arrived)
            {
                continue;
            }

            // Keep-right (rung 8b) evaluated FIRST, against this iteration's starting lane; may
            // update v.LaneId/v.KeepRightProbability directly (own comment: same reasoning as the
            // speed-gain veto below for why a direct write here still honors CLAUDE.md rule 3).
            ApplyKeepRightDecision(v, postMoveNeighbors, dt);

            var lane = _network!.LanesById[v.LaneId];
            var edge = _network.EdgesById[lane.EdgeId];

            // Left neighbor = same edge, index+1 (no neighbor on the leftmost lane) -- this
            // guard is the INERT case CLAUDE.md's briefing calls for: single-lane rungs, and any
            // vehicle already on the leftmost lane (e.g. this same follower once it has already
            // changed left), leave SpeedGainProbability untouched and never fire a change.
            var leftLane = edge.Lanes.FirstOrDefault(l => l.Index == lane.Index + 1);
            if (leftLane is null)
            {
                continue;
            }

            var vMax = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.VType);
            var neighVMax = KraussModel.LaneVehicleMaxSpeed(leftLane.Speed, v.VType);

            // MSLCM_LC2013.cpp:1109-1136's best-lanes continuation distance, simplified per the
            // A2 briefing's scope note to this single-edge dead-end scenario: laneLength minus
            // the vehicle's (post-move) position on it. Large enough here (~2000/~1865) that it
            // never binds the no-leader stop-speed fallback or the `>20` gate below -- a full
            // multi-edge best-lanes port is out of scope until a scenario actually needs it.
            var currentDist = lane.Length - v.Kinematics.Pos;
            var neighDist = leftLane.Length - v.Kinematics.Pos;

            var leader = postMoveNeighbors.GetLeader(v);
            var thisLaneVSafe = Math.Min(vMax, AnticipateFollowSpeed(v, leader, currentDist, dt));

            var neighLead = postMoveNeighbors.GetNeighborLeader(v, leftLane.Id);
            var neighLaneVSafe = Math.Min(neighVMax, AnticipateFollowSpeed(v, neighLead, neighDist, dt));

            // :1682
            var relativeGain = (neighLaneVSafe - thisLaneVSafe) / Math.Max(neighLaneVSafe, relGainNormalizationMinSpeed);

            var speedGainProbability = v.SpeedGainProbability;
            if (thisLaneVSafe > neighLaneVSafe)
            {
                // :1820-1824: this lane is (strictly) better -> decay toward 0.
                if (speedGainProbability > 0)
                {
                    speedGainProbability *= Math.Pow(0.5, actionStepLengthSecs);
                }
            }
            else if (thisLaneVSafe == neighLaneVSafe)
            {
                // :1825-1828
                if (speedGainProbability > 0)
                {
                    speedGainProbability *= Math.Pow(0.8, actionStepLengthSecs);
                }
            }
            else
            {
                // :1829-1831: left lane is better -> accumulate.
                speedGainProbability += actionStepLengthSecs * relativeGain;
            }

            // MSLCM_LC2013.cpp:1020 -- numerical-stability truncation applied to the accumulator
            // (SUMO calls this once per step, in prepareStep, ahead of _wantsChange; ported here
            // immediately after the accumulate/decay step it protects, matching the verified
            // scratch-verify-a2.py reference trajectory bit-for-bit at this scenario's magnitudes
            // -- the two orderings differ by at most 1e-5, far below this scenario's 1e-3
            // parity tolerance and never near a threshold-crossing boundary here).
            speedGainProbability = Math.Ceiling(speedGainProbability * 100000.0) * 0.00001;

            string? targetLaneId = null;
            if (speedGainProbability > changeProbThresholdLeft
                && relativeGain > KraussModel.NumericalEps
                && neighDist / Math.Max(0.1, v.Kinematics.Speed) > 20.0)
            {
                // :1857-1864 fires. Target-lane safety veto (A2-iii) before committing --
                // MSLCM_LC2013::checkBlocking's role, minimal-faithful here (see
                // IsTargetLaneSafe). A vetoed change does NOT reset the accumulator (SUMO only
                // resets on an actually-committed change, :1063/1080) -- it keeps accumulating
                // and is retried next step.
                var neighFollow = postMoveNeighbors.GetNeighborFollower(v, leftLane.Id);
                if (IsTargetLaneSafe(v, neighLead, neighFollow, dt))
                {
                    targetLaneId = leftLane.Id;
                    speedGainProbability = 0.0; // :1063/1080 resetState() on committed change.
                }
            }

            v.SpeedGainProbability = speedGainProbability;

            // Structural change: instant lane-index snap (lanechange.duration=0), exactly like
            // rung 8b's keep-right swap in ExecuteMoves -- applied here, after this phase's own
            // frozen snapshot (postMoveNeighbors) was already built and is no longer read by any
            // other vehicle this step, so mutating v.LaneId now cannot affect another vehicle's
            // decision this same step (CLAUDE.md rule 2/3).
            if (targetLaneId is not null)
            {
                v.LaneId = targetLaneId;
            }
        }
    }

    // MSLCM_LC2013's keep-right sub-block ONLY (see CLAUDE.md briefing's scope note): strategic/
    // cooperative LC blocks all pass through as non-binding and only the keep-right accumulator
    // drives the decision. NOT built (scoped out): strategic/cooperative blocks, general
    // best-lanes (neighDist here is simply the right lane's own length, not a computed
    // continuation distance), continuous lateral (SL2015), lanechange.duration>0, safety/blocker
    // vetoes against neighbors, and multi-edge route lane continuity. `checkOverTakeRight`
    // (:1750-1758) stays unported: it requires lcOvertakeRight=true (non-default, off here) AND
    // ego's OWN-lane leader to be slow, which is never the case in any scenario reachable by this
    // engine today.
    //
    // Called from DecideSpeedGainChanges's post-move phase (see that method's CORRECTED-ORDERING
    // comment for why this is no longer a pre-move Plan/MoveIntent decision) -- reads/writes
    // `v.LaneId`/`v.KeepRightProbability` directly. This is still a single, isolated per-vehicle
    // read/write against the phase's ONE frozen `neighbors` snapshot (CLAUDE.md rule 3): no other
    // vehicle's decision this step reads `v`'s post-keep-right LaneId, since every vehicle's
    // neighbor lookups this phase go through the SAME already-built `neighbors` snapshot, not
    // live `v.LaneId` reads of other vehicles.
    private void ApplyKeepRightDecision(VehicleRuntime v, LaneNeighborQuery neighbors, double dt)
    {
        var lane = _network!.LanesById[v.LaneId];

        // Right neighbor = same edge, index-1 (no neighbor when already on index 0) -- this
        // guard is exactly what leaves single-lane rungs 1/3/4/5/6/7 and 8a (vehicle on index 0)
        // completely unaffected: the accumulator simply never advances off 0.
        var edge = _network.EdgesById[lane.EdgeId];
        var rightLane = edge.Lanes.FirstOrDefault(l => l.Index == lane.Index - 1);
        if (rightLane is null)
        {
            return;
        }

        // actionStepLength=1 in this scenario's config (phase-1 determinism ladder).
        const double keepRightTime = 5.0; // MSLCM_LC2013.cpp:67 KEEP_RIGHT_TIME
        const double changeProbThresholdRight = 2.0; // ctor: (0.2/mySpeedGainRight)/mySpeedGainParam, defaults 0.1/1
        const double keepRightParam = 1.0; // ctor default (LCA_KEEPRIGHT_PARAM)
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        var vMax = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.VType);
        var roadSpeedFactor = vMax / lane.Speed; // getSpeedLimit() of ego's OWN (current) lane

        // Legacy behavior (myKeepRightAcceptanceTime == -1, SUMO's default): acceptanceTime scales
        // with THIS STEP'S (now-settled, post-move) speed -- verified against SUMO via TraCI at
        // full precision per the rung-8b briefing; identical value whether read pre- or post-move
        // since ExecuteMoves already set v.Kinematics.Speed = this step's finalized CF speed.
        var acceptanceTime = 7.0 * roadSpeedFactor * Math.Max(1.0, v.Kinematics.Speed);

        // Scope note: general best-lanes continuation distance is deferred -- on this single-
        // edge dead-end route, the right lane's own length IS its full best-lanes continuation.
        var neighDist = rightLane.Length;

        var fullSpeedGap = Math.Max(0.0, neighDist - KraussModel.BrakeGap(vMax, v.VType.Decel, v.VType.Tau, dt));
        var fullSpeedDrivingSeconds = Math.Min(acceptanceTime, fullSpeedGap / vMax);

        // MSLCM_LC2013.cpp:1743-1748: a slower right-lane leader shrinks the "full speed driving
        // seconds" ego could enjoy before catching it, reducing the keep-right incentive -- reads
        // the SAME frozen post-move `neighbors` snapshot this whole phase reads (see this method's
        // header comment for why post-move, not pre-move). Non-binding (as in every prior single-
        // lane/empty-right-lane rung) whenever the right lane has no leader ahead of ego, or that
        // leader is not slower than vMax.
        var neighLead = neighbors.GetNeighborLeader(v, rightLane.Id);
        if (neighLead is not null && neighLead.Kinematics.Speed < vMax)
        {
            var neighLeadBackPos = neighLead.Kinematics.Pos - neighLead.VType.Length;
            var neighLeadGap = neighLeadBackPos - v.VType.MinGap - v.Kinematics.Pos;
            var secureGap = SecureGap(vMax, v.VType, neighLead.Kinematics.Speed, neighLead.VType.Decel, dt);

            fullSpeedGap = Math.Max(0.0, Math.Min(fullSpeedGap, neighLeadGap - secureGap));
            fullSpeedDrivingSeconds = Math.Min(fullSpeedDrivingSeconds, fullSpeedGap / (vMax - neighLead.Kinematics.Speed));
        }

        // checkOverTakeRight (:1750-1758) stays unported -- see this method's header comment.
        var deltaProb = changeProbThresholdRight * (fullSpeedDrivingSeconds / acceptanceTime) / keepRightTime;
        var keepRightProbability = v.KeepRightProbability - (actionStepLengthSecs * deltaProb);

        if (keepRightProbability * keepRightParam < -changeProbThresholdRight)
        {
            // MSLCM_LC2013.cpp:1789/1061-1064: fires -> lane change requested, accumulator resets
            // to 0 on change (changed()/resetState()). No safety/blocker veto ported here -- every
            // scenario reaching this fire has an empty target (right) lane; a real blocker veto
            // wants its own scenario with target-lane traffic on the RIGHT side (mirrors A2-iii's
            // scope note for the LEFT side).
            v.LaneId = rightLane.Id;
            v.KeepRightProbability = 0.0;
            return;
        }

        v.KeepRightProbability = keepRightProbability;
    }

    // MSLCM_LC2013::anticipateFollowSpeed (MSLCM_LC2013.cpp:1893-1941), non-accelerating-leader
    // branch only (acceleratingLeader is always false in this scenario: neither the slow leader
    // nor an empty left lane's absent leader ever has positive acceleration), and with
    // mySpeedGainLookahead=0 (ctor default, unmodeled here) so the :1926-1939 lookahead-braking
    // correction block never triggers (its outer guard is `mySpeedGainLookahead > 0`).
    private static double AnticipateFollowSpeed(VehicleRuntime ego, VehicleRuntime? leader, double dist, double dt)
    {
        if (leader is null)
        {
            // :1914-1920 (onInsertion=true is always used at this rung's two call sites, so the
            // acceleratingLeader/onInsertion=false arm at :1902-1908 is not reachable from here):
            // maximumSafeStopSpeed(dist, myDecel, mySpeed, onInsertion=true) -- default
            // headway=-1 (falls back to myHeadwayTime == vType.Tau, MSCFModel.cpp:834) and
            // default relaxEmergency=true (MSCFModel.h:612), so this is the emergency-decel-
            // relaxing overload, not the plain one followSpeed's maximumSafeFollowSpeed uses.
            return KraussModel.MaximumSafeStopSpeed(
                dist,
                ego.VType.Decel,
                ego.VType.EmergencyDecel,
                ego.Kinematics.Speed,
                ego.VType.Tau,
                dt,
                relaxEmergency: true);
        }

        // MSLane::getLeader's gap formula, applied to this (possibly adjacent-lane) leader --
        // same formula LeaderFollowSpeedConstraint uses for ego's own-lane leader.
        var leaderBackPos = leader.Kinematics.Pos - leader.VType.Length;
        var gap = leaderBackPos - ego.VType.MinGap - ego.Kinematics.Pos;

        // :1922: maximumSafeFollowSpeed(gap, mySpeed, leaderSpeed, leaderMaxDecel,
        // onInsertion=true) -- onInsertion=true skips the emergency-decel correction block
        // (KraussModel.MaximumSafeFollowSpeed's own `!onInsertion` guard), unlike followSpeed's
        // plan-phase car-following call (onInsertion=false).
        return KraussModel.MaximumSafeFollowSpeed(
            gap,
            ego.Kinematics.Speed,
            leader.Kinematics.Speed,
            leader.VType.Decel,
            ego.VType,
            dt,
            onInsertion: true);
    }

    // A2-iii: minimal-faithful target-lane safety veto, standing in for
    // MSLCM_LC2013::checkBlocking's full blocker-gap machinery (its own myLeftSpace/urgency/
    // yielding logic -- MSLCM_LC2013.cpp's checkBlocking/checkChangeBeforeCommitting family).
    // Ported at the granularity this rung's scenario (empty target lane -> non-binding) actually
    // needs: when either a neighbor leader or follower exists on the target lane, require the
    // same brake-gap-based secure gap MSCFModel::getSecureGap computes (MSCFModel.cpp:166-172,
    // its gComputeLC-relax branch omitted -- not reachable from a plain geometric check). A
    // scenario WITH real target-lane traffic is the right place to port checkBlocking itself.
    private static bool IsTargetLaneSafe(VehicleRuntime ego, VehicleRuntime? neighLead, VehicleRuntime? neighFollow, double dt)
    {
        if (neighLead is not null)
        {
            var gap = (neighLead.Kinematics.Pos - neighLead.VType.Length) - ego.VType.MinGap - ego.Kinematics.Pos;
            var secureGap = SecureGap(ego.Kinematics.Speed, ego.VType, neighLead.Kinematics.Speed, neighLead.VType.Decel, dt);
            if (gap < secureGap)
            {
                return false;
            }
        }

        if (neighFollow is not null)
        {
            var gap = (ego.Kinematics.Pos - ego.VType.Length) - neighFollow.VType.MinGap - neighFollow.Kinematics.Pos;
            var secureGap = SecureGap(neighFollow.Kinematics.Speed, neighFollow.VType, ego.Kinematics.Speed, ego.VType.Decel, dt);
            if (gap < secureGap)
            {
                return false;
            }
        }

        return true;
    }

    // MSCFModel::getSecureGap (MSCFModel.cpp:166-172): the brake-gap-difference secure-gap
    // formula (gComputeLC-relax branch at :173-179 omitted -- see IsTargetLaneSafe's comment).
    private static double SecureGap(double followerSpeed, ResolvedVType followerVType, double leaderSpeed, double leaderMaxDecel, double dt)
    {
        var maxDecel = Math.Max(followerVType.Decel, leaderMaxDecel);
        var leaderBrakeGap = KraussModel.BrakeGap(leaderSpeed, maxDecel, 0.0, dt);
        return Math.Max(0.0, KraussModel.BrakeGap(followerSpeed, followerVType.Decel, followerVType.Tau, dt) - leaderBrakeGap);
    }

    // The engine emits FULL double-precision trajectory values. The goldens are regenerated
    // with SUMO's `--precision` raised well above the default 2 (see scripts/regen-goldens.sh
    // and each scenario's provenance) so the committed FCD carries enough digits for the
    // per-scenario tolerance (1e-3) to be a *real* bar. Do NOT round emitted values to match a
    // low-precision golden: that would silently cap parity sensitivity at ~0.5*10^-precision
    // regardless of tolerance.json, masking genuine sub-0.01 trajectory drift. Lane-relative
    // Pos/Speed are the source of truth; x/y/angle are derived from the lane polyline.
    private void EmitTrajectory(TrajectorySet trajectory, double time)
    {
        foreach (var v in _vehicles)
        {
            if (!v.Inserted || v.Arrived)
            {
                continue;
            }

            var lane = _network!.LanesById[v.LaneId];
            var (x, y, angle) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos);

            trajectory.Add(new TrajectoryPoint(
                VehicleId: v.Def.Id,
                Time: time,
                Lane: v.LaneId,
                Pos: v.Kinematics.Pos,
                Speed: v.Kinematics.Speed,
                X: x,
                Y: y,
                Angle: angle,
                Acceleration: null));
        }
    }
}
