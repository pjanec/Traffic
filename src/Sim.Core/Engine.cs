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

    public void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _demand = DemandParser.Parse(rouXmlPath);
        _config = ScenarioConfigParser.Parse(sumocfgPath);

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

            // Plan/execute contract (DESIGN.md): plan reads start-of-step state and writes
            // only MoveIntent; execute applies all intents afterward. A follower must never
            // see a leader's updated position within the same step. The neighbor query is
            // built ONCE per step, here, from the same frozen start-of-step snapshot every
            // vehicle's plan phase reads (Seam 1: neighbor discovery behind an interface).
            var neighbors = LaneNeighborQuery.Build(_vehicles);
            PlanMovements(neighbors, time);
            ExecuteMoves(dt);
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
        };

        var vPos = constraints.Min();

        // MSCFModel.cpp:191 finalizeSpeed: `vStop = MIN2(vPos, veh->processNextStop(vPos))`.
        // ProcessNextStop reads only the front stop's START-OF-STEP snapshot (Reached/
        // RemainingDuration) and returns the transition to apply at Execute -- never mutates.
        var (processedVelocity, stopUpdate) = ProcessNextStop(v, vPos, actionStepLengthSecs);
        var vStop = Math.Min(vPos, processedVelocity);

        var newSpeed = KraussModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs);

        // Rung 8b (DESIGN.md Seam 4): keep-right (Rechtsfahrgebot) lane-change decision, ported
        // from MSLCM_LC2013::_wantsChange's keep-right block (see
        // sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp ~1721-1794). Evaluated in Plan AFTER this
        // step's CF speed (newSpeed) is known -- the decision reads only start-of-step/this-step-
        // planned values off `v`/`lane`/`laneVehicleMaxSpeed` and the immutable network, and
        // writes nothing but the returned MoveIntent (CLAUDE.md rule 3).
        var (targetLaneId, keepRightProbability) = ComputeKeepRightDecision(v, lane, laneVehicleMaxSpeed, newSpeed, dt);

        return new MoveIntent
        {
            NewSpeed = newSpeed,
            LatOffset = 0.0,
            StopUpdate = stopUpdate,
            TargetLaneId = targetLaneId,
            KeepRightProbability = keepRightProbability,
        };
    }

    // MSLCM_LC2013's keep-right sub-block ONLY (see CLAUDE.md briefing's scope note): on this
    // empty single-edge road every neighbor term (leader/follower on the right lane) is null, so
    // strategic/cooperative/speed-gain LC blocks all pass through as non-binding and only the
    // keep-right accumulator drives the decision. NOT built (scoped out): strategic/cooperative/
    // speed-gain blocks, general best-lanes (neighDist here is simply the right lane's own
    // length, not a computed continuation distance), continuous lateral (SL2015),
    // lanechange.duration>0, safety/blocker vetoes against neighbors (none exist here), and
    // multi-edge route lane continuity.
    private (string? TargetLaneId, double KeepRightProbability) ComputeKeepRightDecision(
        VehicleRuntime v,
        Lane lane,
        double laneVehicleMaxSpeed,
        double newSpeed,
        double dt)
    {
        // Right neighbor = same edge, index-1 (no neighbor when already on index 0) -- this
        // guard is exactly what leaves single-lane rungs 1/3/4/5/6/7 and 8a (vehicle on index 0)
        // completely unaffected: the accumulator simply never advances off 0.
        var edge = _network!.EdgesById[lane.EdgeId];
        var rightLane = edge.Lanes.FirstOrDefault(l => l.Index == lane.Index - 1);
        if (rightLane is null)
        {
            return (null, 0.0);
        }

        // actionStepLength=1 in this scenario's config (phase-1 determinism ladder).
        const double keepRightTime = 5.0; // MSLCM_LC2013.cpp:67 KEEP_RIGHT_TIME
        const double changeProbThresholdRight = 2.0; // ctor: (0.2/mySpeedGainRight)/mySpeedGainParam, defaults 0.1/1
        const double keepRightParam = 1.0; // ctor default (LCA_KEEPRIGHT_PARAM)
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        var vMax = laneVehicleMaxSpeed;
        var roadSpeedFactor = vMax / lane.Speed; // getSpeedLimit() of ego's OWN (current) lane

        // Legacy behavior (myKeepRightAcceptanceTime == -1, SUMO's default): acceptanceTime scales
        // with THIS STEP'S planned speed (newSpeed), not start-of-step speed -- verified against
        // SUMO via TraCI at full precision per the briefing.
        var acceptanceTime = 7.0 * roadSpeedFactor * Math.Max(1.0, newSpeed);

        // Scope note: general best-lanes continuation distance is deferred -- on this single-
        // edge dead-end route, the right lane's own length IS its full best-lanes continuation.
        var neighDist = rightLane.Length;

        var fullSpeedGap = Math.Max(0.0, neighDist - KraussModel.BrakeGap(vMax, v.VType.Decel, v.VType.Tau, dt));
        var fullSpeedDrivingSeconds = Math.Min(acceptanceTime, fullSpeedGap / vMax);

        // No leader/follower on the empty right lane here, so the neighLead/checkOverTakeRight
        // adjustment blocks (MSLCM_LC2013.cpp:1743-1758) are non-binding and correctly omitted.
        var deltaProb = changeProbThresholdRight * (fullSpeedDrivingSeconds / acceptanceTime) / keepRightTime;
        var keepRightProbability = v.KeepRightProbability - (actionStepLengthSecs * deltaProb);

        if (keepRightProbability * keepRightParam < -changeProbThresholdRight)
        {
            // MSLCM_LC2013.cpp:1789/1061-1064: fires -> lane change requested, accumulator resets
            // to 0 on change (changed()/resetState()). No safety/blocker veto to apply -- no
            // neighbor exists on this empty road.
            return (rightLane.Id, 0.0);
        }

        return (null, keepRightProbability);
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

            // Rung 8b: keep-right accumulator write-back (Engine.ComputeKeepRightDecision plans
            // it, only ExecuteMoves ever writes VehicleRuntime.KeepRightProbability -- CLAUDE.md
            // rule 3). Always written (0 when there is no right neighbor, matching the source's
            // own accumulator staying untouched/irrelevant on the rightmost lane).
            v.KeepRightProbability = v.Intent.KeepRightProbability;

            // Structural changes (lane swaps) flush through a command buffer here at step end --
            // this is the first real use of that buffer (DESIGN.md Seam 4 / "The plan/execute
            // contract"): lanechange.duration=0 makes this an INSTANT discrete lane-index snap,
            // not continuous lateral movement, so Pos/Speed are carried over unchanged and
            // LatOffset stays 0.
            if (v.Intent.TargetLaneId is { } targetLaneId)
            {
                v.LaneId = targetLaneId;
            }
        }
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
