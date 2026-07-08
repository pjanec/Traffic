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

    // D3 (FastDataPlane ECS readiness -- move managed/variable-length state off the per-entity
    // record): the shared lane-handle pool every vehicle's LaneSequence now slices into
    // (`[LaneSeqStart, LaneSeqStart+LaneSeqLen)`), replacing the old per-vehicle
    // `IReadOnlyList<string> LaneSequence`/`int[] LaneSequenceHandles` managed collections. A
    // route resolution (insertion or reroute) APPENDS its handle sequence here and repoints the
    // vehicle's slice -- the pool only grows (a reroute abandons its old slice in place; D7 can
    // compact if that ever matters).
    private readonly List<int> _laneSeqPool = new();

    // D3: this vehicle's scheduled stops (Sim.Ingest.VehicleDef.Stops), keyed by
    // VehicleRuntime.EntityIndex -- replaces the old per-vehicle `Queue<StopRuntime> Stops`
    // managed field. Populated once at LoadScenario, ONLY for vehicles that actually have stops
    // (def.Stops.Count > 0); absent from this dictionary is exactly the "no stops" fast path
    // (Count==0) every stop-consuming call site already handles.
    private readonly Dictionary<int, Queue<StopRuntime>> _stopsByEntity = new();

    // D3: this vehicle's already-routed-around-once edge set, keyed by EntityIndex -- replaces
    // the old per-vehicle `HashSet<string> AvoidedEdges` managed field. Lazily created only when
    // a vehicle first reroutes (UpdateReroutes); off the hot path (reroute is opt-in via
    // RerouteThresholdSeconds, +infinity by default).
    private readonly Dictionary<int, HashSet<string>> _avoidedByEntity = new();

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

    // D4 (FDP zero-alloc `OnUpdate` rule): ONE reusable LaneNeighborQuery, (re)built only when
    // LoadScenario changes the network (its bucket count is sized off `LanesByHandle.Count`).
    // Refilled -- not reconstructed -- twice per step: once for the pre-move snapshot (Run(),
    // before PlanMovements) and once for the post-move snapshot (DecideSpeedGainChanges(), after
    // ExecuteMoves). See LaneNeighborQuery's own header comment for why a single reused instance
    // is safe even though it is refilled twice per step.
    private LaneNeighborQuery? _neighborQuery;

    // D5 (FastDataPlane ECS readiness): ONE reusable command buffer for structural mutations
    // (lane swap / route replacement / arrival), matching FDP's `view.GetCommandBuffer()`.
    // Recorded during a phase, `Flush()`ed at that phase's own barrier (see UpdateReroutes/
    // ExecuteMoves/DecideSpeedGainChanges) -- each phase flushes before the next phase starts
    // recording, so one shared instance is safe to reuse sequentially all step long (Flush()
    // clears it). Pure representation refactor: WHEN each mutation applies is unchanged.
    // D7: this concrete instance is now handed to `_world` below (its sole owner conceptually,
    // per the seam) rather than used directly by the systems -- see `_world`/`_commandBuffer`'s
    // own comments just below for how it is reached from here on.
    private readonly CommandBuffer _commandBufferImpl = new();

    // D7 (FastDataPlane ECS readiness -- the FDP-shaped seam / adapter, TASKS.md line ~603): ONE
    // `IWorld` instance (the in-house `World` backend, see World.cs) wrapping the SAME
    // `_vehicles` list and `_commandBufferImpl` instance constructed above -- every engine system
    // below is rewritten by this rung to go through `_world`/`_commandBuffer` (now
    // `IWorld`/`ICommandBuffer`-typed) instead of touching `_vehicles`/`CommandBuffer` directly,
    // proving the drop-in seam is real (an `Fdp.Core`-backed `IWorld` could later replace `World`
    // without touching any call site below). Byte-identical: `World` performs no computation of
    // its own, it only forwards to the same list/buffer this field already owned before this
    // rung (see World.cs's own header comment).
    // D7: field initializers run in declaration order -- `_vehicles` (top of this class) and
    // `_commandBufferImpl` (just above) are both already constructed by the time this runs, so
    // no explicit constructor is needed to sequence them.
    private readonly IWorld _world;

    // D7: cached once from `_world.GetCommandBuffer()` at construction -- every EXISTING
    // `_commandBuffer.ChangeLane`/`ReplaceRoute`/`Destroy`/`Flush()` call site elsewhere in this
    // file is therefore untouched by this rung (same field name, now `ICommandBuffer`-typed
    // instead of `CommandBuffer`-typed; same underlying instance as `_commandBufferImpl` above).
    private readonly ICommandBuffer _commandBuffer;

    public Engine()
    {
        // D7: constructed here (not as a field initializer) purely for readability -- `_vehicles`
        // and `_commandBufferImpl` are already assigned by the time the implicit base/field-
        // initializer chain reaches this constructor body, so this is the first point both are
        // safely available together. `World` wraps them by reference; nothing is copied.
        _world = new World(_vehicles, _commandBufferImpl);
        _commandBuffer = _world.GetCommandBuffer();
    }

    // D6 (FastDataPlane ECS readiness -- phased systems over queries): the `Query()` analog.
    // Every hot-path system below (PlanMovements, ExecuteMoves, EmitTrajectory,
    // DecideSpeedGainChanges, the junction-foe scan, LaneNeighborQuery.Refill's snapshot builds)
    // iterates exactly this filter -- "inserted, not yet arrived" -- so it is expressed ONCE here
    // as a reusable, zero-alloc struct-enumerator query (see VehicleQuery.cs) instead of a
    // repeated inline `if (!v.Inserted || v.Arrived) continue;` guard at each call site.
    // Insertion's own not-yet-inserted candidate scan (InsertDepartingVehicles) is a DIFFERENT
    // predicate (the complement) and is left as a direct `_vehicles` walk, per the briefing.
    // D7: now sourced from `_world.ActiveVehicles()` (the IWorld seam's struct-returning query
    // factory -- see IWorld.cs's header comment for why it stays a concrete struct return, not
    // an IQuery/IEnumerable<T>, to remain zero-alloc) instead of constructing the struct
    // directly against `_vehicles` here; `World.ActiveVehicles()` constructs the exact same
    // `new(_vehicles)` value this method used to build itself, so this is a pure indirection.
    private ActiveVehicleQuery ActiveVehicles() => _world.ActiveVehicles();

    // D8 (FastDataPlane ECS readiness -- parallelize the Simulation phase). Default OFF so every
    // existing scenario/test/benchmark path (and the FCD parity path) stays exactly as it was.
    // When ON, PlanMovements below runs concurrently over `_vehicles` instead of sequentially via
    // ActiveVehicles(). This is provably safe (not just "probably fine"): ComputeMoveIntent's
    // ENTIRE call tree (LeaderFollowSpeedConstraint, StopLineConstraint, RedLightConstraint,
    // JunctionYieldConstraint/AdaptToJunctionLeader/FindFoeVehicle/IndexOfLaneHandle,
    // ObstacleConstraint, ProcessNextStop, KraussModel's static pure functions) reads ONLY:
    // this vehicle's own start-of-step Kinematics/lane/vType/stop-queue-front, the frozen
    // pre-move `neighbors` LaneNeighborQuery snapshot (Refilled once, before PlanMovements is
    // called, and never mutated again until DecideSpeedGainChanges' own later Refill -- see that
    // field's header comment), and the immutable `_network`/`_config`/`_obstacles`/
    // `_laneSeqPool`/`_stopsByEntity`/`_avoidedByEntity` side storage -- none of which is written
    // by anything in the plan phase (writes to those tables happen only in LoadScenario,
    // UpdateReroutes, and ExecuteMoves, all of which have already completed or not yet started
    // relative to PlanMovements). Each loop iteration below writes ONLY `v.Intent`, its own
    // entity's field (ProcessNextStop returns a StopTransition through MoveIntent rather than
    // mutating the stop side-table -- see its own header comment) -- no shared mutable
    // accumulator, no lock, no cross-entity write. That is exactly why plain per-index iteration
    // over `_vehicles` (rather than the ActiveVehicleQuery `foreach`, which is not itself
    // partitionable) is race-free here.
    public bool UseParallelPlan { get; set; } = false;

    // C1-i (TASKS.md "Statistical parity / driver imperfection"): the global seed for every
    // vehicle's per-entity dawdle RNG (Sim.Core.VehicleRng). Each vehicle's RngState is seeded
    // ONCE, at creation, from `VehicleRng.SeedFor(Seed, EntityIndex)` -- see
    // VehicleRuntime.RngState's own comment -- so this property must be set BEFORE
    // LoadScenario for a non-default value to take effect (mirrors how a later ensemble harness
    // (C1-ii/C1-iii, out of scope here) would sweep seeds: `new Engine { Seed = seed }` then
    // `LoadScenario(...)` per run). Defaults to 42, matching every rung-1..11 scenario's own
    // `<random_number><seed value="42"/></random_number>` (ScenarioConfig.Seed, parsed but
    // deliberately NOT auto-applied here by LoadScenario -- see that field's header comment for
    // why keeping this property the single caller-controlled source of truth is the safer
    // choice: auto-applying the config's seed would silently clobber a seed the caller had
    // already set for an ensemble sweep before calling LoadScenario).
    //
    // OWNER DECISION (TASKS.md C1): the statistical parity bar is ensemble/aggregate, not
    // RNG-exact, so this seed does NOT need to reproduce SUMO's own RandHelper stream for a
    // given sumocfg seed value -- it only needs to be deterministic and per-entity-independent
    // (see VehicleRng's own header comment).
    public ulong Seed { get; set; } = 42;

    // C7-i (TASKS.md "speedFactor distribution"): the salt XOR'd into Seed (via
    // VehicleRng.SeedFor's 3-arg overload) to derive the once-at-creation per-vehicle speedFactor
    // draw's RNG from a stream that is guaranteed distinct from RngState's dawdle stream, for
    // every entityIndex, even though both derive from the same Seed -- see VehicleRng.SeedFor's
    // own header comment and VehicleRuntime.SpeedFactor's. The literal value is simply the ASCII
    // bytes of "SpeedFac" (0x53='S',0x70='p',0x65='e',0x65='e',0x64='d',0x46='F',0x61='a',
    // 0x63='c') packed big-endian into a ulong -- memorable and self-documenting, not itself
    // load-bearing (any nonzero salt distinct across call sites would do).
    private const ulong SpeedFactorRngSalt = 0x5370656564466163UL;

    // D9 (FastDataPlane ECS readiness -- info/replication export SEAM, TASKS.md line ~651):
    // the registered `ISimExportObserver`s notified once per active vehicle, once per
    // Export-phase frame, from EmitTrajectory below. Empty by default -- with no observer
    // registered, the notify loop in EmitTrajectory is a no-op `foreach` over an empty list
    // (no virtual call, no allocation), which is exactly the byte-identical/zero-alloc
    // guarantee the briefing requires for every existing scenario/test/benchmark that never
    // calls AddExportObserver.
    private readonly List<ISimExportObserver> _exportObservers = new();

    // D9: the registration point a later FDP `IDescriptorTranslator`-style consumer would call
    // to attach WITHOUT touching any system in this file -- mirrors AddObstacle's add-style
    // idiom just below (a plain public setter method, no structural/command-buffer machinery
    // needed since observers are not simulated entities).
    public void AddExportObserver(ISimExportObserver observer) => _exportObservers.Add(observer);

    public void AddObstacle(string id, string laneId, double frontPos, double length,
        double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity)
    {
        _obstacles[id] = new ExternalObstacle(id, laneId, frontPos, length, startTime, endTime);
    }

    // B5-i: MOVING obstacle add-or-replace, same keyed-by-id contract as AddObstacle. Speed/
    // MaxDecel are the only difference from AddObstacle's construction -- AdvanceObstacles (Input
    // phase) dead-reckons FrontPos by Speed*dt every step from here on, and ObstacleConstraint
    // feeds Speed/MaxDecel into KraussModel.FollowSpeed instead of the static predSpeed=0 case.
    public void AddMovingObstacle(string id, string laneId, double frontPos, double length,
        double speed, double maxDecel,
        double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity)
    {
        _obstacles[id] = new ExternalObstacle(id, laneId, frontPos, length, startTime, endTime, speed, maxDecel);
    }

    // B5-i: per-step position/velocity correction from the external owner of this obstacle.
    // Inert-when-absent (no-op if `id` was never added/was removed) -- mirrors RemoveObstacle's
    // TryGetValue-free `Remove` idiom, just for an update instead of a delete. `record with`
    // preserves LaneId/Length/StartTime/EndTime/MaxDecel unchanged; only FrontPos/Speed move.
    public void UpdateObstacle(string id, double frontPos, double speed)
    {
        if (_obstacles.TryGetValue(id, out var obstacle))
        {
            _obstacles[id] = obstacle with { FrontPos = frontPos, Speed = speed };
        }
    }

    public void RemoveObstacle(string id) => _obstacles.Remove(id);

    public void ClearObstacles() => _obstacles.Clear();

    // B5-i: dead-reckon every MOVING obstacle (Speed != 0) forward by Speed*dt, once per step, in
    // the Input phase BEFORE PlanMovements/the neighbor-query Refill -- so the Plan phase reads a
    // FROZEN obstacle position for this step, exactly like every other piece of start-of-step
    // state (CLAUDE.md rule 2: plan reads start-of-step state only). This is linear extrapolation
    // of an externally-supplied velocity (navmesh/RVO agent, live detection), NOT a SUMO
    // car-following/motion model -- the external owner is expected to call UpdateObstacle with a
    // fresh reading whenever it has one; dead-reckoning just fills the gap between updates.
    // Speed==0 obstacles are skipped entirely (no record replacement), which is exactly why a
    // static (B1) obstacle's FrontPos never changes here and the rest of the step tree sees a
    // byte-identical world to before this rung.
    private readonly List<string> _obstacleIdScratch = new();

    private void AdvanceObstacles(double dt)
    {
        if (_obstacles.Count == 0)
        {
            return;
        }

        // Snapshot the ids first, into a reused scratch list (no per-step allocation): replacing
        // an existing key's VALUE via the indexer below is fine mid-loop, but Dictionary still
        // disallows enumerating _obstacles.Values/Keys directly while any entry's value changes,
        // so we iterate a stable id copy instead.
        _obstacleIdScratch.Clear();
        _obstacleIdScratch.AddRange(_obstacles.Keys);

        foreach (var id in _obstacleIdScratch)
        {
            var obstacle = _obstacles[id];
            if (obstacle.Speed == 0.0)
            {
                continue;
            }

            _obstacles[id] = obstacle with { FrontPos = obstacle.FrontPos + obstacle.Speed * dt };
        }
    }

    public void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _demand = DemandParser.Parse(rouXmlPath);
        _config = ScenarioConfigParser.Parse(sumocfgPath);

        // B3: the cached router is built from the network being replaced above -- invalidate it
        // here so UpdateReroutes lazily rebuilds against the NEW network the next time it is
        // actually needed (never eagerly, since most scenarios never reroute at all).
        _router = null;

        // D4: (re)build the reusable neighbor-query buckets for the newly loaded network's dense
        // handle space -- cold path (once per LoadScenario call), never per step.
        _neighborQuery = new LaneNeighborQuery(_network.LanesByHandle.Count);

        _vehicles.Clear();
        // D3: side storage is keyed by EntityIndex (== _vehicles list index) -- clear it in
        // lockstep with _vehicles so a re-LoadScenario on the same Engine instance never leaves
        // stale entries keyed against the previous scenario's vehicles. The pool only ever grows
        // within one scenario's lifetime; a fresh scenario starts it clean too.
        _laneSeqPool.Clear();
        _stopsByEntity.Clear();
        _avoidedByEntity.Clear();
        foreach (var def in _demand.Vehicles)
        {
            var rawVType = _demand.VTypesById[def.TypeId];
            // vType defaults resolver (CLAUDE.md rule 6: match vType/init first): only vClass
            // and any explicit overrides (e.g. rou.xml's sigma="0") come from the raw parse;
            // everything else is a resolved SUMO vClass default (VTypeDefaults.Resolve).
            var vType = VTypeDefaults.Resolve(rawVType);
            // D3: EntityIndex is this vehicle's stable index in _vehicles, set once here -- see
            // VehicleRuntime.EntityIndex's own comment.
            var entityIndex = _vehicles.Count;
            // D5: the FDP-shaped handle, set once here alongside EntityIndex -- Generation
            // stays 0 (see Entity.cs / VehicleRuntime.Entity's own comments).
            // C1-i: seeded ONCE here, from the engine's global Seed + this vehicle's own stable
            // EntityIndex -- see VehicleRuntime.RngState's and Engine.Seed's own comments. Every
            // vehicle gets an independent stream regardless of insertion/plan order or thread
            // scheduling (UseParallelPlan's parallel-safety argument).
            var rngState = VehicleRng.SeedFor(Seed, entityIndex);

            // C7-i: the per-vehicle speedFactor draw (MSVehicleType::computeChosenSpeedDeviation,
            // MSVehicleControl.cpp:113's once-at-build call site) -- drawn from its OWN salted,
            // local RNG (never stored/reused after this one draw, matching SUMO's own one-shot
            // call), completely independent of `rngState` above (VehicleRuntime.RngState / C1's
            // per-step dawdle stream -- see VehicleRng.SeedFor's 3-arg overload and
            // VehicleRuntime.SpeedFactor's own comments for why this independence matters).
            // ScenarioConfig.SpeedDev is the dev SUMOVTypeParameter.cpp:374-378's
            // `default.speeddev` override resolves to (every existing scenario sets it to 0, so
            // NormcDistribution.SampleNormc's dev<=0 branch returns `vType.SpeedFactor` --
            // 1.0 -- with NO draw at all, byte-identical to every pre-C7 rung).
            var speedFactorRng = VehicleRng.SeedFor(Seed, entityIndex, SpeedFactorRngSalt);
            var speedFactor = NormcDistribution.ComputeChosenSpeedDeviation(
                mean: vType.SpeedFactor, dev: _config.SpeedDev, min: 0.2, max: 2.0, rng: ref speedFactorRng);

            var runtime = new VehicleRuntime
            {
                Def = def,
                VType = vType,
                EntityIndex = entityIndex,
                Entity = new Entity(entityIndex, 0),
                RngState = rngState,
                SpeedFactor = speedFactor,
                // C11-iv: MSCFModel_IDM::VehicleVariables ctor default (MSCFModel_IDM.h:191)
                // `levelOfService(1.)` -- set for every vehicle (harmless/inert for non-IDMM
                // vTypes, see VehicleRuntime.LevelOfService's own comment).
                LevelOfService = 1.0,
                // C8-ii: sentinel so a vehicle's FIRST plan is always an action step (it re-plans
                // on insertion), regardless of its depart time -- see VehicleRuntime.LastActionTime.
                LastActionTime = double.NegativeInfinity,
            };

            // Rung 5 (D3: side table): seed this vehicle's own stop queue (StopRuntime) from its
            // immutable Def, ONLY when it actually has stops. Reached/RemainingDuration start at
            // their defaults (false/0) -- ProcessNextStop only initializes RemainingDuration once
            // the stop is actually reached.
            if (def.Stops.Count > 0)
            {
                var stops = new Queue<StopRuntime>();
                foreach (var stopDef in def.Stops)
                {
                    stops.Enqueue(new StopRuntime
                    {
                        LaneId = stopDef.LaneId,
                        StartPos = stopDef.StartPos,
                        EndPos = stopDef.EndPos,
                        Duration = stopDef.Duration,
                    });
                }

                _stopsByEntity[entityIndex] = stops;
            }

            _vehicles.Add(runtime);
        }
    }

    // D3: front-of-queue lookup against the side table, returning the same "no stops" empty
    // fast path every call site already handled against v.Stops.Count == 0 -- absent from
    // _stopsByEntity is exactly that case (LoadScenario only populates entries that have >=1
    // stop).
    private Queue<StopRuntime>? GetStops(VehicleRuntime v) =>
        _stopsByEntity.TryGetValue(v.EntityIndex, out var stops) ? stops : null;

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

            // D6 (FastDataPlane ECS readiness -- phased systems over queries, matching FDP's
            // `SystemPhase`-ordered systems): the per-step body below is the SAME sequence of
            // passes as before this rung, now labeled by the `SystemPhase` each belongs to
            // (SystemPhase.cs). CLAUDE.md rule 2 / the D6 briefing: preserve calculation order
            // EXACTLY -- this reorganizes how the loop reads, never what runs or when.

            // [SystemPhase.Input] Newly-departed vehicles enter the simulation. Runs before this
            // step's Export so a vehicle inserted THIS step is immediately included in the
            // trajectory point emitted below (matching golden.fcd.xml's presence set: a vehicle
            // is present starting at its own depart-time row, not one step late).
            InsertDepartingVehicles(time);

            // [SystemPhase.Export] Export of the previous frame. This emits the SETTLED state
            // produced by the PRIOR step's PostSimulation phase (plus any vehicle just inserted
            // above) -- it must stay at the TOP of the loop, BEFORE this step's
            // Simulation/PostSimulation run. Moving it to the bottom would change the
            // traffic-light `time+dt` sampling semantics (RedLightConstraint's own comment) and
            // desync the trajectory from the golden. See EmitTrajectory's header comment (also:
            // the `TrajectorySet`/emit allocation and the `Run(int)->TrajectorySet` return
            // contract are untouched here -- that streaming/zero-alloc-export concern belongs to
            // D9's export seam, not D6).
            EmitTrajectory(trajectory, time);

            // [SystemPhase.Input] Reroute-around-prolonged-blockage. Runs ONCE per step, BEFORE
            // Simulation/PlanMovements, so a vehicle that reroutes this step immediately plans
            // against its NEW route this same step (see UpdateReroutes' own header comment for
            // why this ordering, and why it is still a seam-4 structural mutation rather than a
            // Simulation-phase concern).
            UpdateReroutes(time, dt);

            // [SystemPhase.Input] B5-i: dead-reckon MOVING external obstacles (Speed != 0) by
            // Speed*dt. Runs BEFORE the neighbor-query Refill/PlanMovements below so the Plan
            // phase's ObstacleConstraint reads a FROZEN, already-advanced obstacle position for
            // this step (CLAUDE.md rule 2) -- never mutated mid-plan. Static (Speed==0, including
            // every B1 obstacle) obstacles are untouched by this call, so this is a pure no-op
            // for every existing obstacle-free or static-obstacle scenario/test.
            AdvanceObstacles(dt);

            // [SystemPhase.Simulation] Plan/execute contract (DESIGN.md): plan reads
            // start-of-step state and writes only MoveIntent; execute applies all intents
            // afterward. A follower must never see a leader's updated position within the same
            // step. The neighbor query is refilled ONCE per step, here, from the same frozen
            // start-of-step snapshot every vehicle's plan phase reads (Seam 1: neighbor discovery
            // behind an interface). D4: `_neighborQuery` is the ONE reusable instance built in
            // LoadScenario -- Refill clears/re-adds/re-sorts its pre-allocated per-lane buckets,
            // no per-step allocation. This is the async-module analog in FDP terms: RO reads of
            // the frozen snapshot + immutable network/vType data, writing only each vehicle's own
            // MoveIntent (CLAUDE.md rule 3).
            var neighbors = _neighborQuery!;
            neighbors.Refill(ActiveVehicles());
            PlanMovements(neighbors, time);

            // [SystemPhase.PostSimulation] Apply every vehicle's own MoveIntent, integrate
            // position, flush arrival through the command buffer.
            ExecuteMoves(dt);

            // [SystemPhase.PostSimulation] Rung A2 (speed-gain/overtaking lane change): SUMO's
            // own per-step order is planMovements -> executeMovements -> changeLanes
            // (MSNet.cpp:784/790/796) -- the lane-change decision (MSLCM_LC2013::_wantsChange's
            // speed-gain block) runs AFTER this step's longitudinal move, so it sees POST-move
            // gaps (unlike keep-right, which has no leader-gap dependence and stays entirely in
            // the pre-move Plan phase per rung 8b). This is its own PostSimulation pass, not a
            // change to PlanMovements/ExecuteMoves above.
            // B5-ii: `time` is threaded through so the veto below can evaluate obstacle
            // active-windows (StartTime/EndTime) at the SAME instant ObstacleConstraint already
            // does earlier this same step -- see DecideSpeedGainChanges' own header comment and
            // TargetLaneBlockedByObstacle.
            DecideSpeedGainChanges(time, dt);
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
        // D6: the "inserted, not arrived" half of the guard is now the ActiveVehicles() query;
        // the lane filter stays inline (it is specific to this call site, not the reusable
        // predicate).
        foreach (var other in ActiveVehicles())
        {
            if (other.LaneId != laneId)
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
        // D2: keep LaneHandle in lockstep with LaneId at every write site -- the lane just
        // resolved above already carries its own dense Handle, so this is a direct field read,
        // no dictionary lookup needed.
        v.LaneHandle = lane.Handle;
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
        // equals the sequence's first element).
        // D3: append the handle-parallel sequence to the shared pool and slice into it, instead
        // of allocating a per-vehicle array -- same traversal, same order as before.
        var handleSeq = _network!.ResolveLaneSequenceHandles(route.Edges, v.Def.DepartLaneIndex);
        v.LaneSeqStart = _laneSeqPool.Count;
        v.LaneSeqLen = handleSeq.Length;
        _laneSeqPool.AddRange(handleSeq);
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

        // D6: "inserted, not arrived" via the reusable ActiveVehicles() query; the
        // mid-junction/internal-lane skip stays inline (specific to this pass).
        foreach (var v in ActiveVehicles())
        {
            if (v.LaneId.StartsWith(':'))
            {
                // Mid-junction on an internal lane -- a reroute mid-junction has nowhere
                // sensible to redirect from (ego is already committed to the connection it is
                // traversing), so this vehicle is simply skipped this step; it will be
                // reconsidered once it lands on its next normal lane.
                continue;
            }

            // D2: hot per-vehicle lookup -- handle-indexed array instead of a string hash.
            var currentEdge = _network!.LanesByHandle[v.LaneHandle].EdgeId;

            // Distinct FUTURE normal edges (route order, deduplicated), i.e. every normal edge
            // this vehicle's route still has left to traverse AFTER its current position,
            // excluding currentEdge itself and any internal/junction lane's edge id.
            // D3: walk the pool slice `[LaneSeqStart+LaneSeqIndex+1, LaneSeqStart+LaneSeqLen)`
            // mapping handle -> LanesByHandle[h] instead of indexing the old string LaneSequence.
            var futureEdges = new List<string>();
            var futureEdgesSeen = new HashSet<string>(StringComparer.Ordinal);
            for (var i = v.LaneSeqIndex + 1; i < v.LaneSeqLen; i++)
            {
                var seqLane = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]];
                if (seqLane.Id.StartsWith(':'))
                {
                    continue;
                }

                var seqEdgeId = seqLane.EdgeId;
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
            // D3: last element of the pool slice instead of v.LaneSequence[^1].
            var destEdge = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + v.LaneSeqLen - 1]].EdgeId;
            // D3: AvoidedEdges side table (absent == empty set so far -- this vehicle has never
            // rerouted before).
            var avoid = _avoidedByEntity.TryGetValue(v.EntityIndex, out var avoidedSoFar)
                ? new HashSet<string>(avoidedSoFar, StringComparer.Ordinal) { blockedEdge }
                : new HashSet<string>(StringComparer.Ordinal) { blockedEdge };
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
            // D3: v.LaneId/v.LaneHandle are unchanged by a reroute (newEdges[0] == currentEdge, v
            // stays physically where it is), only the REMAINING route changes -- append the newly
            // resolved handle sequence to the shared pool as a NEW slice (the old slice is simply
            // abandoned in the pool; it only grows).
            // D5: the pool append is engine-owned (not per-vehicle deferred state) and stays
            // inline; only the vehicle's own [LaneSeqStart, LaneSeqLen) slice (+ the LaneSeqIndex
            // reset ReplaceRoute's Flush always applies) goes through the command buffer, flushed
            // at the end of this method (matching today's timing exactly -- nothing later in
            // THIS SAME iteration or any other vehicle's iteration this loop reads v's
            // LaneSeqStart/Len/Index after this point, see UpdateReroutes' own D5 comment below).
            var laneIndex = _network.LanesByHandle[v.LaneHandle].Index;
            var newHandleSeq = _network.ResolveLaneSequenceHandles(newEdges, laneIndex);
            var newLaneSeqStart = _laneSeqPool.Count;
            _laneSeqPool.AddRange(newHandleSeq);
            _commandBuffer.ReplaceRoute(v, newLaneSeqStart, newHandleSeq.Length);

            if (!_avoidedByEntity.TryGetValue(v.EntityIndex, out var avoidedEdges))
            {
                avoidedEdges = new HashSet<string>(StringComparer.Ordinal);
                _avoidedByEntity[v.EntityIndex] = avoidedEdges;
            }

            avoidedEdges.Add(blockedEdge);
            v.BlockedByObstacleSeconds = 0.0;
        }

        // D5: apply every ReplaceRoute recorded above, in record order, at this method's end --
        // the SAME point v.LaneSeqStart/Len/Index took effect at before this rung, still strictly
        // before PlanMovements (called next, in Run()) reads them.
        _commandBuffer.Flush();
    }

    // Plan phase (seam 1, parallel-safe): reads start-of-step world state (including the frozen
    // `neighbors` snapshot), writes only to the owning vehicle's own MoveIntent. No shared-state
    // writes, even single-threaded -- the rung-5 stop-transition decision (see ProcessNextStop)
    // is threaded through MoveIntent.StopUpdate rather than mutating v.Stops here, so this rule
    // holds even though a vehicle's own stop bookkeeping "changes" every step it is stopped.
    private void PlanMovements(LaneNeighborQuery neighbors, double time)
    {
        // D8: opt-in concurrent plan -- see UseParallelPlan's own header comment for the
        // race-free argument. Indexes over the backing list (not the ActiveVehicleQuery
        // `foreach`) so Parallel.For can partition it; the "inserted, not arrived" guard is
        // re-checked inline per index, matching ActiveVehicleQuery.Enumerator's own predicate.
        if (UseParallelPlan)
        {
            System.Threading.Tasks.Parallel.For(0, _vehicles.Count, i =>
            {
                var v = _vehicles[i];
                if (!v.Inserted || v.Arrived)
                {
                    return;
                }

                v.Intent = ComputeMoveIntent(v, neighbors, time);
            });
            return;
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {

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
        // D2: hot per-vehicle, per-step lookup -- handle-indexed array instead of a string hash.
        var lane = _network!.LanesByHandle[v.LaneHandle];
        var dt = _config!.StepLength;
        // default.action-step-length=1 in rung 1's config, equal to dt; kept as its own value
        // (not silently assumed == dt) since MSCFModel.cpp divides by it separately from TS.
        var actionStepLengthSecs = _config.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        // C8-ii: action-step (reaction-time) gate. A vehicle re-plans its car-following speed only
        // every actionStepLength seconds; between action steps it CONTINUES with the acceleration
        // decided at the last action step -- ported from MSVehicle.cpp:4443-4462 (a non-action step
        // skips processLinkApproaches and sets `vSafe = getSpeed() + ACCEL2SPEED(myAcceleration)`,
        // and MSVehicle.cpp:4489 skips finalizeSpeed off the action step). isActionStep
        // (MSVehicle.h:638): `(t - myLastActionTime) % actionStepLength == 0`. With
        // actionStepLength == dt (every pre-C8-ii scenario) the `> dt` guard is false and this is
        // entirely inert -- every step re-plans exactly as before (byte-identical).
        if (actionStepLengthSecs > dt)
        {
            // First plan (sentinel) OR a full action interval has elapsed since the last one.
            var isActionStep = double.IsNegativeInfinity(v.LastActionTime)
                || time - v.LastActionTime + KraussModel.NumericalEps >= actionStepLengthSecs;
            if (!isActionStep)
            {
                // Hold the last action step's acceleration -- no constraint re-evaluation, no
                // finalizeSpeed (MSVehicle.cpp:4454). v.Acceleration is this vehicle's realized
                // acceleration from the last completed step (written in ExecuteMoves), the exact
                // analog of myAcceleration read here. MAX2(., 0) mirrors the Euler non-negativity
                // updateState relies on (a held decel cannot drive speed below 0).
                return new MoveIntent
                {
                    NewSpeed = Math.Max(0.0, v.Kinematics.Speed + v.Acceleration * dt),
                    LatOffset = 0.0,
                    StopUpdate = null,
                };
            }

            // Action step: re-plan below, and record it (this vehicle's own field only -- the same
            // per-ego plan-phase write pattern as RngState/LevelOfService, parallel-safe).
            v.LastActionTime = time;
        }

        var laneVehicleMaxSpeed = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);

        // D4 (FDP zero-alloc `OnUpdate` rule): the multi-constraint reducer is still a MINIMUM
        // over the same six constraints, in the same call order (DESIGN.md seam 1) -- just
        // folded into a running `Math.Min` instead of a `new List<double>{ ... }.Min()`, since
        // min is associative/order-independent (no behavior change) but the old per-vehicle,
        // per-step list allocation was the single biggest hot-path allocator this rung removes.
        var vPos = double.PositiveInfinity;

        // Leader car-following (MSCFModel_Krauss.cpp followSpeed -> MSCFModel.cpp
        // maximumSafeFollowSpeed): the REAL formula our resolved carFollowModel="Krauss"
        // uses -- NOT MSCFModel_KraussOrig1::vsafe (removed; see rung-4 briefing, that
        // formula is dead code once a real leader exists). No leader => +infinity
        // (non-binding), matching a gap=+infinity KraussOrig1 vsafe call's short-circuit
        // but via the real code path: simply contribute nothing when there is no leader.
        // C11-i: for an IDM-resolved vType this dispatches to IdmModel.FollowSpeed
        // (MSCFModel_IDM.cpp:104-107) instead -- see FollowSpeedFor's own header comment; the
        // Krauss arm is the SAME KraussModel.FollowSpeed call this line always made.
        vPos = Math.Min(vPos, LeaderFollowSpeedConstraint(v, neighbors, dt, time, laneVehicleMaxSpeed));

        // Desired free-flow speed (MSLane::getVehicleMaxSpeed): lane speed limit adapted
        // by this vehicle's speedFactor, capped by its vType maxSpeed. C11-i: for Krauss this
        // stays the plain laneVehicleMaxSpeed value (byte-identical -- MSCFModel_Krauss never
        // overrides freeSpeed and our simplified single-lane engine never calls the base
        // MSCFModel::freeSpeed braking-curve formula for this term either way, matching every
        // pre-C11 rung exactly); for IDM this routes through IdmModel.FreeSpeed
        // (MSCFModel_IDM.cpp:77-100) -- see FreeFlowDesiredSpeedConstraint's own header comment.
        vPos = Math.Min(vPos, FreeFlowDesiredSpeedConstraint(v, laneVehicleMaxSpeed, dt));

        // Stop line (rung 5): MSVehicle.cpp's planMoveInternal "process stops" block
        // (~lines 2467-2540), non-waypoint arm only. +infinity (non-binding) once reached
        // (the source's own approach-block condition `!stop.reached || (waypoint &&
        // keepStopping())` is simply false for a non-waypoint stop that IS reached) or when
        // there is no stop at all.
        vPos = Math.Min(vPos, StopLineConstraint(v, dt, actionStepLengthSecs, laneVehicleMaxSpeed));

        // Red light (rung 10): MSVehicle.cpp's planMoveInternal per-link loop (~2641-2666,
        // 2734), yellowOrRed arm only. +infinity (non-binding) when this lane's outgoing
        // connection is not TL-controlled, or its light is green, at the time this Plan/
        // Execute cycle's result will be observed (see RedLightConstraint's own comment on
        // why that is `time + dt`, not `time`).
        vPos = Math.Min(vPos, RedLightConstraint(v, lane, time, dt, actionStepLengthSecs, laneVehicleMaxSpeed));

        // Priority-junction yielding (rung 9b-ii/iii, plus B5-iii's external-agent foe): MSLink's
        // right-of-way gate (stop-line brake while a higher-priority foe still approaches) plus
        // MSVehicle::adaptToJunctionLeader (car-following against a foe already on the junction).
        // +infinity (non-binding) whenever ego has no upcoming/current junction link, or
        // that link's foes are all cleared/absent -- see JunctionYieldConstraint's own
        // comment for the full derivation and its determinism note.
        // D6: pass the reusable ActiveVehicles() query (rather than the raw `_vehicles` list) so
        // the foe scan below (FindFoeVehicle) walks the same "inserted, not arrived" filter as
        // every other pass, instead of re-checking it inline.
        // B5-iii: `time` is threaded through purely so ExternalAgentOnFoeLane (called from inside
        // JunctionYieldConstraint's foe-link loop) can evaluate an ExternalObstacle's own
        // [StartTime, EndTime) active window at the SAME instant every other obstacle read this
        // step uses (ObstacleConstraint/TargetLaneBlockedByObstacle's own convention) -- nothing
        // about the pre-existing 9b-ii/iii SUMO-foe machinery reads `time` at all.
        vPos = Math.Min(vPos, JunctionYieldConstraint(v, ActiveVehicles(), time, dt, actionStepLengthSecs, laneVehicleMaxSpeed));

        // B1: external obstacle (DESIGN.md "Two futures" -- a live, non-SUMO input, not a
        // ported SUMO code path). Modeled as one more virtual stopped leader reusing the same
        // KraussModel.FollowSpeed leader car-following formula as LeaderFollowSpeedConstraint
        // above -- the multi-constraint reducer does not care where a constraint's speed came
        // from. +infinity (non-binding) whenever _obstacles is empty or none is active/ahead
        // on this lane -- this is the inert-when-absent guard: an empty store makes this a
        // no-op Min term, leaving every existing (obstacle-free) parity scenario untouched.
        vPos = Math.Min(vPos, ObstacleConstraint(v, time, laneVehicleMaxSpeed));

        // MSCFModel.cpp:191 finalizeSpeed: `vStop = MIN2(vPos, veh->processNextStop(vPos))`.
        // ProcessNextStop reads only the front stop's START-OF-STEP snapshot (Reached/
        // RemainingDuration) and returns the transition to apply at Execute -- never mutates.
        var (processedVelocity, stopUpdate) = ProcessNextStop(v, vPos, actionStepLengthSecs);
        var vStop = Math.Min(vPos, processedVelocity);

        // C1-i: threaded `ref` so the dawdle draw (only taken when v.VType.Sigma>0) advances
        // THIS vehicle's own private RngState in place -- no shared/global RNG, so this remains
        // safe under UseParallelPlan (each loop iteration/task only ever touches its own `v`).
        // C11-i: IDM's finalizeSpeed (MSCFModel_IDM.cpp:67-74) never dawdles (no sigma concept in
        // this port's scope -- see IdmModel.FinalizeSpeed's own header comment), so it takes no
        // `ref VehicleRng` at all; the Krauss arm below is the SAME KraussModel.FinalizeSpeed call
        // this line always made.
        // C11-ii: ACC never overrides finalizeSpeed OR patchSpeedBeforeLC (MSCFModel_ACC.h has no
        // such override), so it inherits the BASE MSCFModel::finalizeSpeed -- the SAME vMin/vMax
        // accel/decel-bound clamp IDM's own override delegates to, with patchSpeedBeforeLC's base
        // (non-Krauss-overridden) default of `return vMax` (MSCFModel.h:102-105) meaning NO dawdle
        // at all, regardless of vType.Sigma. IdmModel.FinalizeSpeed IS that exact base formula
        // (see its own header comment), so ACC reuses it verbatim rather than duplicating it.
        // C11-iii: CACC never overrides finalizeSpeed/patchSpeedBeforeLC either (MSCFModel_CACC.h
        // has no such override, just like MSCFModel_ACC.h) -- it inherits the SAME base
        // MSCFModel::finalizeSpeed dawdle-free clamp ACC/IDM already route through here.
        // C11-iv: IDMM shares this SAME base finalizeSpeed body too (MSCFModel_IDM.cpp:67-74's
        // `vNext = MSCFModel::finalizeSpeed(veh, vPos)` line is unconditional, myIDMM or not) --
        // IdmModel.FinalizeSpeed's call/body below is byte-identical for IDM/ACC/CACC/IDMM alike.
        var newSpeed = v.VType.CarFollowModel is "IDM" or "ACC" or "CACC" or "IDMM"
            ? IdmModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs)
            : KraussModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs, ref v.RngState);

        // C11-iv: MSCFModel_IDM.cpp:69-72's `if (myAdaptationFactor != 1.)` levelOfService update,
        // applied HERE (by the caller) right after `newSpeed` (== the vendored source's own
        // `vNext`) is computed -- see IdmModel.FinalizeSpeed's own C11-iv comment for why this
        // update lives outside that shared function body. Per-ego plan-phase mutation of `v`'s
        // OWN field, the exact same pattern C1's RngState / C11-ii's AccControlMode / C11-iii's
        // CaccControlMode already establish as parallel-safe (Engine.UseParallelPlan's own
        // argument) -- never another vehicle's state, never a MoveIntent field.
        if (v.VType.CarFollowModel == "IDMM")
        {
            v.LevelOfService = IdmmModel.UpdateLevelOfService(v.LevelOfService, newSpeed, laneVehicleMaxSpeed, dt);
        }

        return new MoveIntent
        {
            NewSpeed = newSpeed,
            LatOffset = 0.0,
            StopUpdate = stopUpdate,
        };
    }

    // C11-i/C11-ii dispatch (TASKS.md): every constraint below computes ego's OWN car-following
    // speed against some leader/obstacle/stop -- `vType` is always the EGO vehicle's resolved
    // vType (never a leader's/foe's), so a single vType.CarFollowModel check at each call site
    // is the complete dispatch. The Krauss arm of every one of these two helpers is the EXACT
    // pre-C11 call (same argument values, just routed through a pass-through wrapper) -- see
    // CLAUDE.md rule "byte-identical Krauss" and this rung's briefing.
    //
    // FollowSpeedFor: MSCFModel_Krauss.cpp:111-127 followSpeed (KraussModel.FollowSpeed) vs.
    // MSCFModel_IDM.cpp:104-107 followSpeed (IdmModel.FollowSpeed) vs. MSCFModel_ACC.cpp:96-106
    // followSpeed (AccModel.FollowSpeed) -- IDM/ACC's `desSpeed` argument is
    // `veh->getLane()->getVehicleMaxSpeed(veh)`, i.e. the caller-supplied `laneVehicleMaxSpeed`
    // (ego's OWN current-lane desired speed, independent of which leader/foe this call is against).
    // C11-ii: `time`/`accControlMode`/`accLastUpdateTime` are ONLY read/written by the ACC arm
    // (AccModel.cs's own header comment) -- every Krauss/IDM call site simply threads its own
    // `time` and `ref ego.AccControlMode/ref ego.AccLastUpdateTime` through unread/unwritten, so
    // this stays a pure plumbing change for those two models, not a behavior change (verified by
    // the "Krauss AND IDM byte-identical" parity tests below).
    // C11-iii: `caccControlMode`/`egoAcceleration`/`hasPred`/`predIsCacc` are ONLY read/written by
    // the CACC arm (CaccModel.cs's own header comment) -- CACC's arm ALSO threads
    // `accControlMode`/`accLastUpdateTime` (reused for its own embedded ACC-fallback state, see
    // VehicleRuntime.CaccControlMode's own header comment for why that reuse is required, not
    // optional) -- every Krauss/IDM/ACC call site simply threads the four new parameters through
    // unread/unwritten, keeping this a pure plumbing change for those three models (verified by
    // the "Krauss/IDM/ACC byte-identical" parity tests below).
    // C11-iv: `levelOfService` is the EGO's own start-of-step MSCFModel_IDM::VehicleVariables::
    // levelOfService (VehicleRuntime.LevelOfService) -- ONLY read by the IDMM arm below (to build
    // the adapted headwayTime override), unread/unused for every other CarFollowModel, exactly
    // like `egoAcceleration`/`hasPred`/`predIsCacc` above are CACC-only. Byte-identical for
    // Krauss/IDM/ACC/CACC (verified by the "Krauss/IDM/ACC/CACC byte-identical" parity tests).
    private static double FollowSpeedFor(
        ResolvedVType vType,
        double egoSpeed,
        double gap,
        double predSpeed,
        double predMaxDecel,
        double laneVehicleMaxSpeed,
        double dt,
        double time,
        ref int accControlMode,
        ref double accLastUpdateTime,
        ref int caccControlMode,
        double egoAcceleration,
        bool hasPred,
        bool predIsCacc,
        double levelOfService)
    {
        if (vType.CarFollowModel == "CACC")
        {
            return CaccModel.FollowSpeed(
                egoSpeed, gap, predSpeed, predMaxDecel, laneVehicleMaxSpeed, vType, dt, time,
                egoAcceleration, hasPred, predIsCacc,
                ref caccControlMode, ref accControlMode, ref accLastUpdateTime);
        }

        if (vType.CarFollowModel == "ACC")
        {
            return AccModel.FollowSpeed(
                egoSpeed, gap, predSpeed, predMaxDecel, laneVehicleMaxSpeed, vType, dt, time,
                ref accControlMode, ref accLastUpdateTime);
        }

        if (vType.CarFollowModel == "IDMM")
        {
            // MSCFModel_IDM.cpp:203-207's headwayTime-adaptation block, taken here (as the
            // CALLER) rather than inside IdmModel.V, so plain IDM's call below stays untouched.
            var headwayTimeOverride = IdmmModel.AdaptedHeadwayTime(vType.Tau, levelOfService);
            return IdmModel.FollowSpeed(egoSpeed, gap, predSpeed, laneVehicleMaxSpeed, vType, dt, headwayTimeOverride);
        }

        return vType.CarFollowModel == "IDM"
            ? IdmModel.FollowSpeed(egoSpeed, gap, predSpeed, laneVehicleMaxSpeed, vType, dt)
            : KraussModel.FollowSpeed(egoSpeed, gap, predSpeed, predMaxDecel, vType, dt);
    }

    // StopSpeedFor: MSCFModel_Krauss.cpp:100-107 stopSpeed (KraussModel.StopSpeed) vs.
    // MSCFModel_IDM.cpp:151-173 stopSpeed (IdmModel.StopSpeed) vs. MSCFModel_ACC.cpp:110-115
    // stopSpeed (AccModel.StopSpeed, itself a byte-identical pass-through to
    // KraussModel.StopSpeed -- see AccModel.StopSpeed's own header comment for why ACC's stopSpeed
    // is provably the same formula as Krauss's, not a distinct one) -- same `desSpeed`=
    // laneVehicleMaxSpeed convention as FollowSpeedFor above.
    // C11-iv: `levelOfService` mirrors FollowSpeedFor's own new parameter -- ONLY read by the
    // IDMM arm, unread/unused for Krauss/IDM/ACC/CACC (byte-identical, same argument as above).
    private static double StopSpeedFor(
        ResolvedVType vType,
        double speed,
        double gap,
        double laneVehicleMaxSpeed,
        double dt,
        double actionStepLengthSecs,
        double levelOfService = 1.0)
    {
        // C11-iii: CACC's stopSpeed (MSCFModel_CACC.cpp:148-158) is the SAME formula
        // ACC's/Krauss's own stopSpeed uses (see CaccModel.StopSpeed's own header comment) --
        // reuses AccModel.StopSpeed's byte-identical pass-through rather than a third duplicate.
        if (vType.CarFollowModel is "ACC" or "CACC")
        {
            return AccModel.StopSpeed(speed, gap, vType, dt, actionStepLengthSecs);
        }

        if (vType.CarFollowModel == "IDMM")
        {
            var headwayTimeOverride = IdmmModel.AdaptedHeadwayTime(vType.Tau, levelOfService);
            return IdmModel.StopSpeed(speed, gap, laneVehicleMaxSpeed, vType, dt, actionStepLengthSecs, headwayTimeOverride);
        }

        return vType.CarFollowModel == "IDM"
            ? IdmModel.StopSpeed(speed, gap, laneVehicleMaxSpeed, vType, dt, actionStepLengthSecs)
            : KraussModel.StopSpeed(gap, speed, vType, dt, actionStepLengthSecs);
    }

    // Desired free-flow speed term (Engine.cs's simplified single-constraint analog of
    // MSVehicle.cpp:2908's per-link `cfModel.freeSpeed(this, getSpeed(), seen, laneMaxV)` call).
    // Krauss: MSCFModel_Krauss never overrides freeSpeed, and our engine never calls the base
    // MSCFModel::freeSpeed braking-curve formula for this term (no upcoming-speed-limit-drop
    // lookahead is modeled here) -- so this stays the literal `laneVehicleMaxSpeed` value, exactly
    // the pre-C11 code (byte-identical: FinalizeSpeed's own aMax term is what actually bounds the
    // acceleration ramp toward it, matching every prior rung).
    // IDM: routes through IdmModel.FreeSpeed (MSCFModel_IDM.cpp:77-100) with `seen` set to
    // +infinity -- this engine has no "distance until the next lane's speed limit changes"
    // concept for this term, and the briefing's own scope note confirms that for a normal
    // free-flow lane (no speed-limit drop) `seen` is effectively unbounded, which collapses
    // freeSpeed to its free-accel branch (`speed<=maxSpeed`) exactly, both `maxSpeed` and
    // `desSpeed` being this same `laneVehicleMaxSpeed` value (ego's own current lane, no
    // separate "next lane" tracked here -- see IdmModel.FreeSpeed's own header comment).
    // C11-iv: IDMM routes through the SAME IdmModel.FreeSpeed, with the LOS-adapted headway
    // override (MSCFModel_IDM.cpp:203-207, taken here as the caller -- see FollowSpeedFor's own
    // C11-iv comment for why).
    private static double FreeFlowDesiredSpeedConstraint(VehicleRuntime v, double laneVehicleMaxSpeed, double dt)
    {
        if (v.VType.CarFollowModel == "IDMM")
        {
            var headwayTimeOverride = IdmmModel.AdaptedHeadwayTime(v.VType.Tau, v.LevelOfService);
            return IdmModel.FreeSpeed(v.Kinematics.Speed, double.PositiveInfinity, laneVehicleMaxSpeed, laneVehicleMaxSpeed, v.VType, dt, headwayTimeOverride);
        }

        return v.VType.CarFollowModel == "IDM"
            ? IdmModel.FreeSpeed(v.Kinematics.Speed, double.PositiveInfinity, laneVehicleMaxSpeed, laneVehicleMaxSpeed, v.VType, dt)
            : laneVehicleMaxSpeed;
    }

    // MSVehicle.cpp's planMoveInternal "process stops" block (~2467-2540), non-waypoint
    // (stop.getSpeed()==0) arm only: newStopDist = seen + endPos - lane->getLength(), which on a
    // single lane (seen = laneLength - pos) collapses to `endPos + NUMERICAL_EPS - pos`;
    // stopSpeed = MAX2(cfModel.stopSpeed(this, getSpeed(), newStopDist), vMinComfortable) where
    // vMinComfortable = cfModel.minNextSpeed(getSpeed()) (line 2191). Non-binding (+infinity)
    // once the stop is reached (matches the source's own approach-block guard) or absent.
    private double StopLineConstraint(VehicleRuntime v, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // D3: side table lookup instead of v.Stops.Count == 0 -- absent from _stopsByEntity is
        // exactly the "no stops" fast path.
        var stops = GetStops(v);
        if (stops is null || stops.Count == 0)
        {
            return double.PositiveInfinity;
        }

        var stop = stops.Peek();
        if (stop.Reached || stop.LaneId != v.LaneId)
        {
            return double.PositiveInfinity;
        }

        var newStopDist = stop.EndPos + KraussModel.NumericalEps - v.Kinematics.Pos;
        // MSVehicle.cpp:2191 `vMinComfortable = cfModel.minNextSpeed(getSpeed())` -- virtual;
        // IDM overrides minNextSpeed (see IdmModel.MinNextSpeed's own header comment), so this
        // dispatches too, not just the stopSpeed call below. C11-iv: IDMM shares MSCFModel_IDM's
        // SAME minNextSpeed override (MSCFModel_IDM.cpp:52-62 is unconditional, no
        // myAdaptationFactor/levelOfService term at all) -- so it routes through the identical
        // IdmModel.MinNextSpeed call, no LOS-adapted variant needed here.
        var vMinComfortable = v.VType.CarFollowModel is "IDM" or "IDMM"
            ? IdmModel.MinNextSpeed(v.Kinematics.Speed, v.VType, dt)
            : KraussModel.MinNextSpeed(v.Kinematics.Speed, v.VType, dt);
        var stopSpeed = StopSpeedFor(v.VType, v.Kinematics.Speed, newStopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);

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
    private double RedLightConstraint(VehicleRuntime v, Lane lane, double time, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
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

        // Rung A3: MSVehicle::ignoreRed (MSVehicle.cpp:7266). Scope: only the jm-privilege arm
        // (`ignoreRedTime > redDuration`) is ported -- the sibling `!canBrake` arm ("run the red
        // because you physically cannot stop in time") is NOT exercised by rung 10 (its passenger
        // can always brake) or by this scenario (the emergency vehicle ignores red via the jm
        // arm, not because it can't stop), so it is deliberately left out, consistent with rung
        // 10's existing RedLightConstraint. Also out of scope here: the yellow-arm and the
        // myInfluencer/TraCI early-return, neither reachable in this scenario. With the default
        // JmDriveAfterRedTime = -1 (VTypeDefaults.Resolve), `-1 > redDuration` (redDuration >= 0
        // always) is always false, so this is a no-op for every scenario that doesn't set
        // jmDriveAfterRedTime -- rung 10 stays byte-identical.
        var redDuration = TrafficLightState.GetPhaseElapsed(tlLogic, evalTime);
        if (v.VType.JmDriveAfterRedTime > redDuration)
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

        // C6 (yellow decision): the planMoveInternal gate at MSVehicle.cpp:2754 adds the yellow/red
        // stop-line brake ONLY when `canBrakeBeforeStopLine` -- the vehicle can still halt before
        // the line. If it is too close (cannot brake in time), NO brake is added and it PROCEEDS
        // through the light (the "dilemma zone" go decision). canBrakeBeforeStopLine
        // (MSVehicle.cpp:2648): `seen - lane.getVehicleStopOffset(this) >= brakeDist`, with the
        // stop offset 0 here (no vClass stop offset modeled). Byte-identical for rung 10 / the
        // emergency-red scenario: there the vehicle always approaches from far enough to stop, so
        // this is always true and never gates. (This is the mechanism the rung-A3 ignoreRed comment
        // above notes was deferred -- it is the standard yellow "go if you can't stop" behavior,
        // distinct from ignoreRed's jm-privilege arm.)
        const double vehicleStopOffset = 0.0;
        var canBrakeBeforeStopLine = seen - vehicleStopOffset >= brakeDist;
        if (!canBrakeBeforeStopLine)
        {
            return double.PositiveInfinity;
        }

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

        return StopSpeedFor(v.VType, v.Kinematics.Speed, stopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
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
    // start-of-step lane/position (the same `allVehicles` snapshot LaneNeighborQuery.Refill
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
    //
    // B5-iii: `time` (threaded in purely for ExternalAgentOnFoeLane's obstacle active-window
    // check below -- see this method's foe-link loop) is the only change to this method's
    // pre-existing 9b-ii/iii signature/logic; every SUMO-foe code path above and below is
    // untouched.
    private double JunctionYieldConstraint(VehicleRuntime v, ActiveVehicleQuery allVehicles, double time, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // Step 1: ego's own upcoming/current junction link -- the first internal lane in
        // the pool slice at or after LaneSeqIndex. A lane already passed is simply never found
        // by this forward-only scan (LaneSeqIndex has already advanced beyond it), which is
        // exactly the "already passed -> +infinity" case the briefing calls for.
        // D3: walk the pool slice, mapping handle -> Id for the LinkByInternalLane string lookup.
        var egoLinkSeqIndex = -1;
        string? egoInternalLaneId = null;
        for (var i = v.LaneSeqIndex; i < v.LaneSeqLen; i++)
        {
            var seqLaneId = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Id;
            if (_network!.LinkByInternalLane.ContainsKey(seqLaneId))
            {
                egoLinkSeqIndex = i;
                egoInternalLaneId = seqLaneId;
                break;
            }
        }

        if (egoInternalLaneId is null)
        {
            return double.PositiveInfinity;
        }

        var (junction, egoLink) = _network!.LinkByInternalLane[egoInternalLaneId];

        // D4: manual loop instead of `.FirstOrDefault(r => r.Index == egoLink.Index)` -- the
        // lambda captured `egoLink` from the enclosing scope, so it allocated a closure every
        // call; junction.Requests is small (one row per link), so a plain scan is the "simplest"
        // zero-alloc form (no precomputed index needed).
        JunctionRequest? request = null;
        foreach (var r in junction.Requests)
        {
            if (r.Index == egoLink.Index)
            {
                request = r;
                break;
            }
        }

        if (request is null)
        {
            return double.PositiveInfinity;
        }

        // D3: the pool slice at position egoLinkSeqIndex -- index it directly instead of
        // re-hashing the string already looked up above.
        var egoLane = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + egoLinkSeqIndex]];
        // The lane immediately before ego's internal lane in its route. Null only if the
        // internal lane is the very first element of the sequence -- which cannot happen for a
        // vehicle inserted on a normal lane (egoLinkSeqIndex >= 1 then), so it is used only in
        // the !egoOnInternal branches below, where it is always non-null. Guarded so a future
        // laneless/mid-junction insertion can't index -1 here.
        var approachLane = egoLinkSeqIndex >= 1
            ? _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + egoLinkSeqIndex - 1]]
            : null;
        var egoOnInternal = v.LaneId == egoInternalLaneId;

        // C4-ii: an all-way-stop junction uses a DISTINCT right-of-way rule from priority/RBL
        // junctions -- every approach must fully STOP first, then proceed in arrival order (longest
        // waiter first) -- so it takes its own arm and skips the priority-junction cautious-approach
        // + present-foe-yield logic below (which, being mutual at an all-way-stop, would deadlock).
        if (junction.Type == "allway_stop")
        {
            return AllwayStopConstraint(
                v, junction, egoLink, request, approachLane, egoOnInternal,
                allVehicles, dt, actionStepLengthSecs, laneVehicleMaxSpeed);
        }

        var constraint = double.PositiveInfinity;

        // C3 (TASKS.md "on-ramp merge" / minor-link CAUTIOUS APPROACH): ported from
        // MSVehicle::planMoveInternal's minor-link arm (sumo/src/microsim/MSVehicle.cpp:2655-2664
        // for the minor `laneStopOffset`/`stopDist`, :2734-2735 for the `stopSpeed` call, and the
        // :2805-2806 `couldBrakeForMinor && !determinedFoePresence` gate). This is the piece the
        // pre-existing foe-scan below CANNOT produce: a vehicle approaching a minor link must
        // decelerate toward the stop line even when NO foe is present/approaching, because it
        // "cannot see" whether the foe lanes are clear until it is within the link's foe-visibility
        // distance -- and only then may it re-accelerate and enter. Verified against the vendored
        // v1_20_0 DEBUG_PLAN_MOVE trace for scenarios/19-onramp-merge's `rA` (mA ~390m away, a huge
        // gap, so rA is NOT gap-blocked): seen=22.32 -> 11.906333, seen=10.41 -> 7.406333, released
        // at seen=3.01 (<= visibilityDistance) -> re-accelerates. In scenarios/11-priority-junction
        // this is byte-identical: a foe (vMajor) is approaching the whole time, so the foe-scan
        // below already brakes vMinor to the SAME stop line (stopDist == seen - POSITION_EPS), and
        // where the two overlap they compute the identical stopSpeed (verified 9.433 at seen=14.9),
        // so this Math.Min changes nothing there; far from the junction stopSpeed exceeds the
        // current speed (non-binding). Only applies while ego is still on its APPROACH lane
        // (!egoOnInternal) -- once it has entered its internal lane the link is behind it.
        //
        // "ego's link is minor" == its <request> row yields to at least one foe link
        // (Response has any set bit) -- equivalent to SUMO's `!(*link)->havePriority()` for a
        // priority junction (this rung's scope): a major link's Response row is all-zero.
        if (!egoOnInternal && approachLane is not null && request.Response.Contains('1'))
        {
            // NLHandler.cpp:1413: a link with no explicit `visibility` attribute defaults its
            // foe-visibility distance to 4.5 (for non-ZIPPER links) -- this net specifies none.
            const double visibilityDistance = 4.5;
            var seen = approachLane.Length - v.Kinematics.Pos;

            // couldBrakeForMinor (MSVehicle.cpp:2805): `!havePriority() && brakeDist < seen &&
            // !lastWasContMajor()`. The lastWasContMajor arm is inert here (ego's approach lane is
            // a normal lane, not a major continuation). `!determinedFoePresence` (:2794/:2806) is
            // `seen > visibilityDistance` -- once within visibility the vehicle re-accelerates and
            // the foe-scan below (or free-flow) governs instead. stopDecel = getMaxDecel() (4.5).
            var brakeDist = KraussModel.BrakeGap(v.Kinematics.Speed, v.VType.Decel, headwayTime: 0.0, dt);
            if (brakeDist < seen && seen > visibilityDistance)
            {
                // Minor-link stopDist (MSVehicle.cpp:2656-2664): laneStopOffset =
                // MIN2(visibilityDistance - POSITION_EPS, minorStopOffset), then the
                // avoid-emergency-braking clamp MIN2(., seen - brakeDist) when it can brake before
                // the lane end (always true inside `brakeDist < seen`), then MAX2(POSITION_EPS, .).
                // minorStopOffset = lane.getVehicleStopOffset(this) = 0 here (no vClass stop offset
                // modeled -- this net's <lane>s set none), mirroring RedLightConstraint's own
                // majorStopOffset treatment; with it 0 this resolves to POSITION_EPS, i.e.
                // stopDist = seen - POSITION_EPS (plan to be able to stop AT the junction stop line).
                const double minorStopOffset = 0.0;
                var laneStopOffset = Math.Min(visibilityDistance - PositionEps, minorStopOffset);
                laneStopOffset = Math.Min(laneStopOffset, seen - brakeDist); // canBrakeBeforeLaneEnd
                laneStopOffset = Math.Max(PositionEps, laneStopOffset);
                var stopDist = Math.Max(0.0, seen - laneStopOffset);

                constraint = Math.Min(
                    constraint,
                    StopSpeedFor(v.VType, v.Kinematics.Speed, stopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService));
            }
        }

        for (var j = 0; j < junction.IntLanes.Count; j++)
        {
            if (j == egoLink.Index || !request.RespondsTo(j))
            {
                continue;
            }

            // D4: manual loop instead of `.FirstOrDefault(c => c.EgoLink == ... && c.FoeLink ==
            // j)` -- that lambda captured both `egoLink` and `j`, allocating a closure every
            // call inside this per-vehicle, per-foe-link loop; junction.Conflicts is small, so a
            // plain scan is the "simplest" zero-alloc form.
            JunctionConflict? conflict = null;
            foreach (var c in junction.Conflicts)
            {
                if (c.EgoLink == egoLink.Index && c.FoeLink == j)
                {
                    conflict = c;
                    break;
                }
            }

            if (conflict is null)
            {
                // C4-iv: no geometric CROSSING is recorded for this foe link -- but a sameTarget
                // MERGE (ego's link and the foe's link feed the SAME downstream lane) still
                // requires ego to follow-yield to the merging foe. Own arm; +infinity when not a
                // merge, no foe, or the cautious approach still dominates (see the arm).
                constraint = Math.Min(
                    constraint,
                    SameTargetMergeConstraint(
                        v, junction, egoLink, egoInternalLaneId, egoOnInternal, approachLane,
                        j, allVehicles, dt, time, actionStepLengthSecs, laneVehicleMaxSpeed));
                continue;
            }

            var foeInternalLaneId = junction.IntLanes[j];
            // D3: resolve the foe internal lane's handle once, so both the foe-vehicle scan and
            // the sequence-index lookup below search the pool by handle, not by re-hashing the
            // string per candidate.
            var foeInternalLaneHandle = _network.LaneHandleById[foeInternalLaneId];

            // B5-iii: external-agent foe check, INDEPENDENT of FindFoeVehicle below -- this must
            // fire even when NO SUMO VehicleRuntime occupies/approaches this foe internal lane,
            // which is the pure-external-agent case (a navmesh/RVO agent is never a
            // VehicleRuntime FindFoeVehicle could find). It reuses the EXACT approaching-foe
            // stop-line yield the SUMO-foe branch below uses (the same KraussModel.StopSpeed call
            // against the approach lane's end) -- the only facts it needs are already established
            // above (ego responds to link `j`, and a geometric `conflict` is recorded for it).
            // Unlike a SUMO foe, an external agent has no lane-sequence index to compare against
            // an "approaching vs. on-junction vs. cleared" three-way split -- see
            // ExternalAgentOnFoeLane's own comment: an agent "clears" the junction purely by its
            // owner deactivating (EndTime) or removing it, never by a position-derived state
            // change, so lane membership alone is the complete signal and this always applies the
            // approaching-foe formula while the agent occupies the lane. Once ego itself has
            // already been granted entry (egoOnInternal) it is no longer gated, identical to the
            // SUMO-foe approaching branch's own egoOnInternal short-circuit.
            if (ExternalAgentOnFoeLane(foeInternalLaneId, time))
            {
                var extConstraint = egoOnInternal
                    ? double.PositiveInfinity
                    : StopSpeedFor(
                        v.VType, v.Kinematics.Speed,
                        approachLane!.Length - v.Kinematics.Pos - PositionEps,
                        laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
                constraint = Math.Min(constraint, extConstraint);
            }

            var foe = FindFoeVehicle(v, allVehicles, foeInternalLaneHandle);
            if (foe is null)
            {
                continue;
            }

            var foeInternalSeqIndex = IndexOfLaneHandle(foe, foeInternalLaneHandle);

            double thisConstraint;
            if (foe.LaneId == foeInternalLaneId)
            {
                // On-junction: MSVehicle::adaptToJunctionLeader.
                thisConstraint = AdaptToJunctionLeader(v, egoLane, approachLane, egoOnInternal, conflict, foe, dt, time, actionStepLengthSecs, laneVehicleMaxSpeed);
            }
            else if (foeInternalSeqIndex > foe.LaneSeqIndex)
            {
                // Approaching (foe hasn't reached its own internal lane yet): the stop-line
                // yield only guards ENTRY onto ego's own internal lane -- once ego has already
                // been granted entry (egoOnInternal), it is no longer gated by this foe's
                // approach state.
                thisConstraint = egoOnInternal
                    ? double.PositiveInfinity
                    : StopSpeedFor(
                        v.VType, v.Kinematics.Speed,
                        approachLane!.Length - v.Kinematics.Pos - PositionEps,
                        laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
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

    // C4-ii (TASKS.md "Remaining right-of-way" -- the ALL-WAY-STOP sub-rung). A distinct
    // right-of-way rule from priority/RBL: at an `type="allway_stop"` junction (netconvert emits
    // link state 'w', and a MUTUAL <request> response matrix -- each approach yields to the other)
    // every vehicle must come to a FULL STOP at the stop line, then proceed in arrival order --
    // the vehicle that started waiting earliest goes first. Ported from MSLink::opened /
    // blockedByFoe (sumo/src/microsim/MSLink.cpp:841 the must-stop-first gate, :938-945 the
    // arrival-order tie-break). Replaces the priority-junction cautious-approach + present-foe
    // yield for such junctions (which, being mutual here, would deadlock -- each yields to the
    // other forever; verified: the pre-C4-ii engine leaves BOTH vehicles halted at scenario 27's
    // junction indefinitely).
    //
    // Determinism/parallel-safety: reads only the FROZEN start-of-step snapshot -- ego's own
    // Pos/Speed/WaitingTime and each foe's WaitingTime/lane/LaneSeqIndex (WaitingTime is written
    // only in ExecuteMoves, each vehicle its own) plus the static <request>/conflict matrix -- and
    // writes nothing; the result is independent of vehicle processing order (CLAUDE.md rule 2/5).
    private double AllwayStopConstraint(
        VehicleRuntime v, Junction junction, JunctionLink egoLink, JunctionRequest request,
        Lane? approachLane, bool egoOnInternal, ActiveVehicleQuery allVehicles,
        double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // Once ego is on its own internal lane it has already been granted entry -- no longer
        // gated (same short-circuit as JunctionYieldConstraint's approaching-foe branch).
        if (egoOnInternal || approachLane is null)
        {
            return double.PositiveInfinity;
        }

        var egoSeen = approachLane.Length - v.Kinematics.Pos;
        var stopLineBrake = StopSpeedFor(
            v.VType, v.Kinematics.Speed, egoSeen - PositionEps,
            laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);

        // MSLink::opened (MSLink.cpp:841): `(myState == LINKSTATE_ALLWAY_STOP) && waitingTime == 0
        // => return false` -- until the vehicle has actually halted (WaitingTime > 0) the link is
        // NEVER open, so it must brake to the stop line and stop. This is what drives the full
        // stop (and, unlike the priority-junction cautious approach, it does NOT release once
        // within a visibility distance -- an all-way-stop vehicle always stops).
        if (v.WaitingTime <= 0.0)
        {
            return stopLineBrake;
        }

        // Has stopped: proceed unless a foe outranks ego by arrival order. For each responded-to
        // foe link with a geometric crossing (MSLink::blockedByFoe, MSLink.cpp:938-945):
        var constraint = double.PositiveInfinity;
        for (var j = 0; j < junction.IntLanes.Count; j++)
        {
            if (j == egoLink.Index || !request.RespondsTo(j))
            {
                continue;
            }

            JunctionConflict? conflict = null;
            foreach (var c in junction.Conflicts)
            {
                if (c.EgoLink == egoLink.Index && c.FoeLink == j)
                {
                    conflict = c;
                    break;
                }
            }

            if (conflict is null)
            {
                continue;
            }

            var foeInternalLaneId = junction.IntLanes[j];
            var foeInternalLaneHandle = _network!.LaneHandleById[foeInternalLaneId];
            var foe = FindFoeVehicle(v, allVehicles, foeInternalLaneHandle);
            if (foe is null)
            {
                continue;
            }

            var foeInternalSeqIndex = IndexOfLaneHandle(foe, foeInternalLaneHandle);
            if (foe.LaneId == foeInternalLaneId)
            {
                // Foe is crossing the junction right now -- ego cannot enter into/behind it,
                // regardless of who has waited longer.
                constraint = Math.Min(constraint, stopLineBrake);
            }
            else if (foeInternalSeqIndex > foe.LaneSeqIndex)
            {
                // Foe still approaching: yield ONLY if it outranks ego by arrival order --
                // MSLink::blockedByFoe's `waitingTime > avi.waitingTime` (foe waited strictly
                // longer), or the equal-wait `arrivalTime < avi.arrivalTime` tie-break. The exact
                // registered-arrivalTime path is out of scope (no committed scenario produces an
                // equal-wait tie -- scenario 27's two vehicles' waits differ by 3s); for
                // determinism the equal-wait case falls back to the fixed link-index order (lower
                // index first), which can never let both proceed at once.
                var foeOutranks = foe.WaitingTime > v.WaitingTime
                    || (foe.WaitingTime == v.WaitingTime && j < egoLink.Index);
                if (foeOutranks)
                {
                    constraint = Math.Min(constraint, stopLineBrake);
                }
            }
            // else: foe already cleared its internal lane -> no constraint from it.
        }

        return constraint;
    }

    // C4-iv (TASKS.md "sameTarget-merge yield"). Two junction links whose <connection>s feed the
    // SAME downstream lane (an on-ramp / roundabout-entry MERGE) geometrically converge rather than
    // cross, so no JunctionConflict is recorded -- yet a vehicle entering the merge must follow the
    // foe already traversing the other merging lane (or drive into it at the merge point).
    //
    // Ported (and VERIFIED against the v1_20_0 DEBUG_PLAN_MOVE_LEADERINFO getLeaderInfo trace for
    // scenarios/29 (rA) and 31 (vB)) from MSLink::getLeaderInfo's sameTarget branch
    // (sumo/src/microsim/MSLink.cpp:1379/1604-1663) + MSVehicle::adaptToJunctionLeader with
    // distToCrossing==-1 (MSVehicle.cpp:3223-3239). Two phases (the foe crosses a lane boundary):
    //   PHASE 1 -- foe on its own merging internal lane: it is a car-following LEADER; the gap is
    //     the trace's `gap = distToCrossing - egoMinGap - leaderBackDist`, which (crossingWidth 0
    //     for sameTarget) reduces to `distToMerge - egoMinGap - (foeInternalLen - foeBackPos)`.
    //     gap>=0 -> followSpeed; gap<0 -> stopSpeed to just before the junction entry (the
    //     `stopSpeed(speed, seen - lane.length - POSITION_EPS)` arm -- NOT raw followSpeed).
    //   PHASE 2 -- foe already on the shared TARGET lane while ego is upstream: an ordinary
    //     downstream leader across the boundary, gap = `distToMerge + (foePos - foeLen) - egoMinGap`.
    //
    // GATING (MSVehicle.cpp:3478 `v = MAX2(v, lastLink->myVLinkWait)`): while ego is still on its
    // APPROACH lane AND farther than the link's foe-visibility distance (4.5) from the junction
    // entry, the merge-leader is RELAXED to at least the cautious-approach stop-line speed -- i.e.
    // ego just brakes toward the entry (the C3 cautious approach already models that) and the merge
    // is NON-BINDING here. It binds only once ego is within visibility of the entry (seen<=4.5) or
    // has entered its internal lane -- exactly where the cautious approach itself releases. Without
    // this gate the merge over-brakes on the far approach (verified against the trace: at seen=41 m
    // SUMO's executed speed is the cautious 11.856, not the merge's 10.170).
    //
    // NOTE (asymmetric geometry): the exact SUMO gap also carries a small per-junction
    // `lengthBehindCrossing` term `(flbc - lbc)` (MSLink.cpp:354-382 angle-based conflictSize) that
    // this port does not yet compute -- it is ~0 for a SYMMETRIC merge (the two internal lanes are
    // mirror images, so `flbc==lbc` cancels; scenarios/31) but ~0.005 for an asymmetric one
    // (scenarios/29, a curved vs straight lane pair). Hence scenario 31 is the committed exact
    // anchor; 29 stays a geometry-refinement anchor until that term is ported.
    private double SameTargetMergeConstraint(
        VehicleRuntime ego, Junction junction, JunctionLink egoLink, string egoInternalLaneId,
        bool egoOnInternal, Lane? approachLane, int foeLinkIndex, ActiveVehicleQuery allVehicles,
        double dt, double time, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        if (approachLane is null && !egoOnInternal)
        {
            return double.PositiveInfinity;
        }

        // GATE: on the approach lane and still beyond foe-visibility of the entry -> the cautious
        // approach governs; the merge is non-binding (MSVehicle.cpp:3478 MAX2 relaxation).
        const double visibilityDistance = 4.5;
        if (!egoOnInternal && (approachLane!.Length - ego.Kinematics.Pos) > visibilityDistance)
        {
            return double.PositiveInfinity;
        }

        // Is foe link `foeLinkIndex` a sameTarget merge with ego's link? (connections share the
        // destination edge + lane -- MSLink.cpp:1379 `myLane == foeExitLink->getLane()`.)
        JunctionLink? foeLink = null;
        foreach (var l in junction.Links)
        {
            if (l.Index == foeLinkIndex)
            {
                foeLink = l;
                break;
            }
        }

        if (foeLink is null
            || foeLink.Connection.To != egoLink.Connection.To
            || foeLink.Connection.ToLane != egoLink.Connection.ToLane)
        {
            return double.PositiveInfinity;
        }

        var egoInternalLane = _network!.LanesById[egoInternalLaneId];
        var distToMerge = egoOnInternal
            ? egoInternalLane.Length - ego.Kinematics.Pos
            : (approachLane!.Length - ego.Kinematics.Pos) + egoInternalLane.Length;

        // PHASE 1: foe still on its merging internal lane.
        var foeInternalLaneId = junction.IntLanes[foeLinkIndex];
        var foeInternalLaneHandle = _network.LaneHandleById[foeInternalLaneId];
        var foeMerging = FindFoeVehicle(ego, allVehicles, foeInternalLaneHandle);
        if (foeMerging is not null && foeMerging.LaneId == foeInternalLaneId)
        {
            var foeInternalLane = _network.LanesByHandle[foeInternalLaneHandle];
            var leaderBack = foeMerging.Kinematics.Pos - foeMerging.VType.Length;

            // C4-v: place the merge crossing point via the static lengthBehindCrossing geometry
            // (MergeConflict, from MSLink::setRequestInformation). SUMO's gap (MSLink.cpp:1647,
            // sameTarget -> foeCrossingWidth 0) is `distToCrossing - egoMinGap - leaderBackDist`,
            // with distToCrossing = dist - egoLbc and leaderBackDist = (foeLen - foeLbc) - leaderBack.
            // With lbc==flbc==0 (the pre-C4-v approximation) this reduces to the old
            // `distToMerge - minGap - (foeLen - leaderBack)`; the (flbc - egoLbc) correction is 0 for
            // a symmetric merge and ~0.005 for an asymmetric one (verified against the debug trace).
            var (egoLbc, foeLbc) = MergeLengthsBehindCrossing(junction, egoLink.Index, foeLinkIndex);
            var distToCrossing = distToMerge - egoLbc;
            var leaderBackDist = (foeInternalLane.Length - foeLbc) - leaderBack;
            // MSLink.cpp:1633-1638: for a sameTarget merge, when the foe's back has passed the
            // crossing (leaderBackDist < 0), nudge it forward by the two lanes' crossing-point
            // mismatch (foeLbc - egoLbc, when positive) so both vehicles measure to the same point.
            var leaderBackDist2 = leaderBackDist;
            if (leaderBackDist2 < 0.0)
            {
                var mismatch = foeLbc - egoLbc;
                if (mismatch > 0.0)
                {
                    leaderBackDist2 += mismatch;
                }
            }

            var gap = distToCrossing - ego.VType.MinGap - leaderBackDist2;
            if (gap >= 0.0)
            {
                return FollowSpeedFor(
                    ego.VType, ego.Kinematics.Speed, gap, foeMerging.Kinematics.Speed, foeMerging.VType.Decel,
                    laneVehicleMaxSpeed, dt, time: time,
                    accControlMode: ref ego.AccControlMode, accLastUpdateTime: ref ego.AccLastUpdateTime,
                    caccControlMode: ref ego.CaccControlMode, egoAcceleration: ego.Acceleration,
                    hasPred: true, predIsCacc: foeMerging.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService);
            }

            // gap<0 (MSVehicle.cpp:3228): stop before entering the junction.
            return StopSpeedFor(
                ego.VType, ego.Kinematics.Speed, distToMerge - egoInternalLane.Length - PositionEps,
                laneVehicleMaxSpeed, dt, actionStepLengthSecs, ego.LevelOfService);
        }

        // PHASE 2: the foe has moved onto the shared target lane while ego is still upstream.
        var targetEdge = _network.EdgesById[egoLink.Connection.To];
        Lane? targetLane = null;
        foreach (var l in targetEdge.Lanes)
        {
            if (l.Index == egoLink.Connection.ToLane)
            {
                targetLane = l;
                break;
            }
        }

        if (targetLane is null)
        {
            return double.PositiveInfinity;
        }

        var leaderOnTarget = FindRearmostOnLane(ego, allVehicles, targetLane.Id);
        if (leaderOnTarget is null)
        {
            return double.PositiveInfinity;
        }

        var targetGap = distToMerge + (leaderOnTarget.Kinematics.Pos - leaderOnTarget.VType.Length) - ego.VType.MinGap;
        return FollowSpeedFor(
            ego.VType, ego.Kinematics.Speed, targetGap, leaderOnTarget.Kinematics.Speed, leaderOnTarget.VType.Decel,
            laneVehicleMaxSpeed, dt, time: time,
            accControlMode: ref ego.AccControlMode, accLastUpdateTime: ref ego.AccLastUpdateTime,
            caccControlMode: ref ego.CaccControlMode, egoAcceleration: ego.Acceleration,
            hasPred: true, predIsCacc: leaderOnTarget.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService);
    }

    // C4-iv phase-2 helper: the rearmost vehicle (smallest Pos = ego's immediate downstream leader)
    // CURRENTLY on the given lane, excluding ego. Frozen start-of-step snapshot.
    private VehicleRuntime? FindRearmostOnLane(VehicleRuntime ego, ActiveVehicleQuery allVehicles, string laneId)
    {
        VehicleRuntime? rearmost = null;
        foreach (var other in allVehicles)
        {
            if (ReferenceEquals(other, ego) || other.LaneId != laneId)
            {
                continue;
            }

            if (rearmost is null || other.Kinematics.Pos < rearmost.Kinematics.Pos)
            {
                rearmost = other;
            }
        }

        return rearmost;
    }

    // C4-v: the static (egoLbc, foeLbc) lengthBehindCrossing for a sameTarget merge pair
    // (computed once at ingest -- MergeConflict). (0, 0) when no MergeConflict is recorded for this
    // pair (a dummy merge, or geometry that produced none), matching the pre-C4-v approximation.
    private static (double EgoLbc, double FoeLbc) MergeLengthsBehindCrossing(Junction junction, int egoLink, int foeLink)
    {
        foreach (var m in junction.Merges)
        {
            if (m.EgoLink == egoLink && m.FoeLink == foeLink)
            {
                return (m.EgoLengthBehindCrossing, m.FoeLengthBehindCrossing);
            }
        }

        return (0.0, 0.0);
    }

    // B5-iii (TASKS.md "Junction foe the reducer yields to" -- the THIRD and final B5 sub-rung):
    // is any external, non-SUMO agent (navmesh/RVO agent, pedestrian, live detection -- the same
    // `_obstacles` store B1/B5-i/B5-ii already share) currently occupying the given foe internal
    // lane? This is the DESIGN.md "Two futures" live-input analog of FindFoeVehicle just below --
    // FindFoeVehicle answers the same question for a SUMO VehicleRuntime foe; this answers it for
    // an ExternalObstacle foe, and the two are checked independently (an external agent is never
    // wrapped as a VehicleRuntime, so FindFoeVehicle can never see it).
    //
    // Inert-when-absent (CLAUDE.md rule 3 / this rung's byte-identical-9b constraint):
    // `_obstacles.Count == 0` returns false immediately -- the SAME empty-store fast path
    // ObstacleConstraint/TargetLaneBlockedByObstacle's own header comments document -- so for
    // scenarios 11/08 and every other obstacle-free scenario/test this helper is a no-op and
    // JunctionYieldConstraint's foe-link loop is byte-identical to 9b-ii/iii's pre-existing logic
    // (the `if (ExternalAgentOnFoeLane(...))` guard is simply never entered, so the `Math.Min`
    // beside it never executes and `constraint` is only ever touched by the untouched SUMO-foe
    // path).
    //
    // Active-window/lane filter: identical to ObstacleConstraint's own (`StartTime <= time <
    // EndTime` and a lane-id match), evaluated at the SAME `time` this whole Plan phase reads
    // every other piece of frozen start-of-step state at (CLAUDE.md rule 2 -- reads `_obstacles`
    // only, exactly as AdvanceObstacles's own header comment requires; the Input phase already
    // dead-reckoned FrontPos for this step before Plan ever runs).
    //
    // "Clearing" the junction (documented, not modeled here as a position check): an external
    // agent's dead-reckoned `FrontPos` (AdvanceObstacles) never by itself changes its `LaneId` --
    // exactly like B5-i/B5-ii, the owning external layer is the sole authority on lane membership,
    // signaling a clearance by calling `RemoveObstacle` or letting the obstacle's own `EndTime`
    // elapse (UpdateObstacle only ever moves FrontPos/Speed, never LaneId). So lane-membership
    // alone (not a crossing-point/arc-length comparison the way a SUMO foe's on-junction/
    // approaching/cleared three-way split works) is the complete, correct signal here; a future
    // refinement that lets an agent's own reported position (rather than deactivation/removal)
    // signal "physically past the conflict point" is explicitly out of this rung's scope.
    private bool ExternalAgentOnFoeLane(string foeInternalLaneId, double time)
    {
        if (_obstacles.Count == 0)
        {
            return false;
        }

        foreach (var obstacle in _obstacles.Values)
        {
            if (obstacle.StartTime <= time && time < obstacle.EndTime && obstacle.LaneId == foeInternalLaneId)
            {
                return true;
            }
        }

        return false;
    }

    // MSVehicle::adaptToJunctionLeader (sumo/src/microsim/MSVehicle.cpp:3205-3307), Euler
    // branch only, and the gap formula it is fed (MSLink::getLeaderInfo, MSLink.cpp:1647).
    // `egoLane` is ego's own upcoming/current internal lane (the link this constraint was
    // raised for); `approachLane` is the lane immediately before it in ego's LaneSequence.
    // `foe` is already confirmed to be ON its own internal lane (foe.LaneId equals the
    // conflict's foe internal lane, i.e. `_network.LanesByHandle[foe.LaneHandle]` below IS that
    // lane) by JunctionYieldConstraint before calling in.
    private double AdaptToJunctionLeader(
        VehicleRuntime ego,
        Lane egoLane,
        Lane? approachLane,
        bool egoOnInternal,
        JunctionConflict conflict,
        VehicleRuntime foe,
        double dt,
        double time,
        double actionStepLengthSecs,
        double laneVehicleMaxSpeed)
    {
        // D2: hot per-vehicle lookup -- handle-indexed array instead of a string hash.
        var foeLane = _network!.LanesByHandle[foe.LaneHandle];

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
            vsafeLeader = FollowSpeedFor(
                ego.VType, ego.Kinematics.Speed, gap, foe.Kinematics.Speed, foe.VType.Decel, laneVehicleMaxSpeed, dt,
                time: time, accControlMode: ref ego.AccControlMode, accLastUpdateTime: ref ego.AccLastUpdateTime,
                caccControlMode: ref ego.CaccControlMode, egoAcceleration: ego.Acceleration,
                hasPred: true, predIsCacc: foe.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService);
        }
        else
        {
            // MSVehicle.cpp:3225-3228: leaderInfo.first != this is always true here (foe is a
            // distinct vehicle, never the ego "pedestrian" self-reference).
            vsafeLeader = StopSpeedFor(ego.VType, ego.Kinematics.Speed, seen - egoLane.Length - PositionEps, laneVehicleMaxSpeed, dt, actionStepLengthSecs, ego.LevelOfService);
        }

        if (distToCrossing >= 0)
        {
            // MSVehicle.cpp:3240-3280. leaderInfo.first == this (pedestrian) and
            // leaderInfo.second == -DBL_MAX (continuation-lane/opposite-direction foe) never
            // occur for this rung's foe-vehicle-on-a-plain-internal-lane case, so only the
            // final "else" branch (lines 3260-3280) is reachable here.
            var vStop = StopSpeedFor(ego.VType, ego.Kinematics.Speed, distToCrossing - ego.VType.MinGap, laneVehicleMaxSpeed, dt, actionStepLengthSecs, ego.LevelOfService);
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
    // D3: takes the foe internal lane's HANDLE (resolved once by the caller) instead of its
    // string id, and scans the candidate's pool slice instead of an IReadOnlyList<string>.
    private VehicleRuntime? FindFoeVehicle(VehicleRuntime ego, ActiveVehicleQuery allVehicles, int foeInternalLaneHandle)
    {
        // D6: "inserted, not arrived" via the reusable ActiveVehicleQuery passed in; only the
        // ego-exclusion check stays inline (specific to this call site).
        foreach (var other in allVehicles)
        {
            if (ReferenceEquals(other, ego))
            {
                continue;
            }

            if (IndexOfLaneHandle(other, foeInternalLaneHandle) >= 0)
            {
                return other;
            }
        }

        return null;
    }

    // D3: a tiny manual scan over a vehicle's pool slice `[LaneSeqStart, LaneSeqStart+LaneSeqLen)`
    // to find the position of a given lane HANDLE -- replaces the old string-keyed
    // IReadOnlyList<string>.Contains/manual-index scan over LaneSequence.
    private int IndexOfLaneHandle(VehicleRuntime v, int laneHandle)
    {
        for (var i = 0; i < v.LaneSeqLen; i++)
        {
            if (_laneSeqPool[v.LaneSeqStart + i] == laneHandle)
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
    private (double ReturnedVelocity, StopTransition? Transition) ProcessNextStop(
        VehicleRuntime v,
        double currentVelocity,
        double actionStepLengthSecs)
    {
        // D3: side table lookup instead of v.Stops.Count == 0 -- absent from _stopsByEntity is
        // exactly the "no stops" fast path.
        var stops = GetStops(v);
        if (stops is null || stops.Count == 0)
        {
            // MSVehicle.cpp:1614-1617: myStops.empty() -> return currentVelocity.
            return (currentVelocity, null);
        }

        var stop = stops.Peek();
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
    private static double LeaderFollowSpeedConstraint(VehicleRuntime ego, LaneNeighborQuery neighbors, double dt, double time, double laneVehicleMaxSpeed)
    {
        var leader = neighbors.GetLeader(ego);
        if (leader is null)
        {
            return double.PositiveInfinity;
        }

        var leaderBackPos = leader.Kinematics.Pos - leader.VType.Length;
        var gap = leaderBackPos - ego.VType.MinGap - ego.Kinematics.Pos;

        // C11-iii: `predIsCacc` -- MSCFModel_CACC.cpp:271 `pred->getCarFollowModel().getModelID()
        // != SUMO_TAG_CF_CACC` -- the leader's OWN resolved vType, read from the frozen `leader`
        // snapshot the neighbor query already returned (never re-read mid-step).
        return FollowSpeedFor(
            ego.VType,
            egoSpeed: ego.Kinematics.Speed,
            gap: gap,
            predSpeed: leader.Kinematics.Speed,
            predMaxDecel: leader.VType.Decel,
            laneVehicleMaxSpeed: laneVehicleMaxSpeed,
            dt: dt,
            time: time,
            accControlMode: ref ego.AccControlMode,
            accLastUpdateTime: ref ego.AccLastUpdateTime,
            caccControlMode: ref ego.CaccControlMode,
            egoAcceleration: ego.Acceleration,
            hasPred: true,
            predIsCacc: leader.VType.CarFollowModel == "CACC",
            levelOfService: ego.LevelOfService);
    }

    // B1/B5-i: external-obstacle constraint. Treats the nearest active obstacle ahead of `v` on
    // its current lane as a virtual leader, reusing the exact same KraussModel.FollowSpeed
    // leader-following formula LeaderFollowSpeedConstraint uses for a real vehicle leader -- so a
    // follower approaching a STATIC obstacle (Speed==0, B1's only case) brakes and settles at the
    // same Krauss steady gap it would hold behind a stopped real vehicle (verified against
    // scenario 13-stopped-leader's golden: follower front settles at 242.499 = obstacle back 245
    // - minGap 2.5 - NUMERICAL_EPS 0.001).
    //
    // predSpeed/predMaxDecel now come straight off the obstacle (B5-i), mirroring
    // LeaderFollowSpeedConstraint's real-leader call (predSpeed: leader.Kinematics.Speed,
    // predMaxDecel: leader.VType.Decel) exactly. predMaxDecel is conditional on Speed != 0: at
    // predSpeed=0, KraussModel.BrakeGap(0, ...) is 0 regardless of the decel argument, so for a
    // STATIC obstacle that argument is never actually used by the formula -- passing the ego's
    // own decel (`v.VType.Decel`) there, exactly as B1 always did, is what keeps a Speed==0
    // obstacle byte-identical to B1's output (nothing about the emitted trajectory can depend on
    // an argument the formula provably ignores). For a MOVING obstacle (Speed != 0) the real
    // obstacle.MaxDecel is used, since BrakeGap(predSpeed, ...) is no longer trivially zero.
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
    private double ObstacleConstraint(VehicleRuntime v, double time, double laneVehicleMaxSpeed)
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

        // C11-iii: an ExternalObstacle has no CarFollowModel of its own (it is not a SUMO
        // vehicle) -- `predIsCacc` is always false here, exactly matching CACC's own
        // `pred->getCarFollowModel().getModelID() != SUMO_TAG_CF_CACC` ACC-fallback test for any
        // non-CACC leader (a static/moving obstacle is never CACC).
        return FollowSpeedFor(
            v.VType,
            egoSpeed: v.Kinematics.Speed,
            gap: gap,
            predSpeed: nearest.Speed,
            predMaxDecel: nearest.Speed != 0.0 ? nearest.MaxDecel : v.VType.Decel,
            laneVehicleMaxSpeed: laneVehicleMaxSpeed,
            dt: _config!.StepLength,
            time: time,
            accControlMode: ref v.AccControlMode,
            accLastUpdateTime: ref v.AccLastUpdateTime,
            caccControlMode: ref v.CaccControlMode,
            egoAcceleration: v.Acceleration,
            hasPred: true,
            predIsCacc: false,
            levelOfService: v.LevelOfService);
    }

    // Execute phase: apply each vehicle's own MoveIntent and integrate position.
    // Integration method is a config flag per DESIGN.md (step-method.ballistic), not hard-coded:
    //  - Euler (gSemiImplicitEulerUpdate=true, the phase-1 default, every scenario but 21): the
    //    whole step is taken at the NEW speed -- pos += newSpeed * dt.
    //  - C8-i ballistic (step-method.ballistic=true): the trapezoidal update SUMO uses when
    //    !gSemiImplicitEulerUpdate -- pos += 0.5*(oldSpeed + newSpeed)*dt (the vehicle is treated
    //    as accelerating linearly across the step, so it covers the average of its start/end
    //    speeds). Verified against scenarios/21-ballistic-freeflow's golden (t=1 pos 1.30 =
    //    0.5*(0+2.6)*1; t=6 pos 45.945 = 32.5 + 0.5*(13.0+13.89)*1). Scope: this is the free-flow
    //    ballistic POSITION update only; the ballistic SAFE-SPEED branches
    //    (maximumSafeStopSpeedBallistic / followSpeed / finalizeSpeed) are deferred to a
    //    ballistic-car-following scenario (they never bind free-flow, where the speed sequence is
    //    identical to Euler). Byte-identical to the old code when Ballistic=false.
    private void ExecuteMoves(double dt)
    {
        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            // C8-i: capture the pre-move speed BEFORE overwriting it, for the ballistic
            // trapezoidal position update below (Euler ignores it).
            var oldSpeed = v.Kinematics.Speed;
            // C11-iii: MSVehicle::getAcceleration()'s analog -- the (speed-oldSpeed)/dt this
            // vehicle just realized THIS step, written here so CaccModel's cooperative
            // gap-control law can read it as "last completed step's acceleration" from the
            // FOLLOWING step's Plan phase (see VehicleRuntime.Acceleration's own header comment).
            // Written for every vehicle, unconditionally -- read by nothing except CACC.
            v.Acceleration = (v.Intent.NewSpeed - oldSpeed) / dt;
            v.Kinematics.Speed = v.Intent.NewSpeed;

            // C4-ii: waiting-time accumulation (MSVehicle::updateWaitingTime, MSVehicle.cpp:4081-4088).
            // `+= dt` while halted (speed <= SUMO_const_haltingSpeed 0.1) and not accelerating away
            // (this step's acceleration <= accelThresholdForWaiting = 0.5*maxAccel, MSVehicle.h:2059);
            // else reset. isStopped()/isIdling() (scheduled <stop>s) and the influencer branch are
            // out of scope (no scenario here schedules a stop), so `!isStopped()` is constant-true
            // and omitted. Written unconditionally for every vehicle; read only by the all-way-stop
            // arm of JunctionYieldConstraint.
            v.WaitingTime = v.Intent.NewSpeed <= KraussModel.HaltingSpeed && v.Acceleration <= 0.5 * v.VType.Accel
                ? v.WaitingTime + dt
                : 0.0;
            v.Kinematics.Pos += _config!.Ballistic
                ? 0.5 * (oldSpeed + v.Intent.NewSpeed) * dt
                : v.Intent.NewSpeed * dt;
            v.Kinematics.LatOffset = v.Intent.LatOffset;

            // Rung 5: apply the plan phase's proposed stop-queue update (Engine.ProcessNextStop).
            // This is the only place a vehicle's stop queue is ever mutated (CLAUDE.md rule 3).
            // D3: side table lookup instead of v.Stops -- StopUpdate is only ever non-null when
            // ProcessNextStop found a non-empty queue for this vehicle this same step, so `stops`
            // is guaranteed non-null here.
            if (v.Intent.StopUpdate is { } stopUpdate)
            {
                var stops = GetStops(v)!;
                if (stopUpdate.Resume)
                {
                    stops.Dequeue();
                }
                else
                {
                    var stop = stops.Peek();
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
                // D2: hot per-vehicle, per-step lookup -- handle-indexed array instead of a
                // string hash.
                var currentLane = _network!.LanesByHandle[v.LaneHandle];
                if (v.Kinematics.Pos < currentLane.Length)
                {
                    break;
                }

                // C2-ii (design point 4): the boundary advance may only cross to
                // pool[LaneSeqIndex+1] once the vehicle's ACTUAL lane has CONVERGED onto the
                // route pool's target lane for this edge -- TryStrategicLaneChange guarantees
                // this before the lane end for every reachable route (scenario 18 converges at
                // t=17, long before edge B). For every single-lane-per-edge route the pool is
                // built from the depart lane itself (NetworkModel.ResolveLaneSequence), so
                // actual ALWAYS equals the pool's target there and this guard is always
                // satisfied -- byte-identical to every existing scenario. If NOT converged
                // (stuck on a drop lane with no successful strategic change -- not exercised by
                // any committed golden, but required for safety per the briefing), STOP at the
                // lane end rather than teleport onto a route path this lane was never actually
                // connected to.
                if (v.LaneHandle != _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex])
                {
                    v.Kinematics.Pos = currentLane.Length;
                    v.Kinematics.Speed = 0.0;
                    break;
                }

                if (v.LaneSeqIndex + 1 >= v.LaneSeqLen)
                {
                    // D5: deferred through the command buffer, flushed at the END of this
                    // method's outer foreach (see below) -- safe because the `break` right
                    // after this, not the `while (!v.Arrived)` condition, is what exits this
                    // loop (the condition is never RE-evaluated after this assignment within
                    // this same call), and nothing later in this vehicle's own iteration or any
                    // OTHER vehicle's iteration this SAME ExecuteMoves pass reads v.Arrived
                    // (the outer foreach's own `if (!v.Inserted || v.Arrived) continue;` guard
                    // only ever reads a vehicle's OWN Arrived value, set at the top of ITS OWN
                    // iteration, never another vehicle's just-this-step arrival).
                    _commandBuffer.Destroy(v);
                    break;
                }

                v.Kinematics.Pos -= currentLane.Length;
                v.LaneSeqIndex++;
                // D3: keep LaneHandle/LaneId in lockstep -- direct pool-slice read, no string
                // hash.
                v.LaneHandle = _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex];
                v.LaneId = _network.LanesByHandle[v.LaneHandle].Id;
            }

            // Rung 8b/A2: keep-right and speed-gain lane changes are no longer decided here --
            // both now run in the post-move DecideSpeedGainChanges phase (see Run()'s comment and
            // that method's header comment for why keep-right moved out of Plan/MoveIntent).
        }

        // D5: apply every Destroy recorded above, in record order, at this method's end -- the
        // SAME point v.Arrived took effect at before this rung, still strictly before
        // DecideSpeedGainChanges (called next, in Run()) reads it via its own postMoveNeighbors
        // Refill / `!v.Inserted || v.Arrived` guard.
        _commandBuffer.Flush();
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
    //
    // B5-ii (TASKS.md "Cross-lane blocker vetoing lane changes"): `time` is now threaded through
    // (alongside `dt`) purely so TargetLaneBlockedByObstacle below can evaluate an
    // ExternalObstacle's [StartTime, EndTime) active window at the same instant every other
    // read this phase uses -- it is not otherwise used by the pre-existing keep-right/speed-gain
    // math above it.
    private void DecideSpeedGainChanges(double time, double dt)
    {
        const double relGainNormalizationMinSpeed = 10.0; // MSLCM_LC2013.cpp RELGAIN_NORMALIZATION_MIN_SPEED
        const double changeProbThresholdLeft = 0.2; // ctor: (0.2/mySpeedGainParam), default mySpeedGainParam=1
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        // Refilled ONCE per step, from the now-settled post-move positions every vehicle's
        // keep-right/speed-gain decision below reads -- SUMO's changeLanes phase sees all
        // vehicles already moved this step (MSNet.cpp:784/790/796), so this is the correct (and
        // only) frozen snapshot for this phase, distinct from PlanMovements' pre-move `neighbors`
        // snapshot. D4: reuses the SAME `_neighborQuery` instance Run() refilled for the pre-move
        // snapshot -- safe because PlanMovements (the pre-move snapshot's only reader) has
        // already fully completed by the time this Refill overwrites it (see LaneNeighborQuery's
        // header comment).
        var postMoveNeighbors = _neighborQuery!;
        postMoveNeighbors.Refill(ActiveVehicles());

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            // Keep-right (rung 8b) evaluated FIRST, against this iteration's starting lane; may
            // update v.LaneId/v.KeepRightProbability directly (own comment: same reasoning as the
            // speed-gain veto below for why a direct write here still honors CLAUDE.md rule 3).
            ApplyKeepRightDecision(v, postMoveNeighbors, dt);

            // D2: hot per-vehicle, per-step lookup -- handle-indexed array instead of a string
            // hash. (ApplyKeepRightDecision above may have just changed v.LaneHandle; re-read.)
            var lane = _network!.LanesByHandle[v.LaneHandle];

            // C2-ii: strategic (route-driven) lane change -- evaluated BEFORE speed-gain and,
            // when it fires, taking priority over it (a vehicle never both strategic- and
            // speed-gain-changes in the same step; mirrors SUMO's _wantsChange trying the
            // STRATEGIC/URGENT block first, MSLCM_LC2013.cpp:1324-1327, before ever reaching
            // the speed-gain code). Inert (returns false immediately, touching nothing -- not
            // even v.LookAheadSpeed) for every existing scenario -- see
            // TryStrategicLaneChange's own header comment for the exact gate/byte-identical
            // argument.
            if (TryStrategicLaneChange(v, lane, postMoveNeighbors, time, dt, actionStepLengthSecs))
            {
                continue;
            }

            // Left neighbor = same edge, index+1 (no neighbor on the leftmost lane) -- this
            // guard is the INERT case CLAUDE.md's briefing calls for: single-lane rungs, and any
            // vehicle already on the leftmost lane (e.g. this same follower once it has already
            // changed left), leave SpeedGainProbability untouched and never fire a change. D4:
            // `lane.LeftNeighbor` is precomputed at ingest (NetworkParser) -- O(1) array read
            // instead of a per-step `edge.Lanes.FirstOrDefault(...)` LINQ scan/closure.
            if (lane.LeftNeighbor < 0)
            {
                continue;
            }

            var leftLane = _network.LanesByHandle[lane.LeftNeighbor];

            var vMax = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);
            var neighVMax = KraussModel.LaneVehicleMaxSpeed(leftLane.Speed, v.SpeedFactor, v.VType);

            // MSLCM_LC2013.cpp:1109-1136's best-lanes continuation distance, simplified per the
            // A2 briefing's scope note to this single-edge dead-end scenario: laneLength minus
            // the vehicle's (post-move) position on it. Large enough here (~2000/~1865) that it
            // never binds the no-leader stop-speed fallback or the `>20` gate below -- a full
            // multi-edge best-lanes port is out of scope until a scenario actually needs it.
            var currentDist = lane.Length - v.Kinematics.Pos;
            var neighDist = leftLane.Length - v.Kinematics.Pos;

            var leader = postMoveNeighbors.GetLeader(v);
            var thisLaneVSafe = Math.Min(vMax, AnticipateFollowSpeed(v, leader, currentDist, dt));

            var neighLead = postMoveNeighbors.GetNeighborLeader(v, leftLane.Handle);
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
            var targetLaneHandle = 0;
            if (speedGainProbability > changeProbThresholdLeft
                && relativeGain > KraussModel.NumericalEps
                && neighDist / Math.Max(0.1, v.Kinematics.Speed) > 20.0)
            {
                // :1857-1864 fires. Target-lane safety veto (A2-iii) before committing --
                // MSLCM_LC2013::checkBlocking's role, minimal-faithful here (see
                // IsTargetLaneSafe). A vetoed change does NOT reset the accumulator (SUMO only
                // resets on an actually-committed change, :1063/1080) -- it keeps accumulating
                // and is retried next step.
                //
                // B5-ii: a SECOND, independent veto -- an external (non-SUMO) obstacle currently
                // occupying `leftLane` -- is ANDed in alongside the real-neighbor safety check.
                // Exactly the same "no reset on veto" semantics apply: TargetLaneBlockedByObstacle
                // returning true just leaves `targetLaneId` null for this step, same as
                // IsTargetLaneSafe returning false above, so the vehicle keeps its lane, keeps
                // accumulating SpeedGainProbability, and retries next step -- it changes as soon
                // as the obstacle clears. Inert-when-absent (CLAUDE.md rule 3): with no obstacle
                // ever added to `_obstacles`, TargetLaneBlockedByObstacle's own empty-store fast
                // path returns false unconditionally, so this `&&` is byte-identical to today's
                // bare `IsTargetLaneSafe(...)` gate for scenario 12 and every other
                // obstacle-free scenario/test.
                var neighFollow = postMoveNeighbors.GetNeighborFollower(v, leftLane.Handle);
                if (IsTargetLaneSafe(v, neighLead, neighFollow, dt) && !TargetLaneBlockedByObstacle(v, leftLane, time, dt))
                {
                    targetLaneId = leftLane.Id;
                    targetLaneHandle = leftLane.Handle;
                    speedGainProbability = 0.0; // :1063/1080 resetState() on committed change.
                }
            }

            v.SpeedGainProbability = speedGainProbability;

            // Structural change: instant lane-index snap (lanechange.duration=0), exactly like
            // rung 8b's keep-right swap in ExecuteMoves. D5: recorded through the command
            // buffer rather than applied inline -- safe to DEFER to this method's end because
            // (a) this is already the LAST thing this vehicle's own iteration does this phase
            // (nothing later in THIS iteration re-reads v.LaneId/LaneHandle), and (b) no OTHER
            // vehicle's decision this same phase reads it either -- every vehicle's
            // keep-right/speed-gain lookups go through the ONE frozen `postMoveNeighbors`
            // snapshot built once at the top of this method, never a live read of another
            // vehicle's current LaneId (see this method's own header comment). Contrast with
            // ApplyKeepRightDecision's swap below, which stays INLINE precisely because THIS
            // SAME vehicle's THIS SAME iteration re-reads v.LaneHandle right after calling it
            // (the "may have just changed v.LaneHandle; re-read" comment above) -- deferring
            // that one would change which lane the speed-gain decision runs against.
            if (targetLaneId is not null)
            {
                _commandBuffer.ChangeLane(v, targetLaneHandle, targetLaneId);
            }
        }

        // D5: apply every ChangeLane recorded above, in record order, at this method's end --
        // the SAME point v.LaneId/LaneHandle took effect at before this rung (DecideSpeedGain-
        // Changes is the LAST phase in Run()'s per-step loop, so this flush lands exactly where
        // the inline writes used to, before EmitTrajectory reads LaneId at the top of next
        // step's iteration).
        _commandBuffer.Flush();
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
        // D2: hot per-vehicle, per-step lookup -- handle-indexed array instead of a string hash.
        var lane = _network!.LanesByHandle[v.LaneHandle];

        // Right neighbor = same edge, index-1 (no neighbor when already on index 0) -- this
        // guard is exactly what leaves single-lane rungs 1/3/4/5/6/7 and 8a (vehicle on index 0)
        // completely unaffected: the accumulator simply never advances off 0. D4:
        // `lane.RightNeighbor` is precomputed at ingest (NetworkParser) -- O(1) array read
        // instead of a per-step `edge.Lanes.FirstOrDefault(...)` LINQ scan/closure.
        if (lane.RightNeighbor < 0)
        {
            return;
        }

        var rightLane = _network.LanesByHandle[lane.RightNeighbor];

        // actionStepLength=1 in this scenario's config (phase-1 determinism ladder).
        const double keepRightTime = 5.0; // MSLCM_LC2013.cpp:67 KEEP_RIGHT_TIME
        const double changeProbThresholdRight = 2.0; // ctor: (0.2/mySpeedGainRight)/mySpeedGainParam, defaults 0.1/1
        const double keepRightParam = 1.0; // ctor default (LCA_KEEPRIGHT_PARAM)
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;

        var vMax = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);
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
        var neighLead = neighbors.GetNeighborLeader(v, rightLane.Handle);
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
            // C2-ii: a narrow, commit-only port of MSLCM_LC2013.cpp:1398-1410's "opposite
            // direction" STAY guard -- SUMO's real guard makes _wantsChange return EARLY
            // (before myKeepRightProbability is ever decremented) whenever changing in the
            // examined direction would move away from the best/required lane with too little
            // room to get back; scoped here to just its OBSERVABLE effect (never actually
            // commit onto a lane that does not continue this vehicle's remaining route), not
            // the full early-return-before-accumulation semantics -- KeepRightProbability
            // itself is not a compared golden attribute (only lane/pos/speed are), so this
            // narrower, lower-risk port is sufficient. Byte-identical for every existing
            // single-edge-route scenario (06/07/12/...): LaneContinuesRoute's own fast path
            // returns true unconditionally there (see its own comment), so this `if` is never
            // entered and the commit below is untouched.
            if (!LaneContinuesRoute(v, lane, rightLane.Index))
            {
                v.KeepRightProbability = keepRightProbability;
                return;
            }

            // MSLCM_LC2013.cpp:1789/1061-1064: fires -> lane change requested, accumulator resets
            // to 0 on change (changed()/resetState()). No safety/blocker veto ported here -- every
            // scenario reaching this fire has an empty target (right) lane; a real blocker veto
            // wants its own scenario with target-lane traffic on the RIGHT side (mirrors A2-iii's
            // scope note for the LEFT side).
            //
            // D5: deliberately kept INLINE, NOT routed through the command buffer. The caller
            // (DecideSpeedGainChanges) re-reads `v.LaneHandle` immediately after this call
            // returns ("ApplyKeepRightDecision above may have just changed v.LaneHandle;
            // re-read") to pick the left-neighbor lane for THIS SAME vehicle's speed-gain
            // decision this SAME phase -- a genuine same-vehicle, same-iteration
            // read-after-write. A command buffer flushed at the phase barrier (end of
            // DecideSpeedGainChanges) would leave that re-read seeing the STALE pre-swap lane,
            // changing which lane the speed-gain decision runs against (verified needed by rung
            // A2's scenario 12, see DecideSpeedGainChanges' CORRECTED-ORDERING comment) --
            // exactly the CLAUDE.md rule 4 / this rung's briefing exception: "a command buffer
            // flushed at a phase barrier is only valid where no same-phase reader depends on the
            // write". This write does NOT cross vehicles (every other vehicle's neighbor lookups
            // this phase go through the frozen `postMoveNeighbors` snapshot, never a live read of
            // `v`'s LaneId), so it stays safe/deterministic despite being applied immediately.
            v.LaneId = rightLane.Id;
            // D2: keep LaneHandle in lockstep -- rightLane's own Handle field, no lookup.
            v.LaneHandle = rightLane.Handle;
            v.KeepRightProbability = 0.0;
            return;
        }

        v.KeepRightProbability = keepRightProbability;
    }

    // C2-ii: does `targetLaneIndex` on `fromLane`'s edge continue this vehicle's REMAINING
    // route? A thin wrapper over `NetworkModel.ComputeBestLanes` (C2-i) used ONLY as
    // ApplyKeepRightDecision's commit-time veto (see that method's own comment) -- returns
    // `true` (never vetoes) for a single-edge route WITHOUT even calling ComputeBestLanes
    // (its own "last route edge" special case would report `AllowsContinuation=true` for
    // every lane there anyway; this fast path just avoids the call for the common case),
    // which is exactly why 06/07/12-overtake's existing keep-right returns are byte-identical.
    private bool LaneContinuesRoute(VehicleRuntime v, Lane fromLane, int targetLaneIndex)
    {
        var route = _demand!.RoutesById[v.Def.RouteId];
        if (route.Edges.Count <= 1)
        {
            return true;
        }

        var bestLanes = _network!.ComputeBestLanes(route.Edges, fromLane.EdgeId);
        foreach (var continuation in bestLanes)
        {
            if (continuation.LaneIndex == targetLaneIndex)
            {
                return continuation.AllowsContinuation;
            }
        }

        // Defensive: lane not found on this edge's best-lanes result -- do not veto.
        return true;
    }

    // C2-ii: MSLCM_LC2013's STRATEGIC/URGENT block (`_wantsChange`,
    // sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp ~1216-1327), scoped to the single-look-ahead
    // case C2-i's `ComputeBestLanes` supports: a vehicle whose ACTUAL lane differs from its
    // route pool's target lane on this SAME edge (a drop lane -- scenarios/18-strategic-
    // turnlane's E1_0) must strategic-change toward the target before running off the end of
    // the lane. Ported pieces (constants/derivation cross-checked against the vendored
    // source):
    //   - myLookAheadSpeed growth/decay (`.cpp:1227-1236`, `VehicleRuntime.LookAheadSpeed`).
    //   - laDist (`.cpp:1238-1239`): myLookAheadSpeed * LOOK_FORWARD(10) * myStrategicParam(1.0
    //     ctor default) * (right ? 1 : myLookaheadLeft(2.0 ctor default)) + 2 * lengthWithGap.
    //   - usableDist/currentDistDisallows (`.h:189-191`, `.cpp:1288,1324-1327`):
    //     changeToBest && bestLaneOffset==curr.bestLaneOffset && usableDist/|offset| < laDist
    //     -- changeToBest and bestLaneOffset==curr.bestLaneOffset both collapse to trivially
    //     true here because this method only evaluates the ONE direction `bestLaneOffset`
    //     (the actual lane's own LaneQ field) itself requires, exactly the simplification
    //     SUMO's own two-sided caller (wantsChangeLeft/wantsChangeRight) converges to for a
    //     single-offset lane (see this method's own derivation notes below `right`).
    //
    // NOT ported (out of scope -- no committed scenario needs them yet): the stopped-leader/
    // bidi-lane laDist overrides (`.cpp:1240-1268`), the roundabout bonus, occupation/jam terms
    // (best.occupation is always 0 here -- an empty-road simplification consistent with this
    // file's existing scope notes elsewhere, e.g. A2-iii/keep-right's own "empty target lane"
    // scoping), multi-lane-offset (|offset|>1) blocker-length reservation (`.cpp:1471-1479`),
    // and the STAY/"opposite direction" guard (`.cpp:1398-1441`) -- that guard's only needed
    // EFFECT for this scenario (never keep-right back onto a lane that would undo an already-
    // converged strategic requirement) is instead ported narrowly into
    // ApplyKeepRightDecision's own commit gate (see `LaneContinuesRoute` above) rather than as
    // a full early-return here, since a fuller port risked perturbing scenario 12's existing
    // (golden-verified) keep-right/speed-gain byte-identical behavior without a way to verify
    // it offline (CLAUDE.md rule: no SUMO install in this loop).
    //
    // Gate (design point 3) / byte-identical argument: only evaluated when the ACTUAL lane
    // differs from the route pool's target lane's HANDLE on this SAME edge -- i.e.
    // `pool[LaneSeqIndex] != v.LaneHandle`. For every single-lane-per-edge scenario (and any
    // scenario where the depart lane already IS the continuing lane), NetworkModel.
    // ResolveLaneSequence builds the pool from the depart lane itself, so this is ALWAYS false
    // there -- the whole method returns immediately, touching NOTHING (not even
    // v.LookAheadSpeed, not even a ComputeBestLanes call), which is the byte-identical
    // argument for every existing multi-lane (06/07/12) and multi-edge (9a/9b/A3/15-reroute)
    // parity scenario.
    private bool TryStrategicLaneChange(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, double time, double dt, double actionStepLengthSecs)
    {
        if (v.LaneSeqIndex >= v.LaneSeqLen)
        {
            return false;
        }

        var targetLaneHandle = _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex];
        if (targetLaneHandle == v.LaneHandle)
        {
            // Already converged onto the route path -- the common/inert case for every
            // existing scenario.
            return false;
        }

        var targetLane = _network!.LanesByHandle[targetLaneHandle];
        if (targetLane.EdgeId != lane.EdgeId)
        {
            // Defensive only: ExecuteMoves' convergence guard (design point 4) never advances
            // LaneSeqIndex past an edge boundary while actual != target, so a well-formed pool
            // can never reach this state.
            return false;
        }

        var route = _demand!.RoutesById[v.Def.RouteId];
        var bestLanes = _network.ComputeBestLanes(route.Edges, lane.EdgeId);

        LaneContinuation? curr = null;
        foreach (var continuation in bestLanes)
        {
            if (continuation.LaneIndex == lane.Index)
            {
                curr = continuation;
                break;
            }
        }

        if (curr is null || curr.BestLaneOffset == 0)
        {
            // Defensive only: the pool-mismatch gate above already implies a nonzero offset --
            // that is exactly what NetworkModel.ResolveLaneSequence used to pick the pool's
            // target lane index in the first place.
            return false;
        }

        var bestLaneOffset = curr.BestLaneOffset;
        // Direction is fully determined by the offset's sign -- see this method's own header
        // comment for why evaluating only this one (correctly-signed) direction is equivalent
        // to SUMO's two-sided caller for the STRATEGIC/URGENT trigger.
        var right = bestLaneOffset < 0;

        // MSLCM_LC2013.cpp:1227-1236: grows instantly toward a higher speed, decays slowly
        // otherwise (per-vehicle persistent state).
        if (v.Kinematics.Speed > v.LookAheadSpeed)
        {
            v.LookAheadSpeed = v.Kinematics.Speed;
        }
        else
        {
            const double lookAheadSpeedMemory = 0.9; // LOOK_AHEAD_SPEED_MEMORY
            var memoryFactor = 1.0 - (1.0 - lookAheadSpeedMemory) * actionStepLengthSecs;
            v.LookAheadSpeed = Math.Max(0.0, (memoryFactor * v.LookAheadSpeed) + ((1.0 - memoryFactor) * v.Kinematics.Speed));
        }

        const double lookForward = 10.0; // LOOK_FORWARD
        const double strategicParam = 1.0; // myStrategicParam ctor default
        const double lookaheadLeft = 2.0; // myLookaheadLeft ctor default
        var lengthWithGap = v.VType.Length + v.VType.MinGap;
        var laDist = (v.LookAheadSpeed * lookForward * strategicParam * (right ? 1.0 : lookaheadLeft)) + (2.0 * lengthWithGap);

        // usableDist = MAX2(currentDist - posOnLane - best.occupation*JAM_FACTOR,
        // driveToNextStop): best.occupation is always 0 (empty-road scope, see this method's
        // header comment) and there is no stop on this edge in any committed C2-ii scenario, so
        // driveToNextStop is non-binding against the first MAX2 argument.
        var currentDist = curr.Length;
        var posOnLane = v.Kinematics.Pos;
        var usableDist = currentDist - posOnLane;

        // MSLCM_LC2013.h:189 currentDistDisallows.
        if (usableDist / Math.Abs(bestLaneOffset) >= laDist)
        {
            return false;
        }

        var neighborHandle = right ? lane.RightNeighbor : lane.LeftNeighbor;
        if (neighborHandle < 0)
        {
            // No lane to change into on the required side -- defensive only, not reachable by
            // any committed scenario (ComputeBestLanes never points a route offset off the
            // edge's own lane range).
            return false;
        }

        var neighborLane = _network.LanesByHandle[neighborHandle];

        // Safety veto, mirroring A2-iii's IsTargetLaneSafe / B5-ii's obstacle veto -- on the
        // clear road this scenario exercises, both are trivially non-binding (no neighbor
        // vehicle, no obstacle), matching the briefing's "on a clear road the change is always
        // safe". A scenario WITH target-lane traffic during a strategic change is future work
        // (LCA_URGENT's real blocker-cooperation machinery, `.cpp:1467-1517`, is not ported).
        var neighLead = neighbors.GetNeighborLeader(v, neighborLane.Handle);
        var neighFollow = neighbors.GetNeighborFollower(v, neighborLane.Handle);
        if (!IsTargetLaneSafe(v, neighLead, neighFollow, dt) || TargetLaneBlockedByObstacle(v, neighborLane, time, dt))
        {
            return false;
        }

        _commandBuffer.ChangeLane(v, neighborLane.Handle, neighborLane.Id);
        // MSLCM_LC2013.cpp:1063/1080 resetState() on any committed change (strategic included).
        v.SpeedGainProbability = 0.0;
        return true;
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

    // B5-ii (TASKS.md "Cross-lane blocker vetoing lane changes"): generalizes IsTargetLaneSafe's
    // real-neighLead/neighFollow secure-gap veto to an EXTERNAL (non-SUMO) obstacle sitting on
    // the LEFT (target) lane -- same brake-gap secure-gap machinery (SecureGap below), just with
    // an ExternalObstacle standing in for the real VehicleRuntime neighbor. Reads `_obstacles`
    // exactly as ObstacleConstraint/AdvanceObstacles's own header comments describe: a frozen,
    // already-advanced-for-this-step position (AdvanceObstacles ran in this same step's Input
    // phase, before PlanMovements/ExecuteMoves/this post-move phase), never mutated here
    // (CLAUDE.md rule 2 -- this method only READS `_obstacles`).
    //
    // Ego's projected slot on `targetLane` is [egoBack, egoFront] = [ego.Pos - ego.VType.Length,
    // ego.Pos] -- the commit gate performs an instant lane-index snap (lanechange.duration=0,
    // same convention every other structural change in this engine uses): only LaneId changes,
    // ego's own arc-length Pos is unchanged, so this is exactly the segment ego would occupy on
    // `targetLane` the instant the change commits.
    //
    // Inert-when-absent (CLAUDE.md rule 3 / the A2-byte-identical constraint): `_obstacles.Count
    // == 0` returns false immediately -- the SAME empty-store fast path ObstacleConstraint's own
    // header comment documents -- so for scenario 12 and every other obstacle-free scenario/test
    // this helper is a no-op and the commit gate's `&&` above is byte-identical to a bare
    // `IsTargetLaneSafe(...)` call.
    private bool TargetLaneBlockedByObstacle(VehicleRuntime ego, Lane targetLane, double time, double dt)
    {
        if (_obstacles.Count == 0)
        {
            return false;
        }

        var egoFront = ego.Kinematics.Pos;
        var egoBack = egoFront - ego.VType.Length;

        foreach (var obstacle in _obstacles.Values)
        {
            // Same active-window/lane filter ObstacleConstraint applies, just against
            // `targetLane` instead of `v.LaneId` -- an obstacle on ego's OWN (current, non-
            // target) lane, or outside [StartTime, EndTime), never reaches the checks below
            // (the "obstacle on the OTHER lane doesn't affect the change" scoping).
            if (obstacle.StartTime > time || time >= obstacle.EndTime || obstacle.LaneId != targetLane.Id)
            {
                continue;
            }

            var obstacleBack = obstacle.FrontPos - obstacle.Length;
            var obstacleFront = obstacle.FrontPos;

            if (obstacleBack >= egoFront)
            {
                // Ahead of ego's projected slot -- virtual neighLead: the SAME
                // MSLane::getLeader-shaped gap (leaderBackPos - egoMinGap - egoPos)
                // IsTargetLaneSafe's neighLead branch requires, secured by the SAME brake-gap
                // formula (SecureGap), with the obstacle's own reported Speed/MaxDecel standing
                // in for a real leader's Kinematics.Speed/VType.Decel -- mirrors
                // LeaderFollowSpeedConstraint/ObstacleConstraint's existing predSpeed/
                // predMaxDecel plumbing exactly.
                var gap = obstacleBack - ego.VType.MinGap - egoFront;
                var secureGap = SecureGap(ego.Kinematics.Speed, ego.VType, obstacle.Speed, obstacle.MaxDecel, dt);
                if (gap < secureGap)
                {
                    return true;
                }
            }
            else if (obstacleFront <= egoBack)
            {
                // Behind ego's projected slot -- virtual neighFollow: the SAME gap
                // ((egoBack) - followerMinGap - followerPos) and brake-gap secure-gap
                // IsTargetLaneSafe's neighFollow branch requires, with the obstacle playing the
                // FOLLOWER role this time. An ExternalObstacle carries only Speed/MaxDecel (B5-i's
                // own field comment) -- no MinGap/Tau field exists to fall back to, so two
                // documented proxies stand in, both conservative (widening, never narrowing, the
                // required gap versus what a real trailing vehicle would need):
                //   - follower decel: obstacle.MaxDecel when moving (Speed != 0), else ego's own
                //     VType.Decel -- EXACTLY ObstacleConstraint's own
                //     `nearest.Speed != 0.0 ? nearest.MaxDecel : v.VType.Decel` conditional (a
                //     Speed==0/B1-style static obstacle's decel capability is otherwise
                //     unobservable, so its default MaxDecel=0 must not be trusted here either).
                //   - follower minGap/headway (Tau): reuse ego's own VType.MinGap/VType.Tau (the
                //     only resolved vType in scope) -- there is no B5-i precedent for a follower-
                //     role obstacle at all (obstacles have only ever played the LEADER role, see
                //     LeaderFollowSpeedConstraint/ObstacleConstraint), so this is a fresh, explicit
                //     choice rather than a mirrored one.
                var obstacleDecel = obstacle.Speed != 0.0 ? obstacle.MaxDecel : ego.VType.Decel;
                var gap = egoBack - ego.VType.MinGap - obstacleFront;
                var secureGap = SecureGap(obstacle.Speed, obstacleDecel, ego.VType.Tau, ego.Kinematics.Speed, ego.VType.Decel, dt);
                if (gap < secureGap)
                {
                    return true;
                }
            }
            else
            {
                // Overlaps ego's projected slot outright -- no brake-gap check needed or
                // meaningful (a negative gap by construction): there is no room to change into
                // at all. Hard veto.
                return true;
            }
        }

        return false;
    }

    // MSCFModel::getSecureGap (MSCFModel.cpp:166-172): the brake-gap-difference secure-gap
    // formula (gComputeLC-relax branch at :173-179 omitted -- see IsTargetLaneSafe's comment).
    private static double SecureGap(double followerSpeed, ResolvedVType followerVType, double leaderSpeed, double leaderMaxDecel, double dt) =>
        SecureGap(followerSpeed, followerVType.Decel, followerVType.Tau, leaderSpeed, leaderMaxDecel, dt);

    // B5-ii: raw-decel/tau overload the real-vType SecureGap above now forwards to -- needed
    // because TargetLaneBlockedByObstacle's neighFollow-role branch has no ResolvedVType to hand
    // it (an ExternalObstacle is not a vType-bearing entity; see that branch's own comment for
    // the follower-decel/minGap/tau proxies it passes here). Formula itself is untouched/
    // unmoved: same MSCFModel::getSecureGap math, byte-identical for every existing
    // ResolvedVType-based call site above.
    private static double SecureGap(double followerSpeed, double followerDecel, double followerTau, double leaderSpeed, double leaderMaxDecel, double dt)
    {
        var maxDecel = Math.Max(followerDecel, leaderMaxDecel);
        var leaderBrakeGap = KraussModel.BrakeGap(leaderSpeed, maxDecel, 0.0, dt);
        return Math.Max(0.0, KraussModel.BrakeGap(followerSpeed, followerDecel, followerTau, dt) - leaderBrakeGap);
    }

    // The engine emits FULL double-precision trajectory values. The goldens are regenerated
    // with SUMO's `--precision` raised well above the default 2 (see scripts/regen-goldens.sh
    // and each scenario's provenance) so the committed FCD carries enough digits for the
    // per-scenario tolerance (1e-3) to be a *real* bar. Do NOT round emitted values to match a
    // low-precision golden: that would silently cap parity sensitivity at ~0.5*10^-precision
    // regardless of tolerance.json, masking genuine sub-0.01 trajectory drift. Lane-relative
    // Pos/Speed are the source of truth; x/y/angle are derived from the lane polyline.
    //
    // D6: this is the [SystemPhase.Export] system (see Run()'s phase-tagged call site for the
    // load-bearing "top of loop" timing note). D9 (FastDataPlane ECS readiness -- info/
    // replication export SEAM, TASKS.md line ~651): the per-vehicle `TrajectoryPoint` this
    // method emits now flows FROM a single `VehicleExportSnapshot` built once per vehicle --
    // the `TrajectorySet` is the engine's own default, always-present consumer of that snapshot
    // (see VehicleExportSnapshot.cs/ISimExportObserver.cs's own header comments), and any
    // OTHER registered `ISimExportObserver` is notified with the SAME snapshot value right
    // after. This does not change what is computed or emitted: same one
    // `LaneGeometry.PositionAtOffset` call per vehicle, same fields, same order, same
    // `trajectory.Add(...)`, same null `Acceleration` -- the snapshot is purely a stack-local
    // struct wrapping values EmitTrajectory already computed before this rung. With
    // `_exportObservers` empty (the default), the notify loop below is an empty `foreach`: no
    // virtual call, no allocation -- byte-identical output and allocation to the pre-D9 body.
    private void EmitTrajectory(TrajectorySet trajectory, double time)
    {
        // D9: frame-bracket hooks -- empty loop bodies when `_exportObservers` is empty (the
        // default), so this costs nothing beyond the `Count` check for every existing scenario/
        // test/benchmark.
        for (var i = 0; i < _exportObservers.Count; i++)
        {
            _exportObservers[i].OnFrameBegin(time);
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            var lane = _network!.LanesById[v.LaneId];
            var (x, y, angle) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos);

            var snapshot = new VehicleExportSnapshot(
                entity: v.Entity,
                entityIndex: v.EntityIndex,
                vehicleId: v.Def.Id,
                time: time,
                lane: v.LaneId,
                pos: v.Kinematics.Pos,
                speed: v.Kinematics.Speed,
                x: x,
                y: y,
                angle: angle);

            trajectory.Add(new TrajectoryPoint(
                VehicleId: snapshot.VehicleId,
                Time: snapshot.Time,
                Lane: snapshot.Lane,
                Pos: snapshot.Pos,
                Speed: snapshot.Speed,
                X: snapshot.X,
                Y: snapshot.Y,
                Angle: snapshot.Angle,
                Acceleration: null));

            // D9: notify every registered observer with the SAME snapshot -- empty list by
            // default, so this is a zero-iteration `foreach` (no allocation, no virtual call)
            // for every existing scenario/test/benchmark that never calls AddExportObserver.
            for (var i = 0; i < _exportObservers.Count; i++)
            {
                _exportObservers[i].OnVehicleExported(in snapshot);
            }
        }

        for (var i = 0; i < _exportObservers.Count; i++)
        {
            _exportObservers[i].OnFrameEnd(time);
        }
    }
}
