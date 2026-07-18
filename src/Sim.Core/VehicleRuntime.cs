using Sim.Ingest;

namespace Sim.Core;

// Per-vehicle mutable runtime state, plus the immutable spawn template (Def) it was created
// from. D3 (FastDataPlane ECS readiness): every field left on this class is now unmanaged
// (scalars/structs -- `Kinematics`/`MoveIntent` are already unmanaged structs) or one of the
// two IMMUTABLE blueprint refs (`Def`, `VType`); the managed, variable-length state that used
// to live here (`LaneSequence`/`LaneSequenceHandles`, `Stops`, `AvoidedEdges`) has moved to
// engine-owned side storage keyed by `EntityIndex` -- a shared int pool with a per-entity
// [start,len) slice for the lane sequence, and `Dictionary<int, ...>` side tables for the rare/
// cold stop queue and avoided-edge set. This is the FDP-readiness posture: the class is now
// chunk-storable (no `Queue`/`HashSet`/`IReadOnlyList`/`int[]` fields) modulo `Def`/`VType`
// still being managed refs and the flat scalar layout not yet grouped into sub-structs --
// turning `Def`/`VType` into TKB handles and grouping the scalars is deferred to D7's store
// boundary (out of this rung's scope; see TASKS.md D3/D7).
internal sealed class VehicleRuntime
{
    public required VehicleDef Def { get; init; }

    // Resolved (fully-defaulted) vType parameters (Sim.Ingest.VTypeDefaults) -- the car-
    // following model reads these, never the raw .rou.xml VType with its optional fields.
    //
    // `set` (not `init`) SOLELY for the panic-evac per-vehicle param override
    // (Engine.SetVehicleParams -- PANIC-EVAC.md R2: "flee mode is just another override/call"):
    // the external evac layer bulk-swaps a running vehicle's knobs to aggressive values by
    // assigning a `VType with { ... }` copy. Nothing on the golden/parity path ever assigns this
    // after creation, so the determinism hash (909605E965BFFE59) is byte-identical unless a caller
    // opts in -- exactly the inert-when-unused posture every other laneless/evac seam carries.
    public required ResolvedVType VType { get; set; }

    // D3: this vehicle's stable index in Engine._vehicles, set once at creation (LoadScenario).
    // Vehicles are never removed from that list -- only flagged Arrived -- so the list index is
    // a stable entity id. D5 adds `Entity` (below) as the actual FDP-shaped handle; EntityIndex
    // remains the plain int key the engine's side storage uses directly (lane-sequence pool
    // slice owner id is implicit via LaneSeqStart/LaneSeqLen below; Stops/AvoidedEdges side
    // tables are keyed by this directly) -- always equal to Entity.Index.
    public int EntityIndex;

    // D5 (FastDataPlane ECS readiness): the FDP-shaped handle for this vehicle -- `new
    // Entity(EntityIndex, 0)`, set once at creation alongside EntityIndex (LoadScenario).
    // Generation stays 0 (no recycling yet, see Entity.cs's header comment); nothing in the
    // engine keys off this yet (EntityIndex/side tables are unchanged), it exists so callers
    // can start holding the FDP-shaped handle instead of a raw int.
    public Entity Entity;

    public bool Inserted;

    // Set once the vehicle runs off the end of LaneSequence (route end) during execute;
    // distinct from Inserted so InsertDepartingVehicles never mistakes "arrived" for "not yet
    // departed" and re-inserts.
    public bool Arrived;
    public string LaneId = string.Empty;

    // D2: the dense handle of LaneId (`_network.LaneHandleById[LaneId]`) -- kept in lockstep
    // with LaneId by every Engine write site (insertion, lane traversal, LC swaps, reroute).
    // LaneId remains authoritative for correctness/emit; LaneHandle exists purely so hot-path
    // lookups (LaneNeighborQuery buckets, `_network.LanesByHandle[...]`) can index an array
    // instead of hashing a string every vehicle, every step.
    public int LaneHandle;

    // C10-i: continuous lane-change maneuver (lanechange.duration > 0). While a change is in
    // progress the vehicle slides laterally over several steps instead of snapping; `LaneHandle`
    // (the emitted lane) stays the SOURCE lane until the vehicle crosses the lane midpoint, then
    // becomes the target. LcTargetHandle == -1 means "no maneuver in progress" (the instant-snap
    // default for every duration==0 scenario). LcStepsElapsed counts steps since the change was
    // committed; LcStepsTotal is round(duration/stepLength). See Engine.AdvanceLaneChanges.
    public int LcTargetHandle = -1;
    public string LcTargetId = string.Empty;
    public int LcStepsElapsed;
    public int LcStepsTotal;

    // D3: this vehicle's lane-sequence is now a SLICE `[LaneSeqStart, LaneSeqStart+LaneSeqLen)`
    // into Engine's shared `_laneSeqPool` (a single `List<int>` of lane HANDLES, blob-style) --
    // replacing the old per-vehicle `IReadOnlyList<string> LaneSequence`/`int[]
    // LaneSequenceHandles` managed collections. Set once at insertion (TryInsertOnLane) by
    // appending the resolved handle sequence to the pool; a reroute (UpdateReroutes) appends a
    // NEW slice and simply repoints Start/Len (the old slice is abandoned in the pool -- it only
    // grows; D7 can compact if that ever matters). LaneSeqIndex is the index of the CURRENT lane
    // (LaneHandle) within this slice, advanced by ExecuteMoves as the vehicle's Pos crosses each
    // lane's end. A single-edge route resolves to a one-element slice, so this collapses to rung
    // 1-8's single-lane "reached the end -> arrived" behavior exactly.
    public int LaneSeqStart;
    public int LaneSeqLen;
    public int LaneSeqIndex;
    public Kinematics Kinematics;
    public MoveIntent Intent;

    // C1-i: this vehicle's private dawdle RNG state (Sim.Core.VehicleRng -- a single unmanaged
    // `ulong`, D3-clean). Seeded ONCE, at creation (Engine.LoadScenario), from
    // `VehicleRng.SeedFor(engine.Seed, EntityIndex)` -- never reseeded mid-run. Advanced by
    // exactly one draw per active vehicle per step, ONLY when VType.Sigma>0, inside
    // KraussModel.FinalizeSpeed's dawdle2 port (threaded there `ref` from
    // Engine.ComputeMoveIntent so the draw persists) -- when Sigma==0 this field is written at
    // creation and never read/advanced again, which is exactly why sigma==0 stays
    // byte-identical to every pre-C1 rung (no draw occurs, so this field's value never
    // influences the result). Each entity draws from its own copy here, never a shared/global
    // RNG, which is what keeps Engine.UseParallelPlan race-free with sigma>0 (see that
    // property's own header comment).
    public VehicleRng RngState;

    // C7-i (TASKS.md "speedFactor distribution (heterogeneous desired speeds)"): this vehicle's
    // own chosen speedFactor (MSVehicleType::computeChosenSpeedDeviation -- see
    // NormcDistribution.cs), drawn ONCE at creation (Engine.LoadScenario) from a SEPARATE,
    // SALTED VehicleRng (VehicleRng.SeedFor(Seed, EntityIndex, salt) -- never RngState above, and
    // never persisted/re-seeded after this one draw, matching SUMO's own once-at-vehicle-build
    // call site, MSVehicleControl.cpp:113). Threaded into KraussModel.LaneVehicleMaxSpeed at
    // every one of its four Engine.cs call sites in place of the old `vType.SpeedFactor`-only
    // read. When ScenarioConfig.SpeedDev<=0 (every pre-C7 scenario's `default.speeddev="0"`),
    // NormcDistribution.SampleNormc's `dev<=0` branch returns the vType's mean speedFactor
    // (1.0 for every existing scenario) WITHOUT any draw at all -- this field is then simply
    // `vType.SpeedFactor` exactly, which is exactly why every sigma=0/speeddev=0 scenario stays
    // byte-identical to every pre-C7 rung.
    public double SpeedFactor;

    // D3: this vehicle's scheduled stops (Sim.Ingest.VehicleDef.Stops) moved to Engine's
    // `_stopsByEntity` side table (keyed by EntityIndex), populated once at LoadScenario only
    // for vehicles that actually have stops -- the managed `Queue<StopRuntime>` no longer lives
    // on every vehicle record. Front-of-queue-only access pattern is unchanged.

    // Rung 8b: SUMO's MSLCM_LC2013::myKeepRightProbability -- a stateful per-vehicle accumulator
    // for the keep-right (Rechtsfahrgebot) lane-change incentive. Starts at 0 (SUMO's ctor
    // default); only ever mutated by Engine.ExecuteMoves from the plan phase's MoveIntent
    // (CLAUDE.md rule 3 -- Plan writes only MoveIntent, never this field directly).
    public double KeepRightProbability;

    // C4-vii-b: memo for ApplyKeepRightDecision's strategic stayOnBest suppressor
    // (KeepRightStrategicStay) -- "must this vehicle NOT accumulate keep-right because its right
    // neighbour is a must-avoid turn/exit lane within TURN_LANE_DIST". That answer is a pure
    // function of the current lane + remaining route, so it only changes when the vehicle changes
    // lane (or reroutes); the underlying ComputeBestLanes is an allocating route-wide pass, so
    // memoizing it here (keyed by the LaneHandle it was computed for; -1 = not yet computed) keeps
    // that pass off the per-step hot path -- it fires at most once per lane the vehicle occupies
    // instead of every step. Invalidated on reroute (CommandBuffer's ReplaceRoute resets it to -1).
    // Inert for lane-0/single-lane vehicles: ApplyKeepRightDecision returns on `RightNeighbor < 0`
    // before ever reading this.
    public int KeepRightStayCacheLane = -1;
    public bool KeepRightStaySuppress;

    // Rung A2: SUMO's MSLCM_LC2013::mySpeedGainProbability -- a stateful per-vehicle accumulator
    // for the speed-gain (overtaking) lane-change incentive. Starts at 0 (SUMO's ctor default);
    // unlike KeepRightProbability (plan-phase, pre-move), this is decided/written by the new
    // post-move phase (Engine.DecideSpeedGainChanges) that runs AFTER ExecuteMoves -- SUMO's
    // changeLanes phase reads post-move gaps (MSNet.cpp:784/790/796), so this field is written
    // directly there rather than threaded through MoveIntent (CLAUDE.md rule 3 is still honored:
    // it is written once, after all vehicles' moves are settled, from a single frozen post-move
    // snapshot built at the top of that phase -- not a mid-query shared-state write).
    public double SpeedGainProbability;

    // P2G-2 (docs/HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md): SUMO's MSLCM_LC2013 myVSafes speed-advice
    // channel. A blocked lane-changer's informFollower writes (as a MIN) the speed THIS vehicle should
    // slow to so the changer can cut in ("make room"); the car-following phase reads it as an additive
    // vPos cap NEXT step and clears it. +Infinity == no advice. Written/consumed ONLY when
    // Engine.CoordinatedLaneChange is on, so it stays +Infinity (inert, byte-identical) by default.
    public double CoopSpeedAdvice = double.PositiveInfinity;

    // C2-ii: SUMO's MSLCM_LC2013::myLookAheadSpeed -- a stateful per-vehicle "how fast have I
    // recently been driving" estimate feeding the STRATEGIC lane-change look-ahead distance
    // (laDist, MSLCM_LC2013.cpp:1227-1239) and the keep-right STAY guard's own laDist term.
    // Starts at 0.0 (SUMO's ctor default, LOOK_AHEAD_MIN_SPEED); only ever touched inside
    // Engine.TryStrategicLaneChange, which itself is gated on the vehicle's ACTUAL lane
    // differing from its route pool's target lane on the same edge -- for every single-lane-
    // per-edge scenario (and any scenario where the depart lane already is the continuing
    // lane) that gate is always false, so this field is written once at creation (0.0) and
    // never read/advanced again, exactly like RngState's own sigma==0 byte-identical argument.
    public double LookAheadSpeed;

    // C11-ii: SUMO's MSCFModel_ACC::ACCVehicleVariables (MSCFModel_ACC.h:140-146) -- the
    // per-vehicle ACC control-mode hysteresis state (0=speed control,1=gap control) plus the
    // "written at most once per timestep" guard timestamp, both initialized to 0 (matching
    // ACCVehicleVariables' own ctor / createVehicleVariables default). Only ever read/written
    // from inside AccModel.FollowSpeed, threaded there `ref` from Engine.cs's FollowSpeedFor
    // dispatch -- and ONLY when this vehicle's OWN VType.CarFollowModel=="ACC" (see AccModel.cs's
    // own header comment). Written ONLY by the vehicle that owns it (never a leader's/foe's),
    // exactly like RngState's own per-entity dawdle draw -- see Engine.UseParallelPlan's header
    // comment for why that per-entity-write pattern is already established as parallel-safe.
    // Byte-identical for every non-ACC vType: these two fields are simply never touched.
    public int AccControlMode;
    public double AccLastUpdateTime;

    // C11-iii: SUMO's MSCFModel_CACC::CACCVehicleVariables (MSCFModel_CACC.h:222-228) --
    // `class CACCVehicleVariables : public MSCFModel_ACC::ACCVehicleVariables` literally
    // INHERITS ACC_ControlMode/lastUpdateTime rather than declaring its own copies, adding only
    // ONE new field: CACC_ControlMode (the CACC-specific speed/gap-control hysteresis mode).
    // Ported as CaccControlMode below (default 0, matching createVehicleVariables' own
    // CACC_ControlMode=0 default). The inherited pair is DELIBERATELY reused rather than
    // duplicated: a CACC-typed vehicle's embedded ACC-fallback state (used only when its leader is
    // NOT itself CACC) reuses THIS SAME vehicle's AccControlMode/AccLastUpdateTime fields above --
    // see CaccModel.cs's own header comment for the full citation, including why this reuse is
    // required for byte-parity (the shared lastUpdateTime guard's cross-call interaction), not
    // merely a storage optimization. No collision with an actually-ACC-typed vehicle: a vehicle's
    // CarFollowModel is fixed at exactly one string, so Engine.cs's dispatch never routes a given
    // vehicle through both AccModel.FollowSpeed and CaccModel.FollowSpeed. Only ever read/written
    // from inside CaccModel.FollowSpeed, threaded `ref` from Engine.cs's FollowSpeedFor dispatch,
    // and ONLY when this vehicle's OWN VType.CarFollowModel=="CACC" -- byte-identical for every
    // other vType (Krauss/IDM/ACC): this field is simply never touched.
    public int CaccControlMode;

    // C11-iv: SUMO's MSCFModel_IDM::VehicleVariables::levelOfService (MSCFModel_IDM.h:189-194)
    // -- the IDMM per-vehicle headway-adaptation memory, ctor-defaulted to 1.0 (NOT 0 -- see that
    // ctor's own initializer list). Set to 1.0 for EVERY vehicle at creation (Engine.LoadScenario),
    // not just IDMM ones: only IdmModel/Engine's IDMM dispatch arms ever read or write it (see
    // IdmModel.V's headwayTimeOverride parameter and the IDMM finalizeSpeed arm in Engine.cs's
    // ComputeMoveIntent), so this is byte-identical-inert for every non-IDMM vType, exactly like
    // AccControlMode/CaccControlMode above are for their own non-owning vTypes. At LOS=1.0 (the
    // vendored ctor default, and the value plain IDM/ACC/CACC leave it at forever since they never
    // touch it) the IDMM headway formula collapses to `tau` exactly -- see IdmModel's own
    // Idmm-adaptation comments for that derivation.
    public double LevelOfService;

    // C11-iii: the ego's OWN acceleration from the LAST COMPLETED step
    // ((newSpeed-oldSpeed)/dt), the exact analog of MSVehicle::getAcceleration() at the instant
    // CACC's cooperative gap-control law (MSCFModel_CACC.cpp:287) reads it. Written ONLY in
    // Engine.ExecuteMoves (the EXECUTE phase, right next to the pre-existing `oldSpeed` capture)
    // and read ONLY in the FOLLOWING step's PLAN phase by CaccModel's cooperative branch --
    // consistent with the frozen-start-of-step-snapshot invariant (CLAUDE.md rule 2): never a
    // leader's/foe's acceleration, only this vehicle's own. Default 0.0, matching
    // getAcceleration()'s own value before any step has executed. Written for EVERY vehicle
    // (unconditionally, alongside oldSpeed) but read by nothing except CaccModel -- so this field
    // is byte-identical-inert for every non-CACC vType, exactly like AccControlMode/
    // AccLastUpdateTime above are for every non-ACC/non-CACC vType.
    public double Acceleration;

    // C4-ii: accumulated waiting time (MSVehicle::myWaitingTime), in seconds -- the running count
    // of consecutive time the vehicle has been effectively halted. Ported from
    // MSVehicle::updateWaitingTime (MSVehicle.cpp:4081-4088): each Execute step, `+= dt` while the
    // new speed is <= SUMO_const_haltingSpeed (0.1) AND this step's acceleration is <=
    // accelThresholdForWaiting (0.5*maxAccel); otherwise reset to 0 (the vehicle is moving/
    // accelerating away). Written ONLY in Engine.ExecuteMoves (the EXECUTE phase) and read ONLY in
    // the FOLLOWING step's PLAN phase by JunctionYieldConstraint's all-way-stop arm (the
    // arrival-order tie-break: whoever has waited longer goes first) -- consistent with the
    // frozen-start-of-step-snapshot invariant (CLAUDE.md rule 2), never a foe's mid-step value.
    // Default 0.0. Read by nothing except the all-way-stop arm, so byte-identical-inert for every
    // junction that is not `type="allway_stop"` (and thus for every pre-C4-ii scenario).
    public double WaitingTime;

    // C8-ii: the simulation time of this vehicle's last ACTION step (MSVehicle::myLastActionTime).
    // With actionStepLength > dt a vehicle re-plans its speed only every actionStepLength seconds
    // (its "reaction time"); between action steps it continues with the acceleration decided at the
    // last one. isActionStep (MSVehicle.h:638) is `(t - myLastActionTime) % actionStepLength == 0`;
    // this field is updated to the current time on each action step and read at the top of the next
    // plan to decide whether to re-plan or hold. Initialized to NegativeInfinity so the FIRST plan
    // (on insertion) is always an action step, matching MSVehicle's `myActionStep(true)` initial
    // state. Written only in the PLAN phase (Engine.ComputeMoveIntent), this vehicle's own field
    // only -- parallel-safe exactly like RngState/LevelOfService. Entirely inert when
    // actionStepLength == dt (every pre-C8-ii scenario): the gate that reads it is skipped, so no
    // field access happens at all and behavior is byte-identical.
    public double LastActionTime;

    // C4-viii: SUMO's MSLink::ApproachingVehicleInformation::willPass -- "does this vehicle intend to
    // ENTER its upcoming junction link THIS step". SUMO computes it as
    // `setRequest = (vNext > NUMERICAL_EPS_SPEED && !abortRequestAfterMinor) || leavingCurrentIntersection`
    // (MSVehicle.cpp:2732) and registers it via MSLink::setApproaching BEFORE any MSLink::opened()
    // crossing-yield decision reads it (MSLink.cpp:935 short-circuits `if (!avi.willPass) return
    // false`). The load-bearing fact is the PLANNED vNext, not the start-of-step speed: a foe that is
    // moving at start-of-step but BRAKING TO A STOP this step (because it is itself yielding) has
    // vNext ~ 0 and willPass=false, so it must NOT block ego -- which is what unwinds the dense-grid
    // saturation gridlock. The engine has one PlanMovements pass, so this is cached once per step by a
    // PRE-PASS (Engine.ComputeWillPass) from the frozen start-of-step snapshot, BEFORE PlanMovements,
    // using each vehicle's planned vNext computed WITHOUT the foe-willPass refinement (one level of
    // approximation, mirroring setApproaching-before-opened()), then read in JunctionYieldConstraint's
    // approaching-foe arm. One bool per vehicle, zero-alloc (the KeepRightProbability/LastActionTime
    // plan-phase-cache pattern). Default false. Inert wherever no foe is braking-to-stop at a crossing
    // (every committed scenario) -- there, no vehicle's WillPass is ever read.
    public bool WillPass;

    // C4-viii-b (bug C, the hold arm): set by Engine.ResolveRightBeforeLeftCycles when it breaks a
    // symmetric right-before-left response cycle. The resolver selects a maximal non-conflicting
    // subset of the cycle's links to PASS and marks the rest to YIELD; a yielding vehicle's WillPass
    // is set false, but WillPass=false alone does NOT hold a vehicle -- the crossing gate only makes
    // ego yield to a foe whose WillPass is TRUE, so in a rock-paper-scissors cycle where only ONE
    // vehicle is granted the pass, the OTHER yielders (whose sole higher-priority foe is itself a
    // yielder, WillPass=false) would see no passing foe and wrongly enter, re-locking the junction
    // mid-box. This flag is the resolver's DIRECT abort of those vehicles' entry -- the deterministic
    // analogue of SUMO's MSVehicle::planMoveInternal RNG abort clearing mySetRequest (MSVehicle.cpp:
    // 2818-2839), which holds the aborted vehicle at the stop line regardless of any foe's state.
    // Read ONLY in the real (prePass=false) JunctionYieldConstraint, and ONLY while ego is still on
    // its approach lane (the hold gates ENTRY). Reset to false for every active vehicle at the top of
    // each ResolveRightBeforeLeftCycles pass, then set true for the yielders -- so it is a fresh
    // per-step decision, never stale. Default false; inert for every scenario without an actual
    // right-before-left cycle (no committed golden has one -- the unchanged Sim.Bench hash + green
    // suite are the proof, exactly as for the WillPass write the resolver already performs).
    public bool JunctionCycleHold;

    // Perf (willPass/plan fusion): set by JunctionYieldConstraint DURING the willPass pre-pass
    // (Engine.ComputeWillPass) iff this vehicle takes the finite approaching-foe CROSSING yield --
    // the ONE and only place a real (prePass=false) plan can differ from the pre-pass plan (the
    // `!foe.WillPass` short-circuit at line ~3499 relaxes exactly that finite yield). Every other
    // prePass/real divergence is a side-effect (RngState/LevelOfService/GiveWaySide/LatOffset/
    // LastActionTime) that Engine._fusionEligible excludes at load time. When the scenario is
    // fusion-eligible and this flag is false, PlanMovements REUSES the pre-pass MoveIntent instead
    // of recomputing it -- byte-identical, and it halves the per-junction-vehicle plan cost. Reset
    // to false before each pre-pass ComputeMoveIntent; only ever written by that vehicle's own
    // pre-pass (parallel-safe, per-ego field).
    public bool CrossingYieldTaken;

    // Perf (willPass/plan fusion): the pre-pass tells PlanMovements to REUSE this vehicle's already-
    // computed Intent (skip the second ComputeMoveIntent). True iff the scenario is _fusionEligible,
    // the vehicle was WillPassRelevant (so the pre-pass actually computed its Intent), and it did NOT
    // take the crossing yield (CrossingYieldTaken == false). Own-field, set once per step.
    public bool ReuseIntent;

    // B3: live reroute-around-blockage bookkeeping (DESIGN.md "Two futures" -- not a SUMO
    // field). BlockedByObstacleSeconds accumulates dt while a FUTURE edge of this vehicle's
    // remaining route is sitting under an active external obstacle; reset to 0 the moment no
    // future edge is blocked. Both start at their zero values (0 / empty), which is exactly the
    // inert-when-absent case: with RerouteThresholdSeconds left at its default (+infinity),
    // Engine.UpdateReroutes returns immediately every step and neither this field nor the
    // AvoidedEdges side table below is ever touched.
    public double BlockedByObstacleSeconds;

    // D3: this vehicle's already-routed-around-once edge set moved to Engine's
    // `_avoidedByEntity` side table (keyed by EntityIndex), lazily created only when a vehicle
    // first reroutes -- the managed `HashSet<string>` no longer lives on every vehicle record.
    // Off the hot path (reroute is opt-in via RerouteThresholdSeconds).

    // Rung ER3 (give-way): this vehicle's current "clear the way for an emergency vehicle" intent,
    // recomputed each PLAN step (Engine.DetectGiveWaySide) from the frozen start-of-step snapshot.
    // 0 = none, -1 = clear toward the right lane edge, +1 = clear toward the left lane edge. Read
    // by the ER4 (multi-lane, Engine.DecideGiveWayChanges) and ER5 (single-lane lateral drift,
    // Engine.ComputeLateralEvasion) execution arms, and exported via VehicleExportSnapshot. Written
    // ONLY in the PLAN phase by the owning vehicle (parallel-safe exactly like LevelOfService /
    // WillPass). Default 0, and left 0 for every scenario with no active bluelight EV in range
    // (Engine._anyBluelight short-circuits detection), so byte-identical-inert wherever give-way
    // does not trigger.
    public int GiveWaySide;

    // Rung ER4 (give-way execution, multi-lane): true iff the approaching blue-light EV that
    // triggered this vehicle's give-way intent is in this vehicle's OWN lane (so this vehicle
    // should VACATE the lane by changing to an adjacent one, rather than merely drifting to the
    // edge). Computed alongside GiveWaySide in the PLAN phase from the frozen start-of-step
    // snapshot (Engine.DetectGiveWay), read by the ER4 lane-change arm (Engine.TryGiveWayLaneChange).
    // Default false; left false whenever no EV shares this vehicle's lane, so inert wherever
    // give-way does not trigger a lane change.
    public bool GiveWayEvSameLane;

    // Rung OV1 (opposite-direction overtaking): true iff this vehicle (a) is held up behind a
    // slower same-lane leader and (b) sees the oncoming (opposite-direction) lane clear far enough
    // ahead to consider overtaking through it. Recomputed each PLAN step (Engine.DetectOvertake)
    // from the frozen start-of-step snapshot, written only by the owning vehicle (parallel-safe like
    // GiveWaySide), and exported via VehicleExportSnapshot. Default false; left false for every vType
    // without lcOpposite (Engine._anyLcOpposite short-circuits detection), so inert wherever
    // opposite-direction overtaking is absent. Consumed by the OV2/OV3 decision/execution arms.
    public bool OvertakeActive;

    // Rung D2 (OV3 return-gap enforcement): while overtaking, the EntityIndex of the same-lane leader
    // this vehicle is passing (-1 = none). Remembered when the overtake commits (DetectOvertake held
    // up), and read AFTER this vehicle has nosed ahead of that leader -- once GetLeader no longer
    // returns it -- so the overtaker stays spilled until it is a safe following gap AHEAD of the
    // just-passed leader before recentering, instead of cutting back in the instant it edges past.
    // Transient plan-phase state like OvertakeActive (not captured in the file snapshot); default -1.
    public int OvertakePassedLeaderIndex = -1;

    // Rung OV4 (cooperative oncoming shift): true iff this vehicle is an oncoming driver that sees a
    // spilled opposite-direction overtaker (a bidi-lane vehicle encroaching across the centre line)
    // closing head-on within range, and is therefore pulling to its OWN outer lane edge to widen the
    // corridor for the overtake -- the mirror of the ER3/ER5 give-way drift. Recomputed each PLAN
    // step (Engine.DetectCooperativeShift) from the frozen start-of-step snapshot (it reads the
    // overtaker's already-committed LatOffset, never a same-step plan flag, so it is parallel-safe
    // like GiveWaySide/OvertakeActive), and exported via VehicleExportSnapshot. Default false; left
    // false for every vType wherever no vType has lcOpposite (Engine._anyLcOpposite short-circuits
    // detection), so inert wherever opposite-direction overtaking is absent. Consumed by
    // ComputeLateralEvasion, which drifts ego to its outer edge while it is set.
    public bool CooperativeShift;

    // DR2 (dead-reckoning coordination, issue #3): true iff this vehicle's laneless-RVO / cross-regime
    // lateral solve ACTIVELY COUPLED to a neighbour or crowd agent THIS step (ComputeRvoLateral's
    // forbCount > 0) -- i.e. it is mid-swerve, so its short-horizon lateral is a reactive manoeuvre, NOT
    // linearly lane-predictable. The DR publisher reads this (via Engine.GetDrModel / the DrModels column)
    // to classify the vehicle FreeKinematic-while-swerving vs LaneArc. Pure plan-phase SIDE-WRITE:
    // nothing on the Run()/golden path reads it, so it is byte-identical (determinism hash unmoved), and
    // it is only ever WRITTEN under LanelessRvo && _sublane -- left false for every parity scenario, so a
    // plain lane vehicle is always LaneArc. Recomputed fresh each real plan pass.
    public bool LateralManoeuvre;

    // P1E-4 (HIGH-DENSITY-P1E-DESIGN.md §1A, §9): device.rerouting equip + periodic-reroute
    // schedule -- DISTINCT from the obstacle-triggered BlockedByObstacleSeconds/AvoidedEdges
    // above (that is UpdateReroutes' own one-shot detour mechanism; this is MSDevice_Routing's
    // periodic congestion-reactive device). RerouteEquipped is drawn ONCE at creation
    // (Engine.BuildRuntime) from a salted per-entity RNG against ScenarioConfig.RerouteProbability
    // -- false (and NextRerouteTime left at +infinity, so it can never become due) for every
    // vehicle whenever ScenarioConfig.ReroutePeriod<=0 (every pre-P1E-4 scenario), which is
    // exactly the inert-when-absent guard. NextRerouteTime is the next sim-time this vehicle's
    // periodic reroute pass (Engine.UpdatePeriodicReroutes) is due; it re-arms by
    // `+= ReroutePeriod` each time the vehicle is actually considered (whether or not the
    // candidate route was installed -- §1B). LastRoutingTime is the sim-time of this vehicle's
    // own last periodic routing attempt (MSDevice_Routing's `myLastRouting`), read by the
    // skip-if-stale-weights guard (§1A: skip iff LastRoutingTime >= the last edge-weight
    // adaptation time) -- NegativeInfinity so a vehicle that has never routed is never
    // spuriously treated as "weights unchanged since I last routed".
    public bool RerouteEquipped;
    public double NextRerouteTime = double.PositiveInfinity;
    public double LastRoutingTime = double.NegativeInfinity;

    // P1E-6 (HIGH-DENSITY-P1E-DESIGN.md §11): true once this vehicle's ONE pre-insertion reroute
    // attempt (Engine.InsertDepartingVehicles' pre-insertion pass) has run -- regardless of whether
    // it actually installed a new route (structural failure / identical-edge-list short-circuit
    // both still set this, exactly like the periodic pass always re-arms NextRerouteTime whether or
    // not it installs, §1B). Guards the pass to fire AT MOST ONCE per vehicle lifetime, at/after its
    // own depart time, distinct from and never touched by the periodic schedule
    // (NextRerouteTime/LastRoutingTime above keep firing depart+period, +period, ... unchanged --
    // SUMO does both). Defaults false for every vehicle; `new VehicleRuntime{...}` in
    // Engine.BuildRuntime always constructs a fresh instance (never carries over a recycled slot's
    // old value), so a recycled EntityIndex's occupant starts with PreInsertionRerouteDone=false
    // again, same as any other freshly-built vehicle -- no separate reset code needed.
    public bool PreInsertionRerouteDone;

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §2, §5): true while this vehicle is mid-teleport -- it has
    // been lifted off its lane by the jam-check phase (MSVehicleTransfer::add) and is sitting in
    // Engine._transferQueue awaiting re-insertion (MSVehicleTransfer::checkInsertions). While set,
    // the vehicle is excluded from EVERY active-vehicle query (VehicleQuery, BuildActiveIndices,
    // BuildRegionActive, the parallel-emit scan, TryResolveActive) exactly as SUMO's transferring
    // vehicle is off the network (not planned, not moved, not emitted in FCD). Cleared when the
    // re-insertion pass places it back on a lane. Default false; ONLY ever set when
    // ScenarioConfig.TimeToTeleport>0, so every pre-P1F scenario (time-to-teleport=-1) never
    // touches it and the active-query filters stay byte-identical.
    public bool InTransfer;

    // GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2, docs/SERVE-PATH-PLAN.md): SUMO's
    // MSDevice_Tripinfo::myWaitingTime (MSDevice_Tripinfo::notifyMove, MSDevice_Tripinfo.cpp:179-193)
    // -- a TRIP-TOTAL accumulator, DISTINCT from WaitingTime above (which resets to 0 the instant the
    // vehicle moves/accelerates away -- that field is SUMO's *consecutive*-halt timer,
    // MSVehicle::updateWaitingTime, used by the all-way-stop tie-break). This one NEVER resets: each
    // Engine.ExecuteMoves step, while the vehicle is NOT currently halted at a reached <stop>
    // (!IsStoppedAtStop) AND newSpeed <= haltingSpeed AND this step's acceleration <=
    // accelThresholdForWaiting (0.5*maxAccel) -- the SAME predicate WaitingTime already evaluates --
    // `+= dt`. Written ONLY in Engine.ExecuteMoves; read ONLY by Engine's trip-arrival capture
    // (CaptureCompletedTrips) to populate a completed trip's tripinfo `waitingTime`. Default 0.0;
    // BuildRuntime always constructs a fresh VehicleRuntime (append or recycled slot), so this is
    // always 0 at a vehicle's insertion -- no separate reset path needed.
    public double TripWaitingTime;

    // GAP-2: SUMO's MSVehicle::myTimeLoss (MSVehicle::updateTimeLoss, MSVehicle.cpp:4095-4105) -- a
    // TRIP-TOTAL accumulator of "how much slower than the lane's free-flow speed was I", in seconds.
    // Each Engine.ExecuteMoves step, while the vehicle is NOT currently halted at a reached <stop>
    // (!IsStoppedAtStop): `+= dt * (vmax - newSpeed) / vmax`, where vmax is this lane's
    // KraussModel.LaneVehicleMaxSpeed for this vehicle (lane speed limit x this vehicle's SpeedFactor,
    // capped at VType.MaxSpeed) -- SUMO's `myLane->getVehicleMaxSpeed(this)`. Never resets. Written
    // ONLY in Engine.ExecuteMoves; read ONLY by CaptureCompletedTrips. Default 0.0, same fresh-
    // instance-per-insertion guarantee as TripWaitingTime above.
    public double TripTimeLoss;

    // GAP-2: the RESOLVED depart position -- this vehicle's Kinematics.Pos at the exact moment of
    // insertion (TryInsertOnLane's `insertPos`), captured ONCE there and never touched again. This is
    // SUMO's `-veh.getPositionOnLane()` seed for myRouteLength at NOTIFICATION_DEPARTED
    // (MSDevice_Tripinfo.cpp:239-245): unlike Kinematics.Pos (which advances as the vehicle moves and
    // wraps at each lane boundary), this stays the vehicle's ORIGINAL insertion offset for the whole
    // trip, which CaptureCompletedTrips needs for the routeLength formula (routeLength = sum of full
    // lengths of every route edge before the arrival edge, minus this depart pos, plus the configured
    // arrival pos). Default 0.0 -- always overwritten by TryInsertOnLane before a vehicle can become
    // Inserted (and therefore before it can ever Arrive).
    public double DepartPosResolved;
}
