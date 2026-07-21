using System.Runtime.InteropServices;
using Sim.Ingest;

namespace Sim.Core;

// Task 3: real Krauss/MSCFModel car-following speed law (ported from
// sumo/src/microsim/cfmodels/MSCFModel*.cpp -- see KraussModel.cs) wired into the plan/execute
// contract and lane-relative position model built in Task 2 (DESIGN.md "The plan/execute
// contract", "Seam 2").
public sealed partial class Engine : IEngine
{
    private NetworkModel? _network;

    // Perf (emit hot path): a dense Lane[] indexed by LaneHandle, materialized once per load from
    // _network.LanesByHandle. Emit (and any per-vehicle geometry) indexes this by v.LaneHandle
    // instead of hashing v.LaneId through the LanesById string dictionary -- byte-identical
    // (LanesByHandle[lane.Handle] == LanesById[lane.Id], and v.LaneHandle is kept in lockstep with
    // v.LaneId), but removes a string hash + dictionary probe per vehicle per frame.
    private Lane[] _lanesByHandle = Array.Empty<Lane>();
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

    // P2G-2 robustness: TryReResolveFromActualLane APPENDS to _laneSeqPool/_laneSeqArrival, and it is
    // reached from the REGION-PARALLEL ExecuteMoves phase (a vehicle that hits a lane boundary on a
    // non-pool lane -- rare under parity, FREQUENT under the coordinated model's aggressive lane-changing).
    // The append (Count-read + AddRange) must be atomic across worker threads, or concurrent re-resolves
    // overlap slices and corrupt the List's size -> IndexOutOfRange in ExecuteMoveVehicle's pool reads.
    // This lock serialises ONLY the (rare) append; reads of an already-committed slice are safe without it
    // (List resize is copy-based and _size only grows, so a valid index never faults). Uncontended (~free)
    // on the serial path, so every committed golden (all run serial via dotnet test) is byte-identical.
    private readonly object _laneSeqPoolLock = new();

    // L0d (PERF-ROADMAP.md): reused across steps by InsertDepartingVehicles (a serial, single-threaded
    // Input-phase call) instead of a fresh List+HashSet every step. Cleared at the start of each use.
    private readonly List<VehicleRuntime> _insertCandidates = new();
    private readonly HashSet<string> _insertBlockedLanes = new(StringComparer.Ordinal);

    // Perf (insert): memoize the insertion lane-sequence resolution. ResolveLaneSequenceHandlesWithArrival
    // is a PURE function of (route.Edges, DepartLaneIndex) -- so a candidate that waits several steps to
    // insert (blocked gap) re-resolves the SAME route every step, and vehicles sharing a (route, depart
    // lane) each resolve it. Route resolution is ~72% of the insert phase; caching by (RouteId,
    // DepartLaneIndex) computes each distinct one ONCE. Byte-identical (the cached arrays are read-only
    // -- AddRange copies them into the pool; nothing mutates them). Cleared per LoadScenario.
    private readonly Dictionary<(string RouteId, int DepartLane), (int[] Pool, int[] Arrival)> _insertRouteSeqCache = new();

    // C2-v: the ARRIVAL-lane pool, parallel to _laneSeqPool slot-for-slot (same LaneSeqStart/Len
    // slice). _laneSeqPool[k] is the lane the vehicle must reach to continue its route (the
    // strategic-LC target); _laneSeqArrival[k] is the lane it physically occupies on ENTERING that
    // slot's edge. They differ only at an intra-edge mid-route lane change; for every route with no
    // such change (every pre-C2-v scenario) the two pools are identical, so the crossing (which
    // reads _laneSeqArrival) is byte-identical to reading _laneSeqPool there.
    private readonly List<int> _laneSeqArrival = new();

    // Perf (PERF-ROADMAP.md Layer 0b): memoize NetworkModel.ComputeBestLanes -- a PURE function of
    // (route, current edge), both immutable for the scenario's lifetime. Without this it re-allocated
    // a List<LaneContinuation> (records) per lane-considering vehicle per step; with it, each unique
    // (routeId, edgeId) is computed ONCE and shared for the whole run. ConcurrentDictionary + a
    // state-passing GetOrAdd keeps it thread-safe AND allocation-free on the (overwhelmingly common)
    // cache-hit path under UseParallelPlan; the value factory is a pure function of immutable inputs,
    // so which thread first computes a given key can never change the result (determinism preserved).
    private readonly System.Collections.Concurrent.ConcurrentDictionary<(string RouteId, string EdgeId), IReadOnlyList<LaneContinuation>> _bestLanesCache = new();

    // Bugfix (surfaced by PANIC-EVAC-PHASE5-DESIGN.md T2.1 on a 2-lanes-per-edge net):
    // RerouteActive/UpdateReroutes replace an ACTIVE vehicle's remaining LANE sequence
    // (LaneSeqStart/Len) via the command buffer, but v.Def.RouteId (the key `_routesById` and
    // `_bestLanesCache` are keyed on) is `init`-only on VehicleRuntime and cannot be repointed.
    // v.Def.RouteId is also frequently SHARED by other vehicles still on the ORIGINAL route (SUMO's
    // route reuse -- <route id="r0"/> referenced by several <vehicle>s), so mutating
    // `_routesById[v.Def.RouteId]` in place to reflect ONE rerouted vehicle's new path would corrupt
    // every other vehicle still reading that same entry (TryStrategicLaneChange/KeepRightStrategicStay
    // would then see edges that are not really theirs -- exactly the "edge is not part of the given
    // route" crash this fixes). Instead, a rerouted vehicle gets its OWN synthetic route id, registered
    // fresh in `_routesById` and remembered here by EntityIndex; `EffectiveRouteId` is consulted by the
    // only two ACTIVE-vehicle-hot-path reads of `_routesById[v.Def.RouteId]` + `_bestLanesCache`
    // (TryStrategicLaneChange, KeepRightStrategicStay) -- every other `_routesById[v.Def.RouteId]` read
    // in this file is insertion-time only (a not-yet-active candidate can never have been rerouted) and
    // is deliberately left untouched. Empty for
    // every vehicle that never calls SetDestination/Reroute -- i.e. the entire golden/parity path,
    // where this table is never written and every lookup falls through to v.Def.RouteId unchanged
    // (byte-identical; hash 909605E965BFFE59 unmoved).
    private readonly Dictionary<int, string> _effectiveRouteIdByEntity = new();
    private int _rerouteRouteCounter;

    private string EffectiveRouteId(VehicleRuntime v) =>
        _effectiveRouteIdByEntity.TryGetValue(v.EntityIndex, out var id) ? id : v.Def.RouteId;

    // Register `fullEdges` (starting at the vehicle's CURRENT edge, exactly like RerouteActive's/
    // UpdateReroutes' own `newEdges`) as a fresh route this vehicle alone owns, so its own remaining
    // path is always what `RouteFor`/`EffectiveRouteId` resolve to from here on, without touching the
    // original (possibly shared) `_routesById[v.Def.RouteId]` entry at all.
    private void RegisterRerouted(VehicleRuntime v, IReadOnlyList<string> fullEdges)
    {
        var newId = $"{v.Def.RouteId}__reroute{v.EntityIndex}_{_rerouteRouteCounter++}";
        _routesById[newId] = new Route(newId, new List<string>(fullEdges));
        _effectiveRouteIdByEntity[v.EntityIndex] = newId;
    }

    // P1E-4 (§0.5.3 route-slot recycling): the periodic congestion-reactive device conceptually
    // re-installs a route every ReroutePeriod seconds per equipped vehicle (no improvement gate,
    // §1B) -- minting a FRESH id every time (RegisterRerouted's own `_reroute{idx}_{counter++}`
    // pattern) would grow `_routesById` unboundedly over a long run at scale. Instead this uses a
    // STABLE, entity-index-only id (no counter) that every periodic reroute of THIS vehicle simply
    // OVERWRITES in place -- `_routesById` gains at most ONE extra entry per equipped vehicle for
    // its entire lifetime, never one per reroute (see RungHDp1e4RerouteDeviceTests' recycling
    // test). Safe across slot recycling too: the id is a pure function of EntityIndex, so a reused
    // slot's next periodic reroute simply overwrites the same key again with the new occupant's
    // edges -- never a second, orphaned entry.
    private void RegisterPeriodicReroute(VehicleRuntime v, IReadOnlyList<string> fullEdges)
    {
        var routeId = $"__periodicReroute{v.EntityIndex}";
        _routesById[routeId] = new Route(routeId, new List<string>(fullEdges));
        _effectiveRouteIdByEntity[v.EntityIndex] = routeId;
    }

    // P1E-4 (§1B): the vehicle's remaining route as a distinct, in-order edge list -- currentEdge
    // followed by every FUTURE normal edge still left in its lane-sequence pool slice, deduplicated
    // and excluding internal/junction lanes' edges. Factored out of UpdateReroutes' own inline
    // logic (left untouched there) so the periodic device's identical-edge-list short-circuit
    // (§1B "no improvement gate, but short-circuit on an identical edge list") compares against
    // the exact same notion of "current remaining route" the obstacle-based reroute already uses.
    private List<string> CurrentRemainingRouteEdges(VehicleRuntime v, string currentEdge)
    {
        var result = new List<string> { currentEdge };
        var seen = new HashSet<string>(StringComparer.Ordinal) { currentEdge };
        for (var i = v.LaneSeqIndex + 1; i < v.LaneSeqLen; i++)
        {
            var seqLane = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]];
            if (seqLane.Id.StartsWith(':'))
            {
                continue;
            }

            if (seen.Add(seqLane.EdgeId))
            {
                result.Add(seqLane.EdgeId);
            }
        }

        return result;
    }

    // R4 (rail signal): for each lane whose outgoing connection is controlled by a rail_signal
    // junction, the set of "conflict lane" HANDLES that must be clear for the signal to show green --
    // ported from MSRailSignal::DriveWay::conflictLaneOccupied (the driveway's forward block's bidi
    // partners: the opposing lanes on the shared single track). This is a DENSE array indexed by the
    // approaching (signal-guarded) lane's HANDLE (== its index into LanesByHandle): entry `h` is the
    // conflict-handle set for lane handle `h`, or null when that lane has no rail signal. The whole
    // array is null when the net has no rail_signal junction, so the hot-path check is a single null
    // test (inert-when-absent) for all road/rail non-signal scenarios. Built cold in
    // BuildRailSignalInfo; the per-step constraint only does int-indexed array reads + handle compares
    // (no string hashing/comparison), matching the engine's dense-handle hot-path idiom.
    private int[][]? _railSignalConflictLaneHandles;

    // R5 (rail crossing): rail_crossing junction info, ported from MSRailCrossing. A rail_crossing
    // controls the ROAD links across the tracks (road vehicles yield to trains). Each crossing gets a
    // dense index (0..N-1). _railCrossingByRoadLaneHandle is indexed by ROAD lane HANDLE -> that
    // lane's crossing index, or -1 (not a controlled approach); the whole array is null when the net
    // has no rail_crossing junction. _railCrossingViaLaneHandles[c] is crossing c's internal RAIL
    // via-lane HANDLES (a train there closes the crossing). All handle-based (int-indexed array reads
    // + handle compares on the hot path); empty/null when absent, so the crossing paths are a no-op
    // for every non-crossing net.
    private int[]? _railCrossingByRoadLaneHandle;
    private int[][] _railCrossingViaLaneHandles = Array.Empty<int[]>();
    // Per-crossing (dense index) state machine (MSRailCrossing::updateCurrentPhase): myStep 0='G'
    // 1='y' 2='r' 3='u', and the next-switch time. Reset at the top of every Run().
    // _railCrossingState[c] is the current road-link state char crossing c's road vehicles read.
    private int[] _railCrossingStep = Array.Empty<int>();
    private double[] _railCrossingNextSwitch = Array.Empty<double>();
    private char[] _railCrossingState = Array.Empty<char>();

    // D3: this vehicle's scheduled stops (Sim.Ingest.VehicleDef.Stops), keyed by
    // VehicleRuntime.EntityIndex -- replaces the old per-vehicle `Queue<StopRuntime> Stops`
    // managed field. Populated once at LoadScenario, ONLY for vehicles that actually have stops
    // (def.Stops.Count > 0); absent from this dictionary is exactly the "no stops" fast path
    // (Count==0) every stop-consuming call site already handles.
    private readonly Dictionary<int, Queue<StopRuntime>> _stopsByEntity = new();

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C, §2, §5): the persistent teleport transfer queue --
    // the MSVehicleTransfer::myVehicles analog. A vehicle picked for a jam-teleport is lifted off
    // its lane (InTransfer=true) and appended here with the route-edge index of the edge it has
    // been jumped onto (succEdge(1)) and its virtual-proceed clock; the re-insertion pass
    // (ProcessTransferQueue, the checkInsertions analog) drains it, sorted by EntityIndex
    // ("for repeatable parallel simulation", MSVehicleTransfer.cpp:98). Empty and never touched
    // whenever TimeToTeleport<=0 (the valve is off), so byte-identical for every pre-P1F scenario.
    private readonly List<TransferEntry> _transferQueue = new();

    // P1F-2: reusable scratch for the jam-check phase (CheckJamTeleports) -- the per-lane frontmost
    // non-stopped vehicle (MSLane::executeMovements' firstNotStopped) and the collected teleport
    // candidates. Cleared at the top of each jam-check; never allocated on the pre-P1F path (the
    // phase is gated on TimeToTeleport>0, so these stay empty for every existing scenario).
    private readonly Dictionary<int, VehicleRuntime> _jamFrontmost = new();
    private readonly List<VehicleRuntime> _jamCandidates = new();

    // P1F-2: one queued teleporting vehicle. EdgeIndex is the index INTO its route's Edges list of
    // the edge it is currently (virtually) sitting on -- succEdge(1) at teleport time, advanced by
    // the virtual-proceed hop. ProceedTime is the sim-time the virtual-proceed hop becomes due
    // (MSVehicleTransfer's myProceedTime); -1 = not yet initialized (still trying to re-insert on
    // the current edge this step).
    private sealed class TransferEntry
    {
        public required VehicleRuntime Veh;
        public required int EdgeIndex;
        public double ProceedTime = -1.0;
    }

    // D3: this vehicle's already-routed-around-once edge set, keyed by EntityIndex -- replaces
    // the old per-vehicle `HashSet<string> AvoidedEdges` managed field. Lazily created only when
    // a vehicle first reroutes (UpdateReroutes); off the hot path (reroute is opt-in via
    // RerouteThresholdSeconds, +infinity by default).
    private readonly Dictionary<int, HashSet<string>> _avoidedByEntity = new();

    // SUMOSHARP-API.md §9: vehicle-slot recycling. Despawn pushes the freed EntityIndex here; the next
    // runtime SpawnVehicle reuses it (rebuilding _vehicles[idx] in place + resetting the idx-keyed side
    // state) instead of growing _vehicles forever -- bounded memory for long-lived spawn/despawn-heavy
    // hosts. ONLY ever populated by Despawn (a runtime-host API), so the golden/flow path -- which never
    // despawns -- never recycles and stays byte-identical. Deterministic (LIFO over a deterministic
    // spawn/despawn call order). Toggle off for stable, monotonically-growing indices (e.g. debugging).
    private readonly Stack<int> _freeEntitySlots = new();
    public bool RecycleVehicleSlots { get; set; } = true;

    // B1: external-obstacle store (DESIGN.md "Two futures" -- live-reactivity input surface, not
    // a SUMO concept). SUMOSHARP-API.md §4.3: a handle-keyed struct-of-arrays (ObstacleStore), replacing
    // the former `Dictionary<string, ExternalObstacle>` so per-step corrections are zero-allocation.
    // Deliberately NOT cleared by LoadScenario (tests inject obstacles after loading, before Run) and
    // empty by default, which is exactly the inert-when-absent guard: with no entries, ObstacleConstraint
    // below is a trivial +infinity no-op and every parity scenario's constraints list is unaffected.
    private readonly ObstacleStore _obstacles = new();

    // SUMOSHARP-API.md §9: MUTABLE vType/route registries so demand is not fixed to the loaded rou.xml.
    // Seeded from _demand at each load; runtime DefineVType / SpawnVehicle add to them. Every engine
    // lookup that used to read the immutable `_demand.VTypesById`/`RoutesById` reads these instead --
    // seeded identically, so the loaded-scenario path stays byte-identical. `_vTypeIds` gives each vType
    // a stable VTypeHandle index; the counters name auto-generated runtime routes/vehicles.
    private readonly Dictionary<string, VType> _vTypesById = new(StringComparer.Ordinal);
    private readonly Dictionary<string, Route> _routesById = new(StringComparer.Ordinal);
    private readonly List<string> _vTypeIds = new();
    private int _runtimeRouteCounter;
    private int _runtimeVehicleCounter;

    // GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2): every vehicle that has genuinely ARRIVED
    // (route-end, not a jam-teleport removal or X1 de-jam despawn) so far this scenario, in the
    // order CaptureCompletedTrips appended them (EntityIndex order within a step -- see that
    // method's own comment on why). Cleared on LoadScenario so a re-run starts empty.
    private readonly List<CompletedTripInfo> _completedTrips = new();

    // Public read-only view for a host/CLI (Sim.Sumo's --tripinfo-output writer) to consume after
    // Run()/Step(). Valid immediately after any Advance() call, growing as more vehicles arrive.
    public IReadOnlyList<CompletedTripInfo> CompletedTrips => _completedTrips;

    // SUMO's built-in default vehicle type id -- auto-registered at load so SpawnVehicle works without a
    // prior DefineVType. Harmless for loaded scenarios (no vehicle references it unless spawned).
    private const string DefaultVTypeId = "DEFAULT_VEHTYPE";

    // W1 (warm-start): number of steps advanced so far on the current timeline (via Run or WarmUp),
    // reset to 0 by LoadScenario. Advance resets the timeline state machines and starts the clock at
    // _config.Begin only while this is 0, so WarmUp(W) followed by Run(N) is one continuous run
    // (no mid-timeline reset), while a fresh engine's first Run is byte-identical to before W1.
    private int _elapsedSteps;

    // Rung ER3 (give-way): true iff the loaded demand contains at least one vType with an active
    // blue-light siren (ResolvedVType.HasBluelight). Computed once per LoadScenario. The whole
    // give-way subsystem (DetectGiveWaySide and the ER4/ER5 execution arms) short-circuits when
    // this is false, so every scenario with no emergency vehicle -- i.e. every committed parity
    // scenario and the bench -- takes exactly zero give-way work and stays byte-identical.
    private bool _anyBluelight;

    // Rung OV1 (opposite-direction overtaking): true iff the loaded demand has any vType with
    // lcOpposite. DetectOvertake short-circuits when false, so every scenario with no such vType
    // (all committed scenarios + the bench) takes zero opposite-direction work and stays identical.
    private bool _anyLcOpposite;

    // Phase 2 (sublane, P2.3): SUMO's MSGlobals::gLateralResolution master switch -- true iff the
    // scenario sets lateral-resolution > 0. The GLOBAL sublane gate (not per-vType): the sublane
    // lateral driver (ComputeSublaneLateral) runs ONLY when this is true, so every phase-1 scenario
    // (lateral-resolution 0) takes exactly the pre-existing lane-centred path and stays byte-identical.
    // Set once per LoadScenario from the immutable config.
    private bool _sublane;

    // Perf (willPass/plan fusion): the disqualifier master-switches. The fusion (PlanMovements reuses
    // the willPass pre-pass Intent instead of recomputing it) is byte-identical ONLY when every
    // prePass/real divergence OTHER than the crossing-yield relax is inert -- see the FusionEligible
    // property. `_anyKraussDawdle` = some vType takes a Krauss dawdle RNG draw (sigma>0): its pre-pass
    // uses a throwaway RNG copy, so reusing that Intent would skip the real RngState advance and desync
    // the sigma>0 stream. `_anyIdmm` = some vType is IDMM: the real pass advances LevelOfService, the
    // pre-pass does not. Both aggregated in CreateRuntime exactly like _anyBluelight (so a runtime flow
    // that introduces such a vType flips them too). Bluelight/lcOpposite reuse the switches above.
    private bool _anyKraussDawdle;
    private bool _anyIdmm;

    // Perf (willPass/plan fusion): false iff actionStepLength > 0 and != the step length -- an
    // action-step-skipped plan records LastActionTime only in the real pass, so reusing the pre-pass
    // Intent would drop that write. Computed once per LoadScenario from the immutable config.
    private bool _actionStepFusionOk = true;

    // Perf (willPass/plan fusion): fusion is byte-identical only when NONE of the side-effect
    // divergences between the pre-pass and the real plan can fire this step. Obstacles are checked
    // live (they can be added mid-run and drive LatOffset via ComputeLateralEvasion); the rest are
    // load-time (or flow-time) master switches. When true, PlanMovements reuses the pre-pass Intent
    // for every non-crossing-yield vehicle; when false the exact two-pass path runs (byte-identical).
    private bool FusionEligible =>
        _actionStepFusionOk && !_anyKraussDawdle && !_anyIdmm && !_anyBluelight && !_anyLcOpposite
        && _obstacles.Count == 0 && !_sublane;

    // F2 (probabilistic flow): per-<flow probability=> runtime insertion state, index-aligned with
    // _demand.ProbabilisticFlows. `_probFlowRng[i]` is that flow's own seeded Bernoulli stream (one
    // draw per active flow per step); `_probFlowCounter[i]` is its running arrival counter (the k in
    // "<flowId>.<k>"). Both are seeded/reset ONCE per LoadScenario (like each vehicle's RngState),
    // NOT per Run, so a WarmUp(W) + Run(N) generates one continuous, deterministic arrival stream.
    // Empty for every scenario with no probability flow (GenerateProbabilisticFlows short-circuits
    // on Length==0), so all committed scenarios + the bench stay byte-identical.
    private VehicleRng[] _probFlowRng = Array.Empty<VehicleRng>();
    private int[] _probFlowCounter = Array.Empty<int>();

    // Rung OV1/OV2: the MINIMUM clear-ahead distance for an overtake even when the oncoming lane
    // looks empty/very distant -- a floor beneath OV2's speed-based gap-acceptance requirement.
    private const double OvertakeMinClearDist = 30.0;

    // Rung OV2: extra safety margin added to the computed required-clear distance (the buffer left
    // between completing the pass and the point the oncoming vehicle would reach).
    private const double OvertakeSafetyGap = 25.0;

    // Rung OV1: a leader is "holding ego up" (worth overtaking) only if it is this fraction below
    // ego's own free-flow speed on the lane, and within OvertakeLeaderMaxGap ahead.
    private const double OvertakeLeaderSlowFraction = 0.8;
    private const double OvertakeLeaderMaxGap = 60.0;

    // Rung OV4 (cooperative oncoming shift): how far ahead an oncoming vehicle perceives a spilled
    // overtaker closing head-on down its own (bidi) lane and starts widening the corridor by pulling
    // to its OWN outer edge. Much larger than the give-way reaction distance (25 m, a same-direction
    // siren closing slowly from behind): a head-on encroachment closes at the SUM of both speeds
    // (~28 m/s here), so ~200 m gives a driver ~7 s to ease over before the closest approach -- and,
    // by design, the oncoming makes room throughout the window in which OV2 keeps the overtaker
    // committed and spilled (OV2's own gap acceptance still guarantees the pass finishes before the
    // head-on arrives, so the shift is defence-in-depth margin, never what prevents a collision).
    private const double CooperativeShiftReactionDist = 200.0;

    // Rung OV4: how far an overtaker must have spilled toward the oncoming lane (positive LatOffset,
    // always toward the oncoming lane in the OV3 spill) before an oncoming driver treats it as a
    // head-on encroachment worth making room for. Set above any normal / give-way drift (a give-way
    // edge target is <= laneHalfWidth - egoHalfWidth ~ 0.7 m) and below the OV3 spill (~2.3 m), so it
    // fires only for a genuine centre-line-crossing overtake, never for ordinary lateral motion.
    private const double CooperativeShiftSpillThreshold = 1.0;

    // Rung D3 (coupled OV2/OV4): the minimum lateral corridor (m) left between a spilled overtaker and
    // a cooperatively-shifted oncoming for OV2 to accept a side-by-side pass. Positive only on roads
    // wide enough for the shifted pass -- negative on scenario 57's 3.2 m lanes, so D3 never engages
    // there and every existing OV fixture is byte-identical.
    private const double CooperativeSideBySideMargin = 0.3;

    // Rung ER3 (give-way): SUMO's device.bluelight.reactiondist default (MSDevice_Bluelight.cpp:58)
    // -- the range within which surrounding drivers perceive the siren and start clearing the way.
    private const double GiveWayReactionDist = 25.0;

    // B3: reroute-around-prolonged-blockage (DESIGN.md "Two futures" -- live-reactivity, not a
    // ported SUMO code path; SUMO's analog is a rerouting device / <rerouter> reacting to a
    // closed edge). Left at +infinity by default, which makes UpdateReroutes below an immediate
    // no-op every step -- the inert-when-absent guard: reroute is strictly opt-in, so no existing
    // (obstacle-free or obstacle-present-but-untested) parity scenario is ever affected.
    public double RerouteThresholdSeconds { get; set; } = double.PositiveInfinity;

    // B2 router, built once lazily from the loaded (immutable) network and cached -- cheap to
    // construct, but there is no reason to rebuild it every step. Null until first needed (either
    // LoadScenario has not run yet, or UpdateReroutes never actually reroutes anything). P1E-4's
    // periodic reroute device reuses this SAME lazily-built instance (NetworkRouter.Route/
    // RouteAStar allocate only per-call local search state -- no shared mutable fields -- so it is
    // safe to call concurrently from UpdatePeriodicReroutes' Parallel.For).
    private NetworkRouter? _router;

    // P1E-4 (HIGH-DENSITY-P1E-DESIGN.md §1C/§2/§3, seam #2): the live per-edge smoothed-speed
    // table (Sim.Ingest.RerouteEdgeWeights), constructed ONCE at LoadScenario iff
    // ScenarioConfig.ReroutePeriod>0, else left null -- the inert-when-absent guard: every
    // pre-P1E-4 scenario (and any scenario that simply omits <processing><device.rerouting.*>)
    // never allocates this table and never enters either of the two new step-loop passes below.
    private RerouteEdgeWeights? _edgeWeights;

    // P1E-4 (§3): "getLastAdaptation()" analog -- the sim-time UpdateRerouteEdgeWeights last wrote
    // the weight table. Seeded to ScenarioConfig.Begin at LoadScenario (before any update has run)
    // so a vehicle's LastRoutingTime==NegativeInfinity always compares as stale (< this), and the
    // periodic reroute pass's own skip-if-stale-weights guard reads it every step.
    private double _lastAdaptationTime;

    // P1E-4 salts (see VehicleRng.SeedFor's 3-arg overload / SpeedFactorRngSalt's own comment for
    // the independence argument): two INDEPENDENT once-at-creation draws, neither ever shared with
    // RngState/SpeedFactorRngSalt/VTypeDistRngSalt/ProbFlowRngSalt. "RerEquip"/"RerJitte" packed
    // big-endian ASCII, purely mnemonic -- not itself load-bearing.
    private const ulong RerouteEquipRngSalt = 0x5265724571756970UL;
    private const ulong RerouteJitterRngSalt = 0x5265724A69747465UL;

    // P1E-4 scratch (D4 zero-steady-state-alloc discipline): reused across steps by
    // UpdatePeriodicReroutes (a serial collection pass followed by a Parallel.For, matching
    // _insertCandidates' own reuse pattern) and UpdateRerouteEdgeWeights. Cleared/resized at the
    // start of each use; never read stale across calls.
    private readonly List<VehicleRuntime> _rerouteBatchScratch = new();
    private IReadOnlyList<string>?[] _rerouteCandidateScratch = Array.Empty<IReadOnlyList<string>?>();

    // P1E-6 (§11) scratch: same zero-steady-state-alloc discipline as the periodic pass's own
    // scratch above, but kept as SEPARATE fields (rather than reused) since the pre-insertion pass
    // runs earlier in the SAME step, inside InsertDepartingVehicles -- before UpdatePeriodicReroutes
    // clears/reuses its own scratch later this same AdvanceOneStep call. Cleared/resized at the
    // start of each use; never read stale across calls.
    private readonly List<VehicleRuntime> _preInsertRerouteBatchScratch = new();
    private IReadOnlyList<string>?[] _preInsertRerouteCandidateScratch = Array.Empty<IReadOnlyList<string>?>();
    private readonly Dictionary<string, double> _rerouteEdgeSpeedSumScratch = new(StringComparer.Ordinal);
    private readonly Dictionary<string, int> _rerouteEdgeSpeedCountScratch = new(StringComparer.Ordinal);

    // D4 (FDP zero-alloc `OnUpdate` rule): ONE reusable LaneNeighborQuery, (re)built only when
    // LoadScenario changes the network (its bucket count is sized off `LanesByHandle.Count`).
    // Refilled -- not reconstructed -- twice per step: once for the pre-move snapshot (Run(),
    // before PlanMovements) and once for the post-move snapshot (DecideSpeedGainChanges(), after
    // ExecuteMoves). See LaneNeighborQuery's own header comment for why a single reused instance
    // is safe even though it is refilled twice per step.
    private LaneNeighborQuery? _neighborQuery;

    // Perf (super-linear fix): FindFoeVehicle used to scan EVERY active vehicle's whole remaining
    // lane sequence, per foe-link, per vehicle-at-junction -- an O(vehicles^2 * routeLen) term that
    // dominated the city-scale plan phase (profiled: JunctionYield ~128 us/call, ~55% of plan). The
    // set it wants ("vehicles whose remaining route contains internal lane H") is a pure function of
    // the frozen start-of-step routes, so it is precomputed ONCE per step into a per-internal-lane
    // index (BuildFoeApproachIndex, called right after neighbors.Refill) and looked up in O(1).
    // _foeApproachFirst/Second hold the FIRST TWO distinct vehicles, in _vehicles order, whose route
    // contains that lane -- two, not one, so FindFoeVehicle's "skip ego, take the next" exclusion is
    // reproduced byte-identically (ego can only ever be the first of the two). Indexed by dense lane
    // handle; only internal (':'-prefixed) lanes are ever queried, so only those are recorded.
    private bool[] _isInternalLane = Array.Empty<bool>();
    private VehicleRuntime?[] _foeApproachFirst = Array.Empty<VehicleRuntime?>();
    private VehicleRuntime?[] _foeApproachSecond = Array.Empty<VehicleRuntime?>();

    // Perf (Export-phase parallelism): a reusable, index-keyed buffer for the parallel EmitTrajectory
    // branch. Emit's per-vehicle cost is the LaneGeometry.PositionAtOffset trig, which reads only
    // immutable geometry + the vehicle's OWN settled Kinematics -- independent per vehicle, so it is
    // computed concurrently into slot i (== _vehicles[i]; null == inactive this step) and then appended
    // to the TrajectorySet serially. Byte-identical to the serial append: the comparator and the
    // determinism hash both key by (vehicle, time) (TrajectoryComparator / TrajectorySet.Index), so
    // emission order is irrelevant, and each slot holds the SAME pure-function value the serial path
    // would compute. Grown on demand (never shrunk); every in-range slot is overwritten each step
    // (_vehicles is append-only, so Count is monotonic -- no stale slot survives).
    private TrajectoryPoint?[] _emitScratch = Array.Empty<TrajectoryPoint?>();

    // Perf (dense active list): reused per-step compaction of the ACTIVE vehicle indices
    // (Inserted && !Arrived) into a dense list, so the parallel willPass/plan loops dispatch over
    // ~active-count work items instead of _vehicles.Count. On city-3000 ~40% of _vehicles are not-
    // yet-departed or already-arrived; the old per-index Parallel.For touched each of those
    // scattered heap objects' headers (the Inserted/Arrived bools) once per phase per step just to
    // skip them -- pure bandwidth waste on a memory-bound loop. Rebuilt once per step (serial O(N),
    // cache-linear over _vehicles) from the frozen post-insertion active set, which is stable
    // through the plan phases (arrival is applied later, in ExecuteMoves). Byte-identical: same
    // vehicles, same per-ego writes, order-independent (the loops write only each ego's own fields).
    private readonly List<int> _activeIndices = new();

    // Perf (SPATIAL-OPT probe -- see SPATIAL-OPT.md): the per-vehicle hot fields the SAME-LANE gap
    // math reads off a foe, packed AoS into one ~cache-line struct. `_packed` is rebuilt each step in
    // (lane, pos-ascending) order, so a vehicle's same-lane leader is a NEARBY slot (sequential /
    // prefetched) instead of a random foe-object deref -- the only thing that attacks the random-
    // access bandwidth wall (per-field SoA and AoS-by-EntityIndex both failed because the index is
    // random; see PERF-HANDOVER.md). doubles (not floats) for the vType scalars so the arithmetic is
    // byte-identical. Off by default (`SpatialPlan`), so the deterministic path is untouched.
    internal readonly struct HotVeh
    {
        public readonly double Pos;
        public readonly double Speed;
        public readonly double LatOffset;
        public readonly double Length;
        public readonly double Decel;
        public readonly double Width;
        public readonly bool IsCacc;
        public readonly int EntityIndex;
        public readonly int LaneHandle;
        public HotVeh(double pos, double speed, double latOffset, double length, double decel, double width, bool isCacc, int entityIndex, int laneHandle)
        {
            Pos = pos; Speed = speed; LatOffset = latOffset;
            Length = length; Decel = decel; Width = width; IsCacc = isCacc;
            EntityIndex = entityIndex; LaneHandle = laneHandle;
        }
    }

    private HotVeh[] _packed = System.Array.Empty<HotVeh>();
    private int[] _leaderSlotByPacked = System.Array.Empty<int>();
    private int _packedCount;

    // Perf (domain decomposition -- DOMAIN-DECOMP.md): opt-in SPATIAL partitioning of the parallel
    // plan/willPass. Instead of a per-vehicle Parallel.For (each worker gets an EntityIndex-order,
    // spatially-INCOHERENT slice whose leaders scatter across the whole ~1.6 MB vehicle set -> DRAM
    // traffic), group active vehicles into spatial REGIONS (a G x G grid over the network bounding
    // box) and run one task per region. A region's ~N/G^2 vehicles + their (mostly in-region) leaders
    // form a small working set that stays in L2, cutting the memory traffic that caps scaling at ~3x.
    // BYTE-IDENTICAL: the plan writes only each ego's own MoveIntent (order-independent) and reads the
    // frozen start-of-step neighbour snapshot -- partitioning the WORK changes nothing about the
    // result. Off by default. `RegionGrid` = G (regions = G^2); set before LoadScenario.
    public bool RegionPlan;
    public int RegionGrid = 4;
    private int[] _laneRegion = System.Array.Empty<int>();
    private int _regionCount;
    private List<int>[] _regionActive = System.Array.Empty<List<int>>();
    private List<int>[] _regionLanes = System.Array.Empty<List<int>>();

    // Assign every lane to a spatial region (G x G grid tile over the network's lane-centroid bounding
    // box), once per LoadScenario. A vehicle's region is its current lane's region. Cheap, cold path.
    private void ComputeLaneRegions()
    {
        var laneCount = _network!.LanesByHandle.Count;
        _laneRegion = new int[laneCount];
        var cx = new double[laneCount];
        var cy = new double[laneCount];
        double minX = double.MaxValue, minY = double.MaxValue, maxX = double.MinValue, maxY = double.MinValue;
        for (var h = 0; h < laneCount; h++)
        {
            var shape = _network.LanesByHandle[h].Shape;
            double sx = 0, sy = 0;
            if (shape.Count > 0)
            {
                foreach (var p in shape)
                {
                    sx += p.X;
                    sy += p.Y;
                }

                sx /= shape.Count;
                sy /= shape.Count;
            }

            cx[h] = sx;
            cy[h] = sy;
            if (sx < minX) minX = sx;
            if (sx > maxX) maxX = sx;
            if (sy < minY) minY = sy;
            if (sy > maxY) maxY = sy;
        }

        var g = Math.Max(1, RegionGrid);
        _regionCount = g * g;
        var wx = Math.Max(1e-9, (maxX - minX) / g);
        var wy = Math.Max(1e-9, (maxY - minY) / g);
        for (var h = 0; h < laneCount; h++)
        {
            var tx = Math.Min(g - 1, Math.Max(0, (int)((cx[h] - minX) / wx)));
            var ty = Math.Min(g - 1, Math.Max(0, (int)((cy[h] - minY) / wy)));
            _laneRegion[h] = (ty * g) + tx;
        }

        _regionActive = new List<int>[_regionCount];
        _regionLanes = new List<int>[_regionCount];
        for (var r = 0; r < _regionCount; r++)
        {
            _regionActive[r] = new List<int>();
            _regionLanes[r] = new List<int>();
        }

        // Inverse map region -> its lane handles (for region-parallel refill: each region owns a
        // disjoint set of lanes, so its buckets are written by only its task -- no contention).
        for (var h = 0; h < laneCount; h++)
        {
            _regionLanes[_laneRegion[h]].Add(h);
        }
    }

    // Group active (Inserted && !Arrived) vehicle indices by their current lane's spatial region.
    // Rebuilt once per step (serial O(N) scan) before the region-parallel plan/willPass consume it.
    private void BuildRegionActive()
    {
        for (var r = 0; r < _regionCount; r++)
        {
            _regionActive[r].Clear();
        }

        var vehicles = _vehicles;
        for (var i = 0; i < vehicles.Count; i++)
        {
            var v = vehicles[i];
            // P1F-2: exclude mid-teleport vehicles (InTransfer) -- inert unless TimeToTeleport>0.
            if (v.Inserted && !v.Arrived && !v.InTransfer)
            {
                _regionActive[_laneRegion[v.LaneHandle]].Add(i);
            }
        }
    }

    // Perf (SPATIAL-OPT probe): opt-in for the spatial plan path (iterate `_packed` in (lane,pos)
    // order, same-lane leader read from the adjacent packed slot). OFF by default -> deterministic
    // path (hash 909605E965BFFE59) untouched. Byte-identical when on (the packed leader is the same
    // vehicle GetLeader returns, same start-of-step field values); it is a SCHEDULE/locality change.
    public bool SpatialPlan;

    // SPATIAL-OPT probe: build `_packed` in (lane, pos-ascending) order from the already-sorted
    // neighbor buckets, and precompute each slot's same-lane leader slot (nearest strictly-ahead by
    // pos, co-located skipped -- byte-identical to GetLeader's selection). O(active); short forward
    // scans. Called after Refill + BuildActiveIndices, before the plan phases read it.
    private void BuildPacked(LaneNeighborQuery neighbors)
    {
        var n = _activeIndices.Count;
        if (_packed.Length < n)
        {
            _packed = new HotVeh[n];
            _leaderSlotByPacked = new int[n];
        }

        // Gather + leader-slot precompute, SERIAL. Parallelizing per-lane was tried and REGRESSED
        // (525 ms vs 305 ms @8t): the gather is scattered Kinematics reads (bandwidth-bound), so
        // spreading them across threads only adds contention + dispatch over many empty lanes. The
        // deeper problem this exposes: this per-step random gather from scattered objects costs about
        // as much as the sequential-leader read saves in the plan (~305 ms vs ~239 ms), so total
        // scattered-read traffic is CONSERVED, not reduced -- net-neutral wall. See SPATIAL-OPT.md:
        // a real win needs a PERSISTENT spatially-ordered store (incremental re-sort across steps),
        // not this rebuild-from-scratch gather. This probe validates the mechanism (plan -11%,
        // byte-identical) and is the groundwork for that; it is off by default (SpatialPlan).
        var laneCount = _network!.LanesByHandle.Count;
        var idx = 0;
        for (var h = 0; h < laneCount; h++)
        {
            var bucket = neighbors.OnLane(h); // pos-ascending
            for (var k = 0; k < bucket.Count; k++)
            {
                var v = bucket[k];
                var kin = v.Kinematics;
                var vt = v.VType;
                _packed[idx++] = new HotVeh(
                    kin.Pos, kin.Speed, kin.LatOffset,
                    vt.Length, vt.Decel, vt.Width, vt.CarFollowModel == "CACC",
                    v.EntityIndex, h);
            }
        }

        _packedCount = idx;

        for (var i = 0; i < _packedCount; i++)
        {
            var slot = -1;
            var egoPos = _packed[i].Pos;
            var egoLane = _packed[i].LaneHandle;
            for (var j = i + 1; j < _packedCount && _packed[j].LaneHandle == egoLane; j++)
            {
                if (_packed[j].Pos > egoPos)
                {
                    slot = j; // nearest strictly-ahead same-lane vehicle = the leader
                    break;
                }
                // Pos == egoPos (co-located): not strictly ahead -> skip, matching GetLeader.
            }

            _leaderSlotByPacked[i] = slot;
        }
    }

    // Compact the current active set (Inserted && !Arrived) into _activeIndices in ascending
    // _vehicles order (so iteration order matches the former 0..Count scan, minus the gaps). Called
    // once per step before the parallel willPass/plan phases; only needed on the parallel path.
    private void BuildActiveIndices()
    {
        _activeIndices.Clear();
        var vehicles = _vehicles;
        for (var i = 0; i < vehicles.Count; i++)
        {
            var v = vehicles[i];
            // P1F-2: exclude mid-teleport vehicles (InTransfer) -- inert (never set) unless
            // TimeToTeleport>0, so byte-identical for every pre-P1F scenario.
            if (v.Inserted && !v.Arrived && !v.InTransfer)
            {
                _activeIndices.Add(i);
            }
        }
    }

    // Perf (on-target measurement): opt-in for the parallel Export path (EmitTrajectory's
    // compute-into-scratch-then-append branch). OFF by default because on the 16-core/24-thread
    // target box it is a consistent NET LOSS -- measured serial emit ~421 ms vs parallel 588-755 ms
    // per city-3000 run at every thread count (the per-vehicle geometry is memory-light, so the
    // Parallel.For dispatch + scratch write/re-read cost dominates any compute win). Byte-identical
    // either way (the parallel path is provably equal to the serial append -- see _emitScratch's
    // header), so this only chooses the faster SCHEDULE; the deterministic output is untouched. Left
    // reachable (set true) so the parallel path can be re-measured on higher-bandwidth hardware.
    public bool ParallelExport;

    // Perf (opt-in FAST MODE -- CLAUDE.md iron law, the "fast-mode flag" escape hatch): OFF by
    // default, and when false the deterministic/byte-identical path is COMPLETELY untouched (the
    // committed goldens and the determinism hash 909605E965BFFE59 are unaffected). When true, the
    // engine is permitted to use faster, NOT-SUMO-byte-identical schedules for the order-dependent
    // serial phases (e.g. parallelizing insert/speed-gain/foe-index with a deterministic
    // lowest-EntityIndex tie-break instead of SUMO's exact processing order). Fast mode is still
    // fully DETERMINISTIC (thread-count-independent), just not trajectory-identical to SUMO; it is
    // validated BEHAVIORALLY, not byte-identically -- see Sim.BenchCity `--fast-gate`: 0 gridlock,
    // aggregate parity (arrived / mean duration / mean speed / trip-duration KS) within a tight
    // tolerance of the deterministic run, and no vehicle overlaps. No committed parity test or
    // scenario ever sets this, so every golden stays on the exact SUMO-faithful path.
    public bool FastMode;

    // Laneless direction (docs/LANELESS-DIRECTION.md): opt-in proof-of-concept for the continuous
    // footprint / velocity-obstacle (RVO-lite) lateral layer. When true AND the sublane model is
    // active (_sublane), the lateral intent comes from ComputeRvoLateral (continuous avoidance over
    // near-neighbours + external obstacles, treated uniformly) instead of the SUMO-faithful
    // ComputeSublaneLateral drift. This is the laneless axis's OWN model: SUMO's lateral PHYSICS as
    // the reference (minGapLat clearance, maxSpeedLat bound) but validated BEHAVIOURALLY (no-overlap,
    // overtake-completes, recenters) rather than byte-exact -- because SUMO's exact sublane timing is
    // an ECS-hostile persistent state machine reproducing a lane-anchored approximation (see
    // docs/PHASE2-SUBLANE.md). Default false, so every committed golden (incl. the exact sublane rungs
    // 60/61/62) stays on the SUMO-faithful path and is byte-identical; no committed test sets it.
    public bool LanelessRvo;

    // Cross-regime bridge (docs/LANELESS-DIRECTION.md, "the cross-regime bridge"): an optional
    // OPEN-SPACE crowd whose agents this engine's laneless-RVO vehicles avoid. When set (by the
    // CrossRegimeCoupling), ComputeRvoLateral queries it for crowd agents near ego's WORLD position
    // and projects each onto ego's lane as a one-sided RvoNeighbour -- so a lane vehicle swerves for a
    // pedestrian the same way it swerves for another vehicle, closing the "SUMO traffic respects
    // non-SUMO agents" loop. Null by default => the crowd loop in ComputeRvoLateral is skipped
    // entirely => byte-identical (and it is reachable ONLY under LanelessRvo && _sublane anyway, so no
    // committed golden can touch it). Deliberately a neutral world-disc source (Bridge.
    // ICrowdFootprintSource), NOT the string-keyed ExternalObstacle API the owner is replacing.
    public Sim.Core.Bridge.ICrowdFootprintSource? CrowdSource { get; set; }

    // Perf diagnostics: opt-in per-phase wall-time accounting for the Run loop. OFF by default and
    // effectively free then (one bool test per phase per step -- GetTimestamp is not even called, no
    // allocation, no Stopwatch object). Sim.BenchCity --profile turns it on and prints the breakdown,
    // so the parallelization effort targets the phases that actually dominate the serial fraction
    // rather than guessing. Never read by the engine itself -> zero behavioral effect, parity-inert.
    public bool ProfilePhases;
    private readonly Dictionary<string, long> _phaseTicks = new();
    public IReadOnlyDictionary<string, long> PhaseTicks => _phaseTicks;

    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.AggressiveInlining)]
    private long PhaseStart() => ProfilePhases ? System.Diagnostics.Stopwatch.GetTimestamp() : 0L;

    private void PhaseEnd(string name, long start)
    {
        if (!ProfilePhases)
        {
            return;
        }

        var elapsed = System.Diagnostics.Stopwatch.GetTimestamp() - start;
        _phaseTicks.TryGetValue(name, out var acc);
        _phaseTicks[name] = acc + elapsed;
    }

    // C6-ii: per-TLS stateful actuated phase machines, keyed by tlLogic id -- built once in
    // LoadScenario for every tlLogic whose Type == "actuated", empty for a network with only
    // 'static' programs (so RedLightConstraint's actuated branch is never entered and the static
    // path stays byte-identical). Reset at the start of each Run().
    private readonly Dictionary<string, ActuatedTrafficLightLogic> _actuatedLogics = new(StringComparer.Ordinal);

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
    // Perf (PERF-ROADMAP.md Layer 1): explicit override for the parallel plan phase. `null` (the
    // default) = AUTO: parallelize a step iff the scenario has at least ParallelPlanThreshold
    // vehicles (so tiny parity scenarios and small demos stay serial -- no Parallel.ForEach overhead
    // and a deterministic-timed test loop -- while large city runs parallelize automatically). Set
    // true/false to FORCE (Sim.Bench sets it explicitly for its single-vs-parallel comparison). The
    // getter reports the forced value (false when unset) for back-compat.
    private bool? _forceParallelPlan;
    public bool UseParallelPlan
    {
        get => _forceParallelPlan ?? false;
        set => _forceParallelPlan = value;
    }

    // Perf (core-scaling measurement): caps the degree of parallelism of the plan/willPass/emit
    // Parallel.For loops. Default -1 == unlimited (TPL's own default -- byte-identical to a
    // Parallel.For with no options), so this is inert unless a benchmark sets it. Set to N to pin the
    // engine to at most N worker threads, which is how Sim.BenchCity's --max-parallelism sweeps a
    // 1..coreCount scaling curve on one machine (results are order-independent, so the thread count
    // never changes the trajectory -- only the wall-clock). Cached as a ParallelOptions so the hot
    // loops allocate nothing per step.
    private System.Threading.Tasks.ParallelOptions _parallelOptions = new() { MaxDegreeOfParallelism = -1 };
    public int MaxParallelism
    {
        get => _parallelOptions.MaxDegreeOfParallelism;
        set => _parallelOptions = new System.Threading.Tasks.ParallelOptions
        {
            MaxDegreeOfParallelism = value > 0 ? value : -1,
        };
    }

    // Below this vehicle count a step's plan phase runs serially -- the Parallel.ForEach partition/
    // task overhead is not worth amortizing, and it keeps every committed parity scenario (all far
    // smaller) on the serial path. Post-L0 measurement: parallel already wins from ~150 concurrent
    // vehicles up, so this is a conservative floor. Gated on _vehicles.Count (total demand present),
    // a cheap O(1) proxy for "big scenario".
    private const int ParallelPlanThreshold = 256;

    // The per-step decision: an explicit force wins; otherwise auto by scenario size.
    private bool ShouldParallelizePlan()
        => _forceParallelPlan ?? (_vehicles.Count >= ParallelPlanThreshold);

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

    // F2 (probabilistic flow): a distinct salt so each <flow probability=> gets its own per-flow
    // insertion RNG stream, seeded off (Seed, flowIndex) and fully independent of every per-vehicle
    // stream (RngState / SpeedFactorRngSalt) -- ("Flo2Rng " packed big-endian; any distinct nonzero
    // salt would do).
    private const ulong ProbFlowRngSalt = 0x466C6F32526E6720UL;

    // P0-B (vTypeDistribution resolution, owner Q1b): a distinct salt so the once-at-creation
    // <vTypeDistribution> member draw (BuildRuntime/ResolveEffectiveTypeId) gets its OWN
    // per-entity RNG stream, seeded off (Seed, entityIndex) via VehicleRng.SeedFor's 3-arg
    // overload -- same independence argument as SpeedFactorRngSalt/ProbFlowRngSalt: never shares
    // state with RngState (the dawdle stream) or the SpeedFactor stream, so the draw is a pure,
    // deterministic function of (Seed, entityIndex, distribution) -- independent of thread/
    // scheduling order and of how many other vehicles exist. ("VTypeDis" packed big-endian; any
    // distinct nonzero salt would do.)
    private const ulong VTypeDistRngSalt = 0x5654797065446973UL;

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

    // P0-D (`--statistic-output`'s `<teleports total=.../>`): a running counter surfaced now so
    // a `StatisticWriter` has something real to read. Phase 1 runs with `time-to-teleport="-1"`
    // (CLAUDE.md "Determinism (phase 1)": teleport off), so this stays 0 for every existing
    // scenario/test -- P1-F (when teleport-on grid-lock handling lands) is the only future change
    // that will ever increment it, and it will do so from wherever it detects a teleport, not here.
    public int TeleportCount { get; private set; }

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1E, §4): the JAM sub-count of TeleportCount
    // (MSVehicleControl::registerTeleportJam). Issue-2: the yield/wrongLane sub-counts below are now
    // also produced, so total == jam + yield + wrongLane (matching SUMO's split at MSLane.cpp:2288-2294).
    // Every teleport increments TeleportCount (total) plus exactly one of the three buckets. 0 for every
    // pre-P1F scenario (teleport off).
    public int TeleportCountJam { get; private set; }

    // Issue-2 (docs/ISSUE2-JUNCTION-TELEPORT-DESIGN.md): the YIELD sub-count
    // (MSVehicleControl::registerTeleportYield) -- a stuck front vehicle whose next junction link is a
    // MINOR link (!havePriority) is waiting for a right-of-way foe, NOT jammed behind its own leader.
    // SUMO classifies these as yield (MSLane.cpp:2273,2290). 0 when teleport is off / no minor-link waits.
    public int TeleportCountYield { get; private set; }

    // Issue-2: the WRONG-LANE sub-count (MSVehicleControl::registerTeleportWrongLane) -- a stuck front
    // vehicle on a lane from which its route cannot continue (!appropriate). MSLane.cpp:2261,2288. 0 in
    // every in-scope scenario (the committed goldens + the synthetic-junction repro all report 0).
    public int TeleportCountWrongLane { get; private set; }

    // P2-H (HIGH-DENSITY-P2H-DESIGN.md): count of pending vehicles DELETED because they waited past
    // <max-depart-delay> without an insertion gap (SUMO's MSInsertionControl deleteVehicle(veh, true)).
    // Observability only -- no committed golden reads it (FCD sees the vehicle's absence directly).
    // Stays 0 for every scenario that does not set max-depart-delay (the eviction branch is gated off).
    private int _discardedDepartures;

    // P2-H: read-only accessor for the discarded-departure tally (see _discardedDepartures).
    public int DiscardedDepartureCount => _discardedDepartures;

    // X1 (docs/HIGH-DENSITY-X1-DESIGN.md): the attention-aware realism mask. The host publishes a fresh
    // immutable snapshot via SetVisibleEdges (volatile ref swap, lock-free); AdvanceOneStep captures it
    // ONCE into _activeMask so it cannot change mid-step. null = no camera = fully permissive = inert,
    // so every committed golden is byte-identical (the gates are all `mask is null || mask.MayX(...)`).
    private volatile RealismMask? _realismMask;
    private RealismMask? _activeMask;

    // X1: set the on-camera / high-realism edge set (where cheating is forbidden). Thread-safe: builds
    // an immutable RealismMask and publishes it with a single volatile assignment. Pass an empty set or
    // call ClearVisibleEdges() to return to fully-permissive. `forbidTeleport`/`forbidPop` choose which
    // cheating actions the visible zone forbids (both default true = strict no-cheating on camera).
    public void SetVisibleEdges(IReadOnlyCollection<string> visibleEdgeIds, bool forbidTeleport = true, bool forbidPop = true)
        => _realismMask = new RealismMask(visibleEdgeIds, forbidTeleport, forbidPop);

    // X1: clear the mask -> fully permissive (every edge off-camera). Inert-equivalent for the gates.
    public void ClearVisibleEdges() => _realismMask = null;

    // X1: aggressive OFF-CAMERA de-jam despawn. A frontmost jam blocker on an off-camera lane whose
    // WaitingTime exceeds this (seconds) is DESPAWNED before it would reach time-to-teleport, so
    // off-camera regions never build standing jams. <= 0 disables the action entirely (default) -> the
    // DejamDespawn phase returns immediately, byte-identical for every scenario. Runtime host property,
    // never a sumocfg key (X1 is a non-parity capability, not a scenario config surface).
    public double DejamDespawnTime { get; set; }

    // X1: pop-budget accounting -- cap on off-camera de-jam despawns performed per step (unlimited by
    // default). Bounds and makes measurable the invisible popping.
    public int DejamDespawnBudgetPerStep { get; set; } = int.MaxValue;

    // X1: running tally of off-camera de-jam despawns (observability / tests). 0 unless DejamDespawnTime
    // > 0 has actually fired. Reset per LoadScenario.
    private int _dejamDespawnCount;
    public int DejamDespawnCount => _dejamDespawnCount;

    // P2G-2 (docs/HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md): the dense lane-change model -- the faithful
    // aggressive multi-lane overtaking/merging the parity path leaves on the table (P2G-3 cross-junction
    // speed-gain). This is the PRODUCT DEFAULT in the runtime hosts: it flows the realistic organic
    // multi-lane net BETTER than parity (21 vs 24 stuck on city-organic-L2) with no perf penalty, and adds
    // believable overtaking. Default OFF here (the parity anchor) -> the P2G-3 path is skipped, so every
    // committed golden is byte-identical. Runtime host property, never a sumocfg key.
    public bool CoordinatedLaneChange { get; set; }

    // SUMOSHARP-API.md §4.4: resolve a lane's string id to the int lane handle ONCE at setup, so the
    // per-step obstacle path never touches a string. Requires a loaded scenario.
    public int GetLane(string laneId) =>
        (_network ?? throw new InvalidOperationException("LoadScenario must be called before GetLane."))
            .LaneHandleById[laneId];

    // ----- Dense edge handles (SUMOSHARP-API.md §9) -----
    // The router and edge model are string-keyed; these give a host a stable int "edge handle" it can
    // resolve ONCE at setup and pass to the int-based Spawn/route overloads, so its per-frame code holds
    // ints, not strings (matching GetLane's role for lanes). A handle is the edge's index in the network's
    // deterministic `Edges` list; the map is built lazily and rebuilt if a different network is loaded.
    private NetworkModel? _edgeMapFor;
    private Dictionary<string, int>? _edgeHandleById;
    private string[]? _edgeIdByHandle;

    private void EnsureEdgeMap()
    {
        var net = _network ?? throw new InvalidOperationException("LoadScenario/LoadNetwork must be called before GetEdge.");
        if (ReferenceEquals(_edgeMapFor, net) && _edgeHandleById is not null)
        {
            return;
        }

        var byId = new Dictionary<string, int>(net.Edges.Count, StringComparer.Ordinal);
        var byHandle = new string[net.Edges.Count];
        for (var i = 0; i < net.Edges.Count; i++)
        {
            var id = net.Edges[i].Id;
            byId[id] = i;      // last-wins on the (not-expected) duplicate id; Edges is 1:1 with EdgesById
            byHandle[i] = id;
        }

        _edgeHandleById = byId;
        _edgeIdByHandle = byHandle;
        _edgeMapFor = net;
    }

    // Number of edges (== the valid edge-handle range [0, EdgeCount)).
    public int EdgeCount
    {
        get { EnsureEdgeMap(); return _edgeIdByHandle!.Length; }
    }

    // Resolve an edge's string id to its dense int handle ONCE at setup. Throws for an unknown id.
    public int GetEdge(string edgeId)
    {
        EnsureEdgeMap();
        if (!_edgeHandleById!.TryGetValue(edgeId, out var handle))
        {
            throw new ArgumentException($"unknown edge id '{edgeId}'.", nameof(edgeId));
        }

        return handle;
    }

    // Reverse: the string id for a dense edge handle (for logging / interop with the string-keyed APIs).
    public string GetEdgeId(int edgeHandle)
    {
        EnsureEdgeMap();
        if (edgeHandle < 0 || edgeHandle >= _edgeIdByHandle!.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(edgeHandle), edgeHandle, "edge handle out of range.");
        }

        return _edgeIdByHandle[edgeHandle];
    }

    private string[] EdgeHandlesToIds(ReadOnlySpan<int> edgeHandles)
    {
        EnsureEdgeMap();
        var ids = new string[edgeHandles.Length];
        for (var i = 0; i < edgeHandles.Length; i++)
        {
            var h = edgeHandles[i];
            if (h < 0 || h >= _edgeIdByHandle!.Length)
            {
                throw new ArgumentOutOfRangeException(nameof(edgeHandles), h, "edge handle out of range.");
            }

            ids[i] = _edgeIdByHandle[h];
        }

        return ids;
    }

    // ----- Handle-based obstacle API (SUMOSHARP-API.md §4.4, the primary/shipped surface) -----
    // Zero-allocation: Add returns a generational ObstacleHandle; Update/Remove write columns by index.
    // `laneHandle` comes from GetLane(laneId). D17: avoidanceClass is the reserved RVO reciprocity class,
    // inert for the lane-based engine (default OneSided). B6: latPos/width (default 0/0 == pre-B6
    // full-lane block) give the obstacle a lateral footprint so a car can SWERVE around it.
    public ObstacleHandle AddObstacle(int laneHandle, double frontPos, double length,
        double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity,
        double latPos = 0.0, double width = 0.0, double latSpeed = 0.0,
        AvoidanceClass avoidanceClass = AvoidanceClass.OneSided)
        => AddCore(laneHandle, frontPos, length, startTime, endTime,
                   0.0, 0.0, latPos, width, latSpeed, avoidanceClass);

    // B5-i: MOVING obstacle -- AdvanceObstacles (Input phase) dead-reckons FrontPos by Speed*dt every
    // step, and ObstacleConstraint feeds Speed/MaxDecel into KraussModel.FollowSpeed.
    public ObstacleHandle AddMovingObstacle(int laneHandle, double frontPos, double length,
        double speed, double maxDecel,
        double startTime = double.NegativeInfinity, double endTime = double.PositiveInfinity,
        double latPos = 0.0, double width = 0.0, double latSpeed = 0.0,
        AvoidanceClass avoidanceClass = AvoidanceClass.OneSided)
        => AddCore(laneHandle, frontPos, length, startTime, endTime,
                   speed, maxDecel, latPos, width, latSpeed, avoidanceClass);

    // Per-step corrections from the external owner. Inert-when-absent: a stale/removed handle is a no-op.
    public void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed) =>
        _obstacles.Update(handle, frontPos, speed);

    public void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed, double latPos) =>
        _obstacles.Update(handle, frontPos, speed, latPos);

    public void UpdateObstacle(ObstacleHandle handle, double frontPos, double speed, double latPos, double latSpeed) =>
        _obstacles.Update(handle, frontPos, speed, latPos, latSpeed);

    public void RemoveObstacle(ObstacleHandle handle) => _obstacles.Remove(handle);

    // Resolve the lane handle -> LaneId string (which the lane-filtering consumers -- ObstacleConstraint,
    // the reroute/junction/follower scans -- still match on) and store. The obstacle's string Id is empty
    // (it was only ever the string-API key); ComputeLateralEvasion's tie-break among overlapping obstacles
    // therefore falls to insertion order -- deterministic, and no committed scenario has such an overlap.
    private ObstacleHandle AddCore(int laneHandle, double frontPos, double length,
        double startTime, double endTime, double speed, double maxDecel,
        double latPos, double width, double latSpeed, AvoidanceClass avoidanceClass)
    {
        var laneId = (_network ?? throw new InvalidOperationException("LoadScenario/LoadNetwork must be called before AddObstacle."))
            .LanesByHandle[laneHandle].Id;
        return _obstacles.Add(string.Empty, laneHandle, laneId, frontPos, length, startTime, endTime,
                              speed, maxDecel, latPos, width, latSpeed, avoidanceClass);
    }

    // Remove every obstacle at once.
    public void ClearObstacles() => _obstacles.Clear();

    // B5-i: dead-reckon every MOVING obstacle forward by Speed*dt (and B6-lat LatPos by LatSpeed*dt),
    // once per step, in the Input phase BEFORE PlanMovements/the neighbor-query Refill -- so the Plan
    // phase reads a FROZEN obstacle position for this step, exactly like every other piece of
    // start-of-step state (CLAUDE.md rule 2). The dead-reckoning logic now lives in ObstacleStore.Advance
    // (iterating the dense active list, writing columns in place -- no id-scratch snapshot needed since
    // an array has no enumerate-during-mutation hazard). Byte-identical to the pre-store version.
    private void AdvanceObstacles(double time, double dt) => _obstacles.Advance(time, dt);

    public void LoadScenario(string netXmlPath, string rouXmlPath, string sumocfgPath)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _lanesByHandle = _network.LanesByHandle as Lane[] ?? System.Linq.Enumerable.ToArray(_network.LanesByHandle);
        _demand = DemandParser.Parse(rouXmlPath);
        _config = ScenarioConfigParser.Parse(sumocfgPath);
        InitializeLoaded();
    }

    // P0-A: SUMO-faithful `sumo -c config.sumocfg` -- the cfg alone names its own net-file /
    // route-files / additional-files via <input>, so the caller does not pass them separately (in
    // contrast to the 3-arg LoadScenario above, which stays untouched for every existing
    // call-site). SUMO resolves every <input> path RELATIVE TO THE CFG'S DIRECTORY, not the
    // process CWD -- so a scenario dir can be run from anywhere.
    public void LoadScenario(string sumocfgPath)
    {
        _config = ScenarioConfigParser.Parse(sumocfgPath);
        if (_config.NetFile is null || _config.RouteFiles.Count == 0)
        {
            throw new InvalidDataException(
                $"'{sumocfgPath}' has no <input><net-file>/<route-files> -- use the 3-arg " +
                "LoadScenario(net, rou, cfg) overload for a cfg without an <input> section.");
        }

        var cfgDir = Path.GetDirectoryName(Path.GetFullPath(sumocfgPath)) ?? string.Empty;
        string Resolve(string relative) => Path.Combine(cfgDir, relative);

        _network = NetworkParser.Parse(Resolve(_config.NetFile));
        _lanesByHandle = _network.LanesByHandle as Lane[] ?? System.Linq.Enumerable.ToArray(_network.LanesByHandle);
        _demand = DemandParser.Parse(_config.RouteFiles.Select(Resolve).ToArray());

        // P0-C2: parse <parkingArea> declarations out of the additional-files into a small registry.
        // Every OTHER additional-file element is still tolerated/discarded (the pre-P0-C2 load-and-
        // discard contract); only a missing root element is an error, exactly as before. An
        // additional-file with no <parkingArea> yields an empty registry, so a scenario without
        // parkingAreas takes the byte-identical no-op resolution path below.
        var parkingAreas = new Dictionary<string, ParkingArea>(StringComparer.Ordinal);
        foreach (var additionalFile in _config.AdditionalFiles)
        {
            var path = Resolve(additionalFile);
            using var stream = File.OpenRead(path);
            var root = System.Xml.Linq.XDocument.Load(stream).Root
                ?? throw new InvalidDataException($"additional-file '{path}' has no root element.");

            foreach (var pa in AdditionalFileParser.ParseParkingAreas(root, LaneLengthById))
            {
                if (!parkingAreas.TryAdd(pa.Id, pa))
                {
                    throw new InvalidDataException($"duplicate <parkingArea id='{pa.Id}'>.");
                }
            }
        }

        // P0-C2: resolve every `<stop parkingArea="X"/>` to a concrete lane stop now that both the
        // demand (route-files) and the parkingArea registry (additional-files) are known. No-op --
        // returns the demand unchanged, byte-identical -- when nothing references a parkingArea.
        _demand = ResolveParkingAreaStops(_demand, parkingAreas);

        InitializeLoaded();
    }

    // P0-C2: lane-length lookup used only for a <parkingArea>'s endPos default (SUMO defaults endPos
    // to the lane's length when the attribute is absent). Throws a clear error for an unknown lane,
    // rather than a raw KeyNotFoundException.
    private double LaneLengthById(string laneId) =>
        _network!.LanesById.TryGetValue(laneId, out var lane)
            ? lane.Length
            : throw new InvalidDataException($"<parkingArea> references unknown lane '{laneId}'.");

    // P0-C2/GAP-3 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §3): rewrite every `<stop parkingArea="X"/>`
    // (carrying only a ParkingAreaId placeholder from DemandParser) into a concrete lane stop --
    // LaneId=pa.lane, StartPos=pa.startPos, EndPos=this occupant's OWN distinct lot position (see
    // ParkingArea.LotPosition). After this, departPos="stop" (Engine.cs's TryInsertOnLane) and
    // ProcessNextStop consume the stop exactly like a plain lane <stop>, unchanged -- StopDef.
    // ParkingAreaId itself is left untouched by the `with` below (never null'd out), so
    // StopRuntime.IsParking (set from it in BuildRuntime) still knows this is a parking stop after
    // resolution. Fast path: if no vehicle references a parkingArea, the demand is returned
    // untouched (byte-identical), so any scenario without parkingArea stops is unaffected.
    //
    // GAP-3 occupant assignment (static, load-time -- NO parkingAreaReroute/finite-dwell turnover,
    // per the owner's steer, docs/SERVE-PATH-PLAN.md §3a): every occupant of a given parkingArea
    // gets a DISTINCT lot index, assigned in two passes so the order matches the timeline real SUMO
    // would observe for the served-scenario shapes this engine supports (park-and-stay sinks +
    // departPos="stop" pull-out origins ONLY):
    //   (1) departPos="stop" origins are already parked at t=0 (MSLane::insertVehicle's STOP case)
    //       -- strictly before any moving vehicle can possibly have driven in and reached its own
    //       stop (that takes >= 1 simulated step) -- so they claim the lowest lot indices first, in
    //       vehicle-list (document) order.
    //   (2) every remaining `<stop parkingArea>` on a normally-inserted (moving) vehicle claims the
    //       NEXT lot indices, in vehicle-list order.
    // This is a faithful reproduction of MSParkingArea::computeLastFreePos's "lowest-index free lot"
    // rule for scenarios where no occupant vacates before a later occupant of the SAME area arrives
    // (the only shape the served scenarios exercise -- see scenario 67's own design note for how its
    // timing keeps this unambiguous). A scenario that raced these would need SUMO's full dynamic
    // reservation system, out of GAP-3's scope.
    private static DemandModel ResolveParkingAreaStops(
        DemandModel demand,
        IReadOnlyDictionary<string, ParkingArea> parkingAreas)
    {
        var hasParkingStop = false;
        foreach (var v in demand.Vehicles)
        {
            foreach (var s in v.Stops)
            {
                if (s.ParkingAreaId is not null)
                {
                    hasParkingStop = true;
                    break;
                }
            }

            if (hasParkingStop)
            {
                break;
            }
        }

        if (!hasParkingStop)
        {
            return demand;
        }

        // Pass 1: assign a distinct lot index to every (vehicleIndex, stopIndex) occupant of each
        // referenced parkingArea. Keyed by a value-tuple, not the StopDef itself (records don't have
        // reference identity, and two stops CAN be structurally equal before resolution).
        var nextLotIndexByPa = new Dictionary<string, int>(StringComparer.Ordinal);
        var lotIndexByOccupant = new Dictionary<(int VehicleIndex, int StopIndex), int>();

        // Pass 1a: departPos="stop" origins (Def.Stops[0] is always the one TryInsertOnLane reads
        // for a Stop-kind depart position -- see that method's own DepartPosSpec.Stop arm).
        for (var vi = 0; vi < demand.Vehicles.Count; vi++)
        {
            var v = demand.Vehicles[vi];
            if (v.DepartPos.Kind == DepartPosSpec.Stop && v.Stops.Count > 0 && v.Stops[0].ParkingAreaId is { } originPaId)
            {
                var lotIndex = nextLotIndexByPa.TryGetValue(originPaId, out var n) ? n : 0;
                lotIndexByOccupant[(vi, 0)] = lotIndex;
                nextLotIndexByPa[originPaId] = lotIndex + 1;
            }
        }

        // Pass 1b: every remaining parkingArea stop (moving vehicles that drive in and park).
        for (var vi = 0; vi < demand.Vehicles.Count; vi++)
        {
            var v = demand.Vehicles[vi];
            for (var si = 0; si < v.Stops.Count; si++)
            {
                var s = v.Stops[si];
                if (s.ParkingAreaId is not { } paId || lotIndexByOccupant.ContainsKey((vi, si)))
                {
                    continue;
                }

                var lotIndex = nextLotIndexByPa.TryGetValue(paId, out var n) ? n : 0;
                lotIndexByOccupant[(vi, si)] = lotIndex;
                nextLotIndexByPa[paId] = lotIndex + 1;
            }
        }

        // Pass 2: rewrite each vehicle's stops using its assigned lot index.
        var newVehicles = new List<VehicleDef>(demand.Vehicles.Count);
        for (var vi = 0; vi < demand.Vehicles.Count; vi++)
        {
            var v = demand.Vehicles[vi];
            var needsResolve = false;
            foreach (var s in v.Stops)
            {
                if (s.ParkingAreaId is not null)
                {
                    needsResolve = true;
                    break;
                }
            }

            if (!needsResolve)
            {
                newVehicles.Add(v);
                continue;
            }

            var resolvedStops = new List<StopDef>(v.Stops.Count);
            for (var si = 0; si < v.Stops.Count; si++)
            {
                var s = v.Stops[si];
                if (s.ParkingAreaId is null)
                {
                    resolvedStops.Add(s);
                    continue;
                }

                resolvedStops.Add(ResolveParkingAreaStop(s, parkingAreas, lotIndexByOccupant[(vi, si)]));
            }

            newVehicles.Add(v with { Stops = resolvedStops });
        }

        return demand with { Vehicles = newVehicles };
    }

    private static StopDef ResolveParkingAreaStop(StopDef stop, IReadOnlyDictionary<string, ParkingArea> parkingAreas, int lotIndex)
    {
        if (stop.ParkingAreaId is null)
        {
            return stop;
        }

        if (!parkingAreas.TryGetValue(stop.ParkingAreaId, out var pa))
        {
            throw new InvalidDataException(
                $"<stop parkingArea='{stop.ParkingAreaId}'> references a parkingArea that is not " +
                "declared in any additional-file.");
        }

        // LotPosition throws a clear error if pa.RoadsideCapacity < 1 or lotIndex is out of range
        // (out-of-scope SUMO branches -- see its own header comment).
        return stop with
        {
            LaneId = pa.LaneId,
            StartPos = pa.StartPos,
            EndPos = pa.LotPosition(lotIndex),
        };
    }

    // SUMOSHARP-API.md §9: load a network WITHOUT any demand -- the "start empty and spawn everything at
    // runtime" entry point (games, digital-twins). Optional sumocfg supplies the timeline/flags; absent,
    // a deterministic default is synthesized (Euler, teleport off, step 1s, sigma-neutral). The host then
    // DefineVType()s and SpawnVehicle()s. Equivalent to LoadScenario with an empty rou.xml.
    public void LoadNetwork(string netXmlPath, string? sumocfgPath = null)
        => LoadNetwork(netXmlPath, sumocfgPath is null ? DefaultNetworkConfig() : ScenarioConfigParser.Parse(sumocfgPath));

    // Demand-less network load with an explicit config -- lets a caller (e.g. a demo/tool) pick the
    // step length, lane-change duration, etc. without a .sumocfg file on disk (build one with
    // ScenarioConfigParser.ParseXml). Additive: every golden loads via LoadScenario / the string overload
    // above, so this path is parity-inert.
    public void LoadNetwork(string netXmlPath, ScenarioConfig config)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _lanesByHandle = _network.LanesByHandle as Lane[] ?? System.Linq.Enumerable.ToArray(_network.LanesByHandle);
        _demand = EmptyDemand();
        _config = config;
        InitializeLoaded();
    }

    private static DemandModel EmptyDemand() => new(
        Array.Empty<VType>(),
        new Dictionary<string, VType>(StringComparer.Ordinal),
        Array.Empty<Route>(),
        new Dictionary<string, Route>(StringComparer.Ordinal),
        Array.Empty<VehicleDef>());

    // Deterministic default for a demand-less network load: Begin 0, effectively-open End, 1s Euler steps,
    // teleport off, actionStepLength == stepLength (fusion-eligible), speeddev 0, seed matching Engine.Seed.
    private static ScenarioConfig DefaultNetworkConfig() =>
        new(Begin: 0.0, End: 1e9, StepLength: 1.0, Ballistic: false, TimeToTeleport: -1.0,
            ActionStepLength: 0.0, SpeedDev: 0.0, Seed: 42, LaneChangeDuration: 0.0);

    // (Re)seed the mutable vType/route registries from the just-loaded demand, plus SUMO's built-in
    // default vType. Cleared first so a re-load on the same Engine never inherits the prior scenario.
    private void SeedRegistries()
    {
        _vTypesById.Clear();
        _routesById.Clear();
        _vTypeIds.Clear();
        _runtimeRouteCounter = 0;
        _runtimeVehicleCounter = 0;

        foreach (var vt in _demand!.VTypes)
        {
            _vTypesById[vt.Id] = vt;
            _vTypeIds.Add(vt.Id);
        }

        // A default passenger vType so SpawnVehicle works without an explicit DefineVType. Only added if
        // the scenario did not already define one under that id.
        if (!_vTypesById.ContainsKey(DefaultVTypeId))
        {
            _vTypesById[DefaultVTypeId] = new VType(DefaultVTypeId, VClass: "passenger", Sigma: null);
            _vTypeIds.Add(DefaultVTypeId);
        }

        foreach (var rt in _demand.Routes)
        {
            _routesById[rt.Id] = rt;
        }
    }

    // Shared load initialization for LoadScenario / LoadNetwork. Everything from here down was formerly
    // inline in LoadScenario; _network/_demand/_config are already assigned by the caller.
    private void InitializeLoaded()
    {
        // Caller (LoadScenario / LoadNetwork) has assigned all three; assert it so the rest of this method
        // sees them as non-null (and to fail loudly if a future caller forgets).
        if (_network is null || _demand is null || _config is null)
        {
            throw new InvalidOperationException("InitializeLoaded requires _network, _demand, and _config to be set.");
        }

        // B3: the cached router is built from the network being replaced above -- invalidate it
        // here so UpdateReroutes lazily rebuilds against the NEW network the next time it is
        // actually needed (never eagerly, since most scenarios never reroute at all).
        _router = null;

        // P1E-4: (re)build the periodic edge-weight table for the NEW network iff this scenario's
        // device.rerouting is actually enabled (ReroutePeriod>0) -- null otherwise, so
        // UpdatePeriodicReroutes/UpdateRerouteEdgeWeights both short-circuit to nothing for every
        // pre-P1E-4 scenario. _lastAdaptationTime seeds to Begin (see its own field comment).
        _edgeWeights = _config.ReroutePeriod > 0.0
            ? new RerouteEdgeWeights(_network, _config.RerouteAdaptationSteps)
            : null;
        _lastAdaptationTime = _config.Begin;

        // D4: (re)build the reusable neighbor-query buckets for the newly loaded network's dense
        // handle space -- cold path (once per LoadScenario call), never per step.
        _neighborQuery = new LaneNeighborQuery(_network.LanesByHandle.Count);

        // Domain decomposition: assign lanes to spatial regions for the region-parallel plan (opt-in
        // RegionPlan). Cold path, once per load; inert unless RegionPlan is set.
        ComputeLaneRegions();

        // Perf (super-linear fix): size the per-step foe-approach index to the dense handle space and
        // mark internal ('':''-prefixed) lanes once -- BuildFoeApproachIndex only records those, since
        // FindFoeVehicle is only ever queried with a junction-interior lane handle.
        var laneCount = _network.LanesByHandle.Count;
        _isInternalLane = new bool[laneCount];
        _foeApproachFirst = new VehicleRuntime?[laneCount];
        _foeApproachSecond = new VehicleRuntime?[laneCount];
        for (var h = 0; h < laneCount; h++)
        {
            _isInternalLane[h] = _network.LanesByHandle[h].Id.StartsWith(':');
        }

        // C6-ii: (re)build the actuated phase machines for the newly loaded network. Only tlLogics
        // with type="actuated" get one; a static-only network leaves this empty (no behavior change
        // for any pre-C6 scenario). Their runtime state is (re)initialized here and again at the top
        // of Run() via Reset().
        _actuatedLogics.Clear();
        foreach (var tlLogic in _network.TlLogicsById.Values)
        {
            if (tlLogic.IsActuated)
            {
                _actuatedLogics[tlLogic.Id] = ActuatedTrafficLightLogic.Build(tlLogic, _network, _config.Begin);
            }
        }

        // R4 (rail signal): precompute each rail-signal-guarded lane's conflict-lane set once here
        // (cold path, per LoadScenario). Empty for any network without a rail_signal junction.
        BuildRailSignalInfo();

        // R5 (rail crossing): precompute each rail_crossing junction's controlled road lanes and
        // rail via-lanes once here. Empty for any network without a rail_crossing junction.
        BuildRailCrossingInfo();

        // SUMOSHARP-DEADRECKONING.md §5.2: precompute the set of TL-controlled lanes (cold), so the
        // Step()-only read projection can publish each one's current signal colour for rendering.
        BuildTlControlledLanes();

        _vehicles.Clear();
        // GAP-2: completed-trip records belong to the previous scenario's run -- a fresh
        // (re)load must start CompletedTrips empty, same discipline as every other per-scenario
        // list below.
        _completedTrips.Clear();
        // D3: side storage is keyed by EntityIndex (== _vehicles list index) -- clear it in
        // lockstep with _vehicles so a re-LoadScenario on the same Engine instance never leaves
        // stale entries keyed against the previous scenario's vehicles. The pool only ever grows
        // within one scenario's lifetime; a fresh scenario starts it clean too.
        _laneSeqPool.Clear();
        _laneSeqArrival.Clear();
        _stopsByEntity.Clear();
        _avoidedByEntity.Clear();
        _effectiveRouteIdByEntity.Clear(); // reroute-registered ids belong to the previous scenario
        _rerouteRouteCounter = 0;
        _freeEntitySlots.Clear(); // §9: recycled slots belong to the previous scenario's index space
        // P1F-2: the teleport transfer queue + counters belong to the previous scenario's run.
        _transferQueue.Clear();
        _jamFrontmost.Clear();
        _jamCandidates.Clear();
        TeleportCount = 0;
        TeleportCountJam = 0;
        TeleportCountYield = 0;
        TeleportCountWrongLane = 0;
        _discardedDepartures = 0; // P2-H: reset the max-depart-delay eviction tally per scenario
        _dejamDespawnCount = 0;   // X1: reset the off-camera de-jam despawn tally per scenario
        _laneSource = null; // §6.3: rebuild the render lane-source lazily for the new network
        _bestLanesCache.Clear(); // L0b: route/edge-keyed memo is scenario-specific
        _insertRouteSeqCache.Clear(); // insert route-resolution memo is scenario-specific
        // W1: a freshly (re)loaded scenario is a fresh timeline -- the next Run/WarmUp resets the
        // state machines and starts the clock at Begin.
        _elapsedSteps = 0;
        // §10: fresh timeline -> no pending lifecycle events, and the diff baseline is cleared so the new
        // scenario's vehicles emit their first Departed/Arrived transitions correctly.
        _eventCount = 0;
        Array.Clear(_prevLifecycle, 0, _prevLifecycle.Length);
        // Rung ER3: recompute the give-way master switch for this scenario (reset first so a
        // re-LoadScenario on the same Engine never inherits the previous demand's answer).
        _anyBluelight = false;
        _anyLcOpposite = false;
        // Phase 2 (sublane): the global sublane master switch, from the immutable config.
        _sublane = _config!.LateralResolution > 0.0;
        // Perf (willPass/plan fusion): reset + recompute the disqualifiers for this scenario (same
        // reset discipline as the give-way switch above). CreateRuntime OR's in the per-vType ones.
        _anyKraussDawdle = false;
        _anyIdmm = false;
        _actionStepFusionOk = _config.ActionStepLength <= 0.0
            || Math.Abs(_config.ActionStepLength - _config.StepLength) < 1e-12;

        // Seed the mutable vType/route registries from this scenario's demand (+ a default vType) BEFORE
        // CreateRuntime, which resolves each vehicle's vType/route through them. Identical contents to the
        // former direct _demand reads, so the loaded-scenario path is byte-identical.
        SeedRegistries();

        foreach (var def in _demand.Vehicles)
        {
            CreateRuntime(def);
        }

        // Perf (insert): pre-resolve every static vehicle's insertion lane sequence NOW, in parallel.
        // ResolveLaneSequenceHandlesWithArrival is a pure function of (route.Edges, DepartLaneIndex) and
        // the immutable network, so this is byte-identical to resolving lazily at insertion -- it just
        // moves ~72% of the per-step insert phase (the route resolution) to a one-time parallel load
        // pass. Insertion then hits the cache (no resolution in the hot loop). Flow-generated vehicles
        // (runtime routes) miss and resolve lazily as before.
        PrewarmInsertRouteCache();

        // F2 (probabilistic flow): allocate + seed each <flow probability=>'s own insertion RNG and
        // zero its arrival counter, ONCE per load (independent per-flow streams -- see the field
        // comments). Empty when there are no probability flows, so this is inert for every existing
        // scenario. Seeded from (Seed, flowIndex, ProbFlowRngSalt), so the stream never aliases any
        // per-vehicle RngState/speedFactor stream.
        _probFlowRng = new VehicleRng[_demand.ProbabilisticFlows.Count];
        _probFlowCounter = new int[_demand.ProbabilisticFlows.Count];
        for (var i = 0; i < _probFlowRng.Length; i++)
        {
            _probFlowRng[i] = VehicleRng.SeedFor(Seed, i, ProbFlowRngSalt);
        }
    }

    // Materializes one VehicleDef into a live VehicleRuntime and appends it to _vehicles, assigning
    // the next stable EntityIndex and seeding every once-at-creation per-vehicle field exactly as
    // SUMO builds a vehicle at its depart time. Factored out of LoadScenario so the F2 probabilistic
    // flow (GenerateProbabilisticFlows) creates a runtime at RUNTIME through the SAME path -- a
    // vehicle generated by a flow is indistinguishable from a hand-listed one (same RngState /
    // speedFactor seeding off its EntityIndex, same stop side-table, same master-switch updates).
    // Append a freshly built runtime (the golden/flow path -- LoadScenario demand + F2 probabilistic
    // flows). Byte-identical to the pre-recycling CreateRuntime: this path never despawns, so it never
    // recycles. Runtime SpawnVehicle uses AllocateRuntime instead (which may reuse a freed slot).
    private void CreateRuntime(VehicleDef def)
    {
        var entityIndex = _vehicles.Count;
        _vehicles.Add(BuildRuntime(def, entityIndex));
    }

    // Build one live VehicleRuntime for slot `entityIndex` (does NOT place it in _vehicles -- the caller
    // appends or overwrites). Seeds every once-at-creation field exactly as SUMO builds a vehicle at
    // depart time. Also (re)initialises the idx-keyed side tables for this slot, so a RECYCLED slot never
    // inherits the previous occupant's stops or reroute-avoid set; on a fresh slot the removes are no-ops,
    // keeping the append path byte-identical.
    // P0-B (vTypeDistribution resolution, owner Q1b -- HIGH-DENSITY-P0-DESIGN.md "P0-B"):
    // `typeId` is a vehicle/flow's raw type= string. If it already names a declared <vType>, it is
    // returned AS-IS -- this is the byte-identical fast path every pre-P0-B scenario (and every
    // plain, non-distribution type= after P0-B) takes, with no RNG draw at all. Only a name that is
    // NOT a known vType, but IS a known <vTypeDistribution> id, draws a concrete member.
    //
    // Determinism/parallel-safety: the draw uses a per-entity SALTED VehicleRng
    // (VehicleRng.SeedFor(Seed, entityIndex, VTypeDistRngSalt)) -- a fresh, throwaway RNG built
    // from ONLY (Seed, entityIndex), drawn from exactly once, then discarded (never stored on
    // VehicleRuntime, never advanced again) -- the SAME one-shot-local-RNG pattern the speedFactor
    // draw above already establishes. So the result is a pure function of (Seed, entityIndex,
    // distribution): independent of thread/scheduling order, of insertion order, and of how many
    // other vehicles exist or which other distributions they draw from (NEVER System.Random, NEVER
    // a shared/global stream -- CLAUDE.md's determinism rule).
    //
    // Sampling: standard cumulative-probability selection over `dist.Members` (already normalised
    // to sum to 1 by DemandParser) -- draw one uniform double in [0,1), walk the members in their
    // parsed order accreting a running sum, and return the first member whose cumulative bound
    // exceeds the draw. The final member is a floating-point safety-net fallback (normalised
    // weights sum to 1 exactly in exact arithmetic, but the last cumulative bound can land a hair
    // below 1.0 after repeated float addition) -- never an out-of-range throw.
    private string ResolveEffectiveTypeId(string typeId, int entityIndex)
    {
        if (_vTypesById.ContainsKey(typeId))
        {
            return typeId;
        }

        if (_demand!.VTypeDistributions.TryGetValue(typeId, out var dist))
        {
            var rng = VehicleRng.SeedFor(Seed, entityIndex, VTypeDistRngSalt);
            var draw = rng.NextDouble();
            var cumulative = 0.0;
            foreach (var member in dist.Members)
            {
                cumulative += member.Probability;
                if (draw < cumulative)
                {
                    return member.VTypeId;
                }
            }

            return dist.Members[^1].VTypeId;
        }

        // Neither a known vType nor a known distribution -- fall through unchanged so the caller's
        // direct `_vTypesById[...]` indexer throws the SAME KeyNotFoundException (and message) a
        // pre-P0-B unknown type= always threw, rather than a new/different error shape here.
        return typeId;
    }

    private VehicleRuntime BuildRuntime(VehicleDef def, int entityIndex)
    {
        // Clear any stale idx-keyed side state (only non-empty when recycling a freed slot).
        _stopsByEntity.Remove(entityIndex);
        _avoidedByEntity.Remove(entityIndex);

        // P0-B: resolve def.TypeId to a concrete vType id FIRST -- a plain vType id (every
        // pre-P0-B scenario, and the overwhelming majority even after P0-B) is returned unchanged,
        // so this indexer lookup is byte-identical to the pre-P0-B direct `_vTypesById[def.TypeId]`
        // call. Only a name that is NOT itself a known vType, but IS a known <vTypeDistribution>
        // id, takes the new per-entity weighted-draw path. See ResolveEffectiveTypeId's own
        // comment for the determinism argument.
        var rawVType = _vTypesById[ResolveEffectiveTypeId(def.TypeId, entityIndex)];
        // vType defaults resolver (CLAUDE.md rule 6: match vType/init first): only vClass
        // and any explicit overrides (e.g. rou.xml's sigma="0") come from the raw parse;
        // everything else is a resolved SUMO vClass default (VTypeDefaults.Resolve).
        var vType = VTypeDefaults.Resolve(rawVType);
        // Rung ER3: flip the give-way master switch on if any vehicle is a bluelight EV.
        _anyBluelight |= vType.HasBluelight;
        // Rung OV1: flip the opposite-overtake master switch on if any vType allows it.
        _anyLcOpposite |= vType.LcOpposite;
        // Perf (willPass/plan fusion): flip the disqualifiers -- a Krauss vType with sigma>0 (takes a
        // dawdle RNG draw the pre-pass only makes on a throwaway copy) or an IDMM vType (real-pass-only
        // LevelOfService advance) makes the pre-pass Intent unsafe to reuse. See FusionEligible.
        _anyKraussDawdle |= vType.CarFollowModel == "Krauss" && vType.Sigma > 0.0;
        _anyIdmm |= vType.CarFollowModel == "IDMM";
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
            mean: vType.SpeedFactor, dev: _config!.SpeedDev, min: 0.2, max: 2.0, rng: ref speedFactorRng);

        // P1E-4 (§1A, §0.5.1): device.rerouting equip + first periodic-reroute schedule, drawn
        // ONCE here exactly like speedFactor above. Entirely inert (equipped stays false,
        // nextRerouteTime stays +infinity) whenever ScenarioConfig.ReroutePeriod<=0 -- i.e. every
        // pre-P1E-4 scenario -- so this block never even draws from the RNG stream for them,
        // keeping every existing golden/hash byte-identical.
        var rerouteEquipped = false;
        var nextRerouteTime = double.PositiveInfinity;
        if (_config.ReroutePeriod > 0.0)
        {
            var equipRng = VehicleRng.SeedFor(Seed, entityIndex, RerouteEquipRngSalt);
            rerouteEquipped = equipRng.NextDouble() < _config.RerouteProbability;
            if (rerouteEquipped)
            {
                if (_config.RerouteJitter)
                {
                    // §0.5.1 gated production improvement: per-vehicle phase offset uniform in
                    // [0, period) instead of SUMO's lockstep depart+period -- OWN salted stream,
                    // never shared with the equip draw above.
                    var jitterRng = VehicleRng.SeedFor(Seed, entityIndex, RerouteJitterRngSalt);
                    var phase = jitterRng.NextDouble() * _config.ReroutePeriod;
                    nextRerouteTime = def.Depart + phase;
                }
                else
                {
                    // SUMO-faithful schedule (MSDevice_Routing.cpp:223-237): first periodic
                    // reroute fires at depart+period, no RNG phase.
                    nextRerouteTime = def.Depart + _config.ReroutePeriod;
                }
            }
        }

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
            // P1E-4: see VehicleRuntime.RerouteEquipped/NextRerouteTime/LastRoutingTime's own
            // comments. LastRoutingTime is left at its NegativeInfinity field default (never set
            // here) so it is always "stale" relative to any finite _lastAdaptationTime.
            RerouteEquipped = rerouteEquipped,
            NextRerouteTime = nextRerouteTime,
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
                    // GAP-3: StopDef.ParkingAreaId survives ResolveParkingAreaStop's `with` rewrite
                    // (only LaneId/StartPos/EndPos are overridden there), so this is non-null iff the
                    // ORIGINAL XML was `<stop parkingArea=...>` -- never true for a plain lane stop.
                    IsParking = stopDef.ParkingAreaId is not null,
                });
            }

            _stopsByEntity[entityIndex] = stops;
        }

        return runtime;
    }

    // Runtime SpawnVehicle allocation (SUMOSHARP-API.md §9): reuse a freed EntityIndex slot when one is
    // available (rebuilding _vehicles[idx] in place and resetting its lifecycle-diff baseline so the
    // recycled vehicle emits a fresh Departed), otherwise append. Returns the chosen EntityIndex. The
    // freed slot's generation was already bumped by Despawn, so the handle SpawnVehicle returns for a
    // recycled slot is distinct from the despawned one (stale handles stay rejected by the generation
    // check). Only ever reached from runtime spawns; the golden/flow path uses CreateRuntime (append-only).
    private int AllocateRuntime(VehicleDef def)
    {
        if (RecycleVehicleSlots && _freeEntitySlots.Count > 0)
        {
            var idx = _freeEntitySlots.Pop();
            _vehicles[idx] = BuildRuntime(def, idx);
            if (idx < _prevLifecycle.Length)
            {
                _prevLifecycle[idx] = 0; // §10: fresh diff baseline -> the recycled slot re-emits Departed
            }

            return idx;
        }

        var newIndex = _vehicles.Count;
        _vehicles.Add(BuildRuntime(def, newIndex));
        return newIndex;
    }

    // D3: front-of-queue lookup against the side table, returning the same "no stops" empty
    // fast path every call site already handled against v.Stops.Count == 0 -- absent from
    // _stopsByEntity is exactly that case (LoadScenario only populates entries that have >=1
    // stop).
    private Queue<StopRuntime>? GetStops(VehicleRuntime v) =>
        _stopsByEntity.TryGetValue(v.EntityIndex, out var stops) ? stops : null;

    public TrajectorySet Run(int steps)
    {
        var trajectory = new TrajectorySet();
        Advance(trajectory, steps);
        return trajectory;
    }

    // W1 (warm-start): advance the simulation `steps` steps WITHOUT emitting any trajectory,
    // leaving the engine in a fully populated, valid start-of-step state. A subsequent Run(n)
    // then continues seamlessly from that state -- the clock and every stateful machine
    // (per-vehicle RngState/accumulators, actuated-TLS, rail-crossing phases) carry over rather
    // than restart, because Advance only resets the timeline on a FRESH start (_elapsedSteps==0).
    // Deterministic by construction (the engine is deterministic), so `WarmUp(W); Run(N)` yields
    // exactly the tail of a single `Run(W+N)` -- the basis for starting from an "already-driving"
    // populated network (use a <flow> demand to fill it). Zero added allocation vs Run (same loop
    // body, Export phase skipped).
    public void WarmUp(int steps) => Advance(null, steps);

    // ----- Host-facing stepped read surface (SUMOSHARP-API.md §5, D6) -----
    // Per-step published projection of the live vehicles: a struct-of-arrays a host (game render loop,
    // training obs read, digital-twin) polls between steps. Empty until the first Step(); the spans are
    // valid until the NEXT Step() (they alias reused buffers). Populating them is a NEW path that Run()
    // deliberately does NOT take, so the parity/determinism suite is byte-identical and pays zero overhead.

    private readonly VehicleReadBuffer _readBuffer = new();

    // SUMOSHARP-DEADRECKONING.md §6.3: opt-in PRODUCTION render mode. Off by default (ParityTangent) -> the
    // published render floats (PosX/PosY/Angle) are byte-identical to before and to the FCD/parity Angle.
    // When set to ChordHeading / CornerCutCorrected, the read surface's DERIVED render floats carry the
    // SUMO chord heading (and, for CornerCutCorrected, the swept-path off-tracking bow) so a host that
    // renders locally can look right rather than SUMO-exact. Only the derived floats change; the
    // parity-exact lane-relative doubles (Pos/PosLat/LaneHandle) are untouched, and this whole path is
    // Step()-only (Run()/goldens never publish), so parity + the determinism hash are unaffected either way.
    public RenderRealism RenderMode { get; set; } = RenderRealism.ParityTangent;
    private NetworkLaneSource? _laneSource;
    private int[] _renderUpBuf = new int[8];
    private int[] _renderPreBuf = new int[8];

    // SUMOSHARP-DEADRECKONING.md §5.2: the static set of TL-controlled approach lanes and their current
    // signal-state chars (refreshed each Step in PublishReadState) -- so a renderer can show junction
    // signals (we don't otherwise publish TL state). Step()-only projection; inert for the parity path.
    // Empty for a net with no static/actuated road TL.
    private (int LaneHandle, string TlId, int LinkIndex)[] _tlControlledLanes = Array.Empty<(int, string, int)>();
    private int[] _tlLaneHandles = Array.Empty<int>();
    private byte[] _tlStates = Array.Empty<byte>();

    // Per-vehicle generation for VehicleHandle staleness, indexed by EntityIndex. Presently a constant 1
    // (no vehicle slot is recycled yet); grown lazily off the hot creation path. When runtime despawn
    // lands it is bumped per-slot so a handle held across a despawn goes stale (TryGetVehicle rejects it).
    private ushort[] _vehicleGeneration = Array.Empty<ushort>();

    // SUMOSHARP-API.md §10: the per-Step lifecycle event buffer + the previous-frame lifecycle per vehicle
    // it is diffed against. Populated only in PublishReadState (Step-only), so Run() stays zero-overhead.
    private SimEvent[] _events = Array.Empty<SimEvent>();
    private int _eventCount;
    private byte[] _prevLifecycle = Array.Empty<byte>();

    // Advance one simulation step (or `steps`) WITHOUT accumulating a TrajectorySet, then publish the read
    // snapshot -- the host loop primitive: `while (running) { engine.Step(); render(engine); }`. Reuses the
    // exact same per-step driver as Run/WarmUp (Advance), so the simulation is identical; the only addition
    // is the post-step read projection. For a game wanting a fresh snapshot every frame, call Step() singly.
    public void Step() => Step(1);

    public void Step(int steps)
    {
        Advance(null, steps);
        PublishReadState();
    }

    // Number of active vehicles in the current published snapshot (the span length).
    public int VehicleCount => _readBuffer.Count;

    // Steps advanced on the current timeline, and the current simulation clock (seconds).
    public int StepCount => _elapsedSteps;

    public double CurrentTime => _config is null ? 0.0 : _config.Begin + _elapsedSteps * _config.StepLength;

    // Columnar read spans -- structure-of-arrays, valid until the next Step(). Render-facing geometry is
    // float; parity-exact lane-relative values are double (D7). PosZ is present from day one (0 on 2-D
    // nets, real when geometry-3D lands, SUMOSHARP-API.md §6). Same index across every span == same vehicle.
    public ReadOnlySpan<VehicleHandle> VehicleHandles => _readBuffer.Handles.AsSpan(0, _readBuffer.Count);
    // DR2 (issue #3): per-vehicle dead-reckoning regime, aligned with VehicleHandles (same index == same
    // vehicle). The batched read the DR publisher iterates for 10k+ vehicles: LaneArc unless the vehicle is
    // mid-RVO/ORCA swerve this step (FreeKinematic) or effectively stopped (Stationary). Cast each byte to
    // `DrModel`. Populated only in the Step projection (off the golden path) -> hash unaffected.
    public ReadOnlySpan<byte> DrModels => _readBuffer.DrModel.AsSpan(0, _readBuffer.Count);
    // DR2 (issue #3): the mid-manoeuvre bit aligned with VehicleHandles -- true when the vehicle is
    // swerving/dodging this step (RVO/ORCA lateral, or a normal-mode obstacle/crowd/overtake/give-way
    // steer), so its lateral is a reactive manoeuvre rather than steady lane-following. The DR publisher
    // reads this to raise the vehicle's publish rate; the vehicle stays LaneArc regardless (see RegimeOf).
    public ReadOnlySpan<bool> Manoeuvring => _readBuffer.Manoeuvring.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> PosX => _readBuffer.PosX.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> PosY => _readBuffer.PosY.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> PosZ => _readBuffer.PosZ.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> Angle => _readBuffer.Angle.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> Speed => _readBuffer.SpeedF.AsSpan(0, _readBuffer.Count);
    // Body dimensions (metres) for sized rendering; from the vehicle's vType. Render-facing float.
    public ReadOnlySpan<float> VehicleLengths => _readBuffer.Length.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> VehicleWidths => _readBuffer.Width.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<int> LaneHandles => _readBuffer.LaneHandle.AsSpan(0, _readBuffer.Count);
    // DR lookahead: the next lane handle on each vehicle's route (-1 if none). Lets a dead-reckoning
    // client walk past the current lane's end during extrapolation (SUMOSHARP-DEADRECKONING.md §5.1/§6).
    public ReadOnlySpan<int> NextLaneHandles => _readBuffer.NextLane.AsSpan(0, _readBuffer.Count);
    // Previous lane handle on each vehicle's route (-1 if none). Lets a renderer walk the chord/off-track
    // back point past the current lane's start for a correct heading on curves (§6.2).
    public ReadOnlySpan<int> PrevLaneHandles => _readBuffer.PrevLane.AsSpan(0, _readBuffer.Count);
    // Flattened per-vehicle lane window [prev2,prev1,current,next1,next2,next3] (VehicleReadBuffer layout
    // constants), for multi-lane DR walks (forward + chord/off-track back). Length == VehicleCount * Stride.
    public ReadOnlySpan<int> LaneWindows => _readBuffer.LaneWindow.AsSpan(0, _readBuffer.Count * VehicleReadBuffer.LaneWindowSize);
    public static int LaneWindowStride => VehicleReadBuffer.LaneWindowSize;
    public static int LaneWindowCurrentIndex => VehicleReadBuffer.LaneWindowBack;
    // §5.2: TL-controlled approach lanes (static) and their current signal-state chars (refreshed each
    // Step), aligned index-for-index. For rendering junction signals; empty when the net has no road TL.
    public ReadOnlySpan<int> TlLaneHandles => _tlLaneHandles.AsSpan();
    public ReadOnlySpan<byte> TlStates => _tlStates.AsSpan();

    // P4-1 (docs/PEDESTRIAN-TASKS.md §P4-1): read-only projection of the LIVE signal char at an arbitrary
    // controlled link (tlId, linkIndex), evaluated at CurrentTime. TlStates above is aligned to the
    // vehicle APPROACH lanes BuildTlControlledLanes enumerates (non-internal lanes only), so it structurally
    // cannot cover a pedestrian crossing's gating link -- that link runs walkingarea -> crossing and BOTH
    // endpoints are internal (':'-prefixed), so it is filtered out of that scan. The underlying tlLogic
    // phase-state string (and, for an actuated program, the actuated CurrentState) nonetheless carries EVERY
    // link index, including the crossing links, so this accessor reaches them by (tlId, linkIndex) -- the
    // pair the pedestrian crossing gate already resolves from the net's <connection tl=.. linkIndex=..>.
    //
    // Parity: this is a PURE READ over state Step already produced. It calls the SAME private TlLinkStateChar
    // the frozen parity path uses (identical actuated/static split), evaluates at the Engine's own
    // CurrentTime, mutates nothing, and is never called from Step() -- so it cannot perturb any golden
    // trajectory or the parity hash. Returns false (state = '\0') for an unknown tlId or an out-of-range
    // linkIndex, so a caller built from a mismatched net degrades gracefully instead of throwing.
    public bool TryGetTlLinkState(string tlId, int linkIndex, out char state)
    {
        state = '\0';
        if (_network is null || linkIndex < 0 || !_network.TlLogicsById.TryGetValue(tlId, out var tlLogic))
        {
            return false;
        }

        // The program's link count is the width of its phase state strings (every phase, and an actuated
        // program's CurrentState, share it) -- guard linkIndex against it so TlLinkStateChar's own indexing
        // can never throw.
        var linkCount = tlLogic.Phases.Count > 0 ? tlLogic.Phases[0].State.Length : 0;
        if (linkIndex >= linkCount)
        {
            return false;
        }

        state = TlLinkStateChar(tlId, linkIndex, CurrentTime);
        return true;
    }
    public ReadOnlySpan<double> Pos => _readBuffer.Pos.AsSpan(0, _readBuffer.Count);
    // Longitudinal acceleration (m/s^2), parity-exact double -- the getAcceleration() analog. The key
    // ingredient for renderer-side dead reckoning (SUMOSHARP-API.md §5.1): pos' = pos + v*dt + 0.5*a*dt^2.
    public ReadOnlySpan<double> Acceleration => _readBuffer.AccelD.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<double> PosLat => _readBuffer.PosLat.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<string> VehicleIds => _readBuffer.VehicleId.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<string> VehicleTypes => _readBuffer.VehicleType.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<string> LaneIds => _readBuffer.LaneId.AsSpan(0, _readBuffer.Count);

    // Parity-exact double speed backing TryGetVehicle's VehicleState.Speed (the public columnar `Speed` is
    // render-float). Internal: SimulationSnapshot copies it so the async snapshot's double speed matches.
    internal ReadOnlySpan<double> SpeedExactColumn => _readBuffer.SpeedD.AsSpan(0, _readBuffer.Count);

    // Lifecycle events (Departed / Arrived / …) that occurred during the most recent Step(), valid until
    // the next Step(). Drained by the host each frame; empty until the first Step().
    public ReadOnlySpan<SimEvent> Events => _events.AsSpan(0, _eventCount);

    // Random access by handle (array-of-structures view). Inert-when-absent: false on a stale generation
    // or a vehicle not active in the current snapshot.
    public bool TryGetVehicle(VehicleHandle handle, out VehicleState state)
    {
        var idx = (int)handle.Index;
        if (idx >= 0 && idx < _vehicleGeneration.Length && _vehicleGeneration[idx] == handle.Generation
            && _readBuffer.TryGetSlot(idx, out var slot))
        {
            state = new VehicleState(
                handle, _readBuffer.EntityIndex[slot], _readBuffer.VehicleId[slot], _readBuffer.VehicleType[slot],
                _readBuffer.LaneHandle[slot], _readBuffer.LaneId[slot],
                _readBuffer.Pos[slot], _readBuffer.SpeedD[slot], _readBuffer.PosLat[slot],
                _readBuffer.PosX[slot], _readBuffer.PosY[slot], _readBuffer.PosZ[slot], _readBuffer.Angle[slot]);
            return true;
        }

        state = default;
        return false;
    }

    // DR2 (issue #3): the per-vehicle accessor form of the DrModels column -- the shape SUMOSHARP-API §16
    // names ("DrModel Engine.GetDrModel(VehicleHandle) returning FreeKinematic while swerving"). Same
    // regime the DrModels column carries; use the column for bulk (10k) publishing, this for random
    // access. Inert-when-absent: a stale generation or a vehicle not in the current snapshot -> Stationary
    // (nothing to extrapolate).
    public DrModel GetDrModel(VehicleHandle handle)
    {
        var idx = (int)handle.Index;
        if (idx >= 0 && idx < _vehicleGeneration.Length && _vehicleGeneration[idx] == handle.Generation
            && _readBuffer.TryGetSlot(idx, out var slot))
        {
            return (DrModel)_readBuffer.DrModel[slot];
        }

        return DrModel.Stationary;
    }

    // DR2 (issue #3): is this vehicle mid-manoeuvre (swerving/dodging) in the current published frame?
    // The DR publisher's adaptive-rate signal (per-handle form of the Manoeuvring column). Inert-when-
    // absent: a stale/absent handle -> false.
    public bool IsManoeuvring(VehicleHandle handle)
    {
        var idx = (int)handle.Index;
        if (idx >= 0 && idx < _vehicleGeneration.Length && _vehicleGeneration[idx] == handle.Generation
            && _readBuffer.TryGetSlot(idx, out var slot))
        {
            return _readBuffer.Manoeuvring[slot];
        }

        return false;
    }

    // Fill the read buffer from the current active vehicles, projecting each to (x, y, angle) with the SAME
    // LaneGeometry.PositionAtOffset call EmitTrajectory uses -- so the read columns match the FCD geometry
    // exactly. Reads only committed post-step state; mutates nothing in the simulation.
    private void PublishReadState()
    {
        EnsureVehicleGenerationCapacity(_vehicles.Count);
        _readBuffer.BeginFrame(_vehicles.Count);

        // Reused across vehicles (CA2014: never stackalloc inside the loop -- it would accumulate the
        // whole frame's stack and overflow at high vehicle counts).
        Span<int> laneWindow = stackalloc int[VehicleReadBuffer.LaneWindowSize];

        foreach (var v in ActiveVehicles())
        {
            var lane = _lanesByHandle[v.LaneHandle];
            var (x, y, angle) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, v.Kinematics.LatOffset);
            var handle = new VehicleHandle((uint)v.EntityIndex, _vehicleGeneration[v.EntityIndex]);

            // Geometry-3D (§6): interpolated lane elevation when the lane shape is 3-D; 0 on a 2-D net
            // (ShapeZ == null), byte-identical to before.
            var z = lane.ShapeZ is { } shapeZ
                ? LaneGeometry.ElevationAtOffset(lane.Shape, shapeZ, v.Kinematics.Pos)
                : 0.0;

            // §6.3 production render mode: override ONLY the derived render floats with the chord /
            // off-tracking pose. Default (ParityTangent) skips this entirely -> byte-identical.
            if (RenderMode != RenderRealism.ParityTangent)
            {
                var pose = ComputeRenderPose(v);
                x = pose.X; y = pose.Y; z = pose.Z; angle = pose.HeadingDeg;
            }

            // DR lookahead: the next lane on the route (-1 if this is the last), so a dead-reckoning client
            // can walk past the current lane's end. One handle covers the common single-boundary crossing.
            var nextLane = v.LaneSeqIndex + 1 < v.LaneSeqLen
                ? _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex + 1]
                : -1;
            // The previous lane on the route, so a renderer can walk the chord/off-track BACK point past
            // the current lane's start for a correct heading through curves.
            var prevLane = v.LaneSeqIndex - 1 >= 0
                ? _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex - 1]
                : -1;

            // Multi-lane window [prev2, prev1, current, next1, next2, next3] for multi-boundary DR walks.
            for (var k = 0; k < VehicleReadBuffer.LaneWindowSize; k++)
            {
                var seqIdx = v.LaneSeqIndex + (k - VehicleReadBuffer.LaneWindowBack);
                laneWindow[k] = seqIdx >= 0 && seqIdx < v.LaneSeqLen
                    ? _laneSeqPool[v.LaneSeqStart + seqIdx]
                    : -1;
            }

            // P0-B: the RESOLVED concrete vType id (v.VType.Id), not the raw v.Def.TypeId -- for a
            // plain (non-distribution) type= these are identical, so every pre-P0-B read is
            // byte-identical; for a <vTypeDistribution> id, this publishes the drawn MEMBER's id
            // (matching real SUMO, which exposes the concrete resolved vType, never the
            // distribution name, once a vehicle is built).
            _readBuffer.Add(handle, v.EntityIndex, v.Def.Id, v.VType.Id,
                v.LaneHandle, nextLane, prevLane, laneWindow, v.LaneId, v.Kinematics.Pos, v.Kinematics.Speed, v.Acceleration, v.Kinematics.LatOffset,
                (float)x, (float)y, (float)z, (float)angle, (float)v.VType.Length, (float)v.VType.Width,
                (byte)RegimeOf(v), v.LateralManoeuvre);
        }

        DetectLifecycleEvents();

        // §5.2: refresh the controlled-lane signal chars at the current clock (post-step display state).
        for (var i = 0; i < _tlControlledLanes.Length; i++)
        {
            var (_, tl, li) = _tlControlledLanes[i];
            _tlStates[i] = (byte)TlLinkStateChar(tl, li, CurrentTime);
        }
    }

    // SUMOSHARP-DEADRECKONING.md §6.3: the chord / off-tracking render pose for a vehicle at its CURRENT
    // state (dt=0), used only when RenderMode != ParityTangent. Builds the vehicle's upcoming + preceding
    // lane-handle paths from its lane sequence (reusable buffers; this is the opt-in Step-only path, never
    // the parity path) and delegates to the shared PoseResolver.
    private Pose ComputeRenderPose(VehicleRuntime v)
    {
        _laneSource ??= new NetworkLaneSource(_network!);

        var upLen = v.LaneSeqLen - v.LaneSeqIndex;
        if (upLen < 0) upLen = 0;
        var preLen = v.LaneSeqIndex;
        if (_renderUpBuf.Length < upLen) _renderUpBuf = new int[Math.Max(upLen, _renderUpBuf.Length * 2)];
        if (_renderPreBuf.Length < preLen) _renderPreBuf = new int[Math.Max(preLen, _renderPreBuf.Length * 2)];

        var baseIdx = v.LaneSeqStart + v.LaneSeqIndex;
        for (var i = 0; i < upLen; i++) _renderUpBuf[i] = _laneSeqPool[baseIdx + i];
        for (var i = 0; i < preLen; i++) _renderPreBuf[i] = _laneSeqPool[baseIdx - 1 - i]; // nearest-behind first

        var state = new DrState
        {
            Model = DrModel.LaneArc,
            LaneHandle = v.LaneHandle,
            Pos = v.Kinematics.Pos,
            PosLat = v.Kinematics.LatOffset,
            Length = v.VType.Length,
        };

        return PoseResolver.Resolve(
            _laneSource, state, _renderUpBuf.AsSpan(0, upLen), _renderPreBuf.AsSpan(0, preLen), 0.0, RenderMode);
    }

    // §5.2: enumerate the road-TL-controlled approach lanes once (cold, per LoadScenario). Rail-signal
    // links (tl set but no <tlLogic>) are excluded -- their state is computed elsewhere.
    private void BuildTlControlledLanes()
    {
        var list = new List<(int, string, int)>();
        foreach (var lane in _network!.LanesByHandle)
        {
            if (lane.Id.StartsWith(':'))
            {
                continue; // internal lanes are not TL approach lanes
            }

            if (_network.TryGetTlControlledConnection(lane.EdgeId, lane.Index, out var conn)
                && conn.Tl is { } tl && _network.TlLogicsById.ContainsKey(tl)
                && conn.LinkIndex is { } li && li >= 0)
            {
                list.Add((lane.Handle, tl, li));
            }
        }

        _tlControlledLanes = list.ToArray();
        _tlLaneHandles = new int[_tlControlledLanes.Length];
        for (var i = 0; i < _tlControlledLanes.Length; i++)
        {
            _tlLaneHandles[i] = _tlControlledLanes[i].LaneHandle;
        }

        _tlStates = new byte[_tlControlledLanes.Length];
    }

    // Current signal char (G/g/y/r/...) for a controlled link at `evalTime`. Mirrors RedLightConstraint's
    // actuated/static split (kept separate so the frozen parity path is untouched).
    private char TlLinkStateChar(string tlId, int linkIndex, double evalTime)
    {
        var tlLogic = _network!.TlLogicsById[tlId];
        return tlLogic.IsActuated && _actuatedLogics.TryGetValue(tlId, out var actuated)
            ? actuated.CurrentState[linkIndex]
            : TrafficLightState.GetLinkState(tlLogic, linkIndex, evalTime);
    }

    // SUMO MSLink::havePriority(): a TL-controlled link has right-of-way iff its LIVE signal state
    // char is a major green 'G' (uppercase == priority, the SAME uppercase/minor split
    // ClassifyTeleportKind already uses). SUMO gates its whole minor-link yield family
    // (couldBrakeForMinor / blockedByFoe / the sameTarget arrival-time yield) on `!havePriority()`;
    // the engine's priority-junction arms instead infer "minor" from the STATIC netconvert <request>
    // response matrix, which is correct for an UNSIGNALIZED junction but TL-BLIND -- so a protected-
    // green vehicle would wrongly yield to a junction foe and freeze at the stop line until
    // time-to-teleport (docs/SUMOSHARP-LOWDENSITY-TELEPORT-DESIGN.md mechanism A). This restores the
    // signal's authority: on a major green the vehicle yields to no one. Sampled at time+dt to agree
    // with RedLightConstraint's own timing (see its comment). Returns false for an uncontrolled link
    // (Tl null) and for a rail signal (tl= present but no <tlLogic>) -> the static-matrix behaviour is
    // unchanged there, byte-identical for every non-TL committed golden.
    private bool EgoLinkHasSignalPriority(JunctionLink egoLink, double evalTime)
    {
        var conn = egoLink.Connection;
        if (conn.Tl is not { } tl || conn.LinkIndex is null || !_network!.TlLogicsById.ContainsKey(tl))
        {
            return false;
        }

        var state = TlLinkStateChar(tl, conn.LinkIndex.Value, evalTime);
        return state is >= 'A' and <= 'Z';
    }

    // DR2 (issue #3, re-scoped per the NuGet reply): a lane vehicle's dead-reckoning regime for the
    // current published frame. A VEHICLE is NEVER FreeKinematic -- even mid-swerve it stays LaneArc,
    // because LaneArc extrapolates `pos` along the (possibly curved) lane polyline whereas FreeKinematic
    // would extrapolate a straight world line and drift a swerving car off the road between updates. The
    // mid-manoeuvre state is surfaced SEPARATELY as the `Manoeuvring` bit (the DR publisher reads it to
    // raise that vehicle's publish rate, not to change its extrapolator). FreeKinematic comes only from
    // the crowd source (OrcaCrowd via ICrowdFootprintSource/WorldDisc), never from a vehicle. So a vehicle
    // is Stationary when effectively stopped, else LaneArc.
    private DrModel RegimeOf(VehicleRuntime v) =>
        v.Kinematics.Speed <= DrStationarySpeed ? DrModel.Stationary : DrModel.LaneArc;

    // Below this speed (m/s) a vehicle is classified Stationary for dead-reckoning (position only, no
    // extrapolation this frame). A render/DR-only threshold; never touches the parity path.
    private const double DrStationarySpeed = 0.01;

    // §10: diff each vehicle's lifecycle against the previous published frame and record Departed
    // (Pending -> Active) / Arrived (-> Arrived) events. Iterates ALL vehicles (not just active) so an
    // arrival -- which drops the vehicle from ActiveVehicles -- is still caught. Called only from
    // PublishReadState (Step path), so Run() never pays for it.
    private void DetectLifecycleEvents()
    {
        EnsurePrevLifecycleCapacity(_vehicles.Count);
        _eventCount = 0;

        for (var idx = 0; idx < _vehicles.Count; idx++)
        {
            var v = _vehicles[idx];
            // Encoded as VehicleLifecycle: Pending=1, Active=2, Arrived=3.
            var cur = (byte)(v.Arrived ? 3 : v.Inserted ? 2 : 1);
            if (cur == _prevLifecycle[idx])
            {
                continue;
            }

            if (cur == 2)
            {
                RecordEvent(idx, SimEventKind.Departed);
            }
            else if (cur == 3)
            {
                RecordEvent(idx, SimEventKind.Arrived);
            }

            _prevLifecycle[idx] = cur;
        }
    }

    private void RecordEvent(int entityIndex, SimEventKind kind)
    {
        if (_eventCount >= _events.Length)
        {
            Array.Resize(ref _events, _events.Length == 0 ? 8 : _events.Length * 2);
        }

        _events[_eventCount++] = new SimEvent(
            new VehicleHandle((uint)entityIndex, _vehicleGeneration[entityIndex]), kind);
    }

    private void EnsurePrevLifecycleCapacity(int needed)
    {
        if (_prevLifecycle.Length < needed)
        {
            Array.Resize(ref _prevLifecycle, Math.Max(needed, _prevLifecycle.Length == 0 ? 16 : _prevLifecycle.Length * 2));
        }
    }

    private void EnsureVehicleGenerationCapacity(int needed)
    {
        if (_vehicleGeneration.Length >= needed)
        {
            return;
        }

        var old = _vehicleGeneration.Length;
        var newCap = Math.Max(needed, old == 0 ? 16 : old * 2);
        Array.Resize(ref _vehicleGeneration, newCap);
        for (var i = old; i < newCap; i++)
        {
            _vehicleGeneration[i] = 1;  // live generation starts at 1 so a default VehicleHandle never resolves
        }
    }

    // ----- Runtime demand: vType definition, spawn, reroute, despawn (SUMOSHARP-API.md §9) -----

    // Register (or replace) a vehicle type at runtime. Resolves through the SAME VTypeDefaults pipeline as
    // loaded vTypes. `id` is optional (auto-generated when omitted). Requires a loaded network.
    public VTypeHandle DefineVType(VTypeParams p, string? id = null)
    {
        if (_network is null)
        {
            throw new InvalidOperationException("LoadScenario/LoadNetwork must be called before DefineVType.");
        }

        var typeId = id ?? $"__vtype{_vTypeIds.Count}";
        var raw = new VType(
            typeId, VClass: p.VClass, Sigma: p.Sigma, MaxSpeed: p.MaxSpeed, Accel: p.Accel, Decel: p.Decel,
            Tau: p.Tau, MinGap: p.MinGap, Length: p.Length, EmergencyDecel: p.EmergencyDecel,
            SpeedFactor: p.SpeedFactor, HasBluelight: p.HasBluelight, LcOpposite: p.LcOpposite,
            CarFollowModel: p.CarFollowModel,
            MaxSpeedLat: p.MaxSpeedLat, LatAlignment: p.LatAlignment, MinGapLat: p.MinGapLat);

        var existing = _vTypeIds.IndexOf(typeId);
        _vTypesById[typeId] = raw;
        if (existing >= 0)
        {
            return new VTypeHandle(existing);
        }

        _vTypeIds.Add(typeId);
        return new VTypeHandle(_vTypeIds.Count - 1);
    }

    // The auto-registered SUMO default passenger type (available after any load).
    public VTypeHandle DefaultVType => new(_vTypeIds.IndexOf(DefaultVTypeId));

    public bool TryGetVType(string id, out VTypeHandle handle)
    {
        var idx = _vTypeIds.IndexOf(id);
        handle = new VTypeHandle(idx);
        return idx >= 0;
    }

    // Spawn a vehicle at runtime on an explicit edge-id route. Returns a VehicleHandle immediately in the
    // Pending state; SUMO-parity queued insertion (InsertDepartingVehicles) places it on the road at the
    // next Step() when a safe gap exists at `departPos`. Poll GetLifecycle for Pending -> Active.
    public VehicleHandle SpawnVehicle(VTypeHandle type, IReadOnlyList<string> routeEdges,
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0, bool departBestLane = false)
    {
        if (_network is null || _config is null)
        {
            throw new InvalidOperationException("LoadScenario/LoadNetwork must be called before SpawnVehicle.");
        }

        if (!type.IsValid || type.Index >= _vTypeIds.Count)
        {
            throw new ArgumentException("invalid VTypeHandle (call DefineVType or use DefaultVType).", nameof(type));
        }

        if (routeEdges is null || routeEdges.Count == 0)
        {
            throw new ArgumentException("route must contain at least one edge.", nameof(routeEdges));
        }

        var routeId = $"__route{_runtimeRouteCounter++}";
        _routesById[routeId] = new Route(routeId, new List<string>(routeEdges));

        var def = new VehicleDef(
            Id: $"__veh{_runtimeVehicleCounter++}",
            TypeId: _vTypeIds[type.Index],
            RouteId: routeId,
            Depart: CurrentTime,
            // P0-C1: this runtime-spawn API only ever takes numeric literals (no symbolic string
            // surface), so every call is Given -- byte-identical to before the (Kind, Literal) spec
            // types existed.
            DepartPos: DepartPosValue.Given(departPos),
            DepartSpeed: DepartSpeedValue.Given(departSpeed),
            // departBestLane -> SUMO's departLane="best" (ResolveBestDepartLane): the vehicle enters on
            // the lane that best continues its route, so it isn't forced to cross lanes near a junction.
            // Default false keeps the historical Given(departLane) -> byte-identical for every caller.
            DepartLaneIndex: departBestLane ? DepartLaneValue.Best : DepartLaneValue.Given(departLane));

        // §9: reuse a freed slot when available, else append. Capacity for the (possibly new) slot is
        // ensured afterwards; a recycled slot's generation is already the post-Despawn bumped value.
        var entityIndex = AllocateRuntime(def);
        EnsureVehicleGenerationCapacity(_vehicles.Count);
        return new VehicleHandle((uint)entityIndex, _vehicleGeneration[entityIndex]);
    }

    // Spawn a vehicle routed from `fromEdge` to `toEdge` via the engine's shortest-path router. Throws if
    // no route exists (mirrors SUMO refusing an unroutable vehicle).
    public VehicleHandle SpawnVehicle(VTypeHandle type, string fromEdge, string toEdge,
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0, bool departBestLane = false)
    {
        var edges = Router().Route(fromEdge, toEdge)
            ?? throw new InvalidOperationException($"no route from edge '{fromEdge}' to '{toEdge}'.");
        return SpawnVehicle(type, edges, departPos, departSpeed, departLane, departBestLane);
    }

    // Dense-edge-handle overloads (SUMOSHARP-API.md §9): identical semantics to the string overloads,
    // taking the int handles from GetEdge so a host's per-frame code never touches edge-id strings.
    public VehicleHandle SpawnVehicle(VTypeHandle type, ReadOnlySpan<int> routeEdges,
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0) =>
        SpawnVehicle(type, EdgeHandlesToIds(routeEdges), departPos, departSpeed, departLane);

    public VehicleHandle SpawnVehicle(VTypeHandle type, int fromEdge, int toEdge,
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0) =>
        SpawnVehicle(type, GetEdgeId(fromEdge), GetEdgeId(toEdge), departPos, departSpeed, departLane);

    // Lifecycle of a spawned/loaded vehicle (Pending/Active/Arrived), or Unknown for a stale handle.
    public VehicleLifecycle GetLifecycle(VehicleHandle handle)
    {
        var idx = (int)handle.Index;
        if (idx < 0 || idx >= _vehicles.Count || idx >= _vehicleGeneration.Length
            || _vehicleGeneration[idx] != handle.Generation)
        {
            return VehicleLifecycle.Unknown;
        }

        var v = _vehicles[idx];
        if (v.Arrived)
        {
            return VehicleLifecycle.Arrived;
        }

        return v.Inserted ? VehicleLifecycle.Active : VehicleLifecycle.Pending;
    }

    // Remove a vehicle from the simulation (Active or still-Pending). Marks it arrived so it drops out of
    // every active scan, and bumps its generation so the handle goes stale. No-op (false) on a stale handle
    // or an already-arrived vehicle. The freed EntityIndex slot is offered for recycling by the next
    // runtime SpawnVehicle (§9, when RecycleVehicleSlots is on).
    public bool Despawn(VehicleHandle handle)
    {
        var idx = (int)handle.Index;
        if (idx < 0 || idx >= _vehicles.Count || idx >= _vehicleGeneration.Length
            || _vehicleGeneration[idx] != handle.Generation)
        {
            return false;
        }

        var v = _vehicles[idx];
        if (v.Arrived)
        {
            return false;
        }

        v.Inserted = true;   // so InsertDepartingVehicles never (re)considers a despawned-while-Pending vehicle
        v.Arrived = true;    // drops it from ActiveVehicles / the read snapshot
        _vehicleGeneration[idx] = unchecked((ushort)(_vehicleGeneration[idx] + 1));
        if (RecycleVehicleSlots)
        {
            _freeEntitySlots.Push(idx); // §9: offer the slot to the next runtime SpawnVehicle
        }

        return true;
    }

    // Re-route an ACTIVE vehicle to a new destination edge, keeping it physically where it is. Returns false
    // if the handle is stale/not active, or no route exists from the vehicle's current edge (e.g. it is
    // mid-junction on an internal lane -- retry next step).
    public bool SetDestination(VehicleHandle handle, string toEdge)
    {
        if (!TryResolveActive(handle, out var v))
        {
            return false;
        }

        var currentEdge = _network!.LanesByHandle[v.LaneHandle].EdgeId;
        var edges = Router().Route(currentEdge, toEdge);
        if (edges is null)
        {
            return false;
        }

        RerouteActive(v, edges);
        return true;
    }

    // Dense-edge-handle overloads (SUMOSHARP-API.md §9) of the destination/reroute API.
    public bool SetDestination(VehicleHandle handle, int toEdge) =>
        SetDestination(handle, GetEdgeId(toEdge));

    public bool Reroute(VehicleHandle handle, ReadOnlySpan<int> avoidEdges) =>
        Reroute(handle, EdgeHandlesToIds(avoidEdges));

    // Re-route an ACTIVE vehicle to its EXISTING destination while avoiding `avoidEdges`. Returns false as
    // for SetDestination, or if no alternate route exists.
    public bool Reroute(VehicleHandle handle, IReadOnlyList<string> avoidEdges)
    {
        if (!TryResolveActive(handle, out var v))
        {
            return false;
        }

        var currentEdge = _network!.LanesByHandle[v.LaneHandle].EdgeId;
        var destEdge = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + v.LaneSeqLen - 1]].EdgeId;
        var avoid = new HashSet<string>(avoidEdges, StringComparer.Ordinal);
        var edges = Router().Route(currentEdge, destEdge, avoid);
        if (edges is null)
        {
            return false;
        }

        RerouteActive(v, edges);
        return true;
    }

    // PANIC-EVAC.md R2: override the individual SUMO driver knobs of an ACTIVE vehicle at runtime.
    // Merges the non-null fields of `ov` onto the vehicle's resolved vType (a `with`-copy) and its
    // SpeedFactor; unset fields keep their current value, so knobs are independently settable and
    // "flee mode" is just this call with an aggressive preset. Returns false if the handle is stale
    // or the vehicle is not active. Purely additive -- no golden/parity scenario ever calls it, so
    // the determinism hash is unaffected (VehicleRuntime.VType's `set` header carries the argument).
    public bool SetVehicleParams(VehicleHandle handle, VehicleParamOverride ov)
    {
        if (!TryResolveActive(handle, out var v))
        {
            return false;
        }

        if (ov.SpeedFactor is double sf)
        {
            v.SpeedFactor = sf;
        }

        var t = v.VType;
        var newDecel = ov.Decel ?? t.Decel;
        // EmergencyDecel must never sit below Decel (SUMO invariant): honour an explicit override,
        // else lift it to the raised decel when needed, otherwise leave it be.
        var newEmergency = ov.EmergencyDecel ?? Math.Max(t.EmergencyDecel, newDecel);
        v.VType = t with
        {
            MaxSpeed = ov.MaxSpeed ?? t.MaxSpeed,
            Tau = ov.Tau ?? t.Tau,
            MinGap = ov.MinGap ?? t.MinGap,
            Accel = ov.Accel ?? t.Accel,
            Decel = newDecel,
            EmergencyDecel = newEmergency,
            // ApparentDecel tracks Decel unless the caller changed neither (SUMO defaults apparent
            // to decel); only move it when Decel actually changed.
            ApparentDecel = ov.Decel is null ? t.ApparentDecel : newDecel,
            JmIgnoreFoeProb = ov.JmIgnoreFoeProb ?? t.JmIgnoreFoeProb,
            JmIgnoreFoeSpeed = ov.JmIgnoreFoeSpeed ?? t.JmIgnoreFoeSpeed,
            JmIgnoreJunctionFoeProb = ov.JmIgnoreJunctionFoeProb ?? t.JmIgnoreJunctionFoeProb,
        };
        return true;
    }

    private NetworkRouter Router() => _router ??= new NetworkRouter(_network!);

    private bool TryResolveActive(VehicleHandle handle, out VehicleRuntime v)
    {
        var idx = (int)handle.Index;
        if (idx >= 0 && idx < _vehicles.Count && idx < _vehicleGeneration.Length
            && _vehicleGeneration[idx] == handle.Generation)
        {
            v = _vehicles[idx];
            // P1F-2: a mid-teleport vehicle is not resolvable as an active vehicle (inert unless
            // TimeToTeleport>0).
            if (v.Inserted && !v.Arrived && !v.InTransfer)
            {
                return true;
            }
        }

        v = null!;
        return false;
    }

    // SUMOSHARP-API.md §5.1 (dead-reckoning lookahead): copy the vehicle's upcoming lane handles -- the
    // CURRENT lane first, then the lanes it will traverse next -- into `dest`, returning the count written
    // (<= dest.Length), or 0 for a stale / not-yet-active handle. A renderer resolves the static lane
    // geometry once (per lane handle) and walks this path to dead-reckon a lane-bound vehicle along its
    // actual curve: integrate pos' = pos + speed*dt + 0.5*accel*dt^2 (from the read columns) and step into
    // the next lane when pos' passes the current lane length. Purely a read over committed route state.
    public int GetUpcomingLanes(VehicleHandle handle, Span<int> dest)
    {
        if (dest.IsEmpty || !TryResolveActive(handle, out var v))
        {
            return 0;
        }

        var from = v.LaneSeqStart + v.LaneSeqIndex;
        var end = v.LaneSeqStart + v.LaneSeqLen;
        var n = Math.Min(dest.Length, end - from);
        for (var i = 0; i < n; i++)
        {
            dest[i] = _laneSeqPool[from + i];
        }

        return n;
    }

    // Apply a new remaining route to an active vehicle -- mirrors UpdateReroutes' reassignment exactly:
    // newEdges[0] is the vehicle's current edge, so it stays physically where it is (Kinematics untouched),
    // re-pointed at the freshly resolved lane sequence from here on. Applied immediately via the command
    // buffer (this runs between steps, so the buffer is empty and Flush takes effect at once).
    private void RerouteActive(VehicleRuntime v, IReadOnlyList<string> newEdges)
    {
        RegisterRerouted(v, newEdges);   // keep _routesById/BestLanes reads (see field header) in sync

        var laneIndex = _network!.LanesByHandle[v.LaneHandle].Index;
        // Issue 1 cross-edge fix: keep the reroute consistent with insertion -- if the NEW remaining
        // route still ends at a qualifying park-and-stay stop, the re-resolved pool must still target
        // its lane (see ParkStopFinalEdgeOverride). null (the common case) is byte-identical to before.
        var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, newEdges);
        var (newPoolSeq, newArrivalSeq) = _network.ResolveLaneSequenceHandlesWithArrival(newEdges, laneIndex, stopOverride: stopOverride);
        var newLaneSeqStart = _laneSeqPool.Count;
        _laneSeqPool.AddRange(newPoolSeq);
        _laneSeqArrival.AddRange(newArrivalSeq);
        _commandBuffer.ReplaceRoute(v, newLaneSeqStart, newPoolSeq.Length);
        _commandBuffer.Flush();
    }

    // Shared driver for Run/WarmUp. `trajectory==null` => warm-up (the per-step Export is skipped).
    // Resets the timeline state machines only on a fresh start so a warm-up + run is one continuous
    // timeline; for a fresh engine (_elapsedSteps==0, the case every existing Run() call is) this is
    // byte-identical to the pre-W1 Run(): reset, then step from _config.Begin.
    private void Advance(TrajectorySet? trajectory, int steps)
    {
        if (_network is null || _demand is null || _config is null)
        {
            throw new InvalidOperationException("LoadScenario must be called before Run/WarmUp.");
        }

        var dt = _config.StepLength;
        if (_elapsedSteps == 0)
        {
            ResetTimelineStateMachines();
        }

        for (var i = 0; i < steps; i++)
        {
            var time = _config.Begin + _elapsedSteps * dt;
            AdvanceOneStep(trajectory, time, dt);
            _elapsedSteps++;
        }
    }

    // C6-ii / R5: reset every actuated phase machine (and its detectors) and every rail-crossing
    // phase machine to its initial state (green, next switch at Begin) so a fresh run starts the
    // timeline from scratch. No-op when there are no actuated programs / no crossings.
    private void ResetTimelineStateMachines()
    {
        foreach (var actuated in _actuatedLogics.Values)
        {
            actuated.Reset();
        }

        for (var c = 0; c < _railCrossingViaLaneHandles.Length; c++)
        {
            _railCrossingStep[c] = 0;
            _railCrossingNextSwitch[c] = _config!.Begin;
            _railCrossingState[c] = 'G';
        }
    }

    // One simulation step. `trajectory==null` skips the Export phase (warm-up). Byte-identical to
    // the pre-W1 inline loop body otherwise (same phase order, same calls).
    private void AdvanceOneStep(TrajectorySet? trajectory, double time, double dt)
    {
        {
            // D6 (FastDataPlane ECS readiness -- phased systems over queries, matching FDP's
            // `SystemPhase`-ordered systems): the per-step body below is the SAME sequence of
            // passes as before this rung, now labeled by the `SystemPhase` each belongs to
            // (SystemPhase.cs). CLAUDE.md rule 2 / the D6 briefing: preserve calculation order
            // EXACTLY -- this reorganizes how the loop reads, never what runs or when.

            // X1 (docs/HIGH-DENSITY-X1-DESIGN.md): capture the realism mask ONCE per step from the
            // volatile field the host writes, so every gated phase this step (insertion, teleport,
            // de-jam despawn) reads a single consistent snapshot even if the camera thread swaps it
            // concurrently. null (the default, no camera) leaves every gate fully permissive -> inert.
            _activeMask = _realismMask;

            // [SystemPhase.Input] F2 (probabilistic flow): materialize any <flow probability=>
            // arrivals decided by this step's Bernoulli draw as depart-now VehicleDefs BEFORE the
            // insertion pass below picks them up -- mirrors SUMO's MSInsertionControl generating flow
            // vehicles ahead of the insertion attempt. Inert (no draw, no allocation) when the demand
            // has no probability flow, so every committed scenario's Input phase is byte-identical.
            GenerateProbabilisticFlows(time, dt);

            // [SystemPhase.Input] Newly-departed vehicles enter the simulation. Runs before this
            // step's Export so a vehicle inserted THIS step is immediately included in the
            // trajectory point emitted below (matching golden.fcd.xml's presence set: a vehicle
            // is present starting at its own depart-time row, not one step late).
            var pInsert = PhaseStart();
            InsertDepartingVehicles(time);
            PhaseEnd("insert", pInsert);

            // [SystemPhase.Input] P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C/§1F, §2): the teleport
            // transfer re-insertion pass -- MSVehicleTransfer::checkInsertions, called from
            // MSNet.cpp:825 AFTER regular insertion. A vehicle lifted off its lane by the previous
            // step's jam-check (CheckJamTeleports, below) is put back onto the free lane of its
            // jumped-to edge here, BEFORE this frame is emitted, so its discontinuous jump lands in
            // the trajectory at the same step SUMO shows it (see the design's timing derivation).
            // Gated on TimeToTeleport>0 and short-circuits on an empty queue, so byte-identical for
            // every pre-P1F scenario.
            if (_config!.TimeToTeleport > 0.0)
            {
                ProcessTransferQueue(time);
            }

            // [SystemPhase.Input] C10-i: advance any in-progress continuous lane-change maneuver
            // (lanechange.duration > 0) by one step BEFORE this frame is emitted, so the emitted lane
            // reflects the maneuver's progress (source until the midpoint, then target). No-op for
            // every duration-0 scenario (no vehicle is ever mid-maneuver).
            AdvanceLaneChanges();

            // Perf (dense active list) + domain decomposition: compact active-vehicle indices and
            // (opt-in) group them by spatial region ONCE per step, right after all lane mutations
            // settle (insertion + AdvanceLaneChanges) and BEFORE emit -- so emit, the neighbour
            // refill, and the plan phases all share one grouping. The active set + current lanes are
            // frozen from here through PlanMovements (nothing between here and plan moves a vehicle;
            // arrival is applied later, in ExecuteMoves).
            if (ShouldParallelizePlan())
            {
                BuildActiveIndices();
                if (RegionPlan)
                {
                    var pRg = PhaseStart();
                    BuildRegionActive();
                    PhaseEnd("region", pRg);
                }
            }

            // [SystemPhase.Export] Export of the previous frame. This emits the SETTLED state
            // produced by the PRIOR step's PostSimulation phase (plus any vehicle just inserted
            // above) -- it must stay at the TOP of the loop, BEFORE this step's
            // Simulation/PostSimulation run. Moving it to the bottom would change the
            // traffic-light `time+dt` sampling semantics (RedLightConstraint's own comment) and
            // desync the trajectory from the golden. See EmitTrajectory's header comment (also:
            // the `TrajectorySet`/emit allocation and the `Run(int)->TrajectorySet` return
            // contract are untouched here -- that streaming/zero-alloc-export concern belongs to
            // D9's export seam, not D6).
            // W1: skipped in warm-up (trajectory==null) -- the only difference between a warmed and
            // an emitted step; everything else (Input/Simulation/PostSimulation) runs identically.
            if (trajectory is not null)
            {
                var pEmit = PhaseStart();
                EmitTrajectory(trajectory, time);
                PhaseEnd("emit", pEmit);
            }

            // [SystemPhase.Input] Reroute-around-prolonged-blockage. Runs ONCE per step, BEFORE
            // Simulation/PlanMovements, so a vehicle that reroutes this step immediately plans
            // against its NEW route this same step (see UpdateReroutes' own header comment for
            // why this ordering, and why it is still a seam-4 structural mutation rather than a
            // Simulation-phase concern).
            UpdateReroutes(time, dt);

            // [SystemPhase.Input] P1E-4 (HIGH-DENSITY-P1E-DESIGN.md §3): the periodic congestion-
            // reactive reroute device (device.rerouting), DISTINCT from the obstacle-triggered
            // UpdateReroutes just above -- runs beside it, still strictly BEFORE PlanMovements, so
            // a vehicle that reroutes this step plans against its new route this SAME step. Reads
            // the edge-weight snapshot UpdateRerouteEdgeWeights settled at the END of the PREVIOUS
            // step (never a same-step write -- §8 risk 1); entirely inert (returns immediately)
            // whenever ScenarioConfig.ReroutePeriod<=0, the default for every pre-P1E-4 scenario.
            UpdatePeriodicReroutes(time, dt);

            // [SystemPhase.Input] B5-i: dead-reckon MOVING external obstacles (Speed != 0) by
            // Speed*dt. Runs BEFORE the neighbor-query Refill/PlanMovements below so the Plan
            // phase's ObstacleConstraint reads a FROZEN, already-advanced obstacle position for
            // this step (CLAUDE.md rule 2) -- never mutated mid-plan. Static (Speed==0, including
            // every B1 obstacle) obstacles are untouched by this call, so this is a pure no-op
            // for every existing obstacle-free or static-obstacle scenario/test.
            AdvanceObstacles(time, dt);

            // [SystemPhase.Input] R5: advance every rail_crossing's phase state machine from the
            // FROZEN start-of-step train positions, BEFORE PlanMovements, so the road vehicle's Plan
            // reads a settled crossing state this step (same discipline as the obstacle/TLS advances
            // above). No-op for every net with no rail_crossing junction.
            AdvanceRailCrossings(time);

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
            // C6-ii: advance every actuated TLS phase machine BEFORE PlanMovements, so
            // RedLightConstraint reads the phase that governs the movement this step is about to
            // compute. Sampled at `time + dt` for the SAME reason RedLightConstraint samples the TL
            // state at `time + dt` (see RedLightConstraint's timing note): the move planned now
            // becomes the trajectory emitted at the NEXT iteration's `time + dt`, so it must see the
            // phase active at that instant. The detectors this reads were settled by the PREVIOUS
            // step's ExecuteMoves -- exactly SUMO's begin-of-step trySwitch ordering. No-op when
            // there are no actuated programs.
            foreach (var actuated in _actuatedLogics.Values)
            {
                actuated.Advance(time + dt);
            }

            var neighbors = _neighborQuery!;

            var pRefill = PhaseStart();
            if (RegionPlan && ShouldParallelizePlan())
            {
                // Domain decomposition: region-parallel refill -- each region refills its disjoint
                // lanes from its own vehicles, no shared bucket. Byte-identical to the serial Refill.
                System.Threading.Tasks.Parallel.For(0, _regionCount, _parallelOptions, r =>
                {
                    neighbors.RefillRegion(_regionActive[r], _vehicles, _regionLanes[r]);
                });
            }
            else
            {
                neighbors.Refill(ActiveVehicles());
            }

            PhaseEnd("refill", pRefill);
            // Perf (super-linear fix): (re)build the O(1) foe-approach index from the SAME frozen
            // start-of-step routes the plan phase reads. Both readers of it -- ComputeWillPass and
            // PlanMovements (via ComputeMoveIntent -> JunctionYieldConstraint -> FindFoeVehicle) -- run
            // below, before any structural (route) mutation, so this one build serves the whole step.
            var pFoe = PhaseStart();
            BuildFoeApproachIndex();
            PhaseEnd("foeIndex", pFoe);
            // SPATIAL-OPT probe: build the (lane,pos)-ordered packed hot array + leader-slot map that
            // PlanMovements' spatial branch reads (needs the refilled buckets + the active indices).
            if (ShouldParallelizePlan() && SpatialPlan)
            {
                var pPk = PhaseStart();
                BuildPacked(neighbors);
                PhaseEnd("packed", pPk);
            }

            var pWill = PhaseStart();
            ComputeWillPass(neighbors, time);
            PhaseEnd("willPass", pWill);
            var pPlan = PhaseStart();
            PlanMovements(neighbors, time);
            PhaseEnd("plan", pPlan);

            // [SystemPhase.PostSimulation] Apply every vehicle's own MoveIntent, integrate
            // position, flush arrival through the command buffer. `time` is threaded through for
            // C6-ii's induction-loop detector feed (the move this executes produces the FCD frame
            // at `time + dt`, which is the SIMTIME MSInductLoop stamps entry/leave with).
            var pExec = PhaseStart();
            ExecuteMoves(time, dt);
            PhaseEnd("execute", pExec);

            // [SystemPhase.PostSimulation] P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1A/§1F, §2): the
            // jam-teleport detection phase -- MSLane::executeMovements' firstNotStopped check,
            // which SUMO runs INSIDE executeMovements (after this step's moves settle, before
            // changeLanes). It reads the post-ExecuteMoves WaitingTime (already updated + its
            // command buffer flushed above) and lifts each lane's frontmost stuck vehicle into the
            // transfer queue. Positioned AFTER ExecuteMoves and BEFORE DecideSpeedGainChanges so
            // the just-transferred vehicle is already excluded from the speed-gain pass. Gated on
            // TimeToTeleport>0, so byte-identical (never runs) for every pre-P1F scenario.
            if (_config!.TimeToTeleport > 0.0)
            {
                CheckJamTeleports(time, dt);
            }

            // [SystemPhase.PostSimulation] X1 (docs/HIGH-DENSITY-X1-DESIGN.md): the aggressive
            // off-camera de-jam despawn. Runs right after the teleport phase (same post-move,
            // WaitingTime-settled state) and is gated OFF by default (DejamDespawnTime <= 0), so it is a
            // no-op / byte-identical for every committed scenario. When enabled it removes off-camera
            // jam blockers before they would reach time-to-teleport.
            if (DejamDespawnTime > 0.0)
            {
                DejamDespawn();
            }

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
            var pGain = PhaseStart();
            DecideSpeedGainChanges(time, dt);
            PhaseEnd("speedGain", pGain);

            // [SystemPhase.PostSimulation] P1E-4 (§1C/§3): end-of-step edge-weight smoothing
            // update for the periodic reroute device -- runs LAST, after ExecuteMoves/
            // DecideSpeedGainChanges have fully settled this step's positions/speeds, so the NEXT
            // step's UpdatePeriodicReroutes (which runs near the TOP of the next AdvanceOneStep
            // call, before PlanMovements) always reads a fully-settled PREVIOUS-step snapshot,
            // never a mid-write one -- the temporal analog of SUMO's waitForAll barrier (§8 risk
            // 1: this relative order is a correctness requirement). Entirely inert (returns
            // immediately) whenever ScenarioConfig.ReroutePeriod<=0.
            UpdateRerouteEdgeWeights(time, dt);
        }
    }

    // F2 (probabilistic flow): for each active <flow probability=>, draw ONCE from that flow's own
    // seeded stream this step and, on a hit, materialize one depart-now VehicleDef ("<flowId>.<k>",
    // k the flow's running counter) via CreateRuntime -- the depart-gated InsertDepartingVehicles
    // pass then places it exactly like any hand-listed vehicle. The per-step probability is
    // `Probability * dt` (Probability is per-second, mirroring SUMO's SUMO_ATTR_PROB * TS), clamped
    // to [0,1]; a flow is active over [Begin, End). EXACTLY ONE draw per active flow per step
    // (whether or not it fires) keeps each flow's stream advancing deterministically, so a
    // WarmUp(W)+Run(N) yields the same arrivals as a single Run(W+N). Short-circuits with zero work
    // when the demand has no probability flow, so every committed scenario is byte-identical.
    private void GenerateProbabilisticFlows(double time, double dt)
    {
        var flows = _demand!.ProbabilisticFlows;
        if (flows.Count == 0)
        {
            return;
        }

        for (var i = 0; i < flows.Count; i++)
        {
            var flow = flows[i];
            // Active-window test is half-open [Begin, End), the same convention F1's deterministic
            // expansion uses. A flow outside its window takes no draw at all (its stream is frozen
            // until it becomes active), so Begin/End shift the stream deterministically too.
            if (time < flow.Begin || time >= flow.End)
            {
                continue;
            }

            var perStep = Math.Min(1.0, flow.Probability * dt);
            // One draw per active flow per step -- advances the stream whether or not it fires.
            if (_probFlowRng[i].NextDouble() >= perStep)
            {
                continue;
            }

            var k = _probFlowCounter[i]++;
            CreateRuntime(new VehicleDef(
                Id: $"{flow.Id}.{k}",
                TypeId: flow.TypeId,
                RouteId: flow.RouteId,
                Depart: time,
                DepartPos: flow.DepartPos,
                DepartSpeed: flow.DepartSpeed,
                DepartLaneIndex: flow.DepartLaneIndex));
        }
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
    // Issue 1 cross-edge fix (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §7): the per-VEHICLE stop-lane
    // override consumed by NetworkModel's route-wide best-lanes pass (ComputeBestLanes/
    // ResolveLaneSequenceHandlesWithArrival's `stopOverride` param) -- non-null ONLY when `stops`
    // (VehicleDef.Stops, NOT the route) carries an unreached `<stop parkingArea>`
    // (StopDef.ParkingAreaId != null) whose resolved lane sits on `routeEdges`'s FINAL edge. A stop is
    // per-vehicle -- two vehicles CAN share a route id with different (or no) stops -- so this must be
    // recomputed per vehicle, never baked into any ROUTE-keyed cache (_bestLanesCache/
    // _insertRouteSeqCache are both keyed by RouteId alone). Returns null for every vehicle without
    // such a stop -- the overwhelmingly common case, including every plain lane <stop> (03/13/44) and
    // every non-final-edge parkingArea stop (67's departPos="stop" pull-outs, duration=5) -- so every
    // caller's existing (possibly cached) path is completely unaffected for them.
    private (string StopLaneId, double StopPos)? ParkStopFinalEdgeOverride(IReadOnlyList<StopDef> stops, IReadOnlyList<string> routeEdges)
    {
        if (stops.Count == 0 || routeEdges.Count == 0)
        {
            return null;
        }

        var lastEdgeId = routeEdges[routeEdges.Count - 1];
        foreach (var s in stops)
        {
            if (s.ParkingAreaId is null)
            {
                continue;
            }

            if (_network!.LanesById.TryGetValue(s.LaneId, out var lane) && lane.EdgeId == lastEdgeId)
            {
                return (s.LaneId, s.StartPos);
            }
        }

        return null;
    }

    // Perf (insert): resolve every distinct (route, departLane) insertion lane-sequence up front, in
    // parallel, into _insertRouteSeqCache. Pure function of the immutable network, so byte-identical to
    // lazy resolution -- see the cache field's header and the LoadScenario call site.
    private void PrewarmInsertRouteCache()
    {
        var seen = new HashSet<(string, int)>();
        var keyList = new List<(string RouteId, int DepartLane)>();
        foreach (var def in _demand!.Vehicles)
        {
            // P0-C1: departLane="best" depends on RUNTIME occupancy, so it cannot be prewarmed by a
            // static (RouteId, DepartLaneIndex) key at load time -- it is resolved (and its
            // lane-sequence cached lazily) inside TryInsertOnLane instead. Given (every pre-P0-C1
            // vehicle) is unaffected.
            if (def.DepartLaneIndex.Kind != DepartLaneSpec.Given)
            {
                continue;
            }

            // Issue 1 cross-edge fix: a vehicle whose route's FINAL edge carries a <stop parkingArea>
            // resolves a per-VEHICLE stop-lane-targeted pool (see ParkStopFinalEdgeOverride) that must
            // NOT be written into this ROUTE-keyed cache -- skip it here, exactly like the
            // departLane=Best skip above; TryInsertOnLane resolves it directly (uncached) instead.
            if (ParkStopFinalEdgeOverride(def.Stops, _routesById[def.RouteId].Edges) is not null)
            {
                continue;
            }

            var k = (def.RouteId, def.DepartLaneIndex.Literal);
            if (seen.Add(k))
            {
                keyList.Add(k);
            }
        }

        var results = new (int[] Pool, int[] Arrival)[keyList.Count];
        System.Threading.Tasks.Parallel.For(0, keyList.Count, i =>
        {
            var edges = _routesById[keyList[i].RouteId].Edges;
            results[i] = _network!.ResolveLaneSequenceHandlesWithArrival(edges, keyList[i].DepartLane);
        });

        for (var i = 0; i < keyList.Count; i++)
        {
            _insertRouteSeqCache[keyList[i]] = results[i];
        }
    }

    // P0-C1: departLane="best" (SUMO's DepartLaneDefinition::BEST_FREE, MSEdge::getDepartLane's
    // BEST_FREE arm, sumo/src/microsim/MSEdge.cpp:628-654). Ranks the depart edge's lanes by
    // route-continuation Length (NetworkModel.ComputeBestLanes -- a pure topology fn, callable
    // before any placement decision). The qualifying threshold is `Min(maxLength,
    // BEST_LANE_LOOKAHEAD=3000)`: normally (maxLength <= 3000) that is exactly maxLength, so ONLY
    // the true best-continuation lane(s) qualify; once maxLength exceeds 3000, the threshold is
    // capped at 3000 instead of maxLength itself, widening the tied-for-best set to every lane
    // whose continuation is at least 3000m (SUMO: "beyond a certain length, all lanes are
    // suitable" -- ignores the small getDepartPosBound offset MSEdge.cpp folds into both sides of
    // the comparison, inert here since every P0-C1 scenario departs at/near pos 0). Ties are then
    // broken by LEAST occupancy -- the unnormalized Sum(VType.Length + VType.MinGap) over every
    // ActiveVehicles() currently on that candidate lane (MSEdge::getFreeLane's getBruttoOccupancy
    // ranking, MSLane.cpp:441; getBruttoOccupancy itself normalizes by lane length, but every lane
    // on one edge shares the SAME length here, so the unnormalized sum ranks identically without
    // needing the per-lane length normalization).
    private const double BestLaneLookahead = 3000.0;

    // Issue 1 cross-edge fix: `stopOverride` (see ParkStopFinalEdgeOverride) folds a qualifying
    // park-and-stay stop into the ranking -- SUMO's own getDepartLane calls the SAME stop-aware
    // updateBestLanes (MSVehicle.cpp:1070/1121 "getDepartLane may call updateBestLanes"), so a sink
    // car departing with departLane="best" (the synthetic grid's own shape) must rank the lane that
    // connects onward to the stop, not just the longest raw continuation. null for every vehicle
    // without such a stop, giving the exact prior (lane-agnostic-terminal) ranking.
    private int ResolveBestDepartLane(Route route, Edge edge, (string StopLaneId, double StopPos)? stopOverride)
    {
        var laneQs = _network!.ComputeBestLanes(route.Edges, route.Edges[0], stopOverride);

        var maxLength = double.NegativeInfinity;
        foreach (var q in laneQs)
        {
            if (q.Length > maxLength)
            {
                maxLength = q.Length;
            }
        }

        var threshold = Math.Min(maxLength, BestLaneLookahead);

        var bestIndex = -1;
        var bestOccupancy = double.PositiveInfinity;
        foreach (var q in laneQs)
        {
            if (q.Length < threshold)
            {
                continue;
            }

            var laneHandle = -1;
            foreach (var l in edge.Lanes)
            {
                if (l.Index == q.LaneIndex)
                {
                    laneHandle = l.Handle;
                    break;
                }
            }

            // GAP-3 follow-up: a parked vehicle is off the lane in SUMO (MSVehicleTransfer), so it
            // must not count toward getBruttoOccupancy's departLane="best" tie-break either.
            var occupancy = 0.0;
            foreach (var other in ActiveVehicles())
            {
                if (!other.IsParked && other.LaneHandle == laneHandle)
                {
                    occupancy += other.VType.Length + other.VType.MinGap;
                }
            }

            if (occupancy < bestOccupancy)
            {
                bestOccupancy = occupancy;
                bestIndex = q.LaneIndex;
            }
        }

        return bestIndex;
    }

    private void InsertDepartingVehicles(double time)
    {
        // SUMO's MSInsertionControl processes the depart queue by depart time, ties broken by the
        // route file's vehicle order (definition order), ACROSS all lanes -- NOT grouped/sorted by
        // lane. This cross-lane order matters for cross-junction insertion safety: a downstream
        // leader defined earlier (scenario 39's `lead` on JB, defined before `foll` on AJ) must be
        // placed before the upstream follower's insertion is checked, so the follower actually sees
        // it and delays. `_vehicles` is in definition order and OrderBy is a stable sort, so this is
        // depart-time order with definition-order ties. Per-lane FIFO is preserved via `blockedLanes`:
        // once a candidate on a lane fails this step, later candidates on that SAME lane queue behind
        // it (they are not attempted this step). For every scenario without a cross-lane insertion
        // dependence, the outcome is independent of this cross-lane order (verified: the D1
        // determinism hash and all committed scenarios are unchanged).
        var candidates = _insertCandidates;
        candidates.Clear();
        foreach (var v in _vehicles)
        {
            if (v.Inserted || v.Arrived || v.Def.Depart > time)
            {
                continue;
            }

            candidates.Add(v);
        }

        // P1E-6 (§11): pre-insertion rerouting -- BEFORE any candidate's route/edge/lane is read
        // below (ResolveBestDepartLane's strategic-continuation choice included), give each
        // equipped, not-yet-pre-insertion-rerouted candidate its ONE reroute attempt on the
        // current edge-weight snapshot, so the placement logic below sees the vehicle's NEW route
        // if one installed. Order-independent (parallel fan-out internally); entirely inert
        // (returns immediately) whenever ScenarioConfig.ReroutePeriod<=0, so every non-rerouting
        // scenario's insertion path is byte-identical.
        PreInsertionReroute(candidates, time);

        // L0d: `_vehicles` (hence `candidates`) is in ascending-EntityIndex definition order, so an
        // in-place sort keyed by (Depart, EntityIndex) is byte-identical to the former stable
        // `OrderBy(c => c.Def.Depart)` -- same depart-time order, same definition-order ties -- without
        // allocating the OrderBy's ordered-enumerable + buffer every step.
        candidates.Sort(static (a, b) =>
        {
            var byDepart = a.Def.Depart.CompareTo(b.Def.Depart);
            return byDepart != 0 ? byDepart : a.EntityIndex.CompareTo(b.EntityIndex);
        });

        var blockedLanes = _insertBlockedLanes;
        blockedLanes.Clear();
        foreach (var v in candidates)
        {
            // P1E-6: EffectiveRouteId, not v.Def.RouteId directly -- picks up a route the
            // pre-insertion pass above just installed (RegisterPeriodicReroute writes
            // _effectiveRouteIdByEntity). Falls back to v.Def.RouteId when unset (every vehicle
            // that was never reroute-registered), so this is byte-identical to before for every
            // scenario without device.rerouting (or without a vehicle actually rerouted).
            var route = _routesById[EffectiveRouteId(v)];
            var edge = _network!.EdgesById[route.Edges[0]];

            // P0-C1: departLane="best" resolves its CONCRETE lane index HERE, before the by-index
            // scan below -- it depends on RUNTIME occupancy (ActiveVehicles()), so it cannot be a
            // load-time constant the way a Given index is. Given takes its Literal unchanged (same
            // value the old plain-int field held).
            var departLaneIndex = v.Def.DepartLaneIndex.Kind == DepartLaneSpec.Best
                ? ResolveBestDepartLane(route, edge, ParkStopFinalEdgeOverride(v.Def.Stops, route.Edges))
                : v.Def.DepartLaneIndex.Literal;

            // L0d: manual lane-by-index scan instead of `edge.Lanes.First(l => l.Index == ...)`, whose
            // predicate captured `v` into a fresh closure per candidate. Same first-match result.
            string laneId = null!;
            var laneHandle = -1;
            for (var li = 0; li < edge.Lanes.Count; li++)
            {
                if (edge.Lanes[li].Index == departLaneIndex)
                {
                    laneId = edge.Lanes[li].Id;
                    laneHandle = edge.Lanes[li].Handle;
                    break;
                }
            }

            // X1 (docs/HIGH-DENSITY-X1-DESIGN.md): the on-lane spawn gate. A vehicle whose depart edge
            // is VISIBLE (on-camera) is NOT inserted -- nothing pops into the middle of a visible lane;
            // it is held (skipped this step, retried next) until the edge leaves the visible zone. The
            // hold is NOT a max-depart-delay eviction (that is for real insertion failure, not a
            // deliberate mask hold), so `continue` bypasses both the insert attempt and the eviction
            // below. Inert when _activeMask is null (MayPop always true).
            if (_activeMask is not null && !_activeMask.MayPop(edge.Id))
            {
                continue;
            }

            // A candidate is "not inserted this step" if it is FIFO-blocked behind an earlier
            // same-lane failure (blockedLanes) OR its own TryInsertOnLane fails -- SUMO treats both
            // as refused emits (MSInsertionControl::tryInsert returning 0 into refusedEmits). Track
            // that so the P2-H max-depart-delay eviction below can fire for either case.
            var inserted = false;
            if (blockedLanes.Contains(laneId))
            {
                // An earlier (same-step) candidate on this lane already failed -- FIFO: later
                // candidates queue behind it and are not attempted this step.
            }
            else if (TryInsertOnLane(v, laneHandle, departLaneIndex))
            {
                inserted = true;
            }
            else
            {
                blockedLanes.Add(laneId);
            }

            // P2-H (HIGH-DENSITY-P2H-DESIGN.md): SUMO's max-depart-delay. A vehicle that FAILED to
            // insert this step and has waited longer than max-depart-delay is DELETED from the pending
            // queue, not retried forever (MSInsertionControl.cpp:168:
            // `if (myMaxDepartDelay >= 0 && time - depart > myMaxDepartDelay) deleteVehicle(veh, true)`
            // -- strict `>`, measured from the ORIGINAL depart time, and only AFTER the insertion
            // attempt above, so a vehicle that CAN insert on the very step it crosses the threshold
            // still departs). Gated on MaxDepartDelay >= 0 (default -1 = never), so byte-identical for
            // every scenario that does not set the option: the branch never runs.
            if (!inserted
                && _config!.MaxDepartDelay >= 0.0
                && time - v.Def.Depart > _config.MaxDepartDelay)
            {
                EvictOverdueDeparture(v);
            }
        }
    }

    // P2-H: delete a pending vehicle that waited past max-depart-delay without finding an insertion
    // gap (SUMO's MSInsertionControl deleteVehicle(veh, true) -- "discard": loaded but never
    // inserted). Reuses Despawn's pending-removal idiom: marking it Inserted+Arrived drops it from
    // InsertDepartingVehicles' candidate scan AND every active scan, so it never emits an FCD point --
    // exactly SUMO's "absent from the road". Not counted as running (summary reads active vehicles
    // only) nor as a teleport (the statistic comparator's only subset), so no committed golden's
    // aggregate is disturbed. `_discardedDepartures` is a plain observability tally (no golden reads
    // it). Serial: InsertDepartingVehicles is the single-threaded insertion pass.
    private void EvictOverdueDeparture(VehicleRuntime v)
    {
        v.Inserted = true;   // never (re)considered by InsertDepartingVehicles again
        v.Arrived = true;    // drops it from ActiveVehicles / the read snapshot; never emitted to FCD
        _discardedDepartures++;
    }

    // MSLane::isInsertionSuccess's leader-gap check only (see InsertDepartingVehicles' header
    // comment for the full derivation/scope). Returns true and performs the insertion iff
    // there is no leader on the lane or gap >= 0; otherwise leaves `v` untouched and returns
    // false (queued for a later step). `departLaneIndex` is the CALLER-resolved concrete lane
    // index (Given's literal, or Best's runtime-resolved index -- see InsertDepartingVehicles);
    // used everywhere the old plain-int DepartLaneIndex field used to be read directly, so Given
    // is byte-identical.
    private bool TryInsertOnLane(VehicleRuntime v, int laneHandle, int departLaneIndex)
    {
        // R3 (rail bidi): MSLane::isInsertionSuccess (MSLane.cpp:843-846 for the departure lane,
        // :999-1002 for each forward route lane up to the first rail signal) refuses to insert a
        // rail vehicle while the bidi partner of a lane it will occupy carries a vehicle
        // (getBidiLane()->getVehicleNumberWithPartials() > 0). This is SUMO's no-signal
        // single-track deadlock avoidance: a train onto a shared track waits until the opposing
        // train has cleared it. R3 has no rail signals, so the whole forward route is checked.
        // Inert for road vehicles (rail-only) and non-bidi lanes (TryGetBidiLaneId returns null),
        // so every road scenario's insertion is byte-identical.
        if (RailBidiTrackOccupied(v, departLaneIndex))
        {
            return false;
        }

        // The departure lane is `laneHandle` (resolved by the caller as the edge's lane whose Index
        // == departLaneIndex). LanesByHandle[laneHandle] is that exact lane (dense array index, no
        // per-candidate `edge.Lanes.First(...)` predicate-closure alloc), byte-identical to the old
        // resolution: same edge (route.Edges[0]), same first-matching lane index. (Resolved here,
        // rather than further down where it used to be, purely so departSpeed=Max below can read
        // lane.Speed -- a pure lookup, no behavior change for the Given path.)
        var lane = _network!.LanesByHandle[laneHandle];

        // P0-C1: departPos="stop" -- Lane <stop> only (MSLane::insertVehicle, MSLane.cpp:692-698):
        // if the vehicle's FIRST scheduled stop is on THIS insertion lane, pos = MAX2(0,
        // stop.endPos); otherwise (including every Given scenario, which never has Kind==Stop)
        // silently falls back to BASE (0). Given keeps today's exact literal.
        double insertPos = v.Def.DepartPos.Kind switch
        {
            DepartPosSpec.Given => v.Def.DepartPos.Literal,
            DepartPosSpec.Stop => v.Def.Stops.Count > 0 && v.Def.Stops[0].LaneId == lane.Id
                ? Math.Max(0.0, v.Def.Stops[0].EndPos)
                : 0.0,
            _ => throw new InvalidDataException($"unsupported DepartPosSpec '{v.Def.DepartPos.Kind}'."),
        };

        // MSLane::getLastVehicleInformation / getLeader (same-lane branch): nearest already-
        // inserted, not-arrived vehicle with Pos >= insertPos on this lane -- includes any
        // vehicle inserted earlier THIS SAME step, since this re-scans _vehicles (the engine's
        // authoritative list) on every call rather than a stale snapshot.
        // GAP-3 follow-up: skip IsParked -- a parked vehicle is off the lane (MSVehicleTransfer),
        // so it must not act as the insertion leader either (gated, byte-identical elsewhere).
        VehicleRuntime? leader = null;
        foreach (var other in ActiveVehicles())
        {
            if (other.IsParked || other.LaneHandle != laneHandle)
            {
                continue;
            }

            if (other.Kinematics.Pos >= insertPos && (leader is null || other.Kinematics.Pos < leader.Kinematics.Pos))
            {
                leader = other;
            }
        }

        // P0-C1: departSpeed="max" (MSLane::getDepartSpeed, MSLane.cpp:588-590) requests
        // MIN2(vType.maxSpeed, laneSpeed x speedFactor) with patchSpeed=true -- an unreachable
        // requested speed is CLAMPED by the leader-safety checks below rather than failing
        // insertion (MSLane.cpp:780-808's checkFailure `speed = MIN2(nspeed, speed)` branch); only
        // a real gap<0 overlap still fails. Given (patchSpeed=false) keeps today's exact literal,
        // gap-checked but never adjusted.
        var isMaxSpeed = v.Def.DepartSpeed.Kind == DepartSpeedSpec.Max;
        // Issue-1 follow-up (docs/ISSUE2-JUNCTION-TELEPORT-DESIGN.md §4-CORRECTION): SUMO inserts a
        // departPos="stop" vehicle STOPPED at its stop (MSLane::insertVehicle STOP case, MSLane.cpp:
        // 692-698 -- a car placed at its stop has speed 0), regardless of departSpeed. Without this,
        // a departPos="stop" + departSpeed="max" car inserts MOVING, immediately lane-changes off the
        // stop lane before ProcessNextStop (which needs speed<=haltingSpeed ON the stop lane) can mark
        // the stop Reached, so the stop becomes a permanent unreached "zombie" in the vehicle's queue.
        // When the car later reaches its real arrival edge, the GAP-3/Issue-1 residency guard
        // (near DestroyWithArrival) misreads that stale zombie as a still-pending parking obligation
        // and freezes the car forever at the lane end instead of arriving it -- the root cause of the
        // synthetic-junction never-arrived gap (149 veh) and the teleport cascade it drives. Forcing
        // speed 0 here lets ProcessNextStop mark the origin stop Reached on step 1 exactly as vanilla
        // does. Byte-identical for every committed departPos="stop" golden (48/67/68 all use
        // departSpeed="0", so resolvedSpeed was already 0); only the departSpeed!="0" case changes.
        var isStopDepart = v.Def.DepartPos.Kind == DepartPosSpec.Stop
            && v.Def.Stops.Count > 0 && v.Def.Stops[0].LaneId == lane.Id;
        var resolvedSpeed = isStopDepart
            ? 0.0
            : isMaxSpeed
                ? KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType)
                : v.Def.DepartSpeed.Literal;

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

            if (isMaxSpeed)
            {
                resolvedSpeed = Math.Min(resolvedSpeed, KraussModel.MaximumSafeFollowSpeed(
                    gap, resolvedSpeed, leader.Kinematics.Speed, leader.VType.Decel,
                    v.VType, _config!.StepLength, onInsertion: true));
            }
        }

        // P1E-6: EffectiveRouteId, not v.Def.RouteId directly -- see InsertDepartingVehicles' own
        // comment on this same substitution.
        var route = _routesById[EffectiveRouteId(v)];

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
            // actual insertion speed in this branch. (patchSpeed=true / Max: resolvedSpeed is
            // already the same-lane-leader-clamped cap computed above.)
            Pos = insertPos,
            Speed = resolvedSpeed,
            // Phase 2 (P2.2): departPosLat initial lateral placement -- 0 for every phase-1 vehicle
            // (lane-centred, gated on _sublane), so byte-identical.
            LatOffset = InitialLatOffset(v, lane),
        };
        // GAP-2: remember the resolved depart position for the WHOLE trip (Kinematics.Pos itself
        // advances/wraps as the vehicle moves) -- see VehicleRuntime.DepartPosResolved's own comment.
        v.DepartPosResolved = insertPos;
        // GAP-2 follow-up: seed the running routeLength accumulator at -departPos (SUMO's
        // MSDevice_Tripinfo myRouteLength at NOTIFICATION_DEPARTED) -- see RouteDistanceTraveled.
        v.RouteDistanceTraveled = -insertPos;

        // Rung 9a: resolve the FULL lane sequence for this vehicle's route (spanning internal/
        // junction lanes between edges), not just the departure edge/lane. For a single-edge
        // route this is exactly `[lane.Id]`, matching rungs 1-8 exactly (v.LaneId above already
        // equals the sequence's first element).
        // D3: append the handle-parallel sequence to the shared pool and slice into it, instead
        // of allocating a per-vehicle array -- same traversal, same order as before.
        // C2-v: resolve the Exit (routing pool) AND Arrival sequences together (see
        // _laneSeqArrival's own comment). Both slices share LaneSeqStart/Len; they differ only where
        // the route requires an intra-edge lane change.
        // P0-C1: keyed by the RESOLVED departLaneIndex, so a Best vehicle's lane sequence is
        // resolved on the fly and cached lazily here (PrewarmInsertRouteCache only prewarms Given).
        // P1E-6: keyed by EffectiveRouteId, not v.Def.RouteId -- a pre-insertion-rerouted vehicle
        // must NOT hit the ORIGINAL route's prewarmed cache entry; falls back to v.Def.RouteId
        // (identical to before) whenever no reroute has been registered for this vehicle.
        // Issue 1 cross-edge fix: a vehicle with a qualifying <stop parkingArea> on its route's FINAL
        // edge resolves its pool DIRECTLY (stop-lane-targeted, per-vehicle) and bypasses the
        // ROUTE-keyed cache entirely -- see ParkStopFinalEdgeOverride's header and
        // PrewarmInsertRouteCache's matching skip. null for every other vehicle, so the cache path
        // below is completely unchanged for them.
        var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, route.Edges);
        (int[] Pool, int[] Arrival) seq;
        if (stopOverride is not null)
        {
            seq = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, departLaneIndex, stopOverride: stopOverride);
        }
        else
        {
            var routeKey = (EffectiveRouteId(v), departLaneIndex);
            if (!_insertRouteSeqCache.TryGetValue(routeKey, out seq))
            {
                seq = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, departLaneIndex);
                _insertRouteSeqCache[routeKey] = seq;
            }
        }

        var (poolSeq, arrivalSeq) = seq;

        // Cross-junction INSERTION safety: MSLane::isInsertionSuccess also follow-checks a leader
        // reachable across the next junction(s), not just same-lane leaders. If the safe insertion
        // follow speed against a close downstream leader is below the requested departSpeed
        // (patchSpeed=false), insertion fails THIS step and retries next -- SUMO delays the vehicle
        // until the leader has moved far enough. This is the SAME cross-junction scan the running
        // CrossJunctionLeaderConstraint uses, with the rearmost-leader source being an ActiveVehicles
        // scan (the neighbor query is not yet refilled at insertion time). Uses insertionFollowSpeed
        // = maximumSafeFollowSpeed(gap, ..., onInsertion:true) (MSCFModel::insertionFollowSpeed).
        // Inert unless a close cross-junction leader exists (every other scenario inserts unchanged).
        // L0d: span over poolSeq (no `poolSeq[1..]` array copy); the rearmost source is a by-value
        // struct (ActiveRearmost) instead of a RearmostOnLaneAmongActive method-group delegate.
        var downstreamAtInsert = poolSeq.Length > 1 ? poolSeq.AsSpan(1) : ReadOnlySpan<int>.Empty;
        if (downstreamAtInsert.Length > 0
            && TryFindCrossJunctionLeader(
                resolvedSpeed, v.VType, v, lane.Handle, insertPos, downstreamAtInsert,
                new ActiveRearmost(this), _config!.StepLength, out var insLeader, out var insGap))
        {
            var insSpeed = KraussModel.MaximumSafeFollowSpeed(
                insGap, resolvedSpeed, insLeader.Kinematics.Speed, insLeader.VType.Decel,
                v.VType, _config.StepLength, onInsertion: true);
            if (isMaxSpeed)
            {
                // patchSpeed=true: clamp down instead of failing (only a real gap<0 -- already
                // ruled out above -- fails insertion outright).
                if (insSpeed < resolvedSpeed)
                {
                    resolvedSpeed = insSpeed;
                    v.Kinematics.Speed = resolvedSpeed;
                }
            }
            else if (insSpeed < resolvedSpeed)
            {
                // Not enough room to enter at departSpeed behind the downstream leader -- retry next
                // step (do NOT append to the pools; v stays un-Inserted).
                return false;
            }
        }

        v.LaneSeqStart = _laneSeqPool.Count;
        v.LaneSeqLen = poolSeq.Length;
        _laneSeqPool.AddRange(poolSeq);
        _laneSeqArrival.AddRange(arrivalSeq);
        v.LaneSeqIndex = 0;

        v.Inserted = true;
        return true;
    }

    // R3 (rail bidi): true iff `v` is a rail vehicle AND the bidi partner of any lane on its planned
    // route carries an active vehicle -- SUMO's no-signal single-track deadlock guard
    // (MSLane::isInsertionSuccess, MSLane.cpp:843-846 + :999-1002). Walks the vehicle's whole
    // forward lane sequence (R3 has no rail signals, so the firstRailSignal cut-off never triggers).
    // Occupancy is "an active vehicle whose current lane IS the bidi partner"; the long-train
    // partial-occupancy of a straddled downstream lane (getVehicleNumberWithPartials) is not needed
    // by the committed single-shared-edge rail meet and is left for a scenario that exercises it.
    // Returns false immediately for road vehicles and for a route with no bidi lane, so every road
    // scenario's insertion path is byte-identical.
    private bool RailBidiTrackOccupied(VehicleRuntime v, int departLaneIndex)
    {
        if (!VTypeDefaults.IsRailway(v.VType))
        {
            return false;
        }

        // P1E-6: EffectiveRouteId, not v.Def.RouteId directly -- see InsertDepartingVehicles' own
        // comment on this same substitution (harmless for rail, which is not device.rerouting-
        // equipped in any committed scenario, but keeps this read consistent with the other three).
        var route = _routesById[EffectiveRouteId(v)];
        var (poolSeq, _) = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, departLaneIndex);
        foreach (var handle in poolSeq)
        {
            var bidiLaneId = _network.TryGetBidiLaneId(_network.LanesByHandle[handle].Id);
            if (bidiLaneId is null)
            {
                continue;
            }

            // GAP-3 follow-up: a parked vehicle is off-lane, so it cannot occupy the bidi track
            // either (gated on IsParked; no committed rail scenario ever sets it, so this is a
            // no-op safety net, not an observed golden change).
            foreach (var other in ActiveVehicles())
            {
                if (!other.IsParked && other.LaneId == bidiLaneId)
                {
                    return true;
                }
            }
        }

        return false;
    }

    // R4 (rail signal): precompute, for each rail_signal-controlled link, the "conflict lanes" that
    // must be clear for that signal to show green -- a scoped port of MSRailSignal's driveway
    // reservation (MSRailSignal::DriveWay::conflictLaneOccupied, MSRailSignal.cpp:954). The full
    // subsystem builds a driveway = the block of lanes from the signal forward to the NEXT rail
    // signal, plus that block's bidi partners, plus flank/crossing-foe lanes, and reserves it iff
    // none is occupied. This minimal single-block port takes the conflict set to be the BIDI
    // PARTNERS of the forward block lanes (the opposing traffic on the shared single track) -- which
    // is exactly what makes a single-track meet hold one train at its signal while the other
    // occupies the shared block. SCOPED OUT (deferred, not exercised by the committed single-block
    // meet anchor scenarios/50-rail-signal-meet): the crossing/flank-foe internal lanes (they clear
    // on the SAME step as the bidi block here, so they do not change this golden), the protecting-
    // switch/link-conflict checks, and the mustYield PRIORITY tie-break for two trains reaching
    // opposing signals SIMULTANEOUSLY (avoided by the scenario's staggered departure, so the winner
    // is unambiguous and each held train simply sees the shared block physically occupied).
    private void BuildRailSignalInfo()
    {
        _railSignalConflictLaneHandles = null;
        if (_network is null)
        {
            return;
        }

        int[][]? byLaneHandle = null;
        foreach (var conn in _network.Connections)
        {
            // A rail signal's controlled connection carries tl="<junction id>" where that junction
            // is type="rail_signal" (netconvert emits no <tlLogic> for it -- it is computed at run
            // time). Every non-rail-signal tl= is a normal <tlLogic> and stays with RedLightConstraint.
            if (conn.Tl is null
                || !_network.JunctionsById.TryGetValue(conn.Tl, out var sigJunction)
                || sigJunction.Type != "rail_signal")
            {
                continue;
            }

            // Forward block: walk the route from the link's to-lane until the edge whose downstream
            // node is the NEXT rail signal (inclusive), collecting the lanes the train will occupy.
            // Single continuation per lane in scope; a 64-hop guard bounds any malformed loop.
            var conflicts = new List<int>();
            var curEdgeId = conn.To;
            var curLaneIndex = conn.ToLane;
            for (var hop = 0; hop < 64; hop++)
            {
                if (!_network.EdgesById.TryGetValue(curEdgeId, out var edge))
                {
                    break;
                }

                var blockLane = edge.Lanes.FirstOrDefault(l => l.Index == curLaneIndex) ?? edge.Lanes.FirstOrDefault();
                if (blockLane is null)
                {
                    break;
                }

                // The opposing lane on the shared track: a bidi partner of a block lane must be clear.
                // Store its HANDLE (dense int) so the hot path compares handles, not strings.
                var bidi = _network.TryGetBidiLaneId(blockLane.Id);
                if (bidi is not null)
                {
                    var bidiHandle = _network.LaneHandleById[bidi];
                    if (!conflicts.Contains(bidiHandle))
                    {
                        conflicts.Add(bidiHandle);
                    }
                }

                // Stop the block at the next rail signal (its own driveway starts there).
                if (_network.JunctionsById.TryGetValue(edge.To, out var toJunction)
                    && toJunction.Type == "rail_signal")
                {
                    break;
                }

                if (!_network.ConnectionsByFromEdgeLane.TryGetValue((curEdgeId, curLaneIndex), out var outs)
                    || outs.Count == 0)
                {
                    break;
                }

                curEdgeId = outs[0].To;
                curLaneIndex = outs[0].ToLane;
            }

            if (conflicts.Count > 0)
            {
                // Allocate the dense (per-lane-handle) array lazily, only for a net that has a rail
                // signal -- so it stays null (and the hot-path guard a single null test) otherwise.
                byLaneHandle ??= new int[_network.LanesByHandle.Count][];
                var fromLaneHandle = _network.EdgesById[conn.From].Lanes.First(l => l.Index == conn.FromLane).Handle;
                byLaneHandle[fromLaneHandle] = conflicts.ToArray();
            }
        }

        _railSignalConflictLaneHandles = byLaneHandle;
    }

    // R4 (rail signal): the stop-line brake for a train approaching a RED rail signal. The signal on
    // the lane's outgoing rail-signal link is RED iff any of the link's precomputed conflict lanes
    // (BuildRailSignalInfo) is occupied -- with PARTIAL occupancy (a long train's body spans several
    // lanes) -- by another train (MSRailSignal::updateCurrentPhase -> DriveWay::reserve ->
    // conflictLaneOccupied's `!lane->isEmpty()` / getVehicleNumberWithPartials, MSRailSignal.cpp:158,
    // 954-1004). A red rail signal is then just a red MSLink: the train brakes to the same stop line
    // a red traffic light uses (majorStopOffset = DIST_TO_STOPLINE_EXPECT_PRIORITY = 1.0 m before the
    // junction), via the identical stop-line math RedLightConstraint uses (MSVehicle.cpp:2641-2666,
    // 2734). Returns +infinity (non-binding) when there is no rail signal on this lane, or its
    // conflict lanes are all clear (green). Inert-when-absent: _railSignalConflictLaneHandles is null
    // for every scenario with no rail_signal junction, so the guard is a single null test and this is
    // a no-op Min term for all road/rail non-signal scenarios.
    private double RailSignalConstraint(
        VehicleRuntime v, Lane lane, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // Dense array index by the ego lane's handle (== its LanesByHandle index) -- no string
        // hashing/comparison. Null array (no rail signal in the net) or null entry (this lane is not
        // signal-guarded) => non-binding.
        var conflictLaneHandles = _railSignalConflictLaneHandles?[lane.Handle];
        if (conflictLaneHandles is null)
        {
            return double.PositiveInfinity;
        }

        var red = false;
        foreach (var conflictLaneHandle in conflictLaneHandles)
        {
            // GAP-3 follow-up: a parked vehicle is off-lane, so it cannot hold a rail signal red
            // either (gated on IsParked; no committed rail scenario ever sets it).
            foreach (var other in ActiveVehicles())
            {
                if (other.IsParked || ReferenceEquals(other, v))
                {
                    continue;
                }

                if (VehicleBodyOccupies(other, conflictLaneHandle))
                {
                    red = true;
                    break;
                }
            }

            if (red)
            {
                break;
            }
        }

        if (!red)
        {
            return double.PositiveInfinity;
        }

        // Stop-line brake, identical to RedLightConstraint's tail (see there for the source cites):
        // stop at majorStopOffset = 1.0 m before the lane end (the junction), never emergency-braking
        // past the point where a comfortable stop is still possible.
        var seen = lane.Length - v.Kinematics.Pos;
        var stopDecel = v.VType.Decel;
        var brakeDist = KraussModel.BrakeGap(v.Kinematics.Speed, stopDecel, headwayTime: 0.0, dt);
        var canBrakeBeforeLaneEnd = seen >= brakeDist;
        const double vehicleStopOffset = 0.0;
        var canBrakeBeforeStopLine = seen - vehicleStopOffset >= brakeDist;
        if (!canBrakeBeforeStopLine)
        {
            return double.PositiveInfinity;
        }

        const double majorStopOffset = 1.0;
        const double positionEps = 0.1;
        var laneStopOffset = majorStopOffset;
        if (canBrakeBeforeLaneEnd)
        {
            laneStopOffset = Math.Min(laneStopOffset, seen - brakeDist);
        }

        laneStopOffset = Math.Max(positionEps, laneStopOffset);
        var stopDist = Math.Max(0.0, seen - laneStopOffset);
        return StopSpeedFor(v.VType, v.Kinematics.Speed, stopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
    }

    // R4 (rail signal): does `other`'s physical body currently touch lane HANDLE `laneHandle`? A train
    // of length L occupies [Pos - L, Pos] along its route, spanning back through the lanes it just
    // traversed -- SUMO's getVehicleNumberWithPartials counts a vehicle on every lane its body
    // overlaps. Walks the vehicle's arrival lane sequence backward from its current (front) lane,
    // consuming L, and returns true if `laneHandle` is any lane the body reaches. The arrival pool
    // ALREADY stores lane HANDLES, so this compares handles directly (no LanesByHandle->Id string
    // resolution). This partial-occupancy span is what makes a rail signal / crossing stay closed
    // until a long train's TAIL clears the block, not just its front.
    private bool VehicleBodyOccupies(VehicleRuntime other, int laneHandle)
    {
        var idx = other.LaneSeqIndex;
        var remaining = other.VType.Length;
        // Body available on the current (front) lane = from the lane start (0) up to the front Pos.
        var availOnLane = other.Kinematics.Pos;
        while (idx >= 0 && remaining > 1e-9)
        {
            if (_laneSeqArrival[other.LaneSeqStart + idx] == laneHandle)
            {
                return true;
            }

            remaining -= availOnLane;
            idx--;
            if (idx >= 0)
            {
                // A full previous lane's length of body may lie on it.
                availOnLane = _network!.LanesByHandle[_laneSeqArrival[other.LaneSeqStart + idx]].Length;
            }
        }

        return false;
    }

    // R5 (rail crossing): default MSRailCrossing timing parameters (MSRailCrossing.cpp:57-64
    // getParameter defaults). All in seconds (== SUMOTime steps at step-length 1). Not overridden
    // by any committed scenario, so the parsed constants stand.
    private const double RailCrossingYellowTime = 5.0;   // "yellow-time"
    private const double RailCrossingOpeningTime = 3.0;  // "opening-time" (red-yellow while opening)
    private const double RailCrossingOpeningDelay = 3.0; // "opening-delay"
    private const double RailCrossingMinGreen = 5.0;     // "min-green"

    // R5 (rail crossing): precompute each rail_crossing junction's controlled ROAD approach lanes
    // and the internal RAIL via-lanes whose occupation closes the crossing. A rail_crossing's
    // <connection>s carry tl="<junction id>": the road links have linkIndex >= 0 (controlled), the
    // rail links have linkIndex < 0 (uncontrolled -- trains pass freely, but occupying the crossing
    // closes it). Ported from MSRailCrossing::init's split of myLinks (road) vs myIncomingRailLinks.
    private void BuildRailCrossingInfo()
    {
        _railCrossingByRoadLaneHandle = null;
        _railCrossingViaLaneHandles = Array.Empty<int[]>();
        _railCrossingStep = Array.Empty<int>();
        _railCrossingNextSwitch = Array.Empty<double>();
        _railCrossingState = Array.Empty<char>();
        if (_network is null)
        {
            return;
        }

        int[]? byRoadLaneHandle = null;
        var viaLaneHandles = new List<int[]>();
        foreach (var junction in _network.Junctions)
        {
            if (junction.Type != "rail_crossing")
            {
                continue;
            }

            var crossingIndex = viaLaneHandles.Count;
            var viaHandles = new List<int>();
            foreach (var conn in _network.Connections)
            {
                if (conn.Tl != junction.Id)
                {
                    continue;
                }

                if (conn.LinkIndex is { } li && li >= 0)
                {
                    // Controlled road approach lane -- the car yields here. Map its HANDLE to this
                    // crossing's dense index (dense int-indexed array; -1 elsewhere).
                    byRoadLaneHandle ??= NewFilledArray(_network.LanesByHandle.Count, -1);
                    var roadLaneHandle = _network.EdgesById[conn.From].Lanes.First(l => l.Index == conn.FromLane).Handle;
                    byRoadLaneHandle[roadLaneHandle] = crossingIndex;
                }
                else if (conn.Via is not null)
                {
                    // Uncontrolled rail link through the crossing -- its via lane HANDLE is what a
                    // train occupies while physically on the crossing.
                    var viaHandle = _network.LaneHandleById[conn.Via];
                    if (!viaHandles.Contains(viaHandle))
                    {
                        viaHandles.Add(viaHandle);
                    }
                }
            }

            viaLaneHandles.Add(viaHandles.ToArray());
        }

        if (viaLaneHandles.Count > 0)
        {
            _railCrossingByRoadLaneHandle = byRoadLaneHandle;
            _railCrossingViaLaneHandles = viaLaneHandles.ToArray();
            _railCrossingStep = new int[viaLaneHandles.Count];
            _railCrossingNextSwitch = new double[viaLaneHandles.Count];
            _railCrossingState = new char[viaLaneHandles.Count];
            var begin = _config?.Begin ?? 0.0;
            for (var c = 0; c < viaLaneHandles.Count; c++)
            {
                _railCrossingStep[c] = 0;
                _railCrossingNextSwitch[c] = begin;
                _railCrossingState[c] = 'G';
            }
        }
    }

    // Small cold-path helper: a new int[] of `length` filled with `value` (used to init the
    // road-lane -> crossing-index map to -1, i.e. "not a crossing approach").
    private static int[] NewFilledArray(int length, int value)
    {
        var a = new int[length];
        Array.Fill(a, value);
        return a;
    }

    // R5 (rail crossing): advance every rail_crossing's phase state machine one step -- a scoped port
    // of MSRailCrossing::updateCurrentPhase (MSRailCrossing.cpp:120). Runs once per step in the Input
    // phase (before PlanMovements) off the FROZEN start-of-step train positions, so the road vehicle's
    // Plan reads a settled crossing state (CLAUDE.md rule 2). The crossing closes while a train
    // physically occupies a rail via-lane (getViaLane()->getVehicleNumberWithPartials() > 0, line 136),
    // then runs the opening sequence (red -> red-yellow -> green) before reopening. SCOPED OUT
    // (deferred, not exercised by the committed anchor scenarios/51-rail-crossing, whose road vehicle
    // only reacts once the train is physically on the crossing): the approaching-train ARRIVAL-TIME
    // prediction arm (avi.arrivalTime/leavingTime vs time-gap, lines 128-134) that pre-closes the
    // crossing before the train arrives -- this port closes purely on physical occupancy. No-op for
    // any net without a rail_crossing junction.
    private void AdvanceRailCrossings(double time)
    {
        if (_railCrossingViaLaneHandles.Length == 0)
        {
            return;
        }

        for (var crossing = 0; crossing < _railCrossingViaLaneHandles.Length; crossing++)
        {
            var occupied = false;
            foreach (var viaLaneHandle in _railCrossingViaLaneHandles[crossing])
            {
                // GAP-3 follow-up: a parked vehicle is off-lane, so it cannot hold a rail crossing
                // closed either (gated on IsParked; no committed rail scenario ever sets it).
                foreach (var other in ActiveVehicles())
                {
                    if (!other.IsParked && VehicleBodyOccupies(other, viaLaneHandle))
                    {
                        occupied = true;
                        break;
                    }
                }

                if (occupied)
                {
                    break;
                }
            }

            // MSRailCrossing.cpp:136-139: while a train occupies the crossing, stayRedUntil is held
            // DELTA_T + opening-delay ahead of now, i.e. wait = 1 + opening-delay; else wait = 0.
            var wait = occupied ? 1.0 + RailCrossingOpeningDelay : 0.0;
            var step = _railCrossingStep[crossing];
            var nextSwitch = _railCrossingNextSwitch[crossing];

            // The base TL logic only re-runs trySwitch (updateCurrentPhase) when the current phase's
            // scheduled duration elapses; fixed-duration phases (yellow, opening) are not re-checked
            // until they end. Model that with nextSwitch.
            if (time >= nextSwitch)
            {
                switch (step)
                {
                    case 0: // 'G' green: stay open unless a train is (about to be) on the crossing
                        if (wait == 0.0)
                        {
                            nextSwitch = time + 1.0;
                        }
                        else
                        {
                            step = 1;
                            nextSwitch = time + RailCrossingYellowTime;
                        }

                        break;
                    case 1: // 'y' yellow over -> red
                        step = 2;
                        nextSwitch = time + Math.Max(1.0, wait);
                        break;
                    case 2: // 'r' red: may we start opening?
                        if (wait == 0.0)
                        {
                            step = 3;
                            nextSwitch = time + RailCrossingOpeningTime;
                        }
                        else
                        {
                            nextSwitch = time + wait;
                        }

                        break;
                    default: // 3: 'u' red-yellow (opening) over
                        if (wait == 0.0)
                        {
                            step = 0;
                            nextSwitch = time + RailCrossingMinGreen;
                        }
                        else
                        {
                            step = 2;
                            nextSwitch = time + wait;
                        }

                        break;
                }
            }

            _railCrossingStep[crossing] = step;
            _railCrossingNextSwitch[crossing] = nextSwitch;
            _railCrossingState[crossing] = step switch { 0 => 'G', 1 => 'y', 2 => 'r', _ => 'u' };
        }
    }

    // R5 (rail crossing): a road vehicle approaching a rail_crossing whose road link is not green
    // brakes to the crossing's stop line (road vehicles yield to trains). The crossing's road-link
    // state is 'G' (go) only when open; 'y'/'r'/'u' all mean stop-if-you-can, handled by the same
    // stop-line brake a red light uses (RedLightConstraint's tail). +infinity (non-binding) when
    // this lane is not a controlled crossing approach or the crossing is open. Inert for every net
    // with no rail_crossing junction (_railCrossingByRoadLaneHandle null).
    private double RailCrossingConstraint(
        VehicleRuntime v, Lane lane, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // Dense array index by the ego lane's handle -> its crossing index (-1 = not a controlled
        // approach). Null array (no crossing in the net) => non-binding. No string lookup.
        var crossing = _railCrossingByRoadLaneHandle?[lane.Handle] ?? -1;
        if (crossing < 0)
        {
            return double.PositiveInfinity;
        }

        if (_railCrossingState[crossing] == 'G')
        {
            return double.PositiveInfinity;
        }

        // Stop-line brake, identical to RedLightConstraint's / RailSignalConstraint's tail: stop at
        // majorStopOffset = 1.0 m before the crossing, never emergency-braking past a comfortable stop.
        var seen = lane.Length - v.Kinematics.Pos;
        var stopDecel = v.VType.Decel;
        var brakeDist = KraussModel.BrakeGap(v.Kinematics.Speed, stopDecel, headwayTime: 0.0, dt);
        var canBrakeBeforeLaneEnd = seen >= brakeDist;
        const double vehicleStopOffset = 0.0;
        var canBrakeBeforeStopLine = seen - vehicleStopOffset >= brakeDist;
        if (!canBrakeBeforeStopLine)
        {
            return double.PositiveInfinity;
        }

        const double majorStopOffset = 1.0;
        const double positionEps = 0.1;
        var laneStopOffset = majorStopOffset;
        if (canBrakeBeforeLaneEnd)
        {
            laneStopOffset = Math.Min(laneStopOffset, seen - brakeDist);
        }

        laneStopOffset = Math.Max(positionEps, laneStopOffset);
        var stopDist = Math.Max(0.0, seen - laneStopOffset);
        return StopSpeedFor(v.VType, v.Kinematics.Speed, stopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
    }

    // Cross-junction insertion helper: the rearmost (smallest-Pos) active vehicle on a lane handle,
    // scanned directly from the engine's vehicle list (the neighbor query is not yet refilled when
    // InsertDepartingVehicles runs at the top of the step). Mirrors LaneNeighborQuery.GetRearmost.
    // GAP-3 follow-up (ISSUE2-JUNCTION-KEEPCLEAR-DESIGN.md): this ActiveVehicles() scan does NOT go
    // through LaneNeighborQuery, so it does not inherit that query's IsParked exclusion -- skip parked
    // vehicles here too, or a park-and-stay car on a downstream/exit lane is wrongly returned as the
    // cross-junction leader (TryFindCrossJunctionLeader's insertion-time ActiveRearmost source), making
    // an approaching/inserting vehicle brake for a car that, in SUMO, is off the lane entirely. Gated on
    // IsParked (default false), so byte-identical for every scenario without a parked vehicle.
    private VehicleRuntime? RearmostOnLaneAmongActive(int laneHandle)
    {
        VehicleRuntime? rearmost = null;
        foreach (var other in ActiveVehicles())
        {
            if (other.IsParked)
            {
                continue;
            }

            if (other.LaneHandle == laneHandle
                && (rearmost is null || other.Kinematics.Pos < rearmost.Kinematics.Pos))
            {
                rearmost = other;
            }
        }

        return rearmost;
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
            RegisterRerouted(v, newEdges);   // keep _routesById/BestLanes reads in sync (see field header)

            var laneIndex = _network.LanesByHandle[v.LaneHandle].Index;
            // C2-v: append BOTH the Exit (pool) and Arrival slices in lockstep (they share
            // LaneSeqStart/Len). The reroute keeps the vehicle physically where it is (arrival[0] ==
            // its current lane), so for the common no-intra-change route this is identical to before.
            // Issue 1 cross-edge fix: see RerouteActive's matching comment -- keep the stop-lane
            // override alive across an obstacle-triggered reroute too.
            var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, newEdges);
            var (newPoolSeq, newArrivalSeq) = _network.ResolveLaneSequenceHandlesWithArrival(newEdges, laneIndex, stopOverride: stopOverride);
            var newLaneSeqStart = _laneSeqPool.Count;
            _laneSeqPool.AddRange(newPoolSeq);
            _laneSeqArrival.AddRange(newArrivalSeq);
            _commandBuffer.ReplaceRoute(v, newLaneSeqStart, newPoolSeq.Length);

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

    // P1E-4 (HIGH-DENSITY-P1E-DESIGN.md §1A/§1B/§3/§4, seam #4): the periodic congestion-reactive
    // reroute device (MSDevice_Routing). Inert-when-disabled: returns immediately whenever
    // ScenarioConfig.ReroutePeriod<=0 (the default) or _edgeWeights is null (only ever null in
    // that same case -- see InitializeLoaded), so this costs nothing and changes nothing for any
    // scenario that does not explicitly opt in.
    private void UpdatePeriodicReroutes(double time, double dt)
    {
        if (_config!.ReroutePeriod <= 0.0 || _edgeWeights is null)
        {
            return;
        }

        _router ??= new NetworkRouter(_network!);
        var network = _network!;
        var router = _router;
        var edgeWeights = _edgeWeights;

        // Collect this step's due batch: equipped, active ("inserted, not arrived" via
        // ActiveVehicles(), same D6 query UpdateReroutes/PlanMovements use), NOT mid-junction
        // (an internal-lane vehicle is committed to the connection it is traversing -- nowhere
        // sensible to reroute from until it lands on its next normal lane, exactly like
        // UpdateReroutes' own guard), due (time >= NextRerouteTime), and NOT skip-stale (§1A:
        // the weights must have changed since this vehicle's OWN last routing attempt).
        var batch = _rerouteBatchScratch;
        batch.Clear();
        foreach (var v in ActiveVehicles())
        {
            if (!v.RerouteEquipped || v.LaneId.StartsWith(':') || time < v.NextRerouteTime
                || v.LastRoutingTime >= _lastAdaptationTime)
            {
                continue;
            }

            // Issue 1 cross-edge fix follow-up (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md par 7): a
            // vehicle currently HELD at a stop (including parked, off-lane) must not be rerouted --
            // ported from MSDevice_Routing::wrappedRerouteCommandExecute (sumo/src/microsim/devices/
            // MSDevice_Routing.cpp:278-284): `if (myHolder.isStopped()) { myRerouteAfterStop = true;
            // } else { reroute(...); }` -- a stopped holder is skipped THIS cycle, not rerouted.
            // Without this, a park-and-stay sink that has ALREADY converged onto its parkingArea's
            // lane and parked (the very case the cross-edge fix restores) gets swept into a later
            // periodic-reroute batch anyway (ActiveVehicles() does not exclude parked vehicles --
            // see RearmostOnLaneAmongActive's own IsParked-exclusion comment for why that query is
            // deliberately narrower); the router then computes a same-edge-to-same-edge candidate
            // that differs from the trivial "stay put" answer, installs a fresh
            // RegisterPeriodicReroute route/pool via ReplaceRoute, and the vehicle spuriously
            // resumes moving and drives off -- undoing the residency fix. `NextRerouteTime` is
            // deliberately left UNADVANCED here (unlike a vehicle that actually gets routed below),
            // so a still-stopped vehicle is cheaply re-skipped every step and a vehicle that later
            // resumes becomes reroute-eligible again immediately, matching `myRerouteAfterStop`'s
            // "reroute once it starts moving again" intent closely enough for this scenario shape.
            // GATED on GetStops(v)'s front stop being Reached (MSVehicle::isStopped(): `!myStops.
            // empty() && myStops.front().reached`) -- false for every moving vehicle, so byte-
            // identical for every existing device.rerouting scenario (none of which combine it with
            // a stop that is still held when a periodic reroute comes due).
            if (GetStops(v) is { Count: > 0 } vStops && vStops.Peek().Reached)
            {
                continue;
            }

            batch.Add(v);
        }

        if (batch.Count == 0)
        {
            return;
        }

        // PARALLEL fan-out (§4): each vehicle's router call is a PURE read of the FROZEN
        // edge-weight snapshot (Update never runs during this pass -- only at end of step, see
        // UpdateRerouteEdgeWeights) into its OWN scratch slot; no shared writes, so the result is
        // independent of thread scheduling/order -- parallel is bit-identical to serial.
        EnsureRerouteCandidateCapacity(batch.Count);
        var candidates = _rerouteCandidateScratch;
        var useAStar = string.Equals(_config.RoutingAlgorithm, "astar", StringComparison.Ordinal);
        System.Threading.Tasks.Parallel.For(0, batch.Count, _parallelOptions, i =>
        {
            var v = batch[i];
            var currentEdge = network.LanesByHandle[v.LaneHandle].EdgeId;
            var destEdge = network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + v.LaneSeqLen - 1]].EdgeId;
            var vehicleMaxSpeed = v.VType.MaxSpeed * v.SpeedFactor;
            double EdgeEffort(string edgeId) => edgeWeights.Effort(edgeId, vehicleMaxSpeed);

            candidates[i] = useAStar
                ? router.RouteAStar(currentEdge, destEdge, EdgeEffort)
                : router.Route(currentEdge, destEdge, EdgeEffort);
        });

        // SERIAL apply (§4, matching ChangeLane's/ReplaceRoute's own record discipline): every due
        // vehicle re-arms its schedule regardless of outcome (§1B: NO improvement gate -- the
        // candidate always replaces the current route, unless it is empty/missing or IDENTICAL to
        // the current remaining route, in which case only the identical-list short-circuit skips
        // the install, never the schedule advance).
        for (var i = 0; i < batch.Count; i++)
        {
            var v = batch[i];
            v.NextRerouteTime += _config.ReroutePeriod;
            v.LastRoutingTime = time;

            var candidate = candidates[i];
            if (candidate is null || candidate.Count == 0)
            {
                // Structural failure (unreachable destination) -- leave the vehicle on its
                // current route, exactly like UpdateReroutes' own "no alternate route exists" arm.
                continue;
            }

            var currentEdge = network.LanesByHandle[v.LaneHandle].EdgeId;
            var currentRemainingEdges = CurrentRemainingRouteEdges(v, currentEdge);
            if (candidate.SequenceEqual(currentRemainingEdges))
            {
                continue; // §1B: identical edge list -- short-circuit, no new route object
            }

            // P1E-4's own route-slot recycling (§0.5.3) -- distinct from RegisterRerouted, which
            // the obstacle-based UpdateReroutes/RerouteActive above keep using unchanged.
            RegisterPeriodicReroute(v, candidate);

            var laneIndex = network.LanesByHandle[v.LaneHandle].Index;
            // Issue 1 cross-edge fix: see RerouteActive's matching comment -- keep the stop-lane
            // override alive across a periodic congestion reroute too (the synthetic grid equips
            // device.rerouting on every vehicle, sinks included).
            var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, candidate);
            var (newPoolSeq, newArrivalSeq) = network.ResolveLaneSequenceHandlesWithArrival(candidate, laneIndex, stopOverride: stopOverride);
            var newLaneSeqStart = _laneSeqPool.Count;
            _laneSeqPool.AddRange(newPoolSeq);
            _laneSeqArrival.AddRange(newArrivalSeq);
            _commandBuffer.ReplaceRoute(v, newLaneSeqStart, newPoolSeq.Length);
        }

        // Flushed BEFORE PlanMovements (called later this same AdvanceOneStep), matching
        // UpdateReroutes' own end-of-method flush timing exactly.
        _commandBuffer.Flush();
    }

    private void EnsureRerouteCandidateCapacity(int n)
    {
        if (_rerouteCandidateScratch.Length < n)
        {
            _rerouteCandidateScratch = new IReadOnlyList<string>?[n];
        }
    }

    // P1E-6 (HIGH-DENSITY-P1E-DESIGN.md §11): pre-insertion rerouting. SUMO reroutes each
    // device.rerouting-equipped vehicle ONCE at/around departure (MSDevice_Routing::
    // preInsertionReroute), not only periodically -- without this, a SumoSharp vehicle
    // pre-positions (departLane="best"'s strategic-continuation choice, ResolveBestDepartLane)
    // for its ORIGINAL route until its first periodic reroute fires at depart+period, landing in a
    // different lane than SUMO's already-rerouted vehicle on a multi-lane road. This runs from
    // InsertDepartingVehicles, over exactly the vehicles that method already gathered as due-to-
    // insert this step (time >= Depart, not yet Inserted/Arrived) -- BEFORE that method reads any
    // vehicle's route for lane/edge resolution, so a rerouted vehicle inserts and pre-positions on
    // its NEW route. Reuses the SAME machinery as UpdatePeriodicReroutes (§1B/§3/§4): A*/Dijkstra
    // per RoutingAlgorithm, cost = the settled previous-step _edgeWeights snapshot (never a same-
    // step write -- UpdateRerouteEdgeWeights still only runs at end of step), no improvement gate,
    // identical-edge-list short-circuit, one reusable per-entity route slot (RegisterPeriodicReroute
    // -- never a second id per vehicle). Entirely inert whenever ScenarioConfig.ReroutePeriod<=0
    // (_edgeWeights stays null), so every pre-P1E-6 / non-rerouting scenario's insertion path never
    // even calls this and stays byte-identical (see the guard below).
    //
    // Distinct from and never touches NextRerouteTime/LastRoutingTime (the periodic schedule
    // depart+period, +period, ... is untouched -- SUMO does both pre-insertion AND periodic).
    // Guarded to run AT MOST ONCE per vehicle via PreInsertionRerouteDone (set true regardless of
    // whether a new route actually installs -- done is done, exactly like the periodic pass always
    // re-arms its schedule whether or not it installs, §1B).
    private void PreInsertionReroute(List<VehicleRuntime> insertCandidates, double time)
    {
        if (_config!.ReroutePeriod <= 0.0 || _edgeWeights is null)
        {
            return;
        }

        var network = _network!;
        var edgeWeights = _edgeWeights;

        // Collect the sub-batch: equipped, not yet pre-insertion-rerouted, among this step's
        // due-to-insert candidates (insertCandidates is already exactly "time >= Depart, not yet
        // Inserted/Arrived" -- InsertDepartingVehicles' own header comment). A vehicle stays a
        // candidate across multiple steps if insertion keeps failing (gap/leader checks), but this
        // guard means it is only ever routed here ONCE, on the first step it becomes eligible.
        var batch = _preInsertRerouteBatchScratch;
        batch.Clear();
        foreach (var v in insertCandidates)
        {
            if (!v.RerouteEquipped || v.PreInsertionRerouteDone)
            {
                continue;
            }

            batch.Add(v);
        }

        if (batch.Count == 0)
        {
            return;
        }

        _router ??= new NetworkRouter(network);
        var router = _router;

        // PARALLEL fan-out (§4, mirrors UpdatePeriodicReroutes): each vehicle's router call is a
        // PURE read of the frozen edge-weight snapshot into its OWN scratch slot -- no shared
        // writes, so the result is independent of thread scheduling/order (parallel is
        // bit-identical to serial). Source = the vehicle's ORIGINAL (as-yet uninstalled) route's
        // first edge (its fixed departure edge -- never changes here, only what comes after it);
        // destination = that same route's last edge (unchanged, per §11's design).
        if (_preInsertRerouteCandidateScratch.Length < batch.Count)
        {
            _preInsertRerouteCandidateScratch = new IReadOnlyList<string>?[batch.Count];
        }

        var candidates = _preInsertRerouteCandidateScratch;
        var useAStar = string.Equals(_config.RoutingAlgorithm, "astar", StringComparison.Ordinal);
        System.Threading.Tasks.Parallel.For(0, batch.Count, _parallelOptions, i =>
        {
            var v = batch[i];
            var originalEdges = _routesById[v.Def.RouteId].Edges;
            var currentEdge = originalEdges[0];
            var destEdge = originalEdges[^1];
            var vehicleMaxSpeed = v.VType.MaxSpeed * v.SpeedFactor;
            double EdgeEffort(string edgeId) => edgeWeights.Effort(edgeId, vehicleMaxSpeed);

            candidates[i] = useAStar
                ? router.RouteAStar(currentEdge, destEdge, EdgeEffort)
                : router.Route(currentEdge, destEdge, EdgeEffort);
        });

        // SERIAL apply (§4): mark done regardless of outcome (this vehicle's ONE pre-insertion
        // attempt is spent either way); install iff the candidate is non-empty/structurally valid
        // and NOT identical to the vehicle's current (still-original, since it has not yet
        // installed anything) full route edge list -- the same identical-list short-circuit §1B
        // uses, just compared against the ORIGINAL route's edges directly (a not-yet-inserted
        // vehicle has no lane-sequence slice for CurrentRemainingRouteEdges to walk).
        for (var i = 0; i < batch.Count; i++)
        {
            var v = batch[i];
            v.PreInsertionRerouteDone = true;

            var candidate = candidates[i];
            if (candidate is null || candidate.Count == 0)
            {
                // Structural failure (unreachable destination) -- leave the vehicle on its
                // original route, exactly like the periodic pass's own "no alternate exists" arm.
                continue;
            }

            var originalEdges = _routesById[v.Def.RouteId].Edges;
            if (candidate.SequenceEqual(originalEdges))
            {
                continue; // §1B: identical edge list -- short-circuit, no new route object
            }

            // Reuse the SAME per-entity route slot as the periodic pass (§0.5.3) -- never a second
            // id per vehicle, and harmless to overwrite again later if/when the periodic pass also
            // fires for this vehicle. No CommandBuffer/_laneSeqPool splice needed here (unlike
            // UpdatePeriodicReroutes): this vehicle has not been inserted yet, so it has no
            // existing lane sequence to replace -- InsertDepartingVehicles/TryInsertOnLane (which
            // now resolve routes via EffectiveRouteId, not v.Def.RouteId) will build its lane
            // sequence fresh from this new route when they place it later in this same call.
            RegisterPeriodicReroute(v, candidate);
        }
    }

    // P1E-4 (§1C/§3, seam #2): end-of-step edge-weight smoothing update. MUST run strictly AFTER
    // ExecuteMoves/DecideSpeedGainChanges settle this step's positions (see AdvanceOneStep's own
    // placement comment) -- this is the temporal analog of SUMO's waitForAll barrier (§8 risk 1):
    // a reroute always reads the PREVIOUS step's fully-settled weights, never a mid-write
    // snapshot. Inert-when-disabled exactly like UpdatePeriodicReroutes above.
    private void UpdateRerouteEdgeWeights(double time, double dt)
    {
        if (_config!.ReroutePeriod <= 0.0 || _edgeWeights is null)
        {
            return;
        }

        var nextTime = time + dt;
        if (nextTime < _lastAdaptationTime + _config.RerouteAdaptationInterval - 1e-9)
        {
            return; // not due this step yet
        }

        var edgeWeights = _edgeWeights;
        var network = _network!;

        // Latch trigger (§8 risk 2 -- "a vehicle being on the edge THIS step", never reset): mark
        // every edge currently carrying an active vehicle as delayed (MarkDelayed is itself an
        // idempotent no-op once latched). In the SAME fixed-order pass (ActiveVehicles() walks
        // `_vehicles` in EntityIndex order -- §4 "deterministic, fixed per-edge order"), accumulate
        // each such edge's occupant speeds for the mean-speed sample below.
        var sums = _rerouteEdgeSpeedSumScratch;
        var counts = _rerouteEdgeSpeedCountScratch;
        sums.Clear();
        counts.Clear();
        foreach (var v in ActiveVehicles())
        {
            var edgeId = network.LanesByHandle[v.LaneHandle].EdgeId;
            edgeWeights.MarkDelayed(edgeId);
            sums[edgeId] = sums.TryGetValue(edgeId, out var s) ? s + v.Kinematics.Speed : v.Kinematics.Speed;
            counts[edgeId] = counts.TryGetValue(edgeId, out var c) ? c + 1 : 1;
        }

        // MSEdge::getMeanSpeed(): mean of occupants' speeds, or the edge's free-flow (speed-limit)
        // seed when empty -- sampled once per delayed edge inside RerouteEdgeWeights.Update's own
        // fixed _edgeOrder walk.
        edgeWeights.Update(edgeId => counts.TryGetValue(edgeId, out var c) && c > 0
            ? sums[edgeId] / c
            : edgeWeights.FreeFlowSpeed(edgeId));

        _lastAdaptationTime = nextTime;
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
        if (ShouldParallelizePlan())
        {
            // L1: per-index Parallel.For over the DENSE active list (built once per step in
            // AdvanceOneStep), not 0.._vehicles.Count. The old sparse scan dispatched over every
            // not-yet-departed/arrived index and touched its scattered object header only to skip it;
            // the dense list drops that ~40% bandwidth waste. A chunked range partitioner over the
            // sparse list was tried and reverted earlier (it load-imbalanced on the gaps); the dense
            // list removes the gaps, but per-index work-stealing is retained here to balance the
            // still-uneven PER-VEHICLE cost (a near-junction vehicle with many foes costs far more
            // than a free-flowing one). Each iteration writes only its own v.Intent (race-free, see
            // UseParallelPlan's header); byte-identical to serial.
            var active = _activeIndices;
            var vehicles = _vehicles;

            // SPATIAL-OPT probe: iterate `_packed` in (lane,pos) order (contiguous chunks = spatial
            // regions per thread) and thread each ego's packed slot so LeaderFollowSpeedConstraint
            // reads its same-lane leader from the adjacent packed slot (sequential) rather than a
            // random foe-object deref. Byte-identical (same leader, same field values, order-
            // independent per-ego writes). Every OTHER constraint still uses `neighbors` unchanged.
            if (SpatialPlan)
            {
                var packed = _packed;
                var n = _packedCount;
                System.Threading.Tasks.Parallel.For(0, n, _parallelOptions, i =>
                {
                    var v = vehicles[packed[i].EntityIndex];
                    if (v.ReuseIntent)
                    {
                        return;
                    }

                    v.Intent = ComputeMoveIntent(v, neighbors, time, prePass: false, packedEgoSlot: i);
                });
                return;
            }

            // Domain decomposition: one task per spatial REGION (dynamic scheduling balances uneven
            // region occupancy). Each task processes only its region's vehicles, so a worker's working
            // set (its vehicles + their mostly-in-region leaders) stays small/L2-resident. Byte-
            // identical (per-ego-only writes, order-independent). See RegionPlan's header.
            if (RegionPlan)
            {
                System.Threading.Tasks.Parallel.For(0, _regionCount, _parallelOptions, r =>
                {
                    var list = _regionActive[r];
                    for (var idx = 0; idx < list.Count; idx++)
                    {
                        var v = vehicles[list[idx]];
                        if (v.ReuseIntent)
                        {
                            continue;
                        }

                        v.Intent = ComputeMoveIntent(v, neighbors, time);
                    }
                });
                return;
            }

            System.Threading.Tasks.Parallel.For(0, active.Count, _parallelOptions, k =>
            {
                var v = vehicles[active[k]];
                if (v.ReuseIntent)
                {
                    // ReuseIntent: the willPass pre-pass already computed this vehicle's final Intent
                    // (fusion-eligible, no crossing-yield relax) -- reuse it verbatim, skip the
                    // redundant recompute. See PrePlanVehicle / FusionEligible. (Inserted && !Arrived
                    // is guaranteed by the dense active list, so it is no longer re-checked here.)
                    return;
                }

                v.Intent = ComputeMoveIntent(v, neighbors, time);
            });
            return;
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            if (v.ReuseIntent)
            {
                continue; // fused: keep the pre-pass Intent (see PrePlanVehicle)
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
    // C4-viii: `prePass` selects the willPass PRE-PASS mode (Engine.ComputeWillPass). It computes each
    // vehicle's planned vNext WITHOUT the approaching-foe willPass refinement (JunctionYieldConstraint's
    // crossing arm keeps its blanket yield, i.e. the pre-C4-viii behaviour) and must leave NO side
    // effect on `v`: the LastActionTime record, the IDMM LevelOfService update, and the dawdle RngState
    // advance are all suppressed/copied so the pre-pass is a pure read whose only observable output is
    // its caller's `v.WillPass = ...` write. With prePass=false (the normal PlanMovements call) this is
    // byte-identical to the pre-C4-viii method except the crossing arm now additionally reads foe.WillPass.
    // SPATIAL-OPT probe: `packedEgoSlot` is this vehicle's slot in `_packed` when PlanMovements takes
    // the spatial branch (else -1). When >= 0, LeaderFollowSpeedConstraint reads the same-lane leader
    // from the packed array (sequential) via `_leaderSlotByPacked[packedEgoSlot]`; otherwise it uses
    // the neighbor query. Byte-identical either way. Every other constraint ignores it.
    private MoveIntent ComputeMoveIntent(VehicleRuntime v, LaneNeighborQuery neighbors, double time, bool prePass = false, int packedEgoSlot = -1)
    {
        // D2: hot per-vehicle, per-step lookup -- handle-indexed array instead of a string hash.
        var lane = _network!.LanesByHandle[v.LaneHandle];
        // P2-G Bug-3 (generalized): reset the red-held flag each plan; RedLightConstraint (called
        // below in the constraint chain) re-sets it iff this vehicle is stopping for a red this step.
        v.HeldByRedThisStep = false;
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
                    // B6: hold the current lateral offset on a non-action (held) step rather than
                    // snapping to centre (0 for every lane-centred vehicle -> byte-identical).
                    LatOffset = v.Kinematics.LatOffset,
                    StopUpdate = null,
                };
            }

            // Action step: re-plan below, and record it (this vehicle's own field only -- the same
            // per-ego plan-phase write pattern as RngState/LevelOfService, parallel-safe).
            // C4-viii: the willPass pre-pass must not advance the action-step clock -- it only computes
            // the vNext this step WOULD plan; the real PlanMovements call records LastActionTime. (With
            // actionStepLength == dt this branch never runs anyway.)
            if (!prePass)
            {
                v.LastActionTime = time;
            }
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
        vPos = Math.Min(vPos, LeaderFollowSpeedConstraint(v, neighbors, dt, time, laneVehicleMaxSpeed, packedEgoSlot));

        // Cross-junction leader following: car-follow a slow leader that has already crossed onto a
        // downstream lane, while ego is still on its approach (the same-lane constraint above cannot
        // see it). +infinity unless a close downstream leader exists on ego's route path -- inert for
        // every scenario without one. See CrossJunctionLeaderConstraint's own header comment.
        vPos = Math.Min(vPos, CrossJunctionLeaderConstraint(v, neighbors, dt, time, laneVehicleMaxSpeed));

        // Desired free-flow speed (MSLane::getVehicleMaxSpeed): lane speed limit adapted
        // by this vehicle's speedFactor, capped by its vType maxSpeed. C11-i: for Krauss this
        // stays the plain laneVehicleMaxSpeed value (byte-identical -- MSCFModel_Krauss never
        // overrides freeSpeed and our simplified single-lane engine never calls the base
        // MSCFModel::freeSpeed braking-curve formula for this term either way, matching every
        // pre-C11 rung exactly); for IDM this routes through IdmModel.FreeSpeed
        // (MSCFModel_IDM.cpp:77-100) -- see FreeFlowDesiredSpeedConstraint's own header comment.
        vPos = Math.Min(vPos, FreeFlowDesiredSpeedConstraint(v, laneVehicleMaxSpeed, dt));

        // C4-iii (successive-lane speed limit): FreeFlowDesiredSpeedConstraint above caps by the
        // CURRENT lane only; MSVehicle::planMoveInternal additionally caps the free-flow speed so
        // the vehicle never enters an UPCOMING lane on its route faster than that lane permits
        // (MSVehicle.cpp:2896 per-lane `va = MAX2(freeSpeed(getSpeed(), seen, laneMaxV),
        // vMinComfortable); v = MIN2(va, v)`). Non-binding (+infinity) unless a slower lane lies
        // within braking distance ahead -- the first scenario to exercise it on-path is the
        // roundabout (32), whose curved internal ring lanes drop the speed limit.
        vPos = Math.Min(vPos, SuccessiveLaneSpeedConstraint(v, lane, dt));

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

        // Rail signal (rung R4): MSRailSignal's block-based hold -- a train approaching a rail signal
        // whose forward block's opposing (bidi) lane is occupied by another train brakes to a stop at
        // the signal until that block clears. +infinity (non-binding) when this lane has no rail
        // signal or its conflict lanes are clear (green). Inert for every scenario with no
        // rail_signal junction (_railSignalConflictLaneHandles null). See RailSignalConstraint.
        vPos = Math.Min(vPos, RailSignalConstraint(v, lane, dt, actionStepLengthSecs, laneVehicleMaxSpeed));

        // Rail crossing (rung R5): a road vehicle yields to a train at a level crossing -- brakes to
        // the crossing's stop line while it is closed (not green). +infinity (non-binding) when this
        // lane is not a controlled crossing approach or the crossing is open. Inert for every net
        // with no rail_crossing junction (_railCrossingByRoadLaneHandle null). See RailCrossingConstraint.
        vPos = Math.Min(vPos, RailCrossingConstraint(v, lane, dt, actionStepLengthSecs, laneVehicleMaxSpeed));

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
        vPos = Math.Min(vPos, JunctionYieldConstraint(v, ActiveVehicles(), time, dt, actionStepLengthSecs, laneVehicleMaxSpeed, prePass));

        // C5 (keepClear / don't-block-the-box): MSVehicle::checkRewindLinkLanes' downstream
        // available-space accounting. A vehicle approaching a junction whose EXIT is jammed (a
        // stopped vehicle downstream, no room for ego) must stop at the junction ENTRY rather than
        // creep onto the internal lane and block cross traffic. +infinity (non-binding) unless a
        // stopped vehicle is found on ego's downstream exit chain with leftSpace < 0 -- so every
        // existing (jam-free) scenario is untouched.
        vPos = Math.Min(vPos, KeepClearConstraint(v, ActiveVehicles(), dt, actionStepLengthSecs, laneVehicleMaxSpeed));

        // B1: external obstacle (DESIGN.md "Two futures" -- a live, non-SUMO input, not a
        // ported SUMO code path). Modeled as one more virtual stopped leader reusing the same
        // KraussModel.FollowSpeed leader car-following formula as LeaderFollowSpeedConstraint
        // above -- the multi-constraint reducer does not care where a constraint's speed came
        // from. +infinity (non-binding) whenever _obstacles is empty or none is active/ahead
        // on this lane -- this is the inert-when-absent guard: an empty store makes this a
        // no-op Min term, leaving every existing (obstacle-free) parity scenario untouched.
        vPos = Math.Min(vPos, ObstacleConstraint(v, time, laneVehicleMaxSpeed));

        // Cross-regime bridge (Direction B longitudinal safety): brake for a crowd agent ego is still
        // laterally overlapping -- the "stop for a pedestrian you can't swerve clear of" net. +Infinity
        // (inert) unless a coupling has attached a CrowdSource, so byte-identical for every golden.
        vPos = Math.Min(vPos, CrowdLongitudinalConstraint(v, time, laneVehicleMaxSpeed));

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
        // C4-viii: in the willPass pre-pass the Krauss dawdle draw must NOT advance this vehicle's real
        // RngState (it would desync the sigma>0 stream the real PlanMovements call then reads); advance
        // a throwaway copy instead. Byte-identical for sigma==0 (no draw is taken) and non-Krauss models.
        // R6: the Rail traction model's base finalizeSpeed (bounded by its own min/maxNextSpeed);
        // no dawdle (MSCFModel_Rail does not override patchSpeedBeforeLC), so it takes no RngState.
        var newSpeed = v.VType.CarFollowModel == "Rail"
            ? RailModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs)
            : v.VType.CarFollowModel is "IDM" or "ACC" or "CACC" or "IDMM"
            ? IdmModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs)
            : prePass
                ? FinalizeKraussPrePass(v, vPos, vStop, laneVehicleMaxSpeed, dt, actionStepLengthSecs)
                : KraussModel.FinalizeSpeed(v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs, ref v.RngState);

        // C11-iv: MSCFModel_IDM.cpp:69-72's `if (myAdaptationFactor != 1.)` levelOfService update,
        // applied HERE (by the caller) right after `newSpeed` (== the vendored source's own
        // `vNext`) is computed -- see IdmModel.FinalizeSpeed's own C11-iv comment for why this
        // update lives outside that shared function body. Per-ego plan-phase mutation of `v`'s
        // OWN field, the exact same pattern C1's RngState / C11-ii's AccControlMode / C11-iii's
        // CaccControlMode already establish as parallel-safe (Engine.UseParallelPlan's own
        // argument) -- never another vehicle's state, never a MoveIntent field.
        // C4-viii: suppressed in the pre-pass -- it must leave levelOfService untouched so the real
        // PlanMovements call performs the single authoritative update (inert for non-IDMM vTypes).
        if (!prePass && v.VType.CarFollowModel == "IDMM")
        {
            v.LevelOfService = IdmmModel.UpdateLevelOfService(v.LevelOfService, newSpeed, laneVehicleMaxSpeed, dt);
        }

        // Rung ER3 (give-way): detect an approaching blue-light emergency vehicle from the frozen
        // start-of-step snapshot and form a "clear the way" intent (which lane edge to vacate).
        // Real pass only -- the willPass pre-pass leaves GiveWaySide untouched so it stays
        // side-effect-free. Inert (0, no scan) whenever the scenario has no bluelight EV. Written
        // to the vehicle's OWN field, the same parallel-safe per-ego plan-phase write as
        // LevelOfService/WillPass; consumed by the ER4/ER5 execution arms and exported for tests.
        if (!prePass)
        {
            var (side, evSameLane) = DetectGiveWay(v, lane, time);
            v.GiveWaySide = side;
            v.GiveWayEvSameLane = evSameLane;

            // Rung OV1: form an opposite-direction overtake intent (held up behind a slower leader,
            // oncoming lane clear ahead). Real pass only; inert (false, no scan) when no vType has
            // lcOpposite. Written to the ego's own field, read by the OV2/OV3 arms and exported.
            //
            // Rung D2 (return-gap enforcement): the raw held-up decision drops the instant ego noses
            // ahead of the leader (GetLeader no longer returns it), which would recenter ego right in
            // front of the leader. So once ego is committed and has passed the leader, KEEP it spilled
            // until the re-entry gap to that just-passed leader is safe (OvertakeReturnGapSafe). An
            // ABORT (gap acceptance refused while ego is still BEHIND the leader) is NOT a return, so
            // OvertakeReturnGapSafe returns true there and ego recenters as before. Inert for every
            // non-lcOpposite vehicle (heldUp false and OvertakeActive already false -> the else arm).
            var heldUp = DetectOvertake(v, lane, neighbors, out var overtakeLeaderIndex);
            if (heldUp)
            {
                v.OvertakeActive = true;
                v.OvertakePassedLeaderIndex = overtakeLeaderIndex;
            }
            else if (v.OvertakeActive && v.OvertakePassedLeaderIndex >= 0
                && !OvertakeReturnGapSafe(v, v.OvertakePassedLeaderIndex, dt))
            {
                v.OvertakeActive = true; // passed the leader but not yet a safe gap ahead -> stay spilled
            }
            else
            {
                v.OvertakeActive = false;
                v.OvertakePassedLeaderIndex = -1;
            }

            // Rung OV4: the mirror -- an oncoming driver forms a "make room" intent when a spilled
            // overtaker is closing head-on down its bidi lane, and drifts to its own outer edge. Real
            // pass only; inert (false, no scan) when no vType has lcOpposite. Reads only the frozen
            // snapshot (the overtaker's already-committed LatOffset), so it never depends on whether
            // the overtaker has been planned yet this step -- parallel-safe like OvertakeActive.
            v.CooperativeShift = DetectCooperativeShift(v, lane);
        }

        // GAP-3 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §3): true iff this vehicle IS PARKED after this
        // step's stop-transition is applied -- i.e. its front stop (a) is a parkingArea stop
        // (StopRuntime.IsParking) and (b) is Reached and NOT resuming this step. Read from
        // `stopUpdate` (this SAME call's ProcessNextStop result), not the stale pre-step
        // v.IsParked, so the lateral offset flips the SAME step Engine.ExecuteMoves' stop-transition
        // apply block flips StopRuntime.Reached (see that block's own GAP-3 comment) -- exactly
        // matching scenario 48's golden (parked lateral offset appears one step after insertion,
        // not at t=0; disappears the step the vehicle resumes). `GetStops(v)!.Peek()` is the SAME
        // front-of-queue read ProcessNextStop above just examined -- Plan never mutates the stop
        // queue, so this is a consistent, race-free re-read.
        var isParkedAfterStep = stopUpdate is { Resume: false, Reached: true } && GetStops(v)!.Peek().IsParking;

        return new MoveIntent
        {
            NewSpeed = newSpeed,
            // B6: emergency lateral evasion. Only in the real pass (the willPass pre-pass keeps 0 so it
            // stays side-effect-free); ComputeLateralEvasion returns 0 for every vehicle with no
            // dodgeable obstacle in range, so this is inert wherever no lateral obstacle is present.
            // ER5 additionally routes an ER3 give-way intent through the same lateral-drift primitive.
            // Phase 2 (P2.3): when the sublane model is active, the SUMO sublane lateral driver
            // (ComputeSublaneLateral) replaces the external-agent evasion path -- gated on _sublane,
            // so every phase-1 scenario keeps exactly the ComputeLateralEvasion path below.
            // GAP-3: a parked vehicle short-circuits BOTH the sublane and evasion paths -- it is off
            // the running lane at a fixed bay offset, not doing lateral car-following/evasion.
            // Gated on isParkedAfterStep (default false, only true for a resolved
            // `<stop parkingArea>`), so byte-identical for every scenario without one.
            LatOffset = prePass ? 0.0
                : isParkedAfterStep ? ParkingArea.LateralParkOffset(lane.Width)
                : _sublane ? (LanelessRvo ? ComputeRvoLateral(v, lane, neighbors, time, dt)
                                          : ComputeSublaneLateral(v, lane, dt))
                : ComputeLateralEvasion(v, lane, neighbors, time, dt),
            StopUpdate = stopUpdate,
        };
    }

    // Rung ER3 (give-way) DETECTION. Per-ego, reads only the frozen start-of-step snapshot (each
    // other vehicle's start-of-step lane/pos/speed/vType), writes nothing -- the caller stores the
    // result in this vehicle's own GiveWaySide field. Returns the side of the lane this vehicle
    // should vacate to clear a path for an approaching blue-light emergency vehicle: 0 = none,
    // -1 = clear toward the right edge, +1 = clear toward the left edge.
    //
    // Detection (our adaptation of SUMO's MSDevice_Bluelight, which is a device that PUSHES a
    // preferred lateral alignment onto neighbours -- incompatible with the parallel-safe ECS
    // plan/execute contract, so per CLAUDE.md rule 4 we invert it to a per-ego pull, like B6):
    // ego clears the way iff an ACTIVE emergency vehicle (VType.HasBluelight) is on ego's edge,
    // at/behind ego's front, and within GiveWayReactionDist behind it -- the "siren approaching
    // from behind" case. (An EV ahead of ego is not something ego needs to clear for.)
    //
    // Side rule, matching SUMO's rescue-lane alignment (MSDevice_Bluelight.cpp:189-192: align RIGHT
    // unless on the leftmost lane, then LEFT): the leftmost lane of a MULTI-lane road (a left-edge
    // lane that still has a lane to its right) vacates toward the LEFT edge; every other lane --
    // rightmost, middle, and single-lane -- vacates toward the RIGHT edge. This opens a rescue
    // corridor down the middle on multi-lane roads, and pulls a lone vehicle to the right edge on a
    // single lane (the case SUMO's lane-based rescue cannot form at all -- ER5's enhancement).
    //
    // Inert: returns (0, false) immediately when the scenario has no bluelight EV (_anyBluelight
    // false), or when the vehicle IS itself a bluelight EV (an EV never gives way to another).
    // Also reports (ER4) whether the qualifying EV is in ego's OWN lane -- so the multi-lane
    // execution arm vacates that lane rather than merely drifting to its edge.
    private (int Side, bool EvSameLane) DetectGiveWay(VehicleRuntime v, Lane lane, double time)
    {
        if (!_anyBluelight || v.VType.HasBluelight)
        {
            return (0, false);
        }

        var egoFront = v.Kinematics.Pos;
        var reacting = false;
        var evSameLane = false;
        foreach (var ev in ActiveVehicles())
        {
            if (!ev.VType.HasBluelight || ev.EntityIndex == v.EntityIndex)
            {
                continue;
            }

            // Handle comparison (int) rather than a LaneId string compare, and the same-edge test
            // via the handle-indexed lane table rather than a string-keyed LanesById hash lookup --
            // the D2 hot-path convention (this scan runs per vehicle per step whenever a bluelight
            // EV is present). The only string touched is the final EdgeId compare, unavoidable
            // without edge handles and how the rest of the engine compares edges too.
            var sameLane = ev.LaneHandle == lane.Handle;
            if (!sameLane && _network!.LanesByHandle[ev.LaneHandle].EdgeId != lane.EdgeId)
            {
                continue;
            }

            // EV must be at or behind ego's front (approaching from behind) and within the siren
            // reaction range. Longitudinal comparison is on the shared edge's arc-length `pos`.
            var behind = egoFront - ev.Kinematics.Pos;
            if (behind >= 0.0 && behind <= GiveWayReactionDist)
            {
                reacting = true;
                // OR across all qualifying EVs -> order-independent (a boolean "does any in-range
                // EV share ego's lane"), so the result never depends on ActiveVehicles order.
                evSameLane |= sameLane;
            }
        }

        if (!reacting)
        {
            return (0, false);
        }

        // SUMO's align-RIGHT-unless-leftmost rule (see method header).
        var isLeftmostOfMultiLane = lane.LeftNeighbor < 0 && lane.RightNeighbor >= 0;
        return (isLeftmostOfMultiLane ? +1 : -1, evSameLane);
    }

    // Rung ER5: the lateral offset that pulls ego fully to the lane edge on its give-way side,
    // leaving the maximum gap on the other side for the EV to pass. LatOffset convention is
    // positive = LEFT of centre, so a side of -1 (clear right) targets the right edge (negative)
    // and +1 (clear left) targets the left edge (positive). The magnitude places ego's OUTER edge
    // on the lane boundary: |offset| = laneHalfWidth - egoHalfWidth (clamped at 0 for an ego wider
    // than its lane, which then simply stays centred).
    private static double GiveWayEdgeTarget(VehicleRuntime v, Lane lane)
    {
        var margin = Math.Max(0.0, lane.Width / 2.0 - v.VType.Width / 2.0);
        return v.GiveWaySide < 0 ? -margin : margin;
    }

    // Rung OV1 (opposite-direction overtaking) DETECTION. Per-ego, reads only the frozen
    // start-of-step snapshot + immutable network, writes nothing (the caller stores the result in
    // this vehicle's own OvertakeActive). Returns true iff ego -- a vType that ALLOWS opposite
    // overtaking (lcOpposite) -- is HELD UP behind a slower same-lane leader AND the oncoming
    // (opposite-direction) lane is CLEAR at least OvertakeClearAheadDist ahead in ego's travel
    // direction. This is the OV1 decision; OV2 replaces the fixed clear-ahead distance with a
    // closing-speed / time-to-collision gap acceptance, and OV3 executes the pass.
    //
    // Inert: returns false immediately when the scenario has no lcOpposite vType (_anyLcOpposite
    // false), when ego is not lcOpposite, or when ego's lane has no opposite (bidi) partner.
    private bool DetectOvertake(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, out int leaderIndex)
    {
        leaderIndex = -1;
        if (!_anyLcOpposite || !v.VType.LcOpposite)
        {
            return false;
        }

        var oppLaneId = _network!.TryGetBidiLaneId(v.LaneId);
        if (oppLaneId is null)
        {
            return false;
        }

        // Held up? A same-lane leader meaningfully slower than ego's own free-flow speed and close
        // enough ahead to be worth passing.
        var leader = neighbors.GetLeader(v);
        if (leader is null)
        {
            return false;
        }

        var egoFreeSpeed = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);
        var leaderGap = leader.Kinematics.Pos - leader.VType.Length - v.Kinematics.Pos;
        if (leader.Kinematics.Speed >= egoFreeSpeed * OvertakeLeaderSlowFraction
            || leaderGap > OvertakeLeaderMaxGap)
        {
            return false;
        }

        // D2: remember the leader we are held up behind, so once we nose ahead of it (GetLeader stops
        // returning it) the caller can keep us spilled until the re-entry gap to it is safe.
        leaderIndex = leader.EntityIndex;

        // Opposite lane clear enough? Map ego and each oncoming vehicle to a world coordinate and
        // measure the head-on gap along ego's travel direction. (Straight-road model: ego's forward
        // direction is the sign of increasing x along its lane shape -- OV fixtures are E-W.)
        var (egoX, _, _) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, 0.0);
        var egoDirX = Math.Sign(lane.Shape[^1].X - lane.Shape[0].X);
        if (egoDirX == 0)
        {
            egoDirX = 1;
        }

        var oppLane = _network.LanesById[oppLaneId];
        var nearestAhead = double.PositiveInfinity;
        var nearestOncomingSpeed = 0.0;
        VehicleRuntime? nearestOncoming = null;
        foreach (var o in ActiveVehicles())
        {
            if (o.LaneId != oppLaneId)
            {
                continue;
            }

            var (ox, _, _) = LaneGeometry.PositionAtOffset(oppLane.Shape, o.Kinematics.Pos, 0.0);
            var aheadDist = (ox - egoX) * egoDirX; // > 0 == ahead of ego in its travel direction
            if (aheadDist > 0.0 && aheadDist < nearestAhead)
            {
                nearestAhead = aheadDist;
                nearestOncomingSpeed = o.Kinematics.Speed;
                nearestOncoming = o;
            }
        }

        // OV2 gap acceptance: commit only if the nearest oncoming is farther than the distance the
        // manoeuvre needs. To PASS the leader, ego must gain (relative to the leader) the gap to it +
        // both bodies + a re-entry gap; at its speed advantage that takes overtakeTime seconds, during
        // which ego and the oncoming close at (egoFreeSpeed + oncomingSpeed). Required head-on clear =
        // that closing distance + a safety buffer, floored at OvertakeMinClearDist. If ego has no
        // speed advantage over the leader it can never complete the pass -> refuse.
        var relAdvantage = egoFreeSpeed - leader.Kinematics.Speed;
        if (relAdvantage <= KraussModel.NumericalEps)
        {
            return false;
        }

        var overtakeDist = Math.Max(0.0, leaderGap) + leader.VType.Length + v.VType.Length + 2.0 * v.VType.MinGap;
        var overtakeTime = overtakeDist / relAdvantage;
        var requiredClear = Math.Max(
            OvertakeMinClearDist,
            (egoFreeSpeed + nearestOncomingSpeed) * overtakeTime + OvertakeSafetyGap);

        if (nearestAhead > requiredClear)
        {
            return true;
        }

        // Rung D3 (coupled OV2/OV4 -- cooperative side-by-side pass). The conservative rule above
        // requires the pass to complete AND return before the head-on arrives, so the overtaker and
        // oncoming never share a longitudinal position while ego is spilled. But if the nearest
        // oncoming will COOPERATE -- a normal (non-spilled) vehicle that OV4 pulls to its own outer
        // edge -- and the road is wide enough that ego's spill and the oncoming's shift leave a safe
        // lateral corridor (CooperativeSideBySideSafe), the two can pass SIDE BY SIDE. Then ego only
        // needs enough head-on room to reach lateral clearance before contact (OvertakeMinClearDist),
        // not the full complete-and-return distance. This is the only place OV2 bets on another
        // vehicle's cooperation; it is safe because (a) the geometry check refuses on any lane too
        // narrow for the shifted pass (so scenario 57 and every existing OV fixture are unaffected --
        // their clearance is negative), and (b) an oncoming that is itself spilled toward ego (an
        // opposing overtaker) fails the not-spilled check, so ego never commits into a head-on spill.
        if (nearestOncoming is not null
            && nearestAhead > OvertakeMinClearDist
            && CooperativeSideBySideSafe(v, lane, oppLane, nearestOncoming))
        {
            return true;
        }

        return false;
    }

    // Rung D3: is a cooperative side-by-side pass geometrically safe against this oncoming? True iff
    // (a) the oncoming will cooperate -- it is not itself spilled toward ego across the centre line
    // (its committed LatOffset, read from the frozen snapshot, is at or below the spill threshold) --
    // and (b) ego's overtake spill and the oncoming's OV4 outer-edge shift leave a non-negative
    // lateral corridor with margin. The corridor width is, in the straight-road model,
    //   laneSeparation + oncomingCoopShift - egoSpill - (egoHalfWidth + oncomingHalfWidth)
    // where laneSeparation is the gap between the two lane centres, egoSpill = Width + OvertakeSpillGap
    // (the OV3 spill), and oncomingCoopShift = oppLaneHalfWidth - oncomingHalfWidth (the OV4 drift to
    // the outer edge). On a lane too narrow for the shifted pass this is negative, so this returns
    // false and OV2 stays fully conservative (byte-identical to pre-D3 for every existing scenario).
    private static bool CooperativeSideBySideSafe(VehicleRuntime ego, Lane egoLane, Lane oppLane, VehicleRuntime oncoming)
    {
        if (oncoming.Kinematics.LatOffset > CooperativeShiftSpillThreshold)
        {
            return false; // the oncoming is itself overtaking (spilled toward ego) -> will not cooperate
        }

        var laneSeparation = Math.Abs(oppLane.Shape[0].Y - egoLane.Shape[0].Y);
        var egoSpill = ego.VType.Width + OvertakeSpillGap;
        var oncomingCoopShift = Math.Max(0.0, oppLane.Width / 2.0 - oncoming.VType.Width / 2.0);
        var corridor = laneSeparation + oncomingCoopShift - egoSpill - (ego.VType.Width / 2.0 + oncoming.VType.Width / 2.0);
        return corridor >= CooperativeSideBySideMargin;
    }

    // Rung D2 (OV3 return-gap enforcement). After an overtaker has nosed ahead of the leader it was
    // passing (so DetectOvertake no longer reports "held up"), decide whether it may recenter yet:
    // true = safe to drop the spill, false = stay spilled a little longer. Reads only the frozen
    // start-of-step snapshot (the just-passed leader, looked up by the remembered EntityIndex).
    //
    //  - passed leader gone (arrived / off ego's lane) -> safe (nothing to cut in front of);
    //  - ego NOT actually ahead of it -> this is an ABORT, not a return, so safe (recenter/abort as
    //    before -- preserves the OV3b abort-mid-spill behaviour);
    //  - ego ahead -> require the same follower secure-gap a real lane change back into the lane would
    //    (IsTargetLaneSafe's neighFollow branch), so ego does not merge back within the leader's
    //    braking distance.
    private bool OvertakeReturnGapSafe(VehicleRuntime v, int passedLeaderIndex, double dt)
    {
        VehicleRuntime? leader = null;
        foreach (var o in ActiveVehicles())
        {
            if (o.EntityIndex == passedLeaderIndex)
            {
                leader = o;
                break;
            }
        }

        if (leader is null || leader.LaneHandle != v.LaneHandle || leader.Kinematics.Pos >= v.Kinematics.Pos)
        {
            return true;
        }

        return IsTargetLaneSafe(v, neighLead: null, neighFollow: leader, dt);
    }

    // Rung OV4 (cooperative oncoming shift) DETECTION. The mirror of ER3's give-way detection, for
    // opposite-direction overtaking: a normal oncoming driver (it need NOT itself be lcOpposite --
    // it is cooperating, not overtaking) that sees a SPILLED overtaker closing head-on down its own
    // bidi lane forms a "make room" intent, so ComputeLateralEvasion can pull it to its own outer
    // edge and widen the corridor. Per-ego, reads only the frozen start-of-step snapshot + immutable
    // network (in particular the overtaker's ALREADY-COMMITTED LatOffset, never a same-step plan
    // flag), writes nothing -- the caller stores the result in this vehicle's own CooperativeShift.
    //
    // Fires iff an opposite-lane (bidi) vehicle is (a) spilled toward ego across the centre line
    // (LatOffset > CooperativeShiftSpillThreshold -- the OV3 spill is always positive/toward the
    // oncoming lane) and (b) ahead of ego in ego's travel direction within CooperativeShiftReactionDist
    // (i.e. genuinely closing head-on, not already passed). Same straight-road world-x mapping as
    // DetectOvertake.
    //
    // Inert: returns false immediately when the scenario has no lcOpposite vType (_anyLcOpposite
    // false) or ego's lane has no opposite (bidi) partner.
    private bool DetectCooperativeShift(VehicleRuntime v, Lane lane)
    {
        if (!_anyLcOpposite)
        {
            return false;
        }

        var oppLaneId = _network!.TryGetBidiLaneId(v.LaneId);
        if (oppLaneId is null)
        {
            return false;
        }

        var (egoX, _, _) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, 0.0);
        var egoDirX = Math.Sign(lane.Shape[^1].X - lane.Shape[0].X);
        if (egoDirX == 0)
        {
            egoDirX = 1;
        }

        var oppLane = _network.LanesById[oppLaneId];
        foreach (var o in ActiveVehicles())
        {
            if (o.LaneId != oppLaneId || o.Kinematics.LatOffset <= CooperativeShiftSpillThreshold)
            {
                continue;
            }

            var (ox, _, _) = LaneGeometry.PositionAtOffset(oppLane.Shape, o.Kinematics.Pos, 0.0);
            var aheadDist = (ox - egoX) * egoDirX; // > 0 == ahead of ego in its travel direction
            if (aheadDist > 0.0 && aheadDist < CooperativeShiftReactionDist)
            {
                return true;
            }
        }

        return false;
    }

    // Rung ER4 (give-way execution, multi-lane). When a blue-light EV is approaching in ego's OWN
    // lane, ego changes to an adjacent lane to VACATE its lane for the EV, reusing the existing
    // lane-change machinery: the same post-move neighbor snapshot, the same IsTargetLaneSafe gap
    // veto, and the same CommitLaneChange command-buffer path the speed-gain/keep-right changes
    // use. Direction preference follows the give-way side (-1 -> right first, +1 -> left first),
    // falling back to the opposite side when the preferred side has no lane -- so a car pinned on
    // the leftmost lane of a 2-lane road (GiveWaySide +1, no left neighbour) still vacates to the
    // right. Returns true iff a change was committed. Does nothing (leaving ER5's within-lane drift
    // to make room) when the EV is NOT in ego's lane, when there is no adjacent lane at all
    // (single-lane road -> ER5), or when neither adjacent lane is gap-safe this step. Behavioral
    // (no golden); reached only for a vehicle with GiveWaySide != 0, i.e. never in a scenario with
    // no bluelight EV.
    private bool TryGiveWayLaneChange(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, double dt)
    {
        if (!v.GiveWayEvSameLane)
        {
            return false;
        }

        // Preferred vacate direction from the give-way side, then the opposite side as a fallback.
        var firstHandle = v.GiveWaySide < 0 ? lane.RightNeighbor : lane.LeftNeighbor;
        var secondHandle = v.GiveWaySide < 0 ? lane.LeftNeighbor : lane.RightNeighbor;

        foreach (var targetHandle in stackalloc[] { firstHandle, secondHandle })
        {
            if (targetHandle < 0)
            {
                continue;
            }

            var target = _network!.LanesByHandle[targetHandle];
            var neighLead = neighbors.GetNeighborLeader(v, targetHandle);
            var neighFollow = neighbors.GetNeighborFollower(v, targetHandle);
            if (IsTargetLaneSafe(v, neighLead, neighFollow, dt) && !IsTargetLaneOverlapped(v, targetHandle, neighbors, dt))
            {
                CommitLaneChange(v, targetHandle, target.Id);
                return true;
            }
        }

        return false;
    }

    // C4-viii: the pre-pass Krauss finalizeSpeed -- identical to KraussModel.FinalizeSpeed but advances
    // a COPY of this vehicle's dawdle RngState so the pre-pass leaves no side effect (the real
    // PlanMovements call owns the authoritative advance). For sigma==0 no draw is taken and the copy is
    // never mutated, so this is byte-identical to the direct call.
    private static double FinalizeKraussPrePass(
        VehicleRuntime v, double vPos, double vStop, double laneVehicleMaxSpeed, double dt, double actionStepLengthSecs)
    {
        var rngCopy = v.RngState;
        return KraussModel.FinalizeSpeed(
            v.Kinematics.Speed, vPos, vStop, laneVehicleMaxSpeed, v.VType, dt, actionStepLengthSecs, ref rngCopy);
    }

    // C4-viii: the willPass PRE-PASS (SUMO's MSVehicle::setApproaching, run during planMove BEFORE any
    // MSLink::opened() yield check reads a foe's willPass). For every active vehicle it computes the
    // plan-phase vNext WITHOUT the approaching-foe willPass refinement (ComputeMoveIntent(prePass:true)
    // keeps JunctionYieldConstraint's crossing arm at its blanket yield) and caches
    //   WillPass = (planned vNext > NUMERICAL_EPS_SPEED)
    // -- SUMO's `setRequest = (v > NUMERICAL_EPS_SPEED && !abortRequestAfterMinor) ||
    // leavingCurrentIntersection` (MSVehicle.cpp:2732), with the abortRequestAfterMinor visibility case
    // folding in for free (a foe braking for its own minor link has vNext ~ 0 anyway) and
    // leavingCurrentIntersection handled by the on-junction (AdaptToJunctionLeader) branch, not the
    // approaching-foe branch this bool gates. NUMERICAL_EPS_SPEED = 0.1 * NUMERICAL_EPS * TS
    // (MSVehicle.cpp:124) -- a "> 0" threshold.
    //
    // Ordering / plan-execute contract (DESIGN.md): runs on the SAME frozen start-of-step snapshot
    // PlanMovements reads (called from Run() immediately before it, after the neighbour Refill), and
    // writes ONLY each vehicle's own WillPass (ComputeMoveIntent(prePass:true) suppresses the
    // LastActionTime/LevelOfService/RngState writes). No pre-pass iteration reads another vehicle's
    // WillPass, so it is parallel-safe by PlanMovements' own argument; the subsequent PlanMovements
    // reads a fully-populated cache. Inert-when-absent: with no braking-to-stop foe at a crossing, no
    // vehicle's WillPass is ever read and trajectories are byte-identical (every committed scenario).
    private void ComputeWillPass(LaneNeighborQuery neighbors, double time)
    {
        // C4-viii perf: WillPass is read only by JunctionYieldConstraint's crossing arm, which can only
        // fire on a network that HAS junction links (internal lanes). On a junction-free net (a plain
        // highway -- e.g. the Sim.Bench workload) no vehicle's WillPass is ever read, so skip the whole
        // pre-pass. One dictionary-count check per step; the per-vehicle proximity gate
        // (WillPassRelevant) handles the far-from-junction vehicles on nets that DO have junctions.
        if (_network!.LinkByInternalLane.Count == 0)
        {
            return;
        }

        // NUMERICAL_EPS_SPEED (MSVehicle.cpp:124): 0.1 * NUMERICAL_EPS * TS, TS == the step length.
        var willPassSpeedEps = 0.1 * KraussModel.NumericalEps * _config!.StepLength;
        var dt = _config.StepLength;
        // Perf (willPass/plan fusion): snapshot the eligibility ONCE for the whole step. When set, a
        // relevant vehicle's pre-pass MoveIntent (cached into v.Intent below) is REUSED by
        // PlanMovements verbatim -- unless it took the crossing yield (v.CrossingYieldTaken), the only
        // term the real pass relaxes. Byte-identical (see FusionEligible); on the city-scale grid it
        // removes the redundant second ComputeMoveIntent for the ~2/3 of the plan that is
        // near-junction traffic. When NOT eligible, ReuseIntent stays false and the exact original
        // two-pass path runs.
        var eligible = FusionEligible;

        if (ShouldParallelizePlan())
        {
            // L1: per-index Parallel.For over the DENSE active list (built once per step in
            // AdvanceOneStep) rather than 0.._vehicles.Count -- skips dispatching over (and touching
            // the scattered header of) every not-yet-departed/arrived vehicle. Each iteration writes
            // only its own vehicle's fields (race-free); byte-identical (same active set, same order
            // basis; PrePlanVehicle keeps its own Inserted/Arrived guard for the serial path).
            var active = _activeIndices;
            var vehicles = _vehicles;

            // Domain decomposition: region-parallel willPass (same spatial working-set benefit as the
            // plan; see RegionPlan's header). Byte-identical -- PrePlanVehicle writes only each ego's
            // own fields, order-independent (ResolveRightBeforeLeftCycles below is the serial barrier).
            if (RegionPlan)
            {
                System.Threading.Tasks.Parallel.For(0, _regionCount, _parallelOptions, r =>
                {
                    var list = _regionActive[r];
                    for (var idx = 0; idx < list.Count; idx++)
                    {
                        PrePlanVehicle(vehicles[list[idx]], neighbors, time, dt, willPassSpeedEps, eligible);
                    }
                });
                ResolveRightBeforeLeftCycles(dt);
                return;
            }

            System.Threading.Tasks.Parallel.For(0, active.Count, _parallelOptions, k =>
            {
                PrePlanVehicle(vehicles[active[k]], neighbors, time, dt, willPassSpeedEps, eligible);
            });
            ResolveRightBeforeLeftCycles(dt);
            return;
        }

        foreach (var v in ActiveVehicles())
        {
            PrePlanVehicle(v, neighbors, time, dt, willPassSpeedEps, eligible);
        }

        ResolveRightBeforeLeftCycles(dt);
    }

    // C4-viii pre-pass body (willPass) + the willPass/plan fusion bookkeeping. Computes this
    // vehicle's WillPass from its pre-pass planned vNext and, when the scenario is fusion-eligible,
    // decides whether PlanMovements may REUSE the pre-pass Intent instead of recomputing it:
    //   - a vehicle too far to have its WillPass read (WillPassRelevant false) is NOT pre-planned; its
    //     Intent is computed once, in PlanMovements, exactly as before (ReuseIntent = false).
    //   - a relevant vehicle is pre-planned here (Intent cached); if it did not take the crossing yield
    //     it is reused (ReuseIntent = true), else PlanMovements recomputes it with the foe-willPass
    //     refinement. Writes only this vehicle's own fields -> parallel-safe (Engine.UseParallelPlan).
    private void PrePlanVehicle(
        VehicleRuntime v, LaneNeighborQuery neighbors, double time, double dt, double willPassSpeedEps, bool eligible)
    {
        if (!v.Inserted || v.Arrived)
        {
            return;
        }

        if (!WillPassRelevant(v, dt))
        {
            v.WillPass = true;
            v.ReuseIntent = false; // computed once, in PlanMovements (unchanged path)
            return;
        }

        v.CrossingYieldTaken = false;
        var intent = ComputeMoveIntent(v, neighbors, time, prePass: true);
        v.Intent = intent; // tentative -- reused by PlanMovements iff eligible && !CrossingYieldTaken
        // P2-G Bug-3 (generalized): a vehicle stopping for a red light this step does NOT enter its
        // junction, so it must not read as a passing foe -- force WillPass=false even though its
        // braking vNext is still > 0 (it is rolling toward the stop line). RedLightConstraint (run
        // inside the ComputeMoveIntent above) set HeldByRedThisStep on exactly the will-stop path.
        v.WillPass = intent.NewSpeed > willPassSpeedEps && !v.HeldByRedThisStep;
        // The pre-pass Intent carries LatOffset == 0 (prePass short-circuit). The real pass only
        // reproduces that when ComputeLateralEvasion returns exactly 0 -- which, under an eligible
        // scenario (no obstacle/EV/overtake trigger), holds iff the vehicle carries NO residual
        // lateral offset. A vehicle mid-recenter (LatOffset != 0 after an obstacle was removed this
        // run -- _obstacles.Count can toggle back to 0 via RemoveObstacle/ClearObstacles) would
        // otherwise SNAP to centre instead of DriftToward-ing, so exclude it from reuse. Inert for
        // every lane-centred vehicle (LatOffset always 0, the lane-mode invariant).
        v.ReuseIntent = eligible && !v.CrossingYieldTaken && v.Kinematics.LatOffset == 0.0;
    }

    // C4-viii-b (bug C: symmetric right-before-left circular-yield deadlock). SUMO's
    // MSVehicle::planMoveInternal (MSVehicle.cpp:2818-2839) detects a right-before-left deadlock -- a
    // LINKSTATE_EQUAL link whose blocker chain (getFirstApproachingFoe) wraps back to itself -- and
    // breaks it by RANDOMLY aborting one vehicle's request. The base willPass pre-pass here yields the
    // pathological all-false state for a symmetric cycle (every vehicle brakes to yield in the blanket
    // pre-pass, so WillPass=false for all, and the real-pass crossing gate's `foeYieldsThisStep`
    // then lets ALL of them enter and mutually gridlock mid-junction). This post-pass replaces SUMO's
    // RNG abort with a DETERMINISTIC, order-independent resolution (CLAUDE.md determinism: our result
    // must not depend on thread/processing order): among the vehicles approaching a junction, find the
    // directed "yields-to" response cycle(s); within each cycle select a maximal set of mutually
    // non-conflicting links GREEDILY by ascending link index (the canonical priority SUMO's link
    // ordering also biases toward) and mark their vehicles WillPass=true (pass), the rest WillPass=false
    // (yield). The real-pass gate then lets exactly the selected, non-conflicting movements cross while
    // their foes hold -- e.g. the symmetric 4-way straight cross resolves to the lower-index axis first,
    // matching SUMO's observed N-S-then-E-W order (scenarios/_diag/sym-rbl-straight).
    //
    // INERT wherever no directed response cycle exists: acyclic junctions (a priority major/minor split,
    // the ASYMMETRIC 2-vehicle right-before-left of scenario 26, any TLS whose simultaneously-green links
    // never conflict) keep their base WillPass untouched. allway_stop junctions are EXCLUDED (their
    // mutual response IS a 2-cycle, but they are governed by AllwayStopConstraint's arrival-order arm,
    // not the willPass crossing gate). Reads only the frozen start-of-step snapshot + the static
    // <request> matrix, writes only WillPass -> deterministic and parallel-safe.
    private void ResolveRightBeforeLeftCycles(double dt)
    {
        // Collect, per junction, the lead approaching vehicle on each link that is close enough to be
        // entering this step (WillPassRelevant) and not already committed onto an internal lane.
        Dictionary<Junction, Dictionary<int, VehicleRuntime>>? byJunction = null;
        foreach (var v in ActiveVehicles())
        {
            // Fresh per-step decision: clear last step's hold for EVERY active vehicle before any is
            // re-marked below, so a vehicle that was held last step but is no longer a cycle yielder
            // (its cycle dissolved, or it committed onto its internal lane) is released, never stale.
            v.JunctionCycleHold = false;

            var lane = _network!.LanesByHandle[v.LaneHandle];
            if (lane.EdgeId.Length > 0 && lane.EdgeId[0] == ':')
            {
                continue; // already on an internal lane -- entry already committed
            }

            if (!WillPassRelevant(v, dt))
            {
                continue; // too far from its next internal lane to be entering this step
            }

            if (!TryGetUpcomingJunctionLink(v, out var junction, out var egoLink))
            {
                continue;
            }

            if (junction.Type == "allway_stop" || junction.Requests.Count == 0)
            {
                continue;
            }

            // P2-G Bug-2: traffic_light junctions are EXCLUDED, exactly as allway_stop is. This resolver
            // is a deterministic stand-in for SUMO's RNG deadlock-abort, which fires ONLY for
            // LINKSTATE_EQUAL (uncontrolled, mutually-yielding right-before-left) links. At a TL junction
            // the signal program already sequences every movement -- simultaneously-green links are
            // conflict-free by construction, red links are held by the TL gate -- so no equal-priority
            // response cycle exists to break. The cycle DETECTOR below, however, reads only the static
            // <request> foe matrix (TL-state-blind); on a dense TL junction it finds the geometric 4-way
            // cycle and, via the greedy ascending-index select, marks a *green* link JunctionCycleHold=true
            // (held a full signal cycle) while a *red* link "wins" pass=true. That is the progressive
            // gridlock on short TL approaches (the synthetic_junction2 witness). Letting the TL program
            // own these links -- never overriding live signal state with a geometric tie-break -- removes
            // it. Simple TL goldens are unaffected: their sparse approaches never formed a cycle here, so
            // this path was already inert for them (verified byte-identical across all committed goldens).
            if (junction.Type == "traffic_light")
            {
                continue;
            }

            byJunction ??= new Dictionary<Junction, Dictionary<int, VehicleRuntime>>();
            if (!byJunction.TryGetValue(junction, out var links))
            {
                links = new Dictionary<int, VehicleRuntime>();
                byJunction[junction] = links;
            }

            // Keep the vehicle closest to entry on each link (largest position along its approach lane).
            if (!links.TryGetValue(egoLink.Index, out var existing)
                || v.Kinematics.Pos > existing.Kinematics.Pos)
            {
                links[egoLink.Index] = v;
            }
        }

        if (byJunction is null)
        {
            return;
        }

        foreach (var (junction, links) in byJunction)
        {
            if (links.Count < 2)
            {
                continue; // a cycle needs at least two mutually-yielding links
            }

            // request-by-index lookup for this junction's active links.
            var reqByIndex = new Dictionary<int, JunctionRequest>();
            foreach (var r in junction.Requests)
            {
                if (links.ContainsKey(r.Index))
                {
                    reqByIndex[r.Index] = r;
                }
            }

            // A link is a CYCLE member iff, following "yields-to" edges (L -> M when L responds to an
            // active foe link M), it can reach itself. Bounded DFS per active link.
            var cycleMembers = new List<int>();
            foreach (var startIdx in links.Keys)
            {
                if (ReachesSelf(startIdx, links, reqByIndex))
                {
                    cycleMembers.Add(startIdx);
                }
            }

            if (cycleMembers.Count == 0)
            {
                continue; // acyclic -- leave base WillPass untouched (inert)
            }

            // Greedy max-independent-set over the cycle members, ascending link index: select a link if
            // it conflicts (FoeWith) no already-selected link. Selected -> pass, rest -> yield.
            cycleMembers.Sort();
            var selected = new List<int>();
            foreach (var idx in cycleMembers)
            {
                var req = reqByIndex[idx];
                var conflictsSelected = false;
                foreach (var s in selected)
                {
                    if (req.FoeWith(s))
                    {
                        conflictsSelected = true;
                        break;
                    }
                }

                var pass = !conflictsSelected;
                if (pass)
                {
                    selected.Add(idx);
                }

                links[idx].WillPass = pass;
                // A yielder (pass == false) is HELD at the stop line this step -- the crossing gate's
                // foe-relative `foeYieldsThisStep` cannot hold it (its higher-priority foe in the cycle
                // is itself a yielder, WillPass=false), so mark it explicitly. The selected (pass ==
                // true) vehicles are released (hold false) and cross; the resolver re-runs next step and
                // admits the next lowest-index cycle member, staggering the movements out one at a time.
                links[idx].JunctionCycleHold = !pass;
            }
        }
    }

    // True iff `startIdx` lies on a directed cycle of the "yields-to" graph restricted to `links`
    // (L -> M when request[L] responds to active foe link M). Bounded DFS (junctions have few links).
    private static bool ReachesSelf(int startIdx, Dictionary<int, VehicleRuntime> links, Dictionary<int, JunctionRequest> reqByIndex)
    {
        var stack = new Stack<int>();
        var visited = new HashSet<int>();
        foreach (var m in links.Keys)
        {
            if (m != startIdx && reqByIndex.TryGetValue(startIdx, out var r0) && r0.RespondsTo(m))
            {
                stack.Push(m);
            }
        }

        while (stack.Count > 0)
        {
            var cur = stack.Pop();
            if (cur == startIdx)
            {
                return true;
            }

            if (!visited.Add(cur))
            {
                continue;
            }

            if (!reqByIndex.TryGetValue(cur, out var req))
            {
                continue;
            }

            foreach (var m in links.Keys)
            {
                if (m != cur && req.RespondsTo(m))
                {
                    stack.Push(m);
                }
            }
        }

        return false;
    }

    // Ego's upcoming junction link -- the first internal (':'-edge) lane in the pool at/after
    // LaneSeqIndex, mapped via LinkByInternalLane. Mirrors JunctionYieldConstraint's Step-1 scan.
    private bool TryGetUpcomingJunctionLink(VehicleRuntime v, out Junction junction, out JunctionLink egoLink)
    {
        for (var i = v.LaneSeqIndex; i < v.LaneSeqLen; i++)
        {
            var seqLaneId = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Id;
            if (_network.LinkByInternalLane.TryGetValue(seqLaneId, out var link))
            {
                junction = link.Junction;
                egoLink = link.Link;
                return true;
            }
        }

        junction = null!;
        egoLink = null!;
        return false;
    }

    // C4-viii perf: the pre-pass only needs an ACCURATE WillPass for a vehicle whose WillPass could
    // actually be READ -- i.e. one that is a foe WITHIN reservation distance of a crossing conflict lane
    // (JunctionYieldConstraint's `foeNotApproaching` gate already makes a farther foe non-blocking,
    // regardless of WillPass). A foe's distance to any conflict lane is >= its distance to its NEXT
    // upcoming internal lane, so a vehicle beyond its OWN reservation distance from its next internal
    // lane can NEVER have WillPass read. For those, ComputeWillPass sets WillPass = true (the safe
    // "will pass" value if it ever were read) and SKIPS the expensive full ComputeMoveIntent -- which is
    // almost every vehicle on a long approach / highway. This is byte-identical for every vehicle whose
    // WillPass matters (the unchanged Sim.Bench hash + green suite are the proof) and just recovers the
    // pre-pass cost for the rest. Uses the SAME reservation formula (speed -> SPEED2DIST + brakeGap) the
    // crossing arm's `foeReservationDist` uses.
    private bool WillPassRelevant(VehicleRuntime v, double dt)
    {
        var lane = _network!.LanesByHandle[v.LaneHandle];

        // On an internal lane (mid-junction) the on-junction foe path (AdaptToJunctionLeader) governs,
        // not the approaching-foe gate; compute accurately (rare) to be safe.
        if (lane.EdgeId.Length > 0 && lane.EdgeId[0] == ':')
        {
            return true;
        }

        var maxV = KraussModel.MaxNextSpeed(v.Kinematics.Speed, v.VType, dt);
        var reservationDist = KraussModel.Speed2Dist(maxV, dt)
            + KraussModel.BrakeGap(maxV, v.VType.Decel, v.VType.Tau, dt);

        // Walk the route pool forward, accumulating distance to the entry of the next INTERNAL
        // (':'-edge) lane; bail as soon as the accumulated distance passes the reservation range.
        var seen = lane.Length - v.Kinematics.Pos;
        for (var i = v.LaneSeqIndex + 1; i < v.LaneSeqLen; i++)
        {
            if (seen > reservationDist)
            {
                return false;
            }

            var poolLane = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]];
            if (poolLane.EdgeId.Length > 0 && poolLane.EdgeId[0] == ':')
            {
                return true; // next internal lane is within reservation range
            }

            seen += poolLane.Length;
        }

        return false;
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
        double levelOfService,
        // C8-iii: ballistic integration flips the Krauss safe-speed to its ballistic branch. Only
        // the Krauss arm reads it (scenario 42's vType is Krauss); IDM/ACC/CACC ballistic is out of
        // scope and those arms ignore it -- byte-identical for every non-ballistic (config.Ballistic
        // == false) call, which is every scenario but 21/42.
        bool ballistic = false)
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

        // R6: MSCFModel_Rail::followSpeed (moving-block leader model) is deferred (not exercised by
        // the single free-running train anchor); a Rail leader would fall through to Krauss here.
        return vType.CarFollowModel == "IDM"
            ? IdmModel.FollowSpeed(egoSpeed, gap, predSpeed, laneVehicleMaxSpeed, vType, dt)
            : KraussModel.FollowSpeed(egoSpeed, gap, predSpeed, predMaxDecel, vType, dt, ballistic);
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

    // C4-iii: the successive-lane free-flow speed cap -- MSVehicle::planMoveInternal's per-lane
    // loop (MSVehicle.cpp:2894-2900). For each lane on ego's route BEYOND its current one, within
    // the plan-move lookahead `dist` (MSVehicle.cpp:2238 = SPEED2DIST(maxV) + brakeGap(maxV)),
    // require the free-flow speed to be low enough that ego arrives at that lane no faster than the
    // lane allows: `va = MAX2(cfModel.freeSpeed(getSpeed(), seen, laneMaxV), vMinComfortable)`,
    // taking the running MIN. Krauss uses the base Euler freeSpeed (KraussModel.FreeSpeed); the IDM
    // family uses its own override (IdmModel.FreeSpeed), whose desSpeed is ego's CURRENT lane's max
    // speed (MSCFModel_IDM.cpp:87/92 `veh->getLane()->getVehicleMaxSpeed(veh)`). `seen` is measured
    // from ego's front (MSVehicle.cpp:2251 `myLane->getLength() - myState.myPos`), accumulating the
    // length of each traversed lane. Returns +infinity when no upcoming lane within `dist` binds --
    // i.e. every pre-C4-iii scenario, whose on-route lanes never drop the speed limit ahead of ego
    // (verified: the suite is unchanged). vMinComfortable/minNextSpeed are model-dispatched exactly
    // as MSVehicle.cpp:2191 dispatches cfModel.minNextSpeed.
    private double SuccessiveLaneSpeedConstraint(VehicleRuntime v, Lane currentLane, double dt)
    {
        if (v.LaneSeqIndex + 1 >= v.LaneSeqLen)
        {
            return double.PositiveInfinity;
        }

        var speed = v.Kinematics.Speed;
        var isIdm = v.VType.CarFollowModel is "IDM" or "ACC" or "CACC" or "IDMM";
        var currentLaneMaxV = KraussModel.LaneVehicleMaxSpeed(currentLane.Speed, v.SpeedFactor, v.VType);
        var vMinComfortable = isIdm
            ? IdmModel.MinNextSpeed(speed, v.VType, dt)
            : KraussModel.MinNextSpeed(speed, v.VType, dt);
        double? headwayTimeOverride = v.VType.CarFollowModel == "IDMM"
            ? IdmmModel.AdaptedHeadwayTime(v.VType.Tau, v.LevelOfService)
            : null;

        // MSVehicle.cpp:2238: only links within this lookahead are taken into account.
        var maxV = KraussModel.MaxNextSpeed(speed, v.VType, dt);
        var dist = KraussModel.Speed2Dist(maxV, dt) + KraussModel.BrakeGap(maxV, v.VType.Decel, headwayTime: 0.0, dt);

        var result = double.PositiveInfinity;
        var seen = currentLane.Length - v.Kinematics.Pos;
        for (var i = v.LaneSeqIndex + 1; i < v.LaneSeqLen && seen <= dist; i++)
        {
            var nextLane = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]];
            var laneMaxV = KraussModel.LaneVehicleMaxSpeed(nextLane.Speed, v.SpeedFactor, v.VType);
            var free = isIdm
                ? IdmModel.FreeSpeed(speed, seen, laneMaxV, currentLaneMaxV, v.VType, dt, headwayTimeOverride)
                : KraussModel.FreeSpeed(speed, v.VType.Decel, seen, laneMaxV, dt);
            result = Math.Min(result, Math.Max(free, vMinComfortable));
            seen += nextLane.Length;
        }

        return result;
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

        // GAP-3 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §3): MSVehicle.cpp:2477-2481 -- the generic
        // `endPos = stop.getEndPos(*this) + NUMERICAL_EPS` is OVERRIDDEN for a parkingArea stop:
        // `endPos = stop.parkingarea->getLastFreePosWithReservation(t, *this, brakePos)`, with NO
        // `+NUMERICAL_EPS` added. A plain lane stop keeps the `+eps` term (verified byte-identical
        // against scenarios 03/13/44); a parkingArea stop's braking target is `stop.EndPos - pos`
        // exactly, eps SMALLER -- which is why a moving vehicle driving into a parkingArea brakes
        // very slightly harder and settles a hair short of its lot's endPos (empirically confirmed
        // against SUMO 1.20.0: a solo vehicle approaching an otherwise-empty parkingArea lot at
        // pos=205 settles at 204.999, not 205.000 -- reproduced ONLY once this eps term is dropped
        // for IsParking stops). Gated on stop.IsParking (false for every plain lane stop), so
        // byte-identical for every pre-GAP-3 scenario.
        var newStopDist = stop.IsParking
            ? stop.EndPos - v.Kinematics.Pos
            : stop.EndPos + KraussModel.NumericalEps - v.Kinematics.Pos;
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

        // R4 (rail signal): a rail_signal-controlled connection also carries tl="<junction id>" but
        // has NO <tlLogic> (its state is computed at run time, not from a static/actuated program).
        // Those links are handled by RailSignalConstraint -- skip them here so this static-program
        // lookup never throws on a rail signal. No-op for every scenario without a rail signal.
        if (!_network.TlLogicsById.ContainsKey(connection.Tl!))
        {
            return double.PositiveInfinity;
        }

        var tlLogic = _network.TlLogicsById[connection.Tl!];
        var linkIndex = connection.LinkIndex!.Value;
        var evalTime = time + dt;

        // C6-ii: an actuated program's phase is not a pure function of time -- read the current
        // phase from its stateful machine (already advanced this step to the phase active at
        // evalTime, see Run()'s Advance call). A static program stays on the pure-function path
        // (TrafficLightState), byte-identical to every pre-C6 scenario.
        char state;
        if (tlLogic.IsActuated && _actuatedLogics.TryGetValue(tlLogic.Id, out var actuated))
        {
            state = actuated.CurrentState[linkIndex];
        }
        else
        {
            state = TrafficLightState.GetLinkState(tlLogic, linkIndex, evalTime);
        }

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
        // C6-ii: same actuated/static split as the state read above (no-op unless a vType sets
        // jmDriveAfterRedTime, which none in scope do).
        var redDuration = tlLogic.IsActuated && _actuatedLogics.TryGetValue(tlLogic.Id, out var actuatedElapsed)
            ? actuatedElapsed.PhaseElapsed(evalTime)
            : TrafficLightState.GetPhaseElapsed(tlLogic, evalTime);
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

        // P2-G Bug-3 (generalized): this vehicle is red/yellow-held AND can brake before the line, so
        // it will STOP here and not enter the junction this step. Record that so ComputeWillPass can
        // force WillPass=false -- a red-held foe must not make a green ego yield (SUMO's mySetRequest
        // is unset for a vehicle stopping at a red). Set only on the will-stop path: the ignoreRed and
        // cannot-brake branches above already returned +inf (the vehicle runs the light -> it DOES
        // enter -> WillPass stays true), so the flag correctly stays false for them.
        v.HeldByRedThisStep = true;

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
    // C4-viii: `prePass` is true only during the willPass PRE-PASS (ComputeWillPass). In that mode the
    // approaching-foe crossing arm keeps its blanket yield (the pre-C4-viii behaviour) so the pre-pass
    // computes each vehicle's vNext WITHOUT reading foes' as-yet-uncomputed WillPass -- the one level of
    // approximation that breaks the yield circularity (SUMO's setApproaching runs before opened()). With
    // prePass=false (the real PlanMovements call) the crossing arm additionally skips a foe whose
    // WillPass is false, matching MSLink::blockedByFoe's `!avi.willPass` short-circuit.
    private double JunctionYieldConstraint(VehicleRuntime v, ActiveVehicleQuery allVehicles, double time, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed, bool prePass = false)
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

        // R4 (rail signal): at a rail_signal junction the SIGNAL arbitrates right-of-way (via
        // RailSignalConstraint's block reservation), NOT the static <request> priority matrix.
        // netconvert still emits a foes/response matrix for the junction's crossing links, but a
        // train obeys its rail signal (stop on red, go on green) rather than the priority yield --
        // so the winning train (whose block is reserved / signal green) must NOT also yield to the
        // held train via this 9b priority path. Skip it here. Inert for every non-rail-signal
        // junction (no committed scenario had a rail_signal junction before this rung).
        if (junction.Type == "rail_signal")
        {
            return double.PositiveInfinity;
        }

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

        // Low-density teleport fix (docs/SUMOSHARP-LOWDENSITY-TELEPORT-DESIGN.md mechanism A): does
        // ego's junction link hold a protected-green traffic signal (SUMO havePriority())? When it
        // does, ego has absolute right-of-way and must NOT take any of the priority-junction yield
        // arms below (cautious-approach, approaching-foe crossing yield, sameTarget arrival-time
        // yield) -- those infer "minor" from the TL-blind static <request> matrix. Sampled at time+dt
        // to agree with RedLightConstraint. False (inert, unchanged) for every uncontrolled link, so
        // byte-identical for non-TL scenarios. The on-junction AdaptToJunctionLeader (rear-end
        // following of a foe already on the junction) and merge PHASE 1 (following a merger on its
        // internal lane) are NOT gated -- those are car-following safety, which SUMO applies
        // regardless of priority (checkLinkLeader), not the minor-link yield.
        var egoHasSignalPriority = EgoLinkHasSignalPriority(egoLink, time + dt);

        // C4-vii-a (cont-turn) fix, extended to the MERGE arm: the true distance from ego's front to
        // its junction-link internal lane (egoInternalLaneId). For an ORDINARY turn `approachLane` is
        // the normal lane immediately before that internal lane and ego is on it, so this reduces to
        // `approachLane.Length - pos` (the loop body never runs -- byte-identical for every committed
        // merge scenario). For a CONT turn (a U-turn / turn split across TWO internal lanes, e.g.
        // I10I11 -> :I11_11_0 -> :I11_19_0 -> I11I10) `approachLane` is the intermediate INTERNAL lane
        // (:I11_11_0, ~1 m) while `pos` is on the normal lane far upstream, so `approachLane.Length -
        // pos` is negative garbage -- which made SameTargetMergeConstraint fire a spurious hard stop
        // hundreds of metres early and FREEZE the vehicle (the dominant city-3000 gridlock seed: ~47%
        // of that demand is U-turn routes). Walking the pool from ego's current lane to the link's
        // internal lane gives the real distance. The cautious-approach block below already does this
        // for its own `seen`; this hoists the same computation so the merge arm can share it.
        var egoDistToEntry = _network.LanesByHandle[v.LaneHandle].Length - v.Kinematics.Pos;
        for (var i = v.LaneSeqIndex + 1; i < egoLinkSeqIndex; i++)
        {
            egoDistToEntry += _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Length;
        }

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

        // C4-viii-b (bug C, the hold arm): ResolveRightBeforeLeftCycles (the willPass post-pass) has
        // broken a symmetric right-before-left cycle by selecting one non-conflicting subset to pass
        // and marking the rest to YIELD (JunctionCycleHold). A held vehicle must stop AT its junction
        // stop line -- SUMO's deterministic analogue of the RNG-aborted request (mySetRequest=false,
        // MSVehicle.cpp:2818-2839) that holds a car at the entry regardless of any foe's state. The
        // foe-relative crossing arm below cannot enforce this (a held car's only higher-priority foe
        // is itself a yielder, so no foe.WillPass is true for it to yield to). Applied ONLY in the real
        // pass (the pre-pass computes each vNext WITHOUT this refinement, mirroring the foeYieldsThisStep
        // gate) and ONLY before ego commits onto its internal lane (the hold gates ENTRY; once on the
        // junction the on-junction leader path governs). Uses the SAME stop-line StopSpeedFor the
        // crossing arm applies (approach-lane end minus PositionEps).
        //
        // FIX (synthetic-junction2 root cause, completing C4-vii-a): the stop-line distance was
        // `approachLane.Length - v.Kinematics.Pos`, the SAME raw formula C4-vii-a already replaced with
        // `egoDistToEntry` for the cautious-approach arm above and for SameTargetMergeConstraint (see
        // that comment) -- this call site was simply missed. For a cont-turn chain (ego still on its
        // normal approach lane, `approachLane` the short INTERMEDIATE internal lane) the raw formula
        // goes deeply negative (observed on synthetic-junction2's TL node 181, veh 114 stalled on lane
        // 182_1 at pos=16.92 with approachLane=":181_14_0" length=7.80: 7.80 - 16.92 = -9.14 m), so
        // StopSpeedFor returns ~0 -- permanently freezing a JunctionCycleHold vehicle regardless of
        // actual gap availability, which is exactly the synthetic-junction2 Y1 teleport pattern.
        // Pinned via temporary per-constraint-arm instrumentation: with the SAME seen/brakeDist/stopDist
        // inputs, the pre-pass (which skips this JunctionCycleHold-gated arm entirely, `!prePass`)
        // computed a sane 7.0854 constraint from the cautious-approach arm alone, while the real pass
        // (which also runs this arm) collapsed to exactly 0.0000 -- isolating this arm's own distance
        // term as the sole culprit. `egoDistToEntry` is mathematically IDENTICAL to
        // `approachLane.Length - pos` for every ORDINARY (single-segment) link (the cont pool-walk loop
        // never executes when approachLane immediately precedes egoInternalLaneId, and then
        // `_network.LanesByHandle[v.LaneHandle]` IS `approachLane`) -- byte-identical no-op for every
        // committed non-cont-turn golden; it only changes a `cont`-chain link, i.e. only when
        // JunctionCycleHold is ALSO true (a rare RBL-tie-break condition). NOTE: the same stale
        // `approachLane.Length - pos` formula also appears at this function's ExternalAgentOnFoeLane arm
        // and its foe-loop approaching-branch (both a few dozen lines below) -- applying this SAME
        // substitution there was tried and reverted: it further reduced synthetic-junction2's teleport
        // count but regressed two saturated-grid stress tests (WillPassSaturationDiagTests,
        // RungHDp2g2CoordinatedLaneChangeTests) to gridlock, so it is deliberately NOT applied there.
        // Those two call sites keep the pre-existing (still cont-unaware, still imperfect) formula.
        if (!prePass && v.JunctionCycleHold && !egoOnInternal && approachLane is not null)
        {
            constraint = Math.Min(
                constraint,
                StopSpeedFor(
                    v.VType, v.Kinematics.Speed,
                    egoDistToEntry - PositionEps,
                    laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService));
        }

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
        // priority junction (this rung's scope): a major link's Response row is all-zero. For a
        // TL-controlled link the static matrix is NOT the RoW authority -- a protected-green ('G')
        // link IS major (havePriority) regardless of geometric conflicts, so `egoHasSignalPriority`
        // suppresses this minor cautious-approach exactly as SUMO's `!havePriority()` gate does.
        if (!egoOnInternal && approachLane is not null && request.Response.Contains('1') && !egoHasSignalPriority)
        {
            // NLHandler.cpp:1413: a link with no explicit `visibility` attribute defaults its
            // foe-visibility distance to 4.5 (for non-ZIPPER links) -- this net specifies none.
            const double visibilityDistance = 4.5;

            // C4-vii-a 2a: `seen` is the distance to ego's junction-link internal lane (egoInternalLaneId).
            // For an ordinary (single-internal-lane) turn `approachLane` IS the normal lane immediately
            // before that internal lane, so `seen = approachLane.Length - pos` -- the exact pre-existing
            // computation, kept verbatim on that path so every committed minor-link scenario stays
            // byte-identical. For a `cont` turn the junction link's internal lane (:C_16_0) is reached
            // only AFTER an intermediate internal lane (:C_3_0), so `approachLane` is itself an internal
            // (':'-edge) lane and `approachLane.Length - pos` is meaningless (pos is on a different lane).
            // There, walk the pool from ego's CURRENT lane to the link's internal lane, summing lane
            // lengths, to get the true distance to the junction-link stop line.
            double seen;
            if (approachLane.EdgeId.Length > 0 && approachLane.EdgeId[0] == ':')
            {
                seen = _network.LanesByHandle[v.LaneHandle].Length - v.Kinematics.Pos;
                for (var i = v.LaneSeqIndex + 1; i < egoLinkSeqIndex; i++)
                {
                    seen += _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Length;
                }
            }
            else
            {
                seen = approachLane.Length - v.Kinematics.Pos;
            }

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
                        v, junction, egoLink, egoInternalLaneId, egoOnInternal, approachLane, egoDistToEntry,
                        j, allVehicles, dt, time, actionStepLengthSecs, laneVehicleMaxSpeed, egoHasSignalPriority));
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

            var foe = FindFoeVehicle(v, foeInternalLaneHandle);
            if (foe is null)
            {
                continue;
            }

            var foeInternalSeqIndex = IndexOfLaneHandle(foe, foeInternalLaneHandle);

            double thisConstraint;
            if (foe.LaneId == foeInternalLaneId)
            {
                // On-junction: MSVehicle::adaptToJunctionLeader.
                // Rung ER2: an emergency vehicle with jmIgnoreJunctionFoeProb IGNORES the
                // on-junction link-leader (MSVehicle.cpp:3430 -- checkLinkLeaderCurrentAndParallel's
                // `continue`), so this foe imposes no constraint. Inert (IgnoresJunctionFoe returns
                // false) for every vType that leaves jmIgnoreJunctionFoeProb at its 0 default.
                thisConstraint = IgnoresJunctionFoe(v, time)
                    ? double.PositiveInfinity
                    : AdaptToJunctionLeader(v, egoLane, approachLane, egoOnInternal, conflict, foe, dt, time, actionStepLengthSecs, laneVehicleMaxSpeed);
            }
            else if (foeInternalSeqIndex > foe.LaneSeqIndex)
            {
                // Approaching (foe hasn't reached its own internal lane yet): the stop-line
                // yield only guards ENTRY onto ego's own internal lane -- once ego has already
                // been granted entry (egoOnInternal), it is no longer gated by this foe's
                // approach state.
                // C4-vi (far-routed-foe false positive): FindFoeVehicle matches ANY vehicle whose
                // route includes this internal lane -- including one many junctions / kilometres
                // away that merely passes through here much later. SUMO never yields to such a foe:
                // MSLink::opened only sees a foe registered via MSLink::setApproaching, and a
                // vehicle registers approaching a link ONLY while it is within its own planMove
                // lookahead `dist = SPEED2DIST(maxV) + brakeGap(maxV)` of that link (MSVehicle.cpp:
                // 2238). A foe farther than that has not reserved this link, so opened() reports
                // numApproaching==0 and ego is not blocked. This is the SAME reservation-distance
                // gate the sameTarget-merge PHASE-0 arm already applies (SameTargetMergeConstraint),
                // ported here to the crossing arm it was missing from -- without it, on a dense
                // multi-junction network essentially every approach lane is on SOME distant
                // vehicle's route and the minor road never gets a gap (the city-300 benchmark:
                // 58.8% of vehicles permanently stuck while SUMO runs the identical net at free
                // flow). `SeenToInternalLaneEntry` is the foe's distance to this internal lane's
                // start; beyond the reservation range the foe is not yet approaching -> ego proceeds.
                // (Single-foe-per-link scope is unchanged: FindFoeVehicle still returns the first
                // route-matching foe; a genuinely-close foe hidden behind a far one is out of scope
                // here as before.)
                var foeMaxV = KraussModel.MaxNextSpeed(foe.Kinematics.Speed, foe.VType, dt);
                var foeReservationDist = KraussModel.Speed2Dist(foeMaxV, dt)
                    + KraussModel.BrakeGap(foeMaxV, foe.VType.Decel, foe.VType.Tau, dt);
                var foeNotApproaching = SeenToInternalLaneEntry(foe, foeInternalLaneHandle) > foeReservationDist;

                // C5 follow-on (willPass): SUMO's blockedByFoe short-circuits on `!avi.willPass`
                // (MSLink.cpp:935) -- a foe that will NOT enter the junction does not block ego. The
                // engine's approaching-foe yield is otherwise blanket; here we skip it when the foe
                // is keepClear-BLOCKED (its own checkRewindLinkLanes removal cleared its request,
                // MSVehicle.cpp:5245 `dpi.mySetRequest = false`), i.e. its downstream exit is jammed
                // so it stop-line-holds at the junction entry rather than crossing. Reuses
                // KeepClearConstraint as the predicate: finite -> the foe is keepClear-held -> ego
                // (the crossing vehicle) proceeds. Inert for every scenario without a downstream jam
                // (KeepClearConstraint is +infinity there), so only scenario 38 is affected.
                var foeWillNotPass = !egoOnInternal && FoeKeepClearBlocked(foe, allVehicles, dt, actionStepLengthSecs);

                // C4-viii (the willPass gate -- the dense-grid saturation fix): `foe.WillPass` (cached by
                // Engine.ComputeWillPass from the frozen start-of-step snapshot) is true iff the foe's
                // PLANNED vNext this step carries it INTO its upcoming junction link -- SUMO's
                // setRequest/willPass (MSVehicle.cpp:2732). A foe that is ITSELF yielding (moving at
                // start-of-step but braking to a stop THIS step, vNext ~ 0) has WillPass=false and, per
                // MSLink::blockedByFoe's `if (!avi.willPass) return false` (MSLink.cpp:935), does NOT
                // block ego -- which unwinds the mutual brake-to-stop deadlock a saturated grid produces.
                // The load-bearing term is the PLANNED vNext, not start-of-step speed: a braking foe has
                // speed > 0 this step, so a raw-speed proxy misses it. Applied only in the real
                // PlanMovements pass; the pre-pass keeps the blanket yield (prePass short-circuits this to
                // false) so it can compute each vNext without the foe-willPass refinement -- the one
                // level of approximation that breaks the circularity (setApproaching-before-opened()).
                var foeYieldsThisStep = !prePass && !foe.WillPass;
                // Rung ER2: an emergency vehicle with jmIgnoreFoeProb IGNORES an approaching
                // priority foe whose speed is at/below jmIgnoreFoeSpeed (MSLink.cpp:898-902, the
                // opened()/blockedAtTime foe-skip), so the approaching foe imposes no stop-line
                // yield. Inert (IgnoresApproachingFoe returns false) for every vType that leaves
                // jmIgnoreFoeProb at its 0 default.
                var ignoresFoe = IgnoresApproachingFoe(v, foe.Kinematics.Speed, time);
                // P2-G Bug-3: a foe held at a RED traffic light does not block ego. This is now handled
                // generally by the `!foe.WillPass` term above: RedLightConstraint sets the foe's
                // HeldByRedThisStep and ComputeWillPass forces its WillPass=false (SUMO's mySetRequest
                // is unset for a vehicle stopping at a red). That subsumes -- and, unlike -- the earlier
                // ad-hoc single-lane red check, also reaches a cont (internal-junction) turn foe, whose
                // request-matrix lane is the internal continuation rather than the red entry lane.
                // egoHasSignalPriority: a protected-green ('G') ego holds SUMO havePriority() and does
                // NOT stop-line-yield to an approaching foe (the signal already resolved the conflict --
                // the foe on the conflicting movement is red). Only the APPROACHING stop-line yield is
                // gated; the on-junction AdaptToJunctionLeader branch above is car-following, untouched.
                var takesCrossingYield = !(egoOnInternal || foeWillNotPass || foeNotApproaching || foeYieldsThisStep || ignoresFoe || egoHasSignalPriority);
                // Perf (willPass/plan fusion): a finite approaching-foe crossing yield taken in the
                // pre-pass is the ONLY thing the real pass can relax (via `!foe.WillPass`), so flag it
                // -- PlanMovements must then RECOMPUTE this vehicle rather than reuse the pre-pass
                // Intent. Set only in the pre-pass; the real pass never reuses, so it never reads this.
                if (prePass && takesCrossingYield)
                {
                    v.CrossingYieldTaken = true;
                }

                thisConstraint = takesCrossingYield
                    ? StopSpeedFor(
                        v.VType, v.Kinematics.Speed,
                        approachLane!.Length - v.Kinematics.Pos - PositionEps,
                        laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService)
                    : double.PositiveInfinity;
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

    // Rung ER2 (emergency ignore-FOE). Distinct salts so the two junction-foe ignore streams are
    // independent of each other and of RngState/SpeedFactorRngSalt (see VehicleRng.SeedFor's
    // 3-arg overload) -- only ever consumed on the 0<prob<1 statistical path below, which no
    // committed (exact-parity) scenario exercises.
    private const ulong JmIgnoreFoeRngSalt = 0x49676E6F7246466FUL;         // "IgnorFFo"
    private const ulong JmIgnoreJunctionFoeRngSalt = 0x49676E4A756E4666UL; // "IgnJunFf"

    // MSVehicle::checkLinkLeaderCurrentAndParallel (sumo/src/microsim/MSVehicle.cpp:3419/3430):
    // ignore an ON-JUNCTION link-leader iff jmIgnoreJunctionFoeProb>0 AND jmIgnoreJunctionFoeProb
    // >= rand() (no speed gate). Inert for every vType that leaves the prob at its 0 default.
    private bool IgnoresJunctionFoe(VehicleRuntime v, double time)
        => IgnoreProbHit(v.VType.JmIgnoreJunctionFoeProb, v, time, JmIgnoreJunctionFoeRngSalt);

    // MSLink::blockedAtTime (sumo/src/microsim/MSLink.cpp:898-902, reached from MSLink::opened):
    // ignore an APPROACHING foe iff jmIgnoreFoeProb>0 AND jmIgnoreFoeSpeed>=foe.speed AND
    // jmIgnoreFoeProb>=rand(). The speed gate (:901 `jmIgnoreFoeSpeed < it.second.speed`) means a
    // foe faster than jmIgnoreFoeSpeed is NOT ignored. Inert for the 0 default.
    private bool IgnoresApproachingFoe(VehicleRuntime v, double foeSpeed, double time)
    {
        if (v.VType.JmIgnoreFoeProb <= 0.0 || foeSpeed > v.VType.JmIgnoreFoeSpeed)
        {
            return false;
        }

        return IgnoreProbHit(v.VType.JmIgnoreFoeProb, v, time, JmIgnoreFoeRngSalt);
    }

    // SUMO's foe-ignore probability test (MSLink.cpp:902 `prob < rand()` negated == ignore when
    // `prob >= rand()`; MSVehicle.cpp:3420/3431 `prob >= rand()`). CLAUDE.md rule 5 / the give-way
    // briefing: any probabilistic knob uses per-entity seeded VehicleRng, NEVER System.Random.
    //   prob <= 0 -> inert (never ignore): the default for every vType, keeps all ~30 junction
    //               scenarios byte-identical (no draw, so RngState/bench hash are untouched).
    //   prob >= 1 -> always ignore (deterministic): the EXACT-parity emergency case (scenarios
    //               51/52 set prob=1), no draw needed.
    //   0<prob<1 -> a STATELESS per-(entity, step) draw from a salted VehicleRng seeded off
    //               (Seed, EntityIndex, salt^stepBits) -- a pure function of those inputs, hence
    //               order-independent / parallel-safe, and it never advances the dawdle RngState.
    //               This is a STATISTICAL (behavioral) path with no exact golden and is not
    //               exercised by any committed scenario (matches the give-way rungs' RNG posture).
    private bool IgnoreProbHit(double prob, VehicleRuntime v, double time, ulong salt)
    {
        if (prob <= 0.0)
        {
            return false;
        }

        if (prob >= 1.0)
        {
            return true;
        }

        var stepBits = (ulong)BitConverter.DoubleToInt64Bits(time);
        var rng = VehicleRng.SeedFor(Seed, v.EntityIndex, salt ^ stepBits);
        return prob >= rng.NextDouble();
    }

    // C5 (keepClear / don't-block-the-box): the "removal" half of MSVehicle::checkRewindLinkLanes
    // (sumo/src/microsim/MSVehicle.cpp:5025). A vehicle must not enter a junction it cannot clear:
    // when its downstream EXIT lane is jammed by a stopped vehicle and there is no room for ego
    // (`availableSpace - lengthWithGap < 0`), SUMO sets `removalBegin` and brakes ego to the
    // junction-entry stop line (`myVLinkPass = myVLinkWait`) rather than letting it advance onto the
    // internal lane and block cross traffic. Ported and VERIFIED against the vendored v1_20_0
    // DEBUG_CHECKREWINDLINKLANES trace for scenarios/34-keepclear's mThrough (per step: exit lane JE
    // `stls=1.0`, empty internal lane, `avail=1.0` -> `leftSpace=1.0-7.5=-6.5` -> `removalBegin=0`,
    // brake to `WJ.len-1.0=91.8`). keepClear applies iff the link has crossing foes
    // (`request.Foes` has a set bit) -- MSVehicle::keepClear's `link->hasFoes() && link->keepClear()`
    // (the keepClear connection flag defaults true; a `keepClear="0"` override is not parsed, not
    // exercised). +infinity (non-binding) unless a STOPPED vehicle is found on ego's downstream exit
    // chain with negative leftSpace, so every jam-free scenario is untouched.
    //
    // SIMPLIFICATIONS (documented, matching the committed anchor): `lengthsInFront` (vehicles ahead
    // of ego on its OWN approach lane) is taken as 0 -- a queue on the approach itself is handled by
    // ordinary car-following, not this gate; the back-propagation `allowsContinuation` reduces, for a
    // single empty internal lane, to copying the exit lane's space to the entry link (done here by
    // accumulating `seenSpace` straight through and stopping at the first stopped vehicle); the
    // stop-line offset uses the priority-link `DIST_TO_STOPLINE_EXPECT_PRIORITY` (1.0), not the
    // foe-visibility-limited general form.
    private double KeepClearConstraint(VehicleRuntime v, ActiveVehicleQuery allVehicles, double dt, double actionStepLengthSecs, double laneVehicleMaxSpeed)
    {
        // Ego's upcoming junction ENTRY link -- the first internal lane at/after LaneSeqIndex (the
        // same forward scan JunctionYieldConstraint uses).
        var egoLinkSeqIndex = -1;
        string? egoInternalLaneId = null;
        for (var i = v.LaneSeqIndex; i < v.LaneSeqLen; i++)
        {
            var seqLaneId = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Id;
            if (_network.LinkByInternalLane.ContainsKey(seqLaneId))
            {
                egoLinkSeqIndex = i;
                egoInternalLaneId = seqLaneId;
                break;
            }
        }

        if (egoInternalLaneId is null || v.LaneId == egoInternalLaneId || egoLinkSeqIndex < 1)
        {
            // No upcoming junction, already on the internal lane (committed), or no approach lane.
            return double.PositiveInfinity;
        }

        var (junction, egoLink) = _network!.LinkByInternalLane[egoInternalLaneId];

        JunctionRequest? request = null;
        foreach (var r in junction.Requests)
        {
            if (r.Index == egoLink.Index)
            {
                request = r;
                break;
            }
        }

        if (request is null || !request.Foes.Contains('1'))
        {
            // keepClear only applies at a link with crossing foes.
            return double.PositiveInfinity;
        }

        // Downstream available-space walk from the entry link: subtract each internal lane's brutto
        // vehicle-length sum, add each normal exit lane's space-till-last-standing, stop at the first
        // STOPPED vehicle (lengthsInFront == 0, see header).
        var seenSpace = 0.0;
        var foundStopped = false;
        for (var i = egoLinkSeqIndex; i < v.LaneSeqLen && !foundStopped; i++)
        {
            var lane = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]];
            if (lane.Id.StartsWith(':'))
            {
                seenSpace -= LaneBruttoVehLenSum(lane, v);
            }
            else
            {
                seenSpace += LaneSpaceTillLastStanding(lane, v, dt, out foundStopped);
            }
        }

        if (!foundStopped || seenSpace - (v.VType.Length + v.VType.MinGap) >= 0.0)
        {
            // Either the exit chain is clear, or ego fits -> not a box-blocking situation.
            return double.PositiveInfinity;
        }

        // Blocked box: brake to the junction-entry stop line (approach-lane end minus the 1.0
        // priority stop offset -- DIST_TO_STOPLINE_EXPECT_PRIORITY).
        const double distToStopLine = 1.0;
        var approachLane = _network.LanesByHandle[_laneSeqPool[v.LaneSeqStart + egoLinkSeqIndex - 1]];
        var stopDist = approachLane.Length - v.Kinematics.Pos - distToStopLine;
        return StopSpeedFor(v.VType, v.Kinematics.Speed, stopDist, laneVehicleMaxSpeed, dt, actionStepLengthSecs, v.LevelOfService);
    }

    // C5 follow-on (willPass): is `foe` keepClear-BLOCKED at its upcoming junction entry -- i.e. its
    // own downstream exit is jammed, so checkRewindLinkLanes would clear its request and it will NOT
    // enter the junction? Reuses KeepClearConstraint (finite result == blocked). +infinity (not
    // blocked) for every foe without a downstream jam, so this is inert outside the keepClear box.
    private bool FoeKeepClearBlocked(VehicleRuntime foe, ActiveVehicleQuery allVehicles, double dt, double actionStepLengthSecs)
    {
        var foeLane = _network!.LanesByHandle[foe.LaneHandle];
        var foeLaneMaxV = KraussModel.LaneVehicleMaxSpeed(foeLane.Speed, foe.SpeedFactor, foe.VType);
        return KeepClearConstraint(foe, allVehicles, dt, actionStepLengthSecs, foeLaneMaxV) < double.PositiveInfinity;
    }

    // C5 helper: MSLane::getBruttoVehLenSum -- the sum of lengthWithGap (length + minGap) over the
    // vehicles currently on the lane (frozen start-of-step snapshot), excluding ego.
    // Perf (super-linear fix): reads the per-lane bucket (O(vehicles-on-lane)) instead of scanning
    // every active vehicle. The bucket is the same frozen snapshot the flat scan walked, so the SET
    // summed is identical; the sum's iteration order differs (pos-sorted vs _vehicles order) but the
    // committed keepClear scenarios are homogeneous (equal length+minGap), so the sum is
    // order-independent and byte-identical -- the 227 goldens are the guard.
    private double LaneBruttoVehLenSum(Lane lane, VehicleRuntime ego)
    {
        var sum = 0.0;
        var onLane = _neighborQuery!.OnLane(lane.Handle);
        for (var i = 0; i < onLane.Count; i++)
        {
            var other = onLane[i];
            if (!ReferenceEquals(other, ego))
            {
                sum += other.VType.Length + other.VType.MinGap;
            }
        }

        return sum;
    }

    // C5 helper: MSLane::getSpaceTillLastStanding (sumo/src/microsim/MSLane.cpp). Walk the lane's
    // vehicles front-first (largest pos first); at the first STOPPED one (speed < haltingSpeed)
    // return its back position + its brakeGap minus the lengthWithGap of the vehicles ahead of it;
    // if none is stopped return laneLength minus the total lengthWithGap. `foundStopped` reports
    // whether a stopped vehicle bounded the result.
    // Perf (super-linear fix): reads the per-lane bucket instead of scanning every active vehicle and
    // re-sorting. The bucket is pos-ASCENDING, so walking it in REVERSE is the same front-first
    // (pos-descending) order the former collect-and-sort produced -- byte-identical for the (universal
    // in phase-1) case of distinct positions; the accumulated lengthWithGap is homogeneous so
    // order-independent regardless. Ego is skipped inline exactly as the old LaneId filter excluded it.
    private double LaneSpaceTillLastStanding(Lane lane, VehicleRuntime ego, double dt, out bool foundStopped)
    {
        foundStopped = false;
        var onLane = _neighborQuery!.OnLane(lane.Handle);

        var lengths = 0.0;
        for (var i = onLane.Count - 1; i >= 0; i--)
        {
            var last = onLane[i];
            if (ReferenceEquals(last, ego))
            {
                continue;
            }

            if (last.Kinematics.Speed < KraussModel.HaltingSpeed)
            {
                foundStopped = true;
                var lastBrakeGap = KraussModel.BrakeGap(last.Kinematics.Speed, last.VType.Decel, headwayTime: 0.0, dt);
                return (last.Kinematics.Pos - last.VType.Length) + lastBrakeGap - lengths;
            }

            lengths += last.VType.Length + last.VType.MinGap;
        }

        return lane.Length - lengths;
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
            var foe = FindFoeVehicle(v, foeInternalLaneHandle);
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
        bool egoOnInternal, Lane? approachLane, double egoDistToEntry, int foeLinkIndex, ActiveVehicleQuery allVehicles,
        double dt, double time, double actionStepLengthSecs, double laneVehicleMaxSpeed, bool egoHasSignalPriority = false)
    {
        if (approachLane is null && !egoOnInternal)
        {
            return double.PositiveInfinity;
        }

        // GATE: on the approach lane and still beyond foe-visibility of the entry -> the cautious
        // approach governs; the merge is non-binding (MSVehicle.cpp:3478 MAX2 relaxation).
        const double visibilityDistance = 4.5;
        if (!egoOnInternal && egoDistToEntry > visibilityDistance)
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
            : egoDistToEntry + egoInternalLane.Length;

        // PHASE 1: foe still on its merging internal lane.
        var foeInternalLaneId = junction.IntLanes[foeLinkIndex];
        var foeInternalLaneHandle = _network.LaneHandleById[foeInternalLaneId];
        var foeMerging = FindFoeVehicle(ego, foeInternalLaneHandle);

        // PHASE 0 (APPROACHING foe -- junction arrival-time RoW): the responded merge foe is still
        // on its OWN approach lane, heading for the shared merge (not yet ON its merging internal
        // lane -- PHASE 1 -- nor on the shared target lane -- PHASE 2). SUMO decides this via
        // MSLink::opened/blockedByFoe: ego stop-line yields iff the foe's arrival-time window at the
        // conflict overlaps ego's (verified against the v1_20_0 arrival-time DEBUG trace for
        // scenarios/32-roundabout's vSouth: `blocked (hard conflict)` t=14..18, then PHASE 1 takes
        // over at t=19 once vWest is on :RS_1). This is what makes a roundabout entry yield to
        // circulating traffic that has not yet reached the ring node -- while NOT over-yielding to a
        // far foe (scenario 19's distant mainline lands in blockedByFoe's "ego leader" arm and does
        // not block). Once ego is on its internal lane it is committed and no longer gated.
        // egoHasSignalPriority: a protected-green ('G') ego has SUMO havePriority() and does NOT take
        // this arrival-time stop-line yield -- the signal already resolved the merge conflict (the
        // conflicting merger is red). Only PHASE 0 (the stop-line yield) is gated; PHASE 1 below
        // (following a foe already on its merging internal lane) is car-following and stays active.
        if (!egoOnInternal
            && !egoHasSignalPriority
            && foeMerging is not null
            && foeMerging.LaneId != foeInternalLaneId
            && IndexOfLaneHandle(foeMerging, foeInternalLaneHandle) > foeMerging.LaneSeqIndex)
        {
            var foeInternalLaneAppr = _network.LanesByHandle[foeInternalLaneHandle];
            var egoInternalLaneAppr = _network.LanesById[egoInternalLaneId];
            var egoSeen = egoDistToEntry;
            var foeSeen = SeenToInternalLaneEntry(foeMerging, foeInternalLaneHandle);

            // Approach-reservation range (MSVehicle::setApproaching only registers a vehicle at links
            // within its own planMove lookahead dist = SPEED2DIST(maxV) + brakeGap(maxV),
            // MSVehicle.cpp:2238): a foe farther than this from the conflict has not reserved the
            // link, so opened() never sees it (numApproaching==0) and ego is not blocked. This is
            // what distinguishes a close circulating foe (scenario 32, ~8 m) from a distant mainline
            // foe (scenario 19, ~362 m -- unblocked despite responding in the request matrix).
            // SPEED2DIST(maxV) + brakeGap(maxV): the single-arg MSCFModel::brakeGap(maxV) expands to
            // brakeGap(maxV, myDecel, myHeadwayTime) (MSCFModel.h), so the reservation range carries
            // the foe's headway (tau) reaction margin -- pass VType.Tau, not 0.
            var foeMaxV = KraussModel.MaxNextSpeed(foeMerging.Kinematics.Speed, foeMerging.VType, dt);
            var foeReservationDist = KraussModel.Speed2Dist(foeMaxV, dt)
                + KraussModel.BrakeGap(foeMaxV, foeMerging.VType.Decel, foeMerging.VType.Tau, dt);
            if (foeSeen > foeReservationDist)
            {
                return double.PositiveInfinity;
            }

            // Arrival speeds: the current speed (a constant-speed arrival estimate). The block
            // decision compares windows against a 1 s lookAhead with wide margins, so it is robust
            // to this approximation -- confirmed against the trace's per-step verdicts.
            var egoSpeed = ego.Kinematics.Speed;
            var foeSpeed = foeMerging.Kinematics.Speed;
            var egoArrival = KraussModel.MinimalArrivalTime(egoSeen, egoSpeed, egoSpeed, ego.VType);
            var foeArrival = KraussModel.MinimalArrivalTime(foeSeen, foeSpeed, foeSpeed, foeMerging.VType);
            var egoLeave = egoArrival + (egoInternalLaneAppr.Length + ego.VType.Length)
                / Math.Max(0.5 * (egoSpeed + egoSpeed), KraussModel.NumericalEps);
            var foeLeave = foeArrival + (foeInternalLaneAppr.Length + foeMerging.VType.Length)
                / Math.Max(0.5 * (foeSpeed + foeSpeed), KraussModel.NumericalEps);

            if (BlockedByMergeFoe(
                    egoArrival, egoLeave, egoSpeed, egoSpeed, ego.VType.Decel,
                    foeArrival, foeLeave, foeSpeed, foeSpeed, foeMerging.VType.Decel))
            {
                return StopSpeedFor(
                    ego.VType, ego.Kinematics.Speed,
                    egoDistToEntry - PositionEps,
                    laneVehicleMaxSpeed, dt, actionStepLengthSecs, ego.LevelOfService);
            }
        }

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
                    hasPred: true, predIsCacc: foeMerging.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService, ballistic: _config!.Ballistic);
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
            hasPred: true, predIsCacc: leaderOnTarget.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService, ballistic: _config!.Ballistic);
    }

    // C4-iv phase-2 helper: the rearmost vehicle (smallest Pos = ego's immediate downstream leader)
    // CURRENTLY on the given lane, excluding ego. Frozen start-of-step snapshot.
    // GAP-3 follow-up: this is a raw ActiveVehicles() scan (not the parked-excluding neighbor
    // query), used as the merge-target-lane LEADER for a vehicle merging onto a shared exit lane a
    // foe has already crossed onto -- the exact E1D1_0-style exit-lane case the off-lane fix targets.
    // Skip IsParked so a park-and-stay car on the shared target lane cannot wrongly act as this
    // leader (gated, byte-identical elsewhere).
    private VehicleRuntime? FindRearmostOnLane(VehicleRuntime ego, ActiveVehicleQuery allVehicles, string laneId)
    {
        VehicleRuntime? rearmost = null;
        foreach (var other in allVehicles)
        {
            if (other.IsParked || ReferenceEquals(other, ego) || other.LaneId != laneId)
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

    // Arrival-time RoW: the distance from a vehicle's front to the ENTRY of the given internal lane
    // along its own route (the junction stop line for that link) -- (currentLane.length - pos) plus
    // the lengths of any lanes strictly between the current one and the target internal lane. This
    // is the `seen` fed to MSCFModel::getMinimalArrivalTime for the vehicle's arrival window at the
    // conflicting link. +infinity when the internal lane is not on the vehicle's route.
    private double SeenToInternalLaneEntry(VehicleRuntime veh, int targetInternalLaneHandle)
    {
        var targetIndex = IndexOfLaneHandle(veh, targetInternalLaneHandle);
        if (targetIndex < 0)
        {
            return double.PositiveInfinity;
        }

        var seen = _network!.LanesByHandle[veh.LaneHandle].Length - veh.Kinematics.Pos;
        for (var i = veh.LaneSeqIndex + 1; i < targetIndex; i++)
        {
            seen += _network.LanesByHandle[_laneSeqPool[veh.LaneSeqStart + i]].Length;
        }

        return seen;
    }

    // Arrival-time RoW (MSLink::blockedByFoe, MSLink.cpp:919-1013, impatience==0 / sameTargetLane
    // arm -- phase 1 has no impatience, and a sameTarget MERGE always has sameTargetLane==true):
    // does the foe's arrival window at the merge conflict block ego? ego occupies its internal lane
    // during [egoArrival, egoLeave]; the foe during [foeArrival, foeLeave]; lookAhead == 1.0 s
    // (MSLink::myLookaheadTime = TIME2STEPS(1), no JM_TIMEGAP_MINOR override). Times are relative
    // (seconds from now) -- the common `t - DELTA_T` offset cancels between ego and foe.
    //   - ego is follower (foeLeave < egoArrival): blocked if the gap is under lookAhead OR the
    //     merge speeds are unsafe.
    //   - ego is leader   (foeArrival > egoLeave + lookAhead): blocked only if the merge speeds are
    //     unsafe (a far foe -- e.g. the scenario-19 mainline -- lands here and does NOT block).
    //   - otherwise the windows overlap: hard conflict, blocked.
    private static bool BlockedByMergeFoe(
        double egoArrival, double egoLeave, double egoArrSpeed, double egoLeaveSpeed, double egoDecel,
        double foeArrival, double foeLeave, double foeArrSpeed, double foeLeaveSpeed, double foeDecel)
    {
        const double lookAhead = 1.0;

        // MSLink.h unsafeMergeSpeeds(leaderSpeed, followerSpeed, leaderDecel, followerDecel) =
        // leaderSpeed^2/leaderDecel <= followerSpeed^2/followerDecel.
        static bool Unsafe(double leaderSpeed, double followerSpeed, double leaderDecel, double followerDecel) =>
            leaderSpeed * leaderSpeed / leaderDecel <= followerSpeed * followerSpeed / followerDecel;

        if (foeLeave < egoArrival)
        {
            return egoArrival - foeLeave < lookAhead || Unsafe(foeLeaveSpeed, egoArrSpeed, foeDecel, egoDecel);
        }

        if (foeArrival > egoLeave + lookAhead)
        {
            return Unsafe(egoLeaveSpeed, foeArrSpeed, egoDecel, foeDecel);
        }

        return true;
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
                hasPred: true, predIsCacc: foe.VType.CarFollowModel == "CACC", levelOfService: ego.LevelOfService, ballistic: _config!.Ballistic);
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
    // Perf (super-linear fix): O(1) lookup into the per-step foe-approach index (BuildFoeApproachIndex)
    // instead of the former O(vehicles * routeLen) flat scan. BYTE-IDENTICAL to that scan, which
    // returned "the first active vehicle, in _vehicles order, that is not ego and whose remaining route
    // contains foeInternalLaneHandle": the index holds the first two DISTINCT such vehicles in that same
    // order, and ego (if present at all) can only be the first of them -- so if the first is ego the
    // original would have skipped it and taken the very next match, which is exactly the second slot.
    private VehicleRuntime? FindFoeVehicle(VehicleRuntime ego, int foeInternalLaneHandle)
    {
        var first = _foeApproachFirst[foeInternalLaneHandle];
        if (first is null)
        {
            return null;
        }

        return ReferenceEquals(first, ego) ? _foeApproachSecond[foeInternalLaneHandle] : first;
    }

    // Perf (super-linear fix): fill _foeApproachFirst/Second for this step -- for every internal lane
    // handle, the FIRST TWO distinct active vehicles (in _vehicles iteration order) whose remaining
    // lane sequence contains it. Reproduces FindFoeVehicle's former per-call scan order exactly, but
    // pays the O(sum of route lengths) cost ONCE per step instead of once per foe-link per
    // vehicle-at-junction. Runs single-threaded before the (possibly parallel) plan phase reads it.
    private void BuildFoeApproachIndex()
    {
        Array.Clear(_foeApproachFirst, 0, _foeApproachFirst.Length);
        Array.Clear(_foeApproachSecond, 0, _foeApproachSecond.Length);
        foreach (var v in ActiveVehicles())
        {
            // GAP-3 follow-up (ISSUE2-JUNCTION-KEEPCLEAR-DESIGN.md): a parked (park-and-stay) vehicle
            // is off the lane in SUMO -- it must not register as a "foe approaching" some internal
            // lane still ahead in its (never-to-be-driven, while parked) remaining route. Left
            // unguarded, a park-and-stay car with unresolved downstream route lanes would show up as
            // a permanent phantom foe that a real vehicle yields to forever -- exactly the deadlock
            // pattern the off-lane exclusion exists to prevent. Gated on IsParked (default false), so
            // byte-identical for every scenario without a parked vehicle.
            if (v.IsParked)
            {
                continue;
            }

            for (var i = 0; i < v.LaneSeqLen; i++)
            {
                var h = _laneSeqPool[v.LaneSeqStart + i];
                if (!_isInternalLane[h])
                {
                    continue;
                }

                if (_foeApproachFirst[h] is null)
                {
                    _foeApproachFirst[h] = v;
                }
                else if (_foeApproachSecond[h] is null && !ReferenceEquals(_foeApproachFirst[h], v))
                {
                    _foeApproachSecond[h] = v;
                }
            }
        }
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
    // C8-iii: instance (was static) only so it can read the immutable `_config.Ballistic` flag for
    // the FollowSpeedFor call below -- a read of load-time-constant config, safe under UseParallelPlan
    // (the parallel-plan invariant forbids WRITING shared state during planning, not reading it).
    private double LeaderFollowSpeedConstraint(VehicleRuntime ego, LaneNeighborQuery neighbors, double dt, double time, double laneVehicleMaxSpeed, int packedEgoSlot = -1)
    {
        // SPATIAL-OPT probe: when the plan takes the spatial branch, the same-lane leader is the
        // adjacent packed slot (sequential/prefetched) instead of a random foe-object deref. The
        // packed leader is byte-identically the same vehicle GetLeader returns (nearest strictly-
        // ahead by pos, co-located skipped) with the same frozen start-of-step field values (doubles),
        // so this returns the identical speed. Every OTHER neighbor read in this method's callers is
        // unchanged (still on `neighbors`); only this same-lane leader read moves to `_packed`.
        if (packedEgoSlot >= 0)
        {
            var leaderSlot = _leaderSlotByPacked[packedEgoSlot];
            if (leaderSlot < 0)
            {
                return double.PositiveInfinity;
            }

            ref readonly var lp = ref _packed[leaderSlot];
            if (!FootprintsOverlap(lp.LatOffset, lp.Width, ego.Kinematics.LatOffset, ego.VType.Width))
            {
                return double.PositiveInfinity;
            }

            var gapP = (lp.Pos - lp.Length) - ego.VType.MinGap - ego.Kinematics.Pos;
            return FollowSpeedFor(
                ego.VType,
                egoSpeed: ego.Kinematics.Speed,
                gap: gapP,
                predSpeed: lp.Speed,
                predMaxDecel: lp.Decel,
                laneVehicleMaxSpeed: laneVehicleMaxSpeed,
                dt: dt,
                time: time,
                accControlMode: ref ego.AccControlMode,
                accLastUpdateTime: ref ego.AccLastUpdateTime,
                caccControlMode: ref ego.CaccControlMode,
                egoAcceleration: ego.Acceleration,
                hasPred: true,
                predIsCacc: lp.IsCacc,
                levelOfService: ego.LevelOfService,
                ballistic: _config!.Ballistic);
        }

        var leader = neighbors.GetLeader(ego);
        if (leader is null)
        {
            return double.PositiveInfinity;
        }

        // Rung ER5 (give-way execution, single-lane fallback): two vehicles sharing one wide lane
        // may PASS each other when their lateral footprints no longer overlap -- the same
        // FootprintsOverlap test B6 uses for a dodged obstacle, applied to a same-lane leader. This
        // is what lets a blue-light EV get past a give-way vehicle that has drifted to the lane edge
        // (ER5), AND lets that drifted vehicle NOT slam on the brakes for the EV as it draws
        // alongside. SELF-GATING and inert for every parity scenario: two lane-CENTRED vehicles
        // (LatOffset 0, the only state any committed scenario ever has) always overlap, so this never
        // fires unless a vehicle has genuinely drifted clear -- which only happens on the give-way /
        // B6 lateral paths, neither of which any golden scenario exercises with a vehicle leader.
        if (!FootprintsOverlap(leader.Kinematics.LatOffset, leader.VType.Width, ego.Kinematics.LatOffset, ego.VType.Width))
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
            levelOfService: ego.LevelOfService,
            ballistic: _config!.Ballistic);
    }

    // Cross-junction leader following: MSVehicle::planMoveInternal's per-downstream-lane leader scan
    // (the `ahead`/getLeaderInfo loop over myLFLinkLanes, MSVehicle.cpp:2508-2544). A vehicle
    // approaching a junction must car-follow a leader that has ALREADY crossed onto a downstream lane
    // -- the same-lane LeaderFollowSpeedConstraint (above) sees only ego's CURRENT lane, so without
    // this a follower blows through the junction at full speed and brakes erratically once it lands
    // behind the slow leader. Walks the route pool forward from ego's current lane, accumulating
    // distance, and follows the nearest downstream leader found within the plan-move lookahead
    // `dist = SPEED2DIST(maxV) + brakeGap(maxV)` (MSVehicle.cpp:2238, the same window
    // JunctionYieldConstraint / setApproaching use). +inf (non-binding) when no such leader exists,
    // so every scenario without a close cross-junction leader is untouched. The junction RoW /
    // red-light constraints separately cap the speed (Min), so this is safe to evaluate
    // unconditionally: when ego must yield it stops at the line (a smaller speed) and this term is
    // dominated; when ego proceeds, this is the leader it will actually follow through.
    private double CrossJunctionLeaderConstraint(VehicleRuntime ego, LaneNeighborQuery neighbors, double dt, double time, double laneVehicleMaxSpeed)
    {
        // L0d (PERF-ROADMAP.md): the downstream (route-ahead) pool lanes are the CONTIGUOUS slice of
        // `_laneSeqPool` immediately after ego's current slot -- a zero-alloc `ReadOnlySpan<int>` over
        // it instead of copying into a per-vehicle-per-step `new List<int>`. The pool is stable during
        // the Plan phase (appended only at insertion, in the earlier Input phase), so the span is safe
        // to read even when Plan runs in parallel. The rearmost-leader source is a by-value struct
        // callback (see NeighborRearmost) instead of a `h => neighbors.GetRearmost(ego, h)` closure.
        var downstreamCount = ego.LaneSeqLen - ego.LaneSeqIndex - 1;
        if (downstreamCount <= 0)
        {
            return double.PositiveInfinity;
        }

        var downstreamStart = ego.LaneSeqStart + ego.LaneSeqIndex + 1;
#if NET8_0_OR_GREATER
        // net8.0 (the parity target): zero-copy span over the pool's backing array. Byte-identical
        // to the frozen behavior -- this branch is exactly the original code.
        ReadOnlySpan<int> downstream = CollectionsMarshal.AsSpan(_laneSeqPool)
            .Slice(downstreamStart, downstreamCount);
#else
        // netstandard2.1 (Unity/Godot): CollectionsMarshal is unavailable, so copy the small
        // downstream slice into a stack buffer (heap only for the rare long-route case). Not on the
        // parity path -- ns2.1 is not golden-checked -- and functionally identical to the span above.
        Span<int> downstreamBuf = downstreamCount <= 64
            ? stackalloc int[downstreamCount]
            : new int[downstreamCount];
        for (int i = 0; i < downstreamCount; i++)
        {
            downstreamBuf[i] = _laneSeqPool[downstreamStart + i];
        }
        ReadOnlySpan<int> downstream = downstreamBuf;
#endif

        // LANE-CHANGE-OVERLAP (docs/LANE-CHANGE-OVERLAP-DESIGN.md §3 Stage 2): the pool span above is the
        // sequence of EXIT lanes ego converges onto via intra-edge lane changes, which can differ from the
        // ARRIVAL lanes it physically enters at each junction (the C2-v arrival-vs-exit distinction at
        // ~line 9011). When they differ (a multi-lane route with an intra-edge change), the pool scan
        // looks at the wrong lane and misses a leader on ego's actual arrival lane, so ego crosses the
        // junction ON TOP of it (design §2 -- the residual dense-junction overlaps). SUMO's cross-junction
        // getLeader walks the physical path, so scan the ARRIVAL span too and take the tighter follow
        // speed. Purely ADDITIVE braking (Math.Min): it can only ever SLOW ego, matching SUMO's own
        // arrival-lane braking. Byte-identical wherever arrival == exit for every downstream slot (the
        // arrival span then equals the pool span -> same leader/gap/speed), which is every route with no
        // intra-edge lane change.
        var poolFollow = TryFindCrossJunctionLeader(
                ego.Kinematics.Speed, ego.VType, ego, ego.LaneHandle, ego.Kinematics.Pos,
                downstream, new NeighborRearmost(neighbors, ego), dt, out var leader, out var gap)
            ? FollowSpeedFor(
                ego.VType, ego.Kinematics.Speed, gap, leader.Kinematics.Speed, leader.VType.Decel,
                laneVehicleMaxSpeed, dt, time, ref ego.AccControlMode, ref ego.AccLastUpdateTime,
                ref ego.CaccControlMode, ego.Acceleration, hasPred: true,
                predIsCacc: leader.VType.CarFollowModel == "CACC", ego.LevelOfService, _config!.Ballistic)
            : double.PositiveInfinity;

#if NET8_0_OR_GREATER
        ReadOnlySpan<int> arrivalDownstream = CollectionsMarshal.AsSpan(_laneSeqArrival)
            .Slice(downstreamStart, downstreamCount);
#else
        Span<int> arrivalBuf = downstreamCount <= 64 ? stackalloc int[downstreamCount] : new int[downstreamCount];
        for (int i = 0; i < downstreamCount; i++)
        {
            arrivalBuf[i] = _laneSeqArrival[downstreamStart + i];
        }
        ReadOnlySpan<int> arrivalDownstream = arrivalBuf;
#endif
        var arrivalFollow = TryFindCrossJunctionLeader(
                ego.Kinematics.Speed, ego.VType, ego, ego.LaneHandle, ego.Kinematics.Pos,
                arrivalDownstream, new NeighborRearmost(neighbors, ego), dt, out var aLeader, out var aGap)
            ? FollowSpeedFor(
                ego.VType, ego.Kinematics.Speed, aGap, aLeader.Kinematics.Speed, aLeader.VType.Decel,
                laneVehicleMaxSpeed, dt, time, ref ego.AccControlMode, ref ego.AccLastUpdateTime,
                ref ego.CaccControlMode, ego.Acceleration, hasPred: true,
                predIsCacc: aLeader.VType.CarFollowModel == "CACC", ego.LevelOfService, _config!.Ballistic)
            : double.PositiveInfinity;

        return Math.Min(poolFollow, arrivalFollow);
    }

    // Shared cross-junction downstream-leader scan (used by CrossJunctionLeaderConstraint during
    // planning and by TryInsertOnLane for cross-junction insertion safety). Walks `downstreamLanes`
    // (the pool lanes AHEAD of ego, in route order), accumulating distance from ego's front, and
    // returns the nearest leader within the lookahead `dist = SPEED2DIST(maxV) + brakeGap(maxV)`.
    // `rearmostOnLane` returns the lane's rearmost (nearest-to-junction) vehicle -- from the frozen
    // neighbor query during planning, or an ActiveVehicles scan at insertion. gap = (distance to the
    // start of the leader's lane) + leaderBackPos - egoMinGap, matching MSLink::getLeaderInfo's
    // cross-boundary gap. Only the FIRST (nearest) downstream leader is returned -- sufficient for
    // this rung's single-leader anchor; a multi-leader min across several downstream lanes is not
    // needed here (documented).
    //
    // L0d (PERF-ROADMAP.md): `downstreamLanes` is a `ReadOnlySpan<int>` (zero-alloc slice of the
    // caller's pool/array, no `List`/array copy) and `rearmostOnLane` is a by-value struct callback
    // (`where TRearmost : struct, IRearmostSource`) so the JIT specializes the call and no `Func`
    // closure is allocated on the per-vehicle-per-step planning path. The body is otherwise
    // byte-identical to the former IReadOnlyList/Func version.
    private bool TryFindCrossJunctionLeader<TRearmost>(
        double egoSpeed, ResolvedVType egoVType, VehicleRuntime? egoSelf,
        int startLaneHandle, double startPos, ReadOnlySpan<int> downstreamLanes,
        TRearmost rearmostOnLane, double dt,
        out VehicleRuntime leader, out double gap)
        where TRearmost : struct, IRearmostSource
    {
        leader = null!;
        gap = double.PositiveInfinity;

        var startLane = _network!.LanesByHandle[startLaneHandle];
        var maxV = KraussModel.MaxNextSpeed(egoSpeed, egoVType, dt);
        var lookahead = KraussModel.Speed2Dist(maxV, dt) + KraussModel.BrakeGap(maxV, egoVType.Decel, egoVType.Tau, dt);
        var seen = startLane.Length - startPos;

        foreach (var laneHandle in downstreamLanes)
        {
            if (seen > lookahead)
            {
                break;
            }

            var cand = rearmostOnLane.Rearmost(laneHandle);
            if (cand is not null && (egoSelf is null || !ReferenceEquals(cand, egoSelf)))
            {
                gap = seen + (cand.Kinematics.Pos - cand.VType.Length) - egoVType.MinGap;
                leader = cand;
                return true;
            }

            seen += _network.LanesByHandle[laneHandle].Length;
        }

        return false;
    }

    // L0d: a lane's rearmost (nearest-to-junction) vehicle, supplied to TryFindCrossJunctionLeader as
    // a by-value struct so the generic call is JIT-specialized and allocates no closure. Two
    // implementations: NeighborRearmost reads the frozen start-of-step neighbor query (planning);
    // ActiveRearmost scans ActiveVehicles (insertion, before the neighbor query is refilled).
    private interface IRearmostSource
    {
        VehicleRuntime? Rearmost(int laneHandle);
    }

    private readonly struct NeighborRearmost : IRearmostSource
    {
        private readonly LaneNeighborQuery _neighbors;
        private readonly VehicleRuntime _ego;

        public NeighborRearmost(LaneNeighborQuery neighbors, VehicleRuntime ego)
        {
            _neighbors = neighbors;
            _ego = ego;
        }

        public VehicleRuntime? Rearmost(int laneHandle) => _neighbors.GetRearmost(_ego, laneHandle);
    }

    private readonly struct ActiveRearmost : IRearmostSource
    {
        private readonly Engine _engine;

        public ActiveRearmost(Engine engine) => _engine = engine;

        public VehicleRuntime? Rearmost(int laneHandle) => _engine.RearmostOnLaneAmongActive(laneHandle);
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

            // B6: a dodgeable (Width > 0) obstacle only blocks ego LONGITUDINALLY while ego's own
            // lateral footprint still overlaps it. As the car swerves clear (LatOffset drifts away),
            // the overlap ends and this obstacle stops being a leader -- so the car proceeds PAST it
            // instead of braking to a stop. A full-lane obstacle (Width <= 0) always overlaps, so the
            // pre-B6 "stop dead behind it" behaviour is unchanged.
            if (!ObstacleOverlapsLaterally(obstacle, v.Kinematics.LatOffset, v.VType.Width))
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

        // ExternalObstacle is now a value type; capture the resolved leader once (stack copy).
        var near = nearest.Value;
        var gap = nearestBack - v.VType.MinGap - v.Kinematics.Pos;

        // C11-iii: an ExternalObstacle has no CarFollowModel of its own (it is not a SUMO
        // vehicle) -- `predIsCacc` is always false here, exactly matching CACC's own
        // `pred->getCarFollowModel().getModelID() != SUMO_TAG_CF_CACC` ACC-fallback test for any
        // non-CACC leader (a static/moving obstacle is never CACC).
        return FollowSpeedFor(
            v.VType,
            egoSpeed: v.Kinematics.Speed,
            gap: gap,
            predSpeed: near.Speed,
            predMaxDecel: near.Speed != 0.0 ? near.MaxDecel : v.VType.Decel,
            laneVehicleMaxSpeed: laneVehicleMaxSpeed,
            dt: _config!.StepLength,
            time: time,
            accControlMode: ref v.AccControlMode,
            accLastUpdateTime: ref v.AccLastUpdateTime,
            caccControlMode: ref v.CaccControlMode,
            egoAcceleration: v.Acceleration,
            hasPred: true,
            predIsCacc: false,
            levelOfService: v.LevelOfService,
            ballistic: _config!.Ballistic);
    }

    // Cross-regime bridge (Direction B, LONGITUDINAL safety net): brake for a crowd agent directly
    // ahead in ego's path -- the "car stops for a pedestrian it hasn't swerved clear of" behaviour that
    // makes the bridge collision-safe even when a lateral swerve alone cannot clear in time (a fast or
    // still-accelerating vehicle meeting a pedestrian). Exactly mirrors ObstacleConstraint: a crowd
    // agent is a virtual stopped-ish leader (Krauss car-following) ONLY while ego's lateral footprint
    // still overlaps it; as ego swerves aside the overlap ends and this releases, so ego proceeds past
    // (never a permanent block -- the swerve is still the primary manoeuvre, this only covers the gap).
    // Gated on CrowdSource != null -> +Infinity (inert) for every scenario without a coupling attached,
    // so byte-identical (no committed golden sets CrowdSource). Uses the neutral world-disc seam +
    // LaneProjection, NOT the string ExternalObstacle store.
    private double CrowdLongitudinalConstraint(VehicleRuntime v, double time, double laneVehicleMaxSpeed)
    {
        if (CrowdSource is null)
        {
            return double.PositiveInfinity;
        }

        var lane = _network!.LanesByHandle[v.LaneHandle];
        var egoHalf = v.VType.Width * 0.5;
        var (egoX, egoY, _) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, v.Kinematics.LatOffset);
        var radius = v.Kinematics.Speed * 3.0 + v.VType.MinGap + 2.0 * v.VType.Length + 5.0;

        Span<Sim.Core.Bridge.WorldDisc> discs = stackalloc Sim.Core.Bridge.WorldDisc[16];
        var got = CrowdSource.QueryNear(egoX, egoY, radius, discs);

        var nearestBack = double.PositiveInfinity;
        var nearestSpeed = 0.0;
        var found = false;
        for (var d = 0; d < got; d++)
        {
            var disc = discs[d];
            var (offset, latOff, _) = LaneProjection.Project(lane.Shape, disc.X, disc.Y);
            var back = offset - disc.Radius;                 // near (longitudinal) edge of the agent
            if (back < v.Kinematics.Pos)
            {
                continue;                                    // not ahead of ego
            }

            // Lateral overlap with ego's CURRENT footprint. Releases as ego swerves clear (|Δlat| grows
            // past the half-widths), exactly like ObstacleOverlapsLaterally for a dodgeable obstacle.
            if (Math.Abs(latOff - v.Kinematics.LatOffset) >= egoHalf + disc.Radius)
            {
                continue;
            }

            if (back < nearestBack)
            {
                nearestBack = back;
                var (lon, _) = LaneFrameVelocity(lane, offset, disc.Vx, disc.Vy);
                nearestSpeed = Math.Max(0.0, lon);           // approaching agent -> treat as stopped (conservative)
                found = true;
            }
        }

        if (!found)
        {
            return double.PositiveInfinity;
        }

        var gap = nearestBack - v.VType.MinGap - v.Kinematics.Pos;
        return FollowSpeedFor(
            v.VType,
            egoSpeed: v.Kinematics.Speed,
            gap: gap,
            predSpeed: nearestSpeed,
            predMaxDecel: v.VType.Decel,
            laneVehicleMaxSpeed: laneVehicleMaxSpeed,
            dt: _config!.StepLength,
            time: time,
            accControlMode: ref v.AccControlMode,
            accLastUpdateTime: ref v.AccLastUpdateTime,
            caccControlMode: ref v.CaccControlMode,
            egoAcceleration: v.Acceleration,
            hasPred: true,
            predIsCacc: false,
            levelOfService: v.LevelOfService,
            ballistic: _config!.Ballistic);
    }

    // B6 emergency lateral-evasion tuning. Not a SUMO-parity behaviour (SUMO's sublane model does
    // not do emergency ped-swerves) -- this is the external-agent "live reactivity" seam (DESIGN.md),
    // property-tested, inert when no dodgeable obstacle is present.
    private const double SwerveMaxLateralSpeed = 2.0;    // m/s, how fast the car can slide sideways
    private const double SwerveLateralGap = 0.5;         // m clearance kept between the car edge and the ped edge
    private const double SwervePredictionHorizon = 4.0;  // s cap on how far ahead a lunging agent's lateral motion is extrapolated

    // Rung OV3 (opposite-direction overtake execution): how far ego spills toward the oncoming lane
    // while overtaking -- one ego-width plus a clearance gap, positive (LEFT, toward the oncoming
    // lane in right-hand traffic). This puts ego's near edge past a similar-width leader's far edge,
    // so the same-lane !FootprintsOverlap leader bypass lets ego pass it.
    private const double OvertakeSpillGap = 0.5;

    // Phase 2 (sublane, P2.3): the single-vehicle lateral driver -- SUMO's MSLCM_SL2015 preferred-
    // alignment drift. Returns ego's NEW absolute lateral offset (posLat) this step: a bounded drift
    // toward the lane position its latAlignment prefers, at maxSpeedLat. Ported from MSLCM_SL2015:
    //   * the alignment target (MSLCM_SL2015.cpp:1891-1902 _wantsChangeSublane): RIGHT ->
    //     -halfLaneWidth + halfVehWidth, LEFT -> +halfLaneWidth - halfVehWidth, CENTER -> 0, where
    //   * halfVehWidth uses MSLCM_SL2015::getWidth() == vType.Width + NUMERICAL_EPS
    //     (MSLCM_SL2015.cpp:3386) -- this is why SUMO settles at 1.4995, not 1.5, on a 4.8 m lane
    //     with a 1.8 m car ((4.8-1.801)/2 = 1.4995);
    //   * the per-step truncation to maxSpeedLat (MSLCM_SL2015.cpp:2301 computeSpeedLat
    //     `latDist = MAX2(MIN2(latDist, maxDist), -maxDist)`, maxDist = maxSpeedLat*dt), applied here
    //     by DriftToward.
    // For a LONE vehicle the lane-edge safe clamp (mySafeLatDistRight, checkBlocking:2339) exactly
    // coincides with this target (the target IS the edge-keep position, so safe-dist there is 0 and
    // DriftToward never overshoots it) -- so no neighbour gap arithmetic is needed at this rung; the
    // multi-vehicle safe clamp arrives with the per-sublane neighbour query (P2.2). Reads only ego's
    // own frozen state, writes nothing (caller stores the result in ego's MoveIntent) -> parallel-safe.
    // Runs ONLY when _sublane (lateral-resolution > 0); every phase-1 scenario keeps the lane-centred
    // ComputeLateralEvasion path and is byte-identical.
    private double ComputeSublaneLateral(VehicleRuntime v, Lane lane, double dt)
    {
        // DR2 (issue #3): sublane drift toward the alignment target is STEADY and lane-predictable (its
        // latSpeed captures it for LaneArc extrapolation), so it is not a "manoeuvre" for the publish-rate
        // signal. Side-write only; off the golden path.
        v.LateralManoeuvre = false;
        var curLat = v.Kinematics.LatOffset;
        var halfLaneWidth = lane.Width * 0.5;
        // MSLCM_SL2015::getWidth() = vType width + NUMERICAL_EPS (keeps the vehicle NUMERICAL_EPS
        // inside the lane edge -- the source of SUMO's 1.4995 vs the naive 1.5).
        var halfVehWidth = (v.VType.Width + KraussModel.NumericalEps) * 0.5;

        double target = v.VType.LatAlignment switch
        {
            "right" => -halfLaneWidth + halfVehWidth,
            "left" => halfLaneWidth - halfVehWidth,
            // "center"/"default" and every not-yet-ported alignment (nice/compact/arbitrary/numeric
            // offset) hold the centreline for now -- P2.3 ports only the fixed right/left/center
            // alignments a single-vehicle drift exercises; the rest arrive with their own scenarios.
            _ => 0.0,
        };

        // MSLCM_SL2015.cpp:1924: the preferred-alignment drift is applied only when it is larger
        // than NUMERICAL_EPS * actionStepLengthSecs -- SUMO ignores sub-epsilon lateral corrections.
        // This is why a vehicle placed by departPosLat at exactly the lane edge (±(halfLane -
        // width/2), REAL width) does NOT then creep the extra 0.0005 to the alignment target
        // (halfLane - (width+EPS)/2): the 0.0005 correction is below the threshold and skipped.
        var latDist = target - curLat;
        var actionStepLengthSecs = _config!.ActionStepLength > 0 ? _config.ActionStepLength : dt;
        if (Math.Abs(latDist) <= KraussModel.NumericalEps * actionStepLengthSecs)
        {
            return curLat;
        }

        var maxStep = v.VType.MaxSpeedLat * dt;
        return DriftToward(curLat, target, maxStep);
    }

    // Laneless direction (docs/LANELESS-DIRECTION.md), Stage 2: the RECIPROCAL, MULTI-NEIGHBOUR
    // continuous footprint / velocity-obstacle lateral driver. Opt-in (Engine.LanelessRvo); replaces
    // the SUMO-faithful ComputeSublaneLateral drift when set. Returns ego's new absolute posLat this
    // step. Each step ego is pushed away from every laterally-overlapping near-neighbour (same-lane
    // leader AND follower) by HALF the lateral deficit needed to reach `minGapLat` clearance
    // (reciprocal ORCA-style: the neighbour, running the same solve, takes the other half -> mutual
    // separation, no oscillation, no persistent state); when nothing couples it, ego drifts back to
    // centre. All motion is bounded by maxSpeedLat. Because clearing a slower leader by minGapLat
    // makes their footprints disjoint, the existing !FootprintsOverlap same-lane leader bypass
    // (LeaderFollowSpeedConstraint) then lets ego accelerate past -> an EMERGENT overtake in which
    // BOTH vehicles share the lateral move (unlike the Stage-1 one-sided PoC).
    //
    // SUMO's lateral PHYSICS are the reference (minGapLat clearance, maxSpeedLat bound); the exact
    // posLat/timing are our own, validated BEHAVIOURALLY (no-overlap, overtake-completes, recentres,
    // reciprocity) not byte-exact. Parallel-safe: reads only ego's own frozen state + the frozen
    // neighbour query, writes nothing (caller stores the result in ego's own MoveIntent).
    //
    // Neighbours are consumed as neutral value-typed `RvoNeighbor`s (no strings, no dictionary) built
    // from VehicleRuntime today; the Stage-3 scalable int-indexed agent store (replacing the current
    // ExternalObstacle string API -- see LANELESS-DIRECTION.md) will feed the SAME list, unifying
    // external navmesh/RVO agents into this solve without touching it. Full 2D reciprocal ORCA over a
    // fixed-radius spatial-hash query is the further Stage-2/2b work; this is the 1D-lateral reduction
    // (longitudinal stays the validated Krauss car-following).
    private readonly struct RvoNeighbor
    {
        public readonly double Pos;         // longitudinal front position (lane-relative)
        public readonly double Length;
        public readonly double LatOffset;   // lateral centre (+left)
        public readonly double HalfWidth;
        public readonly double Speed;
        // reciprocity share: 0.5 for a SUMO vehicle (it runs the same solve, takes the other half),
        // 1.0 one-sided for an external agent (we don't control it -> ego avoids fully). Populated from
        // the SoA store's per-agent AvoidanceClass byte for external agents (coord B1): Reciprocal -> 0.5,
        // OneSided/StaticBlocker -> 1.0.
        public readonly double Share;

        public RvoNeighbor(double pos, double length, double latOffset, double halfWidth, double speed, double share)
        {
            Pos = pos; Length = length; LatOffset = latOffset; HalfWidth = halfWidth; Speed = speed; Share = share;
        }

        public static RvoNeighbor FromVehicle(VehicleRuntime n) => new(
            n.Kinematics.Pos, n.VType.Length, n.Kinematics.LatOffset, n.VType.Width * 0.5, n.Kinematics.Speed, 0.5);
    }

    private double ComputeRvoLateral(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, double time, double dt)
    {
        var curLat = v.Kinematics.LatOffset;
        var eps = KraussModel.NumericalEps;
        var egoHalf = v.VType.Width * 0.5;
        var maxOffset = lane.Width * 0.5 - egoHalf;    // reachable lateral extent (real width)
        var maxStep = v.VType.MaxSpeedLat * dt;
        var egoDesired = v.VType.MaxSpeed * v.SpeedFactor;

        var egoPos = v.Kinematics.Pos;
        var egoLen = v.VType.Length;
        var minGapLat = v.VType.MinGapLat;
        // Stage 2b: gather ALL near footprint agents -- SUMO vehicles AND external agents -- within a
        // fixed radius into one list, then reduce over them uniformly. This is Seam-1's phase-2 form:
        // the solve consumes a single radius query, not lane-structured leader/follower + a separate
        // obstacle loop. The reaction radius covers the ahead horizon (~3 s) + a car-length behind.
        // Spatial index: the per-lane bucket is already Pos-sorted, so filtering it by radius is the
        // O(k) near-neighbour scan (a genuine cross-lane fixed-radius grid over global x/y -- for a
        // curved/multi-edge laneless space -- is the further step; on the wide single lane the sublane
        // model uses, "same lane within radius" IS the full neighbourhood). MaxRvoNeighbors caps the
        // reduction; denser-than-16 neighbourhoods truncate (behavioural, opt-in).
        const int MaxRvoNeighbors = 16;
        Span<RvoNeighbor> near = stackalloc RvoNeighbor[MaxRvoNeighbors];
        var count = 0;
        var radius = v.Kinematics.Speed * 3.0 + v.VType.MinGap + 2.0 * egoLen + 5.0;

        var laneList = neighbors.OnLane(v.LaneHandle);
        for (var i = 0; i < laneList.Count && count < MaxRvoNeighbors; i++)
        {
            var o = laneList[i];
            if (ReferenceEquals(o, v) || Math.Abs(o.Kinematics.Pos - egoPos) > radius)
            {
                continue;
            }
            near[count++] = RvoNeighbor.FromVehicle(o);
        }

        // External agents on ego's lane within radius, folded into the SAME RvoNeighbor list. A Width<=0
        // agent is a full-lane block ego cannot go around -> excluded here so ObstacleConstraint brakes
        // ego to a stop (the B6 behaviour). This is the Stage-3 adapter now retargeted onto the landed
        // SoA store (ObstacleStore, docs/SUMOSHARP-API.md §4.3-4.4): `_obstacles.Values` materialises each
        // live slot's columns by value, so the frozen `RvoNeighbor` seam reads the SoA columns directly --
        // the solve below never changed. The reciprocity `share` is now SOURCED from the store's
        // per-agent AvoidanceClass byte (coord B1): a Reciprocal agent (a cooperative navmesh/RVO mover
        // running its own solve) -> 0.5; a OneSided/StaticBlocker agent -> 1.0. NB `RvoNeighbor.Share` is
        // currently INERT in this 1D lateral feasible-interval solve -- that solve is inherently one-sided
        // (ego fully clears; reciprocity is emergent from both vehicles running it), so it forbids the full
        // band regardless of share. Share is consumed by the open-space 2D ORCA path (Agent.Responsibility)
        // and reserved for the future unified two-population solver. Wiring it from the class here keeps the
        // seam honest (the field reflects the store's class) with no behavioural change: every committed
        // scenario's obstacles are the default OneSided, and the field is unread here regardless.
        if (_obstacles.Count > 0)
        {
            foreach (var obstacle in _obstacles.Values)
            {
                if (count >= MaxRvoNeighbors)
                {
                    break;
                }
                if (obstacle.Width <= 0.0 || obstacle.LaneId != lane.Id
                    || obstacle.StartTime > time || time >= obstacle.EndTime
                    || Math.Abs(obstacle.FrontPos - egoPos) > radius)
                {
                    continue;
                }
                var share = obstacle.AvoidanceClass == AvoidanceClass.Reciprocal ? 0.5 : 1.0;
                near[count++] = new RvoNeighbor(
                    obstacle.FrontPos, obstacle.Length, obstacle.LatPos, obstacle.Width * 0.5, obstacle.Speed, share);
            }
        }

        // Cross-regime bridge (Direction B -- vehicle avoids crowd): query the open-space CrowdSource
        // for crowd agents near ego's WORLD position and project each onto ego's lane as a one-sided
        // RvoNeighbour, so the feasible-interval solve below forbids their lateral band exactly like a
        // vehicle's or obstacle's. This is how a lane vehicle swerves for a pedestrian. Gated: skipped
        // entirely when no crowd is attached (CrowdSource == null), so byte-identical when unused; and
        // reachable only here, under LanelessRvo && _sublane, so no committed golden is affected. Uses
        // the neutral world-disc seam + LaneProjection (the inverse of PositionAtOffset), NOT the
        // string ExternalObstacle store.
        if (CrowdSource is not null && count < MaxRvoNeighbors)
        {
            var (egoWorldX, egoWorldY, _) = LaneGeometry.PositionAtOffset(lane.Shape, egoPos, curLat);
            Span<Sim.Core.Bridge.WorldDisc> discs = stackalloc Sim.Core.Bridge.WorldDisc[MaxRvoNeighbors];
            var got = CrowdSource.QueryNear(egoWorldX, egoWorldY, radius, discs);
            for (var d = 0; d < got && count < MaxRvoNeighbors; d++)
            {
                var disc = discs[d];
                var (offset, latOff, dist) = LaneProjection.Project(lane.Shape, disc.X, disc.Y);
                // Ignore a crowd agent too far off THIS lane laterally to interact (it belongs to a
                // different corridor); the disc's own radius + ego half-width + minGapLat is the reach.
                if (dist > egoHalf + disc.Radius + minGapLat + maxOffset)
                {
                    continue;
                }

                // Decompose the agent's world velocity into ego's lane frame: longitudinal (for the
                // ahead/behind coupling test) and lateral (to PREDICT where a CROSSING agent will be by
                // the time ego reaches it). Without the lateral prediction the myopic solve dodges to
                // the currently-nearer gap -- which a perpendicular crosser then walks into; predicting
                // its lateral position at time-to-encounter (as B6's ComputeLateralEvasion does for
                // obstacles) makes ego commit to the side the crosser is leaving. Mirrors the swerve's
                // "predicted lateral position" logic exactly.
                var (laneSpeed, latVel) = LaneFrameVelocity(lane, offset, disc.Vx, disc.Vy);
                var gapAhead = offset - disc.Radius - egoPos;
                var tte = gapAhead > 0.0 ? gapAhead / Math.Max(v.Kinematics.Speed, KraussModel.NumericalEps) : 0.0;
                var predictedLat = latOff + latVel * Math.Min(tte, 4.0);   // cap the horizon at 4 s
                near[count++] = new RvoNeighbor(offset, 2.0 * disc.Radius, predictedLat, disc.Radius, laneSpeed, 1.0);
            }
        }

        // Stage 2b-ii: reduce over the gathered neighbours by a 1D lateral FEASIBLE-INTERVAL solve --
        // the half-plane intersection reduced to the lateral axis, which correctly resolves CONFLICTING
        // neighbours (a plain push-sum could strand ego between two that push opposite ways). Each
        // COUPLED neighbour forbids the lateral band where ego would be within minGapLat of it:
        // (n.lat - sep, n.lat + sep). Ego then drifts toward the feasible latOffset CLOSEST TO ITS
        // CURRENT LINE (stickiness -> holds a cleared line, commits to one side; keep-right tie-break),
        // or toward centre when nothing couples it. If NO feasible point exists (neighbours block the
        // whole lane) ego holds and the longitudinal car-following (LeaderFollowSpeedConstraint /
        // ObstacleConstraint, both footprint-gated) brakes it -- the "too narrow -> no overtake"
        // fallback. NB full 2D reciprocal ORCA (both agents share, holonomic, disc-shaped) is the
        // OPEN-SPACE navmesh/RVO layer; lane-derived vehicles are elongated and quasi-1D longitudinally,
        // so a lateral feasible-interval + validated car-following longitudinally is the right model for
        // them (docs/LANELESS-DIRECTION.md).
        //
        // Coupling: a neighbour AHEAD (slower, within the reaction horizon) or OVERLAPPING ego
        // longitudinally forbids a band; one fully BEHIND is ignored (it avoids ego via its own solve).
        // The overlapping case is what yields the reciprocal nudge -- while alongside, ego's forbidden
        // band reaches the neighbour, so the neighbour (running the same solve) also steps aside.
        Span<double> forbLo = stackalloc double[MaxRvoNeighbors];
        Span<double> forbHi = stackalloc double[MaxRvoNeighbors];
        var forbCount = 0;
        var reactionHorizon = v.Kinematics.Speed * 3.0 + v.VType.MinGap;
        for (var k = 0; k < count; k++)
        {
            var n = near[k];
            var gapAhead = n.Pos - n.Length - egoPos;   // >0: neighbour fully ahead
            var gapBehind = egoPos - egoLen - n.Pos;    // >0: neighbour fully behind
            bool couple;
            if (gapAhead > 0.0)
            {
                couple = n.Speed < egoDesired - eps && gapAhead < reactionHorizon;
            }
            else if (gapBehind > 0.0)
            {
                couple = false;                          // fully behind -> it avoids ego, not vice versa
            }
            else
            {
                couple = true;                           // overlapping -> gap-keep (reciprocal nudge)
            }

            if (couple)
            {
                var sep = egoHalf + n.HalfWidth + minGapLat;
                forbLo[forbCount] = n.LatOffset - sep;
                forbHi[forbCount] = n.LatOffset + sep;
                forbCount++;
            }
        }

        double target;
        if (forbCount == 0)
        {
            target = 0.0;                                // nothing couples ego -> recentre
        }
        else
        {
            target = FeasibleClosestTo(curLat, forbLo, forbHi, forbCount, -maxOffset, maxOffset, eps);
            if (double.IsNaN(target))
            {
                target = curLat;                         // fully blocked -> hold; car-following brakes
            }
        }

        // DR2 (issue #3): record whether ego actively coupled to a neighbour/crowd this step -- i.e. it is
        // mid-swerve, so its short-horizon lateral is reactive and NOT linearly lane-predictable. Pure
        // side-write consumed only by the (opt-in) DR classification (Engine.GetDrModel / DrModels);
        // nothing on the golden path reads it, so byte-identical. forbCount > 0 == coupled == FreeKinematic.
        v.LateralManoeuvre = forbCount > 0;

        return DriftToward(curLat, target, maxStep);
    }

    // Cross-regime bridge helper: decompose a world velocity (vx, vy) into ego's lane frame at
    // arc-length `offset` -- LONGITUDINAL (along the tangent, for the ahead/behind coupling test) and
    // LATERAL (along the +left normal, for predicting a crossing agent's lateral drift). Finds the
    // segment containing `offset` and projects onto its unit tangent / left-normal. A pedestrian
    // crossing perpendicular -> ~0 longitudinal (couples as a slow foe) and a large lateral component.
    private static (double Longitudinal, double Lateral) LaneFrameVelocity(Lane lane, double offset, double vx, double vy)
    {
        var shape = lane.Shape;
        if (shape.Count < 2)
        {
            return (0.0, 0.0);
        }

        var remaining = offset;
        for (var i = 0; i < shape.Count - 1; i++)
        {
            var (x1, y1) = shape[i];
            var (x2, y2) = shape[i + 1];
            var dx = x2 - x1;
            var dy = y2 - y1;
            var segLen = Math.Sqrt(dx * dx + dy * dy);
            if (remaining <= segLen || i == shape.Count - 2)
            {
                if (segLen <= 0.0)
                {
                    return (0.0, 0.0);
                }

                var lon = (vx * dx + vy * dy) / segLen;
                var lat = (vx * (-dy) + vy * dx) / segLen;   // +left normal, matching LatOffset's sign
                return (lon, lat);
            }

            remaining -= segLen;
        }

        return (0.0, 0.0);
    }

    // The lateral feasible-interval solve: return the point in [lo, hi] MINUS the union of the
    // `count` forbidden OPEN bands (forbLo[i], forbHi[i]) that is closest to `preferred`, or NaN if
    // none exists (every point is inside some band -> ego cannot clear laterally). Keep-right tie:
    // the smaller (more rightward) point wins an exact tie. The feasible set is a union of closed
    // sub-intervals, so the closest feasible point is either `preferred` itself (if feasible) or a
    // band boundary just outside a forbidden band, or a bound -- a small finite candidate set.
    private static double FeasibleClosestTo(
        double preferred, Span<double> forbLo, Span<double> forbHi, int count, double lo, double hi, double eps)
    {
        var p = Math.Clamp(preferred, lo, hi);
        if (LateralFeasible(p, forbLo, forbHi, count, eps))
        {
            return p;
        }

        var best = double.NaN;
        var bestDist = double.PositiveInfinity;
        ConsiderCandidate(lo, preferred, forbLo, forbHi, count, lo, hi, eps, ref best, ref bestDist);
        ConsiderCandidate(hi, preferred, forbLo, forbHi, count, lo, hi, eps, ref best, ref bestDist);
        for (var i = 0; i < count; i++)
        {
            ConsiderCandidate(forbLo[i] - eps, preferred, forbLo, forbHi, count, lo, hi, eps, ref best, ref bestDist);
            ConsiderCandidate(forbHi[i] + eps, preferred, forbLo, forbHi, count, lo, hi, eps, ref best, ref bestDist);
        }
        return best;
    }

    private static void ConsiderCandidate(
        double x, double preferred, Span<double> forbLo, Span<double> forbHi, int count,
        double lo, double hi, double eps, ref double best, ref double bestDist)
    {
        if (x < lo - eps || x > hi + eps)
        {
            return;
        }
        x = Math.Clamp(x, lo, hi);
        if (!LateralFeasible(x, forbLo, forbHi, count, eps))
        {
            return;
        }
        var dist = Math.Abs(x - preferred);
        // strictly closer, or an exact tie broken keep-right (smaller x wins).
        if (dist < bestDist - eps || (dist <= bestDist + eps && (double.IsNaN(best) || x < best)))
        {
            best = x;
            bestDist = dist;
        }
    }

    private static bool LateralFeasible(double x, Span<double> forbLo, Span<double> forbHi, int count, double eps)
    {
        for (var i = 0; i < count; i++)
        {
            if (x > forbLo[i] + eps && x < forbHi[i] - eps)   // strictly inside a forbidden band
            {
                return false;
            }
        }
        return true;
    }

    // Phase 2 (sublane, P2.2): SUMO's departPosLat initial lateral placement. Returns the vehicle's
    // starting lateral offset (posLat) on its depart lane. Only non-zero when the sublane model is
    // active (_sublane == lateral-resolution > 0); every phase-1 vehicle stays lane-centred (0), so
    // insertion is byte-identical. "left"/"right" put the vehicle's outer edge on the lane border
    // using the REAL vehicle width (±(halfLaneWidth - width/2)) -- SUMO places at ±1.5 on a 4.8 m lane
    // with a 1.8 m car, NOT the width+EPS the alignment target uses; a numeric value is an absolute
    // offset (m, +left). "center"/absent -> 0. Other symbolic values (random/free/...) are out of
    // scope for this rung.
    private double InitialLatOffset(VehicleRuntime v, Lane lane)
    {
        if (!_sublane)
        {
            return 0.0;
        }

        var departPosLat = v.Def.DepartPosLat;
        if (string.IsNullOrEmpty(departPosLat) || departPosLat == "center")
        {
            return 0.0;
        }

        var edge = lane.Width * 0.5 - v.VType.Width * 0.5;
        return departPosLat switch
        {
            "left" => edge,
            "right" => -edge,
            _ when double.TryParse(departPosLat, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var numeric) => numeric,
            _ => throw new NotSupportedException(
                $"departPosLat=\"{departPosLat}\" (vehicle '{v.Def.Id}') is not supported; only " +
                "\"center\"/\"left\"/\"right\"/a numeric offset are in scope for the sublane rungs."),
        };
    }

    // B6: given a dodgeable (Width > 0) obstacle ahead on ego's lane that ego CANNOT stop before,
    // return the vehicle's new lateral offset this step -- a bounded drift toward a target that
    // clears the obstacle (within ego's lane if it fits, otherwise spilling into a SAFE adjacent
    // lane), or toward the lane centre when there is no threat. Returns ego's CURRENT offset (hold)
    // when the obstacle cannot be cleared at all (too wide / neighbours unsafe), leaving
    // ObstacleConstraint to brake to a stop behind it. Reads only the frozen snapshot + `_obstacles`;
    // writes nothing (the caller stores the result in this vehicle's own MoveIntent). Called ONLY in
    // the real plan pass (never the willPass pre-pass), so it never perturbs willPass.
    private double ComputeLateralEvasion(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, double time, double dt)
    {
        var curLat = v.Kinematics.LatOffset;
        var maxStep = SwerveMaxLateralSpeed * dt;

        // DR2 (issue #3): default this step's manoeuvre bit to false; each ACTIVE-steer branch below sets
        // it true (overtake spill, cooperative shift, give-way, an obstacle/crowd swerve or a threat-hold).
        // Steady lane-following / recentre paths leave it false. Pure side-write (read only by the Step
        // DR projection), so byte-identical / off the golden path.
        v.LateralManoeuvre = false;

        // Rung OV3 (opposite-direction overtake execution): while committed to an overtake
        // (OvertakeActive, set by DetectOvertake / OV2's gap acceptance), spill laterally toward the
        // oncoming lane far enough to clear the leader's footprint, so the same-lane
        // !FootprintsOverlap leader bypass (LeaderFollowSpeedConstraint) lets ego accelerate past it.
        // When the intent clears -- ego has passed the leader (no longer held up) or the gap
        // acceptance dropped it because a newly-close oncoming appeared -- OvertakeActive goes false
        // and ego drifts back to its lane centre via the recenter path below. Takes precedence over
        // give-way / obstacle drift: a vehicle mid-overtake is committed to this manoeuvre. Inert for
        // every vehicle with OvertakeActive == false (i.e. every scenario with no lcOpposite vType).
        if (v.OvertakeActive)
        {
            v.LateralManoeuvre = true;
            return DriftToward(curLat, v.VType.Width + OvertakeSpillGap, maxStep);
        }

        // Rung OV4 (cooperative oncoming shift): the mirror of the OV3 spill for the ONCOMING driver.
        // When a spilled overtaker is closing head-on down this vehicle's bidi lane (CooperativeShift,
        // set by DetectCooperativeShift), pull to its OWN outer lane edge -- negative LatOffset (the
        // right/outer side in right-hand traffic, away from the centre line the overtaker is crossing)
        // -- to widen the corridor, reusing the same bounded lateral drift as give-way/overtake. When
        // the intent clears (the overtaker has passed or recentred) it drifts back to centre via the
        // recenter path below. Inert for every vehicle with CooperativeShift == false (i.e. every
        // scenario with no lcOpposite vType). Takes precedence over the give-way / obstacle drift
        // below, which cannot co-occur with an opposite-direction overtake in a supported scenario.
        if (v.CooperativeShift)
        {
            v.LateralManoeuvre = true;
            var outerMargin = Math.Max(0.0, lane.Width / 2.0 - v.VType.Width / 2.0);
            return DriftToward(curLat, -outerMargin, maxStep);
        }

        // Rung ER5 (give-way execution, single-lane fallback): when this vehicle is clearing the way
        // for an approaching EV but has NO lane to change into -- a single lane (no left AND no right
        // neighbour); the multi-lane case is handled by ER4's lane change -- reuse the B6 lateral
        // drift to PULL TO THE LANE EDGE on the give-way side, making room for the EV within the lane
        // (the case SUMO's lane-based rescue cannot form at all). When the intent clears (the EV has
        // passed, GiveWaySide back to 0) it drifts back to centre. This is the ONLY lateral driver on
        // a single lane, so it takes precedence over the external-obstacle evasion below (a single
        // lane with both an active EV and an external obstacle is out of scope).
        var singleLane = lane.LeftNeighbor < 0 && lane.RightNeighbor < 0;
        if (singleLane && v.GiveWaySide != 0)
        {
            v.LateralManoeuvre = true;
            return DriftToward(curLat, GiveWayEdgeTarget(v, lane), maxStep);
        }

        if (_obstacles.Count == 0 && CrowdSource is null)
        {
            // Recentre if a just-cleared give-way (or other) drift left us off-centre; otherwise the
            // unchanged fast path (curLat is already 0 in lane-centred mode -> returns 0 exactly).
            // NB the CrowdSource guard: when a crowd is attached we must fall through to the crowd scan
            // below even with no obstacles. CrowdSource is null for every committed golden, so this is
            // byte-identical there.
            return curLat != 0.0 ? DriftToward(curLat, 0.0, maxStep) : curLat;
        }

        // Nearest dodgeable obstacle that is ahead-or-ALONGSIDE (its front is not yet behind ego's own
        // back) and that a LANE-CENTRED car would hit -- but at the obstacle's PREDICTED lateral
        // position by the time ego reaches it (LatPos + LatSpeed * time-to-encounter), so a pedestrian
        // LUNGING into the lane faster than ego's own swerve speed is detected early. Testing the
        // CENTRED footprint (not ego's current offset) is what makes an in-progress swerve "sticky":
        // once ego has slid aside, the obstacle it is passing still counts as a threat -- so it HOLDS
        // the swerve until the obstacle is fully behind it, instead of recentring alongside and
        // oscillating (when alongside, time-to-encounter -> 0 so the prediction collapses to the
        // current position and the hold is stable).
        ExternalObstacle? threat = null;
        var threatBack = double.PositiveInfinity;
        var threatPredLat = 0.0;
        var threatIsCrowd = false;
        foreach (var o in _obstacles.Values)
        {
            if (o.Width <= 0.0 || o.StartTime > time || time >= o.EndTime || o.LaneId != v.LaneId)
            {
                continue;
            }

            var predLat = PredictedLatPos(o, v);
            if (o.FrontPos < v.Kinematics.Pos - v.VType.Length
                || !FootprintsOverlap(predLat, o.Width, 0.0, v.VType.Width))
            {
                continue; // fully behind ego, or a centred car would clear its predicted position anyway
            }

            var back = o.FrontPos - o.Length;
            // Nearest by back-position; ties broken by Id (ordinal) so the selection is fully
            // order-independent even with multiple overlapping obstacles (never a committed scenario,
            // but keeps the external-agent path deterministic regardless of _obstacles enumeration).
            if (back < threatBack || (back == threatBack && (threat is null || string.CompareOrdinal(o.Id, threat.Value.Id) < 0)))
            {
                threatBack = back;
                threat = o;
                threatPredLat = predLat;
            }
        }

        // Q6 (option b): make CrowdSource crowd agents first-class dodgeable threats in NORMAL mode too,
        // so a vehicle SWERVES around a pedestrian/agent it can clear instead of hard-stopping (the
        // brake-vs-swerve gate below is skipped for a crowd threat -> swerve preferred). Gated on
        // CrowdSource != null, which no committed golden sets and no normal-mode test sets (only the
        // laneless bridge attaches a crowd, and that runs the ComputeRvoLateral path, not this one) --
        // so byte-identical / hash-safe by construction. Each nearby crowd agent is projected onto ego's
        // lane and synthesised as an ExternalObstacle, so the entire B6 selection + swerve machinery
        // (predicted lateral, vacating-side, spill-to-adjacent-lane) applies unchanged. Uses the neutral
        // world-disc seam + LaneProjection, NOT the string obstacle store.
        if (CrowdSource is not null)
        {
            var (egoWX, egoWY, _) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, 0.0);
            var crowdLookahead = v.Kinematics.Speed * SwervePredictionHorizon + v.VType.Length + v.VType.MinGap + 5.0;
            Span<Sim.Core.Bridge.WorldDisc> discs = stackalloc Sim.Core.Bridge.WorldDisc[16];
            var got = CrowdSource.QueryNear(egoWX, egoWY, crowdLookahead, discs);
            for (var d = 0; d < got; d++)
            {
                var disc = discs[d];
                var (offset, latOff, dist) = LaneProjection.Project(lane.Shape, disc.X, disc.Y);

                // Ignore an agent too far off THIS lane laterally to be a same-lane threat.
                if (dist > v.VType.Width / 2.0 + disc.Radius + lane.Width / 2.0)
                {
                    continue;
                }

                var (laneSpeed, latVel) = LaneFrameVelocity(lane, offset, disc.Vx, disc.Vy);
                var width = 2.0 * disc.Radius;
                var synth = new ExternalObstacle(
                    string.Empty, lane.Id, offset + disc.Radius, width,
                    double.NegativeInfinity, double.PositiveInfinity, laneSpeed, 0.0, latOff, width, latVel);

                var predLat = PredictedLatPos(synth, v);
                if (synth.FrontPos < v.Kinematics.Pos - v.VType.Length
                    || !FootprintsOverlap(predLat, width, 0.0, v.VType.Width))
                {
                    continue;   // fully behind ego, or a centred car would clear its predicted position
                }

                var back = synth.FrontPos - synth.Length;
                if (back < threatBack)   // strict: an obstacle wins an exact tie (deterministic)
                {
                    threatBack = back;
                    threat = synth;
                    threatPredLat = predLat;
                    threatIsCrowd = true;
                }
            }
        }

        if (threat is null)
        {
            return DriftToward(curLat, 0.0, maxStep); // no threat -> recentre toward the lane centre
        }

        // ExternalObstacle is now a value type; capture the resolved threat once (stack copy).
        var th = threat.Value;

        // For an OBSTACLE threat: swerve ONLY when braking alone cannot stop before it (the "jumped out,
        // can't stop in time" case) -- while it is still strictly AHEAD and ego is lane-centred and CAN
        // stop, just brake (ObstacleConstraint) and stay centred (the pre-B6 stop-behind behaviour).
        // For a CROWD threat (Q6 option b): PREFER the swerve -- skip this stop-and-stay-centred gate so
        // a dodgeable crowd agent is gone around (decelerating as needed via CrowdLongitudinalConstraint)
        // rather than hard-stopped; if no side is feasible below, ego still holds and brakes. Once ego
        // has committed to a swerve (off-centre) or can no longer stop, it evades either way.
        var stillAhead = threatBack >= v.Kinematics.Pos;
        var gap = threatBack - v.VType.MinGap - v.Kinematics.Pos;
        var brakeGap = KraussModel.BrakeGap(v.Kinematics.Speed, v.VType.Decel, headwayTime: 0.0, dt);
        if (!threatIsCrowd && stillAhead && brakeGap <= gap && Math.Abs(curLat) < 1e-6)
        {
            return DriftToward(curLat, 0.0, maxStep);
        }

        var halfEgo = v.VType.Width / 2.0;
        var halfLane = lane.Width / 2.0;
        // Car-centre offsets that put ego's near edge SwerveLateralGap beyond the obstacle's PREDICTED
        // edge, on the left (higher LatOffset) or right (lower). Using the predicted centre means the
        // car dodges to the side the agent is VACATING: a ped lunging left pushes both targets left, so
        // the right-side target becomes the smaller (and within-lane / safe) steer.
        var targetLeft = (threatPredLat + th.Width / 2.0) + SwerveLateralGap + halfEgo;
        var targetRight = (threatPredLat - th.Width / 2.0) - SwerveLateralGap - halfEgo;

        const double eps = 1e-9;
        // Each side is feasible if it clears the agent WITHIN the ego lane, OR by spilling into an
        // adjacent lane that exists and is gap-safe (reuses the lane-change safety check).
        var leftFeasible = (targetLeft + halfEgo <= halfLane + eps)
            || (lane.LeftNeighbor >= 0 && NeighborSpillSafe(v, lane.LeftNeighbor, neighbors, dt));
        var rightFeasible = (targetRight - halfEgo >= -halfLane - eps)
            || (lane.RightNeighbor >= 0 && NeighborSpillSafe(v, lane.RightNeighbor, neighbors, dt));

        double? chosen;
        if (leftFeasible && rightFeasible)
        {
            // Both sides work: dodge to the side the agent is VACATING -- opposite its lateral velocity
            // -- so the car steers behind the agent's motion rather than into its path. A laterally
            // static agent (LatSpeed == 0) has no vacating side, so take the smaller steer from centre.
            chosen = th.LatSpeed > 0.0 ? targetRight       // agent moving LEFT  -> pass on its right
                : th.LatSpeed < 0.0 ? targetLeft           // agent moving RIGHT -> pass on its left
                : Math.Abs(targetLeft - curLat) <= Math.Abs(targetRight - curLat) ? targetLeft : targetRight;
        }
        else if (leftFeasible)
        {
            chosen = targetLeft;
        }
        else if (rightFeasible)
        {
            chosen = targetRight;
        }
        else
        {
            v.LateralManoeuvre = true;   // engaged with a threat it cannot clear (about to hold + brake)
            return curLat; // cannot dodge -> hold; ObstacleConstraint brakes to a stop behind it
        }

        v.LateralManoeuvre = true;       // actively swerving around the obstacle/crowd threat
        return DriftToward(curLat, chosen.Value, maxStep);
    }

    // B6: does the obstacle's lateral footprint [LatPos +/- Width/2] overlap ego's [egoLat +/-
    // egoWidth/2]? A Width <= 0 obstacle is the pre-B6 FULL-LANE block: always overlapping. (Uses the
    // obstacle's CURRENT LatPos -- ObstacleConstraint's immediate-braking check; the evasion's
    // anticipatory check uses the PREDICTED position via FootprintsOverlap + PredictedLatPos instead.)
    private static bool ObstacleOverlapsLaterally(ExternalObstacle obstacle, double egoLat, double egoWidth)
    {
        if (obstacle.Width <= 0.0)
        {
            return true;
        }

        return FootprintsOverlap(obstacle.LatPos, obstacle.Width, egoLat, egoWidth);
    }

    // B6: do two lateral footprints (centre +/- width/2) overlap?
    private static bool FootprintsOverlap(double aLat, double aWidth, double bLat, double bWidth)
    {
        var aLo = aLat - aWidth / 2.0;
        var aHi = aLat + aWidth / 2.0;
        var bLo = bLat - bWidth / 2.0;
        var bHi = bLat + bWidth / 2.0;
        return aLo < bHi && aHi > bLo;
    }

    // B6-lat: the obstacle's lateral centre PREDICTED forward to the moment ego reaches it --
    // LatPos + LatSpeed * timeToEncounter, where timeToEncounter is the longitudinal distance to the
    // obstacle divided by the closing speed, clamped to [0, SwervePredictionHorizon]. Alongside (or
    // for a laterally-static agent) this is just LatPos, so it degrades to the un-predicted behaviour.
    private double PredictedLatPos(ExternalObstacle o, VehicleRuntime v)
    {
        if (o.LatSpeed == 0.0)
        {
            return o.LatPos;
        }

        var longDist = Math.Max((o.FrontPos - o.Length) - v.Kinematics.Pos, 0.0);
        var closing = Math.Max(v.Kinematics.Speed - o.Speed, 0.5); // >= 0.5 m/s so a near-stationary ego still predicts
        var tte = Math.Min(longDist / closing, SwervePredictionHorizon);
        return o.LatPos + o.LatSpeed * tte;
    }

    // B6: move `cur` toward `target` by at most `maxStep` (the per-step lateral-speed cap).
    private static double DriftToward(double cur, double target, double maxStep)
    {
        var delta = target - cur;
        if (delta > maxStep)
        {
            delta = maxStep;
        }
        else if (delta < -maxStep)
        {
            delta = -maxStep;
        }

        return cur + delta;
    }

    // B6: is spilling into the given adjacent lane safe -- reuses the lane-change gap check against
    // that neighbour lane's leader/follower (the same IsTargetLaneSafe the strategic/speed-gain
    // changes use), so a swerve never cuts off a neighbour-lane vehicle.
    private bool NeighborSpillSafe(VehicleRuntime ego, int neighborHandle, LaneNeighborQuery neighbors, double dt)
    {
        var neighLead = neighbors.GetNeighborLeader(ego, neighborHandle);
        var neighFollow = neighbors.GetNeighborFollower(ego, neighborHandle);
        return IsTargetLaneSafe(ego, neighLead, neighFollow, dt);
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
    private void ExecuteMoves(double time, double dt)
    {
        // Domain decomposition: region-parallel execute when opted in AND there are no actuated TLS
        // (whose induction-loop detector feed is a shared write -- ExecuteMoveVehicle skips it on this
        // path anyway, but gate to be safe). Per-vehicle moves are independent (each writes only its
        // own Kinematics/lane); arrivals go through the now-thread-safe command buffer and apply
        // order-independently at Flush, so the result is byte-identical regardless of thread timing.
        if (RegionPlan && _actuatedLogics.Count == 0)
        {
            var vehicles = _vehicles;
            System.Threading.Tasks.Parallel.For(0, _regionCount, _parallelOptions, r =>
            {
                var list = _regionActive[r];
                for (var idx = 0; idx < list.Count; idx++)
                {
                    ExecuteMoveVehicle(vehicles[list[idx]], time, dt);
                }
            });

            _commandBuffer.Flush();
            // GAP-2: read right after Flush (guaranteed serial even though the moves above ran
            // region-parallel -- Parallel.For blocks until every region finishes) -- see
            // CaptureCompletedTrips' own comment.
            CaptureCompletedTrips();
            return;
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            ExecuteMoveVehicle(v, time, dt);
        }

        _commandBuffer.Flush();
        CaptureCompletedTrips();
    }

    // GAP-2 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §2): drains CommandBuffer.ArrivedThisFlush (the
    // vehicles this ExecuteMoves' Flush() just marked Arrived via DestroyWithArrival -- i.e. genuine
    // route-end arrivals, never a jam-teleport removal or X1 de-jam despawn) into `_completedTrips`.
    // Called only from ExecuteMoves, right after `_commandBuffer.Flush()`, which is ALWAYS a serial
    // point (even on the region-parallel path, Parallel.For has already joined by the time Flush()
    // runs) -- so this method itself never needs its own locking despite ExecuteMoveVehicle's
    // DestroyWithArrival calls being recorded from parallel workers.
    private void CaptureCompletedTrips()
    {
        var arrivals = _commandBufferImpl.ArrivedThisFlush;
        if (arrivals.Count == 0)
        {
            return;
        }

        // Determinism (CLAUDE.md iron law): under RegionPlan, DestroyWithArrival calls are recorded
        // from multiple worker threads under CommandBuffer's own lock, so ArrivedThisFlush's order
        // reflects non-deterministic thread-interleaving timing, not a reproducible ordering. Re-sort
        // by EntityIndex before appending -- the SAME "repeatable parallel simulation" discipline
        // ProcessTransferQueue/CheckJamTeleports already apply to their own parallel-collected
        // candidate lists, so CompletedTrips' order is deterministic regardless of RegionPlan/thread
        // scheduling.
        var ordered = new List<(VehicleRuntime Vehicle, double ArrivalTime)>(arrivals);
        ordered.Sort(static (a, b) => a.Vehicle.EntityIndex.CompareTo(b.Vehicle.EntityIndex));

        foreach (var (v, arrivalTime) in ordered)
        {
            _completedTrips.Add(BuildCompletedTripInfo(v, arrivalTime));
        }
    }

    // GAP-2: builds one CompletedTripInfo from a just-arrived vehicle's SETTLED state (LaneId/
    // Kinematics/TripWaitingTime/TripTimeLoss were all finalized by ExecuteMoveVehicle before it
    // called DestroyWithArrival -- Flush's Kind.Destroy case only sets Arrived=true, it does not
    // mutate any of these). See CompletedTripInfo's own header comment for the field-by-field
    // citation of each SUMO formula.
    private CompletedTripInfo BuildCompletedTripInfo(VehicleRuntime v, double arrivalTime)
    {
        // Configured arrival pos: the arrival edge's lane length (no scenario sets a literal
        // <vehicle arrivalPos=>, so SUMO's MIN2(param.arrivalPos, lastLaneLength) default -- the
        // last lane's length -- is exact for every committed route). v.LaneId/v.LaneHandle are the
        // FINAL (arrival) lane -- the arrival branch in ExecuteMoveVehicle breaks before advancing
        // past it.
        var arrivalLane = _network!.LanesByHandle[v.LaneHandle];
        var arrivalPos = arrivalLane.Length;

        // routeLength = (running distance travelled) + arrivalPos, matching SUMO's MSDevice_Tripinfo
        // `myRouteLength + arrivalPos`. RouteDistanceTraveled is the running accumulator seeded at
        // -departPos and grown by each left lane's length in ExecuteMoveVehicle (incl. internal
        // junction lanes); using the accumulator instead of re-summing the CURRENT lane-sequence pool
        // is what makes routeLength correct across a device.rerouting reroute (which rebuilds the pool
        // for only the remaining route -- the pool-sum lost all pre-reroute distance). For a
        // non-rerouted trip the accumulator equals the old pool-sum exactly, so scenarios 66/72 stay
        // byte-identical.
        var routeLength = v.RouteDistanceTraveled + arrivalPos;

        return new CompletedTripInfo(
            Id: v.Def.Id,
            Depart: v.Def.Depart,
            Arrival: arrivalTime,
            ArrivalLane: v.LaneId,
            ArrivalPos: arrivalPos,
            ArrivalSpeed: v.Kinematics.Speed,
            Duration: arrivalTime - v.Def.Depart,
            RouteLength: routeLength,
            WaitingTime: v.TripWaitingTime,
            TimeLoss: v.TripTimeLoss);
    }

    // One vehicle's move (integration + lane-boundary wrap + arrival), extracted so ExecuteMoves can
    // run it serially or region-parallel. Writes only this vehicle's own state and records arrivals to
    // the thread-safe command buffer. The actuated-TLS detector feed inside is a shared write, so the
    // region-parallel path is gated on there being no actuated programs.
    private void ExecuteMoveVehicle(VehicleRuntime v, double time, double dt)
    {
            // C8-i: capture the pre-move speed BEFORE overwriting it, for the ballistic
            // trapezoidal position update below (Euler ignores it).
            var oldSpeed = v.Kinematics.Speed;
            // C6-ii: capture the pre-move lane + position for the induction-loop detector feed
            // below (before the lane-boundary wrap advances LaneHandle/Pos).
            var detLaneHandle = v.LaneHandle;
            var detOldPos = v.Kinematics.Pos;
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
            // else reset. Written unconditionally for every vehicle; read by the all-way-stop arm of
            // JunctionYieldConstraint and (P1F) by the jam-teleport check.
            // P1F-1 (HIGH-DENSITY-P1F-DESIGN.md §6 risk 2): the SUMO `(!isStopped()||isIdling())`
            // factor -- a vehicle currently AT a scheduled <stop> (IsStoppedAtStop, its front stop
            // reached) does NOT accumulate WaitingTime, so a parked <stop>-blocker never teleports.
            // Byte-identical for the existing suite: scenarios with no stops never hit IsStoppedAtStop
            // (GetStops returns null), and a stopped vehicle's WaitingTime is never read by any
            // committed scenario (none pairs a scheduled stop with an allway_stop junction / teleport).
            var stoppedAtStopThisMove = IsStoppedAtStop(v);
            var haltedLowAccelThisMove = v.Intent.NewSpeed <= KraussModel.HaltingSpeed && v.Acceleration <= 0.5 * v.VType.Accel;
            v.WaitingTime = haltedLowAccelThisMove && !stoppedAtStopThisMove
                ? v.WaitingTime + dt
                : 0.0;

            // GAP-2: SUMO applies a stop's reached/duration-decrement transition SYNCHRONOUSLY inside
            // the SAME executeMove() call that later runs updateWaitingTime/updateTimeLoss/notifyMove
            // (MSVehicle::processNextStop is called BEFORE those, MSVehicle.cpp's own executeMove
            // ordering) -- so on the exact step a vehicle's front stop transitions (just reached, or
            // just resumed), SUMO's isStopped() already reflects THIS step's new state. Our engine
            // instead defers applying `stop.Reached` to the "apply stop-queue update" block further
            // down THIS SAME method (D5's command-buffer-shaped deferred-mutation discipline) -- so
            // `stoppedAtStopThisMove` above (IsStoppedAtStop, a plain field read) is stale by one step
            // exactly at a transition. v.Intent.StopUpdate is this step's ALREADY-DECIDED target
            // (ProcessNextStop, computed in the Plan phase) -- reading `.Reached` from it when present
            // gives the post-transition value SUMO's synchronous ordering would see, matching
            // MSDevice_Tripinfo's real tripinfo output exactly (verified against scenario
            // 66-tripinfo-arrivallane's golden: the leader's front stop transition step must NOT
            // accumulate waitingTime/timeLoss, or its golden waitingTime=0.00 does not reproduce).
            // Deliberately NOT applied to the existing WaitingTime field above (unchanged, per its own
            // "byte-identical for the existing suite" comment) -- no committed scenario pairs a
            // scheduled stop with the allway_stop/jam-teleport readers of that field, so this
            // corrected predicate is scoped to the two NEW trip-total accumulators only.
            var stoppedAtStopForTrip = v.Intent.StopUpdate?.Reached ?? stoppedAtStopThisMove;

            // GAP-2 (MSDevice_Tripinfo::notifyMove, MSDevice_Tripinfo.cpp:179-193): the tripinfo
            // device's own trip-TOTAL waitingTime -- the SAME halted+low-accel+not-at-stop predicate
            // as WaitingTime just above (using the corrected stoppedAtStopForTrip), but accumulated
            // WITHOUT ever resetting (see VehicleRuntime.TripWaitingTime's own comment).
            if (haltedLowAccelThisMove && !stoppedAtStopForTrip)
            {
                v.TripWaitingTime += dt;
            }

            // GAP-2 (MSVehicle::updateTimeLoss, MSVehicle.cpp:4095-4105): trip-TOTAL "how much slower
            // than free-flow was I", gated on the SAME corrected !isStopped() predicate. vmax is THIS
            // lane's speed-limit x speedFactor cap -- the exact laneVehicleMaxSpeed convention every
            // car-following constraint above already uses (KraussModel.LaneVehicleMaxSpeed), evaluated
            // on v.LaneHandle BEFORE the lane-boundary-crossing loop below advances it, matching SUMO's
            // own pre-processLaneAdvances timing.
            if (!stoppedAtStopForTrip)
            {
                var tripVMax = KraussModel.LaneVehicleMaxSpeed(_network!.LanesByHandle[v.LaneHandle].Speed, v.SpeedFactor, v.VType);
                if (tripVMax > 0.0)
                {
                    v.TripTimeLoss += dt * (tripVMax - v.Intent.NewSpeed) / tripVMax;
                }
            }
            v.Kinematics.Pos += _config!.Ballistic
                ? 0.5 * (oldSpeed + v.Intent.NewSpeed) * dt
                : v.Intent.NewSpeed * dt;
            // Phase 2 (P2.1/P2.3): lateral velocity = this step's lateral displacement / dt (computed
            // from the OLD offset before it is overwritten). 0 for every lane-centred vehicle (Intent
            // .LatOffset == current == 0), so inert for phase-1; not hashed/compared, so byte-identical.
            v.Kinematics.LatSpeed = (v.Intent.LatOffset - v.Kinematics.LatOffset) / dt;
            v.Kinematics.LatOffset = v.Intent.LatOffset;

            // C6-ii: feed the actuated-TLS induction loops this vehicle's within-step motion along
            // its START-of-step lane, using the RAW advanced position (before the lane-boundary wrap
            // below re-bases Pos onto the next lane) so a vehicle whose back crosses the detector
            // while its front has already left the lane is still counted in this lane's coordinate.
            // `time + dt` is the FCD time this move produces (SUMO's SIMTIME during executeMove),
            // which MSInductLoop stamps entry/leave with. No-op when there are no actuated programs.
            if (_actuatedLogics.Count > 0)
            {
                foreach (var actuated in _actuatedLogics.Values)
                {
                    actuated.NotifyMove(
                        detLaneHandle, v.EntityIndex,
                        detOldPos, v.Kinematics.Pos, oldSpeed, v.Intent.NewSpeed, v.VType.Length, time + dt);
                }
            }

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
                    var resumedStop = stops.Dequeue();
                    // GAP-3: pulling out of a parkingArea stop clears IsParked the SAME step (matches
                    // scenario 48's golden: y snaps back to lane-centre the step the vehicle resumes,
                    // not one step later) -- see VehicleRuntime.IsParked's own header comment. A no-op
                    // for a plain lane stop (IsParking false), so byte-identical for scenarios
                    // 03/13/44.
                    if (resumedStop.IsParking)
                    {
                        v.IsParked = false;
                    }
                }
                else
                {
                    var stop = stops.Peek();
                    stop.Reached = stopUpdate.Reached;
                    stop.RemainingDuration = stopUpdate.RemainingDuration;
                    // GAP-3: this arm only ever sets Reached=true (see StopTransition's two
                    // constructions in ProcessNextStop), so `stop.Reached` here is always true --
                    // written as `stop.Reached` rather than a literal `true` for clarity/robustness.
                    // No-op for a plain lane stop (IsParking false) -- byte-identical for 03/13/44.
                    if (stop.IsParking)
                    {
                        v.IsParked = stop.Reached;
                    }
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
                // Strict `>` boundary, matching MSVehicle::processLaneAdvances
                // (MSVehicle.cpp:4282 `if (myState.myPos > myLane->getLength())`): a vehicle only
                // leaves its lane once its position STRICTLY exceeds the lane length. Landing
                // EXACTLY at the lane end (pos == length) keeps it on the current lane at
                // pos == length -- it crosses next step. This is measure-zero in free flow (every
                // pre-C4-iii scenario is unaffected), but a vehicle braking for a downstream slower
                // lane routinely lands exactly on the boundary (the roundabout's ring-lane entry,
                // where the successive-lane cap sets speed = remaining distance to the lane end).
                if (v.Kinematics.Pos <= currentLane.Length)
                {
                    break;
                }

                if (v.LaneSeqIndex + 1 >= v.LaneSeqLen)
                {
                    // Last route edge: the vehicle runs off the end of its route and ARRIVES.
                    // C4-vii-b: this check precedes the pool-convergence guard below because SUMO's
                    // arrival is position-based on the FINAL edge and lane-AGNOSTIC (MSVehicle removes
                    // the vehicle when it passes arrivalPos on whatever lane it currently occupies).
                    // A legitimate keep-right onto a SIBLING lane of the same final edge -- which SUMO
                    // itself performs (verified on scenarios/45: a through-vehicle keep-rights onto its
                    // arrival edge's right lane and STILL arrives) -- must therefore arrive here, not
                    // strand at the lane end. The convergence guard is only meaningful for a MID-route
                    // edge, where the NEXT edge requires a specific connecting lane; on the last edge
                    // there is no next edge, so requiring exact-pool-lane arrival was a bug that froze
                    // any final-edge lane-changer at speed 0 forever. Byte-identical for every existing
                    // scenario: their vehicles reach the last lane end already ON the pool lane, so both
                    // orderings arrive them identically.
                    //
                    // D5: deferred through the command buffer, flushed at the END of this method's
                    // outer foreach (see below) -- safe because the `break` right after this, not the
                    // `while (!v.Arrived)` condition, is what exits this loop (the condition is never
                    // RE-evaluated after this assignment within this same call), and nothing later in
                    // this vehicle's own iteration or any OTHER vehicle's iteration this SAME
                    // ExecuteMoves pass reads v.Arrived (the outer foreach's own `if (!v.Inserted ||
                    // v.Arrived) continue;` guard only ever reads a vehicle's OWN Arrived value, set at
                    // the top of ITS OWN iteration, never another vehicle's just-this-step arrival).
                    //
                    // GAP-2: DestroyWithArrival (via the concrete `_commandBufferImpl`, not the
                    // ICommandBuffer-typed `_commandBuffer` field -- see that method's own comment)
                    // instead of plain Destroy -- this IS a genuine SUMO tripinfo ARRIVED event (the
                    // OTHER three Destroy call sites, jam-teleport removal and X1 de-jam despawn, are
                    // not, so they keep calling plain Destroy). `time + dt` is this step's end-of-
                    // interval sim time, matching SUMO's notifyLeave `getCurrentTimeStep()` convention
                    // (verified against the regenerated scenario 66 golden.tripinfo.xml). Still
                    // thread-safe under the region-parallel path: DestroyWithArrival takes the same
                    // `_recordLock` Destroy does.
                    //
                    // Issue 1 residency guard (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §7,
                    // defense-in-depth alongside the TryStrategicLaneChange fix above): a
                    // park-and-stay vehicle -- one whose front stop is an UNREACHED parkingArea
                    // stop (StopRuntime.IsParking) -- must never be treated as "arrived" merely
                    // because it ran off the position-based end of its final route edge; it has
                    // not parked yet, so removing it here would silently vanish a car that is
                    // supposed to stay resident for the whole horizon (the product ruling in §7:
                    // "cars must remain resident... for the full horizon"). With the strategic-LC
                    // fix above the car brakes and parks well before reaching this point in the
                    // normal case, so this rarely fires; it exists to guarantee a wrong-lane car
                    // is clamped and kept alive (exactly like the drop-lane guard a few lines
                    // below) rather than silently destroyed. GATED on GetStops(v)'s front stop
                    // being BOTH unreached AND IsParking -- every existing scenario either has no
                    // stop on its final edge, or already has it Reached/non-parking by the time
                    // arrival is checked, so this is byte-identical elsewhere.
                    var frontStopAtArrival = GetStops(v) is { Count: > 0 } arrivalStops ? arrivalStops.Peek() : null;
                    if (frontStopAtArrival is { Reached: false, IsParking: true })
                    {
                        v.Kinematics.Pos = currentLane.Length;
                        v.Kinematics.Speed = 0.0;
                        break;
                    }

                    _commandBufferImpl.DestroyWithArrival(v, time + dt);
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
                // connected to. (Only reached for a NON-last edge now -- the last-edge arrival
                // above already returned.)
                if (v.LaneHandle != _laneSeqPool[v.LaneSeqStart + v.LaneSeqIndex])
                {
                    // C4-vii-c (route->lane over-constraint fix -- the -L2 grid flow blocker): the
                    // vehicle reached this lane end on a lane that is NOT the pool's resolved exit for
                    // this edge (its strategic lane change onto the pool lane never completed -- e.g.
                    // that lane was jammed by cross traffic). The pool over-constrains: on a grid,
                    // MULTIPLE lanes of an edge connect to the same next edge (a straight move leaves
                    // from either lane), yet the pool pins ONE exit lane (chasing a downstream
                    // bestLaneOffset hint), so a vehicle sitting on the OTHER, equally-valid connecting
                    // lane was stranded at speed 0 forever -- the dominant cause of the committed diag
                    // grid's gridlock (29 of 38 stuck). SUMO never strands it: it leaves via whatever
                    // connection its actual lane has to the next route edge and keeps lane-changing
                    // toward the hint opportunistically on later edges (MSVehicle::updateBestLanes'
                    // bestLaneOffset is a hint, not a hard gate). So if THIS lane still connects onward,
                    // re-resolve the remaining route from it and proceed; only clamp when the lane
                    // genuinely does not connect (a true drop lane -- the original guard's purpose).
                    if (TryReResolveFromActualLane(v, currentLane))
                    {
                        continue;
                    }

                    v.Kinematics.Pos = currentLane.Length;
                    v.Kinematics.Speed = 0.0;
                    break;
                }

                // GAP-2 follow-up: the vehicle is LEAVING currentLane -- accumulate its full length
                // into the running routeLength total (SUMO's MSDevice_Tripinfo += lane.getLength() on
                // each NOTIFICATION_JUNCTION leave). This is the ONE site a lane is fully left, so it
                // captures the internal junction lanes AND survives a device.rerouting pool rebuild --
                // see VehicleRuntime.RouteDistanceTraveled.
                v.RouteDistanceTraveled += currentLane.Length;
                v.Kinematics.Pos -= currentLane.Length;
                v.LaneSeqIndex++;
                // C2-v: land on the new slot's ARRIVAL lane (the lane physically entered via the
                // incoming connection), NOT its Exit/pool lane. They differ only at an intra-edge
                // mid-route lane change: the vehicle arrives on lane A here and the strategic lane
                // change (TryStrategicLaneChange, target = pool[slot]) then converges it onto the
                // Exit lane B before the NEXT edge boundary (the convergence guard above waits for
                // actual == pool[slot]). For every route with no intra-edge change Arrival == Exit,
                // so this is byte-identical to reading _laneSeqPool. D3: direct pool-slice read.
                v.LaneHandle = _laneSeqArrival[v.LaneSeqStart + v.LaneSeqIndex];
                v.LaneId = _network.LanesByHandle[v.LaneHandle].Id;
            }

            // Rung 8b/A2: keep-right and speed-gain lane changes are no longer decided here --
            // both now run in the post-move DecideSpeedGainChanges phase (see Run()'s comment and
            // that method's header comment for why keep-right moved out of Plan/MoveIntent).
    }

    // C4-vii-c: re-resolve a vehicle's remaining route starting from the lane it is ACTUALLY on when
    // it reaches a lane boundary NOT on the pool's resolved exit lane (its strategic lane change to
    // that exit never completed). Returns true -- and splices a fresh pool/arrival slice into the
    // shared pool, with LaneSeqIndex reset to this lane's slot -- when the actual lane connects to the
    // next route edge (so the crossing can proceed via THAT lane's connection); returns false when it
    // does not (a genuine drop lane -> the caller clamps at the lane end, the original guard's job).
    // SUMO-faithful: a vehicle follows whatever connection leaves its current lane and lane-changes
    // toward bestLaneOffset opportunistically, never stranding itself on an equally-valid connecting
    // lane (see ResolveLaneSequenceHandlesWithArrival's `forceFirstExitToArrival`). Only ever reached
    // from the boundary branch that used to unconditionally clamp -- so every scenario whose vehicles
    // reach boundaries ON their pool lane (every committed golden) never calls this and is
    // byte-identical. Direct v.LaneSeq* writes are safe: this is the execute phase, each vehicle
    // mutates only its own state, and nothing later this step reads another vehicle's LaneSeq*.
    private bool TryReResolveFromActualLane(VehicleRuntime v, Lane currentLane)
    {
        // The remaining NORMAL route edges from the current slot onward (skip internal ':'-edge
        // lanes, which are junction interiors, not route edges).
        var remaining = new List<string>();
        string? lastEdge = null;
        for (var k = v.LaneSeqIndex; k < v.LaneSeqLen; k++)
        {
            var lane = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + k]];
            if (lane.EdgeId.Length > 0 && lane.EdgeId[0] == ':')
            {
                continue;
            }

            if (lane.EdgeId != lastEdge)
            {
                remaining.Add(lane.EdgeId);
                lastEdge = lane.EdgeId;
            }
        }

        // The current slot is on currentLane's own edge; the last-edge arrival check upstream already
        // returned for a route end, so a next edge must exist. Bail defensively otherwise.
        if (remaining.Count < 2 || remaining[0] != currentLane.EdgeId)
        {
            return false;
        }

        // Genuine drop lane: the actual lane has no connection to the next route edge -> clamp.
        if (!_network!.ConnectionsByFromLaneTo.ContainsKey((currentLane.EdgeId, currentLane.Index, remaining[1])))
        {
            return false;
        }

        // Re-resolve from the actual lane (pinning this edge's exit to it) and splice a fresh slice
        // into the shared pool -- the old slice is abandoned, the pool only grows (the same
        // discipline UpdateReroutes' ReplaceRoute uses). Guarded against an unroutable parallel path
        // (ResolveSequenceCore throws if some downstream edge has no connecting lane): on failure,
        // fall back to the clamp rather than crash.
        int[] pool;
        int[] arrival;
        try
        {
            // Issue 1 cross-edge fix: see RerouteActive's matching comment -- a park-and-stay stop
            // still further down `remaining` must keep steering the re-resolved pool toward its lane.
            var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, remaining);
            (pool, arrival) = _network.ResolveLaneSequenceHandlesWithArrival(remaining, currentLane.Index, forceFirstExitToArrival: true, stopOverride: stopOverride);
        }
        catch (InvalidDataException)
        {
            return false;
        }

        // P2G-2 robustness: atomic Count-read + append -- ExecuteMoves is region-parallel, and the
        // coordinated model reaches this re-resolve concurrently from many workers. Without the lock the
        // Count reads race and slices overlap, corrupting _laneSeqPool's size (IndexOutOfRange downstream).
        // The offset chosen depends on lock order, but it is internal bookkeeping -- the slice CONTENT is
        // the vehicle's own, so the trajectory stays deterministic (serial vs --region byte-identical).
        int start;
        lock (_laneSeqPoolLock)
        {
            start = _laneSeqPool.Count;
            _laneSeqPool.AddRange(pool);
            _laneSeqArrival.AddRange(arrival);
        }
        v.LaneSeqStart = start;
        v.LaneSeqLen = pool.Length;
        v.LaneSeqIndex = 0;
        // The remaining route's lane assignment changed -> the keep-right stayOnBest memo may be
        // stale even on the same lane (same reasoning as CommandBuffer.ReplaceRoute's own reset).
        v.KeepRightStayCacheLane = -1;
        return true;
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

        // Domain decomposition: region-parallel the WHOLE post-move phase. Each vehicle's speed-gain/
        // keep-right decision reads ONLY the frozen postMoveNeighbors snapshot + immutable network +
        // the ConcurrentDictionary best-lanes cache, and writes ONLY its own fields + the order-
        // independent, thread-safe command buffer (see DecideSpeedGainForVehicle) -- so both the
        // neighbour refill (each region owns disjoint lanes) and the decision loop parallelize
        // byte-identically. The active set + lanes here are the POST-move ones (execute already
        // applied moves/arrivals), so the region grouping is rebuilt from current lanes, distinct from
        // the pre-move grouping built at the top of the step.
        if (RegionPlan && ShouldParallelizePlan())
        {
            BuildActiveIndices();
            BuildRegionActive();
            System.Threading.Tasks.Parallel.For(0, _regionCount, _parallelOptions, r =>
                postMoveNeighbors.RefillRegion(_regionActive[r], _vehicles, _regionLanes[r]));

            var act = _activeIndices;
            System.Threading.Tasks.Parallel.For(0, act.Count, _parallelOptions, ai =>
                DecideSpeedGainForVehicle(_vehicles[act[ai]], postMoveNeighbors, time, dt, actionStepLengthSecs));
        }
        else
        {
            postMoveNeighbors.Refill(ActiveVehicles());

            // D6: the Query() analog -- see ActiveVehicles()'s own comment.
            foreach (var v in ActiveVehicles())
            {
                DecideSpeedGainForVehicle(v, postMoveNeighbors, time, dt, actionStepLengthSecs);
            }
        }

        // D5: apply every ChangeLane recorded above, in record order, at this method's end -- the SAME
        // point v.LaneId/LaneHandle took effect at before this rung (DecideSpeedGainChanges is the LAST
        // phase in Run()'s per-step loop). Safe under the parallel record above: each command targets a
        // distinct vehicle and Flush applies them order-independently (see CommandBuffer's header).
        _commandBuffer.Flush();
    }

    // One vehicle's post-move keep-right / strategic / speed-gain lane-change decision, extracted from
    // DecideSpeedGainChanges so the decision loop can run concurrently. Reads only the frozen
    // `postMoveNeighbors` snapshot (refilled once, before the loop) + the immutable network + the
    // ConcurrentDictionary best-lanes cache; writes only `v`'s own fields (SpeedGainProbability, and an
    // inline keep-right LaneId swap that only v's own decision re-reads) and the thread-safe command
    // buffer. No cross-vehicle live read and no shared non-concurrent write, so running this
    // concurrently over distinct vehicles is byte-identical to the serial foreach it replaced.
    private void DecideSpeedGainForVehicle(VehicleRuntime v, LaneNeighborQuery postMoveNeighbors, double time, double dt, double actionStepLengthSecs)
    {
        const double relGainNormalizationMinSpeed = 10.0; // MSLCM_LC2013.cpp RELGAIN_NORMALIZATION_MIN_SPEED
        const double changeProbThresholdLeft = 0.2; // ctor: (0.2/mySpeedGainParam), default mySpeedGainParam=1

        {
            // Issue 1 cross-edge fix follow-up (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md par 7): a
            // PARKED vehicle (VehicleRuntime.IsParked) is off the running lane entirely in SUMO --
            // MSLaneChanger's checkChangeOnLane machinery only ever runs for a vehicle actually IN a
            // lane's vehicle list, which a parked vehicle is not (MSVehicleTransfer / the parking
            // removal). It therefore makes NO keep-right / strategic / speed-gain decision at all
            // while parked. Without this guard, a park-and-stay sink parked on the PARKINGAREA's own
            // lane still runs the full keep-right/strategic/speed-gain decision every step -- on the
            // synthetic grid (real cross-lane traffic present, unlike the single-vehicle scenarios
            // 48/67/69/70), the SPEED-GAIN branch further below (comparing ego's -- stationary,
            // speed=0 -- current lane against a moving neighbor lane) can decide a spurious LEFT
            // change off the parkingArea's lane; once ego's LaneId no longer equals the stop's LaneId,
            // StopLineConstraint/ProcessNextStop's `stop.LaneId != v.LaneId` guard stops holding it
            // (see those methods' own header comments), so the "parked" vehicle silently starts
            // driving again and eventually wrongly "arrives" -- exactly undoing the residency fix
            // this session restores. Gated on IsParked (false for every moving vehicle, default
            // false), so byte-identical for every existing scenario: none of the committed
            // parking goldens (48/66/67/68/69/70) has a SECOND vehicle on an adjacent lane to even
            // make the (pre-existing, unguarded) speed-gain comparison non-trivial while parked.
            if (v.IsParked)
            {
                return;
            }

            // C10-i: a vehicle mid continuous-maneuver is committed to that change -- it makes no
            // new keep-right/strategic/speed-gain decision until the maneuver completes (SUMO holds
            // the change to completion). Inert when lanechange.duration 0 (LcTargetHandle stays -1).
            if (v.LcTargetHandle >= 0)
            {
                return;
            }

            // C4-vii-c: no lane-change decision while inside a junction interior. SUMO's
            // MSLaneChanger runs only on normal edges, never on an internal (`:`-prefixed) lane, so a
            // vehicle mid-junction makes no keep-right / strategic / speed-gain decision. This is also
            // load-bearing as a CRASH GUARD: TryStrategicLaneChange (and ApplyKeepRightDecision) call
            // ComputeBestLanes(route, lane.EdgeId), which throws "edge not part of route" for an
            // internal lane (its edge is never on the route). ApplyKeepRightDecision already guards
            // this internally (commit eac0a5b); hoisting the check here additionally covers the
            // strategic-LC path, which the convergence fix (TryReResolveFromActualLane) can now route
            // a vehicle onto -- a vehicle that used to CLAMP at a lane end (never entering the
            // junction) now proceeds onto the internal lane and, if it lands there at step end, would
            // otherwise reach TryStrategicLaneChange -> ComputeBestLanes on that internal edge. Inert
            // for every committed scenario: single-lane junction interiors have no left/right neighbor
            // (keep-right/speed-gain already no-op) and no committed multi-edge route stops a vehicle
            // mid-internal-lane to reach the strategic path, so the guard is never the deciding return.
            if (_network!.LanesByHandle[v.LaneHandle].EdgeId is { Length: > 0 } egoEdge && egoEdge[0] == ':')
            {
                return;
            }

            // Rung ER4 (give-way execution): a blue-light EV drives straight and relies on OTHERS
            // clearing the way -- it makes no ordinary overtaking/keep-right lane change of its own
            // (which would otherwise let it speed-gain past the very traffic that is trying to
            // vacate for it, defeating the rescue lane). Behavioral / opt-in: HasBluelight is set by
            // no committed parity scenario, so this is inert everywhere give-way is absent.
            if (v.VType.HasBluelight)
            {
                return;
            }

            // Rung ER4 (give-way execution, multi-lane): a vehicle actively clearing the way for an
            // approaching blue-light EV (GiveWaySide != 0, set this step by DetectGiveWay) takes
            // PRIORITY over -- and SUPPRESSES -- the ordinary keep-right/strategic/speed-gain
            // decisions, mirroring SUMO's MSDevice_Bluelight disabling an influenced vehicle's
            // strategic lane-changing (MSDevice_Bluelight.cpp:256, LCA_STRATEGIC_PARAM=-1). When the
            // EV is in ego's own lane and a safe adjacent lane exists, ego changes into it to VACATE
            // its lane for the EV; otherwise it holds this lane (ER5's within-lane drift, computed in
            // the plan phase, pulls it to the edge). Inert (returns immediately) for every vehicle
            // with GiveWaySide == 0, i.e. every vehicle in every scenario with no bluelight EV.
            if (v.GiveWaySide != 0)
            {
                TryGiveWayLaneChange(v, _network.LanesByHandle[v.LaneHandle], postMoveNeighbors, dt);
                return;
            }

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
                return;
            }

            // Left neighbor = same edge, index+1 (no neighbor on the leftmost lane) -- this
            // guard is the INERT case CLAUDE.md's briefing calls for: single-lane rungs, and any
            // vehicle already on the leftmost lane (e.g. this same follower once it has already
            // changed left), leave SpeedGainProbability untouched and never fire a change. D4:
            // `lane.LeftNeighbor` is precomputed at ingest (NetworkParser) -- O(1) array read
            // instead of a per-step `edge.Lanes.FirstOrDefault(...)` LINQ scan/closure.
            if (lane.LeftNeighbor < 0)
            {
                return;
            }

            var leftLane = _network.LanesByHandle[lane.LeftNeighbor];

            var vMax = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);
            var neighVMax = KraussModel.LaneVehicleMaxSpeed(leftLane.Speed, v.SpeedFactor, v.VType);

            // MSLCM_LC2013.cpp:1109-1136's best-lanes continuation distance, simplified per the
            // A2 briefing's scope note to this single-edge dead-end scenario: laneLength minus
            // the vehicle's (post-move) position on it. Large enough here (~2000/~1865) that it
            // never binds the no-leader stop-speed fallback or the `>20` gate below -- a full
            // multi-edge best-lanes port is out of scope until a scenario actually needs it.
            // P2G-3 (docs/HIGH-DENSITY-P2G3-DESIGN.md §5.2), gated on CoordinatedLaneChange: use the
            // best-lanes CONTINUATION distance (so a clear continuing lane reads vMax, not the short
            // remaining-length stop-speed floor). Default OFF keeps the single-lane distance -> the
            // whole speed-gain path is byte-identical.
            double currentDist, neighDist;
            if (CoordinatedLaneChange && TryBestLanesForEdge(v, lane.EdgeId, out var sgBestLanes))
            {
                currentDist = ContinuationLength(sgBestLanes, lane.Index, lane.Length) - v.Kinematics.Pos;
                neighDist = ContinuationLength(sgBestLanes, leftLane.Index, leftLane.Length) - v.Kinematics.Pos;
            }
            else
            {
                // Default (parity) distance, AND the coordinated fallback when ego is briefly on an edge
                // not in its nominal route (organic/rerouted nets do this) -- ComputeBestLanes would throw
                // there, so TryBestLanesForEdge returns false and we use the single-lane distance.
                currentDist = lane.Length - v.Kinematics.Pos;
                neighDist = leftLane.Length - v.Kinematics.Pos;
            }

            var leader = postMoveNeighbors.GetLeader(v);
            var thisLaneVSafe = Math.Min(vMax, AnticipateFollowSpeed(v, leader, currentDist, dt));

            // P2G-3 (§5.2 part 1), gated: reduce thisLaneVSafe for a leader on ego's CONTINUATION across
            // the junction (which the same-edge GetLeader misses), creating SUMO's relativeGain asymmetry
            // (slow leader ahead on this lane, clear neighbour). Uses the scan's accumulated cross-
            // junction gap directly with MaximumSafeFollowSpeed (NOT AnticipateFollowSpeed's same-lane
            // gap). Inert by default.
            if (CoordinatedLaneChange && leader is null
                && TryFindContinuationLeader(v, postMoveNeighbors, dt, out var contLeader, out var contGap))
            {
                var contVSafe = KraussModel.MaximumSafeFollowSpeed(
                    contGap, v.Kinematics.Speed, contLeader.Kinematics.Speed, contLeader.VType.Decel, v.VType, dt, onInsertion: true);
                thisLaneVSafe = Math.Min(thisLaneVSafe, contVSafe);
            }

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
                if (IsTargetLaneSafe(v, neighLead, neighFollow, dt) && !TargetLaneBlockedByObstacle(v, leftLane, time, dt) && !IsTargetLaneOverlapped(v, leftLane.Handle, postMoveNeighbors, dt))
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
                CommitLaneChange(v, targetLaneHandle, targetLaneId);
            }
        }
    }

    // MSLCM_LC2013's keep-right sub-block ONLY (see CLAUDE.md briefing's scope note): strategic/
    // cooperative LC blocks all pass through as non-binding and only the keep-right accumulator
    // drives the decision. NOT built (scoped out): strategic/cooperative blocks, general
    // best-lanes (neighDist here is simply the right lane's own length, not a computed
    // continuation distance), continuous lateral (SL2015), lanechange.duration>0, and multi-edge
    // route lane continuity. (P2-G: the target-lane safety/blocker veto against neighbors IS now
    // ported at the fire site below -- see that comment.) `checkOverTakeRight`
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

        // No lane change while inside a junction interior: SUMO's MSLaneChanger runs only on normal
        // edges, never on internal (`:`-prefixed) lanes. This guard is also load-bearing for the
        // C4-vii-b stayOnBest veto below -- KeepRightStrategicStay -> ComputeBestLanes(route,
        // lane.EdgeId) requires lane.EdgeId to be ON the route, which an internal lane never is
        // (it would throw "edge not part of route"). Multi-lane junctions have 2+-lane internal
        // edges (e.g. a straight-through :C_13_0/:C_13_1), so a vehicle traversing the right one has
        // a RightNeighbor and would otherwise reach that call; every committed scenario is a single-
        // lane junction (internal lanes have no right neighbor) so this is inert there, but a general
        // -L2 net hits it every junction crossing.
        if (lane.EdgeId.Length > 0 && lane.EdgeId[0] == ':')
        {
            return;
        }

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

        // C4-vii-b: strategic STAY-on-best (MSLCM_LC2013.cpp:1421-1440, "VARIANT_21 stayOnBest").
        // When ego is on a route-continuing lane whose RIGHT neighbour is a turn/exit lane that
        // leaves the route within TURN_LANE_DIST, SUMO's `_wantsChange` sets LCA_STAY|LCA_STRATEGIC
        // and RETURNS *before* the keep-right accumulator runs -- so myKeepRightProbability is never
        // decremented while ego is held on that required lane. This is the full early-return-before-
        // accumulation semantics the former commit-only shortcut omitted: that earlier form let the
        // accumulator run on every lane and only vetoed the COMMIT, so a vehicle held on a required
        // lane over-accumulated keep-right and then fired a SPURIOUS change on a LATER lane where the
        // veto lifted -- e.g. a left-turner reaching its multi-lane arrival edge would immediately
        // keep-right off its arrival lane and strand itself (scenarios/44 / 45 bug B). Confirmed
        // against SUMO via TraCI: on the scenarios/44 net a left-turner's keepRightProbability stays
        // exactly 0 on the approach turn-lane and only starts accumulating once it reaches the arrival
        // edge, never crossing the fire threshold over the short remaining distance.
        //
        // The DISTANCE gate is load-bearing (MSLCM_LC2013.cpp:1428 `neighDist < TURN_LANE_DIST`): a
        // right lane that leaves the route but only FAR ahead (>= 200 m of usable length) is NOT a
        // must-avoid turn lane -- SUMO DOES keep-right onto it (there is room to change back before
        // the split), so this stay must NOT fire there. `neighDist` is the right lane's best-lanes
        // continuation length (ComputeBestLanes' Length), matching SUMO's `neigh.length`. The stay
        // decision is a pure function of (current lane, remaining route) -> memoized per lane (see
        // VehicleRuntime) so the allocating ComputeBestLanes pass stays off the per-step hot path.
        // Byte-identical for single-edge routes (fast path: no stay) and for the highway benchmark
        // (right lanes all continue the route, so !AllowsContinuation is never true) -- verified: the
        // Sim.Bench determinism hash is unchanged. SIMPLIFICATION (documented): only VARIANT_21 of
        // SUMO's several strategic-STAY rules is ported; the change-back-in-time rules (:1398-1420)
        // and the `getLinkCont().size()!=0` "leads somewhere" guard are not modelled -- the committed
        // anchors' right lanes all lead onward and sit inside/outside the 200 m band unambiguously.
        if (v.KeepRightStayCacheLane != v.LaneHandle)
        {
            v.KeepRightStayCacheLane = v.LaneHandle;
            v.KeepRightStaySuppress = KeepRightStrategicStay(v, lane, rightLane.Index);
        }
        if (v.KeepRightStaySuppress)
        {
            return;
        }

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
            // MSLCM_LC2013.cpp:1789/1061-1064: fires -> lane change REQUESTED, then subject to the
            // same target-lane safety block every change goes through.
            //
            // P2-G (docs/HIGH-DENSITY-P2G-DESIGN.md): target-lane LEADER safety veto. SUMO's
            // MSLaneChanger::checkChange (MSLaneChanger.cpp:744-935) blocks a change on the target
            // lane's LEADER secure gap (:843-870) and FOLLOWER secure gap (:798-837) -- a change
            // executes only if (state & LCA_BLOCKED) == 0 (:430) -- so a keep-right (RIGHT change) is
            // subject to the same secure-gap block as a speed-gain (LEFT change). Without ANY veto the
            // engine fired a keep-right that lands unsafely behind the right lane's leader and forces
            // a brake -- the ~7 m/2.6 m/s dense multi-lane divergence (empirically: a vehicle SUMO
            // keeps on the left lane that the pre-fix engine wrongly keep-rights, cutting in and
            // braking 13.89->11.52 m/s). Adding the LEADER veto collapses that divergence (a straight
            // 2-lane control: max pos error 82.28 m -> 2.37 m; first divergence t=8 s -> t=72 s).
            //
            // LANE-CHANGE-OVERLAP (docs/LANE-CHANGE-OVERLAP-DESIGN.md §3 Stage 1): the FOLLOWER half of
            // checkChange is now applied here too (previously keep-right passed a null follower -- a
            // documented reduction in HIGH-DENSITY-P2G-DESIGN.md §4.1, made because a follower veto
            // WITHOUT cooperative LC once regressed the saturated grid 0->30 stuck). That gridlock trap
            // is obsolete: the serve-path junction traffic-light fixes (HIGH-DENSITY-P2G2-...-DESIGN.md
            // §0) removed the root gridlock, so re-measurement this session shows the full leader+
            // follower+overlap block keeps willpass-saturation at 0 stuck while removing the keep-right
            // follower-side cut-ins. The overlap block (IsTargetLaneOverlapped) additionally catches an
            // occupant AT ego's exact position -- the dominant overlap source, invisible to both the
            // leader and follower lookups (design §2). `neighLead` is already fetched above; fetch the
            // real follower and apply the full block, mirroring every other change path.
            var neighFollowKr = neighbors.GetNeighborFollower(v, rightLane.Handle);
            if (IsTargetLaneSafe(v, neighLead, neighFollowKr, dt) && !IsTargetLaneOverlapped(v, rightLane.Handle, neighbors, dt))
            {
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
                v.KeepRightProbability = 0.0; // resetState() on a COMMITTED change (:1063/1080).
                return;
            }

            // BLOCKED: like the LEFT/speed-gain veto, a vetoed change does NOT reset the accumulator
            // (SUMO resets only on a committed change) -- fall through to store the decremented
            // keepRightProbability, keep the lane, and retry next step, so the vehicle keeps right the
            // instant the leader gap opens. Byte-identical for every committed scenario: each one
            // reaching this fire has an empty right lane, so IsTargetLaneSafe is true and the swap
            // still happens (single-lane scenarios never reach here at all -- RightNeighbor < 0).
        }

        v.KeepRightProbability = keepRightProbability;
    }

    // C4-vii-b: SUMO's stayOnBest keep-right suppressor (MSLCM_LC2013.cpp:1421-1440, VARIANT_21).
    // Returns true when ego must NOT accumulate the keep-right incentive because its RIGHT neighbour
    // (`rightLaneIndex` on `fromLane`'s edge) is a must-avoid turn/exit lane: ego is on a route-
    // continuing lane (its own best-lanes offset is 0), the right lane does NOT continue the route,
    // AND the right lane leaves the route within TURN_LANE_DIST (its best-lanes continuation length
    // is short enough that there is no room to keep-right and change back before the split). A
    // single-edge route never stays (fast path: no ComputeBestLanes call, so 06/07/12-overtake's
    // keep-right stays byte-identical). See ApplyKeepRightDecision's call site for the full rationale
    // and TraCI cross-check.
    private const double KeepRightTurnLaneDist = 200.0; // MSLCM_LC2013.cpp:71 TURN_LANE_DIST

    // Perf (PERF-ROADMAP.md Layer 0b): memoized NetworkModel.ComputeBestLanes accessor -- see
    // _bestLanesCache. State-passing GetOrAdd so a cache HIT allocates nothing and the value factory
    // (a pure function of the immutable route + network) captures no closure. Shared across every
    // vehicle and step for the scenario's lifetime; cleared per LoadScenario. Byte-identical to
    // calling ComputeBestLanes directly (same immutable inputs -> same result).
    private IReadOnlyList<LaneContinuation> BestLanesCached(string routeId, IReadOnlyList<string> routeEdges, string edgeId)
        => _bestLanesCache.GetOrAdd(
            (routeId, edgeId),
            static (key, state) => state.Network.ComputeBestLanes(state.RouteEdges, key.EdgeId),
            (Network: _network!, RouteEdges: routeEdges));

    // P2G-3: robust best-lanes accessor for the coordinated LC path. Returns false (instead of letting
    // ComputeBestLanes THROW "edge not part of route") when ego's current edge is not in its nominal
    // route -- which happens transiently on organic / rerouted nets where a vehicle traverses an edge its
    // route does not list. The speed-gain caller then falls back to the single-lane distance. The
    // membership scan is O(routeLen) (~10-20 edges) and only on the coordinated multi-lane speed-gain path.
    private bool TryBestLanesForEdge(VehicleRuntime v, string edgeId, out IReadOnlyList<LaneContinuation> bestLanes)
    {
        var routeId = EffectiveRouteId(v);
        var edges = _routesById[routeId].Edges;
        for (var i = 0; i < edges.Count; i++)
        {
            if (edges[i] == edgeId)
            {
                bestLanes = BestLanesCached(routeId, edges, edgeId);
                return true;
            }
        }

        bestLanes = System.Array.Empty<LaneContinuation>();
        return false;
    }

    // P2G-3 (docs/HIGH-DENSITY-P2G3-DESIGN.md §5.2): best-lanes CONTINUATION length for a lane index
    // (SUMO's LaneQ.length) -- the on-route distance drivable without a lane change, accumulated across
    // downstream edges. Falls back to the lane's own length if no continuation entry matches.
    private static double ContinuationLength(IReadOnlyList<LaneContinuation> bestLanes, int laneIndex, double fallbackLaneLength)
    {
        for (var i = 0; i < bestLanes.Count; i++)
        {
            if (bestLanes[i].LaneIndex == laneIndex)
            {
                return bestLanes[i].Length;
            }
        }

        return fallbackLaneLength;
    }

    // P2G-3 (§5.2 part 1): nearest leader on EGO's best-lanes CONTINUATION across the junction (the
    // leader that crossed onto the junction internal / next edge, which the same-edge GetLeader misses),
    // with the correctly-accumulated cross-junction gap. Mirrors CrossJunctionLeaderConstraint's
    // downstream scan (a read-only copy so the byte-identical car-following path is untouched). Only ever
    // called under CoordinatedLaneChange (see the speed-gain call site).
    private bool TryFindContinuationLeader(VehicleRuntime ego, LaneNeighborQuery neighbors, double dt, out VehicleRuntime leader, out double gap)
    {
        leader = null!;
        gap = double.PositiveInfinity;

        var downstreamCount = ego.LaneSeqLen - ego.LaneSeqIndex - 1;
        if (downstreamCount <= 0)
        {
            return false;
        }

        var downstreamStart = ego.LaneSeqStart + ego.LaneSeqIndex + 1;
#if NET8_0_OR_GREATER
        ReadOnlySpan<int> downstream = CollectionsMarshal.AsSpan(_laneSeqPool).Slice(downstreamStart, downstreamCount);
#else
        Span<int> downstreamBuf = downstreamCount <= 64 ? stackalloc int[downstreamCount] : new int[downstreamCount];
        for (int i = 0; i < downstreamCount; i++)
        {
            downstreamBuf[i] = _laneSeqPool[downstreamStart + i];
        }
        ReadOnlySpan<int> downstream = downstreamBuf;
#endif

        return TryFindCrossJunctionLeader(
            ego.Kinematics.Speed, ego.VType, ego, ego.LaneHandle, ego.Kinematics.Pos,
            downstream, new NeighborRearmost(neighbors, ego), dt, out leader, out gap);
    }

    private bool KeepRightStrategicStay(VehicleRuntime v, Lane fromLane, int rightLaneIndex)
    {
        var routeId = EffectiveRouteId(v);   // see _effectiveRouteIdByEntity's header comment
        var route = _routesById[routeId];
        if (route.Edges.Count <= 1)
        {
            return false;
        }

        var bestLanes = BestLanesCached(routeId, route.Edges, fromLane.EdgeId);
        var currContinues = false;
        var rightLeavesRoute = false;
        var rightSoon = false;
        foreach (var continuation in bestLanes)
        {
            if (continuation.LaneIndex == fromLane.Index)
            {
                // SUMO's `bestLaneOffset == 0`: ego's own lane is on the best (route) path.
                currContinues = continuation.BestLaneOffset == 0;
            }
            else if (continuation.LaneIndex == rightLaneIndex)
            {
                rightLeavesRoute = !continuation.AllowsContinuation;
                rightSoon = continuation.Length < KeepRightTurnLaneDist;
            }
        }

        return currContinues && rightLeavesRoute && rightSoon;
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
    // C10-i: round(lanechange.duration / stepLength) -- the number of steps a continuous lane change
    // spans. <= 1 means "instant" (duration 0, every pre-C10 scenario), the discrete-snap default.
    private int LaneChangeSteps() =>
        (int)Math.Round(_config!.LaneChangeDuration / _config.StepLength, MidpointRounding.AwayFromZero);

    // C10-i: commit a decided lane change -- an INSTANT lane-index snap (lanechange.duration 0, the
    // discrete default) or a continuous MANEUVER (duration > 0) that holds the source lane label and
    // slides over LaneChangeSteps() steps (Engine.AdvanceLaneChanges). Both go through the command
    // buffer, so the decision phase (DecideSpeedGainChanges / its TryStrategicLaneChange) still only
    // records; nothing mutates mid-scan.
    // Realism knob (NOT a SUMO default; 0 = off = byte-identical to every golden, so parity is untouched).
    // When > 0, a vehicle may not INITIATE a discrete lane change while its speed is below this value --
    // i.e. it does not snap sideways a full lane width while essentially stopped in a queue; it sorts into
    // its lane while moving instead. This mirrors the effect of SUMO's sublane `maxSpeedLatStanding=0`
    // (no lateral movement at standstill) without porting the whole sublane model. Set by the live-city
    // demo; every parity scenario leaves it 0.
    public double LaneChangeMinSpeed { get; set; }

    private void CommitLaneChange(VehicleRuntime v, int targetHandle, string targetId)
    {
        // Below the realism threshold (default 0 -> never triggers), suppress the change: a barely-moving
        // car keeps its lane this step and re-evaluates once it is actually moving. Deadlock-safe at a low
        // threshold because any forward creep (queue advancing, light turning green) clears the gate.
        if (LaneChangeMinSpeed > 0.0 && v.Kinematics.Speed < LaneChangeMinSpeed)
        {
            return;
        }

        var steps = LaneChangeSteps();
        if (steps <= 1)
        {
            _commandBuffer.ChangeLane(v, targetHandle, targetId);
        }
        else
        {
            _commandBuffer.StartLaneChangeManeuver(v, targetHandle, targetId, steps);
        }
    }

    // C10-i: advance every in-progress continuous lane-change maneuver by one step (runs at the TOP
    // of the loop, before EmitTrajectory, so the emitted lane reflects the maneuver's progress this
    // frame). The emitted lane stays the SOURCE until the vehicle center crosses the lane midpoint --
    // halfway through the maneuver (MSVehicle emits the lane whose half its center is in) -- then
    // becomes the target; the maneuver completes after LcStepsTotal steps. No-op (LcTargetHandle < 0)
    // for every vehicle not mid-change, so a duration-0 scenario never enters here.
    private void AdvanceLaneChanges()
    {
        foreach (var v in ActiveVehicles())
        {
            if (v.LcTargetHandle < 0)
            {
                continue;
            }

            // Realism (LaneChangeMinSpeed > 0, demo-only; 0 = off = byte-identical): HOLD an in-progress
            // maneuver while the car is essentially stopped, so the lateral centre-to-centre flip (and its
            // on-screen sideways step) never happens at a standstill -- it resumes once the car moves.
            if (LaneChangeMinSpeed > 0.0 && v.Kinematics.Speed < LaneChangeMinSpeed)
            {
                continue;
            }

            v.LcStepsElapsed++;
            // Flip the emitted lane once past the midpoint: with a constant lateral speed
            // laneWidth/duration, the center crosses the mid-line (laneWidth/2 lateral travel) after
            // duration/2 steps, i.e. once 2*elapsed > total. (SIMPLIFICATION, documented: at an even
            // `total` the exact-midpoint step resolves to the source lane -- no committed scenario
            // uses an even duration; scenario 43 is duration 3.)
            if (v.LaneHandle != v.LcTargetHandle && (2 * v.LcStepsElapsed) > v.LcStepsTotal)
            {
                v.LaneHandle = v.LcTargetHandle;
                v.LaneId = v.LcTargetId;
            }

            if (v.LcStepsElapsed >= v.LcStepsTotal)
            {
                v.LcTargetHandle = -1;
                v.LcTargetId = string.Empty;
                v.LcStepsElapsed = 0;
                v.LcStepsTotal = 0;
            }
        }
    }

    private bool TryStrategicLaneChange(VehicleRuntime v, Lane lane, LaneNeighborQuery neighbors, double time, double dt, double actionStepLengthSecs)
    {
        // C10-i: a vehicle already mid-maneuver is committed to it -- do not start a second change
        // (SUMO holds the maneuver to completion). Inert when duration 0 (LcTargetHandle stays -1).
        if (v.LcTargetHandle >= 0)
        {
            return false;
        }

        if (v.LaneSeqIndex >= v.LaneSeqLen)
        {
            return false;
        }

        // Issue 1 (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §7, park-and-stay residency): a vehicle's
        // FRONT pending (unreached) stop, when it sits on THIS edge with a DIFFERENT lane index
        // than v's current lane, must steer the strategic lane-change toward the STOP's lane.
        // Ported from MSVehicle::updateBestLanes, which folds a same-edge stop into the LaneQ by
        // truncating every OTHER lane's `length` to the stop position while the stop's own lane
        // keeps the full edge length (MSVehicle.cpp:5920-5933) -- which is exactly what steers
        // bestLaneOffset toward it -- combined with MSLCM_LC2013::wantsChangeStrategic's
        // `driveToNextStop = nextStopDist()` term that makes usableDist bind on distance-to-stop
        // rather than distance-to-edge-end (MSLCM_LC2013.cpp:1161-1182, 1288). Ported directly at
        // this call site (not inside ComputeBestLanes/BackwardPassEdge, which several unrelated
        // callers -- ApplyKeepRightDecision, insertion -- share) so the change stays scoped to
        // exactly this one case: a stop lane that differs from the route-continuation pool target.
        //
        // GATING (byte-identical elsewhere): stopLaneOverride stays -1 unless GetStops(v) has an
        // unreached front stop whose lane is on THIS edge with a lane index != v's current lane
        // index. Every existing stop scenario (03-approach-and-stop, 13-stopped-leader,
        // 44-summary-output, 48-parking-depart) already has the vehicle on the stop's own lane
        // (stopLane.Index == lane.Index there), so the override is inert for every one of them --
        // execution falls straight through to the pre-existing pool/best-lane path, unchanged.
        var stopLaneOverride = -1;
        var stopDistOverride = 0.0;
        var stops = GetStops(v);
        if (stops is { Count: > 0 })
        {
            var frontStop = stops.Peek();
            if (!frontStop.Reached
                && _network!.LanesById.TryGetValue(frontStop.LaneId, out var stopLane)
                && stopLane.EdgeId == lane.EdgeId
                && stopLane.Index != lane.Index)
            {
                stopLaneOverride = stopLane.Index;
                // MSLCM_LC2013.cpp:1167 driveToNextStop = myVehicle.nextStopDist() -- the
                // remaining lane-relative distance to the stop's braking position.
                stopDistOverride = frontStop.EndPos - v.Kinematics.Pos;
            }
        }

        int bestLaneOffset;
        double usableDist;

        if (stopLaneOverride >= 0)
        {
            bestLaneOffset = stopLaneOverride - lane.Index;
            usableDist = stopDistOverride;
        }
        else
        {
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

            var routeId = EffectiveRouteId(v);   // see _effectiveRouteIdByEntity's header comment
            var route = _routesById[routeId];

            // Issue 1 cross-edge fix (docs/SUMOSHARP-SERVE-PATH-DROP-IN.md §7): a vehicle with a
            // qualifying park-and-stay stop on its route's FINAL edge (ParkStopFinalEdgeOverride) must
            // read a STOP-AWARE bestLanes here -- the SAME override the pool itself was built with at
            // insertion/reroute (see TryInsertOnLane/RerouteActive) -- so `curr.BestLaneOffset` agrees
            // with the pool's own steering (per-vehicle, so it deliberately bypasses the shared
            // ROUTE-keyed `_bestLanesCache`; see that cache's own caching-hazard note). null for every
            // other vehicle, taking the exact prior cached path, byte-identical.
            var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, route.Edges);
            var bestLanes = stopOverride is not null
                ? _network!.ComputeBestLanes(route.Edges, lane.EdgeId, stopOverride)
                : BestLanesCached(routeId, route.Edges, lane.EdgeId);

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

            bestLaneOffset = curr.BestLaneOffset;

            // usableDist = MAX2(currentDist - posOnLane - best.occupation*JAM_FACTOR,
            // driveToNextStop): best.occupation is always 0 (empty-road scope, see this method's
            // header comment) and, with stopLaneOverride < 0 (no same-edge unreached stop to bind
            // driveToNextStop against), the first MAX2 argument is the only one available.
            usableDist = curr.Length - v.Kinematics.Pos;
        }

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

        // MSLCM_LC2013.h:189 currentDistDisallows.
        if (usableDist / Math.Abs(bestLaneOffset) >= laDist)
        {
            return false;
        }

        var neighborHandle = right ? lane.RightNeighbor : lane.LeftNeighbor;
        if (neighborHandle < 0)
        {
            // No lane to change into on the required side -- defensive only, not reachable by any
            // committed scenario (ComputeBestLanes never points a route offset off the edge's own
            // lane range) nor by a well-formed stopLaneOverride (a parkingArea's lane is always a
            // real lane on this same edge, so the direct neighbor toward it always exists on a
            // 2-lane edge; a >=3-lane edge crosses one lane per call/step, same as SUMO).
            return false;
        }

        var neighborLane = _network!.LanesByHandle[neighborHandle];

        // Safety veto, mirroring A2-iii's IsTargetLaneSafe / B5-ii's obstacle veto -- on the
        // clear road this scenario exercises, both are trivially non-binding (no neighbor
        // vehicle, no obstacle), matching the briefing's "on a clear road the change is always
        // safe". A scenario WITH target-lane traffic during a strategic change is future work
        // (LCA_URGENT's real blocker-cooperation machinery, `.cpp:1467-1517`, is not ported).
        var neighLead = neighbors.GetNeighborLeader(v, neighborLane.Handle);
        var neighFollow = neighbors.GetNeighborFollower(v, neighborLane.Handle);
        if (!IsTargetLaneSafe(v, neighLead, neighFollow, dt) || TargetLaneBlockedByObstacle(v, neighborLane, time, dt) || IsTargetLaneOverlapped(v, neighborLane.Handle, neighbors, dt))
        {
            return false;
        }

        CommitLaneChange(v, neighborLane.Handle, neighborLane.Id);
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

    // LANE-CHANGE-OVERLAP (docs/LANE-CHANGE-OVERLAP-DESIGN.md §3 Stage 1): SUMO's checkChange
    // LCA_OVERLAPPING block (MSLaneChanger.cpp:767/780 -- a change is blocked when neighFollow.second < 0
    // OR neighLead.second < 0, i.e. a target-lane neighbour's BODY overlaps ego's slot). IsTargetLaneSafe
    // above ports the secure-gap test but NOT this overlap block, and -- critically -- the lane-change
    // neighbour lookup (GetNeighborLeader skips pos<=egoPos, GetNeighborFollower skips pos>=egoPos)
    // classifies an occupant at ego's EXACT position as neither leader nor follower, so the secure-gap
    // veto never sees it. In a saturated grid, vehicles on adjacent lanes stop at the same stop-line arc
    // position, so this exact-tie blind spot is the DOMINANT overlap source (design §2). This scans the
    // target lane's pos-sorted bucket directly and blocks if ANY occupant's body [pos-len, pos] overlaps
    // ego's projected slot [egoPos-len, egoPos] -- the faithful negative-gap block, robust to the tie the
    // leader/follower lookup misses. Reads only the frozen snapshot + immutable network; ego's fields
    // otherwise. Inert (returns false) when the target lane carries no body-overlapping occupant -- every
    // committed golden at a change, so byte-identical.
    private bool IsTargetLaneOverlapped(VehicleRuntime ego, int targetLaneHandle, LaneNeighborQuery neighbors, double dt)
    {
        var occupants = neighbors.OnLane(targetLaneHandle);
        var egoFront = ego.Kinematics.Pos;
        var egoBack = egoFront - ego.VType.Length;
        for (var i = 0; i < occupants.Count; i++)
        {
            var o = occupants[i];
            if (ReferenceEquals(o, ego))
            {
                continue;
            }

            var oFront = o.Kinematics.Pos;
            var oBack = oFront - o.VType.Length;
            // Bodies overlap iff each front is ahead of the other's back (strict: a bumper-to-bumper
            // touch, gap == 0, is not an overlap -- matches SUMO's strict `neigh.second < 0`).
            if (oBack < egoFront && egoBack < oFront)
            {
                return true;
            }
        }

        return false;
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

    // P0-D (--summary-output `meanSpeedRelative`, MSNet.cpp:607-647): MSEdge::getSpeedLimit()
    // analog -- the max speed limit over the CURRENT edge's own lanes (an edge's lanes may in
    // principle post different limits, e.g. a faster HOV lane; SUMO takes the max). Every phase-1
    // scenario's lanes all share one <lane speed=.../> per edge, so this is numerically identical
    // to "the current lane's own speed" for every committed golden -- computed from the edge, not
    // the lane, so a future net with per-lane speed overrides still matches SUMO's own definition.
    private double EdgeSpeedLimitOf(Lane lane)
    {
        var edge = _network!.EdgesById[lane.EdgeId];
        var limit = 0.0;
        foreach (var edgeLane in edge.Lanes)
        {
            if (edgeLane.Speed > limit)
            {
                limit = edgeLane.Speed;
            }
        }

        return limit;
    }

    // P0-D (--summary-output `stopped`, MSVehicleControl::getStoppedVehiclesCount() analog): true
    // iff this vehicle's own front stop (StopRuntime, D3 side table) is currently `Reached` --
    // i.e. it is being held at a <stop> this step (ProcessNextStop's `stop.Reached` branch), not
    // merely scheduled for later. No stops at all (GetStops returns null) or a front stop not yet
    // reached both read false, matching "not currently stopped" exactly like MSVehicle's own
    // `isStopped()` (myStops.front().reached).
    private bool IsStoppedAtStop(VehicleRuntime v)
    {
        var stops = GetStops(v);
        return stops is { Count: > 0 } && stops.Peek().Reached;
    }

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C, §3): SUMO's MSVehicleTransfer TeleportMinSpeed
    // (1 m/s) -- the virtual-proceed hop speed when a teleporting vehicle's jumped-to edge is
    // ALSO jammed (MSVehicleTransfer.cpp:191/203: proceed to the next edge after
    // edgeLength / TeleportMinSpeed seconds).
    private const double TeleportMinSpeed = 1.0;

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1A, §2 item 3): the jam-teleport detection phase, ported
    // from MSLane::executeMovements' firstNotStopped / time-to-teleport check (MSLane.cpp:2239-2260).
    // Runs serially after ExecuteMoves (WaitingTime already settled + flushed); gated by the caller
    // on TimeToTeleport>0. Picks each lane's frontmost NON-stopped vehicle and, if it has waited
    // strictly longer than time-to-teleport, teleports it.
    private void CheckJamTeleports(double time, double dt)
    {
        var ttt = _config!.TimeToTeleport;

        // firstNotStopped per lane: the frontmost (max Pos) vehicle NOT held at a scheduled <stop>
        // (MSLane.cpp:2239-2246). A parked blocker (IsStoppedAtStop) is skipped -- it never
        // accumulates WaitingTime anyway (the !isStopped guard in ExecuteMoveVehicle), and it must
        // not be the lane's teleport candidate. Deterministic: ActiveVehicles() walks ascending
        // EntityIndex, and the strict `>` keeps the earlier (lower-index) vehicle on a Pos tie.
        _jamFrontmost.Clear();
        foreach (var v in ActiveVehicles())
        {
            if (IsStoppedAtStop(v))
            {
                continue;
            }

            if (!_jamFrontmost.TryGetValue(v.LaneHandle, out var cur) || v.Kinematics.Pos > cur.Kinematics.Pos)
            {
                _jamFrontmost[v.LaneHandle] = v;
            }
        }

        // Lanes whose frontmost-stuck vehicle has waited STRICTLY longer than ttt (MSLane.cpp:2260
        // `r1 = ttt>0 && getWaitingTime() > ttt`). With dt=1, ttt=120 => fires at 121s of waiting.
        _jamCandidates.Clear();
        foreach (var kv in _jamFrontmost)
        {
            if (kv.Value.WaitingTime > ttt)
            {
                // X1 (docs/HIGH-DENSITY-X1-DESIGN.md): the teleport gate. A jam blocker on a VISIBLE
                // (on-camera) edge is NOT teleported -- strict no-cheating in the high-realism zone; it
                // is simply held (stays jammed, keeps accumulating WaitingTime). Off-camera / no-mask
                // edges teleport as before. Inert when _activeMask is null (every gate permissive).
                if (_activeMask is not null
                    && !_activeMask.MayTeleport(_network!.LanesByHandle[kv.Value.LaneHandle].EdgeId))
                {
                    continue;
                }

                _jamCandidates.Add(kv.Value);
            }
        }

        if (_jamCandidates.Count == 0)
        {
            return;
        }

        // Deterministic teleport order regardless of Dictionary enumeration: ascending EntityIndex,
        // mirroring MSVehicleTransfer's numerical-id sort.
        _jamCandidates.Sort(static (a, b) => a.EntityIndex.CompareTo(b.EntityIndex));
        foreach (var v in _jamCandidates)
        {
            TeleportVehicle(v);
        }

        // Apply removals recorded above (remove-variant / past-last-edge). The lift-into-transfer
        // path mutates InTransfer directly (serial phase, like InsertDepartingVehicles), so it
        // needs no flush.
        _commandBuffer.Flush();
    }

    // X1 (docs/HIGH-DENSITY-X1-DESIGN.md): aggressive OFF-CAMERA de-jam despawn. For each lane's
    // frontmost non-stopped blocker that has waited longer than DejamDespawnTime AND sits on an
    // off-camera (MayPop) edge, DESPAWN it (remove entirely, _commandBuffer.Destroy -- the same removal
    // the teleport remove-variant uses) -- a more eager, off-camera-only sibling of the jam teleport, so
    // hidden regions never build standing jams. A VISIBLE-edge blocker is never despawned (strict
    // no-cheating; it is held). Reuses the frontmost-non-stopped-per-lane scan (skipping parked blockers
    // via IsStoppedAtStop, ties by lowest EntityIndex), removes in ascending EntityIndex order, capped at
    // DejamDespawnBudgetPerStep. Serial PostSimulation phase (like CheckJamTeleports). Gated OFF by the
    // caller unless DejamDespawnTime > 0, so byte-identical for every committed scenario.
    private void DejamDespawn()
    {
        _jamFrontmost.Clear();
        foreach (var v in ActiveVehicles())
        {
            if (IsStoppedAtStop(v))
            {
                continue;
            }

            if (!_jamFrontmost.TryGetValue(v.LaneHandle, out var cur) || v.Kinematics.Pos > cur.Kinematics.Pos)
            {
                _jamFrontmost[v.LaneHandle] = v;
            }
        }

        _jamCandidates.Clear();
        foreach (var kv in _jamFrontmost)
        {
            var v = kv.Value;
            if (v.WaitingTime <= DejamDespawnTime)
            {
                continue;
            }

            // Off-camera only: a blocker on a VISIBLE (pop-forbidden) edge is held, never despawned.
            // Inert when _activeMask is null (MayPop always true -> every edge treated off-camera).
            if (_activeMask is not null && !_activeMask.MayPop(_network!.LanesByHandle[v.LaneHandle].EdgeId))
            {
                continue;
            }

            _jamCandidates.Add(v);
        }

        if (_jamCandidates.Count == 0)
        {
            return;
        }

        _jamCandidates.Sort(static (a, b) => a.EntityIndex.CompareTo(b.EntityIndex));
        var remaining = DejamDespawnBudgetPerStep;
        foreach (var v in _jamCandidates)
        {
            if (remaining <= 0)
            {
                break;
            }

            _commandBuffer.Destroy(v);
            _dejamDespawnCount++;
            remaining--;
        }

        _commandBuffer.Flush();
    }

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C item 1, §2 item 4): MSVehicleTransfer::add. Count the
    // jam-teleport, then either REMOVE the vehicle (time-to-teleport.remove, or no succEdge(1)) or
    // lift it into the transfer queue having jumped it onto succEdge(1).
    // Issue-2 teleport classification (MSLane.cpp:2261,2272-2274).
    private enum TeleportKind { Jam, Yield, WrongLane }

    // Classify a jam-teleport exactly as SUMO: wrongLane = !appropriate(v); else if ego's NEXT junction
    // link is MINOR (!havePriority) it is a YIELD wait (waiting for a right-of-way foe); else JAM
    // (blocked by a leader on its own lane). wrongLane is not produced yet (a documented simplification:
    // every in-scope scenario -- the committed goldens + the synthetic-junction repro -- reports 0
    // wrongLane, matching SUMO); the WrongLane bucket exists for schema completeness.
    private TeleportKind ClassifyTeleportKind(VehicleRuntime v)
    {
        // Ego's next junction link = the first internal (':') lane at/after LaneSeqIndex on its route
        // sequence -- SUMO's succLinkSec(v, 1, ..., getBestLanesContinuation()). The same forward scan
        // KeepClearConstraint uses.
        for (var i = v.LaneSeqIndex; i < v.LaneSeqLen; i++)
        {
            var seqLaneId = _network!.LanesByHandle[_laneSeqPool[v.LaneSeqStart + i]].Id;
            if (_network.LinkByInternalLane.TryGetValue(seqLaneId, out var jl))
            {
                // MSLink::havePriority(): myState in 'A'..'Z' (uppercase == priority). Lowercase
                // (m/g/s/w/...) == minor -> the vehicle is yielding to a foe, not jammed.
                var state = LinkStateChar(jl.Link);
                return (state >= 'A' && state <= 'Z') ? TeleportKind.Jam : TeleportKind.Yield;
            }
        }

        // No junction link ahead (route end, or a lane that cannot continue): SUMO's link == myLinks.end()
        // makes minorLink false -> jam.
        return TeleportKind.Jam;
    }

    // Issue-2: the CURRENT right-of-way state char of a junction link -- the live TL phase char for a
    // TL-controlled link, else the static netconvert <connection state="..."> char (default 'M' major
    // when the attribute is absent, e.g. pre-existing nets parsed before the state field was added).
    private char LinkStateChar(JunctionLink link)
    {
        var conn = link.Connection;
        if (conn.Tl is { } tl && conn.LinkIndex is { } li)
        {
            return TlLinkStateChar(tl, li, CurrentTime);
        }

        return conn.State is { Length: > 0 } s ? s[0] : 'M';
    }

    private void TeleportVehicle(VehicleRuntime v)
    {
        // Counted once here at the decision, regardless of whether a later virtual-proceed hop
        // eventually removes the vehicle (MSVehicleControl.cpp:561-564). Issue-2: classify the teleport
        // into wrongLane / yield / jam exactly as SUMO does (MSLane.cpp:2261,2272-2294) instead of
        // charging every one to jam.
        TeleportCount++;
        switch (ClassifyTeleportKind(v))
        {
            case TeleportKind.WrongLane:
                TeleportCountWrongLane++;
                break;
            case TeleportKind.Yield:
                TeleportCountYield++;
                break;
            default:
                TeleportCountJam++;
                break;
        }

        var route = _routesById[EffectiveRouteId(v)];
        var edges = route.Edges;
        var curEdgeId = _network!.LanesByHandle[v.LaneHandle].EdgeId;
        var nextEdgeIndex = NextRouteEdgeIndex(edges, curEdgeId);

        if (_config!.TimeToTeleportRemove || nextEdgeIndex < 0)
        {
            // gRemoveGridlocked (MSLane.cpp:2295-2297), or succEdge(1)==nullptr -- teleport past the
            // arrival edge (MSVehicleTransfer.cpp:65-70): end the trip, no re-insertion.
            _commandBuffer.Destroy(v);
            return;
        }

        // Lift off the lane and enqueue on succEdge(1). Serial phase -> set InTransfer directly.
        v.InTransfer = true;
        _transferQueue.Add(new TransferEntry { Veh = v, EdgeIndex = nextEdgeIndex });
    }

    // P1F-2: index into `edges` of the NEXT NORMAL route edge after `curEdgeId` (SUMO's succEdge(1),
    // which returns normal edges only -- the route's Edges list contains only normal edges, never
    // internal/junction ones). -1 when there is none (curEdgeId is the last edge, or -- out of P1F
    // scope -- the vehicle sits on an internal lane not in the route). The frontmost-stuck jam
    // candidate is always on a normal edge in the committed scenario, so this resolves succEdge(1)
    // exactly (NOTE: this deliberately reads the route rather than _laneSeqPool[..+1], which for a
    // multi-edge route holds the intervening INTERNAL lane, not succEdge(1)'s normal edge).
    private static int NextRouteEdgeIndex(IReadOnlyList<string> edges, string curEdgeId)
    {
        for (var i = 0; i < edges.Count; i++)
        {
            if (edges[i] == curEdgeId)
            {
                return i + 1 < edges.Count ? i + 1 : -1;
            }
        }

        return -1;
    }

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C items 2-3, §2 item 5): MSVehicleTransfer::checkInsertions.
    // Drains the transfer queue -- sorted by EntityIndex ("for repeatable parallel simulation",
    // MSVehicleTransfer.cpp:98) -- trying to re-insert each vehicle on the free lane of its current
    // (jumped-to) edge; failing that, virtual-proceeds it downstream at TeleportMinSpeed. Serial
    // Input-phase pass (the insertion analog), so it relocates vehicle/pool state directly like
    // TryInsertOnLane; removals go through the command buffer. Gated by the caller on
    // TimeToTeleport>0 (and short-circuits on an empty queue).
    private void ProcessTransferQueue(double time)
    {
        if (_transferQueue.Count == 0)
        {
            return;
        }

        _transferQueue.Sort(static (a, b) => a.Veh.EntityIndex.CompareTo(b.Veh.EntityIndex));

        var anyRemoved = false;
        for (var qi = 0; qi < _transferQueue.Count;)
        {
            var entry = _transferQueue[qi];
            var v = entry.Veh;
            var edges = _routesById[EffectiveRouteId(v)].Edges;
            var edge = _network!.EdgesById[edges[entry.EdgeIndex]];

            // getFreeLane: the least-occupied lane of the current edge (single-lane => lane 0).
            var lane = SelectFreeLane(edge);
            if (lane is not null && TryTeleportInsert(v, lane, edges, entry.EdgeIndex))
            {
                // Re-inserted -> teleport ends (MSVehicleTransfer.cpp:173-177).
                _transferQueue.RemoveAt(qi);
                continue;
            }

            // Could not insert -> virtual-proceed (MSVehicleTransfer.cpp:180-206). The proceed
            // clock uses the CURRENT (this-iteration) edge's travel time at TeleportMinSpeed.
            if (entry.ProceedTime < 0)
            {
                entry.ProceedTime = time + EdgeTravelTime(edge);
                qi++;
            }
            else if (entry.ProceedTime < time)
            {
                var nextIdx = entry.EdgeIndex + 1;
                if (nextIdx >= edges.Count)
                {
                    // teleport beyond the arrival edge -> remove (MSVehicleTransfer.cpp:194-199).
                    _commandBuffer.Destroy(v);
                    _transferQueue.RemoveAt(qi);
                    anyRemoved = true;
                    continue;
                }

                // Hop to the next edge; re-arm the clock off THIS iteration's edge (SUMO uses the
                // pre-hop `e` at MSVehicleTransfer.cpp:205).
                entry.EdgeIndex = nextIdx;
                entry.ProceedTime = time + EdgeTravelTime(edge);
                qi++;
            }
            else
            {
                qi++;
            }
        }

        if (anyRemoved)
        {
            _commandBuffer.Flush();
        }
    }

    // P1F-2: MSEdge::getFreeLane analog (reduced) -- the least-occupied lane of `edge` (fewest
    // active vehicles), ties broken by lowest lane index. For the committed single-lane jam this
    // is always lane 0. Returns null only for an edge with no lanes (never in scope).
    // GAP-3 follow-up: a parked vehicle is off the lane in SUMO, so it must not count toward this
    // getFreeLane occupancy tally either (gated on IsParked, byte-identical elsewhere).
    private Lane? SelectFreeLane(Edge edge)
    {
        Lane? best = null;
        var bestCount = int.MaxValue;
        for (var li = 0; li < edge.Lanes.Count; li++)
        {
            var lane = edge.Lanes[li];
            var count = 0;
            foreach (var other in ActiveVehicles())
            {
                if (!other.IsParked && other.LaneHandle == lane.Handle)
                {
                    count++;
                }
            }

            if (count < bestCount)
            {
                bestCount = count;
                best = lane;
            }
        }

        return best;
    }

    // P1F-2: edge travel time at TeleportMinSpeed -- the virtual-proceed hop delay
    // (MSVehicleTransfer.cpp:191, e->getCurrentTravelTime(TeleportMinSpeed)). In a jam the edge's
    // mean speed floors at TeleportMinSpeed, so this reduces to laneLength / TeleportMinSpeed (the
    // "hop downstream at 1 m/s" the design describes).
    private static double EdgeTravelTime(Edge edge) => edge.Lanes[0].Length / TeleportMinSpeed;

    // P1F-2 (HIGH-DENSITY-P1F-DESIGN.md §1C item 2): MSLane::freeInsertion for a teleporting
    // vehicle. Relocates `v` onto `lane` at the minimal free position and installs its route
    // continuation from the jumped-to edge onward. Returns false (leaving `v` in the transfer
    // queue) when the lane is too full to admit it, so the caller virtual-proceeds.
    private bool TryTeleportInsert(VehicleRuntime v, Lane lane, IReadOnlyList<string> edges, int edgeIndex)
    {
        // minPos = MIN2(laneLength, vehLength) for a teleport (MSLane.cpp:492-494) -- for an empty
        // lane the vehicle lands with its back at 0, front at vehLength (the golden's eB pos=5.0).
        // Requested speed = MIN2(laneSpeedLimit, vType max*speedFactor) (MSVehicleTransfer.cpp:171).
        var minPos = Math.Min(lane.Length, v.VType.Length);
        var reqSpeed = KraussModel.LaneVehicleMaxSpeed(lane.Speed, v.SpeedFactor, v.VType);

        // Rearmost active vehicle already on the target lane (MSLane::freeInsertion's leader-gap
        // check, reduced to the last-vehicle branch -- sufficient for the P1-F queue-drain case).
        // GAP-3 follow-up: skip IsParked -- a parked vehicle is off-lane, so it cannot supply the
        // teleport re-insertion gap either (gated, byte-identical elsewhere).
        VehicleRuntime? rearmost = null;
        foreach (var other in ActiveVehicles())
        {
            if (other.IsParked || other.LaneHandle != lane.Handle)
            {
                continue;
            }

            if (rearmost is null || other.Kinematics.Pos < rearmost.Kinematics.Pos)
            {
                rearmost = other;
            }
        }

        var insertSpeed = reqSpeed;
        if (rearmost is not null)
        {
            // Can we sit at minPos behind the rearmost vehicle? gap = its back - minPos - minGap.
            var gap = (rearmost.Kinematics.Pos - rearmost.VType.Length) - minPos - v.VType.MinGap;
            if (gap < 0)
            {
                return false; // no room -> caller virtual-proceeds
            }

            insertSpeed = Math.Min(reqSpeed, KraussModel.MaximumSafeFollowSpeed(
                gap, reqSpeed, rearmost.Kinematics.Speed, rearmost.VType.Decel,
                v.VType, _config!.StepLength, onInsertion: true));
        }
        // Empty-lane NOTE (P1F scope): MSLane::freeInsertion's getMissingRearGap correction (a
        // follower approaching on a predecessor lane pushing minPos further downstream) is not
        // modeled -- the committed jam scenario has none, and SUMO inserts at exactly minPos.

        // Relocate + install the route continuation from this (jumped-to) edge onward. For a single
        // remaining edge this resolves to that one lane (matching TryInsertOnLane's single-edge case).
        var continuation = edgeIndex == 0
            ? edges
            : edges.Skip(edgeIndex).ToList();
        // Issue 1 cross-edge fix: see RerouteActive's matching comment -- a jam-teleported vehicle
        // re-installing its route continuation must keep steering toward a further-down park-and-stay
        // stop's lane too.
        var stopOverride = ParkStopFinalEdgeOverride(v.Def.Stops, continuation);
        var (poolSeq, arrivalSeq) = _network!.ResolveLaneSequenceHandlesWithArrival(continuation, lane.Index, stopOverride: stopOverride);

        v.LaneId = lane.Id;
        v.LaneHandle = lane.Handle;
        v.Kinematics = new Kinematics { Pos = minPos, Speed = insertSpeed, LatOffset = 0.0 };
        v.LaneSeqStart = _laneSeqPool.Count;
        v.LaneSeqLen = poolSeq.Length;
        _laneSeqPool.AddRange(poolSeq);
        _laneSeqArrival.AddRange(arrivalSeq);
        v.LaneSeqIndex = 0;
        // The vehicle is moving again -> clear the mid-teleport flag and the stale waiting-time
        // (ExecuteMoves would reset it this step anyway since speed>HaltingSpeed, but a junction
        // waiting-time reader must never see the pre-teleport 121s).
        v.WaitingTime = 0.0;
        v.InTransfer = false;
        return true;
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

        // Perf (Export-phase parallelism): on a large scenario with NO registered export observer,
        // compute each vehicle's frame concurrently into _emitScratch, then append serially. The
        // determinism/parity test loop and the city benchmark (--fcd-out "") both satisfy the
        // no-observer gate; the FCD-writer observer path falls through to the serial branch below so
        // its file-emission order is preserved exactly. Race-free (index i writes only slot i) and
        // byte-identical (the append below is in EntityIndex order -- identical to the serial
        // ActiveVehicles() order -- so even the emitted XML byte-order matches). Size-gated by
        // ShouldParallelizePlan so every small parity scenario stays on the serial path.
        if (ParallelExport && _exportObservers.Count == 0 && ShouldParallelizePlan())
        {
            if (_emitScratch.Length < _vehicles.Count)
            {
                _emitScratch = new TrajectoryPoint?[_vehicles.Count];
            }

            var scratch = _emitScratch;
            System.Threading.Tasks.Parallel.For(0, _vehicles.Count, _parallelOptions, i =>
            {
                var v = _vehicles[i];
                // P1F-2: a mid-teleport vehicle (InTransfer) is off the network -- not emitted,
                // exactly like an un-inserted/arrived one. Inert unless TimeToTeleport>0.
                if (!v.Inserted || v.Arrived || v.InTransfer)
                {
                    scratch[i] = null;
                    return;
                }

                var laneP = _lanesByHandle[v.LaneHandle];
                var (xp, yp, anglep) = LaneGeometry.PositionAtOffset(laneP.Shape, v.Kinematics.Pos, v.Kinematics.LatOffset);
                scratch[i] = new TrajectoryPoint(
                    VehicleId: v.Def.Id,
                    Time: time,
                    Lane: v.LaneId,
                    Pos: v.Kinematics.Pos,
                    Speed: v.Kinematics.Speed,
                    X: xp,
                    Y: yp,
                    Angle: anglep,
                    Acceleration: null)
                { PosLat = v.Kinematics.LatOffset };
            });

            for (var i = 0; i < _vehicles.Count; i++)
            {
                if (scratch[i] is { } point)
                {
                    trajectory.Add(point);
                }
            }

            return;
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            var lane = _lanesByHandle[v.LaneHandle];
            // B6: render the lateral offset so a swerve is visible in x/y. LatOffset is 0 for every
            // lane-centred vehicle, so this is byte-identical wherever no evasion is active.
            var (x, y, angle) = LaneGeometry.PositionAtOffset(lane.Shape, v.Kinematics.Pos, v.Kinematics.LatOffset);

            var snapshot = new VehicleExportSnapshot(
                entity: v.Entity,
                entityIndex: v.EntityIndex,
                vehicleId: v.Def.Id,
                // P0-B: resolved concrete vType id -- see the read-buffer Add call's own comment
                // above (same reasoning, same byte-identical-for-non-distribution guarantee).
                vehicleType: v.VType.Id,
                time: time,
                lane: v.LaneId,
                pos: v.Kinematics.Pos,
                speed: v.Kinematics.Speed,
                x: x,
                y: y,
                angle: angle,
                giveWaySide: v.GiveWaySide,
                overtakeActive: v.OvertakeActive,
                cooperativeShift: v.CooperativeShift,
                posLat: v.Kinematics.LatOffset,
                edgeSpeedLimit: EdgeSpeedLimitOf(lane),
                isStoppedAtStop: IsStoppedAtStop(v));

            trajectory.Add(new TrajectoryPoint(
                VehicleId: snapshot.VehicleId,
                Time: snapshot.Time,
                Lane: snapshot.Lane,
                Pos: snapshot.Pos,
                Speed: snapshot.Speed,
                X: snapshot.X,
                Y: snapshot.Y,
                Angle: snapshot.Angle,
                Acceleration: null)
            { PosLat = snapshot.PosLat });

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
