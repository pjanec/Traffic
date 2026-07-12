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

    // D3: this vehicle's already-routed-around-once edge set, keyed by EntityIndex -- replaces
    // the old per-vehicle `HashSet<string> AvoidedEdges` managed field. Lazily created only when
    // a vehicle first reroutes (UpdateReroutes); off the hot path (reroute is opt-in via
    // RerouteThresholdSeconds, +infinity by default).
    private readonly Dictionary<int, HashSet<string>> _avoidedByEntity = new();

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
        && _obstacles.Count == 0;

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
    // LoadScenario has not run yet, or UpdateReroutes never actually reroutes anything).
    private NetworkRouter? _router;

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
            if (v.Inserted && !v.Arrived)
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
            if (v.Inserted && !v.Arrived)
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

    // SUMOSHARP-API.md §4.4: resolve a lane's string id to the int lane handle ONCE at setup, so the
    // per-step obstacle path never touches a string. Requires a loaded scenario.
    public int GetLane(string laneId) =>
        (_network ?? throw new InvalidOperationException("LoadScenario must be called before GetLane."))
            .LaneHandleById[laneId];

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

    // SUMOSHARP-API.md §9: load a network WITHOUT any demand -- the "start empty and spawn everything at
    // runtime" entry point (games, digital-twins). Optional sumocfg supplies the timeline/flags; absent,
    // a deterministic default is synthesized (Euler, teleport off, step 1s, sigma-neutral). The host then
    // DefineVType()s and SpawnVehicle()s. Equivalent to LoadScenario with an empty rou.xml.
    public void LoadNetwork(string netXmlPath, string? sumocfgPath = null)
    {
        _network = NetworkParser.Parse(netXmlPath);
        _lanesByHandle = _network.LanesByHandle as Lane[] ?? System.Linq.Enumerable.ToArray(_network.LanesByHandle);
        _demand = EmptyDemand();
        _config = sumocfgPath is null ? DefaultNetworkConfig() : ScenarioConfigParser.Parse(sumocfgPath);
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

        _vehicles.Clear();
        // D3: side storage is keyed by EntityIndex (== _vehicles list index) -- clear it in
        // lockstep with _vehicles so a re-LoadScenario on the same Engine instance never leaves
        // stale entries keyed against the previous scenario's vehicles. The pool only ever grows
        // within one scenario's lifetime; a fresh scenario starts it clean too.
        _laneSeqPool.Clear();
        _laneSeqArrival.Clear();
        _stopsByEntity.Clear();
        _avoidedByEntity.Clear();
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
    private void CreateRuntime(VehicleDef def)
    {
        var rawVType = _vTypesById[def.TypeId];
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
            mean: vType.SpeedFactor, dev: _config!.SpeedDev, min: 0.2, max: 2.0, rng: ref speedFactorRng);

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
    public ReadOnlySpan<float> PosX => _readBuffer.PosX.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> PosY => _readBuffer.PosY.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> PosZ => _readBuffer.PosZ.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> Angle => _readBuffer.Angle.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<float> Speed => _readBuffer.SpeedF.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<int> LaneHandles => _readBuffer.LaneHandle.AsSpan(0, _readBuffer.Count);
    public ReadOnlySpan<double> Pos => _readBuffer.Pos.AsSpan(0, _readBuffer.Count);
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

    // Fill the read buffer from the current active vehicles, projecting each to (x, y, angle) with the SAME
    // LaneGeometry.PositionAtOffset call EmitTrajectory uses -- so the read columns match the FCD geometry
    // exactly. Reads only committed post-step state; mutates nothing in the simulation.
    private void PublishReadState()
    {
        EnsureVehicleGenerationCapacity(_vehicles.Count);
        _readBuffer.BeginFrame(_vehicles.Count);

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

            _readBuffer.Add(handle, v.EntityIndex, v.Def.Id, v.Def.TypeId,
                v.LaneHandle, v.LaneId, v.Kinematics.Pos, v.Kinematics.Speed, v.Kinematics.LatOffset,
                (float)x, (float)y, (float)z, (float)angle);
        }

        DetectLifecycleEvents();
    }

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
            CarFollowModel: p.CarFollowModel);

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
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0)
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
            DepartPos: departPos,
            DepartSpeed: departSpeed,
            DepartLaneIndex: departLane);

        var entityIndex = _vehicles.Count;
        CreateRuntime(def);
        EnsureVehicleGenerationCapacity(_vehicles.Count);
        return new VehicleHandle((uint)entityIndex, _vehicleGeneration[entityIndex]);
    }

    // Spawn a vehicle routed from `fromEdge` to `toEdge` via the engine's shortest-path router. Throws if
    // no route exists (mirrors SUMO refusing an unroutable vehicle).
    public VehicleHandle SpawnVehicle(VTypeHandle type, string fromEdge, string toEdge,
        double departPos = 0.0, double departSpeed = 0.0, int departLane = 0)
    {
        var edges = Router().Route(fromEdge, toEdge)
            ?? throw new InvalidOperationException($"no route from edge '{fromEdge}' to '{toEdge}'.");
        return SpawnVehicle(type, edges, departPos, departSpeed, departLane);
    }

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
    // or an already-arrived vehicle. The slot is not recycled (EntityIndex stays stable).
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

    private NetworkRouter Router() => _router ??= new NetworkRouter(_network!);

    private bool TryResolveActive(VehicleHandle handle, out VehicleRuntime v)
    {
        var idx = (int)handle.Index;
        if (idx >= 0 && idx < _vehicles.Count && idx < _vehicleGeneration.Length
            && _vehicleGeneration[idx] == handle.Generation)
        {
            v = _vehicles[idx];
            if (v.Inserted && !v.Arrived)
            {
                return true;
            }
        }

        v = null!;
        return false;
    }

    // Apply a new remaining route to an active vehicle -- mirrors UpdateReroutes' reassignment exactly:
    // newEdges[0] is the vehicle's current edge, so it stays physically where it is (Kinematics untouched),
    // re-pointed at the freshly resolved lane sequence from here on. Applied immediately via the command
    // buffer (this runs between steps, so the buffer is empty and Flush takes effect at once).
    private void RerouteActive(VehicleRuntime v, IReadOnlyList<string> newEdges)
    {
        var laneIndex = _network!.LanesByHandle[v.LaneHandle].Index;
        var (newPoolSeq, newArrivalSeq) = _network.ResolveLaneSequenceHandlesWithArrival(newEdges, laneIndex);
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
    // Perf (insert): resolve every distinct (route, departLane) insertion lane-sequence up front, in
    // parallel, into _insertRouteSeqCache. Pure function of the immutable network, so byte-identical to
    // lazy resolution -- see the cache field's header and the LoadScenario call site.
    private void PrewarmInsertRouteCache()
    {
        var seen = new HashSet<(string, int)>();
        var keyList = new List<(string RouteId, int DepartLane)>();
        foreach (var def in _demand!.Vehicles)
        {
            var k = (def.RouteId, def.DepartLaneIndex);
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
            var route = _routesById[v.Def.RouteId];
            var edge = _network!.EdgesById[route.Edges[0]];

            // L0d: manual lane-by-index scan instead of `edge.Lanes.First(l => l.Index == ...)`, whose
            // predicate captured `v` into a fresh closure per candidate. Same first-match result.
            string laneId = null!;
            var laneHandle = -1;
            for (var li = 0; li < edge.Lanes.Count; li++)
            {
                if (edge.Lanes[li].Index == v.Def.DepartLaneIndex)
                {
                    laneId = edge.Lanes[li].Id;
                    laneHandle = edge.Lanes[li].Handle;
                    break;
                }
            }

            if (blockedLanes.Contains(laneId))
            {
                // An earlier (same-step) candidate on this lane already failed -- FIFO: later
                // candidates queue behind it and are not attempted this step.
                continue;
            }

            if (!TryInsertOnLane(v, laneHandle))
            {
                blockedLanes.Add(laneId);
            }
        }
    }

    // MSLane::isInsertionSuccess's leader-gap check only (see InsertDepartingVehicles' header
    // comment for the full derivation/scope). Returns true and performs the insertion iff
    // there is no leader on the lane or gap >= 0; otherwise leaves `v` untouched and returns
    // false (queued for a later step).
    private bool TryInsertOnLane(VehicleRuntime v, int laneHandle)
    {
        // R3 (rail bidi): MSLane::isInsertionSuccess (MSLane.cpp:843-846 for the departure lane,
        // :999-1002 for each forward route lane up to the first rail signal) refuses to insert a
        // rail vehicle while the bidi partner of a lane it will occupy carries a vehicle
        // (getBidiLane()->getVehicleNumberWithPartials() > 0). This is SUMO's no-signal
        // single-track deadlock avoidance: a train onto a shared track waits until the opposing
        // train has cleared it. R3 has no rail signals, so the whole forward route is checked.
        // Inert for road vehicles (rail-only) and non-bidi lanes (TryGetBidiLaneId returns null),
        // so every road scenario's insertion is byte-identical.
        if (RailBidiTrackOccupied(v))
        {
            return false;
        }

        var insertPos = v.Def.DepartPos;

        // MSLane::getLastVehicleInformation / getLeader (same-lane branch): nearest already-
        // inserted, not-arrived vehicle with Pos >= insertPos on this lane -- includes any
        // vehicle inserted earlier THIS SAME step, since this re-scans _vehicles (the engine's
        // authoritative list) on every call rather than a stale snapshot.
        VehicleRuntime? leader = null;
        foreach (var other in ActiveVehicles())
        {
            if (other.LaneHandle != laneHandle)
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

        var route = _routesById[v.Def.RouteId];
        // The departure lane is `laneHandle` (resolved by the caller as the edge's lane whose Index
        // == DepartLaneIndex). LanesByHandle[laneHandle] is that exact lane (dense array index, no
        // per-candidate `edge.Lanes.First(...)` predicate-closure alloc), byte-identical to the old
        // resolution: same edge (route.Edges[0]), same first-matching lane index.
        var lane = _network!.LanesByHandle[laneHandle];

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
        // C2-v: resolve the Exit (routing pool) AND Arrival sequences together (see
        // _laneSeqArrival's own comment). Both slices share LaneSeqStart/Len; they differ only where
        // the route requires an intra-edge lane change.
        var routeKey = (v.Def.RouteId, v.Def.DepartLaneIndex);
        if (!_insertRouteSeqCache.TryGetValue(routeKey, out var seq))
        {
            seq = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, v.Def.DepartLaneIndex);
            _insertRouteSeqCache[routeKey] = seq;
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
                v.Def.DepartSpeed, v.VType, v, lane.Handle, v.Def.DepartPos, downstreamAtInsert,
                new ActiveRearmost(this), _config!.StepLength, out var insLeader, out var insGap))
        {
            var insSpeed = KraussModel.MaximumSafeFollowSpeed(
                insGap, v.Def.DepartSpeed, insLeader.Kinematics.Speed, insLeader.VType.Decel,
                v.VType, _config.StepLength, onInsertion: true);
            if (insSpeed < v.Def.DepartSpeed)
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
    private bool RailBidiTrackOccupied(VehicleRuntime v)
    {
        if (!VTypeDefaults.IsRailway(v.VType))
        {
            return false;
        }

        var route = _routesById[v.Def.RouteId];
        var (poolSeq, _) = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, v.Def.DepartLaneIndex);
        foreach (var handle in poolSeq)
        {
            var bidiLaneId = _network.TryGetBidiLaneId(_network.LanesByHandle[handle].Id);
            if (bidiLaneId is null)
            {
                continue;
            }

            foreach (var other in ActiveVehicles())
            {
                if (other.LaneId == bidiLaneId)
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
            foreach (var other in ActiveVehicles())
            {
                if (ReferenceEquals(other, v))
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
                foreach (var other in ActiveVehicles())
                {
                    if (VehicleBodyOccupies(other, viaLaneHandle))
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
    private VehicleRuntime? RearmostOnLaneAmongActive(int laneHandle)
    {
        VehicleRuntime? rearmost = null;
        foreach (var other in ActiveVehicles())
        {
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
            var laneIndex = _network.LanesByHandle[v.LaneHandle].Index;
            // C2-v: append BOTH the Exit (pool) and Arrival slices in lockstep (they share
            // LaneSeqStart/Len). The reroute keeps the vehicle physically where it is (arrival[0] ==
            // its current lane), so for the common no-intra-change route this is identical to before.
            var (newPoolSeq, newArrivalSeq) = _network.ResolveLaneSequenceHandlesWithArrival(newEdges, laneIndex);
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

        return new MoveIntent
        {
            NewSpeed = newSpeed,
            // B6: emergency lateral evasion. Only in the real pass (the willPass pre-pass keeps 0 so it
            // stays side-effect-free); ComputeLateralEvasion returns 0 for every vehicle with no
            // dodgeable obstacle in range, so this is inert wherever no lateral obstacle is present.
            // ER5 additionally routes an ER3 give-way intent through the same lateral-drift primitive.
            LatOffset = prePass ? 0.0 : ComputeLateralEvasion(v, lane, neighbors, time, dt),
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
            if (IsTargetLaneSafe(v, neighLead, neighFollow, dt))
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
        v.WillPass = intent.NewSpeed > willPassSpeedEps;
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
                var takesCrossingYield = !(egoOnInternal || foeWillNotPass || foeNotApproaching || foeYieldsThisStep || ignoresFoe);
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
        double dt, double time, double actionStepLengthSecs, double laneVehicleMaxSpeed)
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
        if (!egoOnInternal
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
        Array.Clear(_foeApproachFirst);
        Array.Clear(_foeApproachSecond);
        foreach (var v in ActiveVehicles())
        {
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

        var downstream = CollectionsMarshal.AsSpan(_laneSeqPool)
            .Slice(ego.LaneSeqStart + ego.LaneSeqIndex + 1, downstreamCount);

        if (!TryFindCrossJunctionLeader(
                ego.Kinematics.Speed, ego.VType, ego, ego.LaneHandle, ego.Kinematics.Pos,
                downstream, new NeighborRearmost(neighbors, ego), dt, out var leader, out var gap))
        {
            return double.PositiveInfinity;
        }

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
            return DriftToward(curLat, GiveWayEdgeTarget(v, lane), maxStep);
        }

        if (_obstacles.Count == 0)
        {
            // Recentre if a just-cleared give-way (or other) drift left us off-centre; otherwise the
            // unchanged fast path (curLat is already 0 in lane-centred mode -> returns 0 exactly).
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

        if (threat is null)
        {
            return DriftToward(curLat, 0.0, maxStep); // no threat -> recentre toward the lane centre
        }

        // ExternalObstacle is now a value type; capture the resolved threat once (stack copy).
        var th = threat.Value;

        // Swerve ONLY when braking alone cannot stop before the obstacle (the "jumped out, can't stop
        // in time" case). While the obstacle is still strictly AHEAD and ego is still lane-centred and
        // CAN stop, just brake (ObstacleConstraint) and stay centred -- the pre-B6 stop-behind
        // behaviour. Once ego has committed to a swerve (off-centre) or can no longer stop, it evades.
        var stillAhead = threatBack >= v.Kinematics.Pos;
        var gap = threatBack - v.VType.MinGap - v.Kinematics.Pos;
        var brakeGap = KraussModel.BrakeGap(v.Kinematics.Speed, v.VType.Decel, headwayTime: 0.0, dt);
        if (stillAhead && brakeGap <= gap && Math.Abs(curLat) < 1e-6)
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
            return curLat; // cannot dodge -> hold; ObstacleConstraint brakes to a stop behind it
        }

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
            return;
        }

        // D6: the Query() analog -- see ActiveVehicles()'s own comment.
        foreach (var v in ActiveVehicles())
        {
            ExecuteMoveVehicle(v, time, dt);
        }

        _commandBuffer.Flush();
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
                    _commandBuffer.Destroy(v);
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
            (pool, arrival) = _network.ResolveLaneSequenceHandlesWithArrival(remaining, currentLane.Index, forceFirstExitToArrival: true);
        }
        catch (InvalidDataException)
        {
            return false;
        }

        var start = _laneSeqPool.Count;
        _laneSeqPool.AddRange(pool);
        _laneSeqArrival.AddRange(arrival);
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
                CommitLaneChange(v, targetLaneHandle, targetLaneId);
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

    private bool KeepRightStrategicStay(VehicleRuntime v, Lane fromLane, int rightLaneIndex)
    {
        var route = _routesById[v.Def.RouteId];
        if (route.Edges.Count <= 1)
        {
            return false;
        }

        var bestLanes = BestLanesCached(v.Def.RouteId, route.Edges, fromLane.EdgeId);
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
    private void CommitLaneChange(VehicleRuntime v, int targetHandle, string targetId)
    {
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

        var route = _routesById[v.Def.RouteId];
        var bestLanes = BestLanesCached(v.Def.RouteId, route.Edges, lane.EdgeId);

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
                if (!v.Inserted || v.Arrived)
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
                    Acceleration: null);
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
                vehicleType: v.Def.TypeId,
                time: time,
                lane: v.LaneId,
                pos: v.Kinematics.Pos,
                speed: v.Kinematics.Speed,
                x: x,
                y: y,
                angle: angle,
                giveWaySide: v.GiveWaySide,
                overtakeActive: v.OvertakeActive,
                cooperativeShift: v.CooperativeShift);

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
