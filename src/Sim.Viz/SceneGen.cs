using Sim.Core;
using Sim.Core.Bridge;
using Sim.Core.Mixed;
using Sim.Core.Orca;
using Sim.Ingest;
using Sim.Pedestrians;
using Sim.Pedestrians.Crossing;
using Sim.Pedestrians.Lod;
using Sim.Pedestrians.Navigation;
using Sim.Pedestrians.Navigation.Bake;
using Sim.Pedestrians.Obstacles;
using Sim.Pedestrians.Parking;
using Sim.Replication;
using static Sim.Viz.PayloadBuilder;

namespace Sim.Viz;

// Programmatic generation of the laneless-only showcase scenes (C/D/E) that have NO golden FCD
// input -- they are produced by running the engine's own open-space ORCA layer / cross-regime
// bridge here at export time (Sim.Viz links Sim.Core). This is a VISUALIZATION-only driver: it
// exercises the same public APIs the parity tests use (Engine, OrcaCrowd) and never touches the
// engine's committed inputs/goldens or the determinism hash.
internal static class SceneGen
{
    // Disc kinds understood by the front-end palette (template.js DISC_COLORS).
    private const int KindStreamA = 0;   // #38bdf8
    private const int KindStreamB = 1;   // #fb7185
    private const int KindPedestrian = 2; // #c084fc
    internal const int KindPedLowPower = 9;    // #94a3b8 -- PathArc (low-power) pedestrian
    internal const int KindPedHighPower = 10;  // #f97316 -- promoted (full-ORCA) pedestrian
    internal const int KindInterestSource = 11; // #facc15 -- sim-LOD interest source marker
    private const int KindReroutingPedestrian = 3; // #f59e0b -- pedestrian on the reroute-demonstration route
    internal const int KindObstacle = 12;      // #78716c -- static/dynamic box obstacle
    private const int KindParkingCar = 13;     // #4f8ef7 -- the one maneuvering car in the parking demo
    internal const int KindPedPaused = 14;     // #eab308 -- ActivityTimeline Pause / idle-clamp pedestrian
    internal const int KindPedDwellSit = 15;   // #22d3ee -- ActivityTimeline Dwell(visible) pedestrian
    internal const int KindPedTalk = 16;       // #f472b6 -- ActivityTimeline Interact ("talk") pedestrian
    internal const int KindPedWaiter = 17;     // #fbbf24 -- LIVE-POC-3 waiter micro-scenario actor
    internal const int KindSafeZone = 18;      // #22c55e -- evac-district safe-zone (corner) marker

    // ---------------------------------------------------------------------------------------
    // Scene C -- "Car avoids a pedestrian": the cross-regime bridge. A laneless-RVO lane vehicle
    // swerves around a person crossing the bridge lane. Driven end-to-end through the real Engine
    // (LanelessRvo + Engine.CrowdSource), exactly as OrcaCrossRegimeBridgeTests does.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildCarAvoidsPedestrian(string bridgeScenarioDir)
    {
        var engine = new Engine { LanelessRvo = true };
        engine.LoadScenario(
            Path.Combine(bridgeScenarioDir, "net.net.xml"),
            Path.Combine(bridgeScenarioDir, "rou.rou.xml"),
            Path.Combine(bridgeScenarioDir, "config.sumocfg"));

        // ONE pedestrian crossing the lane (start below the carriageway, walk up across it).
        var crowd = new OrcaCrowd();
        var pedIdx = crowd.Add(new Vec2(34, -6.5), 0.35, maxSpeed: 0.55, goal: new Vec2(34, 1.0));
        engine.CrowdSource = crowd;

        // Reuse the real bridge net for the drawn network (one lane band, junctions, etc.).
        var network = BuildNetwork(NetworkParser.Parse(Path.Combine(bridgeScenarioDir, "net.net.xml")));

        // Fixed vehicle slots keyed by the stable handle index, so a slot is always the same car.
        var slotByHandle = new Dictionary<uint, int>();
        var frames = new List<FramePayload>();

        for (var step = 0; step < 26; step++)
        {
            engine.Step();     // engine reads the crowd footprint (ped) via CrowdSource
            crowd.Step(1.0);   // advance the pedestrian

            var handles = engine.VehicleHandles;
            var px = engine.PosX;
            var py = engine.PosY;
            var pa = engine.Angle;

            // Grow the slot table for any newly-seen vehicle.
            for (var i = 0; i < handles.Length; i++)
            {
                if (!slotByHandle.ContainsKey(handles[i].Index))
                {
                    slotByHandle[handles[i].Index] = slotByHandle.Count;
                }
            }

            var v = new double[slotByHandle.Count][];
            for (var i = 0; i < handles.Length; i++)
            {
                var slot = slotByHandle[handles[i].Index];
                v[slot] = new[] { R(px[i]), R(py[i]), R(pa[i]) };
            }

            var ped = crowd.Position(pedIdx);
            var d = new[] { new[] { R(ped.X), R(ped.Y), R(crowd.Radius(pedIdx)), (double)KindPedestrian } };

            frames.Add(new FramePayload(v, d));
        }

        // Backfill: every frame's V must be the final slot count (early frames were built when
        // fewer slots were known); short arrays are padded with nulls (vehicle-not-present).
        NormalizeVehicleSlots(frames, slotByHandle.Count);

        return new ScenePayload(
            "Car avoids a pedestrian",
            "Cross-regime bridge: a laneless-RVO lane vehicle swerves around a person crossing the lane. "
            + "Purple disc = pedestrian (open-space ORCA); box = the SUMO-lane vehicle it mutually avoids.",
            new double[] { 10, -9, 60, 3 },
            network,
            new double[] { 5.0, 1.8 },
            1.0,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Crossing gate" (docs/PEDESTRIAN-POC-PLAN.md POC-2): POC-0's real signalized west
    // crosswalk. Pedestrians accumulate at the near-side portal while the crossing's actual
    // <tlLogic>-derived signal is closed (CrossingGate + CrossingTlReader/CrossingSignalFactory,
    // exactly as CrossingGateCrowdTests drives them -- ported behaviour, not reimplemented), then
    // surge across once the walk phase opens. A car (Engine + Engine.CrowdSource, the same seam
    // BuildCarAvoidsPedestrian/CarStopsForPedestrianTests use) drives through the junction on its
    // own green and halts for a pedestrian standing in its lane: an emergent ORCA-mediated stop,
    // not scripted braking. Network + real SUMO-native signal heads + a sustained pedestrian queue
    // + one car, all driven through the committed Sim.Pedestrians controllers.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildCrossingGate(string scenarioDir)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var westCrossing = pedNetwork.Crossings.Single(c => c.Id == ":c_c3_0");
        var signal = CrossingSignalFactory.ForCrossing(netPath, westCrossing);

        // Portal derivation from the crossing's own centreline endpoints (no hardcoded net
        // coordinates): "near" = the higher-Y end (the plaza/queue side on POC-0's fixture), "far"
        // = the roadway's opposite side -- see CrossingGateCrowdTests' remarks for this geometry.
        var shape = westCrossing.Shape;
        var end0 = shape[0];
        var end1 = shape[^1];
        var portalNear = end0.Y >= end1.Y ? end0 : end1;
        var portalFar = end0.Y >= end1.Y ? end1 : end0;
        var queueDir = portalNear - portalFar;        // CrossingGate normalizes this itself
        var queueDirNorm = queueDir.Normalized();
        var farDir = (portalFar - portalNear).Normalized();
        var destBase = portalFar + (farDir * 8.0);
        var perp = new Vec2(-farDir.Y, farDir.X);
        Vec2 DestinationFor(int k) => destBase + (perp * (k * 0.7)); // spread goals -- avoid shared-goal contention

        const double MaxSpeed = 1.4;
        const double PedRadius = 0.3;
        const double ArriveRadius = 0.4;
        const double QueueSpacing = 2.0;
        const int FrontSlotBuffer = 2; // mirrors CrossingGate's own private constant (see its remarks)
        const int MaxQueuedPeds = 10;
        const int SpawnEveryNSteps = 8;

        var crowd = new OrcaCrowd { MaxNeighbours = 8 };
        var gate = new CrossingGate(crowd, new WaypointFollower(), signal, ArriveRadius, queueDir, QueueSpacing);

        // ----- the car: real Engine, real Krauss driving, obstacle-avoiding the shared crowd -----
        var engine = new Engine();
        engine.LoadNetwork(netPath);
        var vtype = engine.DefineVType(new VTypeParams { VClass = "passenger", Sigma = 0.0 });
        var carHandle = engine.SpawnVehicle(vtype, "nc", "cs", departPos: 5.0);
        engine.CrowdSource = crowd;

        var slotByHandle = new Dictionary<uint, int>();
        var frames = new List<FramePayload>();
        var discsKeyedPerFrame = new List<List<(string Key, double[] Disc)>>();
        var pedIds = new List<(OrcaHandle Handle, string Key)>();

        var registered = 0;
        var jaywalkerAdded = false;
        var jaywalkerReleased = false;
        var jaywalkerAddedAtStep = -1;
        var jaywalkerIdx = default(OrcaHandle);
        var jaywalkerPos = default(Vec2);
        const string JaywalkerKey = "jaywalker";

        const double Dt = 1.0;
        const int steps = 170;
        for (var step = 0; step < steps; step++)
        {
            var now = step * Dt;

            // Wave-spawn crossing pedestrians -- a sustained queue-then-surge over the whole clip,
            // not a single scripted release. Each spawns some distance behind its own eventual
            // queue slot (CrossingGate's own hold-point stagger), so it genuinely walks in.
            if (step % SpawnEveryNSteps == 0 && registered < MaxQueuedPeds)
            {
                var k = registered;
                var holdSlot = portalNear + (queueDirNorm * ((k + FrontSlotBuffer) * QueueSpacing));
                var spawn = holdSlot + (queueDirNorm * 3.0);
                var idx = crowd.Add(spawn, PedRadius, MaxSpeed, goal: spawn);
                var path = new[] { portalNear, portalFar, DestinationFor(k) };
                gate.AddRoute(idx, path, portalIndex: 0, MaxSpeed, now);
                pedIds.Add((idx, $"ped{k}"));
                registered++;
            }

            gate.Update(now);
            engine.Step();
            crowd.Step(Dt);

            // Once the car has settled onto its real travel lane (not the transient sidewalk-lane
            // first reading -- see CarStopsForPedestrianTests' remarks), place a stationary
            // pedestrian directly in its path so the ORCA stop is visible.
            if (!jaywalkerAdded && engine.TryGetVehicle(carHandle, out var settled)
                && !settled.LaneId.EndsWith("_0", StringComparison.Ordinal))
            {
                jaywalkerPos = new Vec2(settled.X, 111.6);
                jaywalkerIdx = crowd.Add(jaywalkerPos, PedRadius, maxSpeed: 0.0, goal: jaywalkerPos);
                jaywalkerAdded = true;
                jaywalkerAddedAtStep = step;
            }

            // A few seconds later the jaywalker finishes crossing and clears the lane, freeing the
            // car to proceed -- the "car waits, then resumes" payoff.
            if (jaywalkerAdded && !jaywalkerReleased && step - jaywalkerAddedAtStep >= 9)
            {
                crowd.SetGoal(jaywalkerIdx, jaywalkerPos + new Vec2(-6.0, 0.0));
                jaywalkerReleased = true;
            }

            // Note: pedestrians are NOT removed from the crowd once their route completes --
            // CrossingGate keeps its own internal per-route bookkeeping keyed by handle for the
            // route's whole lifetime (it has no "unregister" API), so removing a still-registered
            // handle from the crowd would make the next gate.Update() dereference a dead handle.
            // Arrived pedestrians simply idle at their (spread-out) destination, same as
            // CrossingGateCrowdTests.

            var handles = engine.VehicleHandles;
            var px = engine.PosX;
            var py = engine.PosY;
            var pa = engine.Angle;
            for (var i = 0; i < handles.Length; i++)
            {
                if (!slotByHandle.ContainsKey(handles[i].Index))
                {
                    slotByHandle[handles[i].Index] = slotByHandle.Count;
                }
            }

            var v = new double[slotByHandle.Count][];
            for (var i = 0; i < handles.Length; i++)
            {
                var slot = slotByHandle[handles[i].Index];
                v[slot] = new[] { R(px[i]), R(py[i]), R(pa[i]) };
            }

            var discs = new List<(string, double[])>();
            foreach (var (handle, key) in pedIds)
            {
                var p = crowd.Position(handle);
                discs.Add((key, new[] { R(p.X), R(p.Y), PedRadius, (double)KindPedestrian }));
            }

            if (jaywalkerAdded)
            {
                var jp = crowd.Position(jaywalkerIdx);
                discs.Add((JaywalkerKey, new[] { R(jp.X), R(jp.Y), PedRadius, (double)KindPedestrian }));
            }

            frames.Add(new FramePayload(v, Array.Empty<double[]?>()));
            discsKeyedPerFrame.Add(discs);
        }

        NormalizeVehicleSlots(frames, slotByHandle.Count);
        AssignStableDiscSlots(frames, discsKeyedPerFrame);

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        return new ScenePayload(
            "Crossing gate",
            "POC-0's signalized west crosswalk: pedestrians queue at the portal while the light is "
            + "red, then surge across on the real <tlLogic> walk phase (CrossingGate + "
            + "CrossingTlReader). A car crossing the junction on its own green halts for a "
            + "pedestrian standing in its lane -- an emergent ORCA vehicle/pedestrian interaction "
            + "(Engine.CrowdSource), not a scripted stop.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 5.0, 1.8 },
            Dt,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "LOD promotion" (docs/PEDESTRIAN-DESIGN.md §5; docs/PEDESTRIAN-POC-PLAN.md POC-3):
    // low-power PathArc pedestrians walk fixed sidewalk routes across POC-0's junction (O(1) motion,
    // no ORCA) while a MOVING interest source sweeps through; any ped it comes near promotes to a
    // high-power full-ORCA agent (a distinct colour) and demotes back once the source moves away and
    // the dwell elapses -- driven end-to-end through the real PedLodManager + InterestField (never
    // reimplemented here). Pedestrians cycle back and forth across their route for a sustained clip
    // (PedLodManager itself does not loop a completed route -- this driver re-adds each ped with its
    // path reversed once it arrives).
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildLodPromotion(string scenarioDir)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var walkableAddPath = Path.Combine(scenarioDir, "walkable.add.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath, walkableAddPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // Four candidate arm-crossing pairs (near/far sidewalk point of each of the junction's four
        // symmetric arms), mirroring the proven WestNorthArm/EastNorthArm pair PedLodManagerTests /
        // SumoBakeNavigationTests use for the north arm -- rotated to the other three arms by the
        // junction's own 90-degree symmetry. Validated at runtime (nav.FindPath) rather than assumed,
        // so a demo run never crashes on a geometry that turns out not to be perfectly symmetric.
        const double cx = 120.0, cy = 120.0; // POC-0's junction centre (net.net.xml <location netOffset>)
        var candidates = new (Vec2 A, Vec2 B)[]
        {
            (new Vec2(cx - 7.4, cy + 20.0), new Vec2(cx + 7.4, cy + 20.0)), // north arm
            (new Vec2(cx + 20.0, cy - 7.4), new Vec2(cx + 20.0, cy + 7.4)), // east arm
            (new Vec2(cx - 7.4, cy - 20.0), new Vec2(cx + 7.4, cy - 20.0)), // south arm
            (new Vec2(cx - 20.0, cy - 7.4), new Vec2(cx - 20.0, cy + 7.4)), // west arm
        };

        var validPairs = new List<(Vec2[] Forward, Vec2[] Backward)>();
        foreach (var (a, b) in candidates)
        {
            var path = nav.FindPath(a, b);
            if (path is { Count: >= 2 })
            {
                var fwd = path.ToArray();
                var bwd = fwd.Reverse().ToArray();
                validPairs.Add((fwd, bwd));
            }
        }

        if (validPairs.Count == 0)
        {
            throw new InvalidOperationException("BuildLodPromotion: no valid arm-crossing routes found on the net.");
        }

        const double MaxSpeed = 1.3;
        const double PedRadius = 0.3;
        const double ArriveRadius = 0.3;
        const double DwellSeconds = 1.0;
        const double PromoteRadius = 6.0;
        const double DemoteRadius = 13.0;
        const double Dt = 0.2;
        const int Decimate = 2;
        const int steps = 700; // 140s simulated, decimated to 350 recorded frames
        const int MaxPeds = 14;
        const int SpawnEveryNSteps = 20; // a fresh ped roughly every 4s

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var field = new InterestField();
        var sourcePos = new Vec2(cx - 25.0, cy);
        var source = new InterestSource(sourcePos, PromoteRadius, DemoteRadius);
        var sourceId = field.Register(source, InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        // Radius chosen from the routes' own measured span (each arm-crossing route's distance from
        // the junction centre ranges ~10.7 m at its nearest waypoint to ~21.3 m at its arm-end
        // waypoints) so the sweep circle passes close to the busiest (near-junction) part of every
        // route as it comes around to that arm's angular sector.
        const double SweepRadius = 16.0;
        const double SweepPeriod = 70.0; // seconds per revolution

        var pedInfo = new Dictionary<int, (Vec2[] Forward, Vec2[] Backward, bool GoingForward)>();
        var activeIds = new List<int>();
        var nextId = 1;
        var pairCursor = 0;

        void Spawn(double now)
        {
            var (fwd, bwd) = validPairs[pairCursor % validPairs.Count];
            var goingForward = (pairCursor / validPairs.Count) % 2 == 0;
            pairCursor++;

            var id = nextId++;
            var path = goingForward ? fwd : bwd;
            manager.AddPed(id, path, MaxSpeed, PedRadius, now);
            pedInfo[id] = (fwd, bwd, goingForward);
            activeIds.Add(id);
        }

        var snapshots = new List<List<(string Key, double[] Disc)>>();

        var now = 0.0;
        for (var step = 0; step < steps; step++)
        {
            if (step % SpawnEveryNSteps == 0 && activeIds.Count < MaxPeds)
            {
                Spawn(now);
            }

            // Sweep the interest source in a slow circle around the junction so it passes near every
            // arm's route in turn over the clip.
            var angle = (now / SweepPeriod) * 2.0 * Math.PI;
            sourcePos = new Vec2(cx + (SweepRadius * Math.Cos(angle)), cy + (SweepRadius * Math.Sin(angle)));
            field.Move(sourceId, sourcePos);

            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            // Cycle each ped back and forth once it reaches the end of its current leg -- a sustained
            // flow instead of everyone arriving and stopping.
            foreach (var id in activeIds)
            {
                var (fwd, bwd, goingForward) = pedInfo[id];
                var dest = goingForward ? fwd[^1] : bwd[^1];
                if ((manager.PositionOf(id, now) - dest).Abs < 0.75)
                {
                    manager.RemovePed(id);
                    var nextPath = goingForward ? bwd : fwd;
                    manager.AddPed(id, nextPath, MaxSpeed, PedRadius, now);
                    pedInfo[id] = (fwd, bwd, !goingForward);
                }
            }

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>(activeIds.Count + 1);
            foreach (var id in activeIds)
            {
                var pos = manager.PositionOf(id, now);
                var kind = manager.ModelOf(id) == PedDrModel.FreeKinematic ? KindPedHighPower : KindPedLowPower;
                discs.Add(($"ped{id}", new[] { R(pos.X), R(pos.Y), PedRadius, (double)kind }));
            }

            discs.Add(("source", new[] { R(sourcePos.X), R(sourcePos.Y), 1.5, (double)KindInterestSource }));
            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        // Pad out to the sweep's own extent (the source travels further from the junction centre than
        // the lane/junction geometry alone would frame).
        const double pad = 6.0;
        minX = Math.Min(minX, cx - SweepRadius - pad);
        maxX = Math.Max(maxX, cx + SweepRadius + pad);
        minY = Math.Min(minY, cy - SweepRadius - pad);
        maxY = Math.Max(maxY, cy + SweepRadius + pad);

        return new ScenePayload(
            "LOD promotion",
            "Low-power PathArc pedestrians (grey) walk fixed sidewalk routes across the junction at "
            + "O(1) cost -- no ORCA, no neighbour queries. A moving interest source (yellow marker) "
            + "sweeps through; any pedestrian it nears promotes to a full-ORCA, reactive high-power "
            + "agent (orange) and demotes back once the source moves on and a dwell period elapses. "
            + "Driven end-to-end through PedLodManager + InterestField.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Remote (over the wire)" (P3-3, docs/PEDESTRIAN-TASKS.md; docs/PEDESTRIAN-DESIGN.md
    // §7): the SAME LOD-promotion setup as BuildLodPromotion (a sweeping interest source promotes
    // low-power sidewalk walkers to full-ORCA high-power agents and demotes them once it passes), but
    // this scene's discs are drawn from a PedRemoteReconstructor (P3-3, NEW) fed by the real GATED
    // PedReplicationPublisher (PedPublishScheduler + PedBandwidthGovernor, P3-2) over an
    // InMemoryPedReplicationBus (P3-1) -- NOT from PedLodManager.PositionOf/ModelOf directly. Every
    // ped's PathArc leg is sent once; every high-power ped's position is DR-error-gated (sent only
    // when the receiver's own linear extrapolation would drift out of tolerance); the promotion/
    // demotion DR-model switch rides the same lifecycle topic. The reconstructor applies a
    // playout-delay render clock plus capped-correction smoothing so the sparse high-power stream
    // (and the DR-switch itself) render with no visible pop -- this is the whole server -> wire -> IG
    // -> render loop, watchable.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildPedRemote(string scenarioDir)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var walkableAddPath = Path.Combine(scenarioDir, "walkable.add.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath, walkableAddPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // Same four arm-crossing candidates as BuildLodPromotion (see its remarks for the geometry).
        const double cx = 120.0, cy = 120.0;
        var candidates = new (Vec2 A, Vec2 B)[]
        {
            (new Vec2(cx - 7.4, cy + 20.0), new Vec2(cx + 7.4, cy + 20.0)), // north arm
            (new Vec2(cx + 20.0, cy - 7.4), new Vec2(cx + 20.0, cy + 7.4)), // east arm
            (new Vec2(cx - 7.4, cy - 20.0), new Vec2(cx + 7.4, cy - 20.0)), // south arm
            (new Vec2(cx - 20.0, cy - 7.4), new Vec2(cx - 20.0, cy + 7.4)), // west arm
        };

        var validPairs = new List<(Vec2[] Forward, Vec2[] Backward)>();
        foreach (var (a, b) in candidates)
        {
            var path = nav.FindPath(a, b);
            if (path is { Count: >= 2 })
            {
                var fwd = path.ToArray();
                var bwd = fwd.Reverse().ToArray();
                validPairs.Add((fwd, bwd));
            }
        }

        if (validPairs.Count == 0)
        {
            throw new InvalidOperationException("BuildPedRemote: no valid arm-crossing routes found on the net.");
        }

        const double MaxSpeed = 1.3;
        const double PedRadius = 0.3;
        const double ArriveRadius = 0.3;
        const double DwellSeconds = 1.0;
        const double PromoteRadius = 6.0;
        const double DemoteRadius = 13.0;
        const double Dt = 0.2;
        const int Decimate = 2;
        const int steps = 700; // 140s simulated, decimated to 350 recorded frames
        const int MaxPeds = 14;
        const int SpawnEveryNSteps = 20; // a fresh ped roughly every 4s

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, ArriveRadius, DwellSeconds);
        var field = new InterestField();
        var sourcePos = new Vec2(cx - 25.0, cy);
        var source = new InterestSource(sourcePos, PromoteRadius, DemoteRadius);
        var sourceId = field.Register(source, InterestSourceKind.EntityAttached);
        var noEntities = Array.Empty<WorldDisc>();

        // The wire: gated publisher (DR-error scheduler + global bandwidth governor, P3-2) ->
        // InMemoryPedReplicationBus (a true byte loopback, P3-1) -> PedRemoteReconstructor (P3-3). The
        // demo's discs are drawn from THIS reconstructor's render poses (below), not from
        // manager.PositionOf/ModelOf -- the rendered crowd really is reconstructed from the one
        // multicast stream.
        var meter = new PedBandwidthMeter();
        var scheduler = new PedPublishScheduler(new PedDrErrorPublishPolicy());
        var governor = new PedBandwidthGovernor(scheduler, meter, maxMbitPerSecond: 500.0);
        var bus = new InMemoryPedReplicationBus();
        var wirePublisher = new PedReplicationPublisher(bus.Sink, scheduler, governor, meter, stepDt: Dt);
        var reconstructor = new PedRemoteReconstructor(bus.Source);

        // Radius chosen from the routes' own measured span, same as BuildLodPromotion.
        const double SweepRadius = 16.0;
        const double SweepPeriod = 70.0; // seconds per revolution

        var pedInfo = new Dictionary<int, (Vec2[] Forward, Vec2[] Backward, bool GoingForward)>();
        var activeIds = new List<int>();
        var nextId = 1;
        var pairCursor = 0;

        void Spawn(double now)
        {
            var (fwd, bwd) = validPairs[pairCursor % validPairs.Count];
            var goingForward = (pairCursor / validPairs.Count) % 2 == 0;
            pairCursor++;

            var id = nextId++;
            var path = goingForward ? fwd : bwd;
            manager.AddPed(id, path, MaxSpeed, PedRadius, now);
            pedInfo[id] = (fwd, bwd, goingForward);
            activeIds.Add(id);
        }

        var snapshots = new List<List<(string Key, double[] Disc)>>();

        var now = 0.0;
        for (var step = 0; step < steps; step++)
        {
            // Whole-step wire batch: capture the event-list cursor BEFORE this step's spawn/promotion/
            // demotion so a freshly-spawned ped's spawn-time PathArcRecord (published by AddPed, above)
            // rides the SAME wire batch as everything PedLodManager.Step emits this tick -- exactly
            // the whole-step publish granularity PedReplicationRoundTripTests/PedRemoteReconstructorTests
            // rely on (a promotion's DR-switch and its first FreeKinematicSample always land in the same
            // Drain(), so the receiver never observes a "switched but no sample yet" ped).
            var beforeCount = publisher.Events.Count;

            if (step % SpawnEveryNSteps == 0 && activeIds.Count < MaxPeds)
            {
                Spawn(now);
            }

            // Sweep the interest source in a slow circle around the junction so it passes near every
            // arm's route in turn over the clip.
            var angle = (now / SweepPeriod) * 2.0 * Math.PI;
            sourcePos = new Vec2(cx + (SweepRadius * Math.Cos(angle)), cy + (SweepRadius * Math.Sin(angle)));
            field.Move(sourceId, sourcePos);

            manager.Step(now, Dt, field, noEntities);
            now += Dt;

            // Cycle each ped back and forth once it reaches the end of its current leg -- a sustained
            // flow instead of everyone arriving and stopping. Only recycles a currently LOW-power ped:
            // RemovePed/AddPed here is a demand-side "reached destination, walk back" reset, not
            // PedLodManager's own promote/demote transition, so it never publishes a DR-switch: cycling
            // a still-FreeKinematic ped this way would desync the wire (the reconstructor would keep
            // believing it is high-power with no more samples ever coming). A high-power ped instead
            // demotes and re-routes through PedLodManager.Step's own dwell-based mechanism (which DOES
            // publish the DR-switch + a fresh PathArcRecord), then cycles normally next time it arrives
            // as a plain PathArc ped.
            foreach (var id in activeIds)
            {
                if (manager.ModelOf(id) != PedDrModel.PathArc)
                {
                    continue;
                }

                var (fwd, bwd, goingForward) = pedInfo[id];
                var dest = goingForward ? fwd[^1] : bwd[^1];
                if ((manager.PositionOf(id, now) - dest).Abs < 0.75)
                {
                    manager.RemovePed(id);
                    var nextPath = goingForward ? bwd : fwd;
                    manager.AddPed(id, nextPath, MaxSpeed, PedRadius, now);
                    pedInfo[id] = (fwd, bwd, !goingForward);
                }
            }

            var newEvents = new List<PedEvent>(publisher.Events.Count - beforeCount);
            for (var e = beforeCount; e < publisher.Events.Count; e++)
            {
                newEvents.Add(publisher.Events[e]);
            }

            wirePublisher.Publish(newEvents);
            reconstructor.Pump(now);

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>(activeIds.Count + 1);
            foreach (var id in activeIds)
            {
                if (!reconstructor.TryGetRenderPose(id, out var pos, out var visible, out _) || !visible)
                {
                    continue; // not yet observed on the wire this run, or (a future liveliness combo) hidden
                }

                var kind = reconstructor.Ig.ModelOf(id) == PedDrModel.FreeKinematic ? KindPedHighPower : KindPedLowPower;
                discs.Add(($"ped{id}", new[] { R(pos.X), R(pos.Y), PedRadius, (double)kind }));
            }

            discs.Add(("source", new[] { R(sourcePos.X), R(sourcePos.Y), 1.5, (double)KindInterestSource }));
            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        const double pad = 6.0;
        minX = Math.Min(minX, cx - SweepRadius - pad);
        maxX = Math.Max(maxX, cx + SweepRadius + pad);
        minY = Math.Min(minY, cy - SweepRadius - pad);
        maxY = Math.Max(maxY, cy + SweepRadius + pad);

        return new ScenePayload(
            "Remote (over the wire)",
            "This crowd is rendered from the one multicast stream, not the sim: each pedestrian's "
            + "sidewalk path is sent once (grey = low-power PathArc), a moving interest source (yellow "
            + "marker) promotes nearby walkers to full-ORCA high-power agents (orange) whose positions "
            + "are DR-error-gated onto the wire (sent only when the receiver's own dead-reckoning would "
            + "drift out of tolerance), and a playout-delay render clock plus capped-correction smoothing "
            + "reconstruct every pose -- including the promotion/demotion DR-switch itself -- with no "
            + "visible pop. Driven end-to-end through PedLodManager -> the gated PedReplicationPublisher "
            + "-> InMemoryPedReplicationBus -> PedRemoteReconstructor: server == IG within render "
            + "tolerance.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Liveliness" (LIVE-POC-1, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §12): a handful of
    // pedestrians, each a pure deterministic ActivityTimeline replay -- Walk to a sip stop (Pause),
    // Walk to a table (Dwell, visible -- sat down), Walk to a building door (Dwell, INVISIBLE -- gone
    // inside), re-emerge, Walk to the exit. No behavior loop runs here at all: every frame just calls
    // ActivityTimeline.PoseAt(now) per ped -- the exact function the server AND the IG would call
    // (HeadlessIg.ReconstructSample), open-space (no net file), open-space pure ORCA-free schedule.
    // Disc colour/label is driven by the sampled AnimTag (grey = walk, yellow = paused/idle, cyan =
    // dwelling/seated); a Dwell(visible:false) sample emits NO disc for that ped that frame -- the
    // "gone inside, no cheating" liveliness/no-visible-cheating synergy (§6, §11).
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildLiveliness()
    {
        const int pedCount = 5;
        const double laneSpacing = 4.5;
        const double speed = 1.3;
        const double dt = 0.2;
        const int decimate = 1;

        var timelines = new List<(int Id, ActivityTimeline Timeline)>();
        for (var i = 0; i < pedCount; i++)
        {
            var laneY = (i - (pedCount - 1) / 2.0) * laneSpacing;
            var spawn = new Vec2(-24.0, laneY);
            var sipPoint = new Vec2(-12.0, laneY);
            var tablePoint = new Vec2(-2.0, laneY + 2.2);
            var doorPoint = new Vec2(10.0, laneY - 2.0);
            var exit = new Vec2(24.0, laneY);

            // Staggered spawn so every ped is in a DIFFERENT segment of its own timeline at any given
            // frame -- someone is always walking, someone paused, someone seated, someone inside.
            var t0 = i * 3.0;

            var segments = new ActivitySegment[]
            {
                new WalkSegment(new[] { spawn, sipPoint }, speed),
                new PauseSegment(2.5, "sip"),
                new WalkSegment(new[] { sipPoint, tablePoint }, speed),
                new DwellSegment(tablePoint, new Vec2(0.0, 1.0), 4.0, "sit", Visible: true),
                new WalkSegment(new[] { tablePoint, doorPoint }, speed),
                new DwellSegment(doorPoint, new Vec2(1.0, 0.0), 3.0, "enter", Visible: false),
                new WalkSegment(new[] { doorPoint, exit }, speed),
            };

            timelines.Add((i + 1, new ActivityTimeline(t0, segments)));
        }

        var maxEnd = 0.0;
        foreach (var (_, timeline) in timelines)
        {
            maxEnd = Math.Max(maxEnd, timeline.EndTime);
        }

        var steps = (int)((maxEnd + 3.0) / dt);
        var snapshots = new List<List<(string Key, double[] Disc)>>(steps);

        for (var step = 0; step < steps; step++)
        {
            if (step % decimate != 0)
            {
                continue;
            }

            var now = step * dt;
            var discs = new List<(string, double[])>(pedCount);
            foreach (var (id, timeline) in timelines)
            {
                var sample = timeline.PoseAt(now);
                if (!sample.Visible)
                {
                    continue; // inside the building this frame -- no disc, no cheating
                }

                var kind = sample.AnimTag switch
                {
                    ActivityTimeline.WalkAnimTag => KindPedLowPower,
                    "sit" => KindPedDwellSit,
                    _ => KindPedPaused, // "sip", the before/after Idle clamp, and any future Pause tag
                };

                discs.Add(($"ped{id}", new[] { R(sample.Pos.X), R(sample.Pos.Y), 0.3, (double)kind }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        return new ScenePayload(
            "Liveliness (activity timeline replay)",
            "Deterministic activity-timeline replay (LIVE-POC-1): each low-power pedestrian's pose AND "
            + "animation state is a pure function of (timeline, now) -- ActivityTimeline.PoseAt -- the "
            + "same one-time-broadcast, server==IG trick as PathArc, generalized to Walk+Pause+Dwell. "
            + "Grey = walking, yellow = paused (sipping/idle), cyan = dwelling (seated at a table); a "
            + "pedestrian INSIDE the building (Dwell, hidden) renders no disc at all until it re-emerges.",
            new double[] { -28, -13, 28, 13 },
            null,
            new double[] { 0, 0 },
            dt * decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Meet & talk" (LIVE-POC-2, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §5, §12): several
    // PAIRS of pedestrians on converging approaches, each pair paired up front by SocialPlanner --
    // both peds' Walk->Interact->Walk timelines are authored TOGETHER (matching meet point/time/
    // duration, opposite step-aside offsets, headings aimed at each other) purely from their nominal
    // O->D plans, with NO runtime negotiation between them. Rendered exactly like BuildLiveliness:
    // ActivityTimeline.PoseAt(now) per ped, the same function the server AND the IG would call. The
    // "talk" AnimTag drives a distinct disc colour (KindPedTalk) so the stepped-aside, face-to-face
    // conversation reads visually distinct from ordinary low-power walking.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildSocial()
    {
        const int pairCount = 3;
        const double laneSpacing = 11.0;
        const double speed = 1.3;
        const double dt = 0.2;
        const double meetOffset = 0.6;
        const double talkDuration = 4.0;

        var timelines = new List<(int Id, ActivityTimeline Timeline)>();
        for (var p = 0; p < pairCount; p++)
        {
            var laneY = (p - (pairCount - 1) / 2.0) * laneSpacing;
            var crossX = (p - (pairCount - 1) / 2.0) * 3.0; // stagger each pair's crossing point a bit

            // Ped A walks west->east along its own lane; ped B walks south->north crossing that lane
            // at (crossX, laneY) -- a converging pair of approaches meeting near the lane centre.
            var pedAPath = new[] { new Vec2(-18.0, laneY), new Vec2(18.0, laneY) };
            var pedBPath = new[] { new Vec2(crossX, laneY - 18.0), new Vec2(crossX, laneY + 18.0) };

            var pedA = new PedPlan(p * 2 + 1, pedAPath, 0.0, speed);
            var pedB = new PedPlan(p * 2 + 2, pedBPath, 0.0, speed);

            var (timelineA, timelineB) = SocialPlanner.ScheduleInteraction(pedA, pedB, meetOffset, talkDuration);
            timelines.Add((pedA.Id, timelineA));
            timelines.Add((pedB.Id, timelineB));
        }

        var maxEnd = 0.0;
        foreach (var (_, timeline) in timelines)
        {
            maxEnd = Math.Max(maxEnd, timeline.EndTime);
        }

        var steps = (int)((maxEnd + 3.0) / dt);
        var snapshots = new List<List<(string Key, double[] Disc)>>(steps);

        for (var step = 0; step < steps; step++)
        {
            var now = step * dt;
            var discs = new List<(string, double[])>(timelines.Count);
            foreach (var (id, timeline) in timelines)
            {
                var sample = timeline.PoseAt(now);
                if (!sample.Visible)
                {
                    continue;
                }

                var kind = sample.AnimTag == SocialPlanner.TalkAnimTag ? KindPedTalk : KindPedLowPower;
                discs.Add(($"ped{id}", new[] { R(sample.Pos.X), R(sample.Pos.Y), 0.3, (double)kind }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        return new ScenePayload(
            "Meet & talk",
            "LIVE-POC-2: a pre-scheduled, deterministic two-pedestrian interaction. SocialPlanner pairs "
            + "each converging pair of walkers and writes MATCHING Interact segments into both of their "
            + "ActivityTimelines -- same meet time/duration, opposite step-aside offsets, headings aimed "
            + "at each other -- entirely at schedule time, from their nominal walk plans. No runtime "
            + "negotiation between the two peds: replay is still just ActivityTimeline.PoseAt(now), so "
            + "server == IG and the pair stays exactly as low-power as a solo walker. Pink = talking.",
            new double[] { -22, -20, 22, 20 },
            null,
            new double[] { 0, 0 },
            dt,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Waiter" (LIVE-POC-3, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §7, §12): a templated,
    // scripted, low-power actor bound to a (building, table-cluster) anchor. A restaurant building
    // (drawn as a static box, KindObstacle reused for the footprint) has a service door on its near
    // edge; an open-air cluster of tables sits in front of it, each with 1-2 seated patrons -- their
    // OWN long-lived visible Dwell ActivityTimelines (KindPedDwellSit, exactly LIVE-POC-1's "seated"
    // rendering). The waiter itself is a SINGLE WaiterScenario.Build(...) timeline: it emerges from the
    // door, walks to a table, Dwell(serve), walks back, Dwell(inside, hidden) -- and loops, visiting
    // tables in the scenario's seed-varied rotation. Nothing here runs a behavior loop: every frame just
    // calls ActivityTimeline.PoseAt(now) per actor (patrons AND waiter), exactly like BuildLiveliness/
    // BuildSocial -- so the waiter is exactly as low-power and server==IG-reconstructable as a solo
    // walker. The waiter's disc (KindPedWaiter) is omitted entirely while its sampled Visible is false
    // (the hidden inside-dwell, §6) -- "gone inside, no cheating", the same rule BuildLiveliness's
    // building draws on.
    // ---------------------------------------------------------------------------------------
    // Restaurant building footprint (a static box, drawn via the shared KindObstacle box encoding) and
    // the table cluster in front of it -- shared, closed-form layout constants so Program.cs's report
    // can rebuild the EXACT same waiter ActivityTimeline (via BuildWaiterTimeline below) for its
    // hidden/serving evidence without duplicating a second copy of these numbers.
    private static readonly Vec2 WaiterBuildingCenter = new(0.0, -7.0);
    private const double WaiterBuildingHalfLen = 6.0; // half-extent along X
    private const double WaiterBuildingHalfWid = 3.0; // half-extent along Y
    private static readonly Vec2 WaiterDoorPos = new(0.0, WaiterBuildingCenter.Y + WaiterBuildingHalfWid); // (0, -4)

    private static readonly Vec2[] WaiterTables =
    {
        new(-4.5, 1.0),
        new(-1.5, 2.5),
        new(1.5, 2.5),
        new(4.5, 1.0),
    };

    private const double WaiterSpeed = 1.6;
    private const double WaiterServeSeconds = 3.0;
    private const double WaiterInsideSeconds = 2.0;
    private const int WaiterLoops = 6;
    private const ulong WaiterSeed = 20260718UL;

    // The waiter's own micro-scenario timeline -- ONE WaiterScenario.Build call, the single source of
    // truth both BuildWaiter (rendering) and Program.cs's --ped-waiter report (hidden/serving evidence)
    // reconstruct from, so the two can never drift apart.
    internal static ActivityTimeline BuildWaiterTimeline()
    {
        var cfg = new WaiterScenarioConfig(
            WaiterDoorPos, WaiterTables, StartTime: 0.0, WaiterSpeed, WaiterServeSeconds, WaiterInsideSeconds,
            WaiterLoops, WaiterSeed);
        return WaiterScenario.Build(cfg);
    }

    internal static ScenePayload BuildWaiter()
    {
        const double dt = 0.15;
        const double pedRadius = 0.3;

        var buildingCenter = WaiterBuildingCenter;
        const double buildingHalfLen = WaiterBuildingHalfLen;
        const double buildingHalfWid = WaiterBuildingHalfWid;
        var tables = WaiterTables;

        // 1-2 seated patrons per table (alternating), each a static (whole-clip) visible Dwell --
        // its own ActivityTimeline, unrelated to the waiter's, rendered exactly like BuildLiveliness's
        // "sit" dwells. Patron seats are offset a little off the table's own centre (the waiter's exact
        // serve pose) so the two kinds of disc don't fully coincide.
        var waiterTimeline = BuildWaiterTimeline();

        var clipDuration = waiterTimeline.EndTime + 3.0;

        var patronTimelines = new List<ActivityTimeline>();
        var seatOffsets = new[] { new Vec2(-0.7, 0.4), new Vec2(0.7, -0.4) };
        for (var t = 0; t < tables.Length; t++)
        {
            var seatCount = t % 2 == 0 ? 2 : 1;
            for (var s = 0; s < seatCount; s++)
            {
                var seatPos = tables[t] + seatOffsets[s];
                var toTable = tables[t] - seatPos;
                var heading = toTable.Abs > 1e-9 ? toTable.Normalized() : Vec2.Zero;
                var segments = new ActivitySegment[]
                {
                    new DwellSegment(seatPos, heading, clipDuration + 10.0, "sit", Visible: true),
                };
                patronTimelines.Add(new ActivityTimeline(0.0, segments));
            }
        }

        var steps = (int)(clipDuration / dt) + 1;
        var snapshots = new List<List<(string Key, double[] Disc)>>(steps);

        for (var step = 0; step < steps; step++)
        {
            var now = step * dt;
            var discs = new List<(string, double[])>(patronTimelines.Count + 2);

            // The building footprint -- static every frame, drawn as a box via the shared obstacle
            // disc encoding [x, y, radius, kind, angleDeg, shape, halfLen, halfWid].
            discs.Add(("building", new[]
            {
                R(buildingCenter.X), R(buildingCenter.Y), 1.0, (double)KindObstacle, 0.0, 0.0,
                buildingHalfLen, buildingHalfWid,
            }));

            for (var p = 0; p < patronTimelines.Count; p++)
            {
                var sample = patronTimelines[p].PoseAt(now);
                if (sample.Visible)
                {
                    discs.Add(($"patron{p}", new[] { R(sample.Pos.X), R(sample.Pos.Y), pedRadius, (double)KindPedDwellSit }));
                }
            }

            var waiterSample = waiterTimeline.PoseAt(now);
            if (waiterSample.Visible)
            {
                discs.Add(("waiter", new[] { R(waiterSample.Pos.X), R(waiterSample.Pos.Y), pedRadius, (double)KindPedWaiter }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        // Tight framing (tens of metres) around the building + table cluster -- a phone-watchable
        // close-up of the one micro-scenario, the same "not the whole net" discipline BuildObstacleDodge/
        // BuildCrossingReroute use.
        return new ScenePayload(
            "Waiter",
            "LIVE-POC-3: a templated, scripted micro-scenario actor. A waiter emerges from the "
            + "restaurant's service door, walks to a table in the seed-varied rotation, Dwells (serves) "
            + "there, walks back, and Dwells (goes inside, no disc) before the next round -- all from ONE "
            + "WaiterScenario.Build(...) ActivityTimeline, so it stays exactly as low-power and "
            + "server==IG-reconstructable as a solo walker. Seated patrons (cyan) are their own static "
            + "Dwell timelines. Amber = the waiter (visible only outside the building).",
            new double[] { -9.0, -12.0, 9.0, 7.0 },
            null,
            new double[] { 0, 0 },
            dt,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "OD routing" (docs/PEDESTRIAN-NAVMESH-CONTRACT.md "OD demand"; docs/PEDESTRIAN-
    // DESIGN.md §4): PedDemand spawns pedestrians on a Poisson process, each routed origin-
    // >destination across the junction's real sidewalks/crossings/walkingareas (SumoNavMesh), and
    // despawns them on arrival -- a sustained routed crowd on the real pedestrian network, not a
    // scripted one-shot. No LOD/interest-source layer here (an empty InterestField): every
    // pedestrian stays low-power PathArc, which is exactly PedDemand's own "low-power motion is the
    // cheap default" behaviour.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildOdRouting(string scenarioDir)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var walkableAddPath = Path.Combine(scenarioDir, "walkable.add.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath, walkableAddPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // The same four arms' near/far sidewalk points BuildLodPromotion validates, reused here as a
        // flat OD point set -- any point may be drawn as either an origin or a destination, so demand
        // naturally crisscrosses the junction across every arm.
        const double cx = 120.0, cy = 120.0;
        var odPoints = new[]
        {
            new Vec2(cx - 7.4, cy + 20.0), new Vec2(cx + 7.4, cy + 20.0),  // north arm
            new Vec2(cx + 20.0, cy - 7.4), new Vec2(cx + 20.0, cy + 7.4),  // east arm
            new Vec2(cx - 7.4, cy - 20.0), new Vec2(cx + 7.4, cy - 20.0),  // south arm
            new Vec2(cx - 20.0, cy - 7.4), new Vec2(cx - 20.0, cy + 7.4),  // west arm
        };

        var config = new Sim.Pedestrians.Demand.PedDemandConfig
        {
            Origins = odPoints,
            Destinations = odPoints,
            SpawnRatePerSecond = 0.5,
            PopulationCap = 14,
            Seed = 20260718UL,
            MaxSpeed = 1.3,
            Radius = 0.3,
            ArrivalRadius = 0.6,
        };

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, arriveRadius: 0.3, dwellSeconds: 1.0);
        var demand = new Sim.Pedestrians.Demand.PedDemand(config, nav, manager, startTime: 0.0);
        var field = new InterestField(); // no interest sources -- everyone stays low-power PathArc
        var noEntities = Array.Empty<WorldDisc>();

        const double Dt = 0.2;
        const int Decimate = 2;
        const int steps = 600; // 120s simulated, decimated to 300 recorded frames

        var snapshots = new List<List<(string Key, double[] Disc)>>();

        // Fix "O-D peds stack on top of each other" (low-power PathArc peds walk exact centrelines,
        // so concurrent peds on the same sidewalk/crossing literally coincide): a deterministic
        // per-ped lateral offset, perpendicular to the ped's own heading, spreads them into a loose
        // band instead. Heading is finite-differenced against the ped's own PREVIOUS RECORDED-FRAME
        // position (last-recorded-pos dictionary below); a ped with no recorded history yet (just
        // spawned) or ~zero displacement (arrived, idling) gets zero offset so it still spawns/arrives
        // exactly on its O/D point -- only rendering is perturbed, never the sim position itself.
        var lastRecordedPos = new Dictionary<int, Vec2>();

        var now = 0.0;
        for (var step = 0; step < steps; step++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>(demand.LiveIds.Count);
            foreach (var id in demand.LiveIds)
            {
                var pos = manager.PositionOf(id, now);
                var heading = lastRecordedPos.TryGetValue(id, out var prev) ? pos - prev : Vec2.Zero;
                lastRecordedPos[id] = pos;

                var rendered = pos;
                if (heading.Abs > 1e-6)
                {
                    var headingUnit = heading.Normalized();
                    var perp = new Vec2(-headingUnit.Y, headingUnit.X);
                    rendered += perp * OdLateralOffsetMeters(id);
                }

                discs.Add(($"ped{id}", new[] { R(rendered.X), R(rendered.Y), 0.3, (double)KindPedestrian }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        return new ScenePayload(
            "OD routing",
            "PedDemand spawns pedestrians on a Poisson process, each routed origin->destination across "
            + "the junction's real sidewalks, crossings, and walkingareas via SumoNavMesh, and despawns "
            + "them on arrival -- a sustained routed crowd on the real pedestrian network. Driven "
            + "end-to-end through the committed PedDemand + PedLodManager.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Lively crowd" (LIVE-PROD-1b, docs/PEDESTRIAN-LIVELINESS-DESIGN.md §4;
    // docs/PEDESTRIAN-TASKS.md): the SAME routed O-D crowd as BuildOdRouting, above, but with
    // PedDemandConfig.Liveliness turned ON -- each spawn's route becomes an ActivityTimeline with
    // seeded Pause ("sip") beats spliced in, via PedDemand.TrySpawnOne's new AddPedLively branch,
    // rather than a bare PathArc. Still low-power (ActivityTimeline.PoseAt is the same O(1) pure-
    // function-of-time evaluator PathArc used) and still server==IG (the whole timeline is broadcast
    // once by PedLodManager.AddPedLively, exactly like AddPed's "path sent once"). Disc colour is
    // driven by PedLodManager.AnimTagOf(id, now): a paused ped (anything other than
    // ActivityTimeline.WalkAnimTag -- here, just "sip") renders as KindPedPaused (yellow); a walking
    // ped renders as the ordinary KindPedestrian (purple), matching BuildOdRouting's own look so the
    // only visible difference IS the new paused state, not a palette change.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildLivelyCrowd(string scenarioDir)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var walkableAddPath = Path.Combine(scenarioDir, "walkable.add.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath, walkableAddPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        // Same four arms' OD point set as BuildOdRouting.
        const double cx = 120.0, cy = 120.0;
        var odPoints = new[]
        {
            new Vec2(cx - 7.4, cy + 20.0), new Vec2(cx + 7.4, cy + 20.0),  // north arm
            new Vec2(cx + 20.0, cy - 7.4), new Vec2(cx + 20.0, cy + 7.4),  // east arm
            new Vec2(cx - 7.4, cy - 20.0), new Vec2(cx + 7.4, cy - 20.0),  // south arm
            new Vec2(cx - 20.0, cy - 7.4), new Vec2(cx - 20.0, cy + 7.4),  // west arm
        };

        var config = new Sim.Pedestrians.Demand.PedDemandConfig
        {
            Origins = odPoints,
            Destinations = odPoints,
            SpawnRatePerSecond = 0.5,
            PopulationCap = 14,
            Seed = 20260718UL,
            MaxSpeed = 1.3,
            Radius = 0.3,
            ArrivalRadius = 0.6,
            Liveliness = new Sim.Pedestrians.Demand.PedLivelinessConfig
            {
                PauseProbability = 0.6,
                MinPauseSeconds = 2.0,
                MaxPauseSeconds = 5.0,
                MaxPausesPerTrip = 2,
                PauseAnimTag = "sip",
            },
        };

        var publisher = new PedPublisher();
        var manager = new PedLodManager(nav, publisher, arriveRadius: 0.3, dwellSeconds: 1.0);
        var demand = new Sim.Pedestrians.Demand.PedDemand(config, nav, manager, startTime: 0.0);
        var field = new InterestField(); // no interest sources -- everyone stays low-power (PathArc or ActivityTimeline)
        var noEntities = Array.Empty<WorldDisc>();

        const double Dt = 0.2;
        const int Decimate = 2;
        const int steps = 600; // 120s simulated, decimated to 300 recorded frames

        var snapshots = new List<List<(string Key, double[] Disc)>>();
        var lastRecordedPos = new Dictionary<int, Vec2>();

        var now = 0.0;
        for (var step = 0; step < steps; step++)
        {
            demand.Step(now, Dt, field, noEntities);
            now += Dt;

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>(demand.LiveIds.Count);
            foreach (var id in demand.LiveIds)
            {
                var pos = manager.PositionOf(id, now);
                var animTag = manager.AnimTagOf(id, now);
                var kind = animTag == ActivityTimeline.WalkAnimTag ? KindPedestrian : KindPedPaused;

                // Same deterministic lateral-offset overlap fix as BuildOdRouting (finite-differenced
                // heading against the ped's own last RECORDED-frame position). While paused, `pos` is
                // unchanging frame-to-frame, so `heading` naturally decays to ~zero and the disc
                // renders exactly on the centreline while stopped -- a paused ped visibly settling
                // onto its stopping point rather than an offset walking pose freezing mid-stride.
                var heading = lastRecordedPos.TryGetValue(id, out var prev) ? pos - prev : Vec2.Zero;
                lastRecordedPos[id] = pos;

                var rendered = pos;
                if (heading.Abs > 1e-6)
                {
                    var headingUnit = heading.Normalized();
                    var perp = new Vec2(-headingUnit.Y, headingUnit.X);
                    rendered += perp * OdLateralOffsetMeters(id);
                }

                discs.Add(($"ped{id}", new[] { R(rendered.X), R(rendered.Y), 0.3, (double)kind }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        return new ScenePayload(
            "Lively crowd (routed + activity timelines)",
            "The same routed O-D crowd as the plain OD-routing demo, now graduated to LIVE-PROD-1b: "
            + "PedDemand's schedule generator builds each spawn's route as an ActivityTimeline with "
            + "seeded Pause (\"sip\") beats along the way, instead of a bare PathArc. Yellow = paused "
            + "(stopped in place, mid-beat); purple = walking -- both are still low-power, still "
            + "server==IG, still routed across the junction's real sidewalks/crossings/walkingareas.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Dodge / reroute" (docs/PEDESTRIAN-POC-PLAN.md POC-5; docs/PEDESTRIAN-TASKS.md P2-2):
    // two distinct obstacle-avoidance mechanisms sharing one crowd. (1) LOCAL dodge: a bidirectional
    // pedestrian stream on a straight sidewalk swerves around a static box obstacle purely via
    // OrcaCrowd's own reciprocal/obstacle avoidance (BoxObstacle.Corners + AddObstacle) -- no
    // rerouting, no navmesh query, just steering around it. (2) STRATEGIC reroute: a second, amber
    // pedestrian pair walks the junction's north crossing back and forth; partway through the clip a
    // blocker box appears over that crossing (BlockerRegistry) and RerouteDriver detects it and
    // recomputes a detour through the junction's walkingarea ring for exactly the affected ped(s) --
    // then the blocker disappears again later. Both mechanisms are the committed, unmodified
    // Sim.Pedestrians controllers (PedRouteController/BlockerRegistry/RerouteDriver), driven here
    // exactly as DynamicBlockerRerouteTests/ObstacleDodgeTests drive them.
    // ---------------------------------------------------------------------------------------
    // Two watchable framings of the SAME simulation (both mechanisms run in one crowd): one camera
    // zoomed on the sidewalk box so the local dodge arc is clearly visible, one on the north crossing
    // so the strategic detour is. Framing them together in one 100 m-tall scene made each ~3 px on a
    // phone -- imperceptible -- so they get separate, tight views of the same underlying run.
    internal static ScenePayload BuildObstacleDodge(string scenarioDir) => BuildDodgeReroute(scenarioDir, "dodge");

    internal static ScenePayload BuildCrossingReroute(string scenarioDir) => BuildDodgeReroute(scenarioDir, "reroute");

    private static ScenePayload BuildDodgeReroute(string scenarioDir, string focus)
    {
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var walkableAddPath = Path.Combine(scenarioDir, "walkable.add.xml");
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var pedNetwork = PedNetworkParser.Load(netPath, walkableAddPath);
        network = WithCrossings(network, pedNetwork, netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var space = new SumoWalkableSpace(polygons);
        var nav = new SumoNavMesh(polygons, space);

        const double MaxSpeed = 1.3;
        const double PedRadius = 0.3;
        const double ArriveRadius = 0.3;
        const double Dt = 0.1;
        const int Decimate = 3;
        const int steps = 700; // 70s simulated, decimated to ~233 recorded frames
        const int BlockerAppearsAtStep = 150; // t=15s
        const int BlockerClearsAtStep = 450;  // t=45s

        var crowd = new OrcaCrowd();
        var controller = new PedRouteController(crowd, new WaypointFollower(), ArriveRadius);
        var registry = new BlockerRegistry(polygons);
        var driver = new RerouteDriver(crowd, nav, polygons, debounceSeconds: 0.5, commitDwellSeconds: 1.0);

        // ----- (1) local dodge: a static box obstacle on a straight, junction-free sidewalk stretch --
        // same "nc_0" sidewalk (x in [111.6,113.6]) DynamicBlockerRerouteTests' FarNorthA/FarNorthB use,
        // well clear of the junction, so this half of the demo never touches the reroute machinery.
        var dodgeSidewalk = pedNetwork.Sidewalks.FirstOrDefault(
            s => s.Shape.Any(p => Math.Abs(p.X - 112.6) < 0.5 && p.Y is > 195 and < 220));
        var dodgeHalfWidth = Math.Min(1.0, (dodgeSidewalk?.Width ?? 2.0) / 2.0);

        // The box sits on the RIGHT of the ~2 m sidewalk, its left edge just past the centreline, so a
        // centreline pedestrian MUST leave the line to pass -- but it leaves a clear ~0.8 m corridor on
        // the LEFT. (A box centred on the sidewalk spanning most of its width leaves under one
        // ped-diameter each side, so ORCA has no feasible gap and peds get shoved straight through it --
        // the failure mode ObstacleDodgeTests.BuildPedPath is written to avoid.)
        const double SidewalkCenterX = 112.6;
        const double BoxY = 207.0;
        var boxLeftX = SidewalkCenterX - (0.15 * dodgeHalfWidth);   // just left of the centreline
        var boxRightX = SidewalkCenterX + dodgeHalfWidth;           // out to the sidewalk's right edge
        var obstacleCenter = new Vec2((boxLeftX + boxRightX) / 2.0, BoxY);
        var obstacleHalfX = (boxRightX - boxLeftX) / 2.0;
        var obstacleCorners = BoxObstacle.Corners(obstacleCenter, obstacleHalfX, 1.3, angleRadians: 0.0);
        crowd.AddObstacle(obstacleCorners);

        // Dodge peds are ROUTED through the left corridor via a mid-path waypoint set clear of the box
        // (the ObstacleDodgeTests recipe): the waypoint supplies the persistent lateral INTENT, and
        // ORCA supplies the fine clearance from the box and from oncoming peds. A bare straight goal
        // behind the box gives no lateral pull, so the ped never arcs -- it just walks into the box.
        var dodgeLaneX = boxLeftX - PedRadius - 0.15;  // corridor centre, > ped radius clear of the box
        var dodgeWaypoint = new Vec2(dodgeLaneX, BoxY);
        var dodgeOrigin = new Vec2(SidewalkCenterX, 189.0);  // both ends well clear of the box's y-band
        var dodgeDest = new Vec2(SidewalkCenterX, 225.0);
        Vec2[] DodgePath(Vec2 from, Vec2 to) => new[] { from, dodgeWaypoint, to };

        // Three peds heading up, three down, all spawned OUTSIDE the box's y-band [~205.7,208.3] so none
        // starts inside the obstacle; staggered along the approach so they form a loose bidirectional
        // single file that has to thread past the box.
        var dodgeSpawns = new[]
        {
            (Y: 189.0, Up: true), (Y: 195.0, Up: true), (Y: 201.0, Up: true),
            (Y: 225.0, Up: false), (Y: 219.0, Up: false), (Y: 213.0, Up: false),
        };
        var dodgePeds = new List<(OrcaHandle Handle, Vec2 Origin, Vec2 Dest, bool GoingToDest)>();
        foreach (var (startY, up) in dodgeSpawns)
        {
            var start = new Vec2(SidewalkCenterX, startY);
            var goal = up ? dodgeDest : dodgeOrigin;
            var handle = crowd.Add(start, PedRadius, MaxSpeed, goal: start);
            controller.AddRoute(handle, DodgePath(start, goal), MaxSpeed);
            dodgePeds.Add((handle, dodgeOrigin, dodgeDest, up));
        }

        // ----- (2) strategic reroute: two peds cycling the north crossing back and forth -----
        var westNorthArm = new Vec2(112.6, 140.0);
        var eastNorthArm = new Vec2(127.4, 140.0);
        var northCrossing = polygons.Single(p => p.Kind == BakedPolygonKind.Crossing && p.Id == ":c_c0_0");

        var reroutePeds = new List<(OrcaHandle Handle, Vec2 Goal, bool GoingEast)>();
        void SpawnReroutePed(Vec2 start, Vec2 goal, bool goingEast)
        {
            var path = nav.FindPath(start, goal) ?? new[] { start, goal };
            var handle = crowd.Add(start, PedRadius, MaxSpeed, goal: path[0]);
            controller.AddRoute(handle, path, MaxSpeed);
            driver.RegisterPed(handle, goal, path);
            reroutePeds.Add((handle, goal, goingEast));
        }

        SpawnReroutePed(westNorthArm, eastNorthArm, goingEast: true);
        SpawnReroutePed(eastNorthArm, westNorthArm, goingEast: false);

        // Box covering ~60% of the north crossing's own bounding-box width / 90% of its height (mirrors
        // DynamicBlockerRerouteTests.BoxCoveringPolygon -- proven to occlude just that one crossing,
        // not its neighbours across the corner).
        Vec2[] BoxCoveringPolygon(BakedPolygon polygon)
        {
            var minX = polygon.Vertices.Min(v => v.X);
            var maxX = polygon.Vertices.Max(v => v.X);
            var minY = polygon.Vertices.Min(v => v.Y);
            var maxY = polygon.Vertices.Max(v => v.Y);
            var center = new Vec2((minX + maxX) / 2.0, (minY + maxY) / 2.0);
            var halfX = (maxX - minX) / 2.0 * 0.6;
            var halfY = (maxY - minY) / 2.0 * 0.9;
            return BoxObstacle.Corners(center, halfX, halfY, angleRadians: 0.0);
        }

        var blockerVerts = BoxCoveringPolygon(northCrossing);
        Vec2 BlockerCenter()
        {
            double sx = 0, sy = 0;
            foreach (var v in blockerVerts) { sx += v.X; sy += v.Y; }
            return new Vec2(sx / blockerVerts.Length, sy / blockerVerts.Length);
        }

        BlockerId? blockerId = null;
        var lastEventCount = 0;
        var snapshots = new List<List<(string Key, double[] Disc)>>();

        var time = 0.0;
        for (var step = 0; step < steps; step++)
        {
            controller.Update();
            crowd.Step(Dt);
            time += Dt;

            if (step == BlockerAppearsAtStep)
            {
                blockerId = registry.Register(blockerVerts);
            }
            else if (step == BlockerClearsAtStep && blockerId is { IsValid: true })
            {
                registry.Unregister(blockerId.Value);
                blockerId = null;
            }

            driver.Update(time, registry.BlockedPolygons());
            if (driver.Events.Count > lastEventCount)
            {
                for (var e = lastEventCount; e < driver.Events.Count; e++)
                {
                    var evt = driver.Events[e];
                    controller.AddRoute(evt.Ped, driver.PathOf(evt.Ped), MaxSpeed);
                }

                lastEventCount = driver.Events.Count;
            }

            // Respawn each dodge ped once it reaches its current end -- a sustained bidirectional flow.
            for (var i = 0; i < dodgePeds.Count; i++)
            {
                var (handle, origin, dest, goingToDest) = dodgePeds[i];
                if (controller.IsRouteComplete(handle))
                {
                    var newGoal = goingToDest ? origin : dest;
                    controller.AddRoute(handle, DodgePath(crowd.Position(handle), newGoal), MaxSpeed);
                    dodgePeds[i] = (handle, origin, dest, !goingToDest);
                }
            }

            // Respawn each reroute-demo ped once it arrives -- re-querying the path under whatever is
            // CURRENTLY effectively blocked, so a spawn during the blocked window gets the detour and a
            // spawn after it clears gets the direct path again.
            for (var i = 0; i < reroutePeds.Count; i++)
            {
                var (handle, goal, goingEast) = reroutePeds[i];
                if ((crowd.Position(handle) - goal).Abs <= ArriveRadius && controller.IsRouteComplete(handle))
                {
                    var newGoal = goingEast ? westNorthArm : eastNorthArm;
                    var newPath = nav.FindPath(goal, newGoal, driver.EffectiveBlockedPolygons) ?? new[] { goal, newGoal };
                    controller.AddRoute(handle, newPath, MaxSpeed);
                    driver.RegisterPed(handle, newGoal, newPath);
                    reroutePeds[i] = (handle, newGoal, !goingEast);
                }
            }

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>();
            foreach (var (handle, _, _, _) in dodgePeds)
            {
                var p = crowd.Position(handle);
                discs.Add(($"dodge{handle.Index}", new[] { R(p.X), R(p.Y), PedRadius, (double)KindPedestrian }));
            }

            foreach (var (handle, _, _) in reroutePeds)
            {
                var p = crowd.Position(handle);
                discs.Add(($"reroute{handle.Index}", new[] { R(p.X), R(p.Y), PedRadius, (double)KindReroutingPedestrian }));
            }

            discs.Add(("obstacle", new[]
            {
                R(obstacleCenter.X), R(obstacleCenter.Y), 1.0, (double)KindObstacle, 0.0, 0.0, 1.0, R(obstacleHalfX),
            }));

            if (blockerId is { IsValid: true })
            {
                var bc = BlockerCenter();
                var halfX = (blockerVerts.Max(v => v.X) - blockerVerts.Min(v => v.X)) / 2.0;
                var halfY = (blockerVerts.Max(v => v.Y) - blockerVerts.Min(v => v.Y)) / 2.0;
                discs.Add(("blocker", new[] { R(bc.X), R(bc.Y), 1.0, (double)KindObstacle, 0.0, 0.0, R(halfY), R(halfX) }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        Console.WriteLine($"dodge-reroute: {driver.Events.Count} reroute event(s) recorded over the clip");

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        Track(dodgeOrigin.X, dodgeOrigin.Y);
        Track(dodgeDest.X, dodgeDest.Y);

        // Two tight framings of the same run (see BuildObstacleDodge/BuildCrossingReroute). Each view is
        // small enough (tens of metres, not the whole ~100 m cross) that the behaviour is clearly
        // visible on a phone; peds outside the frame are simply off-canvas.
        var (name, desc, view) = focus == "reroute"
            ? ("Crossing reroute",
                "Strategic reroute (not local avoidance): an amber pedestrian pair walks the junction's "
                + "north crossing back and forth. Partway through, a blocker box appears over the crossing "
                + "(grey); RerouteDriver detects it and recomputes a detour through the walkingarea ring "
                + "for exactly the affected pedestrian -- watch them swing wide around the block -- then "
                + "the blocker clears and the direct crossing resumes. Driven end-to-end through "
                + "PedRouteController + BlockerRegistry + RerouteDriver.",
                new double[] { 106.0, 116.0, 134.0, 147.0 })
            : ("Obstacle dodge",
                "Local avoidance: a bidirectional pedestrian stream (purple) walks a sidewalk with a "
                + "static box obstacle (grey) blocking the centreline. Each ped is routed through the "
                + "clear corridor beside the box by a single off-line waypoint, and OrcaCrowd's own "
                + "reciprocal/obstacle avoidance keeps it clear of the box and of oncoming peds -- so the "
                + "stream arcs around the box and re-forms past it. No navmesh reroute; just steering. "
                + "Driven through OrcaCrowd (BoxObstacle + AddObstacle) + PedRouteController.",
                new double[] { R(obstacleCenter.X - 5.5), R(BoxY - 15.0), R(obstacleCenter.X + 5.5), R(BoxY + 15.0) });

        return new ScenePayload(
            name,
            desc,
            view,
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Parking" (docs/PEDESTRIAN-POC-PLAN.md POC-6, success condition 3;
    // docs/PEDESTRIAN-DESIGN.md §6 "Parking lots"): LotCoupling's car<->pedestrian mutual-avoidance
    // bridge inside a parking lot -- a non-holonomic car maneuvers in and out of a parking slot among
    // static parked-car boxes while pedestrians weave across the drive aisle (some paths deliberately
    // pass straight through a parked column, so ORCA's local avoidance is what actually steers them
    // around); a walker boards the car once it parks, and a fresh pedestrian alights once the car later
    // returns to the lot exit. Pure crowds (no road network) -- the lot's own coordinates are taken
    // from the POC-0 fixture's walkable.add.xml "parkinglot" polygon and entry/exit POIs, but
    // LotCoupling itself has no navmesh dependency, so this is not rendered against the SUMO net.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildParking()
    {
        const double MaxSpeed = 1.3;
        const double PedRadius = 0.3;
        const double ArriveRadius = 0.5;
        const double CarMaxSpeed = 2.2;
        const double Dt = 0.15;
        const int Decimate = 2;
        const int steps = 900; // 135s simulated, decimated to 450 recorded frames
        const int HoldSteps = 30; // ~4.5s dwell at each stop before the car's next leg

        var lot = new LotCoupling();

        // Layout, from walkable.add.xml's "parkinglot" polygon (x in [-180,-130], y in [-80,-20]) and
        // its entry/exit POIs at (-130,-30) / (-130,-70): an east-west drive aisle with a parked row on
        // each side, nose-in (box length along Y, BoxObstacle.Corners' angleRadians=+-pi/2 rotates its
        // local "length" +X axis to the world Y axis).
        var entry = new Vec2(-130, -30);
        var exit = new Vec2(-130, -70);
        const double halfLen = 2.2, halfWid = 1.0;
        var slotXs = new[] { -173.0, -163.0, -153.0, -143.0, -133.0 };
        const double northRowY = -28.0, southRowY = -72.0;
        const int emptySlotIndex = 2; // x = -153 -- the car's target slot, left unoccupied

        for (var i = 0; i < slotXs.Length; i++)
        {
            if (i != emptySlotIndex)
            {
                lot.AddParkedCarBox(new Vec2(slotXs[i], northRowY), halfLen, halfWid, Math.PI / 2.0);
            }

            lot.AddParkedCarBox(new Vec2(slotXs[i], southRowY), halfLen, halfWid, Math.PI / 2.0);
        }

        var emptySlot = new Vec2(slotXs[emptySlotIndex], northRowY);

        var carId = lot.AddCar(entry, emptySlot, CarMaxSpeed);
        var carPhase = 0; // 0: entry->slot, 1: slot->exit, 2: exit->entry (loop)
        var phaseArrivedAtStep = -1;

        // ----- weaving pedestrians: some columns deliberately line up with a parked box, so the
        // straight-line path genuinely enters it and ORCA's own avoidance must steer around it. -----
        var weavers = new List<(OrcaHandle Handle, Vec2 A, Vec2 B, bool GoingToB)>();
        var weaverXs = new[] { -173.0, -163.0, -153.0, -143.0, -133.0 };
        for (var i = 0; i < weaverXs.Length; i++)
        {
            var x = weaverXs[i];
            var a = new Vec2(x, -20.0);
            var b = new Vec2(x, -80.0);
            var goingToB = i % 2 == 0;
            var start = goingToB ? a : b;
            var handle = lot.AddPedestrian(start, PedRadius, MaxSpeed, goal: goingToB ? b : a);
            weavers.Add((handle, a, b, goingToB));
        }

        // ----- boarding walker: approaches the empty slot's near edge, "boards" (despawns) once the
        // car has parked there and the walker has reached it. -----
        var boardApproach = new Vec2(emptySlot.X, northRowY + 4.2);
        var boardWalker = lot.AddPedestrian(new Vec2(entry.X, entry.Y + 5.0), PedRadius, MaxSpeed, boardApproach);
        var boarded = false;

        OrcaHandle? alightWalker = null;
        var alighted = false;

        var snapshots = new List<List<(string Key, double[] Disc)>>();

        for (var step = 0; step < steps; step++)
        {
            lot.Step(Dt);

            // Car phase machine: advance once arrived + a short hold has elapsed.
            if (lot.CarArrived(carId))
            {
                if (phaseArrivedAtStep < 0)
                {
                    phaseArrivedAtStep = step;
                }

                if (step - phaseArrivedAtStep >= HoldSteps)
                {
                    switch (carPhase)
                    {
                        case 0:
                            lot.SetCarGoal(carId, exit);
                            carPhase = 1;
                            break;
                        case 1:
                            if (!alighted)
                            {
                                alightWalker = lot.AddPedestrian(
                                    new Vec2(exit.X + 1.5, exit.Y), PedRadius, MaxSpeed, new Vec2(exit.X + 10.0, exit.Y - 3.0));
                                alighted = true;
                            }

                            lot.SetCarGoal(carId, entry);
                            carPhase = 2;
                            break;
                        default:
                            lot.SetCarGoal(carId, emptySlot);
                            carPhase = 0;
                            break;
                    }

                    phaseArrivedAtStep = -1;
                }
            }
            else
            {
                phaseArrivedAtStep = -1;
            }

            // Boarding: despawn the walker once it reaches the slot's near edge AND the car has parked.
            if (!boarded && carPhase == 0 && lot.CarArrived(carId)
                && (lot.PedPosition(boardWalker) - boardApproach).Abs < 0.6)
            {
                lot.Peds.Remove(boardWalker);
                boarded = true;
            }

            // Weavers: retarget once each reaches its current end -- a sustained back-and-forth flow.
            for (var i = 0; i < weavers.Count; i++)
            {
                var (handle, a, b, goingToB) = weavers[i];
                var target = goingToB ? b : a;
                if ((lot.PedPosition(handle) - target).Abs < ArriveRadius)
                {
                    var newTarget = goingToB ? a : b;
                    lot.SetPedGoal(handle, newTarget);
                    weavers[i] = (handle, a, b, !goingToB);
                }
            }

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>();
            foreach (var box in lot.ParkedCarFootprints)
            {
                var cx = (box[0].X + box[2].X) / 2.0;
                var cy = (box[0].Y + box[2].Y) / 2.0;
                discs.Add(($"box{cx:F1}_{cy:F1}", new[] { R(cx), R(cy), 1.0, (double)KindObstacle, 90.0, 0.0, halfLen, halfWid }));
            }

            {
                var cp = lot.CarPosition(carId);
                var cls = lot.CarClass(carId);
                var headingDeg = lot.CarHeading(carId) * 180.0 / Math.PI;
                discs.Add(("car", new[]
                {
                    R(cp.X), R(cp.Y), 1.0, (double)KindParkingCar, R(headingDeg), 0.0, cls.Length * 0.5, cls.Width * 0.5,
                }));
            }

            if (!boarded)
            {
                var p = lot.PedPosition(boardWalker);
                discs.Add(("boardWalker", new[] { R(p.X), R(p.Y), PedRadius, (double)KindPedestrian }));
            }

            if (alightWalker is { } aw)
            {
                var p = lot.PedPosition(aw);
                discs.Add(("alightWalker", new[] { R(p.X), R(p.Y), PedRadius, (double)KindPedestrian }));
            }

            foreach (var (handle, _, _, _) in weavers)
            {
                var p = lot.PedPosition(handle);
                discs.Add(($"weave{handle.Index}", new[] { R(p.X), R(p.Y), PedRadius, (double)KindPedestrian }));
            }

            snapshots.Add(discs);
        }

        var frames = new List<FramePayload>(snapshots.Count);
        var noVehicles = Array.Empty<double[]?>();
        foreach (var _ in snapshots)
        {
            frames.Add(new FramePayload(noVehicles, Array.Empty<double[]?>()));
        }

        AssignStableDiscSlots(frames, snapshots);

        var labels = new string[14];
        labels[2] = "pedestrian";
        labels[12] = "parked car";
        labels[13] = "maneuvering car";

        return new ScenePayload(
            "Parking",
            "LotCoupling's car<->pedestrian mutual-avoidance bridge: a non-holonomic car (blue) "
            + "maneuvers into an empty parking slot among static parked cars (grey) and later pulls back "
            + "out to the lot exit, while pedestrians (purple) weave across the drive aisle -- some "
            + "paths run straight through a parked column, so it is genuinely ORCA's own local avoidance "
            + "steering them around, not a scripted detour. One walker boards the car once it parks; "
            + "another alights once the car returns to the exit. Driven end-to-end through LotCoupling.",
            // Generous margin beyond the lot's own [-180,-130]x[-80,-20] footprint: a non-holonomic
            // car occasionally overshoots a sharp goal change before curving back (LotCoupling's own
            // documented pursuit behaviour), so the view needs slack, not just the lot's bounding box.
            new double[] { -195, -95, -110, -5 },
            null,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray(),
            labels);
    }

    // ---------------------------------------------------------------------------------------
    // Scene D -- "Open-space crowd (counter-flow)": two head-on pedestrian streams that must
    // interleave. Pure OrcaCrowd, NO network. Self-organising lanes emerge.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildCounterFlow()
    {
        const int perStream = 7;
        var crowd = new OrcaCrowd(2 * perStream) { SymmetryBreak = 0.05 };
        var kinds = new List<int>();

        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(-14, i * 1.5), 0.5, 1.5, goal: new Vec2(14, i * 1.5));           // -> stream A
            kinds.Add(KindStreamA);
        }

        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(14, i * 1.5 + 0.75), 0.5, 1.5, goal: new Vec2(-14, i * 1.5 + 0.75)); // <- stream B
            kinds.Add(KindStreamB);
        }

        var frames = RunCrowd(crowd, kinds, dt: 0.25, maxSteps: 260, arrivalEps: 0.4, decimate: 2);

        return new ScenePayload(
            "Open-space crowd (counter-flow)",
            "Two pedestrian streams walk head-on through each other (no lanes). ORCA reciprocal avoidance "
            + "self-organises passing lanes. Blue = rightbound stream, red = leftbound stream.",
            new double[] { -15, -2, 15, 12 },
            null,
            new double[] { 0, 0 },
            0.25 * 2, // decimated: 2 sim steps per stored frame
            frames);
    }

    // ---------------------------------------------------------------------------------------
    // Scene E -- "Open-space crowd (90 crossing)": two streams crossing at right angles. Pure
    // OrcaCrowd, NO network.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildCrossing()
    {
        const int perStream = 6;
        var crowd = new OrcaCrowd(2 * perStream) { SymmetryBreak = 0.05 };
        var kinds = new List<int>();

        for (var i = 0; i < perStream; i++)
        {
            crowd.Add(new Vec2(-14, i * 1.4 - 3.5), 0.5, 1.5, goal: new Vec2(14, i * 1.4 - 3.5)); // -> stream A (W->E)
            kinds.Add(KindStreamA);
            crowd.Add(new Vec2(i * 1.4 - 3.5, -14), 0.5, 1.5, goal: new Vec2(i * 1.4 - 3.5, 14)); // ^ stream B (S->N)
            kinds.Add(KindStreamB);
        }

        var frames = RunCrowd(crowd, kinds, dt: 0.25, maxSteps: 260, arrivalEps: 0.5, decimate: 2);

        return new ScenePayload(
            "Open-space crowd (90 crossing)",
            "Two pedestrian streams cross at right angles in open space (no lanes). ORCA negotiates the "
            + "conflict at the centre. Blue = W->E stream, red = S->N stream.",
            new double[] { -15, -15, 15, 15 },
            null,
            new double[] { 0, 0 },
            0.25 * 2, // decimated
            frames);
    }

    // ---------------------------------------------------------------------------------------
    // Scene F -- "Uncontrolled junction (dense)": the Egypt/India-style chaotic intersection. Four
    // dense streams of mixed-size movers (cars/tuk-tuks/bikes) approach a shared crossroads from
    // N/S/E/W with NO lanes and NO signals, and negotiate through the packed centre by reciprocal
    // avoidance. Pure OrcaCrowd at scale, using the Q2/Q3 machinery so a heavily-loaded junction keeps
    // FLOWING instead of gridlocking: nearest-k neighbour culling (MaxNeighbours) + a deterministic
    // symmetry break so the 4-way conflict resolves, removal-on-arrival so movers that clear the box
    // leave (draining the queue), and the shared spatial hash (UseSpatialHash) so ~90 agents stay cheap.
    // Movers are wave-spawned each few steps to sustain the congestion. Colour = travel axis
    // (blue E<->W, red N<->S); sizes cycle across vehicle classes.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildDenseJunction()
    {
        const double half = 24.0;    // field half-extent (world units ~= metres)
        const double roadHalf = 7.0; // half-width of each carriageway (14 m road)

        var crowd = new OrcaCrowd(400)
        {
            SymmetryBreak = 0.12,     // break the 4-way symmetry so the centre doesn't lock
            MaxNeighbours = 10,       // nearest-k cull (Q2) -- desymmetrises + bounds work
            RemoveOnArrival = true,   // movers that clear the far side leave (drains the queue)
            ArrivalRadius = 1.0,
            UseSpatialHash = true,    // Q3 -- keep the crowd cheap at scale
            NeighbourDist = 8.0,
            TimeHorizonObst = 8.0,    // react to the building walls early (more clearance margin)
        };

        // Confine movers to the CROSS-shaped carriageway: the four corner "buildings" are STATIC ORCA
        // obstacles (the Q1 obstacle-line port), so a mover cannot cut a corner into a building -- it is
        // bounded within the road along the arms and only mixes freely in the central box. Wound CCW
        // (agents stay outside), extended well past the exit goals so the arm walls confine movers the
        // whole way in and out (no lateral drift even off-screen).
        const double b = roadHalf, outer = 60.0;
        void Building(double x0, double y0, double x1, double y1) =>
            crowd.AddObstacle(new[] { new Vec2(x0, y0), new Vec2(x1, y0), new Vec2(x1, y1), new Vec2(x0, y1) });
        Building(-outer, b, -b, outer);       // NW
        Building(b, b, outer, outer);         // NE
        Building(-outer, -outer, -b, -b);     // SW
        Building(b, -outer, outer, -b);       // SE

        var kinds = new List<int>();      // colour: 0 = E<->W (blue), 1 = N<->S (red)
        var shapes = new List<int>();     // 0 = rectangle (car / tuk-tuk), 1 = hexagon (motorcycle)
        var headings = new List<double>();
        var rows = new[] { 1.6, 3.6, 5.6 };  // 3 sub-streams per direction (keep-right side)
        var spawn = 0;
        var vt = 0;

        // Deterministic vehicle mix -- mostly motorcycles + cars, some tuk-tuks (the Egypt/India blend).
        static (double Radius, int Shape) VType(int i) => (i % 6) switch
        {
            0 => (0.95, 0),   // car (rectangle)
            1 => (0.48, 1),   // motorcycle (hexagon)
            2 => (0.72, 0),   // tuk-tuk (rectangle)
            3 => (0.50, 1),   // motorcycle
            4 => (0.90, 0),   // car
            _ => (0.52, 1),   // motorcycle
        };

        void TryAdd(Vec2 start, Vec2 goal, int kind)
        {
            if (crowd.Count >= 340)
            {
                return;
            }

            var (r, shape) = VType(vt++);
            crowd.Add(start, r, maxSpeed: 2.6, goal: goal);
            kinds.Add(kind);
            shapes.Add(shape);
            var dir = goal - start;
            headings.Add(Math.Atan2(dir.Y, dir.X) * 180.0 / Math.PI);   // initial facing = toward goal
        }

        void SpawnWave()
        {
            var row = rows[(spawn / 4) % rows.Length];
            TryAdd(new Vec2(-half, -row), new Vec2(half + 12, -row), 0);  // W->E (right side = -y)
            TryAdd(new Vec2(half, row), new Vec2(-half - 12, row), 0);    // E->W (+y)
            TryAdd(new Vec2(row, -half), new Vec2(row, half + 12), 1);    // S->N (+x)
            TryAdd(new Vec2(-row, half), new Vec2(-row, -half - 12), 1);  // N->S (-x)
            spawn++;
        }

        var snapshots = new List<double[][]>();
        void Snapshot()
        {
            var d = new double[crowd.Count][];
            for (var i = 0; i < crowd.Count; i++)
            {
                var p = crowd.Position(i);
                var vel = crowd.Velocity(i);
                if (vel.Abs > 0.3)   // update facing only when actually moving (hold it steady while jammed)
                {
                    headings[i] = Math.Atan2(vel.Y, vel.X) * 180.0 / Math.PI;
                }

                d[i] = new[]
                {
                    R(p.X), R(p.Y), R(crowd.Radius(i)), (double)kinds[i], R(headings[i]), (double)shapes[i],
                };
            }

            snapshots.Add(d);
        }

        const int steps = 340;
        const int stopSpawnAt = 300;   // keep the box busy nearly the whole clip, then let it drain
        for (var step = 0; step < steps; step++)
        {
            if (step < stopSpawnAt && step % 5 == 0)
            {
                SpawnWave();
            }

            crowd.Step(0.25);
            Snapshot();
        }

        // Keep only frames where the junction is actually populated (trim any empty lead-in / drained
        // tail), so the clip is wall-to-wall traffic. "On screen" = within the drawn field.
        int OnScreen(double[][] d)
        {
            var n = 0;
            foreach (var a in d)
            {
                if (Math.Abs(a[0]) <= half && Math.Abs(a[1]) <= half)
                {
                    n++;
                }
            }

            return n;
        }

        var lastBusy = snapshots.Count - 1;
        while (lastBusy > 0 && OnScreen(snapshots[lastBusy]) < 6)
        {
            lastBusy--;
        }

        var frames = new List<FramePayload>();
        var noVehicles = Array.Empty<double[]?>();
        for (var i = 0; i <= lastBusy; i += 2)   // decimate ~2x
        {
            frames.Add(new FramePayload(noVehicles, snapshots[i]));
        }

        // Two wide crossing carriageways as the drawn network (hand-built -- no SUMO net needed).
        var hRoad = new LanePayload("EW", "EW", 0, 2 * roadHalf, new double[] { -half - 6, 0, half + 6, 0 });
        var vRoad = new LanePayload("NS", "NS", 0, 2 * roadHalf, new double[] { 0, -half - 6, 0, half + 6 });
        var network = new NetworkPayload(
            new[] { hRoad, vRoad }, Array.Empty<JunctionPayload>(),
            Array.Empty<TlLogicPayload>(), Array.Empty<SignalHeadPayload>());

        return new ScenePayload(
            "Uncontrolled junction (dense)",
            "No lanes, no signals: dense mixed traffic (cars, tuk-tuks, bikes) streams into a shared "
            + "crossroads from all four directions and negotiates through the packed centre by reciprocal "
            + "avoidance -- the laneless model at scale. Blue = east<->west, red = north<->south.",
            new double[] { -half, -half, half, half },
            network,
            new double[] { 0, 0 },
            0.25 * 2,
            frames.ToArray());
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Indian junction (shaped, soft priority)": the believable mixed-traffic module
    // (docs/INDIA-TRAFFIC.md). Unlike the dense-junction disc scene, vehicles here are ANISOTROPIC
    // oriented footprints (buses long, motorcycles compact) avoided by the shaped velocity obstacle
    // (ShapedVoSolver), and priority is SOFT: the east-west "main road" runs an assertive fleet
    // (buses/cars) while the north-south cross road runs a yielding one (motorcycles/auto-rickshaws),
    // so a dominant stream emerges with no signals. Driven by MixedTrafficCrowd at export time.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildIndianJunction()
    {
        const double half = 32.0;    // drawn field half-extent
        const double roadHalf = 9.0; // half-width of each 18 m carriageway

        var crowd = new MixedTrafficCrowd(400)
        {
            Nonholonomic = true,     // car-like steering: no reverse, bounded turn, no pivot-in-place
            SafetyMargin = 0.5,      // NH-ORCA tracking-error margin: real bodies stay apart despite steer lag
            SymmetryBreak = 0.05,    // gentle jitter to break 4-way standoffs at the centre
            MaxNeighbours = 10,      // RVO2-ish; fewer simultaneous constraints -> rarer infeasibility
            RemoveOnArrival = true,
            ArrivalRadius = 3.0,
            NeighbourDist = 18.0,
            TimeHorizon = 3.0,       // longer look-ahead: bounded steering must start avoiding EARLY
            // CreepSpeed left at 0: strict kinematics (a stopped vehicle never turns). At this inflow
            // the junction FLOWS without a creep hack; pushing inflow higher gridlocks the greedy
            // solve (vehicles freeze mid-turn) -- so density is metered to the flowing regime below.
        };

        // Four corner buildings confine the movers to the cross-shaped carriageway (their road-facing
        // faces sit at +/-roadHalf). Tiled into local wall segments by AddBlock -> robust confinement.
        const double b = roadHalf, outer = 80.0, thick = 1.5;
        crowd.AddBlock(-outer, b, -b, outer, thick);       // NW
        crowd.AddBlock(b, b, outer, outer, thick);         // NE
        crowd.AddBlock(-outer, -outer, -b, -b, thick);     // SW
        crowd.AddBlock(b, -outer, outer, -b, thick);       // SE

        // Assertive main-road fleet (E<->W) vs yielding cross-road fleet (N<->S). The class carries
        // the shape + assertiveness; the mix is what makes the E<->W stream dominate.
        static VehicleClass MainClass(int i) => (i % 6) switch
        {
            0 => VehicleClass.Bus,          // one bus per six -- present but not so dense it jams
            1 => VehicleClass.Car,
            2 => VehicleClass.Car,
            3 => VehicleClass.AutoRickshaw,
            4 => VehicleClass.Car,
            _ => VehicleClass.Car,
        };

        static VehicleClass CrossClass(int i) => (i % 6) switch
        {
            0 => VehicleClass.Motorcycle,
            1 => VehicleClass.AutoRickshaw,
            2 => VehicleClass.Motorcycle,
            3 => VehicleClass.Car,
            4 => VehicleClass.Motorcycle,
            _ => VehicleClass.AutoRickshaw,
        };

        var headings = new List<double>();
        // For a TURNING vehicle: its final exit goal, held until it reaches the junction box, then
        // swapped in (a two-leg route enter->centre->exit) so it arcs THROUGH the junction instead of
        // cutting a straight diagonal. null for a straight-through vehicle.
        var pendingExit = new List<Vec2?>();
        var mainRows = new[] { -5.0, -1.8 };   // W->E keeps to y<0 ; E->W to y>0 (loose, Indian-style)
        var mainRows2 = new[] { 1.8, 5.0 };
        var crossRows = new[] { 1.8, 5.0 };    // S->N keeps to x>0 ; N->S to x<0
        var crossRows2 = new[] { -5.0, -1.8 };
        var spawn = 0;
        var vtMain = 0;
        var vtCross = 0;

        // Exit-arm lane points (well past the field edge so vehicles drive fully off before despawn).
        var exN = new Vec2(2.5, half + 16);
        var exS = new Vec2(-2.5, -(half + 16));
        var exE = new Vec2(half + 16, -2.5);
        var exW = new Vec2(-(half + 16), 2.5);
        var centre = new Vec2(0, 0);

        // Cap the number of SIMULTANEOUSLY-active movers. Kept in the FLOWING regime: too high and the
        // greedy non-holonomic solve gridlocks (vehicles freeze mid-turn and never clear), which reads
        // as "stuck despite gaps". Metered here so the junction keeps draining.
        const int liveCap = 70;
        int Live()
        {
            var n = 0;
            for (var i = 0; i < crowd.Count; i++)
            {
                if (crowd.IsActive(i))
                {
                    n++;
                }
            }

            return n;
        }

        // A vehicle only enters when its spawn point is clear -- a real approach queues at the mouth
        // rather than materialising on top of the car ahead. This matters under non-holonomic steering
        // (vehicles accelerate gently and linger near the entry), and it naturally meters inflow to
        // what the junction can absorb instead of forcing an overlap.
        bool SpawnClear(Vec2 p)
        {
            for (var i = 0; i < crowd.Count; i++)
            {
                if (crowd.IsActive(i) && (crowd.Position(i) - p).Abs < 7.0)
                {
                    return false;
                }
            }

            return true;
        }

        // A movement is straight-through (exit == null) or a turn routed via the junction centre
        // (initial goal = centre, exit swapped in on arrival at the box). `main` picks the fleet mix.
        void TryAdd(bool main, Vec2 start, Vec2 goal, Vec2? exit)
        {
            if (crowd.Count >= 380 || Live() >= liveCap || !SpawnClear(start))
            {
                return;
            }

            var cls = main ? MainClass(vtMain++) : CrossClass(vtCross++);
            var dir = goal - start;
            crowd.Add(cls, start, goal, maxSpeedOverride: cls.MaxSpeed * 0.4);
            headings.Add(Math.Atan2(dir.Y, dir.X) * 180.0 / Math.PI);
            pendingExit.Add(exit);
        }

        // Pick a movement for an arm: mostly straight, some left, some right (deterministic cycle).
        // Returns (initialGoal, exit): straight keeps the row across; a turn heads to the centre first.
        (Vec2 goal, Vec2? exit) Movement(int phase, Vec2 straightGoal, Vec2 leftExit, Vec2 rightExit) =>
            (phase % 4) switch
            {
                2 => (centre, leftExit),    // ~25% turn left through the junction
                3 => (centre, rightExit),   // ~25% turn right
                _ => (straightGoal, (Vec2?)null),   // ~50% straight through
            };

        void SpawnWave()
        {
            var m1 = mainRows[(spawn / 2) % mainRows.Length];
            var m2 = mainRows2[(spawn / 2) % mainRows2.Length];
            var c1 = crossRows[(spawn / 2) % crossRows.Length];
            var c2 = crossRows2[(spawn / 2) % crossRows2.Length];

            // W->E : straight E, left N, right S.
            var we = Movement(spawn, new Vec2(half + 16, m1), exN, exS);
            TryAdd(true, new Vec2(-half, m1), we.goal, we.exit);
            // E->W : straight W, left S, right N.
            var ew = Movement(spawn + 1, new Vec2(-half - 16, m2), exS, exN);
            TryAdd(true, new Vec2(half, m2), ew.goal, ew.exit);
            // S->N : straight N, left W, right E.
            var sn = Movement(spawn + 2, new Vec2(c1, half + 16), exW, exE);
            TryAdd(false, new Vec2(c1, -half), sn.goal, sn.exit);
            // N->S : straight S, left E, right W.
            var ns = Movement(spawn + 3, new Vec2(c2, -half - 16), exE, exW);
            TryAdd(false, new Vec2(c2, half), ns.goal, ns.exit);
            spawn++;
        }

        var snapshots = new List<double[][]>();
        void Snapshot()
        {
            var d = new double[crowd.Count][];
            for (var i = 0; i < crowd.Count; i++)
            {
                var p = crowd.Position(i);
                headings[i] = crowd.Heading(i) * 180.0 / Math.PI;   // crowd tracks heading (held when slow)
                var cls = crowd.Class(i);
                d[i] = new[]
                {
                    R(p.X), R(p.Y), R(cls.Shape().CircumRadius()), (double)cls.VizColorKind,
                    R(headings[i]), (double)cls.VizShape, R(cls.Length * 0.5), R(cls.Width * 0.5),
                };
            }

            snapshots.Add(d);
        }

        const int steps = 360;
        const int stopSpawnAt = 315;
        for (var step = 0; step < steps; step++)
        {
            if (step < stopSpawnAt && step % 3 == 0)
            {
                SpawnWave();
            }

            // A turning vehicle heads for the junction centre first; once inside the box, swap in its
            // exit goal so it arcs out along the destination arm (rather than being removed at the
            // centre or cutting a straight diagonal). Done before Step so this step already steers out.
            for (var i = 0; i < crowd.Count; i++)
            {
                if (crowd.IsActive(i) && pendingExit[i] is { } exit
                    && crowd.Position(i).Abs < roadHalf)
                {
                    crowd.SetGoal(i, exit);
                    pendingExit[i] = null;
                }
            }

            crowd.Step(0.2);

            // Despawn any mover that has left the field -- normal exits past the arm goals, and the
            // rare bus a dense jam shoves backward out of the domain (holonomic limit). Keeps the clip
            // clean and frees the space (a departed vehicle should not keep constraining the box).
            const double bound = half + 12.0;
            for (var i = 0; i < crowd.Count; i++)
            {
                if (crowd.IsActive(i))
                {
                    var p = crowd.Position(i);
                    if (Math.Abs(p.X) > bound || Math.Abs(p.Y) > bound)
                    {
                        crowd.Deactivate(i);
                    }
                }
            }

            Snapshot();
        }

        int OnScreen(double[][] d)
        {
            var n = 0;
            foreach (var a in d)
            {
                if (Math.Abs(a[0]) <= half && Math.Abs(a[1]) <= half)
                {
                    n++;
                }
            }

            return n;
        }

        var lastBusy = snapshots.Count - 1;
        while (lastBusy > 0 && OnScreen(snapshots[lastBusy]) < 8)
        {
            lastBusy--;
        }

        var frames = new List<FramePayload>();
        var noVehicles = Array.Empty<double[]?>();
        for (var i = 0; i <= lastBusy; i += 2)
        {
            frames.Add(new FramePayload(noVehicles, snapshots[i]));
        }

        var hRoad = new LanePayload("EW", "EW", 0, 2 * roadHalf, new double[] { -half - 6, 0, half + 6, 0 });
        var vRoad = new LanePayload("NS", "NS", 0, 2 * roadHalf, new double[] { 0, -half - 6, 0, half + 6 });
        var network = new NetworkPayload(
            new[] { hRoad, vRoad }, Array.Empty<JunctionPayload>(),
            Array.Empty<TlLogicPayload>(), Array.Empty<SignalHeadPayload>());

        return new ScenePayload(
            "Indian junction (shaped, soft priority)",
            "Believable mixed traffic: SHAPED vehicles (long buses, compact motorcycles) negotiate an "
            + "uncontrolled crossroads by anisotropic reciprocal avoidance, with CAR-LIKE kinematics -- "
            + "non-holonomic steering (no reverse, no pivot-in-place) and a kinematic-bicycle body that "
            + "pivots about its rear axle, so a turning bus swings its front wide while the rear tracks "
            + "in (real off-tracking). About half the traffic turns left/right through the junction. "
            + "Priority is SOFT -- assertive buses/cars hold their line while motorcycles/auto-rickshaws "
            + "weave and yield. No lanes, no signals. Blue=car, pink=motorcycle, purple=auto-rickshaw, "
            + "amber=bus.",
            new double[] { -half, -half, half, half },
            network,
            new double[] { 0, 0 },
            0.2 * 2,
            frames.ToArray(),
            new[] { "car", "motorcycle", "auto-rickshaw", "bus" });
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Panic evacuation" (docs/PANIC-EVAC-DESIGN.md S6 / docs/EVAC-DEMO-TLS.md): the
    // organized-traffic-to-panic transition on the richer, SIGNALIZED evac grid. Driven end-to-end
    // through EvacTlsScenario.Build (the TLS sibling of the fixture EvacSpineTests/EvacPhase*Tests
    // use), so the viz exercises the exact public seams the tests do -- Engine for the driving core
    // (including the net's <tlLogic> TLS programs, built by Engine.InitializeLoaded on the
    // LoadNetwork path same as LoadScenario), EvacDirector for the external panic/pedestrian layer.
    // The original (untouched) EvacGridScenario/evac-grid demo stays pinned by its own tests; this
    // scene is the denser, signalized successor used for the opening viz.
    // ---------------------------------------------------------------------------------------
    // internal (not private): Program.cs's --evac-organic stats printer counts discs by kind too.
    internal const int KindFleeing = 4;    // #38bdf8 -- fleeing pedestrian
    internal const int KindEscaped = 5;    // #34d399 -- escaped pedestrian
    internal const int KindAbandoned = 6;  // #b91c1c -- abandoned car
    internal const int KindPushingCar = 8; // #fb923c -- car pushing onto the shoulder (Phase 3)

    internal static ScenePayload BuildEvacGrid(string repoRoot)
    {
        var netPath = Path.Combine(repoRoot, "scenarios", "evac-grid-tls", "net.net.xml");
        var incidentSpec = Sim.Evac.EvacTlsScenario.DefaultIncident;
        var (engine, director, _) = Sim.Evac.EvacTlsScenario.Build(netPath, incidentSpec);
        var network = BuildNetwork(NetworkParser.Parse(netPath));

        var slotByHandle = new Dictionary<uint, int>();
        var frames = new List<FramePayload>();
        var discsKeyedPerFrame = new List<List<(string Key, double[] Disc)>>();

        for (var step = 0; step < 240; step++)
        {
            director.Tick();

            var handles = engine.VehicleHandles;
            var px = engine.PosX;
            var py = engine.PosY;
            var pa = engine.Angle;

            for (var i = 0; i < handles.Length; i++)
            {
                if (!slotByHandle.ContainsKey(handles[i].Index))
                {
                    slotByHandle[handles[i].Index] = slotByHandle.Count;
                }
            }

            var v = new double[slotByHandle.Count][];
            for (var i = 0; i < handles.Length; i++)
            {
                var slot = slotByHandle[handles[i].Index];
                v[slot] = new[] { R(px[i]), R(py[i]), R(pa[i]), R(director.Fear(handles[i])) };
            }

            // Stable per-entity disc keys -> fixed slots via AssignStableDiscSlots (see BuildEvacOrganic
            // for why: index-matched interpolation must lerp like-with-like, not smear entities).
            var discs = new List<(string, double[])>();
            for (var i = 0; i < director.PedestrianCount; i++)
            {
                var p = director.PedestrianPosition(i);
                var kind = director.PedestrianEscaped(i) ? KindEscaped : KindFleeing;
                discs.Add(($"p{i}", new[] { R(p.X), R(p.Y), 0.6, (double)kind }));
            }

            for (var i = 0; i < director.AbandonedCarCount; i++)
            {
                var c = director.AbandonedCar(i);
                discs.Add(($"a{i}", new[] { R(c.X), R(c.Y), R(c.Radius), (double)KindAbandoned }));
            }

            foreach (var (handle, px_, py_, headingRad) in director.ActivePushersWithHandle())
            {
                var headingDeg = headingRad * 180.0 / Math.PI;   // math radians (0=+x, CCW) -> degrees CCW from +x, exactly what drawShaped wants
                discs.Add(($"push{handle.Index}",
                    new[] { R(px_), R(py_), 2.5, (double)KindPushingCar, R(headingDeg), 0.0, 2.5, 0.9 }));
                //         x       y       radius kind                head          shape halfLen halfWid   (rectangle 5.0x1.8)
            }

            frames.Add(new FramePayload(v, Array.Empty<double[]?>()));
            discsKeyedPerFrame.Add(discs);
        }

        AssignStableDiscSlots(frames, discsKeyedPerFrame);

        NormalizeVehicleSlots(frames, slotByHandle.Count);

        var nm = director.NavMesh;
        var boundary = new[]
        {
            R(nm.MinX), R(nm.MinY), R(nm.MinX), R(nm.MaxY), R(nm.MaxX), R(nm.MaxY), R(nm.MaxX), R(nm.MinY),
        };

        var cfg = Sim.Evac.EvacTlsScenario.DefaultConfig();
        var incident = new[]
        {
            R(incidentSpec.X), R(incidentSpec.Y), R(incidentSpec.Radius), R(incidentSpec.StartTime), R(cfg.SafeRadius),
        };

        var labels = new string[9];
        labels[4] = "fleeing pedestrian";
        labels[5] = "escaped pedestrian";
        labels[6] = "abandoned car";
        labels[8] = "abandoning car (shoulder)";

        return new ScenePayload(
            "Panic evacuation",
            "Organized grid traffic; a central incident (amber zone) triggers panic; cars flee to exits "
            + "and jam; boxed-in drivers abandon cars (dark-red discs) and flee on foot (cyan, turning "
            + "green past the safe radius); the dashed rectangle is the known-world hard edge. Driving "
            + "core = SUMO port; pedestrians + panic = external Sim.Evac layer.",
            new double[] { R(nm.MinX), R(nm.MinY), R(nm.MaxX), R(nm.MaxY) },
            network,
            new double[] { 5.0, 1.8 },
            Sim.Evac.EvacTlsScenario.StepLength,
            frames.ToArray(),
            labels,
            incident,
            boundary);
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Panic evacuation (organic town)" (PANIC-EVAC-PHASE5-DESIGN.md §4, T4.1): the Tier-1
    // realistic-town evac. Mirrors BuildEvacGrid's structure exactly (fear-tinted vehicle frames,
    // pedestrian discs, abandoned cars, pushers, hard-edge boundary, incident overlay) but drives the
    // organic-town fixture (EvacOrganicScenario.Build) instead of the hand-built grid: LoadScenario's
    // own demand inserts vehicles under the director's Tick() loop and the director AUTO-attaches to
    // whatever drives into the working region -- there is no explicit handle list to iterate (unlike
    // EvacGrid's manual spawn-and-Track), so vehicle frames are built the same way BuildEvacGrid does,
    // straight off engine.VehicleHandles each tick. NOT part of --bundle (a few-MB payload at this
    // frame/vehicle count would bloat the showcase); emitted standalone via `--evac-organic`.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildEvacOrganic(string repoRoot)
    {
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_bench", "city-organic-L2");
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var cfgPath = Path.Combine(scenarioDir, "config.sumocfg");

        var (engine, director) = Sim.Evac.EvacOrganicScenario.Build(repoRoot);
        var network = BuildNetwork(NetworkParser.Parse(netPath));
        var scenarioConfig = ScenarioConfigParser.Parse(cfgPath);
        var stepLength = scenarioConfig.StepLength > 0 ? scenarioConfig.StepLength : 1.0;

        var slotByHandle = new Dictionary<uint, int>();
        var frames = new List<FramePayload>();
        var discsKeyedPerFrame = new List<List<(string Key, double[] Disc)>>();

        for (var step = 0; step < 300; step++)
        {
            director.Tick();

            var handles = engine.VehicleHandles;
            var px = engine.PosX;
            var py = engine.PosY;
            var pa = engine.Angle;

            for (var i = 0; i < handles.Length; i++)
            {
                if (!slotByHandle.ContainsKey(handles[i].Index))
                {
                    slotByHandle[handles[i].Index] = slotByHandle.Count;
                }
            }

            var v = new double[slotByHandle.Count][];
            for (var i = 0; i < handles.Length; i++)
            {
                var slot = slotByHandle[handles[i].Index];
                v[slot] = new[] { R(px[i]), R(py[i]), R(pa[i]), R(director.Fear(handles[i])) };
            }

            // Each disc carries a STABLE identity key (pedestrian index / abandoned-car index / pusher
            // handle) so BuildStableDiscFrames can pin it to a fixed slot across frames. Without that,
            // rebuilding the disc list fresh each frame shuffles which entity lands at index i as counts
            // grow, and the front-end's index-matched interpolation smears one entity's position into
            // another's -- the "discs flying between abandoned-car positions" artifact.
            var discs = new List<(string, double[])>();
            for (var i = 0; i < director.PedestrianCount; i++)
            {
                var p = director.PedestrianPosition(i);
                var kind = director.PedestrianEscaped(i) ? KindEscaped : KindFleeing;
                discs.Add(($"p{i}", new[] { R(p.X), R(p.Y), 0.6, (double)kind }));
            }

            for (var i = 0; i < director.AbandonedCarCount; i++)
            {
                var c = director.AbandonedCar(i);
                discs.Add(($"a{i}", new[] { R(c.X), R(c.Y), R(c.Radius), (double)KindAbandoned }));
            }

            foreach (var (handle, px_, py_, headingRad) in director.ActivePushersWithHandle())
            {
                var headingDeg = headingRad * 180.0 / Math.PI;
                discs.Add(($"push{handle.Index}",
                    new[] { R(px_), R(py_), 2.5, (double)KindPushingCar, R(headingDeg), 0.0, 2.5, 0.9 }));
            }

            frames.Add(new FramePayload(v, Array.Empty<double[]?>()));
            discsKeyedPerFrame.Add(discs);
        }

        NormalizeVehicleSlots(frames, slotByHandle.Count);
        AssignStableDiscSlots(frames, discsKeyedPerFrame);

        // Camera = the NET's own geometric extent (lanes + junctions) -- NOT the navmesh, which pads
        // outward by VicinityWidth and is drawn separately below as the hard-edge boundary rectangle.
        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        var nm = director.NavMesh;
        var boundary = new[]
        {
            R(nm.MinX), R(nm.MinY), R(nm.MinX), R(nm.MaxY), R(nm.MaxX), R(nm.MaxY), R(nm.MaxX), R(nm.MinY),
        };

        var incidentSpec = director.Incident;
        var cfg = Sim.Evac.EvacOrganicScenario.DefaultConfig();
        var incident = new[]
        {
            R(incidentSpec.X), R(incidentSpec.Y), R(incidentSpec.Radius), R(incidentSpec.StartTime), R(cfg.SafeRadius),
        };

        var labels = new string[9];
        labels[4] = "fleeing pedestrian";
        labels[5] = "escaped pedestrian";
        labels[6] = "abandoned car";
        labels[8] = "abandoning car (shoulder)";

        return new ScenePayload(
            "Panic evacuation (organic town)",
            "Realistic organic town (274 junctions, 1186 edges, ~406 peak concurrent traffic); a local "
            + "incident at a busy signalized interior junction triggers panic ONLY in the auto-tracked "
            + "working region around it -- distant traffic stays pure parity lane traffic, untouched by "
            + "the evac layer (the load-bearing Phase-5 locality property). Cars flee toward town-edge "
            + "exits and jam; boxed-in drivers nose onto the shoulder, then abandon (dark-red discs) and "
            + "flee on foot (cyan, turning green past the safe radius); the dashed rectangle is the "
            + "known-world hard edge. Driving core = SUMO port; pedestrians + panic = external Sim.Evac "
            + "layer.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 5.0, 1.8 },
            stepLength,
            frames.ToArray(),
            labels,
            incident,
            boundary);
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Panic evacuation (10k city)" (PANIC-EVAC-PHASE5-TIER2-DESIGN.md §3b/§3c, TASKS T2.6):
    // the Tier-2 CITY-SCALE evac. Drives EvacCityScenario (scenarios/_bench/city-15000, ~13-17k peak
    // concurrent) exactly the way BuildEvacOrganic drives EvacOrganicScenario -- fear-tinted vehicle
    // frames, stable-keyed pedestrian/abandoned-car/pusher discs, hard-edge boundary, incident overlay
    // -- but a 10k-car x N-frame payload at FULL per-vehicle detail would be tens of MB (design §3c),
    // so this builder adds a REGION-CROP compaction pass after the raw run: every vehicle the evac
    // layer ever auto-tracked into the working region (director.IsTracked) keeps full per-frame detail
    // (it's the thing the demo is actually about); every other ("distant") vehicle is either dropped
    // entirely or kept as a decimated sample (every k-th by first-seen slot order) so the city still
    // reads as populated in the background. The exact counts are ALWAYS logged to the console (no
    // silent truncation) -- see the "region-crop:" line below.
    // ---------------------------------------------------------------------------------------
    internal const int DistantSampleEvery = 20;   // keep 1 in 20 distant (never-tracked) vehicles

    // PANIC-EVAC-PHASE5-TIER2-DESIGN.md §3c: layered payload bounding -- region-crop (above) alone
    // still left an 18.6 MB payload at ticks=400 (measured), mostly the per-frame null padding of
    // thousands of ever-spawned pedestrian disc columns growing with frame count. Frame decimation
    // (keep every Nth tick's snapshot; the director still TICKS every step, so the simulation itself
    // is unaffected -- only which snapshots are RECORDED is thinned) cuts every per-frame array
    // linearly and brings the payload back under the stated budget. Logged like every other drop.
    internal const int FrameDecimate = 3;   // keep 1 recorded frame per 3 ticks

    internal static ScenePayload BuildEvacCity(string repoRoot, int ticks = 400, int frameDecimate = FrameDecimate)
    {
        var scenarioDir = Path.Combine(repoRoot, "scenarios", "_bench", "city-15000");
        var netPath = Path.Combine(scenarioDir, "net.net.xml");
        var cfgPath = Path.Combine(scenarioDir, "config.sumocfg");

        var (engine, director) = Sim.Evac.EvacCityScenario.Build(repoRoot);
        var network = BuildNetwork(NetworkParser.Parse(netPath));
        var scenarioConfig = ScenarioConfigParser.Parse(cfgPath);
        var stepLength = scenarioConfig.StepLength > 0 ? scenarioConfig.StepLength : 1.0;

        var slotByHandle = new Dictionary<uint, int>();
        var everTracked = new HashSet<uint>();
        var frames = new List<FramePayload>();
        var discsKeyedPerFrame = new List<List<(string Key, double[] Disc)>>();
        var recordedFrameCount = 0;

        for (var step = 0; step < ticks; step++)
        {
            director.Tick();

            var handles = engine.VehicleHandles;
            var px = engine.PosX;
            var py = engine.PosY;
            var pa = engine.Angle;

            // Tracking bookkeeping runs EVERY tick regardless of frame decimation -- a vehicle that
            // passes through the working region between two recorded frames must still count as
            // tracked (region-crop must not silently miss it just because its frame wasn't kept).
            for (var i = 0; i < handles.Length; i++)
            {
                if (!slotByHandle.ContainsKey(handles[i].Index))
                {
                    slotByHandle[handles[i].Index] = slotByHandle.Count;
                }

                if (director.IsTracked(handles[i]))
                {
                    everTracked.Add(handles[i].Index);
                }
            }

            if ((step + 1) % frameDecimate != 0)
            {
                continue;   // thinned tick: simulation advanced, but no frame recorded
            }

            recordedFrameCount++;

            var v = new double[slotByHandle.Count][];
            for (var i = 0; i < handles.Length; i++)
            {
                var slot = slotByHandle[handles[i].Index];
                v[slot] = new[] { R(px[i]), R(py[i]), R(pa[i]), R(director.Fear(handles[i])) };
            }

            var discs = new List<(string, double[])>();
            for (var i = 0; i < director.PedestrianCount; i++)
            {
                var p = director.PedestrianPosition(i);
                var kind = director.PedestrianEscaped(i) ? KindEscaped : KindFleeing;
                discs.Add(($"p{i}", new[] { R(p.X), R(p.Y), 0.6, (double)kind }));
            }

            for (var i = 0; i < director.AbandonedCarCount; i++)
            {
                var c = director.AbandonedCar(i);
                discs.Add(($"a{i}", new[] { R(c.X), R(c.Y), R(c.Radius), (double)KindAbandoned }));
            }

            foreach (var (handle, px_, py_, headingRad) in director.ActivePushersWithHandle())
            {
                var headingDeg = headingRad * 180.0 / Math.PI;
                discs.Add(($"push{handle.Index}",
                    new[] { R(px_), R(py_), 2.5, (double)KindPushingCar, R(headingDeg), 0.0, 2.5, 0.9 }));
            }

            frames.Add(new FramePayload(v, Array.Empty<double[]?>()));
            discsKeyedPerFrame.Add(discs);
        }

        NormalizeVehicleSlots(frames, slotByHandle.Count);
        AssignStableDiscSlots(frames, discsKeyedPerFrame);

        // ----- region-crop compaction (§3c): keep every tracked vehicle's column; keep a decimated
        // sample of distant (never-tracked) columns; drop the rest. Slots kept in original first-seen
        // order so the mapping stays deterministic. -----
        var totalSlots = slotByHandle.Count;
        var handleBySlot = new uint[totalSlots];
        foreach (var (handleIndex, slot) in slotByHandle)
        {
            handleBySlot[slot] = handleIndex;
        }

        var keptSlots = new List<int>();
        var distantOrdinal = 0;
        var sampledDistantCount = 0;
        for (var slot = 0; slot < totalSlots; slot++)
        {
            if (everTracked.Contains(handleBySlot[slot]))
            {
                keptSlots.Add(slot);
                continue;
            }

            var keepSample = distantOrdinal % DistantSampleEvery == 0;
            distantOrdinal++;
            if (keepSample)
            {
                keptSlots.Add(slot);
                sampledDistantCount++;
            }
        }

        var droppedCount = totalSlots - keptSlots.Count;
        Console.WriteLine(
            $"region-crop: {everTracked.Count} in-region + {sampledDistantCount}/{totalSlots - everTracked.Count} " +
            $"sampled distant (every {DistantSampleEvery}th); dropped {droppedCount} distant vehicles " +
            $"(kept {keptSlots.Count}/{totalSlots} total vehicle columns)");
        Console.WriteLine(
            $"frame-decimation: kept 1 of every {frameDecimate} ticks -> {recordedFrameCount} recorded frames " +
            $"of {ticks} simulated ticks (dropped {ticks - recordedFrameCount} intermediate frames; dt scaled " +
            $"{stepLength:F1}s -> {stepLength * frameDecimate:F1}s between recorded frames)");

        for (var f = 0; f < frames.Count; f++)
        {
            var oldV = frames[f].V;
            var newV = new double[keptSlots.Count][];
            for (var newSlot = 0; newSlot < keptSlots.Count; newSlot++)
            {
                newV[newSlot] = oldV[keptSlots[newSlot]];
            }

            frames[f] = frames[f] with { V = newV };
        }

        double minX = double.PositiveInfinity, minY = double.PositiveInfinity;
        double maxX = double.NegativeInfinity, maxY = double.NegativeInfinity;
        void Track(double x, double y)
        {
            if (x < minX) minX = x;
            if (y < minY) minY = y;
            if (x > maxX) maxX = x;
            if (y > maxY) maxY = y;
        }

        foreach (var lane in network.Lanes)
        {
            for (var p = 0; p < lane.Shape.Length; p += 2) Track(lane.Shape[p], lane.Shape[p + 1]);
        }

        foreach (var j in network.Junctions)
        {
            for (var p = 0; p < j.Shape.Length; p += 2) Track(j.Shape[p], j.Shape[p + 1]);
        }

        var nm = director.NavMesh;
        var boundary = new[]
        {
            R(nm.MinX), R(nm.MinY), R(nm.MinX), R(nm.MaxY), R(nm.MaxX), R(nm.MaxY), R(nm.MaxX), R(nm.MinY),
        };

        var incidentSpec = director.Incident;
        var cfg = Sim.Evac.EvacCityScenario.DefaultConfig();
        var incident = new[]
        {
            R(incidentSpec.X), R(incidentSpec.Y), R(incidentSpec.Radius), R(incidentSpec.StartTime), R(cfg.SafeRadius),
        };

        var labels = new string[9];
        labels[4] = "fleeing pedestrian";
        labels[5] = "escaped pedestrian";
        labels[6] = "abandoned car";
        labels[8] = "abandoning car (shoulder)";

        return new ScenePayload(
            "Panic evacuation (10k city)",
            "A 10k-vehicle-class city (24x24 grid, 1 lane, ~13-17k peak concurrent); a large central "
            + "incident triggers panic ONLY in the auto-tracked working region around it -- distant "
            + "traffic (shown as a thinned sample so the city still reads) stays pure parity lane "
            + "traffic, untouched by the evac layer. Cars flee toward grid-fringe exits and jam; "
            + "boxed-in drivers nose onto the shoulder, then abandon (dark-red discs) and flee on foot "
            + "(cyan, turning green past the safe radius); the dashed rectangle is the known-world hard "
            + "edge. Driving core = SUMO port; pedestrians + panic = external Sim.Evac layer.",
            new double[] { R(minX), R(minY), R(maxX), R(maxY) },
            network,
            new double[] { 5.0, 1.8 },
            stepLength * frameDecimate,
            frames.ToArray(),
            labels,
            incident,
            boundary);
    }

    // ---------------------------------------------------------------------------------------
    // Scene -- "Panic evacuation (district, routed on foot)" (P5-1(B), docs/PEDESTRIAN-TASKS.md;
    // docs/PEDESTRIAN-DESIGN.md §6): the pedestrian-side evac unification onto Sim.Pedestrians. Unlike
    // BuildEvacGrid/BuildEvacOrganic/BuildEvacCity (car-centric EvacDirector, radial FakeNavMesh flee),
    // this scene has NO vehicles at all -- it is driven end-to-end by EvacDistrictDirector, which
    // routes every panicked pedestrian to its NEAREST safe zone ALONG the real sidewalk grid (via
    // PedestrianWorld/SumoNavMesh), not radially through the blocks. Calm (not-yet-panicked) peds
    // render low-power grey; once panicked they are forced high-power (SetForcedHighPower) and render
    // orange, streaming along the streets to a corner. NOT part of --bundle (its own standalone mode,
    // like --evac-organic/--evac-city); emitted via `--evac-district`.
    // ---------------------------------------------------------------------------------------
    internal static ScenePayload BuildEvacDistrict(string repoRoot)
    {
        var netPath = Path.Combine(repoRoot, "scenarios", "_ped", "evac-district", "net.net.xml");
        var pedNetwork = PedNetworkParser.Load(netPath);
        var polygons = WalkablePolygonBaker.Bake(pedNetwork);
        var nav = new SumoNavMesh(polygons, new SumoWalkableSpace(polygons));

        var network = BuildNetwork(NetworkParser.Parse(netPath));
        network = WithCrossings(network, pedNetwork, netPath);

        var safeZones = new[] { new Vec2(0, 0), new Vec2(200, 0), new Vec2(0, 200), new Vec2(200, 200) };
        var incident = new Sim.Evac.Incident(X: 100.0, Y: 100.0, StartTime: 8.0, Radius: 90.0);
        var cfg = new Sim.Evac.EvacDistrictConfig
        {
            PedCount = 36,
            MaxSpeed = 1.3,
            PedRadius = 0.3,
            ArriveRadius = 1.0,
            PanicRadius = 90.0,   // deliberately < the full district span so some peds stay calm throughout
        };

        var director = new Sim.Evac.EvacDistrictDirector(nav, incident, safeZones, cfg);

        const double Dt = 0.5;
        const int Decimate = 2;
        const int Steps = 500; // 250s simulated, decimated to 250 recorded frames

        var frames = new List<FramePayload>();
        var discsKeyedPerFrame = new List<List<(string Key, double[] Disc)>>();

        for (var step = 0; step < Steps; step++)
        {
            director.Step(Dt);

            if (step % Decimate != 0)
            {
                continue;
            }

            var discs = new List<(string, double[])>(cfg.PedCount + safeZones.Length);
            for (var id = 1; id <= director.PedestrianCount; id++)
            {
                if (director.IsEscaped(id))
                {
                    continue;   // arrived at its safe zone -- stop rendering (AssignStableDiscSlots
                                // leaves its slot null from here on, exactly like an FCD arrival).
                }

                var pos = director.PositionOf(id);
                var kind = director.IsPanicked(id) ? KindPedHighPower : KindPedLowPower;
                discs.Add(($"ped{id}", new[] { R(pos.X), R(pos.Y), cfg.PedRadius, (double)kind }));
            }

            for (var i = 0; i < safeZones.Length; i++)
            {
                var z = safeZones[i];
                discs.Add(($"safe{i}", new[] { R(z.X), R(z.Y), 4.0, (double)KindSafeZone }));
            }

            frames.Add(new FramePayload(Array.Empty<double[]?>(), Array.Empty<double[]?>()));
            discsKeyedPerFrame.Add(discs);
        }

        AssignStableDiscSlots(frames, discsKeyedPerFrame);

        const double Pad = 15.0;
        var view = new double[] { -Pad, -Pad, 200.0 + Pad, 200.0 + Pad };

        // No single "safe radius" ring makes sense here (four discrete corner safe zones, not one
        // circle) -- pass 0 so the front-end's dashed safe-radius ring collapses to nothing; the
        // corner safe-zone discs themselves (kind 18, green) carry that meaning instead.
        var incidentOverlay = new[] { R(incident.X), R(incident.Y), R(cfg.PanicRadius), R(incident.StartTime), 0.0 };

        var labels = new string[19];
        labels[9] = "pedestrian (calm / low-power)";
        labels[10] = "pedestrian (panicked / forced high-power)";
        labels[18] = "safe zone (corner)";

        return new ScenePayload(
            "Panic evacuation (district, routed on foot)",
            "A synthetic 3x3 sidewalk district (200x200 m); an incident at the centre panics any "
            + "pedestrian within its radius (red halo). Calm walkers (grey) stroll ambient routes; a "
            + "panicked walker (orange) is forced high-power (reactive full-ORCA) and routed to its "
            + "NEAREST safe-zone corner (green) ALONG the real sidewalk grid -- bending around the "
            + "blocks, never a straight line through them -- via the Sim.Pedestrians PedestrianWorld "
            + "facade (EvacDistrictDirector). Vehicles are not modelled in this scene.",
            view,
            network,
            new double[] { 0, 0 },
            Dt * Decimate,
            frames.ToArray(),
            labels,
            incidentOverlay,
            null);
    }

    // Run a pure crowd to convergence, snapshotting disc positions each step and keeping every
    // `decimate`-th snapshot (the frames are dense, so linear interpolation on the front end is
    // ample). Disc kind is per-agent (its stream), captured at Add time.
    private static FramePayload[] RunCrowd(OrcaCrowd crowd, List<int> kinds, double dt, int maxSteps, double arrivalEps, int decimate)
    {
        var snapshots = new List<double[][]>();

        void Snapshot()
        {
            var d = new double[crowd.Count][];
            for (var i = 0; i < crowd.Count; i++)
            {
                var p = crowd.Position(i);
                d[i] = new[] { R(p.X), R(p.Y), R(crowd.Radius(i)), (double)kinds[i] };
            }

            snapshots.Add(d);
        }

        Snapshot(); // initial state = frame 0
        for (var step = 0; step < maxSteps && !crowd.AllArrived(arrivalEps); step++)
        {
            crowd.Step(dt);
            Snapshot();
        }

        var frames = new List<FramePayload>();
        var noVehicles = Array.Empty<double[]?>();
        for (var i = 0; i < snapshots.Count; i += decimate)
        {
            frames.Add(new FramePayload(noVehicles, snapshots[i]));
        }

        return frames.ToArray();
    }

    // Pad every frame's vehicle array up to `slotCount` with null (absent) slots, so all frames
    // share the same fixed-slot vehicle indexing the front end relies on.
    private static void NormalizeVehicleSlots(List<FramePayload> frames, int slotCount)
    {
        for (var f = 0; f < frames.Count; f++)
        {
            var v = frames[f].V;
            if (v.Length == slotCount)
            {
                continue;
            }

            var grown = new double[slotCount][];
            Array.Copy(v, grown, v.Length);
            frames[f] = frames[f] with { V = grown };
        }
    }

    // Pin each disc entity to a FIXED slot across all frames, keyed by its stable identity, and set
    // frame f's D array from that (absent entities -> null slot). This is the disc analogue of
    // NormalizeVehicleSlots: it guarantees D[i] refers to the SAME entity in every frame, so the front
    // end's index-matched interpolation (interpolatedDiscs) lerps like-with-like instead of smearing a
    // pedestrian into an abandoned car as per-frame counts grow. Slots are assigned in first-seen order
    // (frames in time order, discs in the builder's stable within-frame order), so the mapping is
    // deterministic. Null slots are held/dropped by interpolatedDiscs (mirrors the vehicle path).
    private static void AssignStableDiscSlots(
        List<FramePayload> frames, List<List<(string Key, double[] Disc)>> discsKeyedPerFrame)
    {
        var slotByKey = new Dictionary<string, int>();
        foreach (var frame in discsKeyedPerFrame)
        {
            foreach (var (key, _) in frame)
            {
                if (!slotByKey.ContainsKey(key))
                {
                    slotByKey[key] = slotByKey.Count;
                }
            }
        }

        var slotCount = slotByKey.Count;
        for (var f = 0; f < frames.Count; f++)
        {
            var d = new double[slotCount][];
            foreach (var (key, disc) in discsKeyedPerFrame[f])
            {
                d[slotByKey[key]] = disc;
            }

            frames[f] = frames[f] with { D = d };
        }
    }

    // Deterministic per-ped lateral-offset amplitude for BuildOdRouting's overlap fix, in metres,
    // range approx [-0.5, +0.5). A cheap integer hash (Knuth's multiplicative constant, NOT
    // System.Random -- CLAUDE.md forbids it for determinism) so the same ped id always renders on
    // the same side/offset of its centreline on every run (same bytes on re-run).
    private static double OdLateralOffsetMeters(int id)
    {
        var h = unchecked((uint)id * 2654435761u) >> 8;
        return (h % 1000) / 1000.0 - 0.5;
    }
}
