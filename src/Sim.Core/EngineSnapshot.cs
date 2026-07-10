using System.Globalization;
using System.Xml.Linq;
using Sim.Ingest;

namespace Sim.Core;

// W2 (warm-start snapshot): serialize a live, populated simulation to a file and restore it, so a
// run can start from an already-driving network without re-warming. A snapshot IS a frozen
// start-of-step state (the plan/execute boundary), which is why it is a clean, complete capture.
//
// COVERS every active vehicle -- cars and TRAINS identically (RailModel is stateless; a train
// carries no special per-vehicle field, and its traction params come from the immutable
// ResolvedVType restored by re-loading the same rou.xml) -- plus the engine-level rail-crossing
// phase state machine and the clock (_elapsedSteps). Static traffic lights need nothing (pure fn of
// the captured clock). The vehicle TEMPLATES (id, type, route -- including <flow>-expanded ones,
// which re-expand deterministically) come from LoadScenario, so the snapshot stores only DYNAMIC
// state keyed by id and overlays it.
//
// SCOPED OUT (guarded, throws at save time rather than silently mis-restoring -- add in a follow-up):
// actuated-TLS internal state, mid-stop progress, and post-reroute lane sequences / avoided edges.
public sealed partial class Engine
{
    private const string SnapshotRoot = "snapshot";

    // Serialize the current populated state to `path`. Requires a loaded scenario that has been
    // advanced (Run/WarmUp); saving before any step writes an empty (t=Begin) snapshot.
    public void SaveSnapshot(string path)
    {
        if (_network is null || _demand is null || _config is null)
        {
            throw new InvalidOperationException("LoadScenario must be called before SaveSnapshot.");
        }

        // Loud engine-level guard for the scoped-out state machine (mirrors the per-vehicle stop /
        // reroute guards below): an actuated TLS has stateful phase + per-detector occupancy that is
        // NOT clock-derivable, and LoadScenario rebuilds it at its Begin/Reset state, so restoring a
        // mid-run snapshot would silently pin the phase machine at Begin while the clock jumps. Throw
        // rather than mis-restore until actuated-TLS capture is added.
        if (_actuatedLogics.Count > 0)
        {
            throw new NotSupportedException(
                "SaveSnapshot: network has actuated traffic lights; actuated-TLS phase/detector state is not yet captured.");
        }

        var root = new XElement(SnapshotRoot,
            new XAttribute("elapsedSteps", _elapsedSteps.ToString(CultureInfo.InvariantCulture)),
            new XAttribute("time", (_config.Begin + _elapsedSteps * _config.StepLength).ToString("R", CultureInfo.InvariantCulture)));

        // F2b: each probabilistic <flow>'s own insertion RNG (raw SplitMix64 word) + arrival counter,
        // so a restored run CONTINUES the same Bernoulli stream (same future arrivals, same "<id>.<k>"
        // numbering) instead of restarting from the seed. Index-aligned with _demand.ProbabilisticFlows
        // (LoadScenario rebuilds that list identically from the same rou). Absent when no flow exists.
        for (var i = 0; i < _probFlowRng.Length; i++)
        {
            root.Add(new XElement("probFlow",
                new XAttribute("index", i.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("rng", _probFlowRng[i].RawState.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("counter", _probFlowCounter[i].ToString(CultureInfo.InvariantCulture))));
        }

        // F2b: which active vehicles came from a probabilistic flow (their ids are NOT in the demand's
        // static vehicle list -- they were materialized at runtime by GenerateProbabilisticFlows).
        // These have no template in _demand, so their snapshot record is self-contained (it carries
        // the VehicleDef fields + stable EntityIndex) and LoadSnapshot RE-CREATES them before
        // overlaying dynamic state -- see LoadSnapshot's flow-vehicle pre-pass.
        var demandIds = new HashSet<string>(_demand.Vehicles.Select(d => d.Id), StringComparer.Ordinal);

        // Engine-level rail-crossing phase machine (R5), indexed by the crossing order
        // BuildRailCrossingInfo rebuilds identically on the same net -> restore by index.
        for (var c = 0; c < _railCrossingViaLaneHandles.Length; c++)
        {
            root.Add(new XElement("railCrossing",
                new XAttribute("index", c.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("step", _railCrossingStep[c].ToString(CultureInfo.InvariantCulture)),
                new XAttribute("nextSwitch", _railCrossingNextSwitch[c].ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("state", _railCrossingState[c].ToString())));
        }

        foreach (var v in _vehicles)
        {
            var isFlow = !demandIds.Contains(v.Def.Id);

            // A demand vehicle that has not yet departed re-inserts from its own Def in the continued
            // run, so it needs no record. A flow-generated one has NO Def in _demand, so even a
            // not-yet-inserted (queued) one must be captured or it would be lost on restore.
            if (!v.Inserted && !isFlow)
            {
                continue;
            }

            // F2b: a flow-generated vehicle's self-contained VehicleDef fields + stable EntityIndex,
            // written on EVERY flow record (inserted, queued, or arrived) so LoadSnapshot can
            // reconstruct it at the SAME EntityIndex (which keeps future flow arrivals' seeding, and
            // thus determinism, identical to a single continuous run). Empty for demand vehicles.
            XAttribute[] FlowMeta() => isFlow
                ? new[]
                {
                    new XAttribute("flowGenerated", "true"),
                    new XAttribute("entityIndex", v.EntityIndex.ToString(CultureInfo.InvariantCulture)),
                    new XAttribute("typeId", v.Def.TypeId),
                    new XAttribute("routeId", v.Def.RouteId),
                    new XAttribute("depart", v.Def.Depart.ToString("R", CultureInfo.InvariantCulture)),
                    new XAttribute("departPos", v.Def.DepartPos.ToString("R", CultureInfo.InvariantCulture)),
                    new XAttribute("departSpeed", v.Def.DepartSpeed.ToString("R", CultureInfo.InvariantCulture)),
                    new XAttribute("departLane", v.Def.DepartLaneIndex.ToString(CultureInfo.InvariantCulture)),
                }
                : Array.Empty<XAttribute>();

            if (!v.Inserted)
            {
                // Flow-generated but still queued: capture its identity so it re-queues (and inserts
                // at its own depart) in the continued run. No dynamic state yet (never planned).
                root.Add(new XElement("vehicle",
                    new XAttribute("id", v.Def.Id),
                    new XAttribute("inserted", "false"),
                    FlowMeta()));
                continue;
            }

            if (v.Arrived)
            {
                // Already finished its route: capture the lifecycle flag (plus flow identity so a flow
                // vehicle still reoccupies its EntityIndex) so LoadSnapshot does not re-insert it.
                root.Add(new XElement("vehicle",
                    new XAttribute("id", v.Def.Id),
                    new XAttribute("arrived", "true"),
                    FlowMeta()));
                continue;
            }

            // Loud guard: state W2 does not yet capture (rather than silently mis-restore).
            if (GetStops(v) is { Count: > 0 })
            {
                throw new NotSupportedException(
                    $"SaveSnapshot: vehicle '{v.Def.Id}' has scheduled stops; stop progress is not yet captured.");
            }
            if (_avoidedByEntity.TryGetValue(v.EntityIndex, out var avoided) && avoided.Count > 0)
            {
                throw new NotSupportedException(
                    $"SaveSnapshot: vehicle '{v.Def.Id}' has rerouted (avoided edges); reroute state is not yet captured.");
            }

            root.Add(new XElement("vehicle",
                new XAttribute("id", v.Def.Id),
                new XAttribute("arrived", "false"),
                FlowMeta(),
                new XAttribute("laneSeqIndex", v.LaneSeqIndex.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("pos", v.Kinematics.Pos.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("speed", v.Kinematics.Speed.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("latOffset", v.Kinematics.LatOffset.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("rng", v.RngState.RawState.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("keepRightProb", v.KeepRightProbability.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("speedGainProb", v.SpeedGainProbability.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("lookAheadSpeed", v.LookAheadSpeed.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("levelOfService", v.LevelOfService.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("acceleration", v.Acceleration.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("waitingTime", v.WaitingTime.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("lastActionTime", v.LastActionTime.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("accControlMode", v.AccControlMode.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("accLastUpdateTime", v.AccLastUpdateTime.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("caccControlMode", v.CaccControlMode.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("blockedByObstacleSeconds", v.BlockedByObstacleSeconds.ToString("R", CultureInfo.InvariantCulture)),
                new XAttribute("lcTargetHandle", v.LcTargetHandle.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("lcTargetId", v.LcTargetId),
                new XAttribute("lcStepsElapsed", v.LcStepsElapsed.ToString(CultureInfo.InvariantCulture)),
                new XAttribute("lcStepsTotal", v.LcStepsTotal.ToString(CultureInfo.InvariantCulture))));
        }

        new XDocument(root).Save(path);
    }

    // Restore a snapshot written by SaveSnapshot. Requires a prior LoadScenario with the SAME net +
    // rou (so vehicle templates -- including <flow> expansions -- and the rail-crossing index order
    // match). Overlays the dynamic state; the next Run() continues from the restored clock without a
    // timeline reset (because _elapsedSteps becomes > 0).
    public void LoadSnapshot(string path)
    {
        if (_network is null || _demand is null || _config is null)
        {
            throw new InvalidOperationException("LoadScenario must be called before LoadSnapshot.");
        }

        var root = XDocument.Load(path).Root
            ?? throw new InvalidDataException("snapshot file has no root element.");

        _elapsedSteps = int.Parse(
            root.Attribute("elapsedSteps")?.Value ?? "0", CultureInfo.InvariantCulture);

        foreach (var rc in root.Elements("railCrossing"))
        {
            var idx = int.Parse(RequireAttr(rc, "index"), CultureInfo.InvariantCulture);
            if (idx < 0 || idx >= _railCrossingViaLaneHandles.Length)
            {
                throw new InvalidDataException($"snapshot railCrossing index {idx} out of range for this net.");
            }

            _railCrossingStep[idx] = int.Parse(RequireAttr(rc, "step"), CultureInfo.InvariantCulture);
            _railCrossingNextSwitch[idx] = double.Parse(RequireAttr(rc, "nextSwitch"), CultureInfo.InvariantCulture);
            _railCrossingState[idx] = RequireAttr(rc, "state")[0];
        }

        // F2b: restore each probabilistic flow's insertion RNG + arrival counter so the continued run
        // resumes the same Bernoulli stream. Index-aligned with _demand.ProbabilisticFlows (rebuilt by
        // the prior LoadScenario), which also sized/seeded these arrays -- we overwrite that seeding.
        foreach (var pf in root.Elements("probFlow"))
        {
            var idx = int.Parse(RequireAttr(pf, "index"), CultureInfo.InvariantCulture);
            if (idx < 0 || idx >= _probFlowRng.Length)
            {
                throw new InvalidDataException($"snapshot probFlow index {idx} out of range for this demand.");
            }

            _probFlowRng[idx] = new VehicleRng(ulong.Parse(RequireAttr(pf, "rng"), CultureInfo.InvariantCulture));
            _probFlowCounter[idx] = int.Parse(RequireAttr(pf, "counter"), CultureInfo.InvariantCulture);
        }

        // F2b: RE-CREATE the flow-generated vehicles (no _demand template exists for them) BEFORE the
        // dynamic-state overlay, in ascending captured EntityIndex order. LoadScenario has already
        // created the demand vehicles at indices 0..M-1; the flow vehicles reoccupy M.. exactly as in
        // the original run, so their (and future arrivals') per-vehicle seeding stays identical.
        foreach (var fv in root.Elements("vehicle")
                     .Where(e => e.Attribute("flowGenerated")?.Value == "true")
                     .OrderBy(e => int.Parse(RequireAttr(e, "entityIndex"), CultureInfo.InvariantCulture)))
        {
            var expectedIndex = int.Parse(RequireAttr(fv, "entityIndex"), CultureInfo.InvariantCulture);
            if (expectedIndex != _vehicles.Count)
            {
                throw new InvalidDataException(
                    $"snapshot flow vehicle '{RequireAttr(fv, "id")}' EntityIndex {expectedIndex} does not match the reconstruction order (expected {_vehicles.Count}).");
            }

            CreateRuntime(new VehicleDef(
                Id: RequireAttr(fv, "id"),
                TypeId: RequireAttr(fv, "typeId"),
                RouteId: RequireAttr(fv, "routeId"),
                Depart: ParseR(fv, "depart"),
                DepartPos: ParseR(fv, "departPos"),
                DepartSpeed: ParseR(fv, "departSpeed"),
                DepartLaneIndex: int.Parse(RequireAttr(fv, "departLane"), CultureInfo.InvariantCulture)));
        }

        var byId = new Dictionary<string, VehicleRuntime>(_vehicles.Count, StringComparer.Ordinal);
        foreach (var v in _vehicles)
        {
            byId[v.Def.Id] = v;
        }

        foreach (var ve in root.Elements("vehicle"))
        {
            var id = RequireAttr(ve, "id");
            if (!byId.TryGetValue(id, out var v))
            {
                throw new InvalidDataException(
                    $"snapshot vehicle '{id}' is not in the loaded demand (net/rou mismatch?).");
            }

            if (ve.Attribute("arrived")?.Value == "true")
            {
                v.Inserted = true;
                v.Arrived = true;
                continue;
            }

            // F2b: a flow vehicle captured while still queued (never inserted). It was already
            // re-created above with its Def (fresh RngState/Kinematics = its original pre-insertion
            // state), so leave it un-inserted -- the continued run's InsertDepartingVehicles places
            // it at its own depart exactly as the original run would.
            if (ve.Attribute("inserted")?.Value == "false")
            {
                continue;
            }

            RestoreActiveVehicle(v, ve);
        }
    }

    private void RestoreActiveVehicle(VehicleRuntime v, XElement ve)
    {
        // Re-resolve the vehicle's full lane sequence from its route (identical to insertion), then
        // point LaneSeqIndex at the saved progress. LaneId/Handle derive from the sequence entry, so
        // a mid-route vehicle lands on the correct lane. (Reroute would diverge here -- guarded out
        // at save time.)
        var route = _demand!.RoutesById[v.Def.RouteId];
        var (poolSeq, arrivalSeq) = _network!.ResolveLaneSequenceHandlesWithArrival(route.Edges, v.Def.DepartLaneIndex);

        var laneSeqIndex = int.Parse(RequireAttr(ve, "laneSeqIndex"), CultureInfo.InvariantCulture);
        if (laneSeqIndex < 0 || laneSeqIndex >= poolSeq.Length)
        {
            throw new InvalidDataException(
                $"snapshot vehicle '{v.Def.Id}' laneSeqIndex {laneSeqIndex} out of range for its route.");
        }

        v.LaneSeqStart = _laneSeqPool.Count;
        v.LaneSeqLen = poolSeq.Length;
        _laneSeqPool.AddRange(poolSeq);
        _laneSeqArrival.AddRange(arrivalSeq);
        v.LaneSeqIndex = laneSeqIndex;

        v.LaneHandle = poolSeq[laneSeqIndex];
        v.LaneId = _network.LanesByHandle[v.LaneHandle].Id;

        v.Kinematics = new Kinematics
        {
            Pos = ParseR(ve, "pos"),
            Speed = ParseR(ve, "speed"),
            LatOffset = ParseR(ve, "latOffset"),
        };

        v.RngState = new VehicleRng(ulong.Parse(RequireAttr(ve, "rng"), CultureInfo.InvariantCulture));
        v.KeepRightProbability = ParseR(ve, "keepRightProb");
        v.SpeedGainProbability = ParseR(ve, "speedGainProb");
        v.LookAheadSpeed = ParseR(ve, "lookAheadSpeed");
        v.LevelOfService = ParseR(ve, "levelOfService");
        v.Acceleration = ParseR(ve, "acceleration");
        v.WaitingTime = ParseR(ve, "waitingTime");
        v.LastActionTime = ParseR(ve, "lastActionTime");
        v.AccControlMode = int.Parse(RequireAttr(ve, "accControlMode"), CultureInfo.InvariantCulture);
        v.AccLastUpdateTime = ParseR(ve, "accLastUpdateTime");
        v.CaccControlMode = int.Parse(RequireAttr(ve, "caccControlMode"), CultureInfo.InvariantCulture);
        v.BlockedByObstacleSeconds = ParseR(ve, "blockedByObstacleSeconds");
        v.LcTargetHandle = int.Parse(RequireAttr(ve, "lcTargetHandle"), CultureInfo.InvariantCulture);
        v.LcTargetId = ve.Attribute("lcTargetId")?.Value ?? string.Empty;
        v.LcStepsElapsed = int.Parse(RequireAttr(ve, "lcStepsElapsed"), CultureInfo.InvariantCulture);
        v.LcStepsTotal = int.Parse(RequireAttr(ve, "lcStepsTotal"), CultureInfo.InvariantCulture);

        v.Inserted = true;
        v.Arrived = false;
    }

    private static double ParseR(XElement el, string name) =>
        double.Parse(RequireAttr(el, name), CultureInfo.InvariantCulture);

    private static string RequireAttr(XElement el, string name) =>
        el.Attribute(name)?.Value
        ?? throw new InvalidDataException($"snapshot <{el.Name.LocalName}> missing attribute '{name}'.");
}
