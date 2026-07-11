namespace Sim.Core;

// D5 (FastDataPlane ECS readiness): an in-house command buffer modeled on FDP's
// `view.GetCommandBuffer()` -> deferred `AddComponent`/`DestroyEntity` surface (see
// FastDataPlane Docs/architectural-rules.md, "Structural changes via command buffer").
// Records structural mutations during a phase; `Flush()` applies them, IN RECORD ORDER, at
// that phase's barrier -- the SAME point each of these mutations already took effect at
// before this rung (CLAUDE.md rule 2/4: preserve calculation order exactly; this generalizes
// the engine's existing seam-4 deferred-mutation discipline into FDP's shape, it does not
// move WHEN anything applies).
//
// This engine has three structural-mutation kinds today (see Engine.cs's UpdateReroutes/
// DecideSpeedGainChanges/ExecuteMoves): a discrete lane-index snap (ChangeLane, the FDP
// "AddComponent"-shaped swap of which lane a vehicle occupies), a route/lane-sequence-slice
// swap (ReplaceRoute, a reroute), and marking a vehicle arrived (Destroy, the FDP
// "DestroyEntity" analog -- no index-recycling/generation-bumping yet, that is D7's store-
// boundary concern).
//
// D4 zero-steady-state-alloc discipline: ONE pre-allocated backing `List<Command>`, `Clear()`
// each `Flush()` (keeps its backing array/capacity -- after the first few steps' worth of
// commands, this list never grows again, so recording+flushing every step allocates nothing
// beyond its own steady-state capacity). `Command` is a plain field struct (no boxing) with a
// `Kind` tag and just enough payload fields to cover all three command kinds.
//
// D7 (FastDataPlane ECS readiness -- the FDP-shaped seam / adapter): implements `ICommandBuffer`
// so `World`/`Engine` can be written against the interface instead of this concrete class (see
// ICommandBuffer.cs's own header comment) -- the four methods below are UNCHANGED, this is a
// pure `: ICommandBuffer` addition, no method body differs.
internal sealed class CommandBuffer : ICommandBuffer
{
    private enum Kind : byte
    {
        ChangeLane,
        StartLaneChangeManeuver,
        ReplaceRoute,
        Destroy,
    }

    private struct Command
    {
        public Kind Kind;
        public VehicleRuntime Vehicle;

        // ChangeLane: IntArg0 = newLaneHandle, StringArg0 = newLaneId.
        // StartLaneChangeManeuver: IntArg0 = targetLaneHandle, StringArg0 = targetLaneId, IntArg1 = totalSteps.
        // ReplaceRoute: IntArg0 = laneSeqStart, IntArg1 = laneSeqLen.
        // Destroy: unused.
        public int IntArg0;
        public int IntArg1;
        public string? StringArg0;
    }

    private readonly List<Command> _commands = new();

    // Domain decomposition: the recording methods below may be called concurrently (region-parallel
    // execute/speed-gain). Guard the shared backing list with a lock -- commands are rare (arrivals /
    // lane changes, a handful per step), so contention is negligible, and Flush applies them
    // order-independently (each command targets a distinct vehicle), so the non-deterministic record
    // ORDER does not affect the result. Uncontended (serial default) the lock is ~free.
    private readonly object _recordLock = new();

    // Discrete lane-index snap (lanechange.duration=0): the ego vehicle's current lane
    // becomes `newLaneHandle`/`newLaneId`. Mirrors the inline `v.LaneId = ...; v.LaneHandle =
    // ...;` pairs this replaces (D2's "keep LaneHandle in lockstep with LaneId" invariant is
    // preserved -- both fields are always recorded/applied together).
    public void ChangeLane(VehicleRuntime v, int newLaneHandle, string newLaneId)
    {
        lock (_recordLock)
        {
            _commands.Add(new Command { Kind = Kind.ChangeLane, Vehicle = v, IntArg0 = newLaneHandle, StringArg0 = newLaneId });
        }
    }

    // C10-i: START a continuous lane-change maneuver (lanechange.duration > 0) instead of the
    // instant snap above. The vehicle KEEPS its current (source) lane label; `Engine.AdvanceLaneChanges`
    // then slides it over `totalSteps` steps and flips the label at the midpoint. Records the
    // maneuver target/duration onto the vehicle's own Lc* fields at flush time.
    public void StartLaneChangeManeuver(VehicleRuntime v, int targetLaneHandle, string targetLaneId, int totalSteps)
    {
        lock (_recordLock)
        {
            _commands.Add(new Command { Kind = Kind.StartLaneChangeManeuver, Vehicle = v, IntArg0 = targetLaneHandle, StringArg0 = targetLaneId, IntArg1 = totalSteps });
        }
    }

    // Route/lane-sequence-slice swap (reroute): repoints the vehicle's `[LaneSeqStart,
    // LaneSeqStart+LaneSeqLen)` slice into Engine's shared `_laneSeqPool` to a NEW slice
    // (already appended to the pool by the caller before recording -- the pool itself is
    // engine-owned, not per-vehicle deferred state). `LaneSeqIndex` always resets to 0 on a
    // reroute (the new slice's first element is the vehicle's current lane, matching
    // UpdateReroutes' existing behavior), so Flush applies that reset too -- it is inherent to
    // "swap the lane-sequence slice", not a separate command kind.
    public void ReplaceRoute(VehicleRuntime v, int laneSeqStart, int laneSeqLen)
    {
        _commands.Add(new Command { Kind = Kind.ReplaceRoute, Vehicle = v, IntArg0 = laneSeqStart, IntArg1 = laneSeqLen });
    }

    // Marks the vehicle arrived (the "DestroyEntity" analog for this engine -- no recycling:
    // the entity's slot/record stays in place, just stops being planned/executed/emitted from
    // the next step onward, exactly like the inline `v.Arrived = true` this replaces).
    public void Destroy(VehicleRuntime v)
    {
        lock (_recordLock)
        {
            _commands.Add(new Command { Kind = Kind.Destroy, Vehicle = v });
        }
    }

    // Applies every recorded command, in record order, then clears the buffer for reuse.
    public void Flush()
    {
        foreach (var cmd in _commands)
        {
            switch (cmd.Kind)
            {
                case Kind.ChangeLane:
                    cmd.Vehicle.LaneId = cmd.StringArg0!;
                    cmd.Vehicle.LaneHandle = cmd.IntArg0;
                    break;

                case Kind.StartLaneChangeManeuver:
                    // Keep the current (source) lane label; record the target + duration. The label
                    // flips mid-maneuver in Engine.AdvanceLaneChanges.
                    cmd.Vehicle.LcTargetHandle = cmd.IntArg0;
                    cmd.Vehicle.LcTargetId = cmd.StringArg0!;
                    cmd.Vehicle.LcStepsTotal = cmd.IntArg1;
                    cmd.Vehicle.LcStepsElapsed = 0;
                    break;

                case Kind.ReplaceRoute:
                    cmd.Vehicle.LaneSeqStart = cmd.IntArg0;
                    cmd.Vehicle.LaneSeqLen = cmd.IntArg1;
                    cmd.Vehicle.LaneSeqIndex = 0;
                    // C4-vii-b: the remaining route changed -> the keep-right stayOnBest memo
                    // (ApplyKeepRightDecision) may be stale even on the same lane; force recompute.
                    cmd.Vehicle.KeepRightStayCacheLane = -1;
                    break;

                case Kind.Destroy:
                    cmd.Vehicle.Arrived = true;
                    break;
            }
        }

        _commands.Clear();
    }
}
