namespace Sim.Core;

// D9 (FastDataPlane ECS readiness -- info/replication export SEAM, READINESS ONLY, TASKS.md
// line ~651). This is the observer seam a later FDP `IDescriptorTranslator`-style consumer
// would implement and attach to `Engine` (via `Engine.AddExportObserver`) to mirror per-frame,
// per-vehicle exportable component state WITHOUT touching the sim -- no `NetworkEntityMap`, no
// `INetworkIdAllocator`, no network transport is wired here or anywhere else in this project;
// this file defines only the in-house callback shape.
//
// `OnVehicleExported` is called once per active vehicle, once per Export-phase frame, from
// `Engine.EmitTrajectory` -- the SAME pass that already builds the frame's `TrajectoryPoint`s,
// so an observer sees exactly the vehicles/frames the committed `TrajectorySet` does, in the
// same order, from the same `VehicleExportSnapshot` value (see that struct's own header
// comment). The snapshot is passed `in` (by reference, read-only) rather than by value or
// through a boxed `object`/`IEnumerable<T>` callback, so notifying a registered observer never
// allocates -- and with ZERO observers registered (the default, unless a caller explicitly
// registers one), `Engine`'s notify loop is an empty `foreach` over an empty list: no virtual
// call, no allocation, byte-identical to the pre-D9 `EmitTrajectory` body.
//
// `OnFrameBegin`/`OnFrameEnd` are optional per-frame bracket hooks (a translator that batches a
// frame's worth of descriptors before flushing them needs to know where a frame starts/ends);
// kept as no-op-by-default interface members (C# 8+ default interface methods) so implementors
// that only care about the per-vehicle callback do not need to write empty overrides, and so the
// interface stays minimal per the briefing.
public interface ISimExportObserver
{
    void OnVehicleExported(in VehicleExportSnapshot snapshot);

    void OnFrameBegin(double time) { }

    void OnFrameEnd(double time) { }
}
