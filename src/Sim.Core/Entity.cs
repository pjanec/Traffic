namespace Sim.Core;

// D5 (FastDataPlane ECS readiness): FDP's entity handle -- see FastDataPlane
// Docs/architectural-rules.md ("Entity = struct handle (Index:32 + Generation:16). Never
// store raw ints; gate stored handles on World.IsAlive(e)."). `Index` mirrors
// VehicleRuntime.EntityIndex (this vehicle's stable slot); `Generation` is reserved for
// index-recycling and stays 0 for every entity in this rung -- bumping it on recycle/reuse
// is a D7 store-boundary concern (out of scope here; see TASKS.md D5/D7). The engine's side
// tables (lane-sequence pool slices, stop/avoided-edge tables) keep keying by the plain
// `int EntityIndex` (== `Entity.Index`), unchanged by this rung.
public readonly record struct Entity(int Index, int Generation);
