namespace Sim.Core;

// D6 (FastDataPlane ECS readiness -- phased systems over queries). The `Query()` analog: FDP's
// `world.Query().With<A>().With<B>().Build()` returns a queryable, filtered view of entities that
// a system's `foreach` walks with zero allocation. This engine's equivalent recurring filter is
// "an active, on-road vehicle" -- `Inserted && !Arrived` -- which used to be re-typed as an
// inline `if (!v.Inserted || v.Arrived) continue;` guard at the top of every pass's `foreach (var
// v in _vehicles)` loop (PlanMovements, ExecuteMoves, EmitTrajectory, DecideSpeedGainChanges,
// LaneNeighborQuery.Refill, the junction-foe scan, ...). `ActiveVehicles()` makes that filter one
// explicit, reusable query object instead of a copy-pasted guard.
//
// Zero-alloc (FDP's `OnUpdate` rule -- no `new`/boxing/LINQ/`IEnumerable<T>` iterator blocks in
// the hot path): `ActiveVehicleQuery` is a `readonly struct` over the engine's backing
// `List<VehicleRuntime>`; its `GetEnumerator()` returns a hand-written `struct Enumerator`, so
// `foreach (var v in engine.ActiveVehicles())` compiles the same way `foreach` over `List<T>`
// does -- a duck-typed `MoveNext()`/`Current` pattern resolved at compile time, no interface
// dispatch, no heap allocation for the enumerator itself. `VehicleRuntime` is already a reference
// type, so `Current` is a plain reference copy, not a box.
internal readonly struct ActiveVehicleQuery
{
    private readonly List<VehicleRuntime> _vehicles;

    public ActiveVehicleQuery(List<VehicleRuntime> vehicles)
    {
        _vehicles = vehicles;
    }

    public Enumerator GetEnumerator() => new(_vehicles);

    public struct Enumerator
    {
        private readonly List<VehicleRuntime> _vehicles;
        private int _index;

        public Enumerator(List<VehicleRuntime> vehicles)
        {
            _vehicles = vehicles;
            _index = -1;
        }

        public readonly VehicleRuntime Current => _vehicles[_index];

        public bool MoveNext()
        {
            while (true)
            {
                _index++;
                if (_index >= _vehicles.Count)
                {
                    return false;
                }

                var v = _vehicles[_index];
                if (v.Inserted && !v.Arrived)
                {
                    return true;
                }
            }
        }
    }
}
