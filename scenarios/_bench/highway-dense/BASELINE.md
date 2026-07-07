# highway-dense — D1 benchmark baseline

The many-vehicle workload the **Group-D (FastDataPlane ECS readiness)** refactor is measured
against. NO SUMO golden — this is a *benchmark*, not a parity scenario (it measures the engine's
cost and proves determinism; it does not assert a trajectory). `ParameterCrossCheckTests` skips it
(non-recursive scenario discovery + no `golden.vtype.json`).

## Workload
- 3-lane straight edge `e0`, 5000 m, speed 13.89.
- 420 vehicles, staggered departs (one per lane every 3 s), ~20 % capped at `maxSpeed=8` so the
  lane-change / neighbor-query / reducer hot paths all fire (following, keep-right, speed-gain
  overtakes). `lanechange.duration=0`, `sigma=0`, Euler, `collision.action=none`, seed 42.

## Reproduce
```
dotnet run -c Release --project src/Sim.Bench 500
```
(`Sim.Bench` is a console harness, NOT part of `dotnet test`. `RungD1BenchmarkDeterminismTests`
runs a short slice of this scenario in the offline loop to guard determinism.)

## Baseline — current engine (AoS `VehicleRuntime` class, LINQ + per-step allocations)
Captured on the reference VM (.NET 8, Release, workstation GC), 500 steps:

| metric | value |
|---|---|
| peak concurrent vehicles | 378 |
| veh-steps emitted | 115,141 |
| wall time | 0.389 s |
| throughput | **1284 steps/s** (0.779 ms/step) |
| alloc total | **80.9 MiB** |
| alloc / step | 165.6 KiB |
| alloc / veh-step | **736.5 B** |
| GC gen0/1/2 | 5 / 3 / 0 |
| deterministic (2 runs identical) | **True** |

Absolute numbers are machine-dependent; what matters is the **delta each Group-D rung produces**
against this baseline (re-run the harness and update this table, keeping the old row for history).

## What the numbers say (targets for D2–D8)
- **~736 B allocated per vehicle-step** is the headline: this is the AoS `class` entities +
  `LaneNeighborQuery`'s per-step `Dictionary`/`List` (built twice/step) + the reducer's
  `new List<double>` per vehicle + LINQ iterators + the `TrajectorySet` emit (SortedDictionary +
  a `TrajectoryPoint` per veh-step). FDP's rule is **zero** heap alloc in the `OnUpdate` hot path —
  D4 targets the step-loop allocations, and the emit can move to a reusable buffer.
- **deterministic = True** at 378 concurrent vehicles is the load-bearing invariant: it is what lets
  D8 parallelize the Simulation phase and still get byte-identical output. The offline determinism
  test locks it in so no later rung can silently break it.
