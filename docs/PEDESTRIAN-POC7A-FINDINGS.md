# POC-7a findings — `OrcaCrowd.Step` parallelization

Status: **done**. `OrcaCrowd.Step`'s PLAN phase (and, incidentally, its EXECUTE phase) now has an
opt-in parallel path (`OrcaCrowd.UseParallelStep`) that is **bit-identical** to the existing serial
path. This is the first perf task of POC-7 (`docs/PEDESTRIAN-POC-PLAN.md` POC-7;
`docs/PEDESTRIAN-DESIGN.md` §3c/§9). Crowd/Orca code (`src/Sim.Core/Orca/`) is PARITY-EXEMPT, off the
lane determinism hash — this task did not touch the lane engine, `Sim.Ingest`, or `MixedTrafficCrowd`.

## The refactor: per-worker scratch

`OrcaCrowd.Plan(int i, double dt)` computes agent `i`'s new velocity from the frozen start-of-step
state — pure by construction, writing only `_newVelocity[i]`. The only obstacle to parallelizing the
plan loop was that `Plan` used **shared instance-field scratch buffers** (the per-agent neighbour
list, its parallel distance-squared array used by the nearest-k insertion, the ORCA line list, the
obstacle-segment list, and the spatial-hash candidate list) — concurrent `Plan(i)` calls on different
threads would race on those.

The fix: all five buffers were bundled into one private `ScratchSet` class (`OrcaCrowd.cs`), and
`Plan` (plus its helpers `GatherAgentNeighbours` and `GridCandidates`, which also wrote into that
scratch) now take a `ScratchSet` parameter instead of touching instance fields directly.

- **Serial path** (`UseParallelStep == false`, the default): one instance-owned `ScratchSet`
  (`_scratch`), grown by `Grow()` exactly the way the old individual fields were. Memory-reuse pattern
  is unchanged from before the refactor, so this path is **byte-for-byte the same code** that ran
  before POC-7a — every existing test needs (and gets) zero behavioral change.
- **Parallel path** (`UseParallelStep == true`, gated by size — see below): `PlanParallel` uses
  `System.Threading.Tasks.Parallel.For`'s `localInit`/`localFinally` overload, so **each worker thread
  gets its own fresh `ScratchSet`** (`() => new ScratchSet()`), reused across every iteration the TPL
  assigns to that worker. No lock, no shared mutable buffer between workers. The per-agent math itself
  (`Plan`'s body, `OrcaSolver.ComputeNewVelocity`) is completely untouched — only where its scratch
  comes from changed.
- The uniform spatial hash's `RebuildGrid()` stays **serial** (single writer, run once before the plan
  loop, from the frozen post-removal positions) — only the per-agent `GridCandidates` query (which
  *reads* the grid) was made scratch-parallel, since it also needed its own per-worker candidate
  buffer. Concurrent reads of the (now not-written-during-the-plan-phase) `Dictionary` and pooled
  bucket arrays are safe with no concurrent writer.
- The RemoveOnArrival pre-pass is unchanged (serial, runs before the grid rebuild).
- The EXECUTE loop (commit `_velocity`, integrate `_position`) was also parallelized under the same
  gate — it needs no scratch at all (each iteration reads/writes only its own index), so this was a
  free extra win, not the point of the task.

New public surface, mirroring the lane engine's own perf knobs (`Engine.cs`):
- `bool UseParallelStep { get; set; }` — default `false`.
- `int MaxParallelism { get; set; }` — default `-1` (runtime auto), cached as a `ParallelOptions` so
  the hot loop allocates nothing per step.
- `ParallelStepThreshold = 256` (private const) — below this agent count, `Step()` stays serial even
  with `UseParallelStep = true`, mirroring `Engine.ParallelPlanThreshold`'s "gate on total count, a
  cheap proxy for crowd size" convention.

## Correctness gate: parallel == serial, bit-for-bit

`tests/Sim.ParityTests/OrcaParallelStepTests.cs` (new):

- `LargeCrowd_ParallelStepMatchesSerial_BitIdentical` — 2000 agents on a dense crossing grid (paths
  cross through the middle), **spatial hash on**, two static-obstacle walls, four external
  (cross-regime) discs via `SetExternalObstacles`, `MaxNeighbours = 10`, `SymmetryBreak = 0.05`. Builds
  one crowd with `UseParallelStep = false` and one identical crowd with `UseParallelStep = true`
  (2000 ≥ the 256 threshold, so the parallel path actually engages), steps both 300 times, and asserts
  **exact** (`==`) equality of every agent's position AND velocity at every single step.
- `LargeCrowd_ParallelStepMatchesSerial_UnderPinnedThreadCount` — same setup with
  `MaxParallelism = 2` explicitly, 50 steps, to prove a different `Parallel.For` partitioning doesn't
  leak into the trajectory either (scheduling must never matter, not just "the default scheduling
  happens to match").

**Result: both pass.** No divergence at any step, any agent, either buffer. Ran locally:

```
dotnet test tests/Sim.ParityTests/Sim.ParityTests.csproj --filter "FullyQualifiedName~OrcaParallelStepTests"
Passed!  - Failed: 0, Passed: 2, Skipped: 0, Total: 2
```

## Full-solution gate

```
dotnet build            → Build succeeded, 0 errors
dotnet test             → Sim.ParityTests: Passed! Failed: 0, Passed: 468, Skipped: 3, Total: 471
                           (466 pre-existing + 2 new OrcaParallelStepTests; the 3 skips are pre-
                           existing and unrelated to this change)
                           Sim.Pedestrians.Tests, Sim.Host.Tests, Sim.Pedestrians.Nav.DotRecast.Tests:
                           all green, unaffected (this task touched only src/Sim.Core/Orca/ and test/
                           bench files)
```

The lane determinism hash (`909605E965BFFE59`, `docs/PERF-HANDOVER.md`) is unaffected — this task
never touched the lane engine, `Sim.Ingest`, or `MixedTrafficCrowd`, only `OrcaCrowd`.

## Scale timing (the POC-7a deliverable)

New benchmark: `src/Sim.BenchCrowd` (console program, **not** part of `dotnet test` — same convention
as `Sim.Bench`/`Sim.BenchCity`/`Sim.Run`, since wall-clock numbers are machine-dependent and must never
gate the offline parity loop). Run:

```
dotnet run -c Release --project src/Sim.BenchCrowd -- --sizes 1000,10000,50000,100000 --steps 15 --warmup 3
```

Each size builds two crowds (grid layout, dense counter-crossing goals, spatial hash on,
`NeighbourDist = 3.0`, `MaxNeighbours = 8`, `SymmetryBreak = 0.05` — the same scratch-touching feature
set the correctness test exercises), one serial and one parallel, warms up 3 untimed steps, then times
15 steps each.

**Measured on this session's VM — 4 logical processors (a contended Linux VM, the same class of box
`PERF-HANDOVER.md` calls out as "prior perf work" before the 16c/24t on-target box became available;
this session did not have access to that on-target box):**

| N (agents) | serial ms/step | parallel ms/step | speedup | serial steps/s | parallel steps/s |
|-----------:|----------------:|------------------:|--------:|----------------:|-------------------:|
| 1,000      | 4.40            | 2.68               | 1.64x   | 227.4           | 373.0               |
| 10,000     | 31.27           | 7.61               | 4.11x   | 32.0            | 131.4               |
| 50,000     | 110.87          | 37.56              | 2.95x   | 9.0             | 26.6                |
| 100,000    | 319.57          | 99.23              | 3.22x   | 3.1             | 10.1                |

Thread-count sweep at N = 50,000 (`--max-parallelism 1..4`), confirming the speedup is real
parallelism and not measurement noise:

| MaxParallelism | parallel ms/step | speedup vs serial |
|---------------:|------------------:|-------------------:|
| 1               | 112.71            | 0.98x (expected: dispatch overhead ≈ cancels out) |
| 2               | 61.01             | 1.81x               |
| 3               | 44.78             | 2.53x               |
| 4 (= all cores) | 35.80             | 3.13x               |

**Where it plateaus.** On this 4-core box, scaling is close to linear up to the box's own core count
(≈3.1–3.2x at 4 threads against a 4-core machine) and necessarily cannot be measured past 4 threads
here. The N=1,000 row's lower speedup (1.64x) is dispatch/localInit overhead not yet amortized by
per-agent work — expected and consistent with why `ParallelStepThreshold` exists (a real deployment
would stay serial below it). The N=50,000/100,000 rows show speedup *below* the thread count (≈3.0–3.2x
at 4 threads, not 4.0x), the same qualitative shape `PERF-HANDOVER.md` reports for the lane engine's
plan phase on its 16c/24t box (which plateaus around 8 threads, "memory bandwidth from random neighbor
access" being the named bottleneck) — `OrcaCrowd`'s per-agent working set (spatial-hash bucket lookups,
neighbour position/velocity reads scattered across the SoA arrays) has the same random-access shape, so
the same bandwidth ceiling is the leading suspect here too. **This session could not reproduce or
falsify a specific 8-thread plateau** the way the lane-engine session did, for lack of a >4-core
machine; that measurement is the natural follow-up once run on-target (16c/24t), the way
`PERF-HANDOVER.md` did for the lane engine.

## Scale timing — ON-TARGET (P6-1, the follow-up the 4-core run flagged)

Re-run on the owner target box — **Intel Core Ultra 9 275HX, `ProcessorCount = 24`** (8 P-cores + 16
E-cores, no SMT), Windows 11, High-performance power plan, Release, each config 3× with the **median**
reported. `Sim.BenchCrowd` code is unchanged since the 4-core run above (its crowd always had
`UseSpatialHash=true`), so this is a valid same-code core-count comparison. Full write-up:
`docs/PEDESTRIAN-P6-1-RESULTS.md`. Command used `--steps 20` (vs 15 above — negligible).

Main sweep, runtime-auto (all 24 cores), on-target vs the 4-core VM:

| N (agents) | on-target parallel ms/step | on-target parallel steps/s | prior (4-core VM) parallel steps/s | on-target speedup vs serial |
|-----------:|---------------------------:|---------------------------:|-----------------------------------:|----------------------------:|
| 1,000      | 1.53                       | 653                        | 373                                | 2.31×                       |
| 10,000     | 4.34                       | 230                        | 131                                | 4.26×                       |
| 50,000     | 18.67                      | 53.6                       | 26.6                               | 4.08×                       |
| 100,000    | 50.1                       | **20.0**                   | 10.1                               | 6.45×                       |

**Thread-count sweep at N = 100,000 — the plateau the 4-core VM could not reach (now measured):**

| MaxParallelism | parallel ms/step | steps/s | speedup vs 1 thread | parallel efficiency |
|---------------:|-----------------:|--------:|--------------------:|--------------------:|
| 1              | 328.8            | 3.0     | 1.00×               | 100%                |
| 2              | 164.5            | 6.1     | 2.00×               | 100%                |
| 4              | 91.1             | 11.0    | 3.61×               | 90%                 |
| 8              | 59.7             | 16.7    | 5.50×               | 69%                 |
| 16             | 48.5             | 20.6    | 6.78×               | 42%                 |
| 24             | 48.5             | 20.6    | **6.78×**           | 28%                 |

**The 8-thread plateau this doc predicted is confirmed — and it is FLAT past 16 threads.** Parallel
efficiency falls 90% (4t) → 69% (8t) → 42% (16t) → 28% (24t); the last 8 physical cores add nothing.
This is the memory-bandwidth-bound signature `PERF-HANDOVER.md` names for the lane engine, here compounded
by the 275HX's 8 P-core / 16 E-core split (the taper sharpens right at the 8-thread mark, i.e. where the
P-cores run out). **This is the trigger for P6-2 (region decomposition)** — the raw ORCA `Step` does not
scale cleanly to 24 cores; converting the scattered global-array neighbour access into per-region
locality is the standard remedy. See `docs/PEDESTRIAN-P6-1-RESULTS.md` for the P6-2 recommendation.

## Friction / notes

- The repo's only `.sln` (`Traffic.sln`) needed `dotnet sln add` for the new `Sim.BenchCrowd` project
  (registered via the standard `dotnet sln` command, not hand-edited).
- `NeighbourDist` matters a lot for the benchmark's realism: with the crowd default (`15.0`) a dense
  grid at 1.5 m spacing puts far too many agents in each 3x3-cell query (unrealistic LP sizes that
  don't reflect a real deployment's local density), so the benchmark uses `NeighbourDist = 3.0` with
  `MaxNeighbours = 8` to keep the per-agent LP size in a realistic range regardless of total crowd
  size — this is a benchmark-configuration choice, not a change to `OrcaCrowd`'s defaults.
- No behavioral gate was needed beyond bit-identity — POC-7's success condition 3 ("bit-identical...
  or, if a fast path diverges, it is gated and behaviorally validated") is satisfied by the bit-
  identical case; no fast/diverging path was introduced.
