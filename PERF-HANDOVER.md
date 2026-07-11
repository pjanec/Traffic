# Performance handover — on-target (Windows, 16-core / 24-thread) optimization

You are a Claude Code session running **on the target hardware** (Windows 11, 16 physical
cores / 24 logical). Your job: make this engine run **WAY faster than single-threaded SUMO**
on this multi-core box, **without ever breaking behavioral parity** (the iron law below).
All prior perf work was done on a contended 4-core Linux VM; you can finally measure and
optimize where the bottleneck actually manifests. Read `CLAUDE.md` and `DESIGN.md` first —
this file assumes them.

---

## SESSION RESULTS (on-target Windows, 16-core/24-thread) — read this before §2–§4

An on-target session ran the plan below. Its findings **supersede the SoA recommendation in
§4** — read them first.

**Toolchain / gates:** now **229 passed, 1 skipped** (added one determinism test pinning the
parallel-export path byte-identical). Determinism hash unchanged: `909605E965BFFE59`.

**Bottleneck: CONFIRMED memory-bandwidth, not GC.** `dotnet-counters`: `% Time in GC` avg 6.5%;
Server GC A/B a wash (5.83 vs 5.85s) → GC is not the wall. CPU avg **11.4% of 24 cores** (~2.9
busy) → box not saturated (a bandwidth wall leaves cores stalled, not busy). Per-phase 1→8t
scaling: `plan` peaks **3.43×@8t** then regresses; `willPass` **2.56×@8t** then regresses;
16/24 threads are *slower* than 8 (HT oversubscription). Sweet spot for city-3000 = **8 threads**.

**Three byte-identical wins committed** (each: 3 gates green + parity-reviewer ACCEPT):
- `perf(export)` — serial emit by default (opt-in `Engine.ParallelExport`); parallel emit was a
  net loss here (0.56–0.71× at every thread count). ~3% @8t.
- `perf(plan)` — dense active-index list; the parallel loops dispatched over all `_vehicles` and
  touched ~40% dead (not-departed/arrived) scattered object headers per step just to skip them.
  ~2.5% @8t (9/12 paired rounds).
- `perf(insert)` — int `LaneHandle` filter (was string `LaneId`) + `LanesByHandle[h]` re-resolution
  (drops a per-candidate closure alloc) in `TryInsertOnLane`. ~noisy few %.

**vs single-threaded SUMO 1.20.0 on THIS box** (measured; SUMO installed from the win64 zip):
SUMO 1-thread ≈ 17–18 s. Engine **step-loop** (the hot path): serial ~1.3×, **8-thread ~2.4–3×
faster**. Whole-process is dragged to ~1.4× ONLY because `Sim.BenchCity`'s own post-processing
(build a dict of ~5M points + `OrderBy` per vehicle + tripinfo/summary XML) adds ~4.6 s that SUMO
doesn't do — that is a benchmark-harness artifact, NOT engine cost. Engine `LoadScenario` ≈ 1.1 s.

**SoA (§4): TWO NEGATIVE RESULTS — do NOT re-attempt the mirror approach.**
1. Caching `Pos` inline in the `LaneNeighborQuery` buckets (so sort/binary-search stop chasing
   pointers): **null** (−0.1% wall). The search/sort chases were not the cost.
2. Full foe-field SoA *probe* on the dominant hot read (`LeaderFollowSpeedConstraint`): pack
   Pos/Speed/LatOffset/Length/Decel/Width/isCacc into `EntityIndex`-keyed arrays, have the
   leader lookup return an index, read the leader entirely from the arrays (zero foe-object
   touch). Byte-identical, but **REGRESSED ~3–4%** (plan phase got *worse*). Root cause: refreshing
   the arrays each step reads every active vehicle's scattered `Kinematics`/`VType` — the *same*
   traffic it was meant to avoid — then the plan reads it *again*. Net memory traffic goes **up**.
   The "cheap sequential refresh from objects" premise in §4 is **false** on this workload.
   For SoA to help it must be the **source of truth** (remove `Kinematics` from `VehicleRuntime`,
   `ExecuteMoves` writes the arrays, hundreds of read/write sites migrate) — a massive, high-risk
   rewrite with now-*demonstrated-uncertain* payoff. Not justified for the current ~2.4× at 8t.

**Practical conclusion:** in this reference-object architecture the hot-path bandwidth wall is
the ceiling (~2.4× SUMO @8t) for byte-identical work; the remaining lever is a source-of-truth
SoA/struct rewrite (huge, risky) or SIMD (marginal, gather-heavy — see DESIGN.md). Load (~1.1 s)
is XML-parsing-bound (a vType-resolve memoization was tried: no-op).

**Measurement note:** the box is noisy (~8% run-to-run, thermal drift). Only **interleaved paired
A/B of two snapshotted builds** (build both, alternate runs, count paired wins) reliably resolves
sub-5% changes — single-config medians taken minutes apart are confounded by drift.

### The 4× target and the opt-in fast-mode gate (goal: ≥4× SUMO hot-path @8 cores)

Precise hot-path gap: **SUMO sim-tick 17.19 s** (its own load is only 0.64 s) vs **engine
step-loop @8t 5.62 s = 3.06×**. To reach 4× the step-loop must drop to **≤4.30 s (−23%)**.

**Shipped: the opt-in fast-mode escape hatch + its automated behavioral validator** (commit
`feat(fast)`). `Engine.FastMode` (default false → deterministic path byte-identical, untouched).
`Sim.BenchCity --fast-gate` runs deterministic + fast on the same scenario and asserts fast mode is
*behaviorally sound* without SUMO: **0 gridlock**, **aggregate parity vs the deterministic baseline**
(arrived / mean duration / mean speed / trip-duration KS within a tight 2–5% tolerance, vs the loose
0.35 SUMO bar), and **no vehicle overlaps** beyond the model-inherent baseline (relative criterion,
internal junction lanes excluded). This makes any non-byte-identical fast-mode change auto-checkable.

**Where 4× can and cannot come from (measured this session):**
- **Serial tail is 44% of the tick @8t** (insert ~923 ms + speedGain ~391 + foeIndex ~353 + refill
  ~195 + execute ~177 + serial-emit ~458) — the Amdahl anchor — but it **resists parallelization**:
  `insert`/`speedGain` mutate shared lane state in order (the LC-arbitration order-sensitivity), and
  a byte-identical **parallel `foeIndex`** (two-lowest-EntityIndex reduction under per-lane locks —
  provably identical, verified by a serial-vs-parallel city-3000 trip SHA match) **REGRESSED**
  (728 ms @8t vs 441 ms serial) on **lock contention** on popular junction lanes. Even a best-case
  fast-mode parallelization of the tractable serial work maths to only **~3.3–3.5×**.
- **So 4× fundamentally needs the PARALLEL phases (willPass+plan, 59% of the tick, ~3320 ms @8t)
  to scale better than ~3×.** Getting them to ~5× (≈2000 ms) would hit ~4.0×. The only lever is
  cutting their per-vehicle memory traffic = **source-of-truth SoA**: make the hot foe fields
  (Pos/Speed/LatOffset + a vtype-index table) the AUTHORITATIVE store that `ExecuteMoves`/insertion
  write directly (write-through at the few `Kinematics` write sites — no per-step refresh pass, which
  is what made the earlier mirror probe regress), and read foe fields from those arrays across the
  ~10 gap-math constraint methods. Byte-identical by construction, but a large dedicated rewrite with
  still-uncertain payoff. This — or spatial partitioning *paired with region-local memory layout* —
  is the only credible path to 4×; the serial-tail/fast-mode work alone cannot reach it.

**UPDATE — the source-of-truth SoA was BUILT and MEASURED, and it REGRESSES. Do not retry per-field
SoA.** Implemented the full write-through design (no refresh pass): neighbor buckets return an
EntityIndex, EntityIndex-keyed arrays (`_soaPos/Speed/LatOffset/Length/Decel/Width/IsCacc`) written
through at insertion + end of `ExecuteMoves` + snapshot restore (the snapshot test caught a missing
write site — fixed), and `LeaderFollowSpeedConstraint` reads the leader entirely from the arrays
(zero foe-object touch). **Fully byte-identical** (229 tests, hash `909605E965BFFE59`, city-3000
serial-vs-parallel trip SHA match, aggregate PASS). Interleaved paired A/B @8t: **plan phase −5.4%
(worse), wall neutral-to-worse (4/12 rounds).** Root cause, and the general lesson: **the gap math
reads ~7 fields of a SINGLE leader (Pos/Speed/LatOffset/Length/Decel/Width/isCacc) — an AoS-shaped
access.** The `VehicleRuntime` object already packs those into 1–2 cache lines; **per-field SoA
splits them into 7 separate arrays = 7 cache lines for one foe read.** SoA-per-field wins for
streaming ONE field over MANY entities; it is the WRONG layout for MANY fields of ONE foe. An
AoS-struct-array (`HotFields[]` indexed by EntityIndex) would be ~neutral, not a win: the real cost
is the **random leader access** (`leaderIdx` is arbitrary → ~1 random cache line), which is ~1 line
regardless of AoS/SoA and is only removable by **spatial memory reordering** (physically sorting
vehicles by position each step so a follower's leader is adjacent in memory) — a far more invasive
change with its own per-step reordering overhead and uncertain net payoff.

**Bottom line on 4× (@8 cores, hot-path):** not reachable via the data-layout or serial-tail levers
explored this session. The parallel phases are capped by random-neighbor-access bandwidth (SoA
doesn't fix it — wrong access shape); the serial tail (44%) is order-dependent / lock-contended.
The only untried theoretical paths are (a) **spatial memory reordering** of the vehicle store, or
(b) an aggressive **fast-mode** that approximates/skips work (validated by the shipped `--fast-gate`)
— both high-effort, high-risk, uncertain. Shipped this session: 3 byte-identical wins + the
fast-mode scaffold + behavioral gate; measured hot-path **3.06× SUMO @8t**.

---

## 0. The iron law (never negotiable)

A change is only acceptable if **either**:
1. it is **byte-identical** — all committed goldens pass and the determinism hash is unchanged; **or**
2. it is behind an **opt-in fast-mode flag** (off by default), so the deterministic path is untouched.

**The three parity gates — run all three after every change:**
```powershell
dotnet test -c Release                       # must be: 228 passed, 1 skipped
dotnet run  -c Release --project src\Sim.Bench   # hashA == hashPar == 909605E965BFFE59
# city-3000 must stay 0 stuck + aggregate PASS:
dotnet run -c Release --project src\Sim.BenchCity -- scenarios\_bench\city-3000 --no-fcd `
  --sumo-summary  scenarios\_bench\city-3000\summary.xml `
  --sumo-tripinfo scenarios\_bench\city-3000\tripinfo.xml `
  --aggregate-tolerance scenarios\_bench\city-3000\aggregate-tolerance.json
```
If the hash changes or a golden fails and the change was meant to be byte-identical: **revert it.**
The `dotnet test` loop must never call SUMO or the network (it runs on committed goldens only).

Get a `parity-reviewer` sign-off (see `.claude/agents/`) before promoting any hot-path change — it
already caught one real edge case in this work (the LatOffset recenter gap).

---

## 1. Where we are (facts, not guesses)

**Branch:** `claude/rail-support-2tas1n` (continue here or a child branch; never force-push shared history).

**What shipped this session (all byte-identical, all committed):**
- **willPass/plan fusion** — the engine used to run *two* full plan passes over every near-junction
  vehicle (a willPass pre-pass, then the real plan). They differ only in one crossing-yield term, so
  the pre-pass intent is now cached and reused by `PlanMovements` unless the vehicle takes the crossing
  yield — gated by `FusionEligible` (falls back to the exact two-pass path for sigma>0 / IDMM /
  bluelight / opposite-overtake / obstacles / action-step-skip scenarios). See `Engine.PrePlanVehicle`,
  `Engine.FusionEligible`, `VehicleRuntime.ReuseIntent`/`CrossingYieldTaken`. This cut true-serial
  city-3000 from ~54s → ~36s on the VM.
- **Parallel Export phase** — `EmitTrajectory` computes per-vehicle geometry into a reusable buffer in
  parallel, appends serially (order-independent comparator/hash). Gated on size + no export observer.
- **Measurement knobs on `Sim.BenchCity`:** `--serial` (force single-thread, no `Parallel.For`),
  `--max-parallelism N` (cap worker threads — `Engine.MaxParallelism`), `--no-fcd` (skip FCD without the
  PowerShell-hostile `--fcd-out ""`), `--profile` (`Engine.ProfilePhases`, per-phase wall time).
- **`scripts\bench-scaling.ps1`** — sweeps thread counts, prints wall/speedup/efficiency/vs-SUMO + CSV.
  Locale-proofed (cs-CZ comma decimals handled).

**The measured scaling curve on this box (24 logical cores, city-3000):**

| threads | wall  | vs-serial | efficiency |
|--------:|------:|----------:|-----------:|
| serial  | 11.48 | 1.00×     | —          |
| 1       | 11.55 | 0.99×     | 99%        |
| 2       |  7.90 | 1.45×     | 73%        |
| 4       |  6.34 | 1.81×     | 45%        |
| 8       |  5.68 | 2.02×     | 25%        |
| 16      |  5.67 | 2.02×     | 13%        |
| 24      |  6.13 | 1.87×     |  8%        |

**Speedup plateaus at ~2× and 24 threads is *slower* than 16** (hyperthread oversubscription).

**The per-phase serial profile (`--profile --serial`, 4-core VM — fractions are stable):**

| phase     | share | parallel? |
|-----------|------:|-----------|
| plan      | 37.6% | yes       |
| willPass  | 36.1% | yes       |
| emit      | 14.6% | yes (size+no-observer gated) |
| insert    |  5.9% | no        |
| foeIndex  |  2.5% | no        |
| speedGain |  1.7% | no        |
| refill    |  1.1% | no        |
| execute   |  0.5% | no        |

**The diagnosis:** ~88% of the work is in phases that *are* parallelized (plan+willPass+emit), yet the
whole thing plateaus at ~2×. So the ceiling is **not** a serial tail (that's only ~12%) — it's that the
**memory-bound parallel arithmetic phases stop scaling past ~2–4 threads**. On fast cores the compute
shrinks but the per-vehicle memory traffic (chasing scattered `VehicleRuntime` heap objects + random
neighbor/foe objects) saturates the memory subsystem. **This is the classic case SoA is meant to fix** —
but confirm it with a profiler before the big refactor (step 2 below).

---

## 2. First: CONFIRM the bottleneck (cheap, do before any refactor)

Do not start SoA blind. Two competing cheap hypotheses could explain the plateau; rule them in/out first:

**(a) GC pressure.** The run allocates ~2.5 GiB. If GC is serializing, the fix is trivial and huge.
- Try Server GC + concurrent: add to `src\Sim.BenchCity\Sim.BenchCity.csproj`
  `<ServerGarbageCollection>true</ServerGarbageCollection>` (and try `<ConcurrentGarbageCollection>`),
  or set env `DOTNET_gcServer=1`, and re-run the scaling sweep. If efficiency jumps, GC was the wall.
- Measure GC %: `dotnet-counters monitor -- <the benchcity run>` (watch `% Time in GC`), or capture with
  **PerfView** (`GCStats`). If GC% is high, chase allocations (the emit path builds ~5M point structs +
  snapshots; `alloc total` in the metric line quantifies it).

**(b) Memory bandwidth / cache misses.** If GC is low but the parallel phases still don't scale, it's
bandwidth. Confirm with **PerfView** (`Memory` / hardware counters) or **Intel VTune** (Memory Access
analysis → look for high LLC miss rate / DRAM-bound in `ComputeMoveIntent`). If DRAM-bound, SoA is justified.

Report which one it is (with the counter numbers) before proceeding — it decides the whole plan.

---

## 3. The optimization backlog (ranked by expected ROI × safety)

Work top-down; measure each on this box; keep only what beats noise (run 3+, take median; the first run
is JIT warm-up). The VM was too noisy and too few cores to see these — you can.

1. **Server GC / runtimeconfig tuning** (trivial, possibly big). See 2(a). Byte-identical by construction.
2. **Kill emit allocations** (emit = 14.6%, ~5M `VehicleExportSnapshot`+`TrajectoryPoint`/run). In the
   benchmark the FCD isn't written (`--no-fcd`) yet the full `TrajectorySet` is still built for the stuck
   detector. Options: a columnar/preallocated point store, or a lighter benchmark-only sink. Keep the
   golden path (real FCD parity) byte-identical — gate any lossy sink behind the benchmark.
3. **Re-try a chunked/range partitioner** for the plan/willPass `Parallel.For`. Per-index work-stealing
   was chosen earlier for load-balance on a *sparse* active list; the fusion changed the balance and a
   `Partitioner.Create(0, N)` (or compacting active indices into a dense array first — see below) may cut
   dispatch overhead and improve locality. Byte-identical (order-independent). Measure — it may or may not help.
4. **Dense active-index list.** ~40% of `_vehicles` are not-yet-departed/arrived; the parallel loops still
   dispatch over them. Compact `Inserted && !Arrived` indices once per step (serial, O(N), cheap) and have
   plan/willPass iterate the dense list. Byte-identical. (This was prototyped here but lost in a stash
   mishap and was inconclusive on the VM — re-do it and measure on real cores.)
5. **SoA of the hot per-vehicle fields** — *the main event if step 2 says bandwidth-bound.* Detail in §4.
6. **Fast-mode flag** (opt-in, lossy) for the ~12% serial phases (insert/foeIndex/speedGain). **Low
   priority** — the profile shows this is a small slice, so parallelizing it can't break the 2× plateau.
   Only worth it after the parallel phases scale.

---

## 4. The SoA refactor (do this if §2 confirms memory-bound)

**Hypothesis:** `ComputeMoveIntent`'s hot loop reads the ego `VehicleRuntime` (a large heap *class* in
`List<VehicleRuntime> _vehicles`) plus *random* neighbor/foe `VehicleRuntime` objects. Scattered large
objects = many cache lines touched per vehicle = bandwidth-bound at scale. Packing the hot fields into
contiguous `EntityIndex`-keyed arrays makes each access dense.

**Scope reality:** a *partial* SoA gives ZERO benefit — the loop still dereferences the object for every
field you didn't move. It's all-or-most-of the hot path. Plan it as a dedicated, parity-gated effort:

- **Hot fields to pack** (read in the plan/willPass/emit inner loop and by neighbor/foe lookups):
  `Pos, Speed, LatOffset, LaneHandle, LaneSeqStart, LaneSeqLen, LaneSeqIndex`, and a `VTypeIndex`
  (resolved vTypes are per-type — keep them in a small dense array indexed by type, already cache-friendly).
- **Keep cold fields** (stops, RNG, reroute bookkeeping, EV/overtake state) on the class.
- **The random-access win is the biggest:** make `FindFoeVehicle` and the `LaneNeighborQuery` leader/
  follower lookups return **indices**, and rewrite the gap math (`LeaderFollowSpeedConstraint`,
  `SameTargetMergeConstraint`, `AdaptToJunctionLeader`, `SeenToInternalLaneEntry`) to read the SoA arrays
  for the *foe/leader* fields instead of dereferencing that vehicle's object.
- **Sync discipline:** either make the SoA arrays the source of truth (object mirrors them) or refresh
  them once per step from the objects (cheap sequential pass) — but respect the plan/execute contract:
  the plan reads *start-of-step* SoA, `ExecuteMoves` writes the new values. No mid-plan mutation.
- **Verify continuously:** the arithmetic must stay bit-identical — same field values, same order of
  operations. Run all three parity gates after each slice; expect the hash to stay `909605E965BFFE59`.
  If a slice can't be made byte-identical, it's wrong (not "close enough") — the deterministic path is sacred.

**Incremental path that keeps it green at every step:** (a) add the SoA arrays alongside the class,
populated but unread (byte-identical, no perf change — just proves sync). (b) Switch the *foe/neighbor*
reads to the arrays (biggest bandwidth win, self-contained). Measure. (c) Switch the ego reads. Measure.
Stop when the scaling curve flattens out to your target.

---

## 5. Also worth doing while here

- **Multilane at scale** is *not* solved. city-3000 is pinned to `-L1` because
  `NetworkModel.ResolveSequenceCore` resolves a *single* greedy lane sequence per multi-edge route and
  throws `"No lane on edge … has a <connection> to …"` when its per-hop lower-index-nearest-hint choice
  dead-ends — which happens for a fraction of routes on dense `netgenerate -L2+` grids. Generalizing this
  (resolve lane *sets* per hop, or SUMO's backward best-lanes with runtime re-resolution) is the unlock
  for multi-lane city benchmarks — and it's what makes lane-changing traffic actually exercise the cores
  you're trying to parallelize. Regenerate an `-L2` city bench via `scripts\gen-benchmark.sh` once routes resolve.
- **HT oversubscription:** 24 threads < 16 threads on this box. Consider defaulting `MaxParallelism` to the
  *physical* core count when you productionize any auto-parallel path.

---

## 6. Workflow on this box

- **Two loops, separate:** `dotnet test` = offline parity (no SUMO, no net). SUMO only for regen/investigation.
- **SUMO baseline:** `scripts\bench-scaling.ps1 -Sumo` (needs `sumo` on PATH) gives the vs-SUMO column; the
  committed reference is ~35s single-thread. Beating single-thread SUMO with our *serial* path is a stated
  goal — you're at ~parity (11.5s here vs SUMO on this box; run `-Sumo` to get the real ratio).
- **Commit discipline:** small, verified, parity-gated commits. End messages with the Co-Authored-By /
  Claude-Session trailers already in the log. Push to `claude/rail-support-2tas1n`.
- **Measure like this:** `pwsh scripts\bench-scaling.ps1 -Threads 1,2,4,8,16 -Repeats 5 -Sumo` for a curve;
  `dotnet run -c Release --project src\Sim.BenchCity -- scenarios\_bench\city-3000 --no-fcd --profile` for phases.

**Definition of done for this effort:** the scaling curve keeps climbing past 4 threads (efficiency stays
high to 8–16), the engine is many× single-threaded SUMO on this box, and all three parity gates are green
(or the speedup is behind an opt-in fast-mode flag with the deterministic path still byte-identical).
