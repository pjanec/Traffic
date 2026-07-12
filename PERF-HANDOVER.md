# Performance handover — on-target (Windows, 16-core / 24-thread) optimization

You are a Claude Code session running **on the target hardware** (Windows 11, 16 physical
cores / 24 logical). Your job: make this engine run **WAY faster than single-threaded SUMO**
on this multi-core box, **without ever breaking behavioral parity** (the iron law below).
All prior perf work was done on a contended 4-core Linux VM; you can finally measure and
optimize where the bottleneck actually manifests. Read `CLAUDE.md` and `DESIGN.md` first —
this file assumes them.

---

## ON-TARGET SESSION LOG — what was tried, what happened, why (READ THIS FIRST)

> This log **supersedes the SoA recommendation in §2/§4 below** — §4's "source-of-truth SoA is the
> main event" was tried in full and it **regresses**. §2–§6 are the original (pre-measurement) plan,
> kept for context; where they disagree with this log, the log is right (it was measured on-target).
>
> **The one-paragraph summary:** the engine is **~3.5× single-threaded SUMO on the hot-path tick @8
> cores** (region path @8t grid-32 ≈ 4.82 s cool vs SUMO 17.19 s = **3.57×**) with byte-identical wins
> banked (the region foundation + two new ones — insert-prewarm and emit-handle-array — see the
> Session-2 addendum) and an opt-in fast-mode + behavioral gate shipped. The bottleneck is **memory
> bandwidth from random neighbor access** in the plan/willPass phases (62% of the tick, capped at ~3×
> scaling). The **≥4× goal is not reached byte-identically** and is now understood to be structurally
> out of reach on this object-graph memory layout: every data-layout and serial-tail lever tried
> either did nothing or regressed (below). Crossing 4× needs either spatial memory reordering (attacks
> the wall directly; research) or an aggressive opt-in fast-mode serial-tail parallelization (caps
> ~3.9×, trades byte-identity for --fast-gate behavioral parity).

### Current state (measured)

- **Hot-path (step loop) @8 threads: 5.62 s → 3.06× vs single-threaded SUMO 1.20.0** on this box
  (SUMO sim-tick 17.19 s; SUMO's own load is only 0.64 s). Serial engine ~1.3×. Sweet spot = **8
  threads** (16/24 are *slower* — HT oversubscription).
- **Gates:** `dotnet test` **229 passed / 1 skipped**; determinism hash **`909605E965BFFE59`**
  (single == parallel); city-3000 **0 stuck + aggregate PASS**. All wins below hold all three.
- **Whole-process** (load+sim+output) looks like only ~1.4× SUMO, but that is a **benchmark-harness
  artifact**: `Sim.BenchCity` post-processing (dict of ~5M points + `OrderBy`/vehicle + tripinfo/
  summary XML) adds ~4.6 s that SUMO doesn't do. Engine `LoadScenario` itself is only ~1.1 s
  (XML-parsing-bound). Compare the **step loop** for engine speed, not whole-process.

### Bottleneck diagnosis (confirmed, not guessed)

- **Not GC.** `dotnet-counters`: `% Time in GC` avg 6.5%; **Server GC A/B is a wash** (5.83 vs
  5.85 s). If GC were the wall, per-core heaps would have moved it.
- **Memory bandwidth, from random neighbor access.** CPU avg **11.4% of 24 cores** (~2.9 busy) → the
  box is *not* saturated; cores stall on loads. Per-phase 1→8t scaling: `plan` peaks **3.43×@8t**
  then regresses, `willPass` **2.56×@8t** then regresses. The hot loop's dominant cost is a follower
  dereferencing its **leader/foe** (`Kinematics` etc.) at a **random** heap address — ~1 cache-line
  miss per foe read, per vehicle, per step, and it doesn't parallelize past ~3× because the memory
  subsystem saturates.

### Experiments log — MOST OPTIMIZATION IDEAS HERE WERE ALREADY TRIED. Check before repeating.

Legend: **WIN** = shipped (committed); **NULL** = byte-identical but no measurable gain (reverted);
**REGRESS** = made it slower (reverted). All A/B is **interleaved paired** (see methodology note).

**Shipped wins (byte-identical, parity-reviewer ACCEPT):**
1. **WIN — serial emit by default** (`perf(export)`, opt-in `Engine.ParallelExport`). The parallel
   Export path (compute-into-scratch-then-append) is a net loss here (0.56–0.71× at every thread
   count) — per-vehicle geometry is memory-light, so `Parallel.For` dispatch + scratch write/re-read
   dominates. Serial emit ~+3% @8t.
2. **WIN — dense active-index list** (`perf(plan)`). The parallel willPass/plan loops dispatched over
   ALL `_vehicles` and touched the scattered header of every not-yet-departed/arrived vehicle (~40%)
   just to skip it. Compact active indices once/step → dispatch over the dense list. ~+2.5% @8t
   (9/12 paired rounds).
3. **WIN — insert int `LaneHandle` filter** (`perf(insert)`). `TryInsertOnLane`'s leader scan
   filtered `ActiveVehicles()` by *string* `LaneId`; switched to the dense int `LaneHandle` (in
   lockstep, provably equivalent) + `LanesByHandle[h]` re-resolution (drops a per-candidate closure
   alloc). Strict work reduction; wall gain small/noisy.

**Blind alleys (do NOT repeat — each was built, measured, reverted):**
4. **REGRESS — source-of-truth per-field SoA for foe reads** (the big one §4 recommends). Full
   write-through design, NO per-step refresh pass: neighbor buckets return an `EntityIndex`;
   `EntityIndex`-keyed arrays `_soaPos/Speed/LatOffset/Length/Decel/Width/IsCacc` written through at
   insertion + end of `ExecuteMoves` + snapshot restore; `LeaderFollowSpeedConstraint` reads the
   leader entirely from the arrays (zero foe-object touch). **Fully byte-identical** (229 tests, hash
   held, city-3000 serial-vs-parallel trip SHA match, aggregate PASS). Result @8t: **plan −5.4%,
   wall neutral-to-worse.** **Root cause / the lesson:** the gap math reads **~7 fields of ONE
   leader** — an **AoS-shaped** access the `VehicleRuntime` object already packs into 1–2 cache
   lines. Per-field SoA splits them into **7 separate arrays = 7 cache lines** for one foe read.
   SoA-per-field is for streaming ONE field over MANY entities; it is the wrong layout for MANY
   fields of ONE foe. An AoS-struct-array (`HotFields[]`) would be ~neutral (still 1 random line);
   the cost is the *random* `leaderIdx` access, ~1 line regardless of layout. **Per-field SoA cannot
   help this access pattern — do not retry it.**
5. **REGRESS (earlier variant) — mirror SoA with a per-step refresh pass.** Same arrays but
   *refreshed* each step from the objects. Regressed ~3–4% because the refresh re-reads every active
   vehicle's scattered `Kinematics`/`VType` — the SAME traffic it was meant to avoid — then the plan
   reads it again. (Superseded by #4, which removed the refresh and STILL regressed for the deeper
   AoS reason.)
6. **NULL — inline `Pos` in the `LaneNeighborQuery` buckets.** Store the sort key inline so the
   per-step sort + leader binary-search stop chasing pointers. Byte-identical, **−0.1%** (noise). The
   search/sort pointer-chases were NOT the cost; the gap-math foe deref is.
7. **REGRESS — byte-identical parallel `foeIndex`.** `BuildFoeApproachIndex`'s output is the two
   lowest-EntityIndex vehicles per internal lane (order-independent), so a parallel two-lowest
   reduction under per-lane locks is provably identical (verified by a serial-vs-parallel city-3000
   trip SHA match). But it **regressed 728 ms vs 441 ms @8t** on **lock contention** on popular
   junction lanes (+ dispatch overhead over tiny per-vehicle work). Lock-free partial-merge might
   avoid it but the phase is only ~6% of the tick — low ceiling.
8. **NULL — vType-resolve memoization** (load-time). `CreateRuntime` calls `VTypeDefaults.Resolve`
   per vehicle (3000× for one shared type on city-3000). Memoizing (resolve each distinct vType once)
   is byte-identical and a strict alloc reduction, but **no measurable load gain** — load is
   XML-parsing-bound, not resolve-bound. Reverted (kept the codebase clean).
9. **Server GC** — a wash (see diagnosis). `<ServerGarbageCollection>` left false.

**Serial-tail parallelization — why it's blocked (analysis, mostly not attempted after #7):** the
serial tail is **44% of the tick @8t** (insert ~923 ms + speedGain ~391 + foeIndex ~353 + refill
~195 + execute ~177 + serial-emit ~458). But `insert` and `speedGain` **mutate shared lane state in
order** (the lane-change-arbitration order-sensitivity DESIGN.md warns about) → not parallelizable
byte-identically; `foeIndex` regressed on locks (#7); `refill`-sort and `execute`-integration are
individually tiny/independent (dispatch overhead ≈ work). Even a best-case fast-mode parallelization
of the tractable serial work maths to only **~3.3–3.5×**.

### Session-2 addendum — region foundation + two new byte-identical wins (~3.06× → ~3.57×)

A later session built the **domain-decomposition region path** (`--region`, see `DOMAIN-DECOMP.md`)
and banked two more byte-identical wins on top. All hold the three gates (229 tests, hash
`909605E965BFFE59`, city-3000 default-vs-`--region` trip-SHA match + aggregate PASS + 0 stuck).

- **WIN — insert route-sequence prewarm** (`perf(insert)`, commit `11fc90b`). `TryInsertOnLane`
  re-resolved each vehicle's lane sequence (`ResolveLaneSequenceHandlesWithArrival`, a **pure**
  function of `(route.Edges, DepartLaneIndex)` + immutable network) at insertion time, serially, in
  the hot path. Pre-resolve **every distinct `(RouteId, DepartLaneIndex)` in parallel at load** into a
  cache; insertion becomes a dict lookup. **insert 734 → 206 ms (−72%)**, ~9% wall. Distinct from the
  older #3 (int-handle filter) — this moves the whole resolution off the hot path.
- **WIN — emit by lane-handle array, not `LaneId` string hash** (`perf(emit)`, commit `fd7bf4b`).
  `EmitTrajectory` did `LanesById[v.LaneId]` (string hash + dict probe) per vehicle per frame.
  Materialize `_network.LanesByHandle` into a dense `Lane[]` once at load, index by `v.LaneHandle`
  (kept in lockstep with `LaneId`). **serial emit 525 → 380 ms (−28%)**, applies to default and region.
- **WIN — region-parallel `speedGain`** (`perf(region)`, commit `e2a3950`). `DecideSpeedGainChanges`
  (the post-move keep-right/strategic/speed-gain phase) is **byte-identically parallelizable** —
  surprising, but its own header already proves the property: every vehicle's decision reads ONLY the
  ONE frozen `postMoveNeighbors` snapshot (never a live read of another vehicle's LaneId), and writes
  ONLY its own fields + the order-independent thread-safe command buffer (the speed-gain change is
  deferred; the inline keep-right swap is re-read only by the same vehicle). Extracted the body to
  `DecideSpeedGainForVehicle`, rebuild the POST-move region grouping, region-parallel the refill
  (`RefillRegion`, disjoint lanes) AND the decision loop. **speedGain 607 → 395 ms interleaved @8t
  (−35%).** KEY lesson: parallelizing the decision loop ALONE *regressed* (the serial post-move refill
  ~140 ms is a fixed floor + `BuildActiveIndices` overhead > the loop's saving); region-parallelizing
  the refill too is what makes the phase a net win. Verified default-vs-`--region` trip-SHA match
  across repeated runs (deterministic → race-free), 229 tests, hash held, aggregate PASS + 0 stuck.
  **This refutes the old "serial tail is order-dependent" blanket claim for `speedGain`** — the frozen-
  snapshot discipline makes it order-INdependent. (`insert` and `foeIndex` remain un-parallelizable:
  insert is genuinely order-dependent + pool-racing; foeIndex is sync-overhead-bound, #7.)
- **REGRESS (re-confirmed #7) — region-parallel emit** (commit `59037e1`, then removed in `fd7bf4b`).
  A region-parallel emit into `_emitScratch` (EntityIndex-ordered append → byte-identical) beat serial
  emit *only while the string hash was still there* (481 vs 525 ms). Once the handle-array removed the
  hash, serial emit (380 ms) **beats** the parallel dispatch (472 ms) — so it was removed. Lesson: once
  a per-vehicle phase is memory-light, `Parallel.For` dispatch overhead dominates (same shape as #1).
- **REGRESS (re-confirmed #7, twice) — byte-identical parallel `foeIndex`, no-int-array variant.**
  Retried with `v.EntityIndex` as the ordering key + only `Array.Clear` (dropping the O(laneCount)
  int-array reset), striped locks (256). **Still regressed: 762 ms vs 377 ms serial.** Confirms #7's
  root cause is the **per-touch lock overhead itself** (~25 ns × millions of internal-lane touches ≈
  the entire serial cost), not the reset. The phase's work-per-touch is too fine-grained to survive
  ANY synchronization. **Do not retry a locked foeIndex** — only a lock-free partition-merge could
  help, and the ceiling (~6% of tick) does not justify it.

### Shipped enabler: opt-in fast-mode + automated behavioral gate

`Engine.FastMode` (default false → deterministic path byte-identical, untouched — no committed
test/scenario sets it). `Sim.BenchCity --fast-gate` validates a non-byte-identical fast-mode change
**behaviorally, without SUMO**: runs deterministic + fast on the same scenario and asserts (1) **0
gridlock**, (2) **aggregate parity vs the deterministic baseline** (arrived / mean duration / mean
speed / trip-duration KS within a tight **2–5%** tolerance — much tighter than the 0.35 SUMO bar),
(3) **no vehicle overlaps** beyond the model-inherent baseline (relative criterion; internal junction
lanes excluded). Bootstrap (fast==deterministic) passes. **This is how any future fast-mode work is
checked** — it answers "behaviorally sound, not just faster?" automatically.

### The ≥4× verdict and the only untried paths

Gap: step-loop must drop **5.62 s → ≤4.30 s (−23%)**. This session ruled out the credible byte-
identical levers: the **parallel phases (59% of tick)** are capped by random-neighbor-access
bandwidth and **SoA is the wrong access shape** (#4), and the **serial tail (44%)** is
order-dependent / lock-contended. So 4× is **not reachable** via data-layout or serial-tail
parallelization. The only remaining theoretical paths, both **high-effort / high-risk / uncertain**:
- **(a) Spatial memory reordering** of the vehicle store — physically sort the hot per-vehicle data
  by lane+position each step so a follower's leader is *adjacent in memory* (turns the random foe
  access into a sequential one, the only thing that actually attacks the wall). Has its own per-step
  reorder cost; net benefit unknown. This is the one lever that could plausibly move the parallel
  phases — prototype it in isolation and measure before committing.
- **(b) Aggressive opt-in fast-mode** that approximates/skips work (e.g. coarser neighbor queries,
  parallelized-with-deterministic-tie-break serial tail), validated by `--fast-gate`. Buys the
  serial tail but not the parallel-phase bandwidth wall; realistically ~3.5×, not 4×, unless combined
  with (a).

### Measurement methodology (critical — the box is noisy)

Run-to-run noise is ~8% (thermal drift). Only **interleaved paired A/B of two snapshotted builds**
reliably resolves sub-5% changes: build BOTH variants, copy each `bin/Release/net8.0` aside, then
**alternate** runs (old, new, old, new…) and count paired wins + compare medians. Single-config
medians taken minutes apart are confounded by drift and will lie. Verify byte-identity of a change on
a **junction** scenario via `city-3000 --serial` vs default **tripinfo SHA match** (the determinism
hash in `Sim.Bench` uses a junction-free highway, so it does NOT exercise junction/foe-index code).

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

> ⛔ **OUTDATED — this was BUILT, MEASURED, and it REGRESSES.** See experiment #4 in the ON-TARGET
> SESSION LOG at the top of this file. The per-field-SoA hypothesis below is *wrong for this access
> pattern*: the gap math reads ~7 fields of ONE foe (AoS-shaped), so per-field SoA turns one foe read
> into 7 cache-line touches. Do not re-attempt it. The section is kept only for historical context.

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
