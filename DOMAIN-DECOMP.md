# DOMAIN-DECOMP.md — spatial domain decomposition: overnight exploration + findings

**Status: region-parallel plan/willPass SHIPPED (byte-identical, modest); the big win needs a
major rewrite (scoped below). This records an exhaustive overnight exploration so a future dedicated
effort starts informed.** Read `PERF-HANDOVER.md` (the wall diagnosis + experiments log) and
`SPATIAL-OPT.md` (the spatial-store analysis) first — this continues them.

## The goal and the wall

Goal: break the ~3× hot-path ceiling toward ≥4× SUMO @8 cores. The confirmed bottleneck is
**memory bandwidth from RANDOM neighbor access**: the plan/willPass phases dereference each
follower's leader/foe `VehicleRuntime` at a random heap address (~1 cache-line miss per foe read),
which saturates the memory subsystem and caps parallel scaling at ~3×.

**The overnight finding, stated plainly:** in this architecture (`VehicleRuntime` is a C# *reference
type*; the GC controls placement, so a set of vehicles is scattered across the heap regardless of
how you partition the work) **every byte-identical memory/spatial restructuring is capped at
single-digit %, because the per-step cost of imposing spatial order (a gather or a sort) ~offsets
the sequential-access gain it buys.** This was shown repeatedly (below). The big win requires either
(a) a *value-type* contiguous store kept in stable spatial order (no per-step reorder), or (b)
fast-mode independent regions that parallelize the *serial tail* — both are major rewrites.

## What SHIPPED (byte-identical, committed) — region-parallel plan + willPass + execute + refill

`Engine.RegionPlan` (`--region [--region-grid G]`, off by default → deterministic path untouched,
hash `909605E965BFFE59`, 229 tests, city-3000 trip-SHA match + aggregate PASS). Active vehicles are
grouped into a G×G spatial grid over the lane-centroid bounding box; **each region owns a DISJOINT set
of lanes**, so its buckets/vehicles are touched by only its task — no locks. Dynamic scheduling over
regions balances occupancy (the "automatic chunk load management"). The KEY structural win: **vehicle
handoff between regions is FREE** — `BuildRegionActive` re-buckets by current lane every step, so a
vehicle that crossed into another region is simply grouped there next step; no explicit state transfer.

Region-parallelized phases (all byte-identical), measured @8t:
- **plan + willPass** (the bandwidth-bound phases) — modest per-phase (~3%); the scattered heap
  objects mean a region's vehicles are still N scattered cache lines, so the working-set benefit is
  partial.
- **`perf(region)` ExecuteMoves — execute 284 → 130 ms (−54%, ~2.2×).** Per-vehicle moves are
  independent; arrivals go through a now-thread-safe command buffer (a lock on the rare Destroy/
  ChangeLane appends) and apply order-independently at Flush. Gated on no actuated TLS.
- **`perf(region)` neighbour Refill — refill 248 → 175 ms (−29%).** Each region refills its disjoint
  lanes; `RefillRegion` is byte-identical to the serial Refill.

**Cumulative: ~8.7% wall @8t (region grid G24 = 576 regions — FINER grids win: smaller working sets +
better load balance; the sweet spot scales with vehicle count). From 3.06× → ~3.35× SUMO.** Real,
safe, byte-identical progress — but not yet 4×.

## The remaining lever to 4× (fast-mode) — NOT built (risk/return point)

The math: SUMO sim-tick ~17.2 s; tick @8t ~5.6 s (=3.06×); 4× needs ≤4.3 s. The serial tail is
~2.3 s @8t; **execute + refill are now region-parallel (byte-identical), but the biggest serial phase,
INSERT (~720–950 ms), is not** — it is order-dependent (cross-lane insertion safety) and races on the
shared `_laneSeqPool` (each insertion appends its lane sequence). Region-parallelizing it naively is **not just fast-mode but NON-DETERMINISTIC**, and that is the real
blocker: a follower approaching a junction from region B checks (via `TryInsertOnLane`'s cross-junction
scan) whether a leader on region A's lane has been inserted yet — so whether it inserts or waits
depends on the A-vs-B thread *timing*, giving run-to-run-variable trajectories. `--fast-gate` requires
a *deterministic* fast path (it diffs fast vs the deterministic baseline), so a non-deterministic
insert can't be validated that way. Making it deterministic needs a two-phase scheme (gather all
candidates, then insert in a deterministic cross-region order) that reclaims the parallelism only for
the per-candidate route-resolution/leader-scan work (~360 ms of insert's ~500–720 ms is the
per-insertion route resolution — parallelizable under a lock on the shared `_laneSeqPool` append),
NOT the ordering decision. That's the honest, bounded path: **region-parallel the per-insertion WORK
(deterministic, fast-mode where the cross-region ordering is approximated by a fixed region order),
not the ordering.** Combined with fast-mode speedGain/foeIndex, parallelizing the tail could still
approach ~4× — but it is delicate (determinism + the pool race) and was left un-built overnight to
keep the system working. The shipped region foundation (disjoint lanes, free handoff, thread-safe
command buffer, `RefillRegion`) is what it builds on.

## What was tried and FAILED (reverted — do not repeat)

- **Persistent write-through `_hot` store (EntityIndex-keyed) + BuildPacked = copy + sort.** Built the
  full write-through (`HotWrite` at insertion/ExecuteMoves-end; single-lane city-3000 needs no
  lane-change hook), verified byte-identical (trip-SHA). Goal: remove the 305 ms scattered gather.
  **Result: WORSE — `packed` rose to ~450 ms.** Root cause: the original gather reads from the
  neighbour buckets which are ALREADY pos-sorted (by Refill), so it needs NO sort; the persistent
  store loses that pre-sortedness and must re-sort each step, and the sort (even sorting a 4-byte
  index array by a primitive composite (lane,pos) key, then gathering) costs *more* than the gather
  it replaced. **Imposing (lane,pos) order costs ~the same whether by gather-from-sorted-buckets or
  by sort — it always ~offsets the plan's sequential-leader saving.**
- **Spatial packed probe (gather-built `_packed`, sequential leader read).** Validated the mechanism
  (plan −11%) but the 305 ms gather offsets it → net-neutral wall. (Committed gated-off as
  groundwork; see SPATIAL-OPT.md §0.)
- **Per-field SoA, AoS-by-EntityIndex, mirror-SoA-with-refresh, parallel foeIndex (locks):** all
  regressed or null — see PERF-HANDOVER.md's experiments log.

## Why "separate engine instances" (your idea) doesn't escape the wall cheaply — and where it WOULD win

Domain decomposition into independent chunks is architecturally the right way to scale a spatial
sim, and it's what the shipped `Engine.FastMode` + `--fast-gate` are for (it's not byte-identical to
SUMO — partitioning changes global insertion/junction/boundary ordering — so it's validated
*behaviorally*, not against goldens). Two honest points from the exploration:

1. **It does NOT give big memory-locality wins for free**, because a chunk's vehicles are still
   scattered C# heap objects. The region-parallel PLAN already tests intra-region locality and gets
   only ~2.6–6%. Full independence wouldn't dramatically beat that on the *memory* axis.
2. **Its REAL potential is parallelizing the 44%-of-tick SERIAL TAIL** (insert ~950 ms, speedGain
   ~391, foeIndex ~353, refill ~195, execute ~177 @8t) that byte-identical work *cannot* touch
   (order-dependent). Even 3× on ~2500 ms of serial work would save ~1600 ms → ~4× SUMO. THIS is the
   prize, and it needs fast-mode. **Nice property discovered:** vehicle handoff between chunks is
   FREE — `BuildRegionActive` re-buckets by current lane every step, so a vehicle that crossed into
   another region is simply grouped there next step; no explicit state transfer. The remaining work
   is per-region command buffers + a region-parallel (or shared read-only) neighbour query + making
   insert/junctions region-local with deterministic tie-breaks.

## The scoped real-breakthrough path (future dedicated effort — prototype-first, kill-gated)

Two independent big levers; do whichever, prototype-and-measure before committing:

**A. Fast-mode region-parallel WHOLE STEP (attacks the serial tail — highest ceiling).**
- Per-region command buffers (the shared `List<Command>` is not thread-safe); flush deterministically.
- Region-parallel refill (byte-identical, per-lane independent), execute (per-vehicle; gate on no
  actuated-TLS or make detectors thread-safe; arrivals/lane-changes are order-independent so flush
  order is irrelevant), speedGain and foeIndex (fast-mode, deterministic tie-breaks), and insert
  (fast-mode: per-region candidate processing; cross-region insertion order differs from global).
- Validate EVERY step with `--fast-gate` (0 gridlock, aggregate parity vs the deterministic baseline,
  no overlaps). Kill if it can't beat ~3× while staying gate-green.

**B. Value-type SEGMENTED persistent store (byte-identical — moderate ceiling ~7–8%).**
- Per-lane `HotVeh` segments (value type), write-through, kept in stable pos order (same-lane order
  never changes — no in-lane passing). A lane change / boundary cross is an O(1) move between
  segments. `_packed` = concatenation of segments each step — O(N) sequential copy, **no gather, no
  sort** (the thing that capped every flat approach). This is the only byte-identical way to get
  spatial order without the per-step reorder cost. Complexity: segment lifecycle + an
  EntityIndex→(lane,slot) map. Kill if the concatenation + segment maintenance doesn't beat the
  305 ms gather in an interleaved paired A/B.

**Bottom line:** the safe, byte-identical ceiling on this architecture is ~3× SUMO @8t (region-parallel
banks a few %). ≥4× is reachable only via A (fast-mode serial-tail parallelization — highest ceiling,
biggest build, parity-by-behavioral-gate) or B (value-type segmented store — byte-identical but
moderate). Both are multi-day, prototype-first efforts; neither was completed overnight to avoid
leaving the system half-built. `--fast-gate` (shipped) is ready to validate path A.
