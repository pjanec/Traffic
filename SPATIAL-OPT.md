# SPATIAL-OPT.md — design for spatial (cache-local) parallelization of the plan phase

**Status: PROBE BUILT + MEASURED (§8). Mechanism VALIDATED; naive version net-neutral on wall; the
real win needs a persistent store (below).** This is the one lever left to attack the hot-path
memory-bandwidth wall (see `PERF-HANDOVER.md` — the ON-TARGET SESSION LOG). Read `PERF-HANDOVER.md`
first — this assumes its diagnosis and its list of what already failed.

## 0. PROBE RESULTS (measured — read before the design below)

The §8 probe was built (opt-in `Engine.SpatialPlan` / `Sim.BenchCity --spatial`, off by default,
byte-identical — 229 tests, hash `909605E965BFFE59`, city-3000 default-vs-`--spatial` trip SHA match)
and measured @8t (interleaved, same build, flag toggle):

- **Mechanism VALIDATED:** the same-lane leader read from the adjacent packed slot (sequential) makes
  the **`plan` phase ~11% faster** (1952 ms vs 2191 ms), consistently (every spatial run < every base
  run). This is the **first thing all session to actually speed up the plan phase** — the
  sequential-leader hypothesis is real.
- **But WALL is net-neutral**, because building `_packed` costs **`packed` = 305 ms serial** (a
  per-step **random gather** of every vehicle's `Kinematics`/vType from scattered objects) ≈ the
  239 ms the plan saved. **Total scattered-read traffic is CONSERVED, just relocated** from the
  parallel plan to the (serial) build.
- **Parallelizing the gather REGRESSED it** (525 ms vs 305 ms @8t): the gather is bandwidth-bound
  scattered reads, so spreading them across threads only adds contention + dispatch over many empty
  lanes. There is no cheap shortcut — to build a (lane,pos)-sorted array you must read the data in
  some order, and it's random either way (bucket order ≠ EntityIndex order ≠ heap order).

**Sort-cost measurement (persistent-store viability):** adding a full `Array.Sort` of the CONTIGUOUS
`_packed` to `BuildPacked` cost only **~115 ms** (packed went 305 → ~420 ms). So a persistent store's
per-step build ≈ contiguous copy (~cheap, sequential) + sort (~115 ms) ≈ **~150 ms**, vs the current
**305 ms scattered gather** — roughly half. Projected full outcome (build −155 ms, plan −239 ms, and
willPass ~−280 ms if extended): **~370 ms wall ≈ ~6%** (3.06× → ~3.27× SUMO). Real, but modest — the
first genuine wall-mover, though NOT 4× on its own. (An INCREMENTAL re-sort could be even cheaper than
the full 115 ms sort, since same-lane pos-order is stable — no in-lane passing — so only structural
movers are out of order; that would push the build below ~150 ms.)

**Incremental-re-sort floor (measured):** an INSERTION sort by (lane,pos) over the already-ordered
`_packed` adds only **~25 ms** (packed 305 → ~330 ms) — vs Array.Sort's ~115 ms. So the sort itself is
nearly free on ordered data. **BUT that is the zero-mover FLOOR.** The real incremental case is
churn-dominated, and a **FLAT** (lane,pos) array handles lane changes badly: a vehicle crossing a lane
boundary (A→B, ~400×/step on city-3000 as vehicles reach lane ends) moves between lane *segments* — a
LARGE displacement in the flat array (lane B's handle is arbitrary vs A's), so insertion-sorting ~400
large displacements/step would be expensive. **The flat-array incremental re-sort's floor is cheap but
its real cost is churn-dominated.** The fix is a **SEGMENTED per-lane PERSISTENT structure** (a lane
change is an O(1) move between segments; same-lane order is stable so never re-sorts) concatenated into
`_packed` each step (O(N) sequential copy) — essentially making the neighbor buckets persistent +
incrementally maintained. That is the real design; its true per-step cost is only knowable by building
it. Net-gain estimate therefore spans **~6 % (flat, full Array.Sort) to ~8–13 % (segmented, if churn
cooperates), plus willPass** — real, bounded, but NOT 4×, and the segmented store is a substantial build.

**Verdict:** the sequential-leader mechanism works, but a **rebuild-from-scratch gather each step**
cannot win — it re-pays the exact bandwidth it saves. **A real win requires a PERSISTENT
spatially-ordered hot store**: keep `_packed` (or equivalent) sorted by (lane, pos) *across* steps and
**incrementally re-sort** only the few vehicles that changed relative order (moved past a neighbor /
changed lane) — so there is NO per-step random gather. The probe is kept (gated off, byte-identical)
as the validated groundwork (`HotVeh`, the packed leader read in `LeaderFollowSpeedConstraint`, the
spatial plan branch); the persistent store replaces only `BuildPacked`. That is the next, bigger step
— see §10 (added).

---

**Original design (below).** It is **research**: a projected win, prototyped prove-or-kill.

---

## 1. The problem, precisely

The plan phase (`willPass` + `plan`, ~59% of the tick) is **memory-bandwidth-bound on RANDOM neighbor
access**. Per vehicle per step, a follower dereferences its **leader/foe** `VehicleRuntime` at a
*random* heap address → ~1 cache-line miss per foe read. This does not parallelize past ~3× because the
memory subsystem saturates on these random gathers, and CPU sits at ~11% of 24 cores (cores stall).

**What already failed (do not repeat — see PERF-HANDOVER):**
- **Per-field SoA** (`_soaPos[]`, `_soaSpeed[]`, …): REGRESSED. The gap math reads ~7 fields of ONE foe
  (AoS-shaped); per-field SoA turns that into 7 cache-line touches.
- **AoS-struct-array indexed by EntityIndex** (`HotVeh[entityIndex]`): predicted ~neutral. It packs the
  foe into 1 cache line, but `entityIndex` is **random**, so it is still 1 *random* miss — no better than
  the object's `Kinematics` line.

The lesson: **the cost is the RANDOMNESS of the access, not the field layout.** Neither SoA nor AoS
helps while the index is random. The only thing that changes the game is making the leader access
**sequential/local in memory.**

## 2. The core idea — restore SUMO's lane-ordered locality with a contiguous array

**Key fact about car-following:** a vehicle's same-lane leader is the *next vehicle ahead on its own
lane*. If the hot per-vehicle data were stored **contiguously, sorted by (lane, position)**, then a
follower and its leader are **adjacent array slots**, and reading the leader is a **sequential** access
(prefetcher-friendly, cache-warm) instead of a random miss.

**This is what SUMO already does.** SUMO processes vehicles **lane-by-lane, front-to-back**, walking each
lane's ordered vehicle list; a follower's leader is the element it just visited — already in cache. Our
engine *broke* that locality by switching to a dense `EntityIndex`-ordered parallel loop over scattered
heap objects. The spatial optimization is: **re-create SUMO's lane-ordered processing locality, but over
a contiguous array instead of pointer-linked lists** — which also gives clean cache-local parallel chunks.

**Why this is different from the failed AoS-struct-array:** we do NOT index the packed array by the random
`EntityIndex`. We **iterate the plan in packed (lane, pos) order**, so consecutive iterations touch
consecutive slots and each ego's leader (`_packed[i+1]`) is already prefetched. AoS + sorted + sequential
iteration — all three together — is the combination none of the prior experiments had.

## 3. Data structures

```csharp
// One cache line (~56 bytes). Every field the SAME-LANE gap math reads off a foe, packed together.
// vType scalars are per-type; inline the few the gap math needs (cheaper than a second indirection).
readonly struct HotVeh {
    double Pos;         // 8   } the Kinematics struct is already contiguous in VehicleRuntime,
    double Speed;       // 8   } so gathering Pos also brings Speed/LatOffset into cache for free
    double LatOffset;   // 8   }
    float  Length;      // 4  \
    float  Decel;       // 4   > vType scalars LeaderFollowSpeedConstraint reads off the leader
    float  Width;       // 4  /
    int    EntityIndex; // 4   back-ref to the full VehicleRuntime (ego cold fields + intent write)
    int    LaneHandle;  // 4   to detect "leader is on a different lane" at a segment boundary
    // + a CACC bit (pack into a spare byte/flags int) if kept.
}

HotVeh[] _packed;       // rebuilt each step, sorted by (LaneHandle, Pos ascending). Contiguous.
int[]    _laneSegStart; // per lane handle: index of its first slot in _packed (its segment).
                        // A lane's vehicles occupy _packed[_laneSegStart[h] .. next lane's start).
```

Within a lane segment, ascending `Pos` ⇒ the leader of `_packed[i]` is `_packed[i+1]` **iff**
`_packed[i+1].LaneHandle == _packed[i].LaneHandle` (else `i` is the front vehicle of its lane — no
same-lane leader).

## 4. Step flow

**(a) Build `_packed` — piggyback on the neighbor Refill.** `LaneNeighborQuery.Refill` *already* touches
every active vehicle and sorts per-lane by `Pos`. Extend it to emit `_packed` in (lane, pos) order and
record `_laneSegStart`. The gather is **nearly free**: Refill already reads `v.Kinematics.Pos`, which
pulls the whole `Kinematics` (Pos/Speed/LatOffset — one contiguous struct) into cache, so Speed/LatOffset
cost nothing extra; the vType scalars come from the per-type `ResolvedVType` (one object, cache-resident
— city-3000 is one type). This is the crux that separates it from the failed mirror-SoA refresh (which
re-read scattered objects for *no* other reason).

**(b) Plan iterates `_packed` in order**, partitioned into **contiguous chunks** for parallelism. For
slot `i`:
- ego hot fields come from `_packed[i]` (sequential);
- the **same-lane leader** is `_packed[i+1]` when same lane (sequential, prefetched) — feed it straight
  into `LeaderFollowSpeedConstraint` (which reads exactly `HotVeh`'s fields);
- ego **cold** fields (RNG, stops, ACC/CACC state, LC bookkeeping) come from `_vehicles[_packed[i].EntityIndex]`
  — one random object touch **per ego**, read once, and skipped entirely for the common Krauss/no-stop
  case (those constraints early-out). Write `MoveIntent` back to that object.

**(c) Cross-lane / junction foes stay on the existing neighbor query.** `GetNeighborLeader/Follower`
(adjacent lanes) and `GetRearmost`/`FindFoeVehicle` (cross-junction) are NOT same-lane-adjacent; they keep
the current lookup (which can read `HotVeh` via the segment, ~1 random line — no worse than today). These
are the minority (near-junction / lane-changing vehicles); the bulk (same-lane car-following) gets the win.

## 5. Why this should reduce random traffic ~2–4×

Per vehicle, today's plan does ~1 random ego-object read **plus ~1–3 random foe-object reads** (leader +
junction foes). Spatial replaces the dominant **same-lane leader** read with a sequential `_packed[i+1]`
and streams ego hot fields from `_packed`. Remaining random access: ego cold fields (once, often skipped)
+ the minority cross-lane foes. Net: from ~2–4 random misses/vehicle down toward ~1 — and, critically,
**each parallel thread now streams a contiguous chunk of `_packed`** (a spatial region), so the memory
subsystem serves prefetched sequential reads that scale far better than random gathers. That is the
mechanism by which the parallel phases could climb past ~3× toward the 4× target.

## 6. Byte-identity

Preserved by construction:
- The plan writes only each ego's own `MoveIntent` (order-independent), so iterating in (lane, pos) order
  instead of EntityIndex order is byte-identical — exactly the argument the current parallel plan already
  relies on.
- `_packed[i+1]` (same-lane) is *the same leader* `GetLeader` returns (nearest ahead on the lane); the
  fields are the same start-of-step `Kinematics`/vType values. Verify with the standard checks: `dotnet
  test` (229), determinism hash `909605E965BFFE59`, and a **city-3000 `--serial` vs default tripinfo SHA
  match** (the junction byte-identity check — the highway hash does NOT exercise this).
- Plan/execute contract intact: `_packed` is built from **frozen start-of-step** state before the plan,
  read-only during the plan; `ExecuteMoves` writes new state after.

## 7. Risks — why it might not pan out (be honest)

1. **The gather might not be as free as hoped.** If filling `HotVeh` measurably adds to Refill (extra
   fields, sort of larger structs), it can eat the win. *Mitigation/measure:* Refill already sorts + reads
   Pos; measure Refill delta in isolation.
2. **Sorting 56-byte structs** moves more bytes than sorting refs. Could be a wash. *Measure* refill phase.
3. **Ego cold reads stay random.** The plan reads many things off ego across the constraint tree; only foe
   reads move to `_packed`. If ego-object traffic dominates (not foe traffic), the win shrinks. This is the
   biggest unknown — the diagnosis says *foe* access is dominant, but it's not separately measured.
4. **Only same-lane leader is sequential**; junction-dense scenarios (many cross-lane foes) benefit less.
5. **It's a medium-large change** touching Refill, the plan loop, and the leader constraint. The projected
   win is not measured. → cheap probe first (below).

## 8. Prototype (prove-or-kill) and kill criteria

**Minimal probe that tests the CORE hypothesis** (sequential same-lane leader + contiguous-chunk plan
iteration → better scaling), without the full migration:

1. In `Refill`, additionally build `_packed` (HotVeh sorted by lane,pos) + `_laneSegStart`. (Buckets stay
   as-is; this is additive.)
2. Add a plan variant that iterates `_packed` in **contiguous chunks** (`Partitioner.Create(0, _packed.Length)`),
   ego = `_vehicles[_packed[i].EntityIndex]`, and feeds the same-lane leader from `_packed[i+1]` into
   `LeaderFollowSpeedConstraint` (thread the `HotVeh` in; leave every other constraint unchanged, still
   using `neighbors`). Gate behind a flag so the default path is untouched.
3. **Verify byte-identical** (tests + hash + city-3000 trip SHA). This is non-negotiable — if it can't be
   made byte-identical, the probe is wrong, not "close enough."
4. **Measure** interleaved paired A/B @8t (build both, alternate runs): the decisive number is the **`plan`
   phase time** and **wall**, and — most importantly — the **plan phase's 1→8t scaling** (does it climb
   past the current ~3.4× ceiling?).

**KILL CRITERIA (decide fast, don't sink weeks):**
- If the probe is **not byte-identical** and can't be made so → kill.
- If the `plan` phase does **not** improve by a clear margin that beats the ~8% noise floor (say ≥10% at
  8t **and** better 1→8t scaling) in interleaved paired A/B → kill (the ego-object traffic or gather cost
  dominates; spatial reorder is not the answer here either).
- If it wins → promote incrementally: extend the packed reads to the other same-lane consumers, then the
  cross-lane foes via segment lookups, measuring each slice, keeping all three parity gates green.

**Fallback if killed:** the practical ceiling on this architecture is ~3× SUMO @8t for byte-identical work;
the only remaining option would be an aggressive opt-in fast-mode (validated by `--fast-gate`) that trades
parity for speed — a different bar, not a bandwidth fix.

## 9. Open design questions (decide before building the probe)

- **Iterate `_packed` directly, or keep the dense active list and thread a `slotMap[EntityIndex]`?** Direct
  packed iteration is required for the *sequential* win (a `slotMap` read is itself random). Favor direct.
- **Inline vType scalars in `HotVeh`, or a `VTypeIndex` + tiny table?** Inline is simpler and fits one
  cache line for the handful of fields the leader gap math needs; revisit only if `HotVeh` outgrows 64 B.
- **How to handle the willPass pre-pass** (it also calls the leader constraint): it reads the same
  start-of-step `_packed`, so it iterates packed order too — same treatment.
- **Chunk size / partitioner** for the parallel plan over `_packed` (contiguous ranges vs work-stealing):
  contiguous ranges are the point (cache locality); measure chunk size.

## 10. The persistent spatially-ordered store (the real fix — next step, bigger)

The §8 probe (§0 results) proved the sequential-leader read speeds the plan ~11%, but the per-step
gather that builds `_packed` re-pays that bandwidth. To actually bank the win, `_packed` must NOT be
rebuilt-by-gather each step — it must **persist across steps, already in (lane, pos) order**, so the
plan reads it with no random gather.

**Core idea.** Make the hot data (`HotVeh`, keyed by a *slot*, not EntityIndex) the store that
`ExecuteMoves` writes **in place** (write-through, like the reverted SoA but AoS + spatially ordered),
and keep it sorted by (lane, pos) by **incremental re-sort** each step:
- Vehicles move a little each step, so the array is **almost sorted**; only vehicles that (a) passed a
  same-lane neighbor or (b) changed lane are out of order.
- An **insertion-sort-style local repair** (or a per-lane merge of the handful of movers) fixes it in
  ~O(N + swaps), with swaps ≈ the small number of order changes — far cheaper than an O(N) random
  gather + full sort. Almost-sorted arrays are the best case for insertion sort.
- Lane changes / insertions / arrivals move a slot between lane segments — handle via the command
  buffer at step end (same discipline as structural mutations today).

**Why this removes the gather.** `ExecuteMoves` already computes each vehicle's new Pos/Speed and
already touches that vehicle — writing it into its (persistent) slot is free (no separate gather pass).
The plan then reads `_packed` sequentially. No per-step random read of scattered objects at all — the
hot data LIVES in the sorted array; the `VehicleRuntime` object is touched only for cold fields.

**Risks / unknowns (why it's still research):**
- The incremental re-sort cost must stay well below the gather it replaces. If churn is high (dense,
  lots of overtaking / lane changes), the swap count grows. Measure on city-3000.
- Slot identity vs EntityIndex: every site that today keys by EntityIndex (side tables, snapshot,
  command buffer) must map through a slot↔EntityIndex index, or the object keeps EntityIndex and only
  the hot store is slot-keyed. Design the seam carefully.
- It's a substantial change (a second source-of-truth store with its own lifecycle). Byte-identity must
  hold throughout (trip-SHA on a junction scenario every slice).

**Incremental path:** (a) add the persistent `HotVeh` store written-through at `ExecuteMoves`/insertion,
kept EntityIndex-keyed first (byte-identical, no perf change — proves the write-through). (b) Add the
(lane,pos) ordering + incremental re-sort, and point the plan's spatial branch at it instead of
`BuildPacked`. Measure the re-sort cost vs the gather it replaces. (c) If the wall improves and scales,
extend the packed reads to `willPass` (the bigger phase — another ~11% on ~2500 ms) and to the
cross-lane / junction foe lookups via segment offsets.

**Kill criterion:** if the incremental re-sort is not decisively cheaper than the 305 ms gather (i.e.
the wall still doesn't improve @8t in interleaved paired A/B, byte-identical), then the random-access
wall is unbeatable in this architecture for byte-identical work, and the remaining option is an
opt-in fast-mode (validated by `--fast-gate`), not a bandwidth fix.
