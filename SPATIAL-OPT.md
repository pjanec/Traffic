# SPATIAL-OPT.md — design for spatial (cache-local) parallelization of the plan phase

**Status: DESIGN / not yet built.** This is the one lever left to attack the hot-path memory-bandwidth
wall (see `PERF-HANDOVER.md` — the ON-TARGET SESSION LOG). It is **research**: a projected win with a
real chance of not panning out, so it is designed prove-or-kill with a cheap probe and hard kill
criteria. Read `PERF-HANDOVER.md` first — this assumes its diagnosis and its list of what already failed.

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
