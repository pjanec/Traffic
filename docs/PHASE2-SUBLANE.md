# PHASE2-SUBLANE.md — laneless / sublane lateral model: progress + strategic checkpoint

Phase 2 adds SUMO's **sublane** lateral model (`MSLCM_SL2015`, activated by
`--lateral-resolution > 0`) on top of the byte-identical lane-based core. It rides the four
seams (`docs/DESIGN.md`) and stays **inert-when-absent**: every phase-1 scenario has
`lateral-resolution = 0`, so the global `_sublane` gate is off and the whole subsystem is dead
— the determinism hash stays `909605E965BFFE59` and all pre-existing goldens are byte-identical.

The bar is the same as phase 1: **exact 1e-3 against real SUMO 1.20.0 goldens**, now including a
new `posLat` attribute (sublane movement is deterministic at `sigma=0`). The harness compares
`posLat` when a scenario lists it; SUMO omits `posLat` from the default FCD set, so sublane
goldens are regenerated with `--fcd-output.attributes …,posLat` (wired into `regen-goldens.sh`).

## Landed rungs (exact @1e-3, committed, byte-identical for phase 1)

- **P2.0 — lateral comparison harness.** `posLat` threaded through `FcdParser` /
  `ToleranceConfig` / `TrajectoryComparator` / `TrajectoryPoint` / the engine export. Self-tested
  (compares + localizes a lateral offset; ignores `posLat` when a scenario doesn't list it, so
  phase-1 goldens can't be perturbed).
- **P2.1 — lateral state + gate.** `Kinematics.LatSpeed` (additive); `ScenarioConfig
  .LateralResolution` parsed from `<lateral-resolution>`; the global `_sublane` master switch.
- **P2.3 — single-vehicle lateral drift** (`scenarios/60-sublane-drift`). `latAlignment="right"`
  drift to the lane edge, exact incl. the `getWidth()=width+NUMERICAL_EPS` detail
  (`−1.4995`, not `−1.5`). `Engine.ComputeSublaneLateral` (target from `MSLCM_SL2015.cpp:1891-1902`
  + `DriftToward` bounded by `maxSpeedLat`); vType `maxSpeedLat`/`latAlignment` resolved to SUMO
  defaults (1.0 / "center").
- **P2.2a — side-by-side coexistence + `departPosLat`** (`scenarios/61-sublane-sidebyside`). A
  slow leader (left sublane) and a fast follower (right sublane) coexist with disjoint footprints,
  so the follower free-flows past with no braking — the defining sublane-vs-lane difference.
  Adds `departPosLat` placement (`Engine.InitialLatOffset`) + the small-`latDist` drift-skip
  threshold (`MSLCM_SL2015.cpp:1924`). Needs NO new leader logic — the existing `FootprintsOverlap`
  same-lane leader bypass handles one non-overlapping leader.
- **Coverage: sublane following, too-narrow → no overtake** (`scenarios/62-sublane-overtake`).
  On a 4.8 m lane there is no room to pass, so SUMO keeps plain Krauss following (posLat 0). The
  CURRENT engine reproduces it with NO new code — validates the drift code is inert for a
  centered-following pair.

## STRATEGIC CHECKPOINT — P2.4 sublane speed-gain overtake (DEFERRED, characterized)

**Anchor:** `scenarios/63-sublane-overtake-wide` (committed golden). On a 7.2 m lane a held-up
follower overtakes by drifting to the right sublane, then recenters:
`posLat 0→−1→−2→−2.6995 (held) → −1.6995→−0.6995→0`, with a brake-while-drifting speed profile
(`9.366/7.366/6.183`) and post-clear acceleration to 13.89.

A faithful minimal-slice port of `_wantsChangeSublane`'s speed-gain branch (attempted) reproduces
this **almost** exactly — **pos/speed to ~3e-7 for all 40 steps, and `posLat` EXACTLY through
t=13** (the entire trigger → drift → brake → hold). Findings from the source read
(`MSLCM_SL2015.cpp` `_wantsChangeSublane` 1067-2029, `computeSpeedGain` 2224, `updateExpected
SublaneSpeeds` 2087; `MSLeaderInfo::getSubLanes`):
- **direction** = full right edge: the per-sublane loop iterates rightmost-ascending and ties
  (`==maxGain`) don't overwrite `latDist`, so the first (rightmost) equally-free sublane wins;
- **trigger** = the first step ego's speed drops below free acceleration (the follow constraint binds);
- **grid quantization** = exact multi-vehicle sublane following needs `MSLeaderInfo::getSubLanes`'s
  discretized `[rightmost,leftmost]` sublane-index occupancy, NOT a continuous overlap test (at an
  intermediate `posLat` the continuous test says "clear" while the grid cell still overlaps — this
  is what makes the exact brake profile).

**Two residuals block byte-exact parity, both needing the deeper SL2015 machinery:**
1. the follower's recenter fires **one step late** (release wants the true per-sublane
   expected-speed grid `updateExpectedSublaneSpeeds`, not a single-scalar EMA proxy);
2. the **leader wiggles laterally** `0→0.3015→0` as it is passed — an unmodeled `keepLatGap` /
   `minGapLat` lateral gap-keeping reaction (a genuine two-vehicle lateral coupling).

The incomplete port was **reverted** (kept the tree clean; the branch stays green with the P2.4
test `[Fact(Skip=…)]`, mirroring the pre-existing C4-vii deferred rung). No non-exact golden is
committed as passing.

### The decision this checkpoint frames (per DESIGN.md phasing)
Byte-exact **multi-vehicle sublane decisions** (overtake, and the leader's gap-keeping) require the
SL2015 core: the **per-sublane leader grid (`MSLeaderInfo`) + `updateExpectedSublaneSpeeds` +
`keepLatGap`**. That is the natural next investment and the "complete full SL2015 vs. cap" fork:
- **Continue:** port `MSLeaderInfo` (the sublane grid — also the foundation for P2.2b and P2.5) +
  the expected-speed grid + lateral gap-keeping; then P2.4 becomes exact and P2.2b/P2.5 follow.
- **Cap:** bank the validated exact lateral core (single-vehicle drift, coexistence, placement,
  following-parity) as phase-2's deliverable; multi-vehicle sublane *decisions* stay characterized.

The highest-risk tail (cooperative `inform*` signaling) remains the push-vs-ECS concern flagged in
the original plan — to be re-cast push→pull (as B6 did with `MSDevice_Bluelight`) or capped.

## Core-port investigation (2 faithful attempts) — the definitive scope finding

The core port was attempted seriously, decomposed into isolated golden-anchored sub-mechanisms:

- **`keepLatGap`** (lateral minGapLat maintenance) — anchor `scenarios/64-sublane-keeplatgap`
  (a pinned leader; the passer nudges `−0.3015` and returns). The **magnitude mechanism is SOLVED
  and exact**: the `−0.3015` is a 0.6 m `minGapLat` measured against v0's **grid-quantized nearest
  sublane-column boundary** (`MSLeaderInfo::getSubLanes`), not its continuous physical edge —
  `gap = |4.8 − 3.6| − 0.9015 = 0.2985`, `surplusGapLeft = 0.2985 − 0.6 = −0.3015` (the continuous
  edge gives `+0.2985`, wrong sign). The port matched pos/speed to ~1e-13 for all 40 steps and kept
  scenarios 60/61/62 byte-exact + the hash held — but **diverged on the HOLD DURATION** (snap-back
  at step 7 vs. the golden's hold through step 11).
- **P2.4 speed-gain overtake** — `posLat` exact through t=13, residual = recenter one step late.

**Both attempts converge on the same root cause:** the hold/release *timing* is governed by SUMO's
**persistent cross-step lateral state** — `mySafeLatDistRight`/`mySafeLatDistLeft` (seeded in
`prepareStep`, updated by `checkBlocking`'s own `updateGaps`, decremented by travelled lateral
distance each step) plus `updateExpectedSublaneSpeeds`. Per-step-recomputed slices get the
*magnitude* exact but not the *duration*, because exactness is **emergent from the full stateful
machine**, not any single formula.

**Verdict:** byte-exact multi-vehicle sublane **decisions** require porting SL2015's full persistent
lateral state machine as a cohesive unit (`MSLeaderInfo` grid + `updateGaps` +
`mySafeLatDist*` state + `updateExpectedSublaneSpeeds` + the change decision) — a large, dedicated
effort, not incremental sub-rungs. The magnitude/geometry mechanisms are now understood and
documented; the remaining work is the state machine. Non-exact partial ports were reverted (iron
law); the inert `minGapLat` vType plumbing is retained as ready groundwork.

**Phase-2 delivered (exact @1e-3, committed, byte-identical for phase 1):** the single-vehicle and
static-multi-vehicle lateral core — P2.0/P2.1 (harness+gate), P2.3 (drift), P2.2a (coexistence +
`departPosLat`), following-parity coverage. The dynamic multi-vehicle *decisions* (overtake,
gap-keep hold) are characterized with their mechanisms solved, pending the state-machine port.
