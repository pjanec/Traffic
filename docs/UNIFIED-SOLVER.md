# UNIFIED-SOLVER.md — design of record for the unified two-population solver

**Status: PROPOSED — awaiting owner go-ahead before the committed lane/bridge path is re-routed.**
This document is the approval artifact. A gated *standalone prototype* (a parallel driver that does NOT
replace the committed path) may be built and validated ahead of approval; wiring it into the shipped
coupling is the architectural commitment this doc asks you to authorize.

**Prototype status (Q5 — LANDED, standalone).** `src/Sim.Core/Unified/UnifiedWorld.cs` implements this
design's two load-bearing claims and validates them on the canonical hard case (`UnifiedSolverTests`):
the single joint plan/execute (both regimes read the same frozen start-of-step snapshot, §4) and the
parity escape hatch (sub-stepping the vehicle's crowd-reaction, §5). Measured on the exact crossing the
shipped bridge documents as its residual (a 13.9 m/s vehicle, a pedestrian crossing perpendicular at
`dt=1`): with a SINGLE per-step re-plan the vehicle footprint grazes the pedestrian by −0.21 m (overlap);
sub-stepping the reaction 8× turns it collision-free (+0.77 m clearance) while the vehicle still drives
through, and the pedestrian is mutually deflected in the same step. It embeds the real `OrcaCrowd` (so it
inherits Q1 static obstacles + Q3 spatial hash) and is NOT reachable from any golden (hash unaffected).
It deliberately uses a simplified vehicle *longitudinal* model (safe-speed follower, not the parity
Krauss reduction) — the prototype proves the COUPLING architecture; the real integration (still pending
your go-ahead) reuses the Engine's exact longitudinal reduction. The §10 questions remain open.

Read alongside: `docs/LANELESS-DIRECTION.md` (the bar decision + the staged plan), `docs/LANELESS-HANDOFF.md`
(state snapshot; residual #1 is the problem this solves), and the existing bridge
(`src/Sim.Core/Bridge/CrossRegimeCoupling.cs`).

---

## 1. The problem this solves

The cross-regime bridge (`CrossRegimeCoupling`) makes lane vehicles and open-space crowd agents avoid
one another, but with a **documented residual** (LANELESS-HANDOFF §residuals, LANELESS-DIRECTION stage 6):

> A close-range *perpendicular* crossing at the lane sim's fixed `dt=1` can still graze: the vehicle
> re-plans its lateral only once per lane step and each regime reads the other's *previous*-step state
> (one-step latency).

Two distinct defects hide in that sentence:

1. **Asymmetric latency.** `CrossRegimeCoupling.Step()` runs `_engine.Run(1)` (the vehicle plans against
   the crowd's *previous committed* state — Direction B) and *then* steps the crowd against the vehicle's
   *freshly moved* discs (Direction A). So the crowd reacts to the vehicle's current position but the
   vehicle reacted to the crowd's stale position. The coupling is not a clean plan/execute over a single
   frozen world — it is two sequential engines with a half-step skew.

2. **Single lateral re-plan per `dt`.** The parity core cannot be sub-stepped (iron law: the committed
   no-crowd trajectory must stay byte-exact), so a vehicle travelling `speed·dt` metres in one step makes
   exactly one lateral decision over that whole sweep. A fast vehicle "teleports" across a cell between
   decisions.

The bridge closed most of this (lateral prediction of a crosser at time-to-encounter; the
`CrowdLongitudinalConstraint` brake for a blocker you can't swerve around; optional crowd sub-stepping for
Direction A). What remains is the lack of a **continuous cross-regime collision guarantee** in the worst
case: a pedestrian essentially on a fast vehicle's path at close range.

---

## 2. Goal and non-negotiable constraints

**Goal.** A single shared per-step solve that sees *both* populations in one neighbourhood, eliminates the
asymmetric latency, and gives a collision guarantee whose only residual is *physically* unavoidable (a
pedestrian appearing under the wheels within one step) rather than an artefact of the coupling structure.

**Constraints (in priority order):**

1. **The iron law — parity of the committed path.** The determinism hash stays `909605E965BFFE59` and every
   committed golden stays byte-identical. This is achievable *structurally*: no committed scenario attaches
   a crowd, so **the entire unified path is gated behind "a crowd is present"** exactly as the bridge is
   today (`CrowdSource != null`). The no-crowd trajectory is produced by the unchanged `Engine.Run`.
2. **The parity escape hatch (the key insight — see §5).** The iron law pins the *no-crowd* trajectory
   only. The crowd-coupled path is **behavioural, has no golden**, and therefore *may* do things the
   committed path may not — including **sub-stepping the vehicle's reaction to the crowd**. This is what
   makes a real guarantee reachable without violating parity.
3. **Determinism.** Plan/execute double buffer, fixed iteration order, seeded per-entity RNG only, no
   wall-clock. Single-thread and parallel must agree bit-for-bit.
4. **Longitudinal parity of car-following even under coupling.** When a crowd is present but far, a
   vehicle's longitudinal motion must still be the validated Krauss/constraint reduction — we do not
   re-implement car-following; we *reuse* the engine's exact reduction and merely add crowd agents as
   extra constraints (the `CrowdLongitudinalConstraint` already does this).

---

## 3. Data model

A single world-space **footprint agent** set, unified across regimes, indexed by an int handle (no
strings — consistent with the SoA `ObstacleStore` just merged in, and its reserved `AvoidanceClass` byte):

```
UnifiedAgent {
    Vec2   WorldPos;        // world x/y (front-centre for a vehicle, centre for a crowd disc)
    Vec2   WorldVel;
    double Radius;          // crowd disc radius, or the vehicle's per-disc half-width
    Regime regime;          // Vehicle | Crowd
    AvoidanceClass klass;   // OneSided | StaticBlocker | Reciprocal  (drives the responsibility split)
    // Vehicle-only back-references (null for crowd):
    int    vehicleHandle;   // into the lane engine
    LaneRef lane;           // lane + arc-length, for the DOF projection
}
```

- A **crowd agent** is one disc, holonomic (both axes free).
- A **vehicle** is a *restricted-DOF* agent: its motion is `longitudinal along the lane tangent`
  (governed by Krauss car-following — its only "brake" axis) plus `lateral within the lane`
  (feasible-interval / bounded by `maxSpeedLat`). It is represented to the *crowd's* ORCA as the same
  short chain of world discs the bridge already builds (`CrossRegimeCoupling.BuildVehicleDiscs`), so the
  crowd sees a faithful elongated footprint, not a single fat disc.

**Shared spatial index.** A uniform grid / spatial hash over world x/y (the Q3 crowd hash, generalised to
hold both regimes). Each mover queries one neighbourhood and receives both crowd discs and vehicle discs.
This is also the perf axis — it removes today's `O(n²)` crowd scan and the per-ego lane radius scan.

---

## 4. The per-step algorithm (single joint plan/execute)

Replace `Run(1)`-then-`crowd.Step()` with **one** plan/execute step over the unified set:

```
Step(dt):
  # FREEZE — one snapshot of the whole world at step start
  freeze WorldPos/WorldVel of every UnifiedAgent (both regimes)

  # PLAN — every mover computes its intent from the SAME frozen snapshot (no asymmetric latency)
  for each crowd agent c:            # holonomic
      neighbours = grid.query(c)     # crowd discs + vehicle discs, all frozen
      newVel[c] = ORCA(c, neighbours, prefVel_toward_goal)   # reciprocal share by AvoidanceClass
  for each vehicle v:                # restricted DOF — REUSE the engine's exact reduction
      longitudinal: Krauss + the engine's constraint set, INCLUDING frozen crowd agents as
                    CrowdLongitudinalConstraint leaders (already implemented)
      lateral:      feasible-interval solve over frozen lane vehicles + frozen crowd agents projected
                    to the lane (ComputeRvoLateral crowd loop, already implemented)
      newLong[v], newLat[v] = that plan

  # EXECUTE — commit all intents together, integrate positions
  commit crowd velocities + integrate
  commit vehicle long/lat + integrate (via the engine's normal execute/command-buffer)
```

Because *both* regimes plan against the same frozen snapshot and write only their own intent, the
asymmetric latency of §1.1 is gone: it is a textbook plan/execute over a heterogeneous agent set.

**Reciprocity is DOF-aware (the new bit).** ORCA's reciprocal share assumes both agents can move in any
direction. A vehicle can't — it can brake (longitudinal) and swerve a little (lateral), but it cannot
side-step arbitrarily. So the responsibility split is **per-axis**:

- In the **lateral** axis, the vehicle *can* take its share → a crowd agent treats a vehicle as
  `Reciprocal` (share ≈ 0.5) laterally.
- In the **longitudinal** axis, the vehicle's only move is braking, bounded by its deceleration → the
  crowd treats the vehicle as `OneSided` longitudinally (the crowd yields fully along that axis) and the
  vehicle contributes what braking it can via `CrowdLongitudinalConstraint`.

This is expressed through the `AvoidanceClass` byte plus a small per-axis responsibility rule in the
unified ORCA, not by forcing one avoidance model onto both regimes.

---

## 5. The collision guarantee and its honest bound

A single non-iterated reciprocal pass at `dt=1` still cannot guarantee zero grazing when a crowd agent is
within `speed·dt` of a fast vehicle's path: the vehicle's brake is deceleration-bounded and its swerve is
`maxSpeedLat`-bounded — both physical limits.

**The escape hatch (constraint §2.2) delivers the real guarantee.** Parity pins only the *no-crowd*
trajectory. When a crowd is present the path is behavioural, so we may **sub-step the vehicle's
crowd-reaction** without touching the committed golden:

- The vehicle's **longitudinal integration** stays at `dt` when uncoupled (parity), but its
  **lateral/brake decision *with respect to the crowd*** is re-evaluated `K` times per lane step against
  the sub-stepped crowd positions. This is not sub-stepping car-following (which would change the
  no-crowd trajectory); it sub-steps only the *reaction to the crowd*, which is zero when no crowd is
  present → byte-identical committed path.
- Concretely: within one lane step, advance the crowd `K` sub-steps (dead-reckoning vehicle discs along
  their velocity, as the bridge's `SubSteps` already does for Direction A), and at each sub-step let the
  vehicle re-clamp its lateral offset and its longitudinal brake against the crowd's current sub-step
  position. The net vehicle displacement over the step is still Krauss-consistent; what improves is that
  the vehicle's swerve/brake *tracks* the crossing agent continuously instead of once.

**Resulting bound (to state plainly in validation):** with sub-stepping, the coupling is collision-free
for any crossing whose closing geometry the vehicle can physically resolve within its decel + `maxSpeedLat`
envelope; the only residual is the physically-unavoidable case (an agent entering the swept footprint
faster than any brake/swerve could clear). That is the correct place for the guarantee to end.

---

## 6. Determinism

- Plan/execute double buffer over the unified set; fixed order (regime-major, then handle-ascending).
- The spatial hash yields neighbours in a deterministic order (sort by handle within a cell, or iterate
  cells in fixed order) — required so the ORCA line order (hence the LP result) is reproducible.
- No `System.Random`; the symmetry-break jitter stays the existing deterministic per-`(agent,step)` hash.
- Single vs parallel plan must be bit-identical (the lane engine already gates parallel plan; the crowd
  plan is embarrassingly parallel and order-independent by construction).

---

## 7. Validation plan (behavioural — no SUMO golden exists)

1. **Parity guard (the hard gate):** with no crowd attached, `dotnet test` stays 278/3/0 and the hash stays
   `909605E965BFFE59`. Structural, because nothing committed reaches the unified path.
2. **Latency elimination:** reproduce the grazing scenario the bridge documents (close-range perpendicular
   crossing, fast vehicle). Assert the unified solver keeps footprints disjoint at every (sub-)step where
   the bridge grazes. This is the headline behavioural win.
3. **Regression vs the bridge:** the existing `OrcaCrossRegimeBridgeTests` cases (car swerves ~1.9 m for a
   person; car stops for a 5 m blocker; pedestrian routes around a parked car) must all still hold under
   the unified driver — same or better clearances.
4. **Reciprocity correctness:** a cooperative crowd agent and a vehicle each take a share laterally (the
   vehicle swerves *less* than under the one-sided double-yield, the crowd a bit more) — assert the split.
5. **Determinism:** bit-identical single vs parallel; bit-identical run-to-run.
6. **Scale/perf smoke:** N crowd + M vehicles via the shared hash completes within a sane budget and stays
   deterministic (not a parity gate, a sanity one).

---

## 8. Rollback / gating

- The unified solver is a **new driver** (`src/Sim.Core/Unified/`), selected only when a crowd is attached;
  the default (no crowd) path is the unchanged `Engine.Run`. Removing/disabling the driver reverts to the
  bridge with zero effect on committed behaviour.
- Ship it *behind* the existing bridge, not replacing it, until §7.2–§7.4 are green: a
  `CrossRegimeCoupling.Mode ∈ { LockstepBridge (default), Unified }` switch, default `LockstepBridge`, so
  the merge is inert until explicitly opted in.

---

## 9. Relationship to the existing bridge and prior work

- **Reuses, does not rebuild:** `BuildVehicleDiscs` (vehicle → world discs), `CrowdLongitudinalConstraint`
  (brake for a blocker), the `ComputeRvoLateral` crowd loop (swerve + lateral prediction), `OrcaSolver`
  (the 2D reciprocal solve + the Q1 obstacle lines), and the Q3 spatial hash (generalised to both regimes).
- **Changes:** the *coordination structure* — from sequential `Run(1)`+`Step()` with a half-step skew to a
  single joint plan/execute over a unified, spatially-indexed agent set, with DOF-aware reciprocity and the
  parity-safe crowd-reaction sub-stepping.

---

## 10. Open questions for the owner (please answer before §Q5 wiring)

1. **Reciprocity default:** should a vehicle be `Reciprocal` (swerves its half-share for cooperative crowd
   agents) or stay `OneSided` (crowd yields fully, today's conservative double-yield)? Recommendation:
   `Reciprocal` laterally, `OneSided` longitudinally (§4), but this changes vehicle trajectories under
   coupling, so it's your call.
2. **Sub-step count `K`:** the guarantee strength vs cost knob (§5). Recommendation: default `K=4`
   (matches the bridge's `SubSteps` intent), tunable.
3. **Replace or coexist:** do you want the unified driver to *replace* the lockstep bridge once validated,
   or to remain an opt-in `Mode` indefinitely? Recommendation: coexist behind the `Mode` switch until it
   has soaked, then flip the default.
4. **Scope of "vehicle":** unify only laneless-RVO vehicles (today's `LanelessRvo && _sublane` gate), or
   also normal lane-mode vehicles (the Q6 question)? Recommendation: start with laneless-RVO vehicles; the
   normal-mode swerve is a separate product decision.
