# LANELESS-HANDOFF.md — continuation handoff for the laneless / sublane effort

**Branch:** `claude/sumo-phase-2-planning-p3w7kh` (off `main`, never on `main`).
**Scope:** phase-2 laneless/sublane lateral movement — SUMO sublane parity where cheap, and our
own continuous velocity-obstacle (RVO/ORCA) model where SUMO's exact bar is the wrong fit.
**Read first, in order:** `docs/LANELESS-DIRECTION.md` (the bar decision + the staged plan, the
authoritative design record) → `docs/PHASE2-SUBLANE.md` (the byte-exact sublane findings) →
`/root/.claude/plans/*` phase-2 plan (the original prove-or-kill ladder) → this file (state + next).

This file is the **state snapshot + next-step map**. The design *rationale* lives in
`LANELESS-DIRECTION.md`; don't duplicate it here — this is "where things stand and what to do next."

---

## The one-paragraph state

The lane-based phase-1 core stays **byte-exact to SUMO** (determinism hash `909605E965BFFE59`,
untouched). On top of it, gated entirely behind opt-in switches, sits a laneless axis with **two
regimes**: (1) lane-derived vehicles do a **1D lateral feasible-interval** solve + Krauss
car-following (`Engine.ComputeRvoLateral`, opt-in `Engine.LanelessRvo`); (2) open-space holonomic
crowd/navmesh agents do **full 2D reciprocal ORCA** (`src/Sim.Core/Orca/`). A **cross-regime bridge**
(`src/Sim.Core/Bridge/`) makes the two mutually avoid (a car swerves for a pedestrian, and stops for a
blocker it can't swerve around; a pedestrian routes around a car). Everything is **behaviourally /
statistically** validated (no byte-exact bar for the laneless model — see the "why" in
`LANELESS-DIRECTION.md`), and everything is **byte-identical when its switch is off** by construction.

**Test state at handoff:** `dotnet test` → **256 passed / 3 skipped / 0 failed**; determinism hash
**`909605E965BFFE59`** (single == parallel). The 3 skips are the SL2015-core anchors (see below).

---

## Non-negotiable invariants (do not break these)

1. **The determinism hash stays `909605E965BFFE59`.** Every laneless feature is gated: with its switch
   off (`LateralResolution == 0` → `_sublane` false; `LanelessRvo` false; `CrowdSource == null`) no new
   branch is entered, so byte-identity is *structural*, not tested-into-existence. After ANY engine
   edit run `dotnet run -c Release --project src/Sim.Bench` and confirm the hash.
2. **`dotnet test` never invokes SUMO** and must pass on a fresh VM with no SUMO (CLAUDE.md). SUMO is
   only for authoring/regenerating goldens and direct diagnosis.
3. **Do NOT depend on the string `ExternalObstacle` API** (owner constraint: it must be replaced by a
   scalable int-indexed store — no string ids, high agent counts). The laneless layer talks to the
   world through the neutral value-typed seams **`RvoNeighbor`** (lane regime) and **`WorldDisc` /
   `ICrowdFootprintSource`** (open-space + bridge). `RvoNeighbor` is the **frozen seam** with the NuGet
   packaging branch (`claude/sumo-csharp-nuget-strategy-4vlkki`) — see the coordination section in
   `LANELESS-DIRECTION.md`, status CLOSED.
4. **Follow the plan/execute double buffer.** Every solve reads FROZEN start-of-step state and writes
   only the ego's own intent. No `System.Random`; no `Date.now`/wall-clock — determinism is required.

---

## What is built (by stage) — files, switches, tests

All lane-regime work is gated behind `--lateral-resolution R > 0` (parsed to
`ScenarioConfig.LateralResolution` → the `_sublane` master switch), plus `Engine.LanelessRvo` for the
RVO path. Commits are on the branch, newest-last in `git log main..HEAD`.

### A. Byte-exact sublane anchors (the cheap, kept-exact rungs) — `parityMode="exact"` @1e-3
- **P2.0** lateral harness: `TrajectoryPoint.PosLat`, `FcdParser`/`ToleranceConfig`/`TrajectoryComparator`
  parse+compare `posLat`. **P2.1** lateral state + gate: `Kinematics.LatSpeed`, vType
  `MaxSpeedLat`/`LatAlignment`/`MinGapLat`, `ScenarioConfig.LateralResolution`.
- **P2.2a** side-by-side coexistence + `departPosLat` (`scenarios/61`), **P2.3** single-vehicle drift to
  `latAlignment` (`scenarios/60`), **P2.2** following-parity (`scenarios/62`). Engine:
  `ComputeSublaneLateral`, `InitialLatOffset`.
- **Deferred (SKIPPED tests, empirical anchors committed):** **P2.4** sublane speed-gain overtake
  (`scenarios/63`, `RungP24SublaneOvertakeTests` [Skip]) and **keepLatGap hold** (`scenarios/64`,
  `RungP2CoreKeepLatGapTests` [Skip]). Two faithful ports each nailed the *magnitude* and missed the
  *hold duration* — exactness is emergent from SUMO's **persistent lateral state machine**
  (`mySafeLatDist*` + `updateExpectedSublaneSpeeds` + push-based `inform*`), which is ECS-hostile.
  Diagnosed in `docs/PHASE2-SUBLANE.md`. These are the 3 skips (the 3rd is an unrelated multilane
  junction skip).

### B. Lane-regime RVO model (our own, behavioural) — `Engine.LanelessRvo`, opt-in
- `Engine.ComputeRvoLateral` + `RvoNeighbor` (frozen seam) + `FeasibleClosestTo`/`LateralFeasible`
  (the 1D lateral feasible-interval / half-plane solve; conflict-correct — resolves neighbours that a
  push-sum would strand). Longitudinal stays Krauss car-following (the two-regime insight).
- Tests: `RungRvoLateralPocTests`, `RungRvoMultiNeighborTests`, `RungRvoConflictingNeighborsTests`,
  `RungRvoAgentUnificationTests`. **Stage 4 statistical cross-check:**
  `RungRvoStatisticalCrossCheckTests` vs `scenarios/65-mixed-sublane` — RVO aggregate mean speed within
  0.34 % of a real SUMO sublane run, lateral spread 84 % of SUMO's (golden is an aggregate reference
  only, no exact compare).

### C. Open-space 2D ORCA (the second regime) — `src/Sim.Core/Orca/`, fully standalone
- `Vec2`, `OrcaSolver` (faithful port of van den Berg et al. "Reciprocal n-Body Collision Avoidance" —
  ORCA lines + LP1/LP2/LP3; ORCA is our reference here the way SUMO is for lanes), `OrcaCrowd` (SoA
  agent store, plan/execute double buffer, deterministic). `OrcaCrowd.SymmetryBreak` (default 0) is the
  deterministic per-`(agent,step)` jitter that breaks the measure-zero perfectly-symmetric deadlock.
- Tests: `OrcaOpenSpaceTests` (no-overlap incl. under the antipodal-circle deadlock, counter-flow,
  16-agent permutation crossing, head-on, crossing, mirror-symmetry, determinism).
- **Known:** at *maximal packing* (perfectly-antipodal circle) pure ORCA deadlocks — a convergence
  limit, never a safety one. `SymmetryBreak` resolves real, non-perfectly-symmetric inputs. **Q2
  update (measured):** `maxNeighbours` culling + removal-on-arrival were added (`OrcaCrowd.MaxNeighbours`
  / `RemoveOnArrival`) but do NOT resolve the antipodal circle at any `SymmetryBreak` up to 0.5 (the
  earlier "escape like RVO2" assumption is not reproducible); their proven value is faster draining of
  dense flows + a per-agent work bound (`OrcaConvergenceTests`).

### D. Cross-regime bridge — `src/Sim.Core/Bridge/`, gated
- `WorldDisc` + `ICrowdFootprintSource` (neutral world-space seam), `LaneProjection` (Sim.Ingest; the
  inverse of `LaneGeometry.PositionAtOffset`), `CrossRegimeCoupling` (lockstep coordinator: steps
  `Engine.Run(1)` + `OrcaCrowd` together, exchanges frozen world snapshots via the export-observer).
- Engine seams (both gated on `CrowdSource != null` → byte-identical): `Engine.CrowdSource` +
  the crowd loop in `ComputeRvoLateral` (Direction B, with **lateral prediction** of a crosser at
  time-to-encounter, à la B6); `Engine.CrowdLongitudinalConstraint` (the safety net — brake for a crowd
  agent you can't swerve clear of, mirrors `ObstacleConstraint`). `OrcaSolver.Agent.Responsibility`
  (0.5 reciprocal / 1.0 one-sided). `CrossRegimeCoupling.SubSteps` (finer crowd time, default 1).
- Tests: `OrcaCrossRegimeBridgeTests` — car swerves ~1.9 m around a person (Direction B, end-to-end);
  car brakes to a stop for a 5 m blocker it can't clear (longitudinal net); pedestrian routes around a
  parked car disc (Direction A). Fixture: `scenarios/_fixtures/bridge-crossing/`.

---

## Honest residuals / open limitations (don't re-discover these — they're characterised)

1. **No continuous cross-regime collision GUARANTEE.** A close-range *perpendicular* crossing at the
   lane sim's fixed `dt=1` can still graze: the vehicle re-plans its lateral only once per lane step
   (the parity core cannot be sub-stepped) and the two regimes couple with **one-step latency** (each
   reads the other's previous-step state). Realistic separations are safe (a person ≥14 m ahead clears
   by 1–2.3 m). Sub-stepping the crowd and the longitudinal brake close most of the gap but not this.
   **A true hard guarantee needs a single shared spatial/velocity solve over BOTH populations** (or
   sub-stepping the lane core, which parity forbids). This is the biggest open item — see next steps.
2. **ORCA vs a fast one-sided obstacle a slow agent cannot out-run** grazes (ORCA's best-effort LP3) —
   fundamental, not a bug.
3. **Perf is not addressed for laneless.** The RVO/ORCA queries are correctness-first brute-force
   radius scans (`O(k)` per ego, `O(n²)` crowd). A shared spatial hash is the perf axis, explicitly not
   built (gated off for every lane-mode scenario, so phase-1 perf is untouched — verify with the hash +
   `scripts/run-benchmarks.sh` if you touch the hot path).

---

## Next steps (recommended order)

1. **[BIG, needs owner go-ahead] Unified two-population solver** — the real "hard guarantee." Replace
   the one-step-latency lockstep + double-yield with a single solve that sees vehicles and crowd agents
   in one neighbourhood each step (shared spatial index over world x/y; vehicles constrained to their
   lane's longitudinal+lateral DOF, crowd holonomic). This is an architectural move, not an incremental
   rung — scope and confirm before starting.
2. **[medium] Open-space ORCA robustness** — static polygon obstacles (walls/navmesh boundaries) as
   ORCA obstacle lines (the LP3 already reserves `numObstLines`, currently always 0); optional
   `maxNeighbours` culling to make the antipodal circle converge like RVO2.
3. **[medium] Perf** — the shared spatial hash for both regimes (the SPATIAL-OPT groundwork is the
   start), gated so lane-mode stays byte-identical.
4. **[small] Wire the bridge into normal lane mode** — currently the vehicle side is under
   `LanelessRvo && _sublane`; a normal SUMO vehicle braking for a crowd agent via
   `CrowdLongitudinalConstraint` already works whenever `CrowdSource` is set (it is in the reducer,
   not gated on laneless), but the *swerve* (`ComputeRvoLateral`) is laneless-only. Decide whether
   normal-mode vehicles should also swerve for crowd agents (would use the B6 `ComputeLateralEvasion`
   path, which reads the string `_obstacles` — would need a non-string crowd hook there too).
5. **[optional] SL2015 full-state-machine port** — only if byte-exact dynamic sublane *decisions* are
   ever required. `docs/PHASE2-SUBLANE.md` scopes it; the magnitude/geometry mechanisms are already
   understood and reused as the RVO physics constants. Large, brittle, ECS-hostile — the explicit
   "what we do NOT do" in `LANELESS-DIRECTION.md`.

---

## How to verify (the gates, every change)

```
dotnet test                                        # 256 passed / 3 skipped expected
dotnet run -c Release --project src/Sim.Bench      # hash must stay 909605E965BFFE59 (single == parallel)
scripts/run-benchmarks.sh                          # only if you touched the lane-mode hot path (perf/no-stuck)
```
Golden authoring (rare, network + SUMO): `scripts/regen-goldens.sh`, then commit the goldens. The
laneless goldens use `--lateral-resolution R` + `sigma=0`; the RVO/ORCA behaviour has **no** SUMO
golden and is validated behaviourally/statistically.

## Coordination note (MERGE EXECUTED)

The NuGet packaging branch (`claude/sumo-csharp-nuget-strategy-4vlkki`) owns the SoA obstacle-store
redesign; this layer consumes it via `RvoNeighbor` / `WorldDisc`. Merge order agreed (their store
first, this adapter second), `RvoNeighbor` frozen. Full detail + the reserved
`avoidanceKind`/`Responsibility` per-agent bit in `LANELESS-DIRECTION.md` (coordination section, CLOSED).

**Done (this session):** merged `origin/claude/sumo-csharp-nuget-strategy-4vlkki` into this branch (their
`ObstacleStore` SoA store + handle API + `AvoidanceClass` byte, `Sim.LiveHost`, stepped read surface,
runtime demand, lifecycle events, async runner, geometry-3D). The string `AddObstacle(id, …)` creation
API is **gone** — replaced by `AddObstacle(GetLane(id), …)` (handle-based); the two laneless RVO tests
that used it were migrated. The Stage-3 RVO adapter in `ComputeRvoLateral` now reads the SoA store
transparently: `ObstacleStore.Values` materialises each live slot's columns as an `ExternalObstacle`
value, so the frozen `RvoNeighbor` seam consumes the SoA columns with **no solve change**. `RvoNeighbor`'s
reciprocity `share` is now sourced from the store's per-agent `AvoidanceClass` (Reciprocal→0.5, else 1.0)
— but note it is still **inert in the 1D lateral solve** (that solve is inherently one-sided; share is
consumed only by the open-space 2D ORCA path and reserved for the unified solver). Gates after merge:
`dotnet test` → **278 passed / 3 skipped / 0 failed**; determinism hash **`909605E965BFFE59`** (single ==
parallel). The 3 skips are unchanged (P2.4 overtake, keepLatGap, C4-vii multilane junction).
