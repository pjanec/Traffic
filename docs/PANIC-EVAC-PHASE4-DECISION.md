# PANIC-EVAC-PHASE4-DECISION.md — sublane (filter-to-front): DEFERRED

**Decision (2026-07-14, owner-approved): defer Phase 4.** This doc records *why*, so it is not
re-litigated. Phase 4 (`PANIC-EVAC.md` §6.4 / R1) is the optional sublane "filter-to-front at red lights"
behaviour — scooters/cyclists using lateral freedom to advance past stopped cars to the stop line.

## The question that decided it
Does Phase 4 require replicating SUMO's sublane model (`MSLCM_SL2015`), and would enabling it endanger the
optimized non-sublane engine (≈3× single-threaded SUMO on multiple cores; target: 10k+ vehicle cities)?

## What SL2015 is here today (done vs deferred)
- **Done & byte-exact (committed):** the single-vehicle / static lateral *physics* only — lateral state +
  the `LateralResolution` gate, `departPosLat` placement, lateral drift/alignment (scenarios 60–62).
  `ComputeSublaneLateral` (`Engine.cs:6367`, ~37 lines) reads **only ego's own state** → cheap,
  parallel-safe. This is *drift*, not multi-vehicle decisions.
- **Deferred (exactly what filter-to-front needs):** the SL2015 decision core — `MSLeaderInfo` per-sublane
  leader grid, `updateExpectedSublaneSpeeds`, `keepLatGap`, and the persistent cross-step state
  `mySafeLatDistRight/Left`. Marked by two skipped tests (`RungP24SublaneOvertakeTests`,
  `RungP2CoreKeepLatGapTests`). A scooter sliding to the front is precisely this deferred logic — none of
  it is ported.

## Why it is hard (documented history)
- **Two faithful byte-exact attempts, both reverted** (`docs/PHASE2-SUBLANE.md:80-99`): the `keepLatGap`
  and P2.4 speed-gain ports each "nailed the magnitude and missed the duration" — "exactness is emergent
  from the state machine, not any single formula." Same root cause both times: persistent cross-step
  lateral state.
- **The ORCA/RVO pivot was also troubled** (honest negative findings): ORCA deadlock
  (`docs/LANELESS-DIRECTION.md:147`), junction-coordination "still open" with a rejected un-stick
  heuristic that "made the whole junction worse" (`docs/INDIA-TRAFFIC.md:171`), "right-of-way structurally
  incompatible with reciprocal ORCA" (`INDIA-TRAFFIC.md:25`), unified-solver "attempted … then REVERTED —
  no benefit, slight harm" (`docs/UNIFIED-SOLVER.md:8`).
- Docs' own verdict: a byte-exact port is "a large, dedicated effort … a cohesive port, not incremental
  sub-rungs" (`PHASE2-SUBLANE.md:101`).

## The performance analysis (the decisive part)
1. **The 3× was measured with sublane OFF, and sublane is a single global per-scenario switch.** `_sublane
   = LateralResolution > 0` is one engine-wide bool (`Engine.cs:1040`); there is **no per-edge/per-lane
   gate** (mirrors SUMO's global `--lateral-resolution`). The whole perf suite (`city-30/300/3000/15000`,
   all `-L 1`) is non-sublane; there is **zero** sublane benchmarking in the repo. → The perf-critical 10k
   city is a *separate* non-sublane scenario; any sublane demo is a different run.
2. **Enabling sublane on a scenario has a compounding cost** — it disables the plan-fusion fast path
   (`FusionEligible` is gated `&& !_sublane`, `Engine.cs:183`), and the byte-exact core widens each
   vehicle's read-set (per-sublane occupancy + adjacent expected speeds) and adds cross-vehicle lateral
   coupling. SUMO's own sublane is a well-known 2–5× hit; the fusion loss compounds it. So one would never
   want the 10k city sublane.
3. **That coupling is exactly what the parallel model avoids.** The 3× rests on the plan/execute invariant
   (plan reads a frozen snapshot, writes only each vehicle's own `MoveIntent`, `Engine.cs:634-653`). The
   docs call SL2015's machinery "ECS-hostile — persistent per-vehicle lateral state and push-based
   neighbor signaling fight the plan/execute double buffer" (`LANELESS-DIRECTION.md:33`).

**Conclusion:** the 10k performance and the sublane feature are **decoupled by the global gate**. Phase 4
threatens the 3× only if the big city is deliberately made sublane — which filter-to-front never requires.

## Decision & standing constraint
- **Defer Phase 4.** It is the largest, riskiest, parity-BOUND item on the roadmap (two failed attempts),
  for a feature R1 itself calls "optional, separate, lower-priority." Phases 1–3 already deliver the full
  organized → panic → flee → shoulder-push → foot-exodus story.
- **If it is ever taken up:** do it as an **isolated, benchmarked `MSLCM_SL2015` parity port in its own
  small signalized scenario** — never by flipping the 10k city to sublane. Port cohesively from
  `/sumo/src/microsim/lcmodels/MSLCM_SL2015.cpp` (grid + `updateGaps` + `mySafeLatDist*` +
  `updateExpectedSublaneSpeeds` + the change decision) with SUMO-generated goldens, and measure the
  sublane-scenario cost explicitly before committing.

## What we do instead (next)
Low-risk, parity-exempt work that advances the evacuation product without touching the golden path:
the Phase-3 wall-solver tech-debt (make `MixedTrafficCrowd` NH-wall confinement solver-robust so the
`VehicleMover` clamp backstop is no longer needed), and/or a denser / signalized evac demo.
