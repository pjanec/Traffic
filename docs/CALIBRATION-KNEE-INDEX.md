# CALIBRATION-KNEE-INDEX.md — orientation map for the high-density / calibration-knee investigation

**Purpose:** one place to orient in the multi-session line of work aimed at making SumoSharp a trustworthy
high-density engine with byte-identical behavioral parity to vanilla SUMO 1.20.0. Read this first, then dive
into the linked docs. **Last updated: 2026-07-22 (session 4).**

## The goal & the two roles
SumoSharp must serve two roles for SumoData's live-city pipeline:
- **CALIBRATE** — find the highest believable traffic density (vanilla's "knee"). Measured under
  **sustained insertion held at the calibrated density** (a discharge deficit compounds into unbounded
  accumulation), NOT one-shot demand.
- **SERVE / RUN** — load the calibrated scenario and run it believably (the live demo).

**Standing guidance (SumoData):** *calibrate the knee with vanilla, serve/run with SumoSharp.* The serve
role is solid (box loads + runs, arrivals + parking at parity). The knee work below is about whether
SumoSharp can eventually calibrate *itself*.

## Where the canonical docs live
- **Design / HOW (the main narrative, dated `§2.3.x` sections):** `docs/HIGH-DENSITY-CALIBRATION-DESIGN.md`
- **Checklist:** `docs/HIGH-DENSITY-CALIBRATION-TRACKER.md` · **Tasks/success conditions:** `…-TASKS.md`
- **Resume docs (self-contained, per sub-problem):** `docs/GAP1-RESUME.md`, `docs/DISCHARGE-YIELD-RESUME.md`
- **Repro witnesses (committed, offline):** see the table at the bottom.
- **SumoData hand-offs:** `docs/SUMOSHARP-RESPONSE-sustained-insertion-knee.md`,
  `docs/FOLLOWUP-TL-throughput-flowrate.md` (now largely superseded — see §2.3.9), and the uploaded NEED/NOTE
  docs from the SumoData session.
- Branch for all of this: `claude/dense-lane-overlap-fix-5tr4ha`.

## The investigation, gap by gap (chronological; ✅ = landed, byte-identical goldens)

| # | Gap | Status | Root cause → fix | What did NOT work | Detail |
|---|---|---|---|---|---|
| 1 | **Gap-1 dead-lane gridlock** (2× dense synthetic: 10 tp / 275 arr / ~45 stuck) | ✅ SOLVED | A car forced onto a lane with no connection to its next route edge slammed the lane end and clamped. Fix = 3 dead-lane-gated parts: `DeadLaneMergeBrakeConstraint` (port of `MSLCM_LC2013::informLeader stopSpeed`), boundary reroute (5 s), `TryRerouteStuckDeadLane` (90 s gate). → 2× **0/290**, 1× **1/290** = vanilla. | Instant reroute (churned both densities, cascaded at 1×); reroute-only (insufficient — needs the merge brake); eager 5 s stuck-reroute (churns dense). | DESIGN §2.3.1–§2.3.5, `GAP1-RESUME.md`, `synthetic-junction2/` |
| 2 | **Stage-4 box one-shot** (arrivals 47 vs vanilla 96) | ✅ CRACKED | Ring-fringe sink drainage + final-edge arrival + residency-guard narrowing → 47 → **106** (van 96). | — | DESIGN §2.3.6 |
| 3 | **Parking (reroute drops mid-route parking)** — inflates running count under sustained load | ✅ LANDED (parity-safe) | 3 reroute sites shortest-path'd `currentEdge→finalDest`, collapsing the parking detour. Fix preserves mid-route parking but **excludes the departure edge** (`0 < pos < last`) — the synthetic's 10 "mid-route" cars are actually `departPos="stop"` edge-0 parking. → box parks **853** vs van **858**. | Blanket "skip reroute for parking cars" (loses Gap-1 drainage rerouting AND still needs mid-route parking to run); preserving-parking naively (regressed Gap-1 synthetic — mid-route parking jams under load). WIP on `claude/reroute-with-stops-wip` (do-not-merge). | DESIGN §2.3.7, `SUMOSHARP-RESPONSE-sustained-insertion-knee.md` |
| 4 | **Permissive-yield** (`lt`: 112 left-turns vs vanilla 7) | ✅ LANDED (realism win, **separate axis**) | `FindFoeVehicle` returned an already-crossed foe → a saturated permissive left never saw the oncoming stream. Fix: `FindCrossFoeVehicle` (excludes crossed foes) + `BlockedByCrossingFoe` arrival-time window (vLinkPass) + impatience. → **7** = vanilla, goldens byte-identical. | Impatience is **not** the lever for the dense-synthetic teleport cascade (60 s == 300 s). Foe-selection guard alone over-yields the merge arm (must keep merge arm on `FindFoeVehicle`). | DESIGN §2.3.8, `DISCHARGE-YIELD-RESUME.md`, `saturation-flow/` |
| 5 | **THE KNEE BLOCKER — signalized-discharge redistribution** (real box 5.5× / 538% / 382 tp) | ✅ ROOT-CAUSED + FIXED | `RedLightConstraint` braked vehicles whose route **ends** at a TL edge (arriving/exiting cars that never cross the TL). SUMO breaks out of the link walk on the final edge (MSVehicle.cpp:2587). Fix: skip the TL brake when `LaneSeqIndex+1 >= LaneSeqLen`. → faithful repro matches vanilla on every axis (3-way ratio 2.45×→**1.01×**). | The first offline grid (`sustained-box/`) attached fringe to every border node → all degree-4, no asymmetry → showed parity while the real box blew up 5.5×. (SumoData's decisive cross-check exposed this.) | DESIGN §2.3.9, `signalized-asymmetry/FINDINGS.md` |

## Key cross-cutting lessons (so future-you doesn't repeat them)
1. **Aggregate metrics hide local redistribution.** The knee blocker was network-wide only ~1.22× but 8–10×
   on specific approaches. Always measure **per-edge / per-approach density**, not just total arrivals.
2. **A synthetic can lie by omission.** `sustained-box` (uniform degree-4) reported parity on the very
   commit the real box blew up 5.5× on. Match the real net's *structural* features (here: 3-way/4-way TL
   connectivity asymmetry, 3-lane approaches, arrival edges at TLs) before trusting a repro. Validate a
   candidate repro against the real pipeline / a known-divergent commit.
3. **Realism ≠ throughput.** The permissive-yield fix (correct realism) *reduces* junction throughput — it
   is on the opposite axis from the knee (which needs *more*). Keep the axes separate.
4. **The connectivity-asymmetry framing was a proxy.** The causal driver of the "3-way T-lights pile up" was
   **arrival edges at TLs** (routes terminate at low-degree border junctions). Chase the FCD to the causal
   mechanism, don't stop at the correlation.
5. **Follow SUMO source for the exemption, not just the happy path.** The fix came from reading
   `planMoveInternal`'s final-edge `break` — the arriving-vehicle exemption most constraints already had via
   the forward-scan, but `RedLightConstraint` lacked because it keys off the current lane.

## Repro witnesses (all committed, offline, no SUMO needed at test time)
| dir | what it isolates | verdict |
|---|---|---|
| `scenarios/_repro/synthetic-junction2/` | Gap-1 dead-lane gridlock (2× dense) | ✅ drains 0–2 tp / 290 arr = vanilla |
| `scenarios/_repro/saturation-flow/` | per-movement discharge: straight (parity) vs permissive-left | ✅ `lt` 112→7 |
| `scenarios/_repro/sustained-box/` | sustained-load grid — **UNFAITHFUL** (no connectivity asymmetry) | ⚠️ reports parity even when the real box blows up; kept + labeled as a lesson |
| `scenarios/_repro/signalized-asymmetry/` | **faithful** knee repro: 3-way/4-way TL asymmetry, arrival-edge backup | ✅ 2.45×→1.01× after the fix |
| `scenarios/_repro/parking-maneuver/` | SumoData's parking-maneuver hypothesis (maneuver + capacity-fill sink) | ❌ NOT the knee — SumoSharp *under*-accumulates at parking (opposite direction); found 2 wrong-direction fidelity bugs |
| `scenarios/_repro/arterial-tjunction/` | **THE KNEE mechanism**: through traffic PAST 3-way T-lights, shared through+left lanes | ✅ reproduces the 5.5×-direction deficit (running 462 vs 303); FCD-traced to **turn-lane mis-segregation** |

Anchor tests: `tests/Sim.ParityTests/DenseFlowDeadLaneDrainTests.cs` (Gap-1, intent-encoded: arrivals ≥ 290
hard, teleports ≤ 2 documented), plus the full byte-identical golden suite (657 parity + 227 pedestrian).

## Current open state (2026-07-22)
- **Landed & pushed** on `claude/dense-lane-overlap-fix-5tr4ha`: Gap-1, Stage-4, parking, permissive-yield
  (`f69a58d`), sustained-box witness (`2c00179`), **arrival-TL discharge fix** (`ca8d515`).
- **SumoData re-ran `ca8d515`:** it's a real fix to KEEP, but did NOT drop their box overshoot (marginally
  worse — a *knee-selection artifact*: faster probes → higher selected knee → bigger served overshoot; so
  "made probes flow" ≠ "fixed the knee"). Their box's blocker is in the `auto_parking` demand (vehicles
  arrive off-lane at parkingAreas, so the arrival-TL bug mostly doesn't apply).
- **Parking-maneuver hypothesis tested → NOT the blocker** (`parking-maneuver/`): SumoSharp *under*-accumulates
  at parking. The knee's over-accumulation remains unlocalized on a faithful offline repro.
- **SumoData cracked the mechanism** (`SUMOSHARP-MECHANISM-3way-tlight-through-discharge.md`): through-discharge
  deficit at 3-way T-lights; a vehicle stopped at the stop line during green isn't cleared until the next
  cycle (they pointed at `ComputeWillPass`). With their repro upgrade (through traffic PAST the T-lights,
  shared lanes) I **reproduced it** (`arterial-tjunction/`) and **FCD-traced the real root: turn-lane
  mis-segregation.** Vanilla runs left-turners 100% on the dedicated left lane and keeps through off it;
  SumoSharp mixes them, so a stuck permissive left-turner sits at the head of a *through* lane and
  serial-blocks it. The `WillPass`/stopped-during-green pattern is the **symptom** (a through car stuck behind
  a mis-laned turner).
- **NEXT (needs alignment — big/delicate):** the fix is in the lane-change / lane-choice model
  (`getBestLanes`-equivalent lane desirability / `ResolveBestDepartLane` / strategic LC desire), a
  heavily golden-tested area — bigger than the localized arrival-edge exemption. Align with SumoData (does
  their box show the lane-mixing?) + user on scope before implementing; verify byte-identical goldens and
  served-density discharge (≈ vanilla's throughput).
- **Two independent fidelity bugs found (parking, wrong-direction for the knee), open as cheap serve/viz wins:**
  FCD excludes parked vehicles (Bug B); no on-lane queue for a full parkingArea (Bug A, verify vs
  `parking.rerouting`).
- Rule: validate any knee-targeted fix at **served/sustained density**, never sparse-probe or one-shot flow
  (a local fix can improve those while the sustained overshoot gets worse — the knee-selection artifact).
- **If residual overshoot remains:** localize the next hotspot the same way — per-edge density over the
  overshoot window → find the pile → read the FCD to the causal mechanism → build/extend a faithful repro →
  port the SUMO exemption → verify goldens byte-identical.
- The one-shot ~27% deficit in `FOLLOWUP-TL-throughput-flowrate.md` is **stale** (closed by the fixes above;
  the non-dense synthetic is now 290/290 arrivals, SumoSharp slightly faster).
