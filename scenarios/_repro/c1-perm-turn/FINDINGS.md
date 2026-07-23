# c1-perm-turn — SumoData's verified C1 witness: the deficit is the STEM THROUGH, via turn-lane mis-segregation at d_4_2 (NOT gap-acceptance)

**Date:** 2026-07-23. **Source:** SumoData's `c1-perm-turn-starvation` witness package (a netconvert crop of
their real box around junctions `d_4_1`/`d_4_2`, static routes, no `device.rerouting`, `sigma=0.5`). This is
the FAITHFUL box-derived anchor we asked for. Files here are their `net.xml` / `routes.rou.xml` /
`scenario.sumocfg` verbatim + their `README.md` / `MECHANISM-STATUS-and-how-to-use.md`.

## Reproduced on `claude/dense-lane-overlap-fix-5tr4ha` (rule-2 HEAD)
`--end 1200`, final step: vanilla **253 running / 1301 arrived / 2.27 m/s**; SumoSharp **316 / 1081 / 1.50**
(+25% running, −17% arrived, −34% meanSpeed). Same direction as the witness's `1a908ee` numbers (390/1009/1.04);
milder because rule-2 fixed part of the d_4_1 segregation and sigma adds run variance.

## What we INSTRUMENTED (per SumoData's ask — stop attributing from aggregates)
Progressive per-constraint `vPos` trace on head-of-lane stopped vehicles. Findings, in order:

1. **The deficit is the STEM THROUGH movement, not the permissive left.** Per-movement discharge (distinct
   vehicles crossing each junction internal lane, whole run):

   | movement | vanilla | SumoSharp |
   |---|---|---|
   | `:d_4_1_1_*` opposing through | 269/268 | 274/261 (parity) |
   | **`:d_4_1_4_0` stem THROUGH** | **252** | **70** (−72%) |
   | `:d_4_1_6_0` permissive LEFT | 27 | **26 (parity!)** |
   | **`:d_4_2_4_0` stem THROUGH (mirror)** | **243** | **55** (−77%) |

   The permissive LEFT discharges identically to vanilla. **Gap-acceptance is NOT the deficit** — the
   head-of-lane binder histogram shows `JunctionYieldConstraint` binds ~0–3 steps; the binders are
   car-following (`leader`, `crossJxnLeader`) + `redLight`. This is the FOURTH failed "gap-starvation"
   attribution (after C3-rerouting, clean-C1, local-starvation) — all now retired.

2. **The through fails to discharge into a FREE exit.** Through-chain occupancy (mean veh/step): SumoSharp's
   exit `e_d_4_2_d_4_3` is nearly EMPTY (1.3 vs vanilla 4.8) while its upstream edges are MORE jammed
   (`e_d_4_1_d_4_2` 46.7 vs 37.8). So it is not downstream backpressure — the through is blocked at the
   junction while the exit sits empty.

3. **ROOT CAUSE: turn-lane mis-segregation at d_4_2.** Lane distribution on the d_4_2 approach `e_d_4_1_d_4_2`
   (only lane2 connects to the d_4_2 left `e_d_4_2_d_3_2`):

   | flow | vanilla | SumoSharp |
   |---|---|---|
   | `flow_left2` (left) | **L2 = 100%** | **L1 = 35%**, L2 = 65% |
   | `flow_through` | L1 91% / L2 9% | L1 63% / L2 37% |

   **35% of left-turners sit on lane1 — the through lane they cannot turn from — blocking the stem through
   behind them** (`crossJxnLeader`-bound, following the jammed left destination `e_d_4_2_d_3_2` which the
   opposing2 stream saturates). A stem-through car queued behind a mis-laned left-turner reads as
   `leader`/`crossJxnLeader`-blocked → the through under-discharges → `e_d_4_0_d_4_1` / `e_d_4_1_d_4_2` pile up
   while the exit starves.

## Why d_4_1 is (mostly) fixed but d_4_2 is not
Rule-2 got `flow_left` 100% onto lane2 at d_4_1 — it DEPARTS on the long, initially-clear approach
`e_d_4_0_d_4_1` and settles on lane2. `flow_left2` DEPARTS on `e_d_4_1_d_4_2`, which is already **saturated**
by the transiting `flow_through` (occupancy 46.7), so 35% insert on / are stuck on lane1 and cannot reach the
only left-capable lane2. This is the **merge-into-the-turn-capable-lane failure under saturation** — the same
mechanism the arterial exposed, and it DOES map to the shared-lane box (lane2 is shared through+left, but it is
the ONLY lane connecting to the left, so a left-turner must reach it).

## Conclusion / hand-off
The knee deficit on this verified box witness is **turn-lane segregation under saturation** (left-turners not
reaching the only turn-capable lane when they insert into a jammed shared-lane edge), manifesting as a **stem
THROUGH under-discharge** because mis-laned turners block the through lane. It is **not** permissive-turn
gap-acceptance (parity), **not** a through-release gate (C3, retired), **not** keep-right drift alone (that is
downstream). The fix direction: get a left-turner onto the only turn-capable lane under congestion —
strategic-LC-toward-the-turn-lane with the urgency/cooperation that lets it complete even when inserting into a
saturated edge (LCA_URGENT blocker-cooperation is the piece SumoSharp still lacks; see
`Engine.cs` TryStrategicLaneChange's "future work" note). Golden-sensitive (LC model) → design-first + nod
before implementing; validate on THIS witness (through discharge → vanilla) + box discharge, byte-identical
goldens.

---

## UPDATE (same day) — keep-right drift is MINOR here; dominant cause is the stem-through's own low discharge (crossJxnLeader regress), not mis-segregation

Deeper instrumentation refined the picture — and partly reversed the "mis-segregation is the driver" read above:

1. **Insertion is clean.** `flow_left2` inserts **100% on lane2** (correct); the 21% that end on lane1
   (16/78 veh; 35% of veh-STEPS because they sit stuck) get there by **keep-right DRIFT** (L2→L1), not
   bad insertion and not merge-in failure. Vanilla's `--lanechange-output`: **0** `flow_left2` lane changes.
2. **But suppressing the drift barely helps.** Forcing full keep-right suppression for eligible turners
   (`EXP_FULLSUPPRESS`) recovers the stem-through only **70→94** (d_4_1) / **55→67** (d_4_2) — vanilla is
   252/243. Network: 316→299 running, 1081→1104 arrived. So **keep-right drift ≈ 15% of the through
   deficit**, not the dominant driver. (Component-2 confirmed real but minor here too.)
3. **The through's own discharge is the dominant deficit.** With drift suppressed, the d_4_2 through head
   (`e_d_4_1_d_4_2_1`, at the stop line) is bound by **`CrossJunctionLeaderConstraint`** — car-following the
   through vehicle just ahead across the junction, with **negative gaps** (−0.66…−2.19 = within minGap,
   bumper-to-bumper). That is a REGRESS: each through follows the one ahead. The exit `e_d_4_2_d_4_3` is
   nearly empty (1.3) while the queue packs tight → the through crosses at a **low discharge rate / high
   start-up lost time**, not blocked by backpressure. `JunctionYieldConstraint` binds ~0 throughout
   (gap-acceptance is not involved). `RedLight` binds only during genuine red.

**Where this leaves the mechanism:** the knee deficit on this witness is dominated by **low stem-through
discharge across the static-TL junction** (the through queue barely advances during its green even into an
empty exit), with keep-right drift a real but ~15% secondary contributor. The next step is to trace the
ABSOLUTE head-of-queue through vehicle at start-of-green (what binds the very first car when the light turns
green and the exit is empty) — that isolates whether it is a start-up/discharge-headway effect, a
cross-junction gap/where-the-internal-lane-ends detail, or the TL green-timing response. NOT gap-acceptance,
NOT a through-release gate (C3), NOT primarily mis-segregation. Design deferred until the head-of-queue binder
is pinned.
