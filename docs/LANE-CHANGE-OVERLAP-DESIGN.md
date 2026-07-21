# LANE-CHANGE-OVERLAP-DESIGN.md — dense multi-lane car overlaps: root cause + fix (HOW)

**Status:** design (evidence-based, this session). Implements the fix requested in
`docs/LANE-CHANGE-OVERLAP-SPEC.md`. **Type:** parity-sensitive engine port. Grounds in vendored
`/sumo/src/microsim/MSLaneChanger.cpp::checkChange` (v1_20_0).

This document supersedes the *mechanism hypothesis* in the SPEC (which framed the bug as a missing
**follower-gap veto + cooperative lane-changing**). Direct instrumentation this session shows the
dominant cause is different and simpler, and that the cooperative-LC machinery the SPEC prescribed is
**not required** to reach the acceptance gates. See §2 for the evidence; §7 for why the SPEC's plan is
not followed verbatim.

---

## 1. WHAT — reference the SPEC

`docs/LANE-CHANGE-OVERLAP-SPEC.md` is the WHAT (reproduce, metric, acceptance). One line: in the dense
`scenarios/_diag/willpass-saturation` grid the engine lets a vehicle finish a lane change into an
occupied slot, so two cars sit on the same lane closer than one vehicle length (a physical overlap);
vanilla SUMO 1.20.0 produces 0 on the same net. Acceptance metric = consecutive same-lane pairs with
lane-relative `pos` gap `< 5.5 m`, counted per timestep over 200 steps
(`tests/Sim.ParityTests/LaneChangeOverlapDiagTests.cs`, currently `[Fact(Skip)]`).

## 2. Root cause — measured, not assumed (this session)

Every committed lane-change commit site was instrumented (env-gated `LC_DIAG`) to record, per path,
whether the change lands a vehicle within `5.5 m` of a target-lane occupant, categorised by the
occupant's signed position delta (leaderSide / followerSide / **co-located**) and by stopped/moving.
Measured on `willpass-saturation`, 200 steps, **library-default (parity) mode** — the mode the
acceptance test runs (`new Engine()`):

| path         | commits | overlap-creating commits (category)                          |
|--------------|---------|--------------------------------------------------------------|
| keep-right   | 171     | 22 co-located·stopped, 5 co-located·moving, 10 follower-side  |
| strategic    | 204     | 13 **co-located·stopped**                                     |
| speed-gain   | 120     | 0                                                             |
| give-way     | —       | 0 (bluelight-only, inert)                                     |

**Findings:**

1. **The dominant cause is a neighbour-query blind spot, not a follower gap.** ~40 of ~50 bad commits
   are **co-located**: a target-lane occupant at essentially ego's exact longitudinal position. The
   lane-change neighbour lookup classifies such an occupant as **neither** leader (`GetNeighborLeader`
   skips `pos ≤ egoPos`) **nor** follower (`GetNeighborFollower` skips `pos ≥ egoPos`), so `IsTargetLaneSafe`
   sees no neighbour and returns *safe* — even on the **strategic** path, which already passes both a
   leader and a follower. This is why the SPEC's "add the follower veto" plan is *insufficient*: an
   **exact-position tie** is invisible to the follower lookup too. Exact ties are common in a saturated
   grid because vehicles on adjacent lanes stop at the same stop-line arc position.

2. **SUMO already models exactly this** as `LCA_OVERLAPPING`: `MSLaneChanger::checkChange`
   (`MSLaneChanger.cpp:767,780`) hard-blocks a change when the neighbour follower/leader gap is `< 0`
   (bodies overlap), independent of the secure-gap test that follows. The engine's `IsTargetLaneSafe`
   ported the secure-gap test but **not** the overlap block, and its neighbour lookup never surfaces the
   overlapping vehicle.

3. **Keep-right also passes a `null` follower** (a documented reduction, `HIGH-DENSITY-P2G-DESIGN.md`
   §4.1), so the ~10 genuine follower-side keep-right cut-ins are unvetoed.

4. **The §5 gridlock trap is obsolete.** The SPEC (from `HIGH-DENSITY-P2G-DESIGN.md` §5) warns that a
   follower veto without cooperative LC regressed the grid 0→30 stuck. That measurement predates the
   serve-path junction traffic-light fixes (`HIGH-DENSITY-P2G2-...-DESIGN.md` §0), which removed the
   gridlock at its root. Re-measured this session: the overlap block **plus** the keep-right follower
   veto keeps `willpass-saturation` at **0 stuck** over 700 steps. So **cooperative LC is not needed**
   to satisfy the acceptance gates.

**Net of the two-part core fix (overlap block on every path + keep-right follower veto):** overlaps
**197 → 3**, both-stopped **186 → 0**, stuck **0**, full parity suite **654 green**.

5. **Residual 3 are junction-emergence overlaps, not stopped-car lane changes.** All 3 are transient
   (1–2 steps), moving, and occur at the very start of an edge (a junction exit):
   - **Case A (cross-junction follower):** a near-stopped **speed-gain** into a junction-exit slot
     (`pos ≈ 1.5 m`) whose imminent follower is still on the feeding internal lane (`:J_*`), so the
     target-lane follower/overlap veto cannot see it. SUMO's `getFollowersOnConsecutive` looks upstream
     across the junction and blocks it.
   - **Case B (junction merge):** two vehicles crossing one junction via **different** internal lanes
     both emerge onto the **same** normal lane one step apart and overlap — a cross-junction
     car-following tightness issue, not a lane change at all.

## 3. HOW — the fix, in three stages

### Stage 1 — overlap block on every path + universal keep-right follower veto (the core)
Port `checkChange`'s `LCA_OVERLAPPING` block faithfully and make it reachable on every commit path:

- Add `IsTargetLaneOverlapped(ego, targetLaneHandle, neighbors)` — true iff any vehicle on the target
  lane's pos-sorted bucket (`LaneNeighborQuery.OnLane`) has a body `[pos-len, pos]` overlapping ego's
  projected slot `[egoPos-len, egoPos]`. This is the `neigh.second < 0` block, made robust to the
  exact-tie the leader/follower lookup misses. Reads only the frozen post-move snapshot + immutable
  network; ego's own fields only otherwise. Inert when the target lane has no overlapping occupant.
- AND `!IsTargetLaneOverlapped(...)` into every commit gate: keep-right, speed-gain, strategic, give-way.
- Keep-right additionally passes the **real** target-lane follower to `IsTargetLaneSafe` (drop the
  `null`), matching `checkChange`'s symmetric follower block for a RIGHT change.

Determinism/parity: the block only ever **prevents** a change (never creates one); a vetoed change does
NOT reset its accumulator (SUMO resets only on a committed change), so the vehicle retries next step. On
every committed golden the target lane has no overlapping/blocking occupant at a change, so both
additions are inert → **byte-identical** (verified by the full suite + a golden byte-diff).

### Stage 2 — cross-junction follower for the lane-change veto (Case A)
Build a load-time reverse index `normalLaneHandle → feeding internal (`:`) lane handle(s)` from
`Network.Connections` (`Via` → `To` lane). In the target-lane veto, additionally scan vehicles on the
feeding internal lanes near their end; project each to the target lane (`projFront = -(internalLen -
pos)`) and apply the SAME secure-gap follower test `IsTargetLaneSafe` uses (`MSCFModel::getSecureGap`),
blocking on a negative/insufficient gap. This is `getFollowersOnConsecutive` at the granularity this
case needs (one junction upstream). Inert unless the target lane is a junction exit with a vehicle about
to emerge — no committed golden exercises that, so byte-identical.

### Stage 3 — junction-merge residual (Case B)
Determine whether Case B survives once Stage 2 changes the trajectory. If it does, it is a cross-junction
**car-following** gap (the second emerging vehicle overshoots its cross-junction leader), addressed in
the emergence/transfer path, not the lane changer. Kept as its own stage precisely because it is a
different mechanism from the lane-change overlaps and must not be conflated. Exact approach chosen after
Stage-2 re-measurement (candidate: tighten the internal→normal transfer so an emerging vehicle clamps
behind the target lane's rearmost occupant — the same body-non-overlap invariant, enforced at the
junction exit).

## 4. Determinism / parity invariants (the iron law)
- Every committed golden stays **byte-identical**: each new check only vetoes, is inert at low density,
  and reads only the frozen snapshot + immutable network. Gate = full `dotnet test Traffic.sln` green
  AND a per-scenario byte-diff of `engine.fcd.xml` vs `golden.fcd.xml` on representative multi-lane
  goldens.
- No `System.Random`; all state per-vehicle / read-only-shared; region-parallel byte-identical to serial
  (the veto reads the frozen snapshot and writes only ego).
- `willpass-saturation` stuck-count stays **0** — re-run every stage (hard gate).

## 5. Success conditions — see `docs/LANE-CHANGE-OVERLAP-TASKS.md`

## 6. Files touched
- `src/Sim.Core/Engine.cs` — `IsTargetLaneSafe` call sites (keep-right, speed-gain, strategic, give-way);
  new `IsTargetLaneOverlapped` + cross-junction follower scan; load-time reverse index.
- `src/Sim.Core/LaneNeighborQuery.cs` — reuse `OnLane`; no new shared-query semantics.
- `tests/Sim.ParityTests/LaneChangeOverlapDiagTests.cs` — unskip, assert `== 0`.

## 7. Deviation from the SPEC's prescribed mechanism (documented, evidence-gated)
The SPEC mandates porting `MSLCM_LC2013::informFollower`/`saveBlockerLength`/`updateBlockerLength`
(cooperative LC) *together with* a follower veto, on the argument that the follower veto alone gridlocks
the grid (0→30 stuck). This session's measurements show (a) the dominant overlap cause is the
`LCA_OVERLAPPING` blind spot, not a follower gap, and (b) with the junction-TL fixes already in the
tree, the follower veto no longer gridlocks (stuck stays 0). Porting cooperative LC would therefore add
a large, organic-flow-degrading subsystem (retired once already, `HIGH-DENSITY-P2G2-...-DESIGN.md` §0)
that buys nothing against the acceptance gates. Per CLAUDE.md rule 4 the fix still follows SUMO faithfully
— it ports `checkChange`'s overlap+follower block and `getFollowersOnConsecutive` — it simply declines to
port a *separate* SUMO feature that the evidence shows is not binding here. If a future scenario proves a
follower-blocked change that only cooperative LC can resolve without gridlock, this doc's §3 is the seam
to add it behind.
