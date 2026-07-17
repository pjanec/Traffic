# HIGH-DENSITY-P2G3-DESIGN.md — scenario-46 speedGain residual: diagnosis + deferred deep fix

**Status:** DIAGNOSIS COMPLETE — the originally-proposed fix (best-lanes continuation distance) was
implemented and INSTRUMENTED to be necessary-but-INSUFFICIENT; the binding gap is deeper (cross-junction
leader anticipation in the speedGain incentive). Reverted, not landed. Item **P2G-3** (from
`HIGH-DENSITY-P2G-DESIGN.md` §7). Grounds in `sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp` +
`sumo/src/microsim/MSVehicle.cpp`. See §5.1 for the instrumented finding and the true remaining gap.

## 1. WHAT (the gap) — scenario 46's residual, correctly diagnosed

After the P2-G keep-right leader veto, `scenarios/46-reroute-multilane` still diverges from SUMO by
~7 m. The diagnostic (SUMO `--lanechange-output`) shows the residual is **NOT** cooperative LC (zero
cooperative / zero strategic changes occur in the whole scenario): it is a **`speedGain` overtake the
engine fails to make**. On the 2-lane detour, at the K junction the two turn lanes have different
internal geometry (lane 0's turn is slower), so a leader on lane 0 decelerates to ~6.2 m/s; SUMO's
follower does a `speedGain` change to the faster lane 1 to avoid the slowdown. The engine never does.

**Root cause (confirmed three ways — SUMO reason string, engine code, gate arithmetic):** the engine's
speedGain and keep-right decisions compute the target-lane distance as the **single lane's own remaining
length**, not the **multi-edge best-lanes continuation**. At t=91 vehicle `f.1` is at pos 584.53 on a
603.25 m edge, so the engine's `neighDist = 603.25 - 584.53 = 18.72 m`, and the fire gate
`neighDist / max(.1, speed) > 20` evaluates `18.72 / 13.55 = 1.38 < 20` → the change is suppressed.

## 2. SUMO reference

`MSLCM_LC2013::_wantsChange` (`MSLCM_LC2013.cpp:1112-1136`) sets `currentDist = curr.length` and
`neighDist = neigh.length`, where `curr`/`neigh` are the vehicle's best-lanes `LaneQ` entries. The
speedGain fire gate (`:1857-1859`) is `mySpeedGainProbability > threshold && relativeGain > EPS &&
neighDist / MAX2(.1, speed) > 20`. `LaneQ.length` (`MSVehicle.cpp:5911` base, accumulated backward at
`:6040` `j.length += bestConnectedNext->length`) is **the on-route continuation distance from the lane
start** — "the distance the vehicle can go on its route without changing lanes from that lane"
(`:1109-1110`). For `f.1`'s route `e_src → e_det1 → e_det2 → e_dest`, the continuation of `e_det1_1` is
~1703 m, so SUMO's gate is `1703 / 13.55 ≈ 126 > 20` → the speedGain fires.

## 3. HOW (the fix)

Replace the single-lane distance with the best-lanes continuation length in BOTH decisions, using the
engine's existing memoized `BestLanesCached(routeId, route.Edges, edgeId)` (the same `ComputeBestLanes`
the strategic / `KeepRightStrategicStay` paths already call), preserving each path's EXISTING position
handling so single-edge routes stay byte-identical.

### 3.1 speedGain (`DecideSpeedGainForVehicle`)
Currently:
```
var currentDist = lane.Length - v.Kinematics.Pos;
var neighDist  = leftLane.Length - v.Kinematics.Pos;
```
Becomes (continuation length for the respective lane index, still minus position — the path's existing
convention):
```
var bestLanes  = BestLanesCached(EffectiveRouteId(v), route.Edges, lane.EdgeId);
var currentDist = ContinuationLength(bestLanes, lane.Index)     - v.Kinematics.Pos;
var neighDist   = ContinuationLength(bestLanes, leftLane.Index) - v.Kinematics.Pos;
```
`ContinuationLength` finds the `LaneContinuation` with matching `LaneIndex` and returns its `.Length`
(falling back to the lane's own `Length` if, defensively, no entry matches). These feed both
`AnticipateFollowSpeed` (`thisLaneVSafe`/`neighLaneVSafe`) and the `>20` gate, exactly as today.

### 3.2 keep-right (`ApplyKeepRightDecision`)
Currently `var neighDist = rightLane.Length;` (no position term — its existing convention, matching
SUMO's `neigh.length`). Becomes the right lane's continuation length:
```
var neighDist = ContinuationLength(BestLanesCached(routeId, route.Edges, lane.EdgeId), rightLane.Index);
```
feeding the existing `fullSpeedGap`/`fullSpeedDrivingSeconds` incentive math unchanged. (The
`KeepRightStrategicStay` suppressor already uses the continuation length; this aligns the incentive
distance with it.)

## 4. Determinism / parity argument (byte-identical for single-edge; ≤ SUMO for multi-edge)

- **Single-edge routes** (every vehicle whose current edge is the last route edge): `ComputeBestLanes`'
  base case sets each lane's continuation `Length` = its own lane length, so `ContinuationLength(...) ==
  laneLength` and both formulas reduce EXACTLY to the current code. Every committed single-edge scenario
  is therefore byte-identical.
- **Multi-edge routes**: the continuation length is longer, so `neighDist` grows. Since SUMO's own
  `neigh.length` is longer still (it does not subtract position), the engine's gate can only become
  *closer to* SUMO's, never fire a change SUMO would not — the change is monotone toward SUMO. The full
  `dotnet test` suite is the gate: every prior golden must stay green/byte-identical.
- Reads only the immutable route/network via the memoized best-lanes cache + the frozen post-move
  snapshot; writes only ego's own fields. No new cross-vehicle coupling; the region-parallel plan phase
  stays byte-identical to serial. `BestLanesCached` is a `ConcurrentDictionary` memo (pure function of
  immutable route+network), so the added per-vehicle lookup is cache-hot and thread-safe.

## 5.1 INSTRUMENTED FINDING — neighDist was necessary but NOT the binding constraint

The §3 fix was implemented (speedGain + keep-right continuation distance) and instrumented on
scenario 46. Results:

- **The continuation distance works as designed:** for `f.1` at the K junction, `neighDist` went from
  ~18.7 m (single-lane) to **~1145 m** (continuation), so the `>20 s` gate now passes trivially
  (82.5 ≫ 20). But `f.1` **still does not fire the speedGain**, and the scenario-46 trajectory is
  **byte-for-byte unchanged**.
- **Why:** the speedGain accumulator `mySpeedGainProbability` never builds, because the engine's
  `thisLaneVSafe == neighLaneVSafe == 13.89` (so `relativeGain ≈ 0`, accumulator decays). The engine's
  `f.1` **does not see the slow leader `f.0`** in the speedGain incentive: `thisLaneVSafe` uses a
  same-lane leader lookup (`postMoveNeighbors.GetLeader`), which loses `f.0` the moment `f.0` crosses
  onto the junction internal lane (`:K_0_0`). SUMO's `thisLaneVSafe` (`getRawSpeed`) anticipates the
  leader **across the best-lanes continuation** (through the junction), sees `f.0` at 6.18 m/s, and
  accumulates → fires at t=91. (The engine's *car-following* phase does brake `f.1` for `f.0` across
  the junction — via the junction-link leader path — so `f.1` still slows; it is only the *lane-change
  incentive's* `thisLaneVSafe` that is blind to the cross-junction leader.)
- **The keep-right half of the §3 fix separately regressed** the saturated `-L2` diagnostic
  (`willpass-saturation`, 0 → 90 stuck), because a longer keep-right incentive distance over-fires
  keep-rights in dense multi-edge traffic — the same cooperative-LC coupling P2-G hit. So even the
  keep-right continuation is not landable alone.

**Conclusion:** scenario 46's residual is a **cross-junction leader-anticipation** gap in the speedGain
incentive's `thisLaneVSafe`/`neighLaneVSafe` — SUMO's `getRawSpeed`/best-lanes-continuation leader
look-ahead (`MSLCM_LC2013::_wantsChange`'s `getSlowest`/anticipated-speed path). The engine's LC
decision reads only same-edge leaders. Closing it means giving the LC incentive a continuation-aware
leader lookup (look across the vehicle's best-lanes continuation for the constraining leader), a
genuine LC-model extension comparable in depth to cooperative LC — NOT the small distance tweak first
scoped. **Deferred pending an explicit owner decision** (§7). The neighDist continuation change was
reverted (byte-identical everywhere → no distinguishing anchor → not landable on its own).

## 5.2 THE FIX DESIGN (session 3 — cross-junction leader anticipation)

SUMO's `_wantsChange` computes `thisLaneVSafe = MIN2(vMax, anticipateFollowSpeed(leader, currentDist, …))`
(`MSLCM_LC2013.cpp:1549`) where `leader` is the CONTINUATION-spanning leader (SUMO's `getLeader` walks the
best-lanes continuation, so it finds `f.0` on the junction internal lane `:K_0_0`, which is on `e_det1_0`'s
continuation). The engine's `leader = postMoveNeighbors.GetLeader(v)` is SAME-EDGE only, so it misses `f.0`.
With no leader seen and BOTH lanes' `currentDist`/`neighDist` equal, `thisLaneVSafe == neighLaneVSafe` →
`relativeGain ≈ 0` → the accumulator never builds. SUMO sees `f.0` on ego's lane (→ reduced `thisLaneVSafe`)
and a clear left lane (→ `neighLaneVSafe == vMax`), so `relativeGain > 0` → it fires the speedGain.

**Two coupled parts (both needed for bit-exact scenario 46):**

1. **Cross-junction leader for `thisLaneVSafe` (the essential, novel piece).** Reuse the running-phase
   `CrossJunctionLeaderConstraint` machinery (`Engine.cs:7085` — downstream span of `_laneSeqPool` from
   `LaneSeqStart+LaneSeqIndex+1`, `TryFindCrossJunctionLeader` + `NeighborRearmost`) to find the nearest
   leader on EGO's continuation and its correctly-accumulated cross-junction gap. Reduce
   `thisLaneVSafe = MIN(thisLaneVSafe, MaximumSafeFollowSpeed(contGap, …, onInsertion:true))`. NOTE:
   `AnticipateFollowSpeed`'s gap formula is SAME-LANE (`leaderBackPos - minGap - egoPos`), so the
   cross-junction leader must NOT be routed through it — call `MaximumSafeFollowSpeed` directly with the
   scan's accumulated `contGap`.
2. **Continuation distance for the speedGain `currentDist`/`neighDist` (speedGain path ONLY).** So the
   clear left lane reads `neighLaneVSafe == vMax` (not the short-remaining-distance stop-speed floor),
   matching SUMO. This is the reverted §5.1 change but applied to the speedGain path ONLY (never
   keep-right — that half regressed the saturated grid). Byte-identical for single-edge routes.

**HARD GATE — the saturated-grid diagnostic.** Adding a cross-junction leader to the LC incentive makes
vehicles speedGain-left MORE (to escape a slow-continuation lane); this MUST NOT regress
`scenarios/_diag/willpass-saturation` (currently 0 stuck). Adversarially re-run it every iteration; if it
regresses, the fix is gated/reverted like the follower veto was. The neigh-lane cross-junction leader
(needed for full faithfulness where the LEFT continuation has a downstream leader) is a further step —
scenario 46's left lane is clear, so part 1 (ego side) + part 2 suffice for THIS anchor; a neigh-side
symmetric scan is deferred unless a scenario needs it.

## 5.3 IMPLEMENTED + MEASURED (session 3) — the fix WORKS but hits the saturation hard gate

Both §5.2 parts were implemented (cross-junction leader for `thisLaneVSafe` via a `TryFindContinuationLeader`
copy of `CrossJunctionLeaderConstraint`'s downstream scan, gated on `leader is null`; + continuation
distance for the speed-gain `currentDist`/`neighDist` only). Measured:

- **Scenario 46 lane choice is FIXED.** First divergence moved from a LANE mismatch at t=91
  (engine `e_det1_0` vs SUMO `e_det1_1`) to a **0.75 m same-lane pos/speed mismatch at t=92 — both now on
  `e_det1_1`**. The engine fires the speedGain across the junction like SUMO. Mechanism confirmed correct.
- **Byte-identical on EVERY bit-exact golden.** Full suite: 570 passed, and the ONLY failure was the
  saturated-grid diagnostic — so the change touches nothing but the intended dense multi-lane path.
- **BUT the saturated-grid HARD GATE regressed: `willpass-saturation` 0 → 51 stuck.** The cross-junction
  leader makes vehicles speed-gain-left aggressively to escape a slow-continuation lane; in a saturated
  `-L2` grid that thrashes (interacting with the P2-G target-lane vetoes) into gridlock. SUMO flows the
  same net because it has the FULL coordinated LC model (cooperative changes / `informFollower` keeping
  the thrash coordinated). Reverted per the hard gate.

**DEFINITIVE FINDING — the two deep items are ONE coupled problem.** This is the THIRD multi-lane LC
faithfulness improvement to regress the saturated grid in isolation (P2-G follower-veto → 30, keep-right
continuation → 90, P2G-3 cross-junction speedGain → 51). Each is individually byte-identical on the parity
goldens and individually more SUMO-faithful at moderate density, yet each gridlocks the saturated grid
because the engine lacks the **coordinated dense LC model** (cooperative LC / `informFollower`, and the
speed-advice `myVSafes` channel) that lets SUMO make all these changes AND keep flowing. **P2G-3 cannot
land in the default path without P2G-2 (cooperative LC) alongside** — they are the same problem.

Landing options for the owner (none taken without a go): (a) build the coordinated dense LC model
(cooperative LC + speed-advice channel) so BOTH land together and the saturated grid flows like SUMO —
large; (b) land P2G-3 behind an **opt-in flag** (default off → saturation diag + all goldens unchanged;
opt-in gives the more-faithful moderate-density lane choice) per owner steer #1, once the flag-on
downstream cascade (a large late max-pos-err spike seen under the flag) is understood; (c) accept
scenario 46 as behavioural and stop. The `TryFindContinuationLeader` helper + the two-part change are
documented here and trivially re-appliable.

## 5. Success conditions (acceptance gate) — target for the §5.2 fix

1. **scenario 46 residual closes:** with the fix, the engine performs the `speedGain` change moving
   `f.1` onto `e_det1_1` at t=91 (matching SUMO), and the scenario-46 engine-vs-golden max position
   error drops from ~7.09 m toward ~0. Re-measure and report.
2. **New bit-exact anchor** `scenarios/51-multilane-speedgain-continuation`: a minimal 2-lane →
   (junction) → 2-lane route where a follower must `speedGain` past a slow leader across the edge
   boundary — impossible under the single-lane distance (gate fails), correct under the continuation
   distance. `sigma=0`, SUMO 1.20.0 golden (`--precision 6`, `--fcd-output.acceleration`), tolerance
   exact `lane`/`pos`/`speed` @1e-3. The engine matches @1e-3 only with the fix; the test asserts the
   follower reaches the faster lane like SUMO.
3. **No regression:** full suite stays green and byte-identical for every prior golden (561 → 561 + new
   anchor tests). `Sim.Bench` determinism hash unchanged.
4. **If scenario 46 still shows a material residual after the fix** (e.g. the keepRight-back on e_det2
   is also involved), report it and localise the next facet rather than chasing it.

## 6. Explicitly deferred (unchanged)

- Cooperative LC (LCA_COOPERATIVE / `informFollower`) — a separate, heavier gap (P2G-2), not scenario
  46's residual; pursue only if a scenario proves it binding (the saturated-grid follower-block case).
- SUMO's terminal-edge nonzero best-lane offsets for unequal-length terminal lanes (`ComputeBestLanes`
  base-case simplification) — no committed anchor needs it.
