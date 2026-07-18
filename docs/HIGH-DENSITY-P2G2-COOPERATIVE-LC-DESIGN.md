# HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md — coordinated dense lane-change model (config-gated)

**Status:** ANALYSIS + PLAN (owner decision 2026-07-17: implement, but **behind a configuration gate**
so it can be toggled on/off when tuning performance). Item **P2G-2**. This is the coordinated-dense-LC
model that unblocks the faithful multi-lane lane-change work (P2G-3 and the P2-G follower veto) without
gridlocking saturated traffic. Grounds in `sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp`
(`informFollower`/`informLeader`/`amBlockingFollowerPlusNB`/`saveBlockerLength`, the `myVSafes` channel).

## 1. The analysis — why this exists, what it buys, what it costs

### Is the coordinated LC model required for FLOW? NO.
The engine **already flows dense multi-lane traffic without it** — the saturated `-L2` grid drains at
**0 stuck** on the current default; the 2-lane grid ran 74/75 arrived, 0 gridlock. The engine sits in a
self-consistent regime: **modest lane-changing + no coordination → flows**. SUMO sits in a different
stable regime: **aggressive lane-changing + coordination → flows**. The gridlocks this project hit
(follower-veto 0→30 stuck, keep-right continuation 0→90, P2G-3 cross-junction speedGain 0→51 — all in
`docs/HIGH-DENSITY-P2G3-DESIGN.md` §5.3) came from a THIRD, unstable regime: **aggressive lane-changing
+ NO coordination → thrash → jam**. Cooperative LC is the coordination that lets the faithful aggressive
lane-changing flow. **So it is required for PARITY (SUMO-faithful lane distribution), not for flow.**

### What we GAIN
- **SUMO-faithful multi-lane overtaking / lane distribution** — use-the-fast-lane, merge back right,
  cooperative merging. Closes scenario 46's residual and makes multi-lane trajectories track SUMO. This
  is **fidelity/realism**, most valuable for the **on-camera hero area** (X1) where the visible traffic
  should look exactly like SUMO.
- **Real capacity gain, but topology-specific:** cooperative gap-creation genuinely raises the jam
  threshold at **merges / on-ramps / lane-drops** (prevents merge-bottleneck breakdown). Real dense
  SumoData sub-areas have these. For **grid-like nets it buys little** — the engine already flows those.
- It is **NOT** a general "higher density without jam" lever — the density levers are already built
  (rerouting P1-E, teleport valve P1-F, X1 off-camera popping). Cooperative LC does not raise the
  *global* knee for grid traffic.

### What we LOSE
- **Performance:** a moderate per-step cost in the lane-change phase — the cooperative decision
  (`amBlockingFollowerPlusNB`) + a **cross-vehicle speed-advice channel** (`informFollower` → a
  per-vehicle imposed-speed field the car-following reads next step). **Inert on single-lane**, active
  only in multi-lane, fits the existing command-buffer/deferred-write pattern → gateable + localized,
  likely single-digit-% overall. **This is the reason for the config gate** (§2): turn it off to reclaim
  the perf when the faithful lane distribution is not needed.
- **Complexity + regression risk:** the big cost. Deep RoW/LC-core work with cross-vehicle coupling; the
  dense path is violently sensitive (three 0→30/90/51-stuck regressions this project). Multi-session
  effort with a high adversarial-testing bar.

### Product read
For **"a dense sub-area that flows and pops invisibly off-camera"** the engine ALREADY does that —
cooperative LC adds nothing needed. For **"on-camera multi-lane traffic that overtakes/merges exactly
like SUMO"** cooperative LC is the way, bought with perf + careful core work. It is a
**realism/fidelity investment for the visible hero area**, not a capacity enabler.

## 2. The config gate (owner requirement)

A single master switch, default **OFF** = today's byte-identical flowing behaviour; **ON** = the
coordinated dense LC model (cooperative changes + speed-advice channel) AND the currently-blocked
faithful pieces it unblocks (P2G-3 cross-junction speedGain, the P2-G follower-veto half).

- **Runtime Engine property** (like X1's controls — a non-parity behavioural mode, not a sumocfg parity
  key): e.g. `Engine.CoordinatedLaneChange { get; set; } = false;`. Default off keeps every committed
  golden byte-identical AND the saturated-grid diagnostic at 0 stuck.
- Owner steer #1 alignment: parity-faithful-but-slower is the OPT-IN here (inverse of the usual
  fast-mode flag, because in this engine the faithful path is the more expensive one). Document clearly.
- When ON, the parity target is behavioural/statistical on dense nets (not bit-exact — the coordinated
  model is where SUMO's exact per-vehicle draw order would otherwise lock us in); keep a moderate-density
  anchor (e.g. scenario 46) bit-exact under the flag as the faithful anchor.

## 3. Scope when built (design outline — not yet implemented)

1. **Speed-advice channel (`myVSafes` analog):** a per-vehicle deferred "imposed speed cap" written by
   one vehicle's LC decision and consumed by another's car-following NEXT step. Determinism: order-
   independent (MIN of all advice), published via the command-buffer pattern → parallel-safe.
2. **`informFollower`/`informLeader`:** a changing/blocked vehicle asks its target-lane follower to slow
   (writes advice) / adjusts for the leader (`MSLCM_LC2013.cpp:430-740`).
3. **Cooperative change (`amBlockingFollowerPlusNB`, `:1639-1659`):** a vehicle moves over to make room
   for a blocked neighbour (LCA_COOPERATIVE).
4. **Unblock the gated pieces under the flag:** P2G-3 cross-junction speedGain (the
   `TryFindContinuationLeader` change, `docs/HIGH-DENSITY-P2G3-DESIGN.md` §5.2) and the P2-G follower-veto
   half — both currently reverted because they gridlock saturation WITHOUT this coordination.
5. **Gate — adversarial:** the saturated-grid diagnostic must stay at 0 stuck with the flag ON (the
   whole point), and byte-identical with it OFF. Suggested spike: build the `informFollower` speed-advice
   half FIRST and re-run the reverted P2G-3 change under it to measure whether it clears the 0→51 gate and
   what it costs — cheapest signal on feasibility + perf.

## 3.5 SPIKE RESULT (session 3) — the architecture is VALIDATED

The informFollower speed-advice spike (§3.1-2) is built behind `Engine.CoordinatedLaneChange` and measured:

| | saturated `-L2` grid (gate ≤5 stuck) | scenario 46 first-divergence |
|---|---|---|
| P2G-3 faithful LC, NO coordination | **51 stuck** (why P2G-3 couldn't land) | lane fixed at t=91 |
| **+ cooperative informFollower (gate ON)** | **0 stuck** ✅ | lane fixed (t=91→ residual at t=92) |
| gate OFF (default) | unchanged | byte-identical |

**Headline: the cooperative speed-advice coordination CLEARS the 0→51 saturated-grid wall — the faithful
aggressive lane-changing now flows the dense grid at 0 stuck, like SUMO.** This is the proof the
coordinated-LC architecture is right: coordination is what lets the faithful lane-changing flow. Landed
as a gated foundation:
- **Speed-advice channel:** `VehicleRuntime.CoopSpeedAdvice` (+∞ = none) + a `CommandBuffer.SpeedAdvice`
  op applied as a commutative MIN (parallel-order-independent), consumed as a `vPos` cap in
  `ComputeMoveIntent` and cleared on the real pass. `informFollower` writes it from the blocked-speed-gain
  path (`DecideSpeedGainForVehicle`). P2G-3 (continuation distance + cross-junction leader) re-applied,
  all gated on `CoordinatedLaneChange`.
- **Gate OFF = byte-identical** (full parity suite green, 585 prior unchanged); 3 functional tests
  (`RungHDp2g2CoordinatedLaneChangeTests`) pin: coordinated grid flows (≤5 stuck), coordinated scn46 f.1
  takes the fast lane `e_det1_1`, default keeps it slow.

### Remaining to make the gate-ON path BIT-EXACT (next iterations — not blocking the foundation)
1. **Scenario 46 downstream residual:** the lane choice at t=91 is fixed, but a downstream divergence
   remains (a large late max-pos-err + a residual lane mismatch) — chase the cascade (likely a subsequent
   lane decision / the neigh-side cross-junction leader / the exact SUMO `informFollower` helpDecel
   formula vs the spike's follow-speed approximation).
2. **Full SUMO `informFollower`/`informLeader`** (the `helpDecel`/`HELP_OVERTAKE` formula,
   `MSLCM_LC2013.cpp:645-740`) replacing the spike's follow-speed advice.
3. **Cooperative CHANGE decision** (`amBlockingFollowerPlusNB`, `:1639-1659`) — a vehicle moving over to
   help, not yet built.
4. **Keep-right follower veto + informFollower** (the P2-G follower half) under the same gate.
5. **A bit-exact anchor + golden** under the flag, and a **perf measurement** of the gate-ON LC phase.

## 3.6 PERFORMANCE MEASURED (session 3) — the gate-ON cost is negligible-to-neutral

Measured with `Sim.BenchCity --coordinated-lc` (the flag added for this), gate-OFF vs gate-ON:

| scenario | gate OFF | gate ON | note |
|---|---|---|---|
| dense saturated `-L2` grid (412 veh, 700 steps) | ~1.6-1.7 s, ~410-495x RTF | ~1.2-1.6 s, ~426-577x RTF | both drain 411/412, 0 stuck; ON **≈ neutral / slightly faster** |
| light `-L2` grid (75 veh, 600 steps) | ~0.38-0.48 s | ~0.30-0.41 s | both 74 arrived, 0 stuck; delta **within noise** |

**Finding: the coordinated model is NOT a performance regression.** The extra per-vehicle LC work
(memoized `BestLanesCached`, a bounded `TryFindContinuationLeader` scan, rare `informFollower`) is cheap;
on dense traffic the improved flow *offsets it entirely* (fewer vehicle-steps of congestion). The engine
runs 400-2000x realtime in both modes. So the config gate's original perf worry is largely moot — ON is
a cheap opt-in, not a slow one. (Default stays OFF only to preserve the byte-identical parity anchor, not
for speed.)

### Direction update (owner steer, 2026-07-18): bit-exact parity is NOT the goal
Smooth + believable flow + high performance are. The coordinated mode already delivers all three (flows
dense at 0 stuck, SUMO-like overtaking, perf-neutral), so the §3.5 "make gate-ON bit-exact" iteration
list is DE-PRIORITISED — pursue those only if a believability defect (not a parity delta) shows up. The
scenario-46 downstream divergence is a parity delta, not a believability defect (all vehicles flow +
arrive, 0 stuck), so it is acceptable.

## 4. Tracked as
`docs/HIGH-DENSITY-PLAN.md` P2G-2 (config-gated). Foundation + spike + perf LANDED; the coordinated mode
is a believable, perf-neutral opt-in. Bit-exact gate-ON is de-prioritised per the owner steer above.
