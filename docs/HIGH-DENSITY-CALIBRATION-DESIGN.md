# HIGH-DENSITY-CALIBRATION-DESIGN.md — make SumoSharp a trustworthy high-density calibration engine

**Status:** DESIGN (this session, 2026-07-21). Owner asked for all three gaps fixed so SumoSharp can
**auto-calibrate the highest believable traffic density** (the sub-area pipeline's "knee"), not just
serve/run. **Self-contained on purpose** — written so the conversation can be compacted and a later pass
can implement from these docs alone. Companions: `HIGH-DENSITY-CALIBRATION-TASKS.md` (stages + success
conditions), `HIGH-DENSITY-CALIBRATION-TRACKER.md` (checklist), `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`
(Gap 1 deep-dive with measured evidence). Source NEEDs (from the SumoData session):
`SUMOSHARP-NEED-dense-flow-gridlock-vs-vanilla.md`, `SUMOSHARP-NEED-serve-calibration-parity-gaps.md`.
SUMO reference is vendored read-only at `/sumo` (v1_20_0); `SUMO_VERSION`=1.20.0; `sumo` binary present.

---

## 0. The north star and why these three
The sub-area pipeline (`preprocess.py`/`auto_calibrate.py`, in the *SumoData* repo, not here) computes the
**maximum believable insertion density** for a crop by probing densities until the teleport pop% exceeds a
budget — the "knee". It swaps the sim engine via `SUMO_BINARY`. Today it must calibrate with **vanilla
`sumo`** because SumoSharp:
- **Gap 1 (HIGH):** gridlocks/teleports at **3–5× lower density than vanilla** on the identical net —
  so its calibrated knee is 4–5× too low and untrustworthy. THE headline blocker.
- **Gap 2 (MEDIUM):** rejects the full pre-built `demo_city/box` at load — a `roadsideCapacity=1`
  parkingArea referenced by 3 vehicles across the run (static load-time lot assignment, no time reuse).
- **Gap 3 (LOW):** rejects `departPos="base"` (30 vehicles in the box use it).

Gap 1 is the behavioral blocker. Gaps 2+3 are load-time blockers that stop the **full box** loading on
SumoSharp at all (the *crop* flow already runs because it regenerates `departPos="stop"` demand and doesn't
oversubscribe capacity-1 lots). All three are needed for "SumoSharp-only auto-calibration on the full box".

## 1. Reproduction (deterministic; keep for every pass)
Substrate: `scenarios/_repro/synthetic-junction2` (committed throughput witness: TL + priority junctions,
heavy demand on short approaches; regenerate with its `build.py`). Drop-in binary: `src/Sim.Sumo`
(`sumosharp.dll`), SUMO-arg compatible.
```
# build the drop-in
dotnet build -c Release src/Sim.Sumo/Sim.Sumo.csproj
DLL=src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll
# 2x-density stress demand (compress departs; amplifies Gap 1 into clear gridlock):
python3 -c "import re;s=open('scenarios/_repro/synthetic-junction2/scenario.rou.xml').read();\
open('/tmp/dense.rou.xml','w').write(re.sub(r'depart=\"([0-9.]+)\"',lambda m:f'depart=\"{float(m.group(1))*0.5:.2f}\"',s))"
# copy net + support files to /tmp and point a /tmp/dense.sumocfg at /tmp/dense.rou.xml (see diag doc).
sumo      -c /tmp/dense.sumocfg --end 1000 --statistic-output /tmp/v_stat.xml --tripinfo-output /tmp/v_trip.xml --summary-output /tmp/v_sum.xml --no-step-log true
dotnet $DLL -c /tmp/dense.sumocfg --end 1000 --statistic-output /tmp/s_stat.xml --tripinfo-output /tmp/s_trip.xml --summary-output /tmp/s_sum.xml --no-step-log true
# compare: <teleports total/> ; count <tripinfo ; summary running/halting/meanSpeed over time.
```
**Measured today (main `8bb8219`, 2× density):** vanilla = 0 teleports / 290 arrivals / halting drains to
0; SumoSharp = 10 teleports (8 *yield*) / 275 arrivals / **~45 permanently stuck (meanSpeed 0 from
t=600)**. The offline gate (`dotnet test Traffic.sln`, no SUMO/network) must stay green throughout.

---

## 2. GAP 1 — reroute-on-wrong-lane (the dense-flow gridlock). PRIMARY.

### 2.1 Root cause (measured — full evidence in `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`)
Under dense insertion, a vehicle that needs to change into a **specific route-required exit lane** at a
junction, but cannot complete that lane change in the dense queue, reaches the junction **on the wrong
lane** — a lane with **no connection to its next route edge**. SumoSharp then **clamps it at the lane end
(stops it dead, speed 0, forever)**. The stalled front vehicle blocks its whole approach queue; a handful
across the net cascade into gridlock + yield-teleports. Vanilla instead takes the connection its actual
lane offers and **reroutes** (`device.rerouting` on) — every stalled SumoSharp vehicle (295/242/155/172/
316) arrives in vanilla; none in SumoSharp. Concrete: veh 295 route `…30→124…`, but `→124` connects only
from lane `30_0`; 295 is stuck on `30_1` (pos 24.1, speed 0, t=520→680) while junction 29 sits **idle**
(0 crossers). It is NOT yield/RoW (junction empty), NOT car-following, NOT the landed overlap fix (A/B:
pre-fix main gridlocks the same — the overlap fix costs only ~6 veh here from its arrival-lane junction
braking; watch but not the driver).

### 2.2 The exact fix locus
`Engine.cs` → `TryReResolveFromActualLane(v, currentLane)` (~line 9051), called from `ExecuteMoveVehicle`
(~8960-9013) when a vehicle reaches a lane end on a lane ≠ its pool exit lane. Line ~9080:
```csharp
// Genuine drop lane: the actual lane has no connection to the next route edge -> clamp.
if (!_network!.ConnectionsByFromLaneTo.ContainsKey((currentLane.EdgeId, currentLane.Index, remaining[1])))
{
    return false;   // <-- caller then CLAMPS: v.Pos = laneLength; v.Speed = 0; (permanent stall)
}
```
This `return false` is where veh 295 dies. There is a SECOND path to the same stall: a vehicle can halt
at the stop line *before* reaching the lane end if its lane offers no link toward its next route edge
(planning finds no valid outgoing link) — so the reroute must be reachable from the **approach**, not only
at the physical lane end.

### 2.3 HOW (design)
Mirror vanilla: when a vehicle's current lane cannot reach its next route edge, **reroute via a
connection the current lane DOES have**, instead of clamping. Concretely, a new helper
`TryRerouteFromDeadLane(v, currentLane)`:
1. Enumerate the outgoing connections `currentLane` actually has (`ConnectionsByFromEdgeLane[(edgeId,
   laneIndex)]` → candidate next edges, e.g. `30_1 → {44, 112, -30}`).
2. For each candidate next edge, ask the router (`_router.Route(candidateNextEdge, destEdge, avoid)` — the
   same `NetworkRouter`/astar `UpdateReroutes` uses) whether the destination is reachable. Pick the
   candidate whose `currentEdge → candidateNextEdge → …dest` is the cheapest/best (matches SUMO's
   "best link" pick, `MSLane::getLinkTo`/best-lanes continuation; a simple router-cost min is adequate and
   deterministic).
3. Splice the new route (`currentEdge` + routed tail) into the pool via the existing `ReplaceRoute` +
   `_laneSeqPool.AddRange` discipline (`RegisterRerouted` keeps `_routesById`/best-lanes in sync), exactly
   as `UpdateReroutes`/`TryReResolveFromActualLane` already do — pin this edge's exit to `currentLane.Index`
   (`forceFirstExitToArrival:true`) so the vehicle proceeds via the connection its lane has.
4. Only if NO candidate connection routes to the destination (a true dead end) fall back to the clamp.
Wire it at BOTH: (a) the `return false` at ~9080 (lane-end drop), and (b) the approach — add a
plan-phase check so a vehicle whose current lane has no link toward its next route edge triggers the same
reroute *before* it stop-line-halts (so a stalled front vehicle recovers within a step, not never). Gate
the approach-side trigger to fire only when a strategic change onto the exit lane is no longer possible
(e.g. within N metres of the junction and blocked), so it does not pre-empt a normal, completing lane
change.

**Determinism / parity:** the router is a pure function of the immutable net + edge weights; the reroute
uses the established command-buffer/pool-append discipline (order-independent). No committed golden reaches
the drop-lane clamp today (they never strand — verified byte-identical before this session's overlap fix),
so replacing clamp→reroute is byte-identical on all goldens. Gate = full `dotnet test Traffic.sln` green +
`Sim.Bench` determinism hash unchanged. If any golden moves, the trigger is over-firing — tighten it.

**Escalation (only if 2.3 is insufficient):** SUMO also *completes* the exit-lane change more reliably via
cooperative lane-changing (target-lane followers open a gap — the retired `informFollower`,
`HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`). Prefer the reroute (cheap, matches vanilla's observed
behavior here); revisit cooperative LC only if reroute alone leaves material gridlock. Also consider
earlier/longer best-lanes lookahead so the exit-lane change starts far enough upstream to complete.

### 2.3.1 MEASURED (2026-07-21): reroute-only is INSUFFICIENT — escalate to lane-completion (2.3 escalation)
Candidate 1 (reroute-on-dead-lane) was fully implemented and measured on the 2× dense synthetic, then
**reverted** — it is a band-aid over a deeper lane-assignment problem, not a clean fix:
- **It does help the dense case:** replacing the drop-lane clamp with a reroute (via a connection the
  current lane has, costed by the LIVE smoothed edge weights `_edgeWeights.Effort` — free-flow cost sent
  cars back into the jam and *inflated* teleports 5→16) drained the 2× gridlock fully (halting 45→34,
  meanSpeed 0→~8, arrived ≈292 ≥ vanilla's 290). So the diagnosis (clamp = gridlock driver) is confirmed.
- **But it over-fires and can loop, regressing low density.** At 1× density the reroute fired for **87–109
  vehicles** (not "a handful") and some vehicles **looped forever**: veh 58, stuck on `109_1`, reroutes →
  its every path to the destination goes back through `109` and it lands on `109_1` again → reroutes …
  (308× with free-flow, 140× after a U-turn/reverse-edge skip; avoiding the current edge in the tail broke
  the loop but forced worse paths that wedged). Net at 1×: teleports **5→12–21**, arrivals 287→268–284.
  Because it **raises the low-density teleport floor**, it *hurts* the calibration knee (calibration stops
  early when teleport% is already high at low density) — the opposite of the goal — and it fails the
  committed `LowDensityTeleportTests` guard (≤5).
- **Root finding — it is a lane-completion problem, not a routing one.** veh 58 keeps landing on `109_1`
  and can never reach `109_0` (the lane its route needs) nor avoid `109`; no reroute can fix that. In
  vanilla the car simply *changes onto `109_0`* (its best-lanes/cooperative-LC is not route-locked and
  commits earlier). So the real fix is **candidate 2/3**: complete the strategic exit-lane change /
  commit to the route's exit lane far enough upstream, so the car is on the RIGHT lane at the junction
  and never strands. The reroute should remain only as a **last-resort fallback** for a genuinely
  unreachable exit lane (a true topology drop), gated so it fires for a *handful*, not ~100 cars.
- **Reverted artifacts (for the next pass):** the reroute lived in `Engine.cs` as `TryRerouteFromDeadLane`
  (wired into `TryReResolveFromActualLane`'s drop-lane branch), `EdgeFreeFlowCost`, using
  `_edgeWeights.Effort` + a U-turn skip. Determinism/parity were sound (route-dict writes lock-guarded,
  never read during the region-parallel execute; inert for every committed golden). It is the *policy*
  (reroute vs complete-the-LC) that was wrong, not the plumbing.

### 2.3.2 MEASURED passes 2-4 (2026-07-21): diagnosis + a SAFE gated reroute LANDED; full drainage still open
- **Try 2 (diagnosis).** Traced the actual stuck cars in both engines. At 1× vanilla AND SumoSharp both
  COMPLETE the tight exit-lane change (veh 58 enters `109_1` — forced by the sole `69→109` connection —
  and changes to `109_0` within the 10.83 m edge, then turns to 173): at 1× the baseline is already fine
  (5 tp / 287 arr). At 2× the change genuinely can't complete (target lane full): veh 295 enters `30_1`,
  needs `30→124` which leaves only from `30_0`, never gets a gap, and clamps at the lane end from t=428 to
  699. **Vanilla does NOT complete veh 295's change either — it REROUTES** (leaves `30_1` via `30_1`'s own
  connection, arrives t=412). So the reroute is the right vanilla-faithful move for a truly-blocked dense
  car; the instant-reroute failure at 1× is a CASCADE (rerouting the ~8 genuine strands perturbs SumoSharp's
  fragile substrate → dozens more strand → some loop). vanilla doesn't cascade because its LC/junction
  behaviour has margin.
- **Try 3-4 (the landed fix).** Make the dead-lane reroute a LAST RESORT: a car only reroutes after it has
  actually been clamped/blocking for `DeadLaneRerouteWaitSeconds` (5 s) — like vanilla's periodic 30 s
  cadence, not instantly at the first lane-end touch — plus a U-turn (reverse-edge) skip and a per-car
  `MaxDeadLaneReroutes` cap (both hard anti-loop bounds). Measured: **1× stays EXACTLY at baseline
  (5 tp / 287 arr, ≤5 guard green); 2× improves — teleports 10→3, arrivals 275→281, halting 45→42.** Full
  suite 656 green, all committed goldens byte-identical (inert — no golden strands), determinism holds
  (route-dict + count writes lock-guarded, never read during the region-parallel execute). Code:
  `Engine.cs` `TryRerouteFromDeadLane` + `EdgeFreeFlowCost` + `_deadLaneRerouteCount`, wired into
  `TryReResolveFromActualLane`'s drop-lane branch; live-weight cost via `_edgeWeights.Effort`.
- **STILL OPEN (the real Gap-1 finish).** 2× does not FULLY drain (~7 cars stay stuck, meanSpeed 0): the
  same gate that protects 1× is too slow to stop the 2× jam forming (an instant reroute drains 2× but
  cascades 1× / fails the guard — the two cannot be reconciled by tuning the reroute alone). Full drainage
  needs the ROOT fix below.

### 2.3.3 MEASURED pass 5 (2026-07-21, branch `gap1-lane-completion`, discarded): the wall is quantified
Went straight into the "root" fix on a throwaway branch. Result: the reroute genuinely cannot reconcile
1× and 2×, and the obvious root fix is contraindicated. Evidence:
- **Cooperative LC (`informFollower`) is a DEAD END here.** `HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`
  records it was RETIRED: its only benefit (saturated-grid rescue) was obsoleted by the P2-G junction
  fixes, and it *degrades* organic flow (+perf cost too). Re-adding it would most likely hurt, not help.
- **Minimal-deviation reroute is not what vanilla does.** Traced vanilla veh 295's full path: it diverges
  at 30 onto a COMPLETELY different corridor (`44 55 69 109 173 -1119 -508 506`) and rejoins the original
  route only much later at `828`. So vanilla reroutes FULLY (live-weight optimal), not minimally — yet it
  does not cascade, because it reroutes SMOOTHLY (the car crosses `30_1→44` while still moving; it never
  stops on edge 30). SumoSharp holds the car until it stops (queue blocks), then reroutes.
- **The 1× churn is a real, quantified cascade (net −14 arrivals).** With an INSTANT reroute at 1×, vs the
  no-reroute baseline (287 arr): the reroute SAVES 3 genuine strands (152/262/317) but LOSES 17 previously-
  fine cars (58/76/87/89/102/109/122/146/147/156/170/174/220/232/234/238/246) → **273 arr, a net loss.**
  The lost cars arrive fine WITHOUT the reroute; rerouting the ~3 genuine strands perturbs SumoSharp's
  fragile substrate into ~87 dead-lane hits, most harmful churn. vanilla perturbs just as much (full
  alternate paths) but does NOT churn because its cars complete their LCs robustly.
- **No gate value threads the needle.** Sweep: instant (0 s) churns 1× (−14 arr); any gate ≥1 step gives a
  clean 1× (baseline) but the 2× jam persists (the gated front car unsticks ~1 car/s — slower than dense
  arrivals — so the queue never clears). Instant is required to drain 2×; a gate is required to keep 1×;
  they are mutually exclusive **for the reroute alone**.

**Conclusion:** Gap 1's clean finish is NOT reachable by the dead-lane reroute (the landed gated version is
the safe optimum of that lever) NOR by re-adding cooperative LC. It needs a genuinely different attack on
the *substrate fragility* — why a small perturbation tips SumoSharp cars into dead lanes when vanilla's
don't. Candidates for a future focused pass, none cheap: (a) a smarter genuine-strand detector that fires
the INSTANT reroute for only the truly-stuck front cars (not the transient churn) — the WaitingTime proxy
is too slow; (b) make the exit-lane change actually complete on short approaches (best-lanes lookahead so
the change starts on the PRIOR edge, since the forced-entry lanes like `30_1`/`109_1` give only ~10–24 m);
(c) let a dead-lane car cross via its lane's connection WHILE MOVING (never hold it at the stop line), so
no queue forms — requires intercepting the junction "no-valid-link" brake, parity-sensitive. Each is a
multi-session, golden-risk effort; the gated reroute stands as the safe, non-regressing partial.

**Next pass (candidate 2/3) — the root, not the symptom.** SumoSharp's substrate is *fragile*: cars that
should complete their exit-lane change under density don't, so they strand, and any reroute of them
cascades. Fix the completion itself: (a) start the strategic exit-lane change earlier (best-lanes
lookahead so it begins far enough upstream on short approaches like the 10.83 m edge 30/109), and/or (b)
complete it under occupancy via a **cooperative gap** — a target-lane follower briefly yields so the
mandatory change lands (SUMO's `informFollower`/LCA_COOPERATIVE, retired in
`HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`). With the substrate robust, far fewer cars strand, the
cascade disappears, and the instant reroute (or no reroute at all) drains 2× without touching 1×. Keep the
landed gated reroute as the bounded last-resort fallback. Re-measure teleports/halting at BOTH 1× (≤5) and
2× (drain, ≈vanilla) every iteration; this is parity-sensitive (touches strategic LC) so guard the goldens.

### 2.3.4 THE ACTUAL SUMO MECHANISM (2026-07-21, read from /sumo source — supersedes the guesses above)
Course-correction (owner prompt): stop inventing heuristics that *reproduce* vanilla's results; **read the
SUMO source and port what it actually does** (CLAUDE.md rule 4). Doing so overturns the earlier hypotheses:
- **It is NOT congestion rerouting.** vanilla's own `--device.rerouting.output` at the decision instant
  shows the `124` corridor is CHEAPER than the `44` corridor (edge 124 traveltime 1.74, 148 = 0.29 vs
  edge 44 = 3.16, 55 = 3.92). A cost-based rerouter would KEEP 124. So congestion weights are not what
  sends veh 295 down `44`. (SumoSharp's periodic rerouter is a faithful port — `RerouteEdgeWeights` +
  `UpdatePeriodicReroutes` — and correctly keeps 124 too; it is not the bug.)
- **It IS `ignore-route-errors` + lane-continuation.** SUMO plans each vehicle's move along the
  continuation of **its actual current lane** (`MSVehicle::getBestLanesContinuation`, used to build
  `myLFLinkLanes` in `planMoveInternal`), NOT along the route's ideal lane. So veh 295, stuck on `30_1`,
  simply plans `30_1 → 44` (the connection its lane HAS) and drives across the junction **while still
  moving** — it never waits for the impossible `30_1 → 124`. `MSVehicle::executeMove`
  (`/sumo/src/microsim/MSVehicle.cpp:4344-4346`) only hits *"there is no connection to the next edge"*
  (→ emergency/teleport) when the lane has NO onward link at all; a lane with SOME link just follows it.
  The `device.rerouting` device then re-syncs the now-off-route vehicle to a valid route from `44`
  onward — which is why turning rerouting OFF gives 10 teleports (the car goes off-route via `44` but has
  no device to repair the route): **measured vanilla rerouting off = 10 tp / 289 arr, identical to
  SumoSharp; rerouting on = 0 tp / 290 arr.** Both mechanisms are needed: lane-continuation for the smooth
  cross, rerouting to repair the route after.
- **SumoSharp's exact gap:** it PINS the pool exit lane (`30_0` for `124`) and HOLDS the car when it can't
  converge onto it, so the car stops at the junction, blocks its `30_1` queue, and gridlocks. It should
  instead do what SUMO does — plan the crossing along the car's ACTUAL lane's connection (`30_1 → 44`)
  when the pool lane is unreachable, so the car flows through, THEN let the (already-faithful) reroute
  repair the route. This is the real, SUMO-faithful Gap-1 fix, and it explains why the reactive dead-lane
  reroute is only a partial patch (it fires at the lane end AFTER the car has already stopped/queued,
  reproducing the *destination* but not the *smooth-cross* that prevents the jam).

**THE fix to implement (SUMO-faithful, replaces candidates a/b/c above).** Make the vehicle's junction
continuation follow its ACTUAL lane when that lane cannot reach the next route edge — i.e. resolve the
lane-sequence pool / best-lanes continuation from the current lane's real connections (port
`getBestLanesContinuation`'s "continue along the lane you are on" semantics), rather than pinning the
route's ideal exit lane and holding. Then the vehicle plans a valid link (`30_1→44`) in the PLAN phase and
crosses while moving; the existing periodic reroute repairs the route from the new edge. Parity guard: for
every committed golden the car IS on its pool exit lane at each junction (they never strand), so
"continue along the actual lane" == "continue along the pool lane" — byte-identical. Locus: the pool /
continuation resolution feeding `TryStrategicLaneChange` + the junction link selection in Plan, and the
boundary-crossing in `ExecuteMoveVehicle`; the reactive `TryRerouteFromDeadLane` can then be simplified to
just the route-repair (or removed in favour of the periodic reroute). Measure 1× (≤5) and 2× (drain) and
diff every golden.

### 2.3.5 SOLVED (2026-07-21, session 2, branch `claude/dense-lane-overlap-fix-5tr4ha`) — vanilla parity
The §2.3.4 mechanism was implemented and it **cracks the gridlock**: 2× dense synthetic now drains to
**0 teleports / 290 arrivals**, exactly matching vanilla SUMO 1.20.0 (was 10 tp / 275 arr / ~45 stuck).
1× stays clean (**5→1 teleport / 290 arr**, guard `<=2`). Full suite green (657 pass), every committed
golden byte-identical, deterministic (two runs identical; serial == `--max-parallelism 8`).

**Why the earlier reactive-reroute passes stalled, and what changed.** Measuring the instant (gate=0)
reroute again confirmed it churns BOTH densities (2× tp 3→**18**, 1× tp 5→**18**) — the churn is NOT from
stopping (gate=0 crosses at speed yet still churns) but from **too many cars spilling onto the alternate
corridor** because SumoSharp's LC never converged them onto the good (through) lane. That is the missing
piece §2.3.4 named: SUMO's URGENT strategic change **brakes to find a gap** (`MSLCM_LC2013::informLeader`,
`stopSpeed(myLeftSpace)`); SumoSharp's strategic LC only ever changed-or-gave-up (the secure-gap veto),
so dense cars piled onto the dead lane at full speed and hard-deadlocked.

**The fix that landed — three faithful parts, all gated on the car being on a DEAD LANE** (its current
lane has no connection to its next route edge), a state no committed golden vehicle is ever in ⇒
byte-identical for every golden by construction:
1. **`DeadLaneMergeBrakeConstraint`** (plan phase, new): a dead-lane car decelerates via
   `StopSpeedFor(usableDist)` toward the forced-merge point instead of barreling to the lane end. Port of
   `informLeader`'s `stopSpeed(myLeftSpace)` (myLeadingBlockerLength = 0). This lets it fall back and
   re-try the strategic merge onto its through lane each step (and stops the hard slam-and-clamp). ALONE
   this took 2× from 275→**290** arrivals (full drainage) but left 7 teleports at 1× / 3 at 2× (yield
   teleports at TL junctions like `-2437_1`→`-2337`, where the brake concentrates cars that then never
   reach the lane end to trigger the boundary reroute).
2. **Boundary reroute** (existing, `DeadLaneRerouteWaitSeconds = 5`): a dead-lane car that DOES reach the
   lane end crosses via its actual lane's connection and reroutes (getBestLanesContinuation semantics).
3. **`TryRerouteStuckDeadLane`** (execute phase, new, `StuckDeadLaneRerouteWaitSeconds = 90`): a dead-lane
   car held short of the lane end by a junction yield / red light — which never reaches the boundary —
   reroutes onto its actual lane's connection **just before time-to-teleport (120 s)**. A SEPARATE, LONGER
   gate than the boundary path is essential: swept {5, 60, 90, 110} s — an eager stuck-reroute churns the
   dense case (gate=5 → 2× tp 3→6), while 90 s diverts ONLY the cars that would otherwise jam-teleport,
   leaving the rest to drain. gate=90 gives 2× **0 tp / 290 arr** and 1× **1 tp / 290 arr**.

Anchor committed: `scenarios/_repro/synthetic-junction2/scenario.dense.{rou,sumocfg}` (325-vehicle 2×
demand) + `tests/Sim.ParityTests/DenseFlowDeadLaneDrainTests.cs` (asserts 0 tp / ≥290 arr offline via
SumoShim). `LowDensityTeleportTests` tightened `<=5`→`<=2`.

**Still open (Stage 4, broader):** the full `demo_city/box` runs end-to-end but is not yet at parity
(SumoSharp 2 tp / 22 arr vs vanilla 0 / 36 at t=800) — additional issues beyond this dead-lane gap
(other junction/pedestrian/parking dynamics), a separate calibration effort. Validated the dead-lane fix's
box impact directly (toggle both new mechanisms off): halting **396→365** (better), arrivals **21→22**
(+1), teleports **0→2** (the stuck-reroute diverts a couple of box dead-lane cars into spots that later
teleport). So the fix is net-neutral-to-slightly-positive on the box — correct, but the box's gap is
**dominated by non-dead-lane causes**: its extra halting (365 vs vanilla's 165, ~200 cars) is DIFFUSE
(worst single edge only ~6 halted cars at t=790, spread across rings/diagonals/garage stubs), not the
concentrated dead-lane deadlock the synthetic isolates. Stage 4 = chase those diffuse junction/flow gaps
(and, minor, the box's +2 dead-lane-reroute teleports) separately.

### 2.3.6 STAGE 4 — box throughput CRACKED (2026-07-22, session 2). Superseding the "diffuse" note above.
Re-diagnosed the box at a longer horizon (t=2500; the t=800 "diffuse" read was warm-up — only 36/861
arrive by t=800 even in vanilla). The gap was NOT diffuse: SumoSharp arrivals **plateaued at ~47 after
t=1500** while vanilla drained to **96**, and the entire deficit localized to cars destined for the **ring
N/W/E fringe exits** (dead-end sinks): SumoSharp 4/4/2 vs vanilla 24/18/14. Two arrival/exit bugs, both
fixed byte-identical (full suite 657 green, deterministic serial == parallel):

1. **Final-edge arrival was strict lane-CROSSING (`pos > laneLength`).** SUMO arrival is position-based and
   lane-agnostic (`MSVehicle::hasArrived`: remove at `pos >= arrivalPos`; default arrivalPos == laneLength).
   A car that braked to a halt EXACTLY at a dead-end exit's lane end (pos == length, on an outer lane with
   no onward connection) never satisfied the strict `>` and froze forever, backing up the whole exit queue.
   Fix (commit `cfc52fc`): a final-edge car sitting AT the lane end falls through to the arrival branch.
   Measure-zero in free flow (cars overshoot). Box 47 → 59.
2. **The park-and-stay residency guard (Issue-1) clamped the wrong cars.** It held ANY vehicle reaching its
   final edge with an unreached parkingArea front-stop (so a park-and-stay car isn't vanished mid-park) —
   but did not check WHERE the stop is. A box mall/garage car whose MID-ROUTE parking stop it drove past
   (see residual) reaches its real destination (a ring/roundabout exit) with that stop still unreached, and
   the guard clamped it there forever. Fix (commit `b02076f`): only clamp when the unreached stop is on the
   CURRENT edge (genuinely parking here); otherwise arrive. Box 59 → **106** (vanilla 96); ring-fringe sinks
   now drain to parity (23/18/14 vs 24/18/14).

**Isolation method that cracked it:** compare per-sink arrival counts SS vs vanilla (same 8 canonical
sinks) → the deficit is entirely at the ring fringes; trace ONE stuck car (`v2_mall_shop_3`) → frozen at
pos == laneLength speed 0 on a final-edge outer lane; log its executed speed → plan wants 16 m/s but the
boundary loop clamps it; log the clamp path → the residency guard's `{Reached:false, IsParking:true}` arm.

**RESIDUAL (open — Gap-2 / rerouting-with-stops).** SumoSharp now arrives ~10 MORE than vanilla (106 vs 96)
because the **dead-lane reroute routes to the FINAL destination and drops an intermediate parking detour**:
`TryRerouteFromDeadLane` sets `destEdge = remaining[^1]` and shortest-paths to it, so a car that hits an
early dead lane and has a mid-route parking stop (`pa_v2_mall_lot`, cap 24, 6 cars — not oversubscribed) is
rerouted AROUND the mall, skips parking, and arrives instead of staying resident (verified: with rerouting
even fully OFF the car visits 13 edges, none in the mall). SUMO reroutes BETWEEN stops and preserves them.
Fix = route the dead-lane reroute/re-resolve via the next unreached stop's edge first, then to the
destination. Deferred here: it must not regress the Gap-1 synthetic parity (§2.3.5) and wants its own
design pass. Note `--stop-output` is not implemented in SumoSharp, so parking must be verified via
trajectories/tripinfo, not stopinfo. Also minor: box teleports 3 vs vanilla 1.

### 2.4 Success criteria (Gap 1)
On the 2× dense synthetic: SumoSharp teleports ≈ 0 (was 10), no permanent gridlock (halting drains toward
0 like vanilla, not stuck at ~45), arrivals ≈ vanilla (≈290, was 275). Full `dotnet test Traffic.sln`
green + goldens byte-identical (or provably SUMO-faithful with a provenance bump). A new committed
`_repro` or `scenarios/` anchor that pins "wrong-lane vehicle reroutes instead of stalling".

---

## 3. GAP 2 — parkingArea time-based lot reuse

### 3.1 Root cause
`Engine.ResolveParkingAreaStops` (`Engine.cs` ~1223-1310) assigns each parkingArea occupant a **distinct**
lot index across the WHOLE run via a monotonic `nextLotIndexByPa` counter (two passes: departPos="stop"
origins first, then moving-vehicle stops, both in vehicle-list order). It has **no notion of a lot being
freed when its occupant departs**. So a `roadsideCapacity=1` lot referenced by 3 vehicles over the run is
assigned lot indices 0,1,2 → `ParkingArea.LotPosition` (`Sim.Ingest/ParkingArea.cs`) throws *"lot index 1
is out of range"* at load. This is a deliberate GAP-3 scope limit (`docs/SERVE-PATH-PLAN.md §3a`) — only
non-overlapping-in-time single-occupant shapes were in scope.

### 3.2 SUMO reference
`MSParkingArea::computeLastFreePos` assigns the **lowest-index currently-free lot** at the moment a vehicle
parks, and frees it when the vehicle departs (real-time turnover). A `roadsideCapacity=N` lot serves any
number of vehicles across time as long as **≤ N are parked simultaneously**.

### 3.3 HOW (design) — two options; recommend A
**Option A (faithful, dynamic runtime assignment) — RECOMMENDED for the calibration robustness goal.**
Move lot assignment from load-time to **runtime**: when a vehicle actually parks (reaches its
parkingArea stop), claim the lowest-index free lot of that area from a per-area free-set; when it departs
(stop ends), return the lot to the free-set. This is `MSParkingArea::computeLastFreePos` verbatim.
- New runtime state: per-parkingArea `bool[capacity] occupied` (or a min-heap of free indices), in Engine.
- On park (the stop-reached transition, where `IsParked` is set): claim `lowestFreeLot`; set the vehicle's
  parked lateral/longitudinal lot position from `ParkingArea.LotPosition(lotIndex)`; if the area is full,
  SUMO reroutes to an alternative parkingArea (`parkingAreaReroute`) or the vehicle waits — for scope,
  match SUMO's default (wait/queue on the lane) or reject only if genuinely unsatisfiable.
- On depart: free the lot.
- Determinism: claim/free in the deterministic per-step order the engine already uses; the free-set pick is
  "lowest index", order-independent. Keep the committed single-occupant parking goldens (48/66-70)
  **byte-identical**: with ≤ capacity simultaneous occupants the lowest-free-lot pick reproduces the current
  static lot indices exactly (verify).

**Option B (cheaper, static interval scheduling).** Keep load-time assignment but make it interval-aware:
estimate each occupant's `[arrival, departure)` window (departure = arrival + stop `duration`/`until`;
arrival ≈ stop-reached time, which for departPos="stop" origins is t=departure-time and for moving vehicles
needs a rough travel estimate) and assign the lowest lot free during that window. Simpler but approximate
and fragile on arrival-time estimation. Use only if A proves too invasive.

**REFINEMENT (measured 2026-07-21 — locks Option A, rejects B).** The box's failing area
`pa_e_d_2_2_d_3_2` (cap 1) is used by 3 vehicles whose vanilla `--stop-output` PARK intervals are
`veh0 [345,546]`, `veh1 [590,738]` — reusing lot 0 (both park at pos 231.40) — with the 3rd never
reaching it. Crucially the actual **park** times (345, 590) are ~285 s after the **depart** times
(59.79, 223.76): drive + jam. So load-time interval estimation (B) is unreliable (depart+freeflow ≈ 60
vs real park 345), and even depart-window overlap ([60,261] vs [224,372]) would falsely force distinct
lots and overflow. **Only true runtime park-time assignment (A) is correct.** Implemented mechanism:
- StopRuntime for a parking stop carries `ParkingAreaId` + `AssignedLot` (−1 = unclaimed); its `EndPos`
  (the brake target) is resolved at RUNTIME, not baked at load. `ResolveParkingAreaStops` stops assigning
  lot indices — it only fills `LaneId`/`StartPos`/`ParkingAreaId` (no `LotPosition` call at load, so an
  oversubscribed area no longer throws).
- Engine runtime state: `Dictionary<string,bool[]> _parkingLotOccupied` per area (capacity-sized).
- **Plan (read-only, start-of-step snapshot):** `StopLineConstraint` for an unclaimed parking front-stop
  computes the provisional lot = lowest `i` with `!occupied[i]` and brakes toward `LotPosition(i)`; if the
  area is FULL it brakes toward `StartPos` and the reached-gate below refuses to mark it reached (wait-
  when-full, SUMO's "no free pos" path). A claimed stop uses `LotPosition(AssignedLot)`.
- **ProcessNextStop reached-gate:** a parking stop is marked reached only if a lot is free/assigned (so a
  full area makes the vehicle wait at `StartPos` instead of parking on air).
- **Execute (deterministic order, mutates occupancy):** on the reached transition (the existing
  `stop.IsParking → v.IsParked=true` block) claim the live lowest-free lot, set `AssignedLot` + `EndPos`,
  mark occupied; on the resume transition (the existing `resumedStop.IsParking → v.IsParked=false` block)
  free the lot. departPos="stop" origins claim at insertion (they insert already-parked at `LotPosition`).
- **Parity:** committed goldens (48/66-72) never reuse and never oversubscribe → snapshot lowest-free ==
  the old static index at every step, so brake target + parked pos are byte-identical (verified by the
  full suite). Determinism: Plan reads the frozen snapshot; Execute claims/frees in the engine's existing
  per-step order; the pick is "lowest index" (order-independent). Anchor: `scenarios/76-parking-lot-reuse`
  (cap-1, veh0 pulls out then veh1 reuses lot 0 @ pos 210).

### 3.4 Success criteria (Gap 2)
The full `demo_city/box` (`scenarios/_ped/demo_city/box/scenario.sumocfg`) LOADS on SumoSharp (no "lot
index out of range") — combined with Gap 3 it runs end-to-end. Committed parking goldens (48/66/67/68/69/
70) stay byte-identical. A new anchor: a `roadsideCapacity=1` area referenced by ≥2 non-overlapping-in-time
vehicles loads and both park correctly (matching SUMO lot positions/tripinfo).

---

## 4. GAP 3 — departPos="base" (and confirm other symbolic departPos)

### 4.1 Root cause
`DemandParser.ParseDepartPos` (`Sim.Ingest/DemandParser.cs` ~560) accepts only a numeric literal or
`"stop"`; `"base"` (and `random`/`free`/`last`/…) throw *"unsupported departPos"*. 30 box vehicles use
`"base"`.

### 4.2 SUMO reference (exact — do NOT just map to "0")
`MSBaseVehicle::basePos(edge)` (`/sumo/src/microsim/MSBaseVehicle.cpp:1117`):
```cpp
double result = MIN2(getVehicleType().getLength() + POSITION_EPS, edge->getLength());
if (hasStops() && stops.front().edge == route.begin() && stops.front().lane->edge == route.begin())
    result = MIN2(result, MAX2(0.0, stops.front().getEndPos()));   // capped by a first-edge stop
return result;
```
`DepartPosDefinition::BASE` and `DEFAULT` both resolve to `basePos` (`MSLane.cpp:699-720`, non-SPLIT arm).
`POSITION_EPS` is SUMO's small epsilon (`StdDefs.h`, 0.1). So `"base"` = vehicle front bumper at
`MIN(vTypeLength + 0.1, edgeLength)`, further capped to a first-edge stop's endPos. The NEED doc's
"base→0 is bit-identical" is only true when that MIN/stop cap collapses to ~0 for the box's shapes — do the
**faithful formula**, not a hardcoded 0, so it is correct for every shape.

### 4.3 HOW (design)
- Add `DepartPosSpec.Base` to `DepartValue.cs` (`DepartPosValue.Base`), and accept `"base"` in
  `ParseDepartPos` → `DepartPosValue.Base`.
- Resolve it at INSERTION (`Engine.TryInsertOnLane`/the departPos resolution arm, where `Stop`/numeric are
  resolved today) using the faithful `basePos`: `MIN(vType.Length + PositionEps, lane.Length)`, capped to
  the first stop's endPos when the first stop is on the depart edge. Reuse the same `PositionEps` the
  parkingArea/stop code already uses.
- Keep every OTHER symbolic value (`random`/`free`/`last`/`random_free`/`speedLimit`) throwing (out of
  scope) — but the throw message can note they are unported, not "unsupported forever".

### 4.4 Success criteria (Gap 3)
A vehicle with `departPos="base"` (with and without a first-edge stop) inserts at SUMO's `basePos`
position (verify against a tiny SUMO FCD @1e-3). The 30 box vehicles no longer block the load. Committed
goldens byte-identical (none uses `"base"` today, so the new arm is inert there).

---

## 5. Global guardrails (CLAUDE.md iron law — applies to all three)
- **Parity:** every committed golden stays byte-identical (or within its `tolerance.json`); gate = full
  `dotnet test Traffic.sln` green on a fresh clone WITHOUT SUMO, plus a full-sweep engine-FCD byte-diff of
  the multi-lane/junction/parking goldens (the method used for the overlap fix — generate parity-mode FCD
  before/after, `diff`). `Sim.Bench` determinism hash unchanged.
- **Determinism:** no `System.Random`; per-vehicle/immutable state only; serial == region-parallel
  byte-identical (the reroute reads the immutable net + frozen snapshot, writes ego's own route slice via
  the command buffer).
- **Follow SUMO** (rule 4): reroute-on-dead-lane mirrors `ignore-route-errors + device.rerouting`;
  parking mirrors `computeLastFreePos`; base mirrors `basePos`. Deviate only where ECS structurally forces.
- **Design-first, staged:** land Gap 3 → Gap 2 → Gap 1 (increasing risk/size), each with its own anchor +
  green suite before the next. Gap 1 may need multiple passes (reroute → measure → cooperative-LC only if
  needed).

## 6. Order of work + file map
1. **Gap 3** (small, unblocks box load w/ Gap 2): `DepartValue.cs`, `DemandParser.cs`, `Engine.cs`
   (insertion departPos arm).
2. **Gap 2** (medium, unblocks box load): `Engine.cs` (`ResolveParkingAreaStops` → runtime lot manager),
   `ParkingArea.cs` (LotPosition unchanged; assignment moves to runtime).
3. **Gap 1** (large, the density fix): `Engine.cs` (`TryReResolveFromActualLane` drop-lane branch +
   `ExecuteMoveVehicle` + a plan-phase approach trigger + new `TryRerouteFromDeadLane`). Re-measure the
   dense synthetic every iteration; add the anchor; keep the suite green.

## 7. Pointers (code + refs)
- Gap 1: `Engine.cs` `TryReResolveFromActualLane` (~9051, drop-lane clamp ~9080), `ExecuteMoveVehicle`
  (~8960-9013), `UpdateReroutes`/`RegisterRerouted`/`ReplaceRoute` (reroute discipline), `NetworkRouter`.
  Evidence: `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`. Prior residual: `FOLLOWUP-TL-throughput-flowrate.md`.
- Gap 2: `Engine.cs` `ResolveParkingAreaStops` (~1223-1310), `Sim.Ingest/ParkingArea.cs`
  (`LotPosition`, capacity check ~30-45). SUMO: `MSParkingArea::computeLastFreePos`. Scope note:
  `docs/SERVE-PATH-PLAN.md §3a`.
- Gap 3: `Sim.Ingest/DemandParser.cs` `ParseDepartPos` (~560), `Sim.Ingest/DepartValue.cs`
  (`DepartPosSpec`/`DepartPosValue`), `Engine.cs` insertion departPos arm (search `DepartPosSpec.Stop`).
  SUMO: `MSBaseVehicle::basePos` (`/sumo/src/microsim/MSBaseVehicle.cpp:1117`), `MSLane.cpp:699`.
- Repro: `scenarios/_repro/synthetic-junction2`; full box: `scenarios/_ped/demo_city/box`. Drop-in:
  `src/Sim.Sumo`. Branch for this work: `claude/dense-flow-throughput-diag` (restart from `origin/main`).
