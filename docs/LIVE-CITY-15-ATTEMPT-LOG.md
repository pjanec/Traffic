# Issue #15 residual — ATTEMPT LOG (running journal)

Purpose: a dated, honest journal of what was tried for the live-city junction-jam residual, what the
evidence said, what worked, and **what failed and why** — so a future session does not repeat a dead
end. Append newest entries at the bottom. This is the scratch/decisions trail; the polished diagnosis
lives in `LIVE-CITY-15-RESIDUAL-FINDINGS.md`, the repro in `LIVE-CITY-15-RESIDUAL-REPRO.md`.

Branch: `claude/livecity-15-turnlane-segregation` (off the merged live-city tip incl. `184fb31`).
Iron rule throughout: `Sim.ParityTests` stays **657/4** byte-identical (or a change is gated behind an
explicit fast-mode flag); bench determinism (serial == parallel) holds.

---

## 2026-07-23 — Repro + engine merge (context, already landed on the live-city line)
- Built the headless `LIVECITY-GRIDLOCK` probe (`--mode live-city --smoke`); reproduced the pre-merge
  **terminal** gridlock (stoppedFrac → 0.94, arrivals 38/200 s).
- Merged `claude/dense-lane-overlap-fix-5tr4ha` wholesale → terminal-lock became **jam-and-recover**
  (arrivals 81, stoppedFrac oscillates 0.38–0.73). Parity 654→657/4, bench hash unchanged. **Helped,
  not cured** (GPU verdict: jams still look unrealistic).

## 2026-07-23 — Witness: ground-truth on the residual stalls
- Added `LiveCitySim.WitnessAuthoritative()` + `LIVECITY_WITNESS=1` dump (engine-authoritative per-car
  lane/pos/posLat/speed/TL/gap). Read-only, host-side.
- **Findings that reframed the problem:**
  - `posLat = 0` for every stuck car → the GPU "lateral float" is a **render/DR artifact**, not engine
    motion. Do NOT chase a lateral-jockey bug in the engine.
  - Most stopped cars are at **red** or **behind a stopped leader** (consequence of a jam). The anomaly
    is `stuckOnGreenClear` (7–16 cars): speed≈0, TL green (incl. protected `G`), no leader within 15 m,
    at pos≈226–233 (the stop line). These block the queues behind them.
- Code + `/sumo/` analysis (subagent): getBestLanes offsets are CORRECT and keep-right rule 2 is landed
  → **"wrong turn-lane selection" hypothesis FALSIFIED**. Root localized to **missing LCA_URGENT
  strategic-change cooperation**: `TryStrategicLaneChange` (`Engine.cs:11048-11053`) does a bare
  `return false` when the turn lane is blocked; no ego brake-to-wait, no follower gap-opening. Turner
  never merges → strands at stop line → `Speed=0` clamp (`Engine.cs:9587-9611`).

## OPEN QUESTION raised by owner (2026-07-23) — MUST verify before designing
**"How does telling the follower to slow solve a car already stopped on green at a clear junction?"**
Valid skepticism. The link is INDIRECT (see the explanation section below): cooperation is **upstream
prevention**, not stop-line release. It stops turners from *arriving* stranded in the wrong lane; it does
NOT release a car already stranded.
- **Therefore, before committing to option (1) as the fix, verify the causal chain per stuck car:** does
  each `stuckOnGreenClear` car sit in a lane whose connections do NOT include its next route edge
  (= wrong-lane strand, which cooperation prevents), or does its lane DO connect but it is blocked for
  another reason (junction exit occupied / keep-clear / RoW)? Plan: extend the witness to print, per
  stuck car, whether `currentLane` has a connection to the car's next route edge, and the occupancy of
  the intended next (across-junction) lane. If most are NOT wrong-lane strands, option (1) is the wrong
  lever and we re-target. **Do not design the cooperation port until this split is measured.**

## Attempts that FAILED / were abandoned (avoid repeating)
- **Standalone `tools/livecity-repro/` console harness** — abandoned: the built-in `--mode live-city
  --smoke` path already runs the identical LiveCitySim loop; a separate project just duplicated it.
  Use `--smoke` (+ `LIVECITY_WITNESS=1`), not a new csproj.
- **Component-2 (LaneQ `occupation`/`maxJam`) on the dense-flow branch** — designed (`ad8d738`),
  implemented, measured **low-ceiling (~15% recovery) and entangled with the merge-in lever, then
  REVERTED** (`965fc45`). Do NOT re-attempt occupation-based urgency before the cooperation/merge-in
  lever; it was already shown ineffective in isolation.
- **"Turn-lane mis-segregation via getBestLanes"** as the root — FALSIFIED (offsets correct, rule 2
  landed). Don't re-open the getBestLanes offset port.

## 2026-07-23 — MEASUREMENT RESULT: option (1) REFUTED as the primary lever
Added a decisive, parity-neutral diagnostic counter `Engine.StrandedOffRouteThisStep` (Interlocked,
reset per step) at the EXACT wrong-lane dead-end clamp site (`Engine.cs` ExecuteMoveVehicle, the
`Pos=laneLength; Speed=0` branch reached only when the car is off its pool lane AND its lane has no
onward connection to its next route edge). Surfaced via `LiveCitySim.StrandedOffRouteLastStep` +
`LIVECITY-WITNESS`. **Parity verified byte-identical: `Sim.ParityTests` 657/4, bench hash
D96213B7BB4021A7 unchanged (a counter never read by sim logic cannot move a trajectory).**

Result (cap 160):
```
t     stuck  stuckOnGreenClear  stuckRed  stuckBehindLeader  strandedDeadEnd
 80    122         12             92          18                0
120     80         12             62           6                3
180     83         16             24          43                3
200     47          7             36           4                3
```
- **`strandedDeadEnd` = 0–3 at any instant** — the wrong-lane dead-end strand that URGENT cooperation
  would PREVENT is barely occurring. It cannot explain a 47–122-car jam.
- Corroborating: the `stuckOnGreenClear` example lanes/cars **turn over** between checkpoints (not the
  same cars stuck forever), and `arrivals` keep climbing — so those stalls are **temporary junction
  holds**, not permanent dead-end strands.
- **CONCLUSION: option (1) (URGENT LC cooperation for turn-lane merge) is NOT the right lever for the
  residual.** It would fix ≤3 cars. The owner's skepticism ("I don't see the link") was correct.
- **DO NOT build the URGENT-cooperation port for #15.** (The mechanism-gathering pass and the
  determinism-safe shape it found — the retired `CoopSpeedAdvice`/`afec614` machinery — are recorded
  below for the record, but are shelved unless a future finding actually needs them.)

## Re-diagnosis: what the residual actually is (hypothesis, to verify)
Dominant stuck categories are **red-light queuing** (`stuckRed`) + **queue propagation**
(`stuckBehindLeader`) + a small set of **holds on green** (`stuckOnGreenClear`, NOT dead-end) —
consistent with junction **keep-clear / right-of-way** conservatism and normal signalized-grid
congestion. Open question the witness cannot answer alone: **is this a REAL throughput deficit vs
vanilla SUMO, or just a dense signalized grid where 40–70% stopped-at-instant is normal?**
- **NEXT STEP (authoritative): SUMO cross-check.** SUMO 1.20.0 is available. Run vanilla SUMO on the
  same `net.xml` + comparable demand; diff tripinfo/summary (arrivals, mean speed, timeLoss) against our
  engine on the identical scene. If SUMO clears far more, the deficit is real and localizable (which
  junctions / movements); if SUMO is similar, the "jam" is largely normal dense-grid behaviour and the
  fix is demo-tuning (lower density / signal timing), not an engine bug. This is investigation only —
  the offline `dotnet test` loop never invokes SUMO.

## 2026-07-23 — SUMO 1.20.0 cross-check: real deficit confirmed, moderate
Exported the EXACT procedural live-city demand (238 trips) via `LIVECITY_DUMPROUTES` (added a
`LiveCitySim.SpawnLog` + a `.rou.xml` writer in `RunLiveCitySmoke`; host-side, parity untouched), then
ran **pinned SUMO 1.20.0** (`pip install eclipse-sumo==1.20.0`, matching `SUMO_VERSION` — the apt 1.18
was discarded per "no unnecessary deviation") on the SAME `net.xml` + demand, matched settings
(`--step-length 0.5 --time-to-teleport -1 --lanechange.duration 2.0 --default.speeddev 0`, vType
sigma=0, departPos 5 / departSpeed 0 / departLane best).

| metric | our engine | SUMO 1.20.0 |
|---|---|---|
| arrivals @200 s (of 238) | **81** | **110** (+36%) |
| full drain (end 800) | jam-and-recover | all 238 by ~800 s |
| mean trip speed @200 s | ~3–7 m/s (oscillates) | 9.03 m/s |
| mean timeLoss / wait per trip | — | 47.7 s / 30.2 s |

- **Real deficit: ~26 % fewer arrivals than SUMO in the same window** (81 vs 110) → genuine engine
  under-discharge, not purely normal congestion.
- **But SUMO also waits a LOT** (30–48 s mean wait/trip) → the scene IS a congested dense grid; much of
  the "whole city stopped" look is normal, the bug is the *extra* ~26 %.
- Consistent with the witness localization: the deficit is **junction right-of-way** (minor-green
  over-yield + the protected-green stalls), NOT lane-change segregation.
- Caveats (honest): SUMO routes the trips with its own weights (route choice may differ slightly);
  SUMO run has no peds (but `LIVECITY_YIELD=0` already showed peds are not the driver). The ~26 % gap
  is well outside those confounds.

**NEXT: drill the junction RoW.** Localize *which* movements/junctions our engine over-yields on vs
SUMO (per-junction throughput diff, or instrument the yield decision for a `majorGreenSTUCK` /
`minorGreenYield` car — what foe it thinks is blocking, whether that foe is real/cleared). Then
design-first a parity-safe fix to the permissive-yield (`blockedByFoe`, `f69a58d`) path. Do NOT touch
lane-change cooperation.

## 2026-07-23 — Owner correction: "city full of cars standing on GREEN" — render-faithful, real
Owner pushed back that the visible symptom is a CITY FULL of cars stopped on GREEN, not the "mostly
normal red queuing" I had framed. Checked the render-vs-engine TL hypothesis directly: added `TlWire`
(the wire/rendered state `VehicleSource.TlStateByLane`) alongside the engine state in the witness.
Result (cap 160): **`tlRenderLie = 0` at every checkpoint** — the rendered signal heads faithfully
match the engine (NO render lie), and **`renderedGreen = 11–30` of the 47–122 stopped cars (~25–45%)
are genuinely under a green head.** So:
- The cars really ARE on green in the engine — not a rendering artifact (unlike the earlier posLat
  float). My "mostly normal red congestion" framing UNDER-counted the on-green stalls; correction
  logged.
- Most on-green stopped cars are **queues backed up behind a junction whose FRONT car will not
  discharge** (minor-green yield / protected-green stall / blocked exit). The front car is the root;
  the queue behind it (all on green) is the visible "city full on green."
- Sharpest no-innocent-explanation signature = **`majorGreenSTUCK`** (protected green `G`, own lane
  clear, exit empty, speed 0): 0–10 cars/step. A protected-green car with clear road AND empty exit
  has NO legitimate reason to stop.

**NEXT: instrument WHY a `majorGreenSTUCK` car is held** — dump its binding speed constraint
(which term in `ComputeMoveIntent`'s Min-reduction pins vPos to ~0: junction-yield? a phantom foe? an
internal-junction leader the same-lane gap misses?). That is the direct root cause of the on-green
stall. Still junction/RoW territory, NOT lane changes.

## 2026-07-23 — Box-blocking RULED OUT; root = front car stalls on PROTECTED green
Owner: "junctions are full of blocked cars … at least one direction is clear and the TL state changes,
so it should unblock eventually." Added an internal-lane counter (`onInternal`/`stuckInternal`: cars
whose LaneId starts with `:` = physically inside a junction). Net has 2182 internal lanes, so the
engine DOES model cars in junctions.
- **`stuckInternal` = 0–4 even at LIVECITY_CARS=300 and 500** → it is NOT box-blocking / cars frozen
  mid-junction. The `onInternal` cars are all moving through.
- At density, `renderedGreen` = 47–108 stopped-on-green and `majorGreenSTUCK` = 14–19 → the "junctions
  full" is the four APPROACH arms packed to the stop line, not the junction interior.
- They don't clear because each arm's FRONT car wastes its green (`majorGreenSTUCK`: protected green
  `G`, own lane clear, exit mouth clear, speed 0). This is exactly the owner's "one direction clear +
  lights cycle but doesn't unblock."
- Engine HAS protected-green priority (`EgoLinkHasSignalPriority`, `Engine.cs:2298`, suppresses the
  minor-yield arms on `'G'`), so a `'G'` car being frozen is held by something OTHER than the normal
  junction-yield — the exact binding constraint is the last unknown.

**NEXT (final drill): binding-constraint trace for a `majorGreenSTUCK` car.** Instrument the engine's
speed reduction (`ComputeMoveIntent`'s Min-over-constraints) to record, for a flagged protected-green
frozen car, WHICH constraint pins vPos≈0 (a phantom junction foe? a moving internal-lane foe on the
conflicting path? a crowd/crossing term? a leader across the junction the same-lane gap misses?).
Parity-neutral diagnostic. That line IS the root cause of the on-green stall.

## 2026-07-23 — ROOT CAUSE PROVEN: JunctionYieldConstraint over-yields on PROTECTED green
Added a per-vehicle binding-constraint trace: `Engine.ComputeMoveIntent`'s Min-fold now records the
argmin (WHICH of the 13 constraints limited the speed) into `VehicleRuntime.BindingConstraint` →
`VehicleReadBuffer` column → `Engine.BindingConstraints` span → witness. **Parity byte-identical
(657/4), bench hash D96213B7BB4021A7 unchanged** — the fold value/order/side-effects are untouched;
only an argmin is captured alongside.

Binder histogram for the `majorGreenSTUCK` cars (protected green `G`, own lane clear, exit empty),
`LIVECITY_CARS=300`:
```
t=100  junctionYield=10  deadLaneMerge=4  freeFlow=1     (of 15)
t=120  junctionYield=2   deadLaneMerge=4  freeFlow=1     (of 7)
t=160  freeFlow=2 deadLaneMerge=2 crossJxnLeader=1 redLight=1  (of 6)
```
- **DOMINANT root: `JunctionYieldConstraint` binds protected-green (`'G'`) cars** — they are yielding
  at the junction even though a major-green link holds priority (`havePriority`) and should yield to
  no one (`EgoLinkHasSignalPriority`, `Engine.cs:2298`, is supposed to suppress exactly this). So the
  bug is a right-of-way OVER-YIELD that the protected-green suppression is NOT catching on this path.
- **Secondary: `DeadLaneMergeBrakeConstraint`** (GAP-1 wrong-lane/dead-lane brake) — a few cars braking
  because their lane can't reach the next route edge (turn-lane-adjacent, but via the merge-brake, not
  the strand clamp; `strandedDeadEnd` stays ~0-3).
- **Noise:** `redLight` on a "majorGreen" car = the witness's any-green `TlForLane` mislabelled a lane
  whose OWN movement link is red (per-movement red under a lane green for another movement);
  `freeFlow` = a just-spawned car still at pos≈8 accelerating from 0. Both small.

**ROOT CAUSE (proven, not inferred): the junction right-of-way gate over-yields vehicles that hold a
protected green.** Next = read `JunctionYieldConstraint` (+ `EgoLinkHasSignalPriority` /
`adaptToJunctionLeader`) to find WHY a `'G'`-priority ego still yields to a foe, then design-first a
parity-safe fix. This is squarely `Sim.Core` junction RoW; lane-change cooperation stays shelved.

## 2026-07-23 — CORRECTION: NOT a protected-green over-yield. Measure-first saved a bad fix.
Was about to implement a "protected-green priority not suppressing junction yield" fix. Added a
junction-yield SUB-ARM trace first (`VehicleRuntime.JunctionYieldArm`: which arm bound + a 0x80 bit =
`egoHasSignalPriority` was true), plumbed like `BindingConstraint`. **Parity 657/4 byte-identical,
bench hash unchanged.**

For the `majorGreenSTUCK` cars whose binder is `junctionYield` (`LIVECITY_CARS=300`):
```
t=100  JYarm: adaptToJxnLeader=8(prio0)  approachingCross=2(prio0)
t=120  JYarm: adaptToJxnLeader=2(prio0)
t=140  JYarm: adaptToJxnLeader=1(prio0)
```
- **`prio0` in EVERY case → `egoHasSignalPriority` is FALSE.** These cars do NOT hold a protected-green
  priority for their own movement. The witness's lane-level `Tl='G'` (any-green-wins) MISLABELLED them:
  the lane has a green movement, but the stuck car's own junction link is minor/permissive. **So the
  "protected-green over-yield" bug I was about to fix DOES NOT EXIST.** (Both static-matrix minor arms —
  cautiousApproach, approachingCross — are already correctly gated by `egoHasSignalPriority`; and the RBL
  `JunctionCycleHold` already excludes TL junctions, P2-G Bug-2.)
- **Dominant real hold = `adaptToJunctionLeader`** (following a foe physically ON the junction =
  clearance / don't-drive-into-a-crossing-car). That is car-following safety SUMO applies regardless of
  priority (`checkLinkLeader`) — largely LEGITIMATE. A few are the minor-movement approaching-cross yield.
- Net: there is **no single clean over-yield bug**. The real ~26% deficit vs SUMO is subtler — most
  likely gap-acceptance / impatience / clearance aggressiveness under saturation (SUMO's impatient front
  cars force gaps + clear faster), where the un-gated-by-design `adaptToJunctionLeader` dominates and is
  not subject to impatience. This needs a careful engine-vs-SUMO *per-junction* comparison before any
  fix — a blind change here is high parity risk for an unproven target.

**DO NOT implement a protected-green-priority fix — measured false.** Next honest step (if pursued):
compare our minor-movement / clearance discharge to SUMO on ONE saturated junction (tripinfo + a
per-junction throughput diff) to localize whether impatience/gap-acceptance or clearance timing is the
gap — then design-first. This is a genuinely harder, subtler target than a one-line RoW gate.

## 2026-07-23 — COMPREHENSIVE per-car dump of every "stuck with clear road": no phantom, no bug
Owner: dump ALL cars stuck-with-clear-road and record full status to find what's wrong. Added the bound
junction foe's SPEED (`VehicleRuntime.JunctionYieldFoeSpeed`) and a `LIVECITY-STUCKCLEAR` dump: every car
with speed<0.3 AND own-lane gap>15 AND exit-mouth>15, broken down by binding constraint, and — for the
junction-yield foe arms — the bound foe's speed bucket. **Parity 657/4 byte-identical; bench hash
unchanged.** Result (`LIVECITY_CARS=300`, per interval):
```
byBinder: freeFlow=34-50  redLight=11-19  deadLaneMerge=6-8  junctionYield=2-17  leaderFollow=1-3
JYfoe:    moving=ALL      slow=0          stopped=0          none=0-1
```
Reading each group with full per-car lines:
- **`freeFlow` (the LARGEST group) are NOT stuck** — they are cars ACCELERATING away from a stop this
  step (bound by their own desired speed = free to go). The speed<0.3 filter caught momentarily-slow
  movers. Example lines confirm `tlLane=r` cars pulling away as their phase releases.
- **`redLight`** = legitimately stopped at a red phase (`tlLane=r`). Normal.
- **`junctionYield`** = yielding to a foe that is **MOVING** in EVERY case (foeSpd 1.3, 2.6, 7.94, 13.89
  m/s; **stopped=0, none≈0**). So it is REAL cross traffic crossing the junction — NOT a phantom foe,
  NOT a stopped foe blocking the box, NOT a stale detection. `egoPrio=-` confirms these are minor
  movements (correctly yielding).
- **`deadLaneMerge`** = a handful braking for a wrong-lane/dead-lane (GAP-1); the only "improvable"
  engine-behaviour signal, and small.

**DEFINITIVE CONCLUSION:** there is NO "stuck on green with a clear road" bug. A green car that is
genuinely stopped is **correctly yielding to real, moving cross traffic** on the junction (its OWN road
is clear; the perpendicular movers are what hold it — invisible looking down the ego's road). Therefore
we CANNOT simply "detect and unblock" per-step: forcing such a car forward drives it through active
cross traffic (a collision). The only safe unblock is an **escape valve that fires after a long wait**
(SUMO's teleport / time-to-teleport), by which time the situation is a genuine deadlock, not an active
yield. The residual ~26% vs SUMO is saturation standoffs SUMO breaks with teleport (+ its impatient
gap-forcing) — behaviours we largely have EXCEPT teleport.

## 2026-07-23 — FIX LANDED: SUMO teleport escape valve (short timeout) unblocks the demo
Owner spec: short timeout (few secs, not 120), unblock if road clear after that, feels like a driver who
didn't notice the gap and recovers quickly.
- Built the SAFE version first: `Engine.JunctionYieldTimeoutSeconds` (opt-in) — after N s waiting, force
  the gap vs an APPROACHING foe (never vs an on-junction foe = collision-safe). Measured **ineffective**
  (81 → 81 arrivals): the dominant hold is a foe PHYSICALLY crossing (moving 7-14 m/s), which cannot be
  overridden safely. Kept as a correct/harmless knob.
- **Discovered SUMO's jam teleport (`CheckJamTeleports`) is ALREADY ported**, gated on `TimeToTeleport>0`,
  off in live-city. Wired `LiveCityConfig.TimeToTeleportSeconds` (default **5 s**, env `LIVECITY_TELEPORT`)
  → spliced `<time-to-teleport>` into the live-city engine config. **Measured: arrivals 81 → 188,
  stoppedFrac 0.39 → 0.10 (free flow), meanSpeed 6.9 → 10.8; clearStuck ~30 → ~12-27; overlaps ~0 (no
  collisions); car count stable (teleport reinserts, never deletes).**
- **Parity 657/4 byte-identical, bench hash D96213B7BB4021A7 unchanged** — both knobs default-off in the
  Engine/scenario/bench path (only the demo enables them), same inert-when-off guarantee as
  `LaneChangeMinSpeed`.
- **This is THE #15 residual fix.** It IS the "no teleport ported" gap the whole investigation kept
  pointing at (report #15). The only tradeoff is teleport's forward *jump* (SUMO's own behaviour) — GPU
  session to tune 5 vs 10 s for the best look. Design: `docs/LIVE-CITY-15-YIELD-TIMEOUT-DESIGN.md`.

## 2026-07-23 — TERMINAL gridlock IS reproduced (needed ~900 s); root = accumulating DEAD-END strands
Owner: the GPU viewer gets stuck EVERY run; almost all cars standing, junctions "occupied", cars on
green with NO visible reason; teleport (jump across) is the wrong cure — the car must travel THROUGH.
Prior runs only went to 200-400 s and missed it. Ran the DEFAULT density (160) to 1000 s, teleport OFF:
```
t   stoppedFrac arrivals
340   0.63       154
580   0.86       228
760   0.93       250
940   0.99       257   <- terminal: ~all standing
1000  0.97       258   (flatlined: +1 arrival in 120 s)
```
So the demo DOES reach TOTAL gridlock; it just takes ~15 min of sim time (the merge fix only DELAYED
it). Terminal-state witness (t=1000):
- **`stuckInternal=0, onInternal=0`** — junctions are EMPTY. NOT a box-block / cars-frozen-mid-junction.
- **`strandedDeadEnd` grew 3 → 8–22** — cars PERMANENTLY frozen in a dead-end lane (their lane has no
  connection to their next route edge → the engine clamps Speed=0 FOREVER, `TryReResolveFromActualLane`
  fails → `Engine.cs:9587-9611`). These are the only PERMANENT blockers; the red (70) + behindLeader
  (62) masses are downstream consequences that can never clear because the strands wall off lanes.
- Matches the owner exactly: "car on green, no visible reason" = stranded at a lane-end with no route
  connection (no foe, no light — a lane/route mismatch); teleport masks it, the car must travel through.

**This is the turn-lane / dead-end strand issue** — dismissed early (strandedDeadEnd ~0-3 at 200 s) but
it ACCUMULATES: each permanently-frozen car seeds a growing jam until the network locks. NOT the
cross-traffic yield (that was the pre-terminal transient), NOT box-block, NOT teleport-curable properly.

**REAL FIX (not teleport):** when a car dead-ends (its actual lane has no connection to its next route
edge), RE-RESOLVE / reroute it onto a connection its lane DOES offer and let it drive through, instead
of the permanent Speed=0 clamp. = the DENSE-FLOW-THROUGHPUT-DIAGNOSIS "candidate 1" (generalize
`TryReResolveFromActualLane` to fire at the stop-line approach). Design-first, parity-gated.
Teleport default set back to 0 (owner rejected it as unrealistic).

## 2026-07-23 — HONEST BOTTOM LINE: the terminal gridlock resists every "keep-cars-flowing" fix
Implemented the approved dead-end DRIVE-THROUGH fix (`Engine.DeadLaneDriveThrough`: when a dead-ended
car's congestion-weighted reroute finds no path, retry FREE-FLOW, then drive onto ANY forward
connection = SUMO ignore-route-errors). Parity-safe (657/4; gated, inert on goldens).
**MEASURED: it does NOT cure the terminal gridlock** — arrivals 257 at t=940, byte-for-byte the same
as baseline; the fallback barely fires, so the strands are NOT routing-failures.

Fixes TRIED against the ~1000 s terminal gridlock, all at cap 160, teleport off:
| fix | arrivals@940s | stoppedFrac | verdict |
|---|---|---|---|
| baseline | 257 | 0.99 | terminal |
| `MaxDeadLaneReroutes` 2→50 | 265 | 0.95 | ~no change |
| dead-lane drive-through | 257 | 0.99 | **no change** |
| junction-yield timeout | 257 | 0.99 | no change |
| **SUMO teleport (5 s)** | **~free flow** | **~0.10** | **CURES it — but relocates the car (owner rejected)** |

Also: the lock is **density-INDEPENDENT** (cap 80 and 120 both go terminal by 1000 s) and **junctions
stay EMPTY** (`stuckInternal=0`) — so it is neither pure saturation nor box-block. The stuck mass is
red (legit) + behindLeader (queues) that never clear; the dead-lane strands are a minority and mostly
temporary (clamp-and-retry), NOT the permanent seed I hypothesised.

**Conclusion (honest):** the live-city terminal gridlock is a DEEP junction-discharge deficit — the
dense-flow branch's own core UNSOLVED problem ("improved, not cured"). Every realistic "keep the cars,
make them flow" lever I tried fails; only teleport (SUMO's own escape valve, which relocates a car)
cures it. A genuine cure is a major engine-research effort on junction discharge, not a scoped knob.
The gated knobs (`DeadLaneDriveThrough`, `JunctionYieldTimeoutSeconds`, `TimeToTeleportSeconds`) are all
kept, default-off, parity-safe, available for experimentation. Design (kept for the record):
`docs/LIVE-CITY-15-DEADLANE-DRIVETHROUGH-DESIGN.md`.

## 2026-07-23 — PIVOTAL: SUMO does NOT gridlock on the same net+demand -> it's OUR ENGINE, and it's FIXABLE
Owner refused "give up"; hypothesis "scenario is wrong / car has nowhere to go". Checked the net: 51
traffic lights, ALL `type="static"` (fixed-time), `offset="0"` (uncoordinated) -> a plausible spillback
cause. DECISIVE TEST: dumped the EXACT sustained demand our engine spawns over 1000 s (418 trips,
`LIVECITY_DUMPROUTES`) and ran **SUMO 1.20 on the SAME net + SAME 418 trips**, teleport OFF:
```
        our engine            SUMO 1.20
t=100   ~free flow start      running=153 halting=12 meanSpeed 11.5
t=300   degrading             running= 99 halting=30 meanSpeed  9.0
t=500   heading to lock       running= 49 halting= 5 meanSpeed 11.3  (drained/recovered)
t=900   0.99 STOPPED          running= 12 halting= 3 meanSpeed  2.3  (nearly done)
final   gridlock, 258 arr     drains the demand, meanSpeed 9-11 throughout
```
**SUMO DRAINS it; our engine GRIDLOCKS on the identical net + demand.** Therefore:
- **NOT the scenario** — the static/uncoordinated lights are fine; SUMO discharges them. "Scenario is
  wrong" REFUTED.
- **NOT inherent / not density-saturation** — SUMO proves the same load flows.
- **It IS our engine's junction discharge** — a REAL, FIXABLE bug. SUMO is the existence proof.

This vindicates continuing. Next: engine-vs-SUMO DIFFERENTIAL to localize WHERE our discharge lags
(per-junction throughput diff, or a trip SUMO completes fast that our engine never completes -> trace
that car's hold in our engine). This is the dense-flow branch's core deficit, now PROVEN solvable, so
worth the dig. Teleport was masking a real bug, not curing an inherent limit -- correctly rejected.

## Retired-machinery note (shelved, for the record — NOT to build now)
Mechanism-gathering found the follower-cooperation channel was already built and RETIRED in `afec614`
("Retire the cooperative informFollower"): `VehicleRuntime.CoopSpeedAdvice` (+∞ default) +
`CommandBuffer.SpeedAdvice(follower, speed)` applied as a commutative MIN in Flush, consumed one step
later in `ComputeMoveIntent` — a determinism-safe shape (recoverable via `git show afec614`). Ego
brake-to-wait template = the `WaitingTime` self-write/next-step-read pattern (`VehicleRuntime.cs`).
Its retired failure mode: organic-net follower over-braking for OPTIONAL overtakes. Shelved.
