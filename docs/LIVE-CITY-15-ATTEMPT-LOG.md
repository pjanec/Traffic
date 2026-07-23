# Issue #15 residual — ATTEMPT LOG (running journal)

Purpose: a dated, honest journal of what was tried for the live-city junction-jam residual, what the
evidence said, what worked, and **what failed and why** — so a future session does not repeat a dead
end. Append newest entries at the bottom. This is the scratch/decisions trail; the polished diagnosis
lives in `LIVE-CITY-15-RESIDUAL-FINDINGS.md`, the repro in `LIVE-CITY-15-RESIDUAL-REPRO.md`.

Branch: `claude/livecity-15-turnlane-segregation` (off the merged live-city tip incl. `184fb31`).
Iron rule throughout: `Sim.ParityTests` stays **657/4** byte-identical (or a change is gated behind an
explicit fast-mode flag); bench determinism (serial == parallel) holds.

---

## HOW TO REPRODUCE (read this first — exact data, build, params, and how to read the output)

Everything below is deterministic: same inputs → same numbers, on any fresh VM, **no SUMO needed** (SUMO
is only for the optional cross-check in §"SUMO drains it"). Run from the repo root
(`cd "$(git rev-parse --show-toplevel)"`).

**Data (all committed, nothing generated at run time):** the coupled cars+peds demo over
`scenarios/_ped/demo_city/box/` — a pinned downtown crop `[2055,2055]–[2895,2895]` (~840 m). The host
is `Sim.LiveCity.LiveCitySim`; the headless test loop is `RunLiveCitySmoke` in `src/Sim.Viewer/Program.cs`
(`--mode live-city --smoke`). Inputs used: `net.xml` (51 static uncoordinated TLs), `scenario.rou.xml`,
`vType.config.xml` (Krauss **army vehicles, 7–14 m long**, `lcStrategic=0.3`). Cars spawn crop-edge→
crop-edge on the **best departure lane** (`departBestLane: true`), refill to a concurrent cap on arrival;
the RNG seed is fixed in `LiveCityConfig` (`CarRngSeed`, `PedSeed`), so the demand stream is identical
every run.

**Build once:** `dotnet build src/Sim.Viewer -c Release` (add `--no-build` to the run commands after).

**THE repro command (headless, ~1000 s of sim → terminal gridlock):**
```bash
LIVECITY_TELEPORT=0 LIVECITY_CARS=160 \
  dotnet run --project src/Sim.Viewer -c Release -- --mode live-city --smoke --frames 2000 \
  2>&1 | grep "LIVECITY-GRIDLOCK:"
```
`--frames N` = N sim steps of `Dt=0.5 s` (so 2000 frames ≈ 1000 s). Baseline outcome (knob off): by
t≈940 `stoppedFrac ≈ 0.99`, `arrivals` flatline ≈258. `LIVECITY-GRIDLOCK` columns are:
`t  liveCars  stoppedFrac  meanSpd  aggMove  arrivals  peds`.

**The per-car autopsy (why cars are stuck — add `LIVECITY_WITNESS=1`):**
```bash
LIVECITY_WITNESS=1 LIVECITY_TELEPORT=0 LIVECITY_CARS=160 \
  dotnet run --project src/Sim.Viewer -c Release --no-build -- --mode live-city --smoke --frames 1400 \
  2>&1 | grep -E "LIVECITY-WITNESS:|LIVECITY-STUCKCLEAR:|LIVECITY-STRANDREASON|STRANDDUMP"
```
- `LIVECITY-WITNESS` — stuck breakdown (minorGreenYield / majorGreenSTUCK / red / behindLeader /
  `strandedDeadEnd` / renderedGreen / `stuckInternal` / `tlRenderLie`).
- `LIVECITY-STUCKCLEAR` — every car stuck (speed<0.3) **with a clear road** (own-lane gap>15 AND
  exit-mouth>15), bucketed by binding constraint, plus the first ~8 dumped as `  CAR …` lines
  (lane, pos, tlLane, binder, gap, exitMouth).
- `LIVECITY-STRANDREASON(cumulative)` — WHY wrong-lane cars resolve as they do at a lane end:
  `reResolveOK`/`rerouteOK` (recovered) vs strand causes (`poolEdgeMismatch`, `capSpent`, `waitGate`,
  `noRouteToTarget`, `noOutgoingConn`, …). This is the histogram that localized the root cause.
- `STRANDDUMP` — concrete desynced cars (actualLane / edge / seqIdx / pool). **Gated on
  `LIVECITY_WRONGLANE=1`** (the dump only, not the histogram), capped at 8 lines.

**Knobs (env overrides, all default to the parity-safe/measured setting; only set to experiment):**
`LIVECITY_CARS=<n>` concurrent car cap (80/120/160 all gridlock) · `LIVECITY_PEDS=<n>` ·
`LIVECITY_TELEPORT=<s>` SUMO jam-teleport seconds (0=off; owner-rejected cure) ·
`LIVECITY_WRONGLANE=0|1` reroute-at-approach (**now default ON** — the never-clamp blockage cure) ·
`LIVECITY_DRIVETHROUGH=0|1` any-forward-connection fallback (**now default ON**) · `LIVECITY_YIELDTIMEOUT=<s>` ·
`LIVECITY_LCMIN=<mps>` lane-change min speed (currently 1.5; **under revision** — see "UPDATED LANE-CHANGE
RULES" below: the true rule is "no PURELY-lateral motion", not "no slow-speed change") · `LIVECITY_HZ=<hz>` ·
`LIVECITY_SEQDESYNC=1` desync tracer · `LIVECITY_DUMPROUTES=<path>` export the sustained demand as SUMO `<trip>`s.

**Gates that must hold for any code change:** `dotnet test tests/Sim.ParityTests -c Release` = **657
passed / 4 skipped** byte-identical; `dotnet run --project src/Sim.Bench -c Release` → `deterministic=True`,
`parallel==single`, hash **`D96213B7BB4021A7`**; no `System.Random`. The diagnostic knobs above are all
inert on every committed golden (byte-identical), which is why they can live in the engine default-off.

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

## DECISIVE ROOT-CAUSE PROOF (this session) — "frozen on green" = WRONG-TURN-LANE strand at the lane end
Ran the per-car autopsy (`LIVECITY_WITNESS=1` -> `LIVECITY-STUCKCLEAR` + per-car `CAR` dumps) on the
baseline (knob off) terminal-gridlock repro and read the individual frozen cars. Result, unambiguous:

- The dominant binder among cars **stuck with a totally clear road** (Speed<0.3, own-lane gap>15 AND
  exit-mouth>15) is **`freeFlow`** (id 3): 13-31 such cars per 20 s sample. `freeFlow` for a Krauss car
  just returns `laneVehicleMaxSpeed` (~11-13 m/s) -- i.e. **no speed constraint is holding these cars;
  the constraint fold says GO.** They are frozen anyway -> the hold is DOWNSTREAM of `ComputeMoveIntent`,
  in the MOVE EXECUTOR.
- The per-car dumps show them on **GREEN** (`tlLane=G`/`g`), **no leader** (`gap=inf`), **clear exit
  mouth** (`exitMouth=inf`), binder `freeFlow`, and **frozen at a FIXED pos that never changes** over
  hundreds of seconds. Examples: `e_d_4_3_d_4_4_2 pos=223.0`, `e_d_2_3_d_2_2_1 pos=223.0`,
  `e_d_4_5_d_3_5_1 pos=233.4`.
- **`pos` == the lane's physical length, EXACTLY** (net.xml: `e_d_4_3_d_4_4_2 length="223.00"`,
  `e_d_4_5_d_3_5_1 length="233.40"`). So every "frozen on green" car is sitting AT THE LANE END -- it is
  the dead-lane strand clamp (`Pos=laneLength; Speed=0`), re-applied every step. `ComputeMoveIntent`
  keeps computing `freeFlow` (unaware of the clamp); ExecuteMoves keeps re-clamping.
- **WHY it strands (the connection topology proves it):** edge `e_d_4_3_d_4_4` has
  `lane 0 -> :d_4_4_w1` (pedestrian walking area ONLY, no vehicular connection),
  `lane 1 -> d_5_4(right) + d_4_5(straight)`, `lane 2 -> d_4_5(straight) + d_3_4(left)`.
  A car whose next route edge is the RIGHT turn (`d_5_4`) but which is on **lane 2** has **no connection
  to its next edge** (lane 2 serves straight+left only). It drives to the lane end and freezes forever.
  This is a **wrong-TURN-LANE** failure: the car never got sorted into the lane that serves its turn.

### What this means (answers the owner's "no visible reason" directly)
The "cars standing on green with no visible reason" ARE cars that reached a junction on a lane whose
connections do not include their next (turn) edge. The reason is invisible in the viewer because the
light is green and the road ahead is clear -- but the car's LANE physically cannot take it where its
route needs to go. It is not a yield, not a leader, not a red, not box-block (`stuckInternal=0` in
baseline): it is a **lane-assignment / strategic-lane-change parity gap**. SUMO drains the identical
demand because SUMO sorts vehicles into a turn-compatible lane before the junction (best-lanes +
strategic LC) and, failing that, does not permanently freeze them; our engine leaves them on the wrong
lane and then clamps them dead.

### Why the `WrongLaneRerouteAtApproach` fix (task #21) does NOT cure it (measured, subagent run)
Generalising the reroute to fire at the approach + retry every step (Engine knob, default off; parity
verified 657/4, bench hash `D96213B7BB4021A7` unchanged) was implemented and MEASURED:
| t(s) | OFF arrivals / stoppedFrac | ON arrivals / stoppedFrac |
|---|---|---|
| 580 | 228 / 0.86 | 214 / 0.93 |
| 940 | 257 / 0.99 | 225 / 0.99 |
Both still go terminal; **ON is WORSE** (225 vs 258 final arrivals). Diagnostics: `strandedDeadEnd` is
tamer with the knob on (bounded 9-14 vs 8-22), BUT `stuckInternal` (cars frozen ON an internal
`:`-junction lane = box-blocking) goes from **0 throughout baseline** to a **sustained 14-19** with the
knob on. I.e. rerouting the wrong-lane car earlier just drives it INTO the junction, where it then jams
against a full downstream lane -- **relocating** the block from "clamped before the stop line" to "stuck
inside the box," a strictly worse failure mode. So reroute-at-approach is NOT the cure; knob left in the
engine (parity-safe, default OFF) but the LiveCityConfig default is set back to OFF (was a regression).

### The real direction (SUMO-faithful)
The cure is upstream of the strand: get the car into a **turn-compatible lane before the junction**, the
way SUMO's best-lanes + LC2013 strategic model does (`lcStrategicLookahead=1000` in the vTypes). The gap
to investigate next is our strategic lane-change / best-lanes lane selection under dense, long-vehicle
(army 7-14 m) traffic: are our cars even attempting the strategic change into the turn lane, and if so
why do they fail where SUMO succeeds on the identical params? That is the parity gap to close; the
dead-lane clamp is only the symptom of un-sorted cars. This is real engine work (LC/best-lanes), not a
scoped knob -- next session's target.

## Knob-ON box-block autopsy (why "just reroute the wrong-lane car" isn't enough alone)
Ran `LIVECITY_WRONGLANE=1 LIVECITY_WITNESS=1` (reroute-at-approach ON). The reroute WORKS at its own
job -- the wrong-lane car no longer freezes permanently at the stop line (`strandedDeadEnd` bounded
9-15 vs baseline's peak 22, and the frozen-at-lane-end `freeFlow` clamp count drops). BUT a NEW failure
appears: `stuckInternal` (cars frozen ON an internal `:`-junction lane) climbs 7 -> 19 from t~360, when
baseline held it at **0** the whole run. I.e. giving the wrong-lane car a valid connection lets it ENTER
the junction -- and then it box-blocks, because **nothing prevents a car from entering a junction it
cannot clear** (the downstream lane is full). The block simply MOVES from "frozen at the stop line" to
"frozen inside the box," and box-blocks are worse (they wall cross traffic too), so net arrivals drop
258 -> 225. (`overlaps` 1-2 is pre-existing probe noise -- baseline shows the same on curved lanes --
not a reroute artifact.)

### Conclusion: the cure is TWO parts, and part (B) is the SUMO-parity core
- **(A) reroute the wrong-lane car so it never permanently freezes** -- DONE
  (`Engine.WrongLaneRerouteAtApproach`), but insufficient ALONE: it relocates the jam into the junction.
- **(B) never let a car ENTER a junction it cannot CLEAR** -- the missing piece. SUMO's `MSLink::opened`
  / `MSVehicle::checkRewindLinkLanes` hold a vehicle BEFORE the stop line when the downstream lane lacks
  room for it, so junctions stay empty and discharge cleanly (this is WHY SUMO drains the identical
  demand while we gridlock). Our engine has `KeepClearConstraint` but it evidently does not gate entry
  on downstream free space the way SUMO does. Part (B) is a real, parity-sensitive engine change ->
  design-first (docs/DESIGN.md rule); it likely also improves the baseline directly (a car that never
  box-blocks can't seed the cascade). This is the next target: port SUMO's "don't block the junction"
  entry gate faithfully, THEN re-enable (A) so a genuinely wrong-lane car reroutes AND queues cleanly
  instead of either freezing or box-blocking.

## DESIGN PRINCIPLES for the #15 cure (owner-stated, binding — honor these over any SUMO shortcut)
These are the owner's hard constraints on HOW the wrong-lane/gridlock problem may be solved. A fix that
violates any of them is wrong even if it improves the metric:

1. **No fake congestion excuse.** There are only ~160 cars on a rectangular grid with abundant alternative
   routes to any point -- it is *physically impossible* to genuinely congest it (SUMO drains the identical
   demand, running->12). So any "terminal gridlock" is a BUG (a freeze-cascade or a circular deadlock),
   never real saturation. Do not explain it away as congestion/discharge limits.

2. **A wrong-lane car MUST NOT stop.** A car that ended up on a lane with no connection to its next route
   edge must NOT freeze at the stop line. It must **reroute its trajectory** onto one of the grid's many
   alternatives and keep moving. Permanent Speed=0 clamp is a bug.

3. **NO lane changes while slow or stationary.** A car may only change lanes while moving -- shoving
   sideways through a stopped/crawling queue is unrealistic and forbidden, *even though SUMO does it*.
   (So the `LaneChangeMinSpeed` gate STAYS; LCMIN=0 is NOT an acceptable fix.) Corollary: cars must be
   sorted into their turn lane EARLY, while still cruising, well before the approach queue forms.

4. **A needed-but-blocked lane change must be COOPERATIVE.** If a moving car needs to change into an
   occupied lane, the fix is the target-lane car opening a gap by waiting a while (SUMO LC2013
   cooperative `informFollower`), NOT force-insertion and NOT a slow-speed snap. NB: this repo has a
   RETIRED cooperative channel (`VehicleRuntime.CoopSpeedAdvice` + `CommandBuffer.SpeedAdvice`, git
   `afec614`) that is a determinism-safe template to revive if cooperation is the chosen lever.

5. **Verify every hypothesis against the per-car state dump BEFORE chasing it.** No knob-sweep-driven
   conclusions. Confirm the mechanism in `LIVECITY-STUCKCLEAR` / `LIVECITY-STRANDREASON` / the per-car
   `CAR` dumps first, then act. (This is why the strand-reason histogram below was added.)

## Instrumentation added to serve principle #5: LIVECITY-STRANDREASON histogram
`Engine.StrandReasonHistogram` (parity-neutral cumulative counters, Interlocked; exposed via
`LiveCitySim.StrandReasonHistogram`, printed by the smoke probe as `LIVECITY-STRANDREASON`). At each
lane-end resolution of a wrong-lane car it records WHICH branch fired:
`reResolveOK`/`rerouteOK` (recovered, NOT stranded) vs the strand causes
`onDestEdge`/`waitGate`/`capSpent`/`noOutgoingConn`/`noRouteToTarget`/`reResolveThrew`/`remainingLt2`.
This decides the open fork: are the strands **recoverable-but-gated** (capSpent/waitGate -> lift a
policy gate) or **route-topology dead** (noRouteToTarget/noOutgoingConn -> the lane truly can't reach
the destination, so only a reroute-from-actual-lane or an upstream lane-sort helps)? Result to be read
next run.

## ROOT CAUSE CORRECTED (per-car verified): the dominant strand is a POOL/EDGE DESYNC, not a turn-lane issue
Ran the new `LIVECITY-STRANDREASON` histogram (baseline, knob off) + a `STRANDDUMP` of concrete cars.
Result overturns the earlier "wrong-turn-lane" writeup:
- The dominant strand reason by FAR is `remainingLt2`, and splitting it shows it is ~entirely
  **`poolEdgeMismatch`** (`remaining[0] != currentLane.EdgeId`), NOT `remainingCountLt2`, and NOT the
  turn-lane causes (`capSpent` only appears LATE, t>900, as a small secondary; `noRouteToTarget`/
  `noOutgoingConn` ~never). Cumulative `poolEdgeMismatch` climbs into the thousands while `reResolveOK`
  (recovered) stays ~120-170.
- Concrete `STRANDDUMP` examples (knob on to enable the dump; the branch is knob-independent):
  ```
  actualLane=e_d_6_3_d_5_3_2 edge=e_d_6_3_d_5_3 seqIdx=1/16 remaining.Count=6 remaining[0]=e_d_5_3_d_4_3
     pool=[:d_5_3_5_0, e_d_5_3_d_4_3_3, :d_4_3_1_2, e_d_4_3_d_3_3_1, ...]
  actualLane=e_d_4_5_d_3_5_2 edge=e_d_4_5_d_3_5 seqIdx=1/17 remaining[0]=e_d_3_5_d_2_5
     pool=[:d_3_5_6_0, e_d_3_5_d_2_5_2, ...]
  ```
  Read: the car is PHYSICALLY on edge `e_d_6_3_d_5_3` (approaching junction d_5_3), but its
  `LaneSeqIndex` (=1) already points ONE JUNCTION AHEAD to `e_d_5_3_d_4_3`, and **its route pool does
  not even contain the edge it is standing on** (pool[0] is the internal `:d_5_3`, pool's first normal
  edge is the one LEAVING d_5_3). So the car's lane-sequence index is desynced one edge ahead of its
  physical edge.

### Mechanism & why it gridlocks
When such a car reaches its lane end, the boundary guard (`Engine.cs` ~9647,
`v.LaneHandle != _laneSeqPool[seqIdx]`) sends it to `TryReResolveFromActualLane`, which only handles a
wrong LANE on the SAME edge; for a wrong EDGE it bails immediately (`remaining[0] != currentLane.EdgeId`
-> return false) and the caller clamps `Pos=laneLength; Speed=0` **forever**. That is the "frozen on
green, nowhere to go" car -- but the true cause is that its route pool/index no longer matches the edge
it is on, NOT that its turn lane lacks a connection. A stream of these desynced cars freeze and wall
their corridors -> the whole grid cascades (exactly the owner's "160 cars can't really congest -> it
must be a bug" -- it is: a route-tracking desync). This supersedes the earlier commit's turn-lane
framing (that path -- capSpent/noRoute -- is real but minor and late).

### Fix direction (next, design-first)
Two prongs, in order:
1. **Find WHY LaneSeqIndex desyncs one edge ahead of the physical edge** (the true root). Suspects: a
   cross-boundary lane change, or a reroute/re-resolve splice that sets `LaneSeqIndex`/pool so the
   current edge is dropped. Instrument the transition that produces the first mismatch for one car.
2. **Make recovery robust (owner principle #2 "must not stop"):** when `poolEdgeMismatch` is detected,
   re-route the remaining trajectory from the car's ACTUAL current edge (it clearly connects onward on a
   grid) instead of bailing to the dead clamp -- i.e. generalise `TryReResolveFromActualLane` to the
   wrong-EDGE case, reusing `TryRerouteFromDeadLane`'s Router-from-actual-edge. Must respect principles
   #3/#4 (no slow lane snap; cooperative if a change is needed) and stay parity-safe (gated, the branch
   is inert on every golden).

### Instrumentation landed this step (parity-safe, 657/4, byte-identical)
`Engine.StrandReasonHistogram` (+ `MarkStrandReason`, indices incl. 9 remainingCountLt2 / 10
poolEdgeMismatch) and the gated `STRANDDUMP` line; surfaced via `LiveCitySim.StrandReasonHistogram` and
the probe's `LIVECITY-STRANDREASON`. Also added `LIVECITY_DRIVETHROUGH` env override.

## PRONG 1 COMPLETE — TRUE ROOT CAUSE (per-car verified): lane-change maneuver straddles a junction
Added a gated invariant tracer (`Engine.DiagSeqDesync` / `SEQDESYNC-CREATED@<site>`, env
`LIVECITY_SEQDESYNC=1`) that flip-detects the first operation to desync a car's `LaneSeqIndex` from its
physical edge, checked after every mutation site (entry, boundaryCross, afterLaneChanges,
afterUpdateReroutes, afterPeriodicReroute, postExecute). Result is unambiguous:

**Every desync is created by `AdvanceLaneChanges()`** (tag `@afterLaneChanges`; `UpdateReroutes`/
`UpdatePeriodicReroutes` are no-ops in live-city — RerouteThresholdSeconds=+inf, ReroutePeriod<=0, no
obstacles). Decisive dump:
```
SEQDESYNC-CREATED@afterLaneChanges: actLane=e_d_6_3_d_5_3_2 seqIdx=1/16 actEdgeFoundAtPoolSlot=0
   win=[0]e_d_6_3_d_5_3_1 *[1]:d_5_3_5_0 [2]e_d_5_3_d_4_3_3 [3]:d_4_3_1_2
```
The car's physical edge is at pool slot **0**, but `LaneSeqIndex` is **1** (the internal lane past it).
`*` marks the seqIdx slot. `actEdgeFoundAtPoolSlot=0` proves the edge IS in the pool, one slot behind.

### Mechanism (complete causal chain)
1. The live-city demo sets `<lanechange.duration value="2.0"/>` (`LiveCitySim.cs:215`) → lane changes are
   **4-step continuous maneuvers** (`LaneChangeSteps = 2.0/0.5`). Every PARITY golden uses
   `LaneChangeDuration=0.0` (instant snap) — so this whole failure mode is **impossible on any golden**,
   i.e. a fix here is byte-identical/parity-safe by construction.
2. A vehicle mid-maneuver crosses a junction boundary during those 4 steps. The boundary-cross code
   (`Engine.cs` ~9682-9693) advances `LaneSeqIndex++` and sets `LaneHandle = arrival[newIndex]` (the
   internal/next lane) but does **NOT** cancel the in-progress maneuver; `LcTargetHandle` still points at
   a lane on the **edge just left**.
3. Next step `AdvanceLaneChanges` (`Engine.cs` ~11021) completes the maneuver: `LaneHandle =
   LcTargetHandle` — snapping the car's physical lane **backward onto the departed edge**, while
   `LaneSeqIndex` already points one slot ahead (the internal/next-edge). => `pool[LaneSeqIndex].edge !=
   LaneHandle.edge` (the `poolEdgeMismatch` desync).
4. At the next lane end the boundary guard (`v.LaneHandle != pool[seqIdx]`) sends the car to
   `TryReResolveFromActualLane`, which only handles a wrong LANE on the SAME edge; for a wrong EDGE it
   bails (`remaining[0] != currentLane.EdgeId`) and the caller clamps `Pos=laneLength; Speed=0` FOREVER.
   That clamped car is the "frozen on green, nowhere to go" seed; a stream of them walls corridors and the
   grid cascades. This is the whole of #15 (matches owner's "160 cars can't congest -> it's a bug").

### THE FIX (prong 2, next — design-first, parity-safe by construction)
A lane-change maneuver must not straddle a junction. At the boundary cross, FINALIZE any in-progress
maneuver before advancing `LaneSeqIndex`: if the vehicle is past the maneuver midpoint snap to the target
lane, else abort to the source lane, then clear `LcTarget*`/`LcSteps*` — so `AdvanceLaneChanges` can never
later write a stale cross-edge target. (Equivalently: `AdvanceLaneChanges` must skip/clear a maneuver whose
`LcTargetHandle` edge != the vehicle's current edge.) SUMO completes/aborts the lateral change at the lane
end; it never carries a discrete lane-target across an internal junction lane. Gated implicitly by
`LaneChangeDuration>0` (0 on every golden => inert => `Sim.ParityTests` 657/4 unchanged). This is the
actual #15 cure; the reroute-at-approach knob (task #21) treated the symptom and is correctly default-off.

### Instrumentation landed (parity-safe, gated `DiagSeqDesync` default off; 657/4 byte-identical)
`Engine.DiagSeqDesync` + `CheckSeqDesync` (flip-detected `SEQDESYNC-CREATED@site` with a pool window),
wired via `LIVECITY_SEQDESYNC=1`. Sweeps after AdvanceLaneChanges/UpdateReroutes/UpdatePeriodicReroutes/
ExecuteMoves + checks at ExecuteMoveVehicle entry and each boundary cross. All no-ops when the flag is off.

## PRONG 2 COMPLETE — #15 CURED (measured, parity-safe)
Implemented the fix from `docs/LIVE-CITY-15-LANECHANGE-JUNCTION-FIX-DESIGN.md`: a lane-change maneuver
may not straddle a junction. Two guards + `ClearLaneChangeManeuver(v)` helper (`Engine.cs`):
1. Boundary cross (ExecuteMoveVehicle): after `LaneSeqIndex++`/`LaneHandle` update, clear any in-progress
   maneuver (`LcTargetHandle >= 0`) so it can't re-apply onto the departed edge.
2. AdvanceLaneChanges: before the midpoint flip, if the maneuver target's edge != the vehicle's current
   edge, abort the (stale) maneuver instead of snapping LaneHandle back.
Both inert when `LcTargetHandle < 0` (always, for `LaneChangeDuration == 0`) => byte-identical on goldens.

**Measured (all first-hand):**
- Parity **657/4** byte-identical; bench hash **D96213B7BB4021A7**, parallel==single. Unchanged.
- `LIVECITY_SEQDESYNC=1 --frames 800`: **0** `SEQDESYNC-CREATED` (was >=25) -- desync eliminated.
- Gridlock repro `LIVECITY_TELEPORT=0 LIVECITY_CARS=160 --frames 2000/3000`: arrivals **258 -> 553 (t=940)
  -> 713 (t=1500), still climbing**; `stoppedFrac` 0.99 -> ~0.4-0.65 mid-run (oscillating, never terminal);
  `meanSpd` holds 3-7 m/s vs baseline 0.01. Traffic sustainably drains -- the terminal lock is GONE.
- Residual (separate, smaller): high-t (>1300s) `stoppedFrac` creeps to ~0.8-0.94 while still draining
  (arrivals climbing) -- a peak-load effect, not the desync lock. Track separately if the demo needs it.

`WrongLaneRerouteAtApproach` (task #21) stays default-OFF -- it treated the symptom; THIS is the cause.

## POST-CURE: eliminate the residual queue-blocking "floater" (owner priority: floaters must not block)
After the lane-change-straddles-junction CURE, the residual stuck cars are no longer desync strands
(`poolEdgeMismatch` = 0). The autopsy on the fixed build shows the remaining blockers are **`capSpent`**
strands: genuine wrong-lane cars that could not merge into their turn lane, rerouted twice, hit
`MaxDeadLaneReroutes=2`, and clamped `Speed=0` FOREVER -- one such car walls its whole approach queue
even though parallel lanes are free (owner's "single floater blocks the queue" report).

**Primary fix (owner: solve the cause, not just dodge it):** never let a wrong-lane car clamp. Re-enabled
`WrongLaneRerouteAtApproach` + `DeadLaneDriveThrough` as the live-city DEFAULTS -- previously measured as a
regression, but that was BEFORE the cure; with the desync cascade gone they now work cleanly:
| metric (cap 160, teleport off) | baseline | cure only | cure + never-clamp |
|---|---|---|---|
| arrivals @ t~1100-1380 | 258 (flat) | ~600 | **860 -> 1025** |
| stoppedFrac late | 0.99 | ~0.5-0.65 | **0.2-0.5** |
| strand reasons | poolEdgeMismatch huge | capSpent grows to 7k | **reResolveOK+rerouteOK only (0 strands)** |
| strandedDeadEnd / stuckInternal | 8-22 / 0 | small / 0 | **0 / 0-3** |
Every wrong-lane car RECOVERS (reroutes/drives-through) instead of clamping -> no floater ever walls a
queue. Parity **657/4** byte-identical (Engine props false on every golden). Owner's dodge idea
(followers route around a blocker; forward+lateral is physical from any speed incl. a full stop, only if
the target lane is free) is the FALLBACK -- kept as a future realism polish, not needed once the blocker
itself never forms.

## STILL OPEN (secondary, VISUAL -- needs the 3D viewer, not headlessly verifiable)
The lateral "float" the owner sees is a DR/render artifact: the ENGINE keeps every car on a lane
centerline (`LatOffset` is only written by overtaking-evasion, never by the lane-change maneuver, which
flips `LaneHandle` DISCRETELY at the maneuver midpoint). The renderer interpolates that ~1-lane-width
discrete jump over the tick => a visible sideways slide. Owner's physical rule (consolidated): lateral
displacement must always be accompanied by FORWARD displacement (a car can dodge from a full stop by
turning + creeping forward, but never slides purely sideways), and only if the target lane is free.
=> Fix is render/maneuver-kinematics (couple the lateral interpolation to forward progress, not wall
time), verified in the viewer -- tracked separately.

## UPDATED LANE-CHANGE / DODGE RULES (owner, 2026-07-23, supersede the earlier "no slow-speed change" note)
The earlier `LaneChangeMinSpeed=1.5` "no lane change while slow/stopped" rule was a PROXY for the real
intent and is too coarse. The owner clarified the actual physical model, in four refinements:
1. **The only thing forbidden is PURELY LATERAL motion** — a car sliding sideways with ZERO forward
   progress. That is physically impossible and is the unrealistic "float".
2. **Forward + lateral is always allowed** — a normal lane change / dodge traces a diagonal (forward and
   a bit of sideways). Fully physical.
3. **Even a FULLY STOPPED car may dodge** — it turns its wheels and creeps forward+sideways. So "speed==0"
   is NOT a valid reason to forbid a change; the constraint is on the MOTION (must have some forward
   component whenever there is lateral), not on the starting speed.
4. **Only if the target (parallel/overtaking) lane is FREE** — changing into an occupied lane is
   physically impossible and forbidden (standard lane-change gap safety; the engine already enforces this
   via its safe-gap check, i.e. it never opens an overlap).
Net rule to implement: *a car may change lanes from any speed including a stop, iff the target lane has a
safe gap, AND its lateral displacement is always accompanied by forward displacement (never pure-lateral).*

## NEW PROBLEM SET (current, post-#15-cure)
- **[SOLVED, primary] Floater that blocks a queue.** Root: a wrong-lane car that clamps `Speed=0` forever
  (`capSpent`), walling its lane while parallel lanes are free. Fixed by the never-clamp defaults
  (`WrongLaneRerouteAtApproach` + `DeadLaneDriveThrough`): the car reroutes/drives-through and keeps
  moving, so the blocker never forms. Owner: solving the CAUSE (blocker never forms) is preferred over the
  dodge fallback. Verified headlessly (arrivals 258->1025, 0 strand clamps).
- **[OPEN, secondary, VISUAL] Lateral "float".** The engine keeps every car on a lane centerline
  (`LatOffset` is written only by overtaking-evasion, never by the lane-change maneuver, which flips
  `LaneHandle` DISCRETELY at the maneuver midpoint). The renderer interpolates that ~1-lane-width discrete
  jump over the tick => a visible sideways slide, which looks like pure-lateral float when the car is not
  also moving forward. FIX DIRECTION (needs the 3D viewer to verify; NOT headlessly measurable): couple
  the rendered lateral interpolation to FORWARD progress (arc length travelled), not wall-clock time, so a
  car with no forward motion shows no lateral motion; and let the engine permit a change to START from a
  stop (rule #3 above) with the lateral realized only as the car creeps forward. Likely touches the
  City3D/DR lateral-interp path + the `LaneChangeMinSpeed`/AdvanceLaneChanges hold policy.
- **[OPEN, fallback] Dodge past a stopped blocker.** If a blocker ever does form, a follower should be
  able to change into a free parallel lane and pass (rules #1-4). Lower priority than removing the blocker
  itself; only needed as a safety net.
- **[PROPOSED] Liveness/throughput regression test.** Add a headless, no-SUMO test (fixed seeds) that runs
  the live-city smoke ~1000 s and asserts `arrivals >= threshold` and late `stoppedFrac <= threshold` —
  the guard the parity gate structurally cannot provide (the #15 bug was inert on every golden). Separate
  from the parity gate; thresholds set from the post-cure baseline.
