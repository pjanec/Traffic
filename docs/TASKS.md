# TASKS.md — Work queue for coding sessions

Each task is a **self-contained briefing**. A subagent starts from near-zero context, so a
task must name every input it needs: the `/sumo/` reference file, the target C# files, the
scenario, the command, and the numeric done-condition. Do tasks in order. One task = one
committed, green state you can check out later and continue from.

Read `CLAUDE.md` (rules) and `DESIGN.md` (architecture) before starting any task.

Legend: **[net]** = needs network + human (golden regen / vendoring); everything else is
the offline `dotnet test` loop.

---

## Task 0 — Bootstrap the harness (green on a blank checkout)

**Goal.** A committed test harness that passes `dotnet test` on a fresh clone into an empty
VM, **without SUMO and without any simulation engine existing yet**. This is the
checkout-and-continue baseline everything else grows from.

**Why it can be green with no engine and no SUMO.** The harness proves itself with a
*self-test*: feed the comparator two synthetic trajectories that are identical (assert zero
diff) and two that are deliberately offset (assert the diff is detected and localized).
This exercises the whole comparison path offline.

**Create the solution + projects:**
- `Sim.Core` — ECS components/systems/integration (empty scaffolding for now; no models yet)
- `Sim.Ingest` — `.net.xml` / `.rou.xml` parsers (empty scaffolding for now)
- `Sim.Harness` — FCD parsing + trajectory comparison (implement now)
- `Sim.ParityTests` — xUnit test project (implement the self-test now)

**Implement in `Sim.Harness`:**
- An **FCD parser**: read a SUMO `--fcd-output` XML into an in-memory model. Per timestep,
  per vehicle, capture: `id, lane, pos, speed, x, y, angle` and (when present)
  `acceleration`. Structure it for lookup by `(vehicleId, time)`.
- A **trajectory comparator**: given two trajectory sets + a tolerance config, return, per
  attribute: max-abs error, RMSE over the trajectory, and the **first timestep** where any
  attribute exceeds tolerance (or "no divergence"). Compare only vehicle/time pairs present
  in both; report any presence mismatch (missing/extra vehicles or steps) explicitly.
- A **`tolerance.json` schema** + loader: per-attribute tolerances (`pos`, `speed`, `x`,
  `y`, `angle`, `acceleration`) plus a `parityMode` field (`"exact"` | `"statistical"`)
  and an optional `comparedAttributes` list (phase 1 uses `["lane","pos","speed"]` — see
  DESIGN.md "layered comparison metric").

**Define the engine seam (no implementation):**
- `IEngine` in `Sim.Core`: loads a scenario (net + rou + cfg paths), runs N steps, and
  emits a trajectory set in the **same in-memory shape** the FCD parser produces, so engine
  output and golden output are directly comparable. Leave it unimplemented — later tasks
  fill it in.

**Implement in `Sim.ParityTests` (the self-test):**
- Construct two identical synthetic trajectory sets → assert comparator reports zero diff /
  no divergence.
- Construct two sets differing by a known offset at a known step → assert the comparator
  reports that attribute over tolerance and pinpoints the correct first-divergence step.
- Round-trip test: parse a tiny hand-written FCD XML fixture (commit it under
  `scenarios/_fixtures/`) and assert field values load correctly.

**Also create (committed, but not exercised by the test loop):**
- `scripts/install-sumo.sh`, `scripts/regen-goldens.sh`, `SUMO_VERSION` — already drafted;
  place them and make the shell scripts executable (`chmod +x`).
- `.gitignore` for `bin/`, `obj/`, NuGet caches, and any local SUMO install dir.

**Done-condition.** Fresh clone into an empty VM → `dotnet test` **passes on the self-test
alone**, with no SUMO installed and no engine implemented. Commit.

---

## Task 1 [net] — Vendor SUMO + generate the rung-1 golden

**Human/network step**, done once outside the offline loop.

- Vendor SUMO source at the tag matching `SUMO_VERSION` into `/sumo/` (see CLAUDE.md).
- Author the rung-1 scenario under `scenarios/01-single-free-flow/`:
  - `net.net.xml` — one straight edge, one lane, long enough to reach cruising speed
    (e.g. 1000 m), a single speed limit.
  - `rou.rou.xml` — one `<vType>` (passenger defaults; `sigma="0"`) and one vehicle
    departing at a fixed time/speed.
  - `config.sumocfg` — fixed `step-length`, Euler stepping, teleport off, no randomness.
  - `tolerance.json` — `parityMode="exact"`, `comparedAttributes=["lane","pos","speed"]`,
    tight tolerances (e.g. `pos` 1e-3 m, `speed` 1e-3 m/s).
- Run `scripts/regen-goldens.sh` → produces `golden.fcd.xml`, `golden.state.xml`,
  `provenance.txt`. **Commit them.**

**Done-condition.** `/sumo/` present at the correct tag; rung-1 scenario + goldens
committed with provenance stamped at `SUMO_VERSION`.

---

## Task 2 — Ingest + engine skeleton wired to rung 1

**Reference:** `/sumo/src/microsim/MSLane.cpp`, `/sumo/src/microsim/MSVehicle.cpp` for how
position is represented (lane-relative arc-length `pos`, global x/y derived).

- Implement `.net.xml` parsing in `Sim.Ingest` for the rung-1 subset: edges, one lane with
  its `shape` polyline, length, speed limit. Store the network as immutable arrays (see
  DESIGN.md), not entities.
- Implement `.rou.xml` parsing: `<vType>` attributes and a single vehicle/route.
- Implement `IEngine` enough to: place the vehicle, step with **fixed dt**, and emit a
  trajectory in the comparator's shape. Longitudinal position is lane-relative `pos`;
  derive x/y by walking the lane polyline (needed only if x/y are compared — phase 1
  compares `lane,pos,speed`, so x/y derivation can be minimal/stubbed).
- **Lateral field discipline (future-proofing, costs nothing now):** include a `LatOffset`
  in the transform and always write 0. Do not add lateral kinematics yet.

**Done-condition.** Engine runs rung 1 and emits a trajectory the harness can compare
(expected to be OUT of tolerance until Task 3 — that's fine; this task is plumbing).

---

## Task 3 — Krauss car-following, single vehicle free-flow parity

**Reference (read before porting):**
- `/sumo/src/microsim/cfmodels/MSCFModel_Krauss.cpp`
- `/sumo/src/microsim/cfmodels/MSCFModel.cpp` (base: `maximumSafeStopSpeed*`,
  `finalizeSpeed`, accel/decel bounds)
- Cross-check the safe-speed formula against the paper cited in the project docs
  ("SUMO's Interpretation of the Krauß Model"). **Do not trust remembered formulas** — the
  correct exact form is `v_safe = -b*tau + sqrt((b*tau)^2 + V^2 + 2*b*g)`; the Taylor form
  in the Gemini docs was transcribed with a misplaced gap term. Port from source, verify
  against golden.

**Implement the plan/execute contract (this is the load-bearing part):**
- **Plan phase:** each vehicle computes its next speed from the **start-of-step** state of
  its neighbors, writing the result to its own `MoveIntent` only. No shared writes. (Even
  single-threaded, honor this — it's what makes threading a later scheduling change, not a
  rewrite. See DESIGN.md.)
- **Execute phase:** apply intents, integrate position with the configured method (Euler in
  phase 1), advance `pos`.
- For rung 1 there is no leader, so `v_safe` is unconstrained by a leader; the vehicle
  accelerates (bounded by `accel`) toward `speedFactor * speedLimit` and holds. This
  isolates the acceleration bound + speed cap + integration before following matters.
- **Multi-constraint reducer (build the shape now):** compute next speed as the min over a
  *collection* of speed constraints, even though the collection has size 1 here. Junctions
  and (later) shadow lanes feed the same reducer. See DESIGN.md "seam 1".

**Verify vType init first:** diff your resolved passenger defaults against
`golden.state.xml` (accel 2.6, decel 4.5, sigma 0.5→but forced 0 here, tau 1.0,
minGap 2.5). Ruling out an init bug up front saves chasing trajectory drift.

**Done-condition.** `dotnet test` shows rung-1 trajectory within `tolerance.json` on
`lane,pos,speed` for all steps. Commit. This is the first real parity milestone — it proves
the plan/execute contract, the integration step, and the reducer shape on the smallest
possible surface.

---

## Next batch (define fully when Task 3 is green)

Kept here as a roadmap, not yet as briefings. Each becomes a self-contained task with
its own `/sumo/` references and scenario when we reach it:

4. **Two-vehicle following** — Krauss safe speed with a real leader; steady-state gap.
   Adds leader lookup from the per-lane sorted list. Ref: `MSLane` leader/follower logic.
5. **Approach to a stopped vehicle / dead end** — the discrete `maximumSafeStopSpeedEuler`
   overshoot-prevention math in isolation. The subtle one; nail it alone.
   - Note from rung 4 review: the leader constraint passes `predMaxDecel = leader vType
     `decel``. That is correct while `apparentDecel == decel` (the phase-1 default). If any
     vType overrides `apparentDecel`, revisit whether SUMO uses `getCurrentApparentDecel()`
     rather than `getMaxDecel()` for the leader term (`MSCFModel::maximumSafeFollowSpeed`).
   - Also: `maximumSafeFollowSpeed`'s emergency-decel correction (`decel!=emergencyDecel`) is
     ported but was unexercised by rung 4 — rung 5's hard stop is the first real test of it.
6. **Insertion spacing** — departure FIFO + gap-gated insertion. DONE.
   - Note from rung 6 review: `TryInsertOnLane`'s per-lane break-on-first-failure assumes a
     blocked earlier departure blocks all later ones — exact when all departures share one
     insertion point (as here). If a future scenario puts vehicles at DIFFERENT departPos on
     the same lane, revisit (SUMO retries each pending vehicle independently).
7. **Platoon shockwave** — still `sigma=0`, deterministic; multi-vehicle propagation.
8. **Two lanes + LC2013** — first structural change via command buffer; first real use of
   the multi-constraint reducer with a lateral intent. Ref: `MSLCM_LC2013`. **PARTIALLY DONE.**
   - 8a (scenario `06-two-lane-cruise`): two-lane `.net.xml` ingest + per-lane emission,
     vehicle stays right. Green, no engine change (parser already reads all `<lane>`).
   - 8b (scenario `07-keep-right-change`): the command buffer + lateral intent (seam 4,
     discrete instant lane-index snap) + a MINIMAL faithful slice of `MSLCM_LC2013`'s
     keep-right block, reproducing the exact accumulator so a single empty-road vehicle
     changes right at t=6. Right-neighbor guard leaves all single-lane rungs untouched.
   - **Deferred LC2013 work** (each its own future rung, needs its own scenario+golden):
     strategic (route/connectivity-forced) changes + general best-lanes (`getBestLanes`),
     cooperative changes, speed-gain overtake (the tactical block + `mySpeedGainProbability`),
     safety/blocker vetoes and the neighbor follower/leader gap checks (need a 2-lane
     scenario WITH traffic), `lanechange.duration>0` continuous lateral, and multi-edge lane
     continuity. Also: the command buffer currently applies each vehicle's lane swap inline
     in `ExecuteMoves` (fine for one changer); revisit batching if a scenario has multiple
     simultaneous changers competing for a gap (DESIGN.md conflict-resolution tie-break).
9. **Priority intersection** — right-of-way matrix + link-leader yielding, feeding the
   reducer. Ref: `MSRightOfWayJunction`, `MSLink`. **PARTIALLY DONE.**
   - 9a (scenario `08-junction-straight`): multi-edge routing + internal-lane traversal
     (route expands to a lane sequence via each `<connection>`'s `via`; pos carries over across
     lane boundaries). A major-road vehicle drives straight through, no yielding. Green.
   - **9b — priority yielding — DONE.** Scenario `scenarios/11-priority-junction/` (not `10-`; A1 took
     `10-`); `Rung9bParityTests` green within 1e-3. The whole hard rung was cracked WITHOUT a SUMO
     debug build by hand-matching the golden with the engine's own `KraussModel`. Mechanism (two
     phases, keyed on the foe's location): while the priority major is APPROACHING on its normal lane
     the minor does a stop-line brake `stopSpeed(approachLen − pos − POSITION_EPS=0.1)` (→ `9.433`/
     `4.933`); once the major is on its internal lane `:J_2_0` the minor runs `adaptToJunctionLeader`
     (→ `2.033`), whose `distToCrossing` uses the conflict-WIDTH shift `conflictSize = foeWidth(3.2) ×
     widthFactor(1)` (perpendicular) → crossing shifted back 1.6 m so `vStop = stopSpeed(distToCrossing
     − minGap) = 2.033`; once the major clears, no constraint (free `+2.6`/step). 9b-i (junction
     `<request>` + conflict geometry) and 9b-ii/iii (the reducer constraint + `adaptToJunctionLeader`
     port) are committed; `dotnet test` = 41 green. Determinism policy: static `<request>` priority
     matrix + frozen start-of-step foe snapshot → order-independent (no arrival-time race). Ported
     from `MSLink::setRequestInformation`/`getLeaderInfo` + `MSVehicle::adaptToJunctionLeader`. **Full
     per-step breakdown in `RUNG9B.md` "Progress log".** Summary characterization below.
     Scenario probed: major `WJ JE` (priority),
     minor `SJ JN` (yields); the minor brakes 13.89→9.433→4.933→2.033 as the major approaches,
     threads through just behind it, then accelerates.

     **Mechanism (reverse-engineered from source).** The minor yields by treating the crossing
     major as a virtual "link leader": `MSVehicle::checkLinkLeader` → `adaptToJunctionLeader`
     (MSVehicle.cpp). For a crossing foe with gap≥0 it computes
     `vsafeLeader = MAX2(followSpeed(gap=leaderInfo.second, leaderSpeed, leaderDecel),
     MIN2(v2, vStop))` where `vStop = stopSpeed(distToCrossing − minGap)`,
     `leaderPastCPTime = (distToCrossing − leaderInfo.second) / max(leaderSpeed, haltingSpeed)`,
     `vFinal = MAX2(speed, 2*(distToCrossing − minGap)/leaderPastCPTime − speed)`,
     `v2 = speed + ACCEL2SPEED((vFinal − speed)/leaderPastCPTime)`; then `v = MIN2(v, vsafeLeader)`.
     Geometry (this net): internal lanes cross at (201.60, 198.40) — `:J_1_0` (minor,
     x=201.60 vertical) meets `:J_2_0` (major, y=198.40 horizontal), 5.60 m into each; so
     `distToCrossing = 201.60 − pos` for both.

     **Why it is NOT a single-pass port (the blockers).** Reproducing 9.433 exactly needs, and
     none of these exist in the engine yet:
     1. **Junction conflict geometry** (`MSLink::myConflicts`: crossing point, `lengthBehind-
        Crossing` for both lanes, `conflictSize`/widths, `myRadius`/`sagitta`). netconvert/NBNode
        precomputes this at build time; it is NOT in the `.net.xml`. Must be reimplemented from
        the internal-lane shapes/widths (the crossing point itself IS derivable from geometry, as
        above, but the conflict widths/sagitta are more work).
     2. **Approaching-vehicle registration** (`MSLink::setApproaching`/`getApproaching` →
        `ApproachingVehicleInformation` with `arrivalTime`/`leaveTime`/`willPass`). Each vehicle
        registers its planned arrival/leave at every link it approaches; the minor reads the
        major's registration to decide. This is a genuinely two-vehicle-coupled, stateful pass —
        a new phase between plan and execute.
     3. **`MSLink::getLeaderInfo`** (~396 lines) — the foe-vehicle gap (`leaderInfo.second`) +
        `distToCrossing` computation, with many special cases (contLane, sameTarget/sameSource,
        inTheWay, pastTheCrossingPoint, indirect turns). This is where the exact gap convention
        lives; hand-derivation did not reproduce 9.433 (got 9.39 via followSpeed(0) or 11.57 via
        the vStop branch), i.e. the gap/priority conventions need the real getLeaderInfo, not a
        guess.
     4. **Priority / `isLeader`** (`MSLink::opened`, response/foe matrix, `myJunctionEntryTime`
        ordering) to decide the major is the leader.

     Recommended decomposition when tackled: (9b-i) parse the junction `<request>` response/foes
     matrix + compute conflict geometry from internal-lane shapes; (9b-ii) add the approaching-
     registration pass (arrivalTime/leaveTime) as new engine infrastructure; (9b-iii) port
     getLeaderInfo's gap for the single-crossing case + adaptToJunctionLeader; iterate against the
     golden (target 9.433/4.933/2.033). Also decide here the junction-determinism policy
     (match-SUMO-order vs deterministic tie-break-by-id; DESIGN.md "parallelization").
10. **Traffic light** — `<tlLogic>` state machine; red light as a stop-line constraint. **DONE**
    (scenario `09-traffic-light`; red → `stopSpeed(seen − DIST_TO_STOPLINE_EXPECT_PRIORITY 1.0)`,
    green → traverse; TL sampled at `time+dt` for the emit-before-plan ordering).
    - Note from rung 10 review: `TrafficLightState.IsRedOrYellow` matches only `'r'`/`'y'`;
      widen to `'u'` (red-yellow) and `'Y'` (yellow-major) before a scenario uses those phases.
      Yellow "stop if you can brake, else go" decision logic is also not yet built (no yellow here).
11. **Parameter-extraction cross-check pass** — automated diff of C# vType defaults vs the
    committed golden across all scenarios, run before trajectory tests as a fast fail. **DONE**
    (`ParameterCrossCheckTests`, a data-driven `[Theory]` over every scenario's
    `golden.vtype.json` `vTypes`; subsumes the former per-scenario init cross-checks). Note: it
    diffs against `golden.vtype.json` (the empirical libsumo/TraCI dump), NOT `golden.state.xml`,
    because `--save-state` does not expand implicit vType defaults (see the IMMEDIATE-task finding
    / DESIGN.md). Rung 1's pure-defaults reference stays covered by `VTypeInitCrossCheckTests`.

At rung 8+ decide explicitly the junction determinism policy (match-SUMO-order vs
deterministic-tie-break-by-id); see DESIGN.md "parallelization". Sublane/laneless mode is a
whole separate phase layered on top — no task here should preclude it (see the four seams).

---

## Extended roadmap — deferred features (characterized, not yet briefed)

Two distinct groups. **Group A** continues the SUMO-parity ladder: same golden-FCD validation, same
"port from `/sumo/`, match to 1e-3" bar, each its own scenario + golden. **Group B** is a direction
shift — reacting to *live external inputs that are not in an offline SUMO run* — so golden-FCD parity
does NOT directly apply; it needs a different validation model (see the Group B framing note). Both
groups are additive to the architecture (the multi-constraint reducer + the four seams + the command
buffer absorb them); neither is a rewrite. Order within each group respects the dependencies noted.

### Group A — completes the lane-based SUMO-parity core

- **A1. Multi-vClass vType resolver** (prerequisite; low-risk, high-value). **DONE.**
  `VTypeDefaults.Resolve` (was `ResolvePassenger`) now dispatches on `vType.VClass` over a curated
  road-vClass table: passenger, truck, bus, coach, delivery, trailer, bicycle, motorcycle, moped,
  emergency. Ported straight from the vendored source: `SUMOVTypeParameter.cpp`
  `VClassDefaultValues` ctor (minGap/maxSpeed/width/height), `getDefaultAccel`/`getDefaultDecel`/
  `getDefaultEmergencyDecel` (the `MAX2(decel, vcDecel)` form, kept derived — not hardcoded)/
  `getDefaultImperfection`, and `SUMOVehicleClass.cpp::getDefaultVehicleLength`. Passenger path is
  byte-identical (inert-when-absent); out-of-scope classes (rail*, tram, ship, pedestrian, …) still
  throw `NotSupportedException`; null/empty vClass resolves to passenger per SUMO's parser default.
  Validated by scenario `10-truck-free-flow` (truck accel 1.3 + maxSpeed 36.11 both bind; speed
  limit 40): `ParameterCrossCheckTests` picks up its `golden.vtype.json` and `RungA1ParityTests`
  checks the truck free-flow trajectory. `dotnet test` = 30 green. Adding another vClass now = one
  scenario + golden; the tests extend for free. `getVehicleStopOffset` was not needed (not a
  resolved-vType field). Remaining classes for A3/etc. (each its own scenario+golden when reached).
- **A2. Overtaking (speed-gain lane change). DONE.** `scenarios/12-overtake/` + golden: a fast
  follower overtakes a slow leader (`maxSpeed=5`) on a 2-lane edge — LEFT change at t11→t12,
  keep-right RETURN at t19→t20; `RungA2ParityTests` green within 1e-3. The "open subtlety" was NOT a
  look-ahead gap — it was step ORDERING: SUMO runs `planMovements → executeMovements → changeLanes`
  (`MSNet.cpp:784/790/796`), so the lane-change decision uses the POST-move leader gap. Feeding the
  post-move gap into `relativeGain = (neighLaneVSafe − thisLaneVSafe)/max(neighLaneVSafe,10)` (with
  `thisLaneVSafe = min(vMax, maximumSafeFollowSpeed(gap,…,onInsertion:true))`) reproduces the golden
  exactly (accumulate `+= relGain`, fire at `>0.2`, reset on change). Implemented as a NEW post-move
  phase `Engine.DecideSpeedGainChanges`; keep-right was MOVED into the same phase (both LC decisions
  run post-move, matching SUMO's single `changeLanes` pass) — 8a/8b stay byte-identical. Extended
  `LaneNeighborQuery` with adjacent-lane leader+follower; added a faithful `IsTargetLaneSafe`
  secure-gap veto (non-binding on the empty target lane here — the full blocker veto wants a scenario
  WITH target-lane traffic). `dotnet test` = 44 green. **Future note (non-blocking):** enforce a
  single LC decision per vehicle per step before a scenario stresses both keep-right and speed-gain
  on one vehicle in one step. **See `RUNGA2.md` for the full breakdown.** Original characterization
  below.
- **A2 (original). Overtaking (speed-gain lane change).** The other main branch of LC2013's `_wantsChange`
  (the one rung 8b did NOT port — rung 8b was keep-right only). A vehicle held up by a slower leader
  accumulates `mySpeedGainProbability` from the potential speed advantage and changes left when it
  crosses `myChangeProbThresholdLeft`; keep-right (rung 8b) brings it back after passing. Ref:
  `MSLCM_LC2013.cpp` `_wantsChange` speed-gain block + `mySpeedGainProbability` accumulation.
  **Requires the target-lane safety veto** — the neighbor leader/follower gap check
  (`MSLCM_LC2013::checkChangeBeforeCommitting` / the `blocked` logic) that rung 8b never exercised
  (it changed lanes on an empty road). So this is the first LC rung with *traffic on the target lane*
  and needs the `LaneNeighborQuery` extended to return the adjacent lane's leader AND follower.
  Scenario: a 2-lane road, a slow leader (maxSpeed capped, as rung 4's leader) with a fast follower
  that overtakes then returns. Note: on a SINGLE lane, "no overtaking" is already CORRECT (rung 4's
  follower correctly settles behind the slow leader forever) — this rung is strictly a multi-lane
  addition. De-risk the decision via TraCI `getParameter(..., "laneChangeModel.speedGainProbability"/
  "speedGainLP")` exactly as rung 8b used `keepRightP`.
- **A3. Priority / emergency vehicles. PARTIALLY DONE (ignore-red slice).** Two behaviors were in
  scope: (i) the emergency vehicle's own privileges (ignoring red/foes); (ii) OTHER vehicles giving
  way. **DONE: (i) ignore-RED** — `scenarios/16-emergency-red` + golden: an emergency vehicle with the
  junction-model ATTRIBUTE `jmDriveAfterRedTime="1000"` drives straight through junction J's RED
  light at free-flow, ported from `MSVehicle::ignoreRed` (the `jmDriveAfterRedTime > redDuration`
  arm). `VType`/`ResolvedVType` gained `JmDriveAfterRedTime` (default -1 = never ignore, so INERT for
  every other vType — rung 10 byte-identical); `TrafficLightState.GetPhaseElapsed` gives `redDuration`;
  `RedLightConstraint` returns +inf when the vehicle ignores red. Also the first real-SUMO validation
  of A1's emergency vClass table (ParameterCrossCheck on scenario 16). `RungA3ParityTests` green;
  `dotnet test` = 61. Gated ACCEPT.
  - **ER1 DONE (`!canBrake` ignore-red arm, PARITY).** `scenarios/50-emergency-red-dilemma` + golden:
    an emergency vehicle (`vClass=emergency`, `maxSpeed=25`, `jmDriveAfterRedTime="0"`) cruises WJ at
    25 toward J; the light is Green [0,11) then RED directly (no yellow), and at the step planning
    t=10→11 the vehicle (pos 255, seen 45 < brakeDist ~57.5) is too close to stop, so it PROCEEDS
    through the red at free-flow, crossing onto JE by t=12. KEY: SUMO's `MSVehicle::ignoreRed`
    (MSVehicle.cpp:7302) `!canBrake` arm is only reachable for a privileged vehicle (`ignoreRedTime>=0`),
    but for the stop-line brake the outer `MSVehicle.cpp:2754` `&& canBrakeBeforeStopLine` gate already
    makes ANY vehicle proceed when it cannot brake — and the engine already models this via
    `RedLightConstraint`'s `if (!canBrakeBeforeStopLine) return +inf` (added in C6's yellow decision,
    scenario 30 covered only YELLOW). So ER1 is a NON-VACUOUS RED-arm COVERAGE add with NO engine diff;
    `jmDriveAfterRedTime="0"` isolates the `!canBrake` return in SUMO (0≥0 takes the red branch, `0>redDuration`
    never fires). `RungER1ParityTests` green; `dotnet test` = 183. parity-reviewer ACCEPT.
  - **ER2 DONE (ignore-FOE at junctions, PARITY, HIGH-RISK).** Two SUMO foe-ignore mechanisms,
    both ported as default-inert vType flags (all 0 = "never ignore", so the ~30 junction scenarios
    stay byte-identical / bench hash 909605E965BFFE59 unchanged): (1) `jmIgnoreJunctionFoeProb`
    (MSVehicle.cpp:3430) gates `JunctionYieldConstraint`'s on-junction `AdaptToJunctionLeader` arm;
    (2) `jmIgnoreFoeProb` + `jmIgnoreFoeSpeed` (MSLink.cpp:898-902, from `opened()`) gates the
    approaching-foe stop-line-yield arm (speed-gated: only foes at/below `jmIgnoreFoeSpeed`). Two
    anchors, both reusing scenario 11's net with the yielding minor vehicle made an emergency vType
    (all probs 1.0 = deterministic): `scenarios/51-emergency-foe` (foe on-junction at t=0 → link-leader
    arm) and `scenarios/52-emergency-foe-approaching` (foe departs t=4, still approaching → opened() arm).
    Each non-vacuously isolates ONE gate (disabling it fails only that scenario). Probabilistic path
    (0<prob<1) uses a stateless per-entity salted `VehicleRng` draw (behavioral, no golden), never
    System.Random; prob>=1 / prob<=0 take no draw. `RungER2ParityTests` green; `dotnet test` = 189.
    parity-reviewer ACCEPT.
  - **ER3 DONE (give-way DETECTION + intent, BEHAVIORAL).** Adapts SUMO's `MSDevice_Bluelight` (a
    device that PUSHES a preferred lateral alignment onto neighbours — incompatible with the ECS
    plan/execute contract) into a per-ego PULL: each vehicle detects an approaching blue-light EV
    from the frozen snapshot and forms a "clear the way" intent (`VehicleRuntime.GiveWaySide`: 0/±1).
    New opt-in vType flag `HasBluelight` (default false) + `_anyBluelight` master switch so the whole
    subsystem is zero-work / byte-identical when no EV is present (scenarios 16/50/51/52 set no
    bluelight, unaffected). `DetectGiveWaySide` reads only the frozen snapshot, writes only the ego's
    own field, no System.Random, order-independent, skipped in the willPass pre-pass. Side rule
    mirrors SUMO's align-RIGHT-unless-leftmost-lane (rescue corridor). Exposed via
    `VehicleExportSnapshot.GiveWaySide` for the property test. `scenarios/53-giveway-single` fixture
    (no golden); `RungER3GiveWayDetectionTests` (reacts only in range, EV never self-reacts, no-EV
    inert). `dotnet test` = 194; bench hash 909605E965BFFE59 unchanged. parity-reviewer ACCEPT.
  - **ER4 DONE (give-way EXECUTION, multi-lane, BEHAVIORAL).** When a blue-light EV is approaching in
    a vehicle's OWN lane (`GiveWayEvSameLane`, computed in the plan phase alongside `GiveWaySide`),
    that vehicle changes to an adjacent lane to VACATE its lane for the EV, reusing the existing
    lane-change machinery (post-move neighbor snapshot + `IsTargetLaneSafe` gap veto + `CommitLaneChange`
    command buffer). Preferred direction from `GiveWaySide`, fallback opposite. A reacting vehicle's
    give-way change SUPPRESSES its ordinary keep-right/strategic/speed-gain (SUMO's bluelight disables
    strategic LC); a bluelight EV makes no overtaking change of its own (holds its lane, relies on
    others clearing). Both branches gated on `HasBluelight`/`GiveWaySide`, so the LC phase is
    byte-identical for every non-give-way scenario. `scenarios/54-giveway-multi` (2-lane, no golden):
    car0 in the rightmost lane (keep-right/speed-gain inert -> non-vacuous) vacates to e0_1 as the EV
    passes in e0_0. `RungER4GiveWayMultiLaneTests` (vacate + EV-passes + no-collision). `dotnet test`
    = 199; bench hash 909605E965BFFE59 unchanged. parity-reviewer ACCEPT.
  - **ER5 DONE (give-way EXECUTION, single-lane lateral drift, BEHAVIORAL).** The fallback when no
    lane change is possible: a vehicle with a give-way intent on a SINGLE lane (no left AND no right
    neighbour) pulls to the lane edge via a lateral drift (reuses B6's `DriftToward` in
    `ComputeLateralEvasion`; `GiveWayEdgeTarget` puts its outer edge on the lane boundary), then
    recenters when the intent clears. A same-lane leader whose lateral footprint no longer overlaps
    ego's imposes no car-following constraint (`!FootprintsOverlap` in `LeaderFollowSpeedConstraint`),
    so the EV passes the drifted car within a wide lane AND the drifted car does not brake for the
    passing EV. SELF-GATING / byte-identical: two lane-centred vehicles (LatOffset 0, the only state
    any golden scenario has) always overlap, so the hot-path bypass never fires; the drift arm needs
    `GiveWaySide != 0`. `scenarios/55-giveway-drift` (single 6 m lane, no golden): car0 drifts to the
    edge, the EV passes centred (no overlap), car0 recenters. `RungER5GiveWayDriftTests` (drift +
    EV-passes + no-collision + recenter + no hard-brake). `dotnet test` = 202; bench hash
    909605E965BFFE59 unchanged. parity-reviewer ACCEPT. **A3 give-way arc COMPLETE.**
    "priority *road*" right-of-way is rung 9b, not this.

- **A-impatience. Junction-yield IMPATIENCE / arrival-time gap acceptance. DEFERRED — take when the
  low-density teleport residual (mechanism B) or a saturated-junction throughput gap becomes blocking.**
  Full diagnosis + fix spec already written: `docs/SUMOSHARP-LOWDENSITY-TELEPORT-DESIGN.md` §T3 (do NOT
  re-derive — it has the oracle evidence and code anchors). Summary: the engine hardcodes
  `impatience == 0`, so at a saturated junction a vehicle waits for a perfect gap that never comes,
  whereas SUMO's growing impatience makes it accept a tighter gap and break the deadlock. Result: on
  `scenarios/_repro/synthetic-junction2` vanilla drains the queue into TL junction 2336 (15 through, 0
  stuck by t~500) while SumoSharp never drains it (3 permanently stuck, 10 through) — the standing jam
  feeds ~5 spurious teleports. The `havePriority` fix (committed, see the design doc T1) already removed
  the TLS half (10→5); this is the remaining half.
  - **Port from:** `/sumo/src/microsim/MSLink.cpp` `blockedByFoe` (lines ~947-965 — the impatience
    blend of the foe arrival window toward ego's braking-arrival), `MSVehicle::getImpatience`, and the
    `--time-to-impatience` default (180; `MSFrame.cpp:481`). Grows with accumulated `getWaitingTime`.
  - **Target C#:** `src/Sim.Core/Engine.cs` — the junction-yield arms (`JunctionYieldConstraint`'s
    approaching-foe crossing yield and `SameTargetMergeConstraint`'s PHASE-0 arrival-time RoW). A prior
    session already proved a byte-identical impatience port is possible (goldens unaffected).
  - **THE HARD PART / why it is not a patch:** applying impatience to the approach arm naively
    regressed `WillPassSaturationDiagTests` (0→15 stuck). The open design question is *where/how* to
    apply the blend faithfully without re-breaking that saturated-grid stress test. Needs its own
    design cycle, not a tweak.
  - **Done-condition:** `synthetic-junction2` SumoSharp teleports → ~0 (matching vanilla) and the
    pop%↔density curve monotone within noise; **all committed goldens byte-identical**; AND
    `WillPassSaturationDiagTests` + `RungHDp2g2*` still green. Tighten `LowDensityTeleportTests`'s
    `<= 5` bound toward 0 when it lands.

### Group B — beyond parity: live external-obstacle reactivity

**Framing (read before briefing any B task).** These react to obstacles injected from OUTSIDE the
simulation — a non-SUMO vehicle, a pedestrian, a robot, a real-world detection — that are NOT present
in the fixed offline SUMO run the goldens come from. Consequences:
- The golden-FCD parity harness does **not** directly validate them (there is no SUMO golden for "an
  external object appeared at t=12"). Validate with **behavioral / property tests** (e.g. "the
  vehicle's front never overlaps the obstacle", "it resumes within N s of the obstacle clearing",
  "it never routes onto a closed edge") plus, WHERE a SUMO analog exists (dynamic rerouting via
  `MSDevice_Routing`/`<rerouter>`, or a stopped blocker), a targeted parity scenario.
- This makes the engine a **live simulator with external inputs**, not only an offline parity
  reproducer. `IEngine` needs an input surface to inject/update/remove external obstacles between
  steps (a small API: `AddObstacle(laneId, pos, length, ...)`, cleared/updated per step). Keep it
  behind the same plan/execute discipline: obstacles are read start-of-step, like any neighbor.
- It also partly leaves strict SUMO parity as the bar — record that explicitly per task, and keep
  these features **gated/optional** so they never perturb the parity scenarios (same inert-when-
  absent guard pattern as rungs 8b/10).

- **B1. External-obstacle ingestion + "stop before blocker". DONE.** `IEngine` gained
  `AddObstacle`/`RemoveObstacle`/`ClearObstacles` (an `ExternalObstacle` = lane, frontPos, length,
  optional [startTime,endTime) window); an active obstacle feeds the multi-constraint reducer as a
  virtual STOPPED leader (speed 0) via `KraussModel.FollowSpeed`, `+inf` (inert) when none — so every
  parity scenario stays byte-identical. Validation is behavioral (Group-B bar), NOT golden-FCD:
  `RungB1ExternalObstacleTests` asserts no-overlap (follower front ≤ obstacle back every step), the
  Krauss steady gap, and resume-on-removal (stop → accelerate → 13.89). The steady gap is
  cross-checked against the committed SUMO analog `scenarios/13-stopped-leader` (real stopped leader →
  follower front 242.499). Fixture: `scenarios/14-external-obstacle` (single follower, NO golden).
  `dotnet test` = 48 green. Gated ACCEPT.
- **B2. Network routing layer. DONE.** `Sim.Ingest.NetworkRouter(NetworkModel)` + `Route(fromEdge,
  toEdge)` — Dijkstra over the edge-connectivity graph (arc A→B iff a `<connection from=A to=B>`
  between two normal edges exists = SUMO's turn-permission graph; internal `:`-edges excluded), edge
  cost = `length / max-lane-speed` (free-flow travel time, SUMO's `DijkstraRouter`/`MSDevice_Routing`
  default effort), deterministic (dist, then edge-id) tie-break, `[from]` for from==to, `null` for
  unknown/internal/unreachable. Purely additive (no engine/parity code touched). Validated by
  `RungB2RouterTests` against a committed SUMO `duarouter` golden
  (`scenarios/_fixtures/routing-diamond/`: top path AB/BD beats bottom AC/CD; golden routes
  `SA→DE`/`SA→CD`/`AB→DE` reproduced exactly) + trivial/unreachable/turn-permission cases. `dotnet
  test` = 55 green. Gated ACCEPT. Ready for B3 to consume (`Sim.Core` references `Sim.Ingest`).
- **B3. Reroute-around on prolonged blockage. DONE.** When an active B1 obstacle sits on a FUTURE
  edge of a vehicle's route and persists past `Engine.RerouteThresholdSeconds`, the engine recomputes
  a route via the B2 `NetworkRouter.Route(currentEdge, destEdge, avoid={blockedEdge})` and replaces
  the vehicle's `LaneSequence` (seam-4 structural mutation, run once per step before plan; keeps Pos
  since `newEdges[0]==currentEdge`; `AvoidedEdges` prevents re-triggering). INERT by default
  (`RerouteThresholdSeconds` = +inf → `UpdateReroutes` returns immediately), so no parity scenario is
  perturbed. `NetworkRouter` gained a `Route(from,to,avoidEdges)` overload. Validation is behavioral
  (Group-B): `RungB3RerouteTests` on `scenarios/15-reroute` (diamond net, vehicle routed top path
  SA AB BD DE) — (1) persistent obstacle on BD + threshold 5 → diverts to the bottom path
  SA AC CD DE, never enters BD, reaches DE; (2) obstacle clears before the threshold → keeps the top
  path; (3) disabled by default → inert even with an obstacle present. `dotnet test` = 59 green.
  Gated ACCEPT. (Optional future: a matched SUMO `<rerouter>` parity scenario.)
- **B4. U-turn when no route around exists. SKIPPED — superseded by navmesh/RVO movement.** A
  free-form reversal maneuver is a poor fit for SUMO's lane-based car-following (opposite-edge/bidi
  topology + a reversal that has no clean golden). Decision (session): defer this to a
  continuous/agent-based movement layer (navmesh path + RVO collision avoidance), where a U-turn is
  natural and the validation bar is behavioral/plausibility, not golden-FCD parity. See the
  navmesh/RVO note under Group C (C10 sublane/continuous is the bridge) and the external-agent interop
  (B5). Not planned as a lane-based rung.

**Suggested order (Groups A/B, done this session):** A1 → 9b (RUNG9B.md) → A2 → B1 → B2 → B3 → A3.
All committed green. Below is the realism roadmap that extends the engine toward believable ground
traffic; it is characterized (not yet briefed), same as the original "next batch" was.

---

## Realism roadmap (Group C + external-agent interop) — characterized, not yet briefed

Grounded in a session-end gap analysis. Two organizing facts drive the order:

1. **The determinism ladder.** Phase 1 is `sigma=0`/Euler/`actionStepLength=1` for EXACT parity.
   Almost everything realistic (stop-and-go waves, capacity, heterogeneous speeds, gap acceptance)
   needs `sigma>0` and per-entity RNG → a **statistical parity** bar (aggregate/ensemble, not 1e-3).
   `tolerance.json` already carries a `parityMode` field for exactly this. C1 is the gate.
2. **Lane-plan vs edge-plan.** Routing (B2) and LC so far are EDGE-level. Correct multi-lane traffic
   needs a LANE plan (which lane reaches the next connection). C2 is the gate for that.

Keep every new feature **inert-when-absent** so the deterministic parity scenarios (rungs 1–11, A1–
A3) remain the byte-for-byte correctness anchor (same discipline as rungs 8b/10/B1–B3/A3).

### The external-agent interop (the "SUMO respects non-SUMO agents" direction — HIGH PRIORITY)

- **B5. DONE (all three sub-rungs B5-i/B5-ii/B5-iii). Moving external agents as dynamic obstacles /
  foes (generalizes B1).** B1 already lets SUMO
  lane-based vehicles STOP behind a STATIC external obstacle (a virtual stopped leader on one lane).
  Generalize to **moving** external agents driven OUTSIDE SUMO (navmesh + RVO, a pedestrian crowd, a
  real detection): SUMO vehicles must *respect* them as (a) a dynamic **leader/follower on a lane**
  (obstacle carries a velocity → `FollowSpeed` with `predSpeed≠0`, and a `predMaxDecel`), (b) a
  **cross-lane blocker** vetoing lane changes (feed into A2's `IsTargetLaneSafe`/neighbor query), and
  (c) a **junction foe** the reducer yields to (feed the external agent's position/arrival into 9b's
  `JunctionYieldConstraint` as an approaching foe). The external agent's motion is NOT SUMO's model,
  so there is NO golden — validate **behaviorally** (no overlap; SUMO vehicle yields/brakes/avoids
  correctly; resumes when clear). This is the core two-way-sharing interop the project is aiming at:
  the navmesh/RVO agents move freely; the lane-based traffic reacts. Reuse the B1 `_obstacles` surface
  (extend `ExternalObstacle` with velocity/heading + a per-step update). Depends on B1 + 9b + A2.
  Inert-when-absent.

  **Decomposed (like 9b) — one shared `ExternalObstacle` velocity extension, three integration points:**
  - **B5-i. DONE. Dynamic leader/follower on a lane.** `ExternalObstacle` (Sim.Core/ExternalObstacle.cs)
    extended with `Speed` and `MaxDecel` (both default 0.0, so every existing `AddObstacle` call site
    is unaffected). `IEngine`/`Engine` gained `AddMovingObstacle(id, laneId, frontPos, length, speed,
    maxDecel, startTime=-inf, endTime=+inf)` (add-or-replace by id, same keying as `AddObstacle`) and
    `UpdateObstacle(id, frontPos, speed)` (no-op if `id` absent; preserves LaneId/Length/StartTime/
    EndTime/MaxDecel via `record with`). `Engine.AdvanceObstacles(dt)` dead-reckons `FrontPos +=
    Speed*dt` for every `Speed != 0` obstacle, called ONCE per step in the `[SystemPhase.Input]`
    section of `Run(int)` — BEFORE `neighbors.Refill`/`PlanMovements` — so the Plan phase always reads
    an already-advanced-for-this-step but otherwise FROZEN obstacle position (never mutated mid-plan);
    `Speed==0` obstacles are skipped entirely, so AdvanceObstacles is a no-op for every B1/static
    obstacle. `ObstacleConstraint` now passes `predSpeed: nearest.Speed`, `predMaxDecel: nearest.Speed
    != 0 ? nearest.MaxDecel : v.VType.Decel` — the conditional is what keeps a `Speed==0` obstacle
    byte-identical to B1: at `predSpeed=0`, `KraussModel.BrakeGap(0, ...)` is 0 regardless of the decel
    argument, so the formula provably never uses `predMaxDecel` in that case, and it still receives the
    same `v.VType.Decel` B1 always passed. For a moving obstacle this exactly mirrors
    `LeaderFollowSpeedConstraint`'s real-leader call. New behavioral tests in
    `tests/Sim.ParityTests/RungB5MovingObstacleTests.cs` (mirrors `RungB1ExternalObstacleTests`'s
    structure/idiom, reusing `scenarios/14-external-obstacle`): (1) no-overlap-ever against a
    per-step-reconstructed moving obstacle back position, plus late-state trailing at a positive speed
    near the obstacle's own speed with a bounded gap; (2) resume-to-free-flow-max within a bounded step
    count after the obstacle deactivates (`endTime`); (3) a `Speed=0` `AddMovingObstacle` reproduces
    B1's exact stop-at-`242.499` steady state, proving the moving path degenerates exactly to B1.
    `RungB1ExternalObstacleTests` and scenarios 13/14 verified unchanged/still green. Full suite: 67
    green (64 baseline + 3 new Facts), 0 failed.
  - **B5-ii. DONE. Cross-lane blocker vetoing lane changes.** `DecideSpeedGainChanges` (Engine.cs) now
    takes `(double time, double dt)` (was `dt` only) — `Run(int)`'s call site threads its loop `time`
    through, needed only so the veto below can evaluate an obstacle's `[StartTime, EndTime)` active
    window at the same instant every other obstacle read this step uses; nothing else in the pre-
    existing keep-right/speed-gain math reads `time`. New `TargetLaneBlockedByObstacle(ego, targetLane,
    time, dt)`: returns `false` immediately when `_obstacles` is empty (the same empty-store fast path
    `ObstacleConstraint` documents — the inert-when-absent guard). Otherwise, for each obstacle active
    at `time` whose `LaneId == targetLane.Id`, treats it as a virtual neighbor against ego's PROJECTED
    target-lane slot `[ego.Pos - ego.VType.Length, ego.Pos]` (the instant lane-index snap the commit
    gate performs uses ego's own POST-move Pos unchanged, only LaneId moves): obstacle entirely ahead
    (`obstacleBack >= egoFront`) is a virtual `neighLead` requiring `IsTargetLaneSafe`'s own leader
    secure-gap (`SecureGap(ego.Speed, ego.VType, obstacle.Speed, obstacle.MaxDecel, dt)`, mirroring
    `LeaderFollowSpeedConstraint`/`ObstacleConstraint`'s existing predSpeed/predMaxDecel plumbing
    exactly); obstacle entirely behind (`obstacleFront <= egoBack`) is a virtual `neighFollow` requiring
    the same secure-gap with the obstacle playing follower — since `ExternalObstacle` has no vType,
    two documented (conservative, gap-widening) proxies stand in: follower decel is
    `obstacle.Speed != 0 ? obstacle.MaxDecel : ego.VType.Decel` (exactly `ObstacleConstraint`'s own
    conditional) and follower minGap/Tau reuse ego's own `VType.MinGap`/`VType.Tau` (no B5-i precedent
    exists for an obstacle-as-follower role at all, so this is a fresh, explicitly-documented choice);
    any other overlap of ego's projected slot is a hard veto (no secure-gap arithmetic needed — there
    is no room to change into at all). `SecureGap` was split into a raw-decel/tau overload (the
    ResolvedVType-taking overload now just forwards to it) so the obstacle-as-follower branch has
    somewhere to pass its proxied decel/tau without a fake `ResolvedVType`; both overloads are
    byte-identical for every existing real-vType call site. Wired into the A2-iii commit gate as a
    SECOND, ANDed veto: `if (IsTargetLaneSafe(...) && !TargetLaneBlockedByObstacle(v, leftLane, time,
    dt))` — a vetoed change does NOT reset `SpeedGainProbability` (same no-reset-on-veto semantics
    `IsTargetLaneSafe`'s own veto already had: MSLCM_LC2013.cpp:1063/1080 only resets on an actually-
    COMMITTED change), so the vehicle keeps its lane, keeps accumulating, and retries every subsequent
    step until the obstacle clears. Inert-when-absent / A2-byte-identical: with `_obstacles` empty the
    helper's own fast path makes the `&&` a no-op, so `RungA2ParityTests`/scenario 12 are untouched and
    remain byte-for-byte identical (verified: unchanged files, still green). New behavioral tests in
    `tests/Sim.ParityTests/RungB5LaneChangeVetoTests.cs` (mirrors `RungB5MovingObstacleTests`'s idiom,
    reuses `scenarios/12-overtake`, anchored on the golden-verified `follow` overtake step t=11->12):
    (1) baseline sanity — no obstacle, `follow` still changes to `e0_1` at t=11->12; (2) a whole-lane
    (`FrontPos`==`Length`==lane length, so back=0) static obstacle on `e0_1` active through t=20 holds
    `follow` on `e0_0` for the ENTIRE window (t=11..19), never lets it reach `e0_1` while active, then
    `follow` completes the change within a few steps once the obstacle deactivates (proving the
    accumulator survived every vetoed retry — a delay, not a permanent block); (3) an obstacle on the
    NON-target lane (`e0_0`, positioned with a strictly-negative back so it is also inert to the
    separate `ObstacleConstraint` leader-follow check) does not affect the change at all — same t=11->12
    timing as baseline, proving the veto is target-lane-scoped. Full suite: 70 green (67 baseline + 3
    new Facts), 0 failed.
  - **B5-iii. DONE. Junction foe the reducer yields to.** `JunctionYieldConstraint` (Engine.cs) now
    takes `(double time)` (threaded from `ComputeMoveIntent`'s own `time` parameter, itself threaded
    from `PlanMovements`'s loop variable) — needed only so the new external-agent foe check below can
    evaluate an `ExternalObstacle`'s `[StartTime, EndTime)` active window at the same instant every
    other obstacle read this step uses (`ObstacleConstraint`/`TargetLaneBlockedByObstacle`'s own
    convention); nothing about the pre-existing 9b-ii/iii SUMO-foe machinery reads `time`. New
    `ExternalAgentOnFoeLane(foeInternalLaneId, time)`: returns `false` immediately when `_obstacles` is
    empty (the same empty-store fast path `ObstacleConstraint`/`TargetLaneBlockedByObstacle` already
    document — the inert-when-absent guard), otherwise `true` iff any obstacle is active at `time` AND
    sitting on `foeInternalLaneId` (an external agent "clears" a junction purely by its owner
    deactivating it via `EndTime` or calling `RemoveObstacle` — `AdvanceObstacles`'s dead-reckoned
    `FrontPos` never by itself changes `LaneId`, so lane membership alone is the complete, correct
    signal, exactly as B5-i/B5-ii already treat it; a future refinement letting an agent's own reported
    position signal "physically past the crossing point" is out of scope). Called from INSIDE
    `JunctionYieldConstraint`'s existing foe-link loop, right after `foeInternalLaneId` is resolved and
    INDEPENDENT of `FindFoeVehicle` (so it fires even with zero SUMO vehicles on the junction — the
    pure-external-agent case a `FindFoeVehicle`-only foe scan could never see, since an external agent
    is never wrapped as a `VehicleRuntime`): when true, `constraint = Math.Min(constraint, extConstraint)`
    where `extConstraint` reuses the EXACT approaching-foe stop-line yield the SUMO-foe branch already
    uses (`egoOnInternal ? +infinity : KraussModel.StopSpeed(approachLane.Length - v.Pos - PositionEps,
    ...)`) — ego brakes to a stop before ENTERING its own internal lane while the agent occupies a
    RESPONDED-TO foe lane (`JunctionRequest.RespondsTo`, the static `<request>` bitstring matrix — the
    SAME scoping 9b's SUMO-foe path already enforces, so an agent on a foe link ego's own request row
    does NOT respond to is correctly ignored), and is no longer gated once ego itself has been granted
    entry (`egoOnInternal`), identical to the SUMO-foe approaching branch's own short-circuit.
    Inert-when-absent / 9b-byte-identical: with no obstacle on any foe internal lane,
    `ExternalAgentOnFoeLane`'s empty-store fast path means the new `if` block is never entered, so the
    `Math.Min` beside it never executes and `constraint` is only ever touched by the pre-existing,
    untouched SUMO-foe path — `JunctionYieldConstraint` is byte-for-byte what it was before this rung
    for every obstacle-free scenario. Verified: `scenarios/11-priority-junction`, `08-junction-straight`,
    and `Rung9bParityTests`/`Rung9aParityTests`/`Rung9biJunctionGeometryTests` all unchanged/still green.
    New fixture `scenarios/_fixtures/junction-external-foe/` (behavioral, no golden — net.net.xml copied
    verbatim from scenario 11; rou.rou.xml has ONLY `vMinor` on the minor yielding route SJ→JN, no
    vMajor, so with no external agent the ONLY thing that could make it yield is an injected obstacle).
    New behavioral/differential tests in `tests/Sim.ParityTests/RungB5JunctionFoeTests.cs` (mirrors
    `RungB5LaneChangeVetoTests`' idiom): (1) baseline — with no agent (and no vMajor), `vMinor` never
    sustained-stops on its approach lane `SJ_0` and crosses to `JN_0` by t=17 (observed free-flow
    profile: departs at rest, reaches cruise speed 13.89 by t=6, clears the 11.20m internal lane
    `:J_1_0` inside a single 1s step around t=16→17); (2) an external agent on the RESPONDED-TO major
    foe lane `:J_2_0` (link 2, set in link 1's own `response="1100"` row) active `[-inf, 20)` forces
    `vMinor` to brake (the same 9.433/4.933 stop-line profile 9b's own SUMO-foe branch produces) and
    hold at ~pos 192.699 on `SJ_0` through t=19 (differential vs. the baseline, which is already deep
    into `JN_0` at that same t=19), never enters `:J_1_0` while the agent is active, then resumes and
    reaches `JN_0` by t=23 once the agent deactivates at t=20 — proving yield-then-go; (3) an agent on
    the NON-responded-to internal lane `:J_0_0` (link 0, not in link 1's response bits), active for the
    entire run, does NOT force any yield at all — trajectory identical to the no-agent baseline (crosses
    at t=17) — proving the `<request>`-matrix scoping is load-bearing, not "any obstacle near the
    junction stops everyone." Full suite: 73 green (70 baseline + 3 new Facts), 0 failed. **B5 (all
    three sub-rungs — dynamic lane leader/follower, lane-change veto, junction foe) is now DONE.**

- **B6. DONE. Emergency LATERAL EVASION (swerve around an external agent that jumped into the lane).**
  Behavioural (no SUMO golden — the external-agent bar). `ExternalObstacle` gained `LatPos`/`Width` (a
  lateral footprint; default `Width=0` == the pre-B6 FULL-LANE block, so every existing B1/B5 call is
  byte-identical). `AddObstacle`/`AddMovingObstacle`/`UpdateObstacle` gained matching optional
  `latPos`/`width` params (+ an `UpdateObstacle(id, frontPos, speed, latPos)` overload for a walking
  ped). Three coupled pieces in `Engine.cs`: (1) `ObstacleConstraint` gained a lateral-overlap gate — a
  finite-width agent only brakes ego while ego's CURRENT footprint overlaps it, so a car that has
  swerved clear proceeds past instead of stopping; (2) `ComputeLateralEvasion` (called from
  `ComputeMoveIntent`'s real-pass MoveIntent return, never the willPass pre-pass) picks a target
  `LatOffset` that clears the agent — WITHIN the ego lane if it fits, else SPILLING into a safe adjacent
  lane (`NeighborSpillSafe` reuses `IsTargetLaneSafe` against the neighbour's leader/follower), else
  holds and lets ObstacleConstraint brake to a stop — and drifts toward it bounded by
  `SwerveMaxLateralSpeed` (2.0 m/s); the threat test uses the CENTRED footprint so an in-progress swerve
  is sticky (no oscillation) until the agent is fully behind, then recentres. Swerve engages ONLY when
  braking alone cannot stop in time (the "sudden jump" case); a stoppable agent is still just braked
  for. (3) `LaneGeometry.PositionAtOffset` gained an optional perpendicular `latOffset` so the swerve is
  VISIBLE in the emitted x/y (0 for every lane-centred vehicle → byte-identical). INERT when no
  dodgeable obstacle is present: whole committed suite byte-identical (173 passed / 1 skip), `Sim.Bench`
  hash unchanged (`909605E965BFFE59`). Property tests `RungB6LateralEvasionTests` (swerve-within-lane +
  pass, fills-lane → brake-to-stop centred, spill-into-adjacent-lane, Width=0 → stop-dead-as-B1), each
  with an explicit no-collision assertion (lateral footprints disjoint whenever longitudinally
  overlapping). NOT a SUMO-parity behaviour (SUMO's sublane model has no emergency ped-swerve); this is
  the external-agent live-reactivity seam. Parity-reviewer ACCEPT.
  - **B6-lat (PREDICTIVE swerve for a LUNGING agent). DONE.** `ExternalObstacle` gained `LatSpeed` (the
    agent's lateral velocity; default 0 = byte-identical); `AdvanceObstacles(time, dt)` dead-reckons
    `LatPos += LatSpeed*dt` and now only advances an active, past-appearance obstacle (guard inert for
    every pre-B6 -inf..+inf window — B5 moving tests unchanged); `AddObstacle`/`AddMovingObstacle` gained
    `latSpeed`, plus an `UpdateObstacle(id,frontPos,speed,latPos,latSpeed)` overload. `ComputeLateralEvasion`
    now PREDICTS the agent's lateral centre at time-to-encounter (`LatPos + LatSpeed*tte`) for both threat
    detection and the clear-target, and picks the side the agent is VACATING (opposite `LatSpeed`) — so a
    ped lunging left FASTER than the car's 2 m/s swerve is dodged to the RIGHT, not steered into. New
    behavioural fixture `scenarios/_diag/wide-swerve` (6 m lane) + test
    `LungingPedestrian_CarDodgesToTheSideItIsVacating_NoCollision`. Inert (suite 174 green, bench hash
    unchanged). Parity-reviewer ACCEPT.

### Group C — realism beyond the deterministic phase-1 core

- **C1. Statistical parity / driver imperfection (`sigma>0`). THE determinism-ladder shift; do
  first — unblocks most of the rest.** Port Krauss dawdling (`MSCFModel_Krauss::dawdle`) and the
  per-vehicle SEEDED RNG (CLAUDE.md rule: no `System.Random`; seed per entity so results are
  independent of thread order). Add a **statistical** `parityMode` to the harness.

  **DECISION (owner, this session): the statistical bar is ENSEMBLE/AGGREGATE, not RNG-exact.** We do
  NOT reproduce SUMO's `RandHelper`/MT19937 per-vehicle stream (brittle, version-dependent, and it
  fights the ECS parallelism). Instead we validate aggregate properties over N seeds (mean + spread of
  speed/flow, or the fundamental diagram) within a statistical tolerance. Dawdle is ported to the
  ALGORITHM faithfully; only the RNG *stream* is ours (any good deterministic per-entity-seeded PRNG).

  **Decomposed (like B5/9b):**
  - **C1-i. DONE (OFFLINE, no SUMO). Dawdle + per-entity seeded RNG.** Ported
    `MSCFModel_Krauss::dawdle2` (sumo/src/microsim/cfmodels/MSCFModel_Krauss.cpp:129-151) and
    `MSCFModel_KraussOrig1::patchSpeedBeforeLC`'s `MAX2(vMin, dawdle2(vMax, sigma, rng))` bound
    (MSCFModel_Krauss.cpp:90-96, the default per-step `sigmaStep==DELTA_T` path only —
    MSCFModel_Krauss.cpp:73-89's `myDawdleStep > DELTA_T` sub-stepped `accelDawdle` machinery is
    DEFERRED/out of scope) as `KraussModel.Dawdle2`, called from `KraussModel.FinalizeSpeed`
    (`src/Sim.Core/KraussModel.cs`) right where `vNext` used to be set unconditionally to `vMax`:
    `vNext = vType.Sigma > 0.0 ? MAX2(vMin, Dawdle2(vMax, sigma, accel, dt, ref rng)) : vMax`.
    New `VehicleRng` struct (`src/Sim.Core/VehicleRng.cs`) — SplitMix64 (Vigna, public domain), a
    single unmanaged `ulong` of state, `NextDouble()` in `[0,1)`, `SeedFor(globalSeed,
    entityIndex)` mixing the two through one SplitMix64 step. Explicitly NOT SUMO's
    RandHelper/MT19937 stream (owner decision above) — determinism + per-entity independence is
    the bar, not stream-matching. `VehicleRuntime.RngState` (new `VehicleRng` field, unmanaged,
    D3-clean) is seeded ONCE at vehicle creation in `Engine.LoadScenario` from
    `VehicleRng.SeedFor(Seed, entityIndex)`; new `Engine.Seed` property (`ulong`, default 42,
    settable before `LoadScenario` — later ensemble harnesses vary it per run) drives the global
    seed. `ScenarioConfig.Seed`/`ScenarioConfigParser` now also parse the sumocfg's
    `<random_number><seed value="..."/></random_number>` (default 42) for future use, but it is
    NOT auto-applied to `Engine.Seed` (keeps `Engine.Seed` the single caller-controlled source of
    truth, so a caller setting it before `LoadScenario` for an ensemble sweep is never silently
    clobbered). `Engine.ComputeMoveIntent` threads `ref v.RngState` into `FinalizeSpeed` so the
    draw advances that vehicle's own private state in place — no shared/global RNG, so
    `UseParallelPlan=true` stays race-free (each `Parallel.For` iteration only ever touches its
    own entity's `RngState`, exactly the D8 argument already made for every other field). `sigma
    == 0` never calls `Dawdle2` (no draw at all, not a draw-then-multiply-by-zero) → **byte-
    identical to every existing deterministic rung**: confirmed via the full `dotnet test` run
    (78 passed, 0 failed, up from 73) AND by name-checking `Rung1ParityTests`/`Rung9bParityTests`
    (real `golden.fcd.xml`/`tolerance.json` comparisons) still pass unchanged. New fixture
    `scenarios/_fixtures/dawdle-single-lane` (reuses `scenarios/01-single-free-flow`'s net, one
    `sigma=0.5` vehicle "dawdler") + `tests/Sim.ParityTests/RungC1DawdleTests.cs` (5 new
    behavioral/property tests, no golden): same-seed determinism (byte-identical trajectories),
    sigma>0 reduces mean steady-state speed below free-flow max with positive step-to-step
    variance (contrasted against the sigma=0 control, zero variance), different seeds diverge but
    stay bounded (`speed∈[0,vMax]`, no NaN, non-decreasing position), and `UseParallelPlan=true`
    reproduces the sequential sigma>0 result exactly.
  - **C1-ii. DONE (OFFLINE, no SUMO). Statistical parity harness mode.** Added
    `TrajectoryComparator.CompareEnsemble(actualRuns, expectedRuns, tolerance)`
    (`src/Sim.Harness/TrajectoryComparator.cs`), a sibling to the existing exact-mode `Compare`
    (untouched). For each attribute in `tolerance.ComparedAttributes` (default list, minus `"lane"`
    — categorical, never averaged; silently skipped, documented in the method's XML doc) it pools
    EVERY `(run, vehicle, time)` sample of that attribute across the whole ensemble into one flat
    list (no per-run pre-averaging, no cross-ensemble run alignment) and computes the POPULATION
    mean and std (`sqrt(mean((x-mean)^2))`, not Bessel-corrected) for actual vs. expected. Verdict
    per attribute is `|meanActual-meanExpected| <= meanTolerance` AND `|stdActual-stdExpected| <=
    stdTolerance`; empty ensembles yield mean=std=0 by convention (guarded, no throw/NaN).
    `EnsembleComparisonResult`/`AttributeEnsembleComparisonResult`
    (`src/Sim.Harness/EnsembleComparisonResult.cs`) mirror `ComparisonResult`/
    `AttributeComparisonResult`'s reporting shape (actual/expected/error/withinTolerance for both
    mean and std) so a failing test prints which stat failed and by how much. Explicitly deferred:
    a per-time-bin flow/density (fundamental-diagram) variant — this only does whole-ensemble
    pooling, documented as the richer later extension.
    Schema: extended `ToleranceConfig` (`src/Sim.Harness/ToleranceConfig.cs`) with an optional
    `IReadOnlyDictionary<string, StatisticalAttributeTolerance>? Statistical` property (new record
    `StatisticalAttributeTolerance(double Mean, double Std)`) and `MeanToleranceFor`/
    `StdToleranceFor` accessors mirroring `ToleranceFor`. JSON shape (new optional top-level
    `"statistical"` object, keyed by attribute name):
    `{"parityMode":"statistical","statistical":{"speed":{"mean":0.5,"std":0.5},"pos":{"mean":5.0,"std":5.0}}}`.
    `parityMode="exact"` configs are untouched — the new DTO field is nullable/absent, and
    `ToleranceConfigDto`/`Load`/`Parse` parse existing exact JSON byte-for-byte as before (verified
    by a self-test that round-trips an existing-shape exact JSON and asserts `Statistical` is null
    and every prior accessor is unchanged).
    Self-test `tests/Sim.ParityTests/RungC1StatisticalHarnessTests.cs` (8 new tests, no SUMO, no
    scenario files, no golden — synthetic in-memory `TrajectorySet` ensembles built with a
    deterministic splitmix64-style local generator, mirroring the Task-0 comparator self-test
    idiom): identical ensembles match (mean/std error ~0); a mean-shifted actual ensemble fails and
    is attributed specifically to `MeanWithinTolerance=false` with `StdWithinTolerance=true`; an
    actual ensemble with inflated noise amplitude (same mean, much larger spread) fails and is
    attributed specifically to std, not mean; two independently-seeded same-parameter ensembles
    (small per-point jitter) stay within the tolerance band (proves the tolerance is a real band,
    not exact equality); `"lane"` in `comparedAttributes` is skipped without throwing even absent a
    statistical tolerance entry for it; empty ensembles yield mean=std=0 without throwing; a
    `parityMode="statistical"` JSON round-trips through `MeanToleranceFor`/`StdToleranceFor`; an
    `exact` JSON still loads unchanged and `MeanToleranceFor` throws on it (no `Statistical` block).
    Full suite: 86 passed, 0 failed (up from 78) — all prior exact-mode tests (`TrajectoryComparatorTests`,
    every `Rung*ParityTests` real-golden test) verified unchanged and green.
  - **C1-iii. DONE ([net] golden regen, offline test). Statistical golden + parity test.** New
    scenario `scenarios/17-dawdle-freeflow` (single vehicle, `sigma=0.5`, free flow on a 2000m single
    lane so it never reaches the end within `end=80` → all runs fully present for clean pooling).
    Golden = a 24-run SUMO ENSEMBLE (`golden.ensemble/seed01..24.fcd.xml`, `sumo --seed 1..24
    --precision 6`), committed with `provenance.txt`. `tolerance.json` is `parityMode="statistical"`,
    `comparedAttributes=["speed"]` (pos excluded — a cumulative, non-stationary quantity), with
    `statistical.speed.mean=0.05`/`std=0.05`. Test `RungC1iiiStatisticalParityTests` parses the 24
    golden FCDs as the expected ensemble, runs the engine over 24 seeds as the actual ensemble, and
    asserts `TrajectoryComparator.CompareEnsemble` (C1-ii) `IsMatch`. **This is a real, tight test,
    not a loose one:** because C1-i ported SUMO's `dawdle2` ALGORITHM faithfully, the two ensembles
    (different RNG streams, same formula) converge to the same pooled speed distribution — observed
    at authoring **meanΔ=0.001140, stdΔ=0.003687 m/s**, so the committed 0.05 band is ~13–40× that
    noise floor yet far tighter than any real dawdle bug (a `sigma` factor-of-2 shifts the mean
    ~0.65 m/s). The RNG stream is NOT compared (the ensemble-not-RNG-exact decision). Golden regen
    (SUMO) was a deliberate `[net]` step; the committed goldens make the test run in the offline
    `dotnet test` loop with no SUMO. Full suite: 87 passed, 0 failed.

  Produces stop-and-go waves and realistic capacity. Prereq for C7 and for believable everything.
  **C1 (all three sub-rungs) is now DONE — the determinism-ladder gate is open; the statistical bar
  (ensemble/aggregate) is built and validated against SUMO.**
- **C2. Strategic (route-driven) lane changes + lane-to-lane continuity. The #1 lane-based realism
  gap.** Today a vehicle can sit in a lane that cannot reach its route. Port LC2013's STRATEGIC block
  (`LCA_STRATEGIC`/`LCA_URGENT`, `getBestLanes`/`bestLaneOffset` — `MSLCM_LC2013::_wantsChange` +
  `MSLane::getBestLanes`) so a vehicle moves into a lane that continues its route. Requires
  **lane-level** routing: honor `<connection fromLane/toLane>` turn permissions (B2 is edge-level).
  Parity axis. Reuses A2's neighbor query + the post-move LC phase.

  **ARCHITECTURAL FINDING (this session): C2 is the largest structural Group-C rung — it reworks the
  lane-sequence model.** Today `NetworkModel.ResolveLaneSequence` precomputes an EXACT lane path
  (`_laneSeqPool` slice) at insertion and THROWS if the depart lane has no `<connection>` to the next
  edge; `ExecuteMoves`'s lane-boundary advance blindly walks that precomputed sequence
  (`v.LaneSeqIndex++; v.LaneHandle = pool[start+index]`), while speed-gain/keep-right LC change
  `v.LaneHandle` to a neighbor WITHOUT updating the sequence. These never conflict today only because
  every multi-edge route is single-lane-per-edge (no LC) and every multi-lane scenario is single-edge
  (no advance). C2 is the first rung to combine multi-lane + multi-edge + lane choice, so the
  advance must follow the vehicle's ACTUAL current lane's connection, not a precomputed path — a
  change that touches the core every multi-edge parity scenario (9a/9b/A3) depends on. Gate HARD for
  byte-identical on those.

  **`[net]` anchor DONE: `scenarios/18-strategic-turnlane`** (committed golden). E1 (2 lanes, A→B,
  500m) → E2 (1 lane, B→C); ONLY `E1_1` (left) connects to E2 (via `:B_0_0`), `E1_0` (right) is a
  drop lane. `veh0` routes E1→E2 departing `E1_0`, so it MUST strategic-change left before B. Verified
  SUMO trajectory (`golden.fcd.xml`, `sigma=0`, seed 42): on `E1_0` through t≤16, strategic-changes to
  `E1_1` by t=17, crosses `:B_0_0` at t=38, reaches `E2_0` at t=39. Net built from committed
  `nodes.nod.xml`/`edges.edg.xml`/`connections.con.xml` via netconvert. `tolerance.json` exact,
  `["lane","pos","speed"]` @1e-3.

  **Suggested decomposition (engine work, offline against the anchor golden):**
  - **C2-i (additive, byte-identical). `getBestLanes` lane-continuity data. DONE.** Added
    `LaneContinuation(LaneIndex, AllowsContinuation, BestLaneOffset, Length)` +
    `NetworkModel.ComputeBestLanes(routeEdges, currentEdgeId)` in `src/Sim.Ingest/NetworkModel.cs`
    — a scoped port of `struct LaneQ` / `MSVehicle::updateBestLanes`
    (`sumo/src/microsim/MSVehicle.h:865-886`, `sumo/src/microsim/MSVehicle.cpp:5744-6063`), queried
    off the existing `ConnectionsByFromEdgeLane` table (no XML re-parse). **Scope: single
    look-ahead only** — current edge → the immediately next route edge; a lane "continues" iff it
    has any `<connection>` to that next edge; `Length` is always just the current edge's own
    length (never a route-wide sum). **Deferred**: SUMO's backward recursion
    (`MSVehicle.cpp:6003-6063`) that accumulates a continuing lane's `Length` across every
    remaining route edge — not needed until a multi-junction scenario requires it. **Sign
    convention**: `BestLaneOffset` positive = toward the LEFT, matching `Lane.LeftNeighbor` ==
    Index+1 (confirmed against SUMO's own `bestLaneOffset = bestThisIndex - index`,
    `MSVehicle.cpp:5973`, positive toward a higher lane index — same sense as this repo's
    left-is-higher-index convention). Unit test `tests/Sim.ParityTests/RungC2iBestLanesTests.cs`:
    on scenario 18's `E1` (route `E1 E2`), `E1_0` (drop lane) → `AllowsContinuation=false`,
    `BestLaneOffset=+1` (toward `E1_1`, confirmed to be `E1_0.LeftNeighbor`); `E1_1` →
    `AllowsContinuation=true`, `BestLaneOffset=0`; last route edge `E2` → every lane continues,
    offset 0. Inert-control proof on `11-priority-junction`'s single-lane-per-edge minor (`SJ JN`)
    and major (`WJ JE`) routes: every lane on every route edge has `BestLaneOffset=0` and
    `AllowsContinuation=true`, i.e. C2-ii built on this data is inert/byte-identical on every
    existing single-lane-per-edge parity scenario. Purely additive — touched only
    `src/Sim.Ingest/NetworkModel.cs` + the new test file; no simulation/engine/LC code path
    changed. Full suite: 93 passed, 0 failed (was 87; +6 new tests).
  - **C2-ii (behavioral, SUMO golden). Strategic LC + actual-lane advance. DONE.** Ported
    LC2013's STRATEGIC/URGENT block (`_wantsChange`, `sumo/src/microsim/lcmodels/MSLCM_LC2013.cpp`
    ~1216-1327, `currentDistDisallows` at `MSLCM_LC2013.h:189-191`) plus the entangled lane-sequence
    rework the briefing called out. Broke the old `v.LaneHandle`/`pool[LaneSeqIndex]` lockstep exactly
    per the 4-point design:
    1. **Pool = route path via the continuing/best lane.** `NetworkModel.ResolveLaneSequence` now
       resolves the FIRST edge's start lane via `ComputeBestLanes` (C2-i) instead of always using
       `departLaneIndex` literally: if the depart lane already `AllowsContinuation`, nothing changes
       (byte-identical fast path — covers every existing scenario); otherwise it starts from
       `departLaneIndex + BestLaneOffset` (the continuing lane `ComputeBestLanes` points at). Scenario
       18's pool resolves to `[E1_1, :B_0_0, E2_0]` — the old "no `<connection>` found from the depart
       lane" throw can no longer be reached via a non-continuing depart lane (unchanged for a
       genuinely unreachable route). `ResolveLaneSequenceHandles`/`TryInsertOnLane`/`UpdateReroutes`
       needed no changes (they already call `ResolveLaneSequence` internally).
    2. **Actual lane tracked separately — already true.** `TryInsertOnLane` already set
       `v.LaneId`/`v.LaneHandle` to the DEPART lane BEFORE calling `ResolveLaneSequenceHandles`, so
       once (1) changed what the pool resolves to, actual (`E1_0`) and pool[0] (`E1_1`) diverge
       automatically at insertion — no engine changes needed here.
    3. **Strategic LC** — new `Engine.TryStrategicLaneChange`, called from `DecideSpeedGainChanges`
       BEFORE the existing keep-right/speed-gain block (and `continue`s past speed-gain when it
       fires). Gated on `pool[LaneSeqIndex]`'s HANDLE differing from `v.LaneHandle` on the same edge —
       exactly the point-3 offset≠0 condition, and the gate that makes the whole method a no-op
       (not even touching the new `VehicleRuntime.LookAheadSpeed` field) for every existing scenario,
       since `NetworkModel.ResolveLaneSequence`'s own byte-identical fast path means the pool is
       always built from the depart lane there. Ported faithfully: `myLookAheadSpeed` growth/decay
       (`.cpp:1227-1236`), `laDist = myLookAheadSpeed·10·myStrategicParam(1.0)·(right?1:
       myLookaheadLeft(2.0)) + 2·lengthWithGap` (`.cpp:1238-1239`), `usableDist = curr.Length −
       posOnLane` (occupation/stop terms scoped to 0/unset — empty road, no stop on this edge) and
       the `currentDistDisallows` trigger (`.h:189`). `changeToBest`/`bestLaneOffset==curr.
       bestLaneOffset` collapse to trivially true because only the ONE direction `BestLaneOffset`'s
       own sign requires is ever evaluated (equivalent to SUMO's two-sided caller for this trigger).
       On commit: the SAME `IsTargetLaneSafe`/`TargetLaneBlockedByObstacle` veto A2/B5-ii use (clear
       road ⇒ never binding here), then a command-buffer `ChangeLane` lateral snap + `SpeedGainProbability`
       reset (`.cpp:1063/1080`).
    4. **Advance requires convergence.** `ExecuteMoves`' boundary-advance loop now checks
       `v.LaneHandle == pool[LaneSeqIndex]` before crossing to `pool[index+1]`; if not converged at
       the lane end, it clamps `Pos` to the lane length and zeroes `Speed` (stop at the lane end)
       instead of teleporting onto a route path this lane never connected to. Always true for
       single-lane-per-edge routes (pool built from the depart lane) ⇒ unexercised guard, but present
       for safety per the briefing.
    **One extra, empirically-necessary fix beyond the 4-point design:** after convergence (on `E1_1`,
    `bestLaneOffset=0`), the EXISTING (untouched) keep-right accumulator would have — per hand-derived
    arithmetic (`deltaProb≈0.138/step`, threshold `-2.0`) — spuriously fired ~14 steps later and moved
    the vehicle BACK onto the `E1_0` drop lane, contradicting the golden (which never returns to
    `E1_0`). Root cause (found by reading `MSLCM_LC2013.cpp:1398-1410`, the "opposite direction" STAY
    guard): real SUMO's `_wantsChange` returns early — BEFORE the keep-right accumulator is ever
    touched — once `currentDistDisallows(neighLeftPlace, |bestLaneOffset|+2, laDist)` holds, which for
    this net's numbers becomes true immediately upon convergence (`posOnLane > 188.2`, and the vehicle
    enters `E1_1` at `pos=205.68`). Ported the OBSERVABLE effect only (not the full early-return-
    before-accumulation semantics, since `KeepRightProbability` is not itself a compared golden
    attribute — only `lane`/`pos`/`speed` are): a new commit-time veto in `ApplyKeepRightDecision`,
    `LaneContinuesRoute(v, lane, rightLane.Index)` (a thin `ComputeBestLanes` wrapper, with a
    `route.Edges.Count <= 1` fast path that skips the call entirely for every single-edge scenario),
    ANDed into the existing `keepRightProbability * keepRightParam < -changeProbThresholdRight` commit
    gate. Verified inert for 06/07/12 (the only scenarios with a valid `RightNeighbor`) by exhaustive
    check: every scenario with multi-lane edges (06/07/12) is single-edge-route (`ComputeBestLanes`'
    own "last route edge" special case ⇒ `AllowsContinuation=true` for every lane, unconditionally),
    and every multi-edge scenario (9a/9b/A3/B3's 15-reroute) is single-lane-per-edge (`RightNeighbor`
    always `-1`, guard returns before this code is ever reached).
    **Byte-identical argument:** `TryStrategicLaneChange`'s gate (`pool[LaneSeqIndex]`'s handle ≠
    `v.LaneHandle`) is false for every existing scenario because C2-i already proved `BestLaneOffset`
    is 0 on every lane of every route edge for every single-lane-per-edge scenario, which is exactly
    what makes `ResolveLaneSequence`'s new continuing-lane resolution a no-op there (the depart lane
    always `AllowsContinuation`) — so the pool is unchanged, `ExecuteMoves`' new convergence check is
    always satisfied (actual always equals target), and the new keep-right veto's own fast path
    (`route.Edges.Count <= 1`) or unreachability (`RightNeighbor < 0`) makes it inert too. Full suite:
    **94 passed, 0 failed** (was 93; +1 new `RungC2ParityTests`); `Rung9aParityTests`/
    `Rung9bParityTests`/`RungA3ParityTests`/`RungA2ParityTests`/`Rung8bParityTests`/
    `Rung8aParityTests`/`RungC2iBestLanesTests` all re-verified green/unchanged in the same run.
    **Empirically confirmed trigger step** (temporary instrumentation, since removed): at `t=16`
    (post-move `pos=191.79`) `usableDist=304.21`, `laDist=292.8` (already saturated,
    `myLookAheadSpeed=13.89` since `t=6`) → `304.21 ≥ 292.8` → no change. At the next step
    (`pos=205.68`, emitted as `t=17`) `usableDist=290.32 < 292.8` → fires; golden shows `lane=E1_1`
    at `t=17` with `pos=205.68` unchanged (pure lateral snap) — exact match, pins the change at
    `t=17` as specified. Gated ACCEPT by parity-reviewer (byte-identity of the whole existing golden
    set proven, not merely within-tolerance, via a topology sweep: every existing scenario is either
    single-edge-route OR single-lane-per-edge, so all four modified mechanisms are provably inert).
    **FOLLOW-UP (non-blocking, from the C2-ii review):** the keep-right `LaneContinuesRoute` veto
    ports only the OBSERVABLE effect of `MSLCM_LC2013.cpp:1398-1410`'s STAY guard, not the full
    early-return-before-`myKeepRightProbability`-decrement. Latent gap (no committed golden exercises
    it): a FUTURE multi-edge route where a vehicle sits on a lane whose right neighbor does NOT
    continue the route would accumulate `KeepRightProbability` faster than SUMO (no early return) and
    could fire a keep-right change one-or-more steps early onto a lane that DOES continue. When the
    first such scenario lands (with its own golden), port the full early-return semantics (return
    before the accumulator decrement) instead of the commit-gate veto, and re-anchor 07/12
    byte-identical.
  - **C2-iii. DONE (parity-track, exact @1e-3). Multi-hop lane-to-lane continuity (route-wide
    best-lanes backward pass).** Anchor `scenarios/36-multihop-lanes`
    (`RungC2iiiMultiHopLanesParityTests`, exact). Ported SUMO's backward pass
    (`MSVehicle::updateBestLanes`, MSVehicle.cpp:6003-6063) into `NetworkModel.ComputeBestLanes` (new
    `BackwardPassEdge`): builds each route edge's LaneQ from the route END back, accumulating a
    route-wide continuation `Length` and steering `BestLaneOffset` toward a lane that stays connected
    to the route end. `ResolveLaneSequence` now (a) redirects the pool onto the best-continuing lane
    on nonzero `BestLaneOffset` (not `!AllowsContinuation` -- SUMO keeps a dead-ending lane's
    `allowsContinuation` true when its immediate downstream has any length, MSVehicle.cpp:6046, yet
    still sets the offset), and (b) threads multi-connection hops through the best downstream lane
    (betterContinuation). v0 departs the dead-ending E0_0, gets `bestLaneOffset +1`, strategic-changes
    (the existing C2-ii `TryStrategicLaneChange`) to E0_1 at t=8, then E1_1 (t=15), E2_0 (t=29) --
    exact. INERT: a 2-edge route's backward pass reduces to the C2-i single-hop result and
    single-connection hops are byte-identical, so scenario 18 (behavioral) + all committed scenarios +
    the `RungD1BenchmarkDeterminismTests` hash stay green (128). The C2-i UNIT test's continuing-lane
    `Length` assertion was updated 496 -> 992 (now route-wide, correct). SIMPLIFICATIONS (documented
    in `BackwardPassEdge`, none exercised): no bidi/`nextLinkPriority`/vClass-change/elecHybrid
    arms, no disconnected-route (`bestConnectedLength<=0`) arm. Unblocks DEPARTURE-edge /
    multi-connection multi-hop; **intra-edge mid-route lane change (C2-v below) is still required
    before a general `-L 2` city runs** -- C2-iii redirects the pool only at `routeEdges[0]`, so a
    vehicle forced onto the wrong lane of a MIDDLE edge still throws (verified on a plain
    `netgenerate -L 2` grid). Parity-reviewer ACCEPT.
  - **C4-viii. DONE (parity-track). willPass pre-pass for multi-lane SATURATION -- the last gate for
    multi-lane AT SCALE.** After C4-vii-c (route->lane fix) moderate-density -L2 runs clean, but a
    SATURATED -L2 city still gridlocked (`NEED-multilane-density-willpass.md`; verified `main@f899f4e`:
    a 6x6 -L2 tls.guess grid at ~126 concurrent left ~46 vehicles PERMANENTLY stuck at junction stop
    lines while SUMO ran the identical net at 0 teleports and fully drained). This is the willPass
    ordering bug originally scoped for C4-vii-c but correctly deferred there (moderate density was a
    different, smaller bug). **MECHANISM:** the crossing-yield decision read a foe's raw START-OF-STEP
    speed; a root blocker yields to a foe that is close AND moving (speed > 0) but is BRAKING TO A STOP
    this step (it is itself yielding), so its planned vNext ~ 0 and SUMO's willPass for it is false --
    SUMO does not yield to it. Density is the trigger (moderate is clean), the tell for a
    start-of-step-speed ordering bug, not a static RoW error. **FIX (a new phase in the RoW core):**
    `Engine.ComputeWillPass` -- a willPass PRE-PASS run on the frozen start-of-step snapshot BEFORE
    PlanMovements (mirroring SUMO's MSLink::setApproaching populating myApproaching before opened() is
    consulted) -- computes each vehicle's planned vNext-at-link (via ComputeMoveIntent(prePass:true),
    side-effect-free: no LastActionTime/LevelOfService/RngState writes) WITHOUT the foe-willPass
    refinement (one level of approximation, the setApproaching-before-opened() ordering) and caches
    `VehicleRuntime.WillPass = vNext > NUMERICAL_EPS_SPEED`. JunctionYieldConstraint's crossing arm then
    blocks ego only on a foe whose WillPass is true (MSLink::blockedByFoe's `!avi.willPass`
    short-circuit). The load-bearing term is the PLANNED vNext, not start-of-step speed -- a braking foe
    has speed > 0 this step, so the earlier stopped-foe (`speed<=eps`) proxy misses it. **RESULT:** the
    dense repro drops ~46 -> ~2 stuck (== SUMO's ~0; the ~2 residual are symmetric mutual-yield corners,
    the arrival-time-RoW class = scenario-44 bug C, out of willPass scope). INERT: full suite stays
    **166 + 1 skip** green, moderate -L2 stays clean, `Sim.Bench` hash unchanged (`909605E965BFFE59`) --
    the pre-pass is a no-op wherever no foe is braking-to-stop at a crossing (every committed scenario).
    **ANCHOR:** `scenarios/_diag/willpass-saturation` (`WillPassSaturationDiagTests`) -- a small 3x3
    -L2 TLS grid (412 fixed-route trips) tuned to the saturation point; WITHOUT the pre-pass ~50
    vehicles stay permanently stuck, WITH it the grid flows (0 stuck, drains by t=605 == SUMO). Gauged
    by stuck-count/arrival (a gridlocked state is not a per-step FCD golden; exact @1e-3 is unreachable
    for a saturated -L2 grid where keep-right/lane-change/junction ordering diverge -- the established
    gridlock-anchor convention). Parity-reviewer ACCEPT.
    **PERF -- optimized (proximity gate landed).** The pre-pass runs a second ComputeMoveIntent per
    active vehicle, but WillPass is READ only by the crossing arm and only for a foe WITHIN reservation
    distance of a conflict lane (`foeNotApproaching` already makes a farther foe non-blocking). So
    `Engine.WillPassRelevant` skips the full compute for any vehicle beyond its own reservation distance
    from its NEXT internal lane (WillPass=true, the safe never-read value), and ComputeWillPass skips the
    WHOLE phase on a junction-free net (LinkByInternalLane empty -- the Sim.Bench highway workload).
    Correct because a foe's distance to any conflict lane >= its distance to its next internal lane, so a
    skipped vehicle's WillPass can never be read (byte-identical: Sim.Bench hash unchanged, suite green,
    dense grid still ~2 stuck). Sim.Bench throughput is back to baseline within noise (vs the true
    no-pre-pass baseline ~1667 single / ~1590 parallel steps/s; the pre-pass overhead was ~12%/32% before
    the gate, ~4%/negligible after). The city case (where willPass matters) still pays for the
    near-junction vehicles it needs.
  - **C4-vii. TODO (parity-track, exact @1e-3). Multi-lane junction passage -- vehicles deadlock at
    the stop line. THE TRUE GATE for multi-lane at scale** (C2-vi unblocked -L2 INSERTION; this
    unblocks -L2 FLOW). Briefing transcribed from `NEED-multilane-junction-passage.md` (on main,
    commit 618893c, from the viz/benchmark track). C2-vi fixed multi-lane route->lane resolution, so a
    general `-L2` city now runs to completion -- but ~60% of vehicles get PERMANENTLY STUCK at
    junction stop lines. **Reproduced vs SUMO on the identical net+demand:** `netgenerate --grid
    --grid.number=6 --grid.length=250 -L 2 --tls.guess --seed 7` + `randomTrips -e 300 -p 4
    --fringe-factor 10 --min-distance 500 --seed 7` + `duarouter --named-routes --ignore-errors` ->
    engine 27 arrived / 47 stuck (all braked to a junction stop line, speed 0, never proceed); SUMO 75
    inserted / 0 running / 0 teleports (all arrive, ordinary free-to-moderate traffic).
    **DISCRIMINATED (three checks, all in the NEED):** (1) NOT TLS-specific -- `--tls.set ""` (all
    priority junctions) gives the same 47 stuck. (2) NOT a lane-change/wrong-lane failure -- a stuck
    car (veh 0) is on `E3D3` lane 0 for a STRAIGHT move to `D3C3`, and lane 0 DOES connect onward
    (`fromLane=0 dir=s`); it is on the right lane, just never granted passage. (3) Single-lane
    junctions all work (9b/C3/C4/rung10 + the `-L1` benchmark run 0 stuck). The only new variable is
    2 lanes per edge. **DIAGNOSIS (confirm with instrumentation):** the junction RoW/passage machinery
    (`Engine.JunctionYieldConstraint` / `FindFoeVehicle` / the `MSLink` conflict geometry in
    `NetworkModel`/`Engine`, the 9b + C4 family) was built + parity-tested EXCLUSIVELY on SINGLE-LANE
    junctions. A multi-lane junction has more links per approach (each lane's connection is its own
    link with its own `linkIndex` + internal lane), and the two through-lanes' internal lanes can be
    flagged as mutual foes/conflicts in the `<request>` matrix. Most likely: a vehicle on a valid
    connecting lane yields FOREVER to a foe that never clears -- e.g. it treats the PARALLEL
    through-lane's internal lane (or a companion link at the same junction) as a blocking foe, or the
    multi-lane conflict/response bitstring is indexed wrong so a non-conflicting movement reads as
    conflicting. Even a straight-through movement (which should have priority) is blocked -> points at
    foe/conflict RESOLUTION, not a legitimate yield. (Distinct from C4-vi's far-routed false-positive.)
    **First probe (smallest repro):** a single 2-lane priority (and/or TLS) crossroads, one
    straight-through vehicle, NO real conflicting traffic -- confirm it still brakes to the stop line
    and never proceeds, then read WHICH foe/link `JunctionYieldConstraint` yields to and why (the
    `<request>` response/foes bits at the multi-lane link indices vs SUMO's `MSLink`).
    **DIAGNOSIS CONFIRMED (this session, instrumented the priority `--tls.set ""` repro):** the
    deadlock is a GRIDLOCK CYCLE, not a single false foe. veh 0 (stuck on E3D3_0 approaching D3, link
    6/7) yields to foe 11 which is STOPPED (speed 0) on D4D3_0 (link 2; the response matrix is
    asymmetric -- 6/7 yields to 2, 2 does NOT yield back, so no direct 2-cycle). Foe 11 is stopped
    because IT yields to links 8/18, whose vehicles yield onward -> a rotational cycle around the
    junction that never unwinds. **SUMO does NOT gridlock here (runs free-flow) because `MSLink::opened`
    /`blockedByFoe` gates the yield on the foe's `willPass` (MSLink.cpp: `if (!avi.willPass) return
    false`) + arrival/leave-time overlap: a foe that is ITSELF yielding (stopped, not requesting
    passage this step) has `willPass=false` and does NOT block ego, so the cycle unwinds.** The engine's
    crossing approaching-foe arm (`JunctionYieldConstraint`, ~Engine.cs:1698) yields to ANY approaching
    foe within the C4-vi reservation distance REGARDLESS of whether that foe will actually pass -- the
    C5-willPass gate (`FoeKeepClearBlocked`) only catches the keepClear-downstream-jam case, not a foe
    that is yielding to ITS OWN priority foe. **FIX SHAPE (structural, faithful):** port SUMO's
    `willPass` signal. SUMO computes it in a SEPARATE phase (`setApproaching` during planMove registers
    each vehicle's link request BEFORE the `opened()` yield checks read foes' `willPass`); the engine
    does yield decisions in one PlanMovements pass reading start-of-step state, so this needs a
    willPass PRE-PASS: compute, from start-of-step state, whether each vehicle intends to cross its
    upcoming junction link this step (not stop-line-gated), then the crossing yield only blocks on a
    foe whose `willPass` is true. HIGH REGRESSION RISK (touches the verified 9b/C4 RoW core underpinning
    ~30 scenarios) -- build a minimal deterministic multi-vehicle crossing anchor to pin the exact
    yield before/after, and gate hard.
    **Done-condition:** (1) the `-L2` repro flows (stuck-count comparable to SUMO's ~0). (2) NEW anchor:
    a minimal 2-lane junction where a through vehicle is granted passage against correctly-resolved
    multi-lane foes (distinct from the single-lane 9b/C4 anchors); sigma=0, SUMO golden `--precision 6`,
    exact @1e-3. (3) INERT: all committed single-lane junction scenarios (9b/C3/C4/rung10/36/37) +
    everything stay green (158); `Sim.Bench` hash unchanged. (4) parity-reviewer ACCEPT, faithful to
    `MSLink`/`MSRightOfWayJunction`/`MSVehicle`. Build the anchor + golden FIRST, then instrument + fix.
    **SESSION UPDATE (anchor built; scope RE-DIAGNOSED as a CLUSTER, not a one-line willPass gate).**
    Committed the deterministic, network-free minimal anchor `scenarios/44-multilane-junction-turn`
    (single symmetric 2-lane priority crossroads; 4 vehicles, one per approach, ALL turning left,
    departing together from rest; SUMO flows all four staggered t=17..20 and every one clears by t=38,
    0 teleports) + its `--precision 6` golden + `provenance.txt` (full diagnosis) +
    `RungC4viiMultilaneJunctionParityTests` gated `[Fact(Skip=...)]` until this lands (suite 159 green +
    1 skipped). Instrumenting this anchor + the -L2 grid this session showed **C4-vii is at least THREE
    entangled multi-lane bugs, and the willPass gate alone does NOT fix it** -- each needs its own
    isolated sub-anchor. Recommend DECOMPOSING into:
      - **C4-vii-a (multi-internal-lane "cont" path). PART 1 DONE; parts 2+ open.** SUMO's left turn
        traverses TWO internal lanes (e.g. `NC->CE` = `:C_3_0` then the internal-junction lane
        `:C_16_0`, `<request>` index 3 `cont="1"`); the engine collapsed it to `NC_1 -> CE_1`, never
        modeling `:C_16_0` (the lane in junction C's `<request>`/IntLanes for this link), so
        JunctionYieldConstraint could not even find ego's link for a cont turn.
        **PART 1 DONE (the internal via-chain SEQUENCE):** `NetworkModel.ResolveSequenceCore` now follows
        the full via-chain (`:C_3_0` then `:C_16_0`) -- SUMO's getViaLaneOrLane / MSLane internal-
        following chain -- so the pool includes every internal lane crossed and the engine OCCUPIES
        `:C_16_0`. Inert for single-internal-lane junctions (every committed green scenario + the -L2
        diag grids; suite byte-identical, Sim.Bench hash unchanged `909605E965BFFE59`). ANCHOR
        `scenarios/_diag/cont-turn-sequence` (`ContTurnSequenceDiagTests`) -- a lone `NC->CE` turn;
        without the chain the engine visits only `{NC_1, CE_1}` (collapse), with it `:C_16_0`. This is
        the SEQUENCE fix the decomposition asked to anchor first (not arrival).
        **PART 2 (the cont-turn SPEED) -- FIXED, exact @1e-3 (parity-reviewer ACCEPT).** Turned out to
        be a ONE-POINT fix, not the predicted 2a-2c internal-junction port. The engine already models
        the cont link on its junction internal lane `:C_16_0`; the minor-link cautious-approach arm just
        measured `seen` from the wrong lane. FIX (Engine.cs): when the cautious arm's `approachLane` is
        itself an internal (':'-edge) lane (the cont-turn signature), measure `seen` to the junction-link
        internal lane by walking the pool through the intermediate internal lane; the ordinary path keeps
        `approachLane.Length - pos` verbatim. The existing machinery then reproduces SUMO's dip
        (13.89 -> 10.036 -> 5.536 on `:C_3_0` -> 8.136 -> 9.260) EXACTLY. Anchor
        `scenarios/_diag/cont-turn-sequence` gained a `--precision 6` golden + tolerance +
        `ContTurn_SpeedMatchesGoldenExact` (max-abs err 3.4e-07). INERT on every ordinary minor-link
        turn (suite byte-identical, Sim.Bench hash unchanged). Full scenario 44 additionally needs bug B
        (spurious final-edge lane change under conflict) and the
        symmetric-4-way arrival-time RoW (bug C -- a distinct port, NOT willPass: with all four
        arriving together, priority/willPass alone can't break the tie; SUMO uses arrival-time). Anchor
        `44` stays skip-gated until 2a-2c + B + C land.
        **BUG C ISOLATED + DIAGNOSED (this session, NOT fixed).** Committed the cleanest isolated
        repro `scenarios/_diag/sym-rbl-straight/` (one single-lane `right_before_left` cross, four
        STRAIGHT vehicles arriving together; `--precision 6` golden + skip-gated
        `RungSymRblStraightDiagTests`). The four straights form a directed right-before-left 4-CYCLE
        (`NC→CS`→`WC→CE`→`SC→CN`→`EC→CW`→back); with no priority winner the C4-viii willPass pre-pass
        sets `WillPass=false` for all four, so the crossing gate's `foeYieldsThisStep=!foe.WillPass`
        lets ALL four enter the junction simultaneously → mutual mid-junction gridlock. SUMO breaks it
        with an EXPLICIT deadlock detector (verified: `MSVehicle::planMoveInternal` MSVehicle.cpp:
        2818-2839): a `LINKSTATE_EQUAL` link with `waitingTime>0` walks the `getFirstApproachingFoe`
        blocker chain, and if it wraps back to ego it aborts the request RANDOMLY (`RandHelper::rand <
        0.25` straight / `0.75` turn). **FIXED (parity-reviewer ACCEPT):
        `Engine.ResolveRightBeforeLeftCycles` — a DETERMINISTIC, order-independent equivalent** of the
        RNG abort (our RNG stream can't match SUMO's, so exact @1e-3 is unreachable; stuck-count parity
        is the bar): detect the directed response cycle among approaching vehicles and select a maximal
        non-conflicting set greedily by ascending link index to pass, the rest yield. Anchor
        `RungSymRblStraightDiagTests` un-skipped, 0 stuck == SUMO. INERT wherever no cycle exists (suite
        byte-identical, Sim.Bench hash unchanged). Residual on DENSE grids (strictly improved 195→0
        stuck / 1794→112 overlaps, still not SUMO's 100/0) is the pre-existing missing keepClear
        box-blocking (#3), not this change. Details in `C4-VII-REMAINING.md` "#2".
      - **C4-vii-b. DONE (parity-track, exact @1e-3). Keep-right over-accumulation + final-edge arrival
        strand.** Root-caused to TWO entangled bugs: (1) the narrow keep-right port
        (`ApplyKeepRightDecision`) accumulated the keep-right probability on a REQUIRED lane (it vetoed
        the COMMIT but not the accumulation), so a vehicle held on a turn/route lane over-accumulated
        and then fired a SPURIOUS keep-right on a LATER multi-lane edge; (2) that change moved the
        vehicle off its resolved arrival lane, and the final-edge advance required the exact pool lane
        to arrive -- so it FROZE at the lane end (speed 0) forever. Confirmed against SUMO via TraCI: a
        held vehicle's `keepRightProbability` stays exactly 0 on its required lane (SUMO's stayOnBest,
        `MSLCM_LC2013.cpp:1421`, suppresses accumulation when the right lane leaves the route within
        `TURN_LANE_DIST=200`). **FIX:** (a) port stayOnBest to gate keep-right ACCUMULATION (new
        `Engine.KeepRightStrategicStay`, memoized per-lane on `VehicleRuntime` so `ComputeBestLanes`
        stays off the per-step hot path); the `neighDist < 200` distance gate is load-bearing -- on a
        LONG off-lane SUMO DOES keep-right-and-back (verified 8 m/s / 400 m off-ramp: SUMO 3 changes,
        fixed engine reproduces the excursion) so an unconditional veto would be UNfaithful. (b) make
        last-edge arrival lane-AGNOSTIC (`Engine.ExecuteMoves`: the "last route edge -> arrive" check
        now precedes the pool-lane-convergence guard) -- SUMO's arrival is position-based on the final
        edge. **ANCHOR:** `scenarios/45-multilane-keepright-arrival` (`RungC4viibKeepRightArrivalParity`
        Tests, Run 30), STRAIGHT-through (off-ramp forces the through-vehicle onto `AB_1`; SUMO stays
        `AB_1`->`BC_1`, 0 lane changes, arrives `BC_1`); pre-fix engine strands v0 at `BC_0` pos 60
        speed 0 (non-vacuous: baseline fails at step 21, lane err + presence mismatch). INERT: full
        suite 161 green + 1 skip; `Sim.Bench` hash unchanged (`7291978050025285112` both ways -- the
        highway's right lanes all continue the route so stayOnBest never fires there). SIMPLIFICATION:
        only VARIANT_21 of SUMO's strategic-STAY rules is ported (the change-back-in-time rules and the
        `getLinkCont()!=0` "leads somewhere" guard are not modelled -- the committed anchors sit
        unambiguously inside/outside the 200 m band). Note: this does NOT resolve scenarios/44 (still
        skip-gated for bugs a + c).
      - **C4-vii-c. DONE (parity-track). The -L2 grid FLOW blocker -- root cause was a route->lane
        OVER-CONSTRAINT, NOT willPass.** Instrumenting the committed diag grid overturned the willPass
        framing: **29 of 38 stuck vehicles were clamped by the C2-ii boundary-convergence guard**
        (`Engine.cs` ExecuteMoves, `v.LaneHandle != _laneSeqPool[slot]`) -- stranded at a lane end at
        speed 0 because their actual lane != the pool's resolved exit lane, EVEN THOUGH that lane
        connects onward. The pool pins one exit lane (chasing a downstream `bestLaneOffset` hint), but
        208 of the grid's 224 two-lane edges have BOTH lanes connecting to a common downstream edge, so
        a vehicle on the other equally-valid connecting lane froze forever. The remaining 9 stuck all
        transitively yielded to those clamp victims, so the willPass gate had ZERO independent effect
        (gate on/off identical on the grid; a ~20-config dense-`-L2` sweep never showed willPass helping).
        The original "foes moving then braking to a stop this step" instrumentation was watching a moving
        vehicle hit the convergence clamp (execute-time hard stop), misread as a plan-time willPass yield.
        **FIX:** `Engine.TryReResolveFromActualLane` + `NetworkModel.ResolveSequenceCore`'s
        `forceFirstExitToArrival` -- a vehicle reaching a lane end on a non-pool lane that still connects
        to the next route edge re-resolves the remaining route from that lane (pinning the edge exit to
        it) and proceeds, matching `MSVehicle::updateBestLanes` (bestLaneOffset is a HINT it lane-changes
        toward opportunistically, not a hard gate). Only reached from the boundary branch that used to
        clamp -> every committed golden byte-identical; diag grid ~38 -> **0** stuck (== SUMO), suite
        **162 + 1 skip** green, `Sim.Bench` hash unchanged (`909605E965BFFE59`). **ANCHOR:**
        `scenarios/46-convergence-lane` (`RungC4viiConvergenceLaneParityTests`, single vehicle keep-rights
        onto a connecting non-pool lane; pre-fix strands it at the lane end pos 300 speed 0, post-fix
        flows it -- pos/speed EXACT vs SUMO, `lane` excluded because the keep-right AB_1->AB_0 timing is a
        separate C4-vii-b fidelity gap). The willPass port was implemented then REVERTED (faithful but
        provably inert + un-anchorable + doubled plan-phase cost). **Follow-up EXPOSED then FIXED:** the
        fix lets vehicles that used to clamp proceed onto junction interiors, making a latent crash
        reachable -- a vehicle transiting a MULTI-LANE junction's internal lane with a pending strategic
        LC hit `ComputeBestLanes(route, ':A0_2')` (an internal edge is never on an edge-only route). FIX:
        skip all lane-change decisions (keep-right/strategic/speed-gain) on internal lanes -- SUMO's
        `MSLaneChanger` runs only on normal edges (mirrors `eac0a5b`, hoisted to cover the strategic
        path). REGRESSION ANCHOR `scenarios/_diag/multilane-internal-lc-crash` (3x3 -L2, 10 trips;
        `MultilaneInternalLcCrashDiagTests` -- crashes without the guard, flows 0 stuck == SUMO with it);
        verified across a ~10-config dense-`-L2` sweep (up to 800 veh) that all now run without crashing.
        **No residual gridlock (checked):** dense TLS grids show MORE engine vehicles queued than SUMO at
        a mid-run cutoff (36 vs 19 at t=400 on tls.guess 6x6/150; 137 vs 66 on a 375-veh TLS grid) but
        this is TRANSIENT -- run longer and both fully drain (150 veh: 0 stuck by t=800; 375 veh: 0 by
        t=1000). The engine just drains slower on dense TLS density (an exact-throughput/-timing gap on
        the broad -L2 TLS FCD-parity frontier, its own rung), NOT a gridlock defect; all queuing is on
        normal lanes (not mid-junction/keepClear). Priority grids need no extra time.
    C4-vii-a (cont-internal-lane path) is the only remaining C4-vii RoW strand; anchor `44` stays
    skip-gated until it lands.
  - **C4-vi. DONE (parity-track, exact @1e-3). Priority-junction far-routed-foe false positive fixed.**
    The crossing approaching-foe arm of `JunctionYieldConstraint` (`Engine.cs`, the
    `foeInternalSeqIndex > foe.LaneSeqIndex` branch) now gates the yield by the foe's
    approach-reservation range `SPEED2DIST(maxV) + brakeGap(maxV)` (`MSVehicle::setApproaching`'s
    registration window, MSVehicle.cpp:2238) -- the SAME gate the sameTarget-merge PHASE-0 arm
    already applied, ported to the crossing arm it was missing from. A foe farther than that from the
    conflict internal lane (`SeenToInternalLaneEntry`) has not reserved the link, so `opened()` sees
    no approaching foe and ego is not blocked. **TEST `RungC4viFarRoutedFoeParityTests`**
    (`scenarios/40-farrouted-foe`, Run 46) passes exact @1e-3; suite 138 -> 140 (with C6-ii). ANCHOR:
    a priority junction J where the minor ego (SJ->JN) responds to the major link WJ->JE, and the lone
    major vehicle `foeFar` departs 496 m up WJ. Golden: ego does the cautious minor-approach slowdown
    (13.89->8.64 by t=9) then PROCEEDS, crossing at t=10-11; foeFar reaches J only at t=38. The bug
    reproduced exactly on this anchor (pre-fix engine stops ego at the SJ stop line pos=92.699 and
    holds it stuck through the foe's whole approach); NON-VACUOUS: disabling the gate fails the test at
    FirstDivergenceStep=10 (pos err 85.7 m, ego stuck). INERT for genuinely-close foes: scenario 11 /
    19 / 32 (close/circulating foes) + right-before-left + every committed scenario stay green (140);
    D1 determinism hash unchanged. Single-foe-per-link scope unchanged (FindFoeVehicle still returns
    the first route-matching foe). Parity-reviewer ACCEPT (byte-identical to the already-accepted
    merge-arm gate; MSVehicle.cpp:2238; forced-off fails at step 10; 140 green). UNBLOCKS the
    scaled-city benchmark scaling ladder (single- AND multi-lane). *(original briefing retained below
    for context.)*
    Priority-junction far-routed-foe false positive --
    `FindFoeVehicle` matches an approaching foe ANYWHERE on its route, not on its actual approach.
    A VERIFIED `Sim.Core` correctness bug (root-caused against `main`'s engine, reproduced on the
    scaled-city benchmark, NOT yet patched -- benchmark work must not touch `Sim.Core`). **This
    gates ALL benchmark scaling, single- AND multi-lane** (whereas C2-v below only affects `-L 2`),
    so sequence it FIRST. It is also a real priority-junction realism gap independent of the
    benchmark. Briefing transcribed from `NEED-priorityjunction-farrouted-foe-falsepositive.md` on
    sibling branch `claude/spec-docs-review-qgwatc` (viz/benchmark track -- do NOT merge/cherry-pick
    that branch; only this one file's content is ported here).
    **The bug (file:line on the current engine):** `Engine.FindFoeVehicle` (`src/Sim.Core/Engine.cs:
    ~2279`) returns ANY active vehicle whose FULL route lane-sequence contains the foe internal lane
    -- `IndexOfLaneHandle` (`Engine.cs:~2302`) scans the entire `[LaneSeqStart, LaneSeqStart +
    LaneSeqLen)` precomputed route (insertion->arrival), with no proximity bound.
    `JunctionYieldConstraint` (`Engine.cs:~1550-1583`) then treats that foe as "approaching" whenever
    `foeInternalSeqIndex > foe.LaneSeqIndex` (the foe internal lane is anywhere AHEAD in the foe's
    route) -- NO distance / hop-count / arrival-time filter. So the ego yields to a vehicle that may
    be many junctions and kilometres away and won't reach the conflict for minutes. The engine's own
    `FindFoeVehicle` comment flags it as a deliberate 9b single-foe-per-link scoping shortcut.
    **Why invisible on the ladder:** on `11-priority-junction` / `19-onramp-merge` (1-2 junctions,
    short routes) "some vehicle's route includes this lane" and "some vehicle is actually near this
    junction soon" are the SAME fact; the false positive only fires once routes are long AND traffic
    is dense enough that essentially every approach lane is on SOME distant vehicle's route.
    **Reproduced (not guessed):** `scenarios/_bench/city-300` (24x24 grid, 576 junctions, `-L 1`, same
    net+demand): engine 46 arrived vs SUMO 238, 404 vehicles permanently stuck at stop lines, while
    SUMO runs the identical input at `meanSpeedRelative=0.98` / `halting=1` (free flow). Traced stuck
    vehicle `281` sits 0.1 m from a stop line for 437+ continuous seconds (its `M14M15->M15L15` left
    turn yields to all four `L15M15` outgoing links; ANY far-off vehicle routed through `L15M15_0`
    holds it). Repro command + the full trace are in the NEED file (city-300 not committed here; it
    is regenerable via `netgenerate --grid --grid.number=24 --grid.length=500 -L 1 --tls.guess
    --seed 42` + `randomTrips.py --fringe-factor 5 --seed 42`).
    **Port target:** SUMO never derives "approaching" from route membership. `MSLink::setApproaching`
    populates `myApproaching` (the map `MSLink::opened()` reads) only once a vehicle is within that
    link's own lookahead/braking-distance window of the link; `removeApproaching`/entry deregisters
    it. Port that registration, OR (lighter, same effect) bound the `FindFoeVehicle` match to foes
    within a small lane-hop / distance window of the foe internal lane (e.g. cap
    `foeInternalSeqIndex - other.LaneSeqIndex`, matching setApproaching's registration point), so a
    foe still far up its route is NOT counted.
    **Done-condition (standard isolate -> golden -> reverse-engineer -> port -> gate):** (1) NEW
    anchor scenario: a foe whose route INCLUDES the conflict lane but is still far away must NOT be
    yielded to (build + commit its 1e-3 golden). (2) INERT when the foe is genuinely close:
    `11-priority-junction` / `19-onramp-merge` / every committed scenario stays green at 137;
    `Sim.Bench` determinism hash unchanged. (3) parity-reviewer ACCEPT, faithful to `MSLink`. Build
    the anchor + golden FIRST (pin exactly SUMO's registration distance), then port.
  - **C2-v. DONE (parity-track, exact @1e-3). Intra-edge mid-route lane change to reach an onward
    connection.** Port: the routing lane-sequence is now resolved as (Exit, Arrival) pairs per slot
    (`NetworkModel.ResolveSequenceCore` + the new `_laneSeqArrival` pool parallel to `_laneSeqPool`).
    `_laneSeqPool[k]` (Exit) is the lane the vehicle must reach to continue (strategic-LC target +
    onward-connection source); `_laneSeqArrival[k]` is the lane it physically occupies on ENTERING
    that slot's edge. The crossing lands the vehicle on the ARRIVAL lane; the existing C2-ii
    `TryStrategicLaneChange` (target = pool[slot]) then converges arrival->exit, and the crossing
    convergence guard holds the next boundary until it does -- the multi-lane generalization of the
    departure-edge redirect, now unified: exit = arrivalLane + `bestLaneOffset` CLAMPED to the edge's
    lane range (an offset that points off a 1-lane edge is a downstream change the vehicle makes on a
    later edge, not here -- this is what fixed the old `lane -1` crash without a special case).
    **TEST `RungC2vIntraEdgeLaneChangeParityTests`** (`scenarios/37-intraedge-lanechange`, Run 44)
    passes exact @1e-3: E0_0 (t<=14) -> arrive E1_1 (t=15, shown for 3 steps) -> strategic-LC to E1_0
    (t=18) -> :N2_0_0 (t=29) -> E2_0 (t=30), matching SUMO. Suite 140 -> 141. INERT: for every route
    with no intra-edge change Arrival == Exit at every slot, so the Exit pool is byte-identical and
    the crossing (reading `_laneSeqArrival`) is byte-identical -- scenario 18/36 + all committed +
    the D1 determinism hash stay green (141). NON-VACUOUS: snapping the crossing to the Exit lane
    fails at FirstDivergenceStep=15 (vehicle jumps to E1_0 instead of showing E1_1). SIMPLIFICATION
    (documented, unexercised): a |offset| > 1 clamps/stays rather than moving one lane per edge.
    Parity-reviewer ACCEPT (byte-identical for no-intra-change routes -- D1 hash unchanged, 141 green;
    snap-to-exit fails at step 15; clamp faithful to SUMO's per-edge bounded bestLaneOffset).
    **UNBLOCKS a general `netgenerate -L 2` city.**
    *(original briefing retained below for context.)*
    Intra-edge mid-route lane change to reach an onward
    connection. The remaining half of multi-hop. **BLOCKS a general `netgenerate -L 2` city** (the
    benchmark generator `scripts/gen-benchmark.sh` stays pinned to `-L 1` until this lands); the
    hand-built C2-iii anchor (scenario 36) only exercises the DEPARTURE edge.
    **The gap:** C2-iii's `ResolveLaneSequence` redirects the pool onto the best-continuing lane
    ONLY at the departure edge (the `if (routeEdges.Count > 1)` block redirects `currentLaneIndex`
    once, for `routeEdges[0]`). At every INTERMEDIATE edge it still follows `connection.ToLane`
    rigidly and hard-throws (`NetworkModel.cs:~296`) when the arrival lane has no onward connection
    to the next route edge. Missing case: a vehicle ENTERS a middle edge on lane A (fixed by the
    incoming connection's toLane) but the only onward connection to its next route edge leaves from
    lane B != A, so it must lane-change A->B while traversing that edge, before the junction.
    **Repro (clean, no `--ignore-errors`):** `netgenerate --grid --grid.number=3 --grid.length=200
    -L 2 --tls.guess --seed 42`, route `B1A1 A1B1 B1B0 B0A0 A0A1`: enter A1B1 on lane 1 (conn
    `B1A1_1->A1B1_1`), but `A1B1->B1B0` only from lane 0 -> must intra-edge-change 1->0 on A1B1. SUMO
    inserts + completes all 40 such vehicles; the engine throws `No <connection> found from edge
    'A1B1' lane 1 to edge 'B1B0'` at `ResolveLaneSequence` (NetworkModel.cs:302) via
    `Engine.TryInsertOnLane` at INSERTION.
    **Port target:** same `MSVehicle::updateBestLanes`/`LaneQ` (MSVehicle.cpp:5744-6063) --
    `bestLaneOffset` is a PER-EDGE quantity along the whole route, and the arrival lane and exit lane
    on one edge legitimately differ; on each edge the vehicle occupies, a nonzero `bestLaneOffset`
    drives a `LCA_STRATEGIC` change (the existing C2-ii `TryStrategicLaneChange`) toward the
    connecting lane.
    **STRUCTURAL NOTE (from analysis, for whoever implements):** the departure edge handles
    arrival!=exit because the vehicle INSERTS on the depart lane (actual `LaneHandle`) while the pool
    records the EXIT lane, and strategic LC converges actual->pool before the junction (the
    ExecuteMoves convergence guard at ~Engine.cs:2332). An intermediate edge has an INCOMING
    connection that lands the vehicle on the arrival lane, and the crossing sets
    `v.LaneHandle = _laneSeqPool[...]` (~Engine.cs:2358) -- so the pool's single-lane-per-edge slot
    cannot hold BOTH the arrival lane (for the FCD to show lane A first, matching SUMO) AND the exit
    lane (for the onward connection + strategic LC target). Landing directly on the exit lane would
    mismatch SUMO's `lane` attribute (SUMO shows the arrival lane for the steps before the change).
    So this likely needs the pool/`LaneSequence` to represent an intra-edge lateral transition
    (arrival lane -> exit lane on one edge), not just a longitudinal lane-per-edge chain -- build the
    anchor + golden FIRST to pin exactly when SUMO shows the A->B change, then decide the pool
    representation.
    **ANCHOR BUILT (committed, NON-TESTED): `scenarios/37-intraedge-lanechange`** -- E0(1 lane)->
    E1(2 lanes)->E2(1 lane); E0_0 connects only to E1_1 (forced arrival on the WRONG middle lane),
    E1->E2 only from E1_0, so v0 must intra-edge-change E1_1->E1_0 on E1. SUMO golden: E0_0 (t<=14),
    arrive E1_1 at t=15, STAY on E1_1 t=15-17, change to E1_0 at t=18, :N2_0_0 (t=29), E2_0 (t=30) --
    so the FCD shows the arrival lane E1_1 for 3 steps before the change (the port must reproduce
    that, not snap to the exit lane). The engine crashes at insertion (`InvalidOperationException`):
    the C2-iii backward pass propagates E1_1's offset -1 back to the 1-lane E0_0 and the departure
    redirect applies it -> lane -1, so the port ALSO has to (a) apply bestLaneOffset per-edge where
    actionable / clamp the departure redirect to the edge's lane range.
    **Done-condition (standard isolate -> golden -> reverse-engineer -> port -> gate):** (1) intra-edge
    redirection at EVERY hop (arrival lane with no onward connection -> move to the same-edge sibling
    that connects, record it as the edge's exit lane, strategic-LC across); no throw for any route
    SUMO routes. (2) the clean `-L 2` repro runs to completion in the engine. (3) anchor
    `scenarios/37-intraedge-lanechange` passes exact @1e-3 (golden already committed). (4) INERT: scenario 36 + 18 + all committed stay green; `Sim.Bench`
    highway-dense determinism hash `42F875C2662DB78E` unchanged; departure-edge + single-connection
    behavior byte-identical. (5) parity-reviewer ACCEPT, faithful to `MSVehicle.cpp`. Briefing
    transcribed from NEED-C2iii-followup-intraedge-lanechange.md (sibling viz/benchmark track).
    *(original briefing retained below for context)*
    The deferred second half of C2-i/ii. **BLOCKS the scaled-city
    benchmark's multi-lane rungs** (`-L 2+`, the 300/3k/15k concurrency levels in
    `BENCHMARK_SPEC.md`/`VIZ_BENCH_TASKS.md` Phase 2 -- bring-up is pinned to `-L 1` today precisely
    because of this gap, `scenarios/_bench/city-30/NOTES.md`); also a real realism gap (any
    multi-lane route with a forced turn a couple of junctions out is currently unroutable by the
    engine though SUMO handles it).
    **The gap (verified on `main@e30269a`):** the engine's route->lane resolution is
    SINGLE-look-ahead. `NetworkModel.ResolveLaneSequence(routeEdges, departLaneIndex)`
    (`src/Sim.Ingest/NetworkModel.cs:255-308`) picks the depart lane once via `ComputeBestLanes`
    (which only considers the transition to the IMMEDIATELY next route edge) and then walks the route
    HARD-REQUIRING an exact `ConnectionsByFromLaneTo[(fromEdge, currentLaneIndex, toEdge)]` at every
    hop (`NetworkModel.cs:288-294`) -- `currentLaneIndex` only advances by each hop's
    `connection.ToLane`, with NO lane-change planning across hops. If hop k's `ToLane` has no
    `<connection>` onward to edge k+2, insertion throws
    `InvalidDataException: No <connection> found from edge '...' lane N to edge '...'`
    (`NetworkModel.cs:292`, via `Engine.TryInsertOnLane` ~`Engine.cs:588` during
    `InsertDepartingVehicles`). `ComputeBestLanes` (`NetworkModel.cs:328-368`) documents this scope
    itself: its own comment (`:346-353`) says the BACKWARD PASS is DEFERRED.
    **Port target:** SUMO's backward pass in `MSVehicle::updateBestLanes`
    (`sumo/src/microsim/MSVehicle.cpp:6003-6063`; `LaneQ` at `MSVehicle.h:865-886`; the already-ported
    forward/per-edge `LaneQ` build is `:5896-5918`, last-route-edge special case `:5951-5989`,
    non-continuing-lane offset tie-break `:5970-5976`). The missing piece: for each CONTINUING lane,
    recurse to the best downstream lane, ACCUMULATE route-wide continuation `length`, and set
    `bestLaneOffset` so the vehicle is steered -- across multiple junctions -- toward a lane that
    stays connected to the END of its route. This makes `bestLaneOffset` a route-wide quantity, not a
    one-junction hint.
    **Repro:** `netgenerate --grid --grid.number=3 --grid.length=200 -L 2 --tls.guess --seed 42`
    + `randomTrips.py`/`duarouter --named-routes` -> the engine throws at `A1B1` lane 1 -> `B1B0`; a
    single-lane net (`-L 1`) never hits it (one unambiguous connection per direction).
    **Done-condition (standard isolate -> golden -> reverse-engineer -> port -> gate):**
    (1) `ComputeBestLanes`/`ResolveLaneSequence` thread a valid `<connection>` at EVERY hop of a
    multi-lane, multi-junction route (SUMO's backward recursion picks the best downstream lane); no
    `InvalidDataException` for any route SUMO itself routes. (2) Where the depart/upstream lane can't
    reach the route but a lateral neighbor can, the vehicle strategic-changes into the connecting lane
    -- REUSE the existing C2-ii `Engine.TryStrategicLaneChange`/`bestLaneOffset` path, do not
    hard-throw. (3) Anchor `scenarios/NN-multihop-lanes/`: a minimal 2-lane, >=2-junction net where
    the correct INSERTION lane is dictated by a connection TWO hops ahead (so the single-look-ahead
    choice is wrong), one `sigma=0` vehicle, forced turn; SUMO golden `--precision 6`; match
    lane/pos/speed @1e-3 EXACT. (4) INERT-when-absent: `scenarios/18-strategic-turnlane` and every
    committed scenario stay byte-for-byte within tolerance (`dotnet test` green; the `Sim.Bench`
    highway-dense determinism hash must NOT move) -- the single-junction fast path stays
    behavior-identical. (5) parity-reviewer ACCEPT; faithful to `MSVehicle.cpp` (not a curve-fit).
    Fits the C-track after the merge/junction rungs; portable in-session (deterministic net-graph
    algorithm, SUMO golden as oracle -- no runtime DEBUG trace needed, unlike the timing rungs).
  - **C2-vi. DONE (parity-track, exact @1e-3). Complete route->lane resolution for a general
    `netgenerate -L 2` city -- the LAST multi-lane routing blocker.** Root cause (found by
    instrumenting the real repro): `ResolveSequenceCore` computed the exit lane as
    `arrivalLane + bestLaneOffset` and applied it whenever the target lane EXISTED -- but
    `bestLaneOffset` is a per-edge DOWNSTREAM hint that can point at a sibling with NO connection to
    the route's immediate next edge (real case: route `C1B1 B1B2 B2A2`; `C1B1` lane 0 is the ONLY
    lane connecting to `B1B2` yet inherits offset +1 from `B1B2`'s downstream best lane, so the old
    code chose the non-connecting lane 1 and threw). FIX (`NetworkModel.ResolveSequenceCore`): the
    exit lane MUST connect to the route's next edge -- among the lanes that DO connect, pick the one
    nearest the `bestLaneOffset` target. This subsumes the old `targetExists` clamp (an off-edge or
    non-connecting target naturally falls back to the nearest connecting lane) and reconciles both
    prior cases: scenario 36 (E0_0 redirected TO a connecting sibling because its own path dead-ends)
    and C2-vi (E0_0 NOT redirected because the offset target doesn't connect). Verified: all 100
    routes of the actual `netgenerate --grid --grid.number=3 -L 2 --seed 42` + `randomTrips --seed 42`
    demand now resolve (was 8 throws). **TEST `RungC2viForcedTurnLaneParityTests`**
    (`scenarios/41-forced-turn-lane`, Run 44) passes exact @1e-3: E0_0 (t<=13), :J_0_0 (t=14), E1_1
    (t=15, the immediate post-junction strategic LC off the dead-end E1_0), :K_1_0 (t=29), E2_0
    (t=30) -- matching SUMO. Suite 151/153 -> 153. INERT: byte-identical for every route whose offset
    target already connects (single connecting lane, or target is a connecting lane) -- scenarios
    18/36/37 + all committed + the D1 hash stay green. NON-VACUOUS: reverting to the old `targetExists`
    logic throws `No <connection> found from edge 'E0' lane 1 to edge 'E1'` (the exact C1B1 bug).
    SIMPLIFICATION (documented): two equidistant connecting lanes tie-break to the lower index (a
    betterContinuation rank would be more faithful; no committed scenario has such a tie). Parity-
    reviewer ACCEPT (rigorous byte-identical case analysis: the connecting-lane-nearest-target rule
    collapses to the old exit lane for every route the old code didn't throw on; faithful to
    MSVehicle.cpp:updateBestLanes). **RE-GATE FLAG (reviewer, not reached by any committed scenario):**
    if a future `-L2` golden lands with `offset != 0 && !targetExists` on a multi-lane edge where a
    NON-arrival lane connects strictly nearer the clamped target, the new rule picks that lane where
    the old returned `arrivalIndex` -- re-gate that specific branch then. **UNBLOCKS the scaled-city
    benchmark's multi-lane rungs.**
    *(original briefing retained below for context.)*
    Complete route->lane resolution for a general
    `netgenerate -L 2` city -- the LAST multi-lane routing blocker. C2-iii (multi-hop best-lanes)
    and C2-v (intra-edge lane change) closed most of the multi-lane route->lane gap and their anchors
    (scenarios 36, 37) pass, but a GENERAL `-L 2` grid still throws at insertion. Verified on
    `main@7718953`. Briefing transcribed from `NEED-C2iii-followup-intraedge-lanechange.md` (refreshed
    post-C2-v by the viz/benchmark track, now merged to main). This is the last blocker for the
    scaled-city benchmark's MULTI-LANE rungs (single-lane `-L 1` already runs); also a real realism
    gap (a general multi-lane route with a forced turn is currently unroutable though SUMO handles it).
    **The throw (reproduced on current main):** `netgenerate --grid --grid.number=3 --grid.length=200
    -L 2 --tls.guess --seed 42` + `randomTrips.py --fringe-factor 5 --seed 42` -> route
    `C1B1 B1B2 B2C2` -> `InvalidDataException: No <connection> found from edge 'C1B1' lane 1 to edge
    'B1B2'` at `NetworkModel.ResolveSequenceCore` (`NetworkModel.cs:~350`, via
    `ResolveLaneSequenceHandlesWithArrival` -> `Engine.TryInsertOnLane`). Topology: the only onward
    connection from `C1B1` to the route's next edge `B1B2` is `fromLane=0 toLane=0 dir=r` (lane 0
    only); `C1B1` lane 1 connects only to `B1A1`/`B1B0`/`B1C1` (other edges), NEVER `B1B2`. The
    vehicle is on lane 1 and is NOT redirected to lane 0, so `ResolveSequenceCore` finds zero
    candidate connections and throws.
    **Why C2-iii/C2-v don't cover it:** `ResolveSequenceCore` computes each edge's EXIT lane as
    `arrivalLane + BestLaneOffset` but only when `offset != 0 && targetExists`. Here the arrival lane
    (lane 1) has NO connection to the route's IMMEDIATE next edge at all, yet the redirect to the
    connecting sibling (lane 0) is not happening -- likely `ComputeBestLanes` isn't giving lane 1 a
    nonzero offset toward lane 0 for this geometry (SUSPECT: the backward pass treats "continues to
    SOME next edge" as continuing -- `allowsContinuation` / `toLanes[i].Count > 0` keyed on ANY
    connection -- rather than "continues to the ROUTE's next edge", so lane 1 looks like it continues
    when it only continues off-route). First step: instrument `ComputeBestLanes`/`ResolveSequenceCore`
    for route `C1B1 B1B2 B2C2` to confirm which (the offset value lane 1 gets, and whether
    `BackwardPassEdge`'s `toLanes`/`allows` is filtering to the route's next edge -- it already takes
    `nextEdgeId`, so check the forward-init `allows[i]`/`reachableNext` path).
    **Invariant to enforce:** on EVERY edge, the resolved exit lane must have a `<connection>` to the
    route's next edge; if the arrival lane doesn't, redirect to the sibling lane that does (a
    strategic intra-edge change) -- exactly SUMO's per-edge `bestLaneOffset` (`MSVehicle::
    updateBestLanes`/`LaneQ`, `MSVehicle.cpp:5744-6063`), the SAME port target as C2-iii/C2-v.
    **Done-condition (standard isolate -> golden -> reverse-engineer -> port -> gate):** (1) the
    general `-L 2` repro (and `scripts/gen-benchmark.sh` regenerated at `-L 2`) inserts + runs to
    completion -- no `No <connection> found` for any route SUMO routes. (2) NEW anchor: a minimal
    2-lane net where a vehicle's arrival/depart lane on an edge does NOT connect to its route's next
    edge but a sibling does (DISTINCT from 36 = multi-hop best-lane and 37 = intra-edge change on a
    lane that DID connect immediately); sigma=0, SUMO golden `--precision 6`, exact @1e-3 on
    lane/pos/speed. (3) INERT: scenarios 36/37/18 + all committed stay green (currently 151);
    `Sim.Bench` highway-dense determinism hash unchanged. (4) parity-reviewer ACCEPT, faithful to
    `MSVehicle.cpp` (no curve-fit). Build the anchor + golden FIRST, then instrument + fix.
  - **C2-iv. DONE (ingest-robustness -- NOT a parity axis). `DemandParser` stock-output loading.**
    Both forms the benchmark used to work around at the generation layer now load directly
    (`RungC2ivIngestRobustnessTests`, 4 offline unit tests). (a) **Embedded routes:** a
    `<route edges="..."/>` nested inside `<vehicle>` (duarouter's default output) is synthesized into
    a named Route keyed `!<vehId>` and referenced by the vehicle, so the route-by-id pipeline is
    unchanged. (b) **`DEFAULT_VEHTYPE` synthesis:** a `<vehicle>` with no `type=` falls back to
    `DEFAULT_VEHTYPE`, synthesized (once, if referenced-but-undeclared) as SUMO's built-in default
    (vClass passenger, sigma 0.5, class param table via `VTypeDefaults.Resolve`) instead of throwing
    `KeyNotFoundException` on `VTypesById[""]`; a user-declared `DEFAULT_VEHTYPE` still wins. INERT for
    every committed scenario (all use explicit `type=` + top-level routes) -- suite 128 -> 132 green.
    No golden/parity-reviewer gate (not a behavioral parity change). Lets the benchmark drop its
    `--named-routes` + explicit-`DEFAULT_VEHTYPE` post-processing workarounds.
- **C3. Merging / on-ramp / zipper. DONE (exact parity @1e-3).** Minor-link CAUTIOUS APPROACH —
  the "slow to the stop line, then go once the gap is confirmed" half of the priority-junction
  mechanism that 9b did not cover (9b ported only yield-to-a-present-foe). Test:
  `RungC3OnRampMergeParityTests` (72 steps, `["lane","pos","speed"]` @1e-3, both vehicles full extent).

  **Scenario `scenarios/19-onramp-merge`** (committed golden, SUMO v1_20_0). Mainline `M` (A→J, 500m,
  priority 10) + ramp `R` (B→J, ~104m, priority 1) BOTH feed the same downstream lane `D_0`
  (`sameTarget` merge via `:J_1_0`/`:J_0_0`); junction J makes link 1 (M→D) major, link 0 (R→D) minor
  (`request index=0 response="10"`). `mA` (mainline, depart 0) + `rA` (ramp, depart 2), both at 13.89,
  `sigma=0`. SUMO: `rA` cruises `R` at 13.89, then DECELERATES near the junction (t=8: 11.906, t=9:
  7.406) toward the stop line, enters `:J_0_0` at t=10 (10.006), merges to `D_0` at t=11 and
  re-accelerates. `mA` is ~390 m away (pos 111 at t=8) — a HUGE gap — so `rA` is NOT gap-blocked; the
  slowdown is purely the cautious approach (a minor vehicle decelerates to be able to yield as it
  nears the junction, because it "cannot see" the foe lanes until within the link's foe-visibility
  distance, then re-accelerates once the gap is confirmed clear).

  **How it was resolved (this session):** the exact per-step speed is NOT a closed form derivable by
  static reading (documented dead-ends: it emerges from `planMoveInternal`'s `vLinkWait`/`opened()`
  path, not the `arrivalSpeed` cap). Built a `DEBUG_PLAN_MOVE` instrumented v1_20_0 Debug binary in a
  separate clone (`scripts/sumo-debug-instructions.md`), captured the `rA` trace, and read the exact
  internals. **The mechanism turned out simpler than the `arrivalSpeed` block suggested:** the actual
  per-step brake is just `vLinkWait = stopSpeed(speed, stopDist)` at `MSVehicle.cpp:2734`, with
  `stopDist = seen − laneStopOffset` and (minor arm, `.cpp:2656-2664`) `laneStopOffset` resolving to
  `POSITION_EPS` (0.1) since this net's lanes set no stop offset — i.e. plan to be able to stop AT the
  junction. The `arrivalSpeed`/`maxSpeedAtVisDist`/`maxArrivalSpeed` values feed only the
  arrival-TIME (`opened()`) decision, not the executed speed. Reproduced to <1e-3 from the trace:
  `stopSpeed(13.89, 22.22)=11.906333`, `stopSpeed(11.906333, 10.313667)=7.406333`; release at
  `seen(3.01) ≤ visibilityDistance(4.5)` → accelerate through.

  **Port:** a new arm in `Engine.JunctionYieldConstraint` (folded in, reusing the already-resolved
  ego-link / `<request>` / approach lane): when ego is on its approach lane and its link is minor
  (Response has any set bit ≡ `!havePriority()`) and `brakeDist < seen && seen > visibilityDistance`,
  contribute `stopSpeed(speed, seen − POSITION_EPS)` to the reducer; once `seen ≤ visibilityDistance`
  (4.5, the `NLHandler.cpp:1413` non-zipper default) it releases and free-flow/foe terms govern.
  `visibilityDistance` is a constant (no net attribute in scope). **Byte-identical elsewhere:** in
  `scenarios/11-priority-junction` a foe (vMajor) is approaching the whole time, so 9b's foe-scan
  already brakes vMinor to the SAME stop line (`stopDist == seen − POSITION_EPS`) — the two overlap
  at the identical `stopSpeed` (verified 9.433 at seen=14.9) and `Math.Min` changes nothing; far from
  the junction `stopSpeed` exceeds current speed (non-binding). `RungB5JunctionFoeTests`' two
  "lone-vMinor crosses" behavioral facts were updated: a lone minor vehicle now correctly performs
  the cautious dip (13.89→9.433→4.933, min 4.933, never a sustained stop) and crosses to `JN_0` at
  t=19 (was a naive free-cruise t=17) — the SAME golden-verified mechanism, and still cleanly
  differential vs. the external-agent FULL-halt fact (b).
- **C4. Remaining right-of-way: right-before-left, roundabouts, stop signs.** 9b did PRIORITY
  junctions only. Reuses 9b's `<request>` response/foe matrix + `opened()`. Parity axis, one
  scenario per RoW type. Goldens regenerated IN-SESSION (SUMO 1.20.0 is provisioned in the cloud
  env -- `sumo`/`netconvert` on PATH, version-matched to `SUMO_VERSION`, verified byte-for-byte
  against an existing committed golden before use).
  - **C4-i. DONE (no engine change). Right-before-left.** `scenarios/26-right-before-left`
    (`RungC4iRightBeforeLeftParityTests`, exact @1e-3). An uncontrolled symmetric cross (node type
    `right_before_left`); netconvert resolves it into a request matrix that is priority-like per
    vehicle (link 0 SJ->JN MAJOR `response="00"`, link 1 WJ->JE EQUAL state `=` `response="01"`),
    so vWest yields to vSouth (on its right). Because `JunctionYieldConstraint` is driven ENTIRELY
    by the `<request>` matrix, the 9b + C3 machinery reproduces the golden exactly with no new code
    (vSouth cruises; vWest cautious-approaches then junction-leader-follows across the crossing
    13.89->9.6097->5.1097). Anchors that the matrix-driven yield generalizes `m/M` -> `=/M`.
  - **C4-ii. DONE. All-way-stop.** `scenarios/27-allway-stop` (`RungC4iiAllwayStopParityTests`,
    exact @1e-3). Node type `allway_stop`, mutual `<request>` (each yields to the other, state
    `w`). Genuinely new mechanism vs priority/RBL: every approach must fully STOP first, then
    proceed in arrival order (longest waiter first) -- the pre-C4-ii engine DEADLOCKED here (mutual
    yield -> both halted forever). Port: `VehicleRuntime.WaitingTime` accrual in ExecuteMoves
    (MSVehicle::updateWaitingTime, `+= dt` while speed<=0.1 && accel<=0.5*maxAccel), and
    `Engine.AllwayStopConstraint` dispatched from `JunctionYieldConstraint` ONLY when
    `junction.Type=="allway_stop"` -- must-stop-first (`WaitingTime==0` => not open, MSLink.cpp:841)
    then proceed unless a responded foe is crossing now or has waited strictly longer
    (MSLink.cpp:940-945). Equal-wait arrivalTime tie-break approximated by link-index order
    (documented; not exercised -- scenario 27's waits differ by 3s). Byte-identical for every
    priority/RBL scenario (gated on junction.Type). Parity-reviewer ACCEPT (stash-test confirmed
    the deadlock without the change).
  - **C4-iii. DONE (both mechanisms, exact @1e-3) -- successive-lane speed limit AND two-vehicle
    entry-yield via the junction arrival-time subsystem** (header was stale; both sub-bullets below
    landed with passing trace-verified tests, scenarios 32/33). A single-lane priority roundabout (circulating edges
    priority 10, approaches priority 1; each entry a standard minor link + `sameTarget` MERGE onto
    the ring's next lane). Two independent mechanisms surfaced here:
    - **DONE (this session): successive-lane free-flow speed limit.** `MSVehicle::planMoveInternal`
      caps the free-flow speed so a vehicle never enters an UPCOMING route lane faster than that lane
      permits (MSVehicle.cpp:2894-2900: per following lane `va = MAX2(cfModel.freeSpeed(getSpeed(),
      seen, laneMaxV), vMinComfortable); v = MIN2(va, v)`). Every prior scenario left this
      unexercised (speed-dropping junction turn lanes were always OFF the tested vehicle's path); a
      roundabout's curved ring internal lanes (`:RW_1` 9.11, `:RS_1` 5.58) are the first on-route
      drop. Port = `KraussModel.FreeSpeed` (base MSCFModel.cpp:105-121 Euler `freeSpeed`, NOT
      overridden by Krauss; IDM family routes through its own `IdmModel.FreeSpeed` override,
      MSCFModel_IDM.cpp:78) + `Engine.SuccessiveLaneSpeedConstraint`. Also fixed the lane-end
      boundary to STRICT `>` (`MSVehicle::processLaneAdvances`, MSVehicle.cpp:4282 `myPos >
      getLength()`): a vehicle that lands EXACTLY on `pos == laneLength` (routine when the
      successive-lane cap sets speed = remaining distance) stays on its lane one more step -- was
      `>=` (advance at ==), a latent deviation invisible until a vehicle landed on the boundary.
      Anchor `scenarios/33-roundabout-solo` (`RungC4iiiSuccessiveLaneSpeedParityTests`, single
      circulating vehicle, exact @1e-3); byte-identical for all prior scenarios (122 green).
    - **DONE (this session): two-vehicle entry-yield via junction ARRIVAL-TIME right-of-way.**
      `scenarios/32-roundabout` (`RungC4iiiRoundaboutRowParityTests`, exact @1e-3) adds vSouth
      entering at RS while vWest circulates through. vSouth must STOP-line yield to vWest while vWest
      is still APPROACHING the merge (on the ring's approach lane, not yet on its internal lane).
      `SameTargetMergeConstraint` only followed a foe already ON the merge/target lane; a blanket
      "stop for any approaching merge foe" over-yields when the foe is FAR (broke scenario 19/C3,
      mainline ~362 m away). Ported SUMO's arrival-time gate (`MSLink::opened`/`blockedByFoe`,
      MSLink.cpp:747-1013): a new PHASE-0 arm in `SameTargetMergeConstraint` blocks ego iff a
      responded foe **within its approach-reservation range** (`MSVehicle::setApproaching`'s
      lookahead `dist = SPEED2DIST(maxV) + brakeGap(maxV)`) has an arrival-time window overlapping
      ego's within a 1 s `lookAhead` (`KraussModel.MinimalArrivalTime` = getMinimalArrivalTime +
      `Engine.BlockedByMergeFoe` = the three-way follower/leader/hard-conflict test). The
      reservation-distance gate is what excludes the distant scenario-19 mainline (it never reserves
      the link). VERIFIED per-step against the vendored v1_20_0 `MSLink_DEBUG_OPENED` trace
      (`debug/arrivaltime-row-trace` on `pjanec/sumo`): `blocked (hard conflict)` for vSouth t=14..18,
      then PHASE 1 (merge-leader, vWest on :RS_1) at t=19. Arrival SPEEDS are approximated by the
      current speed (a constant-speed arrival); the block decision's wide margins make it robust
      (exact trajectory + no regression across the 19/29/31/32 merge scenarios). The leader-branch
      `unsafeMergeSpeeds` path is present for fidelity but not exercised by a committed scenario (the
      reservation gate short-circuits the only far-foe case). Suite 122 -> 123 green.
    - **DIAGNOSED + ATTEMPTED + REVERTED (deferred rung): tight-roundabout box-blocking under saturation.**
      On a pathologically tight 25 m single-lane ring (~35 m ring edges) at 251 concurrent trips the engine
      gridlocks (69 stuck vs SUMO 0). SUMO keeps it flowing via `checkRewindLinkLanes`
      (`MSVehicle.cpp:5026`, active because the ring is an UNMARKED priority ring, not a
      `isRoundabout()` edge — marked roundabouts SKIP the check and flow via circulating-priority, which
      the engine already handles: viz `city-mixed-1k` ~2% stuck). The exact missing piece is the
      `last->myHaveToWaitOnNextLink` exit-space termination (`MSVehicle.cpp:5126`) — a moving vehicle
      braking to a stop at its OWN next link counts as blocking. TRIED the `!WillPass` snapshot proxy in
      `LaneSpaceTillLastStanding`; it made the ring WORSE (69→76) and was reverted (the front-first walk
      returns space to the frontmost about-to-stop vehicle without subtracting moving vehicles behind it).
      A correct fix needs the full `availableSpace` accumulation with a real has-to-wait flag (a second
      planning pass) kept byte-identical on 34/38 — a deep architectural addition, deferred. Full
      write-up: `C4-VII-REMAINING.md` "#3 Roundabout box-blocking".
  - **C4-iv. DONE (symmetric merge, exact @1e-3). sameTarget-merge yield (the C3 merge half).**
    `scenarios/31-merge-yield-sym` (`RungC4ivMergeYieldParityTests`, exact). A SLOW major vehicle mA
    crawls across the merge exactly as the minor vehicle vB arrives, so vB must follow-YIELD to mA
    onto the shared lane and then car-follow it (scenario 19's mA is far, so C3 never exercised
    this). Port = `Engine.SameTargetMergeConstraint`, VERIFIED per-step against the vendored v1_20_0
    `DEBUG_PLAN_MOVE_LEADERINFO` getLeaderInfo/adaptToJunctionLeader trace
    (`c4iv-merge-trace` on `pjanec/sumo`). Two phases (foe on its internal lane -> foe on the shared
    target lane), each a car-following LEADER; gap<0 -> stopSpeed to the junction entry; and the key
    gate -- the merge is NON-BINDING while ego is on its approach lane beyond foe-visibility (4.5) of
    the entry (SUMO's `MAX2(vSafeLeader, vLinkWait)` relaxation, MSVehicle.cpp:3478 -- the cautious
    approach governs there), binding only within visibility / on the internal lane. Byte-identical
    for every existing scenario incl. scenario 19/C3 (also a sameTarget merge but mA is never on the
    merge lane while rA is within visibility -> the arm returns +infinity). **Asymmetric-geometry
    refinement DONE in C4-v (see below):** `scenarios/29-merge-yield` (an ASYMMETRIC on-ramp: curved
    R vs straight M internal lanes) is now an EXACT anchor (`RungC4vMergeGeometryParityTests`) -- its
    residual was the `lengthBehindCrossing` term `(flbc - lbc)` this port originally set to 0; C4-v
    ports the merge conflict-geometry so both the symmetric (31) and asymmetric (29) merges pass. The
    full two-phase mechanism + gating below was the hard part and is DONE.
  - **C4-v. DONE (exact @1e-3). sameTarget-MERGE conflict geometry (`lengthBehindCrossing`).**
    Ports `MSLink::computeDistToDivergence` (MSLink.cpp:561, `sameSource=false` arm) to
    `PolylineGeometry.ComputeDistToDivergence` + the `MergeConflict` static geometry at ingest, so
    `SameTargetMergeConstraint`'s gap uses `distToCrossing = distToMerge - egoLbc`,
    `leaderBackDist = (foeLen - foeLbc) - leaderBack`, and the `leaderBackDist2` correction
    (MSLink.cpp:1632-1638). Verified byte-exact against the v1_20_0 DEBUG trace (lbc/flbc
    10.827662/10.822642 scen29, 14.747385/14.748960 scen31). Makes scenario 29 exact; scenario 31
    unchanged (symmetric no-op). Parity-reviewer ACCEPT (stash test: 29 fails pos maxAbs 0.010
    reverted).
    *(Original blocked-anchor note for `scenarios/29-merge-yield`, retained for context:)* A SLOW
    mainline vehicle mA (maxSpeed 6) crawls across the `:J_1_0` merge lane exactly when ramp vehicle
    rA arrives, so rA must follow-YIELD to mA onto the shared lane D.
    **Mechanism fully reverse-engineered this session** (from `MSLink::getLeaderInfo`,
    `sumo/src/microsim/MSLink.cpp:1349-1663`): a sameTarget pair (ego's + foe's connections feed the
    same `(To,ToLane)`) geometrically MERGES, not crosses, so no `JunctionConflict` is recorded --
    the foe is instead a car-following LEADER at the merge point (shared internal-lane end),
    crossingWidth forced to 0, gap = `distToMerge - egoMinGap - (foeInternalLaneLen - foeBackPos)`.
    Verified: the golden's first brake is `7.4063 -> 2.9063 == speed - maxDecel` (the negative merge
    gap drives max comfortable deceleration).
    **BOTH phases IMPLEMENTED this session, then REVERTED (parity bar -- converges but not
    1e-3-exact).** It is genuinely TWO mechanisms: (1) merge-leader while the foe is ON its internal
    lane -- `MSVehicle::adaptToJunctionLeader` with `distToCrossing==-1` (MSVehicle.cpp:3223-3239):
    `gap>=0` -> `followSpeed` against the foe; `gap<0` -> `stopSpeed(speed, seen - egoInternalLen -
    POSITION_EPS)` (stop before the junction entry, NOT raw followSpeed -- that was the first bug,
    it over-braked to 0); (2) **cross-lane leader following** once the foe moves onto the shared
    target lane while ego is upstream -- ego car-follows the rearmost vehicle currently on the target
    lane, gap = `distToMerge + (leaderPos - leaderLen) - egoMinGap`. With both, the whole trajectory
    TRACKS the golden and converges (asymmetric anchor 29: within ~0.005; symmetric anchor 31: within
    ~0.01 in the convergence tail). Two residuals block 1e-3, each needing the runtime trace to pin
    exactly:
      - **Conflict-geometry offset (anchor 29):** the gap is off by `egoLBC - foeLBC` (~0.0047) --
        the `lengthBehindCrossing` differs between the CURVED `:J_0_0` and STRAIGHT `:J_1_0`
        (angle-based `conflictSize`, MSLink.cpp:354-382, + `interpolateGeometryPosToLanePos` on the
        curve). ANCHOR 31 (`scenarios/31-merge-yield-sym`, a SYMMETRIC Y-merge with mirror-image
        internal lanes) was built precisely to make `egoLBC==foeLBC` cancel -- isolating this out.
      - **leaderBack / partial-occupancy (anchor 31):** at the step the foe FIRST enters the merge
        lane (front just past the start, back still on the previous edge), the engine's gap is ~5 m
        (~one vehicle length) SMALLER than SUMO's -- `getLeaderInfo`'s `leaderBack =
        getBackPositionOnLane` semantics for a vehicle spanning the lane boundary differ from the
        naive `pos - length`. This over-brakes ego for ~2 steps before reconverging.
    Net: mechanism + gap formulae are correct and committed as analysis; the two residuals are
    exactly the entangled runtime details a `DEBUG_PLAN_MOVE`/`getLeaderInfo`-gDebug trace resolves
    (the C3 situation -- build the instrumented v1_20_0 per `scripts/sumo-debug-instructions.md`,
    gate `DEBUG_COND` to the ramp vehicle, read the printed `getLeaderInfo` gap/leaderBack/
    distToCrossing per step). Two anchors (29 asymmetric, 31 symmetric) + goldens + this analysis
    committed. Own focused rung; unblocks C4-iii (roundabouts).
- **C5. Junction-blocking avoidance (`keepClear` / don't-block-the-box).** `MSLink::keepClear` + jam
  detection so a vehicle does not enter a junction it cannot clear. Prevents artificial gridlock /
  spillback across intersections. Parity axis; also a property test (junction never deadlocks).
  **SCOPING (this session):** the mechanism is `MSVehicle::checkRewindLinkLanes`
  (`sumo/src/microsim/MSVehicle.cpp:5025`, ~235 lines) -- the `myLFLinkLanes` downstream
  available-space accounting that reserves room on the exit lane before committing to a link, plus
  the `jm_ignore_keepclear_time` gate. keepClear applies iff `link->hasFoes() && link->keepClear()`
  (default true) -- a junction with crossing foes. The walk seeds `seenSpace = -lengthsInFront`,
  subtracts each downstream lane's `getBruttoVehLenSum` / adds `getSpaceTillLastStanding` until a
  STOPPED vehicle is found, then if `availableSpace - lengthWithGap < 0` at a keepClear link sets
  `removalBegin` -> `myVLinkPass = myVLinkWait` (brake to the stop line). The engine has none of this
  downstream-lane occupancy machinery.
  **DONE (this session, exact @1e-3): `scenarios/34-keepclear`** (`RungC5KeepClearParityTests`). A
  4-way priority cross where `mBlock` sits STOPPED on exit edge JE (pos 6); `mThrough` (W->E major)
  keepClear-stops at the J entry (`WJ@91.8` = WJ.len 92.80 - `DIST_TO_STOPLINE_EXPECT_PRIORITY` 1.0)
  instead of creeping onto `:J_1` and blocking the box. keepClear fires because the WJ->JE link has
  crossing foe LINKS (N->S) -- `link->hasFoes()` -- even with no crossing vehicle. Port =
  `Engine.KeepClearConstraint` + the `LaneBruttoVehLenSum` / `LaneSpaceTillLastStanding` lane-
  occupancy queries over the frozen snapshot: the "removal" half of `checkRewindLinkLanes` (walk the
  downstream exit chain, subtract internal-lane brutto sums, add `getSpaceTillLastStanding`, and when
  a stopped vehicle leaves `leftSpace = avail - lengthWithGap < 0` brake to the junction-entry stop
  line). VERIFIED byte-exact against the vendored v1_20_0 `DEBUG_CHECKREWINDLINKLANES` trace
  (`debug/keepclear-trace` on `pjanec/sumo`: exit JE `stls=1.0`, `avail=1.0`, `leftSpace=-6.5`,
  `removalBegin=0`). Inert (+infinity) for every jam-free scenario (125 green). SIMPLIFICATIONS
  (documented in the method header): `lengthsInFront=0` (a queue on ego's own approach is ordinary
  car-following), the single-empty-internal-lane back-propagation, and the 1.0 priority stop offset.
  **FOLLOW-ON DONE (willPass): the keepClear/cross-traffic coupling.** A crossing vehicle no longer
  yields to a keepClear-stopped foe. Anchor `scenarios/38-keepclear-crosstraffic`
  (`RungC5WillPassParityTests`, exact @1e-3) = scenario 34's box WITH `nCross` (N->S minor) restored:
  mThrough keepClear-stops, and nCross CROSSES freely (SUMO clears mThrough's request via
  `setRequest=false`, so `blockedByFoe`'s `!avi.willPass` short-circuit, MSLink.cpp:935, lets nCross
  proceed). Port = `Engine.FoeKeepClearBlocked` gating JunctionYieldConstraint's approaching-foe arm:
  when the approaching foe is keepClear-blocked (its own checkRewindLinkLanes removal would fire), ego
  does not yield. INERT outside the keepClear box (KeepClearConstraint is +infinity there) -- suite
  133 -> 135 green. NOTE: the willPass predicate covers only the keepClear reason; other
  not-passing reasons (red-light-held, stopped-for-any-cause) are not modeled (no committed scenario
  exercises them). **DONE (parity-track, exact @1e-3): general cross-junction leader following.**
  `scenarios/39-crossjunction-leader` (`RungCrossJunctionLeaderParityTests`, Run 45) passes exact
  @1e-3. AJ->J->JB single lane, a SLOW leader (maxSpeed 3) on JB just past the junction; the follower
  (13.89) on AJ car-follows it ACROSS the junction. Two entangled mechanisms, both ported:
  (1) **cross-junction leader following** (`Engine.CrossJunctionLeaderConstraint` +
  `TryFindCrossJunctionLeader`): a per-downstream-lane leader scan (MSVehicle::planMoveInternal's
  `ahead`/getLeaderInfo loop) Min'd into the plan-phase speed alongside the same-lane
  `LeaderFollowSpeedConstraint` -- walks the route pool forward within the plan-move lookahead
  `SPEED2DIST(maxV)+brakeGap(maxV)`, gap = distToLaneStart + leaderBackPos - egoMinGap. The follower
  decelerates to 10.403 at t=4 while STILL ON AJ, then follows onto JB, converging to the leader's 3.0
  -- matching SUMO. (2) **cross-junction INSERTION safety** (`Engine.TryInsertOnLane`): the insertion
  follow-check also considers the downstream leader (`maximumSafeFollowSpeed(..., onInsertion:true)`);
  if the safe speed < departSpeed, insertion fails that step -- SUMO delays the follower to t=2. This
  required processing insertions in vehicle-DEFINITION order (SUMO's MSInsertionControl order, ties by
  route-file order) across all lanes rather than lane-sorted, so the downstream leader `lead` (defined
  first) is placed before `foll` is checked; a per-lane `blockedLanes` set preserves per-lane FIFO.
  Suite 141 -> 142. INERT: no committed scenario has a close cross-junction leader or a cross-lane
  insertion dependence, so the constraint is +inf and the insertion order is outcome-neutral there --
  all committed scenarios + the D1 determinism hash stay green (142). NON-VACUOUS: disabling the
  running constraint fails at step 4 (follower blows through), disabling the insertion check fails at
  step 2 (follower inserts at t=0, presence mismatch). SIMPLIFICATION (documented): only the FIRST
  (nearest) downstream leader is followed (no multi-leader min across several downstream lanes) --
  sufficient for the single-leader anchor. keepClear/C5 covers the box-blocking STOPPED-downstream
  case (checkRewindLinkLanes); this is the MOVING-leader case. Parity-reviewer ACCEPT (both mechanisms
  non-vacuous; insertion reorder verified outcome-neutral vs 15 multi-departure scenarios + D1/D8/B3
  hashes; gap/lookahead/onInsertion-followSpeed faithful to MSVehicle.cpp:2238/3040 + MSLane.cpp:1341
  + MSCFModel.cpp:334). **FOLLOW-UP (flagged by reviewer, not yet needed):** `TryFindCrossJunctionLeader`
  returns only the NEAREST downstream leader; SUMO Min's over ALL downstream lanes within the
  lookahead. Inert for every committed scenario -- but a future scenario with a farther, slower
  downstream leader that binds tighter than a nearer one would need the multi-leader min.
- **C6. Actuated / adaptive traffic lights + yellow decision.** Rung 10 did STATIC `tlLogic` only.
  - **C6-i. DONE. Yellow decision ("stop if you can brake, else go").** `scenarios/30-yellow-decision`
    (`RungC6YellowDecisionParityTests`, exact @1e-3). Ported the `canBrakeBeforeStopLine` gate from
    MSVehicle.cpp:2754 (condition at :2648, `seen - stopOffset >= brakeDist`) into
    `Engine.RedLightConstraint`: a vehicle too close to a yellow/red light to stop in time PROCEEDS
    through the junction instead of emergency-braking (the dilemma-zone "go" decision), rather than
    always braking as rung 10 did. Scenario: scenario 09's TLS net, lane speed 25, a Green/yellow/red
    static program; veh0 hits yellow at seen 44 m < its 57.5 m braking distance and cruises through
    at 25 (the pre-C6 engine wrongly halted it at the stop line -- stash-test confirms). Byte-
    identical for rung 10 (scenario 09) and emergency-red (scenario 16): those vehicles always
    approach from far enough that the gate never fires. Parity-reviewer gated.
  - **C6-ii. DONE (parity-track, exact @1e-3). Actuated (detector-driven) TLS -- the FIRST stateful
    traffic-light program.** Port: `src/Sim.Core/ActuatedTrafficLightLogic.cs` (new) =
    `MSActuatedTrafficLightLogic` gap-based algorithm (`trySwitch`/`gapControl`/`duration`) +
    `MSInductLoop` (notifyMove/getTimeSinceLastDetection); parser gained `TlLogic.Type` +
    per-phase `TlPhase.MinDur`/`MaxDur` (`IsActuated` = MinDur != MaxDur); Engine builds one machine
    per `type="actuated"` tlLogic in LoadScenario, resets them at Run() top, `Advance(time+dt)` before
    PlanMovements, feeds the induction loops from ExecuteMoves (`NotifyMove`, newTime=`time+dt`), and
    RedLightConstraint reads the machine's current phase instead of the pure-function
    `TrafficLightState` when `IsActuated`. **TEST `RungC6iiActuatedTlsParityTests`** (scenario 35,
    Run 40) passes exact @1e-3 across all 5 vehicles / 40 steps; suite 137 -> 138. **THE HARD PART
    (detector-timing convention) RESOLVED:** the +1.0 lag noted below is exactly reproduced by
    stamping `myLastLeaveTime = newTime + passingTime(...)` where `newTime = time+dt` (the FCD frame
    the ExecuteMoves move produces = SUMO's SIMTIME during executeMove) and advancing the event-driven
    machine at `time+dt` using detector state settled by the PREVIOUS step -- i.e. SUMO's
    begin-of-step trySwitch seeing leave-times through step tau-1. Phase 0 extends 5->7->8->10->12->13
    exactly as the trace; ns0-3 + ew0 FCD match to 1e-3, which pins the whole timeline (a one-step
    phase shift would blow ew0's release by a full second). NON-VACUOUS: forcing the static path holds
    WJ red past t=40 (ew0 never moves) -> test fails. INERT: static-TLS + all committed scenarios stay
    byte-identical (138 green incl. the D1 determinism hash). DOCUMENTED SCOPE (unexercised, called out
    inline in `ActuatedTrafficLightLogic.cs`): default gap-based algorithm only (no
    switching-rules/conditions/multi-target/TraCI overrides), jam disabled, greenMinor check1a-f
    admittance not ported (anchor's green links are all major), offset != 0 unhandled. Parity-reviewer
    gated. *(original SCOPED briefing retained below for context.)*
    Actuated
    (detector-driven) programs. `MSActuatedTrafficLightLogic` (~1436 lines) + `MSInductLoop`. Unlike
    the STATIC TLS (a pure function of time, `Sim.Core.TrafficLightState`), an actuated program is
    STATEFUL: each green phase EXTENDS while an induction-loop detector keeps seeing vehicles
    (`gapControl` returns `detectionGap < maxGap`) and ENDS when the gap opens, bounded by
    minDur/maxDur. Core algorithm (`trySwitch` :710 -> `gapControl` :834 -> `duration` :814): per
    step, `detectionGap = min over the phase's loops of getTimeSinceLastDetection (< maxGap, not
    jammed)`; if finite, `duration()` extends the phase (`newDuration = MAX3(minDur-actDur,
    TIME2STEPS(detectorGap - detectionGap), 1)`, integer-rounded, capped to maxDur-actDur/latest);
    else switch to the next phase. Detectors placed at `laneLength - MIN2(detectorGap*speed,
    (minDur/passingTime + 0.5)*7.5)`. Defaults: max-gap 3.0, detector-gap 2.0, passing-time 1.9.
    **ANCHOR `scenarios/35-actuated-tls`** (committed, NOW TESTED -- see DONE marker above): a single actuated junction J (2
    green phases Gr/rG, minDur 5 / maxDur 50, + 3s yellows). Four N-S vehicles stream over the SJ
    detector so phase 0 EXTENDS from minDur 5 to t=13 (vs the static duration=42) before ending; ew0
    waits at the WJ red and is released when phase 2 turns green at t=16. `phase-timeline.txt` records
    the golden's actuated phase sequence (ground truth). The engine parses tlLogic as STATIC (ignores
    `type="actuated"` + minDur/maxDur), so it holds ew0 far too long -> divergence.
    **PORT SCOPE (large, stateful -- the bulk is architecture, not numbers):** (1) an induction-loop
    detector model with `getTimeSinceLastDetection` updated each step from vehicle positions; (2)
    per-TLS runtime state (`myStep`/`myLastSwitch`) -- the FIRST stateful TLS, so a new per-step
    system phase that updates detectors + runs `trySwitch` (the current TLS is a pure plan-phase
    recompute); (3) `gapControl`/`duration`/`trySwitch`; (4) parser support for `type="actuated"` +
    per-phase `minDur`/`maxDur`. Trace handoff prepared:
    `scripts/sumo-actuated-tls-trace-instructions.md` (`DEBUG_DETECTORS` + `DEBUG_PHASE_SELECTION`
    gated to `J`) + `actuated-tls-trace.zip`. **TRACE RECEIVED + ANALYZED** (`debug/actuated-tls-trace`
    on `pjanec/sumo`, verified byte-matching scenario 35). Detector placement confirmed: `ilpos =
    laneLength - inductLoopPosition`, `inductLoopPosition = MIN2(detectorGap*speed=2*13.89,
    (minDur/passingTime+0.5)*7.5) = 23.4868` (SJ det ilpos 119.313, WJ 122.513), detLength 0.
    `trySwitch` is EVENT-DRIVEN (called only at the phase's computed end, not every step); at each
    call `gapControl` = min over the phase's loops of `getTimeSinceLastDetection` (if < maxGap 3.0 and
    !jammed, else -1=inf=end); if finite, `duration()` = `MAX3(minDur-actDur, ceil(myDetectorGap 2.0
    - detectionGap), 1)` sets the next end (scenario-35 phase-0 extends 5->7->8->10->12->13 then ends
    when gap>=3.0). **THE HARD PART (and why it is deferred as HIGH-RISK / precision-sensitive):**
    `getTimeSinceLastDetection` = `now - myLastLeaveTime` where `myLastLeaveTime = SIMTIME +
    passingTime(oldBackPos, ilpos, newBackPos, ...)` (MSInductLoop.cpp:165). The trace's actualGap is
    consistently the naive geometric value MINUS exactly 1.0 s (e.g. ns1 back-exits ilpos at 3.9102
    but the gap at t=5 is `5-(3.9102+1.0)=0.0898`, not `5-3.9102`), i.e. a one-step lag from SUMO's
    detector-update-vs-TLS-control ordering + the FCD/SIMTIME step convention. The phase chain is
    INTEGER-ROUNDING sensitive (a wrong gap flips `duration()`'s ceil or the `>=maxGap` end test ->
    the whole timeline shifts), so this rung must nail the step/time convention exactly -- best done
    as a focused pass with the trace open, not folded into a broad autonomous run. Everything except
    that detector-timing convention is fully characterized above.
- **C7. `speedFactor` distribution (heterogeneous desired speeds).** Per-vehicle desired-speed
  variation (`speedFactor` = `normc(1.0, dev)`, `default.speeddev`); today everyone wants exactly the
  limit (mean 1.0, dev forced 0). Depends on C1 (seeded RNG). Statistical parity. Produces realistic
  speed spread and overtaking pressure.

  **Decomposed (like C1/B5/9b):**
  - **C7-i. DONE (OFFLINE, no SUMO). `speedFactor` sampler + per-vehicle draw.** New
    `NormcDistribution` (`src/Sim.Core/NormcDistribution.cs`), porting
    `Distribution_Parameterized::sample` (`sumo/src/utils/distribution/Distribution_Parameterized.cpp:107-120`,
    the `normc`/4-param `(mean,dev,min,max)` branch: `dev<=0` returns `mean` with NO draw at all;
    otherwise `randNorm` + a reject-resample `while(val<min||val>max)` clamp loop),
    `RandHelper::randNorm` (`sumo/src/utils/common/RandHelper.cpp:137-147`, the polar/Marsaglia
    method incl. the `ceil(log(q)*1e14)/1e14` quantized log term), and
    `MSVehicleType::computeChosenSpeedDeviation` (`sumo/src/microsim/MSVehicleType.cpp:89-91`:
    `roundDecimal(MAX2(minDev, sample), gPrecisionRandom)`). **Source correction (CLAUDE.md rule
    1):** `gPrecisionRandom` is **4**, not 6 as an earlier draft assumed — `sumo/src/utils/common/StdDefs.cpp:28`
    sets `int gPrecisionRandom = 4;` outright; ported from the source, not the stale assumption.
    `roundDecimal` (`StdDefs.cpp:52-56`, round-half-away-from-zero, NOT banker's rounding) ported
    verbatim as `NormcDistribution.RoundDecimal`. OWNER DECISION (mirrors C1): the distribution
    SHAPE is ported faithfully; the RNG STREAM is ours (`VehicleRng`/SplitMix64), never SUMO's
    RandHelper/MT19937.
    New `VehicleRng.SeedFor(globalSeed, entityIndex, salt)` 3-arg overload (`src/Sim.Core/VehicleRng.cs`)
    derives a SECOND, fully independent per-entity stream from the same `(Seed, entityIndex)` pair
    via an XOR salt — used ONLY for the once-at-creation `speedFactor` draw
    (`Engine.LoadScenario`, salt = the ASCII bytes of `"SpeedFac"` packed into a `ulong`), NEVER
    for `VehicleRuntime.RngState` (C1's per-step dawdle stream), so the two draws can never alias
    or steal from each other regardless of `default.speeddev`. New `VehicleRuntime.SpeedFactor`
    (plain `double`, D3-clean unmanaged field) holds the drawn value, computed ONCE at vehicle
    creation from `NormcDistribution.ComputeChosenSpeedDeviation(vType.SpeedFactor /*mean*/,
    ScenarioConfig.SpeedDev /*dev*/, min: 0.2, max: 2.0, ref speedFactorRng)` — `vType.SpeedFactor`
    is now purely the distribution MEAN fed into the sampler (still 1.0 for every existing
    scenario/vType), not the vehicle's actual desired-speed multiplier.
    `KraussModel.LaneVehicleMaxSpeed` gained a `speedFactor` parameter (`Math.Min(laneSpeed *
    speedFactor, vType.MaxSpeed)`); all four `Engine.cs` call sites (~817, 1727, 1728, 1879) now
    pass `v.SpeedFactor` instead of reading `vType.SpeedFactor` directly.
    **Byte-identical when `speeddev<=0`:** every existing scenario's `default.speeddev="0"` makes
    `NormcDistribution.SampleNormc`'s `dev<=0` branch return `mean` (1.0) immediately — no draw of
    any kind, from either RNG — so `v.SpeedFactor` is exactly `vType.SpeedFactor` (1.0) and
    `LaneVehicleMaxSpeed` is bit-for-bit its pre-C7 formula. Confirmed via the full `dotnet test`
    run (98 passed, 0 failed, up from 94) AND by name-checking `Rung1ParityTests`/
    `Rung9bParityTests` (real `golden.fcd.xml`/`tolerance.json` comparisons) still pass unchanged.
    New fixtures `scenarios/_fixtures/speedfactor-single-lane` (single vehicle, `sigma=0`,
    `default.speeddev=0.1`, isolating the speedFactor effect from dawdle) and
    `scenarios/_fixtures/speedfactor-independence` (a matched LOW-mean/HIGH-mean vType pair,
    `sigma=0.5`, `default.speeddev=0.05`, sharing one net/config) +
    `tests/Sim.ParityTests/RungC7SpeedFactorTests.cs` (4 new behavioral/property tests, no
    golden): (1) `speeddev=0` control reaches exactly the lane free-flow speed, no draw; (2)
    same-seed determinism with `speeddev>0`; (3) a 50-seed ensemble of single-vehicle
    `sigma=0`/`speeddev=0.1` runs shows positive cross-seed variance in steady-state speed, a mean
    near the lane's free-flow speed, and every sample bounded by the `normc` clamp
    `[0.2*laneMax, 2.0*laneMax]`; (4) **C1 independence** — because
    `KraussModel.FinalizeSpeed`'s own formula reduces to `vMax = MaxNextSpeed(oldV)` (accel-only,
    target-independent) during the depart-from-rest accel ramp, a same-seed LOW-mean-target
    (~13.89 m/s) vs HIGH-mean-target (~25 m/s) run pair — otherwise identical vType/sigma, both
    with a REAL (`dev=0.05>0`) speedFactor draw — MUST produce byte-identical dawdle-perturbed
    speeds for t∈[1,3]s unless the speedFactor sampler's salted RNG leaked into `RngState`; the
    test asserts exactly that equality (and it holds, confirming the two streams never alias).
  - **C7-ii. DONE ([net] golden regen, offline test). SUMO ensemble golden + statistical parity
    test.** New scenario `scenarios/20-speedfactor-freeflow` (single vehicle, `default.speeddev=0.1`,
    `sigma=0` to isolate speedFactor, free flow on a 2000m lane). Golden = a 50-run SUMO ENSEMBLE
    (`golden.ensemble/seed01..50.fcd.xml`, `sumo --seed 1..50`), committed with `provenance.txt`.
    `tolerance.json` `parityMode="statistical"`, `comparedAttributes=["speed"]`,
    `statistical.speed.mean=0.7`/`std=0.2`. Test `RungC7iiStatisticalParityTests` runs the engine over
    50 seeds and asserts `CompareEnsemble` (C1-ii) `IsMatch`. **Result:** the ramp (0→steady) is
    byte-identical (accel 2.6); the pooled speed **std matches to Δ=0.046** (the discriminating check —
    spread == the 0.1 dev; `std` tol 0.2 catches a dev error >~7%), validating the distribution.
    The pooled **mean delta is 0.504** (the honest finite-ensemble sampling floor: two independent
    50-sample draws of `speedFactor~N(1,0.1)` have means bracketing 1.0 — SUMO 0.986, engine 1.023 —
    a ~0.51 m/s gap within 50-sample sampling error), covered by `mean` tol 0.7. Both deltas
    deterministic (fixed seeds + golden) so the test is stable. **C7 (both sub-rungs) DONE** — the
    speedFactor distribution is built and validated against SUMO. Full suite: 99 passed, 0 failed.
- **C8. Ballistic integration + `actionStepLength > 1` (reaction time).** SUMO's ballistic update
  (more accurate than Euler) and sub-second/multi-second reaction time. The integration method is
  already a config flag (DESIGN.md seam); this ports the ballistic `finalizeSpeed`/position update and
  the action-step sub-sampling. Parity axis — effectively a config variant of every scenario, so it
  needs its own goldens (ballistic on).
  - **C8-i. DONE (ballistic integration, free flow). `[net]` golden + offline test.** Ballistic
    (`step-method.ballistic=true`) differs from Euler ONLY in the position update: SUMO's trapezoidal
    `pos += 0.5*(oldSpeed + newSpeed)*dt` (the `!gSemiImplicitEulerUpdate` branch) vs Euler's
    `pos += newSpeed*dt`; the free-flow SPEED sequence is identical (accel-bounded). `ExecuteMoves`
    now branches on `_config.Ballistic` (capturing `oldSpeed` before the overwrite); **byte-identical
    to the old code when `Ballistic=false` (every existing scenario)**. New scenario
    `scenarios/21-ballistic-freeflow` (scenario 01's net, `ballistic=true`, single vehicle 0→13.89)
    + SUMO golden (verified t=1 pos 1.30 = 0.5·2.6·1, t=6 pos 45.945). Test `RungC8ParityTests`
    matches it to 1e-3. Full suite 100 green (99 Euler unchanged + this). **Deferred to C8-ii+**: the
    ballistic SAFE-SPEED branches (`maximumSafeStopSpeedBallistic`/`followSpeed`/`finalizeSpeed`
    ballistic) — they never bind free-flow; need a ballistic-with-leader scenario.
  - **C8-ii. DONE. `actionStepLength > 1` (reaction time).** `scenarios/28-actionstep`
    (`RungC8iiActionStepParityTests`, exact @1e-3, action-step-length=2). A vehicle re-plans its
    speed only every `actionStepLength` seconds; between action steps it CONTINUES with the
    acceleration decided at the last one (ported from MSVehicle.cpp:4443-4462 -- a non-action step
    skips `processLinkApproaches` and sets `vSafe = speed + accel*dt` with NO `finalizeSpeed`;
    isActionStep at MSVehicle.h:638). Port: `VehicleRuntime.LastActionTime` + an action-step gate at
    the top of `Engine.ComputeMoveIntent`, guarded by `actionStepLengthSecs > dt` so every prior
    scenario (all action-step-length=1) is byte-identical (the block is skipped entirely). The
    discriminating golden step: at the action step t=4 (speed 10.4) SUMO plans accel 1.745 to reach
    the 13.89 cap over the 2s interval and HOLDS it through the non-action step t=5 (12.145) into
    t=6 (13.89) -- every-step re-planning would instead give ~13.02 at t=6 (confirmed: stash-test
    fails at first-divergence t=6). NOTE (scoped): the isActionStep schedule is anchored for a
    depart-0 vehicle; per-vType action-step offsets and depart!=0 phase alignment are untested
    (no scenario needs them yet).
  - **C8-iii. DONE (parity-track, exact @1e-3). Ballistic car-following.** The ballistic safe-speed
    branches, completing ballistic parity beyond free-flow (C8-i did the position update). Port:
    `KraussModel.MaximumSafeStopSpeedBallistic` (MSCFModel.cpp:855-910, all three cases:
    insertion / stop-within-tau / positive-speed-after-tau) + the ballistic `BrakeGap` branch
    (`speed*(headway + 0.5*speed/decel)`), threaded through `MaximumSafeFollowSpeed` -> `FollowSpeed`
    -> `Engine.FollowSpeedFor` via a `bool ballistic` flag sourced from `config.Ballistic` at the 6
    FollowSpeedFor call sites (LeaderFollowSpeedConstraint made instance to read `_config`). **TEST
    `RungC8iiiBallisticFollowParityTests`** (`scenarios/42-ballistic-follow`, Run 40) passes exact
    @1e-3: a fast follower car-follows a slow (maxSpeed 5) leader under `step-method.ballistic=true`,
    matching SUMO from the first braking step (13.24 at t=8) down to ~5. Suite 153 -> 156. INERT:
    the flag defaults false and `config.Ballistic` is false for every scenario but 21/42, so the Euler
    safe-speed path is byte-identical (all committed + D1 hash green); only the Krauss arm reads the
    flag (IDM/ACC/CACC ballistic out of scope). NON-VACUOUS: forcing the Euler safe-speed under
    ballistic fails at step 8 (speed err 1.93). SIMPLIFICATION (documented): the braking is gentler
    than maxDecel so `finalizeSpeed`'s ballistic `minNextSpeed`/negative-speed branch is not
    exercised; a ballistic EMERGENCY-brake scenario (vehicle stops within a step, speed goes negative)
    would need that branch + the ballistic position update for a stopping vehicle. Parity-reviewer
    ACCEPT (faithful port of MSCFModel.cpp:854-917; byte-identical Euler -- 156 green incl. D1 hash;
    non-vacuous, Euler-forced fails at step 8). **RE-GATE FLAG (reviewer):** a HARD-BRAKING ballistic
    scenario (per-step delta > decel -> safe speed goes negative) would exercise `finalizeSpeed`'s
    ballistic `minNextSpeed`/negative-speed branch AND the Euler-flavored `MAX2(x,0)` clamp in the
    emergency-decel correction -- both latent Euler paths today; port them when authoring that rung.
- **C9. Cooperative lane changes.** LC2013's COOPERATIVE block (`LCA_COOPERATIVE` — make room for a
  blocked/merging neighbor). Depends on A2's neighbor query + C3 (merging pressure). Parity axis.
- **C10. Continuous lateral / lane changes over time (the lateral axis). SCOPE (per user, 2026-07):
  stay ENTIRELY within SUMO-proven mechanisms and SUMO's lane model -- NO divergences, NO navmesh/RVO.**
  navmesh/RVO is a SEPARATE layer for special maneuvers (threat-zone escape, fully-blocked street) and
  EXTERNAL vehicles, out of scope for this SUMO port; the only thing this port owes the external world
  is an input surface to be TOLD about external vehicles (the B-group obstacle seam), not the avoidance
  math. The lateral axis grows only as far as SUMO's own behaviors need (continuous lane change, then
  the emergency rescue-lane give-way), staying golden-testable.
  - **C10-i. DONE (parity-track, exact @1e-3). Continuous lane change (`lanechange.duration>0`) --
    lane-label TIMING.** A lane change now spreads over `round(duration/stepLength)` steps instead of an
    instant lane-index snap: SUMO emits the SOURCE lane until the vehicle center crosses the lane
    midpoint (halfway), then the target. Port: `VehicleRuntime.Lc*` maneuver state + the
    `StartLaneChangeManeuver` command + `Engine.AdvanceLaneChanges` (a per-step Input-phase pass run
    before EmitTrajectory) + `CommitLaneChange` (routes decided changes to instant-snap when
    `LaneChangeSteps() <= 1`, else a maneuver) + mid-maneuver guards (no new keep-right/strategic/
    speed-gain decision until the maneuver completes). `ScenarioConfig.LaneChangeDuration` parsed from
    `<processing><lanechange.duration>` (default 0). **TEST `RungC10iContinuousLaneChangeParityTests`**
    (`scenarios/43-continuous-lanechange`, Run 40): E0(2 lanes)->E1(1 lane), E0_0 dead-ends so v0
    strategic-changes E0_0->E0_1 on a CLEAR road; SUMO holds E0_0 through t=1, flips at t=2 (duration
    3), pos/speed free-flow -- exact. Suite 156 -> 158. INERT: gated on duration>0, so every duration-0
    scenario keeps the instant snap (all committed + D1 hash byte-identical). NON-VACUOUS: forcing the
    instant snap fails at step 1 (E0_1 too early). SIMPLIFICATION (documented): an EVEN `total`
    resolves the exact-midpoint step to the source lane (no committed scenario uses an even duration).
    Parity-reviewer ACCEPT (byte-identical for duration=0 -- structurally proven, all 20 duration-
    touching scenarios set 0; 158 green incl. D1 hash; non-vacuous, instant-snap fails at step 1;
    midpoint rule matches the golden geometry). **RE-GATE FLAG (reviewer):** the even-`total`
    simplification resolves the exact-midpoint step to the SOURCE lane, which SUMO's geometric crossing
    may not -- gate/test before adding any even-duration scenario. **FOLLOW-ONS (each its own rung):** (C10-ii) the lateral POSITION
    (`LatOffset` interpolation + y/posLat emit + a lateral-comparison harness) -- the y-slide is
    deterministic (verified: -4.8 -> -1.6 over 3 steps) so it is exact-parity-testable once the harness
    compares lateral; (C10-iii) SHADOW-LANE car-following during the straddle (a blocked-road change
    decelerates while still overlapping the slow leader's lane -- verified on a blocker scenario: the
    mover drops to 6.64 mid-change); then the emergency rescue-lane give-way on top (SUMO's
    `MSDevice_Bluelight` -- sublane lateral-alignment + a STOCHASTIC reaction, so statistical not exact
    parity, matched via SUMO's own mechanism, not an approximation).
    Ref `MSAbstractLaneChangeModel` (continuous change), `MSLCM_SL2015`/`MSLaneChangerSublane` (full
    sublane), `MSDevice_Bluelight` (give-way).
- **C11. Alternative car-following models (IDM, ACC/CACC).** `MSCFModel_IDM`, `MSCFModel_ACC`,
  `MSCFModel_CACC` for modern / automated traffic. Each is a resolver dispatch (`carFollowModel`
  attribute) + a model port behind the same `KraussModel`-style constraint interface. Parity axis, one
  scenario per model.
  - **C11-i. DONE. IDM (Intelligent Driver Model).** Ported `sumo/src/microsim/cfmodels/
    MSCFModel_IDM.cpp` (whole file, plain-IDM ctor arm — `myIDMM=false` — only; ACC/CACC/IDMM
    deferred, see below) as `src/Sim.Core/IdmModel.cs`: the iterated `_v` core (delta=4.0,
    iterations=`MAX2(1,int(TS/stepping(0.25)+.5))`=4 at dt=1s, `twoSqrtAccelDecel=2*sqrt(accel*decel)`,
    headwayTime=tau — the `myAdaptationFactor!=1` headway-scaling/level-of-service branches are
    provably dead for plain IDM, adaptationFactor hardwired to 1.0, and are omitted, not ported as
    no-ops); the four entry points `freeSpeed`/`followSpeed`/`stopSpeed`/`finalizeSpeed`
    (finalizeSpeed = the shared base `MSCFModel::finalizeSpeed` accel/decel-bound clamp with NO
    dawdle — IDM never overrides `patchSpeedBeforeLC`, whose base default is a plain `return vMax`);
    `getSecureGap`; and the `minNextSpeed` OVERRIDE (`MAX2(myDecel, MIN2(myEmergencyDecel,1.5))` —
    virtual-dispatched from both `MSCFModel::finalizeSpeed`'s own vMin term and
    `StopLineConstraint`'s `vMinComfortable`, not just the stopSpeed call).
    `carFollowModel` is now parsed from `<vType>` (`DemandParser`/`DemandModel.VType.CarFollowModel`)
    and resolved into `ResolvedVType.CarFollowModel` (`VTypeDefaults.Resolve`, default "Krauss"
    unchanged). `Engine.ComputeMoveIntent` dispatches per EGO vehicle (`vType.CarFollowModel=="IDM"`)
    at every constraint that computes ego's own car-following speed: `LeaderFollowSpeedConstraint`,
    the free-flow desired-speed term (`FreeFlowDesiredSpeedConstraint`, new — IDM routes through
    `IdmModel.FreeSpeed` with `seen=+infinity`, i.e. the free-accel branch, since this engine has no
    "next lane's speed limit" lookahead for this term), `StopLineConstraint`, `RedLightConstraint`,
    `JunctionYieldConstraint`/`AdaptToJunctionLeader`, `ObstacleConstraint`, and the top-level
    `FinalizeSpeed` call — via two small dispatch wrappers (`FollowSpeedFor`/`StopSpeedFor`) whose
    Krauss arm is the EXACT pre-C11 `KraussModel.FollowSpeed`/`StopSpeed` call (same argument values,
    same order) — Krauss stays byte-identical (100 pre-existing parity tests unchanged, verified
    including `Rung1`/`Rung9b`/`RungB1`). New anchor: `scenarios/22-idm-carfollow`
    (`tests/Sim.ParityTests/RungC11ParityTests.cs`, 60 steps, exact 1e-3) — both vTypes IDM, leader
    maxSpeed=6 free-accelerates, follower free-accelerates to ~13.7 then brakes via IDM's gap term
    and settles following the leader at the IDM equilibrium gap. **Deferred**: ACC/CACC (separate
    `MSCFModel_ACC`/`_CACC` ports), IDMM (`myIDMM=true`'s adaptation-factor/level-of-service state),
    and IDM+junction/stop interplay beyond what `stopSpeed`'s port itself guarantees (ported but only
    anchored by this rung's follow/free scenario, not exercised end-to-end by a junction golden yet).
  - **C11-ii. DONE. ACC (Adaptive Cruise Control).** Ported `sumo/src/microsim/cfmodels/
    MSCFModel_ACC.cpp` (whole file) + `.h` (the `ACCVehicleVariables` state) as
    `src/Sim.Core/AccModel.cs`: the stateful `_v` control-mode machine (`accelSpeedControl` =
    `SC_GAIN(-0.4)*vErr`; `accelGapControl` selects gap/collision-avoidance/gap-closing mode by the
    `|spacingErr|<0.2 && |vErr|<0.1` / `spacingErr<0` thresholds, gains `GC=(0.07,0.23)`,
    `CA=(0.23,0.8)`, `GCC=(0.8,0.04)`; `_v` itself switches speed-control (`gap2pred>120`) vs.
    gap-control (`gap2pred<100`) vs., in the `[100,120]` hysteresis band, the vehicle's OWN
    *previous* mode — read/written via a per-vehicle `AccControlMode`/`AccLastUpdateTime` state
    pair, guarded by a "written at most once per timestep" `lastUpdateTime` check exactly like the
    vendored source); `followSpeed` (= `_v`'s result, overridden by `maximumSafeFollowSpeed()+2.0`
    — `EMERGENCY_THRESHOLD` — whenever that safety floor is more than 2.0 below it, reusing
    `KraussModel.MaximumSafeFollowSpeed` verbatim, not a distinct ACC safety formula); `stopSpeed`
    (provably the SAME formula as `MSCFModel_Krauss::stopSpeed`, so `AccModel.StopSpeed` is a thin
    pass-through to `KraussModel.StopSpeed`, not a duplicate). ACC does not override `freeSpeed` (the
    `FreeFlowDesiredSpeedConstraint` dispatch's existing non-IDM `else` arm — plain
    `laneVehicleMaxSpeed` — already covers it, no code change needed there) or `finalizeSpeed`/
    `patchSpeedBeforeLC` (inherits the base class's dawdle-free clamp, i.e. the SAME formula
    `IdmModel.FinalizeSpeed` already ports — `Engine.ComputeMoveIntent`'s dispatch now reads
    `v.VType.CarFollowModel is "IDM" or "ACC"` for that one line). **STATE**: `VehicleRuntime` gained
    `AccControlMode`(int)/`AccLastUpdateTime`(double), both default 0 (matching
    `ACCVehicleVariables`' own ctor default), written ONLY by the owning vehicle from inside
    `AccModel.FollowSpeed`, threaded `ref` through `FollowSpeedFor`'s three call sites
    (`LeaderFollowSpeedConstraint`, `ObstacleConstraint`, `JunctionYieldConstraint`/
    `AdaptToJunctionLeader` — all three now also thread `time`, the Plan-phase per-step timestamp,
    as the analog of `MSNet::getCurrentTimeStep()`) — parallel-safe under `UseParallelPlan` by the
    same per-entity-write argument as C1's dawdle-RNG mutation (that property's own header comment).
    Krauss/IDM stay byte-identical: the two new `FollowSpeedFor` parameters are simply threaded
    through unread/unwritten by their arms (100 pre-C11-ii parity tests unchanged, including
    `Rung1`/`Rung9b`/`RungA2`/`RungB1`/`RungC11ParityTests` (IDM) — verified, plus the pre-existing
    IDM anchor scenario 22). New anchor: `scenarios/23-acc-carfollow`
    (`tests/Sim.ParityTests/RungC11AccParityTests.cs`, 70 steps, exact 1e-3) — both vTypes ACC,
    leader maxSpeed=6 departs at pos 140 (initial gap ~132.5 > 120 → follower does SPEED CONTROL),
    transitions through the 100-120 HYSTERESIS band, then GAP CONTROL (<100), settling behind the
    slow leader. **Deferred**: CACC (`MSCFModel_CACC`, now done — see C11-iii below), and
    ACC+junction/stop interplay beyond what the ported `stopSpeed`/`AdaptToJunctionLeader`
    plumbing itself guarantees (not exercised end-to-end by a junction golden yet).
  - **C11-iii. DONE. CACC (Cooperative Adaptive Cruise Control).** Ported
    `sumo/src/microsim/cfmodels/MSCFModel_CACC.cpp` (whole file, CACC_NO_OVERRIDE
    `CommunicationsOverrideMode` path only) + `.h` (the `CACCVehicleVariables` state) as
    `src/Sim.Core/CaccModel.cs`: the stateful, COOPERATIVE `_v` (time-gap-based mode select —
    `timeGap>2`→speed control, `timeGap<1.5`→gap control (`speedGapControl`), `[1.5,2]`
    hysteresis→the vehicle's OWN previous `CaccControlMode`) and `speedGapControl` (no
    leader→`speedSpeedControl`; leader NOT CACC→ACC fallback calling `AccModel.V` — widened from
    `private` to `internal` — DIRECTLY, not `AccModel.FollowSpeed`, with `headwayTime=
    HEADWAYTIME_ACC=1.0`, not `vType.Tau`; leader IS CACC→the cooperative law
    `spacingErr=gap-tau*speed`, `speedErr=predSpeed-speed+tau*egoAcceleration`, gap/collision-
    avoidance/gap-closing sub-modes selected by the `0<spacingErr<0.2 && vErr<0.1` /
    `spacingErr<0` thresholds, gains `GC=(0.45,0.0125)`, `CA=(0.45,0.05)`, `GCC=(0.005,0.05)`);
    `followSpeed` (= `_v`'s result, overridden by `maximumSafeFollowSpeed(...,onInsertion:true)+2.0`
    whenever that safety floor is more than 2.0 below it — `onInsertion=true` here, UNLIKE ACC's own
    `followSpeed`, which omits the argument); `stopSpeed` (provably the same formula as
    Krauss/ACC's own, so `CaccModel.StopSpeed` is a thin pass-through, not a duplicate). CACC does
    not override `freeSpeed` (beyond a debug-only `caccVehicleMode` side-effect, not ported — no
    behavior change) or `finalizeSpeed`/`patchSpeedBeforeLC` (inherits the same base-class
    dawdle-free clamp ACC/IDM already route through — `Engine.ComputeMoveIntent`'s dispatch now
    reads `v.VType.CarFollowModel is "IDM" or "ACC" or "CACC"` for that one line).
    **STATE — the key subtlety**: `MSCFModel_CACC.h`'s `CACCVehicleVariables` literally
    *inherits* `MSCFModel_ACC::ACCVehicleVariables` rather than declaring its own
    `ACC_ControlMode`/`lastUpdateTime` copies, and CACC's own outer `_v` guard
    (`vars->lastUpdateTime`) and its embedded ACC-fallback call (`acc_CFM._v(veh,...)`, reading
    the SAME `veh`'s SAME inherited field) share that ONE physical field — confirmed by reading
    `createVehicleVariables()`, which initializes `ACC_ControlMode=0`/`lastUpdateTime=0` alongside
    `CACC_ControlMode=0` on a single allocated object. Ported literally: `VehicleRuntime` gained
    only ONE new field, `CaccControlMode` (int, default 0) — the ACC-fallback state REUSES this
    vehicle's own pre-existing `AccControlMode`/`AccLastUpdateTime` (C11-ii) rather than adding a
    redundant, behaviorally-divergent `CaccLastUpdateTime` (a separate field would silently break
    the real cross-call guard interaction: CACC's outer `_v` stamps `lastUpdateTime` to the current
    step BEFORE calling into the ACC fallback in the same call, making the fallback's OWN guard see
    an already-current timestamp and skip its mode rewrite that step — exactly reproduced by
    sharing the field, broken by not sharing it). No collision with an actually-ACC-typed vehicle:
    `CarFollowModel` is fixed at one string per vehicle, so a vehicle is never dispatched through
    both `AccModel.FollowSpeed` and `CaccModel.FollowSpeed`. **EGO-ACCELERATION**: CACC's
    cooperative law reads `veh->getAcceleration()` — the ego's own acceleration from the LAST
    COMPLETED step — ported as a new `VehicleRuntime.Acceleration` (double, default 0), written
    unconditionally in `Engine.ExecuteMoves` right next to the pre-existing `oldSpeed` capture
    (`v.Acceleration = (v.Intent.NewSpeed - oldSpeed) / dt`) and read only by CACC's cooperative
    branch in the FOLLOWING step's Plan phase — consistent with the frozen-start-of-step-snapshot
    invariant, and read-by-nothing-but-CACC for every other vType. Both new state fields are
    threaded `ref`/by-value through `FollowSpeedFor`'s three call sites
    (`LeaderFollowSpeedConstraint`, `ObstacleConstraint`, `JunctionYieldConstraint`/
    `AdaptToJunctionLeader`), which now also pass `hasPred`/`predIsCacc` (the leader's/foe's own
    `VType.CarFollowModel=="CACC"`; always `false` for a B1 `ExternalObstacle`, which has no
    `CarFollowModel` at all). Krauss/IDM/ACC stay byte-identical: the four new `FollowSpeedFor`
    parameters and the `ExecuteMoves` `Acceleration` write are threaded/written unconditionally but
    read ONLY by the CACC arm (103 pre-C11-iii parity tests unchanged, including
    `Rung1`/`Rung9b`/`RungA2`/`RungB1`/`RungC11ParityTests` (IDM)/`RungC11AccParityTests` (ACC) —
    verified). New anchor: `scenarios/24-cacc-carfollow`
    (`tests/Sim.ParityTests/RungC11CaccParityTests.cs`, 70 steps, exact 1e-3) — both vTypes CACC,
    leader maxSpeed=6; follower free-accelerates under speed control, transitions through the
    1.5-2.0s time-gap HYSTERESIS band, then COOPERATIVE gap control (leader IS CACC, so the ACC
    fallback is never actually exercised by this golden — ported and cited but unreached by this
    anchor), settling at the tight CACC cooperative gap. **Deferred**: the
    `CACC_MODE_NO_LEADER`/`CACC_MODE_LEADER_NO_CAV`/`CACC_MODE_LEADER_CAV`
    `CommunicationsOverrideMode` branches (unreachable — no vType/param in this engine's ingest
    ever sets `CACC_CommunicationsOverrideMode` away from its `CACC_NO_OVERRIDE` ctor default),
    the ACC-fallback path end-to-end (ported, but not exercised by any committed golden — needs a
    mixed CACC-follows-non-CACC scenario), and CACC+junction/stop interplay beyond what the ported
    `stopSpeed`/`AdaptToJunctionLeader` plumbing itself guarantees.
  - **C11-iv. DONE. IDMM (IDM with Memory / "Improved IDM").** SUMO builds IDMM from the SAME
    `MSCFModel_IDM` class as plain IDM, just with its `idmm=true` ctor arm
    (`sumo/src/microsim/cfmodels/MSCFModel_IDM.cpp:37-47`): `myAdaptationFactor=1.8`,
    `myAdaptationTime=600` (plain IDM: `1.0`/`0.0`), plus a per-vehicle
    `VehicleVariables::levelOfService` (`.h:189-194`, ctor-defaulted to `1.0`, NOT `0`). Two
    `myAdaptationFactor != 1.` branches only IDMM ever takes: (1) `_v`'s headway adaptation
    (`.cpp:203-207`) — `headwayTime = tau * (myAdaptationFactor + levelOfService*(1-
    myAdaptationFactor))`, i.e. `tau * (1.8 - 0.8*LOS)` for IDMM, used everywhere `_v` used plain
    `tau`; (2) `finalizeSpeed`'s memory update (`.cpp:67-74`) — AFTER the base (unmodified)
    `vNext = MSCFModel::finalizeSpeed(...)`, `levelOfService += (vNext/laneMaxSpeed -
    levelOfService)/600*TS`. Ported WITHOUT touching `IdmModel.cs`'s existing plain-IDM body at
    all (byte-identical proof, not just an argument): `IdmModel.V`/`FollowSpeed`/`FreeSpeed`/
    `StopSpeed` each gained an **optional** `double? headwayTimeOverride = null` parameter —
    inside `V`, `headwayTime = headwayTimeOverride ?? vType.Tau`; every pre-existing IDM/ACC/CACC
    call site passes nothing, so this resolves to the exact literal `vType.Tau` these functions
    always used. `IdmModel.FinalizeSpeed`'s body is untouched (no `ref levelOfService` parameter
    added there at all) — the LOS memory update is applied by the CALLER
    (`Engine.ComputeMoveIntent`'s IDMM dispatch arm) immediately after that call returns
    `newSpeed`, mirroring the vendored source's own sequencing (base finalizeSpeed first, then the
    memory update) without growing the shared function's signature. `GetSecureGap` is
    DELIBERATELY left unadapted (`MSCFModel_IDM::getSecureGap`, `.cpp:190-193`, always uses the
    plain unadapted `myHeadwayTime` member, never `levelOfService`, even for IDMM — ported
    faithfully as-is, not an oversight). New `src/Sim.Core/IdmmModel.cs` holds only the two small
    IDMM-specific pieces (`AdaptationFactor=1.8`/`AdaptationTime=600.0` constants,
    `AdaptedHeadwayTime(tau, los)`, `UpdateLevelOfService(los, vNext, laneMaxSpeed, dt)`) — no
    duplicate `_v`/finalizeSpeed body. **STATE**: `VehicleRuntime.LevelOfService` (double), set to
    `1.0` for EVERY vehicle at creation (`Engine.LoadScenario`, matching the vendored ctor
    default) — harmless/inert for non-IDMM vTypes since only the IDMM dispatch arms below ever
    read or write it, the exact same per-entity plan-phase-mutation pattern C1's `RngState` /
    C11-ii's `AccControlMode` / C11-iii's `CaccControlMode` already establish as parallel-safe.
    **Dispatch** (`Engine.cs`): `FollowSpeedFor`/`StopSpeedFor` gained a `levelOfService`
    parameter (read-only, ONLY consumed by their new `"IDMM"` arms to build
    `IdmmModel.AdaptedHeadwayTime(vType.Tau, levelOfService)` and pass it as
    `headwayTimeOverride`), threaded from every one of their six/three call sites as the ego's own
    `v.LevelOfService`/`ego.LevelOfService`; `FreeFlowDesiredSpeedConstraint` gained its own
    `"IDMM"` arm computing the same override inline (it already has direct `VehicleRuntime`
    access); the `vMinComfortable`/`minNextSpeed` dispatch (`StopLineConstraint`) simply added
    `"IDMM"` alongside `"IDM"` to its existing check — `minNextSpeed` (`.cpp:52-62`) has NO
    `myAdaptationFactor`/`levelOfService` term at all, so IDMM shares the identical
    `IdmModel.MinNextSpeed` call, no override needed; the `FinalizeSpeed` dispatch
    (`ComputeMoveIntent`) added `"IDMM"` alongside `"IDM" or "ACC" or "CACC"` (same
    `IdmModel.FinalizeSpeed` call, byte-identical body) and then, ONLY for `"IDMM"`, updates
    `v.LevelOfService = IdmmModel.UpdateLevelOfService(v.LevelOfService, newSpeed,
    laneVehicleMaxSpeed, dt)` right after. Krauss/IDM/ACC/CACC stay byte-identical: every new
    parameter/field is threaded/written unconditionally but read only by the `"IDMM"` arms (104
    total parity tests, 0 failed — including `Rung1`/`Rung9b`/`RungC11ParityTests` (IDM)/
    `RungC11AccParityTests`/`RungC11CaccParityTests` all unchanged/green — verified). New anchor:
    `scenarios/25-idmm-carfollow` (`tests/Sim.ParityTests/RungC11IdmmParityTests.cs`, 250 steps,
    exact 1e-3) — a LONG 3000m single lane so the follower stays in sustained congestion behind
    the slow (maxSpeed=6) leader long enough for `levelOfService` to actually drift (600s
    time-constant); at LOS=1.0 IDMM==IDM exactly, and as the follower settles into congestion the
    steady-state gap visibly GROWS over the run (~8.81m@t60 → ~9.52m@t249 in the golden), the
    discriminating memory effect this anchor exercises. This completes the IDM/ACC/CACC/IDMM
    car-following-model set (all four now ported).
- **C12. Pedestrians & crossings; public transport.** Pedestrians already appear as junction foes in
  the ported `getLeaderInfo` (the `leader==nullptr` ped branch); add vehicles yielding at crosswalks,
  and bus stops / dwell times / schedules. Breadth; behavioral, with parity where a SUMO analog exists
  (`MSPModel`, `<busStop>`).

**Suggested realism order:** **C1** (unblocks realism) → **C2** (correct multi-lane routing) →
**B5** (external-agent interop, the project's stated direction) → C3/C4 (merges + the rest of RoW) →
C5/C6 (junction blocking + actuated TLs) → C7/C9 (speed spread + cooperative LC) → C8/C11 (integration
+ CF models) → C10 (sublane/continuous → navmesh bridge) → C12 (peds/PT). C1 and C2 are the two
highest-leverage items; B5 is the one that directly serves "lane traffic respects navmesh/RVO agents."

---

## Group D — FastDataPlane ECS readiness (characterized, not yet briefed)

**Goal — READINESS, NOT INTEGRATION (owner's clarification).** Make the engine FDP-*shaped* so it
could later drop into the owner's own ECS library derived from **FastDataPlane**
(`github.com/pjanec/FastDataPlane`, namespaces `Fdp.Core`/`Fdp.ModuleHost`) — WITHOUT adding any
`FastDataPlane` dependency or wiring its network layer now. Everything here is in-house: the
representation refactor (int handles, unmanaged-style component structs, zero-alloc hot path,
command-buffer lifecycle, phased systems, parallelizable) plus thin SEAMS (D7/D9) that make an
`Fdp.Core` backend / info-replication a later drop-in. This is a representation refactor, NOT a
behavior change: the **committed parity/behavioral tests are the oracle** — every D-rung must leave
them byte-identical (`dotnet test` green), exactly like the inert-when-absent discipline used
throughout, and each re-runs the D1 benchmark to record its alloc/throughput delta.

**FDP conventions to target** (from its `Docs/architectural-rules.md` + `USER-GUIDE-OVERVIEW.md`):
- `Entity` = struct handle (`Index`:32 + `Generation`:16). Never store raw ints; gate stored handles
  on `World.IsAlive(e)`.
- Components = **unmanaged value structs** in fixed-size chunks (SoA); NO managed/reference fields.
- Queries = `world.Query().With<A>().With<B>().Build()` + `GetComponentRW<T>`/`GetComponentRO<T>`.
- Systems = phases `SystemPhase.{Input,BeforeSync,Simulation,PostSimulation,Export}`, ordered via
  `[UpdateBefore]`/`[UpdateAfter]`; module systems (`IModule`) run **async** in `Simulation`.
- **Zero-alloc hot path**: no LINQ/`new`/boxing in `OnUpdate`; use `foreach` queries, `stackalloc`,
  `Span<T>`, pre-allocated `NativeArray<T>`. (Spawning is "cold path" — allocation allowed there.)
- Structural changes via `view.GetCommandBuffer()` → `cmd.AddComponent<T>()`/`DestroyEntity()`.
- Determinism via `GlobalTime`/`DeltaTime` (never `DateTime`/`Stopwatch`); `HasAuthority<T>` for split
  authority; static blueprints live in `TkbDatabase`; network via `NetworkEntityMap` /
  `INetworkIdAllocator` / `IDescriptorTranslator`.

**Concept mapping (this engine → FDP):**
| This engine | FDP concept |
|---|---|
| `Kinematics`, `MoveIntent` (already structs) | ECS components (unmanaged) |
| `VehicleRuntime` (class, managed fields) | **must decompose** into component structs + `Entity` |
| `NetworkModel`, `ResolvedVType`, `Route` (immutable) | **TKB descriptors** (static blueprints) |
| seam-4 deferred lane swaps / reroute / insertion | `GetCommandBuffer()` (AddComponent/DestroyEntity) |
| Insert / Emit / Plan / Execute / DecideLaneChanges / UpdateReroutes passes | `SystemPhase` systems |
| frozen-snapshot plan (RO reads, own RW write) | async `Simulation` module systems |
| no `System.Random` / no wallclock (already) | FDP determinism rule (`GlobalTime`) — already satisfied |

**Already aligned (the hard, load-bearing parts):** deferred command-buffer discipline, phase-based
passes, order-independent frozen-snapshot plan/execute, value-type components, immutable blueprints,
and determinism (no RNG/wallclock). **The gap is representation, not architecture.**

- **D1. Many-vehicle benchmark scenario + baseline harness. DONE.** `scenarios/_bench/highway-dense`
  (3-lane, 5 km, 420 vehicles, ~20 % slow so LC/neighbor/reducer hot paths fire; NO golden) +
  `src/Sim.Bench` console harness (steps/sec, alloc/step, GC, peak concurrency, determinism hash — NOT
  in `dotnet test`) + `RungD1BenchmarkDeterminismTests` (a fast offline guard that two dense runs are
  byte-identical). **Baseline (current AoS/LINQ engine, 500 steps, 378 peak concurrent): 1284 steps/s
  (0.779 ms/step), 80.9 MiB total, ~736 B/veh-step, deterministic=True.** See
  `scenarios/_bench/highway-dense/BASELINE.md`. The ~736 B/veh-step is the D2–D4 target; the
  deterministic=True at load is the invariant D8 parallelization relies on. `dotnet test` = 62 green.
- **D2. Int-handle identity (strings → dense indices). DONE.** `Lane` gained a global dense `int
  Handle`; `NetworkModel` exposes `LanesByHandle`/`LaneHandleById` + `ResolveLaneSequenceHandles`;
  `VehicleRuntime` gained `LaneHandle`/`LaneSequenceHandles` (parallel to the string `LaneId`/
  `LaneSequence`, kept for the FCD/obstacle/router boundary); `LaneNeighborQuery` buckets are keyed by
  `int` handle (array-indexed, not string-hashed); per-vehicle hot-path lane lookups use
  `LanesByHandle[handle]`. Pure refactor — trajectory hash UNCHANGED (`909605E965BFFE59` before/after),
  `dotnet test` = 62 green. Alloc drop modest (~0.7 B/veh-step; D2 removes string hashing, not the
  allocations — that's D4) but it's the prerequisite for D3 (unmanaged component `int[]` LaneSequence)
  and D4 (handle-indexed reusable buckets). Benchmark row appended to BASELINE.md.
- **D3. Move managed/variable-length state off the entity. DONE.** The real FDP-readiness gap was the
  three managed collections on the `VehicleRuntime` class; they now live in ENGINE side storage keyed
  by a stable `EntityIndex`: `LaneSequence`(string) + `LaneSequenceHandles`(int[]) → a shared
  `_laneSeqPool` (`List<int>`) with a per-entity `[LaneSeqStart, LaneSeqLen)` slice (blob-style; a
  reroute appends a new slice); `Stops` (`Queue`) → `_stopsByEntity` (populated only for vehicles with
  stops); `AvoidedEdges` (`HashSet`) → `_avoidedByEntity` (lazily, only for rerouted vehicles). The
  entity now holds ONLY unmanaged scalars/structs (`Kinematics`, `MoveIntent`, `int`/`double`/`bool`)
  + the two IMMUTABLE blueprint refs (`Def`, `VType`) — verified: no `Queue`/`HashSet`/`IReadOnlyList`/
  `int[]` field remains. Pure refactor — hash UNCHANGED (`909605E965BFFE59`), `dotnet test` = 62 green
  (incl. the stop + reroute suites that exercise the side tables). Alloc unchanged on the bench
  (no stops/reroute there) — the win is representational (chunk-storable entity). **Deferred to D7's
  store boundary** (kept low-risk here): grouping the flat scalars into sub-structs (`RouteProgressC`/
  `LcStateC`/…) and turning `Def`/`VType` into TKB handles.
- **D4. Allocation-free hot path. DONE.** Reducer `new List<double>{…}.Min()` → running `Math.Min`
  over the same six constraints (same order); `LaneNeighborQuery` from a per-step `Build` factory →
  ONE reused instance with `Refill()` (pre-allocated per-lane buckets `Clear()`ed + refilled in place,
  zero steady-state alloc, both pre- and post-move snapshots); junction `Requests`/`Conflicts`
  `FirstOrDefault` → plain `foreach`; left/right neighbor-lane LINQ scans → O(1) `Lane.LeftNeighbor`/
  `RightNeighbor` handles precomputed at ingest. Pure refactor — hash UNCHANGED (`909605E965BFFE59`),
  `dotnet test` = 62 green. **alloc/veh-step 735.8 → 207.1 B (−71.9%)**, GC gen0 5 → 2. The remaining
  ~207 B is the `TrajectorySet`/FCD emit (a `TrajectoryPoint` + `SortedDictionary` insert per veh-step)
  — an output-contract allocator, out of D4 scope; it moves to a reusable buffer when the emit becomes
  an Export-phase system (D6). Benchmark row in BASELINE.md.
- **D5. Entity handle + command-buffer structural mutations. DONE.** `Entity(int Index, int
  Generation)` (FDP's handle shape; `Generation` reserved at 0) on every `VehicleRuntime`; a reusable
  `CommandBuffer` (`ChangeLane`/`ReplaceRoute`/`Destroy`/`Flush`, zero steady-state alloc). The deferred
  structural mutations now record into the buffer and flush at their existing phase barriers — reroute
  route-replacement (end of `UpdateReroutes`), arrival = `Destroy` (end of `ExecuteMoves`), and the
  speed-gain lane swap (end of the post-move LC phase). The **keep-right** swap was deliberately kept
  INLINE (documented): the speed-gain decision re-reads that same vehicle's lane in the same iteration
  right after it — a genuine same-phase read-after-write a barrier flush can't honor, so correctness
  beats FDP-purity there. Pure refactor — hash UNCHANGED (`909605E965BFFE59`), `dotnet test` = 62 green,
  alloc unchanged (~206 B/veh-step). Index-recycling / generation-bumping deferred to D7's store
  boundary. Benchmark row in BASELINE.md.
- **D6. Restructure the step loop into phased systems over queries. DONE.** `SystemPhase.cs`
  (`Input`/`Simulation`/`PostSimulation`/`Export`) + `VehicleQuery.cs` (`ActiveVehicleQuery`, a
  zero-alloc hand-written struct enumerator yielding `Inserted && !Arrived` vehicles — the FDP
  `Query()` analog). Every hot-path `foreach(_vehicles){if(!Inserted||Arrived)continue;…}` now reads
  `foreach (var v in ActiveVehicles())`. `Run()`'s per-step body keeps its exact order with
  `// [SystemPhase.X]` tags: Input=`InsertDepartingVehicles`,`UpdateReroutes`; Export=`EmitTrajectory`
  (stays between Insert and Simulation — emits the prior step's settled result; the emit-before-plan
  `time+dt` timing is load-bearing, NOT moved); Simulation=`PlanMovements` (RO frozen-snapshot reads,
  own-`MoveIntent` writes); PostSimulation=`ExecuteMoves`,`DecideSpeedGainChanges`. Pure refactor —
  hash UNCHANGED (`909605E965BFFE59`), `dotnet test` = 62 green, query is zero-alloc (bench 205.9 B/
  veh-step, no GC increase). The `TrajectorySet` emit alloc is left for D9's export seam. Baseline row
  appended.
- **D7. The FDP-shaped seam / adapter (READINESS — NO `Fdp.Core` dependency). DONE.**
  `ICommandBuffer` (`ChangeLane`/`ReplaceRoute`/`Destroy`/`Flush`, the FDP `view.
  GetCommandBuffer()` -> deferred `AddComponent`/`DestroyEntity` analog) — D5's `CommandBuffer`
  now `: ICommandBuffer`, same four method bodies, unchanged. `IWorld` (`GetCommandBuffer()` +
  `ActiveVehicles()`) — the FDP `World`/`View` surface scoped to what this engine needs: a way to
  reach the command buffer and a way to reach the "active, on-road vehicle" query (D6's
  `Query()` analog). `IQuery` is deliberately NOT a separate interface: `IWorld.ActiveVehicles()`
  returns the concrete `ActiveVehicleQuery` struct BY VALUE (a factory method), not an `IQuery`/
  `IEnumerable<VehicleRuntime>` — FDP's own query surface is struct-based for the same reason
  (boxing the enumerator behind an interface would allocate every vehicle, every step, undoing
  D4's alloc work). One in-house backend, `World` (`src/Sim.Core/World.cs`), wraps the SAME
  `List<VehicleRuntime>`/`CommandBuffer` instance `Engine` already owned (D3–D6) — no state moved,
  no computation added. `Engine` now holds `_world` (`IWorld`) and `_commandBuffer`
  (`ICommandBuffer`, cached once from `_world.GetCommandBuffer()` in a new constructor so every
  EXISTING `_commandBuffer.X(...)` call site is untouched); `ActiveVehicles()` now reads
  `_world.ActiveVehicles()` instead of constructing `new(_vehicles)` directly. New files:
  `ICommandBuffer.cs`, `IWorld.cs`, `World.cs`. Pure representation refactor — hash UNCHANGED
  (`909605E965BFFE59` in both single-threaded `hashA` and parallel `hashPar`), `dotnet test` = 63
  green, alloc/veh-step unchanged (206.1 B single / 214.4 B parallel, matching D6/D8's
  205.9 B/~206–215 B range — no boxing, no new per-step allocation). This is the drop-in point —
  an `Fdp.Core`-backed `IWorld` implementation could be added LATER by the owner without touching
  any system in `Engine.cs`, but this project does NOT add the `FastDataPlane` reference or write
  that backend. Benchmark row in BASELINE.md. (When the owner later wires `Fdp.Core`, read
  `Engine/Fdp.ModuleHost` + `Engine/Examples` for the exact `IModule`/registration signatures.)
- **D8. Parallelize the Simulation phase. DONE.** `Engine.UseParallelPlan` (default `false`,
  opt-in): when `true`, `PlanMovements` iterates `_vehicles` via `System.Threading.Tasks.
  Parallel.For(0, _vehicles.Count, i => {...})` (guarded per-index by the same `Inserted &&
  !Arrived` predicate `ActiveVehicleQuery` applies) instead of the sequential `foreach (var v in
  ActiveVehicles())`. `ComputeMoveIntent`'s entire call tree (`LeaderFollowSpeedConstraint`,
  `StopLineConstraint`, `RedLightConstraint`, `JunctionYieldConstraint`/
  `AdaptToJunctionLeader`/`FindFoeVehicle`, `ObstacleConstraint`, `ProcessNextStop`) was verified
  to read ONLY start-of-step state (this vehicle's own `Kinematics`/lane/vType/stop-queue-front,
  the frozen pre-move `LaneNeighborQuery` snapshot Refilled once before `PlanMovements` runs, and
  the immutable network/config/obstacle/lane-sequence-pool/stop/avoided-edge side storage) and
  write ONLY `v.Intent` — no shared mutable accumulator, no lock, no cross-entity write anywhere
  in that call tree — so concurrent per-vehicle iteration is race-free by construction (see
  `UseParallelPlan`'s own header comment in `Engine.cs` for the full argument). `ExecuteMoves` and
  the post-move LC phase (`DecideSpeedGainChanges`, which has a genuine intra-phase
  read-after-write via the inline keep-right swap) are deliberately left sequential, per the
  briefing. New test `RungD8ParallelDeterminismTests` runs `scenarios/_bench/highway-dense` for
  120 steps with `UseParallelPlan=false` and again with `UseParallelPlan=true` and asserts the
  trajectory hashes are IDENTICAL and peak concurrent >= 50. `Sim.Bench` now runs both modes and
  reports each — the 500-step hash is `909605E965BFFE59` in BOTH modes, every run captured;
  measured speedup was small and noisy (1.01x–1.06x) on this shared 4-core VM (`PlanMovements` is
  only one of five per-step phases, and the workload is cheap enough post-D4 that `Parallel.For`'s
  own scheduling overhead competes with the parallelism dividend) — the byte-identical hash, not
  the speedup, is this rung's point. `dotnet test` = 63 green (62 + this rung's new test).
  Benchmark row appended to `scenarios/_bench/highway-dense/BASELINE.md`.
- **D9. Info/replication export SEAM (READINESS — NO FDP network wiring). DONE.**
  `VehicleExportSnapshot` (`src/Sim.Core/VehicleExportSnapshot.cs`, a `readonly struct`) — the
  "ECS component → external descriptor" SOURCE shape FDP's `IDescriptorTranslator` consumes:
  D5's `Entity` handle (the id a translator would key its external descriptor off) +
  `EntityIndex` (the plain side-table key) + the same `VehicleId`/`Time`/`Lane`/`Pos`/`Speed`/
  `X`/`Y`/`Angle` fields `TrajectoryPoint` already carries. `ISimExportObserver`
  (`src/Sim.Core/ISimExportObserver.cs`) — the observer seam a later `IDescriptorTranslator`-
  style consumer would implement: `OnVehicleExported(in VehicleExportSnapshot snapshot)` (passed
  `in`, never by value/boxed) plus optional no-op-by-default `OnFrameBegin(double time)`/
  `OnFrameEnd(double time)` bracket hooks. `Engine.AddExportObserver(ISimExportObserver)`
  registers into a new `_exportObservers` list, empty by default. `EmitTrajectory` (the
  `[SystemPhase.Export]` system) now builds ONE `VehicleExportSnapshot` per active vehicle per
  frame and (a) produces the exact same `TrajectoryPoint`/`trajectory.Add(...)` from it — same
  one `LaneGeometry.PositionAtOffset` call, same fields, same order, same null `Acceleration` —
  and (b) notifies every registered observer with that same snapshot (`in snapshot`); the
  `TrajectorySet` is the engine's own default, always-present consumer of the snapshot. With
  ZERO observers registered (the default — no existing scenario/test/benchmark calls
  `AddExportObserver`), the notify loop and the frame-bracket loops are empty `foreach`es over
  an empty list: no virtual call, no allocation, byte-identical to the pre-D9 `EmitTrajectory`
  body. New test `RungD9ExportObserverTests` registers an in-house recording
  `ISimExportObserver` on `scenarios/_bench/highway-dense` (120 steps, peak concurrent ≥ 50) and
  asserts the observer's (VehicleId, Time) set EQUALS the returned `TrajectorySet`'s (no
  vehicle/frame missing or extra) and every observed Lane/Pos/Speed/X/Y/Angle matches exactly —
  the "faithful mirror" property test the briefing asks for. No `FastDataPlane`/`Fdp.Core`
  reference, no `NetworkEntityMap`/`INetworkIdAllocator`, no network transport added anywhere —
  READINESS ONLY. `dotnet test` = 64 green (63 + this rung's new test). Trajectory hash
  UNCHANGED (`909605E965BFFE59` in both `hashA`/`hashPar`), alloc/veh-step unchanged (206.1 B
  single / 214.3–214.4 B parallel, matching D7's own 206.1 B/214.4 B exactly). Benchmark row
  appended to `scenarios/_bench/highway-dense/BASELINE.md`. Depends on D3/D7.

**Suggested Group-D order:** **D1** ✅ (measure) → **D2** ✅ (int handles) → **D3** (component structs +
move managed state out) → **D4** (zero-alloc hot path) → **D5** (entity lifecycle via command buffer)
→ **D6** (phased systems over queries) → **D7** (in-house FDP-shaped adapter seam) → **D8**
(parallelize) → **D9** (export seam). All in-house — READINESS, not integration: no `FastDataPlane`
dependency is added (D7/D9 are the drop-in seams the owner wires to `Fdp.Core` later). D2/D3 are the
load-bearing enablers; D4 is the biggest measurable alloc win; D8 proves the parallel payoff. Every
rung keeps the tests byte-identical — the refactor changes representation and speed, never behavior.

---

## Group E — live-traffic features (opposite-overtake + flow demand + warm-start)

Three user-requested live-traffic features, all landed on `main` and green, all **inert-when-absent**
(gated on a master switch so every deterministic parity scenario stays byte-identical / bench hash
`909605E965BFFE59` unchanged). Each is Group-B-style (behavioral/property tests, no golden-FCD) except
where a network golden is called out. Full per-rung diagnosis lives in the referenced markdowns.

### Landed (each parity-reviewer ACCEPT)

- **Opposite-direction overtaking (OV1–OV4)** — gated on `_anyLcOpposite` (`lcOpposite` vType flag);
  scenario `scenarios/57-overtake-opposite/` (hand-written two-way `bidi` net, no golden).
  - **OV1** detection (held up behind a slow leader, oncoming bidi lane clear ahead →
    `VehicleRuntime.OvertakeActive`), **OV2** closing-speed gap acceptance (refuse if the head-on
    closes before the pass could finish), **OV3** execution (lateral spill via B6 `DriftToward` + the
    ER5 `!FootprintsOverlap` leader bypass, then recenter), **OV3b** adversarial abort-mid-spill safety
    test, **OV4** cooperative oncoming shift (an oncoming driver seeing a spilled overtaker pulls to
    its own outer edge — mirror of the ER3/ER5 give-way drift). Tests
    `RungOV1..OV4*Tests`. **Diagnosis + deferred items: `OV-REMAINING.md`.**
- **Probabilistic / flow demand insertion (F1, F2a, F2b)** — gated inert (no `<flow>` / no
  `probability=` → zero work).
  - **F1** deterministic `<flow>` expansion at ingest (period / vehsPerHour / number; exact-parity,
    offline; scenario `scenarios/56-flow-equiv/`). **F2a** runtime `<flow probability=>` insertion via
    a per-flow seeded `VehicleRng` (SplitMix64, never `System.Random`) — deterministic and
    reproducible. **F2b** captures the per-flow RNG state + arrival counter + runtime-generated
    vehicles in the file snapshot so save→load→continue is point-for-point identical. Tests
    `RungF1FlowEquivalenceTests`, `RungF2*Tests`.
- **Warm-start snapshot (W1, W2)** — `WarmUp(steps)` deterministically pre-populates a live network
  in memory; `SaveSnapshot`/`LoadSnapshot` round-trips ALL vehicles including trains (rail developed
  in a separate session, on `main`) plus the engine-level rail-crossing / clock state. Guards throw
  `NotSupportedException` for the not-yet-captured cases (stops, reroute, actuated TLS). Tests
  `RungW1WarmUpTests`, `RungW2SnapshotTests`.
- **[net] SUMO cross-checks (T1, T2)** — the network-golden parity checks, DONE (SUMO 1.20.0 runs in
  this env). **T1** ensemble statistical parity of the `<flow probability=>` INSERTION-COUNT
  distribution vs SUMO (`scenarios/58-flow-probability`, 50-seed committed `golden.ensemble`,
  `RungT1ProbabilisticFlowEnsembleTests`; engine mean 8.42/std 2.16 vs SUMO 8.68/2.36 — confirms our
  probabilistic insertion + entry-lane gating already match SUMO; the earlier "2.6× gap" was a
  measurement artifact, completed-trips vs insertions). **T2** `SaveSnapshot` vs SUMO `--save-state`
  mid-run cross-check (`golden.state.mid.xml` for `12-overtake` t=15 and `22-idm-carfollow` t=30,
  `RungT2SnapshotStateParityTests`; pos/speed/posLat agree within 1e-2 — the state-boundary hardening
  on top of the in-engine round-trip determinism). Both no engine change (bench hash unchanged).

### Remaining (deferred, with diagnosis in the markdowns — do these next)

- **[overtake] OV deferred — see `OV-REMAINING.md`.** Three coupled items, none offline-vacuous alone:
  **D1** the cross-lane hard-brake backstop (prototyped, REVERTED — never binds under conservative
  OV2, so it has no non-vacuous test until OV2 can commit optimistically); **D2** the OV3 return-gap
  enforcement bug (the return recenters the instant ego nudges ahead of the passed leader, without a
  safe re-entry gap — needs a small per-ego "remember the passed leader until the gap is safe" state);
  **D3** the coupled OV2/OV4 decision that would enable a genuine reduced-clearance side-by-side pass
  (OV2 must account for the oncoming's cooperative shift; re-add D1 together with this, as D1 would
  then bind). Each needs a fixture that forces the currently-vacuous path to bind.
- **[overtake] OV follow-ups (D2, D3) — DONE; D1 confirmed unnecessary — see `OV-REMAINING.md`.**
  **D2** OV3 return-gap enforcement (overtaker stays spilled until a safe re-entry gap ahead of the
  passed leader; `RungD2ReturnGapTests`). **D3** coupled OV2/OV4 cooperative side-by-side pass on a
  wide-enough lane (`scenarios/59-overtake-cooperative`, corridor predicate self-gates so scenario 57
  is byte-identical; `RungD3CooperativeOvertakeTests`). **D1** (explicit cross-lane brake) was
  re-investigated under D3's optimistic commitment and STILL does not bind — two overtakers spilling
  head-on both abort and pass safely (the one-step abort is the safety response), so it stays unbuilt
  rather than add speculative untestable code. Both engine changes: bench hash unchanged, inert for
  non-lcOpposite, parity-reviewer ACCEPT.

Group-E is COMPLETE — no remaining items. The `[net]` tail (T1/T2) is DONE (see the landed list above
and `TAIL-NETWORK-REMAINING.md`).

**Group-E order (done this session, in order):** F1 → W1 → W2 → OV1 → OV2 → OV3 → OV3b → OV4 → F2a →
F2b → T1 → T2 → D2 → D3 (with D1 confirmed unnecessary).

---

## Perf: multi-core scaling — on-target (Windows 16-core) — see `PERF-HANDOVER.md`

This session (on a 4-core VM) landed the **willPass/plan fusion** + **parallel Export phase**
(both byte-identical, hash `909605E965BFFE59`) and the **measurement toolkit** (`--serial`,
`--max-parallelism`, `--no-fcd`, `--profile`, `scripts/bench-scaling.ps1`). Serial city-3000
dropped ~54s→~36s (VM); ~2.15× on 4 cores.

The 24-thread scaling curve (measured on target) plateaus at **~2×**; the `--profile` breakdown
shows ~88% of work is in the *already-parallel* plan/willPass/emit phases, so the ceiling is the
**memory-bound parallel phases not scaling**, not a serial tail. Next steps (confirm GC vs bandwidth,
then dealloc / partitioner / dense-active / **SoA of hot per-vehicle fields**, and the multilane `-L2`
route-resolution generalization) are handed off to a Claude Code session running **on the target
hardware** — full plan, ranked backlog, and the parity gates are in `PERF-HANDOVER.md`.
