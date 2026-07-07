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
  `dotnet test` = 61. Gated ACCEPT. **DEFERRED (each its own future rung + scenario):** the `!canBrake`
  ignore-red arm; **ignore-FOE** at junctions — note `jmIgnoreJunctionFoeProb` only bypasses the
  on-junction link-leader path (`checkLinkLeader`), NOT 9b's `opened()`/approaching stop-line yield,
  so a real "emergency ignores a priority foe" needs the `opened()`-level ignore too; and behavior
  **(ii) give-way** (other vehicles moving aside — the LC blue-light layer). "priority *road*"
  right-of-way is rung 9b, not this.

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

- **B5. Moving external agents as dynamic obstacles / foes (generalizes B1).** B1 already lets SUMO
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

### Group C — realism beyond the deterministic phase-1 core

- **C1. Statistical parity / driver imperfection (`sigma>0`). THE determinism-ladder shift; do
  first — unblocks most of the rest.** Port Krauss dawdling (`MSCFModel_Krauss::dawdle`) and the
  per-vehicle SEEDED RNG (CLAUDE.md rule: no `System.Random`; seed per entity so results are
  independent of thread order). Add a **statistical** `parityMode` to the harness: either reproduce
  SUMO's RNG stream for trajectory-exact parity (hard — needs SUMO's `RandHelper`/per-vehicle seeding
  reproduced), or, more realistically, an **ensemble/aggregate** tolerance (mean + spread of
  speed/flow over N seeds, or the fundamental diagram). Produces stop-and-go waves and realistic
  capacity. Prereq for C7 and for believable everything.
- **C2. Strategic (route-driven) lane changes + lane-to-lane continuity. The #1 lane-based realism
  gap.** Today a vehicle can sit in a lane that cannot reach its route. Port LC2013's STRATEGIC block
  (`LCA_STRATEGIC`/`LCA_URGENT`, `getBestLanes`/`bestLaneOffset` — `MSLCM_LC2013::_wantsChange` +
  `MSLane::getBestLanes`) so a vehicle moves into a lane that continues its route. Requires
  **lane-level** routing: honor `<connection fromLane/toLane>` turn permissions (B2 is edge-level).
  Parity axis. Scenario: a multi-lane approach where a vehicle must reach a dedicated turn lane before
  the junction. Reuses A2's neighbor query + the post-move LC phase.
- **C3. Merging / on-ramp / zipper.** Gap-acceptance merging where two lanes join (`sameTarget`
  links). Extends 9b's foe machinery + A2's neighbor leader/follower. Parity axis. Scenario: an
  on-ramp merging onto a mainline; the ramp vehicle accepts a gap or waits.
- **C4. Remaining right-of-way: right-before-left, roundabouts, stop signs.** 9b did PRIORITY
  junctions only. Right-before-left (uncontrolled symmetric), roundabout yielding (+
  `myRoundaboutBonus`/cooperative), all-way-stop (`LINKSTATE_ALLWAY_STOP`). Reuses 9b's `<request>`
  response/foe matrix + `opened()`. Parity axis. One scenario per RoW type.
- **C5. Junction-blocking avoidance (`keepClear` / don't-block-the-box).** `MSLink::keepClear` + jam
  detection so a vehicle does not enter a junction it cannot clear. Prevents artificial gridlock /
  spillback across intersections. Parity axis; also a property test (junction never deadlocks).
- **C6. Actuated / adaptive traffic lights + yellow decision.** Rung 10 did STATIC `tlLogic` only.
  Add actuated/`delay_based` programs (`MSActuatedTrafficLightLogic` — gap-based phase extension) and
  the yellow "stop if you can brake, else go" decision (the rung-10 deferred note; `MSLink::haveYellow`
  + the `canBrake` branch). Parity axis (actuated needs detector state).
- **C7. `speedFactor` distribution (heterogeneous desired speeds).** Per-vehicle desired-speed
  variation (`speedFactor` = `normc(1.0, dev)`, `default.speeddev`); today everyone wants exactly the
  limit (mean 1.0, dev forced 0). Depends on C1 (seeded RNG). Statistical parity. Produces realistic
  speed spread and overtaking pressure.
- **C8. Ballistic integration + `actionStepLength > 1` (reaction time).** SUMO's ballistic update
  (more accurate than Euler) and sub-second/multi-second reaction time. The integration method is
  already a config flag (DESIGN.md seam); this ports the ballistic `finalizeSpeed`/position update and
  the action-step sub-sampling. Parity axis — effectively a config variant of every scenario, so it
  needs its own goldens (ballistic on).
- **C9. Cooperative lane changes.** LC2013's COOPERATIVE block (`LCA_COOPERATIVE` — make room for a
  blocked/merging neighbor). Depends on A2's neighbor query + C3 (merging pressure). Parity axis.
- **C10. Sublane / continuous lateral (SL2015). The lateral axis and the BRIDGE to navmesh/RVO.**
  Continuous lateral position (`minGapLat`, lateral speed, `latAlignment`), movement within and across
  lanes without discrete index snaps (`lanechange.duration>0` is the first step). Seam 2 (the lateral
  field, always-written-0 today) and seam 1 (neighbor query → spatial hash) were built precisely for
  this. Where it leaves SUMO's lane model it moves to a **behavioral** bar — and it is the natural
  meeting point with the navmesh/RVO continuous-movement layer (B4's U-turn, free-form avoidance).
  Large; its own phase. Ref `MSLCM_SL2015`, `MSLaneChangerSublane`.
- **C11. Alternative car-following models (IDM, ACC/CACC).** `MSCFModel_IDM`, `MSCFModel_ACC`,
  `MSCFModel_CACC` for modern / automated traffic. Each is a resolver dispatch (`carFollowModel`
  attribute) + a model port behind the same `KraussModel`-style constraint interface. Parity axis, one
  scenario per model.
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

**Goal.** Make the engine integrate cleanly into the project owner's own ECS library derived from
**FastDataPlane** (`github.com/pjanec/FastDataPlane`, namespaces `Fdp.Core`/`Fdp.ModuleHost`), so a
later **info/replication system** can consume the simulation over FDP's network layer. This is a
representation refactor, NOT a behavior change: the **61 committed parity/behavioral tests are the
oracle** — every D-rung must leave them byte-identical (`dotnet test` green), exactly like the
inert-when-absent discipline used throughout.

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
- **D2. Int-handle identity (strings → dense indices). The biggest single enabler.** FDP components are
  unmanaged — string ids cannot be component fields. Intern lane/edge/junction/vType ids to dense
  `int` handles at ingest; `NetworkModel` exposes arrays indexed by handle (it already stores arrays);
  `LaneSequence` becomes `int[]` (lane handles). Keep a handle↔string map for I/O only (FCD emit,
  scenario load). Prereq for D3/D4. Pure refactor; parity tests unchanged.
- **D3. Decompose `VehicleRuntime` into FDP-style component structs + move managed state out.** Split
  the class into unmanaged value-type components: `KinematicsC` (=today's `Kinematics`), `IntentC`
  (=`MoveIntent`), `RouteProgressC {laneSeqStart, laneSeqLen, laneSeqIndex}`, `LcStateC {keepRightP,
  speedGainP}`, `RerouteStateC {blockedSeconds}`, `StopStateC`, `VTypeRef {vTypeHandle}`. The
  **managed/variable-length** fields are the real work: `LaneSequence` → a shared `int[]` pool with a
  per-entity `[start,len]` slice (blob-style); `Stops` (`Queue`) → a fixed-capacity inline buffer or a
  side "buffer component"; `AvoidedEdges` (`HashSet<string>`) → a small fixed-cap set or a bitset over
  edge handles. Provide a component-container abstraction now (see D7) so this can back onto either an
  in-house SoA store or FDP `World` later. Parity tests are the safety net.
- **D4. Allocation-free hot path (satisfy FDP's zero-alloc `OnUpdate` rule).** Remove per-step / per-
  entity heap allocations and LINQ from the plan/execute/decide passes: the reducer's
  `new List<double>` → a running `Math.Min` over an inline set of constraints; `LaneNeighborQuery`'s
  per-step `Dictionary<string,List<>>` (built TWICE/step) → reused pre-allocated per-lane buckets or a
  spatial hash behind the seam-1 interface; drop `.First`/`.FirstOrDefault`/`.OrderBy`/`.Min`/`.Sum`
  from per-entity code (`Engine.cs` junction/LC/reroute paths). Verify with D1's allocations/step
  metric (target: 0 in steady state). Parity unchanged.
- **D5. Entity lifecycle via the command buffer.** Route insertion (`CreateEntity` + `AddComponent`s)
  and arrival (`DestroyEntity`) through a deferred command buffer applied at a barrier point, matching
  FDP's `GetCommandBuffer()`. Generalize the existing seam-4 buffer (lane swaps/reroute) to entity
  create/destroy + component add/remove. Use generation-checked handles; gate stored handles on an
  `IsAlive`-equivalent. Parity unchanged (insertion order/timing must match — the FIFO gap-gate stays).
- **D6. Restructure the step loop into phased systems over queries.** Recast Insert/Emit/Plan(CF +
  multi-constraint reducer)/Execute/DecideLaneChanges/UpdateReroutes as ordered systems mapped to
  `SystemPhase` (Input=insert/emit, Simulation=plan+decide, PostSimulation=execute/structural,
  Export=FCD emit), each iterating a `Query().With<…>()`. Preserve the plan/execute contract (plan =
  RO reads + own RW; structural = command buffer) — it maps 1:1 onto FDP async `Simulation` modules.
  Parity unchanged; this is the shape D7 plugs FDP into.
- **D7. The FDP seam / adapter (the actual integration point).** Introduce a thin `IWorld`/`IQuery`/
  `ICommandBuffer` abstraction the engine's systems target, with TWO backends: (a) the in-house SoA
  store built in D3–D6, and (b) an **`Fdp.Core`-backed** implementation (components as FDP structs in
  its chunks, systems as `IModule` in `SystemPhase.Simulation`, blueprints as `TkbDatabase`
  descriptors, time via `GlobalTime`). Read `Engine/Fdp.ModuleHost` + `Engine/Examples` for the exact
  `IModule`/system-registration signatures when building this (the overview doc doesn't pin them). Add
  `Fdp.Core` as a dependency here. Parity tests run against BOTH backends → proof of equivalence.
- **D8. Parallelize the Simulation phase.** With D2–D6 done, run the plan/decide systems concurrently
  (in-house: `Parallel.For`; FDP: async `IModule` in `Simulation`). The frozen-snapshot + order-
  independent-decision discipline already guarantees byte-identical output — this rung *proves* it
  (same trajectory single- vs multi-threaded) and reports the throughput delta on D1's benchmark.
- **D9. Info/replication readiness (the end goal: "integrate an info system using FastDataPlane").**
  Expose vehicle/network state as **networked components** via `IDescriptorTranslator`
  (ECS component ↔ network descriptor) + `NetworkEntityMap`/`INetworkIdAllocator`, and honor
  `HasAuthority<T>` so an external info system can observe/replicate the running simulation over FDP's
  network layer. Behavioral/property validation (a subscriber sees a faithful mirror). Depends on D7.

**Suggested Group-D order:** **D1** (measure) → **D2** (int handles) → **D3** (component structs +
move managed state out) → **D4** (zero-alloc hot path) → **D5** (entity lifecycle via command buffer)
→ **D6** (phased systems over queries) → **D7** (FDP adapter + `Fdp.Core` backend) → **D8**
(parallelize) → **D9** (network/info-system). D2 and D3 are the load-bearing enablers; D7 is where
FastDataPlane actually plugs in. Every rung keeps the 61 tests byte-identical — the refactor changes
representation and speed, never behavior.
