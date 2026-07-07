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
- **A3. Priority / emergency vehicles.** Depends on A1 (emergency is a vClass) AND on 9b + A2 (the
  yield/foe + LC-safety machinery). Two behaviors: (i) the emergency vehicle's own privileges
  (`jmIgnoreFoeProb`, ignoring red/foes — `MSVehicle::ignoreFoe`/`ignoreRed`); (ii) OTHER vehicles
  giving way (moving aside / not blocking). This is an advanced layer built on the junction + LC foe
  logic; do it only after 9b and A2 exist. Note: "priority *road*" right-of-way (major vs minor at a
  junction) is a different thing and is rung 9b, not this.

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
- **B4. U-turn when no route around exists.** When B3 finds no alternate route (dead-end / fully
  blocked), the vehicle reverses direction — a new structural maneuver (turn onto the opposing edge
  if one exists, else stop). This is the most exotic: it needs the opposing edge / bidi-lane
  topology and a reversal move (SUMO has `<neigh>`/opposite-direction driving and `--allow-uturn`;
  ref `MSVehicle::checkReversal` and opposite-lane handling — but faithful parity here is a stretch,
  so scope it as behavior-first). Do last; depends on B2/B3.

**Suggested order overall:** A1 (quick, unblocks much) → 9b (RUNG9B.md) → A2 → B1 → B2 → B3 → A3 →
B4. A1 and B1 are the two cheapest high-value items and are independent of the hard 9b work.
