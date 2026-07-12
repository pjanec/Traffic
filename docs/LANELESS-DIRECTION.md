# LANELESS-DIRECTION.md — the validation bar + model for the laneless axis

Status: **direction decided (owner, this session)**. This note scopes a deliberate **bar change for
the laneless axis only**. It does not touch the phase-1 lane-based core, whose byte-exact SUMO
parity discipline (`CLAUDE.md`) stays sacred and unchanged.

## The decision, in one line

For laneless/heterogeneous traffic: **keep SUMO's per-step lateral *physics* as the reference, but
validate behaviorally + statistically — not byte-exact — and build the lateral layer as a continuous
footprint / velocity-obstacle (RVO/ORCA) model that unifies SUMO vehicles with external navmesh/RVO
agents.**

## Why (the evidence from the sublane byte-exact attempt)

Porting SUMO's sublane model (`MSLCM_SL2015`) to byte-exact parity split cleanly (see
`docs/PHASE2-SUBLANE.md`):

- **Cheap & achieved exact:** the per-step *physics/geometry* — single-vehicle drift
  (`scenarios/60`), side-by-side coexistence (`61`), `departPosLat` placement, following-parity
  (`62`), and even the exact `keepLatGap` magnitude (`−0.3015`, grid-quantized minGapLat).
- **Expensive & the wall:** the dynamic multi-vehicle *decisions* — overtake timing and the gap-keep
  **hold duration** — because those are governed by SUMO's **persistent cross-step lateral state
  machine** (`mySafeLatDist*` + `updateExpectedSublaneSpeeds` + push-based `inform*`). Two faithful
  ports each nailed the *magnitude* and missed the *duration*: exactness is **emergent from the state
  machine**, not any single formula.

Two structural facts make byte-exact the *wrong* bar here specifically:

1. **SUMO's sublane model is itself an approximation of laneless traffic** — lane-anchored,
   sublane-discretized. Byte-matching it means exactly reproducing an approximation.
2. **Its exact-timing machinery is ECS-hostile** — persistent per-vehicle lateral state and
   push-based neighbor signaling fight the plan/execute double buffer (the same push→pull conflict
   B6 hit with `MSDevice_Bluelight`). We'd pay the highest cost to import the least parallel-friendly
   part of SUMO, to reproduce an approximation of the behavior we actually want.

The lane-based core was different: there, byte-exact parity *is* the value (a validated, unambiguous
reproduction of SUMO's 20-year-tuned car-following). That stays. The laneless axis has looser
real-world requirements (the owner's stated bar) and a continuous target behavior, so it earns a
different, cheaper, better-fitting bar.

## The bar: behavioral + statistical, SUMO-grounded

This is not "no validation" — it reuses tools the project already has:

- **Byte-exact anchors (keep):** the tractable static/single-vehicle sublane rungs (`60/61/62`) stay
  as golden tests. They pin the physics constants and prove sublane mode does not perturb
  car-following. These are cheap and worth keeping exact.
- **Behavioral property tests (the Group-B bar, already in use):** no-overlap (footprints never
  intersect), overtake-completes, lateral-gap-maintained, resumes-when-clear, never-leaves-the-road.
  This is exactly how the external-agent behaviors (B1/B5/B6) are already validated.
- **Statistical / aggregate parity vs SUMO (already supported):** `tolerance.json` carries
  `parityMode="statistical"`; compare distributions (lateral position spread, headway, throughput,
  mean speed, fundamental diagram) over an ensemble, within a statistical tolerance — *not* per-step
  position. Use it for the lane-following regime (where good continuous avoidance should reduce to
  car-following) and for aggregate flow on mixed scenarios.

Rule of thumb: **exact where a SUMO golden is cheap and diagnostic; behavioral where the behavior is
continuous/heterogeneous; statistical where we want SUMO-comparability without per-step matching.**

## The model: continuous footprint / velocity-obstacle (RVO/ORCA)

Represent every mover — SUMO-derived vehicle *and* external navmesh/RVO agent — as a **footprint
agent** (position, velocity, footprint) and pick each agent's next velocity to avoid its
near-neighbors' velocity obstacles. Why this fits:

- **ECS-native & embarrassingly parallel.** Each agent independently reads its near-neighbors
  (frozen start-of-step) and writes only its own velocity/`MoveIntent` — the plan phase exactly, with
  **no persistent push-state** (the thing that made SUMO's sublane ECS-hostile).
- **Reuses what exists:** footprints (`ExternalObstacle.Width/LatPos`, vType `Width`), the spatial-
  hash groundwork (`_packed`/`HotVeh`, `SPATIAL-OPT.md`), the multi-constraint reducer, `DriftToward`,
  `FootprintsOverlap`, and the external-obstacle input surface (`EXTERNAL-AGENTS-VIZ.md`).
- **Unifies the two worlds for free.** "SUMO traffic respects non-SUMO navmesh/RVO agents" (a stated
  project goal, and why B4 u-turn was deferred *to* this layer) becomes trivial: both are the same
  kind of footprint agent doing velocity-obstacle avoidance. No special integration seam.
- **Better for the real target.** Dense heterogeneous/mixed traffic is what continuous avoidance
  handles well and lane-anchored sublane handles poorly.

SUMO stays the *physics reference*: the avoidance keeps SUMO's `minGapLat` lateral clearance and
`maxSpeedLat` lateral-speed bound; the longitudinal stays the validated Krauss car-following (the
lane-following regime should emerge as RVO reducing to car-following when laterally boxed in).

## Staged plan (each stage independently verifiable, all opt-in / byte-identical when off)

1. **RVO-lite lateral PoC (this session):** an opt-in `Engine.LanelessRvo` flag; a continuous
   lateral avoidance over near-neighbors (vehicles + external obstacles, treated uniformly) that
   produces an **emergent overtake** (drift to clear a slower leader, accelerate past via the
   existing `FootprintsOverlap` bypass, recenter). Validated **behaviorally** (no-overlap, passes,
   recenters). Off by default → phase-1 and the exact sublane rungs (`60/61/62`) untouched.
2. **Fixed-radius query + feasible-interval solve (DONE, 2b-i + 2b-ii).**
   - **2b-i:** the RVO solve gathers all near footprint agents (vehicles + external) within a radius
     from ONE query (Seam 1's phase-2 form) — the per-lane Pos-sorted bucket is the spatial index.
   - **2b-ii:** the lateral reduction is a **1D feasible-interval (half-plane-intersection) solve**:
     each coupled neighbour forbids a lateral band `(lat ± (halfWidths + minGapLat))`; ego drifts to
     the feasible point closest to its line (keep-right tie), or brakes via car-following if the lane
     is fully blocked. This **correctly resolves CONFLICTING neighbours** (a plain push-sum can strand
     ego between two that push opposite ways — the conflicting-neighbours test proves the fix). It is
     effectively **one-sided** (the manoeuvring vehicle clears; the other holds its line).

   **Two-regime insight (important):** *full holonomic 2D ORCA is the wrong model for lane-derived
   vehicles* — they are elongated (not disc-like) and quasi-1D longitudinally, where the validated
   Krauss car-following already handles the longitudinal "velocity obstacle" (braking). So lane
   vehicles use **lateral feasible-interval + car-following longitudinally**. Full reciprocal 2D ORCA
   (disc agents sharing avoidance, both axes) is the **open-space navmesh/RVO layer** for the external
   crowd/holonomic agents. The two regimes are unified by the shared `RvoNeighbor` footprint
   abstraction (and the `Share`/avoidance-class field, reserved for the open-space reciprocal solve),
   not by forcing one avoidance model onto both.
3. **External-agent unification:** route navmesh/RVO agents through the same ORCA solve as vehicles.
   **API note (owner):** the current external-obstacle surface (`ExternalObstacle`, string-`Id`-keyed
   dictionary in `EXTERNAL-AGENTS-VIZ.md`) will be **replaced** — it does not scale (string ids, per-
   obstacle record, dictionary lookup). The unification targets a **scalable int-indexed footprint-
   agent store** (array/SoA, entity-handle keyed, spatial-hash indexed, high agent counts). So the RVO
   layer is deliberately built on a **neutral value-typed footprint-neighbour abstraction**
   (`RvoNeighbor`: pos/length/latOffset/halfWidth/speed — no strings, no dictionary), populated from
   `VehicleRuntime` today and from the new agent store later. Do **not** couple the RVO solve to the
   current `ExternalObstacle` API.
4. **Statistical SUMO cross-check (DONE).** On a mixed/heterogeneous scenario
   (`scenarios/65-mixed-sublane`: 8 interleaved fast + slow vehicles on one 7.2 m sublane lane,
   `lateral-resolution=0.8`, `sigma=0`, run to t=120) compare **aggregate** flow + lateral
   distribution to a real SUMO sublane run — the SUMO-grounding without byte-chasing.
   `RungRvoStatisticalCrossCheckTests` runs OUR laneless RVO model on the same inputs and asserts
   distribution-level aggregates (mean speed + posLat std over all vehicle-steps) are SUMO-comparable
   (mean speed within 20 %, lateral spread within ~2×), **not** per-step positions. Measured: SUMO
   `meanSpeed=8.8350, posLatStd=0.5288`; RVO `meanSpeed=8.8648` (**0.34 %** rel err), `posLatStd=0.4457`
   (**84 %** of SUMO's lateral spread). The golden is committed as an aggregate reference only (no
   `tolerance.json` / exact compare); byte-identity holds — `LanelessRvo` is off for every parity
   golden and the determinism hash is unchanged.
5. **Open-space full 2D reciprocal ORCA (DONE) — the SECOND regime.** The two-regime insight (Stage 2)
   said lane-derived vehicles use the 1D lateral feasible-interval solve, while *holonomic
   crowd/navmesh agents* need full 2D reciprocal ORCA (disc agents, both axes, sharing avoidance).
   This builds that second regime as a **self-contained subsystem** (`src/Sim.Core/Orca/`): `Vec2`,
   `OrcaSolver` (a faithful port of van den Berg et al. "Reciprocal n-Body Collision Avoidance", the
   RVO2 reference LP1/LP2/LP3 — ORCA is our reference here the way SUMO is for lane behaviour), and
   `OrcaCrowd` (a struct-of-arrays agent store stepped with the same plan/execute double buffer the
   lane engine uses). Doubles, fixed order, no RNG/wall-clock → deterministic (bit-identical runs).
   - **Independent by construction.** It does **not** touch the lane-parity core or the string-keyed
     `ExternalObstacle` API (the owner's constraint: the external-agent API must scale — no string
     ids, high agent counts). Agents are int-indexed SoA; the future entity-handle store backs the
     same arrays without changing the solve. So the determinism hash and every golden are unaffected
     (nothing here is reachable from the lane engine) — verified: `909605E965BFFE59` held, full suite
     green with the 10 new ORCA tests.
   - **Validated behaviourally** (no SUMO counterpart exists): no-overlap (the hard ORCA guarantee),
     reaches-goal on the flows ORCA robustly solves (counter-flow / bidirectional streams, a 16-agent
     permutation crossing, head-on, perpendicular crossing), reciprocal mirror-symmetry, determinism.
   - **Honest finding — the symmetric deadlock.** At *maximal packing* where every path crosses one
     point (the famous perfectly-antipodal circle, and any exactly-symmetric crossing) pure reciprocal
     ORCA can **deadlock** — agents jam in a stable touching ring instead of passing through (reference
     implementations only escape this via nearest-`maxNeighbours` culling + removal-on-arrival, neither
     of which this correctness-first driver does). This is a *convergence* limitation, **never a
     safety one**: the no-overlap guarantee still holds (asserted directly — `AntipodalCircle_
     NeverOverlap_EvenUnderDeadlock`). The remedy for real (never-perfectly-symmetric) inputs is a
     **deterministic** per-`(agent, step)` symmetry-break jitter on the preferred velocity
     (`OrcaCrowd.SymmetryBreak`, default 0.0 = pure): a hash-derived nudge that mimics RVO2's per-step
     random perturbation but is reproducible, resolving symmetric crossings in ~45 steps.
   - **Not yet wired (the cross-regime bridge, next step):** letting open-space agents and lane
     vehicles enter one another's neighbourhoods so SUMO traffic and crowd agents *mutually* avoid.
     That is the genuine unification and is intentionally deferred — this stage delivers and validates
     the open-space solver itself.

## Coordination with SUMOSHARP-API (the NuGet packaging branch)

Reviewed `docs/SUMOSHARP-API.md` (branch `claude/sumo-csharp-nuget-strategy-4vlkki`; currently the
design doc only, no implementation, forked from the same `main` as this branch — so all overlaps are
*future*, at their Phase-1 implementation). Findings:

**STRONG ALIGNMENT (no conflict).** Both sessions independently concluded the string-keyed
`ExternalObstacle` record-dictionary must be **replaced by a handle-based struct-of-arrays store**
(SUMOSHARP-API §4.2–4.4 / D5: `ObstacleHandle` = 32-bit index + 16-bit generation, direct-mapped
columns `laneHandle/frontPos/length/startTime/endTime/speed/maxDecel/latPos/width/latSpeed`, zero-alloc
`UpdateObstacle`, dense active list, batch span API). This is exactly the "scalable int-indexed
footprint-agent store" this doc's Stage 3 calls for. The lateral columns (`latPos/width/latSpeed`)
they already list are load-bearing for the RVO solve — keep them.

**OWNERSHIP SPLIT (to avoid building the store twice).** The obstacle-store redesign is ONE
subsystem both directions want. Decision: the **NuGet session owns the store redesign** (SUMOSHARP-API
§4.3–4.4, Phase 1); the **laneless/RVO layer consumes it** via the neutral `RvoNeighbor` value
abstraction (already built for exactly this decoupling — the RVO solve reads `RvoNeighbor`s, never the
store directly). Today Stage 3 folds obstacles in through a **thin transitional adapter** over the
current `_obstacles` store (one loop in `ComputeRvoLateral`); when the SoA store lands, only that
adapter changes, not the solve.

**INCOMPATIBILITIES needing attention in the NuGet session:**
1. **Read API (§5) must expose lateral state.** The column list is `PosX/Y/Z/Angle/Speed/LaneHandle`
   but the sublane/laneless axis makes `posLat` (and `latSpeed`) first-class — this branch already
   added `VehicleExportSnapshot.PosLat`, `TrajectoryPoint.PosLat`, `Kinematics.LatSpeed`. Add a
   `PosLat` (double, parity-exact) column (lateral IS visible via derived `PosX/Y`, but hosts that
   consume lane-relative state and the sublane goldens need `posLat` directly).
2. **`VTypeParams` / `DefineVType` (§9, open item §14.1) must include the sublane vType params:**
   `maxSpeedLat`, `latAlignment`, `minGapLat` (added here to `VType`/`ResolvedVType`). Without them,
   runtime-spawned vehicles cannot do sublane/laneless lateral behaviour.
3. **Surface the new engine modes** through the facade/config: `ScenarioConfig.LateralResolution`
   (the `_sublane` master switch) and `Engine.LanelessRvo`.
4. **Merge-conflict surface (additive, resolvable):** both branches edit `Engine.cs`,
   `VehicleExportSnapshot.cs`, `DemandModel.cs`, `DemandParser.cs`, `VTypeDefaults.cs`,
   `ScenarioConfig.cs`, `TrajectoryPoint.cs`, `ToleranceConfig.cs`. All additions are in distinct
   regions; whoever merges second reconciles the additive hunks. Coordinate merge order. Both sides
   hold the same parity anchor (hash `909605E965BFFE59`, goldens byte-identical), so the merged result
   must too — both are additive/gated, so it should hold.

### Reply to SUMOSHARP-API §15 / D15-D16 (A: merge order, B: sanity check)

**STATUS: CLOSED.** The NuGet session committed B1/B2 and the merge order; **`RvoNeighbor` is the
frozen seam** between the two branches (their SoA store produces them; this RVO solve consumes them).
No further cross-session reconciliation is open.


Read their §15 + D15/D16 — agreed on all of it (ownership split, the three folded flags, the merge
invariant). Answers to their two asks:

**A. Merge order — AGREED: your obstacle-store redesign lands to `main` first, then my Stage-3
adapter collapses onto it.** Detail:
- My **Stage 1–2** RVO (vehicle↔vehicle: `ComputeRvoLateral` + `RvoNeighbor` + `SeparationPush`) does
  **not** touch the obstacle store at all — it reads the vehicle neighbour query. So Stages 1–2 are
  **merge-order-independent** and can land whenever.
- Only my **Stage 3** reads `_obstacles` (one transitional loop in `ComputeRvoLateral`, explicitly
  marked). It stays on the current `_obstacles` until your SoA store lands (gated/byte-identical, so
  it blocks nothing); when your store lands, **that one loop retargets** — build `RvoNeighbor` from the
  SoA columns instead of the record. Confirmed: your store first, my adapter second.

**B. Sanity check — does anything else in the laneless design need a public API shape you haven't
accounted for?** Mostly confirmations; two genuinely new items:

- **B1 (NEW — cheap to reserve now):** the SoA agent store should carry a **per-agent
  "cooperative/avoidance-class" bit**. My solve picks the reciprocity `share`: `0.5` for a SUMO vehicle
  (it runs the same solve, takes the other half) vs `1.0` one-sided for a dumb external obstacle. A
  navmesh/RVO agent that DOES avoid reciprocally should get `0.5` too — which the solve can only know
  from a per-agent flag. Reserve one column/bit now (e.g. `avoidanceKind: {StaticBlocker,
  OneSided, Reciprocal}`); default `OneSided` = today's behaviour.
- **B2 (NEW — semantic to document):** `Engine.LanelessRvo` currently only takes effect when
  `LateralResolution > 0` (the RVO path is nested under the `_sublane` gate, to keep phase-1
  byte-identical). The facade should **document that coupling** (RVO is a no-op unless
  lateral-resolution is set), and note it may later be promoted to an **independent** continuous-lateral
  mode (RVO does not conceptually need SUMO's `lateral-resolution`).
- **B3 (CONFIRM — already covered by your columns):** the adapter iterates **active agents filtered by
  lane** and reads `{laneHandle, frontPos, length, latPos, width, speed, startTime, endTime}` per
  agent. All eight are in your §4.3 columns + the dense active list; just confirm the store supports
  "iterate active agents (on a given lane)" read access. `RvoNeighbor` needs exactly
  `{frontPos→pos, length, latPos→latOffset, width→halfWidth, speed}` — a direct column read, no new shape.
- **B4 (CLARIFY — no Core impact):** `ToleranceConfig.PosLat` is **`Sim.Harness` (test-only,
  `IsPackable=false`)**, not `SumoSharp.Core` — it's not a shipped public API. Only relevant if you
  later ship `SumoSharp.Testing`.
- **B5 (CONFIRM):** `DefineVType` must **default** `maxSpeedLat=1.0`, `latAlignment="center"`,
  `minGapLat=0.6` when omitted (SUMO defaults, resolved in `VTypeDefaults.Resolve`), and these three are
  **runtime-settable** vType params (your §14.1 open item) — pure behaviour, not net-fixed.
- **No dependency** on: the vehicle read API (RVO reads internal `VehicleRuntime` via the neighbour
  query, never the public read columns — your frozen vehicle SoA is untouched); vehicle handles; the
  spatial hash (Stage-2b internal, no public surface); insertion/spawn. No new `ScenarioConfig` keys
  beyond `LateralResolution`.

## What we explicitly do NOT do

Port SUMO's persistent lateral state machine (`mySafeLatDist*` / `updateExpectedSublaneSpeeds` /
`inform*`) to byte-exact. It is large, brittle, version-dependent, ECS-hostile, and reproduces an
approximation while blocking the continuous model we actually want. The magnitude/geometry mechanisms
it relies on are already understood and documented (`docs/PHASE2-SUBLANE.md`) and are reused as the
RVO layer's physics constants (`minGapLat`, `maxSpeedLat`, footprint widths).
