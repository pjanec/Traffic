# LANE-CHANGE-OVERLAP-STATUS.md — where we stand (2026-07-21)

Single "current standing + how to resume" reference for the dense multi-lane car-overlap work.
Companion docs: `-SPEC.md` (WHAT/acceptance), `-DESIGN.md` (HOW/root cause), `-TASKS.md` (stages +
success conditions), `-TRACKER.md` (checklist). Prior context: `HIGH-DENSITY-P2G-DESIGN.md`,
`HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`, `SUMOSHARP-ISSUE-stopped-lane-change-overlap.md`.

## TL;DR
- **Reported bug (unrealistic stopped/near-stopped lane change into an occupied slot → cars overlapping):
  FIXED.** Same-lane overlaps on the committed dense diagnostic `scenarios/_diag/willpass-saturation`
  (200 steps, library/parity mode) go **197 → 2**, both-stopped **186 → 0**, stuck **0**.
- **Landed on branch `claude/dense-lane-overlap-fix-5tr4ha`**, commit *"fix(engine): dense multi-lane
  car-overlap — port checkChange LCA_OVERLAPPING + follower veto + arrival-lane cross-junction leader"*.
- **Every committed golden byte-identical**, full `dotnet test Traffic.sln` green, two-run deterministic,
  **high-density throughput unchanged** (saturated grid still drains all 412 veh, stuck 0).
- **Owner decision (C): accept the 2 residual for now.** They are NOT the reported stopped-car bug — they
  are transient MOVING junction-emergence overlaps (see §3). The acceptance test
  `LaneChangeOverlapDiagTests` therefore stays `[Fact(Skip)]` (it asserts ==0; we are at 2).

## 1. What the fix does (committed)
Three faithful, parity-safe changes in `src/Sim.Core/Engine.cs`:
1. **`IsTargetLaneOverlapped`** — ports SUMO `MSLaneChanger::checkChange`'s `LCA_OVERLAPPING` block
   (`MSLaneChanger.cpp:767/780`): a change is vetoed if any target-lane occupant's body overlaps ego's
   slot. ANDed into every change path (keep-right, speed-gain, strategic, give-way). This is the load-
   bearing fix — it catches the dominant cause (see §2).
2. **Keep-right follower veto** — keep-right now passes the real target-lane follower to
   `IsTargetLaneSafe` (previously `null`), matching `checkChange`'s symmetric follower block.
3. **Arrival-lane cross-junction leader** — `CrossJunctionLeaderConstraint` additively scans the ARRIVAL
   lane span (physical path), not just the pool/exit span, so a vehicle brakes for a leader on the lane
   it actually emerges onto when arrival≠exit (multi-lane intra-edge change). Additive braking → faithful.

## 2. Root cause (measured — differs from the SPEC's hypothesis)
The SPEC hypothesised a missing follower-gap veto + cooperative lane-changing. Direct per-commit
instrumentation showed the DOMINANT cause is a **neighbour-query blind spot**: a target-lane car at
ego's EXACT stop-line position is classified as neither leader nor follower (`GetNeighborLeader` skips
`pos≤ego`, `GetNeighborFollower` skips `pos≥ego`), so the secure-gap veto never sees it — even on the
strategic path which already passes both. SUMO models exactly this as the `LCA_OVERLAPPING` negative-gap
block. ~40 of ~50 overlap-creating commits were this exact-tie case.
**Cooperative LC is NOT required**: with the serve-path junction-TL fixes already in-tree, the
`HIGH-DENSITY-P2G-DESIGN.md §5` gridlock trap is obsolete — the keep-right follower veto no longer
gridlocks (stuck stays 0). The retired `informFollower` machinery was deliberately NOT resurrected.

## 3. The 2 residual overlaps (accepted, deferred) — precise mechanism
Neither is a lane-change overlap; both are junction-emergence / car-following artifacts of the ECS
frozen-snapshot + deferred-placement model (SUMO avoids them via strict serial per-vehicle processing):
- **(a) Junction merge** — two vehicles cross ONE junction via DIFFERENT internal lanes and emerge onto
  the SAME normal lane in the same/adjacent step (e.g. `:B0_0` + `:B0_5` → `B0A0_1`). Each is on a
  `:`-internal lane at the other's planning time, so the frozen neighbour snapshot cannot see it, and
  neither is the other's cross-junction *leader* (they come from different approaches).
- **(b) Car-following reaction** — a follower closes ~0.25 m for ONE step when its leader hard-brakes
  (e.g. 8→0 m/s) immediately after changing into the follower's lane; on a saturated grid the engine's
  trajectory legitimately diverges from SUMO's (statistical, not bit-exact, parity), so this transient
  differs from SUMO's specific overlap-free path.
Both are transient (1 step), moving (not stopped), and rare (2 in 200 steps × ~150 veh).

Why not closed yet: faithful car-following fixes reduce but cannot fully eliminate them (each new
constraint shifts the trajectory and can surface a different transient — whack-a-mole). A predictive
"hold at the junction" gate is UNFAITHFUL (it fired spuriously on congested traffic → moved scenario-45
out of tolerance and, in one variant, gridlocked 0→41 stuck). A post-settle resolver cannot cleanly
clamp a just-emerged car (clamping it behind the leader yields a negative arc position, back across the
junction).

## 4. Verification done this session (all green)
- Overlaps (parity/library mode, `willpass-saturation`, 200 steps): **197 → 2** (both-stopped 186 → 0).
- Overlaps (coordinated/runtime-host mode, the live-city path): **258 → 3**.
- Stuck (700 steps): **0** (both modes). Throughput: unchanged vs baseline (all 412 drain, fully by ~t=600).
- Parity: **every committed scenario byte-identical** (full-sweep engine-FCD byte-diff, parity mode);
  full `dotnet test Traffic.sln` **green** (654 ParityTests + pedestrians/host); no golden regenerated.
- Determinism: two runs byte-identical.
- No new `System.Random`; the veto reads only the frozen snapshot + immutable network, writes only ego.

## 5. How to reproduce / measure (for the testing session)
```
# engine overlaps (parity mode == the acceptance-test's `new Engine()`):
dotnet run -c Release --project src/Sim.Run -- scenarios/_diag/willpass-saturation --steps 200 \
    --fcd-out /tmp/ss.fcd.xml --parity
# coordinated mode (live-city default, no --parity):
dotnet run -c Release --project src/Sim.Run -- scenarios/_diag/willpass-saturation --steps 200 \
    --fcd-out /tmp/ss_c.fcd.xml
# overlap metric (per timestep, consecutive same-lane pairs < 5.5 m front-to-front, skip ':' lanes):
#   see tests/Sim.ParityTests/LaneChangeOverlapDiagTests.cs (the authoritative metric, lane-relative Pos)
# vanilla SUMO reference (0 overlaps):
cd scenarios/_diag/willpass-saturation && sumo -c config.sumocfg --end 200 \
    --fcd-output /tmp/sumo.fcd.xml --fcd-output.attributes x,y,speed,lane
# live-city (coordinated mode; density knob):
LIVECITY_CARS=110 dotnet run -c Release --project src/Sim.Viz -- --live-city /tmp/out.html
```
Offline gate (no SUMO, no network): `dotnet test Traffic.sln` — must stay green on a fresh clone.

## 6. Resume plan for the FULL fix (Stage 3 — if/when green-lit)
Target: `LaneChangeOverlapDiagTests` unskipped, asserting **== 0**, while keeping every golden
byte-identical and stuck 0. Recommended approach **Option A (faithful)**: port SUMO's internal-junction
follower/foe handling so a vehicle yields BEFORE emerging into an occupied/merge slot
(`MSLink`/`MSLane::getLeaderInfo` + the junction response matrix), which addresses residual (a) at the
source. Residual (b) then needs a look at the cross-junction/just-changed-leader visibility on the
saturated grid. Build incrementally and re-measure overlaps + stuck + full byte-diff at each step; if a
step moves a golden, it must be provably SUMO-faithful with a regenerated golden + provenance bump, else
reverted. Design-doc it first (CLAUDE.md design-first).
Known dead-ends (do not repeat): predictive "hold at junction exit" gate (unfaithful, breaks scen-45 /
gridlocks); resurrecting `informFollower`/cooperative-LC (not the cause; retired for good reason).

## 7. Broader realism lever noticed (not done — separate from the overlap fix)
`LaneChangeMinSpeed` (the opt-in "don't change lanes while stopped/near-stopped" realism knob, default 0)
is enforced in `CommitLaneChange` + `AdvanceLaneChanges` but the **keep-right inline swap bypasses it**,
so a stopped/near-stopped car can still keep-right (slide sideways in a queue) even with the knob on.
Closing that (gate the keep-right swap on `LaneChangeMinSpeed`) is a cheap, opt-in, byte-identical-when-
off realism improvement for the product/live-city mode — tracked here for the realism pass, independent
of the overlap acceptance gate.
