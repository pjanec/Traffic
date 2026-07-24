# Issue #15 (live-city junction gridlock + lateral float) — RESUMPTION doc

**Read this first after a compaction / fresh session.** Self-contained current state of the #15 work.
Branch: **`claude/livecity-15-turnlane-segregation`**. Full chronological trail (every hypothesis, all
measurements): `docs/LIVE-CITY-15-ATTEMPT-LOG.md` (READ its top "HOW TO REPRODUCE" + bottom sections).
Design docs: `LIVE-CITY-15-LANECHANGE-JUNCTION-FIX-DESIGN.md`, `LIVE-CITY-15-COOPERATIVE-LC-DESIGN.md`.

---

## 1. STATUS — what is DONE (all committed + pushed; latest tip `1f2e654`)
Four distinct problems found and fixed, in order, each verified first-hand (the 4th, into-occupied cut-ins,
is summarized in §2 item 1 — gridlock/blockage/float are the first three below). ALL §2 follow-ups that can
be verified HEADLESSLY are now DONE (into-occupied, liveness test, per-area LOD gate); only the 3D visual
confirm (§2 item 4) and the moving-camera LOD source remain, both needing the GUI / owner input. Full
solution `dotnet test` green (657 parity + 22 live-city + 11 viewer + 19 motion + 259 ped + 11 ig + 6 host).

1. **The terminal gridlock (#15 core) — CURED.** Root cause (per-car verified): a continuous
   lane-change maneuver (`lanechange.duration=2.0`) that crossed a junction boundary left a stale
   `LcTargetHandle` on the departed edge; `AdvanceLaneChanges` then snapped the car back onto that edge
   while `LaneSeqIndex` advanced → a pool/edge DESYNC → the car clamped `Speed=0` forever = "frozen on
   green". Fix (`Engine.cs`): clear any in-progress maneuver at the boundary cross + reject a stale
   cross-edge target in `AdvanceLaneChanges`. Inert at `LaneChangeDuration==0` (every golden). Result:
   arrivals 258→~713, terminal lock gone.

2. **The queue-blocking "floater" — SOLVED.** Residual blockers were `capSpent` wrong-lane strands
   (cars that hit `MaxDeadLaneReroutes=2` and clamped forever). Fix: made `WrongLaneRerouteAtApproach`
   + `DeadLaneDriveThrough` the live-city DEFAULTS (a wrong-lane car reroutes/drives-through instead of
   clamping — never blocks). Result: arrivals →1025, `strandedDeadEnd=0`, `stuckInternal=0-3`.

3. **The lateral "float" (car slides sideways at a light) — SOLVED via COOPERATIVE LANE CHANGE.**
   Per-car proven (`LIVECITY-LCSWAP`) that the ONLY engine source of stopped lateral swaps was
   **keep-right** (commits inline, bypassing the `LaneChangeMinSpeed` stopped-car guard): 448 stopped
   swaps → the float. Simply suppressing them box-blocks (arrivals 1025→458, stuckInternal→50) because
   the demo's flow DEPENDED on that stopped lane-sort. Fix: revive the retired `CoopSpeedAdvice`
   informFollower (git `afec614`) + add a STRATEGIC-path informFollower, so cars sort into their turn
   lane COOPERATIVELY (target follower opens a gap) — mostly up-front while moving; extreme case waits.
   THEN the keep-right stopped-swap guard is applied, GATED on cooperation. Result (verified first-hand):
   keepRight stop 634→**0** (float gone), arrivals 1025→**1085**, stuckInternal **0-4**, coopAdvice=**340**
   (cooperation fires). The guard is gated on `CooperativeInformFollower`, so the master switch flips both:
   coop ON = high realism (no float); coop OFF (`LIVECITY_COOP=0`) = low realism = cheap swap restored
   (~1000 arrivals) — the owner's "optionally OFF" fallback.

**Gates (ALWAYS re-verify first-hand before accepting any change):** `dotnet test tests/Sim.ParityTests
-c Release` = **657 passed / 4 skipped** byte-identical; `dotnet run --project src/Sim.Bench -c Release`
→ `deterministic=True`, `parallel==single`, hash **`D96213B7BB4021A7`**; no `System.Random`. All #15
fixes are demo-gated → inert on every golden → byte-identical.

---

## 2. FOLLOW-UPS (the queue)
In priority order:
1. **DONE — "into-occupied" cut-ins (commits `ff23060` + `89743b8`).** Per-car analysis (new
   `LIVECITY-LCDETAIL` probe: side × gap bucket) showed the residual was FOLLOWER-side, gap 2-5 m, by a
   MOVING changer — nothing ever <2 m. Root loophole: `IsTargetLaneSafe` is a braking-gap check a STOPPED
   follower satisfies at any closeness. Fix (all demo-gated, engine knobs default 0 ⇒ goldens inert):
   `Engine.MergeStoppedMinGap` (5 m) vetoes a tight cut-in ahead of a stopped follower on the
   DISCRETIONARY paths (speed-gain, keep-right → 14→0); `Engine.MergeStoppedStrategicDeferDist` (15 m)
   urgency-gates the REQUIRED strategic path (defer while ample road remains, allow once must-merge → 46→16,
   strand-safe). Blanket-vetoing the strategic path was MEASURED to reform gridlock (arrivals→361) — a
   saturated turn lane has no non-tight merge point, so required queue-joins must be allowed. A/B sweep found
   a sharp flow cliff between 20-25 m (deferral). Net: total follower-side tight cut-ins **60→16 (−73%)**,
   flow unchanged (arrivals 1085→1068, stoppedFrac 0.34), parity 657/4 + hash `D96213B7BB4021A7`. Survivors
   are required moving forward+lateral queue-joins at the must-merge point (owner-permitted). Env knobs:
   `LIVECITY_MERGEGAP`, `LIVECITY_MERGEDEFER`. Design: `LIVE-CITY-15-INTO-OCCUPIED-DESIGN.md`.
2. **DONE — Automatic per-area realism LOD gate.** Cooperative LC / into-occupied vetoes / keep-right float
   guard are now PER-CAR: `useCoop(v) = CooperativeInformFollower && !v.LowRealismLaneChange` (engine helper
   `CooperativeLcFor`). The host (`LiveCitySim.Step`) classifies each live car from its previous-snapshot
   position vs the static high-realism pocket (`IsLowRealismLaneChangePos`, promote 70 m) and sets the
   per-vehicle flag via `Engine.SetLowRealismLaneChange` BEFORE the engine step. Inside the pocket =
   cooperative + no float; outside = cheap swap (float permitted, distant/unobserved). Deterministic (pure fn
   of frozen position + static pocket). Demo-gated: `LowRealismLaneChange` defaults false and goldens drive
   the Engine directly (never `LiveCitySim`), so parity **657/4** byte-identical + hash `D96213B7BB4021A7`.
   Verified: flow healthy (arrivals 1022, no gridlock), `keepRight stop 0→571` (floats outside pocket),
   `coopAdvice > 0` (cooperation inside pocket); `Sim.LiveCity.Tests` 22 passed (2 new). Design:
   `LIVE-CITY-15-PER-AREA-LOD-DESIGN.md`. FOLLOW-UP: tie the interest source to a moving camera/avatar
   (currently a static crop-centre pocket) so the high-realism zone tracks where the viewer is looking.
3. **DONE — Liveness/throughput regression TEST.** `tests/Sim.LiveCity.Tests/LiveCitySimTests.cs`
   → `DenseFlow_OverAThousandSeconds_KeepsDischarging_NoGridlock`. Headless, no-SUMO, deterministic: runs
   the coupled sim 2000 steps (1000 s) with the shipped dense-flow config PINNED (immune to `LIVECITY_*`
   env), asserts final arrivals ≥ 450, arrivals-growth in the last 400 steps ≥ 40 (the anti-flatline — the
   sharpest gridlock signal), and late stopped-fraction ≤ 0.85. Measured separation: healthy ≈736 arrivals /
   +145 growth / 0.35 frac vs a #15 gridlock ≈360 / +2 / 1.0. **Non-vacuousness proven first-hand**: pointing
   the pinned config at a veto-all setting (`MergeStoppedStrategicDeferDist=0.5`) reproduces the gridlock and
   the test FAILS (`got 360`). ~11 s wall. The parity gate structurally cannot catch this class (inert on
   every golden); this is the guard.
4. **Visual confirm in the 3D viewer.** The engine now emits ZERO stopped lateral swaps, so the float
   source is gone at the source (not masked). Owner to confirm cars cooperatively merge instead of
   floating. Remaining keep-rights are all ≥1.5 m/s (forward+lateral diagonals, owner-accepted).
5. **DEFERRED (owner) — detach the live-city DEMO data from the LOCKED `box` regression fixture.** The
   liveness test (and several existing tests) pin `scenarios/_ped/demo_city/box/`, so it can't be freely
   edited as demo scene data. Plan: lock `box` (🔒 README), give the demo its own `box-live` copy, split
   `LiveCityConfig.ForRepoRoot` (demo default) vs a new `ForRegressionFixture` (locked), repoint the 7 test
   `ForRepoRoot` callers. Full concrete plan + open naming decision recorded in the GLOBAL queue
   `docs/TASKS.md` ("Deferred — detach the live-city DEMO data …") and `box/README.md`.

---

## 3. HOW TO REPRODUCE / MEASURE (deterministic, headless, no SUMO)
```bash
dotnet build src/Sim.Viewer -c Release
# THE repro + all diagnostics (coop+guard are the defaults; ~1400s of sim):
LIVECITY_LCLOG=1 LIVECITY_WITNESS=1 LIVECITY_TELEPORT=0 LIVECITY_CARS=160 \
  dotnet run --project src/Sim.Viewer -c Release --no-build -- --mode live-city --smoke --frames 2800 \
  2>&1 | grep -E "LIVECITY-GRIDLOCK:|LIVECITY-LCSWAP|LIVECITY-WITNESS:"
```
Probes: **LIVECITY-GRIDLOCK** `t liveCars stoppedFrac meanSpd aggMove arrivals peds`. **LIVECITY-LCSWAP**
per path `[stop/slow/move,intoStoppedTgt]` + `coopAdvice` (executed swaps; stopped=pure-lateral float;
coopAdvice=strategic informFollower fired). **LIVECITY-WITNESS** stuck breakdown incl. `strandedDeadEnd`,
`stuckInternal` (box-block). **LIVECITY-STRANDREASON** why wrong-lane cars resolve (reResolveOK/rerouteOK
= recovered; poolEdgeMismatch/capSpent = strand). **LIVECITY-STUCKCLEAR** per-binder autopsy of
stuck-with-clear-road cars.
Env knobs: `LIVECITY_COOP=0/1` cooperative LC (default on) · `LIVECITY_WRONGLANE`/`LIVECITY_DRIVETHROUGH`
never-clamp (default on) · `LIVECITY_LCMIN` lane-change min speed (1.5) · `LIVECITY_LCLOG=1` swap analysis
· `LIVECITY_SEQDESYNC=1` desync tracer · `LIVECITY_WITNESS=1` per-car autopsy · `LIVECITY_CARS/PEDS/TELEPORT`.

## 4. OWNER'S BINDING RULES (do not violate)
- **No pure-lateral motion** (car sliding sideways with zero forward). Forward+lateral (diagonal, any
  speed incl. creeping from a stop) is fine; only pure-lateral is forbidden.
- **Never swap into an occupied lane.** If a merge is required but blocked, it must be **cooperative**
  (target follower opens a gap; in the extreme the merger stops/waits — briefly blocking is OK).
- **Don't mask an engine swap in the renderer** — fix the cause.
- **Realism is a per-area LOD knob** — high-realism never cheats; low-realism may (perf).
- **Verify every hypothesis in the per-car data BEFORE chasing it**; verify every fix first-hand
  (parity 657/4 + bench hash + the behavioral metrics); NEVER ship a flow regression.
- Parity is the iron law; all #15 work is demo-gated and byte-identical on goldens.

## 5. KEY FILES
- `src/Sim.Core/Engine.cs` — the maneuver-straddle fix (boundary-cross clear + `AdvanceLaneChanges` stale
  reject), `WrongLaneRerouteAtApproach`/`DeadLaneDriveThrough`, `CoopSpeedAdvice` consumption +
  speed-gain & NEW strategic informFollower, `CooperativeInformFollower`/`CoordinatedLaneChange` gates,
  keep-right float guard (gated on coop), diagnostics (`StrandReasonHistogram`, `DiagLaneChangeLog`/
  `CoopAdviceIssued`, `DiagSeqDesync`).
- `src/Sim.Core/{CommandBuffer,ICommandBuffer,VehicleRuntime}.cs` — the CoopSpeedAdvice channel.
- `src/Sim.LiveCity/LiveCityConfig.cs` — demo knobs (`CooperativeLaneChange`, `WrongLaneRerouteAtApproach`,
  `DeadLaneDriveThrough`, `LaneChangeMinSpeed`, env overrides).
- `src/Sim.LiveCity/LiveCitySim.cs` — engine wiring + diagnostic passthroughs.
- `src/Sim.Viewer/Program.cs` `RunLiveCitySmoke` — all the `LIVECITY-*` probes.
- `scenarios/_ped/demo_city/box/` — the demo net/demand (51 static TLs, Krauss army vehicles 7-14m).
