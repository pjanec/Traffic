# Issue #15 (live-city junction gridlock + lateral float) — RESUMPTION doc

**Read this first after a compaction / fresh session.** Self-contained current state of the #15 work.
Branch: **`claude/livecity-15-turnlane-segregation`**. Full chronological trail (every hypothesis, all
measurements): `docs/LIVE-CITY-15-ATTEMPT-LOG.md` (READ its top "HOW TO REPRODUCE" + bottom sections).
Design docs: `LIVE-CITY-15-LANECHANGE-JUNCTION-FIX-DESIGN.md`, `LIVE-CITY-15-COOPERATIVE-LC-DESIGN.md`.

---

## 1. STATUS — what is DONE (all committed + pushed; latest tip `8f1c0ed`)
Three distinct problems found and fixed, in order, each verified first-hand:

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

## 2. FOLLOW-UPS (the queue — owner wants to compact BEFORE starting these)
In priority order:
1. **NEXT: "into-occupied" cut-ins.** `LIVECITY-LCSWAP intoStoppedTgt` shows ~28 lane changes over 1000s
   that slot in close to a STANDING target-lane car (keepRight ~5, strategic ~50 cumulative, speedGain
   ~17). "Safe" per `IsTargetLaneSafe` (a braking-gap check, not an "empty lane" check — a stopped
   follower needs ~no gap), but visually a tight push-in. Owner rule: never swap into an occupied lane;
   if required, cooperative. Likely fix: a demo-gated minimum-ABSOLUTE-clear-gap veto (don't merge within
   ~X m of a standing target-lane car) + let the strategic informFollower open the gap first. MEASURE it
   doesn't hurt flow (parity-sensitive if applied to the shared IsTargetLaneSafe → keep it demo-gated).
2. **Automatic per-area realism LOD gate.** Cooperative LC is currently a GLOBAL flag. Owner requirement
   (documented, attempt log "OWNER REQUIREMENT ... per-area LOD"): when globally ON, each area's realism
   LOD should AUTO-decide per car — HIGH realism (on-screen) = cooperative + no float; LOW realism
   (distant/off-screen) = auto-fallback to the cheap swap for perf. Key it on the ped-side
   `InterestField`/`PedLodManager`/`Sim.Pedestrians.Lod` split. `useCoop = globalCoopEnabled &&
   areaIsHighRealism(car)`; the keep-right guard follows `useCoop`. Must stay deterministic (pure fn of
   frozen start-of-step state).
3. **Liveness/throughput regression TEST in the suite.** The parity gate structurally CANNOT catch #15
   (the bug was inert on every golden). Add a separate headless, no-SUMO test (fixed seeds): run the
   live-city smoke ~1000s, assert `arrivals >= threshold` and late `stoppedFrac <= threshold`. Thresholds
   from the post-cure baseline (arrivals ≥ ~900). Separate from parity.
4. **Visual confirm in the 3D viewer.** The engine now emits ZERO stopped lateral swaps, so the float
   source is gone at the source (not masked). Owner to confirm cars cooperatively merge instead of
   floating. Remaining keep-rights are all ≥1.5 m/s (forward+lateral diagonals, owner-accepted).

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
