# Issue #15 (live-city junction gridlock) — RESUMPTION doc

**Read this first after a compaction / fresh session.** It is the self-contained state of the #15
investigation and the in-flight fix. Companion trail (full detail, newest at bottom):
`docs/LIVE-CITY-15-ATTEMPT-LOG.md`. Diagnosis of record: `docs/DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`.

Branch: **`claude/livecity-15-turnlane-segregation`**.

> **UPDATE (root cause now PROVEN at the per-car level; the task-#21 reroute fix was measured and does
> NOT cure it — it regresses).** The "cars frozen on green with no visible reason" are cars sitting at
> `pos == laneLength` (the physical lane end) whose lane has **no connection to their next (turn) route
> edge** — a **wrong-turn-lane strand**, permanently clamped `Speed=0`. Proven by the `LIVECITY-STUCKCLEAR`
> per-car dumps (binder=`freeFlow`, i.e. no speed constraint holds them — the MOVE EXECUTOR does) + the
> net.xml connection topology (e.g. edge `e_d_4_3_d_4_4`: lane 2 serves straight+left only, so a car
> needing the right turn `d_5_4` on lane 2 can never proceed). `Engine.WrongLaneRerouteAtApproach`
> (reroute at approach) was implemented parity-safe (657/4, bench hash unchanged) but MEASURED to make it
> WORSE (arrivals 258→225, `stuckInternal` 0→14-19 box-blocking) — knob kept, **default OFF**. The real
> cure is upstream: sort cars into a turn-compatible lane before the junction (SUMO best-lanes/strategic
> LC). See `docs/LIVE-CITY-15-ATTEMPT-LOG.md` bottom section for the full proof + numbers. That LC gap is
> the next target (real engine work, not a knob).

---

## 1. The problem (owner's words)
The live-city 3D viewer **always** gridlocks after ~10–15 min: almost all cars standing, "junctions
occupied", cars **waiting on green with no visible reason**. Owner: teleport is the WRONG cure (a car
must travel THROUGH a junction, not jump across); the car "has nowhere to go next".

## 2. ROOT CAUSE — PROVEN (not a hypothesis)
It is a **real, fixable engine junction-discharge bug**, NOT the scenario, NOT inherent saturation,
NOT the ped coupling, NOT rendering, NOT box-block:
- **Mechanism:** a car that cannot complete its strategic lane-change into the route-required exit lane
  in dense traffic reaches a junction on the **WRONG lane** — one with **no connection to its next
  route edge**. It **stops dead at the stop line and never moves** (no foe, nowhere to go). The rescue
  that would reroute it (`Engine.TryReResolveFromActualLane` / `TryRerouteFromDeadLane`) **only fires at
  the PHYSICAL lane end**, but the car stops SHORT of it → permanent stall. A handful of these strands
  accumulate (`strandedDeadEnd` grows 3 → 22 over ~900 s), each walling its approach queue, cascading to
  total lock. SUMO reroutes off the connection its lane DOES have and drives through, so it never strands.
- **Proof it's the engine, not the scenario:** dumped the exact 418-trip sustained demand our engine
  spawns over 1000 s and ran **SUMO 1.20 on the SAME net + SAME trips** (teleport off): **SUMO DRAINS it**
  (running 153→12, meanSpeed 9–11 m/s throughout) while **our engine GRIDLOCKS** (0.99 stopped, 258
  arrivals flatlined). SUMO is the existence proof that this net+demand flows.
- **Density-independent** (cap 80/120/160 all lock over ~1000 s); **junctions stay empty**
  (`stuckInternal=0`, so NOT box-block). Matches `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md` §"Why it happens"
  exactly (candidate fix 1).

## 3. THE FIX (in flight)
Generalise the wrong-lane reroute to fire at the **stop-line APPROACH**, not only the physical lane end,
and **retry every step** instead of permanently one-shot-clamping. Gated engine knob
`Engine.WrongLaneRerouteAtApproach` (bool, **default false = byte-identical**; the dead-lane path is
already inert on every golden). Wired ON in live-city via `LiveCityConfig.WrongLaneRerouteAtApproach`
(default true, env `LIVECITY_WRONGLANE`).
- **Success criteria (the implementer MUST hit these; verify first-hand, don't trust the report):**
  1. `dotnet test tests/Sim.ParityTests -c Release` = **657 passed / 4 skipped**, byte-identical (knob off).
  2. `dotnet run --project src/Sim.Bench -c Release` → `deterministic=True`, `parallel==single=True`,
     hash **`D96213B7BB4021A7`** unchanged.
  3. Repro (below), knob ON: the sim **no longer goes terminal** — late `stoppedFrac` well below ~0.9,
     `arrivals` climb past the ~258 flatline toward SUMO's drain, `strandedDeadEnd` stays small.
- If it does NOT lift the gridlock, the numbers say so — do not ship it as the cure without the measured
  before/after. If it works, commit + push, mark task #21 done, tell the owner + Windows session.

## 4. Repro & measurement commands
```bash
# THE repro (headless, ~1000 s of sim). Baseline (knob off): by t=940 stoppedFrac ~0.99, arrivals ~258 flat.
LIVECITY_TELEPORT=0 LIVECITY_CARS=160 dotnet run --project src/Sim.Viewer -c Release -- \
  --mode live-city --smoke --frames 2000 2>&1 | grep "LIVECITY-GRIDLOCK:"
# Per-car autopsy (binders, strandedDeadEnd, stuckInternal, foe speeds, green/red split):
LIVECITY_WITNESS=1 ...same... 2>&1 | grep -E "LIVECITY-WITNESS:|LIVECITY-STUCKCLEAR:"
# A/B a knob: LIVECITY_WRONGLANE=0/1, LIVECITY_TELEPORT=<s>, LIVECITY_YIELDTIMEOUT=<s>, LIVECITY_CARS=<n>
# Export the sustained demand for a SUMO cross-check:
LIVECITY_DUMPROUTES=/tmp/x.rou.xml LIVECITY_TELEPORT=0 ...same... ; # writes N <trip>s + prints engine arrivals
```
**SUMO 1.20** (pinned; matches SUMO_VERSION): `pip install eclipse-sumo==1.20.0`; binary at
`/usr/local/lib/python3.11/dist-packages/sumo/bin/sumo`. Run: `$SUMO -n scenarios/_ped/demo_city/box/net.xml
-r <trips> --step-length 0.5 --time-to-teleport -1 --lanechange.duration 2.0 --default.speeddev 0
--summary-output S.xml --no-step-log --xml-validation never --ignore-route-errors`. SUMO drains it.

**Gates that must always hold:** `dotnet test tests/Sim.ParityTests -c Release` = 657/4; `Sim.Bench`
deterministic + hash `D96213B7BB4021A7`; no `System.Random`.

## 5. What was TRIED against the terminal gridlock (measured)
| lever | result | verdict |
|---|---|---|
| dense-flow engine MERGE (`184fb31`) | terminal→jam-and-recover, arrivals 38→81@200s | helped, only DELAYED the terminal lock |
| `MaxDeadLaneReroutes` 2→50 | arrivals 257→265@940s | ~nothing |
| `DeadLaneDriveThrough` (drive-through at lane END) | 257, no change | **nothing** — cars stall SHORT of the lane end, never reach this code |
| `JunctionYieldTimeoutSeconds` (impatient gap-force) | 257, no change | nothing (dominant hold is a real MOVING cross-foe, can't override safely) |
| SUMO teleport (`TimeToTeleportSeconds`=5) | arrivals→188, stoppedFrac 0.39→0.10 | CURES it, but RELOCATES the car — **owner rejected** |
| **`WrongLaneRerouteAtApproach` (reroute at APPROACH)** | **IN FLIGHT** | the confirmed fix — reroute before the car stops short |

Also ruled out with instrumentation: not a TL render-lie (`tlRenderLie=0`), not protected-green
over-yield (`egoHasSignalPriority` false on the stuck cars), not box-block (`stuckInternal≈0`).

## 6. Instrumentation added this investigation (all parity-neutral, off the golden path, committed)
- Diagnostic engine columns: `Engine.BindingConstraints` (which of 13 speed constraints bound each car),
  `JunctionYieldArms` (+0x80 priority bit), `JunctionYieldFoeSpeeds`; backed by `VehicleRuntime` fields +
  `VehicleReadBuffer` columns. Argmin capture is inline in `ComputeMoveIntent`'s Min-fold and inside
  `JunctionYieldConstraint` — verified byte-identical (657/4, hash unchanged).
- `LiveCitySim.ArrivedTotal`, `WitnessAuthoritative()`, `SampleCars()`, `SpawnLog`.
- `RunLiveCitySmoke` probes (`src/Sim.Viewer/Program.cs`): `LIVECITY-GRIDLOCK` (stoppedFrac/meanSpd/
  aggMove/arrivals), `LIVECITY-WITNESS` (stuck breakdown incl. stuckInternal/strandedDeadEnd/rendered-
  green vs engine TL), `LIVECITY-STUCKCLEAR` (per-binder + foe-speed + overlaps for cars stuck WITH a
  clear road), `LIVECITY-BINDER`/`JYarm` histograms, `LIVECITY_DUMPROUTES` SUMO-demand export.
- Env knobs: `LIVECITY_WITNESS`, `LIVECITY_DUMPROUTES`, `LIVECITY_TELEPORT`, `LIVECITY_YIELDTIMEOUT`,
  `LIVECITY_WRONGLANE`, plus the pre-existing `LIVECITY_CARS/PEDS/YIELD/LCMIN/HZ`.

## 7. Gated demo knobs currently in `LiveCityConfig` (Engine defaults all OFF = parity-safe)
`JunctionYieldTimeoutSeconds=5` (minor, harmless), `TimeToTeleportSeconds=0` (off; owner rejected teleport),
`DeadLaneDriveThrough=false` (kept, doesn't cure), `WrongLaneRerouteAtApproach=true` (the fix, in flight).

## 8. Key files
- `src/Sim.Core/Engine.cs` — `ComputeMoveIntent` (constraint fold + binder), `JunctionYieldConstraint`,
  `CrossJunctionLeaderConstraint`, `TryReResolveFromActualLane` (~9793), `TryRerouteFromDeadLane` (~9884),
  the lane-end strand+clamp (~9647-9675), `CheckJamTeleports` (teleport, gated on `TimeToTeleport>0`),
  the knob properties (`LaneChangeMinSpeed`, `JunctionYieldTimeoutSeconds`, `DeadLaneDriveThrough`,
  `WrongLaneRerouteAtApproach`). Constants: `MaxDeadLaneReroutes=2`, `DeadLaneRerouteWaitSeconds=5`.
- `src/Sim.LiveCity/LiveCitySim.cs` (engine wiring), `LiveCityConfig.cs` (knobs+env).
- `src/Sim.Viewer/Program.cs` `RunLiveCitySmoke` (the probes).
- `scenarios/_ped/demo_city/box/net.xml` — the demo net (51 static uncoordinated TLs — fine, SUMO handles it).

## 9. Other open items on this branch (not #15)
- Task #10 (owner deferred): Raylib replay timeline-scrub jerk.
- Windows GPU session runs the viewer, sees the gridlock every run — will want the fix pulled + verified.
