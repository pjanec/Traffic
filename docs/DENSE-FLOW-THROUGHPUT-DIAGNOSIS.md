# DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md — why SumoSharp gridlocks at low density (2026-07-21)

Diagnosis for the two SumoData NEEDs: `SUMOSHARP-NEED-dense-flow-gridlock-vs-vanilla.md` (Gap 1, HIGH)
and `SUMOSHARP-NEED-serve-calibration-parity-gaps.md` (Gap 1). Related: `FOLLOWUP-TL-throughput-flowrate.md`.
**Status: DIAGNOSIS COMPLETE — root cause localised; fix is design-first, not yet started.**

## TL;DR — the root cause
Under dense insertion SumoSharp **gridlocks vehicles that need to change into a specific route-required
exit lane at a junction but cannot complete that lane change in the dense queue**. Instead of doing what
vanilla does — take the connection its *actual* lane offers and **reroute** to the destination — SumoSharp
**rigidly holds its planned route, stops dead at the stop line (its current lane has no connection to its
next route edge), and never moves again**. The stalled front vehicle blocks the whole approach queue; a
handful of these across the net cascades into the gridlock/teleport blow-up the calibration sees.

This is a **lane-selection + reroute-on-wrong-lane** deficit, NOT a car-following, yield/RoW, or the
already-fixed lane-change-overlap problem.

## Reproduction (clean, deterministic, ~2 min)
Substrate: `scenarios/_repro/synthetic-junction2` (the committed throughput witness — TL + priority
junctions, heavy demand on short approaches). Run via the `sumosharp` drop-in (`src/Sim.Sumo`) vs vanilla
SUMO 1.20.0 with a **2× compressed-depart** demand (`depart *= 0.5`, same 325 vehicles) to stress it:

| engine | teleports | arrivals | halting @ t=600..900 | verdict |
|---|---|---|---|---|
| vanilla SUMO 1.20.0 | **0** | **290** | drains to **0** | flows |
| SumoSharp (main `8bb8219`) | **10** (8 *yield*, 2 jam) | 275 | **stuck at ~45**, meanSpeed **0** | gridlock |

Vanilla drains the identical net at 2× density; SumoSharp locks ~45 vehicles permanently (meanSpeed 0
from t=600). Baseline (1× depart) shows the same direction, milder (5 teleports, 287 vs 290 arrivals).
Commands: `sumosharp -c <cfg> --end 1000 --statistic-output S --tripinfo-output T --summary-output U`;
vanilla is the same with `sumo`. (The full `demo_city/box` can't yet load on SumoSharp — `departPos="base"`
+ parkingArea oversubscription, Gap 2/3 in the serve-calibration NEED — so the synthetic is the substrate.)

## Localisation (measured, not assumed)
- Of the vehicles SumoSharp stalls (excluding legitimately-parked sink vehicles both engines hold), the
  **root stalls are all "wrong-lane" strands**: the vehicle sits at a junction entry on a lane that has
  **no connection to its next route edge**, e.g. veh `295`/`155` on `30_1` whose route needs `30→124` but
  the `→124` connection leaves only from `30_0`; veh `242` on `-2451_1` needing `→-221`. The other stalled
  vehicles are simply **queued behind** a wrong-lane strand (they are lane-OK themselves).
- The stalled front vehicle **never moves** (veh 295: pos 24.1, speed 0, t=520→680) and **nothing crosses
  the junction** during that time (0 crossers on `:29_*`) — the junction is idle, the vehicle just won't go.
- **Every one of these vehicles arrives in vanilla** (295@412, 242@345, 155@264, 172@372, 316@334) and
  **none in SumoSharp**. Vanilla's 295 does NOT reach `124` either — it takes `30_1→44` (the connection its
  lane offers) and **reroutes** to the destination (`device.rerouting.probability=1.0`, on in this cfg).

## Why it happens (mechanism)
1. A vehicle's route pins a specific exit lane at a junction (the best-lanes / pool target lane).
2. In dense traffic the **strategic lane change onto that exit lane never completes** (the target lane is
   full — no acceptable gap), so the vehicle reaches the junction on the wrong lane.
3. SumoSharp's junction planning finds **no valid link for its route from the current lane**, so it holds
   at the stop line (StopLineConstraint / no-valid-link). The re-resolve that would rescue it
   (`TryReResolveFromActualLane`, `Engine.cs` ~8986) only fires when the vehicle **reaches the lane end**,
   but a vehicle stopped short at the stop line never gets there → permanent stall.
4. The stalled front vehicle blocks its approach queue; several such stalls across the net gridlock it.
   Vanilla instead **reroutes off the available connection immediately**, so it never strands.

## NOT the cause (ruled out this session)
- **The just-merged lane-change-overlap fix is NOT the cause.** A/B on the dense synthetic: pre-fix main
  gridlocks essentially the same (11 teleports / halting 39) as post-fix (10 teleports / halting 45). The
  overlap fix is throughput-**neutral-to-marginal** here (a small ~6-vehicle cost from the added arrival-
  lane junction braking + LC vetoes — worth watching, not the driver). The gridlock is **pre-existing**.
- Not a yield/RoW over-conservatism per se: the stalled vehicles are not yielding to a real foe (the
  junction is idle) — they have no valid link for their route from the wrong lane.

## Candidate fixes (for the design phase — pick after a design doc)
1. **Reroute-on-wrong-lane at the stop line (most direct, matches vanilla).** When a vehicle reaches a
   junction and its current lane offers no link toward its next route edge, do what vanilla does: reroute
   via an available connection (or take the best available link and let the periodic rerouter recover),
   rather than holding at the stop line waiting for a lane change that cannot happen. Hook: generalise
   `TryReResolveFromActualLane` to fire at the stop-line approach, not only at the physical lane end.
2. **Complete the strategic exit-lane change earlier / more reliably under density** (so the vehicle is on
   the right lane before the queue forms). This is where SUMO's cooperative lane-changing (retired here as
   `informFollower`, `HIGH-DENSITY-P2G2-...`) actually IS relevant — target-lane followers opening a gap so
   the strategic change completes. Heavier; revisit only if (1) is insufficient.
3. **Best-lanes lookahead / earlier lane commitment** so the exit-lane change is initiated far enough
   upstream to complete in dense flow.

## Guardrails (CLAUDE.md iron law)
Any fix must keep every committed golden byte-identical (or within `tolerance.json`), verified by full
`dotnet test Traffic.sln`; the offline loop must not need SUMO. Success criterion (from the NEED):
on the dense synthetic (and, once Gap 2/3 unblock it, the `demo_city/box` crop) SumoSharp reaches
teleports ≈ 0, arrivals ≈ vanilla, no permanent gridlock (halting drains) — i.e. its calibrated max-density
knee matches vanilla within tolerance.

## Pointers
- Repro substrate: `scenarios/_repro/synthetic-junction2` (+ `build.py`). Drop-in: `src/Sim.Sumo`.
- Re-resolve / lane-end clamp: `Engine.cs` `ExecuteMoveVehicle` (~8960-9013), `TryReResolveFromActualLane`.
- Prior throughput residual + candidates: `docs/FOLLOWUP-TL-throughput-flowrate.md`.
- Retired cooperative LC (relevant to candidate 2): `docs/HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`.
