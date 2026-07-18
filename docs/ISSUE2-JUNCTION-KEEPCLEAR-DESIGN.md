# ISSUE2 diagnosis — the jam-teleports are caused by PARKED CARS on the travel lane, not junction RoW

**Status:** DIAGNOSED — root cause proven by experiment. **The SumoData "junction right-of-way /
gap-acceptance" framing for Issue 2 is a mis-attribution.** The 21 jam-teleports are a *downstream
consequence of the park-and-stay cars blocking the travel lane* — i.e. the same parkingArea code path as
Issue 1, owned by the sibling session on `claude/sumosharp-drop-in-binary-vq7u9p`. This branch
(`claude/sumosharp-junction-row-issue2`) is based on it. There is **no independent junction-RoW defect**.

## 1. Reproduced (deterministic)

Synthetic 8×8 priority grid, `scenarios/_repro/synthetic-parity/`, `--end 1000`, seed 42. Built the
`sumosharp` drop-in and reproduced exactly: vanilla `jam=0` (total 4: 3 yield, 1 wrongLane), SumoSharp
`jam=21`. Rerouting ruled out (rerouting off: still ss 21 jam vs van 2).

## 2. The decisive experiment

Strip the `<stop parkingArea=…>` from the 120 park-and-stay cars (they now drive through instead of
parking), keep everything else identical:

| run | jam teleports | total teleports |
|---|---:|---:|
| SumoSharp, **parking ON** (committed repro) | **21** | 21 |
| SumoSharp, **parking removed** | **0** | 0 |
| vanilla, parking removed | 0 | 5 (yield) |

**Removing parking eliminates every jam-teleport.** With no on-lane parked cars, SumoSharp flows the
priority grid cleanly — *fewer* teleports than vanilla. So Issue 2 is entirely a parking artifact.

## 3. Mechanism (FCD evidence at junction E1, t=262)

The persistent wedges concentrate on a few internal lanes — `:E1_6_0` alone has **12** vehicles stopped
on it over the run (vanilla's worst internal lane has 2, spread thin). `:E1_6_0` = connection
`F1E1 → E1D1` (straight, minor). The exit lane `E1D1_0` carries a **park-and-stay car (118)** that parks
in `pa_E1D1` (a roadside `parkingArea` at startPos 2–16 on lane `E1D1_0`) with `duration=100000`, i.e.
for the whole run. All four persistent blockers (118 on E1D1_0, 82 on G2G1_0, 73 on H5G5_0, 91 on
G1G0_0) are park-and-stay cars sitting at pos ≈6.7 for 800+ s.

- **Vanilla:** the parked car is **off the travel lane** (roadside bay). Through-traffic flows *past* it
  on `E1D1_0` (snapshot: vehicles at pos 18, 50, 91, 99 *beyond* the parked car at 6.7). The junction
  `:E1_6` stays clear; cross traffic resolves. `jam=0`.
- **SumoSharp:** `E1D1_0` holds **only** the parked car 118 (pos 6.65) and is empty ahead — through
  traffic cannot get past it, so it backs up **onto the junction** `:E1_6` (12 vehicles wedge there over
  the run), blocking every crossing movement at E1 → mutual deadlock → after 120 s, jam-teleport.

So SumoSharp's parked car **blocks the travel lane**; SUMO's does not.

## 4. Why the existing anti-block machinery doesn't save it

SumoSharp already has a faithful `KeepClearConstraint` (`Engine.cs`, ported from
`MSVehicle::checkRewindLinkLanes`) that would hold a vehicle on its approach lane when the exit is
blocked — and it works when the blocker is a normal stopped vehicle. But it **cannot see the parked
car**: `LaneNeighborQuery.Refill` excludes every `IsParked` vehicle from the per-lane buckets
(`LaneNeighborQuery.cs:76`, a GAP-3/parkingArea change), by design, so parked cars are "invisible …
exactly like SUMO's real off-lane transfer, and a following through-vehicle is never blocked by it"
(`VehicleRuntime.cs:455-465`). Verified by probe: `LaneSpaceTillLastStanding(E1D1_0)` sees `bucket=0`
though 118 is on the lane, so keepClear reads the exit as clear and releases the vehicle onto the
junction.

**The contradiction that is the bug:** the parked car is excluded from the plan-phase neighbor query
(so keepClear and ordinary car-following don't hold vehicles back), yet through-traffic still cannot pass
it — `E1D1_0` stays empty ahead of the parked car and vehicles wedge on the junction rather than driving
through. `ExecuteMoveVehicle` only applies the plan-phase speed (no execute-phase leader re-clamp), so the
speed-0 must originate in the **plan phase** — i.e. *some* consumer still treats the parked car as
occupying the travel lane even though `LaneNeighborQuery` excludes it. Candidates to audit: the
`ActiveVehicles`-based cross-junction/insertion scans (`Engine.cs:3249-3262`, which do *not* go through
the parked-excluding neighbor query), any lane-occupancy / next-lane-full check on the internal→exit
transition, and the parkingArea slot/blocking model itself. The off-lane exclusion is **incomplete**: SUMO
lifts a parked vehicle off the lane's vehicle list *entirely* (`MSVehicleTransfer::add`); SumoSharp lifts
it only from the planning snapshot, so a residual path re-introduces it as a blocker. Pinning the exact
consumer is the first step of the fix.

## 5. Concrete fix plan (parkingArea code path — Issue 1's domain)

Make an `IsParked` vehicle fully off-lane, matching SUMO: it must not block a following/through vehicle
in **any** phase — not just the plan-phase neighbor query, but also the execute-phase lane-advance /
overlap / next-lane-occupancy check and any `ActiveVehicles`-based leader/rearmost scan on the travel
lane. Concretely: audit every consumer that can stop a moving vehicle behind a longitudinal position and
confirm it applies the same `IsParked` exclusion the neighbor query does (or drop the parked vehicle from
the lane's active list the way SUMO's transfer does). With that, `E1D1_0` carries through-traffic past the
parked car (like vanilla), `:E1_6` stays clear, and the jam-teleports go to 0 — exactly the
"parking-removed" result above, but with the cars still parked.

This is in the **parkingArea / stop / off-lane residency** path, which the instructions reserve for the
sibling Issue-1 session. It is the *same* root as Issue 1 (park-and-stay residency): once parked cars
correctly vacate the travel lane, both the residency metric and the junction jams resolve. Coordinate on
who lands it.

## 6. Regression test

The right regression is a **parking test**, not a junction one: a single parkingArea on a 1-lane through
edge, a car that parks in it, and a following through-car that must pass unobstructed (assert the
follower's speed never drops to 0 at the parked car's position, and it arrives). Because that exercises
the parkingArea off-lane path, it belongs with the Issue-1 fix. A junction/keepClear golden would *not*
capture this bug (keepClear is correct; it simply — correctly — can't see an off-lane car).

## 7. Recommendation

Report to the owner (done): Issue 2 has no separate junction-RoW fix; it is the parkingArea off-lane
residency bug (Issue 1's code). Either fold it into the sibling session's Issue-1 work, or explicitly
re-scope this session to touch the parkingArea path. Do not add a junction-RoW golden or change keepClear
— both would be fixing the wrong thing.
