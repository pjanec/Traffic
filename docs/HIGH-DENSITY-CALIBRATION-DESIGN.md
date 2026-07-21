# HIGH-DENSITY-CALIBRATION-DESIGN.md — make SumoSharp a trustworthy high-density calibration engine

**Status:** DESIGN (this session, 2026-07-21). Owner asked for all three gaps fixed so SumoSharp can
**auto-calibrate the highest believable traffic density** (the sub-area pipeline's "knee"), not just
serve/run. **Self-contained on purpose** — written so the conversation can be compacted and a later pass
can implement from these docs alone. Companions: `HIGH-DENSITY-CALIBRATION-TASKS.md` (stages + success
conditions), `HIGH-DENSITY-CALIBRATION-TRACKER.md` (checklist), `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`
(Gap 1 deep-dive with measured evidence). Source NEEDs (from the SumoData session):
`SUMOSHARP-NEED-dense-flow-gridlock-vs-vanilla.md`, `SUMOSHARP-NEED-serve-calibration-parity-gaps.md`.
SUMO reference is vendored read-only at `/sumo` (v1_20_0); `SUMO_VERSION`=1.20.0; `sumo` binary present.

---

## 0. The north star and why these three
The sub-area pipeline (`preprocess.py`/`auto_calibrate.py`, in the *SumoData* repo, not here) computes the
**maximum believable insertion density** for a crop by probing densities until the teleport pop% exceeds a
budget — the "knee". It swaps the sim engine via `SUMO_BINARY`. Today it must calibrate with **vanilla
`sumo`** because SumoSharp:
- **Gap 1 (HIGH):** gridlocks/teleports at **3–5× lower density than vanilla** on the identical net —
  so its calibrated knee is 4–5× too low and untrustworthy. THE headline blocker.
- **Gap 2 (MEDIUM):** rejects the full pre-built `demo_city/box` at load — a `roadsideCapacity=1`
  parkingArea referenced by 3 vehicles across the run (static load-time lot assignment, no time reuse).
- **Gap 3 (LOW):** rejects `departPos="base"` (30 vehicles in the box use it).

Gap 1 is the behavioral blocker. Gaps 2+3 are load-time blockers that stop the **full box** loading on
SumoSharp at all (the *crop* flow already runs because it regenerates `departPos="stop"` demand and doesn't
oversubscribe capacity-1 lots). All three are needed for "SumoSharp-only auto-calibration on the full box".

## 1. Reproduction (deterministic; keep for every pass)
Substrate: `scenarios/_repro/synthetic-junction2` (committed throughput witness: TL + priority junctions,
heavy demand on short approaches; regenerate with its `build.py`). Drop-in binary: `src/Sim.Sumo`
(`sumosharp.dll`), SUMO-arg compatible.
```
# build the drop-in
dotnet build -c Release src/Sim.Sumo/Sim.Sumo.csproj
DLL=src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll
# 2x-density stress demand (compress departs; amplifies Gap 1 into clear gridlock):
python3 -c "import re;s=open('scenarios/_repro/synthetic-junction2/scenario.rou.xml').read();\
open('/tmp/dense.rou.xml','w').write(re.sub(r'depart=\"([0-9.]+)\"',lambda m:f'depart=\"{float(m.group(1))*0.5:.2f}\"',s))"
# copy net + support files to /tmp and point a /tmp/dense.sumocfg at /tmp/dense.rou.xml (see diag doc).
sumo      -c /tmp/dense.sumocfg --end 1000 --statistic-output /tmp/v_stat.xml --tripinfo-output /tmp/v_trip.xml --summary-output /tmp/v_sum.xml --no-step-log true
dotnet $DLL -c /tmp/dense.sumocfg --end 1000 --statistic-output /tmp/s_stat.xml --tripinfo-output /tmp/s_trip.xml --summary-output /tmp/s_sum.xml --no-step-log true
# compare: <teleports total/> ; count <tripinfo ; summary running/halting/meanSpeed over time.
```
**Measured today (main `8bb8219`, 2× density):** vanilla = 0 teleports / 290 arrivals / halting drains to
0; SumoSharp = 10 teleports (8 *yield*) / 275 arrivals / **~45 permanently stuck (meanSpeed 0 from
t=600)**. The offline gate (`dotnet test Traffic.sln`, no SUMO/network) must stay green throughout.

---

## 2. GAP 1 — reroute-on-wrong-lane (the dense-flow gridlock). PRIMARY.

### 2.1 Root cause (measured — full evidence in `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`)
Under dense insertion, a vehicle that needs to change into a **specific route-required exit lane** at a
junction, but cannot complete that lane change in the dense queue, reaches the junction **on the wrong
lane** — a lane with **no connection to its next route edge**. SumoSharp then **clamps it at the lane end
(stops it dead, speed 0, forever)**. The stalled front vehicle blocks its whole approach queue; a handful
across the net cascade into gridlock + yield-teleports. Vanilla instead takes the connection its actual
lane offers and **reroutes** (`device.rerouting` on) — every stalled SumoSharp vehicle (295/242/155/172/
316) arrives in vanilla; none in SumoSharp. Concrete: veh 295 route `…30→124…`, but `→124` connects only
from lane `30_0`; 295 is stuck on `30_1` (pos 24.1, speed 0, t=520→680) while junction 29 sits **idle**
(0 crossers). It is NOT yield/RoW (junction empty), NOT car-following, NOT the landed overlap fix (A/B:
pre-fix main gridlocks the same — the overlap fix costs only ~6 veh here from its arrival-lane junction
braking; watch but not the driver).

### 2.2 The exact fix locus
`Engine.cs` → `TryReResolveFromActualLane(v, currentLane)` (~line 9051), called from `ExecuteMoveVehicle`
(~8960-9013) when a vehicle reaches a lane end on a lane ≠ its pool exit lane. Line ~9080:
```csharp
// Genuine drop lane: the actual lane has no connection to the next route edge -> clamp.
if (!_network!.ConnectionsByFromLaneTo.ContainsKey((currentLane.EdgeId, currentLane.Index, remaining[1])))
{
    return false;   // <-- caller then CLAMPS: v.Pos = laneLength; v.Speed = 0; (permanent stall)
}
```
This `return false` is where veh 295 dies. There is a SECOND path to the same stall: a vehicle can halt
at the stop line *before* reaching the lane end if its lane offers no link toward its next route edge
(planning finds no valid outgoing link) — so the reroute must be reachable from the **approach**, not only
at the physical lane end.

### 2.3 HOW (design)
Mirror vanilla: when a vehicle's current lane cannot reach its next route edge, **reroute via a
connection the current lane DOES have**, instead of clamping. Concretely, a new helper
`TryRerouteFromDeadLane(v, currentLane)`:
1. Enumerate the outgoing connections `currentLane` actually has (`ConnectionsByFromEdgeLane[(edgeId,
   laneIndex)]` → candidate next edges, e.g. `30_1 → {44, 112, -30}`).
2. For each candidate next edge, ask the router (`_router.Route(candidateNextEdge, destEdge, avoid)` — the
   same `NetworkRouter`/astar `UpdateReroutes` uses) whether the destination is reachable. Pick the
   candidate whose `currentEdge → candidateNextEdge → …dest` is the cheapest/best (matches SUMO's
   "best link" pick, `MSLane::getLinkTo`/best-lanes continuation; a simple router-cost min is adequate and
   deterministic).
3. Splice the new route (`currentEdge` + routed tail) into the pool via the existing `ReplaceRoute` +
   `_laneSeqPool.AddRange` discipline (`RegisterRerouted` keeps `_routesById`/best-lanes in sync), exactly
   as `UpdateReroutes`/`TryReResolveFromActualLane` already do — pin this edge's exit to `currentLane.Index`
   (`forceFirstExitToArrival:true`) so the vehicle proceeds via the connection its lane has.
4. Only if NO candidate connection routes to the destination (a true dead end) fall back to the clamp.
Wire it at BOTH: (a) the `return false` at ~9080 (lane-end drop), and (b) the approach — add a
plan-phase check so a vehicle whose current lane has no link toward its next route edge triggers the same
reroute *before* it stop-line-halts (so a stalled front vehicle recovers within a step, not never). Gate
the approach-side trigger to fire only when a strategic change onto the exit lane is no longer possible
(e.g. within N metres of the junction and blocked), so it does not pre-empt a normal, completing lane
change.

**Determinism / parity:** the router is a pure function of the immutable net + edge weights; the reroute
uses the established command-buffer/pool-append discipline (order-independent). No committed golden reaches
the drop-lane clamp today (they never strand — verified byte-identical before this session's overlap fix),
so replacing clamp→reroute is byte-identical on all goldens. Gate = full `dotnet test Traffic.sln` green +
`Sim.Bench` determinism hash unchanged. If any golden moves, the trigger is over-firing — tighten it.

**Escalation (only if 2.3 is insufficient):** SUMO also *completes* the exit-lane change more reliably via
cooperative lane-changing (target-lane followers open a gap — the retired `informFollower`,
`HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`). Prefer the reroute (cheap, matches vanilla's observed
behavior here); revisit cooperative LC only if reroute alone leaves material gridlock. Also consider
earlier/longer best-lanes lookahead so the exit-lane change starts far enough upstream to complete.

### 2.4 Success criteria (Gap 1)
On the 2× dense synthetic: SumoSharp teleports ≈ 0 (was 10), no permanent gridlock (halting drains toward
0 like vanilla, not stuck at ~45), arrivals ≈ vanilla (≈290, was 275). Full `dotnet test Traffic.sln`
green + goldens byte-identical (or provably SUMO-faithful with a provenance bump). A new committed
`_repro` or `scenarios/` anchor that pins "wrong-lane vehicle reroutes instead of stalling".

---

## 3. GAP 2 — parkingArea time-based lot reuse

### 3.1 Root cause
`Engine.ResolveParkingAreaStops` (`Engine.cs` ~1223-1310) assigns each parkingArea occupant a **distinct**
lot index across the WHOLE run via a monotonic `nextLotIndexByPa` counter (two passes: departPos="stop"
origins first, then moving-vehicle stops, both in vehicle-list order). It has **no notion of a lot being
freed when its occupant departs**. So a `roadsideCapacity=1` lot referenced by 3 vehicles over the run is
assigned lot indices 0,1,2 → `ParkingArea.LotPosition` (`Sim.Ingest/ParkingArea.cs`) throws *"lot index 1
is out of range"* at load. This is a deliberate GAP-3 scope limit (`docs/SERVE-PATH-PLAN.md §3a`) — only
non-overlapping-in-time single-occupant shapes were in scope.

### 3.2 SUMO reference
`MSParkingArea::computeLastFreePos` assigns the **lowest-index currently-free lot** at the moment a vehicle
parks, and frees it when the vehicle departs (real-time turnover). A `roadsideCapacity=N` lot serves any
number of vehicles across time as long as **≤ N are parked simultaneously**.

### 3.3 HOW (design) — two options; recommend A
**Option A (faithful, dynamic runtime assignment) — RECOMMENDED for the calibration robustness goal.**
Move lot assignment from load-time to **runtime**: when a vehicle actually parks (reaches its
parkingArea stop), claim the lowest-index free lot of that area from a per-area free-set; when it departs
(stop ends), return the lot to the free-set. This is `MSParkingArea::computeLastFreePos` verbatim.
- New runtime state: per-parkingArea `bool[capacity] occupied` (or a min-heap of free indices), in Engine.
- On park (the stop-reached transition, where `IsParked` is set): claim `lowestFreeLot`; set the vehicle's
  parked lateral/longitudinal lot position from `ParkingArea.LotPosition(lotIndex)`; if the area is full,
  SUMO reroutes to an alternative parkingArea (`parkingAreaReroute`) or the vehicle waits — for scope,
  match SUMO's default (wait/queue on the lane) or reject only if genuinely unsatisfiable.
- On depart: free the lot.
- Determinism: claim/free in the deterministic per-step order the engine already uses; the free-set pick is
  "lowest index", order-independent. Keep the committed single-occupant parking goldens (48/66-70)
  **byte-identical**: with ≤ capacity simultaneous occupants the lowest-free-lot pick reproduces the current
  static lot indices exactly (verify).

**Option B (cheaper, static interval scheduling).** Keep load-time assignment but make it interval-aware:
estimate each occupant's `[arrival, departure)` window (departure = arrival + stop `duration`/`until`;
arrival ≈ stop-reached time, which for departPos="stop" origins is t=departure-time and for moving vehicles
needs a rough travel estimate) and assign the lowest lot free during that window. Simpler but approximate
and fragile on arrival-time estimation. Use only if A proves too invasive.

**REFINEMENT (measured 2026-07-21 — locks Option A, rejects B).** The box's failing area
`pa_e_d_2_2_d_3_2` (cap 1) is used by 3 vehicles whose vanilla `--stop-output` PARK intervals are
`veh0 [345,546]`, `veh1 [590,738]` — reusing lot 0 (both park at pos 231.40) — with the 3rd never
reaching it. Crucially the actual **park** times (345, 590) are ~285 s after the **depart** times
(59.79, 223.76): drive + jam. So load-time interval estimation (B) is unreliable (depart+freeflow ≈ 60
vs real park 345), and even depart-window overlap ([60,261] vs [224,372]) would falsely force distinct
lots and overflow. **Only true runtime park-time assignment (A) is correct.** Implemented mechanism:
- StopRuntime for a parking stop carries `ParkingAreaId` + `AssignedLot` (−1 = unclaimed); its `EndPos`
  (the brake target) is resolved at RUNTIME, not baked at load. `ResolveParkingAreaStops` stops assigning
  lot indices — it only fills `LaneId`/`StartPos`/`ParkingAreaId` (no `LotPosition` call at load, so an
  oversubscribed area no longer throws).
- Engine runtime state: `Dictionary<string,bool[]> _parkingLotOccupied` per area (capacity-sized).
- **Plan (read-only, start-of-step snapshot):** `StopLineConstraint` for an unclaimed parking front-stop
  computes the provisional lot = lowest `i` with `!occupied[i]` and brakes toward `LotPosition(i)`; if the
  area is FULL it brakes toward `StartPos` and the reached-gate below refuses to mark it reached (wait-
  when-full, SUMO's "no free pos" path). A claimed stop uses `LotPosition(AssignedLot)`.
- **ProcessNextStop reached-gate:** a parking stop is marked reached only if a lot is free/assigned (so a
  full area makes the vehicle wait at `StartPos` instead of parking on air).
- **Execute (deterministic order, mutates occupancy):** on the reached transition (the existing
  `stop.IsParking → v.IsParked=true` block) claim the live lowest-free lot, set `AssignedLot` + `EndPos`,
  mark occupied; on the resume transition (the existing `resumedStop.IsParking → v.IsParked=false` block)
  free the lot. departPos="stop" origins claim at insertion (they insert already-parked at `LotPosition`).
- **Parity:** committed goldens (48/66-72) never reuse and never oversubscribe → snapshot lowest-free ==
  the old static index at every step, so brake target + parked pos are byte-identical (verified by the
  full suite). Determinism: Plan reads the frozen snapshot; Execute claims/frees in the engine's existing
  per-step order; the pick is "lowest index" (order-independent). Anchor: `scenarios/76-parking-lot-reuse`
  (cap-1, veh0 pulls out then veh1 reuses lot 0 @ pos 210).

### 3.4 Success criteria (Gap 2)
The full `demo_city/box` (`scenarios/_ped/demo_city/box/scenario.sumocfg`) LOADS on SumoSharp (no "lot
index out of range") — combined with Gap 3 it runs end-to-end. Committed parking goldens (48/66/67/68/69/
70) stay byte-identical. A new anchor: a `roadsideCapacity=1` area referenced by ≥2 non-overlapping-in-time
vehicles loads and both park correctly (matching SUMO lot positions/tripinfo).

---

## 4. GAP 3 — departPos="base" (and confirm other symbolic departPos)

### 4.1 Root cause
`DemandParser.ParseDepartPos` (`Sim.Ingest/DemandParser.cs` ~560) accepts only a numeric literal or
`"stop"`; `"base"` (and `random`/`free`/`last`/…) throw *"unsupported departPos"*. 30 box vehicles use
`"base"`.

### 4.2 SUMO reference (exact — do NOT just map to "0")
`MSBaseVehicle::basePos(edge)` (`/sumo/src/microsim/MSBaseVehicle.cpp:1117`):
```cpp
double result = MIN2(getVehicleType().getLength() + POSITION_EPS, edge->getLength());
if (hasStops() && stops.front().edge == route.begin() && stops.front().lane->edge == route.begin())
    result = MIN2(result, MAX2(0.0, stops.front().getEndPos()));   // capped by a first-edge stop
return result;
```
`DepartPosDefinition::BASE` and `DEFAULT` both resolve to `basePos` (`MSLane.cpp:699-720`, non-SPLIT arm).
`POSITION_EPS` is SUMO's small epsilon (`StdDefs.h`, 0.1). So `"base"` = vehicle front bumper at
`MIN(vTypeLength + 0.1, edgeLength)`, further capped to a first-edge stop's endPos. The NEED doc's
"base→0 is bit-identical" is only true when that MIN/stop cap collapses to ~0 for the box's shapes — do the
**faithful formula**, not a hardcoded 0, so it is correct for every shape.

### 4.3 HOW (design)
- Add `DepartPosSpec.Base` to `DepartValue.cs` (`DepartPosValue.Base`), and accept `"base"` in
  `ParseDepartPos` → `DepartPosValue.Base`.
- Resolve it at INSERTION (`Engine.TryInsertOnLane`/the departPos resolution arm, where `Stop`/numeric are
  resolved today) using the faithful `basePos`: `MIN(vType.Length + PositionEps, lane.Length)`, capped to
  the first stop's endPos when the first stop is on the depart edge. Reuse the same `PositionEps` the
  parkingArea/stop code already uses.
- Keep every OTHER symbolic value (`random`/`free`/`last`/`random_free`/`speedLimit`) throwing (out of
  scope) — but the throw message can note they are unported, not "unsupported forever".

### 4.4 Success criteria (Gap 3)
A vehicle with `departPos="base"` (with and without a first-edge stop) inserts at SUMO's `basePos`
position (verify against a tiny SUMO FCD @1e-3). The 30 box vehicles no longer block the load. Committed
goldens byte-identical (none uses `"base"` today, so the new arm is inert there).

---

## 5. Global guardrails (CLAUDE.md iron law — applies to all three)
- **Parity:** every committed golden stays byte-identical (or within its `tolerance.json`); gate = full
  `dotnet test Traffic.sln` green on a fresh clone WITHOUT SUMO, plus a full-sweep engine-FCD byte-diff of
  the multi-lane/junction/parking goldens (the method used for the overlap fix — generate parity-mode FCD
  before/after, `diff`). `Sim.Bench` determinism hash unchanged.
- **Determinism:** no `System.Random`; per-vehicle/immutable state only; serial == region-parallel
  byte-identical (the reroute reads the immutable net + frozen snapshot, writes ego's own route slice via
  the command buffer).
- **Follow SUMO** (rule 4): reroute-on-dead-lane mirrors `ignore-route-errors + device.rerouting`;
  parking mirrors `computeLastFreePos`; base mirrors `basePos`. Deviate only where ECS structurally forces.
- **Design-first, staged:** land Gap 3 → Gap 2 → Gap 1 (increasing risk/size), each with its own anchor +
  green suite before the next. Gap 1 may need multiple passes (reroute → measure → cooperative-LC only if
  needed).

## 6. Order of work + file map
1. **Gap 3** (small, unblocks box load w/ Gap 2): `DepartValue.cs`, `DemandParser.cs`, `Engine.cs`
   (insertion departPos arm).
2. **Gap 2** (medium, unblocks box load): `Engine.cs` (`ResolveParkingAreaStops` → runtime lot manager),
   `ParkingArea.cs` (LotPosition unchanged; assignment moves to runtime).
3. **Gap 1** (large, the density fix): `Engine.cs` (`TryReResolveFromActualLane` drop-lane branch +
   `ExecuteMoveVehicle` + a plan-phase approach trigger + new `TryRerouteFromDeadLane`). Re-measure the
   dense synthetic every iteration; add the anchor; keep the suite green.

## 7. Pointers (code + refs)
- Gap 1: `Engine.cs` `TryReResolveFromActualLane` (~9051, drop-lane clamp ~9080), `ExecuteMoveVehicle`
  (~8960-9013), `UpdateReroutes`/`RegisterRerouted`/`ReplaceRoute` (reroute discipline), `NetworkRouter`.
  Evidence: `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`. Prior residual: `FOLLOWUP-TL-throughput-flowrate.md`.
- Gap 2: `Engine.cs` `ResolveParkingAreaStops` (~1223-1310), `Sim.Ingest/ParkingArea.cs`
  (`LotPosition`, capacity check ~30-45). SUMO: `MSParkingArea::computeLastFreePos`. Scope note:
  `docs/SERVE-PATH-PLAN.md §3a`.
- Gap 3: `Sim.Ingest/DemandParser.cs` `ParseDepartPos` (~560), `Sim.Ingest/DepartValue.cs`
  (`DepartPosSpec`/`DepartPosValue`), `Engine.cs` insertion departPos arm (search `DepartPosSpec.Stop`).
  SUMO: `MSBaseVehicle::basePos` (`/sumo/src/microsim/MSBaseVehicle.cpp:1117`), `MSLane.cpp:699`.
- Repro: `scenarios/_repro/synthetic-junction2`; full box: `scenarios/_ped/demo_city/box`. Drop-in:
  `src/Sim.Sumo`. Branch for this work: `claude/dense-flow-throughput-diag` (restart from `origin/main`).
