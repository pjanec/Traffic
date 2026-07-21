# GAP1-RESUME.md — single entry point to finish the dense-flow gridlock fix (Gap 1)

**Written 2026-07-21 to survive context compaction.** Self-contained: a fresh session should be able to
resume from this file alone. Companions with the deep detail: `HIGH-DENSITY-CALIBRATION-DESIGN.md`
(esp. **§2.3.4 — the real SUMO mechanism**, and §2 overall), `DENSE-FLOW-THROUGHPUT-DIAGNOSIS.md`,
`HIGH-DENSITY-CALIBRATION-TRACKER.md`. SUMO source is at `/sumo` (read-only, tag v1_20_0).

---

## RESUME PROMPT (paste this to restart)

> Resume Gap 1 (dense-flow gridlock) on branch `claude/dense-flow-throughput-diag`. Read
> `docs/GAP1-RESUME.md` first, then `docs/HIGH-DENSITY-CALIBRATION-DESIGN.md §2.3.4`. Gaps 2+3 are DONE
> and merged into the branch; a **safe partial** for Gap 1 (gated dead-lane reroute) is landed. The root
> cause and the exact SUMO-faithful fix are already diagnosed — **do not re-investigate, implement.** The
> fix: make a vehicle's junction continuation follow its ACTUAL current lane's connection when that lane
> cannot reach its next route edge (port SUMO's `getBestLanesContinuation` "continue along the lane you're
> on" semantics), instead of pinning the route's ideal exit lane and holding the car at the junction. This
> is byte-identical for every committed golden by construction. Work on a fresh experimental branch off
> `claude/dense-flow-throughput-diag`; keep it there if it breaks goldens but improves 2× drainage. Verify
> with the repro + numbers in this doc. DO NOT retry the dead ends listed here (congestion-reroute tuning,
> cooperative LC, instant-vs-gated reroute tuning) — they are proven not to work.

---

## 1. State of play (branch `claude/dense-flow-throughput-diag`, HEAD ~`d9efd6c`)
- **Gap 3 (departPos="base")** — DONE. Anchor `scenarios/75-base-depart`. Faithful `MSBaseVehicle::basePos`.
- **Gap 2 (parkingArea runtime lot reuse)** — DONE. Anchor `scenarios/76-parking-lot-reuse`. The full
  `scenarios/_ped/demo_city/box` now LOADS + runs end-to-end (was rejected at load).
- **Gap 1 (dense-flow gridlock)** — SAFE PARTIAL LANDED + root cause fully diagnosed. This doc is about
  finishing it.
- Full offline suite: **656 pass, 4 skip, 0 fail**; all committed goldens byte-identical; deterministic.
  Always keep it that way: `dotnet test Traffic.sln` (no SUMO/network needed).

## 2. THE ROOT CAUSE (one paragraph — this is the whole thing)
Under density a car is forced onto a lane whose connections do NOT include its next route edge (e.g. veh
295 enters edge `30` on `30_1` because the sole `25/…→30` connection lands it there, but its route needs
`30→124` which leaves **only from `30_0`**; edge 30 is just 24.12 m). **SUMO** plans that car's move along
its ACTUAL lane's own continuation (`MSVehicle::getBestLanesContinuation` → `myLFLinkLanes` in
`planMoveInternal`): it simply takes `30_1→44` (the connection its lane HAS) and **crosses the junction
while still moving**, never waiting for the impossible `30_1→124`; the `device.rerouting` device then
repairs the now-off-route car. **SumoSharp** instead PINS the pool's ideal exit lane (`30_0`) and HOLDS
the car at the junction when it can't converge onto it — so the car stops, blocks its `30_1` queue, and a
handful of these across the net cascade into the gridlock + teleports.

## 3. THE FIX TO IMPLEMENT (SUMO-faithful — do this)
Make the vehicle's junction continuation follow its ACTUAL current lane when that lane cannot reach the
next route edge, rather than pinning the route's exit lane and holding. Concretely: when a car is
approaching a junction on a lane `L` whose connections do not include its next route edge, resolve its
continuation via `L`'s own connection (like `getBestLanesContinuation` does — "continue along the lane
you are on"), so the PLAN phase sees a valid outgoing link (`30_1→44`) and the car flows through while
moving; then the existing (already-faithful) periodic reroute repairs the route from the new edge.

**Why this is byte-identical for goldens (the parity argument):** every committed golden's car is ALWAYS
on its pool exit lane at each junction (they never strand), so "continue along the actual lane" ≡
"continue along the pool exit lane" — the new branch is never taken. Guard: full `dotnet test` green +
`Sim.Bench` determinism hash unchanged + serial == region-parallel.

### Code loci (methods; line numbers drift, search by name) — all in `src/Sim.Core/Engine.cs`
- **`ExecuteMoveVehicle`** (~L8903): the boundary-crossing `while (pos >= laneLength)` loop calls
  `TryReResolveFromActualLane` (~L9227) when the car reaches a lane end NOT on its pool exit lane; the
  drop-lane branch there now calls **`TryRerouteFromDeadLane`** (~L9318, the landed gated reroute). This
  is the REACTIVE-at-lane-end path — the fix needs the continuation chosen on APPROACH (Plan), not here.
- **`TryStrategicLaneChange`** (~L10263): resolves best-lanes / pool continuation and the strategic LC
  toward the pool exit lane. Around L10290-L10319 it reads the car's front stop / pool target
  (`stopDistOverride`). This + how the pool exit lane is chosen is where "follow the actual lane" belongs.
- **`StopLineConstraint`** (~L6020) and the junction-yield/link constraints (search `JunctionYield`,
  `approachLane`, `EgoLinkHasSignalPriority`): where the car brakes at a junction. The fix must make a
  dead-lane car NOT brake-to-hold here (it should have a valid link toward its actual lane's connection).
- **`ResolveLaneSequenceHandlesWithArrival`** (in `NetworkModel`, called from many sites incl. L3553,
  L9281, L9426, `RerouteActive` L2663, `UpdatePeriodicReroutes` L4439): builds the pool/arrival lane
  sequence from an edge list + a lane index. `forceFirstExitToArrival:true` pins the first edge's exit to
  a given lane — the existing hook the reroute uses; the continuation fix will lean on the same machinery.
- **`UpdatePeriodicReroutes`** (~L4322) + `RerouteEdgeWeights` (`src/Sim.Ingest/RerouteEdgeWeights.cs`):
  the periodic congestion reroute — **already a faithful port, DO NOT "fix" it** (see dead ends).
- SUMO reference to port: `/sumo/src/microsim/MSVehicle.cpp` — `getBestLanesContinuation` (~L6236),
  `planMoveInternal` (~L1315 builds `myLFLinkLanes` from the continuation), `executeMove` (~L4344 the
  "no connection to the next edge" fallback → only when the lane has NO onward link at all). Also
  `MSVehicle::updateBestLanes` for how the per-lane continuation + `bestLaneOffset` are computed.

After the continuation fix works, the reactive `TryRerouteFromDeadLane` can likely be simplified to just
the route-repair, or removed in favour of the periodic reroute (measure before removing).

## 4. DEAD ENDS — do NOT retry these (each is proven not to work)
1. **Congestion-reroute tuning.** It is NOT congestion. vanilla's own `--device.rerouting.output` shows
   the `124` corridor is CHEAPER than `44` (124=1.74, 148=0.29 vs 44=3.16, 55=3.92); a cost router keeps
   124. SumoSharp's periodic rerouter correctly keeps 124 too — it is a faithful port, not the bug.
2. **Cooperative lane-change (`informFollower`/LCA_COOPERATIVE).** RETIRED in this repo for degrading
   organic flow (`HIGH-DENSITY-P2G2-COOPERATIVE-LC-DESIGN.md`); re-adding would most likely hurt. And it
   is NOT what vanilla does for veh 295 (vanilla reroutes/takes-44, it does not open a gap into `30_0`).
3. **Instant-vs-gated dead-lane reroute tuning.** Fully swept. INSTANT reroute drains 2× but churns 1×
   (net **−14 arrivals**: saves 3 genuine strands, LOSES 17 previously-fine cars — quantified) and fails
   the ≤5 low-density teleport guard. ANY gate ≥1 step keeps 1× clean but the 2× jam persists (gated car
   unsticks ~1/sec, slower than dense arrivals fill the queue). Mutually exclusive **for the reroute
   lever alone** — that lever is exhausted; the landed gated version (`DeadLaneRerouteWaitSeconds=5`) is
   its safe optimum.
4. **Minimal-deviation reroute (rejoin the route ASAP).** Not faithful: vanilla veh 295 takes a fully
   different corridor (`44 55 69 109 173 -1119 -508 506`) and rejoins only at `828`.

## 5. Reproduction (regenerate from committed files; ~2 min)
```
# build the drop-in
dotnet build -c Release src/Sim.Sumo/Sim.Sumo.csproj
DLL=src/Sim.Sumo/bin/Release/net8.0/sumosharp.dll
# 2x-density stress demand from the committed throughput witness:
D=/tmp/densej2; rm -rf $D; mkdir -p $D
cd scenarios/_repro/synthetic-junction2
cp grid.net.xml scenario.add.xml vType.config.xml vType_pedestrians.xml vTypeDist.config.xml $D/
python3 -c "import re;s=open('scenario.rou.xml').read();open('$D/scenario.rou.xml','w').write(re.sub(r'depart=\"([0-9.]+)\"',lambda m:f'depart=\"{float(m.group(1))*0.5:.2f}\"',s))"
sed 's#</configuration>#  <output/>\n</configuration>#' scenario.sumocfg > $D/dense.sumocfg
cd /home/user/SumoSharp
# run + measure (teleports, arrivals, halting/meanSpeed tail):
sumo      -c $D/dense.sumocfg --end 1000 --statistic-output $D/v_st.xml --tripinfo-output $D/v_ti.xml --summary-output $D/v_su.xml --no-step-log true
dotnet $DLL -c $D/dense.sumocfg --end 1000 --statistic-output $D/s_st.xml --tripinfo-output $D/s_ti.xml --summary-output $D/s_su.xml --no-step-log true
# teleports: grep 'teleports total' *_st.xml ; arrivals: grep -c '<tripinfo ' *_ti.xml
# halting/meanSpeed tail: grep -oE 'halting="[0-9]+"[^>]*meanSpeed="[0-9.-]+"' *_su.xml | tail
```
Also useful: `--vehroute-output` (SUMO) shows each car's reroutes; `--device.rerouting.output` (SUMO)
dumps per-edge travel-times per interval; vanilla `--device.rerouting.probability 0` turns rerouting off
(→ 10 teleports, proving rerouting's role). Also run the **1× case** (the committed
`scenarios/_repro/synthetic-junction2/scenario.sumocfg` unmodified, `--end 2000`) — it is guarded by
`tests/Sim.ParityTests/LowDensityTeleportTests.cs` (asserts teleports ≤ 5).

## 6. Numbers: baseline, current, target
2× dense synthetic (compressed depart, 325 vehicles), t=1000:
| | teleports | arrivals | halting tail | meanSpeed |
|---|---|---|---|---|
| vanilla SUMO 1.20.0 | **0** | **290** | drains to ~34 (parked sinks) | flows |
| SumoSharp baseline (pre-Gap1) | 10 | 275 | **stuck ~45** | **0 (gridlock)** |
| SumoSharp NOW (gated reroute, landed) | **3** | **281** | ~42 | 0 (partial) |
| **TARGET (this fix)** | **≈0** | **≈290** | drains | > 0 |

1× (committed cfg, `--end 2000`): baseline & now = **5 teleports / 287 arrivals** (guard ≤5). The fix
**must keep 1× here** (byte-identical goldens ⇒ 1× should not move at all).

Vanilla control: rerouting ON = 0 tp / 290 arr; rerouting OFF = 10 tp / 289 arr (== SumoSharp) — confirms
the missing piece is the smooth cross-then-repair, not extra caution.

## 7. Success criteria (definition of done for Gap 1)
1. 2× dense synthetic: teleports ≈ 0 (was 10), halting drains (no permanent stuck), arrivals ≈ vanilla
   (≈290). 1× stays at 5 tp / 287 arr (guard green).
2. Full `dotnet test Traffic.sln` green; every committed golden byte-identical; `Sim.Bench` determinism
   hash unchanged; serial == region-parallel.
3. A new committed anchor pinning "a car whose lane can't reach its next route edge crosses via its lane's
   connection while moving (does not hold/gridlock)".
4. Bonus: the full `demo_city/box` teleports drop toward vanilla (end-to-end calibration, Stage 4).
