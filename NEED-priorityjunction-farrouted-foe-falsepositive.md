# NEED — priority-junction yield: `FindFoeVehicle` matches a foe ANYWHERE on its route, not on
# its actual approach

**For the SUMO parity coding session.** Found during the scaled-city benchmark (`BENCHMARK_SPEC.md`
/ `VIZ_BENCH_TASKS.md` VB-8 rungs 2-4), not fixed here (benchmark work must not touch `Sim.Core`
per `CLAUDE.md`; this is a report-and-flag, not a patch). This is a **correctness bug**, not a
capacity/gridlock finding: SUMO runs the identical net+demand at ~free flow
(`meanSpeedRelative=0.98`, 1 halting vehicle) while the engine gets 58.8% of departed vehicles
permanently stuck on the SAME input. It is scale-dependent (network size × concurrent-vehicle
count), which is why the small anchor scenarios and the `city-30` bring-up rung never caught it.

## One-line ask

`FindFoeVehicle` (and its `IndexOfLaneHandle` foe-lookup) must only treat a vehicle as an
"approaching foe" for a junction link if that vehicle's *next few lanes* actually lead to the foe
internal lane soon — not merely because the foe internal lane appears **somewhere in the
vehicle's full route-long lane sequence**, however many junctions and however far away.

## Reproduced (not guessed) on `claude/spec-docs-review-qgwatc`

Scenario: `scenarios/_bench/city-300/` (`netgenerate --grid --grid.number=24 --grid.length=500 -L 1
--tls.guess --seed 42`, `randomTrips.py --fringe-factor 5 --seed 42`, target concurrency 300,
`END=900`). `Sim.BenchCity` run:

```
dotnet run --project src/Sim.BenchCity -c Release -- scenarios/_bench/city-300 \
  --sumo-summary scenarios/_bench/city-300/summary.xml \
  --sumo-tripinfo scenarios/_bench/city-300/tripinfo.xml \
  --aggregate-tolerance scenarios/_bench/city-300/aggregate-tolerance.json
```

Result: 723 vehicles departed, only **46 arrived** (SUMO reference: 238/240 arrived on the same
demand), **425 EVER stuck** (>=120s continuously at <0.1 m/s), **404 still stuck at sim end**.
SUMO's own `--summary-output` for the identical net+routes shows `meanSpeedRelative=0.98`,
`halting=1` at the final step -- i.e. SUMO sees essentially free-flow traffic. The engine's
divergence is not "the network is genuinely near capacity" (SUMO proves it isn't); it is an
engine-side false-positive foe.

### Concrete stuck vehicle, traced from `engine.fcd.xml`

Vehicle `281`, route `M12M13 M13M14 M14M15 M15L15 L15K15 K15J15 J15I15 I15I16`:

- Enters lane `M14M15_0` (length 485.60 m) doing 13.89 m/s (free speed) at t=425.
- Reaches `pos=485.499` (i.e. the very end of the lane, at the junction M15 stop line) by t=462.
- Speed goes to **exactly 0 and never recovers** through t=899 (the run's last emitted step) --
  437+ continuous seconds stationary, 0.1 m from the stop line, well inside the 4.5 m
  foe-visibility distance the engine's own cautious-approach code uses, so it is NOT still in the
  "can't see the foe lanes yet" phase.
- Its move is `M14M15 -> M15L15`, a LEFT turn. `net.net.xml`'s junction M15
  `<request index="10" response="0000000000000111" .../>` (matches connection
  `M14M15 -> M15L15 via=":M15_10_0" dir="l" state="m"`) says link 10 yields to links 13/14/15,
  which are ALL FOUR of `L15M15`'s outgoing connections (right/straight/left/U-turn) -- i.e. this
  junction's netgenerate-assigned priority makes the entire west approach "major" relative to this
  specific left turn, regardless of which direction a west-approach vehicle is itself heading.
- **This is the exact bug surface**: any vehicle ANYWHERE in the whole 24x24-junction, ~723-vehicle
  network whose route happens to include lane `L15M15_0` (or its internal continuations) at ANY
  point in its journey -- even a vehicle that departed minutes ago on the opposite side of the
  city and won't reach M15 for a long time yet -- is picked up by `FindFoeVehicle` as an
  "approaching foe" and forces vehicle 281 to hold at the stop line. On a small net (city-30, 9
  junctions, ~289 total vehicles across the whole 600s run) the chance of some far-flung vehicle's
  route happening to pass through a given lane at any moment is low, so this rarely bites and
  `city-30`'s reported "0 stuck vehicles" is real but not representative. On a 576-junction net
  with hundreds of concurrent vehicles, essentially every non-trivial approach lane has SOME
  vehicle whose route crosses it eventually, so the false-positive fires almost everywhere,
  almost always -- which is exactly the 58.8%-stuck outcome observed.

## Root cause (read, don't guess)

`src/Sim.Core/Engine.cs`:

```csharp
// Engine.cs:2279
private VehicleRuntime? FindFoeVehicle(VehicleRuntime ego, ActiveVehicleQuery allVehicles, int foeInternalLaneHandle)
{
    foreach (var other in allVehicles)
    {
        if (ReferenceEquals(other, ego)) continue;
        if (IndexOfLaneHandle(other, foeInternalLaneHandle) >= 0)   // <-- searches the OTHER
            return other;                                          //     vehicle's ENTIRE route
    }
    return null;
}

// Engine.cs:2302
private int IndexOfLaneHandle(VehicleRuntime v, int laneHandle)
{
    for (var i = 0; i < v.LaneSeqLen; i++)        // <-- v.LaneSeqLen spans the vehicle's FULL
        if (_laneSeqPool[v.LaneSeqStart + i] == laneHandle) return i;   //     precomputed route,
    return -1;                                                          //     insertion->arrival
}
```

`JunctionYieldConstraint` (`Engine.cs:1550-1583`) then does:

```csharp
var foe = FindFoeVehicle(v, allVehicles, foeInternalLaneHandle);
if (foe is null) continue;
var foeInternalSeqIndex = IndexOfLaneHandle(foe, foeInternalLaneHandle);
...
else if (foeInternalSeqIndex > foe.LaneSeqIndex)
{
    // "Approaching (foe hasn't reached its own internal lane yet)"
    thisConstraint = egoOnInternal ? +inf : StopSpeedFor(..., approachLane.Length - v.Kinematics.Pos - EPS, ...);
}
```

`foe.LaneSeqIndex` is the foe's CURRENT progress pointer into its own route; `foeInternalSeqIndex`
is simply the position of the foe internal lane somewhere in that route array. Any foe whose
route hasn't YET reached that lane satisfies `foeInternalSeqIndex > foe.LaneSeqIndex` -- true for
a vehicle that is five junctions and two kilometers away just as much as one that is 20 m from
entering the conflicting lane. There is no proximity/time-window filter at all: no check on
`foeInternalSeqIndex - foe.LaneSeqIndex` (how many hops away), no distance-to-junction estimate,
no arrival-time comparison. Real SUMO's equivalent (`MSLink::opened()` / the `myApproaching` map
populated by `MSLink::setApproaching`) only registers a vehicle as "approaching a link" once it is
within that link's own lookahead/braking distance of the link itself -- never merely because the
link is somewhere in the vehicle's remaining route.

## Why this wasn't caught by the parity ladder

Every anchor scenario that exercises `JunctionYieldConstraint` (`11-priority-junction`,
`19-onramp-merge`, etc.) has short routes and only ONE or TWO junctions in play, so "some vehicle's
route includes this internal lane" and "some vehicle is actually near this junction soon" are
effectively the same fact there -- the bug is invisible at that scale. It only diverges once routes
are long (many hops) and the network has enough concurrent traffic that far-away route-inclusion
becomes common. `city-30`'s bring-up (9 junctions, ~40 concurrent) sits right at the edge where it
mostly doesn't fire; `city-300`+ (576 junctions) is well past it.

## Suggested fix shape (for the parity session, NOT applied here)

Restrict the "approaching" match to foes within a bounded lookahead of the foe internal lane --
e.g. only consider `other` a foe if `foeInternalSeqIndex - other.LaneSeqIndex` is small (on the
order of "currently on the immediate approach lane to that junction", matching SUMO's
`setApproaching` registration point), or better, port `MSLink::setApproaching`'s actual
registration (foe registers itself as approaching a specific link once within a
lookahead/braking-distance window, and deregisters on `removeApproaching`/entry) instead of
re-deriving "approaching" from a route-membership scan every step. This is a **parity-track**
change (touches the already-verified 9b-ii/iii/C3/C4 family) -- needs the standard anchor +
golden + parity-reviewer gate per `HANDOFF.md`, not a benchmark-side workaround.

## Impact on the benchmark (why rungs 2-4 report high stuck-counts as the headline finding)

This bug's severity scales with (a) network size/junction count and (b) concurrent vehicle count
-- exactly the two things the scaled-city benchmark's higher rungs deliberately increase. It is
expected (and reported honestly in `scenarios/_bench/SCALING.md` and each rung's `NOTES.md`, per
`CLAUDE.md`'s "report a parity failure" / the benchmark briefing's "stability is a real outcome,
not a given") that the engine's stuck-count grows sharply from rung to rung, independent of demand
tuning -- this is a real `Sim.Core` defect exposed at scale, not a benchmark-harness artifact and
not something a different net size or insertion period can work around.
