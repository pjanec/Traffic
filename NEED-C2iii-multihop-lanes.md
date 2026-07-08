# NEED — C2-iii: multi-hop lane-to-lane continuity (route-wide best-lanes)

**For the SUMO parity coding session.** This is a **parity-track** `Sim.Core` change (EXACT
parity bar, needs an anchor scenario + SUMO golden + parity-reviewer gate — the standard method
in `HANDOFF.md`). It is the deferred second half of **C2** and blocks the scaled-city benchmark's
multi-lane rungs. Written from the `claude/spec-docs-review-qgwatc` branch after rebasing onto
`main@e30269a`; verified the gap still exists there.

## One-line ask

Extend the engine's route→lane resolution from **single-look-ahead** to **full multi-hop**, so a
vehicle on a multi-lane, multi-junction route is placed in (and lane-changes into) a lane sequence
that actually threads a `<connection>` at **every** hop to the route's end — instead of throwing
when the greedy one-hop choice dead-ends.

## The exact failure (reproduced on `main@e30269a`)

`Sim.Ingest.NetworkModel.ResolveLaneSequence(routeEdges, departLaneIndex)`
(`src/Sim.Ingest/NetworkModel.cs:255-308`) does two things:

1. Picks the **departure** lane once, via `ComputeBestLanes` — but that call only considers the
   transition to the **immediately next** route edge (see below).
2. Then walks the route edge-by-edge and **hard-requires** an exact
   `ConnectionsByFromLaneTo[(fromEdge, currentLaneIndex, toEdge)]` at every hop
   (`NetworkModel.cs:288-294`):
   ```
   if (!ConnectionsByFromLaneTo.TryGetValue(key, out var connection))
       throw new InvalidDataException(
           $"No <connection> found from edge '{fromEdgeId}' lane {currentLaneIndex} to edge '{toEdgeId}'.");
   ```
   `currentLaneIndex` only advances by whatever `connection.ToLane` each hop lands on — there is
   **no lane-change planning across hops**. If hop *k*'s `ToLane` is a lane that has no
   `<connection>` onward to edge *k+2*, insertion throws outright.

`ComputeBestLanes` (`NetworkModel.cs:328-368`) documents the scope itself: it is a
"single-look-ahead scoped port of `MSVehicle::updateBestLanes` / `LaneQ`", built for the
single-junction anchor `scenarios/18-strategic-turnlane`. Its own comment (lines 346-353) says the
**backward pass is DEFERRED**:

> DEFERRED (not built here — no scenario needs it yet): SUMO's backward pass
> (MSVehicle.cpp:6003-6063) that, for a CONTINUING lane, recursively ADDS the best downstream
> lane's own `length` onto this lane's `length`, accumulating a route-wide continuation distance
> across every remaining edge (and picks the best of several downstream lanes when more than one
> continues). … NOT the full multi-edge recursion; a multi-junction scenario would need that
> deferred piece built out before `Length` could be trusted route-wide.

### Concrete repro
```
export SUMO_HOME=/usr/local/lib/python3.11/dist-packages/sumo
netgenerate --grid --grid.number=3 --grid.length=200 -L 2 --tls.guess --seed 42 -o net.net.xml
python3 $SUMO_HOME/tools/randomTrips.py -n net.net.xml -e 120 -p 2 --fringe-factor 5 --seed 42 \
        -o trips.xml -r trips.rou.xml
duarouter -n net.net.xml -r trips.xml -o rou.rou.xml --seed 42 --ignore-errors --named-routes
# (add a DEFAULT_VEHTYPE vType + type= on each <vehicle>; see the note at the bottom)
dotnet run --project src/Sim.Run -- <thatDir> --steps 120 --fcd-out /tmp/x.fcd.xml
```
→ `System.IO.InvalidDataException: No <connection> found from edge 'A1B1' lane 1 to edge 'B1B0'`
at `ResolveLaneSequence` (`NetworkModel.cs:292`), via `Engine.TryInsertOnLane`
(`Engine.cs:~588`) during `InsertDepartingVehicles`. A single-lane net (`-L 1`) never hits this
(one unambiguous connection per direction), which is exactly why the benchmark bring-up is pinned
to `-L 1` (`scenarios/_bench/city-30/NOTES.md`).

## What SUMO does (port target)

- `MSVehicle::updateBestLanes` + `struct LaneQ` — `sumo/src/microsim/MSVehicle.cpp:5744-6063`
  (`LaneQ` declared `sumo/src/microsim/MSVehicle.h:865-886`).
- The already-ported forward/per-edge `LaneQ` build is `MSVehicle.cpp:5896-5918`; the
  last-route-edge special case is `5951-5989`; the non-continuing-lane offset tie-break is
  `5970-5976` (all cited in `ComputeBestLanes`'s comment).
- **The missing piece is the backward pass `MSVehicle.cpp:6003-6063`:** for each CONTINUING lane,
  recurse to the best downstream lane, accumulate route-wide continuation `length`, and set
  `bestLaneOffset` so the vehicle is steered — across multiple junctions — toward a lane that
  stays connected to the end of its route. This is what makes `bestLaneOffset` a route-wide
  quantity rather than a one-junction hint.

## Definition of done

1. **Multi-hop resolution.** `ComputeBestLanes`/`ResolveLaneSequence` produce a lane sequence that
   threads a valid `<connection>` at every hop of a multi-lane, multi-junction route, choosing the
   best downstream lane per SUMO's backward recursion. No `InvalidDataException` for any route that
   SUMO itself routes and runs.
2. **Lane-change to stay connected.** Where the departure/upstream lane cannot reach the route but
   a lateral neighbor can, the vehicle changes into the connecting lane (strategic `LCA_STRATEGIC`
   /`bestLaneOffset` drive), reusing the existing C2-ii strategic-LC path
   (`Engine.TryStrategicLaneChange`) rather than hard-throwing.
3. **Anchor scenario + golden (the parity bar).** Add a minimal `scenarios/NN-multihop-lanes/`:
   a 2-lane, ≥2-junction net where the correct lane at insertion is dictated by a connection
   **two hops ahead** (i.e. the single-look-ahead choice is wrong), one `sigma=0` vehicle, forced
   turn. Generate the SUMO golden (`--precision 6`) via `scripts/regen-goldens.sh`. Match `lane`,
   `pos`, `speed` within `1e-3` (EXACT `parityMode`).
4. **Inert-when-absent / no regressions.** `scenarios/18-strategic-turnlane` and every other
   committed scenario stay byte-for-byte within tolerance (`dotnet test` green — currently **134**;
   the `Sim.Bench` highway-dense determinism hash must not move). The single-junction fast path
   must be behavior-identical to today.
5. **Gate.** parity-reviewer ACCEPT before commit; faithfulness to `MSVehicle.cpp` (not a
   curve-fit), no regression, plan/execute + committed-vs-ephemeral invariants intact.

## Why it matters / who's blocked

The scaled-city benchmark (`BENCHMARK_SPEC.md`, tracker `VIZ_BENCH_TASKS.md` Phase 2) is proven
end-to-end at the ~30-concurrent bring-up rung but **only at `-L 1`**, so it exercises *no*
lane-changing/overtaking. The 300 / 3k / 15k rungs the spec targets want multi-lane urban density;
they need this item first. This is also a genuine realism gap independent of the benchmark: any
real multi-lane route with a forced turn a couple of junctions out is currently unroutable by the
engine even though SUMO handles it.

## Secondary, optional (ingest robustness — not the core ask)

Two things the benchmark works around at the *generation* layer today; fixing them in
`Sim.Ingest.DemandParser` would let stock `duarouter`/`randomTrips` output load directly (nice for
the benchmark, not required for parity):
- **Embedded routes:** `duarouter`'s default output nests `<route edges="…"/>` inside `<vehicle>`;
  `DemandParser` only accepts the two-part `<route id=…/>` + `<vehicle route="id"/>` form
  (benchmark uses `duarouter --named-routes` to force the supported form).
- **`DEFAULT_VEHTYPE`:** a `<vehicle>` with no `type=` makes SUMO fall back to its built-in
  `DEFAULT_VEHTYPE`; `DemandParser`/`Engine.LoadScenario` instead throw `KeyNotFoundException` on
  `VTypesById[""]` (benchmark post-processes an explicit `DEFAULT_VEHTYPE` vType in). Synthesizing
  SUMO's default when `type=` is absent would match SUMO and remove the workaround.
