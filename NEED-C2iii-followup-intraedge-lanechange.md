# NEED (follow-up to C2-iii) — intra-edge lane change to reach a mid-route connection

**For the SUMO parity coding session.** C2-iii (`40e53f4`, route-wide best-lanes backward pass)
is real and its anchor `scenarios/36-multihop-lanes` passes — but its claim to **"UNBLOCK the
scaled-city benchmark's multi-lane rungs (`-L 2+`)"** is **not yet true for a general netgenerate
city**. Verified on `main@40e53f4`: a plain `netgenerate -L 2` grid still throws at insertion. This
is the precise remaining case. Same parity-track bar (exact `@1e-3`, anchor + golden + gate).

## What C2-iii fixed vs. what's still open

C2-iii's `ResolveLaneSequence` redirects the pool onto the best-continuing lane **only at the
departure edge** (the `if (routeEdges.Count > 1)` block redirects `currentLaneIndex` once, for
`routeEdges[0]`), and threads multi-connection hops through the best downstream lane. At every
**intermediate** edge it still follows `connection.ToLane` rigidly and **hard-throws** if the lane
it was dumped on has no onward connection (`NetworkModel.cs:~296-303`):

```
var candidates = ConnectionsByFromEdgeLane[(fromEdgeId, currentLaneIndex)].Where(c => c.To == toEdgeId)…
if (candidates.Count == 0)
    throw new InvalidDataException($"No <connection> found from edge '{fromEdgeId}' lane {currentLaneIndex} to edge '{toEdgeId}'.");
```

The missing case: a vehicle **enters an intermediate edge on lane A** (forced by the incoming
connection's `toLane`) but the **only onward connection to the next route edge leaves from lane B ≠
A**, so it must **lane-change from A to B while traversing that edge**, before the junction. SUMO
does this routinely (arrival lane ≠ departure lane on an edge; `bestLaneOffset` steers it). The
engine can't: `ResolveLaneSequence` never applies best-lane redirection at a non-departure edge, so
it throws instead of recording lane B as the edge's exit lane and letting strategic LC move the
vehicle over.

## Exact failing topology (verified, SUMO runs it, engine throws)

`netgenerate --grid --grid.number=3 --grid.length=200 -L 2 --tls.guess --seed 42`, route
`r4 = B1A1 A1B1 B1B0 B0A0 A0A1`:

- Enter A1B1 from B1A1: `<connection from="B1A1" to="A1B1" fromLane="1" toLane="1"/>` → vehicle is
  on **A1B1 lane 1**.
- Leave A1B1 to B1B0: the ONLY onward connection is
  `<connection from="A1B1" to="B1B0" fromLane="0" toLane="0" dir="r"/>` → requires **A1B1 lane 0**.
- A1B1 lane 1's connections go to B1C1 / B1B2 / B1A1 — **never B1B0**.
- ⇒ the vehicle must change **lane 1 → lane 0 on edge A1B1** before junction B1. SUMO inserts and
  completes all 40 such vehicles with 0 errors/0 teleports; the engine throws
  `InvalidDataException: No <connection> found from edge 'A1B1' lane 1 to edge 'B1B0'` at
  `ResolveLaneSequence` (`NetworkModel.cs:302`) via `Engine.TryInsertOnLane` at **insertion**.

### Clean repro (no `--ignore-errors`, so every route is genuinely valid)
```
export SUMO_HOME=/usr/local/lib/python3.11/dist-packages/sumo
netgenerate --grid --grid.number=3 --grid.length=200 -L 2 --tls.guess --seed 42 -o net.net.xml
python3 $SUMO_HOME/tools/randomTrips.py -n net.net.xml -e 120 -p 3 --fringe-factor 5 --seed 42 -o trips.xml
duarouter -n net.net.xml -r trips.xml -o rou.rou.xml --seed 42 --named-routes      # NO --ignore-errors
# add a DEFAULT_VEHTYPE vType + type= on each <vehicle>
sumo -n net.net.xml -r rou.rou.xml --no-step-log true            # SUMO: Inserted 40, Running 0, no errors
dotnet run --project src/Sim.Run -- <thatDir> --steps 120 --fcd-out /tmp/x.fcd.xml   # engine: throws
```

## What SUMO does (port target)

Same source as C2-iii — `MSVehicle::updateBestLanes` / `LaneQ`
(`sumo/src/microsim/MSVehicle.cpp:5744-6063`). The relevant fact: `bestLaneOffset` is a **per-edge**
quantity along the whole route, not just at departure. On EACH edge the vehicle occupies, if its
current lane's `bestLaneOffset ≠ 0` it strategically changes toward the connecting lane
(`LCA_STRATEGIC`, the C2-ii `TryStrategicLaneChange` path already in the engine). The arrival lane
and the exit lane on one edge legitimately differ.

## Definition of done

1. **Intra-edge redirection at every hop.** `ResolveLaneSequence` applies the same best-lane
   redirection C2-iii does at the departure edge to **every** intermediate edge: when the arrival
   lane has no onward connection to the next route edge, move to the same-edge sibling lane that
   does (per `bestLaneOffset` / the best-continuing lane), record that as the edge's exit lane, and
   rely on strategic LC to carry the vehicle laterally across the edge. No throw for any route SUMO
   itself routes and runs.
2. **General `-L 2` city runs.** The clean repro above (and the benchmark's `-L 2` generation) runs
   to completion in the engine, not just the hand-built anchor.
3. **New anchor + golden.** A minimal 2-lane, ≥3-edge net where a vehicle is **forced onto the
   "wrong" lane of a MIDDLE edge** by the incoming connection and must intra-edge-change to reach
   the only lane that turns onto the last edge (i.e. the redirect must happen at a non-departure
   edge — scenario 36 only exercises the departure edge). `sigma=0`, one vehicle, SUMO golden
   `--precision 6`, match `lane`/`pos`/`speed` `@1e-3`.
4. **Inert / no regressions.** Scenario 36 + scenario 18 + all committed scenarios stay green
   (`dotnet test`, currently **137**); `Sim.Bench` highway-dense determinism hash unchanged
   (`42F875C2662DB78E`). Departure-edge and single-connection behavior byte-identical.
5. **Gate.** parity-reviewer ACCEPT; faithful to `MSVehicle.cpp`, no curve-fit, invariants intact.

## Status note to correct

The C2-iii TASKS.md entry says it "UNBLOCKS the scaled-city benchmark's multi-lane rungs (`-L 2+`)".
Please soften that to "unblocks departure-edge / multi-connection multi-hop; intra-edge mid-route
lane change (this follow-up) still required before a general `-L 2` city runs". The benchmark
generator (`scripts/gen-benchmark.sh`) stays pinned to `-L 1` until this lands.
