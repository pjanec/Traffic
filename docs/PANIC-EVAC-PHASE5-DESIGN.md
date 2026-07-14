# PANIC-EVAC-PHASE5-DESIGN.md — scale: large realistic city, local evacuation

Design (HOW) for **Phase 5** (`PANIC-EVAC.md` §6.5). Goal: watch a large-scale evacuation — real
congestion and a massive foot exodus — on a realistic road mesh, with the **final target a 10k-vehicle
city**. Builds on the Phase-1–3 `Sim.Evac` layer. Parity-exempt; determinism preserved.

## 1. The load-bearing principle (owner's insight): the evacuation is LOCAL
A city can hold 10k vehicles, but **the evac layer never needs to process 10k**. The incident is
spatially local (R3: distant traffic stays jammed and unaware), so:

> **The full city runs on the parity engine (already fast — benchmarked to ~15k concurrent). The evac
> layer attaches only to a bounded WORKING REGION around the incident; its cost scales with the local
> affected population (hundreds → low-thousands), not the city size.**

This is both the realism model (R3) and the performance answer. Distant vehicles are pure parity lane
traffic the evac layer never touches; only vehicles that enter the working region get fear / flee /
blocked / push / pedestrian treatment.

## 2. Two tiers (staged; owner approved Tier 1 now, Tier 2 after)
- **Tier 1 — realistic medium town, tractable today.** `LoadScenario` the committed organic net + demand
  (`scenarios/_bench/city-organic-L2`: 274 junctions, 1186 edges, 2 lanes, 618 trips, ~406 peak
  concurrent), fire a local incident, and let the evac layer **auto-attach to vehicles in the working
  region**. At this scale the current O(n²) evac layer is fine for offline generation. Deliverable: a
  watchable realistic-town evac (congestion + foot exodus) **and a measured cost profile** telling us
  which evac hotspot dominates.
- **Tier 2 — 10k city, heavy optimization.** Run a 10k-vehicle city on the parity engine; the evac layer
  stays bounded by the working region, PLUS spatial indexing on the local set where Tier-1 measurement
  shows it is needed, PLUS viz payload management. Detailed after Tier-1 numbers (§6).

## 3. New mechanism: working-region auto-attach (Tier 1, reused by Tier 2)
`EvacDirector` gains an **auto-track** mode driven off the engine's published read surface, so it works
with `LoadScenario` demand (not just explicit `Track` of runtime spawns):
- **Working region** = a disc of radius `WorkingRadius` (a tunable ≥ incident radius + a jam margin)
  centred on the incident. Only vehicles inside it are evac-relevant.
- Each tick, scan the read buffer (`engine.VehicleHandles` / `PosX` / `PosY`, iterated in a **fixed
  order — ascending EntityIndex — for determinism**); any active, not-yet-tracked vehicle inside the
  region is `Track`ed. Vehicles outside stay pure parity traffic (never tracked, zero evac cost).
- Existing tracked-vehicle lifecycle (fear → flee → blocked → push → pedestrian) is unchanged; it now
  just runs over the auto-tracked local set.
- Determinism: fixed iteration order + no RNG → reproducible (a test).
This one mechanism delivers Tier 1 AND is exactly the Tier-2 bound (the tracked set is capped by the
region regardless of city size).

## 4. Tier-1 build
- **Scenario:** `LoadScenario(city-organic-L2 net + rou + cfg)` — realistic net + demand → natural
  congestion. Incident placed at a busy interior junction (pick one with high through-traffic), fired
  mid-run while congestion is present. New `EvacOrganicScenario` demo builder (loads the scenario, wires
  the director in auto-track mode, exposes the incident/region). `EvacGridScenario`/`EvacTlsScenario`
  and their tests are untouched.
- **Director:** add the auto-track-by-region mode + `WorkingRadius` to `EvacConfig`. `Track` stays for
  the existing runtime-spawn demos.
- **Viz:** a scene that renders the town + the local evac (fear-tinted cars, incident zone, pushers,
  pedestrians, hard edge). Camera fits the town; for ~400 vehicles the payload is a few MB (fine).
- **Measurement (deliverable):** instrument generation to report per-phase evac cost (fear update, disc
  feeds, ped-crowd step, pusher step) and total generation time at the ~400-vehicle scale, so Tier 2
  optimizes the *measured* bottleneck, not a guessed one.

## 5. Determinism & parity (both tiers)
- Parity: `Sim.Evac` stays off the golden path; the parity engine is unchanged (evac only DRIVES it via
  public seams). Hash `909605E965BFFE59` unmoved (a test).
- Determinism: auto-track fixed order; the OrcaCrowd spatial hash is already bit-identical to brute-force
  (Q3); any new FearField/MixedTrafficCrowd grid must be built the same way (pure pre-filter, same
  neighbour set + order) so runs stay reproducible (tests).

## 6. Tier-2 optimization plan (principled; exact scope set by Tier-1 measurement)
Only build what Tier-1 shows is needed. Candidates, in likely priority:
- **FearField uniform grid.** Replace the O(n²) contagion scan with a uniform spatial hash over the
  tracked set (cell = `ContagionRadius`), bit-identical (sorted candidate gather), mirroring OrcaCrowd's
  Q3 grid.
- **Spatial disc feeds + `CrowdSource.QueryNear`.** Today the lane engine's per-vehicle crowd query and
  `FeedVehicleDiscsToPeds` are brute-force over all peds/cars. Make the composite footprint source
  spatially indexed so each query is O(local), not O(all peds+pushers).
- **`OrcaCrowd.UseSpatialHash = true`** for the pedestrian crowd (already implemented, just enable at
  scale), and a **`MixedTrafficCrowd` spatial hash** for pushers if pushers grow large.
- **Viz payload management for a 10k city.** The parity engine has 10k cars but the viz should not emit
  all 10k × 240 frames. Options: region-crop (render the working region + a decimated sample of distant
  traffic), frame decimation, and/or per-frame vehicle caps — with an explicit `log`/note of what was
  dropped (no silent truncation).
- **Working-region maintenance at scale** (spatial query to find region entrants in a 10k read buffer
  without a full O(10k) scan each tick, if the full scan proves too costly — likely fine, but measure).

## 7. Non-goals
Sublane (deferred, Phase-4 decision); any parity-core change; real-time interactive performance (the viz
is offline replay); OSM import (organic synthetic nets suffice). Tier-2 exact task list is deferred to a
Tier-2 addendum written against Tier-1's measured profile.

## 8. Risks
- Auto-track scan cost at 10k (full read-buffer scan each tick) — measure in Tier 1; add a spatial query
  only if needed.
- Viz payload blow-up at 10k — addressed by §6 payload management (Tier 2).
- The organic net uses `LoadScenario` (demand) while the director drives `engine.Step()` in `Tick()`;
  confirm the demand-insertion path advances under the director's step loop (it does — `Advance` runs
  `InsertDepartingVehicles`), and that the director's `_stepLength` matches the scenario config.
