# Tracker — arbitrary SUMO road-net import with route-graph pedestrians

At-a-glance checklist for `LIVE-CITY-ARBITRARY-NET-TASKS.md`. Tick a box only when the task's stated success
conditions are verified first-hand (Opus gate for the reviewed tasks). Global gate on every tick: parity
654/4 byte-identical + bench hash unchanged.

## Stage A — dataset param, drivable edges, capability probe
- [ ] **A1** `LiveCityConfig.ForDataset` + `PedNavMode`; `ForRepoRoot` delegates (demo identical)
- [ ] **A2** drivable edges from `net.xml` fallback (demo edge set unchanged)
- [ ] **A3** capability probe + graceful degrade + `PedestriansEnabled`/`CrossingsEnabled`

## Stage B — SumoRouteGraphNav
- [ ] **B1** node/edge graph + nearest-lane spatial index
- [ ] **B2** `FindPath` (A* + polyline assembly through crossings/walkingareas)
- [ ] **B3** `HalfWidthsAlong` from real lane widths
- [ ] **B4** determinism (no `System.Random`, repeat-identical)

## Stage C — road-net mode wiring
- [ ] **C1** mode branch: build route-graph nav, skip sidewalk bake (demo path unchanged)
- [ ] **C2** crossings-only bake + gate/signals + walk-only degrade
- [ ] **C3** O/D sampling from sidewalk centrelines (deterministic)
- [ ] **C4** `RerouteDriver`/concrete-`SumoNavMesh` not wired in road-net mode

## Stage D — config surfacing
- [ ] **D1** ped-demand knobs promoted to config (demo `PedDemandConfig` byte-identical)

## Stage E — offline prep, fixture, tests
- [ ] **E1** `scripts/prep-ped-net.sh` + recipe
- [ ] **E2** committed synthetic road-net fixture (`scenarios/_ped/roadnet_min/`, no proprietary data)
- [ ] **E3** unit + smoke/regression tests (green without SUMO)
- [ ] **E4** coordinate robustness (large/negative/3-D)

## Stage F — final gate (Opus)
- [ ] **F1** parity 654/4 + bench hash + demo liveness/scene green; no `Sim.Core` diff; netstandard2.1 +
  consumer contract intact
