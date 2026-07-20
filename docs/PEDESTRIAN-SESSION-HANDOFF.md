# PEDESTRIAN-SESSION-HANDOFF.md — state of the pedestrian subsystem (handoff)

Snapshot for a fresh session picking up the pedestrian work. Read `CLAUDE.md` first (the rules of the road),
then `docs/PEDESTRIAN-DESIGN.md` (architecture) and `docs/PEDESTRIAN-TRACKER.md` (live checklist). This file
is the orientation map: what exists, what's parked, where to look, and how to run it.

## 1. Branch / repo state

- Everything below is **merged to `main`** (PRs #5 DDS transport, #6 sub-area P8-3/P8-4a/P8-5-ped-side, #7
  P8-1b navmesh connectivity). The dev branch is `claude/pedestrian-dds-transport-c8w2gf`.
- A merged PR is final — start follow-up work from the latest `main` (`git fetch origin main && git checkout -B
  <branch> origin/main`), don't restack on merged history.
- **CI note:** `.github/workflows/ci.yml` pins the engine determinism hash. It was refreshed once
  (`909605E965BFFE59` → `AA6143E74072CD4C`) after vehicle-side work moved it; if a *vehicle/engine* change
  legitimately moves it again, update that constant (with justification) — it is the SUMO-parity anchor, treat
  it as load-bearing.

## 2. Architecture in one screen

- **Two LOD regimes** (`docs/PEDESTRIAN-DESIGN.md` §7): **low-power** = `PathArcMotion` / `ActivityTimeline`,
  pose a *pure function* of `(path, startTime, speed, now)`, O(1)/ped, no neighbour state — the cheap default
  for the bulk crowd; **high-power** = `OrcaCrowd` (ORCA/RVO reciprocal avoidance), for the promoted on-camera
  subset. Promotion is driven by `InterestField` sources (the camera). **Only high-power avoids** (see the
  open realism task in §4).
- **`PedLodManager`** owns the population: `AddPed` (low-power PathArc), `AddPedLively` (low-power timeline),
  promote/demote, `PositionOf`, `RemovePed`. `SetForcedHighPower(id, on)` forces ORCA for a ped.
- **`PedDemand`** (`src/Sim.Pedestrians/Demand/PedDemand.cs`) — self-populating O→D demand above
  `PedLodManager`: seeded spawn timing + O/D draw (`VehicleRng`, dedicated salts), routes via `IPedNavigation`,
  despawns on arrival, holds a `PopulationCap`. Optional `Liveliness` (Pause beats) and `WeightedEndpoints`
  (sub-area demand) are additive + inert-default (null → bit-identical to before).
- **Navmesh** (`src/Sim.Pedestrians/Navigation/Bake/`): `PedNetworkParser.Load` → `WalkablePolygonBaker.Bake`
  → `BakedPolygon[]` shared by `SumoWalkableSpace` (containment) and `SumoNavMesh` (A* routing over
  `PolygonGraph` portal adjacency). See §3 P8-1b for the connectivity model.
- **Replication / determinism:** per-entity seeded `VehicleRng` (SplitMix64, `SeedFor(seed,index,salt)`); no
  `System.Random`; low-power server==IG reconstruction (IG replays the same `PathArcMotion`/`ActivityTimeline`
  from a one-time path broadcast). DDS transport for the ped replication surface exists (`Sim.Replication.Dds`,
  D1–D3; out of `Traffic.sln`).

## 3. What's landed (with pointers)

- **Liveliness** (believable low-power motion: Walk/Pause/Dwell/Interact timelines, social meet, waiter) —
  `docs/PEDESTRIAN-LIVELINESS-DESIGN.md`, `ActivityTimeline`, `--ped-liveliness/--ped-social/--ped-waiter`
  demos. Done through LIVE-PROD-1b.
- **DDS transport** (ped replication over CycloneDDS) — D1–D3, `src/Sim.Replication.Dds/Ped*`, merged (PR #5).
- **P8-3 auto-deduced sub-area demand** — `SubareaDemand` (weighted fringe+POI endpoint set, deterministic
  weighted draw, fringe→sidewalk-midpoint) wired into `PedDemand` behind inert-default `WeightedEndpoints`.
  Every spawn/arrival is a fringe/POI edge → **appearance-legitimate by construction**.
  `docs/PEDESTRIAN-P8-3-DEMAND-DESIGN.md`. Consumes `PedPoi`/`PedPoiReader`, `SubareaManifest`.
- **P8-4a density knob** — `PedDensityKnob`: dialable *pedestrians-per-walkable-km* (mirrors the vehicle
  `knee_veh_lkm` model), Little's-law rate, dial clamped to a LoS-C safe ceiling (static crossing-throughput
  guard). `docs/PEDESTRIAN-P8-4-DENSITY-DESIGN.md`.
- **P8-5 ped side (shared replay)** — `SubareaFcdRecorder` + `PersonFcdWriter` drive a box with P8-3 demand
  sized by the P8-4a knob and emit a SUMO `<person>` FCD in **box XY metres on the vehicle FCD grid** (t=0,
  step matches `step-length`), consumed by the sub-area session's `sim_viz --ped-fcd`
  (`docs/SUBAREA-SHARED-REPLAY-CONTRACT.md`). CLI: `Sim.Viz --ped-subarea-fcd`.
- **P8-1b real-net navmesh connectivity** — additive, area-anchored **overlap/abutment** adjacency pass in
  `PolygonGraph` (`PolygonGeometry.TryFindOverlapPortal`, `AbutProximityEps=5 cm`): bridges genuine overlaps
  and ≤5 cm gaps **only through an area polygon**, preserving the POC-0 no-shortcut invariant and leaving every
  existing bake bit-identical. `SumoNavMesh.ConnectedComponentCount()` diagnostic + recorder connectivity
  report. `docs/PEDESTRIAN-P8-1B-NAVMESH-CONNECTIVITY-DESIGN.md`, witness at
  `scenarios/_ped/subarea-irregular/` (222 → 1 component / peak 0 → 113).

## 4. Open / parked work (what a new session might pick up)

- **PED-REALISM-1 — low-power peds pass through each other (HIGH priority, OPEN).** Thousands of low-power peds
  ride the sidewalk centerline and interpenetrate (opposite directions pass straight through). The fix must
  live in the **low-power** path and preserve the server==IG pure-function identity — see
  `docs/PEDESTRIAN-LOWPOWER-AVOIDANCE-DESIGN.md` (recommended start: deterministic direction-based lateral
  offset / lane slotting; no neighbour-reactive nudging). **This is the most user-visible realism gap.**
- **P8-2 live-camera deny-defer gate** — mechanism (`PedSpawnPolicy`) landed; wiring the live per-tick
  visible-walkable set is parked (no-op on the by-construction demand path until a host publishes it).
- **P8-4b dynamic per-crossing throughput guard** — needs the vehicle-calibration seam + P4
  vehicle-yields-at-crossing (SumoData-owned).
- **P8-5 merge/slot-in** — the car+ped FCD merge + manifest slot-in are the **sub-area session's**; our side
  (the person-FCD recorder) is done.
- **P6-2 phase-2 SoA-reorder** (OrcaCrowd region decomposition) — region-task parallelism is bit-identical but
  fell short of the throughput target; deferred (`docs/PEDESTRIAN-P6-2-REGION-DESIGN.md`).
- Full deferred index: `docs/PEDESTRIAN-P8-BACKLOG.md`.

## 5. Build / test / run

- Build: `dotnet build` · Offline gate: `dotnet test` (must pass on a fresh VM with **no SUMO**; ~650 parity /
  ~174 ped / 2 DotRecast tests). SUMO is only for regenerating goldens / authoring fixtures, never in the test
  loop.
- Sub-area person-FCD recorder:
  `dotnet run --project src/Sim.Viz -c Release -- --ped-subarea-fcd <out.fcd.xml> [--dial d] [--seconds s] [--box <dir>]`
  (prints navmesh components / unreachableSkips; default box `scenarios/_ped/subarea-box`, irregular witness
  `scenarios/_ped/subarea-irregular`).
- Ped viz demos: `Sim.Viz --ped-*` (liveliness, od-routing, lively-crowd, remote, …).

## 6. Invariants a new session must keep

- **Parity is the iron law**: no change may move a committed scenario out of `tolerance.json`; the engine
  determinism hash gate (CI) is the SUMO-parity anchor.
- **Determinism**: per-entity seeded `VehicleRng` only, never `System.Random`; results independent of thread
  order; low-power **server==IG** reconstruction preserved (pose = pure function of the ped's own broadcast).
- **Additive / inert-default**: new features off by default leave every committed golden bit-identical.
- **Design-first**: for a new feature, write the design (HOW) + tasks + tracker entries before code.
- **Two loops kept separate**: `dotnet test` never invokes SUMO; goldens are regenerated + committed offline.
- **Coordination**: sub-area (SumoData) session owns the crop pipeline + car+ped replay merge; a perf/Windows
  session runs throughput. Pull/read shared-branch reports before large changes.

## 7. Coordination docs

`docs/COORDINATION-pedestrian-x-subarea.md` (7-requirement status), `docs/SUBAREA-SHARED-REPLAY-CONTRACT.md`
(person-FCD format), `docs/SUMOSHARP-P8-1-REAL-NET-NAVMESH.md` (the fragmentation finding, resolved).
