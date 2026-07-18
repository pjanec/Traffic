# PEDESTRIAN-TRACKER.md — production checklist

At-a-glance status over `PEDESTRIAN-TASKS.md`. A box is ticked only when the task's stated success
conditions pass, verified first-hand (read the diff, re-run the gate). Keep in sync as work lands.

Legend: `[ ]` not started · `[~]` in progress · `[x]` done · `[!]` blocked/needs-decision

---

## Design & POC phase — COMPLETE

- [x] `PEDESTRIAN-OVERVIEW.md` (WHAT), `PEDESTRIAN-DESIGN.md` (HOW), `PEDESTRIAN-POC-PLAN.md` (experiments)
- [x] POC-0 — pedestrian test network + separate ped ingest
- [x] POC-1 — navigation seam (SUMO-bake + DotRecast providers)
- [x] POC-2 — crosswalk gate + car-stops-for-ped
- [x] POC-3 — sim-LOD promotion + PathArc DR (server==IG, no-flap, silent-low-power)
- [x] POC-4 — dense believability (decision: pure ORCA, no density term)
- [x] POC-5 — obstacle dodging + reroute
- [x] POC-6 — parking lot (mode-switch + board/alight + mutual avoidance)
- [x] POC-7a — parallel `OrcaCrowd.Step` (bit-identical) · POC-7b — quantized transport + bandwidth ·
      POC-7c — integrated LOD scale
- [x] Findings recorded: `PEDESTRIAN-POC4-FINDINGS.md`, `-POC7A-`, `-POC7B-`, `-POC7C-FINDINGS.md`

---

## Stage P0 — Crowd-store Add/Remove (PRIORITY)

- [x] **P0-1** `OrcaCrowd` stable-handle `Add`/`Remove` (free-list + generation) — bit-identical no-remove
      path; determinism + parallel gates *(OrcaHandle; 3 new tests; 578 parity green)*
- [x] **P0-2** `MixedTrafficCrowd` `Add`/`Remove` + `SetExternalObstacles` dynamic disc input
      *(velocity-aware avoidance proven; 582 parity green)*
- [x] **P0-3** `PedLodManager` / `LotCoupling` use Add/Remove (retire rebuild) *(same-VM A/B: churn ~9%
      faster, stable neutral — net win, no regression; high-water compaction noted as a P6 refinement)*

## Stage P1 — Consolidate API + interest-source system

- [x] **P1-1** Movable multi-source interest field (spatial, hysteretic, deterministic)
      *(InterestField; inverse spatial index matches a brute-force oracle; 8 tests; 582 parity green)*
- [ ] **P1-2** `Sim.Pedestrians` production API surface + NuGet packaging + sample

## Stage P2 — Navigation productionization

- [x] **P2-1** Harden SUMO-geometry bake (mitred strips, bent-sidewalk adjacency) + static-obstacle index
      *(OrcaCrowd.UseObstacleSpatialIndex bit-identical serial+parallel, sub-linear scaling; mitred
      sidewalks + 2-polygon-cluster adjacency + spine threading; 585 parity green)*
- [ ] **P2-2** Dynamic blockers + reroute in `IPedNavigation` (no thrash)
- [ ] **P2-3** Production navmesh integration contract + example + ped OD demand

## Stage P3 — Networking (DDS multicast, end-to-end)

- [ ] **P3-1** Crowd + PathArc DDS topics + regime lifecycle events (multicast loopback round-trip)
- [ ] **P3-2** Publisher + global bandwidth governor (single-stream < 500 Mbit/s under spike)
- [ ] **P3-3** IG-side reconstruction (FreeKinematic extrapolator + PathArc follower; server==IG over DDS)

## Stage P4 — Engine coordination seams (Core; with lane-engine session)

- [ ] **P4-1** Engine TLS-crossing signal projection (gate reads live signal; parity hash unchanged) — [!] coordinate

## Stage P5 — Evac generalization

- [ ] **P5-1** `Sim.Evac` consumes `Sim.Pedestrians` (panic = forced promotion + override; route to safe zone)

## Stage P6 — Scale hardening + on-target validation

- [ ] **P6-1** On-target (16+‑core) benchmark run; replace 4-core estimates in findings
- [ ] **P6-2** Region decomposition (only if P6-1 shows flat parallel plateaus)
- [ ] **P6-3** Full property-test suite (reqs 1–7, each named; parity untouched)

## Stage P7 — Pedestrian visualization (existing 3D viewers, in-process + remote-DR)

- [ ] **P7-1** Native viewer (`Sim.Viewer`/Raylib) in-process ped render (generalize evac overlay;
      regime-aware; board/alight appear/disappear) — *schedule early for a visible demo*
- [ ] **P7-2** Native viewer remote ped render over DR/DDS (FreeKinematic extrapolate + PathArc follow;
      server==IG visual parity; no promotion pop)
- [ ] **P7-3** City3D (Godot 3D) ped render — in-process (`SimSource`) + remote (`Reconstructor`/DDS);
      ground placement, heading, GPU LOD-cull

---

## Standing invariants (must stay true every task)

- [x] SUMO parity core untouched — hash `909605E965BFFE59`, `Sim.ParityTests` green *(holds through POC phase)*
- [x] Perf/parallel changes bit-identical to serial (or gated fast-mode) *(POC-7a gate style)*
- [ ] Coordinate all `Engine.cs`/routing touches (P4) with the parallel lane-engine session
