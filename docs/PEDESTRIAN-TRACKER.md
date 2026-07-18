# PEDESTRIAN-TRACKER.md — production checklist

At-a-glance status over `PEDESTRIAN-TASKS.md`. A box is ticked only when the task's stated success
conditions pass, verified first-hand (read the diff, re-run the gate). Keep in sync as work lands.

Legend: `[ ]` not started · `[~]` in progress · `[x]` done · `[!]` blocked/needs-decision

---

## Liveliness (believability) — design done, POCs in progress

Design of record: `PEDESTRIAN-LIVELINESS-DESIGN.md` (deterministic activity-timeline replay generalizing
the PathArc trick; stays low-power; POI net-data). POC plan in its §12; graduates to production tasks
once the POCs converge.

- [x] `PEDESTRIAN-LIVELINESS-DESIGN.md` — design + POC plan (Walk/Pause/Dwell/Interact timeline, animation
      tags, POI schema, social planner, buildings-as-sinks, micro-scenarios, subarea §11)
- [x] **LIVE-POC-1** — Walk+Pause+Dwell activity timeline, server==IG deterministic *(`ActivityTimeline`
      generalizes `PathArcMotion`; pure `PoseAt`→(pos,heading,animTag,visible); `ActivityTimelineRecord`
      broadcast-once + `HeadlessIg.ReconstructSample` share `PoseAt`; 4 tests inc. a 300+‑sample exact
      server==IG sweep; `--ped-liveliness` scene renders walk/sip/sit + hidden-window disc drop 5→4
      verified; 589 parity + 91 ped green)*
- [x] **LIVE-POC-2** — pre-scheduled two-ped Interact (meet, step aside, "talk", resume) — server==IG
      *(`SocialPlanner` writes matching `InteractSegment`s into both timelines: closed-form nearest-
      approach meet point, bisector-perpendicular ±offset, identical [T,T+D] window, headings facing;
      earlier ped's approach re-timed so neither waits; `--ped-social` scene; 6 tests inc. exact
      server==IG sweep (744 samples/ped), identical-window coordination, exact √2/2 stepped-aside
      identity; 589 parity + 97 ped green)*
- [x] **LIVE-POC-3** — waiter micro-scenario (templated scripted actor serving open-air tables)
      *(`WaiterScenario.Build` = a pure looping `ActivityTimeline` anchored to (door, table-cluster):
      emerge → Walk→Dwell(serve)→Walk→hidden Dwell(inside), tables in a closed-form seed-rotation that
      serves each exactly once per cycle; `--ped-waiter` scene (building + seated patrons + waiter,
      hidden between rounds); 4 tests inc. exact server==IG sweep, serves-every-table, inside=hidden;
      589 parity + 101 ped green)*
- [ ] **LIVE-POC-4** — auto-deduced liveliness demand in a subarea + density knob + shared-mask legitimacy
      (after Stage P8 + a real cropped box; zero on-camera pops audited)

Graduation into production (making the *routed ambient crowd* lively):
- [x] **LIVE-PROD-1a** — `PedLodManager` low-power `ActivityTimeline` (lively) peds *(`AddPedLively`;
      `ActivityTimeline.VelocityAt`; promote/demote treats low-power as PathArc OR ActivityTimeline;
      null-timeline path bit-identical — existing 101 ped tests unchanged; 2 new tests: exact server==IG
      sweep <1e-12 + promotion)*
- [x] **LIVE-PROD-1b** — `PedDemand` schedule generator (seeded Pause beats along routes) +
      `--ped-lively-crowd` demo *(dedicated `LivelinessSalt`; `Liveliness=null` → exact original
      `AddPed` call, bit-identical; `BuildLivelyTimeline` splices seeded pauses, last Walk ends at the
      true destination so arrivals fire; `AnimTagOf` drives the paused disc kind; 4 tests: bit-identical
      off, determinism-on, no spawn/O-D shift, paused-ped-still-arrives; 589 parity + 107 ped green)*

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

- [x] **P3-1** Pedestrian wire codec + transport-neutral replication surface (hermetic) *(`ActivityTimelineWire`
      Encode/Decode; `IPedReplicationSink`/`Source` + `InMemoryPedReplicationBus` — a TRUE byte loopback, not
      a struct hand-off; `PedReplicationPublisher`/`Receiver` bridge reusing `HeadlessIg`; `Sim.Pedestrians`→
      `Sim.Replication` one-way, `Sim.Replication` stays ped-free; round-trip test: server==IG over serialized
      bytes — EXACT for PathArc + ActivityTimeline (801 samples), ≤0.02 m for quantized FreeKinematic (meas.
      2.7e-7 m), receiver `ModelOf` flips on promote; 590 parity + 111 ped green, no DDS)*
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

- [x] **P7-0** HTML Sim.Viz pedestrian demo gallery (mobile-watchable; kept as demo/test scenarios for
      GitHub Pages) — five self-contained `--ped-*` scenes (crossing-gate, lod-promotion, od-routing,
      dodge-reroute, parking), wired into `gen-demos.sh` under a "Pedestrians" category; each renders
      from the real components (Engine.CrowdSource, PedLodManager+InterestField, PedDemand+SumoNavMesh,
      RerouteDriver, LotCoupling), screenshot-verified, `everPromoted=True` on the LOD scene *(user
      requested HTML demos before 3D)*
- [ ] **P7-1** Native viewer (`Sim.Viewer`/Raylib) in-process ped render (generalize evac overlay;
      regime-aware; board/alight appear/disappear) — *schedule early for a visible demo*
- [ ] **P7-2** Native viewer remote ped render over DR/DDS (FreeKinematic extrapolate + PathArc follow;
      server==IG visual parity; no promotion pop)
- [ ] **P7-3** City3D (Godot 3D) ped render — in-process (`SimSource`) + remote (`Reconstructor`/DDS);
      ground placement, heading, GPU LOD-cull

## Stage P8 — Subarea integration (SumoData compatibility)

Realizes `SUBAREA-FOR-PEDESTRIAN-SESSION.md` §3; mapping in `COORDINATION-pedestrian-x-subarea.md`, design
consequences in `PEDESTRIAN-LIVELINESS-DESIGN.md` §11. All additive + inert by default (empty visible set →
permissive → existing goldens unchanged). Waits on P2-3 + P1-1 + a real cropped box `net.xml`.

- [ ] **P8-1** Verify the SUMO-geometry bake against a real cropped sub-area net (fringe = boundary-cut
      walkable stubs; pin the fringe set) — *cheap; run when a crop is available*
- [ ] **P8-2** Appearance-legitimacy layer (`PedSpawnPolicy`) — the no-cheating gate, **orthogonal to
      sim-LOD**; spawn/despawn only at fringe/sink/off-camera, reading the same camera visible-edge set as
      the vehicle `RealismMask`; inert-default bit-identical — *the load-bearing new piece*
- [ ] **P8-3** Auto-deduced pedestrian demand (O→D + POIs from walkable-space + land-use; their
      `deduce_weights.py` as template; spawns land at fringe/doors)
- [ ] **P8-4** Pedestrian density knob + crossing-throughput guard (crowds never deadlock the calibrated cars)
- [ ] **P8-5** Scenario/manifest slot-in + shared FCD replay (cars + peds in one Sim.Viz stream)

---

## Standing invariants (must stay true every task)

- [x] SUMO parity core untouched — hash `909605E965BFFE59`, `Sim.ParityTests` green *(holds through POC phase)*
- [x] Perf/parallel changes bit-identical to serial (or gated fast-mode) *(POC-7a gate style)*
- [ ] Coordinate all `Engine.cs`/routing touches (P4) with the parallel lane-engine session
- [ ] Subarea: appearance-legitimacy (P8-2) inert by default — empty visible set leaves every ped golden
      unchanged; and ping the SumoData session to re-calibrate when P4 (vehicle-yields-at-crossing) lands
