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
- [x] **P1-2** `Sim.Pedestrians` production API surface + NuGet packaging + sample *(`PedestrianWorld` thin
      facade over `PedLodManager`+`InterestField`+`PedPublisher` — AddWalker/AddLivelyWalker/Remove/
      `SetForcedHighPower` (evac panic pin, added to `PedLodManager`, unpinned path bit-identical)/interest
      sources/SetExternalObstacles/Step/queries/LiveIds/Publisher; packs as `SumoSharp.Pedestrians`; sample +
      README + 6 facade tests; 590 parity + 127 ped green)*

## Stage P2 — Navigation productionization

- [x] **P2-1** Harden SUMO-geometry bake (mitred strips, bent-sidewalk adjacency) + static-obstacle index
      *(OrcaCrowd.UseObstacleSpatialIndex bit-identical serial+parallel, sub-linear scaling; mitred
      sidewalks + 2-polygon-cluster adjacency + spine threading; 585 parity green)*
- [x] **P2-2** Dynamic blockers + reroute in `IPedNavigation` (no thrash) *(`Obstacles/MovingBlocker` +
      `Navigation/RerouteDriver`: a ped detours around a dynamic blocker via a hysteretic reroute that does
      not thrash; `ObstacleDodgeTests` + the Req3 property test (ped avoids a moving blocker) exercise it;
      landed within the 139-ped-test suite that P3/P5/P6 build on)*
- [x] **P2-3** Production navmesh integration contract + example + ped OD demand *(`Navigation/INavigation`
      contract + `SumoNavMesh` provider; `Demand/PedDemand` Poisson OD spawn (VehicleRng SplitMix64,
      per-salt deterministic) routed over the navmesh; consumed by the `--ped-od-routing`/`--ped-lively-crowd`
      demos and every downstream P3/P5 task; in the 139-ped-test suite)*

## Stage P3 — Networking (DDS multicast, end-to-end)

- [x] **P3-1** Pedestrian wire codec + transport-neutral replication surface (hermetic) *(`ActivityTimelineWire`
      Encode/Decode; `IPedReplicationSink`/`Source` + `InMemoryPedReplicationBus` — a TRUE byte loopback, not
      a struct hand-off; `PedReplicationPublisher`/`Receiver` bridge reusing `HeadlessIg`; `Sim.Pedestrians`→
      `Sim.Replication` one-way, `Sim.Replication` stays ped-free; round-trip test: server==IG over serialized
      bytes — EXACT for PathArc + ActivityTimeline (801 samples), ≤0.02 m for quantized FreeKinematic (meas.
      2.7e-7 m), receiver `ModelOf` flips on promote; 590 parity + 111 ped green, no DDS)*
- [x] **P3-2** Publisher DR-error gating + global bandwidth governor *(`PedPublishScheduler` linear-
      extrapolation DR-error gate — steady ped 7/200 sends @ ~0 err, maneuvering 23/200 within 0.1 m tol;
      `PedBandwidthMeter` real per-topic byte→Mbit/s; `PedBandwidthGovernor` global cap, defers least-urgent
      (first-sightings never) — 100k all-high-power = 144 Mbit/s, 400k-promotion spike held at 499.99 Mbit/s
      (governor engaged, caught up in 2 steps), low-power 0 per-step crowd bytes; gated-by-flag so P3-1
      round-trip unchanged; 590 parity + 116 ped green)*
- [x] **P3-3** Remote reconstruction render pipeline + demo *(`PedRemoteReconstructor`: playout-delay render
      clock + holonomic capped-correction smoothing over P3-1's receiver; `--ped-remote` scene renders the
      crowd from the gated multicast wire, not the sim; tests: server==IG within render tolerance over the
      GATED wire — low-power exact, high-power ≤0.25 m; cap absorbs a 1 m snap over 2 frames (no teleport);
      determinism; 590 parity + 120 ped green)*

**Stage P3 (DDS-multicast networking) COMPLETE** — wire codec + reconstruction (P3-1), DR-error gating +
bandwidth governor (P3-2), remote render with no-pop smoothing (P3-3); all hermetic on the InMemory byte
loopback; the live CycloneDDS binding remains a separate out-of-`Traffic.sln` concern.

## Stage P4 — Engine coordination seams (Core; with lane-engine session)

- [x] **P4-1** Engine TLS-crossing signal projection (gate reads live signal; parity hash unchanged) —
      **DONE: Sim.Core coordination resolved** (core handed to the ped track; main's finished P2-G/junction
      work merged into this branch, full gate re-green at **630 parity (+3 skip)** / 142 ped / 2 / 1 with the
      projection layered on — inert against the new, larger baseline).
      `Engine.TryGetTlLinkState(tlId, linkIndex, out char)` — a pure read-only accessor over the
      SAME private `TlLinkStateChar` the frozen parity path uses (actuated `CurrentState` / static formula
      split), at `CurrentTime`; reaches the walkingarea→crossing links the vehicle-facing `TlStates`
      structurally excludes. Never called from `Step()`, mutates nothing → parity hash `909605E965BFFE59`
      unmoved (full 596 (+3 skip) suite + the DrModelRead hash tests green). Ped side stays Engine-free via a
      delegate seam: `LiveCrossingSignal(Func<double,char>)` + `CrossingSignalFactory.ForCrossingLive(net,
      crossing, readLive)`; the caller wires `readLive = (tl,li) => engine.TryGetTlLinkState(tl,li,out c)?c:'r'`.
      3 tests (142 ped total): over a full stepped POC-0 cycle the live gate opens/closes **exactly** with
      the Engine's crossing signal and agrees char-for-char with today's XML re-derivation (faithful drop-in;
      the actuated case where they'd diverge rides the already-parity-tested `TlLinkStateChar` actuated
      branch), plus the projection's unknown-tl / out-of-range guards. The live path is **opt-in**: an
      Engine-coupled integration wires `ForCrossingLive`; Engine-less ped contexts (demos, pure-ped tests)
      keep the exact-for-static XML re-derivation, since the live read needs an Engine to query. No
      production consumer wires the gate to an Engine today, so nothing to retrofit — the seam is ready for
      the first car+ped coupling that needs it.

## Stage P5 — Evac generalization

- [x] **P5-PRE** Synthetic pedestrian evac-district net (3×3 sidewalk grid; incident centre + 4 corner safe
      zones) — bakes through `SumoNavMesh`, centre→corner routes around blocks (arc >1.2× straight); 2 tests
- [x] **P5-1 (B)** `Sim.Evac` consumes `Sim.Pedestrians` on a real walkable net *(`EvacDistrictDirector` on
      the `PedestrianWorld` facade over the P5-PRE net: panic = `Remove`+`AddWalker`(→nearest safe zone)+
      `SetForcedHighPower` (forced promotion), fleeing ALONG sidewalks — routed arc 1.33–1.78× the radial
      line, never through blocks; everyone escapes; `--evac-district` demo; 5 tests; legacy radial
      `EvacDirector` untouched — all 29 existing Evac tests still green; 595 parity + 129 ped green)*

**Stage P5 (evac generalization) COMPLETE** — the full (B) unification: the pedestrian side of evac now
runs on the `Sim.Pedestrians` engine over a real (synthetic) walkable net, routing panicked peds to safe
zones along sidewalks with forced promotion; the legacy car-centric radial evac stays intact alongside it.

## Stage P6 — Scale hardening + on-target validation

- [x] **P6-1** On-target (16+‑core) benchmark run; replace 4-core estimates in findings — **DONE** on the
      owner box (Intel Core Ultra 9 275HX, `ProcessorCount = 24`, Win11, High-perf, Release, 3×/median).
      Headline (100k, parallel): PedLod **stable = 35.9 ms/step (27.9 steps/s)**, **churn = 54.9 ms/step
      (18.2 steps/s)** — both interactive (vs 4-core ~12.5 / ~3.5 steps/s); raw `OrcaCrowd.Step` 100k =
      50.1 ms/step (20.0 steps/s, 6.45× serial); bandwidth **36.75 / 182.4 / 294.4 Mbit/s** confirmed under
      the 500 Mbit/s budget (byte-identical). All 3 success conditions PASS. Full report:
      `docs/PEDESTRIAN-P6-1-RESULTS.md`; findings docs updated with on-target columns (POC7A/7B/7C).
- [ ] **P6-2** Region decomposition — **GO (triggered by P6-1, confirmed by the combined-load test)**.
      P6-1: the `Sim.BenchCrowd` 100k thread sweep plateaus at ~8–16 threads and is **flat 16→24** on a
      24-physical-core box (efficiency 69% @8t → 42% @16t → 28% @24t) — memory-bandwidth bound + P/E split.
      **Combined-load test (`docs/PEDESTRIAN-COMBINED-LOAD-RESULTS.md`)** makes it concrete: under the
      single-station envelope (~10k veh + 100k ped sharing ~half the box), **peds are the sole real-time-
      marginal engine** — vehicles clear by 29–53×, but ped heavy-churn **fails real-time when starved to
      4 cores (5.5 st/s vs 10 st/s bar)** and is core-scaling-limited, not contention-limited (the
      cross-engine tax is only ~5–9% at 6–8 cores). Porting the vehicle engine's byte-identical `--region`
      decomposition to `OrcaCrowd.Step` is the direct lever (needs ~1.4× ped churn per-core uplift to clear
      churn on a 6-core budget). Safe operating point until then: **peds ≥ 8 cores for churn-heavy loads**
      (the 4+8 split passes today).
- [x] **P6-3** Requirement-indexed property-test suite (reqs 1–7, each named; parity untouched) *(11 tests,
      each over ≥5 seeded configs with anti-vacuous guards: Req1 perf (parallel==serial bit-exact over
      78k comparisons + low-power 0 per-step samples), Req2 believability (ORCA no-overlap, worst margin
      0.1 mm), Req3 interactivity (car halts for ped + ped avoids moving blocker), Req4 evac (24/24 escape,
      routed 1.74×), Req5 disjoint crowds (accumulate on red, drain on green), Req6 parking (min-sep +
      board/alight events), Req7 networking (server==IG 2.7e-7 m + 100k mix ≤17 Mbit/s < 500); parity hash
      `909605E965BFFE59` unmoved; 596 parity + 139 ped green)*

## Stage P7 — Pedestrian visualization (existing 3D viewers, in-process + remote-DR)

- [x] **P7-0** HTML Sim.Viz pedestrian demo gallery (mobile-watchable; kept as demo/test scenarios for
      GitHub Pages) — five self-contained `--ped-*` scenes (crossing-gate, lod-promotion, od-routing,
      dodge-reroute, parking), wired into `gen-demos.sh` under a "Pedestrians" category; each renders
      from the real components (Engine.CrowdSource, PedLodManager+InterestField, PedDemand+SumoNavMesh,
      RerouteDriver, LotCoupling), screenshot-verified, `everPromoted=True` on the LOD scene *(user
      requested HTML demos before 3D)*
- [x] **P7-1** Native viewer (`Sim.Viewer`/Raylib) in-process ped render (generalize evac overlay;
      regime-aware; board/alight appear/disappear) — new domain-agnostic `PedOverlay` (IRenderOverlay):
      per-frame `PedRenderPoint(X,Y,Regime,Visible)` snapshot published in the OnAfterStep hook; regime from
      `ModelOf` (FreeKinematic→HighPower else LowPower), escaped peds appear/disappear (omitted). Wired as
      `DemoKind.Pedestrian`/`DemoCategory.Pedestrians` with the "Pedestrian evac (district)" entry over the
      real `scenarios/_ped/evac-district` net (zero vehicles). Xvfb screenshot-verified: both regimes drawn
      distinct (slate low-power vs cyan high-power), HUD `pop/low/high/escaped` split live; in-sln gate
      596/139/2/1 green, Sim.Core untouched *(scheduled early for a visible demo)*
- [x] **P7-2** Native viewer remote ped render over DR/DDS (FreeKinematic extrapolate + PathArc follow;
      server==IG visual parity; no promotion pop) — new `RemotePedOverlay` (+ `IPedDemoOverlay` seam):
      the native analog of the HTML `BuildPedRemote` scene — same server ped-sim (`PedLodManager` sweep) →
      gated `PedReplicationPublisher` → `InMemoryPedReplicationBus` → `PedRemoteReconstructor`, drawn from
      the reconstructed IG poses with the server ground-truth as coincident outline rings (the parity made
      visible) + an on-screen `max |server−IG|` / wire-rate HUD. Wired as the `"lod-remote"` PedKind demo
      "Pedestrian remote (over the wire)" over `scenarios/_ped/poc0-crossing-plaza`. Verified first-hand:
      in-sln gate 596/139/2/1 green; a fresh headless run reproduced 98 promotion ticks (first t=29.4s) with
      peak render error **0.213 m** across the whole run incl. promotions (no pop), sub-kbit gated stream;
      Xvfb screenshot shows slate/cyan regimes distinct, rings hugging the discs, cyan promotions clustered
      at the amber interest source. (DDS transport for peds not built yet — in-process byte loopback is the
      full server→wire→IG→render proof; DDS-ped binding is a follow-up, same staging as vehicles.)
- [x] **P7-3** City3D (Godot 3D) ped render — CityLib gains `PedSimSource` (ped server-sim + byte-loopback
      wire, mirrors `SimSource`), `PedReconstructor` (wraps `PedRemoteReconstructor` → `ReconstructedPed` in
      Godot space via the one `CoordinateTransform`), `PedTransform`/`PedInstance` (ground-seated slim
      avatar, mirrors `CarTransform`) — all Godot-free; `Viewer/Main.cs` gains a `--peds` path (early-return
      gated, existing vehicle path untouched) with one ped `MultiMeshInstance3D` coloured by regime.
      In-process byte-loopback path (the full server→wire→IG→render proof); **DDS-ped transport deferred** —
      `Sim.Replication.Dds` has no ped topics (design "#### Pedestrians (P7-3)"). Verified first-hand:
      `CityLib.Tests` 48 pass (41+7 new, incl. an end-to-end real-wire test with a `sawAnyPed` anti-vacuous
      guard); in-sln gate 596/139/2/1 green (nothing in-sln touched); `screenshot.sh out.png --peds
      --shot-delay=10` (Xvfb+llvmpipe, Godot 4.7.1) renders slim slate/cyan avatars on the plaza with
      `peds=14 highPower=4` — both regimes distinct. *(user explicitly wanted Godot done here)*

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
