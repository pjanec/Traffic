# 3D city demo — tracker

Checklist for `docs/DEMO-CITY3D-TASKS.md` (design: `docs/DEMO-CITY3D-DESIGN.md`). A box is ticked ONLY
after Opus confirms the task's success conditions first-hand (diff read + gate/self-test re-run; the
desktop-only parts are confirmed by the user and noted as such — never faked from a headless run).

**Standing gate on every `src/`-touching tick:** `dotnet test Traffic.sln -c Release` at the committed
baseline + `Sim.Bench` determinism hash unchanged (single + parallel). Capture the baseline numbers on
the first `src/` task and repeat them here as each box is ticked.

## Stage 0 — Foundations (publisher package + local feed)
- [x] **T0.1** `SumoSharp.Host` (`ReplicationPublisher`) — Opus-verified first-hand: 1:1 port of `DdsPublisher`'s snapshot→record half (scheduler tol, `LaneWindow` offset, spawn/despawn, 1s TL gate all match), only ns2.1 array-copy deviation; builds both TFMs; packs `SumoSharp.Host.0.1.0.nupkg`; non-vacuous test (multi-sample guard) 1/1; full gate **465/0/3 unchanged**; hash **909605E965BFFE59** single+parallel unchanged.
- [x] **T0.2** DRY rewire — Opus-verified first-hand: DDS-write half moved verbatim to `Sim.Replication.Dds.DdsReplicationSink` (same writers/QoS/chunking/bytes); `DdsPublisher` now a 46-line delegator over `ReplicationPublisher` (323→46 lines, no leftover translation logic, public surface intact). `LoopbackSelfTest` PASS with identical numbers (3 lanes / 1 veh / 1 TL); main gate 465/0/3 + hash `909605E965BFFE59` unchanged. Escape hatch not needed.
- [x] **T0.3** local feed — Opus-verified first-hand: `build.sh --pack-only` packs 5 pure-C# packages into `local-nuget/`; probe resolves `SumoSharp.Host` **from the local feed**; with feed emptied + global cache purged, restore fails `NU1101 … PackageSourceMapping is enabled, the following source(s) were not considered: nuget.org` (no nuget.org fallback proven); `local-nuget/` git-ignored (after a `.gitignore` inline-comment fix).

## Stage 1 — Local single-viewport demo (M1, public-facing) — no `src/` changes
> Structure: logic in `CityLib` (headless-testable), Godot `Viewer` is thin glue. With network=Full, the
> Godot 4 (.NET) binary is fetched ephemerally by `fetch-godot.sh` (non-GitHub mirror) and **runs headless
> here** — so `godot --headless` scene smokes are verifiable in-session; only the aesthetic sign-off is human.
- [x] **T1.0** `CityLib` + `CityLib.Tests` — Opus-verified: consumes `SumoSharp.*` 0.1.0 from the local feed (NU1101 without it, proven); not in `Traffic.sln`; tests build+run.
- [x] **T1.1** Godot `Viewer` skeleton — Opus-verified first-hand: Godot 4.7.1 (.NET) project references `CityLib`; `dotnet build` resolves `SumoSharp.*` from the local feed + `Godot.NET.Sdk` from nuget.org; `run-smoke.sh` runs it **headless here** (exit 0), `Main.cs` builds a `SimSource`, ticks, reconstructs, logs non-zero vehicle count, sim time 0→3.0s, quits. *(Caveat: `DrClock` is wall-clock-driven, so a fast headless loop shows near-frozen render motion — fidelity is proven in T1.2's real-time-sleep tests; T1.3+ headless motion/screenshots will need a real-time throttle or synthetic clock.)*
- [x] **T1.2** data path in `CityLib` — Opus-verified first-hand (CityLib.Tests **11/11**): `SimSource` (Engine+Runner+ReplicationPublisher→InMemoryReplicationBus), `Reconstructor` (DrClock→PoseResolver→DrPoseSmoother→Godot coords), `CoordinateTransform` (SUMO→Godot `(x,z,-y)`, navi→yaw), `ReplicationLaneShapeSource`. Asserts: monotonic X advance / no >0.5 m back-jump / net progress through red-light hold→green; local(Z-aware)↔wire lane-source seam agrees ≤0.05 m; reconstructed yaw matches **engine** heading <1°. *(wire `LaneGeo` is 2-D — elevation over the wire needs a future `GeometryCodec` Z-extension; local uses Z-aware `NetworkLaneSource`.)*
- [x] **T1.3** procedural roads — Opus-verified first-hand: `RoadMeshBuilder` (miter-limited ribbon, Godot-space verts, shoelace area) + 16/16 CityLib tests (area diff **0.02%** vs Σ len×width); Viewer renders one `ArrayMesh`/lane + camera/light; **Xvfb software-GL screenshot produced & viewed** (1600×900 PNG). Finding: NO committed scenario carries Z geometry (`_bench/city-organic-L2` "L2" = 2 lanes, not levels) — elevation threading proven via a synthetic net instead; real nets render flat (honest). *desktop: aesthetic sign-off pending*
- [x] **T1.4** procedural buildings — Opus-verified first-hand: `BuildingPlacer` marches non-internal edges, FNV-1a-seeded footprints/heights (6–60 m), set-back off the road; **21/21** tests incl. exact-float determinism (same seed→identical, diff seed→differs) + off-road + height-band; Viewer renders a `MultiMeshInstance3D` box-city (1364 on city-organic); **screenshot viewed** — grey box-city lines the streets. *desktop: aesthetic sign-off pending*
- [x] **T1.5** cars — Opus-verified first-hand: `CarTransform.ForVehicle` (pure) maps a `ReconstructedVehicle`→box instance (scale = Width×height×Length, ground-seated, yaw passthrough); **25/25** tests incl. scale==vType L×W end-to-end; Viewer renders a per-frame car `MultiMeshInstance3D` driven by the DR reconstruction; screenshot at t=30 shows 19 cars distributed on the town (a yellow car box visible on a street in the zoom crop). *(true-size cars are sub-pixel at whole-town zoom → closer camera in T1.7; smooth-motion fidelity proven in T1.2's real-time tests.)*
- [ ] **T1.6** simplified traffic lights (head colour == replicated signal) — *desktop: colours match, queues*
- [ ] **T1.7** scenario switch + scale polish (`09-traffic-light` / `city-mixed-1k` / `city-organic-L2`) — *desktop: believable scale*

## Stage 2 — Remote (M2)
- [ ] **T2.1** `src/Sim.Host.App` generic headless DDS host — publishes (dds + inmem), self-test green, gate unchanged
- [ ] **T2.2** viewer remote mode (DDS source), single channel — headless receive+reconstruct, render code unchanged vs local — *desktop: live remote render*

## Stage 3 — Video wall + performance (M3)
- [ ] **T3.1** auto-detect video wall (N windows/cameras, 1 subscription; default 1) — *desktop: spans monitors; N independent viewers*
- [ ] **T3.2** performance ladder (`city-30…15000`, `city-mixed-1k`) — host rates logged — *desktop: ~1k at interactive fps*

## Stage 4 — Close-out
- [ ] **T4.1** demo README + `PACKAGES.md`/`DEMOS.md`/root README wiring + fresh-clone dry run + final gate

Status: **Stage 0 COMPLETE (T0.1–T0.3, all Opus-verified). Fresh gate baseline: 465 passed / 0 failed / 3 skipped; determinism hash `909605E965BFFE59` (single + parallel). Next: Stage 1 (local viewer), structured CityLib-first so the data path + procedural math are headless-testable — the Godot engine binary is egress-blocked here, so `godot --headless` + on-screen visuals are desktop-only checks.**
