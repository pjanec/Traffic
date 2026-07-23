# Live-city in the interactive viewers — tracker

Checklist for `docs/LIVE-CITY-VIEWERS-TASKS.md` (design: `docs/LIVE-CITY-VIEWERS-DESIGN.md`). A box is
ticked **only** after Opus verifies the task's success conditions first-hand (diff read + gate/smoke
re-run; desktop-only aesthetic sign-off is the user's and noted as such — never faked from a headless run).

**Standing gate on every `src/`-touching tick:** `dotnet test Traffic.sln -c Release` at the committed
baseline + `Sim.Bench` determinism hash unchanged (single + parallel). Capture the baseline fresh on the
first `src/` task of each stage and repeat the numbers here as each box is ticked.

Baseline (captured 2026-07-22, clean checkout at `d1b1638`): **895 pass / 4 skip** —
ParityTests 654/4, Pedestrians 227, IgBridge 11, DotRecast 2, Host 1; determinism hash
**`D96213B7BB4021A7`** (single == parallel). This is the standing bar for every `src/`-touching tick.
Re-capture fresh per task (other sessions may edit the engine).

## Stage A — SumoSharp.LiveCity shared host  ✅ (commit a8c7fae)
- [x] **A1** project scaffold (`src/Sim.LiveCity`, packs `SumoSharp.LiveCity.0.1.0`, net8.0;netstandard2.1)
- [x] **A2** `LiveCitySim`/`LiveCityConfig`/`LiveCitySnapshot` — coupled recipe, exact per-tick order, publishes cars+peds on the in-mem wire + direct `Sample()` read-back. Opus-verified first-hand: 3/3 tests green (PeakCars=163/PeakPeds=160/PeakOccupiedCrossings + CarYieldObservations ON=763 vs OFF=220; float-exact determinism; YIELD A/B), wire non-vacuous (History>0, crowd frame>0). **Zero existing files touched → parity gate unaffected by construction** (ParityTests 654/4 + hash `D96213B7BB4021A7` intact).

## Stage B — Raylib 2D live-city (real-time)  ✅ (commit 3757508)
- [x] **B1** `--mode live-city` + `LiveCityOverlay` (dedicated mode, NOT DemoKind — LiveCitySim owns its coupled engine + pre-step gate). Cars via the shared KinematicReconstructor path (`RenderHelpers.PumpAndBuildVehicleDraws` widened DdsSubscriber→IReplicationSource, backward-compatible), peds regime-colored discs. Opus-verified: headless smoke reconstructedCars=161/peds=160/peakOccupied=46; Xvfb screenshot shows both (HUD cars:148 peds:160); ParityTests 654/4 + hash intact.
- [x] **B2** click-select vehicle + SUMO-id label; `PickNearest` pure helper, 5/5 unit tests (incl. tie-break fix). Opus-verified.

## Stage C — Shared record/replay + playback (Raylib)  ✅ (commit ef4969a)
- [x] **C1** `.simrec` format (`SimRecFormat`) + `RecordingReplicationSink` (car tee) + PEDFRAME snapshots; round-trip test green (geometry=1, lifecycle/frame>0, pedFrames==steps). Opus-verified.
- [x] **C2** `ReplicationFileSource` (wraps InMemory bus, seekable via `PlaybackClock`) + `PedFrameTrack`; replay matches live arc-length within 0.05 m; SeekTo (interior + backward) == linear playthrough (3 dp + same lane). Opus-verified.
- [x] **C3** Raylib playback panel (pause/restart/step/speed + drag-to-seek timeline) + `--record`/`--replay`. Xvfb screenshot shows the scrubbing timeline (t=22.62/75.00s). Additive only → ParityTests 654/4 + hash intact; Sim.Viewer.Tests 9/9. Opus-verified.

## Stage D — City3D local (live + replay, click, Z)  ✅ (commits 65d1db5, 900734c)
- [x] **D1** `--live-city` renders cars + peds in one Godot process over demo_city/box (mutual exclusion dropped; `LiveCitySource` wraps the host; cars reuse the existing Reconstructor path; cropped road meshes). Legacy `--scenario`/`--peds` intact. Opus-verified via Xvfb screenshots (car box + crowd).
- [x] **D2** honor Z: local path feeds Z-aware `LocalLanes`→`RoadMeshBuilder`; synthetic elevated-net fixture + CityLib test (ramp→non-constant Y, flat→Y≈0). Opus-verified (CityLib 61/61).
- [x] **D3** Godot playback panel (Play/Pause/Restart/frame-step/speed + drag-to-seek `HSlider` + time label; Space/←/→) + `--live-city --replay <file>`. Opus-verified: Xvfb replay screenshot shows the panel scrubbing (t=10.1/75.0s) with cars from the `.simrec`.
- [x] **D4** click ray-pick (screen-space `UnprojectPosition` + `VehiclePicker.PickNearestScreen`, 6/6 tests) → emissive ring + id label (live = real SUMO id, replay = handle until Stage E adds name on the wire). Polish: closer camera + cylinder peds for visibility. Opus-verified. ParityTests 654/4 + hash intact.

## Stage E — City3D remote (combined DDS)
- [x] **E1** Z on the wire — `LaneGeo.Z` additive (`hasZ` flag + appended block, GeometryCodec v1→v2, v1 still parses); `Lane.ShapeZ` threaded; DDS geometry blob carries it; consumers un-flattened. Opus-verified: codec round-trip + real DDS loopback (ramp 0,4,8). **Parity byte-identical (independently re-run: 654/4 + hash `D96213B7BB4021A7`).** (commit c95ecdd)
- [x] **E2** vehicle name once-per-spawn — `LifecycleRecord.Name` from `VehicleId`; `IReplicationSource.Names` table (all 3 bindings); `DdsVehicleLifecycle` `FixedString64 Name`. Per-frame record UNCHANGED. Opus-verified (DDS loopback name PASS). (commit c95ecdd)
- [x] **E3** combined cars+peds DDS producer (`Sim.Host.App --live-city`), one net — LiveCitySim gains an additive ped-tee; inmem self-test vehicles=163 + ped crowd/lifecycles nonzero. Opus-verified. (commit 71c5a32)
- [x] **E4** dual subscriber (`--transport=dds --live-city`) — both `DdsSubscriber` + `DdsPedReplicationSource` on one participant; **two-process DDS round-trip renders cars+peds from the wire** (cars=160 peds=164, Z-aware roads, real ids from `Names`). Opus-verified via Xvfb screenshot. ParityTests 654/4 + hash intact. (commit 71c5a32)

Status: **ALL STAGES COMPLETE (A–E).** The coupled cars+pedestrians+crossing-yield "live city" now runs in all three viewers — Raylib 2D, City3D local (live + `.simrec` replay with a scrubbable timeline + click-to-identify), and City3D remote over a single combined DDS producer — with Z/elevation and vehicle names carried additively on the wire. Parity byte-identical throughout (ParityTests 654/4, `Sim.Bench` hash `D96213B7BB4021A7`); every `src/` change additive/render-side; the `CrowdSource` golden path untouched.

Deferred (owner-confirmed, separate branch): cooperative lane-change overlap fix + crossing tunneling.
Needs the owner's GPU/desktop for final sign-off: the aesthetic smoothness look on a real GPU, and a true multi-box DDS / multi-monitor test.

Deferred (owner-confirmed, separate branch): cooperative lane-change overlap fix
(`docs/LANE-CHANGE-OVERLAP-SPEC.md`) and crossing tunneling.
