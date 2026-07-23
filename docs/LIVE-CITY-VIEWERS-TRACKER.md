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

## Stage D — City3D local (live + replay, click, Z)
- [ ] **D1** drop cars-XOR-peds; `--live-city` renders cars + peds over demo_city/box (legacy modes intact)
- [ ] **D2** honor Z on local road/car meshes; synthetic elevated-net test (non-zero Z→non-zero Y; flat→0)
- [ ] **D3** Godot playback controls + `--live-city --replay <file>`
- [ ] **D4** click ray-pick vehicle → highlight + id; scripted-pick test

## Stage E — City3D remote (combined DDS)
- [ ] **E1** Z on the replication wire (GeometryCodec + DDS geometry); round-trip on elevated net; hot path untouched
- [ ] **E2** vehicle name once-per-spawn on the wire; per-frame record unchanged; remote id resolves
- [ ] **E3** combined cars+peds DDS producer (`Sim.Host.App --live-city`), one net; inmem self-consume both
- [ ] **E4** dual subscriber (`--transport=dds --live-city`), two-process round-trip renders cars+peds + Z + ids

Status: **DRAFT — awaiting owner sign-off on the design before implementation begins.**

Deferred (owner-confirmed, separate branch): cooperative lane-change overlap fix
(`docs/LANE-CHANGE-OVERLAP-SPEC.md`) and crossing tunneling.
