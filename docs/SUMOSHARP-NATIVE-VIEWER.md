# Native C# viewer — design (raylib-cs + DDS)

A native C# visualizer for **10k+ vehicle** scenarios, replacing the browser Canvas/WebSocket viewer for
scale work. The browser viewer (`src/Sim.LiveHost`) stays as the zero-install shareable demo; this is a
**parallel** native track. **No JSON / browser / JS on the hot path** — the remote transport is binary DDS.

Rationale (from the live-viz test pass, `docs/SUMOSHARP-LIVEVIZ-OUTCOMES.md`): a 2 Hz socket feed
reconstructed at 60 fps in a sandboxed browser is the *source* of the DR-jitter problem class, and Canvas 2D
won't scale to 10k. Rendering the authoritative snapshot natively removes the jitter class entirely for local
use, and DDS is the right fan-out transport for the remote case.

## Modes (one exe, `--mode`)

```
dotnet run --project src/Sim.Viewer -- --mode <local|remote|loopback> <scenarioDir|net.xml> [opts]
```

- **`local`** — owns the `Engine` + `SimulationRunner`; renders the **authoritative `Snapshot` every frame**
  (`PoseResolver`, dt=0). No transport, no dead reckoning, no interpolation → **no jitter**. The workhorse for
  big scenarios. This is where 10k must be fast.
- **`remote`** — DDS **subscriber only**; renders **dead-reckoned** poses, the render clock **paced off the
  DDS sample `source_timestamp`** (the fix proven in the browser pass), with playout-delay interpolation +
  monotonic clock ported from `HtmlPage.cs`. **View-only** (no interactivity — decided; a command topic is a
  possible later add). A publisher runs elsewhere.
- **`loopback`** — one process runs **both** the publisher (engine → `PublishScheduler` → DDS writers) and the
  subscriber/renderer, over DDS intra-host. The single-app DR test: exercises publish→DDS→subscribe→DR→render
  with no second machine.

## Projects (both OUTSIDE `Traffic.sln` — they pull CycloneDDS + raylib; the hermetic gate never sees them)

- **`src/Sim.Viewer.Core`** — no raylib dependency, so it is **headless-testable**:
  - `EngineHost` — owns `Engine`+`SimulationRunner`, scenario/sandbox modes, restart, random-traffic toggle,
    obstacle inject (lifted from `SimHost`, minus the web/JSON). Exposes the authoritative `Snapshot`.
  - `DdsPublisher` — `EngineHost` + `PublishScheduler` → DDS writers (state + lifecycle + geometry + TL).
  - `DdsSubscriber` — DDS readers → decoded frames + a short per-vehicle history buffer.
  - `DrClock` — the ported pacing: long-baseline rate off `source_timestamp`, strictly-monotonic render
    clock, playout-delay interpolation with extrapolation fallback.
- **`src/Sim.Viewer`** — the desktop window. **raylib-cs 7.0.1** draws the WORLD (`Camera2D` pan/zoom/pick,
  roads = fill/casing/dashed centreline/chevrons, oriented per-vehicle rectangles via `DrawRectanglePro` sized
  per vehicle, TL dots). **Dear ImGui via `rlImgui-cs` 3.2.0 + `ImGui.NET`** draws all UI CHROME — the
  controls panel (mode radios, restart/clear buttons, random-traffic checkbox, DR-delay slider, smoothing
  toggle) and the **perf-diagnostics panel** (fps, frame min/avg/p99, DDS samples/s, decode ms, clock health:
  monotonic / back-steps / playout-delay, vehicle count). GPU-batched draw for 10k.

### UI + font (decided)
Dear ImGui (not raw raylib text) is the UI layer — proper panels/menus/sliders/checkboxes matching the web
demo, and crisp text. **Bundle `DejaVuSans.ttf`** in `src/Sim.Viewer/assets/` (permissive license; copied to
output) and load it into the ImGui atlas via `io.Fonts.AddFontFromFileTTF(path, 18)` + `rlImGui.ReloadFonts()`
so it is identical on Linux and Windows (no reliance on system fonts). Keep ImGui UI strings **ASCII** (or load
extended glyph ranges) — the default atlas is Basic-Latin/Latin-1, so `—`/`→` render as `?`.

## Reused (already built)
`PoseResolver` + `LaneGeometry` (pose/chord/off-tracking), `FrameCodec` / `VehicleRecord` / `FrameChunker`
(binary wire), `DefaultPublishPolicy` / `PublishScheduler` (adaptive rate), the `.Dds` topics. The DR clock
logic ports from `HtmlPage.cs`.

## DDS topics
Existing in `Sim.Replication.Dds`: `DdsWireFrame` (opaque-blob state, canonical), `DdsVehicleBatch`
(structured state), `DdsVehicleLifecycle` (keyed spawn/despawn + dims). **New (this track):**
- **`DdsNetworkGeometry`** — durable (`TRANSIENT_LOCAL`) lane polylines, chunked, so a remote viewer draws
  roads without the net file. (local/loopback can read the net directly.)
- **`DdsTlState`** — low-rate traffic-light state (controlled-lane handle + signal char), so **remote**
  viewers show signals. (Decided: TL is NOT on the high-rate DR packet; it rides this separate low-rate topic,
  same pattern as lifecycle.)

## Feature parity with the web demo
Roads/lanes, sized oriented vehicles, camera pan/zoom/pick, obstacle-drop + restart + random-traffic toggle
(local/loopback only — remote is view-only), adaptive-publish HUD (sent X/Y), chord + off-tracking pose, the
delay slider (interp/extrap) for remote/loopback, smoothing toggle, and the perf overlay.

## Headless verification (this Linux VM)
Proven working: raylib-cs 7.0.1 renders under Xvfb + Mesa software GL. Screenshot recipe:
```
LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe xvfb-run -a -s "-screen 0 1280x800x24" \
  dotnet run -c Release --project src/Sim.Viewer -- --mode local <scenario> --screenshot out.png --frames 120
```
So both the DDS/DR data path (console, `Sim.Viewer.Core`) **and** the rendered visuals (Xvfb screenshot) are
self-verifiable here; live interactive use + 10k perf are verified on a real GPU (Windows).

## Status (2026-07)
**P0, P1, P2 (a+b), and the functional half of P3 are DONE and on `main`** (commits `58d1ed4`, `a8df812`,
`1f269f5`, `69e682a`, `eb6449f`) — each Sonnet-built + Opus-reviewed (build + Xvfb screenshot + code read;
the DDS/DR paths run headlessly here). Working today: `--mode local` (authoritative render), `--mode
loopback` (single-process DR over DDS), `--mode publish` + `--mode remote` (two-process, view-only, with
`TRANSIENT_LOCAL` QoS so a late joiner gets the network), `--selftest`. **Only remaining: the 10k-vehicle
60 fps perf pass — needs a real GPU** (this VM is software-GL only), so it is a Windows task, not done here.

### Running it
```bash
# local (owns the engine, no DR):
dotnet run --project src/Sim.Viewer -- --mode local samples/junctions/cross/net.net.xml
# single-process loopback DR over DDS (delay 0=extrapolate, >0=interpolate):
dotnet run --project src/Sim.Viewer -- --mode loopback scenarios/15-reroute --delay 0.4
# two processes: publisher (headless) then a late-joining view-only remote viewer:
dotnet run --project src/Sim.Viewer -- --mode publish samples/junctions/cross/net.net.xml   # terminal A
dotnet run --project src/Sim.Viewer -- --mode remote                                         # terminal B
# headless screenshot (this VM): LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe xvfb-run -a <cmd> --screenshot out.png --frames 200
```
### Remaining Windows-GPU task (10k perf)
Run `--mode local` (and `loopback`) with ~10k vehicles on a real GPU, profile the frame time, and add GPU
instancing/batching for the vehicle quads if raylib's immediate-mode `DrawRectanglePro` loop doesn't hold
60 fps. Everything else (DDS transport, DR pacing, QoS late-join, the whole render feature set) is verified.

## Build phases (each = one Sonnet delegation, Opus reviews)
- **P0 — scaffold + local render.** `Sim.Viewer.Core` (`EngineHost`) + `Sim.Viewer` (raylib window +
  rlImgui + bundled TTF; `Camera2D` fit; roads; authoritative vehicles from `Snapshot.PosX/Y/Angle/
  Length/Width`; minimal ImGui HUD). `--screenshot`/`--frames` for headless. Done: Xvfb screenshot of
  `samples/junctions/cross` shows roads + oriented vehicles + a readable ImGui HUD.
- **P1 — local interactivity + render polish + diagnostics.** Camera pan/zoom/pick; obstacle drop; restart;
  random-traffic toggle; ImGui controls panel + perf-diagnostics panel; road polish (casing/dashes/chevrons),
  speed-coloured vehicles, TL dots (from the engine in local). Done: Xvfb screenshots across cross/bend/acute
  + a scenario; obstacle-drop screenshot shows the block + queue.
- **P2 — DDS topics + loopback DR.** Add `DdsNetworkGeometry` (durable, chunked) + `DdsTlState` (low-rate) to
  `Sim.Replication.Dds`; `DdsPublisher` + `DdsSubscriber` + `DrClock` (port the monotonic / long-baseline-rate
  / playout-delay-interp logic from `HtmlPage.cs`, paced off DDS `source_timestamp`); **loopback** mode +
  delay slider. Done: a headless console DDS+DR harness (in `Sim.Viewer.Core`) asserts round-trip + a
  monotonic clock, and an Xvfb loopback screenshot matches local.
- **P3 — remote mode + QoS + 10k perf.** Separate publisher/subscriber; QoS (RELIABLE/KEEP_LAST, DEADLINE,
  TIME_BASED_FILTER, LATENCY_BUDGET, multicast); GPU-batched 10k. Remote is **view-only**. 10k perf verified
  on a real GPU (Windows).

## Delegation model (save Opus budget)
Volume build work is delegated to **Sonnet** general-purpose subagents (one per phase, sequential — each
builds on the last); **Opus is the hard reviewer** (reads the diff, builds, runs the Xvfb screenshot + the
console DDS/DR harness, and only then accepts/commits). Every delegation names: exact files to read, files to
create/edit, the APIs to call, the run command, and the numeric/visual done-condition — nothing crosses the
boundary except the prompt. The parity gate is irrelevant to these projects (both are OUT of `Traffic.sln`),
but a Sonnet agent must never edit `src/Sim.Core` or anything in `Traffic.sln` as part of this track.

## Not this track
`src/Sim.Viz` is the separate OFFLINE playback viewer (another workstream) — unrelated. This is the LIVE
native viewer; the web `Sim.LiveHost` stays as-is.
