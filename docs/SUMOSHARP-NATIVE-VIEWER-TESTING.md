# Native viewer — Windows GPU handoff (10k perf pass + interactive verification)

Handoff for a Claude Code (or human) session on a machine with a **real GPU + display** (e.g. Windows) to
(1) run the **10k-vehicle 60 fps perf pass** — the one native-viewer item that cannot be done on the
software-GL CI/Linux VM — and (2) visually verify the interactive bits that were only checked headless (under
Xvfb) so far.

> Design of record: `docs/SUMOSHARP-NATIVE-VIEWER.md`. Everything else (DDS transport, DR pacing, QoS
> late-join, the whole render feature set) is already built, on `main`, and headlessly verified. Paste the
> "Session prompt" block at the bottom into a fresh session on the Windows box (after checking out `main`).

---

## What the native viewer is

`src/Sim.Viewer` (raylib-cs 7.0.1 world + Dear ImGui via rlImgui-cs 3.2.0 UI) + `src/Sim.Viewer.Core`
(engine host + DDS publisher/subscriber + `DrClock`). It is the **native, 10k-scale counterpart** to the
browser demo (`src/Sim.LiveHost`, which stays as the shareable zero-install demo). **Not** `src/Sim.Viz`
(the separate offline-playback viewer). Both viewer projects are **out of `Traffic.sln`** (they pull
CycloneDDS + raylib), so the hermetic parity gate never touches them.

Four modes (`--mode`):
- **`local`** — owns the `Engine`+`SimulationRunner`, renders the authoritative `Snapshot` every frame. No
  transport, no dead reckoning, no jitter. **This is the mode the 10k perf pass targets.**
- **`loopback`** — one process: publisher + subscriber over DDS, renders the dead-reckoned poses. DR-delay
  slider (0 = extrapolate, >0 = interpolate).
- **`publish`** — headless engine + DDS publisher (no window).
- **`remote`** — view-only DDS subscriber + renderer, no engine; a late joiner gets the network via
  `TRANSIENT_LOCAL` QoS.

## Prereqs (Windows)
.NET 8 SDK (`winget install Microsoft.DotNet.SDK.8`). The native deps ship as NuGet win-x64 binaries
(`Raylib-cs`, `rlImgui-cs`, `CycloneDDS.NET`) — no manual native install. First build takes a minute.

## Run
```powershell
git fetch origin main; git checkout main
dotnet run -c Release --project src\Sim.Viewer -- --mode local samples\junctions\cross\net.net.xml
dotnet run -c Release --project src\Sim.Viewer -- --mode loopback scenarios\15-reroute --delay 0.4
# two processes (separate terminals):
dotnet run -c Release --project src\Sim.Viewer -- --mode publish samples\junctions\cross\net.net.xml
dotnet run -c Release --project src\Sim.Viewer -- --mode remote
```
Controls (local/loopback): drag = pan, wheel = zoom, click a road = drop obstacle, `d` = toggle diagnostics;
ImGui panel has restart / clear obstacles / inject-random-traffic / DR-delay slider.

---

## TASK 1 (primary) — 10k-vehicle 60 fps perf pass, `--mode local`

The committed demo nets are small and the sandbox spawner caps at ~80 vehicles, so **getting to 10k is part
of the task** — it will not happen out of the box:
1. **Get a big fleet on a big net.** Options, easiest first:
   - Add a `--fleet <N>` / `--spawn-cap <N>` CLI option that raises `EngineHost`'s spawn cap (currently the
     `Snapshot.Count > 80` guard in `src/Sim.Viewer.Core/EngineHost.cs.SpawnOne`) and front-loads a larger
     spawn burst, and point `--mode local` at the largest available net. A single small junction can't hold
     10k, so also:
   - Generate a large grid net with SUMO's `netgenerate` (e.g. `netgenerate --grid --grid.number 40
     --grid.length 200 -o grid.net.xml`) and spawn ~10k random trips on it. (SUMO is available in this repo's
     environment per `CLAUDE.md`; on Windows install it or copy a generated `.net.xml` over.)
   - Or drive a heavy `.rou.xml` demand in scenario mode.
2. **Measure.** The ImGui diagnostics panel (`d`) already shows fps + frame-time min/avg/p99 + vehicle count.
   Record frame time at ~1k, ~5k, ~10k vehicles.
3. **If raylib's immediate-mode `DrawRectanglePro` loop doesn't hold 60 fps at 10k** (likely the bottleneck):
   add **GPU instancing/batching** for the vehicle quads — build one mesh + a per-instance transform buffer
   and issue a single instanced draw (`Raylib.DrawMeshInstanced` in raylib-cs), instead of one
   `DrawRectanglePro` call per vehicle. The road draw (static) can be baked into a `RenderTexture` or a
   retained vertex buffer once, redrawn each frame. Keep the visual output identical.
4. Report the frame-time numbers at each fleet size, before/after any instancing, and the GPU used.

Perf fixes live in `src/Sim.Viewer/Renderer.cs` (the draw loop) and possibly `EngineHost.cs` (fleet/cap
option). Do NOT change the pose math or `DrClock`.

## TASK 2 (secondary) — visually verify what was only checked headless

These were built + logic-verified under Xvfb but never driven by a human on a real display. Confirm each and
screenshot anything wrong:
- **Interaction:** drag-pan and wheel-zoom feel right; clicking a road drops an obstacle and cars queue
  behind it; `clear obstacles` resumes; `restart` rewinds; the `inject random traffic` checkbox works; `d`
  toggles the diagnostics panel.
- **DR feel (loopback/remote):** at `--delay 0` (extrapolate) reactive vehicles should visibly *snap* when a
  correcting packet arrives; raising the delay to ~0.4–1.0 s should make motion *smooth* (interpolation).
  Confirm the "clock back-steps" diagnostic stays at/near 0 and there's no runaway/rubber-banding.
- **Remote late-join:** start `--mode publish`, wait, then start `--mode remote` — it should show
  `connected: yes` / `geometry: received` and render the network + vehicles + TL dots.
- **Fidelity:** roads/lanes/dashes/chevrons, per-vehicle-sized oriented rectangles (trucks longer),
  speed-colouring (green→red), TL dots, off-tracking bow on long vehicles through curves.

## Guardrails
- Both viewer projects are **out of `Traffic.sln`** — edit `src/Sim.Viewer` / `src/Sim.Viewer.Core` freely.
- Do NOT edit `src/Sim.Core` / `src/Sim.Ingest` / `Traffic.sln`. If you touch `src/Sim.Replication*` (in the
  hermetic solution) you MUST keep the gate green before pushing:
  - `dotnet test Traffic.sln` → **368 passed / 3 skipped / 0 failed**
  - `dotnet run -c Release --project src/Sim.Bench` → determinism hash **`909605E965BFFE59`**, single AND parallel.
- Work on a branch `claude/native-viewer-perf-<short>`; do NOT push straight to `main`.

## Context
`docs/SUMOSHARP-NATIVE-VIEWER.md` (design + phase status), `docs/SUMOSHARP-DEADRECKONING.md` (§6 pose, §7
adaptive rate, §8 interp/extrap). Key code: `src/Sim.Viewer/Renderer.cs`, `src/Sim.Viewer/Program.cs`,
`src/Sim.Viewer.Core/{EngineHost,DdsPublisher,DdsSubscriber,DrClock,DdsQos}.cs`,
`src/Sim.Core/PoseResolver.cs`.

---

## Session prompt (paste into a fresh session on the Windows box)

```
Read docs/SUMOSHARP-NATIVE-VIEWER-TESTING.md in this repo (branch main) and carry out the two tasks it
describes for the native C# viewer (src/Sim.Viewer). You are on a machine with a real GPU + display, so
actually open the window and LOOK — this is the perf + interactive pass that the software-GL CI VM could not do.

TASK 1 (primary): the 10k-vehicle 60 fps perf pass in --mode local. Getting to 10k is part of the task (the
demo nets are small and the spawner caps ~80) — add a --fleet/--spawn-cap option and use a large net
(netgenerate a grid, or a heavy .rou.xml). Measure frame time (the 'd' diagnostics panel shows fps/min/avg/
p99/veh) at ~1k/5k/10k. If the immediate-mode DrawRectanglePro loop can't hold 60 fps at 10k, add GPU
instancing for the vehicle quads (Raylib.DrawMeshInstanced) and bake the static roads; keep the visuals
identical. Report numbers before/after + the GPU used.

TASK 2 (secondary): visually verify the interactive bits only checked headless — pan/zoom/pick, obstacle
drop+queue, restart, random toggle, 'd' diagnostics; the DR-delay feel (delay 0 snaps on reactive vehicles,
higher delay = smooth interpolation, back-steps ~0); and a --mode publish then --mode remote late-join
(connected: yes, geometry: received). Screenshot anything wrong.

Perf/interaction fixes go in src/Sim.Viewer/* and src/Sim.Viewer.Core/* (both OUT of Traffic.sln). Do NOT
edit src/Sim.Core/Sim.Ingest/Traffic.sln; if you touch src/Sim.Replication* keep the gate green (dotnet test
Traffic.sln = 368 passed/3 skipped/0 failed; Sim.Bench hash 909605E965BFFE59 single+parallel). Do NOT change
the pose math or DrClock. Work on a branch claude/native-viewer-perf-<short>, not main.
```
