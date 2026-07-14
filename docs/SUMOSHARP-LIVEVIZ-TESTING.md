# Live visualizer — testing handoff

Handoff for a Claude Code (or human) session that **runs the SumoSharp live dead-reckoning viewer on a
machine with a real display** (e.g. Windows) and visually verifies it. The viewer was built and smoke-tested
headlessly in CI; this session is the human-in-the-loop visual pass.

> Paste the "Session prompt" block below into a fresh session on the target machine (after checking out
> `main`), or just follow it yourself.

---

## What the live viewer is

`src/Sim.LiveHost` — a small ASP.NET Core app that runs the C# traffic engine (`src/Sim.Core`) via a
`SimulationRunner` at a low **~2 Hz** sim rate and streams **lane-relative dead-reckoning state** over a
WebSocket. The browser (`src/Sim.LiveHost/HtmlPage.cs`, plain JS on a `<canvas>`) reconstructs and
extrapolates each vehicle's world pose at **60 fps** by walking the once-sent lane geometry, and demonstrates
an **adaptive publish rate** (only a subset of vehicles' state is sent each step; the client dead-reckons the
rest). It needs **no SUMO and no network** — just the .NET 8 SDK.

**It is NOT `src/Sim.Viz`** — that's a separate offline-playback viewer owned by another workstream. This
test is only about the live viewer, `Sim.LiveHost`.

### Two modes (auto-selected from the input path)
- **SCENARIO mode** — the input is a scenario dir containing a `rou.rou.xml` (+ `.sumocfg`). The viewer
  `LoadScenario`s it and drives the scenario's **own demand** — the exact vehicles that produce the golden
  trajectory — rendered live with dead reckoning. Deterministic, so it doubles as a live view of the parity
  run. Because the scenario's `config.sumocfg` is loaded, sublane scenarios run with their lateral model **on**.
- **SANDBOX mode** — the input is a bare `net.net.xml` (e.g. `samples/junctions/*`, which have no demand).
  The viewer `LoadNetwork`s it and a runtime **random-traffic spawner** keeps the roads busy.

### HUD controls
- **mode label** — `SCENARIO DEMAND` or `SANDBOX`.
- **restart** — rebuilds the sim from t=0 (re-queues the scenario demand / empties the sandbox), clears obstacles.
- **inject random traffic** (checkbox) — toggles the random spawner in either mode (default: **on** in
  sandbox, **off** in scenario). Tick it to mix random fill into a scenario, or to compare.
- **click** a road to drop an obstacle (cars queue behind it); **clear obstacles** button; **wheel** = zoom;
  **drag** = pan.

### Render-mode CLI arg (optional, order-independent, after the path)
- *(none)* — SUMO-exact tangent heading (what the goldens compare).
- `chord` — SUMO back→front chord heading (correct for long vehicles on curves).
- `corner` (or `offtrack`) — chord + swept-path off-tracking ("trucks swing wide").

> **Known limitation (verified 2026-07, live-viz test pass):** this arg currently has **no effect on
> the live viewer**. `Engine.RenderMode` only overrides the engine's derived world-pose floats
> (`X`/`Y`/`Angle`), but `SimHost.BuildFrameJson` streams only *lane-relative* state (`p`/`pl`/`s`/`a`/`lw`)
> — never the angle. The browser (`HtmlPage.resolvePose`) therefore **always** reconstructs the chord
> heading + off-tracking bow client-side, in every mode. So default and `corner`/`chord` runs render
> **identically**: long vehicles always swing wide; there is no "rides-the-centreline" default to compare
> against. To wire the arg through, `BuildFrameJson` would need to send the pose/angle (or gate the client
> bow on a mode flag).

---

## Run

Prereq: .NET 8 SDK (`winget install Microsoft.DotNet.SDK.8` on Windows if missing). It binds
`http://0.0.0.0:5055`; open **http://localhost:5055**. First build takes a minute.

```powershell
git fetch origin main; git checkout main      # or clone fresh

# SCENARIO mode — the scenario's real demand:
dotnet run --project src\Sim.LiveHost -- scenarios\15-reroute

# SCENARIO mode — sublane scenarios show LATERAL movement live (config sets lateral-resolution>0):
dotnet run --project src\Sim.LiveHost -- scenarios\60-sublane-drift
#   good lateral demos (LANDED, visible drift/offset): 60-sublane-drift, 61-sublane-sidebyside, 65-mixed-sublane
#   NOTE: 62-sublane-overtake shows NO lateral motion — its golden keeps both vehicles centred (posLat=0)
#         by design (it's the follow-*no*-overtake parity case), so don't use it to check lateral movement.
#   NOTE: 63/64 (overtake-wide / keep-lat-gap): the sublane speed-gain OVERTAKE decision (rung P2.4) is
#         DEFERRED/unlanded — its parity test is skipped and the engine mainline emits posLat=0, so the
#         viewer faithfully shows no drift there. Not a viewer bug; just not a lateral demo yet.

# SANDBOX mode — junction off-tracking demos (corner mode = swept-path swing):
dotnet run --project src\Sim.LiveHost -- samples\junctions\acute\net.net.xml corner
#   others: cross (signalized, TL dots), tee, bend
```

(Linux/macOS: same, with `src/Sim.LiveHost` and forward slashes.)

---

## What to verify — report per scenario/mode, screenshots welcome

1. **Mode label** matches the input (scenario dir → `SCENARIO DEMAND`; bare net → `SANDBOX`), and the
   random-traffic checkbox defaults correctly (off for scenario, on for sandbox).
2. **Scenario demand is faithful** — in scenario mode with random traffic OFF, the vehicles are exactly the
   scenario's (count/timing), not invented. On the sublane scenarios you should see vehicles **move
   laterally** within lanes (drift/overtake/side-by-side), not just ride the centreline.
3. **Roads** render with lane fill/casing, dashed centrelines, and direction chevrons.
4. **Vehicles** are oriented rectangles **sized per vehicle** (in sandbox ~1/3 are 12 m trucks, visibly longer).
5. **Motion is smooth** (~60 fps) despite the ~2 Hz sim — that's the dead reckoning filling the gaps. No
   stutter, no teleporting, no vehicles vanishing then reappearing (no ghosting).
6. **Off-tracking** (sandbox junctions `bend`/`acute`): long (12 m) vehicles swing **wide** of the lane
   centreline through the turn (rear tracks inside). NOTE: this is **always on** in the live viewer,
   reconstructed client-side — the `corner`/`chord` CLI arg does **not** change it (see the render-mode
   "Known limitation" above), so there is no default-vs-corner comparison to make here.
7. **`cross`** (sandbox, signalized): coloured traffic-light **dots** per controlled approach; cars queue on
   red, go on green.
8. **HUD stat**: e.g. `N vehicles · sent X/Y states/step (Z%) · sim 2.0/s → 60fps DR`. Confirm **X < Y** when
   traffic is steady/queued (adaptive rate deferring predictable + stationary vehicles), X ≈ Y right after
   spawn/acceleration.
9. **restart** rewinds the run (clock resets to ~0; scenario demand replays; sandbox empties then refills if
   the toggle is on). **inject random traffic** toggle adds/removes random vehicles live.
10. **obstacle**: click a road → red ✕ drops, cars queue behind it; **clear obstacles** resumes flow.

## Deliverable
A short report — per scenario/mode, what looked right and what didn't (screenshots welcome). Flag any visual
bug: wrong heading through curves, cars off the road, jitter/ghosting, mis-sized bodies, wrong TL colours,
obstacle not blocking, HUD not updating, mode/toggle wrong, sublane vehicles not moving laterally.

## If you fix something
- Viewer-only fixes live in `src/Sim.LiveHost/HtmlPage.cs` (client rendering/DR) or
  `src/Sim.LiveHost/SimHost.cs` (server frame/publish/modes). Program wiring is `src/Sim.LiveHost/Program.cs`.
- `src/Sim.Core` is the **parity path**. If you touch it you MUST keep the gate green before pushing:
  - `dotnet test Traffic.sln` → **365 passed / 3 skipped / 0 failed**
  - `dotnet run -c Release --project src/Sim.Bench` → determinism hash **`909605E965BFFE59`**, single AND parallel.
- Work on a branch `claude/liveviz-test-<short>`; do NOT push to `main` directly.

## Context docs
- `docs/SUMOSHARP-DEADRECKONING.md` — §5 lane-relative state, §6 pose-resolver (chord / off-tracking), §7
  adaptive publish rate.
- `docs/SUMOSHARP-API.md` §11 — the live demo. The pose math the JS mirrors is in
  `src/Sim.Core/PoseResolver.cs` and `src/Sim.Ingest/LaneGeometry.cs`.

---

## Session prompt (paste into a fresh session on the target machine)

```
Read docs/SUMOSHARP-LIVEVIZ-TESTING.md in this repo (branch main) and carry out the visual test it
describes for the SumoSharp LIVE dead-reckoning viewer (src/Sim.LiveHost). You are on a machine with a real
browser/display, so actually open http://localhost:5055 and LOOK at it — this is a human-in-the-loop visual
pass, not just a build check.

Cover both modes and the render modes: scenario mode on scenarios\15-reroute and on a couple of the sublane
scenarios (60-65, which should show lateral movement live), and sandbox mode on samples\junctions\{cross,
bend,acute} including `corner` mode for off-tracking. Exercise the restart button, the inject-random-traffic
toggle, and click-to-drop-obstacle. Report per scenario/mode what looked right and what didn't, with
screenshots, using the "What to verify" checklist.

Do NOT use src/Sim.Viz (that's the separate offline viewer). If you fix a viewer bug, keep it in
src/Sim.LiveHost/*; if you touch src/Sim.Core you MUST keep the gate green (dotnet test Traffic.sln = 365
passed/3 skipped/0 failed; Sim.Bench determinism hash 909605E965BFFE59 single+parallel). Work on a branch
claude/liveviz-test-<short>, not main.
```
