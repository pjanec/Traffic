# Live visualizer — test-pass outcomes & next-architecture decision

Handoff back to the Linux VM / main dev session. Summarizes a Windows human-in-the-loop session that
(1) ran the visual test in `docs/SUMOSHARP-LIVEVIZ-TESTING.md`, (2) diagnosed and fixed dead-reckoning
smoothness bugs in the browser client, and (3) reached an architecture decision: **the browser
Canvas/WebSocket viewer is a demo, not the path to 10k-vehicle scenarios — replace it with a native C#
viewer in two modes (local no-DR, remote DR), DDS transport for the remote mode.**

All viewer work was client/host only (`src/Sim.LiveHost/*`). **`src/Sim.Core` was NOT touched — the parity
gate is unaffected and was not re-run** (365/3/hash still valid; nothing on the parity path changed).

Branch: `claude/liveviz-test-visual`.

---

## 1. Visual test pass (result: viewer faithful)

Ran `Sim.LiveHost` on Windows with a real browser (screenshots via headless Chrome + CDP; the WebSocket
stream inspected directly with a Node client). All 10 checklist items verified. Full report + screenshots
were published as an artifact during the session; the checklist verdicts:

- Mode label + random-traffic default (scenario off / sandbox on): OK.
- Scenario demand faithful; lateral motion renders on the LANDED sublane scenarios (60 drift 0→−1.4995,
  61 side-by-side ±1.5) matching goldens.
- Roads (fill/casing/dashed centreline/chevrons), oriented per-vehicle rectangles, 12 m trucks, TL dots
  (r/G/y, queue on red), adaptive publish X<Y, restart, random toggle, obstacle drop→queue→clear: all OK.

Two **documentation** inaccuracies were found and already fixed (commit `5a39f16`), neither a viewer bug:
- **A.** The `corner`/`chord` CLI render-mode arg is a **no-op for the live viewer**: `Engine.RenderMode`
  only rewrites the derived `X/Y/Angle` floats, but `SimHost.BuildFrameJson` streams only lane-relative
  state, so the client always reconstructs chord heading + off-tracking bow. Default and `corner` render
  identically. (To wire it: stream the angle, or gate the client bow on a mode flag.)
- **B.** The doc's headline sublane examples don't show lateral motion: `62-sublane-overtake` keeps both
  vehicles centred by design (parity "follow-no-overtake" case), and `63`/`64` are the **deferred/unlanded
  P2.4 rung** (skipped test; the offline `Engine.Run(40)` itself emits `posLat=0`). Viewer is faithful to
  the engine. Use 60/61/65 for lateral demos.

---

## 2. Dead-reckoning smoothness — root causes & fixes (browser client)

User reported jitter: hiccups, forward/back jumps (worst on turning vehicles), and periodic ~1 s stalls.
Diagnosed with a Node WebSocket timing probe + Chrome DevTools Protocol (rAF/frame-timing/long-task/clock
instrumentation). Findings, in order of discovery:

1. **Delivery is jittery, the sim is not.** Steps advance `dSim=1` with zero skips, but wall-delivery
   cadence is ~491 ms mean with a 208–533 ms spread (2 Hz sim × 50 ms send-loop quantization).
2. **Extrapolation of reactive vehicles is the core artefact (inherent, not a bug).** Measured one-step DR
   prediction error on the live stream: **braking vehicles overshoot by mean +1.9 m (up to +6.1 m)** →
   snap **back** when the truer, slower packet arrives; accelerating undershoot (~−0.7 m) → snap forward;
   constant-speed vehicles predict perfectly (smooth). This is why straight cruisers looked fine and
   turners/junction vehicles jumped. The streamed `a` (accel) is also mis-aligned with the next step, so
   the ½·a·dt² term doesn't help. **No amount of tuning fixes extrapolation of reactive traffic** — only
   interpolation (render in the past between two real packets) or a higher publish rate does.
3. **Three client-side bugs (all fixed) that made it worse than necessary:**
   - A PLL render-clock chasing the staircase `latestSim` → periodic velocity pulsing ("caterpillar").
   - The wall→sim clock could tick **backward** on delivery jitter → back-jumps even in interpolation.
   - Playback rate estimated from a **per-gap ratio** (`dT/dW` EMA) → a burst of two steps ms apart (common
     under CPU load) spiked the rate, so the clock **raced then snapped**, producing jerky ~1 s stepping
     at a full 60 fps. This is what the user hit while running two instances.

**Fixes applied (client only, `HtmlPage.cs`):**
- **Long-baseline playback rate** (total sim / total wall since first step) — immune to burst jitter.
- **Strictly monotonic render clock** (forward-only; holds instead of ever reversing).
- **Interactive DR mode: a `delay` slider** — `0` = extrapolate (predict ahead; shows the real overshoot
  artefact), raise it (e.g. 0.5–1.5 s) = **interpolate between two buffered packets** (exact real data,
  time-shifted; smooth, at the cost of latency). Falls back to extrapolation past the newest packet.
- **Optional pose smoothing checkbox** (extrapolation only; off by default — raw DR for diagnosis).
- **Diagnostics overlay (press `d`)**: live fps, worst frame gap, ws/s, `visibilityState`, vehicle count.

**Measured verdict after fixes:** clean Chrome (even GPU-disabled/software) renders **60 fps, 0 long tasks,
monotonic clock, 0 back-steps**. User confirmed: **1-second hiccups gone, browser smooth.** Residuals, by
design/known and accepted (we're moving off this stack): extrapolation still jumps on reactive vehicles
(inherent); a few back-jumps remain at high delay where interpolation falls back to extrapolation at
junctions (lane-window shift) and for adaptive-deferred vehicles.

---

## 3. Decision: replace the browser viewer with a native C# viewer (two modes)

Rationale: a Canvas 2D / WebSocket client will not scale to ~10k vehicles, and the whole DR-jitter problem
class exists only because a 2 Hz socket feed is reconstructed at 60 fps in a sandboxed browser. Keep the
browser viewer as-is for zero-install/shareable demos, but build the real tool natively.

### Mode 1 — LOCAL viewer (no DR)
- Native C# app **co-located with the engine** (same process or shared memory). Render the **authoritative**
  `Snapshot` every frame at 60 fps. **No transport, no dead-reckoning, no interpolation, no jitter** — the
  entire problem class disappears.
- Renderer options: **raylib-cs** (least ceremony), **Silk.NET** (OpenGL/Vulkan, most control), or MonoGame.
  Use GPU instancing for vehicle quads to reach 10k.
- Re-implements what the HTML client does: camera (pan/zoom/pick), road/lane draw, oriented bodies, TL,
  chord/off-tracking pose, HUD. Scope ~ a few days.

### Mode 2 — REMOTE viewer (with DR) over DDS
- Native app as a **DDS subscriber** (`CycloneDDS.NET`, already used by `Sim.Replication`; 0.3.2 runs on
  Linux + Windows). The viewer becomes just another participant — no bespoke server endpoint.
- **Pace the render clock off the DDS sample `source_timestamp`, not arrival time** — this is exactly the
  fix proven here; DDS gives the authoritative timestamp natively (WebSocket made us hand-roll it via
  `m.time`). Then apply **playout-delay + interpolation + monotonic clock** (port the logic from
  `HtmlPage.cs` — it transfers directly). Keep extrapolation only for genuinely sparse/adaptive streams.
- QoS to lean on: RELIABLE + KEEP_LAST history, DEADLINE, TIME_BASED_FILTER, LATENCY_BUDGET; multicast
  fan-out to many viewers. Validate CycloneDDS.NET 0.3.x QoS coverage/stability on target platforms first.
- Browsers can't speak RTPS, so DDS transport implies the native client (or a DDS→WS gateway, which just
  re-adds the hop — not recommended).

### What carries over
The interpolation-with-playout-delay + monotonic-clock pacing built in `HtmlPage.cs` is the reusable core
for Mode 2; the pose/off-tracking math (`PoseResolver.cs`, `LaneGeometry.cs`, and the JS mirror) ports to
either renderer. This session's work is not wasted by the stack change.

---

## 4. Suggested next steps (VM session)
1. Scaffold Mode 1: raylib-cs (or Silk.NET) window that opens a scenario, reads `SimulationRunner.Snapshot`
   each frame, draws roads + authoritative vehicle poses. Target 10k via instancing. No DR.
2. Prototype Mode 2: a `CycloneDDS.NET` subscriber to the `Sim.Replication` topic(s); pace off
   `source_timestamp`; port the delay/interp/monotonic-clock logic.
3. Keep `Sim.LiveHost` (browser) as the shareable demo; optionally address finding A (stream angle) if the
   demo needs true default-vs-corner comparison.

## 5. Files
- Changed this session: `src/Sim.LiveHost/HtmlPage.cs` (DR clock/interp/extrap/overlay), and earlier
  `docs/SUMOSHARP-LIVEVIZ-TESTING.md` (findings A/B, commit `5a39f16`).
- Diagnostic scripts (WebSocket timing probe, CDP frame-timing/clock probe) were run from the Windows
  scratchpad (ephemeral, not committed); the method is described in §2 if it needs reproducing.
