# DR motion jitter (frequent slowdowns/speedups on straight roads) — investigation plan

**Status:** planned investigation (NOT started). Design-first, data-driven — **no fixing before the
cause is localized from recorded data** (the standing rule; see `CLAUDE.md` and the pattern used in
the junction / lane-change passes). **Companion:** `SUMOSHARP-VIEWER-DR-SMOOTHING.md` (the DR pipeline
of record — read §5.1 DrClock, §5.2 Resolve, §5.4 auto-delay, §6.3 delay volatility). **Reported by
the user, 2026-07-15.**

---

## 1. Symptom (as reported)

In the DR viewers (loopback/remote), vehicles moving in a **straight line at roughly constant speed**
show **frequent slowdowns and speedups** — the longitudinal motion is "far from smooth." The user
also observed that the **automatic DR playout delay changes often** and suspects the two are linked.

This is a *longitudinal velocity* smoothness problem (speed fluctuating frame-to-frame), distinct from
the lateral jumps already fixed (junction turns, lane-change slide) and distinct from the lane-change
*heading* jump queued as T4 in `SUMOSHARP-LANE-CHANGE-SMOOTHING-TASKS.md`.

## 2. Why the auto-delay is a prime suspect (mechanism recap)

The render sample instant is `sampleT = renderSim − delay` (`DrClock`/`PumpAndBuildVehicleDraws`).
`renderSim` advances smoothly (rate-fit wall→sim clock). **Any change in `delay` moves `sampleT`**, so
the rendered position jumps back/forward along the trajectory → an apparent velocity glitch. We already
hit this during the junction pass (§6.3 of the pipeline doc): a per-frame EMA recompute teleported
vehicles ~4 m. The current mitigation (in both the loopback and remote loops):

```
target = clamp(AvgSampleInterval * 1.5, 0.3, 2.0)
if !seeded && AvgSampleInterval > 0:  delay = target; seeded = true      // snap once at startup
else if |target − delay| > 0.05:      delay += clamp(target − delay, ±0.008)   // slew, dead-banded
```

If the user still sees frequent delay changes, the likely gaps are:
- `AvgSampleInterval` is a **jittery EMA** (α=0.2 of the wall-clock packet inter-arrival) that, with the
  **adaptive publish cadence** (fast = every step while accel/decel, slow = every 3rd step steady),
  swings enough that `target` repeatedly crosses the 0.05 s dead-band → the slew runs often.
- Even the slew (±0.008 s/frame) is a *continuous* `sampleT` perturbation: at 14 m/s it is ~0.11 m/s of
  induced velocity error whenever it is active — small, but possibly the "far from smooth" the user sees
  if it is active much of the time.

## 3. Competing hypotheses (the investigation must DISTINGUISH these, not assume A)

| # | Cause | Predicted signature in data |
|---|---|---|
| **A** | **Auto-delay changing** shifts `sampleT` | Velocity glitches correlate 1:1 with frames where `delay` changed (Δdelay ≠ 0) |
| **B** | **`renderSim` non-uniform advance** — forward-only hold + capped catch-up (`3·frameDt·simRate`) in `DrClock.Pump` | Glitches correlate with `renderSim` catch-up/hold events, independent of delay; `BackSteps` climbing |
| **C** | **Piecewise-linear interpolation** — `Resolve` lerps arc linearly between bracketing packets, so rendered velocity is the *average* packet velocity, stepping at each packet boundary | Velocity is piecewise-constant; steps land exactly on packet-arrival (`LatestVehicleSampleTime` change) boundaries; step size ∝ real accel |
| **D** | **Adaptive publish cadence switching** (fast↔slow) changes the interpolation segment length | Glitches at cadence-switch moments (gap goes 1-step↔3-step) |
| **E** | **Render frame-dt jitter** (no vsync / GC) misread as velocity | Glitches correlate with frame-dt spikes; vanish when normalized by `rsim` Δt |
| **F** | **Extrapolation⇄interpolation flapping** — `sampleT` hovering near the newest packet toggles `extrap` | Glitches at `extrap` True↔False toggles |

More than one may contribute. The point of the data pass is to attribute the *dominant* one before
touching code.

## 4. Data-driven investigation procedure

### 4.1 Scenario & vehicle
Use a **steady, straight, ~constant-speed** case so any velocity variation is a viewer artifact, not
real dynamics:
- Primary: `scenarios/01-single-free-flow` (one vehicle, free flow) — cleanest.
- Secondary: `scenarios/06-two-lane-cruise` and a longer look at `10-truck-free-flow`.
Trace the moving vehicle (`--trace-veh <id>`; find the id in the scenario's `*.rou.xml`).

### 4.2 Instrument (temporary, revert after — like the junction pass)
The `DRTRACE` line already logs `rsim, x, y, deg, laneH, pos, extrap, spd, delay, avgint`. Add, for this
pass only:
- a **packet-arrival marker**: whether `subscriber.LatestVehicleSampleTime` changed this frame (new
  packet) — reveals interpolation-segment boundaries (hypothesis C/D).
- `DrClock.BackSteps` per frame (hypothesis B).
Keep the `--trace-veh` harness; only add these two fields.

### 4.3 Capture
```
dotnet build src/Sim.Viewer/Sim.Viewer.csproj -c Release
dotnet run -c Release --no-build --project src/Sim.Viewer -- --mode loopback \
    --trace-veh <id> --screenshot <scratch>/jit.png --frames 3000 scenarios/01-single-free-flow \
    2>&1 | grep DRTRACE > <scratch>/jit.log
```
Run **long** (3000 frames) so periodic glitches are visible. Note: headless has no vsync, so frame-dt is
erratic — **normalize by `rsim` Δt** (see §4.4), and cross-check with an INTERACTIVE run (vsync-capped)
since the user's report is from the interactive viewer.

### 4.4 Analyze (Python, as in the junction pass)
Per frame compute **longitudinal render velocity** `v = Δ(along-heading position)/Δrsim`. Then:
1. **Velocity trace & spikes:** plot/inspect `v` vs `rsim`; flag frames where `|v − median(v)|` exceeds,
   say, 15 % of the true cruise speed. Report count, magnitude, periodicity.
2. **Attribution — correlate each flagged glitch with:**
   - `Δdelay ≠ 0` this frame → **A**.
   - a `renderSim` catch-up (Δrsim ≠ expected `simRate·frameDt`) or a `BackSteps` increment → **B**.
   - a packet-arrival boundary (`LatestVehicleSampleTime` changed) → **C/D**.
   - `extrap` toggled → **F**.
   - a frame-dt spike that disappears after Δrsim normalization → **E** (not a real glitch).
3. **Delay-change frequency:** histogram how often `delay` changes and by how much; how long the slew is
   active vs idle; how often `target` crosses the dead-band. Confirms/denies the user's observation.
4. Rank causes by share of the flagged glitches.

## 5. Candidate fixes (evaluate AFTER attribution — listed with tradeoffs, do not pre-commit)

Matched to whichever hypothesis dominates:

- **A (delay changes):**
  - **A1 — freeze delay after seeding**, re-seed only on restart (Generation bump). Simplest; kills all
    steady-state delay motion. Cost: no mid-run adaptation if publish cadence shifts (probably fine — the
    seed already covers the worst gap).
  - **A2 — `renderSim` compensation:** when `delay` changes by Δ, advance `renderSim` by Δ so `sampleT`
    stays continuous — makes delay changes invisible instead of suppressing them. Most elegant; keeps
    adaptivity. Verify it can't break `renderSim` monotonicity.
  - **A3 — steadier interval estimate:** replace the α=0.2 EMA with a median (or max) of the last N gaps,
    and widen the dead-band / slow the slew. Reduces target jitter at the source.
- **B (`renderSim` advance):** review the catch-up cap (`3·frameDt·simRate`) and the forward-only hold;
  consider a gentler catch-up so speed doesn't visibly pulse. Check `BackSteps` isn't ticking in steady
  state.
- **C (piecewise-linear interpolation):** upgrade `Resolve`'s arc interpolation from **linear** to
  **acceleration-aware** (Hermite/quadratic using each packet's `Speed` + `Accel`), so velocity is
  continuous across packet boundaries. Bigger change, viewer-only; the highest-fidelity fix if C
  dominates. Keep behind the existing realism/quality path; no parity impact.
- **D (cadence switching):** smooth the interpolation across a cadence change, or (publisher side) reduce
  the fast/slow ratio — but that trades bandwidth, so prefer the viewer-side C fix.
- **E (frame-dt):** not a real defect; confirm it vanishes under vsync (interactive) and Δrsim
  normalization — if so, no code change.
- **F (extrap flapping):** ensure the auto-delay keeps `sampleT` comfortably behind the newest packet
  (margin ≥ max recent gap) so steady cruise never flaps into extrapolation.

## 6. Success criteria (numeric)

On `01-single-free-flow` at constant cruise, from the DRTRACE:
- Per-frame render velocity stays within **±5 %** of the true cruise speed (no periodic dips/pulses).
- **No** correlation remaining between velocity glitches and `delay`-change events (hypothesis A closed).
- `delay` is effectively constant in steady state (changes only on restart / genuine cadence regime
  change), OR delay changes are provably invisible (A2 compensation).
- Interactive viewer: user confirms the straight-line motion reads as smooth.

## 7. Files / guardrails

- Likely touched (viewer-only, all **out** of `Traffic.sln`): `src/Sim.Viewer/Program.cs` (auto-delay
  block, both loopback + remote sites), `src/Sim.Viewer.Core/DrClock.cs` (`Pump` advance, `Resolve`
  interpolation, `AvgSampleInterval`).
- **Do not** touch `Sim.Core`/`Sim.Ingest`/`Traffic.sln`. Zero parity impact expected (render-only).
- Keep the `--trace-veh` harness; revert only the two temporary added fields (§4.2) after the pass.
- Branch `claude/native-viewer-perf-gpu`; don't push to main.

## 8. Suggested delegation (per the user's 2026-07-15 note)

The **data-capture + attribution** (§4) is well-scoped and Sonnet-suitable: give it the scenarios, the
two instrumentation fields to add, the capture command, and the analysis (velocity from Δpos/Δrsim +
the correlation table in §3). Reserve Opus judgment for **choosing the fix** among §5 (especially the
A2 vs C call, which trades elegance vs fidelity) and the final review. Sequence: (1) Sonnet investigates
→ attribution report; (2) Opus picks the fix + writes it up; (3) implement (Sonnet if mechanical) →
(4) Opus verifies against §6 + interactive sign-off.
