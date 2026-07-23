# Issue #15 residual — ATTEMPT LOG (running journal)

Purpose: a dated, honest journal of what was tried for the live-city junction-jam residual, what the
evidence said, what worked, and **what failed and why** — so a future session does not repeat a dead
end. Append newest entries at the bottom. This is the scratch/decisions trail; the polished diagnosis
lives in `LIVE-CITY-15-RESIDUAL-FINDINGS.md`, the repro in `LIVE-CITY-15-RESIDUAL-REPRO.md`.

Branch: `claude/livecity-15-turnlane-segregation` (off the merged live-city tip incl. `184fb31`).
Iron rule throughout: `Sim.ParityTests` stays **657/4** byte-identical (or a change is gated behind an
explicit fast-mode flag); bench determinism (serial == parallel) holds.

---

## 2026-07-23 — Repro + engine merge (context, already landed on the live-city line)
- Built the headless `LIVECITY-GRIDLOCK` probe (`--mode live-city --smoke`); reproduced the pre-merge
  **terminal** gridlock (stoppedFrac → 0.94, arrivals 38/200 s).
- Merged `claude/dense-lane-overlap-fix-5tr4ha` wholesale → terminal-lock became **jam-and-recover**
  (arrivals 81, stoppedFrac oscillates 0.38–0.73). Parity 654→657/4, bench hash unchanged. **Helped,
  not cured** (GPU verdict: jams still look unrealistic).

## 2026-07-23 — Witness: ground-truth on the residual stalls
- Added `LiveCitySim.WitnessAuthoritative()` + `LIVECITY_WITNESS=1` dump (engine-authoritative per-car
  lane/pos/posLat/speed/TL/gap). Read-only, host-side.
- **Findings that reframed the problem:**
  - `posLat = 0` for every stuck car → the GPU "lateral float" is a **render/DR artifact**, not engine
    motion. Do NOT chase a lateral-jockey bug in the engine.
  - Most stopped cars are at **red** or **behind a stopped leader** (consequence of a jam). The anomaly
    is `stuckOnGreenClear` (7–16 cars): speed≈0, TL green (incl. protected `G`), no leader within 15 m,
    at pos≈226–233 (the stop line). These block the queues behind them.
- Code + `/sumo/` analysis (subagent): getBestLanes offsets are CORRECT and keep-right rule 2 is landed
  → **"wrong turn-lane selection" hypothesis FALSIFIED**. Root localized to **missing LCA_URGENT
  strategic-change cooperation**: `TryStrategicLaneChange` (`Engine.cs:11048-11053`) does a bare
  `return false` when the turn lane is blocked; no ego brake-to-wait, no follower gap-opening. Turner
  never merges → strands at stop line → `Speed=0` clamp (`Engine.cs:9587-9611`).

## OPEN QUESTION raised by owner (2026-07-23) — MUST verify before designing
**"How does telling the follower to slow solve a car already stopped on green at a clear junction?"**
Valid skepticism. The link is INDIRECT (see the explanation section below): cooperation is **upstream
prevention**, not stop-line release. It stops turners from *arriving* stranded in the wrong lane; it does
NOT release a car already stranded.
- **Therefore, before committing to option (1) as the fix, verify the causal chain per stuck car:** does
  each `stuckOnGreenClear` car sit in a lane whose connections do NOT include its next route edge
  (= wrong-lane strand, which cooperation prevents), or does its lane DO connect but it is blocked for
  another reason (junction exit occupied / keep-clear / RoW)? Plan: extend the witness to print, per
  stuck car, whether `currentLane` has a connection to the car's next route edge, and the occupancy of
  the intended next (across-junction) lane. If most are NOT wrong-lane strands, option (1) is the wrong
  lever and we re-target. **Do not design the cooperation port until this split is measured.**

## Attempts that FAILED / were abandoned (avoid repeating)
- **Standalone `tools/livecity-repro/` console harness** — abandoned: the built-in `--mode live-city
  --smoke` path already runs the identical LiveCitySim loop; a separate project just duplicated it.
  Use `--smoke` (+ `LIVECITY_WITNESS=1`), not a new csproj.
- **Component-2 (LaneQ `occupation`/`maxJam`) on the dense-flow branch** — designed (`ad8d738`),
  implemented, measured **low-ceiling (~15% recovery) and entangled with the merge-in lever, then
  REVERTED** (`965fc45`). Do NOT re-attempt occupation-based urgency before the cooperation/merge-in
  lever; it was already shown ineffective in isolation.
- **"Turn-lane mis-segregation via getBestLanes"** as the root — FALSIFIED (offsets correct, rule 2
  landed). Don't re-open the getBestLanes offset port.

## Current plan (pending the OPEN QUESTION verification)
Option (1): port SUMO LCA_URGENT strategic cooperation (ego brake-to-wait + follower gap-opening) so
turners merge into the turn lane upstream. Design-first (HIGH parity risk — most golden-dense area).
Mechanism-gathering pass (SUMO formulas + our phase sequencing + parity surface) in progress.
